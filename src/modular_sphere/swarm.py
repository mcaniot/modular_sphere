"""Multi-agent 3D task: a swarm of attachable spheres climbs a step that no sphere can climb alone.

Each sphere is the sphere of :mod:`modular_sphere.env` (same radius, mass, damping, friction, same
world-frame x, y force in [-1, 1] N) with two extra modular capabilities:

- **Attachment** (ball-joint like): two spheres whose centres are within ``2 r + ATTACH_TOL``
  link when both request it, and stay linked while both keep requesting it. A link fixes the
  distance between the two centres (MuJoCo tendon equality on a spatial tendon between the
  centres), so each sphere can orbit freely around the other, like a magnet rolling on a shell.
  Active links are drawn as a line between the centres.
- **Lift**: an internal vertical force of up to ``MAX_LIFT`` N, applied to the sphere and,
  with the opposite sign split equally, to its linked partners (a lone sphere cannot lift itself).
  ``MAX_LIFT`` exceeds the sphere weight (2.94 N), so a sphere can pivot over a linked partner.

Task (ant-like collective climb): the arena floor rises by ``STEP_H`` (> sphere radius, a 1 N
push cannot climb it) at ``x = STEP_X``; the goal is a gathering zone on the platform (centre at
sphere-centre height, radius ``gather_radius(N)``). The team succeeds when **every** sphere is on
the platform inside the zone, so the swarm must also bring up its last member, which needs a
partner already on the platform. The arena walls are raised to ``WALL_H`` so that spheres on the
platform cannot roll out. Rewards are shared: progress of the mean sphere-goal 3D distance, a
step penalty and a success bonus.

API: PettingZoo ``ParallelEnv`` (agents ``sphere_0..sphere_{N-1}``), action per agent
``[fx, fy, lift, attach]`` in [-1, 1] (``attach > 0`` requests a link; agents missing from the
action dict get a zero action). Local observations are
world frame, see ``obs_slices``; ``state()`` gives the global state for centralised critics.
Physics timestep, ``FRAME_SKIP`` and the sphere parameters are those of the single-sphere contract.

Sources checked: MuJoCo 4 equality constraints (``connect`` pins a point of one shell to the
other centre, so the shell has to spin to orbit; a tendon equality on a centre-to-centre spatial
tendon is a pure distance constraint, ``eq_data[0]`` = length - ``tendon_length0``), toggled with
``MjData.eq_active``; ``MjSpec.delete``/``add_tendon``/``add_equality`` to build the swarm from
``sphere_arena.xml``; PettingZoo 1.27 ``ParallelEnv`` and ``parallel_api_test``.
"""

from __future__ import annotations

from itertools import combinations
from typing import Any, ClassVar

import mujoco
import numpy as np
from gymnasium import spaces
from numpy.typing import NDArray
from pettingzoo import ParallelEnv

from modular_sphere.env import FRAME_SKIP, MODEL_PATH, STEP_PENALTY, SUCCESS_BONUS

N_SPHERES = 4
STEPS_PER_SPHERE = 150  # default episode length is STEPS_PER_SPHERE * N (7.5 s per sphere)
STEP_X = 0.5  # step face [m]
STEP_H = 0.15  # step height [m], between radius (0.10) and diameter (0.20)
WALL_H = 0.5  # arena wall height [m], above a sphere on the platform
MAX_LIFT = 6.0  # internal vertical force [N], about twice the sphere weight
ATTACH_TOL = 0.02  # max gap between surfaces to form a link [m]
START_ZONE = (-1.6, -0.4, -1.2, 1.2)  # x_min, x_max, y_min, y_max of start centres
GOAL_ZONE = (1.1, 1.5, -1.0, 1.0)  # gathering zone centre bounds, the zone stays on the platform
MIN_START_GAP = 0.1  # min distance between start surfaces [m]
ACT_DIM = 4  # fx, fy, lift, attach
_LINK_RGBA = (0.1, 0.9, 0.9, 1.0)
_HIDDEN_GROUP = 5


def gather_radius(n_spheres: int) -> float:
    """Radius [m] of the gathering zone: the N spheres fit in it with room to spare."""
    return 0.2 * float(np.sqrt(n_spheres))


def obs_slices(n_spheres: int) -> dict[str, slice]:
    """Local observation layout of one agent; ``others`` lists the other spheres in cyclic order
    after the agent (``i+1, ..., N-1, 0, ..., i-1``), 7 values each: relative position (3),
    relative velocity (3), linked flag (1)."""
    return {
        "pos": slice(0, 3),  # sphere centre [m], world frame
        "vel": slice(3, 6),  # linear velocity [m/s]
        "ang_vel": slice(6, 9),  # angular velocity [rad/s], world frame
        "goal_rel": slice(9, 12),  # goal - centre [m]
        "others": slice(12, 12 + 7 * (n_spheres - 1)),
    }


class SphereSwarmStepEnv(ParallelEnv):
    """Swarm of ``n_spheres`` modular spheres that must all gather on a raised goal zone."""

    metadata: ClassVar[dict[str, Any]] = {
        "name": "modular_sphere_swarm_step_v0",
        "render_modes": ["rgb_array"],
        "render_fps": 20,
        "is_parallelizable": True,
    }

    def __init__(
        self,
        n_spheres: int = N_SPHERES,
        max_steps: int | None = None,
        render_mode: str | None = None,
        width: int = 640,
        height: int = 480,
        camera_name: str = "overview",
        step_height: float = STEP_H,
    ) -> None:
        if n_spheres < 2:
            raise ValueError(f"n_spheres must be >= 2, got {n_spheres}")
        if not 0 < step_height < WALL_H:
            raise ValueError(f"step_height must be in (0, {WALL_H}), got {step_height}")
        self.step_height = step_height  # lower steps: curricula for learning (default STEP_H)
        if render_mode not in (None, *self.metadata["render_modes"]):
            raise ValueError(f"render_mode must be None or 'rgb_array', got {render_mode!r}")
        self.n, self.render_mode = n_spheres, render_mode
        self.max_steps = STEPS_PER_SPHERE * n_spheres if max_steps is None else max_steps
        self.gather_radius = gather_radius(n_spheres)
        self._size, self._camera = (width, height), camera_name
        self.possible_agents = [f"sphere_{i}" for i in range(n_spheres)]
        self.agents: list[str] = []
        self.model = self._build_model()
        self.data = mujoco.MjData(self.model)
        self.radius = float(self.model.geom("sphere_0").size[0])
        self._pairs = list(combinations(range(n_spheres), 2))
        self._goal_mocap = self.model.body("goal").mocapid[0]
        self._qpos = np.array([self.model.joint(f"slide_x_{i}").qposadr[0] for i in range(self.n)])
        self._qvel = np.array([self.model.joint(f"slide_x_{i}").dofadr[0] for i in range(self.n)])
        self.slices = obs_slices(n_spheres)
        obs_dim = self.slices["others"].stop
        self.observation_spaces = {
            a: spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float64)
            for a in self.possible_agents
        }
        self.action_spaces = {
            a: spaces.Box(-1.0, 1.0, shape=(ACT_DIM,), dtype=np.float32)
            for a in self.possible_agents
        }
        state_dim = 9 * n_spheres + 3 + len(self._pairs)
        self.state_space = spaces.Box(-np.inf, np.inf, shape=(state_dim,), dtype=np.float64)
        self._renderer: mujoco.Renderer | None = None
        self.np_random = np.random.default_rng()
        self._steps, self._prev_dist = 0, 0.0

    def _build_model(self) -> mujoco.MjModel:
        spec = mujoco.MjSpec.from_file(MODEL_PATH)
        proto, damping = spec.geom("sphere"), spec.joint("slide_x").damping
        for name in ("force_x", "force_y"):
            spec.delete(spec.actuator(name))
        spec.delete(spec.body("sphere"))
        spec.worldbody.add_geom(
            name="step",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=((STEP_X + 2.0) / 2, 0, self.step_height / 2),
            size=((2.0 - STEP_X) / 2, 2.0, self.step_height / 2),
            material="wall",
        )
        for wall in ("wall_north", "wall_south", "wall_east", "wall_west"):
            geom = spec.geom(wall)
            geom.size[2], geom.pos[2] = WALL_H / 2, WALL_H / 2
        zone = spec.body("goal").first_geom()  # flat translucent disc on the platform surface
        zone.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        zone.size = (gather_radius(self.n), 0.002, 0)
        zone.pos = (0, 0, -proto.size[0] + 0.002)
        zone.material = ""
        zone.rgba = (0.9, 0.1, 0.1, 0.45)
        for i in range(self.n):
            body = spec.worldbody.add_body(name=f"sphere_{i}")  # slide qpos = world centre
            for axis, vec in zip("xyz", np.eye(3), strict=True):
                body.add_joint(
                    name=f"slide_{axis}_{i}",
                    type=mujoco.mjtJoint.mjJNT_SLIDE,
                    axis=vec,
                    damping=damping if axis != "z" else 0.0,
                )
                act = spec.add_actuator(
                    name=f"force_{axis}_{i}",
                    target=f"slide_{axis}_{i}",
                    trntype=mujoco.mjtTrn.mjTRN_JOINT,
                )
                act.ctrllimited = mujoco.mjtLimited.mjLIMITED_FALSE  # clipping done in step()
            body.add_joint(name=f"roll_{i}", type=mujoco.mjtJoint.mjJNT_BALL)
            body.add_geom(
                name=f"sphere_{i}",
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=proto.size,
                mass=proto.mass,
                material=proto.material,
                condim=proto.condim,
                friction=proto.friction,
            )
            body.add_site(name=f"centre_{i}", size=(0.01, 0, 0))
        for i, j in combinations(range(self.n), 2):
            tendon = spec.add_tendon(name=f"link_{i}_{j}", width=0.015, rgba=_LINK_RGBA)
            tendon.wrap_site(f"centre_{i}")
            tendon.wrap_site(f"centre_{j}")
            spec.add_equality(
                name=f"link_{i}_{j}",
                type=mujoco.mjtEq.mjEQ_TENDON,
                objtype=mujoco.mjtObj.mjOBJ_TENDON,
                name1=f"link_{i}_{j}",
                active=False,
            )
        return spec.compile()

    # PettingZoo API -----------------------------------------------------------------------

    def observation_space(self, agent: str) -> spaces.Box:
        return self.observation_spaces[agent]

    def action_space(self, agent: str) -> spaces.Box:
        return self.action_spaces[agent]

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, NDArray[np.float64]], dict[str, dict[str, Any]]]:
        if seed is not None:
            self.np_random = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        self._sync_link_visibility()
        for i, xy in enumerate(self._sample_starts()):
            self.data.qpos[self._qpos[i] : self._qpos[i] + 3] = (*xy, self.radius)
        x0, x1, y0, y1 = GOAL_ZONE
        goal = (self.np_random.uniform(x0, x1), self.np_random.uniform(y0, y1))
        self.data.mocap_pos[self._goal_mocap] = (*goal, self.step_height + self.radius)
        mujoco.mj_forward(self.model, self.data)
        self.agents = self.possible_agents.copy()
        self._steps, self._prev_dist = 0, self._dist()
        return self._observations(), {a: {} for a in self.agents}

    def step(
        self, actions: dict[str, NDArray[np.floating]]
    ) -> tuple[
        dict[str, NDArray[np.float64]],
        dict[str, float],
        dict[str, bool],
        dict[str, bool],
        dict[str, dict[str, Any]],
    ]:
        if not self.agents:
            raise RuntimeError("step() called on a finished episode, call reset()")
        act = np.zeros((self.n, ACT_DIM))
        for i, agent in enumerate(self.possible_agents):
            if agent in actions:
                act[i] = np.clip(np.asarray(actions[agent], dtype=np.float64), -1.0, 1.0)
        self._update_links(act[:, 3] > 0.0)
        linked = self.links
        ctrl = np.zeros((self.n, 3))
        ctrl[:, :2] = act[:, :2]
        for i in range(self.n):
            partners = np.flatnonzero(linked[i])
            if partners.size:
                lift = MAX_LIFT * act[i, 2]
                ctrl[i, 2] += lift
                ctrl[partners, 2] -= lift / partners.size
        self.data.ctrl[:] = ctrl.ravel()
        mujoco.mj_step(self.model, self.data, nstep=FRAME_SKIP)
        self._steps += 1
        dist = self._dist()
        n_arrived = int(self.arrived.sum())
        success = n_arrived == self.n
        reward = (self._prev_dist - dist) - STEP_PENALTY + SUCCESS_BONUS * success
        self._prev_dist = dist
        truncated = self._steps >= self.max_steps and not success
        info = {
            "distance": dist,
            "n_arrived": n_arrived,
            "is_success": success,
            "n_links": int(linked.sum() // 2),
        }
        agents = self.agents
        obs = self._observations()
        if success or truncated:
            self.agents = []
        return (
            obs,
            {a: float(reward) for a in agents},
            {a: bool(success) for a in agents},
            {a: bool(truncated) for a in agents},
            {a: dict(info) for a in agents},
        )

    def state(self) -> NDArray[np.float64]:
        """Global state: per sphere pos, vel, ang_vel (9 N), goal (3), links (upper triangle)."""
        pos, vel, ang_vel = self._kinematics()
        per_sphere = np.concatenate([pos, vel, ang_vel], axis=1).ravel()
        links = self.data.eq_active.astype(np.float64)
        return np.concatenate([per_sphere, self.goal, links])

    def render(self) -> NDArray[np.uint8] | None:
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            width, height = self._size
            self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
            self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
        self._renderer.update_scene(self.data, self._camera)
        return self._renderer.render().copy()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    # Helpers ------------------------------------------------------------------------------

    @property
    def goal(self) -> NDArray[np.float64]:
        return self.data.mocap_pos[self._goal_mocap].copy()

    @property
    def links(self) -> NDArray[np.bool_]:
        """Symmetric (N, N) adjacency matrix of the active links."""
        linked = np.zeros((self.n, self.n), dtype=bool)
        for k, (i, j) in enumerate(self._pairs):
            linked[i, j] = linked[j, i] = bool(self.data.eq_active[k])
        return linked

    def _kinematics(self) -> tuple[NDArray[np.float64], ...]:
        pos = np.stack([self.data.qpos[a : a + 3] for a in self._qpos])
        vel = np.stack([self.data.qvel[a : a + 3] for a in self._qvel])
        ang_vel = np.zeros((self.n, 3))  # ball-joint velocity is in the body frame
        for i, (qa, va) in enumerate(zip(self._qpos, self._qvel, strict=True)):
            mujoco.mju_rotVecQuat(
                ang_vel[i], self.data.qvel[va + 3 : va + 6], self.data.qpos[qa + 3 : qa + 7]
            )
        return pos, vel, ang_vel

    def _observations(self) -> dict[str, NDArray[np.float64]]:
        pos, vel, ang_vel = self._kinematics()
        linked, goal = self.links, self.goal
        obs = {}
        for i, agent in enumerate(self.possible_agents):
            others = [(i + k) % self.n for k in range(1, self.n)]
            rel = np.concatenate(
                [pos[others] - pos[i], vel[others] - vel[i], linked[i, others, None]], axis=1
            )
            obs[agent] = np.concatenate([pos[i], vel[i], ang_vel[i], goal - pos[i], rel.ravel()])
        return obs

    @property
    def arrived(self) -> NDArray[np.bool_]:
        """Spheres on the platform (centre above the step) inside the gathering zone."""
        pos, goal = self._kinematics()[0], self.goal
        in_zone = np.linalg.norm(pos[:, :2] - goal[:2], axis=1) <= self.gather_radius
        return in_zone & (pos[:, 2] > self.step_height + self.radius / 2)

    def _dist(self) -> float:
        pos = np.stack([self.data.qpos[a : a + 3] for a in self._qpos])
        return float(np.linalg.norm(pos - self.goal, axis=1).mean())

    def _sample_starts(self) -> list[NDArray[np.float64]]:
        x0, x1, y0, y1 = START_ZONE
        starts: list[NDArray[np.float64]] = []
        while len(starts) < self.n:  # rejection sampling, the zone holds far more than N spheres
            xy = np.array([self.np_random.uniform(x0, x1), self.np_random.uniform(y0, y1)])
            if all(np.linalg.norm(xy - s) >= 2 * self.radius + MIN_START_GAP for s in starts):
                starts.append(xy)
        return starts

    def _update_links(self, request: NDArray[np.bool_]) -> None:
        pos = np.stack([self.data.qpos[a : a + 3] for a in self._qpos])
        for k, (i, j) in enumerate(self._pairs):
            want = bool(request[i] and request[j])
            if self.data.eq_active[k] and not want:
                self.data.eq_active[k] = False
            elif not self.data.eq_active[k] and want:
                dist = float(np.linalg.norm(pos[i] - pos[j]))
                if dist <= 2 * self.radius + ATTACH_TOL:
                    self.model.eq_data[k, 0] = dist - self.model.tendon_length0[k]
                    self.data.eq_active[k] = True
        self._sync_link_visibility()

    def _sync_link_visibility(self) -> None:
        # Inactive links go to tendon group 5, hidden by the default visualisation options.
        self.model.tendon_group[:] = np.where(self.data.eq_active, 0, _HIDDEN_GROUP)


class SwarmClimbPolicy:
    """Scripted, centralised ant-like solution: the whole swarm climbs one sphere at a time.

    The sphere closest to the step face becomes the base; the others are climbers, in decreasing
    x order. The base holds against the face in line with the goal. Each climber in turn lines up
    behind the base (the next one waits at a staging point), both request a link, the climber
    pushes into the base and lifts, orbiting over it, then releases on top and rolls to its slot
    in the gathering zone. The last climber stays at the platform edge as a helper: it links with
    the base, which lifts and pivots over the helper onto the platform. Everyone then gathers.
    """

    KV = 3.0
    BRAKE = 0.4
    VMAX = 0.8
    DAMPING = 0.69  # feed-forward of the slide damping
    ALIGN_TOL = 0.03
    TOP_MARGIN = 0.015
    STAGE = 0.3  # climber waits this far behind its slot until the base is in place [m]
    EDGE_X = STEP_X + 0.035  # helper centre on the platform, within link range of the base [m]
    SLOT_PITCH = 0.22  # hexagonal packing pitch of the gathering slots [m]

    def __init__(self, n_spheres: int = N_SPHERES, radius: float = 0.1) -> None:
        self.n, self.r = n_spheres, radius
        self.slices = obs_slices(n_spheres)
        self.reset()

    def reset(self) -> None:
        self._roles: tuple[str, list[str]] | None = None
        self._k = 0  # index of the active climber, len(climbers) during the final pull-up
        self._lifting = False
        self._up: set[str] = set()  # spheres released on the platform

    def _track(
        self, o: NDArray[np.float64], target: NDArray[np.float64], vmax: float | None = None
    ) -> NDArray[np.float64]:
        err = target - o[self.slices["pos"]][:2]
        dist = max(float(np.linalg.norm(err)), 1e-9)
        v_des = (vmax or self.VMAX) * err / dist * min(1.0, dist / self.BRAKE)
        return self.KV * (v_des - o[self.slices["vel"]][:2]) + self.DAMPING * v_des

    def _slots(self, goal: NDArray[np.float64]) -> list[NDArray[np.float64]]:
        """``n`` hexagonal slots around the goal, farthest from the step first."""
        cells = [(0.0, 0.0)]
        ring = 1
        while len(cells) < self.n:
            for side in range(6):
                a0, a1 = np.deg2rad(60 * side), np.deg2rad(60 * (side + 1))
                for s in range(ring):
                    t = s / ring
                    corner0 = ring * np.array([np.cos(a0), np.sin(a0)])
                    corner1 = ring * np.array([np.cos(a1), np.sin(a1)])
                    cells.append(tuple((1 - t) * corner0 + t * corner1))
            ring += 1
        slots = [goal[:2] + self.SLOT_PITCH * np.asarray(c) for c in cells[: self.n]]
        return sorted(slots, key=lambda s: -s[0])

    def _linked(self, o: NDArray[np.float64], i: int, j: int) -> bool:
        """Linked flag of sphere ``j`` in the observation ``o`` of sphere ``i``."""
        others = o[self.slices["others"]].reshape(self.n - 1, 7)
        return bool(others[(j - i) % self.n - 1, 6] > 0.5)

    def _lift(
        self, holder: NDArray[np.float64], lifter: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        fy = self.KV * (holder[1] - lifter[1]) - lifter[4]  # stay in line with the holder
        return np.array([0.3, fy, 1.0, 1.0])

    def act(self, obs: dict[str, NDArray[np.float64]]) -> dict[str, NDArray[np.float64]]:
        agents = sorted(obs, key=lambda a: int(a.rsplit("_", 1)[1]))
        o0 = obs[agents[0]]
        goal = o0[self.slices["pos"]] + o0[self.slices["goal_rel"]]
        base_xy = np.array([STEP_X - self.r - 0.01, goal[1]])
        climb_xy = base_xy - (2 * self.r + 0.03, 0.0)
        edge_xy = np.array([self.EDGE_X, goal[1]])
        if self._roles is None:
            order = [agents[i] for i in np.argsort([-obs[a][0] for a in agents], kind="stable")]
            self._roles = (order[0], order[1:])
        base, climbers = self._roles
        slots = dict(zip([*climbers, base], self._slots(goal), strict=True))
        ob = obs[base]
        act = {a: np.zeros(ACT_DIM) for a in agents}
        for a in self._up:
            act[a][:2] = self._track(obs[a], slots[a])
        if self._k < len(climbers):  # a climber goes over the base
            climber = climbers[self._k]
            oc = obs[climber]
            if not self._lifting:
                base_ready = np.linalg.norm(ob[:2] - base_xy) < self.ALIGN_TOL
                slot = climb_xy if base_ready else climb_xy - (self.STAGE, 0.0)
                act[base][:2] = self._track(ob, base_xy)
                act[climber][:2] = self._track(oc, slot)
                ready = base_ready and np.linalg.norm(oc[:2] - climb_xy) < self.ALIGN_TOL
                speed = max(np.linalg.norm(ob[3:5]), np.linalg.norm(oc[3:5]))
                self._lifting = bool(ready and speed < 0.1)
            if self._lifting:
                act[base][:] = (1.0, 0.0, 0.0, 1.0)
                act[climber][:] = self._lift(ob, oc)
                if oc[2] > 3 * self.r - self.TOP_MARGIN:
                    self._up.add(climber)
                    self._k, self._lifting = self._k + 1, False
        elif base not in self._up:  # the last climber, at the edge, pulls the base up
            helper = climbers[-1]
            oh = obs[helper]
            if not self._lifting:
                act[base][:2] = self._track(ob, base_xy)
                act[helper][:2] = self._track(oh, edge_xy, vmax=0.3)
                ready = (
                    np.linalg.norm(ob[:2] - base_xy) < self.ALIGN_TOL
                    and np.linalg.norm(oh[:2] - edge_xy) < self.ALIGN_TOL
                )
                speed = max(np.linalg.norm(ob[3:5]), np.linalg.norm(oh[3:5]))
                self._lifting = bool(ready and speed < 0.1)
            if self._lifting and not self._linked(oh, agents.index(helper), agents.index(base)):
                act[helper][:2] = self._track(oh, edge_xy, vmax=0.3)  # hold the edge, link first
                act[base][:] = (1.0, 0.0, 0.0, 0.0)
                act[helper][3] = act[base][3] = 1.0
            elif self._lifting:
                act[helper][:] = (1.0, 0.0, 0.0, 1.0)
                act[base][:] = self._lift(oh, ob)
                if ob[2] > oh[2] + self.r and ob[0] > oh[0] - 0.05:
                    self._up.add(base)
        return {a: np.clip(v, -1.0, 1.0) for a, v in act.items()}


def parallel_env(**kwargs: Any) -> SphereSwarmStepEnv:
    """PettingZoo-style factory."""
    return SphereSwarmStepEnv(**kwargs)
