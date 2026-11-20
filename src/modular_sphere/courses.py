"""Obstacle-course tasks for the sphere: reach the goal through walls, a tunnel, a bridge, a gate.

Levels (monotone in the number of mandatory skills and in path length), all in the 4 m x 4 m
arena of ``sphere_arena.xml``, goal drawn uniformly in the level's final zone:

1. baffle: one wall across the arena, route around it.
2. tunnel: the only opening of a dividing wall is a 0.4 m wide, 0.26 m high tunnel (the 0.2 m
   sphere fits, the translucent ceiling is visible from the top camera).
3. bridge: a 0.13 m high wall that cannot be climbed (sphere radius is 0.10 m, so center at 0.10 m
   cannot clear 0.13 m) is crossed by a 0.4 m wide deck 0.18 m high, reached by 10 degree ramps
   (the 1 N motor climbs at most ~20 degrees statically; ~0.5 N is spent on gravity at 10 degrees,
   so slow down). Falling off the deck is recoverable and not penalised: the sphere lands on the
   floor beside the bridge (a 0.18 m drop) and has to drive back to the ramp.
4. gate: a pressure plate on the start side opens a sliding door, the only opening in the wall.
   The plate latches (the sphere cannot hold the plate and pass the door at once); the door is a
   MuJoCo slide joint driven by a position actuator whose target the env sets from the latch.
5. combination: plate -> door -> bridge over a low wall -> tunnel -> goal.

Observation: the 11 D base observation of :mod:`modular_sphere.env` plus ``button`` and ``door``
(see ``OBS_SLICES``); both are 0 on levels without a gate. Action space, dynamics, reward and
success criterion are those of :class:`SphereReachEnv` (reward is distance-to-goal shaped and does
not know about walls: it is a local, not a global, shaping). Episodes are longer than the base
200 steps on the harder levels (``Level.max_steps``).

Sources checked (no existing task-variant library fits): ``mujoco.MjSpec`` (4.0.1, installed
stubs/docstrings) edits the base arena procedurally (``add_geom``, ``add_body``, ``add_joint``,
``add_actuator(...).set_to_position``), so no XML is copied; Gymnasium's ``MujocoEnv`` exposes
``_initialize_simulation`` which is the hook used to compile the edited spec; Gymnasium-Robotics
mazes, dm_control and MuJoCo Playground are not installed and have no ramps/doors for this robot.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np
from gymnasium import spaces
from numpy.typing import NDArray

from modular_sphere.env import MODEL_PATH, SphereReachEnv
from modular_sphere.env import OBS_SLICES as BASE_OBS_SLICES

OBS_SLICES: dict[str, slice] = {
    **BASE_OBS_SLICES,
    "button": slice(11, 12),  # 1.0 once the pressure plate has been pressed (latched), else 0.0
    "door": slice(12, 13),  # door opening in [0, 1] (0 closed, 1 fully open)
}
OBS_DIM = 13

START_JITTER = 0.05  # uniform start position noise [m]
WALL_H = 0.2
WALL_T = 0.05  # half thickness
LOW_WALL_H = 0.13  # unclimbable but lower than the deck underside (0.15)
RAMP_DEG = 10.0
BRIDGE_H = 0.18
BRIDGE_W = 0.4
DECK_T = 0.03
TUNNEL_W = 0.4  # inner width
TUNNEL_H = 0.26  # clearance under the ceiling
DOOR_TRAVEL = 0.7
DOOR_KP = 200.0
BUTTON_RADIUS = 0.15  # sphere centre must be this close to the plate centre
_BUTTON_OFF = (0.95, 0.8, 0.1, 1.0)
_BUTTON_ON = (0.2, 0.85, 0.3, 1.0)

Waypoint = tuple[float, float, float, float]  # x, y, switch tolerance [m], max speed [m/s]


def _box(spec: mujoco.MjSpec, name: str, lo: Any, hi: Any, **kwargs: Any) -> None:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    spec.worldbody.add_geom(
        name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=(lo + hi) / 2, size=(hi - lo) / 2, **kwargs
    )


def _wall_x(spec: mujoco.MjSpec, name: str, x: float, y0: float, y1: float, h: float = WALL_H):
    _box(spec, name, (x - WALL_T, y0, 0), (x + WALL_T, y1, h), material="wall")


def _wall_y(spec: mujoco.MjSpec, name: str, y: float, x0: float, x1: float, h: float = WALL_H):
    _box(spec, name, (x0, y - WALL_T, 0), (x1, y + WALL_T, h), material="wall")


def _tunnel(spec: mujoco.MjSpec, name: str, x0: float, x1: float, yc: float) -> None:
    """Tunnel along x with the dividing walls left to the caller."""
    hw = TUNNEL_W / 2
    top = TUNNEL_H + 0.04
    for sign, side in ((1, "l"), (-1, "r")):
        y_in, y_out = yc + sign * hw, yc + sign * (hw + 0.05)
        _box(
            spec,
            f"{name}_{side}",
            (x0, min(y_in, y_out), 0),
            (x1, max(y_in, y_out), top),
            material="wall",
        )
    _box(
        spec,
        f"{name}_roof",
        (x0, yc - hw - 0.05, TUNNEL_H),
        (x1, yc + hw + 0.05, top),
        rgba=(0.9, 0.55, 0.2, 0.35),
    )


def _ramp(spec: mujoco.MjSpec, name: str, axis: int, c: float, foot: float, sign: int) -> float:
    """Slab rising from the floor at ``foot`` to BRIDGE_H along ``sign * axis``; returns the top."""
    th = np.deg2rad(RAMP_DEG)
    length, run, thick = BRIDGE_H / np.sin(th), BRIDGE_H / np.tan(th), 0.3
    centre_s = foot + sign * run / 2 + sign * np.sin(th) * thick / 2  # mid-slope - normal * t/2
    centre_z = BRIDGE_H / 2 - np.cos(th) * thick / 2
    pos = (centre_s, c, centre_z) if axis == 0 else (c, centre_s, centre_z)
    size = (
        (length / 2, BRIDGE_W / 2, thick / 2)
        if axis == 0
        else (BRIDGE_W / 2, length / 2, thick / 2)
    )
    euler = (0, -sign * th, 0) if axis == 0 else (sign * th, 0, 0)
    spec.worldbody.add_geom(
        name=name,
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=pos,
        size=size,
        euler=euler,
        rgba=(0.7, 0.5, 0.3, 1),
    )
    return foot + sign * run


def _bridge(spec: mujoco.MjSpec, axis: int, c: float, foot_a: float, foot_b: float) -> None:
    """Ramp up from ``foot_a``, narrow deck, ramp down to ``foot_b`` (axis 0 = x, 1 = y)."""
    top_a = _ramp(spec, "ramp_up", axis, c, foot_a, +1)
    top_b = _ramp(spec, "ramp_down", axis, c, foot_b, -1)
    lo, hi = [0.0, 0.0, BRIDGE_H - DECK_T], [0.0, 0.0, BRIDGE_H]
    lo[axis], hi[axis] = top_a, top_b
    lo[1 - axis], hi[1 - axis] = c - BRIDGE_W / 2, c + BRIDGE_W / 2
    _box(spec, "deck", lo, hi, rgba=(0.85, 0.7, 0.35, 1))


def _gate(spec: mujoco.MjSpec, wall_x: float, yc: float, button: tuple[float, float]) -> None:
    """Sliding door on the +x face of a wall (gap y in yc +- 0.3), opening towards +y."""
    body = spec.worldbody.add_body(name="door", pos=(wall_x + 0.09, yc, WALL_H / 2))
    body.add_joint(
        name="door_slide",
        type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=(0, 1, 0),
        range=(0, DOOR_TRAVEL),
        damping=20,
    )
    body.add_geom(
        name="door",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(0.035, 0.33, WALL_H / 2),
        mass=1,
        rgba=(0.95, 0.6, 0.1, 1),
    )
    act = spec.add_actuator(
        name="door_drive", target="door_slide", trntype=mujoco.mjtTrn.mjTRN_JOINT
    )
    act.set_to_position(kp=DOOR_KP)
    act.ctrlrange = (0, DOOR_TRAVEL)
    spec.worldbody.add_geom(
        name="button",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=(BUTTON_RADIUS + 0.03, 0.005, 0),
        pos=(*button, 0.005),
        contype=0,
        conaffinity=0,
        rgba=_BUTTON_OFF,
    )


def _l1(spec: mujoco.MjSpec) -> None:
    _wall_x(spec, "baffle", 0.0, -2.0, 0.9)


def _l2(spec: mujoco.MjSpec) -> None:
    _wall_x(spec, "div_a", 0.0, -2.0, -TUNNEL_W / 2)
    _wall_x(spec, "div_b", 0.0, TUNNEL_W / 2, 2.0)
    _tunnel(spec, "tunnel", -0.6, 0.6, 0.0)


def _l3(spec: mujoco.MjSpec) -> None:
    _wall_x(spec, "div", 0.0, -2.0, 2.0, LOW_WALL_H)
    _bridge(spec, 0, 0.0, -1.4, 1.4)


def _l4(spec: mujoco.MjSpec) -> None:
    _wall_x(spec, "div_a", 0.0, -2.0, -1.5)
    _wall_x(spec, "div_b", 0.0, -0.9, 2.0)
    _gate(spec, 0.0, -1.2, BUTTON_L4)


def _l5(spec: mujoco.MjSpec) -> None:
    _wall_x(spec, "div1_a", -0.8, -2.0, -1.7)
    _wall_x(spec, "div1_b", -0.8, -1.1, 2.0)
    _gate(spec, -0.8, -1.4, BUTTON_L5)
    _wall_y(spec, "mid", 0.0, -0.8, 0.8, LOW_WALL_H)
    _bridge(spec, 1, 0.0, -1.4, 1.4)
    _wall_x(spec, "div2_a", 0.8, -2.0, 1.4)
    _wall_x(spec, "div2_b", 0.8, 1.8, 2.0)
    _tunnel(spec, "tunnel", 0.5, 1.1, 1.6)


BUTTON_L4 = (-1.3, 1.3)  # Level 4: button position (x, y)
BUTTON_L5 = (-1.3, 1.3)  # Level 5: same button position as L4 (reuses L4 gate layout)


@dataclass(frozen=True)
class Level:
    name: str
    build: Callable[[mujoco.MjSpec], None]
    start: tuple[float, float]
    goal_zone: tuple[float, float, float, float]  # (x_min,x_max,y_min,y_max) goal bounds
    waypoints: tuple[Waypoint, ...]  # scripted solution, the goal itself is appended last
    max_steps: int
    button: tuple[float, float] | None = None


_RIGHT = (1.4, 1.9, -1.8, 1.8)
LEVELS: dict[int, Level] = {
    1: Level(
        "baffle", _l1, (-1.6, -1.0), _RIGHT, ((-0.5, 1.45, 0.25, 1.0), (0.5, 1.45, 0.25, 1.0)), 200
    ),
    2: Level(
        "tunnel", _l2, (-1.6, -1.0), _RIGHT, ((-0.9, 0.0, 0.12, 0.8), (0.9, 0.0, 0.15, 0.8)), 200
    ),
    3: Level("bridge", _l3, (-1.7, 0.0), _RIGHT, ((0.0, 0.0, 0.2, 0.5), (1.2, 0.0, 0.1, 0.6)), 250),
    4: Level(
        "gate",
        _l4,
        (-1.6, 0.0),
        _RIGHT,
        ((*BUTTON_L4, 0.1, 0.8), (-0.5, -1.2, 0.2, 0.8), (0.6, -1.2, 0.2, 0.8)),
        300,
        BUTTON_L4,
    ),
    5: Level(
        "combined",
        _l5,
        (-1.7, -0.3),
        _RIGHT,
        (
            (*BUTTON_L5, 0.1, 0.8),
            (-1.3, -1.4, 0.25, 0.8),
            (-0.4, -1.4, 0.2, 0.6),
            (0.0, -1.75, 0.1, 0.5),
            (0.0, 1.6, 0.12, 0.5),
            (0.45, 1.6, 0.1, 0.6),
            (1.3, 1.6, 0.2, 0.8),
        ),
        600,
        BUTTON_L5,
    ),
}


class SphereCourseEnv(SphereReachEnv):
    """:class:`SphereReachEnv` on an obstacle course; observation is 13 D (see ``OBS_SLICES``)."""

    def __init__(self, level: int = 1, **kwargs: Any) -> None:
        if level not in LEVELS:
            raise ValueError(f"level must be in {sorted(LEVELS)}, got {level}")
        self.level_id, self.level = level, LEVELS[level]
        self._pressed = False
        super().__init__(**kwargs)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(OBS_DIM,), dtype=np.float64)
        # The door joint and button geom are added by _initialize_simulation on gate levels.
        has_door = self.level.button is not None
        self._door_qadr = self.model.joint("door_slide").qposadr[0] if has_door else None
        self._button_geom = self.model.geom("button").id if has_door else None

    def _initialize_simulation(self) -> tuple[mujoco.MjModel, mujoco.MjData]:
        spec = mujoco.MjSpec.from_file(MODEL_PATH)
        spec.body("sphere").pos = (*self.level.start, 0.1)  # slide joints are relative to it
        self.level.build(spec)
        model = spec.compile()
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, self.width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, self.height)
        return model, mujoco.MjData(model)

    def _set_action_space(self) -> spaces.Box:  # only the two sphere motors are agent-controlled
        low, high = self.model.actuator_ctrlrange[:2].T.astype(np.float32)
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        return self.action_space

    @property
    def door_opening(self) -> float:
        if self._door_qadr is None:
            return 0.0
        return float(np.clip(self.data.qpos[self._door_qadr] / DOOR_TRAVEL, 0.0, 1.0))

    def _get_obs(self) -> NDArray[np.float64]:
        extras = [float(self._pressed), self.door_opening]
        return np.concatenate([super()._get_obs(), extras])

    def _dist(self) -> float:  # the base class assumes the sphere body sits at the origin
        return float(np.linalg.norm(self.goal - (self.data.qpos[:2] + self._body_offset[:2])))

    def _set_pressed(self, pressed: bool) -> None:
        self._pressed = pressed
        if self._button_geom is not None:
            self.model.geom_rgba[self._button_geom] = _BUTTON_ON if pressed else _BUTTON_OFF

    def do_simulation(self, ctrl: NDArray[np.float64], n_frames: int) -> None:
        door_target = [DOOR_TRAVEL * self._pressed] * (self.model.nu - 2)
        super().do_simulation(np.concatenate([ctrl, door_target]), n_frames)
        if self.level.button is not None and not self._pressed:
            d = np.linalg.norm(self.data.qpos[:2] + self._body_offset[:2] - self.level.button)
            if d < BUTTON_RADIUS:
                self._set_pressed(True)

    def step(  # pyright: ignore[reportIncompatibleMethodOverride]
        self, action: NDArray[np.floating]
    ) -> tuple[NDArray[np.float64], float, bool, bool, dict[str, Any]]:
        obs, reward, terminated, truncated, info = super().step(action)
        info["button_pressed"] = self._pressed
        return obs, reward, terminated, truncated, info

    def reset_model(self) -> NDArray[np.float64]:
        self._set_pressed(False)
        qpos = self.init_qpos.copy()
        qpos[:2] += self.np_random.uniform(-START_JITTER, START_JITTER, size=2)
        self.set_state(qpos, self.init_qvel)
        return self.sample_goal()

    def sample_goal(self) -> NDArray[np.float64]:
        x0, x1, y0, y1 = self.level.goal_zone
        self.data.mocap_pos[self._goal_mocap, :2] = (
            self.np_random.uniform(x0, x1),
            self.np_random.uniform(y0, y1),
        )
        self._prev_dist = self._dist()
        return self._get_obs()

    def set_from_obs(self, obs: NDArray[np.float64]) -> None:
        # Extract base observation (11D) for parent class
        base_obs = np.concatenate(
            [
                obs[BASE_OBS_SLICES["pos"]],
                obs[BASE_OBS_SLICES["vel"]],
                obs[BASE_OBS_SLICES["ang_vel"]],
                obs[BASE_OBS_SLICES["goal_rel"]],
            ]
        )
        super().set_from_obs(base_obs)
        if self._door_qadr is not None:
            qpos = self.data.qpos.copy()
            qpos[self._door_qadr] = obs[OBS_SLICES["door"]][0] * DOOR_TRAVEL
            self.set_state(qpos, self.data.qvel)
        self._set_pressed(bool(obs[OBS_SLICES["button"]][0] > 0.5))


class WaypointPolicy:
    """Velocity-tracking waypoint follower using only the observation and the level waypoints.

    Desired velocity points at the next waypoint (slowing down within ``BRAKE`` of it); the force
    is a proportional velocity feedback plus the feed-forward of the viscous damping.
    """

    KV = 3.0
    BRAKE = 0.6
    FINAL_SPEED = 0.8

    def __init__(self, level: int | Level) -> None:
        self.level = LEVELS[level] if isinstance(level, int) else level
        self.reset()

    def reset(self) -> None:
        self._i = 0

    def act(self, obs: NDArray[np.float64]) -> NDArray[np.float64]:
        pos, vel = obs[OBS_SLICES["pos"]][:2], obs[OBS_SLICES["vel"]][:2]
        wps = self.level.waypoints
        while (
            self._i < len(wps)
            and np.linalg.norm(np.asarray(wps[self._i][:2]) - pos) < wps[self._i][2]
        ):
            self._i += 1
        if self._i < len(wps):
            target, vmax = np.asarray(wps[self._i][:2]), wps[self._i][3]
        else:
            target, vmax = pos + obs[OBS_SLICES["goal_rel"]], self.FINAL_SPEED
        err = target - pos
        dist = max(float(np.linalg.norm(err)), 1e-9)
        v_des = vmax * err / dist * min(1.0, dist / self.BRAKE)
        return np.clip(self.KV * (v_des - vel) + 0.69 * v_des, -1.0, 1.0)
