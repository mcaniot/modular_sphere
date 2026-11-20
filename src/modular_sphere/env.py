"""Gymnasium environment: the modular sphere must roll onto a goal."""

from __future__ import annotations

from importlib import resources
from typing import Any, ClassVar

import mujoco
import numpy as np
from gymnasium import spaces
from gymnasium.envs.mujoco import MujocoEnv
from numpy.typing import NDArray

MODEL_PATH = str(resources.files("modular_sphere") / "assets" / "sphere_arena.xml")
FRAME_SKIP = 25  # 2 ms physics -> 50 ms control step (20 Hz)
MAX_EPISODE_STEPS = 200  # 10 s
GOAL_MIN_DIST = 0.5
GOAL_MAX_DIST = 1.0
SUCCESS_DIST = 0.2  # sphere surfaces touch (2 * radius)
SUCCESS_BONUS = 10.0
STEP_PENALTY = 0.01
SETTLE_STEPS = 500  # steps to let sphere settle on floor (contact penetration included)

# Observation layout, shared with the world-model tooling.
OBS_SLICES: dict[str, slice] = {
    "pos": slice(0, 3),  # sphere centre x, y, z [m], world frame (z changes on wall impacts)
    "vel": slice(3, 6),  # sphere linear velocity x, y, z [m/s], world frame
    "ang_vel": slice(6, 9),  # sphere angular velocity x, y, z [rad/s], world frame
    "goal_rel": slice(9, 11),  # goal - sphere x, y [m], world frame
}
OBS_DIM = 11
ACT_DIM = 2  # world-frame force x, y [N], in [-1, 1]


class SphereReachEnv(MujocoEnv):
    """Push a rolling sphere with a world-frame planar force until it touches the goal.

    The observation is the full Markov state of the dynamics: the sphere is symmetric so its
    orientation does not influence the physics (``set_from_obs`` relies on it).
    """

    metadata: ClassVar[dict[str, Any]] = {
        "render_modes": ["human", "rgb_array", "depth_array"],
        "render_fps": 20,
    }

    def __init__(
        self,
        render_mode: str | None = None,
        width: int = 480,
        height: int = 480,
        camera_name: str = "top",
    ) -> None:
        observation_space = spaces.Box(-np.inf, np.inf, shape=(OBS_DIM,), dtype=np.float64)
        super().__init__(
            MODEL_PATH,
            FRAME_SKIP,
            observation_space,
            render_mode=render_mode,
            width=width,
            height=height,
            camera_name=camera_name,
        )
        self._goal_mocap = self.model.body("goal").mocapid[0]
        self._body_offset = self.model.body("sphere").pos.copy()  # slide joints are relative to it
        # Start every episode with the sphere settled on the floor (contact penetration included).
        mujoco.mj_step(self.model, self.data, nstep=SETTLE_STEPS)
        self.init_qpos[2] = self.data.qpos[2]
        self._prev_dist = 0.0

    @property
    def goal(self) -> NDArray[np.float64]:
        return self.data.mocap_pos[self._goal_mocap, :2].copy()

    def _get_obs(self) -> NDArray[np.float64]:
        pos = self.data.qpos[:3] + self._body_offset
        ang_vel = np.empty(3)  # ball-joint velocity is in the body frame, expose it in world frame
        mujoco.mju_rotVecQuat(ang_vel, self.data.qvel[3:6], self.data.qpos[3:7])
        return np.concatenate([pos, self.data.qvel[:3], ang_vel, self.goal - pos[:2]])

    def _dist(self) -> float:
        return float(np.linalg.norm(self.goal - self.data.qpos[:2]))

    def step(  # pyright: ignore[reportIncompatibleMethodOverride]
        self, action: NDArray[np.floating]
    ) -> tuple[NDArray[np.float64], float, bool, bool, dict[str, Any]]:
        ctrl = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        self.do_simulation(ctrl, self.frame_skip)
        dist = self._dist()
        success = dist < SUCCESS_DIST
        reward = (self._prev_dist - dist) - STEP_PENALTY + SUCCESS_BONUS * success
        self._prev_dist = dist
        if self.render_mode == "human":
            self.render()
        return (
            self._get_obs(),
            float(reward),
            bool(success),
            False,
            {"distance": dist, "is_success": success},
        )

    def reset_model(self) -> NDArray[np.float64]:
        self.set_state(self.init_qpos, self.init_qvel)
        return self.sample_goal()

    def sample_goal(self) -> NDArray[np.float64]:
        """Draw a new goal (|x|, |y| in [GOAL_MIN_DIST, GOAL_MAX_DIST]); returns the new obs."""
        signs = self.np_random.choice([-1.0, 1.0], size=2)
        goal_xy = signs * self.np_random.uniform(GOAL_MIN_DIST, GOAL_MAX_DIST, size=2)
        self.data.mocap_pos[self._goal_mocap, :2] = goal_xy
        self._prev_dist = self._dist()
        return self._get_obs()

    def set_from_obs(self, obs: NDArray[np.float64]) -> None:
        """Set the simulator to the state encoded by an observation (used by the oracle model)."""
        if obs.shape != (OBS_DIM,):
            raise ValueError(f"obs must have shape ({OBS_DIM},), got {obs.shape}")
        qpos = self.init_qpos.copy()
        qvel = np.zeros_like(self.init_qvel)
        pos = obs[OBS_SLICES["pos"]]
        qpos[:3] = pos - self._body_offset
        qvel[:3] = obs[OBS_SLICES["vel"]]
        qvel[3:6] = obs[OBS_SLICES["ang_vel"]]  # identity orientation: body frame == world frame
        self.data.mocap_pos[self._goal_mocap, :2] = pos[:2] + obs[OBS_SLICES["goal_rel"]]
        self.data.qacc_warmstart[:] = 0.0
        self.set_state(qpos, qvel)
        self._prev_dist = self._dist()
