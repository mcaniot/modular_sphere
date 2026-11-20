"""Trajectory datasets for world-model training and evaluation."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from modular_sphere.env import ACT_DIM, OBS_DIM, OBS_SLICES, SUCCESS_DIST, SphereReachEnv


@dataclass(frozen=True)
class TrajectoryDataset:
    """Fixed-length episodes: ``obs[:, t + 1]`` follows ``actions[:, t]`` applied at ``obs[:, t]``.

    Episodes are not cut at success (the goal is not physical), so every window is valid.
    """

    obs: NDArray[np.float64]  # (episodes, steps + 1, obs_dim)
    actions: NDArray[np.float64]  # (episodes, steps, act_dim)
    rewards: NDArray[np.float64]  # (episodes, steps)
    successes: NDArray[np.bool_]  # (episodes, steps)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        n, t = self.actions.shape[:2]
        if (
            self.obs.shape[:2] != (n, t + 1)
            or self.rewards.shape != (n, t)
            or self.successes.shape != (n, t)
        ):
            raise ValueError(
                f"inconsistent shapes: obs {self.obs.shape}, actions "
                f"{self.actions.shape}, rewards {self.rewards.shape}, "
                f"successes {self.successes.shape}"
            )

    @property
    def num_episodes(self) -> int:
        return self.actions.shape[0]

    @property
    def num_steps(self) -> int:
        return self.actions.shape[1]

    def save(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            obs=self.obs,
            actions=self.actions,
            rewards=self.rewards,
            successes=self.successes,
            metadata=json.dumps(self.metadata),
        )

    @classmethod
    def load(cls, path: str | Path) -> TrajectoryDataset:
        with np.load(path, allow_pickle=False) as f:
            return cls(
                obs=f["obs"],
                actions=f["actions"],
                rewards=f["rewards"],
                successes=f["successes"],
                metadata=json.loads(str(f["metadata"])),
            )


def exploration_action(
    rng: np.random.Generator,
    obs: NDArray[np.float64],
    goal_seeking: bool,
    noise: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Smoothed random noise, or a noisy proportional-derivative goal-seeker."""
    noise += -0.15 * noise + 0.3 * rng.standard_normal(noise.shape)  # Ornstein-Uhlenbeck, in place
    if not goal_seeking:
        return np.clip(noise, -1.0, 1.0)
    pd = 2.0 * obs[OBS_SLICES["goal_rel"]] - obs[OBS_SLICES["vel"]][:2]
    return np.clip(pd + 0.5 * noise, -1.0, 1.0)


def collect(
    num_episodes: int, steps: int, seed: int, goal_seeking_ratio: float = 0.5
) -> TrajectoryDataset:
    """Roll out exploration policies in ``SphereReachEnv`` without auto-reset."""
    rng = np.random.default_rng(seed)
    env = SphereReachEnv()
    obs_buf = np.empty((num_episodes, steps + 1, OBS_DIM))
    act_buf = np.empty((num_episodes, steps, ACT_DIM))
    rew_buf = np.empty((num_episodes, steps))
    suc_buf = np.empty((num_episodes, steps), dtype=bool)
    for ep in range(num_episodes):
        obs, _ = env.reset(seed=seed + ep)
        goal_seeking = bool(rng.random() < goal_seeking_ratio)
        noise = np.zeros(ACT_DIM)
        obs_buf[ep, 0] = obs
        for t in range(steps):
            action = exploration_action(rng, obs, goal_seeking, noise)
            obs, reward, success, _, _ = env.step(action)
            obs_buf[ep, t + 1], act_buf[ep, t] = obs, action
            rew_buf[ep, t], suc_buf[ep, t] = reward, success
    env.close()
    metadata = {
        "env": "ModularSphere/Reach-v0",
        "dt": env.dt,
        "seed": seed,
        "steps": steps,
        "goal_seeking_ratio": goal_seeking_ratio,
        "success_dist": SUCCESS_DIST,
    }
    return TrajectoryDataset(obs_buf, act_buf, rew_buf, suc_buf, metadata)
