"""World-model interface, reference models and open-loop evaluation."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np
from numpy.typing import NDArray

from modular_sphere.dataset import TrajectoryDataset
from modular_sphere.env import OBS_SLICES, SphereReachEnv

Array = NDArray[np.float64]


@runtime_checkable
class WorldModel(Protocol):
    """A dynamics model predicting observations open-loop from an initial observation.

    ``rollout(obs0 (B, obs_dim), actions (B, H, act_dim)) -> predicted obs (B, H, obs_dim)``, where
    prediction ``[:, k]`` is the observation after applying ``actions[:, : k + 1]``.
    Models that learn from data may also expose ``fit(dataset: TrajectoryDataset) -> None``.
    """

    def rollout(self, obs0: Array, actions: Array) -> Array: ...


class ConstantVelocityModel:
    """Physics-free reference: positions integrate the current velocity, the rest is frozen."""

    def __init__(self, dt: float) -> None:
        self.dt = dt

    def rollout(self, obs0: Array, actions: Array) -> Array:
        obs = obs0.copy()
        preds = np.empty((*actions.shape[:2], obs0.shape[-1]))
        for k in range(actions.shape[1]):
            displacement = obs[:, OBS_SLICES["vel"]] * self.dt
            obs[:, OBS_SLICES["pos"]] += displacement
            obs[:, OBS_SLICES["goal_rel"]] -= displacement[:, :2]
            preds[:, k] = obs
        return preds


class LinearDeltaModel:
    """Ridge regression of ``obs[t+1] - obs[t]`` on ``[obs[t], action[t], 1]``, rolled out."""

    def __init__(self, ridge: float = 1e-6) -> None:
        self.ridge = ridge
        self.weights: Array | None = None

    @staticmethod
    def _features(obs: Array, actions: Array) -> Array:
        return np.concatenate([obs, actions, np.ones((*obs.shape[:-1], 1))], axis=-1)

    def fit(self, dataset: TrajectoryDataset) -> None:
        x = self._features(dataset.obs[:, :-1], dataset.actions)
        x = x.reshape(-1, x.shape[-1])
        y = np.diff(dataset.obs, axis=1).reshape(-1, dataset.obs.shape[-1])
        self.weights = np.linalg.solve(x.T @ x + self.ridge * np.eye(x.shape[1]), x.T @ y)

    def rollout(self, obs0: Array, actions: Array) -> Array:
        if self.weights is None:
            raise RuntimeError("LinearDeltaModel.fit must be called before rollout")
        obs = obs0.copy()
        preds = np.empty((*actions.shape[:2], obs0.shape[-1]))
        for k in range(actions.shape[1]):
            obs = obs + self._features(obs, actions[:, k]) @ self.weights
            preds[:, k] = obs
        return preds


class MuJoCoOracleModel:
    """Ground truth: re-simulates from the observation (exact because the observation is Markov)."""

    def __init__(self) -> None:
        self.env = SphereReachEnv()

    def rollout(self, obs0: Array, actions: Array) -> Array:
        preds = np.empty((*actions.shape[:2], obs0.shape[-1]))
        for b in range(obs0.shape[0]):
            self.env.set_from_obs(obs0[b])
            for k in range(actions.shape[1]):
                preds[b, k] = self.env.step(actions[b, k])[0]
        return preds


def evaluate(
    model: WorldModel, dataset: TrajectoryDataset, horizons: list[int], stride: int = 10
) -> dict[int, dict[str, float]]:
    """Open-loop RMSE per horizon and observation group, on windows every ``stride`` steps.

    Units are the physical ones of each group (m, m/s, rad/s); ``all`` mixes them.
    """
    horizon = max(horizons)
    if not 1 <= horizon <= dataset.num_steps:
        raise ValueError(f"horizons must be in [1, {dataset.num_steps}], got {horizons}")
    starts = range(0, dataset.num_steps - horizon + 1, stride)
    obs0 = np.concatenate([dataset.obs[:, t] for t in starts])
    actions = np.concatenate([dataset.actions[:, t : t + horizon] for t in starts])
    target = np.concatenate([dataset.obs[:, t + 1 : t + horizon + 1] for t in starts])
    preds = model.rollout(obs0, actions)
    if preds.shape != target.shape:
        raise ValueError(f"rollout returned shape {preds.shape}, expected {target.shape}")
    sq_err = (preds - target) ** 2
    groups = {"all": slice(None), **OBS_SLICES}
    return {
        h: {name: float(np.sqrt(sq_err[:, h - 1, sl].mean())) for name, sl in groups.items()}
        for h in sorted(horizons)
    }
