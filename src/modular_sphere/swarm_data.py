"""Swarm trajectory datasets: scripted-expert collection with DART noise and DAgger relabelling.

Every swarm task is first solved by a scripted approach (``swarm.SwarmClimbPolicy``) and then
learned: this module produces the data for the world model and for imitation.

- Noise injection follows DART (Laskey et al., 2017): the executed action is the expert action plus
  Gaussian noise on the continuous channels (fx, fy, lift), the label is the clean expert action,
  so the data covers the states the learner drifts to. The binary ``attach`` channel is kept clean.
- DAgger (Ross et al., 2011): ``collect`` also accepts a learner ``policy`` that drives the swarm
  with probability ``1 - beta`` per step while the expert labels every visited state.
- World-model exploration (``random_prob``): each sphere, per segment of 5 to 20 steps, executes a
  held uniform random action (any force, lift and attach) with probability ``random_prob``, so the
  model also sees full-force pushes against the step and walls, lifts and links the expert never
  makes; a policy improved in imagination drives there and would otherwise exploit model errors.

Arrays (E episodes, T = max steps, N spheres, padded after the episode end by repeating the last
state and zero actions; ``valid[e, t]`` marks real transitions):
``spheres`` (E, T+1, N, 9) pos, vel, ang_vel per sphere; ``links`` (E, T+1, N, N);
``obs`` (E, T+1, N, obs_dim) local observations; ``actions`` / ``expert`` (E, T, N, 4);
``rewards`` (E, T); ``goal`` (E, 3); ``success`` (E,).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from modular_sphere.swarm import ACT_DIM, SphereSwarmStepEnv, SwarmClimbPolicy

Array = NDArray[np.float64]
ObsDict = dict[str, Array]
Policy = Callable[[ObsDict], ObsDict]


@dataclass
class SwarmDataset:
    spheres: Array
    links: NDArray[np.bool_]
    obs: Array
    actions: Array
    expert: Array
    rewards: Array
    valid: NDArray[np.bool_]
    goal: Array
    success: NDArray[np.bool_]

    @property
    def n_spheres(self) -> int:
        return self.spheres.shape[2]

    def save(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, **{f.name: getattr(self, f.name) for f in fields(self)})

    @classmethod
    def load(cls, path: str | Path) -> SwarmDataset:
        with np.load(path) as data:
            return cls(**{f.name: data[f.name] for f in fields(cls)})

    @classmethod
    def concat(cls, parts: list[SwarmDataset]) -> SwarmDataset:
        return cls(
            **{f.name: np.concatenate([getattr(p, f.name) for p in parts]) for f in fields(cls)}
        )


def empty_arrays(n_episodes: int, horizon: int, n_spheres: int, obs_dim: int) -> dict[str, Array]:
    """Zero-filled ``SwarmDataset`` fields, to fill episode by episode."""
    e, h, n = n_episodes, horizon, n_spheres
    return {
        "spheres": np.zeros((e, h + 1, n, 9)),
        "links": np.zeros((e, h + 1, n, n), dtype=bool),
        "obs": np.zeros((e, h + 1, n, obs_dim)),
        "actions": np.zeros((e, h, n, ACT_DIM)),
        "expert": np.zeros((e, h, n, ACT_DIM)),
        "rewards": np.zeros((e, h)),
        "valid": np.zeros((e, h), dtype=bool),
        "goal": np.zeros((e, 3)),
        "success": np.zeros(e, dtype=bool),
    }


def stack_obs(obs: ObsDict, agents: list[str]) -> Array:
    return np.stack([obs[a] for a in agents])


def record(out: dict[str, Array], env: SphereSwarmStepEnv, e: int, t: int, obs: ObsDict) -> None:
    """Write the current states, links and observations from step ``t`` of episode ``e`` on (the
    tail stays filled with the last state)."""
    out["spheres"][e, t:] = env.spheres()
    out["links"][e, t:] = env.links
    out["obs"][e, t:] = stack_obs(obs, env.possible_agents)


def collect(
    n_episodes: int,
    n_spheres: int = 4,
    seed: int = 0,
    noise: float = 0.3,
    policy: Policy | None = None,
    beta: float = 1.0,
    random_prob: float = 0.0,
) -> SwarmDataset:
    """Roll out episodes ``seed .. seed + n_episodes - 1``; the expert labels every state.

    ``noise`` is the DART Gaussian std on fx, fy, lift of the executed expert action. With a learner
    ``policy``, each step executes the expert (noisy) with probability ``beta``, else the learner.
    """
    env = SphereSwarmStepEnv(n_spheres=n_spheres)
    expert = SwarmClimbPolicy(n_spheres, env.radius)
    agents, horizon = env.possible_agents, env.max_steps
    rng = np.random.default_rng(seed)
    out = empty_arrays(n_episodes, horizon, n_spheres, env.slices["others"].stop)

    for e in range(n_episodes):
        obs, _ = env.reset(seed=seed + e)
        expert.reset()
        out["goal"][e] = env.goal
        record(out, env, e, 0, obs)
        t = 0
        hold = np.zeros(n_spheres, dtype=int)
        explore = np.zeros(n_spheres, dtype=bool)
        random_act = np.zeros((n_spheres, ACT_DIM))
        while env.agents:
            label = stack_obs(expert.act(obs), agents)
            if policy is None or rng.random() < beta:
                executed = label.copy()
                executed[:, :3] += noise * rng.standard_normal((n_spheres, 3))
            else:
                executed = stack_obs(policy(obs), agents)
            if random_prob > 0:  # held random actions (module docstring)
                new = hold <= 0
                hold[new] = rng.integers(5, 21, new.sum())
                explore[new] = rng.random(new.sum()) < random_prob
                random_act[new] = rng.uniform(-1.0, 1.0, (new.sum(), ACT_DIM))
                random_act[new, 3] = np.sign(random_act[new, 3])
                executed[explore] = random_act[explore]
                hold -= 1
            executed = np.clip(executed, -1.0, 1.0)
            obs, rewards, terms, _, _ = env.step(dict(zip(agents, executed, strict=True)))
            out["actions"][e, t], out["expert"][e, t] = executed, label
            out["rewards"][e, t], out["valid"][e, t] = rewards[agents[0]], True
            out["success"][e] = terms[agents[0]]
            t += 1
            record(out, env, e, t, obs)
    return SwarmDataset(**out)


def run_episodes(policy: Policy, n_spheres: int, seeds: list[int],
                 reset: Callable[[], None] | None = None) -> tuple[Array, ...]:  # fmt: skip
    """Closed-loop MuJoCo episodes, one per seed (``reset`` before each): success, steps and
    number of arrived spheres per seed."""
    env = SphereSwarmStepEnv(n_spheres=n_spheres)
    success, steps, arrived = [], [], []
    for seed in seeds:
        obs, _ = env.reset(seed=seed)
        if reset is not None:
            reset()
        t, info = 0, {}
        while env.agents:
            obs, _, _, _, infos = env.step(policy(obs))
            info, t = infos[env.possible_agents[0]], t + 1
        success.append(info["is_success"])
        steps.append(t)
        arrived.append(info["n_arrived"])
    return np.array(success, bool), np.array(steps, np.int64), np.array(arrived, np.int64)


def expert_steps(n_spheres: int, seeds: list[int]) -> NDArray[np.int64]:
    """Episode length of the scripted expert per seed: the time reference of the speed objective."""
    expert = SwarmClimbPolicy(n_spheres)
    return run_episodes(expert.act, n_spheres, seeds, expert.reset)[1]
