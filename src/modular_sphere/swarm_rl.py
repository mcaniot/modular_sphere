"""Multi-agent RL in MuJoCo for the swarm step climb: find a faster collective strategy.

The scripted ``SwarmClimbPolicy`` stages the spheres one by one (about 43 % of them move at a
time). ``train_mappo`` lets the swarm discover its own use of links and lift, with every sphere
free to move at once, and rewards speed:

- MAPPO (Yu et al., 2022): one decentralised actor shared by the spheres (the ``SwarmPolicy``
  network on local observations), a centralised critic on the global state and the episode time
  fraction, PPO clipping (Schulman et al., 2017) with a per-agent ratio and the team advantage
  (GAE, Schulman et al., 2016). Time-outs bootstrap from the critic; successes do not.
- Jump start (Uchendu et al., 2023): the actor starts from the imitation (DAgger) policy, the
  critic is first fitted with the actor frozen; there is no KL or BC term to the demonstration, so
  the strategy is free to change. Exploration: Gaussian on fx, fy, lift with a learned
  state-independent std, Bernoulli on attach.
- Speed: the shared reward is the environment reward minus ``time_penalty`` per step.

Rollouts run in ``SwarmVecEnv`` worker processes (the environment is CPU bound, about 3k steps/s
per process); ``evaluate`` replays fixed seeds with the deterministic policy (mean action,
attach when the logit is positive) and also measures how many spheres move at the same time.
"""

from __future__ import annotations

import contextlib
import multiprocessing as mp
from collections.abc import Callable
from dataclasses import dataclass
from multiprocessing.connection import Connection
from typing import Any

import numpy as np
import torch
from numpy.typing import NDArray
from torch import nn

from modular_sphere.swarm import SphereSwarmStepEnv, moving_fraction, obs_slices
from modular_sphere.swarm_data import stack_obs
from modular_sphere.swarm_torch import SwarmPolicy, _mlp, _ppo_dist

Array = NDArray[np.float64]
TRAIN_SEED = 1_000_000  # first reset seed of worker 0; worker w starts at TRAIN_SEED * (w + 1)


def _worker(conn: Connection, n_spheres: int, k: int, seed: int, time_penalty: float) -> None:
    envs = [SphereSwarmStepEnv(n_spheres=n_spheres) for _ in range(k)]
    agents = envs[0].possible_agents
    height = envs[0].step_height

    def fresh(i: int) -> SphereSwarmStepEnv:
        """Env ``i``, rebuilt when the curriculum changed the step height."""
        if envs[i].step_height != height:
            envs[i] = SphereSwarmStepEnv(n_spheres=n_spheres, step_height=height)
        return envs[i]

    def state(env: SphereSwarmStepEnv) -> Array:
        return np.append(env.state(), env._steps / env.max_steps)

    obs = [np.zeros(0)] * k
    done = np.zeros(k, dtype=bool)
    auto = True
    while True:
        cmd, arg = conn.recv()
        if cmd == "reset":
            seeds, auto = arg
            for i in range(k):
                env = fresh(i)
                if seeds is None:
                    seeds_i, seed = seed, seed + 1
                else:
                    seeds_i = seeds[i]
                done[i] = seeds_i is None  # padding slot: reset, then never stepped
                obs[i] = stack_obs(env.reset(seed=0 if done[i] else seeds_i)[0], agents)
            conn.send((np.stack(obs), np.stack([state(e) for e in envs])))
        elif cmd == "step":
            out = {key: np.zeros(k) for key in ("reward", "term", "trunc", "arrived", "links")}
            final = np.stack([state(e) for e in envs])
            for i, env in enumerate(envs):
                if done[i]:
                    continue
                o, r, te, tr, info = env.step({a: arg[i, j] for j, a in enumerate(agents)})
                a0 = agents[0]
                out["reward"][i] = r[a0] - time_penalty
                out["term"][i], out["trunc"][i] = te[a0], tr[a0]
                out["arrived"][i], out["links"][i] = info[a0]["n_arrived"], info[a0]["n_links"]
                final[i] = state(env)
                if te[a0] or tr[a0]:
                    if auto:
                        o, _ = fresh(i).reset(seed=seed)
                        seed += 1
                    else:
                        done[i] = True
                obs[i] = stack_obs(o, agents)
            out["done"] = done.copy()
            conn.send((np.stack(obs), np.stack([state(e) for e in envs]), final, out))
        elif cmd == "height":
            height = arg  # applied at each env's next reset
        else:
            conn.close()
            return


class SwarmVecEnv:
    """``n_workers`` processes x ``envs_per_worker`` swarm environments, arrays in and out.

    ``reset`` -> obs (E, N, obs_dim), state (E, S) with S = global state + time fraction.
    ``step(actions (E, N, 4))`` -> obs, state, final state (before any auto-reset, for
    bootstrapping time-outs) and per-env arrays ``reward``, ``term``, ``trunc``, ``arrived``,
    ``links``, ``done`` (finished evaluation episodes, when auto-reset is off).
    """

    def __init__(self, n_spheres: int, n_workers: int = 6, envs_per_worker: int = 4,
                 time_penalty: float = 0.0) -> None:  # fmt: skip
        ctx = mp.get_context("spawn")
        self.n, self.k = n_spheres, envs_per_worker
        self.num_envs = n_workers * envs_per_worker
        self._conns, self._procs = [], []
        for w in range(n_workers):
            parent, child = ctx.Pipe()
            args = (child, n_spheres, envs_per_worker, TRAIN_SEED * (w + 1), time_penalty)
            proc = ctx.Process(target=_worker, args=args, daemon=True)
            proc.start()
            self._conns.append(parent)
            self._procs.append(proc)

    def reset(self, seeds: list[int | None] | None = None, auto: bool = True) -> tuple[Array, ...]:
        """New training episodes (``seeds=None``) or the given seeds, padded with ``None``."""
        for w, conn in enumerate(self._conns):
            part = None if seeds is None else seeds[w * self.k : (w + 1) * self.k]
            conn.send(("reset", (part, auto)))
        obs, state = zip(*(c.recv() for c in self._conns), strict=True)
        return np.concatenate(obs), np.concatenate(state)

    def step(self, actions: Array) -> tuple[Array, Array, Array, dict[str, Array]]:
        for w, conn in enumerate(self._conns):
            conn.send(("step", actions[w * self.k : (w + 1) * self.k]))
        res = [c.recv() for c in self._conns]
        info = {key: np.concatenate([r[3][key] for r in res]) for key in res[0][3]}
        return (*(np.concatenate([r[i] for r in res]) for i in range(3)), info)

    def set_step_height(self, height: float) -> None:
        """Curriculum: episodes started after this call use a step of ``height`` m."""
        for conn in self._conns:
            conn.send(("height", height))

    def close(self) -> None:
        for conn in self._conns:
            with contextlib.suppress(BrokenPipeError, OSError):  # worker already gone
                conn.send(("close", None))
            conn.close()
        for proc in self._procs:
            proc.join(timeout=5)
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=1)


@dataclass
class RLEval:
    success: NDArray[np.bool_]  # per seed
    steps: NDArray[np.int64]
    moving: Array  # mean fraction of spheres moving, per seed
    links: Array  # mean number of active links, per seed


def evaluate(policy: SwarmPolicy, vec: SwarmVecEnv, seeds: list[int]) -> RLEval:
    """Deterministic closed-loop episodes on ``seeds`` (batches of ``vec.num_envs``)."""
    if not seeds:
        raise ValueError("seeds must not be empty")
    vel = obs_slices(vec.n)["vel"]
    res: dict[str, list[Any]] = {"success": [], "steps": [], "moving": [], "links": []}
    for b in range(0, len(seeds), vec.num_envs):
        batch = seeds[b : b + vec.num_envs]
        obs, _ = vec.reset([*batch, *[None] * (vec.num_envs - len(batch))], auto=False)
        m = len(batch)
        active = np.ones(m, dtype=bool)
        steps, success = np.zeros(m, np.int64), np.zeros(m, bool)
        moving, links = np.zeros(m), np.zeros(m)
        while active.any():
            moves = moving_fraction(obs[:m, :, vel])
            obs, _, _, info = vec.step(policy.act(obs))
            moving += active * moves
            links += active * info["links"][:m]
            steps += active
            success |= active & (info["term"][:m] > 0)
            active &= ~info["done"][:m]
        res["success"].append(success)
        res["steps"].append(steps)
        res["moving"].append(moving / steps)
        res["links"].append(links / steps)
    return RLEval(**{key: np.concatenate(v) for key, v in res.items()})


def train_mappo(
    policy: SwarmPolicy,
    vec: SwarmVecEnv,
    iterations: int = 300,
    horizon: int = 128,
    epochs: int = 5,
    minibatch: int = 4096,
    gamma: float = 0.995,
    lam: float = 0.95,
    clip: float = 0.2,
    lr: float = 1e-4,
    critic_lr: float = 3e-4,
    log_std: float = -1.0,
    entropy_weight: float = 0.0,
    critic_warmup: int = 10,
    hidden: int = 256,
    seed: int = 0,
    callback: Callable[[int, dict[str, float]], None] | None = None,
) -> list[dict[str, float]]:
    """Improve ``policy`` in place with MAPPO on ``vec`` (time penalty set on the vec env).

    ``callback(iteration, metrics)`` runs after each update (evaluation, checkpoints). Metrics:
    mean reward per step, finished episodes, their success rate and length, policy std, critic
    loss and the clip fraction.
    """
    torch.manual_seed(seed)
    dev, n = policy.device, vec.n
    obs, state = vec.reset()
    critic = _mlp(state.shape[1], 1, hidden).to(dev)
    log_std_p = nn.Parameter(torch.full((3,), log_std, device=dev))
    actor_opt = torch.optim.Adam([*policy.net.parameters(), log_std_p], lr=lr)
    critic_opt = torch.optim.Adam(critic.parameters(), lr=critic_lr)

    def value(s: torch.Tensor) -> torch.Tensor:
        # Raw state: metres, m/s, rad/s, link flags and time fraction are all O(1-10). Statistics
        # of the first reset would divide velocities (all zero then) by a tiny std.
        return critic(s).squeeze(-1)

    t = policy._t
    log: list[dict[str, float]] = []
    ep_len = np.zeros(vec.num_envs)
    for it in range(iterations):
        buf: dict[str, list[Any]] = {k: [] for k in ("obs", "state", "act", "logp", "rew", "nv")}
        dones, lengths, wins = [], [], []
        with torch.no_grad():
            for _ in range(horizon):
                o = t(obs)
                normal, bern = _ppo_dist(policy.net(o, policy.stats), log_std_p)
                cont, att = normal.sample(), bern.sample()
                logp = normal.log_prob(cont).sum(-1) + bern.log_prob(att)
                act = torch.cat([cont, att[..., None] * 2 - 1], -1).cpu().numpy()
                obs_next, state_next, final, info = vec.step(act.astype(np.float64))
                term, trunc = info["term"] > 0, info["trunc"] > 0
                next_v = value(t(final)).cpu().numpy() * ~term  # bootstrap all but terminations
                ep_len += 1
                for i in np.flatnonzero(term | trunc):
                    lengths.append(ep_len[i])
                    wins.append(term[i])
                    ep_len[i] = 0
                for key, val in zip(buf, (o, t(state), torch.cat([cont, att[..., None]], -1),
                                          logp, info["reward"], next_v), strict=True):  # fmt: skip
                    buf[key].append(val)
                dones.append(term | trunc)
                obs, state = obs_next, state_next
            states = torch.stack(buf["state"])  # (T, E, S)
            v = value(states).cpu().numpy()
            rew, nv, done = np.stack(buf["rew"]), np.stack(buf["nv"]), np.stack(dones)
            adv, gae = np.zeros_like(rew), np.zeros(vec.num_envs)
            for k in reversed(range(horizon)):
                delta = rew[k] + gamma * nv[k] - v[k]
                gae = delta + gamma * lam * ~done[k] * gae
                adv[k] = gae
        ret = t(adv + v).flatten()
        adv_t = t((adv - adv.mean()) / (adv.std() + 1e-8)).flatten()
        o_all = torch.stack(buf["obs"]).flatten(0, 1)
        s_all, a_all = states.flatten(0, 1), torch.stack(buf["act"]).flatten(0, 1)
        lp_all = torch.stack(buf["logp"]).flatten(0, 1)
        stats = {"critic": 0.0, "clip_frac": 0.0}
        count = 0
        for _ in range(epochs):
            for idx in torch.randperm(o_all.shape[0], device=dev).split(max(1, minibatch // n)):
                c_loss = nn.functional.smooth_l1_loss(value(s_all[idx]), ret[idx])
                critic_opt.zero_grad(set_to_none=True)
                c_loss.backward()
                critic_opt.step()
                stats["critic"] += c_loss.item()
                count += 1
                if it < critic_warmup:
                    continue
                normal, bern = _ppo_dist(policy.net(o_all[idx], policy.stats), log_std_p)
                a = a_all[idx]
                logp = normal.log_prob(a[..., :3]).sum(-1) + bern.log_prob(a[..., 3])
                ratio = (logp - lp_all[idx]).exp()  # per agent, (B, N)
                adv_i = adv_t[idx, None]
                surr = torch.min(ratio * adv_i, ratio.clamp(1 - clip, 1 + clip) * adv_i)
                ent = normal.entropy().sum(-1) + bern.entropy()
                loss = -(surr + entropy_weight * ent).mean()
                actor_opt.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_([*policy.net.parameters(), log_std_p], 1.0)
                actor_opt.step()
                with torch.no_grad():
                    log_std_p.clamp_(min=-3.0, max=0.0)
                stats["clip_frac"] += ((ratio - 1).abs() > clip).float().mean().item()
        metrics = {
            "reward": float(rew.mean()),
            "episodes": float(len(lengths)),
            "success": float(np.mean(wins)) if wins else float("nan"),
            "length": float(np.mean(lengths)) if lengths else float("nan"),
            "std": float(log_std_p.detach().exp().mean()),
            "critic": stats["critic"] / count,
            "clip_frac": stats["clip_frac"] / count,
        }
        log.append(metrics)
        if callback is not None:
            callback(it, metrics)
    return log
