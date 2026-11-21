"""Skill library of the swarm climb: small expert world models, one per skill.

The collective climb decomposes into skills that each involve one or two spheres:

- ``roll``: sphere ``i`` rolls to an xy target on its current level (floor or platform).
- ``climb``: climber ``c`` goes over base ``b`` held against the step face (align, link, lift,
  release on top), as in ``SwarmClimbPolicy``.
- ``pull``: helper ``h`` on the platform edge links with base ``b`` at the face and ``b`` pivots up.

Each skill has a scripted controller (the expert's pieces), an initiation-set sampler and an
event-based termination (Silver et al., 2022: skills are valid only where they can start and end on
events, not after a fixed duration). Skill data is collected in the two-sphere environment with the
state placed directly in the initiation set and DART noise (Laskey et al., 2017), so that one small
world model per skill (``SwarmWorldModel`` on that skill's data only) can be trained; the start
configuration, duration and success of every episode feed the skill effect models (Liang et al.,
2022: per-skill models of the outcome, composed by a planner).
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
import torch
from numpy.typing import NDArray

from modular_sphere.swarm import (
    ACT_DIM,
    STEP_H,
    STEP_X,
    SphereSwarmStepEnv,
    SwarmClimbPolicy,
)
from modular_sphere.swarm_data import SwarmDataset
from modular_sphere.swarm_torch import SwarmWorldModel

Array = NDArray[np.float64]
SKILLS = ("roll", "climb", "pull")
MAX_SKILL_STEPS = 200  # 10 s: a skill that has not ended by then failed
ROLL_TOL = 0.03  # roll ends within this distance of the target [m] ...
ROLL_SPEED = 0.05  # ... below this speed [m/s]


def place(env: SphereSwarmStepEnv, pos: Array) -> None:
    """Put the spheres at rest at ``pos`` (N, 3), unlinked."""
    env.data.qvel[:] = 0.0
    for i, p in enumerate(pos):
        env.data.qpos[env._qpos[i] : env._qpos[i] + 3] = p
    env.data.eq_active[:] = 0
    env._sync_link_visibility()
    mujoco.mj_forward(env.model, env.data)
    env._prev_dist = env._dist()


@dataclass
class SkillCall:
    """One skill applied to spheres ``agents`` (roll: ``(i,)``; climb: ``(c, b)``; pull: ``(h, b)``)
    with an xy ``target`` (roll target; climb/pull: the line ``y`` of the base in ``target[1]``)."""

    name: str
    agents: tuple[int, ...]
    target: Array


class SkillController:
    """Scripted controller of one skill call, from the ``SwarmClimbPolicy`` building blocks."""

    def __init__(self, call: SkillCall, n_spheres: int, radius: float = 0.1) -> None:
        self.call, self.r = call, radius
        self.ex = SwarmClimbPolicy(n_spheres, radius)
        self.sl = self.ex.slices
        self.lifting = False
        y = float(call.target[1])
        self.base_xy = np.array([STEP_X - radius - 0.01, y])
        self.climb_xy = self.base_xy - (2 * radius + 0.03, 0.0)
        self.edge_xy = np.array([self.ex.EDGE_X, y])

    def done(self, pos: Array, vel: Array) -> bool:
        """Termination event from the true sphere positions and velocities (N, 3)."""
        name, ag = self.call.name, self.call.agents
        if name == "roll":
            i = ag[0]
            near = np.linalg.norm(pos[i, :2] - self.call.target) < ROLL_TOL
            return bool(near and np.linalg.norm(vel[i, :2]) < ROLL_SPEED)
        if name == "climb":
            return bool(pos[ag[0], 2] > 3 * self.r - self.ex.TOP_MARGIN)
        h, b = ag
        return bool(pos[b, 2] > pos[h, 2] + self.r and pos[b, 0] > pos[h, 0] - 0.05)

    def act(self, obs: list[Array]) -> dict[int, Array]:
        """Actions of the involved spheres from their local observations (index -> obs)."""
        name, ag, ex = self.call.name, self.call.agents, self.ex
        out = {i: np.zeros(ACT_DIM) for i in ag}
        if name == "roll":
            out[ag[0]][:2] = ex._track(obs[ag[0]], self.call.target)
            return out
        if name == "climb":
            c, b = ag
            oc, ob = obs[c], obs[b]
            if not self.lifting:
                base_ready = np.linalg.norm(ob[:2] - self.base_xy) < ex.ALIGN_TOL
                slot = self.climb_xy if base_ready else self.climb_xy - (ex.STAGE, 0.0)
                out[b][:2] = ex._track(ob, self.base_xy)
                out[c][:2] = ex._track(oc, slot)
                ready = base_ready and np.linalg.norm(oc[:2] - self.climb_xy) < ex.ALIGN_TOL
                speed = max(np.linalg.norm(ob[3:5]), np.linalg.norm(oc[3:5]))
                self.lifting = bool(ready and speed < 0.1)
            if self.lifting:
                out[b][:] = (1.0, 0.0, 0.0, 1.0)
                out[c][:] = ex._lift(ob, oc)
            return out
        h, b = ag
        oh, ob = obs[h], obs[b]
        if not self.lifting:
            out[b][:2] = ex._track(ob, self.base_xy)
            out[h][:2] = ex._track(oh, self.edge_xy, vmax=0.3)
            ready = (
                np.linalg.norm(ob[:2] - self.base_xy) < ex.ALIGN_TOL
                and np.linalg.norm(oh[:2] - self.edge_xy) < ex.ALIGN_TOL
            )
            speed = max(np.linalg.norm(ob[3:5]), np.linalg.norm(oh[3:5]))
            self.lifting = bool(ready and speed < 0.1)
        if self.lifting and not ex._linked(oh, h, b):
            out[h][:2] = ex._track(oh, self.edge_xy, vmax=0.3)  # hold the edge, link first
            out[b][:] = (1.0, 0.0, 0.0, 1.0)
            out[h][3] = 1.0
        elif self.lifting:
            out[h][:] = (1.0, 0.0, 0.0, 1.0)
            out[b][:] = ex._lift(oh, ob)
        return out


def sample_start(name: str, rng: np.random.Generator, radius: float = 0.1) -> tuple[Array, Array]:
    """Random two-sphere start in the skill's initiation set: positions (2, 3), xy target.

    Agents are ``(0,)`` for roll (sphere 1 is a passive bystander) and ``(0, 1)`` otherwise."""
    r, floor, top = radius, radius, STEP_H + radius
    if name == "roll":
        on_top = rng.random() < 0.4
        x_lo, x_hi = (STEP_X + r + 0.02, 1.8) if on_top else (-1.8, STEP_X - r - 0.02)
        z = top if on_top else floor
        while True:
            p = rng.uniform((x_lo, -1.8), (x_hi, 1.8), (3, 2))  # sphere, bystander, target
            if min(np.linalg.norm(p[0] - p[1]), np.linalg.norm(p[2] - p[1])) > 2 * r + 0.1:
                break
        pos = np.array([[*p[0], z], [*p[1], z]])
        return pos, p[2]
    y = rng.uniform(-0.9, 0.9)
    if name == "climb":
        base = (STEP_X - r - rng.uniform(0.01, 0.3), y + rng.uniform(-0.3, 0.3), floor)
        climber = (base[0] - rng.uniform(0.25, 1.2), y + rng.uniform(-0.6, 0.6), floor)
        return np.array([climber, base]), np.array([0.0, y])
    helper = (STEP_X + rng.uniform(0.03, 0.4), y + rng.uniform(-0.3, 0.3), top)
    base = (STEP_X - r - rng.uniform(0.01, 0.3), y + rng.uniform(-0.3, 0.3), floor)
    return np.array([helper, base]), np.array([0.0, y])


def skill_cost(call: SkillCall, x: torch.Tensor, radius: float = 0.1) -> torch.Tensor:
    """Cost of predicted sphere states ``x`` (..., N, 9) for a skill call (lower is better)."""
    ag, y = call.agents, float(call.target[1])
    pos, vel = x[..., :3], x[..., 3:6]
    face = torch.tensor([STEP_X - radius - 0.01, y], dtype=x.dtype, device=x.device)
    if call.name == "roll":
        tgt = torch.as_tensor(call.target, dtype=x.dtype, device=x.device)
        i = ag[0]
        return (pos[..., i, :2] - tgt).norm(dim=-1) + 0.1 * vel[..., i, :2].norm(dim=-1)
    if call.name == "climb":
        c, b = ag
        top = torch.tensor([STEP_X + 0.05, y, 3 * radius], dtype=x.dtype, device=x.device)
        return (pos[..., c, :] - top).norm(dim=-1) + (pos[..., b, :2] - face).norm(dim=-1)
    h, b = ag
    up = torch.tensor([STEP_X + 0.15, y, STEP_H + radius], dtype=x.dtype, device=x.device)
    edge = torch.tensor([STEP_X + 0.035, y], dtype=x.dtype, device=x.device)
    return (pos[..., b, :] - up).norm(dim=-1) + (pos[..., h, :2] - edge).norm(dim=-1)


def skill_features(call: SkillCall, x: torch.Tensor, links: torch.Tensor) -> torch.Tensor:
    """Target-relative state of the involved spheres (..., F) from states (..., N, 9), links."""
    ag = list(call.agents)
    if call.name == "roll":
        ref = (float(call.target[0]), float(call.target[1]), 0.0)
    else:
        ref = (STEP_X, float(call.target[1]), 0.0)
    ref_t = torch.tensor([*ref, *([0.0] * 6)], dtype=x.dtype, device=x.device)
    feats = [(x[..., ag, :] - ref_t).flatten(-2)]
    if len(ag) == 2:
        feats.append(links[..., ag[0], ag[1], None].to(x.dtype))
    return torch.cat(feats, dim=-1)


class _SkillNet(torch.nn.Module):
    """MLP on normalised ``skill_features`` with ``out_dim`` outputs, fitted by ``_fit``."""

    mean: torch.Tensor
    std: torch.Tensor

    def __init__(self, n_agents: int, out_dim: int, hidden: int = 128) -> None:
        super().__init__()
        self.n_agents, self.hidden = n_agents, hidden
        dim = 9 * n_agents + (n_agents == 2)
        self.net = torch.nn.Sequential(
            torch.nn.Linear(dim, hidden), torch.nn.SiLU(), torch.nn.Linear(hidden, hidden),
            torch.nn.SiLU(), torch.nn.Linear(hidden, out_dim),
        )  # fmt: skip
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("std", torch.ones(dim))

    def _raw(self, feats: torch.Tensor) -> torch.Tensor:
        return self.net((feats - self.mean) / self.std)

    def _loss(self, out: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError

    def _fit(self, f: torch.Tensor, y: torch.Tensor, epochs: int, lr: float, batch_size: int,
             seed: int) -> list[float]:  # fmt: skip
        torch.manual_seed(seed)
        self.mean.copy_(f.mean(0))
        self.std.copy_(f.std(0) + 1e-6)
        optim = torch.optim.AdamW(self.net.parameters(), lr=lr, weight_decay=1e-5)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(optim, T_max=epochs)
        gen = torch.Generator(device="cpu").manual_seed(seed)
        losses = []
        for _ in range(epochs):
            total = 0.0
            for idx in torch.randperm(f.shape[0], generator=gen).to(f.device).split(batch_size):
                loss = self._loss(self(f[idx]), y[idx])
                optim.zero_grad(set_to_none=True)
                loss.backward()
                optim.step()
                total += loss.item() * idx.numel()
            sched.step()
            losses.append(total / f.shape[0])
        return losses

    def save(self, path: str) -> None:
        torch.save({"n_agents": self.n_agents, "hidden": self.hidden,
                    "state": self.state_dict()}, path)  # fmt: skip

    @classmethod
    def load(cls, path: str, device: str = "cpu") -> _SkillNet:
        ckpt = torch.load(path, map_location=device, weights_only=True)
        net = cls(ckpt["n_agents"], hidden=ckpt["hidden"]).to(device)  # type: ignore[call-arg]
        net.load_state_dict(ckpt["state"])
        return net


def _episode_tensors(data: SwarmDataset, calls: list[SkillCall], e: int, device: torch.device
                     ) -> tuple[torch.Tensor, NDArray[np.bool_]]:  # fmt: skip
    v = data.valid[e]
    x = torch.as_tensor(data.spheres[e, :-1][v], dtype=torch.float32, device=device)
    lk = torch.as_tensor(data.links[e, :-1][v], device=device)
    return skill_features(calls[e], x, lk), v


class SkillPrior(_SkillNet):
    """Behaviour cloning of a skill controller on target-relative states, used as the MPPI
    proposal (TD-MPC2, MoDem: policy samples next to the Gaussian ones). Output (..., A, 4):
    tanh(fx, fy, lift) and the attach logit of each involved sphere."""

    def __init__(self, n_agents: int, hidden: int = 128) -> None:
        super().__init__(n_agents, n_agents * ACT_DIM, hidden)

    def forward(self, feats: torch.Tensor) -> torch.Tensor:
        out = self._raw(feats).unflatten(-1, (self.n_agents, ACT_DIM))
        return torch.cat([torch.tanh(out[..., :3]), out[..., 3:]], dim=-1)

    def _loss(self, out: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        mse = ((out[..., :3] - y[..., :3]) ** 2).mean()
        bce = torch.nn.functional.binary_cross_entropy_with_logits(
            out[..., 3], (y[..., 3] > 0).float()
        )
        return mse + bce

    def fit(self, data: SwarmDataset, calls: list[SkillCall], epochs: int = 30,
            lr: float = 1e-3, batch_size: int = 1024, seed: int = 0) -> list[float]:  # fmt: skip
        """Fit on the expert labels of every valid step; ``calls[e]`` is episode ``e``'s call."""
        feats, labels = [], []
        for e, call in enumerate(calls):
            f, v = _episode_tensors(data, calls, e, self.mean.device)
            feats.append(f)
            labels.append(torch.as_tensor(data.expert[e][v][:, list(call.agents)],
                                          dtype=torch.float32, device=f.device))  # fmt: skip
        return self._fit(torch.cat(feats), torch.cat(labels), epochs, lr, batch_size, seed)


class SkillValue(_SkillNet):
    """Steps to go until the skill's termination event (Monte Carlo targets from the skill data;
    failed episodes count as ``MAX_SKILL_STEPS`` to go). It is the planner's terminal value
    (TD-MPC) and the duration part of the skill effect model (Liang et al., 2022)."""

    def __init__(self, n_agents: int, hidden: int = 128) -> None:
        super().__init__(n_agents, 1, hidden)

    def forward(self, feats: torch.Tensor) -> torch.Tensor:
        """Predicted steps to go (...,), in [0, MAX_SKILL_STEPS]."""
        return MAX_SKILL_STEPS * torch.sigmoid(self._raw(feats)[..., 0])

    def _loss(self, out: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        return (((out - y) / MAX_SKILL_STEPS) ** 2).mean()

    def fit(self, data: SwarmDataset, calls: list[SkillCall], epochs: int = 30,
            lr: float = 1e-3, batch_size: int = 1024, seed: int = 0) -> list[float]:  # fmt: skip
        """Fit on every valid step of every episode; ``calls[e]`` is episode ``e``'s call."""
        feats, targets = [], []
        for e in range(len(calls)):
            f, _ = _episode_tensors(data, calls, e, self.mean.device)
            n = f.shape[0]
            togo = n - torch.arange(n, device=f.device, dtype=torch.float32)
            feats.append(f)
            targets.append(togo if data.success[e] else torch.full_like(togo, MAX_SKILL_STEPS))
        return self._fit(torch.cat(feats), torch.cat(targets), epochs, lr, batch_size, seed)


class SkillPlanner:
    """MPPI (Williams et al., 2017) in a skill world model: the model, not a policy, picks the
    actions of the involved spheres (others get zero actions). The attach channel is sampled as a
    continuous value and executed as its sign; plans are warm-started by shifting (TD-MPC). With a
    ``prior``, a fraction ``prior_frac`` of the samples are prior rollouts in the model with noise
    ``prior_std`` (TD-MPC2); the model's predicted cost still selects among all samples. With a
    ``value``, the cost is the mean predicted steps to go along the plan instead of
    ``skill_cost``; the temperature is then in steps."""

    def __init__(self, model: SwarmWorldModel, call: SkillCall, n_spheres: int,
                 horizon: int = 15, samples: int = 256, iterations: int = 2, std: float = 0.5,
                 temperature: float = 0.05, seed: int = 0, prior: SkillPrior | None = None,
                 prior_frac: float = 0.25, prior_std: float = 0.1,
                 value: SkillValue | None = None) -> None:  # fmt: skip
        self.model, self.call, self.n = model, call, n_spheres
        self.h, self.k, self.iters = horizon, samples, iterations
        self.std, self.temp = std, temperature
        self.ag = list(call.agents)
        self.mean = torch.zeros(horizon, len(self.ag), ACT_DIM, device=model.device)
        self.gen = torch.Generator(device=model.device).manual_seed(seed)
        self.prior, self.prior_std = prior, prior_std
        self.k_prior = round(prior_frac * samples) if prior is not None else 0
        self.value = value

    def _full(self, plans: torch.Tensor) -> torch.Tensor:
        full = torch.zeros(*plans.shape[:2], self.n, ACT_DIM, device=plans.device)
        full[:, :, self.ag] = plans
        full[..., 3] = torch.where(full[..., 3] > 0, 1.0, -1.0)
        return full

    def _prior_plans(self, x: torch.Tensor, lk: torch.Tensor) -> torch.Tensor:
        """Noisy prior rollouts in the model: plans (K, H, A, 4)."""
        assert self.prior is not None
        plans = []
        for _ in range(self.h):
            a = self.prior(skill_features(self.call, x, lk))
            a = a.clone()
            a[..., 3] = torch.tanh(a[..., 3])  # logit -> [-1, 1] like the Gaussian samples
            noise = torch.randn(a.shape, generator=self.gen, device=a.device) * self.prior_std
            a = (a + noise).clamp(-1.0, 1.0)
            plans.append(a)
            x, lk = self.model.step(x, lk, self._full(a[:, None])[:, 0])
        return torch.stack(plans, 1)

    @torch.no_grad()
    def act(self, spheres: Array, links: NDArray[np.bool_]) -> Array:
        """Actions (N, 4) from the true sphere states (N, 9) and links (N, N)."""
        dev, t = self.model.device, self.model._t
        x0, l0 = t(spheres)[None].expand(self.k, -1, -1), t(links)[None].expand(self.k, -1, -1)
        mean = self.mean
        pri = self._prior_plans(x0[: self.k_prior], l0[: self.k_prior]) if self.k_prior else None
        for _ in range(self.iters):
            noise = torch.randn(self.k, *mean.shape, generator=self.gen, device=dev) * self.std
            plans = (mean[None] + noise).clamp(-1.0, 1.0)
            plans[0] = mean  # keep the current plan as a candidate
            if pri is not None:
                plans[1 : 1 + self.k_prior] = pri
            full = self._full(plans)
            x, lk, cost = x0, l0, torch.zeros(self.k, device=dev)
            for s in range(self.h):
                x, lk = self.model.step(x, lk, full[:, s])
                if self.value is None:
                    cost = cost + skill_cost(self.call, x)
                else:  # mean predicted steps to go along the plan
                    cost = cost + self.value(skill_features(self.call, x, lk)) / self.h
            w = torch.softmax(-(cost - cost.min()) / (self.temp * self.h), dim=0)
            mean = (w[:, None, None, None] * plans).sum(0)
        self.mean = torch.cat([mean[1:], mean[-1:]])  # shift for the next control step
        out = np.zeros((self.n, ACT_DIM))
        out[self.ag] = mean[0].cpu().numpy()
        out[:, 3] = np.where(out[:, 3] > 0, 1.0, -1.0)
        return out


@dataclass
class SkillData:
    """Two-sphere transitions of one skill (``data``) and per-episode outcomes for effect models:
    ``start`` (E, 2, 3) positions, ``target`` (E, 2), ``steps`` (E,), ``success`` (E,)."""

    data: SwarmDataset
    start: Array
    target: Array
    steps: NDArray[np.int64]
    success: NDArray[np.bool_]


def collect_skill(name: str, n_episodes: int, seed: int = 0, noise: float = 0.2) -> SkillData:
    """Run the skill controller from sampled starts with DART noise on fx, fy, lift."""
    if name not in SKILLS:
        raise ValueError(f"unknown skill {name!r}, expected one of {SKILLS}")
    env = SphereSwarmStepEnv(n_spheres=2, max_steps=MAX_SKILL_STEPS + 1)
    agents, horizon, rng = env.possible_agents, MAX_SKILL_STEPS, np.random.default_rng(seed)
    ag = (0,) if name == "roll" else (0, 1)
    obs_dim = env.slices["others"].stop
    out = {
        "spheres": np.zeros((n_episodes, horizon + 1, 2, 9)),
        "links": np.zeros((n_episodes, horizon + 1, 2, 2), dtype=bool),
        "obs": np.zeros((n_episodes, horizon + 1, 2, obs_dim)),
        "actions": np.zeros((n_episodes, horizon, 2, ACT_DIM)),
        "expert": np.zeros((n_episodes, horizon, 2, ACT_DIM)),
        "rewards": np.zeros((n_episodes, horizon)),
        "valid": np.zeros((n_episodes, horizon), dtype=bool),
        "goal": np.zeros((n_episodes, 3)),
        "success": np.zeros(n_episodes, dtype=bool),
    }
    starts, targets = np.zeros((n_episodes, 2, 3)), np.zeros((n_episodes, 2))
    steps = np.zeros(n_episodes, dtype=np.int64)

    def record(e: int, t: int, obs: dict[str, Array]) -> None:
        out["spheres"][e, t:] = np.concatenate(env._kinematics(), axis=1)
        out["links"][e, t:] = env.links
        out["obs"][e, t:] = np.stack([obs[a] for a in agents])

    for e in range(n_episodes):
        env.reset(seed=seed + e)
        starts[e], targets[e] = sample_start(name, rng, env.radius)
        place(env, starts[e])
        env.data.mocap_pos[env._goal_mocap][:2] = (10.0, 10.0)  # no task success mid-skill
        ctrl = SkillController(SkillCall(name, ag, targets[e]), 2, env.radius)
        obs = env._observations()
        record(e, 0, obs)
        for t in range(horizon):
            label = np.zeros((2, ACT_DIM))
            for i, a in ctrl.act([obs[a] for a in agents]).items():
                label[i] = a
            executed = label.copy()
            executed[list(ag), :3] += noise * rng.standard_normal((len(ag), 3))
            executed = np.clip(executed, -1.0, 1.0)
            obs, *_ = env.step(dict(zip(agents, executed, strict=True)))
            out["actions"][e, t], out["expert"][e, t] = executed, label
            out["valid"][e, t] = True
            record(e, t + 1, obs)
            pos, vel = env._kinematics()[:2]
            if ctrl.done(pos, vel):
                out["success"][e] = True
                break
        steps[e] = t + 1
    return SkillData(SwarmDataset(**out), starts, targets, steps, out["success"].copy())
