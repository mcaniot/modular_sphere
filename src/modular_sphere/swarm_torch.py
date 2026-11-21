"""Learned swarm control: graph world model and per-agent policy (optional extra ``torch``).

The scripted swarm approach is learned in three stages (data from :mod:`modular_sphere.swarm_data`):

1. ``SwarmWorldModel``: interaction network over the spheres (Battaglia et al., 2016; GNS,
   Sanchez-Gonzalez et al., 2020). Nodes carry pos, vel, ang_vel, the clipped distances to the
   walls and the step (GNS boundary features; without them the model lets spheres drive through
   the step) and the sphere action; directed edges carry the relative position and velocity,
   the link flag and the sender action. A message
   MLP summed per receiver feeds a node MLP that predicts the normalized change of the sphere
   state; an edge MLP predicts the next link logits. Trained on unrolled multi-step windows (as
   GNS training noise, it limits compounding error). The reward is not learned: it is computed
   from the predicted state with the task definition (deviation from Dreamer's reward head).
2. ``SwarmPolicy``: one network shared by all agents on their local observation (decentralised
   execution): the other spheres form a set encoded by a shared MLP and mean/max pooled
   (Deep Sets, Zaheer et al., 2017). Trained by behaviour cloning of the expert labels, iterated
   with DAgger (``swarm_data.collect(policy=...)``).
3. Improvement in imagination (``improve_in_imagination``): Dreamer-style actor-critic
   (Hafner et al., 2023) on lambda-returns of world-model rollouts started from dataset states,
   with a behaviour-cloning regulariser towards the expert (as TD3+BC, Fujimoto & Gu, 2021).
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass

import numpy as np
import torch
from numpy.typing import NDArray
from torch import nn

from modular_sphere.env import STEP_PENALTY, SUCCESS_BONUS
from modular_sphere.swarm import (
    ACT_DIM,
    STEP_H,
    STEP_X,
    SphereSwarmStepEnv,
    gather_radius,
    obs_slices,
)
from modular_sphere.swarm_data import SwarmDataset

Array = NDArray[np.float64]
EPS = 1e-6
SPHERE_DIM = 9  # pos, vel, ang_vel
ARENA_HALF = 2.0  # inner faces of the arena walls (sphere_arena.xml) [m]
BOUNDARY_RANGE = 0.3  # boundary distances are clipped to this range [m] (GNS)
BOUNDARY_DIM = 6


def _mlp(d_in: int, d_out: int, hidden: int, layers: int = 2) -> nn.Sequential:
    dims = [d_in] + [hidden] * layers
    blocks: list[nn.Module] = []
    for a, b in itertools.pairwise(dims):
        blocks += [nn.Linear(a, b), nn.SiLU()]
    return nn.Sequential(*blocks, nn.Linear(hidden, d_out))


def _device(device: str | None) -> torch.device:
    return torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))


def _boundary(pos: torch.Tensor) -> torch.Tensor:
    """Clipped distances to the static geometry, as GNS node features: the four arena walls, the
    step face and the step top (signed: negative once past them), in units of the clip range."""
    x, y, z = pos.unbind(-1)
    d = torch.stack(
        [ARENA_HALF - x, x + ARENA_HALF, ARENA_HALF - y, y + ARENA_HALF, STEP_X - x, z - STEP_H],
        dim=-1,
    )
    return d.clamp(-BOUNDARY_RANGE, BOUNDARY_RANGE) / BOUNDARY_RANGE


class _InteractionNet(nn.Module):
    def __init__(self, hidden: int) -> None:
        super().__init__()
        node_in, edge_in = SPHERE_DIM + BOUNDARY_DIM + ACT_DIM, 3 + 1 + 3 + 1 + ACT_DIM + ACT_DIM
        self.message = _mlp(edge_in, hidden, hidden)
        self.node = _mlp(node_in + hidden, SPHERE_DIM, hidden)
        self.link = _mlp(edge_in, 1, hidden)

    def forward(
        self,
        x: torch.Tensor,
        links: torch.Tensor,
        act: torch.Tensor,
        stats: dict[str, torch.Tensor],
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """``x`` (B, N, 9) raw sphere state, ``links`` (B, N, N) in [0, 1], ``act`` (B, N, 4).

        Returns the normalized state change (B, N, 9) and the next link logits (B, N, N)."""
        n = x.shape[1]
        pos, vel = x[..., :3], x[..., 3:6]
        rel_p = pos[:, None, :, :] - pos[:, :, None, :]  # [b, i, j] = p_j - p_i
        rel_v = vel[:, None, :, :] - vel[:, :, None, :]
        dist = rel_p.norm(dim=-1, keepdim=True)
        a_i = act[:, :, None, :].expand(-1, n, n, -1)
        a_j = act[:, None, :, :].expand(-1, n, n, -1)
        edge = torch.cat(
            [rel_p / 0.2, dist / 0.2, rel_v, links[..., None], a_i, a_j], dim=-1
        )  # distances in sphere diameters
        mask = 1.0 - torch.eye(n, device=x.device)
        messages = (self.message(edge) * mask[None, :, :, None]).sum(dim=2)
        node = torch.cat([(x - stats["x_mean"]) / stats["x_std"], _boundary(pos), act], dim=-1)
        delta = self.node(torch.cat([node, messages], dim=-1))
        logits = self.link(edge).squeeze(-1)
        logits = 0.5 * (logits + logits.transpose(1, 2))  # links are symmetric
        return delta, logits.masked_fill(mask == 0, -20.0)


class SwarmWorldModel:
    """Learned swarm dynamics: ``rollout(spheres0, links0, actions) -> (spheres, links)``."""

    def __init__(self, hidden: int = 128, epochs: int = 30, window: int = 8,
                 batch_size: int = 512, lr: float = 1e-3, seed: int = 0,
                 device: str | None = None) -> None:  # fmt: skip
        self.hidden, self.epochs, self.window = hidden, epochs, window
        self.batch_size, self.lr, self.seed = batch_size, lr, seed
        self.device = _device(device)
        self.net = _InteractionNet(hidden).to(self.device)
        self.stats: dict[str, torch.Tensor] = {}
        self.train_losses: list[float] = []

    def _t(self, x: Array | NDArray[np.bool_]) -> torch.Tensor:
        return torch.as_tensor(np.asarray(x, dtype=np.float32), device=self.device)

    def step(
        self, x: torch.Tensor, links: torch.Tensor, act: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """One differentiable step: next raw state and next link probabilities."""
        delta, logits = self.net(x, links, act, self.stats)
        return x + delta * self.stats["d_std"] + self.stats["d_mean"], torch.sigmoid(logits)

    def fit(self, data: SwarmDataset) -> None:
        """Train on ``data``; a second call continues training with the first normalization."""
        torch.manual_seed(self.seed)
        sph, links, act = self._t(data.spheres), self._t(data.links), self._t(data.actions)
        valid = torch.as_tensor(data.valid, device=self.device)
        x_flat, d_flat = sph[:, :-1][valid], (sph[:, 1:] - sph[:, :-1])[valid]
        if not self.stats:
            self.stats = {
                "x_mean": x_flat.mean((0, 1)),
                "x_std": x_flat.std((0, 1)) + EPS,
                "d_mean": d_flat.mean((0, 1)),
                "d_std": d_flat.std((0, 1)) + EPS,
            }
        h = self.window
        e_idx, t_idx = torch.nonzero(valid[:, : valid.shape[1] - h + 1], as_tuple=True)
        optim = torch.optim.AdamW(self.net.parameters(), lr=self.lr, weight_decay=1e-5)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(optim, T_max=self.epochs)
        gen = torch.Generator(device="cpu").manual_seed(self.seed)
        steps = torch.arange(h + 1, device=self.device)
        self.train_losses = []
        for _ in range(self.epochs):
            perm = torch.randperm(e_idx.numel(), generator=gen).to(self.device)
            total = 0.0
            for batch in perm.split(self.batch_size):
                e, t = e_idx[batch, None], t_idx[batch, None] + steps
                xs, ls, acts = sph[e, t], links[e, t], act[e, t[:, :-1]]
                w = valid[e, t[:, :-1]].float()  # padded steps after the end do not count
                x, lk, loss = xs[:, 0], ls[:, 0], torch.zeros((), device=self.device)
                for k in range(h):
                    delta, logits = self.net(x, lk, acts[:, k], self.stats)
                    target = (xs[:, k + 1] - x - self.stats["d_mean"]) / self.stats["d_std"]
                    mse = ((delta - target) ** 2).mean((1, 2))
                    bce = nn.functional.binary_cross_entropy_with_logits(
                        logits, ls[:, k + 1], reduction="none"
                    ).mean((1, 2))
                    loss = loss + ((mse + bce) * w[:, k]).sum() / w[:, k].sum().clamp(min=1)
                    x = x + delta * self.stats["d_std"] + self.stats["d_mean"]
                    lk = torch.sigmoid(logits)
                loss = loss / h
                optim.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(self.net.parameters(), 10.0)
                optim.step()
                total += loss.item() * batch.numel()
            sched.step()
            self.train_losses.append(total / e_idx.numel())

    @torch.no_grad()
    def rollout(
        self, spheres0: Array, links0: NDArray[np.bool_], actions: Array
    ) -> tuple[Array, Array]:
        """``spheres0`` (B, N, 9), ``links0`` (B, N, N), ``actions`` (B, H, N, 4) ->
        predicted spheres (B, H, N, 9) and link probabilities (B, H, N, N)."""
        x, lk, acts = self._t(spheres0), self._t(links0), self._t(actions)
        xs, ls = [], []
        for k in range(acts.shape[1]):
            x, lk = self.step(x, lk, acts[:, k])
            xs.append(x)
            ls.append(lk)
        return torch.stack(xs, 1).double().cpu().numpy(), torch.stack(ls, 1).double().cpu().numpy()

    def save(self, path: str) -> None:
        torch.save(
            {"hidden": self.hidden, "state": self.net.state_dict(), "stats": self.stats}, path
        )

    @classmethod
    def load(cls, path: str, device: str | None = None) -> SwarmWorldModel:
        ckpt = torch.load(path, map_location=_device(device), weights_only=True)
        model = cls(hidden=ckpt["hidden"], device=device)
        model.net.load_state_dict(ckpt["state"])
        model.stats = {k: v.to(model.device) for k, v in ckpt["stats"].items()}
        return model


def evaluate_world_model(
    model: SwarmWorldModel, data: SwarmDataset, horizons: list[int], stride: int = 20
) -> dict[int, dict[str, float]]:
    """Open-loop RMSE per horizon (pos [m], vel [m/s], ang_vel [rad/s]) and link accuracy, on
    fully valid windows; ``const_vel_pos`` is the constant-velocity baseline position RMSE."""
    h = max(horizons)
    dt = 0.05
    windows = [
        (e, t)
        for e in range(data.valid.shape[0])
        for t in range(0, data.valid.shape[1] - h + 1, stride)
        if data.valid[e, t : t + h].all()
    ]
    e, t = np.array(windows).T
    steps = t[:, None] + np.arange(1, h + 1)
    pred, links = model.rollout(
        data.spheres[e, t], data.links[e, t], data.actions[e[:, None], t[:, None] + np.arange(h)]
    )
    target, target_links = data.spheres[e[:, None], steps], data.links[e[:, None], steps]
    x0 = data.spheres[e, t]
    out = {}
    for k in sorted(horizons):
        err = pred[:, k - 1] - target[:, k - 1]
        cv = x0[..., :3] + k * dt * x0[..., 3:6] - target[:, k - 1, ..., :3]
        out[k] = {
            "pos": float(np.sqrt((err[..., :3] ** 2).mean())),
            "vel": float(np.sqrt((err[..., 3:6] ** 2).mean())),
            "ang_vel": float(np.sqrt((err[..., 6:] ** 2).mean())),
            "link_acc": float(((links[:, k - 1] > 0.5) == target_links[:, k - 1]).mean()),
            "const_vel_pos": float(np.sqrt((cv**2).mean())),
        }
    return out


class _PolicyNet(nn.Module):
    def __init__(self, n_spheres: int, hidden: int) -> None:
        super().__init__()
        self.sl = obs_slices(n_spheres)
        self.n = n_spheres
        self.other = _mlp(12 + 7, hidden, hidden)
        self.head = _mlp(12 + 2 * hidden, ACT_DIM, hidden)

    def forward(self, obs: torch.Tensor, stats: dict[str, torch.Tensor]) -> torch.Tensor:
        """``obs`` (..., obs_dim) -> (..., 4): tanh(fx, fy, lift) and the attach logit."""
        obs = (obs - stats["mean"]) / stats["std"]
        own = obs[..., :12]
        others = obs[..., self.sl["others"]].unflatten(-1, (self.n - 1, 7))
        tokens = torch.cat([own[..., None, :].expand(*others.shape[:-1], 12), others], dim=-1)
        enc = self.other(tokens)
        out = self.head(torch.cat([own, enc.mean(-2), enc.amax(-2)], dim=-1))
        return torch.cat([torch.tanh(out[..., :3]), out[..., 3:]], dim=-1)


class SwarmPolicy:
    """Shared decentralised policy: callable ``obs dict -> action dict`` (attach in {-1, 1})."""

    def __init__(self, n_spheres: int, hidden: int = 256, epochs: int = 20,
                 batch_size: int = 2048, lr: float = 1e-3, seed: int = 0,
                 device: str | None = None) -> None:  # fmt: skip
        self.n, self.hidden, self.epochs = n_spheres, hidden, epochs
        self.batch_size, self.lr, self.seed = batch_size, lr, seed
        self.device = _device(device)
        self.net = _PolicyNet(n_spheres, hidden).to(self.device)
        self.stats: dict[str, torch.Tensor] = {}
        self.train_losses: list[float] = []

    def _t(self, x: Array) -> torch.Tensor:
        return torch.as_tensor(np.asarray(x, dtype=np.float32), device=self.device)

    def loss(self, obs: torch.Tensor, expert: torch.Tensor) -> torch.Tensor:
        out = self.net(obs, self.stats)
        mse = ((out[..., :3] - expert[..., :3]) ** 2).mean(-1)
        bce = nn.functional.binary_cross_entropy_with_logits(
            out[..., 3], (expert[..., 3] > 0).float(), reduction="none"
        )
        return mse + bce

    def fit(self, data: SwarmDataset) -> None:
        """Behaviour cloning on every valid (local obs, expert label) pair."""
        torch.manual_seed(self.seed)
        valid = data.valid
        obs = self._t(data.obs[:, :-1][valid].reshape(-1, data.obs.shape[-1]))
        expert = self._t(data.expert[valid].reshape(-1, ACT_DIM))
        self.stats = {"mean": obs.mean(0), "std": obs.std(0) + EPS}
        optim = torch.optim.AdamW(self.net.parameters(), lr=self.lr, weight_decay=1e-5)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(optim, T_max=self.epochs)
        gen = torch.Generator(device="cpu").manual_seed(self.seed)
        self.train_losses = []
        for _ in range(self.epochs):
            perm = torch.randperm(obs.shape[0], generator=gen).to(self.device)
            total = 0.0
            for idx in perm.split(self.batch_size):
                loss = self.loss(obs[idx], expert[idx]).mean()
                optim.zero_grad(set_to_none=True)
                loss.backward()
                optim.step()
                total += loss.item() * idx.numel()
            sched.step()
            self.train_losses.append(total / obs.shape[0])

    @torch.no_grad()
    def __call__(self, obs: dict[str, Array]) -> dict[str, Array]:
        agents = list(obs)
        out = self.net(self._t(np.stack([obs[a] for a in agents])), self.stats).cpu().numpy()
        out[:, 3] = np.where(out[:, 3] > 0, 1.0, -1.0)
        return dict(zip(agents, out.astype(np.float64), strict=True))

    def save(self, path: str) -> None:
        torch.save({"n": self.n, "hidden": self.hidden, "state": self.net.state_dict(),
                    "stats": self.stats}, path)  # fmt: skip

    @classmethod
    def load(cls, path: str, device: str | None = None) -> SwarmPolicy:
        ckpt = torch.load(path, map_location=_device(device), weights_only=True)
        policy = cls(ckpt["n"], hidden=ckpt["hidden"], device=device)
        policy.net.load_state_dict(ckpt["state"])
        policy.stats = {k: v.to(policy.device) for k, v in ckpt["stats"].items()}
        return policy


@dataclass
class PolicyEval:
    success: NDArray[np.bool_]  # per seed
    steps: NDArray[np.int64]  # episode length per seed
    n_arrived: NDArray[np.int64]


def evaluate_policy(policy: SwarmPolicy, n_spheres: int, seeds: list[int]) -> PolicyEval:
    """Closed-loop evaluation in MuJoCo (ground truth), one episode per seed."""
    env = SphereSwarmStepEnv(n_spheres=n_spheres)
    success, steps, arrived = [], [], []
    for seed in seeds:
        obs, _ = env.reset(seed=seed)
        t, info = 0, {}
        while env.agents:
            obs, _, _, _, infos = env.step(policy(obs))
            info, t = infos[env.possible_agents[0]], t + 1
        success.append(info["is_success"])
        steps.append(t)
        arrived.append(info["n_arrived"])
    return PolicyEval(np.array(success), np.array(steps), np.array(arrived))


def task_reward(
    x: torch.Tensor, prev_x: torch.Tensor, goal: torch.Tensor, radius: float = 0.1
) -> tuple[torch.Tensor, torch.Tensor]:
    """Reward and success of ``SphereSwarmStepEnv`` from raw sphere states (B, N, 9)."""
    n = x.shape[1]
    mean_dist = (x[..., :3] - goal[:, None]).norm(dim=-1).mean(-1)
    prev = (prev_x[..., :3] - goal[:, None]).norm(dim=-1).mean(-1)
    in_zone = (x[..., :2] - goal[:, None, :2]).norm(dim=-1) <= gather_radius(n)
    success = (in_zone & (x[..., 2] > STEP_H + radius / 2)).all(-1).float()
    return prev - mean_dist - STEP_PENALTY + SUCCESS_BONUS * success, success


def local_obs(x: torch.Tensor, links: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
    """Torch twin of ``SphereSwarmStepEnv._observations``: (B, N, 9), (B, N, N), (B, 3) ->
    (B, N, obs_dim) local observations, others in cyclic order after each agent."""
    n = x.shape[1]
    idx = (torch.arange(n)[:, None] + torch.arange(1, n)[None, :]) % n  # (N, N-1)
    pos, vel = x[..., :3], x[..., 3:6]
    rel = torch.cat(
        [
            pos[:, idx] - pos[:, :, None],
            vel[:, idx] - vel[:, :, None],
            torch.gather(links, 2, idx[None].expand(x.shape[0], -1, -1).to(x.device))[..., None],
        ],
        dim=-1,
    )
    own = torch.cat([x, goal[:, None] - pos], dim=-1)
    return torch.cat([own, rel.flatten(2)], dim=-1)


class _Critic(nn.Module):
    """Value of the global state, invariant to the sphere order (Deep Sets)."""

    def __init__(self, hidden: int) -> None:
        super().__init__()
        self.sphere = _mlp(9 + 3 + 1, hidden, hidden)
        self.head = _mlp(2 * hidden, 1, hidden)

    def forward(self, x: torch.Tensor, links: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        feat = torch.cat([x, x[..., :3] - goal[:, None], links.sum(-1, keepdim=True)], dim=-1)
        enc = self.sphere(feat)
        return self.head(torch.cat([enc.mean(1), enc.amax(1)], dim=-1)).squeeze(-1)


def improve_in_imagination(
    policy: SwarmPolicy,
    model: SwarmWorldModel | list[SwarmWorldModel],
    data: SwarmDataset,
    iterations: int = 2000,
    horizon: int = 15,
    batch_size: int = 256,
    gamma: float = 0.99,
    lam: float = 0.95,
    bc_weight: float = 1.0,
    noise: float = 0.1,
    lr: float = 1e-4,
    uncertainty_weight: float = 100.0,
    critic_warmup: int = 200,
    value_weight: float = 1.0,
    seed: int = 0,
) -> list[dict[str, float]]:
    """Fine-tune ``policy`` in place on imagined rollouts of ``model`` (world models frozen).

    Actor gradients flow through the differentiable world model (Dreamer's dynamics
    backpropagation for continuous actions); the binary attach channel uses a straight-through
    sign. Each update adds the behaviour-cloning loss on a dataset batch of expert labels.
    With an ensemble, each imagined step follows one random member (MBPO, Janner et al., 2019) and
    the reward is penalised by ``uncertainty_weight`` times the members' disagreement, the largest
    std of the predicted sphere positions in metres (MOPO, Yu et al., 2020): without it the policy
    exploits model errors (imagined return up, MuJoCo success down). The first ``critic_warmup``
    iterations train only the critic, on rollouts of the unchanged policy. ``value_weight`` scales
    the critic bootstrap in the actor objective (0: pure model return over ``horizon``).
    """
    torch.manual_seed(seed)
    dev = policy.device
    models = model if isinstance(model, list) else [model]
    params = [p for m in models for p in m.net.parameters()]
    for p in params:
        p.requires_grad_(False)
    t = policy._t
    sph, links, goal = t(data.spheres), t(data.links), t(data.goal)
    valid = torch.as_tensor(data.valid, device=dev)
    obs_all = t(data.obs[:, :-1][data.valid].reshape(-1, data.obs.shape[-1]))
    exp_all = t(data.expert[data.valid].reshape(-1, ACT_DIM))
    e_idx, t_idx = torch.nonzero(valid, as_tuple=True)
    critic = _Critic(policy.hidden).to(dev)
    target = _Critic(policy.hidden).to(dev)
    target.load_state_dict(critic.state_dict())
    actor_opt = torch.optim.Adam(policy.net.parameters(), lr=lr)
    critic_opt = torch.optim.Adam(critic.parameters(), lr=3e-4)
    gen = torch.Generator(device="cpu").manual_seed(seed)
    log: list[dict[str, float]] = []
    for it in range(iterations):
        pick = torch.randint(e_idx.numel(), (batch_size,), generator=gen).to(dev)
        e, s = e_idx[pick], t_idx[pick]
        x, lk, g = sph[e, s], links[e, s], goal[e]
        xs, lks, rewards, conts = [x], [lk], [], []
        alive = torch.ones(batch_size, device=dev)
        spread = torch.zeros((), device=dev)
        for _ in range(horizon):
            out = policy.net(local_obs(x, lk, g), policy.stats)
            cont = (out[..., :3] + noise * torch.randn_like(out[..., :3])).clamp(-1, 1)
            soft = torch.tanh(out[..., 3:])
            attach = soft + (torch.sign(soft) - soft).detach()  # straight-through
            prev, act = x, torch.cat([cont, attach], dim=-1)
            preds = [m.step(x, lk, act) for m in models]
            x, lk = preds[int(torch.randint(len(preds), (1,), generator=gen))]
            r, done = task_reward(x, prev, g)
            if len(preds) > 1:
                pos = torch.stack([p[0][..., :3] for p in preds])
                u = pos.std(0).norm(dim=-1).amax(-1)
                r, spread = r - uncertainty_weight * u, spread + u.mean() / horizon
            rewards.append(r * alive)
            alive = alive * (1 - done)
            conts.append(alive)
            xs.append(x)
            lks.append(lk)
        with torch.no_grad():
            values_t = [target(xi, li, g) for xi, li in zip(xs, lks, strict=True)]
        values = [critic(xi.detach(), li.detach(), g) for xi, li in zip(xs, lks, strict=True)]
        ret = values_t[-1]
        returns = []
        for k in reversed(range(horizon)):  # TD(lambda) through the imagined trajectory
            ret = rewards[k] + gamma * conts[k] * ((1 - lam) * values_t[k + 1] + lam * ret)
            returns.append(ret)
        returns = returns[::-1]
        # Actor: maximise the discounted model return plus the critic bootstrap, differentiably
        # (rewards through the model) + BC.
        actor_ret = torch.zeros_like(values[-1])
        disc = 1.0
        for k in range(horizon):
            actor_ret = actor_ret + disc * rewards[k]
            disc *= gamma
        actor_ret = actor_ret + value_weight * disc * conts[-1] * target(xs[-1], lks[-1], g)
        bc_pick = torch.randint(obs_all.shape[0], (batch_size * 4,), generator=gen).to(dev)
        bc = policy.loss(obs_all[bc_pick], exp_all[bc_pick]).mean()
        actor_loss = -actor_ret.mean() + bc_weight * bc
        if it >= critic_warmup:  # an untrained critic's value gradients would steer the actor
            actor_opt.zero_grad(set_to_none=True)
            actor_loss.backward()
            nn.utils.clip_grad_norm_(policy.net.parameters(), 1.0)
            actor_opt.step()
        critic_loss = torch.stack(
            [((v - r.detach()) ** 2).mean() for v, r in zip(values[:-1], returns, strict=True)]
        ).mean()
        critic_opt.zero_grad(set_to_none=True)
        critic_loss.backward()
        critic_opt.step()
        with torch.no_grad():
            for p, q in zip(target.parameters(), critic.parameters(), strict=True):
                p.lerp_(q, 0.02)
        if it % 50 == 0:
            metrics = {"return": actor_ret.mean(), "bc": bc, "critic": critic_loss,
                       "uncertainty": spread}  # fmt: skip
            log.append({"it": it, **{k: float(v.detach()) for k, v in metrics.items()}})
    for p in params:
        p.requires_grad_(True)
    return log


def _ppo_dist(
    policy: SwarmPolicy, obs: torch.Tensor, log_std: torch.Tensor
) -> tuple[torch.distributions.Normal, torch.distributions.Bernoulli]:
    out = policy.net(obs, policy.stats)
    return (
        torch.distributions.Normal(out[..., :3], log_std.exp().expand_as(out[..., :3])),
        torch.distributions.Bernoulli(logits=out[..., 3]),
    )


def _bernoulli_kl(logits_p: torch.Tensor, logits_q: torch.Tensor) -> torch.Tensor:
    """KL(p || q) of Bernoullis from logits, finite for saturated probabilities."""
    log_p, log_q = nn.functional.logsigmoid(logits_p), nn.functional.logsigmoid(logits_q)
    log_np, log_nq = nn.functional.logsigmoid(-logits_p), nn.functional.logsigmoid(-logits_q)
    return log_p.exp() * (log_p - log_q) + log_np.exp() * (log_np - log_nq)


def _log_prob(
    dists: tuple[torch.distributions.Normal, torch.distributions.Bernoulli],
    cont: torch.Tensor,
    attach: torch.Tensor,
) -> torch.Tensor:
    """Joint log-probability of the swarm action (sum over agents and channels): (B,)."""
    return dists[0].log_prob(cont).sum((-1, -2)) + dists[1].log_prob(attach).sum(-1)


def improve_with_ppo(
    policy: SwarmPolicy,
    models: SwarmWorldModel | list[SwarmWorldModel],
    data: SwarmDataset,
    iterations: int = 200,
    horizon: int = 30,
    batch_size: int = 256,
    epochs: int = 4,
    minibatch: int = 1024,
    gamma: float = 0.99,
    lam: float = 0.95,
    clip: float = 0.2,
    kl_weight: float = 1.0,
    time_penalty: float = 0.05,
    uncertainty_weight: float = 100.0,
    log_std: float = -1.5,
    lr: float = 3e-5,
    seed: int = 0,
    anchor: SwarmPolicy | None = None,
) -> list[dict[str, float]]:
    """Fine-tune ``policy`` in place with PPO (Schulman et al., 2017) on world-model rollouts.

    Unlike ``improve_in_imagination`` no gradient goes through the model or into a critic's value
    of the actor's action, which removes the two exploits seen there (model contacts, critic
    overestimation). Execution stays decentralised; the team reward and a centralised critic on the
    global state follow MAPPO (Yu et al., 2022). The policy is Gaussian on fx, fy, lift around the
    network output (fixed std ``exp(log_std)``) and Bernoulli on attach; deployment uses the mean.
    The reward is the task reward minus ``time_penalty`` per step (speed objective) and the
    ensemble disagreement penalty (MOPO); a KL term keeps the policy near its initial (DAgger)
    behaviour (or near ``anchor``, the policy to stay close to over several calls), the trust
    region against the greedy short-horizon reward.
    """
    torch.manual_seed(seed)
    dev = policy.device
    models = models if isinstance(models, list) else [models]
    t = policy._t
    sph, links, goal = t(data.spheres), t(data.links), t(data.goal)
    e_idx, t_idx = torch.nonzero(torch.as_tensor(data.valid, device=dev), as_tuple=True)
    reference = _PolicyNet(policy.n, policy.hidden).to(dev)
    ref_policy = anchor or policy
    reference.load_state_dict(ref_policy.net.state_dict())
    std = torch.tensor(log_std, device=dev)
    critic = _Critic(policy.hidden).to(dev)
    actor_opt = torch.optim.Adam(policy.net.parameters(), lr=lr)
    critic_opt = torch.optim.Adam(critic.parameters(), lr=3e-4)
    gen = torch.Generator(device="cpu").manual_seed(seed)
    log: list[dict[str, float]] = []
    for it in range(iterations):
        pick = torch.randint(e_idx.numel(), (batch_size,), generator=gen).to(dev)
        x, lk, g = sph[e_idx[pick], t_idx[pick]], links[e_idx[pick], t_idx[pick]], goal[e_idx[pick]]
        buf: dict[str, list[torch.Tensor]] = {k: [] for k in ("x", "lk", "obs", "c", "a", "lp")}
        rewards, alives, success = [], [], torch.zeros(batch_size, device=dev)
        alive = torch.ones(batch_size, device=dev)
        with torch.no_grad():
            for _ in range(horizon):
                obs = local_obs(x, lk, g)
                dists = _ppo_dist(policy, obs, std)
                cont, attach = dists[0].sample(), dists[1].sample()
                step = (x, lk, obs, cont, attach, _log_prob(dists, cont, attach))
                for k, v in zip(buf, step, strict=True):
                    buf[k].append(v)
                act = torch.cat([cont.clamp(-1, 1), 2 * attach[..., None] - 1], dim=-1)
                preds = [m.step(x, lk, act) for m in models]
                prev = x
                x, lk = preds[int(torch.randint(len(preds), (1,), generator=gen))]
                lk = (lk > 0.5).float()
                r, done = task_reward(x, prev, g)
                if len(preds) > 1:
                    pos = torch.stack([p[0][..., :3] for p in preds])
                    r = r - uncertainty_weight * pos.std(0).norm(dim=-1).amax(-1)
                rewards.append((r - time_penalty) * alive)
                success = torch.maximum(success, done * alive)
                alive = alive * (1 - done)
                alives.append(alive)
            values = [critic(xi, li, g) for xi, li in zip(buf["x"], buf["lk"], strict=True)]
            values.append(critic(x, lk, g))
            adv, gae = [], torch.zeros(batch_size, device=dev)
            for k in reversed(range(horizon)):  # GAE; the horizon end is a truncation
                delta = rewards[k] + gamma * alives[k] * values[k + 1] - values[k]
                gae = delta + gamma * lam * alives[k] * gae
                adv.append(gae)
            adv = torch.stack(adv[::-1])
            ret = adv + torch.stack(values[:-1])
            live = torch.stack([torch.ones(batch_size, device=dev), *alives[:-1]]) > 0
        flat = {k: torch.stack(v)[live] for k, v in buf.items()}
        flat_g = g.expand(horizon, -1, -1)[live]
        adv_f, ret_f = adv[live], ret[live]
        adv_f = (adv_f - adv_f.mean()) / (adv_f.std() + 1e-8)
        for _ in range(epochs):
            for idx in torch.randperm(adv_f.numel(), device=dev).split(minibatch):
                dists = _ppo_dist(policy, flat["obs"][idx], std)
                ratio = (_log_prob(dists, flat["c"][idx], flat["a"][idx]) - flat["lp"][idx]).exp()
                surrogate = torch.minimum(
                    ratio * adv_f[idx], ratio.clamp(1 - clip, 1 + clip) * adv_f[idx]
                )
                with torch.no_grad():
                    ref = reference(flat["obs"][idx], ref_policy.stats)
                ref_n = torch.distributions.Normal(ref[..., :3], std.exp().expand_as(ref[..., :3]))
                kl = torch.distributions.kl_divergence(dists[0], ref_n).sum((-1, -2))
                kl = kl + _bernoulli_kl(dists[1].logits, ref[..., 3]).sum(-1)
                actor_loss = -surrogate.mean() + kl_weight * kl.mean()
                actor_opt.zero_grad(set_to_none=True)
                actor_loss.backward()
                nn.utils.clip_grad_norm_(policy.net.parameters(), 0.5)
                actor_opt.step()
                value = critic(flat["x"][idx], flat["lk"][idx], flat_g[idx])
                critic_loss = ((value - ret_f[idx]) ** 2).mean()
                critic_opt.zero_grad(set_to_none=True)
                critic_loss.backward()
                critic_opt.step()
        log.append({
            "it": it,
            "return": float(torch.stack(rewards).sum(0).mean()),
            "success": float(success.mean()),
            "kl": float(kl.mean().detach()),
            "critic": float(critic_loss.detach()),
        })  # fmt: skip
    return log
