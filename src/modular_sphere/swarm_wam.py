"""Small world-action model (WAM) per skill.

A world-action model predicts, from a short history, both the future of the scene and the chunk of
actions that produces it, with the action tokens attending to the predicted future (GR00T 2,
NVIDIA 2026; Cosmos Policy, ICLR 2026). Here the "scene" is the target-relative state of the
spheres involved in a skill (``swarm_skills.skill_features``), so the model is a small transformer
instead of a video model. Action chunks and a history window follow ACT (Zhao et al., 2023), which
also handles the non-Markovian phase latch of the skill controllers.

Deviations from GR00T 2: deterministic regression instead of flow matching, state tokens instead
of video latents, one model per skill trained on that skill's data only.
"""

from __future__ import annotations

from collections import deque

import numpy as np
import torch
from numpy.typing import NDArray
from torch import nn

from modular_sphere.swarm import ACT_DIM, SphereSwarmStepEnv
from modular_sphere.swarm_data import SwarmDataset, empty_arrays
from modular_sphere.swarm_skills import (
    MAX_SKILL_STEPS,
    STD_FLOOR,
    SkillCall,
    SkillController,
    feature_dim,
    place,
    sample_start,
    skill_features,
)  # fmt: skip

Array = NDArray[np.float64]


class SkillWAM(nn.Module):
    """History of ``history`` feature frames -> ``chunk`` future frames and ``chunk`` actions.

    Tokens: history frames, then future-frame queries, then action queries; every token attends
    to every other one, so the actions are read out next to the imagined future."""

    feat_mean: torch.Tensor
    feat_std: torch.Tensor
    delta_std: torch.Tensor

    def __init__(self, n_agents: int, history: int = 4, chunk: int = 10, width: int = 128,
                 layers: int = 3, heads: int = 4) -> None:  # fmt: skip
        super().__init__()
        self.cfg = {"n_agents": n_agents, "history": history, "chunk": chunk, "width": width,
                    "layers": layers, "heads": heads}  # fmt: skip
        self.n_agents, self.history, self.chunk = n_agents, history, chunk
        self.dim = dim = feature_dim(n_agents)
        self.inp = nn.Linear(dim, width)
        self.pos = nn.Parameter(torch.zeros(history + 2 * chunk, width))
        self.kind = nn.Parameter(torch.zeros(3, width))  # history, future frame, action
        nn.init.normal_(self.pos, std=0.02)
        nn.init.normal_(self.kind, std=0.02)
        layer = nn.TransformerEncoderLayer(width, heads, 4 * width, dropout=0.0,
                                           batch_first=True, norm_first=True)  # fmt: skip
        self.body = nn.TransformerEncoder(layer, layers, enable_nested_tensor=False)
        self.frame_head = nn.Linear(width, dim)
        self.act_head = nn.Linear(width, n_agents * ACT_DIM)
        self.register_buffer("feat_mean", torch.zeros(dim))
        self.register_buffer("feat_std", torch.ones(dim))
        self.register_buffer("delta_std", torch.ones(dim))

    def forward(self, hist: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """``hist`` (B, history, F) raw features -> future frames (B, chunk, F) as deltas from the
        last history frame in units of ``delta_std``, and actions (B, chunk, A, 4): tanh(fx, fy,
        lift) and the attach logit."""
        b, k, h = hist.shape[0], self.history, self.chunk
        x = self.inp((hist - self.feat_mean) / self.feat_std) + self.kind[0]
        q = self.kind[1:].repeat_interleave(h, 0)[None].expand(b, -1, -1)
        z = self.body(torch.cat([x, q], 1) + self.pos)
        frames = self.frame_head(z[:, k : k + h])
        out = self.act_head(z[:, k + h :]).unflatten(-1, (self.n_agents, ACT_DIM))
        return frames, torch.cat([torch.tanh(out[..., :3]), out[..., 3:]], -1)

    def _windows(self, data: SwarmDataset, calls: list[SkillCall], device: torch.device
                 ) -> tuple[torch.Tensor, ...]:  # fmt: skip
        """Training windows of every valid step: history, future frames, actions, masks."""
        k, h = self.history, self.chunk
        hists, futs, acts, masks = [], [], [], []
        for e, call in enumerate(calls):
            n = int(data.valid[e].sum())
            if n == 0:
                continue
            x = torch.as_tensor(data.spheres[e, : n + 1], dtype=torch.float32, device=device)
            lk = torch.as_tensor(data.links[e, : n + 1], device=device)
            f = skill_features(call, x, lk)  # (n + 1, F)
            a = torch.as_tensor(data.expert[e, :n][:, list(call.agents)], dtype=torch.float32,
                                device=device)  # fmt: skip
            idx = torch.arange(n, device=device)
            past = (idx[:, None] + torch.arange(-k + 1, 1, device=device)).clamp(min=0)
            fut = idx[:, None] + torch.arange(1, h + 1, device=device)
            hists.append(f[past])
            futs.append(f[fut.clamp(max=n)])
            acts.append(a[(fut - 1).clamp(max=n - 1)])
            masks.append((fut - 1 < n).float())
        return torch.cat(hists), torch.cat(futs), torch.cat(acts), torch.cat(masks)

    def fit(self, data: SwarmDataset, calls: list[SkillCall], epochs: int = 60,
            lr: float = 3e-4, batch_size: int = 512, frame_weight: float = 1.0,
            seed: int = 0, keep_stats: bool = False, input_noise: float = 0.0
            ) -> list[float]:  # fmt: skip
        """Joint loss on future frames and action chunks (labels: ``data.expert``); ``calls[e]``
        is episode ``e``'s call. ``keep_stats`` keeps the normalisation (fine-tuning).
        ``input_noise`` [m] adds Gaussian sensor noise to the history frames (positions, twice
        that on velocities, none on links) while the targets stay clean."""
        torch.manual_seed(seed)
        dev = self.feat_mean.device
        hist, fut, act, mask = self._windows(data, calls, dev)
        if not keep_stats:
            self.feat_mean.copy_(hist[:, -1].mean(0))
            self.feat_std.copy_(hist[:, -1].std(0).clamp(min=STD_FLOOR))
            delta = (fut - hist[:, -1:]).flatten(0, 1).std(0)
            self.delta_std.copy_(delta.clamp(min=STD_FLOOR / 10))  # per-step deltas are smaller
        scale = torch.tensor([1.0] * 3 + [2.0] * 6, device=dev).repeat(self.n_agents)
        scale = input_noise * nn.functional.pad(scale, (0, self.dim - scale.numel()))
        optim = torch.optim.AdamW(self.parameters(), lr=lr, weight_decay=1e-4)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(optim, T_max=epochs)
        gen = torch.Generator(device="cpu").manual_seed(seed)
        losses = []
        for _ in range(epochs):
            total = 0.0
            for idx in torch.randperm(hist.shape[0], generator=gen).to(dev).split(batch_size):
                h = hist[idx]
                noisy = h + scale * torch.randn_like(h)
                loss = self.loss(noisy, fut[idx], act[idx], mask[idx], frame_weight, h[:, -1:])
                optim.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(self.parameters(), 1.0)
                optim.step()
                total += loss.item() * idx.numel()
            sched.step()
            losses.append(total / hist.shape[0])
        return losses

    def loss(self, hist: torch.Tensor, fut: torch.Tensor, act: torch.Tensor, mask: torch.Tensor,
             frame_weight: float = 1.0, last: torch.Tensor | None = None
             ) -> torch.Tensor:  # fmt: skip
        """``last``: clean last history frame the future deltas are taken from (default:
        ``hist[:, -1:]``)."""
        frames, out = self(hist)
        target = (fut - (hist[:, -1:] if last is None else last)) / self.delta_std
        frame_loss = ((frames - target) ** 2).mean()
        mse = ((out[..., :3] - act[..., :3]) ** 2).mean(-1)
        bce = nn.functional.binary_cross_entropy_with_logits(
            out[..., 3], (act[..., 3] > 0).float(), reduction="none"
        )
        act_loss = ((mse + bce).mean(-1) * mask).sum() / mask.sum().clamp(min=1.0)
        return act_loss + frame_weight * frame_loss

    def save(self, path: str) -> None:
        torch.save({"cfg": self.cfg, "state": self.state_dict()}, path)

    @classmethod
    def load(cls, path: str, device: str = "cpu") -> SkillWAM:
        ckpt = torch.load(path, map_location=device, weights_only=True)
        wam = cls(**ckpt["cfg"]).to(device)
        wam.load_state_dict(ckpt["state"])
        return wam


class WAMController:
    """Runs a ``SkillWAM`` for one skill call with ACT temporal ensembling: every step predicts a
    new chunk, and the executed action averages all chunks' predictions for this step with
    weights ``exp(-ensemble * i)``, ``i = 0`` for the oldest chunk as in ACT (``ensemble=None``
    executes the newest chunk only)."""

    def __init__(self, wam: SkillWAM, call: SkillCall, n_spheres: int,
                 ensemble: float | None = 0.1) -> None:  # fmt: skip
        self.wam, self.call, self.n, self.ensemble = wam, call, n_spheres, ensemble
        self.hist: deque[torch.Tensor] = deque(maxlen=wam.history)
        self.chunks: deque[tuple[int, torch.Tensor]] = deque(maxlen=wam.chunk)
        self.t = 0

    @torch.no_grad()
    def act(self, spheres: Array, links: NDArray[np.bool_]) -> Array:
        """Actions (N, 4) from the true sphere states (N, 9) and links (N, N)."""
        dev = self.wam.feat_mean.device
        x = torch.as_tensor(spheres, dtype=torch.float32, device=dev)
        f = skill_features(self.call, x, torch.as_tensor(links, device=dev))
        if not self.hist:
            self.hist.extend([f] * self.wam.history)
        self.hist.append(f)
        _, chunk = self.wam(torch.stack(list(self.hist))[None])
        self.chunks.append((self.t, chunk[0]))
        if self.ensemble is None:
            a = chunk[0, 0]
        else:
            # the deque keeps the last ``chunk`` chunks: each still covers the current step
            ages = [self.t - t0 for t0, _ in self.chunks]
            preds = [c[age] for age, (_, c) in zip(ages, self.chunks, strict=True)]
            order = max(ages) - torch.tensor(ages, dtype=torch.float32, device=dev)  # 0 = oldest
            w = torch.exp(-self.ensemble * order)
            a = (w[:, None, None] * torch.stack(preds)).sum(0) / w.sum()
        self.t += 1
        out = np.zeros((self.n, ACT_DIM))
        out[list(self.call.agents)] = a.cpu().numpy()
        out[:, 3] = np.where(out[:, 3] > 0, 1.0, -1.0)
        return out


def rollout_skill(wam: SkillWAM, name: str, n_starts: int, group: int = 8, noise: float = 0.2,
                  seed: int = 0, ensemble: float | None = 0.1
                  ) -> tuple[SwarmDataset, list[SkillCall], NDArray[np.int64]]:  # fmt: skip
    """``group`` rollouts of the WAM from each of ``n_starts`` sampled starts (two-sphere env),
    with Gaussian noise on the executed fx, fy, lift. Episodes are ordered start by start; the
    dataset's ``expert`` holds the noisy samples before clipping (self-imitation labels)."""
    env = SphereSwarmStepEnv(n_spheres=2, max_steps=MAX_SKILL_STEPS + 1)
    ag = (0,) if name == "roll" else (0, 1)
    rng, n_eps, horizon = np.random.default_rng(seed), n_starts * group, MAX_SKILL_STEPS
    out = empty_arrays(n_eps, horizon, 2, env.slices["others"].stop)
    calls, steps = [], np.zeros(n_eps, dtype=np.int64)
    for s in range(n_starts):
        start, target = sample_start(name, rng, env.radius)
        call = SkillCall(name, ag, target)
        for g in range(group):
            e = s * group + g
            env.reset(seed=seed + s)
            place(env, start)
            env.data.mocap_pos[env._goal_mocap][:2] = (10.0, 10.0)
            judge = SkillController(call, 2, env.radius)
            ctrl = WAMController(wam, call, 2, ensemble)
            out["spheres"][e] = np.concatenate(env._kinematics(), 1)
            out["links"][e] = env.links
            for t in range(horizon):
                act = ctrl.act(out["spheres"][e, t], env.links)
                act[list(ag), :3] += noise * rng.standard_normal((len(ag), 3))
                # label = unclipped sample: clipped labels would bias saturated actions (lift = 1)
                out["expert"][e, t] = act
                act = np.clip(act, -1.0, 1.0)
                env.step({a: act[i] for i, a in enumerate(env.possible_agents)})
                out["actions"][e, t] = act
                out["valid"][e, t] = True
                out["spheres"][e, t + 1 :] = np.concatenate(env._kinematics(), 1)
                out["links"][e, t + 1 :] = env.links
                pos, vel = env._kinematics()[:2]
                if judge.done(pos, vel):
                    out["success"][e] = True
                    break
            steps[e] = t + 1
            calls.append(call)
    return SwarmDataset(**out), calls, steps


def select_elite(success: NDArray[np.bool_], steps: NDArray[np.int64], group: int
                 ) -> NDArray[np.bool_]:  # fmt: skip
    """Group-relative filter (GRPO-induced self-imitation, GR00T 2): per start, keep the
    successful rollouts at least as fast as the group's mean successful duration, i.e. a
    non-negative group-relative advantage under the reward ``success - time``."""
    keep = np.zeros(len(steps), dtype=bool)
    for s in range(len(steps) // group):
        sl = slice(s * group, (s + 1) * group)
        ok = success[sl]
        if ok.any():
            keep[sl] = ok & (steps[sl] <= steps[sl][ok].mean())
    return keep
