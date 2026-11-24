"""Small per-skill world models against one large world model of the whole swarm.

Every model predicts the next state of all spheres from their states, the executed actions and
the links (action-conditioned, actions and links given at every step). Kinds (``KINDS``):

- ``world``: one set transformer over sphere tokens and one token per step face of the course,
  world frame (the large monolithic model);
- ``id``: ``world`` plus the skill and role of each sphere (SkiMo / TD-MPC2-style conditioning);
- ``frame``: one shared transformer, each sphere in the frame of the skill call it executes
  (``call_ref``; idle spheres in their own frame), with skill, role and the call's step rise;
- ``modular``: the inputs of ``frame``, but one small transformer per skill (idle, roll, climb,
  pull), each attending only to the spheres of the same call; a sphere's prediction comes from
  the expert of its skill. ``frame`` against ``modular`` isolates modularity from framing.

Episodes come from the skill scheduler with the active call of every sphere at every step.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np
import torch
from numpy.typing import NDArray
from torch import nn

from modular_sphere.swarm import Course, SphereSwarmStepEnv
from modular_sphere.swarm_plan import Executor, ExecutorFactory, SkillScheduler
from modular_sphere.swarm_skills import (
    STD_FLOOR,
    SkillCall,
    SkillValue,
    skill_features,
    swarm_action,
)
from modular_sphere.swarm_wam import SkillWAM

Array = NDArray[np.float64]
# cost (K,) of end states (K, N, 9) with links (K, N, N) for a call; lower is better
Objective = Callable[[SkillCall, torch.Tensor, torch.Tensor], torch.Tensor]
KINDS = ("world", "id", "frame", "modular")
SKILL_IDS = {"idle": 0, "roll": 1, "climb": 2, "pull": 3}
N_CATS = 2 * len(SKILL_IDS)  # skill id * 2 + role in the call
MAX_FACES = 3
DELTA_FLOOR = 1e-3  # floor of the state-change std [m, m/s, rad/s per step]
STEP_KEYS = ("links", "cat", "ref", "rise", "group")


def call_ref(call: SkillCall) -> Array:
    """Frame of a call: origin (x, y, z) as in ``skill_features`` and the step rise."""
    x0 = float(call.target[0]) if call.name == "roll" else call.face
    return np.array([x0, float(call.target[1]), call.floor, call.rise])


@dataclass
class Episodes:
    """Scheduler episodes padded to ``T`` steps: states (E, T+1, N, 9), executed actions
    (E, T, N, 4), links (E, T, N, N), valid steps (E, T); per step and sphere: category
    ``cat`` (skill id * 2 + role), call frame origin ``ref`` (nan: idle, own frame), call rise,
    and ``group`` (call index, ``-1 - i`` for idle sphere ``i``); course faces (E, MAX_FACES, 3)
    as (face x, floor in front, rise), nan padded; success per episode."""

    spheres: Array
    actions: Array
    links: NDArray[np.bool_]
    valid: NDArray[np.bool_]
    cat: NDArray[np.int64]
    ref: Array
    rise: Array
    group: NDArray[np.int64]
    faces: Array
    success: NDArray[np.bool_]

    def save(self, path: str) -> None:
        np.savez_compressed(path, **self.__dict__)

    @classmethod
    def load(cls, path: str) -> Episodes:
        with np.load(path) as f:
            return cls(**{k: f[k] for k in f.files})

    def subset(self, idx: Any) -> Episodes:
        return Episodes(**{k: v[idx] for k, v in self.__dict__.items()})


def record_episode(sched: SkillScheduler, seed: int, course: Course | None = None,
                   dynamics: Callable[[SphereSwarmStepEnv, int], None] | None = None,
                   perturb: Callable[[Array], Array] | None = None) -> dict[str, Any]:  # fmt: skip
    """One closed-loop episode of ``sched`` (as ``evaluate_schedule``), with the active call of
    every sphere at every step; ``perturb`` maps the scheduler's actions to the executed ones."""
    env = SphereSwarmStepEnv(n_spheres=sched.n, course=course)
    if dynamics is not None:
        dynamics(env, seed)
    obs, _ = env.reset(seed=seed)
    sp = env.spheres()
    sched.reset(sp[:, :3], env.goal, course)
    xs, acts, lks, rows, runs = [sp], [], [], [], []
    info: dict[str, Any] = {}
    while env.agents:
        links = env.links
        act = sched.act([obs[a] for a in env.possible_agents], sp, links)
        if perturb is not None:
            act = perturb(act)
        row = np.full(sched.n, -1)
        for i, run in sched.runs.items():
            if not any(run is r for r in runs):
                runs.append(run)
            row[i] = next(k for k, r in enumerate(runs) if r is run)
        obs, _, _, _, infos = env.step(dict(zip(env.possible_agents, act, strict=True)))
        info = infos[env.possible_agents[0]]
        sp = env.spheres()
        xs.append(sp)
        acts.append(act)
        lks.append(links)
        rows.append(row)
    c = env.course
    faces = [(f, c.floor(k), c.rises[k]) for k, f in enumerate(c.faces)]
    return {"x": np.stack(xs), "a": np.stack(acts), "l": np.stack(lks), "c": np.stack(rows),
            "calls": [r.call for r in runs], "ok": bool(info["is_success"]),
            "faces": np.array(faces)}  # fmt: skip


class HeldNoise:
    """Temporally correlated exploration: with probability ``p`` per segment, a Gaussian offset
    (``sigma`` on fx, fy, lift of every sphere) held for a segment of 1 to ``max_len`` steps, the
    shape of ``WMVerifier``'s candidates; white per-step noise leaves held offsets unseen."""

    def __init__(self, seed: int, sigma: float = 0.3, p: float = 0.5, max_len: int = 10) -> None:
        self.rng, self.sigma, self.p, self.max_len = np.random.default_rng(seed), sigma, p, max_len
        self.left, self.off = 0, np.zeros(3)

    def __call__(self, act: Array) -> Array:
        if self.left == 0:
            self.left = int(self.rng.integers(1, self.max_len + 1))
            on = self.rng.random() < self.p
            self.off = on * self.sigma * self.rng.standard_normal((len(act), 3))
        self.left -= 1
        out = act.copy()
        out[:, :3] = np.clip(out[:, :3] + self.off, -1.0, 1.0)
        return out


def pad_episodes(eps: list[dict[str, Any]]) -> Episodes:
    e, t, n = len(eps), max(len(d["a"]) for d in eps), eps[0]["x"].shape[1]
    out = Episodes(
        spheres=np.zeros((e, t + 1, n, 9)),
        actions=np.zeros((e, t, n, 4)),
        links=np.zeros((e, t, n, n), dtype=bool),
        valid=np.zeros((e, t), dtype=bool),
        cat=np.zeros((e, t, n), dtype=np.int64),
        ref=np.full((e, t, n, 3), np.nan),
        rise=np.zeros((e, t, n)),
        group=np.tile(-1 - np.arange(n), (e, t, 1)),
        faces=np.full((e, MAX_FACES, 3), np.nan),
        success=np.array([d["ok"] for d in eps]),
    )
    for i, d in enumerate(eps):
        k = len(d["a"])
        out.spheres[i, : k + 1], out.spheres[i, k + 1 :] = d["x"], d["x"][-1]
        out.actions[i, :k], out.links[i, :k], out.valid[i, :k] = d["a"], d["l"], True
        out.faces[i, : len(d["faces"])] = d["faces"]
        for s in range(k):
            for a, c in enumerate(d["c"][s]):
                if c < 0:
                    continue
                call = d["calls"][c]
                ref = call_ref(call)
                out.cat[i, s, a] = 2 * SKILL_IDS[call.name] + call.agents.index(a)
                out.ref[i, s, a], out.rise[i, s, a], out.group[i, s, a] = ref[:3], ref[3], c
    return out


class _Block(nn.Module):
    def __init__(self, width: int, heads: int) -> None:
        super().__init__()
        self.ln1, self.ln2 = nn.LayerNorm(width), nn.LayerNorm(width)
        self.attn = nn.MultiheadAttention(width, heads, batch_first=True)
        self.mlp = nn.Sequential(
            nn.Linear(width, 4 * width), nn.GELU(), nn.Linear(4 * width, width)
        )

    def forward(self, h: torch.Tensor, bias: torch.Tensor) -> torch.Tensor:
        z = self.ln1(h)
        h = h + self.attn(z, z, z, attn_mask=bias, need_weights=False)[0]
        return h + self.mlp(self.ln2(h))


class SetNet(nn.Module):
    """Pre-LN transformer over sphere tokens (and optional face tokens) predicting a normalised
    state change per sphere. Links bias the attention (one learned weight per layer and head);
    ``group`` (B, N) restricts the attention to spheres of the same group."""

    def __init__(self, width: int, layers: int, heads: int, d_in: int, cats: bool,
                 faces: bool) -> None:  # fmt: skip
        super().__init__()
        self.heads = heads
        self.sphere = nn.Linear(d_in, width)
        self.cat = nn.Embedding(N_CATS, width) if cats else None
        self.face = nn.Linear(3, width) if faces else None
        self.blocks = nn.ModuleList(_Block(width, heads) for _ in range(layers))
        self.link_w = nn.Parameter(torch.zeros(layers, heads))
        self.out = nn.Linear(width, 9)

    def forward(self, tok: torch.Tensor, cat: torch.Tensor, links: torch.Tensor,
                group: torch.Tensor | None, faces: torch.Tensor | None
                ) -> torch.Tensor:  # fmt: skip
        h = self.sphere(tok)
        if self.cat is not None:
            h = h + self.cat(cat)
        b, n = h.shape[:2]
        if group is None:
            allow = torch.ones(b, n, n, dtype=torch.bool, device=h.device)
        else:
            allow = group[:, :, None] == group[:, None, :]
        lk = links.float()
        if self.face is not None and faces is not None:
            pad = torch.isnan(faces[..., 0])
            h = torch.cat([h, self.face(torch.nan_to_num(faces))], 1)
            m = faces.shape[1]
            lk = nn.functional.pad(lk, (0, m, 0, m))
            allow = nn.functional.pad(allow, (0, m, 0, m), value=True)
            allow[:, :, n:] &= ~pad[:, None]
        neg = torch.zeros(allow.shape, dtype=h.dtype, device=h.device).masked_fill(~allow, -1e9)
        for k, blk in enumerate(self.blocks):
            bias = self.link_w[k][None, :, None, None] * lk[:, None] + neg[:, None]
            h = blk(h, bias.flatten(0, 1))
        return self.out(h[:, :n])


class SwarmWM(nn.Module):
    """World model of one ``kind`` (``KINDS``). ``step(x, act, ctx)`` maps raw states (B, N, 9)
    and actions (B, N, 4) to the next raw states; ``ctx`` holds ``links`` (B, N, N), ``cat``,
    ``ref``, ``rise``, ``group`` per sphere at that step and the course ``faces``."""

    stats: torch.Tensor

    def __init__(self, kind: str, width: int = 64, layers: int = 2, heads: int = 4) -> None:
        super().__init__()
        if kind not in KINDS:
            raise ValueError(f"unknown kind {kind!r}, expected one of {KINDS}")
        self.kind, self.cfg = kind, {"width": width, "layers": layers, "heads": heads}
        self.framed = kind in ("frame", "modular")
        n_nets = len(SKILL_IDS) if kind == "modular" else 1
        self.nets = nn.ModuleList(
            SetNet(width, layers, heads, 13 + self.framed, kind != "world", not self.framed)
            for _ in range(n_nets)
        )
        self.register_buffer("stats", torch.ones(4, 9))  # x mean, x std, delta mean, delta std

    def _rel(self, x: torch.Tensor, ctx: dict[str, torch.Tensor]) -> torch.Tensor:
        if not self.framed:
            return x
        ref = ctx["ref"]
        origin = torch.where(torch.isnan(ref), x[..., :3], ref)
        return torch.cat([x[..., :3] - origin, x[..., 3:]], -1)

    def step(self, x: torch.Tensor, act: torch.Tensor, ctx: dict[str, torch.Tensor]
             ) -> torch.Tensor:  # fmt: skip
        z = (self._rel(x, ctx) - self.stats[0]) / self.stats[1]
        tok = torch.cat([z, act, ctx["rise"][..., None]] if self.framed else [z, act], -1)
        cat, links = ctx["cat"], ctx["links"]
        if self.kind != "modular":
            d = self.nets[0](tok, cat, links, None, None if self.framed else ctx["faces"])
        else:
            d, skill = torch.zeros_like(x), cat // 2
            for s, net in enumerate(self.nets):
                own = skill == s
                if own.any():
                    d = torch.where(own[..., None], net(tok, cat, links, ctx["group"], None), d)
        return x + d * self.stats[3] + self.stats[2]

    def fit_stats(self, x: torch.Tensor, nxt: torch.Tensor, ctx: dict[str, torch.Tensor],
                  mask: torch.Tensor) -> None:  # fmt: skip
        """Normalisation from transitions (M, N, 9) -> (M, N, 9), valid where ``mask`` (M,)."""
        rel, d = self._rel(x, ctx)[mask].flatten(0, 1), (nxt - x)[mask].flatten(0, 1)
        self.stats.copy_(torch.stack([rel.mean(0), rel.std(0).clamp_min(STD_FLOOR),
                                      d.mean(0), d.std(0).clamp_min(DELTA_FLOOR)]))  # fmt: skip

    def n_params(self) -> int:
        return sum(p.numel() for p in self.parameters())


def windows(ep: Episodes, w: int, stride: int, device: torch.device | str = "cpu"
            ) -> dict[str, torch.Tensor]:  # fmt: skip
    """Windows of ``w`` steps starting every ``stride`` valid steps: ``x`` (M, w+1, N, 9),
    ``act`` (M, w, N, 4), the ``STEP_KEYS`` (M, w, N, ...), ``mask`` (M, w), ``faces``."""
    e_idx, t_idx = np.nonzero(ep.valid)
    keep = t_idx % stride == 0
    e_idx, t_idx = e_idx[keep], t_idx[keep]
    steps = t_idx[:, None] + np.arange(w + 1)
    big = ep.valid.shape[1]
    st = np.minimum(steps[:, :w], big - 1)
    out = {
        "x": ep.spheres[e_idx[:, None], np.minimum(steps, big)],
        "act": ep.actions[e_idx[:, None], st],
        "mask": ep.valid[e_idx[:, None], st] & (steps[:, :w] < big),
        "faces": ep.faces[e_idx],
        "episode": e_idx,
        "t": t_idx,
    }
    for k in STEP_KEYS:
        out[k] = getattr(ep, k)[e_idx[:, None], st]
    floats = {"x", "act", "faces", "ref", "rise"}
    return {k: torch.as_tensor(v, dtype=torch.float32 if k in floats else None, device=device)
            for k, v in out.items()}  # fmt: skip


def _ctx(b: dict[str, torch.Tensor], t: int) -> dict[str, torch.Tensor]:
    return {**{k: b[k][:, t] for k in STEP_KEYS}, "faces": b["faces"]}


def rollout(model: SwarmWM, b: dict[str, torch.Tensor], h: int | None = None) -> torch.Tensor:
    """Open-loop prediction (M, h, N, 9) of a window batch from its first state."""
    xh, out = b["x"][:, 0], []
    for t in range(b["act"].shape[1] if h is None else h):
        xh = model.step(xh, b["act"][:, t], _ctx(b, t))
        out.append(xh)
    return torch.stack(out, 1)


def unroll_loss(model: SwarmWM, b: dict[str, torch.Tensor]) -> torch.Tensor:
    """Mean squared normalised state-change error of the open-loop unroll over valid steps."""
    err = ((rollout(model, b) - b["x"][:, 1:]) / model.stats[3]) ** 2
    m = b["mask"].float()
    return (err.mean((-1, -2)) * m).sum() / m.sum().clamp_min(1)


def fit(model: SwarmWM, data: dict[str, torch.Tensor], epochs: int, lr: float = 3e-4,
        batch_size: int = 256, seed: int = 0, params: list[nn.Parameter] | None = None
        ) -> list[float]:  # fmt: skip
    """Normalisation from the windows' first transitions, then AdamW with cosine decay on the
    unroll loss; returns the loss per epoch. With ``params`` (continual learning), trains only
    those parameters and keeps the normalisation."""
    torch.manual_seed(seed)
    if params is None:
        params = list(model.parameters())
        with torch.no_grad():
            model.fit_stats(data["x"][:, 0], data["x"][:, 1], _ctx(data, 0), data["mask"][:, 0])
    opt = torch.optim.AdamW(params, lr=lr, weight_decay=1e-5)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
    n = len(data["x"])
    gen = torch.Generator().manual_seed(seed)
    losses = []
    for _ in range(epochs):
        tot = 0.0
        for idx in torch.randperm(n, generator=gen).split(batch_size):
            idx = idx.to(data["x"].device)
            loss = unroll_loss(model, {k: v[idx] for k, v in data.items()})
            opt.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(params, 1.0)
            opt.step()
            tot += loss.item() * len(idx)
        sched.step()
        losses.append(tot / n)
    return losses


@torch.no_grad()
def position_errors(model: SwarmWM, data: dict[str, torch.Tensor], batch_size: int = 256
                    ) -> NDArray[np.float64]:  # fmt: skip
    """Open-loop position error norms (M, w, N) [m], nan on invalid steps."""
    out = []
    for i in range(0, len(data["x"]), batch_size):
        b = {k: v[i : i + batch_size] for k, v in data.items()}
        e = (rollout(model, b)[..., :3] - b["x"][:, 1:, :, :3]).norm(dim=-1)
        out.append(torch.where(b["mask"][..., None], e, torch.nan).cpu().numpy())
    return np.concatenate(out).astype(np.float64)


def load_wm(path: str, device: torch.device | str = "cpu") -> SwarmWM:
    """A world model saved by ``docs/paper/experiments/modular_e1.py``, in eval mode."""
    ck = torch.load(path, map_location=device)
    model = SwarmWM(ck["kind"], **ck["cfg"]).to(device)
    model.load_state_dict(ck["state"])
    return model.eval()


def sphere_context(calls: dict[int, SkillCall], n: int) -> dict[str, NDArray[Any]]:
    """Per-sphere ``cat``, ``ref``, ``rise``, ``group`` (as in ``Episodes``) of the active calls
    (sphere -> its call; spheres without a call are idle)."""
    cat, ref = np.zeros(n, dtype=np.int64), np.full((n, 3), np.nan)
    rise, group = np.zeros(n), -1 - np.arange(n)
    ids: list[SkillCall] = []
    for i, call in calls.items():
        if not any(call is c for c in ids):
            ids.append(call)
        r = call_ref(call)
        cat[i] = 2 * SKILL_IDS[call.name] + call.agents.index(i)
        ref[i], rise[i] = r[:3], r[3]
        group[i] = next(k for k, c in enumerate(ids) if c is call)
    return {"cat": cat, "ref": ref, "rise": rise, "group": group}


class WMVerifier:
    """The policy proposes, the world model verifies. For every skill call, the WAM executor's
    action chunk plus ``k - 1`` Gaussian offsets (``sigma`` on fx, fy, lift, constant over the
    chunk; candidate 0 is the policy's chunk) are rolled out ``horizon`` steps in the swarm world
    model ``wm`` (other spheres keep their last executed action, links stay fixed); the candidate
    whose end state has the fewest predicted steps to go (the skill's ``SkillValue``) is executed
    (its first action) if it beats the policy's chunk by ``margin`` steps.

    ``executor(wams)`` is an ``ExecutorFactory`` for ``SkillScheduler``; before an episode set
    ``scheduler`` and ``course``, and after every step ``last`` (N, 4), the executed actions."""

    def __init__(self, wm: SwarmWM, values: dict[str, SkillValue], k: int = 16,
                 sigma: float = 0.3, horizon: int = 10, margin: float = 1.0,
                 seed: int = 0) -> None:  # fmt: skip
        self.wm, self.values, self.k, self.sigma = wm, values, k, sigma
        self.horizon, self.margin = horizon, margin
        self.gen = torch.Generator(device=wm.stats.device).manual_seed(seed)
        self.scheduler: SkillScheduler | None = None
        self.course: Course | None = None
        self.last: Array | None = None
        self.overrides = 0  # executed candidates other than the policy's
        # replaces ``score`` (for example a simulator oracle): (call, act, spheres, links) -> (K,)
        self.scorer: Callable[[SkillCall, torch.Tensor, Array, NDArray[np.bool_]],
                              torch.Tensor] | None = None  # fmt: skip
        # replaces the value as cost of the end states, e.g. ``progress_cost``
        self.objective: Objective | None = None

    def _faces(self) -> torch.Tensor:
        c = Course() if self.course is None else self.course
        f = np.full((MAX_FACES, 3), np.nan)
        f[: len(c.faces)] = [(x, c.floor(i), c.rises[i]) for i, x in enumerate(c.faces)]
        return torch.as_tensor(f, dtype=torch.float32, device=self.wm.stats.device)

    @torch.no_grad()
    def choose(self, call: SkillCall, chunk: torch.Tensor, spheres: Array,
               links: NDArray[np.bool_]) -> torch.Tensor:  # fmt: skip
        """First action (A, 4) of the best candidate around the policy's chunk (H, A, 4)."""
        cand, act = self.candidates(call, chunk, len(spheres))
        togo = (self.scorer or self.score)(call, act, spheres, links)
        best = int(torch.argmin(togo))
        if togo[best] + self.margin >= togo[0]:
            best = 0
        self.overrides += best != 0
        return cand[best, 0]

    def candidates(self, call: SkillCall, chunk: torch.Tensor, n: int
                   ) -> tuple[torch.Tensor, torch.Tensor]:  # fmt: skip
        """Candidate chunks of the call's spheres (K, H, A, 4) and swarm actions (K, H, N, 4)."""
        dev, ag, kk = self.wm.stats.device, list(call.agents), self.k
        h = min(self.horizon, chunk.shape[0])
        cand = chunk[:h][None].repeat(kk, 1, 1, 1)
        off = self.sigma * torch.randn(kk, 1, len(ag), 3, generator=self.gen, device=dev)
        off[0] = 0.0
        cand[..., :3] = (cand[..., :3] + off).clamp(-1.0, 1.0)
        cand[..., 3] = torch.where(cand[..., 3] > 0, 1.0, -1.0)  # as executed (swarm_action)
        last = np.zeros((n, 4)) if self.last is None else self.last
        act = torch.as_tensor(last, dtype=torch.float32, device=dev).repeat(kk, h, 1, 1)
        act[:, :, ag] = cand
        return cand, act

    @torch.no_grad()
    def score(self, call: SkillCall, act: torch.Tensor, spheres: Array,
              links: NDArray[np.bool_]) -> torch.Tensor:  # fmt: skip
        """Predicted steps to go (K,) of the call after the swarm actions (K, H, N, 4)."""
        x = self.predict(act, spheres, links)
        lk = torch.as_tensor(links, device=x.device).expand(len(x), *links.shape)
        if self.objective is not None:
            return self.objective(call, x, lk)
        return self.values[call.name](skill_features(call, x, lk))

    @torch.no_grad()
    def predict(self, act: torch.Tensor, spheres: Array, links: NDArray[np.bool_]) -> torch.Tensor:
        """Predicted sphere states (K, N, 9) after the swarm actions (K, H, N, 4)."""
        if self.scheduler is None:
            raise RuntimeError("set WMVerifier.scheduler before running an episode")
        dev, n, kk, h = self.wm.stats.device, len(spheres), act.shape[0], act.shape[1]
        calls = {i: r.call for i, r in self.scheduler.runs.items()}
        ctx = {}
        for key, v in sphere_context(calls, n).items():
            t = torch.as_tensor(v, device=dev)
            ctx[key] = (t.float() if key in ("ref", "rise") else t).expand(kk, *v.shape)
        ctx["links"] = torch.as_tensor(links, device=dev).expand(kk, n, n)
        ctx["faces"] = self._faces().expand(kk, -1, -1)
        x = torch.as_tensor(spheres, dtype=torch.float32, device=dev).repeat(kk, 1, 1)
        for t in range(h):
            x = self.wm.step(x, act[:, t], ctx)
        return x

    def executor(self, wams: dict[str, SkillWAM]) -> ExecutorFactory:
        """WAM executors (newest chunk) whose chunks pass through ``choose``."""

        def make(call: SkillCall, n: int) -> Executor:
            wam = wams[call.name]
            hist: deque[torch.Tensor] = deque(maxlen=wam.history)
            dev = wam.feat_mean.device

            @torch.no_grad()
            def act(obs: list[Array], spheres: Array, links: NDArray[np.bool_]) -> Array:
                x = torch.as_tensor(spheres, dtype=torch.float32, device=dev)
                f = skill_features(call, x, torch.as_tensor(links, device=dev))
                if not hist:
                    hist.extend([f] * wam.history)
                hist.append(f)
                _, chunk = wam(torch.stack(list(hist))[None])
                a = self.choose(call, chunk[0].to(self.wm.stats.device), spheres, links)
                return swarm_action(n, call.agents, a.cpu().numpy())

            return act

        return make


def simulator_rollouts(env: SphereSwarmStepEnv, act: torch.Tensor) -> tuple[Array, Array]:
    """Sphere states (K, N, 9) and links (K, N, N) after executing each candidate's swarm
    actions (K, H, N, 4) in ``env`` from its current state, which is restored afterwards."""
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    s0 = np.empty(mujoco.mj_stateSize(env.model, spec))
    mujoco.mj_getState(env.model, env.data, s0, spec)
    eq, grp = env.model.eq_data.copy(), env.model.tendon_group.copy()
    saved = (env._steps, env._prev_dist, list(env.agents))
    xs, lks = [], []
    for a in act.cpu().numpy():
        for t in range(len(a)):
            if env.agents:
                env.step(dict(zip(env.possible_agents, a[t], strict=True)))
        xs.append(env.spheres())
        lks.append(env.links)
        mujoco.mj_setState(env.model, env.data, s0, spec)
        env.model.eq_data[:], env.model.tendon_group[:] = eq, grp
        env._steps, env._prev_dist, env.agents = saved[0], saved[1], list(saved[2])
        mujoco.mj_forward(env.model, env.data)
    return np.stack(xs), np.stack(lks)


def simulator_scorer(env: SphereSwarmStepEnv, values: dict[str, SkillValue],
                     objective: Objective | None = None
                     ) -> Callable[[SkillCall, torch.Tensor, Array, NDArray[np.bool_]],
                                   torch.Tensor]:  # fmt: skip
    """``WMVerifier.scorer`` with the simulator as world model (an oracle upper bound)."""

    def score(call: SkillCall, act: torch.Tensor, spheres: Array, links: NDArray[np.bool_]
              ) -> torch.Tensor:  # fmt: skip
        x, lk = simulator_rollouts(env, act)
        xt = torch.as_tensor(x, dtype=torch.float32, device=act.device)
        lt = torch.as_tensor(lk, device=act.device)
        if objective is not None:
            return objective(call, xt, lt)
        return values[call.name](skill_features(call, xt, lt))

    return score


def progress_cost(call: SkillCall, x: torch.Tensor, links: torch.Tensor, radius: float = 0.1
                  ) -> torch.Tensor:  # fmt: skip
    """Geometric distance [cm] (K,) of end states (K, N, 9) to the call's termination event
    (``SkillController.done``), a cost that needs no learned value: roll, xy distance to the
    target; climb, height the climber still lacks; pull, height and x the base still lacks
    relative to the helper."""
    ag = list(call.agents)
    if call.name == "roll":
        tgt = torch.as_tensor(call.target, dtype=x.dtype, device=x.device)
        d = (x[:, ag[0], :2] - tgt).norm(dim=-1)
    elif call.name == "climb":
        d = (call.floor + 3 * radius - x[:, ag[0], 2]).clamp_min(0.0)
    else:
        h, b = ag
        d = (x[:, h, 2] + radius - x[:, b, 2]).clamp_min(0.0)
        d = d + (x[:, h, 0] - 0.05 - x[:, b, 0]).clamp_min(0.0)
    return 100.0 * d
