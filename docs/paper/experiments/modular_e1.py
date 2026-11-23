"""E1: open-loop accuracy of small per-skill world models against one large swarm world model.

Usage:
    modular_e1.py collect SET [workers=6]
    modular_e1.py train KIND WIDTH LAYERS SEED DEVICE [epochs=40] [w=8] [frac=1]
    modular_e1.py eval DEVICE [h=30] (every saved model on every set)
    modular_e1.py finetune STEM DEVICE [epochs=10] (on train + explore, saved as STEM_x)

SETs (scheduler with the scripted skills; DART noise 0.05 on fx, fy, lift in half the seeds of
train, val and test, none elsewhere: the scripted skills fail often above that):
    train (seeds 0..399), val (300000..049), test (200000..049): four spheres, one step; three
    seeds in four have a random rise in [0.10, 0.15] m and randomised mass and friction
    (``perturb_dynamics``), the fourth is nominal. Out of distribution (seeds 600000..049):
    courses (random 1-3 step courses), n6 (six spheres), mass0.7, mass1.5, rise0.17.
    explore (seeds 700000..399): as train, but ``HeldNoise`` (held offsets of 0.3) on the executed
    actions instead of DART, for action coverage (E2: the verifier's candidates are held offsets).
Saves data/swarm/modular/{SET}.npz, wm_{KIND}_{WIDTH}x{LAYERS}_s{SEED}[_f{FRAC}].pt (with its
loss curve and training time) and e1.pkl {model: {set: errors (M, h, N)}}.
"""

import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import torch

from modular_sphere.swarm import STEP_X, Course, random_course
from modular_sphere.swarm_modular import (
    Episodes,
    HeldNoise,
    SwarmWM,
    fit,
    pad_episodes,
    position_errors,
    record_episode,
    windows,
)
from modular_sphere.swarm_plan import EffectModel, SkillScheduler, perturb_dynamics, script_executor
from modular_sphere.swarm_skills import SKILLS, SkillValue

ROOT = Path(__file__).resolve().parents[3] / "data/swarm"
OUT = ROOT / "modular"
SETS = {
    "train": range(0, 400),
    "explore": range(700_000, 700_400),
    "val": range(300_000, 300_050),
    "test": range(200_000, 200_050),
    **{k: range(600_000, 600_050) for k in ("courses", "n6", "mass0.7", "mass1.5", "rise0.17")},
}


def scale_mass(k):
    def apply(env, seed):
        import mujoco

        body = env.model.body_mass > 0
        env.model.body_mass[body] *= k
        env.model.body_inertia[body] *= k
        mujoco.mj_setConst(env.model, env.data)

    return apply


def condition(name, seed):
    """(spheres, course, dynamics) of one episode of set ``name``."""
    if name in ("train", "val", "test", "explore"):
        if seed % 4 == 0:
            return 4, None, None
        rise = round(float(np.random.default_rng(seed).uniform(0.10, 0.15)), 3)
        return 4, Course((STEP_X,), (rise,)), perturb_dynamics
    if name == "courses":
        return 4, random_course(np.random.default_rng(seed)), None
    if name == "n6":
        return 6, None, None
    if name.startswith("mass"):
        return 4, None, scale_mass(float(name[4:]))
    assert name == "rise0.17", name
    return 4, Course((STEP_X,), (0.17,)), None


def _record(args):
    name, seeds = args
    torch.set_num_threads(1)
    effect = EffectModel({s: SkillValue.load(f"{ROOT}/skills/value_{s}.pt") for s in SKILLS})
    out = []
    for seed in seeds:
        n, course, dyn = condition(name, seed)
        noise = 0.05 if name in ("train", "val", "test") and (seed // 4) % 2 else 0.0
        sched = SkillScheduler(n, effect, script_executor(), noise=noise, seed=seed)
        perturb = HeldNoise(seed) if name == "explore" else None
        out.append(record_episode(sched, seed, course, dyn, perturb))
    return out


def collect(name, workers):
    seeds = list(SETS[name])
    chunks = [(name, seeds[i::workers]) for i in range(workers)]
    with ProcessPoolExecutor(workers) as pool:
        parts = list(pool.map(_record, chunks))
    eps = [None] * len(seeds)
    for i, part in enumerate(parts):
        eps[i::workers] = part
    ep = pad_episodes(eps)
    OUT.mkdir(parents=True, exist_ok=True)
    ep.save(f"{OUT}/{name}.npz")
    steps = ep.valid.sum(1)
    print(f"{name}: {len(seeds)} episodes, success {ep.success.mean():.2f}, "
          f"steps {steps.mean():.0f} (max {steps.max()})")  # fmt: skip


def train(kind, width, layers, seed, device, kw):
    epochs, w, frac = int(kw.get("epochs", "40")), int(kw.get("w", "8")), float(kw.get("frac", "1"))
    ep = Episodes.load(f"{OUT}/train.npz")
    if frac < 1:
        ep = ep.subset(np.arange(round(frac * len(ep.valid))))
    data = windows(ep, w, 2, device)
    model = SwarmWM(kind, width, layers).to(device)
    t0 = time.time()
    losses = fit(model, data, epochs, seed=seed)
    dt = time.time() - t0
    name = f"wm_{kind}_{width}x{layers}_s{seed}" + (f"_f{frac:g}" if frac < 1 else "")
    torch.save({"kind": kind, "cfg": model.cfg, "state": model.state_dict(), "losses": losses,
                "time": dt, "params": model.n_params()}, f"{OUT}/{name}.pt")  # fmt: skip
    print(
        f"{name}: {model.n_params()} params, {dt:.0f} s, loss {losses[0]:.3f} -> {losses[-1]:.4f}"
    )


def finetune(stem, device, kw):
    ck = torch.load(f"{OUT}/{stem}.pt", map_location=device)
    model = SwarmWM(ck["kind"], **ck["cfg"]).to(device)
    model.load_state_dict(ck["state"])
    parts = [windows(Episodes.load(f"{OUT}/{s}.npz"), 8, 2, device) for s in ("train", "explore")]
    data = {k: torch.cat([p[k] for p in parts]) for k in parts[0]}
    t0 = time.time()
    losses = fit(model, data, int(kw.get("epochs", "10")), lr=1e-4)
    dt = time.time() - t0
    torch.save({"kind": ck["kind"], "cfg": model.cfg, "state": model.state_dict(), "losses": losses,
                "time": dt, "params": model.n_params()}, f"{OUT}/{stem}_x.pt")  # fmt: skip
    print(f"{stem}_x: {dt:.0f} s, loss {losses[0]:.4f} -> {losses[-1]:.4f}")


def evaluate(device, kw):
    h = int(kw.get("h", "30"))
    path = OUT / "e1.pkl"
    res = pickle.load(open(path, "rb")) if path.exists() else {}
    sets = {s: windows(Episodes.load(f"{OUT}/{s}.npz"), h, 10, device)
            for s in SETS if s != "train" and (OUT / f"{s}.npz").exists()}  # fmt: skip
    for f in sorted(OUT.glob("wm_*.pt")):
        ck = torch.load(f, map_location=device)
        model = SwarmWM(ck["kind"], **ck["cfg"]).to(device)
        model.load_state_dict(ck["state"])
        model.eval()
        res[f.stem] = {s: position_errors(model, d) for s, d in sets.items()}
        row = "  ".join(f"{s} {np.sqrt(np.nanmean(e[:, -1] ** 2)) * 100:.1f}" for s, e in
                        res[f.stem].items())  # fmt: skip
        print(f"{f.stem}: h{h} RMSE [cm] {row}")
        pickle.dump(res, open(path, "wb"))


if __name__ == "__main__":
    cmd, args = sys.argv[1], [a for a in sys.argv[2:] if "=" not in a]
    kw = dict(a.split("=") for a in sys.argv[2:] if "=" in a)
    if cmd == "collect":
        collect(args[0], int(kw.get("workers", "6")))
    elif cmd == "finetune":
        finetune(args[0], args[1], kw)
    elif cmd == "train":
        train(args[0], int(args[1]), int(args[2]), int(args[3]), args[4], kw)
    else:
        evaluate(args[0], kw)
