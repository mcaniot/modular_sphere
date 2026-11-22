"""WAM ablations: which ingredient makes the per-skill WAM solve climb (paper §5.5).

Usage: skills_wam_ablate.py DEVICE N_EPISODES [skills=climb] [variants=...] [seed=0]
Variants (one ingredient removed each, otherwise as skills_wam.py): ``nohist`` (history 1),
``nochunk`` (chunk 1), ``noframe`` (frame loss weight 0), ``bare`` (all three removed),
``full`` (control; with seed != 0 it
measures training-seed variance). MLP behaviour cloning (``SkillPrior``, the BC of skills_wam.py):
``mlp`` (its recipe: 30 epochs, lr 1e-3, batch 1024), ``mlp_long`` (the WAM recipe: 60 epochs,
lr 3e-4, batch 512), ``mlp_wide`` (hidden 512, WAM recipe). Executes the newest chunk (no
ensembling), on the same starts as skills_wam.py (env seed = episode, start sampler seed 123). Saves
``wam_ablate_{skill}_{N}.pkl``: {variant_seed: (success, steps)}, merged with previous runs.
"""

import os
import pickle
import sys
import time
from pathlib import Path

import numpy as np

from modular_sphere.swarm import SphereSwarmStepEnv
from modular_sphere.swarm_data import SwarmDataset
from modular_sphere.swarm_skills import (
    MAX_SKILL_STEPS,
    SkillCall,
    SkillController,
    SkillPrior,
    place,
    sample_start,
)
from modular_sphere.swarm_wam import SkillWAM, WAMController

device, n_eps = sys.argv[1], int(sys.argv[2])
kw = dict(a.split("=") for a in sys.argv[3:])
skills = kw.get("skills", "climb").split(",")
variants = kw.get("variants", "nohist,nochunk,noframe").split(",")
seed = int(kw.get("seed", "0"))
root = str(Path(__file__).resolve().parents[3] / "data/swarm/skills")
CFG = {
    "full": ({}, 1.0),
    "nohist": ({"history": 1}, 1.0),
    "nochunk": ({"chunk": 1}, 1.0),
    "noframe": ({}, 0.0),
    "bare": ({"history": 1, "chunk": 1}, 0.0),
}
MLP = {
    "mlp": (128, {}),
    "mlp_long": (128, {"epochs": 60, "lr": 3e-4, "batch_size": 512}),
    "mlp_wide": (512, {"epochs": 60, "lr": 3e-4, "batch_size": 512}),
}


class PriorAsWAM:
    """A ``SkillPrior`` behind the ``SkillWAM`` interface used by ``WAMController``."""

    history = chunk = 1

    def __init__(self, prior: SkillPrior) -> None:
        self.prior, self.feat_mean = prior, prior.mean

    def __call__(self, hist):
        return None, self.prior(hist[:, -1])[:, None]


def calls_of(name: str, n: int) -> list[SkillCall]:
    with open(f"{root}/{name}_outcomes.pkl", "rb") as f:
        out = pickle.load(f)
    targets = np.concatenate([p[1] for p in out["train"]])
    ag = (0,) if name == "roll" else (0, 1)
    calls = [SkillCall(name, ag, t) for t in targets]
    assert len(calls) == n
    return calls


def evaluate(wam, name: str) -> tuple[np.ndarray, np.ndarray]:
    ag = (0,) if name == "roll" else (0, 1)
    rng = np.random.default_rng(123)
    env = SphereSwarmStepEnv(n_spheres=2, max_steps=MAX_SKILL_STEPS + 1)
    ok, steps = np.zeros(n_eps, dtype=bool), np.zeros(n_eps, dtype=np.int64)
    for e in range(n_eps):
        env.reset(seed=e)
        start, target = sample_start(name, rng, env.radius)
        place(env, start)
        call = SkillCall(name, ag, target)
        judge, ctrl = SkillController(call, 2, env.radius), WAMController(wam, call, 2, None)
        for t in range(MAX_SKILL_STEPS):
            steps[e] = t + 1
            act = ctrl.act(env.spheres(), env.links)
            env.step({a: act[i] for i, a in enumerate(env.possible_agents)})
            pos, vel = env._kinematics()[:2]
            if judge.done(pos, vel):
                ok[e] = True
                break
    return ok, steps


for name in skills:
    ag = (0,) if name == "roll" else (0, 1)
    data = SwarmDataset.load(f"{root}/{name}_train.npz")
    calls = calls_of(name, data.valid.shape[0])
    path = f"{root}/wam_ablate_{name}_{n_eps}.pkl"
    results = pickle.load(open(path, "rb")) if os.path.exists(path) else {}
    for var in variants:
        t0 = time.time()
        if var in MLP:
            hidden, fit_kw = MLP[var]
            prior = SkillPrior(len(ag), hidden=hidden).to(device)
            losses = prior.fit(data, calls, seed=seed, **fit_kw)
            prior.save(f"{root}/prior_{name}_{var}_s{seed}.pt")
            model = PriorAsWAM(prior)
        else:
            cfg, frame_weight = CFG[var]
            model = SkillWAM(len(ag), **cfg).to(device)
            losses = model.fit(data, calls, frame_weight=frame_weight, seed=seed)
            model.save(f"{root}/wam_{name}_{var}_s{seed}.pt")
        ok, steps = evaluate(model, name)
        results[f"{var}_s{seed}"] = (ok, steps)
        pickle.dump(results, open(path, "wb"))
        print(
            f"{name} {var} seed {seed}: loss {losses[-1]:.4f}; success {ok.sum()}/{n_eps}, "
            f"steps on success {steps[ok].mean() if ok.any() else float('nan'):.1f} "
            f"({time.time() - t0:.0f}s)",
            flush=True,
        )
