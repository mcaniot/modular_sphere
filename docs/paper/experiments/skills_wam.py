"""S3c: one small world-action model (WAM) per skill; MuJoCo success vs the BC skill policy.

Usage: skills_wam.py DEVICE N_EPISODES [train=0|1] [skills=climb,pull,roll]
Same starts as skills_s3.py (env seed = episode, start sampler seed 123).
"""

import pickle
import sys
import time
from pathlib import Path

import numpy as np
import torch

from modular_sphere.swarm import SphereSwarmStepEnv
from modular_sphere.swarm_data import SwarmDataset
from modular_sphere.swarm_skills import (
    MAX_SKILL_STEPS,
    SkillCall,
    SkillController,
    SkillPrior,
    place,
    sample_start,
    skill_features,
)  # fmt: skip
from modular_sphere.swarm_wam import SkillWAM, WAMController

device, n_eps = sys.argv[1], int(sys.argv[2])
kw = dict(a.split("=") for a in sys.argv[3:])
train = kw.get("train", "1") == "1"
skills = kw.get("skills", "climb,pull,roll").split(",")
root = str(Path(__file__).resolve().parents[3] / "data/swarm/skills")


def calls_of(name: str, split: str, n: int) -> list[SkillCall]:
    with open(f"{root}/{name}_outcomes.pkl", "rb") as f:
        out = pickle.load(f)
    targets = np.concatenate([p[1] for p in out["train"]]) if split == "train" else out["val"][1]
    ag = (0,) if name == "roll" else (0, 1)
    calls = [SkillCall(name, ag, t) for t in targets]
    assert len(calls) == n
    return calls


for name in skills:
    ag = (0,) if name == "roll" else (0, 1)
    if train:
        t0 = time.time()
        data = SwarmDataset.load(f"{root}/{name}_train.npz")
        val = SwarmDataset.load(f"{root}/{name}_val.npz")
        wam = SkillWAM(len(ag)).to(device)
        losses = wam.fit(data, calls_of(name, "train", data.valid.shape[0]))
        wam.save(f"{root}/wam_{name}.pt")
        with torch.no_grad():
            h, fut, act, mask = wam._windows(val, calls_of(name, "val", val.valid.shape[0]),
                                             torch.device(device))  # fmt: skip
            frames, out = wam(h)
            pred = h[:, -1:] + frames * wam.delta_std
            pos_err = (pred - fut)[..., : 3 * len(ag)].unflatten(-1, (len(ag), 3)).norm(dim=-1)
            act_err = ((out[..., :3] - act[..., :3]) ** 2).mean(-1).sqrt()
            m = mask.bool()
        print(
            f"{name}: WAM loss {losses[0]:.4f} -> {losses[-1]:.4f}; val future pos error "
            f"step1 {pos_err[:, 0].mean() * 1000:.1f} mm, step10 {pos_err[:, -1].mean() * 1000:.1f} mm; "
            f"action RMSE {act_err[m].mean():.3f} ({time.time() - t0:.0f}s)",
            flush=True,
        )
    wam = SkillWAM.load(f"{root}/wam_{name}.pt", device)
    prior = SkillPrior.load(f"{root}/prior_{name}.pt", device)
    per_mode = {}
    for mode in ("bc", "wam", "wam_ens"):
        rng = np.random.default_rng(123)
        env = SphereSwarmStepEnv(n_spheres=2, max_steps=MAX_SKILL_STEPS + 1)
        res = []
        for e in range(n_eps):
            env.reset(seed=e)
            start, target = sample_start(name, rng, env.radius)
            place(env, start)
            call = SkillCall(name, ag, target)
            judge = SkillController(call, 2, env.radius)
            ctrl = WAMController(wam, call, 2, ensemble=0.1 if mode == "wam_ens" else None)
            ok = False
            for t in range(MAX_SKILL_STEPS):
                st = env.spheres()
                if mode == "bc":
                    with torch.no_grad():
                        f = skill_features(call, torch.as_tensor(st, dtype=torch.float32,
                                                                 device=device),
                                           torch.as_tensor(env.links, device=device))  # fmt: skip
                        act = np.zeros((2, 4))
                        act[list(ag)] = prior(f).cpu().numpy()
                        act[:, 3] = np.where(act[:, 3] > 0, 1.0, -1.0)
                else:
                    act = ctrl.act(st, env.links)
                env.step({a: act[i] for i, a in enumerate(env.possible_agents)})
                pos, vel = env._kinematics()[:2]
                if judge.done(pos, vel):
                    ok = True
                    break
            res.append((ok, t + 1))
        r = np.array(res)
        ok = r[:, 0].astype(bool)
        per_mode[mode] = (ok, r[:, 1])
        print(
            f"{name} {mode}: success {ok.sum()}/{n_eps}, steps on success "
            f"{r[ok, 1].mean() if ok.any() else float('nan'):.0f}",
            flush=True,
        )
    with open(f"{root}/wam_eval_{name}_{n_eps}.pkl", "wb") as f:
        pickle.dump(per_mode, f)
