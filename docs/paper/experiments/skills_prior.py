"""S3b: BC prior per skill (target-relative features), saved next to the skill WMs."""

import pickle
import sys

import numpy as np

from modular_sphere.swarm_data import SwarmDataset
import torch

from modular_sphere.swarm_skills import SKILLS, SkillCall, SkillPrior, SkillValue, skill_features

device = sys.argv[1]
root = "/home/mcaniot/Documents/modular_sphere/data/swarm/skills"
for name in SKILLS:
    data = SwarmDataset.load(f"{root}/{name}_train.npz")
    with open(f"{root}/{name}_outcomes.pkl", "rb") as f:
        targets = np.concatenate([p[1] for p in pickle.load(f)["train"]])
    ag = (0,) if name == "roll" else (0, 1)
    calls = [SkillCall(name, ag, t) for t in targets]
    assert len(calls) == data.valid.shape[0]
    prior = SkillPrior(len(ag)).to(device)
    losses = prior.fit(data, calls, epochs=40)
    prior.save(f"{root}/prior_{name}.pt")
    print(f"{name}: prior loss {losses[0]:.4f} -> {losses[-1]:.4f}", flush=True)
    value = SkillValue(len(ag)).to(device)
    losses = value.fit(data, calls, epochs=40)
    value.save(f"{root}/value_{name}.pt")
    val = SwarmDataset.load(f"{root}/{name}_val.npz")
    with open(f"{root}/{name}_outcomes.pkl", "rb") as f:
        vt = pickle.load(f)["val"][1]
    err = []
    with torch.no_grad():
        for e, t in enumerate(vt):
            v = val.valid[e]
            x = torch.as_tensor(val.spheres[e, :-1][v], dtype=torch.float32, device=device)
            lk = torch.as_tensor(val.links[e, :-1][v], device=device)
            pred = value(skill_features(SkillCall(name, ag, t), x, lk)).cpu().numpy()
            n = len(pred)
            true = n - np.arange(n) if val.success[e] else np.full(n, 200.0)
            err.append(pred - true)
    err = np.concatenate(err)
    print(f"{name}: value loss {losses[0]:.4f} -> {losses[-1]:.4f}; val steps-to-go MAE "
          f"{np.abs(err).mean():.1f}, RMSE {np.sqrt((err ** 2).mean()):.1f}", flush=True)
