"""S3b: BC prior per skill (target-relative features), saved next to the skill WMs."""

import pickle
import sys

import numpy as np

from modular_sphere.swarm_data import SwarmDataset
from modular_sphere.swarm_skills import SKILLS, SkillCall, SkillPrior

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
