"""Pull skill data with the energy-shaped pull (the S2 recipe, same seeds and noise levels).

Usage: skills_shaped.py. Saves data/swarm/skills/pull_shaped_{train,val}.npz and
pull_shaped_outcomes.pkl, read by ``skills_chain.py ... pull=shaped``.
"""

import pickle
import time
from pathlib import Path

from modular_sphere.swarm_data import SwarmDataset
from modular_sphere.swarm_skills import collect_skill

out = Path(__file__).resolve().parents[3] / "data/swarm/skills"
t0 = time.time()
parts = [
    collect_skill("pull", 150, seed=1000 * k, noise=nz) for k, nz in enumerate((0.1, 0.2, 0.3))
]
val = collect_skill("pull", 60, seed=90_000, noise=0.2)
SwarmDataset.concat([p.data for p in parts]).save(out / "pull_shaped_train.npz")
val.data.save(out / "pull_shaped_val.npz")
with open(out / "pull_shaped_outcomes.pkl", "wb") as f:
    pickle.dump({"train": [(p.start, p.target, p.steps, p.success) for p in parts],
                 "val": (val.start, val.target, val.steps, val.success)}, f)  # fmt: skip
for nz, p in zip((0.1, 0.2, 0.3, "val 0.2"), (*parts, val), strict=True):
    print(f"noise {nz}: success {p.success.sum()}/{len(p.success)}, steps {p.steps.mean():.1f}")
print(f"({time.time() - t0:.0f}s)")
