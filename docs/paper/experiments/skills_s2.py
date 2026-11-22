"""S2: skill data + one small world model per skill; open-loop RMSE vs the whole-swarm WM."""

import pickle
import sys
import time
from pathlib import Path

from modular_sphere.swarm_data import SwarmDataset
from modular_sphere.swarm_skills import SKILLS, collect_skill
from modular_sphere.swarm_torch import SwarmWorldModel, evaluate_world_model

device = sys.argv[1]
out = Path(__file__).resolve().parents[3] / "data/swarm/skills"
out.mkdir(parents=True, exist_ok=True)
whole = SwarmWorldModel.load(str(out.parent / "wmc0.pt"), device)
for name in SKILLS:
    t0 = time.time()
    parts = [
        collect_skill(name, 150, seed=1000 * k, noise=nz) for k, nz in enumerate((0.1, 0.2, 0.3))
    ]
    val = collect_skill(name, 60, seed=90_000, noise=0.2)
    train = SwarmDataset.concat([p.data for p in parts])
    train.save(out / f"{name}_train.npz")
    val.data.save(out / f"{name}_val.npz")
    with open(out / f"{name}_outcomes.pkl", "wb") as f:
        pickle.dump(
            {
                "train": [(p.start, p.target, p.steps, p.success) for p in parts],
                "val": (val.start, val.target, val.steps, val.success),
            },
            f,
        )
    print(
        f"{name}: data {int(train.valid.sum())} transitions ({time.time() - t0:.0f}s)", flush=True
    )
    t0 = time.time()
    wm = SwarmWorldModel(hidden=64, epochs=30, device=device)
    wm.fit(train)
    wm.save(str(out / f"wm_{name}.pt"))
    for label, model in (("skill", wm), ("whole", whole)):
        res = evaluate_world_model(model, val.data, [1, 10, 30], stride=5)
        print(
            f"{name} {label} WM: "
            + ", ".join(
                f"h{k} pos {v['pos'] * 1000:.1f} mm link {v['link_acc']:.3f} cv {v['const_vel_pos'] * 1000:.0f}"
                for k, v in res.items()
            )
            + f" ({time.time() - t0:.0f}s)",
            flush=True,
        )
