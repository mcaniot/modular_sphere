"""E3 report: adding the pull skill to world models trained without it (data/swarm/modular/e3.pkl).

h30 position RMSE [cm], mean and standard deviation over seeds, per model and phase (p1: before
pull; p2_k10: pull added with 10 episodes; p2_kall: with all of them), and the forgetting on the
windows without pull (p2 - p1, the backward transfer with the sign of an error).
Usage: uv run python docs/paper/experiments/modular_e3_report.py
"""

import pickle
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parents[3] / "data/swarm/modular"
res = pickle.load(open(OUT / "e3.pkl", "rb"))


def rmse(e):
    return 100 * np.sqrt(np.nanmean(e[:, -1] ** 2))


models = sorted({m for m, _ in res})
sets = ("test_nopull", "test_pull", "courses_nopull", "courses_pull")
lines = ["| model | phase | " + " | ".join(sets) + " |", "|---" * (len(sets) + 2) + "|"]
forget = ["| model | k | test_nopull forgetting | courses_nopull forgetting |", "|---|---|---|---|"]
for m in models:
    seeds = sorted(s for mm, s in res if mm == m)
    for ph in ("p1", "p2_k10", "p2_kall"):
        v = np.array([[rmse(res[m, s][ph][st]) for st in sets] for s in seeds])
        cells = " | ".join(f"{a:.1f} ± {b:.1f}" for a, b in zip(v.mean(0), v.std(0), strict=True))
        lines.append(f"| {m} (n={len(seeds)}) | {ph} | {cells} |")
    for ph in ("p2_k10", "p2_kall"):
        d = np.array([[rmse(res[m, s][ph][st]) - rmse(res[m, s]["p1"][st])
                       for st in ("test_nopull", "courses_nopull")] for s in seeds])  # fmt: skip
        cells = " | ".join(f"{a:+.1f} ± {b:.1f}" for a, b in zip(d.mean(0), d.std(0), strict=True))
        forget.append(f"| {m} | {ph[3:]} | {cells} |")
text = "\n".join(["## E3: h30 RMSE [cm]", "", *lines, "", "## Forgetting [cm]", "", *forget])
(OUT / "e3_report.md").write_text(text + "\n")
print(text)
