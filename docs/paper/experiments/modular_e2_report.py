"""E2 report: every finished data/swarm/modular/e2_{COND}_{MODE}.pkl against e2_{COND}_mosaic.pkl
on the same seeds (success, exact McNemar, steps of the episodes both succeed).

Usage: uv run --with scipy python docs/paper/experiments/modular_e2_report.py
"""

import pickle
from pathlib import Path

from scipy.stats import binomtest

OUT = Path(__file__).resolve().parents[3] / "data/swarm/modular"
rows = ["| condition | mode | success | mosaic | rescued | broken | McNemar p | steps (both ok) |",
        "|---|---|---|---|---|---|---|---|"]  # fmt: skip
for base in sorted(OUT.glob("e2_*_mosaic.pkl")):
    cond = base.stem[3:-7]
    ref = pickle.load(open(base, "rb"))
    for f in sorted(OUT.glob(f"e2_{cond}_*.pkl")):
        if f == base:
            continue
        r = pickle.load(open(f, "rb"))
        n = len(r["success"])
        a, b = ref["success"][:n], r["success"]
        up, down = int((~a & b).sum()), int((a & ~b).sum())
        p = binomtest(up, up + down).pvalue if up + down else 1.0
        ok = a & b
        steps = f"{r['steps'][ok].mean():.0f} vs {ref['steps'][:n][ok].mean():.0f}"
        rows.append(f"| {cond} | {f.stem[4 + len(cond):]} | {b.sum()}/{n} | {a.sum()}/{n} | {up} | "
                    f"{down} | {p:.3f} | {steps} |")  # fmt: skip
text = "## E2: verified WAM chunks against the WAM alone\n\n" + "\n".join(rows) + "\n"
(OUT / "e2_report.md").write_text(text)
print(text)
