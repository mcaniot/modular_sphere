"""Paired statistics of skills_adapt.py runs: MOSAIC against the expert and against MOSAIC before.

Usage: skills_adapt_stats.py TAG (reads data/swarm/skills/adapt_{COND}{TAG}.pkl, modes expert,
mosaic0, mosaic). Exact McNemar on success, sign test on steps over the seeds both solve.
"""

import pickle
import sys
from pathlib import Path

from scipy.stats import binomtest

root = str(Path(__file__).resolve().parents[3] / "data/swarm/skills")
tag = sys.argv[1]


def mcnemar(a, b):
    x, y = int((a & ~b).sum()), int((~a & b).sum())
    return x, y, binomtest(min(x, y), x + y).pvalue if x + y else 1.0


for c in ["base", "noise0.005", "fric0.5", "fric1.5", "mass0.7", "mass1.5", "rise0.10", "n5", "n6"]:
    d = pickle.load(open(f"{root}/adapt_{c}{tag}.pkl", "rb"))
    e, o, m = d["expert"], d["mosaic0"], d["mosaic"]
    both = e["success"] & m["success"]
    f = int((m["steps"][both] < e["steps"][both]).sum())
    s = int((m["steps"][both] > e["steps"][both]).sum())
    ratio = (m["steps"][both] / e["steps"][both]).mean()
    p = binomtest(f, f + s).pvalue if f + s else 1.0  # sign test, no tie-free pair: p = 1
    print(f"{c}: expert {e['success'].sum()} before {o['success'].sum()} mosaic "
          f"{m['success'].sum()} | vs expert {mcnemar(e['success'], m['success'])} faster "
          f"{f}/{f + s} p={p:.2g} ratio {ratio:.2f} | vs before "
          f"{mcnemar(o['success'], m['success'])}")  # fmt: skip
