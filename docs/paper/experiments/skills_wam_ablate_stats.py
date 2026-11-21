"""Paired statistics of skills_wam_ablate.py against the full WAM and BC of skills_wam.py.

Usage: uv run --with scipy python skills_wam_ablate_stats.py [skill=climb]
"""

import pickle
import sys

from scipy.stats import binomtest

root = "/home/mcaniot/Documents/modular_sphere/data/swarm/skills"
name = sys.argv[1] if len(sys.argv) > 1 else "climb"
ref = pickle.load(open(f"{root}/wam_eval_{name}_50.pkl", "rb"))
abl = pickle.load(open(f"{root}/wam_ablate_{name}_50.pkl", "rb"))
for var, (ok, st) in abl.items():
    line = f"{name} {var}: {ok.sum()}/50 steps {st[ok].mean():.1f}"
    for base in ("wam", "bc"):
        bo = ref[base][0]
        b, c = int((ok & ~bo).sum()), int((~ok & bo).sum())
        p = binomtest(b, b + c, 0.5).pvalue if b + c else 1.0
        line += f" | vs {base}: b={b} c={c} p={p:.2g}"
    print(line)
