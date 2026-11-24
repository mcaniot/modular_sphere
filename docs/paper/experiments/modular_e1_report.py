"""E1 report: open-loop position RMSE [cm] of every world-model variant (mean +- sd over training
seeds) per set and horizon, per skill of the sphere at the window start, and on windows where the
sphere changes skill call within the horizon. Paired comparison of variants per sphere-window
(sign test on the squared errors at the last horizon, pooled over seeds).

Usage: modular_e1_report.py [h=1,10,30] [vs=modular_64x2]
"""

import pickle
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.stats import binomtest

from modular_sphere.swarm_modular import SKILL_IDS, Episodes, windows

OUT = Path(__file__).resolve().parents[3] / "data/swarm/modular"
kw = dict(a.split("=") for a in sys.argv[1:])
hs = [int(v) for v in kw.get("h", "1,10,30").split(",")]
ref_model = kw.get("vs", "modular_64x2")
res = pickle.load(open(OUT / "e1.pkl", "rb"))
params = {f.stem: __import__("torch").load(f, map_location="cpu")["params"]
          for f in OUT.glob("wm_*.pt")}  # fmt: skip

groups = defaultdict(list)
for name in res:
    groups[re.sub(r"^wm_|_s\d+$", "", name)].append(name)
sets = list(next(iter(res.values())))
info = {}
for s in sets:
    w = windows(Episodes.load(f"{OUT}/{s}.npz"), max(hs), 10)
    skill = (w["cat"][:, 0] // 2).numpy()
    changed = (w["group"] != w["group"][:, :1]).cumsum(1).numpy() > 0  # (M, h, N)
    info[s] = skill, changed


def rmse(e):
    return np.sqrt(np.nanmean(e**2)) * 100


def cell(names, s, h, sel=None):
    vals = []
    for n in names:
        e = res[n][s][:, h - 1]
        vals.append(rmse(e if sel is None else e[sel]))
    return f"{np.mean(vals):.2f}±{np.std(vals):.2f}" if len(vals) > 1 else f"{vals[0]:.2f}"


print(f"## Open-loop position RMSE [cm], horizons {hs}\n")
print("| variant | params | " + " | ".join(sets) + " |")
print("|---|---|" + "---|" * len(sets))
for g, names in sorted(groups.items()):
    row = [" / ".join(cell(names, s, h) for h in hs) for s in sets]
    print(f"| {g} (n={len(names)}) | {params[names[0]]:,} | " + " | ".join(row) + " |")

h = max(hs)
print(f"\n## Per skill at window start and at skill changes, h={h}, set test and courses\n")
inv = {v: k for k, v in SKILL_IDS.items()}
for s in ("test", "courses"):
    if s not in info:
        continue
    skill, changed = info[s]
    cols = [(inv[k], skill == k) for k in range(len(SKILL_IDS))]
    cols += [("same call", ~changed[:, h - 1]), ("call change", changed[:, h - 1])]
    print(f"### {s}\n\n| variant | " + " | ".join(c for c, _ in cols) + " |")
    print("|---|" + "---|" * len(cols))
    for g, names in sorted(groups.items()):
        print(f"| {g} | " + " | ".join(cell(names, s, h, m) for _, m in cols) + " |")
    print()

print(f"## Paired sign test against {ref_model} (squared error at h={h}, per sphere-window)\n")
if ref_model in groups:
    for g, names in sorted(groups.items()):
        if g == ref_model:
            continue
        row = []
        for s in sets:
            wins = n_tot = 0
            for a, b in zip(sorted(groups[ref_model]), sorted(names), strict=False):
                ea, eb = res[a][s][:, h - 1].ravel(), res[b][s][:, h - 1].ravel()
                ok = np.isfinite(ea) & np.isfinite(eb) & (ea != eb)
                wins += int((ea[ok] < eb[ok]).sum())
                n_tot += int(ok.sum())
            p = binomtest(wins, n_tot).pvalue if n_tot else np.nan
            row.append(f"{wins / max(n_tot, 1):.2f} (p={p:.1g})")
        print(f"| {g} | " + " | ".join(row) + " |")
