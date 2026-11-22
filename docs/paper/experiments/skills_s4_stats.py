"""Paired statistics of skills_s4.py (run with: uv run --with scipy python skills_s4_stats.py
[NAME], NAME of the pickle, default s4_50)."""

import pickle
import sys
from pathlib import Path

from scipy.stats import beta, binomtest

name = sys.argv[1] if len(sys.argv) > 1 else "s4_50"
root = Path(__file__).resolve().parents[3] / "data/swarm/skills"
r = pickle.load(open(root / f"{name}.pkl", "rb"))


def ci(k, n):
    lo = beta.ppf(0.025, k, n - k + 1) if k > 0 else 0.0
    hi = beta.ppf(0.975, k + 1, n - k) if k < n else 1.0
    return lo, hi


for mode, res in r.items():
    ok, st = res["success"], res["steps"]
    lo, hi = ci(ok.sum(), len(ok))
    line = (f"{mode}: {ok.sum()}/{len(ok)} [{lo:.2f}, {hi:.2f}] steps {st[ok].mean():.1f}, "
            f"moving {res['moving'].mean():.2f}")  # fmt: skip
    if res["predicted"].any():
        err = res["predicted"][ok] - st[ok]
        line += f", predicted-real {err.mean():+.1f} (MAE {abs(err).mean():.1f})"
        line += f", restarts {res['restarts'].sum()}"
    print(line)
    for base in ("expert", "dagger"):
        if base == mode or base not in r:
            continue
        bo, bs = r[base]["success"], r[base]["steps"]
        b, c = int((ok & ~bo).sum()), int((~ok & bo).sum())
        p = binomtest(b, b + c, 0.5).pvalue if b + c else 1.0
        both = ok & bo
        f, s = int((st[both] < bs[both]).sum()), int((st[both] > bs[both]).sum())
        ps = binomtest(f, f + s, 0.5).pvalue if f + s else 1.0
        ratio = (st[both] / bs[both]).mean()
        print(f"  vs {base}: McNemar {b}-{c} p={p:.2g}; both {both.sum()}, faster {f} slower {s} "
              f"sign p={ps:.2g}, steps ratio {ratio:.2f}")  # fmt: skip
