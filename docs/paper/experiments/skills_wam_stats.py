"""Paired statistics of skills_wam.py results (run with: uv run --with scipy python ...)."""

import pickle
from pathlib import Path

from scipy.stats import beta, binomtest

root = str(Path(__file__).resolve().parents[3] / "data/swarm/skills")


def ci(k, n):
    lo = beta.ppf(0.025, k, n - k + 1) if k > 0 else 0.0
    hi = beta.ppf(0.975, k + 1, n - k) if k < n else 1.0
    return lo, hi


for name in ("climb", "pull", "roll"):
    r = pickle.load(open(f"{root}/wam_eval_{name}_50.pkl", "rb"))
    for mode, (ok, st) in r.items():
        lo, hi = ci(ok.sum(), len(ok))
        print(f"{name} {mode}: {ok.sum()}/50 CI [{lo:.2f}, {hi:.2f}] steps {st[ok].mean():.1f}")
    bo, bs = r["bc"]
    for mode in ("wam", "wam_ens"):
        ok, st = r[mode]
        b, c = int((ok & ~bo).sum()), int((~ok & bo).sum())
        p = binomtest(b, b + c, 0.5).pvalue if b + c else 1.0
        both = ok & bo
        faster, slower = int((st[both] < bs[both]).sum()), int((st[both] > bs[both]).sum())
        ps = binomtest(faster, faster + slower, 0.5).pvalue if faster + slower else 1.0
        d = (st[both] - bs[both]).mean()
        print(
            f"  {mode} vs bc: McNemar b={b} c={c} p={p:.2g}; both ok {both.sum()}, "
            f"faster {faster} slower {slower} sign p={ps:.2g}, mean diff {d:+.1f}"
        )
    ok, st = r["wam_ens"]
    o2, s2 = r["wam"]
    b, c = int((ok & ~o2).sum()), int((~ok & o2).sum())
    print(f"  wam_ens vs wam: b={b} c={c} p={binomtest(b, b + c, 0.5).pvalue if b + c else 1:.2g}")
