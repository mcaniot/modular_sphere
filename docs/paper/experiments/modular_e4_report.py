"""E4 report: online adaptation to unknown mass (data/swarm/modular/e4_*_s*.pkl).

Per condition, learning rate, arm and number of adaptation episodes k: h30 position RMSE [cm]
on the held-out episodes of the condition and on the nominal test set (forgetting = change from
k = 0), mean ± std over seeds, adaptation wall-clock and updated parameters. Then, per k, a paired
bootstrap over the held-out episodes (2000 resamples of whole episodes, the clusters of their
overlapping windows and spheres; errors averaged over seeds) of the RMSE difference
modular_64x2 - frame_256x4 on the held-out episodes.
Usage: uv run python docs/paper/experiments/modular_e4_report.py
"""

import pickle
from pathlib import Path

import numpy as np

from modular_sphere.swarm_modular import Episodes

OUT = Path(__file__).resolve().parents[3] / "data/swarm/modular"
res = {}
N_ADAPT = 20  # as modular_e4.py
for f in sorted(OUT.glob("e4_*_s*.pkl")):
    res.update(pickle.load(open(f, "rb")))
ARMS = ("modular_64x2", "frame_128x2", "frame_256x4", "frame_256x4_head", "world_256x4")


def rmse(e):
    return 100 * np.sqrt(np.nanmean(e**2))


def cell(v, sign=False):
    return f"{np.mean(v):{'+' if sign else ''}.2f} ± {np.std(v):.2f}"


lines = []
for cond in sorted({k[0] for k in res}):
    for lr in sorted({k[3] for k in res if k[0] == cond and k[3] > 0}, reverse=True):
        lines += [f"## {cond}, lr {lr:g}", "",
                  "| arm | k | seeds | held-out h30 | nominal h30 | forgetting | time [s] "
                  "| params |",
                  "|---|---|---|---|---|---|---|---|"]  # fmt: skip
        for arm in ARMS:
            seeds = sorted({k[2] for k in res if k[:2] == (cond, arm)})
            ks = sorted({k[4] for k in res if k[:2] == (cond, arm) and k[3] == lr})
            for k in [0, *ks]:
                rows = [res.get((cond, arm, s, 0.0 if k == 0 else lr, k)) for s in seeds]
                rows = [(s, r) for s, r in zip(seeds, rows, strict=True) if r is not None]
                if not rows:
                    continue
                m = [rmse(r["mass"]) for _, r in rows]
                nom = [rmse(r["nominal"]) for _, r in rows]
                fg = [
                    rmse(r["nominal"]) - rmse(res[cond, arm, s, 0.0, 0]["nominal"]) for s, r in rows
                ]
                t = np.mean([r["time"] for _, r in rows])
                lines.append(f"| {arm} | {k} | {len(rows)} | {cell(m)} | {cell(nom)} | "
                             f"{cell(fg, True)} | {t:.0f} | "
                             f"{rows[0][1]['params']:,} |")  # fmt: skip
        lines += [
            "",
            "Paired bootstrap, held-out h30 RMSE modular_64x2 - frame_256x4 [cm] (95 % CI):",
            "",
        ]
        rng = np.random.default_rng(0)
        for k in sorted({k[4] for k in res if k[0] == cond and k[3] == lr}):
            pair = []
            for arm in ("modular_64x2", "frame_256x4"):
                seeds = sorted({kk[2] for kk in res if kk[:2] == (cond, arm)})
                runs = [res.get((cond, arm, s, lr, k)) for s in seeds]
                if any(r is None for r in runs):
                    break
                pair.append(np.mean([r["mass"] ** 2 for r in runs], 0).ravel())
            if len(pair) < 2:
                continue
            ep = Episodes.load(f"{OUT}/{cond}.npz").valid[N_ADAPT:]
            e_idx, t_idx = np.nonzero(ep)
            starts = e_idx[t_idx % 10 == 0]  # episode of each test window, as windows()
            cluster = np.repeat(starts, pair[0].size // len(starts))  # (M, N) raveled
            sums = [np.bincount(cluster, np.nan_to_num(p), len(ep)) for p in pair]
            counts = np.bincount(cluster, ~np.isnan(pair[0]), len(ep))
            idx = rng.integers(0, len(ep), (2000, len(ep)))
            a, b = (s[idx].sum(1) / counts[idx].sum(1) for s in sums)
            d = 100 * (np.sqrt(a) - np.sqrt(b))
            diff = 100 * (
                np.sqrt(sums[0].sum() / counts.sum()) - np.sqrt(sums[1].sum() / counts.sum())
            )
            lines.append(
                f"- k = {k}: {diff:+.2f} "
                f"[{np.percentile(d, 2.5):+.2f}, {np.percentile(d, 97.5):+.2f}]"
            )
        lines.append("")
text = "# E4: online adaptation to unknown mass\n\n" + "\n".join(lines)
(OUT / "e4_report.md").write_text(text)
print(text)
