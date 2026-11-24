"""Figures of the paper's swarm world-model experiments (E1-E4) and a swarm climb filmstrip.

Usage: uv run --with matplotlib python docs/paper/experiments/modular_figures.py [NAME ...]
(NAME among scene, e1, e2, e3, e4; all by default). Reads data/swarm/modular/{e1,e3}.pkl,
e2_report.md, e4_*.pkl; writes docs/images/modular_*.png.
"""

import pickle
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "data/swarm/modular"
IMG = ROOT / "docs/images"
COLORS = {"modular": "#1b9e77", "frame": "#7570b3", "id": "#e6ab02", "world": "#d95f02"}
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})


def rmse(e, axis=None):
    return 100 * np.sqrt(np.nanmean(e**2, axis=axis))


def scene():
    import mujoco

    from modular_sphere.swarm import STEP_X, SphereSwarmStepEnv, SwarmClimbPolicy

    env = SphereSwarmStepEnv(4)
    obs, _ = env.reset(seed=3)
    pol = SwarmClimbPolicy(4, env.radius)
    ren = mujoco.Renderer(env.model, 360, 480)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (STEP_X - 0.35, 0.0, 0.08)
    cam.distance, cam.azimuth, cam.elevation = 2.2, -60.0, -18.0
    frames, t, done = [], 0, False
    while env.agents and not done:
        if t % 10 == 0:
            ren.update_scene(env.data, cam)
            frames.append(ren.render().copy())
        obs, _, term, trunc, _ = env.step(pol.act(obs))
        done = any(term.values()) or any(trunc.values())
        t += 1
    ren.update_scene(env.data, cam)
    frames.append(ren.render().copy())
    pick = np.linspace(0, len(frames) - 1, 5).round().astype(int)
    fig, axes = plt.subplots(1, 5, figsize=(10, 2.0))
    for ax, i in zip(axes, pick, strict=True):
        ax.imshow(frames[i])
        ax.set_title(f"t = {i * 10 * 0.05:.0f} s")
        ax.axis("off")
    fig.suptitle(f"Scripted swarm climb, four linked spheres ({t} steps, 20 Hz)")
    fig.tight_layout()
    fig.savefig(IMG / "modular_scene.png", dpi=130)


def e1():
    res = pickle.load(open(OUT / "e1.pkl", "rb"))
    variants = [
        "modular_64x2",
        "frame_128x2",
        "frame_256x4",
        "id_256x4",
        "world_256x4",
        "world_512x6",
    ]
    sets = ("test", "courses", "mass1.5")
    fig, axes = plt.subplots(1, len(sets), figsize=(10, 3.2), sharey=False)
    h = np.arange(1, 31) * 0.05
    for ax, s in zip(axes, sets, strict=True):
        for v in variants:
            runs = [res[k][s] for k in res if k.startswith(f"wm_{v}_s") and not k.endswith("_x")]
            curves = np.array([rmse(e, (0, 2)) for e in runs])
            kind = v.split("_")[0]
            ls = ":" if v == "world_512x6" else ("--" if v.endswith("128x2") else "-")
            ax.plot(h, curves.mean(0), ls, color=COLORS[kind], label=v.replace("_", " "))
            if len(curves) > 1:
                ax.fill_between(h, curves.min(0), curves.max(0), color=COLORS[kind], alpha=0.15)
        ax.set_title({"test": "test (in distribution)", "courses": "unseen 1-3 step courses",
                      "mass1.5": "mass x1.5"}[s])  # fmt: skip
        ax.set_xlabel("open-loop horizon [s]")
    axes[0].set_ylabel("position RMSE [cm]")
    axes[1].legend(fontsize=7, frameon=False)
    fig.suptitle("E1: open-loop error, mean and range over 3 training seeds")
    fig.tight_layout()
    fig.savefig(IMG / "modular_e1.png", dpi=130)


def e2():
    rows = [r.split("|")[1:-1] for r in (OUT / "e2_report.md").read_text().splitlines()
            if r.startswith("| mass") or r.startswith("| noise")]  # fmt: skip
    labels = {"oracle_s05m3": "MuJoCo oracle", "wm_wm_modular_64x2_s0_s05m3": "modular 64x2",
              "wm_wm_frame_256x4_s0_s05m3": "frame 256x4",
              "wm_wm_world_256x4_s0_s05m3": "world 256x4"}  # fmt: skip
    fig, axes = plt.subplots(1, 2, figsize=(9, 2.8), sharey=True)
    for ax, cond, title in zip(axes, ("noise0.005", "mass1.5"), ("noise 5 mm", "mass x1.5"),
                               strict=True):  # fmt: skip
        sel = {c[1].strip(): c for c in rows if c[0].strip() == cond and c[1].strip() in labels}
        names = ["MOSAIC alone"] + [labels[m] for m in labels]
        base = int(next(iter(sel.values()))[3].split("/")[0])
        vals = [base] + [int(sel[m][2].split("/")[0]) for m in labels]
        ps = [None] + [float(sel[m][6]) for m in labels]
        cols = ["#999999", "#444444"] + [COLORS[labels[m].split()[0]] for m in list(labels)[1:]]
        bars = ax.barh(names, vals, color=cols)
        for b, p in zip(bars, ps, strict=True):
            pv = "" if p is None else ("  p<0.001" if p < 1e-3 else f"  p={p:.2g}")
            txt = f"{b.get_width():.0f}/50{pv}"
            ax.text(
                b.get_width() + 0.5, b.get_y() + b.get_height() / 2, txt, va="center", fontsize=7
            )
        ax.set_xlim(0, 62)
        ax.invert_yaxis()
        ax.set_title(title)
        ax.set_xlabel("successes on 50 paired random courses")
    fig.suptitle(
        "E2 (negative): verifying executor chunks with a world model, exact McNemar vs MOSAIC"
    )
    fig.tight_layout()
    fig.savefig(IMG / "modular_e2.png", dpi=130)


def e3():
    res = pickle.load(open(OUT / "e3.pkl", "rb"))
    models = ["modular_64x2", "frame_128x2", "world_256x4"]
    fig, axes = plt.subplots(1, 2, figsize=(9, 3))
    x = np.arange(3)
    for j, (ph, lab) in enumerate((("p1", "phase 1 (no pull)"), ("p2_k10", "+ pull, 10 episodes"),
                                    ("p2_kall", "+ pull, all episodes"))):  # fmt: skip
        for ax, st in zip(axes, ("test_nopull", "test_pull"), strict=True):
            v = [[rmse(res[m, s][ph][st][:, -1]) for s in range(3)] for m in models]
            ax.bar(x + (j - 1) * 0.27, np.mean(v, 1), 0.27, yerr=np.std(v, 1), label=lab,
                   color=["#cccccc", "#88aacc", "#335577"][j], capsize=2)  # fmt: skip
    for ax, t in zip(axes, ("known skills (forgetting)", "new skill: pull"), strict=True):
        ax.set_xticks(x, [m.replace("_", " ") for m in models])
        ax.set_title(t)
    axes[0].set_ylabel("h30 position RMSE [cm]")
    axes[0].legend(fontsize=7, frameon=False)
    fig.suptitle("E3: adding the pull skill (3 seeds); the modular model only trains its pull net")
    fig.tight_layout()
    fig.savefig(IMG / "modular_e3.png", dpi=130)


def e4():
    res = {}
    for f in sorted(OUT.glob("e4_*_s*.pkl")):
        res.update(pickle.load(open(f, "rb")))
    arms = {"modular_64x2": ("modular", "-"), "frame_128x2": ("frame", "--"),
            "frame_256x4": ("frame", "-"), "frame_256x4_head": ("frame", ":"),
            "world_256x4": ("world", "-")}  # fmt: skip
    lr = 3e-4
    conds = sorted({k[0] for k in res})
    fig, axes = plt.subplots(2, len(conds), figsize=(4.5 * len(conds), 5.6), squeeze=False)
    for j, cond in enumerate(conds):
        for arm, (kind, ls) in arms.items():
            ks = sorted({k[4] for k in res if k[:2] == (cond, arm) and k[3] in (0.0, lr)})
            seeds = sorted({k[2] for k in res if k[:2] == (cond, arm)})
            ks = [
                k for k in ks if all((cond, arm, s, 0.0 if k == 0 else lr, k) in res for s in seeds)
            ]
            if not ks:
                continue
            for i, key in enumerate(("mass", "nominal")):
                v = np.array([[rmse(res[cond, arm, s, 0.0 if k == 0 else lr, k][key]) for k in ks]
                              for s in seeds])  # fmt: skip
                axes[i, j].errorbar(np.array(ks) + 0.5, v.mean(0), v.std(0), fmt=ls,
                                    color=COLORS[kind], label=arm.replace("_", " "),
                                    capsize=2)  # fmt: skip
        axes[0, j].set_title(f"{cond.replace('mass', 'mass x')}: held-out episodes")
        axes[1, j].set_title("nominal test (forgetting)")
        for i in range(2):
            axes[i, j].set_xscale("log")
            axes[i, j].set_xticks(
                [0.5, 1.5, 2.5, 5.5, 10.5, 20.5], ["0", "1", "2", "5", "10", "20"]
            )
            axes[i, j].set_ylabel("h30 position RMSE [cm]")
        axes[1, j].set_xlabel("adaptation episodes k")
    axes[0, 0].legend(fontsize=7, frameon=False)
    n_seeds = len({k[2] for k in res})
    fig.suptitle(f"E4: online adaptation at lr {lr:g}, same update budget, mean ± std over "
                 f"{n_seeds} seeds")  # fmt: skip
    fig.tight_layout()
    fig.savefig(IMG / "modular_e4.png", dpi=130)


if __name__ == "__main__":
    for name in sys.argv[1:] or ["scene", "e1", "e2", "e3", "e4"]:
        globals()[name]()
        print(f"docs/images/modular_{name}.png")
