"""Figures of the paper's swarm world-model experiments (E1-E4) and a swarm climb filmstrip.

Usage: uv run --with matplotlib python docs/paper/experiments/modular_figures.py [NAME ...]
(NAME among scene, arch, mosaic, e1, e2, e3, e4, gifs; all by default). Reads
data/swarm/modular/{e1,e3}.pkl, e2_report.md, e4_*.pkl and, for gifs, the MOSAIC models in
data/swarm/skills; writes docs/images/modular_*.png and swarm_{climb,mosaic}.gif.
"""

from __future__ import annotations

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
MOSAIC = "#D55E00"  # Reserved for MOSAIC / the proposed per-skill model.
INK, SLATE, GRID = "#202936", "#718096", "#E5E9EF"
COLORS = {"modular": MOSAIC, "frame": "#4477AA", "id": "#8877AA", "world": SLATE}
SKILLS = ROOT / "data/swarm/skills"
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "mathtext.fontset": "dejavusans",
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.labelsize": 9,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.7,
        "axes.edgecolor": SLATE,
        "text.color": INK,
        "axes.labelcolor": INK,
        "xtick.color": INK,
        "ytick.color": INK,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 8,
        "legend.frameon": False,
        "savefig.facecolor": "white",
    }
)


def panel(ax, letter: str, title: str, *, grid: bool = True) -> None:
    """Common panel heading and unobtrusive reference grid."""
    ax.set_title(f"({letter})  {title}", pad=10)
    ax.set_axisbelow(True)
    if grid:
        ax.grid(axis="y", color=GRID, linewidth=0.6)
    ax.tick_params(length=3, width=0.6)


def model_style(variant: str) -> dict:
    """Redundant colour, dash and marker encoding, stable across experiments."""
    kind = variant.split("_")[0]
    markers = {"modular": "D", "frame": "o", "id": "^", "world": "s"}
    ls = "-"
    if variant.endswith("128x2"):
        ls = "--"
    elif variant.endswith("512x6") or variant.endswith("head"):
        ls = ":"
    return {
        "color": COLORS[kind],
        "linewidth": 2.2 if kind == "modular" else 1.5,
        "linestyle": ls,
        "marker": markers[kind],
        "markersize": 4,
        "markeredgecolor": "white",
        "markeredgewidth": 0.5,
    }


def finish(fig, filename: str, note: str = "", *, legend_ax=None, ncol: int = 2) -> None:
    """Save at publication resolution; reserve whitespace, never cover data."""
    if legend_ax is not None:
        handles, labels = legend_ax.get_legend_handles_labels()
        fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.035), ncol=ncol)
    if note:
        fig.text(0.5, 0.008, note, ha="center", va="bottom", fontsize=8, color=SLATE)
    fig.savefig(IMG / filename, dpi=250, bbox_inches="tight", pad_inches=0.10)
    plt.close(fig)


def label(v):
    """Legend name of a world-model variant; the per-skill one is the MOSAIC-style model."""
    s = v.replace("_", " ")
    return f"{s} (proposed)" if v.startswith("modular") else s


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
    cam.lookat[:] = (STEP_X - 0.7, 0.0, 0.08)
    cam.distance, cam.azimuth, cam.elevation = 3.8, -60.0, -32.0
    frames, times, t, done = [], [], 0, False
    while env.agents and not done:
        if t % 10 == 0:
            ren.update_scene(env.data, cam)
            frames.append(ren.render().copy())
            times.append(float(env.data.time))
        obs, _, term, trunc, _ = env.step(pol.act(obs))
        done = any(term.values()) or any(trunc.values())
        t += 1
    ren.update_scene(env.data, cam)
    frames.append(ren.render().copy())
    times.append(float(env.data.time))
    pick = np.linspace(0, len(frames) - 1, 5).round().astype(int)
    fig, axes = plt.subplots(1, 5, figsize=(7.2, 1.8))
    for letter, ax, i in zip("abcde", axes, pick, strict=True):
        ax.imshow(frames[i])
        panel(ax, letter, f"{times[i]:.1f} s", grid=False)
        ax.axis("off")
    fig.subplots_adjust(left=0.01, right=0.99, top=0.92, bottom=0.17, wspace=0.05)
    finish(fig, "modular_scene.png", "Scripted expert · four spheres · seed 3 · simulation")
    ren.close()
    env.close()


def _gif(env, act, path, cam, every=4):
    import imageio.v2 as imageio
    import mujoco

    ren = mujoco.Renderer(env.model, 300, 400)
    frames, t, info = [], 0, {}
    while env.agents:
        if t % every == 0:
            ren.update_scene(env.data, cam)
            frames.append(ren.render().copy())
        info = act()
        t += 1
    ren.update_scene(env.data, cam)
    frames += [ren.render().copy()] * 10  # hold the last frame
    imageio.mimwrite(path, frames, duration=1000 * every * 0.05, loop=0)
    return t, info


def gifs(seed=600003):
    """Scripted climb of the single step, and MOSAIC (learned WAM executors) on a random course."""
    import mujoco

    from modular_sphere.swarm import SphereSwarmStepEnv, SwarmClimbPolicy, random_course
    from modular_sphere.swarm_plan import EffectModel, SkillScheduler, wam_executor
    from modular_sphere.swarm_skills import SKILLS, SkillValue
    from modular_sphere.swarm_wam import SkillWAM

    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.1, 0.0, 0.0)
    cam.distance, cam.azimuth, cam.elevation = 4.4, -65.0, -35.0
    env = SphereSwarmStepEnv(4)
    obs, _ = env.reset(seed=3)
    pol = SwarmClimbPolicy(4, env.radius)
    state = {"obs": obs}

    def scripted():
        state["obs"], _, _, _, infos = env.step(pol.act(state["obs"]))
        return infos[env.possible_agents[0]]

    print("climb", _gif(env, scripted, IMG / "swarm_climb.gif", cam))
    skills = ROOT / "data/swarm/skills"
    effect = EffectModel({s: SkillValue.load(f"{skills}/value_{s}.pt") for s in SKILLS})
    wams = {s: SkillWAM.load(f"{skills}/chain_wam_mixe_r0_{s}.pt") for s in SKILLS}
    course = random_course(np.random.default_rng(seed))
    env = SphereSwarmStepEnv(n_spheres=4, course=course)
    state["obs"], _ = env.reset(seed=seed)
    sched = SkillScheduler(4, effect, wam_executor(wams))
    sched.reset(env.spheres()[:, :3], env.goal, course)

    def mosaic():
        a = sched.act([state["obs"][x] for x in env.possible_agents], env.spheres(), env.links)
        state["obs"], _, _, _, infos = env.step(dict(zip(env.possible_agents, a, strict=True)))
        return infos[env.possible_agents[0]]

    print("mosaic", course, _gif(env, mosaic, IMG / "swarm_mosaic.gif", cam))


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
    fig, axes = plt.subplots(1, len(sets), figsize=(7.2, 3.5), sharey=False)
    h = np.arange(1, 31) * 0.05
    for letter, ax, s in zip("abc", axes, sets, strict=True):
        for v in variants:
            runs = [res[k][s] for k in res if k.startswith(f"wm_{v}_s") and not k.endswith("_x")]
            curves = np.array([rmse(e, (0, 2)) for e in runs])
            kind = v.split("_")[0]
            ax.plot(h, curves.mean(0), label=label(v), markevery=6, **model_style(v))
            if len(curves) > 1:
                ax.fill_between(h, curves.min(0), curves.max(0), color=COLORS[kind], alpha=0.15)
        panel(ax, letter, {"test": "Test", "courses": "Unseen courses", "mass1.5": "Mass x1.5"}[s])
        ax.set_xlabel("Prediction horizon (s)")
        ax.set_ylim(bottom=0)
    axes[0].set_ylabel("Position RMSE (cm)")
    fig.subplots_adjust(left=0.085, right=0.99, top=0.89, bottom=0.39, wspace=0.32)
    finish(
        fig,
        "modular_e1.png",
        "Mean + seed range; 3 training seeds (world 512x6: 1).",
        legend_ax=axes[0],
    )


def e2():
    rows = [r.split("|")[1:-1] for r in (OUT / "e2_report.md").read_text().splitlines()
            if r.startswith("| mass") or r.startswith("| noise")]  # fmt: skip
    labels = {"oracle_s05m3": "MuJoCo oracle", "wm_wm_modular_64x2_s0_s05m3": "modular 64x2",
              "wm_wm_frame_256x4_s0_s05m3": "frame 256x4",
              "wm_wm_world_256x4_s0_s05m3": "world 256x4"}  # fmt: skip
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.6), sharey=True)
    for letter, ax, cond, title in zip(
        "ab", axes, ("noise0.005", "mass1.5"), ("Noise 5 mm", "Mass x1.5"), strict=True
    ):
        sel = {c[1].strip(): c for c in rows if c[0].strip() == cond and c[1].strip() in labels}
        names = ["MOSAIC alone"] + ["+ " + labels[m] for m in labels]
        base = int(next(iter(sel.values()))[3].split("/")[0])
        vals = [base] + [int(sel[m][2].split("/")[0]) for m in labels]
        ps = [None] + [float(sel[m][6]) for m in labels]
        cols = [MOSAIC, "#444444"] + [COLORS[labels[m].split()[0]] for m in list(labels)[1:]]
        bars = ax.barh(names, vals, color=cols, height=0.64, edgecolor="white", linewidth=0.7)
        bars[0].set_hatch("///")
        ax.axvline(base, color=MOSAIC, ls="--", lw=1)
        for b, p in zip(bars, ps, strict=True):
            pv = "" if p is None else ("\np<0.001" if p < 1e-3 else f"\np={p:.2g}")
            txt = f"{b.get_width():.0f}/50{pv}"
            ax.text(51.5, b.get_y() + b.get_height() / 2, txt, va="center", fontsize=8)
        ax.set_xlim(0, 63)
        ax.set_xticks([0, 25, 50])
        panel(ax, letter, title, grid=False)
        ax.set_axisbelow(True)
        ax.grid(axis="x", color=GRID, linewidth=0.6)
        ax.set_xlabel("Successful courses (of 50)")
    axes[0].invert_yaxis()
    fig.subplots_adjust(left=0.21, right=0.99, top=0.88, bottom=0.20, wspace=0.25)
    finish(fig, "modular_e2.png", "Point estimates; exact McNemar vs MOSAIC alone (paired seeds).")


def e3():
    res = pickle.load(open(OUT / "e3.pkl", "rb"))
    models = ["modular_64x2", "frame_128x2", "world_256x4"]
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.8))
    x = np.arange(3)
    for j, (ph, lab) in enumerate((("p1", "phase 1 (no pull)"), ("p2_k10", "+ pull, 10 episodes"),
                                    ("p2_kall", "+ pull, all episodes"))):  # fmt: skip
        for ax, st in zip(axes, ("test_nopull", "test_pull"), strict=True):
            v = [[rmse(res[m, s][ph][st][:, -1]) for s in range(3)] for m in models]
            ax.bar(
                x + (j - 1) * 0.25,
                np.mean(v, 1),
                0.25,
                yerr=np.std(v, 1),
                label=lab,
                color=[COLORS[m.split("_")[0]] for m in models],
                alpha=[0.35, 0.65, 1][j],
                hatch=["", "///", "..."][j],
                edgecolor=INK,
                linewidth=0.5,
                error_kw={"elinewidth": 0.9, "capsize": 2, "capthick": 0.9},
            )
    for letter, ax, t in zip("ab", axes, ("Known skills", "New skill: pull"), strict=True):
        ax.set_xticks(x, [label(m).replace(" (", "\n(").replace("_", " ") for m in models])
        panel(ax, letter, t)
    axes[0].set_ylabel("30-step position RMSE (cm)")
    # Phase legend uses neutral patches: colour encodes model, hatch/opacity encodes phase.
    from matplotlib.patches import Patch

    handles = [
        Patch(facecolor=SLATE, alpha=a, hatch=h, edgecolor=INK, label=lab)
        for a, h, lab in zip(
            (0.35, 0.65, 1),
            ("", "///", "..."),
            ("Before pull", "+ pull: 10 episodes", "+ pull: all episodes"),
            strict=True,
        )
    ]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.07), ncol=3)
    fig.subplots_adjust(left=0.085, right=0.99, top=0.88, bottom=0.33, wspace=0.28)
    finish(
        fig, "modular_e3.png", "Mean ± population SD · 3 training seeds · proposed: pull net only."
    )


def e4():
    res = {}
    for f in sorted(OUT.glob("e4_*_s*.pkl")):
        res.update(pickle.load(open(f, "rb")))
    arms = ("modular_64x2", "frame_128x2", "frame_256x4", "frame_256x4_head", "world_256x4")
    lr = 3e-4
    conds = sorted({k[0] for k in res})
    fig, axes = plt.subplots(2, len(conds), figsize=(7.2, 5.5), squeeze=False)
    for j, cond in enumerate(conds):
        for arm in arms:
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
                axes[i, j].errorbar(
                    np.array(ks) + 0.5,
                    v.mean(0),
                    v.std(0),
                    label=label(arm),
                    **model_style(arm),
                    elinewidth=0.9,
                    capthick=0.9,
                    capsize=2,
                )
        mass = cond.replace("mass", "Mass x")
        panel(axes[0, j], "abcd"[j], f"{mass} · adaptation")
        panel(axes[1, j], "abcd"[len(conds) + j], f"{mass} · retention")
        for i in range(2):
            axes[i, j].set_xscale("log")
            axes[i, j].set_xticks(
                [0.5, 1.5, 2.5, 5.5, 10.5, 20.5], ["0", "1", "2", "5", "10", "20"]
            )
            axes[i, j].set_ylabel("30-step RMSE (cm)")
        axes[1, j].set_xlabel("Adaptation episodes, k")
    n_seeds = len({k[2] for k in res})
    fig.subplots_adjust(left=0.09, right=0.99, top=0.93, bottom=0.26, wspace=0.30, hspace=0.40)
    finish(
        fig,
        "modular_e4.png",
        f"Mean ± population SD · {n_seeds} seeds · lr = {lr:g} · matched updates.",
        legend_ax=axes[0, 0],
    )


def _box(ax, x, y, w, h, text="", fc="white", ec="#333333", fs=7.5, rot=0, lw=0.8, ls="-", r=0.6):
    from matplotlib.patches import FancyBboxPatch

    ax.add_patch(
        FancyBboxPatch(
            (x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}", fc=fc, ec=ec, lw=lw, ls=ls
        )
    )
    if text:
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, rotation=rot)


def _arrow(ax, a, b, style="-|>", color="#333333", lw=0.9, rad=0.0, ls="-"):
    from matplotlib.patches import FancyArrowPatch

    ax.add_patch(
        FancyArrowPatch(
            a,
            b,
            arrowstyle=style,
            mutation_scale=8,
            color=color,
            lw=lw,
            ls=ls,
            connectionstyle=f"arc3,rad={rad}",
            shrinkA=0,
            shrinkB=0,
        )
    )


def _add(ax, x, y, r=1.0):
    from matplotlib.patches import Circle

    ax.add_patch(Circle((x, y), r, fc="white", ec="#333333", lw=0.8))
    ax.text(x, y, "+", ha="center", va="center", fontsize=8)


SKILL_C = {"pull": "#CC79A7", "climb": "#009E73", "roll": "#0072B2", "idle": "#999999"}
EXAMPLE = ["pull", "pull", "climb", "roll"]  # one pull call {s1, s2}, one climb, one roll
EX_LINKS = {(0, 1), (1, 2), (2, 3)}


def _mask(ax, letter, title, names, allow, link, rowc, groups=None):
    """Attention matrix of one layer: queries in rows, keys / values in columns."""
    from matplotlib.patches import Rectangle

    n = len(names)
    for i in range(n):
        for j in range(n):
            x, y = j, n - 1 - i
            if allow[i][j]:
                ax.add_patch(
                    Rectangle(
                        (x + 0.08, y + 0.08),
                        0.84,
                        0.84,
                        fc=rowc[i],
                        ec=INK if link[i][j] else rowc[i],
                        lw=0.5,
                        hatch="////" if link[i][j] else None,
                        alpha=0.85,
                    )
                )
            else:
                ax.add_patch(Rectangle((x + 0.08, y + 0.08), 0.84, 0.84, fc="white", ec=GRID))
        ax.text(i + 0.5, n + 0.15, names[i], ha="center", va="bottom", fontsize=8)
        ax.text(-0.15, n - 0.5 - i, names[i], ha="right", va="center", fontsize=8)
    ax.text(n / 2, n + 0.95, "K / V", ha="center", fontsize=8, fontweight="bold")
    ax.text(-1.55, n / 2, "Q", va="center", fontsize=8, fontweight="bold")
    if groups:
        for k, (g0, g1, text, c) in enumerate(groups):
            ax.plot([g0 + 0.1, g1 - 0.1], [-0.25, -0.25], color=c, lw=1.6)
            ax.text(
                (g0 + g1) / 2,
                -0.45 - 0.5 * (k % 2 and g1 - g0 == 1),
                text,
                ha="center",
                va="top",
                fontsize=7,
                color=c,
            )
            if g0 > 0:
                ax.plot([g0, g0], [-0.1, n + 0.1], color=SLATE, lw=0.6, ls=":")
                ax.plot([-0.1, n + 0.1], [n - g0, n - g0], color=SLATE, lw=0.6, ls=":")
    ax.set(xlim=(-1.8, 6.3), ylim=(n - 7.3, n + 1.4), aspect="equal")
    ax.set_anchor("N")
    ax.axis("off")
    ax.set_title(f"({letter})  {title}", pad=2, fontsize=9)


def _trap(ax, x, y, w, h, text, fc, ls="-"):
    from matplotlib.patches import Polygon

    d = 0.12 * w
    ax.add_patch(
        Polygon([(x, y), (x + w, y), (x + w - d, y + h), (x + d, y + h)], fc=fc, ec=fc, ls=ls)
    )
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=7.5, color="white")


def _tokens(ax, xs, y, colors, hatch=None, s=4.2):
    from matplotlib.patches import Rectangle

    for x, c in zip(xs, colors, strict=True):
        ax.add_patch(
            Rectangle(
                (x, y),
                s,
                s,
                fc=c,
                ec="white" if not hatch else c,
                lw=0.6,
                hatch=hatch,
                alpha=0.9 if not hatch else 0.35,
            )
        )


def arch():
    """SwarmWM in the style of transformer world-model papers: (a) one prediction step,
    (b, c) the attention mask of the shared and the per-skill (modular) models."""
    from matplotlib.patches import Rectangle

    fig = plt.figure(figsize=(7.2, 4.9))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.45, 1], hspace=0.12, wspace=0.02)
    ax = fig.add_subplot(gs[:, 0])
    ax.set(xlim=(0, 100), ylim=(0, 100))
    ax.axis("off")
    ax.set_title("(a)  SwarmWM · one prediction step", pad=4, fontsize=9.5)
    skc = [SKILL_C[s] for s in EXAMPLE]
    dark, pale = "#34495E", "#EDF3F9"
    arr = {"color": SLATE, "lw": 1.0}

    # Inputs.
    fy = 9.5
    ax.plot([2, 11, 11, 20], [fy, fy, fy + 4, fy + 4], color=INK, lw=0.9)
    pts = [(5, fy + 1.1), (8.5, fy + 1.1), (13.5, fy + 5.1), (17.5, fy + 5.1)]
    for i, j in EX_LINKS:
        ax.plot(*zip(pts[i], pts[j], strict=True), color=SLATE, lw=0.8)
    for p, c in zip(pts, skc, strict=True):
        ax.plot(*p, "o", ms=5.5, color=c, mec="white", mew=0.5)
    for k in range(4):
        x = 25 + 3.2 * k
        ax.plot(x, fy + 2.5, "o", ms=3.5, color="#B8C2CF")
        ax.annotate(
            "",
            (x + 1.6 * (-1) ** k, fy + 5.5),
            (x, fy + 2.5),
            arrowprops={"arrowstyle": "-|>", "color": INK, "lw": 0.7, "mutation_scale": 6},
        )
    _box(ax, 39, fy - 0.8, 15, 8.4, fc="white", ec=SLATE, r=0.6)
    for k, (t, c) in enumerate(
        [
            ("pull {s1, s2}", SKILL_C["pull"]),
            ("climb {s3}", SKILL_C["climb"]),
            ("roll {s4}", SKILL_C["roll"]),
        ]
    ):
        ax.text(46.5, fy + 5.8 - 2.6 * k, t, ha="center", va="center", fontsize=6.5, color=c)
    _box(ax, 57, fy - 0.8, 13, 8.4, fc="white", ec=SLATE, ls="--", r=0.6)
    ax.plot([58.5, 63, 63, 68.5], [fy + 0.5, fy + 0.5, fy + 5, fy + 5], color=INK, lw=0.8)
    ax.plot(63, fy + 2.7, "D", ms=3.5, color=INK)
    for i in range(4):
        for j in range(4):
            link = (min(i, j), max(i, j)) in EX_LINKS and i != j
            ax.add_patch(
                Rectangle(
                    (80 + 2 * j, fy + 5.5 - 2 * i),
                    1.8,
                    1.8,
                    ec=INK if link else GRID,
                    fc="#DDE3EA" if link else "white",
                    hatch="////" if link else None,
                    lw=0.4,
                )
            )
    for x, t in [
        (11, "State $x_t$\nN $\\times$ 9"),
        (30, "Action $a_t$\nN $\\times$ 4"),
        (46.5, "Skill calls\nskill · role"),
        (63.5, "Faces*\nworld, id"),
        (84, "Links $A_t$\nN $\\times$ N"),
    ]:
        ax.text(x, fy - 1.8, t, ha="center", va="top", fontsize=7.5)

    # Encoders and tokens.
    for x in (11, 30, 46.5):
        _arrow(ax, (x, fy + 8.2), (x, 22), **arr)
    _trap(ax, 3, 22, 51, 8, "Sphere encoder\nlinear + skill·role emb.*", dark)
    ax.text(12.5, fy + 9.8, "frame*,\nnorm.", fontsize=6.5, color=SLATE, va="center")
    _arrow(ax, (63.5, fy + 8.2), (63.5, 22), **arr)
    _trap(ax, 55.5, 22, 16, 8, "Face\nenc.*", "#8D99A8")
    _arrow(ax, (28.5, 30), (28.5, 33), **arr)
    _arrow(ax, (63.5, 30), (63.5, 33), **arr)
    _tokens(ax, [18.5, 23.5, 28.5, 33.5], 33, skc)
    _tokens(ax, [59, 64], 33, ["#8D99A8"] * 2, hatch="////")
    ax.text(16.5, 35.1, "$h_{1..4}$", ha="right", va="center", fontsize=8)
    ax.text(69.5, 35.1, "$f_{1,2}$", ha="left", va="center", fontsize=8)

    # Transformer stack: one SetNet per skill (modular), L pre-LN blocks.
    _box(ax, 4, 41, 74, 43, fc="none", ec=SLATE, ls="--", r=1.5)
    ax.text(77, 85, "L $\\times$", ha="right", fontsize=9, style="italic", color=SLATE)
    for k in (3, 2, 1):
        _box(ax, 10 + 1.4 * k, 44 + 1.2 * k, 56, 35, fc="white", ec=MOSAIC, lw=0.7, r=1.2)
    _box(ax, 10, 44, 56, 35, fc="white", ec=MOSAIC, lw=1.4, r=1.2)
    layers = [
        (51, "LayerNorm", pale, INK),
        (58, "Masked attention + link bias", dark, "white"),
        (65, "LayerNorm", pale, INK),
        (72, "FFN (GELU MLP)", pale, INK),
    ]
    for y, t, fc, tc in layers:
        _box(ax, 14, y, 48, 5, fc=fc, ec=SLATE, lw=0.6, r=0.8)
        ax.text(38, y + 2.5, t, ha="center", va="center", fontsize=7.5, color=tc)
    ax.text(
        38,
        46.3,
        "SetNet $\\times$ 4 (one per skill)",
        ha="center",
        va="center",
        fontsize=8,
        fontweight="bold",
        color=MOSAIC,
    )
    _arrow(ax, (38, 37.2), (38, 44), **arr)
    _arrow(ax, (84, fy + 8.2), (84, 60.5), style="-", **arr)
    _arrow(ax, (84, 60.5), (62, 60.5), **arr)
    ax.text(
        85.5,
        40,
        "bias $w_{\\ell h}A_{ij}$\n+ call-group\nmask*\n(b, c)",
        fontsize=7.5,
        va="center",
        color=INK,
    )

    # Head and unrolled output.
    _arrow(ax, (38, 79), (38, 88), **arr)
    _tokens(ax, [28.5, 33.5, 38.5, 43.5], 88, skc)
    ax.text(50, 90.1, "linear head: $\\hat\\Delta_i \\in \\mathbb{R}^9$", va="center", fontsize=8)
    ax.text(
        28,
        90.1,
        "$\\hat x_{t+1}=x_t+\\sigma\\hat\\Delta+\\mu$",
        ha="right",
        va="center",
        fontsize=8,
    )
    _arrow(ax, (1.5, 90.1), (1.5, fy + 2), style="-", color=MOSAIC, lw=1.0, ls="--")
    _arrow(ax, (1.5, fy + 2), (3.5, fy + 2), color=MOSAIC, lw=1.0, ls="--")
    ax.text(
        0.2,
        62,
        "open-loop unroll: $\\hat x_{t+1}$ replaces $x_t$",
        rotation=90,
        va="center",
        ha="right",
        fontsize=7.5,
        color=MOSAIC,
    )

    n = len(EXAMPLE)
    names = [f"s{i + 1}" for i in range(n)]
    lk = [
        [(min(i, j), max(i, j)) in EX_LINKS and i != j for j in range(n + 2)] for i in range(n + 2)
    ]
    _mask(
        fig.add_subplot(gs[0, 1]),
        "b",
        "Shared model (world, id, frame)",
        [*names, "f1", "f2"],
        [[True] * (n + 2)] * (n + 2),
        lk,
        ["#9FB0C4"] * n + ["#C9D1DB"] * 2,
        [(0, n, "spheres", SLATE), (n, n + 2, "faces*", SLATE)],
    )
    group = [0, 0, 1, 2]
    axc = fig.add_subplot(gs[1, 1])
    _mask(
        axc,
        "c",
        "Per-skill model (modular): same call only",
        names,
        [[group[i] == group[j] for j in range(n)] for i in range(n)],
        lk,
        skc,
        [
            (0, 2, "pull", SKILL_C["pull"]),
            (2, 3, "climb", SKILL_C["climb"]),
            (3, 4, "roll", SKILL_C["roll"]),
        ],
    )
    from matplotlib.patches import Patch

    axc.legend(
        handles=[
            Patch(fc="#9FB0C4", ec="#9FB0C4", label="allowed"),
            Patch(fc="#9FB0C4", ec=INK, hatch="////", label="+ link bias"),
            Patch(fc="white", ec=GRID, label="masked"),
            Patch(fc=MOSAIC, label="skill net (row)"),
        ],
        loc="lower center",
        bbox_to_anchor=(0.62, -0.02),
        ncol=2,
        fontsize=7.5,
        handlelength=1.2,
    )
    finish(
        fig,
        "modular_arch.png",
        "* variant-specific (paper §4.2). Actions, links and calls are supplied; the "
        "model predicts states, not topology.",
    )


def mosaic():
    """MOSAIC against the expert and the learned baselines (Appendix A.10-A.11 runs)."""

    def stats(r, k):
        ok, st = r[k]["success"], r[k]["steps"]
        return int(ok.sum()), float(st[ok].mean()) if ok.any() else np.nan

    s4 = pickle.load(open(SKILLS / "s4_50.pkl", "rb"))
    conds = {"base": "nominal", "noise0.005": "noise\n5 mm", "fric0.5": "friction\nx0.5",
             "fric1.5": "friction\nx1.5", "mass0.7": "mass\nx0.7", "mass1.5": "mass\nx1.5",
             "rise0.10": "rise\n0.10 m", "n5": "N = 5", "n6": "N = 6"}  # fmt: skip
    adapt = {c: pickle.load(open(SKILLS / f"adapt_{c}_shaped.pkl", "rb")) for c in conds}
    arms = {
        "expert": ("Scripted expert", INK),
        "script": ("Scheduler + scripted skills", SLATE),
        "mosaic": ("MOSAIC (learned skills)", MOSAIC),
    }
    fig = plt.figure(figsize=(7.2, 7.2))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], hspace=0.55, wspace=0.60)

    # (a) Success and speed retain the original paired evaluation statistics.
    ax = fig.add_subplot(gs[0, 0])
    pts = {
        "expert": ("Expert", INK, "s"),
        "dagger": ("DAgger", COLORS["frame"], "o"),
        "bc512": ("BC skills\n(no chain)", COLORS["id"], "^"),
        "wam": ("WAM skills\n(no chain)", SLATE, "v"),
        "script": ("Scripted skills", SLATE, "s"),
        "chain_wam_mixe": ("MOSAIC", MOSAIC, "D"),
    }
    for k, (lab, c, marker) in pts.items():
        n, st = stats(s4, k)
        big = k == "chain_wam_mixe"
        ax.scatter(
            st,
            n,
            s=55 if big else (75 if k == "script" else 35),
            facecolor="none" if k == "script" else c,
            zorder=3,
            edgecolor=SLATE if k == "script" else "white",
            linewidth=1 if k == "script" else 0.6,
            marker=marker,
        )
        offset = {
            "expert": (-6, 8),
            "dagger": (-10, -16),
            "script": (6, -20),
            "bc512": (6, -8),
            "wam": (6, -6),
            "chain_wam_mixe": (0, 9),
        }[k]
        ax.annotate(
            lab,
            (st, n),
            xytext=offset,
            textcoords="offset points",
            ha="right" if k == "expert" else "left",
            fontsize=8,
            color=c,
            fontweight="bold" if big else "normal",
            arrowprops={"arrowstyle": "-", "color": SLATE, "lw": 0.7} if k == "script" else None,
        )
    ax.set_xlim(110, 300)
    ax.set_ylim(22, 56)
    ax.set_xlabel("Mean steps on success ↓")
    ax.set_ylabel("Successful episodes (of 50)")
    panel(ax, "a", "Single step · N = 4")

    # (c) Each ratio averages only over courses solved by both policies.
    ax_speed = fig.add_subplot(gs[0, 1])
    x = np.arange(len(conds))
    for j, k in enumerate(("script", "mosaic")):
        ratios = []
        for cd in conds:
            r = adapt[cd]
            both = r[k]["success"] & r["expert"]["success"]
            ratios.append((r[k]["steps"][both] / r["expert"]["steps"][both]).mean())
        ax_speed.barh(
            x + (j - 0.5) * 0.38,
            ratios,
            0.38,
            color=arms[k][1],
            hatch="///" if k == "mosaic" else "",
            edgecolor="white",
            linewidth=0.5,
        )
    ax_speed.axvline(1, color=INK, lw=0.9, ls="--")
    ax_speed.set_yticks(x, [c.replace("\n", " ") for c in conds.values()])
    ax_speed.invert_yaxis()
    ax_speed.set_xlim(0, 1.15)
    ax_speed.set_xlabel("Steps / expert steps ↓")
    panel(ax_speed, "b", "Paired-course speed", grid=False)
    ax_speed.grid(axis="x", color=GRID, linewidth=0.6)

    # (b) Zero-shot performance, one shared legend for both course panels.
    ax_courses = fig.add_subplot(gs[1, :])
    for j, (k, (lab, c)) in enumerate(arms.items()):
        v = [stats(adapt[cd], k)[0] for cd in conds]
        ax_courses.bar(
            x + (j - 1) * 0.25,
            v,
            0.25,
            color=c,
            label=lab,
            hatch="///" if k == "mosaic" else "",
            edgecolor="white",
            linewidth=0.5,
        )
    ax_courses.set_xticks(x, list(conds.values()))
    ax_courses.set_ylim(0, 55)
    ax_courses.set_ylabel("Successful courses (of 50)")
    panel(ax_courses, "c", "Unseen 1-3 step courses · zero-shot perturbations")
    fig.subplots_adjust(left=0.09, right=0.99, top=0.94, bottom=0.18)
    finish(
        fig,
        "mosaic_benchmark.png",
        "50 paired evaluation seeds per condition · point estimates, no uncertainty intervals.",
        legend_ax=ax_courses,
        ncol=3,
    )


if __name__ == "__main__":
    for name in sys.argv[1:] or ["scene", "arch", "mosaic", "e1", "e2", "e3", "e4", "gifs"]:
        globals()[name]()
        print(f"done {name}")
