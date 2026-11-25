"""Figures of the paper's swarm world-model experiments (E1-E4) and a swarm climb filmstrip.

Usage: uv run --with matplotlib python docs/paper/experiments/modular_figures.py [NAME ...]
(NAME among scene, arch, mosaic, e1, e2, e3, e4, gifs; all by default). Reads
data/swarm/modular/{e1,e3}.pkl, e2_report.md, e4_*.pkl and, for gifs, the MOSAIC models in
data/swarm/skills; writes docs/images/modular_*.png and swarm_{climb,mosaic}.gif.
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
MOSAIC = "#c0392b"  # the MOSAIC system in every figure
SKILLS = ROOT / "data/swarm/skills"
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})


def label(v):
    """Legend name of a world-model variant; the per-skill one is the MOSAIC-style model."""
    s = v.replace("_", " ")
    return f"{s} (per skill, MOSAIC-style)" if v.startswith("modular") else s


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
    fig, axes = plt.subplots(1, len(sets), figsize=(10, 3.2), sharey=False)
    h = np.arange(1, 31) * 0.05
    for ax, s in zip(axes, sets, strict=True):
        for v in variants:
            runs = [res[k][s] for k in res if k.startswith(f"wm_{v}_s") and not k.endswith("_x")]
            curves = np.array([rmse(e, (0, 2)) for e in runs])
            kind = v.split("_")[0]
            ls = ":" if v == "world_512x6" else ("--" if v.endswith("128x2") else "-")
            lw = 2.4 if kind == "modular" else 1.4
            ax.plot(h, curves.mean(0), ls, color=COLORS[kind], label=label(v), lw=lw)
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
        names = ["MOSAIC alone (reference)"] + [labels[m] for m in labels]
        base = int(next(iter(sel.values()))[3].split("/")[0])
        vals = [base] + [int(sel[m][2].split("/")[0]) for m in labels]
        ps = [None] + [float(sel[m][6]) for m in labels]
        cols = [MOSAIC, "#444444"] + [COLORS[labels[m].split()[0]] for m in list(labels)[1:]]
        bars = ax.barh(names, vals, color=cols)
        ax.axvline(base, color=MOSAIC, ls="--", lw=1)
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
        "E2 (negative): MOSAIC alone vs MOSAIC + world-model verifier (exact McNemar vs MOSAIC)"
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
        ax.set_xticks(x, [label(m).replace(" (", "\n(") for m in models], fontsize=7.5)
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
                                    color=COLORS[kind], label=label(arm),
                                    lw=2.4 if kind == "modular" else 1.4,
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


def arch():
    """SwarmWM: state flow, exact residual block, and variant inputs/routing."""
    fig, ax = plt.subplots(figsize=(16, 9.6))
    ax.set(xlim=(0, 160), ylim=(0, 96))
    ax.axis("off")
    blue, green, ink = "#e4eff8", "#edf3ed", "#243447"

    def text(x, y, label, size=10, **kw):
        ax.text(x, y, label, fontsize=size, color=ink, va="center", **kw)

    def box(x, y, w, h, label, color="white"):
        _box(ax, x, y, w, h, label, fc=color, ec="#82909e", fs=10, r=0.7)

    def arrow(a, b, **kw):
        _arrow(ax, a, b, color=ink, lw=1.15, **kw)

    text(2, 93, "(a) State prediction: one step, then repeat", 14, fontweight="bold")
    text(2, 89, "B: batch   N: spheres   M: course faces   w: token width   L: blocks")
    box(
        2,
        67,
        21,
        17,
        "Raw state $x_t$\n$B \\times N \\times 9$\nposition, velocity,\nangular velocity",
        blue,
    )
    box(
        28,
        67,
        31,
        17,
        "State preprocessing\nworld / id: $x_t$\nframe / modular: "
        "$[p-o_c,v,\\omega]$\nthen $(\\cdot-\\mu_x)/\\sigma_x$",
        blue,
    )
    box(
        64,
        67,
        27,
        17,
        "Sphere tokens\nconcat state + action (4)\n+ rise (1) if framed"
        "\nLinear $d_{in}\\to w$; $B\\times N\\times w$",
        blue,
    )
    box(
        96,
        67,
        26,
        17,
        "SetNet\n+ category embedding*\nappend face tokens*\n$L$ pre-LN blocks → head",
        green,
    )
    box(
        127,
        67,
        31,
        17,
        "Next raw state\n$d$: head output (9 / "
        "sphere)\n$\\hat{x}_{t+1}=x_t+\\sigma_\\Delta d+\\mu_\\Delta$\n$B \\times N "
        "\\times 9$",
        blue,
    )
    for x1, x2 in [(23, 28), (59, 64), (91, 96), (122, 127)]:
        arrow((x1, 75.5), (x2, 75.5))
    text(28, 63.5, "Translation only; velocities and axes stay world-aligned. Idle: $o_c=p$.")
    text(104, 63.5, "* Inputs / routing: panel (c).")
    arrow((143, 67), (143, 58), style="-", ls="--")
    arrow((143, 58), (12, 58), style="-", ls="--")
    arrow((12, 58), (12, 67), ls="--")
    text(
        80,
        55,
        "State-autoregressive unroll: predicted state fed back; recorded actions, links "
        "and call context supplied at each step.",
        ha="center",
    )
    text(
        80,
        51.5,
        "Training: 8-step loss, state error scaled by $\\sigma_\\Delta$. Evaluation: "
        "position RMSE at 30 steps. No topology or call prediction.",
        ha="center",
    )

    text(2, 46, "(b) Inside SetNet: attention over objects, not time", 13, fontweight="bold")
    box(2, 14, 77, 28, "", "#f8fafb")
    text(4, 39, "$L \\times$ pre-LN residual block", 11, fontweight="bold")
    text(4, 28, "$h$")
    box(9, 24, 10, 8, "LN")
    box(23, 24, 18, 8, "Multi-head\nattention", green)
    _add(ax, 45, 28, 1.4)
    box(50, 24, 9, 8, "LN")
    box(63, 24, 10, 8, "MLP", green)
    _add(ax, 76, 28, 1.4)
    for x1, x2 in [(6, 9), (19, 23), (41, 43.6), (46.4, 50), (59, 63), (73, 74.6)]:
        arrow((x1, 28), (x2, 28))
    for x1, x2, y in [(7, 45, 35), (48, 76, 18)]:
        arrow((x1, 28), (x1, y), style="-")
        arrow((x1, y), (x2, y), style="-")
        arrow((x2, y), (x2, 28 + (1.4 if y > 28 else -1.4)))
    text(4, 11, "MLP: Linear $w$ → $4w$ → GELU → Linear $4w$ → $w$")
    text(4, 7, "After $L$ blocks: Linear $w$ → 9, sphere tokens only; no final LN.")
    text(4, 3, "Attention logits: $QK^T/\\sqrt{d_h}+\\alpha_{l,h}\\ell_{ij}+A_{ij}$", 11)
    text(24, 20, "bias + mask", 9, ha="center")
    arrow((29, 21.5), (29, 24))

    text(84, 46, "(c) Four kinds: inputs, attention scope, routing", 13, fontweight="bold")
    columns = [84, 97, 111, 126, 142]
    for x, label in zip(
        columns, ["Kind", "$d_{in}$ / cat.", "Faces", "Sphere scope", "Networks"], strict=True
    ):
        text(x + 0.5, 40, label, fontweight="bold")
    rows = [
        ("world", "13 / no", "yes", "all", "1 shared"),
        ("id", "13 / yes", "yes", "all", "1 shared"),
        ("frame", "14 / yes", "no", "all", "1 shared"),
        ("modular", "14 / yes", "no", "same call", "4 skill nets"),
    ]
    for j, row in enumerate(rows):
        y = 34 - 6 * j
        ax.plot([84, 158], [y + 3, y + 3], color="#d4dbe1", lw=0.7)
        for k, (x, label) in enumerate(zip(columns, row, strict=True)):
            text(x + 0.5, y, label, **({"fontweight": "bold"} if k == 0 else {}))
        ax.plot([83, 83], [y - 2, y + 2], color=COLORS[row[0]], lw=3)
    text(84, 10, "Category: 8 skill-role embeddings. Routing: cat // 2 selects", 9.5)
    text(84, 6.5, "idle / roll / climb / pull output per sphere; no learned gate.", 9.5)
    text(84, 3, "Modular: $A_{ij}=0$ for same call, $-10^9$ otherwise; idle groups unique.", 9.5)
    fig.text(
        0.5,
        0.005,
        "Links: learned scalar per layer/head, initially zero. Faces: Linear 3→w, zero "
        "link bias to/from faces, padded face keys masked. Normalization: fitted state / "
        "delta statistics.",
        ha="center",
        va="bottom",
        fontsize=9,
        color="#546477",
    )
    fig.subplots_adjust(left=0.01, right=0.99, bottom=0.045, top=0.99)
    fig.savefig(IMG / "modular_arch.png", dpi=200, bbox_inches="tight", pad_inches=0.12)
    plt.close(fig)


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
    arms = {"expert": ("scripted expert (sequential)", "#222222"),
            "script": ("scheduler + scripted skills", "#9e9e9e"),
            "mosaic": ("MOSAIC (learned skills)", MOSAIC)}  # fmt: skip
    fig = plt.figure(figsize=(13, 3.9))
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 1.7, 1.25], wspace=0.32)

    # (a) test seeds: success against speed
    ax = fig.add_subplot(gs[0])
    pts = {"expert": ("expert", "#222222"), "dagger": ("DAgger\n(whole task)", COLORS["world"]),
           "bc512": ("BC skills,\nno chaining", "#c9a0a0"),
           "wam": ("WAM skills,\nno chaining", "#e08e8e"),
           "script": ("scripted\nskills", "#9e9e9e"),
           "chain_wam_mixe": ("MOSAIC", MOSAIC)}  # fmt: skip
    for k, (lab, c) in pts.items():
        n, st = stats(s4, k)
        big = k == "chain_wam_mixe"
        ax.scatter(st, n, s=150 if big else 55, color=c, zorder=3, edgecolor="white",
                   marker="*" if big else "o")  # fmt: skip
        dx, dy = {"expert": (-12, 1.5), "dagger": (-22, -5), "script": (8, -4.5),
                  "bc512": (6, -1), "wam": (6, -1), "chain_wam_mixe": (-38, 2)}[k]  # fmt: skip
        ax.annotate(lab, (st, n), (st + dx, n + dy), fontsize=7, color=c,
                    fontweight="bold" if big else "normal")  # fmt: skip
    ax.set_xlim(100, 290)
    ax.set_ylim(22, 55)
    ax.set_xlabel("mean steps on success (lower is faster)")
    ax.set_ylabel("successes / 50")
    ax.set_title("(a) test seeds, one step, N = 4", fontsize=9)

    # (b) success on fresh courses under perturbations
    ax = fig.add_subplot(gs[1])
    x = np.arange(len(conds))
    for j, (k, (lab, c)) in enumerate(arms.items()):
        v = [stats(adapt[cd], k)[0] for cd in conds]
        ax.bar(x + (j - 1) * 0.27, v, 0.27, color=c, label=lab)
    ax.set_xticks(x, list(conds.values()), fontsize=7)
    ax.set_ylim(0, 56)
    ax.set_ylabel("successes / 50")
    ax.legend(fontsize=7.5, frameon=False, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.17))
    ax.set_title("(b) unseen random 1-3 step courses, zero-shot", fontsize=9)

    # (c) speed relative to the expert, courses both solve
    ax = fig.add_subplot(gs[2])
    for j, k in enumerate(("script", "mosaic")):
        ratios = []
        for cd in conds:
            r = adapt[cd]
            both = r[k]["success"] & r["expert"]["success"]
            ratios.append((r[k]["steps"][both] / r["expert"]["steps"][both]).mean())
        ax.barh(x + (j - 0.5) * 0.38, ratios, 0.38, color=arms[k][1], label=arms[k][0])
    ax.axvline(1, color="#222222", lw=1, ls="--")
    ax.text(0.98, len(conds) - 0.2, "expert", fontsize=7, ha="right")
    ax.set_yticks(x, [c.replace("\n", " ") for c in conds.values()], fontsize=7)
    ax.invert_yaxis()
    ax.set_xlim(0, 1.15)
    ax.set_xlabel("steps / expert steps (lower is faster)")
    ax.set_title("(c) speed on courses both solve", fontsize=9)
    title = "MOSAIC against the scripted expert and learned baselines (50 paired seeds per bar)"
    fig.suptitle(title, y=1.04)
    fig.savefig(IMG / "mosaic_benchmark.png", dpi=150, bbox_inches="tight")


if __name__ == "__main__":
    for name in sys.argv[1:] or ["scene", "arch", "mosaic", "e1", "e2", "e3", "e4", "gifs"]:
        globals()[name]()
        print(f"done {name}")
