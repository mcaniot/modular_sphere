"""Adaptability tests of MOSAIC on random courses (seed0..): step-frame ablation and
out-of-distribution conditions (rises, swarm size, mass, friction, observation noise).

Usage: skills_adapt.py CONDITION [modes=expert,script,mosaic,mosaic0] [seeds=50] [seed0=400000]
[wam=chain_wam] [tag=]; mosaic: executors {wam}_r0_*, mosaic0: chain_wam_r0_* (MOSAIC before).
Saves data/swarm/skills/adapt_{CONDITION}.pkl {mode: {success, steps, restarts}} (per seed).
"""

import pickle
import sys
from dataclasses import replace

import numpy as np

from modular_sphere.swarm import (
    STEP_H,
    STEP_X,
    Course,
    SphereSwarmStepEnv,
    SwarmClimbPolicy,
    random_course,
)
from modular_sphere.swarm_plan import EffectModel, SkillScheduler, script_executor, wam_executor
from modular_sphere.swarm_skills import SKILLS, SkillValue
from modular_sphere.swarm_wam import SkillWAM

root = "/home/mcaniot/Documents/modular_sphere/data/swarm/skills"
cond = sys.argv[1]
kw = dict(a.split("=") for a in sys.argv[2:])
modes = kw.get("modes", "expert,script,mosaic").split(",")
seed0 = int(kw.get("seed0", "400000"))
seeds = list(range(seed0, seed0 + int(kw.get("seeds", "50"))))
effect = EffectModel({s: SkillValue.load(f"{root}/value_{s}.pt") for s in SKILLS})
wam = kw.get("wam", "chain_wam")  # executor files {wam}_r0_{skill}.pt
wams = {s: SkillWAM.load(f"{root}/{wam}_r0_{s}.pt") for s in SKILLS}
old = {s: SkillWAM.load(f"{root}/chain_wam_r0_{s}.pt") for s in SKILLS}  # mode mosaic0
out = cond + kw.get("tag", "")

n, mass, fric, noise, rise = 4, 1.0, 1.0, 0.0, None
if cond[0] == "n" and cond[1:].isdigit():
    n = int(cond[1:])
elif cond.startswith("mass"):
    mass = float(cond[4:])
elif cond.startswith("fric"):
    fric = float(cond[4:])
elif cond.startswith("noise"):
    noise = float(cond[5:])
elif cond.startswith("rise"):
    rise = float(cond[4:])
else:
    assert cond in ("base", "noframe"), cond


def course_of(seed):
    c = random_course(np.random.default_rng(seed))
    return c if rise is None else Course(c.faces, (rise,) * len(c.faces))


def blind(make):
    """Ablation: the executor sees every call in the training step's frame (no step frame)."""
    return lambda call, k: make(replace(call, face=STEP_X, floor=0.0, rise=STEP_H, back=-np.inf), k)


def controller(mode, course):
    if mode == "expert":
        return SwarmClimbPolicy(n, 0.1, course)
    ex = script_executor() if mode == "script" else wam_executor(old if mode == "mosaic0" else wams)
    return SkillScheduler(n, effect, blind(ex) if cond == "noframe" else ex)


res = {m: {"success": [], "steps": [], "restarts": []} for m in modes}
rng = np.random.default_rng(0)
for seed in seeds:
    course = course_of(seed)
    for mode in modes:
        env = SphereSwarmStepEnv(n_spheres=n, course=course)
        body = env.model.body_mass > 0
        env.model.body_mass[body] *= mass
        env.model.body_inertia[body] *= mass
        env.model.geom_friction[:, 0] *= fric
        obs, _ = env.reset(seed=seed)
        ctl = controller(mode, course)
        sp = np.concatenate(env._kinematics(), 1)
        if mode != "expert":
            ctl.reset(sp[:, :3], env.goal, course)
        t, info = 0, {}
        while env.agents:
            if noise > 0:  # position noise [m] and twice that on velocities, every sensor
                sp = sp + noise * rng.standard_normal(sp.shape) * np.repeat([1, 2, 2], 3)
                obs = {a: o + noise * rng.standard_normal(o.shape) for a, o in obs.items()}
            if mode == "expert":
                act = ctl.act(obs)
            else:
                a = ctl.act([obs[x] for x in env.possible_agents], sp, env.links)
                act = dict(zip(env.possible_agents, a, strict=True))
            obs, _, _, _, infos = env.step(act)
            info, t = infos[env.possible_agents[0]], t + 1
            sp = np.concatenate(env._kinematics(), 1)
        res[mode]["success"].append(bool(info["is_success"]))
        res[mode]["steps"].append(t)
        res[mode]["restarts"].append(0 if mode == "expert" else ctl.restarts)
res = {m: {k: np.asarray(v) for k, v in d.items()} for m, d in res.items()}
pickle.dump(res, open(f"{root}/adapt_{out}.pkl", "wb"))
for m, d in res.items():
    ok = d["success"]
    st = d["steps"][ok].mean() if ok.any() else float("nan")
    print(f"{out} {m}: {ok.sum()}/{len(ok)}, steps {st:.1f}, restarts {d['restarts'].sum()}",
          flush=True)  # fmt: skip
