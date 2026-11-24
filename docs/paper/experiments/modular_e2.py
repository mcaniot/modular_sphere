"""E2: does a world model that verifies the policy's proposals help the learned skills (MOSAIC)?

Usage: modular_e2.py CONDITION MODE [seeds=50] [seed0=600000] [k=16] [sigma=0.3] [h=10]
    [margin=1] [objective=value|progress] [wam=chain_wam_mixe] [device=cuda] [tag=]
CONDITION as in skills_adapt.py: base, noise0.005, mass1.5, fric0.5, n5, rise0.17, ... on random
courses (seed0..). MODE: ``mosaic`` (the WAM executors alone) or ``wm:STEM`` (the same executors
through ``WMVerifier`` with world model data/swarm/modular/STEM.pt) or ``oracle`` (the same
verifier with MuJoCo itself, from the true state, as world model: the upper bound of
verification). Saves data/swarm/modular/e2_{CONDITION}_{MODE}{tag}.pkl {success, steps, restarts,
overrides} per seed.
"""

import pickle
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import torch

from modular_sphere.swarm import Course, SphereSwarmStepEnv, random_course
from modular_sphere.swarm_modular import (
    WMVerifier,
    load_wm,
    progress_cost,
    simulator_scorer,
)
from modular_sphere.swarm_plan import EffectModel, SkillScheduler, wam_executor
from modular_sphere.swarm_skills import SKILLS, SkillValue
from modular_sphere.swarm_wam import SkillWAM

ROOT = Path(__file__).resolve().parents[3] / "data/swarm"
cond, mode = sys.argv[1], sys.argv[2]
kw = dict(a.split("=") for a in sys.argv[3:])
dev = kw.get("device", "cuda")
torch.set_num_threads(1)  # several conditions run side by side on the CPU
seed0 = int(kw.get("seed0", "600000"))
seeds = list(range(seed0, seed0 + int(kw.get("seeds", "50"))))
skills = f"{ROOT}/skills"
effect = EffectModel({s: SkillValue.load(f"{skills}/value_{s}.pt") for s in SKILLS})
values = {s: SkillValue.load(f"{skills}/value_{s}.pt", dev) for s in SKILLS}
wams = {s: SkillWAM.load(f"{skills}/{kw.get('wam', 'chain_wam_mixe')}_r0_{s}.pt") for s in SKILLS}

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
    assert cond == "base", cond

verifier = None
if mode.startswith("wm:") or mode == "oracle":
    wm = load_wm(f"{ROOT}/modular/{mode[3:] if mode != 'oracle' else 'wm_modular_64x2_s0'}.pt", dev)
    opts = {"k": int(kw.get("k", "16")), "sigma": float(kw.get("sigma", "0.3")),
            "horizon": int(kw.get("h", "10")), "margin": float(kw.get("margin", "1"))}  # fmt: skip
    verifier = WMVerifier(wm, values, **opts)
    if kw.get("objective", "value") == "progress":
        verifier.objective = progress_cost
    else:
        assert kw.get("objective", "value") == "value", kw["objective"]
else:
    assert mode == "mosaic", mode

res = {"success": [], "steps": [], "restarts": [], "overrides": [], "time": []}
rng = np.random.default_rng(0)
for seed in seeds:
    c = random_course(np.random.default_rng(seed))
    course = c if rise is None else Course(c.faces, (rise,) * len(c.faces))
    env = SphereSwarmStepEnv(n_spheres=n, course=course)
    body = env.model.body_mass > 0
    env.model.body_mass[body] *= mass
    env.model.body_inertia[body] *= mass
    env.model.geom_friction[:, 0] *= fric
    mujoco.mj_setConst(env.model, env.data)
    obs, _ = env.reset(seed=seed)
    ex = wam_executor(wams) if verifier is None else verifier.executor(wams)
    sched = SkillScheduler(n, effect, ex)
    if verifier is not None:
        verifier.scheduler, verifier.course, verifier.last, verifier.overrides = (
            sched,
            course,
            None,
            0,
        )
        if mode == "oracle":
            verifier.scorer = simulator_scorer(env, values, verifier.objective)
    sp = env.spheres()
    sched.reset(sp[:, :3], env.goal, course)
    t, info, t0 = 0, {}, time.time()
    while env.agents:
        if noise > 0:  # position noise [m] and twice that on velocities, as skills_adapt.py
            sp = sp + noise * rng.standard_normal(sp.shape) * np.repeat([1, 2, 2], 3)
            obs = {a: o + noise * rng.standard_normal(o.shape) for a, o in obs.items()}
        a = sched.act([obs[x] for x in env.possible_agents], sp, env.links)
        if verifier is not None:
            verifier.last = a
        obs, _, _, _, infos = env.step(dict(zip(env.possible_agents, a, strict=True)))
        info, t = infos[env.possible_agents[0]], t + 1
        sp = env.spheres()
    res["success"].append(bool(info["is_success"]))
    res["steps"].append(t)
    res["restarts"].append(sched.restarts)
    res["overrides"].append(0 if verifier is None else verifier.overrides)
    res["time"].append(time.time() - t0)
    print(f"{seed}: {res['success'][-1]} {t} steps, restarts {sched.restarts}, "
          f"overrides {res['overrides'][-1]}, {res['time'][-1]:.0f} s", flush=True)  # fmt: skip
res = {k: np.asarray(v) for k, v in res.items()}
name = f"e2_{cond}_{mode.replace(':', '_')}{kw.get('tag', '')}"
pickle.dump(res, open(f"{ROOT}/modular/{name}.pkl", "wb"))
ok = res["success"]
print(f"{name}: {ok.sum()}/{len(ok)}, steps {res['steps'][ok].mean():.1f}, "
      f"restarts {res['restarts'].sum()}, overrides {res['overrides'].sum()}")  # fmt: skip
