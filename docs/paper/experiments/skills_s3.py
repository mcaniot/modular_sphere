"""S3: MPPI in each skill world model drives the skill in MuJoCo (two-sphere env)."""

import sys
import time
from pathlib import Path

import numpy as np
import torch

from modular_sphere.swarm import SphereSwarmStepEnv
from modular_sphere.swarm_skills import (
    MAX_SKILL_STEPS,
    SKILLS,
    SkillCall,
    SkillController,
    SkillPlanner,
    SkillPrior,
    SkillValue,
    place,
    sample_start,
    skill_features,
)
from modular_sphere.swarm_torch import SwarmWorldModel

device, n_eps = sys.argv[1], int(sys.argv[2])
kw = {k: float(v) for k, v in (a.split("=") for a in sys.argv[3:])}
for k in ("horizon", "samples", "iterations"):
    if k in kw:
        kw[k] = int(kw[k])
skills = [kw.pop("skill")] if "skill" in kw else SKILLS
mode = {0: "mppi", 1: "prior", 2: "bc", 3: "value", 4: "value+prior"}[int(kw.pop("mode", 0))]
root = str(Path(__file__).resolve().parents[3] / "data/swarm/skills")
for name in skills:
    if isinstance(name, float):
        name = SKILLS[int(name)]
    wm = SwarmWorldModel.load(f"{root}/wm_{name}.pt", device)
    rng = np.random.default_rng(123)
    env = SphereSwarmStepEnv(n_spheres=2, max_steps=MAX_SKILL_STEPS + 1)
    ag = (0,) if name == "roll" else (0, 1)
    prior = SkillPrior.load(f"{root}/prior_{name}.pt", device) if mode != "mppi" else None
    value = SkillValue.load(f"{root}/value_{name}.pt", device) if "value" in mode else None
    res, t0 = [], time.time()
    for e in range(n_eps):
        env.reset(seed=e)
        start, target = sample_start(name, rng, env.radius)
        place(env, start)
        call = SkillCall(name, ag, target)
        judge = SkillController(call, 2, env.radius)
        planner = SkillPlanner(
            wm, call, 2, seed=e, prior=prior if "prior" in mode else None, value=value, **kw
        )
        ok = False
        for t in range(MAX_SKILL_STEPS):
            pos, vel, ang = env._kinematics()
            st = np.concatenate([pos, vel, ang], 1)
            if mode == "bc":
                with torch.no_grad():
                    f = skill_features(call, wm._t(st), wm._t(env.links))
                    a = prior(f).cpu().numpy()
                act = np.zeros((2, 4))
                act[list(ag)] = a
                act[:, 3] = np.where(act[:, 3] > 0, 1.0, -1.0)
            else:
                act = planner.act(st, env.links)
            env.step({a: act[i] for i, a in enumerate(env.possible_agents)})
            pos, vel = env._kinematics()[:2]
            if judge.done(pos, vel):
                ok = True
                break
        res.append((ok, t + 1))
    r = np.array(res)
    ok = r[:, 0].astype(bool)
    print(
        f"{name} {mode} {kw}: success {ok.sum()}/{n_eps}, steps on success "
        f"{r[ok, 1].mean() if ok.any() else float('nan'):.0f} ({time.time() - t0:.0f}s)",
        flush=True,
    )
