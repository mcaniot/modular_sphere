"""S4: skill scheduler (parallel lanes chosen with skill effect models) vs expert and DAgger.

Usage: skills_s4.py DEVICE N_SEEDS [modes=expert,dagger,script,script_serial,bc512,wam] [n=4]
Test seeds 200000.. as stage10.py. Saves data/swarm/skills/s4_{N_SEEDS}.pkl, merged with previous
runs: {mode: dict of per-seed arrays (success, steps, moving[, predicted, lanes, restarts])}.
"""

import os
import pickle
import sys
import time
from dataclasses import asdict

import numpy as np

from modular_sphere.swarm import SwarmClimbPolicy
from modular_sphere.swarm_plan import (
    EffectModel,
    SkillScheduler,
    evaluate_schedule,
    prior_executor,
    script_executor,
    wam_executor,
)
from modular_sphere.swarm_skills import SKILLS, SkillPrior, SkillValue
from modular_sphere.swarm_torch import SwarmPolicy
from modular_sphere.swarm_wam import SkillWAM

device, n_seeds = sys.argv[1], int(sys.argv[2])
kw = dict(a.split("=") for a in sys.argv[3:])
modes = kw.get("modes", "expert,dagger,script,script_serial,bc512,wam").split(",")
n = int(kw.get("n", "4"))
seeds = list(range(200_000, 200_000 + n_seeds))
data = "/home/mcaniot/Documents/modular_sphere/data/swarm"
root = f"{data}/skills"
path = f"{root}/s4_{n_seeds}.pkl"
results = pickle.load(open(path, "rb")) if os.path.exists(path) else {}
effect = EffectModel({s: SkillValue.load(f"{root}/value_{s}.pt", device) for s in SKILLS})


def executor(mode):
    if mode.startswith("script"):
        return script_executor()
    if mode == "bc512":
        return prior_executor(
            {s: SkillPrior.load(f"{root}/prior_{s}_mlp_wide_s0.pt", device) for s in SKILLS}
        )
    return wam_executor({s: SkillWAM.load(f"{root}/wam_{s}.pt", device) for s in SKILLS})


class Whole:
    """A whole-swarm policy (expert or DAgger) behind the scheduler interface."""

    def __init__(self, policy):
        self.n, self.policy, self.lanes = n, policy, []

    def reset(self, pos, goal):
        self.predicted, self.restarts = 0.0, 0
        if hasattr(self.policy, "reset"):  # the expert keeps its roles
            self.policy.reset()

    def act(self, obs, spheres, links):
        policy = getattr(self.policy, "act", self.policy)
        out = policy({f"sphere_{i}": o for i, o in enumerate(obs)})
        return np.stack([out[f"sphere_{i}"] for i in range(self.n)])


for mode in modes:
    t0 = time.time()
    if mode == "expert":
        res = asdict(evaluate_schedule(Whole(SwarmClimbPolicy(n)), seeds))
    elif mode == "dagger":
        dagger = SwarmPolicy.load(f"{data}/dagger.pt", "cpu")
        res = asdict(evaluate_schedule(Whole(dagger), seeds))
    else:
        lanes = 1 if mode.endswith("_serial") else None
        sched = SkillScheduler(n, effect, executor(mode), max_lanes=lanes)
        res = asdict(evaluate_schedule(sched, seeds))
    results[mode] = res
    pickle.dump(results, open(path, "wb"))
    ok, st = res["success"], res["steps"]
    extra = ""
    if "predicted" in res:
        extra = (f", lanes {np.bincount(res['lanes']).tolist()}, predicted "
                 f"{res['predicted'][ok].mean():.0f}, "
                 f"restarts {res['restarts'].sum()}")  # fmt: skip
    moving = f", moving {res['moving'].mean():.2f}" if "moving" in res else ""
    print(f"{mode}: {ok.sum()}/{n_seeds}, steps on success {st[ok].mean():.1f}{moving}{extra} "
          f"({time.time() - t0:.0f}s)", flush=True)  # fmt: skip
