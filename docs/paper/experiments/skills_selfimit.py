"""S3d: GRPO-induced self-imitation of a skill WAM in MuJoCo (GR00T 2 recipe at small scale).

Per round: GROUP noisy rollouts from each of STARTS sampled starts, keep per group the successful
rollouts at least as fast as the group's mean successful duration, fine-tune the WAM on them mixed
with the expert skill data (replay), evaluate deterministically on the fixed evaluation starts.

Usage: skills_selfimit.py DEVICE SKILL [rounds=3] [starts=40] [group=8] [noise=0.2] [eval=50]
"""

import dataclasses
import pickle
import sys
import time
from pathlib import Path

import numpy as np

from modular_sphere.swarm import SphereSwarmStepEnv
from modular_sphere.swarm_data import SwarmDataset
from modular_sphere.swarm_skills import (
    MAX_SKILL_STEPS,
    SkillCall,
    SkillController,
    place,
    sample_start,
)
from modular_sphere.swarm_wam import SkillWAM, WAMController, rollout_skill, select_elite

device, name = sys.argv[1], sys.argv[2]
kw = dict(a.split("=") for a in sys.argv[3:])
rounds, starts, group = int(kw.get("rounds", 3)), int(kw.get("starts", 40)), int(kw.get("group", 8))
noise, n_eval = float(kw.get("noise", 0.2)), int(kw.get("eval", 50))
ens = None if kw.get("ens", "0.1") == "none" else float(kw.get("ens", 0.1))
root = str(Path(__file__).resolve().parents[3] / "data/swarm/skills")
ag = (0,) if name == "roll" else (0, 1)


def subset(d: SwarmDataset, keep: np.ndarray) -> SwarmDataset:
    return SwarmDataset(**{f.name: getattr(d, f.name)[keep] for f in dataclasses.fields(d)})


def evaluate(wam: SkillWAM) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(123)
    env = SphereSwarmStepEnv(n_spheres=2, max_steps=MAX_SKILL_STEPS + 1)
    ok, steps = np.zeros(n_eval, dtype=bool), np.zeros(n_eval, dtype=int)
    for e in range(n_eval):
        env.reset(seed=e)
        start, target = sample_start(name, rng, env.radius)
        place(env, start)
        call = SkillCall(name, ag, target)
        judge, ctrl = SkillController(call, 2, env.radius), WAMController(wam, call, 2, ens)
        for t in range(MAX_SKILL_STEPS):
            act = ctrl.act(env.spheres(), env.links)
            env.step({a: act[i] for i, a in enumerate(env.possible_agents)})
            pos, vel = env._kinematics()[:2]
            if judge.done(pos, vel):
                ok[e] = True
                break
        steps[e] = t + 1
    return ok, steps


expert = SwarmDataset.load(f"{root}/{name}_train.npz")
with open(f"{root}/{name}_outcomes.pkl", "rb") as f:
    targets = np.concatenate([p[1] for p in pickle.load(f)["train"]])
expert_calls = [SkillCall(name, ag, t) for t in targets]
wam = SkillWAM.load(f"{root}/wam_{name}.pt", device)
ok, steps = evaluate(wam)
results = {"r0": (ok, steps)}
print(f"{name} round 0: success {ok.sum()}/{n_eval}, steps on success {steps[ok].mean():.1f}",
      flush=True)  # fmt: skip
for r in range(1, rounds + 1):
    t0 = time.time()
    data, calls, st = rollout_skill(wam, name, starts, group, noise, seed=100_000 * r, ensemble=ens)
    keep = select_elite(data.success, st, group)
    elite = subset(data, keep)
    elite_calls = [c for c, k in zip(calls, keep, strict=True) if k]
    mixed = SwarmDataset.concat([elite, expert])
    wam.fit(mixed, elite_calls + expert_calls, epochs=15, lr=1e-4, keep_stats=True, seed=r)
    wam.save(f"{root}/wam_{name}_si{r}.pt")
    ok, steps = evaluate(wam)
    results[f"r{r}"] = (ok, steps)
    print(
        f"{name} round {r}: rollouts success {data.success.mean():.2f}, steps on success "
        f"{st[data.success].mean():.1f}, elite {keep.sum()}; eval success {ok.sum()}/{n_eval}, "
        f"steps on success {steps[ok].mean():.1f} ({time.time() - t0:.0f}s)",
        flush=True,
    )
with open(f"{root}/selfimit_{name}.pkl", "wb") as f:
    pickle.dump(results, f)
