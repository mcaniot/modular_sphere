"""ModSwarm skill chaining: skill data whose starts are the end states of the preceding skills.

Usage: skills_chain.py DEVICE KIND [rounds=3] [episodes=100] [noise=0.2] [val=50] [tag=]
    [input_noise=0] [dynamics=0]
    [rises=LOW,HIGH] [extra=NAME,...] (segments of chain_segments_{NAME}.pkl added to every fit)
KIND is ``bc512`` (cloned MLP, 512 units) or ``wam``. Round 0 runs the scheduler with the scripted
skills and DART noise in the four-sphere env (train seeds 0.., never the validation seeds
300000.., the test seeds 200000.. or the held-out six-sphere seeds 400000..) and records every skill
call with the scripted labels; each later round runs the scheduler with the learned executors of
the previous round (DAgger, scripted labels). Each round refits the executors on the two-sphere
skill data plus all chain data, then evaluates the scheduler on the validation seeds. Saves
data/swarm/skills/chain_{KIND}_r{round}_{skill}.pt, chain_segments_{KIND}.pkl and
chain_{KIND}_val.pkl ({round: ScheduleEval as dict}).
"""

import pickle
import sys
import time
from dataclasses import asdict

import numpy as np

from modular_sphere.swarm import STEP_X, Course
from modular_sphere.swarm_data import SwarmDataset
from modular_sphere.swarm_plan import (
    EffectModel,
    SkillScheduler,
    evaluate_schedule,
    perturb_dynamics,
    prior_executor,
    script_executor,
    segments_to_data,
    wam_executor,
)
from modular_sphere.swarm_skills import SKILLS, SkillCall, SkillPrior, SkillValue
from modular_sphere.swarm_wam import SkillWAM

device, kind = sys.argv[1], sys.argv[2]
kw = dict(a.split("=") for a in sys.argv[3:])
rounds, episodes = int(kw.get("rounds", "3")), int(kw.get("episodes", "100"))
noise, n_val = float(kw.get("noise", "0.2")), int(kw.get("val", "50"))
input_noise = float(kw.get("input_noise", "0"))  # WAM sensor-noise augmentation [m]
dyn = perturb_dynamics if kw.get("dynamics", "0") == "1" else None  # randomised chain data
rises = [float(v) for v in kw["rises"].split(",")] if "rises" in kw else None


def courses(seed):  # single step of random rise (chain data only)
    return Course((STEP_X,), (round(float(np.random.default_rng(seed).uniform(*rises)), 3),))


tag = kind + kw.get("tag", "")  # output name: chain_{KIND}{tag}_...
root = "/home/mcaniot/Documents/modular_sphere/data/swarm/skills"
VAL = list(range(300_000, 300_000 + n_val))
N_AGENTS = {"roll": 1, "climb": 2, "pull": 2}
effect = EffectModel({s: SkillValue.load(f"{root}/value_{s}.pt", device) for s in SKILLS})


def base_data(name):
    data = SwarmDataset.load(f"{root}/{name}_train.npz")
    with open(f"{root}/{name}_outcomes.pkl", "rb") as f:
        targets = np.concatenate([p[1] for p in pickle.load(f)["train"]])
    ag = tuple(range(N_AGENTS[name]))
    return data, [SkillCall(name, ag, t) for t in targets]


def fit(name, data, calls, r):
    path = f"{root}/chain_{tag}_r{r}_{name}.pt"
    if kind == "bc512":
        model = SkillPrior(N_AGENTS[name], hidden=512).to(device)
        model.fit(data, calls, epochs=60, lr=3e-4, batch_size=512)
    else:
        model = SkillWAM(N_AGENTS[name]).to(device)
        model.fit(data, calls, input_noise=input_noise)
    model.save(path)
    return model


def executor(models):
    return prior_executor(models) if kind == "bc512" else wam_executor(models)


extra = []
for name in kw["extra"].split(",") if "extra" in kw else []:
    with open(f"{root}/chain_segments_{name}.pkl", "rb") as f:
        extra += pickle.load(f)
segments, val_res = [], {}
make = script_executor()
for r in range(rounds):
    t0 = time.time()
    sched = SkillScheduler(4, effect, make, labeler=script_executor(), noise=noise, seed=r)
    seeds = list(range(r * episodes, (r + 1) * episodes))
    ev = evaluate_schedule(sched, seeds, courses if rises else None, dyn)
    segments += sched.segments
    pickle.dump(segments, open(f"{root}/chain_segments_{tag}.pkl", "wb"))
    counts = {s: sum(seg.call.name == s for seg in sched.segments) for s in SKILLS}
    models = {}
    for name in SKILLS:
        data, calls = base_data(name)
        chain, chain_calls = segments_to_data(extra + segments, name)
        models[name] = fit(name, SwarmDataset.concat([data, chain]), calls + chain_calls, r)
    make = executor(models)
    val = evaluate_schedule(SkillScheduler(4, effect, make), VAL)
    val_res[r] = asdict(val)
    pickle.dump(val_res, open(f"{root}/chain_{tag}_val.pkl", "wb"))
    ok = val.success
    print(f"round {r}: collect success {ev.success.sum()}/{episodes}, segments {counts}; "
          f"val {ok.sum()}/{n_val}, steps {val.steps[ok].mean():.1f}, "
          f"restarts {val.restarts.sum()} ({time.time() - t0:.0f}s)", flush=True)  # fmt: skip
