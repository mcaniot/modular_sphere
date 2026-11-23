"""E2 diagnosis: do the world models rank the policy's candidate chunks like the simulator does?

Usage: modular_e2_rank.py CONDITION [seeds=5] [seed0=600000] [every=5] [k=16] [sigma=0.3] [h=10]
    [wms=wm_modular_64x2_s0,wm_frame_256x4_s0,wm_world_256x4_s0]

Runs MOSAIC (WAM executors, the policy's action always executed) on random courses; every
``every`` steps of every skill call, scores the same ``k`` candidate chunks (``WMVerifier``
sampling) by the skill's steps-to-go value after ``h`` steps predicted by each world model and by
MuJoCo itself (state saved and restored: the oracle). Reports per world model the Spearman rank
correlation with the oracle, and the oracle steps-to-go gain of the world model's choice over the
policy's chunk (negative: better), against the oracle's own best choice.
Saves data/swarm/modular/e2_rank_{CONDITION}.pkl.
"""

import pickle
import sys
from pathlib import Path

import mujoco
import numpy as np
import torch
from scipy.stats import spearmanr

from modular_sphere.swarm import Course, SphereSwarmStepEnv, random_course
from modular_sphere.swarm_modular import WMVerifier, load_wm, simulator_rollouts
from modular_sphere.swarm_plan import EffectModel, SkillScheduler
from modular_sphere.swarm_skills import SKILLS, SkillValue, skill_features
from modular_sphere.swarm_wam import SkillWAM

ROOT = Path(__file__).resolve().parents[3] / "data/swarm"
cond = sys.argv[1]
kw = dict(a.split("=") for a in sys.argv[2:])
seed0, n_seeds, every = (
    int(kw.get("seed0", "600000")),
    int(kw.get("seeds", "5")),
    int(kw.get("every", "5")),
)
k, sigma, h = int(kw.get("k", "16")), float(kw.get("sigma", "0.3")), int(kw.get("h", "10"))
stems = kw.get("wms", "wm_modular_64x2_s0,wm_frame_256x4_s0,wm_world_256x4_s0").split(",")
torch.set_num_threads(2)
sk = f"{ROOT}/skills"
values = {s: SkillValue.load(f"{sk}/value_{s}.pt") for s in SKILLS}
effect = EffectModel(values)
wams = {s: SkillWAM.load(f"{sk}/chain_wam_mixe_r0_{s}.pt") for s in SKILLS}
scorers = {s: WMVerifier(load_wm(f"{ROOT}/modular/{s}.pt"), values, k=k, sigma=sigma, horizon=h)
           for s in stems}  # fmt: skip

n, mass, noise, rise = 4, 1.0, 0.0, None
if cond.startswith("mass"):
    mass = float(cond[4:])
elif cond.startswith("noise"):
    noise = float(cond[5:])
elif cond.startswith("rise"):
    rise = float(cond[4:])
elif cond[0] == "n" and cond[1:].isdigit():
    n = int(cond[1:])
else:
    assert cond == "base", cond


def oracle(env, call, act):
    """Steps to go (K,) and sphere states (K, N, 9) after each candidate in MuJoCo."""
    xs, lks = simulator_rollouts(env, act)
    x = torch.as_tensor(xs, dtype=torch.float32)
    return values[call.name](skill_features(call, x, torch.as_tensor(lks))), x


rec = {s: {"rho": [], "gain": [], "skill": [], "err": []} for s in stems}
rec["oracle"] = {"gain": [], "skill": [], "spread": []}
rng = np.random.default_rng(0)
for seed in range(seed0, seed0 + n_seeds):
    c = random_course(np.random.default_rng(seed))
    course = c if rise is None else Course(c.faces, (rise,) * len(c.faces))
    env = SphereSwarmStepEnv(n_spheres=n, course=course)
    body = env.model.body_mass > 0
    env.model.body_mass[body] *= mass
    env.model.body_inertia[body] *= mass
    mujoco.mj_setConst(env.model, env.data)
    obs, _ = env.reset(seed=seed)
    probe = WMVerifier(scorers[stems[0]].wm, values, k=k, sigma=sigma, horizon=h, seed=seed)
    sched = SkillScheduler(n, effect, probe.executor(wams))
    probe.scheduler, probe.course = sched, course
    for v in scorers.values():
        v.scheduler, v.course = sched, course
    tick = {"t": 0}

    def measure(call, act, spheres, links, env=env, probe=probe, tick=tick):
        tick["t"] += 1
        if tick["t"] % every == 0:
            true, xt = oracle(env, call, act)
            true = true.numpy()
            rec["oracle"]["spread"].append(true.std())
            rec["oracle"]["gain"].append(true.min() - true[0])
            rec["oracle"]["skill"].append(call.name)
            for s, v in scorers.items():
                v.last = probe.last
                xp = v.predict(act, spheres, links)
                rec[s]["err"].append((xp[..., :3] - xt[..., :3]).norm(dim=-1).mean().item())
                pred = v.score(call, act, spheres, links).numpy()
                rec[s]["rho"].append(spearmanr(pred, true).statistic)
                rec[s]["gain"].append(true[int(pred.argmin())] - true[0])
                rec[s]["skill"].append(call.name)
        return torch.zeros(len(act))  # always execute the policy's chunk

    probe.scorer = measure
    sp = env.spheres()
    sched.reset(sp[:, :3], env.goal, course)
    while env.agents:
        if noise > 0:
            sp = sp + noise * rng.standard_normal(sp.shape) * np.repeat([1, 2, 2], 3)
        a = sched.act([obs[x] for x in env.possible_agents], sp, env.links)
        probe.last = a
        obs, _, _, _, infos = env.step(dict(zip(env.possible_agents, a, strict=True)))
        sp = env.spheres()
    print(f"{seed}: success {infos[env.possible_agents[0]]['is_success']}, "
          f"decisions {len(rec['oracle']['gain'])}", flush=True)  # fmt: skip

pickle.dump(rec, open(f"{ROOT}/modular/e2_rank_{cond}.pkl", "wb"))
og = np.array(rec["oracle"]["gain"])
print(f"oracle best gain over the policy chunk: mean {og.mean():.2f} steps, "
      f"<= -5 steps in {np.mean(og <= -5):.0%} of decisions")  # fmt: skip
for s in stems:
    rho, g = np.array(rec[s]["rho"]), np.array(rec[s]["gain"])
    print(f"{s}: Spearman {np.nanmean(rho):.2f}, gain of its choice {g.mean():+.2f} steps "
          f"(better {np.mean(g < 0):.0%}, worse {np.mean(g > 0):.0%})")  # fmt: skip
print(f"oracle steps-to-go spread across candidates: {np.mean(rec['oracle']['spread']):.2f}")
for s in stems:
    print(f"{s}: end-position error {100 * np.mean(rec[s]['err']):.2f} cm")
