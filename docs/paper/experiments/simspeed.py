"""Env throughput (steps/s, one process) and expert parallelism (fraction of spheres moving)."""

import time

import numpy as np

from modular_sphere.swarm import SphereSwarmStepEnv, SwarmClimbPolicy, obs_slices

env = SphereSwarmStepEnv(n_spheres=4)
obs, _ = env.reset(seed=0)
rng = np.random.default_rng(0)
t0, n = time.time(), 0
while n < 2000:
    act = {a: rng.uniform(-1, 1, 4) for a in env.agents}
    obs, r, term, trunc, _ = env.step(act)
    n += 1
    if any(term.values()) or any(trunc.values()):
        obs, _ = env.reset(seed=n)
print(f"random: {n / (time.time() - t0):.0f} steps/s", flush=True)

vel = obs_slices(4)["vel"]
for seed in range(5):
    obs, _ = env.reset(seed=200_000 + seed)
    pol = SwarmClimbPolicy(4)
    moving, links, steps = [], [], 0
    while env.agents:
        obs, r, term, trunc, info = env.step(pol.act(obs))
        moving.append(np.mean([np.linalg.norm(o[vel][:2]) > 0.05 for o in obs.values()]))
        links.append(next(iter(info.values()))["n_links"])
        steps += 1
    print(f"expert seed {seed}: {steps} steps, moving frac {np.mean(moving):.2f}, "
          f"link-steps {np.mean(links):.2f}", flush=True)
