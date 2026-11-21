"""Highest step a lone sphere climbs with full push (curriculum start)."""

import numpy as np

from modular_sphere.swarm import STEP_X, SphereSwarmStepEnv

for h in (0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.1):
    env = SphereSwarmStepEnv(n_spheres=2, step_height=h, max_steps=200)
    obs, _ = env.reset(seed=0)
    up = False
    while env.agents:
        obs, *_ = env.step({"sphere_0": np.array([1.0, 0, 0, -1]),
                            "sphere_1": np.array([1.0, 0, 0, -1])})
        pos = env.state()[:3]
        up |= pos[0] > STEP_X + 0.1 and pos[2] > h + 0.05
    print(f"h={h}: lone sphere up: {up}", flush=True)
