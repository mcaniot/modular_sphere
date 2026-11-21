"""Greedy go-to-goal policy (no links) at low steps: success and steps (curriculum sanity)."""

import numpy as np

from modular_sphere.swarm import SphereSwarmStepEnv, obs_slices

sl = obs_slices(4)
for h in (0.02, 0.04, 0.06):
    res = []
    for seed in range(10):
        env = SphereSwarmStepEnv(n_spheres=4, step_height=h)
        obs, _ = env.reset(seed=300_000 + seed)
        t, info = 0, {}
        while env.agents:
            act = {}
            for a, o in obs.items():
                g = o[sl["goal_rel"]][:2]
                f = np.clip(3.0 * g - 1.0 * o[sl["vel"]][:2], -1, 1)
                act[a] = np.array([f[0], f[1], 0.0, -1.0])
            obs, _, _, _, infos = env.step(act)
            info, t = infos["sphere_0"], t + 1
        res.append((info["is_success"], t, info["n_arrived"]))
    s = np.array(res)
    print(f"h={h}: success {s[:, 0].sum()}/10, steps {s[:, 1].mean():.0f}, arrived {s[:, 2].mean():.1f}",
          flush=True)
