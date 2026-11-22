"""Test seeds 200000..200049: DAgger vs PPO best-val checkpoints; success, speed, McNemar, paired steps."""

import sys
from pathlib import Path

import numpy as np

from modular_sphere.swarm_data import expert_steps
from modular_sphere.swarm_torch import SwarmPolicy, evaluate_policy
from modular_sphere.tools.train_swarm import mcnemar

out = Path(__file__).resolve().parents[3] / "data/swarm"
N = 4
TEST = list(range(200_000, 200_050))
ref = expert_steps(N, TEST)
print(f"expert: mean steps {ref.mean():.0f}", flush=True)
base = evaluate_policy(SwarmPolicy.load(str(out / "dagger.pt"), "cpu"), N, TEST)


def speed(ev):
    return np.where(ev.success, ref / ev.steps, 0.0)


print(f"dagger: {base.success.sum()}/50, speed {speed(base).mean():.3f}", flush=True)
for name in sys.argv[1:]:
    ev = evaluate_policy(SwarmPolicy.load(str(out / name), "cpu"), N, TEST)
    both = base.success & ev.success
    faster, slower = (ev.steps[both] < base.steps[both]), (ev.steps[both] > base.steps[both])
    sign = mcnemar(slower, faster)  # exact sign test on paired steps (ties dropped)
    print(
        f"{name}: {ev.success.sum()}/50, speed {speed(ev).mean():.3f}, McNemar {mcnemar(base.success, ev.success)}, "
        f"steps on both {base.steps[both].mean():.1f} -> {ev.steps[both].mean():.1f} "
        f"(faster on {(ev.steps[both] < base.steps[both]).sum()}/{both.sum()}, sign test p={sign['p_value']:.4f}, slower on {slower.sum()})",
        flush=True,
    )
