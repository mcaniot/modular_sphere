"""S1: skill controller success and duration per skill (noise 0 and 0.2)."""

import sys

from modular_sphere.swarm_skills import SKILLS, collect_skill

n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
for name in SKILLS:
    for noise in (0.0, 0.2):
        d = collect_skill(name, n, seed=0, noise=noise)
        ok = d.success
        print(f"{name} noise {noise}: success {ok.sum()}/{n}, steps on success "
              f"{d.steps[ok].mean():.0f} (max {d.steps[ok].max() if ok.any() else 0})", flush=True)
