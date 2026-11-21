"""MAPPO in MuJoCo from DAgger: val success, speed vs expert, moving fraction, links per eval."""

import sys
import time
from pathlib import Path

import numpy as np

from modular_sphere.swarm_data import expert_steps
from modular_sphere.swarm_rl import SwarmVecEnv, evaluate, train_mappo
from modular_sphere.swarm_torch import SwarmPolicy


def main():
    device, iters, every, tp = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4])
    tag = sys.argv[5]
    extra = dict(a.split("=") for a in sys.argv[6:])
    kw = {k: float(v) for k, v in extra.items()}
    for k in ("horizon", "epochs", "critic_warmup", "minibatch"):
        if k in kw:
            kw[k] = int(kw[k])
    out = Path("/home/mcaniot/Documents/modular_sphere/data/swarm")
    N = 4
    VAL = list(range(300_000, 300_020))
    ref = expert_steps(N, VAL)
    vec = SwarmVecEnv(N, n_workers=5, envs_per_worker=4, time_penalty=tp)
    ev_vec = SwarmVecEnv(N, n_workers=1, envs_per_worker=20)
    pol = SwarmPolicy.load(str(out / "dagger.pt"), device)

    def show(name, ev):
        speed = np.where(ev.success, ref / ev.steps, 0.0).mean()
        st = ev.steps[ev.success].mean() if ev.success.any() else float("nan")
        print(f"{name}: val {ev.success.sum()}/20, steps {st:.0f}, speed {speed:.3f}, "
              f"moving {ev.moving.mean():.2f}, links {ev.links.mean():.2f}", flush=True)
        return speed

    best = [show(f"{tag} dagger", evaluate(pol, ev_vec, VAL))]
    t0 = [time.time()]

    def cb(it, m):
        if (it + 1) % every:
            return
        sp = show(f"{tag} it={it + 1} rew {m['reward']:.4f} train succ {m['success']:.2f} "
                  f"len {m['length']:.0f} std {m['std']:.2f} critic {m['critic']:.4f} "
                  f"clip {m['clip_frac']:.3f} ({time.time() - t0[0]:.0f}s)",
                  evaluate(pol, ev_vec, VAL))
        t0[0] = time.time()
        if sp >= best[0]:
            best[0] = sp
            pol.save(str(out / f"{tag}_best.pt"))
        pol.save(str(out / f"{tag}_last.pt"))

    train_mappo(pol, vec, iterations=iters, callback=cb, **kw)
    vec.close()
    ev_vec.close()


if __name__ == "__main__":
    main()
