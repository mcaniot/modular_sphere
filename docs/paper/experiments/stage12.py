"""MAPPO from scratch (no demonstration) with a step-height curriculum; val at the full step."""

import sys
import time
from pathlib import Path

import numpy as np

from modular_sphere.swarm import STEP_H
from modular_sphere.swarm_data import expert_steps
from modular_sphere.swarm_rl import SwarmVecEnv, evaluate, train_mappo
from modular_sphere.swarm_torch import SwarmPolicy, _PolicyNet


def main():
    iters, every, tp, tag = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    kw = {k: (v if k == "init" else float(v)) for k, v in (a.split("=") for a in sys.argv[5:])}
    h0 = kw.pop("h0", 0.04)
    for k in ("horizon", "epochs", "critic_warmup", "minibatch"):
        if k in kw:
            kw[k] = int(kw[k])
    out = Path("/home/mcaniot/Documents/modular_sphere/data/swarm")
    N = 4
    VAL = list(range(300_000, 300_020))
    ref = expert_steps(N, VAL)
    vec = SwarmVecEnv(N, n_workers=5, envs_per_worker=4, time_penalty=tp)
    ev_vec = SwarmVecEnv(N, n_workers=1, envs_per_worker=20)
    pol = SwarmPolicy.load(str(out / "dagger.pt"), "cpu")  # observation statistics only
    init = kw.pop("init", None)
    pol.net = _PolicyNet(N, pol.hidden)
    if init:
        pol = SwarmPolicy.load(str(out / init), "cpu")
    height, streak = h0, [0]
    vec.set_step_height(height)
    vec.reset()
    state = {"h": height, "t0": time.time(), "best": -1.0}

    def cb(it, m):
        if m["success"] == m["success"]:  # nan: no episode ended in this rollout, keep streak
            streak[0] = streak[0] + 1 if m["success"] >= 0.7 else 0
        if streak[0] >= 3 and state["h"] < STEP_H - 1e-9:
            state["h"] = min(STEP_H, round(state["h"] + 0.01, 3))
            vec.set_step_height(state["h"])
            streak[0] = 0
            print(f"{tag} it={it + 1} curriculum -> h={state['h']:.2f}", flush=True)
        if (it + 1) % every:
            return
        ev = evaluate(pol, ev_vec, VAL)
        speed = np.where(ev.success, ref / ev.steps, 0.0).mean()
        st = ev.steps[ev.success].mean() if ev.success.any() else float("nan")
        print(f"{tag} it={it + 1} h={state['h']:.2f} train succ {m['success']:.2f} "
              f"len {m['length']:.0f} std {m['std']:.2f} | val(h=0.15) {ev.success.sum()}/20, "
              f"steps {st:.0f}, speed {speed:.3f}, moving {ev.moving.mean():.2f}, "
              f"links {ev.links.mean():.2f} ({time.time() - state['t0']:.0f}s)", flush=True)
        state["t0"] = time.time()
        pol.save(str(out / f"{tag}_last.pt"))
        if speed > state["best"]:
            state["best"] = speed
            pol.save(str(out / f"{tag}_best.pt"))

    train_mappo(pol, vec, iterations=iters, callback=cb, **kw)
    vec.close()
    ev_vec.close()


if __name__ == "__main__":
    main()
