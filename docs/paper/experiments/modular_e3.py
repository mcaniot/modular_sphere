"""E3: adding a skill to a world model (continual learning, Zhou et al. 2026 protocol).

Usage: modular_e3.py KIND WIDTH LAYERS SEED DEVICE [epochs=40] [epochs2=20] [k=10,all]

Phase 1 trains on the training windows (E1 ``train`` set, W=8) where no sphere pulls; phase 2
adds the pull skill from the windows with a pull of the first ``k`` training episodes that contain
one. ``modular`` trains only its new pull expert in phase 2 (the other experts are frozen); the
shared models are fine-tuned on all their parameters (the normalisation of phase 1 is kept for
every model). Test windows (E1 ``test`` and ``courses`` sets, h=30) are split into windows
without and with a pull: forgetting (backward transfer) is the change of the error on the first,
learning of the new skill the error on the second. Saves data/swarm/modular/e3.pkl
{(KIND_WxL, seed): {phase: {set: errors (M, h, N)}}} (phases ``p1``, ``p2_k{k}``).
"""

import copy
import pickle
import sys
import time
from pathlib import Path

import numpy as np
import torch

from modular_sphere.swarm_modular import SKILL_IDS, Episodes, SwarmWM, fit, position_errors, windows

OUT = Path(__file__).resolve().parents[3] / "data/swarm/modular"
PULL = SKILL_IDS["pull"]


def has_pull(w):
    return (w["cat"] // 2 == PULL).flatten(1).any(1) & w["mask"].any(1)


def take(w, sel):
    return {k: v[sel] for k, v in w.items()}


def main():
    kind, width, layers, seed, dev = sys.argv[1:6]
    width, layers, seed = int(width), int(layers), int(seed)
    kw = dict(a.split("=") for a in sys.argv[6:])
    epochs, epochs2 = int(kw.get("epochs", "40")), int(kw.get("epochs2", "20"))
    ks = kw.get("k", "10,all").split(",")
    path = OUT / "e3.pkl"
    if path.exists() and (f"{kind}_{width}x{layers}", seed) in pickle.load(open(path, "rb")):
        return  # already done (resume)
    train = windows(Episodes.load(f"{OUT}/train.npz"), 8, 2, dev)
    pull = has_pull(train)
    tests = {}
    for s in ("test", "courses"):
        w = windows(Episodes.load(f"{OUT}/{s}.npz"), 30, 10, dev)
        p = has_pull(w)
        tests[f"{s}_nopull"], tests[f"{s}_pull"] = take(w, ~p), take(w, p)

    def errors(model):
        model.eval()
        out = {s: position_errors(model, w) for s, w in tests.items()}
        model.train()
        return out

    model = SwarmWM(kind, width, layers).to(dev)
    t0 = time.time()
    fit(model, take(train, ~pull), epochs, seed=seed)
    res = {"p1": errors(model), "time_p1": time.time() - t0}
    eps = torch.unique(train["episode"][pull])  # episodes with a pull, in order
    for k in ks:
        keep = eps if k == "all" else eps[: int(k)]
        data = take(train, pull & torch.isin(train["episode"], keep))
        m = copy.deepcopy(model)
        params = list(m.nets[PULL].parameters()) if kind == "modular" else list(m.parameters())
        t0 = time.time()
        fit(m, data, epochs2, seed=seed, params=params)
        res[f"p2_k{k}"], res[f"time_p2_k{k}"] = errors(m), time.time() - t0
    allres = pickle.load(open(path, "rb")) if path.exists() else {}
    allres[(f"{kind}_{width}x{layers}", seed)] = res
    pickle.dump(allres, open(path, "wb"))
    for ph in ["p1"] + [f"p2_k{k}" for k in ks]:
        row = "  ".join(
            f"{s} {np.sqrt(np.nanmean(e[:, -1] ** 2)) * 100:.1f}" for s, e in res[ph].items()
        )
        print(f"{kind}_{width}x{layers} s{seed} {ph}: h30 RMSE [cm] {row}")


if __name__ == "__main__":
    main()
