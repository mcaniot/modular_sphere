"""E4: online adaptation of swarm world models to an unknown mass (x1.5; x0.7 as replication).

Usage: modular_e4.py COND SEED DEVICE [k=1,2,5,10,20] [lr=3e-4,1e-4] [updates=400]

The E1 models (seed SEED) trained on the nominal ``train`` set are adapted with the windows (W=8,
stride 2) of the first k episodes of the E1 set COND (``mass1.5`` or ``mass0.7``, seeds
600000-600019), then tested open loop (h=30, stride 10) on its held-out episodes 20..49 and on the
nominal ``test`` set (forgetting). Every arm gets the same budget: ``updates`` AdamW steps of
batch 256 (epochs = updates / batches per epoch) at the same learning rate, normalisation kept.
Arms: ``modular_64x2`` updates only the experts of the skills present in the adaptation data
(every adaptation episode holds all four, so all experts are updated);
``frame_128x2`` (same size), ``frame_256x4`` and ``world_256x4`` are fine-tuned on all their
parameters; ``frame_256x4_head`` only on its output layer (a cheap-adaptation baseline). k=0 is
the zero-shot model. Saves data/swarm/modular/e4_{COND}_s{SEED}.pkl
{(COND, arm, seed, lr, k): {"mass", "nominal": h30 errors (M, N) [m], "time" [s], "params"
(updated)}}; resumes by key.
"""

import copy
import math
import pickle
import sys
import time
from pathlib import Path

import numpy as np
import torch

from modular_sphere.swarm_modular import Episodes, fit, load_wm, position_errors, windows

OUT = Path(__file__).resolve().parents[3] / "data/swarm/modular"
ARMS = {  # arm: (checkpoint stem without seed, trainable part)
    "modular_64x2": ("wm_modular_64x2", "skills"),
    "frame_128x2": ("wm_frame_128x2", "all"),
    "frame_256x4": ("wm_frame_256x4", "all"),
    "frame_256x4_head": ("wm_frame_256x4", "head"),
    "world_256x4": ("wm_world_256x4", "all"),
}
N_ADAPT = 20


def take(w, sel):
    return {k: v[sel] for k, v in w.items()}


def trainable(model, part, data):
    if part == "all":
        params = list(model.parameters())
    elif part == "head":
        params = list(model.nets[0].out.parameters())
    else:
        skills = torch.unique(data["cat"][data["mask"]] // 2).tolist()
        params = [p for s in skills for p in model.nets[s].parameters()]
    model.requires_grad_(False)  # frozen parameters out of the backward pass
    for p in params:
        p.requires_grad_(True)
    return params


def save(res, path):
    tmp = path.with_suffix(".tmp")  # atomic: a crash while writing keeps the previous file
    pickle.dump(res, open(tmp, "wb"))
    tmp.replace(path)


def h30(model, data):
    model.eval()
    return position_errors(model, data)[:, -1]


def main():
    cond, seed, dev = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    kw = dict(a.split("=") for a in sys.argv[4:])
    ks = [int(k) for k in kw.get("k", "1,2,5,10,20").split(",")]
    lrs = [float(v) for v in kw.get("lr", "3e-4,1e-4").split(",")]
    updates = int(kw.get("updates", "400"))
    torch.set_num_threads(1)
    path = OUT / f"e4_{cond}_s{seed}.pkl"
    ep = Episodes.load(f"{OUT}/{cond}.npz")
    assert max(ks) <= N_ADAPT, f"k must be <= {N_ADAPT} (held-out episodes follow)"
    adapt = windows(ep.subset(np.arange(N_ADAPT)), 8, 2, dev)
    test = windows(ep.subset(np.arange(N_ADAPT, len(ep.valid))), 30, 10, dev)
    nominal = windows(Episodes.load(f"{OUT}/test.npz"), 30, 10, dev)
    res = pickle.load(open(path, "rb")) if path.exists() else {}
    for arm, (stem, part) in ARMS.items():
        base = load_wm(f"{OUT}/{stem}_s{seed}.pt", dev)
        if (cond, arm, seed, 0.0, 0) not in res:
            res[cond, arm, seed, 0.0, 0] = {"mass": h30(base, test), "nominal": h30(base, nominal),
                                            "time": 0.0, "params": 0}  # fmt: skip
            save(res, path)
        for lr in lrs:
            for k in ks:
                key = (cond, arm, seed, lr, k)
                if key in res:
                    continue
                data = take(adapt, adapt["episode"] < k)
                m = copy.deepcopy(base).train()
                params = trainable(m, part, data)
                epochs = math.ceil(updates / math.ceil(len(data["x"]) / 256))
                if dev.startswith("cuda"):
                    torch.cuda.synchronize()
                t0 = time.time()
                fit(m, data, epochs, lr=lr, seed=seed, params=params)
                if dev.startswith("cuda"):
                    torch.cuda.synchronize()
                res[key] = {"mass": h30(m, test), "nominal": h30(m, nominal),
                            "time": time.time() - t0,
                            "params": sum(p.numel() for p in params)}  # fmt: skip
                r = res[key]
                print(f"{cond} {arm} s{seed} lr{lr:g} k{k}: h30 RMSE mass "
                      f"{100 * np.sqrt(np.nanmean(r['mass'] ** 2)):.1f} nominal "
                      f"{100 * np.sqrt(np.nanmean(r['nominal'] ** 2)):.1f} cm, {r['time']:.0f} s, "
                      f"{r['params']} params", flush=True)  # fmt: skip
                save(res, path)


if __name__ == "__main__":
    main()
