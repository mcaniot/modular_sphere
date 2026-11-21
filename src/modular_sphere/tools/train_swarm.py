"""Learn the swarm step climb from the scripted expert: data, world model, DAgger, PPO in the model.

Pipeline (each stage saved under ``--out-dir``; a JSON report summarises them):

1. ``collect``: scripted ``SwarmClimbPolicy`` with DART noise -> ``train.npz``, ``val.npz``.
2. Behaviour cloning, then DAgger rounds (learner drives, expert relabels) -> ``dagger.pt``.
3. Exploration episodes with held random actions, then an ensemble of world models
   (interaction networks, one seed each) on all real transitions -> ``wm{k}.pt``; open-loop RMSE
   of the first member vs constant velocity.
4. PPO in the ensemble's rollouts (team reward with a time penalty for speed, KL to the DAgger
   policy, disagreement penalty), in chunks; the chunk with the best MuJoCo ``speed_vs_expert``
   on separate selection seeds is kept -> ``ppo.pt``. (Backpropagating through the model with
   ``improve_in_imagination`` exploited it: 0 % real success.)
5. Closed-loop MuJoCo evaluation of every policy on the same held-out seeds: success, then speed
   (steps against the scripted expert on the same seed); PPO vs DAgger with an exact McNemar
   test on the discordant seeds.

Example: ``uv run python -m modular_sphere.tools.train_swarm --spheres 4 --out-dir data/swarm``.
"""

from __future__ import annotations

import argparse
import json
from math import comb
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from modular_sphere.swarm_data import SwarmDataset, collect, expert_steps
from modular_sphere.swarm_torch import (
    PolicyEval,
    SwarmPolicy,
    SwarmWorldModel,
    evaluate_policy,
    evaluate_world_model,
    improve_with_ppo,
)

EVAL_SEED = 200_000
VAL_SEED = 100_000
DAGGER_SEED = 10_000
TRAIN_NOISE = (0.1, 0.2, 0.3, 0.2)  # DART std per quarter of the training episodes
EXPLORE_SEED = 30_000
EXPLORE_PROB = (0.15, 0.3, 0.5)  # held random-action probability per third of exploration
SELECT_SEED = 300_000  # MuJoCo seeds that pick the best PPO chunk, disjoint from evaluation


def mcnemar(a: NDArray[np.bool_], b: NDArray[np.bool_]) -> dict[str, float]:
    """Exact two-sided McNemar test on paired successes; ``only_b`` counts b-only successes."""
    only_b, only_a = int((~a & b).sum()), int((a & ~b).sum())
    n = only_a + only_b
    tail = sum(comb(n, i) for i in range(min(only_a, only_b) + 1))
    p = 1.0 if n == 0 else min(1.0, 2 * tail / 2**n)
    return {"p_value": p, "only_a": only_a, "only_b": only_b}


def _summary(ev: PolicyEval, reference: NDArray[np.int64]) -> dict[str, float]:
    """Success, then the speed objective: ``speed_vs_expert`` averages expert steps / policy steps
    on successes and 0 on failures (success weighted by speed, as SPL weights by path length)."""
    s = ev.success
    return {
        "success_rate": float(s.mean()),
        "speed_vs_expert": float(np.where(s, reference / ev.steps, 0.0).mean()),
        "mean_steps_on_success": float(ev.steps[s].mean()) if s.any() else float("nan"),
        "mean_arrived": float(ev.n_arrived.mean()),
    }


def train_swarm(
    out_dir: Path,
    n_spheres: int = 4,
    episodes: int = 400,
    val_episodes: int = 50,
    wm_epochs: int = 20,
    bc_epochs: int = 30,
    dagger_betas: tuple[float, ...] = (0.5, 0.0, 0.0),
    dagger_episodes: int = 100,
    explore_episodes: int = 300,
    ppo_iterations: int = 400,
    ppo_chunk: int = 50,
    kl_weight: float = 1.0,
    time_penalty: float = 0.05,
    select_episodes: int = 20,
    ensemble: int = 3,
    uncertainty_weight: float = 100.0,
    eval_episodes: int = 50,
    hidden: int | None = None,
    device: str | None = None,
) -> dict[str, object]:
    out_dir.mkdir(parents=True, exist_ok=True)
    size = {} if hidden is None else {"hidden": hidden}
    quarter = max(1, episodes // len(TRAIN_NOISE))
    train = SwarmDataset.concat(
        [
            collect(quarter, n_spheres, seed=k * quarter, noise=nz)
            for k, nz in enumerate(TRAIN_NOISE)
        ]
    )
    val = collect(val_episodes, n_spheres, seed=VAL_SEED, noise=0.2)
    train.save(out_dir / "train.npz")
    val.save(out_dir / "val.npz")
    report: dict[str, object] = {
        "n_spheres": n_spheres,
        "data": {
            "transitions": int(train.valid.sum()),
            "expert_success": float(train.success.mean()),
        },
    }

    seeds = list(range(EVAL_SEED, EVAL_SEED + eval_episodes))
    evals: dict[str, PolicyEval] = {}
    policy = SwarmPolicy(n_spheres, epochs=bc_epochs, device=device, **size)
    policy.fit(train)
    policy.save(str(out_dir / "bc.pt"))
    evals["bc"] = evaluate_policy(policy, n_spheres, seeds)
    data = train
    for rnd, beta in enumerate(dagger_betas):
        seed = DAGGER_SEED + 1000 * rnd
        new = collect(dagger_episodes, n_spheres, seed=seed, noise=0.2, policy=policy, beta=beta)
        data = SwarmDataset.concat([data, new])
        policy = SwarmPolicy(n_spheres, epochs=bc_epochs, device=device, **size)
        policy.fit(data)
    if dagger_betas:
        policy.save(str(out_dir / "dagger.pt"))
        evals["dagger"] = evaluate_policy(policy, n_spheres, seeds)

    # World models on every real transition: expert, DAgger and held random exploration.
    third = max(1, explore_episodes // len(EXPLORE_PROB))
    explore = [
        collect(third, n_spheres, seed=EXPLORE_SEED + k * third, noise=0.2, random_prob=prob)
        for k, prob in enumerate(EXPLORE_PROB)
    ]
    wm_data = SwarmDataset.concat([data, *explore])
    models: list[SwarmWorldModel] = []
    for k in range(ensemble):
        wm = SwarmWorldModel(epochs=wm_epochs, seed=k, device=device, **size)
        wm.fit(wm_data)
        wm.save(str(out_dir / f"wm{k}.pt"))
        models.append(wm)
    horizons = [h for h in (1, 10, 30) if h < val.valid.shape[1]]
    report["world_model"] = {
        str(h): v for h, v in evaluate_world_model(models[0], val, horizons).items()
    }

    base = "dagger" if dagger_betas else "bc"
    select = list(range(SELECT_SEED, SELECT_SEED + select_episodes))
    select_ref = expert_steps(n_spheres, select)
    best = _summary(evaluate_policy(policy, n_spheres, select), select_ref)["speed_vs_expert"]
    policy.save(str(out_dir / "ppo.pt"))
    ppo_log: list[dict[str, float]] = []
    for chunk in range(0, ppo_iterations, ppo_chunk):
        log = improve_with_ppo(
            policy,
            models,
            data,
            min(ppo_chunk, ppo_iterations - chunk),
            kl_weight=kl_weight,
            time_penalty=time_penalty,
            uncertainty_weight=uncertainty_weight,
            seed=chunk,
        )
        speed = _summary(evaluate_policy(policy, n_spheres, select), select_ref)["speed_vs_expert"]
        ppo_log.append({**log[-1], "iteration": chunk + len(log), "select_speed": speed})
        if speed >= best:  # ties keep the later, more trained chunk
            best = speed
            policy.save(str(out_dir / "ppo.pt"))
    policy = SwarmPolicy.load(str(out_dir / "ppo.pt"), device)
    evals["ppo"] = evaluate_policy(policy, n_spheres, seeds)
    report["ppo_log"] = ppo_log
    reference = expert_steps(n_spheres, seeds)
    report["expert_mean_steps"] = float(reference.mean())
    report["policies"] = {name: _summary(ev, reference) for name, ev in evals.items()}
    report["eval_seeds"] = [seeds[0], seeds[-1]]
    report[f"mcnemar_{base}_vs_ppo"] = mcnemar(evals[base].success, evals["ppo"].success)
    (out_dir / "report.json").write_text(json.dumps(report, indent=2))
    return report


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--out-dir", type=Path, default=Path("data/swarm"))
    parser.add_argument("--spheres", type=int, default=4)
    parser.add_argument("--episodes", type=int, default=400)
    parser.add_argument("--val-episodes", type=int, default=50)
    parser.add_argument("--wm-epochs", type=int, default=20)
    parser.add_argument("--bc-epochs", type=int, default=30)
    parser.add_argument("--dagger-betas", type=float, nargs="*", default=[0.5, 0.0, 0.0])
    parser.add_argument("--dagger-episodes", type=int, default=100)
    parser.add_argument("--explore-episodes", type=int, default=300)
    parser.add_argument("--ppo-iterations", type=int, default=400)
    parser.add_argument("--ppo-chunk", type=int, default=50)
    parser.add_argument("--kl-weight", type=float, default=1.0)
    parser.add_argument("--time-penalty", type=float, default=0.05)
    parser.add_argument("--select-episodes", type=int, default=20)
    parser.add_argument("--ensemble", type=int, default=3)
    parser.add_argument("--uncertainty-weight", type=float, default=100.0)
    parser.add_argument("--eval-episodes", type=int, default=50)
    parser.add_argument("--hidden", type=int, default=None)
    parser.add_argument("--device", default=None)
    args = parser.parse_args()
    report = train_swarm(
        args.out_dir,
        n_spheres=args.spheres,
        episodes=args.episodes,
        val_episodes=args.val_episodes,
        wm_epochs=args.wm_epochs,
        bc_epochs=args.bc_epochs,
        dagger_betas=tuple(args.dagger_betas),
        dagger_episodes=args.dagger_episodes,
        explore_episodes=args.explore_episodes,
        ppo_iterations=args.ppo_iterations,
        ppo_chunk=args.ppo_chunk,
        kl_weight=args.kl_weight,
        time_penalty=args.time_penalty,
        select_episodes=args.select_episodes,
        ensemble=args.ensemble,
        uncertainty_weight=args.uncertainty_weight,
        eval_episodes=args.eval_episodes,
        hidden=args.hidden,
        device=args.device,
    )
    print(json.dumps({"policies": report["policies"]}, indent=2))


if __name__ == "__main__":
    main()
