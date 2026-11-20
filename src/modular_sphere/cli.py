"""Command line: ``modular-sphere {teleop,record,collect,eval-wm}``."""

from __future__ import annotations

import argparse
import importlib
import json
from typing import Any

from modular_sphere import __version__
from modular_sphere.dataset import TrajectoryDataset, collect
from modular_sphere.world_model import (
    ConstantVelocityModel,
    LinearDeltaModel,
    MuJoCoOracleModel,
    WorldModel,
    evaluate,
)


def _mlp() -> Any:
    try:
        from modular_sphere.torch_world_model import MLPWorldModel
    except ImportError as err:
        raise SystemExit("model 'mlp' needs PyTorch: pip install 'modular-sphere[torch]'") from err
    return MLPWorldModel()


def _build_model(spec: str, dt: float) -> Any:
    builtin = {
        "mlp": _mlp,
        "constant-velocity": lambda: ConstantVelocityModel(dt),
        "linear": LinearDeltaModel,
        "oracle": MuJoCoOracleModel,
    }
    if spec in builtin:
        return builtin[spec]()
    module, _, attr = spec.partition(":")  # user model: "package.module:factory"
    if not attr:
        models = ", ".join(repr(m) for m in sorted(builtin))
        raise SystemExit(f"unknown model {spec!r}: use one of {models} or 'package.module:factory'")
    # Security: restrict imports to modular_sphere module only
    if module != "modular_sphere" and not module.startswith("modular_sphere."):
        raise SystemExit(
            f"module {module!r} is not allowed; only 'modular_sphere' submodules are permitted"
        )
    return getattr(importlib.import_module(module), attr)()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="modular-sphere")
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("teleop", help="drive the sphere with the arrow keys in the MuJoCo viewer")
    p.add_argument("--seed", type=int, default=0)

    p = sub.add_parser("collect", help="record exploration trajectories to a .npz dataset")
    p.add_argument("--out", required=True)
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--steps", type=int, default=200)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--goal-seeking-ratio", type=float, default=0.5)

    p = sub.add_parser("record", help="render a rollout to MP4 (classic or Filament renderer)")
    p.add_argument("--out", required=True)
    p.add_argument("--renderer", choices=["classic", "filament"], default="classic")
    p.add_argument("--camera", choices=["overview", "top", "track"], default="overview")
    p.add_argument("--policy", choices=["goal", "random"], default="goal")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--steps", type=int, default=200)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)

    p = sub.add_parser("eval-wm", help="open-loop evaluation of a world model on a dataset")
    p.add_argument(
        "--model",
        required=True,
        help="constant-velocity | linear | mlp | oracle | package.module:factory",
    )
    p.add_argument("--eval-data", required=True)
    p.add_argument("--train-data", help="dataset passed to model.fit() when the model has one")
    p.add_argument("--horizons", type=int, nargs="+", default=[1, 5, 10, 20, 50])
    p.add_argument("--stride", type=int, default=10)

    args = parser.parse_args(argv)
    if args.command == "teleop":
        from modular_sphere.teleop import main as teleop

        teleop(args.seed)
    elif args.command == "record":
        from modular_sphere.render import record

        n = record(args.out, args.renderer, args.camera, args.policy, args.seed, args.steps,
                   args.width, args.height)  # fmt: skip
        print(f"saved {n} frames to {args.out}")
    elif args.command == "collect":
        dataset = collect(args.episodes, args.steps, args.seed, args.goal_seeking_ratio)
        dataset.save(args.out)
        print(f"saved {dataset.num_episodes} x {dataset.num_steps} steps to {args.out}")
    else:
        eval_data = TrajectoryDataset.load(args.eval_data)
        model = _build_model(args.model, eval_data.metadata["dt"])
        if not isinstance(model, WorldModel):
            raise SystemExit(f"{args.model} does not implement rollout(obs0, actions)")
        if hasattr(model, "fit"):
            if args.train_data is None:
                raise SystemExit(f"{args.model} needs --train-data")
            model.fit(TrajectoryDataset.load(args.train_data))
        print(json.dumps(evaluate(model, eval_data, args.horizons, args.stride), indent=2))


if __name__ == "__main__":
    main()
