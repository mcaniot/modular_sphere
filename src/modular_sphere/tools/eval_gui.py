"""Visual evaluation of a world model: the sphere runs in MuJoCo while a translucent ghost shows the
model's open-loop prediction from the last real state, re-anchored every ``--horizon`` steps.

The error printed at each re-anchor is the predicted-vs-real position distance after ``horizon``
steps of the same actions (the open-loop error that matters for planning).

Usage:
    python eval_gui.py --model models/mlp_world_model.pt                   # reach task, viewer
    python eval_gui.py --level 2 --model models/mlp_world_model_course_l2.pt
    python eval_gui.py --model models/mlp_world_model.pt --record eval.mp4  # headless MP4
    python eval_gui.py --level 3                                            # policy only, no model
"""

from __future__ import annotations

import argparse
import time
from collections.abc import Callable, Iterator

import mujoco
import mujoco.viewer
import numpy as np

from modular_sphere.courses import LEVELS, SphereCourseEnv, WaypointPolicy
from modular_sphere.env import OBS_SLICES, SphereReachEnv

Policy = Callable[[np.ndarray], np.ndarray]
GHOST_RGBA = np.array([0.2, 0.6, 1.0, 0.4], dtype=np.float32)
SPHERE_RADIUS = 0.1
GOAL = OBS_SLICES["goal_rel"]


def make_env(level: int) -> SphereReachEnv:
    return SphereCourseEnv(level=level) if level else SphereReachEnv()


def make_policy(
    name: str, level: int, rng: np.random.Generator
) -> tuple[Policy, Callable[[], None]]:
    """Return (act, reset); the waypoint follower keeps its waypoint index across steps."""
    if name == "waypoint":
        waypoint = WaypointPolicy(level)
        return waypoint.act, waypoint.reset
    acts: dict[str, Policy] = {
        "pd": lambda o: np.clip(2.0 * o[GOAL] - o[OBS_SLICES["vel"]][:2], -1.0, 1.0),
        "random": lambda o: rng.uniform(-1.0, 1.0, size=2),
    }
    return acts[name], lambda: None


def load_model(path: str | None, obs_dim: int):
    if path is None:
        return None
    from modular_sphere.torch_world_model import MLPWorldModel

    model = MLPWorldModel.load(path)
    if model.net is not None and model.net[-1].out_features != obs_dim:
        dim = model.net[-1].out_features
        raise SystemExit(f"{path} predicts {dim}-D observations, env has {obs_dim}-D")
    return model


def episode(
    env: SphereReachEnv, act: Policy, model, steps: int, horizon: int, seed: int
) -> Iterator[tuple[int, np.ndarray | None, bool]]:
    """Step the env; yield (t, ghost observation or None, success) and print horizon errors."""
    obs, _ = env.reset(seed=seed)
    ghost = obs.copy() if model is not None else None
    errors: list[float] = []
    success = False
    for t in range(1, steps + 1):
        action = act(obs)
        if ghost is not None:
            ghost = model.rollout(ghost[None], action[None, None])[0, 0]
        obs, _, success, _, _ = env.step(action)
        if ghost is not None and (t % horizon == 0 or success):
            errors.append(float(np.linalg.norm(ghost[OBS_SLICES["pos"]] - obs[OBS_SLICES["pos"]])))
            print(f"  t={t:4d}  {horizon}-step position error {errors[-1] * 100:.1f} cm")
            ghost = obs.copy()
        yield t, ghost, success
        if success:
            break
    summary = f", mean error {np.mean(errors) * 100:.1f} cm" if errors else ""
    print(f"seed {seed}: {'success' if success else 'timeout'} at step {t}{summary}")


def draw_ghost(scene: mujoco.MjvScene, ghost: np.ndarray | None) -> None:
    if ghost is None or scene.ngeom >= scene.maxgeom:
        return
    mujoco.mjv_initGeom(
        scene.geoms[scene.ngeom],
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([SPHERE_RADIUS, 0.0, 0.0]),
        ghost[OBS_SLICES["pos"]],
        np.eye(3).ravel(),
        GHOST_RGBA,
    )
    scene.ngeom += 1


def run_viewer(env: SphereReachEnv, run: Callable[[int], Iterator], seed: int) -> None:
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        while viewer.is_running():
            for _, ghost, _ in run(seed):
                start = time.monotonic()
                with viewer.lock():
                    viewer.user_scn.ngeom = 0
                    draw_ghost(viewer.user_scn, ghost)
                viewer.sync()
                if not viewer.is_running():
                    return
                time.sleep(max(0.0, env.dt - (time.monotonic() - start)))
            seed += 1


def record(env: SphereReachEnv, run: Callable[[int], Iterator], seed: int, path: str) -> None:
    from modular_sphere.render import write_mp4

    frames = []
    with mujoco.Renderer(env.model, 600, 800) as renderer:
        for _, ghost, _ in run(seed):
            renderer.update_scene(env.data, "overview")
            draw_ghost(renderer.scene, ghost)
            frames.append(renderer.render())
    write_mp4(frames, path, fps=1.0 / env.dt)
    print(f"saved {len(frames)} frames to {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--level", type=int, default=0, choices=[0, *LEVELS], help="0 = reach, else course"
    )
    parser.add_argument("--model", help="trained MLPWorldModel (.pt); omit to run the policy only")
    parser.add_argument(
        "--policy",
        choices=["pd", "waypoint", "random"],
        help="default: waypoint on courses, pd on reach",
    )
    parser.add_argument("--steps", type=int, default=600, help="max steps per episode")
    parser.add_argument(
        "--horizon", type=int, default=20, help="open-loop prediction horizon [steps]"
    )
    parser.add_argument(
        "--seed", type=int, default=0, help="first episode seed (then +1 per episode)"
    )
    parser.add_argument("--record", metavar="MP4", help="render one episode headless to this file")
    args = parser.parse_args()

    env = make_env(args.level)
    act, reset_policy = make_policy(
        args.policy or ("waypoint" if args.level else "pd"),
        args.level,
        np.random.default_rng(args.seed),
    )
    model = load_model(args.model, env.observation_space.shape[0])

    def run(seed: int) -> Iterator:
        reset_policy()
        return episode(env, act, model, args.steps, args.horizon, seed)

    try:
        if args.record:
            record(env, run, args.seed, args.record)
        else:
            run_viewer(env, run, args.seed)
    finally:
        env.close()


if __name__ == "__main__":
    main()
