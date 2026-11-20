"""Regenerate the README illustrations in ``docs/images`` (classic renderer, deterministic seeds).

Usage: ``MUJOCO_GL=egl uv run python docs/make_figures.py``
"""

from __future__ import annotations

from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

from modular_sphere.courses import LEVELS, SphereCourseEnv, WaypointPolicy
from modular_sphere.env import SphereReachEnv

OUT = Path(__file__).parent / "images"


def snapshot(env: SphereReachEnv, camera: str, width: int, height: int, seed: int) -> np.ndarray:
    env.reset(seed=seed)
    with mujoco.Renderer(env.model, height, width) as renderer:
        renderer.update_scene(env.data, camera)
        return renderer.render()


def course_gif(level: int, seed: int, path: Path, every: int = 4) -> None:
    """Scripted waypoint run on a course, top camera, one frame every ``every`` control steps."""
    env, policy = SphereCourseEnv(level=level), WaypointPolicy(level)
    obs, _ = env.reset(seed=seed)
    policy.reset()
    frames = []
    with mujoco.Renderer(env.model, 256, 256) as renderer:
        for t in range(LEVELS[level].max_steps):
            obs, _, success, _, _ = env.step(policy.act(obs))
            if t % every == 0 or success:
                renderer.update_scene(env.data, "top")
                frames.append(renderer.render())
            if success:
                break
    env.close()
    imageio.mimwrite(path, frames, duration=1000 * every * env.dt, loop=0)


def main() -> None:
    OUT.mkdir(exist_ok=True)
    env = SphereReachEnv()
    imageio.imwrite(OUT / "reach_overview.png", snapshot(env, "overview", 960, 540, seed=0))
    env.close()

    tiles = []
    for level in LEVELS:
        env = SphereCourseEnv(level=level)
        tiles.append(snapshot(env, "top", 320, 320, seed=0))
        env.close()
    imageio.imwrite(OUT / "courses_top.png", np.concatenate(tiles, axis=1))

    course_gif(5, seed=0, path=OUT / "course_l5.gif")


if __name__ == "__main__":
    main()
