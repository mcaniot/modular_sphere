"""Collect data from obstacle courses for world model training."""

import argparse
from pathlib import Path

import numpy as np

from modular_sphere.courses import LEVELS, OBS_DIM, SphereCourseEnv
from modular_sphere.dataset import TrajectoryDataset, exploration_action
from modular_sphere.env import ACT_DIM, SUCCESS_DIST


def collect_course_data(
    level: int,
    num_episodes: int,
    steps: int,
    seed: int,
    goal_seeking_ratio: float = 0.5,
) -> TrajectoryDataset:
    """Collect data from a specific course level."""
    rng = np.random.default_rng(seed)

    # Get the max steps for this level
    max_steps = LEVELS[level].max_steps
    steps = min(steps, max_steps)

    env = SphereCourseEnv(level=level)

    obs_buf = np.empty((num_episodes, steps + 1, OBS_DIM))
    act_buf = np.empty((num_episodes, steps, ACT_DIM))
    rew_buf = np.empty((num_episodes, steps))
    suc_buf = np.empty((num_episodes, steps), dtype=bool)

    for ep in range(num_episodes):
        obs, _ = env.reset(seed=seed + ep)
        goal_seeking = bool(rng.random() < goal_seeking_ratio)
        noise = np.zeros(ACT_DIM)
        obs_buf[ep, 0] = obs

        for t in range(steps):
            action = exploration_action(rng, obs, goal_seeking, noise)
            obs, reward, success, _, _ = env.step(action)
            obs_buf[ep, t + 1], act_buf[ep, t] = obs, action
            rew_buf[ep, t], suc_buf[ep, t] = reward, success

    env.close()

    metadata = {
        "env": f"ModularSphere/Course-L{level}-v0",
        "dt": env.dt,
        "seed": seed,
        "steps": steps,
        "goal_seeking_ratio": goal_seeking_ratio,
        "success_dist": SUCCESS_DIST,
        "level": level,
    }

    return TrajectoryDataset(obs_buf, act_buf, rew_buf, suc_buf, metadata)


def main():
    parser = argparse.ArgumentParser(description="Collect course data for world model training")
    parser.add_argument(
        "--level", type=int, choices=list(LEVELS), help="Specific level to collect (1-5)"
    )
    parser.add_argument("--all-levels", action="store_true", help="Collect all levels 1-5")
    parser.add_argument("--episodes", type=int, default=50, help="Number of episodes per level")
    parser.add_argument(
        "--steps", type=int, default=None, help="Steps per episode (uses level default if None)"
    )
    parser.add_argument("--seed", type=int, default=0, help="Base seed")
    parser.add_argument("--out-dir", type=str, default="data/courses", help="Output directory")
    parser.add_argument(
        "--goal-seeking-ratio",
        type=float,
        default=0.5,
        help="Ratio of goal-seeking episodes",
    )

    args = parser.parse_args()

    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    if args.level is not None:
        levels = [args.level]
    elif args.all_levels:
        levels = list(LEVELS.keys())
    else:
        parser.error("Please specify either --level or --all-levels")

    for level in sorted(levels):
        steps = args.steps if args.steps else LEVELS[level].max_steps
        dataset = collect_course_data(
            level=level,
            num_episodes=args.episodes,
            steps=steps,
            seed=args.seed + level * 1000,
            goal_seeking_ratio=args.goal_seeking_ratio,
        )
        out_path = Path(args.out_dir) / f"course_l{level}.npz"
        dataset.save(out_path)
        print(
            f"Level {level}: saved {dataset.num_episodes} x {dataset.num_steps} steps to {out_path}"
        )


if __name__ == "__main__":
    main()
