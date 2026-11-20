# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning 2.0.0](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.0] - 2026-11-20

Complete rewrite on MuJoCo. Breaking: the PyBullet API, the `stable_baselines` examples and the
pretrained PPO2 model are removed (see "Migration" in the README).

### Added

- `ModularSphere/Reach-v0`: deterministic Gymnasium environment (2 ms physics, 20 Hz control,
  11-D Markov observation, world-frame force action).
- `ModularSphere/Course-L1..L5-v0`: obstacle courses (wall, tunnel, bridge, plate and door)
  built with `mujoco.MjSpec`, with a scripted `WaypointPolicy` that solves every level.
- World-model bench: `WorldModel` protocol, `oracle`, `linear`, `constant-velocity` and PyTorch
  `mlp` baselines, multi-horizon open-loop evaluation, `.npz` trajectory datasets.
- `modular-sphere` CLI (`teleop`, `collect`, `record`, `eval-wm`, `--version`), classic and
  Filament renderers, MP4 export.
- Tools: `modular_sphere.tools.collect_courses` and `modular_sphere.tools.eval_gui` (ghost
  prediction in the MuJoCo viewer).
- pytest suite (100 % line and branch coverage), GitHub Actions CI on Python 3.11 and 3.12 with Codecov,
  release workflow, Dependabot, tool-agnostic `AGENTS.md` for coding agents, English
  documentation.

### Removed

- PyBullet/qibullet simulation, `setup.py`, Travis CI, TensorFlow 1 PPO2 examples.

## [0.0.2] - 2020-01-08

- Last PyBullet version: teleoperation, PPO2 training and replay examples.

## [0.0.1] - 2019-07-08

- First PyBullet prototype of the modular sphere.

[Unreleased]: https://github.com/mcaniot/modular_sphere/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/mcaniot/modular_sphere/compare/0.0.2...v1.0.0
[0.0.2]: https://github.com/mcaniot/modular_sphere/compare/0.0.1...0.0.2
[0.0.1]: https://github.com/mcaniot/modular_sphere/releases/tag/0.0.1
