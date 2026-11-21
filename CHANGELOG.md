# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning 2.0.0](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Working paper `docs/paper/skill_world_models.md` (methods, results with confidence intervals and
  paired tests, negative results, verified references) and its experiment scripts.
- `swarm_skills`: `skill_features`, a cloned skill policy `SkillPrior`, a learned steps-to-go
  `SkillValue`, and `SkillPlanner` options to sample around the prior and to use the value as cost.
- `swarm_wam`: per-skill world-action model `SkillWAM` (joint prediction of future skill frames
  and an action chunk), `WAMController` with optional temporal ensembling, and group-relative
  self-imitation helpers `rollout_skill`, `select_elite`.
- Tests for `swarm_rl`, `swarm_skills`, `swarm_wam` and the `step_height` option.
- `modular_sphere.swarm.SphereSwarmStepEnv`: first 3D multi-agent task (PettingZoo
  `ParallelEnv`). The whole swarm of N spheres must climb a step higher than the sphere radius
  and gather on the platform, using distance-preserving links between spheres and an internal
  lift force; scripted `SwarmClimbPolicy` solves it. New dependency: `pettingzoo>=1.25`.
- Swarm learning: `swarm_data` (expert datasets with DART noise and DAgger relabelling),
  `swarm_torch` (interaction-network world model, decentralised Deep Sets policy, actor-critic
  fine-tuning in imagination) and the `tools.train_swarm` pipeline with a paired McNemar report.
  World models use wall and step distance features, an ensemble disagreement penalty and
  exploration data with held random actions; the report measures speed against the expert.
- `swarm_torch.improve_with_ppo`: PPO in the world-model ensemble with a time penalty and a KL
  term to the DAgger policy; `train_swarm` now uses it (`--ppo-iterations`, `--kl-weight`,
  `--time-penalty`, best chunk on selection seeds) and writes `ppo.pt` instead of
  `imagination.pt`.

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
