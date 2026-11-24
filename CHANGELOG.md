# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning 2.0.0](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.1.0] - 2026-11-24

### Added

- `swarm_modular`: swarm world models over sphere tokens in four kinds (`world`, `id`, `frame` in
  the frame of each sphere's skill call, `modular` with one net per skill), episodes labelled with
  the executed skill calls (`record_episode`, `Episodes`), open-loop unroll training (`fit`, also on
  a parameter subset for continual learning), `HeldNoise` exploration, a chunk verifier
  `WMVerifier` with `progress_cost`, and a MuJoCo oracle (`simulator_rollouts`,
  `simulator_scorer`). Experiments E1–E3 (`docs/paper/experiments/modular_e*.py`) and paper
  Section 5.
- Experiment E4 (online adaptation of the swarm world models to an unknown mass,
  `modular_e4.py`, `modular_e4.sh`, `modular_e4_report.py`), paper figures 1–5 from
  `docs/paper/experiments/modular_figures.py` in `docs/images/`, animated swarm GIFs (scripted
  climb, MOSAIC on an unseen random course), and illustrations in the README,
  the paper and `docs/architecture.html`.
- Working paper `docs/paper/skill_world_models.md` (methods, results with confidence intervals and
  paired tests, negative results, verified references) and its experiment scripts.
- `swarm_skills`: `skill_features`, a cloned skill policy `SkillPrior`, a learned steps-to-go
  `SkillValue`, and `SkillPlanner` options to sample around the prior and to use the value as cost.
- `swarm_wam`: per-skill world-action model `SkillWAM` (joint prediction of future skill frames
  and an action chunk), `WAMController` with optional temporal ensembling, and group-relative
  self-imitation helpers `rollout_skill`, `select_elite`.
- `swarm_plan`: skill scheduler that composes the skills in parallel lanes over sphere subsets,
  choosing roles and lanes by the makespan predicted with the steps-to-go effect models
  (`EffectModel`, `plan`, `SkillScheduler`), with scripted, cloned or WAM skill executors and
  `evaluate_schedule`. With a `labeler` and DART `noise`, the scheduler records every executed
  skill call (`Segment`) and `segments_to_data` turns them into skill data whose starts are the
  end states of the preceding skills (skill-chaining data, DAgger with learned executors).
- Courses of steps: `swarm.Course` (faces and rises), `random_course` (1 to 3 steps with random
  faces and rises) and the `course` option of `SphereSwarmStepEnv` (default: the single step, so
  default episodes are unchanged); `SwarmClimbPolicy` climbs a course level by level and waits in
  front of each next face. Skill calls carry their step frame (`SkillCall.face`, `floor`, `rise`,
  `back`), so skills learned on the training step run on any level. `SkillScheduler` plans and
  replans level by level (`Level`, `course_levels`), swaps crossing slot rolls, pulls a sphere
  that fell back off the platform up again, and `evaluate_schedule` takes per-seed courses.
  Climbers queued on a floor too short for the queue wait side by side (`QUEUE_SIDE`).
- Skill networks and `SkillWAM` floor their feature normalisation at `STD_FLOOR`, so nearly
  constant features no longer amplify sensor noise; `SkillWAM.fit(input_noise=...)` trains on
  noisy history frames with clean targets.
- `swarm_plan.perturb_dynamics` (domain randomisation of sphere masses and friction,
  `DYNAMICS_MASS`, `DYNAMICS_FRICTION`) and the `dynamics` option of `evaluate_schedule`.
- Paper section 5.9 (adaptability, robust MOSAIC) and its scripts `skills_adapt.py`,
  `skills_adapt_stats.py`; `skills_chain.py` options `input_noise`, `dynamics`, `rises`, `extra`.
- Tests for `swarm_rl`, `swarm_skills`, `swarm_wam`, `swarm_plan` and the `step_height` option.
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

### Fixed

- The scripted pull skill (`SkillController`) meters the lift by energy
  (`SwarmClimbPolicy._pump`, `_pull_step(shaped=True)`, Åström and Furuta energy swing-up): full
  lift launched the base and dragged a light helper off the edge (mass ×0.7: scripted skills
  23/50 → 50/50 on fresh courses). The expert `SwarmClimbPolicy` keeps the full-lift pull.
  Experiment scripts `skills_shaped.py`, `skills_chain.py pull=shaped`, `skills_adapt.py old=`;
  paper section 5.9.
- `swarm_plan.perturb_dynamics` recomputes the derived constants (`mj_setConst`) after scaling
  masses; `swarm.Course` rejects faces that leave no room for the start queue or lie past the
  last allowed face; start sampling raises `RuntimeError` instead of looping when the floor is
  too small for the swarm; `SkillPlanner` keeps one sampled plan with `prior_frac=1`;
  `improve_with_ppo(anchor=...)` keeps the KL and the normalisation statistics on the imitation
  policy across `train_swarm` chunks.
- `skills_adapt.py` recomputes the derived constants (`mj_setConst`) after scaling masses; paper
  section 5.9 reports the rerun mass conditions (mass ×0.7: robust MOSAIC 4/50, scripted skills
  23/50). Experiment scripts resolve `data/swarm` relative to the repository instead of an
  absolute path; `skills_adapt_stats.py` no longer crashes without tie-free pairs; `stage10.py`
  runs on Python 3.11.
- Swarm learning: `SwarmWorldModel.fit` raises `ValueError` on episodes shorter than its window,
  `train_swarm` on `select_episodes` or `eval_episodes` below 1 and `swarm_rl.evaluate` on empty
  seeds; `SwarmVecEnv.close` joins its workers.

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

[Unreleased]: https://github.com/mcaniot/modular_sphere/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/mcaniot/modular_sphere/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/mcaniot/modular_sphere/compare/0.0.2...v1.0.0
[0.0.2]: https://github.com/mcaniot/modular_sphere/compare/0.0.1...0.0.2
[0.0.1]: https://github.com/mcaniot/modular_sphere/releases/tag/0.0.1
