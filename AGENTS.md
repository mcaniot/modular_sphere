# AGENTS.md

MuJoCo simulation of a spherical robot and a world-model test bench (Gymnasium). Human docs:
`README.md` (usage, simulation contract), `docs/architecture.html` (design). Keep this file
for what an agent needs and cannot guess.

## Layout

- `src/modular_sphere/env.py`: `SphereReachEnv`, `OBS_SLICES`, `OBS_DIM = 11`, `ACT_DIM = 2`.
- `src/modular_sphere/courses.py`: `SphereCourseEnv` levels 1-5 (`LEVELS`, 13-D obs), `WaypointPolicy`.
- `src/modular_sphere/swarm.py`: `SphereSwarmStepEnv` (PettingZoo `ParallelEnv`, N spheres,
  links + lift, whole swarm climbs a step and gathers), `obs_slices(n)`, `gather_radius(n)`,
  `SwarmClimbPolicy`.
- `src/modular_sphere/swarm_data.py`: `SwarmDataset`, `collect` (expert + DART noise, DAgger);
  `swarm_torch.py`: `SwarmWorldModel`, `SwarmPolicy`, `improve_with_ppo`, evaluations.
- `src/modular_sphere/assets/sphere_arena.xml`: the only MJCF; courses and swarm edit it with `mujoco.MjSpec`.
- `src/modular_sphere/world_model.py`: `WorldModel` protocol, baselines, `evaluate`.
- `src/modular_sphere/torch_world_model.py`: optional PyTorch `MLPWorldModel` (extra `torch`).
- `src/modular_sphere/dataset.py`: `TrajectoryDataset` (`.npz`), `collect`, `exploration_action`.
- `src/modular_sphere/render.py`: classic and Filament renderers, `write_mp4`.
- `src/modular_sphere/cli.py`: `modular-sphere` entry point; `tools/`: `collect_courses`, `eval_gui`,
  `train_swarm` (expert data -> world model -> BC/DAgger -> PPO in the model -> paired report).
- `tests/`: one file per module plus `test_error_paths.py`; `docs/make_figures.py` regenerates `docs/images/`.
- Ignore for search: `.venv/`, `data/`, `models/`, `videos/`, `htmlcov/`, `uv.lock`.

## Commands

```bash
uv sync --extra dev --extra torch                      # environment (Python >= 3.11)
uv run ruff format src tests && uv run ruff check src tests
MUJOCO_GL=egl uv run pytest -q                         # osmesa on CPU-only machines
uv run pytest -q tests/test_env.py::test_gymnasium_api # single test
uv run pytest --cov                                    # coverage (CI uploads to Codecov)
```

Filament tests need Python >= 3.12 and an X display (`xvfb-run -a` headless); they skip otherwise.
CI (`.github/workflows/ci.yml`) runs ruff check, ruff format --check and pytest on 3.11 and 3.12.

## Contracts (change only deliberately, with tests and a CHANGELOG entry)

- Physics timestep 2 ms, `frame_skip=25` (20 Hz control), reach episodes 200 steps.
- Action: world-frame x, y force in N, clipped to [-1, 1].
- Observation order: `pos`(3) `vel`(3) `ang_vel`(3) `goal_rel`(2), world frame; courses append
  `button`, `door`. Datasets and saved models depend on this order.
- Swarm: action `[fx, fy, lift, attach]` per agent; lift (6 N max) is internal to linked spheres;
  links need both requests and contact; local observation layout from `swarm.obs_slices(n)`.
- Environments are deterministic for a given seed; `MuJoCoOracleModel` must stay exact (< 1e-6).
- `--model package.module:factory` may only import `modular_sphere.*` (security allowlist).

## Task workflow (project rule)

Every new task is first solved by a scripted approach, then that approach must be learned with a
world model and action learning: expert data with DART noise, a state world model, imitation with
DAgger, then improvement in the model (PPO on model rollouts with a KL to the imitation policy;
pattern: `swarm_data`, `swarm_torch`, `tools/train_swarm.py`). Compare learned policies on the
same MuJoCo seeds (exact McNemar on success, speed against the expert).

## Code style

- Ruff (line length 100, rules in `pyproject.toml`); type hints and `from __future__ import annotations`.
- Numpy in, numpy out at module boundaries; no new dependencies without need.
- English for code, comments, docs and commit messages.

## Versioning and commits

- Semantic Versioning; the version lives only in `pyproject.toml` (`__version__` reads package
  metadata). Bump with `uv version --bump {major,minor,patch}`.
- Every user-visible change gets a line under `## [Unreleased]` in `CHANGELOG.md` (Keep a Changelog).
- Commits follow Conventional Commits: `feat:`, `fix:`, `docs:`, `test:`, `ci:`, `refactor:`,
  `chore:`; `!` or a `BREAKING CHANGE:` footer for contract changes.
- Release: move `Unreleased` to the new version, bump, commit `chore(release): vX.Y.Z`, push an
  annotated tag `vX.Y.Z`; `.github/workflows/release.yml` builds and publishes the GitHub release.
- `main` is the default branch; work on short-lived branches and open pull requests.
