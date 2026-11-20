import contextlib
import sys

import imageio
import mujoco
import mujoco.viewer
import numpy as np
import pytest

from modular_sphere.cli import _build_model
from modular_sphere.courses import OBS_DIM
from modular_sphere.dataset import TrajectoryDataset
from modular_sphere.tools import collect_courses, eval_gui
from modular_sphere.tools.collect_courses import collect_course_data
from modular_sphere.world_model import LinearDeltaModel


def test_collect_course_data_shapes():
    dataset = collect_course_data(level=1, num_episodes=2, steps=5, seed=0)
    assert dataset.num_episodes == 2 and dataset.num_steps == 5
    assert dataset.obs.shape == (2, 6, OBS_DIM)
    assert dataset.metadata["env"] == "ModularSphere/Course-L1-v0"


def test_collect_courses_cli(tmp_path, monkeypatch):
    argv = ["collect_courses", "--level", "2", "--episodes", "1", "--steps", "4"]
    monkeypatch.setattr(sys, "argv", [*argv, "--out-dir", str(tmp_path)])
    collect_courses.main()
    assert TrajectoryDataset.load(tmp_path / "course_l2.npz").num_steps == 4


@pytest.mark.parametrize("args", [[], ["--level", "0"]])
def test_collect_courses_cli_rejects_missing_or_bad_level(args, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["collect_courses", *args])
    with pytest.raises(SystemExit):
        collect_courses.main()


@pytest.mark.parametrize("spec", ["os:getcwd", "modular_sphere_evil:f"])
def test_build_model_rejects_foreign_modules(spec):
    with pytest.raises(SystemExit, match="not allowed"):
        _build_model(spec, 0.05)


def test_build_model_accepts_package_factory():
    model = _build_model("modular_sphere.world_model:LinearDeltaModel", 0.05)
    assert isinstance(model, LinearDeltaModel)


def test_eval_gui_waypoint_solves_course_with_ghost(capsys):
    """Scripted pilot reaches the goal; the ghost (perfect model) has zero re-anchor error."""
    env = eval_gui.make_env(1)
    act, reset = eval_gui.make_policy("waypoint", 1, np.random.default_rng(0))
    reset()

    class Oracle:  # replays the env dynamics from the ghost state
        def rollout(self, obs0, actions):
            env_copy.set_from_obs(obs0[0])
            env_copy.step(actions[0, 0])
            return env_copy._get_obs()[None, None]

    env_copy = eval_gui.make_env(1)
    env_copy.reset(seed=0)
    steps = list(eval_gui.episode(env, act, Oracle(), steps=600, horizon=20, seed=0))
    env.close()
    env_copy.close()
    assert steps[-1][2], "waypoint pilot should reach the goal"
    assert "success" in capsys.readouterr().out


def test_eval_gui_record(tmp_path, monkeypatch):
    out = tmp_path / "eval.mp4"
    monkeypatch.setattr(sys, "argv", ["eval_gui", "--steps", "5", "--record", str(out)])
    eval_gui.main()
    assert len(imageio.mimread(out)) == 5


def test_eval_gui_load_model_checks_obs_dim(tmp_path):
    torch_wm = pytest.importorskip("modular_sphere.torch_world_model")
    model = torch_wm.MLPWorldModel(hidden=8, epochs=1, device="cpu")
    model.fit(collect_course_data(level=1, num_episodes=1, steps=5, seed=0))
    model.save(tmp_path / "m.pt")
    assert eval_gui.load_model(str(tmp_path / "m.pt"), obs_dim=OBS_DIM).net is not None
    with pytest.raises(SystemExit, match="13-D observations, env has 11-D"):
        eval_gui.load_model(str(tmp_path / "m.pt"), obs_dim=11)


def test_collect_courses_all_levels(tmp_path, monkeypatch):
    argv = ["collect_courses", "--all-levels", "--episodes", "1", "--steps", "2"]
    monkeypatch.setattr(sys, "argv", [*argv, "--out-dir", str(tmp_path)])
    collect_courses.main()
    assert sorted(p.name for p in tmp_path.iterdir()) == [f"course_l{i}.npz" for i in range(1, 6)]


class _FakeViewer:
    """Passive viewer stand-in: real scene for the ghost geom, closes after ``ticks`` syncs."""

    def __init__(self, model, ticks):
        self.user_scn, self.ticks, self.ghosts = mujoco.MjvScene(model, maxgeom=1), ticks, 0

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def is_running(self):
        return self.ticks > 0

    def lock(self):
        return contextlib.nullcontext()

    def sync(self):
        self.ticks -= 1
        self.ghosts += self.user_scn.ngeom
        eval_gui.draw_ghost(self.user_scn, np.zeros(OBS_DIM))  # scene full: no-op


def test_eval_gui_viewer_draws_ghost(monkeypatch):
    viewers = []

    def launch(model, data):
        viewers.append(_FakeViewer(model, ticks=5))
        return viewers[-1]

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch)
    monkeypatch.setattr(eval_gui.time, "sleep", lambda _: None)
    monkeypatch.setattr(sys, "argv", ["eval_gui", "--level", "1", "--steps", "3"])
    monkeypatch.setattr(eval_gui, "load_model", lambda path, dim: _GhostModel())
    eval_gui.main()
    assert viewers[0].ticks == 0 and viewers[0].ghosts == 5


def test_eval_gui_viewer_closed_between_episodes(monkeypatch):
    """Window closed after an episode ends: the outer loop stops without starting a new one."""
    env = eval_gui.make_env(0)
    viewer = _FakeViewer(env.model, ticks=0)
    running = iter([True, True, False])  # outer loop, after the only frame, outer loop again
    monkeypatch.setattr(viewer, "is_running", lambda: next(running))
    monkeypatch.setattr(mujoco.viewer, "launch_passive", lambda model, data: viewer)
    monkeypatch.setattr(eval_gui.time, "sleep", lambda _: None)
    seeds = []

    def run(seed):
        seeds.append(seed)
        yield 1, None, True

    eval_gui.run_viewer(env, run, seed=7)
    env.close()
    assert seeds == [7]


class _GhostModel:
    def rollout(self, obs0, actions):
        return obs0[:, None]
