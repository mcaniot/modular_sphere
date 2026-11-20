import mujoco.viewer

from modular_sphere import teleop
from modular_sphere.env import SphereReachEnv


class _FakeViewer:
    """Presses the right arrow once, then runs ``ticks`` control steps without a window."""

    def __init__(self, model, data, key_callback, ticks=10):
        self.data, self.ticks = data, ticks
        key_callback(262)  # GLFW right arrow -> +x force

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def is_running(self):
        self.ticks -= 1
        return self.ticks >= 0

    def sync(self):
        pass


def test_arrow_key_pushes_sphere_along_world_x(monkeypatch):
    viewers = []

    def launch(model, data, key_callback):
        viewers.append(_FakeViewer(model, data, key_callback))
        return viewers[-1]

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch)
    teleop.main(seed=0)
    vel = viewers[0].data.qvel
    assert vel[0] > 0.1 and abs(vel[1]) < 1e-6


class _GoalOnSphereEnv(SphereReachEnv):
    """Every goal is drawn under the sphere, so each step succeeds."""

    def sample_goal(self):
        self.data.mocap_pos[self._goal_mocap, :2] = self.data.qpos[:2]
        self._prev_dist = self._dist()
        return self._get_obs()


def test_n_key_and_success_reset_the_goal(monkeypatch, capsys):
    def launch(model, data, key_callback):
        key_callback(78)  # GLFW N -> new goal
        key_callback(32)  # unbound key (space): ignored
        return _FakeViewer(model, data, lambda key: None, ticks=3)

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch)
    monkeypatch.setattr(teleop, "SphereReachEnv", _GoalOnSphereEnv)
    teleop.main(seed=0)
    assert capsys.readouterr().out.count("goal reached") == 3
