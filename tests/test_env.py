import gymnasium as gym
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from modular_sphere import ENV_ID, OBS_SLICES, SphereReachEnv
from modular_sphere.env import GOAL_MAX_DIST, GOAL_MIN_DIST, MAX_EPISODE_STEPS


@pytest.fixture
def env():
    e = SphereReachEnv()
    yield e
    e.close()


def test_gymnasium_api():
    e = gym.make(ENV_ID)
    check_env(e.unwrapped, skip_render_check=True)
    assert e.spec.max_episode_steps == MAX_EPISODE_STEPS
    assert e.unwrapped.dt == pytest.approx(0.05)
    e.close()


def test_reset_is_seeded_and_goal_in_range(env):
    a, _ = env.reset(seed=42)
    b, _ = env.reset(seed=42)
    np.testing.assert_array_equal(a, b)
    for seed in range(20):
        obs, _ = env.reset(seed=seed)
        np.testing.assert_allclose(obs[OBS_SLICES["vel"]], 0.0)
        goal = np.abs(obs[OBS_SLICES["goal_rel"]] + obs[OBS_SLICES["pos"]][:2])
        assert np.all((goal >= GOAL_MIN_DIST) & (goal <= GOAL_MAX_DIST))


def test_force_pushes_and_sphere_rolls(env):
    env.reset(seed=0)
    for _ in range(10):
        obs, *_ = env.step(np.array([1.0, 0.0]))
    vx = obs[OBS_SLICES["vel"]][0]
    wy = obs[OBS_SLICES["ang_vel"]][1]
    assert vx > 0.5
    assert wy == pytest.approx(vx / 0.1, rel=0.05)  # rolling without slipping: w = v / r
    assert abs(obs[OBS_SLICES["pos"]][1]) < 1e-6


def test_walls_contain_the_sphere(env):
    env.reset(seed=0)
    for _ in range(200):
        obs, *_ = env.step(np.array([1.0, 1.0]))
    assert np.all(np.abs(obs[OBS_SLICES["pos"]][:2]) < 2.0)


def test_success_terminates_with_bonus(env):
    obs, _ = env.reset(seed=0)
    for _ in range(MAX_EPISODE_STEPS):
        action = np.clip(2.0 * obs[OBS_SLICES["goal_rel"]] - obs[OBS_SLICES["vel"]][:2], -1, 1)
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated:
            break
    assert terminated and not truncated and info["is_success"]
    assert reward > 9.0


def test_rgb_render():
    e = SphereReachEnv(render_mode="rgb_array", width=64, height=64)
    e.reset(seed=0)
    frame = e.render()
    assert frame.shape == (64, 64, 3) and frame.std() > 0
    e.close()


def test_human_mode_renders_every_step(monkeypatch):
    e = SphereReachEnv(render_mode="human")
    calls = []
    monkeypatch.setattr(e, "render", lambda: calls.append(1))  # no window in tests
    e.reset(seed=0)  # Gymnasium's MujocoEnv also renders on reset in human mode
    calls.clear()
    for _ in range(3):
        e.step(np.zeros(2))
    assert len(calls) == 3
