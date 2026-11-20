import gymnasium as gym
import numpy as np
import pytest

import modular_sphere  # noqa: F401  (registers the environments)
from modular_sphere.courses import (
    DOOR_TRAVEL,
    LEVELS,
    OBS_DIM,
    OBS_SLICES,
    SphereCourseEnv,
    WaypointPolicy,
)

SEEDS = range(3)


def rollout(env, policy, seed, max_steps=None):
    obs, _ = env.reset(seed=seed)
    policy.reset()
    min_dist, max_xy, max_z = 0.0, 0.0, 0.0
    for step in range(max_steps or LEVELS[env.unwrapped.level_id].max_steps):
        obs, _, terminated, truncated, _ = env.step(policy.act(obs))
        assert np.isfinite(obs).all()
        data = env.unwrapped.data
        min_dist = min(min_dist, data.contact.dist.min() if data.ncon else 0.0)
        max_xy, max_z = max(max_xy, np.abs(obs[:2]).max()), max(max_z, obs[2])
        if terminated or truncated:
            return terminated, step, min_dist, max_xy, max_z, obs
    return False, step, min_dist, max_xy, max_z, obs


@pytest.mark.parametrize("level", LEVELS)
def test_registered_env_shapes_and_determinism(level):
    env = gym.make(f"ModularSphere/Course-L{level}-v0")
    obs, _ = env.reset(seed=7)
    assert obs.shape == (OBS_DIM,) == env.observation_space.shape
    assert env.action_space.shape == (2,)
    assert env.spec.max_episode_steps == LEVELS[level].max_steps
    assert not obs[OBS_SLICES["button"]].any() and not obs[OBS_SLICES["door"]].any()
    obs2, _ = env.reset(seed=7)
    np.testing.assert_array_equal(obs, obs2)
    goal = obs[:2] + obs[OBS_SLICES["goal_rel"]]
    x0, x1, y0, y1 = LEVELS[level].goal_zone
    assert x0 <= goal[0] <= x1 and y0 <= goal[1] <= y1
    assert np.linalg.norm(obs[OBS_SLICES["goal_rel"]]) > 1.0  # never starts on the goal


@pytest.mark.parametrize("level", LEVELS)
def test_scripted_policy_solves_level(level):
    env = gym.make(f"ModularSphere/Course-L{level}-v0")
    policy = WaypointPolicy(level)
    for seed in SEEDS:
        success, _, min_dist, max_xy, max_z, _ = rollout(env, policy, seed)
        assert success, f"level {level} seed {seed}"
        assert min_dist > -0.01  # penetration bounded to 1 cm
        assert max_xy < 2.0 and max_z < 0.4  # stays in the arena, no launch


@pytest.mark.parametrize("level", LEVELS)
def test_negative_controls_fail(level):
    env = gym.make(f"ModularSphere/Course-L{level}-v0")
    zero = type("Zero", (), {"reset": lambda s: None, "act": lambda s, o: np.zeros(2)})()
    assert not rollout(env, zero, 0)[0]
    rng = np.random.default_rng(0)
    rand = type("Rand", (), {"reset": lambda s: None, "act": lambda s, o: rng.uniform(-1, 1, 2)})()
    assert not rollout(env, rand, 0)[0]
    # the obstacles are mandatory: pushing straight at the goal never succeeds
    straight = type(
        "Straight",
        (),
        {
            "reset": lambda s: None,
            "act": lambda s, o: np.clip(5 * o[OBS_SLICES["goal_rel"]], -1, 1),
        },
    )()
    assert not rollout(env, straight, 0)[0]


@pytest.mark.parametrize("level", [4, 5])
def test_button_opens_door(level):
    env = SphereCourseEnv(level=level)
    obs, _ = env.reset(seed=0)
    for _ in range(40):  # no button: the door stays closed
        obs, *_ = env.step(np.zeros(2))
    assert obs[OBS_SLICES["button"]][0] == 0 and obs[OBS_SLICES["door"]][0] == 0
    # teleport on the plate through the oracle interface, as a world model would
    state = obs.copy()
    state[:2] = LEVELS[level].button
    env.set_from_obs(state)
    obs, _, _, _, info = env.step(np.zeros(2))
    assert info["button_pressed"] and obs[OBS_SLICES["button"]][0] == 1.0
    for _ in range(60):
        obs, *_ = env.step(np.zeros(2))
    assert obs[OBS_SLICES["door"]][0] > 0.95
    assert env.data.qpos[env._door_qadr] == pytest.approx(DOOR_TRAVEL, abs=0.05)
    obs, _ = env.reset(seed=0)  # reset closes the door and clears the latch
    assert obs[OBS_SLICES["button"]][0] == 0 and obs[OBS_SLICES["door"]][0] == 0


def test_door_closed_blocks_and_invalid_level():
    env = SphereCourseEnv(level=4)
    obs, _ = env.reset(seed=0)
    for _ in range(100):  # push straight through the wall line at the door height
        obs, *_ = env.step(np.array([1.0, 0.0]))
    assert obs[0] < 0.0
    with pytest.raises(ValueError):
        SphereCourseEnv(level=0)
