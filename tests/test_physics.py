"""Physical validation of the MuJoCo model against the analytic rolling-sphere model.

A solid sphere (I = 2/5 m r^2) rolling without slipping under a horizontal force F with viscous
damping c has an effective mass m_eff = m + I / r^2 = 1.4 m, so v(t) = F / c (1 - exp(-t / tau))
with tau = m_eff / c.
"""

import mujoco
import numpy as np
import pytest

from modular_sphere import OBS_SLICES, SphereReachEnv
from modular_sphere.dataset import exploration_action

# Physics constants matching sphere_arena.xml: mass=0.3, radius=0.1, damping=0.69 on slide joints
MASS, RADIUS, DAMPING = 0.3, 0.1, 0.69
FORCE = 1.0  # Unit force used in tests
V_TERMINAL = FORCE / DAMPING
TAU = 1.4 * MASS / DAMPING


@pytest.fixture
def env():
    """Sphere at rest near the west wall: 3.7 m of runway along +x."""
    e = SphereReachEnv()
    obs, _ = e.reset(seed=0)
    obs[OBS_SLICES["pos"]][0] = -1.85
    e.set_from_obs(obs)
    yield e
    e.close()


def _vx(env, action, steps):
    for _ in range(steps):
        obs, *_ = env.step(np.asarray(action, dtype=float))
    return obs[OBS_SLICES["vel"]][0]


def test_model_parameters_match_contract(env):
    body = env.model.body("sphere")
    assert body.mass[0] == pytest.approx(MASS)
    np.testing.assert_allclose(body.inertia, 0.4 * MASS * RADIUS**2)
    assert env.model.opt.timestep == pytest.approx(0.002)
    np.testing.assert_allclose(env.model.actuator_gear[:, 0], 1.0)


@pytest.mark.parametrize("steps", [10, 20, 40])
def test_force_response_matches_rolling_model(env, steps):
    t = steps * env.dt
    expected = V_TERMINAL * (1.0 - np.exp(-t / TAU))
    assert _vx(env, [1.0, 0.0], steps) == pytest.approx(expected, rel=0.02)


def test_free_decay_matches_rolling_model(env):
    v0 = _vx(env, [1.0, 0.0], 20)
    assert _vx(env, [0.0, 0.0], 20) == pytest.approx(v0 * np.exp(-20 * env.dt / TAU), rel=0.03)


def test_action_is_saturated(env):
    v_sat = _vx(env, [5.0, 0.0], 10)
    env.set_from_obs(np.r_[-1.85, 0.0, RADIUS + env.init_qpos[2], np.zeros(8)])
    assert v_sat == pytest.approx(_vx(env, [1.0, 0.0], 10))


def test_zero_action_negative_control(env):
    obs0 = env._get_obs()
    for _ in range(200):
        obs, _, terminated, _, _ = env.step(np.zeros(2))
        assert not terminated
    np.testing.assert_allclose(obs, obs0, atol=1e-9)


def test_timestep_convergence():
    """The 2 ms step agrees with a 0.5 ms step to 1 mm over 3 s of exploration (with impacts)."""
    rng = np.random.default_rng(0)
    actions = []
    noise = np.zeros(2)
    probe = SphereReachEnv()
    obs, _ = probe.reset(seed=3)
    for _ in range(60):
        actions.append(exploration_action(rng, obs, False, noise).copy())
    fine = SphereReachEnv()
    fine.model.opt.timestep = 0.0005
    fine.frame_skip = 100
    finals = []
    for e in (probe, fine):
        e.reset(seed=3)
        for a in actions:
            obs, *_ = e.step(a)
        finals.append(obs)
    np.testing.assert_allclose(finals[0][OBS_SLICES["pos"]], finals[1][OBS_SLICES["pos"]],
                               atol=1e-3)  # fmt: skip


def test_bang_bang_rollouts_stay_physical():
    env = SphereReachEnv()
    rng = np.random.default_rng(0)
    floor = env.model.geom("floor").id
    for ep in range(10):
        env.reset(seed=ep)
        action = rng.choice([-1.0, 1.0], size=2)
        for t in range(400):
            if t % 20 == 0:
                action = rng.choice([-1.0, 1.0], size=2)
            obs, *_ = env.step(action)
            assert np.all(np.isfinite(obs))
            pos, vel = obs[OBS_SLICES["pos"]], obs[OBS_SLICES["vel"]]
            # Walls inner face at 2.0 m; diagonal impacts at ~2 m/s penetrate up to ~1 cm (measured,
            # default soft contact), steady pressing at 1 N stays under 0.1 mm.
            assert np.all(np.abs(pos[:2]) < 2.0 - RADIUS + 0.015)
            assert RADIUS - 0.005 < pos[2] < 3 * RADIUS
            assert np.linalg.norm(vel[:2]) < np.sqrt(2) * V_TERMINAL * 1.05  # no energy gain
            depths = [c.dist for c in env.data.contact[: env.data.ncon]]
            assert min(depths, default=0.0) > -0.015
        assert any(floor in (c.geom1, c.geom2) for c in env.data.contact[: env.data.ncon])
    env.close()


def test_goal_seeking_controller_solves_task_and_random_does_not():
    env = SphereReachEnv()
    rng = np.random.default_rng(0)
    rates = {}
    for goal_seeking in (True, False):
        successes = 0
        for ep in range(30):
            obs, _ = env.reset(seed=ep)
            noise = np.zeros(2)
            for _ in range(200):
                obs, _, terminated, _, _ = env.step(
                    exploration_action(rng, obs, goal_seeking, noise)
                )
                if terminated:
                    successes += 1
                    break
        rates[goal_seeking] = successes / 30
    env.close()
    assert rates[True] == 1.0
    assert rates[False] < 0.5


def test_mujoco_version_supports_used_api():
    assert tuple(int(x) for x in mujoco.__version__.split(".")[:2]) >= (3, 2)
