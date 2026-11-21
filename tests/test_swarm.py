import numpy as np
import pytest
from pettingzoo.test import parallel_api_test

from modular_sphere.swarm import (
    ACT_DIM,
    MAX_LIFT,
    STEP_H,
    STEP_X,
    WALL_H,
    Course,
    SphereSwarmStepEnv,
    SwarmClimbPolicy,
    gather_radius,
    obs_slices,
    parallel_env,
    random_course,
)


def place(env, positions):
    """Put sphere centres at ``positions`` ((x, y) on the floor or (x, y, z)), at rest."""
    for i, p in enumerate(positions):
        env.data.qpos[env._qpos[i] : env._qpos[i] + 3] = p if len(p) == 3 else (*p, env.radius)
        env.data.qvel[env._qvel[i] : env._qvel[i] + 6] = 0.0
    env.data.eq_active[:] = False


def act(env, **per_agent):
    return {a: np.asarray(per_agent.get(a, np.zeros(ACT_DIM))) for a in env.possible_agents}


def test_pettingzoo_parallel_api():
    parallel_api_test(parallel_env(n_spheres=3), num_cycles=50)


@pytest.mark.parametrize("n", [2, 4])
def test_spaces_and_observation_layout(n):
    env = SphereSwarmStepEnv(n_spheres=n)
    obs, infos = env.reset(seed=0)
    dim = obs_slices(n)["others"].stop
    assert dim == 12 + 7 * (n - 1)
    assert set(obs) == set(infos) == set(env.possible_agents)
    for agent, o in obs.items():
        assert o.shape == env.observation_space(agent).shape == (dim,)
        assert env.action_space(agent).shape == (ACT_DIM,)
    assert env.state().shape == env.state_space.shape == (9 * n + 3 + n * (n - 1) // 2,)
    sl = obs_slices(n)
    o0 = obs["sphere_0"]
    np.testing.assert_allclose(o0[sl["pos"]] + o0[sl["goal_rel"]], env.goal)
    others = o0[sl["others"]].reshape(n - 1, 7)
    np.testing.assert_allclose(others[0, :3], obs["sphere_1"][:3] - o0[:3])  # cyclic order
    np.testing.assert_allclose(obs["sphere_1"][sl["others"]][-7:-4], o0[:3] - obs["sphere_1"][:3])
    assert env.goal[2] == pytest.approx(STEP_H + env.radius)


def test_deterministic_for_a_seed():
    def run(seed):
        env = SphereSwarmStepEnv(n_spheres=3)
        env.reset(seed=seed)
        rng = np.random.default_rng(1)
        for _ in range(30):
            env.step({a: rng.uniform(-1, 1, ACT_DIM) for a in env.agents})
        return env.state()

    np.testing.assert_array_equal(run(5), run(5))
    assert not np.array_equal(run(5), run(6))


def test_lone_sphere_cannot_climb_the_step():
    env = SphereSwarmStepEnv(n_spheres=2)
    env.reset(seed=0)
    place(env, [(STEP_X - 0.3, 0.0), (-1.5, 1.0)])
    max_x = -np.inf
    for _ in range(100):  # full push into the face and full lift request (no partner)
        obs, *_ = env.step(act(env, sphere_0=[1.0, 0.0, 1.0, 1.0]))
        max_x = max(max_x, obs["sphere_0"][0])
    assert max_x < STEP_X - env.radius + 0.01
    assert obs["sphere_0"][2] < env.radius + 0.01


def test_link_needs_both_requests_and_contact_and_keeps_distance():
    env = SphereSwarmStepEnv(n_spheres=3)
    env.reset(seed=0)
    place(env, [(-1.0, 0.0), (-1.0 + 2 * env.radius + 0.005, 0.0), (-1.0, 0.5)])
    env.step(act(env, sphere_0=[0, 0, 0, 1]))
    assert not env.links.any()  # one request only
    env.step(act(env, sphere_0=[0, 0, 0, 1], sphere_2=[0, 0, 0, 1]))
    assert not env.links.any()  # both request but too far apart
    obs, _, _, _, infos = env.step(act(env, sphere_0=[0, 0, 0, 1], sphere_1=[0, 0, 0, 1]))
    assert env.links[0, 1] and env.links[1, 0] and infos["sphere_0"]["n_links"] == 1
    assert obs["sphere_0"][obs_slices(3)["others"]][6] == 1.0  # linked flag of sphere_1
    d0 = np.linalg.norm(obs["sphere_0"][:3] - obs["sphere_1"][:3])
    for _ in range(20):  # pull apart while linked
        obs, *_ = env.step(act(env, sphere_0=[-1, 0, 0, 1], sphere_1=[1, 0, 0, 1]))
    assert np.linalg.norm(obs["sphere_0"][:3] - obs["sphere_1"][:3]) == pytest.approx(d0, abs=2e-3)
    assert env.model.tendon_group[0] == 0 and env.model.tendon_group[1] == 5  # drawn if linked
    env.step(act(env, sphere_0=[-1, 0, 0, 0], sphere_1=[1, 0, 0, 1]))
    assert not env.links.any()


def test_lift_is_internal_and_lets_a_sphere_pivot_over_its_partner():
    env = SphereSwarmStepEnv(n_spheres=3)
    env.reset(seed=0)
    place(env, [(-1.0, 0.0), (-1.0 + 2 * env.radius, 0.0), (-1.0 + 4 * env.radius, 0.0)])
    req = [0, 0, 0, 1]
    env.step(act(env, sphere_0=req, sphere_1=req, sphere_2=req))
    assert env.links.sum() == 4  # chain 0-1-2
    env.step(act(env, sphere_0=req, sphere_1=[0, 0, 1, 1], sphere_2=req))
    ctrl_z = env.data.ctrl.reshape(3, 3)[:, 2]
    np.testing.assert_allclose(ctrl_z, [-MAX_LIFT / 2, MAX_LIFT, -MAX_LIFT / 2])
    assert ctrl_z.sum() == pytest.approx(0.0)
    # A pair: the lifting sphere rises over the other.
    env.reset(seed=0)
    place(env, [(-1.0, 0.0), (-1.0 + 2 * env.radius, 0.0), (1.5, 1.5)])
    for _ in range(10):
        obs, *_ = env.step(act(env, sphere_0=[0, 0, 1, 1], sphere_1=req))
    assert obs["sphere_0"][2] > 2 * env.radius


@pytest.mark.parametrize("n", [2, 3, 4])
@pytest.mark.parametrize("seed", range(3))
def test_scripted_swarm_climbs_the_step(n, seed):
    env = SphereSwarmStepEnv(n_spheres=n)
    policy = SwarmClimbPolicy(n, env.radius)
    obs, _ = env.reset(seed=seed)
    total = 0.0
    while env.agents:
        obs, rewards, terms, truncs, infos = env.step(policy.act(obs))
        total += rewards["sphere_0"]
    assert terms["sphere_0"] and not truncs["sphere_0"] and infos["sphere_0"]["is_success"]
    assert all(terms.values()) and len(set(rewards.values())) == 1  # shared team outcome
    assert infos["sphere_0"]["n_arrived"] == n and total > 0
    for o in obs.values():  # every sphere is on the platform, inside the gathering zone
        assert o[0] > STEP_X and o[2] > STEP_H + env.radius / 2
        assert np.linalg.norm(o[9:11]) <= gather_radius(n)


def test_courses_are_validated_and_random_courses_stay_in_bounds():
    for faces, rises in (((), ()), ((0.5,), (0.1, 0.1)), ((0.5, 0.2), (0.1, 0.1)), ((0.5,), (0,))):
        with pytest.raises(ValueError, match="course"):
            Course(faces, rises)
    c = Course((-0.3, 0.5), (0.12, 0.14))
    assert c.floor(0) == 0 and c.floor(1) == 0.12 and np.isclose(c.top, 0.26)
    sizes = set()
    for seed in range(30):
        c = random_course(np.random.default_rng(seed))
        sizes.add(len(c.faces))
        assert -0.4 <= c.faces[0] and c.faces[-1] <= 1.0
        assert all(0.12 <= h <= 0.15 for h in c.rises)
        assert all(0.649 <= d <= 0.851 for d in np.diff(c.faces))
    assert sizes == {1, 2, 3}
    assert random_course(np.random.default_rng(3)) == random_course(np.random.default_rng(3))


def test_scripted_swarm_climbs_a_three_step_course():
    course = random_course(np.random.default_rng(0))
    assert len(course.faces) == 3
    env = SphereSwarmStepEnv(n_spheres=4, course=course)
    assert env.step_height == course.top and env.max_steps == 3 * SphereSwarmStepEnv().max_steps
    policy = SwarmClimbPolicy(4, env.radius, course)
    obs, _ = env.reset(seed=0)
    assert env.goal[0] >= course.faces[-1] + 0.45
    while env.agents:
        obs, _, _, _, infos = env.step(policy.act(obs))
    assert infos["sphere_0"]["is_success"] and policy._level == 2
    for o in obs.values():
        assert o[0] > course.faces[-1] and o[2] > course.top + env.radius / 2


def test_waiting_slots_stay_inside_the_walls_and_apart():
    policy = SwarmClimbPolicy(4, 0.1)
    for y in (-1.6, 0.0, 1.6):
        ys = sorted(s[1] for s in policy._wait_slots(0.5, y))
        assert len(ys) == 4 and min(np.diff(ys)) >= 0.3 - 1e-9 and max(map(abs, ys)) <= 1.7
        assert all(s[0] == 0.5 - policy.WAIT_X for s in policy._wait_slots(0.5, y))


def test_success_needs_every_sphere_in_the_zone():
    env = SphereSwarmStepEnv(n_spheres=3)
    env.reset(seed=0)
    g, z = env.goal, STEP_H + env.radius
    assert env.gather_radius == pytest.approx(0.2 * np.sqrt(3))
    assert env.model.geom("wall_east").size[2] * 2 == pytest.approx(WALL_H)
    place(env, [(g[0], g[1], z), (g[0] + 0.22, g[1], z), (0.0, 0.0)])
    _, rewards, terms, _, infos = env.step(act(env))
    assert infos["sphere_0"]["n_arrived"] == 2 and not terms["sphere_0"] and env.agents
    place(env, [(g[0], g[1], z), (g[0] + 0.22, g[1], z), (g[0] - 0.22, g[1], z)])
    _, rewards, terms, _, infos = env.step(act(env))
    assert infos["sphere_0"]["n_arrived"] == 3 and terms["sphere_0"] and not env.agents
    assert rewards["sphere_0"] > 1.0  # progress of the mean distance and success bonus


def test_truncation_render_and_errors():
    env = SphereSwarmStepEnv(n_spheres=2, max_steps=3, render_mode="rgb_array", width=64, height=48)
    env.reset(seed=0)
    for _ in range(3):
        _, _, terms, truncs, _ = env.step({})  # missing agents get a zero action
    assert truncs["sphere_0"] and not terms["sphere_0"] and env.agents == []
    with pytest.raises(RuntimeError, match="reset"):
        env.step(act(env))
    env.render()
    frame = env.render()
    assert frame.shape == (48, 64, 3) and frame.dtype == np.uint8
    env.close()
    env.close()
    assert SphereSwarmStepEnv(n_spheres=2).render() is None
    with pytest.raises(ValueError, match="n_spheres"):
        SphereSwarmStepEnv(n_spheres=1)
    with pytest.raises(ValueError, match="render_mode"):
        SphereSwarmStepEnv(render_mode="human")
