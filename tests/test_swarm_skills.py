import numpy as np
import pytest

from modular_sphere.swarm import STEP_H, SphereSwarmStepEnv

torch = pytest.importorskip("torch")

from modular_sphere.swarm_skills import (  # noqa: E402
    MAX_SKILL_STEPS,
    SKILLS,
    STD_FLOOR,
    SkillCall,
    SkillController,
    SkillPlanner,
    SkillPrior,
    SkillValue,
    collect_skill,
    place,
    sample_start,
    skill_cost,
    skill_features,
)
from modular_sphere.swarm_torch import SwarmWorldModel  # noqa: E402
from modular_sphere.swarm_wam import (  # noqa: E402
    SkillWAM,
    WAMController,
    rollout_skill,
    select_elite,
)


def _calls(name, d):
    ag = (0,) if name == "roll" else (0, 1)
    return [SkillCall(name, ag, t) for t in d.target]


@pytest.fixture(scope="module")
def climb():
    return collect_skill("climb", 3, seed=0, noise=0.1)


@pytest.mark.parametrize("name", SKILLS)
def test_skill_controllers_reach_their_termination_event(name):
    d = collect_skill(name, 2, seed=0, noise=0.0)
    assert d.success.all() and (d.steps < MAX_SKILL_STEPS).all()
    assert d.data.spheres.shape == (2, MAX_SKILL_STEPS + 1, 2, 9)
    assert (d.data.valid.sum(1) == d.steps).all()
    if name != "roll":
        assert d.data.links.any()  # climb and pull link the two spheres


def test_collect_skill_rejects_unknown_skills():
    with pytest.raises(ValueError, match="unknown skill"):
        collect_skill("jump", 1)


def test_sample_start_is_in_the_initiation_set():
    rng = np.random.default_rng(0)
    for name in SKILLS:
        for _ in range(20):
            pos, target = sample_start(name, rng)
            assert pos.shape == (2, 3) and target.shape == (2,)
            gap = np.linalg.norm(pos[0, :2] - pos[1, :2])
            assert gap > 0.2 or pos[0, 2] != pos[1, 2]  # spheres do not overlap
    pos, _ = sample_start("pull", rng)
    assert pos[0, 2] == pytest.approx(STEP_H + 0.1) and pos[1, 2] == pytest.approx(0.1)


def test_place_puts_spheres_at_rest_and_unlinked():
    env = SphereSwarmStepEnv(n_spheres=2)
    env.reset(seed=0)
    pos = np.array([[-1.0, 0.0, 0.1], [-0.5, 0.3, 0.1]])
    place(env, pos)
    p, v, _ = env._kinematics()
    np.testing.assert_allclose(p, pos, atol=1e-9)
    assert not v.any() and not env.links.any()


def test_skill_features_and_cost_are_target_relative():
    call = SkillCall("roll", (0,), np.array([0.3, -0.2]))
    x = torch.zeros(2, 9)
    x[0, :3] = torch.tensor([0.3, -0.2, 0.1])
    f = skill_features(call, x, torch.zeros(2, 2, dtype=torch.bool))
    assert f.shape == (9,) and torch.allclose(f[:3], torch.tensor([0.0, 0.0, 0.1]))
    assert skill_cost(call, x).item() == pytest.approx(0.0, abs=1e-6)
    climb = SkillCall("climb", (0, 1), np.array([0.0, 0.2]))
    assert skill_features(climb, x, torch.ones(2, 2, dtype=torch.bool)).shape == (19,)
    assert skill_cost(climb, x).item() > 0
    assert skill_cost(SkillCall("pull", (0, 1), np.array([0.0, 0.2])), x).item() > 0


def test_prior_value_and_planner(climb, tmp_path):
    calls = _calls("climb", climb)
    prior, value = SkillPrior(2, hidden=32), SkillValue(2, hidden=32)
    losses = prior.fit(climb.data, calls, epochs=3)
    assert losses[-1] < losses[0]
    assert prior.std.min() >= STD_FLOOR
    value.fit(climb.data, calls, epochs=3)
    prior.save(str(tmp_path / "p.pt"))
    value.save(str(tmp_path / "v.pt"))
    prior, value = SkillPrior.load(str(tmp_path / "p.pt")), SkillValue.load(str(tmp_path / "v.pt"))
    f = skill_features(calls[0], torch.zeros(4, 2, 9), torch.zeros(4, 2, 2, dtype=torch.bool))
    assert prior(f).shape == (4, 2, 4)
    v = value(f)
    assert v.shape == (4,) and ((v >= 0) & (v <= MAX_SKILL_STEPS)).all()
    wm = SwarmWorldModel(hidden=16, epochs=1, device="cpu")
    wm.fit(climb.data)
    spheres = climb.data.spheres[0, 0]
    links = climb.data.links[0, 0]
    for kw in ({}, {"prior": prior, "value": value}):
        planner = SkillPlanner(wm, calls[0], 2, horizon=3, samples=8, iterations=1, **kw)
        a = planner.act(spheres, links)
        assert a.shape == (2, 4) and np.isin(a[:, 3], (-1.0, 1.0)).all()
        assert np.abs(a[:, :3]).max() <= 1.0


def test_skill_controller_done_events():
    call = SkillCall("roll", (0,), np.array([0.0, 0.0]))
    ctrl = SkillController(call, 2)
    pos, vel = np.zeros((2, 3)), np.zeros((2, 3))
    assert ctrl.done(pos, vel)
    vel[0, 0] = 1.0
    assert not ctrl.done(pos, vel)


def test_wam_fit_control_rollout_and_elite(climb, tmp_path):
    calls = _calls("climb", climb)
    wam = SkillWAM(2, history=2, chunk=3, width=32, layers=1, heads=2)
    losses = wam.fit(climb.data, calls, epochs=3, batch_size=64)
    assert losses[-1] < losses[0]
    assert wam.feat_std.min() >= STD_FLOOR and wam.delta_std.min() >= STD_FLOOR / 10
    noisy = SkillWAM(2, history=2, chunk=3, width=32, layers=1, heads=2)
    assert np.isfinite(noisy.fit(climb.data, calls, epochs=1, input_noise=0.01)).all()
    frames, acts = wam(torch.zeros(5, 2, wam.dim))
    assert frames.shape == (5, 3, wam.dim) and acts.shape == (5, 3, 2, 4)
    mean = wam.feat_mean.clone()
    wam.fit(climb.data, calls[:1], epochs=1, keep_stats=True)
    assert torch.equal(mean, wam.feat_mean)
    wam.save(str(tmp_path / "w.pt"))
    wam = SkillWAM.load(str(tmp_path / "w.pt"))
    for ens in (None, 0.1):
        ctrl = WAMController(wam, calls[0], 2, ensemble=ens)
        for _ in range(4):  # beyond the chunk length: old chunks drop out of the ensemble
            a = ctrl.act(climb.data.spheres[0, 0], climb.data.links[0, 0])
        assert a.shape == (2, 4) and np.isin(a[:, 3], (-1.0, 1.0)).all()
    data, rcalls, steps = rollout_skill(wam, "climb", n_starts=1, group=2, noise=3.0, seed=0)
    assert len(rcalls) == 2 and data.valid.sum() == steps.sum() == 2 * MAX_SKILL_STEPS
    valid = data.valid
    executed, labels = data.actions[valid], data.expert[valid]
    np.testing.assert_array_equal(np.clip(labels, -1.0, 1.0), executed)
    assert np.abs(labels).max() > 1.0  # labels are the samples before clipping
    success = np.array([True, True, False, False, True, False, False, False, False])
    keep = select_elite(success, np.array([10, 30, 5, 5, 50, 1, 9, 9, 9]), group=3)
    np.testing.assert_array_equal(
        keep, [True, False, False, False, True, False, False, False, False]
    )


def test_wam_windows_skip_empty_episodes_and_rollouts_stop_on_success(climb, monkeypatch):
    data = climb.data
    empty = type(data)(**{**vars(data), "valid": data.valid.copy()})
    empty.valid[0] = False
    wam = SkillWAM(2, history=2, chunk=3, width=16, layers=1, heads=2)
    hist, *_ = wam._windows(empty, _calls("climb", climb), torch.device("cpu"))
    assert hist.shape[0] == data.valid[1:].sum()
    monkeypatch.setattr(SkillController, "done", lambda self, pos, vel: True)
    d, _, steps = rollout_skill(wam, "climb", n_starts=1, group=2, seed=0)
    assert d.success.all() and (steps == 1).all()


def test_collect_skill_marks_unfinished_episodes(monkeypatch):
    monkeypatch.setattr(SkillController, "done", lambda self, pos, vel: False)
    d = collect_skill("roll", 1, seed=0)
    assert not d.success.any() and d.steps[0] == MAX_SKILL_STEPS


def test_skill_net_base_has_no_loss():
    from modular_sphere.swarm_skills import _SkillNet

    with pytest.raises(NotImplementedError):
        _SkillNet(1, 1)._loss(torch.zeros(1), torch.zeros(1))
