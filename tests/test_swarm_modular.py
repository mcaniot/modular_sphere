from __future__ import annotations

import numpy as np
import pytest
import torch

from modular_sphere.swarm import Course
from modular_sphere.swarm_modular import (
    KINDS,
    SKILL_IDS,
    Episodes,
    HeldNoise,
    SwarmWM,
    fit,
    pad_episodes,
    position_errors,
    progress_cost,
    record_episode,
    simulator_rollouts,
    unroll_loss,
    windows,
)
from modular_sphere.swarm_plan import SkillScheduler, script_executor
from modular_sphere.swarm_skills import SkillCall


class _ConstEffect:
    def duration(self, name, states, target, level=None):
        return 50.0


@pytest.fixture(scope="module")
def episodes():
    eps = []
    for seed in (0, 1):
        sched = SkillScheduler(4, _ConstEffect(), script_executor(), seed=seed)
        eps.append(record_episode(sched, seed))
    return pad_episodes(eps)


def test_episodes_label_every_sphere_with_its_call(episodes):
    ep = episodes
    assert ep.spheres.shape[1] == ep.valid.shape[1] + 1
    v = ep.valid
    skill = ep.cat[v] // 2
    assert set(np.unique(skill)) <= set(SKILL_IDS.values())
    assert (skill > 0).any()
    idle = skill == 0
    assert np.isnan(ep.ref[v][idle]).all() and not np.isnan(ep.ref[v][~idle]).any()
    assert (ep.group[v][idle] < 0).all() and (ep.group[v][~idle] >= 0).all()
    # the two spheres of a climb or pull call share its group and take both roles
    pair = (skill >= 2) & (ep.cat[v] % 2 == 1)
    assert pair.any()


def test_windows_mask_the_episode_end(episodes):
    data = windows(episodes, 8, 4)
    assert data["x"].shape[1:] == (9, 4, 9) and data["act"].shape[1:] == (8, 4, 4)
    last = int(episodes.valid[0].sum())
    rows = (data["episode"] == 0) & (data["t"] + 8 > last)
    assert rows.any() and not data["mask"][rows].all()


def test_modular_experts_only_see_their_call(episodes):
    torch.manual_seed(0)
    model = SwarmWM("modular", 16, 1, 2)
    b = windows(episodes, 1, 1)
    ctx = {k: b[k][:, 0] for k in ("links", "cat", "ref", "rise", "group")}
    ctx["faces"] = b["faces"]
    x, act = b["x"][:, 0], b["act"][:, 0]
    out = model.step(x, act, ctx)
    # moving a sphere of another group (far, with another action) leaves the others unchanged
    i = int(torch.nonzero(ctx["group"][:, 0] != ctx["group"][:, 1])[0])
    x2, act2 = x.clone(), act.clone()
    x2[i, 1, :3] += 1.0
    act2[i, 1] = -act2[i, 1]
    out2 = model.step(x2, act2, ctx)
    torch.testing.assert_close(out2[i, 0], out[i, 0])
    assert not torch.allclose(out2[i, 1], out[i, 1])


def test_framed_models_are_translation_invariant_in_xy():
    torch.manual_seed(0)
    model = SwarmWM("frame", 16, 1, 2)
    x = torch.randn(3, 2, 9)
    act = torch.randn(3, 2, 4)
    ctx = {"links": torch.zeros(3, 2, 2, dtype=torch.bool), "cat": torch.full((3, 2), 4),
           "ref": torch.zeros(3, 2, 3), "rise": torch.full((3, 2), 0.12),
           "group": torch.zeros(3, 2, dtype=torch.long), "faces": torch.zeros(3, 3, 3)}  # fmt: skip
    shift = torch.tensor([0.4, -0.2, 0.0])
    moved = {**ctx, "ref": ctx["ref"] + shift}
    x2 = x.clone()
    x2[..., :3] += shift
    out, out2 = model.step(x, act, ctx), model.step(x2, act, moved)
    torch.testing.assert_close(out2[..., :3] - shift, out[..., :3])
    torch.testing.assert_close(out2[..., 3:], out[..., 3:])


@pytest.mark.parametrize("kind", KINDS)
def test_fit_lowers_the_unroll_loss(episodes, kind):
    data = windows(episodes, 4, 8)
    model = SwarmWM(kind, 16, 1, 2)
    losses = fit(model, data, 4, lr=3e-3, seed=0)
    assert losses[-1] < losses[0]
    with torch.no_grad():
        assert float(unroll_loss(model, data)) == pytest.approx(losses[-1], rel=0.5)
    err = position_errors(model, windows(episodes, 6, 20))
    assert err.shape[1:] == (6, 4) and np.isfinite(np.nanmean(err))


def test_unknown_kind_is_rejected():
    with pytest.raises(ValueError, match="unknown kind"):
        SwarmWM("huge")


def test_call_ref_uses_the_roll_target_or_the_step_face():
    from modular_sphere.swarm_modular import call_ref

    roll = SkillCall("roll", (0,), np.array([0.3, -0.2]), face=1.0, floor=0.1, rise=0.12)
    climb = SkillCall("climb", (0, 1), np.array([0.0, 0.4]), face=1.0, floor=0.1, rise=0.12)
    np.testing.assert_allclose(call_ref(roll), [0.3, -0.2, 0.1, 0.12])
    np.testing.assert_allclose(call_ref(climb), [1.0, 0.4, 0.1, 0.12])


def test_fit_on_one_expert_keeps_the_others_and_the_normalisation(episodes):
    data = windows(episodes, 4, 8)
    model = SwarmWM("modular", 16, 1, 2)
    fit(model, data, 1, seed=0)
    before = [p.detach().clone() for p in model.parameters()]
    stats = model.stats.clone()
    fit(model, data, 2, lr=3e-3, seed=1, params=list(model.nets[3].parameters()))
    own = {id(p) for p in model.nets[3].parameters()}
    pairs = list(zip(model.parameters(), before, strict=True))
    assert all(torch.equal(p, q) for p, q in pairs if id(p) not in own)
    assert not all(torch.equal(p, q) for p, q in pairs if id(p) in own)
    torch.testing.assert_close(model.stats, stats)


def test_held_noise_holds_clipped_offsets_and_keeps_attach():
    noise = HeldNoise(0, sigma=0.5, p=1.0, max_len=3)
    act = np.zeros((4, 4))
    act[:, 3] = 1.0
    out = np.stack([noise(act) for _ in range(30)])
    assert np.all(out[..., 3] == 1.0) and np.all(np.abs(out[..., :3]) <= 1.0)
    held = np.all(np.isclose(out[1:, :, :3], out[:-1, :, :3]), axis=(1, 2))
    assert 0.3 < held.mean() < 0.9  # offsets last 1 to 3 steps


def test_progress_cost_is_zero_at_the_termination_event():
    x, lk = torch.zeros(2, 2, 9), torch.zeros(2, 2, 2, dtype=torch.bool)
    roll = SkillCall("roll", (0,), np.array([0.3, 0.4]))
    x[1, 0, :2] = torch.tensor([0.3, 0.4])
    torch.testing.assert_close(progress_cost(roll, x, lk), torch.tensor([50.0, 0.0]))
    climb = SkillCall("climb", (0, 1), np.array([0.0, 0.0]), floor=0.1)
    x[1, 0, 2] = 0.4
    torch.testing.assert_close(progress_cost(climb, x, lk), torch.tensor([40.0, 0.0]))
    pull = SkillCall("pull", (0, 1), np.array([0.0, 0.0]))
    x[:, 0, 2], x[1, 1, 2], x[1, 1, 0] = 0.2, 0.35, 0.3
    torch.testing.assert_close(progress_cost(pull, x, lk), torch.tensor([30.0, 0.0]))


def test_simulator_rollouts_restore_the_state_exactly():
    from modular_sphere.swarm import SphereSwarmStepEnv

    env = SphereSwarmStepEnv(n_spheres=2)
    env.reset(seed=0)
    acts = {a: np.array([1.0, 0.0, 0.0, 1.0]) for a in env.possible_agents}
    for _ in range(3):
        env.step(acts)
    before, links = env.spheres(), env.links
    act = torch.zeros(3, 4, 2, 4)
    act[1, :, :, 0], act[2, :, :, 1] = 1.0, -1.0
    x, lk = simulator_rollouts(env, act)
    assert x.shape == (3, 2, 9) and lk.shape == (3, 2, 2)
    assert not np.allclose(x[1], x[2])
    np.testing.assert_array_equal(env.spheres(), before)
    np.testing.assert_array_equal(env.links, links)
    again, _ = simulator_rollouts(env, act)
    np.testing.assert_array_equal(again, x)


def test_episodes_round_trip_and_subset(episodes, tmp_path):
    episodes.save(str(tmp_path / "e.npz"))
    back = Episodes.load(str(tmp_path / "e.npz"))
    for k, v in episodes.__dict__.items():
        np.testing.assert_array_equal(getattr(back, k), v)
    assert back.subset([1]).valid.shape[0] == 1


def test_record_episode_applies_dynamics_and_perturbation():
    seen = []
    sched = SkillScheduler(4, _ConstEffect(), script_executor(), seed=0)

    def dyn(env, seed):
        seen.append(seed)

    def zero(act):
        out = act.copy()
        out[:, :3] = 0.0
        return out

    ep = record_episode(sched, 3, dynamics=dyn, perturb=zero)
    assert seen == [3] and np.all(ep["a"][:, :, :3] == 0.0) and not ep["ok"]


@pytest.fixture(scope="module")
def verifier_parts(episodes, tmp_path_factory):
    from modular_sphere.swarm_skills import SKILLS, SkillValue
    from modular_sphere.swarm_wam import SkillWAM

    n_agents = {"roll": 1, "climb": 2, "pull": 2}
    model = SwarmWM("modular", 16, 1, 2)
    fit(model, windows(episodes, 4, 8), 1, seed=0)
    path = tmp_path_factory.mktemp("wm") / "wm.pt"
    torch.save({"kind": "modular", "cfg": model.cfg, "state": model.state_dict()}, path)
    values = {s: SkillValue(n_agents[s], hidden=16) for s in SKILLS}
    wams = {s: SkillWAM(n_agents[s], history=2, chunk=4, width=16, layers=1, heads=2)
            for s in SKILLS}  # fmt: skip
    return str(path), values, wams


def _run_verified(verifier, wams, steps=6):
    from modular_sphere.swarm import SphereSwarmStepEnv

    env = SphereSwarmStepEnv(n_spheres=4)
    sched = SkillScheduler(4, _ConstEffect(), verifier.executor(wams), seed=0)
    verifier.scheduler = sched
    obs, _ = env.reset(seed=0)
    sp = env.spheres()
    sched.reset(sp[:, :3], env.goal)
    for _ in range(steps):
        a = sched.act([obs[x] for x in env.possible_agents], sp, env.links)
        assert a.shape == (4, 4) and np.all(np.abs(a) <= 1.0) and set(np.unique(a[:, 3])) <= {-1, 1}
        verifier.last = a
        obs, *_ = env.step(dict(zip(env.possible_agents, a, strict=True)))
        sp = env.spheres()
    return env


def test_wm_verifier_selects_among_candidates_with_a_world_model(verifier_parts):
    from modular_sphere.swarm_modular import WMVerifier, load_wm

    path, values, wams = verifier_parts
    wm = load_wm(path)
    assert not wm.training
    with pytest.raises(RuntimeError, match="scheduler"):
        WMVerifier(wm, values).predict(torch.zeros(2, 1, 4, 4), np.zeros((4, 9)), np.eye(4) > 0)
    v = WMVerifier(wm, values, k=4, sigma=0.3, horizon=3, margin=-1e9)
    _run_verified(v, wams)
    assert v.overrides > 0  # a negative margin always takes the best candidate
    v = WMVerifier(wm, values, k=4, horizon=3, margin=1e9)
    v.objective, v.course = progress_cost, Course()
    _run_verified(v, wams)
    assert v.overrides == 0  # a huge margin always keeps the policy's chunk


@pytest.mark.parametrize("objective", [None, progress_cost])
def test_simulator_scorer_ranks_rollouts_in_mujoco(verifier_parts, objective):
    from modular_sphere.swarm import SphereSwarmStepEnv
    from modular_sphere.swarm_modular import simulator_scorer

    env = SphereSwarmStepEnv(n_spheres=4)
    env.reset(seed=0)
    call = SkillCall("roll", (0,), env.spheres()[0, :2] + [0.3, 0.0])
    act = torch.zeros(3, 4, 4, 4)
    act[1, :, 0, 0] = 1.0
    out = simulator_scorer(env, verifier_parts[1], objective)(call, act, env.spheres(), env.links)
    assert out.shape == (3,) and torch.isfinite(out).all()
    if objective is not None:
        assert out[1] < out[0]  # pushing toward the target makes progress


def test_sphere_context_shares_the_group_of_a_call_and_counts_params():
    from modular_sphere.swarm_modular import sphere_context

    pull = SkillCall("pull", (2, 0), np.array([0.0, 0.1]))
    ctx = sphere_context({0: pull, 2: pull}, 3)
    assert ctx["group"][0] == ctx["group"][2] == 0 and ctx["group"][1] < 0
    assert ctx["cat"][2] == 2 * SKILL_IDS["pull"] and ctx["cat"][0] == 2 * SKILL_IDS["pull"] + 1
    assert SwarmWM("modular", 16, 1, 2).n_params() > SwarmWM("world", 16, 1, 2).n_params()


def test_simulator_rollouts_hold_a_finished_episode():
    from modular_sphere.swarm import SphereSwarmStepEnv

    env = SphereSwarmStepEnv(n_spheres=2)
    env.reset(seed=0)
    env.agents = []
    x, _ = simulator_rollouts(env, torch.ones(2, 3, 2, 4))
    np.testing.assert_array_equal(x[0], env.spheres())
