import numpy as np
import pytest

from modular_sphere.swarm_data import SwarmDataset, collect

torch = pytest.importorskip("torch")

from modular_sphere.swarm_torch import (  # noqa: E402
    SwarmPolicy,
    SwarmWorldModel,
    evaluate_policy,
    evaluate_world_model,
    improve_in_imagination,
    improve_with_ppo,
    local_obs,
    task_reward,
)


@pytest.fixture(scope="module")
def data():
    return collect(3, n_spheres=2, seed=0, noise=0.2)


def test_collect_layout_padding_and_roundtrip(data, tmp_path):
    e, t1, n, _ = data.spheres.shape
    assert (e, n) == (3, 2) and t1 == data.actions.shape[1] + 1 == 301
    assert data.obs.shape == (3, 301, 2, 19) and data.links.shape == (3, 301, 2, 2)
    length = data.valid.sum(1)
    for k in range(e):  # valid prefix, then padding repeats the last state with zero actions
        assert data.valid[k, : length[k]].all() and not data.valid[k, length[k] :].any()
        np.testing.assert_array_equal(data.spheres[k, length[k]], data.spheres[k, -1])
        assert not data.actions[k, length[k] :].any()
    assert data.links.any()  # the expert links spheres to climb
    assert np.abs(data.actions).max() <= 1.0
    noisy = data.valid[..., None] & (data.actions[..., :3] != data.expert[..., :3]).any(-1)
    assert noisy.any()  # DART: executed noisy, labelled clean
    np.testing.assert_array_equal(data.actions[..., 3], data.expert[..., 3])
    data.save(tmp_path / "d.npz")
    again = SwarmDataset.load(tmp_path / "d.npz")
    np.testing.assert_array_equal(again.obs, data.obs)
    both = SwarmDataset.concat([data, again])
    assert both.n_spheres == 2 and both.success.shape == (6,)


def test_torch_obs_and_reward_match_the_env(data):
    x = torch.as_tensor(data.spheres[:, 40])
    lk = torch.as_tensor(data.links[:, 40].astype(np.float64))
    goal = torch.as_tensor(data.goal)
    np.testing.assert_allclose(local_obs(x, lk, goal).numpy(), data.obs[:, 40])
    prev = torch.as_tensor(data.spheres[:, 39])
    reward, _ = task_reward(x, prev, goal)
    np.testing.assert_allclose(reward.numpy(), data.rewards[:, 39], atol=1e-9)
    k = int(data.valid[0].sum())  # last transition of a successful episode
    if data.success[0]:
        end = torch.as_tensor(data.spheres[:1, k])
        assert task_reward(end, torch.as_tensor(data.spheres[:1, k - 1]), goal[:1])[1].item() == 1


def test_world_model_policy_and_imagination_pipeline(data, tmp_path):
    wm = SwarmWorldModel(hidden=32, epochs=3, window=4, device="cpu")
    wm.fit(data)
    assert wm.train_losses[-1] < wm.train_losses[0]
    report = evaluate_world_model(wm, data, [1, 5], stride=50)
    assert set(report) == {1, 5} and 0 <= report[1]["link_acc"] <= 1
    pred, links = wm.rollout(data.spheres[:2, 0], data.links[:2, 0], data.actions[:2, :6])
    assert pred.shape == (2, 6, 2, 9) and links.shape == (2, 6, 2, 2)
    wm.save(str(tmp_path / "wm.pt"))
    np.testing.assert_allclose(
        SwarmWorldModel.load(str(tmp_path / "wm.pt"), "cpu").rollout(
            data.spheres[:2, 0], data.links[:2, 0], data.actions[:2, :6]
        )[0],
        pred,
    )
    policy = SwarmPolicy(2, hidden=32, epochs=2, device="cpu")
    policy.fit(data)
    obs = {"sphere_0": data.obs[0, 0, 0], "sphere_1": data.obs[0, 0, 1]}
    act = policy(obs)
    assert act["sphere_0"].shape == (4,) and act["sphere_0"][3] in (-1.0, 1.0)
    log = improve_in_imagination(
        policy, wm, data, iterations=3, horizon=3, batch_size=8, critic_warmup=1
    )
    assert len(log) == 1 and np.isfinite(log[0]["return"]) and log[0]["uncertainty"] == 0
    assert all(p.requires_grad for p in wm.net.parameters())
    other = SwarmWorldModel(hidden=32, epochs=1, window=4, seed=1, device="cpu")
    other.fit(data)
    stats = other.stats
    other.fit(data)  # continued training keeps the first normalization
    assert other.stats is stats
    log = improve_in_imagination(policy, [wm, other], data, iterations=1, horizon=3, batch_size=8)
    assert log[0]["uncertainty"] > 0  # disagreement penalty of the ensemble (MOPO)
    assert all(p.requires_grad for p in other.net.parameters())
    policy.save(str(tmp_path / "pi.pt"))
    again = SwarmPolicy.load(str(tmp_path / "pi.pt"), "cpu")
    np.testing.assert_allclose(again(obs)["sphere_1"], policy(obs)["sphere_1"])
    ev = evaluate_policy(again, 2, [0])
    assert ev.success.shape == ev.steps.shape == ev.n_arrived.shape == (1,)


def test_dagger_collection_uses_the_learner(data):
    policy = SwarmPolicy(2, hidden=16, epochs=1, device="cpu")
    policy.fit(data)
    mixed = collect(1, n_spheres=2, seed=5, noise=0.0, policy=policy, beta=0.0)
    steps = mixed.valid[0]
    # The learner drives (attach in {-1, 1}), the expert labels (attach in {0, 1}).
    assert set(np.unique(mixed.actions[0, steps, :, 3])) <= {-1.0, 1.0}
    assert set(np.unique(mixed.expert[0, steps, :, 3])) <= {0.0, 1.0}


def test_mcnemar_exact():
    from modular_sphere.tools.train_swarm import mcnemar

    a = np.array([True] * 5 + [False] * 5)
    assert mcnemar(a, a) == {"p_value": 1.0, "only_a": 0, "only_b": 0}
    res = mcnemar(np.zeros(6, bool), np.ones(6, bool))
    assert res["only_b"] == 6 and res["p_value"] == pytest.approx(2 / 64)


def test_train_swarm_cli(tmp_path, monkeypatch):
    import json
    import sys

    from modular_sphere.tools import train_swarm

    argv = ["train_swarm", "--out-dir", str(tmp_path), "--spheres", "2", "--episodes", "4",
            "--val-episodes", "1", "--wm-epochs", "1", "--bc-epochs", "1", "--dagger-betas", "0.5",
            "--dagger-episodes", "1", "--explore-episodes", "1", "--ppo-iterations", "2",
            "--ppo-chunk", "1", "--eval-episodes", "1", "--select-episodes", "1",
            "--hidden", "16", "--device", "cpu", "--ensemble", "2"]  # fmt: skip
    monkeypatch.setattr(sys, "argv", argv)
    train_swarm.main()
    report = json.loads((tmp_path / "report.json").read_text())
    assert set(report["policies"]) == {"bc", "dagger", "ppo"}
    assert [r["iteration"] for r in report["ppo_log"]] == [1, 2]
    assert 0 <= report["policies"]["bc"]["speed_vs_expert"] <= 2 and report["expert_mean_steps"] > 0
    assert "mcnemar_dagger_vs_ppo" in report
    for name in ("train.npz", "val.npz", "wm1.pt", "bc.pt", "dagger.pt", "ppo.pt"):
        assert (tmp_path / name).exists()


def test_train_swarm_without_dagger_compares_bc(tmp_path):
    from modular_sphere.tools.train_swarm import train_swarm

    report = train_swarm(tmp_path, n_spheres=2, episodes=4, val_episodes=1, wm_epochs=1,
                         bc_epochs=1, dagger_betas=(), explore_episodes=1,
                         ppo_iterations=1, eval_episodes=1, select_episodes=1,
                         hidden=16, ensemble=1, device="cpu")  # fmt: skip
    assert set(report["policies"]) == {"bc", "ppo"}
    assert "mcnemar_bc_vs_ppo" in report


def test_collect_random_exploration_holds_random_actions():
    d = collect(1, n_spheres=2, seed=3, noise=0.0, random_prob=1.0)
    steps = d.valid[0]
    act, label = d.actions[0, steps], d.expert[0, steps]
    assert (act != label).any(-1).mean() > 0.9  # every sphere explores, labels stay expert
    assert set(np.unique(act[:, :, 3])) <= {-1.0, 1.0}
    held = (np.diff(act[:, 0, 0]) == 0).mean()
    assert held > 0.7  # segments of 5-20 steps


def test_ppo_in_world_model_ensemble(data):
    wms = [SwarmWorldModel(hidden=16, epochs=1, window=4, seed=s, device="cpu") for s in (0, 1)]
    for wm in wms:
        wm.fit(data)
    policy = SwarmPolicy(2, hidden=16, epochs=1, device="cpu")
    policy.fit(data)
    before = [p.detach().clone() for p in policy.net.parameters()]
    log = improve_with_ppo(policy, wms, data, iterations=2, horizon=4, batch_size=8, minibatch=8)
    assert len(log) == 2 and all(np.isfinite(m["return"]) and m["kl"] >= 0 for m in log)
    assert 0 <= log[-1]["success"] <= 1
    assert any((p != q).any() for p, q in zip(policy.net.parameters(), before, strict=True))
    single = improve_with_ppo(policy, wms[0], data, iterations=1, horizon=2, batch_size=4)
    assert np.isfinite(single[0]["critic"])


def test_bernoulli_kl_is_finite_when_saturated():
    from modular_sphere.swarm_torch import _bernoulli_kl

    kl = _bernoulli_kl(torch.tensor([0.0, 50.0, 3.0]), torch.tensor([0.0, -200.0, 3.0]))
    assert torch.isfinite(kl).all() and kl[0] == 0 and kl[2] == 0 and kl[1] > 100


def test_train_swarm_keeps_the_best_ppo_chunk(tmp_path, monkeypatch):
    from modular_sphere.tools import train_swarm as ts

    speeds = iter([0.9, 0.1])  # DAgger start, then a worse PPO chunk
    real = ts._summary

    def summary(ev, ref):
        out = real(ev, ref)
        return {**out, "speed_vs_expert": next(speeds, out["speed_vs_expert"])}

    monkeypatch.setattr(ts, "_summary", summary)
    report = ts.train_swarm(tmp_path, n_spheres=2, episodes=4, val_episodes=1, wm_epochs=1,
                            bc_epochs=1, dagger_betas=(), explore_episodes=1, ppo_iterations=1,
                            eval_episodes=1, select_episodes=1, hidden=16, ensemble=1,
                            device="cpu")  # fmt: skip
    assert report["ppo_log"][0]["select_speed"] == 0.1
    bc = ts.SwarmPolicy.load(str(tmp_path / "bc.pt"), "cpu")
    ppo = ts.SwarmPolicy.load(str(tmp_path / "ppo.pt"), "cpu")
    assert all(torch.equal(a, b) for a, b in zip(
        bc.net.state_dict().values(), ppo.net.state_dict().values(), strict=True))  # fmt: skip
