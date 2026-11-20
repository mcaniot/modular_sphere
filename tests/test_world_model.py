import numpy as np
import pytest

from modular_sphere.cli import main
from modular_sphere.dataset import TrajectoryDataset, collect
from modular_sphere.world_model import (
    ConstantVelocityModel,
    LinearDeltaModel,
    MuJoCoOracleModel,
    WorldModel,
    evaluate,
)


@pytest.fixture(scope="module")
def train():
    return collect(num_episodes=20, steps=100, seed=0)


@pytest.fixture(scope="module")
def evald():
    return collect(num_episodes=5, steps=100, seed=1000)


def test_dataset_shapes_and_roundtrip(tmp_path, evald):
    assert evald.obs.shape == (5, 101, 11) and evald.actions.shape == (5, 100, 2)
    assert np.all(np.abs(evald.actions) <= 1.0)
    path = tmp_path / "d.npz"
    evald.save(path)
    loaded = TrajectoryDataset.load(path)
    np.testing.assert_array_equal(loaded.obs, evald.obs)
    assert loaded.metadata == evald.metadata


def test_collect_is_deterministic():
    a, b = collect(2, 20, seed=7), collect(2, 20, seed=7)
    np.testing.assert_array_equal(a.obs, b.obs)


def test_dataset_rejects_inconsistent_shapes():
    with pytest.raises(ValueError):
        TrajectoryDataset(
            np.zeros((1, 3, 11)), np.zeros((1, 3, 2)), np.zeros((1, 3)), np.zeros((1, 3), bool)
        )


def test_oracle_is_exact(evald):
    metrics = evaluate(MuJoCoOracleModel(), evald, horizons=[1, 20])
    assert metrics[20]["all"] < 1e-6


def test_baselines_rank(train, evald):
    linear = LinearDeltaModel()
    linear.fit(train)
    constant = ConstantVelocityModel(evald.metadata["dt"])
    assert isinstance(linear, WorldModel) and isinstance(constant, WorldModel)
    lin = evaluate(linear, evald, [1, 10])
    cv = evaluate(constant, evald, [1, 10])
    assert lin[10]["all"] < cv[10]["all"]
    assert lin[1]["all"] <= lin[10]["all"]


def test_evaluate_checks_horizon_and_shape(evald):
    with pytest.raises(ValueError):
        evaluate(ConstantVelocityModel(0.05), evald, [evald.num_steps + 1])

    class Broken:
        def rollout(self, obs0, actions):
            return obs0

    with pytest.raises(ValueError):
        evaluate(Broken(), evald, [5])


def test_cli_collect_and_eval(tmp_path, capsys):
    data = str(tmp_path / "d.npz")
    main(["collect", "--out", data, "--episodes", "2", "--steps", "30"])
    main(["eval-wm", "--model", "linear", "--train-data", data, "--eval-data", data,
          "--horizons", "1", "5"])  # fmt: skip
    assert '"5"' in capsys.readouterr().out
    with pytest.raises(SystemExit):
        main(["eval-wm", "--model", "linear", "--eval-data", data])
