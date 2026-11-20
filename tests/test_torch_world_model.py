import numpy as np
import pytest

pytest.importorskip("torch")

from modular_sphere.cli import main
from modular_sphere.dataset import collect
from modular_sphere.torch_world_model import MLPWorldModel
from modular_sphere.world_model import ConstantVelocityModel, WorldModel, evaluate


@pytest.fixture(scope="module")
def fitted():
    model = MLPWorldModel(hidden=128, epochs=40, device="cpu")
    model.fit(collect(num_episodes=20, steps=100, seed=0))
    return model


def test_mlp_learns_and_beats_constant_velocity(fitted):
    assert isinstance(fitted, WorldModel)
    assert fitted.train_losses[-1] < 0.5 * fitted.train_losses[0]
    evald = collect(num_episodes=5, steps=100, seed=1000)
    mlp = evaluate(fitted, evald, [1, 10])
    cv = evaluate(ConstantVelocityModel(evald.metadata["dt"]), evald, [1, 10])
    assert mlp[1]["all"] < cv[1]["all"] and mlp[10]["all"] < cv[10]["all"]


def test_mlp_rollout_shape_and_checkpoint(fitted, tmp_path):
    rng = np.random.default_rng(0)
    obs0, actions = rng.normal(size=(3, 11)), rng.uniform(-1, 1, size=(3, 7, 2))
    pred = fitted.rollout(obs0, actions)
    assert pred.shape == (3, 7, 11) and pred.dtype == np.float64
    fitted.save(tmp_path / "wm.pt")
    np.testing.assert_allclose(MLPWorldModel.load(tmp_path / "wm.pt", "cpu").rollout(obs0, actions),
                               pred, atol=1e-6)  # fmt: skip


def test_mlp_requires_fit():
    with pytest.raises(RuntimeError):
        MLPWorldModel(device="cpu").rollout(np.zeros((1, 11)), np.zeros((1, 2, 2)))


def test_cli_mlp(tmp_path, capsys):
    data = str(tmp_path / "d.npz")
    main(["collect", "--out", data, "--episodes", "2", "--steps", "30"])
    main(["eval-wm", "--model", "mlp", "--train-data", data, "--eval-data", data,
          "--horizons", "1", "5"])  # fmt: skip
    assert '"5"' in capsys.readouterr().out
