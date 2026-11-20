"""Error paths and fallbacks of the CLI, env and world models."""

import sys

import numpy as np
import pytest

from modular_sphere import teleop
from modular_sphere.cli import _build_model, main
from modular_sphere.dataset import collect
from modular_sphere.env import SphereReachEnv
from modular_sphere.world_model import LinearDeltaModel


@pytest.fixture(scope="module")
def eval_npz(tmp_path_factory):
    path = tmp_path_factory.mktemp("data") / "eval.npz"
    collect(num_episodes=2, steps=10, seed=1).save(path)
    return str(path)


def test_cli_version_matches_package_metadata(capsys):
    from importlib.metadata import version

    with pytest.raises(SystemExit):
        main(["--version"])
    assert capsys.readouterr().out.strip() == f"modular-sphere {version('modular-sphere')}"


def test_unknown_model_lists_builtins():
    with pytest.raises(SystemExit, match=r"unknown model 'nope'.*'linear'"):
        _build_model("nope", 0.05)


def test_mlp_without_torch(monkeypatch):
    monkeypatch.setitem(sys.modules, "modular_sphere.torch_world_model", None)
    with pytest.raises(SystemExit, match="needs PyTorch"):
        _build_model("mlp", 0.05)


def test_cli_teleop_dispatch(monkeypatch):
    seeds = []
    monkeypatch.setattr(teleop, "main", seeds.append)
    main(["teleop", "--seed", "3"])
    assert seeds == [3]


def test_cli_eval_rejects_non_world_model(eval_npz):
    with pytest.raises(SystemExit, match="does not implement rollout"):
        main(["eval-wm", "--model", "modular_sphere.env:SphereReachEnv", "--eval-data", eval_npz])


def test_cli_eval_learning_model_needs_train_data(eval_npz):
    with pytest.raises(SystemExit, match="needs --train-data"):
        main(["eval-wm", "--model", "linear", "--eval-data", eval_npz])


def test_cli_eval_model_without_fit(eval_npz, capsys):
    main(["eval-wm", "--model", "constant-velocity", "--eval-data", eval_npz, "--horizons", "1"])
    assert '"1"' in capsys.readouterr().out


def test_set_from_obs_checks_shape():
    env = SphereReachEnv()
    with pytest.raises(ValueError, match="obs must have shape"):
        env.set_from_obs(np.zeros(3))
    env.close()


def test_linear_model_requires_fit():
    with pytest.raises(RuntimeError, match="fit must be called"):
        LinearDeltaModel().rollout(np.zeros((1, 11)), np.zeros((1, 2, 2)))


def test_mlp_device_fallbacks(monkeypatch):
    torch = pytest.importorskip("torch")
    from modular_sphere.torch_world_model import MLPWorldModel

    with pytest.raises(RuntimeError, match="nothing to save"):
        MLPWorldModel(device="cpu").save("unused.pt")
    monkeypatch.setattr(torch.cuda, "is_available", lambda: False)
    assert MLPWorldModel(device="tpu").device.type == "cpu"
    assert MLPWorldModel(device="cuda").device.type == "cpu"
