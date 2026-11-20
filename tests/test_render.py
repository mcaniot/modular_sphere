import os
import subprocess
import sys

import imageio
import numpy as np
import pytest

from modular_sphere import SphereReachEnv, render
from modular_sphere.cli import main
from modular_sphere.render import make_renderer


@pytest.fixture(scope="module")
def env():
    env = SphereReachEnv()
    env.reset(seed=0)
    yield env
    env.close()


@pytest.mark.parametrize("camera", ["overview", "top", "track"])
def test_classic_frames(env, camera):
    renderer = make_renderer("classic", env.model, 160, 90)
    try:
        frame = renderer.render(env.data, camera)
    finally:
        renderer.close()
    assert frame.shape == (90, 160, 3) and frame.dtype == np.uint8
    assert frame.std() > 5  # a scene, not a blank image


def test_unknown_renderer(env):
    with pytest.raises(ValueError):
        make_renderer("raytracer", env.model, 16, 16)


def test_cli_record(tmp_path, capsys):
    out = tmp_path / "demo.mp4"
    main(["record", "--out", str(out), "--steps", "9", "--width", "160", "--height", "90"])
    assert "saved 10 frames" in capsys.readouterr().out
    frames = imageio.v2.mimread(out)
    assert len(frames) == 10 and frames[0].shape == (90, 160, 3)
    assert imageio.v2.get_reader(out).get_meta_data()["codec"] == "h264"


FILAMENT_SMOKE = """
from modular_sphere import SphereReachEnv
from modular_sphere import render
from modular_sphere.render import make_renderer
env = SphereReachEnv(); env.reset(seed=0)
r = make_renderer("filament", env.model, 160, 90)
img = r.render(env.data, "overview"); r.close()
print("FILAMENT", img.shape, float(img.std()))
"""


@pytest.mark.skipif(sys.version_info < (3, 12), reason="mujoco 4.0.1 Filament needs Python>=3.12")
@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="Filament needs an X display")
def test_filament_frame():
    # Subprocess: Filament owns a GL context and engine thread; keep it out of the test process.
    proc = subprocess.run([sys.executable, "-c", FILAMENT_SMOKE], capture_output=True, text=True,
                          timeout=120)  # fmt: skip
    assert proc.returncode == 0, proc.stderr[-2000:]
    line = next(ln for ln in proc.stdout.splitlines() if ln.startswith("FILAMENT"))
    assert "(90, 160, 3)" in line and float(line.split()[-1]) > 5


def test_filament_without_studio_raises(env, monkeypatch):
    def missing(name):
        raise ImportError(name)

    monkeypatch.setattr(render.importlib, "import_module", missing)
    with pytest.raises(RuntimeError, match="mujoco-studio"):
        make_renderer("filament", env.model, 16, 16)


def test_custom_numeric_default(env):
    assert render._custom_numeric(env.model, "no.such.numeric", 1.5) == 1.5


def test_record_keeps_going_after_success(tmp_path):
    # The goal-seeking policy reaches several goals in 600 steps; a new goal is drawn each time.
    n = render.record(tmp_path / "long.mp4", steps=600, width=32, height=32)
    assert n == 601
