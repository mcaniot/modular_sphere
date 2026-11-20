"""Offscreen renderers (classic OpenGL or Filament PBR) and MP4 recording of rollouts."""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Protocol

import imageio
import mujoco
import numpy as np
from numpy.typing import NDArray

from modular_sphere.dataset import exploration_action
from modular_sphere.env import SphereReachEnv

Frame = NDArray[np.uint8]
RENDERERS = ("classic", "filament")


class FrameRenderer(Protocol):
    def render(self, data: mujoco.MjData, camera: str) -> Frame: ...

    def close(self) -> None: ...


class ClassicRenderer:
    """MuJoCo's OpenGL renderer (EGL / OSMesa / GLFW according to ``MUJOCO_GL``)."""

    def __init__(self, model: mujoco.MjModel, width: int, height: int) -> None:
        self._renderer = mujoco.Renderer(model, height=height, width=width)

    def render(self, data: mujoco.MjData, camera: str) -> Frame:
        self._renderer.update_scene(data, camera)
        return self._renderer.render().copy()

    def close(self) -> None:
        self._renderer.close()


class FilamentRenderer:
    """MuJoCo 4 Filament physically based renderer, offscreen through OpenGL.

    Requires the ``mujoco-studio`` package shipped with ``mujoco>=4`` and Python >= 3.12 (mujoco
    4.0.1's Filament module imports ``collections.abc.Buffer``, although the wheel allows 3.11),
    plus an X display: a GPU session, or Mesa llvmpipe under ``xvfb-run`` on CPU-only machines.
    Must be created, used and closed on the same thread (Filament engine affinity).
    """

    def __init__(self, model: mujoco.MjModel, width: int, height: int) -> None:
        try:
            importlib.import_module("mujoco.studio.window")  # registers Filament resource providers
            from mujoco import mjrf
            from mujoco.rendering.filament import model_scene, renderer
        except ImportError as err:
            raise RuntimeError(
                "Filament rendering needs mujoco>=4 with mujoco-studio (Python >= 3.12)"
            ) from err
        self._model = model
        self._view_cls = renderer.View
        self._ctx = mjrf.Context(
            mjrf.ContextConfig(graphics_api=mjrf.GraphicsApi.GRAPHICS_API_OPENGL)
        )
        self._renderer = renderer.Renderer(self._ctx)
        self._scene = model_scene.ModelScene(self._ctx, model, self._renderer.scene("scene"))
        self._pixels = np.zeros((height, width, 3), dtype=np.uint8)
        self._renderer.target("out", self._pixels)
        self._camera = mujoco.MjvCamera()
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        # Enable Filament headlight only if classic headlight is active and no positive-intensity
        # lights exist (classic-authored scene, fallback mode). Filament expects the application to
        # add the headlight in lux, like MuJoCo Studio does (ModelLights::GetHeadlightIntensity).
        self._headlight = bool(model.vis.headlight.active) and not np.any(model.light_intensity > 0)
        self._headlight_lux = _custom_numeric(
            model, "filament.fallback.head_light_intensity", 40000.0
        )

    def render(self, data: mujoco.MjData, camera: str) -> Frame:
        self._camera.fixedcamid = self._model.camera(camera).id
        self._scene.update(data)
        gl_camera = mujoco.mjv_camera2GLCamera(self._model, data, self._camera)
        view = self._view_cls(scene="scene", target="out", camera=gl_camera,
                              headlight=self._headlight, headlight_intensity=self._headlight_lux,
                              headlight_color=tuple(self._model.vis.headlight.diffuse))  # fmt: skip
        self._renderer.render([view])
        return self._pixels.copy()

    def close(self) -> None:
        # Release scene, then renderer, then context (reverse creation order), on this thread.
        del self._scene, self._renderer, self._ctx


def _custom_numeric(model: mujoco.MjModel, name: str, default: float) -> float:
    try:
        return float(model.numeric(name).data[0])
    except KeyError:
        return default


def make_renderer(name: str, model: mujoco.MjModel, width: int, height: int) -> FrameRenderer:
    if name == "classic":
        return ClassicRenderer(model, width, height)
    if name == "filament":
        return FilamentRenderer(model, width, height)
    raise ValueError(f"unknown renderer {name!r}, expected one of {RENDERERS}")


def write_mp4(frames: list[Frame], path: str | Path, fps: float) -> None:
    """Encode RGB frames to H.264 with imageio and the ffmpeg binary bundled by imageio-ffmpeg."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    # macro_block_size=2 keeps the requested size (yuv420p codec requires even dimensions).
    imageio.v2.mimwrite(path, frames, fps=fps, codec="libx264", quality=None,
                        macro_block_size=2, output_params=["-crf", "18"])  # fmt: skip


def record(path: str | Path, renderer: str = "classic", camera: str = "overview",
           policy: str = "goal", seed: int = 0, steps: int = 200,
           width: int = 1280, height: int = 720) -> int:  # fmt: skip
    """Roll out an exploration policy (``goal`` or ``random``) and save an MP4; returns frames."""
    env = SphereReachEnv()
    frame_renderer = make_renderer(renderer, env.model, width, height)
    try:
        obs, _ = env.reset(seed=seed)
        rng = np.random.default_rng(seed)
        noise = np.zeros(env.action_space.shape)
        frames = [frame_renderer.render(env.data, camera)]
        for _ in range(steps):
            action = exploration_action(rng, obs, policy == "goal", noise)
            obs, _, success, _, _ = env.step(action)
            if success:  # keep the demo going: new goal, the sphere keeps rolling
                obs = env.sample_goal()
            frames.append(frame_renderer.render(env.data, camera))
    finally:
        frame_renderer.close()
        env.close()
    write_mp4(frames, path, fps=1.0 / env.dt)
    return len(frames)
