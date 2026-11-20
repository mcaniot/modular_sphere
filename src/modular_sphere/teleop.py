"""Keyboard teleoperation in the MuJoCo viewer: arrow keys push the sphere, N draws a new goal."""

from __future__ import annotations

import time

import mujoco.viewer
import numpy as np

from modular_sphere.env import SphereReachEnv

# GLFW key codes -> world-frame force direction.
_ARROWS = {262: (1.0, 0.0), 263: (-1.0, 0.0), 265: (0.0, 1.0), 264: (0.0, -1.0)}
_KEY_N = 78
_HOLD_S = 0.25  # the passive viewer reports presses and repeats, not releases


def main(seed: int = 0) -> None:
    env = SphereReachEnv()
    env.reset(seed=seed)
    force = np.zeros(2)
    state = {"until": 0.0, "reset": False}

    def on_key(key: int) -> None:
        if key in _ARROWS:
            force[:] = _ARROWS[key]
            state["until"] = time.monotonic() + _HOLD_S
        elif key == _KEY_N:
            state["reset"] = True

    with mujoco.viewer.launch_passive(env.model, env.data, key_callback=on_key) as viewer:
        while viewer.is_running():
            tick = time.monotonic()
            if state["reset"]:
                env.reset()
                state["reset"] = False
            action = force if tick < state["until"] else np.zeros(2)
            _, _, success, _, info = env.step(action)
            if success:
                print(f"goal reached (distance {info['distance']:.3f} m)")
                env.reset()
            viewer.sync()
            time.sleep(max(0.0, env.dt - (time.monotonic() - tick)))
    env.close()
