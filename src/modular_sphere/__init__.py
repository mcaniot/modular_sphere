"""MuJoCo simulation of a modular sphere robot and a world-model test bench."""

from importlib.metadata import version

from gymnasium.envs.registration import register

from modular_sphere.courses import LEVELS
from modular_sphere.env import MAX_EPISODE_STEPS, OBS_SLICES, SphereReachEnv

ENV_ID = "ModularSphere/Reach-v0"

register(
    id=ENV_ID, entry_point="modular_sphere.env:SphereReachEnv", max_episode_steps=MAX_EPISODE_STEPS
)

for _level, _spec in LEVELS.items():
    register(
        id=f"ModularSphere/Course-L{_level}-v0",
        entry_point="modular_sphere.courses:SphereCourseEnv",
        max_episode_steps=_spec.max_steps,
        kwargs={"level": _level},
    )

__all__ = ["ENV_ID", "OBS_SLICES", "SphereReachEnv", "__version__"]
__version__ = version("modular-sphere")  # single source: pyproject.toml
