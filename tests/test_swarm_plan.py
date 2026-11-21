import numpy as np
import pytest

from modular_sphere.swarm import STEP_X, SphereSwarmStepEnv

torch = pytest.importorskip("torch")

from modular_sphere.swarm_plan import (  # noqa: E402
    LANE_Y,
    EffectModel,
    SkillScheduler,
    base_spot,
    evaluate_schedule,
    lane_lines,
    plan,
    prior_executor,
    queue_spot,
    script_executor,
    wam_executor,
)
from modular_sphere.swarm_skills import MAX_SKILL_STEPS, SKILLS, SkillPrior, SkillValue  # noqa: E402
from modular_sphere.swarm_wam import SkillWAM  # noqa: E402

N_AGENTS = {"roll": 1, "climb": 2, "pull": 2}


class _DistEffect(EffectModel):
    """Durations growing with the distance of the first sphere to its target."""

    def __init__(self) -> None:
        super().__init__({})

    def duration(self, name, states, target):
        goal = target if name == "roll" else np.array([STEP_X, target[1]])
        return 40.0 + 50.0 * float(np.linalg.norm(states[0, :2] - goal))


def _starts(seed=200_000):
    env = SphereSwarmStepEnv(n_spheres=4)
    env.reset(seed=seed)
    return env._kinematics()[0], env.goal


def test_plan_prefers_parallel_lanes_and_respects_the_lane_limit():
    pos, goal = _starts()
    lanes, span = plan(_DistEffect(), pos, goal[:2])
    assert len(lanes) == 2 and span > 0
    assert sorted(i for ln in lanes for i in [ln.base, *ln.climbers]) == [0, 1, 2, 3]
    assert lanes[0].y < lanes[1].y
    serial, serial_span = plan(_DistEffect(), pos, goal[:2], max_lanes=1)
    assert len(serial) == 1 and len(serial[0].climbers) == 3 and serial_span > span
    xs = [pos[c, 0] for c in serial[0].climbers]
    assert xs == sorted(xs, reverse=True)  # closest to the step first
    with pytest.raises(ValueError, match="no plan"):
        plan(_DistEffect(), pos[:1], goal[:2])


def test_lane_lines_and_spots_stay_in_range():
    for goal_y in (-1.0, 0.0, 1.0):
        ys = lane_lines(goal_y, (-0.45, 0.45))
        assert np.isclose(ys[1] - ys[0], 0.9) and ys.min() >= -LANE_Y and ys.max() <= LANE_Y
    b, q0, q1 = base_spot(0.2), queue_spot(0.2, 0), queue_spot(0.2, 1)
    assert b[0] < STEP_X and q1[0] < q0[0] < b[0] and q0[1] == 0.2


def test_effect_model_durations_are_bounded():
    effect = EffectModel({s: SkillValue(N_AGENTS[s]) for s in SKILLS})
    assert 0.0 <= effect.duration("roll", np.zeros((1, 3)), np.array([1.0, 0.0])) <= MAX_SKILL_STEPS
    two = np.zeros((2, 9))
    assert 0.0 <= effect.duration("climb", two, np.array([0.0, 0.1])) <= MAX_SKILL_STEPS


def test_scripted_schedule_solves_the_swarm_in_parallel():
    sched = SkillScheduler(4, _DistEffect(), script_executor())
    ev = evaluate_schedule(sched, [200_000])
    assert ev.success[0] and ev.lanes[0] == 2 and ev.restarts[0] == 0
    assert 0.0 < ev.moving[0] <= 1.0 and ev.predicted[0] > 0
    names = [e[1] for e in sched.events]
    assert names.count("climb") == 2 and names.count("pull") == 2
    assert all(stage == "gather" for stage in sched.stage)
    serial = SkillScheduler(4, _DistEffect(), script_executor(), max_lanes=1)
    evaluate_schedule(serial, [200_000])
    assert [e[1] for e in serial.events].count("climb") == 3  # two gather before the pull


def test_learned_executors_run_and_stuck_skills_restart():
    priors = {s: SkillPrior(N_AGENTS[s]) for s in SKILLS}
    sched = SkillScheduler(4, _DistEffect(), prior_executor(priors), max_lanes=1)
    ev = evaluate_schedule(sched, [200_001])
    assert not ev.success[0] and ev.restarts[0] > 0 and ev.lanes[0] == 1
    wams = {s: SkillWAM(N_AGENTS[s], history=2, chunk=2, width=16, layers=1, heads=2)
            for s in SKILLS}  # fmt: skip
    sched = SkillScheduler(4, _DistEffect(), wam_executor(wams))
    env = SphereSwarmStepEnv(n_spheres=4)
    obs, _ = env.reset(seed=0)
    spheres = np.concatenate(env._kinematics(), 1)
    sched.reset(spheres[:, :3], env.goal)
    act = sched.act([obs[a] for a in env.possible_agents], spheres, env.links)
    assert act.shape == (4, 4) and np.all(np.abs(act) <= 1.0)
