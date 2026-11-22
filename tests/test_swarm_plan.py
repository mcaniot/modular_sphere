import numpy as np
import pytest

from modular_sphere.swarm import STEP_H, STEP_X, Course, SphereSwarmStepEnv

torch = pytest.importorskip("torch")

from modular_sphere.swarm_plan import (  # noqa: E402
    DYNAMICS_FRICTION,
    DYNAMICS_MASS,
    LANE_Y,
    QUEUE_SIDE,
    EffectModel,
    Level,
    SkillScheduler,
    base_spot,
    course_levels,
    evaluate_schedule,
    lane_lines,
    perturb_dynamics,
    plan,
    prior_executor,
    queue_spot,
    script_executor,
    segments_to_data,
    wam_executor,
)
from modular_sphere.swarm_skills import MAX_SKILL_STEPS, SKILLS, SkillPrior, SkillValue  # noqa: E402
from modular_sphere.swarm_wam import SkillWAM  # noqa: E402

N_AGENTS = {"roll": 1, "climb": 2, "pull": 2}


class _DistEffect(EffectModel):
    """Durations growing with the distance of the first sphere to its target."""

    def __init__(self) -> None:
        super().__init__({})

    def duration(self, name, states, target, level=None):
        level = Level() if level is None else level
        goal = target if name == "roll" else np.array([level.face, target[1]])
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
    spheres = env.spheres()
    sched.reset(spheres[:, :3], env.goal)
    act = sched.act([obs[a] for a in env.possible_agents], spheres, env.links)
    assert act.shape == (4, 4) and np.all(np.abs(act) <= 1.0)


def test_labelled_schedule_records_chain_segments_as_skill_data():
    sched = SkillScheduler(4, _DistEffect(), script_executor(), labeler=script_executor(),
                           noise=0.1, seed=3)  # fmt: skip
    evaluate_schedule(sched, [200_000])
    assert len(sched.segments) == len(sched.events)
    for seg in sched.segments:
        t, k = seg.labels.shape[:2]
        assert seg.spheres.shape == (t + 1, k, 9) and seg.links.shape == (t + 1, k, k)
        assert k == len(seg.call.agents) and np.isfinite(seg.labels).all()
    data, calls = segments_to_data(sched.segments, "climb")
    assert len(calls) == data.spheres.shape[0] == 2 and calls[0].agents == (0, 1)
    assert data.expert.shape == (2, MAX_SKILL_STEPS, 2, 4) and data.success.all()
    assert data.valid.sum() == sum(len(s.labels) for s in sched.segments if s.call.name == "climb")
    roll, _ = segments_to_data(sched.segments, "roll")
    assert np.all(roll.spheres[:, :, 1] == 0.0)  # single-sphere calls pad the second slot
    priors = {s: SkillPrior(N_AGENTS[s]) for s in SKILLS}
    stuck = SkillScheduler(4, _DistEffect(), prior_executor(priors), max_lanes=1,
                           labeler=script_executor())  # fmt: skip
    evaluate_schedule(stuck, [200_001])
    assert any(not s.success for s in stuck.segments)


def test_course_levels_frame_the_skill_calls_of_each_step():
    course = Course((-0.3, 0.5), (0.12, 0.14))
    low, high = course_levels(course)
    assert low == Level(-0.3, 0.0, 0.12) and high == Level(0.5, 0.12, 0.14, -0.3)
    roll = high.call("roll", (0,), np.array([0.2, 0.0]))
    assert roll.floor == 0.12 - STEP_H and roll.face == 0.5 and roll.back == -0.3
    climb = high.call("climb", (0, 1), np.array([0.0, 0.1]))
    assert climb.floor == 0.12 and climb.rise == 0.14
    assert low.call("roll", (0,), np.zeros(2)).floor == 0.0
    assert base_spot(0.0, level=high)[2] == 0.12 + 0.1
    assert queue_spot(0.0, 5, level=high)[0] >= -0.3 + 0.1 + 0.05 - 1e-9
    for y in (-0.4, 0.4):  # a floor too short for the queue: climbers wait side by side
        spots = np.array([queue_spot(y, q, level=high) for q in range(4)])
        gaps = np.linalg.norm(spots[:, None, :2] - spots[None, :, :2], axis=-1)
        assert gaps[~np.eye(4, dtype=bool)].min() >= 0.2 - 1e-9
        assert np.abs(spots[:, 1]).max() <= LANE_Y + QUEUE_SIDE + 1e-9


def test_scripted_schedule_climbs_a_two_step_course_level_by_level():
    course = Course((-0.3, 0.5), (0.15, 0.15))
    sched = SkillScheduler(4, _DistEffect(), script_executor())
    ev = evaluate_schedule(sched, [300_000], lambda seed: course)
    assert ev.success[0] and sched.level == 1 and ev.restarts[0] == 0
    assert [e[1] for e in sched.events].count("pull") == 2 * ev.lanes[0]


def _platform_scheduler():
    sched = SkillScheduler(4, _DistEffect(), script_executor())
    pos = np.array([[-1.0, y, 0.1] for y in (-0.9, -0.3, 0.3, 0.9)])
    sched.reset(pos, np.array([1.4, 0.0, 0.4]), Course((-0.3, 0.5), (0.15, 0.15)))
    sched.runs.clear()
    spheres = np.zeros((4, 9))
    spheres[:, :3] = pos
    return sched, spheres, sched.top.floor + 0.1


def test_crossing_slot_rolls_swap_targets():
    sched, spheres, top = _platform_scheduler()
    a, b = np.array([0.3, 0.6]), np.array([0.3, -0.3])
    sched.slots = [a, b]
    spheres[0, :3] = (0.3, -0.25, top)
    sched._gather(0, spheres)
    assert np.allclose(sched.up[0], b)
    spheres[0, :2], spheres[1, :3] = (0.3, 0.6), (0.3, -0.3, top)  # sphere 0 got pushed across
    sched._gather(1, spheres)
    assert np.allclose(sched.up[0], a) and np.allclose(sched.up[1], b)
    assert np.allclose(sched.runs[0].call.target, a) and not sched.slots


def test_fallen_spheres_are_pulled_back_up_by_free_platform_spheres():
    sched, spheres, top = _platform_scheduler()
    spheres[0, :3], spheres[1, :3] = (0.0, 0.3, top), (0.1, -0.5, top)
    sched._gather(0, spheres)
    sched._gather(1, spheres)
    free = len(sched.slots)
    spheres[0, :3] = (-0.45, 0.3, 0.1)  # sphere 0 fell back in front of the face
    sched._rescue(spheres)
    assert sched.rescues == {0: 1} and sched.runs[0] is sched.runs[1]
    assert sched.runs[0].call.name == "pull" and len(sched.slots) == free + 2 and not sched.up
    spheres[1, :3] = (-0.45, -0.5, 0.1)  # the helper fell too: nobody left on the platform
    sched._rescue(spheres)
    assert sched.rescues == {0: -1, 1: -1} and not sched.runs
    spheres[2, :3] = (0.2, 0.0, top)
    sched._gather(2, spheres)
    sched._rescue(spheres)
    assert sorted(sched.rescues.values()) == [-1, 2]
    i = next(k for k, h in sched.rescues.items() if h == 2)
    sched.runs.pop(i)  # the pull ended: both go to slots
    sched.runs.pop(2)
    spheres[i, :3] = (0.15, spheres[i, 1], top)
    sched._rescue(spheres)
    ((j, h),) = sched.rescues.items()  # the other one gets a helper among the two
    assert j == 1 - i and h in (i, 2) and set(sched.up) == {i, 2} - {h}


def test_perturbed_dynamics_stay_in_range_and_episodes_still_run():
    env, ref = SphereSwarmStepEnv(n_spheres=4), SphereSwarmStepEnv(n_spheres=4)
    perturb_dynamics(env, 7)
    body = ref.model.body_mass > 0
    mass = env.model.body_mass[body] / ref.model.body_mass[body]
    fric = env.model.geom_friction[:, 0] / ref.model.geom_friction[:, 0]
    assert np.ptp(mass) < 1e-9 and DYNAMICS_MASS[0] <= mass[0] <= DYNAMICS_MASS[1]
    assert np.ptp(fric) < 1e-9 and DYNAMICS_FRICTION[0] <= fric[0] <= DYNAMICS_FRICTION[1]
    free = ref.model.body_invweight0[:, 0] > 0
    weight = env.model.body_invweight0[free, 0] / ref.model.body_invweight0[free, 0]
    np.testing.assert_allclose(weight, 1.0 / mass[0])  # mass-derived constants follow the masses
    sched = SkillScheduler(4, _DistEffect(), script_executor())
    ev = evaluate_schedule(sched, [200_000], dynamics=perturb_dynamics)
    assert ev.steps[0] > 0
