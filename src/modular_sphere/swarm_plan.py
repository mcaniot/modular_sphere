"""Skill scheduler: the swarm climb composed from per-skill controllers running in parallel.

A plan splits the N spheres into ``lanes``. Each lane has a line ``y``, a base and its climbers:
everyone first rolls into the lane (base just before the step face, climbers queued behind), the
climbers then climb over the base one at a time, the last one stays at the edge and pulls the base
up, and every sphere on the platform rolls to a gathering slot. Lanes run in parallel, so two
lanes of two spheres need one climb and one pull each instead of three climbs and a pull.

The plan is chosen by search over role assignments, lane counts and lane lines, scoring each plan
by its makespan predicted with skill effect models: the steps-to-go models (``SkillValue``)
evaluated at each skill's nominal start (Liang et al., 2022, skill effect models composed by a
search). The climb order inside a lane is replanned from the true state at every climb start.
Every skill call is executed by an interchangeable executor: the scripted ``SkillController``, a
cloned ``SkillPrior`` or a ``SkillWAM``. Skills that exceed ``MAX_SKILL_STEPS`` are restarted.

On a ``Course`` of several steps, each step is a ``Level`` (its face, floor and rise, given to
every skill call as its frame): the swarm plans and runs one level, waits on a line in front of
the next face (the expert's waiting slots), then replans the next level from the true state.
"""

from __future__ import annotations

import itertools
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field, replace

import numpy as np
import torch
from numpy.typing import NDArray

from modular_sphere.swarm import (
    ACT_DIM,
    STEP_H,
    STEP_X,
    Course,
    SphereSwarmStepEnv,
    SwarmClimbPolicy,
    obs_slices,
)
from modular_sphere.swarm_data import SwarmDataset
from modular_sphere.swarm_rl import MOVING_SPEED
from modular_sphere.swarm_skills import (
    MAX_SKILL_STEPS,
    SkillCall,
    SkillController,
    SkillPrior,
    SkillValue,
    skill_features,
)
from modular_sphere.swarm_wam import SkillWAM, WAMController

Array = NDArray[np.float64]
Executor = Callable[[list[Array], Array, NDArray[np.bool_]], Array]
ExecutorFactory = Callable[[SkillCall, int], Executor]
LANE_Y = 0.9  # lane lines stay inside the skills' training range [m]
LANE_OFFSETS = {1: ((0.0,),), 2: ((-0.3, 0.3), (-0.45, 0.45)), 3: ((-0.6, 0.0, 0.6),)}
QUEUE_GAP = 0.45  # first queued climber behind the base [m] ...
QUEUE_PITCH = 0.3  # ... and spacing of the next ones
QUEUE_SIDE = 0.25  # side spacing of climbers queued on a floor too short for the queue [m]
DYNAMICS_MASS = (0.7, 1.5)  # domain randomisation: sphere mass factor ...
DYNAMICS_FRICTION = (0.5, 1.5)  # ... and sliding friction factor


@dataclass
class Lane:
    """A base and the climbers that go over it on the line ``y``; the last climber pulls."""

    y: float
    base: int
    climbers: list[int]


@dataclass(frozen=True)
class Level:
    """One step of a course as a skill frame (defaults: the training step); ``back`` is the x
    where the floor in front of the face starts (the previous face)."""

    face: float = STEP_X
    floor: float = 0.0
    rise: float = STEP_H
    back: float = -np.inf

    def call(self, name: str, agents: tuple[int, ...], target: Array) -> SkillCall:
        """Skill call in this frame; a roll on a raised floor is framed as a roll on the training
        platform (floor ``STEP_H`` below the surface), the roll data's two surfaces."""
        target = np.asarray(target, dtype=np.float64)
        floor = self.floor - STEP_H if name == "roll" and self.floor > 0 else self.floor
        return SkillCall(name, agents, target, self.face, floor, self.rise, self.back)


def course_levels(course: Course) -> list[Level]:
    """The levels of a course, lowest first."""
    backs = (-np.inf, *course.faces[:-1])
    return [
        Level(f, course.floor(k), h, b)
        for k, (f, h, b) in enumerate(zip(course.faces, course.rises, backs, strict=True))
    ]


def base_spot(y: float, radius: float = 0.1, level: Level = Level()) -> Array:
    """Where the base waits before its first climb, inside the climb initiation set."""
    return np.array([level.face - radius - 0.11, y, level.floor + radius])


def queue_spot(y: float, q: int, radius: float = 0.1, level: Level = Level()) -> Array:
    """Where the ``q``-th climber of a lane waits, on the level's floor; when the floor is too
    short for the queue, climbers wait at its back edge, side by side across the lane."""
    spot = base_spot(y, radius, level) - (QUEUE_GAP + QUEUE_PITCH * q, 0.0, 0.0)
    back = level.back + radius + 0.05
    if spot[0] < back:
        inward = -1.0 if y > 0 else 1.0  # first towards the middle of the arena
        side = QUEUE_SIDE * ((q + 1) // 2) * (inward if q % 2 else -inward)
        spot[:2] = back, np.clip(y + side, -LANE_Y - QUEUE_SIDE, LANE_Y + QUEUE_SIDE)
    return spot


class EffectModel:
    """Skill durations predicted by the steps-to-go models at a skill's start state."""

    def __init__(self, values: dict[str, SkillValue]) -> None:
        self.values = values

    @torch.no_grad()
    def duration(self, name: str, states: Array, target: Array, level: Level = Level()) -> float:
        """Predicted steps of skill ``name`` from the involved spheres' states (k, 3) or (k, 9)
        (positions only: at rest) towards ``target`` (roll: xy; climb, pull: (0, lane y))."""
        value = self.values[name]
        x = np.zeros((len(states), 9))
        x[:, : states.shape[1]] = states
        t = torch.as_tensor(x, dtype=torch.float32, device=value.mean.device)
        links = torch.zeros(len(x), len(x), dtype=torch.bool, device=t.device)
        call = level.call(name, tuple(range(len(x))), target)
        return float(value(skill_features(call, t, links)))


def lane_makespan(effect: EffectModel, lane: Lane, pos: Array, goal_xy: Array,
                  radius: float = 0.1, level: Level = Level()) -> float:  # fmt: skip
    """Predicted steps until the lane's base reaches ``goal_xy`` on the level's platform."""
    y, r, lv = lane.y, radius, level
    line = np.array([0.0, y])
    face = np.array([lv.face - r - 0.01, y, lv.floor + r])
    edge = np.array([lv.face + 0.1, y, lv.floor + lv.rise + r])
    t = effect.duration("roll", pos[[lane.base]], base_spot(y, r, lv)[:2], lv)
    for q, c in enumerate(lane.climbers):
        t = max(t, effect.duration("roll", pos[[c]], queue_spot(y, q, r, lv)[:2], lv))
        base = base_spot(y, r, lv) if q == 0 else face
        t += effect.duration("climb", np.stack([queue_spot(y, q, r, lv), base]), line, lv)
    t += effect.duration("pull", np.stack([edge, face]), line, lv)
    top = Level(lv.face, lv.floor + lv.rise)  # the final roll is on the platform
    return t + effect.duration("roll", edge[None], goal_xy, top)


def _set_partitions(items: list[int]) -> Iterator[list[list[int]]]:
    if not items:
        yield []
        return
    first, rest = items[0], items[1:]
    for p in _set_partitions(rest):
        yield [[first], *p]
        for i in range(len(p)):
            yield [*p[:i], [first, *p[i]], *p[i + 1 :]]


def lane_lines(goal_y: float, offsets: tuple[float, ...]) -> Array:
    """Lane lines around the goal line, shifted as a block to stay within ``LANE_Y``."""
    ys = goal_y + np.asarray(offsets)
    ys -= max(ys.max() - LANE_Y, 0.0)
    ys += max(-LANE_Y - ys.min(), 0.0)
    return ys


def plan(effect: EffectModel, pos: Array, goal_xy: Array, radius: float = 0.1,
         max_lanes: int | None = None, level: Level = Level(),
         ) -> tuple[list[Lane], float]:  # fmt: skip
    """Plan of minimum predicted makespan over lane partitions (blocks of two spheres or more,
    sorted by mean y), bases and lane lines; climbers ordered closest to the step first. Lanes
    end at ``goal_xy`` on the level's platform."""
    n = len(pos)
    k_max = n // 2 if max_lanes is None else min(max_lanes, n // 2)
    best: tuple[list[Lane], float] | None = None
    for blocks in _set_partitions(list(range(n))):
        if len(blocks) > k_max or min(len(b) for b in blocks) < 2:
            continue
        blocks = sorted(blocks, key=lambda b: float(pos[b, 1].mean()))
        for offsets in LANE_OFFSETS.get(len(blocks), ()):
            ys = lane_lines(float(goal_xy[1]), offsets)
            for bases in itertools.product(*blocks):
                lanes = [
                    Lane(float(y), b, sorted((i for i in blk if i != b), key=lambda i: -pos[i, 0]))
                    for y, blk, b in zip(ys, blocks, bases, strict=True)
                ]
                span = max(lane_makespan(effect, ln, pos, goal_xy, radius, level) for ln in lanes)
                if best is None or span < best[1]:
                    best = (lanes, span)
    if best is None:
        raise ValueError(f"no plan for {n} spheres with at most {k_max} lanes")
    return best


def script_executor(radius: float = 0.1) -> ExecutorFactory:
    """Executes skill calls with the scripted ``SkillController``."""

    def make(call: SkillCall, n: int) -> Executor:
        ctrl = SkillController(call, n, radius)

        def act(obs: list[Array], spheres: Array, links: NDArray[np.bool_]) -> Array:
            out = np.zeros((n, ACT_DIM))
            for i, a in ctrl.act(obs).items():
                out[i] = a
            return out

        return act

    return make


def prior_executor(priors: dict[str, SkillPrior]) -> ExecutorFactory:
    """Executes skill calls with cloned skill policies (``SkillPrior``)."""

    def make(call: SkillCall, n: int) -> Executor:
        prior = priors[call.name]
        dev = prior.mean.device

        @torch.no_grad()
        def act(obs: list[Array], spheres: Array, links: NDArray[np.bool_]) -> Array:
            x = torch.as_tensor(spheres, dtype=torch.float32, device=dev)
            f = skill_features(call, x, torch.as_tensor(links, device=dev))
            out = np.zeros((n, ACT_DIM))
            out[list(call.agents)] = prior(f).cpu().numpy()
            out[:, 3] = np.where(out[:, 3] > 0, 1.0, -1.0)
            return out

        return act

    return make


def wam_executor(wams: dict[str, SkillWAM]) -> ExecutorFactory:
    """Executes skill calls with per-skill world-action models (newest chunk, no ensembling)."""

    def make(call: SkillCall, n: int) -> Executor:
        ctrl = WAMController(wams[call.name], call, n, ensemble=None)
        return lambda obs, spheres, links: ctrl.act(spheres, links)

    return make


@dataclass
class Segment:
    """One executed skill call, involved spheres only: states (T + 1, k, 9), links (T + 1, k, k),
    labels of the ``labeler`` (T, k, 4) and whether the call reached its termination event."""

    call: SkillCall
    spheres: Array
    links: NDArray[np.bool_]
    labels: Array
    success: bool


@dataclass
class _Run:
    call: SkillCall
    act: Executor
    judge: SkillController
    label: Executor | None = None
    t: int = 0
    frames: list[tuple[Array, NDArray[np.bool_], Array]] = field(default_factory=list)


class SkillScheduler:
    """Runs a plan: starts skill calls on events, replans the climb order, restarts stuck skills.

    With a ``labeler`` (for example ``script_executor()``), every executed skill call is recorded
    in ``segments`` with the labeler's actions on the visited states, and Gaussian noise ``noise``
    perturbs the executed fx, fy, lift (DART; DAgger when the executor is learned): skill data
    whose starts are the end states of the preceding skills, as the composition visits them."""

    def __init__(self, n_spheres: int, effect: EffectModel, make_executor: ExecutorFactory,
                 radius: float = 0.1, max_lanes: int | None = None,
                 labeler: ExecutorFactory | None = None, noise: float = 0.0,
                 seed: int = 0) -> None:  # fmt: skip
        self.n, self.effect, self.make, self.r = n_spheres, effect, make_executor, radius
        self.max_lanes, self.labeler, self.noise = max_lanes, labeler, noise
        self.rng = np.random.default_rng(seed)
        self.segments: list[Segment] = []

    def reset(self, pos: Array, goal: Array, course: Course | None = None) -> None:
        """New plan from the sphere positions (N, 3), the goal (3,) and the course (default: the
        single training step)."""
        self.goal, self.levels = goal, course_levels(Course() if course is None else course)
        self.runs: dict[int, _Run] = {}
        self.restarts, self.t, self.level, self.predicted = 0, 0, 0, 0.0
        # end step, skill, agents, steps of every finished skill call (-1: restarted)
        self.events: list[tuple[int, str, tuple[int, ...], int]] = []
        self._plan_level(pos)

    def _plan_level(self, pos: Array) -> None:
        lv, goal, ex = self.levels[self.level], self.goal, SwarmClimbPolicy(self.n, self.r)
        if self.level == len(self.levels) - 1:
            self.slots, end = ex._slots(goal), goal[:2]  # farthest from the step first
        else:
            nxt = self.levels[self.level + 1].face
            self.slots = ex._wait_slots(nxt, float(goal[1]))
            end = np.array([nxt - ex.WAIT_X, goal[1]])
        lanes, span = plan(self.effect, pos, end, self.r, self.max_lanes, lv)
        self.lanes, self.predicted = lanes, self.predicted + span
        self.top = Level(lv.face, lv.floor + lv.rise, lv.rise, lv.face)  # the level's platform
        self.stage = ["approach"] * len(self.lanes)
        self.todo = [list(ln.climbers) for ln in self.lanes]
        self.cur: list[int | None] = [None] * len(self.lanes)
        self.helper = [-1] * len(self.lanes)
        self.up: dict[int, Array] = {}  # spheres sent to a slot on the platform -> slot
        self.rescues: dict[int, int] = {}  # fallen sphere -> helper pulling it back up
        for ln in self.lanes:
            self._start(lv.call("roll", (ln.base,), base_spot(ln.y, self.r, lv)[:2]))
            for q, c in enumerate(ln.climbers):
                self._start(lv.call("roll", (c,), queue_spot(ln.y, q, self.r, lv)[:2]))

    def _start(self, call: SkillCall) -> None:
        label = self.labeler(call, self.n) if self.labeler is not None else None
        run = _Run(call, self.make(call, self.n), SkillController(call, self.n, self.r), label)
        for i in call.agents:
            self.runs[i] = run

    def _close(self, run: _Run, spheres: Array, links: NDArray[np.bool_], success: bool) -> None:
        if run.label is None or not run.frames:
            return
        ag = list(run.call.agents)
        x, lk, lab = zip(*run.frames, strict=True)
        self.segments.append(Segment(run.call, np.stack([*x, spheres[ag]]),
                                     np.stack([*lk, links[np.ix_(ag, ag)]]), np.stack(lab),
                                     success))  # fmt: skip

    def _gather(self, i: int, spheres: Array) -> None:
        # final level: farthest slot first; waiting line: nearest free slot, and a sphere still
        # rolling to its slot swaps with this one when that shortens both paths (no crossing)
        d, pi = np.linalg.norm, spheres[i, :2]
        if self.level == len(self.levels) - 1:
            tgt = self.slots.pop(0)
        else:
            tgt = self.slots.pop(int(np.argmin([d(s - pi) for s in self.slots])))
            for j, sj in [(j, sj) for j, sj in self.up.items() if j in self.runs]:
                pj = spheres[j, :2]
                if d(pi - sj) + d(pj - tgt) < d(pi - tgt) + d(pj - sj) - 1e-6:
                    self.up[j] = tgt
                    self._start(self.top.call("roll", (j,), tgt))
                    tgt = sj
        self.up[i] = tgt
        self._start(self.top.call("roll", (i,), tgt))

    def _release(self, i: int) -> None:
        # a sphere leaves its slot (fallen, or helper of a rescue): cancel its roll, free the slot
        self.runs.pop(i, None)
        self.slots.insert(0, self.up.pop(i))

    def _rescue(self, spheres: Array) -> None:
        """A sphere that fell back off the platform is pulled up again by the nearest sphere on
        the platform; both then go to slots. A helper that falls too is rescued in turn."""
        lv, low = self.levels[self.level], self.top.floor + self.r - 0.05
        for i, h in list(self.rescues.items()):
            if h >= 0 and spheres[h, 2] < low:  # the helper fell: both wait for a rescue
                self.runs.pop(i, None), self.runs.pop(h, None)
                self.rescues[i] = self.rescues[h] = -1
            elif h >= 0 and i not in self.runs:
                del self.rescues[i]
                self._gather(h, spheres)
                self._gather(i, spheres)
        for i in [i for i in self.up if spheres[i, 2] < low]:
            self._release(i)
            self.rescues[i] = -1
        for i in [i for i, h in self.rescues.items() if h < 0]:
            free = [j for j in self.up if spheres[j, 2] > low]
            if not free:
                continue
            spot = np.array([self.top.face, spheres[i, 1]])
            h = min(free, key=lambda j: float(np.linalg.norm(spheres[j, :2] - spot)))
            self._release(h)
            self.rescues[i] = h
            self._start(lv.call("pull", (h, i), np.array([0.0, spheres[i, 1]])))

    def _active(self) -> list[_Run]:
        return list({id(r): r for r in self.runs.values()}.values())

    def _next_climber(self, k: int, spheres: Array) -> int:
        lane, todo = self.lanes[k], self.todo[k]
        line = np.array([0.0, lane.y])
        lv = self.levels[self.level]
        dur = [self.effect.duration("climb", spheres[[c, lane.base]], line, lv) for c in todo]
        return todo[int(np.argmin(dur))]

    def _advance(self, k: int, spheres: Array) -> None:
        lane, busy, lv = self.lanes[k], self.runs, self.levels[self.level]
        if self.stage[k] == "approach":
            if lane.base not in busy and lane.climbers[0] not in busy:
                self.stage[k] = "climb"
        if self.stage[k] == "climb":
            c = self.cur[k]
            if c is not None and c not in busy:  # the climb ended
                self.cur[k] = None
                if self.todo[k]:
                    self._gather(c, spheres)
                else:
                    self.stage[k], self.helper[k] = "pull", c
                    self._start(lv.call("pull", (c, lane.base), np.array([0.0, lane.y])))
            elif c is None:
                c = self._next_climber(k, spheres)
                self.todo[k].remove(c)
                self.runs.pop(c, None)  # cancel its queue roll
                self.cur[k] = c
                self._start(lv.call("climb", (c, lane.base), np.array([0.0, lane.y])))
        elif self.stage[k] == "pull" and lane.base not in busy:
            self.stage[k] = "gather"
            self._gather(self.helper[k], spheres)
            self._gather(lane.base, spheres)

    def act(self, obs: list[Array], spheres: Array, links: NDArray[np.bool_]) -> Array:
        """Actions (N, 4) from the local observations, sphere states (N, 9) and links (N, N)."""
        pos, vel = spheres[:, :3], spheres[:, 3:6]
        for run in self._active():
            if run.judge.done(pos, vel):
                for i in run.call.agents:
                    del self.runs[i]
                self.events.append((self.t, run.call.name, run.call.agents, run.t))
                self._close(run, spheres, links, True)
            elif run.t >= MAX_SKILL_STEPS:
                self.restarts += 1
                self.events.append((self.t, run.call.name, run.call.agents, -1))
                self._close(run, spheres, links, False)
                self._start(run.call)
        for k in range(len(self.lanes)):
            self._advance(k, spheres)
        self._rescue(spheres)
        if (
            self.level < len(self.levels) - 1
            and not self.runs
            and not self.rescues
            and all(s == "gather" for s in self.stage)
        ):  # everyone waits in front of the next face: plan the next level
            self.level += 1
            self._plan_level(pos)
        out = np.zeros((self.n, ACT_DIM))
        for run in self._active():
            ag = list(run.call.agents)
            out[ag] = run.act(obs, spheres, links)[ag]
            out[ag, :3] += self.noise * self.rng.standard_normal((len(ag), 3))
            if run.label is not None:
                label = run.label(obs, spheres, links)[ag]
                run.frames.append((spheres[ag], links[np.ix_(ag, ag)], label))
            run.t += 1
        self.t += 1
        return np.clip(out, -1.0, 1.0)


@dataclass
class ScheduleEval:
    success: NDArray[np.bool_]  # per seed
    steps: NDArray[np.int64]
    moving: Array  # mean fraction of spheres moving
    predicted: Array  # predicted makespan of the chosen plan [steps]
    lanes: NDArray[np.int64]  # number of lanes of the chosen plan
    restarts: NDArray[np.int64]  # skill calls restarted after MAX_SKILL_STEPS


def perturb_dynamics(env: SphereSwarmStepEnv, seed: int) -> None:
    """Domain randomisation (Peng et al., 2018): scales the sphere masses and inertias and every
    sliding friction of ``env`` by factors drawn uniformly in ``DYNAMICS_*`` from ``seed``."""
    rng = np.random.default_rng(seed)
    body = env.model.body_mass > 0
    mass = rng.uniform(*DYNAMICS_MASS)
    env.model.body_mass[body] *= mass
    env.model.body_inertia[body] *= mass
    env.model.geom_friction[:, 0] *= rng.uniform(*DYNAMICS_FRICTION)


def evaluate_schedule(scheduler: SkillScheduler, seeds: list[int],
                      courses: Callable[[int], Course] | None = None,
                      dynamics: Callable[[SphereSwarmStepEnv, int], None] | None = None
                      ) -> ScheduleEval:  # fmt: skip
    """Closed-loop episodes in MuJoCo, one per seed, on the course ``courses(seed)`` (default:
    the single training step); ``dynamics(env, seed)`` edits each episode's fresh model (for
    example ``perturb_dynamics``)."""
    env = SphereSwarmStepEnv(n_spheres=scheduler.n)
    res: dict[str, list[float]] = {k: [] for k in ScheduleEval.__dataclass_fields__}
    for seed in seeds:
        course = None if courses is None else courses(seed)
        if course is not None or dynamics is not None:
            env = SphereSwarmStepEnv(n_spheres=scheduler.n, course=course)
        if dynamics is not None:
            dynamics(env, seed)
        obs, _ = env.reset(seed=seed)
        spheres = np.concatenate(env._kinematics(), 1)
        scheduler.reset(spheres[:, :3], env.goal, course)
        t, moving, info = 0, 0.0, {}
        while env.agents:
            moving += float(np.mean(np.linalg.norm(spheres[:, 3:5], axis=1) > MOVING_SPEED))
            act = scheduler.act([obs[a] for a in env.possible_agents], spheres, env.links)
            obs, _, _, _, infos = env.step(dict(zip(env.possible_agents, act, strict=True)))
            info, t = infos[env.possible_agents[0]], t + 1
            spheres = np.concatenate(env._kinematics(), 1)
        res["success"].append(info["is_success"])
        res["steps"].append(t)
        res["moving"].append(moving / t)
        res["predicted"].append(scheduler.predicted)
        res["lanes"].append(len(scheduler.lanes))
        res["restarts"].append(scheduler.restarts)
    dtypes = {"success": bool, "steps": np.int64, "lanes": np.int64, "restarts": np.int64}
    return ScheduleEval(**{k: np.asarray(v, dtype=dtypes.get(k, float)) for k, v in res.items()})


def segments_to_data(segments: list[Segment], name: str) -> tuple[SwarmDataset, list[SkillCall]]:
    """The segments of skill ``name`` in the two-sphere layout of the skill data (``expert`` =
    labels, involved spheres first, renumbered from 0) and their calls, for ``SkillPrior.fit``,
    ``SkillWAM.fit`` and ``SkillValue.fit``."""
    segs = [s for s in segments if s.call.name == name]
    e, h = len(segs), MAX_SKILL_STEPS
    out = {
        "spheres": np.zeros((e, h + 1, 2, 9)),
        "links": np.zeros((e, h + 1, 2, 2), dtype=bool),
        "obs": np.zeros((e, h + 1, 2, obs_slices(2)["others"].stop)),
        "actions": np.zeros((e, h, 2, ACT_DIM)),
        "expert": np.zeros((e, h, 2, ACT_DIM)),
        "rewards": np.zeros((e, h)),
        "valid": np.zeros((e, h), dtype=bool),
        "goal": np.zeros((e, 3)),
        "success": np.array([s.success for s in segs], dtype=bool),
    }
    calls = []
    for i, s in enumerate(segs):
        t, k = len(s.labels), s.spheres.shape[1]
        out["spheres"][i, :, :k] = s.spheres[np.minimum(np.arange(h + 1), t)]
        out["links"][i, :, :k, :k] = s.links[np.minimum(np.arange(h + 1), t)]
        out["expert"][i, :t, :k] = s.labels
        out["valid"][i, :t] = True
        calls.append(replace(s.call, agents=tuple(range(k))))
    return SwarmDataset(**out), calls
