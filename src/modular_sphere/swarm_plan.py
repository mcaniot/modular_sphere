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
"""

from __future__ import annotations

import itertools
from collections.abc import Callable, Iterator
from dataclasses import dataclass

import numpy as np
import torch
from numpy.typing import NDArray

from modular_sphere.swarm import ACT_DIM, STEP_H, STEP_X, SphereSwarmStepEnv, SwarmClimbPolicy
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


@dataclass
class Lane:
    """A base and the climbers that go over it on the line ``y``; the last climber pulls."""

    y: float
    base: int
    climbers: list[int]


def base_spot(y: float, radius: float = 0.1) -> Array:
    """Where the base waits before its first climb, inside the climb initiation set."""
    return np.array([STEP_X - radius - 0.11, y, radius])


def queue_spot(y: float, q: int, radius: float = 0.1) -> Array:
    """Where the ``q``-th climber of a lane waits."""
    return base_spot(y, radius) - (QUEUE_GAP + QUEUE_PITCH * q, 0.0, 0.0)


class EffectModel:
    """Skill durations predicted by the steps-to-go models at a skill's start state."""

    def __init__(self, values: dict[str, SkillValue]) -> None:
        self.values = values

    @torch.no_grad()
    def duration(self, name: str, states: Array, target: Array) -> float:
        """Predicted steps of skill ``name`` from the involved spheres' states (k, 3) or (k, 9)
        (positions only: at rest) towards ``target`` (roll: xy; climb, pull: (0, lane y))."""
        value = self.values[name]
        x = np.zeros((len(states), 9))
        x[:, : states.shape[1]] = states
        t = torch.as_tensor(x, dtype=torch.float32, device=value.mean.device)
        links = torch.zeros(len(x), len(x), dtype=torch.bool, device=t.device)
        call = SkillCall(name, tuple(range(len(x))), np.asarray(target, dtype=np.float64))
        return float(value(skill_features(call, t, links)))


def lane_makespan(effect: EffectModel, lane: Lane, pos: Array, goal_xy: Array,
                  radius: float = 0.1) -> float:  # fmt: skip
    """Predicted steps until the lane's base reaches the gathering zone."""
    y, r = lane.y, radius
    line = np.array([0.0, y])
    face = np.array([STEP_X - r - 0.01, y, r])
    edge = np.array([STEP_X + 0.1, y, STEP_H + r])
    t = effect.duration("roll", pos[[lane.base]], base_spot(y, r)[:2])
    for q, c in enumerate(lane.climbers):
        t = max(t, effect.duration("roll", pos[[c]], queue_spot(y, q, r)[:2]))
        base = base_spot(y, r) if q == 0 else face
        t += effect.duration("climb", np.stack([queue_spot(y, q, r), base]), line)
    t += effect.duration("pull", np.stack([edge, face]), line)
    return t + effect.duration("roll", edge[None], goal_xy)


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
         max_lanes: int | None = None) -> tuple[list[Lane], float]:  # fmt: skip
    """Plan of minimum predicted makespan over lane partitions (blocks of two spheres or more,
    sorted by mean y), bases and lane lines; climbers ordered closest to the step first."""
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
                span = max(lane_makespan(effect, ln, pos, goal_xy, radius) for ln in lanes)
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
class _Run:
    call: SkillCall
    act: Executor
    judge: SkillController
    t: int = 0


class SkillScheduler:
    """Runs a plan: starts skill calls on events, replans the climb order, restarts stuck skills."""

    def __init__(self, n_spheres: int, effect: EffectModel, make_executor: ExecutorFactory,
                 radius: float = 0.1, max_lanes: int | None = None) -> None:  # fmt: skip
        self.n, self.effect, self.make, self.r = n_spheres, effect, make_executor, radius
        self.max_lanes = max_lanes

    def reset(self, pos: Array, goal: Array) -> None:
        """New plan from the sphere positions (N, 3) and the goal (3,)."""
        self.lanes, self.predicted = plan(self.effect, pos, goal[:2], self.r, self.max_lanes)
        self.slots = SwarmClimbPolicy(self.n, self.r)._slots(goal)  # farthest from the step first
        self.runs: dict[int, _Run] = {}
        self.stage = ["approach"] * len(self.lanes)
        self.todo = [list(ln.climbers) for ln in self.lanes]
        self.cur: list[int | None] = [None] * len(self.lanes)
        self.helper = [-1] * len(self.lanes)
        self.restarts = 0
        self.t = 0
        # end step, skill, agents, steps of every finished skill call (-1: restarted)
        self.events: list[tuple[int, str, tuple[int, ...], int]] = []
        for ln in self.lanes:
            self._start("roll", (ln.base,), base_spot(ln.y, self.r)[:2])
            for q, c in enumerate(ln.climbers):
                self._start("roll", (c,), queue_spot(ln.y, q, self.r)[:2])

    def _start(self, name: str, agents: tuple[int, ...], target: Array) -> None:
        call = SkillCall(name, agents, np.asarray(target, dtype=np.float64))
        run = _Run(call, self.make(call, self.n), SkillController(call, self.n, self.r))
        for i in agents:
            self.runs[i] = run

    def _gather(self, i: int) -> None:
        self._start("roll", (i,), self.slots.pop(0))

    def _active(self) -> list[_Run]:
        return list({id(r): r for r in self.runs.values()}.values())

    def _next_climber(self, k: int, spheres: Array) -> int:
        lane, todo = self.lanes[k], self.todo[k]
        line = np.array([0.0, lane.y])
        dur = [self.effect.duration("climb", spheres[[c, lane.base]], line) for c in todo]
        return todo[int(np.argmin(dur))]

    def _advance(self, k: int, spheres: Array) -> None:
        lane, busy = self.lanes[k], self.runs
        if self.stage[k] == "approach":
            if lane.base not in busy and lane.climbers[0] not in busy:
                self.stage[k] = "climb"
        if self.stage[k] == "climb":
            c = self.cur[k]
            if c is not None and c not in busy:  # the climb ended
                self.cur[k] = None
                if self.todo[k]:
                    self._gather(c)
                else:
                    self.stage[k], self.helper[k] = "pull", c
                    self._start("pull", (c, lane.base), np.array([0.0, lane.y]))
            elif c is None:
                c = self._next_climber(k, spheres)
                self.todo[k].remove(c)
                self.runs.pop(c, None)  # cancel its queue roll
                self.cur[k] = c
                self._start("climb", (c, lane.base), np.array([0.0, lane.y]))
        elif self.stage[k] == "pull" and lane.base not in busy:
            self.stage[k] = "gather"
            self._gather(self.helper[k])
            self._gather(lane.base)

    def act(self, obs: list[Array], spheres: Array, links: NDArray[np.bool_]) -> Array:
        """Actions (N, 4) from the local observations, sphere states (N, 9) and links (N, N)."""
        pos, vel = spheres[:, :3], spheres[:, 3:6]
        for run in self._active():
            if run.judge.done(pos, vel):
                for i in run.call.agents:
                    del self.runs[i]
                self.events.append((self.t, run.call.name, run.call.agents, run.t))
            elif run.t >= MAX_SKILL_STEPS:
                self.restarts += 1
                self.events.append((self.t, run.call.name, run.call.agents, -1))
                self._start(run.call.name, run.call.agents, run.call.target)
        for k in range(len(self.lanes)):
            self._advance(k, spheres)
        out = np.zeros((self.n, ACT_DIM))
        for run in self._active():
            ag = list(run.call.agents)
            out[ag] = run.act(obs, spheres, links)[ag]
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


def evaluate_schedule(scheduler: SkillScheduler, seeds: list[int]) -> ScheduleEval:
    """Closed-loop episodes in MuJoCo, one per seed."""
    env = SphereSwarmStepEnv(n_spheres=scheduler.n)
    res: dict[str, list[float]] = {k: [] for k in ScheduleEval.__dataclass_fields__}
    for seed in seeds:
        obs, _ = env.reset(seed=seed)
        spheres = np.concatenate(env._kinematics(), 1)
        scheduler.reset(spheres[:, :3], env.goal)
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
