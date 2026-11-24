import multiprocessing as mp
import threading

import numpy as np
import pytest

from modular_sphere.swarm import obs_slices

torch = pytest.importorskip("torch")

from modular_sphere.swarm_rl import SwarmVecEnv, _worker, evaluate, train_mappo  # noqa: E402
from modular_sphere.swarm_torch import SwarmPolicy  # noqa: E402


def _policy(n):
    policy = SwarmPolicy(n, hidden=16, device="cpu")
    dim = obs_slices(n)["others"].stop
    policy.stats = {"mean": torch.zeros(dim), "std": torch.ones(dim)}
    return policy


def test_worker_protocol_in_process():
    """The worker loop, run in a thread so that coverage sees it."""
    parent, child = mp.Pipe()
    thread = threading.Thread(target=_worker, args=(child, 2, 2, 0, 0.1))
    thread.start()
    parent.send(("reset", ([3, None], False)))
    obs, state = parent.recv()
    assert obs.shape == (2, 2, 19) and state.shape[0] == 2
    zeros = np.zeros((2, 2, 4))
    finished = False
    for _ in range(400):
        parent.send(("step", zeros))
        obs, state, _, info = parent.recv()
        assert info["done"][1]  # padding slot never runs
        if info["done"][0]:
            finished = True
            break
    assert finished and info["trunc"][0] and info["reward"][0] < 0
    parent.send(("height", 0.05))
    parent.send(("reset", (None, True)))  # training episodes from the worker seed, new height
    parent.recv()
    for _ in range(301):  # time-out, then auto-reset
        parent.send(("step", zeros))
        *_, info = parent.recv()
    assert not info["done"].any()
    parent.send(("close", None))
    thread.join(timeout=5)
    assert not thread.is_alive()


def test_close_terminates_hung_worker():
    """close() tolerates a dead pipe and terminates a worker that does not exit."""

    class Conn:
        closed = False

        def send(self, msg):
            raise BrokenPipeError

        def close(self):
            self.closed = True

    class Proc:
        alive = True
        terminated = False

        def join(self, timeout=None):
            if self.terminated:
                self.alive = False

        def is_alive(self):
            return self.alive

        def terminate(self):
            self.terminated = True

    vec = SwarmVecEnv.__new__(SwarmVecEnv)
    vec._conns, vec._procs = [Conn()], [Proc()]
    vec.close()
    assert vec._conns[0].closed and vec._procs[0].terminated and not vec._procs[0].alive


def test_vec_env_evaluate_and_mappo():
    vec = SwarmVecEnv(2, n_workers=1, envs_per_worker=2, time_penalty=0.01)
    try:
        policy = _policy(2)
        res = evaluate(policy, vec, [0, 1, 2])  # two batches, the second one padded
        assert res.success.shape == res.steps.shape == (3,)
        assert ((res.moving >= 0) & (res.moving <= 1)).all() and (res.links >= 0).all()
        vec.set_step_height(0.05)
        seen = []
        log = train_mappo(policy, vec, iterations=2, horizon=160, epochs=1, minibatch=64,
                          critic_warmup=1, hidden=16,
                          callback=lambda i, m: seen.append(i))  # fmt: skip
        assert seen == [0, 1] and len(log) == 2
        assert log[-1]["episodes"] >= 2 and np.isfinite(log[-1]["length"])
        assert np.isnan(log[0]["success"]) and log[0]["episodes"] == 0
        assert -3.0 <= np.log(log[-1]["std"]) <= 0.0
        assert len(train_mappo(policy, vec, iterations=1, horizon=4, hidden=16)) == 1
        with pytest.raises(ValueError):
            evaluate(policy, vec, [])
    finally:
        vec.close()
