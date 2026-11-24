#!/usr/bin/env bash
# E2 campaign: the WAM executors alone (mosaic) against the same executors verified by MuJoCo
# (oracle) or by a world model, on 50 paired random-course seeds per condition.
# Usage: modular_e2.sh CONDITION MODE... (one sequential queue; run several queues side by side)
set -u
cd "$(dirname "$0")/../../.."
cond=$1
shift
export CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=1 MUJOCO_GL=egl
for mode in "$@"; do
    tag=_s05m3
    [ "$mode" = mosaic ] && tag=
    [ -f "data/swarm/modular/e2_${cond}_${mode/:/_}${tag}.pkl" ] && continue
    uv run python docs/paper/experiments/modular_e2.py "$cond" "$mode" device=cpu sigma=0.05 margin=3 \
        tag=$tag > "data/swarm/modular/e2_${cond}_${mode/:/_}${tag}.log" 2>&1
done
