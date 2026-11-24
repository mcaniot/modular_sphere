#!/usr/bin/env bash
# E4 campaign: online adaptation to mass x1.5 and x0.7, 3 seeds, run one after the other so that
# adaptation times are not shared between processes. Each run resumes from its pickle (the GPU
# sometimes fails with an illegal instruction): up to 3 tries; exit 1 if a run never completes.
# Usage: docs/paper/experiments/modular_e4.sh [DEVICE]  (after the E1 sets and models)
set -uo pipefail
dev=${1:-cuda}
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
failed=0
for cond in mass1.5 mass0.7; do
  for seed in 0 1 2; do
    ok=0
    for _ in 1 2 3; do
      uv run python docs/paper/experiments/modular_e4.py "$cond" "$seed" "$dev" && ok=1 && break
    done
    [ "$ok" = 1 ] || { echo "E4 $cond s$seed failed" >&2; failed=1; }
  done
done
exit "$failed"
