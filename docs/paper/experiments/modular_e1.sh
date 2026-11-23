#!/usr/bin/env bash
# E1 campaign: every world-model variant, 3 training seeds, then the open-loop evaluation.
# A failed training is retried up to three times, then skipped (transient CUDA errors on this
# laptop GPU, shared with other jobs).
# Usage: docs/paper/experiments/modular_e1.sh [DEVICE]  (after `modular_e1.py collect SET`)
set -euo pipefail
dev=${1:-cuda}
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True  # the GPU is shared: limit fragmentation
py="uv run python docs/paper/experiments/modular_e1.py"
for seed in 0 1 2; do
  for v in "modular 64 2" "modular 128 2" "world 128 2" "id 128 2" "frame 128 2" "world 256 4" "id 256 4" \
    "frame 256 4"; do
    read -r kind width layers <<<"$v"
    [[ -f data/swarm/modular/wm_${kind}_${width}x${layers}_s${seed}.pt ]] && continue
    for _ in 1 2 3; do $py train "$kind" "$width" "$layers" "$seed" "$dev" && break; done || true
  done
done
[[ -f data/swarm/modular/wm_world_512x6_s0.pt ]] || $py train world 512 6 0 "$dev"
$py eval "$dev"
