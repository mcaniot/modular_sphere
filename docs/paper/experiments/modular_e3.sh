#!/usr/bin/env bash
# E3 campaign: phase 1 without the pull skill, phase 2 adds it; 3 seeds per variant.
# Usage: docs/paper/experiments/modular_e3.sh [DEVICE]  (after the E1 `collect` sets)
set -euo pipefail
dev=${1:-cuda}
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
for seed in 0 1 2; do
  for v in "modular 64 2" "frame 128 2" "world 256 4"; do
    read -r kind width layers <<<"$v"
    for _ in 1 2 3; do
      uv run python docs/paper/experiments/modular_e3.py "$kind" "$width" "$layers" "$seed" "$dev" \
        && break
    done || true
  done
done
