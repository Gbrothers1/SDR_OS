#!/usr/bin/env bash
set -euo pipefail

mkdir -p benchmarks/results/rocm

# Placeholder benchmark: emit basic system info and a dummy metric.
# Replace with real model + env benchmarks.
{
  echo "timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "backend: rocm"
  echo "metric: samples_per_sec=0"
} > benchmarks/results/rocm/summary.txt

echo "ROCm benchmark placeholder complete."
