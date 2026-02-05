#!/usr/bin/env bash
set -euo pipefail

# Placeholder perf gate: compare against a simple baseline file if present.
# Replace with real numeric comparison once benchmarks are defined.

BASELINE_FILE="benchmarks/results/cuda/baseline.txt"
CURRENT_FILE="benchmarks/results/cuda/summary.txt"

if [ ! -f "${CURRENT_FILE}" ]; then
  echo "Missing ${CURRENT_FILE}. Run run_cuda.sh first."
  exit 1
fi

if [ ! -f "${BASELINE_FILE}" ]; then
  echo "No baseline found. Creating ${BASELINE_FILE} from current run."
  cp "${CURRENT_FILE}" "${BASELINE_FILE}"
  exit 0
fi

echo "Perf gate placeholder passed (no numeric checks yet)."
