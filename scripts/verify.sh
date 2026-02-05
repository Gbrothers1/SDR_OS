#!/usr/bin/env bash
# SDR_OS Phase 1 verification script.
# Run locally to check the same things CI checks.
#
# Usage:
#   ./scripts/verify.sh          # CPU-only checks (fast)
#   ./scripts/verify.sh --gpu    # Include GPU/Docker checks (slower)
#   ./scripts/verify.sh --all    # Everything including container builds
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass=0
fail=0
skip=0

check() {
    local name="$1"
    shift
    printf "  %-40s" "$name"
    if output=$("$@" 2>&1); then
        printf "${GREEN}PASS${NC}\n"
        pass=$((pass + 1))
    else
        printf "${RED}FAIL${NC}\n"
        echo "$output" | head -5 | sed 's/^/    /'
        fail=$((fail + 1))
    fi
}

skip_check() {
    local name="$1"
    printf "  %-40s${YELLOW}SKIP${NC}\n" "$name"
    skip=$((skip + 1))
}

MODE="${1:-}"

echo ""
echo "========================================"
echo "  SDR_OS Verification"
echo "========================================"

# ──────── CPU checks (always run) ────────
echo ""
echo "CPU checks:"

check "docker-compose.yml valid" \
    docker compose config --quiet

check "SHM ringbuffer tests" \
    env PYTHONPATH=src python3 -m pytest tests/unit/test_shm_ringbuffer.py -q --tb=short

check "NVENC script syntax" \
    python3 -c "import ast; ast.parse(open('scripts/validate_nvenc.py').read())"

check "Python package importable" \
    env PYTHONPATH=src python3 -c "from sdr_os.ipc.shm_ringbuffer import ShmRingWriter, ShmRingReader"

# ──────── Docker checks (--gpu or --all) ────────
if [[ "$MODE" == "--gpu" || "$MODE" == "--all" ]]; then
    echo ""
    echo "Docker checks:"

    check "ROS2 Jazzy container builds" \
        docker compose --profile dev build ros-bridge

    check "ROS2 Jazzy rosbridge present" \
        docker compose --profile dev run --rm ros-bridge bash -c \
            "source /opt/ros/jazzy/setup.bash && ros2 pkg list 2>/dev/null | grep rosbridge_server"

    if [[ "$MODE" == "--all" ]]; then
        check "CUDA container builds" \
            docker compose --profile cuda build app-cuda

        check "NVENC validation (GPU)" \
            docker compose --profile cuda run --rm app-cuda python3 scripts/validate_nvenc.py
    fi
else
    echo ""
    echo "Docker checks:"
    skip_check "ROS2 Jazzy build (use --gpu)"
    skip_check "NVENC validation (use --all)"
fi

# ──────── Summary ────────
echo ""
echo "========================================"
total=$((pass + fail + skip))
printf "  Results: ${GREEN}%d passed${NC}" "$pass"
if [ "$fail" -gt 0 ]; then
    printf ", ${RED}%d failed${NC}" "$fail"
fi
if [ "$skip" -gt 0 ]; then
    printf ", ${YELLOW}%d skipped${NC}" "$skip"
fi
printf " / %d total\n" "$total"
echo "========================================"
echo ""

exit "$fail"
