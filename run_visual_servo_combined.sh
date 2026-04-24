#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${APP_DIR}/build"
BIN="${BUILD_DIR}/servoFrankaIBVS_combined"
EMC="${APP_DIR}/config/eMc.yaml"   # update path if needed
IP="172.16.0.2"
MODE=${MODE:-1}  # 1: single (CHRPS), 2: multi-sequence (Investment)

# Match the remote GUI launcher environment when ROS Jazzy is installed.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  if [ "$had_nounset" -eq 1 ]; then
    set -u
  fi
fi

export LD_LIBRARY_PATH="/opt/ros/jazzy/lib/x86_64-linux-gnu:$HOME/visp_install/lib:${LD_LIBRARY_PATH:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ip)
      IP="$2"
      shift 2
      ;;
    --eMc)
      EMC="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    *)
      break
      ;;
  esac
done

if [[ "$EMC" != /* ]]; then
  EMC="${APP_DIR}/${EMC}"
fi

# Rebuild every time (only recompiles if sources changed)
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release -DVISP_DIR=$HOME/visp_install/lib/cmake/visp
make -j"$(nproc)" servoFrankaIBVS_combined

# Run the binary
"$BIN" \
  --eMc "$EMC" \
  --ip "$IP" \
  --no-convergence-threshold \
  --adaptive-gain \
  --tag-size 0.05 \
  --desired-factor 8 \
  --mode "$MODE" \
  "$@"
