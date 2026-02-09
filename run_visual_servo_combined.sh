#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${APP_DIR}/build"
BIN="${BUILD_DIR}/servoFrankaIBVS_combined"
EMC="${APP_DIR}/config/eMc.yaml"   # update path if needed
MODE=${MODE:-1}  # 1: single (CHRPS), 2: multi-sequence (Investment)

# Make sure ViSP libs are found
export LD_LIBRARY_PATH="$HOME/visp_install/lib:${LD_LIBRARY_PATH:-}"

# Rebuild every time (only recompiles if sources changed)
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release -DVISP_DIR=$HOME/visp_install/lib/cmake/visp
make -j"$(nproc)" servoFrankaIBVS_combined

# Run the binary
"$BIN" \
  --eMc "$EMC" \
  --ip 172.16.0.2 \
  --no-convergence-threshold \
  --adaptive-gain \
  --plot \
  --tag-size 0.05 \
  --desired-factor 7 \
  --mode "$MODE"
