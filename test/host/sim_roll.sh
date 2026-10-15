#!/usr/bin/env bash
# Build-only helper for the 1-DOF roll simulator: compiles sim_roll against the
# production controller and echoes the binary path. Not a test - it never runs
# the binary; test_sim_scenarios.sh drives it.
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

# shellcheck disable=SC2086  # CFLAGS/INC are deliberately word-split
$CC $CFLAGS $INC \
    sim_roll.c ../../Core/Src/control/roll_ctrl.c ../../Core/Src/control/quat.c \
    -o "$OUT/sim_roll" -lm
echo "built $OUT/sim_roll"
