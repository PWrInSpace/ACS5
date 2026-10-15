#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_roll_ctrl.c \
    ../../Core/Src/control/roll_ctrl.c \
    ../../Core/Src/control/quat.c
