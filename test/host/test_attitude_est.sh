#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_attitude_est.c \
    ../../Core/Src/filter/attitude.c \
    ../../Core/Src/filter/mahony.c \
    ../../Core/Src/control/quat.c
