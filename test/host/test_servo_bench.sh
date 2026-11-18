#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_servo_bench.c \
    ../../Core/Src/sensor_drivers/servos/servo_bench.c
