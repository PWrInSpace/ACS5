#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_servo_t75_math.c \
    ../../Core/Src/sensor_drivers/servos/servo_t75_math.c
