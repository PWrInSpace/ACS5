#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_ms5611_math.c \
    ../../Core/Src/sensor_drivers/baro/ms5611_math.c
