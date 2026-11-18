#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_ubx_parse.c \
    ../../Core/Src/sensor_drivers/gnss/ubx_parse.c
