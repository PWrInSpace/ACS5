#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_arming.c \
    ../../Core/Src/control/arming.c \
    ../../Core/Src/control/flight_sm.c
