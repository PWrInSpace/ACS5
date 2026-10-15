#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_flight_sm.c \
    ../../Core/Src/control/flight_sm.c
