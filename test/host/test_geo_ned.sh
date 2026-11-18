#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_geo_ned.c \
    ../../Core/Src/utils/geo_ned.c
