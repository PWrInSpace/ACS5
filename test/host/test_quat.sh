#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_quat.c \
    ../../Core/Src/control/quat.c
