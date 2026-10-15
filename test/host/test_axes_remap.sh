#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

build_run test_axes_remap.c \
    ../../Core/Src/utils/axes_remap.c
