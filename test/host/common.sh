#!/usr/bin/env bash
# Shared toolchain settings for the host tests. Source this, do not execute it.
#
# build_run <test.c> <module.c>... compiles each module, links it against the
# test, and runs the result. Objects go to $OUT to keep the tree clean.

CC=${CC:-gcc}
CFLAGS=${CFLAGS:-"-std=c11 -O2 -Wall -Wextra -Werror"}
INC="-I../../Core/Inc -I../../Core/Inc/sensor_drivers"
OUT=${OUT:-/tmp}

build_run() {
    local test_src=$1; shift
    local bin="$OUT/${test_src%.c}"
    local objs=()

    for src in "$@"; do
        local obj="$OUT/$(basename "${src%.c}").o"
        # shellcheck disable=SC2086  # CFLAGS/INC are deliberately word-split
        $CC $CFLAGS $INC -c "$src" -o "$obj"
        objs+=("$obj")
    done

    # shellcheck disable=SC2086
    $CC $CFLAGS $INC "$test_src" "${objs[@]}" -o "$bin" -lm
    "$bin"
}
