#!/usr/bin/env bash
# Host-side tests for the platform-independent control and *_math modules.
# Compiles each test, runs it, fails loudly on the first error.
set -euo pipefail
cd "$(dirname "$0")"

run() { echo "=== $1 ==="; shift; "$@"; }

shopt -s nullglob
tests=(test_*.sh)
if [ ${#tests[@]} -eq 0 ]; then echo "no host tests yet"; exit 0; fi
for t in "${tests[@]}"; do run "$t" bash "$t"; done
echo "ALL HOST TESTS PASSED"
