#!/usr/bin/env bash
# Configure, build and run the host unit tests (ASan + UBSan) via the "host" CMake preset.
set -euo pipefail
cd "$(dirname "$0")/../.."
cmake --preset host >/dev/null
cmake --build --preset host
ctest --preset host -j"$(nproc)"
