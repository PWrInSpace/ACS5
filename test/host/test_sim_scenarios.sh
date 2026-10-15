#!/usr/bin/env bash
# Regression gate: the tuned config must pass every scenario. Until tuned gains
# land this asserts only that the sim runs and completes the maneuver nominally
# (pre-tuning gains may miss the timing criteria).
set -euo pipefail
cd "$(dirname "$0")"
source ./common.sh

bash sim_roll.sh
"$OUT/sim_roll" scenario nominal
"$OUT/sim_roll" scenario spin
"$OUT/sim_roll" scenario weak
"$OUT/sim_roll" scenario strong
"$OUT/sim_roll" scenario noisy
"$OUT/sim_roll" scenario trim
# Monte Carlo belongs in the gate too: the five scenarios above are single
# points, and the gains are only defensible across the uncertainty band.
"$OUT/sim_roll" mc 200 12345
