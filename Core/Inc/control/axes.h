#pragma once

#include "utils/axes_remap.h"

/* Body-frame axis convention for this airframe: index 2 (+Z) is the
 * longitudinal axis, pointing at the nose. The roll controller twists about it,
 * the aggregator takes the roll rate and the launch-detect acceleration from it. */
#define BODY_AXIS_ROLL 2

/* Sensor mounting on this PCB revision. Both packages sit on the board face
 * that looks at the nose, and their datasheet Z axes run opposite ways there:
 * IIM-42653 +Z leaves the top of the package, MMC5983MA +Z enters it.
 * A new board revision changes these two values and nothing else. */
#define BOARD_IMU_REMAP ((axes_remap_t){ .map = {0, 1, 2}, .sign = { 1,  1,  1} })

/* The in-plane half of the magnetometer mounting is deliberately not modelled.
 * The heading reference latched in mahony.c absorbs it while the longitudinal
 * axis is near vertical; under the chute in RECOVERY, where corrections are
 * back on at large tilt, it leaks into the logged heading. */
#define BOARD_MAG_REMAP ((axes_remap_t){ .map = {0, 1, 2}, .sign = { 1, -1, -1} })
