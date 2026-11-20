#pragma once

#include "utils/axes_remap.h"

/* Body-frame roll axis: index 2 (+Z) is the longitudinal axis, pointing at the nose. */
#define BODY_AXIS_ROLL 2

/* Sensor mounting on this PCB revision (both on the nose-facing face; IIM-42653 +Z leaves
 * the package top, MMC5983MA +Z enters it). A new board revision changes only these two. */
#define BOARD_IMU_REMAP ((axes_remap_t){ .map = {0, 1, 2}, .sign = { 1,  1,  1} })

/* The in-plane magnetometer mounting is not modelled: mahony.c's latched heading reference
 * absorbs it near vertical; at large tilt under the chute it leaks into the logged heading. */
#define BOARD_MAG_REMAP ((axes_remap_t){ .map = {0, 1, 2}, .sign = { 1, -1, -1} })
