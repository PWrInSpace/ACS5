#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Sensor-frame to body-frame axis remap, as a signed permutation. A general 3x3
 * would also admit reflections, which no chip placement can produce. */

typedef struct
{
    int8_t map[3];   /* body axis i reads sensor axis map[i] */
    int8_t sign[3];  /* ...times sign[i] */
} axes_remap_t;

#define AXES_REMAP_IDENTITY ((axes_remap_t){ .map = {0, 1, 2}, .sign = {1, 1, 1} })

/* Fails axes_remap_is_rotation, so a caller that forgets to supply the board's
 * mounting is rejected rather than silently given a plausible one. */
#define AXES_REMAP_UNSET ((axes_remap_t){ .map = {0, 0, 0}, .sign = {0, 0, 0} })

/* True when the remap is a signed permutation with determinant +1. */
bool axes_remap_is_rotation(const axes_remap_t *r);

/* out[] = remapped in[]. Safe when out aliases in.
 * Undefined unless axes_remap_is_rotation(r) - map[] indexes in[] unchecked. */
void axes_remap_apply(const axes_remap_t *r, const float in[3], float out[3]);
