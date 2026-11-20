/* Dynamic pressure for roll-control gain scheduling. */
#pragma once

#include <stdbool.h>

/* 0.5 * rho * |v|^2 with rho from baro pressure/temperature (dry air); 0 when unknown. */
float qbar_from_baro(bool baro_valid, float pressure_pa, float temperature_c,
                     const float vel_mps[3]);
