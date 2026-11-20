#include "control/qbar.h"

#include <math.h>

#define KELVIN_OFFSET     273.15f
#define R_DRY_AIR_J_KG_K  287.05f

float qbar_from_baro(bool baro_valid, float pressure_pa, float temperature_c,
                     const float vel_mps[3])
{
    /* roll_ctrl reads 0 as "no measurement" and falls back to fixed gains. */
    if (!baro_valid || !isfinite(pressure_pa) || !isfinite(temperature_c))
        return 0.0f;

    const float rho = pressure_pa / (R_DRY_AIR_J_KG_K * (temperature_c + KELVIN_OFFSET));
    const float vsq = vel_mps[0] * vel_mps[0] +
                      vel_mps[1] * vel_mps[1] +
                      vel_mps[2] * vel_mps[2];
    if (!isfinite(rho) || !isfinite(vsq) || !(rho > 0.0f))
        return 0.0f;
    return 0.5f * rho * vsq;
}
