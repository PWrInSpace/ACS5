#include "flight/estimator_policy.h"

estimator_gates_t estimator_gates(flight_state_t state, bool calib_done)
{
    /* Only on the pad and under the chute does the accelerometer measure gravity. */
    const bool on_pad = (state == FLIGHT_STATE_STANDBY);
    return (estimator_gates_t){
        .corrections_inhibited = (state == FLIGHT_STATE_ACCELERATION ||
                                  state == FLIGHT_STATE_FREE_FLIGHT  ||
                                  state == FLIGHT_STATE_FALLING),
        .collecting_calib      = on_pad && !calib_done,
        .zupt_active           = on_pad,
    };
}

bool estimator_baro_vel_valid(flight_state_t state, bool baro_datum_set, bool baro_recent)
{
    if (!baro_recent) return false;
    switch (state) {
        /* On the pad flight_sm reports 0, which is the truth there. */
        case FLIGHT_STATE_STANDBY:     return baro_datum_set;
        case FLIGHT_STATE_FREE_FLIGHT:
        case FLIGHT_STATE_FALLING:     return true;
        /* flight_sm does not update baro_vel_z in boost or under the chute. */
        default:                       return false;
    }
}

bool baro_sample_fresh(uint32_t sample_us, uint32_t now_us, uint32_t max_age_us)
{
    return sample_us != 0u && (uint32_t)(now_us - sample_us) < max_age_us;
}
