#include "flight/flight_sm.h"
#include "config/airframe.h"
#include "utils/control_rate.h"
#include "utils/mathx.h"

#include <math.h>

flight_sm_config_t flight_sm_config_default(void)
{
    return (flight_sm_config_t){
        .launch_accel_mps2  = 1.5f * GRAVITY,
        .launch_debounce    = (int)SEC_TO_TICKS(0.040f),
        .burnout_accel_mps2 = 1.0f,
        .burnout_debounce   = (int)SEC_TO_TICKS(0.032f),
        .apogee_vel_mps     = -1.0f,
        .apogee_alt_drop_m  = 7.5f,
        .recovery_vel_mps   = -5.0f,
        /* Ticks, not baro samples: at 0.05 s per vz update this spans only one or two vz values. */
        .recovery_debounce  = (int)SEC_TO_TICKS(0.064f),
        .baro_vel_tau_s     = 0.34f,
        .baro_vel_min_span_s = 0.05f,
        .post_falling_timeout_ticks = SEC_TO_TICKS(240.0f),
        .max_coast_ticks       = SEC_TO_TICKS(AIRFRAME_MAX_COAST_S),
        .launch_confirm_ticks  = SEC_TO_TICKS(AIRFRAME_LAUNCH_CONFIRM_S),
        .launch_confirm_baro_m = AIRFRAME_LAUNCH_CONFIRM_BARO_M,
        .launch_confirm_imu_m  = AIRFRAME_LAUNCH_CONFIRM_IMU_M,
    };
}

void flight_sm_init(flight_sm_t *sm, const flight_sm_config_t *cfg)
{
    *sm = (flight_sm_t){0};
    sm->cfg   = *cfg;
    sm->state = FLIGHT_STATE_STANDBY;
}

const char *flight_state_name(flight_state_t s)
{
    switch (s)
    {
        case FLIGHT_STATE_STANDBY:      return "STANDBY";
        case FLIGHT_STATE_ACCELERATION: return "ACCEL";
        case FLIGHT_STATE_FREE_FLIGHT:  return "FREE_FLT";
        case FLIGHT_STATE_FALLING:      return "FALLING";
        case FLIGHT_STATE_RECOVERY:     return "RECOVERY";
        default:                        return "UNKNOWN";
    }
}

/* Baro-derived vertical speed; only meaningful on fresh baro ticks.
 * Why the span is fixed, not the publish interval: docs/design.md#baro-velocity */
static void update_baro_vel(flight_sm_t *sm, const flight_sm_input_t *in)
{
    /* The low-pass cannot recover from NaN, and it feeds the only exits from FREE_FLIGHT
     * and FALLING. The negated compare also rejects a NaN dt, which "dt <= 0" would pass. */
    if (!in->baro_fresh || !(in->baro_dt_s > 0.0f)) return;
    if (!sm->baro_vel_seeded) {
        if (!isfinite(in->baro_alt_m)) return;
        sm->prev_baro_alt   = in->baro_alt_m;
        sm->baro_span_s     = 0.0f;
        sm->baro_vel_seeded = true;
        return;
    }

    /* Accumulated before the sample is vetted: a rejected sample still consumed time,
     * and the divisor must equal the interval prev_baro_alt was measured over. */
    sm->baro_span_s += in->baro_dt_s;
    if (!isfinite(in->baro_alt_m)) return;
    if (sm->baro_span_s < sm->cfg.baro_vel_min_span_s) return;

    const float raw = (in->baro_alt_m - sm->prev_baro_alt) / sm->baro_span_s;
    const float a   = sm->cfg.baro_vel_tau_s
                    / (sm->cfg.baro_vel_tau_s + sm->baro_span_s);
    sm->baro_vel_z    = a * sm->baro_vel_z + (1.0f - a) * raw;
    sm->prev_baro_alt = in->baro_alt_m;
    sm->baro_span_s   = 0.0f;
}

bool flight_sm_step(flight_sm_t *sm, const flight_sm_input_t *in)
{
    const flight_sm_config_t *c = &sm->cfg;
    flight_state_t entry_state = sm->state;
    sm->tick++;

    switch (sm->state)
    {
        case FLIGHT_STATE_STANDBY:
            if (in->baro_fresh && isfinite(in->baro_alt_m)) {
                sm->pad_baro_alt   = in->baro_alt_m;
                sm->pad_baro_valid = true;
            }
            if (in->az_mps2 > c->launch_accel_mps2) {
                if (++sm->launch_count >= c->launch_debounce) {
                    sm->state           = FLIGHT_STATE_ACCELERATION;
                    sm->launch_count    = 0;
                    sm->launch_tick     = sm->tick;
                    sm->boost_baro_seen = false;
                    sm->boost_baro_max  = sm->pad_baro_alt;
                    sm->boost_imu_v     = 0.0f;
                    sm->boost_imu_h     = 0.0f;
                }
            } else sm->launch_count = 0;
            break;

        case FLIGHT_STATE_ACCELERATION:
            sm->boost_imu_v += (in->az_mps2 - GRAVITY) * CONTROL_DT_S;
            sm->boost_imu_h += sm->boost_imu_v * CONTROL_DT_S;
            if (in->baro_fresh && isfinite(in->baro_alt_m)) {
                sm->boost_baro_seen = true;
                if (in->baro_alt_m > sm->boost_baro_max) sm->boost_baro_max = in->baro_alt_m;
            }
            /* Without baro on both sides of the launch there is no evidence
             * against it, and undoing a real launch is the worse error. */
            if (sm->tick - sm->launch_tick == c->launch_confirm_ticks &&
                sm->pad_baro_valid && sm->boost_baro_seen &&
                sm->boost_baro_max - sm->pad_baro_alt < c->launch_confirm_baro_m &&
                sm->boost_imu_h < c->launch_confirm_imu_m) {
                sm->state         = FLIGHT_STATE_STANDBY;
                sm->burnout_count = 0;
                sm->false_launch_count++;
                break;
            }
            if (in->az_mps2 <= c->burnout_accel_mps2) {
                if (++sm->burnout_count >= c->burnout_debounce) {
                    sm->state           = FLIGHT_STATE_FREE_FLIGHT;
                    sm->burnout_count   = 0;
                    sm->coast_entry_tick = sm->tick;
                    sm->baro_vel_z      = isfinite(in->vel_z_mps) ? in->vel_z_mps : 0.0f;
                    sm->peak_alt        = in->alt_m;
                    sm->prev_baro_alt   = in->baro_alt_m;
                    sm->baro_span_s     = 0.0f;
                    /* A NaN seed would poison every later difference; update_baro_vel
                     * seeds from the next good sample instead. */
                    sm->baro_vel_seeded = isfinite(in->baro_alt_m);
                }
            } else sm->burnout_count = 0;
            break;

        case FLIGHT_STATE_FREE_FLIGHT:
            if (in->alt_m > sm->peak_alt) sm->peak_alt = in->alt_m;
            update_baro_vel(sm, in);
            if ((sm->baro_vel_z < c->apogee_vel_mps &&
                 (sm->peak_alt - in->alt_m) > c->apogee_alt_drop_m) ||
                sm->tick - sm->coast_entry_tick >= c->max_coast_ticks) {
                sm->state              = FLIGHT_STATE_FALLING;
                sm->falling_entry_tick = sm->tick;
            }
            break;

        case FLIGHT_STATE_FALLING:
            update_baro_vel(sm, in);
            if (sm->baro_vel_z > c->recovery_vel_mps) {
                if (++sm->recovery_count >= c->recovery_debounce) {
                    sm->state          = FLIGHT_STATE_RECOVERY;
                    sm->recovery_count = 0;
                }
            } else sm->recovery_count = 0;
            break;

        case FLIGHT_STATE_RECOVERY:
            break;
    }

    /* Counts from FALLING entry, in FALLING and RECOVERY both. */
    if ((sm->state == FLIGHT_STATE_FALLING || sm->state == FLIGHT_STATE_RECOVERY) &&
        !sm->mission_complete &&
        (sm->tick - sm->falling_entry_tick) >= c->post_falling_timeout_ticks)
        sm->mission_complete = true;

    return sm->state != entry_state;
}
