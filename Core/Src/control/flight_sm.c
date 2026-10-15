#include "control/flight_sm.h"
#include "control/control_rate.h"
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
        /* Ticks, not baro samples: at 0.05 s per vz update this window spans
         * one or two distinct values and filters far less than its duration
         * suggests. */
        .recovery_debounce  = (int)SEC_TO_TICKS(0.064f),
        .baro_vel_tau_s     = 0.34f,
        .baro_vel_min_span_s = 0.05f,
        .post_falling_timeout_ticks = SEC_TO_TICKS(240.0f),
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
 *
 * Differencing altitude over a span S gives velocity noise sigma_h*sqrt(2)/S,
 * so the span - not the publish cadence - sets the SNR, and the low-pass only
 * recovers sqrt(S/tau) of what a short span throws away. The MS5611 at OSR 4096
 * resolves 1.2 Pa RMS, about 0.1 m, which over 0.05 s and tau 0.34 s leaves
 * roughly 1 m/s on vz - already the width of the -1.0 m/s apogee gate. Taking
 * the difference across every publish instead would scale that by
 * sqrt(0.05/dt) and drown the gate; holding tau responsible instead would need
 * tau near 2 s, which is slower than the altitude-drop term this gate is ANDed
 * with. So the span is held fixed and the publish rate is free to change. */
static void update_baro_vel(flight_sm_t *sm, const flight_sm_input_t *in)
{
    /* baro_vel_z feeds the only exits from FREE_FLIGHT and FALLING, and the
     * low-pass has no path back out of NaN: one bad sample would strand the
     * state machine for the rest of the flight. Negated compares also reject
     * a NaN dt, which "dt <= 0" would let through. */
    if (!in->baro_fresh || !(in->baro_dt_s > 0.0f)) return;
    if (!sm->baro_vel_seeded) {
        if (!isfinite(in->baro_alt_m)) return;
        sm->prev_baro_alt   = in->baro_alt_m;
        sm->baro_span_s     = 0.0f;
        sm->baro_vel_seeded = true;
        return;
    }

    /* Accumulated before the sample is vetted: a rejected sample still consumed
     * wall time, and charging it to the next difference is what keeps the
     * divisor equal to the interval prev_baro_alt was actually measured over. */
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
            if (in->az_mps2 > c->launch_accel_mps2) {
                if (++sm->launch_count >= c->launch_debounce) {
                    sm->state        = FLIGHT_STATE_ACCELERATION;
                    sm->launch_count = 0;
                }
            } else sm->launch_count = 0;
            break;

        case FLIGHT_STATE_ACCELERATION:
            if (in->az_mps2 <= c->burnout_accel_mps2) {
                if (++sm->burnout_count >= c->burnout_debounce) {
                    sm->state           = FLIGHT_STATE_FREE_FLIGHT;
                    sm->burnout_count   = 0;
                    sm->peak_alt        = in->alt_m;
                    sm->prev_baro_alt   = in->baro_alt_m;
                    sm->baro_span_s     = 0.0f;
                    /* A NaN here would poison every later difference, and the
                     * filter has no path back out; leave it unseeded instead
                     * and let update_baro_vel seed from the next good sample. */
                    sm->baro_vel_seeded = isfinite(in->baro_alt_m);
                }
            } else sm->burnout_count = 0;
            break;

        case FLIGHT_STATE_FREE_FLIGHT:
            if (in->alt_m > sm->peak_alt) sm->peak_alt = in->alt_m;
            update_baro_vel(sm, in);
            if (sm->baro_vel_z < c->apogee_vel_mps &&
                (sm->peak_alt - in->alt_m) > c->apogee_alt_drop_m) {
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
