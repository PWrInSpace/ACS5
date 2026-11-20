// Estimator + flight state machine + input policy on a synthetic vertical flight.
#include <math.h>
#include <stdio.h>

#include "test_assert.h"

#include "control/qbar.h"
#include "filter/attitude.h"
#include "flight/estimator_policy.h"
#include "flight/flight_sm.h"
#include "utils/control_rate.h"
#include "utils/mathx.h"

#define TICKS(s)        ((int)((s) * CONTROL_LOOP_HZ + 0.5f))
#define BARO_EVERY      5          /* 100 Hz baro publishes */
#define BOOST_SF_G      5.0f       /* specific force during boost, g */
#define DRAG_G          0.3f       /* specific force during coast, g (aft) */
#define PAD_T_C         15.0f

typedef struct { float h, v; } truth_t;

static float isa_pressure(float h) { return 101325.0f * powf(1.0f - h / 44330.0f, 5.255f); }

typedef struct {
    attitude_t est;
    flight_sm_t sm;
    float baro_alt;
    float baro_vel_age_s;
} pipeline_t;

/* One control tick as flight_task runs it, with a level vehicle. */
static void tick(pipeline_t *p, float sf_z, const truth_t *t, int k)
{
    const bool baro_fresh = (k % BARO_EVERY) == 0;
    if (baro_fresh) p->baro_alt = t->h;

    attitude_input_t in = {0};
    in.accel_mps2[2] = sf_z;
    in.imu_valid     = true;
    estimator_gates_t g = estimator_gates(p->sm.state, p->est.calib.done);
    in.corrections_inhibited = g.corrections_inhibited;
    in.collecting_calib      = g.collecting_calib;
    in.zupt_active           = g.zupt_active;
    in.baro_alt_m     = p->baro_alt;
    in.baro_valid     = true;
    in.baro_vel_z     = p->sm.baro_vel_z;
    in.baro_vel_valid = estimator_baro_vel_valid(p->sm.state, true, true);
    attitude_update(&p->est, &in);

    flight_sm_input_t s = {
        .az_mps2 = p->est.acc_original[2], .alt_m = p->est.pos[2],
        .baro_alt_m = p->baro_alt, .baro_fresh = baro_fresh,
        .baro_dt_s = baro_fresh ? BARO_EVERY * CONTROL_DT_S : 0.0f,
        .vel_z_mps = p->est.vel[2],
    };
    flight_sm_step(&p->sm, &s);
}

static void test_boost_velocity_and_qbar_track_truth(void)
{
    pipeline_t p = {0};
    attitude_config_t ecfg = attitude_config_default();
    ecfg.require_calib = false;
    attitude_init(&p.est, &ecfg);
    flight_sm_config_t scfg = flight_sm_config_default();
    flight_sm_init(&p.sm, &scfg);

    truth_t t = {0};
    int k = 0;
    for (; k < TICKS(2.0f); k++) tick(&p, GRAVITY, &t, k);
    CHECK(p.sm.state == FLIGHT_STATE_STANDBY);

    for (int i = 0; i < TICKS(2.0f); i++, k++) {
        t.v += (BOOST_SF_G - 1.0f) * GRAVITY * CONTROL_DT_S;
        t.h += t.v * CONTROL_DT_S;
        tick(&p, BOOST_SF_G * GRAVITY, &t, k);
    }
    CHECK(p.sm.state == FLIGHT_STATE_ACCELERATION);
    const float v_burnout_true = t.v;
    const float v_burnout_est  = p.est.vel[2];

    float q_half_est = 0.0f, q_half_true = 0.0f;
    for (int i = 0; i < TICKS(1.0f); i++, k++) {
        t.v += -(1.0f + DRAG_G) * GRAVITY * CONTROL_DT_S;
        t.h += t.v * CONTROL_DT_S;
        tick(&p, -DRAG_G * GRAVITY, &t, k);
        if (i == TICKS(0.5f)) {
            const float v_true[3] = {0.0f, 0.0f, t.v};
            q_half_true = qbar_from_baro(true, isa_pressure(t.h), PAD_T_C, v_true);
            q_half_est  = qbar_from_baro(true, isa_pressure(p.baro_alt), PAD_T_C, p.est.vel);
        }
    }
    CHECK(p.sm.state == FLIGHT_STATE_FREE_FLIGHT);

    printf("  burnout vz est %.1f / true %.1f m/s; +0.5 s qbar est %.0f / true %.0f Pa\n",
           v_burnout_est, v_burnout_true, q_half_est, q_half_true);
    CHECK_NEAR(v_burnout_est, v_burnout_true, 0.1f * v_burnout_true);
    CHECK_NEAR(p.est.vel[2], t.v, 0.1f * t.v);
    CHECK_NEAR(q_half_est, q_half_true, 0.1f * q_half_true);
}

int main(void)
{
    test_boost_velocity_and_qbar_track_truth();
    return test_report("flight_pipeline");
}
