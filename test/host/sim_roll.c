// 1-DOF roll simulator linking the PRODUCTION roll_ctrl.c - the controller
// that flies is the controller under test.
//
//   I_xx * p_dot = qbar*S*d * (C_ld*delta + C_lp * p*d/(2V))
//
// Includes the effects that actually bound the achievable gains: first-order
// servo lag + slew, gyro noise, decaying post-burnout airspeed.
//
// Success criteria: overshoot < 5 deg, |hold error| < 2 deg,
// rotate+settle < 3 s nominal.
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/control_rate.h"
#include "control/roll_ctrl.h"
#include "utils/mathx.h"

/* Integrate the plant on the exact step the controller runs at: gains tuned
 * against this sim describe the real vehicle only if the plant and the
 * controller share a timestep. */
#define DT   CONTROL_DT_S

/* Powered flight ends 3.04 s in at 148 m and 77.6 m/s; apogee is at 10.32 s.
 * Past that the canards have nothing to work with, so the run ends there. */
#define COAST_S 7.28f

/* Dynamic pressure crosses the controller's floor at about 6 s, and the
 * maneuver is done by 3 s. The hold criterion is measured between the two. */
#define HOLD_WINDOW_START_S 4.0f
#define HOLD_WINDOW_END_S   5.5f

typedef struct
{
    /* Joust 1 / K353, extracted from joust1_prot3.ork by the OpenRocket driver.
     * C_ld and C_lp come from Barrowman, which models each fin set in isolation:
     * the canard wake over the aft fins opposes the commanded roll and is in
     * neither number. Treat C_ld as an upper bound until flight data says
     * otherwise, and keep the +-50% Monte Carlo band as a floor, not a comfort. */
    float I_xx;            /* roll inertia at burnout, kg*m^2   */
    float C_ld;            /* canard roll moment coeff, 1/rad   */
    float C_lp;            /* roll damping coeff (<0)           */
    float S;               /* reference area, m^2               */
    float d;               /* reference diameter, m             */
    float rho;             /* air density, kg/m^3 (mid-coast)   */
    float v0;              /* airspeed at burnout, m/s          */
    float k_drag;          /* dV/dt = -(g + k_drag*V^2)         */
    float servo_tau_s;     /* servo first-order lag             */
    float servo_slew_rads; /* T75 @ 6 V: 375 deg/s horn
                              (assumes 1:1 linkage until bench measures it) */
    float gyro_noise_rads; /* 1-sigma white noise               */
    float p0;              /* roll rate at maneuver start       */
    /* Standing roll moment from fin misalignment, as the equivalent canard
     * deflection needed to cancel it. This is what ki_rate exists to reject and
     * what the DAMP->ROTATE gate fails on, so a plant without it cannot qualify
     * either. A quarter degree of aft-fin misalignment is about 2 deg here. */
    float trim_deg;
} sim_params_t;

static sim_params_t sim_params_default(void)
{
    return (sim_params_t){
        .I_xx = 0.04377f, .C_ld = 1.5414f, .C_lp = -61.397f,
        .S = 0.01815f, .d = 0.1520f, .rho = 1.195f,
        .v0 = 77.6f, .k_drag = 3.65e-4f,
        .servo_tau_s = 0.03f, .servo_slew_rads = 6.54f,
        .gyro_noise_rads = 0.01f, .p0 = 0.5f,
        .trim_deg = 0.0f,
    };
}

/* xorshift32 + Box-Muller: deterministic runs for a given seed */
static uint32_t g_rng;
static float frand(void) { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17;
                           g_rng ^= g_rng << 5;  return (float)(g_rng >> 8) / 16777216.0f; }
static float gauss(void)
{
    float u1 = frand() + 1e-7f, u2 = frand();
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI_F * u2);
}

typedef struct
{
    float overshoot_deg;   /* max theta_true - 90 deg during/after ROTATE */
    float hold_err_deg;    /* max |90 deg - theta_true| in last 1 s */
    float t_complete_s;    /* time from ROTATE entry to first |err|<2 deg with |p|<10 deg/s */
    int   completed;       /* reached HOLD */
    int   aborted;
} sim_result_t;

static sim_result_t simulate(const sim_params_t *pp, const roll_ctrl_config_t *cc,
                             float t_end_s, FILE *csv)
{
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, cc);

    float p = pp->p0, theta = 0.0f, delta_act = 0.0f, delta_lim = 0.0f;
    float rotate_entry_theta = 0.0f, rotate_entry_t = -1.0f;
    float V = pp->v0;
    sim_result_t res = {0};
    res.t_complete_s = -1.0f;

    int n = (int)(t_end_s / DT);
    for (int i = 0; i < n; i++) {
        float t = (float)i * DT;
        float qbar = 0.5f * pp->rho * V * V;

        /* attitude quaternion about body Z from integrated true angle */
        float s = sinf(theta * 0.5f);
        quat_t q = {cosf(theta * 0.5f), 0.0f, 0.0f, s};

        roll_ctrl_input_t in = {
            .q = q, .q_valid = true,
            .roll_rate_rads = p + pp->gyro_noise_rads * gauss(),
            .qbar_pa = qbar,
            .maneuver_enabled = true,
        };
        roll_ctrl_phase_t prev = rc.phase;
        float delta_cmd = roll_ctrl_step(&rc, &in);
        if (prev != ROLL_CTRL_ROTATE && rc.phase == ROLL_CTRL_ROTATE) {
            rotate_entry_theta = theta;
            rotate_entry_t     = t;
        }
        if (rc.phase == ROLL_CTRL_HOLD)  res.completed = 1;
        if (rc.phase == ROLL_CTRL_ABORT) res.aborted   = 1;

        /* Servo: the electronics slew-limit an internal setpoint (delta_lim)
         * and the horn tracks it with first-order lag. delta_lim must persist
         * across ticks; slewing from delta_act instead compounds the rate
         * limit with the lag and caps actuator travel at slew*DT/(tau+DT). */
        float step = pp->servo_slew_rads * DT;
        delta_lim += clampf(delta_cmd - delta_lim, -step, step);
        delta_act += (delta_lim - delta_act) * (DT / (pp->servo_tau_s + DT));

        /* plant */
        /* The trim enters where it physically acts: as a moment on the
         * airframe, present whatever the canards are doing. */
        float m_ctrl = qbar * pp->S * pp->d * pp->C_ld
                            * (delta_act + pp->trim_deg / RAD2DEG);
        float m_damp = (V > 1.0f)
                     ? qbar * pp->S * pp->d * pp->C_lp * (p * pp->d / (2.0f * V))
                     : 0.0f;
        p     += (m_ctrl + m_damp) / pp->I_xx * DT;
        theta += p * DT;

        /* Coast deceleration: gravity plus quadratic drag, fitted to the
         * OpenRocket trajectory. Airspeed sets qbar, and qbar sets the plant
         * gain, which falls by more than an order of magnitude between burnout
         * and apogee - a fixed-gain controller is correct at one instant of
         * this run and wrong at every other. */
        V -= (GRAVITY + pp->k_drag * V * V) * DT;
        if (V < 1.0f) V = 1.0f;

        /* metrics relative to the angle at ROTATE entry */
        if (rotate_entry_t >= 0.0f) {
            float rel = (theta - rotate_entry_theta) * RAD2DEG;
            if (rel - 90.0f > res.overshoot_deg) res.overshoot_deg = rel - 90.0f;
            if (res.t_complete_s < 0.0f &&
                fabsf(rel - 90.0f) < 2.0f && fabsf(p) * RAD2DEG < 10.0f)
                res.t_complete_s = t - rotate_entry_t;
            /* Measured while the canards still have authority. Past the qbar
             * floor the command is zero and the airframe simply coasts at
             * whatever angle it reached, so a window at the end of the run
             * would report the frozen angle and call it a hold. */
            if (t > HOLD_WINDOW_START_S && t < HOLD_WINDOW_END_S) {
                float e = fabsf(90.0f - rel);
                if (e > res.hold_err_deg) res.hold_err_deg = e;
            }
        }

        if (csv)
            fprintf(csv, "%.4f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f\n",
                    t, (int)rc.phase, p * RAD2DEG,
                    (theta - rotate_entry_theta) * RAD2DEG,
                    rc.theta_meas * RAD2DEG, rc.theta_sp * RAD2DEG,
                    rc.rate_sp * RAD2DEG, delta_cmd * RAD2DEG, delta_act * RAD2DEG);
    }
    return res;
}

/* scenario = named parameter perturbation.
 * Baro dropout is NOT a sim scenario on purpose: roll_ctrl has no baro
 * input - a dead baro drops the altitude arming gate in the aggregator,
 * which is covered by test_disable_returns_to_idle_neutral in
 * test_roll_ctrl.c. */
static sim_params_t scenario(const char *name)
{
    sim_params_t p = sim_params_default();
    if      (!strcmp(name, "weak"))   p.C_ld *= 0.5f;
    else if (!strcmp(name, "strong")) p.C_ld *= 2.0f;
    else if (!strcmp(name, "spin"))   p.p0 = 3.5f;          /* 200 deg/s entry */
    else if (!strcmp(name, "noisy"))  p.gyro_noise_rads = 0.05f;
    else if (!strcmp(name, "trim"))   p.trim_deg = 2.0f;
    return p;   /* "nominal" or unknown -> defaults */
}

static int criteria_pass(const sim_result_t *r)
{
    return r->completed && !r->aborted &&
           r->overshoot_deg < 5.0f &&
           r->hold_err_deg  < 2.0f &&
           r->t_complete_s > 0.0f && r->t_complete_s < 3.0f;
}

static void print_result(const char *name, const sim_result_t *r)
{
    printf("%-8s overshoot=%.2f deg  hold_err=%.2f deg  t=%.2f s  "
           "completed=%d aborted=%d  %s\n",
           name, r->overshoot_deg, r->hold_err_deg, r->t_complete_s,
           r->completed, r->aborted, criteria_pass(r) ? "PASS" : "FAIL");
}

int main(int argc, char **argv)
{
    roll_ctrl_config_t cfg = roll_ctrl_config_default();

    if (argc == 2 && !strcmp(argv[1], "nominal")) {
        sim_params_t p = sim_params_default();
        g_rng = 0xAC54C0DEu;
        printf("t,phase,p_dps,theta_true_deg,theta_meas_deg,theta_sp_deg,"
               "rate_sp_dps,delta_cmd_deg,delta_act_deg\n");
        simulate(&p, &cfg, COAST_S, stdout);
        return 0;
    }

    if (argc == 3 && !strcmp(argv[1], "scenario")) {
        /* Exit code gates only completed && !aborted - the timing/accuracy
         * criteria become binding once tuned gains land (mc mode enforces
         * them). The PASS/FAIL label always reflects full criteria. */
        sim_params_t p = scenario(argv[2]);
        g_rng = 0xAC54C0DEu;
        sim_result_t r = simulate(&p, &cfg, COAST_S, NULL);
        print_result(argv[2], &r);
        return (r.completed && !r.aborted) ? 0 : 1;
    }

    if (argc == 4 && !strcmp(argv[1], "mc")) {
        int n = atoi(argv[2]);
        uint32_t seed = (uint32_t)strtoul(argv[3], NULL, 10);
        int fails = 0;
        sim_result_t worst = {0};
        for (int i = 0; i < n; i++) {
            g_rng = seed + (uint32_t)i * 2654435761u;
            if (g_rng == 0u) g_rng = 0xAC54C0DEu;  /* xorshift32 fixed point */
            sim_params_t p = sim_params_default();
            p.C_ld *= 1.0f + (frand() * 2.0f - 1.0f) * 0.5f;
            p.C_lp *= 1.0f + (frand() * 2.0f - 1.0f) * 0.5f;
            p.I_xx *= 1.0f + (frand() * 2.0f - 1.0f) * 0.5f;
            p.trim_deg = (frand() * 2.0f - 1.0f) * 2.0f;
            sim_result_t r = simulate(&p, &cfg, COAST_S, NULL);
            if (!criteria_pass(&r)) {
                fails++;
                print_result("FAILRUN", &r);
                printf("  params: C_ld=%.3f C_lp=%.3f I_xx=%.4f (run %d)\n",
                       p.C_ld, p.C_lp, p.I_xx, i);
            }
            if (r.hold_err_deg  > worst.hold_err_deg)  worst.hold_err_deg  = r.hold_err_deg;
            if (r.overshoot_deg > worst.overshoot_deg) worst.overshoot_deg = r.overshoot_deg;
            if (r.t_complete_s  > worst.t_complete_s)  worst.t_complete_s  = r.t_complete_s;
        }
        printf("mc: %d/%d PASS  worst: hold_err=%.2f deg overshoot=%.2f deg t=%.2f s\n",
               n - fails, n, worst.hold_err_deg, worst.overshoot_deg, worst.t_complete_s);
        return fails ? 1 : 0;
    }

    if (argc == 2 && !strcmp(argv[1], "sweep")) {
        static const char *names[] = {"nominal", "weak", "strong", "spin",
                                      "noisy", "trim"};
        for (float kpa = 0.5f; kpa <= 4.01f; kpa += 0.5f) {
            for (float kpr = 0.02f; kpr <= 0.161f; kpr += 0.02f) {
                roll_ctrl_config_t c = roll_ctrl_config_default();
                c.kp_angle = kpa;
                c.kp_rate  = kpr;
                sim_result_t worst = {0};
                int all_pass = 1;
                for (int s = 0; s < 6; s++) {
                    sim_params_t p = scenario(names[s]);
                    g_rng = 0xAC54C0DEu;
                    sim_result_t r = simulate(&p, &c, COAST_S, NULL);
                    if (!criteria_pass(&r)) all_pass = 0;
                    if (r.overshoot_deg > worst.overshoot_deg) worst.overshoot_deg = r.overshoot_deg;
                    if (r.hold_err_deg  > worst.hold_err_deg)  worst.hold_err_deg  = r.hold_err_deg;
                    if (r.t_complete_s  > worst.t_complete_s)  worst.t_complete_s  = r.t_complete_s;
                }
                printf("kp_angle=%.2f kp_rate=%.3f  worst: overshoot=%.2f hold=%.2f t=%.2f  %s\n",
                       kpa, kpr, worst.overshoot_deg, worst.hold_err_deg,
                       worst.t_complete_s, all_pass ? "PASS" : "fail");
            }
        }
        return 0;
    }

    fprintf(stderr, "usage: sim_roll nominal | scenario <name> | mc <N> <seed> | sweep\n");
    return 2;
}
