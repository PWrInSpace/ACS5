#include "tasks/flight_task.h"
#include "filter/attitude.h"
#include "control/axes.h"
#include "utils/control_rate.h"
#include "flight/flight_sm.h"
#include "flight/arming.h"
#include "flight/baro_datum.h"
#include "flight/estimator_policy.h"
#include "control/qbar.h"
#include "control/roll_ctrl.h"
#include "sensor_drivers/mag/mmc5983ma_math.h"
#include "sensor_drivers/servos/servo_t75.h"
#include "sensor_drivers/servos/canard_lock.h"
#ifdef ACS5_SERVO_BENCH
#include "sensor_drivers/servos/servo_bench.h"
#endif
#include "tasks/gnss_task.h"
#include "tasks/sd_log_task.h"
#include "utils/mathx.h"
#include "utils/quat.h"
#include "utils/fault_record.h"
#include "utils/timestamp.h"
#include "spi.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "usb_device.h"
#include "usb_print.h"
#include "buzzer/buzzer.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* -- Global Sensor Instances --------------------------------------------- */
static iim42653_t   g_imu;
static ms5611_t     g_baro;
static mmc5983ma_t  g_mag;
static attitude_t g_est;
static const float k_roll_axis[3] = { [BODY_AXIS_ROLL] = 1.0f };
static servo_bank_t75_t g_servos;


/* SPLIT V3 camera: MOSFET gate HIGH powers it. Only PC1 is driven. */
static void camera_set_enabled(bool on)
{
    GPIO_PinState s = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(CAM2_GPIO_Port, CAM2_Pin, s); /* PC1 */
}

static void halt_forever(const char *msg)
{
    for (;;)
    {
        USB_PRINTF("[FATAL] %s\r\n", msg);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* -- Timing & Logging ----------------------------------------------------- */
/* One MS5611 state step per call; at OSR 4096 a conversion needs 9.04 ms, so
 * 10 ms is the fastest schedulable poll that never finds the ADC busy. */
#define BARO_INTERVAL          HZ_TO_TICKS(100.0f)
/* Matches the MMC5983MA's continuous-measurement rate; polling faster only
 * spends an SPI status transaction to be told the sample has not changed. */
#define MAG_INTERVAL           HZ_TO_TICKS(100.0f)
#define LOG_INTERVAL_FLIGHT    SEC_TO_TICKS(0.008f)
#define LOG_INTERVAL_STANDBY   SEC_TO_TICKS(0.24f)   /* bench-testing rate */
#define STANDBY_RESET_INTERVAL SEC_TO_TICKS(24.0f)
#define TELEMETRY_INTERVAL     HZ_TO_TICKS(12.5f)
/* |B| reference: mean of the first pad samples; later samples must stay within MAG_REF_TOL. */
#define MAG_REF_SAMPLES        100u
#define MAG_REF_TOL            0.3f
/* A brown-out of the IMU alone powers its sensors off; reads then fail forever. */
#define IMU_REINIT_AFTER       SEC_TO_TICKS(0.1f)
#define IMU_REINIT_EVERY       SEC_TO_TICKS(1.0f)

/* 5x the worst publish gap (two polls on a temperature cycle); well inside the
 * estimator's 0.5 s baro blend, so a dead part cannot drag pos Z. */
#define BARO_MAX_AGE_US        100000u
/* Horizon of flight_sm's baro velocity filter: past this its output is frozen. */
#define BARO_VEL_MAX_AGE_US    500000u

/* Rising C-major arpeggio, ~0.7 s. */
static const BuzzerNote_t STARTUP_CHIME[] = {
    { 2093, 120 },
    { 2637, 120 },
    { 3136, 120 },
    { 4186, 300 },
};
static const uint16_t STARTUP_CHIME_LEN =
    (uint16_t)(sizeof(STARTUP_CHIME) / sizeof(STARTUP_CHIME[0]));


#ifdef ACS5_SERVO_BENCH
static void run_servo_bench(servo_bank_t75_t *servos)
{
    USB_PRINTF("[BENCH] Servo bench build - flight loop disabled\r\n");

    const uint32_t   t0          = osKernelGetTickCount();
    uint32_t         wake        = t0;
    servo_bench_cmd_t prev       = { .active_fin = -2 };
    float            prev_deg    = 0.0f;

    for (;;)
    {
        servo_bench_cmd_t cmd = servo_bench_cmd_at(osKernelGetTickCount() - t0);

        for (uint8_t i = 0; i < SERVO_T75_FIN_COUNT; i++)
            servo_bank_t75_set_angle_deg(servos, i, cmd.fin_deg[i]);
        servo_bank_t75_tick(servos, CONTROL_TICK_MS);
        canard_lock_set_us(cmd.lock_us);

        float active_deg = (cmd.active_fin >= 0) ? cmd.fin_deg[cmd.active_fin] : 0.0f;
        if (cmd.active_fin != prev.active_fin || cmd.lock_us != prev.lock_us ||
            active_deg != prev_deg)
        {
            if (cmd.active_fin >= 0)
                USB_PRINTF("[BENCH] %s %+.0f deg | target=%u us\r\n",
                           servo_bench_socket_name((uint8_t)cmd.active_fin), active_deg,
                           (unsigned)servos->target_us[cmd.active_fin]);
            else
                USB_PRINTF("[BENCH] LOCK %s pulse=%u us\r\n",
                           cmd.lock_us == CANARD_LOCK_US_LOCKED ? "CLOSED" : "OPEN",
                           (unsigned)cmd.lock_us);
            prev     = cmd;
            prev_deg = active_deg;
        }

        wake += CONTROL_TICK_MS;
        osDelayUntil(wake);
    }
}
#endif

/* ZYX Euler by axis; z is the airframe roll axis. */
static void print_attitude_deg(const float euler_rad[3])
{
    USB_PRINTF("EUL x:%7.2f y:%7.2f z:%7.2f deg\r\n",
               euler_rad[0] * RAD2DEG,
               euler_rad[1] * RAD2DEG,
               euler_rad[2] * RAD2DEG);
}


/* One full IMU bring-up: soft reset, ID check, configuration, sensors on. */
static bool imu_bring_up(const iim42653_config_t *cfg)
{
    return iim42653_init(&g_imu, &hspi2, IMU_CS_GPIO_Port, IMU_CS_Pin) &&
           iim42653_configure(&g_imu, cfg);
}

/* Print the fault that reset the previous run, if any. */
static void report_previous_fault(void)
{
    fault_record_t f;
    if (!fault_record_take(&f))
        return;
    USB_PRINTF("[FAULT] previous run: kind=%lu pc=0x%08lx lr=0x%08lx cfsr=0x%08lx hfsr=0x%08lx "
               "bfar=0x%08lx mmfar=0x%08lx at %s:%lu\r\n",
               (unsigned long)f.kind, (unsigned long)f.pc, (unsigned long)f.lr,
               (unsigned long)f.cfsr, (unsigned long)f.hfsr, (unsigned long)f.bfar,
               (unsigned long)f.mmfar, f.file ? f.file : "-", (unsigned long)f.line);
}

/* IMU is mandatory and halts on failure; baro and mag only report. */
static void sensors_init(void)
{
    /* -- IMU -------------------------------------------------------------- */
    /* No estimation or control without inertial data: a dead IMU keeps the vehicle
     * on the pad (no servo arming, no canard unlock, no control loop). */
    iim42653_config_t imu_cfg = iim42653_config_rocket_default();
    imu_cfg.axes = BOARD_IMU_REMAP;
    /* configure() fails the same way on a bad remap as on a dead part; name the remap. */
    if (!axes_remap_is_rotation(&imu_cfg.axes))
        halt_forever("BOARD_IMU_REMAP is not a rotation - check axes.h");
    bool imu_up = false;
    for (int attempt = 1; attempt <= 3 && !imu_up; attempt++)
    {
        imu_up = imu_bring_up(&imu_cfg);
        if (!imu_up)
        {
            USB_PRINTF("[ERROR] IMU bring-up FAILED (attempt %d/3)\r\n", attempt);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    if (!imu_up)
        halt_forever("IMU dead - control loop not started");
    USB_PRINTF("[OK] IMU Initialized\r\n");

    /* -- Barometer -------------------------------------------------------- */
    ms5611_config_t baro_cfg = ms5611_config_rocket_default();
    bool baro_up = false;
    for (int attempt = 1; attempt <= 3 && !baro_up; attempt++)
    {
        baro_up = ms5611_init(&g_baro, &hspi2, BARO_CS_GPIO_Port, BARO_CS_Pin, &baro_cfg);
        if (!baro_up)
            USB_PRINTF("[ERROR] Baro init FAILED (attempt %d/3)\r\n", attempt);
    }
    if (baro_up)
        USB_PRINTF("[OK] Baro Initialized\r\n");


    mmc5983ma_config_t mag_cfg = mmc5983ma_config_rocket_default();
    mag_cfg.axes = BOARD_MAG_REMAP;
    if (!axes_remap_is_rotation(&mag_cfg.axes))
        USB_PRINTF("[ERROR] BOARD_MAG_REMAP is not a rotation - check axes.h\r\n");
    if (!mmc5983ma_init(&g_mag, &hspi2, MAG_CS_GPIO_Port, MAG_CS_Pin))
        USB_PRINTF("[ERROR] Mag init FAILED (ID 0x%02X, expected 0x30)\r\n",
                   (unsigned)g_mag.product_id);
    else if (!mmc5983ma_configure(&g_mag, &mag_cfg))
        USB_PRINTF("[ERROR] Mag configure FAILED\r\n");
    else
        USB_PRINTF("[OK] Mag Initialized\r\n");
}

/* Servos to neutral and canards locked; halts if PWM cannot start. */
static void actuators_init(void)
{
    servo_t75_config_t servo_cfg = servo_t75_config_rocket_default();
    if (!servo_bank_t75_init(&g_servos, &htim3, &servo_cfg) ||
        !canard_lock_init(&htim2))
        halt_forever("Servo / lock PWM init failed");

    canard_lock_set_us(CANARD_LOCK_US_UNLOCKED);
    osDelay(3000);
    canard_lock_set_us(CANARD_LOCK_US_LOCKED);
    vTaskDelay(pdMS_TO_TICKS(50));

    servo_bank_t75_arm(&g_servos);
    USB_PRINTF("[OK] Servos armed (neutral), canards locked\r\n");
}

/* ===========================================================================
 *  Flight Task
 * =========================================================================== */
void StartFlightTask(void *argument)
{
    (void)argument;
    MX_USB_DEVICE_Init();

    vTaskDelay(pdMS_TO_TICKS(5000));
    timestamp_init();
    vTaskDelay(pdMS_TO_TICKS(3000));

    USB_PRINTF("--- ACS5 Flight Task Starting ---\r\n");
    report_previous_fault();

    sensors_init();
    actuators_init();

#ifdef ACS5_SERVO_BENCH
    /* Separate build so the bench sequence cannot exist in flight firmware. */
    run_servo_bench(&g_servos);
#endif

    camera_set_enabled(false);
    USB_PRINTF("[OK] Cameras OFF (SPLIT V3 MOSFET low)\r\n");

    Buzzer_Init(&htim15);
    Buzzer_PlayMelody(STARTUP_CHIME, STARTUP_CHIME_LEN);
    USB_PRINTF("[OK] Startup chime played\r\n");

    /* -- Flight Data & Timing -------------------------------------------- */
    flight_data_t flight_data = {0};

    const uint32_t TICK_MS = CONTROL_TICK_MS;
    uint32_t wake_time     = osKernelGetTickCount();
    uint32_t loop_counter  = 0;

    /* Every dt here is a constant, so an overrunning loop silently desynchronises
     * the integrators from wall time; measure and report the achieved rate. */
    uint32_t last_report_ms = wake_time;
    uint32_t overrun_count  = 0;
    uint32_t busy_max_cycles = 0;   /* longest loop body since the last report */

    /* State estimation */
    attitude_config_t est_cfg = attitude_config_default();
    attitude_init(&g_est, &est_cfg);

    /* -- State Machine ---------------------------------------------------- */
    static flight_sm_t g_sm;   /* static: keeps task stack small */
    flight_sm_config_t sm_cfg = flight_sm_config_default();
    flight_sm_init(&g_sm, &sm_cfg);

    /* -- Roll control ---------------------------------------------------- */
    static roll_ctrl_t g_rc;
    roll_ctrl_config_t rc_cfg = roll_ctrl_config_default();
    roll_ctrl_init(&g_rc, &rc_cfg);

    static ctrl_arm_t g_arm;
    ctrl_arm_config_t arm_cfg = ctrl_arm_config_default();
    ctrl_arm_init(&g_arm, &arm_cfg);

    baro_datum_t baro_datum   = {0};
    uint32_t imu_fail_streak  = 0;
    float    mag_ref_ut       = 0.0f;
    uint32_t mag_ref_n        = 0;
    static gnss_fix_t last_gnss;   /* static: keeps the task stack small */
    bool     calib_reported   = false;

    USB_PRINTF("Gravity calibration starting (%d samples = %.0f s)...\r\n",
               est_cfg.n_calib, (float)est_cfg.n_calib / CONTROL_LOOP_HZ);
    USB_PRINTF("Entering Main Loop @ %.0f Hz (STANDBY)...\r\n", CONTROL_LOOP_HZ);

    /* Ask GNSS task to latch pad datum once we have a 3D fix on the rail. */
    gnss_capture_datum();
    bool camera_on = false;

    for (;;)
    {
        const uint32_t body_start = timestamp_cycles();

        /* -- 1. IMU (CONTROL_LOOP_HZ) ------------------------------------- */
        /* A failed read leaves the previous sample in place; imu_ok is what
         * tells the estimator not to integrate it again. */
        bool imu_ok = iim42653_read(&g_imu, &flight_data.imu_sample);
        if (!imu_ok)
        {
            imu_fail_streak++;
            /* Printing costs real loop time, so report the onset and then at
             * most once a second. */
            if (imu_fail_streak == 1u || imu_fail_streak % SEC_TO_TICKS(1.0f) == 0u)
                USB_PRINTF("[WARN] IMU read failed (streak %lu)\r\n",
                           (unsigned long)imu_fail_streak);
            if (imu_fail_streak >= IMU_REINIT_AFTER &&
                (imu_fail_streak - IMU_REINIT_AFTER) % IMU_REINIT_EVERY == 0u)
            {
                iim42653_config_t imu_cfg = iim42653_config_rocket_default();
                imu_cfg.axes = BOARD_IMU_REMAP;
                bool ok = imu_bring_up(&imu_cfg);
                USB_PRINTF("[WARN] IMU re-init %s\r\n", ok ? "OK" : "FAILED");
            }
        }
        else
        {
            imu_fail_streak = 0;
        }

        /* Best-effort: a failed read leaves the previous sample in place and
         * only drops mag_valid for this tick, which Mahony treats as gyro-only. */
        bool mag_ok = false;
        if (loop_counter % MAG_INTERVAL == 0)
            mag_ok = mmc5983ma_read(&g_mag, &flight_data.mag_sample);
        if (mag_ok)
        {
            /* A nearby magnet or current loop shifts |B| before it saturates. */
            const float *b = flight_data.mag_sample.field_ut;
            const float mag = sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
            if (mag_ref_n < MAG_REF_SAMPLES && g_sm.state == FLIGHT_STATE_STANDBY)
                mag_ref_ut += (mag - mag_ref_ut) / (float)(++mag_ref_n);
            else
                mag_ok = mag_ref_n == MAG_REF_SAMPLES &&
                         mmc5983ma_magnitude_plausible(mag, mag_ref_ut, MAG_REF_TOL);
        }

        /* -- 2. Baro: one ms5611_update state per call; dt_s is driver-measured. */
        bool  baro_fresh = false;
        float baro_dt_s  = 0.0f;
        if (loop_counter % BARO_INTERVAL == 0)
        {
            ms5611_update(&g_baro);
            if (ms5611_has_new_data(&g_baro))
            {
                ms5611_sample_t s = ms5611_sample(&g_baro);
                /* Tracking the pad pressure keeps the datum fresh through a long hold. */
                baro_datum_action_t act =
                    baro_datum_on_sample(&baro_datum, g_sm.state == FLIGHT_STATE_STANDBY);
                if (act.retarget_qnh)
                    ms5611_set_qnh(&g_baro, s.pressure_pa);

                if (act.publish)
                {
                    flight_data.baro_sample = s;
                    baro_dt_s  = s.dt_s;      /* driver-measured; 0 = no usable interval */
                    baro_fresh = (s.dt_s > 0.0f);
                }
            }
        }

        /* 3. State Estimation */
        attitude_input_t est_in = {0};
        for (int i = 0; i < 3; i++) {
            est_in.accel_mps2[i] = flight_data.imu_sample.accel_mps2[i];
            est_in.gyro_rads[i]  = flight_data.imu_sample.gyro_rads[i];
            est_in.mag_ut[i]     = flight_data.mag_sample.field_ut[i];
        }
        est_in.imu_valid  = imu_ok;
        est_in.mag_valid  = mag_ok;
        estimator_gates_t gates = estimator_gates(g_sm.state, g_est.calib.done);
        est_in.corrections_inhibited = gates.corrections_inhibited;
        est_in.collecting_calib      = gates.collecting_calib;
        est_in.zupt_active           = gates.zupt_active;
        est_in.baro_alt_m = flight_data.baro_sample.altitude_m;
        /* A dead barometer keeps its last finite sample; only the age reveals it. */
        const uint32_t now_us = timestamp_us();
        est_in.baro_valid = baro_sample_fresh(flight_data.baro_sample.timestamp_us,
                                              now_us, BARO_MAX_AGE_US);
        est_in.baro_vel_z     = g_sm.baro_vel_z;
        est_in.baro_vel_valid = estimator_baro_vel_valid(
            g_sm.state, baro_datum.set,
            baro_sample_fresh(flight_data.baro_sample.timestamp_us, now_us, BARO_VEL_MAX_AGE_US));
        attitude_update(&g_est, &est_in);

        if (g_est.calib.done && !calib_reported) {
            calib_reported = true;
            USB_PRINTF("[CALIB] Done. |g|=%.3f m/s2 - filter is live.\r\n",
                       g_est.calib.mag_mean);
        }

        /* -- 4. State Machine --------------------------------------------- */
        flight_sm_input_t sm_in = {
            .az_mps2    = g_est.acc_original[BODY_AXIS_ROLL],
            .alt_m      = g_est.pos[2],
            .baro_alt_m = flight_data.baro_sample.altitude_m,
            .baro_fresh = baro_fresh,
            .baro_dt_s  = baro_dt_s,
            .vel_z_mps  = g_est.vel[2],
        };

        if (flight_sm_step(&g_sm, &sm_in))
        {
            /* Prints are off in flight; back on the pad they resume first so the
             * transition line itself goes out. */
            if (g_sm.state == FLIGHT_STATE_STANDBY)
                print_flag = 1u;

            USB_PRINTF("[STATE] -> %s | Az=%.1f m/s2  Alt=%.1f m  BaroVelZ=%.2f m/s\r\n",
                       flight_state_name(g_sm.state), sm_in.az_mps2,
                       g_est.pos[2], g_sm.baro_vel_z);
            if (g_sm.state == FLIGHT_STATE_STANDBY)
                USB_PRINTF("[STATE] false launch #%lu - no climb on baro or IMU\r\n",
                           (unsigned long)g_sm.false_launch_count);

            if (g_sm.state != FLIGHT_STATE_STANDBY)
                print_flag = 0u;

            /* Power SPLIT V3 camera(s) as soon as we leave the pad. */
            if (!camera_on && g_sm.state != FLIGHT_STATE_STANDBY)
            {
                camera_set_enabled(true);
                camera_on = true;
                USB_PRINTF("[CAM] SPLIT V3 MOSFET HIGH\r\n");
            }

            if (g_sm.state == FLIGHT_STATE_FALLING)
                USB_PRINTF("[LOG] cutoff in %lu ticks\r\n",
                           (unsigned long)sm_cfg.post_falling_timeout_ticks);
        }

        /* -- 5. Flight Control ------------------------------------------- */
        ctrl_arm_input_t arm_in = {
            .flight_state   = g_sm.state,
            .alt_m          = flight_data.baro_sample.altitude_m,
            .baro_datum_set = baro_datum.set,
        };
        ctrl_arm_state_t prev_arm = g_arm.state;
        bool maneuver_enabled = ctrl_arm_step(&g_arm, &arm_in);
        if (g_arm.state != prev_arm)
            USB_PRINTF("[CTRL] arm %d -> %d at %.1f m AGL\r\n",
                       (int)prev_arm, (int)g_arm.state,
                       flight_data.baro_sample.altitude_m);

        /* Estimator |v| suffices: vertical is baro-anchored and the vehicle is near
         * vertical through the maneuver. */
        float qbar_pa = qbar_from_baro(est_in.baro_valid,
                                       flight_data.baro_sample.pressure_pa,
                                       flight_data.baro_sample.temperature_c,
                                       g_est.vel);

        roll_ctrl_input_t rc_in = {
            .q                = g_est.q,
            .q_valid          = g_est.q_valid,
            .roll_rate_rads   = g_est.ang_vel[BODY_AXIS_ROLL],
            .rate_stale       = !g_est.rate_fresh,
            .qbar_pa          = qbar_pa,
            .maneuver_enabled = maneuver_enabled,
        };
        roll_ctrl_phase_t prev_phase = g_rc.phase;
        float delta_rad = roll_ctrl_step(&g_rc, &rc_in);
        if (g_rc.phase != prev_phase)
            USB_PRINTF("[CTRL] phase %d -> %d | theta=%.1f deg\r\n",
                       (int)prev_phase, (int)g_rc.phase, g_rc.theta_meas * RAD2DEG);

        /* -- 6. Servos ---------------------------------------------------- */
        /* Canards held neutral and locked: delta_rad is computed and logged, not actuated. */
        for (uint8_t i = 0; i < SERVO_T75_FIN_COUNT; i++)
            servo_bank_t75_set_angle_deg(&g_servos, i, 0.0f);
        servo_bank_t75_tick(&g_servos, TICK_MS);

        /* -- 7. SD Logging ------------------------------------------------ */
        bool do_log = false;
        if (!g_sm.mission_complete)
        {
            do_log = (g_sm.state == FLIGHT_STATE_STANDBY)
                     ? (loop_counter % LOG_INTERVAL_STANDBY == 0)
                     : (loop_counter % LOG_INTERVAL_FLIGHT  == 0);
        }

        if (do_log)
        {
            log_record_t rec = {0};
            rec.tick_ms      = osKernelGetTickCount();
            rec.flight_state = (uint8_t)g_sm.state;
            rec.ctrl_phase   = (uint8_t)g_rc.phase;
            rec.ctrl_arm     = (uint8_t)g_arm.state;
            const iim42653_sample_t *imu = &flight_data.imu_sample;
            rec.flags = (uint8_t)((imu->accel_clipped   ? LOG_FLAG_ACCEL_CLIPPED  : 0u) |
                                  (imu->gyro_clipped[0] ? LOG_FLAG_GYRO_CLIPPED_X : 0u) |
                                  (imu->gyro_clipped[1] ? LOG_FLAG_GYRO_CLIPPED_Y : 0u) |
                                  (imu->gyro_clipped[2] ? LOG_FLAG_GYRO_CLIPPED_Z : 0u) |
                                  (g_est.rate_fresh     ? LOG_FLAG_RATE_FRESH     : 0u) |
                                  (last_gnss.fix_valid  ? LOG_FLAG_GNSS_FIX       : 0u));
            for (int i = 0; i < 3; i++) {
                rec.pos_m[i]        = g_est.pos[i];
                rec.vel_mps[i]      = g_est.vel[i];
                rec.acc_mps2[i]     = g_est.acc[i];
                rec.ang_vel_rads[i] = g_est.ang_vel[i];
                /* Before the axis remap: the one thing a log cannot re-derive if the remap is wrong. */
                rec.accel_raw_mps2[i] = imu->accel_raw_mps2[i];
                rec.gyro_raw_rads[i]  = imu->gyro_raw_rads[i];
                rec.mag_raw_ut[i]     = flight_data.mag_sample.field_raw_ut[i];
            }
            float eul[3];
            attitude_euler(&g_est, eul);
            memcpy(rec.eul_rad, eul, sizeof eul);
            rec.roll_twist_rad      = quat_twist_angle(g_est.q, k_roll_axis);
            rec.baro_alt_m          = flight_data.baro_sample.altitude_m;
            rec.baro_press_pa       = flight_data.baro_sample.pressure_pa;
            rec.baro_temp_c         = flight_data.baro_sample.temperature_c;
            rec.ctrl_theta_sp_rad   = g_rc.theta_sp;
            rec.ctrl_rate_sp_rads   = g_rc.rate_sp;
            rec.ctrl_theta_meas_rad = g_rc.theta_meas;
            rec.ctrl_canard_cmd_rad = delta_rad;
            rec.ctrl_canard_us      = servo_bank_t75_current_pulse_us(&g_servos, 0);
            rec.false_launch_count  = (uint8_t)(g_sm.false_launch_count > 255u ? 255u
                                                                                : g_sm.false_launch_count);
            rec.gyro_bias_n         = g_est.gyro_bias_n;

            /* Non-blocking; a contended read keeps the previous snapshot. */
            (void)gnss_get_fix(&last_gnss);
            rec.gnss_num_sv   = last_gnss.pvt.num_sv;
            rec.gnss_lat_1e7  = last_gnss.pvt.lat_1e7;
            rec.gnss_lon_1e7  = last_gnss.pvt.lon_1e7;
            rec.gnss_h_msl_mm = last_gnss.pvt.h_msl_mm;
            memcpy(rec.gnss_ned_m, last_gnss.ned_m, sizeof rec.gnss_ned_m);
            memcpy(rec.gnss_vel_ned_mps, last_gnss.pvt.vel_ned_mps, sizeof rec.gnss_vel_ned_mps);

            log_publish(&rec);
        }

        /* -- 8. Telemetry -------------------------------------------------- */
        if (loop_counter % TELEMETRY_INTERVAL == 0)
        {
            uint32_t now_ms = osKernelGetTickCount();
            uint32_t span   = now_ms - last_report_ms;
            last_report_ms  = now_ms;

            HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
            USB_PRINTF("[%-7s] %.0f Hz ovr=%lu busy=%lu us | ", flight_state_name(g_sm.state),
                       span ? (1000.0f * TELEMETRY_INTERVAL / (float)span) : 0.0f,
                       (unsigned long)overrun_count,
                       (unsigned long)(busy_max_cycles / CYCLES_PER_US));
            busy_max_cycles = 0;
            {
                float euler[3];
                attitude_euler(&g_est, euler);
                print_attitude_deg(euler);
            }
        }

        /* -- 9. Standby drift reset (rocket static). Placed after logging:
         * docs/design.md#standby-drift-reset-ordering */
        if (g_sm.state == FLIGHT_STATE_STANDBY &&
            loop_counter > 0 && loop_counter % STANDBY_RESET_INTERVAL == 0)
        {
            attitude_reset(&g_est);
            uint32_t log_dropped, log_errors;
            log_stats(&log_dropped, &log_errors);
            USB_PRINTF("[LOG] dropped=%lu errors=%lu\r\n",
                       (unsigned long)log_dropped, (unsigned long)log_errors);
            USB_PRINTF("[RESET] Standby drift reset | BaroAlt=%.1f m  AccBias=[%.3f,%.3f,%.3f]"
                       "  GyroBias=[%.4f,%.4f,%.4f] deg/s n=%lu still=%.1f s\r\n",
                       flight_data.baro_sample.altitude_m,
                       g_est.acc_bias_w[0], g_est.acc_bias_w[1], g_est.acc_bias_w[2],
                       g_est.gyro_bias[0] * RAD2DEG, g_est.gyro_bias[1] * RAD2DEG,
                       g_est.gyro_bias[2] * RAD2DEG, (unsigned long)g_est.gyro_bias_n,
                       (float)g_est.still_ticks * CONTROL_DT_S);
        }

        /* -- 10. Timing ---------------------------------------------------- */
        const uint32_t body_cycles = timestamp_cycles() - body_start;
        if (body_cycles > busy_max_cycles)
            busy_max_cycles = body_cycles;
        loop_counter++;
        wake_time += TICK_MS;
        /* Signed compare so the count stays right across the tick-count wrap. */
        if ((int32_t)(osKernelGetTickCount() - wake_time) >= 0)
            overrun_count++;
        osDelayUntil(wake_time);
    }
}
