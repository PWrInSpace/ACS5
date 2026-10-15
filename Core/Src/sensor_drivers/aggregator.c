#include "aggregator.h"   /* flight_data_t + iim42653/ms5611/mmc5983ma headers */
#include "filter/attitude.h"
#include "control/axes.h"
#include "control/control_rate.h"
#include "control/flight_sm.h"
#include "control/arming.h"
#include "control/roll_ctrl.h"
/* ACS5: servo bank kept in tree but not driven (Meteor pinout / no canards). */
#if 0
#include "servos/servo_t75.h"
#endif
#include "flight/flight_data.h"
#include "utils/mathx.h"
#include "utils/timestamp.h"
#include "spi.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "usb_print.h"
#include "buzzer/buzzer.h"
#include <stdbool.h>
#include <string.h>

/* -- Global Sensor Instances --------------------------------------------- */
static iim42653_t   g_imu;
static ms5611_t     g_baro;
static mmc5983ma_t  g_mag;
static attitude_t g_est;

extern void MX_USB_DEVICE_Init(void);
#if 0
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
#endif
extern TIM_HandleTypeDef htim15;
extern osThreadId_t sdTaskHandle;

/* SPLIT V3: MOSFET gate HIGH powers the camera. PC0/PC2 held off for now. */
static void camera_set_enabled(bool on)
{
    GPIO_PinState s = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#if 0
    HAL_GPIO_WritePin(CAM1_GPIO_Port, CAM1_Pin, s); /* PC0 */
#endif
    HAL_GPIO_WritePin(CAM2_GPIO_Port, CAM2_Pin, s); /* PC1 */
#if 0
    HAL_GPIO_WritePin(CAM3_GPIO_Port, CAM3_Pin, s); /* PC2 */
#endif
}

/* -- Timing & Logging ----------------------------------------------------- */
/* One MS5611 state step per call; at OSR 4096 a conversion needs 9.04 ms, so
 * 10 ms is the fastest schedulable poll that never finds the ADC busy. */
#define BARO_INTERVAL          HZ_TO_TICKS(100.0f)
/* Matches the MMC5983MA's continuous-measurement rate; polling faster only
 * spends an SPI status transaction to be told the sample has not changed. */
#define MAG_INTERVAL           HZ_TO_TICKS(100.0f)
#define LOG_INTERVAL_FLIGHT    SEC_TO_TICKS(0.008f)
#define LOG_INTERVAL_STANDBY   SEC_TO_TICKS(0.24f)   /* lowered for bench testing */
#define STANDBY_RESET_INTERVAL SEC_TO_TICKS(24.0f)
#define TELEMETRY_INTERVAL     HZ_TO_TICKS(12.5f)

/* Five times the worst legitimate gap between publishes: pressure lands on
 * every poll except the one cycle in MS5611_TEMP_EVERY_N spent on temperature,
 * which stretches that gap to two polls. Well inside the estimator's 0.5 s baro
 * blend constant, so a dead part is caught before it can drag pos Z. */
#define BARO_MAX_AGE_US        100000u

static const BuzzerNote_t BARKA_CHORUS[] = {
    { 2093, 1947 },
    { 3520,  968 },
    { 3520,  968 },
    { 3520,  316 },
    { 3951,  316 },
    { 4186,  316 },
    { 3951,  316 },
    { 3520,  316 },
    { 3136,  316 },
    { 3136,  968 },
    { 3136,  968 },
    { 2960,  642 },
    { 2637,  316 },
    { 2637,  968 },
    { 2637,  968 },
    { 2637,  316 },
    { 2794,  316 },
    { 3136,  316 },
    { 2794,  316 },
    { 2637,  316 },
    { 2349,  316 },
    { 2637, 1946 },
    { 2093,  968 },
    { 2093,  968 },
    { 3520,  968 },
    { 3520,  968 },
    { 3520,  316 },
    { 3951,  316 },
    { 4186,  316 },
    { 3951,  316 },
    { 3520,  316 },
    { 3136,  316 },
    { 3136,  968 },
    { 3136,  968 },
    { 2794,  642 },
    { 2637,  316 },
    { 2794,  968 },
    { 2794, 1284 },
    { 2349,  316 },
    { 2637,  316 },
    { 2794,  316 },
    { 2637,  316 },
    { 2349,  316 },
    { 2093, 1294 },
};
static const uint16_t BARKA_CHORUS_LEN =
    (uint16_t)(sizeof(BARKA_CHORUS) / sizeof(BARKA_CHORUS[0]));


#if 0
static void set_lock_servo(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}
#endif


static void print_attitude_deg(const float euler_rad[3])
{
    USB_PRINTF("ATT Pitch:%7.2f  Yaw:%7.2f  Roll:%7.2f\r\n",
               euler_rad[1] * RAD2DEG,
               euler_rad[2] * RAD2DEG,
               euler_rad[0] * RAD2DEG);
}


/* ===========================================================================
 *  Aggregator Task
 * =========================================================================== */
void StartAggregator(void *argument)
{
    (void)argument;
    MX_USB_DEVICE_Init();

    vTaskDelay(pdMS_TO_TICKS(5000));
    timestamp_init();
    vTaskDelay(pdMS_TO_TICKS(3000));

    USB_PRINTF("--- ACS5 Aggregator Task Starting ---\r\n");

    /* -- IMU -------------------------------------------------------------- */
    /* Estimation and roll control are unrecoverable without inertial data, and
     * a configure that fails partway leaves the part powered down while still
     * reporting initialized. A dead IMU therefore has to keep the vehicle on
     * the pad: no servo arming, no canard unlock, no control loop. */
    iim42653_config_t imu_cfg = iim42653_config_rocket_default();
    imu_cfg.axes = BOARD_IMU_REMAP;
    /* configure() rejects a bad remap the same way it rejects a dead part, and
     * BOARD_*_REMAP is a build-time constant - say which one it is. */
    if (!axes_remap_is_rotation(&imu_cfg.axes))
        for (;;)
        {
            USB_PRINTF("[FATAL] BOARD_IMU_REMAP is not a rotation - check axes.h\r\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    bool imu_up = false;
    for (int attempt = 1; attempt <= 3 && !imu_up; attempt++)
    {
        if (!iim42653_init(&g_imu, &hspi2, IMU_CS_GPIO_Port, IMU_CS_Pin))
            USB_PRINTF("[ERROR] IMU init FAILED (attempt %d/3)\r\n", attempt);
        else if (!iim42653_configure(&g_imu, &imu_cfg))
            USB_PRINTF("[ERROR] IMU configure FAILED (attempt %d/3)\r\n", attempt);
        else
            imu_up = true;

        if (!imu_up)
            vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!imu_up)
    {
        for (;;)
        {
            USB_PRINTF("[FATAL] IMU dead - control loop not started\r\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    USB_PRINTF("[OK] IMU Initialized\r\n");

    /* -- Barometer -------------------------------------------------------- */
    ms5611_config_t baro_cfg = ms5611_config_rocket_default();
    if (!ms5611_init(&g_baro, &hspi2, BARO_CS_GPIO_Port, BARO_CS_Pin, &baro_cfg))
        USB_PRINTF("[ERROR] Baro init FAILED\r\n");
    else
        USB_PRINTF("[OK] Baro Initialized\r\n");


    mmc5983ma_config_t mag_cfg = mmc5983ma_config_rocket_default();
    mag_cfg.axes = BOARD_MAG_REMAP;
    if (!axes_remap_is_rotation(&mag_cfg.axes))
        USB_PRINTF("[ERROR] BOARD_MAG_REMAP is not a rotation - check axes.h\r\n");
    if (!mmc5983ma_init(&g_mag, &hspi2, MAG_CS_GPIO_Port, MAG_CS_Pin))
        USB_PRINTF("[ERROR] Mag init FAILED\r\n");
    else if (!mmc5983ma_configure(&g_mag, &mag_cfg))
        USB_PRINTF("[ERROR] Mag configure FAILED\r\n");
    else
        USB_PRINTF("[OK] Mag Initialized\r\n");

    /* -- Servos (disabled on ACS5 / Meteor) -------------------------------- */
#if 0
    static servo_bank_t75_t g_servos;
    servo_t75_config_t servo_cfg = servo_t75_config_rocket_default();
    if (!servo_bank_t75_init(&g_servos, &htim4, &servo_cfg))
        USB_PRINTF("[ERROR] Servo bank init FAILED\r\n");

    set_lock_servo(1050);
    osDelay(3000);
    set_lock_servo(1900);
    vTaskDelay(pdMS_TO_TICKS(50));

    servo_bank_t75_arm(&g_servos);
    USB_PRINTF("[OK] Servos armed (neutral)\r\n");
#else
    USB_PRINTF("[OK] Servos DISABLED (ACS5)\r\n");
#endif

    camera_set_enabled(false);
    USB_PRINTF("[OK] Cameras OFF (SPLIT V3 MOSFET low)\r\n");

    USB_PRINTF("[BUZZER] Playing Barka (~20 s)...\r\n");
    Buzzer_Init(&htim15);
    Buzzer_PlayMelody(BARKA_CHORUS, BARKA_CHORUS_LEN);
    osDelay(1000);
    USB_PRINTF("[BUZZER] Done.\r\n");

    /* -- Flight Data & Timing -------------------------------------------- */
    flight_data_t flight_data = {0};

    const uint32_t TICK_MS = CONTROL_TICK_MS;
    uint32_t wake_time     = osKernelGetTickCount();
    uint32_t loop_counter  = 0;

    /* Every dt in this task is a constant, not a measurement, so a loop body
     * that outgrows TICK_MS would slip osDelayUntil silently and desynchronise
     * every integrator from wall time. Report the achieved rate instead. */
    uint32_t last_report_ms = wake_time;
    uint32_t overrun_count  = 0;

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

    bool     baro_datum_set   = false;
    uint32_t imu_fail_streak  = 0;
    bool     calib_reported   = false;

    USB_PRINTF("Gravity calibration starting (%d samples = %.0f s)...\r\n",
               est_cfg.n_calib, (float)est_cfg.n_calib / CONTROL_LOOP_HZ);
    USB_PRINTF("Entering Main Loop @ %.0f Hz (STANDBY)...\r\n", CONTROL_LOOP_HZ);

    /* Ask GNSS task to latch pad datum once we have a 3D fix on the rail. */
    gnss_capture_datum();
    bool camera_on = false;

    for (;;)
    {
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

        /* -- 2. Baro: ms5611_update advances one state per call. In steady
         * state one poll reads D1 and publishes; the temperature refresh costs
         * an extra poll, so publishes track the poll rate less that one cycle
         * in MS5611_TEMP_EVERY_N. The driver measures each sample's dt_s (with
         * the timestamp-wrap guard); we just forward it. */
        bool  baro_fresh = false;
        float baro_dt_s  = 0.0f;
        if (loop_counter % BARO_INTERVAL == 0)
        {
            ms5611_update(&g_baro);
            if (ms5611_has_new_data(&g_baro))
            {
                ms5611_sample_t s = ms5611_sample(&g_baro);
                bool publish = true;

                /* On the pad the current pressure IS the zero reference, so the
                 * whole system speaks metres above the launch site and the day's
                 * weather cancels itself. Tracking rather than latching keeps the
                 * reference fresh through a long hold; it stops on its own at
                 * launch detection because the state stops being STANDBY. */
                if (g_sm.state == FLIGHT_STATE_STANDBY)
                {
                    ms5611_set_qnh(&g_baro, s.pressure_pa);

                    /* altitude_m is computed inside the driver at publish time
                     * against the QNH in force then, so this sample still carries
                     * the previous datum - roughly 700 m at a 500 m ASL site.
                     * Publishing it would anchor attitude.pos[2] there and
                     * spike flight_sm.baro_vel_z past 1000 m/s for ~2 s. Drop it;
                     * the next sample is AGL. */
                    if (!baro_datum_set)
                    {
                        baro_datum_set = true;
                        publish        = false;
                    }
                }

                if (publish)
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
        /* Standby and recovery are the phases where the accelerometer really is
         * measuring gravity - on the pad, and hanging under the chute. Between
         * them it measures thrust and drag, and levelling against those walks
         * the attitude away from the truth the maneuver is steering by. */
        est_in.corrections_inhibited =
            (g_sm.state == FLIGHT_STATE_ACCELERATION ||
             g_sm.state == FLIGHT_STATE_FREE_FLIGHT  ||
             g_sm.state == FLIGHT_STATE_FALLING);
        est_in.baro_alt_m = flight_data.baro_sample.altitude_m;
        /* The estimator holds the last altitude between publishes, which is
         * correct; a dead barometer looks identical unless the sample age is
         * checked, and the blend would drag pos Z onto a frozen value. */
        est_in.baro_valid =
            (flight_data.baro_sample.timestamp_us != 0) &&
            ((timestamp_us() - flight_data.baro_sample.timestamp_us) < BARO_MAX_AGE_US);
        est_in.baro_vel_z     = g_sm.baro_vel_z;
        est_in.baro_vel_valid = (g_sm.state != FLIGHT_STATE_STANDBY) || baro_datum_set;
        est_in.collecting_calib = (g_sm.state == FLIGHT_STATE_STANDBY) && !g_est.calib.done;
        est_in.zupt_active      = (g_sm.state == FLIGHT_STATE_STANDBY);
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
        };

        if (flight_sm_step(&g_sm, &sm_in))
        {
            USB_PRINTF("[STATE] -> %s | Az=%.1f m/s2  Alt=%.1f m  BaroVelZ=%.2f m/s\r\n",
                       flight_state_name(g_sm.state), sm_in.az_mps2,
                       g_est.pos[2], g_sm.baro_vel_z);

            /* Launch / leave pad: stop USB prints for the rest of the flight.
             * Cleared after the transition line so that last message still goes out. */
            if (g_sm.state != FLIGHT_STATE_STANDBY)
                print_flag = 1u;

            /* Power SPLIT V3 camera(s) as soon as we leave the pad. */
            if (!camera_on && g_sm.state != FLIGHT_STATE_STANDBY)
            {
                camera_set_enabled(true);
                camera_on = true;
                USB_PRINTF("[CAM] SPLIT V3 MOSFET HIGH\r\n");
            }

#if 0
            if (g_sm.state == FLIGHT_STATE_FREE_FLIGHT)
            {
                set_lock_servo(1050);
                USB_PRINTF("[LOCK] Canards UNLOCKED\r\n");
            }
#endif
            if (g_sm.state == FLIGHT_STATE_FALLING)
                USB_PRINTF("[LOG] cutoff in %lu ticks\r\n",
                           (unsigned long)sm_cfg.post_falling_timeout_ticks);
        }

        /* -- 5. Flight Control ------------------------------------------- */
        ctrl_arm_input_t arm_in = {
            .flight_state   = g_sm.state,
            .alt_m          = flight_data.baro_sample.altitude_m,
            .baro_datum_set = baro_datum_set,
        };
        ctrl_arm_state_t prev_arm = g_arm.state;
        bool maneuver_enabled = ctrl_arm_step(&g_arm, &arm_in);
        if (g_arm.state != prev_arm)
            USB_PRINTF("[CTRL] arm %d -> %d at %.1f m AGL\r\n",
                       (int)prev_arm, (int)g_arm.state,
                       flight_data.baro_sample.altitude_m);

        /* Dynamic pressure for gain scheduling. Density from the barometer's own
         * pressure and temperature (dry-air gas constant), speed from the
         * estimator - its vertical component is anchored to the barometer, and
         * the vehicle is near vertical through the maneuver, so |v| is sound
         * even though the lateral components are dead-reckoned.
         *
         * The same age check the estimator uses gates this: a dead barometer
         * keeps returning its last finite sample, so a finiteness test alone
         * would compute density from a frozen reading. Stale or non-finite
         * leaves qbar at zero, which roll_ctrl reads as "no measurement" and
         * answers with fixed gains rather than by standing the canards down. */
        float qbar_pa = 0.0f;
        if (est_in.baro_valid &&
            isfinite(flight_data.baro_sample.pressure_pa) &&
            isfinite(flight_data.baro_sample.temperature_c))
        {
            float t_kelvin = flight_data.baro_sample.temperature_c + 273.15f;
            float rho = flight_data.baro_sample.pressure_pa / (287.05f * t_kelvin);
            float vsq = g_est.vel[0] * g_est.vel[0] +
                        g_est.vel[1] * g_est.vel[1] +
                        g_est.vel[2] * g_est.vel[2];
            if (isfinite(rho) && isfinite(vsq) && rho > 0.0f)
                qbar_pa = 0.5f * rho * vsq;
        }

        roll_ctrl_input_t rc_in = {
            .q                = g_est.q,
            .q_valid          = g_est.q_valid,
            .roll_rate_rads   = g_est.ang_vel[BODY_AXIS_ROLL],
            .qbar_pa          = qbar_pa,
            .maneuver_enabled = maneuver_enabled,
        };
        roll_ctrl_phase_t prev_phase = g_rc.phase;
        float delta_rad = roll_ctrl_step(&g_rc, &rc_in);
        if (g_rc.phase != prev_phase)
            USB_PRINTF("[CTRL] phase %d -> %d | theta=%.1f deg\r\n",
                       (int)prev_phase, (int)g_rc.phase, g_rc.theta_meas * RAD2DEG);

        /* -- 6. Servos (disabled) ---------------------------------------- */
#if 0
        for (uint8_t i = 0; i < SERVO_T75_FIN_COUNT; i++)
            servo_bank_t75_set_angle_deg(&g_servos, i, delta_rad * RAD2DEG);
        servo_bank_t75_tick(&g_servos, TICK_MS);
#else
        (void)delta_rad;
#endif

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
            /* Write unconditionally - the SD task owns the consumer side.
             * Dropping the !g_log.fresh guard was the original logging bug:
             * if the SD task stalls, fresh stays true and nothing gets packed. */
            for (int i = 0; i < 3; i++) {
                g_log.pos[i]       = g_est.pos[i];
                g_log.vel[i]       = g_est.vel[i];
                g_log.acc[i]       = g_est.acc[i];
                g_log.ang_vel[i]   = g_est.ang_vel[i];
                /* Logged before the axis remap: the mounting is the one thing a
                 * flight log cannot re-derive if the remap itself is wrong. */
                g_log.accel_raw[i] = flight_data.imu_sample.accel_raw_mps2[i];
                g_log.gyro_raw[i]  = flight_data.imu_sample.gyro_raw_rads[i];
                g_log.mag_raw[i]   = flight_data.mag_sample.field_raw_ut[i];
            }
            attitude_euler(&g_est, g_log.ang_pos);
            g_log.baro_alt     = flight_data.baro_sample.altitude_m;
            g_log.baro_press   = flight_data.baro_sample.pressure_pa;
            g_log.baro_temp    = flight_data.baro_sample.temperature_c;
            g_log.flight_state = (uint8_t)g_sm.state;
            g_log.ctrl_phase      = (uint8_t)g_rc.phase;
            g_log.ctrl_arm        = (uint8_t)g_arm.state;
            g_log.ctrl_theta_sp   = g_rc.theta_sp;
            g_log.ctrl_rate_sp    = g_rc.rate_sp;
            g_log.ctrl_theta_meas = g_rc.theta_meas;
            g_log.ctrl_canard_cmd = delta_rad;
#if 0
            g_log.ctrl_canard_us  = servo_bank_t75_current_pulse_us(&g_servos, 0);
#else
            g_log.ctrl_canard_us  = 0;
#endif
            g_log.timestamp_ms = osKernelGetTickCount();

            /* GNSS snapshot for the CSV row (non-blocking; skip if contended). */
            if (g_gnss_mutex != NULL &&
                osMutexAcquire(g_gnss_mutex, 0) == osOK)
            {
                g_log.gnss_fix_valid = g_gnss_fix.fix_valid;
                g_log.gnss_num_sv     = g_gnss_fix.num_sv;
                g_log.gnss_lat_1e7    = g_gnss_fix.lat_1e7;
                g_log.gnss_lon_1e7    = g_gnss_fix.lon_1e7;
                g_log.gnss_h_msl_mm   = g_gnss_fix.h_msl_mm;
                memcpy(g_log.gnss_ned_m, g_gnss_fix.ned_m, sizeof(g_log.gnss_ned_m));
                memcpy(g_log.gnss_vel_ned_mps, g_gnss_fix.vel_ned_mps,
                       sizeof(g_log.gnss_vel_ned_mps));
                osMutexRelease(g_gnss_mutex);
            }

            __DMB();  /* ensure all writes to g_log are visible before waking SD task */
            xTaskNotify((TaskHandle_t)sdTaskHandle, 0x01, eSetBits);
        }

        /* -- 8. Telemetry -------------------------------------------------- */
        if (loop_counter % TELEMETRY_INTERVAL == 0)
        {
            uint32_t now_ms = osKernelGetTickCount();
            uint32_t span   = now_ms - last_report_ms;
            last_report_ms  = now_ms;

            HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
            USB_PRINTF("[%-7s] %.0f Hz ovr=%lu | ", flight_state_name(g_sm.state),
                       span ? (1000.0f * TELEMETRY_INTERVAL / (float)span) : 0.0f,
                       (unsigned long)overrun_count);
            {
                float euler[3];
                attitude_euler(&g_est, euler);
                print_attitude_deg(euler);
            }
        }

        /* -- 9. Standby drift reset - rocket is static, kill accumulated drift.
         * Runs after the log and telemetry blocks because STANDBY_RESET_INTERVAL
         * is a whole multiple of both cadences: resetting first would hand every
         * reset-tick row zeroed vel/pos and an identity attitude, erasing the one
         * sample per period that records how much drift there was to discard.
         * Safe here - flight_sm reads only az_mps2 in STANDBY, and the reset
         * leaves acc_original untouched. */
        if (g_sm.state == FLIGHT_STATE_STANDBY &&
            loop_counter > 0 && loop_counter % STANDBY_RESET_INTERVAL == 0)
        {
            attitude_reset(&g_est);
            USB_PRINTF("[RESET] Standby drift reset | BaroAlt=%.1f m  AccBias=[%.3f,%.3f,%.3f]\r\n",
                       flight_data.baro_sample.altitude_m,
                       g_est.acc_bias_w[0], g_est.acc_bias_w[1], g_est.acc_bias_w[2]);
        }

        /* -- 10. Timing ---------------------------------------------------- */
        loop_counter++;
        wake_time += TICK_MS;
        /* Signed compare so the count stays right across the tick-count wrap. */
        if ((int32_t)(osKernelGetTickCount() - wake_time) >= 0)
            overrun_count++;
        osDelayUntil(wake_time);
    }
}
