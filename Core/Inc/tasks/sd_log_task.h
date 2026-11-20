/* SD-card flight log: the flight task fills g_log and notifies the SD task. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pos[3];
    float vel[3];
    float acc[3];
    float ang_pos[3];   // roll, pitch, yaw (rad)
    float ang_vel[3];
    float baro_alt;
    float baro_press;
    float baro_temp;
    float accel_raw[3];
    float gyro_raw[3];
    float mag_raw[3];
    float ctrl_theta_sp;    // rad
    float ctrl_rate_sp;     // rad/s
    float ctrl_theta_meas;  // rad, unwrapped maneuver angle
    float ctrl_canard_cmd;  // rad, commanded deflection (identical on all four fins)
    uint16_t ctrl_canard_us; // fin 0 pulse after the driver's slew limit
    uint8_t flight_state;
    uint8_t ctrl_phase;
    uint8_t ctrl_arm;
    uint32_t timestamp_ms;
    /* GNSS snapshot fields (copied under mutex at log time) */
    bool     gnss_fix_valid;
    uint8_t  gnss_num_sv;
    float    gnss_ned_m[3];
    float    gnss_vel_ned_mps[3];
    int32_t  gnss_lat_1e7;
    int32_t  gnss_lon_1e7;
    int32_t  gnss_h_msl_mm;
} SharedLog_t;

/* Written by the flight task, which outranks the SD task; read under a critical section. */
extern SharedLog_t g_log;

/* FreeRTOS task entry (registered via osThreadNew in freertos.c). */
void StartSDTask(void *argument);
