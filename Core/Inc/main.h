/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   ACS5 / ORC Meteor pinout (STM32H725VGT6).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
/* ORC Meteor (see ORC_Meteor.pdf MCU sheet) */
#define SD_DETECT_Pin GPIO_PIN_0
#define SD_DETECT_GPIO_Port GPIOD

#define LD1_Pin GPIO_PIN_7          /* LED_1 */
#define LD1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_3          /* LED_2 */
#define LD2_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_7          /* only two status LEDs on Meteor; alias LD1 */
#define LD3_GPIO_Port GPIOA

#define MAG_CS_Pin GPIO_PIN_8
#define MAG_CS_GPIO_Port GPIOD
#define BARO_CS_Pin GPIO_PIN_10
#define BARO_CS_GPIO_Port GPIOD
#define IMU_CS_Pin GPIO_PIN_13
#define IMU_CS_GPIO_Port GPIOD

#define IMU_INT1_Pin GPIO_PIN_11
#define IMU_INT1_GPIO_Port GPIOD
#define IMU_INT2_Pin GPIO_PIN_12
#define IMU_INT2_GPIO_Port GPIOD
#define MAG_INT_Pin GPIO_PIN_9
#define MAG_INT_GPIO_Port GPIOD

/* SPLIT V3 camera power: MOSFET gate HIGH = camera on */
#define CAM1_Pin GPIO_PIN_0
#define CAM1_GPIO_Port GPIOC
#define CAM2_Pin GPIO_PIN_1
#define CAM2_GPIO_Port GPIOC
#define CAM3_Pin GPIO_PIN_2
#define CAM3_GPIO_Port GPIOC

#define GPS_RST_Pin GPIO_PIN_6
#define GPS_RST_GPIO_Port GPIOC
#define GPS_INT_Pin GPIO_PIN_7
#define GPS_INT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
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

extern SharedLog_t g_log;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
