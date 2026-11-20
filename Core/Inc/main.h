/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   ACS5 / ORC Meteor pinout (STM32H725VGT6).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAM2_Pin GPIO_PIN_1
#define CAM2_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_4
#define LD2_GPIO_Port GPIOA
#define MAG_CS_Pin GPIO_PIN_8
#define MAG_CS_GPIO_Port GPIOD
#define MAG_INT_Pin GPIO_PIN_9
#define MAG_INT_GPIO_Port GPIOD
#define BARO_CS_Pin GPIO_PIN_10
#define BARO_CS_GPIO_Port GPIOD
#define IMU_INT1_Pin GPIO_PIN_11
#define IMU_INT1_GPIO_Port GPIOD
#define IMU_INT2_Pin GPIO_PIN_12
#define IMU_INT2_GPIO_Port GPIOD
#define IMU_CS_Pin GPIO_PIN_13
#define IMU_CS_GPIO_Port GPIOD
#define GPS_RST_Pin GPIO_PIN_6
#define GPS_RST_GPIO_Port GPIOC
#define GPS_INT_Pin GPIO_PIN_7
#define GPS_INT_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_8
#define LD1_GPIO_Port GPIOA
#define SD_DETECT_Pin GPIO_PIN_1
#define SD_DETECT_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
/* Only two status LEDs on Meteor; LD3 aliases LD1. */
#define LD3_Pin LD1_Pin
#define LD3_GPIO_Port LD1_GPIO_Port

/* SPLIT V3 camera power, MOSFET gate HIGH = on. PC0/PC2 stay unconfigured
 * until those cameras are fitted; PC1 (CAM2) is configured in CubeMX. */
#define CAM1_Pin GPIO_PIN_0
#define CAM1_GPIO_Port GPIOC
#define CAM3_Pin GPIO_PIN_2
#define CAM3_GPIO_Port GPIOC

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
