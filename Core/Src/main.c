/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "fatfs.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "usb_print.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "aggregator.h"
#include "control/flight_sm.h"
#include "utils/mathx.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SharedLog_t g_log = {0};

/* --- SD Task --- */
osThreadId_t sdTaskHandle;
const osThreadAttr_t sdTask_attributes = {
    .name       = "sdTask",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t) osPriorityNormal,  /* lower than aggregator */
};

/* --- Aggregator Task --- */
osThreadId_t aggregatorTaskHandle;
const osThreadAttr_t aggregatorTask_attributes = {
    .name       = "aggregatorTask",
    .stack_size = 1024 * 8,
    .priority   = (osPriority_t) osPriorityAboveNormal,
};

/* --- GNSS Task --- */
osThreadId_t gnssTaskHandle;
const osThreadAttr_t gnssTask_attributes = {
    .name       = "gnssTask",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * AXI SRAM layout (0x24000000, 64 KB MPU region):
 *   0x24000000  FATFS  (~560 B)
 *   0x24000400  FIL    (~580 B)
 *
 * These must stay in the MPU-protected, non-cacheable AXI SRAM region
 * so SDMMC DMA sees coherent data.
 */
static FATFS *const pSDFatFS = (FATFS *)0x24000000;
static FIL   *const pLogFile = (FIL *)  0x24000400;

int _write(int file, char *ptr, int len) {
    (void)file;
    if (len > 0)
        usb_print_write((const uint8_t *)ptr, (uint16_t)len);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_UART7_Init();   /* GNSS (MAX-M10S) — PE8 TX / PE7 RX */
  /* MX_TIM4_Init / MX_TIM1_Init kept for Cube parity; servo GPIOs disabled */
  MX_TIM4_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  HAL_Delay(1000);
  MX_TIM1_Init();
  MX_TIM15_Init();   /* buzzer on PE5 */
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port,  IMU_CS_Pin,  GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port,  MAG_CS_Pin,  GPIO_PIN_SET);
  HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);

#if 0
  /* ACS5: servo / lock PWM outputs disabled (Meteor pinout). */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
#endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_SMPS_2V5_SUPPLIES_LDO);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  SD card logging task.
  *
  * LED legend (mirrors the working reference project):
  *   LD2 ON solid        -> task started, waiting on f_mount
  *   LD2 OFF             -> f_mount returned
  *   LD1 blinks N times  -> f_mount failed, N = FRESULT error code
  *   LD3 ON solid        -> mount OK, file opened
  *   LD3 toggles         -> write loop running
  *   LD2 fast blink      -> f_open failed
  */
void StartSDTask(void *argument)
{
    memset((void*)pSDFatFS, 0, sizeof(FATFS));
    memset((void*)pLogFile, 0, sizeof(FIL));

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    osDelay(2000);

    /* -- Mount ------------------------------------------------------------ */
    FRESULT res = f_mount(pSDFatFS, SDPath, 1);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    if (res != FR_OK)
    {
        for (;;)
        {
            for (int i = 0; i < (int)res; i++)
            {
                HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
                osDelay(200);
                HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
                osDelay(200);
            }
            osDelay(2000);
        }
    }

    /* -- Find next available LOG_NNN.CSV ---------------------------------- */
    char filename[20];
    uint16_t log_num = 1;
    for (; log_num <= 999; log_num++)
    {
        snprintf(filename, sizeof(filename), "LOG_%03u.CSV", log_num);
        FILINFO fno;
        if (f_stat(filename, &fno) != FR_OK)
            break;
    }
    if (log_num > 999)
        snprintf(filename, sizeof(filename), "LOG_999.CSV");

    /* -- Open file and write CSV header ----------------------------------- */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

    if (f_open(pLogFile, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
    {
        for (;;)
        {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            osDelay(100);
        }
    }

    const char *header =
        "timestamp_ms,state,"
        "pos_x,pos_y,pos_z,"
        "vel_x,vel_y,vel_z,"
        "acc_x,acc_y,acc_z,"
        "roll_deg,pitch_deg,yaw_deg,"
        "ang_vel_x,ang_vel_y,ang_vel_z,"
        "accel_raw_x,accel_raw_y,accel_raw_z,"
        "gyro_raw_x,gyro_raw_y,gyro_raw_z,"
        "mag_raw_x,mag_raw_y,mag_raw_z,"
        "baro_alt,baro_press,baro_temp,"
        "ctrl_phase,ctrl_arm,theta_sp_deg,rate_sp_dps,theta_meas_deg,"
        "canard_cmd_deg,canard_us,"
        "gnss_ok,gnss_sv,gnss_lat_1e7,gnss_lon_1e7,gnss_h_msl_mm,"
        "gnss_n_m,gnss_e_m,gnss_d_m,"
        "gnss_vn,gnss_ve,gnss_vd\r\n";

    UINT bw;
    f_write(pLogFile, header, strlen(header), &bw);
    f_sync(pLogFile);

    uint32_t sync_counter = 0;

    /* -- Write loop - blocks on task notification from aggregator ---------- */
    for (;;)
    {
        /* Block until aggregator calls xTaskNotify - zero CPU while waiting.
         * pdTRUE clears the notification value on exit (auto-reset).
        Disable interrupts temporarily so the Aggregator cannot preempt the copy */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        SharedLog_t snap;


        taskENTER_CRITICAL();
        snap = g_log;
        taskEXIT_CRITICAL();
        const char *sname = flight_state_name((flight_state_t)snap.flight_state);

        char line[768];
        int len = snprintf(line, sizeof(line),
            "%lu,%s,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%.2f,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%.2f,"
            "%.2f,%.2f,%.2f,"
            "%u,%u,"
            "%.2f,%.2f,%.2f,"
            "%.2f,%u,"
            "%u,%u,%ld,%ld,%ld,"
            "%.2f,%.2f,%.2f,"
            "%.2f,%.2f,%.2f"
            "\r\n",
            (unsigned long)snap.timestamp_ms, sname,
            snap.pos[0],      snap.pos[1],      snap.pos[2],
            snap.vel[0],      snap.vel[1],      snap.vel[2],
            snap.acc[0],      snap.acc[1],      snap.acc[2],
            snap.ang_pos[0]*RAD2DEG, snap.ang_pos[1]*RAD2DEG, snap.ang_pos[2]*RAD2DEG,
            snap.ang_vel[0],  snap.ang_vel[1],  snap.ang_vel[2],
            snap.accel_raw[0], snap.accel_raw[1], snap.accel_raw[2],
            snap.gyro_raw[0],  snap.gyro_raw[1],  snap.gyro_raw[2],
            snap.mag_raw[0],  snap.mag_raw[1],  snap.mag_raw[2],
            snap.baro_alt, snap.baro_press, snap.baro_temp,
            (unsigned)snap.ctrl_phase, (unsigned)snap.ctrl_arm,
            snap.ctrl_theta_sp   * RAD2DEG,
            snap.ctrl_rate_sp    * RAD2DEG,
            snap.ctrl_theta_meas * RAD2DEG,
            snap.ctrl_canard_cmd * RAD2DEG,
            (unsigned)snap.ctrl_canard_us,
            (unsigned)snap.gnss_fix_valid, (unsigned)snap.gnss_num_sv,
            (long)snap.gnss_lat_1e7, (long)snap.gnss_lon_1e7, (long)snap.gnss_h_msl_mm,
            snap.gnss_ned_m[0], snap.gnss_ned_m[1], snap.gnss_ned_m[2],
            snap.gnss_vel_ned_mps[0], snap.gnss_vel_ned_mps[1], snap.gnss_vel_ned_mps[2]);

        /* snprintf reports the length it would have written, so a truncated row
         * would otherwise push stack bytes past the buffer onto the card. A
         * diverged estimator alone can do it: "%.3f" of -3.4e38 is 44
         * characters against the 9 a sane sample takes. */
        if (len < 0)
            len = 0;
        else if (len > (int)sizeof(line) - 1)
            len = (int)sizeof(line) - 1;

        f_write(pLogFile, line, len, &bw);

        /* Sync every 50 rows - ~500 ms at 10 ms/row.
         * Protects against data loss if power is cut during descent.    */
        if (snap.flight_state == FLIGHT_STATE_STANDBY || ++sync_counter >= 50)
        {
            f_sync(pLogFile);
            sync_counter = 0;
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        }
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) HAL_IncTick();
  if (htim->Instance == TIM1) { /* PWM - nothing needed here */ }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
