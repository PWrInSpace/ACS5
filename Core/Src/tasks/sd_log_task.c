#include "tasks/sd_log_task.h"

#include "main.h"
#include "fatfs.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "flight/flight_sm.h"
#include "utils/attributes.h"
#include "utils/mathx.h"

#include <stdio.h>
#include <string.h>

SharedLog_t g_log = {0};

/* SDMMC1 IDMA cannot reach DTCM, so the FatFs sector windows live in AXI SRAM. */
static FATFS s_sd_fatfs ATTR_SECTION(".dma_buf") ATTR_ALIGNED(32);
static FIL   s_log_file ATTR_SECTION(".dma_buf") ATTR_ALIGNED(32);
static FATFS *const pSDFatFS = &s_sd_fatfs;
static FIL   *const pLogFile = &s_log_file;

/**
  * @brief  SD card logging task.
  *
  * LED legend:
  *   LD2 ON solid        -> task started, waiting on f_mount
  *   LD2 OFF             -> f_mount returned
  *   LD1 blinks N times  -> f_mount failed, N = FRESULT error code
  *   LD3 ON solid        -> mount OK, file opened
  *   LD3 toggles         -> write loop running
  *   LD2 fast blink      -> f_open failed
  */
void StartSDTask(void *argument)
{
    (void)argument;
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

    /* -- Write loop - one row per notification from the flight task ------ */
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* The flight task preempts this one, so copy g_log without being interrupted. */
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

        /* Sync every 50 rows - ~400 ms at 8 ms/row.
         * Protects against data loss if power is cut during descent.    */
        if (snap.flight_state == FLIGHT_STATE_STANDBY || ++sync_counter >= 50)
        {
            f_sync(pLogFile);
            sync_counter = 0;
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        }
    }
}
