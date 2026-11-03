/*
 * ACS5 - GNSS task. Owns the FreeRTOS boundary for the MAX-M10S driver.
 *
 */


#include "stm32h7xx_hal.h"

#include "cmsis_os2.h"

#include "tasks/gnss_task.h"
#include "gnss/max_m10s.h"

#include <string.h>

/*------------------------------------------------------------------- globals */

static gnss_fix_t      s_fix;        // zero-initialised at startup
static osMutexId_t     s_fix_mutex;  // guards s_fix; NULL until gnss_task() starts
static osSemaphoreId_t s_idle_sem;   // binary semaphore, given from UART idle ISR

static max_m10s_driver_t s_drv;

#define GNSS_TASK_WAKE_TIMEOUT_MS 1000u

/* NED datum and its capture request. Owned by this task; the request flag is
 * set from another task by gnss_capture_datum(). Zero-initialised, so the datum
 * starts out invalid. */
static ubx_datum_t   s_datum;
static volatile bool s_capture_pending;

/*--------------------------------------------------------------- idle callback */

// The STM32 HAL calls this from the UART idle IRQ.  
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)size;
    if (huart == &MAX_M10S_HUART_HANDLE)
        osSemaphoreRelease(s_idle_sem);
}

// The HAL calls this after a UART receive error (overrun, noise, framing...). It
// has already aborted the DMA, so no idle event will ever come again unless the
// driver restarts reception: flag it and wake the task to do so.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &MAX_M10S_HUART_HANDLE)
    {
        max_m10s_on_uart_error(&s_drv);
        osSemaphoreRelease(s_idle_sem);
    }
}

/* -------------------------------------------------------------- capture datum
 *
 * Called by the flight state machine when the rocket is on the 
 * pad with a confirmed 3D fix. The task latches
 * the datum from the next valid fix. */
void gnss_capture_datum(void)
{
    s_capture_pending = true;
}

/* ----------------------------------------------------------------- get fix
 *
 * Non-blocking so a low-rate consumer (the logger) can never be stalled by the
 * GNSS task holding the mutex. */
bool gnss_get_fix(gnss_fix_t *out)
{
    if (out == NULL || s_fix_mutex == NULL)
        return false;
    if (osMutexAcquire(s_fix_mutex, 0) != osOK)
        return false;
    *out = s_fix;
    osMutexRelease(s_fix_mutex);
    return true;
}

/*-------------------------------------------------------------------- task */

void gnss_task(void *arg)
{
    (void)arg;

    s_fix_mutex    = osMutexNew(NULL); 
    s_idle_sem = osSemaphoreNew(1, 0, NULL);

    max_m10s_status_t cfg = max_m10s_init(&s_drv);
    if (cfg == MAX_M10S_OK)
        cfg = max_m10s_configure(&s_drv);
    if (cfg != MAX_M10S_OK)
    {
        for (;;)
            osDelay(1000);
    }

    for (;;)
    {
        // Timeout so that a failed receive restart is retried even though no
        // event arrives while the DMA is stopped.
        (void)osSemaphoreAcquire(s_idle_sem, GNSS_TASK_WAKE_TIMEOUT_MS);

        max_m10s_frame_t frame;
        while (max_m10s_drain(&s_drv, &frame))
        {
            if (frame.msg_class == UBX_CLASS_NAV &&
                frame.msg_id   == UBX_ID_NAV_PVT)
            {
                ubx_nav_pvt_t pvt;
                if (!ubx_decode_nav_pvt(frame.payload, frame.len, &pvt))
                    continue;

                /* Apply datum if pending and fix is valid. */
                if (s_capture_pending &&
                    pvt.fix_ok && pvt.fix_type >= UBX_FIX_3D)
                {
                    ubx_datum_set(&s_datum,
                                  pvt.lat_1e7, pvt.lon_1e7, pvt.height_mm);
                    s_capture_pending = false;
                }

                /* Build fix snapshot. */
                gnss_fix_t snap;
                memset(&snap, 0, sizeof(snap));

                snap.timestamp_ms = HAL_GetTick();
                snap.fix_valid    = pvt.fix_ok && (pvt.fix_type >= UBX_FIX_3D);

                snap.itow_ms        = pvt.itow_ms;
                snap.year           = pvt.year;
                snap.month          = pvt.month;
                snap.day            = pvt.day;
                snap.hour           = pvt.hour;
                snap.min            = pvt.min;
                snap.sec            = pvt.sec;
                snap.valid_date     = pvt.valid_date;
                snap.valid_time     = pvt.valid_time;
                snap.fully_resolved = pvt.fully_resolved;
                snap.t_acc_ns       = pvt.t_acc_ns;
                snap.nano           = pvt.nano;

                snap.fix_type = pvt.fix_type;
                snap.fix_ok   = pvt.fix_ok;
                snap.num_sv   = pvt.num_sv;

                snap.lon_1e7   = pvt.lon_1e7;
                snap.lat_1e7   = pvt.lat_1e7;
                snap.height_mm = pvt.height_mm;
                snap.h_msl_mm  = pvt.h_msl_mm;
                snap.h_acc_m   = pvt.h_acc_m;
                snap.v_acc_m   = pvt.v_acc_m;

                snap.vel_ned_mps[0]   = pvt.vel_ned_mps[0];
                snap.vel_ned_mps[1]   = pvt.vel_ned_mps[1];
                snap.vel_ned_mps[2]   = pvt.vel_ned_mps[2];
                snap.ground_speed_mps = pvt.ground_speed_mps;
                snap.s_acc_mps        = pvt.s_acc_mps;
                snap.head_motion_deg  = pvt.head_motion_deg;
                snap.head_acc_deg     = pvt.head_acc_deg;
                snap.p_dop            = pvt.p_dop;

                /* NED from pad datum. */
                if (s_datum.valid && snap.fix_valid)
                {
                    ubx_geodetic_to_ned(&s_datum,
                                        pvt.lat_1e7, pvt.lon_1e7,
                                        pvt.height_mm, snap.ned_m);
                    snap.ned_valid = true;
                }

                // Health counters from the driver.
                max_m10s_stats_t stats;
                max_m10s_get_stats(&s_drv, &stats);
                snap.frames_ok       = stats.frames_ok;
                snap.checksum_errors = stats.checksum_errors;
                snap.resyncs         = stats.resyncs;
                snap.oversized       = stats.oversized;
                snap.uart_overruns   = stats.uart_overruns;

                // Publish under mutex. 
                osMutexAcquire(s_fix_mutex, osWaitForever);
                s_fix = snap;
                osMutexRelease(s_fix_mutex);
            }
        }
    }
}
