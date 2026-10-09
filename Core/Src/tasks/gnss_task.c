/*
 * ACS5 - GNSS task. Owns the FreeRTOS boundary for the MAX-M10S driver.
 *
 */


#include "stm32h7xx_hal.h"

#include "cmsis_os2.h"

#include "flight/flight_data.h"
#include "gnss/max_m10s.h"

#include <string.h>

/*------------------------------------------------------------------- globals */

gnss_fix_t      g_gnss_fix;        //zero-initialised at startup 
osMutexId_t     g_gnss_mutex;
 osSemaphoreId_t g_gnss_idle_sem;  //binary semaphore, given from UART idle ISR 

static gnss_driver_t s_drv;

/*--------------------------------------------------------------- idle callback */

// The STM32 HAL calls this from the UART idle IRQ.  
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)size;
    if (huart == &GNSS_HUART_HANDLE)
        osSemaphoreRelease(g_gnss_idle_sem);
}

/* -------------------------------------------------------------- capture datum
 *
 * Called by the flight state machine when the rocket is on the 
 * pad with a confirmed 3D fix. The driver applies 
 * the datum on the next valid frame. */
void gnss_capture_datum(void)
{
    s_drv.capture_pending = true;
}

/*-------------------------------------------------------------------- task */

void gnss_task(void *arg)
{
    (void)arg;

    g_gnss_mutex    = osMutexNew(NULL); 
    g_gnss_idle_sem = osSemaphoreNew(1, 0, NULL);

    max_m10s_init(&s_drv);

    gnss_status_t cfg = max_m10s_configure(&s_drv);
    if (cfg != GNSS_OK)
    {
        for (;;)
            osDelay(1000);
    }

    for (;;)
    {
        osSemaphoreAcquire(g_gnss_idle_sem, osWaitForever);

        while (max_m10s_drain(&s_drv))
        {
            if (s_drv.parser.msg_class == UBX_CLASS_NAV &&
                s_drv.parser.msg_id   == UBX_ID_NAV_PVT)
            {
                ubx_nav_pvt_t pvt;
                if (!ubx_decode_nav_pvt(s_drv.parser.payload,
                                        s_drv.parser.len, &pvt))
                    continue;

                /* Apply datum if pending and fix is valid. */
                if (s_drv.capture_pending &&
                    pvt.fix_ok && pvt.fix_type >= UBX_FIX_3D)
                {
                    ubx_datum_set(&s_drv.datum,
                                  pvt.lat_1e7, pvt.lon_1e7, pvt.height_mm);
                    s_drv.capture_pending = false;
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
                if (s_drv.datum.valid && snap.fix_valid)
                {
                    ubx_geodetic_to_ned(&s_drv.datum,
                                        pvt.lat_1e7, pvt.lon_1e7,
                                        pvt.height_mm, snap.ned_m);
                    snap.ned_valid = true;
                }

                // Health counters from parser. 
                snap.frames_ok       = s_drv.parser.frames_ok;
                snap.checksum_errors = s_drv.parser.checksum_errors;
                snap.resyncs         = s_drv.parser.resyncs;
                snap.oversized       = s_drv.parser.oversized;
                snap.uart_overruns   = s_drv.uart_overruns;

                // Publish under mutex. 
                osMutexAcquire(g_gnss_mutex, osWaitForever);
                g_gnss_fix = snap;
                osMutexRelease(g_gnss_mutex);
            }
        }
    }
}
