/*
 * ACS5 - GNSS task. Owns the FreeRTOS boundary for the MAX-M10S driver.
 *
 */


#include <assert.h>

#include "stm32h7xx_hal.h"

#include "cmsis_os2.h"

#include "tasks/gnss_task.h"
#include "sensor_drivers/gnss/max_m10s.h"
#include "utils/geo_ned.h"

/*------------------------------------------------------------------- globals */

static gnss_fix_t      s_fix;            // zero-initialised at startup
static osMutexId_t     s_fix_mutex;      // guards s_fix; NULL until gnss_init() has run
static osSemaphoreId_t s_gnss_idle_sem;  // binary semaphore, given from UART idle ISR

static max_m10s_driver_t s_drv;

/* The UART the module is on, given to gnss_init(). Written once, after the
 * semaphore exists, and then only compared by the callbacks below. */
static UART_HandleTypeDef *volatile s_huart;

#define GNSS_TASK_WAKE_TIMEOUT_MS    1000u
#define GNSS_CONFIG_RETRY_DELAY_MS   2000u   /* pause between configuration attempts */

/* NED datum and its capture request. Owned by this task; the request flag is
 * set from another task by gnss_capture_datum(). Zero-initialised, so the datum
 * starts out invalid. */
static geo_ned_datum_t   s_datum;
static volatile bool s_capture_pending;

/*-------------------------------------------------------------- UART events
 *
 * The HAL's UART callbacks are global (one definition per program), so they live
 * in one central place, usart.c, which forwards each event to whichever module
 * owns that UART. These two are the GNSS module's part; both run in interrupt
 * context and ignore any UART that is not the GNSS one. */

// A receive-to-idle event: new bytes are waiting in the DMA buffer.
void gnss_task_on_uart_rx_event(UART_HandleTypeDef *huart)
{
    if (huart == s_huart)
        osSemaphoreRelease(s_gnss_idle_sem);
}

// A UART receive error (overrun, noise, framing...). The HAL has already aborted
// the DMA, so no idle event will ever come again unless the driver restarts
// reception: flag it and wake the task to do so.
void gnss_task_on_uart_error(UART_HandleTypeDef *huart)
{
    if (huart == s_huart)
    {
        max_m10s_on_uart_error(&s_drv);
        osSemaphoreRelease(s_gnss_idle_sem);
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

/* -------------------------------------------------------------------- init
 *
 * Creates the RTOS objects the other tasks and the UART callbacks depend on.
 * Called before the kernel starts, so none of them can run before these exist. */
bool gnss_init(UART_HandleTypeDef *huart)
{
    assert(huart != NULL);

    osMutexId_t mutex = osMutexNew(NULL);
    if (mutex == NULL)
        return false;

    osSemaphoreId_t sem = osSemaphoreNew(1, 0, NULL);
    if (sem == NULL)
    {
        (void)osMutexDelete(mutex);
        return false;
    }

    s_fix_mutex     = mutex;
    s_gnss_idle_sem = sem;
    s_huart         = huart;   // last: the callbacks use the semaphore once this is set
    return true;
}

/* ----------------------------------------------------------------- get fix
 *
 * Non-blocking so a low-rate consumer (the logger) can never be stalled by the
 * GNSS task holding the mutex. Safe to call before gnss_init(): the mutex does
 * not exist yet, so it just returns false. */
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

// Publishes the link state without touching the rest of the snapshot, so the
// rest of the system can tell "module not working" from "no fix yet".
static void publish_link(gnss_link_t link, uint8_t error, uint8_t failures)
{
    osMutexAcquire(s_fix_mutex, osWaitForever);
    s_fix.timestamp_ms    = HAL_GetTick();
    s_fix.link            = link;
    s_fix.link_error      = error;
    s_fix.config_failures = failures;
    osMutexRelease(s_fix_mutex);
}

// Brings the module up, retrying for as long as it takes (it may be powered
// late, or the link may glitch) and saying so to everyone who reads the fix.
// Only configure() is repeated once the driver is initialised: it restarts the
// receive itself, so it is safe to call again. Returns the number of failed
// attempts (saturating).
static uint8_t bring_up_module(void)
{
    uint8_t failures = 0;
    bool driver_ready = false;

    for (;;)
    {
        max_m10s_status_t st = MAX_M10S_OK;
        if (!driver_ready)
        {
            st = max_m10s_init(&s_drv, s_huart);
            driver_ready = (st == MAX_M10S_OK);
        }
        if (driver_ready)
            st = max_m10s_configure(&s_drv);
        if (st == MAX_M10S_OK)
            break;

        if (failures < UINT8_MAX)
            failures++;
        publish_link(GNSS_LINK_FAILED, (uint8_t)st, failures);
        osDelay(GNSS_CONFIG_RETRY_DELAY_MS);
    }

    publish_link(GNSS_LINK_OK, 0u, failures);
    return failures;
}

static bool is_nav_pvt(const max_m10s_frame_t *frame)
{
    return frame->msg_class == UBX_CLASS_NAV && frame->msg_id == UBX_ID_NAV_PVT;
}

// Latches the NED datum from this fix if one was requested and it has a position.
static void latch_datum_if_requested(const ubx_nav_pvt_t *pvt)
{
    if (s_capture_pending && ubx_nav_pvt_has_position(pvt))
    {
        geo_ned_datum_set(&s_datum, pvt->lat_1e7, pvt->lon_1e7, pvt->height_mm);
        s_capture_pending = false;
    }
}

// Fills *snap from a decoded NAV-PVT (kept whole), the NED datum and the driver's
// counters.
static void build_snapshot(const ubx_nav_pvt_t *pvt, uint8_t failures, gnss_fix_t *snap)
{
    *snap = (gnss_fix_t){0};

    snap->timestamp_ms    = HAL_GetTick();
    snap->link            = GNSS_LINK_OK;
    snap->config_failures = failures;
    snap->fix_valid       = ubx_nav_pvt_has_position(pvt);

    snap->pvt = *pvt;

    /* NED from pad datum. */
    if (s_datum.valid && snap->fix_valid)
    {
        geo_ned_from_geodetic(&s_datum, pvt->lat_1e7, pvt->lon_1e7,
                              pvt->height_mm, snap->ned_m);
        snap->ned_valid = true;
    }

    max_m10s_get_stats(&s_drv, &snap->stats);
}

// Makes the snapshot the one gnss_get_fix() returns.
static void publish(const gnss_fix_t *snap)
{
    osMutexAcquire(s_fix_mutex, osWaitForever);
    s_fix = *snap;
    osMutexRelease(s_fix_mutex);
}

void gnss_task(void *arg)
{
    (void)arg;
    assert(s_huart != NULL && "gnss_init() must run before the task starts");

    const uint8_t failures = bring_up_module();

    for (;;)
    {
        // Timeout so that a failed receive restart is retried even though no
        // event arrives while the DMA is stopped.
        (void)osSemaphoreAcquire(s_gnss_idle_sem, GNSS_TASK_WAKE_TIMEOUT_MS);

        max_m10s_frame_t frame;
        while (max_m10s_drain(&s_drv, &frame))
        {
            if (!is_nav_pvt(&frame))
                continue;

            ubx_nav_pvt_t pvt;
            if (!ubx_decode_nav_pvt(frame.payload, frame.len, &pvt))
                continue;

            latch_datum_if_requested(&pvt);

            gnss_fix_t snap;
            build_snapshot(&pvt, failures, &snap);
            publish(&snap);
        }
    }
}
