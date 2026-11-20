/* GNSS task: the FreeRTOS boundary for the MAX-M10S driver. */


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
/* No frame for this long means the module reset and lost its RAM configuration. */
#define GNSS_SILENCE_REINIT_MS       3000u

/* NED datum (owned by this task) and the capture request set by gnss_capture_datum(). */
static geo_ned_datum_t   s_datum;
static volatile bool s_capture_pending;

/*-------------------------------------------------------------- UART events */

// A receive-to-idle event: new bytes are waiting in the DMA buffer.
void gnss_task_on_uart_rx_event(UART_HandleTypeDef *huart)
{
    if (huart == s_huart)
        osSemaphoreRelease(s_gnss_idle_sem);
}

// The HAL has already aborted the DMA, so no idle event comes until the driver
// restarts reception: flag it and wake the task.
void gnss_task_on_uart_error(UART_HandleTypeDef *huart)
{
    if (huart == s_huart)
    {
        max_m10s_on_uart_error(&s_drv);
        osSemaphoreRelease(s_gnss_idle_sem);
    }
}

/* -------------------------------------------------------------- capture datum */
void gnss_capture_datum(void)
{
    s_capture_pending = true;
}

/* -------------------------------------------------------------------- init */
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

/* ----------------------------------------------------------------- get fix */
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

// Publishes the link state without touching the rest of the snapshot.
static void publish_link(gnss_link_t link, uint8_t error, uint8_t failures)
{
    osMutexAcquire(s_fix_mutex, osWaitForever);
    s_fix.timestamp_ms    = HAL_GetTick();
    s_fix.link            = link;
    s_fix.link_error      = error;
    s_fix.config_failures = failures;
    osMutexRelease(s_fix_mutex);
}

// Brings the module up, retrying forever and publishing each failure; returns
// the failed attempts (saturating).
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

// Fills *snap from a decoded NAV-PVT, the NED datum and the driver's counters.
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

    uint8_t  failures   = bring_up_module();
    uint32_t last_frame = HAL_GetTick();

    for (;;)
    {
        // Timeout so that a failed receive restart is retried even though no
        // event arrives while the DMA is stopped.
        (void)osSemaphoreAcquire(s_gnss_idle_sem, GNSS_TASK_WAKE_TIMEOUT_MS);

        if ((uint32_t)(HAL_GetTick() - last_frame) > GNSS_SILENCE_REINIT_MS)
        {
            publish_link(GNSS_LINK_FAILED, (uint8_t)MAX_M10S_ERR_TIMEOUT, failures);
            failures   = bring_up_module();
            last_frame = HAL_GetTick();
        }

        max_m10s_frame_t frame;
        while (max_m10s_drain(&s_drv, &frame))
        {
            last_frame = HAL_GetTick();
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
