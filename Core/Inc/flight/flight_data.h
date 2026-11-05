/*
 * ACS5 - Shared flight data types.
 * This header defines the data types shared between the GNSS driver and the 
 * flight state machine.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensor_drivers/gnss/max_m10s.h"
#include "sensor_drivers/gnss/ubx_parse.h"

/* ------------------------------------------------------------------ GNSS */

/*
 * State of the link to the GNSS module, separate from whether it has a fix: a
 * module that is not working looks the same as "no fix yet" without this.
 */
typedef enum
{
    GNSS_LINK_STARTING = 0,   /* the task has not configured the module yet */
    GNSS_LINK_OK,             /* configured and receiving; see fix_valid for the fix */
    GNSS_LINK_FAILED,         /* configuration is failing; the task keeps retrying */
} gnss_link_t;

/*
 * The latest GNSS snapshot: the receiver's last NAV-PVT solution (pvt, exactly as
 * decoded), plus what the task derives from it (fix_valid, NED position relative
 * to the pad datum), the link status and driver health counters.
 *
 * Coordinate conventions to check before integrating with anything else:
 *
 *   pvt.vel_ned_mps[2]  Down velocity, POSITIVE DOWNWARD. A climbing rocket
 *                       has a negative value here. This is the opposite sign
 *                       to a barometric altitude rate.
 *
 *   ned_m[2]            Down position relative to the pad datum, POSITIVE
 *                       DOWNWARD. A rocket above the pad has a negative value.
 *
 *   pvt.lon_1e7 / pvt.lat_1e7  Integer degrees x 1e7. Not converted to float to
 *                              preserve precision; use ned_m for metre-level
 *                              position.
 */
typedef struct
{
    /* system time */
    uint32_t timestamp_ms;

    /* ------------------------------------------------------ module link */

    gnss_link_t link;

    /* Why the last attempt failed while link == GNSS_LINK_FAILED: the driver's
     * status code (1 = HAL/DMA error, 2 = module rejected the settings (NAK),
     * 3 = module did not answer). 0 otherwise. */
    uint8_t  link_error;

    /* Failed configuration attempts so far (saturates at 255). */
    uint8_t  config_failures;

    /* ------------------------------------------------- fix validity gate */

    /* True when the solution has a usable position: pvt.fix_ok and a 3D or
     * GNSS + dead reckoning fix. A time-only fix is not valid. */
    bool fix_valid;

    /* ------------------------------------------------- receiver solution */

    /* The last decoded UBX-NAV-PVT, unmodified: time, fix type (check pvt.fix_ok
     * alongside it; see fix_valid), position, accuracy, NED velocity (Down is
     * POSITIVE DOWNWARD), DOP. All zeros until the first one has arrived. */
    ubx_nav_pvt_t pvt;

    /* ------------------------------------------ local NED from pad datum */

    /* Valid only after gnss_capture_datum() has been called AND a fix with a
     * position has arrived. Check ned_valid before using these. */
    float    ned_m[3];

    /* True when the datum has been captured AND this snapshot has a valid
     * fix (fix_valid), i.e. ned_m is meaningful. */
    bool     ned_valid;

    /* ----------------------------------------------- driver health counters */
    max_m10s_stats_t stats;   /* frames, checksum errors, resyncs, UART errors */

} gnss_fix_t;
