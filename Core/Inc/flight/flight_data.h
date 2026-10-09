/*
 * ACS5 - Shared flight data types.
 * This header defines the data types shared between the GNSS driver and the 
 * flight state machine.
 *
 * To use gnss_fix_t from another task:
 *
 *   #include "flight/flight_data.h"
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os2.h"

/* ------------------------------------------------------------------ GNSS */

/*
 * Everything the MAX-M10S delivers in one NAV-PVT frame, plus derived
 * quantities (NED position relative to the pad datum) and driver health
 * counters. 
 * 
 * Coordinate conventions to check before integrating with anything else:
 *
 *   vel_ned_mps[2]  Down velocity, POSITIVE DOWNWARD. A climbing rocket
 *                   has a negative value here. This is the opposite sign
 *                   to a barometric altitude rate.
 *
 *   ned_m[2]        Down position relative to the pad datum, POSITIVE
 *                   DOWNWARD. A rocket above the pad has a negative value.
 *
 *   lon_1e7 / lat_1e7  Integer degrees × 1e7. Not converted to float to
 *                      preserve precision; use ned_m for
 *                      metre-level position.
 */
typedef struct
{
    /* system time */
    uint32_t timestamp_ms;

    /* ------------------------------------------------- fix validity gate */

    /* True when fix_type >= UBX_FIX_3D AND fix_ok.  */
    bool fix_valid;

    uint32_t itow_ms;

    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    bool     valid_date;
    bool     valid_time;
    bool     fully_resolved;   /* UTC offset known; time is unambiguous */
    uint32_t t_acc_ns;         /* time accuracy estimate */
    int32_t  nano;             /* fraction of second, -1e9 .. +1e9 ns */

    /* -------------------------------------------- UBX-NAV-PVT: fix status */

    /* 0=none 1=dead-reck 2=2D 3=3D 4=GNSS+DR 5=time-only.
     * Check fix_ok alongside this; see fix_valid above. */
    uint8_t  fix_type;
    bool     fix_ok;
    uint8_t  num_sv;

    /* raw coordinates */
    int32_t  lon_1e7;
    int32_t  lat_1e7;

    /* Height above the WGS-84 ellipsoid and above mean sea level, in mm. */
    int32_t  height_mm;
    int32_t  h_msl_mm;

    float    h_acc_m;
    float    v_acc_m;

    /* ---------------------------------------- UBX-NAV-PVT: NED velocity */

    /* North / East / Down velocity in m/s.
     * Down is POSITIVE DOWNWARD. */
    float    vel_ned_mps[3];
    float    ground_speed_mps;
    float    s_acc_mps;
    float    head_motion_deg;
    float    head_acc_deg;
    float    p_dop;

    /* ------------------------------------------ local NED from pad datum */

    /* Valid only after gnss_capture_datum() has been called AND at least
     * one 3D fix has arrived. Check ned_valid before using these. */
    float    ned_m[3];

    /* True when the datum has been captured AND this snapshot has a valid
     * fix (fix_valid), i.e. ned_m is meaningful. */
    bool     ned_valid;

    /* ----------------------------------------------- driver health counters */
    uint32_t frames_ok;
    uint32_t checksum_errors;
    uint32_t resyncs;
    uint32_t oversized;
    uint32_t uart_overruns;    /* UART peripheral overrun counter, from HAL */

} gnss_fix_t;

/*
 * Call once from the flight state machine when the rocket is on the launch pad. 
 * The driver will set the datum on the next valid frame 
 * and begin populating ned_m. Before this is called, ned_m is all
 * zeros and ned_valid is false.
 *
 * Defined in gnss_task.c.
 */
void gnss_capture_datum(void);

/* -------------------------------------------------- shared driver globals */

extern gnss_fix_t    g_gnss_fix;
extern osMutexId_t   g_gnss_mutex;
