/* Local NED metres (down positive) from WGS-84 positions in deg*1e7 and mm, relative to a datum.
 * Flat-earth scale factors fixed at the datum, accurate over the few km a flight covers. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t lat_1e7;
    int32_t lon_1e7;
    int32_t height_mm;
    float   m_per_deg_lat;   /* meridian radius at lat_1e7*/
    float   m_per_deg_lon;   /* prime-vertical radius times cos(lat)*/
    bool    valid;
} geo_ned_datum_t;   /* all-zero (e.g. a static) is an unset datum: valid == false */

/* Sets the datum to the given position and computes its scale factors. */
void geo_ned_datum_set(geo_ned_datum_t *d, int32_t lat_1e7, int32_t lon_1e7,
                       int32_t height_mm);

/* Position relative to the datum, in metres: north, east, down. The datum must be
 * set (d->valid); checking that is left to the caller. */
void geo_ned_from_geodetic(const geo_ned_datum_t *d,
                           int32_t lat_1e7, int32_t lon_1e7, int32_t height_mm,
                           float out_ned_m[3]);
