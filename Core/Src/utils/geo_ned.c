#include "utils/geo_ned.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

/* WGS-84 ellipsoid: semi-major axis in metres, and first eccentricity squared
 * (e2 = 2f - f^2, with flattening f = 1/298.257223563). */
#define WGS84_A  6378137.0
#define WGS84_E2 0.00669437999014
#define DEG2RAD  0.017453292519943295

void geo_ned_datum_set(geo_ned_datum_t *d, int32_t lat_1e7, int32_t lon_1e7,
                       int32_t height_mm)
{
    assert(d != NULL);

    d->lat_1e7   = lat_1e7;
    d->lon_1e7   = lon_1e7;
    d->height_mm = height_mm;

    /* Compute the scale factors once; they are reused for every fix. */
    const double lat_rad = (double)lat_1e7 * 1e-7 * DEG2RAD;
    const double s       = sin(lat_rad);
    const double w       = 1.0 - WGS84_E2 * s * s;
    const double sqrt_w  = sqrt(w);

    /* Meridian radius of curvature (north) and prime-vertical radius (east). */
    const double m_radius = WGS84_A * (1.0 - WGS84_E2) / (w * sqrt_w);
    const double n_radius = WGS84_A / sqrt_w;

    d->m_per_deg_lat = (float)(m_radius * DEG2RAD);
    d->m_per_deg_lon = (float)(n_radius * cos(lat_rad) * DEG2RAD);
    d->valid         = true;
}

void geo_ned_from_geodetic(const geo_ned_datum_t *d,
                           int32_t lat_1e7, int32_t lon_1e7, int32_t height_mm,
                           float out_ned_m[3])
{
    assert(d != NULL && out_ned_m != NULL);
    assert(d->valid);   /* the caller decides what to do without a datum */

    // The differences are taken as integers so no precision is lost before the scaling. 
    const int64_t dlat = (int64_t)lat_1e7 - (int64_t)d->lat_1e7;
    const int64_t dlon = (int64_t)lon_1e7 - (int64_t)d->lon_1e7;
    const int64_t dh   = (int64_t)height_mm - (int64_t)d->height_mm;

    out_ned_m[0] = (float)((double)dlat * 1e-7) * d->m_per_deg_lat;
    out_ned_m[1] = (float)((double)dlon * 1e-7) * d->m_per_deg_lon;
    /* Down is positive, so a height gain is negative D. */
    out_ned_m[2] = -(float)((double)dh * 1e-3);
}
