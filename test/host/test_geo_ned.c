// Tests for the local North-East-Down conversion: scale factors, axis signs and
// edge cases. Reference values are computed independently from the WGS-84
// ellipsoid in double precision.
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "utils/geo_ned.h"

#define A_WGS84  6378137.0
#define E2_WGS84 0.00669437999014
#define DEG      (3.14159265358979323846 / 180.0)   /* pi is not in strict C11 math.h */

// Metres per degree of latitude / longitude at a given latitude (degrees).
static double ref_m_per_deg_lat(double lat_deg) {
    double s = sin(lat_deg * DEG), w = 1.0 - E2_WGS84 * s * s;
    return A_WGS84 * (1.0 - E2_WGS84) / (w * sqrt(w)) * DEG;
}
static double ref_m_per_deg_lon(double lat_deg) {
    double s = sin(lat_deg * DEG), w = 1.0 - E2_WGS84 * s * s;
    return A_WGS84 / sqrt(w) * cos(lat_deg * DEG) * DEG;
}

static int32_t deg_1e7(double deg) { return (int32_t)llround(deg * 1e7); }

static void test_unset_datum_is_invalid(void) {
    geo_ned_datum_t d;
    memset(&d, 0, sizeof d);          // what a static starts out as
    assert(!d.valid);
}

static void test_datum_set_stores_position_and_scales(void) {
    geo_ned_datum_t d;
    memset(&d, 0, sizeof d);
    geo_ned_datum_set(&d, deg_1e7(52.1), deg_1e7(17.2), 83000);
    assert(d.valid);
    assert(d.lat_1e7 == deg_1e7(52.1) && d.lon_1e7 == deg_1e7(17.2) && d.height_mm == 83000);
    // float scale factors, so a few centimetres per degree of tolerance
    assert(fabs(d.m_per_deg_lat - ref_m_per_deg_lat(52.1)) < 0.05);
    assert(fabs(d.m_per_deg_lon - ref_m_per_deg_lon(52.1)) < 0.05);
}

static void test_scale_changes_with_latitude(void) {
    // A degree of longitude shrinks towards the poles, a degree of latitude
    // grows slightly. Check against the reference at several latitudes,
    // both hemispheres.
    const double lats[] = {0.0, 10.0, 52.0, 70.0, 89.0, -33.0, -80.0};
    for (size_t i = 0; i < sizeof lats / sizeof lats[0]; i++) {
        geo_ned_datum_t d;
        geo_ned_datum_set(&d, deg_1e7(lats[i]), 0, 0);
        assert(fabs(d.m_per_deg_lat - ref_m_per_deg_lat(lats[i])) < 0.05);
        assert(fabs(d.m_per_deg_lon - ref_m_per_deg_lon(lats[i])) < 0.05);
        assert(d.m_per_deg_lon > 0.0f);
    }
    geo_ned_datum_t eq, hi;
    geo_ned_datum_set(&eq, 0, 0, 0);
    geo_ned_datum_set(&hi, deg_1e7(60.0), 0, 0);
    assert(hi.m_per_deg_lon < 0.55f * eq.m_per_deg_lon);   // cos(60 deg) = 0.5
    assert(hi.m_per_deg_lat > eq.m_per_deg_lat);
}

static void test_same_position_is_zero(void) {
    geo_ned_datum_t d;
    geo_ned_datum_set(&d, deg_1e7(51.5), deg_1e7(-0.12), 35000);
    float n[3] = {9, 9, 9};
    geo_ned_from_geodetic(&d, deg_1e7(51.5), deg_1e7(-0.12), 35000, n);
    assert(n[0] == 0.0f && n[1] == 0.0f && n[2] == 0.0f);
}

static void test_axis_signs(void) {
    // North is +N, east is +E, and Down is positive downward: a climb is -D.
    geo_ned_datum_t d;
    geo_ned_datum_set(&d, deg_1e7(45.0), deg_1e7(10.0), 0);
    float n[3];

    geo_ned_from_geodetic(&d, deg_1e7(45.001), deg_1e7(10.0), 0, n);
    assert(n[0] > 0.0f && n[1] == 0.0f && n[2] == 0.0f);
    geo_ned_from_geodetic(&d, deg_1e7(44.999), deg_1e7(10.0), 0, n);
    assert(n[0] < 0.0f);

    geo_ned_from_geodetic(&d, deg_1e7(45.0), deg_1e7(10.001), 0, n);
    assert(n[1] > 0.0f && n[0] == 0.0f);
    geo_ned_from_geodetic(&d, deg_1e7(45.0), deg_1e7(9.999), 0, n);
    assert(n[1] < 0.0f);

    geo_ned_from_geodetic(&d, deg_1e7(45.0), deg_1e7(10.0), 1000, n);       // +1 m up
    assert(fabsf(n[2] + 1.0f) < 1e-6f);
    geo_ned_from_geodetic(&d, deg_1e7(45.0), deg_1e7(10.0), -2500, n);      // 2.5 m down
    assert(fabsf(n[2] - 2.5f) < 1e-6f);
}

static void test_known_distances_at_the_equator(void) {
    // 1e-5 degree is 100 units of 1e-7 degree: about 1.1057 m north (a degree of
    // meridian at the equator is 110 574 m) and 1.1132 m east.
    geo_ned_datum_t d;
    geo_ned_datum_set(&d, 0, 0, 0);
    float n[3];
    geo_ned_from_geodetic(&d, 100, 100, 0, n);
    assert(fabsf(n[0] - 1.10574f) < 1e-4f);
    assert(fabsf(n[1] - 1.11319f) < 1e-4f);
}

static void test_matches_reference_over_flight_scale_distances(void) {
    // A few km either way, several datums: the result is the integer coordinate
    // difference times the scale, so it must agree with the double reference to
    // well under a centimetre per kilometre.
    const double lats[] = {-45.0, 0.0, 38.0, 52.1};
    const double lons[] = {-120.0, -0.1, 17.2, 150.0};
    for (size_t i = 0; i < sizeof lats / sizeof lats[0]; i++) {
        geo_ned_datum_t d;
        geo_ned_datum_set(&d, deg_1e7(lats[i]), deg_1e7(lons[i]), 100000);
        const double dlat = 0.03, dlon = -0.045;          // about 3 km north, 3 km west
        float n[3];
        geo_ned_from_geodetic(&d, deg_1e7(lats[i] + dlat), deg_1e7(lons[i] + dlon),
                              100000 + 3000000, n);
        assert(fabs(n[0] - dlat * ref_m_per_deg_lat(lats[i])) < 0.05);
        assert(fabs(n[1] - dlon * ref_m_per_deg_lon(lats[i])) < 0.05);
        assert(fabsf(n[2] + 3000.0f) < 1e-3f);
    }
}

static void test_difference_is_taken_in_integers(void) {
    // Two positions 1 mm-scale apart near the pole of precision: a float
    // latitude (about 7 digits) could not tell these apart, the integer
    // difference can.
    geo_ned_datum_t d;
    geo_ned_datum_set(&d, 519999999, 170000000, 0);
    float n[3];
    geo_ned_from_geodetic(&d, 520000000, 170000000, 0, n);   // 1e-7 degree, about 1.1 cm
    assert(n[0] > 0.010f && n[0] < 0.013f);
}

static void test_extreme_coordinates_do_not_overflow(void) {
    // From the far side of the globe the difference is beyond 32 bits in
    // longitude (3.6e9 units of 1e-7 degree, more than INT32_MAX); the 64-bit
    // subtraction must not wrap. Nobody flies there; this guards the arithmetic.
    float n[3];
    geo_ned_datum_t d;

    geo_ned_datum_set(&d, 900000000, 1800000000, 0);
    geo_ned_from_geodetic(&d, -900000000, 1800000000, 0, n);
    assert(isfinite(n[0]) && n[0] < -1.9e7f);          // about 180 degrees of latitude south

    geo_ned_datum_set(&d, 0, 1800000000, 0);
    geo_ned_from_geodetic(&d, 0, -1800000000, 0, n);
    assert(fabs(n[1] - (-360.0 * ref_m_per_deg_lon(0.0))) < 5.0);   // -360 degrees of longitude

    geo_ned_datum_set(&d, 0, -1800000000, 0);
    geo_ned_from_geodetic(&d, 0, 1800000000, 0, n);
    assert(fabs(n[1] - (360.0 * ref_m_per_deg_lon(0.0))) < 5.0);
}

int main(void) {
    test_unset_datum_is_invalid();
    test_datum_set_stores_position_and_scales();
    test_scale_changes_with_latitude();
    test_same_position_is_zero();
    test_axis_signs();
    test_known_distances_at_the_equator();
    test_matches_reference_over_flight_scale_distances();
    test_difference_is_taken_in_integers();
    test_extreme_coordinates_do_not_overflow();
    puts("test_geo_ned: all passed");
    return 0;
}
