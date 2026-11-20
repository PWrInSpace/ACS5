// Golden tests for the MS5611 math: the datasheet worked example plus one
// altitude self-consistency check.
#include "test_assert.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "baro/ms5611_math.h"

static void test_datasheet_golden(void) {
    // MS5611 datasheet worked example.
    const uint16_t c[6] = {40127, 36924, 23317, 23282, 33464, 28312};
    float p = 0, t = 0;
    ms5611_compensate(9085466, 8569150, c, &p, &t);
    CHECK_NEAR(t, 20.07f, 0.02f);      // 20.07 degC
    CHECK_NEAR(p, 100009.0f, 1.0f);    // 100009 Pa
}

// 95460 Pa is roughly the pad pressure at a 500 m ASL site; the point is only
// that it is not the 101325 Pa standard datum.
#define PAD_QNH_PA 95460.0f

static void test_altitude_self_consistency(void) {
    // Pressure equal to the QNH reference must yield 0 m, at any reference.
    CHECK_NEAR(ms5611_pressure_to_altitude(101325.0f, 101325.0f), 0.0f, 1e-3f);
    CHECK_NEAR(ms5611_pressure_to_altitude(PAD_QNH_PA, PAD_QNH_PA), 0.0f, 1e-3f);
}

static void test_altitude_rises_as_pressure_falls(void) {
    // Against a pad datum, climbing must produce a strictly increasing altitude.
    float prev = ms5611_pressure_to_altitude(PAD_QNH_PA, PAD_QNH_PA);
    for (float p = PAD_QNH_PA - 500.0f; p > PAD_QNH_PA - 12000.0f; p -= 500.0f) {
        float h = ms5611_pressure_to_altitude(p, PAD_QNH_PA);
        CHECK(h > prev);
        prev = h;
    }
    CHECK(prev > 900.0f);   // ~12 kPa below the pad is well over 900 m
}

static void test_pad_datum_differs_from_datum_delta(void) {
    // Setting QNH to the pad pressure is NOT the same as subtracting two
    // standard-datum altitudes: h is not linear in the reference. The two
    // differ by exactly (pad/101325)^0.190284.
    const float factor = powf(PAD_QNH_PA / 101325.0f, 0.190284f);
    for (float p = PAD_QNH_PA - 1000.0f; p > PAD_QNH_PA - 12000.0f; p -= 1000.0f) {
        float agl_new = ms5611_pressure_to_altitude(p, PAD_QNH_PA);
        float agl_old = ms5611_pressure_to_altitude(p, 101325.0f)
                      - ms5611_pressure_to_altitude(PAD_QNH_PA, 101325.0f);
        CHECK(agl_new > agl_old);
        CHECK_NEAR(agl_old, agl_new * factor, 1e-2f);
    }
}

static void test_interval_first_sample(void) {
    // No previous sample yet -> interval unknown.
    CHECK(ms5611_interval_s(1000000u, 0u) == 0.0f);
}

static void test_interval_normal(void) {
    // 60 ms interval.
    CHECK_NEAR(ms5611_interval_s(1060000u, 1000000u), 0.06f, 1e-6f);
}

static void test_interval_too_short(void) {
    // 4 ms, below the 5 ms floor.
    CHECK(ms5611_interval_s(1004000u, 1000000u) == 0.0f);
}

static void test_interval_too_long(void) {
    // 600 ms, above the 500 ms ceiling.
    CHECK(ms5611_interval_s(1600000u, 1000000u) == 0.0f);
}

static void test_interval_dwt_wrap(void) {
    // now has wrapped past prev; the unsigned subtraction yields thousands of
    // seconds, which the upper bound rejects.
    CHECK(ms5611_interval_s(100000u, 14200000u) == 0.0f);
}

int main(void) {
    test_datasheet_golden();
    test_altitude_self_consistency();
    test_altitude_rises_as_pressure_falls();
    test_pad_datum_differs_from_datum_delta();
    test_interval_first_sample();
    test_interval_normal();
    test_interval_too_short();
    test_interval_too_long();
    test_interval_dwt_wrap();
    return test_report("ms5611_math");
}
