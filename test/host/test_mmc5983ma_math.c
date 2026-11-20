// MMC5983MA 18-bit unpack, conversion to uT, and saturation detection.
#include "test_assert.h"

#include "sensor_drivers/mag/mmc5983ma_math.h"

static void test_unpack_places_the_low_bits(void)
{
    /* X = 0x12345 | Y = 0x20000 (zero field) | Z = 0x3FFFF */
    const uint8_t buf[7] = {
        0x48, 0xD1,              /* X[17:2] */
        0x80, 0x00,              /* Y[17:2] */
        0xFF, 0xFF,              /* Z[17:2] */
        (0x1 << 6) | (0x0 << 4) | (0x3 << 2),
    };
    uint32_t raw[3];
    mmc5983ma_unpack(buf, raw);
    CHECK(raw[0] == 0x12345u);
    CHECK(raw[1] == 0x20000u);
    CHECK(raw[2] == 0x3FFFFu);
}

static void test_counts_to_microtesla(void)
{
    CHECK_NEAR(mmc5983ma_raw_to_ut(131072u), 0.0f, 1e-6f);
    CHECK_NEAR(mmc5983ma_raw_to_ut(131072u + 16384u), 100.0f, 1e-3f);   /* 1 G */
    CHECK_NEAR(mmc5983ma_raw_to_ut(131072u - 8192u), -50.0f, 1e-3f);
}

static void test_rails_are_saturation(void)
{
    const uint32_t ok[3]   = {131072u, 140000u, 120000u};
    const uint32_t low[3]  = {131072u, 0u, 120000u};
    const uint32_t high[3] = {0x3FFFFu, 140000u, 120000u};
    CHECK(!mmc5983ma_saturated(ok));
    CHECK(mmc5983ma_saturated(low));
    CHECK(mmc5983ma_saturated(high));
}

static void test_magnitude_plausibility(void)
{
    CHECK(mmc5983ma_magnitude_plausible(50.0f, 50.0f, 0.3f));
    CHECK(mmc5983ma_magnitude_plausible(64.0f, 50.0f, 0.3f));
    CHECK(!mmc5983ma_magnitude_plausible(66.0f, 50.0f, 0.3f));
    CHECK(!mmc5983ma_magnitude_plausible(30.0f, 50.0f, 0.3f));
}

int main(void)
{
    test_unpack_places_the_low_bits();
    test_counts_to_microtesla();
    test_rails_are_saturation();
    test_magnitude_plausibility();
    return test_report("mmc5983ma_math");
}
