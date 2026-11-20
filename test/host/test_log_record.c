// Log record sealing and CRC; with "write <file>" also emits a sample log for the decoder test.
#include <stdio.h>
#include <string.h>

#include "test_assert.h"

#include "log/log_record.h"

static log_record_t sample(uint32_t i)
{
    log_record_t r;
    memset(&r, 0, sizeof r);
    r.tick_ms        = 1000u + 8u * i;
    r.flight_state   = 2;
    r.flags          = LOG_FLAG_GNSS_FIX | LOG_FLAG_RATE_FRESH;
    r.pos_m[2]       = 100.0f + (float)i;
    r.eul_rad[2]     = 0.5f;
    r.gnss_lat_1e7   = 521234567;
    r.gyro_bias_n    = 12345u;
    r.log_dropped    = 0u;
    return r;
}

static void test_crc32_check_value(void)
{
    CHECK(log_crc32("123456789", 9) == 0xCBF43926u);
}

static void test_sealed_record_is_valid(void)
{
    log_record_t r = sample(0);
    log_record_seal(&r, 7u);
    CHECK(r.magic == LOG_RECORD_MAGIC && r.seq == 7u && r.size == sizeof r);
    CHECK(log_record_valid(&r));
}

static void test_any_flipped_bit_is_caught(void)
{
    log_record_t r = sample(1);
    log_record_seal(&r, 1u);
    unsigned char *b = (unsigned char *)&r;
    for (size_t i = 0; i < sizeof r; i++) {
        b[i] ^= 0x10u;
        CHECK(!log_record_valid(&r));
        b[i] ^= 0x10u;
    }
    CHECK(log_record_valid(&r));
}

/* Three records with seq 0, 1, 3 (one dropped), then 1 KiB of zeros as a preallocated tail. */
static int write_sample(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) return 1;
    const uint32_t seqs[3] = {0u, 1u, 3u};
    for (uint32_t i = 0; i < 3; i++) {
        log_record_t r = sample(i);
        r.log_dropped = (i == 2) ? 1u : 0u;
        log_record_seal(&r, seqs[i]);
        fwrite(&r, sizeof r, 1, f);
    }
    static const unsigned char zeros[1024];
    fwrite(zeros, 1, sizeof zeros, f);
    return fclose(f) == 0 ? 0 : 1;
}

int main(int argc, char **argv)
{
    if (argc == 3 && strcmp(argv[1], "write") == 0)
        return write_sample(argv[2]);
    test_crc32_check_value();
    test_sealed_record_is_valid();
    test_any_flipped_bit_is_caught();
    return test_report("log_record");
}
