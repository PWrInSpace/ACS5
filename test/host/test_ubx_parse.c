// Tests for the UBX codec: frame building, the byte-at-a-time parser and its
// counters, the NAV-PVT decoder, the position-fix rule and the ACK matcher.
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor_drivers/gnss/ubx_parse.h"

// Feeds n bytes; returns how many of them completed a frame.
static int feed(ubx_parser_t *p, const uint8_t *b, size_t n) {
    int frames = 0;
    for (size_t i = 0; i < n; i++)
        if (ubx_parse_byte(p, b[i])) frames++;
    return frames;
}

/* ------------------------------------------------------------- building */

static void test_frame_build_known_vectors(void) {
    // Worked examples from the u-blox protocol description.
    const uint8_t ack_payload[2] = {0x06, 0x8A};   // ACK-ACK for CFG-VALSET
    const uint8_t ack_expected[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00,
                                    0x06, 0x8A, 0x98, 0xC1};
    uint8_t out[16];
    uint16_t n = ubx_frame_build(out, sizeof out, UBX_CLASS_ACK, UBX_ID_ACK_ACK,
                                 ack_payload, 2);
    assert(n == sizeof ack_expected);
    assert(memcmp(out, ack_expected, n) == 0);

    // Poll request: no payload.
    const uint8_t poll_expected[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
    n = ubx_frame_build(out, sizeof out, UBX_CLASS_NAV, UBX_ID_NAV_PVT, NULL, 0);
    assert(n == sizeof poll_expected);
    assert(memcmp(out, poll_expected, n) == 0);
}

static void test_frame_build_length_and_capacity(void) {
    uint8_t payload[40], out[64];
    memset(payload, 0x11, sizeof payload);

    // Total length is payload + UBX_FRAME_OVERHEAD, and the length field says so.
    uint16_t n = ubx_frame_build(out, sizeof out, 0x06, 0x8A, payload, 40);
    assert(n == 40 + UBX_FRAME_OVERHEAD);
    assert(out[UBX_OFS_LEN] == 40 && out[UBX_OFS_LEN + 1] == 0);

    // A buffer of exactly that size works, one byte less is refused untouched.
    uint8_t tight[48 + 1];
    memset(tight, 0xEE, sizeof tight);
    assert(ubx_frame_build(tight, 48, 0x06, 0x8A, payload, 40) == 48);
    assert(tight[48] == 0xEE);                       // nothing written past the end
    memset(tight, 0xEE, sizeof tight);
    assert(ubx_frame_build(tight, 47, 0x06, 0x8A, payload, 40) == 0);
    assert(tight[0] == 0xEE);                        // refused before writing

    // A payload longer than 255 bytes needs the high length byte.
    static uint8_t big[300], bigout[320];
    n = ubx_frame_build(bigout, sizeof bigout, 0x01, 0x02, big, 300);
    assert(n == 300 + UBX_FRAME_OVERHEAD);
    assert(bigout[UBX_OFS_LEN] == (300 & 0xFF) && bigout[UBX_OFS_LEN + 1] == (300 >> 8));
}

/* -------------------------------------------------------------- parsing */

static void test_parse_reports_frame_on_last_byte(void) {
    uint8_t pl[92] = {0}, f[100];
    uint16_t n = ubx_frame_build(f, sizeof f, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92);
    ubx_parser_t p;
    ubx_parser_init(&p);
    for (uint16_t i = 0; i < n - 1; i++) assert(!ubx_parse_byte(&p, f[i]));
    assert(ubx_parse_byte(&p, f[n - 1]));
    assert(p.msg_class == UBX_CLASS_NAV && p.msg_id == UBX_ID_NAV_PVT && p.len == 92);
    assert(p.stats.frames_ok == 1);
}

static void test_parse_payload_is_kept(void) {
    uint8_t pl[10], f[32];
    for (int i = 0; i < 10; i++) pl[i] = (uint8_t)(0xA0 + i);
    uint16_t n = ubx_frame_build(f, sizeof f, 0x06, 0x8A, pl, 10);
    ubx_parser_t p;
    ubx_parser_init(&p);
    assert(feed(&p, f, n) == 1);
    assert(p.len == 10 && memcmp(p.payload, pl, 10) == 0);
}

static void test_parse_back_to_back_and_zero_length(void) {
    uint8_t pl[4] = {1, 2, 3, 4}, a[32], b[32], stream[64];
    uint16_t na = ubx_frame_build(a, sizeof a, 0x01, 0x01, NULL, 0);   // empty payload
    uint16_t nb = ubx_frame_build(b, sizeof b, 0x01, 0x02, pl, 4);
    memcpy(stream, a, na);
    memcpy(stream + na, b, nb);
    ubx_parser_t p;
    ubx_parser_init(&p);
    assert(feed(&p, stream, na + nb) == 2);
    assert(p.stats.frames_ok == 2 && p.msg_id == 0x02 && p.len == 4);
}

static void test_parse_skips_garbage_and_sync_runs(void) {
    uint8_t pl[2] = {0x06, 0x8A}, f[16], stream[64];
    uint16_t n = ubx_frame_build(f, sizeof f, UBX_CLASS_ACK, UBX_ID_ACK_ACK, pl, 2);
    // Noise, a lone sync byte, then a run of 0xB5 immediately before the frame.
    const uint8_t junk[] = {0x00, 0xFF, 0x62, 0xB5, 0x00, 0xB5, 0xB5, 0xB5};
    memcpy(stream, junk, sizeof junk);
    memcpy(stream + sizeof junk, f, n);
    ubx_parser_t p;
    ubx_parser_init(&p);
    assert(feed(&p, stream, sizeof junk + n) == 1);
    assert(p.msg_class == UBX_CLASS_ACK && p.stats.checksum_errors == 0);
}

static void test_parse_same_result_for_any_chunking(void) {
    uint8_t pl[92], f[100];
    for (int i = 0; i < 92; i++) pl[i] = (uint8_t)(i * 7);
    uint16_t n = ubx_frame_build(f, sizeof f, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92);
    // The parser has no notion of chunks, but the driver feeds it in bursts of
    // varying size, so check that explicitly.
    for (uint16_t chunk = 1; chunk <= n; chunk++) {
        ubx_parser_t p;
        ubx_parser_init(&p);
        int frames = 0;
        for (uint16_t i = 0; i < n; i += chunk)
            frames += feed(&p, f + i, (uint16_t)(n - i < chunk ? n - i : chunk));
        assert(frames == 1 && memcmp(p.payload, pl, 92) == 0);
    }
}

static void test_parse_counts_each_kind_of_error(void) {
    ubx_parser_t p;
    ubx_parser_init(&p);
    uint8_t pl[92] = {0}, f[300];
    uint16_t n;

    n = ubx_frame_build(f, sizeof f, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92);
    for (int i = 0; i < 5; i++) feed(&p, f, n);                       // 5 good

    for (int i = 0; i < 2; i++) {                                     // 2 bad ck_a
        n = ubx_frame_build(f, sizeof f, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92);
        f[n - 2] ^= 0x55;
        assert(feed(&p, f, n) == 0);
    }
    n = ubx_frame_build(f, sizeof f, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92);
    f[n - 1] ^= 0x55;                                                 // 1 bad ck_b
    assert(feed(&p, f, n) == 0);

    static uint8_t big[300];                                          // 1 oversized
    n = ubx_frame_build(big, sizeof big, UBX_CLASS_NAV, 0x99, big + 6, 200);
    assert(feed(&p, big, n) == 0);

    assert(p.stats.frames_ok == 5);
    assert(p.stats.checksum_errors == 3);
    assert(p.stats.oversized == 1);
    assert(p.stats.resyncs == 4);       // every one of the four bad frames
}

static void test_parse_recovers_after_a_bad_frame(void) {
    uint8_t pl[92] = {0}, bad[100], good[100];
    uint16_t nb = ubx_frame_build(bad, sizeof bad, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92);
    uint16_t ng = ubx_frame_build(good, sizeof good, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92);
    bad[40] ^= 0xFF;                                    // corrupt the payload
    ubx_parser_t p;
    ubx_parser_init(&p);
    assert(feed(&p, bad, nb) == 0);
    assert(feed(&p, good, ng) == 1);
    assert(p.stats.frames_ok == 1 && p.stats.checksum_errors == 1);
}

static void test_resync_keeps_counters_init_clears_them(void) {
    uint8_t pl[2] = {0x06, 0x8A}, f[16];
    uint16_t n = ubx_frame_build(f, sizeof f, UBX_CLASS_ACK, UBX_ID_ACK_ACK, pl, 2);
    ubx_parser_t p;
    ubx_parser_init(&p);
    assert(feed(&p, f, n) == 1);
    feed(&p, f, 4);                                     // stop in the middle of a frame
    ubx_parser_resync(&p);
    assert(p.stats.frames_ok == 1 && p.stats.resyncs == 1);
    assert(feed(&p, f, n) == 1);                        // hunting again, finds the next one
    ubx_parser_init(&p);
    assert(p.stats.frames_ok == 0 && p.stats.resyncs == 0 && p.stats.checksum_errors == 0);
}

/* ------------------------------------------------------------ NAV-PVT */

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i));
}
static void put_i32(uint8_t *p, int32_t v) { put_u32(p, (uint32_t)v); }

// A payload with every field set to a distinct, known value (offsets from the
// u-blox M10 interface description, written out here on purpose).
static void make_pvt(uint8_t pl[92], uint8_t fix_type, uint8_t flags) {
    memset(pl, 0, 92);
    put_u32(pl + 0, 123456789u);           // iTOW, ms
    put_u16(pl + 4, 2026);                 // year
    pl[6] = 11; pl[7] = 5; pl[8] = 9; pl[9] = 41; pl[10] = 27;
    pl[11] = 0x07;                         // validDate | validTime | fullyResolved
    put_u32(pl + 12, 2500);                // tAcc, ns
    put_i32(pl + 16, -1500);               // nano
    pl[20] = fix_type;
    pl[21] = flags;
    pl[23] = 17;                           // numSV
    put_i32(pl + 24, 170000000);           // lon  17.0 deg
    put_i32(pl + 28, -511234567);          // lat -51.1234567 deg
    put_i32(pl + 32, 123456);              // height above ellipsoid, mm
    put_i32(pl + 36, 120000);              // height above MSL, mm
    put_u32(pl + 40, 1800);                // hAcc, mm
    put_u32(pl + 44, 2400);                // vAcc, mm
    put_i32(pl + 48, 250000);              // velN, mm/s
    put_i32(pl + 52, -125000);             // velE
    put_i32(pl + 56, -310000);             // velD (negative: climbing)
    put_i32(pl + 60, 279508);              // gSpeed
    put_i32(pl + 64, 12345678);            // headMot, 1e-5 deg
    put_u32(pl + 68, 450);                 // sAcc, mm/s
    put_u32(pl + 72, 250000);              // headAcc, 1e-5 deg
    put_u16(pl + 76, 123);                 // pDOP, 0.01
}

static void test_decode_nav_pvt_fields_and_scaling(void) {
    uint8_t pl[92];
    make_pvt(pl, 3, 0x01);
    ubx_nav_pvt_t v;
    assert(ubx_decode_nav_pvt(pl, 92, &v));

    assert(v.itow_ms == 123456789u);
    assert(v.year == 2026 && v.month == 11 && v.day == 5);
    assert(v.hour == 9 && v.min == 41 && v.sec == 27);
    assert(v.valid_date && v.valid_time && v.fully_resolved);
    assert(v.t_acc_ns == 2500u && v.nano == -1500);
    assert(v.fix_type == UBX_FIX_3D && v.fix_ok && v.num_sv == 17);
    assert(v.lon_1e7 == 170000000 && v.lat_1e7 == -511234567);   // longitude comes first on the wire
    assert(v.height_mm == 123456 && v.h_msl_mm == 120000);
    assert(fabsf(v.h_acc_m - 1.8f) < 1e-5f && fabsf(v.v_acc_m - 2.4f) < 1e-5f);
    assert(fabsf(v.vel_ned_mps[0] - 250.0f) < 1e-3f);
    assert(fabsf(v.vel_ned_mps[1] + 125.0f) < 1e-3f);
    assert(fabsf(v.vel_ned_mps[2] + 310.0f) < 1e-3f);            // Down is positive downward
    assert(fabsf(v.ground_speed_mps - 279.508f) < 1e-3f);
    assert(fabsf(v.s_acc_mps - 0.45f) < 1e-5f);
    assert(fabsf(v.head_motion_deg - 123.45678f) < 1e-3f);
    assert(fabsf(v.head_acc_deg - 2.5f) < 1e-5f);
    assert(fabsf(v.p_dop - 1.23f) < 1e-5f);
}

static void test_decode_nav_pvt_flags_and_length(void) {
    uint8_t pl[92];
    ubx_nav_pvt_t v;

    make_pvt(pl, 3, 0x00);                               // gnssFixOK clear
    assert(ubx_decode_nav_pvt(pl, 92, &v) && !v.fix_ok);
    make_pvt(pl, 3, 0xFE);                               // every other flag set
    assert(ubx_decode_nav_pvt(pl, 92, &v) && !v.fix_ok);

    pl[11] = 0x00;                                       // time/date not valid
    assert(ubx_decode_nav_pvt(pl, 92, &v));
    assert(!v.valid_date && !v.valid_time && !v.fully_resolved);

    assert(!ubx_decode_nav_pvt(pl, 91, &v));             // wrong lengths are refused
    assert(!ubx_decode_nav_pvt(pl, 93, &v));
    assert(!ubx_decode_nav_pvt(pl, 0, &v));
}

static void test_nav_pvt_survives_the_wire(void) {
    // Decoding what the parser delivered, end to end.
    uint8_t pl[92], f[100];
    make_pvt(pl, 4, 0x01);
    uint16_t n = ubx_frame_build(f, sizeof f, UBX_CLASS_NAV, UBX_ID_NAV_PVT, pl, 92);
    ubx_parser_t p;
    ubx_parser_init(&p);
    assert(feed(&p, f, n) == 1);
    ubx_nav_pvt_t v;
    assert(ubx_decode_nav_pvt(p.payload, p.len, &v));
    assert(v.fix_type == UBX_FIX_GNSS_DR && v.num_sv == 17 && v.lat_1e7 == -511234567);
}

static void test_has_position_truth_table(void) {
    // Only a flagged-valid 3D or GNSS+dead-reckoning solution has a position.
    for (int type = 0; type <= 5; type++) {
        for (int ok = 0; ok <= 1; ok++) {
            ubx_nav_pvt_t v;
            memset(&v, 0, sizeof v);
            v.fix_type = (ubx_fix_type_t)type;
            v.fix_ok = ok != 0;
            const bool want = ok && (type == UBX_FIX_3D || type == UBX_FIX_GNSS_DR);
            assert(ubx_nav_pvt_has_position(&v) == want);
        }
    }
}

/* ---------------------------------------------------------------- ACK */

static void parse_ack(ubx_parser_t *p, uint8_t id, uint8_t cls, uint8_t msg) {
    uint8_t pl[2] = {cls, msg}, f[16];
    uint16_t n = ubx_frame_build(f, sizeof f, UBX_CLASS_ACK, id, pl, 2);
    ubx_parser_init(p);
    assert(feed(p, f, n) == 1);
}

static void test_is_ack_for(void) {
    ubx_parser_t p;
    bool acked = false;

    parse_ack(&p, UBX_ID_ACK_ACK, UBX_CLASS_CFG, UBX_ID_CFG_VALSET);
    assert(ubx_is_ack_for(&p, UBX_CLASS_CFG, UBX_ID_CFG_VALSET, &acked) && acked);

    parse_ack(&p, UBX_ID_ACK_NAK, UBX_CLASS_CFG, UBX_ID_CFG_VALSET);
    acked = true;
    assert(ubx_is_ack_for(&p, UBX_CLASS_CFG, UBX_ID_CFG_VALSET, &acked) && !acked);

    // An answer to a different message is not ours.
    parse_ack(&p, UBX_ID_ACK_ACK, UBX_CLASS_CFG, 0x01);
    assert(!ubx_is_ack_for(&p, UBX_CLASS_CFG, UBX_ID_CFG_VALSET, &acked));
    parse_ack(&p, UBX_ID_ACK_ACK, UBX_CLASS_NAV, UBX_ID_CFG_VALSET);
    assert(!ubx_is_ack_for(&p, UBX_CLASS_CFG, UBX_ID_CFG_VALSET, &acked));

    // A frame that is not an ACK/NAK at all.
    uint8_t pl[2] = {UBX_CLASS_CFG, UBX_ID_CFG_VALSET}, f[16];
    uint16_t n = ubx_frame_build(f, sizeof f, UBX_CLASS_NAV, 0x01, pl, 2);
    ubx_parser_init(&p);
    assert(feed(&p, f, n) == 1);
    assert(!ubx_is_ack_for(&p, UBX_CLASS_CFG, UBX_ID_CFG_VALSET, &acked));

    // Right class, unknown id, and a payload of the wrong length.
    n = ubx_frame_build(f, sizeof f, UBX_CLASS_ACK, 0x07, pl, 2);
    ubx_parser_init(&p);
    assert(feed(&p, f, n) == 1);
    assert(!ubx_is_ack_for(&p, UBX_CLASS_CFG, UBX_ID_CFG_VALSET, &acked));
    n = ubx_frame_build(f, sizeof f, UBX_CLASS_ACK, UBX_ID_ACK_ACK, pl, 1);
    ubx_parser_init(&p);
    assert(feed(&p, f, n) == 1);
    assert(!ubx_is_ack_for(&p, UBX_CLASS_CFG, UBX_ID_CFG_VALSET, &acked));
}

int main(void) {
    test_frame_build_known_vectors();
    test_frame_build_length_and_capacity();
    test_parse_reports_frame_on_last_byte();
    test_parse_payload_is_kept();
    test_parse_back_to_back_and_zero_length();
    test_parse_skips_garbage_and_sync_runs();
    test_parse_same_result_for_any_chunking();
    test_parse_counts_each_kind_of_error();
    test_parse_recovers_after_a_bad_frame();
    test_resync_keeps_counters_init_clears_them();
    test_decode_nav_pvt_fields_and_scaling();
    test_decode_nav_pvt_flags_and_length();
    test_nav_pvt_survives_the_wire();
    test_has_position_truth_table();
    test_is_ack_for();
    puts("test_ubx_parse: all passed");
    return 0;
}
