/*
 * ACS5 - u-blox UBX protocol, platform-independent layer.
 *
 * Frame state machine, Fletcher-8 checksum, UBX-NAV-PVT decode, frame
 * construction for the configuration messages, and the geodetic-to-local-NED
 * conversion.
 *
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ framing */

#define UBX_SYNC1 0xB5u
#define UBX_SYNC2 0x62u

#define UBX_CLASS_NAV 0x01u
#define UBX_CLASS_ACK 0x05u
#define UBX_CLASS_CFG 0x06u

#define UBX_ID_NAV_PVT    0x07u
#define UBX_ID_ACK_NAK    0x00u
#define UBX_ID_ACK_ACK    0x01u
#define UBX_ID_CFG_VALSET 0x8Au

#define UBX_NAV_PVT_LEN 92u
#define UBX_ACK_LEN      2u

/* Framing overhead: 2 sync + class + id + 2 length + 2 checksum. */
#define UBX_FRAME_OVERHEAD 8u

#define UBX_MAX_PAYLOAD UBX_NAV_PVT_LEN

typedef enum
{
    UBX_ST_SYNC1 = 0,
    UBX_ST_SYNC2,
    UBX_ST_CLASS,
    UBX_ST_ID,
    UBX_ST_LEN_LO,
    UBX_ST_LEN_HI,
    UBX_ST_PAYLOAD,
    UBX_ST_CK_A,
    UBX_ST_CK_B,
} ubx_state_t;

typedef struct
{
    uint8_t  state;          /* ubx_state_t */
    uint8_t  msg_class;
    uint8_t  msg_id;
    uint16_t len;            /* declared payload length of the frame in progress */
    uint16_t idx;            /* payload bytes consumed so far */
    uint8_t  ck_a, ck_b;
    uint8_t  payload[UBX_MAX_PAYLOAD];

    /* Health counters: */
    uint32_t frames_ok;
    uint32_t checksum_errors;
    uint32_t resyncs;        /* hunts restarted after a malformed frame */
    uint32_t oversized;
} ubx_parser_t;

void ubx_parser_init(ubx_parser_t *p);

/* Feeds one received byte. Returns true exactly when a complete frame with a
 * valid checksum is available.
 */
bool ubx_parse_byte(ubx_parser_t *p, uint8_t b);

/* Fletcher-8 over buf[0..len) */
void ubx_checksum(const uint8_t *buf, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);

/* Writes a complete frame into out (sync through checksum) and returns its
 * total length, or 0 if out_cap cannot hold it. */
uint16_t ubx_frame_build(uint8_t *out, uint16_t out_cap,
                         uint8_t msg_class, uint8_t msg_id,
                         const uint8_t *payload, uint16_t payload_len);

/* ------------------------------------------------------------------ NAV-PVT */

/* Fix quality. The numeric values are the receiver's. */
typedef enum
{
    UBX_FIX_NONE      = 0,
    UBX_FIX_DEAD_RECK = 1,
    UBX_FIX_2D        = 2,
    UBX_FIX_3D        = 3,
    UBX_FIX_GNSS_DR   = 4,
    UBX_FIX_TIME_ONLY = 5,
} ubx_fix_type_t;

/* Decoded UBX-NAV-PVT */
typedef struct
{
    uint32_t itow_ms;          /* GPS time of week, ms */

    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    bool     valid_date;
    bool     valid_time;
    bool     fully_resolved;   /* UTC offset known; time is unambiguous */
    uint32_t t_acc_ns;
    int32_t  nano;             /* fraction of second */

    uint8_t  fix_type;       
    bool     fix_ok;
    uint8_t  num_sv;           /* satellites used in the solution */

    int32_t  lon_1e7;
    int32_t  lat_1e7;
    int32_t  height_mm;        /* above the WGS-84 ellipsoid */
    int32_t  h_msl_mm;         /* above mean sea level (geoid) */

    float    h_acc_m;          /* horizontal accuracy estimate, 1 sigma */
    float    v_acc_m;
    /* North, East, Down. Down is POSITIVE DOWNWARD.*/
    float    vel_ned_mps[3];
    float    ground_speed_mps; 
    float    s_acc_mps;        /* speed accuracy estimate */
    float    head_motion_deg;  /* course over ground */
    float    head_acc_deg;
    float    p_dop;            /* position dilution of precision */
} ubx_nav_pvt_t;


bool ubx_decode_nav_pvt(const uint8_t *payload, uint16_t len, ubx_nav_pvt_t *out);

/* Decodes an ACK-ACK / ACK-NAK payload into the class and id being answered.
 * *acknowledged is false for a NAK. */
bool ubx_decode_ack(uint8_t msg_id, const uint8_t *payload, uint16_t len,
                    uint8_t *acked_class, uint8_t *acked_id, bool *acknowledged);

                 
/* ------------------------------------------------------------- local NED */

typedef struct
{
    int32_t lat_1e7;
    int32_t lon_1e7;
    int32_t height_mm;
    float   m_per_deg_lat;   /* meridian radius at lat_1e7*/
    float   m_per_deg_lon;   /* prime-vertical radius times cos(lat)*/
    bool    valid;
} ubx_datum_t;

void ubx_datum_clear(ubx_datum_t *d);


void ubx_datum_set(ubx_datum_t *d, int32_t lat_1e7, int32_t lon_1e7,
                   int32_t height_mm);


void ubx_geodetic_to_ned(const ubx_datum_t *d,
                         int32_t lat_1e7, int32_t lon_1e7, int32_t height_mm,
                         float out_ned_m[3]);
