/*
 * ACS5 - u-blox UBX protocol.
 */
#include "gnss/ubx_parse.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------ little endian */

//Read byte-by-byte 
static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t *p)
{
    /* Cast through uint32_t to avoid implementation-defined signed shifts. */
    return (int32_t)rd_u32(p);
}

/* ------------------------------------------------------------------ checksum */

void ubx_checksum(const uint8_t *buf, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        a = (uint8_t)(a + buf[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

/* ------------------------------------------------------------------- framing */

void ubx_parser_init(ubx_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = UBX_ST_SYNC1;
}

/* Abandons the frame in progress and resumes hunting for a sync pair. */
static void resync(ubx_parser_t *p)
{
    p->resyncs++;
    p->state = UBX_ST_SYNC1;
}

bool ubx_parse_byte(ubx_parser_t *p, uint8_t b)
{
    switch (p->state)
    {
    case UBX_ST_SYNC1:
        if (b == UBX_SYNC1)
            p->state = UBX_ST_SYNC2;
        break;

    case UBX_ST_SYNC2:
        if (b == UBX_SYNC2)
        {
            p->state = UBX_ST_CLASS;
        }
        else if (b == UBX_SYNC1)
        {
            /* B5 B5 62: the second B5 is the real frame's first byte. Staying
             * here rather than returning to SYNC1 is what makes a run of B5
             * bytes before a frame harmless. */
        }
        else
        {
            p->state = UBX_ST_SYNC1;
        }
        break;

    case UBX_ST_CLASS:
        p->msg_class = b;
        p->ck_a      = b;
        p->ck_b      = b;
        p->state     = UBX_ST_ID;
        break;

    case UBX_ST_ID:
        p->msg_id = b;
        p->ck_a   = (uint8_t)(p->ck_a + b);
        p->ck_b   = (uint8_t)(p->ck_b + p->ck_a);
        p->state  = UBX_ST_LEN_LO;
        break;

    case UBX_ST_LEN_LO:
        p->len   = b;
        p->ck_a  = (uint8_t)(p->ck_a + b);
        p->ck_b  = (uint8_t)(p->ck_b + p->ck_a);
        p->state = UBX_ST_LEN_HI;
        break;

    case UBX_ST_LEN_HI:
        p->len  = (uint16_t)(p->len | ((uint16_t)b << 8));
        p->ck_a = (uint8_t)(p->ck_a + b);
        p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        p->idx  = 0;

        if (p->len > UBX_MAX_PAYLOAD)
        {
            /* Either an unsupported message or a corrupt header; see the
             * UBX_MAX_PAYLOAD comment for why both resync rather than skip. */

            p->oversized++;
            resync(p);
        }
        else
        {
            /* Zero-length messages skip the payload and go straight to checksum. */
            p->state = (p->len == 0u) ? UBX_ST_CK_A : UBX_ST_PAYLOAD;
        }
        break;

    case UBX_ST_PAYLOAD:
        p->payload[p->idx++] = b;
        p->ck_a              = (uint8_t)(p->ck_a + b);
        p->ck_b              = (uint8_t)(p->ck_b + p->ck_a);
        if (p->idx >= p->len)
            p->state = UBX_ST_CK_A;
        break;

    case UBX_ST_CK_A:
        if (b != p->ck_a)
        {
            p->checksum_errors++;
            resync(p);
            break;
        }
        p->state = UBX_ST_CK_B;
        break;

    case UBX_ST_CK_B:
        p->state = UBX_ST_SYNC1;
        if (b != p->ck_b)
        {
            p->checksum_errors++;
            /* The frame started but was invalid. */
            p->resyncs++;
            break;
        }
        p->frames_ok++;
        return true;

    default:
        /* Recover from a corrupted state instead of hanging the parser. */
        resync(p);
        break;
    }

    return false;
}

uint16_t ubx_frame_build(uint8_t *out, uint16_t out_cap,
                         uint8_t msg_class, uint8_t msg_id,
                         const uint8_t *payload, uint16_t payload_len)
{
    const uint32_t total = (uint32_t)payload_len + UBX_FRAME_OVERHEAD;
    if (out == NULL || total > out_cap)
        return 0;
    if (payload_len > 0u && payload == NULL)
        return 0;

    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = msg_class;
    out[3] = msg_id;
    out[4] = (uint8_t)(payload_len & 0xFFu);
    out[5] = (uint8_t)(payload_len >> 8);
    if (payload_len > 0u)
        memcpy(&out[6], payload, payload_len);

    uint8_t ck_a, ck_b;
    /* Class..payload inclusive, i.e. everything after the sync pair. */
    ubx_checksum(&out[2], (uint16_t)(payload_len + 4u), &ck_a, &ck_b);
    out[6 + payload_len] = ck_a;
    out[7 + payload_len] = ck_b;

    return (uint16_t)total;
}

/* ------------------------------------------------------------------ NAV-PVT */

// Field offsets within the 92-byte NAV-PVT payload. Note that longitude precedes latitude.
enum
{
    PVT_ITOW    = 0,
    PVT_YEAR    = 4,
    PVT_MONTH   = 6,
    PVT_DAY     = 7,
    PVT_HOUR    = 8,
    PVT_MIN     = 9,
    PVT_SEC     = 10,
    PVT_VALID   = 11,
    PVT_TACC    = 12,
    PVT_NANO    = 16,
    PVT_FIXTYPE = 20,
    PVT_FLAGS   = 21,
    PVT_NUMSV   = 23,
    PVT_LON     = 24,
    PVT_LAT     = 28,
    PVT_HEIGHT  = 32,
    PVT_HMSL    = 36,
    PVT_HACC    = 40,
    PVT_VACC    = 44,
    PVT_VELN    = 48,
    PVT_VELE    = 52,
    PVT_VELD    = 56,
    PVT_GSPEED  = 60,
    PVT_HEADMOT = 64,
    PVT_SACC    = 68,
    PVT_HEADACC = 72,
    PVT_PDOP    = 76,
};

//valid (offset 11) bit positions. 
#define PVT_VALID_DATE     0x01u
#define PVT_VALID_TIME     0x02u
#define PVT_FULLY_RESOLVED 0x04u

// flags (offset 21) bit positions. 
#define PVT_FLAGS_GNSS_FIX_OK 0x01u

bool ubx_decode_nav_pvt(const uint8_t *payload, uint16_t len, ubx_nav_pvt_t *out)
{
    if (payload == NULL || out == NULL || len != UBX_NAV_PVT_LEN)
        return false;

    memset(out, 0, sizeof(*out));

    out->itow_ms = rd_u32(&payload[PVT_ITOW]);

    out->year  = rd_u16(&payload[PVT_YEAR]);
    out->month = payload[PVT_MONTH];
    out->day   = payload[PVT_DAY];
    out->hour  = payload[PVT_HOUR];
    out->min   = payload[PVT_MIN];
    out->sec   = payload[PVT_SEC];

    const uint8_t valid  = payload[PVT_VALID];
    out->valid_date      = (valid & PVT_VALID_DATE) != 0u;
    out->valid_time      = (valid & PVT_VALID_TIME) != 0u;
    out->fully_resolved  = (valid & PVT_FULLY_RESOLVED) != 0u;
    out->t_acc_ns        = rd_u32(&payload[PVT_TACC]);
    out->nano            = rd_i32(&payload[PVT_NANO]);

    out->fix_type = payload[PVT_FIXTYPE];
    out->fix_ok   = (payload[PVT_FLAGS] & PVT_FLAGS_GNSS_FIX_OK) != 0u;
    out->num_sv   = payload[PVT_NUMSV];

    out->lon_1e7   = rd_i32(&payload[PVT_LON]);
    out->lat_1e7   = rd_i32(&payload[PVT_LAT]);
    out->height_mm = rd_i32(&payload[PVT_HEIGHT]);
    out->h_msl_mm  = rd_i32(&payload[PVT_HMSL]);

    out->h_acc_m = (float)rd_u32(&payload[PVT_HACC]) * 1e-3f;
    out->v_acc_m = (float)rd_u32(&payload[PVT_VACC]) * 1e-3f;

    out->vel_ned_mps[0] = (float)rd_i32(&payload[PVT_VELN]) * 1e-3f;
    out->vel_ned_mps[1] = (float)rd_i32(&payload[PVT_VELE]) * 1e-3f;
    out->vel_ned_mps[2] = (float)rd_i32(&payload[PVT_VELD]) * 1e-3f;

    out->ground_speed_mps = (float)rd_i32(&payload[PVT_GSPEED]) * 1e-3f;
    out->s_acc_mps        = (float)rd_u32(&payload[PVT_SACC]) * 1e-3f;
    out->head_motion_deg  = (float)rd_i32(&payload[PVT_HEADMOT]) * 1e-5f;
    out->head_acc_deg     = (float)rd_u32(&payload[PVT_HEADACC]) * 1e-5f;
    out->p_dop            = (float)rd_u16(&payload[PVT_PDOP]) * 0.01f;

    return true;
}

bool ubx_decode_ack(uint8_t msg_id, const uint8_t *payload, uint16_t len,
                    uint8_t *acked_class, uint8_t *acked_id, bool *acknowledged)
{
    if (payload == NULL || acked_class == NULL || acked_id == NULL ||
        acknowledged == NULL || len != UBX_ACK_LEN)
        return false;
    if (msg_id != UBX_ID_ACK_ACK && msg_id != UBX_ID_ACK_NAK)
        return false;

    *acked_class   = payload[0];
    *acked_id      = payload[1];
    *acknowledged  = (msg_id == UBX_ID_ACK_ACK);
    return true;
}

/* ------------------------------------------------------------- local NED */

/* WGS-84 constants used for local NED conversion
* WGS-84. e2 = 2f - f^2 for f = 1/298.257223563. */
#define WGS84_A  6378137.0
#define WGS84_E2 0.00669437999014
#define DEG2RAD  0.017453292519943295

void ubx_datum_clear(ubx_datum_t *d)
{
    memset(d, 0, sizeof(*d));
}

void ubx_datum_set(ubx_datum_t *d, int32_t lat_1e7, int32_t lon_1e7,
                   int32_t height_mm)
{
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

void ubx_geodetic_to_ned(const ubx_datum_t *d,
                         int32_t lat_1e7, int32_t lon_1e7, int32_t height_mm,
                         float out_ned_m[3])
{
    if (d == NULL || out_ned_m == NULL)
        return;

    if (!d->valid)
    {
        out_ned_m[0] = 0.0f;
        out_ned_m[1] = 0.0f;
        out_ned_m[2] = 0.0f;
        return;
    }

    // The differences are taken as integers so no precision is lost before the scaling. 
    const int64_t dlat = (int64_t)lat_1e7 - (int64_t)d->lat_1e7;
    const int64_t dlon = (int64_t)lon_1e7 - (int64_t)d->lon_1e7;
    const int64_t dh   = (int64_t)height_mm - (int64_t)d->height_mm;

    out_ned_m[0] = (float)((double)dlat * 1e-7) * d->m_per_deg_lat;
    out_ned_m[1] = (float)((double)dlon * 1e-7) * d->m_per_deg_lon;
    /* Down is positive, so a height gain is negative D. */
    out_ned_m[2] = -(float)((double)dh * 1e-3);
}
