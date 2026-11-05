/*
 * ACS5 - u-blox UBX protocol.
 */
#include "gnss/ubx_parse.h"

#include <assert.h>
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

/* One Fletcher-8 step: fold byte `b` into the running sums. */
static void ck_step(uint8_t *ck_a, uint8_t *ck_b, uint8_t b)
{
    *ck_a = (uint8_t)(*ck_a + b);
    *ck_b = (uint8_t)(*ck_b + *ck_a);
}

/* Fletcher-8 over buf[0..len). Only used to build outgoing frames; the parser
 * folds each received byte in with ck_step() as it arrives. */
static void ubx_checksum(const uint8_t *buf, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (uint16_t i = 0; i < len; i++)
        ck_step(ck_a, ck_b, buf[i]);
}

/* ------------------------------------------------------------------- framing */

void ubx_parser_init(ubx_parser_t *p)
{
    assert(p != NULL);
    memset(p, 0, sizeof(*p));
    p->state = UBX_ST_SYNC1;
}

/* Abandons the frame in progress and resumes hunting for a sync pair. */
void ubx_parser_resync(ubx_parser_t *p)
{
    assert(p != NULL);
    p->resyncs++;
    p->state = UBX_ST_SYNC1;
}

bool ubx_parse_byte(ubx_parser_t *p, uint8_t b)
{
    assert(p != NULL);

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
        p->ck_a      = 0;
        p->ck_b      = 0;
        ck_step(&p->ck_a, &p->ck_b, b);
        p->state     = UBX_ST_ID;
        break;

    case UBX_ST_ID:
        p->msg_id = b;
        ck_step(&p->ck_a, &p->ck_b, b);
        p->state  = UBX_ST_LEN_LO;
        break;

    case UBX_ST_LEN_LO:
        p->len   = b;
        ck_step(&p->ck_a, &p->ck_b, b);
        p->state = UBX_ST_LEN_HI;
        break;

    case UBX_ST_LEN_HI:
        p->len  = (uint16_t)(p->len | ((uint16_t)b << 8));
        ck_step(&p->ck_a, &p->ck_b, b);
        p->idx  = 0;

        if (p->len > UBX_MAX_PAYLOAD)
        {
            /* Either an unsupported message or a corrupt header; see the
             * UBX_MAX_PAYLOAD comment for why both resync rather than skip. */

            p->oversized++;
            ubx_parser_resync(p);
        }
        else
        {
            /* Zero-length messages skip the payload and go straight to checksum. */
            p->state = (p->len == 0u) ? UBX_ST_CK_A : UBX_ST_PAYLOAD;
        }
        break;

    case UBX_ST_PAYLOAD:
        p->payload[p->idx++] = b;
        ck_step(&p->ck_a, &p->ck_b, b);
        if (p->idx >= p->len)
            p->state = UBX_ST_CK_A;
        break;

    case UBX_ST_CK_A:
        if (b != p->ck_a)
        {
            p->checksum_errors++;
            ubx_parser_resync(p);
            break;
        }
        p->state = UBX_ST_CK_B;
        break;

    case UBX_ST_CK_B:
        if (b != p->ck_b)
        {
            p->checksum_errors++;
            ubx_parser_resync(p);
            break;
        }
        p->state = UBX_ST_SYNC1;
        p->frames_ok++;
        return true;

    default:
        /* Recover from a corrupted state instead of hanging the parser. */
        ubx_parser_resync(p);
        break;
    }

    return false;
}

uint16_t ubx_frame_build(uint8_t *out, uint16_t out_cap,
                         uint8_t msg_class, uint8_t msg_id,
                         const uint8_t *payload, uint16_t payload_len)
{
    const uint32_t total = (uint32_t)payload_len + UBX_FRAME_OVERHEAD;
    assert(out != NULL);
    assert(payload_len == 0u || payload != NULL);

    if (total > out_cap)
        return 0;

    out[UBX_OFS_SYNC1]   = UBX_SYNC1;
    out[UBX_OFS_SYNC2]   = UBX_SYNC2;
    out[UBX_OFS_CLASS]   = msg_class;
    out[UBX_OFS_ID]      = msg_id;
    out[UBX_OFS_LEN]     = (uint8_t)(payload_len & 0xFFu);
    out[UBX_OFS_LEN + 1u] = (uint8_t)(payload_len >> 8);
    if (payload_len > 0u)
        memcpy(&out[UBX_OFS_PAYLOAD], payload, payload_len);

    /* The checksum covers class .. payload, i.e. everything after the sync
     * pair and before the checksum itself, and is stored right after the payload. */
    const uint16_t ck_pos = (uint16_t)(UBX_OFS_PAYLOAD + payload_len);
    uint8_t ck_a, ck_b;
    ubx_checksum(&out[UBX_OFS_CLASS],
                 (uint16_t)(ck_pos - UBX_OFS_CLASS), &ck_a, &ck_b);
    out[ck_pos]      = ck_a;
    out[ck_pos + 1u] = ck_b;

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
    assert(payload != NULL && out != NULL);

    if (len != UBX_NAV_PVT_LEN)
        return false;

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

    out->fix_type = (ubx_fix_type_t)payload[PVT_FIXTYPE];
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

bool ubx_is_ack_for(const ubx_parser_t *p, uint8_t cls, uint8_t id, bool *acked)
{
    assert(p != NULL && acked != NULL);

    if (p->msg_class != UBX_CLASS_ACK)
        return false;
    if (p->msg_id != UBX_ID_ACK_ACK && p->msg_id != UBX_ID_ACK_NAK)
        return false;
    if (p->len != UBX_ACK_LEN || p->payload[0] != cls || p->payload[1] != id)
        return false;

    *acked = (p->msg_id == UBX_ID_ACK_ACK);
    return true;
}
