/* Binary flight-log record: one per log tick, little-endian, CRC-protected.
 * Layout is mirrored by tools/decode_log.py and described in docs/log_format.md. */
#pragma once

#include <stddef.h>
#include <stdint.h>

#define LOG_RECORD_MAGIC   0x474C3541u   /* "A5LG" read as bytes */
#define LOG_RECORD_VERSION 1u

/* log_record_t.flags */
#define LOG_FLAG_GNSS_FIX       (1u << 0)
#define LOG_FLAG_ACCEL_CLIPPED  (1u << 1)
#define LOG_FLAG_GYRO_CLIPPED_X (1u << 2)   /* chip axes, before the board remap */
#define LOG_FLAG_GYRO_CLIPPED_Y (1u << 3)
#define LOG_FLAG_GYRO_CLIPPED_Z (1u << 4)
#define LOG_FLAG_RATE_FRESH     (1u << 5)

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;              /* sizeof(log_record_t) */
    uint32_t seq;               /* consecutive per record published, dropped ones included */
    uint32_t tick_ms;

    uint8_t  flight_state;
    uint8_t  ctrl_phase;
    uint8_t  ctrl_arm;
    uint8_t  flags;             /* LOG_FLAG_* */

    float    pos_m[3];          /* world, pad origin */
    float    vel_mps[3];
    float    acc_mps2[3];       /* world, gravity removed */
    float    eul_rad[3];        /* ZYX Euler; [2] is about body Z, the airframe roll axis */
    float    roll_twist_rad;    /* twist of the attitude about body Z */
    float    ang_vel_rads[3];   /* body, gyro bias removed */
    float    accel_raw_mps2[3]; /* chip axes, before the board remap */
    float    gyro_raw_rads[3];
    float    mag_raw_ut[3];
    float    baro_alt_m;        /* above the pad datum */
    float    baro_press_pa;
    float    baro_temp_c;
    float    ctrl_theta_sp_rad;
    float    ctrl_rate_sp_rads;
    float    ctrl_theta_meas_rad;
    float    ctrl_canard_cmd_rad;
    uint16_t ctrl_canard_us;    /* fin 0 pulse after the servo slew limit */
    uint8_t  gnss_num_sv;
    uint8_t  false_launch_count;
    float    gnss_ned_m[3];     /* Down positive */
    float    gnss_vel_ned_mps[3];
    int32_t  gnss_lat_1e7;
    int32_t  gnss_lon_1e7;
    int32_t  gnss_h_msl_mm;
    uint32_t gyro_bias_n;       /* pad samples behind the gyro bias */
    uint32_t log_dropped;       /* records lost to a full queue so far */
    uint32_t log_errors;        /* SD write/sync failures so far */

    uint32_t crc32;             /* CRC-32 (zlib) over every byte before this field */
} log_record_t;

/* tools/decode_log.py hard-codes this size; change both together. */
_Static_assert(sizeof(log_record_t) == 204, "log_record_t layout changed");

/* CRC-32 as computed by zlib.crc32 (reflected, poly 0xEDB88320). */
uint32_t log_crc32(const void *data, size_t len);

/* Fill magic, version, size and seq, then the CRC over the finished record. */
void log_record_seal(log_record_t *r, uint32_t seq);

/* True when magic, version, size and CRC all match. */
int log_record_valid(const log_record_t *r);
