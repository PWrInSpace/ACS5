#include "log/log_record.h"

uint32_t log_crc32(const void *data, size_t len)
{
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
    return ~crc;
}

void log_record_seal(log_record_t *r, uint32_t seq)
{
    r->magic   = LOG_RECORD_MAGIC;
    r->version = LOG_RECORD_VERSION;
    r->size    = (uint16_t)sizeof *r;
    r->seq     = seq;
    r->crc32   = log_crc32(r, offsetof(log_record_t, crc32));
}

int log_record_valid(const log_record_t *r)
{
    return r->magic == LOG_RECORD_MAGIC && r->version == LOG_RECORD_VERSION &&
           r->size == sizeof *r && r->crc32 == log_crc32(r, offsetof(log_record_t, crc32));
}
