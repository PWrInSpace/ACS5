/*
 * ACS5 - MAX-M10S HAL interface.
 *
 * This file only moves bytes: DMA setup, CFG-VALSET framing, ACK waiting and
 * the drain loop.
 */

#include "stm32h7xx_hal.h"

#include "gnss/max_m10s.h"

#include <string.h>

#include "utils/attributes.h"

/* ------------------------------------------------------------------- CFG-VALSET
 *
 * payload = version(1) + layers(1) + reserved(2), then one or more
 *           key(4) + value(1|2|4) pairs. */
#define MAX_M10S_CFG_HDR_LEN     4u
#define MAX_M10S_CFG_VAL_MAX_LEN 4u
#define MAX_M10S_CFG_ITEM_MAX_LEN (4u + MAX_M10S_CFG_VAL_MAX_LEN)
#define MAX_M10S_CFG_MAX_ITEMS   8u   /* most pairs one message here may carry */
#define MAX_M10S_CFG_PAYLOAD_CAP \
    (MAX_M10S_CFG_HDR_LEN + MAX_M10S_CFG_MAX_ITEMS * MAX_M10S_CFG_ITEM_MAX_LEN)
#define MAX_M10S_CFG_FRAME_CAP   (MAX_M10S_CFG_PAYLOAD_CAP + UBX_FRAME_OVERHEAD)

/* One key/value pair of a CFG-VALSET message. */
typedef struct
{
    uint32_t key;
    uint32_t value;
    uint8_t  size;   /* value width in bytes: 1, 2 or 4 */
} cfg_item_t;

/* CFG-VALSET RAM layer. */
#define MAX_M10S_CFG_LAYER_RAM 0x01u

/* Configuration keys (u-blox M10 SPG 5.10 interface description). */
#define MAX_M10S_KEY_UART1_BAUDRATE         0x40520001u   /* U4 */
#define MAX_M10S_KEY_NAVSPG_DYNMODEL        0x20110021u   /* U1 */
#define MAX_M10S_KEY_RATE_MEAS              0x30210001u   /* U2, ms */
#define MAX_M10S_KEY_RATE_NAV               0x30210002u   /* U2, cycles */
#define MAX_M10S_KEY_MSGOUT_NAV_PVT_UART1   0x20910007u   /* U1 */
#define MAX_M10S_KEY_UART1OUTPROT_NMEA      0x10740002u   /* L  */

/* Configuration values. */
#define MAX_M10S_DYNMODEL_AIRBORNE_4G  8u    /* airborne, < 4 g */
#define MAX_M10S_MEAS_PERIOD_MS        50u   /* 50 ms -> 20 Hz with NAV rate 1 */
#define MAX_M10S_NAV_RATE_CYCLES       1u    /* one solution per measurement */

/* ------------------------------------------------------------ tuning constants */

/* DMA RX ring size. */
#define MAX_M10S_DMA_BUF_LEN    512u

/* UART speed used after configuration, and the module's power-on speed. The
 * configuration lives in the module's RAM, so after an MCU-only reset the module
 * is still at the flight speed; after a power cycle it is at the boot speed. */
#define MAX_M10S_BAUD_FLIGHT    115200u
#define MAX_M10S_BAUD_BOOT      38400u

/* Blocking HAL transmit timeout for one CFG-VALSET frame. */
#define MAX_M10S_TX_TIMEOUT_MS  50u

/* Per-message ACK timeout. */
#define MAX_M10S_CFG_TIMEOUT_MS 200u

/* Retry budget per key before failing configure(). */
#define MAX_M10S_CFG_RETRIES    3u

/* ------------------------------------------------------------------ DMA buffer */

static uint8_t s_dma_buf[MAX_M10S_DMA_BUF_LEN] ATTR_SECTION(".dma_buf") ATTR_ALIGNED(32);

/* Index the DMA will write next. NDTR counts down from the buffer length and
 * can read as 0 (or the full length right after a wrap); the modulo folds a
 * full-length result back to 0 so the result is always a valid ring index and
 * a read index can actually equal it. */
static uint16_t dma_write_pos(void)
{
    const uint16_t ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(MAX_M10S_HUART_HANDLE.hdmarx);
    return (uint16_t)((MAX_M10S_DMA_BUF_LEN - ndtr) % MAX_M10S_DMA_BUF_LEN);
}

/* Starts circular receive-to-idle DMA into the ring buffer. */
static bool start_rx_dma(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&MAX_M10S_HUART_HANDLE, s_dma_buf,
                                     MAX_M10S_DMA_BUF_LEN) != HAL_OK)
        return false;

    /* Idle interrupt is the drain trigger; HT interrupt is not used. */
    __HAL_DMA_DISABLE_IT(MAX_M10S_HUART_HANDLE.hdmarx, DMA_IT_HT);
    return true;
}

/* Drops cached copies of the ring bytes in [from, to), which may wrap.*/
static void invalidate_ring(uint16_t from, uint16_t to)
{
    if (from == to)
        return;

    if (from < to)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)&s_dma_buf[from], to - from);
        return;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t *)&s_dma_buf[from],
                                 MAX_M10S_DMA_BUF_LEN - from);
    if (to > 0u)
        SCB_InvalidateDCache_by_Addr((uint32_t *)s_dma_buf, to);
}

/* Feeds ring bytes from *read_pos up to write_pos to the parser. Stops and
 * returns true at the first complete frame, with *read_pos just past it;
 * returns false once write_pos is reached.
 */
static bool ring_feed(ubx_parser_t *parser, uint16_t *read_pos, uint16_t write_pos)
{
    invalidate_ring(*read_pos, write_pos);

    while (*read_pos != write_pos)
    {
        const uint8_t b = s_dma_buf[*read_pos];
        *read_pos = (uint16_t)((*read_pos + 1u) % MAX_M10S_DMA_BUF_LEN);

        if (ubx_parse_byte(parser, b))
            return true;
    }
    return false;
}


/* Writes the low `size` bytes of value to dst, little-endian. */
static void put_le(uint8_t *dst, uint32_t value, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
        dst[i] = (uint8_t)((value >> (8u * i)) & 0xFFu);
}

/* -------------------------------------------------------------- CFG-VALSET ACK
 *
 * Sends one CFG-VALSET carrying `count` key/value pairs and waits for a single
 * ACK-ACK or ACK-NAK for the whole message, retransmitting up to `attempts`
 * times. */
static max_m10s_status_t send_cfg_valset(const cfg_item_t *items, size_t count,
                                         uint8_t attempts)
{
    uint8_t payload[MAX_M10S_CFG_PAYLOAD_CAP];
    uint8_t frame[MAX_M10S_CFG_FRAME_CAP];

    if (count == 0u || count > MAX_M10S_CFG_MAX_ITEMS)
        return MAX_M10S_ERR_HAL;

    payload[0] = 0x00u;                     /* version */
    payload[1] = MAX_M10S_CFG_LAYER_RAM;    /* layers */
    payload[2] = 0x00u;                     /* reserved */
    payload[3] = 0x00u;                     /* reserved */

    uint16_t payload_len = MAX_M10S_CFG_HDR_LEN;
    for (size_t i = 0; i < count; i++)
    {
        if (items[i].size == 0u || items[i].size > MAX_M10S_CFG_VAL_MAX_LEN)
            return MAX_M10S_ERR_HAL;

        put_le(&payload[payload_len], items[i].key, 4u);
        payload_len = (uint16_t)(payload_len + 4u);
        put_le(&payload[payload_len], items[i].value, items[i].size);
        payload_len = (uint16_t)(payload_len + items[i].size);
    }

    const uint16_t frame_len   = ubx_frame_build(frame, sizeof(frame),
                                                 UBX_CLASS_CFG, UBX_ID_CFG_VALSET,
                                                 payload, payload_len);
    if (frame_len == 0u)
        return MAX_M10S_ERR_HAL;

    for (uint8_t attempt = 0; attempt < attempts; attempt++)
    {
        if (HAL_UART_Transmit(&MAX_M10S_HUART_HANDLE, frame, frame_len,
                              MAX_M10S_TX_TIMEOUT_MS) != HAL_OK)
            return MAX_M10S_ERR_HAL;

        /* Snapshot write pointer; only newer bytes can belong to this ACK. */
        ubx_parser_t ack_parser;
        ubx_parser_init(&ack_parser);
        uint16_t read_pos = dma_write_pos();

        const uint32_t start = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start) < MAX_M10S_CFG_TIMEOUT_MS)
        {
            while (ring_feed(&ack_parser, &read_pos, dma_write_pos()))
            {
                bool acknowledged;
                if (!ubx_is_ack_for(&ack_parser, UBX_CLASS_CFG,
                                    UBX_ID_CFG_VALSET, &acknowledged))
                    continue;

                if (!acknowledged)
                    return MAX_M10S_ERR_NAK;   /* NAK: do not retry. */

                return MAX_M10S_OK;
            }
        }
        /* Timed out this attempt; loop retransmits and waits again. */
    }

    return MAX_M10S_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------------- init */

/* The zeroing below also puts the parser in its initial state. */
_Static_assert(UBX_ST_SYNC1 == 0, "memset() must leave the parser in UBX_ST_SYNC1");

max_m10s_status_t max_m10s_init(max_m10s_driver_t *drv)
{
    memset(drv, 0, sizeof(*drv));

    return start_rx_dma() ? MAX_M10S_OK : MAX_M10S_ERR_HAL;
}

/* -------------------------------------------------------------------- configure */

/* The baud-rate setting. Sent on its own, before anything else. */
static const cfg_item_t s_baud_item =
    { MAX_M10S_KEY_UART1_BAUDRATE, MAX_M10S_BAUD_FLIGHT, 4u };

/* Everything else, sent after the baud change as one CFG-VALSET message with a
 * single ACK. */
static const cfg_item_t s_cfg_table[] =
{
    { MAX_M10S_KEY_NAVSPG_DYNMODEL,      MAX_M10S_DYNMODEL_AIRBORNE_4G, 1u },
    { MAX_M10S_KEY_RATE_MEAS,            MAX_M10S_MEAS_PERIOD_MS,       2u },
    { MAX_M10S_KEY_RATE_NAV,             MAX_M10S_NAV_RATE_CYCLES,      2u },
    { MAX_M10S_KEY_MSGOUT_NAV_PVT_UART1, 1u,                            1u },  /* enable */
    { MAX_M10S_KEY_UART1OUTPROT_NMEA,    0u,                            1u },  /* disable */
};

#define CFG_TABLE_LEN (sizeof(s_cfg_table) / sizeof(s_cfg_table[0]))
_Static_assert(CFG_TABLE_LEN <= MAX_M10S_CFG_MAX_ITEMS,
               "configuration table does not fit in one CFG-VALSET message");

/* Re-initialises the STM32 UART at `baud` and restarts the receive DMA (the
 * peripheral resets on a baud change). */
static bool set_host_baud(uint32_t baud)
{
    if (HAL_UART_AbortReceive(&MAX_M10S_HUART_HANDLE) != HAL_OK)
        return false;
    MAX_M10S_HUART_HANDLE.Init.BaudRate = baud;
    if (HAL_UART_Init(&MAX_M10S_HUART_HANDLE) != HAL_OK)
        return false;
    return start_rx_dma();
}

/* Brings the link up at MAX_M10S_BAUD_FLIGHT whatever speed the module is at.
 *
 * 1. Try the flight speed first: if the module is already there, and its ACK 
 *    proves the link.
 * 2. No answer: the module is at the boot speed. Send the same command at the
 *    boot speed.
 * 3. Switch the STM32 to the flight speed and require an ACK there. */
static max_m10s_status_t negotiate_baud(void)
{
    max_m10s_status_t st;

    if (!set_host_baud(MAX_M10S_BAUD_FLIGHT))
        return MAX_M10S_ERR_HAL;
    st = send_cfg_valset(&s_baud_item, 1u, 1u);
    if (st != MAX_M10S_ERR_TIMEOUT)
        return st;   /* OK: already at flight speed. NAK or HAL error: give up. */

    if (!set_host_baud(MAX_M10S_BAUD_BOOT))
        return MAX_M10S_ERR_HAL;
    st = send_cfg_valset(&s_baud_item, 1u, 1u);
    if (st != MAX_M10S_OK && st != MAX_M10S_ERR_TIMEOUT)
        return st;

    if (!set_host_baud(MAX_M10S_BAUD_FLIGHT))
        return MAX_M10S_ERR_HAL;
    return send_cfg_valset(&s_baud_item, 1u, MAX_M10S_CFG_RETRIES);
}

max_m10s_status_t max_m10s_configure(max_m10s_driver_t *drv)
{
    max_m10s_status_t st = negotiate_baud();
    if (st != MAX_M10S_OK)
        return st;

    st = send_cfg_valset(s_cfg_table, CFG_TABLE_LEN, MAX_M10S_CFG_RETRIES);
    if (st != MAX_M10S_OK)
        return st;

    /* Reset parser read pointer. Data already in flight may be skipped. */
    drv->read_pos = dma_write_pos();

    return MAX_M10S_OK;
}

/* ----------------------------------------------------------------------- drain */

bool max_m10s_drain(max_m10s_driver_t *drv, max_m10s_frame_t *frame)
{
    /* A receive error stopped the DMA: restart it, and drop what was in flight
     * (read pointer to the new write position, parser back to hunting). If the
     * restart itself fails, stay pending and try again on the next call. */
    if (drv->rx_restart_pending)
    {
        if (!start_rx_dma())
            return false;
        drv->rx_restart_pending = false;
        drv->read_pos = dma_write_pos();
        ubx_parser_resync(&drv->parser);
    }

    const uint16_t write_pos = dma_write_pos();

    if (write_pos == drv->read_pos)
        return false;

    if (!ring_feed(&drv->parser, &drv->read_pos, write_pos))
        return false;

    frame->msg_class = drv->parser.msg_class;
    frame->msg_id    = drv->parser.msg_id;
    frame->len       = drv->parser.len;
    frame->payload   = drv->parser.payload;
    return true;
}

/* ------------------------------------------------------------------- stats */

void max_m10s_on_uart_error(max_m10s_driver_t *drv)
{
    if ((HAL_UART_GetError(&MAX_M10S_HUART_HANDLE) & HAL_UART_ERROR_ORE) != 0u)
        drv->uart_overruns++;
    else
        drv->uart_rx_errors++;

    drv->rx_restart_pending = true;
}

void max_m10s_get_stats(const max_m10s_driver_t *drv, max_m10s_stats_t *out)
{
    out->frames_ok       = drv->parser.frames_ok;
    out->checksum_errors = drv->parser.checksum_errors;
    out->resyncs         = drv->parser.resyncs;
    out->oversized       = drv->parser.oversized;
    out->uart_overruns   = drv->uart_overruns;
    out->uart_rx_errors  = drv->uart_rx_errors;
}
