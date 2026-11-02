/*
 * ACS5 - MAX-M10S HAL interface.
 *
 * This file only moves bytes: DMA setup, CFG-VALSET framing, ACK waiting and
 * the drain loop.
 */

#if __has_include("stm32h7xx_hal.h")
#include "stm32h7xx_hal.h"
#endif

#include "gnss/max_m10s.h"

#include <string.h>

#include "utils/attributes.h"

/* ------------------------------------------------------------------- CFG-VALSET
 *
 * payload = version(1) + layers(1) + reserved(2) + key(4) + value(1|2|4). */
#define MAX_M10S_CFG_HDR_LEN     8u
#define MAX_M10S_CFG_VAL_MAX_LEN 4u
#define MAX_M10S_CFG_PAYLOAD_CAP (MAX_M10S_CFG_HDR_LEN + MAX_M10S_CFG_VAL_MAX_LEN)
#define MAX_M10S_CFG_FRAME_CAP   (MAX_M10S_CFG_PAYLOAD_CAP + UBX_FRAME_OVERHEAD)

/* CFG-VALSET RAM layer. */
#define MAX_M10S_CFG_LAYER_RAM 0x01u

/* ------------------------------------------------------------ tuning constants */

/* DMA RX ring size. */
#define MAX_M10S_DMA_BUF_LEN    512u

/* UART speed used after configuration; the module boots at 38400. */
#define MAX_M10S_BAUD_FLIGHT    115200u

/* Per-message ACK timeout. */
#define MAX_M10S_CFG_TIMEOUT_MS 200u

/* Retry budget per key before failing configure(). */
#define MAX_M10S_CFG_RETRIES    3u

/* ------------------------------------------------------------------ DMA buffer */

static uint8_t s_dma_buf[MAX_M10S_DMA_BUF_LEN]
    __attribute__((section(".dma_buf"), aligned(32)));


/* -------------------------------------------------------------- CFG-VALSET ACK
 *
 * Sends one CFG-VALSET and waits for ACK-ACK or ACK-NAK. */
static WARN_UNUSED max_m10s_status_t send_cfg_valset(uint32_t key, const uint8_t *val,
                                                 uint8_t val_len)
{
    uint8_t payload[MAX_M10S_CFG_PAYLOAD_CAP];
    uint8_t frame[MAX_M10S_CFG_FRAME_CAP];

    payload[0] = 0x00u;                          /* version */
    payload[1] = MAX_M10S_CFG_LAYER_RAM;              /* layers */
    payload[2] = 0x00u;                           /* reserved */
    payload[3] = 0x00u;                           /* reserved */
    payload[4] = (uint8_t)(key & 0xFFu);
    payload[5] = (uint8_t)((key >> 8)  & 0xFFu);
    payload[6] = (uint8_t)((key >> 16) & 0xFFu);
    payload[7] = (uint8_t)((key >> 24) & 0xFFu);
    memcpy(&payload[MAX_M10S_CFG_HDR_LEN], val, val_len);

    const uint16_t payload_len = (uint16_t)(MAX_M10S_CFG_HDR_LEN + val_len);
    const uint16_t frame_len   = ubx_frame_build(frame, sizeof(frame),
                                                 UBX_CLASS_CFG, UBX_ID_CFG_VALSET,
                                                 payload, payload_len);
    if (frame_len == 0u)
        return MAX_M10S_ERR_HAL;

    for (uint8_t attempt = 0; attempt < MAX_M10S_CFG_RETRIES; attempt++)
    {
        if (HAL_UART_Transmit(&MAX_M10S_HUART_HANDLE, frame, frame_len, 50u) != HAL_OK)
            return MAX_M10S_ERR_HAL;

        /* Snapshot write pointer; only newer bytes can belong to this ACK. */
        ubx_parser_t ack_parser;
        ubx_parser_init(&ack_parser);
        uint16_t read_pos = (uint16_t)(MAX_M10S_DMA_BUF_LEN -
                                       (uint16_t)__HAL_DMA_GET_COUNTER(MAX_M10S_HUART_HANDLE.hdmarx));

        const uint32_t start = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start) < MAX_M10S_CFG_TIMEOUT_MS)
        {
            const uint16_t ndtr      = (uint16_t)__HAL_DMA_GET_COUNTER(MAX_M10S_HUART_HANDLE.hdmarx);
            const uint16_t write_pos = (uint16_t)(MAX_M10S_DMA_BUF_LEN - ndtr);

            while (read_pos != write_pos)
            {
                const uint8_t b = s_dma_buf[read_pos];
                read_pos = (uint16_t)((read_pos + 1u) % MAX_M10S_DMA_BUF_LEN);

                if (!ubx_parse_byte(&ack_parser, b))
                    continue;
                if (ack_parser.msg_class != UBX_CLASS_ACK)
                    continue;

                uint8_t acked_class, acked_id;
                bool    acknowledged;
                if (!ubx_decode_ack(ack_parser.msg_id, ack_parser.payload,
                                    ack_parser.len, &acked_class, &acked_id,
                                    &acknowledged))
                    continue;
                if (acked_class != UBX_CLASS_CFG || acked_id != UBX_ID_CFG_VALSET)
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

void max_m10s_init(max_m10s_driver_t *drv)
{
    memset(drv, 0, sizeof(*drv));
    ubx_parser_init(&drv->parser);

    HAL_UARTEx_ReceiveToIdle_DMA(&MAX_M10S_HUART_HANDLE, s_dma_buf, MAX_M10S_DMA_BUF_LEN);
    /* Idle interrupt is the drain trigger; HT interrupt is not used. */
    __HAL_DMA_DISABLE_IT(MAX_M10S_HUART_HANDLE.hdmarx, DMA_IT_HT);
}

/* -------------------------------------------------------------------- configure */

max_m10s_status_t max_m10s_configure(max_m10s_driver_t *drv)
{
    uint8_t val[MAX_M10S_CFG_VAL_MAX_LEN];
    max_m10s_status_t st;

    // Step 1: set UART1 baud to 115200 at the boot baud.
    val[0] = (uint8_t)(MAX_M10S_BAUD_FLIGHT & 0xFFu);
    val[1] = (uint8_t)((MAX_M10S_BAUD_FLIGHT >> 8)  & 0xFFu);
    val[2] = (uint8_t)((MAX_M10S_BAUD_FLIGHT >> 16) & 0xFFu);
    val[3] = (uint8_t)((MAX_M10S_BAUD_FLIGHT >> 24) & 0xFFu);
    st = send_cfg_valset(0x40520001u, val, 4u);
    if (st != MAX_M10S_OK)
        return st;

    /* Switch the STM32 UART to 115200 before sending step 2. Re-initialise
     * and restart DMA, the peripheral resets on baud change. */
    MAX_M10S_HUART_HANDLE.Init.BaudRate = MAX_M10S_BAUD_FLIGHT;
    if (HAL_UART_Init(&MAX_M10S_HUART_HANDLE) != HAL_OK)
        return MAX_M10S_ERR_HAL;

    HAL_UARTEx_ReceiveToIdle_DMA(&MAX_M10S_HUART_HANDLE, s_dma_buf, MAX_M10S_DMA_BUF_LEN);
    __HAL_DMA_DISABLE_IT(MAX_M10S_HUART_HANDLE.hdmarx, DMA_IT_HT);

    // Step 2: CFG-NAVSPG-DYNMODEL = 8 (airborne <4g).
    val[0] = 8u;
    st = send_cfg_valset(0x20110021u, val, 1u);
    if (st != MAX_M10S_OK)
        return st;

    // Step 3: CFG-RATE-MEAS = 50 ms measurement period. 
    val[0] = (uint8_t)(50u & 0xFFu);
    val[1] = (uint8_t)((50u >> 8) & 0xFFu);
    st = send_cfg_valset(0x30210001u, val, 2u);
    if (st != MAX_M10S_OK)
        return st;

    // Step 4: CFG-RATE-NAV = 1 solution per measurement.
    val[0] = (uint8_t)(1u & 0xFFu);
    val[1] = (uint8_t)((1u >> 8) & 0xFFu);
    st = send_cfg_valset(0x30210002u, val, 2u);
    if (st != MAX_M10S_OK)
        return st;

    //  Step 5: CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1. 
    val[0] = 1u;
    st = send_cfg_valset(0x20910007u, val, 1u);
    if (st != MAX_M10S_OK)
        return st;

    // Step 6: CFG-UART1OUTPROT-NMEA = 0 (disable NMEA).
    val[0] = 0u;
    st = send_cfg_valset(0x10740002u, val, 1u);
    if (st != MAX_M10S_OK)
        return st;

    /* Reset parser read pointer. Data already in flight may be skipped. */
    drv->read_pos = (uint16_t)(MAX_M10S_DMA_BUF_LEN -
                               (uint16_t)__HAL_DMA_GET_COUNTER(MAX_M10S_HUART_HANDLE.hdmarx));

    return MAX_M10S_OK;
}

/* ----------------------------------------------------------------------- drain */

bool max_m10s_drain(max_m10s_driver_t *drv, max_m10s_frame_t *frame)
{
    /* Invalidate cache before reading DMA memory. */
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_dma_buf, MAX_M10S_DMA_BUF_LEN);

    /* 1. Compute write position from NDTR. */
    const uint16_t ndtr      = (uint16_t)__HAL_DMA_GET_COUNTER(MAX_M10S_HUART_HANDLE.hdmarx);
    const uint16_t write_pos = (uint16_t)(MAX_M10S_DMA_BUF_LEN - ndtr);

    /* 2. If nothing new, check for UART overrun and return false. */
    if (write_pos == drv->read_pos)
    {
        if (__HAL_UART_GET_FLAG(&MAX_M10S_HUART_HANDLE, UART_FLAG_ORE))
        {
            __HAL_UART_CLEAR_OREFLAG(&MAX_M10S_HUART_HANDLE);
            drv->uart_overruns++;
        }
        return false;
    }

    /* 3. Feed bytes to the parser, stopping on the first complete frame. */
    while (drv->read_pos != write_pos)
    {
        const uint8_t b = s_dma_buf[drv->read_pos];
        drv->read_pos = (uint16_t)((drv->read_pos + 1u) % MAX_M10S_DMA_BUF_LEN);

        if (ubx_parse_byte(&drv->parser, b))
        {
            frame->msg_class = drv->parser.msg_class;
            frame->msg_id    = drv->parser.msg_id;
            frame->len       = drv->parser.len;
            frame->payload   = drv->parser.payload;
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------- stats */

void max_m10s_get_stats(const max_m10s_driver_t *drv, max_m10s_stats_t *out)
{
    out->frames_ok       = drv->parser.frames_ok;
    out->checksum_errors = drv->parser.checksum_errors;
    out->resyncs         = drv->parser.resyncs;
    out->oversized       = drv->parser.oversized;
    out->uart_overruns   = drv->uart_overruns;
}
