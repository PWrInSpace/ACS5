#include "tasks/sd_log_task.h"

#include "main.h"
#include "fatfs.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "cmsis_os2.h"
#include "utils/attributes.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define LOG_QUEUE_RECORDS   64u      /* ~0.5 s at the flight log rate */
#define LOG_PREALLOC_BYTES  (32u * 1024u * 1024u)
#define LOG_SECTOR          512u
#define LOG_WBUF_BYTES      1024u
#define LOG_SYNC_MS         400u
#define LOG_IDLE_FLUSH_MS   1000u

/* Single writer (flight task), single reader (this task): what a message buffer requires. */
_Static_assert(LOG_WBUF_BYTES >= LOG_SECTOR + sizeof(log_record_t), "write buffer too small");

static StaticMessageBuffer_t s_queue_ctrl;
static uint8_t s_queue_storage[LOG_QUEUE_RECORDS * (sizeof(log_record_t) + sizeof(size_t)) + 1u];
static MessageBufferHandle_t volatile s_queue;

static uint32_t          s_seq;
static volatile uint32_t s_dropped;
static volatile uint32_t s_errors;

/* SDMMC1 IDMA cannot reach DTCM, so FatFs windows and the write buffer live in AXI SRAM. */
static FATFS   s_fs   ATTR_SECTION(".dma_buf") ATTR_ALIGNED(32);
static FIL     s_file ATTR_SECTION(".dma_buf") ATTR_ALIGNED(32);
static uint8_t s_wbuf[LOG_WBUF_BYTES] ATTR_SECTION(".dma_buf") ATTR_ALIGNED(32);

void log_publish(log_record_t *r)
{
    r->log_dropped = s_dropped;
    r->log_errors  = s_errors;
    log_record_seal(r, s_seq++);

    MessageBufferHandle_t q = s_queue;
    if (q == NULL || xMessageBufferSend(q, r, sizeof *r, 0) != sizeof *r)
        s_dropped++;
}

void log_stats(uint32_t *dropped, uint32_t *errors)
{
    *dropped = s_dropped;
    *errors  = s_errors;
}

static void write_failed(void)
{
    s_errors++;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

/* Write len bytes from the front of s_wbuf and keep the rest. */
static size_t write_front(size_t fill, size_t len)
{
    UINT bw = 0;
    if (f_write(&s_file, s_wbuf, (UINT)len, &bw) != FR_OK || bw != len)
        write_failed();
    memmove(s_wbuf, s_wbuf + len, fill - len);
    return fill - len;
}

/* Blink LD1 code times, forever: the log cannot start. */
static void halt_with_code(unsigned code)
{
    for (;;)
    {
        for (unsigned i = 0; i < code; i++)
        {
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
            osDelay(200);
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
            osDelay(200);
        }
        osDelay(2000);
    }
}

/* SD card logging task. LED codes: docs/design.md#sd-logging-led-codes */
void StartSDTask(void *argument)
{
    (void)argument;
    s_queue = xMessageBufferCreateStatic(sizeof s_queue_storage, s_queue_storage, &s_queue_ctrl);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    osDelay(2000);

    FRESULT res = f_mount(&s_fs, SDPath, 1);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    if (res != FR_OK)
        halt_with_code((unsigned)res);

    char filename[20];
    uint16_t log_num = 1;
    for (; log_num <= 999; log_num++)
    {
        snprintf(filename, sizeof filename, "LOG_%03u.BIN", log_num);
        FILINFO fno;
        if (f_stat(filename, &fno) != FR_OK)
            break;
    }
    if (log_num > 999)
        snprintf(filename, sizeof filename, "LOG_999.BIN");

    if (f_open(&s_file, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        halt_with_code(20u);
    /* Contiguous clusters up front: no FAT updates mid-flight. The decoder stops at
     * the never-written tail. Without enough contiguous space, log unallocated. */
    (void)f_expand(&s_file, LOG_PREALLOC_BYTES, 1);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

    size_t   fill      = 0;
    uint32_t last_sync = osKernelGetTickCount();
    bool     dirty     = false;

    for (;;)
    {
        log_record_t r;
        size_t n = xMessageBufferReceive(s_queue, &r, sizeof r, pdMS_TO_TICKS(LOG_IDLE_FLUSH_MS));
        if (n == sizeof r)
        {
            memcpy(s_wbuf + fill, &r, sizeof r);
            fill += sizeof r;
        }

        /* Whole sectors while records flow; everything once they stop (end of mission). */
        if (fill >= LOG_SECTOR)
        {
            fill  = write_front(fill, fill - fill % LOG_SECTOR);
            dirty = true;
        }
        else if (n == 0 && fill > 0)
        {
            fill  = write_front(fill, fill);
            dirty = true;
        }

        const uint32_t now = osKernelGetTickCount();
        if (dirty && (now - last_sync >= LOG_SYNC_MS || n == 0))
        {
            if (f_sync(&s_file) != FR_OK)
                write_failed();
            last_sync = now;
            dirty     = false;
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        }
    }
}
