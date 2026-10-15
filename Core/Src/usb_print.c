#include "usb_print.h"

#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include <stdbool.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

osMutexId_t usb_print_mutex;
volatile uint8_t print_flag = 1u;

void usb_print_init(void) {
    /* Recursive: USB_PRINTF may be reached from a printf-ing helper that the
     * caller already holds the mutex for. PrioInherit: the control-loop
     * prints, so a low-priority holder must not block it unbounded. */
    const osMutexAttr_t attributes = {
        "usbPrintMutex",
        osMutexRecursive | osMutexPrioInherit,
        NULL,
        0
    };

    usb_print_mutex = osMutexNew(&attributes);
    print_flag = 1u;
}

/* Producer is any printf-ing task via _write; consumer is the USB transmit-
 * complete interrupt. Indices move with interrupts off, so the interleaving is
 * safe whatever priority the OTG IRQ runs at. A full ring drops the newest
 * bytes: the 500 Hz control loop must never wait on debug output. */
#define USB_TX_RING_SIZE  2048u
#define USB_TX_CHUNK_SIZE 256u

static uint8_t           ring[USB_TX_RING_SIZE];
static volatile uint16_t ring_head;
static volatile uint16_t ring_tail;
static volatile bool     tx_active;
static uint8_t           chunk[USB_TX_CHUNK_SIZE];

static bool usb_ready(void)
{
    return hUsbDeviceHS.pClassData != NULL &&
           hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED;
}

/* Caller must have interrupts disabled. */
static void kick_locked(void)
{
    if (tx_active || ring_head == ring_tail || !usb_ready())
        return;

    uint16_t n = 0;
    uint16_t t = ring_tail;
    while (t != ring_head && n < USB_TX_CHUNK_SIZE)
    {
        chunk[n++] = ring[t];
        t = (uint16_t)((t + 1u) % USB_TX_RING_SIZE);
    }

    if (CDC_Transmit_HS(chunk, n) == USBD_OK)
    {
        ring_tail = t;
        tx_active = true;
    }
    /* BUSY or FAIL: the bytes stay queued for the next write or completion. */
}

void usb_print_write(const uint8_t *data, uint16_t len)
{
    if (!print_flag || len == 0)
        return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* A transfer in flight when the host disappears never completes. */
    if (tx_active && !usb_ready())
        tx_active = false;

    for (uint16_t i = 0; i < len; ++i)
    {
        uint16_t next = (uint16_t)((ring_head + 1u) % USB_TX_RING_SIZE);
        if (next == ring_tail)
            break;
        ring[ring_head] = data[i];
        ring_head = next;
    }
    kick_locked();

    __set_PRIMASK(primask);
}

void usb_print_tx_complete(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tx_active = false;
    kick_locked();
    __set_PRIMASK(primask);
}
