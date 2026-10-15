// usb_print.h
#pragma once
#include "cmsis_os2.h"
#include <stdint.h>
#include <stdio.h>

extern osMutexId_t usb_print_mutex;

/* 1 = USB_PRINTF emits; 0 = skip (launch / flight - cheap branch, no I/O). */
extern volatile uint8_t print_flag;

void usb_print_init(void);

/* Enqueue bytes for USB CDC; never blocks, drops when the ring is full or no
 * host is enumerated. */
void usb_print_write(const uint8_t *data, uint16_t len);

/* Continue draining the ring; called from the CDC transmit-complete ISR. */
void usb_print_tx_complete(void);

#define USB_PRINTF(fmt, ...) do { \
    if (print_flag) { \
        osMutexAcquire(usb_print_mutex, osWaitForever); \
        printf(fmt, ##__VA_ARGS__); \
        osMutexRelease(usb_print_mutex); \
    } \
} while(0)
