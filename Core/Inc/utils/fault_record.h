/* Last fatal error, kept across the reset it triggers and reported at the next boot. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    FAULT_RECORD_HARDFAULT     = 1,
    FAULT_RECORD_ERROR_HANDLER = 2,
    FAULT_RECORD_ASSERT        = 3,
} fault_record_kind_t;

typedef struct
{
    uint32_t magic;
    uint32_t kind;          /* fault_record_kind_t */
    uint32_t pc;            /* HardFault: stacked PC */
    uint32_t lr;            /* HardFault: stacked LR; otherwise the caller */
    uint32_t cfsr, hfsr, bfar, mmfar;
    const char *file;       /* assert: source file (a flash string, valid after reset) */
    uint32_t line;          /* assert: source line */
} fault_record_t;

/* Record a HardFault from its exception stack frame, then reset. */
void fault_record_hardfault(const uint32_t *frame) __attribute__((noreturn));

/* Record a call to Error_Handler from caller, then reset. */
void fault_record_error_handler(uint32_t caller) __attribute__((noreturn));

/* Record a failed configASSERT, then reset. */
void fault_record_assert(const char *file, int line) __attribute__((noreturn));

/* Copy out and clear the record left by the previous run; false if there is none. */
bool fault_record_take(fault_record_t *out);
