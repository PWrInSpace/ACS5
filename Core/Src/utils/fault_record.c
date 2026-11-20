#include "utils/fault_record.h"

#include "main.h"
#include "utils/attributes.h"

#define FAULT_RECORD_MAGIC 0xFA17FA17u

/* .noinit survives a system reset (not a power cycle); the magic tells a record from garbage. */
static volatile fault_record_t s_record ATTR_SECTION(".noinit");

static void store_and_reset(uint32_t kind) __attribute__((noreturn));
static void store_and_reset(uint32_t kind)
{
    s_record.kind  = kind;
    s_record.cfsr  = SCB->CFSR;
    s_record.hfsr  = SCB->HFSR;
    s_record.bfar  = SCB->BFAR;
    s_record.mmfar = SCB->MMFAR;
    s_record.magic = FAULT_RECORD_MAGIC;
    __DSB();
    NVIC_SystemReset();
}

/* Reached only from the HardFault_Handler asm branch, which LTO cannot see. */
__attribute__((used, externally_visible))
void fault_record_hardfault(const uint32_t *frame)
{
    s_record.lr   = frame[5];
    s_record.pc   = frame[6];
    s_record.file = NULL;
    s_record.line = 0u;
    store_and_reset(FAULT_RECORD_HARDFAULT);
}

void fault_record_error_handler(uint32_t caller)
{
    s_record.pc   = 0u;
    s_record.lr   = caller;
    s_record.file = NULL;
    s_record.line = 0u;
    store_and_reset(FAULT_RECORD_ERROR_HANDLER);
}

void fault_record_assert(const char *file, int line)
{
    __disable_irq();
    s_record.file = file;
    s_record.line = (uint32_t)line;
    s_record.pc   = 0u;
    s_record.lr   = (uint32_t)__builtin_return_address(0);
    store_and_reset(FAULT_RECORD_ASSERT);
}

bool fault_record_take(fault_record_t *out)
{
    if (s_record.magic != FAULT_RECORD_MAGIC)
        return false;
    *out = *(const fault_record_t *)&s_record;
    s_record.magic = 0u;
    /* The record is read after a reset; a file pointer outside flash is not to be trusted. */
    if ((uint32_t)out->file < FLASH_BANK1_BASE || (uint32_t)out->file >= FLASH_END)
        out->file = NULL;
    return true;
}
