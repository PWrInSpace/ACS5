#include "utils/timestamp.h"

counter64_t g_cycle_counter;

void timestamp_init(void)
{
    /* Every timeout, conversion delay and sample dt in the system is denominated
     * in CPU_HZ. A regenerated clock tree would skew all of them silently. */
    if (SystemCoreClock != CPU_HZ) Error_Handler();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR    = 0xC5ACCE55;          /* unlock DWT on Cortex-M7 */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    g_cycle_counter = (counter64_t){0};
}
