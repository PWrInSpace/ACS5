#include "servos/canard_lock.h"

static TIM_HandleTypeDef *s_htim;
static uint16_t           s_pulse_us;

bool canard_lock_init(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) return false;

    s_htim     = htim;
    s_pulse_us = 0;
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, 0U);
    return HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1) == HAL_OK;
}

void canard_lock_set_us(uint16_t pulse_us)
{
    if (s_htim == NULL) return;

    s_pulse_us = pulse_us;
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, pulse_us);
}

uint16_t canard_lock_current_us(void)
{
    return s_pulse_us;
}
