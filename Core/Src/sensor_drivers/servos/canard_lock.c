#include "servos/canard_lock.h"

static TIM_HandleTypeDef *s_htim;

bool canard_lock_init(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) return false;

    s_htim = htim;
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, 0U);
    return HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1) == HAL_OK;
}

void canard_lock_set_us(uint16_t pulse_us)
{
    if (s_htim == NULL) return;

    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, pulse_us);
}
