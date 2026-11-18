/*
 * Canard lock servo on J4 CH6 (PA5, TIM2_CH1). Pulse values in
 * servos/servo_bench.h (CANARD_LOCK_US_LOCKED / _UNLOCKED).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "utils/attributes.h"
#include "stm32h7xx_hal.h"

/* Starts the PWM channel with no pulse; the servo stays limp until set_us(). */
WARN_UNUSED bool canard_lock_init(TIM_HandleTypeDef *htim);

void     canard_lock_set_us(uint16_t pulse_us);
uint16_t canard_lock_current_us(void);
