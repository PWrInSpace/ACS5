/* PWM driver for the CMT-8504 buzzer on PE5 / TIM15 CH1 (low-side IRLML6344). */

#ifndef BUZZER_H
#define BUZZER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* -- Public constants -------------------------------------------------------- */

#define BUZZER_RESONANT_FREQ_HZ     2400U          /* CMT-8504 typical resonance */
#define BUZZER_TIMER_CLOCK_HZ       150000000UL    /* must match the CubeMX APB2 timer clock */

/* -- Note struct for melody playback ----------------------------------------- */
typedef struct {
    uint16_t freq_hz;       /* 0 = REST (silence) */
    uint16_t duration_ms;
} BuzzerNote_t;

/* Bind the TIM15 handle; call once after MX_TIM15_Init(), before any other buzzer call. */
void Buzzer_Init(TIM_HandleTypeDef *htim);

/* Start the tone (idempotent). */
void Buzzer_On(void);

/* Stop the tone (idempotent). */
void Buzzer_Off(void);

/* Play len notes, blocking via osDelay; task context only. */
void Buzzer_PlayMelody(const BuzzerNote_t *melody, uint16_t len);

#endif /* BUZZER_H */