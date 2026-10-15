/**
 * @file    buzzer.c
 * @brief   PWM buzzer driver - CMT-8504-100-SMT-TR
 *
 * Circuit recap (from schematic):
 *   PE5 (TIM15_CH1) --[R48 1k]--> IRLML6344 gate
 *                                  IRLML6344 drain --> Buzzer(-)
 *                                  Buzzer(+) --> 3V3
 *                                  D20 flyback across buzzer
 *                   --[R49 33k]--> GND  (ensures gate low when pin floats)
 *
 * Timer maths:
 *   APB2 Timer clock = 150 MHz
 *   ARR  = (150,000,000 / freq_hz) - 1
 *   CCR1 = ARR / 2          -> 50 % duty cycle (max magnetic drive)
 *
 * Inter-note gap:
 *   A 10 ms silent gap is inserted between every note during melody playback.
 *   This articulates notes that share the same pitch (e.g. two consecutive D4s)
 *   and prevents the buzzer from ringing continuously through ties.
 */

#include "buzzer/buzzer.h"
#include "tim.h"            /* CubeMX-generated TIM15 handle (htim15) */
#include "cmsis_os2.h"      /* osDelay */

/* -- Private state ----------------------------------------------------------- */

static TIM_HandleTypeDef *_htim    = NULL;
static uint8_t            _running = 0;

/* -- Private helpers --------------------------------------------------------- */

static void _apply_freq(uint32_t freq_hz)
{
    if (freq_hz == 0) freq_hz = 1;

    uint32_t psc  = 0;
    uint32_t arr  = (BUZZER_TIMER_CLOCK_HZ / freq_hz) - 1UL;

    /* If ARR overflows 16-bit, introduce prescaler.
     * Find smallest PSC such that arr fits in uint16.                */
    while (arr > 65535UL) {
        psc++;
        arr = (BUZZER_TIMER_CLOCK_HZ / ((psc + 1UL) * freq_hz)) - 1UL;
    }

    uint32_t ccr1 = arr / 2UL;

    __HAL_TIM_SET_PRESCALER(_htim, psc);
    __HAL_TIM_SET_AUTORELOAD(_htim, arr);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, ccr1);
    _htim->Instance->EGR = TIM_EGR_UG;
}

/* -- Public API -------------------------------------------------------------- */

void Buzzer_Init(TIM_HandleTypeDef *htim)
{
    _htim    = htim;
    _running = 0;
    _apply_freq(BUZZER_RESONANT_FREQ_HZ);
}

void Buzzer_On(void)
{
    if (_htim == NULL || _running) return;
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1);
    _running = 1;
}

void Buzzer_Off(void)
{
    if (_htim == NULL || !_running) return;
    HAL_TIM_PWM_Stop(_htim, TIM_CHANNEL_1);
    _running = 0;
}

void Buzzer_SetFreq(uint32_t freq_hz)
{
    if (_htim == NULL) return;
    _apply_freq(freq_hz);
}

void Buzzer_Beep(uint32_t freq_hz, uint32_t duration_ms)
{
    if (_htim == NULL) return;
    _apply_freq(freq_hz);
    Buzzer_On();
    HAL_Delay(duration_ms);
    Buzzer_Off();
}

void Buzzer_BeepAsync(uint32_t freq_hz)
{
    if (_htim == NULL) return;
    _apply_freq(freq_hz);
    Buzzer_On();
}

/* -- Melody playback --------------------------------------------------------- */

#define NOTE_GAP_MS   10u   /* silent gap between notes for articulation */

void Buzzer_PlayMelody(const BuzzerNote_t *melody, uint16_t len)
{
    if (_htim == NULL || melody == NULL || len == 0) return;

    for (uint16_t i = 0; i < len; i++)
    {
        uint32_t freq = melody[i].freq_hz;
        uint32_t dur  = melody[i].duration_ms;

        /* Clamp duration so the gap always fits */
        uint32_t tone_dur = (dur > NOTE_GAP_MS) ? (dur - NOTE_GAP_MS) : dur;

        if (freq == 0)
        {
            /* REST: stay silent for the full note duration */
            Buzzer_Off();
            osDelay(dur);
        }
        else
        {
            _apply_freq(freq);
            Buzzer_On();
            osDelay(tone_dur);
            Buzzer_Off();
            osDelay(NOTE_GAP_MS);
        }
    }
}
