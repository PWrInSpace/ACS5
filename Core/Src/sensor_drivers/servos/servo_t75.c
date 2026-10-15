#include "servos/servo_t75.h"
#include "tim.h"    /* CubeMX-generated, declares htim4 */

/* -- Fin -> TIM channel mapping ----------------------------------------------
 * Matches PCB silkscreen CH1..CH4_PWM on PD12..PD15:
 *   fin 0 -> PD15 -> TIM4_CH4
 *   fin 1 -> PD14 -> TIM4_CH3
 *   fin 2 -> PD13 -> TIM4_CH2
 *   fin 3 -> PD12 -> TIM4_CH1
 * --------------------------------------------------------------------------- */
static const uint32_t kFinToHalChannel[SERVO_T75_FIN_COUNT] = {
    TIM_CHANNEL_4,   /* fin 0 - PD15 */
    TIM_CHANNEL_3,   /* fin 1 - PD14 */
    TIM_CHANNEL_2,   /* fin 2 - PD13 */
    TIM_CHANNEL_1,   /* fin 3 - PD12 */
};

static void set_pwm_pulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(htim, channel, pulse_us);
}

static servo_t75_limits_t limits_for(const servo_bank_t75_t *self, uint8_t fin_idx)
{
    servo_t75_limits_t lim = {
        .neutral_us     = self->cfg.neutral_us[fin_idx],
        .min_us         = self->cfg.min_us[fin_idx],
        .max_us         = self->cfg.max_us[fin_idx],
        .direction_sign = self->cfg.direction_sign[fin_idx],
        .us_per_deg     = self->cfg.us_per_deg,
    };
    return lim;
}

servo_t75_config_t servo_t75_config_rocket_default(void)
{
    servo_t75_config_t cfg = {
        .neutral_us     = {1500, 1500, 1500, 1500},  // per-servo neutral
        .min_us         = {1300, 1300, 1300, 1300},  // neutral - 200
        .max_us         = {1700, 1700, 1720, 1720},  // neutral + 200
        .direction_sign = {+1, +1, +1, +1},
        .us_per_deg     = 11.111f,
        .max_angle_deg  = 15.0f,
        /* 0.16 s/60 deg at 6 V, the lowest rated supply, = 375 deg/s; at
         * 11.111 us/deg that is 4.17 us/ms. Two caveats before trusting it as
         * a rate: the datasheet figure is NO-LOAD, so under hinge moment the
         * horn is slower and this limiter stops binding at all; and
         * servo_t75_slew_step rounds into uint16_t every call, so at a 2 ms
         * tick the 8.34 us step truncates to 8 and the achieved rate is
         * 4.00 us/ms (360 deg/s), not 4.17. */
        .slew_us_per_ms = 4.17f,
    };
    return cfg;
}

bool servo_bank_t75_init(servo_bank_t75_t *self, TIM_HandleTypeDef *htim,
                         const servo_t75_config_t *cfg)
{
    if (htim == NULL) return false;

    self->htim        = htim;
    self->cfg         = *cfg;
    self->initialized = true;
    self->armed       = false;

    for (uint8_t i = 0; i < SERVO_T75_FIN_COUNT; ++i)
    {
        self->target_us[i]  = self->cfg.neutral_us[i];
        self->current_us[i] = self->cfg.neutral_us[i];
    }

    /* TIM4 must already be started by CubeMX MX_TIM4_Init().
     * We just start the PWM channels here - pins stay at 0 until arm(). */
    for (uint8_t i = 0; i < SERVO_T75_FIN_COUNT; ++i)
    {
        HAL_TIM_PWM_Start(self->htim, kFinToHalChannel[i]);
        set_pwm_pulse(self->htim, kFinToHalChannel[i], 0U);  /* no pulse until armed */
    }

    return true;
}

void servo_bank_t75_arm(servo_bank_t75_t *self)
{
    if (!self->initialized) return;
    self->armed = true;

    for (uint8_t i = 0; i < SERVO_T75_FIN_COUNT; ++i)
    {
        self->target_us[i]  = self->cfg.neutral_us[i];
        self->current_us[i] = self->cfg.neutral_us[i];
        set_pwm_pulse(self->htim, kFinToHalChannel[i], self->current_us[i]);
    }
}

void servo_bank_t75_set_angle_deg(servo_bank_t75_t *self, uint8_t fin_idx, float deg)
{
    if (!self->initialized || fin_idx >= SERVO_T75_FIN_COUNT) return;

    const servo_t75_limits_t lim = limits_for(self, fin_idx);
    self->target_us[fin_idx] =
        servo_t75_angle_to_pulse_us(deg, &lim, self->cfg.max_angle_deg);
}

void servo_bank_t75_tick(servo_bank_t75_t *self, uint32_t dt_ms)
{
    if (!self->initialized) return;

    for (uint8_t i = 0; i < SERVO_T75_FIN_COUNT; ++i)
    {
        self->current_us[i] = servo_t75_slew_step(
            self->current_us[i], self->target_us[i], self->cfg.slew_us_per_ms, dt_ms);

        if (self->armed)
        {
            set_pwm_pulse(self->htim, kFinToHalChannel[i], self->current_us[i]);
        }
    }
}

uint16_t servo_bank_t75_current_pulse_us(const servo_bank_t75_t *self, uint8_t i)
{
    return (i < SERVO_T75_FIN_COUNT) ? self->current_us[i] : 0;
}
