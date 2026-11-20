#include "sensor_drivers/servos/servo_t75.h"
#include "tim.h"    /* CubeMX-generated, declares htim3 */

/* Fin n -> Meteor servo header J4 CH(n+2); J4 CH1 has no MCU pin, CH6 is the canard lock. */
static const uint32_t kFinToHalChannel[SERVO_T75_FIN_COUNT] = {
    TIM_CHANNEL_4,   /* fin 0 - PB1 */
    TIM_CHANNEL_3,   /* fin 1 - PB0 */
    TIM_CHANNEL_2,   /* fin 2 - PA7 */
    TIM_CHANNEL_1,   /* fin 3 - PA6 */
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
        .max_us         = {1700, 1700, 1720, 1720},  // neutral + 200 (+220 on fins 2-3)
        .direction_sign = {+1, +1, +1, +1},
        .us_per_deg     = 11.111f,
        .max_angle_deg  = 15.0f,
        /* Datasheet no-load 375 deg/s at 6 V; achieves 4.00 us/ms at a 2 ms tick.
         * Why: docs/design.md#servo-slew-limit */
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

    /* TIM3 is configured by MX_TIM3_Init(); channels start with no pulse until arm(). */
    bool ok = true;
    for (uint8_t i = 0; i < SERVO_T75_FIN_COUNT; ++i)
    {
        ok &= HAL_TIM_PWM_Start(self->htim, kFinToHalChannel[i]) == HAL_OK;
        set_pwm_pulse(self->htim, kFinToHalChannel[i], 0U);  /* no pulse until armed */
    }

    return ok;
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
