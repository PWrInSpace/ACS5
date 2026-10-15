/*
 * SRT Servo T75 HV bank driver: 4 fins on one TIM, driven as PWM channels.
 * Per-fin trim/limits/direction live in the config; angle->pulse mapping and
 * slew limiting come from servo_t75_math. Pins stay dark until arm().
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "utils/attributes.h"
#include "servos/servo_t75_math.h"
#include "stm32h7xx_hal.h" // TIM_HandleTypeDef

#define SERVO_T75_FIN_COUNT 4

typedef struct
{
    uint16_t neutral_us[SERVO_T75_FIN_COUNT];      // per-servo neutral
    uint16_t min_us[SERVO_T75_FIN_COUNT];
    uint16_t max_us[SERVO_T75_FIN_COUNT];
    int8_t   direction_sign[SERVO_T75_FIN_COUNT];  // +1 or -1
    float    us_per_deg;
    float    max_angle_deg;
    float    slew_us_per_ms;
} servo_t75_config_t;

servo_t75_config_t servo_t75_config_rocket_default(void);

typedef struct
{
    TIM_HandleTypeDef *htim;
    servo_t75_config_t cfg;
    bool               initialized;
    bool               armed;
    uint16_t           target_us[SERVO_T75_FIN_COUNT];
    uint16_t           current_us[SERVO_T75_FIN_COUNT];
} servo_bank_t75_t;

/* htim is your CubeMX-generated &htim4 */
WARN_UNUSED bool servo_bank_t75_init(servo_bank_t75_t *self, TIM_HandleTypeDef *htim,
                                     const servo_t75_config_t *cfg);

void servo_bank_t75_arm(servo_bank_t75_t *self);
void servo_bank_t75_set_angle_deg(servo_bank_t75_t *self, uint8_t fin_idx, float deg);
void servo_bank_t75_tick(servo_bank_t75_t *self, uint32_t dt_ms);

uint16_t servo_bank_t75_current_pulse_us(const servo_bank_t75_t *self, uint8_t i);
