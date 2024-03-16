#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "stdint.h"
#include "tim.h"
#include "stdbool.h"

typedef struct actuator_config {
    TIM_HandleTypeDef *timer;
    uint8_t channel;
    uint8_t min_pulse_width;
    uint8_t max_pulse_width;
} actuator_config_t;

void pwm_init(actuator_config_t *pwm);
void pwm_update_percentage(actuator_config_t* pwm, uint8_t percent);

#endif
