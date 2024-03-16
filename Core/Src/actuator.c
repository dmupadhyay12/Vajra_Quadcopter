#include "actuator.h"
#include "tim.h"
#include "stdint.h"

#define MAX_PULSE_TICKS 6400
#define MIN_PULSE_TICKS 3200

void pwm_init(actuator_config_t *pwm) {
    HAL_TIM_PWM_Start(pwm -> timer, pwm -> channel);
}

void pwm_update_percentage(actuator_config_t *pwm, uint8_t pwm_update_percentage) {
    // map percentage between 1 and 2 ms, which amounts to a range between 
    // 3200 and 6400 ticks, based on a 3.2 MHz clock (16 Mhz with prescaler of 5)
    uint16_t ccr_value = (uint16_t)(MAX_PULSE_TICKS - MIN_PULSE_TICKS) * (pwm_update_percentage / 100.0);
    __HAL_TIM_SET_COMPARE(pwm -> timer, pwm -> channel, (uint32_t) ccr_value);
}