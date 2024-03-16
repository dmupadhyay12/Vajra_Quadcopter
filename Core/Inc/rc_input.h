#ifndef RC_INPUT
#define RC_INPUT

#include "stdint.h"
#include "tim.h"

typedef struct input_channel {
    uint8_t min_pulse_width;
    uint8_t max_pulse_width;
    uint16_t current_pulse_width;
} input_channel_t;

uint8_t get_channel_percentage(input_channel_t *channel);

void update_pulse_width(input_channel_t *channel, uint16_t rising_edge_capture, uint16_t falling_edge_capture);

#endif