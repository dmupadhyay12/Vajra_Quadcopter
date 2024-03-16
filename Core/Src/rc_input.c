#include "tim.h"
#include "rc_input.h"

uint8_t get_channel_percentage(input_channel_t *channel) {
    // convert the current pulse width into a percentage from 0-100%,
    // based on the tick frequency

    // 16 MHz is passed through prescaler of 5, creating net frequency of 3.2 MHz,
    // meaning that 1 ms is 3200 ticks, and 2 ms is 6400 ticks
    uint8_t percentage = 0;
    // if (channel -> current_pulse_width > channel -> max_pulse_width) {
    //     percentage = 100;
    // } else  if (channel -> current_pulse_width < channel -> min_pulse_width) {
    //     percentage = 0;
    // } else {
    //     percentage = (uint8_t)(((float)(channel -> current_pulse_width) * 0.75) / (float) (channel -> max_pulse_width));
    // }
    uint8_t local_pulse_width = channel -> current_pulse_width;
    percentage = (uint8_t)(((float)(local_pulse_width) / 3200.0) * 100.0);
    return percentage;
}

void update_pulse_width(input_channel_t *channel, uint16_t rising_edge_capture, uint16_t falling_edge_capture) {
    uint16_t new_pulse_width = 0;
    if (falling_edge_capture < rising_edge_capture) {
        new_pulse_width = (65536 - falling_edge_capture) + rising_edge_capture;
    } else {
        new_pulse_width = falling_edge_capture - rising_edge_capture;
    }
    channel -> current_pulse_width = new_pulse_width;
}