#include "sbus.h"
#include "stdint.h"
#include "stdbool.h"
#include <stddef.h>

// Helper function for mapping one range onto another


void update_channels(channel_info_t* channel_info, uint8_t* buf) {
    // iterates through the 25-byte buffer received as per SBUS protocol
    // updates the channels when a new packet is received

    if (buf != NULL && channel_info != NULL) {
        channel_info -> channels[0] = (buf[1] | (buf[2] << 8)) & 0x07FF;
        channel_info -> channels[1] = ((buf[2] >> 3) | (buf[3] << 5)) & 0x07FF;
        channel_info -> channels[2] = ((buf[3] >> 6) | (buf[4] << 2) | (buf[5] << 10)) & 0x07FF;
        channel_info -> channels[3] = ((buf[5] >> 1) | (buf[6] << 7)) & 0x07FF;
        channel_info -> channels[4] = ((buf[6] >> 4) | (buf[7] << 4)) & 0x07FF;
        channel_info -> channels[5] = ((buf[7] >> 7) | (buf[8] << 1) | (buf[9] << 9)) & 0x07FF;
        channel_info -> channels[6] = ((buf[9] >> 2) | (buf[10] << 6)) & 0x07FF;
        channel_info -> channels[7] = ((buf[10] >> 5) | (buf[11] << 3)) & 0x07FF;
        channel_info -> channels[8] = ((buf[12] >> 0) | (buf[13] << 8)) & 0x07FF;
        channel_info -> channels[9] = ((buf[13] >> 3) | (buf[14] << 5)) & 0x07FF;
        channel_info -> channels[10] = ((buf[14] >> 6) | (buf[15] << 2) | (buf[16] << 10)) & 0x07FF;
        channel_info -> channels[11] = ((buf[16] >> 1) | (buf[17] << 7)) & 0x07FF;
        channel_info -> channels[12] = ((buf[17] >> 4) | (buf[18] << 4)) & 0x07FF;
        channel_info -> channels[13] = ((buf[18] >> 7) | (buf[19] << 1) | (buf[20] << 9)) & 0x07FF;
        channel_info -> channels[14] = ((buf[20] >> 2) | (buf[21] << 6)) & 0x07FF;
        channel_info -> channels[15] = ((buf[21] >> 5) | (buf[22] << 3)) & 0x07FF;

        if (buf[23] & (1 << 0)) {
            channel_info -> channels[16] = 1;
        } else {
            channel_info -> channels[16] = 0;
        }

        if (buf[23] & (1 << 1)) {
            channel_info -> channels[17] = 1;
        } else {
            channel_info -> channels[17] = 0;
        }

        channel_info -> frame_lost = buf[23] & (1 << 2);
        channel_info -> failsafe_activated = buf[23] & (1 << 3);

        // For channels 0 through 3, map inputs from -100 to +100


    }
}