#ifndef SBUS
#define SBUS

#include "stdint.h"
#include "stdbool.h"

#define HEADER_BYTE 0x0F
#define ELEVEN_BYTES_MASK 0x07FF
#define CHANNEL_0_LOW 44
#define CHANNEL_0_HIGH 1811
#define CHANNEL_1_LOW 172
#define CHANNEL_1_HIGH 1795
#define CHANNEL_2_LOW 172
#define CHANNEL_2_HIGH 1233
#define CHANNEL_3_LOW 172
#define CHANNEL_3_HIGH 1811

typedef struct channel_info {
    uint16_t channels[18];
    bool frame_lost;
    bool failsafe_activated;
} channel_info_t;

void update_channels(channel_info_t* channel_info, uint8_t* buf);

#endif