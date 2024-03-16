#include "stdint.h"
#include "tim.h"

typedef struct utime {
    uint64_t upper_32bits;
    uint8_t rollover_count;
    TIM_HandleTypeDef *handle;
} utime_t;

uint64_t utimeNow(utime_t *timer);

void rolloverCallback(utime_t *timer);

