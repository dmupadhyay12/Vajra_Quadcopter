#include "utime.h"
#include "stdint.h"
#include "tim.h"

uint64_t utimeNow(utime_t *timer) {
    return ((timer -> upper_32bits) << 32) | (uint64_t) (__HAL_TIM_GET_COUNTER(timer -> handle));
}

void rolloverCallback(utime_t *timer) {
    if (timer -> upper_32bits == 0xFFFFFFFF) {
        timer -> upper_32bits = 0;
        timer -> rollover_count ++;
    } else {

    	timer -> upper_32bits ++;
    }
}
