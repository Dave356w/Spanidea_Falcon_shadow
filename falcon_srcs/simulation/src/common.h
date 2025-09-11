#ifndef __COMM_H_
#define __COMM_H_

#include <inttypes.h>
#include <time.h>

extern void alarm_service();
extern void disable_alarm();
extern void enable_alarm();

inline uint32_t millis()
{   
    struct timespec tr;
    clock_gettime(CLOCK_MONOTONIC, &tr);
    return (tr.tv_sec * 1000) + (tr.tv_nsec / 1000000);
}

#endif