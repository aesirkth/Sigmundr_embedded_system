#ifndef APOAPSIS_DETECTION_H
#define APOAPSIS_DETECTION_H

#include "definitions.h"
#include "stdlib.h"
#include "sys/types.h"

typedef struct {
    int tv_sec;
    int tv_msec;
}time_value;

__uint8_t detectEventsAndTriggerParachute();
#endif