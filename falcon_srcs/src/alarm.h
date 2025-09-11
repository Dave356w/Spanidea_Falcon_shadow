#include <Arduino.h>
#include "common.h"

#define BEEP_FLASH_TIME_MS 300

extern uint8_t alarm_status_g;

void setup_alarm();
void alarm_service();
void enable_alarm();
void disable_alarm();
