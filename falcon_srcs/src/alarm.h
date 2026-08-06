/*
 ***********************************************************
 * File   : alarm.h
 * Author : Biju Nair
 *
 *
 * Copyright : CreeperNET Consulting 2025-26
 ***********************************************************
 */

#ifndef _ALARM_H_
#define _ALARM_H_

#include <Arduino.h>
#include "common.h"

#define BEEP_FLASH_TIME_MS        100
//#define BEEP_FLASH_TIME_MS        50
#define BATTERY_FLASH_TIME_MS     600

extern uint8_t alarm_status_g;
extern uint8_t chase_led_status_g;
extern uint8_t battery_alarm_status_g;

void setup_alarm();
void check_for_active_alarm();
void check_for_battery_alarm();
void enable_alarm();
void enable_chase_leds();
void enable_battery_alarm();
void disable_alarm();
void disable_chase_leds();
void disable_battery_alarm();
uint8_t get_alarm_status();

#endif
