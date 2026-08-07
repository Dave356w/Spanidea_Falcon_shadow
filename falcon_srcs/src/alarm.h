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

/*
 * How long the accelerometer stays blanked after PIN_PIEZO drops, milliseconds.
 *
 * The piezo keeps ringing mechanically after the drive pin goes low, and that
 * ringdown is what commit b795dd2 was really fighting when it blanked sensor
 * reads for the whole alarm. Blanking just the ringdown instead of the entire
 * cycle recovers most of the beep-off phase for listening.
 *
 * 50 ms of a 300 ms off-phase leaves 250 ms usable out of every 500 ms cycle.
 * If the average still looks polluted during an alarm, raise this before
 * reaching for a filter -- see the warning in Eng_Notes §5 about the ~2 Hz beep
 * envelope sitting directly on the elevator motion band.
 */
#define BUZZER_RINGDOWN_MS        50

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
bool get_buzzer_status();

#endif
