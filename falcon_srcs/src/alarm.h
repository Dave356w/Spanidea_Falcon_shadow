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

/*
 * Buzzer duty cycle, in units of BEEP_FLASH_TIME_MS (100 ms).
 *
 *   BUZZER_ON_STEPS / BUZZER_CYCLE_STEPS = 2/10 = 200 ms on, 800 ms off
 *
 * Was 2/5 (200 ms on, 300 ms off). The change is about detection, not sound.
 *
 * buzzer_on gates BOTH the accelerometer read (§5 item 7) and whether an
 * any-motion edge is trusted for arrival clustering. At 2/5, with the 50 ms
 * ringdown on top, the piezo owned 250 ms of every 500 ms -- half the time --
 * so roughly half of a real arrival's interrupt edges were raised while the
 * buzzer was driving and had to be discarded. On the 2026-08-07 50 fpm down
 * run the arrival produced exactly two edges 656 ms apart; one carried bz=1,
 * was rejected, and the arrival went undetected.
 *
 * With 2 edges at an arrival, the chance both land in a quiet window is q^2:
 *
 *      duty            quiet fraction q    P(both quiet)
 *   2/5  + ringdown          0.50              25%
 *   2/10 + ringdown          0.75              56%
 *
 * It cuts in the other direction too: buzzer-induced edges can only occur
 * while the piezo is driven, so less driving means fewer false ones. On the
 * up run six were raised and discarded, including a 196 ms pair that would
 * otherwise have released the alarm mid-ride.
 *
 * Side benefit: the piezo draws current for a quarter of the time instead of
 * half, which matters for a latch that can now sound for 80+ seconds (§13.4).
 *
 * ⚠️ This changes what the product SOUNDS like -- same beep length, half as
 * often. That is a user-facing change, not just a tuning constant.
 */
#define BUZZER_CYCLE_STEPS        10
#define BUZZER_ON_STEPS           2
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
