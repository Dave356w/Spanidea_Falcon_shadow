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
 *   BUZZER_ON_STEPS / BUZZER_CYCLE_STEPS = 2/5 = 200 ms on, 300 ms off
 *
 * ⚠️ DO NOT LOWER THE DUTY TO WIN BACK ARRIVAL SIGNAL. It was tried on
 * 2026-08-07 (2/10, 200 ms on / 800 ms off) and had to be reverted the same
 * afternoon. The reasoning was sound and the result was a regression.
 *
 * buzzer_on gates whether an any-motion edge is trusted for arrival
 * clustering, so at 2/5 roughly half of every arrival's edges were being
 * discarded -- which looked like pure loss. Raising the quiet fraction from
 * 0.50 to 0.75 did make arrivals cluster reliably. It also made CRUISE
 * cluster:
 *
 *              runs held    released by     quiet edges/min while alarming
 *   2/5        68.4, 70.6 s   polled                 1.8 - 4.5
 *   2/10        5.4,  4.1 s   any-motion            31.3
 *
 * At 50 fpm the car's own vibration keeps any-motion asserted continuously,
 * so every 1 s status poll raises an edge and two consecutive polls look
 * exactly like an arrival. Measured cluster gaps were 1016 ms during cruise
 * against 802-1671 ms at real arrivals -- the same number, because both mean
 * "still asserted". No window or count separates them.
 *
 * The 50% duty was not merely avoiding false positives; it was randomly
 * decimating cruise edges and that decimation is what kept consecutive polls
 * from pairing. It is doing real work. Leave it at 2/5.
 *
 * If arrival sensitivity needs improving, do it somewhere that does not also
 * amplify cruise -- the polled margin was 4% on the 2026-08-07 down run
 * (0.312 against a 0.30 threshold) and that is the number worth attacking.
 */
#define BUZZER_CYCLE_STEPS        5
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
