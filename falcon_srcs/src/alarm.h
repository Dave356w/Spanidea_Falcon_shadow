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
 *   BUZZER_ON_STEPS / BUZZER_CYCLE_STEPS = 1/5 = 100 ms on, 400 ms off
 *
 * (was 2/5 = 200 ms on / 300 ms off until 2026-08-11 -- see the block at the
 * end of this comment, which is the authority on the current value.)
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
 *
 * ---- 2026-08-11: LOWERED 2/5 -> 1/5, and the warning above no longer binds --
 *
 * NOT to buy sample coverage. To halve piezo current, because the board was
 * measured browning out 242.7 s into a continuous alarm on FRESH cells, at an
 * ADC reading 300 counts above the low-battery trip. Alarm endurance is the
 * product's most serious defect and this is the only firmware lever on it.
 *
 * WHY THE WARNING NO LONGER APPLIES. It describes one mechanism: a higher quiet
 * fraction lets CRUISE any-motion edges pair up and satisfy the arrival
 * cluster, which released an alarm 12 s into a ride. That test has since been
 * rebuilt twice. Both arrival paths now require arrival_peak_hit():
 *
 *     arrival_edge_count >= ARRIVAL_EDGE_COUNT && ... && arrival_peak_hit()
 *
 * Cruise peaks measure 0.07-0.20 against a 0.45 gate, so the peak never crosses
 * during cruise and no amount of edge pairing can release the beacon. The
 * failure the warning is about is gated off, not merely less likely.
 *
 * AND THIS IS A DIFFERENT CHANGE FROM THE ONE THAT WAS REVERTED. 2026-08-07
 * tried 2/10 -- the same 200 ms beep at half the cadence. This is 1/5: the same
 * 500 ms cadence with a 100 ms beep. The rhythm the mechanic ranges by is
 * unchanged; only the beep length moves.
 *
 * COSTS, both real:
 *   - The beacon is audibly shorter. The spec treats the beep pattern as a UX
 *     parameter precisely because the mechanic locates the counterweight by
 *     ear. Dave made this call on 2026-08-11 knowing that.
 *   - It only helps if the brownout is driven by AVERAGE current. If the rail
 *     collapses inside each individual beep, this reduces how often that
 *     happens without stopping it. The pack fell 2442 -> 2324 gradually over
 *     four minutes, which looks like depletion rather than instantaneous sag,
 *     so average current is the better bet -- but it is a bet until somebody
 *     puts a meter across the pack during an alarm.
 *
 * ⬜ TEST: a deliberate 4+ minute continuous alarm. If endurance now exceeds
 * 242.7 s the diagnosis is confirmed and the lever works.
 */
#define BUZZER_CYCLE_STEPS        5
#define BUZZER_ON_STEPS           1
#define BATTERY_FLASH_TIME_MS     600

/* Length of one ready-signal chirp, and of the gap between chirps. */
#define READY_CHIRP_MS            200

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
void ready_signal(uint8_t chirps);

#endif
