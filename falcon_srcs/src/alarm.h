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
 *
 * ─── ⛔ 2026-08-14: THERE WAS NO BROWNOUT. IT WAS THE COM LINK ───────────────
 *
 * Established at the bench by Dave. The ~243 s "deaths" were a serial-link
 * lockup, not the device: "dead" had only ever been inferred from the LOG
 * STOPPING -- no reset flag, no LED state, no measured rail -- while the bench
 * procedure already recorded that the USB-serial cable back-powers this board
 * through the RX ESD clamp.
 *
 * ⭐ SO THE BEACON WAS MADE QUIETER TO DEFEND AGAINST A PHANTOM. The 2/5 -> 1/5
 * cut above, and the duty ceiling the sequence-aligned block below reasons
 * within, were both bought with audible beacon length on a device the mechanic
 * ranges BY EAR. That constraint is void and audibility can be re-opened.
 *
 * ⚠️ BUT NOT BY REVERTING. The real trade was never the battery -- and the
 * pack measurements confirm it: the rails hold a stable 4.9 V through an alert,
 * and D2 at full is 38.8 mA against a 3.2 mA idle. Piezo duty trades against
 * SENSOR COVERAGE, because the FSM must hear the arrival transient while the
 * beacon is sounding. 🔴 And the 2026-08-07 quiet-fraction-0.75 false release
 * still binds: the present timing sits at exactly 0.75 and is safe only because
 * both arrival paths also require arrival_peak_hit().
 *
 * The question to answer is "how much blast can be bought before the listening
 * window costs an arrival?", and it is answerable by replay against the arrival
 * bursts on file BEFORE anything is flown.
 */
/*
 * ⛔ BUZZER_CYCLE_STEPS / BUZZER_ON_STEPS ARE GONE, and everything above
 * describes a mechanism that NO LONGER RUNS. The whole comment is retained
 * because its 2026-08-07 revert history is still the authority on WHY the duty
 * cycle cannot be lowered casually, and because the 2026-08-11 reasoning about
 * piezo current still applies. But the numbers in it are historical:
 * the buzzer is now driven by the sequence-aligned block below, at 150 ms on /
 * 650 ms off.
 *
 * 🔴 ONE NUMBER FROM THAT HISTORY NOW MATTERS AGAIN. The 2026-08-07 failure was
 * caused by raising the QUIET FRACTION to 0.75, which let cruise any-motion
 * edges pair up and satisfy the arrival cluster -- releasing an alarm 12 s into
 * a ride. The sequence-aligned timing below puts the quiet fraction at
 * 600/800 = 0.75. EXACTLY THAT FIGURE.
 *
 * It is safe ONLY because both arrival paths now AND in arrival_peak_hit()
 * (movement_service.cpp), and cruise peaks measure 0.10-0.32 against a 0.45
 * gate, so no amount of edge pairing can release the beacon. ⚠️ IF THE PEAK
 * GATE IS EVER WEAKENED OR REMOVED FROM THE ANY-MOTION PATH, this timing
 * re-arms the 2026-08-07 failure. Check that before touching either.
 */
/*
 * ─── LOW-BATTERY CHIRP, 2026-08-14 ───────────────────────────────────────────
 *
 * WHAT WAS THERE. BATTERY_FLASH_TIME_MS 600, with counter_a cycling 1..6 and
 * the piezo driven while counter_a < 4 -- i.e. 1800 ms ON / 1800 ms OFF,
 * CONTINUOUSLY, for as long as the pack stayed under the trip point. Roughly
 * 50% piezo duty. (The inline comment claiming "200 ms on, 300 ms off" was
 * stale and had not matched the constants for some time.)
 *
 * WHY THAT WAS THE WRONG RESPONSE. A continuous alarm was measured browning the
 * board out 242.7 s in ON FRESH CELLS. Answering "the pack is low" with a
 * 50%-duty sounder spends the remaining charge faster than anything else the
 * firmware can do -- it accelerates the failure it is reporting, and on a device
 * whose worst outcome is SILENCE WHILE MOVING that is the wrong direction.
 *
 * WHAT THIS IS. The smoke-alarm convention the customer asked for: one short
 * chirp plus one red LED flash, at a long interval. Piezo duty falls from ~50%
 * to 80/45000 = 0.18%, about 280x less energy per unit time, and the signal is
 * the one every mechanic already recognises as "change the batteries".
 *
 * SUPPRESSED ENTIRELY WHILE THE MOVEMENT BEACON IS SOUNDING. The piezo is
 * shared, so a chirp during a ride would punch a hole in the rhythm the mechanic
 * ranges by AND unblank the accelerometer mid-blast. This is what makes the old
 * "it must not speak for the movement alarm" hazard (see check_for_battery_alarm)
 * go away by construction rather than by careful ordering.
 *
 * ✅ AND THE TRIP POINT IS NOW SOUND TOO, later the same day: the divider was
 * metered at 2:1, the ADC prescaler brought into spec, and the whole chain
 * validated against a meter to within one LSB. The trip sits at 3.6 V of pack.
 * See the threshold block in main.cpp. The earlier "do not ship this to a
 * customer until the divider is characterised" warning is discharged.
 */
#define BATTERY_CHIRP_PERIOD_MS   45000  /* silence between chirps            */
#define BATTERY_CHIRP_MS          80     /* piezo + red LED, per chirp        */

/*
 * ─── SEQUENCE-ALIGNED ALARM, 2026-08-13 ──────────────────────────────────────
 *
 * WHAT WAS WRONG. The buzzer and the chase ran on INDEPENDENT timers and
 * counters (alarm_timer/counter_b, chase_led_timer/counter_c), both stepping at
 * 100 ms. The buzzer cycled every 500 ms; the 74HC4017 advanced every 100 ms and
 * free-ran, so a visual sequence took 800 ms (or 1000 ms if the part runs its
 * full decade -- the firmware never knew, because nothing tracked position and
 * only disable_chase_leds() ever pulsed MR). 500 against 800/1000 is
 * incommensurate, so the blast drifted through the sequence and Dave observed
 * it landing near both the start and the end of each chase. That was a beat
 * pattern, not a design.
 *
 * WHAT THIS DOES. One master step counter drives both, and MR is pulsed by
 * FIRMWARE at step 0. That makes the sequence length whatever we define --
 * exactly CHASE_LED_COUNT LEDs -- regardless of the 4017's natural modulus, so
 * the 8-vs-10 question stops mattering. One blast per sequence, at its start.
 *
 * THE NUMBERS, and why the duty cycle is the constraint. A naive "one 200 ms
 * blast per 800 ms" is 25% duty against the old 20% -- MORE average piezo
 * current, which runs straight into the 243 s brownout that 2026-08-11's
 * 2/5 -> 1/5 change was made to attack. So the step is halved to 50 ms to buy
 * the resolution for a 150 ms blast instead:
 *
 *                          old              new
 *   blast length           100 ms           150 ms      (50% longer)
 *   cadence                500 ms, drifting 800 ms, aligned
 *   piezo duty             20%              18.75%      (slightly LESS current)
 *   blanked fraction       30%              25%
 *   longest quiet stretch  350 ms           600 ms      (+71%)
 *
 * So the beacon is audibly longer per blast, the monitoring window is 71%
 * wider, and average current goes DOWN rather than up. The rhythm the mechanic
 * ranges by slows from 2 Hz to 1.25 Hz -- that is the real UX cost, and it is
 * the reason this is a judgement call rather than a free win.
 *
 * ⚠️ Keep ALARM_SEQ_STEPS = CHASE_LED_COUNT * CHASE_STEPS_PER_LED, or the MR
 * pulse stops landing on the first LED and the alignment silently breaks.
 */
#define ALARM_STEP_MS             50   /* master step for buzzer + chase       */
#define ALARM_SEQ_STEPS           16   /* 16 x 50 ms = 800 ms per sequence     */
#define ALARM_BUZZ_STEPS          3    /* 3 x 50 ms = 150 ms blast, at step 0  */
#define ALARM_RED_STEPS           4    /* 4 x 50 ms = 200 ms red LED           */
#define CHASE_STEPS_PER_LED       1    /* advance every 50 ms -- see below     */
/*
 * LEDs FITTED on the 4017 -- eight, D3..D10, on outputs Q1..Q8. Q0 and Q9 are
 * unpopulated, which is what gives the ring an off state (common.h).
 *
 * ⚠️ THIS IS A COUNT OF LIT POSITIONS, NOT OF SWEEP STEPS, and the reset step
 * does not consume one of them: the sweep pulses MR and advances onto Q1 in the
 * same step, so all eight show in CHASE_SWEEP_STEPS steps. Until 2026-08-14 the
 * reset step sat on the dark Q0 and only seven ever lit -- D10 never came on
 * during an alert. See check_for_buzzer_alert().
 */
#define CHASE_LED_COUNT           8

/*
 * Steps in one full visual sweep. MR is pulsed at the start of EVERY sweep, not
 * just at step 0, so the chase can run faster than the buzzer while still
 * beginning each sweep on LED 1.
 *
 * ⚠️ ALARM_SEQ_STEPS MUST BE AN INTEGER MULTIPLE OF THIS, or the last sweep in a
 * sequence is truncated and the blast stops landing on LED 1.
 *   16 / (8 x 1) = 2 sweeps per sequence.  ok
 *   16 / (8 x 2) = 1 sweep  per sequence.  also ok (the 2026-08-13 original)
 *
 * ─── CHASE DOUBLED 2026-08-13, PIEZO CADENCE UNTOUCHED ───────────────────────
 *
 * CHASE_STEPS_PER_LED 2 -> 1 on Dave's request after seeing the faster cadence
 * of the bench test build: 8 LEDs in 400 ms rather than 800 ms, so TWO clean
 * sweeps per sequence with the blast on the first.
 *
 * The buzzer is driven off the step index (steps 0..ALARM_BUZZ_STEPS-1 of
 * ALARM_SEQ_STEPS), so its cadence and duty are completely unaffected. This
 * changes only what the eye sees.
 *
 * IT COSTS NOTHING ON EITHER AXIS THAT MATTERS, which is why it was worth doing:
 *
 *  - SENSOR COVERAGE: unchanged. Blanking is set by `buzzer_on`, which follows
 *    the blast plus BUZZER_RINGDOWN_MS. The chase does not gate the sample read
 *    at all, so the 25% blanked fraction and the 600 ms contiguous quiet stretch
 *    are exactly as before.
 *  - CURRENT: unchanged. The 4017 holds exactly ONE output high at any instant
 *    regardless of how fast it is clocked, so average LED current is identical.
 *    Nothing is spent against the alarm-endurance budget.
 *
 * ⚠️ WHAT WOULD COST SENSOR COVERAGE is speeding up the BUZZER, because
 * BUZZER_RINGDOWN_MS is a FIXED 50 ms paid once per blast and does not scale
 * with cadence. For a sequence of period T with the blast at ALARM_BUZZ_STEPS /
 * ALARM_SEQ_STEPS = 3/16:
 *
 *     blanked %        = 18.75% + 5000/T
 *     contiguous quiet = 0.8125 x T - 50 ms
 *
 *        T = 400 ms   31.3% blanked   275 ms quiet   <- worse than pre-08-13
 *        T = 600 ms   27.1% blanked   438 ms quiet
 *        T = 800 ms   25.0% blanked   600 ms quiet   <- current
 *        T = 1200 ms  22.9% blanked   925 ms quiet
 *
 * So halving the SEQUENCE would put the listening window below where it was
 * before any of this work. Speed the chase, not the buzzer.
 */
#define CHASE_SWEEP_STEPS         (CHASE_LED_COUNT * CHASE_STEPS_PER_LED)

/*
 * Persistence-of-vision step for the ready flash. The 74HC4017 is a DECADE
 * COUNTER -- exactly one output is high at a time, so there is no state in which
 * all eight LEDs are on and no firmware change can create one. Clocking through
 * every position faster than the eye integrates gives the APPEARANCE of all of
 * them lit, at 1/8 brightness. 2 ms per position is ~50-62 Hz for a full pass
 * depending on whether the part wraps at 8 or 10, which is above flicker.
 */
#define CHASE_POV_STEP_MS         2

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

/*
 * ─── IDLE HEARTBEAT, 2026-08-14 ──────────────────────────────────────────────
 *
 * THE PROBLEM, as the customer put it: at rest the unit gives no indication of
 * whether it is switched off or armed and waiting for a trip. There is no way to
 * tell a dead device from a listening one without picking it up -- on a product
 * whose worst failure is silence while moving, that is the exact ambiguity you
 * do not want a mechanic resolving by assumption.
 *
 * IT RUNS ON D2, THE CENTRE LED, DIMMED. See the silkscreen map in common.h --
 * the firmware names and the board labels are different vocabularies, and that
 * cost two bench iterations before this landed on the right part.
 *
 * ⛔ THERE IS NO GREEN LED ON V2. Confirmed against all seven sheets of
 * RTC1273R2_SCH.pdf on 2026-08-14: D1 is a D5V0F1U2LP-7B ESD diode on the POWER
 * sheet, not an indicator. PIN_GREEN_LED is a V1 leftover pointing at PC2, which
 * on this board is BAT_ADC -- the battery divider. Nothing to drive. Do not
 * revive it.
 *
 * ⛔ AND THE CHASE BAR WAS THE WRONG HOME, tried first and abandoned:
 *   - D3..D10 are the PERIMETER ring. "Centre LED" always meant D2.
 *   - selecting a ring position means pulsing MR and CLOCKING to it, because the
 *     4017 has no output-enable, and the positions walked through are genuinely
 *     lit on the way -- order 10 us each against the destination, roughly
 *     1000:1 dimmer. Invisible on paper; visible to a dark-adapted eye. Dave
 *     saw a faint D3->D4 chase ahead of every beat on D5. There is no firmware
 *     fix; a position cannot be selected without walking to it. (That sighting
 *     is also what established D3 = Q1 -- see the silkscreen map in common.h.)
 * D2 is directly driven, so both problems disappear rather than being managed.
 *
 * ⚠️ D2 COSTS REAL CHARGE, SO THE BEAT IS DIMMED. Sheet 3 is titled "LED DRIVER
 * RED LED 200mA": D2 is an LXM2-PD01-0050 behind U2, a TPS92201 constant-current
 * driver with EN and PWM control. On the chase ring a beat cost nothing, because
 * the 4017 lights exactly one output regardless of which. Here it does not.
 *
 * ⛔ THE "200 mA" IS THE DRIVER'S CAPABILITY, NOT ITS SETTING. Measured at the
 * pack 2026-08-14, and every earlier figure in this file was arithmetic off that
 * 200 mA and wrong:
 *
 *                            claimed      MEASURED
 *     D2 at full             200 mA       38.8 mA
 *     D2 at HEARTBEAT_PWM_DUTY  ~12 mA     0.20 mA
 *     heartbeat average        ~250 uA     4.0 uA
 *
 * ⭐ THE DIMMING IS WORTH FAR MORE THAN THE ARITHMETIC SUGGESTED. Idle baseline
 * is 3.2 mA, so a FULL-brightness beat would cost 0.78 mA -- 24% of idle. The
 * dimmed beat costs 4 uA, or 0.1%. Dimming buys back very nearly the whole cost
 * of having a heartbeat at all.
 *
 * ⬜ AND THE DIM IS FAR DIMMER THAN ITS DUTY CYCLE, which is worth understanding
 * before anyone tunes it. HEARTBEAT_PWM_DUTY 16/255 is 6.3%, but the measured
 * current is 0.52% of full -- a factor of 12 low. The likely cause is the
 * TPS92201's soft-start: at Timer0's 61 Hz the on-time is only ~1 ms, and the
 * converter spends much of that ramping rather than regulating. So brightness is
 * NOT linear in HEARTBEAT_PWM_DUTY at this frequency, and raising the duty will
 * buy visibility more cheaply than the duty number implies. ⚠️ If the wink turns
 * out to be too faint to serve its purpose, raise the duty and RE-MEASURE rather
 * than scaling from 6.3%.
 *
 * WHY THE FLASH IS 80 ms AND NOT 20. LED_PWM is PD6 = OC0A, and at F_CPU = 1 MHz
 * the Arduino core's Timer0 PWM runs at 1e6/(64 x 256) = 61 Hz. A 20 ms window
 * spans barely one PWM cycle, so the "dim" beat would land as a single short
 * pulse whose length depended on where the window fell in the cycle. 80 ms spans
 * about five cycles and integrates to a genuine dim glow. ⚠️ DO NOT SHORTEN THIS
 * below ~50 ms without moving off Timer0 -- the dimming stops averaging and the
 * beat becomes phase-dependent.
 *
 * TIMER0 IS SAFE TO USE. The firmware drives Timer1 only (the 25 Hz sample CTC);
 * Timer0 is left entirely to the Arduino core, so analogWrite() on OC0A works
 * and millis() is unaffected. The alarm path still uses plain digitalWrite for
 * full brightness, and digitalWrite on a PWM pin clears the compare output, so
 * the two cannot fight over the pin.
 *
 * ⚠️ D2 IS SHARED THREE WAYS -- alarm blast, low-battery chirp, heartbeat. The
 * arbitration is strict priority in loop() order: the alarm suppresses both
 * others outright, and a chirp in progress suppresses the beat. The two
 * survivors are separated to the eye by brightness and rhythm, and the chirp has
 * the piezo behind it, so they are not confusable in practice.
 *
 * PATTERN CARRIES THE CALIBRATION VERDICT, which is free and closes a real gap.
 * "Armed, but the site could not be measured -- expect a twitchy device" is
 * announced once as 3 chirps at the end of calibration and then never again, so
 * a mechanic who walked away during the chirp has no way to recover it. A
 * double-blink heartbeat keeps that state visible for the whole deployment.
 */
#define HEARTBEAT_PERIOD_MS       4000   /* quiet gap between beats            */
#define HEARTBEAT_FLASH_MS        80     /* ~5 Timer0 PWM cycles at 1 MHz      */
#define HEARTBEAT_GAP_MS          180    /* spacing of the degraded double     */
#define HEARTBEAT_PWM_DUTY        16     /* of 255 -- ~6% of D2's 200 mA       */

#define HEARTBEAT_OFF             0      /* not armed: booting or calibrating  */
#define HEARTBEAT_OK              1      /* armed, site measured               */
#define HEARTBEAT_DEGRADED        2      /* armed, fallback or noisy floor     */

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
void heartbeat_set(uint8_t pattern);
void heartbeat_service();

#if defined(IDLE_CURRENT_TEST)
/*
 * ⛔ BENCH ONLY. Hold D2 in a FIXED state so a meter can read it, instead of
 * chasing an 80 ms pulse every 4 s. 0 = off, 1 = HEARTBEAT_PWM_DUTY, 2 = full.
 *
 * Lives in alarm.cpp rather than in the test harness so the OCR0A / COM0A1
 * handling stays in one file -- the same code the heartbeat uses, so what the
 * meter reads is what the heartbeat actually costs.
 */
void bench_set_d2(uint8_t mode);
#endif

#endif
