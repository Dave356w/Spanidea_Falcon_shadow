/*
 ***********************************************************
 * File   : alarm.cpp
 * Author : Biju Nair
 *
 *
 * Copyright : CreeperNET Consulting 2025-26
 ***********************************************************
 */

#include "alarm.h"

static uint32_t alarm_timer;          /* master step timer, buzzer + chase   */
static uint32_t battery_alarm_timer;  /* chirp interval AND chirp length      */
static bool beep, led_on;
static bool buzzer_on = false;

/*
 * beep_a and counter_a are gone with the 1800/1800 continuous battery sounder;
 * the chirp is a two-state timer and needs neither. See alarm.h.
 */

/*
 * True while the low-battery chirp is driving D2. File scope rather than a
 * function static because the heartbeat runs on the same LED and has to stand
 * aside for it -- see the arbitration note in alarm.h.
 */
static bool battery_chirping = false;

/* Idle heartbeat state -- see the IDLE HEARTBEAT block in alarm.h. */
static uint8_t  heartbeat_pattern = HEARTBEAT_OFF;
static uint32_t heartbeat_timer   = 0;
static uint8_t  heartbeat_phase   = 0;

/*
 * chase_led_timer, counter_b and counter_c are gone. They belonged to the two
 * independent timers that made the blast drift against the visual sequence --
 * one master step counter replaces them, in check_for_buzzer_alert(). counter_b
 * is still referenced inside the #if 0 block below, which is why that block now
 * declares its own.
 */

inline void advance_chase_leds();

/*
 * Configure the Alarm ports. Configure RED, CHASE and the Piezo buzzer here
 */
void setup_alarm()
{
    pinMode(PIN_GREEN_LED, OUTPUT);
    digitalWrite(PIN_GREEN_LED, LOW);

    pinMode(PIN_PIEZO, OUTPUT);
    digitalWrite(PIN_PIEZO, LOW);

    pinMode(PIN_RED_LED_PWM, OUTPUT);
    pinMode(PIN_RED_LED_EN, OUTPUT);
    digitalWrite(PIN_RED_LED_PWM, LOW);
    digitalWrite(PIN_RED_LED_EN, LOW);

    pinMode(PIN_CHASE_LED, OUTPUT);
    digitalWrite(PIN_CHASE_LED, HIGH);
    digitalWrite(PIN_CHASE_LED, LOW);

    pinMode(PIN_CHASE_CLK, OUTPUT);

}

#if 0

static uint8_t counter_b = 0;   /* dead code below kept its own counter */

void check_for_active_alarm()
{
    /*
     * Check if the alarm flag is enabled. If not, then bail out
     */
    if (alarm_status_g == 0) {
        return ;
    }

    if (millis() - alarm_timer > BEEP_FLASH_TIME_MS)
    {
        alarm_timer = millis();
        counter_b = (counter_b % 5) + 1;

        // it keeps buzzer and led on for 200 ms, and buzzer and led off for 300ms
        if (counter_b < 3) {
            beep = 1;
        } else {
            beep = 0;
        }

        // toggle the chase LED pin, 100ms once it advance the chase LED
        advance_chase_leds();
    }

    /*
     * Turn on RED-LED & Buzzer
     */
    if (beep)
    {
//        digitalWrite(PIN_PIEZO, HIGH);
        digitalWrite(PIN_RED_LED_PWM, HIGH);
        digitalWrite(PIN_RED_LED_EN, HIGH);
        buzzer_on = true;
    }
    else
    {
//        digitalWrite(PIN_PIEZO, LOW);
        digitalWrite(PIN_RED_LED_PWM, LOW);
        digitalWrite(PIN_RED_LED_EN, LOW);
        buzzer_on = false;
    }

    return; 
}
#endif

/*
 * One master-stepped sequence driving the buzzer, the chase and the red LED
 * together. Replaces check_for_buzzer_alert() and check_for_chase_led_alert(),
 * which ran on independent timers and drifted against each other -- see the
 * SEQUENCE-ALIGNED ALARM block in alarm.h for the measurements and the reason.
 *
 * The buzzer and the chase are still enabled independently (alarm_status_g and
 * chase_led_status_g), and in practice the FSM turns them on and off together
 * at STATE_MOVING entry and at release. The step counter runs whenever EITHER
 * is active, so the sequence phase stays coherent if that ever changes.
 */
void check_for_buzzer_alert()
{
    static bool     ringdown_active = false;
    static uint32_t ringdown_start  = 0;
    static uint8_t  seq_step        = 0;

    bool buzz_on  = (alarm_status_g != 0);
    bool chase_en = (chase_led_status_g != 0);

    if (!buzz_on && !chase_en) {
        /*
         * Reset the phase so the next alarm starts at step 0 with a fresh MR
         * pulse, and clear the ringdown state so a stale timestamp from the
         * previous alarm cannot make the next one skip its blanking window.
         */
        seq_step        = 0;
        ringdown_active = false;
        beep            = 0;
        buzzer_on       = false;
        return ;
    }

    if (millis() - alarm_timer >= ALARM_STEP_MS)
    {
        alarm_timer = millis();

        if ((seq_step % CHASE_SWEEP_STEPS) == 0) {
            /*
             * Start of a sweep. Pulse MR so the 4017 lands on a known output --
             * this is what makes the alignment independent of whether the part
             * wraps at 8 or 10. At seq_step 0 this also coincides with the
             * blast, which is the alignment Dave asked for; later sweeps in the
             * same sequence get their own reset so a faster chase still starts
             * each sweep on the first LED.
             */
            digitalWrite(PIN_CHASE_LED, HIGH);
            digitalWrite(PIN_CHASE_LED, LOW);

            /*
             * ⭐ AND STEP STRAIGHT OFF Q0, because Q0 IS UNPOPULATED -- D3 is Q1
             * (see the silkscreen map in common.h).
             *
             * THE BUG THIS FIXES, reported at the bench 2026-08-14: the sweep is
             * CHASE_SWEEP_STEPS = 8 steps and MR used to consume one of them on
             * an output that lights nothing, so only SEVEN positions ever showed
             * -- Q1..Q7 = D3..D9. D10 never lit during an alert and the ring
             * carried a visible gap. The beacon is the product's primary visual
             * and it had been running at 7/8 for the whole project.
             *
             * WHY IT IS DONE HERE RATHER THAN BY LENGTHENING THE SWEEP. The
             * obvious fix -- CHASE_LED_COUNT 8 -> 9 to buy a step for MR --
             * forces ALARM_SEQ_STEPS to a multiple of 9, and 16 is not one. That
             * drags in a new sequence length, a new cadence, and a re-check of
             * the blanked/quiet table below. Folding the advance into the reset
             * step costs two digitalWrites and changes NO timing at all: same
             * 800 ms sequence, same 150 ms blast at step 0, same 25% blanked,
             * same 600 ms contiguous quiet. The blast now lands on D3 rather
             * than on a dark slot, which is if anything the better alignment.
             */
            if (chase_en) {
                advance_chase_leds();
            }
        } else if (chase_en && (seq_step % CHASE_STEPS_PER_LED) == 0) {
            /* Advance through the rest of the sweep: Q2..Q8 = D4..D10. */
            advance_chase_leds();
        }

        beep   = (seq_step < ALARM_BUZZ_STEPS) ? 1 : 0;
        led_on = (seq_step < ALARM_RED_STEPS)  ? 1 : 0;

        seq_step = (uint8_t)((seq_step + 1) % ALARM_SEQ_STEPS);
    }

    /*
     * Red LED follows the blast rather than keeping its own cadence, so the
     * audible and visual cues coincide. Its duty drops from 40% to 25%, which
     * is a small additional current saving.
     */
    if (chase_en && led_on) {
        digitalWrite(PIN_RED_LED_PWM, HIGH);
        digitalWrite(PIN_RED_LED_EN, HIGH);
    } else {
        digitalWrite(PIN_RED_LED_PWM, LOW);
        digitalWrite(PIN_RED_LED_EN, LOW);
    }

    if (!buzz_on) {
        /* Chase only: keep the piezo silent and the sensor unblanked. */
        digitalWrite(PIN_PIEZO, LOW);
        buzzer_on = false;
        return ;
    }

    /*
     * buzzer_on gates the accelerometer read in the timer ISR. Setting it true
     * in BOTH branches -- as 59e945f did -- blanked the sensor for the entire
     * alarm, not just while the pin was driven, which is why the device could
     * not tell whether the car was still moving while it was alarming.
     *
     * Measured 2026-08-07: 10,224 ms of alarm produced ONE accelerometer sample.
     * Coverage during an alarm was not the 33% estimated from source reading,
     * it was effectively zero.
     *
     * The buzzer must still be blanked while the piezo is driven, and for a
     * short ringdown afterwards -- that part of b795dd2 was correct. But the
     * rest of the off-phase is usable, and §3's latched FSM depends on it: the
     * decel transient that releases the alarm happens *while the alarm is
     * sounding*, so something has to be listening then.
     *
     * Any-motion cannot substitute here. The 2026-08-07 bench run showed the
     * sensor's own engine triggers continuously on the buzzer (Eng_Notes §11),
     * so moving the decision into hardware does not avoid this.
     */
    if (beep)
    {
        digitalWrite(PIN_PIEZO, HIGH);
        buzzer_on = true;
        ringdown_active = false;
    }
    else
    {
        digitalWrite(PIN_PIEZO, LOW);

        if (!ringdown_active) {
            /* Falling edge: start the ringdown blanking window. */
            ringdown_active = true;
            ringdown_start  = millis();
        }

        if ((millis() - ringdown_start) >= BUZZER_RINGDOWN_MS) {
            buzzer_on = false;
        }
    }

}

/*
 * The chase and the red LED are driven from the master sequence in
 * check_for_buzzer_alert() now, so this no longer exists as a separate path.
 * It had its own timer and its own mod-5 counter, which is precisely why the
 * blast drifted against the visual sequence.
 */

void check_for_active_alarm()
{
    check_for_buzzer_alert();

    return;

}

/*
 * Low-battery chirp -- one short piezo pulse and one red LED flash every
 * BATTERY_CHIRP_PERIOD_MS. See the LOW-BATTERY CHIRP block in alarm.h for what
 * this replaced and why a continuous sounder was the wrong answer to a flat
 * pack.
 */
void check_for_battery_alarm()
{
    static bool     ringdown_a       = false;
    static uint32_t ringdown_a_start = 0;
    uint32_t        now              = millis();

    /*
     * Check if the alarm flag is enabled. If not, then bail out
     */
    if (battery_alarm_status_g == 0) {
        battery_chirping = false;
        ringdown_a       = false;
        return ;
    }

    /*
     * THE MOVEMENT BEACON OWNS THE PIEZO. Suppress the chirp outright while it
     * is sounding, and restart the interval so the first chirp after a ride is
     * a full period away rather than landing on the heels of the alarm.
     *
     * This is what retires the hazard the old implementation had to handle by
     * ordering. loop() calls check_for_active_alarm() and THEN this function,
     * so whatever this writes to buzzer_on wins; the old 1800/1800 pattern
     * could therefore clear buzzer_on -- unblanking the accelerometer -- for
     * well over a second while the movement buzzer was still beeping. With the
     * chirp suppressed there is no second writer at all.
     */
    if (alarm_status_g != 0 || chase_led_status_g != 0) {
        if (battery_chirping) {
            /* Mid-chirp when the ride began: drop the pin, keep the ringdown. */
            digitalWrite(PIN_RED_LED_PWM, LOW);
            digitalWrite(PIN_RED_LED_EN, LOW);
            battery_chirping = false;
        }
        ringdown_a          = false;
        battery_alarm_timer = now;
        return ;
    }

    if (!battery_chirping) {
        if ((now - battery_alarm_timer) >= BATTERY_CHIRP_PERIOD_MS) {
            battery_chirping    = true;
            battery_alarm_timer = now;

            digitalWrite(PIN_PIEZO, HIGH);
            digitalWrite(PIN_RED_LED_PWM, HIGH);
            digitalWrite(PIN_RED_LED_EN, HIGH);

            /* Blanks the accelerometer for the length of the chirp. */
            buzzer_on  = true;
            ringdown_a = false;
        }
    } else if ((now - battery_alarm_timer) >= BATTERY_CHIRP_MS) {
        battery_chirping    = false;
        /* Interval is measured from the END of a chirp. */
        battery_alarm_timer = now;

        digitalWrite(PIN_PIEZO, LOW);
        digitalWrite(PIN_RED_LED_PWM, LOW);
        digitalWrite(PIN_RED_LED_EN, LOW);

        /*
         * RINGDOWN. The piezo keeps vibrating mechanically after the pin drops,
         * so the sensor stays blanked for BUZZER_RINGDOWN_MS afterwards. The
         * original code cleared buzzer_on the instant the pin went low, and the
         * ringing was then sampled unblanked -- and worse, counted as a
         * trustworthy any-motion edge for arrival clustering.
         */
        ringdown_a       = true;
        ringdown_a_start = now;
    }

    if (ringdown_a && (millis() - ringdown_a_start) >= BUZZER_RINGDOWN_MS) {
        ringdown_a = false;
        if (alarm_status_g == 0) {
            buzzer_on = false;
        }
    }

    return;
}

void enable_chase_leds()
{
    if (chase_led_status_g == 1) {
        return ;
    }

    chase_led_status_g = 1;

}

void enable_alarm()
{
    /*
     * Check if alarm-flag is already enabled. If so, it means
     * that the alarm is already active.
     */

    if (alarm_status_g == 1) {
        return ;
    }

    // disable the RGB LED
    digitalWrite(PIN_GREEN_LED, LOW);

    alarm_status_g = 1;
}

void enable_battery_alarm()
{
    /*
     * Check if alarm-flag is already enabled. If so, it means
     * that the alarm is already active.
     */

    if (battery_alarm_status_g == 1) {
        return ;
    }

    /*
     * Chirp AT ONCE on the transition rather than a full period later. The
     * detection itself already takes ~90 s of sampling to arm (see
     * check_for_battery_voltage), and a mechanic is far more likely to still be
     * beside the unit now than 45 s from now.
     */
    battery_alarm_timer = millis() - BATTERY_CHIRP_PERIOD_MS;

    battery_alarm_status_g = 1;
}

bool get_buzzer_status()
{
    return (buzzer_on);
}

/*
 * Blocking chirp pattern used to report the calibration result.
 *
 * "Signals ready to use" is the only rich channel this product has -- the one
 * moment the mechanic is standing beside the device rather than a hoistway
 * away from it. One chirp is a clean arm; three say the unit armed but the
 * site could not be measured and it will be twitchy.
 *
 * Blocking is acceptable and nothing else is: this runs at the end of
 * STATE_CALIBERATION, before the FSM is armed, so there is no beacon to
 * delay. It is deliberately NOT built out of enable_alarm(), whose duty cycle
 * is a sensing parameter that must not be repurposed for UX.
 */
/*
 * Hold every chase LED apparently lit for duration_ms, then leave the counter
 * reset.
 *
 * The 74HC4017 is a decade counter -- exactly one output is high at a time, so
 * "all LEDs on" is not a state the hardware has. Clocking through the positions
 * faster than the eye integrates produces the appearance of all of them lit at
 * 1/8 brightness. See CHASE_POV_STEP_MS in alarm.h.
 *
 * Blocking, and deliberately so: this runs once at boot from the calibration
 * path, where nothing else needs the CPU.
 */
static void chase_pov_all_on(uint16_t duration_ms)
{
    uint32_t start = millis();

    /* MR pulse: begin from a known position. */
    digitalWrite(PIN_CHASE_LED, HIGH);
    digitalWrite(PIN_CHASE_LED, LOW);

    while ((millis() - start) < duration_ms) {
        advance_chase_leds();
        delay(CHASE_POV_STEP_MS);
    }

    /* Leave the counter reset so the first alarm sequence starts clean. */
    digitalWrite(PIN_CHASE_LED, HIGH);
    digitalWrite(PIN_CHASE_LED, LOW);
}

/*
 * Ready / calibration-complete signal.
 *
 * Previously this held PIN_CHASE_LED (which is MR, not a data line) HIGH for the
 * whole chirp, pinning the 4017 in reset so exactly ONE LED lit -- a weak
 * visual for "the device is armed and you can walk away". Now the chirp is
 * accompanied by an apparent all-LED flash plus the red LED, so readiness is
 * unmistakable across a machine room.
 *
 * Timing is unchanged: chase_pov_all_on() runs for READY_CHIRP_MS and replaces
 * the delay() that used to sit there, so 1 chirp still means "good" and 3 still
 * mean "calibrated while moving, or noisy".
 *
 * NOTE: this drives the piezo directly and deliberately does NOT set
 * buzzer_on -- see the comment at its call site in movement_service.cpp.
 */
void ready_signal(uint8_t chirps)
{
    while (chirps--) {
        digitalWrite(PIN_PIEZO, HIGH);
        digitalWrite(PIN_RED_LED_PWM, HIGH);
        digitalWrite(PIN_RED_LED_EN, HIGH);

        chase_pov_all_on(READY_CHIRP_MS);

        digitalWrite(PIN_PIEZO, LOW);
        digitalWrite(PIN_RED_LED_PWM, LOW);
        digitalWrite(PIN_RED_LED_EN, LOW);

        if (chirps) {
            delay(READY_CHIRP_MS);
        }
    }
}

/*
 * ─── IDLE HEARTBEAT ──────────────────────────────────────────────────────────
 *
 * Rationale, the current arithmetic and the Timer0 constraint are all in the
 * IDLE HEARTBEAT block in alarm.h. This drives D2 -- the centre LED, behind the
 * TPS92201 constant-current driver -- at HEARTBEAT_PWM_DUTY rather than full
 * brightness, because at 200 mA a full-brightness beat would cost about three
 * times the idle MCU.
 *
 * The chase ring is not touched at all any more, which retires the 4017
 * alignment hazard the first implementation had to tiptoe around: this cannot
 * clock the counter out from under the alarm sequence because it never clocks it.
 */
static void heartbeat_led(bool on)
{
    if (on) {
        /*
         * PWM sets brightness, EN gates the driver. Order matters: set the duty
         * before enabling, so the driver never sees a full-brightness instant on
         * the way up.
         *
         * ⚠️ DELIBERATELY NOT analogWrite(). That function switches over every
         * timer pin on the part and cost 310 bytes of flash -- against 772 free
         * and a documented 1198-byte shortfall for WIRE_TIMEOUT (platformio.ini),
         * which is the fix for the TWI lockup. Two register writes do the same
         * job. If this ever needs to become analogWrite() again, check the flash
         * budget first.
         *
         * Timer0 is already running in fast PWM at /64 from the Arduino core's
         * init() -- the firmware only ever configures Timer1 -- so setting COM0A1
         * is all that is needed to route OC0A to the pin. OCR0A is not used by
         * millis(), which runs off the overflow interrupt.
         */
        OCR0A   = HEARTBEAT_PWM_DUTY;
        TCCR0A |= (1 << COM0A1);
        digitalWrite(PIN_RED_LED_EN, HIGH);
    } else {
        digitalWrite(PIN_RED_LED_EN, LOW);

        /*
         * Release OC0A before driving the pin, or the compare output would keep
         * overriding the port register. This is what hands the pin back to the
         * alarm path, which drives it with plain digitalWrite for full
         * brightness.
         */
        TCCR0A &= ~(1 << COM0A1);
        digitalWrite(PIN_RED_LED_PWM, LOW);
    }
}

void heartbeat_set(uint8_t pattern)
{
    if (pattern == heartbeat_pattern) {
        return ;
    }

    heartbeat_pattern = pattern;

    /* Restart cleanly so a pattern change cannot strand a beat half-done. */
    if (heartbeat_phase != 0) {
        heartbeat_led(false);
        heartbeat_phase = 0;
    }
    heartbeat_timer = millis();
}

void heartbeat_service()
{
    uint32_t now = millis();

    /*
     * Stand aside for anything with a stronger claim on D2, and hold the
     * interval from expiring meanwhile so the first beat afterwards is a full
     * period away rather than landing the instant the LED is released.
     *
     *   HEARTBEAT_OFF        not armed -- booting or calibrating
     *   alarm / chase        the beacon owns D2 outright
     *   battery_chirping     an 80 ms chirp is mid-flight on the same LED
     *
     * ⚠️ The battery case suppresses the BEAT, not the heartbeat. A low pack is
     * exactly when "is this thing still alive?" matters most, so the beat
     * resumes between chirps rather than being disabled for the deployment.
     */
    if (heartbeat_pattern == HEARTBEAT_OFF ||
        alarm_status_g != 0 || chase_led_status_g != 0 ||
        battery_chirping) {

        if (heartbeat_phase != 0) {
            /*
             * Only clear the LED if nothing else is driving it -- otherwise this
             * would blank an alarm pulse or a chirp that owns the pin right now.
             */
            if (!battery_chirping && alarm_status_g == 0 &&
                chase_led_status_g == 0) {
                heartbeat_led(false);
            }
            heartbeat_phase = 0;
        }
        heartbeat_timer = now;
        return ;
    }

    switch (heartbeat_phase) {

    case 0:                                  /* quiet, waiting for the beat   */
        if ((now - heartbeat_timer) >= HEARTBEAT_PERIOD_MS) {
            heartbeat_timer = now;
            heartbeat_led(true);
            heartbeat_phase = 1;
        }
        break;

    case 1:                                  /* first flash                   */
        if ((now - heartbeat_timer) >= HEARTBEAT_FLASH_MS) {
            heartbeat_timer = now;
            heartbeat_led(false);
            heartbeat_phase =
                (heartbeat_pattern == HEARTBEAT_DEGRADED) ? 2 : 0;
        }
        break;

    case 2:                                  /* gap of the degraded double    */
        if ((now - heartbeat_timer) >= HEARTBEAT_GAP_MS) {
            heartbeat_timer = now;
            heartbeat_led(true);
            heartbeat_phase = 3;
        }
        break;

    case 3:                                  /* second flash                  */
        if ((now - heartbeat_timer) >= HEARTBEAT_FLASH_MS) {
            heartbeat_timer = now;
            heartbeat_led(false);
            heartbeat_phase = 0;
        }
        break;

    default:
        heartbeat_phase = 0;
        break;
    }
}

uint8_t get_alarm_status()
{
    uint8_t al_status = alarm_status_g | battery_alarm_status_g;
    return (al_status);
}

void disable_alarm()
{
    /*
     * Check if alarm-flag is already disabled. If so, it means
     * that the alarm is already disabled.
     */
    if (alarm_status_g == 0) {
      return ;
    }
    /*
     * Turn OFF the alarm & RED LED
     */
    digitalWrite(PIN_PIEZO, LOW);
    buzzer_on = false;

//    digitalWrite(PIN_CHASE_LED, HIGH);
//    digitalWrite(PIN_CHASE_LED, LOW);
//    digitalWrite(PIN_RED_LED_PWM, LOW);
//    digitalWrite(PIN_RED_LED_EN, LOW);

    alarm_status_g = 0;
}

void disable_chase_leds()
{
    /*
     * Check if alarm-flag is already disabled. If so, it means
     * that the alarm is already disabled.
     */
    if (chase_led_status_g == 0) {
      return ;
    }
    /*
     * Turn OFF the alarm & RED LED
     */
    digitalWrite(PIN_CHASE_LED, HIGH);
    digitalWrite(PIN_CHASE_LED, LOW);
    digitalWrite(PIN_RED_LED_PWM, LOW);
    digitalWrite(PIN_RED_LED_EN, LOW);

    chase_led_status_g = 0;
}

void disable_battery_alarm()
{
    /*
     * Check if alarm-flag is already disabled. If so, it means
     * that the alarm is already disabled.
     */

    if (battery_alarm_status_g == 0) {
        return ;
    }

    /*
     * Turn OFF the alarm & RED LED.
     *
     * The red LED is cleared here as well now. The chirp drives it, so a
     * recovery that landed inside the 80 ms pulse used to leave it latched on
     * for the rest of the deployment -- which on this product reads as an
     * alarm state that nothing will ever clear.
     *
     * Guarded on the movement alarm: it owns both pins while it is sounding.
     */
    if (alarm_status_g == 0 && chase_led_status_g == 0) {
        digitalWrite(PIN_PIEZO, LOW);
        digitalWrite(PIN_RED_LED_PWM, LOW);
        digitalWrite(PIN_RED_LED_EN, LOW);

        /*
         * ⚠️ AND CLEAR THE BLANKING FLAG. buzzer_on gates the accelerometer read
         * in the timer ISR. The chirp sets it true for its 80 ms, so a recovery
         * detected inside that window would exit with the flag stuck true and
         * NOTHING would ever clear it -- the sensor would stay blanked for the
         * rest of the deployment and the device would be deaf to a departure.
         * Narrow window, worst possible consequence: this is exactly the
         * silence-while-moving direction.
         *
         * The chirp's own ringdown is abandoned along with it. That is the safe
         * trade: the ringdown protects a metric from piezo ringing, and losing
         * it costs at most a few polluted samples, whereas leaving the flag set
         * costs every sample from here on.
         */
        buzzer_on = false;
    }

    battery_alarm_status_g = 0;
}

inline void advance_chase_leds()
{
    // Generate pulse to advance the Chase LEDs
    digitalWrite(PIN_CHASE_CLK, HIGH);
    digitalWrite(PIN_CHASE_CLK, LOW);
}
