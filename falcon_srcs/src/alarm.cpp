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
static uint32_t battery_alarm_timer;
static bool beep, beep_a, led_on;
static uint8_t counter_a = 0;         /* battery alarm only                  */
static bool buzzer_on = false;

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
             * Start of a sweep. Pulse MR so the 4017 lands on its first output
             * -- this is what makes the alignment independent of whether the
             * part wraps at 8 or 10. At seq_step 0 this also coincides with the
             * blast, which is the alignment Dave asked for; later sweeps in the
             * same sequence get their own reset so a faster chase still starts
             * each sweep on LED 1.
             */
            digitalWrite(PIN_CHASE_LED, HIGH);
            digitalWrite(PIN_CHASE_LED, LOW);
        } else if (chase_en && (seq_step % CHASE_STEPS_PER_LED) == 0) {
            /* Advance to LEDs 2..CHASE_LED_COUNT within the current sweep. */
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

void check_for_battery_alarm()
{
    static bool     ringdown_a = false;
    static uint32_t ringdown_a_start = 0;

    /*
     * Check if the alarm flag is enabled. If not, then bail out
     */
    if (battery_alarm_status_g == 0) {
        ringdown_a = false;
        return ;
    }

    if (millis() - battery_alarm_timer > BATTERY_FLASH_TIME_MS)
    {
        battery_alarm_timer = millis();
        counter_a = (counter_a % 6) + 1;

        // it keeps buzzer and led on for 200 ms, and buzzer and led off for 300ms
        if (counter_a < 4) {
            beep_a = 1;
        } else {
            beep_a = 0;
        }

    }

    /*
     * Turn on RED-LED & Buzzer
     *
     * Two things this has to get right, both of which it used to get wrong.
     *
     * 1. RINGDOWN. The movement path blanks the accelerometer for
     *    BUZZER_RINGDOWN_MS after the pin drops, because the piezo keeps
     *    vibrating mechanically. This path cleared buzzer_on the instant the
     *    pin went low, so the ringing was sampled unblanked -- and worse,
     *    counted as a trustworthy any-motion edge for arrival clustering.
     *
     * 2. IT MUST NOT SPEAK FOR THE MOVEMENT ALARM. loop() calls
     *    check_for_active_alarm() and then check_for_battery_alarm(), so this
     *    function writes buzzer_on last and wins. The battery pattern is
     *    1800 ms on / 1800 ms off against the movement pattern's 200/300, so
     *    with both alarms active this would clear buzzer_on for well over a
     *    second while the movement buzzer was still beeping -- unblanking the
     *    sensor at exactly the wrong moment. It now only clears the flag when
     *    the movement alarm is not running.
     */
    if (beep_a)
    {
        digitalWrite(PIN_PIEZO, HIGH);
        buzzer_on = true;
        ringdown_a = false;
    }
    else
    {
        digitalWrite(PIN_PIEZO, LOW);

        if (!ringdown_a) {
            ringdown_a = true;
            ringdown_a_start = millis();
        }

        if ((millis() - ringdown_a_start) >= BUZZER_RINGDOWN_MS &&
            alarm_status_g == 0) {
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
     * Turn OFF the alarm & RED LED
     */

    digitalWrite(PIN_PIEZO, LOW);
    battery_alarm_status_g = 0;
}

inline void advance_chase_leds()
{
    // Generate pulse to advance the Chase LEDs
    digitalWrite(PIN_CHASE_CLK, HIGH);
    digitalWrite(PIN_CHASE_CLK, LOW);
}
