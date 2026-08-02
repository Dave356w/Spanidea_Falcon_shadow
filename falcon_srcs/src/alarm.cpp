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

static uint32_t alarm_timer;
static uint32_t chase_led_timer;
static uint32_t battery_alarm_timer;
static bool beep, beep_a, led_on;
static uint8_t counter_a = 0;
static uint8_t counter_b = 0;
static uint8_t counter_c = 0;
static bool buzzer_on = false;

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

void check_for_buzzer_alert()
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

    }

    if (beep)
    {
        digitalWrite(PIN_PIEZO, HIGH);
        buzzer_on = true;
    }
    else
    {
        digitalWrite(PIN_PIEZO, LOW);
//        buzzer_on = false;
        buzzer_on = true;
    }

}

void check_for_chase_led_alert()
{
    /*
     * Check if the CHASE-LED flag is enabled. If not, then bail out
     */
    if (chase_led_status_g == 0) {
        return ;
    }

    if (millis() - chase_led_timer > BEEP_FLASH_TIME_MS)
    {
        chase_led_timer = millis();
        counter_c = (counter_c % 5) + 1;

        if (counter_c < 3) {
            led_on = 1;
        } else {
            led_on = 0;
        }

        // Toggle the chase LED pin, 100ms once it advance the chase LED
        advance_chase_leds();

    }

    if (led_on)
    {
        digitalWrite(PIN_RED_LED_PWM, HIGH);
        digitalWrite(PIN_RED_LED_EN, HIGH);
    }
    else
    {
        digitalWrite(PIN_RED_LED_PWM, LOW);
        digitalWrite(PIN_RED_LED_EN, LOW);
    }

}

void check_for_active_alarm()
{
    check_for_buzzer_alert();
    check_for_chase_led_alert();

    return; 

}

void check_for_battery_alarm()
{
    /*
     * Check if the alarm flag is enabled. If not, then bail out
     */
    if (battery_alarm_status_g == 0) {
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
     */
    if (beep_a)
    {
        digitalWrite(PIN_PIEZO, HIGH);
        buzzer_on = true;
    }
    else
    {
        digitalWrite(PIN_PIEZO, LOW);
        buzzer_on = false;
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
