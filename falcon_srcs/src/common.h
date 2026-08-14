/*
 ***********************************************************
 * File   : common.h
 * Author : Biju Nair
 *
 *
 * Copyright : CreeperNET Consulting 2025-26
 ***********************************************************
 */

#ifndef _COMMON_H_
#define _COMMON_H_

/*
 * ─── SILKSCREEN MAP, confirmed at the bench 2026-08-14 ───────────────────────
 *
 * Recorded because the firmware names and the board labels are not the same
 * vocabulary, and "the centre LED" meant something different to each of them --
 * the 2026-08-14 heartbeat was built on the chase bar for exactly that reason.
 *
 *   D1        ⛔ NOT AN LED. D5V0F1U2LP-7B, an ESD/TVS protection diode on the
 *             POWER sheet. There is NO green indicator anywhere on this board --
 *             checked across all seven sheets of RTC1273R2_SCH.pdf.
 *
 *             PIN_GREEN_LED below is therefore a V1 LEFTOVER and is dead. It
 *             points at PC2, which on V2 is the BAT_ADC divider node, and
 *             configure_adc_channel() re-drives that pin to INPUT after
 *             setup_alarm() has claimed it as an output -- so every write to it
 *             after boot lands on an ADC input. Do not try to revive it.
 *
 *   D2        CENTRE LED. LXM2-PD01-0050 behind U2, a TPS92201 constant-current
 *             driver -- sheet 3 is titled "LED DRIVER RED LED 200mA". Controlled
 *             by PIN_RED_LED_PWM (PD6, = OC0A, hardware PWM capable) and
 *             PIN_RED_LED_EN (PD7). Genuinely off when EN is low.
 *
 *             ⚠️ MEASURED 38.8 mA at full, 2026-08-14 -- the "200mA" on the
 *             sheet is the driver's CAPABILITY, not its configured current, and
 *             several comments in this tree were built on the larger figure
 *             before anyone put a meter on it. 38.8 mA is still 12x the 3.2 mA
 *             idle baseline, so anything that lights D2 on a duty cycle costs
 *             real charge -- unlike the ring below, which is free. The idle
 *             heartbeat dims it via PWM for exactly this reason (see alarm.h).
 *
 *   D3..D10   the 8 PERIMETER ring LEDs. APDA3020SECK parts driven through
 *             PUMH10 transistor arrays from the 74HC4017 (U6).
 *
 *             ⭐ Q0 IS UNPOPULATED AND D3 IS Q1. The 4017 has ten outputs and
 *             sheet 6 fits eight LEDs, so two outputs go nowhere -- and Q0, the
 *             reset position, is one of them. That is a deliberate and rather
 *             good hardware decision: it gives the ring an OFF state that the
 *             part itself does not have, because exactly one output is always
 *             high and MR parks it on the one that lights nothing.
 *
 *             Established 2026-08-14 from two independent observations: the ring
 *             is dark at rest, and the first heartbeat implementation -- MR then
 *             three advances -- was seen to transit D3, D4 and settle on D5,
 *             which is Q1, Q2, Q3. Both require D3 = Q1.
 *
 * ⛔ AN EARLIER VERSION OF THIS COMMENT CLAIMED D3 BURNS CURRENT CONTINUOUSLY,
 * on the reasoning that MR parks the counter on Q0 and the 4017 always has one
 * output high. The reasoning was sound and the premise was wrong: Q0 lights
 * nothing. The ring costs NOTHING at rest, and there is no idle LED drain to
 * hunt for. Do not re-derive the alarm.
 *
 * ✅ AND IT CAUGHT A REAL BUG, now fixed. The alarm sweep used to spend its
 * reset step sitting on the dark Q0, so only SEVEN positions ever lit --
 * Q1..Q7 = D3..D9. D10 never came on during an alert and the ring carried a
 * visible gap. Predicted from this map on 2026-08-14 and confirmed at the bench
 * the same day ("D10 doesn't light on alert, chase skips D10"). The beacon is
 * the product's primary visual and it had been running at 7/8 for the whole
 * project. Fixed by advancing onto Q1 in the same step as the MR pulse -- see
 * check_for_buzzer_alert(), which explains why that rather than a longer sweep.
 */

//#define FALCON_V1
#ifdef FALCON_V1

    /*
     * This is the pin details for Version-1 board
     */

    #define PIN_ADC_CS                          14 //PC0
    #define PIN_GREEN_LED                       PIN_PC2 //PC2-16
    #define ARD_SDA_PIN                         18 //PC4
    #define ARD_SCL_PIN                         19 //PC5
    #define PIN_RX                              0 //PD0
    #define PIN_TX                              1 //PD1
    #define PIN_PSI_INT                         3 //PD3
    #define PIN_CHASE_LED                       PIN_PD6 //PD6-6
    #define PIN_CHASE_CLK                       PIN_PD7 //PD7-7
    #define PIN_PIEZO                           PIN_PB0 //PB0-8
    #define PIN_RED_LED                         PIN_PB1 //PB1-9

#else

    #define PIN_ADC_CS                          14      // TBD
    #define PIN_GREEN_LED                       PIN_PC2 // TBD
    #define ARD_SDA_PIN                         PIN_PE0
    #define ARD_SCL_PIN                         PIN_PE1
    #define PIN_RX                              PIN_PD0
    #define PIN_TX                              PIN_PD1
    #define PIN_PSI_INT                         3       // TBD
    #define PIN_CHASE_LED                       PIN_PC5 // Testing of this pin in progress
    #define PIN_CHASE_CLK                       PIN_PB1
    #define PIN_PIEZO                           PIN_PD5  // Buzzer pin
    #define PIN_RED_LED_PWM                     PIN_PD6 // TBD
    #define PIN_RED_LED_EN                      PIN_PD7

    #define BAT_ADC_ENABLE                      PIN_PB0
    #define BAT_ADC_CHANNEL                     PIN_PC2

    /*
     * BMA456 interrupt outputs.
     *
     * Confirmed against RTC1273R2 PAGE04:uC on 2026-08-06: net INT1_ACC lands
     * on INT0/PD2 and INT2_ACC on INT1/PD3. Both are true external interrupt
     * pins, so no pin-change-interrupt workaround is needed.
     *
     * NOTE: PIN_PSI_INT above is also PD3. That name predates the DPS310 being
     * depopulated -- the pin is the accelerometer's second interrupt, not a
     * pressure-sensor line. Nothing reads PIN_PSI_INT today. See Eng_Notes §11.
     */
    #define PIN_ACC_INT1                        PIN_PD2
    #define PIN_ACC_INT2                        PIN_PD3

#endif

#endif
