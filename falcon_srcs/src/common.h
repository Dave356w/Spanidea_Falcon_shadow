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
