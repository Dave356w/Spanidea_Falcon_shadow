/*
 ***********************************************************
 * File   : adc.cpp
 * Author : Biju Nair
 *
 *
 * Copyright : CreeperNET Consulting 2025-26
 ***********************************************************
 */

#include "main.h"
#include "arduino_bma456.h"
#include "common.h"
#include <EEPROM.h>

uint16_t read_battery_voltage();


int configure_adc_channel()
{  
    /*
     * Configure the ADC channel for battery voltage measurement using PC2 (ADC2).
     * This will be used to monitor the battery voltage and trigger low battery alarm if needed.
     */
    
    // Configure PC2 as input (should already be in input mode by default)
    pinMode(BAT_ADC_CHANNEL, INPUT);
    
    // Enable BAT_ADC_ENABLE pin for ADC measurement control
    pinMode(BAT_ADC_ENABLE, OUTPUT);
    digitalWrite(BAT_ADC_ENABLE, LOW);
    
    // Initialize the internal ADC
    // ADMUX: Set reference voltage to AVCC and select channel ADC2 (PC2)
    ADMUX = (1 << REFS0) | (0x02 & 0x07);  // REFS0=1 (AVCC), ADC2 (PC2)
    
    // ADCSRA: Enable ADC, set prescaler to 128 (for 8MHz clock: 8MHz/128 = 62.5kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
   
    DIDR0 |= (1 << ADC2D);
    // External ADC chip initialization (kept for compatibility)
//    adc.begin();
    
    return (0);

}

/*
 * Read ADC value from PC2 (ADC2 channel)
 * Returns: 10-bit ADC result (0-1023)
 */
uint16_t read_adc_pc2()
{
    // Start ADC conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC)) {
        // Conversion in progress
    }
    
    // Read the 10-bit result from ADC register
    // ADCL must be read first, then ADCH (as per datasheet)
    uint16_t adc_result = ADCL;           // Read low byte first
    adc_result |= (ADCH << 8);            // Read high byte and combine
    
    return adc_result;
}

/*
 * Read ADC value from PC2 and convert to voltage (mV)
 * Assumes AVCC (3.3V) as reference voltage
 * Returns: Voltage in millivolts
 */
uint16_t read_adc_pc2_voltage()
{
    uint16_t adc_result = read_adc_pc2();
    
    // Convert 10-bit ADC result to voltage
    // Voltage = (ADC_result / 1023) * VREF
    // For 3.3V reference: Voltage_mV = (ADC_result * 3300) / 1023
    uint16_t voltage_mv = (uint32_t)adc_result * 3100 / 1023;
    
    return voltage_mv;
}

