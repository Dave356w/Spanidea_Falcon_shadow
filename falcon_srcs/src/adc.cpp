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

#if defined(BROWNOUT_TEST)
/*
 * ─── BENCH ONLY: measure VCC itself, via the 1.1 V bandgap ───────────────────
 *
 * WHY THE EXISTING BATTERY READ CANNOT SEE A SAGGING RAIL. configure_adc_channel()
 * sets ADMUX = (1<<REFS0) | ADC2 -- AVCC as the REFERENCE, with the battery
 * divider as the INPUT. If AVCC is the rail that is sagging, then input and
 * reference fall together and the ratio barely moves: the measurement is
 * RATIOMETRIC and is blind to VCC by construction. The 2026-08-11 note explained
 * the blindness as the average hiding an instantaneous dip; this is the better
 * explanation, and it means no amount of faster sampling on ADC2 would have
 * revealed it.
 *
 * The fix is to measure a FIXED voltage against AVCC instead. MUX=1110 selects
 * the internal 1.1 V bandgap, so the result is inversely proportional to VCC:
 *
 *     VCC_mV = 1100 * 1024 / adc
 *
 * A sagging rail makes this reading RISE. No divider, no external meter.
 *
 * ⚠️ Accuracy caveats, both acceptable for a relative in-blast vs quiet
 * comparison but not for absolute calibration:
 *   - The bandgap is only specified to about +/-2%, untrimmed.
 *   - The ADC prescaler here is /128, set in a comment that says "for 8MHz
 *     clock" while F_CPU is 1 MHz -- so the ADC clock is 7.8 kHz, BELOW the
 *     50 kHz datasheet minimum for full 10-bit accuracy, and a conversion takes
 *     ~1.7 ms rather than ~104 us. Left alone deliberately: the shipping
 *     battery thresholds were characterised with this prescaler, and 1.7 ms
 *     still fits comfortably inside a 150 ms blast.
 */
uint16_t read_vcc_mv()
{
    uint8_t admux_saved = ADMUX;

    /* AVCC reference, MUX = 1110 = internal 1.1 V bandgap. */
    ADMUX = (1 << REFS0) | 0x0E;

    /*
     * The bandgap needs time to start up after being selected, and the S/H cap
     * needs to settle. The datasheet quotes ~70 us; be generous, this is a
     * bench build.
     */
    delay(2);

    /* Discard one conversion, for the same reason. */
    (void)read_adc_pc2();
    uint16_t adc = read_adc_pc2();

    ADMUX = admux_saved;
    delay(1);
    (void)read_adc_pc2();   /* leave the battery channel settled again */

    if (adc == 0) {
        return 0;
    }
    return (uint16_t)((1100UL * 1024UL) / adc);
}
#endif /* BROWNOUT_TEST */

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

