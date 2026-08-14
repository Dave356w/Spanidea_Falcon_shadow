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
    
    /*
     * ─── ADC PRESCALER: /128 -> /16, 2026-08-14 ──────────────────────────────
     *
     * WHAT WAS WRONG. The comment said "for 8MHz clock: 8MHz/128 = 62.5kHz" and
     * the arithmetic was right -- for an 8 MHz part. F_CPU here is 1 MHz, so
     * /128 gave a 7.8 kHz ADC clock against a datasheet minimum of 50 kHz for
     * full 10-bit accuracy. Every battery reading this project has ever taken
     * was out of spec (Eng_Notes §5.8).
     *
     * ⚠️ BUT THE OBVIOUS FIX IS NOT THE RIGHT ONE, and this is the part worth
     * reading. The divider is 499k/499k (sheet 3, metered 2:1), so the ADC sees
     * a source impedance of about 250 kOhm -- TWENTY-FIVE TIMES the 10 kOhm the
     * datasheet recommends. The sample-and-hold capacitor (~14 pF) charges
     * through that with a time constant of ~3.5 us, and the S/H window is only
     * 1.5 ADC clocks wide:
     *
     *      /8   125 kHz   S/H  12 us   3.4 tau   ~3% short   <- worse than now
     *      /16   62.5 kHz S/H  24 us   6.9 tau   ~0.1%       <- chosen
     *      /32   31.25kHz S/H  48 us   14 tau    settled, but BELOW 50 kHz
     *      /128   7.8 kHz S/H 192 us   55 tau    settled, far below 50 kHz
     *
     * So the two datasheet limits pull in OPPOSITE directions here: resolution
     * wants a fast clock, the high-impedance source wants a slow one. The old
     * value was accidentally satisfying the second while violating the first.
     *
     * /16 is the SLOWEST IN-SPEC option, which maximises S/H time while meeting
     * the resolution requirement -- the only choice that satisfies both. Do not
     * "improve" this to /8: it is faster, it looks more correct, and it would
     * make the reading worse.
     *
     * The remaining ~0.1% is covered by a discarded conversion in
     * read_adc_pc2_voltage(), which pre-charges the S/H cap so the conversion
     * that counts starts from the right voltage rather than from whatever the
     * previous channel left behind.
     *
     * SIDE EFFECT, all upside: a conversion drops from ~1.7 ms to ~208 us.
     *
     * ✅ VALIDATED AGAINST A METER, same day. After this change the firmware
     * reported a 5012 mV pack against 5.010 V measured -- 2 mV, inside one LSB.
     * Before it, the same chain read 4860 against a 4900 meter, 0.8% LOW, which
     * is precisely the direction an unfinished S/H charge errs. The fix is not
     * merely datasheet compliance; it removed a real and consistent bias.
     */
    ADCSRA = (1 << ADEN) | (1 << ADPS2);
   
    DIDR0 |= (1 << ADC2D);
    // External ADC chip initialization (kept for compatibility)
//    adc.begin();
    
    return (0);

}

#if defined(BROWNOUT_TEST) || defined(BATTERY_BENCH)
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
 * ⚠️ Accuracy caveat: the bandgap is only specified to about +/-2%, untrimmed.
 * Fine for a relative in-blast vs quiet comparison, not for absolute
 * calibration -- for that, meter the pack and compare against pack_mv.
 *
 * (The second caveat that used to sit here described the /128 prescaler and the
 * ~1.7 ms conversion it caused. Both are gone: the prescaler is /16, in spec,
 * and a conversion is ~208 us. See configure_adc_channel().)
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
#endif /* BROWNOUT_TEST || BATTERY_BENCH */

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
    /*
     * DISCARD ONE CONVERSION. The battery divider is ~250 kOhm -- 25x the
     * datasheet's recommended source impedance -- so the sample-and-hold cap
     * cannot fully charge within one 1.5-clock sample window at any in-spec
     * prescaler (see configure_adc_channel()). A throwaway conversion leaves
     * the cap sitting at very nearly the node voltage, so the conversion that
     * counts starts from the right place instead of from whatever the previous
     * read left on it.
     *
     * Costs ~208 us at /16. The same trick is already used in read_vcc_mv()
     * for the bandgap, and for the same reason.
     */
    (void)read_adc_pc2();

    uint16_t adc_result = read_adc_pc2();
    
    // Convert 10-bit ADC result to voltage
    // Voltage = (ADC_result / 1023) * VREF
    // For 3.3V reference: Voltage_mV = (ADC_result * 3300) / 1023
    uint16_t voltage_mv = (uint32_t)adc_result * 3100 / 1023;
    
    return voltage_mv;
}

