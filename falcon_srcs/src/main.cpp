/*
 ************************************************************
 *
 * Copyright Spanidea 2024-25
 ************************************************************
 */

#include "main.h"
#include "arduino_bma456.h"
#include "common.h"
#include <EEPROM.h>

uint16_t x_axis_1, x_axis_2, y_axis_1, y_axis_2, z_axis_1, z_axis_2;
uint8_t sensor_value_updated = 0;
uint8_t alarm_status_g = 0;
uint8_t chase_led_status_g = 0;
uint8_t battery_alarm_status_g = 0;
uint32_t temp_timer = 0;
uint32_t init_time_g = 0;
float acc_mss_g = 0.0, vel_ms_g = 0.0, adj_acc_g = 0.0;
SystemStates state = SystemStates::SYSTEM_STATE_INITIALIZING;
static boolean in_isr = false;
RollingAvg<float> pressure_avg_g(2);
RollingAvg<float> acceleration_avg_g(4);
//RollingAvg<float> adj_acc_avg_g(16);
RollingAvg<uint16_t> battery_avg(8);
float x = 0, y = 0, z = 0;
float g_value, accel_value;
RollingAvg<float> bosch_acceleration_avg_g(4);
float self_calib_acceleration = 0.0;
extern eeprom_db eeprom_db_g;

MCP3208 adc(ADC_VREF, PIN_ADC_CS);
MovementService ms(&acceleration_avg_g, &acc_mss_g, &adj_acc_g, &vel_ms_g, &pressure_avg_g);

#define EN_3_AXIS_SENS 1

/*
 * Sample telemetry.
 *
 * Serial.print() must never be called from inside the timer ISR. HardwareSerial
 * drains its TX ring buffer from the UART interrupt, so printing from another
 * ISR (which re-enables interrupts) races the drain and drops characters. The
 * 2026-07-15 EFT captures show 14 corrupted lines across 8 runs, every one of
 * them inside a state-transition burst.
 *
 * The ISR now only publishes a snapshot here; loop() does the printing.
 */
typedef struct {
    uint32_t t_ms;          /* millis() at time of sample                   */
    float    accel;         /* z acceleration, m/s^2                        */
    float    avg;           /* rolling average after this sample was added  */
    uint16_t read_us;       /* duration of the bma456 I2C read, microseconds */
    uint8_t  fsm_state;     /* MovementService state at time of sample      */
    uint8_t  err_run;       /* consecutive failed sensor reads, 0 = healthy */
} sample_log_t;

static volatile sample_log_t isr_sample;
static volatile bool         sample_pending = false;
static volatile uint16_t     sample_overrun = 0;

/*
 * Free-running count of sensor reads attempted, incremented on every entry to
 * read_acceleration_mss() regardless of what happens afterwards.
 *
 * This exists to diagnose the gap described in Eng_Notes §6a: printed samples
 * arrive in bursts of ~6 followed by a gap of exactly 6 sample periods, while
 * ov= sits at 6 under every condition. Because ov= only counts snapshots the
 * ISR overwrote before loop() printed them, it cannot distinguish "the ISR
 * never ran" from "the ISR ran and the print path dropped it".
 *
 * tk= closes that gap. Compare its growth against the number of lines printed:
 *
 *   tk advances ~12 while 6 lines print  -> ISR is firing; the publish/print
 *                                           path is losing samples, and ov=
 *                                           is failing to count them
 *   tk advances ~6 while 6 lines print   -> the ISR itself is not firing, and
 *                                           the timer stalls for ~2 s at a
 *                                           time. No timing measured from
 *                                           these logs can be trusted
 *
 * Diagnostic only. Remove once §6a is resolved.
 */
static volatile uint16_t     isr_ticks = 0;

/*
 * Sensor read failures. err_run is consecutive (cleared by any good read) and
 * is what a fault decision should key off; err_total is cumulative since boot
 * and is there to catch an intermittent bus that never trips a run threshold.
 */
static volatile uint16_t     sensor_err_total = 0;
static volatile uint8_t      sensor_err_run   = 0;

/*
 * Print one line per N published samples. At the current ~3 Hz ISR rate one
 * line per sample costs about 15% of the 9600 baud budget. Raise this when the
 * timer is fixed to 100 Hz, or serial becomes the new bottleneck.
 */
#define LOG_DECIMATE_N  1

/*
 * Battery thresholds, in raw ADC counts.
 *
 * NOT millivolts. Release.txt describes a 3.2 V trip point but the code has
 * always compared against a raw count, and the scale factor between the two has
 * never been established -- see the open question in Eng_Notes §8. Do not
 * convert these to volts without measuring the divider first.
 *
 * LOW is left at the historical 1600 so this change does not alter when a
 * genuinely flat battery trips. CLEAR sits above it to give hysteresis: with a
 * single threshold, a pack sitting near the boundary would chatter the alarm on
 * and off every measurement cycle. The gap is deliberately wider than the
 * sample-to-sample spread observed on the bench (2324-2390, about 66 counts).
 */
#define BATTERY_LOW_THRESHOLD     1600
#define BATTERY_CLEAR_THRESHOLD   1750

/*
 * Battery readings to discard after boot before the alarm logic is armed, on
 * top of the averaging. See the comment in check_for_battery_voltage().
 */
#define BATTERY_SETTLE_SAMPLES    2

void emit_sample_log();

uint16_t read_battery_voltage();
extern int configure_adc_channel();
extern uint16_t read_adc_pc2_voltage();
extern bool get_buzzer_status();

void setup() {

    /*
     * Configure the debug serial port here
    */
    Serial.begin(9600);

    Serial.print(F("\r\n\nDevice Booted \r\n"));
    /*
     * Configure the ADC chip-select line here
    */
#if 0
    pinMode(PIN_ADC_CS, OUTPUT);
    digitalWrite(PIN_ADC_CS, HIGH);
#endif

    /*
     * Configure the ADC chip here for SPI protocol.
    */
    SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
    SPI.begin();
    SPI.beginTransaction(settings);
    Serial.print(F("Configured SPI interface \r\n"));

    // read_calib_data_from_eeprom();
    /*
     * Configure all Alarm Ports here
    */
    setup_alarm();
    disable_alarm();

    Serial.print(F("Configured Alarms \r\n"));

    digitalWrite(PIN_GREEN_LED, HIGH);
    init_time_g = millis();

    bma456.initialize(RANGE_2G, ODR_100_HZ, NORMAL_AVG4, CONTINUOUS);
    Serial.print(F("Configured BMA456 \r\n"));

    /*
     * Configure the ADC channel for battery voltage measurement using PC2 (ADC2).
     * This will be used to monitor the battery voltage and trigger low battery alarm if needed. 
    */

    configure_adc_channel();
}

void initialization()
{

    /*
     * One device boot-up, read the acceleration data  for
     * some time.
     */

    if (millis() - temp_timer > INIT_TIMER_MS) {

        temp_timer = millis();
//        read_battery_voltage();
        battery_avg.add(read_battery_voltage());
    }

    if (millis() - init_time_g > INIT_TIME_MS) {

        state = SystemStates::SYSTEM_STATE_NOMINAL;
        enable_timer();

        digitalWrite(PIN_PIEZO, HIGH);  
        for (int i = 0; i < 8; i++) {
            digitalWrite(PIN_CHASE_CLK, HIGH);
            delay(100);   
            digitalWrite(PIN_CHASE_CLK, LOW);
            delay(100);   
        }

        Serial.print(F("Device initialized completely \r\n"));
        digitalWrite(PIN_PIEZO, LOW);
        digitalWrite(PIN_GREEN_LED, LOW);
        digitalWrite(PIN_CHASE_LED, HIGH);
        digitalWrite(PIN_CHASE_LED, LOW);
    }
    return;
}


void loop() 
{
    static int  bma_read_counter = 0;

    switch (state) {

    case SystemStates::SYSTEM_STATE_INITIALIZING:
        initialization();
        break;

    case SystemStates::SYSTEM_STATE_NOMINAL:
        ms.fsm_run();

        emit_sample_log();

        if (bma_read_counter++ > 1000 ) {
            log_data();
            bma_read_counter = 0;
        }

        check_for_battery_voltage();

        check_for_active_alarm();
        check_for_battery_alarm();
        break;

    default:
        state = SystemStates::SYSTEM_STATE_HOLD;
        break;
    }

}

/*
 * Read z-axis accelerometer data and convert to m/(s*s)
 */

float read_acceleration_mss()
{
    uint32_t read_start;
    uint16_t rslt;

    isr_ticks++;

    read_start = micros();
    rslt = bma456.getAcceleration(&x, &y, &z);

    if (rslt != BMA4_OK) {
        /*
         * The sensor did not answer. Do NOT feed anything into the rolling
         * average: x/y/z still hold the last good sample, and zeroing them (as
         * this function used to) drags the average to 0.0, which the FSM reads
         * as "perfectly still". A unit whose accelerometer has died then
         * reports the all-clear forever. See Eng_Notes §10.2.
         *
         * The average is left holding the last good value, so the FSM neither
         * alarms nor clears on stale data while the fault persists.
         */
        sensor_err_total++;
        if (sensor_err_run < 0xFF) {
            sensor_err_run++;
        }

        if (sample_pending) {
            sample_overrun++;
        }

        isr_sample.t_ms      = millis();
        isr_sample.read_us   = (uint16_t)(micros() - read_start);
        isr_sample.fsm_state = (uint8_t)ms.get_state();
        isr_sample.err_run   = sensor_err_run;
        sample_pending = true;

        return (bosch_acceleration_avg_g.avg());
    }

    sensor_err_run = 0;

    g_value = z / 1000.0;
    accel_value = g_value * 9.81;

    acceleration_avg_g.add(accel_value);

    /*
     * Publish a snapshot for loop() to print. No serial I/O in here.
     */
    if (sample_pending) {
        /*
         * loop() has not printed the previous sample yet, so it is about to be
         * overwritten. A non-zero overrun count means the log is decimated by
         * something other than LOG_DECIMATE_N and sample timing cannot be
         * inferred from the log alone.
         */
        sample_overrun++;
    }

    isr_sample.t_ms      = millis();
    isr_sample.accel     = accel_value;
    isr_sample.avg       = acceleration_avg_g.avg();
    isr_sample.read_us   = (uint16_t)(micros() - read_start);
    isr_sample.fsm_state = (uint8_t)ms.get_state();
    isr_sample.err_run   = 0;
    sample_pending = true;

    return (bosch_acceleration_avg_g.avg());
}

/*
 * Print the most recently published sample. Called from loop() only.
 */
void emit_sample_log()
{
    static uint8_t  decimate = 0;
    sample_log_t    s;
    uint16_t        overrun;
    uint16_t        ticks;
    uint16_t        err_total;

    if (!sample_pending) {
        return;
    }

    /*
     * Copy the snapshot with interrupts masked so the ISR cannot update it
     * halfway through the read.
     */
    noInterrupts();
    memcpy(&s, (const void *)&isr_sample, sizeof(s));
    overrun   = sample_overrun;
    ticks     = isr_ticks;
    err_total = sensor_err_total;
    sample_pending = false;
    interrupts();

    if (++decimate < LOG_DECIMATE_N) {
        return;
    }
    decimate = 0;

    Serial.print(F("t="));
    Serial.print(s.t_ms);

    if (s.err_run) {
        /*
         * Failed read. Print the fault rather than a stale or fabricated
         * value, so a dead sensor is visible in the log instead of looking
         * like a stationary car.
         */
        Serial.print(F(" a=ERR er="));
        Serial.print(s.err_run);
    } else {
        Serial.print(F(" a="));
        Serial.print(s.accel, 4);
        Serial.print(F(" avg="));
        Serial.print(s.avg, 4);
    }

    Serial.print(F(" st="));
    Serial.print(s.fsm_state);
    Serial.print(F(" rd="));
    Serial.print(s.read_us);
    Serial.print(F(" ov="));
    Serial.print(overrun);
    Serial.print(F(" tk="));
    Serial.print(ticks);

    if (err_total) {
        Serial.print(F(" et="));
        Serial.print(err_total);
    }

    Serial.print(F("\r\n"));
}
#if 1
void enable_timer()
{
    cli();
    TCCR1B = 0;

    // enable the internal clock with 1024 prescale
    TCCR1B |= ((1 << CS12) | (1 <<CS10));

    // enable overflow interrupt 
    TIMSK1 |= (1 << OCIE1A);
    TCCR1B |= (1 << WGM13);
    OCR1A = 156;
    sei();
}
#endif

#if 0
void enable_timer()
{
    cli();

    // Reset control registers
    TCCR1A = 0;
    TCCR1B = 0;

    // Set CTC mode (Mode 4)
    TCCR1B |= (1 << WGM12);

    // Set compare value (for ~10 ms interrupt)
    OCR1A = 156;

    // Enable Compare Match A interrupt
    TIMSK1 |= (1 << OCIE1A);

    // Start timer with prescaler = 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);

    sei();
}
#endif
void disable_timer1()
{
    // reset Control Register to disable timer
    TCCR1B = 0;
}


ISR(TIMER1_COMPA_vect) 
{
    /*
     * If we are already inside an ISR, just bail out.
     */
    if(in_isr) {
        return;
    }

    in_isr = true;

    /*
     * Re-enable interrupts so that interrupt-based functions can 
     * be used inside this function
     */
    interrupts();

    if (get_alarm_status()) {
        if (get_buzzer_status() == false) {
            read_acceleration_mss();
        }
    } else {
        read_acceleration_mss();
    }

    /*
     * Turn off interrupts so we can't be interrupted while 
     * resetting our special variable
     */  
    noInterrupts();

    sensor_value_updated = 1;
    in_isr = false;
    return;
}

void log_data()
{
    static int loop_cnt = 0;
    static int index = 1;

    if (loop_cnt++ > 100) {
        loop_cnt = 0; 
        index++;
    }

}

void debug_log(char *p_log)
{
    Serial.print(p_log);

}

inline uint16_t read_battery_voltage() 
{
    float vol_temp = (((float) adc.read(BATT_SENSE) / 4096.0) * 3300) * (VBATT_CONST) * (1.06);

    return (uint16_t)vol_temp;
}

/*
 * This function will check for battery-voltage level and notify the user
 */
void check_for_battery_voltage()
{
    static unsigned int adc_loop_counter = 0;
    static uint8_t      settled_samples  = 0;
    uint16_t  battery_v = 0;

    adc_loop_counter++;

    if (adc_loop_counter == 30000) {
        digitalWrite(BAT_ADC_ENABLE, HIGH);
    }

    if (adc_loop_counter == 30100) {
        adc_loop_counter = 0;
        battery_v = read_adc_pc2_voltage();

        Serial.print(F("  Voltage value : "));
        Serial.print(battery_v);

        digitalWrite(BAT_ADC_ENABLE, LOW);

        /*
         * Discard the opening samples entirely.
         *
         * The 30000/30100 counters are loop passes, not milliseconds, so the
         * divider gets a very short settle at F_CPU = 1 MHz. The first read
         * also lands during the startup transient, with the boot buzzer pulse
         * and the chase LEDs loading the rail. On 2026-08-06 that produced a
         * single 1545 on a healthy battery that read 2324-2390 for the rest of
         * the session -- and because the alarm had no recovery path, that one
         * sample latched it permanently. See Eng_Notes §10.3.
         */
        if (settled_samples < BATTERY_SETTLE_SAMPLES) {
            settled_samples++;
            Serial.print(F("  (settling, ignored)\r\n"));
            return;
        }

        /*
         * Prime the whole window from the first sample we trust.
         *
         * RollingAvg fills its array with init_val (0 here), so a freshly
         * constructed battery_avg reads 0 and climbs one eighth of a sample at
         * a time. Calling add() on an unprimed window would put avg() near 300
         * for the first reading and trip the low-battery test on a perfectly
         * good battery -- reintroducing the bug this change exists to fix.
         */
        if (settled_samples == BATTERY_SETTLE_SAMPLES) {
            settled_samples++;
            battery_avg.fill(battery_v);
        } else {
            battery_avg.add(battery_v);
        }

        Serial.print(F("  avg : "));
        Serial.print(battery_avg.avg());
        Serial.print(F("\r\n"));

        /*
         * Decide on the rolling average, never on a single sample, and use
         * separate trip and clear thresholds so a reading hovering at the
         * boundary cannot chatter the alarm on and off.
         */
        if (battery_avg.avg() < BATTERY_LOW_THRESHOLD) {
            if (battery_alarm_status_g == 0) {
                Serial.print(F("  LOW Battery detected \r\n"));
            }
            enable_battery_alarm();

        } else if (battery_avg.avg() > BATTERY_CLEAR_THRESHOLD) {
            /*
             * The recovery path that did not exist before. disable_battery_alarm()
             * was defined in alarm.cpp and never called from anywhere in the
             * tree, so a latched alarm survived until a true cold boot -- which,
             * given the serial back-feed in §10.1, is harder to achieve than it
             * sounds.
             */
            if (battery_alarm_status_g) {
                Serial.print(F("  Battery recovered, alarm cleared \r\n"));
            }
            disable_battery_alarm();
        }
    }
}
