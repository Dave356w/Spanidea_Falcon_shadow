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
 * Cleared until acceleration_avg_g has been primed from a real sample. See
 * read_acceleration_mss() for why an unprimed window is dangerous.
 */
static volatile bool         accel_avg_primed = false;

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

/*
 * Any-motion interrupt test (Eng_Notes §11).
 *
 * OBSERVATION ONLY. The interrupt runs alongside the existing polled detector
 * and changes no FSM behaviour; it exists to answer three questions before any
 * architecture decision is made:
 *
 *   1. Does the INT1_ACC -> PD2 path work at all on this hardware?
 *   2. Does the sensor flag real elevator departures and arrivals?
 *   3. Does the piezo trigger it? §5 says the buzzer couples mechanically into
 *      the accelerometer, and the sensor's engine sits behind the same physics,
 *      so any-motion is expected to see the buzzer too. Measuring how badly is
 *      the point.
 *
 * THRESHOLD ARITHMETIC -- treat as a starting point, not a tuned value.
 *
 * The field is 11 bits (BMA456_ANY_NO_MOTION_THRES_MSK = 0x07FF) spanning
 * roughly 1 g at the configured RANGE_2G, giving ~0.488 mg per count. That LSB
 * is derived from the mask width, NOT read off the datasheet -- confirm against
 * Datasheet/ before trusting any number computed from it.
 *
 * Against the July captures:
 *   arrival transient  +1.35 m/s^2 = 138 mg  ~= 282 counts
 *   stationary noise   +/-0.16 m/s^2 = 16 mg ~=  33 counts
 *
 * 96 counts (~47 mg) sits comfortably above the noise floor and well below the
 * arrival transient. The 18 fpm departure was below the noise floor in July and
 * may not trigger at any threshold -- that is one of the things being tested.
 *
 * Z AXIS ONLY as of the hoistway run. The raw a= samples are logged regardless,
 * so parse_falcon_log.py can sweep thresholds after the fact and one run tests
 * many values; ACC-INT tells you what the sensor actually did at 96.
 *
 * Duration is in 50 Hz samples: 5 = 100 ms. This is the hardware equivalent of
 * §7's sustain gating, which found N>=3 consecutive samples eliminated every
 * false fire.
 */
#define ANYMOTION_THRESHOLD       96
#define ANYMOTION_DURATION        5
#define ANYMOTION_INT_LINE        BMA4_INTR1_MAP

/* How often to read/clear the sensor interrupt status, milliseconds. */
#define ACC_INT_POLL_MS           1000

static volatile uint16_t     acc_int_count = 0;
static volatile uint32_t     acc_int_last_ms = 0;

void acc_int1_isr();
void emit_acc_int_log();
void poll_acc_int_status();

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
     * Arm the any-motion engine and listen on INT1_ACC / PD2. Observation only
     * -- see the ANYMOTION_* block above.
     */
    {
        uint16_t rslt = bma456.configureAnyMotion(ANYMOTION_THRESHOLD,
                                                  ANYMOTION_DURATION,
                                                  BMA4_DISABLE,
                                                  ANYMOTION_INT_LINE);
        if (rslt != BMA4_OK) {
            Serial.print(F("AnyMotion config FAILED : "));
            Serial.print(rslt);
            Serial.print(F("\r\n"));
        } else {
            Serial.print(F("AnyMotion armed thr="));
            Serial.print(ANYMOTION_THRESHOLD);
            Serial.print(F(" dur="));
            Serial.print(ANYMOTION_DURATION);
            Serial.print(F("\r\n"));
        }
    }

    pinMode(PIN_ACC_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ACC_INT1), acc_int1_isr, RISING);

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
        emit_acc_int_log();
        poll_acc_int_status();

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

    /*
     * Prime the window from the first good sample instead of letting it fill
     * from zero.
     *
     * RollingAvg initialises every slot to init_val (0.0 here), so an unprimed
     * acceleration_avg_g(4) reports 0.00, 2.43, 4.86, 7.29 and only then the
     * true ~9.72 as the window fills. That ramp is indistinguishable from a
     * large upward acceleration, and on 2026-08-06 it produced a false
     * STATE_MOVEMENT_DETECTED on a device sitting still on a bench.
     *
     * It then got stuck: the FSM captured its baseline at 2.4265 (one sample
     * into the unprimed window, 9.7152/4) and compared against it forever,
     * giving "FSM: Delta : 7.294032" and "Still in movement" on every cycle
     * with no way back to STATE_MONITORING. The alarm never cleared.
     *
     * Priming makes the average correct from the first sample onward. This is
     * the same defect that was fixed in battery_avg -- see
     * check_for_battery_voltage() -- and the acceleration path is the more
     * damaging place to have it.
     */
    if (!accel_avg_primed) {
        accel_avg_primed = true;
        acceleration_avg_g.fill(accel_value);
    } else {
        acceleration_avg_g.add(accel_value);
    }

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
    Serial.print(F(" im="));
    Serial.print(acc_int_count);

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

    /*
     * Clear the counter and load OCR1A BEFORE the waveform mode and the clock
     * are selected. Two hazards, both observed on the bench 2026-08-06:
     *
     * 1. TCNT1 was never cleared. In mode 9 the counter cannot reach a TOP that
     *    is below it, so if TCNT1 held a large value it had to run all the way
     *    to 0xFFFF and wrap first. At F_CPU/1024 = 976 Hz that is up to 67
     *    seconds before the first compare match.
     *
     * 2. OCR1A is DOUBLE BUFFERED in mode 9. Writing it after the mode bits, as
     *    this function used to, does not take effect until the counter reaches
     *    the *previous* TOP -- so the first period is governed by whatever
     *    OCR1A happened to hold, not by 156. Written here while TCCR1B is still
     *    0 (normal mode), the write is immediate and unbuffered.
     *
     * The symptom was one sensor read at t=5275 and the next at t=72335, then
     * a perfectly regular 3.13 Hz. The 10-second self-calibration in between
     * ran on that single sample: Zero-Calib-Value came out as exactly the one
     * reading taken at t=5275.
     *
     * NOTE: the prescaler and waveform mode are deliberately left as they are.
     * Both are wrong (Eng_Notes §2) and fixing them is roadmap item 5, which
     * changes the sample rate 32x and invalidates every tuning number in the
     * document. This change is only about the timer STARTING predictably, so
     * the rate stays at 3.13 Hz and detection behaviour is unchanged.
     */
    TCNT1 = 0;
    OCR1A = 156;

    // enable overflow interrupt
    TIMSK1 |= (1 << OCIE1A);
    TCCR1B |= (1 << WGM13);

    // enable the internal clock with 1024 prescale -- start it last
    TCCR1B |= ((1 << CS12) | (1 <<CS10));
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

/*
 * Any-motion interrupt handler. Deliberately trivial: no I2C, no serial, no
 * FSM. The sensor's status register is not read here -- that would be an I2C
 * transaction from an ISR, and §6 is what happens when this codebase does I/O
 * from interrupt context. loop() reports the count; the sensor latches its own
 * status until read, so nothing is lost by not reading it immediately.
 */
void acc_int1_isr()
{
    acc_int_count++;
    acc_int_last_ms = millis();
}

/*
 * Report new any-motion edges. Called from loop() only.
 *
 * The buzzer state is printed alongside because §5's mechanical coupling is the
 * main thing that could sink this approach: if bz=1 accompanies most edges, the
 * sensor is detecting its own piezo rather than the car, and the threshold has
 * to discriminate between them -- or it cannot be used while alarming, which is
 * exactly the deadlock the polled version already has.
 */
void emit_acc_int_log()
{
    static uint16_t last_reported = 0;
    uint16_t        count;
    uint32_t        when;

    noInterrupts();
    count = acc_int_count;
    when  = acc_int_last_ms;
    interrupts();

    if (count == last_reported) {
        return;
    }
    last_reported = count;

    Serial.print(F("ACC-INT n="));
    Serial.print(count);
    Serial.print(F(" t="));
    Serial.print(when);
    Serial.print(F(" st="));
    Serial.print(ms.get_state());
    Serial.print(F(" bz="));
    Serial.print(get_buzzer_status() ? 1 : 0);
    Serial.print(F(" pin="));
    Serial.print(digitalRead(PIN_ACC_INT1));
    Serial.print(F("\r\n"));
}

/*
 * Poll and clear the sensor's interrupt status.
 *
 * BMA4_NON_LATCH_MODE alone did not re-arm the pin on the bench: one edge at
 * t=35471 and then nothing across three handling events the polled detector
 * caught easily. Either the mode is being overwritten by the feature-config
 * write or it is not doing what the datasheet implies, so this reads the status
 * register periodically, which clears the condition explicitly and drops the
 * pin so the next motion produces a fresh rising edge.
 *
 * The pin level is logged alongside because it distinguishes the two failure
 * modes at a glance:
 *
 *   pin=1 persistently  -> the interrupt IS asserted and stuck; latching is the
 *                          problem and this poll should fix it
 *   pin=0 with s=0      -> the sensor is not re-asserting at all; the threshold
 *                          is wrong, or any-motion is not actually enabled
 *
 * Rate-limited because each read is a ~6 ms I2C transaction (§10.4).
 */
void poll_acc_int_status()
{
    static uint32_t last_poll = 0;
    uint16_t        status = 0;
    uint16_t        rslt;

    if (millis() - last_poll < ACC_INT_POLL_MS) {
        return;
    }
    last_poll = millis();

    rslt = bma456.readInterruptStatus(&status);

    /*
     * Only speak up when there is something to say: a live status bit, a stuck
     * pin, or a failed read. Otherwise this would add a line every second.
     */
    if (rslt != BMA4_OK) {
        Serial.print(F("ACC-STAT read FAILED\r\n"));
        return;
    }

    if (status != 0 || digitalRead(PIN_ACC_INT1) != 0) {
        Serial.print(F("ACC-STAT s=0x"));
        Serial.print(status, HEX);
        Serial.print(F(" pin="));
        Serial.print(digitalRead(PIN_ACC_INT1));
        Serial.print(F(" n="));
        Serial.print(acc_int_count);
        Serial.print(F("\r\n"));
    }
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
