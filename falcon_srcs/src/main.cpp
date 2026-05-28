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

uint16_t read_battery_voltage();
extern int configure_adc_channel();
extern uint16_t read_adc_pc2_voltage();

void setup() {

    /*
     * Configure the debug serial port here
    */
    Serial.begin(9600);

    Serial.print("\r\n\nDevice Booted \r\n");
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
    Serial.print("Configured SPI interface \r\n");

    // read_calib_data_from_eeprom();
    /*
     * Configure all Alarm Ports here
    */
    setup_alarm();
    disable_alarm();

    Serial.print("Configured Alarms \r\n");

    digitalWrite(PIN_GREEN_LED, HIGH);
    init_time_g = millis();

    bma456.initialize(RANGE_2G, ODR_100_HZ, NORMAL_AVG4, CONTINUOUS);
    Serial.print("Configured BMA456 \r\n");

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

        Serial.print("Device initialized completely \r\n");
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
    x = y = z = 0;
    bma456.getAcceleration(&x, &y, &z);


    g_value = z / 16384.0;
    accel_value = g_value * 9.81;

#if 1
    Serial.print(" Data : ");
    Serial.print(z, 6);
    Serial.print("  G value : ");
    Serial.print(accel_value, 6);
    Serial.print("\r\n");
#endif

    acceleration_avg_g.add(accel_value);

    return (bosch_acceleration_avg_g.avg());
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
//        Serial.print("Skipping Sensor Read \r\n");
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
    uint16_t  battery_v = 0;

    adc_loop_counter++;

    if (adc_loop_counter == 30000) {
        digitalWrite(BAT_ADC_ENABLE, HIGH);
    }

    if (adc_loop_counter == 30100) {
        adc_loop_counter = 0;
        battery_v = read_adc_pc2_voltage();

        Serial.print("  Voltage value : ");
        Serial.print(battery_v);
        Serial.print("\r\n");

        battery_avg.add(battery_v);
        digitalWrite(BAT_ADC_ENABLE, LOW);

//        if (battery_avg.avg() < 1660) {
        if (battery_v < 1600) {
            Serial.print("  LOW Battery detected \r\n");
            enable_battery_alarm();

        }
    }
}
