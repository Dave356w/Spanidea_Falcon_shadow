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
eeprom_db eeprom_db_g;

MCP3208 adc(ADC_VREF, PIN_ADC_CS);
MovementService ms(&acceleration_avg_g, &acc_mss_g, &adj_acc_g, &vel_ms_g, &pressure_avg_g);

#define EN_3_AXIS_SENS 1

uint16_t read_battery_voltage();

float get_threshold_data()
{
    /*
     * This function will return the calibration value read from EEPROM. This value will be used for self-calibration of the device.
     */
    return eeprom_db_g.thresh_value;
}

int read_calib_data_from_eeprom()
{
    /*
     * Read the calibration data from EEPROM and populate the global variable
     * eeprom_db_g. This will be used for self-calibration of the device.
     */
    
    // Define EEPROM start address
    const int EEPROM_ADDR = 0;
    
    // Define magic number for validation
    const uint32_t EEPROM_MAGIC = 0xDEADBEEF;
    
    // Read magic number (4 bytes)
    eeprom_db_g.magic = EEPROM.read(EEPROM_ADDR);
    eeprom_db_g.magic |= (uint32_t)EEPROM.read(EEPROM_ADDR + 1) << 8;
    eeprom_db_g.magic |= (uint32_t)EEPROM.read(EEPROM_ADDR + 2) << 16;
    eeprom_db_g.magic |= (uint32_t)EEPROM.read(EEPROM_ADDR + 3) << 24;
   
    /*
     * First check whether we have a valid data in EEPROM. If not, then populate it with 
     * a default value and write this information back to EEPROM. This will ensure that we 
     * have a valid data in EEPROM for subsequent reads, and also help us to detect any 
     * EEPROM corruption in the future.
    */
    if (eeprom_db_g.magic != EEPROM_MAGIC) {
        Serial.print("EEPROM: Invalid magic number found. \r\n");
        // Initialize with default values if magic is invalid
        eeprom_db_g.magic = EEPROM_MAGIC;
        eeprom_db_g.version = 1;
        eeprom_db_g.thresh_value = 0.01;

        // Write default values back to EEPROM
        EEPROM.write(EEPROM_ADDR, (uint8_t)(eeprom_db_g.magic & 0xFF));
        EEPROM.write(EEPROM_ADDR + 1, (uint8_t)((eeprom_db_g.magic >> 8) & 0xFF));
        EEPROM.write(EEPROM_ADDR + 2, (uint8_t)((eeprom_db_g.magic >> 16) & 0xFF));
        EEPROM.write(EEPROM_ADDR + 3, (uint8_t)((eeprom_db_g.magic >> 24) & 0xFF));
        EEPROM.write(EEPROM_ADDR + 4, (uint8_t)(eeprom_db_g.version & 0xFF));
        EEPROM.write(EEPROM_ADDR + 5, (uint8_t)((eeprom_db_g.version >> 8) & 0xFF));
        EEPROM.write(EEPROM_ADDR + 6, (uint8_t)((eeprom_db_g.version >> 16) & 0xFF));
        EEPROM.write(EEPROM_ADDR + 7, (uint8_t)((eeprom_db_g.version >> 24) & 0xFF));
        uint8_t *float_bytes = (uint8_t*)&eeprom_db_g.thresh_value;
        for (int i = 0; i < 4; i++) {   
            EEPROM.write(EEPROM_ADDR + 8 + i, float_bytes[i]);
        }       

        Serial.print("EEPROM: Default calibration saved into EEPROM. \r\n");
        return (0);
    }

    // Read version (4 bytes)
    eeprom_db_g.version = EEPROM.read(EEPROM_ADDR + 4);
    eeprom_db_g.version |= (uint32_t)EEPROM.read(EEPROM_ADDR + 5) << 8;
    eeprom_db_g.version |= (uint32_t)EEPROM.read(EEPROM_ADDR + 6) << 16;
    eeprom_db_g.version |= (uint32_t)EEPROM.read(EEPROM_ADDR + 7) << 24;
    
    // Read calibration_value (4 bytes as float)
    uint8_t float_bytes[4];
    for (int i = 0; i < 4; i++) {
        float_bytes[i] = EEPROM.read(EEPROM_ADDR + 8 + i);
    }
    // Copy bytes into float
    memcpy(&eeprom_db_g.thresh_value, float_bytes, sizeof(float));
    
    // Validate magic number
    if (eeprom_db_g.magic != EEPROM_MAGIC) {
        Serial.print("EEPROM: ERROR Invalid magic number. \r\n");
        return -1;  // Return -1 to indicate invalid EEPROM data
    }
    
    Serial.print("EEPROM: Successfully read calibration data \r\n");
    Serial.print("  Version: ");
    Serial.print(eeprom_db_g.version);
    Serial.print(", Calibration value: ");
    Serial.print(eeprom_db_g.thresh_value, 6);
    Serial.print("\r\n");
    
    return 0;  // Return 0 on success
}

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
    uint16_t voltage_mv = (uint32_t)adc_result * 3300 / 1023;
    
    Serial.print("  Voltage value : ");
    Serial.print(voltage_mv);
    Serial.print("\r\n");
    return voltage_mv;
}

void setup() {

    /*
     * Configure the debug serial port here
    */
    Serial.begin(9600);

    Serial.print("\r\n\nDevice Booted \r\n");
    /*
     * Configure the ADC chip-select line here
    */
    pinMode(PIN_ADC_CS, OUTPUT);
    digitalWrite(PIN_ADC_CS, HIGH);

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
        battery_avg.add(read_battery_voltage());
    }

    if (millis() - init_time_g > INIT_TIME_MS) {

//        adj_acc_g = acceleration_avg_g.avg() * -1;
//        acceleration_avg_g.fill(read_acceleration_mss() + adj_acc_g);

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
    static unsigned int adc_loop_counter = 0;

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

//        check_for_battery_voltage();
        adc_loop_counter++;

        if (adc_loop_counter == 10000) {
            digitalWrite(BAT_ADC_ENABLE, HIGH);
        }

        if (adc_loop_counter == 10100) {
            adc_loop_counter = 0;
            read_adc_pc2_voltage();
            digitalWrite(BAT_ADC_ENABLE, LOW);
        }

        check_for_active_alarm();
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

    Serial.print(" Data : ");
#if 0
    Serial.print(x, 6);
    Serial.print(" ");
    Serial.print(y, 6);
    Serial.print(" ");
#endif
    Serial.print(z, 6);

    Serial.print("  G value : ");
    Serial.print(accel_value, 6);
    Serial.print("\r\n");

    acceleration_avg_g.add(accel_value);

    return (bosch_acceleration_avg_g.avg());
}
#if 0
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
#if 1
    if (get_alarm_status()) {
        Serial.print("Skipping Sensor Read \r\n");
//        acceleration_avg_g.fill(1);
    } else {
        read_acceleration_mss();
    }
#endif

//    read_acceleration_mss();

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
#if 0
        Serial.print("\r\n Index: ");
        Serial.print(index);
        Serial.print(" X: ");
        Serial.print(x);
        Serial.print(" Y: ");
        Serial.print(y);
        Serial.print(" Z: ");
        Serial.print(z);
        Serial.print("\r\n Accl: ");
        Serial.print(acceleration_avg_g.avg());

#endif
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
    static unsigned long status_timer = 0;

    if(ms.state != MotionStates::STATE_NOT_MOVING || alarm_status_g == 1) {
        return ;
    }

    if (millis() - status_timer < 1000) {
        return; 
    }

    status_timer = millis(); 
    uint16_t battery_voltage = read_battery_voltage();
    battery_avg.add(battery_voltage);

    if (battery_avg.avg() > VOLTAGE_THRESHOLD) {

        /*
         * If the battery voltage is > 3.7 v
         */

        digitalWrite(PIN_GREEN_LED, HIGH);   
        delay(BATTERY_BLINK_DURATION_MS);     
        digitalWrite(PIN_GREEN_LED, LOW); 

    } else if ((battery_avg.avg() > VOLTAGE_LOW) && 
               (battery_avg.avg() < VOLTAGE_THRESHOLD)) 
    {

        /*
         * If the Battery voltage is between 3.0V and 3.7V)
         */

//        digitalWrite(PIN_RED_LED, HIGH);
//        digitalWrite(PIN_PIEZO, HIGH);
        delay(BATTERY_BLINK_DURATION_MS);
//        digitalWrite(PIN_RED_LED, LOW);
//        digitalWrite(PIN_PIEZO, LOW);

    } else 
    {
        /*
         * If the Battery voltage is < 3.0V
         */

//        digitalWrite(PIN_RED_LED, HIGH);
//        digitalWrite(PIN_PIEZO, HIGH);
    }
}
