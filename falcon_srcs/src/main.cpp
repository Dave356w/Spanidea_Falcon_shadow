/*
 ************************************************************
 *
 * Copyright Spanidea 2024-25
 ************************************************************
 */

#include "main.h"
#include "arduino_bma456.h"

uint16_t x_axis_1, x_axis_2, y_axis_1, y_axis_2, z_axis_1, z_axis_2;
uint8_t sensor_value_updated = 0;
uint8_t alarm_status_g = 0;
uint32_t temp_timer = 0;
uint32_t init_time_g = 0;
float acc_mss_g = 0.0, vel_ms_g = 0.0, adj_acc_g = 0.0;//pressure_g = 0.0, 
SystemStates state = SystemStates::SYSTEM_STATE_INITIALIZING;
static boolean in_isr = false;
RollingAvg<uint32_t> x_acc_avg(64);
RollingAvg<float> pressure_avg_g(32);
RollingAvg<float> acceleration_avg_g(16);
RollingAvg<float> adj_acc_avg_g(16);
RollingAvg<uint16_t> battery_avg(8);
float x = 0, y = 0, z = 0;
float g_value, accel_value;
int32_t  temp = 0;

MCP3208 adc(ADC_VREF, PIN_ADC_CS);
MovementService ms(&acceleration_avg_g, &acc_mss_g, &adj_acc_g, &vel_ms_g, &pressure_avg_g);

Adafruit_DPS310 dps;

#define EN_3_AXIS_SENS 1

uint16_t read_battery_voltage();

void setup() {

    /*
     * Configure the debug serial port here
    */
    Serial.begin(115200);

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

    /*
     * Configure all Alarm Ports here
    */
    setup_alarm();
    disable_alarm();

    Serial.print("Configured Alarms \r\n");

    init_pressure_sensor();

    digitalWrite(PIN_GREEN_LED, HIGH);
    init_time_g = millis();
    Serial.print("Device Booted \r\n");

    bma456.initialize(RANGE_2G, ODR_100_HZ, NORMAL_AVG4, CONTINUOUS);
}

void initialization()
{

    /*
     * One device boot-up, read the acceleration data  for
     * some time.
     */

    if (millis() - temp_timer > INIT_TIMER_MS) {

        temp_timer = millis();
        acceleration_avg_g.add(read_acceleration_mss());
        read_pressure();
        battery_avg.add(read_battery_voltage());
    }

    if (millis() - init_time_g > INIT_TIME_MS) {

        adj_acc_g = acceleration_avg_g.avg() * -1;
        acceleration_avg_g.fill(read_acceleration_mss() + adj_acc_g);
        adj_acc_avg_g.fill(acceleration_avg_g.avg());

        state = SystemStates::SYSTEM_STATE_NOMINAL;
        enable_timer1();

        digitalWrite(PIN_PIEZO, HIGH);  
        for (int i = 0; i < 8; i++) {
            digitalWrite(PIN_CHASE_CLK, HIGH);
            digitalWrite(PIN_CHASE_CLK, LOW);
            delay(100);   
        }

        Serial.print("Transitioning to Normal Operation \r\n");
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
#if 0
    if (sensor_value_updated) {
        log_data();
        sensor_value_updated = 0;
    }
#endif
    switch (state) {

    case SystemStates::SYSTEM_STATE_INITIALIZING:
        initialization();
        break;

    case SystemStates::SYSTEM_STATE_NOMINAL:
//        ms.run();
        if (bma_read_counter++ > 100 ) {
            bma456.getAcceleration(&x, &y, &z);
            temp = bma456.getTemperature();
            g_value = z / 16384.0;
            accel_value = g_value * 9.81;
            log_data();
            bma_read_counter = 0;
        }

        check_for_battery_voltage();
        alarm_service();
        break;

    default:
        state = SystemStates::SYSTEM_STATE_HOLD;
        break;
    }

}


// read z-axis accelerometer data and convert to m/(s*s)
float read_acceleration_mss()
{

#ifdef EN_3_AXIS_SENS
    z_axis_1 = adc.read(Z1);
    z_axis_2 = adc.read(Z2);

    x_acc_avg.add(z_axis_2);
    float acc_mss = ((((float)x_acc_avg.avg() / 4096.0) * 3.3) - 1.65) / 0.440;//(x_acc_avg.avg() * 0.001220703) - 2.5;
    acc_mss = (acc_mss * 9.80665) - 9.80665;

  // (((((float)x_acc_avg.avg() / 4096.0) * 3.3) - 1.65) / 0.440) * ( 9.80665) - 9.80665
  // 0.440 --> sensitivity 440 mV/g
  // 0.017956516 * x_acc_avg - 46.5815875

#else

    z_axis_1 = adc.read(EW_ACC_Y);
    z_axis_2 = adc.read(NS_ACC_Y);

    x_acc_avg.add(((z_axis_1+z_axis_2) / 2));
    float acc_mss = (x_acc_avg.avg() * 0.001220703) - 2.5;
    acc_mss = (acc_mss * 9.80665);

    if (acc_mss > 0) {
        acc_mss -= 9.80665;
    } else {
        acc_mss += 9.80665;
    }

#endif

    return acc_mss;
}

void enable_timer1()
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

  // x_axis_1 = adc.read(PCB_ACC_X);
  // x_axis_2 = adc.read(EW_ACC_X);
  // y_axis_1 = adc.read(PCB_ACC_Y);
  // y_axis_2 = adc.read(NS_ACC_X);

    // float acc_mss = read_acceleration_mss();
    acc_mss_g = (read_acceleration_mss());// + acc_mss_g) / 2;
    acceleration_avg_g.add(acc_mss_g + adj_acc_g);

    if ((ms.state >=  MotionStates::MOVEMENT_DETECTED) && 
        (ms.state < MotionStates::DECELERATING) )
    {
        vel_ms_g = vel_ms_g + (acceleration_avg_g.avg()  * 0.02);
    }

    if ((ms.state == MotionStates::NOT_MOVING || ms.state == MotionStates::ERROR_RESET)  
        && fabs(acceleration_avg_g.avg()) < 0.04) 
    {
        adj_acc_avg_g.add(acceleration_avg_g.avg());

        if (adj_acc_avg_g.get(0) == adj_acc_avg_g.get(adj_acc_avg_g.size() - 1)) 
        {
            adj_acc_g = acc_mss_g * -1;
        }
    }

    read_pressure();

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
    char log_con[100];
    static int loop_cnt = 0;
    static int index = 1;

    if (loop_cnt++ > 100) {
        Serial.print("\r\n Index: ");
        Serial.print(index);
        Serial.print(" X: ");
        Serial.print(x);
        Serial.print(" Y: ");
        Serial.print(y);
        Serial.print(" Z: ");
        Serial.print(z);
        Serial.print(" Acceleration: ");
        Serial.print(accel_value);
        Serial.print(" Temp: ");
        Serial.print(temp);

        loop_cnt = 0; 
        index++;
    }

#if 0
    Serial.print((double)acc_mss_g, 5);
    Serial.print(", ");
    Serial.print((double)acceleration_avg_g.avg(), 5);
    Serial.print(", ");
    Serial.print((double)adj_acc_g, 5);
    Serial.print(", ");
    Serial.print((double)vel_ms_g, 5);
    Serial.print(", ");
    Serial.print((double)ms.vel_threshold, 5);
    Serial.print(", ");
    Serial.print(ms.state);
    Serial.print(", ");
    Serial.print(alarm_status_g);
    Serial.print(", ");
    Serial.print((double)ms.variance_acc, 5);
    Serial.print(", ");
    Serial.print(pressure_avg_g.avg());
    Serial.print(", ");
    Serial.print((double)ms.variance_pres, 5);
    Serial.print(", ");
    Serial.println(battery_avg.avg());
#endif
}

void debug_log(char *p_log)
{
    Serial.print(p_log);

}

/*
 * Initialize the pressure sensor here
*/
void init_pressure_sensor()
{
    if (! dps.begin_I2C()) {             // Can pass in I2C address here
        Serial.println("Failed to find DPS");
//        while (1) yield();
    }

    Serial.println("DPS Pressure Sensor OK!");
#if 0
    dps.configurePressure(DPS310_32HZ, DPS310_32SAMPLES);
    dps.configureTemperature(DPS310_32HZ, DPS310_32SAMPLES);
#endif
}

uint8_t read_pressure()
{
    static sensors_event_t pressure_event;
#if 0
    if (dps.pressureAvailable()) {
        dps.getEvents(NULL, &pressure_event);
        // pressure_g = pressure_event.pressure * 100.0;
        pressure_avg_g.add(pressure_event.pressure * 10.0);

        return 0;
    }
#endif
    return 1;
}

inline uint16_t read_battery_voltage() 
{

    float vol_temp = (((float) adc.read(BATT_SENSE) / 4096.0) * 3300) * (VBATT_CONST) * (1.06);

  // t1=r4 tolerance=5%, t2=r5 tolerance = 1%,  vd(r4)=voltage drop of r4=2.774, vd(r5)=voltage drop of r5=0.815
  // (adc_raw)*(VBATT_CONST)*(1+(t1+t2/100))+vd(r5)/vd(r4)
    return (uint16_t)vol_temp;
}

/*
 * This function will check for battery-voltage level and notify the user
 */
void check_for_battery_voltage() 
{
    static unsigned long status_timer = 0;

    if(ms.state != MotionStates::NOT_MOVING || alarm_status_g == 1) {
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

    } else if (battery_avg.avg() > VOLTAGE_LOW && battery_avg.avg() < VOLTAGE_THRESHOLD) {

        /*
         * If the Battery voltage is between 3.0V and 3.7V)
         */

        digitalWrite(PIN_RED_LED, HIGH);
        digitalWrite(PIN_PIEZO, HIGH);
        delay(BATTERY_BLINK_DURATION_MS);
        digitalWrite(PIN_RED_LED, LOW);
        digitalWrite(PIN_PIEZO, LOW);

    } else {

        /*
         * If the Battery voltage is < 3.0V
         */


        digitalWrite(PIN_RED_LED, HIGH);
        digitalWrite(PIN_PIEZO, HIGH);
    }
}
