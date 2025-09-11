#ifndef __MAIN_H__
#define __MAIN_H__

#include <inttypes.h>
#include "RollingAvg.h"
#include <math.h> 
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "common.h"

#define NUM_AVGS    32


#define PIN_ADC_CS                          14 //PC0 - ADC Chip Select
#define PIN_GREEN_LED                       16 //PC2 - Center LED
#define PIN_RGB_BLUE                        17 //PC3 - NA
#define ARD_SDA_PIN                         18 //PC4 - I2C Data
#define ARD_SCL_PIN                         19 //PC5 - I2C Clock

#define PIN_RX                              0 //PD0  - Debug UART Rx
#define PIN_TX                              1 //PD1  - Debug UART Tx
#define PIN_PSI_INT                         3 //PD3  - Pressure Sensor Interrupt
#define PIN_RGB_RED                         4 //PD4  - NA
#define PIN_CHASE_LED                       6 //PD6  - Circular LED's (MRST)
#define PIN_CHASE_CLK                       7 //PD7a - Clock for LED

#define PIN_PIEZO                           8 //PB0
#define PIN_RED_LED                         9 //PB1
#define PIN_CPWM                            10 //PB2

#define PIN_TOUT_NS                         ADC6
#define PIN_TOUT_EW                         ADC7

//  ADC Channel definitions
#define PCB_ACC_TOUT                        MCP3208::Channel::SINGLE_0
#define BATT_SENSE                          MCP3208::Channel::SINGLE_1
#define PCB_ACC_Y                           MCP3208::Channel::SINGLE_2
#define PCB_ACC_X                           MCP3208::Channel::SINGLE_3
#define EW_ACC_Y                            MCP3208::Channel::SINGLE_4
#define EW_ACC_X                            MCP3208::Channel::SINGLE_5
#define NS_ACC_Y                            MCP3208::Channel::SINGLE_6
#define NS_ACC_X                            MCP3208::Channel::SINGLE_7


#define ADC_VREF    3300     // 3.3V Vref
#define ADC_CLK     1000000  // SPI clock 1MHz

#define INIT_TIMER_MS 20
#define INIT_TIME_MS 3000
#define BEEP_FLASH_TIME_MS 70

enum SystemStates {
    SYSTEM_STATE_INITIALIZING = 0,
    SYSTEM_STATE_NOMINAL,
    SYSTEM_STATE_HOLD
};




float read_acceleration_mss();
uint8_t read_pressure(float *pressure);
void enable_timer1();
void disable_timer1();
void monitor_service();

#endif
