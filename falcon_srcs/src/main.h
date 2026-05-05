/*
 ***********************************************************
 * File   : main.h
 * Author : Biju Nair
 *
 *
 * Copyright : CreeperNET Consulting 2025-26
 ***********************************************************
 */

 #ifndef _MAIN_H_
 #define _MAIN_H_

#include <Arduino.h>
#include <RollingAvg.h>
#include <Mcp320x.h>
#include <SPI.h>
#include <Adafruit_DPS310.h>
#include "movement_service.h"
#include "alarm.h"

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

#define Z1                            MCP3208::Channel::SINGLE_5
#define Z2                            MCP3208::Channel::SINGLE_2


#define ADC_VREF    3300     // 3.3V Vref
#define ADC_CLK     1000000  // SPI clock 1MHz

#define INIT_TIMER_MS 20
#define INIT_TIME_MS 80
#define BATTERY_BLINK_PERIOD_MS           1000  // time the RGB LED waits to blink to indicate the battery state
#define BATTERY_BLINK_DURATION_MS         25    // time the RGB LED blinks on to indicate battery state

#define HEALTH_CHECK_INTERVAL_MS  100

#define BATT_R1                           5100.0
#define BATT_R2                           1500.0
#define VBATT_CONST  ((BATT_R1 + BATT_R2)/BATT_R2)
#define VBATT_DIODE_DROP                  0
#define VOLTAGE_THRESHOLD                 3700
#define VOLTAGE_LOW                       3400


void enable_timer();
void disable_timer1();
float read_acceleration_mss();
uint8_t read_pressure();
void check_for_battery_voltage();
void log_data();
void init_pressure_sensor();
uint16_t read_adc_pc2();
uint16_t read_adc_pc2_voltage();


enum SystemStates
{
  SYSTEM_STATE_INITIALIZING = 0,
  SYSTEM_STATE_NOMINAL,             // store for normal operation
  SYSTEM_STATE_HOLD,                    // intermediate state after an alarm to shut alarm down before transitioning to NOMINAL
  SYSTEM_HEALTH_CRITICAL_SHUTDOWN  // state used to shut down due to some critical health check error
};

#endif