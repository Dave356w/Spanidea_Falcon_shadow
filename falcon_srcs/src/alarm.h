/*
 ***********************************************************
 * File   : alarm.h
 * Author : Biju Nair
 *
 *
 * Copyright : CreeperNET Consulting 2025-26
 ***********************************************************
 */

#ifndef _ALARM_H_
#define _ALARM_H_

#include <Arduino.h>
#include "common.h"

#define BEEP_FLASH_TIME_MS 300

extern uint8_t alarm_status_g;

void setup_alarm();
void check_for_active_alarm();
void enable_alarm();
void disable_alarm();
uint8_t get_alarm_status();

#endif