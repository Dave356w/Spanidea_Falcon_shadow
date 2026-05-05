/*
 ***********************************************************
 * File   : eeprom.cpp
 * Author : Biju Nair
 *
 *
 * Copyright : CreeperNET Consulting 2025-26
 ***********************************************************
 */

#include "main.h"
#include "arduino_bma456.h"
#include "common.h"
#include <EEPROM.h>

eeprom_db eeprom_db_g;

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

