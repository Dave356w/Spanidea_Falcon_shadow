/*
    A library for Grove - Step Counter(BMA456)

    Copyright (c) 2018 seeed technology co., ltd.
    Author      : Wayen Weng
    Create Time : June 2018
    Change Log  :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "arduino_bma456.h"

#ifdef __AVR__

uint8_t config_file[8] = { 0 };

#endif

/*
 * These two are the Bosch driver's entire view of the bus. Both used to
 * `return 0` unconditionally, which meant BMA4_OK propagated all the way up
 * even when the sensor was not answering at all: bma4_read_accel_xyz() reported
 * success, the caller's buffer kept whatever it already held, and a dead sensor
 * surfaced as a perfectly valid reading of 0.0 -- which the FSM reads as
 * "perfectly still". See Eng_Notes §10.2.
 *
 * Checking the return code further up the stack (as the first attempt did) has
 * no effect while the transport hardcodes success. The check has to be here.
 */
static uint16_t bma_i2c_write(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t len) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    for (uint16_t i = 0; i < len; i++) {
        Wire1.write(data[i]);
    }

    /* 0 = success; 2 = NACK on address, 3 = NACK on data, 4 = other. */
    if (Wire1.endTransmission() != 0) {
        return (BMA4_E_FAIL);
    }

    return (BMA4_OK);
}

static uint16_t bma_i2c_read(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t len) {
    uint16_t i = 0;

    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    if (Wire1.endTransmission() != 0) {
        return (BMA4_E_FAIL);
    }

    Wire1.requestFrom((int16_t)addr, len);
    while (Wire1.available()) {
        data[i++] = Wire1.read();
    }

    /*
     * A short read is a failure. Previously the while loop simply did not
     * execute and the caller was told everything was fine.
     */
    if (i != len) {
        return (BMA4_E_FAIL);
    }

    return (BMA4_OK);
}

static void bma_delay_ms(uint32_t ms) {
    delay(ms);
}

void BMA456::initialize(MA456_RANGE range, MBA456_ODR odr, MA456_BW bw, MA456_PERF_MODE mode) {
    uint16_t    init_status = 0;

    Serial.print("Address :");
    Serial.print(BMA4_I2C_ADDR_PRIMARY);
    Serial.print("\r\n");

    Wire1.begin();
    TWSR1 = 0x00;   // prescaler = 1
    TWBR1 = 0;

//    Wire1.setClock(400000);

//  For breakout board
//   accel.dev_addr        = BMA4_I2C_ADDR_SECONDARY;

//  For inbuilt sensor
    accel.dev_addr        = BMA4_I2C_ADDR_PRIMARY;

    accel.interface       = BMA4_I2C_INTERFACE;
    accel.bus_read        = bma_i2c_read;
    accel.bus_write       = bma_i2c_write;
    accel.delay           = bma_delay_ms;
    accel.read_write_len  = 8;
    accel.resolution      = 16;
    accel.feature_len     = BMA456_FEATURE_SIZE;

    init_status = bma456_init(&accel);

    if (init_status != 0) {
        Serial.print("BMA456-Init Failed :");
        Serial.print(init_status);
        Serial.print("\r\n");
        Serial.print("Chip-ID :");
        Serial.print(accel.chip_id);
        Serial.print("\r\n");
        return;
    } else {
        Serial.print("BMA456 : Initialization done \r\n");
    }

    bma4_set_command_register(0xB6, &accel); // reset device

    delay(500); // wait for POR finish
    Serial.print("BMA456 : Writing configurations .. ");

    bma456_write_config_file(&accel);

    Serial.print(" Done \r\n");
    accel_conf.odr = (uint8_t)odr;
    accel_conf.range = (uint8_t)range;
    accel_conf.bandwidth = (uint8_t)bw;
    accel_conf.perf_mode = (uint8_t)mode;

    bma4_set_accel_config(&accel_conf, &accel);

    if (range == RANGE_2G) {
        devRange = 2000;
    } else if (range == RANGE_4G) {
        devRange = 4000;
    } else if (range == RANGE_8G) {
        devRange = 8000;
    } else if (range == RANGE_16G) {
        devRange = 16000;
    }

    bma4_set_accel_enable(BMA4_ENABLE, &accel);
    Serial.print("BMA456 : Completed configuration \r\n");
}

uint16_t BMA456::configureAnyMotion(uint16_t threshold, uint16_t duration,
                                    uint8_t nomotion_sel, uint8_t int_line) {
    struct bma456_anymotion_config any_cfg;
    struct bma4_int_pin_config     pin_cfg;
    uint16_t                       rslt;

    any_cfg.threshold    = threshold;
    any_cfg.duration     = duration;
    any_cfg.nomotion_sel = nomotion_sel;

    rslt = bma456_set_any_motion_config(&any_cfg, &accel);
    if (rslt != BMA4_OK) {
        return (rslt);
    }

    /*
     * Elevator motion is along Z. X and Y are enabled as well for this first
     * test so that a hand-wave registers -- it makes the interrupt easy to
     * exercise on the bench without a hoistway. Narrow to BMA456_Z_AXIS_EN
     * before drawing any conclusion about real detection performance.
     */
    rslt = bma456_anymotion_enable_axis(BMA456_ALL_AXIS_EN, &accel);
    if (rslt != BMA4_OK) {
        return (rslt);
    }

    /*
     * Electrical behaviour of the INT pin.
     *
     * Edge-triggered active-high suits this test, which never sleeps.
     * SLEEP_MODE_PWR_DOWN stops the I/O clock and can only be woken by a
     * LEVEL-triggered INT0/INT1, so anyone combining this with the sleep work
     * must switch to BMA4_ACTIVE_LOW + BMA4_LEVEL_TRIGGER and attach the
     * handler as LOW. The Anymotion branch's commented-out RISING would never
     * have woken the part. See Eng_Notes §11.
     */
    pin_cfg.edge_ctrl  = BMA4_EDGE_TRIGGER;
    pin_cfg.lvl        = BMA4_ACTIVE_HIGH;
    pin_cfg.od         = BMA4_PUSH_PULL;
    pin_cfg.output_en  = BMA4_OUTPUT_ENABLE;
    pin_cfg.input_en   = BMA4_INPUT_DISABLE;

    rslt = bma4_set_int_pin_config(&pin_cfg, int_line, &accel);
    if (rslt != BMA4_OK) {
        return (rslt);
    }

    /*
     * Non-latched, and this is not optional.
     *
     * In latched mode the pin goes high and STAYS high until the status
     * register is read, so an edge-triggered handler sees exactly one edge and
     * then nothing ever again. That is what the first bench run showed: n=1 at
     * t=61521 and im=1 for the rest of the session. Non-latch lets the sensor
     * clear the pin itself once the condition passes, so every motion event
     * produces a fresh edge with no I2C traffic needed to re-arm.
     */
    rslt = bma4_set_interrupt_mode(BMA4_NON_LATCH_MODE, &accel);
    if (rslt != BMA4_OK) {
        return (rslt);
    }

    rslt = bma456_feature_enable(nomotion_sel ? BMA456_NO_MOTION
                                              : BMA456_ANY_MOTION,
                                 BMA4_ENABLE, &accel);
    if (rslt != BMA4_OK) {
        return (rslt);
    }

    return (bma456_map_interrupt(int_line, BMA456_ANY_NO_MOTION_INT,
                                 BMA4_ENABLE, &accel));
}

uint16_t BMA456::readInterruptStatus(uint16_t *int_status) {
    return (bma456_read_int_status(int_status, &accel));
}

void BMA456::stepCounterEnable(MA456_PLATFORM_CONF conf, bool cmd) {
    bma456_reset_step_counter(&accel);
    bma456_select_platform(conf, &accel);
    bma456_feature_enable(BMA456_STEP_CNTR, cmd, &accel);
}

uint16_t BMA456::getAcceleration(float* x, float* y, float* z) {
    struct bma4_accel sens_data;
    uint16_t          rslt;

    rslt = bma4_read_accel_xyz(&sens_data, &accel);

    if (rslt != BMA4_OK) {
        /*
         * Leave the outputs alone. Writing zeros here is what made a dead
         * sensor indistinguishable from a stationary one.
         */
        return (rslt);
    }

    /*
     * Plausibility check: all three axes reading exactly zero.
     *
     * The transport now reports bus failures, but a sensor that acknowledges
     * and returns zeros -- unconfigured, held in reset, or browning out -- still
     * passes every error check above. That is what was observed on the bench:
     * rd=4800 with a successful read and a=0.0000. Physically, all three axes
     * at exactly zero means sustained free fall in every direction at once,
     * which is not a state an elevator alarm needs to support.
     */
    if (sens_data.x == 0 && sens_data.y == 0 && sens_data.z == 0) {
        return (BMA4_E_FAIL);
    }

    *x = (float)sens_data.x * devRange / 32768;
    *y = (float)sens_data.y * devRange / 32768;
    *z = (float)sens_data.z * devRange / 32768;

    return (BMA4_OK);
}

int32_t BMA456::getTemperature(void) {
    int32_t temp = 0;

    bma4_get_temperature(&temp, BMA4_DEG, &accel);

    return (temp / 1000);
}

uint32_t BMA456::getStepCounterOutput(void) {
    uint32_t step = 0;

    bma456_step_counter_output(&step, &accel);

    return step;
}

BMA456 bma456;
