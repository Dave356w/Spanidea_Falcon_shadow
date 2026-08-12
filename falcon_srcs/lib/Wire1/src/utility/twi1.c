/*
  twi.c - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
  Modified 2020 by Greyson Christoforo (grey@christoforo.net) to implement timeouts
*/

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <compat/twi.h>
#include "Arduino.h" // for digitalWrite and micros
#include "Wire_timeout.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "pins_arduino.h"
#include "twi1.h"

static volatile uint8_t twi_state;
static volatile uint8_t twi_slarw;
static volatile uint8_t twi_sendStop; // should the transaction end with a stop
static volatile uint8_t twi_inRepStart; // in the middle of a repeated start

// twi_timeout_us > 0 prevents the code from getting stuck in various while loops here
// if twi_timeout_us == 0 then timeout checking is disabled (the previous Wire lib behavior)
// at some point in the future, the default twi_timeout_us value could become 25000
// and twi_do_reset_on_timeout could become true
// to conform to the SMBus standard
// http://smbus.org/specs/SMBus_3_1_20180319.pdf
static volatile uint32_t twi_timeout_us = 0ul;
static volatile bool twi_timed_out_flag = false;  // a timeout has been seen
static volatile bool twi_do_reset_on_timeout = false;  // reset the TWI registers on timeout

/*
 * ─── FALCON LOCAL MODIFICATION: BOUNDED WAITS ────────────────────────────────
 *
 * WHY THIS LIBRARY IS VENDORED. Every wait loop below is unbounded upstream
 * unless WIRE_TIMEOUT is defined, and the BMA456 sample read runs inside the
 * 25 Hz timer ISR. A bus glitch wedges the TWI state machine, the wait never
 * returns, the ISR never returns, and the device freezes solid -- serial
 * silent, single LED lit, only an external reset recovers. Observed 5 times on
 * 2026-08-12, and on 2026-08-12 evening a pair of them ate an entire 2->1 car
 * run: the watchdog reset mid-descent, the FSM came up in recalibration, and
 * no departure was ever latched. That is a beacon that does not sound.
 *
 * WHY NOT -DWIRE_TIMEOUT. It costs 1554 bytes (measured, with -Wl,--relax)
 * against 356 free, because it compares 32-bit micros() at seven sites AND
 * adds three function pointers to the shared TwoWire constructor -- so it
 * cannot be enabled for this library alone without an ABI mismatch against
 * the framework's Wire.cpp. See platformio.ini.
 *
 * WHAT THIS DOES INSTEAD. A 16-bit spin counter per wait site. No timebase, no
 * 32-bit arithmetic, no change to TwoWire. On expiry it calls
 * twi_handleTimeout1(true), which resets the TWI peripheral and reapplies
 * TWBR1/TWAR1, then returns the normal error code -- the same recovery
 * upstream performs, reached far more cheaply.
 *
 * ⚠️ TWI1_GUARD_SPINS IS A SPIN COUNT, NOT A TIME. It does not track F_CPU or
 * the TWI bit rate, so it must be re-checked if either changes. At F_CPU
 * 1 MHz a spin is roughly 6-8 cycles, so 8000 spins is on the order of 50 ms
 * against a normal transaction of ~11.5 ms -- about 4x headroom, chosen to be
 * far longer than any healthy transaction and far shorter than the 2 s
 * watchdog. The guard is a LAST RESORT BELOW THE WATCHDOG, not a bus-error
 * handler: if it trips regularly, the bus is the problem, not the bound.
 *
 * Set TWI1_GUARD_SPINS to 0 to compile the guards out and restore upstream
 * unbounded behaviour -- the rollback, matching the arming discipline the rest
 * of this firmware uses.
 *
 * twi1_guard_trips1() exposes a saturating count so main.cpp can log whether
 * the guard ever fired. A trip is DATA: it means a wedge was caught that would
 * previously have become a watchdog reset.
 */
#ifndef TWI1_GUARD_SPINS
#define TWI1_GUARD_SPINS 8000u
#endif

#if TWI1_GUARD_SPINS > 0

static volatile uint8_t twi1_guard_trip_count = 0;

/*
 * Deliberately not inlined: one shared body keeps the per-site cost down to a
 * call, which is the whole point of this modification.
 */
static void __attribute__((noinline)) twi1_guard_tripped(void)
{
  if (twi1_guard_trip_count < 255) {
    ++twi1_guard_trip_count;
  }
  twi_handleTimeout1(true);
}

uint8_t twi1_guard_trips1(void)
{
  return twi1_guard_trip_count;
}

/* Bounded replacement for `while (cond) continue;`. */
#define TWI1_WAIT(cond, onfail)                        \
  do {                                                 \
    uint16_t twi1_g = (uint16_t)(TWI1_GUARD_SPINS);     \
    while (cond) {                                     \
      if (--twi1_g == 0u) {                            \
        twi1_guard_tripped();                          \
        onfail;                                        \
      }                                                \
    }                                                  \
  } while (0)

/* Bounded replacement for the TWWC do/while, which must write TWDR1 each pass. */
#define TWI1_WAIT_TWWC(onfail)                         \
  do {                                                 \
    uint16_t twi1_g = (uint16_t)(TWI1_GUARD_SPINS);     \
    do {                                               \
      TWDR1 = twi_slarw;                               \
      if (--twi1_g == 0u) {                            \
        twi1_guard_tripped();                          \
        onfail;                                        \
      }                                                \
    } while (TWCR1 & _BV(TWWC));                       \
  } while (0)

#else /* guards compiled out -- upstream unbounded behaviour */

uint8_t twi1_guard_trips1(void) { return 0; }

#define TWI1_WAIT(cond, onfail)   do { while (cond) { continue; } } while (0)
#define TWI1_WAIT_TWWC(onfail)    do { do { TWDR1 = twi_slarw; } while (TWCR1 & _BV(TWWC)); } while (0)

#endif /* TWI1_GUARD_SPINS */

static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(uint8_t*, int);

static uint8_t *twi_masterBuffer;
static volatile uint8_t twi_masterBufferIndex;
static volatile uint8_t twi_masterBufferLength;

static uint8_t twi_txBuffer[TWI1_BUFFER_SIZE];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;

static uint8_t twi_rxBuffer[TWI1_BUFFER_SIZE];
static volatile uint8_t twi_rxBufferIndex;

static volatile uint8_t twi_error;

/*
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init1(void)
{
  // initialize state
  twi_state = TWI_READY;
  twi_sendStop = true; // default value
  twi_inRepStart = false;

  // activate internal pullups for twi.
  digitalWrite(SDA1, 1);
  digitalWrite(SCL1, 1);

  // initialize twi prescaler and bit rate
  cbi(TWSR1, TWPS0);
  cbi(TWSR1, TWPS1);
  TWBR1 = ((F_CPU / TWI_FREQ) - 16) / 2;

  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
  note: TWBR1 should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */

  // enable twi module, acks, and twi interrupt
  TWCR1 = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

/*
 * Function twi_disable
 * Desc     disables twi pins
 * Input    none
 * Output   none
 */
void twi_disable1(void)
{
  // disable twi module, acks, and twi interrupt
  TWCR1 &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));

  // deactivate internal pullups for twi.
  digitalWrite(SDA1, 0);
  digitalWrite(SCL1, 0);
}

/*
 * Function twi_slaveInit
 * Desc     sets slave address and enables interrupt
 * Input    none
 * Output   none
 */
void twi_setAddress1(uint8_t address)
{
  // set twi slave address (skip over TWGCE bit)
  TWAR1 = address << 1;
}

/*
 * Function twi_setFrequency
 * Desc     sets twi bit rate
 * Input    Clock frequency
 * Output   none
 */
void twi_setFrequency1(uint32_t frequency)
{
  TWBR1 = ((F_CPU / frequency) - 16) / 2;

  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR1))
  note: TWBR1 should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */
}

/*
 * Function twi_readFrom
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes to read into array
 *          sendStop: Boolean indicating whether to send a stop at the end
 * Output   number of bytes read
 */
uint8_t twi_readFrom1(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
  // ensure data will fit into buffer
  if(TWI1_BUFFER_SIZE < length){
    return 0;
  }

  // wait until twi is ready, become master receiver
  TWI1_WAIT(TWI_READY != twi_state, return 0);
  twi_state = TWI_MRX;
  twi_sendStop = sendStop;
  // reset error state (0xFF.. no error occurred)
  twi_error = 0xFF;

  // initialize buffer iteration vars
  twi_masterBuffer = data;
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length-1;  // This is not intuitive, read on...
  // On receive, the previously configured ACK/NACK setting is transmitted in
  // response to the received byte before the interrupt is signalled. 
  // Therefore we must actually set NACK when the _next_ to last byte is
  // received, causing that NACK to be sent in response to receiving the last
  // expected byte of data.

  // build sla+w, slave device address + w bit
  twi_slarw = TW_READ;
  twi_slarw |= address << 1;

  if (true == twi_inRepStart) {
    // if we're in the repeated start state, then we've already sent the start,
    // (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
    // We need to remove ourselves from the repeated start state before we enable interrupts,
    // since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
    // up. Also, don't enable the START interrupt. There may be one pending from the
    // repeated start that we sent ourselves, and that would really confuse things.
    twi_inRepStart = false; // Remember, we're dealing with an ASYNC ISR
    TWI1_WAIT_TWWC(return 0);
    TWCR1 = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE); // enable INTs, but not START
  }
  else
    // send start condition
    TWCR1 = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

  // wait for read operation to complete
  TWI1_WAIT(TWI_MRX == twi_state, return 0);

  if (twi_masterBufferIndex < length)
    length = twi_masterBufferIndex;

  return length;
}

/*
 * Function twi_writeTo
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 *          sendStop: boolean indicating whether or not to send a stop at the end
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 *          5 .. timeout
 */
uint8_t twi_writeTo1(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
  // ensure data will fit into buffer
  if(TWI1_BUFFER_SIZE < length){
    return 1;
  }

  // wait until twi is ready, become master transmitter
  TWI1_WAIT(TWI_READY != twi_state, return 5);

  twi_state = TWI_MTX;
  twi_sendStop = sendStop;
  // reset error state (0xFF.. no error occurred)
  twi_error = 0xFF;

  // initialize buffer iteration vars
  twi_masterBuffer = data;
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length;

  // build sla+w, slave device address + w bit
  twi_slarw = TW_WRITE;
  twi_slarw |= address << 1;

  // if we're in a repeated start, then we've already sent the START
  // in the ISR. Don't do it again.
  //
  if (true == twi_inRepStart) {
    // if we're in the repeated start state, then we've already sent the start,
    // (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
    // We need to remove ourselves from the repeated start state before we enable interrupts,
    // since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
    // up. Also, don't enable the START interrupt. There may be one pending from the 
    // repeated start that we sent ourselves, and that would really confuse things.
    twi_inRepStart = false; // Remember, we're dealing with an ASYNC ISR
    TWI1_WAIT_TWWC(return 5);
    TWCR1 = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE); // enable INTs, but not START
  }
  else
    // send start condition
    TWCR1 = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA); // enable INTs

  // wait for write operation to complete
  TWI1_WAIT(wait && (TWI_MTX == twi_state), return 5);

  if (twi_error == 0xFF)
    return 0; // success
  else if (twi_error == TW_MT_SLA_NACK)
    return 2; // error: address send, nack received
  else if (twi_error == TW_MT_DATA_NACK)
    return 3; // error: data send, nack received
  else
    return 4; // other twi error
}

/*
 * Function twi_transmit
 * Desc     fills slave tx buffer with data
 *          must be called in slave tx event callback
 * Input    data: pointer to byte array
 *          length: number of bytes in array
 * Output   1 length too long for buffer
 *          2 not slave transmitter
 *          0 ok
 */
uint8_t twi_transmit1(const uint8_t* data, uint8_t length)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI1_BUFFER_SIZE < (twi_txBufferLength+length)){
    return 1;
  }

  // ensure we are currently a slave transmitter
  if(TWI_STX != twi_state){
    return 2;
  }

  // set length and copy data into tx buffer
  for(i = 0; i < length; ++i){
    twi_txBuffer[twi_txBufferLength+i] = data[i];
  }
  twi_txBufferLength += length;

  return 0;
}

/*
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveRxEvent1( void (*function)(uint8_t*, int) )
{
  twi_onSlaveReceive = function;
}

/*
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveTxEvent1( void (*function)(void) )
{
  twi_onSlaveTransmit = function;
}

/*
 * Function twi_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
void twi_reply1(uint8_t ack)
{
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR1 = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{
    TWCR1 = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

/*
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void twi_stop1(void)
{
  // send stop condition
  TWCR1 = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  // wait for stop condition to be executed on bus
  // TWINT is not set after a stop condition!
  //
  // This site is reached from the TWI ISR, so it could never use micros()
  // upstream either -- it fell back to _delay_us and 32-bit division. The spin
  // counter is both cheaper and ISR-safe. On a trip, twi1_guard_tripped()
  // re-inits the peripheral, which itself sets twi_state = TWI_READY, so the
  // early return does not leave the state machine wedged.
  TWI1_WAIT(TWCR1 & _BV(TWSTO), return);

  // update twi state
  twi_state = TWI_READY;
}

/*
 * Function twi_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
void twi_releaseBus1(void)
{
  // release bus
  TWCR1 = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

  // update twi state
  twi_state = TWI_READY;
}

/* 
 * Function twi_setTimeoutInMicros
 * Desc     set a timeout for while loops that twi might get stuck in
 * Input    timeout value in microseconds (0 means never time out)
 * Input    reset_with_timeout: true causes timeout events to reset twi
 * Output   none
 */
void twi_setTimeoutInMicros1(uint32_t timeout, bool reset_with_timeout){
  twi_timed_out_flag = false;
  twi_timeout_us = timeout;
  twi_do_reset_on_timeout = reset_with_timeout;
}

/* 
 * Function twi_handleTimeout
 * Desc     this gets called whenever a while loop here has lasted longer than
 *          twi_timeout_us microseconds. always sets twi_timed_out_flag
 * Input    reset: true causes this function to reset the twi hardware interface
 * Output   none
 */
void twi_handleTimeout1(bool reset){
  twi_timed_out_flag = true;

  if (reset) {
    // remember bitrate and address settings
    uint8_t previous_TWBR = TWBR1;
    uint8_t previous_TWAR = TWAR1;

    // reset the interface
    twi_disable1();
    twi_init1();

    // reapply the previous register values
    TWAR1 = previous_TWAR;
    TWBR1 = previous_TWBR;
  }
}

/*
 * Function twi_manageTimeoutFlag
 * Desc     returns true if twi has seen a timeout
 *          optionally clears the timeout flag
 * Input    clear_flag: true if we should reset the hardware
 * Output   none
 */
bool twi_manageTimeoutFlag1(bool clear_flag){
  bool flag = twi_timed_out_flag;
  if (clear_flag){
    twi_timed_out_flag = false;
  }
  return(flag);
}

ISR(TWI1_vect)
{
  // #define TW_STATUS  (TWSR & TW_STATUS_MASK)
  switch(TWSR1 & TW_STATUS_MASK){
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
      // copy device address and r/w bit to output register and ack
      TWDR1 = twi_slarw;
      twi_reply1(1);
      break;

    // Master Transmitter
    case TW_MT_SLA_ACK:  // slave receiver acked address
    case TW_MT_DATA_ACK: // slave receiver acked data
      // if there is data to send, send it, otherwise stop
      if(twi_masterBufferIndex < twi_masterBufferLength){
        // copy data to output register and ack
        TWDR1 = twi_masterBuffer[twi_masterBufferIndex++];
        twi_reply1(1);
      }else{
  if (twi_sendStop)
          twi_stop1();
  else {
    twi_inRepStart = true; // we're going to send the START
    // don't enable the interrupt. We'll generate the start, but we
    // avoid handling the interrupt until we're in the next transaction,
    // at the point where we would normally issue the start.
    TWCR1 = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
    twi_state = TWI_READY;
  }
      }
      break;
    case TW_MT_SLA_NACK:  // address sent, nack received
      twi_error = TW_MT_SLA_NACK;
      twi_stop1();
      break;
    case TW_MT_DATA_NACK: // data sent, nack received
      twi_error = TW_MT_DATA_NACK;
      twi_stop1();
      break;
    case TW_MT_ARB_LOST: // lost bus arbitration
      twi_error = TW_MT_ARB_LOST;
      twi_releaseBus1();
      break;

    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
      // put byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR1;
      /* fall through */
    case TW_MR_SLA_ACK:  // address sent, ack received
      // ack if more bytes are expected, otherwise nack
      if(twi_masterBufferIndex < twi_masterBufferLength){
        twi_reply1(1);
      }else{
        twi_reply1(0);
      }
      break;
    case TW_MR_DATA_NACK: // data received, nack sent
      // put final byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR1;
  if (twi_sendStop)
          twi_stop1();
  else {
    twi_inRepStart = true;  // we're going to send the START
    // don't enable the interrupt. We'll generate the start, but we
    // avoid handling the interrupt until we're in the next transaction,
    // at the point where we would normally issue the start.
    TWCR1 = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
    twi_state = TWI_READY;
  }
  break;
    case TW_MR_SLA_NACK: // address sent, nack received
      twi_stop1();
      break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
      // enter slave receiver mode
      twi_state = TWI_SRX;
      // indicate that rx buffer can be overwritten and ack
      twi_rxBufferIndex = 0;
      twi_reply1(1);
      break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
      // if there is still room in the rx buffer
      if(twi_rxBufferIndex < TWI1_BUFFER_SIZE){
        // put byte in buffer and ack
        twi_rxBuffer[twi_rxBufferIndex++] = TWDR1;
        twi_reply1(1);
      }else{
        // otherwise nack
        twi_reply1(0);
      }
      break;
    case TW_SR_STOP: // stop or repeated start condition received
      // ack future responses and leave slave receiver state
      twi_releaseBus1();
      // put a null char after data if there's room
      if(twi_rxBufferIndex < TWI1_BUFFER_SIZE){
        twi_rxBuffer[twi_rxBufferIndex] = '\0';
      }
      // callback to user defined callback
      twi_onSlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
      // since we submit rx buffer to "wire" library, we can reset it
      twi_rxBufferIndex = 0;
      break;
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
      // nack back at master
      twi_reply1(0);
      break;

    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
      // enter slave transmitter mode
      twi_state = TWI_STX;
      // ready the tx buffer index for iteration
      twi_txBufferIndex = 0;
      // set tx buffer length to be zero, to verify if user changes it
      twi_txBufferLength = 0;
      // request for txBuffer to be filled and length to be set
      // note: user must call twi_transmit(bytes, length) to do this
      twi_onSlaveTransmit();
      // if they didn't change buffer & length, initialize it
      if(0 == twi_txBufferLength){
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }
      // transmit first byte from buffer, fall
      /* fall through */
    case TW_ST_DATA_ACK: // byte sent, ack returned
      // copy data to output register
      TWDR1 = twi_txBuffer[twi_txBufferIndex++];
      // if there is more to send, ack, otherwise nack
      if(twi_txBufferIndex < twi_txBufferLength){
        twi_reply1(1);
      }else{
        twi_reply1(0);
      }
      break;
    case TW_ST_DATA_NACK: // received nack, we are done
    case TW_ST_LAST_DATA: // received ack, but we are done already!
      // ack future responses
      twi_reply1(1);
      // leave slave receiver state
      twi_state = TWI_READY;
      break;

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      twi_error = TW_BUS_ERROR;
      twi_stop1();
      break;
  }
}
