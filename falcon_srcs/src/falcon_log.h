/*
 * falcon_log.h -- compile-time logging level.
 *
 * WHY THIS EXISTS. The serial log is 99% periodic sample lines and every one
 * of them is a BLOCKING Serial.print from loop(). That costs flash (the F()
 * strings and the Print code), current (~1.10 mA), and -- the part that
 * matters -- it is the sole cause of ring overrun, because the sample ring is
 * drained one sample per loop() pass and a print in that pass throttles the
 * consumer directly. See falcon_test_plan_2026-08-18.md §1.2.
 *
 * LEVELS
 *
 *   2  everything. The build every threshold on file was measured on, and the
 *      build every test session must use. This is the default.
 *   1  events only -- FSM transitions, BURST, JOGV, ARM, calibration, battery.
 *      Drops the periodic "t= a= avg= ..." line, which is where the bytes and
 *      the per-sample blocking are. A unit in service is still diagnosable.
 *   0  silent. Serial is never even begun.
 *
 * ⚠️ LEVELS BELOW 2 ARE NOT A PRINT-ONLY CHANGE. Removing the per-sample print
 * removes the blocking that has been throttling the sample consumer all along,
 * so the detectors see MORE samples than the population any threshold on file
 * was derived from. That is the direction that widens dt on the calibration
 * metric and it has never been characterised in a car. Do not ship a level < 2
 * build to an installation until session A's verification clause is satisfied.
 *
 * ⚠️ ARGUMENTS ARE STILL EVALUATED at every level. The sink swallows the
 * output, it does not swallow the expression, so a Serial.print of something
 * with a side effect keeps that side effect. Nothing in this codebase relies
 * on that today; it is guaranteed so that nothing has to be audited for it.
 */

#ifndef FALCON_LOG_H_
#define FALCON_LOG_H_

#include <Arduino.h>

#ifndef FALCON_LOG
#define FALCON_LOG 2
#endif

/*
 * Interval between HEALTH lines at FALCON_LOG 1. 30 s is short enough to
 * catch a wedge or an overrun burst inside one visit and long enough that
 * the print cannot throttle the sample consumer the way the per-sample line
 * does -- ~1 line per 750 samples at 25 Hz.
 */
#ifndef HEALTH_PERIOD_MS
#define HEALTH_PERIOD_MS 30000UL
#endif

#if FALCON_LOG > 0

#define FLOG           Serial
#define FLOG_BEGIN(b)  Serial.begin(b)

/*
 * flog_fixed() -- print a float as fixed point, WITHOUT Print::print(double).
 *
 * WHY. Arduino's printFloat costs 416 bytes of flash on its own: an isnan /
 * isinf / ovf preamble, a float DIVISION loop to build the rounding term, and
 * a float multiply-and-subtract for every digit. This does the same job with
 * one multiply and integer arithmetic, and the 21 call sites that used to name
 * the two-argument print() are what keep that 416 bytes linked. Every one of
 * them has to move or none of the saving arrives -- a single surviving
 * `print(x, n)` pulls the whole function back in.
 *
 * ⚠️ THE OUTPUT IS NOT BIT-IDENTICAL to what shipped before, and the direction
 * of the difference is toward the correct value, not away from it. Arduino
 * adds its rounding term to the whole number BEFORE splitting off the integer
 * part, then extracts each digit by repeated multiply-and-truncate, so a value
 * sitting on a rounding tie can lose the last digit to accumulated error --
 * 39.5205 printed at 3 dp comes out `39.520` there and `39.521` here. Measured
 * over the real ranges of all 21 sites: 2 dp and 3 dp disagree on 0.00-0.02%
 * of values, 4 dp on 0.01-0.05%, always by one count of the last digit and
 * always on a tie.
 *
 * ⚠️ The 6 dp sites are the exception and they are not a tie case. `zero_calib
 * _value` is ~9.7, where a float's ULP is 9.5e-7 -- one whole count of the 6th
 * decimal -- so that digit was already noise, and Arduino's pre-added rounding
 * term quantises against the same ULP. The two disagree on ~44% of values by
 * one count. Nothing reads it that closely: every consumer in graph/ parses
 * with float() on `(-?\d+\.?\d*)`, the BMA456 resolves 0.0006 m/s2 at +-2g,
 * and 1e-6 m/s2 is four orders below that.
 */
void flog_fixed(float v, uint8_t digits);

/*
 * flog_nl() -- the shared line ending.
 *
 * F("\r\n") appeared 27 times. F() is PSTR(), and PSTR() makes a FRESH static
 * array at every expansion -- the compiler does not merge them -- so those 27
 * were 27 separate 3-byte strings plus the pointer setup and call at each
 * site. One noinline function turns each into a 2-byte rcall.
 */
void flog_nl();

/*
 * flog_state() -- the six STATE transition lines.
 *
 * They shared a 27-character prefix and a 3-character tail, stored six times
 * over because F() cannot merge. Passing only the state name and printing the
 * fixture around it gives byte-identical output from 101 bytes of string
 * instead of 249.
 */
void flog_state(const __FlashStringHelper *name);

/*
 * flog_kv() -- a label and the number that follows it.
 *
 * The log is built almost entirely out of `FLOG.print(F(" q=")); FLOG.print(x);`
 * pairs, and each half of that pair is a MEMBER call: it loads `Serial` into
 * the register pair as well as the argument, so the pair costs about twenty
 * bytes at every one of the 70-odd sites. Folding it into one call halves the
 * setup and turns two calls into one rcall.
 *
 * The value is taken as `long` deliberately. Print promotes every integer type
 * to it anyway, so the output is unchanged, and a `const char *` or an
 * __FlashStringHelper * will not convert to it implicitly -- a string-valued
 * site is a compile error here rather than a silently mangled log line.
 */
void flog_kv(const __FlashStringHelper *label, long v);

/* The same fold for the float sites -- 20 of the 21 carry a label too. */
void flog_kvf(const __FlashStringHelper *label, float v, uint8_t digits);

/*
 * And for a RAW millis() stamp. flog_kv() takes a long because that is what
 * Print promotes to, but a uint32_t millis() value crosses 2^31 after 24.9
 * days -- `HEALTH t=` on a unit that has been up a month would print negative,
 * and a unit on a counterweight is up for months. DELTAS are small and signed
 * and belong on flog_kv(); only a raw stamp needs this one.
 */
void flog_kvt(const __FlashStringHelper *label, unsigned long t);


#else

/*
 * Swallowing sink. print() takes its argument by const reference and does
 * nothing with it, so the F() string it names is never referenced and the
 * linker drops it along with the Print machinery.
 */
struct FalconNullLog {
    template <typename T> void print(const T &) {}
    template <typename T> void print(const T &, int) {}
};

extern FalconNullLog falcon_null_log;

/*
 * Same contract as the sink above: the ARGUMENT is still evaluated at the call
 * site, only the output is dropped.
 */
inline void flog_fixed(float, uint8_t) {}
inline void flog_nl() {}
inline void flog_state(const __FlashStringHelper *) {}
inline void flog_kv(const __FlashStringHelper *, long) {}
inline void flog_kvf(const __FlashStringHelper *, float, uint8_t) {}
inline void flog_kvt(const __FlashStringHelper *, unsigned long) {}

#define FLOG           falcon_null_log
#define FLOG_BEGIN(b)  ((void)0)

#endif  /* FALCON_LOG > 0 */

/* Shared fragments -- see the block above their definitions in main.cpp. */
#define FSTR(s) (reinterpret_cast<const __FlashStringHelper *>(s))
extern const char FS_A[] PROGMEM;
extern const char FS_ARMED[] PROGMEM;
extern const char FS_BAT[] PROGMEM;
extern const char FS_BZ[] PROGMEM;
extern const char FS_DIR[] PROGMEM;
extern const char FS_M[] PROGMEM;
extern const char FS_N[] PROGMEM;
extern const char FS_OV[] PROGMEM;
extern const char FS_PIN[] PROGMEM;
extern const char FS_Q[] PROGMEM;
extern const char FS_RULE[] PROGMEM;
extern const char FS_SP[] PROGMEM;
extern const char FS_ST[] PROGMEM;
extern const char FS_TW[] PROGMEM;
extern const char FS_OBS[] PROGMEM;
extern const char FS_UNARMED[] PROGMEM;


#endif  /* FALCON_LOG_H_ */
