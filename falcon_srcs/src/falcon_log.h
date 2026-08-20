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

#define FLOG           falcon_null_log
#define FLOG_BEGIN(b)  ((void)0)

#endif  /* FALCON_LOG > 0 */

#endif  /* FALCON_LOG_H_ */
