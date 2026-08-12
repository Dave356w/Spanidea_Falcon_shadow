/*
 ***********************************************************
 * File   : lateral.h
 *
 * X/Y lateral monitor -- INSTRUMENTATION and calibration quality gate.
 *
 * This began as the RESET half of a Z + X/Y design ("Z sets the beacon, X/Y
 * resets it"). That release path is DEAD, permanently: the parked-vs-cruise
 * contrast measured 1.19x at 3.13 Hz AND at 25 Hz (falcon_signature
 * 2026-08-11 §1.2) -- an energy level survives aliasing, so no sample rate
 * fixes it. It is a property of this machine, not of the sampling. The
 * release code (XY_RELEASE_ARMED and its FSM block, XY_RELEASE_POLLS,
 * XY_MIN_BEACON_MS, XY_REARM_MS) was removed in the 2026-08-12 cleanup;
 * last version at 8308cdd.
 *
 * What remains, and why it stays:
 *
 *   - the m= / q= log fields -- the lateral level is still the only
 *     per-run mounting-quality signal the capture has, and cruise
 *     vibration measures the MOUNTING as much as the machine;
 *   - the in-situ calibration, whose movement/quorum verdict is the
 *     quality gate the z zero-calibration retry logic keys off
 *     (see CALIB_RETRIES in movement_service.h).
 ***********************************************************
 */

#ifndef _LATERAL_H_
#define _LATERAL_H_

#include <Arduino.h>

/*
 * THE METRIC: |dx| + |dy| between consecutive unblanked samples, m/s^2.
 *
 * A difference, not a deviation from a stored reference, for two reasons:
 *
 *  1. It is immune to mounting angle. x/y have no gravity component only when
 *     the device sits flat; on a tilted counterweight frame some of the 1 g
 *     leaks into them as a STATIC offset. A difference cancels any constant,
 *     so no lateral zero has to be learned or maintained, and the mechanic
 *     nudging the unit mid-job cannot invalidate a stored reference.
 *  2. There is nothing to go stale. §3.1's recurring failure on this device is
 *     a baseline captured once and then wrong forever; a metric with no
 *     baseline cannot have that failure.
 *
 * Aliasing is survivable here in a way it is not for transient detection.
 * Sub-sampling a 40 Hz-filtered signal at 3.13 Hz scrambles the waveform's
 * shape but preserves its power, and this is an energy measure, not a shape
 * measure.
 */

/*
 * Longest gap that may be differenced, milliseconds.
 *
 * Two samples either side of a long gap say nothing about the interval
 * between them. During a beacon the timer ISR drops every sample taken while
 * the piezo is driven plus BUZZER_RINGDOWN_MS after it (see alarm.cpp), so at
 * the 2/5 duty roughly half of wall clock produces no sample at all and the
 * gaps that survive are ~320 ms or ~640 ms.
 *
 * 1200 ms keeps both of those and rejects anything longer. A rejected gap
 * produces NO metric, which means it does not advance the quiet run, which
 * means it cannot release the beacon -- the safe direction.
 */
#define XY_MAX_DT_MS              1200

/* ---- calibration: learning XY_STILL in situ ------------------------------ */

/*
 * The deployment sequence is: place the device on the counterweight, THEN
 * power it on. So the existing 10 s calibration always runs at rest, on the
 * actual machine, in the actual building -- which is precisely the
 * measurement the threshold needs and the reason it can be learned rather
 * than shipped as a constant. A bench-derived number would be far too low for
 * a live hoistway and the unit would sound constantly.
 *
 * ERROR DIRECTION IS WHAT MAKES AUTOMATING THIS SAFE:
 *
 *   floor UNDER-estimated -> threshold too low  -> slow release, over-eager
 *                                                  beacon -- annoying, SAFE
 *   floor OVER-estimated  -> threshold too high -> travel reads as still ->
 *                                                  releases mid-travel -- DANGEROUS
 *
 * Short windows tend to underestimate a peak, so ordinary sampling error
 * lands on the safe side. The one thing that pushes it the dangerous way is a
 * loud event inside the window, which is what the robust statistic below
 * defends against.
 */

/* Bucket width and count. 10 x 1 s covers the 10 s calibration window. */
#define XY_CALIB_BUCKET_MS        1000
#define XY_CALIB_BUCKETS          10

/*
 * Buckets that must contain data before a learned threshold is trusted.
 *
 * At 3.13 Hz a 1 s bucket holds ~3 samples, so a bucket with none means the
 * timer stalled or the sensor was not answering. 6 of 10 is a thin but real
 * quorum; below it the median is being taken over too few numbers to be the
 * robust statistic it is supposed to be.
 */
#define XY_CALIB_MIN_BUCKETS      6

/*
 * STATISTIC: median of per-second maxima, NOT the global maximum.
 *
 * The maximum is exactly the wrong statistic here -- one loud second, another
 * car in the bank or someone working on the cartop, would inflate the
 * threshold and push the error into the dangerous direction for the whole
 * deployment. A median tolerates up to half the window being contaminated.
 */

/*
 * Margin over the learned floor.
 *
 * Small on purpose. The floor is a median of maxima, so it already sits at
 * the top of the parked distribution rather than in the middle of it, and
 * every bit of margin added here is spent moving toward "travel reads as
 * still". 1.5 is the smallest multiplier that keeps ordinary parked jitter
 * from tripping the beacon back on the instant it releases.
 */
#define XY_STILL_MARGIN           (1.50f)

/*
 * Clamps on the learned value.
 *
 * MIN keeps a freakishly quiet calibration -- a device on a rubber mat in a
 * dead building -- from setting a threshold no real parked sample can get
 * under, which would mean the beacon never releases and every run reaches the
 * failsafe.
 *
 * MAX is the safety clamp and matters far more: it bounds how wrong a
 * contaminated calibration can make the device. Above this the site is
 * declared noisy and the mechanic is told so at the one moment they are
 * standing next to the unit.
 *
 * Both numbers are UNMEASURED. The parked z noise floor is 0.103 m/s^2 and
 * x/y have never been recorded on a counterweight at all -- these are placed
 * to bracket that, and the first hoistway capture should replace them.
 */
#define XY_STILL_MIN              (0.02f)
#define XY_STILL_MAX              (0.40f)

/*
 * Threshold used when calibration is rejected. Deliberately at the low clamp:
 * a rejected calibration means we do not know the site, and not knowing must
 * produce the over-eager device, never the deaf one.
 */
#define XY_STILL_FALLBACK         XY_STILL_MIN

/*
 * Above this, a calibration bucket is movement rather than noise, and the
 * whole window is rejected -- a counterweight that moved during those 10 s
 * yields a threshold wrong in the dangerous direction.
 *
 * Sized against z: a real 18 fpm arrival transient measured 0.058 m/s^2 and
 * ordinary parked excursions reach 0.103. UNMEASURED on x/y.
 */
#define XY_CALIB_MOVE_MSS         (0.80f)

class LateralMonitor
{
  public:
    LateralMonitor();

    /*
     * Feed one good, UNBLANKED sample, in loop() context. Callers must not
     * call this for a failed sensor read; main.cpp takes both samples from
     * the ISR's published snapshot, so the buzzer blanking that drops samples
     * in the timer ISR has already been applied upstream.
     */
    void add(float ax, float ay, uint32_t now_ms);

    void  calib_begin(uint32_t now_ms);

    /*
     * Close the window and derive the threshold. Returns false if the
     * calibration cannot be trusted -- too few buckets, or a bucket that
     * looks like movement. On false, nothing is armed.
     */
    bool  calib_finish();

    /* Arm on the conservative compile-time floor after a rejection. */
    void  arm_fallback();

    /* Forget the run state; call when the beacon starts. */
    void  clear_quiet() { quiet = 0; }

    /*
     * Forget the previous sample, so the next metric is not a difference
     * taken across something the device did to itself -- the ready chirp.
     */
    void  drop_anchor() { have_last = false; }

    bool     armed()      { return xy_still > 0.0f; }
    bool     noisy()      { return xy_still >= XY_STILL_MAX; }
    float    still()      { return xy_still; }
    float    m()          { return last_m; }
    uint8_t  quiet_run()  { return quiet; }
    uint8_t  buckets()    { return n_buckets; }
    float    calib_peak() { return peak; }

  private:
    float    last_ax, last_ay;
    uint32_t last_ms;
    bool     have_last;

    float    last_m;     /* most recent metric, for the log            */
    uint8_t  quiet;      /* consecutive metrics under xy_still         */

    bool     calibrating;
    uint32_t bucket_start;
    float    bucket_max;
    bool     bucket_used;
    float    peak;       /* largest bucket max in the window           */
    uint8_t  n_buckets;
    float    bmax[XY_CALIB_BUCKETS];

    float    xy_still;   /* 0.0 means not armed                        */
};

extern LateralMonitor lat_monitor;

#endif
