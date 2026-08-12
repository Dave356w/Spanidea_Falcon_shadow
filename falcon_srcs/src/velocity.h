/*
 * velocity.h -- windowed velocity integral (Eng_Notes, datasheet review §5)
 *
 * WHAT THIS IS FOR
 *
 * Every detector in this firmware up to now has thresholded acceleration
 * AMPLITUDE. That is the wrong observable, and §14.7 is the proof: a real
 * 18 fpm arrival measured 0.058 m/s^2 against a parked ceiling of 0.103, so no
 * threshold separates them and §14.9 concluded arrival detection should stop
 * being tuned.
 *
 * Velocity change is the right observable. A departure to 20 fpm is
 * dv = 0.1016 m/s NO MATTER HOW GENTLY IT RAMPS -- softness spreads the same
 * integral over more time instead of shrinking it. Vibration is zero-mean and
 * cancels under integration; a run does not.
 *
 * (The arrival-side conservation test that once lived here is dead -- see the
 * ⛔ block below.)
 *
 * It also reaches a departure any-motion cannot. A car ramping to 20 fpm over
 * 4 s averages 0.025 m/s^2 -- SIX TIMES below ANYMOTION_THRESHOLD (32 counts =
 * 0.153 m/s^2, confirmed against the datasheet). No any-motion setting that
 * survives cruise vibration can see it. The integral crosses a 0.04 m/s gate
 * 1.6 s in, by which point the car has moved 33 mm.
 *
 *
 * WHY IT IS WINDOWED AND NOT A FREE-RUNNING INTEGRAL
 *
 * Measured, not assumed. Integrating a whole run against a fixed baseline was
 * tried against the 2026-08-07 captures and produced -4.44 m/s on the 171 s
 * descent. A 0.026 m/s^2 baseline error times 171 s IS 4.4 m/s, and the
 * datasheet puts zero-g offset at +/-20 mg = 0.196 m/s^2. Open-loop integration
 * of this sensor is hopeless over a run, and nothing about that is fixable by
 * better tuning.
 *
 * A trailing window of VEL_WINDOW_MS bounds the damage to eps * VEL_WINDOW_MS
 * while still capturing a 1-2 s departure ramp in full. It is a bandpass
 * centred near 1/VEL_WINDOW_MS, which is where the transients live.
 *
 * The consequence to keep in mind: w() is the velocity change over the last
 * few seconds, NOT absolute speed. During cruise it reads ~0, exactly as it
 * does parked. This detects departures and arrivals as EVENTS. It does not
 * observe constant velocity, and §3's argument that nothing can is untouched.
 * The latch is still mandatory.
 *
 *
 * SAMPLE RATE IS THE BINDING CONSTRAINT -- READ THIS BEFORE TUNING
 *
 * Measured on the parked soak in device-monitor-260807-123553.log, 5 s window,
 * decimating the capture to simulate lower rates:
 *
 *   effective rate   sigma_w      vs 3 Hz    sqrt(k)
 *   3.06 Hz          0.0232 m/s    1.00x      1.00x
 *   1.53 Hz          0.0333 m/s    1.44x      1.41x
 *   0.76 Hz          0.0431 m/s    1.86x      2.00x
 *
 * It tracks sqrt(k) in both logs tested. The parked noise is WHITE, which means
 * it is dominated by the 3.13 Hz sampling of a signal filtered to 40.5 Hz --
 * aliasing we create ourselves -- and not by hoistway vibration. More samples
 * help, proportionally.
 *
 *   3.13 Hz today : sigma_w = 0.023 m/s, so 5 sigma = 0.115 m/s = 22.6 fpm
 *   100 Hz         : sigma_w ~ 0.004 m/s (sqrt(32) better), 5 sigma = 4.5 fpm
 *
 * SO: AT THE CURRENT SAMPLE RATE THIS CANNOT MEET THE 20 FPM REQUIREMENT.
 * It is armed here because it is strictly additive -- see the guard rules below
 * -- and because it will meet the requirement comfortably once roadmap item 5
 * (100 Hz) lands. Do not lower VEL_SIGMA_MSS to make the numbers look better;
 * lower it when the sample rate is raised and a fresh soak has been measured.
 *
 *
 * GUARDS AGAINST KNOWN SIDE EFFECTS
 *
 * 1. BASELINE DRIFT (the failure that killed the free-running version).
 *    Tracked as a slow EMA, and FROZEN whenever the FSM is not monitoring,
 *    whenever the buzzer is sounding, or whenever |w| is already elevated --
 *    otherwise a slow real departure gets tracked out by its own detector.
 *
 * 2. UNPRIMED WINDOW. §3.1 records a false alarm on a stationary bench caused
 *    by a RollingAvg ramping 0 -> 2.43 -> 4.86 -> 9.72 from an unprimed state,
 *    which then latched a baseline it could never clear. The same class of bug
 *    would be worse here. Nothing is reported until VEL_PRIME_SAMPLES good
 *    samples have set the baseline, and valid() gates every consumer.
 *
 * 3. BUZZER COUPLING. §5 and §11: the piezo shakes the accelerometer, and the
 *    timer ISR already drops samples while it sounds. Integrating across those
 *    gaps would extrapolate the current sample over blanked time, so dt is
 *    clamped to VEL_MAX_DT_MS. A 200 ms beep plus 50 ms ringdown makes the
 *    real gap ~570 ms while the clamp allows 400, so an arrival that straddles
 *    a beep is UNDER-counted. That is the safe direction: an under-counted
 *    arrival holds the alarm longer (§14.4).
 *
 * 4. SENSOR FAULTS. Only good reads are fed in, so a dead sensor stops the
 *    window rather than filling it with zeros (§10.2). Coverage decays and
 *    valid() goes false within VEL_WINDOW_MS.
 *
 * 5. LONG GAPS. Bins skipped entirely contribute zero, not an extrapolation.
 *    coverage_ms() reports how much of the window was actually observed, and
 *    is logged so a capture shows it rather than hiding it.
 */

#ifndef _VELOCITY_H_
#define _VELOCITY_H_

#include <Arduino.h>

/*
 * Window length. 5 s comfortably covers a 1-2 s departure ramp and the
 * levelling-to-brake-set sequence §13 describes, while bounding drift to
 * 5 * eps. Tested at 2/3/5/8/12/20 s against the captures; 3-5 s gave the best
 * signal-to-parked-noise ratio at both sample rates measured.
 */
#define VEL_WINDOW_MS             5000

/*
 * Bins are the resolution at which the window edge moves. 10 x 500 ms costs
 * 60 bytes of RAM. Finer bins buy nothing at 3.13 Hz -- one sample lands in
 * each -- and would still be adequate at 100 Hz (50 samples per bin).
 */
#define VEL_BINS                  10
#define VEL_BIN_MS                (VEL_WINDOW_MS / VEL_BINS)

/*
 * Largest dt a single sample may be credited with, milliseconds.
 *
 * At 3.13 Hz the nominal period is 320 ms, so 400 passes normal operation
 * untouched and clips only gaps -- which is the point. Set this to ~1.5x the
 * ISR period when the timer is fixed (100 Hz -> 15).
 */
#define VEL_MAX_DT_MS             400

/*
 * Fraction of the window that must have been observed before w() is trusted.
 * The alarm blanks ~45-55% of wall clock (§5 item 7), so this has to sit below
 * half or arrival would never be measurable while the buzzer sounds.
 */
#define VEL_MIN_COVERAGE_MS       1800

/*
 * Good samples averaged into the baseline before the window is armed.
 *
 * Bias in w is (baseline error) x VEL_WINDOW_MS, so holding w's bias under
 * 0.02 m/s needs a baseline good to 0.004 m/s^2. Bench sigma is ~0.013 m/s^2
 * per sample, so 32 samples gives ~0.0023 and a bias near 0.011 m/s.
 *
 * Do not lower this without redoing that arithmetic. Seeding from ONE sample
 * was tried on hardware 2026-08-07 and put w at -0.26 m/s on a stationary
 * bench -- past the departure gate -- because the boot sample sat 0.05 high.
 * At 3.13 Hz this is ~10 s, which fits inside CALIB_TIMEOUT_MS.
 */
#define VEL_PRIME_SAMPLES         32

/*
 * Baseline EMA time constant. Long enough to ignore a 1-2 s departure ramp
 * even if the freeze logic somehow failed, short enough to follow thermal
 * drift (datasheet TCO +/-0.35 mg/K).
 */
#define VEL_BASELINE_TAU_MS       120000UL

/*
 * Freeze baseline tracking once |w| exceeds this, m/s. Prevents the tracker
 * from absorbing a slow genuine departure.
 *
 * MUST sit ABOVE the parked distribution and BELOW the departure gate:
 *
 *   parked p99  (soak)          0.076
 *   THIS                        0.100
 *   departure gate              0.255
 *
 * The first value tried was 0.030, which is INSIDE the parked distribution --
 * so on a stationary bench the tracker spent most of its time frozen and the
 * baseline could not be maintained. Observed on hardware 2026-08-07: w settled
 * near 0.02-0.04, crossed 0.030, froze, and then drifted until VEL_REPRIME_MS
 * forced a re-prime -- a limit cycle that blinds the window for ~10 s each
 * time it goes round.
 *
 * 0.100 leaves the tracker running through normal parked noise while still
 * freezing at 0.4x the departure gate, so a genuine departure -- which is
 * heading for 0.255 and beyond -- is still protected long before it arrives.
 */
#define VEL_BASELINE_FREEZE_MPS   (0.100f)

/*
 * How long |w| may sit above the freeze level, while the FSM is still
 * monitoring, before the baseline is declared wrong and re-primed.
 *
 * The freeze uses |w| as evidence of motion, but |w| is derived FROM the
 * baseline, so a bad baseline freezes itself in place forever. The first
 * hardware run on 2026-08-07 did exactly that: w stuck at -0.26 on a bench,
 * past the departure gate, with no path back. See velocity.cpp.
 *
 * 3x the window. A real departure's w decays within a window or two, and the
 * FSM leaves STATE_MONITORING within MOVEMENT_DETECTION_TIMEOUT_MS anyway --
 * so nothing genuine can hold this for 15 s and still be monitoring.
 */
#define VEL_REPRIME_MS            (3UL * VEL_WINDOW_MS)

/*
 * ⛔ NOT ARMED. The FSM computes and logs the velocity window but does not act
 *    on it. Set to 1 only after a hoistway session has produced the numbers
 *    described below. See the measurement that forced this.
 *
 * The whole module was replayed through the 2026-08-07 captures before being
 * wired up, against the ONE stretch where "parked" is ground truth: the
 * 17.6-minute stationary soak of §14.6, which recorded zero any-motion edges
 * and zero false departures.
 *
 *   parked stretch        1241 s, 3885 samples
 *   |w| median / p99      0.0176 / 0.0757 m/s
 *   |w| PEAK              0.1274 m/s
 *   implied sigma_w       0.0166 m/s
 *
 * The peak is 7.7 sigma. A Gaussian would not produce that once in 3854 looks,
 * so the parked distribution is HEAVY-TAILED -- vibration arrives in bursts,
 * and sigma does not describe it. A 5 sigma gate (0.115 m/s) was crossed TWICE
 * in 20.7 minutes, which extrapolates to roughly 46 false departures per
 * 8-hour shift, each latching the alarm for up to LATCH_FAILSAFE_MS.
 *
 * A gate safe against the measured peak needs to sit near 0.25 m/s = 50 fpm,
 * which is far above the 20 fpm requirement and well inside what any-motion
 * already catches. So AT 3.13 Hz THERE IS NO THRESHOLD THAT IS BOTH SAFE AND
 * USEFUL. Arming this now would make the device worse, not better.
 *
 * This is the same discipline §11 applied to any-motion: wire it in, log what
 * it would have done, characterise it in a real hoistway, and only then let it
 * touch the alarm. That sequence is why the any-motion path works.
 *
 * WHAT UNBLOCKS IT: sample rate, and only sample rate. The parked noise was
 * shown to be white (sigma_w tracks sqrt of the sample interval across 4x of
 * decimation, on both logs), which means it is dominated by 3.13 Hz sampling
 * of a signal filtered to 40.5 Hz -- aliasing this firmware creates -- and not
 * by the hoistway. At 100 Hz the same window should see ~5.7x less. Re-run the
 * soak after roadmap item 5, put the measured PEAK (not sigma) in
 * VEL_PARKED_PEAK below, and set VEL_ARMED to 1.
 */
#define VEL_ARMED                 0

/*
 * MEASURED peak |w| over the parked soak, m/s. Thresholds derive from this
 * rather than from sigma, because the distribution is not Gaussian and sigma
 * understates the tail by a factor of ~1.5 even over 20 minutes.
 *
 *   sample rate   measured / expected VEL_PARKED_PEAK
 *   3.13 Hz       0.1274   <- measured, §14.6 soak
 *   12.5 Hz       ~0.064   expected, UNMEASURED
 *   50 Hz         ~0.032   expected, UNMEASURED
 *   100 Hz        ~0.023   expected, UNMEASURED
 *
 * Only the first row is a measurement. Do not arm against an expected value.
 */
#define VEL_PARKED_PEAK           (0.1274f)

/*
 * Departure gate, m/s.
 *
 * 2x the measured parked peak. The soak is 20 minutes and a deployment is a
 * shift, so the tail has ~23x more opportunity to produce an outlier than the
 * measurement saw; doubling is the smallest defensible allowance for that and
 * should itself be checked against a full-day soak before arming.
 *
 * At 3.13 Hz that is 0.255 m/s = 50 fpm. At 100 Hz it would be ~0.046 m/s =
 * 9 fpm, comfortably inside the 20 fpm requirement -- which is the entire
 * argument for fixing the sample rate.
 */
#define VEL_DEPART_THRESHOLD      (2.0f * VEL_PARKED_PEAK)

/*
 * ⛔ THE CONSERVATION ARRIVAL TEST IS GONE, and it is not coming back.
 *
 * The idea was that the arrival integral must cancel the departure integral.
 * The 2026-08-11 arrival bursts killed it: a smooth drive deceleration
 * integrates to 67% of true speed (window closes mid-ramp) and a brake stop
 * to 128% (the shock impulse inflates it), so NEITHER end of the comparison
 * is reliably measurable -- falcon_signature_2026-08-11.md §4e.1. The code
 * (VEL_ARRIVE_FRACTION, VEL_ARRIVE_MIN, VEL_ARRIVAL_ENABLE and the FSM's 1b
 * path) was removed in the 2026-08-12 cleanup; last version at 8308cdd.
 *
 * What survives is the RAMP measurement, which references nothing about the
 * departure -- see the ramp detector in main.cpp.
 */


class VelocityWindow
{
  public:
    VelocityWindow();

    /*
     * Feed one GOOD, UNBLANKED sample. Callers must not call this for failed
     * reads or while get_buzzer_status() is true -- see guards 3 and 4.
     */
    void add(float accel_mss, uint32_t now_ms);

    /*
     * Age the window on wall clock. MUST be called from loop() every pass:
     * without it the window only decays when a sample arrives, so w() would
     * hold a stale value across a buzzer blank -- which is exactly when the
     * FSM is asking about arrival.
     */
    void tick(uint32_t now_ms);

    /* Drop the window contents. Keeps the baseline. */
    void reset(uint32_t now_ms);

    /*
     * Signed velocity change across the window, m/s. Sign follows the
     * acceleration convention of the caller (z, positive up).
     */
    float w();

    /* False until primed AND enough of the window has been observed. */
    bool valid();

    uint16_t coverage_ms() { return total_cov; }
    float    baseline()    { return base; }
    bool     primed()      { return prime_count >= VEL_PRIME_SAMPLES; }

    /*
     * Baseline tracking is only safe while the FSM believes nothing is moving.
     * main.cpp drives this from the FSM state; see guard 1.
     */
    void set_baseline_tracking(bool allow) { track_baseline = allow; }

  private:
    float    bin_v[VEL_BINS];      /* velocity contribution per bin, m/s   */
    uint16_t bin_cov[VEL_BINS];    /* observed time per bin, ms            */
    uint8_t  cur;                  /* index of the bin being filled        */
    uint32_t cur_bin_start;        /* millis() when that bin opened        */
    uint32_t last_sample_ms;
    bool     have_last;

    float    base;                 /* baseline acceleration, m/s^2         */
    uint16_t prime_count;
    uint32_t w_high_since;         /* when |w| first exceeded the freeze    */
    bool     track_baseline;

    float    w_cache;
    uint16_t total_cov;

    void advance_to(uint32_t now_ms);
    void recompute();
};

extern VelocityWindow vel_window;

#endif /* _VELOCITY_H_ */
