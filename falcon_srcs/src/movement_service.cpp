#include "movement_service.h"

/*
 * Polled departure threshold, on |acceleration_avg - zero_calib|.
 *
 * ⚠️ 0.40 -> 0.20 -> 0.10, 2026-08-18, TRIAL. Revert is this one number.
 *
 * 0.20 was flashed and shown HARMLESS but not USEFUL: 2 runs right-side up,
 * both caught by any-motion, so the backstop never fired. 0.10 is the value
 * where the backstop actually carries weight -- 72% of departures against 18%
 * -- while keeping 3.9x headroom on the worst right-side-up rest sample
 * (MAX 0.0257, n=470).
 *
 * ⚠️ THE REST TAIL IS UNCHARACTERISED IN THIS MOUNTING. 470 quiet samples
 * right-side up against 21391 inverted; MAX 0.0257 is the worst of a SMALL
 * sample and the tail may be higher. There is less room for it at 0.10.
 *
 * ⚠️ HANDLING THE DEVICE WILL NOW LATCH. A hand bump measured 0.136 and
 * walking reached 0.87 -- both far above 0.10. Plugging in debug wiring will
 * fire a departure. That is the threshold working as configured, not a fault.
 *
 * WHY. This path is the BACKSTOP for a missed any-motion departure, and on
 * 2026-08-18 that backstop was needed and absent: any-motion missed 6 of 12
 * slow up runs, `VEL_ARMED` is 0, and this threshold has -- as the comment in
 * main.cpp says -- "never detected a departure in any hoistway run".
 *
 * IT NEVER FIRED BECAUSE IT CANNOT. Measured over 82 departures in the
 * 2026-08-18 capture, the largest excursion this average reaches is **0.398**.
 * The threshold sat just above the best case ever recorded.
 *
 *   T=0.40   1/82 departures ( 1.2%)   at-rest excursions over T:  1/21391
 *   T=0.20  15/82 departures (18.3%)                              15/21391
 *   T=0.15  39/82 departures (47.6%)                              20/21391
 *   T=0.10  59/82 departures (72.0%)                              56/21391
 *
 * 0.20 is the conservative first step: it makes the backstop reachable while
 * the at-rest exposure stays at 0.07% of samples, and those excursions sit
 * inside the walk/bump disturbances that ALREADY latch via any-motion -- so it
 * adds detection without adding a new false-alarm mode.
 *
 * ⚠️ WHAT THIS IS WORTH IS NOT 18%. As a backstop, what matters is the catch
 * rate on the departures any-motion MISSED, and misses are plausibly the
 * WEAKER departures, where this path is also least likely to fire. Treat 18%
 * as an upper bound, not an expectation.
 *
 * ⛔ THE AT-REST FIGURES ARE CARTOP, THE NOISY CASE. The rest tail reaches
 * 0.145 (p99.9) and 0.872 (max) only because the capture contains walks, a
 * hand bump and undetected motion. Against genuinely quiet rest the p99 is
 * 0.015. On a real counterweight -- where nobody walks, and the spec has the
 * device placed BEFORE power-on -- the disturbance population that constrains
 * this may not exist, and a lower value becomes defensible. That measurement
 * has never been taken.
 *
 * 🔴 FAILURE DIRECTION IF THIS IS WRONG: too low means a FALSE departure --
 * beacon on a stationary car, which at LATCH_FAILSAFE_MS 600 s is a ten-minute
 * alarm (see falcon_state_of_project_2026-08-18.md §4.2, LATCH_FAILSAFE_MS --
 * the note this line used to cite as falcon_jog_ringing_2026-08-18.md does not
 * exist under that name). Watch for a latch with no
 * car movement; revert to 0.40 if it appears.
 */
#define DEFAULT_THRESHOLD_VALUE 0.10

/*
 * ─── SELF-CALIBRATED POLLED DEPARTURE THRESHOLD (2026-08-18) ────────────────
 *
 * THE PROBLEM WITH A CONSTANT. The polled departure excursion is
 * MOUNTING-DEPENDENT, measured the same afternoon on one device:
 *
 *   inverted (cwt-bottom sim)   departures median 0.136, max 0.398
 *   right-side up               departures 0.050 - 0.120, median 0.067
 *
 * A 2-3x spread. Any fixed DEFAULT_THRESHOLD_VALUE is therefore wrong in every
 * mounting but the one it was measured in: 0.40 caught 1/82 (dead), 0.20 caught
 * 0/8 right-side up, 0.10 catches 1/8. Meanwhile the REST noise is stable and
 * small in both (right-side up p99 0.0143, MAX 0.0257 over 470 samples;
 * inverted p99 0.0151) -- so the noise floor, not the departure, is the thing
 * worth measuring per deployment.
 *
 * THE FIX, and it reuses a pattern this device already ships. XY_STILL learns
 * the LATERAL noise floor during the 6 s calibration window and sets the
 * lateral gate as a multiple of it, clamped. This does the same for Z: measure
 * the spread of the z average across the calibration window, and set the
 * polled departure threshold to a multiple of that.
 *
 * The deployment sequence is what makes this sound -- placement happens BEFORE
 * power-on, so calibration is guaranteed to run at rest, on the counterweight,
 * in the building (falcon_spec_primary_usecase_2026-08-09 §1).
 *
 * ⚠️ THE MARGIN IS THE UNMEASURED PART, exactly as XY_STILL's was. Rest spread
 * over a 6 s window has never been measured directly -- it is inferred from
 * per-sample deviation, and the weakest real departure seen right-side up is
 * 0.050 against a rest MAX deviation of 0.0257, i.e. under 2x separation. That
 * is thin, and it is why the derived value is PRINTED at READY: read it on the
 * bench, compare against the departures in the same log, and tune Z_THRESH_MARGIN
 * from data rather than from this comment.
 *
 * 🔴 FAILURE DIRECTIONS. Too low -> false departure -> beacon on a stationary
 * car for LATCH_FAILSAFE_MS (600 s). Too high -> the backstop is dead again,
 * which is the state this replaces. The clamps bound both: Z_THRESH_MAX stops a
 * calibration taken during motion from disabling the detector, Z_THRESH_MIN
 * stops a freakishly quiet one from making it hair-trigger.
 */
#define Z_THRESH_MARGIN    (1.50f)   /* multiple of measured rest spread     */
#define Z_THRESH_MIN       (0.040f)  /* floor: below weakest departure seen  */
#define Z_THRESH_MAX       (0.200f)  /* ceiling: a moving calibration        */
#define Z_THRESH_FALLBACK  DEFAULT_THRESHOLD_VALUE

/*
 * Skip the head of the calibration window while the 32-sample average is still
 * converging. Measured 2026-08-18, first samples of a real window:
 *
 *     avg = 7.499, 8.522, 9.758, 9.757, 9.755, ...
 *
 * The |z|>5 priming guard does NOT exclude 7.499/8.522, so the spread came out
 * 2.26 and the derived threshold CLAMPED to Z_THRESH_MAX (printed as exactly
 * 0.200000) -- the backstop dead, which is the state this whole change exists
 * to fix. The average spans 32 samples at 25 Hz = 1.28 s, so 1500 ms clears it
 * with margin and still leaves 4.5 s of the 6 s window to measure noise in.
 */
#define Z_CAL_SETTLE_MS    (1500UL)

/* Raw-sample arrival peak, maintained in the sampling ISR. See main.cpp. */
extern void  arrival_peak_reset();
extern float arrival_peak_get();
extern bool  arrival_peak_hit();
extern void  arrival_zero_set(float z);
extern void  burst_trigger(uint8_t post, uint8_t kind);

/* Ramp detector, maintained in the sampling ISR. See the RAMP_* block there. */
extern bool     ramp_hit();
extern uint16_t ramp_mean_get();
extern uint8_t  ramp_dir_get();

MovementService::MovementService(RollingAvg<float> *acc_avg)
{
    acceleration_avg_ref = acc_avg;
    reset_counter = 100;
    zero_calib_value = 0.0;

    Serial.print(F("Initialing FSM \r\n"));

    state = MotionStates::STATE_ERROR_RESET;
    last_state = MotionStates::STATE_ERROR_RESET;

    any_motion_pending = false;
    arrival_seen       = false;
    stop_confirm_timer = 0;
    monitor_entered_ms = 0;
    rearm_cleared      = false;
    arrival_edge_count = 0;
    arrival_edge_first = 0;
    vel_departure      = 0.0f;
    vel_reported       = false;
    ramp_reported      = false;
    calib_moved        = false;
    calib_attempts     = 0;

    /*
     * Until a calibration completes there is no learned z noise floor, so the
     * polled gate holds the compile-time fallback rather than 0.0 -- which
     * would make every sample a departure.
     */
    threshold_value    = Z_THRESH_FALLBACK;
    z_cal_min          =  1.0e9f;
    z_cal_max          = -1.0e9f;
}

/*
 * Record that the BMA456 any-motion interrupt fired.
 *
 * Two states use it, for opposite purposes:
 *
 *   STATE_MONITORING  a departure -- latch the alarm
 *   STATE_MOVING      an arrival  -- release it, once clustered
 *
 * Every other state ignores it.
 *
 * Called from loop(), not from the ISR. The ISR does nothing but count.
 *
 * THE EDGE IS JUDGED BY WHEN IT HAPPENED, NOT WHEN IT ARRIVED HERE.
 *
 * Two ways a stop would otherwise re-latch as a departure, alarming twice for
 * one trip:
 *
 *  1. Ordering. loop() calls fsm_run() BEFORE emit_acc_int_log(). An interrupt
 *     raised while the FSM was still in STATE_DECELERATING can therefore be
 *     delivered here in the same pass in which the FSM has already reached
 *     STATE_MONITORING -- so a plain state check would accept an edge that
 *     belongs to the arrival.
 *  2. Residual motion. A harsh stop keeps generating any-motion after the FSM
 *     has decided the car arrived: ringing in the rails, doors, passengers.
 *
 * Comparing edge_ms against when MONITORING was entered handles both with one
 * test. The subtraction is done signed so an edge timestamped before entry
 * (case 1) goes negative rather than wrapping to a huge positive.
 */
void MovementService::notify_any_motion(uint32_t edge_ms)
{
    /*
     * While MOVING, any-motion is the PRIMARY arrival detector -- it caught 3
     * of 3 arrivals against polling's 2 of 3, and the one polling missed left
     * the unit alarming at rest.
     *
     * A single edge is not enough: cruise produced isolated singles (1 in 97 s).
     * Requiring ARRIVAL_EDGE_COUNT inside ARRIVAL_EDGE_WINDOW_MS means the
     * condition survived more than one status poll, i.e. sustained motion.
     * Both observed arrivals gave 3 edges inside 1.8 s.
     */
    if (state == MotionStates::STATE_MOVING) {
        /*
         * Discard edges raised while the piezo is driven.
         *
         * An edge that arrives with the buzzer on is not evidence about the
         * car -- the alarm shakes the accelerometer, and the sensor's own
         * engine sits behind the same mechanical coupling the polled path does
         * (§5, §11). This is the interrupt-domain equivalent of the ringdown
         * blanking item 7 applies to sampling.
         *
         * Measured 2026-08-07, edges that formed arrival clusters:
         *
         *   run 1  n=3 bz=0, n=4 bz=0   real arrival, a fell to 5.29
         *   run 2  n=7 bz=1, n=8 bz=0   FALSE -- released mid-ride
         *   run 3  n=13 bz=0, n=14 bz=1 polled path caught it first
         *
         * Counting quiet edges only leaves run 1 detected and run 2 rejected.
         *
         * ⚠️ "~45% of each beep cycle quiet" was true when this was written. The
         * sequence-aligned alarm (2026-08-13, alarm.h) makes it 600/800 = 75%,
         * so MORE real arrival edges are trusted than this comment assumes --
         * the sensitivity argument below is now conservative, not optimistic.
         *
         * Two second-order effects of that change, neither designed:
         *
         * 1. 0.75 is the exact quiet fraction that caused the 2026-08-07 false
         *    release, when cruise edges paired into an arrival cluster. It is
         *    safe now ONLY because this path ANDs in arrival_peak_hit() and
         *    cruise peaks (0.10-0.32) cannot reach the 0.45 gate. Do not remove
         *    that AND.
         * 2. The status poll and the beep cycle used to be locked 2:1 (1000 ms
         *    against 500 ms), so every poll sampled the SAME phase of the beep
         *    and edge decimation was deterministic. At 800 ms they drift with a
         *    4 s beat, so decimation is now pseudo-random. The 2026-08-07 notes
         *    argue that random decimation is what kept consecutive cruise polls
         *    from pairing, so this is probably favourable -- but it is a
         *    behaviour change nobody chose, and it is untested over a long
         *    continuous alarm.
         */
        if (get_buzzer_status()) {
            return;
        }

        if (arrival_edge_count == 0 ||
            (uint32_t)(edge_ms - arrival_edge_first) > ARRIVAL_EDGE_WINDOW_MS) {
            /* First edge, or the previous window has expired -- start again. */
            arrival_edge_first = edge_ms;
            arrival_edge_count = 1;
        } else if (arrival_edge_count < 0xFF) {
            arrival_edge_count++;
        }
        return;
    }

    /*
     * An any-motion edge during calibration means the counterweight moved
     * while its own noise floor was being measured. That is the one input
     * that pushes the learned threshold in the dangerous direction, so the
     * window is thrown away rather than trusted -- see CALIB_RETRIES.
     */
    if (state == MotionStates::STATE_CALIBERATION) {
        calib_moved = true;
        return;
    }

    if (state != MotionStates::STATE_MONITORING) {
        return;
    }

    /*
     * Re-arm blanking, rest-gated. See the MONITOR_REARM_QUIET block in the
     * header for the measured defect this closes and the risk it carries.
     *
     * rearm_cleared is latched by fsm_run() once the lateral has actually gone
     * quiet, so the blank ends when the car has settled rather than after a
     * fixed 6 s. MONITOR_REARM_MS still caps it, so this can only shorten the
     * window in which a real departure is thrown away.
     */
    if (!rearm_cleared &&
        (int32_t)(edge_ms - monitor_entered_ms) < (int32_t)MONITOR_REARM_MS) {
        Serial.print(F("FSM: any-motion ignored, re-arm blanking t="));
        Serial.print((int32_t)(edge_ms - monitor_entered_ms));
        Serial.print(F(" q="));
        Serial.print(lat_monitor.quiet_run());
        Serial.print(F("\r\n"));
        return;
    }

    any_motion_pending = true;
}

/*
 * State-Transition
 *
 *                       STATE_ERROR_RESET
 *                              |
 *                              V
 *                      STATE_CALIBERATION
 *                              |
 *                              V
 *                       STATE_MONITORING<------------+
 *                              |                     |
 *                              V                     |
 *                    STATE_MOVEMENT_DETECTED         |
 *                              |                     |
 *                              V                     |
 *                        STATE_MOVING                |
 *                              |                     |
 *                              V                     |
 *                       STATE_DECELERATING           |
 *                              |                     |
 *                              V                     |
 *                        STATE_STOPPED---------------+
 *
 */

void MovementService::fsm_run()
{
    float    present_accel = 0.0, delta_accel = 0.0;

    switch (state) {

    case MotionStates::STATE_CALIBERATION:
        if (last_state != MotionStates::STATE_CALIBERATION) {
            Serial.print(F("FSM: Performing Self Calibration \r\n"));
            last_state = MotionStates::STATE_CALIBERATION;
            /*
             * Learn the lateral noise floor over the same window that
             * measures the z zero. The deployment sequence guarantees this
             * runs at rest, on the counterweight, in the building -- see
             * lateral.h.
             */
            lat_monitor.calib_begin(millis());

            /*
             * Same window, same reasoning, for the Z noise floor -- see the
             * Z_THRESH_MARGIN block. Seeded so the first sample sets both ends.
             */
            z_cal_min =  1.0e9f;
            z_cal_max = -1.0e9f;
            calib_moved = false;

            /*
             * Nothing is armed yet, so the heartbeat must not claim otherwise.
             * This also covers re-entry via STATE_ERROR_RESET: a device that
             * has rebooted into a fresh calibration should stop asserting the
             * verdict from the deployment before it.
             */
            heartbeat_set(HEARTBEAT_OFF);
        }

        /*
         * Track the z average across the calibration window. Spread rather
         * than sigma: it needs two floats and no accumulation, and the window
         * is short enough that a single outlier cannot be hidden by a mean.
         * See the Z_THRESH_MARGIN block.
         */
        {
            float za = acceleration_avg_ref->avg();

            /*
             * ⚠️ ONLY ONCE THE AVERAGE HOLDS REAL GRAVITY. The FSM enters
             * calibration before the first accelerometer sample has been
             * folded in, so `avg` reads ~0 for the opening pass(es). Measured
             * 2026-08-18: including them made the spread 9.76 (9.759 - 0),
             * the derived threshold 14.6, and the value CLAMPED to
             * Z_THRESH_MAX -- printed as exactly 0.200000, which is how it was
             * caught. Unclamped that would have silently disabled the polled
             * detector for the whole deployment.
             *
             * Any plausible mounting reads |z| near 1 g in this window, so 5.0
             * separates "primed" from "not yet" with enormous margin and needs
             * no orientation assumption -- inverted reads -9.7, upright +9.76.
             */
            if ((millis() - start_timer) > Z_CAL_SETTLE_MS &&
                (za > 5.0f || za < -5.0f)) {
                if (za < z_cal_min) { z_cal_min = za; }
                if (za > z_cal_max) { z_cal_max = za; }
            }
        }

        if ((millis() - start_timer) > CALIB_TIMEOUT_MS) {
            bool ok;

            ok = lat_monitor.calib_finish();

            Serial.print(F("XY: calib b="));
            Serial.print(lat_monitor.buckets());
            Serial.print(F(" peak="));
            Serial.print(lat_monitor.calib_peak(), 4);
            Serial.print(F(" mv="));
            Serial.print(calib_moved ? 1 : 0);
            Serial.print(F("\r\n"));

            /*
             * ─── §XY-BMAX: the per-second maxima, in time order ──────────────
             *
             * WHAT THIS IS FOR. The customer asked whether the 10 s calibration
             * can be 5 s, or can end when the device judges itself stable. That
             * is answerable offline and ONLY from this sequence: the threshold a
             * 5 s window would have produced is
             *
             *     median(b0..b4) x XY_STILL_MARGIN, clamped
             *
             * and an early-exit rule is any predicate over the same numbers. The
             * device already computes them; they were simply never emitted.
             *
             * ⚠️ WHY NOT JUST LOG THE METRIC AT FULL RATE INSTEAD. Tempting --
             * `m=` is already in the sample line -- but LOG_DECIMATE_N thins it
             * 8:1, and un-thinning it during calibration would be actively
             * DANGEROUS. Serial.print blocks loop(), loop() drains the sample
             * ring, and ~20% of samples are already lost to overrun. The metric
             * is |Δax|+|Δay| between CONSECUTIVE samples, so a lost sample
             * widens dt and INFLATES m -- which inflates the learned threshold,
             * which is the direction that makes travel read as still. Adding
             * serial load inside the measurement window would bias the very
             * number being measured, the wrong way. This line costs nothing
             * because it is emitted AFTER the window has closed.
             *
             * Printed on EVERY attempt, including the ones that get retried --
             * a rejected window is exactly the case worth studying.
             */
            Serial.print(F("XY: bmax"));
            for (uint8_t i = 0; i < lat_monitor.buckets(); i++) {
                Serial.print(F(" "));
                Serial.print(lat_monitor.bucket(i), 4);
            }
            Serial.print(F("\r\n"));

            /*
             * Retry rather than trust a window that saw movement or came back
             * short of samples. The z zero is recomputed with it -- a window
             * that cannot be trusted for x/y cannot be trusted for z either.
             */
            if ((!ok || calib_moved) && calib_attempts < CALIB_RETRIES) {
                calib_attempts++;
                Serial.print(F("FSM: NOT READY - recalibrating, attempt "));
                Serial.print(calib_attempts + 1);
                Serial.print(F("\r\n"));
                start_timer = millis();
                lat_monitor.calib_begin(millis());
                calib_moved = false;
                break;
            }

            if (!ok || calib_moved) {
                lat_monitor.arm_fallback();
                Serial.print(F("FSM: READY (fallback) - site not measurable, "
                               "expect a twitchy device \r\n"));
            } else if (lat_monitor.noisy()) {
                Serial.print(F("FSM: READY (noisy) - lateral floor at the "
                               "clamp \r\n"));
            } else {
                Serial.print(F("FSM: READY \r\n"));
            }

            /*
             * CAPTURE THE ZERO BEFORE MAKING ANY NOISE.
             *
             * ready_signal() drives the piezo directly and does NOT set
             * alarm_status_g, so the timer ISR keeps sampling right through
             * it -- unlike enable_alarm(), which the old code used here and
             * which blanks the sensor as a side effect. A 4-sample average at
             * 3.13 Hz spans 1.28 s, so reading it after a chirp of up to 1 s
             * would capture a baseline built mostly out of piezo ringing and
             * hand it to the FSM as "at rest" for the whole deployment.
             */
            zero_calib_value = acceleration_avg_ref->avg();

            /*
             * Derive the polled departure threshold from the z noise measured
             * in THIS deployment, clamped both ways -- see Z_THRESH_MARGIN.
             * A window that produced no samples falls back to the constant
             * rather than to an arbitrary small number.
             */
            /*
             * No separate spread print: `Threshold-Value` below already
             * carries the result at 6 decimals, and the clamps are legible in
             * it -- exactly 0.040000 or 0.200000 means the value was clamped
             * rather than learned. Flash headroom is 60-500 bytes on this
             * build; a redundant string is not worth it.
             */
            if (z_cal_max >= z_cal_min) {
                threshold_value = (z_cal_max - z_cal_min) * Z_THRESH_MARGIN;
                if (threshold_value < Z_THRESH_MIN) { threshold_value = Z_THRESH_MIN; }
                if (threshold_value > Z_THRESH_MAX) { threshold_value = Z_THRESH_MAX; }
            } else {
                threshold_value = Z_THRESH_FALLBACK;
            }

            /* The raw peak detector measures against the same baseline. */
            arrival_zero_set(zero_calib_value);

            Serial.print(F("-------------------------------------\r\n"));
            Serial.print(F("Zero-Calib-Value : "));
            Serial.print(zero_calib_value, 6);
            Serial.print(F("\r\n"));
            Serial.print(F("Threshold-Value  : "));
            Serial.print(threshold_value, 6);
            Serial.print(F("\r\n"));
            Serial.print(F("XY-Still-Value   : "));
            Serial.print(lat_monitor.still(), 4);
            Serial.print(F("\r\n"));
            Serial.print(F("-------------------------------------\r\n"));

            /*
             * Now make the noise, and re-anchor afterwards so the first
             * monitored metric is not a difference across the chirp.
             */
            {
                bool degraded = (!ok || calib_moved || lat_monitor.noisy());

                ready_signal(degraded ? 3 : 1);

                /*
                 * Arm the idle heartbeat with the same verdict the chirp just
                 * announced. The chirp is heard once and only by someone still
                 * standing there; the heartbeat keeps "armed" -- and "armed but
                 * the site could not be measured" -- visible for the whole
                 * deployment. See the IDLE HEARTBEAT block in alarm.h.
                 */
                heartbeat_set(degraded ? HEARTBEAT_DEGRADED : HEARTBEAT_OK);
            }

            lat_monitor.drop_anchor();

            /*
             * And flush the z average for the same reason STATE_STOPPED does:
             * the chirp is still sitting in the window, and a 4-sample
             * average holding it would clear DEFAULT_THRESHOLD_VALUE the
             * instant monitoring begins -- the unit would announce a
             * departure caused by its own ready signal. MONITOR_REARM_MS
             * covers the any-motion side of the same problem.
             */
            acceleration_avg_ref->fill(zero_calib_value);

            set_state(STATE_MONITORING);
        }
        break;

    case MotionStates::STATE_MONITORING:
        if (last_state != MotionStates::STATE_MONITORING) {
            Serial.print(F("FSM: Transitioned to STATE_MONITORING \r\n"));
            last_state = MotionStates::STATE_MONITORING;
            /*
             * Start the re-arm blanking window, and discard anything already
             * queued -- it can only have come from the run that just ended.
             */
            monitor_entered_ms = millis();
            any_motion_pending = false;
            rearm_cleared      = false;

            /*
             * Clear the window for the same reason MONITOR_REARM_MS exists:
             * the arrival that just ended is still inside it, and left there it
             * would re-latch as a fresh departure and alarm twice for one trip.
             * vel_departure is cleared so a stale value cannot scale the next
             * run's conservation test.
             */
            vel_window.reset(millis());
            vel_departure = 0.0f;
            vel_reported  = false;
        }

        /*
         * End the re-arm blank as soon as the car has actually settled.
         *
         * quiet_run() is driven to 0 by the ringing a stop produces and climbs
         * once it passes, so reaching MONITOR_REARM_QUIET is a direct
         * measurement of "settled" rather than the fixed 6 s guess. Latched:
         * once cleared it stays cleared for this MONITORING episode, so ordinary
         * ambient jitter afterwards cannot re-blank a departure.
         *
         * MONITOR_REARM_MIN_MS floors it so the loop()-ordering case and the
         * first instants of ringing are still covered.
         */
        if (!rearm_cleared &&
            (millis() - monitor_entered_ms) >= MONITOR_REARM_MIN_MS &&
            lat_monitor.quiet_run() >= MONITOR_REARM_QUIET) {
            rearm_cleared = true;
            Serial.print(F("FSM: re-arm blank cleared early, settled at "));
            Serial.print(millis() - monitor_entered_ms);
            Serial.print(F(" ms\r\n"));
        }

        present_accel = acceleration_avg_ref->avg();
        delta_accel = 0.0;

        if (present_accel > zero_calib_value)
            delta_accel = present_accel - zero_calib_value;
        else if (present_accel < zero_calib_value)
            delta_accel = zero_calib_value - present_accel;

        /*
         * If there is an acceleration in either direction, then transition to
         * STATE_MOVEMENT_DETECTED state.
         */

        if (delta_accel > threshold_value) {
            start_timer = millis();
            set_state(STATE_MOVEMENT_DETECTED);
        }
        /*
         * Departure via the BMA456 any-motion interrupt.
         *
         * This is the path that actually catches slow departures. The threshold
         * test above operates on a 4-sample average at 3.13 Hz, which spans
         * 1.28 s and flattens a 1-2 s departure ramp to nothing -- it has never
         * detected a departure in any hoistway run (§12.3). The sensor's own
         * engine runs at 100 Hz and caught an 18 fpm departure of -0.116 m/s^2.
         *
         * Both paths are kept: the threshold test still catches violent events
         * the sensor might somehow miss, and costs nothing to leave in.
         */
        if (any_motion_pending) {
            any_motion_pending = false;
            Serial.print(F("FSM: Departure latched (any-motion) \r\n"));
            burst_trigger(BURST_POST_DEP, 0);
            vel_departure = vel_window.valid() ? vel_window.w() : 0.0f;
            start_timer = millis();
            set_state(STATE_MOVEMENT_DETECTED);
            break;
        }

        /*
         * Departure via the windowed velocity integral. OR'd with the two
         * paths above, never AND'd: §14.4 makes a missed departure the only
         * catastrophic failure, so an extra detector may only ever make the
         * unit alarm sooner.
         *
         * This is the path that reaches a departure the others physically
         * cannot. A car ramping to 20 fpm over 4 s averages 0.025 m/s^2 --
         * six times under ANYMOTION_THRESHOLD -- and its 4-sample average at
         * 3.13 Hz never approaches DEFAULT_THRESHOLD_VALUE either. The integral
         * sees 0.1 m/s of velocity change regardless of how the ramp is shaped.
         *
         * At the CURRENT sample rate VEL_DEPART_THRESHOLD works out at 22.6 fpm
         * (5 sigma of measured parked noise), so this does not yet meet the
         * 20 fpm requirement on its own -- any-motion above still carries the
         * slow cases it has been catching all along. velocity.h has the sample
         * rate table and the reason the threshold cannot simply be lowered.
         */
        if (vel_window.valid() && fabs(vel_window.w()) > VEL_DEPART_THRESHOLD) {

            /*
             * Report on the RISING crossing only. Unarmed, the condition can
             * persist for several seconds and one line per sample would drown
             * the capture.
             */
            if (!vel_reported) {
                vel_reported = true;
                vel_departure = vel_window.w();

                Serial.print(F("VEL: departure w="));
                Serial.print(vel_departure, 3);
                Serial.print(F(" cov="));
                Serial.print(vel_window.coverage_ms());
#if VEL_ARMED
                Serial.print(F(" LATCH\r\n"));
                start_timer = millis();
                set_state(STATE_MOVEMENT_DETECTED);
#else
                /*
                 * Not armed -- see velocity.h. The window is characterised in
                 * the log and the FSM is left alone, exactly as §11 introduced
                 * any-motion before §12 promoted it.
                 */
                Serial.print(F(" obs\r\n"));
                vel_departure = 0.0f;
#endif
            }
        } else {
            vel_reported = false;
        }
        break;

    case MotionStates::STATE_MOVEMENT_DETECTED:
        if (last_state != MotionStates::STATE_MOVEMENT_DETECTED) {
            Serial.print(F("FSM: Transitioned to STATE_MOVEMENT_DETECTED \r\n"));
            last_state = MotionStates::STATE_MOVEMENT_DETECTED;
        }

        /*
         * The departure ramp is still running through this state, so keep the
         * largest |w| seen rather than only the value that tripped the latch.
         * That peak is what the arrival conservation test is scaled against,
         * and a latch driven by any-motion may have fired well before the
         * integral had accumulated the full velocity step.
         */
        if (vel_window.valid() && fabs(vel_window.w()) > fabs(vel_departure)) {
            vel_departure = vel_window.w();
        }

        if ((millis() - start_timer) > MOVEMENT_DETECTION_TIMEOUT_MS) {
            movement_start_timer = millis();
            arrival_seen       = false;
            arrival_edge_count = 0;
            arrival_edge_first = 0;

            /*
             * Clear the window before cruise so the departure transient cannot
             * still be sitting in it when arrival detection arms. MIN_TRAVEL_MS
             * (3 s) and VEL_MIN_COVERAGE_MS together mean the window has
             * refilled from cruise data before anything is read from it.
             */
            vel_window.reset(millis());

            Serial.print(F("FSM: departure velocity w="));
            Serial.print(vel_departure, 3);
            Serial.print(F("\r\n"));

            enable_alarm();
            enable_chase_leds();
            set_state(STATE_MOVING);
        }
        break;

    case MotionStates::STATE_MOVING:
        if (last_state != MotionStates::STATE_MOVING) {
            Serial.print(F("FSM: Transitioned to STATE_MOVING \r\n"));
            last_state = MotionStates::STATE_MOVING;
            /*
             * Quiet observed before the beacon started says nothing about the
             * run that has just begun.
             */
            lat_monitor.clear_quiet();
            jog_release_pending = false;   /* no verdict from a past run   */
            ramp_reported       = false;

            /*
             * Start the raw peak detector and the ramp detector for this
             * run. Both stay disarmed until the departure transient has
             * died away -- see main.cpp.
             */
            arrival_peak_reset();
        }

        /*
         * In this state, there are 2 timeouts. They are
         * a. Buzzer Timeout
         * b. Chase LED timeout
         * 
         * After Buzzer timeout happens, we will stop the buzzer, but the chase 
         * LED will continue to blink, till the chase LED timeout happens.
         * This is to because when the buzzer is triggered, there is still some 
         * vibration and this will keep adding to the averaging logic.
         *
         */

        current_time = millis();

        /*
         * Latched: the alarm holds from departure until an arrival transient is
         * seen. No timeout -- a 3-minute express run alarms for 3 minutes, which
         * is the §3 design and the answer to the original complaint.
         *
         * Constant velocity is deliberately NOT tested for. It cannot be: a car
         * cruising at 350 fpm and a car parked both read 1 g on Z (§3). The
         * middle phase is assumed, not measured.
         *
         * Arrival is detected on the polled average rather than any-motion,
         * because the buzzer triggers any-motion continuously while sounding
         * (§11) -- there is no usable edge. The polled path can do it: item 7
         * restored ~45% sample coverage during an alarm (§5), and arrivals are
         * large (raw 1.9-4.8 m/s^2 measured).
         */
        present_accel = acceleration_avg_ref->avg();
        delta_accel = 0.0;

        if (present_accel > zero_calib_value)
            delta_accel = present_accel - zero_calib_value;
        else if (present_accel < zero_calib_value)
            delta_accel = zero_calib_value - present_accel;

        /*
         * Jog verdict release (main.cpp, JOG_VERDICT_ARMED). Checked before
         * the arrival tests: the verdict arrives 3.2 s after the latch, when
         * MIN_TRAVEL_MS still has the tests below blind -- that blindness IS
         * the jog defect. Fires through the failsafe's own exit. Measured
         * basis: 15/15 verdicts, populations opk <=440 (real) vs >=1914
         * (jog), falcon_jog_verdict_2026-08-11.md. Armed on Dave's decision
         * 2026-08-11 with n=4 jogs from one car and one day -- if a false
         * JOG ever silences a moving car, disarm first, diagnose second.
         *
         * SUPERSEDED 2026-08-20 (falcon_corpus_labelled_2026-08-20.md): on
         * 63 labelled real departures and 17 labelled jogs the populations
         * OVERLAP -- real 11-562 plus one at 972, jogs 381-4154 -- and the
         * measured rate is 1 false JOG in 63 runs, 1 miss in 17 jogs. The
         * false JOG the comment above hopes never happens is on file, at
         * opk 972 (260818-105317.log r44). See main.cpp at the gate.
         */
        if (jog_release_pending) {
            jog_release_pending = false;
            Serial.print(F("FSM: Release (jog verdict) \r\n"));
            start_timer = millis();
            set_state(STATE_DECELERATING);
            break;
        }

        /*
         * MIN_TRAVEL_MS keeps the departure transient itself from satisfying any
         * of the tests below and releasing the latch straight after setting it.
         */
        if (!arrival_seen && (current_time - movement_start_timer) > MIN_TRAVEL_MS) {

            /*
             * 1. PRIMARY -- clustered any-motion, CORROBORATED by the polled
             *    average. See notify_any_motion() and ARRIVAL_CLUSTER_DELTA.
             *
             * A cluster on its own is not sufficient. At 58 fpm the car's own
             * vibration keeps any-motion asserted, so consecutive status polls
             * pair up and look exactly like an arrival -- on 2026-08-07 that
             * released an alarm 12 s into a 54 s ride. Requiring the average to
             * be elevated as well rejects that case (0.175) while still
             * accepting a real arrival the polled path alone would have missed
             * (0.278 against its own 0.30 threshold).
             */
            if (arrival_edge_count >= ARRIVAL_EDGE_COUNT &&
                (uint32_t)(current_time - arrival_edge_first) <= ARRIVAL_EDGE_WINDOW_MS &&
                arrival_peak_hit()) {

                arrival_seen = true;
                Serial.print(F("FSM: Arrival (any-motion), edges "));
                Serial.print(arrival_edge_count);
                Serial.print(F(" peak "));
                Serial.print(arrival_peak_get(), 3);
                Serial.print(F("\r\n"));
                start_timer = millis();
                set_state(STATE_DECELERATING);
                break;
            }

            /*
             * (1b, the velocity-conservation release, was removed in the
             * 2026-08-12 cleanup. Measured dead 2026-08-11: a drive ramp
             * integrates to 67% of true speed and a brake shock to 128%, so
             * neither end of a departure-to-arrival comparison is reliable --
             * falcon_signature_2026-08-11.md §4e.1. Last version at 8308cdd.)
             */

            /*
             * 2. SECONDARY -- polled transient on the RAW windowed peak.
             * Independent of the interrupt path, so it still works if that
             * path fails.
             */
            if (arrival_peak_hit()) {
                arrival_seen = true;
                Serial.print(F("FSM: Arrival (polled), peak "));
                Serial.print(arrival_peak_get(), 3);
                Serial.print(F("\r\n"));
                start_timer = millis();
                set_state(STATE_DECELERATING);
                break;
            }

            /*
             * 3. RAMP -- sustained one-signed deceleration at the drive's
             * plateau. The path built for the stop that has no brake
             * transient: a drive-controlled deceleration to standstill,
             * where the windowed peak passes on a 1.009x margin or not at
             * all. Detector and measured basis in main.cpp's RAMP_* block.
             *
             * Even unarmed this fires the arrival burst: the ramp verdict
             * lands mid-deceleration, BEFORE the peak crosses on a slow
             * drive stop, which is the "own trigger below the arrival
             * threshold" the slow-stop capture has been missing
             * (falcon_signature §4e.2).
             */
            if (!ramp_reported && ramp_hit()) {
                ramp_reported = true;
                Serial.print(F("FSM: Arrival (ramp) mean="));
                Serial.print(ramp_mean_get());
                Serial.print(F(" dir="));
                Serial.print(ramp_dir_get());
#if RAMP_ARMED
                Serial.print(F(" (armed)\r\n"));
                arrival_seen = true;
                burst_trigger(BURST_POST_ARR, 1);
                start_timer = millis();
                set_state(STATE_DECELERATING);
                break;
#else
                Serial.print(F(" (unarmed)\r\n"));
                burst_trigger(BURST_POST_ARR, 1);
#endif
            }

        }

        /*
         * Expire a stale cluster so isolated edges spread over a long ride
         * cannot accumulate into a false arrival.
         */
        if (arrival_edge_count > 0 &&
            (uint32_t)(current_time - arrival_edge_first) > ARRIVAL_EDGE_WINDOW_MS) {
            arrival_edge_count = 0;
        }

        /*
         * Failsafe only. Reaching this means an arrival was never detected, which
         * is a fault -- log it as one so it is not mistaken for a normal release.
         */
        if ((current_time - movement_start_timer) > LATCH_FAILSAFE_MS) {
            Serial.print(F("FSM: FAILSAFE - no arrival detected, releasing latch \r\n"));
            start_timer = millis();
            set_state(STATE_DECELERATING);
        }
        break;
    case MotionStates::STATE_DECELERATING:
        if (last_state != MotionStates::STATE_DECELERATING) {
            Serial.print(F("FSM: Transitioned to STATE_DECELERATING \r\n"));
            last_state = MotionStates::STATE_DECELERATING;
            stop_confirm_timer = millis();

            /*
             * Record the stop at full rate. Placed at the entry to this state
             * rather than at each arrival test, so it catches EVERY release
             * path -- clustered any-motion, polled peak, velocity conservation
             * and the failsafe -- from one call.
             *
             * Mostly pre-trigger: by the time any of those have fired, the stop
             * is underway or over. See burst_trigger() in main.cpp.
             *
             * The question it exists to answer: a smooth drive-controlled
             * deceleration should integrate like a departure does, which is
             * what the velocity conservation release depends on. The only stop
             * measured so far was a jog's brake ring, and that did NOT
             * integrate -- it flips sign almost every sample, so its area came
             * out at -0.011 m/s where physics demands -0.18. Whether a smooth
             * stop behaves better is assumed, not known, and it is the
             * assumption the whole approach rests on.
             */
            burst_trigger(BURST_POST_ARR, 1);
        }

        /*
         * Confirm the car has actually settled before silencing, rather than
         * silencing on a fixed timer. The arrival transient is followed by
         * ringing and door operation; releasing on the first quiet sample would
         * clear the alarm while the car is still rocking.
         *
         * The window restarts on any excursion, so the average has to stay
         * inside the band continuously.
         */
        present_accel = acceleration_avg_ref->avg();
        delta_accel = 0.0;

        if (present_accel > zero_calib_value)
            delta_accel = present_accel - zero_calib_value;
        else if (present_accel < zero_calib_value)
            delta_accel = zero_calib_value - present_accel;

        if (delta_accel > STOP_BAND_VALUE) {
            stop_confirm_timer = millis();
        }

        if ((millis() - stop_confirm_timer) > STOP_CONFIRM_MS) {
            disable_alarm();
            disable_chase_leds();
            set_state(STATE_STOPPED);
        }

        /*
         * Backstop: never sit in DECELERATING indefinitely if the car keeps
         * moving enough to restart the confirm window.
         */
        if ((millis() - start_timer) > LATCH_FAILSAFE_MS) {
            Serial.print(F("FSM: FAILSAFE - never settled, forcing stop \r\n"));
            disable_alarm();
            disable_chase_leds();
            set_state(STATE_STOPPED);
        }
        break;

    case MotionStates::STATE_STOPPED:
        if (last_state != MotionStates::STATE_STOPPED) {
            Serial.print(F("FSM: Transitioned to STATE_STOPPED \r\n"));
            last_state = MotionStates::STATE_STOPPED;

            acceleration_avg_ref->fill(zero_calib_value);
        }
        set_state(STATE_MONITORING);
        break;

    case MotionStates::STATE_ERROR_RESET:
        if (last_state != MotionStates::STATE_ERROR_RESET) {
            Serial.print(F("FSM: Transitioned to STATE_ERROR_RESET \r\n"));
            last_state = MotionStates::STATE_ERROR_RESET;
        }

        //Serial.print(F("FSM: Device in ERROR_RESET state \r\n"));
        if (reset_counter == 0) {
            set_state(STATE_CALIBERATION);
            start_timer = millis();
        } else {
            reset_counter--;
        }
        break;

    default:
        break;

    }

    return;
}

void MovementService::set_state(int arg_state)
{
    state = arg_state;
}

/* See the declaration comment in movement_service.h. */
void MovementService::jog_release(void)
{
    if (state == MotionStates::STATE_MOVING) {
        jog_release_pending = true;
    }
}

int MovementService::get_state()
{
    return (state);
}