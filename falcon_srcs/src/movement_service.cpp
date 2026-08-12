#include "movement_service.h"

#define DEFAULT_THRESHOLD_VALUE 0.40

/* Raw-sample arrival peak, maintained in the sampling ISR. See main.cpp. */
extern void  arrival_peak_reset();
extern float arrival_peak_get();
extern bool  arrival_peak_hit();
extern void  arrival_zero_set(float z);
extern void  burst_trigger(uint8_t post, uint8_t kind);

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
    arrival_edge_count = 0;
    arrival_edge_first = 0;
    vel_departure      = 0.0f;
    vel_reported       = false;
    calib_moved        = false;
    calib_attempts     = 0;
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
         * Item 7 leaves ~45% of each beep cycle quiet, so a real arrival still
         * has ample opportunity to register -- run 1's did, twice over.
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

    if ((int32_t)(edge_ms - monitor_entered_ms) < (int32_t)MONITOR_REARM_MS) {
        Serial.print(F("FSM: any-motion ignored, re-arm blanking \r\n"));
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
            calib_moved = false;
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
            threshold_value = DEFAULT_THRESHOLD_VALUE;

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
            ready_signal((!ok || calib_moved || lat_monitor.noisy()) ? 3 : 1);
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

        if (delta_accel > DEFAULT_THRESHOLD_VALUE) {
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

            /*
             * Start the raw peak detector for this run. It stays disarmed
             * until the departure transient has died away -- see main.cpp.
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