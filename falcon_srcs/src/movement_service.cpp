#include "movement_service.h"

//#define DEFAULT_THRESHOLD_VALUE 0.005
#define DEFAULT_THRESHOLD_VALUE 0.40

extern float get_threshold_data();

MovementService::MovementService(RollingAvg<float> *acc_avg, float *acc_mss, float *adj_acc, float *vel_ms, RollingAvg<float> *pres_avg)
{
    acc_varience_counter = 0;
    pressure_varience_counter = 0;
    timer_ms = 0;
    vel_threshold = 0.0;
    acceleration_avg_ref = acc_avg;
    acc_mss_ref = acc_mss;
    adj_acc_ref = adj_acc;
    vel_ms_ref = vel_ms;
    pressure_avg_ref = pres_avg;
    variance_pres = 0.0;
    reset_counter = 100;
    zero_calib_value = 0.0;

    Serial.print(F("Initialing FSM \r\n"));

    state = MotionStates::STATE_ERROR_RESET;
    last_state = MotionStates::STATE_ERROR_RESET;
    log_printed = 0;

    monitor_state = MonitorStates::MONITORING;

#if LATCHED_FSM
    any_motion_pending = false;
    arrival_seen       = false;
    stop_confirm_timer = 0;
    monitor_entered_ms = 0;
#endif
}

#if LATCHED_FSM
/*
 * Record that the BMA456 any-motion interrupt fired.
 *
 * Only STATE_MONITORING acts on it. That matters: the buzzer triggers
 * any-motion continuously while it sounds (Eng_Notes §11), so once the alarm is
 * running these arrive constantly. Latching only from MONITORING makes those
 * harmless -- there is nothing for them to do.
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
    if (state != MotionStates::STATE_MONITORING) {
        return;
    }

    if ((int32_t)(edge_ms - monitor_entered_ms) < (int32_t)MONITOR_REARM_MS) {
        Serial.print(F("FSM: any-motion ignored, re-arm blanking \r\n"));
        return;
    }

    any_motion_pending = true;
}
#endif

void MovementService::reset_counters(void)
{
    timer_ms = millis();
    acc_varience_counter = 0;
    pressure_varience_counter = 0;
    variance_pres = 0.0;
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

    case MotionStates::STATE_NOT_MOVING:
        if (last_state != MotionStates::STATE_NOT_MOVING) {
            Serial.print(F("FSM: Transitioned to STATE_NOT_MOVING \r\n"));
            last_state = MotionStates::STATE_NOT_MOVING;
        }
        break;

    case MotionStates::STATE_CALIBERATION:
        if (last_state != MotionStates::STATE_CALIBERATION) {
            Serial.print(F("FSM: Performing Self Calibration \r\n"));
            last_state = MotionStates::STATE_CALIBERATION;
        }
        if ((millis() - start_timer) > (CALIB_TIMEOUT_MS - 1000)) {
            enable_alarm();
        }

        if ((millis() - start_timer) > CALIB_TIMEOUT_MS) {
            zero_calib_value = acceleration_avg_ref->avg();
            threshold_value = DEFAULT_THRESHOLD_VALUE;

            set_state(STATE_MONITORING);
            Serial.print(F("-------------------------------------\r\n"));
            Serial.print(F("Zero-Calib-Value : "));
            Serial.print(zero_calib_value, 6);
            Serial.print(F("\r\n"));
            Serial.print(F("Threshold-Value  : "));
            Serial.print(threshold_value, 6);
            Serial.print(F("\r\n"));
            Serial.print(F("-------------------------------------\r\n"));
            disable_alarm();
        }
        break;

    case MotionStates::STATE_MONITORING:
        if (last_state != MotionStates::STATE_MONITORING) {
            Serial.print(F("FSM: Transitioned to STATE_MONITORING \r\n"));
            last_state = MotionStates::STATE_MONITORING;
#if LATCHED_FSM
            /*
             * Start the re-arm blanking window, and discard anything already
             * queued -- it can only have come from the run that just ended.
             */
            monitor_entered_ms = millis();
            any_motion_pending = false;
#endif
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
#if LATCHED_FSM
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
            start_timer = millis();
            set_state(STATE_MOVEMENT_DETECTED);
        }
#endif
        break;

    case MotionStates::STATE_MOVEMENT_DETECTED:
        if (last_state != MotionStates::STATE_MOVEMENT_DETECTED) {
            Serial.print(F("FSM: Transitioned to STATE_MOVEMENT_DETECTED \r\n"));
            last_state = MotionStates::STATE_MOVEMENT_DETECTED;
        }

        if ((millis() - start_timer) > MOVEMENT_DETECTION_TIMEOUT_MS) {
            movement_start_timer = millis();
#if LATCHED_FSM
            arrival_seen = false;
#endif
            enable_alarm();
            enable_chase_leds();
            set_state(STATE_MOVING);
        }
        break;

    case MotionStates::STATE_MOVING:
        if (last_state != MotionStates::STATE_MOVING) {
            Serial.print(F("FSM: Transitioned to STATE_MOVING \r\n"));
            last_state = MotionStates::STATE_MOVING;
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

#if LATCHED_FSM
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
         * MIN_TRAVEL_MS keeps the departure transient itself from satisfying the
         * arrival test and releasing the latch immediately after setting it.
         */
        if (!arrival_seen &&
            (current_time - movement_start_timer) > MIN_TRAVEL_MS &&
            delta_accel > ARRIVAL_THRESHOLD_VALUE) {

            arrival_seen = true;
            Serial.print(F("FSM: Arrival transient, delta "));
            Serial.print(delta_accel, 6);
            Serial.print(F("\r\n"));
            start_timer = millis();
            set_state(STATE_DECELERATING);
            break;
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
#else
        /*
         * If the movement has stopped, then we will stop the buzzer and chase LED
         * after 5 seconds. This is to avoid false alarm.
         *
         * If the movement has not stopped, then we will keep the buzzer and chase LED
         * on for 30 seconds. After that, we will stop the buzzer and chase LED.
         *
         * If the movement has not stopped even after 30 seconds, then we will
         * transition to STATE_DECELERATING state.
         *
        */
        if ((current_time - movement_start_timer) > (STOP_TIMEOUT_MS - 5000) &&
            (current_time - movement_start_timer) < (STOP_TIMEOUT_MS)) {
#if 1
            if (log_printed == 0) {
                Serial.print(F("FSM: Buzzer timeout happened \r\n"));
                log_printed = 1;
            }
#endif
            disable_alarm();
        }

        if ((millis() - movement_start_timer) > STOP_TIMEOUT_MS) {

            log_printed = 0;
            Serial.print(F("FSM: Chase-LED timeout happened \r\n"));
            present_accel = acceleration_avg_ref->avg();
            delta_accel = 0.0;


            if (present_accel > zero_calib_value)
                delta_accel = present_accel - zero_calib_value;
            else if (present_accel < zero_calib_value)
                delta_accel = zero_calib_value - present_accel;

            Serial.print(F("FSM: Average : "));
            Serial.print(present_accel, 6);
            Serial.print(F("\r\n"));
            Serial.print(F("FSM: Delta   : "));
            Serial.print(delta_accel, 6);
            Serial.print(F("\r\n"));

            if (delta_accel > DEFAULT_THRESHOLD_VALUE) {
                Serial.print(F("FSM: Still in movement \r\n"));
                movement_start_timer = millis();
                enable_alarm();
                break;
            }
            set_state(STATE_DECELERATING);
        }

        break;
#endif

    case MotionStates::STATE_DECELERATING:
        if (last_state != MotionStates::STATE_DECELERATING) {
            Serial.print(F("FSM: Transitioned to STATE_DECELERATING \r\n"));
            last_state = MotionStates::STATE_DECELERATING;
#if LATCHED_FSM
            stop_confirm_timer = millis();
#endif
        }

#if LATCHED_FSM
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

        if (delta_accel > ARRIVAL_THRESHOLD_VALUE) {
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
#else
        if ((millis() - start_timer) > STOP_TIMEOUT_MS) {
//            disable_alarm();
            disable_chase_leds();
            set_state(STATE_STOPPED);
        }
        break;
#endif

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

/**
 * check the acceleration curve for 5 consecutive times with 100ms delay
 * if all has close to straight line, then the device is not accelerating
 */
bool MovementService::isAtRestOrStable()
{
    float acc_avg = acceleration_avg_ref->avg();

    if ((millis() - timer_ms) > (TEMP_WAIT)) {

        // calculate variance to acceleration curve
        if (fabs(acc_avg) < MOVING_ACC_THRESHOLD) {

            variance_acc = 0.0;
            for (int i =0; i < acceleration_avg_ref->size(); i++) {
                variance_acc += (acceleration_avg_ref->get(i) - acc_avg) * (acceleration_avg_ref->get(i) - acc_avg);
            }

            if (variance_acc < 0.001) {
                // Its almost straigt line, no acceleration. increment
                acc_varience_counter++;
            }
        }

        if (state >= MotionStates::STATE_DECELERATING) {
            float pres_avg = pressure_avg_ref->avg();
            variance_pres = 0.0;
            for (int i =0; i < pressure_avg_ref->size(); i++) {
                variance_pres += ((pressure_avg_ref->get(i) - pres_avg) * (pressure_avg_ref->get(i) - pres_avg));
            }

            if (variance_pres < 0.006) {
                // Its almost straigt line, no acceleration. increment
                pressure_varience_counter++;
            }
        }

        // its not accelerating over 400 ms, so decide its not moving
        if (acc_varience_counter > 4 || pressure_varience_counter > 4) {
            acc_varience_counter = 0;
            pressure_varience_counter = 0;

            return true;
        }

        timer_ms = millis();
    }

    return false;
}

/**
 * velocity should be above threshold. then its moving.
 */
inline bool MovementService::isMovingConfirmed(void)
{
    if ((vel_threshold < 0 && vel_threshold > (*vel_ms_ref)) ||
        (vel_threshold > 0 && vel_threshold < (*vel_ms_ref))) {
        return true;
    } 

    return false;
}

/**
 * velocity should be below threshold. then its decelerating.
 */
inline bool MovementService::isDecelerating(void)
{
    if ((vel_threshold < 0 && vel_threshold < (*vel_ms_ref)) || 
        (vel_threshold > 0 && vel_threshold > (*vel_ms_ref))) {
        return true;
    }

    return false;
}

/**
 * avg accleration must be above set threshold. Or it must be noise.
 */
inline bool MovementService::isStartedMoving(void)
{
    if (fabs(acceleration_avg_ref->avg()) >= MOVING_ACC_THRESHOLD) {
        return true;
    }

    return false;
}

inline void MovementService::startMonitoring()
{
    monitor_state = MonitorStates::MONITORING;
}

inline void MovementService::stopMonitoring()
{
    monitor_state = MonitorStates::NOT_MONITORING;
}

inline bool MovementService::isMonitoring()
{
    if (monitor_state == MonitorStates::MONITORING) {
        return true;
    }

    return false;
}

void MovementService::setErrorResetState()
{
    state = MotionStates::STATE_ERROR_RESET;
    (*vel_ms_ref) = 0;
    vel_threshold = 0;
    timer_ms = millis();
    disable_alarm();
    startMonitoring();
}

void MovementService::set_state(int arg_state)
{
    state = arg_state;
}

int MovementService::get_state()
{
    return (state);
}