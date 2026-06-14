#include "movement_service.h"

#define DEFAULT_THRESHOLD_VALUE 0.005

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

    Serial.print("Initialing FSM \r\n");

    state = MotionStates::STATE_ERROR_RESET;
    last_state = MotionStates::STATE_ERROR_RESET;

    monitor_state = MonitorStates::MONITORING;
}

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
            Serial.print("FSM: Transitioned to STATE_NOT_MOVING \r\n");
            last_state = MotionStates::STATE_NOT_MOVING;
        }
        break;

    case MotionStates::STATE_CALIBERATION:
        if (last_state != MotionStates::STATE_CALIBERATION) {
            Serial.print("FSM: Performing Self Calibration \r\n");
            last_state = MotionStates::STATE_CALIBERATION;
        }
        if ((millis() - start_timer) > (CALIB_TIMEOUT_MS - 1000)) {
            enable_alarm();
        }

        if ((millis() - start_timer) > CALIB_TIMEOUT_MS) {
            zero_calib_value = acceleration_avg_ref->avg();
            threshold_value = DEFAULT_THRESHOLD_VALUE;

            set_state(STATE_MONITORING);
            Serial.print("-------------------------------------\r\n");
            Serial.print("Zero-Calib-Value : ");
            Serial.print(zero_calib_value, 6);
            Serial.print("\r\n");
            Serial.print("Threshold-Value  : ");
            Serial.print(threshold_value, 6);
            Serial.print("\r\n");
            Serial.print("-------------------------------------\r\n");
            disable_alarm();
        }
        break;

    case MotionStates::STATE_MONITORING:
        if (last_state != MotionStates::STATE_MONITORING) {
            Serial.print("FSM: Transitioned to STATE_MONITORING \r\n");
            last_state = MotionStates::STATE_MONITORING;
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
        break;

    case MotionStates::STATE_MOVEMENT_DETECTED:
        if (last_state != MotionStates::STATE_MOVEMENT_DETECTED) {
            Serial.print("FSM: Transitioned to STATE_MOVEMENT_DETECTED \r\n");
            last_state = MotionStates::STATE_MOVEMENT_DETECTED;
        }

        if ((millis() - start_timer) > MOVEMENT_DETECTION_TIMEOUT_MS) {
            movement_start_timer = millis();
            enable_alarm();
            set_state(STATE_MOVING);
        }
        break;

    case MotionStates::STATE_MOVING:
        if (last_state != MotionStates::STATE_MOVING) {
            Serial.print("FSM: Transitioned to STATE_MOVING \r\n");
            last_state = MotionStates::STATE_MOVING;
        }

        /*
         * In this state, if the timeout happens, then we check whether
         * the average reading is still high. This is to because when the
         * buzzer is triggered, there is some vibration and this used to
         * keep adding into the averaging logic.
         */

        if ((millis() - movement_start_timer) > STOP_TIMEOUT_MS) {

            Serial.print("FSM: Alarm timeout happened \r\n");
            present_accel = acceleration_avg_ref->avg();
            delta_accel = 0.0;


            if (present_accel > zero_calib_value)
                delta_accel = present_accel - zero_calib_value;
            else if (present_accel < zero_calib_value)
                delta_accel = zero_calib_value - present_accel;

            Serial.print("FSM: Average : ");
            Serial.print(present_accel, 6);
            Serial.print("\r\n");
            Serial.print("FSM: Delta   : ");
            Serial.print(delta_accel, 6);
            Serial.print("\r\n");

            if (delta_accel > DEFAULT_THRESHOLD_VALUE) {
                Serial.print("FSM: Still in movement \r\n");
                movement_start_timer = millis();
                break;
            }
            set_state(STATE_DECELERATING);
        }

        break;

    case MotionStates::STATE_DECELERATING:
        if (last_state != MotionStates::STATE_DECELERATING) {
            Serial.print("FSM: Transitioned to STATE_DECELERATING \r\n");
            last_state = MotionStates::STATE_DECELERATING;
        }

        if ((millis() - start_timer) > STOP_TIMEOUT_MS) {
            disable_alarm();
            set_state(STATE_STOPPED);
        }
        break;

    case MotionStates::STATE_STOPPED:
        if (last_state != MotionStates::STATE_STOPPED) {
            Serial.print("FSM: Transitioned to STATE_STOPPED \r\n");
            last_state = MotionStates::STATE_STOPPED;

            acceleration_avg_ref->fill(zero_calib_value);
        }
        set_state(STATE_MONITORING);
        break;

    case MotionStates::STATE_ERROR_RESET:
        if (last_state != MotionStates::STATE_ERROR_RESET) {
            Serial.print("FSM: Transitioned to STATE_ERROR_RESET \r\n");
            last_state = MotionStates::STATE_ERROR_RESET;
        }

        //Serial.print("FSM: Device in ERROR_RESET state \r\n");
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
