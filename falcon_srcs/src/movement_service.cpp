#include "movement_service.h"

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

    state = MotionStates::ERROR_RESET;
    last_state = MotionStates::ERROR_RESET;
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
 * ERROR_RESET -> NOT_MOVING -> MOVEMENT_DETECTED -> MOVING -> DECELERATING
 *                    ^                                            |
 *                    |                                            v
 *                    +--------------<-------------------<----- STOPPED
 *
 *
 *
 */
void MovementService::run(void)
{

    /*
     * Perform the following check regardless of whether monitoring mode 
     * is enabled or not
     */

    if (fabs((*vel_ms_ref)) > VEL_MAX_LIMIT) {
        /*
         * Crossed the limit, let move it to reset state
         */

        acceleration_avg_ref->fill(0.0);
        setErrorResetState();
    }

    switch (state) {

    case MotionStates::NOT_MOVING:
        if (last_state != MotionStates::NOT_MOVING) {
            Serial.print("Run-State: NOT_MOVING \r\n");
            last_state = MotionStates::NOT_MOVING;
        }

        if (isStartedMoving() == true) {
            reset_counters();
            vel_threshold = 0;
            state = MotionStates::MOVEMENT_DETECTED;
            MovementService::startMonitoring();
            enable_alarm();
        }
        break;

    case MotionStates::MOVEMENT_DETECTED:
        if (last_state != MotionStates::MOVEMENT_DETECTED) {
            Serial.print("Run-State: MOVEMENT_DETECTED \r\n");
            last_state = MotionStates::MOVEMENT_DETECTED;
        }

        /* 
         * Lets wait for few ms for device to accelerate little bit 
         * and calculate/decide the velocity threshold
         */

        if (vel_threshold == 0 && (millis() - timer_ms) > (VEL_THRESHOLD_SET_TIMEOUT)) {
            vel_threshold = (*vel_ms_ref);

            /*
             * Velocity is too low, must be noise. Should reset to prev state
             */
            if (fabs(vel_threshold) < 0.01) {
                setErrorResetState();
            }

            if (vel_threshold > 0) {
                vel_threshold *= VEL_THRESHOLD_ADJ;
            }

            MovementService::stopMonitoring();
        }

        if (isMovingConfirmed() == true) {
            reset_counters();
            state = MotionStates::MOVING;
            MovementService::startMonitoring();
            enable_alarm();
        }
        break;

    case MotionStates::MOVING:
        if (last_state != MotionStates::MOVING) {
            Serial.print("Run-State: MOVING \r\n");
            last_state = MotionStates::MOVING;
        }

        if (isAtRestOrStable() == true) {
            (*adj_acc_ref) = (*acc_mss_ref) * -1;
        }

        if (isDecelerating() == true) {
            reset_counters();
            (*vel_ms_ref) = 0.0;
            vel_threshold = 0;
            state = MotionStates::DECELERATING;
            enable_alarm();
        }
        break;

    case MotionStates::DECELERATING:
        if (last_state != MotionStates::DECELERATING) {
            Serial.print("Run-State: DECELERATING \r\n");
            last_state = MotionStates::DECELERATING;
        }

        if (isAtRestOrStable() == true) {
            *adj_acc_ref = (*acc_mss_ref) * -1;
            acceleration_avg_ref->fill(0.0);
            reset_counters();
            state = MotionStates::STOPPED;
            disable_alarm();
        }
        break;

    case MotionStates::STOPPED:
        if (last_state != MotionStates::STOPPED) {
            Serial.print("Run-State: STOPPED \r\n");
            last_state = MotionStates::STOPPED;
        }

        stopMonitoring();
        reset_counters();
        state = MotionStates::NOT_MOVING;
        disable_alarm();
        break;

    case MotionStates::ERROR_RESET:
        if (last_state != MotionStates::ERROR_RESET) {
            Serial.print("Run-State: ERROR_RESET \r\n");
            last_state = MotionStates::ERROR_RESET;
        }

        if (isAtRestOrStable() == true) {
            stopMonitoring();
        }

        /* 
         * Check monitoring is done and acceleration below expected
         */
        if (isMonitoring() == false && fabs(acceleration_avg_ref->avg()) < 0.02) {
            state = MotionStates::NOT_MOVING;
            disable_alarm();
        }
        break;

    default:
        break;
    }

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

        if (state >= MotionStates::DECELERATING) {
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
    state = MotionStates::ERROR_RESET;
    (*vel_ms_ref) = 0;
    vel_threshold = 0;
    timer_ms = millis();
    disable_alarm();
    startMonitoring();
}
