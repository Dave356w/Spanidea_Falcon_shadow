#include <Arduino.h>
#include "alarm.h"
#include "RollingAvg.h"

#define TEMP_WAIT 100
#define VEL_MAX_LIMIT 2.0
#define VEL_THRESHOLD_SET_TIMEOUT 400
#define VEL_THRESHOLD_ADJ 2
#define MOVING_ACC_THRESHOLD (0.05)

enum MotionStates
{
    NOT_MOVING = 0,
    MOVEMENT_DETECTED,
    MOVING,
    DECELERATING,
    STOPPED,
    ERROR_RESET,
};

enum MonitorStates {
    MONITORING = 0,
    NOT_MONITORING
};

class MovementService {
  public:
    MotionStates state;
    MotionStates last_state;
    RollingAvg<float> *pressure_avg_ref;
    RollingAvg<float> *acceleration_avg_ref;
    float *vel_ms_ref, *adj_acc_ref, *acc_mss_ref;
    float vel_threshold;
    float variance_acc, variance_pres;
    
    MovementService(RollingAvg<float> *acc_avg, float *acc_mss, float *adj_acc, float *vel_ms, RollingAvg<float> *pres_avg);

    void run(void);
  private:
    uint8_t acc_varience_counter, pressure_varience_counter;
    uint32_t timer_ms;
    MonitorStates monitor_state;

    void reset_counters();
    void setErrorResetState();
    bool isAtRestOrStable();

    inline bool isStartedMoving();
    inline bool isMovingConfirmed();
    inline bool isDecelerating();
    inline void startMonitoring();
    inline void stopMonitoring();
    inline bool isMonitoring();
};