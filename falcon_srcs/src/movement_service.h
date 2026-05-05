#include <Arduino.h>
#include "alarm.h"
#include "RollingAvg.h"

#define TEMP_WAIT                      100
#define VEL_MAX_LIMIT                  (2.0)
#define VEL_THRESHOLD_SET_TIMEOUT      400
#define VEL_THRESHOLD_ADJ              2
#define MOVING_ACC_THRESHOLD           (0.05)
#define CALIB_TIMEOUT_MS               10000
#define MOVEMENT_DETECTION_TIMEOUT_MS  1000
#define STOP_TIMEOUT_MS                10000


enum MotionStates
{
    STATE_NOT_MOVING = 0,
    STATE_CALIBERATION,
    STATE_MONITORING,
    STATE_MOVEMENT_DETECTED,
    STATE_MOVING,
    STATE_DECELERATING,
    STATE_STOPPED,
    STATE_ERROR_RESET,
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
    float zero_calib_value;
    float threshold_value;
    
    MovementService(RollingAvg<float> *acc_avg, float *acc_mss, float *adj_acc, float *vel_ms, RollingAvg<float> *pres_avg);

//    void run(void);
    void fsm_run(void);
  private:
    uint8_t acc_varience_counter, pressure_varience_counter;
    uint32_t timer_ms;
    MonitorStates monitor_state;
    uint32_t reset_counter;
    uint32_t start_timer;
    uint32_t movement_start_timer;

    void reset_counters();
    void setErrorResetState();
    void set_state(int);
    bool isAtRestOrStable();

    inline bool isStartedMoving();
    inline bool isMovingConfirmed();
    inline bool isDecelerating();
    inline void startMonitoring();
    inline void stopMonitoring();
    inline bool isMonitoring();
};
