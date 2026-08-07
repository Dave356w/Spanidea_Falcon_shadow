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
#define STOP_TIMEOUT_MS                15000

/*
 * Roadmap item 8 -- latch on departure, release on arrival (Eng_Notes §3, §12).
 *
 * Set to 0 to restore the timeout-based behaviour of 59e945f exactly. Nothing
 * else in the tree changes; every difference is inside #if LATCHED_FSM blocks.
 *
 * The old design alarmed for a fixed 10 s buzzer / 15 s LED window and then
 * asked "is the average still displaced from calibration?". During constant
 * velocity it is not -- that is §3's physical constraint -- so the alarm cleared
 * mid-ride regardless of whether the car had arrived. That is the original
 * complaint this whole investigation started from.
 *
 * The latched design instead holds the alarm from departure until an arrival
 * transient is seen, per the §3 table. It is only possible now because:
 *
 *   - departures are detectable at all (§12, any-motion at 18 fpm), and
 *   - something is listening during the alarm (§5 item 7, ~45% coverage)
 */
#define LATCHED_FSM                    1

/*
 * Arrival detection threshold, m/s^2, on the rolling average.
 *
 * NOT the same as DEFAULT_THRESHOLD_VALUE (0.400), which is applied while
 * stationary. This one runs while the car is moving and the buzzer is sounding,
 * so it sees a noisier signal built from ~45% sample coverage.
 *
 * Measured arrival deltas ON THE AVERAGE: 0.755, 0.334, 0.32. Note the last two
 * are below 0.400 -- which is exactly why the old code sometimes failed to
 * notice an arrival at all. Cruise noise on the average is roughly +/-0.10
 * (raw +/-0.19 through a 4-sample window).
 *
 * 0.20 sits above cruise and below the weakest arrival observed, but the margin
 * either side is under 2x. This is the least well-evidenced number in the
 * change and the first thing to revisit with more hoistway data.
 */
#define ARRIVAL_THRESHOLD_VALUE        (0.20)

/*
 * Minimum time in STATE_MOVING before arrival detection is armed.
 *
 * The departure transient itself would otherwise satisfy the arrival test
 * immediately and release the latch a fraction of a second after setting it.
 */
#define MIN_TRAVEL_MS                  3000

/*
 * Failsafe. §3 argues for no timeout, and a genuine express run should alarm
 * for its whole duration -- but with no upper bound at all, a missed arrival
 * means the unit alarms until the battery is flat. Five minutes is far longer
 * than any plausible single run and still bounds the worst case.
 *
 * Reaching this is a fault, not normal operation, and it says so in the log.
 */
#define LATCH_FAILSAFE_MS              300000

/*
 * How long the average must stay inside ARRIVAL_THRESHOLD_VALUE of the zero
 * calibration before the alarm is silenced. The window restarts on any
 * excursion, so this is continuous quiet, not elapsed time.
 *
 * 2 s covers the ringing and door operation that follow an arrival. Too short
 * and the alarm clears while the car is still rocking; too long and it keeps
 * sounding after the passenger has left.
 */
#define STOP_CONFIRM_MS                2000


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
    bool log_printed;
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
    int  get_state();

#if LATCHED_FSM
    /*
     * Called from loop() when the BMA456 any-motion interrupt has fired.
     * Latches a departure if the FSM is monitoring; ignored in every other
     * state, so an interrupt raised by the buzzer while already alarming
     * (Eng_Notes §11) cannot do anything.
     */
    void notify_any_motion(void);
#endif

  private:
    uint8_t acc_varience_counter, pressure_varience_counter;
    uint32_t timer_ms;
    MonitorStates monitor_state;
    uint32_t reset_counter;
    uint32_t start_timer;
    uint32_t movement_start_timer;
    uint32_t current_time;
#if LATCHED_FSM
    bool     any_motion_pending;   /* departure seen, not yet acted on   */
    bool     arrival_seen;         /* arrival transient seen while MOVING */
    uint32_t stop_confirm_timer;   /* start of the settled-at-1g window   */
#endif

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
