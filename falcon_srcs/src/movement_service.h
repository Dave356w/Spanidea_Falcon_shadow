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
 * Failsafe. §3 argues for no timeout, and a genuine run should alarm for its
 * whole duration -- but with no upper bound at all, a missed arrival means the
 * unit alarms until the battery is flat. Reaching this is a fault, not a normal
 * release, and it says so in the log.
 *
 * This bounds how long a legitimate run can be, and at slow speeds the bound
 * bites hardest -- which is exactly where this product is weakest:
 *
 *              distance in 60 s / 180 s      floors at ~3.3 m
 *     18 fpm      5.5 m  /  16.5 m             1.6  /   5
 *     38 fpm     11.6 m  /  34.7 m             3.5  /  10
 *    350 fpm      107 m  /   320 m              30  /  97
 *
 * Set to 60 s initially. That was too short: on the 2026-08-07 18 fpm run it
 * fired at 60,064 ms while the car was still moving, dropped the alarm, and
 * returned the FSM to MONITORING 16 s before the real arrival -- so the arrival
 * transient looked like a fresh departure and the unit alarmed twice for one
 * trip. The run was about two floors.
 *
 * 180 s covers roughly five floors at 18 fpm while still bounding a genuinely
 * stuck alarm to three minutes. Raise it further if slower or longer runs turn
 * up in service; the symptom to watch for is a FAILSAFE line followed shortly
 * by a second departure latch.
 */
#define LATCH_FAILSAFE_MS              180000

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

/*
 * How long after entering STATE_MONITORING an any-motion edge is ignored.
 *
 * A harsh stop keeps generating any-motion for a while after the FSM has
 * decided the car arrived: the car rings in the rails, doors operate,
 * passengers step out. Without this, the tail of an arrival re-latches as a
 * fresh departure and the unit alarms twice for one trip.
 *
 * STATE_DECELERATING already requires STOP_CONFIRM_MS of continuous quiet
 * before the alarm is silenced, so this is a second line of defence covering
 * the moment right after that.
 *
 * The cost is that a genuine departure within this window of the previous
 * arrival is missed. Door dwell is normally several seconds, so 2 s is
 * comfortably inside the gap between a stop and the next start.
 */
#define MONITOR_REARM_MS               2000


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
     * edge_ms is when the ISR saw it, NOT when loop() got round to it -- see
     * the ordering race described in the implementation.
     *
     * Latches a departure if the FSM is monitoring; ignored in every other
     * state, so an interrupt raised by the buzzer while already alarming
     * (Eng_Notes §11) cannot do anything.
     */
    void notify_any_motion(uint32_t edge_ms);
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
    uint32_t monitor_entered_ms;   /* when STATE_MONITORING was entered   */
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
