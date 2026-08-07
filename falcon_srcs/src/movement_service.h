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
 * ARRIVAL DETECTION -- two independent paths, plus a stillness backstop.
 *
 * Measured on 2026-08-07, three arrivals:
 *
 *                       polled averaged delta      clustered any-motion
 *   run 1, harsh stop        0.441   detected       3 edges in 1.0 s  detected
 *   run 2, gentle stop       0.136   MISSED         3 edges in 1.8 s  detected
 *   earlier 18 fpm run       detected               3 edges           detected
 *
 * Cruise, for comparison: the averaged signal reaches 0.18, while any-motion
 * gave 1 isolated edge in 97 s and 0 in 86 s.
 *
 * The interrupt wins on both counts. The sensor applies threshold-and-duration
 * to its own 100 Hz waveform; the polled path sees a 4-sample average at
 * 3.13 Hz that flattens the transient. On run 2 the arrival was SMALLER after
 * averaging (0.136) than cruise noise (0.18) -- no threshold can separate those,
 * which is why that run alarmed at rest until the failsafe.
 *
 * So any-motion is primary. Polling is kept as a genuinely independent second
 * opinion: if the interrupt path fails -- sensor fault, config that did not
 * take, or the buzzer coupling described in Eng_Notes §11 turning out to be
 * mounting-dependent -- the alarm still releases.
 */

/*
 * Polled arrival threshold, m/s^2 on the rolling average.
 *
 * Raised from 0.20 to 0.30. Demoted to a backup for violent stops, it no longer
 * needs to reach for gentle arrivals, so it can have real margin over the 0.18
 * cruise noise instead of the 1.1x it had at 0.20. Run 1's 0.441 still clears it.
 */
#define ARRIVAL_THRESHOLD_VALUE        (0.30)

/*
 * Clustered any-motion: edges required, and the window they must fall inside.
 *
 * ACC-STAT polls and clears the sensor status every ACC_INT_POLL_MS (1 s), so
 * two edges inside this window means the any-motion condition survived more
 * than one poll -- sustained motion, not an isolated spike. Both observed
 * arrivals produced 3 edges inside 1.8 s; cruise produced isolated singles.
 *
 * ONLY EDGES RAISED WHILE THE BUZZER IS OFF COUNT. See notify_any_motion().
 * On 2026-08-07 a cluster made of one buzzer-raised edge plus one quiet edge
 * released an alarm mid-ride. Eng_Notes §11 recorded the buzzer triggering
 * any-motion continuously at threshold 96 with all three axes; at Z-only/32 it
 * is sporadic instead -- roughly one edge per 6-10 s -- which is rare enough to
 * look harmless and frequent enough to break a 2-edge rule.
 */
#define ARRIVAL_EDGE_COUNT             2
#define ARRIVAL_EDGE_WINDOW_MS         2500

/*
 * ⛔ NO STILLNESS BACKSTOP -- do not reintroduce one.
 *
 * A "release the latch if the signal has been quiet for N seconds" path was
 * added on 2026-08-07 and removed the same day. It fired mid-ride on both test
 * runs, before either arrival, and released the alarm while the car was moving.
 *
 * The reasoning behind it was wrong in a way §3 already covers. It assumed
 * cruise is measurably noisier than rest, from one ride where cruise excursions
 * reached 0.18 while rest stayed inside 0.082. Two later rides in the same shaft
 * cruised inside 0.081 and 0.04 respectively -- quieter at cruise than the first
 * ride was at rest.
 *
 * That is not a tuning error. At constant velocity the accelerometer reads 1 g,
 * exactly as it does parked; there is no signal to threshold. No band and no
 * duration separates them, and smoother or slower cars make it worse. The same
 * argument rules out the sensor's no-motion feature.
 *
 * The problem a backstop was meant to solve -- an alarm that never clears
 * because the latch was set with no motion coming to end it -- has to be solved
 * by making ARRIVAL detection reliable, not by inventing a rest detector that
 * cannot exist. LATCH_FAILSAFE_MS remains the only time-based escape.
 */

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
    uint8_t  arrival_edge_count;   /* any-motion edges inside the window  */
    uint32_t arrival_edge_first;   /* timestamp of the first of them      */
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
