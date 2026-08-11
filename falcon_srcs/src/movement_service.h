#include <Arduino.h>
#include "alarm.h"
#include "RollingAvg.h"
#include "velocity.h"
#include "lateral.h"

#define TEMP_WAIT                      100
#define VEL_MAX_LIMIT                  (2.0)
#define VEL_THRESHOLD_SET_TIMEOUT      400
#define VEL_THRESHOLD_ADJ              2
#define MOVING_ACC_THRESHOLD           (0.05)
#define CALIB_TIMEOUT_MS               10000

/*
 * How many times a rejected calibration window is repeated before the device
 * arms on XY_STILL_FALLBACK anyway.
 *
 * ⚠️ DELIBERATE DEVIATION from the spec, which says "refuse to arm if the
 * calibration window shows movement". Refusing outright leaves a device on a
 * counterweight that never beacons at all, and a silent device is the one
 * failure the spec calls unrecoverable -- the mechanic is ranging the
 * counterweight by ear. So what is refused here is the LEARNED THRESHOLD, not
 * arming: after the retries the unit arms on the low compile-time floor,
 * which makes it over-eager rather than deaf, and says so out loud at the one
 * moment someone is holding it.
 *
 * Two retries puts the worst case at 30 s of calibration, which is inside the
 * 20-30 s the spec already asks about as a UX question.
 */
#define CALIB_RETRIES                  2

/*
 * How long STATE_MOVEMENT_DETECTED dwells before the beacon sounds.
 *
 * Lowered 1000 -> 200 on 2026-08-10. The bench measured set latency at 1115
 * and 1065 ms across two runs, essentially all of it this dwell. That is
 * inside the 1-3 s budget, so this is not a requirement fix -- it is the
 * SECOND working-range axis, minimum burst DURATION.
 *
 * A sub-second jog still alarms: the latch fires on the any-motion edge and
 * nothing downstream can cancel it. What the dwell did was start the beacon
 * AFTER the jog ended -- a 0.25 s inspection jog sounded from 1.1 s to 2.6 s
 * over an already-stationary counterweight. For a beacon the mechanic ranges
 * by ear that asserts "moving now" about a thing that has stopped, which is
 * the same family of position lie the whole design exists to avoid. And
 * inspection operation is continuous-pressure, so bursts are the normal case.
 *
 * The dwell bought nothing on the path that matters. It is a debounce
 * inherited from the polled threshold test, where a single noisy 4-sample
 * average could latch. Any-motion is already debounced in hardware --
 * ANYMOTION_THRESHOLD sustained over ANYMOTION_DURATION at 100 Hz -- so for
 * the detector that actually catches departures it was redundant.
 *
 * NOT ZERO, deliberately. STATE_MOVEMENT_DETECTED is where vel_departure
 * accumulates its peak across the departure ramp; at zero dwell that collapses
 * to a single sample. That measurement is instrumentation today (VEL_ARMED 0)
 * but it is what would arm the velocity path later, and degrading it silently
 * would be paid for much later. 200 ms still spans a sample at 3.13 Hz.
 *
 * COUPLING, since it is not obvious: this moves movement_start_timer 800 ms
 * earlier, and that is the reference for both MIN_TRAVEL_MS (3000) and
 * XY_MIN_BEACON_MS (1500). Both still clear the departure transient with
 * room, but anything that shortens either of those must be checked against
 * this number rather than against the old 1 s.
 *
 * ⬜ THE DURATION AXIS IS STILL UNMEASURED. No deliberate short jog has ever
 * been recorded on this device. The test is cheap -- taps at roughly 0.25,
 * 0.5, 1 and 2 s -- and until it is run, "the beacon starts during the
 * movement" is reasoning, not a measurement.
 */
#define MOVEMENT_DETECTION_TIMEOUT_MS  200
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
 * A cluster alone is not enough -- the polled average must corroborate it.
 *
 * The two detectors fail in OPPOSITE places, which is what makes this work:
 *
 *   speed     cruise asserts?          polled arrival margin
 *   18 fpm    no, 0.4 edges/min        0.312 vs cruise 0.271  = 1.15x
 *   58 fpm    YES, 999 ms pair         brake set 1.275        = comfortable
 *
 * So clustering is safe at low speed but does not fire there (an 18 fpm
 * arrival produced a single edge), while at 58 fpm it fires when it should
 * not -- on 2026-08-07 it released 12 s into a 54 s ride. Polled is the
 * mirror image: marginal slow, strong fast.
 *
 * Requiring both weak signals to agree covers the range without weakening
 * either. Measured against every case that day:
 *
 *   case                     cluster   delta    result
 *   real arrival, descent      yes     0.278    fires (polled alone missed it)
 *   FALSE release, 58 fpm      yes     0.175    REJECTED
 *   18 fpm arrival             no      0.312    fires on polled alone
 *   brake set                  no      1.275    fires on polled alone
 *
 * 0.20 sits above the 0.175 false case and below the 0.278 real one. That is
 * a 1.14x / 1.39x margin on four samples -- thin, and the first thing to
 * revisit as more speeds are measured.
 */
#define ARRIVAL_CLUSTER_DELTA          (0.20)

/*
 * ⬆️ SUPERSEDED at 25 Hz by ARRIVAL_PEAK_VALUE below. Kept for the record and
 * for anyone reverting the sample rate.
 *
 * ARRIVAL_CLUSTER_DELTA and ARRIVAL_THRESHOLD_VALUE both read the rolling
 * average, and on 2026-08-10 the first 25 Hz run showed why that no longer
 * works: a stop whose RAW excursion was 0.938 m/s^2 registered 0.0072 on the
 * 1.28 s average, because one large sample inside a 32-sample window is
 * divided by 32. The signal did not get smaller -- the averaging got longer.
 */

/*
 * Arrival corroboration, on the RAW sample. See main.cpp's ARRIVAL_QUIET_MSS
 * block for how the peak is collected and why it is armed on quiet rather
 * than on a timer.
 *
 * Measured on the first 25 Hz run, cartop, 5-floor slow descent:
 *
 *   cruise raw peak      0.0875   (95 logged samples; the log is decimated
 *                                  8:1 so the true population max is higher)
 *   brake-set bounce     0.938    = 10.7x
 *
 * 0.40 -> 0.70 -> 0.45. The middle value was set on three runs and was wrong;
 * two more runs the same afternoon corrected BOTH ends of its margin.
 *
 *   run                      cruise ceiling   arrival
 *   18 fpm down                   0.09         2.03
 *   high speed up                 0.23         3.42
 *   120 fpm down, 4 floors        0.23         4.58
 *   22 s run                      0.18       * 0.713
 *   27 s run                    * 0.28         1.413
 *
 * ⚠️ THE 0.713 ARRIVAL CLEARED A 0.70 THRESHOLD BY 1.9%. It fired essentially
 * by luck; a marginally softer stop would have run to the failsafe. And the
 * worst cruise rose to 0.28. So the honest separation between the two
 * populations is 0.28 -> 0.713, about **2.55x**, not the 20x the first run
 * suggested -- that run was an unusually clean case and the headline drawn
 * from it did not survive contact with two more.
 *
 * 0.45 sits near the geometric middle: 1.6x above the worst cruise, 1.6x below
 * the weakest arrival. Balanced rather than comfortable.
 *
 * This is still far and away the best arrival discriminator measured on this
 * device -- every previous one managed between 1.07x and 1.4x, and several
 * were on the wrong side. But 2.55x is a working margin, not a solved problem.
 *
 * WHAT WOULD SETTLE IT: more arrivals, and specifically the SOFTEST stops the
 * machine can produce. The weakest arrival is the number this whole approach
 * rests on and five runs have already moved it by a factor of three. The
 * cruise ceiling is the half that moves with equipment; measure it first on
 * any new machine.
 */
#define ARRIVAL_PEAK_VALUE             (0.45)

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
 * Raised again to 300 s on measurement, not reasoning. A full 18 fpm descent
 * on 2026-08-07 held the alarm for 171.6 s -- 95% of the 180 s budget. That is
 * an ordinary slow run, not a fault, and one more floor would have tripped the
 * failsafe mid-travel and split the trip into two alarms.
 *
 *   18 fpm: 300 s covers ~27 m, roughly 8 floors
 *
 * The symptom that it is still too short is a FAILSAFE line followed shortly
 * by a second departure latch.
 *
 * ── RAISED 300 s -> 600 s, 2026-08-11, on measurement ────────────────────────
 *
 * Dave, testing 8/10: "An eight floor run at 18FPM expired on failsafe
 * timeout." That is the symptom described above, observed in the field. At
 * 18 fpm (0.0914 m/s) 300 s covers 27.4 m, so eight floors of ~3.4 m sits
 * exactly on the boundary -- the estimate in the table above assumed 3.3 m
 * floors and had no margin at all.
 *
 * 600 s covers 55 m, roughly 16 floors at this building's pitch.
 *
 * ⚠️ THIS TIMEOUT IS DOING TWO JOBS THAT NOW CONFLICT, and the conflict is not
 * resolvable until the jog defect is fixed:
 *
 *   1. bounding a FAULT -- a missed arrival must not alarm until the battery
 *      is flat. Wants to be generous, because a legitimate slow run genuinely
 *      does take many minutes and should alarm for all of it.
 *   2. bounding the JOG DEFECT -- a movement finishing inside MIN_TRAVEL_MS
 *      latches and cannot release, so it sounds for exactly this long on a
 *      stationary counterweight. Wants to be short.
 *
 * Raising it therefore makes the jog case worse: a stuck alarm now sounds for
 * ten minutes instead of five. That is accepted deliberately. Splitting a
 * legitimate run in two is a failure of the core function -- the beacon stops
 * while the counterweight is still travelling, then re-alarms, and the
 * mechanic sees a gap that means nothing. The jog case is already unusable at
 * five minutes; it is not made meaningfully worse at ten, and it needs fixing
 * on its own terms rather than being papered over by a short timeout.
 *
 * A signal-based split was considered and rejected: the only candidate for
 * telling "stuck at rest" from "still travelling" is the windowed raw peak,
 * and the 2026-08-11 125 fpm run measured cruise at 0.05-0.10 against rest at
 * 0.02-0.05. They overlap, so a quiet-for-N-seconds rule would fire during a
 * genuine slow run and drop the beacon mid-travel.
 *
 * ⬜ UNVERIFIED: battery endurance for a ten-minute continuous alarm. The duty
 * cycle is 2/5 on the piezo, but no measurement of alarm current against pack
 * capacity exists. If a full-length failsafe alarm is found to flatten the
 * pack, that is an argument for fixing the jog defect rather than for
 * shortening this again.
 *
 * REVISIT once jogs release correctly -- at that point job 2 disappears and
 * this can be sized purely for the tallest realistic inspection run.
 *
 * ── 🔴 REVERTED 600 s -> 240 s, 2026-08-11, hours later ─────────────────────
 *
 * The 600 s above was set on reasoning: that a legitimate slow run deserves to
 * alarm for its whole duration, and that the device could afford to sound for
 * ten minutes. The second half was an assumption and it is false.
 *
 * MEASURED THE SAME AFTERNOON, on FRESH cells: the board died 242.7 s into a
 * continuous alarm, part-way through an 8-floor 20 fpm descent.
 *
 *     t=203554   2442     before the run
 *     t=362315   2357      75 s into the alarm
 *     t=479281   2324     192 s into the alarm
 *     t=530006   dead     243 s into the alarm
 *
 * It stopped at 2324, more than 300 counts ABOVE ADC_VOLTAGE_THRESHOLD (2000),
 * so the battery alarm never fired -- by its own measure the pack was healthy.
 * This is not depletion. It is the rail sagging under sustained piezo load,
 * and the averaged ADC reading never sees the instantaneous dip inside a beep.
 * An earlier failure at 2124 the same morning is the same mechanism; fresh
 * cells bought runway, not immunity.
 *
 * SO A FAILSAFE LONGER THAN ~240 s CAN NEVER FIRE -- the hardware quits first,
 * and the only failsafe worth having is one the device outlives. 240 s is set
 * just under the single measured endurance figure.
 *
 * ⚠️ THIS IS A PLACEHOLDER AROUND A HARDWARE DEFECT, not a tuning value, and
 * it costs exactly what the original 300 s comment warned about: an 8-floor
 * 18 fpm run needs ~300 s and will now be cut short. That regression is
 * accepted only because the alternative is a timeout that never runs.
 *
 * WHAT UNBLOCKS IT: a bench measurement of alarm current against pack voltage
 * under load, with a meter rather than the on-board ADC. That distinguishes
 * cells from regulator from decoupling near the piezo. Until the device can
 * sustain a long alarm, no failsafe value is defensible and the whole
 * long-slow-run case is untestable -- both attempts at it died mid-run for
 * exactly this reason.
 */
#define LATCH_FAILSAFE_MS              240000

/*
 * How long the average must stay inside STOP_BAND_VALUE of the zero
 * calibration before the alarm is silenced. The window restarts on any
 * excursion, so this is continuous quiet, not elapsed time.
 *
 * Raised 2000 -> 5000 on 2026-08-07. Two seconds was short enough that
 * LEVELLING qualified as "stopped": on a terminal-floor approach the arrival
 * cluster fired during levelling, the 2 s confirm elapsed while the car was
 * still settling in, the alarm silenced, and the actual brake set 3.5 s later
 * was read as a fresh departure. That latched an alarm which then ran for
 * 73 seconds with the car provably stationary (a flat at 9.69-9.77).
 *
 * Five seconds holds the alarm through levelling. The cost is that the alarm
 * keeps sounding ~3 s longer after a genuine stop.
 */
#define STOP_CONFIRM_MS                5000

/*
 * The band STATE_DECELERATING calls "stopped", m/s^2 from zero calibration.
 *
 * Previously this reused ARRIVAL_THRESHOLD_VALUE (0.30), which is far too
 * loose for the job: it is sized to detect an arrival TRANSIENT, whereas this
 * has to decide the car is no longer moving at all. Levelling motion sits well
 * under 0.30, so it never reset the confirm timer.
 *
 * 0.10 is above the +/-0.04 seen on a genuinely parked car and below the
 * excursions levelling produces.
 */
#define STOP_BAND_VALUE                (0.10)

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
 * arrival is missed. Door dwell is normally several seconds, so this stays
 * inside the gap between a stop and the next start.
 *
 * Raised 2000 -> 6000 on 2026-08-07. A terminal-floor brake set re-latched
 * 3.5 s after the FSM returned to MONITORING -- already outside the 2 s
 * window -- and produced a 73-second alarm on a stationary car. This is the
 * backstop rather than the fix; STOP_CONFIRM_MS above is what stops the
 * premature silence that created the opportunity.
 */
#define MONITOR_REARM_MS               6000


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

    /*
     * Signed velocity integral measured across the departure, m/s. The arrival
     * conservation test is scaled from it -- see velocity.h. Zero means no
     * usable departure measurement was taken, which disables the velocity
     * arrival path for that run rather than falling back to a fitted constant.
     */
    float    vel_departure;

    /* Edge-detect on the departure gate so the log gets one line, not many. */
    bool     vel_reported;

    /*
     * Re-arm blanking to apply on the NEXT entry to STATE_MONITORING.
     *
     * Set per release path rather than fixed, because the two paths have
     * opposite requirements. A z arrival is followed by ringing that would
     * re-latch, so it needs MONITOR_REARM_MS. An x/y release has already
     * observed XY_RELEASE_POLLS quiet metrics, so the ringing is over and
     * 6 s of deafness would only lose the next inspection jog. See
     * XY_REARM_MS.
     */
    uint32_t rearm_ms;

    /*
     * Calibration bookkeeping for the learned XY_STILL. calib_moved is set by
     * an any-motion edge during the window; calib_attempts bounds how many
     * times a rejected window is retried before the device arms on the
     * conservative fallback instead.
     */
    bool     calib_moved;
    uint8_t  calib_attempts;
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
