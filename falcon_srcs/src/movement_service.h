#include <Arduino.h>
#include "alarm.h"
#include "RollingAvg.h"
#include "velocity.h"
#include "lateral.h"

/*
 * Calibration window. 10000 -> 6000 on 2026-08-14, on Dave's instruction, after
 * the customer asked for 5 s. 6 and not 5 for two reasons, both measured -- see
 * lateral.h and Eng_Notes/falcon_ui_battery_2026-08-14.md §6.3:
 *
 *   PARITY. calib_finish() indexes the sorted buckets at (n-1)/2, the LOWER of
 *   the two middles on an EVEN count, chosen deliberately as the safer
 *   threshold. Across six replayed calibrations, 4 s and 6 s never exceeded the
 *   10 s answer; the ODD windows 3 s and 5 s did, by up to +23.6% and +5.5%.
 *   Above the 10 s answer is the direction that makes travel read as still.
 *
 *   QUORUM. XY_CALIB_MIN_BUCKETS was 6, so a 5 s window would have been
 *   REJECTED outright on every install and the device would have armed on
 *   XY_STILL_FALLBACK every time -- shortening this alone would have disabled
 *   calibration, not shortened it. The quorum moved to 4 with this change.
 *
 * ⚠️ THIS MUST STAY CONSISTENT WITH lateral.h. The static_assert below is the
 * guard; the two constants live in different headers and nothing else ties
 * them together.
 *
 * ⬜ Still bench-only evidence: six calibrations, one mounting, unit at rest on
 * a desk. `XY: bmax` now prints on every attempt, so the next hoistway session
 * collects site data for free -- re-run graph/calib_replay.py against it.
 */
#define CALIB_TIMEOUT_MS               6000

static_assert(CALIB_TIMEOUT_MS / XY_CALIB_BUCKET_MS == XY_CALIB_BUCKETS,
              "CALIB_TIMEOUT_MS and XY_CALIB_BUCKETS have drifted apart: the "
              "window no longer holds exactly XY_CALIB_BUCKETS buckets");
static_assert(XY_CALIB_MIN_BUCKETS <= XY_CALIB_BUCKETS,
              "XY_CALIB_MIN_BUCKETS exceeds the number of buckets the window "
              "can produce -- every calibration would be rejected");

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
 * earlier, and that is the reference for MIN_TRAVEL_MS (3000). It still
 * clears the departure transient with room, but anything that shortens it
 * must be checked against this number rather than against the old 1 s.
 *
 * ⬜ THE DURATION AXIS IS STILL UNMEASURED. No deliberate short jog has ever
 * been recorded on this device. The test is cheap -- taps at roughly 0.25,
 * 0.5, 1 and 2 s -- and until it is run, "the beacon starts during the
 * movement" is reasoning, not a measurement.
 */
#define MOVEMENT_DETECTION_TIMEOUT_MS  200

/*
 * LATCHED FSM -- latch on departure, release on arrival (Eng_Notes §3, §12).
 *
 * The alarm holds from departure until an arrival transient is seen, per the
 * §3 table. The original timeout-based design (fixed 10 s buzzer / 15 s LED,
 * then "is the average still displaced?") cleared mid-ride during constant
 * velocity -- §3's physical constraint -- and was the original complaint this
 * whole investigation started from.
 *
 * The LATCHED_FSM compile switch and the old timeout FSM were removed in the
 * 2026-08-12 cleanup; the last commit carrying them is in history at 8308cdd,
 * and the pre-latch behaviour itself at 59e945f.
 */

/*
 * BURST RECORDER SPLITS -- samples kept AFTER each trigger; the remaining
 * BURST_N - post are pre-trigger history. See burst_trigger() in main.cpp
 * for why the two triggers want different splits.
 *
 * ⚠️ SINGLE SOURCE, deliberately in this header. Until 2026-08-12 main.cpp
 * and movement_service.cpp each carried private copies, and the 2026-08-11
 * "20 -> 60" change (329ce6d) edited only main.cpp's -- which is used for
 * nothing but the pre= label on the dump. The FSM kept triggering at 20 post
 * while every dump CLAIMED 60, so the 350 fpm arrival bursts of runs 4-5
 * were analysed against a trigger index 40 samples off. The intended 60-post
 * arrival split takes real effect from this change onward.
 */
#define BURST_POST_DEP  60  /* departure: 20 pre / 60 post */
#define BURST_POST_ARR  60  /* arrival:   20 pre / 60 post */

/*
 * ✅ RAMP DETECTOR ARMED 2026-08-12 -- the third arrival path now releases.
 *
 * The detector itself (design, thresholds, measured basis, and the rollback)
 * lives in main.cpp above the RAMP_* constants; this flag is here because the
 * FSM is what acts on the verdict. Armed, a ramp hit sets arrival_seen and
 * transitions to STATE_DECELERATING, so the 350 fpm release no longer depends
 * on a 1.009x peak margin. It still fires the arrival burst, armed or not.
 *
 * The unarmed exposure §3.1 requires is done: 17/17 automatic drive stops,
 * zero hits across a full inspection session and 51 replayed departure bursts.
 * ROLLBACK if a RAMP release ever silences a moving car: set 0, diagnose after.
 */
#define RAMP_ARMED      1

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
 * (The rolling-average corroboration constants ARRIVAL_THRESHOLD_VALUE and
 * ARRIVAL_CLUSTER_DELTA were removed in the 2026-08-12 cleanup: both read the
 * 1.28 s average, which the 25 Hz change made blind to arrival transients --
 * a stop whose RAW excursion was 0.938 m/s^2 registered 0.0072 on the
 * average. ARRIVAL_PEAK_VALUE on the raw sample replaced them at 5bd499e;
 * their tuning history is in this header at 8308cdd.)
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
 *
 * ─── ⛔ EVERYTHING ABOVE DESCRIBES A FAULT THAT WAS NOT IN THE DEVICE ────────
 *
 * 🟢 2026-08-14, established at the bench by Dave: THE ~243 s DEATHS WERE A
 * COM-LINK LOCKUP. Not rail sag, not depletion, not the TWI wedge, not an
 * intermittent contact under vibration. The serial link was the failure.
 *
 * Note what the block above actually rests on: "dead" was inferred ENTIRELY
 * from the log stopping. No reset flag, no LED state, no measured rail voltage
 * -- and the bench procedure already recorded that the USB-serial cable
 * back-powers this board through the RX ESD clamp. The one instrument being
 * used to detect the failure was itself powering the thing it was watching.
 * The rail-sag mechanism above was a confident explanation of an artifact, and
 * it is retained only so the reasoning is not rebuilt from scratch.
 *
 * RAISED 240000 -> 600000 on Dave's instruction, 2026-08-14. The clamp existed
 * solely to stay under an endurance limit that does not exist, so the constraint
 * that set it is gone.
 *
 * WHAT 600 s BUYS: the long-slow-run case. An 8-floor 18 fpm descent needs
 * ~300 s and was being cut short at 240 s -- the failsafe was silencing the
 * beacon over a MOVING counterweight, which is the catastrophic direction and
 * far worse than anything a longer timeout does.
 *
 * ⚠️ WHAT IT COSTS, and it is a real cost: this is the backstop that ends a
 * beacon nothing else released, so every stuck-beacon failure now sounds for up
 * to 10 minutes instead of 4. The §5.2 single-floor blind spot is reproducible
 * on demand and produced an 85 s position lie; with this value the same defect
 * could run to 600 s. That is accepted deliberately -- a position lie destroys
 * trust in the instrument, while silence over a moving counterweight is the
 * failure the whole product exists to prevent. The asymmetry decides it.
 *
 * ⬜ THIS IS NOT A LICENCE TO STOP FIXING §5.2. A 10-minute false beacon is a
 * worse symptom than a 4-minute one; the arming-gate redesign (test plan D1)
 * is what actually closes it.
 */
#define LATCH_FAILSAFE_MS              600000UL

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

/*
 * ─── REST-GATED EARLY CLEAR OF THE RE-ARM BLANK (2026-08-18) ────────────────
 *
 * THE DEFECT THIS ADDRESSES. The fixed 6 s blank above is a 6-SECOND WINDOW IN
 * WHICH A REAL DEPARTURE IS SILENTLY DISCARDED, and because cruise generates
 * essentially NO any-motion edges, the departure edge is the ONLY edge until
 * the brake. Discard it and the entire run is undetected: beacon silent over a
 * moving counterweight, then a latch at the stop and an alarm on a car that has
 * already arrived.
 *
 * Measured on the cartop 2026-08-18 (inverted mount, slow up runs from the
 * lower landing): 6 of 12 runs missed, and 4 of those 6 carry explicit
 * "any-motion ignored, re-arm blanking" lines for the departure edge. 32 such
 * discards across the session. See falcon_jog_ringing_2026-08-18.md §0.0c.
 *
 * THE FIX. End the blank as soon as the mechanical disturbance from the
 * previous stop has actually subsided, instead of after a fixed 6 s.
 * lat_monitor.quiet_run() counts consecutive lateral metrics under the
 * per-deployment learned still-threshold: it is driven to 0 by the ringing a
 * stop produces and climbs once that ringing passes. Requiring it to REACH
 * MONITOR_REARM_QUIET once (the flag then latches for that MONITORING episode)
 * is a direct measurement of "the car has settled", which is the condition the
 * 6 s was standing in for.
 *
 * ⚠️ THIS CAN ONLY EVER SHORTEN THE BLANK, NEVER LENGTHEN IT. MONITOR_REARM_MS
 * remains a hard cap, and MONITOR_REARM_MIN_MS is a floor that still covers the
 * two cases the original guard was written for -- loop() ordering (an edge
 * raised in DECELERATING delivered after MONITORING is entered) and the first
 * instants of residual ringing.
 *
 * 🔴 THE RISK, STATED PLAINLY. The 6 s exists because on 2026-08-07 a
 * terminal-floor brake set re-latched 3.5 s after MONITORING was entered and
 * produced a 73-SECOND ALARM ON A STATIONARY CAR. If the lateral goes quiet
 * before such a brake set, this change lets that edge through again. The
 * mitigating fact -- recorded in the comment above -- is that STOP_CONFIRM_MS
 * is what actually closed that hole; MONITOR_REARM_MS is explicitly "the
 * backstop rather than the fix".
 *
 * ⛔ SO THE THING TO WATCH ON THE CAR IS AN ALARM THAT STARTS ON A CAR THAT HAS
 * ALREADY STOPPED.
 *
 * ⚠️ THE REVERT INSTRUCTION THAT USED TO SIT HERE WAS BACKWARDS. It said "revert
 * by setting MONITOR_REARM_QUIET to 0". The clear condition is
 *
 *     elapsed >= MONITOR_REARM_MIN_MS && quiet_run() >= MONITOR_REARM_QUIET
 *
 * so with MONITOR_REARM_QUIET = 0 the second test is ALWAYS TRUE and the blank
 * clears at the MONITOR_REARM_MIN_MS floor every single time -- the SHORTEST
 * blank this code can produce, not the longest. Anyone following it during an
 * incident would have made the symptom worse. Corrected 2026-08-19.
 *
 * TO REVERT to the fixed 6 s behaviour, make the early clear unreachable so
 * MONITOR_REARM_MS governs: set MONITOR_REARM_QUIET to 255. quiet_run() would
 * need 255 consecutive quiet samples, ~10.2 s at 25 Hz, which is longer than
 * the 6 s cap, so the cap binds.
 *
 * ⚠️ AND KNOW WHAT THE REVERT COSTS. Measured 2026-08-19 across 17 stops on one
 * car, the shortest real gap between MONITORING and the next genuine departure
 * was 2801 ms:
 *
 *     blank      real departures it would DISCARD
 *     1500 (now)   0
 *     2500         0
 *     4000         1
 *     6000 (old)   1
 *
 * The 2026-08-07 false re-latch was at 3.5 s. A real departure was seen at
 * 2.8 s. THOSE WINDOWS OVERLAP, so no blank duration separates them -- covering
 * 3.5 s necessarily discards real departures. That is the whole reason this
 * gate is rest-based rather than time-based, and it means MONITOR_REARM_QUIET
 * is the lever to reach for, not MONITOR_REARM_MIN_MS. Raising the quiet
 * requirement costs nothing on an ordinary stop (q reached 76-84 within ~1.5 s
 * on every stop measured) and extends the blank only when the stop is actually
 * ringing, which is the discrimination wanted. Sizing it needs q from a
 * terminal-floor brake set, which is session C and has never been run.
 */
#define MONITOR_REARM_QUIET            8     /* lateral quiet metrics = settled */
/*
 * 1500 -> 2500, 2026-08-19, on a field report of intermittent alarms starting
 * after arrival. THIS IS A HEDGE, NOT A FIX, and it is important that whoever
 * reads this next knows the difference.
 *
 * WHAT IT BUYS. A second more coverage against a re-latch on a car that has
 * already stopped.
 *
 * WHAT IT COSTS. Nothing measured. Across 17 stops on 2026-08-19 the shortest
 * gap between MONITORING and the next GENUINE departure was 2801 ms, so a
 * 2500 ms floor discards none of them. 4000 would have discarded one.
 *
 * WHAT IT DOES NOT DO. The recorded false re-latch (2026-08-07, terminal-floor
 * brake set, 73 s alarm on a stationary car) was at 3.5 s. This floor is below
 * that and does not cover it. It cannot be raised to cover it either: a real
 * departure was observed at 2.8 s and the false re-latch at 3.5 s, so THE TWO
 * POPULATIONS OVERLAP and no floor value separates them. Anything covering
 * 3.5 s discards real departures, and a discarded departure means the beacon
 * stays silent over a moving car -- the worse of the two failures.
 *
 * THE LEVER THAT ACTUALLY DISCRIMINATES is MONITOR_REARM_QUIET, not this floor:
 * it extends the blank only when the stop is genuinely ringing. On every
 * ordinary stop measured, quiet_run() reached 76-84 within ~1.5 s, so raising
 * the quiet requirement is nearly free on a normal arrival. Sizing it needs
 * quiet_run() through a terminal-floor brake set -- session C, never run.
 */
#define MONITOR_REARM_MIN_MS           2500  /* floor; ordering + first ringing */


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

class MovementService {
  public:
    MotionStates state;
    MotionStates last_state;
    RollingAvg<float> *acceleration_avg_ref;
    float zero_calib_value;
    float threshold_value;       /* learned polled departure gate       */
    float z_cal_min;             /* z average min across calibration    */
    float z_cal_max;             /* z average max across calibration    */

    MovementService(RollingAvg<float> *acc_avg);

    void fsm_run(void);
    int  get_state();

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

    /*
     * Called from loop() when the departure-burst jog verdict reads JOG
     * (main.cpp, JOG_VERDICT_ARMED). Requests a release: accepted only if
     * the FSM is still in STATE_MOVING, consumed on the next fsm_run()
     * pass through the same exit the failsafe uses (STATE_DECELERATING),
     * so arming adds a trigger, not a new transition. Ignored in every
     * other state -- if a real arrival beat the verdict to it, there is
     * nothing left to release and no flag is left behind to leak into the
     * next run (belt-and-braces: MOVING entry clears it too).
     */
    void jog_release(void);

  private:
    uint32_t reset_counter;
    uint32_t start_timer;
    uint32_t movement_start_timer;
    uint32_t current_time;
    bool     any_motion_pending;   /* departure seen, not yet acted on   */
    bool     arrival_seen;         /* arrival transient seen while MOVING */
    bool     jog_release_pending = false;  /* jog verdict release request */
    bool     ramp_reported;        /* ramp verdict logged this run        */
    uint32_t stop_confirm_timer;   /* start of the settled-at-1g window   */
    uint32_t monitor_entered_ms;   /* when STATE_MONITORING was entered   */
    bool     rearm_cleared;        /* settled seen -> blank ended early    */
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
     * Calibration bookkeeping for the learned XY_STILL. calib_moved is set by
     * an any-motion edge during the window; calib_attempts bounds how many
     * times a rejected window is retried before the device arms on the
     * conservative fallback instead.
     */
    bool     calib_moved;
    uint8_t  calib_attempts;

    void set_state(int);
};
