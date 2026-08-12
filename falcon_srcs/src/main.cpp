/*
 ************************************************************
 *
 * Copyright Spanidea 2024-25
 ************************************************************
 */

#include "main.h"
#include "arduino_bma456.h"
#include "common.h"
#include "velocity.h"
#include "lateral.h"

uint8_t alarm_status_g = 0;
uint8_t chase_led_status_g = 0;
uint8_t battery_alarm_status_g = 0;
uint32_t temp_timer = 0;
uint32_t init_time_g = 0;
SystemStates state = SystemStates::SYSTEM_STATE_INITIALIZING;
static boolean in_isr = false;
/*
 * 4 -> 32 alongside the 3.13 Hz -> 25 Hz timer change (see enable_timer()).
 *
 * THE WINDOW LENGTH IN SECONDS IS DELIBERATELY UNCHANGED: 4 samples at 3.13 Hz
 * and 32 at 25 Hz both span 1.28 s. Every threshold that reads this average --
 * DEFAULT_THRESHOLD_VALUE, ARRIVAL_THRESHOLD_VALUE, ARRIVAL_CLUSTER_DELTA,
 * STOP_BAND_VALUE -- therefore keeps exactly the meaning it was tuned to, and
 * the sample-rate change does not silently retune the FSM.
 *
 * What DOES change is what the average is made of: 32 real samples instead of
 * 4 aliased ones. Averaging 8x more independent samples over the same interval
 * cuts the noise by about sqrt(8) = 2.8x while leaving a genuine 1 s transient
 * untouched. That ratio is the entire point of the exercise -- §14.7's gentle
 * arrival measured 0.058 against a parked noise floor of 0.103, and a 2.8x
 * quieter floor is what would make it visible.
 */
RollingAvg<float> acceleration_avg_g(32);
RollingAvg<uint16_t> battery_avg(8);
float x = 0, y = 0, z = 0;
float g_value, accel_value;

MCP3208 adc(ADC_VREF, PIN_ADC_CS);
MovementService ms(&acceleration_avg_g);

#define EN_3_AXIS_SENS 1

/*
 * Sample telemetry.
 *
 * Serial.print() must never be called from inside the timer ISR. HardwareSerial
 * drains its TX ring buffer from the UART interrupt, so printing from another
 * ISR (which re-enables interrupts) races the drain and drops characters. The
 * 2026-07-15 EFT captures show 14 corrupted lines across 8 runs, every one of
 * them inside a state-transition burst.
 *
 * The ISR now only publishes a snapshot here; loop() does the printing.
 */
typedef struct {
    uint32_t t_ms;          /* millis() at time of sample                   */
    int16_t  accel;         /* z acceleration, MILLI-m/s^2                  */
    int16_t  avg;           /* rolling average after this sample, milli      */
    /*
     * Lateral axes, m/s^2. Logged but not yet used by the FSM.
     *
     * X and Y carry NO gravity component, so unlike z they have no 1 g
     * pedestal for cruise and rest to hide behind -- what they measure is
     * lateral acceleration, i.e. guide-shoe and rail excitation while
     * travelling and nothing at all while parked. §3's "constant velocity is
     * unobservable" argument is about the gravity axis and does not apply to
     * these; neither do the numbers in movement_service.h's stillness-backstop
     * block, every one of which is z.
     *
     * getAcceleration() has always read all three axes and discarded these
     * two, so logging them costs no extra I2C -- only the serial characters.
     * The open question they answer is the contrast between parked and
     * travelling, which has never been measured on this device.
     *
     * Note aliasing hurts an ENERGY measure far less than it hurts transient
     * detection: sub-sampling scrambles a waveform's shape but preserves its
     * total power, so sample variance of x/y is a usable vibration estimate
     * even at 3.13 Hz.
     */
    int16_t  ax;            /* x acceleration, MILLI-m/s^2                  */
    int16_t  ay;            /* y acceleration, MILLI-m/s^2                  */
    uint16_t read_us;       /* duration of the bma456 I2C read, microseconds */
    uint8_t  fsm_state;     /* MovementService state at time of sample      */
    uint8_t  err_run;       /* consecutive failed sensor reads, 0 = healthy */
} sample_log_t;

/*
 * SAMPLE RING -- replaces the single-slot snapshot, 2026-08-11.
 *
 * The single slot lost ~20% of samples at 25 Hz, and every threshold measured
 * on 2026-08-10/11 therefore rests on 80% of the data. The cause is not the
 * ISR: Serial.print() BLOCKS once the 64-byte TX buffer fills, for roughly
 * 85 ms of a 145-character line, and the ISR keeps publishing into the one
 * slot throughout. At 3.13 Hz a 151 ms line fitted inside a 320 ms period; at
 * 25 Hz it spans two periods and the loser is always the sample.
 *
 * A ring lets loop() drain what accumulated during a blocking print instead of
 * watching it be overwritten. Single producer (the timer ISR) and single
 * consumer (loop), so no locking is needed: head is written only by the ISR,
 * tail only by loop, and uint8_t access is atomic on AVR.
 *
 * WHY THE STRUCT SHRANK. RAM, not taste. Static was already 1374 bytes of 2048
 * with ~176 more on the heap, leaving ~500 for stack; a ring of 8 at the old
 * 24-byte layout would have taken 192 of that. Storing the four floats as
 * scaled 16-bit integers cuts the record 24 -> 16 bytes, so 8 slots cost 128
 * and the single slot it replaces gives 16 back: net +104, measured.
 *
 * That leaves ~390 bytes for stack. Workable, not roomy. The next thing that
 * wants RAM should shrink the ring to 6 or shorten the sample line rather than
 * assume there is headroom.
 *
 * The precision lost is real but confined to the LOG: milli-m/s^2 resolution,
 * so a= prints 3 decimals instead of 4. Nothing that decides anything reads
 * these fields -- the arrival peak, arr_hit and the burst recorder all work
 * from the float in the ISR and never touch the ring.
 */
#define SAMPLE_RING_N 8

static volatile sample_log_t ring[SAMPLE_RING_N];
static volatile uint8_t      ring_head = 0;   /* ISR writes  */
static volatile uint8_t      ring_tail = 0;   /* loop reads  */
static volatile uint16_t     sample_overrun = 0;

/*
 * Free-running count of sensor reads attempted, incremented on every entry to
 * read_acceleration_mss() regardless of what happens afterwards.
 *
 * This exists to diagnose the gap described in Eng_Notes §6a: printed samples
 * arrive in bursts of ~6 followed by a gap of exactly 6 sample periods, while
 * ov= sits at 6 under every condition. Because ov= only counts snapshots the
 * ISR overwrote before loop() printed them, it cannot distinguish "the ISR
 * never ran" from "the ISR ran and the print path dropped it".
 *
 * tk= closes that gap. Compare its growth against the number of lines printed:
 *
 *   tk advances ~12 while 6 lines print  -> ISR is firing; the publish/print
 *                                           path is losing samples, and ov=
 *                                           is failing to count them
 *   tk advances ~6 while 6 lines print   -> the ISR itself is not firing, and
 *                                           the timer stalls for ~2 s at a
 *                                           time. No timing measured from
 *                                           these logs can be trusted
 *
 * Diagnostic only. Remove once §6a is resolved.
 */
static volatile uint16_t     isr_ticks = 0;

/*
 * Sensor read failures. err_run is consecutive (cleared by any good read) and
 * is what a fault decision should key off; err_total is cumulative since boot
 * and is there to catch an intermittent bus that never trips a run threshold.
 */
static volatile uint16_t     sensor_err_total = 0;
static volatile uint8_t      sensor_err_run   = 0;

/*
 * RAW-SAMPLE ARRIVAL PEAK -- roadmap item 5's payoff, 2026-08-10.
 *
 * The brake energising at the end of a run produces an abrupt bounce (Dave,
 * on the cartop: "rollback upon brake energization ... abrupt up and down
 * bounce"). Measured on the first 25 Hz run it reached 0.938 m/s^2 against a
 * cruise raw maximum of 0.0875 -- a 10x separation, where every other arrival
 * signal this project has tried managed 1.07x to 1.4x.
 *
 * IT IS INVISIBLE TO THE ROLLING AVERAGE. One large sample inside a 32-sample
 * window is divided by 32: the same stop reported 0.0072 on avg, against an
 * ARRIVAL_CLUSTER_DELTA of 0.20. The averaging destroys precisely the signal
 * that the sample-rate fix just made visible, which is why arrival now reads
 * the RAW value and the average is left to the departure and stop-band tests
 * it was tuned for.
 *
 * At 3.13 Hz this could not have worked: an ~80 ms bounce falls between
 * samples taken 320 ms apart most of the time, and a detector that sees a
 * quarter of the events it is looking for is not a detector.
 *
 * Tracked as a RUNNING MAX in the ISR rather than sampled from the published
 * snapshot, because ~20% of snapshots are currently lost to overrun and the
 * peak must not be one of them. A max is monotonic, so a lost publish costs
 * nothing.
 *
 * ARMING: the departure produces a bounce of its own, so the peak is not
 * collected until the signal has been quiet for ARRIVAL_ARM_SAMPLES -- the
 * detector waits for the departure transient to end rather than for a fixed
 * time to elapse. That is what makes it a SECOND transient rather than a
 * timer, and it is the property MIN_TRAVEL_MS was approximating badly.
 */
#define ARRIVAL_QUIET_MSS      (0.15f)   /* above cruise raw max 0.0875      */
#define ARRIVAL_ARM_SAMPLES    5         /* 200 ms of quiet at 25 Hz         */

/*
 * ⚠️ THE PEAK MUST BE WINDOWED, NOT CUMULATIVE. Fixed 2026-08-10 after the
 * high-speed run exposed it.
 *
 * The first implementation held a running max for the whole run. A max only
 * ever grows, so cruise vibration ratchets it upward indefinitely: the
 * high-speed run climbed 0.09 -> 0.13 -> 0.17 -> 0.19 -> 0.23 in fifteen
 * seconds of cruise, against a threshold of 0.30. This morning's 25 fpm run
 * lasted 197 seconds. A cumulative max would have crossed any threshold
 * eventually and released the beacon mid-travel -- the catastrophic direction,
 * reached not by a bad threshold but by a detector that cannot forget.
 *
 * An arrival is a BRIEF event, so the honest question is "what was the peak in
 * the last second", not "since the run began". Two alternating buckets, with
 * the reported value being the max of the current and previous one, give a
 * 1-2 s sliding window using two floats and no array.
 */
#define ARRIVAL_PEAK_WINDOW_MS 1000

static volatile float    arr_zero      = 0.0f; /* baseline, published by FSM */
static volatile float    arr_peak_cur  = 0.0f; /* max in the open bucket     */
static volatile float    arr_peak_prev = 0.0f; /* max in the one before it   */
static volatile uint32_t arr_bucket_ms = 0;
static volatile uint8_t  arr_quiet     = 0;
static volatile bool     arr_armed     = false;

/*
 * Sticky record that the windowed peak crossed ARRIVAL_PEAK_VALUE at some
 * point during this run. Added 2026-08-10 after a 1 s jog latched the alarm
 * for a minute with the car provably still.
 *
 * Windowing the peak fixed a detector that could not forget; it created one
 * that forgets too fast. The jog's stop bounce crossed the threshold at
 * t+1.0 s, and MIN_TRAVEL_MS does not open the arrival gate until t+3.2 s --
 * by which point the bounce had rolled out of the 1-2 s window and the FSM
 * read 0.03. The stop was detected and then discarded, unused.
 *
 * A LATCH IS SAFE HERE IN A WAY A RUNNING MAX WAS NOT. A max creeps upward
 * through cruise vibration and crosses any threshold given enough time -- that
 * is what the windowing exists to stop. A latch only ever fires if cruise
 * genuinely exceeds ARRIVAL_PEAK_VALUE, and measured cruise tops out at 0.23
 * against 0.70. So the window keeps bounding the false-trigger risk while this
 * restores the memory the FSM needs to act on a real crossing.
 */
static volatile bool     arr_hit       = false;

/*
 * DEPARTURE BURST RECORDER -- an instrument, not a detector.
 *
 * Question it exists to answer (Dave, 2026-08-11): does a jog have a signature
 * that differentiates it from a real departure? The hypothesis is that a jog
 * produces TWO large transients about a second apart -- its own departure
 * bounce and its stop bounce -- where a real departure produces one and then
 * goes quiet. That is a difference in TIME STRUCTURE, not amplitude.
 *
 * It cannot be answered from the normal log. LOG_DECIMATE_N is 8 and ~20% of
 * snapshots are lost to overrun, so printed samples land ~670 ms apart -- and
 * the structure being resolved is about 1 s wide. Two points across the thing
 * you are trying to measure is not a measurement, and the decimated traces
 * already produced one real departure (the 22 s run, arrival 0.713) that is
 * indistinguishable from a jog. That may be a genuine counter-example or an
 * artefact of sampling; nothing in the current log can tell the difference.
 *
 * So: capture EVERY sample around a departure, undecimated, and dump it once.
 *
 * PRE-TRIGGER. The departure bounce happens before the FSM knows a departure
 * occurred, so a recorder that starts on the trigger misses the very event it
 * is for. This writes continuously into a ring and freezes BURST_POST samples
 * after the trigger, keeping BURST_PRE samples of history.
 *
 * Values are SIGNED (a - zero) in milli-m/s^2, int16 -- 0.001 resolution over
 * +/-32 m/s^2, comfortably past anything this device survives.
 *
 * SIGN IS THE WHOLE POINT for the second question this instrument answers
 * (Dave, 2026-08-11): can velocity be inferred over a jog? A jog accelerates
 * and then decelerates by an equal and opposite amount, so its integral
 * CANCELS to roughly zero net velocity change, while a real departure
 * integrates to the cruise speed and holds it. Storing magnitude would make
 * a deceleration indistinguishable from an acceleration, every sample would
 * sum positive, and nothing would ever cancel -- which is exactly the
 * discriminator being looked for.
 */
#define BURST_N     80      /* 3.2 s at 25 Hz, 160 bytes                     */
/* BURST_POST_DEP / BURST_POST_ARR live in movement_service.h -- single
 * source, shared with the FSM that calls burst_trigger(). See the ⚠️ there. */

/*
 * JOG VERDICT -- log-only classifier, NOT a release path (2026-08-11).
 *
 * A jog latches the alarm until the failsafe because its stop lands inside
 * MIN_TRAVEL_MS and is discarded unseen. Three fixes are measured dead
 * (velocity magnitude, lateral level, arm-on-quiet -- see
 * falcon_signature_2026-08-11.md §5.1); every stillness/quiet approach dies
 * on the same measured fact, that parked is NOISIER than cruise (0.064 vs
 * 0.019/0.047).
 *
 * What survives is the impulse PAIR. A jog is a hand-jerk one way and a
 * brake-jerk the other, net velocity ~0, inside ~1 s. A real departure is an
 * impulse with no compensating partner -- the car keeps going. The 2026-08-11
 * bursts put real 350 fpm departures at opposite/primary impulse ratios of
 * 0.00-0.06; a jog's compensating brake shock demonstrably delivers its
 * impulse (it integrated to 128% of true speed on the one measured stop), so
 * the pair should land near 1.0. The earlier velocity-integral fix failed by
 * comparing MAGNITUDES (a jog out-integrates a gentle 25 fpm ramp); this
 * compares the two signs against each other and never references a speed.
 *
 * Runs once, when the departure burst freezes -- 3.2 s after the latch, so
 * the whole jog (event, stop, aftermath) is inside the window. Armed, the
 * verdict would release ~4 s in; unarmed it prints JOGV and does nothing.
 *
 * THE DEADBAND IS THE UNMEASURED PART. Cruise vibration is symmetric, so on
 * a slow real departure (small one-signed impulse + 2.5 s of cruise) noise
 * feeds both sides equally and inflates the ratio toward 1 -- the exact
 * false-JOG that would silence the beacon on a moving car. 150 mm/s^2 sits
 * above the cartop cruise peaks seen so far (70-280 windowed, raw samples
 * mostly lower) but NO jog and NO slow-departure burst has been captured
 * yet. DO NOT ARM until both populations exist and do not touch -- protocol
 * §3.1, and the two structurally-dead fixes both looked right from a
 * hoistway too.
 */
/*
 * Thresholds re-derived 2026-08-11 evening from the first real populations,
 * captured on the cartop with the tether (falcon_jog_verdict_2026-08-11.md):
 *
 *   real departures  n=8  (5x 350 fpm + 3x slow 25-50 fpm, both directions)
 *                         ratio 0-20   opk 27-396
 *   jogs             n=4  (blip to ~1 s, both directions)
 *                         ratio 46-95  opk 1914-3194
 *
 * No overlap on either axis, and the roles reversed from the design guess:
 * OPK IS THE PRIMARY AXIS (the uncontrolled brake set hammers 1.9-3.2 into
 * the structure; no real departure exceeded 0.4 -- Dave's mechanism: drive
 * energizes, brake releases, possible rollback, then brake sets with no
 * field control). Ratio is the supporting check; it alone would have missed
 * the first 1 s jog at 46 against the original 50 gate. Gates sit at the
 * geometric mean of the populations: real departures pass 2.3-2.7x below,
 * jogs 1.4-2.2x above, on both axes simultaneously. Original design-guess
 * gates were 50/450; the rollback content of real slow departures (opk up
 * to 396) is what pushed the peak gate up, not the jogs.
 */
/*
 * ARMED 2026-08-11, Dave's decision after the concern was raised and
 * reaffirmed: evidence base is 15/15 but one day, one car, one mounting
 * deep. A JOG verdict now silences the alarm ~4 s after the latch via
 * MovementService::jog_release(). If a false JOG ever releases a moving
 * car, set this back to 0 first and diagnose from the JOGV lines after.
 */
#define JOG_VERDICT_ARMED   1    /* verdict JOG -> release the latch        */
#define JOG_DEADBAND_MMSS   150  /* samples below this feed neither impulse */
#define JOG_OPP_RATIO_PCT   33   /* opposite/primary >= this -> jog...      */
#define JOG_OPP_PEAK_MMSS   900  /* ...AND opposite-side peak >= this       */

static volatile int16_t  burst[BURST_N];
static volatile uint8_t  burst_head  = 0;
static volatile uint8_t  burst_post  = 0xFF;   /* 0xFF = not triggered       */
static volatile bool     burst_ready = false;  /* frozen, waiting for dump   */

static volatile uint8_t  burst_kind = 0;       /* 0 = departure, 1 = arrival */

/*
 * Arm the recorder. `post` is how many samples to keep AFTER the trigger; the
 * remaining BURST_N - post are pre-trigger history.
 *
 * THE TWO TRIGGERS WANT OPPOSITE SPLITS, because they fire at opposite ends of
 * the event they are recording:
 *
 *   DEPARTURE fires at the START -- the any-motion edge arrives within ~100 ms
 *     of the machine moving -- so the interesting data is ahead of it. 20 pre
 *     / 60 post, the pre-roll only there to catch the bounce that precedes the
 *     FSM knowing anything happened.
 *
 *   ARRIVAL fires wherever the peak first crosses ARRIVAL_PEAK_VALUE, and
 *     WHERE THAT LANDS DEPENDS ON THE STOP:
 *
 *       slow brake stop -- the deceleration is too gentle to cross, so the FSM
 *         waits and fires on the brake set, AFTER the ramp. Wants pre-heavy.
 *       fast drive stop -- the ramp itself crosses, early in the deceleration.
 *         Wants post-heavy.
 *
 *     These are irreconcilable in one split, and 20 pre / 60 post picks the
 *     fast case, measured 2026-08-11: two 350 fpm automatic runs in a cab both
 *     closed the window still at +/-0.61 m/s^2, having captured only 35-40% of
 *     the speed change, so the full stop was never seen. A 350 fpm stop needs
 *     ~2.9 s at the drive's ~0.6 m/s^2 and the ramp is already ~0.4 s old at
 *     the trigger, so 60 post (2.4 s) covers nearly all of it and 20 pre (0.8 s)
 *     comfortably holds the ramp's start.
 *
 *     THE COST IS THE SLOW-STOP DIAGNOSTIC: a 48 fpm ramp already fell outside
 *     even a 2.4 s pre-roll (the trigger is downstream of the event), so that
 *     case was not being captured either way -- but if it is ever chased, this
 *     constant goes back to 20 and the fix is a separate lower trigger, not a
 *     bigger window.
 *
 * Ignored if a burst is already armed or waiting to be dumped. On a short run
 * the departure burst may still be in flight when the arrival fires; the
 * departure wins, which is the right precedence -- on a movement that brief the
 * departure burst already contains the stop.
 */
void burst_trigger(uint8_t post, uint8_t kind)
{
    noInterrupts();
    if (!burst_ready && burst_post == 0xFF) {
        burst_post = post;
        burst_kind = kind;
    }
    interrupts();
}

/* Called from the FSM on entering STATE_MOVING. */
void arrival_peak_reset()
{
    noInterrupts();
    arr_peak_cur  = 0.0f;
    arr_peak_prev = 0.0f;
    arr_bucket_ms = millis();
    arr_quiet     = 0;
    arr_armed     = false;
    arr_hit       = false;
    interrupts();
}

/* True once the windowed peak has crossed ARRIVAL_PEAK_VALUE this run. */
bool arrival_peak_hit()
{
    bool v;
    noInterrupts();
    v = arr_hit;
    interrupts();
    return v;
}

float arrival_peak_get()
{
    float a, b;
    noInterrupts();
    a = arr_peak_cur;
    b = arr_peak_prev;
    interrupts();
    return (a > b) ? a : b;
}

void arrival_zero_set(float z)
{
    noInterrupts();
    arr_zero = z;
    interrupts();
}

/*
 * Cleared until acceleration_avg_g has been primed from a real sample. See
 * read_acceleration_mss() for why an unprimed window is dangerous.
 */
static volatile bool         accel_avg_primed = false;

/*
 * Print one line per N published samples. At the current ~3 Hz ISR rate one
 * line per sample costs about 15% of the 9600 baud budget. Raise this when the
 * timer is fixed to 100 Hz, or serial becomes the new bottleneck.
 */
/*
 * 1 -> 8 with the move to 25 Hz, to hold the serial load where it already was.
 *
 * A sample line is ~145 characters, which at 9600 baud takes ~151 ms to send.
 * One line per sample was 47% of the budget at 3.13 Hz; at 25 Hz it would be
 * 3.8x more than the link can carry. 8 gives 3.1 lines/s -- the same 47%.
 *
 * ⚠️ Serial.print() BLOCKS once the 64-byte TX buffer fills, for roughly 85 ms
 * of each line. The ISR keeps running through that (it is an interrupt), but
 * loop() does not, so a snapshot can be overwritten before it is consumed --
 * which is what ov= counts. At 3.13 Hz a 151 ms line fitted inside a 320 ms
 * period with room to spare; at 25 Hz it spans two 40 ms periods.
 *
 * EXPECT ov= TO GROW, and read it as sample loss in the detectors, not just in
 * the log -- vel_window and lat_monitor are fed from the same snapshot. Rough
 * estimate is 2 samples lost per printed line, so ~25%. That is tolerable for
 * characterising the new rate and NOT tolerable for shipping. The proper fix is
 * a small ring buffer of samples in place of the single snapshot, so loop() can
 * drain several after a blocking print; do that before trusting any threshold
 * derived at this rate.
 */
#define LOG_DECIMATE_N  8

/*
 * Battery thresholds, in raw ADC counts.
 *
 * NOT millivolts. Release.txt describes a 3.2 V trip point but the code has
 * always compared against a raw count, and the scale factor between the two has
 * never been established -- see the open question in Eng_Notes §8. Do not
 * convert these to volts without measuring the divider first.
 *
 * LOW is left at the historical 1600 so this change does not alter when a
 * genuinely flat battery trips. CLEAR sits above it to give hysteresis: with a
 * single threshold, a pack sitting near the boundary would chatter the alarm on
 * and off every measurement cycle. The gap is deliberately wider than the
 * sample-to-sample spread observed on the bench (2324-2390, about 66 counts).
 */
#define BATTERY_LOW_THRESHOLD     1600
#define BATTERY_CLEAR_THRESHOLD   1750

/*
 * Battery readings to discard after boot before the alarm logic is armed, on
 * top of the averaging. See the comment in check_for_battery_voltage().
 */
#define BATTERY_SETTLE_SAMPLES    2

/*
 * Any-motion interrupt test (Eng_Notes §11).
 *
 * OBSERVATION ONLY. The interrupt runs alongside the existing polled detector
 * and changes no FSM behaviour; it exists to answer three questions before any
 * architecture decision is made:
 *
 *   1. Does the INT1_ACC -> PD2 path work at all on this hardware?
 *   2. Does the sensor flag real elevator departures and arrivals?
 *   3. Does the piezo trigger it? §5 says the buzzer couples mechanically into
 *      the accelerometer, and the sensor's engine sits behind the same physics,
 *      so any-motion is expected to see the buzzer too. Measuring how badly is
 *      the point.
 *
 * THRESHOLD -- set from the 2026-08-07 38 fpm down run, not from arithmetic.
 *
 * The first value, 96, was derived from an assumed ~0.488 mg per count (the
 * 11-bit BMA456_ANY_NO_MOTION_THRES_MSK spanning roughly 1 g at RANGE_2G). That
 * LSB has still NOT been confirmed against Datasheet/, so everything below is
 * expressed as a RATIO to the threshold actually used, which stays valid even
 * if the mg-per-count figure is wrong.
 *
 * Measured on the 38 fpm down run at threshold 96:
 *
 *   departure   -0.317 m/s^2   0.67x threshold   MISSED
 *   cruise      +/-0.09 m/s^2  0.19x threshold   silent (correct)
 *   arrival     +0.646 m/s^2   1.37x threshold   DETECTED
 *   stationary  +/-0.02 m/s^2  0.04x threshold   silent (correct)
 *
 * The stationary noise floor turned out to be +/-0.02 m/s^2, eight times quieter
 * than the +/-0.16 measured in July, so there is far more room to drop the
 * threshold than the original analysis assumed.
 *
 * At 48 a departure was detected for the first time: -0.345 m/s^2 at 1.5x
 * threshold, while the polled FSM missed it entirely (its RollingAvg(4) delta
 * was 0.107 against a 0.400 trigger). The FSM alarmed only on the arrival,
 * which at +3.25 m/s^2 raw was large enough to survive the averaging window.
 *
 * Dropped again to 32 for more slow-speed margin. Against the worst non-event
 * excursions recorded so far -- cruise vibration +/-0.09 m/s^2 and stationary
 * noise +/-0.05:
 *
 *   threshold   departure(0.345)   cruise(0.09)   stationary(0.05)
 *      48           1.5x              0.39x           0.22x
 *      32           2.2x              0.58x           0.33x     <- here
 *      24           3.0x              0.78x           0.43x
 *      16           4.5x              1.17x           0.65x
 *
 * 24 and below run into cruise vibration. A threshold that fires mid-ride is
 * worse than one that misses a departure: it destroys the latched FSM's ability
 * to tell a departure from hoistway noise, which is the whole point of §3.
 *
 * If false fires do appear at 32, raise ANYMOTION_DURATION rather than the
 * threshold. Vibration spikes are brief; a departure ramp lasts 1-2 s and will
 * still sustain a longer gate. Raising the threshold instead gives back the
 * slow-speed sensitivity this change exists to buy.
 *
 * ⬜ UNTESTED AT 18 FPM. The transient depends on the acceleration ramp rather
 * than the top speed, so an 18 fpm departure may be similar to the 38 fpm one
 * or may be half of it. If it is half (~0.17 m/s^2) that is 1.1x at this
 * threshold -- marginal -- and 24 would be the next step, accepting the cruise
 * risk above. That run is what decides whether any-motion can solve §3.
 *
 * Z AXIS ONLY. The raw a= samples are logged regardless, so
 * parse_falcon_log.py can sweep thresholds after the fact and one run tests
 * many values; ACC-INT records what the sensor actually did at this setting.
 *
 * Duration is in 50 Hz samples: 5 = 100 ms. This is the hardware equivalent of
 * §7's sustain gating, which found N>=3 consecutive samples eliminated every
 * false fire.
 */
#define ANYMOTION_THRESHOLD       32
#define ANYMOTION_DURATION        5
#define ANYMOTION_INT_LINE        BMA4_INTR1_MAP

/* How often to read/clear the sensor interrupt status, milliseconds. */
#define ACC_INT_POLL_MS           1000

/*
 * Consecutive ACC_INT_POLL_MS polls with any-motion still asserted, while the
 * FSM is monitoring, before the feature is re-armed. See poll_acc_int_status().
 *
 * 8 s at the current poll rate. Long enough that a real departure -- which
 * moves the FSM out of STATE_MONITORING within MOVEMENT_DETECTION_TIMEOUT_MS
 * and so stops the count -- can never reach it.
 */
#define ACC_INT_STUCK_POLLS       8

static volatile uint16_t     acc_int_count = 0;
static volatile uint32_t     acc_int_last_ms = 0;

void acc_int1_isr();
void emit_acc_int_log();
void poll_acc_int_status();

void emit_sample_log();
void emit_burst_log();

uint16_t read_battery_voltage();
extern int configure_adc_channel();
extern uint16_t read_adc_pc2_voltage();
extern bool get_buzzer_status();

void setup() {

    /*
     * Configure the debug serial port here
    */
    Serial.begin(9600);

    Serial.print(F("\r\n\nDevice Booted \r\n"));
    /*
     * Configure the ADC chip here for SPI protocol.
    */
    SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
    SPI.begin();
    SPI.beginTransaction(settings);
    Serial.print(F("Configured SPI interface \r\n"));

    /*
     * Configure all Alarm Ports here
    */
    setup_alarm();
    disable_alarm();

    Serial.print(F("Configured Alarms \r\n"));

    digitalWrite(PIN_GREEN_LED, HIGH);
    init_time_g = millis();

    bma456.initialize(RANGE_2G, ODR_100_HZ, NORMAL_AVG4, CONTINUOUS);
    Serial.print(F("Configured BMA456 \r\n"));

    /*
     * Arm the any-motion engine and listen on INT1_ACC / PD2. Observation only
     * -- see the ANYMOTION_* block above.
     */
    {
        uint16_t rslt = bma456.configureAnyMotion(ANYMOTION_THRESHOLD,
                                                  ANYMOTION_DURATION,
                                                  BMA4_DISABLE,
                                                  ANYMOTION_INT_LINE);
        if (rslt != BMA4_OK) {
            Serial.print(F("AnyMotion config FAILED : "));
            Serial.print(rslt);
            Serial.print(F("\r\n"));
        } else {
            Serial.print(F("AnyMotion armed thr="));
            Serial.print(ANYMOTION_THRESHOLD);
            Serial.print(F(" dur="));
            Serial.print(ANYMOTION_DURATION);
            Serial.print(F("\r\n"));
        }
    }

    pinMode(PIN_ACC_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ACC_INT1), acc_int1_isr, RISING);

    /*
     * Configure the ADC channel for battery voltage measurement using PC2 (ADC2).
     * This will be used to monitor the battery voltage and trigger low battery alarm if needed. 
    */

    configure_adc_channel();
}

void initialization()
{

    /*
     * One device boot-up, read the acceleration data  for
     * some time.
     */

    if (millis() - temp_timer > INIT_TIMER_MS) {

        temp_timer = millis();
//        read_battery_voltage();
        battery_avg.add(read_battery_voltage());
    }

    if (millis() - init_time_g > INIT_TIME_MS) {

        state = SystemStates::SYSTEM_STATE_NOMINAL;
        enable_timer();

        digitalWrite(PIN_PIEZO, HIGH);  
        for (int i = 0; i < 8; i++) {
            digitalWrite(PIN_CHASE_CLK, HIGH);
            delay(100);   
            digitalWrite(PIN_CHASE_CLK, LOW);
            delay(100);   
        }

        Serial.print(F("Device initialized completely \r\n"));
        digitalWrite(PIN_PIEZO, LOW);
        digitalWrite(PIN_GREEN_LED, LOW);
        digitalWrite(PIN_CHASE_LED, HIGH);
        digitalWrite(PIN_CHASE_LED, LOW);
    }
    return;
}


void loop()
{
    switch (state) {

    case SystemStates::SYSTEM_STATE_INITIALIZING:
        initialization();
        break;

    case SystemStates::SYSTEM_STATE_NOMINAL:
        /*
         * Age the velocity window on wall clock before the FSM reads it.
         * Without this it would only decay when a sample arrives, so w() would
         * hold a stale value right through a buzzer blank -- which is exactly
         * when STATE_MOVING is asking about arrival.
         *
         * Baseline tracking is allowed only while monitoring. Anywhere else the
         * tracker would absorb the very transient it exists to expose; a slow
         * departure is precisely the case where that guarantees a miss. Samples
         * taken while the piezo sounds never reach the window at all -- the
         * timer ISR drops them (§5, §11) -- so buzzer coupling is handled a
         * layer down and does not need gating here.
         */
        vel_window.tick(millis());
        vel_window.set_baseline_tracking(
            ms.get_state() == MotionStates::STATE_MONITORING ||
            ms.get_state() == MotionStates::STATE_CALIBERATION);

        ms.fsm_run();

        emit_sample_log();
        emit_burst_log();
        emit_acc_int_log();
        poll_acc_int_status();

        check_for_battery_voltage();

        check_for_active_alarm();
        check_for_battery_alarm();
        break;

    default:
        state = SystemStates::SYSTEM_STATE_HOLD;
        break;
    }

}

/*
 * Commit one record to the ring. ISR context only.
 *
 * Drops the NEW sample when full rather than overwriting the oldest, so the
 * records loop() does drain stay contiguous in time -- vel_window and
 * lat_monitor both reason from consecutive timestamps and a hole in the middle
 * is worse for them than a gap at the end.
 */
static void ring_push(uint32_t t_ms, float accel, float avg, float ax, float ay,
                      uint16_t read_us, uint8_t st, uint8_t err)
{
    uint8_t next = ring_head + 1;

    if (next >= SAMPLE_RING_N) {
        next = 0;
    }
    if (next == ring_tail) {
        sample_overrun++;
        return;
    }

    ring[ring_head].t_ms      = t_ms;
    ring[ring_head].accel     = (int16_t)(accel * 1000.0f);
    ring[ring_head].avg       = (int16_t)(avg   * 1000.0f);
    ring[ring_head].ax        = (int16_t)(ax    * 1000.0f);
    ring[ring_head].ay        = (int16_t)(ay    * 1000.0f);
    ring[ring_head].read_us   = read_us;
    ring[ring_head].fsm_state = st;
    ring[ring_head].err_run   = err;

    ring_head = next;
}

/*
 * Read z-axis accelerometer data and convert to m/(s*s)
 */

void read_acceleration_mss()
{
    uint32_t read_start;
    uint16_t rslt;

    isr_ticks++;

    read_start = micros();
    rslt = bma456.getAcceleration(&x, &y, &z);

    if (rslt != BMA4_OK) {
        /*
         * The sensor did not answer. Do NOT feed anything into the rolling
         * average: x/y/z still hold the last good sample, and zeroing them (as
         * this function used to) drags the average to 0.0, which the FSM reads
         * as "perfectly still". A unit whose accelerometer has died then
         * reports the all-clear forever. See Eng_Notes §10.2.
         *
         * The average is left holding the last good value, so the FSM neither
         * alarms nor clears on stale data while the fault persists.
         */
        sensor_err_total++;
        if (sensor_err_run < 0xFF) {
            sensor_err_run++;
        }

        ring_push(millis(), 0.0f, 0.0f, 0.0f, 0.0f,
                  (uint16_t)(micros() - read_start),
                  (uint8_t)ms.get_state(), sensor_err_run);

        return;
    }

    sensor_err_run = 0;

    g_value = z / 1000.0;
    accel_value = g_value * 9.81;

    /*
     * Prime the window from the first good sample instead of letting it fill
     * from zero.
     *
     * RollingAvg initialises every slot to init_val (0.0 here), so an unprimed
     * acceleration_avg_g(4) reports 0.00, 2.43, 4.86, 7.29 and only then the
     * true ~9.72 as the window fills. That ramp is indistinguishable from a
     * large upward acceleration, and on 2026-08-06 it produced a false
     * STATE_MOVEMENT_DETECTED on a device sitting still on a bench.
     *
     * It then got stuck: the FSM captured its baseline at 2.4265 (one sample
     * into the unprimed window, 9.7152/4) and compared against it forever,
     * giving "FSM: Delta : 7.294032" and "Still in movement" on every cycle
     * with no way back to STATE_MONITORING. The alarm never cleared.
     *
     * Priming makes the average correct from the first sample onward. This is
     * the same defect that was fixed in battery_avg -- see
     * check_for_battery_voltage() -- and the acceleration path is the more
     * damaging place to have it.
     */
    if (!accel_avg_primed) {
        accel_avg_primed = true;
        acceleration_avg_g.fill(accel_value);
    } else {
        acceleration_avg_g.add(accel_value);
    }

    /*
     * Raw arrival peak. Arm once the departure transient has died away, then
     * hold the largest excursion seen. See the ARRIVAL_QUIET_MSS block.
     */
    {
        float dev = fabs(accel_value - arr_zero);

        if (!arr_armed) {
            if (dev < ARRIVAL_QUIET_MSS) {
                if (++arr_quiet >= ARRIVAL_ARM_SAMPLES) {
                    arr_armed = true;
                    arr_bucket_ms = millis();
                }
            } else {
                arr_quiet = 0;
            }
        } else {
            uint32_t now = millis();

            /* Roll the window: the open bucket becomes the previous one. */
            if ((uint32_t)(now - arr_bucket_ms) >= ARRIVAL_PEAK_WINDOW_MS) {
                arr_peak_prev = arr_peak_cur;
                arr_peak_cur  = 0.0f;
                arr_bucket_ms = now;
            }

            if (dev > arr_peak_cur) {
                arr_peak_cur = dev;
            }

            if (arr_peak_cur > ARRIVAL_PEAK_VALUE) {
                arr_hit = true;
            }
        }

        /*
         * Burst recorder. Writes continuously so the pre-trigger history is
         * always there; freezes once the post-trigger count runs out, and
         * stops writing until loop() has dumped it.
         */
        if (!burst_ready) {
            {
                float sdev = accel_value - arr_zero;
                int16_t sv;

                if (sdev >= 32.0f)       sv =  32000;
                else if (sdev <= -32.0f) sv = -32000;
                else                     sv = (int16_t)(sdev * 1000.0f);

                burst[burst_head] = sv;
            }
            burst_head = (uint8_t)((burst_head + 1) % BURST_N);

            if (burst_post != 0xFF) {
                if (burst_post > 0) {
                    burst_post--;
                } else {
                    burst_post  = 0xFF;
                    burst_ready = true;
                }
            }
        }
    }

    /*
     * Publish a snapshot for loop() to print. No serial I/O in here.
     */
    ring_push(millis(), accel_value, acceleration_avg_g.avg(),
              x / 1000.0f * 9.81f, y / 1000.0f * 9.81f,
              (uint16_t)(micros() - read_start),
              (uint8_t)ms.get_state(), 0);
}

/*
 * Dump a frozen departure burst. loop() only.
 *
 * ~80 values of up to 5 digits is roughly 350 characters, so this blocks the
 * serial link for about a third of a second. That is acceptable and bounded:
 * the peak detector and arr_hit live entirely in the ISR, so a blocked loop()
 * delays the FSM's reaction rather than corrupting the decision, and the burst
 * itself is already captured. The velocity and lateral windows lose samples
 * for the duration -- both are instrumentation now, so that is a fair trade
 * for the one measurement that can settle the jog question.
 *
 * BURST_PRE samples of pre-trigger history come first, so the trigger sits at
 * a known offset and the departure bounce is inside the window rather than
 * clipped off its front.
 */
void emit_burst_log()
{
    uint8_t i, head;

    if (!burst_ready) {
        return;
    }

    noInterrupts();
    head = burst_head;
    interrupts();

    Serial.print(F("BURST k="));
    Serial.print(burst_kind ? F("arr") : F("dep"));
    Serial.print(F(" pre="));
    Serial.print(burst_kind ? (BURST_N - BURST_POST_ARR) : (BURST_N - BURST_POST_DEP));
    Serial.print(F(" n="));
    Serial.print(BURST_N);
    Serial.print(F(" signed_mmss="));

    /* Oldest first: the ring is full, so the oldest entry is at head. */
    for (i = 0; i < BURST_N; i++) {
        Serial.print(burst[(uint8_t)((head + i) % BURST_N)]);
        Serial.print(' ');
    }
    Serial.print(F("\r\n"));

    /*
     * Jog verdict, departure bursts only -- see the block above JOG_VERDICT_ARMED.
     * Integer throughout: impulses in mm/s^2-samples (int32 headroom 80*32767),
     * ratio in percent. Everything the verdict used is printed so thresholds
     * can be re-derived from any log without reflashing.
     */
    if (burst_kind == 0) {
        int32_t  pos = 0, neg = 0;      /* deadbanded impulse, each sign     */
        int16_t  ppk = 0, npk = 0;      /* raw signed peaks, either sign     */
        int16_t  sv;
        uint8_t  ratio_pct;
        int32_t  pri, opp;

        for (i = 0; i < BURST_N; i++) {
            sv = burst[(uint8_t)((head + i) % BURST_N)];
            if (sv > ppk) ppk = sv;
            if (sv < npk) npk = sv;
            if (sv >  JOG_DEADBAND_MMSS) pos += sv;
            if (sv < -JOG_DEADBAND_MMSS) neg -= sv;   /* neg accumulates positive */
        }

        pri = (pos >= neg) ? pos : neg;
        opp = (pos >= neg) ? neg : pos;
        /* opposite-side raw peak: the sign that contributed the smaller impulse */
        sv  = (pos >= neg) ? (int16_t)-npk : ppk;

        ratio_pct = (pri > 0) ? (uint8_t)((opp * 100) / pri) : 0;

        Serial.print(F("JOGV pos="));  Serial.print(pos);
        Serial.print(F(" neg="));      Serial.print(neg);
        Serial.print(F(" ratio="));    Serial.print(ratio_pct);
        Serial.print(F(" opk="));      Serial.print(sv);
        Serial.print(F(" verdict="));
        if (ratio_pct >= JOG_OPP_RATIO_PCT && sv >= JOG_OPP_PEAK_MMSS) {
            Serial.print(F("JOG"));
#if JOG_VERDICT_ARMED
            ms.jog_release();
#endif
        } else {
            Serial.print(F("RUN"));
        }
#if JOG_VERDICT_ARMED
        Serial.print(F(" (armed)\r\n"));
#else
        Serial.print(F(" (unarmed)\r\n"));
#endif
    }

    noInterrupts();
    burst_ready = false;
    interrupts();
}

/*
 * Print the most recently published sample. Called from loop() only.
 */
void emit_sample_log()
{
    static uint8_t  decimate = 0;
    sample_log_t    s;
    uint16_t        overrun;
    uint16_t        ticks;
    uint16_t        err_total;

    /*
     * Drain everything the ISR has queued, not just the newest record.
     *
     * ONE PASS PER CALL, deliberately: the caller is loop(), which runs far
     * faster than 25 Hz, so a queue never persists -- and returning after each
     * record keeps the FSM being serviced between them rather than stalling it
     * behind a backlog we are only draining for the log's benefit.
     *
     * No masking needed. Single producer, single consumer, and uint8_t access
     * is atomic on AVR: the ISR only advances head, this only advances tail.
     */
    if (ring_tail == ring_head) {
        return;
    }

    memcpy(&s, (const void *)&ring[ring_tail], sizeof(s));
    ring_tail = (uint8_t)((ring_tail + 1) >= SAMPLE_RING_N ? 0 : ring_tail + 1);

    overrun   = sample_overrun;
    ticks     = isr_ticks;
    err_total = sensor_err_total;

    /*
     * Feed the velocity window.
     *
     * Deliberately here and not in read_acceleration_mss(). That runs in the
     * timer ISR, and the FSM reads w()/valid() from loop() -- float and
     * multi-byte state shared across that boundary would tear, with no
     * atomicity on AVR. Consuming the published snapshot keeps every access to
     * the window in loop() context, which is the same reason §6 moved printing
     * out of the ISR.
     *
     * BEFORE the decimation return: LOG_DECIMATE_N thins the log, and must
     * never thin the detector. Samples lost to snapshot overrun are simply not
     * credited -- the window's coverage accounting reports the shortfall rather
     * than interpolating across it.
     */
    if (!s.err_run) {
        vel_window.add(s.accel / 1000.0f, s.t_ms);

        /*
         * Same placement, same reasons: the FSM reads the quiet run from
         * loop(), and the snapshot has already had the buzzer blanking and
         * the failed-read guard applied to it. Also before the decimation
         * return -- LOG_DECIMATE_N thins the log and must never thin a
         * detector, least of all one that RELEASES the beacon.
         */
        lat_monitor.add(s.ax / 1000.0f, s.ay / 1000.0f, s.t_ms);
    }

    if (++decimate < LOG_DECIMATE_N) {
        return;
    }
    decimate = 0;

    Serial.print(F("t="));
    Serial.print(s.t_ms);

    if (s.err_run) {
        /*
         * Failed read. Print the fault rather than a stale or fabricated
         * value, so a dead sensor is visible in the log instead of looking
         * like a stationary car.
         */
        Serial.print(F(" a=ERR er="));
        Serial.print(s.err_run);
    } else {
        Serial.print(F(" a="));
        Serial.print(s.accel / 1000.0f, 3);
        Serial.print(F(" avg="));
        Serial.print(s.avg / 1000.0f, 3);
    }

    Serial.print(F(" st="));
    Serial.print(s.fsm_state);
    Serial.print(F(" rd="));
    Serial.print(s.read_us);
    Serial.print(F(" ov="));
    Serial.print(overrun);
    Serial.print(F(" tk="));
    Serial.print(ticks);
    Serial.print(F(" im="));
    Serial.print(acc_int_count);

    /*
     * Velocity window: signed integral and how much of it was actually
     * observed. cv= well under VEL_WINDOW_MS means the buzzer or a fault is
     * eating samples and w= is built on less data than it looks like -- the
     * one thing a capture must not hide. Costs ~14 characters a line, about
     * 5% more of the 9600 baud budget.
     */
    Serial.print(F(" w="));
    Serial.print(vel_window.w(), 3);
    Serial.print(F(" cv="));
    Serial.print(vel_window.coverage_ms());

    /*
     * Lateral axes. Exploration only -- nothing reads these yet. The question
     * they exist to answer is whether |x|,|y| separate a travelling unit from a
     * parked one, which would give a cruise-confirm and 3 s false-alarm veto
     * that z physically cannot (see the sample_log_t comment).
     */
    Serial.print(F(" x="));
    Serial.print(s.ax / 1000.0f, 3);
    Serial.print(F(" y="));
    Serial.print(s.ay / 1000.0f, 3);

    /*
     * The lateral metric the release path actually decides on, and the quiet
     * run it has accumulated. x= and y= alone are not a substitute: the
     * metric is a difference between CONSECUTIVE UNBLANKED samples, and the
     * log does not show which samples the ISR dropped, so it cannot be
     * reconstructed offline. q= is what makes a release explicable after the
     * fact -- and, during the buzzer bench test, what shows whether the
     * metric ever gets under the threshold at all.
     */
    Serial.print(F(" m="));
    Serial.print(lat_monitor.m(), 3);
    Serial.print(F(" q="));
    Serial.print(lat_monitor.quiet_run());

    /*
     * Raw arrival peak. The log is decimated, so this is the ONLY way the
     * capture shows what the detector actually saw -- the individual sample
     * carrying the brake bounce is very unlikely to be one of the printed
     * ones.
     */
    Serial.print(F(" pk="));
    Serial.print(arrival_peak_get(), 2);

    if (err_total) {
        Serial.print(F(" et="));
        Serial.print(err_total);
    }

    Serial.print(F("\r\n"));
}

void enable_timer()
{
    cli();
    TCCR1B = 0;

    /*
     * Clear the counter and load OCR1A BEFORE the waveform mode and the clock
     * are selected. Two hazards, both observed on the bench 2026-08-06:
     *
     * 1. TCNT1 was never cleared. In mode 9 the counter cannot reach a TOP that
     *    is below it, so if TCNT1 held a large value it had to run all the way
     *    to 0xFFFF and wrap first. At F_CPU/1024 = 976 Hz that is up to 67
     *    seconds before the first compare match.
     *
     * 2. OCR1A is DOUBLE BUFFERED in mode 9. Writing it after the mode bits, as
     *    this function used to, does not take effect until the counter reaches
     *    the *previous* TOP -- so the first period is governed by whatever
     *    OCR1A happened to hold, not by 156. Written here while TCCR1B is still
     *    0 (normal mode), the write is immediate and unbuffered.
     *
     * The symptom was one sensor read at t=5275 and the next at t=72335, then
     * a perfectly regular 3.13 Hz. The 10-second self-calibration in between
     * ran on that single sample: Zero-Calib-Value came out as exactly the one
     * reading taken at t=5275.
     *
     * NOTE: the prescaler and waveform mode are deliberately left as they are.
     * Both are wrong (Eng_Notes §2) and fixing them is roadmap item 5, which
     * changes the sample rate 32x and invalidates every tuning number in the
     * document. This change is only about the timer STARTING predictably, so
     * the rate stays at 3.13 Hz and detection behaviour is unchanged.
     */
    /*
     * ROADMAP ITEM 5, 2026-08-10. 3.13 Hz -> 25 Hz.
     *
     * The old configuration was phase-and-frequency-correct PWM (mode 9, TOP =
     * OCR1A) with a 1024 prescaler and OCR1A = 156. That mode counts UP to TOP
     * and back DOWN, so the period is doubled:
     *
     *     2 * 156 * 1024 / 1000000 = 319.5 ms  ->  3.13 Hz
     *
     * which is exactly the rate measured in every capture. Nothing wanted a PWM
     * waveform here -- the timer exists only to raise an interrupt -- so CTC
     * (mode 4) is both correct and half the period for the same numbers:
     *
     *     (624 + 1) * 64 / 1000000 = 40.000 ms  ->  25.000 Hz
     *
     * WHY 25 AND NOT 100. The I2C read of the accelerometer measures 7.5 ms
     * (rd= in every log line), which is a hard floor on the sample period. At
     * 25 Hz that is 19% of the budget. 100 Hz would be 75% and leaves nothing
     * for the rest of the ISR; 50 Hz at 37% is probably reachable and should be
     * earned by measurement, not assumed. Raising F_CPU off the 1 MHz fuse is
     * what actually unlocks the sensor's full 100 Hz output rate.
     *
     * TCCR1A MUST BE CLEARED. Arduino's init() leaves WGM10 set, which combined
     * with WGM13 is what selected mode 9 in the first place. CTC needs WGM12
     * alone, so the low bits cannot be left to whatever ran before.
     *
     * ⚠️ THIS INVALIDATES SAMPLE-RATE-DEPENDENT TUNING. The rolling average is
     * widened 4 -> 32 in step, so every threshold expressed against it keeps
     * its 1.28 s time constant and its existing meaning. velocity.h's constants
     * are NOT rescaled -- that module is unarmed, so w= changes meaning in the
     * log but nothing acts on it. lateral.h needs no change: XY_STILL is
     * learned at calibration and adapts to the new sample spacing by itself.
     */
    /*
     * ORDER IS LOAD-BEARING, and getting it wrong cost a flash cycle on
     * 2026-08-10. OCR1A is DOUBLE BUFFERED in every PWM waveform mode: the
     * write lands in a shadow register and is only copied to OCR1A when the
     * counter reaches TOP. Arduino's init() leaves WGM10 set in TCCR1A, so
     * until TCCR1A is cleared the timer is in a PWM mode and any OCR1A write
     * is buffered rather than applied.
     *
     * So: clear BOTH control registers first, which selects normal mode where
     * OCR1A is written directly; then load TOP; then select CTC; then start
     * the clock. Each step is only ever entered from a known state.
     */
    TCCR1A = 0;                 /* normal mode -- OCR1A writes go straight in */
    TCNT1  = 0;
    OCR1A  = 624;               /* TOP: (624+1) * 64 / 1e6 = 40.000 ms        */

    TCCR1B = (1 << WGM12);      /* CTC, TOP = OCR1A                            */
    TIMSK1 |= (1 << OCIE1A);    /* compare-match A interrupt                   */

    /* clk/64 -- start the clock last, once everything else is settled */
    TCCR1B |= ((1 << CS11) | (1 << CS10));
    sei();
}

void disable_timer1()
{
    // reset Control Register to disable timer
    TCCR1B = 0;
}


ISR(TIMER1_COMPA_vect) 
{
    /*
     * If we are already inside an ISR, just bail out.
     */
    if(in_isr) {
        return;
    }

    in_isr = true;

    /*
     * Re-enable interrupts so that interrupt-based functions can 
     * be used inside this function
     */
    interrupts();

    if (get_alarm_status()) {
        if (get_buzzer_status() == false) {
            read_acceleration_mss();
        }
    } else {
        read_acceleration_mss();
    }

    /*
     * Turn off interrupts so we can't be interrupted while 
     * resetting our special variable
     */  
    noInterrupts();

    in_isr = false;
    return;
}

/*
 * Any-motion interrupt handler. Deliberately trivial: no I2C, no serial, no
 * FSM. The sensor's status register is not read here -- that would be an I2C
 * transaction from an ISR, and §6 is what happens when this codebase does I/O
 * from interrupt context. loop() reports the count; the sensor latches its own
 * status until read, so nothing is lost by not reading it immediately.
 */
void acc_int1_isr()
{
    acc_int_count++;
    acc_int_last_ms = millis();
}

/*
 * Report new any-motion edges. Called from loop() only.
 *
 * The buzzer state is printed alongside because §5's mechanical coupling is the
 * main thing that could sink this approach: if bz=1 accompanies most edges, the
 * sensor is detecting its own piezo rather than the car, and the threshold has
 * to discriminate between them -- or it cannot be used while alarming, which is
 * exactly the deadlock the polled version already has.
 */
void emit_acc_int_log()
{
    static uint16_t last_reported = 0;
    uint16_t        count;
    uint32_t        when;

    noInterrupts();
    count = acc_int_count;
    when  = acc_int_last_ms;
    interrupts();

    if (count == last_reported) {
        return;
    }
    last_reported = count;

    /*
     * Hand the departure to the FSM. It only acts on this in STATE_MONITORING,
     * so the constant stream of interrupts the buzzer raises while alarming
     * (Eng_Notes §11) is harmless.
     *
     * Done here rather than in the ISR so the FSM is only ever touched from
     * loop() -- §6 is what happens when this codebase does work in interrupt
     * context.
     */
    ms.notify_any_motion(when);

    Serial.print(F("ACC-INT n="));
    Serial.print(count);
    Serial.print(F(" t="));
    Serial.print(when);
    Serial.print(F(" st="));
    Serial.print(ms.get_state());
    Serial.print(F(" bz="));
    Serial.print(get_buzzer_status() ? 1 : 0);
    Serial.print(F(" pin="));
    Serial.print(digitalRead(PIN_ACC_INT1));
    Serial.print(F("\r\n"));
}

/*
 * Poll and clear the sensor's interrupt status.
 *
 * BMA4_NON_LATCH_MODE alone did not re-arm the pin on the bench: one edge at
 * t=35471 and then nothing across three handling events the polled detector
 * caught easily. Either the mode is being overwritten by the feature-config
 * write or it is not doing what the datasheet implies, so this reads the status
 * register periodically, which clears the condition explicitly and drops the
 * pin so the next motion produces a fresh rising edge.
 *
 * The pin level is logged alongside because it distinguishes the two failure
 * modes at a glance:
 *
 *   pin=1 persistently  -> the interrupt IS asserted and stuck; latching is the
 *                          problem and this poll should fix it
 *   pin=0 with s=0      -> the sensor is not re-asserting at all; the threshold
 *                          is wrong, or any-motion is not actually enabled
 *
 * Rate-limited because each read is a ~6 ms I2C transaction (§10.4).
 */
void poll_acc_int_status()
{
    static uint32_t last_poll = 0;
    static uint8_t  stuck_polls = 0;
    uint16_t        status = 0;
    uint16_t        rslt;

    if (millis() - last_poll < ACC_INT_POLL_MS) {
        return;
    }
    last_poll = millis();

    rslt = bma456.readInterruptStatus(&status);

    /*
     * Only speak up when there is something to say: a live status bit, a stuck
     * pin, or a failed read. Otherwise this would add a line every second.
     */
    if (rslt != BMA4_OK) {
        Serial.print(F("ACC-STAT read FAILED\r\n"));
        return;
    }

    /*
     * Stuck any-motion reference -- see the datasheet review, finding 3.
     *
     * BST-BMA456-AN002 p.7: any-motion tests |Acc - Reference|, and "reference
     * acceleration sample is updated only when an any-motion interrupt is
     * triggered". If the reference is left stale at a level the device no
     * longer sits at, the slope never falls back under the threshold, the
     * output never de-asserts, and an attachInterrupt(RISING) handler sees ONE
     * edge and then silence for the rest of the deployment. Departure detection
     * would be dead with nothing in the log to say so -- the catastrophic
     * direction (§14.4), and the half §14.9 calls the one that works.
     *
     * Non-latch mode does not protect against this: it clears the pin when the
     * CONDITION passes, and with a stale reference the condition does not pass.
     *
     * The measured data says we are not currently in that state -- §13.1 and
     * §14 show zero any-motion edges through cruise and working arrival
     * clusters afterwards, neither possible with a pin stuck high -- so the
     * sensor evidently re-references more often than AN002 rev 0.1 describes.
     * This is here because the cost is one counter and the failure is silent.
     *
     * Only counted in STATE_MONITORING: while MOVING the output is SUPPOSED to
     * be asserted, and re-arming there would destroy the arrival cluster.
     */
    if ((status & BMA456_ANY_NO_MOTION_INT) &&
        ms.get_state() == MotionStates::STATE_MONITORING) {
        if (stuck_polls < 0xFF) {
            stuck_polls++;
        }
    } else {
        stuck_polls = 0;
    }

    if (stuck_polls >= ACC_INT_STUCK_POLLS) {
        /*
         * Disable and re-enable the feature. That forces the sensor to take a
         * fresh reference at the level the device actually sits at now.
         */
        Serial.print(F("ACC-STAT STUCK "));
        Serial.print(stuck_polls);
        Serial.print(F(" polls, re-arming any-motion\r\n"));

        stuck_polls = 0;

        rslt = bma456.configureAnyMotion(ANYMOTION_THRESHOLD,
                                         ANYMOTION_DURATION,
                                         BMA4_DISABLE,
                                         ANYMOTION_INT_LINE);
        if (rslt != BMA4_OK) {
            Serial.print(F("ACC-STAT re-arm FAILED : "));
            Serial.print(rslt);
            Serial.print(F("\r\n"));
        }
        return;
    }

    if (status != 0 || digitalRead(PIN_ACC_INT1) != 0) {
        Serial.print(F("ACC-STAT s=0x"));
        Serial.print(status, HEX);
        Serial.print(F(" pin="));
        Serial.print(digitalRead(PIN_ACC_INT1));
        Serial.print(F(" n="));
        Serial.print(acc_int_count);
        Serial.print(F(" sk="));
        Serial.print(stuck_polls);
        Serial.print(F("\r\n"));
    }
}

inline uint16_t read_battery_voltage()
{
    float vol_temp = (((float) adc.read(BATT_SENSE) / 4096.0) * 3300) * (VBATT_CONST) * (1.06);

    return (uint16_t)vol_temp;
}

/*
 * This function will check for battery-voltage level and notify the user
 */
void check_for_battery_voltage()
{
    static unsigned int adc_loop_counter = 0;
    static uint8_t      settled_samples  = 0;
    uint16_t  battery_v = 0;

    adc_loop_counter++;

    if (adc_loop_counter == 30000) {
        digitalWrite(BAT_ADC_ENABLE, HIGH);
    }

    if (adc_loop_counter == 30100) {
        adc_loop_counter = 0;
        battery_v = read_adc_pc2_voltage();

        Serial.print(F("  Voltage value : "));
        Serial.print(battery_v);

        digitalWrite(BAT_ADC_ENABLE, LOW);

        /*
         * Discard the opening samples entirely.
         *
         * The 30000/30100 counters are loop passes, not milliseconds, so the
         * divider gets a very short settle at F_CPU = 1 MHz. The first read
         * also lands during the startup transient, with the boot buzzer pulse
         * and the chase LEDs loading the rail. On 2026-08-06 that produced a
         * single 1545 on a healthy battery that read 2324-2390 for the rest of
         * the session -- and because the alarm had no recovery path, that one
         * sample latched it permanently. See Eng_Notes §10.3.
         */
        if (settled_samples < BATTERY_SETTLE_SAMPLES) {
            settled_samples++;
            Serial.print(F("  (settling, ignored)\r\n"));
            return;
        }

        /*
         * Prime the whole window from the first sample we trust.
         *
         * RollingAvg fills its array with init_val (0 here), so a freshly
         * constructed battery_avg reads 0 and climbs one eighth of a sample at
         * a time. Calling add() on an unprimed window would put avg() near 300
         * for the first reading and trip the low-battery test on a perfectly
         * good battery -- reintroducing the bug this change exists to fix.
         */
        if (settled_samples == BATTERY_SETTLE_SAMPLES) {
            settled_samples++;
            battery_avg.fill(battery_v);
        } else {
            battery_avg.add(battery_v);
        }

        Serial.print(F("  avg : "));
        Serial.print(battery_avg.avg());
        Serial.print(F("\r\n"));

        /*
         * Decide on the rolling average, never on a single sample, and use
         * separate trip and clear thresholds so a reading hovering at the
         * boundary cannot chatter the alarm on and off.
         */
        if (battery_avg.avg() < BATTERY_LOW_THRESHOLD) {
            if (battery_alarm_status_g == 0) {
                Serial.print(F("  LOW Battery detected \r\n"));
            }
            enable_battery_alarm();

        } else if (battery_avg.avg() > BATTERY_CLEAR_THRESHOLD) {
            /*
             * The recovery path that did not exist before. disable_battery_alarm()
             * was defined in alarm.cpp and never called from anywhere in the
             * tree, so a latched alarm survived until a true cold boot -- which,
             * given the serial back-feed in §10.1, is harder to achieve than it
             * sounds.
             */
            if (battery_alarm_status_g) {
                Serial.print(F("  Battery recovered, alarm cleared \r\n"));
            }
            disable_battery_alarm();
        }
    }
}
