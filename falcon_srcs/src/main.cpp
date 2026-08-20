/*
 ************************************************************
 *
 * Copyright Spanidea 2024-25
 ************************************************************
 */

#include "main.h"
#include "falcon_log.h"
#include "arduino_bma456.h"
#include "common.h"
#include "velocity.h"
#include "lateral.h"
#include <avr/wdt.h>

/*
 * Wire1 is vendored into lib/Wire1 so its TWI wait loops can be bounded -- see
 * the bounded-waits block in lib/Wire1/src/utility/twi1.c for why, and for the
 * rollback. This is the trip counter, logged as tw= when nonzero.
 */
extern "C" uint8_t twi1_guard_trips1(void);

/*
 * LOCKUP DEFENCES -- added 2026-08-12 after three freezes in one session.
 *
 * Mechanism: the sample read runs in the timer ISR, and Wire1's TWI driver
 * waits in unbounded while-loops (twi_timeout_us defaults to 0 = disabled).
 * A bus glitch mid-transaction -- rail disturbance from the piezo/LEDs is the
 * suspected source; all three freezes followed alarm activity -- wedges the
 * TWI state machine, the ISR never returns, loop() never resumes, and the
 * device is a silent corpse with an LED latched until an external reset.
 * 25 Hz made this ~15x more exposed than 3.13 Hz: 8x the transactions at
 * ~11.5 ms each puts the bus busy ~30% of wall clock.
 *
 * The defence that fits: a 2 s watchdog, kicked from loop(). A wedged TWI
 * wait stops loop() from ever being reached, the WDT expires, and the device
 * reboots into a fresh calibration instead of freezing -- on a beacon a
 * ~12 s self-recovery beats a silent corpse on the counterweight. Longest
 * legitimate loop() stalls are well inside 2 s (burst dump ~350 ms, init
 * chase ~1.6 s, ready chirps ~1.0 s).
 *
 * ⬜ The defence that does NOT fit yet: -DWIRE_TIMEOUT bounds the waits in
 * the driver itself, turning a wedge into an ordinary failed read that the
 * err_run path already handles -- no reboot, no lost run. It measured
 * +1818 bytes against 714 free. Enable it (platformio.ini) plus
 * twi_setTimeoutInMicros1(25000, true) here the moment the bma4 driver swap
 * frees its ~4.9 KB.
 *
 * The .init3 hook below runs BEFORE C++ constructors: it captures the reset
 * cause and disables the watchdog immediately, because after a WDT reset the
 * hardware re-arms the WDT at 16 ms -- waiting until setup() risks a reset
 * loop. Boot cause is printed in setup(): WDRF (0x8) in that byte means the
 * previous boot died to the watchdog, i.e. a lockup was caught in the field.
 */
uint8_t boot_mcusr __attribute__((section(".noinit")));

void wdt_early_disable(void) __attribute__((naked, used, section(".init3")));
void wdt_early_disable(void)
{
    boot_mcusr = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

uint8_t alarm_status_g = 0;
uint8_t chase_led_status_g = 0;
uint8_t battery_alarm_status_g = 0;
/* temp_timer went with the MCP3208 feed removed from initialization(). */
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
#if FALCON_LOG == 0
FalconNullLog falcon_null_log;
#endif

RollingAvg<float, 32> acceleration_avg_g;
RollingAvg<uint16_t, 8> battery_avg;
float x = 0, y = 0, z = 0;
float g_value, accel_value;

MCP3208 adc(ADC_VREF, PIN_ADC_CS);
MovementService ms(&acceleration_avg_g);

#define EN_3_AXIS_SENS 1

/*
 * Sample telemetry.
 *
 * FLOG.print() must never be called from inside the timer ISR. HardwareSerial
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
 * ISR: FLOG.print() BLOCKS once the 64-byte TX buffer fills, for roughly
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
 * §6a RESOLVED 2026-08-19, and the isr_ticks/tk= counter removed with it.
 *
 * tk= existed to separate two failures ov= alone could not: "the ISR never ran"
 * from "the ISR ran and the print path dropped the sample". Its reading was
 * that ~12 ticks per 6 printed lines meant the publish path was losing samples,
 * and ~6 per 6 lines meant the timer itself was stalling -- in which case no
 * timing measured from these logs could be trusted.
 *
 * Measured on the bench across three captures of 60-90 s at the shipping
 * decimation: exactly 8.00 ticks per printed line, min 8, max 8, zero spread,
 * with ov= advancing 0 and a sample period of 39.99-40.01 ms. Neither failure
 * mode. The ISR fires reliably and loop() drains every sample it publishes.
 *
 * The counter was marked "diagnostic only, remove once §6a is resolved", so it
 * is gone. That is 60 bytes of flash, which is what pays for bz= and cp= on the
 * sample line. ov= remains, and is still both the liveness indicator and the
 * overrun counter -- see the LOG_DECIMATE_N block for why it must be watched
 * before any change to log density.
 *
 * Eng_Notes/falcon_state_of_project_2026-08-18.md §7.
 */

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

/*
 * ARMING DIAGNOSTIC -- 2026-08-12, for the arming redesign.
 *
 * A single-floor terminal approach left BOTH detectors switched off for the
 * whole run because arr_armed never armed: the run went departure ramp
 * straight into deceleration ramp with no cruise, so five CONSECUTIVE quiet
 * samples never occurred. Estimated from the decimated log, that run had
 * 4.0-4.9 unblanked in-band samples against the 5 required -- it missed by
 * about one sample -- but that is arithmetic over two decimated points
 * assuming a linear sweep, not a measurement.
 *
 * The measurement cannot be reconstructed offline: the transition falls
 * between the departure burst's end and the arrival burst's start, so no
 * capture on file contains it at full rate.
 *
 * So record the high-water mark of the quiet counter for the run, and where
 * it happened. Printed at the arm (or, if arming never happens, alongside
 * the release) as:
 *
 *     ARM q_hi=4 t=63140 armed=0
 *
 * q_hi < ARRIVAL_ARM_SAMPLES on a run that stopped normally is the defect,
 * measured. How far below tells us whether the gate is marginal (4) or
 * structurally wrong (0-1), and that decides whether the redesign can lean
 * on a shorter quiet run at all or must key on the departure's sign
 * reversal instead. Two bytes of RAM, no behaviour change.
 */
static volatile uint8_t  arr_quiet_hi  = 0;
static volatile bool     arr_q_frozen  = false;


/*
 * ARMING REDESIGN -- second arming path, 2026-08-12.
 *
 * THE DEFECT: arm-on-quiet needs ARRIVAL_ARM_SAMPLES *consecutive* samples
 * inside ARRIVAL_QUIET_MSS. A run with cruise provides them; a short run goes
 * departure ramp straight into deceleration ramp and only SWEEPS through the
 * quiet band. One such run alarmed for 85 s over a car that was stationary
 * for 78 of them, with BOTH detectors switched off throughout -- pk reported
 * 0.00 across a real +0.64 m/s^2 deceleration.
 *
 * ⚠️ THE FIRST q= DIAGNOSTIC COULD NOT MEASURE THE MARGIN, and a claim built
 * on it was withdrawn. It stopped counting the moment arr_armed went true, so
 * q_hi was CAPPED at ARRIVAL_ARM_SAMPLES by construction and q=5 meant only
 * "it armed". Four runs reading 5/5 were read as "the gate delivers exactly
 * the minimum" -- an artefact, not a measurement.
 *
 * FIXED 2026-08-12: counting now continues past arming and freezes when the
 * stretch that armed it BREAKS, so q= is the true length of that stretch.
 *
 *     q=5  armed on the last possible sample -- no margin
 *     q=12 the quiet lasted 12 samples where 5 were needed -- 2.4x
 *     q<5  never armed (the defect), unchanged
 *
 * That is the number that says whether short runs are genuinely marginal or
 * whether the one failure was something rarer.
 *
 * The second path below does not rest on that claim: it exists because a run
 * went COMPLETELY unarmed, which q<5 does report faithfully.
 *
 * THE SECOND PATH: arm on ARM_REV_SAMPLES consecutive samples whose sign is
 * OPPOSITE to the departure's. Physics guarantees the sign: to stop, a car
 * must shed exactly the velocity it gained, so its deceleration is opposite
 * to its departure. Unlike the quiet test this does not need the signal to
 * LINGER anywhere -- the deceleration itself supplies the samples, and it
 * lasts 2.8 s.
 *
 * Sign is usable HERE and not in the ramp block because this compares against
 * the departure measured on THIS run, rather than needing sign to mean
 * something absolute. arr_dep_sign is fixed from the first ARM_DEP_SIGN_N
 * samples after the latch, while the departure ramp is still running.
 *
 * ── WHY 8, AND WHY NOT THE OBVIOUS RULES ─────────────────────────────────
 *
 * Replayed against all 100 bursts on file (51 departure, 49 arrival):
 *
 *   rule                       false arrivals on 51 dep bursts
 *   2 consecutive opposite     12   <- the obvious rule, CATASTROPHIC
 *   3 consecutive opposite      8
 *   4-6 consecutive opposite    3
 *   7 consecutive opposite      1
 *   8 consecutive opposite      0   <- here
 *
 * The obvious formulation -- "arm as soon as the sign reverses" -- would have
 * released the beacon on a moving car in 12 of 51 departures, because the
 * departure IMPULSE contains large opposite-signed content before the ramp
 * settles. Reasoning alone would have shipped it; the replay caught it.
 *
 * 8 is bracketed on both sides by measurement, not chosen for roundness:
 * 7 still produces a false arrival, and 10 stops covering one of the eight
 * automatic arrivals. All 8 automatic (drive-ramp) arrivals carry >= 8
 * consecutive one-signed samples, median 70.
 *
 * ⬜ WHAT IS PROVEN AND WHAT IS NOT. Proven: this path is SAFE -- zero false
 * peaks and zero false ramps across 51 departure bursts. NOT proven: that it
 * RESCUES the failing case, because no burst on file contains a short run's
 * departure-to-deceleration transition at full rate (the departure burst ends
 * before the deceleration starts, which is the same blind spot that hid the
 * defect). The mechanism is measured -- every automatic deceleration supplies
 * >= 8 one-signed samples -- but the end-to-end rescue is inference until a
 * capture spans it.
 *
 * ── PROMOTED TO GATE THE PEAK COLLECTOR TOO, 2026-08-12 ───────────────────
 *
 * It first shipped gating the ramp detector only, on the reasoning that its
 * benefit was inference and it should sit behind two safety layers. That was
 * the right call at the time and the WRONG conclusion an hour later:
 *
 *   MOUNTING MODULATES THE ARMING MARGIN, AND IT REACHES ZERO. Confirmed by
 *   direct test -- same shaft, floors, firmware and slowdown profile, only
 *   the unit's mounting changed (x 0.643 -> 0.710, XY-Still +19%):
 *
 *       2->1 down    9, 8, 9   ->   6, 5, 5
 *       1->2 up      8, 8      ->   10, 11
 *
 *   q=5 is arming on the LAST POSSIBLE SAMPLE, twice in three descents. One
 *   more sample lost to buzzer phase and NEITHER detector runs through the
 *   stop -- which is exactly the 85 s false beacon that started this.
 *
 * So the quiet gate has nothing left on an unfavourable mounting, and this
 * path is the only thing that keeps the PEAK collector -- the production
 * release path -- alive through that stop. Gating the ramp detector alone
 * protected a detector that is itself unarmed, i.e. it protected nothing
 * that ships.
 *
 * Safety basis for the promotion, re-verified against the full corpus at the
 * exact shipping constants: ZERO false peaks and ZERO false ramps across 51
 * departure bursts, with and without the union.
 *
 * ⚠️ STILL UNEXERCISED. v=2 has never fired in ~20 live runs, because arming
 * kept succeeding by a single sample every time. This path is now known to be
 * NEEDED and has never been seen to work on hardware; the first occasion it
 * matters will be the first occasion it runs. Watch ARM lines on the first
 * session: v=2 on a short run is the path working, v=2 during a departure is
 * the failure the replay says cannot happen -- disarm on sight if it appears.
 */
#define ARM_REV_SAMPLES    8    /* consecutive opposite-signed samples       */
#define ARM_DEP_SIGN_N     25   /* samples used to fix the departure's sign  */

static volatile int32_t  arr_dep_acc  = 0;
static volatile uint8_t  arr_dep_n    = 0;
static volatile int8_t   arr_dep_sign = 0;   /* 0 = not yet determined       */
static volatile uint8_t  arr_opp      = 0;   /* consecutive opposite samples */
static volatile uint8_t  arm_via      = 0;   /* 1 = quiet, 2 = reversal      */
static volatile bool     arr_armed     = false;

/*
 * ─── THE RAMP DETECTOR HAS ITS OWN GATE, AND IT IS NOT arr_armed ─────────────
 *
 * Found by graph/arming_replay.py on 2026-08-12 against all 89 departure
 * bursts on file. THE HAZARD, in one real burst (from the departure latch):
 *
 *     -65 -38 -56 -64 -129 | -142 -170 -228 -301 -488 -496 -506 ...
 *     \___ all inside the 150 mm/s^2 quiet band ___/
 *
 * A SLOW DEPARTURE STARTS BELOW THE QUIET BAND. The quiet path arms on the
 * departure's own opening samples, and from that instant the ramp accumulator
 * is fed the departure ramp itself -- which then qualifies at mean 499,
 * directionality 100%, arithmetically indistinguishable from an arrival. With
 * RAMP_ARMED 1 that RELEASES THE LATCH SECONDS AFTER A CAR STARTS MOVING:
 * §14.4's catastrophic direction. 1-2 departures in 89 on logged data.
 *
 * Until 2026-08-12 this was latent -- unarmed, the verdict printed and nothing
 * happened. Arming it is what made it reachable.
 *
 * WHY arr_armed CANNOT BE THE GATE, and why arm_via == 2 is NOT the fix
 * (measured, not assumed): arm_via records whichever path armed FIRST, and the
 * reversal block is inside `if (!arr_armed ...)`, so once quiet arms, arr_opp
 * stops being evaluated and arm_via can never become 2. Quiet has won that race
 * in every run on file -- v=1, ~27 runs. Gating the ramp on arm_via == 2 would
 * make it permanently dead. It has to be an INDEPENDENT condition that keeps
 * tracking after the union arms.
 *
 * SO: the peak collector keeps the union (it needs to arm readily -- the thin
 * end of the shaft arms by a single sample), and the ramp gets ramp_gate, which
 * is reversal-ONLY. A sign reversal against the departure means the departure
 * ramp has ENDED, which is the thing the ramp detector actually needs to know
 * and the thing "the signal went quiet" only approximates.
 *
 * REPLAY EVIDENCE: zero departure-ramp fires at EVERY arming offset 0-6
 * samples, against 1-2 for the union, with arrival capability UNCHANGED at
 * 42/88 paired arrival bursts. The union's safety depended on arming starting
 * >=200 ms after the latch; MOVEMENT_DETECTION_TIMEOUT_MS is exactly 200, but
 * that is a CEILING, not a floor -- STATE_MOVEMENT_DETECTED exits early
 * whenever |w| > |vel_departure|, and vel_departure logs as 0.000 on every
 * run. This gate does not depend on that coincidence.
 *
 * COST: the ramp now needs ARM_REV_SAMPLES + 3 blocks = 44 samples (~1.76 s) of
 * deceleration rather than 36, because it cannot start counting until the
 * reversal completes. Replay says that costs nothing on the bursts on file, but
 * it is the thing to watch: a very short deceleration could fall inside it.
 */
static volatile uint8_t  ramp_opp      = 0;  /* independent reversal counter  */
static volatile bool     ramp_gate     = false;

/*
 * ⚠️ ro= MUST KEEP COUNTING AFTER THE GATE OPENS, or it measures nothing.
 *
 * The first version of this counter lived inside `if (!ramp_gate ...)` and so
 * stopped the instant the gate opened -- capped at ARM_REV_SAMPLES by
 * construction, reading ro=8 on every run where g=1 and meaning only "it
 * armed". That is EXACTLY the defect already found and fixed in the q=
 * instrument (2c3546a), where a capped counter made "q=5 every time" mean
 * nothing and cost a withdrawn margin claim. It was reintroduced here on
 * 2026-08-12 in a brand-new counter, the same afternoon, after reading that
 * note.
 *
 * So, same discipline as arr_quiet_hi/arr_q_frozen: track the high-water mark
 * and freeze it only when the reversal stretch BREAKS. ro is then the true
 * length of the stretch that opened the gate -- ro=8 means it opened on the
 * last possible sample with no margin, ro=25 means roughly 3x headroom. That
 * distinction is the whole point, because the reversal gate is new and has
 * never been exercised on a short run.
 */
static volatile uint8_t  ramp_opp_hi   = 0;
static volatile bool     ramp_o_frozen = false;

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
 * ⚠️ THE DERIVATION ABOVE IS SUPERSEDED -- corrected 2026-08-20 against the
 * labelled corpus (falcon_corpus_labelled_2026-08-20.md; labels and their
 * per-record evidence in falcon_srcs/datasets/session_g_labels.csv).
 *
 * "No overlap on either axis" was true of 12 events from one evening. Scored
 * against 63 labelled real departures and 17 labelled jogs:
 *
 *   real departures  n=63  ratio 0-58   opk 11-562, plus ONE at 972
 *   jogs             n=17  ratio 44-99  opk 381-4154
 *
 * THE REAL-DEPARTURE CEILING IS 562, NOT THE 396/440 THIS COMMENT DERIVED
 * FROM -- 42% higher, and that is the number to reason from. The populations
 * OVERLAP on 381-972; there is no gap for this gate to split. Both axes fail
 * separately (ratio overlaps 44-58), so the AND is doing the work.
 *
 * Measured error rate at these gates: 1 false JOG in 63 real departures (the
 * 972, a genuine run silenced) and 1 miss in 17 jogs (a gentle jog at 381).
 *
 * ⛔ DO NOT RAISE THIS TO ~1000 to clear the 972. It scores better on the
 * corpus as it stands and that is the trap: the labelled ceiling has moved
 * every time data was added -- 440 (08-11) -> 562 (08-12 automatic, labelled)
 * -> 972 (08-18). Fitting the gate to a sample maximum is the refuted
 * approach, not a new one. Lowering it is worse: catching the 381 jog costs
 * four silenced runs, and 381 sits inside the hand-bump population (118-389).
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

/*
 * RAMP DETECTOR -- sustained one-signed deceleration (2026-08-12).
 *
 * The fix of record for the normal-operation boundary. Four 350 fpm automatic
 * stops all released on the peak detector, but the worst margin was 1.009x
 * (peak 0.454 vs gate 0.45) with run-to-run spread 18x the margin -- the FSM
 * peak is a deviation from a rolling average, and a sustained ramp drags the
 * average with it, so the metric gets structurally weaker exactly as ramps
 * get longer. "Releases on luck" (falcon_350fpm_automatic_2026-08-11.md §1).
 *
 * What the same afternoon measured is that the signature to key on is the
 * PLATEAU, not the peak:
 *
 *   - the drive holds 0.605 +/- 0.008 m/s^2 on every ramp, nine of nine,
 *     both directions; 115 fpm measured 0.52. Speed buys DURATION, not
 *     amplitude -- so a FIXED magnitude floor is justified (§2).
 *   - directionality |Σa|/Σ|a| was 0.888-1.000 on every drive ramp and
 *     0.42 / 0.02 on brake stops. A ramp is a sustained push; a brake set
 *     is a ring that cancels itself (falcon_signature §4e). Character, not
 *     amplitude -- brake stops have the LARGER peaks.
 *
 * MECHANISM. Consecutive non-overlapping blocks of RAMP_BLOCK_N samples of
 * signed (a - zero), integer mmss like the burst. A block QUALIFIES when
 *
 *     |Σa|  >=  RAMP_FLOOR_MMSS * RAMP_BLOCK_N     (mean one-signed >= floor)
 *     |Σa| * 100  >=  RAMP_DIR_PCT * Σ|a|          (directionality)
 *
 * and RAMP_BLOCKS consecutive qualifying blocks OF THE SAME SIGN latch
 * ramp_hit, sticky until arrival_peak_reset() -- same discipline as arr_hit.
 *
 * 🔴 WHY IT CANNOT FIRE ON THE DEPARTURE RAMP -- AND WHY THAT GATE IS
 * LOAD-BEARING. Accumulation is gated on arr_armed, which requires
 * ARRIVAL_ARM_SAMPLES of quiet below ARRIVAL_QUIET_MSS; a departure ramp is
 * never quiet, so counting cannot begin until the departure is over and
 * cruise has been observed. MIN_TRAVEL_MS gates the FSM side as well.
 *
 * THIS IS NOT BELT-AND-BRACES. Replayed against the 2026-08-12 cab captures,
 * ALL FOUR DEPARTURE bursts satisfy the block test as readily as the
 * arrivals do -- a 300 fpm departure sustains ~0.5 m/s^2 one-signed for the
 * whole 3.2 s window. The block arithmetic CANNOT tell a departure ramp from
 * an arrival ramp: they are the same shape with opposite sign, and sign
 * cannot be used because it encodes direction of travel, not phase of the
 * run. The arming gate is the ONLY thing standing between this detector and
 * releasing the beacon seconds after the latch, on a moving car -- §14.4's
 * catastrophic direction. Do not weaken it, do not "simplify" it away, and
 * re-run that replay after any change to arming.
 *
 * Replay coverage for RAMP_FLOOR_MMSS 300 (2026-08-12): 24 cartop bursts --
 * brake stops, jogs, slow and 350 fpm departures -- fire at neither 400 nor
 * 300. The floor change opens no new false positive on any measured data.
 *
 * WHY CRUISE CANNOT FIRE IT: cruise WINDOWED PEAKS measured 0.07-0.28 on the
 * cartop and 0.02-0.04 in the cab (block MEANS far lower), against a floor on
 * the block MEAN -- and cruise vibration is symmetric, so the directionality
 * gate starves it from both sides.
 *
 * ── FLOOR 400 -> 300, measured 2026-08-12 ────────────────────────────────
 *
 * 400 was set against 08-11's plateau of 0.605 +/- 0.008 (n=9), claiming 1.5x.
 * Four automatic cab stops on 08-12 measured the plateau at 487-501 -- tight
 * within the session, but 19% BELOW the other session:
 *
 *   2026-08-11, 350 fpm cab      0.605 +/- 0.008   n=9
 *   2026-08-12, 300 fpm cab      0.487-0.501       n=4
 *
 * So "the drive holds a constant deceleration" is true PER CONFIGURATION,
 * not universally, and 400 left only 1.23x on the newer figure. An
 * installation 20% gentler than 08-12 would fail the floor outright and lose
 * the detector SILENTLY -- the failure mode this project keeps being bitten
 * by. 300 restores 1.63x against the measured plateau and costs nothing:
 * DIRECTIONALITY is what discriminates (100% on every drive ramp measured
 * against 0.02-0.42 for brake stops), and no cruise block mean comes near
 * either gate.
 *
 * The 08-12 single-floor run is why a fixed floor is defensible at all: a
 * stop that never reached top speed still held 0.501 for 2.6 s, so the drive
 * sheds whatever speed it has at its own rate.
 *
 * TIME TO FIRE: 3 blocks x 12 samples = 36 samples =~ 1.44 s of observed
 * plateau. A 350 fpm stop sustains ~2.9 s of ramp, so the verdict lands
 * mid-deceleration; a 115 fpm levelled stop sustains ~0.9 s and stays with
 * the peak detector -- exactly the crossover the architecture table in
 * falcon_signature §4e.3 describes. The buzzer blanks samples while sounding,
 * which stretches the 36 samples over more wall clock but does not change
 * the count; the plateau outlives it at speed.
 *
 * ✅ ARMED 2026-08-12 (RAMP_ARMED 1) on Dave's instruction. The unarmed
 * exposure protocol §3.1 asks for is satisfied: 17/17 automatic drive stops
 * latched (means 472-513, directionality 100% on every one), against complete
 * negative evidence -- zero RAMP lines across a full cartop session of brake
 * stops, jogs, cruise and repositioning moves, and zero false ramps across 51
 * replayed departure bursts. A ramp hit now sets arrival_seen and releases the
 * latch; it remains the arrival-burst trigger either way.
 *
 * ⚠️ WHAT THIS DOES NOT FIX, and the rollback. The detector is gated on
 * arr_armed, which is the only thing keeping it off the departure ramp (see
 * the ALL FOUR DEPARTURE bursts paragraph above) -- so arming buys nothing on
 * a run where arming itself fails, which is exactly the single-floor terminal
 * approach that produced the 78 s false beacon. It covers the automatic stop
 * whose peak margin is 1.009x, not the blind spot. If a RAMP release ever
 * silences a moving car: RAMP_ARMED 0 first, diagnose after.
 * RAMP_ARMED lives in movement_service.h because the FSM is what acts on it.
 */
#define RAMP_BLOCK_N      12    /* samples per block, 0.48 s at 25 Hz       */
#define RAMP_BLOCKS       3     /* consecutive qualifying blocks, same sign */
#define RAMP_FLOOR_MMSS   300   /* block mean |Σa|/N floor, milli-m/s^2     */
#define RAMP_DIR_PCT      85    /* directionality floor, percent            */

static volatile int32_t  ramp_sum   = 0;    /* Σa over the open block       */
static volatile int32_t  ramp_abs   = 0;    /* Σ|a| over the open block     */
static volatile uint8_t  ramp_n     = 0;    /* samples in the open block    */
static volatile uint8_t  ramp_run   = 0;    /* consecutive qualifying blocks */
static volatile int8_t   ramp_sign  = 0;    /* sign of the qualifying run   */
static volatile bool     ramp_hit_v = false;
static volatile uint16_t ramp_mean_mmss = 0;   /* last block, for the log   */
static volatile uint8_t  ramp_dir_pct   = 0;   /* last block, for the log   */

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
    arr_quiet       = 0;
    arr_quiet_hi    = 0;
    arr_q_frozen    = false;
    arr_armed     = false;
    arr_hit       = false;

    arr_dep_acc  = 0;
    arr_dep_n    = 0;
    arr_dep_sign = 0;
    arr_opp      = 0;
    arm_via      = 0;

    ramp_opp      = 0;   /* the ramp's own reversal gate, per run */
    ramp_gate     = false;
    ramp_opp_hi   = 0;
    ramp_o_frozen = false;

    ramp_sum   = 0;
    ramp_abs   = 0;
    ramp_n     = 0;
    ramp_run   = 0;
    ramp_sign  = 0;
    ramp_hit_v = false;
    interrupts();
}

/* True once RAMP_BLOCKS consecutive qualifying blocks were seen this run. */
bool ramp_hit()
{
    bool v;
    noInterrupts();
    v = ramp_hit_v;
    interrupts();
    return v;
}

uint16_t ramp_mean_get() { return ramp_mean_mmss; }
uint8_t  ramp_dir_get()  { return ramp_dir_pct;  }

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
 * Print one line per N published samples.
 *
 * HISTORY. 1 -> 8 alongside the 3.13 Hz -> 25 Hz timer change, to hold the
 * serial load where it already was. A sample line was ~145 characters, which at
 * 9600 baud took ~151 ms to send; one line per sample was 47% of the budget at
 * 3.13 Hz and 3.8x more than the link could carry at 25 Hz. 8 gave 3.1 lines/s,
 * the same 47%.
 *
 * ⚠️ THE WARNING THAT USED TO SIT HERE IS RETIRED, 2026-08-19. It stated ~25%
 * sample loss, called that NOT tolerable for shipping, told the reader not to
 * trust any threshold derived at this rate, and prescribed a sample ring as the
 * proper fix. All of it predated two changes that landed within days: the ring
 * itself (2026-08-11, see the SAMPLE RING block above) and 9600 -> 62500 baud
 * (2026-08-13, see the SERIAL BAUD block). Neither retired the warning, so the
 * source went on asserting that every constant in this firmware rested on 80%
 * of the data long after that stopped being true.
 *
 * MEASURED AT N=8. Bench at rest, three captures of 60-90 s, 2026-08-19:
 *
 *     tk= per printed line   8.00, min 8, max 8, zero spread
 *     ov= advance            0 over 90 s
 *     sample period          39.99 - 40.01 ms  (25.00 Hz)
 *     sample line            114-124 chars -> 18.3-19.8 ms at 62500 baud
 *     serial duty            5.7 - 6.2% of the link
 *
 * Exactly 8 ISR ticks per printed line with no spread means loop() drains every
 * sample the ISR publishes: there is no sample loss at the shipping decimation.
 * The same captures answer the Eng_Notes §6a question isr_ticks/tk= was added
 * for -- neither failure mode it was meant to distinguish is present -- so tk=
 * is now removable, which is a small credit against the flash headroom item.
 *
 * ⛔ DO NOT LOWER THIS. Tested at N=2 on the bench, 2026-08-19:
 *
 *     ov= advance            55 over 90 s
 *     tk= per printed line   3.08, min 2, max 4, against an expected exactly
 *                            2.00 -- ~35% of samples never reach the consumer
 *     apparent rate          20.79 Hz, against a true 25
 *     watchdog               repeated Reset cause: 0x8, with full re-boot and
 *                            re-calibration. The device boot-loops.
 *
 * AND NOT FOR THE REASON THIS BLOCK USED TO GIVE. It is not a serial budget
 * problem: N=2 fails at 11.8% duty, with the link 88% idle. The ring is drained
 * ONE SAMPLE PER loop() PASS -- the drain returns after processing a single
 * record -- so its eight slots buy latency tolerance, not throughput, and log
 * density throttles the consumer directly. At N=2 every second pass carries a
 * ~17.5 ms blocking print, the mean loop period crosses the 40 ms sample
 * interval, and the ring backs up permanently rather than transiently. Depth
 * cannot rescue a consumer that is slower than the producer. wdt_reset() lives
 * in that same starved loop, which is why the watchdog fires.
 *
 * The old advice to "raise this when the timer is fixed to 100 Hz, or serial
 * becomes the new bottleneck" therefore had the constraint backwards. Before
 * any increase in log density, make the drain consume the whole ring per pass;
 * then re-measure before trusting what the denser log says. ov= is the counter
 * that settles it. tk= was the other half of that measurement and was removed
 * on the same day (see the §6a block above); reinstate it temporarily -- it is
 * ~60 bytes -- rather than judging a density change on ov= alone, because ov=
 * counts overruns and tk= is what proves the ISR was firing underneath them.
 *
 * Full pair in Eng_Notes/falcon_test_plan_2026-08-18.md §1.2.
 */
#define LOG_DECIMATE_N  8

/*
 * ─── BATTERY THRESHOLDS, DERIVED AND VALIDATED 2026-08-14 ────────────────────
 *
 * ✅ THE WHOLE MEASUREMENT CHAIN IS NOW TRUSTWORTHY END TO END, validated
 * against a meter:
 *
 *     meter at the pack     5.010 V
 *     firmware pack_mv      5012
 *
 * 2 mV apart on 5 V -- better than one LSB (1 LSB = 3100/1023 = 3.03 mV at the
 * node, 6.06 mV of pack), so as close as a 10-bit ADC can resolve. That single
 * comparison validates the divider ratio, the 3100 mV reference assumption and
 * the ADC configuration simultaneously, because an error in any one of them
 * would show up here.
 *
 * It also confirms the prescaler fix did real work rather than just satisfying
 * the datasheet. At /128 the same chain read 4860 against a 4900 meter -- 0.8%
 * LOW, which is exactly the direction a sample-and-hold that has not finished
 * charging from a 250 kOhm source will err. See configure_adc_channel().
 *
 * ⭐ SO THE CHIRP CAN NOW BE PRESENTED AS A CALIBRATED WARNING. The earlier
 * prohibition in this block is lifted; it existed because nobody could say what
 * the logged number meant in volts, and now anybody can.
 *
 * The divider is 2:1, metered on the bench (499k/499k, sheet 3).
 * read_adc_pc2_voltage() returns the NODE voltage in millivolts against the
 * 3100 mV rail, so a pack figure divides by two to become a threshold. Stated
 * in PACK millivolts below and halved at compile time, so the next reader
 * cannot repeat the mistake this replaces.
 *
 * ⭐ THE HISTORICAL NUMBERS WERE RIGHT ALL ALONG. 1600 and 1750 at 2:1 are
 * exactly 3.2 V and 3.5 V, and Release.txt has claimed a 3.2 V trip since V1.2.
 * What was wrong was BATT_R1/BATT_R2 in main.h -- a V1 leftover giving 4.4,
 * which made 1600 look like an implausible 7.04 V and sent three separate
 * investigations looking for a defect that was not there. **This change moves
 * no threshold.** It only makes the derivation explicit.
 *
 * CLEAR sits above LOW for hysteresis: with a single threshold a pack near the
 * boundary would chatter the alarm on and off every measurement cycle. 300 mV
 * of pack is comfortably wider than the sample-to-sample spread on the bench
 * (833 counts +/- 4, about 25 mV of pack).
 *
 * ⚠️ THE READ IS RATIOMETRIC AGAINST AVCC, AND THAT HAS A CONSEQUENCE NOBODY
 * HAS WRITTEN DOWN. The reference is the 3.1 V rail, the input is pack/2. Above
 * the regulator's dropout the rail is fixed and the reading tracks the pack
 * honestly. BELOW dropout the rail follows the pack down, input and reference
 * fall together, and the count PINS at about 511 -- roughly 1548 mV in these
 * units -- no matter how flat the pack gets.
 *
 * That happens to be just under BATTERY_LOW_THRESHOLD, so the alarm does trip
 * and does stay tripped. But the reading SATURATES: below dropout a 3.2 V pack
 * and a 2.2 V pack are indistinguishable. Do not build anything that needs to
 * measure how flat a flat battery is.
 *
 * ─── ✅ WARNING LEAD TIME: 3.2 V -> 3.6 V, Dave's call, 2026-08-14 ───────────
 *
 * U1 is a TPS628438 buck holding 3.1 V from the pack, and it cannot hold that
 * once the pack approaches ~3.2 V. The pack is 3 x AAA -- 5.01 V fresh, metered
 * -- and MEASURED STABLE THROUGH AN ALERT, so there is no load sag and the
 * quiet reading and the in-alert reading are the same number.
 *
 * That made the old value indefensible once it could be read in volts: a trip
 * at 3.2 V fires essentially AT dropout, so the mechanic's first warning would
 * arrive as the device was about to stop working. A warning with no runway is
 * not a warning.
 *
 * 3.6 V leaves ~400 mV above dropout. CLEAR moves with it, keeping the original
 * 300 mV of hysteresis -- with a single threshold a pack sitting on the
 * boundary would chatter the alarm on and off every measurement cycle.
 *
 * ⚠️ RAISING LOW WITHOUT RAISING CLEAR INVERTS THE HYSTERESIS, which is a silent
 * and nasty failure. The static_assert below catches it, and it was confirmed
 * firing on exactly that mistake before this change was committed.
 *
 * ⬜ HOW MUCH RUNTIME 400 mV BUYS IS UNMEASURED. Alkaline discharge is steep at
 * the end, so 3.6 -> 3.2 V could be a short stretch. The honest claim is "some
 * warning instead of none". Measuring it needs a pack run down under a
 * representative duty cycle, which nobody has done.
 *
 * ⬜ NOT A NUISANCE THRESHOLD, at least: 3.6 V is 1.2 V/cell, deep into the
 * discharge curve for AAA alkaline, against 1.67 V/cell fresh.
 *
 * ✅ THE PRESCALER IS FIXED (/128 -> /16, 62.5 kHz, in spec) and these numbers
 * were re-taken afterwards against the meter, which is the reading quoted at
 * the top of this block. Nothing here is left standing on the old setting.
 */
#define VBATT_LOW_MV              3600    /* pack millivolts -- warn with runway */
#define VBATT_CLEAR_MV            3900    /* pack millivolts -- hysteresis       */

#define BATTERY_LOW_THRESHOLD     (VBATT_LOW_MV   / VBATT_DIVIDER)   /* 1600 */
#define BATTERY_CLEAR_THRESHOLD   (VBATT_CLEAR_MV / VBATT_DIVIDER)   /* 1750 */

static_assert(BATTERY_CLEAR_THRESHOLD > BATTERY_LOW_THRESHOLD,
              "battery clear threshold must sit above the trip, or the alarm "
              "chatters on every measurement cycle");

/*
 * Battery readings to discard after boot before the alarm logic is armed, on
 * top of the averaging. See the comment in check_for_battery_voltage().
 */
#define BATTERY_SETTLE_SAMPLES    2

/*
 * ─── BATTERY SAMPLE CADENCE, 2026-08-14 ──────────────────────────────────────
 *
 * WHAT WAS THERE. `adc_loop_counter == 30000` to enable the divider and
 * `== 30100` to read it -- LOOP PASSES, not milliseconds. Nobody knew the
 * period, because it depended on how long loop() happened to take, and loop()
 * at 1 MHz is dominated by an 11.4 ms sensor read. Measured effect: battery
 * telemetry took ~23 MINUTES of uptime to converge (§6.1 of the product
 * document, which documents it as a bench workaround: "discard the first
 * Voltage value after boot").
 *
 * WHY THAT KILLED THE FEATURE. A maintenance visit is minutes long. An alert
 * that cannot arm inside ~20 minutes will never fire during the job it exists
 * to protect -- so the low-battery warning was, in practice, dead code on a
 * device whose worst failure is silence while moving. This is the single
 * highest-value part of the 2026-08-14 battery work; the chirp pattern is
 * cosmetic next to it.
 *
 * NOW. A wall-clock period. 30 s x 8-deep RollingAvg = a 4-minute window, and
 * with BATTERY_SETTLE_SAMPLES the alarm can arm ~90 s after boot instead of
 * ~23 minutes.
 *
 * SETTLE is the gap between enabling the divider and reading it. The old code
 * gave it 100 loop passes, which was never measured either. 10 ms is
 * comfortably more than that and still nothing against a 30 s period; it is a
 * guess, but a guess in the generous direction, and it costs nothing to be
 * wrong high. UNMEASURED -- if the reading looks low, suspect this first.
 */
#define BATTERY_SAMPLE_PERIOD_MS  30000
#define BATTERY_SETTLE_MS         10

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
void emit_ramp_log();
void emit_arm_log();

/*
 * read_battery_voltage() is gone (2026-08-14). It read the EXTERNAL MCP3208 --
 * whose adc.begin() is commented out in configure_adc_channel(), so it was
 * sampling an uninitialised part -- and it was the only user of VBATT_CONST,
 * the V1 divider constant that turned out to describe the wrong board. Its last
 * caller went with the initialization() cleanup earlier the same day.
 */
extern int configure_adc_channel();
extern uint16_t read_adc_pc2_voltage();
extern uint16_t read_adc_pc2();
#if defined(BROWNOUT_TEST) || defined(BATTERY_BENCH)
/* Bench-only VCC measurement via the 1.1 V bandgap -- see adc.cpp. */
extern uint16_t read_vcc_mv();
#endif
#if defined(BATTERY_BENCH)
/* Bench-only battery characterisation -- defined below loop(). */
static void bench_battery_service();
#endif
#if defined(IDLE_CURRENT_TEST)
/* Bench-only stepped current measurement -- defined below loop(). */
static void idle_current_service();
static bool ic_log_enabled = true;
#endif
extern bool get_buzzer_status();

/*
 * ─── SERIAL BAUD: 9600 -> 62500, 2026-08-13 ──────────────────────────────────
 *
 * THIS IS A DETECTION FIX, NOT A CONVENIENCE. FLOG.print blocks loop() once
 * the 64-byte TX buffer fills, and loop() is what drains the sample ring and
 * steps the alarm. At alert onset the firmware emits 732 characters -- four FSM
 * transition lines, a 347-character departure burst dump, and the jog verdict.
 * At 9600 baud that is ~760 ms of blocked loop(), which is
 *
 *   - the visible cause of the chase stutter Dave reported over the first 2-3
 *     sequences of an alert (2026-08-13), and
 *   - the same mechanism behind the ~20% snapshot overrun that every threshold
 *     in Eng_Notes currently stands on 80% of.
 *
 * WHY 62500 AND NOT 115200. At F_CPU = 1 MHz the USART in U2X mode can only
 * produce 1000000 / (8 x (UBRR+1)):
 *
 *   UBRR 12 ->  9615   (+0.16%)   <- the old rate
 *   UBRR  3 -> 31250   (exact)    <- conservative fallback
 *   UBRR  1 -> 62500   (exact)    <- this
 *   UBRR  0 -> 125000  (exact, but UBRR=0 is not reliable)
 *
 * 115200 IS UNREACHABLE AT THIS CLOCK -- the nearest divisor is 125000, an 8.5%
 * error, which is exactly why the 2026-07-15 EFT attempt at 115200 produced a
 * blank terminal. That failure was recorded as unexplained; it was arithmetic.
 *
 * 62500 cuts the 760 ms burst to ~117 ms, a 6.5x reduction, for zero flash.
 *
 * ⚠️ monitor_speed in platformio.ini MUST match. If a terminal or USB-serial
 * adapter garbles at 62500, drop to 31250 -- also exact, still 3.25x better.
 * Both are non-standard rates; pyserial and CP210x handle them, but a different
 * adapter might not.
 */
#define SERIAL_BAUD_RATE    62500L

void setup() {

    /*
     * Configure the debug serial port here
    */
    FLOG_BEGIN(SERIAL_BAUD_RATE);

    FLOG.print(F("\r\n\nDevice Booted \r\n"));

    /*
     * Reset cause, captured by the .init3 hook. 0x8 (WDRF) = the watchdog
     * caught a lockup last boot; 0x1 PORF, 0x2 EXTRF, 0x4 BORF.
     */
    FLOG.print(F("Reset cause: 0x"));
    FLOG.print(boot_mcusr, HEX);
    FLOG.print(F("\r\n"));
    /*
     * Configure the ADC chip here for SPI protocol.
    */
    SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
    SPI.begin();
    SPI.beginTransaction(settings);
    FLOG.print(F("Configured SPI interface \r\n"));

    /*
     * Configure all Alarm Ports here
    */
    setup_alarm();
    disable_alarm();

    FLOG.print(F("Configured Alarms \r\n"));

    digitalWrite(PIN_GREEN_LED, HIGH);
    init_time_g = millis();

    bma456.initialize(RANGE_2G, ODR_100_HZ, NORMAL_AVG4, CONTINUOUS);
    FLOG.print(F("Configured BMA456 \r\n"));

    /* (TWI timeout call goes here when WIRE_TIMEOUT fits -- see above.) */

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
            FLOG.print(F("AnyMotion config FAILED : "));
            FLOG.print(rslt);
            FLOG.print(F("\r\n"));
        } else {
            FLOG.print(F("AnyMotion armed thr="));
            FLOG.print(ANYMOTION_THRESHOLD);
            FLOG.print(F(" dur="));
            FLOG.print(ANYMOTION_DURATION);
            FLOG.print(F("\r\n"));
        }
    }

    pinMode(PIN_ACC_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ACC_INT1), acc_int1_isr, RISING);

    /*
     * Configure the ADC channel for battery voltage measurement using PC2 (ADC2).
     * This will be used to monitor the battery voltage and trigger low battery alarm if needed. 
    */

    configure_adc_channel();

    /* Last: nothing above may run again, so arm the watchdog on the way out. */
    wdt_enable(WDTO_2S);
}

void initialization()
{

    /*
     * One device boot-up, read the acceleration data  for
     * some time.
     */

    /*
     * ⛔ REMOVED 2026-08-14: battery_avg.add(read_battery_voltage()).
     *
     * TWO DIFFERENT ADCs WERE WRITING THE SAME AVERAGE. read_battery_voltage()
     * reads the EXTERNAL MCP3208 over SPI, whose adc.begin() is commented out
     * in configure_adc_channel() -- so it was sampling an uninitialised part and
     * pushing the result into the same battery_avg that check_for_battery_voltage()
     * fills from the INTERNAL ADC on PC2.
     *
     * It was harmless only by accident: INIT_TIME_MS is 80 ms so it contributed
     * about four samples, and the settle path later calls battery_avg.fill(),
     * which overwrites all of them. Leaving two ADC paths feeding one average
     * while the trip point is being characterised is how a measurement bug
     * survives a bench session, so it goes now.
     */

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

        FLOG.print(F("Device initialized completely \r\n"));
        digitalWrite(PIN_PIEZO, LOW);
        digitalWrite(PIN_GREEN_LED, LOW);
        digitalWrite(PIN_CHASE_LED, HIGH);
        digitalWrite(PIN_CHASE_LED, LOW);
    }
    return;
}


void loop()
{
    /*
     * The only watchdog kick. Deliberately nowhere else -- if loop() stops
     * being reached, whatever stopped it is exactly what the reboot exists
     * to recover from.
     */
    wdt_reset();

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

#if defined(BROWNOUT_TEST)
        /*
         * ─── BENCH ONLY: sustained-alarm endurance test ──────────────────────
         *
         * ⛔ NEVER SHIP. NEVER PUT IN A CAR. The FSM is bypassed entirely, so
         * NOTHING can release the beacon -- no arrival path, no jog verdict, no
         * failsafe. That is the point: this build tests the POWER and TWI
         * behaviour of a long continuous alarm, and it must not be able to end
         * the alarm by way of the logic under test. Built only by
         * [env:brownout_test]; the shipping env cannot define BROWNOUT_TEST.
         *
         * WHAT IT DISCRIMINATES. On 2026-08-11 the device stopped 243 s into an
         * alarm and that was attributed to the rail sagging under piezo load.
         * Dave proposed 2026-08-13 that it was the TWI wedge instead -- see
         * Eng_Notes/falcon_500fpm_ui_2026-08-13.md §9. Two independent readouts
         * separate them:
         *
         *   BRN vcc_blast=  VCC sampled WHILE the piezo is driven (peak load)
         *   BRN vcc_quiet=  VCC sampled in the silent phase
         *
         * measured against the 1.1 V bandgap, which unlike the shipping battery
         * read is NOT ratiometric with AVCC (see read_vcc_mv()).
         *
         *   blast markedly below quiet, and both falling  -> rail sag is real
         *   both flat and equal, then the device hangs     -> TWI wedge
         *   survives past 243 s with tw= climbing          -> wedge, now caught
         *   Reset cause: 0x8 mid-alarm                     -> wedge, WDT caught it
         *
         * THE SAMPLE ISR KEEPS RUNNING, so TWI exposure is identical to a real
         * alarm -- that is what makes this a valid test of the wedge. The
         * watchdog is still kicked, both by loop() above and inside the sampling
         * spin below.
         *
         * ⚠️ Because this branch `break`s, everything after it -- fsm_run() and
         * all the log emitters -- is unreachable and the linker drops it. Two
         * consequences:
         *   - the build is ~9.7 KB smaller than the shipping one, which is
         *     expected and not a sign the test is doing less;
         *   - the sample ring is never drained, so ov= climbs monotonically and
         *     is NOT comparable to a normal run. It is still useful as a
         *     LIVENESS indicator: if ov= stops advancing, the ISR has died,
         *     which is the wedge signature.
         */
        {
            static bool     brn_armed   = false;
            static uint32_t brn_start   = 0;
            static uint32_t brn_last    = 0;

            /*
             * ⚠️ THIS CALL IS THE WHOLE TEST. check_for_active_alarm() is what
             * drives PIN_PIEZO and steps the sequence, and in the shipping loop
             * it lives BELOW fsm_run() -- i.e. below this branch's `break`, so
             * it is unreachable here. Without it the alarm flag is set and
             * nothing ever drives the pin: the first flash of this build ran
             * silently and reported vcc_blast=0 on every line, measuring an
             * unloaded rail and testing nothing.
             */
            check_for_active_alarm();

            if (!brn_armed) {
                /* Let calibration finish and the pack settle, then latch on. */
                if (millis() > 15000UL) {
                    brn_armed = true;
                    brn_start = millis();
                    enable_alarm();
                    enable_chase_leds();
                    FLOG.print(F("BRN: continuous alarm armed, FSM bypassed\r\n"));
                }
            } else if ((millis() - brn_last) >= 1000UL) {
                brn_last = millis();

                /*
                 * Sample in the blast phase and the quiet phase of the SAME
                 * second. get_buzzer_status() is true while the piezo is driven
                 * plus the ringdown, so it identifies the loaded phase.
                 *
                 * The spin MUST keep stepping the alarm. Sampling blocks for up
                 * to 900 ms and a 150 ms blast occurs once per 800 ms sequence,
                 * so without stepping inside the loop the piezo would hold
                 * whatever state it was in when the spin began and the blast
                 * phase would never arrive.
                 */
                uint16_t v_blast = 0, v_quiet = 0;
                uint32_t spin = millis();
                while ((millis() - spin) < 900UL && (!v_blast || !v_quiet)) {
                    check_for_active_alarm();
                    if (get_buzzer_status()) {
                        if (!v_blast) v_blast = read_vcc_mv();
                    } else {
                        if (!v_quiet) v_quiet = read_vcc_mv();
                    }
                    wdt_reset();
                }

                FLOG.print(F("BRN t="));
                FLOG.print((millis() - brn_start) / 1000UL);
                FLOG.print(F("s vcc_blast="));
                FLOG.print(v_blast);
                FLOG.print(F(" vcc_quiet="));
                FLOG.print(v_quiet);
                FLOG.print(F(" bat="));
                FLOG.print(read_adc_pc2());
                FLOG.print(F(" tw="));
                FLOG.print(twi1_guard_trips1());
                FLOG.print(F(" ov="));
                FLOG.print(sample_overrun);
                FLOG.print(F("\r\n"));
            }

            /* FSM deliberately not run. */
            break;
        }
#endif /* BROWNOUT_TEST */

        ms.fsm_run();

        /*
         * Phase 4 of the idle-current test compiles nothing out -- it gates the
         * emitters at runtime, so the ONLY difference from phase 1 is whether
         * FLOG.print runs. That is what makes the difference between the two
         * readings attributable to logging rather than to a different build.
         *
         * poll_acc_int_status() is deliberately NOT gated: it is an I2C
         * transaction, not a log line, and stopping it would change what the
         * device is doing rather than what it is saying.
         */
#if defined(IDLE_CURRENT_TEST)
        if (ic_log_enabled)
#endif
        {
            emit_sample_log();
            emit_burst_log();
            emit_ramp_log();
            emit_arm_log();
            emit_acc_int_log();
        }
        poll_acc_int_status();

        /*
         * The bench build replaces the threshold logic outright rather than
         * running alongside it: check_for_battery_voltage() would see a healthy
         * pack and immediately call disable_battery_alarm(), cancelling the
         * forced chirp before it could be heard.
         */
#if defined(BATTERY_BENCH)
        bench_battery_service();
#else
        check_for_battery_voltage();
#endif

        check_for_active_alarm();
        check_for_battery_alarm();

        /*
         * LAST, and after both alarms deliberately. It reads alarm_status_g and
         * chase_led_status_g to decide whether the 4017 is spoken for, so it has
         * to run once those are settled for this pass -- otherwise a beat could
         * clock the counter in the same pass that the alarm sequence pulsed MR,
         * and the blast would stop landing on LED 1.
         */
        heartbeat_service();

#if defined(IDLE_CURRENT_TEST)
        idle_current_service();
#endif
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
        float   sdev = accel_value - arr_zero;
        float   dev  = fabs(sdev);
        int8_t  ssign = (sdev > 0.0f) ? 1 : ((sdev < 0.0f) ? -1 : 0);

        /*
         * Fix this run's departure sign from its opening samples, while the
         * departure ramp is still running. See the ARM_REV_SAMPLES block.
         */
        if (arr_dep_sign == 0) {
            arr_dep_acc += (int32_t)(sdev * 1000.0f);
            if (++arr_dep_n >= ARM_DEP_SIGN_N) {
                arr_dep_sign = (arr_dep_acc >= 0) ? 1 : -1;
            }
        }

        /*
         * Quiet-run tracking. Counting deliberately continues AFTER arming
         * and stops only when the stretch breaks -- see arr_q_frozen.
         */
        if (dev < ARRIVAL_QUIET_MSS) {
            ++arr_quiet;
            if (!arr_q_frozen && arr_quiet > arr_quiet_hi) {
                arr_quiet_hi = arr_quiet;
            }
            if (!arr_armed && arr_quiet >= ARRIVAL_ARM_SAMPLES) {
                arr_armed    = true;
                arm_via      = 1;              /* quiet */
                arr_bucket_ms = millis();
            }
        } else {
            arr_quiet = 0;
            if (arr_armed) {
                arr_q_frozen = true;   /* the arming stretch has ended */
            }
        }

        /*
         * Second arming path: ARM_REV_SAMPLES consecutive samples opposite in
         * sign to this run's departure. Gates the PEAK collector as well as
         * the ramp accumulator -- see the promotion note above.
         */
        if (!arr_armed && arr_dep_sign != 0) {
            if (ssign != 0 && ssign != arr_dep_sign) {
                if (++arr_opp >= ARM_REV_SAMPLES) {
                    arr_armed    = true;
                    arm_via      = 2;          /* sign reversal */
                    arr_bucket_ms = millis();
                }
            } else {
                arr_opp = 0;
            }
        }

        /*
         * The ramp detector's own gate: reversal-only, tracked INDEPENDENTLY of
         * arr_armed so that quiet arming cannot hand the departure ramp to the
         * accumulator. See the ramp_gate block above for the burst that forced
         * this and why arm_via cannot serve.
         */
        if (arr_dep_sign != 0) {
            if (ssign != 0 && ssign != arr_dep_sign) {
                ++ramp_opp;
                if (!ramp_o_frozen && ramp_opp > ramp_opp_hi) {
                    ramp_opp_hi = ramp_opp;    /* true stretch length, not 8 */
                }
                if (!ramp_gate && ramp_opp >= ARM_REV_SAMPLES) {
                    ramp_gate = true;
                }
            } else {
                ramp_opp = 0;
                if (ramp_gate) {
                    ramp_o_frozen = true;  /* the stretch that gated has ended */
                }
            }
        }

        if (arr_armed) {
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
         * Signed sample, shared by the burst recorder and the ramp detector.
         */
        {
            int16_t sv;

            if (sdev >= 32.0f)       sv =  32000;
            else if (sdev <= -32.0f) sv = -32000;
            else                     sv = (int16_t)(sdev * 1000.0f);

            /*
             * Burst recorder. Writes continuously so the pre-trigger history
             * is always there; freezes once the post-trigger count runs out,
             * and stops writing until loop() has dumped it.
             */
            if (!burst_ready) {
                burst[burst_head] = sv;
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

            /*
             * Ramp detector, gated on ramp_gate -- reversal ONLY, deliberately
             * NOT the arr_armed union the peak uses. See the ramp_gate block.
             */
            if (ramp_gate && !ramp_hit_v) {
                ramp_sum += sv;
                ramp_abs += (sv < 0) ? -(int32_t)sv : (int32_t)sv;

                if (++ramp_n >= RAMP_BLOCK_N) {
                    int32_t mag  = (ramp_sum < 0) ? -ramp_sum : ramp_sum;
                    int8_t  sign = (ramp_sum < 0) ? -1 : 1;
                    bool    qual = (mag >= (int32_t)RAMP_FLOOR_MMSS * RAMP_BLOCK_N) &&
                                   (mag * 100 >= (int32_t)RAMP_DIR_PCT * ramp_abs);

                    if (qual && (ramp_run == 0 || sign == ramp_sign)) {
                        ramp_sign = sign;
                        if (++ramp_run >= RAMP_BLOCKS) {
                            ramp_hit_v     = true;
                            ramp_mean_mmss = (uint16_t)(mag / RAMP_BLOCK_N);
                            ramp_dir_pct   = (uint8_t)((mag * 100) / ramp_abs);
                        }
                    } else {
                        ramp_run  = qual ? 1 : 0;
                        ramp_sign = qual ? sign : 0;
                    }

                    ramp_sum = 0;
                    ramp_abs = 0;
                    ramp_n   = 0;
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

    FLOG.print(F("BURST k="));
    FLOG.print(burst_kind ? F("arr") : F("dep"));
    FLOG.print(F(" pre="));
    FLOG.print(burst_kind ? (BURST_N - BURST_POST_ARR) : (BURST_N - BURST_POST_DEP));
    FLOG.print(F(" n="));
    FLOG.print(BURST_N);
    FLOG.print(F(" signed_mmss="));

    /* Oldest first: the ring is full, so the oldest entry is at head. */
    for (i = 0; i < BURST_N; i++) {
        FLOG.print(burst[(uint8_t)((head + i) % BURST_N)]);
        FLOG.print(' ');
    }
    FLOG.print(F("\r\n"));

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

        FLOG.print(F("JOGV pos="));  FLOG.print(pos);
        FLOG.print(F(" neg="));      FLOG.print(neg);
        FLOG.print(F(" ratio="));    FLOG.print(ratio_pct);
        FLOG.print(F(" opk="));      FLOG.print(sv);
        FLOG.print(F(" verdict="));
        if (ratio_pct >= JOG_OPP_RATIO_PCT && sv >= JOG_OPP_PEAK_MMSS) {
            FLOG.print(F("JOG"));
#if JOG_VERDICT_ARMED
            ms.jog_release();
#endif
        } else {
            FLOG.print(F("RUN"));
        }
#if JOG_VERDICT_ARMED
        FLOG.print(F(" (armed)\r\n"));
#else
        FLOG.print(F(" (unarmed)\r\n"));
#endif
    }

    noInterrupts();
    burst_ready = false;
    interrupts();
}

/*
 * Report the ramp verdict when it latches, from loop(), REGARDLESS OF FSM
 * STATE. Added 2026-08-12 after the first two automatic cab runs.
 *
 * The FSM's own ramp check lives inside STATE_MOVING, and on a fast
 * drive-controlled stop it never gets there: the deceleration plateau
 * (~0.5 m/s^2) is ITSELF above ARRIVAL_PEAK_VALUE (0.45), so the polled peak
 * crosses on the ramp's leading edge ~0.3 s in, the FSM leaves STATE_MOVING,
 * and the ramp verdict -- which needs RAMP_BLOCKS x RAMP_BLOCK_N samples,
 * ~1.44 s -- latches about a second later with nobody looking. Both cab runs
 * (300 fpm, both directions) qualified five blocks at 100% directionality
 * and printed nothing.
 *
 * So the detector was invisible exactly where it matters most, and its
 * arming evidence could only be reconstructed by replaying bursts offline.
 * This prints the latch as it happens. It is pure instrumentation: no
 * release decision reads it, and the FSM path is untouched.
 *
 * Edge-detected against arrival_peak_reset(), which clears ramp_hit_v on
 * entry to STATE_MOVING -- so this re-arms once per run.
 */
void emit_ramp_log()
{
    static bool last = false;
    bool now = ramp_hit();

    if (now && !last) {
        FLOG.print(F("RAMP latched mean="));
        FLOG.print(ramp_mean_get());
        FLOG.print(F(" dir="));
        FLOG.print(ramp_dir_get());
        FLOG.print(F("\r\n"));
    }
    last = now;
}

/*
 * One arming summary per run, printed when the FSM leaves STATE_MOVING --
 * so it appears whether or not arming ever happened, which is the whole
 * point. See the arr_quiet_hi block for why this cannot be measured offline.
 *
 *     ARM q_hi=4 t=63140 armed=0
 *
 * q_hi is the longest consecutive quiet run the detector achieved,
 * ARRIVAL_ARM_SAMPLES is what it needed. arrival_peak_reset() clears the
 * high-water mark on entry to STATE_MOVING, so this is per-run.
 */
void emit_arm_log()
{
    static uint8_t last_state = 0;
    uint8_t st = (uint8_t)ms.get_state();

    if (last_state == (uint8_t)MotionStates::STATE_MOVING &&
        st != (uint8_t)MotionStates::STATE_MOVING) {
        uint8_t hi, ropp;
        bool    armed, rgate;

        noInterrupts();
        hi    = arr_quiet_hi;
        armed = arr_armed;
        rgate = ramp_gate;
        ropp  = ramp_opp_hi;   /* high-water mark, NOT the live counter */
        interrupts();

        /*
         * q_hi vs ARRIVAL_ARM_SAMPLES; v = which path armed the PEAK.
         * g/ro are the RAMP's own reversal gate, which is independent of both
         * -- see the ramp_gate block. g=1 means the departure ramp was seen to
         * end, so a RAMP verdict on this run is trustworthy; g=0 means the ramp
         * detector never ran at all, whatever the peak did.
         *
         * ro is the TRUE length of the reversal stretch, counted past the gate
         * opening and frozen when the stretch breaks -- see the ramp_opp_hi
         * block for why that matters and how it was got wrong first. Read it as
         * margin against ARM_REV_SAMPLES: ro=8 opened on the last possible
         * sample, ro=24 had 3x headroom. g=0 with ro=7 is a near miss.
         */
        FLOG.print(F("ARM q="));
        FLOG.print(hi);
        FLOG.print(F(" a="));
        FLOG.print(armed ? 1 : 0);
        FLOG.print(F(" v="));
        FLOG.print(arm_via);
        FLOG.print(F(" g="));
        FLOG.print(rgate ? 1 : 0);
        FLOG.print(F(" ro="));
        FLOG.print(ropp);
        FLOG.print(F("\r\n"));
    }
    last_state = st;
}

/*
 * Print the most recently published sample. Called from loop() only.
 */
void emit_sample_log()
{
    static uint8_t  decimate = 0;
    sample_log_t    s;
    uint16_t        overrun;
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

    /*
     * Health line -- the production build's only window on itself.
     *
     * At FALCON_LOG 1 the per-sample line is gone, so ov= and the sample rate
     * go with it. Those two numbers are exactly what session A's verification
     * clause has to compare between a logging build and a shipping one: the
     * blocking print is the sole cause of ring overrun, so a quieter build
     * feeds its detectors MORE samples than the population every threshold on
     * file was measured on. n= is that measurement, in samples actually
     * published over the interval.
     *
     * One line per HEALTH_PERIOD_MS. Cheap enough to leave on in service and
     * far too sparse to throttle the consumer the way the per-sample line does.
     */
#if FALCON_LOG == 1
    {
        static uint32_t health_last_ms = 0;
        static uint16_t health_n       = 0;

        health_n++;
        if ((uint32_t)(s.t_ms - health_last_ms) >= HEALTH_PERIOD_MS) {
            uint8_t tw = twi1_guard_trips1();
            FLOG.print(F("HEALTH t="));
            FLOG.print(s.t_ms);
            FLOG.print(F(" n="));
            FLOG.print(health_n);
            FLOG.print(F(" ov="));
            FLOG.print(overrun);
            FLOG.print(F(" er="));
            FLOG.print(err_total);
            FLOG.print(F(" st="));
            FLOG.print(s.fsm_state);
            if (tw) {
                FLOG.print(F(" tw="));
                FLOG.print(tw);
            }
            FLOG.print(F("\r\n"));
            health_last_ms = s.t_ms;
            health_n       = 0;
        }
    }
#endif


    /*
     * The periodic sample line: 99% of the log by bytes, and a BLOCKING print
     * in the same loop() pass that drains one sample from the ring. Compiled
     * out below FALCON_LOG 2 -- see falcon_log.h for why that is not a
     * print-only change. Everything above this point is detector input
     * (vel_window, lat_monitor) and is never compiled out.
     */
#if FALCON_LOG >= 2

    if (++decimate < LOG_DECIMATE_N) {
        return;
    }
    decimate = 0;

    FLOG.print(F("t="));
    FLOG.print(s.t_ms);

    if (s.err_run) {
        /*
         * Failed read. Print the fault rather than a stale or fabricated
         * value, so a dead sensor is visible in the log instead of looking
         * like a stationary car.
         */
        FLOG.print(F(" a=ERR er="));
        FLOG.print(s.err_run);
    } else {
        FLOG.print(F(" a="));
        FLOG.print(s.accel / 1000.0f, 3);
        FLOG.print(F(" avg="));
        FLOG.print(s.avg / 1000.0f, 3);
    }

    FLOG.print(F(" st="));
    FLOG.print(s.fsm_state);
    FLOG.print(F(" rd="));
    FLOG.print(s.read_us);
    FLOG.print(F(" ov="));
    FLOG.print(overrun);
    FLOG.print(F(" im="));
    FLOG.print(acc_int_count);

    /*
     * Velocity window: signed integral and how much of it was actually
     * observed. cv= well under VEL_WINDOW_MS means the buzzer or a fault is
     * eating samples and w= is built on less data than it looks like -- the
     * one thing a capture must not hide. Costs ~14 characters a line, about
     * 5% more of the 9600 baud budget.
     */
    FLOG.print(F(" w="));
    FLOG.print(vel_window.w(), 3);
    FLOG.print(F(" cv="));
    FLOG.print(vel_window.coverage_ms());

    /*
     * Lateral axes. Exploration only -- nothing reads these yet. The question
     * they exist to answer is whether |x|,|y| separate a travelling unit from a
     * parked one, which would give a cruise-confirm and 3 s false-alarm veto
     * that z physically cannot (see the sample_log_t comment).
     */
    FLOG.print(F(" x="));
    FLOG.print(s.ax / 1000.0f, 3);
    FLOG.print(F(" y="));
    FLOG.print(s.ay / 1000.0f, 3);

    /*
     * The lateral metric the release path actually decides on, and the quiet
     * run it has accumulated. x= and y= alone are not a substitute: the
     * metric is a difference between CONSECUTIVE UNBLANKED samples, and the
     * log does not show which samples the ISR dropped, so it cannot be
     * reconstructed offline. q= is what makes a release explicable after the
     * fact -- and, during the buzzer bench test, what shows whether the
     * metric ever gets under the threshold at all.
     */
    FLOG.print(F(" m="));
    FLOG.print(lat_monitor.m(), 3);
    FLOG.print(F(" q="));
    FLOG.print(lat_monitor.quiet_run());

    /*
     * Raw arrival peak. The log is decimated, so this is the ONLY way the
     * capture shows what the detector actually saw -- the individual sample
     * carrying the brake bounce is very unlikely to be one of the printed
     * ones.
     */
    FLOG.print(F(" pk="));
    FLOG.print(arrival_peak_get(), 2);

    /*
     * B2 -- beacon state on the PERIODIC line, not only on ACC-INT lines.
     *
     * Without this the log cannot establish whether the beacon was sounding at
     * a given moment, so "the beacon did not fire" can be neither confirmed nor
     * refuted from a capture. bz= already prints on ACC-INT lines; those are
     * edge-triggered and absent for exactly the runs where the question
     * matters.
     */
    FLOG.print(F(" bz="));
    FLOG.print(get_buzzer_status() ? 1 : 0);

    /*
     * B4 -- the RETIRED arrival bucket, printed separately from pk=.
     *
     * pk= is max(arr_peak_cur, arr_peak_prev), a 1-2 s sliding window, so at
     * the moment of a stop it ALREADY carries the arrival transient and cannot
     * report the cruise ceiling. arr_peak_prev alone lags the current sample by
     * one full bucket, so when the stop lands in the open bucket the retired
     * one still holds the pre-stop value.
     *
     * That is the cruise ceiling the arrival gate's denominator needs, and the
     * reason it could not be read before: ARRIVAL_PEAK_VALUE was derived
     * against a worst cruise of 0.28 that has never been re-measured in the
     * slow regime. Read cp= during cruise, not at the stop.
     *
     * No new state -- the buckets already exist for the arrival gate. This is
     * printing, not machinery.
     */
    FLOG.print(F(" cp="));
    noInterrupts();
    float cp_r = arr_peak_prev;
    interrupts();
    FLOG.print(cp_r, 2);

    if (err_total) {
        FLOG.print(F(" et="));
        FLOG.print(err_total);
    }

    /*
     * TWI wait-guard trips, printed only when nonzero -- same discipline as
     * et=. Each one is a wedged TWI transaction that the vendored Wire1's spin
     * bound caught and recovered by re-initialising the peripheral. Before the
     * guard existed this was a hard freeze inside the sample ISR, ending in a
     * watchdog reset (Reset cause: 0x8) with the beacon dropped -- and on
     * 2026-08-12 a pair of them cost an entire car run. So tw= appearing at all
     * is GOOD NEWS about the recovery and BAD NEWS about the bus: the wedge is
     * still happening, it is just no longer fatal. Watch it alongside ov=.
     * Details and the rollback in lib/Wire1/src/utility/twi1.c.
     */
    uint8_t tw = twi1_guard_trips1();
    if (tw) {
        FLOG.print(F(" tw="));
        FLOG.print(tw);
    }

    FLOG.print(F("\r\n"));
#endif  /* FALCON_LOG >= 2 */

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

    FLOG.print(F("ACC-INT n="));
    FLOG.print(count);
    FLOG.print(F(" t="));
    FLOG.print(when);
    FLOG.print(F(" st="));
    FLOG.print(ms.get_state());
    FLOG.print(F(" bz="));
    FLOG.print(get_buzzer_status() ? 1 : 0);
    FLOG.print(F(" pin="));
    FLOG.print(digitalRead(PIN_ACC_INT1));
    FLOG.print(F("\r\n"));
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
        FLOG.print(F("ACC-STAT read FAILED\r\n"));
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
        FLOG.print(F("ACC-STAT STUCK "));
        FLOG.print(stuck_polls);
        FLOG.print(F(" polls, re-arming any-motion\r\n"));

        stuck_polls = 0;

        rslt = bma456.configureAnyMotion(ANYMOTION_THRESHOLD,
                                         ANYMOTION_DURATION,
                                         BMA4_DISABLE,
                                         ANYMOTION_INT_LINE);
        if (rslt != BMA4_OK) {
            FLOG.print(F("ACC-STAT re-arm FAILED : "));
            FLOG.print(rslt);
            FLOG.print(F("\r\n"));
        }
        return;
    }

    if (status != 0 || digitalRead(PIN_ACC_INT1) != 0) {
        FLOG.print(F("ACC-STAT s=0x"));
        FLOG.print(status, HEX);
        FLOG.print(F(" pin="));
        FLOG.print(digitalRead(PIN_ACC_INT1));
        FLOG.print(F(" n="));
        FLOG.print(acc_int_count);
        FLOG.print(F(" sk="));
        FLOG.print(stuck_polls);
        FLOG.print(F("\r\n"));
    }
}

#if defined(IDLE_CURRENT_TEST)
/*
 * ─── BENCH ONLY: stepped quiescent-current measurement (test plan A5) ────────
 *
 * ⛔ NEVER SHIP. Built only by [env:idle_current]. It holds D2 in fixed states
 * and disables logging for a quarter of every cycle, so it does not represent
 * shipping behaviour even though it is built from shipping code.
 *
 * WHY A STEPPED TEST RATHER THAN JUST READING THE METER. The heartbeat is an
 * 80 ms event every 4 s. A DMM integrates over ~100-300 ms, so pointed at that
 * it reports neither the baseline nor the peak but an unrepeatable blend of the
 * two, depending on where its window lands. Holding the LED in a FIXED state
 * gives a steady reading that can actually be trusted, and the heartbeat's true
 * average is then arithmetic on a measured number rather than on a datasheet
 * rating:
 *
 *     heartbeat average = (dim - baseline) x HEARTBEAT_FLASH_MS / HEARTBEAT_PERIOD_MS
 *                       = (dim - baseline) x 80 / 4000
 *                       = (dim - baseline) / 50
 *
 * FOUR PHASES, 60 s each, looping forever. Each is announced by N piezo chirps
 * so the phase is identifiable BY EAR -- which matters, because this measurement
 * is only valid with the serial cable DISCONNECTED (it back-powers the board
 * through the RX ESD clamp, §6.1) and there is therefore no console to watch.
 *
 *   1 chirp   D2 off,  logging ON    baseline: shipping idle, minus the beat
 *   2 chirps  D2 dim,  logging ON    dim - baseline = the heartbeat's LED cost
 *   3 chirps  D2 full, logging ON    full - baseline = the ALARM's LED cost
 *   4 chirps  D2 off,  logging OFF   baseline - this = cost of continuous
 *                                    serial logging, which the shipping build
 *                                    does forever whether or not anyone listens
 *
 * Read the meter in the QUIET stretch between markers, not during them.
 *
 * ⚠️ KEEP THE UNIT STILL. If it is knocked, the FSM latches a departure and the
 * beacon runs -- and LATCH_FAILSAFE_MS is now 600 s, so that would wreck the
 * rest of the cycle. If you hear the beacon, restart the test.
 *
 * ⚠️ The heartbeat is forced off throughout. D2 is driven directly here, and
 * two writers on one LED would produce a reading that is neither state.
 */
#define IC_PHASE_MS   60000UL
#define IC_PHASES     4
#define IC_START_MS   15000UL   /* after calibration completes (~13.3 s) */

static void idle_current_service()
{
    static uint32_t ic_last    = 0;
    static uint8_t  ic_phase   = 0;
    static bool     ic_started = false;
    uint32_t        now        = millis();

    if (!ic_started) {
        if (now < IC_START_MS) {
            return;
        }
        ic_started = true;
        ic_phase   = IC_PHASES - 1;   /* so the first step lands on phase 0 */
        ic_last    = now - IC_PHASE_MS;
    }

    if ((now - ic_last) < IC_PHASE_MS) {
        return;
    }

    ic_phase = (uint8_t)((ic_phase + 1) % IC_PHASES);

    /*
     * Silence the heartbeat every phase, not just once: the FSM re-arms it at
     * the end of any recalibration, and a beat landing mid-measurement would
     * move the meter for reasons the phase label does not explain.
     */
    heartbeat_set(HEARTBEAT_OFF);

    /* Marker first -- it is blocking, and drives D2 itself. */
    ready_signal((uint8_t)(ic_phase + 1));

    switch (ic_phase) {
    case 0:  bench_set_d2(0); ic_log_enabled = true;  break;
    case 1:  bench_set_d2(1); ic_log_enabled = true;  break;
    case 2:  bench_set_d2(2); ic_log_enabled = true;  break;
    default: bench_set_d2(0); ic_log_enabled = false; break;
    }

    /* Start the clock AFTER the marker, so every phase is a full 60 s. */
    ic_last = millis();
}
#endif /* IDLE_CURRENT_TEST */

#if defined(BATTERY_BENCH)
/*
 * ─── BENCH ONLY: battery characterisation (Session A of the test plan) ───────
 *
 * ⛔ NEVER SHIP. Built only by [env:bench_battery]. It replaces the threshold
 * logic with raw instrumentation and FORCES the low-battery alarm on a timer,
 * so it cannot report a real battery state and must never be in a car.
 *
 * ⭐ BUT UNLIKE brownout_test, IT IS A SUPERSET OF SHIPPING, NOT A BYPASS. The
 * FSM, calibration, the beacon and the heartbeat all run exactly as shipped.
 * That is deliberate: the one recorded failure of a test build on this project
 * was brownout_test's first flash measuring an UNLOADED RAIL because the call
 * that drives the piezo sat past a `break` and was unreachable. A build that
 * looks like it is working while testing nothing is the failure mode to design
 * against, and the cheapest defence is to remove as little as possible.
 *
 * ⚠️ IT SHARES configure_adc_channel() BYTE FOR BYTE WITH SHIPPING, and must
 * keep doing so. The whole point is to derive a volts-per-count figure that the
 * SHIPPING build can use; characterising under a different ADC clock would
 * re-create precisely the defect being fixed. If the prescaler is changed, it
 * changes for both, and every number taken here is re-taken afterwards.
 *
 * WHAT IT ANSWERS:
 *
 *   A1  the divider ratio -- put a meter on the pack and on the ADC node while
 *       `raw` is printing, and the volts-per-count falls straight out. The
 *       divider is held on across the whole sweep rather than the shipping
 *       10 ms, because metering a node that exists 0.03% of the time is
 *       impractical.
 *
 *   ⬜  IS BATTERY_SETTLE_MS ENOUGH? Unanswered since the day it was written --
 *       10 ms was a generous guess, never a measurement. The sweep reads at
 *       several delays after the divider is enabled; if the counts have stopped
 *       moving by the 10 ms tap, the shipping value is sound. This is nearly
 *       free while the meter is already out.
 *
 *   A4  the chirp. Forced on at BB_CHIRP_ON_MS and off again at
 *       BB_CHIRP_OFF_MS, which exercises enable_battery_alarm(),
 *       disable_battery_alarm() and the buzzer_on-latch fix, WITHOUT editing
 *       BATTERY_LOW_THRESHOLD -- an edit that could too easily be committed.
 *       Shake the unit during the chirp window to confirm the beacon suppresses
 *       it. A7 (a rejected calibration -> double heartbeat wink) also works in
 *       this build, because the FSM is untouched.
 *
 * ⚠️ NOT FOR A5, quiescent current. Measure that on the SHIPPING build, with the
 * serial cable DISCONNECTED -- the cable back-powers the board through the RX
 * ESD clamp (§6.1 of the product document), so a reading taken with it attached
 * is of a partly externally powered board. This build's once-per-second print
 * would inflate the figure it is trying to establish, too.
 *
 * VERIFY THE RIGHT IMAGE IS ON THE DEVICE BY SIZE. Shipping is ~31416,
 * bench_battery is larger than shipping, brownout_test is ~21086.
 */
#define BB_START_MS       15000UL  /* after calibration completes (~13.3 s)   */
#define BB_CHIRP_ON_MS    30000UL
#define BB_CHIRP_OFF_MS   90000UL
#define BB_PERIOD_MS       1000UL

/* Delays after enabling the divider, milliseconds. */
static const uint8_t bb_taps[] = { 1, 3, 6, 10, 20, 50 };
#define BB_TAPS  (sizeof(bb_taps) / sizeof(bb_taps[0]))

static void bench_battery_service()
{
    static uint32_t bb_last  = 0;
    static bool     bb_on    = false;
    static bool     bb_off   = false;
    uint16_t        c[BB_TAPS];
    uint32_t        now = millis();

    /*
     * Stay out of the calibration window entirely. The sweep busy-waits for up
     * to 50 ms, and loop() is what drains the sample ring -- doing that inside
     * the 6 s window would cost samples from the very measurement the lateral
     * floor is learned from, and a lost sample INFLATES the metric.
     */
    if (now < BB_START_MS) {
        return;
    }

    if (!bb_on && now >= BB_CHIRP_ON_MS) {
        bb_on = true;
        FLOG.print(F("BB: forcing low-battery alarm ON\r\n"));
        enable_battery_alarm();
    }

    if (!bb_off && now >= BB_CHIRP_OFF_MS) {
        bb_off = true;
        FLOG.print(F("BB: clearing low-battery alarm\r\n"));
        disable_battery_alarm();
    }

    if ((now - bb_last) < BB_PERIOD_MS) {
        return;
    }
    bb_last = now;

    /*
     * micros(), not millis(). At F_CPU = 1 MHz Timer0 overflows every
     * 256 x 64 = 16384 us, so millis() advances in ~16 ms STEPS and cannot
     * resolve any of the taps below 20 ms. micros() carries 64 us resolution
     * from the same timer, which is ample.
     */
    digitalWrite(BAT_ADC_ENABLE, HIGH);
    {
        uint32_t t0 = micros();
        for (uint8_t i = 0; i < BB_TAPS; i++) {
            while ((micros() - t0) < ((uint32_t)bb_taps[i] * 1000UL)) {
                /* spin */
            }
            c[i] = read_adc_pc2();
        }
    }
    digitalWrite(BAT_ADC_ENABLE, LOW);

    FLOG.print(F("BB t="));
    FLOG.print(now / 1000UL);
    FLOG.print(F("s raw"));
    for (uint8_t i = 0; i < BB_TAPS; i++) {
        FLOG.print(F(" "));
        FLOG.print(bb_taps[i]);
        FLOG.print(F("ms="));
        FLOG.print(c[i]);
    }

    /*
     * The 10 ms tap is the shipping delay, so this mv= is what the shipping
     * build would have recorded -- the number to correlate with the meter.
     *
     * ⚠️ The conversion itself takes ~1.7 ms at the /128 prescaler, so a tap is
     * "delay before the conversion STARTS", and taps closer together than that
     * cannot be fully independent. Good enough to see whether the node has
     * settled by 10 ms; not a substitute for a scope if it has not.
     */
    FLOG.print(F(" mv10="));
    FLOG.print((uint16_t)(((uint32_t)c[3] * 3100UL) / 1023UL));
    FLOG.print(F(" vcc="));
    FLOG.print(read_vcc_mv());
    FLOG.print(F(" bat="));
    FLOG.print(battery_alarm_status_g);
    FLOG.print(F("\r\n"));
}
#endif /* BATTERY_BENCH */

/*
 * This function will check for battery-voltage level and notify the user
 */
void check_for_battery_voltage()
{
    static uint32_t bat_timer       = 0;
    static bool     bat_settling    = false;
    static uint8_t  settled_samples = 0;
    uint16_t  battery_v = 0;
    uint32_t  now       = millis();

    /*
     * ⛔ TRIED AND WITHDRAWN, 2026-08-14: skipping the sample while the beacon
     * sounds. Recorded so nobody re-derives it.
     *
     * The reasoning was that a pack measured under piezo + 200 mA D2 load reads
     * sag rather than state of charge, and that with the trip at 3.2 V a
     * contaminated sample would false-alarm on healthy cells. It rested on one
     * reading of 3.3 V "in alert" -- and the very next measurement, of the PCB
     * rails during an alert, showed a STABLE 4.9 V. There is no sag to exclude.
     *
     * Corroborated by data that was already on file: the 588 s endurance test
     * compared VCC in the blast phase against the quiet phase over 583 samples
     * and found a mean difference of +0.6 mV. The rail does not move under
     * alarm load, so neither the pack reading nor its AVCC reference is
     * disturbed by sampling during one.
     *
     * The behaviour is therefore back to sampling on the plain 30 s cadence
     * regardless of beacon state, because a special case with no measurement
     * behind it is worse than none.
     */

    /*
     * Two-phase and non-blocking: enable the divider, come back once it has had
     * BATTERY_SETTLE_MS to settle, read, disable. Nothing here spins, so the
     * 2 s watchdog is untouched. See the BATTERY SAMPLE CADENCE block above for
     * what this replaced.
     */
    if (!bat_settling) {
        if ((now - bat_timer) < BATTERY_SAMPLE_PERIOD_MS) {
            return;
        }
        digitalWrite(BAT_ADC_ENABLE, HIGH);
        bat_settling = true;
        bat_timer    = now;
        return;
    }

    if ((now - bat_timer) < BATTERY_SETTLE_MS) {
        return;
    }

    bat_settling = false;
    /* Period is measured from the read, so a slow settle cannot compound. */
    bat_timer    = now;

    battery_v = read_adc_pc2_voltage();

    FLOG.print(F("  Voltage value : "));
    FLOG.print(battery_v);

    digitalWrite(BAT_ADC_ENABLE, LOW);

    /*
     * Discard the opening samples entirely.
     *
     * On 2026-08-06 the first read landed in the startup transient -- boot
     * buzzer pulse and chase LEDs loading the rail -- and produced a single 1545
     * on a healthy battery that read 2324-2390 for the rest of the session.
     * Because the alarm had no recovery path, that one sample latched it
     * permanently. See Eng_Notes §10.3.
     *
     * The wall-clock period now puts the first read 30 s after boot, well clear
     * of both the transient and the ready chirp at the end of calibration, so
     * these discards are belt-and-braces rather than the load-bearing defence
     * they were. Kept at 2: 60 s of extra arming delay against a permanently
     * latched false alarm is not a close trade.
     */
    if (settled_samples < BATTERY_SETTLE_SAMPLES) {
        settled_samples++;
        FLOG.print(F("  (settling, ignored)\r\n"));
        return;
    }

    /*
     * Prime the whole window from the first sample we trust.
     *
     * RollingAvg fills its array with init_val (0 here), so a freshly
     * constructed battery_avg reads 0 and climbs one eighth of a sample at a
     * time. Calling add() on an unprimed window would put avg() near 300 for
     * the first reading and trip the low-battery test on a perfectly good
     * battery -- reintroducing the bug this change exists to fix.
     */
    if (settled_samples == BATTERY_SETTLE_SAMPLES) {
        settled_samples++;
        battery_avg.fill(battery_v);
    } else {
        battery_avg.add(battery_v);
    }

    /*
     * Print the PACK voltage alongside the node reading. The whole 2026-08-14
     * battery investigation existed because three documents disagreed about
     * what the logged number meant; a log that states the pack figure directly
     * cannot start that argument again.
     */
    FLOG.print(F("  avg : "));
    FLOG.print(battery_avg.avg());
    FLOG.print(F("  pack_mv : "));
    FLOG.print((uint16_t)(battery_avg.avg() * VBATT_DIVIDER));
    FLOG.print(F("\r\n"));

    /*
     * Decide on the rolling average, never on a single sample, and use separate
     * trip and clear thresholds so a reading hovering at the boundary cannot
     * chatter the alarm on and off.
     */
    if (battery_avg.avg() < BATTERY_LOW_THRESHOLD) {
        if (battery_alarm_status_g == 0) {
            FLOG.print(F("  LOW Battery detected \r\n"));
        }
        enable_battery_alarm();

    } else if (battery_avg.avg() > BATTERY_CLEAR_THRESHOLD) {
        /*
         * The recovery path that did not exist before. disable_battery_alarm()
         * was defined in alarm.cpp and never called from anywhere in the tree,
         * so a latched alarm survived until a true cold boot -- which, given
         * the serial back-feed in §10.1, is harder to achieve than it sounds.
         */
        if (battery_alarm_status_g) {
            FLOG.print(F("  Battery recovered, alarm cleared \r\n"));
        }
        disable_battery_alarm();
    }
}
