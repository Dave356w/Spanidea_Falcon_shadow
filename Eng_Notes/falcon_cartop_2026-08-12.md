# Cartop session, 2026-08-12 — inspection runs, jogs, and a new lockup

Firmware `289e0e8` (cleanup + ramp detector) for the first runs, then `796bac8`
(watchdog) mid-session. Dave driving, cartop mounting, programmer and serial
attached throughout. Logs: `device-monitor-260812-095101.log` (runs 1, plus two
repositioning moves) and `device-monitor-260812-104647.log` (everything after).

## 1. 🔴 THE DEVICE NOW LOCKS UP — new this week, five times today

Symptom: frozen solid, a single LED lit, serial silent while the port still
enumerates, and **only an external reset recovers it** (the avrdude signature
read that was already routine for reviving a silent device).

**Cause: unbounded waits in the I2C driver, called from the timer ISR.**
Wire1's `twi1.c` ships with `twi_timeout_us = 0`, which disables timeout
checking on every `while(TWI_READY != twi_state)` in the file — the driver's
own comment says a timeout *"prevents the code from getting stuck in various
while loops here"*. The sample read runs inside `TIMER1_COMPA_vect`, so a bus
glitch that wedges the TWI state machine spins the ISR forever: `loop()` never
runs again, sampling stops, the LEDs latch wherever they were.

**Not SRAM.** ~600 bytes of stack headroom, and a stack overflow corrupts and
resets erratically rather than stalling cleanly and permanently.

**Why it appeared this week:** 25 Hz is 8x the transactions of 3.13 Hz at
~11.5 ms each (`rd=` in every line), so the bus is busy ~30% of wall clock
against ~2% before — roughly **15x the exposure** to the same glitch. The
single `ACC-STAT read FAILED` on 08-11 was the visible end of the same
phenomenon: a hiccup that corrupts a byte returns an error, one that wedges the
state machine never returns at all.

**One freeze happened parked and quiet, ~20 s after a calibration**, with no
alarm running. So piezo/rail disturbance is a correlate — three of five
followed alarm activity — but not the sole trigger.

### 1.1 What was done: a watchdog (`796bac8`)

2 s WDT, kicked only from `loop()`. A wedge now costs a ~12 s
reboot-and-recalibrate instead of a silent corpse. `MCUSR` is captured and
cleared in an `.init3` hook (before C++ constructors — after a WDT reset the
hardware re-arms at 16 ms, so waiting until `setup()` risks a reset loop) and
printed as `Reset cause:`; **0x8 = WDRF means the watchdog caught a lockup.**

**Verified live: two catches within 15 minutes, both self-recovered**, against
three manual avrdude revivals in the hour before the flash.

The cost is real and should not be glossed: **a watchdog reboot mid-run drops
the beacon** for the recalibration. This is recovery, not immunity.

### 1.2 ⬜ The actual fix does not fit

`-DWIRE_TIMEOUT` compiles bounded waits into the driver, so a wedge becomes an
ordinary failed read — which `err_run` already handles correctly (average holds
its last good value, `a=ERR` in the log). No reboot, no lost run. **Measured
+1818 bytes against 714 free (103.4%).** Queued behind the bma4 driver swap
(~4.9 KB). Breadcrumbs are in `platformio.ini` and `main.cpp`.

**This belongs with the brownout on the bench.** A marginal rail that sags
under piezo load and an I2C bus that wedges near alarm activity are plausibly
the same electrical problem, and one afternoon with a meter scopes both.

## 2. Runs — 5 protocol runs, all correct

| # | run | departure | jog verdict | arrival peak | margin vs 0.45 |
|---|---|---|---|---|---|
| 1 | **17 fpm down** | any-motion | RUN 0/99 | 2.164 | 4.8x |
| 2 | 56 fpm down | any-motion | RUN 0/68 | 2.571 | 5.7x |
| 3 | 58 fpm up | any-motion | RUN 0/121 | 0.965 | 2.1x |
| 4 | 56 fpm up | any-motion | RUN 0/130 | **0.758** | **1.68x** |
| 5 | 57 fpm up | any-motion | RUN 0/—  | 1.245 | 2.77x |

Two further alarm cycles in the first log were Dave **repositioning the car**,
not test runs; both latched, verdicted RUN and released correctly.

**17 fpm down is the slowest departure this device has ever caught** (previous
best 18 fpm), and it latched on any-motion with a jog verdict of ratio 0 /
opk 99 — nowhere near the 33/900 gates.

### 2.1 🟡 Up arrivals are weaker than down — direction confirmed, magnitude not

```
down   2.164  2.571
up     0.965  0.758  1.245
```

**5/5 clean separation: every up arrival is below every down arrival** (up max
1.245 < down min 2.164). That is a real property of this machine and worth
carrying into the next installation.

**But the magnitude claim did not survive the next run.** Run 4's 0.758 looked
like the up direction ran at ~1.68x; run 5 at the same speed gave 1.245. The
honest statement is *"the up direction produced the day's worst margin"*, not
*"the up direction runs at that margin"* — the up spread is 0.758–1.245, itself
1.6x. **This is the 08-10/11 method lesson repeating for the fourth time:**
a single run is a hypothesis until a second one under the same conditions
agrees.

What stands regardless: **0.758 is the second-weakest arrival ever recorded**
(lifetime worst 0.713, 2026-08-10), and it came from an ordinary soft stop, not
from a measurement artefact. The 0.45 gate has ~1.7x in its worst observed case.

## 3. Jog verdict: 4/4, opk floor recovered

| jog | ratio | opk | verdict |
|---|---|---|---|
| 1 | 69 | 2225 | JOG |
| 2 | 52 | 1592 | JOG |
| 3 | 79 | 1773 | JOG |
| 4 (reverse) | 99 | 2116 | JOG |

**opk floor today is 1592, above yesterday's 1366** — the drift toward the 900
gate did not continue. Real departures this session ran opk 68–283, so the two
populations remain far apart on the axis that carries the verdict. Ratio ran
52–99 for jogs against 0–15 for real departures, still only the AND guard.

**Lifetime 25/25.** Every armed release fired on a genuine jog and none silenced
a moving car.

## 4. Ramp detector: negative evidence complete, positive evidence still missing

**Zero `FSM: Arrival (ramp)` lines all session — which is exactly right.** Every
stop today was an inspection brake set: the arrival bursts show the
characteristic ring (a large spike then sign flips almost every sample), not a
sustained one-signed plateau. The detector declined all of them, plus all
cruise, all jogs and all repositioning moves.

Runs 4 and 5 are the instructive near-misses: both carry a genuine
one-signed deceleration of ~0.9 m/s^2 — but only **~5 samples (0.2 s)** of it
before the brake sets, against the 36 samples (~1.44 s) the detector requires.
The gate is discriminating on duration exactly as designed.

⬜ **What is still needed: automatic operation.** The detector must fire on
every drive-controlled stop, and inspection operation cannot produce one.
Several automatic runs, both directions, 350 fpm plus a mid-speed, is what
arms it — and the same runs re-measure the 350 fpm arrival distribution
(currently two samples hugging the gate at 0.454/0.465).

## 5. Two instrument bugs found before the session

- **`BURST_POST_ARR` was defined twice** — `movement_service.cpp` (20, the value
  the FSM actually passed) and `main.cpp` (60, used only for the `pre=` label on
  the dump). The 08-11 "20 -> 60" change edited only the label. **So the 350 fpm
  arrival bursts of runs 4–5 were captured 60 pre / 20 post while being dumped
  as 20 pre / 60 post, and the trigger index in that analysis is off by 40
  samples.** `falcon_350fpm_automatic_2026-08-11.md` §3's conclusion that "even
  60 post truncates the stop" is therefore unfounded and must be re-measured.
  Now single-sourced in `movement_service.h` at a real 60 post.
- **The parser's polled-arrival regex had gone stale** (`delta` vs the firmware's
  current `peak`), silently leaving those runs unclosed in every per-run summary.
  Fixed, along with jog-release and ramp patterns.

## 6. Battery telemetry is still blind

Three reads all session — 2384, 2424, 2342 — **every one "(settling, ignored)"**,
across boots minutes apart. The settle counter should have expired long ago.
This is the same symptom as 08-11 afternoon and now looks like a genuine bug in
the settling logic rather than settling. The values themselves look healthy.

**Consequence: the brownout watch ran blind all session again**, and that matters
more now that the device is also locking up.
