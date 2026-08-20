# State of the project — 2026-08-18

Supersedes `falcon_state_of_project_2026-08-14.md`. This document defines the
current build, the detection logic as implemented, the evidence supporting each
armed path, and the open items. It is written to be built from without
reference to session history.

---

## 1. Build definition

**⭐ CURRENT PRIMARY BUILD — updated 2026-08-20.** Everything below the table
describes this build unless a line says otherwise.

| | |
|---|---|
| Tag | **`build-2026-08-20`** on `Falcon_Rel_EFT` — the authoritative reference for this build |
| Commit | `976b4a5`. The **firmware** last changed at `6483f9d`; everything after it is documentation plus `platformio.ini` flags for `bench_battery` and `idle_current`, so the `ATmega328PB` binary is byte-identical from `6483f9d` onward |
| Environment | `[env:ATmega328PB]`, `pio run -e ATmega328PB` |
| Build flags | `-DTWI_FREQ=25000L -Wl,--relax` (`FALCON_LOG` defaults to 2) |
| Flash | **32176 / 32256 bytes — 80 bytes free** |
| RAM | **1499 / 2048 bytes** |
| MCU | ATmega328PB, signature `0x1e9516`, F_CPU 1 MHz (internal 8 MHz RC ÷ 8) |
| Fuses | `lfuse 0x62`, `hfuse 0xD9`, `efuse 0xF5` (BOD 2.7 V), `lock 0xFF` |
| Upload | ISP, `stk500`. **The programmer re-enumerates constantly** — find it with a signature read, do not trust a committed port. `platformio.ini` has the command. |
| Reproducibility | Clean rebuild from committed source produces 32176 bytes |

**What changed on 2026-08-20** (`falcon_session_a_2026-08-20.md`,
`falcon_b1_2026-08-20.md`):

- `RollingAvg` holds a fixed member array; **`malloc`/`free`/`realloc` are gone
  from the image** and `rd=` fell 11464 → 10362 µs. RAM rose 1366 → 1499 because
  those arrays are now *counted* rather than taken invisibly from the heap.
- **`FALCON_LOG`** compiles logging out; see §1.1.
- **B1**: every latch prints its path and context —
  `FSM: Departure latched (polled) ml= q= dq= td=`.
- 🔴 **The polled departure path is now re-arm blanked**, which it never was.
  §3.5.

⚠️ **80 bytes free is not headroom.** Session A recovered 644 and B1 plus the
re-arm gate spent 626 of it. Further instrumentation on this env — B3 is the
outstanding one — needs the space recovered again or must be measured on
`production`.

No bootloader is present (`BOOTRST = 1`), which is why opening the serial port
does not reset the board.

### 1.1 Bench/diagnostic builds

Rebuilt and re-measured 2026-08-20. `FALCON_LOG` is 2 unless stated.

| Environment | Purpose | `FALCON_LOG` | Size | Safe in a car |
|---|---|---|---|---|
| `ATmega328PB` | **the primary build**, full logging, every threshold measured on it | 2 | **32176** | yes |
| `production` | events + a `HEALTH` line every 30 s, no per-sample line | 1 | 31706 | yes — but read §1.2 |
| `production_silent` | nothing on the wire; −343 B RAM | 0 | 25590 | yes — but read §1.2, and it cannot be diagnosed |
| `bench_battery` | battery instrumentation, forced chirp | 1 | 31806 | no |
| `idle_current` | stepped quiescent-current measurement | 0 | 25866 | no |
| `brownout_test` | bypasses FSM | 2 | 20434 | no |

Sizes differ enough that a mis-flash is identifiable by inspection.

`bench_battery` and `idle_current` are pinned below level 2 because they no
longer fit at it (32280 and 32474). `idle_current` is silent for a second and
better reason: the logging it exists to measure around costs ~1.10 mA of the
~3.2 mA idle draw, so measuring quiescent current with the log running measures
the log.

### 1.2 ⚠️ Before shipping a `production` build

Measured 2026-08-20: sample throughput is **25.00 / 25.00 / 25.01 Hz** across
the reference, level 2 and level 1 builds, with `ov` flat in all three. A
quieter build does **not** feed its detectors more samples, so no threshold on
file is invalidated by shipping one. That closes session A's verification
clause in the negative.

It is **not** a licence to lower `LOG_DECIMATE_N`, which is separately refuted
(§2.2). And no `production` build has run in a car.

---

## 2. Production readiness assessment

The build is the most reliable measured to date. It is not yet a production
candidate. The blocking items below are ordered by consequence.

### 2.1 ~~Flash headroom is exhausted — 68 bytes~~ — RESOLVED 2026-08-20

Session A recovered 644 bytes by giving `RollingAvg` a fixed member array, and
`FALCON_LOG` recovers a further 470 at level 1 or 6586 at level 0. B1 and the
polled re-arm gate then spent 626, leaving **80 free on the primary build** and
**550 on `production`**.

⚠️ **Two things this section asserted are no longer true:**

- **`-DWIRE_TIMEOUT` is not the reason to want headroom.** It was superseded on
  2026-08-12 by the vendored `lib/Wire1` spin guard, which bounds all seven TWI
  wait sites for **124 bytes**. This section's 1198-byte framing predates that
  and should not be used to justify the driver swap.
- **The bma4 driver replacement does not recover ~4.9 KB.** `bma456_config_file`
  is 6144 bytes of feature-engine blob that any-motion depends on and the swap
  cannot reclaim; the driver code is ~1500. See `falcon_ramp_armed_2026-08-12.md`
  §8.

**The swap is not needed for headroom.** What remains binding is that the
primary build is again close to full, so further instrumentation belongs on
`production` or waits for the next recovery.

### 2.2 Serial logging is compiled into the shipping build

The build prints a decimated sample line at 62500 baud continuously, whether or
not a listener is attached. Measured cost: **1.10 mA, 34% of the 3.2 mA idle
draw.**

| Logging | 1000 mAh | 1200 mAh |
|---|---|---|
| on | 13.0 d | 15.6 d |
| off | 19.8 d | 23.8 d |

~~No compile-time option to remove it currently exists.~~ **Added 2026-08-20:
`FALCON_LOG`, three levels, one source — see §1.1 and `src/falcon_log.h`.** The
flash saving is measured (470 bytes at level 1, 6586 at level 0, plus 343 bytes
of RAM at level 0). ⬜ **The 1.10 mA is still arithmetic, not a measurement** —
`idle_current` now builds and is the env that would settle it.

**Added 2026-08-19 — two consequences of the logging path that were not
recorded here.**

*The sample line is decimated 8:1 for a constraint that no longer binds.*
`LOG_DECIMATE_N` is 8 because a 145-character line took 151 ms at 9600 baud and
one line per sample would have been 47% of the budget. The link was raised to
62500 on 2026-08-13, putting the same line at about 23 ms. Nobody re-derived the
decimation afterwards, and it is now roughly 6.5x more conservative than the
budget that set it. This is half the cause of instrumentation gap 5, and
lowering it costs no flash.

*The comment block above that define is stale in a way that reads as a live
release blocker.* It still states approximately 25% sample loss, calls it "NOT
tolerable for shipping", warns against trusting any threshold derived at this
rate, and prescribes a sample ring as the proper fix. Both fixes have since
landed — the ring on 2026-08-11 and the baud raise on 2026-08-13 — and the
warning was never retired. Read cold, the source asserts that every constant in
section 4 rests on 80% of the data. Whether that is still true is unmeasured
either way; what is certain is that the stated figure predates both fixes. The
comment should be corrected or the loss re-measured, and `ov=` is the counter
that settles it.

### 2.2a Calibration can be poisoned by an unsettled mounting, silently

**Added 2026-08-19. Open item 13.**

Three calibrations taken on the cartop, device re-seated between each:

| # | reset | max XY bucket | `XY-Still` | `Zero-Calib` | `Threshold-Value` | |
|---|---|---|---|---|---|---|
| 1 | 0x7 | 0.0550 | 0.0750 | 9.708342 | **0.069354** | learned |
| 2 | 0x4 | 0.0430 | 0.0570 | 9.746478 | 0.040000 | clamped to floor |
| 3 | 0x4 | 0.0480 | 0.0540 | 9.741517 | 0.040000 | clamped to floor |

Calibration 1 was taken immediately after the device was handled onto the
cartop. It is the noisiest window on every metric available, and its
`Zero-Calib` sits 0.038 off the other two, which says the unit was not yet in
its settled orientation.

**The mechanism is not in doubt.** `Z_THRESH_MARGIN` is 1.50, so the derived
threshold is 1.5x the *measured rest spread*: a noisier window produces a higher
threshold by construction. Back-computing, calibration 1 measured a rest spread
of ~0.046 against under 0.027 for the two settled ones.

**The direction is the dangerous one.** The source says it plainly — "too high
-> the backstop is dead again, which is the state this replaces" — and
`Z_THRESH_MIN` is documented as sitting *below the weakest departure seen*. A
threshold learned at 1.73x that floor moves toward a dead backstop, not away
from it. So the one value on file that ever cleared the clamp did so by learning
the installer's hands, and a reader who sees only "per-deployment learning
finally produced a value" draws exactly the wrong conclusion.

**Nothing caught it.** `mv=` read 0 on all three. The degraded-calibration flag
does not distinguish the poisoned window from the two clean ones, so there is no
runtime signal and no log signal beyond comparing `XY-Still` across calibrations
that were never previously compared.

**This is a near-miss of a failure already on file.** `Z_CAL_SETTLE_MS` (1500 ms)
exists because an unconverged 32-sample average once inflated the spread to 2.26
and clamped to `Z_THRESH_MAX`, killing the backstop outright. That settle window
handles the *filter's* convergence. It does not handle the *installation*
settling, which is what happened here, and the two have been conflated.

**Consequences.**

1. Any commissioning procedure must require the device to settle before
   calibration, and that requirement does not currently exist anywhere.
2. A `Threshold-Value` above the clamp floor is **not** by itself evidence that
   per-deployment learning works. It has to be paired with evidence the window
   was quiet.
3. Candidate fixes, none measured: reject a calibration whose bucket spread
   exceeds the others by some factor; require two consecutive agreeing
   calibrations; or extend `Z_CAL_SETTLE_MS` to cover physical settling, which
   costs window time inside an already 6 s calibration.

### 2.3 One of three departure detectors is disabled

`VEL_ARMED 0`. The velocity-integral path is documented as the only one that can
detect a departure the others physically cannot — a car ramping to 20 fpm over
4 s averages 0.025 m/s², roughly six times below `ANYMOTION_THRESHOLD`.

Its threshold also currently evaluates to approximately 22.6 fpm, above the
20 fpm requirement, so arming it as-is would not close that gap.

### 2.4 Two armed release paths have no positive evidence

| Path | Armed | Evidence |
|---|---|---|
| Any-motion arrival cluster | yes | operating, primary |
| Polled arrival peak | yes | operating |
| Ramp detector | `RAMP_ARMED 1` | has never executed in any latch |
| Jog verdict | `JOG_VERDICT_ARMED 1` | operating, known to misclassify both ways |

The ramp detector is armed and has never fired. Negative evidence is complete
(it correctly declined all inspection stops, cruise, jogs and repositioning
moves across a full session and 51 replayed departure bursts), but it has never
been observed executing correctly on a drive-controlled stop.

**2026-08-19 — the ramp detector has positive evidence and this subsection is
half retired.** It latched on 9 of 9 drive-controlled contract-speed stops and
0 of 8 inspection stops. `FSM: Arrival (ramp)` never prints because the polled
path triggers first, not because the detector declines; the line it actually
emits is `RAMP latched`. The remaining armed-path-without-evidence concern in
this subsection applies to the other path, not the ramp.
`falcon_arrival_gate_2026-08-19.md` §4.

### 2.5 The arrival gate has no margin at low speed

`ARRIVAL_PEAK_VALUE` is 0.45. Measured arrival peaks on slow up runs from a
lower terminal cluster at 0.460–0.517, four of them at 1.02–1.03x the gate.
The distribution mode sits on the threshold. A marginally softer stop does not
release, and the beacon then runs to `LATCH_FAILSAFE_MS`.

The gate was derived as the geometric middle between a worst cruise of 0.28 and
a weakest arrival of 0.713. The weakest-arrival figure is now 0.460, so the
derivation no longer holds and the gate is not the middle of the current
populations.

### 2.5a The weak-arrival population does not exist at inspection speed

**Added 2026-08-19.**

Six logged runs at 19 fpm on the cartop, deliberately including one stop made as
soft as the machine would allow:

| run | dir | cruise ceiling | arrival peak | separation | `ARM q/ro` | departed |
|---|---|---|---|---|---|---|
| 2 | down | 0.080 | 1.360 | 17.0x | 255/8 | any-motion |
| 3 | up | 0.070 | 3.730 | 53.3x | **6/11** | any-motion |
| 4 | up | 0.060 | 2.250 | 37.5x | 20/10 | **polled, silent** |
| 5 | down | 0.060 | 1.430 | 23.8x | 255/8 | any-motion |
| 6 | down | 0.070 | 2.410 | 34.4x | 255/7 | any-motion (softest stop) |

**The deliberate soft stop produced a HARDER arrival than two ordinary ones.**
2.410 against 1.360 and 1.430. The weakest arrival of the session came from an
ordinary stop, and the whole population sits at 1.360–3.730 — a factor of three
above the 0.460 weakest arrival on file, which is the number that makes this
item urgent.

**Why, per the site.** Soft stops are only achievable on **automatic operation
at contract speed** — 150 fpm on hydraulic, 300–500 fpm on traction. On
inspection the brake sets from motion and the arrival transient is set by brake
mechanics, not by how gently the approach is made. That is why trying to stop
softly on inspection does not produce a weak arrival and can produce a harder
one.

**Consequence: the weak-arrival population is an automatic-operation
phenomenon.** The 0.460 and 0.713 figures the gate was derived from are almost
certainly drive-controlled stops. No number of inspection runs will reproduce
them, so the arrival half of the gate cannot be characterised on inspection at
all.

**This also narrows a dismissal that has been too broad.** The plan refuses
further contract-speed work twice — "all transients are far above threshold at
that speed" and "the failures live at inspection speed". That reasoning is
correct for DEPARTURES: a missed 20 fpm departure is an inspection-speed
failure, and contract speed cannot exhibit it. It was then applied as a blanket
rule, and it has been keeping the project out of the only regime where the
ARRIVAL gate's weak tail exists. Two different failure modes, two different
regimes, one dismissal covering both.

### 2.5b MIN_TRAVEL_MS does not clear the departure ramp at inspection speed

**Added 2026-08-19. Open item 14.**

`MIN_TRAVEL_MS` is 3000 (`movement_service.h`) and gates arrival detection at
`movement_service.cpp` — `if (!arrival_seen && (current_time -
movement_start_timer) > MIN_TRAVEL_MS)`. Its stated purpose, in the comment
directly above it, is that it "keeps the departure transient itself from
satisfying any" of the arrival tests.

At 19 fpm it does not do that. Cruise ceiling measured from the same five runs,
varying only how much of the run start is excluded:

| run | dir | +3 s (`MIN_TRAVEL_MS`) | +5 s | +8 s |
|---|---|---|---|---|
| 2 | down | 0.080 | 0.080 | 0.080 |
| 3 | up | **0.150** | 0.100 | 0.070 |
| 4 | up | **0.100** | 0.060 | 0.060 |
| 5 | down | **0.080** | 0.060 | 0.060 |

On three of four runs the value taken at +3 s is inflated, and on every run the
peak within that window occurs between +3.1 s and +3.8 s — i.e. on the window
boundary itself, which is the signature of a transient still decaying rather
than a level being measured. The ramp is not clear until roughly +5 to +8 s, so
the gate opens 2–5 s early.

**The consequence is measurable, not theoretical.** Run 3 armed with
`ARM q=6` against `ARRIVAL_ARM_SAMPLES` of 5 — a six-sample quiet stretch that
then broke. Settled cruise on that run is 0.070, less than half of
`ARRIVAL_QUIET_MSS` (0.15), so cruise cannot have broken the stretch. The only
point where that run's deviation reaches 0.15 is the ramp tail at +3.2 s. The
arrival collector armed on a lull inside the still-decaying departure ramp and
immediately lost the stretch. `main.cpp` already warns about this class of
failure — "a departure ramp is never quiet, so counting cannot begin until the
departure is over and cruise has been observed" — citing a run that alarmed for
85 s over a car stationary for 78 of them.

**Trap for anyone measuring cruise.** A ceiling taken from a window opening at
`MIN_TRAVEL_MS` is a ramp measurement wearing a cruise label. Read at +3 s these
runs produce a spurious direction dependence (up appearing to be twice down) and
a spurious agreement between the up ceiling and `ARRIVAL_QUIET_MSS` at 0.150.
Both dissolve at +8 s. `graph/session_d.py` opens its window at +8 s for this
reason; `graph/parse_falcon_log.py` still uses +3 s to match the firmware, and
its GATE line should be read with that in mind.

### 2.6 A false latch is a ten-minute alarm

`LATCH_FAILSAFE_MS` is 600000. This was raised from 240 s deliberately, on the
correct reasoning that silence over a moving counterweight is worse than a
prolonged alarm. The cost is now observed: any false latch — including one
caused by handling the device — asserts the beacon for ten minutes unless
cleared by a jog, and jog-clearing is itself unreliable (see §5.4).

### 2.7 Single-installation coverage

One traction car in one building supplied all measured data. A 150 fpm hydraulic
elevator in a second building has been run successfully but was not logged, and
150 fpm is far above the speeds at which failures occur.

---

## 3. Detection architecture as implemented

### 3.1 Signal chain

The BMA456 is read in `TIMER1_COMPA_vect` at 25 Hz. `getAcceleration()` reads
all three axes; `read_acceleration_mss()` derives `accel_value` from **z alone**,
scaled to m/s². X and Y are retained only as a lateral vibration metric.

Every consumer is orientation-agnostic: values are compared against the
calibrated rest (`zero_calib_value` / `arr_zero`) and taken as magnitude, or the
departure sign is learned per run. Inverting the device changes the resting sign
(−9.7 rather than +9.75) and nothing else. This has been verified in both
mountings.

A failed sensor read does not feed the rolling average. The average holds its
last good value and `sensor_err_run` increments, so a dead accelerometer does
not report "still".

### 3.2 States

```
STATE_ERROR_RESET -> STATE_CALIBERATION -> STATE_MONITORING
                                              |  departure
                                              v
                                    STATE_MOVEMENT_DETECTED
                                              |  MOVEMENT_DETECTION_TIMEOUT_MS
                                              v
                                        STATE_MOVING  (beacon on)
                                              |  arrival / jog verdict / failsafe
                                              v
                                    STATE_DECELERATING
                                              |  STOP_CONFIRM_MS
                                              v
                                      STATE_STOPPED -> STATE_MONITORING
```

The FSM is latched: the beacon asserts on departure and holds until an arrival
is detected. It does not clear on constant velocity.

### 3.3 Calibration

Runs for `CALIB_TIMEOUT_MS` (6000) at rest, guaranteed by the deployment
sequence — the device is placed on the counterweight before power-on. It
measures three things:

1. `zero_calib_value` — the z rest baseline, also handed to the raw peak
   detector as `arr_zero`.
2. `XY_STILL` — the lateral noise floor, from bucket maxima, median across
   buckets, multiplied by `XY_STILL_MARGIN` and clamped.
3. `threshold_value` — the polled departure gate, from the z-average spread
   across the window (excluding the first `Z_CAL_SETTLE_MS` while the 32-sample
   average converges), multiplied by `Z_THRESH_MARGIN` and clamped.

An any-motion edge during the window sets `calib_moved`, and the window is
discarded rather than trusted. After `CALIB_RETRIES` the device arms on
`XY_STILL_FALLBACK` rather than refusing to arm, because a device that never
beacons is the unrecoverable failure.

Printed at completion: `XY: calib b= peak= mv=`, `Zero-Calib-Value`,
`Threshold-Value`, `XY-Still-Value`. A `Threshold-Value` of exactly `0.040000`
or `0.200000` indicates the learned value was clamped rather than measured.

### 3.4 Departure detection — three paths, OR'd

A missed departure is the only catastrophic failure, so detectors are combined
with OR and may only cause the unit to alarm sooner.

| Path | Mechanism | Status |
|---|---|---|
| BMA456 any-motion | hardware slope engine, z axis only, 100 Hz, `ANYMOTION_THRESHOLD` sustained over `ANYMOTION_DURATION` | primary; catches essentially all departures |
| Polled threshold | `\|acceleration_avg − zero_calib\| > threshold_value` on a 32-sample (1.28 s) average | backstop; learned per deployment |
| Velocity integral | windowed velocity change > `VEL_DEPART_THRESHOLD` | **disabled** (`VEL_ARMED 0`) |

Any-motion fires within approximately 100 ms; the polled average needs 1.28 s to
build. Any-motion therefore wins in practice, and the polled path is only
visible when any-motion fails. Its firing is identifiable in a log by a
transition to `STATE_MOVEMENT_DETECTED` with no preceding
`FSM: Departure latched (any-motion)` line.

The polled path does not call `burst_trigger()`, so a departure it catches
produces no burst and no `JOGV` line. This is an instrumentation gap.

### 3.5 Re-arm blanking

On entering `STATE_MONITORING`, queued any-motion edges are discarded and a
blanking window opens. Motion evidence inside it is ignored, which prevents both
a delivery-ordering artefact and residual stop ringing from re-latching.

🔴 **Corrected 2026-08-20: until that date this blanked the ANY-MOTION PATH
ONLY.** The polled departure test in `fsm_run()` was gated by nothing and went
live the instant `STATE_MONITORING` was entered. That was survivable while the
polled path was unreachable — `DEFAULT_THRESHOLD_VALUE` was 0.40 and reached 1
of 82 departures — but the self-calibrated threshold clamps to `Z_THRESH_MIN`
(0.040) in every mounting measured, so the **ungated path became the sensitive
one and started latching first, by about 50 ms**.

Measured cost of that gap: two taps on a bench produced **nine
latch/alarm/release cycles in 90 s**, seven of them re-latching 541–1556 ms into
MONITORING on the alarm's own ringdown — the 2026-08-07 failure this blank
exists to prevent, arriving through the door it did not cover. Both paths are
now blanked identically and suppressions print:

```
FSM: polled ignored, re-arm blanking t=426 d=0.0652
FSM: any-motion ignored, re-arm blanking t=475 q=0
```

Verified: the same provocation now produces zero latches inside the blank.
`falcon_b1_2026-08-20.md` §2.3.

⚠️ **The cost, accepted deliberately:** a genuine departure inside the blank is
now missed by the polled path too. Any-motion, the path that actually catches
slow departures, has been blanked in this exact window since 08-07, so this
makes the two consistent rather than opening a new blind spot — but it has not
been tested in a car, and only a slow-run session can say whether that window is
ever occupied.

The window ends when the lateral has been measured quiet
(`lat_monitor.quiet_run() >= MONITOR_REARM_QUIET`), floored by
`MONITOR_REARM_MIN_MS` and capped by `MONITOR_REARM_MS`. Measured duration in
service: approximately 1550 ms.

A fixed 6000 ms window previously discarded real departure edges. Because cruise
produces no further any-motion edges, one discarded edge made an entire run
invisible. See `falcon_departure_detection_2026-08-18.md` §2.

### 3.6 Arrival detection — three paths

Arrival collectors arm only after the departure transient has passed, via a
quiet run or a sign reversal against the learned departure direction. The peak
collector uses the union of both; the ramp detector is gated on reversal only,
because a departure ramp fed to the ramp accumulator is arithmetically
indistinguishable from an arrival and would release the beacon seconds after a
car starts moving.

| Path | Gate |
|---|---|
| Clustered any-motion | `ARRIVAL_EDGE_COUNT` edges within `ARRIVAL_EDGE_WINDOW_MS`, AND `arrival_peak_hit()` |
| Polled peak | windowed peak crosses `ARRIVAL_PEAK_VALUE` |
| Ramp detector | `RAMP_BLOCKS` consecutive same-sign blocks, each above `RAMP_FLOOR_MMSS` at `RAMP_DIR_PCT` directionality |

Any-motion edges raised while the piezo is driven are discarded: the alarm
shakes the accelerometer and the sensor engine sits behind the same mechanical
coupling.

`LATCH_FAILSAFE_MS` ends a latch that no arrival path released.

### 3.7 Jog verdict

Runs once per departure burst, 3.2 s after the latch. Computes deadbanded
signed impulses, then declares a jog when
`ratio >= JOG_OPP_RATIO_PCT AND opk >= JOG_OPP_PEAK_MMSS`, where `opk` is the
opposite-sign peak — the signature of an uncontrolled brake set.

Armed, a jog verdict releases the latch.

Known limitations, both measured: it silences real runs whose departure carries
a mechanical jolt, and it misses gentle jogs. Both follow from the populations
overlapping on `opk`. See `falcon_departure_detection_2026-08-18.md` §3.

### 3.8 Alarm output

One master-stepped sequence drives buzzer, chase LEDs and red LED together at
`ALARM_STEP_MS` resolution, `ALARM_SEQ_STEPS` per sequence. The buzzer blanks
the accelerometer only while the piezo is driven plus `BUZZER_RINGDOWN_MS`,
leaving 75% of each cycle available for sampling — the FSM must hear the arrival
transient while the beacon is sounding.

All eight ring LEDs sweep. The 4017 is reset at each sweep start and stepped
immediately off Q0, which is unpopulated.

---

## 4. Constants reference

Values that carry safety consequence, with derivation and evidence status.

### 4.1 Departure

| Constant | Value | Basis | Status |
|---|---|---|---|
| `ANYMOTION_THRESHOLD` | 32 (0.153 m/s²) | 0.4883 mg/count, datasheet-confirmed | no margin at 20 fpm; a hand bump exceeds it |
| `ANYMOTION_DURATION` | 5 (100 ms) | — | untested against false fires |
| `Z_THRESH_MARGIN` | 1.50 | mirrors `XY_STILL_MARGIN` | unvalidated |
| `Z_THRESH_MIN` | 0.040 | below weakest observed departure (0.050) | governs in both mountings tested |
| `Z_THRESH_MAX` | 0.200 | bounds a calibration taken in motion | caught two commissioning defects |
| `Z_CAL_SETTLE_MS` | 1500 | average converges over 1.28 s | measured |
| `VEL_ARMED` | 0 | — | disabled; threshold also above requirement |

### 4.2 Arrival and release

| Constant | Value | Basis | Status |
|---|---|---|---|
| `ARRIVAL_PEAK_VALUE` | 0.45 | geometric middle of cruise 0.28 / arrival 0.713 | derivation stale; weakest arrival now 0.460 |
| `ARRIVAL_QUIET_MSS` | 0.15 | stated as above cruise max 0.0875 | conflicts with later cruise of 0.23–0.28 |
| `ARRIVAL_ARM_SAMPLES` | 5 | 200 ms at 25 Hz | — |
| `JOG_OPP_RATIO_PCT` | 33 | geometric mean of measured populations | populations now known to overlap — measured 2026-08-20 at runs 0–58 vs jogs 44–99 |
| `JOG_OPP_PEAK_MMSS` | 900 | as above | ⚠️ corrected 2026-08-20: real runs observed at 11–972, **2974 is a jog** (08-18 §3.4 jog-clear range). Scored 1 false JOG / 63 labelled runs |
| `RAMP_FLOOR_MMSS` | 300 | arrival decelerations | rejects slow departures; correct for its own use |
| `LATCH_FAILSAFE_MS` | 600000 | deliberate, 240 s constraint void | ten-minute false alarms |
| `STOP_CONFIRM_MS` | 5000 | primary defence against premature silence | — |

### 4.3 Timing

| Constant | Value | Note |
|---|---|---|
| `CALIB_TIMEOUT_MS` | 6000 | static_assert ties it to `XY_CALIB_BUCKETS` |
| `CALIB_RETRIES` | 2 | worst case 30 s of calibration |
| `MOVEMENT_DETECTION_TIMEOUT_MS` | 200 | dwell before beacon; not zero, so `vel_departure` can accumulate |
| `MIN_TRAVEL_MS` | 3000 | referenced from `movement_start_timer` |
| `MONITOR_REARM_MS` | 6000 | now a cap, not the operative value |
| `MONITOR_REARM_QUIET` | 8 | operative; reversion is 0 |
| `MONITOR_REARM_MIN_MS` | 1500 | floor |

---

## 5. Open items

Ordered by consequence. Items are measured unless stated.

1. ~~**Flash headroom exhausted (68 bytes).**~~ **RESOLVED 2026-08-20 — §2.1.**
   The named candidate was right: `RollingAvg` was allocating with `new`, and
   giving it a fixed member array recovered **644 bytes** (costed at 586) and
   removed `malloc`/`free`/`realloc` from the image entirely. `FALCON_LOG`
   recovers a further 470 at level 1 or 6586 at level 0. ⚠️ The TWI-timeout
   framing above is itself stale: `-DWIRE_TIMEOUT` was superseded on 2026-08-12
   by the vendored `lib/Wire1` spin guard for 124 bytes. **What remains is that
   80 bytes free is not headroom** — B1 and the polled re-arm gate spent 626 of
   the recovery, so further instrumentation belongs on `production`.
2. **Arrival gate has no margin at low speed.** Mode of the distribution sits on
   the threshold; four measurements at 1.02–1.03x. §2.5. **2026-08-19: the
   CRUISE CEILING half is measured — 0.060–0.080 across five logged runs, both
   directions, at 19 fpm on the cartop, against the 0.28 the gate was derived
   against.** The weakest-arrival half is NOT measured and cannot be, at
   inspection speed: see §2.5a. **BOTH HALVES NOW MEASURED at contract speed
   the same afternoon: cruise 0.080–0.120, arrivals 0.463–0.700 over five
   automatic runs at 500 fpm. The gate sits at 0.97x the weakest arrival — it is
   already below one of the five. Re-derives by the original method to ~0.236,
   roughly half the shipped 0.45. Five samples, 51% spread; the twenty-run
   minimum stands.** Report: `falcon_arrival_gate_2026-08-19.md`.
3. **Knocks and slow departures are not separable** by amplitude or waveform
   shape. Sensitivity is a design trade. Four approaches refuted. §6.
4. **Jog verdict misclassifies in both directions** — silences jolt-heavy real
   runs, misses gentle jogs. Same overlap in `opk`. Redesign must be replayed
   before flight.
5. **Velocity departure path disabled**, and its threshold is above the 20 fpm
   requirement. §2.3.
6. **Ramp detector armed with no positive evidence — RESOLVED 2026-08-19,
   positively.** §2.4. It latched on **9 of 9** drive-controlled contract-speed
   stops (`dir=100`, mean 571–610 mm/s²) and **0 of 8** inspection stops, which
   is session E's exit criterion in full. An earlier reading the same day
   concluded the opposite by counting `FSM: Arrival (ramp)`, which is zero
   because the polled path wins the race and the FSM has already transitioned —
   the detector emits `RAMP latched`. Do not lower `ARM_REV_SAMPLES` on this:
   the three runs showing `g=0` at ARM time latched anyway, and the constant
   also gates the peak collector, so loosening it is the false-release
   direction. Report: `falcon_arrival_gate_2026-08-19.md` §4.
7. **Serial logging in the shipping build** costs 34% of idle current. §2.2.
8. **`ARRIVAL_QUIET_MSS` conflicts with measured cruise.** May prevent quiet
   arming on faster machines; consistent with arming-margin spread. Not
   investigated.
9. **Re-arm correction may have re-opened the 2026-08-07 stationary-car alarm.**
   Not observed in 24 runs; not ruled out.
10. **Self-calibrated z threshold is unproven.** Clamps to floor in both
    mountings tested. **2026-08-19, three calibrations on the cartop with the
    device re-seated between them: 2 clamped to `Z_THRESH_MIN`, 1 "learned" at
    0.069354.** That is the first value on file above the floor, and it is not
    evidence that learning works — see item 13. The item stands as written.
11. **Single-installation coverage.** One logged installation; one unlogged
    second machine.
12. **Any-motion uses a latched reference.** Documented as permitting silent
    death of departure detection, with nothing checking it. No confirmed
    instance; the 2026-08-18 misses were explained by §2.2 of the test report
    instead.
13. **Calibration has no guard against being run before the installation has
    physically settled, and the failure is silent and in the dangerous
    direction.** Measured 2026-08-19. New. §2.2a. A fourth calibration later
    the same day clamped again at the quietest window yet (`XY-Still` 0.0465),
    making it 3 of 4 clamped and the noise→threshold relationship monotonic
    across all four. The single learned value remains the noisiest window and
    the only one taken straight after handling.
14. **`MIN_TRAVEL_MS` (3000) does not clear the departure ramp at inspection
    speed, and the arrival gate opens into the still-decaying transient.**
    Measured 2026-08-19. New. §2.5b. **Narrowed the same day: at contract speed
    it is adequate** — the cruise reading is identical at +3 s and +5 s on every
    500 fpm run, so the ramp has decayed by `MIN_TRAVEL_MS` there. This is an
    inspection-speed defect specifically.

---

## 6. Approaches tested and refuted

Recorded so they are not re-derived. Detail in
`falcon_departure_detection_2026-08-18.md` §3.

| Approach | Result |
|---|---|
| Retune `ANYMOTION_THRESHOLD` | A hand bump (0.136 m/s²) exceeds a real 18 fpm departure (0.116). No value satisfies both directions. |
| Replace `opk` with windowed waveform shape | Complete overlap across 177 bursts. A jog's departure is a real departure; only the reversal differs, which `opk` already measures. **Re-tested 2026-08-20 against labels, not just bursts: 14 of 17 labelled jogs fall inside the labelled run range. Refuted on labels as well as on structure.** |
| Windowed shape as a knock pre-filter | A confirmed 20 fpm departure scored inside the knock band. Any floor rejecting knocks also rejects it. |
| Raise `JOG_OPP_PEAK_MMSS` | A real run was silenced at 972 against a 900 gate; real departures throw jolts of arbitrary size. **Quantified 2026-08-20 on labelled data** (`falcon_corpus_labelled_2026-08-20.md` §4.3): a gate of 973 scores strictly better on the corpus as it stands — but the labelled run ceiling moved 440 → 562 → 972 across three additions of data, so fitting the gate to a sample maximum is what the refutation is about. Still refuted. |
| Fixed polled departure threshold | Mounting-dependent by 2–3x; wrong in every mounting but the one measured. |
| Lower `LOG_DECIMATE_N` for cruise resolution | Bench pair 2026-08-19, N=8 control vs N=2. `ov=` advance 0 → 55 over 90 s; `tk=` per line 8.00 → 3.08 against an expected 2.00; 25.0 → 20.8 Hz apparent; repeated watchdog resets. Fails at 11.8% serial duty, so the link is not the constraint — the ring is drained one sample per loop pass. |

Two method notes:

- Do not score a candidate rule against the `JOGV verdict=` field. That field is
  `opk`-thresholded by definition, so the comparison is circular and reports a
  perfect separation that means nothing.
- Every one of the above appeared valid on three labelled points and failed on
  the next measurement, in each case at the weak end of the real-departure
  population. Test candidates against the corpus and against deliberately
  produced weak departures before accepting them.

---

## 7. Instrumentation gaps

1. ~~The log cannot distinguish a departure latched at the start of travel from
   one latched at the stop.~~ **CLOSED 2026-08-20 by B1** —
   `FSM: Departure latched (path) ml= q= dq= td=`. `dq` counts motion evidence
   the re-arm blank discarded this episode and `td` says how long ago, which is
   the reconstruction the 08-18 session did by hand. `dq≥1` with `td` of a few
   seconds is the missed-departure signature; `dq=0`, or a large `td`, means the
   latch stands on its own. Verified on the bench to read `dq=3 td=90129` across
   three real discards. ⬜ Not yet seen against a real missed departure, which
   is a 17–35 fpm car event. `falcon_b1_2026-08-20.md`.
2. `bz` prints only on `ACC-INT` lines, never on the periodic sample line, so
   the log cannot establish whether the beacon was sounding at a given moment.
3. Lateral `m` does not reveal slow travel — it reads at rest levels during a
   20 fpm run — so a missed departure leaves almost no trace.
4. The polled departure path produces no burst and no jog verdict. **Half
   closed 2026-08-20: it now PRINTS.** B1 emits the latch line at the state
   transition, which every path passes exactly once, so a polled departure is no
   longer silent and run counts taken from `Departure latched` no longer
   undercount. The burst and the jog verdict are still absent — that is B3, and
   it remains outstanding.
   **Confirmed live 2026-08-19, and it is worse than "no burst": the path
   prints NOTHING.** `FSM: Departure latched (any-motion)` sits inside
   `if (any_motion_pending)`, so a polled departure enters MOVEMENT_DETECTED
   with no line at all. A cartop capture held 4 real runs and 3 latch lines, and
   the polled run was dropped from the analysis entirely until the raw state
   transitions were cross-checked by hand. Any run count taken from
   `Departure latched` undercounts silently. **Seen a second time the same day
   at contract speed (4 latched / 5 transitions), and that silent run produced
   the weakest arrival of the session — the single most important measurement
   in `falcon_arrival_gate_2026-08-19.md`. The omission is not random: it drops exactly the runs
   the backstop caught.**
5. Cruise ceiling cannot be measured from current captures: the sample log is
   decimated and sticky-peak, and bursts are too short to isolate cruise.
   **2026-08-19: lowering the decimation was tried on the bench and is
   REFUTED.** `LOG_DECIMATE_N` 8 does discard seven of every eight samples from
   the log — it prints the eighth sample, not the maximum over the eight — so a
   1-in-8 subsample does systematically under-read a cruise ceiling, and the
   constant is indeed sized for a 9600 baud budget the link left behind on
   2026-08-13. But it costs no flash and buys nothing: at N=2 the device loses
   ~35% of samples, its apparent rate falls from 25.0 to 20.8 Hz, and it
   boot-loops under the watchdog, all at 11.8% serial duty. The ring is drained
   **one sample per `loop()` pass**, so log density throttles the consumer
   directly and the eight slots buy latency tolerance rather than throughput.
   The cruise ceiling therefore has to come from a device-side bucket maximum
   (session B, B4) and stays behind item 1. Full figures in the test plan §1.2.
6. The burst corpus carries no ground-truth labels.

**Closed 2026-08-19 — the §6a question the `tk=` counter was added for.** The
counter's own comment states the reading: `tk` advancing ~12 per 6 lines means
the publish/print path is losing samples, ~6 per 6 lines means the ISR is not
firing and no timing from these logs can be trusted. Measured on the bench at
N=8 over three separate 60–90 s captures: **exactly 8.00 ticks per line, min 8,
max 8, zero spread**, with `ov=` advancing 0 and a sample period of
39.99–40.01 ms against a nominal 40. Neither failure mode. The ISR fires
reliably and `loop()` drains every sample it publishes, so timing measured from
these logs at the shipping decimation is sound.

Two consequences. The stale claim in the `LOG_DECIMATE_N` comment block that
every threshold rests on 80% of the data is **wrong for the current build** and
should be corrected in source (§2.2). And `isr_ticks`/`tk=` is marked
"diagnostic only, remove once §6a is resolved" — it is now resolved, so removing
it is a small credit against item 1.

---

## 8. Data

`falcon_srcs/logs/` is gitignored. The distilled event corpus is versioned at
`falcon_srcs/datasets/` — one file per capture, event lines only, verified to
reproduce replay results identically to the raw logs. ⚠️ **The "387 burst
records" written here is not reproducible.** Measured 2026-08-20 by
`graph/session_g.py`: **56 captures, 273 departures, 416 BURST lines** — 204
`k=dep`, 203 `k=arr`, 9 in the pre-`k=` format. Departure bursts replayable by
`arming_replay.load_bursts` number **204**.

**Labels:** `falcon_srcs/datasets/session_g_labels.csv` — 87 of the 273
departures labelled `run`/`jog`/`disturbance` with a named per-record basis,
plus 19 explicit `unknown`. Merge into a worksheet with
`session_g.py --labels`. Method and limits in
`falcon_corpus_labelled_2026-08-20.md`.

Session reports: `falcon_departure_detection_2026-08-18.md` (departure
detection), `falcon_arrival_gate_2026-08-19.md` (arrival gate, contract speed vs inspection).

Replay tools: `graph/session_d.py` (arrival gate margin per run),
`graph/arming_replay.py` (arrival arming),
`graph/jog_window_replay.py` (departure classification),
`graph/calib_replay.py` (calibration window length).
