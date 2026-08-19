# State of the project — 2026-08-18

Supersedes `falcon_state_of_project_2026-08-14.md`. This document defines the
current build, the detection logic as implemented, the evidence supporting each
armed path, and the open items. It is written to be built from without
reference to session history.

---

## 1. Build definition

| | |
|---|---|
| Commit | `26e38df` on `Falcon_Rel_EFT` |
| Environment | `[env:ATmega328PB]`, `pio run -e ATmega328PB` |
| Build flags | `-DTWI_FREQ=25000L -Wl,--relax` |
| Flash | 32188 / 32256 bytes — **68 bytes free** |
| RAM | 1366 / 2048 bytes |
| MCU | ATmega328PB, signature `0x1e9516`, F_CPU 1 MHz (internal 8 MHz RC ÷ 8) |
| Fuses | `lfuse 0x62`, `hfuse 0xD9`, `efuse 0xF5` (BOD 2.7 V), `lock 0xFF` |
| Upload | ISP, `stk500`, COM6 |
| Reproducibility | Clean rebuild from committed source produces 32188 bytes |

No bootloader is present (`BOOTRST = 1`), which is why opening the serial port
does not reset the board.

### 1.1 Bench/diagnostic builds

| Environment | Purpose | Size | Safe in a car |
|---|---|---|---|
| `ATmega328PB` | shipping | 32188 | yes |
| `bench_battery` | battery instrumentation, forced chirp | 31546 | no |
| `idle_current` | stepped quiescent-current measurement | — | no |
| `brownout_test` | bypasses FSM | 21086 | no |

Sizes differ enough that a mis-flash is identifiable by inspection.

---

## 2. Production readiness assessment

The build is the most reliable measured to date. It is not yet a production
candidate. The blocking items below are ordered by consequence.

### 2.1 Flash headroom is exhausted — 68 bytes

This is the binding constraint on all further work. `-DWIRE_TIMEOUT`, the
correct fix for the I2C driver's unbounded waits, requires 1198 bytes and cannot
be compiled. The current mitigation for a TWI wedge is the watchdog, which
recovers by rebooting and recalibrating — during which the beacon is off.

Recovering space requires either the bma4 driver replacement (approximately
4.9 KB, since the vendored driver is an obsolete variant with a config blob five
times larger than the current Bosch API), or removing serial logging from the
shipping build.

### 2.2 Serial logging is compiled into the shipping build

The build prints a decimated sample line at 62500 baud continuously, whether or
not a listener is attached. Measured cost: **1.10 mA, 34% of the 3.2 mA idle
draw.**

| Logging | 1000 mAh | 1200 mAh |
|---|---|---|
| on | 13.0 d | 15.6 d |
| off | 19.8 d | 23.8 d |

No compile-time option to remove it currently exists. Adding one recovers both
standby life and flash.

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

### 2.5 The arrival gate has no margin at low speed

`ARRIVAL_PEAK_VALUE` is 0.45. Measured arrival peaks on slow up runs from a
lower terminal cluster at 0.460–0.517, four of them at 1.02–1.03x the gate.
The distribution mode sits on the threshold. A marginally softer stop does not
release, and the beacon then runs to `LATCH_FAILSAFE_MS`.

The gate was derived as the geometric middle between a worst cruise of 0.28 and
a weakest arrival of 0.713. The weakest-arrival figure is now 0.460, so the
derivation no longer holds and the gate is not the middle of the current
populations.

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
blanking window opens. Edges timestamped within it are ignored, which prevents
both a delivery-ordering artefact and residual stop ringing from re-latching.

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
| `JOG_OPP_RATIO_PCT` | 33 | geometric mean of measured populations | populations now known to overlap |
| `JOG_OPP_PEAK_MMSS` | 900 | as above | real runs observed at 972–2974 |
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

1. **Flash headroom exhausted (68 bytes).** Blocks the TWI timeout fix and all
   further work. §2.1. **Named candidate not previously listed here
   (2026-08-19): `RollingAvg` still allocates with `new`**, so malloc and free
   are linked into the image. Costed at 586 bytes in the superseded 2026-08-14
   plan (its session D4) and dropped from the 2026-08-18 plan by oversight. It
   also defeats the stated reason the vendored Wire driver avoids `new`.
2. **Arrival gate has no margin at low speed.** Mode of the distribution sits on
   the threshold; four measurements at 1.02–1.03x. §2.5.
3. **Knocks and slow departures are not separable** by amplitude or waveform
   shape. Sensitivity is a design trade. Four approaches refuted. §6.
4. **Jog verdict misclassifies in both directions** — silences jolt-heavy real
   runs, misses gentle jogs. Same overlap in `opk`. Redesign must be replayed
   before flight.
5. **Velocity departure path disabled**, and its threshold is above the 20 fpm
   requirement. §2.3.
6. **Ramp detector armed with no positive evidence.** §2.4.
7. **Serial logging in the shipping build** costs 34% of idle current. §2.2.
8. **`ARRIVAL_QUIET_MSS` conflicts with measured cruise.** May prevent quiet
   arming on faster machines; consistent with arming-margin spread. Not
   investigated.
9. **Re-arm correction may have re-opened the 2026-08-07 stationary-car alarm.**
   Not observed in 24 runs; not ruled out.
10. **Self-calibrated z threshold is unproven.** Clamps to floor in both
    mountings tested.
11. **Single-installation coverage.** One logged installation; one unlogged
    second machine.
12. **Any-motion uses a latched reference.** Documented as permitting silent
    death of departure detection, with nothing checking it. No confirmed
    instance; the 2026-08-18 misses were explained by §2.2 of the test report
    instead.

---

## 6. Approaches tested and refuted

Recorded so they are not re-derived. Detail in
`falcon_departure_detection_2026-08-18.md` §3.

| Approach | Result |
|---|---|
| Retune `ANYMOTION_THRESHOLD` | A hand bump (0.136 m/s²) exceeds a real 18 fpm departure (0.116). No value satisfies both directions. |
| Replace `opk` with windowed waveform shape | Complete overlap across 177 bursts. A jog's departure is a real departure; only the reversal differs, which `opk` already measures. |
| Windowed shape as a knock pre-filter | A confirmed 20 fpm departure scored inside the knock band. Any floor rejecting knocks also rejects it. |
| Raise `JOG_OPP_PEAK_MMSS` | A real run was silenced at 972 against a 900 gate; real departures throw jolts of arbitrary size. |
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

1. The log cannot distinguish a departure latched at the start of travel from
   one latched at the stop. This is the signature of the project's most
   dangerous failure and it has to be reconstructed by hand.
2. `bz` prints only on `ACC-INT` lines, never on the periodic sample line, so
   the log cannot establish whether the beacon was sounding at a given moment.
3. Lateral `m` does not reveal slow travel — it reads at rest levels during a
   20 fpm run — so a missed departure leaves almost no trace.
4. The polled departure path produces no burst and no jog verdict.
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
reproduce replay results identically to the raw logs. 387 burst records.

Replay tools: `graph/arming_replay.py` (arrival arming),
`graph/jog_window_replay.py` (departure classification),
`graph/calib_replay.py` (calibration window length).
