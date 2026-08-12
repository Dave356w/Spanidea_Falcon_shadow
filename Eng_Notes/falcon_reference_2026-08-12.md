# Falcon — product behaviour, detection logic, and reference legend

**As of 2026-08-12, firmware `9d57c9a`, branch `Falcon_Rel_EFT`.**

⚠️ Two things in this document are now out of date, both covered in
`falcon_ramp_armed_2026-08-12.md`: the **ramp detector is ARMED**
(`RAMP_ARMED 1`), and **Wire1 is vendored** into `falcon_srcs/lib/Wire1` with
bounded TWI waits, which adds a `tw=` field to the sample line (printed only
when nonzero — a count of wedged transactions caught and recovered).

This is the orientation document: what the device does, how it decides, and
what every name in the source means. It is written to be read *before* the
session notes, which assume all of this.

Session notes, chronological:
`falcon_analysis_2026-08-06.md` → `falcon_state_of_project_2026-08-07.md` →
`falcon_spec_primary_usecase_2026-08-09.md` → `falcon_zxy_bench_2026-08-10.md`
→ `falcon_25hz_arrival_2026-08-10.md` → `falcon_signature_2026-08-11.md` →
`falcon_jog_verdict_2026-08-11.md` → `falcon_350fpm_automatic_2026-08-11.md` →
`falcon_cartop_2026-08-12.md` → `falcon_cab_automatic_2026-08-12.md` →
`falcon_ramp_armed_2026-08-12.md`.

---

## 1. What the product is

A battery-powered **movement beacon** placed on an elevator counterweight (or
cartop) during maintenance. While the counterweight is moving it sounds a piezo
and flashes chase LEDs, so a mechanic in the hoistway can locate it by ear and
eye. When it stops, the beacon must stop.

**The two failure modes, in order of seriousness:**

1. **Silence while moving** — the mechanic believes a moving counterweight is
   parked. Catastrophic. Every design decision biases against this.
2. **A "position lie"** — the beacon asserting movement over a stationary
   counterweight. Destroys trust in the instrument and drains the pack.

Everything below is shaped by that asymmetry: **a detector may fail toward
alarming, never toward silence.**

**Primary use case is inspection operation** (continuous-pressure, slow,
frequent short moves). Normal automatic operation is a secondary case the
device covers less well — see §6.

---

## 2. Hardware, in one paragraph

ATmega328PB at **1 MHz** (fuse), PlatformIO/MiniCore. **BMA456** accelerometer
on **TWI1** (`Wire1`), sampled by a Timer1 CTC interrupt at **25 Hz**. Piezo on
`PIN_PIEZO`, chase LEDs via a clock line, battery sensed on an internal ADC
channel. A DPS310 pressure sensor is depopulated — pressure code is gone.
Serial logging at 9600 baud, which is a hard constraint on how much can be
printed (§8).

**Flash is effectively full: 32210 / 32256 bytes.** This blocks the TWI timeout
fix and any new instrumentation until the vendor bma4 driver is replaced
(~4.9 KB recoverable).

---

## 3. The core physical problem

**At constant velocity a moving car and a parked car are indistinguishable on
the vertical axis.** Both read 1 g. There is no filter, threshold, or duration
that separates them, because there is no signal to separate.

Consequences that shape the whole design:

- Movement can only be detected as **events** — a departure transient and an
  arrival transient — never as a state.
- Therefore the alarm must be **latched**: set on departure, held through the
  unobservable middle, released on arrival.
- A "has it gone quiet?" rest detector **cannot exist** on this axis. One was
  built on 2026-08-07 and released the beacon mid-ride twice. See the ⛔ block
  in `movement_service.h`.
- The lateral axes carry no gravity pedestal and *were* the hope for a rest
  detector. Measured contrast parked-vs-cruise: **1.19×, at both 3.13 Hz and
  25 Hz.** That path is dead; `lateral.*` survives as instrumentation and as
  the calibration quality gate.

---

## 4. Operating sequence

```
power on (device already placed on the counterweight)
   │
   ├─ 10 s SELF-CALIBRATION  ── learns the z zero and the lateral floor
   │        rejected and retried if movement is seen (CALIB_RETRIES)
   │        chirps once when ready, three times if it had to fall back
   │
   ├─ MONITORING ─── any-motion interrupt ──► DEPARTURE
   │                                            │
   │                                     beacon ON, latched
   │                                            │
   │                      ┌─────────────────────┴────────────────────┐
   │                      │  arrival detection (several paths, §5)   │
   │                      └─────────────────────┬────────────────────┘
   │                                            │
   │                                    DECELERATING
   │                        (5 s of continuous quiet required)
   │                                            │
   └──────────────────────── beacon OFF ────────┘
```

**FSM states** (`MotionStates`, `st=` in the log):

| # | state | meaning |
|---|---|---|
| 1 | `STATE_CALIBERATION` | 10 s learning window at power-on |
| 2 | `STATE_MONITORING` | parked, watching for a departure |
| 3 | `STATE_MOVEMENT_DETECTED` | departure seen, 200 ms dwell before the beacon |
| 4 | `STATE_MOVING` | beacon latched on, hunting for the arrival |
| 5 | `STATE_DECELERATING` | arrival seen, confirming the car has settled |
| 6 | `STATE_STOPPED` | transient; immediately re-enters MONITORING |
| 7 | `STATE_ERROR_RESET` | boot state |

---

## 5. How each decision is actually made

### 5.1 Departure — the sensor's own engine

The BMA456 **any-motion** feature runs on the sensor's 100 Hz stream and
raises an interrupt. This is the only departure detector that works: the
firmware's own polled average spans 1.28 s and flattens a departure ramp to
nothing — it has never caught one in a hoistway.

A polled threshold test remains as a redundant path, and costs nothing.

### 5.2 Arrival — three paths, all reaching the same exit

Detection is **on the raw sample, not the rolling average.** A brake bounce of
0.938 m/s² registers 0.0072 on a 32-sample average — averaging destroys exactly
the signal that matters.

| # | path | fires on | status |
|---|---|---|---|
| 1 | clustered any-motion **+** raw peak | ≥2 interrupt edges in 2.5 s, corroborated | live |
| 2 | raw windowed peak | peak > `ARRIVAL_PEAK_VALUE` | live, **and marginal** — §6 |
| 3 | **ramp detector** | sustained one-signed deceleration | **built, UNARMED** |
| — | jog verdict | a jog's impulse pair | live, releases ~4 s after latch |
| — | failsafe | `LATCH_FAILSAFE_MS` elapsed | fault, logged as one |

**The peak must be windowed, not cumulative.** A running maximum ratchets
upward through cruise vibration and eventually crosses any threshold — the
catastrophic direction, reached not by a bad threshold but by a detector that
cannot forget. Two alternating 1 s buckets give a 1–2 s sliding window.

**But it must also latch.** A jog's stop crossed the threshold at t+1.0 s while
`MIN_TRAVEL_MS` did not open the gate until t+3.2 s — detected, then discarded.
`arr_hit` is sticky for the run; that is safe because cruise never legitimately
exceeds the gate.

### 5.3 Arming — the gate in front of both detectors

Neither the peak collector nor the ramp accumulator runs until the run is
**armed**. Without this, the departure transient would satisfy the arrival test
a fraction of a second after setting the latch.

**Two paths, either sufficient:**

- **quiet** — `ARRIVAL_ARM_SAMPLES` consecutive samples inside
  `ARRIVAL_QUIET_MSS`. Works whenever a run has cruise.
- **sign reversal** — `ARM_REV_SAMPLES` consecutive samples opposite in sign to
  this run's departure. Physics guarantees the sign: a car sheds exactly the
  velocity it gained. Needs no lingering, so it survives runs with no cruise.

**🔴 The arming gate is load-bearing.** Replayed against every burst on file,
*departure* ramps satisfy the ramp block test as readily as arrivals do — the
arithmetic cannot tell them apart, since they are the same shape with opposite
sign and sign encodes direction of travel, not phase. Arming is the only thing
preventing a release seconds after the latch, on a moving car. Do not weaken
it; re-run the replay after any change.

**🔴 Arming margin is an install-time property.** Two mountings in the same car
minutes apart:

```
              2→1 down      1→2 up
mounting A    9, 8, 9       8, 8
mounting B    6, 5, 5, 6    10, 11        (need = 5)
```

On mounting B the descent arms on the *last possible sample*. One more sample
lost to piezo blanking and neither detector runs at all — which produced an
85 s beacon over a car stationary for 78 of them. **A mechanic's placement
decides whether the beacon releases at the bottom terminal.**

### 5.4 Release confirmation

Reaching `STATE_DECELERATING` does not silence anything. The average must stay
within `STOP_BAND_VALUE` of the zero calibration for `STOP_CONFIRM_MS`
continuously — the window restarts on any excursion. This holds the beacon
through levelling, which would otherwise read as "stopped" and let the
subsequent brake set re-latch as a fresh departure.

### 5.5 The jog problem

A movement finishing inside `MIN_TRAVEL_MS` has its stop discarded unseen, so
the beacon runs to the failsafe over a stationary counterweight. Three fixes
were measured dead (velocity magnitude, lateral level, arm-on-quiet).

What works is the **impulse pair**: a jog is a hand-jerk one way and a
brake-jerk the other, net velocity ≈ 0, inside ~1 s; a real departure has no
compensating partner. Classified from the departure burst 3.2 s after the
latch. **`opk` (opposite-side peak) carries the verdict; the ratio is only an
AND guard.** Live and armed — **25/25 lifetime**.

---

## 6. Where the product stands

**Inspection operation works.** Departures caught to **17 fpm** (the slowest
ever measured), arrivals released with 2–5× margin, jogs verdicted 25/25.

**Automatic operation releases on luck.** Nineteen automatic stops measured:

```
arrival peak   0.454 ... 0.544      gate 0.45
```

Every one inside 1.21× of the gate; four inside 1.06×. This is not a
distribution with a thin tail — it is centred on its own threshold. The cause
is structural: the peak is a deviation from a rolling average, and a sustained
deceleration ramp drags the average with it, so the metric gets *weaker* as
ramps get longer (faster cars, taller buildings).

**The ramp detector answers this and is not armed.** Seventeen automatic stops:

```
ramp mean        470 ... 513         floor 300
directionality   100% on every one   gate 85%
```

against complete negative evidence from inspection operation — brake stops,
jogs, cruise, departures, all declined. It is the strongest discriminator this
device has produced and it is one constant away from being live.

**Open, in priority order:**

1. **bma4 driver swap** — 46 bytes free; blocks everything else.
2. **TWI lockup** — `Wire1` waits unbounded and the sample read runs in an ISR;
   a bus glitch freezes the device. A 2 s watchdog recovers it (2 catches
   verified); `-DWIRE_TIMEOUT` is the real fix and does not fit.
3. **Brownout** — the rail sags under sustained piezo load and the board dies
   ~243 s into an alarm, over 300 ADC counts *above* the low-battery trip.
   Tabled by Dave 2026-08-12.
4. **Arm the ramp detector.**
5. **Battery telemetry** prints "(settling, ignored)" indefinitely — a bug in
   the settling logic, not settling.

---

## 7. Function legend

### `main.cpp` — sampling, detectors, instrumentation

| function | context | role |
|---|---|---|
| `setup()` | boot | serial, SPI, alarms, BMA456, any-motion, ADC, **arms the watchdog last** |
| `loop()` | main | kicks the watchdog, runs the FSM, drains all emitters |
| `read_acceleration_mss()` | **ISR** | one I2C read; feeds the average, peak, ramp, burst and sample ring |
| `ring_push()` | ISR | commits one sample record; drops the NEW sample when full so drained records stay contiguous |
| `emit_sample_log()` | loop | drains one ring record per call, feeds the velocity/lateral windows, prints every `LOG_DECIMATE_N`th |
| `emit_burst_log()` | loop | dumps a frozen burst, then runs the jog verdict on departure bursts |
| `emit_ramp_log()` | loop | prints the ramp verdict on its latching edge, **regardless of FSM state** |
| `emit_arm_log()` | loop | one arming summary per run, on leaving `STATE_MOVING` |
| `emit_acc_int_log()` | loop | reports new any-motion edges, hands them to the FSM |
| `poll_acc_int_status()` | loop | reads/clears the sensor status; re-arms any-motion if stuck |
| `burst_trigger()` | either | arms the burst recorder with a pre/post split |
| `arrival_peak_reset()` | FSM | starts a fresh run: peak, ramp, arming state |
| `arrival_peak_hit()` / `_get()` | FSM | sticky crossing / current windowed peak |
| `arrival_zero_set()` | FSM | publishes the zero calibration to the ISR |
| `ramp_hit()` / `ramp_mean_get()` / `ramp_dir_get()` | FSM | ramp verdict and its evidence |
| `enable_timer()` | boot | Timer1 CTC at 25 Hz — **register order is load-bearing** |
| `acc_int1_isr()` | ISR | counts an edge and timestamps it. Nothing else, deliberately |
| `wdt_early_disable()` | `.init3` | captures `MCUSR`, disables the WDT **before** C++ constructors |
| `check_for_battery_voltage()` | loop | averaged battery read with hysteresis |

### `movement_service.cpp` — the FSM

| function | role |
|---|---|
| `fsm_run()` | the state machine; all transitions live here |
| `notify_any_motion()` | departure latch / arrival cluster, **judged by when the edge happened, not when it arrived** |
| `jog_release()` | accepts a JOG verdict; consumed on the next pass through the failsafe's own exit |
| `get_state()` / `set_state()` | accessors |

### Support modules

| unit | role |
|---|---|
| `velocity.*` | windowed velocity integral. Departure path characterised, **unarmed**; the arrival conservation test is deleted (measured dead) |
| `lateral.*` | lateral metric and in-situ calibration. Release path **deleted**; survives as instrumentation and the calibration quality gate |
| `alarm.*` | piezo/LED drive, buzzer blanking, battery alarm |
| `adc.cpp` | internal ADC for battery sense |
| `RollingAvg.h` | fixed-size rolling average |
| `graph/parse_falcon_log.py` | log analysis: per-run summaries, arming/late-release/multi-boot checks |

---

## 8. Variable legend — the log line

```
t=123456 a=9.750 avg=9.763 st=4 rd=11584 ov=39 tk=1703 im=3
        w=-0.029 cv=4686 x=0.647 y=-0.084 m=0.023 q=2 pk=0.06
```

| field | meaning |
|---|---|
| `t=` | `millis()`. **Restarts at every reboot** — split captures on `Device Booted` |
| `a=` | raw z acceleration, m/s² (≈9.76 at rest) |
| `avg=` | 32-sample rolling average (1.28 s) |
| `st=` | FSM state, §4 |
| `tw=` | TWI wait-guard trips, printed only when nonzero. Wedged transactions the vendored Wire1 caught and recovered by re-initialising the peripheral; each one would previously have frozen the sample ISR until the watchdog fired. See `lib/Wire1/src/utility/twi1.c`. |
| `rd=` | duration of the I2C read, µs (~11500 — 29% of the sample period) |
| `ov=` | samples lost to ring overrun |
| `tk=` | ISR ticks since boot; compare against printed lines to separate "ISR not firing" from "print path losing samples" |
| `im=` | cumulative any-motion edge count |
| `w=` | velocity-window integral, m/s (instrumentation) |
| `cv=` | how much of that window was actually observed, ms |
| `x= y=` | lateral acceleration, m/s² — **also the mounting fingerprint** |
| `m=` | lateral metric, `\|dx\|+\|dy\|` between consecutive unblanked samples |
| `q=` | consecutive quiet lateral metrics |
| `pk=` | **windowed raw arrival peak — what the release actually decides on.** `pk=0.00` during a real excursion means the detector was never armed |

### Event lines

| line | meaning |
|---|---|
| `Reset cause: 0xN` | `0x8` = WDT caught a lockup; `0x2`/`0x6` = external reset; `0x1` = power-on |
| `ACC-INT n= t= st= bz=` | any-motion edge; `bz=1` means the piezo was sounding, so it does not count |
| `ACC-STAT` | sensor status poll; only prints when there is something to say |
| `BURST k=dep\|arr pre= n= signed_mmss=` | 80 signed samples, milli-m/s², oldest first |
| `JOGV pos= neg= ratio= opk= verdict=` | jog classifier; `opk` carries the verdict |
| `RAMP latched mean= dir=` | ramp verdict: block mean in mm/s², directionality in % |
| `ARM q= a= v=` | per-run arming summary: longest quiet stretch, armed flag, **`v=1` quiet / `v=2` sign reversal** |
| `FSM: Arrival (…)` | which path released the latch |
| `FSM: FAILSAFE` | no arrival was ever detected — a fault |

### Key internal variables

| name | where | meaning |
|---|---|---|
| `arr_zero` | ISR | zero calibration the peak measures against |
| `arr_armed` / `arm_via` | ISR | the arming gate and which path opened it |
| `arr_quiet` / `arr_quiet_hi` | ISR | current and longest quiet run; `hi` freezes when the arming stretch breaks |
| `arr_dep_sign` | ISR | this run's departure sign, fixed over `ARM_DEP_SIGN_N` samples |
| `arr_opp` | ISR | consecutive opposite-signed samples |
| `arr_peak_cur/prev` | ISR | the two sliding-window buckets |
| `arr_hit` | ISR | sticky: the peak crossed the gate this run |
| `ramp_sum/abs/n/run/sign` | ISR | ramp block accumulator |
| `ramp_hit_v` | ISR | sticky ramp verdict |
| `burst[]`, `burst_post`, `burst_ready` | ISR/loop | burst recorder ring and trigger state |
| `vel_departure` | FSM | departure integral (instrumentation) |
| `arrival_seen`, `any_motion_pending`, `jog_release_pending` | FSM | per-run flags |
| `boot_mcusr` | `.noinit` | reset cause, captured before constructors |

---

## 9. Threshold legend

**⚠️ Every value below is a measurement or a decision, not a default.** The
reasoning for each lives in a comment at its `#define`. Read that before
changing one.

### Arrival detection

| constant | value | meaning |
|---|---|---|
| `ARRIVAL_PEAK_VALUE` | **0.45** | raw windowed peak that declares an arrival. Sits 1.6× above worst cruise and 1.6× below the weakest arrival. **19 automatic stops land at 0.454–0.544** |
| `ARRIVAL_PEAK_WINDOW_MS` | 1000 | sliding window (two buckets → 1–2 s effective) |
| `ARRIVAL_QUIET_MSS` | 0.15 | what counts as quiet, for arming |
| `ARRIVAL_ARM_SAMPLES` | 5 | consecutive quiet samples to arm. **Measured margin: 123 with cruise, 8–9 bottom terminal, 5–6 on an unfavourable mounting** |
| `ARM_REV_SAMPLES` | **8** | consecutive opposite-signed samples to arm. Bracketed by replay: 7 gives a false arrival, 10 loses coverage |
| `ARM_DEP_SIGN_N` | 25 | samples used to fix the departure's sign |
| `ARRIVAL_EDGE_COUNT` / `_WINDOW_MS` | 2 / 2500 | any-motion clustering |
| `MIN_TRAVEL_MS` | 3000 | how long before arrival tests are allowed |

### Ramp detector (built, unarmed)

| constant | value | meaning |
|---|---|---|
| `RAMP_ARMED` | **0** | ⛔ log-only |
| `RAMP_BLOCK_N` | 12 | samples per block (0.48 s) |
| `RAMP_BLOCKS` | 3 | consecutive qualifying blocks (~1.44 s) |
| `RAMP_FLOOR_MMSS` | **300** | block-mean floor. Plateau measures 0.49–0.61 **per configuration** — 08-11 gave 0.605, 08-12 gave 0.49 |
| `RAMP_DIR_PCT` | 85 | directionality floor. Drive ramps 100%, brake stops 2–42% |

### Jog verdict (live, armed)

| constant | value | meaning |
|---|---|---|
| `JOG_VERDICT_ARMED` | **1** | releases the latch on a JOG verdict |
| `JOG_DEADBAND_MMSS` | 150 | samples below this feed neither impulse |
| `JOG_OPP_RATIO_PCT` | 33 | opposite/primary ratio — the AND guard |
| `JOG_OPP_PEAK_MMSS` | **900** | opposite-side peak — **carries the verdict.** Real departures ≤440, jogs ≥1366 |

### Timing and release

| constant | value | meaning |
|---|---|---|
| `LATCH_FAILSAFE_MS` | **240000** | ⚠️ a placeholder around the brownout, not a tuning value — the hardware dies at ~243 s |
| `STOP_CONFIRM_MS` | 5000 | continuous quiet before silencing |
| `STOP_BAND_VALUE` | 0.10 | what "stopped" means to the confirm timer |
| `MONITOR_REARM_MS` | 6000 | deafness after a release, so arrival ringing cannot re-latch |
| `MOVEMENT_DETECTION_TIMEOUT_MS` | 200 | dwell before the beacon sounds |
| `CALIB_TIMEOUT_MS` / `CALIB_RETRIES` | 10000 / 2 | calibration window and retries |

### Sensor and sampling

| constant | value | meaning |
|---|---|---|
| `ANYMOTION_THRESHOLD` | 32 | sensor units. 2.2× a measured departure, 0.58× cruise vibration |
| `ANYMOTION_DURATION` | 5 | 50 Hz samples = 100 ms sustain |
| `ACC_INT_POLL_MS` / `ACC_INT_STUCK_POLLS` | 1000 / 8 | status poll; re-arm if stuck |
| `SAMPLE_RING_N` | 8 | sample ring depth |
| `LOG_DECIMATE_N` | 8 | print 1 line in 8 — 9600 baud is the constraint |
| `BURST_N` / `BURST_POST_DEP` / `BURST_POST_ARR` | 80 / 60 / 60 | burst window and splits. **Single-sourced in `movement_service.h`** — they were duplicated once and the copies diverged |
| Timer1 | CTC, /64, `OCR1A`=624 | 25.000 Hz |

### Battery

| constant | value | meaning |
|---|---|---|
| `BATTERY_LOW_THRESHOLD` / `_CLEAR_` | 1600 / 1750 | raw ADC counts, **not millivolts**; hysteresis |
| `BATTERY_SETTLE_SAMPLES` | 2 | discarded after boot. ⚠️ currently discards *everything* — bug |

### Unarmed / instrumentation only

| constant | value | meaning |
|---|---|---|
| `VEL_ARMED` | 0 | velocity departure path characterised, never armed |
| `VEL_DEPART_THRESHOLD` | 2 × 0.1274 | = 50 fpm at this sample rate; the requirement is 20 |
| `XY_STILL_*` | various | lateral calibration; release path deleted |

---

## 10. Traps that have cost real time

- **A capture spans reflashes** and `millis()` restarts at each one. Mixing
  sections has produced entirely false results twice. Split on `Device Booted`;
  the parser now warns.
- **FSM log lines carry no timestamp.** Anchor them to the *next* sample.
- **Confirm the flash took** — by size when it changes, by a boot banner when
  it does not.
- **The USB-serial cable back-powers the board** through the RX clamp. Pulling
  batteries with it attached is not a power cycle.
- **The serial monitor drops its handle** regularly. If the device is then
  silent, an avrdude signature read revives it.
- **The programmer's COM port changes** on almost every reconnect.
- **Do not trust a single run.** Four conclusions were overturned by the next
  measurement on 08-10/11, and three more on 08-12 — including two of the
  author's own about the arming gate. A single-run number is a hypothesis.
