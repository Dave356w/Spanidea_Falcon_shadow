# Falcon — product behaviour, detection logic, and reference legend

**As of 2026-08-13, firmware `0c4f5ab`, branch `Falcon_Rel_EFT`.**

Brought up to date 2026-08-13. Changes folded in since 2026-08-11:

From `falcon_ramp_armed_2026-08-12.md`:

- **The ramp detector is ARMED** (`RAMP_ARMED 1`) — §5.2, §9.
- **Arming is now TWO SEPARATE GATES, one per detector** — §5.3. This is a
  safety property, not a refactor; read it before touching arming.
- **Wire1 is vendored** into `falcon_srcs/lib/Wire1` with bounded TWI waits —
  §2, §7, §9. Adds `tw=` to the sample line.
- **New log fields `g=`, `ro=`, `tw=`** — §8.
- **§8a: the method for tuning a threshold by replaying logged bursts**,
  including the two mistakes that make a replay silently wrong.

From `falcon_500fpm_ui_2026-08-13.md`:

- **Flash is no longer the constraint** — 1594 free, and the bma4 swap is worth
  ~1500 bytes rather than 4.9 KB. §2 corrects both.
- **Serial is 62500**, not 9600 — §2.
- **The alarm is sequence-aligned**: one 150 ms blast per 8-LED chase, driven
  from one master counter with firmware pulsing MR. Quiet fraction is now 0.75,
  which has history — see `alarm.h` before touching it.
- **500 fpm tested** (the fastest car so far) and **cruise peak varies ~2×
  between identical runs**, which retires that question — §6, §9.

This is the orientation document: what the device does, how it decides, and
what every name in the source means. It is written to be read *before* the
session notes, which assume all of this.

Session notes, chronological:
`falcon_analysis_2026-08-06.md` → `falcon_state_of_project_2026-08-07.md` →
`falcon_spec_primary_usecase_2026-08-09.md` → `falcon_zxy_bench_2026-08-10.md`
→ `falcon_25hz_arrival_2026-08-10.md` → `falcon_signature_2026-08-11.md` →
`falcon_jog_verdict_2026-08-11.md` → `falcon_350fpm_automatic_2026-08-11.md` →
`falcon_cartop_2026-08-12.md` → `falcon_cab_automatic_2026-08-12.md` →
`falcon_ramp_armed_2026-08-12.md` → `falcon_500fpm_ui_2026-08-13.md`.

**Summary of where things stand:** `falcon_state_of_project_2026-08-13.md`
(supersedes the 2026-08-07 one).

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
Serial logging at **62500 baud** (raised from 9600 on 2026-08-13; 115200 is
unreachable at 1 MHz — see §8a of `falcon_500fpm_ui_2026-08-13.md`).

**Flash: 30662 / 32256, 1594 free.** No longer the binding constraint. It was
30 bytes free on 2026-08-12; 1636 bytes were reclaimed by deleting the
preinstantiated `TwoWire Wire` for **TWI0, a bus with nothing connected to it**
(the DPS310 is depopulated). Upstream passes the whole TWI0 driver to that
constructor as function pointers, so `--gc-sections` could never drop it —
including its 600-byte ISR. `lib/Wire` is vendored for this, with
`utility/twi.c` deliberately absent.

⚠️ **Two corrections to older guidance in this document's history:** the TWI
timeout fix did NOT need that space — it was solved for 124 bytes by bounding
the waits in a vendored `Wire1` (§7, §9). And **the bma4 driver swap recovers
~1500 bytes, not 4.9 KB**: `bma456_config_file` is 6144 of those bytes and is
the Bosch feature-engine blob any-motion depends on, so it cannot go, and what
remains is code actually in use on the primary departure detector. Do not plan
around 4.9 KB.

**586 bytes remain available** in `RollingAvg.h`, which is the last dynamic
allocation on the device.

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
| 3 | **ramp detector** | sustained one-signed deceleration | **ARMED** (2026-08-12), on its own gate — §5.3. ⚠️ 14 latches and the armed branch has never executed: the FSM's ramp check sits in `STATE_MOVING` and the peak always crosses first |
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

### 5.3 Arming — TWO SEPARATE GATES, one per detector

Neither the peak collector nor the ramp accumulator runs until its gate opens.
Without this, the departure transient would satisfy the arrival test a fraction
of a second after setting the latch.

**⚠️ They were a single shared gate until 2026-08-12. They are not any more,
and the distinction is a safety property — see the exposure below.**

**The PEAK collector — gate `arr_armed`, two paths, either sufficient:**

- **quiet** — `ARRIVAL_ARM_SAMPLES` consecutive samples inside
  `ARRIVAL_QUIET_MSS`. Works whenever a run has cruise.
- **sign reversal** — `ARM_REV_SAMPLES` consecutive samples opposite in sign to
  this run's departure. Physics guarantees the sign: a car sheds exactly the
  velocity it gained. Needs no lingering, so it survives runs with no cruise.

`arm_via` records which path opened it (`v=1` quiet, `v=2` reversal). **`v=2`
has never fired in ~29 live runs** — quiet always wins the race.

**The RAMP accumulator — gate `ramp_gate`, reversal ONLY.** An independent
counter (`ramp_opp`), deliberately not `arr_armed` and deliberately not
`arm_via == 2`. Reported as `g=`/`ro=`.

**🔴 WHY THE RAMP CANNOT SHARE THE PEAK'S GATE. A slow departure starts BELOW
the quiet band.** From the departure latch, in a real logged burst:

```
-65 -38 -56 -64 -129 | -142 -170 -228 -301 -488 -496 -506 ...
\___ all inside the 150 mm/s² quiet band ___/
```

The quiet path arms on the departure's own opening samples, and from that
instant the ramp accumulator is fed **the departure ramp**, which qualifies at
mean 499 / directionality 100% — indistinguishable from an arrival. With
`RAMP_ARMED 1` that releases the latch seconds after a car starts moving.
**1–2 departures in 89 on logged data.** Latent while unarmed; arming it in
`7210dfd` made it reachable, and `74e2f1c` closed it.

**Why not `arm_via == 2`:** it records whichever path armed *first*, and the
reversal block sits inside `if (!arr_armed …)`, so once quiet arms, `arr_opp`
stops advancing and `arm_via` can never become 2. Since quiet always wins,
gating on it would make the ramp detector **permanently dead**.

**🔴 Both gates are load-bearing.** *Departure* ramps satisfy the ramp block
test as readily as arrivals — 47/89 (52%) of departure bursts on file qualify
the ungated test. The arithmetic cannot tell them apart: same shape, opposite
sign, and sign encodes direction of travel, not phase. **Re-run
`graph/arming_replay.py` after any change to arming or to the ramp constants.**

**⚠️ `MOVEMENT_DETECTION_TIMEOUT_MS` used to be silently load-bearing.** The old
shared gate was safe only if arming began ≥200 ms after the latch, and that
constant is exactly 200 — but it is a **ceiling, not a floor**, because
`STATE_MOVEMENT_DETECTED` exits early whenever `|w| > |vel_departure|` and
`vel_departure` logs as `0.000` every run. `ramp_gate` does not depend on that
coincidence. Do not re-couple them.

**🔴 Arming margin is an install-time property.** Two mountings in the same car
minutes apart:

```
              2→1 down        1→2 up
mounting A    9, 8, 9         8, 8
mounting B    6, 5, 5, 6      10, 11         (need = 5)
mounting C    6, 6, 9, 9      6, 8, 13, 15
```

On mounting B the descent arms on the *last possible sample*. One more sample
lost to piezo blanking and neither detector runs at all — which produced an
85 s beacon over a car stationary for 78 of them. **A mechanic's placement
decides whether the beacon releases at the bottom terminal.**

**⚠️ And the margin is NOT stable within one mounting.** Mounting C spans 6–15
across eight bottom-terminal runs, with parked attitude drifting `x` 0.61→0.64
between sets. Down is both thinner and more stable (6–9) than up (6–15). **So a
mounting's margin cannot be characterised from three runs, which rules out a
commissioning spot-check as the mitigation** — the device should measure and
report its own margin. A claim that this mounting was "6 down / 8 up and
reproducible" was made and withdrawn the same day on the next two runs.

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
AND guard.** Live and armed — **25/25 lifetime** ⚠️ *(that tally is off by
one: `260811-152209.log` carries a fourth `JOGV` line the 08-11 note never
tabulated — an uncounted RUN verdict. And "lifetime" counts verdicts the
firmware issued, not correctness: scored against labels 2026-08-20 the verdict
runs **1 false JOG in 63 real departures and 1 miss in 17 jogs** —
`falcon_corpus_labelled_2026-08-20.md` §4.1.)*

---

## 6. Where the product stands

**Inspection operation works.** Departures caught to **17 fpm** (the slowest
ever measured), arrivals released with 2–5× margin, jogs verdicted 25/25.

**Automatic operation releases on luck.** Thirty-one automatic stops measured:

```
arrival peak   0.454 ... 0.713      gate 0.45      worst margin 1.009×
```

This is not a distribution with a thin tail — it is centred on its own
threshold. The cause is structural: the peak is a deviation from a rolling
average, and a sustained deceleration ramp drags the average with it, so the
metric gets *weaker* as ramps get longer (faster cars, taller buildings).

**The ramp detector answers this and is now ARMED.** Twenty-eight automatic
stops:

```
ramp mean        470 ... 653         floor 300
directionality   100% on every one   gate 85%
```

against complete negative evidence from inspection operation — brake stops,
jogs, cruise, departures, all declined. One stop measured both at once: **peak
1.016×, ramp 2.18×, same signal, same 3.9 s.** It is the strongest
discriminator this device has produced.

**⚠️ But arming it has changed nothing observable.** Eleven latches since, and
the armed branch has **never executed** — every verdict landed after the peak
or any-motion path had already released, because the FSM's ramp check sits
inside `STATE_MOVING` and the peak crosses on the ramp's leading edge. It is
insurance against a stop the peak misses, and no stop has missed yet. Arming it
also opened a real departure-ramp hole that took a second change to close
(§5.3).

**Three release paths are armed; only one has live evidence.** The jog verdict
has fired correctly 29/29. The ramp detector has never executed. The reversal
arming path (`v=2`) has never fired in ~29 runs. The latter two are armed on
replay plus negative evidence alone, and they carry the automatic-operation
boundary between them.

**Open, in priority order:**

1. ~~bma4 driver swap~~ — **DESCOPED 2026-08-13.** It blocks nothing: flash is
   at 1594 free after 1636 bytes were reclaimed from the unused TWI0 bus (§2),
   and the swap is worth only ~1500 bytes of code that is **actually in use** on
   the primary departure detector. `bma456_config_file` is 6144 of those bytes
   and is the any-motion feature blob, which cannot go. **Do not plan around
   4.9 KB.** 586 bytes remain cheaply available in `RollingAvg.h`.
2. **Brownout** — the rail sags under sustained piezo load and the board dies
   ~243 s into an alarm, over 300 ADC counts *above* the low-battery trip.
   Tabled by Dave 2026-08-12; the unblocking step is alarm current vs pack
   voltage **with a meter**, not the on-board ADC.
3. **The arming blind spot on a single-floor terminal approach** — departure
   ramp straight into deceleration, no cruise, no quiet samples, so the peak's
   gate never opens and `pk` reads 0.00 through a real excursion. `ramp_gate`
   does not fix this; it fixes the opposite direction. Wants the §8a method
   against every burst on file before any firmware change.
4. **A stop the peak misses**, to exercise the armed ramp branch even once.
   Needs an automatic stop under 0.45 — none of 31 has been.
5. **~20% snapshot overrun** at 25 Hz. Every threshold here stands on 80% of
   the data.
6. **The device should report its own arming margin.** Margin is an install-time
   property that varies 6–15 within one mounting (§5.3), so a commissioning
   spot-check cannot establish it.

**Closed 2026-08-12:**

- **TWI lockup** — `Wire1` vendored with bounded waits, 124 bytes instead of
  1554. Three wedges caught and recovered, zero resets since. Previously this
  froze the device until the watchdog fired, and on 2026-08-12 a pair of them
  **cost an entire car run** — the beacon never sounded while a counterweight
  moved, which is the only genuinely catastrophic failure mode this product has.
- **Battery telemetry** — not broken after all. It reports normally after
  roughly 23 minutes of uptime; the settling logic is just very slow to
  converge. Worth knowing before anyone rewrites it.

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
| `adc.cpp` | internal ADC for battery sense. ⚠️ note `MCP3208 adc` in `main.cpp` also reads `BATT_SENSE` over SPI — **two battery paths exist** |
| `RollingAvg.h` | fixed-size rolling average |
| `lib/Wire1/` | **VENDORED MiniCore Wire1** — every TWI wait loop bounded by a 16-bit spin counter, because the sample read runs in the ISR and a wedge freezes the device. `twi1_guard_trips1()` exposes the trip count (`tw=`). Rollback: `TWI1_GUARD_SPINS 0`. Do not replace with the framework copy |
| `graph/parse_falcon_log.py` | log analysis: per-run summaries, arming/late-release/multi-boot checks |
| `graph/arming_replay.py` | **replays candidate arming rules against every burst on file** (186: 89 dep, 88 arr), mirroring the ISR arithmetic. `--sweep` sweeps `ARM_REV_SAMPLES`. **Run it after any change to arming or the ramp constants.** Read its header: departure-burst safety is sound, arrival capability is PARTIAL because bursts do not span whole runs |
| `graph/velocity_replay.py` | velocity-path replay |

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
| `ARM q= a= v= g= ro=` | per-run arming summary. `q`/`a`/`v` are the **peak's** gate: longest quiet stretch, armed flag, **`v=1` quiet / `v=2` sign reversal**. `g`/`ro` are the **ramp's** independent gate: `g=1` the departure ramp was seen to end, so a `RAMP` verdict this run is trustworthy; `g=0` the ramp detector never ran at all whatever the peak did. **`ro` is the true reversal-stretch length** (high-water, frozen when the stretch breaks) — read as margin against `ARM_REV_SAMPLES`: `ro=8` opened on the last possible sample, `ro=24` had 3× headroom. ⚠️ `ro=8` in logs before `3993abb` is a capped instrument, not a margin |
| `FSM: Arrival (…)` | which path released the latch |
| `FSM: FAILSAFE` | no arrival was ever detected — a fault |

### Key internal variables

| name | where | meaning |
|---|---|---|
| `arr_zero` | ISR | zero calibration the peak measures against |
| `arr_armed` / `arm_via` | ISR | the arming gate and which path opened it |
| `arr_quiet` / `arr_quiet_hi` | ISR | current and longest quiet run; `hi` freezes when the arming stretch breaks |
| `arr_dep_sign` | ISR | this run's departure sign, fixed over `ARM_DEP_SIGN_N` samples. **Nothing reversal-gated can arm before 25 samples** |
| `arr_opp` | ISR | consecutive opposite-signed samples, for the **peak's** gate. Stops advancing once `arr_armed` — hence `arm_via` can never become 2 after quiet wins |
| `ramp_gate` | ISR | **the ramp's own gate — reversal only, NOT `arr_armed`.** §5.3 |
| `ramp_opp` | ISR | its reversal counter. Keeps advancing past the gate opening, deliberately |
| `ramp_opp_hi` / `ramp_o_frozen` | ISR | true stretch length and its freeze flag, same discipline as `arr_quiet_hi`. **Reported as `ro=`; the first version stopped at the gate and measured nothing** |
| `twi1_guard_trip_count` | Wire1 | saturating count of wedged TWI transactions caught. Reported as `tw=` |
| `arr_peak_cur/prev` | ISR | the two sliding-window buckets |
| `arr_hit` | ISR | sticky: the peak crossed the gate this run |
| `ramp_sum/abs/n/run/sign` | ISR | ramp block accumulator |
| `ramp_hit_v` | ISR | sticky ramp verdict |
| `burst[]`, `burst_post`, `burst_ready` | ISR/loop | burst recorder ring and trigger state |
| `vel_departure` | FSM | departure integral (instrumentation) |
| `arrival_seen`, `any_motion_pending`, `jog_release_pending` | FSM | per-run flags |
| `boot_mcusr` | `.noinit` | reset cause, captured before constructors |

---

## 8a. Method — tuning a threshold by replaying logged bursts

**Every threshold in §9 should be movable without touching the car.** The device
already dumps full-rate evidence; the job is to replay it faithfully. This is
the procedure that produced the `ramp_gate` change and the `ARM_REV_SAMPLES`
sweep, written down so the next threshold does not need the method reinvented.

### The data

`BURST k=dep|arr pre=20 n=80 signed_mmss=…` is **80 consecutive samples at the
full 25 Hz**, in milli-m/s², signed against the zero calibration, oldest first,
with a trigger at index `pre`. This is the only full-rate data that exists off
the device — the `t=` sample lines are decimated to ~11% and **cannot** be used
to reconstruct a detector's input. 186 bursts are on file.

### The procedure

1. **Mirror the arithmetic exactly, from the source, not from the note.** Copy
   the constants and the update order out of the ISR. `graph/arming_replay.py`
   reimplements the block accumulator and both arming counters; if the firmware
   changes, the mirror is stale and the replay is fiction.
2. **Establish where the RUN starts inside the burst.** This is the step that
   silently invalidates everything (see the two failures below).
3. **Test the safety direction first.** For an arrival detector that means: does
   the rule fire on a *departure*? A rule that fails here is disqualified
   regardless of how well it detects arrivals.
4. **Use a criterion the window can actually support.** "The ramp did not
   complete inside the burst" is too weak — a burst ends 2.4 s after the
   trigger and a fast departure ramp outlasts it. The usable criterion is
   **ARMED-INTO-RAMP**: did the gate open while the departure ramp was still
   running? From that instant the accumulator is being fed the departure,
   whether or not the window is long enough to show it.
5. **Sweep the constant and read the whole curve**, not the pass/fail at the
   incumbent value. The sweep is what showed 8 is not the optimum for safety
   and that the fix had to be structural instead.
6. **Sanity-check against live behaviour before believing anything.** A replay
   result that contradicts what the device demonstrably does is wrong until
   proven otherwise. This is the single highest-value step.
7. **State what the method cannot answer.** Bursts do not span whole runs, so
   arrival-side arming is *partial* evidence: the counters run across unlogged
   cruise between the departure and arrival bursts. "Would have armed" in replay
   is not "would have armed in service".

### Two failures of step 2, both real, both from one afternoon

- **Replaying a departure burst from sample 0** arms instantly on the 20
  **pre-trigger parked samples**, which are quiet by definition. This produced
  the conclusion that the shipping firmware fires the ramp on 53% of departures.
- **`burst_trigger()` fires at the departure latch, but the arming counters are
  cleared by `arrival_peak_reset()` on `STATE_MOVING` entry** — later, and by a
  variable amount. Correcting this dropped the exposure from 47/89 to 1–2/89.

Both were caught by step 6, not by inspection. **The corrected finding was still
real** — which is why step 6 must reject wrong results without discarding the
investigation.

### What replay cannot replace

A threshold that depends on **timing between states**, on the piezo blanking
samples, or on anything outside the 80-sample window still needs the car. The
`ro=` margin on a short run is an example: replay said the reversal gate would
hold, and it took two live single-floor runs to confirm 1.5–1.75×.

*Method and `graph/arming_replay.py` added 2026-08-12 by Claude (Opus 5),
working from Dave's logged captures; the findings in §5.3 and the corrections in
`falcon_ramp_armed_2026-08-12.md` §11 came out of it.*

---

## 9. Threshold legend

**⚠️ Every value below is a measurement or a decision, not a default.** The
reasoning for each lives in a comment at its `#define`. Read that before
changing one.

### Arrival detection

| constant | value | meaning |
|---|---|---|
| `ARRIVAL_PEAK_VALUE` | **0.45** | raw windowed peak that declares an arrival. **31 automatic stops now land at 0.454–0.713, i.e. a distribution centred on its own gate** — worst margin 1.009×, and 1.016× measured on a stop whose ramp simultaneously cleared 2.18×. This is the weakest number in the product |
| `ARRIVAL_PEAK_WINDOW_MS` | 1000 | sliding window (two buckets → 1–2 s effective) |
| `ARRIVAL_QUIET_MSS` | 0.15 | what counts as quiet, for arming. ⚠️ **A slow departure starts BELOW this band** — the mechanism behind §5.3's exposure |
| `ARRIVAL_ARM_SAMPLES` | 5 | consecutive quiet samples to arm the **peak**. Measured margin: 123–235 with cruise, **6–15 at the bottom terminal and NOT stable within a mounting**, 5–6 on an unfavourable one |
| `ARM_REV_SAMPLES` | **8** | consecutive opposite-signed samples. Gates the peak's second path *and* (independently) the ramp. Replay sweep 2026-08-12 over 89 departure bursts: **≥13 never arms into a departure ramp at any offset; 3–12 leaves 1–4 such cases**, while arrival coverage falls slowly (85/88 at 3 → 57/88 at 13). 8 is the incumbent, not the sweep optimum — **left alone deliberately** because `ramp_gate` removes the hazard the sweep was measuring, and moving it would change the peak's behaviour too. Measured live at `ro=` 12–14 on short runs (1.5–1.75× margin) |
| `ARM_DEP_SIGN_N` | 25 | samples used to fix the departure's sign |
| `ARRIVAL_EDGE_COUNT` / `_WINDOW_MS` | 2 / 2500 | any-motion clustering |
| `MIN_TRAVEL_MS` | 3000 | how long before arrival tests are allowed |

### Ramp detector (ARMED 2026-08-12)

| constant | value | meaning |
|---|---|---|
| `RAMP_ARMED` | **1** | ✅ releases the latch on a ramp verdict. Armed on 17/17 automatic stops with zero false latches. **Gated on `ramp_gate`, not `arr_armed` — §5.3.** ⚠️ 14 latches since arming (including 3 at 500 fpm, means 597-602) and the armed branch has **never executed**: every one landed after the peak or any-motion path had already released, because the FSM's ramp check sits inside `STATE_MOVING`. It is insurance with no live proof. Rollback on a false release: set 0 |
| `TWI1_GUARD_SPINS` | **8000** | *(lib/Wire1)* spin bound per TWI wait site. ⚠️ **A SPIN COUNT, NOT A TIME** — does not track F_CPU or the TWI rate. ~50 ms at 1 MHz against an ~11.5 ms transaction, deliberately far below the 2 s watchdog. 3 trips caught in one session, zero resets. Rollback: 0 |
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
| `JOG_OPP_PEAK_MMSS` | **900** | opposite-side peak — **carries the verdict.** ⚠️ corrected 2026-08-20 against the labelled corpus: real departures **11–562 plus one at 972**, jogs **381–4154** — the populations OVERLAP and the old "≤440 vs ≥1366" gap does not exist |

### Timing and release

| constant | value | meaning |
|---|---|---|
| `LATCH_FAILSAFE_MS` | **240000** | ⚠️ a placeholder around the brownout, not a tuning value — the hardware dies at ~243 s |
| `STOP_CONFIRM_MS` | 5000 | continuous quiet before silencing |
| `STOP_BAND_VALUE` | 0.10 | what "stopped" means to the confirm timer |
| `MONITOR_REARM_MS` | 6000 | deafness after a release, so arrival ringing cannot re-latch |
| `MOVEMENT_DETECTION_TIMEOUT_MS` | 200 | dwell before the beacon sounds. ⚠️ **A CEILING, NOT A FLOOR** — `STATE_MOVEMENT_DETECTED` exits early whenever `\|w\| > \|vel_departure\|`, and `vel_departure` logs as `0.000` on every run, so the real dwell is usually much shorter. Until `74e2f1c` the ramp detector's safety silently depended on this being ≥200 ms; §5.3. Do not treat it as a guaranteed delay |
| `CALIB_TIMEOUT_MS` / `CALIB_RETRIES` | **6000** / 2 | calibration window and retries (10000 -> 6000 on 2026-08-14; `XY_CALIB_BUCKETS` 10 -> 6 and `XY_CALIB_MIN_BUCKETS` 6 -> 4 with it) |

### Sensor and sampling

| constant | value | meaning |
|---|---|---|
| `ANYMOTION_THRESHOLD` | 32 | sensor units. 2.2× a measured departure, 0.58× cruise vibration |
| `ANYMOTION_DURATION` | 5 | 50 Hz samples = 100 ms sustain |
| `ACC_INT_POLL_MS` / `ACC_INT_STUCK_POLLS` | 1000 / 8 | status poll; re-arm if stuck |
| `SAMPLE_RING_N` | 8 | sample ring depth |
| `LOG_DECIMATE_N` | 8 | print 1 line in 8. Was set when the port ran at 9600; at 62500 there is headroom to decimate less, but the ISR read (11.4 ms of every 40 ms, 28% of CPU) is now the binding cost, not the port |
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
