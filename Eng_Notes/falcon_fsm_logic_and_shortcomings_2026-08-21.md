# The FSM, spelled out — every transition, every shortcoming, and what to do

**2026-08-21. Desk work on `Falcon_Rel_EFT` at `345613f`. No car, no bench.**

Written to answer one request: *state the FSM logic plainly, state what is wrong
with it, and propose fixes that keep the functionality that works.*

Nothing here changes firmware. §4 proposes changes; §5 is the one candidate that
was **measured today against the committed corpus** rather than argued, using a
new tool, `graph/coherence_replay.py`, added with this note.

> **Read `falcon_START_HERE_2026-08-20.md` §0a first.** It draws the line
> between what is established and what is not, and this note stays inside it.
> One confirmed release on a moving car, in 44 contract-speed runs, on one
> machine, on one afternoon. The consequence is catastrophic; the rate is not
> established and nothing here says the device is getting worse.

---

# 1. The FSM as implemented

Seven states in `MotionStates` (`movement_service.h`), driven from
`MovementService::fsm_run()` (`movement_service.cpp:332`), called once per
`loop()` pass from `main.cpp:1565`. `st=` in the log is this state.

```
  power on ── device already placed on the counterweight (spec §1)
     │
     ▼
  ┌──────────────────────┐  7 STATE_ERROR_RESET ── boot only
  └──────────┬───────────┘
             ▼
  ┌──────────────────────┐  1 STATE_CALIBERATION      cpp:338
  │  6 s: learn z zero,  │  exit: CALIB_TIMEOUT_MS elapsed
  │  XY_STILL, z spread  │  an any-motion edge in the window sets calib_moved
  └──────────┬───────────┘  → retry up to CALIB_RETRIES, then READY (fallback)
             ▼                1 chirp = good · 3 chirps = degraded
  ┌──────────────────────┐  2 STATE_MONITORING        cpp:556  ◄────────────┐
  │  parked; three       │                                                  │
  │  departure detectors │  exit on ANY of:                                 │
  │  OR'd, never AND'd   │    · any-motion edge          (latch_path 1)     │
  │  re-arm blanked      │    · polled |avg−zero| > thr  (latch_path 2)     │
  └──────────┬───────────┘    · velocity integral        (latch_path 3,     │
             ▼                                            VEL_ARMED 0)      │
  ┌──────────────────────┐  3 STATE_MOVEMENT_DETECTED cpp:747               │
  │  200 ms dwell        │  exit: MOVEMENT_DETECTION_TIMEOUT_MS             │
  │  B1 prints the latch │  → enable_alarm(), enable_chase_leds()           │
  └──────────┬───────────┘                                                  │
             ▼                                                              │
  ┌──────────────────────┐  4 STATE_MOVING            cpp:814               │
  │  BEACON ON, LATCHED  │  exit on ANY of (all → DECELERATING):            │
  │  no timeout: a 3 min │    · jog verdict            cpp:888              │
  │  run alarms 3 min    │    · any-motion cluster AND peak      cpp:916    │
  │                      │    · windowed raw peak                cpp:942    │
  │  arrival tests are   │    · ramp verdict (RAMP_ARMED 1)      cpp:965    │
  │  blind until         │    · LATCH_FAILSAFE_MS 600 s — logged as a fault │
  │  MIN_TRAVEL_MS       │                                       cpp:999    │
  └──────────┬───────────┘                                                  │
             ▼                                                              │
  ┌──────────────────────┐  5 STATE_DECELERATING      cpp:1005              │
  │  BEACON STILL ON.    │  silence requires STOP_CONFIRM_MS of CONTINUOUS  │
  │  ⭐ the only place   │  |avg − zero| ≤ STOP_BAND_VALUE; any excursion   │
  │  disable_alarm() is  │  restarts the window                cpp:1053     │
  │  reached in flight   │  backstop: LATCH_FAILSAFE_MS        cpp:1063     │
  └──────────┬───────────┘                                                  │
             ▼                                                              │
  ┌──────────────────────┐  6 STATE_STOPPED           cpp:1071              │
  │  refill the average  │  → MONITORING, MONITOR_REARM_MS of deafness ─────┘
  └──────────────────────┘
```

**The single most useful structural fact about this FSM:** `disable_alarm()` is
reached from exactly two lines in flight — `movement_service.cpp:1054` (confirm
satisfied) and `:1065` (the DECELERATING failsafe). *Every* release path — jog,
cluster, peak, ramp, failsafe — funnels through `STATE_DECELERATING` first. Any
condition added there applies to all of them at once. §4.2 uses this.

## 1.1 Departure — three detectors, OR'd, never AND'd

`main.cpp` §14.4 makes a missed departure the only catastrophic failure, so an
extra detector may only ever make the unit alarm *sooner*.

| # | detector | where | state |
|---|---|---|---|
| 1 | BMA456 **any-motion**, 100 Hz on the sensor's own engine | `notify_any_motion()` cpp:202 | live; the only one that catches slow departures (caught an 18 fpm departure at −0.116 m/s²) |
| 2 | **polled** `\|avg − zero\| > threshold_value`, self-calibrated | cpp:644 | live; threshold learned at calibration, clamped `[0.040, 0.200]` |
| 3 | **velocity integral** over a 5 s window | cpp:712 | ⛔ `VEL_ARMED 0` — observation only |

Both live paths are re-arm blanked (`MONITOR_REARM_MS` 6000, cleared early once
the lateral measures settled). The polled path was blanked only on 2026-08-20;
before that two bench taps produced nine alarm cycles in 90 s.

`latch_path` is **evaluation order, not detection order** — the polled test runs
first and is first-setter-wins, so a run logged `(polled)` may also have been
caught by any-motion.

## 1.2 Arming — two gates, deliberately not one

Neither arrival collector runs until **its own** gate opens. Without this the
departure transient satisfies the arrival test a fraction of a second after the
latch.

| collector | gate | opens on |
|---|---|---|
| windowed peak | `arr_armed` | `ARRIVAL_ARM_SAMPLES` 5 quiet samples **OR** `ARM_REV_SAMPLES` 15 sign reversals (`arm_via` 1 or 2) |
| ramp accumulator | `ramp_gate` | **reversal only**, independent counter |

They are separate because the block test cannot tell a departure ramp from an
arrival ramp — same shape, opposite sign, and sign encodes *direction of travel*,
not *phase*. A slow departure starts *below* the quiet band, so quiet can arm on
the departure's own opening samples and hand the departure ramp to the
accumulator. `main.cpp:463` has the burst that forced this.

## 1.3 Arrival — three paths to the same exit

Detection is on the **raw sample**, not the 32-sample average: a 0.938 m/s²
brake bounce registers 0.0072 on the average.

| # | path | fires on |
|---|---|---|
| 1 | clustered any-motion **AND** peak | ≥2 quiet edges in 2.5 s, ANDed with `arrival_peak_hit()` |
| 2 | windowed raw peak | `arr_peak` > `ARRIVAL_PEAK_VALUE` 0.45, sticky for the run |
| 3 | ramp | 3 consecutive 12-sample blocks, mean ≥ 300 mm/s², directionality ≥ 85%, same sign |
| — | jog verdict | `ratio ≥ 33%` **AND** `opk ≥ 900` on the departure burst |
| — | failsafe | `LATCH_FAILSAFE_MS` 600 s, logged as a fault |

The peak is **windowed** (two alternating 1 s buckets) because a running max
ratchets through cruise and crosses any threshold eventually — and **sticky**,
because a jog's stop crossed before `MIN_TRAVEL_MS` opened the gate and was
discarded unused.

## 1.4 Release confirmation

Reaching `STATE_DECELERATING` silences nothing. `|avg − zero|` must stay inside
`STOP_BAND_VALUE` 0.10 for `STOP_CONFIRM_MS` 5000 **continuously**. This holds
the beacon through levelling, which would otherwise read as stopped and let the
subsequent brake set re-latch as a fresh departure.

---

# 2. What is wrong with it

Ordered by consequence, not by ease.

## S1 🔴 The release confirmation cannot tell cruise from stopped

**This is the one that matters.** On 2026-08-20 the beacon released 5.4 s into a
20 s contract-speed run and stayed silent for eleven seconds of confirmed
travel (`falcon_false_release_2026-08-20.md`).

Two independent failures stack, and both are visible in the code:

1. **The arrival gate fired on a cruise transient.** Peak 0.472, inside the real
   arrival population 0.450–0.511. Retuning is refuted — raise the gate and four
   of six real arrivals are rejected.
2. **The stop confirmation then passed.** `STATE_DECELERATING` tests
   `|avg − zero| ≤ 0.10` on the **vertical** channel — and at constant velocity
   an accelerometer reads 1 g, exactly as it does parked. The capture shows the
   vertical average at 9.774 while the car was moving, against a zero of ~9.70:
   **0.07, inside the 0.10 band.** The 5 s confirm was never going to catch this.

The device had the evidence and did not use it. The **lateral** channel read
0.205–0.884 through those eleven seconds against 0.005–0.05 at rest, and
`lat_monitor.quiet_run()` is already computed, already logged as `q=`, and
already trusted to end the re-arm blank.

## S2 🔴 Silence-while-moving is reachable from three other paths, unguarded

The same confirmation gap is the last line of defence for every release path,
and three of them have measured error rates in the catastrophic direction:

- **Jog verdict** — populations overlap (real 11–562 plus one at 972, jogs
  381–4154). Measured **1 false JOG in 63 labelled real departures**: a genuine
  run silenced ~4 s after the latch. `JOG_VERDICT_ARMED` is 1. The comment at
  `main.cpp:672` says "if a false JOG ever releases a moving car, disarm first" —
  **it is on file** (`260818-105317.log` r44).
- **Ramp** — `RAMP_ARMED` 1, and the accumulator is one arming bug away from the
  departure ramp. `ARM_REV_SAMPLES` 8 → 15 closed the measured INTO-RAMP hazard
  (5/227 → 0/227), which is the only thing holding it.
- **Peak** — S1.

## S3 🟠 The peak collector arms on the minimum, into cruise

`ARRIVAL_QUIET_MSS` is 0.15 against measured cruise of 0.27–0.29 on the vertical
channel. The gate is starved on its own channel, and on **8 of 32**
contract-speed runs it armed at exactly `q=5`, the minimum. The false release
armed at `q=5`, mid-travel, and the first cruise transient over 0.45 released it.
Arming earlier than the design intends is what exposed the gate to cruise.

## S4 🟠 The single-floor blind spot

A terminal approach with no cruise supplies no consecutive quiet samples, so
`arr_armed` never opens and **both** collectors stay off for the whole run.
Reproducible; produced a beacon sounding 85 s over a car stationary for 78 of
them. Half-closed: `arm_via = 2` has now fired three times, all clean releases,
all with `q` of 2 or 4 — below the quiet minimum, i.e. **the insurance path fires
in exactly the condition that defines the defect**.

## S5 🟠 `MIN_TRAVEL_MS` does not clear the departure ramp at inspection speed

3000 ms, and at 19 fpm the ramp is not clear until +5 to +8 s. The arrival gate
therefore opens into a still-decaying transient; one run armed on a lull inside
it (`ARM q=6`, then the stretch broke). Adequate at contract speed. Also a trap
for measurement: a cruise ceiling read at +3 s is a ramp measurement wearing a
cruise label.

## S6 🟠 Calibration can be poisoned silently, in the dangerous direction

`Z_THRESH_MARGIN` is 1.5× the *measured rest spread*, so a window taken before
the installation has physically settled produces a **higher** threshold — a
deader backstop. Measured 2026-08-19: the one calibration on file that ever
cleared the clamp was the noisiest window, taken straight after handling.
`mv=` read 0 on all three; nothing catches it. `Z_CAL_SETTLE_MS` covers the
*filter's* convergence, not the *installation's*, and the two have been
conflated.

## S7 🟡 A false latch is a ten-minute alarm

`LATCH_FAILSAFE_MS` 600 s, raised from 240 s deliberately on the asymmetry that
a position lie beats silence over a moving car. Accepted, but it is the cost
that any "hold longer" fix is charged against.

## S8 🟡 One of three departure detectors is off, and the 20 fpm requirement is unmet

`VEL_ARMED 0`, and `VEL_DEPART_THRESHOLD` sits above the 20 fpm requirement
anyway (§2.3 of the 08-18 state-of-project puts it at ~22.6 fpm), so arming it
as-is closes nothing.

## S9 🟡 The firmware cannot tell automatic operation from inspection

This is what blocks the one release rule that *does* prevent the false release.
`ramp_veto` scores 0 MOVING in 198 runs and refuses to release on **100% of
inspection stops** (0 ramp verdicts in 79 runs). It is safe on one population and
unusable on the other, and `g=` is not the discriminator —
`falcon_ramp_priority_2026-08-20.md` §5 names this as the open question.

## S10 🟡 Instrumentation and headroom

- **B3**: the polled departure path still emits no burst and no jog verdict. It
  is now the *sensitive* path, so the corpus is biased toward any-motion runs —
  26 bursts against 33 latches in `260820-150000.log`.
- `bz=` on periodic lines is closed (B2); `m=`/`q=` are logged, which is what
  makes §5's replay possible at all.
- ⛔ **THE CORRECTION ABOVE WAS ITSELF WRONG, AND IT BROKE THE BUILD.
  Re-corrected 2026-08-21 afternoon against real `pio run` output.** It said
  `falcon_START_HERE_2026-08-20.md` §1's "80 free" predated the `RollingAvg`
  and `FALCON_LOG` recoveries and promoted `platformio.ini`'s table instead.
  **That is backwards.** The `platformio.ini` table is the stale one: it was
  measured at `b79298c`, and START_HERE §2 records the **626 bytes B1 spent
  afterwards** on latch-attribution instrumentation. 706 − 626 = 80, which is
  exactly what §1 says. A baseline build of `345613f` on the bench machine
  reproduces START_HERE to the byte on all three environments — **32176 /
  31706 / 25590, i.e. 80 / 550 / 6666 free.**

  The consequence is in §4.2. **`ATmega328PB` overflowed by 498 bytes,
  `production` by 72 and `bench_battery` by 184** — three of five environments
  did not build, including the one that ships and the one every test session
  must use. Recovered in `ce28d9d`, out of the log plumbing rather than out of
  §4.2, which ships unchanged: **32112 / 31746 / 31838, i.e. 144 / 510 / 418
  free**, and RAM 1505 → 1493, flashed and verified on the bench. Full
  account: `falcon_flash_budget_2026-08-21.md`.

  ⚠️ **The method failure is the transferable part.** Both wrong numbers came
  from reading a committed table instead of running the compiler, and the cost
  estimate that sat beside them was *fine* — +588 predicted, +578 and +622
  measured. An estimate is only as good as the budget it is charged against.
  `api.registry.platformio.org` was blocked in the environment that wrote this
  note, so a whole-image build genuinely was not possible there; the error was
  not saying so loudly enough, and treating a committed table as an authority.
  **Build it, on the bench machine, before quoting free space.**

---

# 3. What is already refuted — do not re-derive

| approach | why it is dead |
|---|---|
| retune `ARRIVAL_PEAK_VALUE` | populations overlap; the false release is inside the real one |
| retune `ANYMOTION_THRESHOLD` | a hand bump (0.136) exceeds a real 18 fpm departure (0.116) |
| reversal-only arming as a swap | safe, and misses 51.5% of arrivals outright |
| ramp **priority** | the ramp lands 1115–9798 ms after the peak, 0/105 first; it postpones, into cruise |
| ramp **veto** | 0 MOVING, and NO RELEASE on 100% of inspection stops |
| raise `JOG_OPP_PEAK_MMSS` | the labelled real-run ceiling moved 440 → 562 → 972; fitting a gate to a sample maximum is the refutation |
| a stillness backstop | fired mid-ride on both test runs the day it was added |
| velocity conservation release | a drive ramp integrates to 67% of true speed, a brake shock to 128% |

Every fix below is checked against this table.

---

# 4. Fixes

Each is stated with the property that makes it safe, the functionality it
keeps, and what must be measured before it flies.

## 4.1 ⭐ Corroborate the peak with ONE block of directionality — §5

Addresses **S1**, and mitigates **S3** as a side effect. Measured today; §5 has
the numbers and the tool. This is the only proposal here with evidence behind it.

## 4.2 ⭐ Require lateral stillness before silencing

**✅ IMPLEMENTED 2026-08-21** — `movement_service.h` (`STOP_LATERAL_ARMED`,
`STOP_LATERAL_QUIET`, `STOP_LATERAL_MAX_HOLD_MS`) and the `STATE_DECELERATING`
case in `movement_service.cpp`. Shipped **armed but capped at 60 s**, with
`STOP_LATERAL_ARMED 0` as a one-constant revert that keeps the logging. Read the
`STOP_LATERAL_QUIET` block in the header before changing any of it — the
bench-session watch items are there.

**Measured cost** — `avr-g++ -mmcu=atmega328p -Os`, same flags both sides, the
translation unit before and after:

⛔ **THE `against free` COLUMN BELOW IS WRONG — it uses `platformio.ini`'s stale
table, see S10. The estimate was good; the budget was not.** Struck through and
replaced by the whole-image `pio run` numbers underneath.

| | flash, estimated | ~~against free~~ | ~~spare~~ |
|---|---|---|---|
| `FALCON_LOG` 1 / 2 | **+588 B** | ~~1134 on `production`~~ | ~~546~~ |
| " | " | ~~706 on `ATmega328PB`~~ | ~~118~~ |
| `FALCON_LOG` 0 | **+214 B** | ~~6902 on `production_silent`~~ | ~~6688~~ |

**Measured for real, `pio run` on the bench machine, `345613f` → `4c253b7`:**

| env | before | after | free before | result |
|---|---|---|---|---|
| `ATmega328PB` (FALCON_LOG 2) | 32176 | 32754 | 80 | **498 OVER** ⛔ |
| `production` (FALCON_LOG 1) | 31706 | 32328 | 550 | **72 OVER** ⛔ |
| `bench_battery` | — | 32440 | — | **184 OVER** ⛔ |
| `production_silent` (FALCON_LOG 0) | 25590 | 25842 | 6666 | ok |

So the term costs **+578 on `ATmega328PB` and +622 on `production`** against
the +588 estimated — the estimate was sound to within about 6%. RAM **+6 B**
(`sizeof(MovementService)` 70 → 76), as predicted.

⚠️ **`SL: rel` is 173 bytes** by the stub measurement. Measured whole-image it
is **80–84** — the stub over-counts because it cannot see what the linker
shares. Do not budget from stub numbers.

✅ **Resolved in `ce28d9d`**: 642 bytes recovered from the log plumbing —
`flog_kv`/`flog_kvf`/`flog_kvt` fold 94 label+value pairs, `flog_nl()` replaces
27 separate copies of `F("\r\n")`, 15 duplicated fragments move to shared
`PROGMEM`, `flog_state()` shares one prefix across the six transition lines,
and `flog_fixed()` retires `Print::print(double, int)`. **§4.2 itself was not
touched and ships armed.** `ATmega328PB` 32112 (144 free), `production` 31746
(510 free), `bench_battery` 31838 (418 free), RAM 1505 → 1493. Flashed and
verified. See `falcon_flash_budget_2026-08-21.md`.

⚠️ Indicative, not exact: measured
against a stub `Arduino.h` (`Print` declared but not defined, so the `F()`
strings and the call sequence survive `-Os`) and without the link-time
`-Wl,--relax`. `api.registry.platformio.org` is blocked by this environment's
network policy, so a whole-image `pio run` was not possible. **Build before
flashing.**

Addresses **S1** and **S2** in one place, and it is four lines.

`STATE_DECELERATING` is the single choke point every release path passes
through. Add the lateral channel to the confirmation it already runs:

```c
/* movement_service.cpp, STATE_DECELERATING, at :1049 */
if (delta_accel > STOP_BAND_VALUE ||
    (lat_monitor.armed() && lat_monitor.quiet_run() < STOP_LATERAL_QUIET)) {
    stop_confirm_timer = millis();
}
```

**Why this is not the removed lateral release path.** That one was a *release*:
"lateral says still → silence", and its failure mode is that a low-contrast
installation reports still during a real move and the device goes quiet — the
catastrophic direction, which is why it was measured dead at 1.19× contrast and
removed. This is the **opposite polarity**: lateral says *moving* → **do not
silence**. It can only ever extend an alarm. A dead or low-contrast lateral
channel makes `quiet_run()` climb, the term vanishes, and the behaviour is
exactly today's.

**Why the numbers support it:**

- the false release read `m` 0.205–0.884 for eleven continuous seconds against
  a rest level of 0.005–0.05; every honest release in the corpus reads
  0.018–0.095 in the same window
- the identical predicate already fires reliably after real stops — that is what
  prints `FSM: re-arm blank cleared early, settled at …` inside the 6 s blank
- the metric is `|Δax| + |Δay|` between **consecutive unblanked** samples, and
  `XY_MAX_DT_MS` 1200 refuses to difference across a gap, so a 200 ms buzzer
  blank cannot be credited as quiet. Sampling is already suppressed during the
  blast (`main.cpp:2450`), so the alarm's own vibration is not in the metric.

**Costs, both real:** releases after a genuine stop are delayed by however long
the lateral takes to settle, and an installation whose lateral never settles
holds the beacon to `LATCH_FAILSAFE_MS`. That worst case is **S7's, already
accepted and already bounded** — no new failure mode is introduced. If that is
judged too long, add `STOP_LATERAL_MAX_HOLD_MS` (60 s is more than five times
the 11 s the measured event needed), after which the lateral term is dropped and
logged.

**Before flight — ⚠️ CORRECTED 2026-08-21, and the correction matters.** This
paragraph said to replay it because "every capture carries `m=` and `q=` on the
sample line". **It does not.** `datasets/` holds the *distilled event corpus*;
the periodic sample line is stripped from all but one capture
(`260810-095551.log`, and that is a pre-25 Hz build). The raw captures are
gitignored and live only on the bench machine, so **the release-delay replay
cannot be run against what is committed.**

What *can* be measured offline, and was:

- **`STOP_LATERAL_QUIET` 8 is supported directly.** The same predicate at the
  same value already runs as `MONITOR_REARM_QUIET`, and it prints when it is
  satisfied: **102 occurrences of `re-arm blank cleared early, settled at N ms`
  across nine captures — min 1507, max 5767.**

  ⚠️ **Read the distribution, not the median.** 95 of the 102 sit at or within
  100 ms of `MONITOR_REARM_MIN_MS` 2500, which floors them — those are
  *censored*, and say only "quiet by 2.5 s", not when. The informative subsets
  are the two ends:

  - **19 uncensored**, from the 08-18 captures which predate the floor:
    **1507–1589 ms**. The lateral reached eight consecutive quiet metrics
    within ~1.6 s of beacon-off on every one.
  - **7 above the floor: 2605, 2637, 2768, 2867, 3522, 5079, 5767 ms.** These
    are the cost tail and they are what the added-delay estimate must be built
    on, not the median.

  ⚠️ It also mixes builds — the sub-2500 values can only come from firmware
  without the floor — so this is a pooled observation, not a controlled one.
- **`XY_STILL` is not pinned at its clamp.** 56 calibrations on file learn
  0.0350–0.1920 against clamps of [0.02, 0.40]. (An earlier draft of this note
  confused this with the *Z* threshold, which does clamp to `Z_THRESH_MIN` in
  most calibrations — different constant, S6.)
- ⬜ **What those 102 settles do NOT cover: the piezo was silent in every one
  of them.** They are `STATE_MONITORING` events. This test runs in
  `STATE_DECELERATING` with the beacon still sounding, which is the one
  unmeasured thing and the reason the implementation carries a cap. See the
  `STOP_LATERAL_QUIET` block in `movement_service.h` for the two coupling
  mechanisms, of which the differencing gap is the one to watch.

## 4.3 Answer S9 from the departure burst, not from a new sensor

The mode question blocking `ramp_veto` is answerable from data already on disk.
The departure burst is 80 signed samples at 25 Hz — 3.2 s of the departure at
full rate — so **integrating it gives Δv directly**, and Δv is what separates the
regimes: 300 fpm is 1.52 m/s, 19 fpm is 0.097 m/s. Even at the 67% integration
efficiency measured on drive ramps that is ~1.0 against ~0.065.

Score it over the **227 departure bursts on four machines** already in
`datasets/` before writing any firmware. If Δv separates the populations, the
veto ships behind that flag and S9 closes; if it does not, say so and the veto
stays dead. Either outcome is worth having, and it costs no car time.

This is also the honest route to **S8**: the same integral is `vel_window`'s, and
`vel_departure` currently logs `0.000` on every run because the window is reset
at the exit from `STATE_MOVEMENT_DETECTED` (`cpp:802`) after only a 200 ms dwell.
Capturing peak `|w|` over the first few seconds of `STATE_MOVING` before the
reset would make the field mean something.

## 4.4 Two agreeing calibration windows (S6)

Run the 6 s window twice and require agreement — zeros within ~0.02 and spreads
within 2× — before accepting the learned threshold; on disagreement, re-run up to
`CALIB_RETRIES` and fall back to today's single-window behaviour, which is what
already happens. Print the spread on the READY line so a poisoned calibration is
visible in the log rather than only by comparing captures that were never
compared. Costs 6 s at power-on; changes nothing about how the value is used.

## 4.5 Do not disarm the jog verdict — fence it (S2)

Disarming (`JOG_VERDICT_ARMED 0`) removes a measured catastrophic error, and
costs every jog a 600 s beacon. That is a bad trade and it loses functionality
the request asks to keep. 4.2 fences it instead at no extra cost: a false JOG
fires on a *moving* car, the jog release routes through `STATE_DECELERATING` like
every other path, and the lateral term holds it. Re-score the 63/17 labelled
corpus with that fence before deciding anything further.

## 4.6 Leave these alone, and say why

- **`MIN_TRAVEL_MS` (S5)** — raising it costs the jog gate and the short-run
  cases; it is adequate at contract speed and the actual harm at inspection
  speed is arming into the ramp tail, which 4.1 makes safe. Fix the measurement
  trap in the tooling, not the constant.
- **`ARM_REV_SAMPLES` 15 (S2)** — it is the only thing holding the INTO-RAMP
  hazard at 0/227. The `rev=15` / false-release correlation is n=3 and the
  mechanism exonerates it (the run armed `v=1` and released on the peak).
- **`LATCH_FAILSAFE_MS` (S7)** — the asymmetry that set it has not changed.

---

# 5. The measured candidate: corroborate the peak with one block

**Tool:** `falcon_srcs/graph/coherence_replay.py`, added with this note.
**Corpus:** every arrival `BURST` on file — **231 bursts reaching the shipping
gate, 20 captures, four machines, inspection and automatic.**

## 5.1 What the burst records actually show

The confirmed false release and every healthy release on the same capture,
scored on the ramp detector's own arithmetic:

```
                              burst max   best block mean   directionality
  30 real arrivals, 150000     620-848         476-570            100%
  the false release (:431)         471              50             11%
  its twin (:453)                  471              40             51%
```

A drive-controlled stop is a **sustained push**; a cruise transient is a
**ring that cancels itself**. The discriminator was never amplitude. It is the
directionality the ramp detector already computes — the problem is only that the
ramp needs **three** consecutive blocks (1.4–2.0 s, and it lands 1115–9798 ms
after the peak, which is what refuted priority and veto).

**One block is 480 ms**, and on most stops it is already satisfied by the samples
that produced the crossing.

## 5.2 The rule

```
release when   |dev| >= ARRIVAL_PEAK_HIGH            (unambiguous brake set)
        or     the windowed peak crossed ARRIVAL_PEAK_VALUE
               AND one closed 12-sample block qualifies on the ramp
               detector's own two tests (mean >= 300, directionality >= 85%)
```

**The safety property is structural, not statistical.** `ARRIVAL_PEAK_HIGH ≥
ARRIVAL_PEAK_VALUE`, and the second branch ANDs onto the existing condition — so
the candidate releases a **strict subset** of what ships today, never earlier. It
cannot create a release that does not already happen. Every risk it adds is in
the position-lie direction, which `LATCH_FAILSAFE_MS` bounds. **That is the
property `ramp_veto` could not offer**, and it is why this is not the same trade.

⚠️ **This is not the hazard the `ramp_gate` block warns about.** That warning is
about the ramp as a *disjunctive* release path: quiet-arming on a slow departure
feeds it the departure ramp, which qualifies at mean 499 / dir 100 and releases
on its own. This test is *conjunctive* with the peak crossing — and that same
499 mm/s² plateau already clears the 450 gate today. The exposure is unchanged.

## 5.3 What it costs, over the whole corpus

`ARRIVAL_PEAK_HIGH = 550`:

| | |
|---|---|
| arrival bursts reaching the shipping gate | **231** |
| release at the **same instant** as today | **118** |
| release later | 109 — **median 0 samples**, worst 1840 ms |
| defer past the end of the burst | **4** |

The four deferrals, and this is the whole cost:

```
  260820-150000:431   burst max 471   <- THE CONFIRMED RELEASE ON A MOVING CAR
  260820-150000:453   burst max 471   <- its twin, same signature
  260820-113000:199   burst max 494   <- bench cascade, a tap and not a car
  260812-112254:184   burst max 463   <- automatic cab; unresolved
```

Set against the refuted candidates: `ramp_veto` costs 134 NO RELEASE in 198 and
100% of inspection; `ramp_pri` costs a median 2000 ms and is still catastrophic.
**This costs 4 deferrals in 231 with a median delay of zero, and the confirmed
false release is one of the four.**

The inspection captures — the population that killed the veto — are untouched:
`260818-105317` releases 52 of 62 at the identical instant with **zero**
deferrals; `260818-133242` is 13 of 13 at the identical instant.

## 5.4 Where 550 comes from, and what it is carrying

The same method that derived `ARRIVAL_PEAK_VALUE` — the middle of the two
populations, **not** a fit to a sample maximum, which is the refuted approach.

```
  worst incoherent transient that is not a car arrival   494   (a bench tap;
                                                the confirmed false one is 471)
  weakest incoherent brake set on a car                  675
```

1.11× above one, 1.23× below the other. **That is thin, and it is stated rather
than dressed up.** What makes it shippable where retuning the arrival gate was
not is that both errors degrade gracefully: too low and a cruise transient skips
corroboration, which is exactly today's behaviour; too high and a brake set waits
for the run to supply a block, which is the bounded direction. Neither end
reaches the catastrophic one.

⚠️ **`ARRIVAL_PEAK_HIGH` is not a minor constant.** **112 of the 231** arrival
bursts never produce a qualifying block at all — they are brake sets, which ring
rather than push (directionality 0.02–0.42, `falcon_signature` §4e). **Half the
corpus releases on the HIGH branch alone and gets no corroboration from this
change whatsoever.** What it buys is the other half — drive-controlled stops,
which is exactly where the false release lives.

## 5.5 What this replay cannot see

Four limits, all of which push the reported cost **up**:

1. A burst is 3.2 s around the *first* crossing. **"Defer" is not "never
   releases"** — the run continues and the detector keeps testing. For the
   confirmed false release the capture shows `RAMP latched mean=505 dir=100`
   about ten seconds later on the real deceleration, so that run would have
   released there, correctly, and roughly a second earlier than the three-block
   verdict. **The other three cannot be resolved from a burst.**
2. The burst is written regardless of arming, so the crossing found here can
   precede the firmware's. Delays are measured from the earlier instant.
3. Block boundaries align to the crossing here and to the accumulator on the
   device, so a boundary can fall one block later in reality.
4. Departure bursts come only from the any-motion branch (**B3, S10**), so
   polled-only runs carry none. 26 bursts against 33 latches in
   `260820-150000.log`.

## 5.6 Before this flies

1. **Extend the scoring to the continuous stream**, as
   `ramp_priority_replay.py` had to, and resolve the three unexplained
   deferrals. A burst replay cannot close this on its own.
2. **Re-run `graph/arming_replay.py`.** The rule touches what `arr_hit` means,
   and the standing instruction after any change to arming is to re-run it.
3. **Cost the flash.** `production` has 1134 bytes free per `platformio.ini`'s
   measured table (not the 80 an earlier draft quoted — see S10), so a second
   block accumulator and its gate should fit. Measure it rather than assume.
4. **Ship it unarmed first**, as the ramp detector was: print
   `PKC dir= mean= would_defer=` at every crossing and fly it before it is
   allowed to change a release. That protocol is what caught the ramp's
   departure-ramp hazard.
5. ⛔ **This does not clear the installation.** The standing instruction not to
   rely on the beacon there is unchanged by a desk result.

---

# 6. If only one thing is done

**§4.2** — the lateral term in `STATE_DECELERATING`. It is four lines at the one
choke point every release path passes through; it covers the peak, the ramp and
the jog verdict together; it cannot silence anything that is not already silent
today; its worst case is one the project has already accepted and bounded; and
the channel it reads is the same one that adjudicated the false release offline.

**§5** is the better fix for the mechanism and it needs a continuous-stream
replay, a flash budget and an unarmed flight first. §4.2 needs a replay of data
already on disk.

Neither is a substitute for the other: §5 stops the wrong *moment* being chosen,
§4.2 stops the wrong moment being *acted on*.
