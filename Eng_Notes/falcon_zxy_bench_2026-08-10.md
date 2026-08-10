# Z + X/Y — implementation, and the bench session that cleared risk 1

**Date:** 2026-08-10
**Status:** implemented, armed, and measured on a bench. Not measured in a
hoistway.
**Implements:** `falcon_spec_primary_usecase_2026-08-09.md`. Section refs of
the form §n point at `falcon_analysis_2026-08-06.md` unless stated.

The headline: **the buzzer does not prevent the lateral release.** Spec §8
listed that as risk 1 — "cheapest test, most likely to invalidate the design"
— and it is now closed with data. The design survives its worst known hazard.

---

## 1. What was built

`lateral.{h,cpp}`, plus wiring in `main.cpp` and `movement_service.cpp`.

**Z sets. X/Y resets.** The z departure paths are untouched; every z arrival
path is still present as a backup. The only new control authority is a release.

### 1.1 The metric

`m = |dx| + |dy|` between consecutive **unblanked** samples, m/s².

A difference, not a deviation from a stored lateral reference. Two reasons,
both of which the session then exercised:

- **Immune to attitude.** x/y only lack a gravity component when the device
  sits flat; on a tilted counterweight frame some of the 1 g leaks in as a
  static offset. A difference cancels any constant.
- **Nothing to go stale.** §3.1's recurring failure on this device is a
  baseline captured once and wrong forever. A metric with no baseline cannot
  have that failure.

Gaps longer than `XY_MAX_DT_MS` (1200 ms) produce **no metric at all**, not a
zero. Crediting a blanked stretch as quiet is the one arithmetic mistake here
that could release a beacon on a moving counterweight. Measured gap through a
beep cycle is 639 ms, comfortably inside the limit.

### 1.2 `XY_STILL`, learned in situ

Median of per-second maxima over the existing 10 s calibration × 1.50, clamped
to [0.02, 0.40]. The deployment sequence places the device before powering it,
so calibration always runs at rest on the actual machine in the actual
building.

Rejected, and retried, if the window saw an any-motion edge, if a bucket
exceeds `XY_CALIB_MOVE_MSS` (0.80), or if fewer than 6 of 10 buckets received
samples.

**Deviation from the spec, deliberate.** §4 says "refuse to arm if the
calibration window shows movement". This retries twice and then arms on the
low fallback instead. Refusing outright leaves a device on a counterweight
that never beacons, and §2 calls a silent device the unrecoverable failure —
the mechanic is ranging by ear. What is refused is the *learned threshold*,
not arming; the fallback is at the low clamp, which makes the unit over-eager
rather than deaf, and the ready signal says so.

### 1.3 Release

`XY_RELEASE_POLLS` (2) consecutive quiet metrics after `XY_MIN_BEACON_MS`
(1500) → straight to `STATE_STOPPED`.

Deliberately **not** through `STATE_DECELERATING`. That state exists to hold
the beacon through levelling by demanding `STOP_CONFIRM_MS` (5 s) of quiet z;
x/y handles levelling structurally, because a levelling counterweight is
genuinely moving and cannot produce quiet lateral metrics. Routing through it
would add 5 s to a reset budget of 1–3 s.

Re-arm blanking after a lateral release is `XY_REARM_MS` (500), not
`MONITOR_REARM_MS` (6000). 6 s of deafness is unacceptable under inspection
operation where the next jog can follow the last by a second, and the 6 s
exists for post-arrival ringing that a lateral release has already observed to
be over.

### 1.4 Ready signalling (§6)

One chirp = clean arm. Three = armed but the site could not be measured.
`ready_signal()` drives the piezo directly rather than reusing
`enable_alarm()`, whose duty cycle is a sensing parameter and must not be
repurposed for UX.

⚠️ It therefore does **not** blank the sensor, which `enable_alarm()` did as a
side effect. The z zero is captured *before* the chirp, and the rolling average
is flushed and the lateral anchor dropped *after* it. Without that ordering a
4-sample average at 3.13 Hz — a 1.28 s window — would capture a baseline built
mostly of piezo ringing and hand it to the FSM as "at rest" for the whole
deployment.

---

## 2. Bench session, 2026-08-10

Bench, mains-powered, CP210x on COM5. Flash 30052 / 32256 = 93.2%, RAM 1191.

### 2.1 Calibration

Two boots, two clean first-attempt arms, no retries:

| boot | buckets | peak | moved | `XY_STILL` | orientation |
|---|---|---|---|---|---|
| 09:43 | 10/10 | 0.0533 | 0 | **0.0440** | x≈0.92 y≈−0.46 |
| 09:55 | 10/10 | 0.0335 | 0 | **0.0350** | x≈0.92 y≈−0.25 |

The threshold adapted to attitude across a re-placement, which is the
per-deployment behaviour §4 wanted. Parked metric over a ~7.7 minute soak:
median **0.018**, 93% of samples under the threshold.

The boot transient *did* raise an any-motion edge (t=5275) but landed in
`STATE_ERROR_RESET` before the calibration window opened, so `mv=0`. The
`ERROR_RESET` dwell absorbs it by construction. Worth knowing, because an edge
one state later would reject every first calibration on every deployment.

### 2.2 🟢 Run 1 — risk 1, closed

Vertical jolt, then hands off.

```
t=40583  jolt          z=11.07  m=1.148
t=40615  ACC-INT n=2   Departure latched
t=41730  ACC-INT n=4 bz=1  -> STATE_MOVING, buzzer on
t=42172  bz on   m=1.487  q=0
t=42811  bz on   m=2.049  q=0
t=43450  bz on   m=0.007  q=1
t=44089  bz on   m=0.028  q=2
         FSM: Release (x/y still) m=0.0275 xs=0.0350 q=2
```

**With the piezo actively driven, the metric reached 0.007 and 0.028 against a
0.035 threshold.** §4.3 of the design note called this the largest hole and
said the release path was "not unblocked by the driver swap — it needs
mechanical isolation, or enough host sample rate to decide from the beep-off
gaps alone (2–3 samples/s today — nowhere near)".

That estimate was wrong in our favour. The timer ISR already drops every
sample taken while the piezo is driven plus `BUZZER_RINGDOWN_MS`, so the
samples that reach the detector are *by construction* the clean ones. Measured
1.56 samples/s through the beep cycle, and two of them are all the rule needs.
**Mechanical isolation is not on the critical path.**

### 2.3 🟢 Run 2 — pick up and tilt

Harsher, and it tested the metric's central design choice.

```
t=177635  pickup        z=11.01  x=0.556  y=2.023   m=2.630
t=177668  Departure latched
t=178274  full tilt     z=0.16   x=-2.32  y=9.326   <- gravity onto +Y
t=178733  -> STATE_MOVING, alarm on
t=179240  handling      m=3.411  q=0
t=179879  set down      m=1.387  q=0
t=180518                m=0.035  q=1
t=181157                m=0.039  q=0   <- correctly re-armed
t=182435  moved again   m=0.527  q=0
t=184991                m=0.031  q=1
t=185630                m=0.010  q=2
          FSM: Release (x/y still) m=0.0102 xs=0.0350 q=2
```

Beacon held **6.9 s**, tracking the handling, and re-armed twice mid-alarm when
motion resumed. Released 1278 ms after the last above-threshold metric. That is
the anti-dropout hysteresis of §4.2 working on genuinely bursty input — the
closest bench analogue available to inspection jogging.

**The metric survived a ~90° reorientation.** At t=178274 gravity had moved
almost entirely onto +Y: a ~9.3 m/s² static shift against a 0.035 threshold,
265×. The difference cancelled it and only the motion registered. A
reference-based metric would have read the new attitude as permanent motion
and never released. §1.1's reasoning is now a measurement.

### 2.4 Latencies

| | run 1 | run 2 |
|---|---|---|
| z edge → beacon on | 1115 ms | 1065 ms |
| last motion → release | ~1.3 s | 1.28 s |
| beacon duration | 3.5 s | 6.9 s |

Both inside the 1–3 s budget in both directions.

---

## 3. `MOVEMENT_DETECTION_TIMEOUT_MS` 1000 → 200

Essentially the entire set latency above was the `STATE_MOVEMENT_DETECTED`
dwell. That is inside budget, so this is not a requirement fix — it is §8.5's
second working-range axis, minimum burst **duration**.

A sub-second jog still alarms; the latch fires on the edge and nothing
downstream can cancel it. What the dwell did was start the beacon *after the
jog ended* — a 0.25 s jog sounding from 1.1 s to 2.6 s over an already
stationary counterweight, a beacon asserting "moving now" about a thing that
has stopped.

It bought nothing on the path that matters. It is a debounce inherited from the
polled threshold test; any-motion is already debounced in hardware at 100 Hz.

Not zero, because `vel_departure` accumulates its ramp peak in that state.
Coupling worth knowing: `movement_start_timer` moves 800 ms earlier, and it is
the reference for both `MIN_TRAVEL_MS` (3000) and `XY_MIN_BEACON_MS` (1500).

⬜ **Untested.** No deliberate short jog has ever been recorded on this device.

---

## 4. What is still open

| # | Item | Status |
|---|---|---|
| 1 | 🔴 **Risk 2 — smooth counterweight drops the beacon mid-travel** | Now the top risk. Travel must hold the metric above `XY_STILL` continuously. Untestable on a bench; needs a counterweight |
| 2 | Duration axis — taps at 0.25 / 0.5 / 1 / 2 s | Cheap, bench, not yet run. Gates §3 above |
| 3 | Parked-vs-travelling lateral contrast on a counterweight | The other half of the threshold. Needs roadmap 11 |
| 4 | Flash at 93.2% | 2.2 KB left. Driver swap returns ~4.9 KB |

The ⛔ stillness-backstop block in `movement_service.h` still stands and still
wins if the hoistway contrast measures marginal. Its numbers are all z and its
argument is the 1 g pedestal, which does not transfer to gravity-free axes —
but it was written after a real mid-ride release, and risk 2 is that failure
returning by another route. `XY_RELEASE_ARMED 0` reverts to the previous
behaviour with everything still logged.

## 5. Incidental findings

- **A purely lateral disturbance is invisible to the set side.** Two handling
  events reached m=0.48 and m=1.26 with **zero** any-motion edges, because
  `ANYMOTION_THRESHOLD` is configured Z-axis-only. Harmless for a counterweight,
  where real motion is vertical, but it means bench handling is not a reliable
  way to provoke a departure — use a vertical jolt.
- **z's zero is only valid for the attitude it calibrated in.** Pre-existing,
  not introduced here, and it does not arise in the real sequence where the
  unit is placed before power-on and not touched again. The lateral path is
  unaffected either way.
- `ACC-STAT ... sk=1` fired once during a pickup — the any-motion
  stuck-reference guard. It cleared on its own.
- **The serial lead back-powers the board, so re-seating it is a power cycle.**
  One run was lost to this: the cable was disturbed, telemetry stopped mid-
  stream with no `Device Booted`, and the device silently rebooted and
  re-calibrated at a new attitude. If the log stops without a boot banner,
  suspect the cable before the firmware.

## 6. Log format and tooling

Sample lines gain ` m=` and ` q=`. Both are needed: the metric differences
*consecutive unblanked* samples and the log does not record which samples the
ISR dropped, so it cannot be reconstructed from x= and y= offline.

`graph/parse_falcon_log.py` gains an XY section splitting the metric by FSM
state, and calls out the risk-1 signature explicitly — the metric never
getting under `XY_STILL` while st=4.
