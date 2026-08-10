# Hoistway test protocol — and the first session's result

**Date:** 2026-08-10
**Status:** protocol §1–§4 is reusable. §5 is the first execution of it, and it
returned a negative result for the x/y release path.
**Reads with:** `falcon_zxy_bench_2026-08-10.md` (the bench that preceded it),
`falcon_spec_primary_usecase_2026-08-09.md` (the requirement). Section refs of
the form §n point at `falcon_analysis_2026-08-06.md`.

---

## 1. The constraint that shapes everything

**The counterweight cannot be instrumented.** §5.1: running a serial cable to
a moving counterweight means being in the hoistway, which is the hazard the
device exists to warn about. So the programme splits by what can carry a
cable, and the actual deployment target is the one we can only listen to.

| Phase | Where | Telemetry | Purpose |
|---|---|---|---|
| 0 | bench | full | duration axis; anything settleable without a shaft |
| 1 | cartop | full | **all the numbers** |
| 2 | counterweight | none, audible only | confirm it transfers |

Cartop is not a counterweight — §14.5 says the cwt is rougher, the favourable
direction — but it is the only moving platform that can carry a cable.

---

## 2. Phase 0 — bench

**The duration axis.** Timed taps at ~0.25, 0.5, 1, 2 s. For each: does the
beacon start *during* the motion, and how long does it run?

- Pass: beacon onset under ~0.3 s at every duration
- Expected and correct: a 0.25 s tap producing a `XY_MIN_BEACON_MS` beacon
- Decides: where the minimum-burst floor of §8.5 actually sits

---

## 3. Phase 1 — cartop, instrumented

### 3.1 Run UNARMED first

`XY_RELEASE_ARMED 0`. Log what the release path *would* have done, replay it,
and only then arm.

This is not caution for its own sake. A release that fires mid-ride is the
catastrophic direction, it is the failure that got the stillness backstop
banned on 2026-08-07, and a hoistway is the only place it can appear. Unarmed,
the FSM behaves exactly as the previous build while `m=` and `q=` are logged
throughout. **§5 below is the vindication of this rule** — the first run would
have dropped the beacon within seconds.

### 3.2 Runs to capture

| # | Run | Yields |
|---|---|---|
| 1 | Parked, 10+ min, live building | the real lateral floor. A bench floor is not it |
| 2 | Slowest speed available, longest travel | **the decisive measurement** |
| 3 | Two or three higher speeds | contrast vs speed |
| 4 | Short jogs on inspection, 0.25–2 s | duration axis in a real shaft |
| 5 | Terminal-floor approach with levelling | does levelling hold the metric up? |

Also boot the unit 3–4 times before run 1 and record the `XY_STILL` spread.
Three bench boots gave 0.035 / 0.044 / 0.050 — 44% on one site — which is the
open question of whether 10 s of calibration is enough.

### 3.3 The pass mark

> **min(cruise `m`) at the slowest speed must exceed max(parked `m`), with
> margin.**

Not "cruise median above parked median" — the release rule fires on
consecutive samples, so what matters is whether the *quietest stretch of
travel* stays above the *noisiest parked* level. Report the threshold sweep
and the longest consecutive quiet run during cruise, not just the medians.

### 3.4 Failure signatures

| Observed | Meaning | Direction |
|---|---|---|
| beacon drops mid-travel | risk 2 real | **dangerous** |
| beacon runs to `FSM: FAILSAFE` | threshold too low for the site | annoying, safe |
| tracks the run, releases 1–3 s after the stop | works | — |

---

## 4. Phase 2 — counterweight, audible only

Full sequence: place on the cwt frame, power on, leave it untouched through
calibration, note the ready chirps (one = clean, three = site unmeasurable),
then operate from the cartop and listen with a stopwatch.

Three chirps is itself a result: the cwt moved, or the site could not be
measured, and the unit armed on the fallback.

### Cross-cutting

- **Split every capture on `Device Booted`.** `millis()` restarts at each
  reflash; mixing boot sections has produced a completely false result before.
- **Confirm each flash took** — flash size, or the `AnyMotion armed thr=`
  banner. The armed and unarmed builds differ by 214 bytes, which is a
  convenient check.
- **Do not touch the unit after calibration.** `zero_calib_value` is
  attitude-bound. The lateral path is not.
- **The serial lead back-powers the board**, so re-seating it is a power
  cycle. If telemetry stops with no boot banner, suspect the cable.
- **The programmer's COM port changes on almost every reconnect.** It was COM6
  and COM7 within one session. Enumerate, never assume.

---

## 5. 🔴 First execution, 2026-08-10 — the x/y release path fails

Cartop, live building, unarmed per §3.1. Two runs, 5 floors each.

### 5.1 The measurement

| | n | min | p05 | median | p95 | max | max `q` |
|---|---|---|---|---|---|---|---|
| Parked | 816 | 0.000 | 0.004 | **0.0160** | 0.037 | 0.104 | 37 |
| Cruise, 25 fpm | 309 | 0.001 | 0.004 | **0.0190** | 0.040 | 0.192 | 19 |
| Cruise, 122 fpm | 60 | 0.004 | 0.013 | **0.0465** | 0.120 | 0.245 | 2 |

Longest consecutive quiet run during cruise, by threshold:

```
thr 0.010:  25 fpm =  5    122 fpm = 1
thr 0.020:  25 fpm =  8    122 fpm = 2
thr 0.035:  25 fpm = 41    122 fpm = 2
thr 0.050:  25 fpm = 304   122 fpm = 8
```

### 5.2 What it means

**At 25 fpm there is no separation at all.** Cruise median is 1.19× parked
median; the quietest travel is 0.001 against a noisiest parked of 0.104. The
§3.3 pass mark is missed by two orders of magnitude. Every threshold this
device has ever calibrated (0.035–0.050) leaves 88–98% of cruise below it,
with quiet runs of 41 and 304 against a release rule of 2. **Armed, the
beacon would have dropped within ~1.3 s of departure and stayed dropped for a
five-floor ride.**

**At 122 fpm there is a real but unusable margin.** Cruise median rises to
2.9× parked — so the lateral excitation is genuine and it scales with speed,
which means the design's premise is not wrong, only far too weak. But the
longest quiet run is 2, exactly equal to `XY_RELEASE_POLLS`, so it would still
have released. `XY_RELEASE_POLLS 3` would have held *this* run — a one-sample
margin on 60 samples from a single trip, which is not a basis for arming a
path that can silence a beacon on a moving mass.

**The failure is worst where the product lives.** The requirement is 20 fpm,
and inspection operation is inherently slow. Speed-dependence puts the design's
weakest point exactly at its most important use case.

### 5.3 What still works

Both runs exercised the existing design and it was healthy:

- departures caught by any-motion at both speeds
- arrivals caught by the polled path, delta 0.412 and 0.320
- beacon held 196.8 s and ~40 s, releasing on arrival, no failsafe
- zero false departures across ~13 minutes of parked capture

Note the 196.8 s run is 66% of `LATCH_FAILSAFE_MS`. Five floors at 25 fpm. A
taller building at inspection speed will reach it.

### 5.4 The one thing that could rescue it

**Sample rate, and only sample rate.** The metric scaling with speed proves we
are measuring real vibration rather than noise; at 3.13 Hz we are sampling a
40 Hz-filtered signal and capturing an aliased sliver of it. At 100 Hz the
same metric should see substantially more of the actual power, and the low
speed separation may become usable.

This is the identical conclusion `velocity.h` reached about its own gate, and
it makes **roadmap item 5** the blocking dependency for two independent
features rather than one.

**Re-run §5.1 exactly after the timer fix before abandoning the approach.**
The comparison is cheap once the shaft time is booked, and it is the only
evidence that would justify arming.

### 5.5 Disposition

- `XY_RELEASE_ARMED` stays **0**. Everything remains logged.
- The ⛔ stillness-backstop block in `movement_service.h` **wins**, as it said
  it would if the contrast measured marginal. Its argument was about z and the
  1 g pedestal, and it does not transfer to gravity-free axes — but its
  conclusion does, arriving by the route §8 of the design note predicted.
- Risk 2 is no longer a risk. It is a measured failure.
- The z + x/y architecture is not dead; it is blocked on roadmap item 5.

### 5.6 Still unmeasured

- Phase 0's duration axis. Never run.
- Phase 2 entirely. Given §5.1 there is nothing to confirm yet.
- Whether a counterweight's rougher ride closes a 25 fpm gap this wide. It
  would need ~10x, and it cannot be measured without roadmap item 11.
