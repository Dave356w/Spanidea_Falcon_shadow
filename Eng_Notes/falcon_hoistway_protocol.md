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

### 5.6 ✅ The reset budget was wrong, and that reshapes the roadmap

Unprompted, after both runs, Dave: *"both speeds alarmed held through travel
and promptly reset upon arrival, behavior is best observed to date"*, and then
*"set, cruise and reset was acceptable as observed"*.

Measured, arrival detection to silence:

| run | arrival delta | reset |
|---|---|---|
| 25 fpm down | 0.412 | **7.67 s** |
| 122 fpm up | 0.320 | **8.31 s** |

That is `STOP_CONFIRM_MS` plus a couple of excursions restarting the window —
about **3x the spec's 1-3 s reset budget** — and the person who set the
requirement, watching the device on real equipment, called it prompt.

**The requirement moves, not the firmware.** Spec §2 is amended.

The consequence is larger than a tolerance. A 1-3 s reset was one of the two
justifications for the whole Z + X/Y release path. With it withdrawn, that path
carries only the other one — arrivals z physically cannot see — so it drops
from *necessary* to *an optimisation for a narrower case*, and item 5 loses one
of its two claims on priority.

**What this does NOT relax.** §14.7's gentle 18 fpm arrival measured 0.058 on
the polled path against a 0.30 threshold, and would still be missed entirely,
leaving the unit sounding on a stationary counterweight until
`LATCH_FAILSAFE_MS`. A beacon that never releases is a position lie and stays
unacceptable at any duration. That failure is the one still worth engineering
against.

**Do not over-read the two runs.** n=2 and both were firm stops — the case the
whole arrival investigation started from was not exercised. The 122 fpm margin
was 0.320 against a 0.30 threshold, 7%: it fired, but that is not margin.

**Method note.** Dave's qualitative read and the log agreed here, which is
worth recording because the standing rule is the opposite — when they disagree,
he is usually right, since the log is a 3 Hz keyhole and he is watching the
actual car. An acceptance judgement is a measurement this project cannot take
any other way.

### 5.7 Parked soak, 11.8 min — and an unexpected result for `velocity.h`

Cartop, live building, car parked, untouched. 2218 samples at 3.13 Hz, `st=2`
throughout.

**Zero any-motion edges, zero FSM events, zero false departures.** Matches the
§14.6 soak and is the strongest evidence to date that `ANYMOTION_THRESHOLD 32`
/ `ANYMOTION_DURATION 5` is correctly placed. Averaged z delta peaked at 0.0257
against the 0.40 departure threshold, a 15x margin.

**The lateral verdict, now conclusive.**

| parked | median | p95 | p99 | peak |
|---|---|---|---|---|
| `m` | 0.0150 | 0.0320 | 0.0420 | **0.0640** |

25 fpm cruise median (0.0190) sits between the parked median and p95. 122 fpm
cruise median (0.0465) sits **below the parked peak**. Both cruise speeds fall
inside the parked distribution. The earlier parked max of 0.104 came from
windows contaminated by movement on the cartop; the clean figure is 0.064 and
it still swallows both runs. §5.2's conclusion is now measured rather than
inferred.

**🟢 The unexpected part.**

| | prior soak (§14.6) | this soak |
|---|---|---|
| \|w\| median | 0.0176 | **0.0080** |
| \|w\| p99 | 0.0757 | **0.0300** |
| \|w\| **peak** | 0.1274 | **0.0400** |
| crossings of the 0.255 gate | 2 | **0 of 2218** |

3.2x quieter at the peak, on noisier equipment. `velocity.h` derives its gate
as 2x the measured peak — 0.255 m/s, about 50 fpm, and hence its conclusion
that at 3.13 Hz no threshold is both safe and useful. On these numbers the same
rule gives **0.080 m/s, about 16 fpm, under the 20 fpm requirement.**

Hypothesis: 0.1274 predates the two bugs fixed in `540399f` — the baseline
seeded from a single sample, and the `|w|` freeze deadlock. Both inflate parked
`|w|`, and that commit already recorded the bench falling to 0.013 median
afterwards. If so, **the velocity path may be armable at the current sample
rate**, with no dependency on item 5.

That matters because the velocity **conservation test is a candidate solution
to the one failure still worth engineering against** — §14.7's gentle arrival,
missed by z, leaving the beacon sounding on a stationary counterweight until
the failsafe. It scales itself from each run's own departure rather than a
fitted constant, which is the property that case needs.

⬜ **Not a conclusion.** n=1, 11.8 minutes, and the parked distribution is
heavy-tailed — a shift has roughly 40x more opportunity to throw an outlier
than this soak saw. **Test: a one-hour-plus parked soak replayed through
`graph/velocity_replay.py`.** It needs a bench, not a hoistway, and it can run
overnight. Do that before touching `VEL_ARMED`.

### 5.8 Still unmeasured

- Phase 0's duration axis. Never run.
- Phase 2 entirely. Given §5.1 there is nothing to confirm yet.
- Whether a counterweight's rougher ride closes a 25 fpm gap this wide. It
  would need ~10x, and it cannot be measured without roadmap item 11.
