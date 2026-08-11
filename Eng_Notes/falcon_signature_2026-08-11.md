# Departure and stop have different SHAPES, not different sizes

**Date:** 2026-08-11, cartop, live building
**Status:** two bursts, one jog and one run. A mechanism, not yet a detector.
**Read after:** `falcon_25hz_arrival_2026-08-10.md` (see its correction banner).

This is the current state of the project. Where earlier notes disagree with
this one, this one wins.

---

## 1. Two corrections to yesterday

### 1.1 🔴 The arrival margin was overstated

Yesterday's note reported an 18 fpm arrival at 22.6× margin and a threshold
sitting "3.0× above the worst cruise and 2.9× below the smallest arrival". Two
further runs the same afternoon dismantled both halves:

| run | cruise ceiling | arrival |
|---|---|---|
| 18 fpm down | 0.09 | 2.03 |
| high speed up | 0.23 | 3.42 |
| 120 fpm down, 4 floors | 0.23 | 4.58 |
| 22 s run | 0.18 | **0.713** |
| 27 s run | **0.28** | 1.413 |

**The 0.713 arrival cleared a 0.70 threshold by 1.9%** — it fired by luck. The
honest separation is 0.28 → 0.713, about **2.55×**. The first run was an
unusually clean case and the headline drawn from it did not survive contact
with two more. `ARRIVAL_PEAK_VALUE` is now 0.45, 1.6× each way.

Still the best arrival discriminator this device has had — everything prior
managed 1.07×–1.4× — but a working margin, not a solved problem.

### 1.2 The x/y question is closed, permanently

Re-measured at 25 Hz, with the same analysis that gave 1.19× at 3.13 Hz:

| | n | median | p95 |
|---|---|---|---|
| parked | 13856 | 0.016 | 0.038 |
| genuine cruise | 1661 | 0.019 | 0.056 |

**1.19× at both sample rates.** The eight-fold rate increase moved it by
nothing, and the reason was predicted in `main.cpp`'s own comment before any of
it was measured: *aliasing hurts an ENERGY measure far less than it hurts
transient detection*. The vertical axis won at 25 Hz because it detects a
**transient**; the lateral metric measures a **level**, and levels survive
aliasing.

So the 1.19× is a property of this machine, not of the sampling. No sample rate
will fix it, including the 100 Hz fuse change. `XY_RELEASE_ARMED` stays 0, the
lateral module stays as instrumentation, and the ⛔ stillness-backstop block
keeps its authority.

---

## 2. The burst recorder

The question — *does a jog have a signature that differentiates it from a
departure?* — is about time structure roughly 1 s wide. The normal log
decimates 8:1 and loses ~20% to overrun, putting printed samples ~670 ms apart.
Two points across the thing being measured is not a measurement, and reasoning
from it had already produced an apparent counter-example (the 22 s run) that
might have been nothing but sampling.

So: 80 consecutive samples of **signed** `(a − zero)` in milli-m/s², captured
around each departure with 20 samples of pre-trigger history. 3.2 s at 25 Hz,
160 bytes.

**Sign is load-bearing** and was wrong in the first cut. A jog accelerates and
then decelerates by an equal and opposite amount, so its integral should
cancel; storing magnitude makes a deceleration indistinguishable from an
acceleration and destroys exactly the signal being looked for.

---

## 3. What the bursts show

### 3.1 Velocity CAN be inferred — for departures

| | integrated Δv |
|---|---|
| normal run, down | **−0.29 m/s = −57.6 fpm** |
| 1 s jog, up | **+0.18 m/s = +35.4 fpm** |

Correct sign, plausible magnitude. **The device can measure how fast it
departed**, which it has never been able to do. That is a direct answer to the
working-range question, per run, rather than a curve fitted across sessions.

**But not for stops.** The jog's stop integrates to −0.011 m/s where physics
demands ≈ −0.18, and the burst ends at +0.205 m/s for a device sitting still.
The stop is a bipolar ring flipping sign almost every sample — energy at or
above 12.5 Hz, which 25 Hz cannot integrate faithfully. Same aliasing wall, one
level up, and the reason `velocity.h`'s free-running integral drifted to
−4.44 m/s over a 171 s descent.

### 3.2 🟢 Amplitude cannot separate a departure from a jog

```
peak |a|    run departure   0.501
            jog departure   0.513
            jog STOP        2.131
```

**A real departure and a jog's departure are identical in amplitude.** Every
arrival threshold set in the last two days has been trying to separate
populations that overlap by construction — which is why 0.40 was too low, 0.70
too high, and 0.45 a compromise satisfying nobody.

### 3.3 🟢 Directionality separates them cleanly

`|Σa| / Σ|a|` over a sliding 0.5 s window — how one-signed the acceleration is:

```
RUN:  0.32  0.78  1.00  1.00  1.00  1.00  0.93  0.74  0.20  0.29 ...
JOG:  0.37  0.62  0.79  0.97  1.00  0.47  0.36  0.02  0.08  0.54 ...
                                                 ^^^^ the brake ring
```

| | peak | directionality |
|---|---|---|
| run departure | 0.501 | **0.968** |
| jog departure | 0.513 | **1.000** |
| jog stop | 2.131 | **0.024** |

**A departure is a sustained push in one direction. A stop is a violent ring
that cancels itself.** 40× separation, and it is a difference in *character*
rather than size — so it does not scale with speed, machine, or how hard the
stop was. That is what every amplitude threshold has lacked.

The run holds directionality ≈1.00 for **0.64–1.12 s**; the jog holds it for
**0.48 s** before collapsing into the ring. Duration-of-direction is the jog
discriminator, not the "two humps" hypothesis that prompted the instrument.

---

## 4. What should be built from this

**Arrival detection on two features, not one:** a large peak **and** low
directionality. A rough departure is large but one-signed, so it can no longer
masquerade as an arrival — which is precisely what nearly happened on the 22 s
run whose departure reached 1.49 while its arrival measured only 0.713.

⬜ **Not built.** Two bursts, one of each kind. Before writing a detector:
confirm the signature on several more departures and, critically, capture a
burst around an **arrival** — every burst so far triggers on departure, so the
arrival ring has only been seen inside a jog. That needs a second trigger.

---

## 4a. 🔴 The stillness release is dead too — cruise level is not a function of speed

Proposed mid-session on two points and killed by a third within the hour.

The idea, prompted by Dave's *"a stable Z and baseline stable x/y = no motion"*
and by his 350 fpm failure: cruise vibration looked like it scaled with speed
(3.6× rest at 18 fpm, 9.2× at 120 fpm), so a **self-calibrating** release could
measure each run's own contrast and, where it was high, treat a sustained
return to baseline as the stop. That would have covered exactly the case the
brake-transient detector cannot — a smooth drive-controlled stop at speed.

| run | cruise `pk` | ratio to rest |
|---|---|---|
| 18 fpm down, 8/10 | 0.09 | 3.6× |
| 120 fpm down, 8/10 | 0.23 | 9.2× |
| high speed up, 8/10 | 0.23 | 9.2× |
| **125 fpm up, 8/11** | **0.07** | **2.8×** |

**Two runs at the same speed differ threefold, and today's 125 fpm is quieter
than yesterday's 18 fpm.** Cruise level is not a function of speed.

Most likely cause: **the unit was remounted between sessions** — resting
attitude moved from x≈0.63 to x≈0.57. How rigidly it is clamped governs how
much rail excitation reaches the sensor, and that swamps the speed effect.

That is worth knowing independently of this design: **cruise vibration measures
the mounting as much as the machine**, so any threshold derived from it is
hostage to how the mechanic attached the unit that morning.

The self-calibrating design would have *behaved* correctly here — 2.8× falls
under the cut-off, the release declines to arm, and the run falls back to
brake detection. It fails safe. But it would seldom arm, which makes it close
to useless rather than wrong, and it cannot rescue the 350 fpm case because
there is now no reason to assume contrast is high there.

---

## 4b. Thresholds re-measured at full sample rate

The sample ring recovered the ~20% of samples the single slot was dropping, so
every figure in §1.1 and in the 08-10 note was computed from 80% of the data.
Re-measured on the full stream:

| | full rate | previous (80%) |
|---|---|---|
| rest, max | **0.070** | 0.040 |
| cruise ceiling, 64 fpm (14 s) | **0.200** | — |
| cruise ceiling, 125 fpm (9.8 s) | **0.080** | 0.070 |
| arrivals | **1.690, 1.217** | 0.713–4.58 |

Parked noise rose ~50%, as expected — a denser stream has more chances to catch
a peak. **`ARRIVAL_PEAK_VALUE` 0.45 survives: 2.25× above the worst cruise and
2.7× below the weakest arrival of the day.**

⬜ **But only for SHORT runs.** Both were 14 s and 9.8 s, giving 16 and 8 cruise
points. A ceiling is a tail statistic and a max over 10 seconds systematically
underestimates the max over 200 — yesterday's 25 s cruise gave 0.23 where
today's 10 s gave 0.08, which is mostly run *length*, not speed. The long-run
ceiling is the number that matters now that `LATCH_FAILSAFE_MS` permits ten
minutes, and it is **still unmeasured**.

And the weakest arrival ever recorded is 0.713, not today's 1.217. Against that
figure 0.45 has only 1.6×.

## 4c. 🔋 The battery question answered itself

The 8-floor 21 fpm run — the one measurement that would have closed both §4b
and the 600 s failsafe — was aborted when **the board lost power at the moment
of departure.**

```
ADC_VOLTAGE_THRESHOLD   2000      (low-battery trip)

10:58   2172
11:02   2169
11:20   2166
11:31   2169
11:35   2124   <- last reading, board died here
```

Flat all morning, then a 45-count drop in four minutes, quitting 6% above the
trip point. That is cells collapsing under load, not a connector fault.

The cause is the day's usage: many alarms, several of them stuck jogs running
the full 300 s failsafe to completion. **This is the endurance question flagged
when `LATCH_FAILSAFE_MS` was raised to 600 s, answering itself the hard way** —
and it makes that change look considerably less affordable than it did. A
single stuck jog now sounds for ten minutes.

**It also re-prices the jog defect.** Until now the cost was a false beacon and
lost trust; it is now also pack life, on a device that must survive an entire
job unattended on a counterweight.

⬜ Still unmeasured: alarm current against pack capacity. That is a bench
measurement, it needs no hoistway, and it should happen before anyone relies on
the 600 s failsafe in the field.

## 4d. 🔴 THE DEVICE BROWNS OUT AFTER ~4 MINUTES OF CONTINUOUS ALARM

This outranks everything else in this note.

Measured on **fresh cells**, during an 8-floor 20 fpm descent:

```
t=203554   2442     before the run
t=362315   2357      75 s into the alarm
t=479281   2324     192 s into the alarm
t=530006   dead     243 s into the alarm

ADC_VOLTAGE_THRESHOLD  2000    never reached
```

**It stopped at 2324, over 300 counts above the low-battery trip.** The battery
alarm never fired because by its own measure the pack was healthy. So this is
**not depletion** — it is the rail sagging under sustained piezo load, and the
averaged ADC reading never sees the instantaneous dip inside a beep. An earlier
failure at 2124 the same morning is the same mechanism; fresh cells bought
runway, not immunity.

**Alarm endurance is therefore ~4 minutes**, measured once.

### Consequences, all immediate

- **`LATCH_FAILSAFE_MS` reverted 600 s → 240 s.** A failsafe longer than the
  hardware survives can never fire. The 600 s was set hours earlier on the
  assumption a ten-minute alarm was affordable; it is not, and that assumption
  was never measured. The cost is real and was predicted by the original 300 s
  comment: an 8-floor 18 fpm run needs ~300 s and will now be cut short.
- **The long-slow-run case is currently untestable.** Both attempts at an
  8-floor 20 fpm descent died mid-run — not coincidence, the run simply lasts
  longer than the device's alarm endurance. So the low-speed long-run cruise
  ceiling, the number the 0.45 threshold most needs, cannot be measured until
  this is fixed.
- **It re-prices the jog defect a second time.** A stuck jog runs the failsafe
  to completion, so every one of them now costs a substantial fraction of the
  device's total alarm budget.
- **It threatens the primary use case directly.** The device is meant to sit on
  a counterweight for a whole job. If four minutes of alarm kills it, that is
  ahead of every detection question here.

⬜ **The measurement that unblocks it:** alarm current against pack voltage
under load, with a meter rather than the on-board ADC — which is averaging away
the very sag that is killing it. That separates cells from regulator from
decoupling near the piezo. Bench work, no hoistway.

## 5. Open problems

| # | Problem | Status |
|---|---|---|
| 1 | 🔴 **Normal operation has no brake transient at the stop** | Dave, 350 fpm from the cab: the drive decelerates smoothly to a standstill and sets the brake afterwards, so the unit alarmed and never released. No second mechanism survives contact with data |
| 2 | 🔴 **Jogs latch the alarm until the 300 s failsafe** | Unsolved, four closed doors. Under inspection jogging the device is effectively always sounding |
| 3 | **An 8-floor 18 fpm run expired on `LATCH_FAILSAFE_MS`** | Measured by Dave. 300 s is too short for slow travel in a tall building. Straightforward fix, not yet made |
| 4 | ~20% of samples lost to snapshot overrun | Unfixed. Every threshold rests on 80% of the data. Needs a ring buffer; bench work |
| 5 | Weakest arrival is 0.713 and moved 3× in five runs | Needs more soft stops. The number the approach rests on |
| 6 | Flash 95.0%, RAM 67% | Room for nothing much. Driver swap frees ~4.9 KB and is the prerequisite for on-device recording |

### 5.0 The product boundary, stated plainly

**The device works where a brake sets abruptly** — inspection operation, which
is the stated primary use case, and where Dave reports it "stable at all speeds
on cartop and counterweight" with performance "great so far". Five arrivals
measured, 0.713 to 4.58.

**It does not yet cover normal automatic operation**, because that stop has no
transient to detect. That is a boundary to document and decide about, not a bug
to tune away.

### 5.1 A note on method — three single-run conclusions overturned in one day

The 22.6× arrival margin, the speed-scaling hypothesis, and two proposed jog
fixes were all built on one run or one idea and corrected by the next
measurement. In each case the first result looked decisive and was not.

**Treat any single-run number as a hypothesis until a second run under
different conditions agrees with it.** The burst recorder exists because of
this, and §4's detector should not be written until the arrival ring has been
captured on more than one machine state.

### 5.1 The jog problem, and three closed doors

Movements finishing inside `MIN_TRAVEL_MS` have their stop discarded unseen.
Three candidate fixes are now measured and eliminated:

- **Velocity integral confirm** — the ordering is backwards. A real 25 fpm
  departure integrates to less than a jog does, because a hand on an inspection
  button accelerates harder than a machine ramping gently.
- **Lateral level confirm** — 1.19×, at both sample rates (§1.2).
- **Arm-on-quiet peak detection** — structurally incapable. It needs a quiet
  gap between departure and stop to arm, and a 1 s jog is one continuous
  disturbance with no such gap. The mechanism that makes the detector safe on a
  real run is what blinds it on a jog.

§3.3 is the first candidate that is not obviously ruled out, because it keys on
character rather than amplitude or quiet.

**Two fixes were proposed from a hoistway and both were structurally incapable,
not merely mistuned. Both times the cause was moving before the data was in.**
That is the argument for the burst recorder existing at all, and for not
building §4 until the arrival burst is captured.
