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

## 5. Open problems

| # | Problem | Status |
|---|---|---|
| 1 | 🔴 **Jogs latch the alarm until the 300 s failsafe** | Unsolved. Under inspection jogging the device is effectively always sounding |
| 2 | ~20% of samples lost to snapshot overrun | Unfixed. Every threshold rests on 80% of the data. Needs a ring buffer; bench work |
| 3 | Weakest arrival is 0.713 and moved 3× in five runs | Needs more soft stops. The number the approach rests on |
| 4 | Flash 95.0%, RAM 67% | Room for nothing much. Driver swap frees ~4.9 KB |

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
