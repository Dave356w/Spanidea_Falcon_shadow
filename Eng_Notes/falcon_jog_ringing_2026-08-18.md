# Jog verdict silences real runs on rapid reversal — inverted cartop, 2026-08-18

**Date:** 2026-08-18
**Rig:** cartop, Dave driving. **Device mounted UPSIDE DOWN** to simulate
placement on the bottom of the counterweight frame.
**Firmware:** shipping, flashed this session over ISP. **Size 31464 bytes**
(⚠️ the 2026-08-14 test plan records 31416 — a 48-byte drift; the committed
tree has moved since the plan and the two should be reconciled). Reset cause
`0x2` (external, clean — no watchdog lockup all session).
**Log:** `logs/device-monitor-260818-105317.log`.

---

## 0.0 🔴🔴 THE MOST SERIOUS FINDING — A MISSED DEPARTURE AT 20 fpm UP

**Observed live, 2026-08-18, and it outranks everything else in this note.**
Dave: *"no alert or sound"* … *"alert on stop"* … *"still alerting at rest"*.

At **20 fpm up** the departure was **never detected**. The raw log shows ~**16 s
of real motion with the FSM still in `st=2` MONITORING** (`t=3462000` →
`3478061`): lateral `m` running **0.05–0.20** against an at-rest baseline of
0.007–0.03, `q=0` continuously, `x`/`y` swinging, `a` variance up. The car was
moving the whole time and nothing latched.

### The failure chain

| # | what | consequence |
|---|---|---|
| 1 | 20 fpm up departure too gentle for any-motion (Z) to fire | **departure MISSED — ~16 s of unannounced counterweight travel** |
| 2 | the backstop for exactly this case is **compiled OFF**: `VEL_ARMED 0` (`velocity.h:236`), **zero `VEL: departure` lines all session** | no second chance |
| 3 | even armed, `VEL_DEPART_THRESHOLD` ≈ **22.6 fpm** — still above 20 fpm | would NOT have caught it either |
| 4 | the latch finally fired on the **brake transient at the stop** | "alert on stop" |
| 5 | no arrival transient left to release it → beacon stuck ON over a stationary car, confirmed **81 s** in and counting | **position lie, to the 600 s `LATCH_FAILSAFE_MS`** |

🔴 **`main.cpp` §14.4 states a MISSED DEPARTURE IS THE ONLY CATASTROPHIC
FAILURE, and the spec requirement is 20 fpm. This is a live demonstration that
the device does not meet that requirement in the up direction.** Item 3 of the
chain means the requirement is not merely unmet by tuning — the only armed
detector is a slope interrupt that a gentle ramp does not trip, and the
integral path that was designed to cover it is disarmed *and* set above the
requirement.

### 0.0a Is the any-motion interrupt orientation-agnostic? YES — inversion is not the cause

Dave asked directly. **The interrupt logic is orientation-agnostic and the
inverted mount is not what caused the miss.** Any-motion is a *delta* detector —
it compares acceleration against a reference, so the static gravity offset
cancels; Z resting at −9.81 instead of +9.81 changes nothing. Threshold
`ANYMOTION_THRESHOLD 32` = **0.153 m/s²**, `ANYMOTION_DURATION 5` (100 ms),
arithmetic confirmed against the datasheet at 0.4883 mg/count
(`falcon_bma456_datasheet_review_2026-08-07.md` §2).

**Margin is thin regardless:** `ANYMOTION_THRESHOLD` is 0.153 m/s², and the
18 fpm departure on record was **−0.116 m/s² — already below threshold**, caught
only because instantaneous 100 Hz slope briefly exceeded the ramp average. That
is detection by luck rather than by margin.

🔴 **BUT A SIMPLE SPEED THRESHOLD DOES NOT EXPLAIN THE MISS — see §0.0b.**

🟠 **AND THIS MAY BE THE FIRST LIVE INSTANCE OF A FLAGGED HAZARD.** The 08-07
datasheet review recorded, as safety-critical:

> Any-motion compares against a **latched reference sample**, not the previous
> sample. Read literally that permits a state where **departure detection dies
> silently**; the measured data says it does not happen, **and nothing checks.**

Departure detection died silently for ~16 s today — exactly that failure mode.
⚠️ **The log cannot distinguish "ramp genuinely under threshold" from "stale
latched reference", and that ambiguity exists precisely BECAUSE nothing checks
the reference.** Both readings converge on the same conclusion: the only armed
departure detector has no margin at the spec's own 20 fpm requirement, and there
is no backstop.

### 0.0b ⭐ THE MISS IS INTERMITTENT, NOT A SPEED THRESHOLD — and that changes the fix

Three low-speed up runs, same mounting, same session:

| run | speed | departure | arrival | outcome |
|---|---|---|---|---|
| 5 | 20 fpm | 🔴 **MISSED — ~16 s undetected** | — | latched on brake, stuck alerting at rest |
| 6 | 20 fpm | ✅ caught | 0.499 (**1.11× gate**) | correct |
| 7 | **18 fpm** | ✅ caught, latched ~330 ms after onset | 2.040 (4.5× gate) | correct, `bz=1` at **+672 ms** |

🔴 **Run 7 is the important one: it is SLOWER than the run that was missed, and
it was caught cleanly** (`ratio=0`, `opk=146`, arrival 4.5× gate). A pure
"20 fpm is below the detection threshold" story predicts the 18 fpm run fails
harder. It did not.

**So the miss is intermittent, not speed-determined.** That shifts the weight
between the two candidate explanations:

| explanation | fits an intermittent miss? |
|---|---|
| ramp genuinely below `ANYMOTION_THRESHOLD` | ⚠️ weakly — predicts a consistent speed cutoff, contradicted by run 7 |
| 🟠 **stale latched reference** (`falcon_bma456_datasheet_review_2026-08-07.md` §3) | ✅ **strongly — a stale reference fails sporadically, independent of speed** |

⚠️ **n=3. This is a hypothesis, not a conclusion** — one miss in three runs
cannot separate "rare" from "coin flip", and this tree's own repeated lesson is
that a single run is a hypothesis until a second under the same conditions
agrees. **But it redirects the bench work:** the first move is NOT to lower
`ANYMOTION_THRESHOLD`, it is to **instrument the any-motion reference** and find
out why the interrupt sometimes does not fire. Lowering the threshold would be
tuning against the wrong mechanism and would cost false-fire margin for nothing.

⬜ **Highest-value remaining car test: repeat 20 fpm up ~10 times** and count
misses. That single number — rare vs coin-flip — sets the severity of the worst
item in the project.

⚠️ **Direction-asymmetric:** four 20 fpm **down** runs alerted correctly and
fully in the same session (§0.1); the **up** run was missed. Consistent with the
2026-08-12 "up is weaker than down" asymmetry, now with a missed departure
rather than merely a thin arrival margin.

### 0.1 The 20 fpm batch, for the record

| run | dir | ratio | opk | verdict | arrival | Dave |
|---|---|---|---|---|---|---|
| 1 | down | 63 | 199 | RUN | 2.058 | LED + sound, full alert |
| 2 | down | 96 | 184 | RUN | 1.964 | LED + sound, full alert |
| 3 | down | 85 | 256 | RUN | 1.550 | LED + sound, full alert |
| 4 | up | 0 | 142 | RUN | **0.478** | LED + sound, full alert |
| 5 | **up** | 97 | 664 | RUN | — | 🔴 **no alert; alert on stop; stuck alerting at rest** |

⚠️ **Run 4's arrival peak is 0.478 against the 0.45 gate — a 1.06× margin**,
i.e. item 2 of the readiness list (release-on-luck) reproducing at low speed.

⚠️ **The jog verdict's ratio axis is USELESS at 20 fpm.** Runs 1–3 and 5 ran
ratio **63, 96, 85, 97** — squarely inside the historical *jog* band (46–95) —
and were classified RUN **only because `opk` stayed low**. The `AND` guard has
collapsed to a single axis at this speed: `opk` alone holds the beacon on. Any
slow run that picks up an opk spike over 900 releases with no second axis to
veto it. Run 5 already reached **opk 664**.

## 0.0b-FIX ✅ ROOT CAUSE FOUND AND FIXED IN-SESSION — the 6 s re-arm blank

**The missed departures were not a sensor problem. The firmware was discarding
the departure edge on purpose.**

`MovementService::notify_any_motion()` discarded any any-motion edge arriving
within `MONITOR_REARM_MS` (**6000 ms**) of entering STATE_MONITORING — a blanking
window added 2026-08-07 to stop one trip alarming twice. Because **cruise
generates essentially no any-motion edges**, the departure edge is the ONLY edge
until the brake: discard it and the whole run is invisible, then the brake set
latches and the beacon sounds over a car that has already arrived. That is
exactly the observed failure, and it is why it looked like "rapid departures
after a reset are missed" — a departure inside 6 s of the previous release was
thrown away by design.

**Evidence:** 32 `any-motion ignored, re-arm blanking` lines across the session;
4 of the 6 misses carry one for their departure edge.

### The patch (option 1 — rest-gated, flashed and tested in-session)

End the blank when the car has **actually settled** instead of after a fixed 6 s:
`lat_monitor.quiet_run() >= MONITOR_REARM_QUIET` (8), floored by
`MONITOR_REARM_MIN_MS` (1500) and still **capped by the original 6000 ms**, so
the window can only ever get shorter, never longer. Latched per MONITORING
episode. Flash 31714 (+250), RAM 1358 (+1).

### ✅ RESULT — measured on the car, same session

| | before | after |
|---|---|---|
| **departures missed** | **6 of 12 (50%)** | **0 of 14** |
| blanking discards | 32 | **0** |
| blind window | fixed 6000 ms | **~1550 ms measured** (1507–1573, n=14) |

🔴 **THE DECISIVE NUMBER: the 14 post-patch departures latched at 2883–4325 ms
after MONITORING — every one INSIDE the old 6000 ms blank.** Under the previous
firmware every one of them would have been discarded. This confirms both the
diagnosis and the fix from a single dataset.

⚠️ **Not yet observed but still the risk to watch:** the 2026-08-07 regression
this window was guarding — an alarm that STARTS on an already-stopped car (73 s
on a stationary car back then). None seen in 14 runs; every RUN-verdict run had a
real arrival and no failsafe releases fired. **Revert path if it appears: set
`MONITOR_REARM_QUIET` to 0.**

### ⭐ The post-patch batch was 14/14 CORRECT

Dave confirmed the 3 JOG-verdict events (`opk` 2341/2974/2833) were his own
**deliberate jog-resets**, not real runs. So the batch decomposes as:

| | n | result |
|---|---|---|
| real up runs | **11** | **11 caught, 11 correctly classified RUN, all with a real arrival** |
| deliberate jogs | **3** | **3 correctly classified JOG** |

**No missed departures, no misclassifications, in either direction.** The jog
verdict's discrimination was perfect in this batch — consistent with item 9 being
triggered by a departure jolt rather than being broken in general.

⬜ **Still open after this fix:** arrival margins remain thin — **0.480 and
0.495** against the 0.45 gate (1.07×, 1.10×). Readiness item 2 is untouched by
this patch. Item 9 (jog verdict silencing real jolt-heavy runs) is also untouched
and was simply not provoked here.

## 0.0c 🔴🔴🔴 40% MISSED-DEPARTURE RATE ON SLOW UP RUNS — the worst measured result in this project

☠️ **THIS SECTION WAS FIRST WRITTEN AS "43% of runs silenced by the jog verdict"
AND THAT WAS WRONG.** The four failures are **MISSED DEPARTURES**, not jog
silencings. Confirmed by Dave directly: *"no beacon during travel"* — the beacon
never fired while the car was moving; he stopped, it alarmed on brake set, and he
jogged to silence it. The jog was **not** a separate movement.

### The actual failure sequence, 4 times in 10 runs

```
car travels up SLOWLY  ->  NO DETECTION, beacon silent, counterweight moving
car stops              ->  BRAKE SET latches the alarm
                       ->  beacon ON over a STATIONARY car (position lie)
Dave jogs to clear     ->  burst captures BRAKE HAMMER + JOG = bidirectional
                       ->  verdict JOG -> release
```

**Every piece of evidence fits, including the thing that misled me:** the
bidirectional impulse (`pos` 8597–22305 AND `neg` 4829–14330) is not a jog and
not a jolt-on-a-real-run — it is *brake + jog captured in one burst*, because the
latch happened at the stop rather than at the departure.

### 🔴 The number

**4 of 10 slow up runs from the lower landing were never detected = 40% miss
rate** — and with two further runs logged the same way, **6 of 12 = 50%.** On the
primary safety function, in the primary use case. `main.cpp` §14.4: *a missed
departure is the only catastrophic failure.*

### 🔎 LEADING MECHANISM — a stale edge-triggered any-motion assertion

Dave asked: *why are departures after a long rest caught, but rapid departures
after a reset not?*

**Any-motion is configured EDGE-TRIGGERED on RISING** (`arduino_bma456.cpp`, the
INT pin block). During an alarm the piezo itself drives any-motion continuously —
already documented: *"the sensor's own engine triggers continuously on the
buzzer"* (2026-08-07 bench, Eng_Notes §11). **If the INT pin is still asserted
when the next departure begins, there is no rising edge — no interrupt, no
latch.** A long rest lets the pin de-assert, so the next departure produces a
clean edge and is caught. `sk` in `ACC-STAT` is the firmware's own stuck-poll
counter for precisely this state.

Evidence, stuck-poll counter in the 20 s before each departure:

```
MISS: run 3 (sk=1)  run 6 (sk=2)  run 7 (sk=2)  run 10 (sk=3)  run 11 (clean)  run 12 (clean)
ok:   run 1 (clean) run 2 (clean) run 4 (sk=3)  run 5 (clean)  run 8 (clean)   run 9 (sk=1)
```

⚠️ **SUGGESTIVE, NOT CONCLUSIVE.** `sk>0` precedes 4 of 6 misses — but also 2
successes, and 2 misses had a clean counter. And the reset→departure gap does
**not** correlate at all (successes at 7.4 s, misses at 12.5 s; §"TIMING IS NOT
THE DISCRIMINATOR"). So a stale assertion is the leading candidate, not a proven
cause. An `ACC-STAT read FAILED` (I2C error) also appears in this batch.

⭐ **TARGETED FIX TO TEST FIRST — small, and it aims at the project's most
dangerous failure.** On entering `STATE_MONITORING`, explicitly re-read and clear
the any-motion status and re-arm the interrupt, so a stale assertion from the
previous alarm cannot mask the next departure. Pair with continuous INT-pin
logging (currently sampled only ~1 s). If the misses vanish, the mechanism is
confirmed and closed in one move. **Unlike the jog-verdict redesign this is a
contained change — put it at the top of Session D.**

### ⚠️ INSTRUMENT LIMITATION THAT NEARLY BURIED THIS

**Lateral `m` does not reveal slow up travel** — it reads 0.02–0.05 during these
runs, indistinguishable from at-rest. The 16 s sustained `m=0.05–0.20` signature
from the §0.0 miss is NOT general; it does not appear at these speeds. **A missed
departure therefore leaves almost no fingerprint in the log**, and an automated
pre-latch-motion check (which this analysis used at first) reports "no miss"
on a run that was in fact entirely undetected.

**The reliable signature is structural, not amplitude-based:** a latch with no
preceding departure, whose departure burst is bidirectional because it contains
the brake set plus the operator's recovery jog. ⬜ **Nothing in the firmware
distinguishes "latched at departure" from "latched at the stop" — that is the
single highest-value instrumentation change available**, and without it the log
cannot self-report the project's most dangerous failure.

### The six runs that DID work — arrival margins still marginal

`0.461, 0.506, 0.460, 1.144, 1.844, 0.462` against the **0.45** gate. Four of six
sit at **1.02×–1.12×**, with three within 0.002 of each other just above the
gate. Readiness item 2 ("releases on luck") is systematic at low speed, not a
350 fpm outlier (lifetime worst before today was 1.009×).

## 0.0c-old ~~43% of ordinary slow up runs are silenced~~ — RETRACTED, see above

**Car positioned at the lower landing, slow up runs, one after another. This is
the primary inspection operation, not an edge case, and it is the strongest
evidence in this note.**

```
RUN 1  ratio=2   opk=164   RUN   arrival 0.461   (1.02x gate)
RUN 2  ratio=0   opk=111   RUN   arrival 0.506   (1.12x)
RUN 3  ratio=64  opk=2706  JOG   *** SILENCED ***
RUN 4  ratio=0   opk=92    RUN   arrival 0.460   (1.02x)
RUN 5  ratio=0   opk=102   RUN   arrival 1.144
RUN 6  ratio=58  opk=972   JOG   *** SILENCED ***
RUN 7  ratio=43  opk=1206  JOG   *** SILENCED ***
RUN 8  ratio=0   opk=98    RUN   arrival 1.844
RUN 9  ratio=0   opk=49    RUN   arrival 0.462
RUN 10 ratio=73  opk=1459  JOG   *** SILENCED ***
```

☠️ **The "SILENCED" labels above are WRONG — those four rows are MISSED
DEPARTURES (latched at the brake, then jogged clear). See §0.0c. The table is
kept only as the raw record of what the log printed.**

Real-run `opk` population 49–164; the four missed-departure rows show `opk`
972–2706 because their burst contains the brake hammer plus the recovery jog.

### ☠️ TIMING IS NOT THE DISCRIMINATOR — two hypotheses killed by this batch

Gap between the previous alarm reset (STATE_MONITORING) and the next departure
latch, against the verdict:

```
verdict RUN (good):     7356, 9110, 9929, 12010, 137920 ms
verdict JOG (silenced): 8962, 9290, 10567, 12517 ms
```

**Complete overlap.** The SHORTEST gap in the batch (7.4 s) was a success; the
longest silenced gap (12.5 s) was a failure.

1. ☠️ **Dave's "rapid runs right after the alarm resets cause failures" — NOT
   SUPPORTED.** Tested directly, no correlation.
2. ☠️ **AND THE §4 MECHANISM IN THIS NOTE IS WRONG TOO.** §4 concluded the
   discriminator was *run duration vs the 3.2 s window*, and that spacing runs
   out would fix it. **Every run in this batch was spaced 7–12 s and 40% still
   failed.** Spacing does not protect. §4 stands only as the explanation of the
   *morning's* rapid-reversal batch; it is NOT the general mechanism.

**What actually separates them is the departure jolt itself** — silenced runs
`opk` 972–2706, successful runs 49–164, at the same speed, direction, mounting
and spacing. The variable is whether that particular start produced a mechanical
jolt (brake release / drive pickup roughness), which is not under the operator's
control.

🔴 **Consequence: there is NO operational workaround.** Waiting, spacing, or
avoiding rapid reversals does not avoid the failure. A mechanic cannot work
around this by how they run the car.

⚠️ This is the **twelfth** interpretive claim in four sessions overturned by the
next measurement, and the second one *in this note* (see also §4 vs §5). The
pattern holds: a mechanism that explains one batch is a hypothesis until the
next batch agrees.

### Why the gate cannot simply be raised

The `opk` populations look bimodal — real 92–164, silenced 972–2706 — which
invites "raise the 900 gate". **Run 6 refutes that: `opk=972`, clearing the gate
by 8%.** A real run landed 7× above the normal real-run population. The
distribution is not two clean clusters; real slow runs occasionally throw a
departure jolt of arbitrary size, and no gate placed on this axis separates them
from jogs. ⚠️ Raising the gate also directly re-opens the false-JOG-acceptance
direction the verdict exists to prevent.

### 🔴 And the arrival gate has no margin either

Three of the four successful runs released at **0.460, 0.461, 0.506** against
the **0.45** gate — **1.02×, 1.02×, 1.12×**. Lifetime worst before today was
1.009×. Combined with 0.478 and 0.499 earlier in the session, **every slow up
arrival measured today sits within ~12% of the release threshold.** This is
readiness item 2 ("releases on luck") shown to be systematic at low speed, not
an occasional 350 fpm outlier. A slightly softer stop releases the beacon while
the counterweight is still settling.

### ✅ Item 11 did NOT recur here

Zero missed departures across all 7 runs — every one latched at motion onset,
checked against pre-latch lateral (`m`>0.05 with `st=2`). So the 20 fpm miss
remains ~1 in 4 overall: rarer than the jog silencing, but more catastrophic.

### C5 data (test plan) — arming margin, one mounting

`ro=` **14, 15, 4, 12, 28, 3** — a span of **3 to 28** on a single mounting,
substantially wider than the 6–15 that readiness item 5 was built on. Runs with
`ro`≤4 are the jog-silenced ones (`g=0`).

## 0.0d ⭐ A BUMP IS THE SAME SIZE AS A REAL DEPARTURE — measured, and it settles the "just tune the threshold" argument

**Measured 2026-08-18, end of session.** Dave bumped the mounting by hand while
the device was armed at rest. It latched a departure. The magnitude of that bump
is the most useful single number the session produced.

### The measurement

Raw samples around the latch, against a ~−9.70 rest (inverted mount):

```
t=327778  a=-9.657   +0.043    <- latch
t=328105  a=-9.793   -0.093
t=328695  a=-9.827   -0.127
t=329023  a=-9.623   +0.077
```

Departure burst extremes: `136, 132, 122, -118, -114` mm/s².

**Bump peak ≈ 0.136 m/s² ≈ 0.014 g — a light touch by hand.**

⚠️ Note it is *below* the nominal 0.153 m/s² any-motion threshold **as captured
at 25 Hz**. The hardware engine runs at 100 Hz, and ~20% of samples are lost to
overrun, so the true instantaneous peak fell between logged samples. **The log
under-reads transients by construction** — do not read a sub-threshold logged
peak as "the sensor should not have fired."

### 🔴 The comparison that matters

| event | magnitude |
|---|---|
| **hand bump** | **0.136 m/s²** |
| **real 18 fpm departure** (caught, this session) | **0.116 m/s²** |
| any-motion threshold | 0.153 m/s² |

**A deliberate hand bump is the SAME SIZE as — in fact slightly larger than — a
genuine slow departure.**

⛔ **THEREFORE NO AMPLITUDE THRESHOLD SEPARATES THEM.** Raising the threshold to
reject a bump rejects real departures at the spec's own low-speed requirement.
Lowering it to gain departure margin admits every knock, door slam and handling
transient. **The departure detector cannot be fixed by tuning `ANYMOTION_THRESHOLD`
in either direction** — which retires the instinctive response to the missed
departures (§0.0c) as well as the instinctive response to false latches.

### ⭐ But the SHAPE does separate them, and the firmware already measures it

```
bump:       122  136  -114   77  132   28  -118  -90    <- ALTERNATING = ringing
departure:  -229 -287 -309 -269 -293 -406 -303 -241     <- ONE-SIGNED = ramp
```

A knock rings; a departure ramps one way and stays there. **The jog verdict's
`opk`/`ratio` pair measures exactly this distinction** — but the bump produced
`opk=118`, far below the 900 gate, so it was classified `RUN` and latched.

🔴 **The same knob, two opposite failures.** Lowering `opk` would reject bumps
like this one — and would also silence more real jolt-heavy runs (item 9,
§0.0c). Raising it admits more bumps. **This is independent evidence that `opk`
alone cannot carry the departure/not-departure decision**, and it points the
redesign at *shape over a longer window* (sustained one-signed ramp) rather than
at any single-sample amplitude gate — the same conclusion D1 reached for the
arming gate from the opposite direction.

### Context: the latch that followed

The bump latch went MOVING and could not release — at rest `pk=0.04` against the
0.45 arrival gate, so the release condition is physically unreachable. It ran to
the failsafe. ⚠️ **This is the first field observation of the real cost of the
2026-08-14 `LATCH_FAILSAFE_MS` 240 s → 600 s change: any false latch is now a
TEN-MINUTE alarm**, on a device the mechanic often cannot easily reach. That
change was made on reasoning alone ("silence over a moving counterweight is the
worse failure" — correct), but the bill is now measured, and a ten-minute false
alarm is exactly the kind of thing that trains people to ignore a beacon.

✅ **Not caused by the re-arm patch:** the bump latched **114,557 ms** after
MONITORING — far outside both the old 6000 ms window and the new ~1550 ms one.
The pre-patch firmware would have latched it identically.

⚠️ **Also outside the designed sequence:** the spec has placement happening
*before* power-on precisely so handling transients never reach an armed
detector. Handling an armed unit mid-deployment is not a case the design claims
to cover — but it is a case that will happen in the field.

## 0.0e ☠️ "WINDOWED JOG TEST LIKE THE RAMP DETECTOR" — REPLAYED AND REFUTED

Dave asked: *"windowed jog similar to ramp?"* — following §0.0d's conclusion that
shape, not amplitude, is the discriminator. **Replayed against all 177 departure
bursts on file (`graph/jog_window_replay.py`, written for this) and it does not
work.** The §0.0d conclusion that pointed at it is retracted with it.

### The measurement

Best-block mean and directionality (ramp detector's own math, `RAMP_BLOCK_N` 12,
`RAMP_DIR_PCT` 85) against the shipping `opk` verdict, on today's labelled log:

```
real runs (opk low):   blkmean 148-475 at dir 100%  ... and also 26, 33, 38, 61, 66, 79, 103
jogs      (opk high):  blkmean  83-1017, MANY at dir 100%  (134, 225, 206, 279, 202, 260, 269 ...)
```

**Complete overlap, on both axes.** Across all 177 bursts the floor sweep gives
33–50 disagreements with the shipping gate at *every* floor from 80 to 300 — no
setting reconciles them.

### ⭐ WHY it fails — the principled reason, worth keeping

**A jog's departure IS a real departure.** The car genuinely accelerates, so its
departure ramp is one-signed and sustained, arithmetically identical to a real
run's. The difference between a jog and a run is not in the departure at all —
it is the REVERSAL that follows. **And the reversal is exactly what `opk`
measures.**

🔴 **So `opk` is on the RIGHT axis, and §0.0d's "the redesign should key on
sustained one-signed shape over a longer window" was WRONG for the jog
question.** Shape-of-departure cannot separate a jog from a run because there is
nothing there to separate.

⚠️ **METHOD NOTE — the first version of this comparison was circular.** It scored
the windowed test against "bursts the shipping gate calls JOG/RUN", and those
labels ARE `opk`-thresholded by definition, so `opk` separating them perfectly
(691 vs 972) is a tautology, not evidence. Only the overlap on the *proposed*
statistic carries information. **This is the same trap as scoring a rule against
its own output; do not repeat it.**

### ✅ What DID survive — a different question, a different instrument

The bump from §0.0d separates cleanly on the windowed statistic:

```
bump        blkmean  33   dir  75%
real runs   blkmean 148-475  dir 100%
```

because a knock has **no sustained ramp at all**. That is a different question
from the jog question:

| question | right instrument |
|---|---|
| "was this a departure at all, or a knock?" | **windowed shape** — works |
| "was this departure part of a jog?" | **`opk` / reversal** — shape cannot help |

So the windowed test's place is a **pre-filter that rejects non-departures**, not
a replacement for the jog verdict.

### ⛔ BLOCKED ON THE SAME GAP, AND THIS IS THE REAL ACTION ITEM

Several `RUN`-labelled bursts score low (26, 33, 38, 61, 66, 79, 103). They are
either **genuine slow departures** — in which case a floor of 120 manufactures
MISSED DEPARTURES, the catastrophic direction — or they are **knocks mislabelled
as runs**. **The logs contain no ground truth and nobody can tell which.**

⬜ **THE HIGHEST-VALUE NON-CAR WORK IS NOW LABELLING, NOT ALGORITHM DESIGN.**
Every rule proposed today (this one, D1, the `opk` retune) is unfalsifiable
against a 177-burst corpus with no labels. A labelling pass — even just marking
each burst run/jog/knock from the session notes and the operator's own calls —
converts the whole corpus into something a replay can actually score.

## 0. The headline

**The jog verdict silences the beacon on a real, moving counterweight whenever
the run is short enough (a rapid reversal, ~<3 s) that its own stop falls inside
the 3.2 s departure-classification window** — where a departure-then-hard-stop is
indistinguishable from a jog. Reproduced three independent ways today, including
a controlled settled/rapid pair, and confirmed by the departure burst arrays
(§3.1) and the code (§4). This is a **release-path failure** — silence over a
moving counterweight — and it triggers during the *exact* rapid short-burst
inspection operation the use-case spec
(`falcon_spec_primary_usecase_2026-08-09.md` §1) calls "the normal case, not an
edge case."

The discriminator is **run duration vs the 3.2 s window** — **not direction, not
orientation, not settling time.** Dave confirms it reproduces on a real
counterweight frame (rapid cancels as jog, spaced runs work), which the
orientation-independent mechanism in §4 predicts.

🔴 **BUT READ §5.1 FIRST.** The code says the beacon sounds for ~3 s on every one
of these runs before the verdict releases it; **Dave heard nothing, nine times.**
That conflict is unresolved and is now the most important open item from this
session — it may mean the audibility of short alerts, not the jog verdict, is
the real defect. Two cheap tests in §5.2 settle it.

---

## 1. The clean split

Every run today sorts perfectly by the jog verdict's opening peak `opk` and its
sign-reversal `ratio`, and that sort is identical to "did the mount have time to
settle before departure":

| start condition | `opk` | `ratio` | verdict | beacon | n |
|---|---|---|---|---|---|
| **settled / first from rest** | **38 – 85** | **0** | RUN ✓ | sounds | 6 |
| **rapid reversal (ringing)** | **1457 – 2390** | **54 – 78** | JOG ✗ | **silent** | 5 |

**No overlap.** Settled departures top out at `opk` 85; ringing departures floor
at 1457 — a ~20–45× gap with nothing in between. Every settled run alerted;
every ringing run was silenced.

---

## 2. The controlled pair (the clincher)

Two up runs back to back, the only variable being settle time before departure:

| run | wait before | `opk` | `ratio` | verdict | beacon | arrival |
|---|---|---|---|---|---|---|
| **settled** | ~20 s | **38** | 0 | RUN ✓ | on | 2.453 (5.5× gate) |
| **rapid** | immediate | **1717** | 54 | JOG ✗ | **silent** | — released |

45× on `opk` from settle time alone. Prediction made before the runs, confirmed
by both the log and Dave by ear ("settled clean, rapid failed as predicted").

---

## 3. The rapid batch — 3 of 4 silenced

"Run down stop, run up stop, repeatedly." First run began from rest; the rest
were rapid reversals.

| # | start | `opk` | ratio | verdict | beacon | travel |
|---|---|---|---|---|---|---|
| 1 | from rest | 53 | 0 | RUN ✓ | on | ~23.6 s sustained |
| 2 | rapid | 1684 | 78 | JOG ✗ | silent | ~3.7 s, oscillating |
| 3 | rapid | 1859 | 64 | JOG ✗ | brief `bz=1` **then killed** | short |
| 4 | rapid | 1457 | 68 | JOG ✗ | silent | short |

Run 3 is the tell: `bz=1` appeared — **the beacon started sounding** — and the
jog release then cut it. The alert path works; the verdict overrides it.

### 3.1 The departure burst arrays (the proof)

Signed mm/s², oldest first, trigger at sample 20. Contrast a RUN with a JOG:

**RUN** — one-signed departure ramp, nothing on the opposite side:
```
-60 -71 ... -229 -287 -309 -269 -293 -406 -303 -241 -290 ...   opk=82  ratio=0  → RUN
```

**JOG (rapid)** — clean ramp for ~1 s, THEN the brake/reversal hammer:
```
... -309 -269 -293 -406 -303 ... -163 -11 -29 -15 | -1944 704 2379 607 -1624 1352 -1324 ...
                                    (departure ends)  ^-- stop/reversal hammer, ~1.2 s post-latch
opk=2379  ratio=70  → JOG
```

The hammer is **post-trigger** — it is the current short run's own stop, not
leftover from the previous run. A rapid reversal packs departure *and* stop into
the 3.2 s window, so the burst is shaped exactly like a jog. At 35 fpm even a
~1.5 s burst is **~0.4 m of counterweight travel — unannounced.**

---

## 4. Mechanism — CORRECTED from the burst arrays

⚠️ **An earlier draft of this note blamed "residual ringing from the previous
stop." The departure burst arrays do not support that and it is withdrawn.**
The opposite-sign hammer is *post-trigger*, not pre-trigger — it is the current
run's own stop, not the previous run's aftermath.

The jog verdict (`main.cpp` ~1836) classifies each departure over an **80-sample
(~3.2 s) burst window**, deadbanded at 150 mm/s². It declares JOG when
`ratio ≥ 33% AND opk ≥ 900`, where **`opk` is the OPPOSITE-sign peak** — the
signature of a hard brake-set/rollback ("the uncontrolled brake set hammers
1.9–3.2 into the structure" — the code's own design note, i.e. Dave's jog
mechanism). Real departures are one-signed (`opk` historically 27–396); jogs
carry the brake hammer (`opk` 1914–3194).

1. A **rapid reversal is a short run** — travel lasts ~1–1.5 s before the stop.
2. The jog verdict's window is 3.2 s, so **the run's own stop and reversal fall
   inside its own departure-classification window.**
3. The burst therefore reads: clean one-signed departure ramp, **then** the
   brake/reversal hammer (±1900–2400 mm/s²) ~1.2 s in. See §3.1.
4. That is *definitionally* a jog — a short movement bounded by a hard stop —
   so the verdict rejects the real run and **releases the beacon.** The metric
   cannot separate "a 1 s real run that then stops" from "a jog"; they are the
   same event by this measure.
5. Constant-velocity cruise has no acceleration signature, so once released the
   device sees nothing until the brake — the beacon stays silent for the run.

**Why waiting fixes it:** a spaced-out run is also a *longer* run (>3.2 s), so
its stop falls **outside** the departure burst, leaving a clean one-signed ramp
→ RUN. The discriminator is **run duration vs the 3.2 s window**, not settling
time and not orientation.

---

## 5. ☠️ RETRACTED — "the beacon is gated behind the jog verdict"

**This was wrong, and it was written into an earlier draft of this note and into
the readiness memory before the code was read. It is withdrawn.**

The firmware **already fails loud.** `enable_alarm()` fires on the
MOVEMENT_DETECTED → MOVING transition, and `MOVEMENT_DETECTION_TIMEOUT_MS` is
**200 ms** (`movement_service.h:96`). The jog verdict lands at **3.2 s**. So on
every jog-released run the designed sequence is:

```
t+0.2 s   beacon ON   (enable_alarm at STATE_MOVING)
t+3.2 s   JOGV -> FSM: Release (jog verdict)
```

**≈3 seconds of beacon on every "silenced" run.** Verified: all 9 JOG-failed
runs today reached `STATE_MOVING` — the line `enable_alarm()` immediately
follows. "Sound on departure, release only on a confirmed jog" is not a fix to
propose; **it is the shipping design.**

## 5.1 🔴 THE REAL OPEN CONFLICT — code says ~3 s of beacon, Dave heard NOTHING

Dave, on the cartop beside the device, asked explicitly whether the rapid runs
were truly silent or a short burst he discounted: **"truly silent, nothing."**
Nine times. That is irreconcilable with §5 and one of the two must be wrong:

| # | hypothesis | consequence if true |
|---|---|---|
| A | **Defect in the alarm output path** — `enable_alarm()` runs but no blast actually lands in a ~3 s window | **WORSE than the jog verdict.** Short alerts are silent regardless of verdict, and the verdict is a red herring for the audibility problem |
| B | **It sounded and was inaudible in situ** — ~3 s at 18.75% duty is only 3–4 blasts of 150 ms | The beacon works as designed and is *still* not audible on a short movement. ⭐ Recall the duty was cut 2/5 → 1/5 **to fight the brownout phantom that did not exist**, knowingly trading audible length on a device the mechanic ranges BY EAR. This is that bill arriving |

**Either way today's session is more serious than "the jog verdict misfires",
not less.**

⚠️ **The log cannot settle it.** `bz` is printed **only on ACC-INT lines**, not
on the periodic sample line, so it is sampled only when an any-motion interrupt
happens to fire. **Absence of `bz=1` is NOT evidence of silence** and must not be
cited as such. (8 `bz=1` lines appear across the session, incl. `n=45` inside a
jog-released run — consistent with the beacon running, but far too sparse to
prove either case.)

### 5.1a MEASURED — the alarm timing is exact, and the piezo WAS driven

Latch → STATE_MOVING → verdict, measured across all 26 departures this session:

```
->MOVING  +311..328 ms   (MOVEMENT_DETECTION_TIMEOUT_MS = 200, + loop latency)
->JOGV    +3031..3359 ms
```

**Dead consistent, every run.** So `alarm_status_g` is set ~320 ms after every
latch and cleared ~3.1 s later: **~2.7 s of alarm-on state on every
jog-released run.**

✅ **Stale-verdict overlap RULED OUT.** Dave's hypothesis was that in rapid
succession the *previous* run's burst is emitted during the *next* run's MOVING
state (`ms.jog_release()` checks only `state == STATE_MOVING`, not which run the
verdict belongs to — a real structural hazard). But every verdict landed
3.03–3.36 s after **its own** latch. Not what happened today. ⚠️ The hazard is
still real in the code and worth closing defensively.

🔴 **And the piezo was actually driven inside jog-released runs.** `bz=1` (pin
driven within the last 50 ms) appears inside JOG runs at **+606, +606, +1835,
+2016, +3834 ms** after the latch. Combined with Dave confirming the piezo is
audible and that a deliberate jog fires the beacon up and down, **the alarm
output path works.**

### 5.1b ⚠️ UNRECONCILED — instrument says it fired, Dave says it did not alert

Dave, explicitly, after being asked to separate audibility from alerting: *"its
not a matter of me hearing the piezo, the beacon didnt alert."* This is
**not** reconcilable with §5.1a by measurement alone, and it is the open item.

Candidate reconciliations, distinguishable by ONE observation (watch the RING
LEDs on a rapid run, ignore sound):

| observation | meaning |
|---|---|
| LEDs sweep ~3 s then stop | beacon IS alerting; the defect is **duration** (the jog release) — item 9 stands as written |
| piezo chirps but LEDs never light | **separate chase-LED defect** — `enable_chase_leds()` is a different flag from `enable_alarm()`; "didn't alert" would be literally correct |
| neither fires | something upstream of both flags — Dave's arm/disarm overlap instinct is right; instrument `bz`+chase on the periodic line and settle with data |

☠️ **A masking hypothesis (beacon drowned by machine noise while moving) was
raised and is WITHDRAWN** — the beacon is visual as well as audible; LEDs cannot
be masked.

### 5.2 Two cheap tests that split A from B — DO THESE FIRST NEXT SESSION

1. **Is the piezo audible at all?** Power-cycle; the boot/ready chirp is a
   known-good sound. Ready audible + run-alerts inaudible ⇒ output path is fine
   ⇒ hypothesis B (duty/duration).
2. **A deliberate ~3 s movement**, stopping just before 3 s so the verdict
   fires. Stand still and listen for 3–4 short blasts. That is exactly the
   disputed window.

⬜ **Instrumentation gap to close regardless:** put `bz` (or a blast counter) on
the periodic sample line so beacon state is continuously observable. Today's
central question was unanswerable from a full session log — that is an
instrument defect in its own right.

---

## 6. What is NOT yet established (honesty)

- **~~Rig artefact?~~ RESOLVED — it is not.** The mechanism (§4) is pure timing:
  a short run's own stop inside the 3.2 s window. That is orientation- and
  rig-independent, and Dave confirms it reproduces on a real counterweight frame
  (rapid cancels as jog; spaced runs work). This is a **shipping-relevant
  limitation of the jog verdict, not a property of the upside-down clamp.**
- **Orientation is not the cause.** The signal path was verified
  orientation-agnostic before the session; rightside-up jog record is lifetime
  25/25 clean *because those runs were spaced/long enough to clear the window*,
  not because orientation protected them. Rapid short runs would fail
  rightside-up too.
- **What IS still unquantified:** whether the inverted clamp changes the *size*
  of the stop hammer (it does not need to, to trigger — any brake set exceeds
  the 900 gate). And `opk`/`ratio` are the verdict's own internals, not
  cross-checked against measured displacement this session.
- **The real product question this exposes:** *should a short, rapid
  counterweight movement alert at all?* The jog verdict was built to suppress
  short movements as nuisance. Safety says any movement toward the mechanic is a
  hazard. These conflict, and the conflict — not a tuning error — is what
  produced today's silences. See §7.1.

---

## 7. Fix direction — Session D, bench, replay-gated

⛔ **Not to be flown from the car.** Per the test plan, any arming/verdict-gate
change must be replayed through `graph/arming_replay.py` against all 186 bursts
first. Candidates, in preference order:

1. **Fail loud.** Sound on departure; release only on a positively-confirmed
   jog. Changes the default under uncertainty from silence to sound.
2. **Settled-baseline guard.** Distrust the `opk`/jog gate when the
   pre-departure baseline was still ringing (recent high variance) — treat a
   departure-into-ringing as a run, not a jog.
3. **Blank the opening jolt.** Measure `opk` on a window starting ~200–300 ms
   after departure, past the take-up transient.

🔴 **Re-check against the 2026-08-07 false release before flying any of these.**
That regression came from making the arrival/release path too permissive; a
fail-loud change moves in the same direction and must be replayed against it,
not just against this failure.

---

### 7.1 ⚠️ Fix priority depends on §5.1, so do not start here

The obvious fix — "fail loud" — **is already implemented** (§5). The verdict is
also not "wrong": a rapid reversal genuinely matches the jog shape, so tuning
`opk`/`ratio` cannot fix it without letting real jogs through.

**So the jog verdict may not be the thing to change at all.** If §5.1 resolves
to hypothesis A or B, the beacon is inaudible on short movements *regardless* of
the verdict, and audibility is the defect to fix first. **Settle §5.1 before
spending any Session D effort on the verdict.**

If the verdict does still need work afterwards, the honest framing is the
product question in §6: **should a short, rapid counterweight movement alert at
all?** The verdict was built to suppress short movements as nuisance; safety says
any movement toward the mechanic is a hazard. That conflict — not a tuning
error — is what produced today's releases, and it is Dave's call, not a
threshold tweak.

## 8. Incidental positives this session

- **The inverted mount calibrates equivalently.** 6 s window, `b=6 mv=0`,
  `XY-Still 0.0510` — right in family with the rightside-up in-situ values
  (0.0525–0.0555). First evidence that counterweight-bottom placement behaves
  like the bench for calibration.
- **Reversal arming (`v=1`) fired repeatedly**, e.g. the first down run
  (`ro=25`). This path was "armed, never observed working" in ~32 prior runs
  (open item §5.1); it worked today. Worth confirming whether the inverted
  geometry gives a cleaner reversal or this is incidental.
- **Battery telemetry is live** — `pack_mv` ~5126 (5.13 V, fresh 3×AAA), no
  "(settling, ignored)". The 2026-08-12 settling blindness is gone on this build.
- **No lockups all session** — `sk=0` throughout, watchdog never fired.
- Clean settled arrivals ran 1.185–2.905 (2.6–6.5× the 0.45 gate). Note today's
  up arrivals (2.905, 1.185) straddle the down (2.209); the 08-12 "up weaker
  than down" ordering did **not** hold under the inverted mount. n is small;
  not a conclusion, a flag.
