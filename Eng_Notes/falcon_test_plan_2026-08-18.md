# Open-item test plan

**Date:** 2026-08-18
**Basis:** open items and instrumentation gaps in
`falcon_state_of_project_2026-08-18.md`.
**Build under test:** `26e38df`, flash 32188 / 32256.

Supersedes `falcon_test_plan_2026-08-14.md`, whose sessions A–C are complete and
whose session B was retired.

---

## 1. How this plan is ordered

Three principles, applied in this order.

**Unblock first.** Several measurements cannot be taken with the current
firmware because the log does not record what they need. Instrumentation
therefore precedes measurement, and flash recovery precedes instrumentation,
because there are 68 bytes free and instrumentation costs more than that.

**Rule out what was shipped.** One change in the current build relaxes a guard
that existed for a measured reason. Confirming it has not re-opened that failure
outranks new characterisation, because it is a regression risk the project
introduced rather than one it inherited.

**Then consequence per unit of effort.** A test that closes a release-path item
outranks one that closes an instrumentation item, and a test needing an
elevator outranks one needing only a bench when both are otherwise equal.

### 1.1 Dependency structure

```
A. flash recovery
      |
      v
B. instrumentation ------------------> D. arrival margin population
      |                                E. quiet gate vs cruise
      |                                H. any-motion reference
      v
C. re-arm regression check (independent of A/B, run first if car time is short)

F. second installation, logged  ------> G. threshold transfer
I. corpus labelling -----------------> J. jog verdict redesign
```

Sessions C, F and I depend on nothing and can be run in any order. Everything
else has a prerequisite.

---

## 2. Session A — recover flash headroom

**Bench. No elevator. Engineering, not testing.**

**Open item:** 1.

**Why this is first.** 68 bytes free blocks every instrumentation change in
session B, and B gates three separate measurements. It also blocks
`-DWIRE_TIMEOUT` (1198 bytes), which is the correct fix for the I2C driver's
unbounded waits; the present mitigation is a watchdog reboot that drops the
beacon for the duration of a recalibration.

**Options, in order of expected yield:**

| Option | Expected | Risk |
|---|---|---|
| Compile serial logging out of the shipping build behind a flag | unmeasured; also recovers 1.10 mA | logging is the only field diagnostic — needs a build that keeps it |
| Replace the vendored bma4 driver with the current Bosch API | approximately 4.9 KB | new ASIC feature blob; every any-motion threshold needs re-confirming |
| Remove `brownout_test` remnants and dead V1 paths | small | low |

**Method.** Take the logging flag first and measure the actual saving before
committing to the driver swap. Build both `ATmega328PB` (logging on, for test
sessions) and a production variant (logging off) from the same source.

**Exit criterion.** Either at least 1.5 KB free — enough for `-DWIRE_TIMEOUT`
plus session B's instrumentation — or a recorded decision to defer, naming what
is being given up.

**Caution.** If the driver swap is taken, every any-motion constant in the
constants reference must be re-derived on the bench before the build goes in a
car. A different feature blob is different ASIC firmware.

---

## 3. Session B — instrumentation

**Bench. Depends on A.**

**Open items:** enables 2, 8, 12; closes instrumentation gaps 1, 2, 4, 5.

**Why.** The log currently cannot report the project's most dangerous failure.
A departure latched at the start of travel and one latched at the stop produce
identical lines, so a missed departure has to be reconstructed by hand from
lateral noise — which, at 20 fpm, is indistinguishable from rest. Three open
items are unmeasurable until this changes.

| # | Change | Why it is needed |
|---|---|---|
| B1 | Record and print whether the latch followed a departure or a stop | The signature of a missed departure. Without it, the failure is invisible in the log and was only found because an operator was watching. |
| B2 | Print `bz` on the periodic sample line, not only on `ACC-INT` lines | The log cannot presently establish whether the beacon was sounding at a given moment, so "the beacon did not fire" cannot be confirmed or refuted from data. |
| B3 | Trigger a burst on the polled departure path | A departure caught by the polled path produces no burst and no jog verdict, so the backstop is invisible exactly when it matters. |
| B4 | Capture a cruise-phase peak, distinct from the sticky arrival peak | The cruise ceiling is the denominator of the arrival gate and cannot currently be measured: the sample log is decimated and sticky, and bursts are too short to isolate cruise. |

**Method.** Print-only where possible. B4 requires a small amount of state — a
running peak over a bucket that retires far enough behind the current sample
that the stop transient cannot contaminate it.

**Exit criterion.** A capture from an ordinary run in which departure
provenance, beacon state, and a cruise ceiling can all be read directly.

**Verification.** No behavioural change. Confirm identical FSM transitions on a
replayed bench run before and after.

---

## 4. Session C — re-arm regression check

**Car. Depends on nothing. Run first if car time is limited.**

**Open item:** 9.

**Why.** The re-arm blanking window was shortened from a fixed 6000 ms to a
measured settle of approximately 1550 ms. That window existed because a
terminal-floor brake set re-latched 3.5 s after MONITORING was entered and
produced a 73-second alarm on a stationary car. `STOP_CONFIRM_MS` is the primary
defence and remains unchanged, but this was the backstop, and it is now shorter
than the interval at which that failure was observed.

Twenty-four runs have shown no instance. Those were not terminal-floor stops,
which is the condition that produced the original failure, so the risk is not
yet addressed.

**Method.** Terminal-floor stops, both terminals, logger running. At least ten.
Include the single-floor terminal approach, which is the geometry that produced
the original event. Do not jog between runs; allow the FSM to settle naturally,
since jogging masks the condition under test.

**Exit criterion.** Ten terminal stops with no alarm beginning on a car that has
already stopped. Any instance is a regression: revert `MONITOR_REARM_QUIET` to 0
and record the timing.

**Note.** This is the only test in the plan checking a failure the project may
have introduced, rather than one it inherited.

---

## 5. Session D — arrival margin population

**Car. Depends on B.**

**Open items:** 2, 8.

**Why.** `ARRIVAL_PEAK_VALUE` was derived as the geometric middle between a
worst cruise of 0.28 and a weakest arrival of 0.713. The weakest arrival is now
0.460, so the derivation no longer describes the populations. Four measured
arrivals sit at 1.02–1.03x the gate, and three fall within 0.002 of each other,
which indicates the distribution mode sits on the threshold rather than near it.

The gate cannot be repositioned without both halves. The cruise ceiling has
never been measured in the slow regime, and the two attempts made from existing
captures were both invalid — sticky peak in one case, stop contamination in the
other. B4 exists to supply it.

**Method.** Softest stops the machine can produce, both directions, from both
terminals, at inspection speed. Twenty runs minimum. The weakest arrival is the
number the whole approach rests on and it has moved by a factor of three across
five previous runs, so a small sample will mislead.

**Exit criterion.** A weakest-arrival figure and a cruise ceiling from the same
regime, sufficient to re-derive the gate. State whether the populations still
separate.

**Anticipated outcome worth planning for.** If the separation is below roughly
1.6x, no single gate serves this direction and position, and the arrival path
needs a second axis rather than a new constant. Design work, not a tuning
change.

---

## 6. Session E — automatic operation

**Car with drive control. Depends on nothing.**

**Open item:** 6.

**Why.** `RAMP_ARMED` is 1 and the ramp detector has never executed. Its
negative evidence is complete — it declined every inspection stop, all cruise,
all jogs and all repositioning moves across a full session and 51 replayed
bursts — but an armed release path that has never been observed working carries
the automatic-operation boundary on unverified logic.

Inspection operation cannot produce the input it needs. An inspection stop is a
brake set, which rings; the detector requires a sustained one-signed
deceleration, which only a drive-controlled stop produces.

**Method.** Automatic runs, both directions, several floors, at two speeds
including the machine's rated speed. Record every `FSM: Arrival (ramp)` line
with its block statistics, and every automatic stop that did not produce one.

**Exit criterion.** The detector fires on drive-controlled stops and declines
inspection stops in the same session. If it fires on no automatic stop, it is
not carrying the load it is armed for and should be disarmed until it does.

---

## 7. Session F — second installation, logged

**Hydraulic elevator, second building. Depends on nothing.**

**Open items:** 11, 10, 5.

**Why.** One traction car in one building supplied every logged measurement in
the project, and the arming margin varies with installation, so installation is
a known variable that has never been varied under measurement. A hydraulic
machine has been run successfully but was not logged, and at 150 fpm — far above
the speeds at which every documented failure occurs.

This session also supplies the only untested condition for the self-calibrated
threshold. Both mountings measured so far are quiet enough that the derived
value clamps to `Z_THRESH_MIN`, so per-deployment learning has never actually
produced a learned value. A different machine class is the first realistic
chance of one.

**Method.** Inspection speed, not rated speed. Capture:

- calibration output: `XY: calib`, `Zero-Calib-Value`, `Threshold-Value`,
  `XY-Still-Value`. Note whether `Threshold-Value` reads exactly `0.040000` or
  `0.200000`, which indicates clamping rather than measurement.
- per run: departure provenance, `ARM q=` and `ro=`, arrival peak, verdict.
- at least three separate calibrations, re-seating the device between them.

**Exit criteria.**

1. Whether calibration-derived thresholds transfer, or whether the constants are
   specific to the original installation.
2. Whether `Threshold-Value` exceeds the clamp floor on any mounting.
3. An arming-margin spread for a second installation, comparable against the
   3–28 recorded on one mounting at the first.

**Reasoning on speed.** Running this at 150 fpm again would confirm nothing. All
transients are far above threshold there, which is why the unlogged run
succeeded. The failures live at inspection speed.

---

## 8. Session G — corpus labelling

**Desk. Depends on nothing. No hardware.**

**Open items:** enables 4; prerequisite for any future discrimination work.

**Why.** Four candidate discrimination rules were tested and none could be
scored against the 387-record corpus, because nothing in it records whether a
burst was a real run, a jog, or a disturbance. Each rule was assessed against
the six events an operator happened to label aloud, and the strongest candidate
survived three of those and was refuted by the fourth.

Any future rule faces the same problem. Labelling is the highest-value work in
this plan that needs no elevator and no firmware.

**Method.** Work through `falcon_srcs/datasets/` against the session notes,
marking each departure burst as run, jog, disturbance, or unknown. Unknown is a
valid label and should not be guessed. Record the labelling basis per record.

**Exit criterion.** A labelled corpus, and a count of how many records could be
labelled with confidence. If that count is low, the conclusion is that future
rules need purpose-collected labelled data rather than retrospective labelling.

**Caution.** Do not label from the `JOGV verdict=` field. It is `opk`
-thresholded by definition, so a rule scored against it will report a perfect
separation that means nothing.

---

## 9. Session H — velocity departure path

**Bench, then car. Depends on B.**

**Open item:** 5.

**Why.** `VEL_ARMED` is 0, disabling the only departure detector that can in
principle catch a ramp the others cannot — a car reaching 20 fpm over 4 s
averages 0.025 m/s², roughly six times below `ANYMOTION_THRESHOLD`. The 20 fpm
requirement is precisely where the other two paths are weakest.

It is disabled for a reason: `VEL_DEPART_THRESHOLD` evaluates to approximately
22.6 fpm, above the requirement it exists to meet, so arming it unchanged would
add exposure without adding coverage.

**Method.** Characterise the windowed velocity integral against known slow
departures using existing captures first, then determine whether the threshold
can be lowered to cover 20 fpm without admitting parked noise. The threshold is
derived from measured parked noise, so lowering it is a measurement question,
not a preference.

**Exit criterion.** Either a defensible threshold covering 20 fpm with stated
margin over parked noise, or a recorded finding that the sample rate makes that
impossible — in which case the 20 fpm requirement depends on any-motion alone
and that should be stated plainly.

---

## 10. Session I — disturbance environment on a counterweight

**Counterweight, any installation. Depends on nothing.**

**Open item:** 3.

**Why.** Knocks and slow departures are not separable by amplitude or waveform
shape; a hand bump measures larger than a real 18 fpm departure. Sensitivity is
therefore a design trade, and positioning it requires knowing what disturbances
actually occur in service.

Every disturbance measurement to date was taken on a cartop, where an operator
walks, and walking produced peaks up to 0.87. Nobody walks on a counterweight.
The disturbance population that currently constrains the thresholds may not
exist where the device is actually deployed, and if so, a more sensitive
configuration becomes defensible.

**Method.** Device calibrated and armed on a counterweight frame, logger
running, left undisturbed while the car performs ordinary operation on an
adjacent schedule. Record every latch and the peak that caused it. Several
hours; no operator interaction with the device.

**Exit criterion.** A disturbance peak distribution for the real deployment
position, and a count of false latches over a known interval. This is what
decides whether the current thresholds are conservative, correct, or too loose.

---

## 11. Session J — jog verdict redesign

**Bench and replay only. Depends on G.**

**Open item:** 4.

**Why.** The verdict misclassifies in both directions from a single cause: `opk`
populations overlap. Real jolt-heavy departures reach 972–2974 and are silenced;
gentle jogs reach 381 and are missed. `opk` is nonetheless on the correct axis,
because the reversal following a movement is the only thing distinguishing a jog
from a run — a jog's departure is a real departure.

Any redesign therefore needs a second axis or a different treatment of the same
one, and it cannot be evaluated without labels.

**Gate.** Replay against the full corpus before any car test. Re-check
specifically against the 2026-08-07 false release, in which raising the quiet
fraction to 0.75 let cruise edges pair into an arrival cluster; the present
timing sits at exactly 0.75 and is safe only because both arrival paths also
require `arrival_peak_hit()`.

**Do not attempt** the approaches in the refuted table of the state document
without new evidence. Each failed at the weak end of the real-departure
population.

---

## 12. Item-to-session index

| Open item | Session | Blocked by |
|---|---|---|
| 1 flash headroom | A | — |
| 2 arrival gate margin | D | A, B |
| 3 knock/departure separability | I | — |
| 4 jog verdict | J | G |
| 5 velocity path disabled | H | B |
| 6 ramp detector unevidenced | E | — |
| 7 logging in shipping build | A | — |
| 8 quiet gate vs cruise | D | A, B |
| 9 re-arm regression | C | — |
| 10 z threshold unproven | F | — |
| 11 installation coverage | F | — |
| 12 any-motion latched reference | B | A |

---

## 13. What is deliberately not scheduled

| Not scheduled | Reason |
|---|---|
| Retuning `ANYMOTION_THRESHOLD` | Refuted. A hand bump exceeds a real 18 fpm departure; no value satisfies both directions. |
| Waveform-shape replacement for `opk` | Refuted across 177 bursts. A jog's departure is a real departure. |
| Waveform shape as a knock pre-filter | Refuted. A confirmed 20 fpm departure scored inside the knock band. |
| Raising `JOG_OPP_PEAK_MMSS` | Refuted. A real run was silenced at 972 against a 900 gate. |
| Further 150 fpm runs | Confirms nothing. All transients are far above threshold at that speed. |
| Endurance testing for the brownout | Closed. The failure was in the measuring apparatus. |

---

## 14. Minimum useful subset

If only one bench session and one car session are available:

- **Bench: session A.** Everything else that needs firmware is behind it.
- **Car: session C.** It is the only test covering a failure this project may
  have introduced, it needs no instrumentation, and ten terminal stops is under
  an hour.

If only a desk hour is available, **session G** needs no hardware and unblocks
the largest body of stalled analysis.
