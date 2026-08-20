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
B2, B4  DELIVERED 2026-08-19, no flash recovery needed  (commit 0cacf29)
   |
   +--> D. arrival margin population  (items 2, 8)   UNBLOCKED -- car only

A. flash recovery
      |
      +--> B1, B3 --+--> H. velocity departure path   (item 5)
                    |
                    +--> item 12, any-motion latched reference

G. corpus labelling -------------> J. jog verdict redesign         (item 4)

C. re-arm regression check     (item 9)        no prerequisite
E. automatic operation         (item 6)        no prerequisite
F. second installation         (items 10, 11)  no prerequisite
I. counterweight disturbance   (item 3)        no prerequisite
```

Sessions C, D, E, F, G and I now depend on no firmware work and can be run in
any order. What remains behind session A is B1 and B3, and through them H and
open item 12. J needs G. Session C remains the one to run first if car time is
short; **session D is the one that changed** — its instrumentation prerequisite
was B4, which is delivered, so it is now purely a car session.

**Corrected 2026-08-19.** The diagram above previously carried session letters
from an earlier draft: it named an E for "quiet gate vs cruise", an H for
"any-motion reference" and a G for "threshold transfer", and showed J depending
on a session I. None of those matched sections 2-11 or the index in section 12,
both of which were right. Quiet gate vs cruise is item 8, folded into session D;
any-motion reference is item 12, closed by B itself; threshold transfer is an
exit criterion of F, not a session; and corpus labelling is G, not I.

### 1.2 Added 2026-08-19 — lowering the log decimation is REFUTED, measured

`LOG_DECIMATE_N` is 8. The comment block that sets it, immediately above the
define in `main.cpp`, derives that 8 from holding the sample line at 47% of a
**9600 baud** budget, at 151 ms per line. The link was raised to 62500 on
2026-08-13 (the "SERIAL BAUD" block in the same file), which puts the same line
at about 23 ms. The decimation is therefore roughly 6.5x more conservative than
the constraint that chose it, and nobody re-derived it after the baud change.

This matters because instrumentation gap 5 — "the sample log is decimated and
sticky, so the cruise ceiling cannot be measured" — is half caused by that 8.
Lowering the constant costs **zero flash**, so unlike everything else in session
B it is not behind session A.

**⛔ It was tested on the bench on 2026-08-19 and it fails badly. The standing
warning against logging faster is correct, and understates the failure.**

Controlled pair, same board at rest on the bench, 40 s settle discarded then
90 s measured, N=8 flashed first as the control:

| | N=8 control | N=2 test |
|---|---|---|
| `ov=` advance over 90 s | **0** | **55** |
| `tk=` per printed line | 8.00 (min 8, max 8) | **3.08** (min 2, max 4), against an expected exactly 2.00 |
| apparent sample period | 40.01 ms → 24.99 Hz | **48.10 ms → 20.79 Hz** |
| watchdog resets in window | none | **repeated `Reset cause: 0x8`, with full re-boot and re-calibration** |
| serial duty | 5.7–6.2% | 11.8% |

Roughly 35% of samples never reach the consumer at N=2, the apparent rate falls
to 20.8 Hz, and the device **boot-loops under the watchdog**. This is precisely
the `dt` widening the standing warning predicted, measured directly, plus a
failure mode the warning did not anticipate.

**Why the headroom arithmetic was wrong, and it is worth writing down.** The
serial link is not the binding constraint — the failure occurs at 11.8% duty,
with the link 88% idle. The ring is drained **one sample per `loop()` pass**:
the drain function returns after processing a single sample. So the ring's eight
slots buy latency tolerance, not throughput, and log density throttles the
consumer directly. At N=2 every second pass carries a ~17.5 ms blocking print,
which pushes the mean loop period past the 40 ms sample interval; the ring then
backs up permanently rather than transiently, and depth cannot rescue a consumer
that is slower than the producer. `wdt_reset()` lives in the same starved loop,
which is why the watchdog fires.

**What survives.** Two things. The observation that `LOG_DECIMATE_N` 8 is sized
for a 9600 baud budget is still true and still means nobody re-derived it after
the baud raise — it simply turns out not to be the binding constraint, so there
is no free density there. And the detectors genuinely are fed before the
decimation return, so the constant does not thin them *directly*; the damage at
N=2 arrives by the indirect route above, which is worse rather than better.

**Consequence for gap 5.** The decimation half of instrumentation gap 5 is
**not** cheaply separable. A cruise ceiling has to come from B4 — a bucket
maximum computed on the device, at the current line rate — and B4 is therefore
back behind session A's flash recovery along with the rest of session B. Any
future attempt to raise log density must first make the drain loop consume the
whole ring per pass rather than one sample, and that is firmware work with its
own replay gate, not a constant change.

---

## 2. Session A — recover flash headroom

**Bench. No elevator. Engineering, not testing.**

**Open items:** 1, 7.

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
| Convert `RollingAvg` to static buffers | approximately 586 bytes | low |

The `RollingAvg` row is carried over from session D4 of the superseded
2026-08-14 plan, where it was costed at 586 bytes. It was dropped from this plan
by oversight rather than by decision, and it is still live: `RollingAvg.h` does
`m_avg_array = new T [m_size]`, so malloc and free are linked into the image.
That also defeats the reason the vendored Wire driver avoids `new` — its header
says the point was to keep malloc's 312 bytes and free's 274 out of the build.
At roughly a third of the exit criterion and the lowest risk of the four, it
should be attempted before the driver swap, not after.

**Method.** Take the logging flag first and measure the actual saving before
committing to the driver swap. Build both `ATmega328PB` (logging on, for test
sessions) and a production variant (logging off) from the same source.

**Exit criterion.** Either at least 1.5 KB free — enough for `-DWIRE_TIMEOUT`
plus session B's instrumentation — or a recorded decision to defer, naming what
is being given up.

**Verification.** The logging-off variant is not a print-only change. Removing
the prints removes the blocking `Serial.print` that is the sole cause of ring
overrun, so the production build's detectors see a sample population that no
threshold on file was derived on. Capture `ov=` on the logging-on build under
the same conditions and record the difference, or state plainly that the
production build runs on more data than its constants were measured against.
This is the same class of difference session B verifies for, and session A had
no verification clause at all.

**Caution.** If the driver swap is taken, every any-motion constant in the
constants reference must be re-derived on the bench before the build goes in a
car. A different feature blob is different ASIC firmware.

---

## 3. Session B — instrumentation

**Bench. B2 and B4 are DONE (2026-08-19) and did not need A. B1 and B3 remain,
and those are what still depend on A.**

**Open items:** enables 2, 8, 12; closes instrumentation gaps 1, 2, 4, 5.
Gaps 2 and 5 are now closed by B2 and B4; gaps 1 and 4 remain with B1 and B3.

**Why.** The log currently cannot report the project's most dangerous failure.
A departure latched at the start of travel and one latched at the stop produce
identical lines, so a missed departure has to be reconstructed by hand from
lateral noise — which, at 20 fpm, is indistinguishable from rest. Three open
items are unmeasurable until this changes.

| # | Status | Change | Why it is needed |
|---|---|---|---|
| B1 | outstanding | Record and print whether the latch followed a departure or a stop | The signature of a missed departure. Without it, the failure is invisible in the log and was only found because an operator was watching. |
| B2 | **done** | Print `bz` on the periodic sample line, not only on `ACC-INT` lines | The log cannot presently establish whether the beacon was sounding at a given moment, so "the beacon did not fire" cannot be confirmed or refuted from data. |
| B3 | outstanding | Trigger a burst on the polled departure path | A departure caught by the polled path produces no burst and no jog verdict, so the backstop is invisible exactly when it matters. |
| B4 | **done** | Capture a cruise-phase peak, distinct from the sticky arrival peak | The cruise ceiling is the denominator of the arrival gate and cannot currently be measured: the sample log is decimated and sticky, and bursts are too short to isolate cruise. |

**Does B actually need A? Answered 2026-08-19: not for B2 and B4.** The plan
asserted the gate rather than measuring it, and that assertion set the project's
longest dependency chain — A gates B, B gates D and H. Measured against the
existing 68 bytes free:

| variant | flash | free | vs baseline |
|---|---|---|---|
| baseline `26e38df` | 32188 | 68 | — |
| B2 only | 32212 | 44 | +24 |
| B4 only | 32228 | 28 | +40 |
| B2 + B4 | 32254 | 2 | +66 |
| `tk=` removed | 32128 | 128 | −60 |
| **B2 + B4, `tk=` removed** | **32194** | **62** | **+6** |

Both fit without A even before the `tk=` credit. With it — justified because
§6a is now resolved on measurement and the counter was marked
remove-once-resolved — the pair costs a net 6 bytes and leaves 62 free, within
6 bytes of the previous headroom. RAM went 1366 → 1364. Shipped in `0cacf29`.

**B4 needed no new state at all.** This plan budgeted for "a running peak over a
bucket that retires far enough behind the current sample"; `arr_peak_cur` and
`arr_peak_prev` already *are* that, maintained for the arrival gate on a 1 s
alternating bucket. `pk=` prints `max(cur, prev)`, which is exactly why it
cannot report a cruise ceiling — at a stop the open bucket already holds the
arrival transient. `arr_peak_prev` alone lags one full bucket, so when the stop
lands in the open bucket the retired one still carries the pre-stop value. B4
was printing, not machinery.

**B1 and B3 remain the expensive pair and are unmeasured.** They are the reason
session A may still be forced, and B1 is the more important of the two — it is
the signature of the failure this plan calls the most dangerous. Measure them
the same way before accepting any gate that forces the bma4 driver swap, whose
own caution note requires re-deriving every any-motion constant on the bench.
(B0 was struck on 2026-08-19; see section 1.2.)

**Method.** Print-only where possible. B4 turned out to need no state — see
above. B1 and B3 have not been attempted; B3 in particular touches the polled
departure path rather than the log, so it is the one most likely to cost real
flash.

**Exit criterion.** A capture from an ordinary run in which departure
provenance, beacon state, and a cruise ceiling can all be read directly. **Two
of the three are now available** — `bz=` and `cp=` print on every periodic
sample line. Departure provenance still needs B1.

**Verification.** No behavioural change. Confirm identical FSM transitions on a
replayed bench run before and after.

**Verification result for B2/B4, 2026-08-19.** Bench, 90 s steady plus a full
boot-from-reset capture:

| check | result |
|---|---|
| `bz=`/`cp=` present | 281/281 sample lines |
| `tk=` removed from the log | 0 lines |
| `bz=` with the beacon silent | 0 on every line |
| invariant `pk >= cp` | 401/401 lines, 0 violations |
| `cp=` live, not stuck | range 0.02 – 0.14 |
| `ov=` advance in steady state | **0** — flat at 38 over 363 lines; the pre-change control settled at 38 too |
| sample rate | 25.00 Hz |
| FSM transitions | identical to the pre-change control over the range both captures cover; MONITORING reached; re-arm blank settled 1605 ms against 1589 ms on the control |
| reboots in window | none |

⚠️ **`cp=` is only meaningful during cruise.** It separates from `pk=` only
when there is a stop transient for it to lag behind; at rest the two track, as
they did throughout this bench capture. The cruise ceiling itself is a car
measurement and belongs to session D.

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

**⚠️ Added 2026-08-19 — the rest gate is currently INERT, and this session is
what sizes it.** `MONITOR_REARM_QUIET` is 8, and measured across 20 stops that
day `quiet_run()` was ALREADY at or above 8 at the moment `STATE_MONITORING` was
entered on 13 of them. So the quiet condition contributes nothing on an ordinary
stop and the blank is purely `MONITOR_REARM_MIN_MS` — which is why the settle
figure was a flat 1507–1605 ms before the floor moved, and a flat ~2556 ms
after. The gate is not gating.

Time from MONITORING until `quiet_run()` first reaches each threshold, ordinary
stops, 2026-08-19:

| threshold | reached at |
|---|---|
| 8 | 0 ms on 13 of 20 |
| 16 | 0 ms on 10 of 20 |
| 32 | 0–1934 ms |
| 64 | 0–7045 ms |
| 128 | 1589–18890 ms, 3 never |

**The number this session must produce is the same table for TERMINAL stops.**
If a ringing terminal brake set holds `quiet_run()` down materially longer than
these, the threshold that separates them is the fix — it extends the blank only
when the stop is actually ringing, which no floor value can do. If terminal
stops look the same as these, the rest-gate approach cannot discriminate and
that is worth knowing before more effort goes into it.

**⚠️ Added 2026-08-19 — the two gates are on DIFFERENT AXES, and that is a third
reading of the same table.** `STOP_CONFIRM_MS` judges the vertical channel:
`delta_accel`, the rolling average's displacement from `zero_calib_value`,
against `STOP_BAND_VALUE`. The rest gate judges the lateral channel:
`m = |Δax| + |Δay|` (`lateral.cpp:63`). They differ in axis and in KIND — the
vertical test is a LEVEL relative to a calibrated rest point, the lateral one is
a SAMPLE-TO-SAMPLE CHANGE. A steady lateral offset therefore reads as quiet, and
a purely vertical oscillation is invisible to `m` entirely.

So if a terminal brake set rings mostly in z, `quiet_run()` keeps climbing
straight through the ringing and the rest gate cannot see it AT ANY THRESHOLD.
That produces a table indistinguishable from the "terminal stops look the same"
outcome above, with a different cause and a different fix: the gate would need a
vertical term, not a larger `MONITOR_REARM_QUIET`. Concluding "rest-gating cannot
discriminate" without separating these two is the error this note exists to
prevent.

**The capture already distinguishes them and no firmware change is needed.** The
sample line carries `avg=` — the rolling average of the vertical channel, which
is what `delta_accel` is computed from — alongside `m=` and `q=`, and
`Zero-Calib` is printed at calibration, so `|avg - zero_calib|` reconstructs
offline. Report it per terminal stop on the same time axis as `q`:

| what the capture shows | reading |
|---|---|
| `q` held down through the ringing | rest-gating works; size the threshold from the table |
| `q` climbing while `\|avg - zero_calib\|` shows excursion | the ringing is vertical and the lateral gate is blind to it |
| neither shows excursion | the stop is genuinely not ringing, and a re-latch there has some other cause |

`graph/session_c.py` does not print the vertical column yet, and the 2026-08-19
capture is gitignored, so the change is unvalidated as written and is left for
whoever runs the session.

Run `graph/session_c.py` on the capture; it prints both the regression check and
this table.

**Method.** Terminal-floor stops, both terminals, logger running. At least ten.
Include the single-floor terminal approach, which is the geometry that produced
the original event. Do not jog between runs; allow the FSM to settle naturally,
since jogging masks the condition under test.

**Exit criterion.** Ten terminal stops with no alarm beginning on a car that has
already stopped. Any instance is a regression: **record the timing** — the
interval from `STATE_MONITORING` to the re-latch is the number that decides what
to change, and without it any adjustment is a guess.

⚠️ **Corrected 2026-08-19: this used to say "revert `MONITOR_REARM_QUIET` to 0".
That is backwards.** The clear condition is `elapsed >= MONITOR_REARM_MIN_MS &&
quiet_run() >= MONITOR_REARM_QUIET`, so 0 makes the second test always true and
the blank clears at the 1500 ms floor every time — the shortest blank the code
can produce. To revert to the fixed 6 s, set `MONITOR_REARM_QUIET` to **255**,
which is unreachable inside the 6 s cap.

**And the revert has a measured cost.** Across 17 stops on 2026-08-19 the
shortest gap between `MONITORING` and the next *genuine* departure was 2801 ms.
A blank of 4000 ms or 6000 ms would have discarded one real departure; 1500 and
2500 discard none. The 2026-08-07 false re-latch was at 3.5 s. **Real departures
at 2.8 s and false re-latches at 3.5 s overlap, so no blank duration separates
them** — which is why this gate is rest-based, and why `MONITOR_REARM_QUIET` is
the lever rather than `MONITOR_REARM_MIN_MS`. Sizing it needs `q` from a
terminal-floor brake set, which is this session.

**Field report 2026-08-19:** intermittent alarms after arrival observed in
service. Not reproduced in the 2026-08-19 capture — the one sub-6 s re-latch in
that log (2801 ms) reads as a genuine jog (`a=` 9.70→8.08, `x=` 0.69→1.18,
`m=0.507`, full FSM cycle to a polled arrival), not a false latch on a
stationary car. This session is now the priority.

**Note.** This is the only test in the plan checking a failure the project may
have introduced, rather than one it inherited.

---

## 5. Session D — arrival margin population

**Car. UNBLOCKED 2026-08-19 — no firmware prerequisite remains.** Its
instrumentation dependency was B4, the cruise ceiling, which is delivered and
bench-verified in `0cacf29`; `cp=` now prints on every periodic sample line.
Read `cp=` during cruise, not at the stop — that is the whole point of it
lagging a bucket behind `pk=`.

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

**Method — REVISED 2026-08-19, the original was not executable.** It asked for
"the softest stops the machine can produce ... at inspection speed". Per the
site those are mutually exclusive: **soft stops are only achievable on automatic
operation at contract speed** — 150 fpm hydraulic, 300–500 fpm traction. On
inspection the brake sets from motion and the arrival transient is set by brake
mechanics, not by the approach. Six inspection runs on 2026-08-19 confirmed it:
the deliberate soft stop produced a *harder* arrival (2.410) than two ordinary
ones (1.360, 1.430), and the whole population sat at 3x the 0.460 weakest
arrival on file. State doc §2.5a.

The session therefore splits by regime:

- **Cruise ceiling — DONE at inspection speed.** 0.060–0.080 across five logged
  runs, both directions, 19 fpm. Read `cp=` with a window opening at +8 s, not
  `MIN_TRAVEL_MS`; see §2.5b for why, and use `graph/session_d.py`.
- **Weakest arrival — needs AUTOMATIC OPERATION AT CONTRACT SPEED**, and
  belongs with session E rather than here. Twenty runs minimum still applies:
  the weakest arrival is the number the whole approach rests on and it has
  moved by a factor of three across five previous runs.

**Exit criterion.** A weakest-arrival figure and a cruise ceiling from the same
regime, sufficient to re-derive the gate. State whether the populations still
separate.

**RESULT 2026-08-19 — criterion met at contract speed, and the gate is wrong by
about 2x.** Cruise 0.080–0.120 and arrivals 0.463–0.700 from the same five
automatic runs at 500 fpm. Separation 4.7–6.8x, so the populations separate
comfortably and the 1.6x cliff is not in play: a single gate still serves, its
value is simply wrong. The gate sits at 0.97x the weakest arrival. Re-derived by
the original method it lands near 0.236. Today's weakest, 0.463, reproduces the
0.460 on file to within 0.003 — the first independent reproduction of the number
the whole derivation rests on. **Five samples, 51% spread, same trip producing
0.470 and 0.700: the twenty-run minimum stands and 0.236 is arithmetic, not a
validated constant.** Full report: `falcon_arrival_gate_2026-08-19.md`.

⚠️ **"From the same regime" is now the hard part, and it is not a formality.**
The cruise ceiling is an inspection-speed measurement and the weak arrival is an
automatic-operation one, so they are not from the same regime and cannot simply
be divided. Either re-measure cruise at contract speed during the automatic runs
— `cp=` prints on every sample line, so this costs nothing extra — or state
plainly that the gate is being derived across two regimes and why that is
acceptable. Do not quietly pair the 0.060–0.080 inspection ceiling with an
automatic-operation arrival and report the ratio as a margin.

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

**EXIT CRITERION MET 2026-08-19**, as a by-product of session D's
contract-speed runs. **`RAMP latched` on 9 of 9 automatic stops at 500 fpm,
both directions, `dir=100` throughout — and 0 of 8 inspection stops.** That is
this session's criterion in full: fires on drive-controlled stops, declines
inspection stops, same session.

⚠️ Count `RAMP latched`, **not** `FSM: Arrival (ramp)`. The latter is zero
because the polled path triggers first and the FSM has already moved to
DECELERATING before the ramp verdict lands. An earlier pass the same day counted
the wrong string and concluded the detector never runs. Do **not** lower
`ARM_REV_SAMPLES` to "fix" it — the runs showing `g=0` at ARM time latched
anyway, and that constant also gates the peak collector, so loosening it is the
false-release direction. `falcon_arrival_gate_2026-08-19.md` §4.

**Exit criterion.** The detector fires on drive-controlled stops and declines
inspection stops in the same session. If it fires on no automatic stop, it is
not carrying the load it is armed for and should be disarmed until it does.

---

## 7. Session F — second installation, logged

**Hydraulic elevator, second building. Depends on nothing.**

**Open items:** 11, 10, 8.

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
  ⚠️ **Let the device settle before each one.** On 2026-08-19 this exact
  procedure was run on the cartop and the calibration taken immediately after
  handling produced a spurious "learned" threshold of 0.069354 — 1.73x the
  floor, derived from the installer's residual motion rather than the mounting,
  with `mv=` reading 0 and nothing else flagging it. Run this session
  unsettled and it will manufacture the very result criterion 2 is looking
  for. State doc §2.2a, open item 13.

**Exit criteria.**

1. Whether calibration-derived thresholds transfer, or whether the constants are
   specific to the original installation.
2. Whether `Threshold-Value` exceeds the clamp floor on any mounting **from a
   settled window**. A value above the floor is not on its own evidence that
   learning works — pair it with `XY-Still` and the `XY: bmax` spread to show
   the window was quiet. See open item 13.
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

**Method.** ⚠️ **Superseded 2026-08-20 by
`Eng_Notes/falcon_corpus_labelling_2026-08-20.md`**, which is the runnable
procedure. `graph/session_g.py` (new, validated against the committed corpus)
prints the census and emits a blank worksheet. Run it before committing the day.

**Census measured 2026-08-20, and it changes what this session can conclude.**
273 departures across 56 captures; 211 carry a departure burst; 62 carry none
(the instrument landed 2026-08-11, so all of 08-06/07/10 is empty). *The plan's
387 figure is not reproducible from the corpus — reconcile or supersede it.*

Three findings the plan did not anticipate:

- **Departure bursts fire on the ANY-MOTION path only** — the
  `burst_trigger(BURST_POST_DEP, 0)` call sits inside `if (any_motion_pending)`.
  Departures the polled or velocity path caught produce no burst, so the corpus
  excludes by construction the runs the backstop caught.
- **`span_ms` is not a run duration.** `ACC-INT` is edge-triggered and cruise is
  quiet by construction, so in most runs both edges sit at the departure. 2-edge
  median span is 1146 ms, which is not a 1.1 s ride. Do not label from it.
- **The binding constraint is notes coverage, not burst coverage.** Nothing
  inside a capture says run vs jog — `end_path` is circular for the same reason
  `JOGV verdict=` is — so the dated session notes are the only independent
  source, and they are per-session narrative rather than per-record.

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
| 2 arrival gate margin | D | — (was A, B; cleared by B4) |
| 3 knock/departure separability | I | — |
| 4 jog verdict | J | G |
| 5 velocity path disabled | H | B1/B3, and so A |
| 6 ramp detector unevidenced | E | — |
| 7 logging in shipping build | A | — |
| 8 quiet gate vs cruise | D | — (was A, B; cleared by B4) |
| 9 re-arm regression | C | — |
| 10 z threshold unproven | F | — |
| 11 installation coverage | F | — |
| 12 any-motion latched reference | B1 | A |

**Changed 2026-08-19.** B2 and B4 were built and measured against the existing
68 bytes rather than assumed to need session A. Both fit; with the `tk=` credit
the pair costs a net 6 bytes. That clears items 2 and 8 entirely and makes
session D runnable on car time alone. B1 and B3 are unmeasured and still carry
items 5 and 12 behind session A. Section 3.

---

## 13. What is deliberately not scheduled

| Not scheduled | Reason |
|---|---|
| Retuning `ANYMOTION_THRESHOLD` | Refuted. A hand bump exceeds a real 18 fpm departure; no value satisfies both directions. |
| Waveform-shape replacement for `opk` | Refuted across 177 bursts. A jog's departure is a real departure. |
| Waveform shape as a knock pre-filter | Refuted. A confirmed 20 fpm departure scored inside the knock band. |
| Raising `JOG_OPP_PEAK_MMSS` | Refuted. A real run was silenced at 972 against a 900 gate. |
| Further 150 fpm runs **for departure characterisation** | Confirms nothing. All transients are far above threshold at that speed. ⚠️ Narrowed 2026-08-19: this holds for DEPARTURES only. It was being applied as a blanket rule and was keeping the project out of the only regime where the arrival gate's weak tail exists — soft stops need automatic operation at contract speed. State doc §2.5a. |
| Lowering `LOG_DECIMATE_N` to buy cruise resolution | Refuted on the bench 2026-08-19. At N=2 the device loses ~35% of samples, reads 20.8 Hz instead of 25, and boot-loops under the watchdog — at 11.8% serial duty. The ring is drained one sample per loop pass, so log density throttles the consumer and depth cannot compensate. Section 1.2. |
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
