# Cab session, 2026-08-12 — automatic operation, the ramp detector, and a blind arming gate

Second half of 2026-08-12, Dave inside the cab, automatic operation at 300 fpm.
Read `falcon_cartop_2026-08-12.md` first — same day, inspection operation, and
it carries the TWI lockup post-mortem. Log:
`device-monitor-260812-112254.log` (two boot sections; the reflash at
`RAMP_FLOOR_MMSS` 400 → 300 splits them).

Firmware through the session: `981d863` (ramp detector, unarmed) → ramp
logging → floor 300.

⚠️ **The device did NOT reboot when moved from cartop to cab**, so the first
two runs here ran on the cartop's zero calibration. The reflash that followed
recalibrated in the cab (`Zero-Calib 9.7573`, `XY-Still 0.0525`); runs from
there on belong to this mounting properly.

---

## 1. 🟢 The ramp detector works — 4/4 on automatic stops

This is the positive evidence the detector was missing, and inspection
operation could not supply it.

| run | ramp mean | dir | arrival peak | peak margin |
|---|---|---|---|---|
| 300 down | 490 | 100% | 0.477 | 1.06x |
| 300 up | 495 | 100% | 0.457 | **1.016x** |
| 300 down | 496 | 100% | 0.544 | 1.21x |
| 1 floor (terminal→intermediate, up) | 501 | 100% | 0.465 | 1.03x |

**Every drive-controlled stop produced a clean sustained one-signed ramp at
100% directionality.** Blocks qualified 5-for-5 within each stop. The first two
were reconstructed by replaying the arrival bursts offline; the later ones the
device declared itself (see §2).

### 1.1 The peak detector is not marginal, it is on the threshold

Pooling with 2026-08-11, **eight automatic stops have produced arrival peaks of
0.454–0.544 against a 0.45 gate — every one inside 1.21x, four inside 1.06x.**

That is not a distribution with a thin tail. Automatic stops *sit on* the
threshold, because the FSM peak is a deviation from a rolling average and a
sustained ramp drags the average with it. The 08-11 note called this "releases
on luck" from four samples; eight say the same thing.

The same eight stops give a ramp mean of 487–501 at 100% directionality against
a 400 floor and an 85% gate. **The ramp is not an incremental improvement over
the peak; it is the difference between a measurement and a coin flip.**

## 2. The detector was invisible where it mattered — fixed

The FSM's ramp check lives inside `STATE_MOVING`, and on a fast drive stop it
never gets there. The deceleration plateau (~0.5 m/s²) is ITSELF above
`ARRIVAL_PEAK_VALUE` (0.45), so the polled peak crosses on the ramp's leading
edge ~0.3 s in, the FSM leaves `STATE_MOVING`, and the ramp verdict — which
needs ~1.44 s — latches about a second later with nobody looking.

Both of the first two cab runs qualified five blocks and **printed nothing**.

Fixed by `emit_ramp_log()` in `loop()`: prints the latch as it happens,
regardless of FSM state. Pure instrumentation, no release decision reads it.
First live line, third run:

```
FSM: Arrival (polled), peak 0.544
FSM: Transitioned to STATE_DECELERATING
RAMP latched mean=496 dir=100 st=5
```

`st=5` (DECELERATING) confirms where it had been latching unseen.

## 3. `RAMP_FLOOR_MMSS` 400 → 300, and why the "constant plateau" claim needed qualifying

400 was set against 08-11's plateau of 0.605 ± 0.008 (n=9), claiming 1.5x
headroom. **Today measured 487–501 (n=4) — 19% lower.**

| session | speed | plateau | n |
|---|---|---|---|
| 2026-08-11 | 350 fpm, cab | 0.605 ± 0.008 | 9 |
| 2026-08-12 | 300 fpm, cab | 0.487–0.501 | 4 |

Both are internally tight and they disagree with each other. So **"the drive
holds a constant deceleration" is true PER CONFIGURATION, not universally**, and
400 left only 1.23x on the newer figure. An installation 20% gentler than today
would fail the floor outright and lose the detector *silently* — the failure
direction this project keeps being bitten by.

300 restores 1.63x. It costs nothing in false positives because **directionality
is what discriminates**: 100% on every drive ramp measured, against 0.42/0.02
for brake stops, and no cruise block mean comes near either gate.

**The single-floor run is why a fixed floor is defensible at all:** a stop that
never reached top speed still held 0.501 for 2.6 s, so the drive sheds whatever
speed it has at its own rate.

Replay before flashing: 24 cartop bursts — brake stops, jogs, slow and 350 fpm
departures — fire at neither 400 nor 300.

## 4. 🔴 THE ARMING GATE IS LOAD-BEARING, and it is also the detector's blind spot

### 4.1 The gate is the only thing preventing a catastrophic false positive

Replaying the block test against the cab captures: **all four DEPARTURE bursts
qualify as readily as the arrivals do.** A 300 fpm departure sustains ~0.5 m/s²
one-signed for the whole 3.2 s window.

The block arithmetic **cannot** tell a departure ramp from an arrival ramp: same
shape, opposite sign, and sign cannot be used because it encodes direction of
travel, not phase of the run. `arr_armed` — five consecutive quiet samples — is
the ONLY thing standing between this detector and releasing the beacon seconds
after the latch, on a moving car (§14.4's catastrophic direction).

I had assumed that gate was belt-and-braces when writing the detector. It is
not. **Do not weaken it, do not simplify it away, and re-run the replay after
any change to arming.** Recorded in `main.cpp` at the gate itself.

### 4.2 …and the same gate blinds both detectors on short runs

**Measured live: a single-floor run, intermediate → bottom terminal with an
extended slowdown profile, alarmed for 85 s over a car that was stationary for
78 of them.**

The stop was textbook — one-signed, +0.5 to +0.64 m/s², sustained 3.2 s. The
device reported a peak of **0.00** through all of it:

```
t=63389  a=9.930   pk=0.00
t=64176  a=10.285  pk=0.00     <- deviation +0.53, reported as ZERO
t=66306  a=10.394  pk=0.00     <- deviation +0.64, reported as ZERO
t=67420  a=9.727   pk=0.05     <- arms here, AFTER the car has stopped
```

`arr_armed` never armed. The run went **departure ramp → deceleration ramp with
no cruise between them**, so the five consecutive quiet samples never occurred,
and *both* the peak collector and the ramp accumulator were switched off for the
entire event.

**This is the jog defect's mechanism, at speed.** The 08-11 note killed
arm-on-quiet for jogs with exactly this reasoning: *"It needs a quiet gap
between departure and stop to arm, and a 1 s jog is one continuous disturbance
with no such gap."* The same structure demonstrably breaks **single-floor and
terminal-approach automatic runs** — among the most common trips in service.

**It also means the ramp detector as built cannot do its job.** It was gated on
`arr_armed` to keep it off the departure ramp (§4.1, necessary), and that same
gate blinds it on exactly the short runs where the peak detector also fails.
**A detector cannot cover another's blind spot while sharing its blinding
condition.** That is a design flaw in the detector as committed, not a tuning
issue.

### 4.3 How it ended, which is worse than a stuck beacon

It did **not** release on the failsafe. At t=144.6 s an unrelated disturbance
(doors, or Dave moving in the cab) crossed the gate:

```
FSM: Arrival (polled), peak 0.458
```

| t | event |
|---|---|
| 59.6 s | departure, beacon on |
| 63.4–66.6 s | the real deceleration — invisible, pk=0.00 |
| ~67 s | car stationary |
| 144.6 s | random 0.458 disturbance, beacon finally off |

**78 s of beacon over a stationary car — the position lie, released by accident
rather than by detection.** In the log it is indistinguishable from a healthy
release.

### 4.4 Why the comparison run does not explain it by gap length

| run | gap between ramps | armed? |
|---|---|---|
| terminal→intermediate, up | 672 ms | ✅ pk reached 0.57 |
| intermediate→bottom terminal, down | 803 ms | ❌ pk stayed 0.00 |

**The failing run had the LONGER gap.** So "extended slowdown eats the cruise"
is not sufficient on its own. Arming needs five *consecutive unblanked* samples
under 0.15, and during an alarm roughly half of samples are blanked by the
piezo — so whether five consecutive land inside a sub-second window is marginal
and phase-dependent.

On n=2 the honest reading is **intermittent, not a clean boundary**: short runs
are a coin flip rather than a guaranteed failure. That needs more runs, and it
is exactly the kind of claim this project has been wrong about before.

### 4.5 ⬜ Fix direction, deliberately not built from one run

Arming should key on **"the departure ramp has ended"** rather than **"the
signal went quiet"** — a continuous run never provides the latter. A sign
reversal is the obvious candidate, since a departure and its deceleration are
opposite-signed by physics, and §4.1 shows sign is otherwise unusable.

Not built. It wants the bench and a replay against every burst now on file.

## 5. Parser: the failure was invisible, so it is now checked for

`FSM: Arrival (polled), peak 0.458` reads exactly like a healthy release. Every
historical run summary could contain this and look clean. Added
(`parse_falcon_log.py`):

- **ARMING FAILURE** — `pk=0.00` across a real excursion later than
  `ARM_GRACE_MS` after the latch. The window is what makes it mean anything:
  pk is *deliberately* 0.00 through the departure ramp, and a naive
  "pk==0 during an excursion" test fires on every healthy run (tried; it did).
- **LATE RELEASE** — gap between the last deceleration ramp and the release.
- **MULTI-BOOT WARNING** — a capture spanning reflashes mixes millis() epochs.
  This trap produced 13 phantom "false departures" on 08-11 and a bogus 11.5 s
  late-release verdict *during this very change*.

Two rejected formulations, kept in the source so nobody re-derives them:
*"quiet immediately before the release"* is defeated by the very disturbance
that causes the release; *"longest in-band stretch"* cannot work at all, because
at constant velocity z reads 1 g exactly as parked (§3) — the trap that got the
stillness backstop banned.

Regression: 15 healthy runs, zero false positives; the defective run flagged by
both checks at 78.3 s, matching hand analysis.

### 5.1 Sweep over every historical capture

25 runs across 10 boot sections (all captures carrying `pk=`, i.e. 08-10
onward). **The arming defect appears exactly once — today's terminal-approach
run.**

**That is not evidence it is rare.** Today was the first time the conditions
were exercised: a short automatic run into a terminal with extended slowdown.
The 08-11 350 fpm runs were multi-floor express trips with real cruise and armed
normally. Read it as *first exercised today*, not *one in twenty-five*.

Two limits on that sweep:

- **The late-release check is blind on brake stops** — it keys on a
  deceleration ramp, and an inspection stop is a brake ring with none. For every
  cartop run it cannot distinguish late from on-time.
- Pre-25 Hz captures are excluded: no `pk=` field, and the defect is
  structurally impossible before the arm-on-quiet gate existed.

One historical run deserves folding into the same investigation:
`260811-144408_s1` run 2 held **240.6 s to FAILSAFE with maxpk 0.55** — above
the gate, so the peak likely arrived only at or after the failsafe. A second
instance of "the arrival was not detected when it happened", by a different
mechanism.

## 6. Where this leaves the product

**Inspection operation works** (cartop half of the day: 5/5 runs, 4/4 jogs).

**Automatic operation does not, in two distinct ways:**

1. Multi-floor stops release on a peak margin of 1.0–1.2x — passing, but on
   luck. The ramp detector answers this and is measured 4/4.
2. Short and terminal-approach stops can leave both detectors switched off
   entirely, producing a minute-plus position lie that looks like a normal
   release in the log. **Nothing in the firmware currently addresses this**, and
   the ramp detector as built inherits the flaw.

Item 2 outranks item 1: a marginal release is a reliability problem, a
78-second false beacon on a stationary car is the failure the product exists to
prevent.

---

# Addendum — arming redesign and the floor-by-floor descent (same day, later)

Firmware `6ee1b4a`. Four automatic single-floor runs, cab, descending 4→3→2→1
with the last into the bottom terminal on the extended slowdown profile.

## 7. ⚠️ A claim withdrawn: the arming margin was never measured

Between §4 and this addendum I asserted, from a `q_hi` diagnostic I had just
written, that arming succeeds with **zero margin** — "q_hi = 5 against need =
5, four runs, the gate delivers exactly the minimum and never more" — and drew
a coin-flip conclusion from it.

**That was an artefact of the instrument.** Once `arr_armed` goes true the
counting branch is skipped, so `arr_quiet` stops at `ARRIVAL_ARM_SAMPLES` and
`q_hi` is capped at 5 by construction. `q=5` means only *"it armed"*; `q<5`
means *"it never armed"*. The instrument cannot distinguish a run that armed
comfortably from one that scraped through, and every "5/5" reported here and
in `6ee1b4a`'s message is that cap, not a measurement.

Withdrawn. What survives unaffected:

- the failing run **never armed at all** (`pk` 0.00 across a real +0.64
  excursion) — that is what `q<5` reports faithfully;
- every run since has armed.

Whether the successful ones arm with margin is **unknown**. Measuring it needs
the longest quiet run bounded to the departure→deceleration window, which this
counter does not provide. ⬜ Instrument fix outstanding.

The reversal path does not depend on the withdrawn claim: it exists because a
run went completely unarmed.

## 8. The descent: 4 runs, all clean, the fix unexercised

| run | ARM | arrival peak | ramp |
|---|---|---|---|
| 4→3 | q=5 a=1 **v=1** | 0.477 | 472 / 100% |
| 3→2 | q=5 a=1 **v=1** | 0.464 | 481 / 100% |
| 2→1 terminal | q=5 a=1 **v=1** | 0.498 | 479 / 100% |

**`v=2` never appeared.** The sign-reversal path has never once been needed in
live running. It is committed, proven *safe* against 51 departure bursts, and
entirely unexercised — an honest status for a fix to a failure that has
occurred once in roughly thirty runs and will not reproduce on demand.

**Four attempts at the failing configuration** (single floor into the bottom
terminal, extended slowdown) across the day, all clean. Absence over four
attempts says little about an intermittent failure that took ~25 runs to
appear once; it is not evidence the defect is gone.

## 9. What the day did settle

**The ramp detector is the strongest result on this device.** Nine automatic
stops, both directions, distances from one floor to express:

```
ramp mean   472 481 479 489 490 495 496 501 508     (spread 36, i.e. 7%)
directionality  100% on every single one
```

Against inspection operation's negative evidence — brake stops, jogs, cruise,
repositioning, departures, all declined — this is a detector with a clean
signature on both sides.

**The peak path continues to sit on its gate.** Eleven automatic stops now
measured at 0.454–0.544 against 0.45. Not a tail; a distribution centred on the
threshold.

## 10. Next, in order

1. **Fix the arming-margin instrument** (§7) — the margin question is still
   open and currently unanswerable.
2. **bma4 driver swap.** Flash is at 32184/32256, **72 bytes free**. It blocks
   `-DWIRE_TIMEOUT` (the real TWI lockup fix, +1818 bytes) and everything else.
3. **Promote the reversal path** only after a session shows `v=2` on a short
   run with `RAMP latched` on a stop the peak missed.
4. **Arm the ramp detector** — the evidence is there; the reason to wait is
   that it shares a gate whose behaviour on short runs is still the open
   question.
5. Brownout + lockup electrical work, and the battery settling-logic bug
   (still "(settling, ignored)" on every read).

---

# Addendum 2 — the arming margin, measured

Firmware `a77f146` + the q= instrument fix. Six automatic single-floor runs in
one continuous sequence: 4→3→2→1, then 1→2→3→4. Dave waited for each release
before the next departure, so `MONITOR_REARM_MS` never suppressed a departure.

## 11. The instrument, fixed

The first `q=` counter stopped at `ARRIVAL_ARM_SAMPLES` because counting sat
inside `if (!arr_armed)`. Counting now continues past arming and freezes when
the stretch that armed it BREAKS, so `q=` is the true length of that stretch:

    q=5   armed on the last possible sample -- no margin
    q=12  quiet lasted 12 where 5 were needed -- 2.4x
    q<5   never armed (the defect), unchanged

## 12. 🟢 Arming is NOT marginal — the second correction

| run | q= | margin | arrival peak | ramp |
|---|---|---|---|---|
| 4→3 | **36** | 7.2x | 0.476 | 486 |
| 3→2 | **18** | 3.6x | 0.544 | 492 |
| 2→1 | **9**  | 1.8x | 0.488 | 480 |
| 1→2 | **8**  | 1.6x | 0.504 | 513 |
| 2→3 | **16** | 3.2x | 0.460 | 498 |
| 3→4 | **38** | 7.6x | 0.456 | 513 |

Every run cleared the requirement by 1.6x–7.6x. **The "zero margin, coin flip"
reading recorded earlier in this note was wrong twice over** — the instrument
was capped AND the real margins are comfortable. On these six runs the gate has
room, and the 08-12 morning failure is an OUTLIER rather than the normal
condition. Two successive overstatements of this defect's generality; the
measured version is above.

## 13. 🔵 Margin tracks POSITION IN THE SHAFT, not direction

```
top     4→3  36        3→4  38
        3→2  18        2→3  16
bottom  2→1   9        1→2   8
```

Both directions agree almost exactly at each level, and margin falls ~4x from
top to bottom. **The thin end of the shaft is the bottom — and that is exactly
where the morning's failure occurred** (2→1, bottom terminal, extended
slowdown). A mechanism, not bad luck: the extended slowdown compresses the gap
between departure and deceleration, leaving fewer quiet samples to arm on.

At 8–9 samples the bottom still has margin, but a run losing half its quiet
samples to buzzer phase lands at ~4 — matching the 4.0–4.9 estimated for the
failure from its decimated trace. The independent estimates agree.

⬜ n=1 per floor pair. The symmetry across directions is what makes it
convincing rather than the sample count.

## 14. Where this leaves the two mechanisms

**The reversal path is insurance, not a fix.** It backstops the thin end of the
shaft; it is not answering a pervasive defect. Still unexercised (`v=2` has
never appeared in ~10 live runs), still gating the ramp detector only, still
unpromoted.

**The ramp detector is the finding with weight.** Fifteen automatic stops now:

    ramp mean  472 479 480 481 486 489 490 492 495 496 498 501 508 513 513
    directionality  100% on every one

against complete negative evidence from inspection operation. Meanwhile
**seventeen automatic stops put the arrival peak at 0.456–0.544 against a 0.45
gate** — the release the product ships with is decided by a distribution
centred on its own threshold.

**Arming the ramp detector is the change that would actually improve the
product.** The reason it has not been armed is that it shares a gate whose
behaviour at the thin end of the shaft is the remaining open question.
