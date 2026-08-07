# Falcon firmware — analysis and roadmap

**Date:** 2026-08-06 · **Revised:** 2026-08-06 (bench session, PR #1 flashed)
**Baseline:** `Falcon_Rel_EFT` @ `41b8996` (PR #1 merged; was `59e945f`)
**Evidence:** the eight PuTTY captures on `eft-results-2026-07-15`, source reading,
and a bench session on 2026-08-06 with PR #1 running on hardware
**Status of each item is marked** ✅ fixed · 🔧 in progress · ⬜ not started

This document exists so the reasoning survives independently of any one work
session. Every claim below is either derived from the source or measured from
the July captures; anything unverified is marked **ASSUMPTION** with the check
needed to confirm it.

**2026-08-06 bench session** flashed PR #1 and confirmed two predictions (the
1 MHz fuse, the 3.13 Hz rate) while turning up four defects the log-only analysis
could not see — three of them field-reliability problems that outrank the
detection tuning this document originally focused on. See §10.

---

## 1. Hardware and build facts

| Fact | Source |
|---|---|
| MCU is **ATmega328PB** | `platformio.ini`, `upload_flags = -pm328pb` |
| Framework is **MiniCore**, not stock Arduino AVR | build output paths |
| **F_CPU = 1 MHz** | `board_build.f_cpu = 1000000L`, present since first commit `152e35b` |
| Clock source is **internal 8 MHz RC ÷ 8** | `commands.sh`: `lfuse:w:0x62` — `CKSEL=0010` (8 MHz RC), `CKDIV8` programmed |
| BMA456 is on **TWI1 / `Wire1`**, not TWI0 | `arduino_bma456.cpp:75`, commit `e3be545` |
| I²C clock ≈ **62.5 kHz** | `TWBR1 = 0`, `TWSR1 = 0` → `F_CPU/16` |
| Serial is **9600**, not the documented 115200 | 2026-07-15 EFT session |
| DPS310 barometer is **depopulated** on this hardware revision | confirmed by Dave, 2026-08-06 |

**Evidence that the silicon really runs at 1 MHz** (as opposed to F_CPU merely
being set to it):

1. Serial works at 9600 with a 1 MHz build. Baud is `F_CPU / (16 × (UBRR+1))`;
   if the real clock were 16 MHz the actual rate would be 153600 and the
   terminal would show garbage. July session confirmed 9600 clean, 115200 dead.
2. The alarm was observed cutting out at "roughly 10 seconds", matching
   `STOP_TIMEOUT_MS = 10000` on a correctly-calibrated `millis()`. A 16× clock
   error would have fired it at ~0.6 s.

**Not** evidence: the measured 3.0 Hz sample rate. That was derived against
`millis()`, which scales with the same oscillator, so the ratio cancels and it
only confirms the compile-time constant.

✅ **Confirmed on the bench, 2026-08-06.** `avrdude -c stk500 -P COM6 -p m328pb
-U lfuse:r:-:h` →

```
avrdude: device signature = 0x1e9516 (probably m328pb)
avrdude: reading lfuse memory ...
0x62
```

`lfuse = 0x62` is `CKSEL=0010` (8 MHz internal RC) with `CKDIV8` programmed, so
the part runs at 1 MHz. The signature also confirms a genuine ATmega328PB. All
arithmetic in this document stands on a measured clock, not an assumed one.

`compile_commands.json` is committed and says `F_CPU=16000000L`. It is a stale
artifact from an older build and should be ignored — it caused a wrong turn
during this analysis. Worth untracking along with `.pio/`.

---

## 2. 🔴 The dominant defect: the FSM runs at ~3.1 Hz, not 100 Hz

All eight captures give **exactly 3.0 samples per second**, measured against the
known 1000 ms `STATE_MOVEMENT_DETECTED → STATE_MOVING` transition. The BMA456 is
configured `ODR_100_HZ` and the timer comment says "for ~10 ms interrupt".

`main.cpp`, `enable_timer()`:

```c
TCCR1B |= ((1 << CS12) | (1 << CS10));   // prescaler 1024
TIMSK1 |= (1 << OCIE1A);
TCCR1B |= (1 << WGM13);                  // <-- WGM13, not WGM12
OCR1A = 156;
```

Two independent bugs, multiplying to 32×:

1. **Wrong prescaler.** 10 ms at 1 MHz needs prescaler **64**, not 1024. → 16× slow.
2. **Wrong waveform mode.** `WGM13` alone, combined with the `WGM10` bit the
   Arduino core leaves set in `TCCR1A` (this function never clears `TCCR1A`),
   selects **mode 9 — phase & frequency correct PWM**, TOP = `OCR1A`. Phase
   correct counts up *and back down*, so the period is `2 × TOP`. → 2× slow.

`1e6 / 1024 / (2 × 156)` = **3.13 Hz**. Measured 3.0. The disabled `#if 0` block
twenty lines below has the correct `TCCR1B |= (1 << WGM12)` (CTC mode 4).

✅ **Confirmed to three digits, 2026-08-06.** PR #1's `millis()` timestamps make
the rate directly measurable instead of inferred from operator-observed
transitions. Consecutive `t=` deltas alternate 311 / 328 ms — mean **319.5 ms =
3.130 Hz**, against 3.13 Hz predicted.

This is now a real confirmation rather than the circular one §1 warned about. The
July 3.0 Hz figure was derived against `millis()` on an *assumed* clock; this one
is measured against `millis()` on a clock confirmed by fuse read, so the
oscillator no longer cancels out of the comparison. Both bugs are real and the
32× error is exactly as described.

### ✅ 2a. A third Timer1 defect: the self-calibration ran on one sample

Found on the bench 2026-08-07, fixed the same day. Independent of the two bugs
above, and arguably worse in the field.

```c
TCCR1B = 0;
TCCR1B |= ((1 << CS12) | (1 << CS10));   // clock started FIRST
TIMSK1 |= (1 << OCIE1A);
TCCR1B |= (1 << WGM13);
OCR1A = 156;                             // written LAST
```

Two hazards in the ordering:

1. **`TCNT1` is never cleared.** In mode 9 the counter cannot reach a TOP below
   its current value, so a stale `TCNT1` must run all the way to `0xFFFF` and
   wrap before the first compare match. At `F_CPU/1024` = 976 Hz that is **up to
   67 seconds**.
2. **`OCR1A` is double-buffered in mode 9.** Written after the mode bits, it does
   not take effect until the counter reaches the *previous* TOP — so the first
   period is governed by whatever `OCR1A` happened to hold, not by 156.

Measured: one sensor read at `t=5275`, the next at `t=72335`, then perfectly
regular 3.13 Hz. The 10-second self-calibration fell entirely inside that gap:

```
Zero-Calib-Value : 9.652528     <- byte-identical to the single t=5275 sample
```

**Every unit in the field has been deriving its zero reference from one
reading**, with no averaging and no noise rejection, and the value depends on
whatever the accelerometer happened to report at one arbitrary instant during
boot. This is not a bench artifact.

**Fix:** write `TCNT1 = 0` and `OCR1A` while `TCCR1B` is still 0 (normal mode,
unbuffered), and start the clock last. The prescaler and waveform mode are
deliberately left alone — those are the two bugs above and roadmap item 5.

After the fix, calibration averages ~28 samples and `Zero-Calib-Value` came out
as `9.691297`, matching no individual reading.

⚠️ **Item 5 must preserve this.** CTC mode 4 is not double-buffered, so hazard 2
disappears on its own, but `TCNT1 = 0` is still required.

### Why this breaks slow-speed detection

`RollingAvg<float> acceleration_avg_g(4)` at 3.13 Hz spans **1.28 seconds**.
An 18 fpm departure ramp is roughly 1.5–2 s. The detector is low-pass filtering
the event with a window nearly as long as the event itself, so the transient is
averaged away before any threshold sees it. At the intended 100 Hz the same
window spans 40 ms — negligible.

**The entire detection design assumes 100 Hz and is running at 3.**

---

## 3. 🔴 Constant velocity is not observable with an accelerometer

This is a physical constraint, not a tuning problem. A car cruising at 350 fpm
and a car parked both read exactly 1 g on Z. There is no signal to threshold.

Confirmed empirically. The firmware prints its own decision variable at each
timeout (`FSM: Delta : x`). Converted to true m/s² (×16.384, since these logs
predate the scaling fix):

| Run | Delta | Run | Delta |
|---|---|---|---|
| up_18 | 0.0030, 0.0017 | dwn_18 | 0.0126 |
| up_58 | 0.0454, 0.0088 | dwn_58 | 0.0112, 0.0115 |
| up_107 | 0.0199, 0.0022 | dwn_107 | 0.0045, 0.0033 |
| up_348 | **0.0986** | dwn_350 | 0.0431, 0.0214 |

n=15, **max 0.0986**, mean 0.0196 m/s².

- Old threshold 0.082 → fired once in 15 (up_348, still accelerating)
- Current threshold 0.400 → would fire **zero** times in 15

No threshold value fixes this. The `0.005 → 0.40` change in `7bc6cba` did not
break the check; it was already broken.

**The barometer would have solved it** (dP/dt is directly proportional to
vertical velocity; 18 fpm ≈ 1.1 Pa/s, well within DPS310 resolution) — but the
sensor is depopulated on this hardware. Not available.

### Consequence for the FSM design

The proposed accel → constant-velocity-plateau → decel FSM has the right shape,
but the middle transition cannot be measured. It must be **latched**:

| Phase | Observable? | Implementation |
|---|---|---|
| Acceleration | ✅ | detect transient → alarm ON, latch IN_TRANSIT |
| Constant velocity | ❌ | *assumed*. No timeout. Alarm persists. |
| Deceleration | ✅ | detect transient → arrival |
| End of run | ✅ | confirm stable at 1 g → alarm OFF |

**The fix for the constant-velocity complaint is to delete the timeout**, not to
detect the plateau. Latch on departure, hold indefinitely, release on the decel
transient. A 3-minute express run alarms for 3 minutes.

**The hard part:** you must be *listening* when the decel transient occurs — see
§5.

---

## 4. 🔴 The dead pressure path is rigged to report "at rest"

`pressure_avg_g(2)` is constructed with `init_val = 0` and **never receives a
single `.add()`** anywhere in the tree. In `MovementService::isAtRestOrStable()`:

```c
if (state >= MotionStates::STATE_DECELERATING) {
    float pres_avg = pressure_avg_ref->avg();      // always 0.0
    variance_pres = 0.0;
    for (...) variance_pres += (0 - 0) * (0 - 0);  // always 0.0
    if (variance_pres < 0.006) {                   // always TRUE
        pressure_varience_counter++;
    }
}
...
if (acc_varience_counter > 4 || pressure_varience_counter > 4) {   // OR
    return true;
}
```

Every 100 ms tick increments unconditionally. After ~500 ms in
`STATE_DECELERATING` or beyond, the function returns **true regardless of the
accelerometer** — the `||` lets the rigged vote override the real one.

`Release.txt` V1.2 advertises this as "redundant pressure checking that increases
the reliability of movement detection". It does the opposite. Delete the block.

---

## 5. 🟠 The buzzer blanks the accelerometer

`main.cpp`, in the timer ISR:

```c
if (get_alarm_status()) {
    if (get_buzzer_status() == false) {
        read_acceleration_mss();
    }
} else {
    read_acceleration_mss();
}
```

While the buzzer is on, the accelerometer is not read at all. Deliberate — it
dates to `b795dd2`, "Fixed the issue of not reading the sensor when the buzzer is
on... causing the movement detection to not work when the alarm was triggered."
Piezo vibration was polluting the rolling average.

The consequence is a logical deadlock: **the device cannot tell whether the car
is still moving while it is alarming.** That is what `59e945f` works around, by
stopping the buzzer at 10 s while leaving the chase LEDs running to 15 s,
creating a 5 s quiet listening window.

But `59e945f` also set `buzzer_on = true` in **both** branches of the beep duty
cycle:

```c
else {
    digitalWrite(PIN_PIEZO, LOW);
//  buzzer_on = false;
    buzzer_on = true;      // <- both branches
}
```

This is load-bearing, not a typo — it suppresses reads through the beep-*off*
phase too, presumably because the piezo keeps ringing mechanically. But it drops
sensor coverage to **one 5 s window per 15 s = 33%**, sampled in one lump. A
decel ramp is 1–3 s, so most will be missed.

Restoring `buzzer_on = false` on the off-phase gives a 300 ms window every
500 ms — **~50% coverage, sampled continuously**. Add ~30–50 ms blanking after
the pin drops for ringdown.

⚠️ **Do not reach for a low-pass filter.** The piezo *tone* is kHz and harmlessly
out of band, but the beep *envelope* (200 ms on / 300 ms off) is ~2 Hz, sitting
directly on the elevator motion band. It cannot be filtered out. Sampling in the
gaps is the only approach.

### Measured 2026-08-06: coverage during an alarm is zero, not 33%

`tk=` makes the blanking directly observable for the first time. Across a full
alarm cycle:

```
t=35454 ... tk=94
FSM: Buzzer timeout happened
t=45678 ... tk=95
```

**10,224 ms elapsed and `tk` advanced by one.** At 3.13 Hz that window should
hold ~32 samples; 31 were never taken.

This is worse than the 33%-per-15-s figure derived from source above. That
estimate assumed the 5 s quiet window yields usable coverage. In practice
sampling does not resume until the buzzer stops entirely, so **while the buzzer
is running, coverage is zero** — not reduced, absent. A decel transient of 1–3 s
falling in that window cannot be detected at all, no matter what thresholds are
used.

Everything else in this section stands.

### ✅ Fixed 2026-08-07 (roadmap item 7) — and the samples are clean

`buzzer_on` now goes false on the beep-off phase after a `BUZZER_RINGDOWN_MS`
(50 ms) blanking window measured from the falling edge of `PIN_PIEZO`. The piezo
does keep ringing mechanically after the drive pin drops — that part of
`b795dd2`'s reasoning was sound — but the remaining 250 ms of each 300 ms
off-phase is usable.

Measured over two alarms in one session:

| Window | Elapsed | Samples |
|---|---|---|
| `tk=59` → `tk=73` | 9,896 ms | **14** |
| `tk=125` → `tk=139` | 9,274 ms | **14** |

Against **1** before — roughly 45% coverage, close to the 50% predicted. Sample
intervals during the alarm are 639 or 950 ms, exactly 2× or 3× the 319.5 ms tick,
which is what beating a 319.5 ms sampler against a 500 ms beep cycle produces.
`bz=0` now appears on `ACC-INT` lines during `st=4`, confirming `buzzer_on`
actually toggles.

**The critical question was whether those in-gap samples are usable. They are:**

| | Range |
|---|---|
| Quiet baseline | 9.65 – 9.71 m/s² |
| During buzzer | 9.64 – 9.71 m/s² |

No measurable piezo contamination. **50 ms of ringdown blanking is sufficient —
do not raise it without evidence**, since every extra millisecond costs listening
time. The warning against low-pass filtering the ~2 Hz beep envelope still
applies and is now moot: sampling in the gaps works.

Both alarms cleared normally (Delta 0.0596 / 0.0539), so restoring the reads has
not destabilised the FSM. Self-calibration is now sampled through its own "ready"
beep as well, where it previously went blind for the final second.

**This unblocks roadmap item 8.** The latched departure/arrival FSM now has
something listening while the alarm sounds, which §3 identified as "the hard
part" and §11 confirmed any-motion cannot provide.

✅ **Fixed 2026-08-07 (item 7a) — and the claim below was wrong.**

This originally read "`check_for_battery_alarm()` drives `PIN_PIEZO` directly
and never touches `buzzer_on`". It does touch it. That claim came from a source
read that stopped two lines above the assignment, and it propagated into the
roadmap and two commit messages before being caught.

The two real defects were worse than the one described:

1. **No ringdown blanking.** It cleared `buzzer_on` the instant the pin went
   low, so the piezo's mechanical ringing was sampled unblanked — and counted
   as a trustworthy any-motion edge for arrival clustering.
2. **It spoke for the movement alarm.** `loop()` calls
   `check_for_active_alarm()` and then `check_for_battery_alarm()`, so the
   battery path writes `buzzer_on` last and wins. Its 1800/1800 ms pattern
   against the movement pattern's 200/300 cleared the flag for over a second at
   a time *while the movement buzzer was still beeping*.

Both fixed: ringdown blanking on the falling edge, and the flag is only cleared
when `alarm_status_g` is 0.

⬜ Still open: both paths write `PIN_PIEZO` directly and the last writer wins,
so with both alarms active the audible pattern is whichever ran most recently.
That is undefined behaviour rather than a chosen design.

---

## 6. ✅ Serial printing from the ISR (fixed in PR #1)

`Serial.print()` was called from inside `ISR(TIMER1_COMPA_vect)` via
`read_acceleration_mss()`. The ISR re-enables interrupts, so those writes raced
`HardwareSerial`'s TX ring buffer drain. **14 corrupted lines across 8 runs**,
every one inside a state-transition burst (`STA E_MONITORING`,
`STATE_MONIT RING`).

Fixed: the ISR publishes a snapshot, `loop()` prints. New format adds `millis()`
timestamps, I²C read duration (`rd=`) and an ISR overrun counter (`ov=`).

The timestamps and `rd=` both earned their keep immediately — §2's rate is now
confirmed to three digits and `rd=` turned out to be a usable sensor-health
signal (§10.2). But the overrun counter does not work.

### ✅ 6a. RESOLVED — and `ov=` was correct all along

**Final, 2026-08-07.** This section previously accused `ov=` of failing to count
decimation it should have caught. That accusation was wrong, and so was the
follow-up guess that the `F()`/RAM change had fixed it. Both are corrected here.

**`ov=5` is right.** `initialization()` drives the piezo high and then runs a
**1.6-second blocking loop of `delay(100)`** before printing "Device initialized
completely". The timer is already running by then, so the ISR publishes about
five samples that `loop()` never gets to print and dutifully counts as overruns.
Five overruns, one blocking init loop, exactly — and none afterwards, because
`loop()` keeps up at 3.13 Hz. `ov` was recording a real event, not missing one.

**The "half the samples vanish" symptom was §2a**, the timer-start defect: the
first compare match was delayed by up to 67 seconds while `TCNT1` wrapped, and
`OCR1A`'s double-buffering made the early periods arbitrary. Nothing was being
decimated; the timer genuinely was not ticking. `ov` had nothing to count
because the ISR had not run.

With §2a fixed, `tk=` advances 1:1 with printed lines from the first tick, and
timestamp deltas hold a steady 311/328 ms throughout.

**`tk=` earned its place anyway.** It is what distinguished "the ISR never ran"
from "the ISR ran and the print path dropped it", which is precisely the question
`ov` cannot answer, and it later gave the first direct measurement of §5's
blanking. Worth keeping.

The original — and incorrect — diagnosis follows, for the record.

### 🔴 6a (original, superseded) — `ov=` does not detect the decimation it exists to detect

Printed samples arrive in bursts of ~6 followed by a gap of 1917 or 2229 ms.
1917 ms is exactly 6 × 319.5 ms — six missing samples, so **roughly half of all
samples never reach the log**. Across every bench run `ov` read exactly **6**:
identical with good batteries, with a dead sensor, and with no batteries at all.
A counter that reports the same value under every condition is not counting.

`ov` was added specifically so that "non-zero means the log is being silently
decimated". It is non-zero, it is not tracking the decimation, and its constancy
makes it look like a stable startup artifact rather than an active problem.

⬜ **Not yet diagnosed.** The two candidates are (a) the ISR genuinely stops
firing during the gaps, in which case nothing is overrun and the samples are
never published, or (b) `sample_pending` is being cleared such that the overrun
branch is unreachable. (a) would be the more serious finding, since it would mean
the timer stalls for ~2 s at a time and no timing measured from these logs can be
trusted. Distinguish by incrementing a free-running counter in the ISR itself and
printing it alongside `ov`.

This is a defect in PR #1, not in the firmware it instrumented.

---

## 7. Threshold analysis

Units: `accel_value = g_value * 9.81`, so everything is **m/s²**.

| | Raw | True m/s² |
|---|---|---|
| Original (`z/16384` scaling) | 0.005 | **0.082** |
| Current (`z/1000` scaling) | 0.40 | **0.400** — 4.9× coarser |

The `z/16384` → `z/1000` change in `7bc6cba` is a **genuine units fix**: raw
`Data` values sit at ~998 where 1 g is expected, confirming the driver returns
milli-g. The old code was low by 16.384×. But the threshold was raised 80×,
where pure unit correction needs only 16.384× — so it is ~5× less sensitive than
the field-validated value.

### Sustain gating — the key result

Sweeping threshold × consecutive-sample-count over all eight runs. Counts are
spurious triggers on stationary windows (want 0):

| threshold | N=2 | **N=3** | N=4 | N=5 |
|---|---|---|---|---|
| 0.04 | 5 | **0** | 0 | 0 |
| 0.08 | 4 | **0** | 0 | 0 |
| 0.15 | 3 | **0** | 0 | 0 |
| 0.40 | 0 | **0** | 0 | 0 |

**At N≥3, every threshold from 0.04 to 0.40 gives zero false fires.** Requiring
3 consecutive samples lets the threshold drop 10× at no false-fire cost. You do
not need a higher threshold to stop false fires — you need a sustain
requirement, after which you can *lower* it and gain slow-speed sensitivity.

⚠️ Caveat: ~6 minutes of aggregate stationary data. Encouraging, not proof, for
a unit that sits in a hoistway for months. Needs a soak test.

✅ **Soak done 2026-08-07 (§14.6): 17.6 minutes parked, zero false departures,
zero any-motion edges, polled margin ~4×.** Retires this caveat for the cartop
case. The counterweight remains unmeasured (§14.5), and 17.6 minutes is still
short against a full-day construction deployment.

⚠️ These N values are at **3 Hz**. They must be recomputed after the timer fix —
N=3 at 100 Hz is 30 ms, not 1 s.

### The 18 fpm down miss

Full trace, deviation from baseline (1 char ≈ 0.33 s):

```
[  0- 59] t= 0.0s  |-=--==---=-==-==-=-===-_--==-=-=-==-====-----===--===-----=-|
[ 60-119] t=20.0s  |====-=----=====-=-=-==--==---=-=--==---=-====@@..==_=-=-=-=-|
[120-151] t=40.0s  |-------------====-=-=-----------|
                                                ^^ arrival, t≈35s, +1.35 m/s²
```

35 s of flat noise (±0.16 m/s²), then one clean +1.35 m/s² spike at brake set.
**No departure transient anywhere in the capture.** Either it occurred before
logging started, or it was below the noise floor. Not distinguishable without
timestamps — which the PR #1 change now provides.

No (threshold, sustain) combination recovers a departure from this data.

---

## 8. Roadmap

**Resequenced 2026-08-06.** The original roadmap was ordered around making
detection *better*. The bench session found three defects that make the unit lie
about whether it works at all (§10), and those now come first. A 100 Hz detector
on a sensor that silently reports zero is still a dead unit.

| # | Change | Status | Notes |
|---|---|---|---|
| 1 | Move `Serial.print` out of the ISR; add timestamps | ✅ PR #1 | `ov=` is defective, see 3a |
| 2 | Fix `TWI_FREQ`; instrument I²C read time | ✅ PR #1 | sensor was never affected — it's on TWI1 |
| 3 | **Check the `bma4_read_accel_xyz()` return code; fault instead of reporting 0.0** | ✅ PR #2 | §10.2. `a=ERR` path not yet exercised on hardware |
| 3a | Fix `ov=` so decimation is actually detected | 🔧 PR #2 | §6a. Symptom gone; `tk=` added, cause unproven |
| 4 | **Call `disable_battery_alarm()` when voltage recovers; average before latching** | ✅ PR #2 | §10.3. Confirmed on the bench |
| 7a | Blank the sensor during the battery alarm too | ✅ 2026-08-07 | §5. Ringdown added; no longer overrides the movement alarm's blanking |
| 4a | Decide what the unit should *do* when the sensor is dead | ⬜ | It no longer lies, but raises no user-visible fault. Product decision — ask Biju |
| 5 | **Fix Timer1: CTC mode (`WGM12`), prescaler 64 → 100 Hz** | ⬜ | Was "**the** detection fix". §12 and §14 largely retire that: departure and arrival are both carried by the sensor's own 100 Hz engine, so the firmware rate is no longer the blocker. Still worth doing; no longer urgent |
| 6 | Delete the dead pressure path | ⬜ | §4 |
| 7 | Restore `buzzer_on = false` on beep-off phase | ✅ PR #4 | §5. ~45% coverage, samples clean. Unblocks item 8 |
| 8 | Departure-latch / decel-release FSM, no timeout | ✅ PR #5 | §13, §14. Validated 18–123 fpm, both directions |
| 8a | Arrival release: replace the duration-fragile `ARRIVAL_CLUSTER_DELTA` with a statistic that does not degrade as runs get longer | ⬜ | §14.3. Works today, wrong in principle. **Do 8c first** — it may make this moot |
| 8c | **Decide whether automatic release should exist at all** | ⬜ | §14.9. Top open question. A gentle 18 fpm arrival measured *below* the parked noise floor, so no threshold can reach it. Manual silence cannot fail dangerously |
| 8b | Counterweight characterisation — and logging is not possible safely there | ⬜ | §14.5. Largest remaining risk. Needs on-device recording, see 11 |
| 9 | Sustain-gated threshold, recomputed at 100 Hz | ⬜ | §7. Superseded in practice by the sensor-side threshold+duration gate |
| 11 | On-device black box: record per-run stats to EEPROM, dump over serial afterwards | ⬜ | The only way to characterise the counterweight without a cable in a live hoistway |
| 10 | Cleanups: `#if 0` block, `movement_service.cpp.original`, `current_time` member, missing EOF newline, untrack `.pio/` + `compile_commands.json`, stray `falcon_srcs.code-workspace` in `src/` | ⬜ | |

**Sequencing note:** re-run the EFT after 5–7 and before 8–9. At 100 Hz with a
40 ms average window the detector may behave so differently that the tuning
question changes completely. Every number in the July report describes a system
running 32× slower than designed.

**Item 5 is no longer a free change.** A working I²C read costs 6144 µs (§10.4),
which is 61% of a 10 ms period before any float math or FSM work. Raising the ISR
to 100 Hz at F_CPU = 1 MHz will not fit. Either raise F_CPU to 8 MHz alongside
the timer fix — at a battery-life cost that has not been budgeted — or move the
sensor read out of the ISR. Measure before committing to either.

### Open questions

- ✅ Confirm `lfuse = 0x62` (1 MHz) on the bench. **Done 2026-08-06, §1.**
- ✅ Measure `rd=`. **Done 2026-08-06: 6144 µs, not the ~1400 predicted.** The
  sensor *is* on TWI1 as read (nowhere near 40,000), but the read is 4.4× more
  expensive than estimated. See §10.4 — this is the binding constraint on
  roadmap item 5.
- ⬜ Why is `rd=` 4.4× the estimate? The 1400 µs figure came from 8 bytes ×
  9 bits ÷ 62.5 kHz plus overhead. Candidates: `bma4_read_accel_xyz()` issuing
  more transactions than assumed, per-byte overhead in the `Wire1` path, or
  `TWBR1 = 0` not actually yielding 62.5 kHz. A scope on SCL settles it.
- ⬜ `TWBR1 = 0` violates the datasheet's "TWBR ≥ 10 in Master mode". Left
  unchanged deliberately; measure before touching. Now also a suspect for the
  `rd=` overrun above.
- ⬜ Is the ~2 s sample gap (§6a) a logging artifact or a genuine timer stall?
  Blocks trusting any timing measured from these logs.
- ⬜ Ask Biju **why 0.40** specifically, and whether `59e945f` has had any
  hoistway time yet.
- ✅ Does `Anymotion` supersede any of this? **Investigated 2026-08-06: no.**
  See §11. It is a sleep-mode experiment with an any-motion skeleton around it;
  every any-motion path is `#if 0` or commented out, and the Bosch
  implementation file is absent. It does **not** block roadmap item 7.
- ⬜ Does `Falcon_Rel_EFT_FreeRTOS` supersede any of this? Not yet looked at.
- ✅ Which MCU pin do `INT1_ACC` / `INT2_ACC` land on? **Confirmed from the
  schematic by Dave, 2026-08-06: `INT1_ACC` → INT0/PD2, `INT2_ACC` → INT1/PD3.**
  Both external interrupt pins, both accounted for. No respin needed. See §11.
- ⬜ Can the BMA456 INT pin be configured active-low? Required for wake-from-
  power-down, which only supports level-triggered INT0/INT1 (§11). Bosch's
  `bma4_int_pin_config` exposes the polarity; it has not been checked against
  what the hardware pull-ups allow.
- 🔧 At 1 MHz, is there CPU headroom for a 100 Hz ISR doing an I²C read plus
  float math? **Largely answered: no.** The read alone is 61% of the budget
  (§10.4). Raising F_CPU to 8 MHz is the lever — at the cost of battery life,
  which given §10.3 is now a sensitive number. Moving the read out of the ISR is
  the alternative worth costing.
- ⬜ Is the serial back-feed (§10.1) present on production units, or an artifact
  of this particular CH340 adapter? Changes whether it is a bench-procedure note
  or a hardware fix.
- ⬜ What is `Voltage value` actually in? Raw ADC counts, not mV — the threshold
  is `< 1600` in code while `Release.txt` describes 3.2 V. The scale factor has
  not been established, so no reading in this document can be converted to volts.

---

## 9. Reproducing the analysis

```
python falcon_srcs/graph/parse_falcon_log.py <logfile> [--plot]
```

Reads both log formats. Old captures are corrected on read (×16.384 scale fix,
m/s² relabel) so July data stays comparable with future runs. Reports sample
rate, `RollingAvg(4)` span, stationary noise floor, the firmware's own delta,
I²C read time against the ISR budget, overrun growth and corrupted lines.

Source captures: branch `eft-results-2026-07-15`, `Eng_Notes/falcon_log_*_fpm`.

---

## 10. Bench session 2026-08-06 — power and failure modes

PR #1 was flashed to hardware (`avrdude` verified 21110 bytes) and run on the
bench. Everything below was measured that session. §1 and §2 record the two
predictions it confirmed; this section records what it found that log-only
analysis could not.

The theme: **§2–§7 are about detecting elevator motion better. These are about
the unit failing without saying so.** In a hoistway, a device that silently
reports "no motion" is worse than one that detects motion badly.

### 10.1 🔴 The serial cable back-powers the board

With **batteries removed** and only the USB-serial cable attached, the device
boots, runs the FSM at 3.13 Hz, prints clean telemetry, and drives the piezo
(faintly). Confirmed by direct test.

The adapter idles TX high. That voltage reaches the ATmega's RX pin, forward-
biases the internal ESD clamp diode into VCC, and back-feeds the rail; GND is the
return path. The board sits near (TX − 0.6 V), current-limited by what one I/O
pin can source — hence the weak piezo.

Consequences:

- **Any battery reading taken with serial attached is measuring a contested
  rail.** The divider cannot distinguish battery current from adapter current.
  This is why fresh cells read *lower* (2372) than the depleted ones they
  replaced (2500).
- **The rail can sit in the band where the ATmega runs but the BMA456 does
  not** — see §10.2.
- Pulling the batteries does **not** power-cycle the device while the cable is
  connected. Anything latched stays latched (§10.3).

**Bench procedure:** power the device first, connect serial second, disconnect
serial before any battery swap, and discard the first `Voltage value` after boot.
To force a true cold boot, remove batteries *and* cable.

### 10.2 🔴 A dead sensor is reported as a valid reading of 0.0

`getAcceleration()` discards the return code:

```c
void BMA456::getAcceleration(float* x, float* y, float* z) {
    struct bma4_accel sens_data;
    bma4_read_accel_xyz(&sens_data, &accel);   // return code never checked
```

`read_acceleration_mss()` zeroes `x = y = z = 0` before the call, so a failed
read is indistinguishable from a genuine reading of zero — and zero is exactly
what the FSM interprets as "perfectly still". The unit reports the all-clear.

Observed twice, in both cases with the rail compromised per §10.1: 55+ seconds of
well-formed telemetry, correct timestamps, FSM in `STATE_MONITORING`, and
`a=0.0000 avg=0.0000` throughout.

**`rd=` distinguishes the two states, which PR #1 did not anticipate:**

| Condition | `rd=` | `a=` |
|---|---|---|
| Weak cells + cable | **4736** | 0.0000 |
| Good batteries | 6080–6272 | 9.74 |
| No batteries, cable only | **4736** | 0.0000 |

4736 µs to the microsecond in both failure cases, against ~6144 µs for a working
read. `rd=` is a usable health signal.

⚠️ **Correction, 2026-08-07.** This section originally read the 4736 µs as "the
transaction aborting because the sensor never acknowledges". That was wrong, and
so was the first fix built on it.

The bus completes normally. The failure is that **the transport reported success
no matter what**:

```c
static uint16_t bma_i2c_read(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t len) {
    ...
    Wire1.requestFrom((int16_t)addr, len);
    while (Wire1.available()) { data[i++] = Wire1.read(); }
    return 0;                      /* always */
}
```

`endTransmission()`'s result was discarded and the byte count never checked. When
the sensor did not answer, the loop body simply never ran and the caller was told
everything was fine — so checking `bma4_read_accel_xyz()`'s return code, which
roadmap item 3 originally did, had **no effect whatever**. The shorter `rd=`
reflects a read that returned early with no data, not an aborted transaction.

A sensor can also acknowledge and return zeros while browning out, which passes
every bus-level check. Both are now handled: the wrappers return `BMA4_E_FAIL` on
a NACK or short read, and `getAcceleration()` rejects all three axes reading
exactly zero.

⬜ **Still not exercised on hardware.** No read failure has occurred since the
fix, so the `a=ERR` path remains unproven in practice.

At rest a healthy unit reads **9.74 m/s²** against 9.81 nominal, ~0.7% low, which
is unremarkable for an uncalibrated part.

### 10.3 🔴 The battery alarm latches permanently and cannot clear

```
value : 1545
  LOW Battery detected
```

`main.cpp` tests `battery_v < 1600` on a **single instantaneous sample** and
calls `enable_battery_alarm()`. Every subsequent reading that session was
2324–2390 — the battery was fine.

**`disable_battery_alarm()` is defined in `alarm.cpp` and never called from
anywhere in the tree.** Once latched, nothing in the firmware clears it. The unit
beeps until someone performs a true cold boot (§10.1). In a hoistway that is a
site visit.

Three compounding faults:

1. **No recovery path.** Dead code where the clear should be.
2. **No averaging.** The commented-out `battery_avg.avg() < 1660` immediately
   above the live check suggests this was already suspected. `battery_avg` is
   populated and then not used for the decision.
3. **The sample is taken at the worst moment.** `adc_loop_counter` enables the
   divider at 30000 iterations and reads at 30100. Those are loop passes, not
   milliseconds — a very short settle at 1 MHz — and the first one lands during
   startup, when the boot buzzer pulse and chase LEDs are loading the rail.
   The 1545 was the first battery reading after boot.

**Alarm patterns are distinguishable by ear**, which is the fastest field
diagnostic available:

| Alarm | Period | Pattern |
|---|---|---|
| Movement | `BEEP_FLASH_TIME_MS` 100, `counter_b % 5` | 200 ms on / 300 ms off — rapid chirp |
| Battery | `BATTERY_FLASH_TIME_MS` 600, `counter_a % 6` | **1800 ms on / 1800 ms off — long beep** |

The comment above the battery block claims "200 ms on, and 300 ms off". It is
copy-pasted from the buzzer function and wrong for a 600 ms period.

### 10.4 🟠 The I²C read costs 61% of the 100 Hz budget

A working `bma4_read_accel_xyz()` measures **6144 µs**, against the ~1400 µs §8
estimated. Quantized to 64 µs steps, as expected for `micros()` at F_CPU = 1 MHz
with prescaler 64, so the figure is real and not a measurement artifact.

At the current 319.5 ms period this is 1.9% of the budget and harmless. At the
intended 10 ms period it is **61%**, before float math, the rolling average, or
the FSM. Roadmap item 5 does not fit at F_CPU = 1 MHz. See §8 for the options.

### 10.5 Session log inventory

Not committed — the bench runs were monitor sessions rather than PuTTY captures,
so no log files exist for `parse_falcon_log.py`. Everything above is quoted
inline. **Capture to file next session**; the parser already reads this format
and the gap analysis in §6a would be far easier against a real log.

---

## 11. The `Anymotion` branch — investigated 2026-08-06

§8 asked whether `Anymotion` was a BMA456 hardware-interrupt experiment that
would supersede §5. **It is not, and it does not.** The name is aspirational.

### What is actually on the branch

A single commit, `0e35726` ("Added Anymotion branch", Dec 2025), branching from
`6ec6980` — which predates both the ATmega328PB port and the BMA456 TWI1 patch.

Every any-motion path is inert:

| Piece | State |
|---|---|
| Sensor-side any-motion config (`bma456_map_interrupt`, `bma456_configure_anymotion`) | inside `#if 0` |
| `attachInterrupt(digitalPinToInterrupt(BMS456_INTERRUPT), bosch_interrupt, RISING)` | commented out |
| Interrupt-status polling in `loop()` | inside `#if 0` |
| `bma456_configure_anymotion()` | **defined nowhere in the tree** |
| `bma456_an_read_int_status()`, `bma456_an_set_any_mot_config()` | declared in `bma456_an.h`, **defined in no `.c` file** |

The branch carries Bosch's any-motion *header* but not the matching
`bma456_an.c`; `bma456.c` is still the base variant. Nothing live calls the
missing functions, which is why the undefined references never break the build.

What *is* active is unrelated to any-motion: `SLEEP_MODE_PWR_DOWN` with
`sleep_cpu()`, `RollingAvg` sizes cut from 8/4 to 1, DPS310 commented out, and
serial gated behind `SERIAL_EN`. **It is a power-down sleep experiment.**

It is also on the wrong baseline — `[env:ATmega328P]`, sensor on `Wire`/TWI0, no
`board_build.f_cpu`. There is nothing mergeable. Current `Falcon_Rel_EFT` does
not carry `bma456_an.h` at all.

### Consequences

**Roadmap item 7 is unblocked.** Restoring `buzzer_on = false` on the beep-off
phase remains the real fix for §5, and item 8 cannot proceed without it.

**Any-motion would not dodge §5's root cause anyway.** Reads are blanked because
piezo vibration couples mechanically into the accelerometer, and the sensor's own
any-motion engine sits behind the same physics — it would see the buzzer too. The
work would move the threshold decision from firmware into the sensor, not escape
the coupling. It may still win, because the sensor's threshold and duration are
tunable independently of the FSM and any-motion is a *transient* detector, which
suits §3's latch-on-departure / release-on-decel design better than polling does.
That is a hypothesis to test, not a free pass.

**The sleep work is relevant elsewhere.** If the Timer1 fix forces F_CPU to 8 MHz
(§10.4), battery life is the cost, and power-down between samples is the obvious
offset. Someone has already prototyped it here. Worth reading before costing that
option.

### Hardware: the interrupt lines exist

`common.h` on the branch defines `BMS456_INTERRUPT = PIN_PD2`, with `PIN_PD3`
commented as an alternative. PD2 is INT0 and PD3 is INT1 — the ATmega328PB's only
two external interrupt pins.

The OrCAD design file `HW_Docs/02_Desing files/RTC1273R2.DSN` confirms the
accelerometer interrupts are brought out as named nets:

- `BMA456_1` carries pins `VDDIO ASDA INT1 INT2 GNDIO ASCL`
- Nets **`INT1_ACC`** and **`INT2_ACC`** appear on `PAGE05:SENSOR` **and** on
  `PAGE04:uC`

✅ **Confirmed from the schematic by Dave, 2026-08-06:**

| Net | MCU pin |
|---|---|
| `INT1_ACC` | **INT0 / PD2** |
| `INT2_ACC` | **INT1 / PD3** |

Both accelerometer interrupt outputs land on true external interrupt pins, and
between them they consume both of the ATmega328PB's. The `PIN_PD2` in the
Anymotion branch's `common.h` was right, and the commented `PIN_PD3` alternative
is the *other* sensor interrupt rather than an alternate route for the same one.

**No board respin is required for interrupt-driven operation.** The hardware has
been ready the whole time; only the firmware is missing.

### What the two lines make possible

Having *both* interrupts wired is more useful than one. The BMA456 exposes
any-motion and no-motion as separate features, and each can be mapped to its own
pin. That maps directly onto the latched FSM §3 argues for:

| §3 phase | Sensor feature | Pin |
|---|---|---|
| Departure transient → alarm ON, latch | any-motion | INT1_ACC / PD2 |
| Constant velocity — *assumed*, not measured | — | — |
| Arrival → release the latch | no-motion | INT2_ACC / PD3 |

That is the §3 design implemented in silicon, with the sensor doing the
thresholding continuously and the MCU only reacting to edges. It also sidesteps
§2's problem for the *detection* path specifically: the sensor runs its own
100 Hz ODR internally regardless of how slowly the firmware's timer ticks.

It does **not** sidestep §5. The piezo still couples mechanically into the
accelerometer, and the sensor's engine sits behind the same physics — it would
see the buzzer too. But the discrimination moves into tunable sensor registers
(threshold plus duration) instead of a rolling average the buzzer has to be
blanked around, which is a better place to fight it.

### ⚠️ Constraint if this is combined with sleep

`SLEEP_MODE_PWR_DOWN` stops the I/O clock, so **only a low-level-triggered INT0 /
INT1 can wake the part — edge detection does not work in power-down.** The
Anymotion branch's commented-out line asks for `RISING`:

```c
//    attachInterrupt(digitalPinToInterrupt(BMS456_INTERRUPT), bosch_interrupt, RISING);
```

That would never have woken the MCU from the `sleep_cpu()` on the same branch.
Anyone reviving this needs the BMA456 INT pin configured active-low and
`attachInterrupt(..., LOW)`, or the sleep and the interrupt will silently fail to
work together.

### Bench results, 2026-08-07 — the interrupt path works, and it does not escape §5

The any-motion engine was armed at threshold 96 / duration 5, mapped to
INT1_ACC → INT0/PD2, running alongside the polled detector and changing no FSM
behaviour. Three questions were posed; all three are now answered.

#### ✅ 1. The hardware path works

`INT1_ACC → PD2` is live. Twenty interrupts in one session, each `ACC-INT` line
showing `pin=1` at the edge and `pin=0` after the status read.

**`BMA4_NON_LATCH_MODE` alone is not sufficient.** Setting it returned `BMA4_OK`
and the pin still latched: one edge at `t=35471` and nothing for the following
150 seconds, across three handling events the polled detector caught easily. An
explicit read of the status register (`bma456_read_int_status`, returning
`s=0x40` = `BMA456_ANY_NO_MOTION_INT`) is what drops the pin and re-arms it.
Whether the mode bit is being overwritten by the feature-config write or simply
does not behave as the datasheet implies has not been established — but any
design here must poll the status register, not rely on non-latch.

#### 🔴 2. The buzzer triggers any-motion continuously

```
ACC-INT n=4  t=20103 st=4 bz=1 pin=1
ACC-INT n=5  t=21118 st=4 bz=1 pin=1
ACC-INT n=10 t=63668 st=4 bz=1 pin=1
ACC-INT n=11 t=64684 st=4 bz=1 pin=1
ACC-INT n=12 t=65863 st=4 bz=1 pin=1
```

Those intervals are ~1015 ms — **exactly the status-poll period**. The condition
is not firing once; it re-asserts the instant it is cleared. While the buzzer
runs, any-motion is permanently triggered at this threshold.

This confirms by measurement what §11 predicted from the physics: the sensor's
engine sits behind the same mechanical coupling as the rolling average, and it
hears the piezo.

**Consequence for the §3 latched FSM — this is the important one.** The design
needs a decel transient detected *while the alarm is sounding*. Neither feature
can do it:

| Release mechanism | Why it fails during the alarm |
|---|---|
| any-motion | permanently asserted by the buzzer — no usable edge |
| no-motion | the buzzer prevents stillness, so it never asserts |

**Moving detection into the sensor does not rescue §5.** Roadmap item 7
(`buzzer_on = false` on the beep-off phase) is required whichever architecture
wins. That was the main hope for the interrupt approach and it is now closed.

#### ✅ 3. Any-motion is more sensitive than the polled detector

Events `n=13` through `n=19` fired during `STATE_MONITORING` on handling the FSM
**never flagged** — readings in the 9.2–10.5 m/s² range that `RollingAvg(4)` at
3.13 Hz smooths away before any threshold sees them.

This is the first evidence that anything on this hardware can detect events the
current design misses, and it bears directly on §3's 18 fpm departure problem,
where the transient sat below the polled noise floor. It does **not** prove an
18 fpm departure would be caught — that needs a hoistway — but it is the most
promising result of the two bench days.

⚠️ All three axes were enabled so a hand-wave would register. Elevator motion is
Z-only; narrow to `BMA456_Z_AXIS_EN` before drawing conclusions about real
detection performance, since X/Y pick up handling a mounted unit never sees.

#### Where this leaves the architecture

The interrupt path is viable, more sensitive than polling, and needs no board
change. But it inherits §5 rather than solving it, so it is not a shortcut around
the existing roadmap — it is an alternative detector that still requires item 7,
and still cannot observe constant velocity (§3 is a physical constraint, not an
implementation one).

**Item 7 has since landed** (§5), restoring ~45% coverage during an alarm with
clean samples. That removes the blocker for both architectures rather than
favouring either, so the choice between them still rests on the open question
below.

✅ **Answered 2026-08-07 in the hoistway: yes.** See §12.

---

## 12. ✅ Hoistway results, 2026-08-07 — the 18 fpm departure is detectable

The question this document has carried since July — why the unit cannot detect a
slow-speed departure, and whether anything can — is answered. **The sensor's own
any-motion engine detects an 18 fpm departure. The polled detector never has.**

Capture: `logs/device-monitor-260807-102954.log`, 18 fpm down, threshold 32,
duration 5, Z axis only.

### 12.1 The run

Stationary baseline over the 4 s before departure: 9.6938–9.7639 m/s²,
spread **±0.035**.

| Event | t (ms) | `a=` | Δ from baseline | Any-motion | Polled FSM |
|---|---|---|---|---|---|
| **Departure** | 762462 | 9.6166 | **−0.116** | ✅ n=38, 39 | ❌ |
| Cruise, 20.3 s | 763–783 k | 9.55–9.94 | ±0.19 | ❌ silent ✅ | ❌ silent ✅ |
| **Arrival** | 783663 | 8.2473 → 11.6434 | −1.49 / +1.91 | ✅ n=40–42 | ✅ t=785301 |

**−0.116 m/s² is a third the size of the 38 fpm departure measured earlier the
same day, and it fired.** §3 concluded from the July captures that the 18 fpm
departure sat below the noise floor and that "no (threshold, sustain)
combination recovers a departure from this data". That conclusion was correct
*for the polled path at 3.13 Hz through a `RollingAvg(4)`*. It does not hold for
the sensor running its own engine at 100 Hz.

### 12.2 ⚠️ Amplitude is not what discriminates — duration is

The most important result here is easy to miss:

**Cruise excursions reached ±0.19 m/s², larger than the 0.116 departure, and
produced no interrupts at all.**

On a pure-amplitude model — which is how the threshold was picked, including the
ratio tables in `main.cpp` — cruise should have fired well before the departure
did. It did not. The discriminator is the **100 ms duration gate**: a departure
is a sustained acceleration ramp that holds above threshold, while hoistway
vibration is brief spikes that do not.

Consequences:

- **Do not lower the threshold further.** 32 detects the smallest transient yet
  recorded. There is no case for 24, and going lower begins to expose the spikes
  that duration is currently filtering out.
- **`ANYMOTION_DURATION` is the real tuning knob**, not the threshold. If false
  fires appear, raise it.
- Detectability **cannot be inferred from the 3.13 Hz `a=` peaks**. The polled
  samples undersample a 1–2 s event badly; the sensor sees a waveform the log
  does not show.

### 12.3 The two detectors are complementary

Across every hoistway and bench run at 32:

| | Departure | Cruise | Arrival |
|---|---|---|---|
| **Any-motion** | ✅ down to −0.116 | silent | ✅ |
| **Polled FSM** | ❌ never | silent | ✅ every time |

The polled path misses departures structurally, not by mistuning: a
`RollingAvg(4)` at 3.13 Hz spans 1.28 s and flattens a 1–2 s ramp. On the 38 fpm
run it reduced a raw +3.25 m/s² arrival to a delta of 0.755, and on an earlier
run reduced +1.31 to 0.32 — below the 0.400 trigger, so even a *large* event was
erased. It detects arrivals only because they are violent enough to survive that.

**The unit therefore alarms when a car arrives, not when it departs.** That is
backwards for the product, and it is a consequence of §2, not of any threshold
setting.

### 12.4 What this means for the roadmap

**It decouples item 8 from item 5.** The 100 Hz timer fix was needed because
polling at 3.13 Hz cannot see departures. If departure detection moves into the
sensor, the firmware sample rate stops being the blocker — which matters, because
item 5 is still stuck behind the unbudgeted F_CPU/battery decision (§10.4).

The design the data supports is a **hybrid**, playing each detector to its
demonstrated strength:

| Phase | Detector | Action |
|---|---|---|
| Departure | any-motion interrupt | alarm ON, latch IN_TRANSIT |
| Constant velocity | — | *assumed*, no timeout (§3) |
| Arrival | polled FSM | release the latch |

This also resolves the release problem §11 identified. Any-motion cannot release
the alarm because the buzzer triggers it continuously — but it does not need to,
because release comes from the polled side, which item 7 gave ~45% listening
coverage during an alarm (§5).

⬜ **Open: a long stationary soak.** The quiet stretches measured so far are
seconds to minutes. §7's caveat stands and applies harder at threshold 32: a
false-fire rate that looks like zero over minutes may look very different over
the months a unit sits in a hoistway.

⬜ **Open: `parse_falcon_log.py` does not know the new log lines.** It reports
all `ACC-INT` / `ACC-STAT` lines as "corrupted lines — serial contention", and
its stationary-noise figures are inflated because its window spans whole runs
including the motion. Fix before trusting its summary.

---

## 13. ✅ Item 8 validated in the hoistway, 2026-08-07

Three consecutive runs, latched FSM, threshold 32 / duration 5 / Z axis only.
**All three latched on departure, held the alarm through cruise, and released on
arrival.** No failsafe, no mid-ride release, no double alarm.

| Run | Departure | Arrival | Alarm held | Released by |
|---|---|---|---|---|
| A | t=172294 | t=179159 | 6.9 s | clustered any-motion, 2 edges |
| B | t=188284 | t=270024 | **81.7 s** | clustered any-motion, 2 edges |
| C | t=280051 | t=366772 | **86.7 s** | polled, delta 0.520 |

This is the §3 design doing what it was written for: latch on departure, assume
constant velocity, release on the arrival transient. The original complaint —
the alarm stopping part-way through a slow run — is fixed.

### 13.1 Cruise held for 80+ seconds with zero false releases

Runs B and C cruised for 80.5 s and 84.7 s. Across the whole of run B's cruise
**not one any-motion edge was raised** (`im=6` from t=188825 to t=269975), and
the polled average never came near the 0.30 arrival threshold.

That is the behaviour every earlier attempt failed at. For comparison, the same
stretch would previously have been cut short by:

- the fixed 10 s buzzer / 15 s LED timeout (the original design)
- the 60 s failsafe (fired mid-ride, split one trip into two alarms)
- the stillness release (fired at 15 s, twice)
- a buzzer-raised edge completing an arrival cluster (fired at ~10 s)

Both runs also exceed the 60 s failsafe that was in place this morning, so
raising it to 180 s was necessary, not merely cautious.

### 13.2 Both arrival paths earned their place

Two runs released on clustered any-motion; **run C released on the polled
backup**, because only one quiet edge had arrived before the averaged delta
crossed 0.30.

That is the case the secondary path was kept for. Deleting it — which the
sensitivity comparison in §12 would have justified on its own — would have left
run C running to the failsafe.

### 13.3 The buzzer-edge filter did its job

Run A raised `ACC-INT n=2 t=173686 st=4 bz=1` four seconds into the alarm. Under
the previous build that edge was eligible to form a cluster; on 2026-08-07 an
edge exactly like it released an alarm mid-ride. It was discarded here, and the
real arrival still registered two clean edges 5 s later.

Notably, runs B and C raised **no** buzzer edges at all during 165 s of combined
alarming. Earlier runs saw roughly one per 6–10 s. That variability is itself
worth noting: the coupling is mounting- and ride-dependent, so the filter should
not be removed just because a given session happens not to need it.

### 13.4 What is still not established

⬜ **One unit, one shaft, one afternoon.** Three arrivals is not a sample. The
arrival logic in particular was rewritten three times today, each revision
prompted by the previous run's failure, and it now rests on five observed
arrivals total. Departure detection is on much firmer ground.

✅ **Stationary soak done (§14.6).** 17.6 minutes parked: zero false departures,
zero any-motion edges, polled margin ~4× with a p99 of 0.032. Cartop only.

🔧 **Battery — downgraded.** Runs B and C sounded the buzzer continuously for
80+ seconds. Across the whole 2026-08-07 session, including many such runs, the
reading fell 2236 → 2209 counts and held steady through the soak. Comfortable
for a shift, and §14.4 establishes the device is used case by case rather than
continuously, so this is no longer a headline concern.

✅ **Item 7a done (§5)** — though the description here was wrong.
`check_for_battery_alarm()` *does* set `buzzer_on`. The real defects were the
missing ringdown blanking and the fact that it overrode the movement alarm's
blanking for over a second at a time.

⬜ **The counterweight (§14.5).** The largest remaining risk. Every release
threshold is fitted to cartop data, the device deploys on a counterweight, that
ride is rougher, and **logging there is not possible safely** — so the
measure-then-tune method used all day is unavailable exactly where it matters
most.


---

## 14. Speed sweep, 2026-08-07 (cartop inspection)

Five consecutive correct runs across 18-123 fpm, both directions, on the
latched FSM with corroborated arrival detection. No mid-travel releases, no
failsafes, no double alarms, and zero any-motion edges during cruise on every
run.

| Speed | Dir | Held | Cruise peak | Arrival delta | Released by |
|---|---|---|---|---|---|
| 18 fpm | down | 171.6 s | 0.271 | 0.312 | polled |
| ~50 fpm | down | 53.4 s | 0.086 | 0.278 | any-motion |
| 58 fpm | down | 60.7 s | 0.218 | 0.383 | polled |
| 123 fpm | up | 36.1 s | 0.124 | 0.362 | polled |
| 123 fpm | down | 34.8 s | 0.103 | 0.362 | any-motion + delta |

All figures from `parse_falcon_log.py`, which now splits each run into cruise
and arrival windows -- the aggregate numbers it printed before hid exactly the
distinction that matters.

### 14.1 Both release paths are load-bearing

Neither detector covers the range alone, and today produced a case where each
rescued a run the other would have missed:

- The **~50 fpm descent** released on clustering at delta 0.278. The polled
  threshold is 0.30, so polling alone would have missed it.
- The **18 fpm descent** released on polling at 0.312 with a single arrival
  edge. Clustering alone would have missed it.

Section 12's sensitivity comparison would have justified deleting the polled
path. That would have been wrong.

### 14.2 Direction is symmetric

123 fpm up and down gave near-identical arrival deltas (0.362 both) and
comparable cruise peaks (0.124 / 0.103). The only false release of the day --
12 s into a 54 s ride at 58 fpm up -- was the uncorroborated clustering rule,
and it does not recur once the polled average has to agree.

### 14.3 Cruise peak scales with run DURATION, not speed

| Held | Cruise peak |
|---|---|
| 34.8 s | 0.103 |
| 36.1 s | 0.124 |
| 53.4 s | 0.086 |
| 60.7 s | 0.218 |
| **171.6 s** | **0.271** |

The fastest runs gave the *lowest* cruise peaks and the slowest gave the
highest. Sorted by duration it is near-monotonic; sorted by speed it is noise.

That is what a maximum does: a 171 s run takes ~500 samples and a 35 s run
~110, so the long run has more chances to throw an outlier. **Sample count is
the driver, not speed.**

This makes `ARRIVAL_CLUSTER_DELTA = 0.20` wrong in principle even though it
works here. Cruise peak will keep creeping up on longer runs; in a taller
building at 18 fpm a 300 s run would plausibly exceed 0.20 while also having
more opportunity to cluster, and the corroboration gate would stop rejecting.

- [ ] The fix is not a larger constant. Either compare the arrival against
  *recent* cruise rather than a running maximum, or use a percentile so one
  outlier in 500 samples does not set the bar. Both are real changes and
  deserve more than the seven runs behind them.

### 14.4 Failure modes are NOT symmetric -- this is a life-safety tool

Context from Dave, 2026-08-07: this is a mechanic's device, placed in the
hoistway -- initially on the counterweight -- to warn of unintended movement
while working. "The silent killer." Used case by case, occasionally a full day
for construction crews on running platforms.

That inverts how these parameters should be biased:

| Failure | Consequence |
|---|---|
| Missed departure | The mechanic is not warned. **Catastrophic.** |
| Released mid-travel | Alarm stops while the counterweight is still moving, the exact moment it is needed. **Catastrophic.** |
| Fails to release / alarms too long | Nuisance. The mechanic is present and knows the state. |
| False alarm while parked | Nuisance, but erodes trust -- alarm fatigue is its own hazard. |

**The safe direction is always "alarm longer."** Much of 2026-08-07 was spent
treating a stuck alarm as comparably serious to a mid-travel release, including
shortening `LATCH_FAILSAFE_MS` to 60 s on that basis. That was backwards.

Consequences for tuning:

- **Departure sensitivity is the safety-critical parameter.**
  `ANYMOTION_THRESHOLD = 32` caught an 18 fpm departure at 1.5x margin -- the
  thinnest safety-relevant number in the system.
- **Arrival gates should be biased toward NOT releasing**, not centred between
  their failure cases. Failing long is free; failing short is not.
- Battery concern downgrades: 2236 -> 2209 counts over two hours of heavy use
  is comfortable for a shift, and continuous full-day use is not the norm.

### 14.5 Every number here is from the CARTOP; the device deploys on a COUNTERWEIGHT

Different rails, guide shoes and mass, travelling opposite to the car -- and
confirmed by Dave to be a rougher ride than the cartop.

A rougher ride pushes **both** release paths toward firing during travel:
cruise peak rises toward the 0.30 polled threshold, and more cruise edges make
clustering pair up while the higher cruise delta also clears the 0.20
corroboration gate. Both land on mid-travel release -- the dangerous failure.

**The release thresholds are fitted to the smoothest data available and the
deployment environment is the roughest.** Do not assume they transfer.

- [ ] Before trusting any release threshold on a counterweight: set the arrival
  gates unreachable (e.g. 5.0) so nothing can release early, run slow and fast,
  and let every run hold to the failsafe. That yields counterweight cruise peak,
  edge rate and a real arrival signature at no risk of an early release. Then
  set the gates from that data, biased upward.

- [ ] Worth asking Biju whether automatic release should exist at all. A
  mechanic standing next to the device can silence it; the device deciding from
  a 3 Hz accelerometer that the counterweight has stopped is a judgement with no
  ground truth. Section 3 introduced the latch to stop the alarm cutting out
  mid-run -- "runs until silenced" solves that too, and cannot fail dangerously.

### 14.6 Stationary soak -- zero events in 17.6 minutes

The oldest open item in this document. Section 7 warned that ~6 minutes of
aggregate stationary data was "encouraging, not proof, for a unit that sits in
a hoistway for months", and the latched FSM sharpened it: a single false
departure while parked now holds the alarm until LATCH_FAILSAFE_MS.

Measured 2026-08-07, cartop, device parked and undisturbed:

| | Result |
|---|---|
| Duration | 17.6 min, 3314 samples, entirely in STATE_MONITORING |
| **False departures** | **0** |
| **Any-motion edges** | **0** -- none at all |
| Polled max delta | 0.103 against the 0.400 departure trigger |
| Polled p99 | 0.032 |
| Raw max | 0.367 |
| Sensor read errors | 0 |
| Battery | 2193 -> 2212, no drift |

**Zero any-motion interrupts across the whole window** is the figure that
matters. A departure latches on a single edge, so the entire exposure is how
often a stationary device raises one, and here the rate is zero rather than
small. The polled path had ~4x margin with a p99 of 0.032.

This retires section 7's caveat for the cartop case. It does **not** transfer
to the counterweight (14.5): zero edges on a cartop in a quiet shaft shows the
firmware does not invent departures on its own, not that a live building will
not supply real ones. 17.6 minutes is also short against a full-day
construction deployment.

Method note. The first pass at this analysis reported 13 false departures.
That was wrong -- the capture spans three reflashes, millis() restarts at each,
and the script mixed boot sections while anchoring FSM lines to samples from
the wrong one. It was counting the day's real runs. Any analysis of these
captures must restrict itself to a single boot section; parse_falcon_log.py has
the same limitation and does not detect reboots.

- [ ] Teach parse_falcon_log.py to split on "Device Booted" and analyse the
  last section by default, or refuse to run across a reboot.

### 14.7 A gentle 18 fpm arrival is smaller than the device's own noise

Later on 2026-08-07, an 18 fpm descent to a midpoint stop. Departure detected
normally; **arrival missed entirely**, and the alarm held until the failsafe.

| Event | Time after departure | Raw | Averaged |
|---|---|---|---|
| Departure | +1.3 s | 0.838 | 0.462 |
| **Arrival** | **+91.7 s** | **0.116** | **0.058** |

Between them, across 144 seconds, **not one raw sample exceeded 0.12** and the
mean averaged delta was 0.021. The run was that smooth.

The arrival is not merely below the thresholds -- it is below the noise the
device produces sitting still:

| | Averaged delta |
|---|---|
| This arrival | **0.058** |
| Parked noise ceiling, 17.6 min soak (14.6) | **0.103** |
| Corroboration gate | 0.20 |
| Polled arrival threshold | 0.30 |

**There is no gap to place a threshold in.** Anything low enough to catch 0.058
sits under a level a parked device crosses on its own, so it would release
constantly while stationary -- and that same noise floor is what departure
detection has to stay above.

### 14.8 Speed does not predict arrival strength; stop abruptness does

Arrival deltas measured on 2026-08-07:

| Speed | Arrival delta | Outcome |
|---|---|---|
| 18 fpm | **0.312** | detected (polled, 4% margin) |
| 18 fpm | **0.058** | **missed** |
| ~50 fpm | 0.278 | detected |
| 58 fpm | 0.383 | detected |
| 123 fpm up | 0.362 | detected |
| 123 fpm down | 0.362 | detected |

Two 18 fpm arrivals differ by **5x**, while the two 123 fpm arrivals agree to
three decimals. **The variance at one speed exceeds the variance across
speeds.** Slow running does not produce weak arrivals; it gives the machine room
to stop gently, and this one did.

That means a minimum-speed specification narrows the exposure without closing
it. A "30 fpm floor" was discussed: it would exclude the worst case measured,
but there is no 30 fpm arrival data, and the nearest point (~50 fpm at 0.278) is
itself below the polled threshold and only released because clustering caught
it.

### 14.9 Conclusion: stop tuning arrival detection

Sections 14.7 and 14.8 together say the target is not reliably there. This is a
physical limit of detecting a soft stop with this accelerometer, not a statistic
that more data or a better threshold recovers. Section 3 established that
constant velocity is unobservable; this establishes that a sufficiently gentle
arrival is too.

Every arrival that worked today was relatively abrupt (0.278 to 1.275). The
machine can stop softer than that.

Given the failure asymmetry in 14.4 -- where the safe direction is always "alarm
longer" -- the honest options are:

1. **Let it run to `LATCH_FAILSAFE_MS`.** Annoying, safe, and already the
   behaviour when detection fails.
2. **Have the mechanic silence it.** A button, or power-cycling. Deterministic,
   and it cannot fail dangerously.

Note what is NOT affected: **departure detection, the safety-critical half, has
no equivalent problem.** It caught -0.116 m/s^2 at 18 fpm and 0.838 later the
same day, and has worked at every speed and configuration since the threshold
was set to 32. The half that protects someone is the half that works.

- [ ] Decide with Biju whether automatic release should exist at all. This is
  now the top open question in the project -- it would delete an entire class of
  problem rather than tuning it.

- [ ] If automatic release is kept, document the low-speed behaviour honestly:
  below roughly 30 fpm, expect the alarm to run to the failsafe rather than
  self-clear.

- [ ] Does the device need to detect RELEVELING? That is slow automatic movement
  with a mechanic potentially in the hoistway -- arguably the purest form of the
  hazard this device exists for. If so, sub-18 fpm departure detection is in
  scope regardless of any speed floor, and deserves a deliberate test rather
  than inference from inspection-speed data.
