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

Everything else in this section stands. The fix is still to restore
`buzzer_on = false` on the beep-off phase (roadmap item 7), and the warning
against low-pass filtering the ~2 Hz beep envelope still applies.

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

### ✅ 6a. `ov=` did not detect the decimation it exists to detect — symptom resolved

**Update, same day, after the reliability fixes.** The gaps are gone. `tk=` (a
free-running count of read attempts, added to tell the two candidate causes
apart) advances **1:1 with printed lines across 95 consecutive ticks with no
holes**, and timestamp deltas hold a steady 311/328 ms throughout. `ov=5` is now
legitimate: five overruns during init and none since, because `loop()` keeps up.

**Which change fixed it is not established.** The leading candidate is moving
eleven `Serial.print` string literals to flash with `F()`, which took RAM from
1676 to 1515 bytes — `RollingAvg` heap-allocates, so at 81.8% the heap and stack
were 372 bytes apart, and a stack collision would explain erratic ISR behaviour
that no counter could have diagnosed. But the return-code change touched the same
function, and the two were not tested in isolation. Reverting only the `F()`
wraps would settle it, at the cost of the RAM headroom.

Treat the original diagnosis below as unresolved-but-latent rather than fixed. If
the gaps return, `tk=` is now in place to distinguish the two causes on sight.

The historical description follows.

### 🔴 6a (original) — `ov=` does not detect the decimation it exists to detect

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
| 4a | Decide what the unit should *do* when the sensor is dead | ⬜ | It no longer lies, but raises no user-visible fault. Product decision — ask Biju |
| 5 | **Fix Timer1: CTC mode (`WGM12`), prescaler 64 → 100 Hz** | ⬜ | **the** detection fix; everything below assumes it |
| 6 | Delete the dead pressure path | ⬜ | §4 |
| 7 | Restore `buzzer_on = false` on beep-off phase | ⬜ | §5 |
| 8 | Departure-latch / decel-release FSM, no timeout | ⬜ | §3 |
| 9 | Sustain-gated threshold, recomputed at 100 Hz | ⬜ | §7 |
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
- ⬜ Does `Falcon_Rel_EFT_FreeRTOS` or `Anymotion` supersede any of this?
  `Anymotion` may be a BMA456 hardware-interrupt experiment, which would be a
  genuine alternative to §5.
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

4736 µs to the microsecond in both failure cases — the transaction aborting at
the same point every time because the sensor never acknowledges. A working read
costs ~6144 µs. `rd=` is a usable health signal, but the correct fix is checking
the return code and raising a fault (roadmap item 3).

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
