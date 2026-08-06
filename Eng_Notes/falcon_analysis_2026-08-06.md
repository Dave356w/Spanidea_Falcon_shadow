# Falcon firmware — analysis and roadmap

**Date:** 2026-08-06
**Baseline:** `Falcon_Rel_EFT` @ `59e945f` ("Added a workaround")
**Evidence:** the eight PuTTY captures on `eft-results-2026-07-15`, plus source reading
**Status of each item is marked** ✅ fixed · 🔧 in progress · ⬜ not started

This document exists so the reasoning survives independently of any one work
session. Every claim below is either derived from the source or measured from
the July captures; anything unverified is marked **ASSUMPTION** with the check
needed to confirm it.

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

⬜ **To confirm definitively:** `avrdude -c stk500 -P COM4 -p m328pb -U lfuse:r:-:h`
→ `0x62` confirms. `0xFF` means external crystal and much of the arithmetic
below needs redoing.

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

---

## 6. ✅ Serial printing from the ISR (fixed in PR #1)

`Serial.print()` was called from inside `ISR(TIMER1_COMPA_vect)` via
`read_acceleration_mss()`. The ISR re-enables interrupts, so those writes raced
`HardwareSerial`'s TX ring buffer drain. **14 corrupted lines across 8 runs**,
every one inside a state-transition burst (`STA E_MONITORING`,
`STATE_MONIT RING`).

Fixed: the ISR publishes a snapshot, `loop()` prints. New format adds `millis()`
timestamps, I²C read duration (`rd=`) and an ISR overrun counter (`ov=`).

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

| # | Change | Status | Notes |
|---|---|---|---|
| 1 | Move `Serial.print` out of the ISR; add timestamps | ✅ PR #1 | |
| 2 | Fix `TWI_FREQ`; instrument I²C read time | ✅ PR #1 | sensor was never affected — it's on TWI1 |
| 3 | **Fix Timer1: CTC mode (`WGM12`), prescaler 64 → 100 Hz** | ⬜ | **the** fix; everything else assumes it |
| 4 | Delete the dead pressure path | ⬜ | §4 |
| 5 | Restore `buzzer_on = false` on beep-off phase | ⬜ | §5 |
| 6 | Departure-latch / decel-release FSM, no timeout | ⬜ | §3 |
| 7 | Sustain-gated threshold, recomputed at 100 Hz | ⬜ | §7 |
| 8 | Cleanups: `#if 0` block, `movement_service.cpp.original`, `current_time` member, missing EOF newline, untrack `.pio/` + `compile_commands.json` | ⬜ | |

**Sequencing note:** re-run the EFT after 3–5 and before 6–7. At 100 Hz with a
40 ms average window the detector may behave so differently that the tuning
question changes completely. Every number in the July report describes a system
running 32× slower than designed.

### Open questions

- ⬜ Confirm `lfuse = 0x62` (1 MHz) on the bench.
- ⬜ Measure `rd=` — expect ~1400 µs. Near 40,000 means the sensor is not on
  TWI1 as read here.
- ⬜ `TWBR1 = 0` violates the datasheet's "TWBR ≥ 10 in Master mode". Left
  unchanged deliberately; measure before touching.
- ⬜ Ask Biju **why 0.40** specifically, and whether `59e945f` has had any
  hoistway time yet.
- ⬜ Does `Falcon_Rel_EFT_FreeRTOS` or `Anymotion` supersede any of this?
  `Anymotion` may be a BMA456 hardware-interrupt experiment, which would be a
  genuine alternative to §5.
- ⬜ At 1 MHz, is there CPU headroom for a 100 Hz ISR doing an I²C read plus
  float math? If not, raising F_CPU to 8 MHz is the lever — at the cost of
  battery life.

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
