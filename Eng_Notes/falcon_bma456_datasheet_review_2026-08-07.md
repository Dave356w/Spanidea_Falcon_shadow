# BMA456 datasheet review — what the sensor can do that the firmware does not use

**Date:** 2026-08-07
**Sources:**
- `Datasheet/bst-bma456-ds000.pdf` — rev 3.6, March 2026, BST-BMA456-DS000-09, 103 pp
- Bosch **BST-BMA456-AN002-00** ("AN Feature Set", rev 0.1) — the datasheet defers
  every feature description to the application notes (p.30: *"For configuration
  and details, please check the available application notes"*), so the
  any-motion / no-motion algorithms are only defined there
- Bosch **`BMA456_SensorAPI`** (github.com/boschsensortec) — the current sensor
  API, compared against the 2018 Seeed copy vendored in `falcon_srcs/src/`
- Current firmware as of `ae9ab4b` on `Falcon_Rel_EFT`

**Purpose.** `falcon_state_of_project_2026-08-07.md` lists a set of open
problems and says of the largest one (§5.1a) that arrival detection *"should
stop being tuned"*. Before accepting that, this review asks a narrower question:
**how much of the sensor is the firmware actually using, and does the unused
part change any of those conclusions?**

Answer: the firmware uses one feature (any-motion) out of a feature set it pays
6 KB of flash for, and it does not read a single one of the sensor's four fault
registers. Two of the findings below change a stated conclusion. Most of the
rest are cheap.

Section references in the form §n point at `falcon_analysis_2026-08-06.md`
unless marked *(SoP)*, which means the state-of-project report.

---

## Summary

| # | Finding | Bears on |
|---|---|---|
| 1 | The vendored driver is an obsolete variant. Bosch's current API has a feature set with **independent** any-motion and no-motion, and a config blob **1/5 the size** — ~4.9 KB of flash back | roadmap 11, 5, §5.1a *(SoP)* |
| 2 | ✅ The any-motion threshold LSB is confirmed: **0.4883 mg/count**. `ANYMOTION_THRESHOLD 32` = **0.153 m/s²**. Duration ranges to **163 s**, not 100 ms | closes an open ⬜ in `main.cpp` |
| 3 | 🟠 Any-motion compares against a **latched reference sample**, not the previous sample. Read literally that permits a state where departure detection dies silently; the measured data says it does not happen, and nothing checks | §11, safety-critical |
| 4 | ✅ No-motion is confirmed unusable as an arrival detector — but it is safe, and better than what we have, for the *settled* confirmation | §13, `movement_service.h` |
| 5 | 🟠 The 0.103 parked "noise floor" is **14× the sensor's own noise**. Some of it is likely aliasing we create ourselves, and Bosch has the fix | §14.7, §14.9 — the top open question |
| 6 | The part can prove its own signal path works (**self-test**, 1800 mg minimum). The driver already implements it | roadmap 4a, §5.4 *(SoP)* |
| 7 | Four fault registers are never read. Any one of them would have caught §10.2 | §10.2, §5.4 *(SoP)* |
| 8 | Hardware offset compensation is NVM-backed and would delete the stale-baseline failure — but it is a factory operation, 15 write cycles | §3.1 *(SoP)* |
| 9 | Sensortime gives exact, reboot-proof timestamps for free | §6a, §14.6 method note |

---

## 1. 🔴 The driver is an obsolete variant, and it costs 4.9 KB of flash

`falcon_srcs/src/bma456.c` is the Seeed Grove Step-Counter library from 2018.
Its feature configuration blob is **6,144 bytes** of `PROGMEM`, and
`BMA456_FEATURE_SIZE` is 64. That blob is the wearable feature set: step
counter, step detector, activity recognition, wrist tilt, wake-up, tap.

**The firmware uses none of them.** `stepCounterEnable()`, `getStepCounterOutput()`
and `getTemperature()` are defined in `arduino_bma456.cpp` and called from
nowhere in `main.cpp` or `movement_service.cpp`. The only feature in use is
any-motion.

Bosch's current API splits the part into variants. `bma456_an.c` — the feature
set documented by AN002 — supports **any-motion and no-motion, and nothing
else**:

| | vendored `bma456.c` (2018) | Bosch `bma456_an.c` (current) |
|---|---|---|
| Config blob | **6,144 B** | **1,200 B** |
| `FEATURE_SIZE` | 64 | 12 |
| any-motion / no-motion | one shared config block, one shared enable | `bma456_an_set_any_mot_config()` and `bma456_an_set_no_mot_config()`, **independent** |
| Interrupt mapping | one line: `BMA456_ANY_NO_MOTION_INT` | `BMA456_AN_ANY_MOT_INT` (0x20), `BMA456_AN_NO_MOT_INT` (0x40), `BMA456_AN_ERROR_INT` (0x80) — mappable separately |
| Status bits | one | `INT_STATUS_0.any_motion_out`, `.no_motion_out`, `.error_int_out` (AN002 §3.1.22) |

Two consequences.

**Flash.** 6,144 − 1,200 = **4,944 bytes**. The unit is at 25,084 / 32,256
(77.8%), so ~7.2 KB free becomes ~12.1 KB. Roadmap item 11 — the on-device
black box, which §5.1 *(SoP)* calls a precondition for any counterweight
deployment — currently has to fit in that 7.2 KB alongside everything else.
This is the cheapest 4.9 KB in the project.

**The mutual-exclusion constraint is not real any more.**
`arduino_bma456.h` documents it as a property of the part:

> `nomotion_sel` … They are MUTUALLY EXCLUSIVE in this driver variant — one
> feature and one shared interrupt (`BMA456_ANY_NO_MOTION_INT`), not two
> independent ones.

That is accurate about *this driver* and wrong about *the sensor*. It is a
limitation of a seven-year-old third-party copy. Bosch's current wearable
variant (`bma456w.c`, 5,016 B blob) also has them separate — this is not
specific to the AN variant. Both engines can run at once, on separate
interrupt lines, into separate status bits. §11 confirmed both `INT1_ACC`→PD2
and `INT2_ACC`→PD3 reach true external interrupt pins, and PD3 is unused today.

Cost of the swap: the API changed shape between the two versions (`int8_t`
returns instead of `uint16_t`, renamed structs), so `arduino_bma456.cpp` needs
rewriting against the new headers. It is a contained change — that file is the
only thing that touches the Bosch API.

- [ ] Port `arduino_bma456.cpp` to Bosch `BMA456_SensorAPI` `bma456_an`
- [ ] Confirm the 4,944-byte saving against a real build before relying on it
- [ ] Re-run the bench any-motion check afterwards: a different feature blob is
      different ASIC firmware, and the threshold behaviour needs re-confirming,
      not assuming

---

## 2. ✅ The threshold arithmetic, confirmed

`main.cpp` carries this caveat:

> The first value, 96, was derived from an assumed ~0.488 mg per count … That
> LSB has still **NOT** been confirmed against `Datasheet/`, so everything below
> is expressed as a RATIO to the threshold actually used.

It is confirmed, from two independent places. The driver header
(`bma456.h:687`) and AN002 §3.1.49 both specify the threshold as **5.11g
format** — 5 integer bits, 11 fractional — with a default of `0xAA = 83 mg`:

```
170 counts × 2⁻¹¹ g = 170 × 0.48828 mg = 83.0 mg      ✓ exact
```

So **1 count = 0.4883 mg = 0.004789 m/s²**, and:

| | counts | mg | m/s² |
|---|---|---|---|
| `ANYMOTION_THRESHOLD` (current) | 32 | 15.6 | **0.153** |
| Bosch default | 170 | 83.0 | 0.814 |
| Field maximum (11 bits) | 2047 | 999.5 | 9.80 |

The estimate the thresholds were set from was right, and every ratio in the
`ANYMOTION_*` block in `main.cpp` stands. Two cross-checks against measured
data agree: the 17.6-minute soak (§14.6) recorded zero edges with a polled p99
of 0.032 m/s², which is 0.21× threshold; and cruise vibration at ±0.09 m/s² is
0.59× — matching the table in `main.cpp` that predicted 0.58×.

**The duration field is the more useful discovery.** AN002 §3.1.49 gives it 13
bits, in 50 Hz samples:

```
range 0 … 8191 samples  ×  20 ms  =  0 … 163.8 s
```

`ANYMOTION_DURATION` is 5 (100 ms). `main.cpp` already argues that if false
fires appear the right response is *"raise `ANYMOTION_DURATION` rather than the
threshold. Vibration spikes are brief; a departure ramp lasts 1–2 s"* — that
advice now has three orders of magnitude of headroom behind it, and §12.2's
finding that **duration, not amplitude, is what discriminates** is exactly the
knob this field turns. For the counterweight, where §5.1 *(SoP)* expects a
rougher ride to push cruise vibration up toward the threshold, raising duration
to 50–100 samples (1–2 s) is the adjustment to reach for first.

- [ ] Update the `ANYMOTION_*` comment block in `main.cpp`: drop the "not
      confirmed" caveat, record 0.4883 mg/count, and state the duration range

---

## 3. 🟠 Any-motion uses a latched reference, and we cannot currently tell when it is stale

AN002 p.7, any-motion:

> Any-motion detection uses the slope between current input and **reference**
> acceleration samples … provides an interrupt when the absolute value of the
> slope exceeds the configurable threshold for consecutive duration samples …
> **Reference acceleration sample is updated only when an any-motion interrupt
> is triggered.** The interrupt status is reset as soon as the slope falls below
> the set threshold value.

So the test is `|Acc − Ref| > threshold`, where `Ref` is frozen until the next
trigger. It is *not* a consecutive-sample difference (that is no-motion — see
§4 below). Nothing in §11 or §12 assumed this; the detector was characterised
empirically, and this is the first time its actual rule has been written down
here.

Read literally, that rule has an uncomfortable consequence. If the reference is
captured during a departure transient, it is captured at a displaced value —
the 18 fpm departure that §12 detected was 0.838 m/s² raw, which is **175
counts**, 5.5× the threshold of 32. When the car reaches cruise and Z returns to
a static 1 g, `|Acc − Ref|` would still be ~175 counts, the condition would
never clear, and the output would stay asserted for the rest of the run. The
firmware attaches `acc_int1_isr` as `RISING`, so a pin that goes high and stays
high yields exactly one edge and then silence — which would take the arrival
cluster path (§14.1) with it.

**The measured data says this does not happen.** §13.1 recorded cruise held for
80+ seconds with zero false releases, and §14 five consecutive runs with *"zero
any-motion edges during cruise"* — and the arrival cluster fired afterwards,
which it could not have done from a pin stuck high. So the sensor is
re-referencing more often than "only when an interrupt is triggered" implies.
AN002 rev 0.1 is a thin document; this is most likely an incomplete description
rather than a bug we have been lucky with.

That leaves the finding as an **unverified assumption rather than a known
failure**, and worth closing because of where it sits: a stuck reference
disables departure detection, which §14.9 calls *"the half that protects
someone"*, and it does so silently — nothing in the log distinguishes "quiet
because nothing moved" from "deaf since boot".

Two things narrow the exposure, and both are worth stating so this is not
over-weighted:

- **A gravity-aligned axis is first-order insensitive to tilt.** Z reads ~1 g
  at rest (the state-of-project report records a stationary flat at
  9.69–9.77 m/s²), so a 31 mg DC shift needs `cos θ = 0.969`, i.e. **14.3°** of
  attitude change — not the ~1.8° it would be for a horizontal axis. Ordinary
  re-clamping will not do it.
- **Offset drift will not do it either.** Zero-g offset is ±20 mg over
  lifetime and TCO is ±0.35 mg/K, so temperature alone would need a 90 K swing.

We have also already seen the *symptom* once, from a different cause: §11's
first bench run in latched mode, *"n=1 at t=61521 and im=1 for the rest of the
session"*. Non-latch mode was adopted so the sensor would clear the pin itself
— but non-latch clears when *the condition* passes, so it is not protection
against a stale reference, only against the latch.

The defence is nearly free, because the polling already exists.
`poll_acc_int_status()` runs every `ACC_INT_POLL_MS` (1000 ms) and already
reads both the status word and `digitalRead(PIN_ACC_INT1)`, logging when either
is live. Today that is observation only. It should act:

> If `any_motion_out` reads asserted on N consecutive polls while the FSM is in
> `STATE_MONITORING` — i.e. asserted continuously with no departure having been
> latched — the reference is stuck. Re-arm the feature (disable/enable, which
> forces a fresh reference) and log it as a fault.

Cost: one counter and one comparison, on I²C traffic already being spent.

Note this also explains something the log format cannot currently distinguish:
`ACC-STAT s=0x… pin=1` means "asserted right now", and the existing line does
not separate "asserted because the car is moving" from "asserted since boot and
never coming back". Adding the consecutive count to that line makes the two
readable apart in a capture.

- [ ] Add stuck-reference detection and re-arm to `poll_acc_int_status()`
- [ ] Add the consecutive-assert count to the `ACC-STAT` log line
- [ ] Bench test to settle the mechanism, since AN002 does not: trigger
      any-motion, then tilt the unit ~20° and hold it. If the pin stays high the
      reference is genuinely sticky and the re-arm is load-bearing; if it clears,
      the reference updates continuously and this is diagnostics only

---

## 4. ✅ No-motion cannot detect arrival — confirmed at the algorithm level

`movement_service.h` carries a ⛔ block forbidding a stillness backstop, ending:

> The same argument rules out the sensor's no-motion feature.

That was reasoned from the physics in §3 and from two runs where cruise was
quieter than rest. AN002 p.8 confirms it from the algorithm definition, which is
stronger:

> No-motion detection uses the slope between **two consecutive acceleration
> signal samples** … triggered when the slope on all enabled sensing axis
> remains smaller than the threshold for the duration configured.

Consecutive-sample slope at constant velocity is identical to consecutive-sample
slope at rest — in both cases the signal is a static 1 g. No threshold and no
duration separates them, and unlike the polled path there is not even a
question of averaging window or sample rate. The block in `movement_service.h`
is correct and should stay. **Do not wire no-motion to arrival.**

**But there is one place it is safe, and it is an improvement.**

`STATE_DECELERATING` requires `STOP_CONFIRM_MS` (5 s) of continuous quiet inside
`STOP_BAND_VALUE` (0.10 m/s²) before silencing. That test runs on the 3.13 Hz
rolling average — and 0.10 m/s² is almost exactly the level §14.7 shows a
*parked* device crossing (0.103 ceiling over 17.6 minutes). The confirm window
is being asked to distinguish "settled" using the one statistic we have measured
to be unreliable at that magnitude, which is why it needed raising from 2 s to
5 s after the 73-second false alarm.

No-motion does the identical test on 50 Hz data:

```
threshold  ≈ 0.10 m/s² → 21 counts
duration   = 5 s       → 250 samples
axes       = Z only, matching the polled path
```

Gated behind `arrival_seen`, it **cannot fire mid-travel**, because it is only
consulted after an arrival transient has already been detected by any-motion or
the polled path. It cannot release the alarm on its own. It replaces a weak
3 Hz test with a strong 50 Hz one at the single point in the FSM where a weak
test has already caused a real failure.

This needs the independent engines from finding 1 — with the current driver,
arming no-motion disarms any-motion, which would drop the arrival-cluster path
at exactly the wrong moment.

- [ ] After finding 1 lands: replace the `STOP_CONFIRM_MS` polled window with
      no-motion on INT2/PD3, gated behind `arrival_seen`
- [ ] Keep the polled window as the fallback, the same way §14.1 keeps both
      arrival paths

---

## 5. 🟠 The parked noise floor is 14× the sensor's noise — some of it is ours

§14.7 is the most consequential measurement in the project: a real 18 fpm
arrival at **0.058** against a parked ceiling of **0.103**, concluding *"there
is no gap to place a threshold in"*, and §14.9 concludes from it that arrival
detection should stop being tuned.

That conclusion rests on the 0.103 being a floor. The datasheet says most of it
is not the sensor.

Spec (p.9): output noise density **120 µg/√Hz**. The firmware runs
`RANGE_2G, ODR_100_HZ, NORMAL_AVG4, CONTINUOUS`, and Table 12 gives the 3 dB
bandwidth for ODR 100 Hz normal-filter as **40.5 Hz**:

```
120 µg/√Hz × √40.5 Hz = 764 µg = 0.0075 m/s²   RMS sensor noise
```

Against that:

| | m/s² | × sensor noise |
|---|---|---|
| Sensor noise (spec) | 0.0075 | 1× |
| Soak p99, averaged (§14.6) | 0.032 | 4× |
| **Soak ceiling, averaged** (§14.6) | **0.103** | **14×** |
| Soak raw max (§14.6) | 0.367 | 50× |

So the floor that blocks arrival detection is **not** the sensor's noise. It is
either real hoistway vibration — in which case §14.9 stands unchanged — or it
is aliasing, which we are creating ourselves and can remove.

**The aliasing case is concrete.** The firmware reads `DATA_8..13` at 3.13 Hz
(§2) from a signal band-limited to 40.5 Hz. Nyquist for 3.13 Hz sampling is
1.56 Hz. Everything between 1.56 Hz and 40.5 Hz — building hum, machine-room
vibration, rope resonance, the mechanic moving on the platform — folds into the
baseband and appears as slow, random wander with no way to tell it from a real
arrival transient. `RollingAvg(4)` runs *after* the sampling and cannot undo it;
it just smooths the aliased result. This is a different defect from the one §2
identified: §2 says the window is too long, this says the samples that go into
it are contaminated before averaging starts.

If that is where the 0.103 comes from, then a properly band-limited 3 Hz signal
has a much lower parked ceiling, and 0.058 may sit above it. **That reopens the
question §14.9 closes.** It does not settle it — the vibration may be genuinely
there — but the experiment has never been run, and the conclusion currently
rests on an untested assumption.

### The cheap test, which needs no code

AN002 p.5:

> If performance mode is enabled (`ACC_CONF.acc_perf_mode` is 0b1, device is in
> continuous mode), then the features are functioning properly, **regardless to
> the ODR and the Bandwidth** that the Host would set.

The firmware already uses `CONTINUOUS`. So the ODR can be dropped without
disarming any-motion — the minimum-50 Hz restriction applies only when
performance mode is off. Change one enum:

```c
bma456.initialize(RANGE_2G, ODR_12_5_HZ, NORMAL_AVG4, CONTINUOUS);
```

ODR 12.5 Hz gives a 3 dB bandwidth of **5.06 Hz** (Table 12) against a 3.13 Hz
sample rate — close to properly band-limited. Re-run the 17.6-minute soak and
compare the ceiling. If it drops toward 0.03, the floor was aliasing and there
is a gap after all. If it stays near 0.103, it is real vibration, §14.9 stands,
and the question is settled with evidence instead of assumption.

Verify the claim rather than trusting it: `INTERNAL_STATUS.odr_50hz_error`
(0x2A bit 6) is the sensor's own answer, and a hand-wave should still produce
`ACC-INT` lines. If either says otherwise, back the ODR out to 50 Hz (20.25 Hz
bandwidth) — still a 2× improvement over 40.5 Hz.

### The real fix, if the test says aliasing

The FIFO. 1024 bytes, `FIFO_CONFIG_1.fifo_acc_en`, headerless mode = 6 bytes per
frame = **170 frames = 1.7 s** of buffer at 100 Hz. The host reads a burst and
averages *every* sample — genuine decimation with a 32-sample boxcar whose first
null lands on 3.125 Hz, instead of aliased sub-sampling. `bma4_read_fifo_data()`
and `bma4_extract_accel()` are already in the vendored `bma4.c`, and the FIFO
registers are present in the AN feature set (AN002 §3.1.25–3.1.36), so the
variant swap in finding 1 does not cost them.

Two caveats, both real:

- **It does not fit at F_CPU = 1 MHz.** §10.4 measured a 6-byte read at
  6,144 µs — CPU-bound, not bus-bound. `Wire`'s buffer caps a transfer at 32
  bytes and `accel.read_write_len` is 8, so 600 B/s of FIFO data means many
  chunked transactions. **FIFO is gated on the same 8 MHz clock change roadmap
  item 5 needs**, and should be sequenced with it rather than attempted first.
- **Do not use `FIFO_DOWNS` to make it cheaper.** Downsampling in the sensor
  decimates without averaging, which re-introduces exactly the aliasing the FIFO
  is there to remove.

There is also a middle option: low-power mode with hardware averaging
(`acc_perf_mode = 0`, `acc_bwp = res_avg16/32`) makes the sensor average N
samples taken at 1600 Hz per output. But in that mode the 50 Hz feature
restriction *does* apply, so ODR cannot go below 50 Hz, and at ODR 50 Hz with
avg32 the current draw goes to 152 µA (p.18). Less attractive than the FIFO.

- [ ] Run the ODR 12.5 Hz soak. This is one enum and 20 minutes, and it decides
      whether §14.9 is right
- [ ] Read `INTERNAL_STATUS` after the change and confirm any-motion still fires
- [ ] Only if the floor drops: schedule the FIFO work with roadmap item 5

---

## 6. Self-test: the unit can prove its signal path works

§5.4 *(SoP)* lists `a=ERR` as never having fired on hardware, and roadmap 4a as
unresolved: the unit no longer lies about a dead sensor, but raises no
user-visible fault. The sensor has a positive liveness check for exactly this.

Datasheet p.37: electrostatic forces deflect the seismic mass, so **the entire
signal path** is exercised, not just the bus. Run it in both directions and
difference the results; pass requires **≥ 1800 mg** on each axis.

`bma4_perform_accel_selftest()` (`bma4.c:2622`) already implements the whole
sequence — config, +deflection, read, −deflection, read, validate, soft reset —
and costs ~420 ms of the delays it contains. It is dead code in the current
build.

Ordering matters: it ends with a soft reset (`bma4.c:2664`), which wipes the
feature configuration. Run it **before** `bma456_write_config_file()` and before
arming any-motion, i.e. immediately after `bma456_init()`.

This gives the unit something to say. A boot self-test that fails is a
displayable fault at the one moment the mechanic is holding the device and can
act on it — which is the answer to 4a's product question that does not require
detecting the failure mid-job.

It also gives §5.4's untested path a way to be exercised: a real sensor is hard
to kill on the bench, but the self-test can be forced to fail by inverting the
comparison, which at least proves the fault plumbing works end to end.

- [ ] Call `bma4_perform_accel_selftest()` at boot, before the config file write
- [ ] Decide the failure behaviour with Biju (roadmap 4a) — refuse to arm? beep
      a distinct pattern? Only that this is now answerable at boot is new here

---

## 7. Four fault registers, never read

Every one of these is a single-byte read on I²C traffic already being spent
once per second in `poll_acc_int_status()`.

| Register | Bit | What it says | Why it matters here |
|---|---|---|---|
| `EVENT` (0x1B) | `por_detected`, clear-on-read | '1' after power-up or soft reset | **The important one.** If the sensor browns out mid-job — plausible given §5's buzzer coupling and the battery load — it restarts with the feature config gone: any-motion silently disarmed, `DATA` registers still returning plausible values. Nothing today would notice. This is §10.2's failure with a different cause and no detection |
| `ERR_REG` (0x02) | `fatal_err` | chip not operational; clears only on POR/softreset | The unambiguous "this part is dead" flag |
| | `error_code = 0x01` | illegal `ACC_CONF` | p.19: *"Illegal settings … will result in an error code … The content of the data register is undefined"* — a config typo produces undefined data, not an error |
| `INTERNAL_STATUS` (0x2A) | `message` | `init_ok` / `init_err` / `drv_err` / `sns_stop` | Read once after init. This is the direct check for `movement_service.h`'s "config that did not take", which is currently an unverifiable worry |
| | `odr_50hz_error` | ODR too low for the features | The arbiter for finding 5's ODR change |
| `INT_STATUS_0` | `error_int_out` | sensor stopped on fatal error | Only with the AN feature set (finding 1). AN002: *"the device re-initialization must be done for proper functioning"* — so it is also actionable, not just diagnostic |

The theme of §10.2 was that the device could not tell when it was broken. That
was fixed for the transport and for the all-zeros case. These registers close
the remaining gap: the cases where the bus works, the data looks fine, and the
part has quietly stopped doing what it was configured to do.

- [ ] Read `EVENT.por_detected` in `poll_acc_int_status()`; on a set bit, log it
      and re-run the full sensor configuration
- [ ] Read `ERR_REG` and `INTERNAL_STATUS` once after init; refuse to arm on
      `init_err` / `drv_err` / `fatal_err`
- [ ] Add both to the `ACC-STAT` log line so captures show them

---

## 8. Hardware offset compensation — a manufacturing option, not a firmware one

Datasheet p.38. `OFFSET_0..2` (0x71–0x73), 8-bit two's complement, **LSB 3.9 mg,
range ±0.5 g**, independent of the range setting, enabled by
`NV_CONF.acc_off_en`. NVM-backed and reloaded into the image registers on every
reset. `bma4_set_offset_comp()` is already in the driver.

The attraction: the firmware's `zero_calib_value` is software, derived at boot
from a self-calibration that §2a showed was running on a single sample, and
§3.1 *(SoP)* records the resulting failure — *"produced a false alarm on a
stationary bench and then latched a stale baseline it could never clear"*. A
hardware offset written once at manufacture gives a Z axis that reads near-zero
deviation from the first sample of every boot, with no calibration window to get
wrong.

Two reasons this is a bench/factory operation and not a firmware change:

- **NVM is rated 15 write cycles** (p.8). This is written once, not per boot.
- It compensates a DC offset, not orientation. If the device can be mounted at
  arbitrary attitudes, a fixed offset is wrong for most of them. Worth knowing
  how tightly the mounting actually constrains attitude before pursuing it —
  which is also the question finding 3 turns on.

Listing it for completeness rather than recommending it.

---

## 9. Sensortime — free timestamps that survive a reflash

Datasheet p.21: a free-running 24-bit counter at **39.0625 µs** resolution,
synchronous with every data-register and FIFO update, readable as a burst that
is guaranteed self-consistent. It wraps every 655 s (10.9 min).

Two places this helps:

- §6a spent real effort on `ov=` and `tk=` trying to detect decimation, and the
  cause is still recorded as unproven. Sensortime is the sensor's own answer to
  "when was this sample actually taken", independent of anything the AVR
  believes.
- §14.6's method note records that the first pass at the soak analysis was
  wrong because *"the capture spans three reflashes, `millis()` restarts at
  each, and the script mixed boot sections"*, and leaves an open item to teach
  `parse_falcon_log.py` to split on reboots. Sensortime does not restart on a
  reflash in the way `millis()` does, so a sensortime column makes reboots
  visible in the data itself rather than something the parser has to infer.

Cost is 3 bytes per read, which is not free at 6,144 µs per transaction — but
it only needs to be read once per second, alongside the existing status poll,
not per sample.

---

## 10. What this does and does not change

**Changes a conclusion:**

- §14.9 ("stop tuning arrival detection") rests on a parked noise floor that is
  14× the sensor's own noise. Finding 5 gives a one-enum, 20-minute experiment
  that decides whether that floor is real or self-inflicted. The conclusion may
  well survive it — but it should survive it, not precede it.
- `movement_service.h`'s statement that any-motion and no-motion are mutually
  exclusive is true of the vendored driver and false of the sensor. Finding 4's
  settled-confirmation improvement depends on that.

**Adds a question that was not on the list:**

- Finding 3. Any-motion's reference is latched, and read literally the app note
  permits a state in which departure detection is permanently deaf with nothing
  in the log to say so. The measured data says we are not in that state, so this
  is an unverified assumption rather than a known defect — but it sits under the
  half §14.9 calls *"the half that protects someone"*, and it is the only item
  here that is safety-relevant rather than an improvement.

**Does not change:**

- §3's core physical argument. Constant velocity is unobservable with an
  accelerometer, and no Bosch feature changes that. No-motion is confirmed to
  fail in exactly the way §3 predicts.
- The recommendation in §6.1 *(SoP)* to ask Biju whether automatic release
  should exist at all. Findings 4 and 5 make the automatic path better; they do
  not make it *deterministic*, and the failure asymmetry in §14.4 still favours
  a mechanic-operated silence.

**Suggested sequence**, cheapest and most decisive first:

1. The ODR 12.5 Hz soak (finding 5). One enum. Decides the top open question.
2. Fault-register reads (finding 7) and boot self-test (finding 6). Small,
   independent, and they close §5.4.
3. Stuck-reference detection (finding 3). Small, and it is the safety item.
4. The driver variant swap (finding 1). Larger, and it unblocks findings 4 and
   the flash budget for roadmap item 11.
5. FIFO (finding 5), sequenced with roadmap item 5 and the 8 MHz clock.

---

## 11a. Implemented: the windowed velocity integral (`velocity.h` / `velocity.cpp`)

Finding 5 said the parked floor is probably aliasing rather than physics. The
follow-on question was whether a different statistic could exploit that, and the
answer is yes in principle and not yet in practice. The module is in the tree,
wired into the FSM, and **not armed** (`VEL_ARMED 0`).

**The statistic.** `w(t)` = integral of `(a − baseline)` over a trailing 5 s
window. Velocity change, not acceleration amplitude. A departure to 20 fpm is
0.1016 m/s however gently it ramps, so the softness that defeats amplitude
thresholding (§14.7) does not shrink it; and by conservation the arrival must
cancel the departure exactly, which gives an arrival gate scaled from *this
run's* measurement instead of `ARRIVAL_CLUSTER_DELTA`'s fitted constant (§14.3).

**Why a window and not an integral.** Free-running integration was tried first
against the captures and produced **−4.44 m/s** on the 171 s descent. A
0.026 m/s² baseline error times 171 s *is* 4.4 m/s, and the datasheet puts
zero-g offset at ±20 mg. Open-loop integration of this sensor is not
recoverable by tuning. The window bounds drift to `eps × 5 s`.

**Why it is not armed.** Replayed through the §14.6 soak — the only stretch in
these captures where "parked" is ground truth:

| | |
|---|---|
| Parked stretch | 1241 s, 3885 samples |
| \|w\| median / p99 | 0.0176 / 0.0757 m/s |
| **\|w\| peak** | **0.1274 m/s** |
| Implied σ | 0.0166 m/s — **peak is 7.7σ** |

A Gaussian does not produce 7.7σ once in 3854 looks, so the parked distribution
is **heavy-tailed** and σ does not describe it. A 5σ gate (0.115 m/s) was
crossed **twice in 20.7 minutes** — roughly 46 false departures per shift, each
latching the alarm to the failsafe. A gate safe against the measured peak needs
~0.255 m/s = **50 fpm**, which is above the requirement and inside what
any-motion already catches. At 3.13 Hz no threshold is both safe and useful.

This is the §11 pattern applied again: wire it in, log what it would have done,
characterise it in a hoistway, then arm it. That sequence is why the any-motion
path works.

**What unblocks it.** Sample rate. `graph/velocity_replay.py scale` shows the
parked *peak* scaling as √2 under 2× decimation while σ does not move — the
excursions that set the gate are white even though the bulk is not. Note the
evidence is one octave, not four: the `VEL_MAX_DT_MS` clamp means a 5 s window
cannot reach minimum coverage below ~1.5 Hz, so lower decimations produce no
valid samples. Treat the 100 Hz projection as a hypothesis with one octave
behind it.

**Guards implemented** (each maps to a failure this project has already had):

| Guard | Against |
|---|---|
| Baseline frozen outside `STATE_MONITORING` and above `VEL_BASELINE_FREEZE_MPS` | the tracker absorbing a slow departure — the case where it would guarantee the miss |
| Seeded from the first good sample, `VEL_PRIME_SAMPLES` before anything is reported | §3.1's unprimed-window false alarm and stale latched baseline |
| `dt` clamped to `VEL_MAX_DT_MS` | buzzer coupling (§5, §11). Under-counts an arrival measured through a beep — the safe direction (§14.4) |
| Only good reads fed in; skipped bins contribute zero, never an extrapolation | §10.2's dead sensor reading as a valid 0.0 |
| `coverage_ms()` logged as `cv=` on every sample line | a window built on almost no data looking identical to a full one |
| Fed from the published snapshot in `loop()`, never the ISR | float tearing across the ISR boundary; same reasoning as §6 |
| Window reset on entry to `STATE_MOVING` and `STATE_MONITORING` | the departure transient still sitting in the window when arrival arms, and the arrival re-latching as a fresh departure |

**Cost:** flash 25,084 → 27,472 (77.8% → 85.2%), RAM 1,007 → 1,096 (49.2% →
53.5%). The flash number is why finding 1 matters — the driver variant swap
frees 4,944 bytes, more than this costs.

Also armed in the same change: finding 3's stuck-reference detection and re-arm
in `poll_acc_int_status()`. That one is live rather than observation-only, since
it can only re-arm a detector that has stopped responding and cannot itself
raise an alarm.

- [ ] Hoistway session with `VEL_ARMED 0`: capture `w=` / `cv=` and the `VEL:`
      lines across the speed range, including a genuinely parked hour
- [ ] Set `VEL_PARKED_PEAK` from `velocity_replay.py soak` on that capture
- [ ] Only then consider `VEL_ARMED 1`, and only after the sample rate is fixed

## 11. Additions to the questions for Biju

Alongside the five in §7 *(SoP)*:

- **Is the mounting attitude constrained?** Finding 3 fails at 1.8° of tilt
  relative to where the reference was last set, and finding 8 is only viable if
  the answer is yes.
- **Was the Seeed driver a deliberate choice?** It is a 2018 third-party copy
  with a feature set the product does not use. Knowing whether anything depends
  on it would de-risk finding 1.
- **Is there a bench fixture that can hold the unit at a controlled tilt?**
  Finding 3's test needs a few degrees, repeatably.
