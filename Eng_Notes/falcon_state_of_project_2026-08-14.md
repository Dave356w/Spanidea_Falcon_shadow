# Falcon firmware — state of the project

**Date:** 2026-08-14
**Head:** `Falcon_Rel_EFT` on `Dave356w/Spanidea_Falcon_shadow`
**Firmware on the device:** `31464` — Flash 31464/32256 (792 free), RAM 1357/2048
**Fuses:** `lfuse 0x62` · `hfuse 0xD9` · `efuse 0xF5` (BOD 2.7 V) · `lock 0xFF`
**Supersedes:** `falcon_state_of_project_2026-08-13.md`

**Evidence base — unchanged on the hoistway side.** ~50 instrumented runs across
four configurations, 17–500 fpm, two buildings, 186 full-rate bursts on file.
**No car runs happened on 2026-08-14.** Everything added today is bench work,
schematic work, or measurement with a meter.

Read `falcon_reference_2026-08-12.md` first — it remains the orientation
document. Session detail for today is in `falcon_ui_battery_2026-08-14.md`; the
live open-items list is `falcon_test_plan_2026-08-14.md`.

---

## 0. ⚠️ READ THIS BEFORE THE NEXT CAR RUN

**Nothing in this build has been in an elevator.** Two of today's changes are in
paths that matter, and one of them is a safety threshold:

| change | risk |
|---|---|
| **Calibration 10 s → 6 s**, quorum 6 → 4 | 🔴 **safety path.** Sets `XY_STILL`, the lateral floor. Validated against **six desk calibrations only** |
| **`LATCH_FAILSAFE_MS` 240 s → 600 s** | 🟡 a stuck beacon now sounds for ten minutes, not four |
| **Chase sweep advances onto Q1** | 🟡 changes the beacon visual; no timing change, does not gate sampling |
| Heartbeat on D2, low-battery chirp | 🟢 idle only; both suppressed while the beacon runs |

The next hoistway session is therefore a **regression run**, not a data-gathering
trip that happens to include one. §7 is the checklist.

---

## 1. What the device is for

Unchanged. A battery-powered **movement beacon** placed on an elevator
counterweight during maintenance. While the counterweight moves it sounds a
piezo and runs chase LEDs so a mechanic can locate it by ear. **When it stops,
the beacon must stop.**

Two failure modes, in order of seriousness:

1. **Silence while moving** — the mechanic believes a moving counterweight is
   parked. Catastrophic.
2. **A position lie** — asserting movement over a stationary counterweight.
   Destroys trust in the instrument and drains the pack.

**The safe direction is always "alarm longer."**

The physical constraint that shapes everything: **at constant velocity a moving
car and a parked car are indistinguishable on the vertical axis.** Movement can
only be detected as *events* — a departure transient and an arrival transient —
so the alarm is latched on departure and released on arrival.

---

## 2. Headline

**The reliability picture improved materially, and one long-standing blocker
turned out not to exist.** The ~4-minute "brownout" — the top open item since
2026-08-11, and the reason a failsafe was clamped and a beacon made quieter —
was a **COM-link lockup, not the device**.

**A defect nobody was looking for was found and fixed: the beacon had been
lighting seven of its eight ring LEDs for the entire project.** D10 never lit
during an alert. It was invisible from the source and only became visible once
the schematic was read.

**The battery measurement chain is trustworthy for the first time.** Divider
metered, ADC brought into spec, thresholds derived in pack volts and validated
against a meter to within one LSB. Quiescent current is measured rather than
assumed.

**The detection margins are exactly where they were this morning.** §4.2, §5.1,
§5.2, §5.3 and §5.6 are untouched. **That is where the remaining risk lives**,
and no amount of today's work moved any of it.

**The device is not production-ready.** §6 sets out the assessment.

---

## 3. What changed since 2026-08-13

| area | then | now |
|---|---|---|
| **beacon ring** | **7 of 8 LEDs lit** — D10 never | **all 8**, D3–D10, no timing change |
| §5.7 brownout | top blocker, cause disputed | **closed — it was the COM link** |
| `LATCH_FAILSAFE_MS` | 240 s placeholder | **600 s**; long slow runs no longer cut short |
| calibration | 10 s, quorum 6 of 10 | **6 s, quorum 4 of 6**, `static_assert`-guarded |
| calibration evidence | none exportable | **`XY: bmax` emitted**; `graph/calib_replay.py` |
| idle indication | none | **dim wink on D2 every 4 s**; doubles if calibration degraded |
| low battery | 1800/1800 ms forever, ~50% piezo duty | **80 ms chirp / 45 s**, suppressed during a beacon |
| battery cadence | loop passes; **~23 min to converge** | 30 s wall clock; **armed ~90 s after boot** |
| battery trip | 1600 "counts", meaning disputed | **3.6 V of pack**, derived and meter-validated |
| ADC | `/128` = 7.8 kHz, out of spec | **`/16` = 62.5 kHz**, 0.8% low bias removed |
| BOD | disabled | **enabled at 2.7 V** |
| quiescent current | never measured | **3.2 mA** (2.1 mA without logging) |
| D2 LED | assumed 200 mA | **measured 38.8 mA** |
| flash | 30660 (1596 free) | 31464 (**792 free**) |

Two latent bugs were also closed: `disable_battery_alarm()` could leave
`buzzer_on` latched — **blanking the accelerometer permanently** — and
`initialization()` was feeding the battery average from the unused external
MCP3208 while the real path used the internal ADC.

---

## 4. Validation data

### 4.1 On firm ground — unchanged, no new car runs

- **Departure detection.** Caught at 17, 18, 20, 25, 27 fpm and every speed to
  500. Never missed in a hoistway on current firmware. This is the catastrophic
  direction and it remains the best-evidenced part of the device.
- **The jog verdict.** 29/29 lifetime, opk ≤ 440 for real departures against
  jogs ≥ 1366, gate at 900 — no overlap.
- **The ramp discriminator.** 28+ automatic stops, block means 470–653,
  directionality 100% on every one, zero false latches across 89 replayed
  departure bursts.

### 4.2 The weakest number in the product — unchanged

```
34 automatic stops:   arrival peak 0.454 ... 0.713      gate 0.45
worst margin 1.009x   |   one stop measured 1.016x while its ramp cleared 2.18x
```

Not a thin tail — a distribution sitting on its own gate. ⚠️ And the structural
explanation was contradicted at the speed that most tested it: 500 fpm gave a
worst margin of 1.08× against 1.009× at 350 fpm. Three runs, so not an overturn.

### 4.3 New, bench only

| measurement | value |
|---|---|
| pack, metered vs firmware | 5.010 V vs `pack_mv 5012` — **inside 1 LSB** |
| idle current | **3.2 mA** |
| idle without serial logging | **2.1 mA** — logging is **34% of idle** |
| D2 at full / dim | 38.8 mA / 0.20 mA |
| heartbeat average | **4 µA** |
| divider settle | flat by 3 ms; `BATTERY_SETTLE_MS` 10 ms confirmed |
| 6 s vs 10 s calibration | 6 s never exceeded 10 s across six runs |

Standby life, first time it can be stated: **~13 days** at 1000 mAh, **~20 days**
with logging compiled out.

---

## 5. What is not proven

### 5.1 Two of three armed release paths have never been observed working

| path | status |
|---|---|
| jog verdict | armed, **proven** 29/29 live |
| ramp detector | armed, **never executed** — 14 latches, all after another path released |
| reversal arming (`v=2`) | armed, **never fired** in ~32 runs |

The latter two are armed on replay plus negative evidence alone, and they carry
the automatic-operation boundary between them.

### 5.2 The single-floor arming blind spot is unaddressed, and got worse

A single-floor terminal approach goes departure ramp straight into deceleration
with no cruise and no quiet samples, so the peak's gate never opens: `pk` reads
0.00 through a real excursion and the beacon sounded **85 s over a car
stationary for 78 of them**. Reproducible on demand.

🔴 **Raising `LATCH_FAILSAFE_MS` to 600 s means this defect can now run for ten
minutes rather than four.** That trade was taken deliberately — silence over a
moving counterweight is worse than a longer false beacon — but it raises the
priority of the arming-gate redesign rather than deferring it.

### 5.3 Arming margin is an install-time property that cannot be spot-checked

Bottom-terminal margins on nominally the same mounting span **q=6 to q=15**, with
the reversal gate reaching **`ro=8` — zero margin — once**. Two mountings minutes
apart differ by 40%. **A mechanic's placement decides whether the beacon releases
at the bottom terminal.** The device should measure and report its own margin.

### 5.4 Cruise content varies ~2× between identical runs

0.12, 0.12, 0.21, 0.24, 0.32 across five long runs. No single-run cruise number
means anything.

### 5.5 ~20% of samples are still lost

Every threshold on file stands on ~80% of the data. The dominant cost is the
**11.4 ms I2C sensor read occupying 28% of CPU at 1 MHz**.

### 5.6 Coverage is thin per configuration

Four configurations, but **one shaft and one unit each**; 500 fpm is n=3. The
low-speed long run has **never been completed** — both attempts were abandoned
when the device stopped, which is now understood to have been the COM link, so
**it is runnable again** (with serial disconnected).

### 5.7 ✅ CLOSED — the brownout was the COM link

Established at the bench. Both competing explanations — rail sag and
intermittent contact under vibration — were arguing about a fault in the
measuring apparatus.

It fits what never sat comfortably with either: "dead" was inferred **entirely
from the log stopping** — no reset flag, no LED state, no measured rail — while
the bench procedure already recorded that the USB-serial cable back-powers the
board through the RX ESD clamp. ⚠️ The 588 s "successful" endurance test is
reinterpreted too: it showed the COM link held that time, not that the pack did.

⭐ **Consequence not yet taken: the beacon was made quieter to defend against a
phantom.** The 2026-08-11 piezo duty cut 2/5 → 1/5 was made specifically to
attack this, knowingly costing audible length on a device the mechanic ranges
**by ear**. Audibility can be re-opened — but the real trade was never the
battery. Piezo duty costs **sensor coverage**, and 🔴 the 2026-08-07
quiet-fraction-0.75 false release still binds. Answerable by replay first.

### 5.8 ✅ CLOSED — both hardware settings resolved

BOD enabled at 2.7 V (`efuse 0xF5`). ADC prescaler `/16`, in spec, removing a
measured 0.8% low bias. ⚠️ Note `/8` would have been **worse** — the divider
presents ~250 kΩ, 25× the recommended source impedance, so the two datasheet
limits pull in opposite directions.

### 5.9 New, and unverified

- **Nobody has confirmed the D2 heartbeat wink is visible.** It measures 0.52%
  of full brightness — 12× dimmer than its 6.3% PWM duty implies, probably the
  driver's soft-start at Timer0's 61 Hz. If it cannot be seen, the feature is
  decoration.
- **The low-battery chirp has never been heard.**
- **The degraded double-wink has never been seen.**
- **6 s calibration is bench-only.**

---

## 6. Production readiness

**Three of the eight gaps recorded on 2026-08-12 are now closed** — the brownout,
the flash ceiling, and the battery measurement chain — plus BOD and the ADC.
That is real progress on *reliability*.

**It changes nothing about the release path.** Items §4.2, §5.1, §5.2 and §5.3
are exactly as they were, and they are the direction that produces either a
position lie or silence while moving:

1. **§5.2** — a reproducible position lie, now up to 600 s. The arming-gate
   redesign is the single highest-value engineering item.
2. **§4.2** — automatic operation releases on a 1.009× worst margin.
3. **§5.1** — the two paths meant to fix §4.2 have never been observed working.
4. **§5.3** — whether the beacon releases at a bottom terminal depends on how a
   mechanic placed it, and cannot be verified at commissioning.
5. **§5.6** — one shaft and one unit per configuration. **A second unit in a
   second building is a coverage gap no amount of testing in this shaft closes**,
   and should be scoped separately.

⚠️ **And this build has never been in a car** (§0).

---

## 7. Next steps

### 7.1 The car session — this is the regression run

| | do | pass criterion |
|---|---|---|
| C1 | Power on at the counterweight, capture `XY: bmax`. **3+ mountings** | `graph/calib_replay.py`: **6 s must not exceed the 10 s answer**. If it does, revert to 10 s / quorum 6 |
| C2 | Watch one alert | **all eight ring LEDs sweep, D3→D10, no gap** |
| C3 | Multi-floor automatic run, both directions | departure caught, arrival releases, `ARM q=` recorded |
| C4 | Single-floor terminal approach | expected to still fail — record `pk=0.00` and duration. This is the reproduction, not the fix |
| C5 | Short run **starting near the bottom terminal** | `ARM q=`, `ro=` recorded. A long descent does NOT test this |
| C6 | Low-speed long run | now runnable — **serial disconnected** |

**Rollbacks if the car misbehaves:**

| symptom | first move |
|---|---|
| beacon releases on a moving car | `RAMP_ARMED 0` |
| false jog release | `JOG_VERDICT_ARMED 0` |
| twitchy, or never releases at the terminal | `CALIB_TIMEOUT_MS` 10000 + `XY_CALIB_BUCKETS` 10 + `XY_CALIB_MIN_BUCKETS` 6 |

### 7.2 Bench, no car needed

1. **Confirm the D2 wink is visible** (§5.9). If not, raise the duty and
   re-measure — brightness is not linear in it.
2. **Hear the chirp; see the double wink.** `pio run -e bench_battery -t upload`.
3. **Production build with logging compiled out** — 1.10 mA, 34% of idle,
   ~50% more standby. Largest untaken win.

### 7.3 Engineering, replay-gated

1. **Redesign the arming gate** for §5.2 — key on "the departure ramp has ended"
   rather than "the signal went quiet". ⛔ Must be replayed through
   `graph/arming_replay.py` against all 186 bursts before it goes in a car.
2. **Have the device report its own arming margin** (§5.3).
3. **Re-open beacon audibility** (§5.7) — by replay, not by revert.
4. **Attack the 11.4 ms sensor read** (§5.5).
5. **`RollingAvg` static buffers** — 586 bytes, and the end of dynamic allocation.

---

## 8. Method note

**Five interpretive claims were published and then overturned by the next
measurement today, all of them mine:**

| claim | reality |
|---|---|
| "X/Y are never logged" | they were, in every log already read that session |
| "D3 sits on Q0 and burns current continuously" | Q0 is unpopulated; the ring is free at rest |
| "the pack sags 1.6 V under alert load, and that explains the brownout" | rails are stable at 4.9 V |
| "D2 is a 200 mA part" | 38.8 mA measured |
| "the heartbeat costs ~250 µA" | 4 µA |

A sixth — "the first buckets of a calibration are the noisiest" — was caught
before it was written down, by taking five more runs first.

Two patterns are worth carrying forward, because both recurred:

**A story that neatly explains an old mystery is more suspect, not less.** The
sag hypothesis was satisfying precisely because it resolved the brownout, and
the contradicting evidence was **already on file** — the 588 s test's +0.6 mV
blast-versus-quiet figure — and went unchecked. Check a new story against the
existing corpus before committing it.

**Arithmetic from a datasheet is not a measurement.** The 200 mA rating produced
a current budget, a recommendation to change battery chemistry, and a design
justification, all wrong by a factor of five, and all avoidable by one meter
reading.

> **Treat any interpretive claim in these notes as weaker than the number it
> rests on, including the confidently worded ones.** Where a claim rests on a
> single run or a single reading, it is marked.

What did work: the D10 defect was found by **reading the schematic rather than
the source** — it is invisible from the code, which is correct on its own terms
and only becomes wrong once you know Q0 lights nothing. And every fuse and
threshold change today was verified by **observing the device**, not by trusting
a tool's success message — a distinction that matters, since a bricking-level
BOD would have programmed and verified perfectly.
