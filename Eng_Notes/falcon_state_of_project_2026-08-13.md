# Falcon firmware — state of the project

**Date:** 2026-08-13
**Head:** `Falcon_Rel_EFT` on `Dave356w/Spanidea_Falcon_shadow`
**Firmware on the device:** `4272136` — Flash 30660/32256, RAM 1369/2048
**Supersedes:** `falcon_state_of_project_2026-08-07.md`
**Evidence base:** ~50 instrumented hoistway runs across **four configurations**
(cartop inspection, cab inspection, cab automatic 300–350 fpm, cab automatic
500 fpm), speeds **17–500 fpm**, in **two buildings**; 186 full-rate sample
bursts on file; plus bench soaks and a 588 s continuous-alarm endurance test.

Read `falcon_reference_2026-08-12.md` first — it is the orientation document and
is kept current. This is the summary of where things stand and what is not yet
known. Session notes are listed in that document's chronology.

---

## 1. What the device is for

A battery-powered **movement beacon** placed on an elevator counterweight (or
cartop) during maintenance. While the counterweight moves it sounds a piezo and
runs chase LEDs so a mechanic in the hoistway can locate it. **When it stops, the
beacon must stop.**

Two failure modes, in order of seriousness:

1. **Silence while moving** — the mechanic believes a moving counterweight is
   parked. Catastrophic.
2. **A position lie** — the beacon asserting movement over a stationary
   counterweight. Destroys trust in the instrument and drains the pack.

**The safe direction is always "alarm longer."** Every design decision below is
shaped by that asymmetry.

The physical constraint that shapes everything else: **at constant velocity a
moving car and a parked car are indistinguishable on the vertical axis.** Both
read 1 g. Movement can therefore only be detected as *events* — a departure
transient and an arrival transient — so the alarm must be latched on departure
and released on arrival. A "has it gone quiet?" rest detector cannot exist on
this axis; one was built on 2026-08-07 and released the beacon mid-ride twice.

---

## 2. Headline

**Inspection operation works.** Departures caught down to **17 fpm** — the
slowest ever measured. Arrivals release with 2–5× margin. The jog defect is
verdicted **29/29 lifetime** with zero false releases.

**Automatic operation releases on a distribution centred on its own threshold.**
34 automatic stops measure an arrival peak of **0.454–0.713 against a 0.45 gate**
— worst margin **1.009×**. This is not a thin tail; it is a distribution sitting
on the gate. The cause is structural and understood (§4.2).

**Reliability improved materially on 2026-08-12/13.** The TWI lockup — which had
frozen the device solid and, on 2026-08-12, cost an entire car run with the
beacon silent while a counterweight moved — is fixed. The flash ceiling that
blocked everything is gone. The four-minute "brownout" that was called the
product's most serious defect has been **refuted as a rail problem**, with an
important caveat (§5.7).

**The device is not production-ready**, and §6 sets out that assessment against
Dave's more optimistic read.

---

## 3. What changed since 2026-08-07

| area | then | now |
|---|---|---|
| sample rate | 3.13 Hz (timer defect) | **25 Hz** |
| arrival metric | rolling average | **windowed peak of the raw sample** |
| departure | sensor any-motion engine | unchanged — still the only one that works |
| jog defect | latched to failsafe | **verdicted and armed**, 29/29 |
| ramp detector | did not exist | **built and armed**, own gate; never yet executed |
| arming | single shared gate | **two independent gates**, one per detector |
| TWI lockup | froze the device, unrecoverable | **bounded waits + 2 s watchdog**; 3 wedges caught, 4 WDT catches |
| flash | 30 bytes free | **1594 free** |
| serial | 9600 | **62500** |
| alarm UI | buzzer and chase beating against each other | **one blast per 8-LED sequence**, chase 2× faster |
| replay tooling | none | `graph/arming_replay.py`, 186 bursts |

Details in the session notes; the reference document carries the current
legends, thresholds and logic.

---

## 4. Validation data

### 4.1 What is on firm ground

- **Departure detection.** Caught at 17, 18, 20, 25, 27 fpm and every speed up
  to 500. It has never been missed in a hoistway on current firmware. This is the
  catastrophic direction and it is the best-evidenced part of the device.
- **The jog verdict.** 29/29 lifetime. Real departures measure opk ≤ 440 against
  jogs ≥ 1366, with a gate at 900 — no overlap on the axis that carries the
  verdict.
- **The ramp discriminator.** 28+ automatic stops, block means **470–653**,
  directionality **100% on every one**, against complete negative evidence from
  inspection operation (brake stops, jogs, cruise, departures all declined) and
  zero false latches across 89 replayed departure bursts. At 500 fpm it is
  *tighter* than at lower speeds: 597/600/602.
- **Alarm endurance.** 588 s of continuous sounding with VCC flat at ~3.115 V
  and no measurable sag between blast and quiet phases (mean +0.6 mV over 583
  samples).

### 4.2 The weakest number in the product

```
34 automatic stops:   arrival peak 0.454 ... 0.713      gate 0.45
worst margin 1.009x   |   one stop measured 1.016x while its ramp cleared 2.18x
```

The cause is structural: the peak is a deviation from a rolling average, and a
sustained deceleration ramp drags the average with it, so the metric gets
*weaker* as ramps get longer. The ramp detector exists to answer this.

⚠️ **But the structural argument was contradicted at the speed that most tested
it.** 500 fpm is the longest ramp measured and should have been the worst case;
instead the worst margin there was 1.08× against 1.009× at 350 fpm. Three runs,
so not an overturn — but the effect is smaller than feared.

---

## 5. What is not proven

### 5.1 Two of three armed release paths have never been observed working

| path | status | evidence |
|---|---|---|
| jog verdict | armed, **proven** | 29/29 live |
| ramp detector | armed, **never executed** | 14 latches, every one after another path had already released |
| reversal arming (`v=2`) | armed, **never fired** | ~32 runs |

The ramp check sits inside `STATE_MOVING` and the peak always crosses first, so
the ramp is insurance against a stop the peak misses — and **no stop in 34 has
come in under 0.45.** The latter two are armed on replay plus negative evidence
alone, and they carry the automatic-operation boundary between them.

### 5.2 The single-floor arming blind spot is unaddressed

A single-floor terminal approach goes departure ramp straight into deceleration
with no cruise and no quiet samples, so the peak's gate never opens: `pk` reads
0.00 through a real excursion, and the beacon sounded **85 s over a car
stationary for 78 of them.** Reproducible on demand. The 2026-08-13 `ramp_gate`
work fixed the *opposite* direction (a departure ramp being mistaken for an
arrival) and does not touch this.

### 5.3 Arming margin is an install-time property that cannot be spot-checked

Bottom-terminal margins on nominally the same mounting span **q=6 to q=15**
across eight runs, with the reversal gate reaching **`ro=8` — zero margin — once.
Three of the four gates in front of the arrival detectors have now been observed
at or near their minimum** (quiet 6, reversal 8, peak 1.009×).

Two mountings minutes apart differ by 40%, and one sat at zero. **A mechanic's
placement decides whether the beacon releases at the bottom terminal**, and a
commissioning spot-check cannot establish it. The device should measure and
report its own margin.

### 5.4 Cruise content varies ~2× between identical runs

0.21 against 0.12 on the same shaft, direction, speed and mounting. Across five
long runs: **0.12, 0.12, 0.21, 0.24, 0.32.** No single-run cruise number means
anything — which also means the cruise-to-gate margin is only known to within a
factor of two.

### 5.5 ~20% of samples are still lost

Every threshold on file stands on ~80% of the data. Raising the serial rate
6.5× recovered only ~15% of the loss; the dominant cost is the **11.4 ms I2C
sensor read occupying 28% of CPU at 1 MHz**. That is the next lever, not the log.

### 5.6 Coverage is thin per configuration

Four configurations, but **one shaft and one unit each**, and the 500 fpm regime
is n=3. The low-speed long-run case — the one the 0.45 threshold most needs — has
**never been completed**, because both attempts were abandoned when the device
stopped (§5.7).

### 5.7 ⚠️ The brownout is refuted only while the device is held still

The 588 s endurance test was **stationary on a bench**; the 2026-08-11 failures
were **in a moving car**. That difference admits a hypothesis nobody had
considered: **an intermittent battery contact under vibration.** It would produce
exactly the observed signature — sudden death, no reboot, healthy readings to the
last sample — and would specifically *not* reproduce on a still bench.

Also unresolved: the wedge was not caught in the act either (`tw` stayed 0), so
it cannot be claimed as positively the 2026-08-11 cause; and the pack did not
droop at all, so the original 2442 → 2324 decline still needs its own
explanation.

### 5.8 Two hardware settings that should be checked

- **BOD is probably disabled** (ATmega328 factory efuse `0xFF`). For a device
  whose worst failure is silence while moving, a sagging rail that causes a clean
  reset and reboot is far better than one that hangs. Read the efuse.
- **The ADC prescaler is `/128` under a comment reading "for 8MHz clock"** while
  F_CPU is 1 MHz — so the ADC clock is 7.8 kHz, below the 50 kHz datasheet
  minimum for full 10-bit accuracy. **Every battery reading this project has
  taken is out of spec.**

---

## 6. Production readiness — an explicit disagreement

Dave's read on 2026-08-12 was that the project is close to production. On the
measurements, this document does not agree, and the difference is recorded here
rather than left implicit. His commercial judgement is his own; the engineering
position is:

**Closed since that assessment:** the TWI lockup (§3) and the rail-sag
explanation of the four-minute death (§5.7, with caveat). Both were real
blockers and both moved.

**Still open, in order:**

1. **§5.7's vibration hypothesis.** Until the endurance test runs in a moving
   car, "the device survives a long alarm" is only established for a stationary
   device — and every historical failure was a moving one.
2. **§4.2.** Automatic operation releases on a 1.009× worst margin.
3. **§5.1.** The two paths meant to fix §4.2 have never been observed working.
4. **§5.2.** A reproducible 85 s position lie.
5. **§5.3.** Whether the beacon releases at a bottom terminal depends on how a
   mechanic placed it, and cannot be verified at commissioning.
6. **§5.6.** One shaft and one unit per configuration.

Items 2–5 are all in the release path, which is the direction that produces
either a position lie or silence while moving.

---

## 7. Next steps

1. **Run the endurance test build in a moving car** (`pio run -e brownout_test`).
   Separates the wedge from a vibration-sensitive contact. Cheapest remaining
   test with the largest consequence. ⛔ Bench/test build — it bypasses the FSM
   and nothing can release the beacon.
2. **Read the efuse; consider enabling BOD** (§5.8).
3. **Raise `LATCH_FAILSAFE_MS` back toward 600 s** once (1) is clean — it was
   clamped to 240 s to stay inside a limit that appears not to exist.
4. **Complete a low-speed long run** and measure the cruise ceiling (§5.6).
5. **Attack the 11.4 ms sensor read** — faster TWI or a shorter transaction
   (§5.5).
6. **Redesign the arming gate** for §5.2: key on "the departure ramp has ended"
   rather than "the signal went quiet". Use `graph/arming_replay.py` and the §8a
   method in the reference document; do not change arming without re-running it.
7. **Have the device report its own arming margin** (§5.3).
8. **`RollingAvg` static buffers** — 586 bytes and the end of dynamic allocation.

---

## 8. Method note, and how to read this document

Across 2026-08-12/13, **nine interpretive claims were overturned by the next
measurement, and five of them were written by me** — including two that survived
only hours inside the note that made them, and one bug reintroduced from a note
that documented the same bug being fixed.

The measurements have held up. The *summaries of what they imply* have repeatedly
run ahead of them. So:

> **Treat any interpretive claim in these notes as weaker than the number it
> rests on, including the confidently worded ones.** Where a claim rests on a
> single run, it is marked; single-run numbers should be treated as hypotheses
> until a second run under different conditions agrees.

The habit that has actually worked is distrusting results that contradict
observed device behaviour — that is what caught two invalidating bugs in the
replay tool and one silent test build that was measuring an unloaded rail.
