# Falcon firmware — state of the project

> ## ⚠️ SUPERSEDED, 2026-08-11. Historical only.
>
> This describes the project as of 2026-08-07 and several of its conclusions
> have since been overturned by measurement. In particular: the sample rate is
> now 25 Hz (roadmap item 5), arrival detection no longer reads the rolling
> average, the 1–3 s reset budget has been relaxed by the customer to ~8 s, and
> the Z + X/Y approach designed after this note was written has been measured
> and ruled out on this equipment.
>
> **For the current state read `falcon_state_of_project_2026-08-13.md`**, which
> supersedes this document, and `falcon_reference_2026-08-12.md`, which is the
> orientation doc and is kept current. The intervening detail is in
> `falcon_signature_2026-08-11.md`, `falcon_25hz_arrival_2026-08-10.md` (which
> carries its own correction banner) and `falcon_hoistway_protocol.md`.
>
> Kept because the baseline comparison against `59e945f` and the 2026-08-07
> hoistway numbers are still the reference for how far things have moved.

**Date:** 2026-08-07
**Baseline compared against:** `59e945f` ("Added a workaround") — the vendor
firmware as received, and the last commit before this work began
**Current head:** `Falcon_Rel_EFT` on `Dave356w/Spanidea_Falcon_shadow`
**Evidence:** the eight PuTTY captures on `eft-results-2026-07-15`, two bench
sessions, and 12 instrumented hoistway runs on 2026-08-07 spanning 18–123 fpm

This is a summary. The reasoning and raw numbers behind every claim are in
`falcon_analysis_2026-08-06.md`, which is the working document; section
references below point there.

---

## 1. What the device is for

A mechanic's safety device, placed in the hoistway — initially on the
counterweight — to warn of unintended movement while someone is working. The
counterweight is "the silent killer": it moves without warning and out of view.
Used case by case rather than continuously, though a construction crew on a
running platform may use one all day.

That use case makes the failure modes sharply asymmetric, and it is the single
most important input to how this firmware is now tuned:

| Failure | Consequence |
|---|---|
| Missed departure | The mechanic is not warned. **Catastrophic.** |
| Alarm released mid-travel | Alarm stops while the counterweight is still moving — the exact moment it is needed. **Catastrophic.** |
| Alarm fails to release / runs long | Nuisance. The mechanic is present and knows the state. |
| False alarm while parked | Nuisance, but erodes trust; alarm fatigue is its own hazard. |

**The safe direction is always "alarm longer."**

---

## 2. Headline: the original complaint is fixed

The vendor firmware stopped alarming part-way through a slow run. That was not
a tuning error — it was structural, and it had three independent causes, all now
addressed:

1. **The detector ran at 3.13 Hz, not the 100 Hz it was designed for** (§2). A
   4-sample rolling average therefore spanned 1.28 s and flattened the very
   transients it was meant to catch.
2. **Constant velocity is not observable with an accelerometer** (§3). A car
   cruising and a car parked both read 1 g. The old FSM asked "is the average
   still displaced?" mid-run, and during cruise the honest answer is no — so it
   cleared the alarm whether or not the car had arrived.
3. **Nothing was listening while the alarm sounded** (§5). Measured: 10,224 ms
   of alarm produced **one** accelerometer sample.

The current firmware latches on departure, holds through cruise with no timeout,
and releases on a detected arrival. Validated across 12 hoistway runs.

---

## 3. Changes from the vendor firmware

22 commits, ~1,400 lines changed across 8 source files. Grouped by what they do.

### 3.1 The device could not tell when it was broken

| Defect | Evidence | Fix |
|---|---|---|
| **A dead sensor reported a valid `0.0`** — read errors discarded, outputs pre-zeroed, so a non-responding accelerometer looked like a perfectly stationary one | Observed twice; 55 s of well-formed telemetry with the sensor absent (§10.2) | Transport now reports NACKs and short reads; all-axes-zero rejected; failures logged and excluded from the average |
| **The I²C wrappers returned success unconditionally** | Source; made the above fix inert until corrected (§10.2) | `bma_i2c_read/_write` return `BMA4_E_FAIL` |
| **The battery alarm latched permanently** — `disable_battery_alarm()` was defined and never called | A single 1545 reading on a healthy pack (2324–2390 all session) latched it; only a cold boot cleared it (§10.3) | Decision on an averaged, primed window with hysteresis; recovery path wired up |
| **Self-calibration ran on one sample** — `TCNT1` never cleared and `OCR1A` double-buffered in mode 9, so the first timer tick could be up to 67 s late | `Zero-Calib-Value` came out byte-identical to a single reading (§2a) | Counter and compare value written before the mode and clock bits |
| **`acceleration_avg_g` was never primed** | Unprimed window ramps 0 → 2.43 → 4.86 → 9.72, which the FSM read as a large acceleration; produced a false alarm on a stationary bench and then latched a stale baseline it could never clear |
| **Serial printing from inside the timer ISR** | 14 corrupted lines across the 8 July captures (§6) | ISR publishes a snapshot; `loop()` prints |

### 3.2 Detection was rebuilt around the sensor's own engine

The BMA456 has an any-motion engine that applies threshold-and-duration to its
own 100 Hz waveform. The vendor firmware did not use it. This is the single
biggest change.

- **Departures are now detected by the sensor, not by polling.** The polled path
  has never detected a departure in any hoistway run. The sensor caught an
  **18 fpm departure of −0.116 m/s²** (§12) — a case §3 had concluded from the
  July data that no threshold could recover. That conclusion was true of the
  polled path and false of the sensor.
- **Arrival uses both paths, and both are load-bearing** (§14.1). On 2026-08-07
  a ~50 fpm descent released on the sensor at 0.278, below the polled threshold;
  an 18 fpm descent released on polling with only one sensor edge. Each rescued
  a run the other would have missed.
- **The alarm now holds through cruise.** Item 7 restored ~45% sample coverage
  during a beep cycle, which is what made a latched design possible at all.

Hardware note: `INT1_ACC` → INT0/PD2 and `INT2_ACC` → INT1/PD3 were confirmed
from the schematic (§11). Both accelerometer interrupts reach true external
interrupt pins. **No board change was needed** — the hardware was ready and only
the firmware was missing.

### 3.3 Alarm behaviour

- Latched departure → hold → release on arrival → confirm settled (§13)
- Holds through levelling (5 s of continuous quiet inside 0.10 required before
  silencing) — a terminal-floor approach previously silenced during levelling
  and then re-alarmed on the brake set, producing a 73-second alarm on a
  stationary car
- Failsafe raised to 300 s from measurement: a full 18 fpm descent held the
  alarm for **171.6 s**, 95% of the previous 180 s budget
- Battery-alarm path no longer unblanks the sensor mid-beep (§5, item 7a)

---

## 4. Validation data

12 instrumented runs on 2026-08-07, cartop inspection, 18–123 fpm, both
directions.

| Speed | Dir | Held | Cruise peak | Arrival delta | Released by |
|---|---|---|---|---|---|
| 18 fpm | down | 171.6 s | 0.271 | 0.312 | polled |
| ~50 fpm | down | 53.4 s | 0.086 | 0.278 | any-motion |
| 58 fpm | down | 60.7 s | 0.218 | 0.383 | polled |
| 123 fpm | up | 36.1 s | 0.124 | 0.362 | polled |
| 123 fpm | down | 34.8 s | 0.103 | 0.362 | any-motion + delta |

Five consecutive correct runs in the final configuration: departure latched,
alarm held through cruise and levelling, released at the brake set. No
mid-travel releases, no failsafes, no double alarms, and zero any-motion edges
during cruise on any of them.

**Stationary soak** (§14.6): 17.6 minutes parked — **zero false departures, zero
any-motion edges**, polled margin ~4× with a p99 of 0.032. This retires the
oldest open caveat in the analysis for the cartop case.

**Memory and battery:**

| | Vendor `59e945f` | Now |
|---|---|---|
| Flash | 20,890 bytes | 25,084 (77.8%) |
| RAM | 1,687 bytes | 1,007 (49.2%) |

RAM *fell* despite substantial additions, because `Serial.print` string literals
were moved to flash. Stack headroom roughly doubled.

Battery: 2236 → 2209 counts across two hours of heavy use including many
80-second continuous alarms. Comfortable for a shift.

---

## 5. What is not proven

Listed in rough order of how much they should worry you.

### 5.1 The counterweight — the largest open risk

**Every number in the release logic is fitted to cartop data. The device
deploys on a counterweight, and that ride is rougher.**

A rougher ride pushes *both* release paths toward firing during travel: cruise
peak rises toward the polled threshold, and more cruise edges make the sensor
path cluster. Both failures land on **mid-travel release** — the one failure
that can hurt someone.

Worse, **logging on a counterweight is not safely possible**: a serial cable to
a moving counterweight means being in the hoistway, which is the hazard the
device exists to warn about. So the measure-then-tune method that produced every
number in this report is unavailable exactly where it matters most.

The proposed answer is on-device recording (roadmap item 11): store per-run
statistics to EEPROM during the job, dump them over serial afterwards with the
car parked and the hoistway safe. ~8 bytes per run gives ~100 runs of history in
the 1 KB available.

### 5.1a Arrival detection has a floor that no threshold clears

Late on 2026-08-07 a smooth 18 fpm stop produced an arrival of **0.058**
averaged — against a **0.103** ceiling measured on a *parked* device during the
17.6-minute soak (§14.7).

**The arrival was smaller than the device's own stationary noise.** There is no
gap to place a threshold in: anything low enough to catch it sits under a level
a parked unit crosses on its own, and that same floor is what departure
detection must stay above.

It is not a speed effect. Two 18 fpm arrivals the same day measured 0.312 and
0.058 — a 5× spread — while two 123 fpm arrivals agreed to three decimals
(§14.8). Slow running does not weaken arrivals; it gives the machine room to
stop gently. A minimum-speed specification narrows this exposure without closing
it.

The conclusion (§14.9) is that arrival detection should stop being tuned. The
honest options are to let the alarm run to the failsafe, or to have the mechanic
silence it — the latter being deterministic and unable to fail dangerously.

**Departure detection is unaffected.** It caught −0.116 m/s² at 18 fpm and 0.838
later the same day, and has worked at every speed and configuration since the
threshold was set to 32. The half that protects someone is the half that works.

### 5.2 The arrival threshold is fragile in a way we understand

`ARRIVAL_CLUSTER_DELTA = 0.20` works across every run measured, but cruise peak
is a *running maximum* and therefore grows with run length, not speed (§14.3):

```
34.8 s → 0.103        171.6 s → 0.271
```

A longer run in a taller building would eventually exceed the gate. The fix is a
duration-independent statistic — recent cruise rather than a running maximum, or
a percentile — not a larger constant. Roadmap item 8a.

### 5.3 Sample size

The arrival logic was rewritten three times on 2026-08-07, each revision
prompted by the previous run's failure, and rests on roughly a dozen arrivals in
**one shaft on one unit**. Departure detection is on much firmer ground — it has
worked across every speed and configuration since the threshold was set to 32.

### 5.4 Untested paths

- The `a=ERR` sensor-fault path has never fired on hardware
- What the unit should *do* when the sensor is dead is unresolved (roadmap 4a) —
  it no longer lies, but raises no user-visible fault
- 17.6 minutes of soak is short against a full-day construction deployment

---

## 6. Recommended next steps

1. **Decide whether automatic release should exist at all — now the top
   question.** §14.7 shows a real arrival measuring *below* the device's own
   parked noise floor, so this is no longer a tuning gap that more data closes.
   A mechanic beside the device can silence it; the device deciding from a 3 Hz
   accelerometer that the counterweight has stopped is a judgement with no
   ground truth, and it is the source of nearly every difficult problem in this
   project. "Runs until silenced" solves the original complaint too and cannot
   fail dangerously.
2. **Build the on-device black box** (item 11) before any counterweight
   deployment, so its thresholds can be set from evidence.
3. **If a counterweight test happens first**, set the arrival gates unreachable
   (e.g. 5.0) so nothing can release early and every run holds to the failsafe.
   That yields the characterisation at no risk of an early release.
4. **Fix the duration-fragile arrival statistic** (item 8a).
5. **Timer1 at 100 Hz** (item 5) is no longer urgent — the sensor carries
   detection now — but it costs 61% of a 10 ms period for the I²C read alone
   (§10.4), so it needs either F_CPU at 8 MHz or the read moved out of the ISR.

## 7. Questions for Biju

Unchanged from the analysis, plus one:

- Why `0.40` specifically, and was it field-validated?
- Has `59e945f` had any hoistway time?
- What should the unit do when its accelerometer fails?
- Is the DPS310 permanently depopulated? A barometer would make constant
  velocity directly observable (§3) and would sidestep most of this.
- **Should the alarm end automatically at all, or on mechanic action?**
