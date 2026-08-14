# Test plan — what is left, and the order to do it in

**Date:** 2026-08-14
**Firmware on the device:** `31416` — Flash 31416/32256 (840 free), RAM 1357/2048
**Basis:** open items from `falcon_state_of_project_2026-08-13.md` §5–§7, plus
everything the 2026-08-14 session changed and could not exercise.

---

## 0. The one thing to read first

**Nothing from 2026-08-14 has been in a car.** Four sessions of work landed on
the bench today, and two of those changes are in paths that matter:

| change | risk | why |
|---|---|---|
| **6 s calibration window** | 🔴 **safety path** | it sets `XY_STILL`, the lateral floor. Validated against six *desk* calibrations only |
| **chase sweep advances onto Q1** | 🟡 beacon visual | changes what the mechanic sees; no timing change, does not gate sampling |
| heartbeat on D2 | 🟢 idle only | suppressed whenever the beacon runs |
| battery chirp + cadence | 🟢 idle only | suppressed whenever the beacon runs |

So **session C below is not optional polish — it is the regression run for
today.** Until it happens, the shipping build carries a threshold change
justified entirely by bench data.

---

## 1. Open items, consolidated

### 1.1 Blockers (from §6 of the state-of-project note, unchanged)

| # | item | status |
|---|---|---|
| 1 | **§5.7 vibration hypothesis** — endurance proven *stationary* only (588 s); every historical death was in a *moving* car | untested |
| 2 | **§4.2** — 34 automatic stops release on a distribution centred on the gate; worst margin **1.009×** | measured, unfixed |
| 3 | **§5.1** — ramp detector (14 latches, never executed) and reversal arming (`v=2`, never fired in ~32 runs) | armed, never observed working |
| 4 | **§5.2** — single-floor terminal approach: `pk` reads 0.00 through a real excursion, beacon sounded **85 s** over a car stationary for 78 | reproducible, unfixed |
| 5 | **§5.3** — arming margin spans `q=6…15` on nominally identical mountings, `ro=8` once (zero) | measured, unfixed |
| 6 | **§5.6** — one shaft and one unit per configuration; 500 fpm is n=3 | coverage gap |

### 1.2 Added or left open by 2026-08-14

| # | item | status |
|---|---|---|
| 7 | **Battery trip point uncharacterised** — mV compared against a number with no divider applied; `Release.txt` says 3.2 V, arithmetic says 7.04 V | blocks shipping the chirp |
| 8 | **ADC prescaler `/128` at F_CPU 1 MHz** → 7.8 kHz vs 50 kHz datasheet minimum. Every battery reading ever taken is out of spec | one-line fix, needs a check |
| 9 | **BOD probably disabled** (factory efuse `0xFF`) | read the efuse |
| 10 | **Quiescent current never measured** — heartbeat's ~0.25 mA is arithmetic off a 200 mA rating | blocks any runtime claim |
| 11 | **Low-battery chirp never heard** — pack reads 2535 against a 1600 trip | untested path |
| 12 | **Degraded double-heartbeat never seen** — needs a rejected calibration | untested path |
| 13 | **6 s calibration is bench-only** | needs hoistway `XY: bmax` |
| 14 | **D10 fix unseen in a real alert** | needs a car run |

---

## 2. The plan

Four sessions. **A and B are independent of each other; C depends on nothing but
should follow A; D is engineering, not testing.** The ordering below is by
consequence-per-hour, which is the same principle §7 of the state-of-project
note used to put the endurance test first.

---

### SESSION A — bench with a meter (~1–2 h, no car)

**Why first:** it is the only session that needs no elevator, it unblocks three
separate claims, and item 7 is what stands between the chirp and a customer.

| step | what | exit criterion |
|---|---|---|
| A1 | **Meter the battery divider.** Measure pack voltage and the ADC node with the divider enabled; compare against the printed `Voltage value`. | a volts-per-count figure, written into `main.cpp` next to `BATTERY_LOW_THRESHOLD` |
| A2 | **Set the real trip point.** Convert 3.2 V (or whatever Dave decides) into the units the code actually compares. | `BATTERY_LOW_THRESHOLD` derived, not inherited |
| A3 | **Fix the ADC prescaler** to land the ADC clock inside 50–200 kHz, then **re-run A1** — the thresholds were characterised at the out-of-spec setting, so this invalidates them. | prescaler in spec, A1 numbers re-taken |
| A4 | **Exercise the chirp.** Temporarily raise the trip above the observed reading. | immediate chirp on trip, one per 45 s after, **silent through a simulated alert**, clean clear on recovery |
| A5 | **Measure quiescent current**, heartbeat on and forced off. | a mA figure; heartbeat cost stated in runtime, not arithmetic |
| A6 | **Read the efuse.** `avrdude -c stk500 -P COM7 -p m328pb -U efuse:r:-:h` | BOD state known; decide whether to enable |
| A7 | **Force a rejected calibration** — move the unit during the 6 s window. | `XY: calib mv=1` → retry → `READY (fallback)`, **double** heartbeat wink |

⚠️ **A3 before A2 in practice** — fixing the prescaler moves the readings, so
setting a threshold first wastes the work.

**Unblocks:** items 7, 8, 9, 10, 11, 12.

#### A0 — build it with `bench_battery`

```bash
cd falcon_srcs && pio run -e bench_battery -t upload
```

⛔ **Never in a car** — it replaces the threshold logic with raw instrumentation
and forces the alarm on a timer. But unlike `brownout_test` it is a **superset**
of shipping, not a bypass: FSM, calibration, beacon and heartbeat all run as
shipped, so **A7 happens in the same build**. Sizes: shipping 31416,
`bench_battery` 31546, `brownout_test` 21086 — a mis-flash is visible by
inspection. **Re-flash shipping when done.**

#### ✅ Already answered by the first bench_battery flash (2026-08-14)

**`BATTERY_SETTLE_MS` = 10 ms is confirmed adequate.** It was a generous guess,
never a measurement. The settle sweep across 87 samples:

```
BB t=17s raw 1ms=785 3ms=833 6ms=833 10ms=837 20ms=834 50ms=837  mv10=2536 vcc=3120
```

The 1 ms tap reads ~785 against a settled ~833 — about 6% low. **Everything from
the 3 ms tap onward is flat.** The node settles between 1 and 3 ms, so 10 ms has
3× margin. No change needed; the question is closed.

Cross-check: `mv10` 2521–2536 against the shipping build's 2533–2551 on the same
pack. The bench build agrees with shipping, which is the assurance that the
characterisation transfers.

Also visible: `vcc` drops 3120 → 3103 on the sample where the forced chirp
starts — the piezo load appearing on the rail, via the bandgap read.

#### 🔎 A1 has a strong prior now — check this first with the meter

`main.h` declares `BATT_R1 5100` / `BATT_R2 1500`, ratio 4.4, which is what made
`BATTERY_LOW_THRESHOLD 1600` look like an implausible 7.04 V pack. **But sheet 3
of `RTC1273R2_SCH.pdf` shows the BATTERY MONITORING block built from 499k
resistors** — and a 499k/499k divider is **2:1**, not 4.4:1.

If it is 2:1, everything reconciles at once:

```
node 2540 mV  x 2  =  5.08 V pack     (3 fresh cells in BT1 -- plausible)
trip 1600 mV  x 2  =  3.20 V          (EXACTLY Release.txt's stated 3.2 V)
```

**1600 → 3.2 V matching `Release.txt` to two significant figures is not a
coincidence.** The likely story is that the trip point has been correct all
along, and `VBATT_CONST` is a V1 leftover — the same failure mode as
`PIN_GREEN_LED` and the `BATT_SENSE` MCP3208 path.

⚠️ **Do not act on this without the meter.** It is schematic reading plus
arithmetic, and schematic reading already produced one confidently wrong claim
this session (D3 on Q0). A1 becomes a 30-second confirmation rather than a
characterisation — measure pack and node, and see whether the ratio is 2.

---

### SESSION B — endurance in a moving car (~30 min in car)

**This is item 1, and it is still the cheapest test with the largest
consequence.** The 588 s bench result proves the device survives a long alarm
*while held still*; every historical death was in motion. An intermittent
battery contact under vibration would produce exactly the observed signature —
sudden death, no reboot, healthy readings to the last sample — and would
specifically not reproduce on a still bench.

```bash
cd falcon_srcs && pio run -e brownout_test -t upload
```

⛔ **Bench/test build. The FSM is bypassed and NOTHING can release the beacon.**
Never leave it on the device for a real run.

| observation | verdict |
|---|---|
| survives 600 s+ in motion, `tw=` flat | vibration hypothesis dead; §5.7 closes |
| dies with `vcc_blast` << `vcc_quiet` | rail sag after all |
| dies with both flat, no reboot | **intermittent contact — hardware** |
| `Reset cause: 0x8` mid-alarm | TWI wedge, watchdog caught it |

**If B is clean:** raise `LATCH_FAILSAFE_MS` back toward 600 s (§7.3). It was
clamped to 240 s to stay inside a limit that now appears not to exist, and that
clamp is currently hiding the long-run case.

⚠️ **Re-flash the shipping build afterwards.** Confirm by flash size: shipping
is ~31416, `brownout_test` is ~21086.

---

### SESSION C — hoistway regression + data collection (~1–2 h in car)

**This is the regression run for everything done on 2026-08-14**, and it
collects the site data three separate open items need. Do the *whole* list in
one visit — the expensive part is getting into the hoistway, not the runs.

| step | what | exit criterion |
|---|---|---|
| C1 | **Power on at the counterweight, capture `XY: bmax`.** Repeat on 3+ mountings. | run `graph/calib_replay.py` over the logs. **6 s must not exceed the 10 s answer.** If it does, revert to 10 s / quorum 6 |
| C2 | **Watch one alert.** | **all eight ring LEDs sweep, D3→D10, no gap** (item 14) |
| C3 | **A normal multi-floor automatic run, both directions.** | departure caught, arrival releases, `ARM q=` recorded |
| C4 | **A single-floor terminal approach** (§5.2, item 4) | expected to still fail — record `pk=0.00` and the beacon duration. This is the reproduction, not the fix |
| C5 | **A short run starting near the bottom terminal** (§5.3, item 5) | `ARM q=` and `ro=` recorded. A long descent does NOT test this — arming completes near the top |
| C6 | **The low-speed long run** (§5.6, item 6) — never completed; both attempts died mid-run | needs B clean first, or it dies again |

**Deliberately NOT in scope for C:** fixing §4.2 or §5.2. C *characterises*
them. Changing the arming gate is session D work and must be replayed before it
is flown.

**Rollbacks, if the car misbehaves:**

| symptom | first move |
|---|---|
| beacon releases on a moving car | `RAMP_ARMED 0` |
| false jog release | `JOG_VERDICT_ARMED 0` |
| twitchy / never releases at the terminal | `CALIB_TIMEOUT_MS` 10000 + `XY_CALIB_BUCKETS` 10 + `XY_CALIB_MIN_BUCKETS` 6 |

---

### SESSION D — engineering, replay-gated (bench, no car)

Not testing. Listed so the order is explicit.

| # | work | gate |
|---|---|---|
| D1 | **Redesign the arming gate** for §5.2 — key on "the departure ramp has ended" rather than "the signal went quiet" | ⛔ must be replayed through `graph/arming_replay.py` against all 186 bursts before it goes in a car |
| D2 | **Have the device report its own arming margin** (§5.3) — a mechanic's placement decides whether the beacon releases, and commissioning cannot spot-check it | — |
| D3 | **Attack the 11.4 ms sensor read** (§5.5) — 28% of CPU, the dominant cost in the ~20% sample loss | every threshold on file stands on 80% of the data until this moves |
| D4 | **`RollingAvg` static buffers** — 586 bytes and the end of dynamic allocation | flash headroom is 840; `WIRE_TIMEOUT` wants 1198 |

---

## 3. Dependency order, compressed

```
A (bench, meter)  ──┬──> C (hoistway regression + data)
                    │
B (moving car)  ────┴──> C6 (low-speed long run) ──> LATCH_FAILSAFE_MS -> 600 s

D  independent, but D1 must be replayed before it is ever in a car
```

**If there is time for only one session: B.** It is 30 minutes, it is the
product's core use case, and it is the only open item that can kill the device
outright while a counterweight is moving.

**If there is time for only one bench hour: A1–A3.** Everything about the
battery alert is currently built on a number nobody has measured.

---

## 4. What "testing complete" would actually mean

Honest statement of the bar, because several items above characterise rather
than fix:

- §5.7 closed by measurement (B)
- battery trip point derived from a meter, chirp exercised (A)
- 6 s calibration validated on site, or reverted (C1)
- beacon visual confirmed in a real alert (C2)
- §4.2 / §5.2 / §5.3 **characterised and their fixes designed** — they are not
  closed by testing, they are closed by D1/D2 plus a further car session
- §5.6 still one shaft and one unit per configuration. **A second unit and a
  second building is the coverage gap no amount of testing in this shaft
  closes**, and it should be scoped separately.

⚠️ Items 2–5 sit in the **release path** — the direction that produces either a
position lie or silence while moving. None of them are closed by this plan;
this plan gets them measured well enough to design against.
