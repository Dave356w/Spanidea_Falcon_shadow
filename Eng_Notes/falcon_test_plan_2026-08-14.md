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
| 1 | ~~**§5.7 vibration hypothesis**~~ — **CLOSED 2026-08-14: the deaths were a COM-link lockup, not the device.** See §1.3 | ✅ closed, was #1 |
| 2 | **§4.2** — 34 automatic stops release on a distribution centred on the gate; worst margin **1.009×** | measured, unfixed |
| 3 | **§5.1** — ramp detector (14 latches, never executed) and reversal arming (`v=2`, never fired in ~32 runs) | armed, never observed working |
| 4 | **§5.2** — single-floor terminal approach: `pk` reads 0.00 through a real excursion, beacon sounded **85 s** over a car stationary for 78 | reproducible, unfixed |
| 5 | **§5.3** — arming margin spans `q=6…15` on nominally identical mountings, `ro=8` once (zero) | measured, unfixed |
| 6 | **§5.6** — one shaft and one unit per configuration; 500 fpm is n=3 | coverage gap |

### 1.3 🟢 §5.7 CLOSED — the brownout was the COM link

**Established at the bench 2026-08-14: the ~4-minute deaths during sustained
alarms were a COM-link lockup, not a device failure.**

This retires the item that has sat at the top of this project since 2026-08-11,
and it retires both competing explanations at once — the rail-sag hypothesis and
the intermittent-contact-under-vibration hypothesis were arguing about a failure
that was in the measuring apparatus.

It also fits facts that never sat comfortably with either: the bench procedure
already documents that the USB-serial cable back-powers the board through the RX
ESD clamp, that pulling cells with the cable attached is not a power cycle, and
that "monitor handle-drop plus avrdude-signature-read revival is routine." The
serial link was known to be an active participant in the power and reset
behaviour of the board. It was not suspected as the cause of the deaths.

#### What this unblocks

| | |
|---|---|
| **`LATCH_FAILSAFE_MS`** | ✅ **DONE — raised 240 s → 600 s, flashed 2026-08-14.** The clamp existed only to stay inside a 243 s limit that does not exist. ⚠️ It is the backstop that ends a stuck beacon, so a §5.2 position lie can now run to 600 s rather than 240 s. Accepted deliberately: a position lie destroys trust, silence over a moving counterweight is the failure the product exists to prevent. **Not a licence to stop fixing §5.2 — D1 is what closes it** |
| **The low-speed long run (§5.6, item 6)** | "never completed, because both attempts were abandoned when the device stopped." If the stopping was the COM link, **the test is runnable now** — with serial disconnected, or with the link fixed |
| **Beacon audibility** | see below — the largest product consequence |

#### ⭐ The beacon was made quieter to defend against a phantom

On 2026-08-11 the piezo duty was cut 2/5 → 1/5 **specifically** to attack the
brownout, knowingly costing audible beacon length — and the mechanic ranges the
counterweight **by ear**. The sequence-aligned rework on 2026-08-13 then took
the current 150 ms blast per 800 ms (18.75%) under the same constraint, with
`alarm.h` explicitly reasoning that a naive louder pattern "runs straight into
the 243 s brownout."

**That constraint is void.** The beacon can be made louder and longer.

⚠️ **But it is not free, and the reason has nothing to do with the battery.**
Piezo duty trades directly against **sensor coverage**: every millisecond the
piezo is driven, plus `BUZZER_RINGDOWN_MS`, is a millisecond the accelerometer
is blanked, and the FSM must hear the arrival transient *while the beacon is
sounding*. The current numbers are 25% blanked and a 600 ms contiguous quiet
stretch. `alarm.h` carries the table.

🔴 **And one figure from that history still binds.** The 2026-08-07 false release
— a beacon released 12 s into a ride — was caused by raising the QUIET fraction
to 0.75, which let cruise any-motion edges pair up and satisfy the arrival
cluster. The present timing sits at exactly 0.75 and is safe only because both
arrival paths now also require `arrival_peak_hit()`. **Any change to the blast
must be re-checked against that**, not just against loudness.

**Recommendation:** treat beacon audibility as a deliberate re-open with its own
measurement, not as a revert. The right question is "how much blast can we buy
before the listening window costs us an arrival?", and that is answerable by
replay against the arrival bursts on file before anything is flown.

### 1.2 Added or left open by 2026-08-14

| # | item | status |
|---|---|---|
| 7 | ~~Battery trip point uncharacterised~~ — **✅ CLOSED.** Divider metered 2:1; thresholds derived in pack mV; chain validated against a meter to within 1 LSB | ✅ closed |
| 8 | ~~ADC prescaler `/128`~~ — **✅ CLOSED.** `/16` = 62.5 kHz, the slowest in-spec option (a faster one would be worse: 250 kOhm source). Removed a measured 0.8% low bias | ✅ closed |
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

#### ✅ A1/A2/A3 CLOSED — the chain is validated end to end

```
meter at the pack   5.010 V
firmware pack_mv    5012        (2 mV apart -- inside one LSB of 6.06 mV)
```

One comparison validates the divider ratio, the 3100 mV reference assumption and
the ADC configuration at once, because an error in any of them would surface
here. **The chirp can now be presented to a customer as a calibrated warning** —
the prohibition that blocked it is lifted.

The prescaler fix did real work, not just datasheet compliance: at `/128` the
same chain read 4860 against a 4900 meter, **0.8% low**, exactly the direction an
unfinished sample-and-hold charge errs from a 250 kOhm source.

**Remaining in Session A:** A5 quiescent current (shipping build, **cable
disconnected**), A6 efuse read, A4 hear the chirp, A7 double wink.

#### Historical: the prior that A1 confirmed

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

## A1 RESULT — divider confirmed, and a retraction

**Divider is 2:1**, metered and cross-checked: firmware `pack_mv` 4860 against
4.9 V at the test points — 0.8% apart, or 0.2% once the bandgap-measured 3120 mV
rail replaces the assumed 3100. Pack is **3 × AAA, 4.9 V fresh**.

So the historical thresholds were right all along: 1600 and 1750 at 2:1 are
exactly 3.2 V and 3.5 V, which is what `Release.txt` has claimed since V1.2.
`BATT_R1`/`BATT_R2` were the V1 leftover that made 1600 look like a 7 V pack.
**Item 7 closes.**

### ☠️ RETRACTED WITHIN THE HOUR: "the pack sags to 3.3 V under alert load"

One reading of 3.3 V "in alert" was written up as a 1.6 V sag, and a whole
causal story was built on it — that the sag explained the brownout, that the
buck was riding 100 mV from dropout on fresh cells, that the vibration and rail
hypotheses had merged, and that battery sampling had to be suppressed during
alerts to stop false alarms. A code change went in on that basis.

**The next measurement — PCB rails during an alert — showed a stable 4.9 V.**
There is no sag. Every conclusion above is withdrawn and the code change is
reverted.

What makes this one worth recording is that the contradicting evidence was
**already on file and went unchecked**: the 588 s endurance test compared VCC in
the blast phase against the quiet phase across 583 samples and found a mean
difference of **+0.6 mV**. A rail that does not move under alarm load is
incompatible with a pack sagging 1.6 V. The story should not have survived
first contact with the existing data, let alone reached a commit.

That is the eleventh interpretive claim overturned by the next measurement in
four sessions, and it took under an hour end to end. The 3.3 V reading itself
remains unexplained — most likely a different test point, plausibly the
regulated rail — but it is no longer load-bearing, so it is not worth chasing.

### What survives

- The divider, the derived trip point, and item 7 closing.
- **The lead-time problem, now cleaner.** With no sag, quiet and in-alert
  readings are the same number, so a 3.2 V trip fires essentially at dropout and
  the mechanic's first warning arrives as the device stops working. Raising
  `VBATT_LOW_MV` to ~3600 buys real runway. Still Dave's call.

### What goes back to how it was

🔴 **The brownout is unexplained again, and §5.7 is restored in full.** The
588 s test remains stationary-only, every historical death was in a moving car,
and the intermittent-contact-under-vibration hypothesis is once more the leading
candidate. **Session B is unchanged from its original form** — its question is
"vibration or not?", exactly as written below. Take a meter to the pack while
you are there anyway; it costs nothing and this session showed how far a single
unconfirmed reading can travel.

---

### SESSION B — ~~endurance in a moving car~~ ✅ CLOSED, NOT NEEDED

**Dave established on the bench that the ~4-minute "brownout" was a COM-link
lockup, not the device.** The serial connection was the failure, not the pack
and not a contact.

This was **item 1 — the highest-priority open item in the project since
2026-08-11**, and the whole session existed to run `brownout_test` in a moving
car to separate a rail sag from an intermittent contact. Both hypotheses are
moot: there was never a device failure to explain.

**Do not run this session.** If a long-alarm endurance figure is ever wanted for
its own sake, run it **with serial disconnected** and judge it by ear and by the
LEDs — the instrument was the fault, so instrumenting it the same way would
reproduce the artifact rather than the behaviour.

⚠️ **The 588 s "successful" endurance test is also reinterpreted.** It did not
demonstrate that the device survives a long alarm *when held still*; it
demonstrated that the COM link happened to hold that time. It was never evidence
about the pack in either direction.

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
A (bench, meter)  ──> C (hoistway regression + data)
                                  │
B  CLOSED (§1.3) ─────────────────┴──> C6 (low-speed long run), now unblocked

LATCH_FAILSAFE_MS -> 600 s   DONE 2026-08-14
D  independent, but D1 must be replayed before it is ever in a car
```

**Session B is closed (§1.3), so C now leads.** It is the regression run for
2026-08-14 and it collects the site data three separate items need.

**If there is time for only one bench hour: A1–A3.** Everything about the
battery alert is currently built on a number nobody has measured.

---

## 4. What "testing complete" would actually mean

Honest statement of the bar, because several items above characterise rather
than fix:

- ✅ §5.7 closed — it was the COM link (§1.3)
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
