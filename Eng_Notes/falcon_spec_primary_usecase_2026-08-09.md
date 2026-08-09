# Falcon — specification, primary use case

**Date:** 2026-08-09
**Status:** design settled, nothing measured, nothing implemented.
**Scope:** the counterweight-placement use case ONLY. Reroping, hoisted
materials and the construction running platform are deferred until this one is
established.

This consolidates and supersedes the design threads in
`falcon_use_case_2026-08-09.md` and `falcon_zxy_logic_2026-08-09.md`, which
remain as the record of how it was arrived at, including the reversals. Where
they disagree with this document, this document wins. Section refs of the form
§n point at `falcon_analysis_2026-08-06.md`.

---

## 1. Deployment

1. Mechanic accesses the hoistway from the cartop
2. Operates down to the counterweight
3. Places the falcon on the cwt frame, **then powers on**
4. Falcon boots, calibrates **in place, at rest**, signals ready
5. Mechanic performs work or maintenance, operating the car as needed
6. Falcon alarms — audible and visual — whenever the counterweight moves
7. At the end of the job the mechanic returns and retrieves it

Two properties of this sequence drive the whole design:

- **Placement happens before power-on**, so the handling transient never
  reaches an armed detector. It is not a false-alarm source.
- **Calibration happens at rest on the actual counterweight in the actual
  building**, so the device can measure its own noise floor per deployment.

The mechanic operates on **inspection, which is continuous-pressure**, so
travel comes in short bursts separated by rest. This is the normal case, not an
edge case.

---

## 2. Requirement

> **The beacon is ON if and only if the counterweight is moving, with 1–3 s of
> lag in either direction.**

| | Budget |
|---|---|
| Set, after motion starts | 1–3 s (aim ~0.2 s; z detection is already this fast) |
| Reset, after motion stops | 1–3 s |
| False alarm duration | ≤ 3 s, self-clearing, no user action |
| Minimum burst duration detected | **to be established** — see §7 |

The alarm is a **beacon**: the mechanic ranges the counterweight by ear and eye.
Consequences:

- A beacon still sounding after the counterweight stopped is a **position lie**,
  as damaging as a false alarm.
- A beacon that drops mid-travel is worse than one that never sounded — the
  mechanic is actively tracking an approaching mass and the cue disappears.
- There is **no manual silence**: the device is on the counterweight and out of
  reach for the whole job.

---

## 3. Architecture

**Z sets. X/Y resets.**

```
CALIBRATION (10 s, at rest, in place)
      measure z zero  -> zero_calib_value
      measure x/y floor -> XY_STILL          (§4)
      self-test + fault registers            (§6)
      --> signal READY / NOT READY

MONITORING (silent, armed)
      z any-motion interrupt  -->  BEACON        (~0.2 s)

BEACON (audible + visual, minimum MIN_BEACON_MS)
      X/Y quiet on N consecutive polls  -->  MONITORING   (~1.5 s after stop)
      LATCH_FAILSAFE_MS                 -->  MONITORING   (fault, logged)
```

Why each axis does its half:

| | Z | X / Y |
|---|---|---|
| Gravity | 1 g pedestal | **none** |
| Cruise vs parked | identical — provably (§3) | vibration vs silence |
| Detects | transients: the start | **levels**: whether travel continues |

§14.7 showed a real arrival smaller than the parked noise floor on z, and §14
recorded **zero z any-motion edges during cruise** — so z can detect neither the
stop nor the absence of motion, and no timeout can substitute. X/Y is
load-bearing, not an optimisation.

**No confirm-before-sound.** An earlier design gated the beacon on X/Y
confirmation. It imposed a minimum movement *duration* — bursts under ~0.7 s
were silent — which is fatal when inspection operation is inherently bursty.
Deleted.

---

## 4. In-situ calibration of `XY_STILL`

During the existing 10 s calibration, with the device at rest on the
counterweight, accumulate x/y and derive the stillness threshold.

**Statistic: median of per-second maxima**, not the global maximum. One loud
second — another car passing, someone on the cartop — must not inflate it.

**Error direction is favourable, which is what makes automating this safe:**

| Estimate | Threshold | Consequence |
|---|---|---|
| Floor **under**-estimated | too low | never releases promptly → over-eager beacon → **annoying, safe** |
| Floor **over**-estimated | too high | travel reads as still → **releases mid-travel — dangerous** |

Short windows tend to underestimate a peak, so sampling error lands safely.
Only a sustained loud event during calibration pushes it the dangerous way,
which is what the robust statistic defends against.

**Refuse to arm if the calibration window shows movement.** A counterweight
that moved during those 10 s yields a threshold wrong in the dangerous
direction, and declining to arm is correct.

Open: 10 s at 3.13 Hz is ~31 samples, thin for a safety-critical threshold.
Whether the mechanic tolerates 20–30 s is a UX question.

---

## 5. Parameters

| Name | Value | Source |
|---|---|---|
| `ANYMOTION_THRESHOLD` | 32 counts = 0.153 m/s² | §12, unchanged; LSB confirmed 0.4883 mg/count |
| `ANYMOTION_DURATION` | 5 samples = 100 ms | unchanged; field ranges to 163 s if false fires appear |
| `XY_STILL` | **learned at calibration** | §4; conservative compile-time fallback |
| `NOMO_DURATION` | ~1000 ms | reset budget minus poll latency |
| `RELEASE_POLLS` | 2 consecutive | anti-dropout hysteresis |
| `MIN_BEACON_MS` | ~1500 ms | anti-flicker; also sets the minimum-burst floor |
| `LATCH_FAILSAFE_MS` | 300 s | unchanged; reaching it is a fault |
| `STOP_CONFIRM_MS` | **superseded** | X/Y handles levelling structurally — a levelling cwt is genuinely moving |

---

## 6. Ready / not-ready signalling

The only moment the mechanic is beside the device. It must carry the
calibration result, not just "booted":

| Signal | Meaning |
|---|---|
| Ready, floor clean | normal arm |
| Ready, floor noisy | armed, site is marginal, expect a twitchy device |
| **Not ready** | calibration failed, movement during calibration, or sensor fault |

This is where the BMA456 self-test (±1800 mg, `bma4_perform_accel_selftest()`,
already in the driver) and the fault registers `ERR_REG`, `EVENT.por_detected`
and `INTERNAL_STATUS` should surface. A boot self-test failure is only
actionable while someone is holding the unit.

---

## 7. Working range — what "established" means

Two axes, not one. The second is only visible because inspection travel is
bursty:

- **Minimum speed** — set by the z any-motion threshold
- **Minimum burst duration** — set by z detection latency and `MIN_BEACON_MS`

Test protocol must therefore include **deliberate short jogs — 0.25 s, 0.5 s,
1 s, 2 s — at two or more speeds**, not only continuous runs. Continuous runs
alone cannot establish the duration axis, and it is cheap to add.

Deliverable is a curve, not a threshold: set latency and reset latency against
speed, and false-alarm rate against building vibration.

---

## 8. Known risks, ranked

| # | Risk | Status |
|---|---|---|
| 1 | 🔴 **Buzzer holds X/Y above `XY_STILL` while sounding** → beacon never releases → every run to failsafe. §11 measured the piezo triggering any-motion continuously on z at threshold 96; the release threshold is far lower, on axes with no 1 g pedestal | **Cheapest test, most likely to invalidate the design. Needs only a bench.** Fix if real is mechanical isolation of piezo from sensor |
| 2 | Smooth counterweight: X/Y dips below `XY_STILL` mid-cruise → beacon drops mid-travel | Mitigated by `MIN_BEACON_MS`, N-poll hysteresis, and z re-triggering at rail joints. Not solved |
| 3 | Building vibration exceeds `XY_STILL` → slow release, over-eager beacon | Safe direction. Costs trust, not safety |
| 4 | Sub-threshold burst too short to detect | Bounded by §7, not yet quantified |

The ⛔ stillness-backstop block in `movement_service.h` forbids a rest detector.
Its numbers are all z and its argument is the 1 g pedestal, which does not
transfer to gravity-free axes — **but it was written after a real mid-ride
release, and risk 2 is that failure returning by another route.** If the X/Y
contrast measures marginal, that block wins.

---

## 9. Dependencies

| Need | Why |
|---|---|
| **Bosch driver variant swap** | Independent any-motion (z) and no-motion (X/Y) engines with separate thresholds, axis masks and interrupt lines. The vendored 2018 Seeed driver makes them mutually exclusive. Also frees ~4.9 KB with flash at 86.3% |
| INT2_ACC → PD3 | Confirmed wired and unused (§11) |
| Sample rate / 100 Hz | **Not** on the critical path any more. Wanted for the velocity integral and for risk 1 if isolation is not possible |
| Roadmap 11, on-device black box | Still the only safe way to characterise on a counterweight (§5.1) |

**Demoted:** the windowed velocity integral (`velocity.h`) has no role in the
control path under this design. It stays unarmed as instrumentation for
establishing the working range.

---

## 10. Open questions

- [ ] **Risk 1** — does the buzzer prevent X/Y from ever reading still? Bench,
      no hoistway, build already compiled
- [ ] Parked-vs-travelling X/Y contrast on a counterweight
- [ ] Should repeated short bursts hold the beacon, or drop between them? A
      beacon chattering during inching may read as malfunction — a UX call
- [ ] Calibration length: 10 s or 20–30 s?
- [ ] Is the DPS310 permanently depopulated? It would make constant velocity
      directly observable and give an independent stop signal
