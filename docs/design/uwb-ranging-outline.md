# Falcon — Ranging-Based Hazard Alert for Elevator Hoistways

**Clean-sheet outline · Rev A · 16 Aug 2026**

Status: concept, pre-prototype
Supersedes: the accelerometer + barometer approach in `falcon_srcs/src/movement_service.cpp`

Replacing accelerometer-derived motion inference with direct UWB distance measurement
between a hazard-mounted unit and a worker-carried unit. Secondary alert device, battery
on both ends, no interface to the permanent installation.

> All quantitative figures in this document are estimates pending bench confirmation in
> Phases 1–2.

---

## 1. Why the approach changes

The current firmware estimates velocity by integrating proper acceleration. That is an
ill-posed problem in a body frame of unknown attitude, and the existing source carries the
scar tissue:

- continuous re-zeroing of the gravity offset (`movement_service.cpp:117`, `:136`)
- a velocity-limit bailout into `ERROR_RESET` (`:48`)
- two independent variance heuristics — accelerometer and barometric — that must both
  agree before a state transition is trusted (`:185-230`)
- `vel_threshold` derived from whatever velocity happened to exist 400 ms after trigger,
  then doubled (`:85-97`)

None of that is poor engineering. It is the best available engineering when the only
observable is a second derivative. The limitation is **observability, not tuning**, and no
amount of filter work removes it.

Direct ranging changes the order of the problem. Distance is a zeroth-order observable;
its first derivative is velocity, measured rather than integrated. No drift, no gravity
coupling, no error-reset path.

In a hoistway the geometry is additionally favourable: motion is one-dimensional and
rail-constrained, so a hazard unit travelling along the rail moves *directly* along the
line to the worker unit. The cosine loss that would afflict this method on free-roaming
equipment is exactly unity here.

---

## 2. System concept

Two battery-powered units, peer-to-peer, no fixed infrastructure and no third device. The
asymmetry between them is driven by **how often a human can physically reach each one** —
not by power availability, since neither has mains.

### Hazard unit — Responder / Controlee

| | |
|---|---|
| Mounts | Counterweight frame, or any moving hazard |
| Access | Effectively unreachable once installed |
| Power | Primary cell, 5-year target |
| Does | Wakes, answers ranging, sleeps. No decisions. |

### Worker unit — Initiator / Controller

| | |
|---|---|
| Mounts | Car top, work platform, or worn |
| Access | Carried; charged with the mechanic's other tools |
| Power | Rechargeable, multi-shift runtime |
| Does | Owns schedule, computes closure, raises the alarm |

### Naming

Drop "beacon" and "node" before they reach the firmware. "Beacon" means one-way BLE
advertising and describes the opposite of what the hazard unit does; "node" reads as
infrastructure when the device is actually the mobile, intelligent end.

Equally, avoid *anchor* and *tag* — that is RTLS vocabulary, and it will steer both the
team and any vendor SDK toward a trilateration stack built on assumptions that are all
false here:

| RTLS assumes | This system |
|---|---|
| Three or more anchors for a position fix | Exactly one pair |
| Anchors at surveyed, known positions | No survey; geometry changes daily |
| Anchors are mains-powered infrastructure | Both ends on battery |
| Output is absolute position | Output is one scalar range and its derivative |
| Commissioned installation | Zero-config, tool-free, magnet-mounted |
| The fixed device is the reference | The fixed-role device is the one that moves |

The correct request to put to a vendor is **peer-to-peer two-way ranging between an
initiator and a responder**. That returns the right example code; "anchors and tags"
returns an RTLS stack with a position solver, network coordinator and gateway that will
take months to delete.

---

## 3. Deployment modes

Both modes run identical hardware and firmware. They differ only in what is mounted where,
and the design must not require the units to know which mode they are in.

### Service — existing installation

Hazard unit on the counterweight, worker unit on the car top. Car and counterweight are
roped over the sheave and travel in opposite directions, so relative velocity is twice car
speed — a free doubling of sensitivity. More importantly this measures the real hazard
geometry: the closing separation between the counterweight and the person standing on the
car, with a passing clearance typically in the 100–300 mm range.

### Construction — temporary platform

No permanent car, rails installed only as high as the work has reached, and the geometry
different every day. The counterweight may not yet exist; the live hazards are the
temporary hoist, hoisted materials, and — the highest-value case — an **adjacent live car
in a multi-hoistway bank**, which is a recognised fatality mode and one this system
addresses directly.

The consequence for the design is that there is no static reference. **Do not attempt to
determine which unit moved.** Measure relative closure only; it is the safety-relevant
quantity in both modes, and it removes any need for site survey or commissioning — which
would never survive contact with a construction site anyway.

---

## 4. Link architecture

### Radio

UWB two-way ranging, on a module with an integrated host MCU and BLE — the Qorvo DWM3001C
class of part (DW3110 + nRF52833 + PA/LNA + antenna, pre-certified). That single choice:

- collapses the UWB-for-measurement / BLE-for-control split into one component
- puts deep sleep in the low microamps
- inherits modular FCC/IC/CE approval, which for a low-volume safety accessory is months
  and tens of thousands of dollars avoided

Verify current lifecycle and pricing against Qorvo's QM33 generation before committing —
UWB module availability moves quickly. One firm exclusion: **do not design in DW1000 /
DWM1001**. Previous generation, higher power, not FiRa-compliant, approaching end of life.

**Rejected alternatives.** RSSI on BLE or sub-GHz cannot resolve vehicle-scale motion in a
steel-lined shaft, which is close to a worst-case multipath environment. BLE Channel
Sounding is materially more multipath-sensitive than UWB's leading-edge detection and
should be revisited only as a later cost-reduction path, once ranging behaviour is proven.

### Ranging scheme — single-sided TWR with CFO correction

The default reflex is double-sided TWR, which cancels clock-frequency offset at the cost
of a third message. Single-sided TWR is the better fit here, for two reasons.

**First**, uncorrected SS-TWR error is:

```
ToF   = (T_round − T_reply) / 2
error ≈ (clock_offset × T_reply) / 2
```

At 20 ppm over a 1 ms reply delay that is roughly 3 m. But the receiver's carrier
integrator estimates that offset to about 0.1 ppm, dropping residual error to roughly
1.5 cm.

**Second, and decisively**: the residual is a *slowly varying bias*, drifting on thermal
timescales while the measurement window is about one second. It is common-mode across the
window and **cancels in the slope**. SS-TWR's principal weakness is nearly invisible to a
rate-based alarm.

The payoff lands exactly where it is needed. SS-TWR is two radio operations per end
against three for DS-TWR, so double-sided costs **1.5× the energy on the responder** — the
primary-cell unit nobody can reach for five years.

> **Consequence worth banking.** The same reasoning removes end-of-line antenna delay
> calibration. It is the largest absolute bias term (±30 cm uncalibrated) and normally a
> per-unit production step — but it is constant, so it cancels in rate. Ship one nominal
> per-design calibration. The bias survives only in the absolute range term of
> time-to-contact, where 30 cm does not change any decision.

### Rendezvous

The worker unit initiates; the hazard unit opens narrow scheduled RX windows. This covers
both directions of closure — including a platform closing on a stationary counterweight,
where the hazard unit's own accelerometer would feel nothing and could never self-wake.

After long separation the clocks drift apart, so a re-acquisition path is required: the
hazard unit periodically opens a wide window, and the worker unit transmits a long-preamble
wake at arm time. That energy is paid once per session.

FiRa defines **one-to-many ranging** — a single controller running a round-robin block
against multiple controlees — so the bank-of-elevators case is a supported session type
rather than something to invent. This is the strongest argument for staying on
FiRa-compliant silicon.

---

## 5. Alarm logic

### Time-to-contact, not distance

Inspection speed is capped near 0.75 m/s, but the adjacent-live-car case runs at 2–8 m/s.
A fixed distance threshold that is sensible at inspection speed is uselessly late at line
speed. Use `range ÷ closing_rate`, with a graded response — advisory at roughly 10 s,
urgent at roughly 4 s. It self-scales across the whole speed range.

### Rate estimation

Never two-point differentiate. At 10 Hz with 2 cm range noise that yields about 0.28 m/s
of rate noise — worse than the speeds being detected. A least-squares slope over a window
collapses it:

| Window | Samples | Rate noise (1σ) | Verdict |
|---|---:|---:|---|
| 1 s @ 10 Hz | 10 | ~1.3 cm/s | Ample margin |
| 3 s @ 1 Hz | 3 | ~0.8 cm/s | Ample margin, higher latency |

Precision therefore does not set the update rate — **latency does**. Alarming at 4 s TTC
requires a converged estimate within about a second of motion starting, which is what
argues for 10 Hz *while something is closing*, and nothing like it otherwise.

### Suppressing expected closure

If the mechanic is driving the platform, closure is intentional, and alarming on it will
get the device switched off within a week. **Alarm fatigue is the single most likely cause
of field failure.** The worker unit's accelerometer gives a clean rule:

- **Worker unit stationary + range closing** → alarm, full priority
- **Worker unit moving + range closing** → advisory, or silent

This is where the existing accelerometer work is retained — demoted from velocity
estimation, which it does badly, to context and wake, which it does well.

### Link loss

In a shaft, occlusion and departure both present as link loss, and fail-safe argues for
alarming on both. Alarming immediately, however, produces a nuisance alarm on every rope
swing and body block. Required behaviour is a short hold-off plus dead reckoning from the
last good rate, escalating to alarm only if the link does not recover.

---

## 6. Power budget

Ranging energy is roughly 0.2–0.5 mJ per exchange, with wake and PLL settle typically
dominating the frame itself.

> **The constraint that shapes the design.** A flat 4 Hz poll while armed does *not* meet
> the five-year target. At 8 h × 250 shifts that is roughly 8.6 kJ/year of ranging alone
> against about 33.7 kJ in an ER14505 — under four years before sleep current and before
> derating. **Accelerometer-gated adaptive rate is mandatory, not an optimisation**, and it
> must be designed into the session structure from the start.

| Hazard unit state | Poll rate | Rationale |
|---|---:|---|
| Disarmed | — | Deep sleep, ~5–10 µA; accelerometer wake-on-motion only |
| Armed, hazard stationary | 0.5 Hz | Presence and link health; covers most of any shift |
| Armed, hazard moving | 10 Hz | Full closing-rate tracking at alarm latency |

At roughly 10% motion duty this lands near 3.1 kJ/year — about ten years theoretical,
comfortably five to six derated. The budget closes, but only with gating.

**Arming is the second lever.** An explicit arm-on-inspection step with auto-disarm after a
set period takes the disarmed-state drain down to effectively shelf life. It is also
operationally natural: the mechanic arms the system when work starts.

The worker unit is unconstrained by comparison — 1–3 mA armed on a 2000 mAh cell gives
multiple shifts per charge with headroom for the sounder and any display.

---

## 7. Hardware

Build both ends on **identical modules** for the prototype and differentiate purely in
firmware by role flag. One design, one image, and roles can be swapped on the bench while
the schedule ownership question is still open. Cost-reduce the hazard unit later, once
ranging behaviour is settled and it is clear what can be stripped.

### Antenna is the top hardware risk

A device magnet-mounted onto steel in an arbitrary attitude, knocked off and re-mounted
several times a day, makes antenna pattern a first-order problem rather than a detail. UWB
antennas are meaningfully directional, and mispointing costs both range and first-path
accuracy — compounded on a hazard unit partly shadowed by a rail bracket.

A module variant with an **external u.FL connector** may beat one with a fixed integrated
antenna, because it decouples antenna placement from where the PCB sits in the enclosure.
Evaluate both before the mechanical design is locked.

### Mounting constraint, counterweight rail

If a rail-mounted variant is pursued: the counterweight's guide shoes sweep all three
machined rail faces over the entire travel. Anything on a running face is sheared off.
Mounting is restricted to the rail back, web, or a bracket — which is precisely where steel
shadowing is worst. This is a substantive argument for preferring the car-top topology.

### Cell and pulse response

Li-SOCl₂ gives the shelf life and temperature range wanted for the hazard unit but has poor
pulse response; UWB TX bursts will sag the terminal voltage. A hybrid-layer capacitor or
supercapacitor buffer is required. Budget it early — it is a common late-stage surprise.

### Environment

Roughly −10 to +60 °C, hotter near the machine room. Rail lubricant, dust, condensation,
continuous vibration. RF is untroubled by grime; the real risks are mechanical sealing and
magnet retention under sustained vibration.

---

## 8. Firmware

The integrated module brings its own Cortex-M4, so this ends the ATmega328P and moves
firmware to the Nordic SDK or Zephyr. **That is a full port, not an incremental change**,
and should be priced into the schedule honestly.

| Component | Disposition | Note |
|---|---|---|
| `MovementService` state logic | Survives | Plain C++ float math; ports cleanly, and shrinks substantially |
| `RollingAvg` | Survives | Reused for the range window |
| `ERROR_RESET` path | Deleted | Exists only to catch integration divergence |
| DPS310 barometer + pressure variance | Deleted | Exists only to cross-check accelerometer drift |
| Arduino / AVR HAL, `main.cpp` | Rewritten | Full platform change |
| BMA456 driver | Rewritten | Zephyr has BMA4xx support |
| Alarm output, battery sense | Rewritten | New pin map and power topology |
| Ranging session, scheduling, sleep | New | The genuine engineering effort |

### Signal processing notes

- **NLOS bias is one-sided.** Blocked or reflected first paths always read *longer*, never
  shorter. A single outlier can fake a receding trend or mask a closing one, so fit the
  window with a median or RANSAC estimator rather than plain least squares. Cheap, and it
  matters far more here than in open air.
- **Tune leading-edge detection early.** First-path detection parameters are the difference
  between UWB working in a steel shaft and not. Start at a 128-symbol preamble; a shaft
  waveguides, so range may exceed free-space expectations and preamble length can likely be
  traded back for energy once measured.
- **Channel.** Ch5 (6.5 GHz) propagates better through a steel shaft than Ch9 (8 GHz), but
  regional regulatory rules differ. Confirm against target markets before locking, since it
  drives antenna design.

### Configuration

No dedicated base station. Pairing, thresholds, arm/disarm, log offload and firmware update
all run over BLE from the mechanic's phone — which is what the nRF52833 in the module is
for. A fixed site unit earns its place only if supervisor visibility on a construction
project is wanted later, and that is telemetry, not safety path. Keeping it out preserves
the property that matters most: a zero-config personal safety accessory rather than a
commissioned system.

---

## 9. Risk register

| Risk | Sev | Mitigation |
|---|---|---|
| Multipath in a steel shaft degrades first-path detection beyond usable | High | Phase 1 exists to answer exactly this, before any hardware spend |
| Alarm fatigue from nuisance alarms; device gets switched off | High | Motion-gated suppression rule; link-loss hold-off; TTC rather than distance |
| Antenna orientation sensitivity under arbitrary mounting | High | Characterise early; u.FL variant; possible diversity antenna |
| Five-year hazard-unit life not met | Med | Adaptive gating designed in from the start; arm-on-inspection |
| Required sensitivity is millimetre-scale releveling creep | Med | Open question Q2 — would invalidate the approach; resolve before Phase 2 |
| Module lifecycle or supply discontinuity | Med | FiRa-compliant parts only; second-source assessment at Phase 4 |
| Scope creep toward a certified safety device | Low | Section 13 boundary held explicitly in product and documentation |

---

## 10. Phased plan

Each phase ends at a gate that can stop the project cheaply. The ordering is deliberate:
the cheapest experiment that could invalidate the whole approach runs first.

### Phase 1 — Feasibility in a real hoistway

Two off-the-shelf dev kits, no custom hardware. Log raw range at 10 Hz in an actual shaft
while simultaneously logging the existing accelerometer stream. Run the physical test set
that currently gives the accelerometer approach trouble. Replay offline through the
existing `falcon_srcs/simulation/` harness and `falcon_srcs/graph/plot.py`.

Bill of materials, instrumentation approach and firmware path are in Section 11.

> **Gate** — does Δrange separate bump from real closure on the exact runs where the
> current state machine fails? Characterise multipath, NLOS outlier rate, and usable range
> over full travel. **If this fails, stop here.**

### Phase 2 — Ranging session and power model

Implement SS-TWR with CFO correction, adaptive accelerometer-gated rate, and the sleep/wake
schedule. Measure true energy per exchange and per state on the bench. Validate the rate
estimator and outlier rejection against Phase 1 recordings.

> **Gate** — measured hazard-unit budget projects to five years under a realistic shift
> profile. Rate noise and latency meet the TTC thresholds.

### Phase 3 — Alarm logic and field trial

TTC thresholds, motion-gated suppression, link-loss behaviour, one-to-many scheduling for
the bank case. Trial in both deployment modes with working mechanics, instrumented to count
nuisance alarms rather than relying on recollection.

> **Gate** — nuisance alarm rate low enough that mechanics leave the device armed for a
> full shift. This is the real product risk and it is measured, not assumed.

### Phase 4 — Productisation

Custom PCB, enclosure and magnet mount qualified under sustained vibration, cell and buffer
selection, phone app for configuration and log offload, regional regulatory confirmation,
second-source assessment.

> **Gate** — pilot units survive a full service season without mechanical or power failure.

---

## 11. Phase 1 bill of materials and instrumentation

Prices and stock as researched 16 Aug 2026. UWB module lead times are volatile — reconfirm
before ordering.

### Hardware — 2 × Qorvo DWM3001CDK

Approximately **$29.50 each at DigiKey**, in stock. Under $60 for the pair, and it is the
same silicon this document specifies for production, so Phase 1 measurements carry into
Phase 2 without revalidation on different hardware.

Confirmed against the DWM3001CDK Product Brief (Rev. B, May 2022) and the DWM3001CDK Quick
Start Guide (QM33SDK-1.0.0, Aug 2024):

| Requirement | How the DWM3001CDK meets it |
|---|---|
| UWB ranging on target silicon | DWM3001C = DW3110 (non-PDoA) + nRF52833 with Bluetooth 5.2 |
| Battery operation | Dedicated battery connector with VBAT rail into the onboard DCDC; also 2 × micro-USB, Raspberry Pi header, external supply |
| Debug and capture | Onboard J-Link OB providing SWD and UART; second USB direct to the DWM3001C |
| Both candidate channels testable | Supports UWB bands 5 (6.5 GHz) and 9 (8 GHz) — settles the Section 8 channel question on real hardware |
| Phase 2 power measurement | **Dedicated header on the VCC trace for measuring module current** — directly serves the Phase 2 gate |
| Expansion | 26-pin Raspberry Pi compatible header, access to all DWM3001C GPIOs, GPIO test points |

**Order four, not two.** Mouser flags a long lead time on the part despite immediate stock.
Spares cost ~$60 total and protect against a multi-month stall; a third unit also exercises
the one-to-many bank-of-elevators case without reordering.

### Unresolved: is there an accelerometer on board?

**Treat this as unconfirmed and settle it before ordering.** Qorvo's product page describes
the DWM3001C as having an integrated motion sensor, but neither the DWM3001CDK Product Brief
nor the Quick Start Guide mentions one. The Product Brief's key-feature list enumerates
module-level detail (the nRF52833, both RF bands, the two integrated antennas) and its
functional block diagram enumerates board detail down to the ground point and GPIO test
points — an accelerometer appears in neither. Both documents do treat the DWM3001C itself as
a single block, so an in-module part would not necessarily be drawn. The definitive source is
the DWM3001C *module* datasheet, not the kit documentation.

This does not change the board selection, because the contingency is cheap: the kit exposes
all DWM3001C GPIOs plus a 26-pin Raspberry Pi header, so an I²C accelerometer breakout can
be added to each unit if needed — and the team already has BMA456 driver experience. But the
Phase 1 gate requires logging range *and* acceleration on the same runs, so this must be
known before the parts order rather than discovered during bring-up.

### Instrumentation — where the data lands

The counterweight unit cannot be tethered to a laptop. The instinct is to add SD logging to
both ends and reconcile two files afterwards. **Do not.**

In SS-TWR the *initiator* computes the range. Make the worker unit the initiator, sit it on
the car top tethered to a laptop, and every range measurement lands there already. The
counterweight unit then needs battery and nothing else.

That leaves the counterweight's accelerometer stream, which is wanted because the current
state machine runs on that equipment (subject to the accelerometer question above — if the
module has none, an I²C breakout on the GPIO header supplies it). **Piggyback accelerometer
samples in the TWR response payload.** There is room in the frame, it costs almost nothing,
and both streams land in one file that is inherently time-aligned — no clock
synchronisation, no post-hoc correlation, no second log to reconcile. Build it this way from
the first run.

### Diagnostics to capture from day one

Characterising multipath is the whole point of the Phase 1 gate, and these runs are
expensive to recreate:

- Raw CIR from register `0x15` — 992 samples at 16 MHz PRF, 1016 at 64 MHz
- `dwt_readaccdata()` and `dwt_readdiagnostics()` for first path index (`FP_INDEX`), first
  path amplitude (`FP_AMPL`) and `maxNoise`

### Firmware path

**Boards ship blank.** The Quick Start Guide is explicit that DWM3001CDK boards are not
shipped preprogrammed — the SEGGER J-Link Software and Documentation Pack must be installed
and UCI firmware flashed via J-Flash Lite before anything runs.

- **[Uberi/DWM3001C-starter-firmware](https://github.com/Uberi/DWM3001C-starter-firmware)**
  to get moving quickly — community-maintained, Dockerised build, runs directly on the
  onboard nRF52833, and considerably simpler than the official SEGGER Embedded Studio flow.
- **Official DW3_QM33_SDK** (v1.1.1, Aug 2025, FiRa 2.0 compliant) for CFO correction and
  the diagnostics registers.

### Calibration — mandatory, with a trap

Section 4 argues that per-unit antenna delay calibration can be skipped in *production*,
because a constant bias cancels in the rate. That reasoning still holds. It does not,
however, apply to the dev kits in Phase 1, and the Quick Start Guide is unambiguous:
calibration is **mandatory** on first use of a kit and again whenever the SDK is upgraded,
since calibration data is not guaranteed compatible across firmware versions.

It is also cheap. A preset calibration file ships for this exact board
(`.../calib_files/DWM3001CDK/dual-hoe_non_aoa.json`), and the Qorvo One TWR GUI has an
**auto-calibration** feature that sets antenna delay automatically and is documented as
highly recommended. Run it — Phase 1 then yields trustworthy absolute range for the
time-to-contact term as well as clean rate data.

> **The trap.** Calibration lives in NVM and survives power cycles and firmware updates, but
> **a chip erase silently invalidates it** and it must be reapplied. Because the resulting
> error is a constant bias, it will *not* appear as noise in the rate data — it surfaces as
> an unexplained step in absolute range between runs, which is an expensive artifact to
> chase. Record the calibration state alongside every capture.

### Considered and rejected for Phase 1

| Option | Why not |
|---|---|
| Qorvo QM33120WDK1 | Newer QM33 silicon, AoA daughterboard and *external antennas* — the only one of these that answers the antenna-orientation risk. But bulky nRF52840 DK boards are awkward to battery-mount on a counterweight, and it is materially more expensive. Buy it for Phase 2/4 antenna and production-silicon work, not for the go/no-go. |
| Makerfabs ESP32 UWB DW3000 | Arduino-native and fits the existing PlatformIO workflow, but the DW3000 Arduino library is third-party and aimed at simple ranging demos — no CFO correction, no diagnostics registers. No onboard accelerometer. Choosing familiar tooling over the measurements the phase exists to collect. |
| MDEK1001 / DWM1001 | DW1000 generation. Already excluded in Section 4. |

### Known gap

The DWM3001C carries **two integrated antennas (BLE and UWB) with no u.FL connector**, so it
cannot answer the antenna-orientation question listed as a top-tier risk in Section 9. That
requires the QM33120WDK1 daughterboards or a chip-down test fixture, and belongs to Phase 2.
The DW3110 is additionally **non-PDoA**, confirming that no angle-of-arrival work is possible
on this kit — consistent with Section 7, which rejects AoA for this application anyway.

---

## 12. Open questions

These drive architecture and should not be guessed at. Q2 in particular can invalidate the
entire approach and is cheap to answer now.

**Q1 — Is the hazard unit semi-permanent or per-job?**
Everything in this outline assumes semi-permanent. If mechanics mount and retrieve it each
visit, both the power model and the mount change fundamentally.

**Q2 — What is the minimum detectable motion?**
At roughly 5 cm, UWB is comfortable. If millimetre-scale releveling creep must be caught,
no RF ranging method reaches it and the approach needs rethinking. Resolve before Phase 2.

**Q3 — In construction, before the counterweight exists, which hazard is being
instrumented?**
Temporary hoist, hoisted materials, adjacent live car, or the platform itself. Determines
how many hazard units ship and where they mount.

**Q4 — Where does the alarm need to be audible?**
The worker unit is near the person and is the obvious annunciator, but a hazard-unit
sounder may also be wanted for anyone in the pit or shaft. Affects the hazard unit's power
budget.

**Q5 — Traction only, or must hydraulic and MRL configurations be covered?**
This outline assumes traction with a roped counterweight. Other configurations change the
topology enough to need their own pass.

---

## 13. Scope boundary

> **Hold this line explicitly.** This is a **secondary alert aid, not a protective
> device**. It does not substitute for lockout/tagout or for the mechanic's-protection
> provisions of ASME A17.1, and the product framing, packaging and documentation must say
> so plainly — otherwise the project inherits liability it is not built to carry.

Three deliberate exclusions follow from that, and each one also keeps the engineering
simpler:

- **No interface to the permanent installation.** No controller integration, no wiring, no
  modification to the elevator. Battery and magnet mount only. This is what keeps the
  device out of A17.1 certification scope.
- **No commissioning.** No site survey, no surveyed positions, no calibration step in the
  field. Zero-config or it will not be used.
- **No position, only range.** Absolute position is not needed, not measured, and not
  claimed.
