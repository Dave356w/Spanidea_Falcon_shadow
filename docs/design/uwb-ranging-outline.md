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
| Scope creep toward a certified safety device | Low | Section 12 boundary held explicitly in product and documentation |

---

## 10. Phased plan

Each phase ends at a gate that can stop the project cheaply. The ordering is deliberate:
the cheapest experiment that could invalidate the whole approach runs first.

### Phase 1 — Feasibility in a real hoistway

Two off-the-shelf dev kits, no custom hardware. Log raw range at 10 Hz in an actual shaft
while simultaneously logging the existing accelerometer stream. Run the physical test set
that currently gives the accelerometer approach trouble. Replay offline through the
existing `falcon_srcs/simulation/` harness and `falcon_srcs/graph/plot.py`.

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

## 11. Open questions

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

## 12. Scope boundary

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
