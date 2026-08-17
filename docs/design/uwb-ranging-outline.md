# Falcon — Ranging-Based Hazard Alert for Elevator Hoistways

**Clean-sheet outline · Rev C · 17 Aug 2026**

Status: concept, pre-prototype — with a Phase 0 prototype now scoped on existing hardware
Supersedes: the accelerometer + barometer approach in `falcon_srcs/src/movement_service.cpp`

Two or more identical battery-powered devices measure the distance between themselves by
UWB ranging. **Any relative motion alarms every device.** No accelerometer algorithms, no
orientation dependence, no model of the equipment. Secondary alert aid, no interface to the
permanent installation.

> Quantitative figures are estimates pending bench confirmation in Phases 1–2, except where
> attributed to Qorvo documentation.

**New in Rev C.** Section 11 scopes **Phase 0**, a two-node prototype built from the two
existing Falcons plus Makerfabs MaUWB DW3000 modules on Falcon UART0. It is a learning and
instrumentation exercise on hardware already in hand — it does not alter the target
architecture in Sections 2 to 9, and it is not on the path to the product. Sections 12 to 15
are the former Sections 11 to 14, renumbered.

---

## 1. Why the approach changes

### The structural reason

An accelerometer on the counterweight knows only about itself. It has no channel to a person
in the pit and no knowledge that they exist. However good the motion detection becomes, the
information never reaches the person who needs it.

That is not a performance gap, it is a category difference. **A mechanic in the pit, inside
the counterweight runby, is structurally unreachable by single-device acceleration sensing
and directly addressed by a distance measurement between two devices.** Everything else in
this document is detail by comparison.

### The mathematical reason

Acceleration must be integrated to yield velocity, so error accumulates without bound. A bias
of ε produces velocity error ε·t growing linearly with time. The existing
`MOVING_ACC_THRESHOLD` is 0.05 m/s²; a bias at that level integrated for ten seconds produces
0.5 m/s of phantom velocity — larger than inspection speed. That is why the code needs
`VEL_MAX_LIMIT` and the `ERROR_RESET` escape hatch. The approach is fighting its own
mathematics.

Range requires no integration. Each measurement is independent, and error does not
accumulate. A rate estimate is as good in hour eight of a shift as in minute one.

### The concrete reason: how the current firmware fails in the field

These are defects in the existing implementation, verified by reading the source. They are
recorded here because they are the practical case for the change, and because they matter
regardless of which direction the product takes.

**Note:** `ms.run()` is commented out at `main.cpp:134`. The committed firmware is a
data-collection build; the movement state machine does not execute.

#### Mounting orientation is baked into the arithmetic

`read_acceleration_mss()` ends with a hardcoded gravity subtraction (`main.cpp:166`):

```c
acc_mss = (acc_mss * 9.80665) - 9.80665;
```

This is correct only if the sensitive axis points up and reads +1 g at rest. One line encodes
a mounting orientation.

A boot-time auto-zero (`main.cpp:94`) captures `adj_acc_g` after 8 s and compensates
thereafter — but only for the orientation held *during those 8 seconds*.

#### The silent-failure trap

If the device is powered on in one orientation and mounted in another — the realistic field
sequence, since a mechanic powers a unit then reaches over to place it — the compensation is
captured wrong and never recovers:

1. Powered on flat, the axis reads 0 g, so `adj_acc_g` is captured as **+9.81**
2. Mounted vertical, the axis reads +1 g, leaving a **+9.81 m/s² standing offset**
3. Velocity integration blows through `VEL_MAX_LIMIT` in ~0.2 s → `ERROR_RESET`
4. Leaving `ERROR_RESET` requires `fabs(acceleration_avg) < 0.02` — it is 9.81, so it never does
5. The ISR's recovery re-zero is gated on `fabs(...) < 0.04` — also blocked

**The device sits in `ERROR_RESET` with the alarm disabled, permanently, until power-cycled
in its final orientation.** Green light on, looks healthy, will never alarm. For a safety
device this is the worst available failure mode, and nothing announces it.

#### Inverted mounting inverts the sign

Mounted upside down and powered on in place, the auto-zero absorbs the offset correctly but
the sign of measured acceleration flips. Most of the state machine is sign-symmetric, but one
place is not (`movement_service.cpp:95-97`):

```c
if (vel_threshold > 0) {
    vel_threshold *= VEL_THRESHOLD_ADJ;   // only positive thresholds are doubled
}
```

Inverting the mount swaps which direction of travel receives the doubled threshold. The
device becomes measurably more sensitive travelling one way than the other, depending on how
it was stuck to the counterweight.

#### Tilt is an uncorrected scale error

Off-plumb by θ, the axis measures `a·cos θ` — 13% loss at 30°, 29% at 45°. The auto-zero
corrects DC offset, not scale, and the thresholds are absolute.

#### One constant assumes a ramp profile, and fails both ways

`VEL_THRESHOLD_SET_TIMEOUT` is 400 ms. The firmware waits that long after detecting movement,
samples the velocity, and adopts it as the threshold — assuming the elevator is still
accelerating at that moment.

- **Modern S-curve drive with jerk limiting.** Acceleration ramps in over 0.5–1.5 s, so
  there is barely any velocity at 400 ms. `fabs(vel_threshold) < 0.01` triggers
  `setErrorResetState()` — a real start is discarded as noise.
- **Older, snappier drive.** Already at contract speed by 400 ms, so the threshold is set at
  full speed and then doubled. `isMovingConfirmed()` then requires exceeding *twice* the
  elevator's top speed, which never happens. It sticks in `MOVEMENT_DETECTED`.

#### Three more equipment dependencies

- **`VEL_MAX_LIMIT` is 2.0 m/s**, below the rated speed of much of the installed base —
  mid-rise and express cars run 2.5, 4, even 8 m/s. On those, normal operation trips the
  error path by design, every time.
- **`variance_acc < 0.001`** over a 16-sample window is a very tight flatness requirement.
  With guide-shoe chatter, rope oscillation and building sway it may never be satisfied,
  so `isAtRestOrStable()` never returns true, `DECELERATING` never reaches `STOPPED`, and
  the alarm never clears.
- **Constant-velocity cruise is indistinguishable from rest** — acceleration ~0, variance ~0.
  In the `MOVING` state that satisfies `isAtRestOrStable()`, which re-zeros `adj_acc`
  (`movement_service.cpp:117`) *while travelling at full speed*.

#### The root cause

Every one of these is the same defect in different clothing: **the firmware contains a model
of the equipment and the installation.** `0.05 m/s²`, `400 ms`, `2.0 m/s`, `0.001`, `0.006`
and the hardcoded `− 9.80665` are all assertions about one elevator mounted one way. The
device cannot detect when they are false, so it fails silently or nonsensically instead of
reporting that it is out of its depth.

This is why tuning has been "successful but difficult." It is not a tuning problem. The
constants are being fitted to one installation at a time, and the fit does not transfer.

### What ranging removes

| Failure | Under ranging |
|---|---|
| Hardcoded 1 g subtraction | **Gone.** `r = \|p₁ − p₂\|` is a scalar with no axis and no sign |
| Upside-down mounting | **Gone.** Rotating a device does not change a distance |
| Tilt / cos θ scale error | **Gone.** No projection onto a sensitive axis |
| Boot-orientation calibration trap | **Gone.** There is no gravity reference to capture |
| 400 ms ramp assumption | **Gone.** No constant encodes an expected profile |
| `VEL_MAX_LIMIT` below rated speed | **Gone.** Detection is speed-agnostic |
| Variance thresholds versus vibration | **Largely gone.** Vibration is a mean-reverting range transient, not a trend |
| Cruise indistinguishable from rest | **Gone.** Constant velocity is the *easiest* case — a clean straight line in `r(t)` |

Ranging measures the outcome directly instead of inferring it from a model of how motion
ought to look. There is nothing left to mis-model.

### What ranging does not remove

Stated plainly, because the difference matters:

- **Antenna pattern is orientation-dependent**, even though the measurement is not. Attitude
  affects link budget and first-path quality, not the meaning of the number.
- **Placement still matters.** Both devices must span the hazard and the person (Section 3).
- **The RF environment is now the risk.** A hoistway is a hostile multipath environment and
  the pit is the worst of it.

These have visible symptoms — degraded range, lost link, annunciated faults — rather than a
device that looks fine and silently never alarms.

---

## 2. System concept

**N identical devices.** Same hardware, same firmware, no roles, no variants. Every device
ranges, and every device alarms. Two is the baseline configuration; more can be paired when a
job calls for it, but nothing requires it.

The mental model is deliberately simple, because a mechanic has to get it right on a ladder:

> **Put a device where you are. Put a device on what could hurt you. If the distance between
> them changes, both scream.**

There is no correct or incorrect end, no configuration step, and no way to mount it the wrong
way round.

### Why symmetry, and not a smart/dumb split

An earlier revision of this document split the system into an unreachable primary-cell
"hazard unit" and a carried, rechargeable "worker unit" that did the thinking. That was built
on the premise that the counterweight device is semi-permanent infrastructure.

It is not. Devices are placed per job — counterweight, guide rail, pit floor, car top,
temporary platform — and retrieved afterwards. Every device is reachable, chargeable and
potentially the one nearest a person. So every device needs the full capability, and the
product is one SKU.

### Protocol roles are not product roles

SS-TWR still needs an initiator and a responder for each exchange, but that is an internal
detail, assigned dynamically, not a property of the hardware.

One consequence to design in: in SS-TWR only the initiator computes the range, and both ends
need it in order to alarm. **The initiator piggybacks the computed range back to the responder
in the following exchange** — one extra field, no extra frame. Both devices already perform
equal radio work in an exchange (one TX and one RX each), so the responder gets the result
essentially free. This is materially cheaper than alternating the initiator role, which would
double the exchange count for the same information.

### Scaling beyond two

N devices give N(N−1)/2 links — 1 at N=2, 3 at N=3, 6 at N=4. Airtime and per-device power
scale with that, and per-link update rate divides. FiRa defines one-to-many ranging (a
controller running a round-robin block against multiple controlees), so the multi-device case
is a supported session type rather than something to invent.

### Naming

Avoid *anchor* and *tag* — that is RTLS vocabulary and it will steer both the team and any
vendor SDK toward a trilateration stack built on assumptions that are all false here:

| RTLS assumes | This system |
|---|---|
| Three or more anchors for a position fix | Two peers, occasionally more |
| Anchors at surveyed, known positions | No survey; placement changes every job |
| Anchors are mains-powered infrastructure | All devices battery, all identical |
| Output is absolute position | Output is one scalar distance |
| Commissioned installation | Zero-config, tool-free, magnet-mounted |
| Fixed infrastructure, mobile tags | No fixed and no mobile — any device may be either |

Ask a vendor for **peer-to-peer two-way ranging between an initiator and a responder** and
you get the right example code. Ask for "anchors and tags" and you get an RTLS stack with a
position solver, network coordinator and gateway that will take months to delete.

---

## 3. Deployment

All modes run identical hardware and firmware, and no device needs to know which mode it is
in. The system never attempts to determine *which* device moved — only that the distance
between them changed.

### Pit work — the driving case

A mechanic working in the pit is exposed to the counterweight descending into its runby, and
to the car descending above. This is a known fatality mode and it is the case the product
exists for.

Place one device with the worker in the pit and one on the counterweight. As the counterweight
descends from 10 m above to alongside, the measured range travels from ~10 m to its closest
approach — an enormous, unambiguous signal for the entire approach.

**Pit work argues for three devices, not two.** A person in a pit is exposed to the
counterweight *and* the car, which travel in opposite directions. Two devices cover one of
those hazards and leave the other entirely unguarded. Two is a sound starting product; three
is the honest configuration for pit work.

The pit is also **the hardest RF case in the building** — a concrete box below the car, heavy
steel all round, and the car itself potentially interposed. It deserves a dedicated Phase 1
capture rather than treatment as one position among several.

### Service — existing installation

Counterweight and car top, counterweight and guide rail, car top and pit. Car and
counterweight are roped over the sheave and travel in opposite directions, so a
counterweight/car-top pair sees relative motion at twice car speed.

### Construction — temporary platform

No permanent car, rails installed only as high as the work has reached, geometry different
every day. The counterweight may not yet exist; the live hazards are the temporary hoist,
hoisted materials, and — the highest-value case — an **adjacent live car in a multi-hoistway
bank**, a recognised fatality mode this system addresses directly.

There is no static reference on a construction site, which is precisely why the design must
not depend on one.

### The one deployment rule

**The devices must span the hazard and the person.** Two devices both on hazards, or both
with people, measure nothing useful. Two devices on the same body are correctly silent.

This is the only thing a user can get wrong, and it is the thing training and product
documentation should concentrate on.

---

## 4. Link architecture

### Radio

UWB two-way ranging on a module with an integrated host MCU and BLE — the Qorvo DWM3001C
class of part. Per its datasheet (Rev F, Oct 2025): DW3110 + nRF52833 + planar UWB antenna +
ST LIS2DH12TR accelerometer + DCDC + trimmed crystal, certified to FCC, ISED and ETSI. That
single choice:

- collapses the UWB-for-measurement / BLE-for-control split into one component
- puts sleep current at 850 nA
- inherits modular approval, which for a low-volume safety accessory is months and tens of
  thousands of dollars avoided

Verify lifecycle and pricing against Qorvo's QM33 generation before committing. One firm
exclusion: **do not design in DW1000 / DWM1001** — previous generation, higher power, not
FiRa-compliant, approaching end of life.

**Rejected alternatives.** RSSI on BLE or sub-GHz cannot resolve vehicle-scale motion in a
steel-lined shaft. BLE Channel Sounding is materially more multipath-sensitive than UWB's
leading-edge detection and should be revisited only as a later cost-reduction path.

### Ranging scheme — single-sided TWR with CFO correction

Double-sided TWR cancels clock-frequency offset at the cost of a third message. Single-sided
is the better fit, for two reasons.

**First**, uncorrected SS-TWR error is:

```
ToF   = (T_round − T_reply) / 2
error ≈ (clock_offset × T_reply) / 2
```

The datasheet reports the module crystal is factory trimmed to under 2 ppm, so a realistic
module-to-module offset is a few ppm rather than a 20 ppm worst case. At 4 ppm over a 1 ms
reply delay that is roughly 0.6 m uncorrected. The receiver's carrier integrator then
estimates the residual to about 0.1 ppm, dropping error to roughly 1.5 cm.

**Second**, the residual is a slowly varying bias drifting on thermal timescales, while the
detection window is on the order of a second. It is common-mode across the window and
**cancels in any differential measurement**.

SS-TWR is two radio operations per end against three for DS-TWR — a 1.5× energy saving on
every device, in a design where every device is battery powered.

> **Consequence worth banking.** The same reasoning removes end-of-line antenna delay
> calibration. It is the largest absolute bias term (±30 cm uncalibrated) and normally a
> per-unit production step, but it is constant and the detector is differential. Ship one
> nominal per-design calibration.

### Rendezvous

Devices sharing a job form a session and range on a schedule. After long separation the
clocks drift apart, so a re-acquisition path is required: periodic wide RX windows, and a
long-preamble wake at arm time. That energy is paid once per session.

---

## 5. Alarm logic

### The rule

```
Is the distance between any pair changing, beyond the noise floor,
sustained over a window?
    → alarm, on every device
```

That is the whole detector. There is no time-to-contact, no proximity threshold, no
closing-versus-receding logic, no direction handling and no intent suppression.

**Motion is danger.** A moving car or counterweight is a hazard to anyone in the shaft
regardless of context, which is the premise behind lockout and the pit stop switch. Encoding
that directly is more honest than inferring intent and deciding some motion is acceptable.

### Why this is more robust than a rate-based rule

Range measures only the radial component of motion. For a device at lateral offset `d` from a
counterweight's vertical path, with vertical separation `h`:

```
r  = √(h² + d²)
|ṙ| = v · h / r
```

As the counterweight arrives level — `h → 0` — the range rate goes to **zero**. A
time-to-contact rule would report `TTC = (h² + d²)/(v·h)`, which is minimised at `h = d` and
therefore **can never fall below `2d/v`** no matter how close the hazard gets. At 0.5 m/s
with a 2 m offset, TTC never drops below 8 s, so a 4-second urgent threshold would never fire
through the entire approach and pass.

An any-motion rule is immune. Over the same approach the range travels from ~10 m to ~2 m — a
large, sustained, unambiguous change. The rate passes through zero only instantaneously at
closest approach, long after the alarm has latched.

### Detection margin is not the problem

With σ ≈ 2 cm per sample, the standard error of the mean over a 1 s window at 10 Hz is
~0.6 cm. An elevator at inspection speed covers **75 cm** in that second — roughly a
hundred-fold margin. Slow creep at 5 cm/s still clears the noise floor by 8×.

Detection is easy. The entire engineering risk lies elsewhere.

### The governing design rule

> **False alarms erode trust. Accurate warnings do not.**
>
> A device that reports motion when the counterweight actually moved is telling the truth,
> and truth-telling is what earns the trust it needs in a pit. A device that alarms when
> nothing moved is lying, and one lie costs more credibility than fifty accurate warnings
> earn.

Two consequences, and they govern every later tuning decision:

**Never suppress a true warning to reduce alarm count.** Frequency is not the defect.

**Attack false alarms only through discrimination, never desensitization.**

- *Desensitization* — raising the motion threshold — cuts false and true detections
  together. It buys quiet by missing real events. **Rejected.**
- *Discrimination* — exploiting properties that differ between real motion and artefacts —
  cuts false alarms while leaving true detection untouched. **This is the only acceptable
  mechanism**, and any proposed rejection technique must clear that bar before it goes in.

### The real risk: multipath masquerading as motion

In a shaft the reported range can shift by tens of centimetres with nothing moving — a rope
swings, a door opens, the first path is occluded and the receiver locks onto a reflection.
NLOS bias is one-sided and can present as a sustained trend, not merely a spike.

Permitted discriminators:

- **First-path quality gating.** `FP_AMPL`, `maxNoise` and `FP_INDEX` indicate when the
  receiver is not confident it found the leading edge. Discarding those samples is declining
  to believe an unreliable *measurement*, not suppressing an *event*. Real motion with a
  clean first path is untouched.
- **Signal shape.** A real elevator produces sustained, continuous change over seconds. A
  multipath re-lock produces a **step** — a jump followed by a flat line. Trivially
  separable, and separation costs nothing on true events.
- **Median or RANSAC fitting** over the window rather than plain least squares, since NLOS
  outliers are one-sided.
- **Confirmation windows.** Requiring consecutive qualifying windows costs *latency*, not
  sensitivity. At elevator speeds a few hundred milliseconds is affordable.

### The accelerometer's remaining role

The LIS2DH12 is inside the module at no extra cost. It has three jobs, none of which involve
integrating anything or estimating velocity:

**1. Corroboration — using vibration energy, not acceleration.**

An accelerometer feels acceleration, not velocity. During constant-velocity travel — most of
any long run — a device riding the counterweight registers nothing but gravity, exactly as at
rest. Any corroboration scheme based on DC acceleration would therefore fail during cruise,
suppressing a true warning.

Use **AC energy in the accelerometer band** instead. A running elevator produces continuous
broadband vibration from guide shoes, machine and ropes, which persists through constant
velocity. No integration, no orientation reference, no gravity assumption.

> **Corroboration must never veto.** A clean, sustained range change alarms on its own
> authority regardless of what the accelerometer reports. Vibration raises confidence; its
> absence never blocks an alarm.

**2. Fault cross-check.** Vibration present while range is perfectly static means something
is broken — not that conditions are safe.

**3. Wake gating**, now purely a power optimisation (Section 6) rather than a requirement.

### Liveness — the fail-silent problem

Three states look identical through a range-only detector:

- Nothing is moving *(safe)*
- The peer has crashed or its battery is flat *(unknown)*
- The link has failed *(unknown)*

A safety-adjacent device whose failure mode is indistinguishable from all-clear is
unacceptable. **Ranging updates must arrive at an expected cadence, and their absence is a
fault condition that annunciates.** Silence must never be the same as safety.

### Latch and clear

Alarms latch through the event and clear after a hold-off with hysteresis, so annunciation
does not chatter as motion starts and stops. A brief movement must still produce a clear,
noticeable alert rather than a click.

### Two metrics, never traded

| Metric | Meaning | Target |
|---|---|---|
| **False alarm rate** | Alarms with nothing moving | Zero. This is the trust metric |
| **Detection rate** | Alarms per actual motion event | 100%. This is the safety metric |

Reporting these as a single "nuisance alarm" number invites exactly the trade that must not
be made.

There is a clean way to measure the first: **lock out the elevator, leave the devices mounted
and armed overnight, and count alarms.** Nothing is moving, so every alarm is unambiguously
false. No observer, no interpretation, no judgement. This should be a standing test at every
stage, not a one-off — and it can run unattended in a real hoistway before the car is ever
moved.

---

## 6. Power

**Target: a 40-hour working week on one charge**, all devices rechargeable.

This is a far easier target than the five-year primary cell an earlier revision assumed, and
the consequences are large.

### The budget closes without gating

Using the datasheet's measured currents, an SS-TWR exchange is roughly 0.3 mJ per device.
Ranging **continuously at 10 Hz with no gating at all** costs on the order of 1 mA, and with
MCU, accelerometer and annunciator standby the armed average lands around 3–5 mA.

```
40 h × 5 mA  ≈  200 mAh
```

A 1000 mAh cell provides roughly **5× margin**. The power problem is solved by the
requirement, not by cleverness.

That removes accelerometer-gated adaptive ranging from the critical path entirely. It remains
worth doing for airtime and thermal reasons, and it matters more as device count grows, but
it is now an optimisation rather than the constraint the architecture is built around.

### Measured device currents

| State | Current |
|---|---:|
| SLEEP | 850 nA |
| INIT (Ch5 / Ch9) | 6 mA |
| IDLE Ch5 | 18 mA |
| IDLE Ch9 | 32 mA |
| TX / RX Ch5 | 40 mA |
| TX / RX Ch9 | 45 mA |

### What this deletes

- **Li-SOCl₂ chemistry** and its poor pulse response
- **The hybrid-layer capacitor / supercapacitor buffer** that came with it
- **Sub-µA sleep-current engineering**
- **Adaptive gating as a survival requirement**

### What it adds

Charging: Li-ion cell, charge management, fuel gauge, and a workflow for charging several
devices together with the rest of a mechanic's tools.

**Regulation is mandatory.** The DWM3001C specifies VDD max **3.6 V**; a Li-ion cell reaches
4.2 V fully charged. A buck or LDO stage is required — the cell cannot connect directly.

---

## 7. Hardware

Build every device on the same module, one design, one firmware image. This was already the
right call for the prototype; with symmetric peers it holds through production.

### Antenna is the top hardware risk

Devices are magnet-mounted onto steel at arbitrary attitude, knocked off and re-mounted
several times a day, on counterweight frames, rail brackets and pit floors. Antenna pattern is
a first-order design problem, not a detail.

**The datasheet's keep-out is in direct tension with the product concept.** It requires a
minimum 10 mm clearance with no metal either side, no copper pour in the keep-out area, and
explicitly *"do not place battery under antenna."* A magnet mount clamps the device to a large
steel mass — exactly the condition warned against.

Resolving this is a Phase 4 mechanical problem: cantilever the antenna end of the PCB off the
magnet, stand the enclosure off the steel, or accept a characterised pattern degradation.
Treat it as a named constraint on the enclosure, not something discovered during layout.

It also pulls against the separate guidance in Section 12 to keep the module at least 1 cm
from the carrier PCB edge for multipath resilience. The two must be resolved together.

### Mounting constraint, counterweight rail

The counterweight's guide shoes sweep all three machined rail faces over the entire travel.
Anything on a running face is sheared off. Rail mounting is restricted to the rail back, web,
or a bracket — which is precisely where steel shadowing is worst.

### Annunciation

Every device alarms, in an environment with machine noise, and often out of the operator's
line of sight. A piezo alone is unlikely to be sufficient; plan for loud audible plus visual,
and evaluate haptic for a device carried on the person. This is an open question (Section 14).

### Environment

Module rated −40 to +85 °C, comfortably covering the −10 to +60 °C hoistway environment
(hotter near the machine room). Rail lubricant, dust, condensation, continuous vibration. RF
is untroubled by grime; mechanical sealing and magnet retention under sustained vibration are
the real risks.

---

## 8. Firmware

The module brings its own Cortex-M4, so this ends the ATmega328P and moves firmware to the
Nordic SDK or Zephyr. That is a full port, not an incremental change.

| Component | Disposition | Note |
|---|---|---|
| `RollingAvg` | Survives | Reused for the range window |
| `MovementService` state logic | Mostly deleted | The state machine exists to manage integration drift; almost none of it is needed |
| `ERROR_RESET` path | Deleted | Exists only to catch integration divergence |
| DPS310 barometer + pressure variance | Deleted | Exists only to cross-check accelerometer drift |
| Velocity integration in the ISR | Deleted | The defect at the root of Section 1 |
| Arduino / AVR HAL, `main.cpp` | Rewritten | Full platform change |
| Accelerometer driver | Rewritten | LIS2DH12 over I²C, vibration-energy only |
| Alarm output, battery sense | Rewritten | New pin map, charging topology |
| Ranging session, scheduling, sleep | New | |

Phase 0 (Section 11) moves in the opposite direction — it keeps the ATmega and adds to it —
and none of that firmware survives into the target design. What transfers from it is design
rather than code: the CSV schema, the replay pipeline and whatever the logs teach about
thresholds.
| Change detection and discriminators | New | The genuine engineering effort |

### Signal processing notes

- **NLOS bias is one-sided.** Blocked or reflected first paths always read *longer*, never
  shorter. Fit the window with a median or RANSAC estimator rather than plain least squares.
- **Tune leading-edge detection early.** First-path detection parameters are the difference
  between UWB working in a steel shaft and not. Start at a 128-symbol preamble.
- **Channel — Ch5, on two grounds.** Ch5 (6.5 GHz) propagates better through a steel shaft
  than Ch9 (8 GHz), and draws 18 mA idle against Ch9's 32 mA. Regional regulatory rules
  differ; confirm against target markets before locking, since it drives antenna design.

### Configuration

No dedicated base station. Pairing, arm/disarm, thresholds, log offload and firmware update
run over BLE from the mechanic's phone — which is what the nRF52833 is for. Keeping a fixed
site unit out preserves the property that matters most: a zero-config personal safety
accessory rather than a commissioned system.

---

## 9. Risk register

| Risk | Sev | Mitigation |
|---|---|---|
| Multipath produces false motion; false alarms destroy trust | High | **The primary Phase 1 gate.** Discriminators in Section 5; overnight locked-out measurement |
| Multipath degrades first-path detection beyond usable | High | **Vendor-corroborated** (Section 12). Phase 1 answers it before hardware spend |
| The pit — the driving use case — is the worst RF environment in the building | High | Dedicated Phase 1 capture rather than one position among several |
| Antenna orientation and metal proximity under magnet mounting | High | Characterise early; keep-out versus mount resolved together in Phase 4 |
| Fail-silent: link loss indistinguishable from "nothing moving" | High | Liveness annunciation, Section 5 |
| Devices deployed both-on-hazards or both-with-people | Med | Single deployment rule, Section 3; training and product documentation |
| Module lifecycle or supply discontinuity | Med | FiRa-compliant parts only; second-source assessment at Phase 4 |
| 40-hour target missed | Low | ~5× margin without gating; gating available in reserve |
| Scope creep toward a certified safety device | Low | Section 15 boundary held explicitly |
| Phase 0's rate rule clears the alarm at closest approach | High | Quantified in Section 11; Phase 0 is an instrument, the Section 5 any-motion rule is the detector |
| Phase 0 rail brownout — 3.1 V under alarm plus ranging | Med | Section 11 Step 26 measurement before any production power decision |
| Phase 0 becomes the product path by momentum | Med | Section 11 exit criteria and its Step 27 stop rule; Phase 1 neither waits on Phase 0 nor depends on it |

---

## 10. Phased plan

Each phase ends at a gate that can stop the project cheaply. The cheapest experiment that
could invalidate the approach runs first.

### Phase 0 — Falcon-hosted MaUWB prototype *(optional, parallel)*

Two existing Falcons, two Makerfabs MaUWB DW3000 modules on UART0, a CSV log out of the second
USART. Detailed in Section 11. It runs on hardware already owned, ahead of and independent of
the DWM3001CDK order, and it exists to build ranging intuition, the logging and replay
pipeline, and a tunable state machine — not to produce a product architecture.

> **Gate** — this phase gates nothing. Phase 1 does not wait for it, does not depend on it,
> and is not cancelled by it. If Phase 0 slips or disappoints, Phase 1 proceeds unchanged.
> The one outcome that would matter to Phase 1 is a *positive* one: a clean hoistway range
> trace obtained early, which sharpens what Phase 1 goes looking for.

### Phase 1 — Feasibility in a real hoistway

Three or four off-the-shelf dev kits, no custom hardware. Two questions, in this order:

1. **With devices mounted and nothing moving, what is the false-motion rate from multipath
   alone?** Measurable overnight with the elevator locked out, before the car ever moves.
2. **Does a real elevator produce an unambiguous, sustained range signature** — including in
   the pit, and including the runs where the current state machine fails?

Log range and diagnostics; replay offline through `falcon_srcs/simulation/` and
`falcon_srcs/graph/plot.py`.

> **Gate** — false-motion rate driven to acceptable by first-path quality gating and shape
> discrimination, *without* raising the motion threshold. Real motion detected reliably,
> pit included. **If this fails, stop here.**

### Phase 2 — Detector and session

Implement SS-TWR with CFO correction, the change detector and its discriminators, the
liveness path, and the ranging session with range piggyback. Measure real energy per state
and confirm the 40-hour budget on hardware.

> **Gate** — detection rate at 100% on recorded true events, false alarm rate at zero on
> recorded quiet periods, and 40 h armed demonstrated.

### Phase 3 — Field trial

Alarm annunciation, latch and clear behaviour, arm/disarm, multi-device sessions. Trial in
service and construction settings with working mechanics.

> **Gate** — **false alarm rate remains zero in the field**, and mechanics report the device
> is believed. Detection rate is not traded to achieve it.

### Phase 4 — Productisation

Custom PCB, enclosure and magnet mount qualified under vibration, antenna keep-out resolved
against the mount, charging and cell selection, phone app, regional regulatory confirmation.

> **Gate** — pilot units survive a full service season without mechanical or power failure.

---

## 11. Phase 0 — Falcon-hosted MaUWB prototype

A two-node prototype built from the two existing Falcons and two Makerfabs MaUWB DW3000
modules, wired to Falcon UART0. Each Falcon keeps its BMA456, its LED/strobe and its buzzer,
adds a UWB range measurement, and streams engineering data out of a second UART for logging
and threshold tuning.

### What it is for, and what it is not

Phase 0 exists because the hardware is already on the bench. It buys three things early:
familiarity with UWB ranging behaviour in a real shaft, a CSV capture-and-replay pipeline that
Phase 1 reuses unchanged, and a state machine whose thresholds can be moved without a
reflash. All three are transferable. The hardware it runs on is not.

> **This is not the production architecture, and no part of it should be read as amending
> Sections 2 to 9.** Section 8 ends the ATmega328P; Section 12 records that the target module
> needs no external microcontroller. Phase 0 keeps the ATmega because the ATmega is what
> exists today, not because the argument for removing it has weakened.

Where Phase 0 diverges from the target design, it does so knowingly:

| Target design | Phase 0 | Why the divergence is acceptable |
|---|---|---|
| Module's own Cortex-M4 is the only processor | ATmega328PB hosts a UART-attached module | Uses hardware in hand; the port is deferred, not avoided |
| N identical peers, no roles | One "moving" node, one "fixed" pit node | A deliberate simplification for first light, not a product split |
| Peer-to-peer initiator/responder vocabulary | The module's own anchor/tag configuration words | Confined to module config; see the naming warning below |
| Any sustained change over a window alarms | Range-rate thresholds with a stop timer | **A known defect at closest approach — quantified below** |
| Thresholds set from data, never guessed | Provisional starting values | Labelled test-only, and replaced from logs in Step 24 |
| Vibration energy from the accelerometer | BMA456 magnitude and a motion flag | Fine for bench work; revisit before it means anything |

### Step 1 — the two-node arrangement

```text
NODE A — MOVING                     NODE B — FIXED
car or counterweight                elevator pit

Falcon                              Falcon
  ├── BMA456 accelerometer            ├── MaUWB DW3000
  ├── MaUWB DW3000                    ├── LED / strobe
  ├── LED / strobe                    └── buzzer
  └── buzzer
```

One module is configured as the ranging tag, the other as the anchor.

> **Keep that vocabulary inside the module.** Section 2 explains why *anchor* and *tag* must
> not enter the system description: they pull the design toward an RTLS stack built on
> surveyed positions and fixed infrastructure, none of which is true here. In Phase 0 they are
> two configuration words in a vendor firmware. They are not roles, they are not product
> variants, and nothing in the Falcon firmware should be named after them.

**Nothing connects to the elevator.** No controller interface, no safety circuit, no brake, no
control equipment. This is a supplemental warning device on a bench and then on a magnet — the
boundary in Section 15 applies to Phase 0 in full and from the first day.

### Steps 2–3 — verify what already works, before adding anything

Confirm on both units, with the modules still unconnected: power-on, LED, buzzer, BMA456,
battery ADC, J1 UART, J2 ISP programming. Then measure J2 `VCC_3V1` to GND with the module
disconnected and confirm roughly 3.1 V.

Schematic pinout for J2:

```text
J2-1  VCC        J2-2  MOSI
J2-3  GND        J2-4  MISO
J2-5  SCK        J2-6  RESET
```

> The Rev.2 PCB may present these pads differently from the schematic drawing. **Meter the
> actual 3V1 and GND pogo positions before building the fixture**, not after.

Two facts about the committed firmware are worth knowing before this starts, both from
Section 1: `ms.run()` is commented out at `main.cpp:134`, so what is on the boards is a
data-collection build with the movement state machine dormant; and the DPS310 barometer paths
are `#if 0`-ed out. Phase 0's state machine is a new detector alongside that code, not a layer
on top of it. Do not re-enable `MovementService` to "compare" — Section 1 documents why its
output would not be meaningful.

### Steps 4–5 — power and the UART link

```text
Falcon 3V1 ──┬──────────► MaUWB +V         FALCON            MaUWB
             │                             UART0_TXD ──────► RX
          47–100 µF                        UART0_RXD ◄────── TX
             │                             GND ───────────── GND
            GND

3V1 ─── 100 nF ─── GND      (both close to the module)
```

Tie the module's power pads together rather than feeding it through a single ground contact.
Direct TX/RX wiring, no level translator and no series resistor, is acceptable for a short
bench link with both ends near 3.1 V logic.

> **The rail is the first thing likely to bite.** Falcon's `VCC_3V1` is a battery rail, not a
> regulated supply with headroom, and modules in the MaUWB class carry an ESP32-class host
> alongside the DW3000 — a part whose supply range bottoms out at about 3.0 V and whose
> current draw is spiky, not flat. Running that from 3.1 V leaves almost nothing between a
> transmit burst and a brownout, and the worst case is not ranging alone: it is **ranging
> while the buzzer and strobe are both driven**, which is exactly the moment the device must
> not reset. Treat Step 4 as experimental, and do not close it out until the measurements in
> Step 26 exist. Confirm the module's actual supply range and peak current from its own
> documentation before applying power the first time.

### Step 6 — bench the modules alone, first

Range the two modules against each other at 1, 2, 5 and 10 m before any Falcon firmware is
touched. Record actual distance, reported distance, variability, invalid readings and dropouts,
then walk one unit slowly toward and away from the other and confirm the reported distance
follows smoothly.

> **Establish what "good" looks like on the bench before entering a hoistway.** This is the
> same instruction as Section 13, Stage 2, and for the same reason: a bad number in a shaft is
> uninterpretable without a clean-air reference to compare it against.
>
> **And check one thing here that decides how much Phase 0 can ever prove.** Section 5 permits
> exactly one class of false-alarm mitigation — discrimination on first-path quality, using
> `FP_AMPL`, `FP_INDEX` and `maxNoise` — and forbids the alternative of raising thresholds.
> Whether those diagnostics are reachable through the module's AT interface is unknown and
> should be settled at Step 6, not discovered at Step 23. If they are not exposed, Phase 0 can
> measure the false-motion rate but cannot legitimately reduce it, and any quiet it achieves by
> moving `RANGE_ON` upward is desensitization, which Section 5 rejects.

### Steps 7–9 — two UARTs

Start with three functions on UART0 and nothing else: send command, receive line, parse range.
Prove that the Falcon repeatedly obtains plausible values and recognises missing or malformed
ones before any motion logic exists.

The logging UART uses the ATmega328PB's second USART, whose pins are shared with ISP:

```text
runtime                                    programming
J2 MOSI / PB3 / TXD1 ─────► USB-UART RX    J2 reverts to AVR ISP
J2 MISO / PB4 / RXD1 ◄───── USB-UART TX
J2 GND ──────────────────── USB-UART GND   SCK and RESET stay free for the programmer
```

Giving two independent links: UART0 to the module, UART1 to the laptop. Key the pogo fixture
so it cannot be fitted backwards.

> **Three things to resolve before writing that code.**
>
> - **USART1 exists on the ATmega328PB and not on the ATmega328P.** `platformio.ini` currently
>   builds `board = ATmega328P`, and the Arduino core selected there has no `Serial1`.
>   Phase 0 needs a 328PB target and a core that exposes the second USART; confirm that before
>   designing around it.
> - **The pogo fixture and the programmer must never be attached at once.** PB3/PB4 are MOSI
>   and MISO. A USB-UART sitting on those lines during an ISP cycle is a bus contention, and
>   an ISP cycle during logging corrupts the log.
> - **UART0 is currently the only debug channel.** Every `Serial.print` in `main.cpp` goes out
>   of the port that is now committed to the module. Those calls move to UART1 or they become
>   noise on the module's receive line.

### Steps 10–12 — the engineering stream

The value of Phase 0 is mostly here. Generate a CSV diagnostic stream rather than echoing raw
module output:

```text
time,accel,range_raw,range_filt,delta,velocity,state,alarm
12540,18,8241,8238,3,30,ARMED,0
12640,116,8268,8249,27,110,SUSPECT,0
12740,143,8307,8271,39,225,MOVING,1
12840,72,8350,8302,43,310,MOVING,1
```

Full field set: timestamp, accelerometer magnitude, accelerometer motion flag, raw range,
filtered range, range difference, range velocity, UWB-valid flag, link status, state, alarm
state, transition reason, battery ADC.

Filter lightly — a 3 to 5 sample moving average is enough for an ATmega — and **keep both
`range_raw` and `range_filt` in the output** so the filter itself can be evaluated rather than
trusted. Range rate follows from the filtered series:

```text
range_delta    = filtered_now − filtered_previous
range_velocity = range_delta / elapsed_time
```

Range rate is the measurement that keeps working when the accelerometer goes quiet during
constant-speed travel — the failure Section 1 identifies in the existing firmware and
Section 5 addresses by never depending on acceleration in the first place.

> Keep the CSV column names and units stable from the first capture. Section 10 replays
> captures through `falcon_srcs/simulation/` and `falcon_srcs/graph/plot.py`; a schema that
> changes mid-programme means early runs cannot be re-scored against later thresholds, which
> is most of the reason to log at all.

### Steps 13–14 — state machine and provisional thresholds

```text
STARTUP → ARMED → MOTION_SUSPECTED → MOVING → STOP_CONFIRM → ARMED
                  (accel or range rate) (confirmed)  (quiet for X s)

FAULT — separate, for communications and measurement problems
```

Starting values, **for bring-up only**:

```text
MOTION_RANGE_RATE = 75 mm/s        MOTION_CONFIRM    = 2–3 measurements
STOP_RANGE_RATE   = 25 mm/s        STOP_CONFIRM_TIME = 3000 ms
```

These are firmware variables, not compile-time constants, and they are guesses. Open question
Q4 stands: real values come from logged runs in Step 24. The one rule that governs how they
move is in Section 5 — thresholds are never raised to quiet a false alarm.

> #### The defect in this rule, quantified
>
> Section 5 rejects rate-based alarm logic, and Phase 0 adopts it anyway for tractability. The
> cost is specific and it should be understood before any hoistway run, not rediscovered in
> the data.
>
> For a pit node at lateral offset `d` from the counterweight's path, with vertical separation
> `h` and travel speed `v`:
>
> ```
> r  = √(h² + d²)          |ṙ| = v · h / r  ≈  v · h / d   for h ≪ d
> ```
>
> The range rate passes through **zero** as the counterweight arrives level. It falls below
> `STOP_RANGE_RATE` while `h < d·R_off/v`, and the time spent in that band is:
>
> ```
> dwell = 2·d·R_off / v²
> ```
>
> With `d` = 2 m and `R_off` = 25 mm/s:
>
> | Travel speed | Rate-null dwell | Against `STOP_CONFIRM_TIME` = 3000 ms |
> |---|---|---|
> | 500 mm/s (normal) | 0.4 s | Alarm survives |
> | 150 mm/s (inspection) | **4.4 s** | **Alarm clears at closest approach** |
> | 50 mm/s (creep) | **40 s** | **Alarm clears for the whole approach** |
>
> Dwell scales as `1/v²`, so the slower the car, the worse it gets — and inspection speed is
> the realistic service case, closest approach is the dangerous moment, and the pit node is
> the one next to a person. The accelerometer AND-term does not rescue it: the pit node is
> genuinely stationary throughout.
>
> Three consequences. **One:** `STOP_CONFIRM_TIME` must exceed `2·d·R_off/v²` at the slowest
> speed of interest, which pushes it to tens of seconds and makes the stop rule nearly
> vacuous. **Two:** this is the concrete demonstration of why Section 5 specifies sustained
> *change* over a window rather than instantaneous rate — the any-motion rule has no null.
> **Three:** if Phase 0 is to produce a detector rather than an instrument, the fix is to
> adopt the Section 5 rule, not to tune around this. Recording the null in real data is itself
> a worthwhile Phase 0 result.

### Step 15 — live threshold commands

Tuning without reflashing, over UART1:

```text
STATUS   SET RANGE_ON 75   SET RANGE_OFF 25   SET STOP_TIME 3000
SET ACCEL_ON 120           SAVE               DEFAULTS
```

The unit echoes what it stored — `RANGE_ON=75` / `OK` — so a clamped or rejected value is
visible in the log at the moment it happened.

### Steps 16–18 — alarm, peer, and clearing

On entering `MOVING`, drive `BUZZ_EN` and the strobe pattern; the existing AO3400A MOSFET
already switches the buzzer, so no module-side driver is needed. The moving node also drives
the pit node into alarm, and **the motion condition is refreshed continuously while movement
lasts** rather than sent as a single packet — a lost packet must not clear an alarm.

That refresh is nearly free: Section 4 notes that in single-sided TWR only the initiator
computes the range, so a result has to travel to the peer in any case. The same field carries
the alarm condition.

Clearing requires `|range_velocity| < STOP_RANGE_RATE` **and** a quiet accelerometer,
continuously, for the whole of `STOP_TIME`. One unchanged measurement is not a stop.

### Step 19 — faults must not look like quiet

Detect at minimum: module not responding, no valid range, link lost, UART framing or parse
failure, low battery, unexpected MCU reset. Log the reason, not just the state:

```text
STATE=FAULT
REASON=UWB_TIMEOUT
```

> **A fault must never present as a healthy stationary system.** This is the fail-silent
> requirement from Section 5, and it is the one piece of Phase 0 that is genuinely
> safety-relevant rather than exploratory: a device that goes quiet because its link died,
> while looking exactly like a device that is quiet because nothing is moving, is worse than
> no device. Give it a distinct annunciation, and hold the alarm through a fault that
> interrupts an active alarm rather than clearing it.

### Steps 20–25 — test, plot, tune, freeze

Bench first, moving one Falcon relative to the other: stationary, slow, fast, short movement,
vibration without translation, translation without large acceleration, start-stop-start, and
temporary radio obstruction. Capture a log for every run and plot range, range velocity,
accelerometer, state and alarm on a common time axis.

The two rows that matter are the discrimination cases — vibration without translation must not
alarm, and translation without acceleration must — because they are the two the existing
accelerometer firmware gets wrong.

Then the hoistway: stationary car, initial movement, acceleration, constant-speed travel,
deceleration, stop, short jog, multi-floor travel. Vary car height, pit-node position, antenna
orientation, doors, occupancy, car versus counterweight mounting, and maximum separation.
Record range error, variance, packet loss, maximum usable distance, false detections and
missed movement.

> **Add the overnight run.** Section 5 defines the one measurement that needs no interpretation:
> lock the elevator out, leave both units mounted and armed overnight, and count alarms. Every
> alarm is unambiguously false. It is the primary Phase 1 gate, it costs nothing here beyond a
> night, and without it Phase 0 is a demonstration rather than a measurement.

Freeze what survives: parameters to EEPROM, plus firmware version, module configuration,
threshold set and test results recorded together. Each unit reports its configuration on the
logging UART at startup:

```text
FALCON UWB MOTION TEST
FW=0.4   RANGE_ON=78   RANGE_OFF=24   STOP_TIME=3000   FILTER=5   NODE=PIT
```

Section 13's capture-metadata list applies here too, and for the same reason: identical units
and undocumented configuration make a capture unreadable a week later.

### Steps 26–27 — power measurement, and where Phase 0 stops

Measure current in four conditions — Falcon alone, Falcon plus module idle, Falcon plus module
ranging, and alarm plus ranging together — and record rail behaviour under each. That data
decides between the existing 3.1 V rail and a dedicated 3.3 V regulator off `VCC_4V5_RVP`, and
**no production power design is frozen until it exists**.

Step 27 sketches a Falcon UWB daughterboard: module, UART connection, power input, regulator
if required, bulk and bypass capacitors, programming pads, mechanical support and antenna
clearance, with the antenna near or beyond the PCB edge and away from the battery and large
metal.

> **This is the point to stop and re-read Section 8.** A daughterboard is a commitment to the
> ATmega-plus-module architecture, and Section 8 argues the target design has no external
> microcontroller at all — the module's own MCU is the host, which deletes the UART protocol,
> the second USART, the level and rail questions, and the daughterboard itself. Phase 0 can
> justify its own bring-up fixtures. It cannot justify a production PCB revision. If a
> daughterboard starts to look necessary, that is the signal Phase 0 has outgrown its purpose
> and the decision belongs to Phase 4 on Phase 1 data.

### Phase 0 exit criteria

The prototype has done its job when all of the following hold. The first ten are the Rev-0
success criteria; the last two are what make the exercise a measurement rather than a demo.

| # | Criterion |
|---|---|
| 1 | Both modules range reliably |
| 2 | Falcon reads range over UART0 |
| 3 | UART1 logs state-machine data continuously |
| 4 | Falcon detects the start of equipment motion |
| 5 | UWB confirms continued movement |
| 6 | Both Falcons alarm during movement |
| 7 | The alarm stays active through constant-speed travel |
| 8 | The alarm clears only after a confirmed stop |
| 9 | Link loss produces a distinguishable fault condition |
| 10 | Thresholds can be tuned from logs without reflashing |
| 11 | **An overnight locked-out capture exists, with the false-alarm count from it** |
| 12 | **The closest-approach rate null is either observed in real data or shown not to occur** |

### What Phase 0 can and cannot answer

| Phase 1 question | Phase 0 | Note |
|---|---|---|
| Does a real elevator produce an unambiguous range signature? | **Yes** | The most transferable result available here |
| What is the range envelope and link continuity in a shaft? | **Yes** | Module-specific, but indicative |
| What is the false-motion rate from multipath? | **Partly** | Measurable; reducing it needs first-path diagnostics that may not be exposed |
| Can first-path quality gating suppress false motion? | **Only if the AT interface exposes the metrics** | Settle at Step 6 |
| Does the pit case work? | **Partly** | Worst RF case in the building; a negative here is informative, a positive is not conclusive on different silicon |
| Antenna orientation dependence | **No** | Different module, different antenna; Section 12 records this gap on the DK too |
| Power budget for the product | **No** | Different processor, different radio duty cycle, different battery |
| Sleep and 40-hour endurance | **No** | Phase 0 is mains- or bench-powered in practice |

---

## 12. Phase 1 bill of materials and instrumentation

Prices and stock as researched 16 Aug 2026. Reconfirm before ordering.

### Hardware — Qorvo DWM3001CDK

Approximately **$29.50 each at DigiKey**. **Order three or four**, not two: multi-device
operation is now part of the concept, pit work argues for three, and Mouser flags a long lead
time despite immediate stock.

Confirmed against the DWM3001C datasheet (Rev F, Oct 2025), the DWM3001CDK Product Brief
(Rev B, May 2022) and the Quick Start Guide (QM33SDK-1.0.0, Aug 2024):

| Requirement | How the DWM3001CDK meets it |
|---|---|
| UWB ranging on target silicon | DW3110 (non-PDoA) + nRF52833 with Bluetooth 5.2 |
| Accelerometer on board | **ST LIS2DH12TR** on the nRF52833's I²C bus (module pins 14/15) |
| Battery operation | Dedicated battery connector with VBAT rail into the onboard DCDC |
| Debug and capture | Onboard J-Link OB (SWD, UART); second USB direct to the DWM3001C |
| Both candidate channels | UWB bands 5 (6.5 GHz) and 9 (8 GHz) |
| Phase 2 power measurement | **Dedicated header on the VCC trace for measuring module current** |
| Expansion | 26-pin Raspberry Pi header, all DWM3001C GPIOs, GPIO test points |

Qorvo's stated rationale for the accelerometer is worth noting: *"RTLS tags commonly use
accelerometers to initiate UWB ranging only when a tag moves so that battery life can be
extended."* That is the wake-gating optimisation in Section 6 — the vendor's intended usage.

> **It is not a BMA456.** The LIS2DH12 is a lower-specification part than the sensor currently
> in `falcon_srcs`. This does not matter for the new design, which uses it only for vibration
> energy and health — but it matters if any Phase 1 run is intended to reproduce the *existing*
> algorithm for comparison. Do not silently substitute one for the other.

### Landmine: the dev kits are engineering samples

The kit part number is **DWM3001CDKE1.0**, and the datasheet warns for that part number:

> *"do not use the Channel 5 Antenna Delay in OTP — use default value 16390."*

This is on the critical path, since Section 8 selects Channel 5. A wrong antenna delay is a
**constant bias**, so it is invisible in any differential measurement — the change detector
would look perfectly healthy — while every absolute range is silently offset. Override before
the first capture and record that you did.

The datasheet lists the kit at MOQ 20 units direct from Qorvo; distributors sell singles.

### No external microcontroller is required

The nRF52833 inside the module is the host. The 26-pin Raspberry Pi header is an option for
host-driven setups and an alternative power inlet, not a requirement. This holds through
production: the module solders to the carrier PCB and remains the only processor.

### Instrumentation

In SS-TWR the initiator computes the range, so make the tethered device the initiator and
every measurement lands on the laptop already. Peer devices need battery and nothing else.

Log **every exchange**: timestamp, range, CFO estimate, `FP_INDEX`, `FP_AMPL`, `maxNoise`, RX
power, sequence number, and piggybacked accelerometer vibration energy from the peer — one
file, inherently time-aligned, no clock reconciliation.

> **Raw CIR needs a throughput plan.** A CIR record is ~1016 samples; at 10 Hz that is on the
> order of 40 KB/s against roughly 11.5 KB/s available on a 115200 baud UART. It will not fit.
> Use the **USB CDC interface** (nRF52833 provides USB 2.0 full speed, on the DK's second
> micro-USB port), and log full CIR on a **subsample** or **on trigger** when scalar
> first-path metrics look anomalous, while logging the scalars every exchange.

### Vendor guidance on the exact Phase 1 risk

The datasheet contains a section titled *"Note on Ranging Performance in Harsh Multipath
Environments."* The module antenna is **cross-polarised in some regions** — good news for
arbitrary mounting attitude, since it holds link budget as orientation changes. But Qorvo
warns the same diversity *"can lead to variation in ranging accuracy as tags move in harsh
multipath environments, such as where there are many reflections in confined indoor spaces,
e.g. narrow corridors."*

A hoistway is a narrower, more reflective, fully metal-lined version of that example, and a
pit is worse again. This is the vendor stating that ranging accuracy degrades in precisely the
environment Phase 1 must characterise. Read it as raising the weight on that gate.

The stated mitigation is a Phase 4 layout constraint: keep the module at least 1 cm from the
carrier PCB edge, which suppresses the horizontally polarised component and improves multipath
resilience.

### Firmware path

**Boards ship blank** — the SEGGER J-Link pack must be installed and UCI firmware flashed via
J-Flash Lite before anything runs.

- **[Uberi/DWM3001C-starter-firmware](https://github.com/Uberi/DWM3001C-starter-firmware)** to
  move quickly — community-maintained, Dockerised build, runs directly on the onboard
  nRF52833, considerably simpler than the official SEGGER Embedded Studio flow.
- **Official DW3_QM33_SDK** (v1.1.1, Aug 2025, FiRa 2.0) for CFO correction and the
  diagnostics registers.

### Calibration — mandatory, with a trap

Calibration is **mandatory** on first use and again whenever the SDK is upgraded, since
calibration data is not guaranteed compatible across firmware versions. It is cheap: a preset
file ships for this board (`dual-hoe_non_aoa.json`) and auto-calibration is available in the
Qorvo One TWR GUI.

> **The trap.** Calibration lives in NVM and survives power cycles and firmware updates, but
> **a chip erase silently invalidates it.** The resulting error is a constant bias, invisible
> to a differential detector. Record calibration state alongside every capture.

### Known gap

The DWM3001C carries two integrated antennas with **no u.FL connector**, so it cannot answer
the antenna-orientation question in Section 9. That requires the QM33120WDK1 daughterboards or
a chip-down fixture, and belongs to Phase 2. The DW3110 is additionally **non-PDoA** — no
angle-of-arrival work is possible on this kit, which this design does not need.

---

## 13. Phase 1 bring-up procedure

Ordered deliberately: each stage exists to make the next interpretable. **You cannot diagnose
a bad result in a hoistway unless you already know what a good result looks like on the
bench.** Do not skip Stage 2.

### Stage 0 — before the parts arrive

| Step | Detail |
|---|---|
| Install SEGGER J-Link pack | Boards ship unprogrammed |
| Obtain DW3_QM33_SDK and the starter firmware | Needed for CFO correction and diagnostics |
| Source battery leads | Confirm the DK's battery connector type; make leads for the untethered devices |
| Decide whether any run reproduces the existing algorithm | If so, plan for a BMA456 rather than substituting the LIS2DH12 |

### Stage 1 — out of box

1. Flash UCI firmware to all boards (device `NRF52833_xxAA`, SWD, 4000 kHz).
2. **Physically label every unit** and record identifiers. Every capture references them and
   identical boards are trivially confused.
3. Import the preset calibration `dual-hoe_non_aoa.json`.
4. **Override the Channel 5 antenna delay with `16390`** — engineering samples, OTP value is
   wrong. The step most likely to be skipped and least likely to announce itself later.
5. Run auto-calibration on every unit.
6. Confirm ranging at a known distance before writing any code.

> **Gate.** Do not proceed until reported range at a tape-measured distance agrees within a
> few centimetres. Larger error means the antenna delay override did not take effect.

### Stage 2 — establish the bench reference

Open air, line of sight, before going near a hoistway:

- **Static noise floor** at roughly 1, 5, 10, 20 and 40 m. Record σ at each — the baseline
  every shaft number is compared against.
- **Quiet-period false-motion rate.** With everything static, run the change detector and
  count how often it fires. This is the clean-environment control for the Phase 1 gate, and
  it must be near zero before shaft data means anything.
- **Orientation sweep.** At fixed distance, rotate a unit through the three principal planes,
  logging range error and first-path metrics. The only orientation data obtainable on this
  kit.
- **Absolute accuracy check** at a tape-measured distance, confirming the Stage 1 override.

### Stage 3 — instrument for capture

Move off the GUI to the SDK or starter firmware. Configure SS-TWR with CFO correction, the
tethered device as initiator, and the logging and CIR throughput plan from Section 12.

### Stage 4 — hoistway captures

| Run | Purpose |
|---|---|
| **Locked out, overnight, nothing moving** | **The primary gate.** Every alarm is unambiguously false |
| Pit device plus counterweight, full descent | The driving use case, in the worst RF environment |
| Full travel, both directions | Range envelope, link continuity over the whole shaft |
| Inspection speed, repeated stops and starts | The realistic service case |
| Car stationary, induced bumps and rope movement | Discrimination — the runs the current state machine fails |
| Deliberate occlusion | Link-loss behaviour, recovery, liveness annunciation |

### Capture metadata — record with every run

- Unit identifiers, and which was initiator
- Firmware version and build hash
- Calibration state: preset file, auto-calibration date, antenna delay in use
- Channel, PRF, preamble length, TX power
- Physical mounting: position, orientation, distance from steel
- Elevator configuration, nominal speed, and what was moving

---

## 14. Open questions

**Q1 — Group formation.** How do N devices know they are on the same job, with no setup
screen? Needs a zero-config answer before Phase 3.

**Q2 — Arm and disarm model.** With no intent suppression, arm/disarm is the only control a
mechanic has. Manual? Automatic timeout? Both?

**Q3 — Annunciation modality.** Is audible sufficient in a hoistway with machine noise, or
are visual and haptic required? Affects enclosure, power and cost. Section 7.

**Q4 — Detection threshold and window.** Deliberately unset. Phase 1 data determines these;
do not guess them now, and never set them by raising the threshold to quiet false alarms.

**Q5 — In construction, before the counterweight exists, which hazard is instrumented?**
Temporary hoist, hoisted materials, adjacent live car, or the platform itself. Determines how
many devices ship.

**Q6 — Traction only, or must hydraulic and MRL configurations be covered?** This document
assumes traction with a roped counterweight.

**Q7 — Charging workflow.** Several devices per mechanic, charged with their other tools.
Dock, individual USB-C, or something else.

**Q8 — Does the candidate module expose first-path diagnostics?** Section 5 permits false
alarms to be attacked only by discrimination, and every permitted discriminator needs
`FP_AMPL`, `FP_INDEX` or `maxNoise`. A module that reports a range and nothing else cannot
implement the alarm logic in this document — it can only be desensitized, which is rejected.
**This is a module selection criterion, not a detail**, and it applies to the MaUWB AT
interface in Phase 0 as much as to any production candidate.

### Resolved since Rev A

- *Is the hazard device semi-permanent or per-job?* — **Per-job.** Devices are placed and
  retrieved, which is why the architecture is symmetric.
- *Minimum detectable motion* — ~1 cm with averaging is achievable; detection margin is
  ~100× at inspection speed. No longer a risk to the concept. The real question became the
  false-motion rate.
- *Where must the alarm be audible?* — **Every device alarms.**

---

## 15. Scope boundary

> **Hold this line explicitly.** This is a **secondary alert aid, not a protective device**.
> It does not substitute for lockout/tagout, the pit stop switch, or the
> mechanic's-protection provisions of ASME A17.1, and the product framing, packaging and
> documentation must say so plainly.
>
> This matters more given what motivates the project, not less. A device people trust in a pit
> has to be honest about what it does and does not guarantee, and it must never become a
> reason to skip a procedural protection.

Three deliberate exclusions follow, and each also keeps the engineering simpler:

- **No interface to the permanent installation.** No controller integration, no wiring, no
  modification to the elevator. Battery and magnet mount only. This is what keeps the device
  out of A17.1 certification scope. **This binds prototypes from the first bench day**, not
  only shipped product — a Phase 0 unit is not permitted near the controller, the safety
  circuit or the brakes either.
- **No commissioning.** No site survey, no surveyed positions, no field calibration.
  Zero-config or it will not be used.
- **No position, only distance.** Absolute position is not needed, not measured, not claimed.
