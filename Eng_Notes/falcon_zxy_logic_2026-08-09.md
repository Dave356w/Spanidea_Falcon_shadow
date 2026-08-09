# Z + X/Y confirmation logic — design, before any hardware

**Date:** 2026-08-09
**Status:** design only. Nothing here is implemented and nothing is measured.
**Depends on:** `falcon_use_case_2026-08-09.md` §8 (the spec), §9 (the 1–2 s
turn-on decision). Section refs otherwise point at
`falcon_analysis_2026-08-06.md`.

The requirement this serves, from use-case §8.1:

> The beacon must be ON if and only if the counterweight is moving, with
> ≤ 2 s of lag in either direction.

Z answers *did something happen*. X/Y answers *is it still happening*. Neither
axis can do both, and the split is the whole idea.

---

## 1. Why the two axes divide this way

| | Z | X / Y |
|---|---|---|
| Gravity component | 1 g pedestal | **none** |
| At rest | 1 g | ~0 |
| At constant velocity | 1 g — indistinguishable (§3) | lateral vibration from guides, rails, ropes |
| Good for | transients: departure, arrival | **levels**: is it travelling or not |
| Bad for | telling cruise from parked — provably (§3) | telling a departure from a bump |

§14.7's gentle arrival was smaller than the parked noise floor *on Z*. An X/Y
level test does not care how gently the machine set down, because it is not
looking for the stop — it is looking for the absence of travel afterwards.

The ⛔ stillness-backstop block in `movement_service.h` forbids a rest
detector. Every number in it is Z, and its argument is explicitly the 1 g
pedestal. That reasoning does not transfer to gravity-free axes. **Its safety
lesson does**: what is proposed here is still a rest detector that releases the
alarm, and it was removed last time after a mid-ride release on real hardware.
If the measurements below come back marginal, that block wins.

---

## 2. States ⚠️ SUPERSEDED by §8.2 — kept for the reasoning

```
MONITORING  --Z any-motion-->  PROVISIONAL (silent, t=0)

PROVISIONAL, evaluated ONCE at t = CONFIRM_MS:
      XY_still AND |w| low   -->  SUPPRESSED
      otherwise              -->  BEACON

SUPPRESSED (silent, watching SUPPRESS_WATCH_MS):
      XY becomes active      -->  BEACON     (late, but not never)
      |w| > VEL_DEPART       -->  BEACON
      watch elapses          -->  MONITORING

BEACON (min MIN_BEACON_MS, no timeout):
      XY_still on N consecutive polls  -->  MONITORING
      LATCH_FAILSAFE_MS                -->  MONITORING (fault, logged)
```

**The confirm is evaluated once at the deadline, not continuously.** That is
deliberate and forgiving in the correct direction: a unit still being handled
at 1.4 s has not accumulated the quiet no-motion needs, so it
beacons briefly and releases about a second later. ~1 s of sound, inside the
3 s budget, and no special case needed.

**Suppression is provisional too.** A wrong suppress is otherwise silent for the
entire run — see §4.1. `SUPPRESSED` exists solely so that case degrades to a
late beacon rather than none.

---

## 3. 🔴 Polarity, and the failure it does NOT cover

**Correction to an earlier claim.** It was stated that if the X/Y contrast were
too low the engine would fail to assert and the device would beacon. That is
backwards. Low contrast means the lateral signal sits BELOW the threshold, so
no-motion DOES assert — it reports "still" — and the device suppresses. Silent,
during a real move.

The polarity rule is still worth having, for a different failure:

- no-motion **asserted** → affirmatively told it is still → suppress
- no-motion **not asserted** → travelling, **or the engine is dead,
  misconfigured, or never armed** → **BEACON**

A dead engine's status bit reads 0, which is "not asserted", which sounds. So
the rule covers engine failure. It does **not** cover a threshold set too high
for the machine. That is a tuning failure and needs a tuning answer:

> ### Threshold rule
> **Set `XY_STILL` from the parked floor, not from the travelling level.**
>
> The confirm only ever runs after a Z transient, so a threshold set too low
> cannot raise a false alarm on its own — it can only fail to suppress, and
> failing to suppress means sounding, which is safe. So push it as low as it
> can go while still reliably asserting on a genuinely parked device.
>
> **The number must come from parked in a LIVE HOISTWAY** — other car running,
> HVAC, building traffic — not from a bench. A bench-derived threshold will be
> too low and the device will sound constantly in a real building.

This also relaxes the pass mark given earlier. Not "cruise ≥ 3× parked", but
"the quietest travel exceeds the noisiest parked, with margin".

---

## 4. Scenario walk

| # | Scenario | Z | X/Y | Result |
|---|---|---|---|---|
| 1 | Handling, clean (retrieval, bump, adjust — *not* placement, §7.1) | transient | burst ends ~0.3 s | no-motion asserts ~1.3 s → **suppressed, never sounds** ✓ |
| 2 | Handling, prolonged | transient | still active at 1.5 s | beacons, releases ~1 s later — **~1 s of sound** ✓ |
| 3 | Real departure, rough cwt | transient | continuous | **beacon at 1.5 s**, holds ✓ |
| 4 | **Real departure, smooth cwt** | transient | below threshold | **suppressed — silent during a real move** ✗ see 4.1 |
| 5 | Bump while parked | transient | brief | suppressed ✓ |
| 6 | Building vibration too high | — | never asserts | every Z transient beacons — noisy but **safe** |
| 7 | Stop mid-run | transient (maybe) | goes quiet | no-motion asserts ~1 s → **beacon off ~1.2 s** ✓ |
| 8 | Brief pause then continue | — | quiet then active | beacon off then on — **telling the truth** ✓ |
| 9 | Smooth patch mid-travel | — | dips below threshold | **beacon drops mid-travel** ✗ see 4.2 |
| 10 | Buzzer coupling while beaconing | — | held above threshold | **never releases** ✗ see 4.3 |

### 4.1 Scenario 4 — smooth counterweight, silent device

The catastrophic one. Mitigated, not solved, by three things:

- the threshold rule in §3 (as low as the building floor allows),
- `SUPPRESSED` re-arming on either late lateral activity or a rising velocity
  integral — a late beacon instead of none,
- and the velocity integral being genuinely independent of vibration: a
  placement has near-zero net velocity change across the window, a real
  departure has a growing one.

The integral is weak at 1.5 s — perhaps 0.05 m/s against a measured 0.127 m/s
parked peak — which is exactly why it belongs on the `SUPPRESSED` escape, where
it has seconds to accumulate, and **not** in the confirm decision itself.

### 4.2 Scenario 9 — mid-travel dropout

Hysteresis. Require `XY_still` on N consecutive host polls before releasing,
not on the first assert, and enforce `MIN_BEACON_MS`. The engine's own duration
plus 2 polls at 250 ms puts release at ~1.5 s, still inside the 2 s budget.

### 4.3 🔴 Scenario 10 — the buzzer fights the release

**The largest hole in this design.**

§11 measured the piezo triggering any-motion *continuously* at threshold 96 on
Z. The release path needs a threshold near the parked floor — far lower than 96
— on axes with no 1 g pedestal to swamp the coupling. And no-motion wants ~1 s
of continuous quiet while the beep cycle offers only 300 ms gaps.

If the buzzer holds X/Y above `XY_STILL` while beaconing, **no-motion never
asserts, the beacon never releases, and every run reaches the failsafe** —
total failure of the requirement this design exists for.

This splits the architecture along the blanking boundary:

| Decision | Beacon | Mechanism | Blocked by |
|---|---|---|---|
| **Confirm** | silent | hardware no-motion, uncorrupted | contrast only |
| **Release** | sounding | buzzer coupling corrupts it | **isolation, or host sample rate** |

So the release path is **not** unblocked by the driver swap. It needs
mechanical isolation between piezo and sensor, or enough host sample rate to
decide from the beep-off gaps alone (2–3 samples/s today — nowhere near).

Use-case §4d listed piezo/sensor isolation as a background tension. It is now
on the critical path.

---

## 5. Parameters

| Name | Proposed | Where it comes from |
|---|---|---|
| `CONFIRM_MS` | 1500 | use-case §9, the accepted turn-on delay |
| `XY_STILL` | **unmeasured** | parked floor in a live hoistway, §3 |
| `NOMO_DURATION` | ~1000 ms | stop latency budget minus poll latency |
| `RELEASE_POLLS` | 2 | hysteresis, §4.2 |
| `MIN_BEACON_MS` | 1500 | anti-flicker, §4.2 |
| `SUPPRESS_WATCH_MS` | 10000 | late-beacon window, §4.1 |
| `VEL_DEPART` | 0.255 today | `velocity.h`; falls with sample rate |

`STOP_CONFIRM_MS` (5 s) is superseded for this path. It was raised 2 s → 5 s to
hold through levelling; X/Y handles levelling structurally, because a levelling
counterweight is genuinely moving and never satisfies no-motion.

---

## 6. What must be measured before any of this is written

1. **Parked X/Y floor in a live hoistway.** Sets `XY_STILL`. A bench floor is
   not a substitute (§3).
2. **Travelling X/Y level, slowest speed, on a counterweight.** The other side
   of the same threshold. §14.5 says the counterweight is rougher, which is the
   favourable direction, but it has never been measured.
3. **X/Y under the buzzer, unit stationary.** Decides §4.3, and therefore
   whether the release path is achievable at all in the current mechanical
   design. **Cheapest of the three and the most likely to kill the approach —
   do it first.** It needs no hoistway: a bench, the alarm forced on, and the
   x/y logging build already compiled.

Items 1 and 2 need roadmap 11 (on-device recording) to be done safely on a
counterweight — §5.1: a serial cable to a moving counterweight means being in
the hoistway, which is the hazard the device exists to warn about.

Item 3 needs none of that.

---

## 7. Deployment sequence, 2026-08-09 — and in-situ calibration

Dave's stated sequence:

1. Mechanic accesses the hoistway from the cartop
2. Operates **down to the counterweight**
3. Places the falcon on the cwt frame and **POWERS ON**
4. Falcon boots, calibrates, and **signals ready to use**
5. Mechanic performs work or maintenance
6. Falcon alarms, visual + audible, whenever in motion
7. Start within 1-2 s, hold through travel, stop within 1-2 s

Other stated use cases: reroping (cwt or car, unintended-motion alert for the
crew), hoisted materials (alert to all), construction running platform (cwt
motion alert for everyone in the hoistway). Current project phase is
establishing the **working range** of speeds and accel/vibration that can be
acknowledged, with 1-3 s set/reset and false-alarm reset that does not annoy or
cause mistrust.

### 7.1 Placement is NOT a false-fire source

**Correction to `falcon_use_case_2026-08-09.md` §3**, which called the
placement transient unavoidable and treated it as the thing the 3 s settle
existed for. It is not: the device is **placed first and powered second**, so
the handling happens before the FSM is armed and never reaches an armed
detector.

The remaining false-alarm sources are different, and none of them is a handling
transient:

- building vibration while parked
- adjacent equipment: another car in the bank, work elsewhere in the hoistway
- the mechanic bumping or adjusting the unit mid-job
- retrieval at the end of the job (the mirror of placement, but WITH power on)

### 7.2 ✅ In-situ calibration of `XY_STILL`

§3 said the threshold must come from "parked in a live hoistway, not a bench",
and treated that as a field-measurement dependency. The deployment sequence
removes the dependency: **boot and calibration happen at rest, on the actual
counterweight, in the actual building, every single deployment.**

So the device measures its own parked floor and sets its own threshold, adapted
per site and per machine. During the existing 10 s `STATE_CALIBERATION`,
accumulate x/y alongside the z zero, and write the result into the sensor's
no-motion threshold before arming.

**The estimation error falls in the safe direction**, which is what makes this
trustworthy enough to automate:

| Estimate | Threshold | Consequence |
|---|---|---|
| Floor UNDER-estimated (too few samples) | too low | never suppresses -> beacons on every z transient -> **annoying, safe** |
| Floor OVER-estimated (loud event during calib) | too high | travel reads as still -> **suppresses during a real move -- dangerous** |

Short windows tend to underestimate a peak, so sampling error lands safely. The
dangerous direction comes from a one-off loud event inside the calibration
window, so the statistic must be robust:

> Use the **median of per-second maxima**, not the global maximum. One loud
> second then cannot inflate the threshold.

10 s at 3.13 Hz is ~31 samples, thin for a safety-critical threshold.

- [ ] Would the mechanic tolerate 20-30 s of calibration? They are standing
      next to it, so this is a UX question, not a technical one.
- [ ] Refuse to arm if the calibration window shows movement. A cwt that moved
      during those 10 s produces a threshold wrong in the DANGEROUS direction,
      and declining to arm is the correct response.

### 7.3 "Signals ready to use" is the only rich channel

The one moment the mechanic is beside the device. Today it is a chirp plus a
chase-LED sweep meaning "booted". It should carry the calibration result:

| Signal | Meaning |
|---|---|
| Ready, floor clean | normal arm |
| Ready, floor noisy | armed, but this site is marginal and the unit will be twitchy |
| **Not ready** | calibration failed, sensor fault, or it moved during calibration |

This is also where the BMA456 self-test and the fault registers from the
datasheet review (findings 6 and 7) should surface. A boot self-test that fails
is only actionable at the one moment someone is holding the unit.

### 7.4 Reroping is the hardest stated use case

Hoisted materials and the construction platform should be comfortable —
swinging loads and running platforms generate ample lateral excitation.

**Reroping is the outlier.** A car or counterweight moved slowly by hand or
come-along, possibly a few feet per minute, with almost no guide-rail
excitation to detect. It is simultaneously:

- the worst case for an X/Y confirm (little lateral signal), and
- a case where motion is entirely unexpected, so a missed alarm is
  unambiguously catastrophic.

It may have to be a documented exclusion — "not for use during reroping
operations at hand speed" — rather than something threshold tuning covers.
Worth settling early, because designing for it would constrain everything else.

### 7.5 Effect on the state machine

No structural change. Two simplifications and one addition:

- `SUPPRESSED` (§2) is still needed, but its main job shifts from rejecting
  placement (§7.1: it never reaches an armed FSM) to rejecting building
  vibration, adjacent equipment, bumps, and retrieval.
- `CONFIRM_MS` at 1500 ms remains inside the stated 1-3 s set/reset budget.
- `XY_STILL` moves from a compile-time constant to a value **learned at
  calibration**, with a conservative compile-time fallback if calibration is
  rejected.

---

## 8. 🔴 REVISED 2026-08-09 — sound first, release fast

**This supersedes the confirm-before-sound state machine in §2.** Dave's
observation: *short bursts of movement followed by rest could be a factor of
set/reset logic in the initial use case*. It is, and it invalidates the
confirm.

### 8.1 The confirm imposes a minimum movement DURATION

With `CONFIRM_MS` = 1500 ms and a 1 s no-motion duration:

| Burst | Behaviour | Result |
|---|---|---|
| **0.5 s** | quiet from t=0.5, so 1.0 s of quiet by the deadline → no-motion asserted → suppress | **NEVER SOUNDS** |
| **1.0 s** | only 0.5 s quiet at the deadline → beacon at 1.5, release ~2.5 | sounds AFTER the motion ended |
| **2.0 s** | beacon at 1.5 while still moving, release ~3.5 | correct |
| 5 s+ | — | correct |

Anything under ~0.7 s is silent; anything under ~2 s is late. **A counterweight
moving six inches is enough to injure, and that is a sub-second event.**

Bursty movement is not an edge case. Inspection operation is a
constant-pressure button — jog, release, jog. Reroping is move-then-stop by
definition. The confirm delay is worst exactly where the device matters most.

### 8.2 Revised state machine

The confirm existed to suppress the placement transient. §7.1 removed that
justification: the device is powered AFTER placement, so handling never reaches
an armed FSM. With its purpose gone and §8.1 as its cost, it does not earn its
place.

```
MONITORING  --Z any-motion-->  BEACON immediately (~0.2 s)

BEACON (min MIN_BEACON_MS):
      X/Y quiet on N consecutive polls  -->  MONITORING  (~1.5 s after stop)
      LATCH_FAILSAFE_MS                 -->  MONITORING  (fault, logged)
```

**Z sets. X/Y resets.** Nothing else. `PROVISIONAL` and `SUPPRESSED` are
deleted.

Cost: a bump or a burst of building vibration now produces ~2 s of sound rather
than silence. That is **inside the stated 1-3 s reset budget**, and it is
arguably not a cost at all — a device that chirps briefly when genuinely
disturbed is demonstrating that it is alive and sensitive. Mistrust comes from
alarms that correspond to nothing, or that run long.

### 8.3 It removes the worst failure mode

Under confirm-first, §4.1's smooth counterweight meant the device **never
sounded during a real move**. Under sound-first that case alarms correctly.

The smooth-counterweight risk does not vanish — it moves to the RELEASE side,
where X/Y could go quiet mid-cruise and drop the beacon. But a mid-travel drop
is recoverable: z keeps producing transients at rail joints and speed changes
which re-trigger it. A silent device recovers from nothing.

Scenario table (§4) rows 4 and 6 change accordingly:

| # | Scenario | Under sound-first |
|---|---|---|
| 4 | Real departure, smooth cwt | **beacons correctly** — the confirm was the only thing that could suppress it |
| 6 | Building vibration high | beacons on z transients, releases slowly — over-eager, safe |
| 9 | Smooth patch mid-travel | still the top risk. `MIN_BEACON_MS` + N-poll hysteresis + z re-trigger |
| 10 | Buzzer fights the release | **unchanged and still the largest hole** (§4.3) |

§4.3 is untouched by this revision. If the buzzer holds X/Y above `XY_STILL`
while sounding, the beacon never releases, and sound-first makes that worse
rather than better because the beacon now starts sooner and more often.

### 8.4 Reroping no longer needs an exclusion

§7.4 asked whether reroping had to be documented out. Under confirm-first it
did: slow, bursty, hand-driven, minimal guide excitation would have been
near-useless. Under sound-first it is simply a sequence of short bursts, each
setting the beacon on its z transient. **X/Y only ever decides when to STOP,
never whether to START.**

The X/Y contrast question remains, but its consequence softens: on a machine
with poor lateral contrast the device becomes over-eager rather than deaf, and
degrades toward today's latched behaviour bounded by the failsafe.

### 8.5 The working range has two axes

Previously treated as one. The second is only visible because of §8.1:

- **minimum speed** — set by the z any-motion threshold (32 counts = 0.153 m/s²)
- **minimum burst DURATION** — set by z detection latency and `MIN_BEACON_MS`

- [ ] Test protocol must include deliberate short jogs — 0.25 s, 0.5 s, 1 s,
      2 s — at two or more speeds, not only continuous runs. It is the only way
      to establish the duration axis, and it is cheap to add to a hoistway
      session.

### 8.6 Revised parameters

| Name | Proposed | Note |
|---|---|---|
| `CONFIRM_MS` | **deleted** | §8.1 |
| `SUPPRESS_WATCH_MS` | **deleted** | `SUPPRESSED` state removed |
| `XY_STILL` | learned at calibration | §7.2, unchanged |
| `NOMO_DURATION` | ~1000 ms | unchanged |
| `RELEASE_POLLS` | 2 | unchanged |
| `MIN_BEACON_MS` | 1500 | now also sets the minimum-duration floor (§8.5) |
| `VEL_DEPART` | 0.255 | velocity integral demoted further — with no confirm to inform, it is instrumentation only |
