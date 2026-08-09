# The deployment, and what it changes

**Date:** 2026-08-09
**Source:** Dave, 2026-08-09. This is product context, not something derivable
from the code or the captures, and several conclusions in
`falcon_analysis_2026-08-06.md` were reached without it.

Extends §1 of `falcon_state_of_project_2026-08-07.md` ("What the device is
for"), which is correct but incomplete. Section references below point at the
analysis document unless marked *(SoP)*.

---

## 1. The primary use case, as stated

1. The mechanic works from the **cartop**.
2. From there they place the device **on top of the counterweight**.
3. They then operate the elevator on inspection.
4. The device rides the **opposing counterweight** for the duration of the job.
5. It serves as an additional warning during travel, **identifying
   counterweight location** and warning of it.

Secondary use case: general motion warning when operating equipment — the
device placed on anything that should not move. That is the independent-detector
role §1 *(SoP)* describes, and the failure table there applies to it as written.

The two use cases have genuinely different requirements. Tuning that serves one
does not automatically serve the other, and where they conflict the primary one
should win.

---

## 2. 🔴 Manual silence is not available in the primary use case

**The device is on the counterweight. The mechanic is on the cartop. It cannot
be reached until the job ends.**

§14.9 concluded that the honest options for a missed arrival are to let the
alarm run to `LATCH_FAILSAFE_MS` or to have the mechanic silence it, and
preferred the latter because it "cannot fail dangerously". That option does not
exist here. There is no button within reach, and no way to get to one without
riding to the counterweight.

Consequences:

- **"Run to the failsafe" is not an acceptable outcome**, not merely an annoying
  one. 300 s of alarm on an unreachable device during a job is what gets the
  product left in the van, and a device that is not deployed protects nobody.
- **Automatic settle becomes a hard requirement.** Dave's figure is **3 seconds**
  for a false fire. That is a product constraint, not a preference.
- Any future manual silence for this use case has to be remote, not a button on
  the housing.

This is the single largest change from the new context, and it inverts the
recommendation in §6.1 *(SoP)* and §14.9 for the primary deployment.

---

## 3. ⚠️ SUPERSEDED — placement is not a false-fire source

**Superseded 2026-08-09** by the deployment sequence in
`falcon_zxy_logic_2026-08-09.md` §7.1: the device is **placed first and powered
second**, so the handling transient happens before the FSM is armed and never
reaches an armed detector. The section below is kept because its reasoning
still applies to RETRIEVAL, which happens with power on, and because the 3 s
settle requirement it motivated is unchanged — the sources are just different
ones (building vibration, adjacent equipment, bumps).

---

The mechanic leans over from the cartop, sets the device down on the
counterweight, and lets go. That is a handling transient at close range, and it
will trip any departure detector worth having — the bench run on 2026-08-07
latched on a hand lift measuring 0.186 m/s, and a placement is rougher.

So the sequence every job begins with is: place, spurious latch, and then
however long the device takes to work out that nothing is travelling. Today
that is up to 300 s. This is what the 3 s requirement is really about, and it
is a normal event on every single deployment rather than an edge case.

Retrieval is the mirror image and has the same problem.

---

## 4. The alarm is a BEACON — the mechanic ranges it by ear and eye

**Correction, same day.** An earlier draft of this note argued that constant
alarming during travel would train the mechanic to tune the device out, and
asked whether travel and hazard warnings should sound different. That was
wrong, and it came from treating the alarm as a repeated warning.

The audio and the chase LEDs ARE the information channel. The mechanic locates
the counterweight by hearing it and seeing it, and judges range from how loud
and how visible it is. During travel the alarm is not a repeated alert being
habituated to — it is a continuously tracked signal. And the mechanic knows
whether they commanded the move, so there is no ambiguity for them to resolve.

Three consequences that DO follow, and they are sharper than the point they
replace:

**a) A mid-travel release is worse than §14.4 states.** That table rates it
catastrophic because the alarm stops while the counterweight is still moving.
With the beacon reading it is worse: the mechanic is actively tracking an
approaching mass by sound, and the source goes silent mid-approach. A cue that
is being relied on and then disappears is more dangerous than one that was
never there.

**b) A false fire broadcasts false position.** A stationary device that sounds
tells the mechanic the counterweight is moving when it is not — on a device
they cannot reach. That is misleading, not merely irritating, and it is a
better argument for the 3 s settle (§2) than nuisance was.

**c) 🔴 The beep pattern is a UX parameter, not a sensing knob.** The 200 ms
on / 300 ms off cycle in `check_for_buzzer_alert()` is simultaneously the
beacon AND what blanks the accelerometer for ~50% of wall clock. §5 item 7
recovered sampling to ~45% coverage by fixing the blanking, and it is tempting
to shorten the beep further to buy samples for an X/Y confirm inside a 3 s
window. **That trade is not available if the pattern is what the mechanic
reads.** It pushes the confirm toward the sensor's own no-motion engine, which
needs no host samples at all.

**d) Beacon loudness and sensor integrity are in direct tension.** §5 and §11:
the piezo couples mechanically into the accelerometer, measured triggering
any-motion continuously at threshold 96. Louder carries further and reads
better at range — and corrupts the sensor more. Firmware can only blank around
this; the real fix is mechanical isolation between piezo and sensor.

- [ ] Is the distance cue purely passive — physical attenuation of a constant
      beacon — or is active encoding wanted (faster beeping when closer)? The
      code today is a fixed 200/300 ms pattern with no distance modulation.
      Active encoding would require position sensing and brings §6 back.
- [ ] Hoistway acoustics are a reverberant shaft, closer to a duct than free
      field, so range-by-ear is coarse. Worth knowing how precisely the
      mechanic actually needs to place it.

## 5. Counterweight roughness inverts from risk to signal

§14.5 and §5.1 *(SoP)* treat the rougher counterweight ride as the largest open
risk, because it pushes both Z-based release paths toward firing during travel.

For the **X/Y lateral-vibration** approach under exploration (see below) the
same roughness is the signal. A counterweight running on its guides produces
lateral excitation; a parked one does not. What threatens the Z architecture
feeds this one.

That does not retire §5.1 — the Z paths are still exposed, and logging on a
counterweight is still unsafe (§5.1), so roadmap item 11 is still the
precondition for characterising any of it.

Reading note for counterweight logs: **the counterweight travels opposite to
the car**, so a downward transient on the device corresponds to the mechanic
driving up.

---

## 6. The barometer: still wanted, but NOT for the location feature

**Correction, same day.** An earlier draft made this the top question for Biju,
on the reading that the device had to compute and report counterweight
position. It does not — §4: the device is a beacon and the mechanic does the
ranging. The location feature already exists and does not need a sensor.

The DPS310 case therefore rests on its original merits, which are still good
but no longer include delivering a specified feature. It stays on the list; it
comes off the top of it.

At ~0.002 hPa resolution relative height resolves to a few centimetres, so
populating it would:

- make constant velocity directly observable, retiring §3's central constraint,
- give an arrival signal that does not depend on a transient, retiring §14.7,
- and give the X/Y work below a ground truth to be validated against.

- [ ] Is the DPS310 permanently depopulated, and what would it take to populate
      it? Not the top question any more, but the answer unlocks several things
      at once.

---

## 7. Where this leaves the current work

| Item | Effect of the new context |
|---|---|
| Windowed velocity integral (`velocity.h`) | Unchanged and still unarmed. Still gated on sample rate |
| X/Y lateral vibration | **Promoted.** Better contrast expected on a counterweight, and it is the only automatic path to the 3 s settle |
| §14.9 "run to failsafe or manual silence" | **Both options weakened.** Manual silence unavailable; failsafe unacceptable, and a stationary beacon broadcasts false position (§4b) |
| Driver variant swap (finding 1) | **More urgent.** No-motion on X/Y needs the independent engines, and flash is at 86.3% |
| Roadmap 11, on-device black box | Unchanged and still the precondition for counterweight characterisation |
| DPS310 | Still wanted for §3/§14.7 and as ground truth — but NOT needed for the location feature. See §6 |

**Proposed FSM shape for the 3 s requirement** (not implemented):

```
MONITORING --Z transient--> PROVISIONAL   (alarm sounds IMMEDIATELY)
      within 3 s:  X/Y elevated?  yes --> CONFIRMED (hold, no timeout)
                                  no  --> release, back to MONITORING
      CONFIRMED --X/Y returns to parked floor--> arrival
```

The alarm must sound on the Z latch, not after the confirm, so a real departure
still warns in under a second. The 3 s window only decides whether to KEEP
alarming, which means a wrong veto costs 3 s of warning rather than none — bad,
but recoverable, and it is why the confirm threshold has to sit just above the
parked floor rather than partway to cruise level.

`STOP_CONFIRM_MS` (5 s) exceeds the 3 s budget on its own, so the nuisance path
has to bypass it. That constant exists for real arrivals with levelling and
should not govern a placement transient.

---

## 8. Behavioural answers, 2026-08-09 — and the spec they imply

Asked directly. Answers:

| Question | Answer |
|---|---|
| What is the hazard? | **Both** losing track of the cwt during a commanded move, and uncommanded movement, roughly equally |
| Beacon must go quiet within? | **A second or two** of the counterweight stopping |
| Beacon during a move? | **The whole of every move** — no proximity modulation |
| On a placement false fire, the mechanic? | **Stops using the device** |

### 8.1 The spec

Answers 2 and 4 are one requirement seen from two sides. A beacon still
sounding after the counterweight stopped tells the mechanic it is still
moving — a position lie, exactly as a false fire is (§4b). So:

> **The beacon must be ON if and only if the counterweight is moving, with
> <= 2 s of lag in either direction.**

Measured against that, the current firmware passes on turn-on (any-motion
latches in well under a second) and fails on turn-off: `STOP_CONFIRM_MS` is
5 s at best and `LATCH_FAILSAFE_MS` is 300 s at worst. Even a perfectly
detected arrival misses the budget.

"Both hazards equally" turns out NOT to require the device to tell commanded
from uncommanded travel. The beacon behaviour is identical either way, and the
mechanic supplies the context — they know what they commanded. What it does
require is that departure detection stays reliable for the uncommanded case,
where the beacon is the only warning, while beaconing stays continuous for the
commanded one.

### 8.2 🔴 X/Y is no longer exploratory, and the sensor must do it

§14.9 concluded arrival cannot be detected from the z accelerometer at any
threshold, because §14.7's arrival was smaller than the parked noise floor.
Requiring it within 1-2 s makes that strictly harder. Z cannot meet this, and
neither can the windowed velocity integral, which also needs a transient.

X/Y vibration cessation is a LEVEL test rather than a transient test, so the
softness of the stop does not weaken it. It is the only candidate left.

The latency budget then decides the implementation. Declaring "lateral
vibration has ceased" needs perhaps 0.5-1 s of observed absence:

| Path | Samples in a 1 s window |
|---|---|
| Host-polled, 3.13 Hz, ~50% buzzer blanking | **2-3** -- hopeless |
| Sensor no-motion engine, 50 Hz | **50** -- ample |

**So the hardware no-motion engine on X+Y becomes an architectural
requirement, and the driver variant swap (datasheet review, finding 1) becomes
the blocker for the product requirement rather than a flash saving.**

Two properties fall out for free:

- No-motion asserts only when ALL enabled axes are quiet, so any lateral motion
  on either axis holds the alarm — conservative in the correct direction.
- It dissolves the levelling problem. `STOP_CONFIRM_MS` was raised 2 s -> 5 s
  because levelling qualified as "stopped" and produced a 73 s alarm on a
  stationary car. During levelling the counterweight is genuinely moving, so
  X/Y stays live and the alarm holds with no timer at all.

### 8.3 The windowed velocity integral is demoted

Its two jobs were low-speed departure and arrival by conservation. Any-motion
does the first faster; the second needs a transient and is noise-limited to
~50 fpm at the current sample rate. Neither survives a 2 s turn-off
requirement.

It stays in the tree, unarmed, as an independent second opinion and as
instrumentation for characterising runs. It is no longer the centrepiece, and
the sample-rate work it depends on drops below the driver swap.

### 8.4 The risk that could kill this

If a counterweight runs smoothly enough that cruise lateral vibration sits at
the parked floor, X/Y no-motion releases the alarm mid-travel — the
catastrophic failure, and precisely what the stillness-backstop block in
`movement_service.h` records from the z-based attempt.

Answers 2 and 4 push the settle faster, which makes this worse. **The tension
between them is now the central safety trade in the project, and it has to be
resolved by measurement rather than judgement.** §14.5 says the counterweight
ride is rougher, which is the favourable direction, but lateral vibration has
never been measured on one.

### 8.5 Revised priority

1. **Driver variant swap** — blocks the settle path, and frees 4.9 KB with
   flash at 86.3%
2. **X/Y characterisation** — parked vs travelling contrast, cartop first,
   counterweight when roadmap 11 allows it
3. **No-motion on X+Y** for settle and stop, against measured thresholds
4. Sample rate / 100 Hz — still wanted, no longer the critical path
5. Velocity integral arming — only if it still earns its place afterwards

---

## 9. 2026-08-09 decision: turn-on may take 1-2 s

Dave, asked directly, accepted a 1-2 s turn-on in exchange for a quieter
device. Every other requirement in §8 tightens the turn-OFF; this is the one
piece of slack in the spec, and it is the piece that makes the rest tractable.

The device changes shape: from "sound, then decide whether to stop" to
"decide, then sound". That REMOVES the placement false fire rather than
settling it after the fact, which is the difference between §8's
"stops using the device" and a device that never misbehaved in the first place.

### 9.1 One engine, three jobs

No-motion on X+Y, duration ~1 s:

```
MONITORING
  Z transient (any-motion) --> PROVISIONAL, SILENT
      at t = 1.5 s:  X/Y no-motion asserted?
            yes --> positive evidence of stillness --> discard, never sounded
            no  --> BEACON ON
BEACON ON
  X/Y no-motion asserts --> BEACON OFF  (~1 s after the counterweight stops)
```

| Requirement (§8.1) | Mechanism |
|---|---|
| Placement never sounds | handling stops -> no-motion asserts ~1.3 s -> confirm at 1.5 s suppresses |
| Beacon off <= 2 s after stopping | no-motion duration ~1 s + poll latency |
| Beacon on for the whole move | a travelling counterweight never satisfies no-motion |
| Levelling holds the alarm | counterweight genuinely moving -> X/Y live -> no assert |

**Use NO-motion, not any-motion.** Any-motion requires the threshold exceeded on
CONSECUTIVE samples, and vibration oscillates through zero -- it would not hold
for 50-100 consecutive samples even while genuinely travelling. No-motion's
duration applies to the QUIET condition, which a still device satisfies
continuously and a moving one does not. No-motion also uses consecutive-sample
slope rather than a latched reference, so the stuck-reference risk in the
datasheet review's finding 3 does not apply to it.

### 9.2 🔴 Polarity is the safety property

Confirm-before-sound creates a new failure mode: if X/Y confirmation fails on a
smooth counterweight, the device NEVER SOUNDS -- silent, where z alone would
previously have alarmed. That is worse than what it replaces.

The test must therefore be written to **suppress only on positive evidence of
stillness**, never to sound only on positive evidence of motion:

- no-motion ASSERTED -> affirmatively told it is still -> suppress
- no-motion NOT asserted -> travelling, OR the engine is misconfigured, dead,
  or the contrast is too low -> **BEACON**

Every failure of the X/Y path then produces a sounding device. Written the
other way round -- "sound if X/Y confirms motion" -- every failure produces a
silent one. The two look equivalent in a state diagram and are opposite in
consequence.

### 9.3 The cost, recorded deliberately

At inspection speeds a 2 s delay is roughly **0.2-1.5 m of unwarned approach**
(20 fpm to a 150 fpm inspection limit). Against a mechanic who must notice,
orient and react anyway that is probably inside the noise, but it is a real
reduction in warning for the uncommanded-movement half of §8's hazard answer
and is recorded here as a conscious trade.

### 9.4 The confirm window is UNBLANKED -- correcting §4c

§4c and §8.2 both assumed the X/Y decision would suffer ~50% buzzer blanking.
For the STOP decision that holds -- the beacon is sounding then. **For the
CONFIRM decision it does not: the beacon is silent during the confirm window,
by construction.**

So the confirm gets the full sample rate: ~5 samples in 1.5 s at 3.13 Hz,
unblanked, against 2-3 for the stop decision. Thin for a level test, but not
hopeless.

Which splits the work usefully:

| Decision | Blanked? | Samples | Needs driver swap? |
|---|---|---|---|
| **Confirm** (placement rejection) | no | ~5 in 1.5 s | maybe not |
| **Stop** (beacon off) | yes, ~50% | 2-3 per s | **yes** |

The highest-priority problem in §8 -- placement false fire, the one that gets
the device abandoned -- may be prototypable on host-polled X/Y before the
driver swap. Stop detection cannot be.

- [ ] Measure the parked-vs-travelling X/Y contrast. Still the gate on all of
      this, and still unmeasured.
