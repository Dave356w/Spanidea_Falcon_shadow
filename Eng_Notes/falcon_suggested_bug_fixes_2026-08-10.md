# Falcon firmware — suggested bug fixes after the 25 Hz field test

**Date:** 2026-08-10  
**Branch reviewed:** `Falcon_Rel_EFT`  
**Status:** Proposed fixes only; no firmware changes are implemented by this note.  
**Evidence base:** 8/10 cartop, counterweight, and in-cab testing plus source inspection of the current branch.

---

## 1. Executive decision

The 25 Hz raw-Z peak detector is a strong fix for **inspection-operation brake pick/set events**. Preserve it.

It does **not** yet prove a general arrival detector. The 350 fpm in-cab test activated correctly but did not reset after the drive's smooth stop because there was no useful brake-set transient. The safe next design is:

1. treat the raw-Z peak as an **arrival candidate**, not final proof of rest;
2. confirm rest with a fresh, time-based X/Y settled condition;
3. keep the movement alarm active until both conditions agree;
4. enter a latched fault state on timeout or sensor failure instead of releasing the alarm.

Do not enable the existing X/Y release path by changing `XY_RELEASE_ARMED` to 1. In its current form it can release independently of Z and is not safe enough for that role.

---

## 2. Priority summary

| Priority | Defect | Current risk | Suggested correction |
| --- | --- | --- | --- |
| P0 | 300 s failsafe enters deceleration/release flow | A sufficiently long run can clear the alarm while still moving | Replace release-on-timeout with `STATE_FAULT_LATCHED`; alarm remains active until deliberate reset |
| P0 | Sensor/configuration failures are not fail-safe | A dead or stale sensor can appear quiet and allow a silent or false stop | Verify initialization, track fresh reads, and latch a fault alarm on sustained errors |
| P1 | Raw arrival peak is gated by `MIN_TRAVEL_MS` | 1–2 s jogs lose their valid stop peak before it becomes eligible | Evaluate an armed raw peak without the fixed travel-time gate |
| P1 | Six-second monitor rearm is blind to departures | Closely spaced inspection jogs can start during a detection blind window | Replace fixed rearm with confirmed ringdown/settle logic and test short restart intervals |
| P1 | Z-only stop confirmation cannot distinguish cruise from rest | A mid-run jolt followed by steady cruise can look like an arrival | Require Z candidate plus independent X/Y settled confirmation; cancel on renewed motion |
| P1 | Existing X/Y release is sample-count based and independent | Enabling it can release after only a small number of 25 Hz polls | Convert it to a wall-clock settle timer, require freshness, and make it conjunctive with Z |
| P1 | Battery and movement alarms write the piezo independently | Battery logic can force the piezo low during a movement alarm | Centralize output arbitration; movement alarm has priority |
| P2 | Any-motion “rearm” does not explicitly disable the feature | The device may not refresh its internal reference after a stuck line | Disable, configure, enable, and verify the feature/status explicitly |
| P2 | ISR and I2C hardening are incomplete | A stuck bus or unsafe shared flag can wedge sampling | Add I2C timeout/watchdog recovery and make ISR-shared state explicitly safe |
| P2 | Documentation and sample-count constants lag 25 Hz | Old 3.13 Hz assumptions can silently change timing | Express behavior in milliseconds and update comments/tests |

---

## 3. P0 fixes — required before expanding field use

### 3.1 Never release the movement alarm on the 300 s timeout

**Observed evidence:** An eight-floor run at 18 fpm expired on the existing failsafe timeout.

**Current path:** In `falcon_srcs/src/movement_service.cpp`, `STATE_MOVING` handles `LATCH_FAILSAFE_MS` by starting the stop timer and entering `STATE_DECELERATING`. That state can then accept baseline Z as “settled” for `STOP_CONFIRM_MS` and disable the alarm. At constant velocity, Z is near its baseline, so the timeout can produce a false arrival during travel.

**Suggested change:**

- Add `STATE_FAULT_LATCHED`.
- On `LATCH_FAILSAFE_MS`, enter the fault state and keep the movement alarm/chase active.
- Require an explicit service/manual reset or power cycle according to the product safety policy.
- Log a distinct fault reason such as `FAULT_MOVEMENT_TIMEOUT`.
- Never route a timeout through `STATE_DECELERATING` or `STATE_STOPPED`.

Suggested behavior:

```text
MOVING + timeout
    -> FAULT_LATCHED
    -> movement alarm remains active
    -> no automatic READY transition
```

This is the most important correction because the current behavior fails in the catastrophic direction: alarm release while the machine may still be moving.

### 3.2 Make sensor health part of the state machine

**Current path:**

- `falcon_srcs/src/main.cpp` increments `sensor_err_run` after read failures but continues using the last published average.
- `falcon_srcs/src/arduino_bma456.cpp` exposes `initialize()` as `void` and does not propagate failures from reset, configuration upload, accelerometer configuration, or feature enable.
- Stop confirmation is based on elapsed time and quiet-looking values, not a quorum of fresh successful samples.

A disconnected, wedged, or misconfigured sensor must not produce READY or STOPPED.

**Suggested change:**

1. Change sensor initialization/configuration functions to return a status.
2. Require a complete boot-ready quorum before entering calibration or monitoring:
   - chip communication succeeds;
   - configuration upload succeeds;
   - accelerometer settings read back as expected;
   - any-motion enable and interrupt routing succeed;
   - a minimum number of plausible, fresh samples arrive.
3. Add monotonic sample sequence/time metadata to the published snapshot.
4. In all settle/arrival confirmation logic, count only fresh successful samples.
5. On a sustained read failure, impossible repeated values, stale snapshot age, or configuration error, enter `STATE_FAULT_LATCHED`.
6. Keep the alarm active if the fault occurs while movement is active or arrival is unconfirmed.
7. Provide a distinct diagnostic reason for boot, read, stale-data, and feature-configuration faults.

A stale average must never satisfy a stop timer.

---

## 4. P1 fixes — jog handling and smooth-stop arrival

### 4.1 Remove `MIN_TRAVEL_MS` from the armed raw-peak path

The new ISR raw peak is already protected from the departure bounce: collection begins only after `ARRIVAL_ARM_SAMPLES` quiet samples. However, both raw-peak arrival checks remain inside the `MIN_TRAVEL_MS` gate in `movement_service.cpp`.

For a 1–2 s jog:

1. departure occurs;
2. the raw detector arms after quiet;
3. the stop produces a valid peak;
4. the 1–2 s peak window expires;
5. only afterward does the 3 s travel gate open.

The correct stop evidence is therefore discarded. A later jog can supply another peak and appear to “reset” the first alarm.

**Suggested change:**

- Evaluate an armed raw-Z peak in `STATE_MOVING` regardless of `MIN_TRAVEL_MS`.
- Keep `MIN_TRAVEL_MS` only around legacy arrival signals that still need departure blanking.
- Prefer exposing an explicit `arrival_peak_is_armed()` or equivalent state rather than inferring validity from peak magnitude.
- Clear the peak/candidate deterministically on state entry and after consumption.

### 4.2 Replace release with an arrival-candidate state

A raw-Z peak proves that a sharp vertical event occurred. It does not, by itself, prove that the machine is stopped. A rail joint or other cruise shock could exceed `ARRIVAL_PEAK_VALUE`; after the shock, steady cruise returns Z to baseline and the current five-second Z-only confirmation can release the alarm.

Introduce `STATE_ARRIVAL_CANDIDATE`:

- Enter on a qualified raw-Z peak, an accepted any-motion cluster, or another approved arrival cue.
- Keep the movement alarm active.
- Start independent freshness and settle tracking.
- Confirm STOPPED only when Z is stable **and** X/Y meet a time-based stopped condition.
- If X/Y becomes motion-like again, Z leaves its settle band, or a new any-motion departure pattern occurs, cancel the candidate and return to MOVING.
- If sensor health is lost or the candidate exceeds its maximum duration, enter `STATE_FAULT_LATCHED`, not STOPPED.

### 4.3 Use X/Y only as conjunctive stopped confirmation

The field note asks whether stable X/Y can release after Z shows a deviation. That is the right direction, with one constraint: X/Y must corroborate a Z arrival candidate and must not independently release the alarm.

The current dormant path should be redesigned before enabling it:

- `XY_RELEASE_ARMED` is correctly 0 today.
- The present X/Y path can transition directly to STOPPED without a Z candidate.
- `XY_RELEASE_POLLS = 2` was tuned for the previous 3.13 Hz host loop. At 25 Hz it represents only a small fraction of a second.
- In `lateral.cpp`, an invalid or excessive time gap returns without necessarily discarding accumulated quiet evidence. Measurements separated by an unknown gap can therefore combine.

**Suggested X/Y contract:**

- Use elapsed wall time, such as `XY_SETTLE_MS`, not a poll count.
- Initial field target: 5 s settled, matching Dave's proposed observation interval. Retune from measured logs.
- Require a maximum gap between valid observations.
- Reset the settle timer on invalid/stale samples, excessive gaps, or motion-like X/Y.
- Require the detector to be in `STATE_ARRIVAL_CANDIDATE`.
- Require Z to remain inside its fresh-sample settle band.
- Do not release on X/Y alone.
- Cancel the candidate immediately when renewed movement is observed.

The 350 fpm smooth-stop test is the key acceptance case for this change.

### 4.4 Remove the fixed six-second departure blind window

`MONITOR_REARM_MS = 6000` suppresses departure detection after an arrival and on the first monitoring interval after calibration. Closely spaced inspection jogs can begin inside this window, especially when low-speed detection depends on the sensor's any-motion feature.

**Suggested change:**

- Separate initial boot stabilization from post-arrival ringdown.
- End post-arrival blanking when measured Z and X/Y ringdown has settled for a minimum time, subject to a short maximum cap.
- Continue tracking sensor health while blanked.
- Where possible, record rather than discard motion evidence during blanking and use it to return to MOVING.
- Test restart gaps of 0.5, 1, 2, and 5 seconds.

---

## 5. Output and feature-control corrections

### 5.1 Centralize piezo arbitration

Movement and battery alarm code currently write `PIN_PIEZO` independently. Because the loop checks movement first and battery second, the battery-off phase can force the physical output low even while the movement alarm requests high. Internal `buzzer_on` state can still indicate that the buzzer is on, which also makes sensing blanking inconsistent with the actual output.

**Suggested change:**

Compute requests first, then drive the hardware once:

```text
movement_request = movement alarm waveform
battery_request  = battery warning waveform
piezo_output     = movement_request OR battery_request
```

Movement alarm must have priority. Sampling blanking/ringdown should follow the actual physical output transition, not an earlier request flag.

### 5.2 Perform a real any-motion rearm

The stuck-line recovery comment says the any-motion feature is disabled and re-enabled, but `configureAnyMotion()` enables the feature again without an explicit disable. Rewriting an already-enabled bit does not guarantee that the BMA456 refreshes its reference.

**Suggested sequence:**

1. disable the any-motion feature;
2. clear/verify interrupt status as specified by the Bosch API;
3. write configuration and axes;
4. enable the feature;
5. verify status and interrupt pin behavior;
6. fault-latch if recovery does not succeed.

### 5.3 Harden ISR and I2C failure behavior

- Mark ISR-shared guard/state variables with the correct volatile/atomic semantics.
- Avoid relying on a manually re-enabled nested-interrupt path without an explicit reentrancy analysis.
- Add a bounded I2C transaction timeout.
- Recover or watchdog-reset if the TWI peripheral wedges.
- Count and publish ISR overruns, read failures, stale snapshots, and maximum read duration.
- Ensure fault behavior is deterministic even if no new sample can be published.

---

## 6. Proposed state behavior

| State | Alarm | Exit conditions |
| --- | --- | --- |
| CALIBRATING | Off unless fault policy requires otherwise | Fresh, plausible calibration quorum -> MONITORING; failure -> FAULT_LATCHED |
| MONITORING | Off | Qualified departure -> MOVEMENT_DETECTED; sensor fault -> FAULT_LATCHED |
| MOVEMENT_DETECTED | On | Confirmed movement -> MOVING; rejected transient -> MONITORING; fault -> FAULT_LATCHED |
| MOVING | On | Qualified arrival cue -> ARRIVAL_CANDIDATE; timeout/fault -> FAULT_LATCHED |
| ARRIVAL_CANDIDATE | On | Fresh Z stable **and** X/Y settled -> STOPPED; renewed motion -> MOVING; timeout/fault -> FAULT_LATCHED |
| STOPPED | Off | After controlled ringdown/settle -> MONITORING; new motion -> MOVING |
| FAULT_LATCHED | On | Deliberate service/manual reset only |

No elapsed-time path from MOVING or ARRIVAL_CANDIDATE may automatically disable the movement alarm.

---

## 7. Implementation order

### Patch 1 — fail-safe foundation

- Add `STATE_FAULT_LATCHED` and fault reasons.
- Change the 300 s timeout to latch the fault/alarm.
- Propagate BMA456 initialization/configuration errors.
- Add fresh-sample age/sequence checks.
- Prevent stale samples from satisfying stop confirmation.

### Patch 2 — short-jog correctness

- Move the armed raw-peak test outside `MIN_TRAVEL_MS`.
- Expose explicit raw-peak armed/valid state.
- Replace the fixed monitor rearm with measured ringdown behavior.
- Add short-jog regression logs/tests.

### Patch 3 — smooth-stop confirmation

- Introduce `STATE_ARRIVAL_CANDIDATE`.
- Convert X/Y quiet evidence to a wall-clock settle detector.
- Reset X/Y settle on stale or excessive-gap samples.
- Require Z candidate + fresh stable Z + settled X/Y.
- Cancel the candidate on renewed motion.

### Patch 4 — hardening and cleanup

- Centralize piezo arbitration.
- Correct any-motion rearm.
- Add I2C timeout/watchdog handling.
- Retune or replace all old-rate sample-count constants.
- Update comments and engineering documentation.

Do not combine the 1 MHz fuse/100 Hz change with these state-machine fixes. Validate the 25 Hz behavior first so timing and logic changes remain separable.

---

## 8. Acceptance test matrix

Capture timestamped Z raw/average/peak, X/Y metric and settle time, any-motion status, sensor freshness/error counters, state, fault reason, and physical piezo output.

| Test | Required result |
| --- | --- |
| 0.25, 0.5, 1, and 2 s inspection jogs, both directions | Alarm activates; valid stop is not lost behind `MIN_TRAVEL_MS`; returns only after confirmed rest |
| Repeated jogs with 0.5, 1, 2, and 5 s gaps | No departure is missed by a fixed rearm window; no extra jog is required to reset |
| Run longer than 300 s, including 18 fpm | Alarm never clears mid-travel; timeout enters FAULT_LATCHED |
| Sensor disconnect/NACK at boot | Device never reports normal READY; fault is explicit |
| Sensor disconnect/NACK in MONITORING, MOVING, and ARRIVAL_CANDIDATE | No stale-data stop; correct fault-latched behavior |
| Frozen/repeated sensor values | Freshness/plausibility check faults; no false STOPPED |
| Cruise shock or rail joint above 0.70 m/s² | May create a candidate but cannot release while X/Y remains motion-like |
| 350 fpm normal operation, both directions, smooth slowdown | Alarm activates and releases only after Z candidate/approved cue plus X/Y confirmed rest |
| Low-speed cartop and counterweight inspection | Preserve the demonstrated 25 Hz brake pick/set performance |
| Movement alarm overlapping low-battery warning | Movement tone remains physically audible; sensing blanking follows the actual pin |
| Rougher second machine | Re-measure cruise peak ceiling and X/Y bands before changing thresholds |
| Power cycle and deliberate fault reset | State and alarm behavior match the defined recovery policy |

Release gates:

- zero mid-travel alarm releases;
- zero silent READY states with an unhealthy sensor;
- zero stop confirmations based on stale data;
- short jogs do not require a second jog;
- smooth normal-operation stop passes in both directions;
- logs demonstrate X/Y settle freshness and candidate cancellation;
- build, flash, and field-test evidence is attached to the change review.

Because this device is used around moving elevator equipment, firmware changes should receive an independent safety review and supervised field validation before deployment.

---

## 9. Preserve these parts of the 25 Hz change

The following current decisions are sound and should not be lost while fixing the state machine:

- Timer1 CTC setup order: clear control registers, load `OCR1A = 624`, select CTC, then start with prescaler 64.
- 25 Hz sampling with a 32-sample average preserves the 1.28 s averaging window.
- The ISR-owned, two-bucket raw peak survives host snapshot overruns and forgets old cruise peaks.
- `ARRIVAL_PEAK_VALUE = 0.70` has strong provisional separation on the tested machine.
- `XY_RELEASE_ARMED = 0` and `VEL_ARMED = 0` should remain unchanged until their replacement logic is validated.
- Improved bus-error propagation and zero/zero plausibility checks should remain.

The accurate current claim is: **inspection brake-set arrival is strongly detected at 25 Hz on the tested machine.** Smooth normal-operation arrival remains open.

---

## 10. Repository security blocker

A repository file named `Agent Token` appears to contain a GitHub credential. Do not copy the value into logs, issues, commits, or this note.

Before treating the branch as releasable:

1. remove the credential file;
2. revoke/rotate the exposed token;
3. purge the secret from Git history if it was committed;
4. use the GitHub App or secret injection instead of a repository file;
5. add an appropriate ignore rule and secret-scanning check.

Credential cycling after a session does not make a committed token safe while it remains active or recoverable from history.
