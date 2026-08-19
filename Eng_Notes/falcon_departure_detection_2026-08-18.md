# Departure detection: root cause of missed departures, and the limits of threshold-based discrimination

**Date:** 2026-08-18
**Rigs:** cartop, device inverted (counterweight-underside simulation), then
right-side up. Separately, a 150 fpm hydraulic elevator in a second building.
**Builds:** shipping 31464 at start; `025befd` (31714) and `687b969` (32188)
flashed and tested in sequence.
**Data:** `falcon_srcs/datasets/260818-105317.log`, `260818-133242.log`.

---

## 1. Summary

Missed departures were traced to a firmware blanking window, not to sensor
sensitivity. Correcting it eliminated the failure across 24 subsequent runs in
two mountings.

Four separate attempts to improve departure/disturbance discrimination by
threshold or waveform shape were tested against the burst corpus and against
labelled events. All four failed. The measurements establish that knocks and
slow departures are not separable by the features currently available, which
makes sensitivity a design trade rather than a tuning problem.

| Result | Status |
|---|---|
| Missed-departure root cause and fix | Established, verified 0/24 |
| Arrival gate has no margin at low speed | Measured, unfixed |
| Jog verdict cannot separate its populations | Measured, unfixed |
| Threshold and shape-based discrimination | Four approaches refuted |
| Second installation (hydro) | First data, observational only |

---

## 2. Missed departures: root cause and correction

### 2.1 Failure mode

At 17–35 fpm the beacon did not sound during travel. The latch instead occurred
at the brake, after which no arrival transient remained to release it, leaving
the beacon asserting over a stationary car until `LATCH_FAILSAFE_MS`.

Measured rate before correction: **6 missed departures in 12 slow up runs.**

### 2.2 Cause

`MovementService::notify_any_motion()` discarded any any-motion edge arriving
within `MONITOR_REARM_MS` (6000 ms) of entering `STATE_MONITORING`. The window
was added 2026-08-07 to prevent one trip alarming twice.

Cruise generates effectively no any-motion edges. The departure edge is
therefore the only edge between one stop and the next, and discarding it makes
the entire run invisible to the FSM.

Evidence: 32 `any-motion ignored, re-arm blanking` lines in the session; 4 of
the 6 missed runs carry one for their departure edge.

### 2.3 Correction

The blank now ends when the car is measured to have settled rather than after a
fixed interval: `lat_monitor.quiet_run() >= MONITOR_REARM_QUIET` (8), floored by
`MONITOR_REARM_MIN_MS` (1500) and still capped by the original
`MONITOR_REARM_MS`. The window can only shorten, never lengthen.

### 2.4 Verification

| Condition | Runs | Missed |
|---|---|---|
| Before correction, inverted | 12 | 6 |
| After correction, inverted | 14 | 0 |
| After correction, right-side up | 10 | 0 |

The 14 post-correction departures in the inverted mounting latched 2883–4325 ms
after entering MONITORING — every one inside the former 6000 ms blank, and
therefore every one would have been discarded by the previous build. Measured
blank duration after correction: 1507–1573 ms across 14 calibrations.

### 2.5 Residual risk

The 6000 ms window existed because a terminal-floor brake set re-latched 3.5 s
after MONITORING was entered and produced a 73-second alarm on a stationary car
(2026-08-07). `STOP_CONFIRM_MS` is the primary defence against that; this window
was the backstop. Shortening it re-opens that path.

Not observed in 24 runs. The signature to watch is an alarm that *begins* on a
car that has already stopped. Reversion is `MONITOR_REARM_QUIET 0`.

---

## 3. Discrimination limits

Four approaches to separating real departures from disturbances were tested.
All four are refuted. They are recorded here so they are not re-derived.

### 3.1 Amplitude cannot separate knocks from slow departures

A deliberate hand bump on the mounting produced a peak of **0.136 m/s²**. A real
18 fpm departure measures **0.116 m/s²**. The disturbance is larger than the
signal.

`ANYMOTION_THRESHOLD` is 0.153 m/s². Raising it to reject handling rejects real
low-speed departures; lowering it to gain departure margin admits every knock.
No value satisfies both.

Note that the logged peak reads *below* the configured threshold because the log
samples at 25 Hz with approximately 20% loss while the sensor engine runs at
100 Hz. A sub-threshold logged peak is not evidence that the sensor should not
have fired.

### 3.2 Departure waveform shape cannot identify a jog

Tested by applying the ramp detector's block statistics (`RAMP_BLOCK_N` 12,
`RAMP_DIR_PCT` 85) to all 177 departure bursts on file, via
`falcon_srcs/graph/jog_window_replay.py`.

Real runs produced best-block means of 148–475 at 100% directionality. Jogs
produced 83–1017, many also at 100%. Complete overlap on both axes; the floor
sweep gives 33–50 disagreements with the shipping gate at every floor from 80 to
300.

The reason is structural: a jog's departure *is* a real departure. The car
accelerates identically. What distinguishes a jog is the reversal that follows,
which is what `opk` already measures. `opk` is therefore on the correct axis.

### 3.3 Waveform shape cannot reject knocks either

Tested as a pre-filter — "was this a departure at all" — which is a different
question from 3.2 and initially appeared to work.

| Event | Classification | Block mean | Directionality |
|---|---|---|---|
| Walking on cartop | Not a departure | 14 | 58% |
| Walking on cartop | Not a departure | 34 | 50% |
| Hand bump | Not a departure | 33 | 75% |
| 20 fpm down | Departure | 199 | 92% |
| 20 fpm down | Departure | 160 | 92% |
| **20 fpm down** | **Departure** | **45** | **75%** |

The third departure lands inside the knock band at the same directionality as
the bump. Any floor high enough to reject the disturbances also rejects that
real departure, which is the catastrophic direction. The shipping `opk` gate
classified it correctly, so the pre-filter would have been strictly worse than
the current build on that event.

The apparent 4.4x separation seen before the low-speed departures were added was
an artefact of sampling only strong departures.

### 3.4 The jog verdict misses gentle jogs

A deliberate jog produced `ratio=88 opk=381`, below the 900 gate, and was
classified RUN. It cleared the existing latch and immediately re-latched.
Successful jog-clears in the same session ran `opk` 1459–2974.

This is the same overlap seen in 3.2 acting in the opposite direction: one
threshold is being asked to separate populations that intersect. Operationally
it means clearing a false latch by jogging is not reliable, and several clears
in the session actually occurred through the arrival path rather than the jog
verdict.

### 3.5 Conclusion

With amplitude, departure shape, and reversal as the available features, knocks
and slow departures are not separable. Detector sensitivity sufficient for the
20 fpm requirement necessarily admits handling transients. This is a trade to be
positioned deliberately, not a threshold to be tuned.

---

## 4. Release-path margins

### 4.1 Arrival gate

Arrival peaks measured against the 0.45 gate, slow up runs from the lower
terminal:

```
0.460  0.461  0.462  0.478  0.480  0.495  0.499  0.506  0.517  0.560
```

Four sit at 1.02–1.03x. Three fall within 0.002 of each other. The distribution
mode for this direction and position sits on the threshold rather than near it,
which means a marginally softer stop fails to release and the beacon runs to the
failsafe.

Previous worst margin on file was 1.009x, treated as an outlier. It is not an
outlier at low speed.

Measured in both mountings, so this is a property of direction and position, not
of the inverted rig.

### 4.2 Arming margin

Reversal-arming counts within a single mounting: `ro=` 3, 4, 12, 14, 15, 28.
Wider than the 6–15 previously recorded, on one installation.

### 4.3 Quiet gate versus measured cruise

`ARRIVAL_QUIET_MSS` is 0.15 m/s², justified in comment as "above cruise raw max
0.0875". The arrival-gate derivation in `movement_service.h` records later cruise
ceilings of **0.23, 0.23 and 0.28** on faster runs.

If cruise exceeds the quiet gate, the quiet run resets continuously and the
quiet arming path cannot arm on those machines. This is consistent with the
arming-margin spread in 4.2 and has not been investigated.

---

## 5. Self-calibrated polled departure threshold

The polled departure excursion is mounting-dependent: inverted, departures
measured median 0.136 and maximum 0.398; right-side up, 0.050–0.120 with median
0.067. Rest noise is stable across both (p99 0.0143 and 0.0151).

A fixed `DEFAULT_THRESHOLD_VALUE` is therefore wrong in every mounting but the
one it was measured in. Measured reachability: 0.40 reached 1 of 82 departures;
0.20 reached 0 of 8; 0.10 reached 1 of 8.

`threshold_value` is now derived at calibration as the z-average spread across
the window multiplied by `Z_THRESH_MARGIN`, clamped to
[`Z_THRESH_MIN`, `Z_THRESH_MAX`]. This mirrors the existing `XY_STILL`
mechanism. The deployment sequence — placement before power-on — is what makes
calibration-time measurement valid.

Two defects were found during commissioning, both identified because the clamp
printed an exact bound:

1. The FSM enters calibration before the first accelerometer sample is folded
   in, so the average reads approximately zero on the opening pass. Spread
   computed as 9.76 and clamped to `Z_THRESH_MAX`.
2. The 32-sample average converges across its 1.28 s window (measured
   7.499, 8.522, 9.758). Spread computed as 2.26 and clamped again.

`Z_CAL_SETTLE_MS` (1500) now excludes the convergence period, leaving 4.5 s of
the 6 s window for measurement. Without the ceiling clamp, either defect would
have silently disabled the polled detector for the whole deployment.

**Status: the mechanism operates but the learning is unproven.** Both mountings
tested are quiet enough that the derived value clamps to `Z_THRESH_MIN` (0.040).
Demonstrating that per-deployment learning does anything requires a mounting
whose noise floor produces a value above the floor.

---

## 6. Second installation

A 150 fpm hydraulic elevator in a different building was run up and down with
the beacon behaving correctly in both directions. This is the first machine
other than the original traction car tested on this project, and a hydraulic
drive is a different class: descent is valve-controlled under gravity rather
than motor-driven under field control, so its transients are unlike any burst in
the corpus.

Limits:

1. **Unmeasured.** No capture was running. No departure margin, arrival peak or
   verdict data exists for it.
2. **150 fpm is the permissive regime.** Every failure documented above occurs
   at 17–35 fpm, where transients approach the detection floor and arrival peaks
   approach the release gate. At 150 fpm both are far above threshold.
3. Installation coverage is opened, not closed. The arming-margin and threshold
   transfer questions remain untested at that site.

---

## 7. Recommended next measurements

1. Repeat the hydro at inspection speed with logging. Capture `XY-Still` and
   `Threshold-Value` at calibration, then `ARM q=`/`ro=` and arrival peaks per
   run. This is the first opportunity to test whether learned thresholds
   transfer between installations, and the first mounting that may produce a
   learned z threshold above the clamp floor.
2. Establish whether the re-arm correction has re-opened the 2026-08-07
   stationary-car alarm. Requires terminal-floor stops with the logger running.
3. Investigate `ARRIVAL_QUIET_MSS` against measured cruise (4.3).
4. Label the burst corpus. None of the four candidate rules in §3 could be
   scored against 177 unlabelled bursts; the six labelled events that exist were
   produced deliberately, and were what refuted the strongest candidate.
