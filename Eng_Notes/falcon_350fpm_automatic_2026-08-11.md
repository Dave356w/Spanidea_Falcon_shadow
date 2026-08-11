# 350 fpm automatic operation, inside cab — 2026-08-11 afternoon

Four full express runs on **automatic operation at 350 fpm**, measured from
inside the cab, Dave driving. Firmware `62d77d9` for runs 1–3, then
`BURST_POST_ARR` 20 → 60 reflashed (31124 bytes) for runs 4–5. Logs:
`device-monitor-260811-134652.log` (runs 1–3, boot t≈30000) and
`device-monitor-260811-140425.log` (runs 4–5, fresh boot).

## 1. Headline: every stop released — but on a 1% margin

The failure recorded in the state-of-project note ("normal operation has no
brake transient, the unit alarmed and never released") **did not reproduce**.
All four measured arrivals released the alarm within seconds of the stop:

| run | dir | arrival path | FSM peak | margin vs 0.45 |
|---|---|---|---|---|
| 1 | up   | any-motion | 0.527 | 1.17× |
| 2 | down | polled     | **0.454** | **1.009×** |
| 4 | down | polled     | 0.465 | 1.03× |
| 5 | up   | polled     | 0.508 | 1.13× |

Why it now works at all: `ARRIVAL_PEAK_VALUE` went 0.70 → 0.45 in `0a93d1c`.
Every peak above lands **between** the old and new gates, so the earlier
"never released" observation and today's four releases are consistent — the
threshold moved, not the physics.

**Why this is marginal, not solved:** run-to-run spread (0.454–0.527, i.e.
0.073) is ~18× the worst margin (0.004). Three of four peaks sit within 13%
of the gate. The FSM peak is a deviation from a rolling average, and a
sustained ramp drags the average with it, suppressing the measured deviation
— raw burst peaks were 0.63–0.66 while the FSM saw 0.45–0.53. The metric gets
structurally weaker exactly as ramps get longer (faster cars, taller runs).
This will pass a demo and fail in service. **Problem #1 is downgraded from
"no release" to "releases on luck".**

## 2. The drive holds 0.60 m/s² regardless of speed or direction

Nine ramps measured across the afternoon (five departures, four arrivals,
both directions). Plateau = mean of best 15-sample window; dir = |Σa|/Σ|a|.

| event | plateau (m/s²) | dir |
|---|---|---|
| run 1 dep (up) | +0.605 | 0.995 |
| run 1 arr | −0.615 | 0.957 |
| run 2 dep (down) | −0.603 | 0.982 |
| run 2 arr | +0.605 | 0.902 |
| run 3 dep (up, uncommanded car call) | +0.605 | 0.996 |
| run 4 dep (down) | −0.598 | 0.975 |
| run 4 arr | +0.603 | 0.888 |
| run 5 dep (up) | +0.609 | 0.997 |
| run 5 arr | −0.613 | 0.947 |

**Plateau 0.605 ± 0.008 on every single ramp.** Signs invert correctly with
direction every time. Directionality 0.89–1.00 throughout — squarely the
drive-ramp population from `falcon_signature_2026-08-11.md` §3 (1.000 for the
115 fpm ramp), nothing near the brake-stop population (0.42 / 0.02).

Combined with the 115 fpm plateau of 0.52, the question that gated the ramp
detector is answered: **the drive's deceleration magnitude is essentially
constant (~0.5–0.6 m/s²); speed buys duration, not amplitude.** A ramp
detector can use a **fixed magnitude floor**. The discriminator that scales
with speed is *duration* of one-signed acceleration: departures at 350 fpm
sustain 2.5–2.6 s vs ~0.9 s expected at 115 fpm.

## 3. The arrival window still truncates the stop — even at 60 post

The `BURST_POST_ARR` 20 → 60 change helped less than expected. In runs 4–5
the sustained arrival ramp begins **25–33 samples (1.0–1.3 s) after the
trigger** (trigger at idx ~19, ramp onset idx 45–52) and is still at full
plateau when the window closes:

| | integral captured | of actual 1.778 m/s | end value |
|---|---|---|---|
| run 4 arr | 0.560 m/s | 31% | +0.620 |
| run 5 arr | 0.744 m/s | 42% | −0.601 |

The polled peak crosses 0.45 on the ramp's leading edge, well before the
sustained plateau develops, so even 2.4 s of post-roll runs out mid-ramp. A
350 fpm stop needs ~2.9 s of ramp; **the full deceleration to standstill has
still never been captured end-to-end.** Options for next time: raise
`BURST_N` (80 → 120 costs 80 bytes of the 568 free), or trigger the arrival
burst later. Until then, duration thresholds must be set from *departure*
bursts (complete: integrals 1.32–1.44 of 1.78 m/s, sustained 2.5–2.6 s).

## 4. Design conclusion

Measure the plateau, not the peak. A detector keying on **sustained
one-signed acceleration ≥ ~0.4 m/s² with directionality > 0.9** would have
had margin of ~1.5× on amplitude and 5σ-class separation on character, five
for five, where the peak detector had 1.009×. The ramp detector
(`falcon_signature_2026-08-11.md` §4) is promoted from robustness improvement
to **the fix for a threshold currently passing on luck**. Its two
prerequisites are now met: fixed floor justified (§2), and its trigger
question is understood (§3 — the existing arrival trigger fires early enough
on fast stops; the slow-stop case still needs its own lower trigger).

Caveat: cab mounting flatters the detector — cruise pk was 0.02–0.03 inside
the cab vs 0.07–0.28 on the cartop. The 0.45 gate's false-positive headroom
here says nothing about cartop mounting.

## 5. Anomalies, in the order they'll bite

1. **No valid battery reading the entire session.** Every `Voltage value`
   print was "(settling, ignored)", including one at t≈450000 — seven-plus
   minutes after boot. A settling discard that late looks like a bug in the
   settling logic, not settling. Bench item; and it means the brownout watch
   ran blind all afternoon.
2. **Monitor handle drop again** (log froze at 1845 bytes while the process
   lived), and the device then needed an avrdude signature read to revive.
   The reset is now a routine part of the flash-capture loop, not an anomaly.
3. **Run 3 was an uncommanded car call** — a clean up departure (plateau
   +0.605, dir 0.996) captured after Dave reported stopped. Data kept; noted
   so the run count doesn't confuse anyone re-reading the log.
4. One `ACC-STAT read FAILED` (single I2C hiccup, run 1 capture, no action).

## 6. What was NOT verified today

- Whether 1/5 beep duty extends alarm endurance (needs timed bench alarm
  with a meter — still the top open item; brownout at 242.7 s stands).
- Down-run arrival distribution: two samples (0.454, 0.465) hug the gate.
  Do not treat the release as dependable at 350 fpm down.
- Anything about cartop mounting at speed — all of today is cab-interior.
