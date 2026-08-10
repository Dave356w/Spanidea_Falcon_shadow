# 25 Hz, and the arrival problem solved

**Date:** 2026-08-10, afternoon, cartop, live building
**Status:** measured across three runs at two speeds. Provisional but strong.
**Supersedes:** the conclusion in `falcon_hoistway_protocol.md` §5 that the
device cannot distinguish a stopped machine from a slowly moving one. That was
the right reading of the morning's data and the wrong conclusion about the
cause. Section refs §n point at `falcon_analysis_2026-08-06.md`.

---

## 1. The morning was wrong about the wall

By midday the session had concluded that an accelerometer at 3.13 Hz cannot
tell "stopped" from "moving steadily", having hit it from four directions:

- lateral level — cruise and parked are the same distribution
- any-motion edge rate — zero edges/min at 25 fpm cruise
- vertical average — 1 g whether cruising or parked
- velocity integral — a real 25 fpm run produced a SMALLER signal than a jog

All four are true. The conclusion drawn from them — that this is physics and
needs a different sensor — was not.

**What broke it open was Dave's observation from the cartop:** *"the cartop
operation exhibits some rollback upon brake energization ... abrupt up and down
bounce on pick and set."*

The brake setting produces a sharp transient of roughly 80 ms. Two firmware
properties were destroying it:

1. **3.13 Hz sampling.** One sample every 320 ms catches an 80 ms event about a
   quarter of the time. A detector that sees a quarter of its events is not a
   detector, and no threshold repairs that.
2. **The 1.28 s rolling average.** Whatever survived was then divided by 32.

Neither is physics. Both are arithmetic.

---

## 2. What changed

### 2.1 Timer: 3.13 Hz → 25 Hz

The old configuration was phase-and-frequency-correct PWM (mode 9, TOP=OCR1A),
prescaler 1024, OCR1A=156. That mode counts up **and down**, doubling the
period: `2 * 156 * 1024 / 1e6 = 319.5 ms`, exactly the rate in every capture
ever taken on this device. Nothing wanted a PWM waveform — the timer only
raises an interrupt — so CTC (mode 4), prescaler 64, OCR1A=624 gives
`625 * 64 / 1e6 = 40.000 ms`.

**Why 25 and not 100.** The I2C read measures 7.5 ms (`rd=` in every line) and
is a hard floor on the period. 25 Hz spends 19% of the budget there; 100 Hz
would spend 75%. Getting the sensor's full 100 Hz output rate means coming off
the 1 MHz fuse first.

**Register order is load-bearing**, and getting it wrong cost a flash cycle.
`OCR1A` is double-buffered in every PWM mode, and Arduino's `init()` leaves
`WGM10` set — so an `OCR1A` write before `TCCR1A` is cleared lands in the
shadow register and never arrives. Clear both control registers first (normal
mode, direct write), then load TOP, then select CTC, then start the clock.

`RollingAvg(4)` → `(32)` in the same step, **deliberately holding the window at
1.28 s** so every threshold reading that average keeps the meaning it was tuned
to. Parked noise on the average fell from 0.0259 to 0.0155 max deviation.

### 2.2 Arrival reads the RAW sample, not the average

This is what made the rate change pay, and it is the opposite of the direction
the project had been pushing for a week.

The first 25 Hz run made it unmissable: a stop whose **raw** excursion was
0.938 m/s² registered **0.0072** on the average, against an
`ARRIVAL_CLUSTER_DELTA` of 0.20. One large sample inside a 32-sample window is
divided by 32. The averaging destroys precisely the signal the rate fix just
made visible.

So a peak of `|a − zero|` is tracked in the sampling ISR and both arrival tests
read it. Kept in the ISR rather than sampled from the published snapshot,
because ~20% of snapshots are lost to overrun and the peak must not be one of
them.

**Armed on quiet, not on a timer.** The departure has a bounce of its own, so
the peak is not collected until the signal has been below `ARRIVAL_QUIET_MSS`
for `ARRIVAL_ARM_SAMPLES`. The detector waits for the departure transient to
**end** rather than for a fixed interval to elapse — the property
`MIN_TRAVEL_MS` was approximating badly, and the reason a 1 s jog's stop was
discarded unseen that morning.

**Surgical on purpose.** The any-motion cluster logic is untouched and still
runs on the sensor's internal 100 Hz stream, so host blanking cannot hide a
stop from it. Only the corroborating signal changed.

### 2.3 🔴 The peak must be windowed, not cumulative

The first implementation held a running max for the whole run, and the
high-speed run exposed it within minutes: cruise ratcheted
`0.09 → 0.13 → 0.17 → 0.19 → 0.23` in **fifteen seconds**, against a 0.30
threshold. The morning's 25 fpm run lasted **197 seconds**. A cumulative max
crosses any threshold eventually and releases the beacon mid-travel — the
catastrophic direction, reached not through a bad threshold but through a
detector that cannot forget.

Two alternating one-second buckets, reporting `max(current, previous)`, give a
1–2 s sliding window in two floats and no array.

---

## 3. Measured

| run | cruise ceiling | arrival | margin | reset |
|---|---|---|---|---|
| 18 fpm down | 0.09 | **2.03** | 22.6× | 5.3 s |
| high speed up | 0.23 | **3.42** | 14.9× | 8.5 s |
| 120 fpm down, 4 floors | 0.23 | **4.58** | 19.9× | 8.2 s |

`ARRIVAL_PEAK_VALUE` = **0.70**: 3.0× above the worst cruise observed, 2.9×
below the smallest arrival.

**No arrival discriminator on this device has previously had real margin in
even one direction.** The best prior figure was 1.4×; several were 1.07×.

**The 18 fpm row is the one that matters.** §14.7 measured a gentle 18 fpm
arrival at 0.058 against a parked noise floor of 0.103 — the arrival was
*smaller than the noise*. That measurement is what killed amplitude
thresholding and started the whole Z + X/Y detour. The same class of event now
measures 2.03 against a cruise ceiling of 0.09.

Windowing confirmed on the four-floor run: through 25 s of cruise the value
oscillates 0.07–0.23 and repeatedly falls back, then decays
`0.89 → 0.47 → 0.10 → 0.05` after the arrival. Visible at rest too — parked
`pk` had been stuck at 0.21 and now fluctuates at 0.01–0.02.

---

## 4. What this changes elsewhere

- **The x/y release path is no longer needed for arrivals.** It was carrying
  one justification after the reset budget was withdrawn (protocol §5.6) —
  catching stops z cannot see. Z can now see them, with 20× margin.
  `XY_RELEASE_ARMED` stays 0 and the module stays as instrumentation.
- **Roadmap item 5 is done** at 25 Hz, though not at the 100 Hz the sensor can
  produce. That still wants the fuse change.
- **`velocity.h` is demoted further.** The conservation test was the candidate
  fix for gentle arrivals; the peak detector solves that case directly and far
  more cheaply.

---

## 5. Not proven, and the honest limits

- **Three runs, one machine, one day.** The cruise ceiling is the figure that
  moves with equipment — a rougher machine narrows the 3.0× directly, and it
  is the number to re-measure elsewhere first.
- **~20% of samples are lost to snapshot overrun.** `Serial.print` blocks
  ~85 ms per line while the ISR keeps publishing into a single slot. Every
  number above rests on 80% of the data.
- **The jog defect is untouched.** Movements finishing inside `MIN_TRAVEL_MS`
  still latch until the 300 s failsafe, so under inspection jogging the device
  is effectively always sounding.
- **No gentle stop was deliberately produced.** All three arrivals were normal
  automatic stops. §14.7's case was gentle *because the machine set down
  softly*, and whether the brake still bounces on the softest possible stop is
  the assumption the whole result rests on.
