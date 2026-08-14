# Customer UI requests — heartbeat, low-battery chirp, and a 7/8 beacon

**Date:** 2026-08-14
**Head:** `Falcon_Rel_EFT` on `Dave356w/Spanidea_Falcon_shadow`
**Firmware on the device:** `31464` — Flash 31464/32256 (792 free), RAM 1357/2048
**Session A execution (battery, BOD, failsafe) continues in** `falcon_test_plan_2026-08-14.md`
**Session type:** bench, one unit, no hoistway runs
**Trigger:** three questions from the customer

---

## 1. What was asked

1. Can the 10 s calibration window be shortened to 5 s, or ended when the device
   judges itself stable?
2. At rest the unit is dormant with no indication of whether it is off or armed.
   Can a single LED give a slow heartbeat? Maybe the centre LED only.
3. What is the low-battery behaviour? Could it be a smoke-alarm-style chirp — a
   quick chirp with a single LED?

**All three were implemented and are on the device.** Item 1 shipped as 6 s
rather than the 5 s asked for, and the reason is the interesting part (§6).

**The session also found and fixed a defect nobody was looking for: the beacon
had been lighting seven of its eight ring LEDs for the entire project.** That is
§4, and it is the most consequential thing in this note.

---

## 2. Low battery

### 2.1 The cadence was the real defect

`check_for_battery_voltage()` scheduled itself on `adc_loop_counter == 30000`
and `== 30100` — **loop passes, not milliseconds**. The period was therefore
whatever `loop()` happened to take, and `loop()` at 1 MHz is dominated by an
11.4 ms sensor read. The observed consequence was ~23 minutes of uptime before
battery telemetry converged, which §6.1 of the product document had written down
as a bench workaround rather than as a bug.

**A maintenance visit is minutes long.** An alert that cannot arm inside ~20
minutes will never fire during the job it exists to protect, so the low-battery
warning was in practice dead code — on a device whose worst failure is silence
while moving.

Now a non-blocking two-phase wall clock: enable the divider, return, read
`BATTERY_SETTLE_MS` (10 ms) later, disable. `BATTERY_SAMPLE_PERIOD_MS` is 30 s,
so the 8-deep average spans 4 minutes and the alarm can arm ~90 s after boot.
Nothing spins, so the 2 s watchdog is untouched.

**Measured on the bench the same day:**

```
t=29818   Voltage value : 2542  (settling, ignored)
t=59899   Voltage value : 2551  (settling, ignored)
t=120061  Voltage value : 2533  avg : 2535
t=150142  Voltage value : 2542  avg : 2536
```

Exactly 30 s apart, two discards as designed, average live from ~90 s.

### 2.2 The pattern was the wrong response to a flat pack

`check_for_battery_alarm()` ran `BATTERY_FLASH_TIME_MS` 600 with `counter_a`
cycling 1..6 and the piezo driven while `counter_a < 4` — **1800 ms on / 1800 ms
off, continuously, forever.** Roughly 50% piezo duty. The inline comment claimed
"200 ms on, 300 ms off" and had not matched the constants for some time.

A continuous alarm was measured browning the board out at 242.7 s **on fresh
cells**. Answering "the pack is low" with a 50%-duty sounder spends the remaining
charge faster than anything else the firmware can do — it accelerates the failure
it is reporting. The customer's request is a straight improvement, not a
concession.

Now: one 80 ms piezo pulse plus one D2 flash every 45 s. Duty falls from ~50% to
0.18%, about 280× less energy per unit time. It fires immediately on the low
transition, and is **suppressed outright while the movement beacon is sounding**.

That suppression retires a hazard the old code had to handle by call ordering.
`loop()` runs `check_for_active_alarm()` then `check_for_battery_alarm()`, so the
battery path writes `buzzer_on` last and wins; the old 1800/1800 pattern could
clear it — **unblanking the accelerometer** — for over a second while the movement
buzzer was still beeping. With the chirp suppressed there is no second writer.

### 2.3 Two bugs closed on the way

**A latched blanking flag.** `disable_battery_alarm()` left `buzzer_on` set if
recovery landed inside a chirp. That flag gates the accelerometer read in the
timer ISR and nothing else would ever clear it — the device would have been deaf
to departures for the rest of the deployment. Narrow window, silence-while-moving
consequence.

**Two ADCs feeding one average.** `initialization()` pushed
`read_battery_voltage()` — the **external MCP3208**, whose `adc.begin()` is
commented out — into the same `battery_avg` that `check_for_battery_voltage()`
fills from the **internal ADC on PC2**. Harmless only by accident: `INIT_TIME_MS`
is 80 ms so it contributed ~4 samples, all overwritten by the later `fill()`.
Removed, along with the now-dead `temp_timer`.

### 2.4 ✅ The measurement is now trustworthy too

**Written when it was not**, and resolved later the same day. Recorded in both
states because the intermediate reasoning is what found the defect.

`BATTERY_LOW_THRESHOLD 1600` is compared against `read_adc_pc2_voltage()`, which
returns **millivolts** and does **not** apply `VBATT_CONST`. That constant —
5.1k/1.5k, ratio 4.4 — was a **V1 leftover describing the wrong board**, and it
is the single reason 1600 looked like an implausible 7.04 V pack while
`Release.txt` claimed 3.2 V. Same failure mode as `PIN_GREEN_LED` and the
`BATT_SENSE` MCP3208 path.

**Metered: the divider is 2:1** (499k/499k, sheet 3). So 1600 was always exactly
3.2 V and the historical thresholds were right all along. They are now stated in
PACK millivolts and halved at compile time; the shipping flash size was
byte-identical across that change, which is the proof it moved nothing.

**The ADC was then brought into spec** — `/128` → `/16`, 62.5 kHz — and the whole
chain validated:

```
meter at the pack   5.010 V
firmware pack_mv    5012        2 mV apart, inside one LSB (6.06 mV of pack)
```

One comparison validates the divider, the 3100 mV reference assumption and the
ADC configuration at once. At `/128` the same chain read 4860 against a 4900
meter — **0.8% low**, exactly the direction an unfinished sample-and-hold charge
errs from a 250 kΩ source.

⭐ **So the chirp can be presented as a calibrated warning.** The prohibition
this section originally carried is discharged. The trip was subsequently raised
3.2 V → **3.6 V**, because at 3.2 V it fired essentially at buck dropout and a
warning with no runway is not a warning. Full detail: test plan §A1–A3.

---

## 3. Idle heartbeat

A dim wink on **D2**, the centre LED, every 4 s. A **double** wink carries the
degraded calibration verdict.

### 3.1 It took two wrong homes to find the right one

**There is no green LED on V2.** Checked against all seven sheets of
`RTC1273R2_SCH.pdf`: **D1 is a `D5V0F1U2LP-7B` ESD diode**, not an indicator.
`PIN_GREEN_LED` is a V1 leftover pointing at PC2, which on this board is
`BAT_ADC` — the battery divider. Nothing to drive.

**The first implementation used the chase ring, and that was wrong twice over.**
D3–D10 are the *perimeter*; "centre LED" always meant D2. And selecting a ring
position means pulsing MR and *clocking* to it, because the 4017 has no
output-enable — the walked-through positions are genuinely lit on the way, order
10 µs each against the destination, roughly 1000:1 dimmer. Invisible on paper;
visible to a dark-adapted eye. Dave saw a faint D3→D4 chase ahead of every beat.
There is no firmware fix; a position cannot be selected without walking to it.

D2 is directly driven, so both problems disappear rather than being managed.

### 3.2 On D2 the beat is not free, so it is dimmed

Sheet 3 is titled **"LED DRIVER RED LED 200mA"**: D2 is an `LXM2-PD01-0050`
behind U2, a TPS92201 constant-current driver with EN and PWM control. On the
ring a beat cost nothing, because the 4017 lights exactly one output regardless
of which. Here it costs real charge:

⛔ **AND THE 200 mA IS THE DRIVER'S CAPABILITY, NOT ITS SETTING** — measured
later the same day at **38.8 mA**. Every figure this section originally carried
was arithmetic off the larger number and wrong:

| | claimed | MEASURED |
|---|---|---|
| D2 at full | 200 mA | **38.8 mA** |
| D2 at `HEARTBEAT_PWM_DUTY` | ~12 mA | **0.20 mA** |
| heartbeat average | ~250 µA | **4.0 µA** |

⭐ **The dimming turned out to be worth far more than the arithmetic suggested.**
Idle baseline is 3.2 mA, so a full-brightness beat would cost 0.78 mA — **24% of
idle** — against 4 µA dimmed. The decision was right; the reasoning behind it was
off by 60×.

⬜ **The dim is 12× dimmer than its duty implies** (0.52% of full against a 6.3%
PWM duty), most likely the TPS92201's soft-start losing most of a ~1 ms on-time
at Timer0's 61 Hz. Brightness is **not** linear in `HEARTBEAT_PWM_DUTY` here.
⚠️ **Nobody has confirmed the wink is visible** — that is the open question on
this feature. Test plan §A5.

**The flash is 80 ms and not 20 for a hardware reason.** `LED_PWM` is PD6 = OC0A,
and at F_CPU = 1 MHz the Arduino core's Timer0 PWM runs at 1e6/(64 × 256) =
**61 Hz**. A 20 ms window spans barely one PWM cycle, so the "dim" beat would land
as a single short pulse whose length depended on where the window fell in the
cycle. 80 ms spans about five cycles and integrates to a genuine dim glow.
⚠️ Do not shorten below ~50 ms without moving off Timer0.

Timer0 is safe to use: the firmware configures **Timer1 only** (the 25 Hz sample
CTC). Driven by direct `OCR0A` / `COM0A1` writes rather than `analogWrite()`,
which switches over every timer pin on the part and cost **310 bytes** — against a
documented 1198-byte shortfall for `WIRE_TIMEOUT` competing for the same headroom.

### 3.3 It carries the calibration verdict

"Armed, but the site could not be measured — expect a twitchy device" was
announced once as 3 chirps at the end of calibration and then never again. A
mechanic who walked away during the chirp had no way to recover it. The
double-blink keeps that state visible for the whole deployment. Free, and it
closes a real gap.

D2 is shared three ways — alarm blast, chirp, heartbeat. Arbitration is strict
priority in `loop()` order: the alarm suppresses both others, and a chirp in
flight suppresses the individual beat. ⚠️ The battery case suppresses the *beat*,
not the heartbeat: a low pack is exactly when "is this still alive?" matters.

---

## 4. ⭐ The beacon was running at 7/8

**Found by reading the schematic, not the code.**

The 4017 has ten outputs; sheet 6 fits eight LEDs. Two outputs go nowhere — and
**Q0, the reset position, is one of them. D3 is Q1.** That is a deliberate and
rather good hardware decision: it gives the ring an off state the part does not
natively have, because exactly one output is always high and MR parks it on the
one that lights nothing.

Established from two independent observations: the ring is dark at rest, and the
first heartbeat implementation — MR then three advances — was seen to transit D3,
D4 and settle on **D5**, which is Q1, Q2, Q3. Both require D3 = Q1.

**The consequence, predicted from the map and confirmed at the bench the same
day:** the alarm sweep is `CHASE_SWEEP_STEPS` = 8 steps, and MR consumed one of
them on an output that lights nothing. Only **seven** positions ever showed —
Q1..Q7 = D3..D9. **D10 never lit during an alert** and the ring carried a visible
gap. Dave: *"D10 doesn't light on alert, chase skips D10."*

The beacon is the product's primary visual — the thing a mechanic locates a
moving counterweight by — and it had been at 7/8 for the whole project.

### 4.1 The fix, and why not the obvious one

Step 0 now pulses MR **and advances onto Q1 in the same step**, so the sweep shows
D3..D10 across the same eight steps.

The obvious fix — `CHASE_LED_COUNT` 8 → 9, buying a step for the reset — forces
`ALARM_SEQ_STEPS` to a multiple of 9, and 16 is not one. That drags in a new
sequence length, a new cadence, and a re-check of the blanked/quiet table that
`alarm.h` spends a page justifying. Folding the advance into the reset step costs
two `digitalWrite`s and changes **no timing at all**:

| | before | after |
|---|---|---|
| sequence | 800 ms | 800 ms |
| blast | 150 ms at step 0 | 150 ms at step 0 |
| blanked fraction | 25% | 25% |
| contiguous quiet | 600 ms | 600 ms |
| ring positions lit | **7** | **8** |

The blast now coincides with D3 lighting rather than with a dark slot, which is
if anything the better alignment.

✅ Confirmed on the device: all eight sweeping, no gap.

### 4.2 The method note that earned its keep

This defect is invisible from the source. `check_for_buzzer_alert()` is correct
on its own terms — `CHASE_LED_COUNT` really is 8 and the sweep really is 8 steps.
It only becomes a bug once you know Q0 lights nothing, and that fact lived
nowhere in the tree; it was on sheet 6 of a PDF.

**The firmware names and the board labels were never the same vocabulary**, and
that cost two bench iterations on the heartbeat before it landed on the right
part. The silkscreen map is now in `common.h`, at the top, next to the pin
defines — which is the first place the next reader will look.

⚠️ Note also that an intermediate version of that map asserted the *opposite* —
that D3 burns current continuously — with confident reasoning from a wrong
premise. It survived about twenty minutes, until Dave said the ring was dark.
That is the ninth interpretive claim overturned by the next observation in three
sessions, and it is the reason §8 of the state-of-project note says what it says.

---

## 5. What was changed

| file | change |
|---|---|
| `main.cpp` | battery cadence → wall clock; MCP3208 feed removed; threshold units comment corrected; `heartbeat_service()` in `loop()` |
| `alarm.cpp` | chirp replaces continuous sounder; `buzzer_on` cleared on disable; heartbeat on D2; sweep advances onto Q1 |
| `alarm.h` | chirp and heartbeat constants and rationale; `CHASE_LED_COUNT` semantics |
| `common.h` | silkscreen map D1/D2/D3–D10 |
| `movement_service.cpp` | heartbeat armed with the calibration verdict; `XY: bmax` dump |
| `lateral.cpp` / `.h` | sort a copy so `bmax[]` keeps time order; `bucket()` accessor |
| `graph/calib_replay.py` | new — replays the window length from `XY: bmax` lines |
| `movement_service.h` | `CALIB_TIMEOUT_MS` 10 s → 6 s + static_asserts; `LATCH_FAILSAFE_MS` 240 s → 600 s |
| `adc.cpp` | ADC prescaler `/128` → `/16`; discarded conversion for the 250 kΩ source |
| `main.h` | `BATT_R1`/`R2`/`VBATT_CONST` removed — V1 leftovers describing the wrong board |
| `platformio.ini` | new `bench_battery` and `idle_current` bench envs |
| `Falcon_Product_Document.md` | §2.1 third hardware fact; §6.1 23-minute step corrected; calibration 10 s → 6 s |

Flash 30660 → **31464** (792 free). RAM 1369 → **1357**.

**Fuses changed:** `efuse 0xF7` → `0xF5`, enabling BOD at 2.7 V.

---

## 6. Item 1 — calibration window: assessed, then SHIPPED at 6 s

⚠️ **This section was written while the answer was still 'deferred'.** It is
kept in order because the reasoning is what produced the measurement; the
outcome is §6.5, and it is **6 s with quorum 4**, not the 5 s recommended
below.

**Shortening to 5 s is defensible and the argument is stronger than it was**, but
it touches a safety threshold and is the one request of the three that can move
the release path, so it was not done in the same session as two UI changes.

`CALIB_TIMEOUT_MS` covers two measurements: the lateral noise floor (10 × 1 s
buckets, median of per-second maxima, quorum 6) and the Z zero.

Two reasons it can safely shorten:

- **The error direction is safe.** `lateral.h` spells it out: a short window
  *underestimates* a peak → threshold too low → over-eager beacon. Annoying, not
  dangerous. The dangerous direction needs an over-estimate.
- **The quorum comment is stale and in our favour.** It is written for 3.13 Hz —
  "a 1 s bucket holds ~3 samples". At 25 Hz a bucket holds ~25. **A 5 s window
  today carries ~125 samples against the ~31 the original 10 s window was
  designed around.**

Recommended shape: minimum 3 s, exit early when buckets agree and no any-motion
edge has fired, hard cap 5 s — which is the customer's "whenever the device deems
stable" with a floor. Scale `XY_CALIB_MIN_BUCKETS` with it (3 of 5, not 6 of 5,
which would reject every window).

The larger UX win is elsewhere: **`CALIB_RETRIES 2` means a contaminated
calibration costs 30 s today, not 10.** At 5 s that worst case becomes 15 s.

`XY_STILL_MIN`/`MAX` are flagged UNMEASURED in `lateral.h` — the clamps this
rides on have never been checked against a real counterweight.

### 6.1 ⛔ Correction: X/Y were already logged

An earlier draft of this note, and the advice given during the session, claimed
X/Y are fed to `lat_monitor` and **never printed**. **That was wrong.**
`main.cpp:1984` prints `x=` and `y=` from `s.ax`/`s.ay` — the raw per-sample
values — and `m=` is `lat_monitor.m()`, the calibration metric itself. It was in
every log on file, including ones read earlier the same session.

The real obstacle is different and smaller: `LOG_DECIMATE_N 8`. The logged `m`
values are individually correct, but only one sample in eight appears, so a
bucket maximum rebuilt from the log is a max over ~3 samples instead of ~25 and
systematically under-reads.

### 6.2 ⚠️ Why the fix was NOT to log at full rate

Un-thinning the sample log during calibration is the obvious move and it is
**dangerous**. `Serial.print` blocks `loop()`, `loop()` drains the sample ring,
and ~20% of samples are already lost to overrun. The metric is |Δax| + |Δay|
between **consecutive** samples, so a lost sample widens `dt` and **inflates m** —
which inflates the learned threshold, which is the direction that makes travel
read as still. Adding serial load inside the measurement window would bias the
very number being measured, the wrong way.

### 6.3 What was done instead, and what it says

The device already computes per-second bucket maxima; they were simply never
emitted. `calib_finish()` now sorts a **copy**, so `bmax[]` keeps time order, and
`movement_service.cpp` dumps the sequence after the window has closed — costing
nothing inside it:

```
XY: calib b=10 peak=0.0730 mv=0
XY: bmax 0.0680 0.0690 0.0550 0.0540 0.0580 0.0530 0.0580 0.0510 0.0730 0.0520
```

That one line makes every window length answerable offline, because a shorter
window is the same arithmetic over a prefix. `graph/calib_replay.py` does it.

**Six bench calibrations, same mounting, unit at rest on a desk:**

| window | worst deviation in the DANGEROUS direction |
|---|---|
| 3 s | **+23.6%** — rejected |
| 4 s | +0.0% |
| 5 s | +5.5% |
| 6 s | +0.0% |
| 8 s | +0.0% |

⭐ **PARITY IS DOING REAL WORK, and it was not designed for this.**
`calib_finish()` indexes the sorted buckets at `(n-1)/2`, which on an **even**
count is the **lower-middle** — `lateral.h` chose that deliberately because the
lower of the two is the smaller threshold, i.e. the safe direction. An
even-length window inherits that cushion; an odd-length window takes the true
median and has none. That is why 4 s and 6 s never exceeded the 10 s threshold in
any of the six runs while 3 s and 5 s did. **If the window is shortened, shorten
it to an even number of buckets.**

⛔ **`XY_CALIB_MIN_BUCKETS` IS 6.** Any window shorter than 6 s is rejected
outright by the quorum check and falls back to `XY_STILL_MIN` — an over-eager
device on every install. **The quorum must move with the window or nothing else
matters.** This is the trap in the whole exercise: a 5 s `CALIB_TIMEOUT_MS` alone
would not shorten calibration, it would disable it.

☠️ **A single-run pattern died on contact with five more runs.** Run 1 put its two
highest buckets first, which read as "the start of the window is noisiest —
the device was just set down". Across six runs the first two buckets average
0.0585 against 0.0579 for the rest. **No such effect.** Recorded because it is
the tenth interpretive claim in four sessions to be overturned by the next
measurement, and it was caught only because the numbers were checked before
being written down.

### 6.4 What is still needed

**Bench captures on a desk are not a counterweight in a hoistway.** What
transfers is the SHAPE of the comparison, not the numbers; a site with a wider
bucket-to-bucket spread makes every short window noisier than it looks here.
`XY: bmax` now appears on every calibration attempt, including retried ones, so
**the next hoistway session collects this for free** — no protocol change, no
extra capture. Run `calib_replay.py` over those logs and the decision is made on
site data.

### 6.5 ✅ SHIPPED: 6 s, quorum 4

Dave's call, same session. `CALIB_TIMEOUT_MS` 10000 → **6000**,
`XY_CALIB_BUCKETS` 10 → **6**, `XY_CALIB_MIN_BUCKETS` 6 → **4**.

**The quorum had to move or the change would have been actively harmful.** At 6
of 6, every bucket would have had to report or the calibration would be rejected
and the device would arm on `XY_STILL_FALLBACK` — over-eager, on every install,
from what looks like a pure timing edit. 4 of 6 preserves the original 60% ratio
rather than the absolute count. It is not 3 because a median over three
tolerates only one contaminated bucket.

**Two `static_assert`s now tie the constants together**, because they live in
different headers and nothing else related them:

```
CALIB_TIMEOUT_MS / XY_CALIB_BUCKET_MS == XY_CALIB_BUCKETS
XY_CALIB_MIN_BUCKETS                  <= XY_CALIB_BUCKETS
```

Verified by deliberately setting the window to 7000 and confirming the build
**fails** with the intended message — a guard that has never been seen to fire
is not yet a guard.

**Verified on the device, four fresh calibrations:**

```
XY: bmax 0.0620 0.0600 0.0570 0.0530 0.0530 0.0580   ->  0.0855
XY: bmax 0.0590 0.0580 0.0620 0.0450 0.0510 0.0530   ->  0.0795
XY: bmax 0.0460 0.0620 0.0560 0.0710 0.0550 0.0680   ->  0.0840
XY: bmax 0.0460 0.0500 0.0460 0.0840 0.0540 0.0580   ->  0.0750
```

All `b=6`, all `FSM: READY`, thresholds 0.0750–0.0855 — inside the 0.0750–0.0915
band the 10 s window produced on the same mounting, with no upward drift.
Calibration completes at t≈13.3 s against t≈17.5 s before: the full 4 s.

Worst case with `CALIB_RETRIES 2` goes 30 s → **18 s**.

⬜ **Still bench evidence only.** The window is now short on the strength of six
desk calibrations. `XY: bmax` prints on every attempt, so the next hoistway
session validates it for free — and if the site's bucket spread is wider than
the bench's, `calib_replay.py` over those logs is what says so.

---

## 7. Next

Superseded by `falcon_test_plan_2026-08-14.md`, which consolidates every open
item and is the live document. In short, as of end of day:

- **Closed today:** items 1 (brownout was the COM link), 7 (trip point), 8 (ADC
  prescaler), 9 (BOD), 10 (quiescent current).
- **Open, bench, no code needed:** hear the chirp; see the degraded double wink;
  confirm the heartbeat is visible at all.
- **Open, biggest remaining bench win:** a production build with logging
  compiled out — measured at 1.10 mA, **34% of idle**, worth ~50% more standby.
- **Open, in the release path and untouched by any of this:** §4.2, §5.1, §5.2,
  §5.3, §5.6.

---

## 8. Bench notes

- The programmer now enumerates as **COM7**, not the `COM6` hardcoded at
  `platformio.ini:63`. Passed on the command line; the file is unchanged because
  COM numbers move.
- The monitor port is COM5 at 62500.
- Opening the monitor does **not** reset the board — capture what you need or
  power-cycle deliberately.
- The calibration this unit armed with, for the record:

```
XY: calib b=10 peak=0.0700 mv=0
FSM: READY
Zero-Calib-Value : 9.759331
XY-Still-Value   : 0.0765
```

Ten buckets, no movement, clean verdict — so a **single** heartbeat wink.
