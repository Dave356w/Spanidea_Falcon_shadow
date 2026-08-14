# Falcon — Product Document

**Revision:** 2026-08-13 · **Firmware:** `4272136` · **Branch:** `Falcon_Rel_EFT`
**Platform:** ATmega328PB @ 1 MHz, hardware version 2 (Voguvant)

This is a **living document**: product definition, implemented state logic,
development practice, and test procedure in one place. It is written to be read
cold, without the engineering session notes.

| I want… | read |
|---|---|
| what the product is and how it behaves | §1–§3 here |
| exactly how a decision is made | §4 here, then `Eng_Notes/falcon_reference_2026-08-12.md` |
| to build, flash, and read a log | §5 here |
| to run a hoistway or bench test | §6 here |
| what is proven and what is not | `Eng_Notes/falcon_state_of_project_2026-08-13.md` |
| why a threshold has its value | the comment at its `#define`. Always. |

⚠️ **The source comments are the authority on every threshold.** Each carries the
measurement and the reasoning that set it, including changes that were tried and
reverted. Read the comment before changing the number.

---

# 1. Product

## 1.1 What it is

A battery-powered **movement beacon** placed on an elevator counterweight (or
cartop) during maintenance. While the counterweight moves, it sounds a piezo and
runs a chase-LED sweep so a mechanic working in the hoistway can locate it by ear
and eye. **When the counterweight stops, the beacon must stop.**

It is an awareness instrument, not a control or safety-interlock device. Nothing
it does affects elevator operation.

## 1.2 Primary use case — inspection operation

A mechanic works in the hoistway while the car is driven on inspection
(continuous-pressure) control: slow, frequent, often short moves. The
counterweight travels opposite the car and is the hazard the mechanic cannot
easily see or hear.

The device is placed on the counterweight before the job, powered on, and left
unattended for the duration. **There is no manual silence** — the mechanic cannot
reach it once the job starts. Everything the device does, it must do alone.

The mechanic **ranges the beacon by ear and eye**: the beep rhythm and the LED
sweep are how distance and direction are judged. This makes the alarm pattern a
genuine product parameter, not a cosmetic one — see §3.4.

## 1.3 Secondary use case — normal automatic operation

The car runs under normal automatic control, typically during commissioning or
troubleshooting. Speeds are much higher (300–500 fpm measured) and stops are
drive-controlled rather than brake-set.

This case is **covered less well** than inspection operation. See
`falcon_state_of_project_2026-08-13.md` §4.2.

## 1.4 The physical constraint that shapes everything

**At constant velocity, a moving car and a parked car are indistinguishable on
the vertical axis.** Both read 1 g. There is no filter, threshold or duration
that separates them, because there is no signal to separate.

Three consequences drive the entire architecture:

1. Movement can only be detected as **events** — a departure transient and an
   arrival transient — never as a state.
2. The alarm must therefore be **latched**: set on departure, held through the
   unobservable middle, released on arrival.
3. A "has it gone quiet?" rest detector **cannot exist** on this axis. One was
   built and released the beacon mid-ride twice.

The lateral axes carry no gravity pedestal and were the hope for a rest detector.
Measured parked-vs-cruise contrast: **1.19×**. That path is dead.

## 1.5 Failure modes and the design axiom

| # | failure | severity |
|---|---|---|
| 1 | **Silence while moving** — the mechanic believes a moving counterweight is parked | **Catastrophic** |
| 2 | **A position lie** — beacon asserting movement over a stationary counterweight | Destroys trust; drains the pack |

> ## The axiom
> **A detector may fail toward alarming. It may never fail toward silence.**

Every design decision in §4 follows from this. Where detectors are OR'd rather
than AND'd, where a hit is made sticky, where a gate errs generous — this is why.

---

# 2. Hardware

| item | detail |
|---|---|
| MCU | ATmega328PB, **1 MHz** (internal 8 MHz RC ÷ 8, `lfuse 0x62`) |
| Accelerometer | **BMA456** on **TWI1** (`Wire1`), sampled at **25 Hz** by Timer1 CTC |
| Audible | piezo on `PIN_PIEZO` |
| Visual | **D3–D10**, 8 ring LEDs on a **74HC4017 decade counter** (outputs Q1–Q8; Q0/Q9 unpopulated); **D2**, a 200 mA centre LED behind a TPS92201 driver |
| Battery sense | internal ADC on PC2 (`BAT_ADC`); the MCP3208 path is no longer used |
| Pressure | DPS310 **depopulated** — pressure code removed; **TWI0 carries nothing** |
| Flash / RAM | 31210 / 32256 · 1373 / 2048 |

## 2.1 Three hardware facts that constrain the firmware

**The 74HC4017 is a decade counter — exactly one output is high at any instant.**
There is no state in which all eight LEDs are on, and no firmware change can
create one. `PIN_CHASE_LED` is the **master reset (MR)**, not a data line;
`PIN_CHASE_CLK` advances. An apparent all-on flash is achieved by
persistence-of-vision multiplexing (§4.8).

**Q0 is unpopulated, and D3 is Q1.** Ten outputs, eight LEDs. Parking MR on a
dead output is what gives the ring an off state the part does not natively have,
so the ring costs nothing at rest. It also means **the reset step lights
nothing** — until 2026-08-14 the alarm sweep spent one of its eight steps there
and D10 never lit during an alert. See `falcon_ui_battery_2026-08-14.md`.

**D2 — the centre LED — is a 200 mA part behind a constant-current driver.**
Unlike the ring, where the 4017 lights exactly one output regardless, anything
that drives D2 on a duty cycle costs real charge. `LED_PWM` is PD6 = OC0A, so
brightness is dimmable in hardware, and the idle heartbeat uses that.

**The sensor read costs 11.4 ms of every 40 ms period — 28% of CPU.** This, not
the serial port, is the dominant cost in the sample-loss budget.

## 2.2 Settings that should be reviewed

- **BOD is probably disabled** (ATmega328 factory efuse `0xFF`). For a device
  whose worst failure is silence, a sagging rail that causes a clean reset and
  reboot is far better than one that hangs.
- **The ADC prescaler is `/128` under a comment reading "for 8MHz clock"** while
  F_CPU is 1 MHz — an ADC clock of 7.8 kHz, below the 50 kHz datasheet minimum
  for full 10-bit accuracy.

---

# 3. Behavioural requirements

## 3.1 Core

| # | requirement | status |
|---|---|---|
| R1 | Sound whenever the counterweight is moving | met — departures caught 17–500 fpm |
| R2 | Stop when it stops | met on inspection; **marginal on automatic** (§4.3) |
| R3 | Never assert movement over a stationary counterweight | **one known reproducible violation** (§4.4) |
| R4 | Operate unattended for a full job, no manual intervention | endurance 588 s continuous alarm verified stationary |
| R5 | Self-calibrate to its own mounting at power-on | met (§4.2) |
| R6 | Report readiness unambiguously | met (§4.8) |

## 3.2 Timing budgets

| event | budget | actual |
|---|---|---|
| Turn-on after departure | 1–2 s (customer decision 2026-08-09) | ~200 ms dwell + latch |
| Reset after arrival | originally 1–3 s; **amended to ~8 s by the customer** | ~5 s confirm + transitions |
| Calibration at power-on | ~10 s | 10 s, retried up to 2× if movement is seen |

## 3.3 Failsafe

If no arrival is ever detected, the latch releases after `LATCH_FAILSAFE_MS`
(**240 s**) and logs the release **as a fault**, so it is never mistaken for a
normal release.

⚠️ 240 s is a **placeholder**, not a tuning value. It was clamped down from 600 s
to stay inside an apparent hardware endurance limit that has since been refuted
(`falcon_500fpm_ui_2026-08-13.md` §9.7). It should be raised once the vibration
hypothesis in §9.8 of that note is cleared.

## 3.4 The alarm pattern is a product parameter

The mechanic ranges the beacon by ear. Beep length, cadence and the LED sweep are
therefore specified behaviour, and any change trades against two other budgets:

- **piezo duty → average current → alarm endurance**
- **blast length + ringdown → sensor blanking → detection quality**

Current values and the arithmetic linking them are in §4.8.

---

# 4. State logic as implemented

## 4.1 The finite state machine

```
power on (device already placed on the counterweight)
   │
   ├─ STATE_CALIBERATION ── 10 s: learn the z zero and the lateral floor
   │        rejected and retried if movement is seen
   │        1 chirp = good · 3 chirps = fell back
   │                      + apparent all-LED flash
   │
   ├─ STATE_MONITORING ─── any-motion interrupt ──► departure
   │                                                  │
   │                                    STATE_MOVEMENT_DETECTED (200 ms dwell)
   │                                                  │
   │                                          STATE_MOVING
   │                                      beacon ON, LATCHED
   │                                                  │
   │                        ┌─────────────────────────┴──────────────────────┐
   │                        │   arrival detection — three paths, §4.3        │
   │                        └─────────────────────────┬──────────────────────┘
   │                                                  │
   │                                       STATE_DECELERATING
   │                            5 s of continuous quiet required
   │                                                  │
   │                                        STATE_STOPPED
   └──────────────────── beacon OFF ──────────────────┘
```

| `st=` | state | meaning |
|---|---|---|
| 1 | `STATE_CALIBERATION` | 10 s learning window at power-on |
| 2 | `STATE_MONITORING` | parked, watching for a departure |
| 3 | `STATE_MOVEMENT_DETECTED` | departure seen, dwelling before the beacon |
| 4 | `STATE_MOVING` | beacon latched, hunting the arrival |
| 5 | `STATE_DECELERATING` | arrival seen, confirming the car settled |
| 6 | `STATE_STOPPED` | transient; re-enters MONITORING |
| 7 | `STATE_ERROR_RESET` | boot state |

## 4.2 Calibration

Ten seconds at power-on learns the **z zero** (the 1 g pedestal in this
mounting's attitude) and the **lateral noise floor** (`XY_STILL`). An any-motion
edge during the window means the counterweight moved while its own noise floor
was being measured — the window is discarded and retried, up to `CALIB_RETRIES`.

**Calibration is per-mounting and per-power-cycle.** Moving the device without a
power cycle leaves a stale zero.

## 4.3 Detection

**Departure — the sensor's own engine.** The BMA456 any-motion feature runs on
the sensor's 100 Hz stream and raises an interrupt. It is the only departure
detector that works: the firmware's own rolling average spans 1.28 s and
flattens a departure ramp to nothing. A polled threshold test is retained as a
redundant path, OR'd never AND'd, per the axiom.

**Arrival — three paths, all reaching the same exit.** Detection is on the **raw
sample**, not the average: a 0.938 m/s² brake bounce registers 0.0072 on a
32-sample average.

| # | path | fires on | status |
|---|---|---|---|
| 1 | clustered any-motion **AND** raw peak | ≥2 edges in 2.5 s, corroborated by the peak | live |
| 2 | raw windowed peak | peak > `ARRIVAL_PEAK_VALUE` (0.45) | live, **marginal** |
| 3 | ramp detector | sustained one-signed deceleration | **armed; never yet executed** |
| — | jog verdict | a jog's impulse pair | live, releases ~4 s after latch |
| — | failsafe | `LATCH_FAILSAFE_MS` elapsed | fault, logged as one |

**The peak must be windowed, not cumulative** — a running maximum ratchets
through cruise vibration and eventually crosses any threshold, which is the
catastrophic direction reached by a detector that cannot forget. Two alternating
1 s buckets give a 1–2 s sliding window.

**But it must also latch.** A jog's stop crossed the threshold before
`MIN_TRAVEL_MS` opened the gate — detected, then discarded. `arr_hit` is sticky
for the run.

⚠️ **Path 2 is the weakest number in the product.** 34 automatic stops measure
0.454–0.713 against the 0.45 gate; worst margin **1.009×**. The ramp detector
exists to answer this and has not yet had the opportunity.

## 4.4 🔴 Arming — two independent gates, and the safety-critical part

Neither the peak collector nor the ramp accumulator runs until **its own** gate
opens. Without this the departure transient would satisfy the arrival test a
fraction of a second after the latch.

| detector | gate | opens on |
|---|---|---|
| peak collector | `arr_armed` | `ARRIVAL_ARM_SAMPLES` quiet samples **OR** `ARM_REV_SAMPLES` sign reversals |
| ramp accumulator | `ramp_gate` | **sign reversal only** — independent counter |

**Why they are separate.** The ramp block test cannot distinguish a departure
ramp from an arrival ramp — same shape, opposite sign, and sign encodes direction
of travel, not phase. 47/89 departure bursts on file qualify the ungated test.
**A slow departure starts *below* the quiet band**, so the quiet path can arm on
the departure's own opening samples and then feed the departure ramp to the
accumulator, producing a verdict indistinguishable from an arrival. With the ramp
armed, that releases the latch seconds after a car starts moving. A sign reversal
means the departure ramp has *ended*, which is what the ramp detector actually
needs to know.

⚠️ **Never re-merge these gates, and never remove `arrival_peak_hit()` from the
any-motion arrival path** — that AND is the only thing keeping the current alarm
timing (75% quiet fraction) clear of a historical false-release mode.
Re-run `graph/arming_replay.py` after any change to arming.

🔴 **Known limitation — the single-floor blind spot.** A single-floor terminal
approach goes departure ramp straight into deceleration: no cruise, no quiet
samples, so the peak's gate never opens. `pk` reads 0.00 through a real
excursion, and the beacon has sounded **85 s over a car stationary for 78 of
them.** Reproducible on demand. **This violates R3 and is unfixed.**

## 4.5 Release confirmation

Reaching `STATE_DECELERATING` silences nothing. The average must stay within
`STOP_BAND_VALUE` of the zero calibration for `STOP_CONFIRM_MS` **continuously**;
any excursion restarts the window. This holds the beacon through levelling, which
would otherwise read as "stopped" and let the subsequent brake set re-latch as a
fresh departure.

After release, `MONITOR_REARM_MS` of deafness prevents arrival ringing
re-latching.

## 4.6 The jog verdict

A movement shorter than `MIN_TRAVEL_MS` has its stop discarded unseen, so a jog
would latch until the failsafe. The verdict classifies the impulse pair: real
departures measure **opk ≤ 440**, jogs **≥ 1366**, gate at **900**. Opk carries
the verdict; the ratio is only an AND guard. **29/29 lifetime, zero false
releases.**

## 4.7 Reliability layer

| mechanism | purpose |
|---|---|
| 2 s **watchdog**, kicked only in `loop()` | if `loop()` stops being reached, reboot. 4 catches |
| **bounded TWI waits** (vendored `Wire1`) | a bus glitch used to wedge the state machine inside the sample ISR and freeze the device. 3 wedges caught, recovered transparently |
| `Reset cause:` at boot | `0x8` = watchdog caught a lockup; `0x6`/`0x2` = external; `0x1` = power-on |

**Why this matters:** on 2026-08-12 a pair of lockups cost an entire car run — the
beacon never sounded while a counterweight moved, which is failure mode 1.

## 4.8 Alarm and indicator sequence

One master step counter drives buzzer, chase and red LED, and **firmware pulses
MR at the start of every sweep** — so the sequence length is defined by firmware
rather than by the 4017's natural modulus.

```
ALARM_STEP_MS 50   ALARM_SEQ_STEPS 16  -> 800 ms sequence
ALARM_BUZZ_STEPS 3                     -> 150 ms blast, at the sequence start
ALARM_RED_STEPS 4                      -> 200 ms red LED, with the blast
CHASE_STEPS_PER_LED 1, CHASE_LED_COUNT 8 -> 8 LEDs in 400 ms, 2 sweeps/sequence
BUZZER_RINGDOWN_MS 50                  -> sensor stays blanked 50 ms after the pin drops
```

| property | value |
|---|---|
| piezo duty (→ current) | **18.75%** |
| sensor blanked fraction | **25%** |
| longest contiguous listening window | **600 ms** |
| audible cadence | 1.25 Hz |

⚠️ **Speeding up the *buzzer* costs sensor coverage; speeding up the *chase* is
free.** The ringdown is a fixed 50 ms paid once per blast and does not scale:

```
blanked %        = 18.75% + 5000/T          contiguous quiet = 0.8125·T − 50 ms
T = 400 ms → 31.3% blanked, 275 ms quiet    (worse than before this design)
T = 800 ms → 25.0% blanked, 600 ms quiet    (current)
```

The chase does not gate the sample read, and the 4017 holds exactly one output
high however fast it is clocked — so chase speed costs neither coverage nor
current.

**Ready signalling.** On calibration completion: **1 chirp** = good, **3 chirps**
= fell back (calibrated while moving, or noisy). Each chirp is accompanied by an
apparent **all-LED flash** (persistence-of-vision multiplex, §2.1) and the red
LED.

## 4.9 Armed / unarmed status — read this before trusting a release

| path | armed | live evidence |
|---|---|---|
| departure (any-motion) | ✅ | caught 17–500 fpm, never missed |
| arrival — any-motion + peak | ✅ | multiple live releases |
| arrival — windowed peak | ✅ | multiple live releases, **worst margin 1.009×** |
| **arrival — ramp detector** | ✅ armed | ⚠️ **never executed** in 14 latches |
| **reversal arming (`v=2`)** | ✅ armed | ⚠️ **never fired** in ~32 runs |
| jog verdict | ✅ | 29/29 |
| x/y lateral release | ⛔ removed | measured dead (1.19× contrast) |
| velocity conservation | ⛔ removed | measured dead |

**Two of the armed paths have never been observed working.** They are armed on
replay and negative evidence, and they carry the automatic-operation boundary
between them.

---

# 5. Development

## 5.1 Repository

```
falcon_srcs/          PlatformIO project — open THIS folder
  src/                firmware
  lib/Wire, lib/Wire1 VENDORED, deliberately modified — see §5.3
  graph/              log parsing and replay tooling
  logs/               captures (gitignored — local only)
Eng_Notes/            dated engineering notes; reference doc + state of project
Schematics/ HW_Docs/  hardware, including the BOM
```

Branch: `Falcon_Rel_EFT` on `Dave356w/Spanidea_Falcon_shadow`.

⚠️ **`falcon_srcs/logs/` is gitignored.** The 186-burst corpus the replay tooling
depends on exists only on the development machine — replay findings are not
reproducible from a fresh clone.

## 5.2 Build and flash

```bash
cd falcon_srcs && pio run                      # shipping build
cd falcon_srcs && pio run -t upload            # flash (programmer on COM6)
cd falcon_srcs && pio device monitor -b 62500  # capture (logging port, COM5)
```

- Programmer and logging cable enumerate as **different COM ports**.
  `upload_port` is deliberately left machine-specific.
- avrdude: `C:\Users\DELL\.platformio\packages\tool-avrdude\avrdude.exe` (no `bin\`).
- **Serial is 62500 baud.** 115200 is *unreachable* at F_CPU = 1 MHz — the
  nearest divisor is 8.5% off, which is why an earlier attempt produced a blank
  terminal. Reachable exact rates: 31250, 62500.

## 5.3 Vendored libraries — do not replace with the framework copies

| library | why vendored |
|---|---|
| `lib/Wire1` | every TWI wait loop bounded by a 16-bit spin counter. Upstream waits are unbounded and the sample read runs in the ISR, so a glitch froze the device. `TWI1_GUARD_SPINS` is a **spin count, not a time** — re-check if F_CPU or the TWI rate changes |
| `lib/Wire` | the preinstantiated `TwoWire Wire` for **TWI0 is deleted** — nothing is on that bus, but upstream passes the whole driver as function pointers so `--gc-sections` cannot drop it. Worth 1636 bytes including a 600-byte ISR. `utility/twi.c` is deliberately **absent** |

⛔ **If anything is ever fitted to TWI0**, restore the preinstantiation *and* copy
`utility/twi.c` back. It fails at link, loudly, rather than misbehaving.

## 5.4 Log format

One decimated sample line, plus event lines. Full legend in
`Eng_Notes/falcon_reference_2026-08-12.md` §8.

```
t= a= avg= st= rd= ov= tk= im= w= cv= x= y= m= q= pk= [et=] [tw=]
```

The fields that carry decisions:

| field | meaning |
|---|---|
| `pk=` | **windowed raw arrival peak — what the release actually decides on.** `pk=0.00` during a real excursion means the detector was never armed |
| `st=` | FSM state (§4.1) |
| `ov=` | samples lost to ring overrun (~20%) |
| `tw=` | TWI wedges caught and recovered. Printed only when nonzero |
| `rd=` | I2C read duration, µs (~11 400) |

Event lines: `BURST` (80 full-rate samples), `JOGV` (jog verdict), `RAMP latched`,
`ARM q= a= v= g= ro=` (per-run arming summary — `g`/`ro` are the ramp's own gate),
`FSM: Arrival (…)`, `FSM: FAILSAFE`, `Reset cause:`.

## 5.5 Tooling

| tool | purpose |
|---|---|
| `graph/parse_falcon_log.py` | per-run summaries, arming / late-release / multi-boot checks |
| `graph/arming_replay.py` | **replays candidate arming rules against all 186 bursts**, mirroring the ISR arithmetic. `--sweep` sweeps `ARM_REV_SAMPLES`. **Run after any change to arming or the ramp constants** |
| `graph/velocity_replay.py` | velocity-path replay |

## 5.6 Conventions

1. **Every new release path ships UNARMED on its first hoistway exposure**, and
   logs what it would have done. This turned one session into a data point rather
   than an incident.
2. **Thresholds carry their reasoning in a comment at the `#define`**, including
   changes tried and reverted. When a value moves, the comment moves with it —
   a stale comment beside a live threshold is worse than no comment.
3. **Rollback is named in the source** for anything armed.
4. **Engineering notes are dated and superseded, never silently edited.** The
   reference document and this one are the exceptions and are kept current.
5. **Single-run numbers are hypotheses** until a second run under different
   conditions agrees. See §8 of the state-of-project note for why this is stated
   so firmly.

---

# 6. Testing procedures

## 6.1 Bench workflow

**Power order matters.** The USB-serial cable back-powers the board through the
RX ESD clamp — it will run with no batteries fitted.

1. Power the device (batteries) **first**.
2. Connect serial **second**.
3. Disconnect serial **before** any battery swap. Pulling cells with the cable
   attached is **not** a power cycle.
4. **The first two `Voltage value` lines report `(settling, ignored)`.** Battery
   telemetry is sampled every 30 s on a wall clock, so the low-battery alarm can
   arm about 90 s after boot. ⛔ The ~23-minute convergence this step used to
   warn about was a defect, not a property: the cadence was counted in loop
   passes. Fixed 2026-08-14.

Roles: the engineer drives the car and reports run start/end; flashing, capture
and analysis are driven from the development machine.

## 6.2 Phase 0 — bench

- Boot, calibration, ready signal (1 chirp; 3 means it fell back).
- Parked soak: **zero false departures** over ≥15 minutes.
- Confirm sampling: `rd=` ~11 400 µs, `tk=` advancing, `ov=` not runaway.
- Deliberate shake → departure latch → jog verdict releases ~4 s later.

## 6.3 Phase 1 — cartop, instrumented

**Run any new release path UNARMED first** (§5.6.1).

Runs to capture: slowest available speed both directions; a multi-floor run both
directions; a single-floor run at a **terminal** (this is the arming blind spot,
§4.4); deliberate jogs of <1 s, ~1 s, ~2 s, ~3 s.

**Pass marks**

| check | pass |
|---|---|
| departure latched every run | required — failure mode 1 |
| arrival released every run, no failsafe | required |
| `FSM: FAILSAFE` | must not appear |
| jog verdict | `RUN` on real departures, `JOG` on jogs, no crossover |
| `pk=` during cruise | well clear of 0.45 |
| `ARM q=` | ≥ `ARRIVAL_ARM_SAMPLES`, with margin recorded |
| `tw=` | note any nonzero — a wedge was caught |

**Failure signatures**

| symptom | meaning |
|---|---|
| `pk=0.00` through a real excursion | the detector was never armed — §4.4 blind spot |
| beacon silent during a run | **stop testing.** Failure mode 1 |
| `Reset cause: 0x8` mid-run | watchdog caught a lockup; the beacon dropped |
| log stops, device needs external reset | a hang — check `tw=` before assuming power |
| release seconds after the latch | a departure ramp read as an arrival — §4.4 |

## 6.4 Phase 2 — counterweight, audible only

No cable, no log. The mechanic's actual experience: can the beacon be located by
ear and eye, at range, over machine-room noise? Does it stop when the
counterweight stops?

## 6.5 Automatic operation

Multi-floor runs both directions at service speed, plus single-floor runs at a
terminal. Record the arrival peak on every stop — this is the 0.454–0.713
distribution (§4.3) and it is the number that most needs more samples.

## 6.6 Endurance and power

⛔ **Bench/test build — it bypasses the FSM and NOTHING can release the beacon.
Never flash it for a car run.**

```bash
cd falcon_srcs && pio run -e brownout_test -t upload
cd falcon_srcs && pio device monitor -b 62500
```

Forces a continuous alarm 15 s after boot and prints one `BRN` line per second:
elapsed seconds, VCC in the **blast** phase, VCC in the **quiet** phase, `tw=`,
`ov=`. VCC is measured against the internal 1.1 V bandgap, which — unlike the
shipping battery read — is not ratiometric with AVCC.

| observation | verdict |
|---|---|
| `vcc_blast` well below `vcc_quiet`, both falling | rail sag is real |
| both flat and equal, then a hang | TWI wedge |
| survives with `tw=` climbing | wedge, being caught |
| `Reset cause: 0x8` mid-alarm | wedge, watchdog recovered it |

`ov=` climbs monotonically in this build (the ring is never drained) — treat it
as a **liveness** indicator, not a metric. Result to date: **588 s, no sag, no
reset, no wedge** — but **stationary only**, which leaves a
vibration-sensitive-contact hypothesis open.

## 6.7 Replay-based threshold tuning

Most thresholds can be moved without touching a car — the device already dumps
full-rate `BURST` evidence. Procedure, and the two mistakes that silently
invalidate it, are in `Eng_Notes/falcon_reference_2026-08-12.md` §8a. In short:

1. Mirror the ISR arithmetic **from the source**, not from a note.
2. Establish **where the run starts inside the burst** — this is the step that
   silently invalidates everything.
3. Test the **safety direction first** (does the rule fire on a *departure*?).
4. Sweep the constant and read the whole curve.
5. **Sanity-check against live behaviour.** A replay result that contradicts what
   the device demonstrably does is wrong until proven otherwise. This is the
   highest-value step.
6. State what the method cannot answer — bursts do not span whole runs.

---

# 7. Status

Current assessment, what is proven, and what is not:
**`Eng_Notes/falcon_state_of_project_2026-08-13.md`**.

Summary: inspection operation works and departure detection is the
best-evidenced part of the device. Automatic operation releases on a distribution
centred on its own threshold. Two of three armed release paths have never been
observed working, one reproducible position lie remains unfixed, and arming
margin depends on how the device was placed. **Not production-ready**; §6 of that
note records the reasoning and the disagreement.
