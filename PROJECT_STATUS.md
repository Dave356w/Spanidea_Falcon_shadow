# Falcon Shadow — Project Status Report

**Prepared for:** Project Management
**Date:** 11 August 2026
**Scope:** `dave356w/spanidea_falcon_shadow` @ `9bccb85` (branch `main`), directory `falcon_srcs/`

---

## 1. Headline

The repository holds a **working-in-progress firmware drop, not a release candidate.** The documentation (`Release.txt`) describes a finished V1.2 product — motion alarm, battery health service, pressure-based redundancy — but the code as committed has the **motion-detection state machine commented out of the main loop**. As a consequence, the alarm, the chase LEDs, and the battery-health service are all unreachable in a build made from this commit. What the current firmware actually does is initialise, then print raw accelerometer and temperature values to the serial port.

That is consistent with a **sensor-migration branch caught mid-flight** — the team appears to be moving from the analog MCP3208 accelerometer path to a digital BMA456 part — but it means the repo does not currently represent a shippable device, and nobody looking only at `Release.txt` would know that.

**Recommended read of the status: engineering is mid-rework on the sensing layer; treat V1.2 as "delivered previously on hardware", not "reproducible from this repo today".**

---

## 2. What the product is

An ATmega328P (Arduino framework, PlatformIO) firmware for a battery-powered motion/impact alert device:

| Subsystem | Hardware | Purpose |
|---|---|---|
| Motion sensing | MCP3208 12-bit SPI ADC reading analog accelerometer axes; **plus** a Seeed BMA456 digital accelerometer over I²C | Detect that the unit has started moving, is moving, is decelerating, has stopped |
| Barometric cross-check | Adafruit DPS310 pressure sensor | Second, redundant signal to confirm the unit has come to rest |
| Alerting | Piezo buzzer, red LED, shift-register "chase" LED ring | Audible + visual alarm while motion is in progress |
| Power | Battery divider on ADC channel 1 | Green blink above 3.7 V, red + buzzer between 3.4 V and 3.7 V, solid alarm below |
| Diagnostics | 115200 baud UART log, plus `graph/plot.py` (pandas + matplotlib) | Field log capture and offline plotting of the motion traces |

The design is sound and the state machine is well commented. The concerns below are about **execution state and process**, not about the concept.

---

## 3. Repository and delivery state

| Indicator | Finding |
|---|---|
| Commit history | **2 commits total.** `152e35b` "first cut of sources" (Sep 2025), `9bccb85` "Removed unwanted object files" (Jan 2026). |
| Authorship | Single author (`biju-iontra`). No review history, no PRs, no issues in the repo. |
| Elapsed time | ~4 months between the two commits, ~7 months since the last commit of any kind. |
| Build system | PlatformIO, `env:ATmega328P`, three external library dependencies pinned in `platformio.ini`. Reproducible. |
| Build artifacts | `.pio/` is **still committed** — 80 tracked files including `firmware.elf`, `firmware.hex`, object files and the full vendored library tree. The Jan 2026 cleanup commit did not finish the job. There is **no `.gitignore` anywhere in the repo.** |
| Tests | **None.** `test/` contains only the stock PlatformIO placeholder README. |
| CI | **None.** No `.github/` directory; nothing builds automatically. |
| Dead files | `src/movement_service.cpp.original` is committed alongside the live file. |
| Documentation | `Readme.md` is a good, practical flash-and-log procedure for a technician. There is **no architecture or theory-of-operation document**, and no record of hardware bring-up test results. |

**Practical consequence:** history offers no traceability. There is no way to tell from the repo which commit produced the firmware that was demonstrated, what was tested, or what changed between V1.0, V1.1 and V1.2 beyond the prose in `Release.txt`.

---

## 4. Functional gaps in the committed firmware

These are verifiable by reading the code at HEAD. They are ordered by impact.

### 4.1 The motion state machine never runs — **critical**

`src/main.cpp:134` — inside the nominal-operation branch of `loop()`:

```cpp
case SystemStates::SYSTEM_STATE_NOMINAL:
//        ms.run();
```

`ms.run()` is the only call site of the movement service. With it commented out, `MovementService::state` remains at its constructed value, `ERROR_RESET`, forever. Nothing ever calls `enable_alarm()`, so `alarm_service()` returns immediately on every pass and the buzzer, red LED and chase ring never activate.

### 4.2 Battery-health service is unreachable — **critical, and non-obvious**

`check_for_battery_voltage()` opens with a guard (`src/main.cpp:374`):

```cpp
if (ms.state != MotionStates::NOT_MOVING || alarm_status_g == 1) {
    return;
}
```

Because 4.1 pins the state at `ERROR_RESET`, this guard returns on every call. The low-battery warning — a V1.2 headline feature and a safety-relevant one — **never executes in a build from this commit**, even though the function is still called from the loop. This one is worth flagging to whoever signed off on V1.2.

### 4.3 Pressure sensing is compiled out, and would misbehave if the state machine were switched back on — **high**

The body of `read_pressure()` is wrapped in `#if 0` (`src/main.cpp:345`), so `pressure_avg_g` is never populated and stays at its zero-fill. The DPS310 is still initialised, and the `while (1) yield()` hardware-failure trap is also commented out, so a missing sensor now boots silently.

The latent defect: `MovementService::isAtRestOrStable()` computes the variance of that all-zero pressure buffer, gets exactly `0.0`, compares it against a `< 0.006` threshold, and increments `pressure_varience_counter` on every 100 ms tick. Five ticks satisfy the "device has come to rest" test. **If `ms.run()` is simply uncommented, the unit will declare itself stopped and silence the alarm roughly 500 ms into any deceleration, regardless of what is actually happening.** Re-enabling 4.1 without addressing this will look like an intermittent alarm-cutout bug on the bench.

### 4.4 Timer 1 configuration looks wrong — **high, needs bench confirmation**

`enable_timer1()` (`src/main.cpp:192`) sets `WGM13` alone and enables the compare-match-A interrupt with `OCR1A = 156`. Setting `WGM13` without `WGM12` selects phase-and-frequency-correct PWM with `ICR1` as TOP; `ICR1` is never written, so TOP is 0. In that mode the counter should not reach 156, which would mean `ISR(TIMER1_COMPA_vect)` — the routine that samples acceleration and integrates velocity — **never fires**. CTC mode would require `WGM12`.

Separately, the intended period is ambiguous: `OCR1A = 156` at a /1024 prescale on 16 MHz is ~10 ms, while the velocity integration in the ISR uses a 0.02 s step, implying 20 ms. One of the two is wrong and the velocity trace would be off by 2× accordingly. I have not been able to run this on hardware, so I have flagged it rather than asserted it — but it should be checked before any further tuning of the motion thresholds, because every threshold in the system is calibrated against that integration.

### 4.5 Two accelerometer paths are live at once — **medium**

The ISR reads the analog MCP3208 path (`Z1`/`Z2` channels, 440 mV/g scaling), while `loop()` separately polls the BMA456 over I²C for the values it prints. The device carries both sensing chains with no single source of truth, and the BMA456 values feed nothing but the log. This is the clearest evidence of the migration being unfinished; it also costs flash, RAM and loop time on a part that has little of any.

### 4.6 Diagnostic logging and the analysis tooling have diverged — **medium**

`log_data()` used to emit a comma-separated row (raw counts, acceleration, adjusted acceleration, velocity, threshold, motion state, alarm state, variances, pressure, battery). That block is now `#if 0`-ed out and replaced with a human-readable print of the BMA456 values. `graph/plot.py` still expects the old CSV columns and will not parse the current output. **The team's own field-analysis workflow is broken against the current firmware.**

---

## 5. Documentation vs. code mismatches

| `Release.txt` / docs say | Code says |
|---|---|
| "Battery health service … threshold (3.2 V)" | `VOLTAGE_LOW` is 3400 mV, `VOLTAGE_THRESHOLD` 3700 mV |
| "Blue LED to show device initialization" | No blue LED pin exists in `src/common.h`; `PIN_RGB_BLUE` survives only in the desktop simulation header. Initialisation uses the green LED. |
| "Refactored Movement service and added redundant pressure checking that increases the reliability of movement detection" | The pressure path is compiled out (see 4.3) |
| Alarm comment: "buzzer and led on for 200 ms … off for 300 ms" | `BEEP_FLASH_TIME_MS` is 300 with a 5-step counter, giving a different cadence |

None of these is dangerous on its own. Together they mean the written record cannot be trusted as a description of the build, which is the same root problem as section 3.

---

## 6. The simulation harness has fallen out of sync — **medium**

`simulation/` is a desktop (pthread-based) replay harness that feeds recorded logs through the movement service — a genuinely good asset for tuning motion thresholds without hardware. It has since diverged from the firmware:

- The simulation still uses the older `next()` / `monitor()` state-machine structure; `src/` has been refactored to inline transitions with `reset_counters()` and `last_state` logging.
- Constants differ (`INIT_TIME_MS` 3000 vs 8000).
- `RollingAvg::add()` in the simulation omits the `calc_avg()` call the firmware version has, and the two use different casts in the average computation — so the simulation and the device do not compute the same average from the same input.

**The simulation no longer validates the shipping logic.** Given there are no unit tests either, this was the project's only automated check on the motion algorithm, and it has silently lapsed.

---

## 7. Risk register

| # | Risk | Severity | Note |
|---|---|---|---|
| R1 | Repo HEAD is not the demonstrated product; no tagged/traceable release exists | **Critical** | If the person holding the working binary leaves, or the hardware is reflashed, V1.2 behaviour may not be recoverable from source |
| R2 | Alarm and battery-warning paths are inert at HEAD (4.1, 4.2) | **Critical** | Safety-relevant features silently absent |
| R3 | Pressure-redundancy defect will surface as intermittent alarm cut-out when the state machine is re-enabled (4.3) | **High** | Costly to diagnose if hit blind on the bench |
| R4 | Timer/ISR configuration and sample-period ambiguity (4.4) | **High** | Invalidates threshold calibration if wrong |
| R5 | Single contributor, no reviews, no CI, no tests | **High** | Full bus-factor exposure; no regression safety net |
| R6 | Analysis tooling and simulation both out of sync (4.6, §6) | **Medium** | Slows every future tuning cycle |
| R7 | Build artifacts in git, no `.gitignore` | **Low** | Repo hygiene; risks stale binaries being mistaken for current |

---

## 8. Recommended next steps

**Immediate (this week)**

1. **Establish ground truth.** Ask engineering which binary was demonstrated as V1.2 and get the matching source committed and tagged. Until that exists, treat the repo as pre-release.
2. **Confirm the intent at HEAD** — is the BMA456 migration deliberate and in progress, or was `ms.run()` commented out for a one-off data-capture session and never restored? The answer changes the schedule materially.
3. Fix R2/R3 together, not separately: re-enabling `ms.run()` without first handling the zero-variance pressure path will produce a misleading bench result.

**Short term (2–4 weeks)**

4. Bench-verify the Timer 1 configuration and settle the 10 ms vs 20 ms sample period before any further threshold tuning.
5. Pick one accelerometer path and remove the other; the ATmega328P does not have the headroom to carry both.
6. Restore the CSV log format (or update `plot.py`) so field logs are analysable again.
7. Add a `.gitignore`, untrack `.pio/`, delete `movement_service.cpp.original`.
8. Add a PlatformIO CI build on push — a compile check alone would have caught most of the drift above.

**Medium term**

9. Re-sync `simulation/` with `src/` and run recorded logs through it as a regression gate on every algorithm change. This is the cheapest available substitute for hardware-in-the-loop testing.
10. Write a one-page theory of operation (state machine, thresholds and where each number came from, pin map) so that the calibration constants are not tribal knowledge.

---

## 9. Questions for the engineering team

1. Which commit or binary corresponds to the V1.2 described in `Release.txt`, and has it been through field testing?
2. Is the BMA456 a confirmed hardware change, or an evaluation? What is the decision date?
3. Was the pressure sensor deliberately disabled — for cost, for reliability, or temporarily during bring-up?
4. What are the acceptance criteria for motion detection (detection latency, false-positive rate), and against what has V1.x been measured?
5. Is there test or field-log data held outside this repository that should be brought in?

---

*Prepared from a static review of the repository at commit `9bccb85`. No hardware was available; findings marked "needs bench confirmation" are reasoned from the source and should be validated on a device.*
