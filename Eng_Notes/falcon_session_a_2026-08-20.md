# Session A — flash headroom, and a method error worth recording

**Bench, 2026-08-20.** Device on COM5 (CP210x, log UART) / COM7 (CH340, ISP).
Executes `falcon_test_plan_2026-08-18.md` §2. Commits `23465af` (RollingAvg)
and `b79298c` (FALCON_LOG).

---

## 1. Result

| build | flash | free | RAM | env |
|---|---|---|---|---|
| before | 32194 | **62** | 1364 | — |
| test, `FALCON_LOG` 2 | 31550 | **706** | 1492 | `ATmega328PB` |
| production, events + `HEALTH` | 31122 | **1134** | 1497 | `production` |
| silent | 25354 | **6902** | 1149 | `production_silent` |

Two of the four options taken, in the order §2 recommends. The driver swap was
not needed and was not attempted.

### 1.1 `RollingAvg` — 644 bytes, and the last heap on the device

Costed at 586 in the plan; measured 644. Depth is now a template parameter and
the array a fixed member, so `malloc`, `free` and `realloc` are **gone from the
image** (`avr-nm`). Both instances were file-scope globals with compile-time
constant depths, so the allocation happened once at static init and was never
freed — it bought nothing.

Reported RAM rises 1364 → 1492 because the arrays are now *counted*. Runtime use
is not worse: the same 144 bytes came off the heap before, plus allocator
bookkeeping, and were simply invisible to the size check.

**Unbudgeted gain: `rd=` fell 11464 → 10362 µs.** `calc_avg()` sums 32 floats on
every sample and the compiler addresses a fixed global directly instead of
through a heap pointer. That is 1.1 ms per sample back at 1 MHz — about 2.75% of
the 40 ms budget.

### 1.2 `FALCON_LOG` — 428 bytes at level 1, 6196 at level 0

All 251 `Serial.print` sites route through `FLOG` (`src/falcon_log.h`). Level 2
is **byte-identical in size** to the build it replaces, so the indirection costs
nothing. The per-sample line — 99% of the log by bytes — is compiled out below
level 2; everything above it in `emit_sample_log()` is detector input
(`vel_window`, `lat_monitor`) and is never compiled out.

Level 1 keeps every event line and adds a `HEALTH` line every 30 s carrying
`n= ov= er= st=` and `tw=`. Without it a production unit has no window on
itself, and the two numbers the verification clause needs are exactly the two
the per-sample line was carrying.

## 2. Verification clause — answered, in the negative

§2's clause: removing the blocking print removes the sole cause of ring
overrun, so a quieter build may feed its detectors **more** samples than the
population every threshold on file was measured against.

Measured, one rig, one mounting, matched windows:

| build | samples | window | rate | `ov` |
|---|---|---|---|---|
| reference 32194 | 1992 raw | 79.67 s | **25.00 Hz** | flat at 39 |
| level 2, 31550 | 1984 raw | 79.36 s | **25.00 Hz** | flat at 38 |
| level 1, 31122 | 751 × 3 | 30.03 s each | **25.01 Hz** | flat at 37 |

**No difference.** At `LOG_DECIMATE_N` 8 the logging build is not throttling the
consumer measurably, so a quieter build does not move the sample population and
no threshold on file is invalidated by shipping one.

⚠️ **This is not a licence to lower `LOG_DECIMATE_N`.** At N=2 the device loses
~35% of samples and boot-loops under the watchdog — refuted on the bench
2026-08-19, test plan §1.2. The two results are consistent: at N=8 the print
duty is low enough not to matter, at N=2 it is not.

⚠️ `ov` is flat in **all** builds after boot, including the reference. The
"~20% of snapshots are lost to overrun" figure in `main.cpp` is not what this
rig shows — every overrun observed occurred during boot and calibration, none in
steady-state monitoring. Not investigated; flagged because several comments rest
on it.

## 3. ☠️ A regression I reported and then had to withdraw

**Claimed:** the `RollingAvg` build latches a false departure ~13.8 s after boot,
4 boots out of 4, against 0 of 4 on the reference. Deterministic to the
millisecond.

**Withdrawn.** Re-tested with the two builds interleaved in the same window,
each ran **3 of 3 clean**. The original arms were separated in time — all four
reference boots, then all four new-build boots — and something in the bench
environment drifted between them. The re-arm settle time moved 2573 → 2589 →
2605 ms across the afternoon on builds that were byte-identical, which is the
drift showing up directly.

The observation was real: four consecutive boots latched a departure on a
stationary device on a bench nobody was touching, which is item 9's signature.
It is the *attribution* that was wrong, and it was wrong because of a test
design this project's own notes warn about repeatedly — "a single run is a
hypothesis until a second one under the same conditions agrees", and "flag a
weak test BEFORE running it".

**Standing rule for bench A/B from here: interleave the arms.** Flash A, boot,
flash B, boot, repeat — never all of A then all of B. Every timing-marginal
behaviour in this project has an environment term, and a time-separated A/B
cannot distinguish the two.

One real difference does survive the interleaved test: the re-arm blank settles
**16 ms later** on the `RollingAvg` build (2589 vs 2573), reproducible per build
across boots. That is the faster sample path showing up, and it is the size of
shift that decides which side of a boundary a boot-time edge lands on.

## 4. Left open

- ⬜ **The boot-time latch itself.** Not reproducing at the end of the session.
  Eight captures from the afternoon are on the bench machine. A deterministic
  any-motion edge arrives at t≈13.8 s on every boot of every build, ~150 ms
  after `STATE_MONITORING` is entered, and is normally discarded by the re-arm
  blank. The margin is small enough that it has been seen to fail.
- ⬜ **`VEL: departure w=0.413 cov=1803 obs`** printed once at boot on the
  production build, at the same t≈13.5 s, without latching. The velocity path is
  disabled (`VEL_ARMED 0`) so it cost nothing, but it is the same edge.
- ⬜ **Battery divider.** `Voltage value : 2415 avg : 2415 pack_mv : 4830` — a
  ×2 factor. `VBATT_CONST` is 4.4 (5.1k/1.5k) and the note on file says
  `read_adc_pc2_voltage()` applies neither. Neither matches. Still needs a
  meter, per the standing item.
- ⬜ **Quiescent current.** Level 0 is expected to recover ~1.10 mA and that was
  not measured — it needs the `idle_current` env, which does not currently link
  (32490 bytes, over by 234). It would now fit at level 0; not attempted.

## 5. Method notes for the next bench session

- **COM7 is the ISP, COM5 is the log UART.** Confirmed by signature read exactly
  as `platformio.ini` prescribes: `avrdude -c stk500 -P COM7 -p m328pb -U
  signature:r:-:h` → `0x1e,0x95,0x16`. The same read resets the target, which is
  how every capture in this session was started from a known boot.
- The three `pio device monitor` alternatives all fight the capture; a 20-line
  pyserial reader writing straight to a file was what made A/B possible at all.
