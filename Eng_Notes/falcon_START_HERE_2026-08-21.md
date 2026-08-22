# START HERE — handover, end of 2026-08-21

Supersedes `falcon_START_HERE_2026-08-20.md`, which is still the authority on
everything before today — read its **§0 and §0a first**, they have not changed
and they are the proportion check for everything below.

**Today was a car day: 20 automatic runs at 350 fpm, a second machine, and the
first build that carries the lateral veto.** Nothing failed. The next session is
expected to be a cartop one, and §4 is written for it.

---

## 0. ⛔ WHAT HAS NOT CHANGED

**The beacon released on a moving car on 2026-08-20 and that installation is
still not to be relied on.** Nothing today was a fix. Today produced a
*candidate* with evidence behind it, a piece of instrumentation, and a build
that fits again — none of which is a shipped repair.

## 1. The device and the build

| | |
|---|---|
| On the device | **`67ca85b`**, `ATmega328PB` env, FALCON_LOG 2, flashed and verified 10:52 |
| Flash | **32156 / 32256 — 100 free.** `production` 31788 (468), `bench_battery` 31880 (376) |
| RAM | 1493 |
| Armed | ⛔ **`STOP_LATERAL_ARMED 0` — the §4.2 veto is DISARMED, see §6 item 5a**; `JOG_VERDICT_ARMED 1`, `RAMP_ARMED 1`, `VEL_ARMED 0` |
| New today | `JOG_RELEASE_POLLED 0` — see §3 |
| Mounting | counterweight, `XY_STILL` 0.0555, `Zero-Calib` 9.7455, `Threshold-Value` clamped at 0.040 |

⚠️ **78 bytes free is below this project's own rule that 80 is not headroom.**
Nothing else fits on the test build. `-mcall-prologues` is worth **228** and is
deliberately NOT enabled — it re-generates every function including the ISRs at
1 MHz. **The next change to this build needs it first, measured on its own.**

⭐ **Do not quote a free-space number from a note or from `platformio.ini`.**
Both were stale within a day on 08-21 and it cost a build.
`pio run -e <env>` takes three seconds on the bench machine.

Ports: **COM5 = log UART, COM7 = ISP.** Confirm with a signature read.

## 2. What today added

- **The build fits again.** `Falcon_Rel_EFT` did not build at the start of the
  day — three of five environments overflowed. 642 bytes came back out of the
  log plumbing with the log text unchanged. `falcon_flash_budget_2026-08-21.md`.
- **⭐ The mode question has a measured answer.** Integrating the departure
  burst gives Δv, and it separates automatic from inspection **2.1×**. A
  Δv-gated ramp-veto costs **5 NO-RELEASE in 208 runs instead of 96**, every
  inspection capture at zero, **and it prevents the confirmed 08-20 release on
  a moving car.** `graph/dv_replay.py`, and `falcon_350fpm_2026-08-21.md` §4.
- **B3 shipped**: the polled departure path emits a burst. §3.
- **First car numbers for §4.2's lateral veto**, and the first 350 fpm data on
  any recent build.

## 3. B3, and the one thing it still owes

The polled departure path now calls `burst_trigger()`, so a departure any-motion
does not see finally leaves a record of its own shape. **Before this, 2 of 14
runs at contract speed produced no `BURST k=dep` and no `JOGV` at all**, and
every rule built on the departure burst was blind on them.

⛔ **The jog RELEASE is withheld on those runs** (`JOG_RELEASE_POLLED 0`). A
burst means a verdict, and the verdict's thresholds were derived entirely from
any-motion-triggered bursts; it already has one measured error of exactly that
kind (1 false JOG in 63 labelled real departures). A polled-only burst PRINTS
its verdict and the line reads **`(obs)`** instead of `(armed)`.

**Verified on the car, 6 runs:** 6 departures → **6 bursts** (would have been 4
before), and both `(polled)` latches printed **`(armed)`** — meaning any-motion
did fire and the polled test merely won the evaluation order, so those runs keep
the release they have today. That is the discrimination the change turns on and
it is confirmed, not argued.

⬜ **STILL OWED: a `(obs)` line.** No departure since the flash has been
polled-*only*. It cannot be commanded — it needs a departure any-motion does not
see. **That is most likely at inspection speed, which is why the next cartop
session closes it.** Nothing else is needed from a car for B3.

---

# 4. ⭐ THE NEXT CARTOP SESSION — what to run and why

Read the boot block **before** the first run (§5 trap 1). Then, in rough
priority order:

## 4.1 Ordinary inspection runs, both directions, 15–25 fpm — as many as patience allows

This is the highest-value data in the session and it serves three open questions
at once.

1. **It closes B3.** Every inspection departure is a chance at a polled-only
   latch. Look for `JOGV … (obs)` in the capture; one is enough.
2. **It tests the Δv gate from its dangerous side.** The whole labelled
   inspection population is **26 runs, all on firmware from 08-11 to 08-20**.
   The gate's safety claim is that inspection never reaches 0.60 m/s — measured
   ceiling **0.452** labelled, **0.517** across every burst in the corpus. **An
   inspection run scoring above 0.60 is the finding that kills the rule**, so
   hunt for it: the fastest, most jolting inspection runs you are willing to do,
   and any run that ends in a hard stop.
3. **It gives `SL: rel mx=` its first inspection numbers.** Contrast at 350 fpm
   was 1.1×–4.3× against `xs`. If inspection contrast sits near 1.0× the lateral
   veto is inert across the whole inspection population — which is safe (it can
   only extend an alarm) but means §4.2 does **not** fence the jog verdict where
   item 9 actually lives.

Score with `python graph/dv_replay.py` after adding the capture to `MODE`.

## 4.2 Jogs — and specifically GENTLE ones

`datasets/README.md` names this the highest-value labelling work left: the
labelled set has **exactly one gentle jog** (`opk` 381), and it is the single
member of the population where the jog verdict's one observed miss lives.
Deliberate jogs of varying force, each called aloud so the label has a basis.

⚠️ Score them against **what the car did**, never against `verdict=` — that
field is `opk`-thresholded by definition and scoring against it is circular.

## 4.3 A terminal-landing run, at the bottom

08-21 found the bottom terminal behaves differently at contract speed: Δv 0.932
and 0.965 against 1.27–1.32 mid-shaft, ramp mean 523/509 against 595–611, and
lateral contrast 1.1–1.2×. **Three quantities moving together.** Whether that
reproduces at inspection speed is unknown and cheap to find out.

## 4.4 ⭐ A deliberate quick-reversal batch — measure window B on purpose

Added after the 08-21 wrap-up, from Dave's own observation: *move, stop,
reverse direction, and move again before the arrival has settled* — the new
run is not acknowledged while the old one is ending. Two windows, opposite
safety character:

- **Window A — reversal BEFORE the release.** Still in `STATE_DECELERATING`;
  the movement restarts the confirm window and the second run is subsumed in
  the first alarm. No new `Departure latched` prints, but **the beacon is ON
  while the car moves — the correct direction.** Accounting cost only. The
  §4.2 veto *widens* this window by delaying releases.
- **🔴 Window B — reversal AFTER the release, inside the re-arm blank, before
  the lateral settles.** Both live departure paths are blanked (polled too,
  since 08-20), and the rest-gate never fires because quiet never comes — the
  car is already moving. The blank runs to its 6000 ms cap; after it, cruise
  emits no fresh any-motion edges, so the run can stay invisible until the
  brake latches it — **item 11's shape**. The rest-gated fix's 0-misses-in-24
  was validated on properly spaced runs; Dave confirms his testing always
  spaced them. This is the residual exposure.

**The batch:** pairs of short runs with a reversal at varying gaps after
beacon-off — immediately, ~1 s, ~2 s, ~4 s — plus a few reversing *before*
beacon-off (window A). Call each aloud. Score: did the second departure
latch, on which path, and how long after true motion start.

⚠️ **The blank is load-bearing — do not conclude 'shorten it'.** It exists
because of the HYDRO jerk-at-rest: a down valve levels in smoothly, then a
jerk at rest re-latched a trip that had already ended (the 2026-08-07 guard).
A reversal-before-rest keeps the gate closed *by design*. Any fix must
distinguish jerk-at-rest from a genuine new departure by SHAPE or sign, not
by time — and that is a corpus question first: hydro captures and jog bursts
are on file to score a candidate against before any firmware moves.

## 4.5 Whatever it takes to make `Threshold-Value` exceed its floor

It has read **0.040000 — the clamp floor — on every mounting on file**, so the
self-calibrating polled threshold has never actually been demonstrated to learn
anything. A noisier cartop mounting is the likeliest place it finally does.
Pair any value above the floor with `XY-Still` and the `XY: bmax` spread to
show the window was quiet (see §5 trap 1).

---

# 5. Traps that cost time today — read before running

1. **🔴 S6 IS REAL AND IT BIT US.** The first calibration of the 08-21 session
   learned `XY_STILL` **0.1005** from a window with one bucket at 0.2110 against
   0.05–0.135 for the other five. `mv=` read 0. Nothing flagged it. A
   power-cycle gave 0.0570 — **43% lower**, and every lateral figure in the
   session would otherwise have been meaningless.
   ⭐ **Read `XY: bmax` in the boot block before the first run. If one bucket is
   much larger than the rest, power-cycle and let it settle.**
2. **A still-running capture is not all one session.** `260820-152131.log` had
   been running since 08-20 15:45 and by 08-21 contained bench taps as well as
   car runs. A hand tap latches and releases exactly like a car and scores as a
   release-on-a-moving-car. `graph/dv_replay.py` carries an explicit `CUT` for
   it. **Note the line number of the session's first boot before you start.**
3. **Never stop the capture until the session is over** (08-20 §7, still true).
   `capture.py` now survives a CP210x re-enumeration and appends across
   reconnects, so there is no reason to.
4. **Inserting a log field can silently blind a parser.** An `am=` field was
   written onto the JOGV line and removed: it would have sat between `opk=` and
   `verdict=`, and `session_g.py` matches `opk=(-?\d+)\s+verdict=`. One inserted
   field and every tool that reads jog verdicts stops seeing them, with no
   error. **Check the regexes in `graph/` before adding a field mid-line.**
5. **Distil by DROPPING the sample line, not by an allow-list.** An allow-list
   is how `RAMP latched` went missing from the whole corpus until 08-20. Check
   every event count against the raw capture before committing.

---

# 6. Open items, priority order

1. 🔴 **The arrival gate has no margin left.** 08-21 measured **0.451 against a
   0.45 gate — 1.002×**, beating the lifetime worst of 1.009×, and a second at
   1.01×. Six of fourteen runs inside 1.08×, both directions, second machine.
   Not fixable by tuning — raising the gate rejects real arrivals.
2. 🔴 **Silence while moving is still open.** The Δv-gated veto is the first
   candidate that prevents the confirmed event without the veto's cost, but it
   is **not shipped**, it is **blind where there is no burst**, and it is **not
   strictly safer** — one corpus run is `OK` under the shipping peak and
   `MOVING` under both vetoes. Resolve that before anything flies.
3. 🔴 **Any-motion misses departures it should see.** 2 of 14 at 350 fpm, and
   the transient on those runs measured the same size as on the ones it caught
   (0.631/0.634 against 0.600–0.640). The 25 Hz log cannot resolve the 100 Hz
   per-sample delta, so **the log cannot close this** — it is the 2026-08-07
   latched-reference hazard and it is still uninstrumented.
4. 🟠 **Arming rests on the minimum.** `q` hit exactly `ARRIVAL_ARM_SAMPLES` (5)
   on two of fourteen runs. The reversal backup (`ro` ≥ 15) was available on two.
5. 🟠 **`ARRIVAL_QUIET_MSS` 0.15 against measured cruise 0.27–0.29.**
5a. ⛔ **§4.2's lateral veto is DISARMED (`STOP_LATERAL_ARMED 0`), 2026-08-21
   pm, on Dave's call after measurement.** He reported from the car that the
   release felt slow. It is:

   | | median | p90 | max |
   |---|---|---|---|
   | pre-veto (54 runs, 08-20) | 9.6 s | 9.9 s | 11.4 s |
   | armed (22 runs, 08-21) | 10.9 s | **15.3 s** | **17.1 s** |

   A replay of the confirm window over the 25 stops on file puts the veto's
   own contribution at a median of **2.1 s** and a worst case of **12.4 s**.

   ⚠️ **The mechanism is not a slow settle.** The lateral reaches quiet almost
   at once and is knocked back to zero by isolated ticks a hair over
   `XY_STILL` — 0.054, 0.055, 0.067, 0.068, 0.084 against 0.0555 — and **each
   one restarts the full `STOP_CONFIRM_MS`**, while the car is stationary
   (vertical 0.027–0.031 inside a 0.10 band).

   ⛔ **Relaxing `STOP_LATERAL_QUIET` does not reach this** and was measured
   before being rejected: a break sets `quiet_run()` to 0, and 0 is below 4
   exactly as it is below 8. Replayed at 8 vs 4, the release moves by a
   **median of 0 ms**. The tight quantity is the AMPLITUDE, or the restart
   policy — not the run length. **Do not re-propose the run length.**

   ⭐ **The diagnostic survives disarming.** `SL: held … (obs)` still prints on
   every hold that would have happened, and `SL: rel mx= q=` still gives the
   per-run contrast — so the cost of re-arming stays measurable from ordinary
   captures. Re-arm is one constant, nothing else changes.

   ⛔ **What it gives up:** this was the only deployed mitigation against the
   2026-08-20 release on a moving car, and disarming returns that behaviour
   exactly. §0 was already in force and is unchanged.

5b. ⬜ **UNRESOLVED — a 9.7 s stretch in `STATE_MOVING` with the lateral fully
   quiet**, +14 to +25 s after run 1's departure latch on 08-21. Every
   indicator says the car was STATIONARY while the FSM stayed latched: `w`≈0,
   `pk` 0.02–0.03, vertical flat at 9.716–9.720. One of the session's two
   `ACC-STAT read FAILED` sits inside it. Excluded from the veto-grip analysis
   as not-travel, which is why it needs its own look — it is the position-lie
   direction, and it is the only stretch on file of that shape.
6. **Quiescent current has never been measured.** `idle_current` builds.
7. **Battery divider — three answers, no meter.**
8. **No written definition of production ready.** The only exit criteria in the
   repo belong to one test session, not to the product. Cheap, needs no car, and
   until it exists "close to production" and the open-items list cannot be
   reconciled by evidence.

# 7. Do not re-derive these

`falcon_fsm_logic_and_shortcomings_2026-08-21.md` §3 has the full refuted table.
Added today:

| approach | why it is dead |
|---|---|
| quoting free space from a committed table | stale within a day, twice; it broke the build |
| gating the jog release on `latch_path` | evaluation order, not detection — it would strip the release from runs that have it (3 of 5 polled latches on 08-21) |
| a mean over all 20 pre-trigger samples for the Δv zero | any-motion fires part-way up the ramp, so it subtracts the departure from itself: 2.1× → 1.03× |
| a mean rather than a median for that zero | one 3077 spike manufactured the corpus's worst false-automatic |
| "short hops are weaker" | wrong, and retracted the same day — it is the **terminal**, not the run length |

# 8. Where everything is

| | |
|---|---|
| session note | `falcon_350fpm_2026-08-21.md` |
| flash budget | `falcon_flash_budget_2026-08-21.md` |
| the FSM, spelled out | `falcon_fsm_logic_and_shortcomings_2026-08-21.md` |
| Δv tool | `falcon_srcs/graph/dv_replay.py` (`--veto ../logs` for the veto replay) |
| corpus | `falcon_srcs/datasets/` — **270 departure bursts**, 133 labelled |
| today's captures | `datasets/260821-101915.log` (14 runs), `260821-105230.log` (6 B3 runs) |
| ⚠️ raw logs | `falcon_srcs/logs/` is gitignored — bench machine only |
