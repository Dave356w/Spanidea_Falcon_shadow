# Arrival gate — session report, 2026-08-19

**Session:** D (arrival margin population), with an unplanned partial of E.
**Build:** `0cacf29` — `26e38df` plus B2/B4 instrumentation, flash 32194/32256.
**Site:** the original traction installation. Device on the cartop for the
inspection runs, in the car for the contract-speed runs.
**Capture:** `falcon_srcs/logs/260819-121259-cartop.log` (gitignored).
**Tools:** `graph/session_d.py`, `graph/parse_falcon_log.py`.

---

## 1. What this session settled

**The arrival gate is wrong by about 2x, and the error is now measured rather
than inferred.** `ARRIVAL_PEAK_VALUE` is 0.45. The weakest arrival measured
today is 0.463 — the gate sits at **0.97x the weakest arrival**, i.e. already
below one of five measurements. Re-derived by the same method the original used,
it should be near **0.236**.

**The weak-arrival population is an automatic-operation phenomenon and cannot be
produced on inspection.** This is why the item has resisted characterisation:
the plan's method looked for it in a regime where it does not exist.

**The ramp detector's silence has a mechanism.** It is not failing to recognise
ramps. The reversal gate in front of it barely opens — 2 outright failures in 5
runs, and never more than one sample of margin.

---

## 2. The regime split

Session D's method asked for "the softest stops the machine can produce ... at
inspection speed". Per the site those are mutually exclusive: **soft stops are
only achievable on automatic operation at contract speed** — 150 fpm hydraulic,
300–500 fpm traction. On inspection the brake sets from motion, so the arrival
transient is set by brake mechanics rather than by the approach.

Six inspection runs at 19 fpm, deliberately including one stop made as soft as
the machine would allow:

| run | dir | cruise ceiling | arrival | vs gate | `ARM q/ro` | departed |
|---|---|---|---|---|---|---|
| 2 | down | 0.080 | 1.360 | 3.0x | 255/8 | any-motion |
| 3 | up | 0.070 | 3.730 | 8.3x | **6/11** | any-motion |
| 4 | up | 0.060 | 2.250 | 5.0x | 20/10 | **polled, silent** |
| 5 | down | 0.060 | 1.430 | 3.2x | 255/8 | any-motion |
| 6 | down | 0.070 | 2.410 | 5.4x | 255/7 | any-motion — **softest stop** |

**The deliberate soft stop produced a harder arrival than two ordinary ones.**
2.410 against 1.360 and 1.430. The whole inspection population sits at 3–8x the
gate, a factor of three above the 0.460 that makes this item urgent.

Then five runs on automatic at 500 fpm, full building travel:

| # | span | arrival | vs gate | cruise +5 s | separation | departed |
|---|---|---|---|---|---|---|
| 1 | 14.7 s | 0.470 | 1.04x | 0.100 | 4.7x | any-motion |
| 2 | 14.6 s | 0.484 | 1.08x | 0.080 | 6.0x | any-motion |
| 3 | 14.6 s | 0.700 | 1.56x | 0.120 | 5.8x | any-motion |
| 4 | 14.6 s | **0.463** | **1.03x** | 0.080 | 5.8x | **polled, silent** |
| 5 | 14.6 s | 0.609 | 1.35x | 0.090 | 6.8x | any-motion |

Run 4 was an unmarked repositioning move (1→8) needed to set up the next 8→1;
its direction is inferred from the trip sequence, not from a mark.

**Cruise is stable across both regimes at 0.060–0.120. The arrival collapses
from 1.360–3.730 to 0.463–0.700.** The gate's entire problem is on the arrival
side.

---

## 3. The gate

Sorted arrival population at contract speed: **0.463, 0.470, 0.484, 0.609,
0.700.** Three of five between 1.03x and 1.08x the gate.

Today's weakest is **0.463** against **0.460** on file. That is an independent
reproduction of the number the gate's derivation depends on, in the correct
regime, on a different day — the first time this project has reproduced it at
all.

| | original derivation | measured 2026-08-19 |
|---|---|---|
| worst cruise | 0.28 | **0.120** |
| weakest arrival | 0.713 | **0.463** |
| geometric middle | **0.45** ← shipped | **0.236** |

The original cruise figure is roughly 3x the measured one, and that error pushed
the gate up into the arrival population.

**Separation is not the problem.** 4.7–6.8x across all five runs, far clear of
the 1.6x threshold at which the plan says a single gate stops serving and a
second axis is required. A single gate still works. Its value is wrong.

⚠️ **Do not act on 0.236 yet.** Five samples, 51% spread, and runs 1 and 3 were
the *same trip* (8→1) producing 0.470 and 0.700. What matters for a gate is the
weak tail, and five samples locate a population's centre far better than its
tail. The plan's twenty-run minimum stands. What has changed is that the
derivation now rests on measurements from the regime where the failure lives.

---

## 4. Ramp detector — item 6 gets a mechanism

Zero `FSM: Arrival (ramp)` lines across five drive-controlled contract-speed
stops, both directions. That is the exact condition the plan says the detector
requires and it has now been given its best available test.

The reason is upstream of the detector. `ARM_REV_SAMPLES` is 8:

| run | `g` | `ro` | margin |
|---|---|---|---|
| 1 | 1 | 9 | +1 |
| 2 | 1 | 8 | 0 — opened on the last possible sample |
| 3 | 0 | 6 | **failed, −2** |
| 4 | 0 | 7 | **failed, −1** |
| 5 | 1 | 8 | 0 |

`g=0` means, in the source's own words, "the ramp detector never ran at all,
whatever the peak did". It never ran on two of five runs, and on the other three
its gate opened with at most one sample to spare.

So item 6 should no longer be read as "armed path with no positive evidence,
cause unknown". The cause is a marginal reversal gate. Session E's exit
criterion — disarm it if it fires on no automatic stop — is now closer to being
met, but the finding to act on is the gate, not the detector.

---

## 5. Item B3 confirmed twice, and it nearly cost the session

Two silent polled departures, one in each regime. `FSM: Departure latched
(any-motion)` sits inside `if (any_motion_pending)`, so a polled departure
enters `MOVEMENT_DETECTED` with **no line at all** — no latch, no burst, no jog
verdict.

Counts: inspection 3 latched / 4 transitions; contract speed 4 latched / 5
transitions.

**The contract-speed silent run is the one that produced 0.463 — the weakest
arrival of the day and the single most important measurement in this report.**
It was invisible to the analysis until the raw state transitions were
cross-checked by hand. Any run count or population taken from `Departure
latched` undercounts silently, and the omission is not random: it drops exactly
the runs the backstop caught.

`graph/parse_falcon_log.py` and `graph/session_d.py` now reconstruct these from
the state transition and flag them. That is a workaround in the analysis, not a
fix in the firmware.

---

## 6. Method notes for whoever runs this next

**Read `cp=` from a window opening at +8 s at inspection speed, +5 s at contract
speed.** `MIN_TRAVEL_MS` (3000) does not clear the departure ramp at 19 fpm —
see state doc §2.5b, open item 14. At 500 fpm it does: the cruise reading is
identical at +3 s and +5 s on every contract-speed run, so the ramp has decayed
by then. Item 14 is an inspection-speed problem specifically.

**The +8 s window cannot be used at contract speed.** Full building travel is
~14.6 s here, so a window of [+8 s, −8 s] is empty. Contract-speed cruise has to
be read at +5 s. This is a permanent constraint of the regime, not a property of
this building.

**A capture spanning several boots cannot be filtered on `t=`.** It restarts at
zero on every boot; this session's log had two boots overlapping across
`t=6946..~382000`. Run windows must be bounded by sample index first. The parser
does this now; anything reading the log by hand must too.

**Beware the same-trip spread.** 8→1 produced 0.470, 0.700 and 0.609 on three
separate runs. Do not treat a single run of a trip as characterising it.

---

## 7. Open items touched

| item | before | after |
|---|---|---|
| 2 arrival gate margin | mode sits on the threshold, unmeasurable | **both halves measured in the same regime.** Gate 0.97x the weakest arrival; re-derives to ~0.236 |
| 6 ramp detector unevidenced | armed, never observed working | **mechanism found** — reversal gate failed 2 of 5, never more than 1 sample margin |
| 13 calibration settle | 3 calibrations, 1 spurious learned value | 4th calibration clamped, `XY-Still` 0.0465, monotonic noise→threshold. 3 of 4 clamped |
| 14 `MIN_TRAVEL_MS` | new | **narrowed to inspection speed** — adequate at contract speed |
| gap 4 / B3 | polled path produces no burst | **produces nothing**, observed twice, one of them the session's key measurement |

---

## 8. What this session did not settle

- **The weak tail.** Five samples with 51% spread. Twenty runs minimum stands.
- **Whether 0.236 is right.** It is the correct arithmetic on today's numbers,
  not a validated constant. Any change must be replayed against the corpus
  before it goes in a car.
- **Hydraulic behaviour.** Everything here is one traction machine. Session F is
  untouched, and its exit criterion 2 now also needs the settle requirement
  added on 2026-08-19.
- **Whether the ramp gate can be fixed.** `ARM_REV_SAMPLES` 8 against measured
  6–9 is a tuning question that needs the corpus, not a car.
