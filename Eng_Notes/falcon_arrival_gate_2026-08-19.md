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
today is 0.463 — the gate sits at **0.97x the weakest arrival**, i.e. just 3%
below the weakest of nine measurements, with no margin to speak of. Re-derived
by the same method the original used, it should be near **0.236**.

**The weak-arrival population is an automatic-operation phenomenon and cannot be
produced on inspection.** This is why the item has resisted characterisation:
the plan's method looked for it in a regime where it does not exist.

**The ramp detector works, and session E's exit criterion is met.** It latched
on 9 of 9 drive-controlled contract-speed stops and 0 of 8 inspection stops —
exactly the discrimination that session asks for. It has never been *reported* as
the arrival path only because the polled path wins the race.

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

Then nine runs on automatic at 500 fpm, full building travel:

| # | arrival | vs gate | cruise +5 s | separation | departed |
|---|---|---|---|---|---|
| 1 | 0.470 | 1.04x | 0.100 | 4.7x | any-motion |
| 2 | 0.484 | 1.08x | 0.080 | 6.0x | any-motion |
| 3 | 0.700 | 1.56x | 0.120 | 5.8x | any-motion |
| 4 | **0.463** | **1.03x** | 0.080 | 5.8x | **polled, silent** |
| 5 | 0.609 | 1.35x | 0.090 | 6.8x | any-motion |
| 6 | 0.670 | 1.49x | 0.080 | 8.4x | any-motion |
| 7 | 0.511 | 1.14x | — | — | any-motion |
| 8 | 0.508 | 1.13x | — | — | any-motion |
| 9 | 0.475 | 1.06x | — | — | any-motion |

Runs 7–9 were short enough that even the +5 s cruise window is empty.

Run 4 was an unmarked repositioning move (1→8) needed to set up the next 8→1;
its direction is inferred from the trip sequence, not from a mark.

**Cruise is stable across both regimes at 0.060–0.120. The arrival collapses
from 1.360–3.730 to 0.463–0.700.** The gate's entire problem is on the arrival
side.

---

## 3. The gate

Sorted arrival population at contract speed: **0.463, 0.470, 0.475, 0.484,
0.508, 0.511, 0.609, 0.670, 0.700.** n=9, median 0.508, **six of nine between
1.03x and 1.14x the gate**, and **none below it**.

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

**Separation is not the problem.** 4.7–8.4x across the six runs long enough to
yield a cruise window, far clear of
the 1.6x threshold at which the plan says a single gate stops serving and a
second axis is required. A single gate still works. Its value is wrong.

⚠️ **Do not act on 0.236 yet.** Nine samples, 51% spread, and runs 1 and 3 were
the *same trip* (8→1) producing 0.470 and 0.700. What matters for a gate is the
weak tail, and nine samples locate a population's centre far better than its
tail. The plan's twenty-run minimum stands. What has changed is that the
derivation now rests on measurements from the regime where the failure lives.

---

## 4. Ramp detector — item 6 resolved, positively

⚠️ **This section replaces an earlier version of itself that drew the opposite
conclusion. The first pass counted `FSM: Arrival (ramp)` lines, found zero, and
concluded the detector never runs because its reversal gate is marginal. That
was wrong, and it was wrong because the wrong string was counted.**

The detector emits `RAMP latched mean=<mmss> dir=<pct>`, not
`FSM: Arrival (ramp)`. Counting the right line:

| stops | `RAMP latched` |
|---|---|
| contract speed, automatic, 500 fpm | **9 of 9** |
| inspection, 19 fpm | **0 of 8** |

`dir=100` on every one — fully one-signed — with `mean` between 571 and 610
mm/s². That is session E's exit criterion met in full: fires on drive-controlled
stops, declines inspection stops, same session, both directions.

**Why it is never the reported arrival path.** The polled path triggers first
and the FSM has already moved on:

```
FSM: Arrival (polled), peak 0.470
ARM q=109 a=1 v=1 g=1 ro=9
FSM: Transitioned to STATE_DECELERATING
RAMP latched mean=591 dir=100
```

So `FSM: Arrival (ramp)` printing zero times is a statement about which detector
is faster, not about whether the ramp detector works.

**On `ARM_REV_SAMPLES` (8) against measured `ro` of 5–16.** Do not lower it on
this evidence. Three contract-speed runs showed `g=0` at ARM time (`ro` 6, 7, 7)
and **all three latched the ramp anyway** — `g` is a snapshot at the moment of
arming, not a verdict on the run, and the earlier reading of it as a failure was
too strong. Separately, `ARM_REV_SAMPLES` gates the PEAK collector as well as
the ramp accumulator, so lowering it loosens the arrival path that actually
releases the beacon. That is the false-release direction, and this project has a
recorded false release from exactly that kind of loosening on 2026-08-07. There
is no observed cost to leave it at 8 and no corpus evidence for moving it.

The open question is the inverse one, and it is a design question rather than a
tuning one: the ramp path has clean discrimination (9/9, 0/8) while the polled
path is the one whose threshold sits on its own population. Whether the polled
path should keep winning the race is worth asking.

## 5. Item B3 confirmed twice, and it nearly cost the session

**Three silent polled departures — two on inspection, one at contract speed.**
`FSM: Departure latched (any-motion)` sits inside `if (any_motion_pending)`, so
a polled departure enters `MOVEMENT_DETECTED` with **no line at all** — no
latch, no burst, no jog verdict.

Counts: inspection 6 latched / 8 transitions; contract speed 8 latched / 9
transitions. That is **18% of all runs this session invisible** to any analysis
keyed on the latch line.

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
| 6 ramp detector unevidenced | armed, never observed working | **RESOLVED positively** — `RAMP latched` on 9 of 9 automatic stops, 0 of 8 inspection stops, `dir=100` throughout. Session E's exit criterion met |
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
- **Whether the polled path should keep winning the race.** The ramp path has
  the cleaner discrimination; the polled path has the threshold problem. Which
  should decide an arrival is a design question, and it needs the corpus.
