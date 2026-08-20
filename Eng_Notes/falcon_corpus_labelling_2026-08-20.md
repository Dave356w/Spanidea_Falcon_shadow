# Session G — corpus labelling procedure

**Desk. No hardware, no car, no firmware.** Written 2026-08-20 to be run as
written. Supersedes the method paragraph in
`falcon_test_plan_2026-08-18.md` §8; the plan's *why* and *caution* still stand.

**Open items:** enables 4 (jog verdict); prerequisite for any future
discrimination rule.

---

## 0. Run the census first — five minutes, before committing the day

```
cd falcon_srcs/graph
./session_g.py                                   # census only
./session_g.py --worksheet session_g_sheet.csv   # census + blank worksheet
```

`session_g.py` is new and validated against the committed corpus. It finds
every departure, attaches the context needed to judge it, and reports what the
corpus cannot answer at all. **It does not label** — that is the human half and
the entire point of the session.

The census as measured on 2026-08-20 against `falcon_srcs/datasets/`:

| | count | |
|---|---|---|
| captures | 56 | |
| departures (`STATE_MOVING`) | 273 | |
| departure path: any-motion | 263 | 96.3% |
| departure path: polled, **silent (true B3)** | 3 | 1.1% — capture has latch lines, this departure had none |
| departure path: pre-latch build | 7 | 2.6% — capture has *no* latch line at all; predates the latched FSM (landed 08-07) |
| carrying a departure burst | 211 | 77.3% |
| **no burst at all** | **62** | **22.7% — unscoreable for any shape rule** |

Burst coverage is entirely a function of date, because the instrument landed on
2026-08-11:

| date | departures | bursts |
|---|---|---|
| 260806 / 260807 / 260810 | 54 | **0** |
| 260811 | 42 | 38 |
| 260812 | 64 | 63 |
| 260813 | 15 | 15 |
| 260818 | 98 | 95 |

> **The plan says 387 records. The measured figure is 273 departures and 211
> departure bursts** (416 BURST lines total, of which 203 are arrivals and 9
> are the pre-`k=` format). Reconcile or supersede the 387 — it is not
> reproducible from the committed corpus.

---

## 1. The structural finding, and why it decides the session

**Departure bursts fire on the any-motion path only.** In
`movement_service.cpp` the `burst_trigger(BURST_POST_DEP, 0)` call sits inside
`if (any_motion_pending)`. A departure the polled or velocity path caught
produces no burst.

That is not a logging omission like B3 — it is the instrument itself being
wired to one detector. Any rule scored on this corpus is validated for
**any-motion departures only**, and the runs the backstop caught are absent by
construction. Those are the marginal ones, which is where discrimination is
hardest and matters most.

### What cannot serve as a label

Three tempting sources, all circular or empty:

| source | why not |
|---|---|
| `JOGV verdict=` | `opk`-thresholded **by definition**. A rule scored against it separates perfectly and means nothing. Already flagged in the plan. |
| `end_path` | A jog produces an arrival transient too. And `jog-verdict-release` *is* the opk rule firing, so it is the same circularity wearing a different name. |
| `span_ms` | **Not a run duration.** See below. |

`span_ms` deserves the detail because it looks usable and is not. Measured over
this corpus:

| edges in run | n | median span |
|---|---|---|
| 1 | 44 | 0 ms |
| 2 | 95 | 1146 ms |
| 3–5 | 66 | 4915 ms |

`ACC-INT` is edge-triggered. A departure raises a cluster of edges; cruise is
quiet **by construction** — a car at constant velocity reads 1 g, which is the
physical constraint this whole project is built around. So in most runs both
edges sit at the departure and the span measures that cluster, not the trip. A
2-edge median of 1.1 s is not a 1.1 s ride.

### What that leaves

Nothing inside a capture says run vs jog. The only independent source is the
**dated session notes**, and they are per-session narrative rather than
per-record. So the binding constraint on this session is notes coverage, not
burst coverage — which is not what the plan assumed.

---

## 2. Where the discriminating information actually lives

Before labelling anything, read `falcon_jog_verdict_2026-08-11.md` §3.2. It
concluded, on measurement:

> A jog's departure *is* a real departure. The car accelerates identically.
> What distinguishes a jog is the reversal that follows.

So the departure **ramp** is not where a rule will be found, and a label
attached to ramp shape is wasted effort. The burst is 20 pre + 60 post samples
(`BURST_POST_DEP` = 60, so 2.4 s after the latch), and the reversal falls
inside those post samples. That is the window a label needs to be useful.

Label the **event**, not the waveform. "1 s jog", "slow departure down",
"handling bump" — what the car did, from the notes.

---

## 3. Start from the seed, do not start over

`falcon_jog_verdict_2026-08-11.md` §1 already carries a labelled table — 9
events with an `event` description alongside `pos`, `neg`, `ratio`, `opk`.
§6.1 and §7 carry more from the same evening.

Transcribe those into the worksheet first. They are the highest-confidence
labels on file, they are already reconciled against JOGV fields, and starting
from them means the day extends a corpus rather than founding one.

---

## 4. Working order

Three captures hold 131 of the 211 bursts — 62% of everything scoreable. Work
them first; if the notes do not support them, the rest will not rescue the
session.

| capture | departures | bursts | notes to read against |
|---|---|---|---|
| `260818-105317.log` | 82 | 80 | `falcon_departure_detection_2026-08-18.md` |
| `260812-112254.log` | 35 | 34 | `falcon_cab_automatic_2026-08-12.md`, `falcon_cartop_2026-08-12.md` |
| `260812-140913.log` | 17 | 17 | `falcon_ramp_armed_2026-08-12.md`, `falcon_reference_2026-08-12.md` |

Then the remainder, in the per-capture worklist `session_g.py` prints. Captures
with zero bursts (all of 08-06, 08-07, 08-10) need no attention for rule
scoring — skip them.

### Per record

Fill two columns in the worksheet:

- **`label`** — one of `run`, `jog`, `disturbance`, `unknown`.
- **`basis`** — the *specific* sentence, table row, or timestamp in the notes
  that supports it. Not "from notes". If you cannot name the evidence, the
  label is `unknown`.

**`unknown` is a valid answer and must not be guessed.** The count of unknowns
is a result of this session, not a failure of it.

---

## 5. Exit criterion

A labelled worksheet, committed, plus a count of records labelled with named
evidence — reported separately for `run`, `jog`, `disturbance`, `unknown`.

Then the decision the session exists to make:

- **If the confidently-labelled count supports scoring** (a usable jog
  population against a usable run population), score the four candidate rules
  against it and report each one's separation. That closes item 4.
- **If it does not**, the conclusion is the one §8's exit criterion already
  anticipated, now with a number behind it: retrospective labelling is capped,
  and future rules need labels collected **at the car**.

Given §1 above, expect the second. The census is designed to establish that in
the first five minutes rather than the last hour.

---

## 6. The pivot, specced now because it is free

If the session lands on "collect labels instead", the collection costs nothing
when someone is already in the car for session C or D:

1. **Say the event aloud into the log** before it happens — the operator calls
   "jog", "run up", "run down", "bump" and it goes in the capture as an
   annotation line, or into a paper sheet keyed by wall-clock time.
2. **Deliberately produce both populations.** Jogs are cheap to make on
   inspection: the 08-11 evening session produced a usable jog population in
   one evening. Real runs come free alongside whatever else the session is for.
3. **Keep the ratio honest.** The corpus is 16% firmware-called JOG against 55%
   RUN. A labelled set that is 50/50 by construction will flatter any rule
   scored on it; record the natural ratio separately.
4. **Capture the silent departures too.** True B3 silents are 1.1% of this
   corpus (a further 2.6% are pre-latch-build captures, not the same thing) and
   produce no burst at all, so a purpose-collected set should note when the
   latch line is absent — otherwise the new corpus inherits the same blind spot
   as the old one.

An hour of labelled collection is worth more than a day of retrospective
labelling, and this section exists so that conclusion does not have to be
re-derived.

---

## 7. What this session does not touch

Rule *design*. Four candidates exist and none is scoreable yet; inventing a
fifth before there is a labelled set to score it against repeats the mistake
this session is here to fix.
