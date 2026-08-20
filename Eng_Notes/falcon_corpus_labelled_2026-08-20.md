# Session G run — the labelled corpus, and the first honest score of the jog verdict

**Desk, 2026-08-20. No hardware, no car, no firmware change.** Executes the
procedure in `falcon_corpus_labelling_2026-08-20.md`. Artefacts:
`falcon_srcs/datasets/session_g_labels.csv` (the labels, one named basis per
row) and `falcon_srcs/datasets/session_g_sheet.csv` (the worksheet, regenerated
from it).

**Open items:** closes 4 as a measurement. The verdict is scored, not retuned,
and the case for retuning it is weaker after scoring than before. §4, §6.

---

## 1. Result

| | count | of 273 |
|---|---|---|
| `run` | 66 | 24.2% |
| `jog` | 17 | 6.2% |
| `disturbance` | 4 | 1.5% |
| **labelled with named evidence** | **87** | **31.9%** |
| `unknown` — examined, notes do not pin it | 19 | 7.0% |
| no label row — no note names the capture at all | 167 | 61.2% |

**84 of the 87 carry a departure burst** and are scoreable for a shape rule
(63 run, 17 jog, 4 disturbance); three labelled runs predate the burst
instrument.

Reproduce with:

```bash
cd falcon_srcs/graph && python session_g.py --labels ../datasets/session_g_labels.csv --worksheet ../datasets/session_g_sheet.csv
```

## 2. The procedure's working order is wrong, and its own §1 says why

`falcon_corpus_labelling_2026-08-20.md` §4 says to work the three captures
holding 131 of the 211 bursts first, because they are 62% of everything
scoreable. **Burst count does not predict labellability.** The binding
constraint — which the same note identifies correctly in §1 — is notes
coverage, and only **10 of the 56 captures are named in any session note**:

| capture | deps | labelled | source |
|---|---|---|---|
| `260818-105317.log` | **82** | **10** | `falcon_departure_detection_2026-08-18.md` §3.3/§3.4, state doc §6 |
| `260812-112254.log` | 35 | 26 | `falcon_cab_automatic_2026-08-12.md` |
| `260812-140913.log` | 17 | 10 | `falcon_ramp_armed_2026-08-12.md` |
| `260813-110300.log` | 12 | 1 | `falcon_500fpm_ui_2026-08-13.md` §5 |
| `260811-144408.log` | 12 | 12 | `falcon_jog_verdict_2026-08-11.md` §1/§6.1 |
| `260812-104647.log` | 9 | 8 | `falcon_cartop_2026-08-12.md` |
| `260811-154018.log` | 5 | 5 | `falcon_jog_verdict_2026-08-11.md` §7 |
| `260811-152209.log` | 4 | 3 | `falcon_jog_verdict_2026-08-11.md` §6.1 |
| `260811-134652.log` + `-140425.log` | 5 | 5 | `falcon_350fpm_automatic_2026-08-11.md` |
| `260812-095101.log` | 3 | 3 | `falcon_cartop_2026-08-12.md` |
| `260813-103846.log` | 3 | 2 | `falcon_500fpm_ui_2026-08-13.md` §5 |
| `260810-130833.log` | 2 | 2 | `falcon_signature_2026-08-11.md` §1 |

**The largest capture is the least labellable.** `260818-105317.log` is 30% of
the corpus by departures and yields 10 labels, because the 2026-08-18 note
reports its runs as **counts** — "12 runs, 6 missed", "14 runs, 0 missed" — and
a count cannot be attached to a row. The small 08-11 and 08-12 logs labelled at
90–100% because those notes carry per-run tables with `opk`, ratio, `q=` and
arrival peak in them.

**Concrete lesson for note-writing: a table of per-run measured values stays
labellable indefinitely; a sentence containing a count does not.** It costs
nothing at the time and it is the difference between this session yielding 87
labels and yielding 20.

## 3. Method, and two corpus corrections

No label came from `JOGV verdict=`, `end_path` or `span_ms`. Every basis is a
specific note table row or sentence, recorded per record. Three matching routes,
in descending strength:

1. **Exact field match.** The 08-11 jog note tabulates `pos`/`neg`/`ratio`/`opk`
   per event; those quadruples appear verbatim in the `JOGV` lines and match
   1:1, in order. 20 labels.
2. **Measured-value match.** Per-run tables carrying arrival peak and `q=`.
   Extracting `ARM q=` per departure resolves `260812-112254.log` almost
   completely against §12/§15/§16/§22 — the six-run margin table matches on
   `q=` *and* peak simultaneously. 57 labels.
3. **Statistic fingerprint.** The 08-18 note's §3.3 gives block mean and
   directionality for six deliberately-labelled events; recomputing
   `best_block()` over all 204 departure bursts matches each to **exactly one**
   burst, and the hand bump's `opk=118` corroborates independently from
   `jog_window_replay.py`'s header. Three further records are pinned by unique
   `opk` values quoted in prose (972, 1459, 2974). 10 labels.

Two disagreements between the corpus and the notes:

- **`260811-152209.log` has four `JOGV` lines, not three.** The jog note's §6.1
  table stops at row 15, so its "15/15 lifetime" and the 20/20 that follows omit
  `pos=11815 neg=0 ratio=0 opk=99` at log line 97. Labelled `unknown`. The
  lifetime tally is off by one in the harmless direction — an uncounted RUN
  verdict, not an uncounted misclassification.
- **204, not 211, departure bursts are replayable.** `session_g.py` counts 211
  `has_burst`; 204 are `BURST k=dep` and the other 7 are the pre-`k=` format,
  which `arming_replay.load_bursts` cannot type. Shape rules are scored on 204.

## 4. 🟢 The set is scoreable, which the procedure did not expect

`falcon_corpus_labelling_2026-08-20.md` §5 predicted the second branch —
"retrospective labelling is capped, collect at the car". The cap is real (§1),
but 63 labelled runs against 17 labelled jogs is a usable pair, and this is the
first time either rule has been scored against labels that do not come from the
rule itself.

### 4.1 The shipping gate, scored

`ratio >= 33 AND opk >= 900`:

| population | n | called JOG |
|---|---|---|
| `run` | 63 | **1** |
| `disturbance` | 4 | 0 |
| `jog` | 17 | 16 |

- **The one false JOG is `260818-105317.log` r44, `opk=972 ratio=58`** — the
  silenced run the state doc §6 and test plan §13 already cite as refuting a
  gate raise. It has never been expressed as a rate before: **1 in 63.**
- **The one miss is `260818-105317.log` r75, `opk=381 ratio=88`** — the gentle
  jog of 08-18 §3.4.

Both errors are in the same capture, and both are the events that session went
looking for. Nothing in the other 62 records misclassifies.

### 4.2 ⚠️ The real-departure `opk` ceiling has moved every time data was added

| | `opk` | ratio | best block mean |
|---|---|---|---|
| `run`, n=63 | **11 – 972** (2nd highest **562**) | 0 – 58 | 45 – 618 |
| `jog`, n=17 | **381 – 4154** | 44 – 99 | 12 – 708 |
| `disturbance`, n=4 | 118 – 389 | 0 – 79 | 14 – 64 |

The 08-11 note put real departures at ≤440 against jogs ≥1366 and read the 900
gate as splitting a 2.05×/1.52× gap. On labelled data:

| when | evidence | run ceiling |
|---|---|---|
| 2026-08-11 | 9 real departures, one car, one day | 440 |
| 2026-08-12 | labelled automatic stops, this session | **562** |
| 2026-08-18 | the silenced run | **972** |

**That progression is itself the finding.** The state doc's structural claim —
"real departures throw jolts of arbitrary size" — is not an argument any more,
it is a measured trend across three independent additions of data. The
populations now **overlap on 381–972**, so the gap the gate is described as
splitting does not exist.

**Correction to carry back:** the state doc §4.2 records `JOG_OPP_PEAK_MMSS`'s
status as "real runs observed at 972–2974", and test plan §11 repeats it as
"real jolt-heavy departures reach 972–2974 and are silenced". **2974 is a
jog, not a run** — it is the upper endpoint of the 08-18 note §3.4's
*jog-clear* range 1459–2974, and both endpoints resolve to unique corpus
records (r48, r58), both labelled `jog` here. The only evidenced real run above
the gate is 972. As written, those two lines would justify raising the gate to
~3000 and silencing everything between.

### 4.3 The sweep — and why it does not license a retune

| `opk` gate | runs silenced | jogs caught |
|---|---|---|
| 300 | 6 | 17/17 |
| 381 | 4 | 17/17 |
| 400 | 3 | 16/17 |
| 563 | 1 | 16/17 |
| **900 (shipping)** | **1** | **16/17** |
| **973** | **0** | **16/17** |
| 1200 | 0 | 16/17 |
| 1400 | 0 | 15/17 |

Read naively this says raise the gate to ~1000: it removes the only observed
false JOG at zero cost in jog detection, and 973–1200 is flat. **Do not.** §4.2
is the reason — every time the corpus grew, the run ceiling grew with it, so a
gate fitted to the current maximum is fitted to a sample maximum, and the next
session moves it again. The refuted-table entry stands; this sweep quantifies it
rather than overturning it.

The other direction is closed outright: catching the 381 jog costs **four**
silenced runs, and 381 sits inside the disturbance population (118–389) as well.

### 4.4 Shape is refuted a third time, now against real labels

Best-block mean: runs 45–618 (median 504), jogs 12–708 (median 300).
**14 of 17 jogs fall inside the run range.** §3.2 of the 08-18 note refuted this
against 177 *unlabelled* bursts, resting on structure ("a jog's departure is a
real departure"); it now holds against labels too.

Disturbances sit low (14–64) but overlap the weak end of the run population
(45–618) — §3.3's finding reproduced: the floor that rejects a hand bump also
rejects a 20 fpm departure.

## 5. What is deliberately not concluded

- **The miss rate is not estimable from this set.** 16 of 17 labelled jogs were
  written into a note *because* they were verdict tests, and a jog the firmware
  called RUN is less likely to have been tabulated at all. The false-JOG figure
  (1/63) does not have this problem — runs were labelled from what the car did.
- **44 records carry a firmware JOG verdict; only 15 of them are labelled
  `jog`.** One is labelled `run` (the 972 false JOG), 2 are `unknown`, and 26
  have no label row at all — **21 of those in `260818-105317.log` alone**. If
  even a third of the 26 are genuine jogs, the labelled jog population is a
  small and non-random sample of what is already on file. (Two labelled jogs
  conversely carry no JOG verdict: `260811-144408.log` r2, captured under the
  old ratio-50 gate, and the 381 miss.)
- **Nothing is retuned**, per §4.3.

## 6. Recommendation

1. **Item 4 closes as a measurement.** Session J's premise — "the verdict
   misclassifies in both directions" — is confirmed at 1 false JOG in 63 and 1
   miss in 17, both in one capture, both already documented individually. What
   the labels add is that neither error is a tail of a distribution: they are
   the two ends of a genuine overlap, and no single-axis threshold removes
   either without creating the other.
2. **Fix the two doc lines in §4.2** before anyone acts on them: state doc §4.2
   `JOG_OPP_PEAK_MMSS` status, and test plan §11's "972–2974". Also carry the
   562 into the gate comment — its justification still cites a 440 ceiling.
3. **The collection pivot in the procedure's §6 still stands**, for a different
   reason than expected: not because labelling failed, but because the reachable
   population is bounded by what was written down on 2026-08-11 and 08-12, and
   the gentle-jog end — where the real defect lives — has exactly one member.
   An hour of called-aloud jogs at the car would more than double the jog
   population and is the only way to get the weak end of it.
4. **Note-writing change, free: per-run tables of measured values, not counts.**
   §2.

## 7. Tool changes

`graph/session_g.py`:

- **Windows fix.** The census aborted with `UnicodeEncodeError` on its first
  non-ASCII warning mark (cp1252 console) partway through the end-path table.
  `sys.stdout.reconfigure(encoding='utf-8')`, and the worksheet is written
  UTF-8 explicitly. The tool was validated in a Linux session and did not run on
  the bench machine as committed.
- **`--labels FILE`.** Merges labels into the emitted worksheet and prints the
  label census. The labels file is the durable artefact and the worksheet is
  regenerated from it, so re-running the census cannot lose a label. It flags
  label rows that match no record, and labels carrying no basis.
