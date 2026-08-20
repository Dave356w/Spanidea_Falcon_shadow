# Distilled event corpus

Event lines extracted from the raw serial captures in `../logs/`, which are
**gitignored** (`falcon_srcs/.gitignore:14`) and therefore exist only on whatever
machine recorded them.

**Why this directory exists.** Every replay result, every threshold on file and
every burst-level claim in `Eng_Notes/` is derived from those captures. Until
2026-08-18 none of that evidence was in the repository: the notes cited logs by
name that nobody else could retrieve, and the corpus lived in one place with no
backup. This is the evidence base, versioned.

## What is here

One file per capture, same basename minus the `device-monitor-` prefix. Only
event lines are kept:

    BURST   JOGV   ARM   FSM:   ACC-STAT   ACC-INT   XY:   XY-Still
    Reset cause   VEL:

**Dropped:** the periodic `t=... a=... avg=...` sample lines, which are ~99% of
the bytes and are already decimated to ~11% of the true 25 Hz stream, so they
support neither replay nor reconstruction.

    raw logs/     14 MB
    datasets/    480 KB    416 BURST lines -- identical count to the raw logs

Measured 2026-08-20 by `graph/session_g.py`: **56 captures, 273 departures,
416 BURST lines** -- 204 `k=dep`, 203 `k=arr`, 9 in the pre-`k=` format that
predates the kind marker. *(An earlier revision of this file said 387; that
figure is not reproducible and is superseded.)*

## Verified equivalent

`graph/jog_window_replay.py` produces byte-identical output against either
source:

```bash
python graph/jog_window_replay.py --logs logs
python graph/jog_window_replay.py --logs datasets
```

Both report the same figures. As of 2026-08-20 that is 204 departure bursts,
RUN n=159 opk (4, 859), JOG n=45 opk (972, 4154). Any tool that consumes
`BURST` lines can point here.

## 🟢 LABELS EXIST AS OF 2026-08-20 — `session_g_labels.csv`

**Nothing inside these files records whether a burst was a real run, a jog or a
knock, and that has not changed.** What changed is that the labels were
recovered from *outside* the corpus, from the dated session notes, and are now
versioned beside it:

| file | what it is |
|---|---|
| `session_g_labels.csv` | **the durable artefact.** `log,run,label,basis` — 106 rows: 87 labelled `run`/`jog`/`disturbance` with a named per-record basis, 19 explicit `unknown` |
| `session_g_sheet.csv` | the worksheet, **regenerated** from the labels — do not hand-edit it |

```bash
cd falcon_srcs/graph && python session_g.py --labels ../datasets/session_g_labels.csv --worksheet ../datasets/session_g_sheet.csv
```

Coverage is **87 of 273 departures (32%)**, and the limit is notes coverage,
not burst coverage: only 10 of the 56 captures are named in any session note,
and the largest capture (`260818-105317.log`, 82 departures) yielded 10 labels
because its note reports runs as counts rather than as records. Method, limits
and the resulting scores are in
`Eng_Notes/falcon_corpus_labelled_2026-08-20.md`.

⚠️ **The labelled jog population is 17 and is NOT random** — jogs were written
into notes *because* they were verdict tests. A false-JOG rate computed from
this set is meaningful (runs were labelled from what the car did); a miss rate
is not.

Every basis names a specific note table row or sentence. **If you add a label
and cannot name the evidence, the label is `unknown`** — that is a valid answer
and the count of unknowns is a result, not a failure.

⚠️ **Do not score a candidate rule against the shipping verdict.** The `JOGV`
`verdict=` field is `opk`-thresholded by definition, so measuring `opk` against
it is circular and will report a perfect separation that means nothing. This
mistake was made and caught on 2026-08-18; see
`Eng_Notes/falcon_state_of_project_2026-08-18.md` §6 ("Two method notes") and
`falcon_test_plan_2026-08-18.md` §8 Caution. *(This line used to cite
`falcon_jog_ringing_2026-08-18.md`, which does not exist under that name.)*

### The six events labelled aloud in session (2026-08-18, Dave)

These were the *whole* labelled set until 2026-08-20; they are now 6 of 87, and
all six are in `session_g_labels.csv` (`260818-105317.log` runs 67, 73, 76, 77,
78, 79 — matched by recomputing `best_block()` over every departure burst, each
to exactly one record).

| event | label | `opk` | block mean | dir% |
|---|---|---|---|---|
| walk on cartop #2 | **not a departure** | 299 | 14 | 58% |
| hand bump | **not a departure** | 118 | 33 | 75% |
| walk on cartop #1 | **not a departure** | 389 | 34 | 50% |
| 20 fpm down #2 | **departure** | 188 | **45** | **75%** |
| 20 fpm down #3 | **departure** | 170 | 160 | 92% |
| 20 fpm down #1 | **departure** | 149 | 199 | 92% |

🔴 **They already overlap.** A confirmed 20 fpm departure (45 / 75%) sits inside
the knock band (14–34, bump at 75%). That is what refuted the pre-filter
proposal, and it is why the honest conclusion of that session was that
false-alarm-vs-missed-departure is a **sensitivity trade-off, not a
discrimination problem** with these features.

**Confirmed at scale 2026-08-20:** across 84 labelled records carrying a burst,
14 of 17 labelled jogs fall inside the labelled run range on block mean, and
disturbances (14–64) overlap the weak end of runs (45–618). The six points were
not unlucky.

### If you extend this

The highest-value work is **more labels at the WEAK END of real departures** —
18–25 fpm, both directions — and, added 2026-08-20, **at the weak end of jogs**:
the labelled set has exactly one gentle jog (`opk` 381) and that is the single
member of the population where the verdict's one observed miss lives.

Retrospective labelling is now exhausted at 32%; the rest has to be collected.
The cheap way is to call the event aloud into the log while someone is already
in the car — see `falcon_corpus_labelling_2026-08-20.md` §6, which specs it.

Every candidate rule this project has produced looked clean on three points and
died on the fourth, and it died at the weak end each time. Three points is not a
separation.

## Regenerating

```bash
cd falcon_srcs
for f in logs/device-monitor-*.log; do
  b=$(basename "$f" .log)
  grep -E '^(BURST|JOGV|ARM |RAMP |FSM:|ACC-STAT|ACC-INT|XY:|XY-Still|Reset cause|VEL:|HEALTH|Zero-Calib|Threshold-Value|Device Booted|  Voltage value)' \
    "$f" > "datasets/${b#device-monitor-}.log"
done
```

🔴 **`RAMP ` WAS MISSING FROM THIS PATTERN UNTIL 2026-08-20, and every capture in
this directory was distilled without it.** `RAMP latched mean= dir=` does not
start with `FSM:`, so **not one ramp verdict was in the versioned corpus** --
while the notes carried claims like "17/17 on automatic stops" and "~28 lifetime
firings" that could not be checked against it. Repaired from the raw logs on the
bench machine: **87 ramp verdicts recovered** across five captures. Captures
still showing zero genuinely had none -- 2026-08-18 is inspection and bench
work, where the detector correctly declines.

⚠️ The same omission dropped `Voltage value`, `Device Booted` and the
calibration value lines. Those are restored too. **If a line type matters,
check that it survives this grep — the pattern is the corpus's real schema and
nothing validates it.**
