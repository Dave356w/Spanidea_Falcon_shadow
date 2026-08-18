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
    datasets/    480 KB    387 BURST lines -- identical count to the raw logs

## Verified equivalent

`graph/jog_window_replay.py` produces byte-identical output against either
source:

```bash
python graph/jog_window_replay.py --logs logs
python graph/jog_window_replay.py --logs datasets
```

Both report 189 departure bursts, RUN n=147 opk (4, 859), JOG n=42 opk
(972, 4154). Any tool that consumes `BURST` lines can point here.

## ⛔ THE CORPUS HAS NO GROUND-TRUTH LABELS, AND THAT IS THE BINDING CONSTRAINT

Nothing in these files records whether a burst was a real run, a jog, or a
knock. On 2026-08-18 four separate candidate rules were proposed and refuted,
and **every one of them was unfalsifiable against this corpus alone** — they
could only be tested against the handful of events an operator happened to
label out loud.

⚠️ **Do not score a candidate rule against the shipping verdict.** The `JOGV`
`verdict=` field is `opk`-thresholded by definition, so measuring `opk` against
it is circular and will report a perfect separation that means nothing. This
mistake was made and caught on 2026-08-18; see
`Eng_Notes/falcon_jog_ringing_2026-08-18.md` §0.0e.

### The labelled events that DO exist (2026-08-18, Dave, in session)

| event | label | `opk` | block mean | dir% |
|---|---|---|---|---|
| walk on cartop #2 | **not a departure** | 299 | 14 | 58% |
| hand bump | **not a departure** | 118 | 33 | 75% |
| walk on cartop #1 | **not a departure** | 389 | 34 | 50% |
| 20 fpm down #2 | **departure** | 188 | **45** | **75%** |
| 20 fpm down #3 | **departure** | 170 | 160 | 92% |
| 20 fpm down #1 | **departure** | 149 | 199 | 92% |

🔴 **These six points are the whole labelled set, and they already overlap.** A
confirmed 20 fpm departure (45 / 75%) sits inside the knock band (14–34, bump at
75%). That is what refuted the pre-filter proposal, and it is why the honest
conclusion of that session was that false-alarm-vs-missed-departure is a
**sensitivity trade-off, not a discrimination problem** with these features.

### If you extend this

The highest-value work is **more labels at the WEAK END of real departures** —
18–25 fpm, both directions. Every candidate rule this project has produced looked
clean on three points and died on the fourth, and it died at the weak end each
time. Three points is not a separation.

## Regenerating

```bash
cd falcon_srcs
for f in logs/device-monitor-*.log; do
  b=$(basename "$f" .log)
  grep -E '^(BURST|JOGV|ARM |FSM:|ACC-STAT|ACC-INT|XY:|XY-Still|Reset cause|VEL:)' \
    "$f" > "datasets/${b#device-monitor-}.log"
done
```
