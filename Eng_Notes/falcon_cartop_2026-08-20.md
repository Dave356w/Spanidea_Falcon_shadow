# Cartop inspection, 2026-08-20 — the first runs on the primary build

**Cartop, inspection, 19 fpm, Dave driving.** Build `build-2026-08-20`
(32176/32256). Capture `falcon_srcs/datasets/260820-120250.log`. Five runs.

This is the first car time for session A's `RollingAvg` change, B1's latch
attribution, and the polled re-arm gate — all three landed on the bench the
same morning.

---

## 1. The five runs

Eleven latches total: five runs (§1), five jogs (§5a), and one unidentified
event as the capture was stopped.

| # | dir | stop, as called at the car | dep path | `opk` | `dq` | arrival | margin | ARM |
|---|---|---|---|---|---|---|---|---|
| 1 | up | **into floor level, less harsh brake set** | any-motion | 184 | 0 | **0.465** | **1.03×** | `q=255 g=1 ro=8` |
| 2 | down | cartop set | any-motion | 66 | 0 | 0.953 | 2.12× | `q=255 g=1 ro=9` |
| 3 | up | cartop set | any-motion | 162 | 0 | 1.855 | 4.12× | `q=255 g=1 ro=8` |
| 4 | up | **terminal, floor level** | **polled** | 118 | 0 | 1.389 | 3.09× | `q=248 g=1 ro=9` |
| 5 | down | gentle attempted, came out hard | any-motion | 61 | 0 | 2.421 | 5.38× | `q=255 **g=0** ro=7` |

Mounting: `Zero-Calib 9.7576`, `XY-Still 0.0615`, `Threshold-Value 0.040000`
(clamped to `Z_THRESH_MIN` again — that is now four mountings in a row).
Re-arm settle 2540 ms. `tw=1` for the whole session, one TWI guard trip at boot
and none during runs. Pack 2433 → 2369.

## 2. 🟢 B1 earns its keep on the first run it saw

**Run 4's departure was caught by the POLLED path.** Before 2026-08-20 that path
entered `STATE_MOVEMENT_DETECTED` printing nothing at all, so this run would
have appeared in the capture as a `STATE_MOVING` transition with no latch line —
and would have been dropped from any run count taken from `Departure latched`.
The 08-19 session hit exactly that: 4 real runs, 3 latch lines, and the silent
one recovered only by cross-checking raw state transitions by hand.

It also says something about detection: **any-motion missed run 4 and the
backstop covered it, at 19 fpm** — the speed where any-motion is weakest. The
layered design working, visible for the first time because the instrument now
exists.

`dq=0` on all five, so no motion evidence was discarded on any run and the
missed-departure signature did not appear. **Five for five departures caught;
lifetime slow-departure record since the re-arm correction is now 0 missed.**

## 3. 🟢 The polled re-arm gate does not over-fire

Zero `polled ignored, re-arm blanking` lines across five real stops. The gate
added this morning suppressed a nine-cycle cascade on the bench and did not
touch a single real-car event. That is the evidence that mattered most for it,
and it is the cheapest possible confirmation — it cost nothing but reading the
log.

⬜ It has still never been tested against a departure that begins inside the
blank, which is the case where it would cost something.

## 4. ⚠️ 0.465 — a new low for the soft arrival mode

Run 1 stopped into floor level with what Dave described as a **less harsh brake
set**, and released on a peak of **0.465 against the 0.45 gate — 1.03×**.

The previous floor for this mode was **0.573/0.579**, measured 2026-08-11 on
125 fpm terminal approaches and recorded as a mechanism rather than a tail
("a controller with a finer crawl will land lower — check per installation").
At 19 fpm on inspection it lands **19% below that**, and 3% above the gate.

This is §14.7's gentle arrival, the failure the notes call the one still worth
engineering against: miss it and the beacon asserts over a car that has already
arrived until `LATCH_FAILSAFE_MS`.

**The variable is the brake set, not the floor.** Runs 1 and 4 were both stops
into floor level and produced 0.465 and 1.389. What separated them was the
harshness Dave felt, which he cannot command on inspection.

### 4.1 ☠️ A recorded finding does not survive this mounting

2026-08-12 measured up arrivals as cleanly weaker than down — **5/5 separation,
up max 1.245 below down min 2.164** — and called it "a real property of this
machine worth carrying into the next installation."

Here the populations interleave: up 0.465, 1.389, 1.855 against down 0.953,
2.421. **Up reaches 1.855, above a down of 0.953.** The clean separation is
gone.

What survives is narrower and is still the part that matters: **the weakest
arrival of the session was an up one, and it is the only one near the gate.**

## 5. ⚠️ The reversal gate failed to open on run 5

`ARM q=255 a=1 v=1 g=0 ro=7` — `ro=7` against the 8 required, and `g=0`. Every
other run this session read `g=1`. It cost nothing because the quiet path armed
at 255, but this is the reversal path failing to open on an ordinary run, and
that path exists as insurance for precisely the short runs where quiet fails.

`ro` across the session: 8, 9, 8, 9, **7**. The 2026-08-13 session recorded
8/10/11/14 and called `ro=8` "zero margin". This is the first `ro` below the
gate on a real run.

## 5a. 🔴 Five jogs, and the jog floor moves 1366 → 937

| # | called | `ratio` | `opk` | verdict | margin over the 900 gate |
|---|---|---|---|---|---|
| 6 | "jog down" | 68 | 1190 | JOG | 1.32× |
| 7 | "jog down" (2nd) | 52 | **937** | JOG | **1.04×** |
| 8 | "jog up" | 44 | 953 | JOG | 1.06× |
| 9 | **"brief jog"** | 89 | **3244** | JOG | 3.60× |
| 10 | (in the announced block, not called) | 77 | 1546 | JOG | 1.72× |

**5/5 verdicted JOG and 5/5 released on the verdict.** No jog silenced a moving
car; each cost ~4 s of beacon instead of a 600 s failsafe.

**The lifetime jog floor was 1366 (2026-08-11). Two of these are below it —
937 and 953 — within 6% and 4% of the gate.** That is the largest revision to
this population since it was first measured.

⚠️ **Set against the labelled corpus, the gap has now closed from both sides.**
Real departures were measured at 11–562 with one outlier at **972**; jogs now
reach down to **937**. A real run and a jog have been recorded **35 counts
apart**. The 900 gate sits between them with essentially nothing to spare in
either direction.

What still holds: all five cleared the ratio gate at 44–68, so the AND carried
them. The ratio axis — written off in the 08-11 note as "dead as an independent
discriminator" — is what did the work here.

### 5a.1 ⭐ A brief jog is the EASIEST to detect, not the hardest

The brief jog came in hardest of all five: `ratio=89 opk=3244`. That is not
noise, it reproduces 2026-08-11 §2 exactly — the shortest blip that evening read
ratio 95 and was "the most jog-like event of the day", because **a brief jog has
no cruise segment to dilute the impulse pair**. Release and brake set land back
to back, cancellation is near-perfect, and the full uncontrolled brake shock is
in the window.

**Duration is the wrong axis to chase a gentle jog on**, and the session was
steered that way for a while on the assumption that it was the right one.

### 5a.2 ⬜ A sub-900 jog may not be producible from the cartop

The mechanism on file: cartop jog control runs *drive energizes → brake releases
→ possible rollback → run enable → brake sets with no motor field control*. The
brake set is **uncontrolled by design** on inspection, which is why it delivers
1.9–3.2 m/s² every time. Five jogs today, none under 937.

The only jog ever recorded under the gate (`opk=381`, 2026-08-18) was on the
**inverted bench mounting**, not a cartop.

**If that holds it narrows open item 4 considerably:** the verdict would be
reliable where mechanics actually jog, and the gentle-jog miss would be a
property of a particular mounting rather than of jogging. It is a hypothesis
from one session and one machine — but it is testable, and it is cheaper to test
than a redesign.

## 6. Purpose-collected labels — the session G pivot, executed

`falcon_corpus_labelling_2026-08-20.md` §6 specced collecting labels at the car
because retrospective labelling is capped at 32%. These five are the first:
each carries the operator's own words as its basis, called before or during the
run rather than reconstructed afterwards.

Corpus now **97 labelled with named evidence** (71 run / 22 jog / 4
disturbance) — 33.0% of 294 departures.

**The jog population grew 17 → 22, and it grew at the end that mattered.**
Rows 6–9 carry Dave's own words as their basis, called before or during each
jog. Row 10 was inside the announced jog block but not individually called, and
says so; row 11 is `unknown` — a final latch as the capture stopped, with no
callout and no arrival.

⬜ **Still no jog under 900.** The one gentle jog in the corpus remains the
2026-08-18 `opk=381`, and §5a.2 is the hypothesis for why.

## 7. What this session did not do

- **It is not session C.** That needs automatic operation: the failure is a
  terminal-floor brake set on the controller's extended slowdown profile, and
  inspection is below leveling speed so no such transition occurs. Item 9 stands.
- **No automatic operation at all**, so the ramp detector saw nothing and the
  arrival gate's contract-speed behaviour is untouched.
- **One soft-mode sample.** The mode that sits on the threshold has n=1 in this
  shaft and n=3 lifetime.
