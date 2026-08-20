# Ramp priority, replayed — and it does not do what §4 said

**Desk work, no car.** `falcon_START_HERE_2026-08-20.md` §4 named this as the
next step: extend the replay to ask what each arrival would have done under
ramp-priority. Tool: `falcon_srcs/graph/ramp_priority_replay.py`.

---

## 0. The finding, in one line

**Release priority does not prevent the false release. Release *veto* does.**

§4 proposed "give the ramp verdict priority where one is available, keep the
peak for brake sets", and claimed that "unlike a gate swap this does not trade
one failure direction for the other." Replayed against every instrumented run
on file, the first half is wrong and the second half is the whole difficulty:

- **Deferring** the peak to let the ramp answer first releases the 2026-08-20
  false release anyway, 2 s later, still mid-travel. ⛔ Retract §4's claim.
- **Refusing** to release without ramp confirmation does prevent it — and
  refuses to release on **100% of inspection stops (0 ramp verdicts in 79
  runs)**, holding the beacon to the 600 s failsafe on every one.

So it *does* trade one failure direction for the other. What it buys is that
the trade is now a **mode question** rather than a threshold question, and mode
is something the device may be able to know.

## 1. Why this is measurement, not simulation

`arming_replay.py` works on 80-sample bursts and can only ask "would this rule
have armed". The ramp-priority question is about *timing* — when the ramp
verdict lands relative to the peak — and that needs the continuous stream.

It did not need replaying, because **it was already measured**. Since
2026-08-12 `emit_ramp_log()` prints `RAMP latched mean= dir=` from `loop()` the
moment the verdict latches, regardless of FSM state, and the accumulator keeps
running after the peak has already dropped the latch. Every run since then
carries a direct answer to "when would the ramp have fired if the peak had not
gone first". The tool reads those lines. The only thing simulated is the choice
of which verdict to obey.

**Corpus:** 241 released runs. **43 excluded** as pre-`emit_ramp_log` — they
cannot print a ramp latch, so scoring a ramp rule on them would manufacture a
NO RELEASE out of a missing print. Pool **198**, of which **105** carry a ramp
verdict.

⚠️ The live capture `260820-152131` was still open when these numbers were
taken, so its run count will have grown since. Re-run the tool rather than
quoting the totals.

## 2. The race, measured

§4: "the polled peak always wins the race." Confirmed, and it is not close:

```
ramp minus peak, ms:   min 1115   median 1868   max 9798
ramp landed FIRST on   0/105 runs
```

The ramp verdict is **never** available at the instant the peak fires. Any rule
that prefers it must therefore *wait*, and the whole design question is what to
do when the wait expires.

## 3. The four rules

All four arm identically. Only the release authority changes.

| rule | what it does |
|---|---|
| `peak` | shipping: first peak crossing releases |
| `ramp_only` | the peak never releases; ramp or nothing |
| `ramp_pri(W)` | hold the peak W ms; ramp releases if it lands inside W, else the peak releases late |
| `ramp_veto(W)` | as above, but on W expiring the peak's release is **discarded** |

`ramp_pri` is §4's proposal. `ramp_veto` is the one that works.

```
rule             MOVING     OK   NONE  UNKNOWN  median delay
peak                  2    196      0        0          0 ms  <- CATASTROPHIC
ramp_only             1    103     93        1       1868 ms  <- CATASTROPHIC
ramp_pri              2    196      0        0       2000 ms  <- CATASTROPHIC
ramp_veto             0     64    134        0       1687 ms  ok
```

MOVING is silence while moving — `main.cpp` §14.4's only catastrophic failure.
NONE is the beacon held over a stopped car to the failsafe: a position lie.

**On the false release specifically** (`260820-142248:6504`, peak 0.472, the run
Dave called from inside the car):

```
peak       -> MOVING   lateral 0.311
ramp_pri   -> MOVING   +2000 ms, lateral 0.265   <- still catastrophic
ramp_veto  -> NONE                               <- prevented
ramp_only  -> UNKNOWN  +9798 ms, unscoreable
```

Its ramp landed **9798 ms** after the peak — the largest gap in the corpus, and
far outside any usable hold. `ramp_pri` cannot prevent a release that no ramp
ever confirms; it can only postpone it. It postponed this one into cruise.

## 4. What the veto costs, disaggregated

The pooled NONE rate of 134/198 is **not** the decision-relevant number,
because a declining ramp is *correct* on an inspection brake set and a failure
on a contract-speed stop, and both are in the pool. Split by capture, and swept
on the hold window, the picture separates cleanly:

| capture | runs | ramp | veto NONE @2000 | @2500 |
|---|---|---|---|---|
| `260812-112254` automatic | 33 | 30 | 17 | **3** |
| `260812-140913` automatic | 16 | 14 | 6 | **2** |
| `260813-110300` | 10 | 8 | 2 | **2** |
| `260813-103846` | 3 | 3 | 0 | **0** |
| `260820-142248` contract speed | 32 | 31 | 18 | **3** |
| `260820-152131` today, live | 11 | 11 | 6 | **0** |
| `260819-121259-cartop` mixed | 14 | 8 | 6 | 6 |
| `260818-105317` inspection | 56 | **0** | 56 | 56 |
| `260818-133242` inspection | 16 | **0** | 16 | 16 |
| `260820-120250` cartop | 5 | **0** | 5 | 5 |
| `260820-122616` cartop | 2 | **0** | 2 | 2 |

**W = 2500 ms is a knee.** Below it the ramp has not finished arriving —
median 1868, and half the population lands past 2000. Above it nothing further
is bought. At 2500 the veto's cost on automatic operation falls to roughly
**3 in 32**, while the inspection captures stay at **100% NONE and always
will**, because their ramp verdict count is zero by construction.

⚠️ At W ≥ 2500 one run (`260812-112254:14418`) turns MOVING under the veto, at
1.33× the rest threshold — **unresolved, not a finding.** See §6.

**The long-fallback variant does not rescue `ramp_pri`.** Widening W so the
peak eventually fires anyway never reaches zero MOVING; it only pushes runs
into UNKNOWN as the verdict window runs past the next departure — 38 of 198 at
W=12000. That is the instrument losing the ability to score, not a clean bill.

## 5. So what actually blocks shipping this

Not the threshold. **The mode.**

`ramp_veto` is safe and cheap on automatic operation and unusable on
inspection. The device does not currently know which it is in. `ramp_gate`
(`g=`) is not that discriminator — it is open on plenty of inspection runs
(`260818-105317` opens it and still never qualifies a ramp).

⬜ **The next question is therefore: can the firmware tell automatic operation
from inspection at runtime?** If it can, the veto ships behind that flag and
the false release is closed. If it cannot, this candidate is in the same
position as the arrival gate — correct in one population, catastrophic in the
other, with nothing to select between them.

That is a smaller, sharper question than the one §4 posed, and it is the one
worth spending the next session on.

## 6. Method notes, including two ways this analysis was wrong first

The ground truth is the **lateral channel**, not the log: `FSM: Arrival
(polled), peak 0.472` is byte-identical on a healthy release and on the false
one. `m` reads 0.005–0.05 at rest and 0.175–0.884 moving. The instrument is one
function — median `|m|` over `[t+5 s, t+11 s]` — asked at whatever instant a
rule would have dropped the latch, so a rule that releases later is judged on a
correspondingly later window. Calibrated on `260820-142248`, where the window
reads **0.322** for the known false release and **0.018–0.095** for all 31
others.

Both of the following produced *plausible pooled tables* and were caught only
by checking the one release whose truth is known independently. There is now a
`--selftest` that asserts exactly that.

1. **A derived `t_stop` disagreed with its own calibration.** Scoring releases
   against "first sample after which `m` stays quiet for 2 s" flagged runs as
   *early by 21 s* that read 0.018 — dead at rest — in the verdict window, and
   put the **shipping** build at 50 early releases in 219. A settling tail is
   not a stop time. Deleted, not fixed.

2. **`millis()` is not monotonic across a capture.** `260820-142248` carries a
   `Reset cause: 0x2` mid-file, so `t=161054` occurs twice, 6300 lines apart.
   Selecting the verdict window by timestamp silently pulled *parked-car
   samples from before the reboot* into the window of a run after it, and
   scored the known false release **0.0435 STOPPED**. Windows are now anchored
   in **file order**. Any tool in `graph/` that filters this corpus by
   timestamp alone has this bug.

⚠️ **Blind spot, unavoidable:** this detects a release early enough that the car
is still moving 5–11 s later. A release 1–2 s early sits inside the settling
tail of a correct one and is not separable this way. **Every MOVING count here
is a lower bound**, and a zero means "none this instrument can see".

⚠️ Treat any verdict inside 2× of the rest threshold as unresolved. Two runs in
the whole corpus sit there (`142248:696` at 1.01×, `112254:14418` at 1.33×);
neither is quoted as a finding above.

## 7. Today's live session

`260820-152131`, running while this was written — the 1-4 / 4-1 sequence.
**11 runs, a ramp verdict on all 11**, ramp delays 1573–2441 ms, post-release
lateral 0.024–0.046 across the board. **No release on a moving car that this
instrument can see.** It is also the tightest ramp-delay population on file,
and the only capture where `ramp_veto` at W=2500 costs nothing at all.

That is a good sign and it is not a clearance. Eleven runs is eleven runs, the
blind spot above still applies, and ⛔ the standing instruction not to rely on
the beacon on that installation is unchanged by it.
