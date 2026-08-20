# Automatic operation at contract speed, 2026-08-20 — session C's table, and both arming gates at minimum on one run

**In-car, automatic, 300 fpm, 4-floor building, Dave dispatching.** Build
`build-2026-08-20`. Capture `falcon_srcs/datasets/260820-140000.log`.
**Eleven runs**, full-travel and single-floor, both directions.

The first automatic operation since the primary build landed, and the first
contract-speed data on it at all.

---

## 1. The eleven runs

| # | dep path | `ml` ms | arrival | margin | `q` (need 5) | `ro` (need 8) | ramp |
|---|---|---|---|---|---|---|---|
| 1 | any-motion | 217743 | 0.493 | 1.10× | **5** | **8** | 492 |
| 2 | any-motion | 53510 | 0.606 | 1.35× | 6 | 13 | 511 |
| 3 | any-motion | 99828 | 0.608 | 1.35× | 10 | **4 `g=0`** | 480 |
| 4 | polled | 33276 | 0.584 | 1.30× | 6 | 9 | 484 |
| 5 | any-motion | 177865 | 0.501 | 1.11× | 16 | 10 | 504 |
| 6 | any-motion | 23855 | 0.555 | 1.23× | 8 | 9 | 505 |
| 7 | any-motion | 195822 | 0.657 | 1.46× | 8 | **8** | 471 |
| 8 | polled | 6718 | 0.537 | 1.19× | 35 | **6 `g=0`** | 494 |
| 9 | any-motion | 7160 | **0.459** | **1.02×** | **5** | 12 | 503 |
| **10** | **polled** | **8651** | **0.468** | **1.04×** | **5** | **7 `g=0`** | 497 |
| 11 | polled | 23675 | 0.511 | 1.14× | 9 | **6 `g=0`** | — |

Aggregates:

- **arrivals 0.459–0.657, every one inside 1.46×**, three below 1.10×
- **`q`: 5, 5, 5, 6, 6, 8, 8, 9, 10, 16, 35** — three at the floor
- **`ro`: 4, 6, 6, 7, 8, 8, 9, 9, 10, 12, 13** — `g=0` on **4 of 11**
- **ramp 11/11, `dir=100` every time, mean 471–511**

⚠️ **`dep path` is EVALUATION order, not detection order.** The polled test runs
first in `fsm_run()` and `latch_path` is first-setter-wins; every polled-latched
run here also carries a departure burst, which is emitted only inside the
any-motion branch. Both paths fired on all of them. See
`falcon_cartop_2026-08-20.md` §2.

## 2. 🔴 Run 10 — both arming gates at minimum on the same run

```
#10  polled  ml=8651  arrival 0.468 (1.04×)  q=5  ro=7  g=0
```

- quiet armed at **exactly 5 against a requirement of 5** — zero margin
- the reversal path was **unavailable**, `ro=7` against 8
- the arrival then cleared by **4%**

**Arming rested on a single sample of a single path, with no backup.** One
sample lost to buzzer phase and nothing arms — which switches off *both* the
peak collector and the ramp detector for the whole stop, and the beacon then
holds over a stationary car until `LATCH_FAILSAFE_MS`. That is the 85-second
false-beacon condition of `falcon_cab_automatic_2026-08-12.md` §4.2, and it came
within one sample on an ordinary service run.

**The two gates exist as independent insurance for each other.** Across eleven
runs quiet hit its floor three times and reversal was unavailable four times.
That is not a thin tail — **a majority of runs had one gate or the other at or
below its minimum**, and run 10 had both.

## 3. 🟢 Session C's table — and the rest gate is NOT inert

The plan's question: does a ringing stop hold `quiet_run()` down materially
longer than an ordinary one? If not, rest-gating cannot discriminate.

| threshold | 08-19 inspection, 20 stops | **08-20 automatic, 11 stops** |
|---|---|---|
| `q≥8` | **0 ms on 13 of 20** | **0 of 11 already quiet**, 311–2720 ms |
| `q≥16` | 0 ms on 10 of 20 | 0 of 11, 312–6554 ms |
| `q≥32` | 0–1934 ms | 312–31359 ms |
| `q≥64` | 0–7045 ms | 639–15040 ms, 10/11 |
| `q≥128` | 1589–18890 ms, 3 never | 3277–18711 ms, 9/11 |

**On inspection stops `q` was already ≥8 at MONITORING entry on 13 of 20, which
is why 08-19 concluded "the gate is not gating". On automatic contract-speed
stops it was already ≥8 on ZERO of eleven.**

So the rest gate was not inert — **it was inert on the population it had been
measured against.** `MONITOR_REARM_QUIET` 8 does real work at contract speed,
and stop 2 took **2720 ms**, past the 2500 ms `MIN_MS` floor, so the quiet
condition actually extended the blank. First observation of that mechanism
doing what it was designed to do.

**This reframes item 9.** The concern was that shortening the blank re-opened
the 08-07 stationary-car alarm. The blank *self-extends* on stops that ring,
which no fixed floor can do.

⚠️ **Caveats.** Eleven stops, one machine, one mounting. Floors and directions
come from Dave's callouts; the controller's slowdown profile per stop is not
independently confirmed. **No re-latch occurred on any stop** — a null result on
eleven, not a clean bill.

## 4. ⚠️ Cruise starves the quiet counter — the mechanism behind §2

Measured during the first 4→1 (13.6 s at 300 fpm):

```
lateral m 0.134 .. 0.649    q = 0 for the ENTIRE cruise
```

`XY-Still` on this mounting is **0.06**. Cruise lateral runs **2× to 10× the
still threshold, continuously**, so the quiet counter never advances while the
car moves. `q=5` therefore does not mean "five quiet cruise samples" — **all
five were accumulated from the post-stop ringdown.** The gate arms *during the
arrival it is supposed to be armed before*.

That inverts the recorded model. 08-12 measured `q=119` on a two-floor run and
concluded "a run with real cruise has unlimited quiet to arm on". Here a
four-floor run armed with **less** margin than a single-floor hop.

It also joins the cartop measurement the same day: vertical cruise `cp` reached
0.27–0.29 against a 0.15 quiet gate, lateral `m` reaches 0.649 against a 0.06
still threshold. **Both channels agree that this machine's cruise is not quiet,
and the entire arm-on-quiet design assumes it is.**

## 5. ⬜ The ramp detector: 11/11, and still never the releasing path

Mean 471–511, `dir=100` on every one — the tightest cluster on record, and
exactly the 487–501 band from 08-12. Every single one latched **after** the peak
detector had already released.

Lifetime it has now fired ~28 times and has never once been the path that
released the beacon, because it is gated on the same `arr_armed` flag as the
detector it exists to back up. §2 is what that costs: when arming is marginal
both go off together.

## 6. What this session did not settle

- **No re-latch, no missed departure, no suppression.** `dq=0` on all eleven.
- **The polled re-arm gate added this morning cost nothing** — shortest dwell was
  `ml=6718`, well outside the ~2.5 s blank, so a departure inside the blank
  remains untested in a car.
- **`v=2` still has never fired.** The reversal path was *unavailable* four
  times; it has still never been the path that armed.
