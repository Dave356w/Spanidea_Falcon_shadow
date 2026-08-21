# START HERE — handover, end of 2026-08-20

Read this first, then `falcon_state_of_project_2026-08-18.md` §1 for the build
definition. Everything below is committed and pushed to `Falcon_Rel_EFT`.

---

## 0. 🔴🔴 THE ONE THING THAT MATTERS

**The beacon released on a moving car.** Contract speed, automatic, 300 fpm,
2026-08-20 afternoon. Eleven seconds of silence mid-travel, car confirmed moving
by lateral 0.2–0.884 against a rest level of 0.005–0.05. Dave called it from
inside the car; **the log alone would not have revealed it** — the release line
is identical to a healthy one.

`main.cpp` §14.4 calls silence-while-moving the only catastrophic failure. It is
now measured, not hypothesised. Full write-up:
**`falcon_false_release_2026-08-20.md`**.

⛔ **Do not treat the beacon as trustworthy on that installation** until the
arrival gate is re-derived.

**Why it cannot be fixed by tuning:** real arrivals on that build measured
0.450–0.511 (one *exactly* on the 0.45 gate); the false release was a cruise
transient of **0.472 — inside that population**. Raise the gate and four of six
real arrivals are rejected, each then holding the beacon over a stopped car to
the 600 s failsafe. This retires "retune `ARRIVAL_PEAK_VALUE`" the way 08-18
retired "retune `ANYMOTION_THRESHOLD`".

## 0a. ⚖️ WHAT IS ESTABLISHED vs WHAT IS NOT — read before acting on §0

Nine notes were written on 2026-08-20 by two parallel sessions, and the language
in them is alarming in places. This is the proportionate version.

**Established, and not negotiable:**

- **One release on a moving car, confirmed.** Dave called it from inside the
  car; lateral read 0.2–0.884 for eleven continuous seconds against a rest level
  of 0.005–0.05; the device then re-latched, which it only does on a fresh
  departure. An independent instrument scores its lateral verdict at 0.311
  against 0.018–0.095 for every other release on file. **This is the single
  best-evidenced event of the day. It is not a ghost.**
- **The arrival gate cannot separate the two populations on that machine** —
  real arrivals 0.450–0.511 (one exactly on the gate), false release 0.472,
  inside the population.
- **Every candidate fix is refuted**: retuning the gate, reversal-only arming
  (misses 51.5%), ramp-priority (changes nothing), ramp-veto (blocks 100% of
  inspection stops).

**NOT established — do not act as though it were:**

- **The rate.** One confirmed event, and **35 runs since with none flagged**.
  On the evidence this is roughly 1 in 44 on one machine, and a single event
  cannot support a rate at all. It is not "every trip" and nothing here says it
  is getting worse.
- **A second event.** `142248:696` is flagged MOVING but reads only **1.01×**
  the rest threshold — the tool itself calls it UNRESOLVED. Treat it as
  unresolved, not as a second data point.
- **That the problem generalises.** All contract-speed automatic data comes from
  **one building on one afternoon**. The 350 and 500 fpm machines have never
  been run against this build.
- **That anything got worse today.** Nothing regressed. The firmware behaved the
  same before and after every change; what changed is that the instruments can
  now see what it was already doing.

⚠️ **The catastrophic-failure framing is about CONSEQUENCE, not frequency.**
Silence while moving is the one failure the product exists to prevent, so one
instance is worth this much writing. That is not the same as saying the device
fails often, and the notes should not be read that way.

## 1. Build on the device and in the repo

⚠️ **SUPERSEDED 2026-08-21.** The sizes below were correct on 08-20 — a
baseline build of `345613f` reproduces them to the byte — but the lateral term
of `falcon_fsm_logic_and_shortcomings_2026-08-21.md` §4.2 then pushed three of
five environments past the end of flash. Recovered in `ce28d9d`; the current
numbers are the right-hand column and `falcon_flash_budget_2026-08-21.md` has
the account.

| | | now (`ce28d9d`, flashed and verified) |
|---|---|---|
| Tag | `build-2026-08-20` | |
| Plus | `ARM_REV_SAMPLES` 8 → 15 (commit `f75b0b7`, flashed and verified) | lateral veto in `STATE_DECELERATING`, armed, capped at 60 s |
| `ATmega328PB` | 32176 / 32256 — 80 free, RAM 1499 | **32112 — 144 free, RAM 1493** |
| Envs | `production` (31706), `production_silent` (25590), `bench_battery`, `idle_current`, `brownout_test` — all build | `production` **31746**, `production_silent` **25854**, `bench_battery` **31838** — all six build |

⭐ **Do not quote a free-space number from a note or from `platformio.ini`.**
Both were stale within a day and it cost a build. `pio run -e <env>` takes
three seconds on the bench machine.

Ports on the bench machine: **COM5 = log UART, COM7 = ISP**. Confirm with a
signature read; the programmer re-enumerates constantly.

## 2. What today added

- **Session G** — corpus labelled, now **98 records with named evidence**
  (`datasets/session_g_labels.csv`), including 12 collected *at the car* with
  Dave's own words. Jog population 17 → 22.
- **Session A** — flash 62 → 706 free (`RollingAvg` fixed array, `malloc` gone),
  then `FALCON_LOG` for two production builds. 626 spent since on instrumentation.
- **B1** — every latch prints its path and context:
  `FSM: Departure latched (polled) ml= q= dq= td=`. Closed instrumentation gap 1.
- **Polled re-arm gate** — the polled departure path had never been blanked.
  Two bench taps produced nine alarm cycles in 90 s; fixed and verified.
- **Session C's table** — 11 automatic stops; the rest gate is **not** inert, it
  was inert only on the inspection population it had been measured against.
- **First automatic operation at contract speed** — **33 runs**
  (`falcon_automatic_2026-08-20.md`). The first 11 were analysed live; the
  remaining 18 arrived when the background capture completed, §4a.

## 3. ⚠️ Things I got WRONG today and retracted — do not re-derive the wrong version

Each is corrected in place, but a fresh session reading only the headline could
easily re-introduce them:

1. **"`(polled)` means any-motion missed the departure."** No — `latch_path` is
   **evaluation order**, not detection order. The polled test runs first and is
   first-setter-wins. Every polled-latched run also carries a departure burst,
   which only the any-motion branch emits. (`31b58bc`)
2. **"`g=0` means the reversal path was unavailable."** No — `g=` is sampled
   when the ARM line prints. All four `g=0` runs had the ramp gate open shortly
   after and the ramp latched. (`38f9744`)
3. **"The ramp detector shares `arr_armed`."** No — it has had its own
   reversal-only `ramp_gate` since 2026-08-12. (`38f9744`)
4. **"Cruise starves the arming counter — lateral 0.134–0.649 vs 0.06."** Wrong
   channel. Arrival arming is **vertical** (`ARRIVAL_QUIET_MSS` 0.15); lateral
   quiet gates only the re-arm blank. Conclusion survives on the vertical
   channel (cruise `cp` 0.27–0.29 vs 0.15). (`976e6a8`)
5. **A false-latch regression attributed to `RollingAvg`** — 4/4 vs 0/4 across
   *time-separated* arms, then 3/3 clean on both when interleaved. The bench had
   drifted. **Interleave the arms of any bench A/B.**
   (`falcon_session_a_2026-08-20.md` §3)

## 4. The open decision, with the evidence already gathered

**Reversal-only arming is disqualified as a straight swap** — `arming_replay.py`
now reports a `NO RELEASE` column (nothing fired at all):

```
rule     INTO-RAMP (safety)     NO RELEASE
quiet       3/239  UNSAFE       78/241   32.4%
rev         0/239  ok          124/241   51.5%
union       3/239  UNSAFE       55/241   22.8%   <- shipping
```

Safe and useless: it misses more than half of all arrivals.

**⭐ The candidate worth building next: RELEASE PRIORITY, not a threshold.**

The ramp detector fired on every contract-speed stop analysed, `dir=100` every time,
means 471–511 — and was **never once the releasing path**, because the polled
peak always wins the race. Its discriminator is sustained one-signed
deceleration, which a cruise transient cannot fake. It correctly declines
inspection brake sets (0/N on 08-12), which is why the peak must stay for those.

So: **give the ramp verdict priority where one is available, keep the peak for
brake sets.** Unlike a gate swap this does not trade one failure direction for
the other.

⛔ **RETRACTED THE SAME DAY — see `falcon_ramp_priority_2026-08-20.md`.** The
replay is done. Priority with a peak fallback (`ramp_pri`) releases the false
release anyway, 2 s later, still mid-travel: its ramp landed 9798 ms after the
peak, so no usable hold reaches it. Only a *veto* — refusing to release without
ramp confirmation — prevents it, and that costs a NO RELEASE on **100% of
inspection stops**, which carry 0 ramp verdicts in 79 runs. It does trade one
failure direction for the other. The open question is now whether the firmware
can tell automatic operation from inspection at runtime; `g=` is not that
discriminator.

## 4a. 🟢 One piece of good news, found after the session was written up

**`arm_via = 2` has fired — the reversal arming path works.** Three times in the
corpus, all clean releases at a real arrival: once on **2026-08-13** (which
means "`v=2` has never fired in ~32 runs" was already stale when written, and
the evidence was sitting in the committed corpus) and twice on 08-20.

All three armed with `q` of 2 or 4 — **below** `ARRIVAL_ARM_SAMPLES` 5, so quiet
had not armed and could not have. **The insurance path fires in exactly the
condition §0 identifies as the problem.** Readiness item 4 ("two release paths
with ZERO live evidence") is half wrong.

The background capture ran on after the write-up and holds **33 departures, not
11**. Full stats in `falcon_automatic_2026-08-20.md` §5b–5c. The headline:
**8 of 32 arrivals land within 5% of the gate, one exactly on it.**

## 5. Other open items in priority order

1. **`ARRIVAL_QUIET_MSS` 0.15 vs measured cruise 0.27–0.29** — the arming gate
   is starved on its own channel.
2. **The peak collector armed at exactly its minimum (`q=5`) on 8 of 32**
   contract-speed runs, and **8 of 32 arrivals land within 5% of the gate**
   (one exactly on it). Partly mitigated: when quiet fails outright the
   reversal path arms instead — §4a.
3. **Session C proper** — no re-latch in 11 stops is a null result, not a clean
   bill. Item 9 open.
4. **B3** — burst on the polled departure path. Needs room; 80 bytes free, so it
   belongs on `production`.
5. **Jog gate has no room left** — jogs reached `opk` 937/953 against a labelled
   real run at 972. 35 counts apart.
6. **Quiescent current has never been measured.** `idle_current` now builds.
7. **Battery divider** — `pack_mv` reads ×2, `VBATT_CONST` is 4.4, `Release.txt`
   says 3.2 V. Three answers, no meter.

## 5a. Every note from 2026-08-20

Two sessions wrote in parallel; this is all of it, in reading order.

| note | what it holds |
|---|---|
| **this file** | handover, and §0a's proportionate framing |
| `falcon_false_release_2026-08-20.md` | the release on a moving car — the one that matters |
| `falcon_ramp_priority_2026-08-20.md` | ramp-priority refuted, ramp-veto costed (parallel session) |
| `falcon_automatic_2026-08-20.md` | 33 contract-speed runs, session C's table, the `v=2` firing |
| `falcon_session_a_2026-08-20.md` | flash headroom, `FALCON_LOG`, and the A/B method error |
| `falcon_b1_2026-08-20.md` | latch attribution, the bench cascade, the polled re-arm gate |
| `falcon_cartop_2026-08-20.md` | inspection runs and jogs; the jog floor at 937 |
| `falcon_corpus_labelling_2026-08-20.md` | the session G procedure (written before the run) |
| `falcon_corpus_labelled_2026-08-20.md` | the session G result — 98 labelled records |

## 6. Data

`datasets/260820-*.log` — six captures from today: bench cascade, cartop
inspection (5 runs + 5 jogs), the long 18 fpm run, contract-speed automatic,
and **`260820-150000.log` — 33 departures, containing both the false release
and all three `v=2` arms**.

⚠️ **`falcon_srcs/logs/` is gitignored** — the raw captures exist only on the
bench machine. The distilled event corpus in `datasets/` is what is versioned.

## 7. Method notes that cost real time today

- **Never stop the capture until Dave says the session is over.** "Write it up",
  "commit" and "done" are checkpoints. A 4-minute 18 fpm up run — the longest
  slow run ever attempted — was lost this way.
- **`pio device monitor` fights programmatic capture.** A 20-line pyserial
  reader writing straight to a file is what made repeatable A/B possible.
- **Sample lines lag FSM lines** — the ring is drained one entry per `loop()`
  pass, so do not infer causal order between a `t=` line and an adjacent FSM
  line.
