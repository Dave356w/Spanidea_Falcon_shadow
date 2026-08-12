# Ramp detector armed, and the lockup that ate a run — 2026-08-12 (late session)

**Firmware `7210dfd` (`RAMP_ARMED 1`, 32214 bytes) for all four car runs.**
Bench work afterwards landed `b9ee1fd` (linker relaxation) and `9d57c9a`
(vendored Wire1 with bounded TWI waits, 32056 bytes) — the latter is what the
device is running now.

Read `falcon_cab_automatic_2026-08-12.md` first; this note continues it and
assumes the ramp detector's design and the arming-gate problem.

Two results matter. One is the measurement that justifies the ramp detector
outright, on a single stop. The other is that the TWI lockup stopped being a
reliability annoyance and became a **missed car run** — a beacon that did not
sound while a counterweight moved.

---

## 1. Arming the ramp detector cost 2 bytes, and the bma4 swap was never in the way

`RAMP_ARMED` 0 → 1 in `movement_service.h`. The armed and unarmed branches are
both already compiled; arming swaps which one runs.

    unarmed   32212 bytes
    armed     32214 bytes      +2

The standing plan had the bma4 driver swap as a prerequisite for flash-hungry
work. It is not a prerequisite for this: the swap gates `-DWIRE_TIMEOUT`
(+1554, §5), not a `#define` flip. This was measured, not assumed.

Both comment blocks that claimed the detector ships unarmed were rewritten in
the same commit — `main.cpp` above the `RAMP_*` constants, and the flag's own
block. They carried the arming criteria and the rollback, so leaving them
stale would have been worse than having no comment at all.

Protocol §3.1 was satisfied before arming: 17/17 automatic drive stops latched
(means 472–513, directionality 100% every time) against zero hits across a
full inspection session and 51 replayed departure bursts.

---

## 2. 🟢 One stop, two detectors: 1.016× against 2.18×

Run 3 (1→2 up, single floor) is the whole argument for the ramp path in one
measurement:

| detector | value | gate | margin |
|---|---|---|---|
| windowed arrival peak | 0.457 | 0.45 | **1.016×** |
| ramp block mean | 653 | 300 | **2.18×** |

Same stop, same signal, same 3.9 seconds. The peak cleared its threshold by
1.6%; the ramp cleared its own by 118%, at 100% directionality. 0.457 is the
second-worst peak margin ever recorded on this device (lifetime worst 1.009×).

Nothing about this run was unusual. It was a routine single-floor automatic
move, and the release the product ships with came down to seven thousandths of
a m/s².

---

## 3. The four runs

All on `7210dfd`. Cab, automatic operation.

| # | move | dur | JOGV opk | arrival path | peak | ARM | ramp | dir |
|---|---|---|---|---|---|---|---|---|
| 1 | 4→1 down | 15.7 s | 16 | polled | 0.470 | q=37 a=1 v=1 | 607 | 100 |
| 2 | 1→2 up | 3.9 s | 36 | any-motion (2 edges) | 0.617 | q=6 a=1 v=1 | 615 | 100 |
| — | **2→1 down** | — | — | **MISSED — see §4** | — | — | — | — |
| 3 | 1→2 up | 3.9 s | 21 | any-motion (2 edges) | 0.457 | q=8 a=1 v=1 | 653 | 100 |
| 4 | 2→1 down | 3.4 s | 124 | polled | 0.713 | q=6 a=1 v=1 | 585 | 100 |

Every run correct end to end: latch, jog verdict `RUN`, arrival, release, back
to `STATE_MONITORING`. No failsafes. Three different arrival paths fired across
four runs (polled twice, any-motion twice).

**Jog verdict 4/4 `RUN`**, opk 16–124 — all far below the 900 gate. Lifetime
29/29.

**Up arrivals are weaker than down, again** (0.457/0.617 up vs 0.470/0.713
down). Consistent with every pairing measured to date; still a machine
property, not a defect.

---

## 4. 🔴 The lockup ate a whole run

Between runs 2 and 3, a 2→1 descent happened and the device did not see any
part of it.

    FSM: Transitioned to STATE_MONITORING     <- end of run 2, clean
    Device Booted / Reset cause: 0x8          <- lockup #1, watchdog
    FSM: Performing Self Calibration
    XY: calib b=10 peak=0.2680 mv=1
    FSM: NOT READY - recalibrating, attempt 2 <- came up MID-MOTION
    Device Booted / Reset cause: 0x8          <- lockup #2, watchdog
    XY: calib b=10 peak=0.0500 mv=0
    FSM: READY -> STATE_MONITORING            <- car already stopped

The car was unambiguously moving through the first post-reset boot: **26
printed samples with |w| > 0.1 rad/s**, the rolling average swinging
10.243 → 9.749, and `x` ranging 0.557–0.664 against a parked 0.61.

Calibration behaved **correctly** — `mv=1` means it detected motion and
refused to zero itself, which is the right call. The consequence is still that
the FSM never reached `STATE_MONITORING`, so no departure could be latched and
**the beacon never sounded for a real car movement**.

**This is a different class of failure from everything else in these notes.**
Every threshold, margin and gate discussed anywhere in Eng_Notes assumes the
device is running. Here it was not, and the product's whole premise failed
outright rather than marginally. Lifetime watchdog catches 2 → 4, and this is
the first that demonstrably cost a run.

Both catches came within ~20 s of each other, immediately after a run in which
the buzzer had been sounding — but the second happened during calibration with
the buzzer silent. That does not fit "alarm activity triggers it". It fits a
state the device got into and could not leave, which favours the wedged-TWI
explanation over the rail-sag one. Two events, so not conclusive.

Runs 3 and 4 afterwards had zero resets.

---

## 5. Arming margin: the bottom terminal, and a method error worth recording

| run | move | q= | notes |
|---|---|---|---|
| 1 | 4→1 down | **37** | armed near the top of the shaft |
| 2 | 1→2 up | **6** | single floor |
| 3 | 1→2 up | **8** | single floor |
| 4 | 2→1 down | **6** | single floor |

**On this mounting the bottom-terminal margin is 6 down and 8 up** — above the
5 required, and reproducible. Better than the `6b5b2c3` mounting (down 6,5,5,
one of which armed on the last possible sample) but not comfortable: a single
sample lost to buzzer blanking ends it, and losing arming switches off *both*
the peak collector and the ramp detector. That is the 78 s false-beacon
mechanism.

The direction asymmetry holds across both mountings: **down is the thin side.**

> ⚠️ **"REPRODUCIBLE" WAS WRONG — corrected the same evening by runs 6–7
> (§5a).** Two further bottom-terminal runs gave up q=**13** and down q=**9**,
> against the 6/8/6 above. Five runs on nominally the same mounting span
> **q=6 to q=13**. The worst observed is still 6, but 6 is a worst case, not a
> typical value, and the margin is not stable enough to characterise a mounting
> from three runs. **Sixth single-run-class claim overturned by the next
> measurement — and this one was mine, made hours earlier in this same note.**

⚠️ **Method error, corrected mid-session.** Before run 1 I predicted a 4→1
descent would test the bottom-terminal margin because it *ends* at the bottom.
It does not. `q=` is the high-water mark of the quiet stretch that armed the
run, and arming completes early — q=37 near the top against q=6 on a
single-floor move. **A long run banks its quiet samples before it ever reaches
the thin end.** The zero-margin case needs a short run *starting* near the
bottom. Runs 2–4 are the valid test; run 1 is not.

`v=1` on all four runs. **The reversal arming path has now gone ~24 live runs
without firing**, because the quiet path keeps succeeding — twice by exactly
one sample. It is promoted (`46d1a5d`) and gating the peak collector, and it
has still never been seen to work.

---

## 5a. Runs 6–7 — the bottom terminal again, and the margin moved

Both on `9d57c9a`, parked attitude `x≈0.644`.

| # | move | dur | opk | path | peak | margin | ARM | ramp |
|---|---|---|---|---|---|---|---|---|
| 6 | 1→2 up | 4.3 s | 12 | polled | 0.481 | 1.069× | **q=13** a=1 v=1 | 588 |
| 7 | 2→1 down | 3.8 s | 17 | polled | 0.468 | 1.040× | **q=9** a=1 v=1 | 596 |

**The margin went UP, and that is the finding.** Bottom-terminal margins on
this mounting now read up 6, 8, **13** and down 6, **9** — a span of 6 to 13
across five runs. §5's "reproducible" is withdrawn.

**Most likely cause: parked attitude drifted `x≈0.61 → 0.64` between the two
sets.** That is a small shift, but `6b5b2c3` measured `x 0.643→0.710` swinging
margins from 9,8,9 to 6,5,5 — so a small attitude change producing a
meaningful margin change is *consistent with*, and further evidence for, the
install-time-property finding. Run-to-run variance is the alternative and five
runs cannot separate them. **What this does settle: you cannot characterise a
mounting's margin from three runs.**

**⬜ Deliberately NOT claimed: the 0.32 cruise question is not reopened.** `pk`
reads 0.33 and 0.36 during `st=4` on these runs, which looks like it
contradicts §6a's 0.12. It almost certainly does not. These are ~4 s runs that
barely reach cruise, `pk` only accumulates after arming (late on a short run),
and those are one or two samples each — almost certainly the arrival ramp
rising toward its 0.45 crossing rather than cruise content. §6's 0.32 sat
across sustained cruise, which is a different measurement. The decimated log
cannot separate them, so this is UNRESOLVED, not evidence either way. Settle it
from the arrival burst of a long run, not from `pk` on a short one.

**`tw=3`** — a third wedge caught, still zero resets, correlating with the
`ACC-STAT read FAILED` after run 7's arrival burst. Three catches, three
transparent recoveries.

Seventh ramp latch, still post-release. `v=1` both.

---

## 5b. 🟢 The reversal gate works, including on short runs

Runs 9–12, on `74e2f1c` then `3993abb` (the `ramp_gate` fix and the `ro=`
instrument fix — see §11).

| # | move | peak | path | ARM |
|---|---|---|---|---|
| 9 | 4→1 down | 0.494 | polled | q=235 a=1 v=1 **g=1 ro=8** |
| 10 | 1→2 up | 0.551 | any-motion | q=15 a=1 v=1 **g=1 ro=14** |
| 11 | 2→1 down | 0.544 | any-motion | q=9 a=1 v=1 **g=1 ro=12** |

**The open question is answered: the reversal gate opens with margin on a ~4 s
single-floor run.** `ro=14` and `ro=12` against the 8 required — 1.75× and
1.5×. The risk in §11's fix was that requiring 44 samples instead of 36 would
starve the detector on exactly the short runs it exists for. It does not, and
the ramp latched on all three.

The two gates carry **comparable** margin, so reversal is not a marginal
substitute for quiet: quiet 3.0× and 1.8× (q=15, q=9 against 5) against
reversal 1.75× and 1.5×.

⚠️ Run 9's `ro=8` is NOT a margin reading — it predates the instrument fix and
is capped by construction. §11.

**Bottom-terminal margin, four runs each direction:**

| direction | q= | span |
|---|---|---|
| down | 6, 6, 9, 9 | **6–9** |
| up | 6, 8, 13, 15 | **6–15** |

Down is both thinner AND more stable; up is more variable. Sharper than §5a's
"6 to 13 overall" and consistent with down being the harder direction across
both mountings. Four runs each — hold loosely.

---

## 6. ~~Cruise peak reached 0.32~~ — DEAD, did not reproduce

> **RESOLVED the same evening, and the answer is no.** A second 4→1 descent
> (firmware `9d57c9a`, §6a below) put cruise at a **maximum of 0.12**, with 34
> of 35 MOVING samples between 0.02 and 0.07. The 0.32 does not reproduce, so
> **the peak detector is not being squeezed from below and the 1.47×
> separation claim is withdrawn.** The original reasoning is kept below because
> it was the basis for asking, and because the caveat matters: attitude
> differed slightly between the two runs (`x` 0.604–0.615 then, 0.635–0.645
> now), and cruise vibration is known to track mounting by 3×, so mounting and
> plain run-to-run variance are both live explanations. Either way 0.32 is not
> the cruise ceiling. **This is the fifth single-run number this project has
> overturned with the next measurement.**

On run 1 — the only run long enough to establish real cruise — the windowed
peak reached **0.32**. Previous cruise ceilings were 0.02–0.04 in the cab and
0.07–0.28 on the cartop, so this is the highest ever recorded, by 8× against
the cab figure.

The same run released on a peak of 0.470. So on one run the peak detector was
squeezed to a **1.47× separation between cruise content and the arrival that
released it**, from both ends simultaneously. The ramp detector sat at 2.02×
on the same data.

Single run, and the short runs 2–4 peaked at 0.02 because they never cruise —
so this is **neither confirmed nor contradicted**, and by the 08-10/11 method
lesson it stays a hypothesis until a second long run agrees. It wants one
multi-floor run, deliberately, to settle.

---

## 6a. Run 5 — the first run on the bounded-wait build, and the guard under load

4→1 down, automatic, 15.7 s, firmware `9d57c9a` (32056 bytes).

    JOGV pos=0 neg=31354 ratio=0 opk=38 verdict=RUN (armed)
    FSM: Arrival (polled), peak 0.493
    ARM q=228 a=1 v=1
    RAMP latched mean=599 dir=100

- **Cruise max 0.12** — kills §6, above.
- **Arrival 0.493 against 0.45, 1.096×.** Another automatic stop on its
  threshold. That distribution is settled beyond dispute now.
- **Ramp 599 at 100% directionality — fifth latch, again after the peak
  released.** The armed branch has still never executed.
- **`ARM q=228`** — confirms §5 emphatically. A long descent banks an enormous
  arming margin; the thin end is only visible on short runs.

**🟢 The TWI guard held under load: `tw` stayed at 2 for the whole run.** Zero
new trips across 15.7 s of real vibration with the buzzer sounding, while `ov`
grew 39→52. This matters for calibration: a spin bound set too tight would trip
under exactly these conditions and it did not. Combined with the two parked
trips (§8a), the reading is that 8000 spins is not marginal and the trips it
does record are genuine wedges.

Battery: `Voltage value : 2324 (settling, ignored)`. Still blind.

---

## 8a. 🟢 The guard caught two real wedges within 200 s, and the device never reset

Unplanned, and the most valuable result of the bench work. Parked and idle
after flashing `9d57c9a`:

    ACC-STAT read FAILED
    t=125566 ... et=1 tw=1
    t=130727 ... et=2 tw=2

**Two guard trips, zero `Reset cause` lines, device still sampling normally.**
Before this build each of those was a frozen ISR ending in a watchdog reset —
which is precisely what ate the 2→1 run in §4. Caught and recovered
transparently, with no interruption to the FSM.

**The ambiguity, and why it resolves toward "genuine wedge".** A trip could
mean the bus actually wedged, or that the spin bound is too tight and the guard
*caused* the failed read. Three pieces of evidence favour the former:

1. `ACC-STAT read FAILED` also occurred **twice before any trip**, while `tw`
   was still 0 — so read failures pre-exist the guard and are not manufactured
   by it.
2. Two trips in 200 s is intermittent, not the continuous tripping a too-tight
   bound would produce.
3. Zero trips across a full 15.7 s car run under load (§6a).

⚠️ Not fully closed: the ~50 ms figure for 8000 spins is cycle-counted, not
measured. The evidence bounds it as "not obviously too tight", which is weaker
than a calibration. If `tw` ever climbs quickly, measure before adjusting.

**The guard has moved from unproven to demonstrated in recovery, and remains
unproven only in the sense that no lockup has recurred to test whether it
prevents ALL of them.** If a freeze ever happens again *without* `tw`
incrementing, the cause is not the TWI waits and the brownout explanation
returns.

---

## 7. The armed branch has still never executed

**Seven ramp latches this session — 585, 588, 596, 599, 607, 615, 653, all at
100% directionality.** Combined with the prior 17 automatic stops the range is
now 472–653, still with zero false latches anywhere.

Not one of them released the latch. Every one landed *after* the peak or
any-motion path had already fired:

    FSM: Arrival (polled), peak 0.470
    ARM q=37 a=1 v=1
    FSM: Transitioned to STATE_DECELERATING
    RAMP latched mean=607 dir=100          <- ~0.5 s late

The FSM's ramp check lives inside `STATE_MOVING`; by the time the verdict
latches, the FSM has left. What appears in the log is `emit_ramp_log()` from
`loop()`, never `FSM: Arrival (ramp) … (armed)`.

**This is correct behaviour, not a defect.** The ramp path is insurance: it can
only release a stop the other paths miss, and no stop has missed yet. But after
**seven opportunities** it means arming remains **unexercised** — the same
position the reversal path has been in for ~27 runs. Two release paths are now
armed on the strength of replay and negative evidence, and neither has been
observed to fire in anger. Only the jog verdict earned its arming with live
evidence (25/25), and those two carry the 350 fpm boundary between them.

Run 3 is the closest it has come: 2% lower on that peak and the FSM would have
stayed in `STATE_MOVING` and the ramp would have released it.

---

## 8. 🟢 The TWI lockup is fixed, for 124 bytes

§4 made this the priority over any detection question. The flash census and
the fix:

**`-DWIRE_TIMEOUT` does not fit and never will cheaply.** Re-measured with
relaxation in place: **+1554 bytes** (not the +1818 on record — relaxation
shrinks the timeout code too), landing at 33454/32256 = 103.7%. Shortfall
1198 bytes. RAM cost is trivial (+12).

**Two dead ends, recorded so nobody re-treads them:**

- **No runtime-only escape exists.** `twi_timeout_us` defaults to 0 and looks
  like a runtime switch, but MiniCore wraps every check in
  `#if defined(WIRE_TIMEOUT)`. `Wire1.setWireTimeout()` cannot arm bounded
  waits because the code is not compiled at all.
- **`-DWIRE_TIMEOUT` also adds three function pointers to the shared `TwoWire`
  constructor**, so it cannot be defined for one vendored library without an
  ABI mismatch against the framework's `Wire.cpp`. This is the trap in "just
  define it for our copy".

`micros()` is already linked (the ISR read timing in `main.cpp`), so the 1554
is 32-bit compare arithmetic across seven wait sites, not the timebase.

**Flash census:**

| item | bytes |
|---|---|
| `bma456_config_file` | **6144** (19% of flash) |
| `main` (everything inlined) | 9750 |
| `read_acceleration_mss()` | 2482 |
| 119 `F()` log strings | ~1834 |
| `malloc` + `free` | 586 |
| `Print::print(double,int)` | 422 |

⚠️ **The bma4 swap cannot reclaim that 6144-byte blob.** It is the Bosch
feature-engine firmware image uploaded to the sensor, and any-motion — the
primary departure trigger — depends on it. The swap reclaims the driver *code*
around it, on the order of 1500 bytes. **The "swap frees ~4.9 KB" estimate is
wrong** and should not be relied on when scheduling it as an unblocker.

**What was done instead of freeing 1198 bytes: made the fix cheaper.**

| approach | cost | fits? |
|---|---|---|
| `-DWIRE_TIMEOUT` | 1554 B | no, short by 1198 |
| `-Wl,--relax` | **−314 B** | free, no source change |
| vendored Wire1 + spin guards | **124 B** (+32 logging) | yes, 200 B free |

MiniCore's Wire1 is vendored into `falcon_srcs/lib/Wire1` (project libs take
precedence — verified in the dependency graph, and the *unmodified* copy built
byte-identical at 31900, which is what proves the vendoring itself is inert).
Each of the seven wait sites got a 16-bit spin counter. No timebase, no 32-bit
arithmetic, no change to `TwoWire`. On expiry it calls
`twi_handleTimeout1(true)` — the identical recovery upstream performs,
re-initialising the peripheral and reapplying `TWBR1`/`TWAR1`. `twi_init1` sets
`twi_state = TWI_READY`, so the early return cannot leave the state machine
wedged.

`-flto` was tried and gained **exactly zero** — PlatformIO does not apply it at
the link step for this platform. Do not re-try it.

**Verified in the disassembly, not assumed.** All three loops in
`twi_readFrom1` are independently bounded (`sbiw`/`brne` into the guard at
`0x2dec`, `0x2e36`, `0x2e54`), with the spin count visible as
`ldi 0x40 / ldi 0x1F` = 8000. GCC merges the identical failure tails into one
call per function, which is why only three call sites appear for seven guarded
loops — that briefly looked like guards had been elided.

⚠️ **`TWI1_GUARD_SPINS` is a spin count, not a time.** 8000 spins is ~50 ms at
F_CPU 1 MHz against a normal ~11.5 ms transaction, deliberately far below the
2 s watchdog. It does not track F_CPU or the TWI bit rate — re-check if either
changes. Rollback is `TWI1_GUARD_SPINS 0`.

Trips log as `tw=`, printed only when nonzero, same discipline as `et=`. **A
trip is data:** it means a wedge was caught that would previously have become a
watchdog reset.

**Bench-verified after flashing 32056 bytes:** BMA456 init and configuration
complete (all I2C writes through the patched driver), self-calibration clean at
`Zero-Calib 9.771757` / `XY-Still 0.0555`, `READY` → `STATE_MONITORING`,
sampling normal, `rd=` unchanged at ~11.4 ms — so the guards add no measurable
read overhead — and **zero `tw=` trips on a healthy bus**, which is exactly
what a correct guard looks like when nothing is wrong.

**Still unproven:** nothing has tripped the guard, so the recovery path is
untested against a real wedge. Third armed-but-unexercised path on this device.

---

## 9. Next, in order

1. ~~A long multi-floor automatic run~~ — **DONE, §6a. §6 is dead.**
2. ~~Two or three more bottom-terminal descents~~ — **DONE, §5a, and the
   answer is that the margin is NOT stable: 6 to 13 across five runs.** What
   follows from that: the margin cannot be characterised from a handful of
   runs, so **the device should report its own margin** rather than anyone
   inferring it from a commissioning test. That is the change §5a argues for.
   `v=2` still never seen (~27 runs).
3. **Watch `tw=`.** Two trips recorded, both recovered (§8a). If the lockup
   recurs *without* `tw=` incrementing, the cause is not the TWI waits and the
   brownout explanation comes back. If `tw=` climbs fast, measure the spin
   bound rather than guessing at it.
4. **Arming-gate redesign** — key on "the departure ramp has ended" (sign
   reversal) rather than "the signal went quiet". Still the only fix for the
   single-floor blind spot, which arming the ramp detector did **not**
   address, because the detector shares the gate. Wants bench + replay against
   every burst on file, not one run.
5. **The ~20% snapshot overrun** (`ov` grew 40→68 across two short runs). Every
   threshold at 25 Hz still stands on 80% of the data.
6. Battery settling logic — still `(settling, ignored)` on every read
   (`2445` this session). Genuinely a bug, not settling.
7. Brownout electrical work, tabled by Dave; bench current measurement with a
   meter remains the unblocking step. The lockup fix may or may not have
   touched it — scope them together.

---

## 10. What this session overturned

- **"The bma4 swap must come first."** False for arming (2 bytes) and false as
  stated for `-DWIRE_TIMEOUT`, which turned out not to need the swap either —
  a cheaper fix existed. And the swap's own payoff is ~1500 bytes, not 4.9 KB.
- **"A 4→1 descent tests the bottom terminal."** My own prediction, wrong
  within one run. Arming completes near the top.
- **"`-DWIRE_TIMEOUT` costs 1818 bytes."** 1554 with relaxation.
- **"Arming the ramp detector improves the product."** Not demonstrably, yet.
  It is armed, it is correct, and it has changed nothing observable in five
  runs — because the path it protects has not been needed. The honest claim is
  that it removes a dependence on a 1.016× margin, and run 3 shows how thin
  that margin gets.
- **"Cruise reached 0.32, so the peak detector is squeezed from both ends."**
  My own hypothesis, contradicted by the very next long run at 0.12 (§6, §6a).
  Raised and killed inside one session.
- **"The bottom-terminal margin on this mounting is 6 down / 8 up, and
  reproducible."** Mine too, written earlier in THIS note and overturned by
  runs 6–7 at 9 down / 13 up (§5, §5a). Five runs span 6 to 13.

**Two of the six were my own claims, and one of them survived only a few hours
in its own note.** The standing method lesson — treat any single-run number as
a hypothesis until a second run under different conditions agrees — is not a
historical caution about earlier sessions. It applied twice today, to me, and
the correct habit is to write the confidence level into the note at the moment
the number is recorded, not after it fails.

---

## 11. 🔴 Arming the ramp detector opened a real hole, and the replay found it

`graph/arming_replay.py` replays candidate arming rules against all 186 bursts
on file (89 departure, 88 arrival), mirroring the ISR arithmetic exactly. Read
its header before quoting any number from it: departure-burst SAFETY is sound,
arrival-side capability is PARTIAL because bursts do not span whole runs.

**THE HOLE. A slow departure starts BELOW the quiet band.** From the latch, in
a real logged burst:

    -65 -38 -56 -64 -129 | -142 -170 -228 -301 -488 -496 -506 ...
    \___ all inside the 150 mm/s^2 quiet band ___/

The quiet path arms on the departure's own opening samples. From that instant
the ramp accumulator is fed the departure ramp, which qualifies at **mean 499,
directionality 100%** — arithmetically indistinguishable from an arrival. With
`RAMP_ARMED 1` that **releases the latch seconds after a car starts moving**:
§14.4's catastrophic direction. **1–2 departures in 89 on logged data.**

Until this session it was latent — unarmed, the verdict printed and nothing
happened. **Arming it is what made it reachable, and arming it was my change.**

**It survived only on an undesigned coupling.** The exposure vanishes only if
arming begins ≥200 ms after the latch, and `MOVEMENT_DETECTION_TIMEOUT_MS` is
exactly 200. But that is a **ceiling, not a floor**:
`STATE_MOVEMENT_DETECTED` exits early whenever `|w| > |vel_departure|`, and
`vel_departure` logs as `0.000` on every run. So the ramp detector's safety
rested on a debounce the header itself describes as having "bought nothing".

| arming starts after latch | union RAMPFIRE | union ARMED-INTO-RAMP |
|---|---|---|
| 0 ms | 2/89 | 5/89 |
| 40–80 ms | 1/89 | 3/89 |
| 120–160 ms | 0/89 | 1–2/89 |
| ≥200 ms | 0/89 | 0/89 |

**THE FIX (`74e2f1c`): the ramp gets its own reversal-only gate**, `ramp_gate`,
tracked independently of the union the peak uses. A sign reversal against the
departure means the departure ramp has ENDED — what the ramp detector actually
needs to know, and what "the signal went quiet" only approximates. The peak
keeps the union because it must arm readily; the thin end arms by one sample.
Replay: **zero departure-ramp fires at every arming offset 0–6**, arrival
capability **unchanged at 42/88**. Firmware is stricter than the replay
modelled — `arr_dep_sign` needs 25 samples, so `ramp_gate` cannot arm before
then. Verified live in §5b.

**`arm_via == 2` is NOT the fix, and I proposed it before checking.** `arm_via`
records whichever path armed FIRST, and the reversal block sits inside
`if (!arr_armed ...)`, so once quiet arms, `arr_opp` stops being evaluated and
`arm_via` can never become 2. Quiet has won that race in every run on file
(`v=1`, ~27 runs). Gating on it would have made the ramp detector **permanently
dead**.

**Cost:** the ramp now needs 44 samples (~1.76 s) of deceleration instead of
36. §5b shows that costs nothing on short runs; a very short deceleration
remains the thing to watch.

### 11a. Two methodology bugs in the replay itself, same root cause

The first version claimed the shipping firmware fires the ramp on **53% of
departures**. Thirty-plus live runs with zero false releases flatly contradict
that, which is what prompted the re-check. Both bugs were the same mistake:

1. Replaying a departure burst from sample 0 arms on the **parked pre-trigger
   samples**, which are quiet by definition.
2. The counters actually start at `STATE_MOVING` entry, not at the departure
   latch — `burst_trigger` fires at the latch, `arrival_peak_reset()` on MOVING.

**A replay that contradicts observed behaviour is wrong until proven
otherwise.** That check is what caught both.

### 11b. I reintroduced a bug this project had already fixed

The `ro=` counter added with the gate sat inside `if (!ramp_gate ...)` and so
stopped the instant the gate opened — **capped at `ARM_REV_SAMPLES` by
construction**, reading `ro=8` on every run where `g=1`, meaning only "it
armed".

That is **exactly** the defect already found and fixed in the `q=` instrument
(`2c3546a`), where a capped counter made "q=5 every time" mean nothing and cost
a withdrawn margin claim. I reintroduced it in a brand-new counter the same
afternoon, having read that note. Fixed in `3993abb` by mirroring
`arr_quiet_hi`/`arr_q_frozen`. Run 10's `ro=14` is the fix working; run 9's
`ro=8` is the bug.

**Corrected estimate, for the record:** I called it "~20 bytes". It cost **56**.
Flash is now **32226/32256 — 30 bytes free**, the tightest this firmware has
been. Anything further needs the bma4 work, which reclaims ~1500 bytes, not
4.9 KB (§8).

### 11c. Score for the session

Four of my own claims or artifacts needed correcting: the 0.32 cruise peak, the
"reproducible" bottom-terminal margin, `arm_via == 2`, and the capped `ro=`
counter. Each was caught — but by checking after the fact, and 11b was
avoidable by following the pattern sitting in the same file. **The habit that
actually worked was distrusting results that contradicted live behaviour
(11a).** The habit that failed was reading a note about a bug and then writing
it again.
