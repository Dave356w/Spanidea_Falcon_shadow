# The flash budget: two wrong numbers, a build that did not fit, and 642 bytes

**2026-08-21 afternoon. Bench machine, device attached, COM5 log / COM7 ISP.**
**Flashed and verified on the m328pb; no car.**

`Falcon_Rel_EFT` at `4c253b7` did not build. Three of five environments
overflowed, including **the one that ships and the one every test session must
use**. Nothing was wrong with the lateral corroboration term that §4.2 added —
its cost estimate was sound to within 6%. What was wrong was the **budget it
was charged against**, and the budget was wrong because two notes in a row
quoted a committed table instead of running the compiler.

Fixed in `ce28d9d`, out of the log plumbing. §4.2 is untouched and ships armed.

---

# 1. What was broken

| env | `345613f` | `4c253b7` | free before | result |
|---|---|---|---|---|
| `ATmega328PB` (FALCON_LOG 2) | 32176 | 32754 | 80 | **498 over** ⛔ |
| `production` (FALCON_LOG 1) | 31706 | 32328 | 550 | **72 over** ⛔ |
| `bench_battery` | — | 32440 | — | **184 over** ⛔ |
| `production_silent` (FALCON_LOG 0) | 25590 | 25842 | 6666 | ok |
| `idle_current` | — | 26118 | — | ok |
| `brownout_test` | 20458 | 20458 | — | ok |

`ATmega328PB` is the FALCON_LOG 2 build — the one every threshold on file was
measured on, and the one `falcon_log.h` says every test session must use. It
was 498 bytes past the end of flash. **There was no bench session to be had.**

# 2. The two wrong numbers

**The first**, in `falcon_fsm_logic_and_shortcomings_2026-08-21.md` §S10 as
originally written: it quoted `falcon_START_HERE_2026-08-20.md` §1's "80 free"
and called the build tight. That was **correct**.

**The second was the correction.** §S10 was amended the same day to retract the
80 and promote `platformio.ini`'s table — 706 / 1134 / 6902 free — as "the
authority", concluding "§4 fits". That is backwards, and it is what broke the
build.

The arithmetic was sitting in START_HERE §2 the whole time:

```
  platformio.ini table, measured at b79298c   706 free on ATmega328PB
  B1 instrumentation spent afterwards        -626   (START_HERE §2, verbatim)
                                            ------
                                               80   = START_HERE §1
```

A baseline build of `345613f` on this machine reproduces START_HERE **to the
byte on all three environments**: 32176 / 31706 / 25590. The committed table
was one day and 626 bytes stale.

## 2.1 The transferable part

The estimate was *fine*. §4.2 predicted **+588 B**; whole-image measurement
gives **+578** on `ATmega328PB` and **+622** on `production`. Nobody
mis-estimated the change. **An estimate is only as good as the budget it is
charged against**, and both notes reached for a table because building was
inconvenient — `api.registry.platformio.org` is blocked in the cloud
environment those notes were written in, so a whole-image `pio run` genuinely
was not possible there. The note said so. It did not say it loudly enough, and
then it treated a committed table as an authority anyway.

⭐ **The bench machine has the platform cached. `pio run -e <env>` takes three
seconds and cannot be stale.** `platformio.ini` now says so where the table is.

Two second-order corrections fall out of the same measurement:

- **`SL: rel` is not 173 bytes, it is 80–84.** The stub-`Arduino.h` method
  over-counts, because it cannot see what the linker shares.
- **`-flto` is not inert on this platform.** `platformio.ini` records that it
  "gained EXACTLY ZERO" and should not be retried. The object files in
  `.pio/build/*/src/` are slim LTO bytecode — `.text` of size 0 — so it *is*
  running at link time. The 2026-08-12 experiment measured "adding `-flto`" to
  a build that already had it. Harmless, but that line should not be read as a
  statement about what LTO is worth here.

# 3. Where 642 bytes were hiding

**Not in the FSM.** Every byte came back out of the log plumbing, from two
mechanisms that are invisible at the call site:

1. **`F()` is `PSTR()`, and `PSTR()` emits a FRESH static array at every
   expansion.** The compiler never merges them. `F("\r\n")` appeared 27 times
   and was stored 27 times. `F(" q=")` five times. The six
   `FSM: Transitioned to STATE_…` lines stored the same 27-character prefix six
   times over.
2. **`FLOG.print` is a MEMBER call.** It loads `Serial` into a register pair as
   well as the argument. The log is built almost entirely out of
   `FLOG.print(F(" q=")); FLOG.print(x);` pairs, and each such pair costs about
   twenty bytes at every one of the ~94 sites.

| change | what it does | saved |
|---|---|---|
| `flog_kv` / `flog_kvf` / `flog_kvt` | folds 94 label+value pairs into one call each | 222 |
| `flog_nl()` | 27 copies of `F("\r\n")` → one function, one `rcall` per site | 164 |
| 15 shared `PROGMEM` fragments | `" q="`, `" tw="`, the divider, … stored once | 112 |
| `flog_state()` | six lines share one 27-byte prefix | 122 |
| `flog_fixed()` | retires `Print::print(double, int)` | 54 + **12 B RAM** |

| env | before | after | free | RAM |
|---|---|---|---|---|
| `ATmega328PB` | 32754 ⛔ | **32112** | 144 | 1493 |
| `production` | 32328 ⛔ | **31746** | 510 | 1498 |
| `bench_battery` | 32440 ⛔ | **31838** | 418 | 1504 |
| `production_silent` | 25842 | 25854 | 6402 | 1162 |

**The RAM is a bonus and it is worth understanding.** Arduino's `printFloat`
reaches `print("nan")`, `"inf"` and `"ovf"` as *plain* literals, and a plain
literal on AVR lives in `.data` — i.e. in SRAM. Retiring it returned those 12
bytes. `flog_fixed()` names them with `F()`.

## 3.1 Three helpers, not one, and why

`flog_kv()` takes a **`long`**, because `Print` promotes every integer type to
that anyway, so the output is unchanged — and because a `const char *` or an
`__FlashStringHelper *` will **not** convert to it implicitly. A string-valued
site is therefore a compile error rather than a silently mangled log line.
Two sites are genuinely string-valued (`BURST k=`'s `dep`/`arr` ternary, and
`ACC-INT`'s `name`) and two more carry `HEX`; all four were left as they were.

`flog_kvt()` exists for a **raw `millis()` stamp**. A `uint32_t` crosses 2^31
after 24.9 days, so `HEALTH t=` on a unit that has been up a month would print
negative — and a unit on a counterweight is up for months. **Deltas are small
and signed and belong on `flog_kv()`; only a raw stamp needs `flog_kvt()`.**

# 4. ⚠️ What is NOT bit-identical

**`flog_fixed()` does not always agree with the routine it replaces, and the
difference is toward the correct value.**

Arduino builds its rounding term by float division, adds it to the whole number
**before** splitting off the integer part, then extracts digits by repeated
multiply-and-truncate. Accumulated error can then eat the last digit:
`39.5205` at 3 dp printed `39.520` and now prints `39.521`.

`flog_fixed()` splits the integer part first — `v - (float)(uint32_t)v` is
exact by Sterbenz — pulls digits one at a time, and rounds at the **end**, on
what is left over. Measured against exact decimal rounding over the real range
of all 21 sites:

| | disagrees with correct rounding |
|---|---|
| Arduino `printFloat` | **0.02 – 45%** |
| `flog_fixed()` | **0.000 – 0.36%** |

The 45% is `Zero-Calib-Value` at 6 dp. At ~9.7 a float's ULP is **9.5e-7 — one
whole count of the sixth decimal** — so Arduino was trying to add a rounding
term smaller than the number can represent. **That digit was always noise.**
The BMA456 resolves 6e-4 m/s² at ±2g, four orders coarser, and every consumer
in `graph/` parses with `float()` on a plain decimal pattern.

An infinity now prints `ovf` rather than `inf`. Neither has ever appeared in a
capture, and nothing in `graph/` distinguishes them.

# 5. How it was verified

**Offline, before flashing** — "log-preserving" is the whole claim, so it was
tested rather than asserted:

- the **string pool of `.text`** was extracted from the old image and the new
  one and diffed. The only difference is the intended one: six full transition
  strings out, the shared prefix and six bare state names in.
- **all 132 log literals in the source are present in the new image.** The 17
  that are not were absent from the old image too — `#if`'d out.
- **`Print::print(double, int)` is gone from the symbol table.** This is the
  load-bearing check: one surviving `print(x, n)` pulls all 416 bytes back in
  and the saving silently evaporates.
- **every changed line in `movement_service.cpp` is a log line.** The FSM is
  untouched.

**On the wire**, flashed to the m328pb (32112 bytes written and verified) with
the capture running:

- both boots are in one capture file — the old firmware at line 12384 (reset by
  the signature read) and the new at 12821 — so this is an **interleaved A/B on
  one bench**, which is what 2026-08-20 learned to insist on.
- with every number masked, **the set of distinct line shapes is identical**.
  Nothing only-in-old, nothing only-in-new. The residual line-by-line diff is
  one extra `ACC-INT` and one `ACC-STAT`, i.e. interrupt timing.
- `graph/parse_falcon_log.py` reads both: **0 corrupted lines**, same fields,
  same rate, same event extraction.
- the calibration block, which carries the widest formats, reads
  `Zero-Calib-Value : 9.739123` (6 dp), `Threshold-Value  : 0.040000` (6 dp),
  `XY-Still-Value   : 0.0720` (4 dp), between two `FS_RULE` dividers.
- a bench tap exercised the departure path:

```
FSM: Transitioned to STATE_MOVEMENT_DETECTED
FSM: Departure latched (any-motion) ml=236650 q=0 dq=0 td=-1
FSM: departure velocity w=0.000
FSM: Transitioned to STATE_MOVING
BURST k=dep pre=20 n=80 signed_mmss=-26 -2 -9 1 -15 7 -11 -3 1 -8 …
JOGV pos=1910 neg=647 ratio=33 opk=492 verdict=RUN (armed)
```

  Those six lines exercise `flog_state()`, the B1 `flog_kv` chain, a 3 dp
  `flog_kvf`, the deliberately-unfolded `k=dep` ternary and the shared
  `FS_ARMED` fragment all at once, and every one of them reads correctly.

- **the release path then completed, twice.** Both new `SL:` lines print:

```
FSM: Transitioned to STATE_DECELERATING
SL: held t=2326 m=0.072 q=0 xs=0.0720 (armed)
BURST k=arr pre=20 n=80 signed_mmss=-66 -484 14 -4 0 -2 -8 -10 9 -10 …
SL: rel mx=8.412 q=108
FSM: Transitioned to STATE_STOPPED
FSM: Transitioned to STATE_MONITORING
```

  🟢 **And §4.2 is not inert on this bench: `SL: held` fired.** The lateral
  term restarted the confirm window at `q=0` — the beacon was held, by the
  new code, exactly as designed. `mx=8.412` against `xs=0.0720` is a contrast
  of ~117×, which is what a hand tap looks like and is far above anything a
  car will produce; it says the plumbing works, **not** that the veto has grip
  at 300 fpm. That is still a car measurement and it is still owed.

  All six `flog_state()` transitions have now printed — MONITORING,
  MOVEMENT_DETECTED, MOVING, DECELERATING, STOPPED and back to MONITORING —
  across **four complete departure-to-release cycles**, after which the device
  is back in `STATE_MONITORING`, quiet at `q=255`, beacon off.

## 5.1 🟠 An unplanned finding: `SL: rel` is already earning its 84 bytes

The four releases report:

```
  SL: rel mx=8.412 q=108        contrast 117x   against xs=0.0720
  SL: rel mx=5.266 q=108                  73x
  SL: rel mx=3.068 q=106                  43x
  SL: rel mx=0.075 q=108        contrast 1.04x  <-- no grip at all
```

`mx` is the worst lateral seen while travelling and `xs` is `XY_STILL`; their
ratio is what says whether the lateral veto can discriminate on a given run.
**One of four bench runs came in at 1.04×** — inside the noise, where the term
is inert and the release falls back to the vertical band alone, which is
exactly the 2026-08-09 low-contrast failure mode the `STOP_LATERAL_QUIET`
block warns about.

⚠️ **Do not read this as a rate.** Four hand taps on a bench are not four car
runs, and a gentle tap producing little lateral is what you would expect. The
point is narrower and it is good news: **the diagnostic works.** The line that
was added to make grip visible made a no-grip run visible on its first
outing, and the same column will answer the question directly on a car.

⚠️ **Do not read the bench noise figures as an A/B result.** The parser reports
sd 0.0068 → 0.0116 and max `d(avg4)` 0.0410 → 0.1060 across the two boots. The
windows are minutes apart on a bench that was being handled, and 2026-08-20's
`RollingAvg` false-regression is exactly this trap
(`falcon_session_a_2026-08-20.md` §3). Nothing in this change touches the
sample path.

# 6. What is still open

1. ⚠️ **144 bytes free on the test build is not headroom**, by this project's
   own rule that 80 is not. The next instrumented line has to be paid for.
   **`-mcall-prologues` is worth a further 228 bytes** (31884 measured) and is
   deliberately **not** enabled: it changes codegen for every function
   including the ISRs, at 1 MHz, and that deserves its own measurement rather
   than a free ride in a logging commit.
2. **The `MCP3208 adc` global in `main.cpp` is dead** — `adc.begin()` is
   commented out, `read_battery_voltage()` is gone, nothing references it, and
   the build is 46 bytes smaller without it. Left in place: removing a hardware
   object should be its own decision, not a byte-scavenging side effect.
3. **`falcon_srcs/graph/capture.py` is still uncommitted** on the bench
   machine. It is the hardening that survives a CP210x re-enumeration and keeps
   one session in one file — and it has now been running continuously since
   2026-08-20 15:45, which is how both boots landed in one capture. It should
   be committed.
4. ✅ **Closed during the write-up** — the release path completed twice on the
   bench, both `SL:` lines printed, and all six `flog_state()` transitions
   have now been seen on the wire. See §5.
5. ⛔ **None of this touches §0 of the handover.** The beacon released on a
   moving car on 2026-08-20 and the standing instruction not to rely on it at
   that installation is unchanged by a flash-budget fix.
