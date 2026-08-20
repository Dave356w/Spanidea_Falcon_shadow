# Jog verdict: first real populations — 2026-08-11 evening, cartop tethered

Same-day follow-up to `falcon_350fpm_automatic_2026-08-11.md` §4 and
`falcon_signature_2026-08-11.md` §5.1. The log-only impulse-pair classifier
(`440a987`) got its first real data: **four jogs and three slow departures,
captured from the cartop with programmer and serial attached.** Log:
`device-monitor-260811-144408.log` (single boot, all events one section).

## 1. The complete JOGV record

| # | event | pos | neg | ratio % | opk | verdict | correct? |
|---|---|---|---|---|---|---|---|
| 1 | handling bump (bench) | 221 | 1254 | 17 | 221 | RUN | ✔ safe direction |
| 2 | 1 s jog | 6689 | 14263 | 46 | 1986 | RUN | ✘ missed (46 < 50 gate) |
| 3 | ~1 s jog + release | 9025 | 5584 | 61 | 3194 | JOG | ✔ |
| 4 | brief jog up | 8686 | 7034 | 80 | 3026 | JOG | ✔ |
| 5 | slow departure, down | 0 | 7100 | 0 | 111 | RUN | ✔ |
| 6 | slow departure, down | 1145 | 8867 | 12 | 396 | RUN | ✔ |
| 7 | slow departure, up | 6189 | 1276 | 20 | 338 | RUN | ✔ |
| 8 | shortest blip jog | 8914 | 8529 | 95 | 1914 | JOG | ✔ |
| 9 | 100 fpm departure, down (gap probe, after retune) | 1154 | 9558 | 12 | 440 | RUN | ✔ |

The three slow runs all released normally on the peak detector (arrival
peaks 2.575, 2.182, 1.012 — brake-set stops, the case that works).

## 2. The two populations

|  | ratio % | opk (mm/s²) |
|---|---|---|
| real departures, n=9 (3 slow + 100 fpm + 5× 350 fpm) | **0–20** | **27–440** |
| jogs, n=4 (blip to ~1 s, both directions) | **46–95** | **1914–3194** |

**No overlap on either axis.** Ratio gap 2.3× (20 vs 46); opk gap **4.8×**
(396 vs 1914).

> ⚠️ **SUPERSEDED 2026-08-20 — the no-overlap finding was true of one evening
> and does not survive the labelled corpus.** Across 63 labelled real
> departures and 17 labelled jogs: real **11–562 plus one at 972**, jogs
> **381–4154**; the populations overlap on **381–972**, and ratio overlaps on
> **44–58**. Everything below about *mechanism* stands — opk is still the
> primary axis and the brake-set jolt is still what it measures. What does not
> stand is the gap. `falcon_corpus_labelled_2026-08-20.md` §4.2. And the shortest blip — the direction a miss was expected
from — was the *most* jog-like event of the day (ratio 95: near-perfect
impulse cancellation, because there is no cruise segment to skew it).

## 3. What the data corrected in the design

- **Opk is the primary axis, ratio the supporting check** — reversed from
  the design guess. Dave's mechanism explains it: jog control from the
  cartop runs *drive energizes → brake releases → possible rollback → run
  enable → brake sets without motor field control*. That uncontrolled brake
  set hammers 1.9–3.2 m/s² into the structure on every jog regardless of
  direction or duration. No real departure exceeded 0.4.
- **The ratio-primary design would have failed**: jog #2 came in at 46
  against the 50 gate. The synthetic jog (89%) modeled a clean impulse
  pair; a real jog is an oscillatory shake where both signs accumulate.
- **Real slow departures carry opposite-side content** (opk up to 396 —
  rollback at brake release), which is what pushed the peak gate up from
  450, not the jogs.
- **Gates retuned to the geometric mean: `JOG_OPP_RATIO_PCT` 50 → 33,
  `JOG_OPP_PEAK_MMSS` 450 → 900.** Real departures pass 2.3–2.7× below,
  jogs 1.4–2.2× above, on both axes simultaneously. Against the new gates
  the table scores 8/8 (the handling bump correctly RUN on both axes).

## 4. Arming path — NOT yet armed

`JOG_VERDICT_ARMED` stays 0. n=4 jogs is a real population but a small one,
and the false-JOG failure (beacon silenced on a moving car) is the one
failure this product exists to prevent. Plan, per protocol §3.1:

1. ⬛ Retuned gates flashed, log-only (this commit).
2. ⬛ Mid-speed gap probed same session: 100 fpm down departure read
   RUN at ratio 12 / opk 440 against the retuned gates — 2.0× under the
   peak gate (rollback content grows slightly with speed: 396 → 440), and
   released normally (arrival peak 0.933). One more passive session of
   ordinary use remains the arming bar.
3. ⬛ **ARMED 2026-08-11, same day, on Dave's decision.** The
   recommendation was one further passive session; the concern (n=4 jogs,
   one car, one day) was raised, reaffirmed, and overruled — recorded
   here and in the code comment. Release wiring: verdict JOG →
   `MovementService::jog_release()` → accepted only in STATE_MOVING,
   consumed next `fsm_run()` pass through the failsafe's own exit
   (STATE_DECELERATING), flag cleared on every MOVING entry so no stale
   verdict can leak into a later run. A jog now costs a ~4 s chirp
   instead of a 240 s alarm. JOGV lines print `(armed)`. Flashed and
   verified, 31658 bytes. **If a false JOG ever silences a moving car:
   `JOG_VERDICT_ARMED` back to 0 first, diagnose from JOGV lines after.**

## 5. Session notes

- Voltage telemetry worked this session (8 valid prints) — the afternoon's
  all-"settling, ignored" blindness did not reproduce after a clean
  battery-powered boot. Not yet explained; watch whether it correlates with
  serial-attach order or back-powering.
- Pack trend: 2333 (morning) → 2240 avg (evening) after a day heavy with
  failsafe alarms. Above the 2000 trip, but cells collapse under load —
  fresh cells before the next long session.
- The handling bump (row 1) latched the alarm with zero car movement and
  ran the full failsafe — the defect demonstrating itself on the bench,
  unprompted, as the session opened.

## 6. Late-evening cartop session: jog boundary, ratio-39, and the gentle floor

Continued same evening from the cartop tether. Two more logs
(`device-monitor-260811-144408.log` post-reflash section — NOTE this file
spans two boots, split on `Device Booted` before analysing — and
`device-monitor-260811-152209.log`).

### 6.1 The rest of the JOGV record

| # | event | ratio % | opk | verdict | correct? |
|---|---|---|---|---|---|
| 10 | 2 s medium jog | 98 | 3467 | JOG | ✔ |
| 11 | 27 fpm departure, up | **39** | 251 | RUN | ✔ **opk gate saved it** |
| 12 | 27 fpm departure, down | 0 | 98 | RUN | ✔ |
| 13 | 125 fpm departure, up | 0 | 137 | RUN | ✔ |
| 14 | hard-stop run departure, down | 15 | 260 | RUN | ✔ |
| 15 | 125 fpm departure, down | 1 | 214 | RUN | ✔ |

15/15 lifetime. ⚠️ *(Off by one: `260811-152209.log` carries a **fourth**
`JOGV` line — `pos=11815 neg=0 ratio=0 opk=99`, log line 97 — that this table
never picked up. An uncounted RUN verdict, so the tally errs harmlessly; every
lifetime figure downstream of it inherits the same off-by-one. Found
2026-08-20.)* Two findings of consequence:

- **The 2 s jog closed the boundary question**: 98% cancellation, hardest
  brake shock recorded (±4.2 ringing). The jog band blip→2 s all reads JOG;
  ≥3 s exits the blind window into normal arrival detection. Armed, every
  movement-duration class has a release path — no residual gap.
- **Row 11 is the false-JOG near-miss the AND design exists for.** A slow
  up departure with heavy rollback crossed the ratio gate (39 vs 33) and
  was held RUN only by opk (251 vs 900). Real departures now span ratio
  0–39 against jogs' 46–98 — a 7-point gap. **The ratio axis is dead as an
  independent discriminator; opk carries the verdict** (real ≤ 440, jogs
  ≥ 1914, 4.4×). Keep the AND, but any future retune moves the opk gate,
  not the ratio gate.

### 6.2 Gentle arrivals: the floor is 0.575 and it is a mechanism

Terminal approaches at 125 fpm force the controller through its leveling
transition, producing a brake set from crawl speed — the §14.7 shape:

| stop | peak |
|---|---|
| hard cartop brake sets (4 this session) | 1.215 – 4.391 |
| **terminal leveling stop, 125 fpm up** | **0.579** |
| **terminal leveling stop, 125 fpm down** | **0.573** |

The arrival distribution is bimodal. The soft mode is tightly reproducible
(0.006 apart, opposite directions) and moves the measured floor **0.713 →
0.573**, margin over the 0.45 gate **1.58× → 1.27×**. At 27 fpm no leveling
transition occurs (the car is already below leveling speed) — the soft mode
needs an approach fast enough to force the transition. The release still
rests on a fixed mechanism (brake set from leveling speed), not headroom:
**a controller with a finer crawl will land lower — check per installation.**

### 6.3 Session incidents

- **Third device lockup of the day** (log froze parked, device silent,
  avrdude signature read revived it). **One 125 fpm down run was lost** to
  it and re-run. The monitor was also not restarted after the retune
  reflash, so `…144408.log` spans two boots — the standing trap.
- Pack ended at avg 2227 with one raw print at 2212. Replace before next
  session.

## 7. Armed verification — five jogs, five releases, zero failsafes

Same evening, immediately after arming (`de4844c`), cartop tether. Log
`device-monitor-260811-154018.log`. Every jog produced the full sequence
*latch → beacon ~4 s → JOGV (armed) → `FSM: Release (jog verdict)` →
DECELERATING → STOPPED → MONITORING*, confirmed audible-correct by Dave:

| jog | ratio % | opk | released |
|---|---|---|---|
| 1 | 88 | **1366** | ✔ |
| 2 | 82 | 3508 | ✔ |
| 3 | 65 | 4154 | ✔ |
| 4 | **44** | 1811 | ✔ |
| 5 | 97 | 4154 | ✔ |

Lifetime: **20/20 verdicts, 5/5 armed releases.** Updated populations
(jogs n=10, real departures n=9):

- **Jog opk floor moved on the first armed test: 1914 → 1366** (1.52× over
  the 900 gate). Real ceiling 440 (2.05× under) — ⚠️ **the real ceiling is
  562 on labelled data, and one run reached 972; see the box in §2.** The gate
  still splits the
  gap, but the floor drifted on day one — exactly what the passive-session
  recommendation existed to watch for. A jog under 900 fails SAFE (RUN →
  failsafe runoff, the old behaviour), so the cost of drift is a missed
  chirp, not a silenced moving car.
- **The ratio axis is finished: jogs now reach down to 44 vs real
  departures' 39.** Five points. It survives only as the AND guard, and
  the asymmetry is what matters: crossing it downward makes a jog fail
  safe, while the opk gate alone protects the dangerous direction.

Watch items for the next sessions, in the armed regime: any JOGV on a real
departure with opk approaching 900 from below (the 27 fpm rollback shape,
worst seen 440), and any jog under 900 (worst seen 1366).
