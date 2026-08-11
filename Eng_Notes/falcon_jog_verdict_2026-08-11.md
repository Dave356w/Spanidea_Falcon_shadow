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

The three slow runs all released normally on the peak detector (arrival
peaks 2.575, 2.182, 1.012 — brake-set stops, the case that works).

## 2. The two populations

|  | ratio % | opk (mm/s²) |
|---|---|---|
| real departures, n=8 (this table's 3 slow + the 5× 350 fpm from the afternoon) | **0–20** | **27–396** |
| jogs, n=4 (blip to ~1 s, both directions) | **46–95** | **1914–3194** |

**No overlap on either axis.** Ratio gap 2.3× (20 vs 46); opk gap **4.8×**
(396 vs 1914). And the shortest blip — the direction a miss was expected
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
2. ⬜ One more cartop session of passive JOGV lines — jogs, slow runs, and
   ideally a mid-speed (~100 fpm) departure, the untested gap between the
   slow runs and 350 fpm.
3. ⬜ If nothing crosses, write the release wiring: verdict JOG →
   silence + return to MONITORING, ~4 s after the latch. The jog defect
   then costs one short chirp instead of a 240 s alarm.

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
