# 🔴🔴 THE BEACON RELEASED ON A MOVING CAR — 2026-08-20, contract speed

**In-car, automatic, 300 fpm, 4-floor building.** Build `build-2026-08-20` plus
`ARM_REV_SAMPLES` 15. Capture `falcon_srcs/datasets/260820-150000.log`.
Dave, in the car: **"released in travel."**

This is `main.cpp` §14.4's only catastrophic failure — silence while moving —
observed directly for the first time in the project's history.

---

## 1. What happened

```
t=149504  Departure latched (polled) ml=135757 q=4 dq=0     beacon ON
t=155254  FSM: Arrival (polled), peak 0.472                  BEACON OFF
t=155254  ARM q=5 a=1 v=1 g=0 ro=6
          ... 11 seconds ...
t=163414  any-motion ignored, re-arm blanking t=3146 q=0
t=163741  polled ignored, re-arm blanking t=3391 d=0.0569
t=166297  Departure latched (polled) ml=6029 q=0 dq=3 td=2654  beacon ON again
t=187056  FSM: Arrival (polled), peak 0.471                  released, car stopped
```

The release came **5.4 s into a run that lasted 20+ s.**

## 2. The car was unambiguously moving

Lateral `m` through the beacon-off window (at rest this metric reads
0.005–0.05):

| t | st | `m` | `a` |
|---|---|---|---|
| 155254 | 4 | 0.506 | 10.073 |
| 156499 | 5 | **0.740** | 9.774 |
| 160546 | 2 | 0.205 | 9.704 |
| 162463 | 2 | 0.455 | 9.917 |
| 164052 | 2 | 0.175 | **10.345** |
| 165986 | 2 | **0.884** | 9.936 |

**10× to 170× the rest level for eleven continuous seconds**, with the vertical
average swinging 9.5–10.4. There is no reading of this in which the car was
stationary.

⚠️ **And the log would not have told us on its own.** `FSM: Arrival (polled),
peak 0.472` is indistinguishable from a healthy release — the same line the five
correct runs printed. It was identified only because Dave said so from inside
the car. That is the 2026-08-12 §4.3 warning repeating: *"In the log it is
indistinguishable from a healthy release."*

## 3. 🔴 The cause: cruise transients have reached the arrival gate

`ARRIVAL_PEAK_VALUE` is **0.45**. During that run the polled peak climbed
through cruise:

```
0.17 → 0.21 → 0.31 → 0.34 → 0.47
```

Once the peak collector was armed — `q=5`, the minimum, via the quiet path
(`v=1`) — the first cruise transient over 0.45 released the latch.

**And this is not separable by threshold.** The six real arrivals recorded on
this build:

```
real arrivals   0.468  0.511  0.472  0.471  0.481  0.450
false release                 0.472   <- mid-travel cruise transient
```

**The false release sits INSIDE the real arrival population**, and one real
arrival landed at **0.450 — exactly the gate, 1.000×**.

- Raise the gate to reject 0.472 and **four of six real arrivals are rejected**,
  each of which then holds the beacon over a stationary car to
  `LATCH_FAILSAFE_MS` (600 s).
- Lower it and mid-travel releases become more frequent.

**There is no value of `ARRIVAL_PEAK_VALUE` that separates a stop from cruise on
this machine.** That retires "retune the arrival gate" the way 2026-08-18
retired "retune `ANYMOTION_THRESHOLD`", and for the same reason: the
distributions overlap.

This is the direct consequence of the cruise measurements taken earlier the same
day — vertical `cp` 0.27–0.29 on the cartop at 18 fpm, lateral `m` up to 0.649
at contract speed. Both said cruise on these machines is not quiet. This is what
that costs.

## 4. Was the `ARM_REV_SAMPLES` change responsible?

`ARM_REV_SAMPLES` 8 → 15 was flashed ~20 minutes before this run.

**Mechanically it is not implicated:**

- the run armed via the **quiet** path — `ARM ... a=1 v=1`, `arm_via = 1`
- it released via the **polled peak**, not the ramp
- the ramp gate was never open (`g=0`, `ro=6`)
- raising `ARM_REV_SAMPLES` can only make reversal arming **later**, never
  earlier, so it cannot have caused an earlier arm

**But the correlation is not dismissable on the evidence available:** 11 runs at
`rev=8` were clean, and the first run at `rev=15` failed. Two subsequent runs at
`rev=15` were correct (arrivals 0.481 and 0.450, car confirmed stopped by
lateral). So `rev=15` stands at 2 correct / 1 false release, `rev=8` at 11 / 0.

⬜ **Not reverted, on the mechanism.** The constant is one line and reverting is
available at any time. **This is explicitly a judgement on n=3 and should not be
quoted as a finding.**

## 5. What this does to the readiness position

Readiness item 2 has always read "automatic operation releases on luck — a
distribution centred on its own threshold". That framing described the *failure
to release*. **This is the same distribution failing in the other direction, and
that direction is the catastrophic one.**

- Item 3's blind-spot false beacon (85 s over a stationary car) is a **position
  lie** — bad, and detectable by anyone standing there.
- **This is silence while moving.** The mechanic ranging the counterweight by
  ear gets no warning. `main.cpp` §14.4 calls it the only catastrophic failure
  and it is now measured, not hypothesised.

## 6. What must happen before this car is trusted again

1. ⛔ **No further reliance on the beacon on this installation.**
2. **Re-derive the arrival gate against measured cruise, accepting that the two
   populations overlap.** A single peak threshold cannot do it. The candidates
   that remain are ones that use something other than magnitude — the ramp
   detector's sustained one-signed test is the obvious one, and it fired
   correctly on 11/11 stops today while never being the releasing path.
3. **Consider whether the peak detector should release at all** on machines
   where cruise reaches the gate, or whether release should require the ramp
   verdict. That is a design decision with a missed-departure cost on the other
   side, and it needs replay against the full corpus first.
4. ⬜ The `q=5` starvation (§2 of `falcon_automatic_2026-08-20.md`) is the other
   half of this: the peak collector armed on the minimum, mid-travel, because
   cruise never supplies quiet samples. **Arming earlier than the design intends
   is what exposed the gate to cruise in the first place.**
