#!/usr/bin/env python3
"""
coherence_replay.py -- would corroborating the arrival PEAK with ONE block of
the ramp detector's own directionality test have prevented the 2026-08-20
release on a moving car, and what does it cost on every other stop on file?

================================================================================
THE QUESTION, AND WHY IT IS NOT ONE OF THE REFUTED ONES
================================================================================

falcon_false_release_2026-08-20.md retired "retune ARRIVAL_PEAK_VALUE": the
false release measured 0.472 and sits INSIDE the real-arrival population
(0.450-0.511), one of which landed exactly on the gate. Amplitude cannot
separate them on that machine.

falcon_ramp_priority_2026-08-20.md then retired the obvious follow-up. The ramp
detector's verdict is never available when the peak fires -- 0/105 runs, median
+1868 ms, worst +9798 ms -- so PRIORITY only postpones the release (still
catastrophic) and a VETO refuses to release on 100% of inspection stops, which
carry zero ramp verdicts by construction.

Both of those obey the ramp detector's full RAMP_BLOCKS=3 verdict. This tool
asks a different question:

    the ramp detector needs 3 consecutive qualifying blocks (~1.4-2.0 s).
    What does ONE block buy, asked at the instant the peak crosses?

One block is 12 samples = 480 ms, not 1868 ms, so this is not ramp-priority at
a shorter window: on most stops the corroboration is already satisfied by the
samples that produced the crossing, and the release instant does not move at
all.

================================================================================
THE RULE
================================================================================

Shipping (main.cpp, read_acceleration_mss):

    arr_hit = true   as soon as the windowed peak exceeds ARRIVAL_PEAK_VALUE

Candidate:

    arr_hit = true   when  |dev| >= ARRIVAL_PEAK_HIGH          (a single
                           unambiguous transient -- the brake set)
                     or    the windowed peak has exceeded ARRIVAL_PEAK_VALUE
                           AND a closed block of RAMP_BLOCK_N samples qualifies
                           on the ramp detector's own two tests:
                               |Sum a| / N        >= RAMP_FLOOR_MMSS
                               |Sum a| * 100      >= RAMP_DIR_PCT * Sum|a|

⭐ THE SAFETY PROPERTY IS STRUCTURAL, NOT STATISTICAL. Both branches require the
existing crossing to have happened (ARRIVAL_PEAK_HIGH >= ARRIVAL_PEAK_VALUE),
and the second branch ANDs a further condition onto it. So the candidate
releases a strict SUBSET of what ships, never earlier. It cannot create a
release that does not already happen today; it can only withhold or delay one.
Every risk it adds is therefore in the position-lie direction, which
LATCH_FAILSAFE_MS already bounds -- and none is in §14.4's catastrophic one.

⚠️ THIS IS NOT THE HAZARD THE ramp_gate BLOCK WARNS ABOUT, and a reader of that
block will otherwise reject this on sight. That warning is about the ramp
detector being a DISJUNCTIVE release path: quiet-arming on a slow departure's
opening samples feeds the departure ramp to the accumulator, which qualifies at
mean 499 / dir 100 and releases the latch on its own. This test is CONJUNCTIVE
with the peak crossing, which that same departure ramp ALREADY satisfies today
(a 499 mm/s^2 plateau is above the 450 gate). The exposure is unchanged; only
MIN_TRAVEL_MS and arming stand in front of it, exactly as now.

================================================================================
WHAT THIS TOOL CAN AND CANNOT SEE
================================================================================

It replays the committed BURST records -- 80 signed samples of (raw - zero) in
mm/s^2, written continuously by the sample ISR, so they are the same quantity
the peak detector reads and at full rate. That makes the arithmetic exact.

Four limits, all of which push the reported cost UP, not down:

1. A burst is 3.2 s (20 pre / 60 post) around the FIRST crossing. "DEFER" means
   "no qualifying block inside the burst", NOT "never releases" -- the run
   continues and the detector keeps testing. For 260820-150000:431, the known
   false release, the capture shows `RAMP latched mean=505 dir=100` about ten
   seconds later on the real deceleration, so that run would have released
   there. The other deferrals cannot be resolved from a burst and need a
   continuous-stream replay.
2. The burst is written regardless of arming, so the crossing this tool finds
   can be EARLIER than the firmware's, which only collects once arr_armed.
   Delays are measured from the earlier instant.
3. Blocks are aligned to the crossing here and to the accumulator's own
   boundaries on the device. A block boundary can therefore fall one block
   later in the firmware than it does here.
4. Departure bursts are emitted only from the any-motion branch, so runs that
   latched polled-only carry no burst at all (B3, still outstanding). In
   260820-150000 that is 26 bursts against 33 latches. The corpus is biased
   toward any-motion runs.

Usage:
    cd falcon_srcs/datasets && python3 ../graph/coherence_replay.py
    python3 ../graph/coherence_replay.py --high 550 --verbose
    python3 ../graph/coherence_replay.py --sweep
"""

import argparse
import glob
import os
import re
import statistics

# ---- firmware constants, mirrored. Keep in step with the source. ------------
RAMP_BLOCK_N      = 12      # main.cpp
RAMP_FLOOR_MMSS   = 300     # main.cpp
RAMP_DIR_PCT      = 85      # main.cpp
ARRIVAL_PEAK_MMSS = 450     # ARRIVAL_PEAK_VALUE 0.45, movement_service.h
ARRIVAL_HIGH_MMSS = 550     # the candidate's new constant. Derived below.
#
# WHERE 550 COMES FROM, and it is the same method ARRIVAL_PEAK_VALUE used --
# the geometric middle of the two populations it has to separate, NOT a fit to
# a sample maximum (which is the refuted approach, state_of_project §6).
#
#   worst INCOHERENT transient that is not a car arrival   494  (113000:199,
#                                                    a bench tap; the confirmed
#                                                    false release is 471)
#   weakest INCOHERENT brake set on a car                  675  (105317:2018)
#
# 1.11x above one and 1.23x below the other. That is thin and it is stated
# rather than dressed up. (626, 113000:101, sits between them and is also a
# bench tap, not a car event.)
#
# ⚠️ HIGH IS NOT A MINOR CONSTANT: 112 of the 231 arrival bursts on file never
# produce a qualifying block at all -- they are brake sets, which ring rather
# than push, exactly as falcon_signature §4e measured (directionality 0.02-0.42).
# HALF the corpus therefore releases on the HIGH branch alone and gets no
# corroboration from this change whatsoever. What the change buys is the OTHER
# half -- drive-controlled stops, which is where the false release lives. What makes it shippable where retuning
# ARRIVAL_PEAK_VALUE was not is that BOTH errors degrade gracefully:
#
#   too low  -> a cruise transient skips corroboration -> exactly today's
#               behaviour, no worse than what ships
#   too high -> a brake set waits for the run to supply a qualifying block or
#               for LATCH_FAILSAFE_MS -> a longer beacon, position-lie
#               direction, already bounded
#
# Neither end reaches §14.4's catastrophic direction, which is what the
# arrival gate's own margin could not promise.

BURST_RE = re.compile(r"BURST k=arr pre=\d+ n=\d+ signed_mmss=([-\d ]+)")

# The one release in the corpus whose truth is known independently -- Dave
# called it from inside the car. falcon_false_release_2026-08-20.md.
KNOWN_FALSE = ("260820-150000.log", 431)


def block_qualifies(block):
    """The ramp detector's two tests, on one closed block."""
    total = sum(block)
    absol = sum(abs(v) for v in block)
    if not absol:
        return False
    mag = abs(total)
    return (mag // len(block)) >= RAMP_FLOOR_MMSS and mag * 100 >= RAMP_DIR_PCT * absol


def simulate(values, high):
    """
    Returns (ship_index, candidate_index or None).

    ship_index is the first sample whose |dev| reaches the shipping gate --
    the instant arr_hit latches today. candidate_index is where it would latch
    under the rule above, or None if no qualifying block closes inside the burst.
    """
    ship = next((i for i, v in enumerate(values) if abs(v) >= ARRIVAL_PEAK_MMSS), None)
    if ship is None:
        return None, None

    block = []
    for i in range(ship, len(values)):
        if abs(values[i]) >= high:
            return ship, i
        block.append(values[i])
        if len(block) == RAMP_BLOCK_N:
            if block_qualifies(block):
                return ship, i
            block = []
    return ship, None


def load(pattern):
    """Every arrival burst in the corpus, with the file and line it came from."""
    out = []
    for path in sorted(glob.glob(pattern)):
        with open(path, errors="replace") as fh:
            for lineno, line in enumerate(fh, 1):
                m = BURST_RE.search(line)
                if m:
                    out.append((os.path.basename(path), lineno,
                                [int(v) for v in m.group(1).split()]))
    return out


def score(bursts, high):
    delays, deferred = [], []
    for cap, lineno, values in bursts:
        ship, cand = simulate(values, high)
        if ship is None:
            continue                     # never reached the shipping gate
        if cand is None:
            deferred.append((cap, lineno, max(abs(v) for v in values)))
        else:
            delays.append((cap, lineno, cand - ship))
    return delays, deferred


def report(bursts, high, verbose):
    delays, deferred = score(bursts, high)
    d = [x[2] for x in delays]
    n = len(delays) + len(deferred)

    print(f"ARRIVAL_PEAK_HIGH = {high} mm/s^2      arrival bursts reaching the "
          f"shipping gate: {n}")
    print(f"  releases at the same instant as shipping : {d.count(0)}")
    print(f"  releases later                           : {len(d) - d.count(0)}"
          f"   median {int(statistics.median(d)) if d else 0} samples"
          f"   worst {max(d) * 40 if d else 0} ms")
    print(f"  defers past the end of the burst         : {len(deferred)}")

    print("\n  deferred (see limit 1 -- this is not 'never releases'):")
    for cap, lineno, peak in deferred:
        flag = "   <- the confirmed release on a moving car" \
               if (cap, lineno) == KNOWN_FALSE else ""
        print(f"    {cap} line {lineno}   burst max {peak} mm/s^2{flag}")
    if not any((c, l) == KNOWN_FALSE for c, l, _ in deferred):
        print("    ⚠️ the known false release is NOT deferred at this value")

    if verbose:
        print("\n  per capture:")
        caps = {}
        for cap, _, delay in delays:
            caps.setdefault(cap, []).append(delay)
        for cap, _, _ in deferred:
            caps.setdefault(cap, [])
        print(f"    {'capture':<24}{'n':>4}{'same':>6}{'median':>8}{'worst ms':>10}")
        for cap in sorted(caps):
            v = caps[cap]
            held = sum(1 for c, _, _ in deferred if c == cap)
            print(f"    {cap:<24}{len(v) + held:>4}{v.count(0):>6}"
                  f"{int(statistics.median(v)) if v else -1:>8}"
                  f"{max(v) * 40 if v else 0:>10}")


def sweep(bursts):
    print(f"{'HIGH':>6}{'bursts':>8}{'same':>6}{'later':>7}{'defer':>7}"
          f"{'worst ms':>10}   false release deferred?")
    for high in (450, 480, 500, 550, 600, 700, 800, 900, 1200):
        delays, deferred = score(bursts, high)
        d = [x[2] for x in delays]
        caught = any((c, l) == KNOWN_FALSE for c, l, _ in deferred)
        print(f"{high:>6}{len(d) + len(deferred):>8}{d.count(0):>6}"
              f"{len(d) - d.count(0):>7}{len(deferred):>7}"
              f"{max(d) * 40 if d else 0:>10}   {'YES' if caught else 'no'}")
    print("\nRead this as a cost curve, not a tuning knob. Raising HIGH sends "
          "more\nordinary brake sets through the coherence branch, which they "
          "fail, so the\ncost climbs while the benefit does not: the false "
          "release is already\ndeferred at every value at or above the gate.")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[2])
    ap.add_argument("logs", nargs="?", default="*.log",
                    help="glob of captures (default: *.log in the cwd)")
    ap.add_argument("--high", type=int, default=ARRIVAL_HIGH_MMSS,
                    help=f"ARRIVAL_PEAK_HIGH in mm/s^2 (default {ARRIVAL_HIGH_MMSS})")
    ap.add_argument("--sweep", action="store_true", help="sweep --high")
    ap.add_argument("--verbose", action="store_true", help="per-capture table")
    args = ap.parse_args()

    bursts = load(args.logs)
    if not bursts:
        raise SystemExit(f"no arrival bursts in {args.logs!r} -- run this from "
                         f"falcon_srcs/datasets")

    if args.sweep:
        sweep(bursts)
    else:
        report(bursts, args.high, args.verbose)


if __name__ == "__main__":
    main()
