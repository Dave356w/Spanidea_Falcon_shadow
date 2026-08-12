#!/usr/bin/env python3
"""
arming_replay.py -- replay candidate ARRIVAL ARMING rules against every burst
on file, to answer the question the 2026-08-12 notes left open:

    can the arming gate be moved from "the signal went quiet" to "the departure
    ramp has ended" WITHOUT letting the ramp detector fire on a departure ramp?

WHY THIS MATTERS. The ramp block test cannot tell a departure ramp from an
arrival ramp -- same shape, opposite sign, and sign encodes direction of travel
rather than phase of the run. `arr_armed` is the ONLY thing standing between the
ramp detector and releasing the beacon seconds after the latch, on a moving car.
But the same gate blinded BOTH detectors on a single-floor terminal approach
(departure ramp straight into deceleration, no cruise, no five quiet samples):
pk read 0.00 across a real +0.64 excursion and the beacon sounded 85 s over a
car stationary for 78 of them. So the gate is simultaneously load-bearing and
the blind spot, and any replacement has to be checked against both failure
directions at once.

════════════════════════════════════════════════════════════════════════════════
WHAT THIS TOOL CAN AND CANNOT ESTABLISH -- read before trusting any number
════════════════════════════════════════════════════════════════════════════════

Bursts are 80-sample windows around a trigger, not whole runs. Between a
departure burst and the matching arrival burst lies unlogged cruise, and in the
real firmware the arming counters run continuously across that gap. So:

  ✅ SOUND: departure-burst SAFETY. A departure burst contains the departure
     ramp at full rate from the trigger onward. If a rule arms inside that
     window and the ramp accumulator then qualifies, the rule is unsafe --
     that conclusion needs no information from outside the burst.

  ⚠️ PARTIAL: arrival-burst CAPABILITY. Replaying arrival arming needs the
     run's departure sign, which is fixed from the run's opening samples and
     cannot be recovered from an arrival burst alone. This tool pairs each
     arrival burst with the preceding departure burst in the same log to supply
     it. Where no pair exists the arrival burst is reported as UNPAIRED and
     excluded from capability claims.

  ❌ NOT ANSWERABLE HERE: whether a rule arms in time on a real single-floor
     run. That depends on the continuous stream across the whole run, and the
     sample log is decimated to ~11%, so the 25 Hz stream does not exist off
     the device except inside bursts. Settling that needs either a burst that
     spans an entire short run, or a firmware change to dump continuously.
     DO NOT read a "would have armed" here as "would have armed in service".

Known data defect, carried from falcon_cab_automatic_2026-08-12 §5:
BURST_POST_ARR was defined in two files until 2026-08-12, so some 2026-08-11
arrival bursts were captured 60-pre/20-post while dumped as 20-pre/60-post --
trigger index off by 40. Those are flagged SUSPECT-TRIGGER and excluded from
trigger-relative timing, though their sample values are still valid.

Usage:
    python arming_replay.py [--logs DIR] [--sweep] [--verbose]
"""

import argparse
import glob
import os
import re
import sys
from collections import Counter

# ── Firmware constants, mirrored from src/main.cpp and movement_service.h ─────
# Keep these in step with the firmware or the replay is fiction.
ARRIVAL_QUIET_MSS_MM = 150     # ARRIVAL_QUIET_MSS 0.15f, in milli-m/s^2
ARRIVAL_ARM_SAMPLES = 5        # quiet samples needed to arm
ARM_REV_SAMPLES = 8            # consecutive opposite-signed samples to arm
ARM_DEP_SIGN_N = 25            # samples used to fix the departure's sign
ARRIVAL_PEAK_VALUE_MM = 450    # ARRIVAL_PEAK_VALUE 0.45, in milli-m/s^2
RAMP_BLOCK_N = 12
RAMP_BLOCKS = 3
RAMP_FLOOR_MMSS = 300
RAMP_DIR_PCT = 85

BURST_RE = re.compile(
    r'^BURST\s+(?:k=(?P<k>\w+)\s+)?pre=(?P<pre>\d+)\s+n=(?P<n>\d+)\s+'
    r'signed_mmss=(?P<vals>[-\d\s]+)$'
)


class Burst:
    def __init__(self, kind, pre, samples, log, lineno):
        self.kind = kind or 'unlabelled'
        self.pre = pre
        self.samples = samples
        self.log = log
        self.lineno = lineno
        # 2026-08-11 arrival bursts predate the BURST_POST_ARR single-sourcing
        # fix, so their trigger index cannot be trusted.
        base = os.path.basename(log)
        self.suspect_trigger = ('260811' in base and self.kind == 'arr')

    @property
    def name(self):
        return f"{os.path.basename(self.log)}:{self.lineno} {self.kind}"


def load_bursts(logdir):
    bursts = []
    for path in sorted(glob.glob(os.path.join(logdir, '*.log'))):
        try:
            with open(path, encoding='utf-8', errors='replace') as fh:
                for lineno, line in enumerate(fh, 1):
                    m = BURST_RE.match(line.strip())
                    if not m:
                        continue
                    try:
                        vals = [int(v) for v in m.group('vals').split()]
                    except ValueError:
                        continue
                    n = int(m.group('n'))
                    # Truncated dumps are common when a print is interrupted.
                    if len(vals) < n:
                        continue
                    bursts.append(Burst(m.group('k'), int(m.group('pre')),
                                        vals[:n], path, lineno))
        except OSError as exc:
            print(f"  ! {path}: {exc}", file=sys.stderr)
    return bursts


def dep_sign_of(burst):
    """
    Mirror arr_dep_sign: accumulate the first ARM_DEP_SIGN_N signed samples of
    the RUN and take the sign. The firmware starts that counter at MOVING entry,
    which is near the departure trigger, so for a departure burst the window
    starts at the trigger index rather than at sample 0.
    """
    start = burst.pre
    window = burst.samples[start:start + ARM_DEP_SIGN_N]
    if not window:
        return 0
    return 1 if sum(window) >= 0 else -1


def ramp_would_fire(samples):
    """
    Mirror the ISR block accumulator exactly. Returns (fired, mean, dir_pct,
    index) where index is the sample at which the verdict latched.
    """
    ramp_sum = ramp_abs = 0
    ramp_n = ramp_run = 0
    ramp_sign = 0
    for i, sv in enumerate(samples):
        ramp_sum += sv
        ramp_abs += abs(sv)
        ramp_n += 1
        if ramp_n < RAMP_BLOCK_N:
            continue
        mag = abs(ramp_sum)
        sign = -1 if ramp_sum < 0 else 1
        qual = (mag >= RAMP_FLOOR_MMSS * RAMP_BLOCK_N and
                mag * 100 >= RAMP_DIR_PCT * ramp_abs)
        if qual and (ramp_run == 0 or sign == ramp_sign):
            ramp_sign = sign
            ramp_run += 1
            if ramp_run >= RAMP_BLOCKS:
                return (True, mag // RAMP_BLOCK_N,
                        (mag * 100) // ramp_abs if ramp_abs else 0, i)
        else:
            ramp_run = 1 if qual else 0
            ramp_sign = sign if qual else 0
        ramp_sum = ramp_abs = 0
        ramp_n = 0
    return (False, 0, 0, None)


def arm_index(samples, rule, dep_sign, rev_samples=ARM_REV_SAMPLES):
    """
    Index of the sample at which `rule` arms, or None. Mirrors the firmware's
    quiet and reversal paths.

      'quiet'  -- ARRIVAL_ARM_SAMPLES consecutive samples under the quiet band
      'rev'    -- rev_samples consecutive samples opposite to the departure sign
      'union'  -- either (the shipping behaviour since 46d1a5d)
    """
    quiet = opp = 0
    for i, sv in enumerate(samples):
        dev = abs(sv)
        ssign = 1 if sv > 0 else (-1 if sv < 0 else 0)

        if dev < ARRIVAL_QUIET_MSS_MM:
            quiet += 1
        else:
            quiet = 0
        if dep_sign and ssign and ssign != dep_sign:
            opp += 1
        else:
            opp = 0

        if rule in ('quiet', 'union') and quiet >= ARRIVAL_ARM_SAMPLES:
            return i
        if rule in ('rev', 'union') and dep_sign and opp >= rev_samples:
            return i
    return None


def evaluate(burst, rule, dep_sign, rev_samples=ARM_REV_SAMPLES, start=0):
    """
    Arm, then run the gated detectors over what remains, as the ISR would.

    `start` is where the RUN begins within the burst, and getting it wrong
    invalidates everything. The firmware clears the arming counters on
    STATE_MOVING entry, so it never sees a departure burst's PRE-TRIGGER
    samples -- those are the parked car, and they are quiet by definition.
    Replaying a departure burst from sample 0 therefore arms instantly on the
    parked samples and produces the nonsense conclusion that the shipping
    firmware fires the ramp on half of all departures -- which 30+ live runs
    with zero false releases flatly contradict. Departure bursts MUST be
    replayed from `pre`.
    """
    ai = arm_index(burst.samples[start:], rule, dep_sign, rev_samples)
    if ai is None:
        return dict(armed=False, arm_i=None, ramp=False, peak=False,
                    ramp_mean=0, ramp_dir=0, armed_into_ramp=False)
    ai += start
    tail = burst.samples[ai + 1:]
    fired, mean, dirpct, _ = ramp_would_fire(tail)
    peak = any(abs(v) > ARRIVAL_PEAK_VALUE_MM for v in tail)

    """
    ARMED-INTO-RAMP is the real safety metric, and it is stricter than
    `fired`. A departure burst ends 60 samples (2.4 s) after the trigger, but a
    fast departure ramp outlasts that window -- so "the ramp did not complete
    inside the burst" does NOT mean it would not complete on the device. What
    actually condemns a rule is arming while the departure ramp is still
    running, because from that instant the accumulator is fed the departure.
    Measured as: the block immediately following arming has a mean at or above
    the ramp floor AND still carries the departure's sign.
    """
    blk = burst.samples[ai + 1:ai + 1 + RAMP_BLOCK_N]
    armed_into_ramp = False
    if len(blk) == RAMP_BLOCK_N and dep_sign:
        bsum = sum(blk)
        bsign = -1 if bsum < 0 else 1
        armed_into_ramp = (abs(bsum) >= RAMP_FLOOR_MMSS * RAMP_BLOCK_N
                           and bsign == dep_sign)

    return dict(armed=True, arm_i=ai, ramp=fired, peak=peak,
                ramp_mean=mean, ramp_dir=dirpct,
                armed_into_ramp=armed_into_ramp)


def pair_dep_signs(bursts):
    """
    Supply each arrival burst with the departure sign of the most recent
    departure burst in the same log. Arrival bursts with no preceding departure
    are left unpaired.
    """
    signs = {}
    last_dep_by_log = {}
    for b in bursts:
        if b.kind == 'dep':
            last_dep_by_log[b.log] = dep_sign_of(b)
            signs[id(b)] = dep_sign_of(b)
        else:
            signs[id(b)] = last_dep_by_log.get(b.log, 0)
    return signs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--logs', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'logs'))
    ap.add_argument('--sweep', action='store_true',
                    help='sweep ARM_REV_SAMPLES for the safe range')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()

    bursts = load_bursts(args.logs)
    if not bursts:
        print("no bursts found", file=sys.stderr)
        return 1

    kinds = Counter(b.kind for b in bursts)
    suspect = sum(1 for b in bursts if b.suspect_trigger)
    print(f"Loaded {len(bursts)} bursts from {args.logs}")
    print(f"  by kind: {dict(kinds)}")
    print(f"  SUSPECT-TRIGGER (2026-08-11 arrival, off-by-40): {suspect}")

    signs = pair_dep_signs(bursts)
    deps = [b for b in bursts if b.kind == 'dep']
    arrs = [b for b in bursts if b.kind == 'arr']
    arrs_paired = [b for b in arrs if signs[id(b)] != 0]

    print(f"  arrival bursts with a usable paired departure sign: "
          f"{len(arrs_paired)}/{len(arrs)}")

    # ── Experiment 1: does the block test qualify on departure ramps at all? ──
    print("\n" + "=" * 78)
    print("1. UNGATED: does the ramp block test qualify on a DEPARTURE ramp?")
    print("=" * 78)
    print("   (post-trigger samples only, no arming gate -- this is the hazard")
    print("    arr_armed exists to prevent, not a proposal)")
    hits = 0
    for b in deps:
        fired, mean, dirpct, _ = ramp_would_fire(b.samples[b.pre:])
        if fired:
            hits += 1
            if args.verbose:
                print(f"   qualifies: {b.name} mean={mean} dir={dirpct}")
    print(f"\n   {hits}/{len(deps)} departure bursts qualify the block test "
          f"({100*hits//max(1,len(deps))}%)")
    print("   -> confirms the arithmetic cannot separate departure from arrival.")

    # ── Experiment 2: safety of each rule on departure bursts ────────────────
    print("\n" + "=" * 78)
    print("2. SAFETY: with each rule gating, does the ramp fire on a DEPARTURE?")
    print("=" * 78)
    print("   A rule with ANY ramp hit here would release the beacon on a")
    print("   moving car. This is the disqualifying test.\n")
    print(f"   {'rule':<8} {'armed':>10} {'RAMPFIRED':>11} "
          f"{'ARMED-INTO-RAMP':>17} {'peak':>9}")
    print("   " + "-" * 60)
    for rule in ('quiet', 'rev', 'union'):
        armed = ramp = peak = into = 0
        offenders = []
        for b in deps:
            r = evaluate(b, rule, dep_sign_of(b), start=b.pre)
            armed += r['armed']
            ramp += r['ramp']
            peak += r['peak']
            into += r['armed_into_ramp']
            if r['ramp'] or r['armed_into_ramp']:
                offenders.append((b, r))
        flag = "  <- UNSAFE" if (ramp or into) else "  ok"
        print(f"   {rule:<8} {armed:>4}/{len(deps):<5} {ramp:>6}/{len(deps):<4}"
              f" {into:>11}/{len(deps):<5} {peak:>4}/{len(deps):<4}{flag}")
        if offenders and args.verbose:
            for b, r in offenders[:5]:
                print(f"       {b.name} armed@{r['arm_i']} "
                      f"mean={r['ramp_mean']} dir={r['ramp_dir']}")

    # ── Experiment 3: capability on arrival bursts ───────────────────────────
    print("\n" + "=" * 78)
    print("3. CAPABILITY: with each rule, what fires on an ARRIVAL?")
    print("=" * 78)
    print(f"   paired arrival bursts only (n={len(arrs_paired)}); see the")
    print("   header for why this is PARTIAL evidence, not a service claim.\n")
    print(f"   {'rule':<10} {'armed':>12} {'ramp':>11} {'peak':>11} "
          f"{'neither':>9}")
    print("   " + "-" * 56)
    for rule in ('quiet', 'rev', 'union'):
        armed = ramp = peak = neither = 0
        for b in arrs_paired:
            r = evaluate(b, rule, signs[id(b)])
            armed += r['armed']
            ramp += r['ramp']
            peak += r['peak']
            if r['armed'] and not r['ramp'] and not r['peak']:
                neither += 1
        n = len(arrs_paired)
        print(f"   {rule:<10} {armed:>5}/{n:<6} {ramp:>5}/{n:<5} "
              f"{peak:>5}/{n:<5} {neither:>8}")

    # ── Experiment 4: sweep ARM_REV_SAMPLES ─────────────────────────────────
    if args.sweep:
        print("\n" + "=" * 78)
        print("4. SWEEP: ARM_REV_SAMPLES, reversal-only rule")
        print("=" * 78)
        print("   Looking for the smallest value that arms on arrivals while")
        print("   never letting the ramp fire on a departure.\n")
        print(f"   {'rev_n':>5} {'dep armed':>10} {'DEPRAMP':>8} "
              f"{'INTO-RAMP':>10} {'arr armed':>10} {'arr ramp':>9} "
              f"{'arr peak':>9}")
        print("   " + "-" * 70)
        for rev_n in range(3, 21):
            d_armed = d_ramp = d_into = 0
            for b in deps:
                r = evaluate(b, 'rev', dep_sign_of(b), rev_n, start=b.pre)
                d_armed += r['armed']
                d_ramp += r['ramp']
                d_into += r['armed_into_ramp']
            a_armed = a_ramp = a_peak = 0
            for b in arrs_paired:
                r = evaluate(b, 'rev', signs[id(b)], rev_n)
                a_armed += r['armed']
                a_ramp += r['ramp']
                a_peak += r['peak']
            flag = "  <- UNSAFE" if (d_ramp or d_into) else ""
            print(f"   {rev_n:>5} {d_armed:>6}/{len(deps):<3} "
                  f"{d_ramp:>5}/{len(deps):<3} {d_into:>7}/{len(deps):<3} "
                  f"{a_armed:>6}/{len(arrs_paired):<3}"
                  f" {a_ramp:>5}/{len(arrs_paired):<3} "
                  f"{a_peak:>5}/{len(arrs_paired):<3}{flag}")

    print("\nDone. Read the header before quoting any of this.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
