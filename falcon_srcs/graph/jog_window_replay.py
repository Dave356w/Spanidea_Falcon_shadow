#!/usr/bin/env python3
"""
jog_window_replay.py -- replay a WINDOWED SHAPE test for the jog verdict against
every departure burst on file, and compare it against the shipping opk/ratio
gate.

THE QUESTION (Dave, 2026-08-18): "windowed jog similar to ramp?"

WHY IT WAS ASKED. On 2026-08-18 a hand bump of 0.136 m/s^2 latched a departure.
A real 18 fpm departure is 0.116 m/s^2. The bump is LARGER than a genuine slow
departure, so no amplitude threshold separates them -- and the bump's opk of 118
fell squarely INSIDE the real-run population (92..164). The shipping gate cannot
tell them apart even in principle.

But the SHAPE differs and the firmware already measures shape elsewhere:

    bump        122  136 -114   77  132   28 -118  -90   <- alternating, rings
    departure  -229 -287 -309 -269 -293 -406 -303 -241   <- one-signed ramp

The ramp detector's block test (RAMP_BLOCK_N / RAMP_DIR_PCT / RAMP_FLOOR_MMSS)
is exactly that measurement. This tool asks whether it separates the departure
population better than opk does.

════════════════════════════════════════════════════════════════════════════════
WHAT THIS CAN AND CANNOT ESTABLISH -- read before trusting any number
════════════════════════════════════════════════════════════════════════════════

  ⚠️ THERE ARE NO GROUND-TRUTH LABELS IN THE LOGS THEMSELVES. Nothing in a
     capture records "this burst was a jog" versus "this burst was a real run",
     so this tool CANNOT report an accuracy, a false-JOG rate or a miss rate.

     PARTLY LIFTED 2026-08-20: labels now exist for a SUBSET, recovered from the
     dated session notes and held in datasets/session_g_labels.csv -- 87 of 273
     departures, with a named basis per record. This tool does not read them; it
     remains a separation report over every burst. To score a rule against
     labels, join on (log, run) the way falcon_corpus_labelled_2026-08-20.md §4
     does. Note the labelled jog population is small (17) and NOT random -- jogs
     were written into notes because they were verdict tests -- so a miss rate
     computed from it is still not a miss rate.

  ✅ WHAT IT CAN DO: measure SEPARATION. It computes both statistics on every
     departure burst and reports whether the proposed statistic splits the
     population more cleanly than opk -- overlap between the modes is the
     figure of merit, and overlap needs no labels.

  ✅ AND: report DISAGREEMENTS. Bursts the two rules classify differently are
     the cases a human has to adjudicate, and they are listed by name so they
     can be traced back to a log and a session note.

  ⚠️ Departure bursts only. Arrival bursts are a different question and the
     ramp detector already owns them.

Companion to arming_replay.py, which answers the ARRIVAL arming question and
whose loader this reuses.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arming_replay import load_bursts                      # noqa: E402

# ── shipping jog verdict, mirrored from main.cpp emit_burst_log() ────────────
JOG_DEADBAND_MMSS = 150
JOG_OPP_RATIO_PCT = 33
JOG_OPP_PEAK_MMSS = 900

# ── ramp detector block parameters, from main.cpp ────────────────────────────
RAMP_BLOCK_N = 12
RAMP_DIR_PCT = 85
RAMP_FLOOR_MMSS = 300     # tuned for ARRIVAL decelerations; see report below


def shipping_verdict(samples):
    """Mirror the firmware exactly: pos/neg deadbanded impulses, ratio, opk."""
    pos = neg = 0
    ppk = npk = 0
    for v in samples:
        if v > ppk:
            ppk = v
        if v < npk:
            npk = v
        if v > JOG_DEADBAND_MMSS:
            pos += v
        elif v < -JOG_DEADBAND_MMSS:
            neg -= v
    pri, opp = (pos, neg) if pos >= neg else (neg, pos)
    opk = -npk if pos >= neg else ppk
    ratio = (opp * 100 // pri) if pri > 0 else 0
    is_jog = ratio >= JOG_OPP_RATIO_PCT and opk >= JOG_OPP_PEAK_MMSS
    return {'pos': pos, 'neg': neg, 'ratio': ratio, 'opk': opk,
            'verdict': 'JOG' if is_jog else 'RUN'}


def best_block(samples, block_n=RAMP_BLOCK_N):
    """
    Strongest qualifying block: the ramp detector's own measurement, applied to
    a departure burst. Returns (mean, dir_pct) of the block with the highest
    directional mean.
    """
    best = (0.0, 0.0)
    for b in range(0, len(samples) - block_n + 1, block_n):
        blk = samples[b:b + block_n]
        total = sum(blk)
        pos = sum(1 for v in blk if v > 0)
        neg = sum(1 for v in blk if v < 0)
        mean = abs(total) / block_n
        dir_pct = max(pos, neg) * 100.0 / block_n
        if mean > best[0]:
            best = (mean, dir_pct)
    return best


def windowed_verdict(samples, floor, dir_pct=RAMP_DIR_PCT):
    """A real departure shows a sustained one-signed block; a knock rings."""
    mean, d = best_block(samples)
    return ('RUN' if (mean >= floor and d >= dir_pct) else 'JOG'), mean, d


def main():
    # Non-ASCII in the report aborts on a cp1252 Windows console. Force UTF-8.
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(encoding='utf-8')
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser()
    ap.add_argument('--logs', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'logs'))
    ap.add_argument('--floor', type=float, default=RAMP_FLOOR_MMSS)
    ap.add_argument('--sweep', action='store_true')
    args = ap.parse_args()

    bursts = [b for b in load_bursts(args.logs) if b.kind == 'dep']
    if not bursts:
        print("no departure bursts found", file=sys.stderr)
        return 1

    rows = []
    for b in bursts:
        ship = shipping_verdict(b.samples)
        mean, d = best_block(b.samples)
        rows.append((b, ship, mean, d))

    print(f"departure bursts: {len(rows)}\n")

    # ── separation on each axis ──────────────────────────────────────────────
    ship_jog = [r for r in rows if r[1]['verdict'] == 'JOG']
    ship_run = [r for r in rows if r[1]['verdict'] == 'RUN']

    def span(vals):
        return (min(vals), max(vals)) if vals else (0, 0)

    print("── SHIPPING GATE (opk), as the firmware classifies today ──")
    print(f"  called RUN : n={len(ship_run):3d}  opk {span([r[1]['opk'] for r in ship_run])}")
    print(f"  called JOG : n={len(ship_jog):3d}  opk {span([r[1]['opk'] for r in ship_jog])}")

    print("\n── PROPOSED WINDOWED TEST, same bursts ──")
    print(f"  on bursts the shipping gate calls RUN: "
          f"block-mean {span([round(r[2]) for r in ship_run])}, "
          f"dir% {span([round(r[3]) for r in ship_run])}")
    print(f"  on bursts the shipping gate calls JOG: "
          f"block-mean {span([round(r[2]) for r in ship_jog])}, "
          f"dir% {span([round(r[3]) for r in ship_jog])}")

    # ── floor sweep: where would a departure-side floor sit? ─────────────────
    if args.sweep:
        print("\n── FLOOR SWEEP (dir>=%d%%) ──" % RAMP_DIR_PCT)
        print("  floor   agree-RUN  agree-JOG  disagree")
        for floor in (80, 100, 120, 150, 200, 250, 300):
            aR = aJ = dis = 0
            for b, ship, mean, d in rows:
                w = 'RUN' if (mean >= floor and d >= RAMP_DIR_PCT) else 'JOG'
                if w == ship['verdict'] == 'RUN':
                    aR += 1
                elif w == ship['verdict'] == 'JOG':
                    aJ += 1
                else:
                    dis += 1
            print(f"  {floor:5d}   {aR:9d}  {aJ:9d}  {dis:8d}")

    # ── disagreements at the chosen floor ────────────────────────────────────
    print(f"\n── DISAGREEMENTS at floor={args.floor:.0f}, dir>={RAMP_DIR_PCT}% ──")
    n_dis = 0
    for b, ship, mean, d in rows:
        w = 'RUN' if (mean >= args.floor and d >= RAMP_DIR_PCT) else 'JOG'
        if w != ship['verdict']:
            n_dis += 1
            print(f"  {b.name}")
            print(f"      shipping={ship['verdict']:3s} (opk={ship['opk']}, "
                  f"ratio={ship['ratio']}%)   windowed={w:3s} "
                  f"(mean={mean:.0f}, dir={d:.0f}%)")
    if not n_dis:
        print("  none")

    print("\n⚠️  THIS IS A SEPARATION AND DISAGREEMENT REPORT, not an")
    print("   accuracy report. Every disagreement needs a human to say which")
    print("   rule was right. Labels exist for 87 of 273 departures since")
    print("   2026-08-20 (datasets/session_g_labels.csv); this tool does not")
    print("   read them -- join on (log, run) to score a rule against them.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
