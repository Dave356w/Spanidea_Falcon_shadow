#!/usr/bin/env python3
"""
Replay the lateral calibration at shorter window lengths.

WHAT THIS ANSWERS. The customer asked whether the 10 s calibration can be 5 s,
or can end when the device judges itself stable. Both are decidable offline from
one line the firmware emits at the end of every calibration attempt:

    XY: bmax 0.0680 0.0690 0.0550 ...      per-second maxima, IN TIME ORDER

The device's threshold is median(all buckets) * XY_STILL_MARGIN, clamped. A
shorter window is the same arithmetic over a prefix of the same sequence, so no
new capture and no device change is needed to evaluate one -- only this file.

WHY THE DIRECTION OF THE ERROR IS THE WHOLE QUESTION (lateral.h):

    floor UNDER-estimated -> threshold too low  -> over-eager beacon.  SAFE
    floor OVER-estimated  -> threshold too high -> travel reads as still.
                                                   DANGEROUS

So the number to watch below is not the average deviation, it is the WORST
POSITIVE one. A shorter window that is usually identical and occasionally 20%
high is not a good trade.

    python calib_replay.py <logfile> [logfile ...]

CAVEAT THAT DOES NOT COME OUT OF THE ARITHMETIC. Bench captures on a desk are
not a counterweight in a hoistway. What transfers between the two is the SHAPE of
the comparison, not the numbers; a site with a wider bucket-to-bucket spread will
make every short window noisier than it looks here. Re-run this on hoistway logs
before changing CALIB_TIMEOUT_MS.
"""

import re
import sys

# Must track lateral.h.
XY_STILL_MARGIN = 1.50
XY_STILL_MIN    = 0.02
XY_STILL_MAX    = 0.40
XY_CALIB_MIN_BUCKETS = 6

BMAX_RE = re.compile(r"XY:\s*bmax\s+([0-9.\s]+)")


def threshold(buckets):
    """Exactly what LateralMonitor::calib_finish() computes, for n buckets."""
    if not buckets:
        return None
    s = sorted(buckets)
    # Lower-middle on an even count -- the smaller threshold, the safe side.
    v = s[(len(s) - 1) // 2] * XY_STILL_MARGIN
    return min(max(v, XY_STILL_MIN), XY_STILL_MAX)


def parse(paths):
    runs = []
    for p in paths:
        with open(p, errors="replace") as fh:
            for line in fh:
                m = BMAX_RE.search(line)
                if m:
                    vals = [float(x) for x in m.group(1).split()]
                    if vals:
                        runs.append((p, vals))
    return runs


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1

    runs = parse(argv[1:])
    if not runs:
        print("no 'XY: bmax' lines found -- firmware older than 2026-08-14?")
        return 1

    windows = [3, 4, 5, 6, 8]
    print(f"{len(runs)} calibration(s)\n")
    print("run  n   full      " + "".join(f"{w}s%      " for w in windows))

    worst = {w: (-999.0, None) for w in windows}
    for i, (_, b) in enumerate(runs, 1):
        full = threshold(b)
        row = f"{i:>3} {len(b):>2}  {full:.4f}   "
        for w in windows:
            t = threshold(b[:w])
            if t is None:
                row += "   --    "
                continue
            d = 100.0 * (t / full - 1.0)
            row += f"{d:+6.1f}%  "
            if d > worst[w][0]:
                worst[w] = (d, i)
        print(row)

    print("\nWORST deviation in the DANGEROUS direction (threshold too high):")
    for w in windows:
        d, run = worst[w]
        where = f"  (run {run})" if d > 0.0 else ""
        flag = ""
        if d >= 10.0:
            flag = "   <-- rejects this window"
        elif d > 0.0:
            flag = "   <-- tolerable, but not free"
        print(f"  {w}s: {d:+6.1f}%{where}{flag}")

    print(
        "\nPARITY MATTERS, and it is not an accident. calib_finish() indexes the\n"
        "sorted buckets at (n-1)//2, which on an EVEN count is the LOWER-middle --\n"
        "lateral.h chose that deliberately because the lower of the two is the\n"
        "smaller threshold, i.e. the safe direction. An even-length window\n"
        "inherits that bias; an odd-length window takes the true median and has\n"
        "no such cushion. Expect even windows to look better here, and prefer\n"
        "them when shortening."
    )

    short = [w for w in windows if w < XY_CALIB_MIN_BUCKETS]
    if short:
        print(
            "\nWARNING: XY_CALIB_MIN_BUCKETS is "
            f"{XY_CALIB_MIN_BUCKETS}; windows {short} would be REJECTED outright "
            "by the quorum\ncheck. Shortening the window means lowering the "
            "quorum with it, or every\ncalibration falls back to XY_STILL_MIN "
            "-- an over-eager device on every install."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
