"""Session D -- arrival gate margin, from a cartop capture.

Produces the two halves the gate needs and has never had together: a CRUISE
CEILING and a WEAKEST ARRIVAL from the same regime, plus the separation between
them.

⚠️ WHY THE CRUISE WINDOW OPENS AT +8 s AND NOT parse_falcon_log's +3 s.

`MIN_TRAVEL_MS` is 3000 there, and at 19 fpm that does NOT clear the departure
ramp. Measured 2026-08-19 across four runs: every ceiling taken at +3 s landed
between +3.1 and +3.8 s -- i.e. on the window boundary, still on the ramp tail --
and fell when the window was pushed later:

      run   dir    +3 s    +5 s    +8 s
      2     down   0.080   0.080   0.080
      3     up     0.150   0.100   0.070
      4     up     0.100   0.060   0.060
      5     down   0.080   0.060   0.060

Reading the +3 s column produces a spurious direction dependence (up looking
twice down) and a spurious agreement with ARRIVAL_QUIET_MSS at 0.150. Both
dissolve at +8 s. Any ceiling quoted from a window that starts before the ramp
has decayed is a ramp measurement wearing a cruise label.

The arrival window is the last ARRIVAL_TAIL_MS, matching parse_falcon_log.

Runs are segmented on STATE_MOVING -> "Arrival (", which also picks up runs
whose departure was SILENT. A departure caught by the polled backstop prints
nothing at all -- no latch line, no burst, no jog verdict -- because
"Departure latched (any-motion)" sits inside if (any_motion_pending). One such
run was silently dropped from this analysis on 2026-08-19. That is item B3.

Usage:
    ./session_d.py <capture.log>

Takes ONE capture -- the session D run. Raw captures live in falcon_srcs/logs/
(gitignored, so only on the machine that recorded them); the distilled corpus
in falcon_srcs/datasets/ carries no periodic sample lines, and this tool needs
them for the cruise ceiling.
"""
import io
import os
import re
import sys

CRUISE_SETTLE_MS = 8000      # see the module note
ARRIVAL_TAIL_MS = 8000
ARRIVAL_PEAK_VALUE = 0.45    # movement_service.h
ARRIVAL_ARM_SAMPLES = 5      # main.cpp
SEPARATION_MIN = 1.6         # test plan: below this, a second axis is needed


def parse(path):
    samples, runs = [], []
    cur = None
    pend_arm = None
    silent = False
    saw_latch = False
    for ln in io.open(path, encoding="utf-8", errors="replace"):
        ln = ln.strip()
        if ln.startswith("t=") and " ov=" in ln:
            f = {}
            for p in ln.split():
                if "=" in p:
                    k, v = p.split("=", 1)
                    f[k] = v
            try:
                samples.append((int(f["t"]), int(f.get("st", -1)),
                                float(f.get("cp", "nan")), float(f.get("pk", "nan"))))
            except ValueError:
                pass
            continue
        if "Departure latched" in ln:
            saw_latch = True
        elif "STATE_MOVEMENT_DETECTED" in ln:
            silent = not saw_latch
            saw_latch = False
        elif "STATE_MOVING" in ln and cur is None and samples:
            cur = {"t0": samples[-1][0], "silent": silent, "arm": None}
        elif ln.startswith("ARM q=") and cur is not None:
            m = re.search(r"ARM q=(\d+).*?ro=(\d+)", ln)
            if m:
                cur["arm"] = (int(m.group(1)), int(m.group(2)))
        elif "Arrival (" in ln and cur is not None and samples:
            # Do NOT close here. The ARM line is emitted immediately AFTER the
            # arrival line, so closing on arrival loses q= and ro= -- the two
            # numbers that say whether the arrival path armed with any margin.
            cur["t1"] = samples[-1][0]
            m = re.search(r"Arrival \(([a-z-]+)\)", ln)
            cur["via"] = m.group(1) if m else "?"
            cur["closing"] = True
        elif cur is not None and cur.get("closing") and (
                "STATE_DECELERATING" in ln or "STATE_STOPPED" in ln
                or "STATE_MONITORING" in ln):
            runs.append(cur)
            cur = None
    if cur is not None and cur.get("closing"):
        runs.append(cur)
    return samples, runs


def main():
    # Non-ASCII in the report aborts on a cp1252 Windows console. Force UTF-8.
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(encoding='utf-8')
        except (AttributeError, ValueError):
            pass

    if len(sys.argv) < 2:
        print("usage: session_d.py <capture.log>", file=sys.stderr)
        print("  one session D capture. Raw captures are in falcon_srcs/logs/",
              file=sys.stderr)
        print("  (gitignored). Files from falcon_srcs/datasets/ carry no periodic",
              file=sys.stderr)
        print("  sample lines, which this tool needs for the cruise ceiling.",
              file=sys.stderr)
        return 2
    path = sys.argv[1]
    if not os.path.isfile(path):
        print("no such capture: %s" % path, file=sys.stderr)
        return 2
    samples, runs = parse(path)
    if not samples:
        print("  no sample lines in %s -- this tool needs the periodic" % path,
              file=sys.stderr)
        print("  't= .. st= .. cp= .. pk=' stream, which the datasets/ corpus",
              file=sys.stderr)
        print("  drops. Use the raw capture from falcon_srcs/logs/.",
              file=sys.stderr)
        return 1
    if not runs:
        print("  no runs in %s -- nothing segments on "
              "STATE_MOVING -> \"Arrival (\"." % path, file=sys.stderr)
        return 1
    print("  Session D -- arrival gate margin      %s" % path)
    print("  cruise window [+%.0fs, -%.0fs]   gate %.2f   arm samples %d\n"
          % (CRUISE_SETTLE_MS / 1000.0, ARRIVAL_TAIL_MS / 1000.0,
             ARRIVAL_PEAK_VALUE, ARRIVAL_ARM_SAMPLES))
    print("  run  span    departed        cruise    arrival    separation   ARM q/ro")
    print("  ---  ------  --------------  --------  ---------  -----------  --------")

    ceilings, arrivals, rows = [], [], []
    for i, r in enumerate(runs, 1):
        t0, t1 = r["t0"], r["t1"]
        lo, hi = t0 + CRUISE_SETTLE_MS, t1 - ARRIVAL_TAIL_MS
        cru = [cp for t, st, cp, pk in samples
               if lo <= t <= hi and st == 4 and cp == cp]
        arr = [pk for t, st, cp, pk in samples
               if hi < t <= t1 + 3000 and pk == pk]
        via = "POLLED-SILENT" if r["silent"] else "any-motion"
        if not cru:
            print("  %3d  %5.1fs  %-14s  run too short for a settled cruise window"
                  % (i, (t1 - t0) / 1000.0, via))
            continue
        cc, ap = max(cru), (max(arr) if arr else float("nan"))
        sep = ap / cc if cc > 0 else float("nan")
        q, ro = r["arm"] if r["arm"] else ("-", "-")
        flag = ""
        if isinstance(q, int) and q <= ARRIVAL_ARM_SAMPLES + 2:
            flag = " <-- thin arm"
        print("  %3d  %5.1fs  %-14s  %.3f     %.3f      %6.1fx     %s/%s%s"
              % (i, (t1 - t0) / 1000.0, via, cc, ap, sep, q, ro, flag))
        ceilings.append(cc)
        arrivals.append(ap)
        rows.append((i, cc, ap, sep))

    if not rows:
        return
    print("\n  ---- population ----")
    print("    cruise ceiling  : %.3f - %.3f   (gate was derived against 0.28)"
          % (min(ceilings), max(ceilings)))
    print("    arrival peak    : %.3f - %.3f" % (min(arrivals), max(arrivals)))
    weakest = min(arrivals)
    print("    WEAKEST ARRIVAL : %.3f  = %.2fx the %.2f gate"
          % (weakest, weakest / ARRIVAL_PEAK_VALUE, ARRIVAL_PEAK_VALUE))
    worst_sep = weakest / max(ceilings)
    print("    worst-case separation (weakest arrival / highest cruise): %.1fx"
          % worst_sep)
    if worst_sep < SEPARATION_MIN:
        print("\n    ** BELOW %.1fx -- no single gate serves this direction and"
              " position. Second axis, not a new constant. **" % SEPARATION_MIN)
    else:
        print("    clear of the %.1fx cliff by %.0fx" % (SEPARATION_MIN, worst_sep / SEPARATION_MIN))
    n = len(rows)
    print("\n    %d run%s. The plan asks for 20 minimum: the weakest arrival has"
          % (n, "" if n == 1 else "s"))
    print("    moved by 3x across five previous runs, so a small sample misleads.")


if __name__ == "__main__":
    sys.exit(main())
