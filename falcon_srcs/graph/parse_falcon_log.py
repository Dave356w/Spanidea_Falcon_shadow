#!/usr/bin/python3
"""
Parse and summarise Falcon serial captures (PuTTY logs).

Handles both log formats:

  new (timestamped)   t=48210 a=9.7944 avg=9.7901 st=4 rd=1440 ov=0
  old (pre-2026-08)    Data : 998.352050  G value : 0.597768

The old format has two quirks this script corrects so that captures from
either firmware can be compared directly:

  * "G value" was already m/s^2, not g -- the label is wrong.
  * g_value was computed as z/16384.0, but the BMA456 driver returns
    milli-g, so every reading is low by a factor of 16.384. See commit
    7bc6cba, which changed the divisor to 1000.0.

Old-format logs have no timestamps, so the sample rate is inferred from the
known 1000 ms STATE_MOVEMENT_DETECTED -> STATE_MOVING transition. New-format
logs carry millis() directly and need no inference.

usage:
    python parse_falcon_log.py <logfile> [<logfile> ...]
    python parse_falcon_log.py --plot <logfile>      (needs matplotlib)
"""

import re
import sys
import statistics as st

OLD_SCALE_FIX = 16.384          # old logs: multiply to reach true m/s^2

new_re = re.compile(
    r"t=(\d+)\s+a=(-?\d+\.?\d*)\s+avg=(-?\d+\.?\d*)\s+st=(\d+)"
    r"(?:\s+rd=(\d+))?(?:\s+ov=(\d+))?(?:\s+im=(\d+))?"
)

# Sensor fault line: "t=... a=ERR er=N st=... rd=... ov=... tk=... im=..."
err_re = re.compile(r"t=(\d+)\s+a=ERR\s+er=(\d+)\s+st=(\d+)")

# Lateral stillness fields, appended to the sample line by the Z + X/Y build.
# m= is |dx|+|dy| between consecutive UNBLANKED samples and q= the consecutive
# count under the learned threshold. Searched separately from new_re so that
# captures from older firmware keep parsing unchanged.
xym_re = re.compile(r"\sm=(-?\d+\.?\d*)\s+q=(\d+)")
xycal_re = re.compile(r"XY:\s*calib\s+b=(\d+)\s+peak=(-?\d+\.?\d*)\s+mv=(\d+)")
xystill_re = re.compile(r"XY-Still-Value\s*:\s*(-?\d+\.?\d*)")

# Any-motion interrupt edge, and the status poll that clears it.
accint_re  = re.compile(r"ACC-INT\s+n=(\d+)\s+t=(\d+)\s+st=(\d+)\s+bz=(\d+)(?:\s+pin=(\d+))?")
accstat_re = re.compile(r"ACC-STAT\s+s=0x([0-9A-Fa-f]+)\s+pin=(\d+)\s+n=(\d+)")

# Latched-FSM lifecycle lines (roadmap item 8).
LATCH_RE = (
    ("departure", re.compile(r"FSM:\s*Departure latched")),
    ("arr_int",   re.compile(r"FSM:\s*Arrival \(any-motion\), edges (\d+)")),
    ("arr_poll",  re.compile(r"FSM:\s*Arrival \(polled\), delta (-?\d+\.?\d*)")),
    ("stillness", re.compile(r"FSM:\s*Released on stillness")),
    ("xy_rel",    re.compile(r"FSM:\s*Release \(x/y still\) m=(-?\d+\.?\d*)")),
    ("failsafe",  re.compile(r"FSM:\s*FAILSAFE")),
    ("blanked",   re.compile(r"FSM:\s*any-motion ignored")),
    ("stopped",   re.compile(r"FSM:\s*Transitioned to STATE_MONITORING")),
)
old_re = re.compile(r"Data\s*:\s*(-?\d+\.?\d*)\s+G value\s*:\s*(-?\d+\.?\d*)")
old_gonly_re = re.compile(r"G value\s*:\s*(-?\d+\.?\d*)")
delta_re = re.compile(r"Delta\s*:\s*(-?\d+\.?\d*)")

CORRUPT_HINTS = ("STA ", "ST TE", "MONIT ", "E_MONITORING", "�")

# Lines that look like state names but are healthy output, not contention.
NOT_CORRUPT = ("ACC-INT", "ACC-STAT", "FSM:", "STATE_")


class Capture(object):
    def __init__(self, path):
        self.path = path
        self.fmt = None          # "new" or "old"
        self.t_ms = []           # None-filled for old format
        self.accel = []          # true m/s^2
        self.avg = []
        self.state = []
        self.read_us = []
        self.overrun = []
        self.marks = []          # (sample_index, kind, text)
        self.fw_delta = []       # firmware's own "still moving?" variable
        self.im = []             # cumulative any-motion count per sample
        self.edges = []          # (t_ms, state, bz) per ACC-INT
        self.latch = []          # (t_ms, kind, detail) FSM lifecycle
        self.errs = []           # (t_ms, consecutive) sensor read failures
        self.xy_m = []           # lateral metric per sample, None if absent
        self.xy_q = []           # quiet run per sample
        self.xy_still = None     # learned threshold, m/s^2
        self.xy_calib = None     # (buckets, peak, moved)
        self._parse()

    def _parse(self):
        with open(self.path, "rb") as fh:
            text = fh.read().decode("utf-8", errors="replace")

        for line in text.splitlines():
            s = line.strip()
            if not s or "PuTTY log" in s:
                continue

            m = new_re.search(s)
            if m:
                self.fmt = "new"
                self.t_ms.append(int(m.group(1)))
                self.accel.append(float(m.group(2)))
                self.avg.append(float(m.group(3)))
                self.state.append(int(m.group(4)))
                self.read_us.append(int(m.group(5)) if m.group(5) else None)
                self.overrun.append(int(m.group(6)) if m.group(6) else None)
                self.im.append(int(m.group(7)) if m.group(7) else None)
                xy = xym_re.search(s)
                self.xy_m.append(float(xy.group(1)) if xy else None)
                self.xy_q.append(int(xy.group(2)) if xy else None)
                continue

            m = old_re.search(s) or old_gonly_re.search(s)
            if m:
                self.fmt = self.fmt or "old"
                val = float(m.group(2) if m.lastindex and m.lastindex > 1
                            else m.group(1))
                self.t_ms.append(None)
                self.accel.append(val * OLD_SCALE_FIX)
                self.avg.append(None)
                self.state.append(None)
                self.read_us.append(None)
                self.overrun.append(None)
                self.im.append(None)
                self.xy_m.append(None)
                self.xy_q.append(None)
                continue

            # --- new-format lines the old parser mistook for corruption -----
            m = accint_re.search(s)
            if m:
                self.edges.append((int(m.group(2)), int(m.group(3)),
                                   int(m.group(4))))
                continue

            if accstat_re.search(s):
                continue          # status poll: healthy, carries no new data

            m = xycal_re.search(s)
            if m:
                self.xy_calib = (int(m.group(1)), float(m.group(2)),
                                 int(m.group(3)))
                self.marks.append((len(self.accel), "event", s))
                continue

            m = xystill_re.search(s)
            if m:
                self.xy_still = float(m.group(1))
                self.marks.append((len(self.accel), "event", s))
                continue

            m = err_re.search(s)
            if m:
                self.errs.append((int(m.group(1)), int(m.group(2))))
                continue

            hit = None
            for kind, rx in LATCH_RE:
                mm = rx.search(s)
                if mm:
                    hit = (kind, mm.group(1) if mm.groups() else None)
                    break
            if hit:
                # FSM lines carry no timestamp of their own. Anchor to the
                # sample INDEX and resolve later against the NEXT sample -- the
                # previous one precedes the edge that triggered the transition,
                # so anchoring backwards drops the deciding edge out of the run
                # window and makes a legitimate cluster look impossible.
                self.latch.append((len(self.accel), hit[0], hit[1]))
                self.marks.append((len(self.accel), "event", s))
                continue

            if "Voltage" in s or "Battery" in s:
                continue

            d = delta_re.search(s)
            if d:
                scale = OLD_SCALE_FIX if self.fmt == "old" else 1.0
                self.fw_delta.append(float(d.group(1)) * scale)

            kind = "event"
            if (any(h in s for h in CORRUPT_HINTS)
                    and not any(g in s for g in NOT_CORRUPT)):
                kind = "corrupt"
            if "FSM" in s or "STATE" in s or kind == "corrupt":
                self.marks.append((len(self.accel), kind, s))

    # ---------------------------------------------------------------- rate
    def sample_rate(self):
        """Samples per second. Measured for new logs, inferred for old."""
        if self.fmt == "new" and len(self.t_ms) > 2:
            span = self.t_ms[-1] - self.t_ms[0]
            if span > 0:
                return (len(self.t_ms) - 1) * 1000.0 / span, "measured"

        # old format: MOVEMENT_DETECTED -> MOVING is MOVEMENT_DETECTION_TIMEOUT_MS
        gaps, det = [], None
        for i, kind, s in self.marks:
            if "MOVEMENT_DETECTED" in s:
                det = i
            elif "STATE_MOVING" in s and det is not None:
                if i > det:
                    gaps.append(i - det)
                det = None
        if gaps:
            return st.mean(gaps), "inferred from 1000 ms FSM transition"
        return float("nan"), "unknown"

    def first_motion(self):
        for i, kind, s in self.marks:
            if "MOVEMENT_DETECTED" in s:
                return i
        return len(self.accel)

    def report(self):
        n = len(self.accel)
        if n == 0:
            print("%s: no samples parsed" % self.path)
            return

        rate, how = self.sample_rate()
        fm = self.first_motion()
        quiet = self.accel[:fm]
        base = st.median(quiet) if len(quiet) >= 5 else st.median(self.accel)
        qdev = [abs(v - base) for v in quiet]

        # replicate the firmware pipeline: RollingAvg(4), then consecutive delta
        avg4 = [sum(self.accel[max(0, k - 3):k + 1]) /
                len(self.accel[max(0, k - 3):k + 1]) for k in range(n)]
        deltas = [abs(avg4[k] - avg4[k - 1]) for k in range(1, n)]
        qdeltas = deltas[:max(0, fm - 1)]

        print("=" * 74)
        print("%s   [%s format]" % (self.path, self.fmt))
        print("=" * 74)
        print("  samples          : %d" % n)
        print("  sample rate      : %.2f Hz   (%s)" % (rate, how))
        if rate == rate and rate > 0:
            print("  RollingAvg(4) span: %.0f ms   <-- must be well under the"
                  " event being detected" % (4000.0 / rate))
        print("  baseline         : %.4f m/s^2  (%.4f g)" % (base, base / 9.81))
        print("  stationary window: %d samples" % len(quiet))
        if qdev:
            print("    noise sd       : %.4f m/s^2" % st.pstdev(qdev)
                  if len(qdev) > 1 else "")
            print("    noise max|dev| : %.4f m/s^2" % max(qdev))
        if qdeltas:
            print("    max d(avg4)    : %.4f m/s^2   <-- false-fire floor"
                  % max(qdeltas))

        if self.fw_delta:
            print("  firmware 'still moving?' deltas (true m/s^2):")
            print("    " + ", ".join("%.4f" % d for d in self.fw_delta))
            print("    max %.4f   mean %.4f"
                  % (max(self.fw_delta), st.mean(self.fw_delta)))

        rd = [v for v in self.read_us if v is not None]
        if rd:
            print("  I2C read time    : min %d  median %d  max %d us"
                  % (min(rd), int(st.median(rd)), max(rd)))
            if rate == rate and rate > 0:
                print("    budget at %.0f Hz : %.0f us  (%.0f%% consumed)"
                      % (rate, 1e6 / rate, 100.0 * st.median(rd) * rate / 1e6))
        ov = [v for v in self.overrun if v is not None]
        if ov:
            growth = ov[-1] - ov[0]
            flag = "" if growth == 0 else "   <-- LOG IS BEING DECIMATED"
            print("  ISR overruns     : %d -> %d (grew by %d)%s"
                  % (ov[0], ov[-1], growth, flag))

        corrupt = [m for m in self.marks if m[1] == "corrupt"]
        print("  corrupted lines  : %d%s"
              % (len(corrupt), "" if not corrupt else "   <-- serial contention"))
        for i, _, s in corrupt:
            print("      sample %d: %s" % (i, s[:60]))

        report_xy(self)
        report_runs(self)

        print("  events:")
        for i, kind, s in self.marks:
            if kind == "event":
                t = ""
                if self.fmt == "new" and i < len(self.t_ms) and self.t_ms[i]:
                    t = " t=%dms" % self.t_ms[i]
                elif rate == rate and rate > 0:
                    t = " t~%.1fs" % (i / rate)
                print("    [%5d]%s %s" % (i, t, s))
        print("")


def report_xy(cap):
    """Lateral stillness summary -- the RESET half of the Z + X/Y design.

    The contrast this prints is the whole question. XY_STILL is learned from a
    parked window, so what decides whether the design works is where the
    metric sits while the beacon is sounding:

      beacon metric ABOVE the threshold, always  -> never releases, every run
                                                    ends on FAILSAFE. If the
                                                    unit was stationary, that
                                                    is the buzzer holding x/y
                                                    up -- risk 1
      beacon metric BELOW it while travelling    -> releases mid-travel, the
                                                    dangerous direction
      below while stopped, above while moving    -> the design works

    Split by FSM state rather than by wall clock: st=4 is STATE_MOVING (beacon
    on), st=2 is STATE_MONITORING (armed and silent).
    """
    have = [v for v in cap.xy_m if v is not None]
    if not have:
        return

    print("  lateral (x/y):")
    if cap.xy_calib:
        b, peak, moved = cap.xy_calib
        print("    calibration    : %d buckets, peak %.4f, moved=%d%s"
              % (b, peak, moved, "   <-- REJECTED" if moved else ""))
    if cap.xy_still is not None:
        print("    XY_STILL       : %.4f m/s^2" % cap.xy_still)

    def band(states):
        return [cap.xy_m[k] for k in range(len(cap.xy_m))
                if cap.xy_m[k] is not None and k < len(cap.state)
                and cap.state[k] in states]

    for label, states in (("monitoring (silent)", (2,)),
                          ("beacon (st=4)", (4,)),
                          ("decelerating", (5,))):
        vals = band(states)
        if not vals:
            continue
        line = ("    %-15s: n=%-4d min %.4f  median %.4f  max %.4f"
                % (label, len(vals), min(vals), st.median(vals), max(vals)))
        if cap.xy_still is not None:
            under = sum(1 for v in vals if v < cap.xy_still)
            line += "   %d%% under XY_STILL" % (100 * under / len(vals))
        print(line)

    if cap.xy_still is not None:
        beacon = band((4,))
        if beacon and min(beacon) >= cap.xy_still:
            print("    ** the metric NEVER got under XY_STILL while sounding."
                  " If the unit was stationary this is risk 1: the buzzer is"
                  " holding x/y up and the beacon can never release. **")


def report_runs(cap):
    """Per-run summary of the latched FSM (roadmap item 8).

    One line per trip: how long the alarm held, which path released it, and
    the two numbers that decide an arrival -- quiet-window any-motion edges,
    and the peak polled delta. Cruise edge rate is printed alongside because
    that is what differs between speeds and what the 2-edge clustering rule
    is sensitive to.
    """
    if not any(k == "departure" for _, k, _ in cap.latch):
        return          # pre-item-8 log, or no trip captured

    calib = None
    for _, kind, txt in cap.marks:
        m = re.search(r"Zero-Calib-Value\s*:\s*(-?\d+\.?\d*)", txt)
        if m:
            calib = float(m.group(1))
    if calib is None and cap.avg:
        vals = [v for v in cap.avg if v is not None]
        calib = st.median(vals) if vals else 0.0

    def at(idx):
        if not cap.t_ms:
            return None
        return cap.t_ms[min(idx, len(cap.t_ms) - 1)]

    open_run = None
    runs = []
    for idx, kind, detail in cap.latch:
        t = at(idx)
        if kind == "departure":
            open_run = {"start": t, "release": None, "detail": None}
        elif kind in ("arr_int", "arr_poll", "stillness", "xy_rel",
                      "failsafe") and open_run:
            open_run["release"] = t
            open_run["detail"] = (kind, detail)
            runs.append(open_run)
            open_run = None
    if open_run:
        open_run["release"] = None
        runs.append(open_run)

    LABEL = {"arr_int": "any-motion", "arr_poll": "polled",
             "stillness": "STILLNESS", "xy_rel": "x/y still m=",
             "failsafe": "FAILSAFE"}

    print("  latched-FSM runs:")

    for i, r in enumerate(runs, 1):
        t0 = r["start"]
        t1 = r["release"]
        held = "(open)" if (t1 is None or t0 is None) else "%.1f s" % ((t1 - t0) / 1000.0)
        kind, detail = r["detail"] if r["detail"] else ("none", None)
        how = LABEL.get(kind, kind)
        if detail:
            how += " %s" % detail

        # Any-motion edges inside the run, split by buzzer state.
        #
        # Only st=4 (STATE_MOVING) counts: notify_any_motion() ignores edges in
        # every other state, so edges raised during MOVEMENT_DETECTED or
        # DECELERATING must not be included or the clustering analysis will not
        # match what the firmware actually saw.
        span = [e for e in cap.edges
                if t0 is not None and e[0] >= t0 and (t1 is None or e[0] <= t1)
                and e[1] == 4]
        quiet = [e for e in span if e[2] == 0]
        loud = len(span) - len(quiet)

        # closest quiet-edge pair -- what the clustering rule actually sees
        gap = None
        for a, b in zip(quiet, quiet[1:]):
            d = b[0] - a[0]
            gap = d if gap is None or d < gap else gap

        # Split the run into CRUISE and ARRIVAL.
        #
        # The arrival window is the last ARRIVAL_TAIL_MS of the run; everything
        # between MIN_TRAVEL and that is cruise. Edges during cruise are the
        # number that predicts a mid-travel false release -- at 18 fpm there
        # were none across 80 s, at 50 fpm the condition asserted every poll
        # and clustered. Aggregating the two together hides exactly that.
        ARRIVAL_TAIL_MS = 8000
        MIN_TRAVEL_MS = 3000
        cru_lo = t0 + MIN_TRAVEL_MS if t0 is not None else None
        cru_hi = (t1 - ARRIVAL_TAIL_MS) if t1 is not None else None

        cru_edges = arr_edges = None
        cru_rate = None
        if cru_lo is not None and cru_hi is not None and cru_hi > cru_lo:
            cru_edges = [e for e in quiet if cru_lo <= e[0] <= cru_hi]
            arr_edges = [e for e in quiet if e[0] > cru_hi]
            cru_rate = len(cru_edges) * 60000.0 / (cru_hi - cru_lo)

        # closest quiet pair inside cruise only -- a pair here is a false
        # release waiting to happen
        cru_gap = None
        if cru_edges:
            for a, b in zip(cru_edges, cru_edges[1:]):
                d = b[0] - a[0]
                cru_gap = d if cru_gap is None or d < cru_gap else cru_gap

        # peak polled delta split the same way
        def peak_between(lo, hi):
            best = 0.0
            for t, av in zip(cap.t_ms, cap.avg):
                if t is None or av is None or lo is None or hi is None:
                    continue
                if lo <= t <= hi:
                    best = max(best, abs(av - calib))
            return best

        # peak polled delta during the run
        peak = 0.0
        for t, av in zip(cap.t_ms, cap.avg):
            if t is None or av is None or t0 is None:
                continue
            if t >= t0 and (t1 is None or t <= t1):
                peak = max(peak, abs(av - calib))

        print("    run %d  held %-8s  released by %-18s" % (i, held, how))
        print("           edges %d quiet / %d buzzer   closest quiet pair %s"
              % (len(quiet), loud,
                 ("%d ms" % gap) if gap is not None else "n/a"))
        print("           peak polled delta %.3f  (arrival threshold 0.30)" % peak)

        if cru_rate is not None:
            risk = ""
            if cru_gap is not None and cru_gap <= 2500:
                risk = "   <-- CLUSTERS IN CRUISE, mid-travel release risk"
            print("           CRUISE  %d quiet edges  %.1f/min  closest pair %s%s"
                  % (len(cru_edges), cru_rate,
                     ("%d ms" % cru_gap) if cru_gap is not None else "none",
                     risk))
            print("           ARRIVAL %d quiet edges  peak delta %.3f  (cruise peak %.3f)"
                  % (len(arr_edges), peak_between(cru_hi, t1),
                     peak_between(cru_lo, cru_hi)))

    # cruise edge rate: quiet edges while alarming, per minute
    tot_quiet = sum(1 for e in cap.edges if e[2] == 0 and e[1] == 4)
    tot_loud = sum(1 for e in cap.edges if e[2] == 1 and e[1] == 4)
    span_ms = 0
    for r in runs:
        if r["start"] is not None and r["release"] is not None:
            span_ms += r["release"] - r["start"]
    if span_ms:
        print("  while alarming: %.1f quiet edges/min, %.1f buzzer edges/min"
              % (tot_quiet * 60000.0 / span_ms, tot_loud * 60000.0 / span_ms))
        print("    ^ quiet-edge rate is what the 2-in-2500ms rule risks")

    if cap.errs:
        print("  sensor read failures: %d (max consecutive %d)"
              % (len(cap.errs), max(e[1] for e in cap.errs)))


def plot(cap):
    try:
        from matplotlib import pyplot as plt
    except ImportError:
        sys.exit("matplotlib not installed: pip install matplotlib")

    rate, _ = cap.sample_rate()
    xs = ([t / 1000.0 for t in cap.t_ms] if cap.fmt == "new"
          else [i / rate for i in range(len(cap.accel))])

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(12, 7))
    ax[0].plot(xs, cap.accel, linewidth=0.8, label="a (m/s^2)")
    if cap.fmt == "new" and any(v is not None for v in cap.avg):
        ax[0].plot(xs, cap.avg, linewidth=1.2, label="rolling avg(4)")
    ax[0].set_ylabel("m/s^2")
    ax[0].grid(True)
    ax[0].legend(loc="upper right")
    ax[0].set_title(cap.path)

    for i, kind, s in cap.marks:
        if kind != "event" or i >= len(xs):
            continue
        ax[0].axvline(xs[i], color="r" if "MOVING" in s else "0.6",
                      linewidth=0.7, linestyle="--")

    if cap.fmt == "new" and any(v is not None for v in cap.state):
        ax[1].step(xs, cap.state, where="post")
        ax[1].set_ylabel("FSM state")
        ax[1].grid(True)
    ax[1].set_xlabel("seconds")
    plt.tight_layout()
    plt.show()


def main():
    args = [a for a in sys.argv[1:] if a != "--plot"]
    want_plot = "--plot" in sys.argv
    if not args:
        sys.exit(__doc__)
    for path in args:
        cap = Capture(path)
        cap.report()
        if want_plot:
            plot(cap)


if __name__ == "__main__":
    main()
