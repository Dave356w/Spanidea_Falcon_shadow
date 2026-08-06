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
    r"(?:\s+rd=(\d+))?(?:\s+ov=(\d+))?"
)
old_re = re.compile(r"Data\s*:\s*(-?\d+\.?\d*)\s+G value\s*:\s*(-?\d+\.?\d*)")
old_gonly_re = re.compile(r"G value\s*:\s*(-?\d+\.?\d*)")
delta_re = re.compile(r"Delta\s*:\s*(-?\d+\.?\d*)")

CORRUPT_HINTS = ("STA ", "ST TE", "MONIT ", "STAT ", "E_MONITORING", "�")


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
                continue

            if "Voltage" in s or "Battery" in s:
                continue

            d = delta_re.search(s)
            if d:
                scale = OLD_SCALE_FIX if self.fmt == "old" else 1.0
                self.fw_delta.append(float(d.group(1)) * scale)

            kind = "corrupt" if any(h in s for h in CORRUPT_HINTS) else "event"
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
