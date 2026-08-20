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

# Windowed raw arrival peak. This is what the FSM actually decides an arrival
# on, and reading it back is what exposes the arming defect below.
pk_re = re.compile(r"\spk=(\d+\.?\d*)")

# Session D reference values, from the firmware and the test plan.
#   ARRIVAL_PEAK_VALUE is movement_service.h:236 -- the gate itself.
#   SEPARATION_MIN is the plan's design threshold: below it, no single
#   gate serves this direction and position and the arrival path needs a
#   second axis rather than a retune.
ARRIVAL_PEAK_VALUE = 0.45
SEPARATION_MIN = 1.6
# Added 2026-08-19 with B2/B4 (commit 0cacf29).
#   cp= is the RETIRED arrival bucket -- arr_peak_prev alone, lagging pk= by
#      one full 1 s bucket. pk= is max(cur, prev) and therefore already
#      carries the arrival transient at a stop; cp= is what reports the
#      CRUISE ceiling. Read it during cruise, never at the stop.
#   bz= is beacon state, now on the periodic line and not only on ACC-INT.
cp_re = re.compile(r"\scp=(\d+\.?\d*)")
bz_re = re.compile(r"\sbz=(\d+)")
xycal_re = re.compile(r"XY:\s*calib\s+b=(\d+)\s+peak=(-?\d+\.?\d*)\s+mv=(\d+)")
xystill_re = re.compile(r"XY-Still-Value\s*:\s*(-?\d+\.?\d*)")

# Any-motion interrupt edge, and the status poll that clears it.
accint_re  = re.compile(r"ACC-INT\s+n=(\d+)\s+t=(\d+)\s+st=(\d+)\s+bz=(\d+)(?:\s+pin=(\d+))?")
accstat_re = re.compile(r"ACC-STAT\s+s=0x([0-9A-Fa-f]+)\s+pin=(\d+)\s+n=(\d+)")

# Latched-FSM lifecycle lines (roadmap item 8).
LATCH_RE = (
    ("departure", re.compile(r"FSM:\s*Departure latched")),
    # A departure caught by the POLLED path prints nothing at all:
    # "FSM: Departure latched (any-motion)" lives inside
    # if (any_motion_pending) in movement_service.cpp, so the polled
    # backstop enters MOVEMENT_DETECTED silently -- no line, no burst, no
    # jog verdict. Observed live 2026-08-19: a capture held 4 real runs and
    # only 3 "Departure latched" lines, and the polled run was dropped from
    # the analysis entirely. Track the state transition so a silent
    # departure still opens a run. This is instrumentation gap 4 / item B3;
    # it is a WORKAROUND in the parser, not a fix in the firmware.
    ("dep_move",  re.compile(r"FSM:\s*Transitioned to STATE_MOVEMENT_DETECTED")),
    ("arr_int",   re.compile(r"FSM:\s*Arrival \(any-motion\), edges (\d+)")),
    ("arr_poll",  re.compile(r"FSM:\s*Arrival \(polled\), (?:delta|peak) (-?\d+\.?\d*)")),
    # Ramp verdict: a release only when armed; unarmed it logs and the run
    # continues, so it must NOT close a run here -- handled as informational.
    ("arr_ramp",  re.compile(r"FSM:\s*Arrival \(ramp\) mean=(\d+) dir=(\d+) \(armed\)")),
    ("ramp_obs",  re.compile(r"FSM:\s*Arrival \(ramp\) mean=(\d+) dir=(\d+) \(unarmed\)")),
    ("jog_rel",   re.compile(r"FSM:\s*Release \(jog verdict\)")),
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
        self.pk = []             # windowed raw arrival peak, None if absent
        self.cp = []             # retired bucket = cruise ceiling, None if absent
        self.bz = []             # beacon state per sample, None if absent
        self.boots = 0           # 'Device Booted' markers seen
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

            if "Device Booted" in s:
                self.boots += 1

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
                pkm = pk_re.search(s)
                self.pk.append(float(pkm.group(1)) if pkm else None)
                cpm = cp_re.search(s)
                self.cp.append(float(cpm.group(1)) if cpm else None)
                bzm = bz_re.search(s)
                self.bz.append(int(bzm.group(1)) if bzm else None)
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
                self.pk.append(None)
                self.cp.append(None)
                self.bz.append(None)
                continue

            # --- new-format lines the old parser mistook for corruption -----
            m = accint_re.search(s)
            if m:
                # 4th element: sample index at parse time.
                #
                # t= RESTARTS AT ZERO ON EVERY BOOT, so a capture holding more
                # than one boot overlaps in t and any value-only filter silently
                # mixes them. Measured 2026-08-19: one capture had boot 1 at
                # t=6946..388546 and boot 3 at t=6946..381976 -- near-total
                # overlap. Index is monotonic across the whole file, so run
                # windows are bounded by index and only then compared on t.
                self.edges.append((int(m.group(2)), int(m.group(3)),
                                   int(m.group(4)), len(self.t_ms)))
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
                    hit = (kind, " ".join(mm.groups()) if mm.groups() else None)
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

        # A capture that spans reflashes contains several millis() epochs, and
        # every timing figure below silently mixes them -- the symptom is a
        # NEGATIVE inter-edge gap. This trap has produced a completely false
        # result at least twice (13 phantom "false departures" on 2026-08-11,
        # and a bogus 11.5 s late-release on 2026-08-12). Warn rather than
        # guess: splitting is a judgement about which section matters.
        if self.boots > 1:
            print("  ** %d BOOT SECTIONS IN THIS CAPTURE -- timestamps restart"
                  " at each one and every figure below mixes them. Split on"
                  " 'Device Booted' and analyse one section. **" % self.boots)
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


"""
LATE-RELEASE AND ARMING CHECK -- added 2026-08-12.

A run on 2026-08-12 (single floor, intermediate to bottom terminal, extended
slowdown) alarmed for 85 s over a car that had been stationary for 78 of them.
The deceleration itself was textbook -- one-signed, +0.5 to +0.64 m/s^2 for
3.2 s -- and BOTH detectors were switched off through all of it, because
arr_armed never armed: arming needs 5 consecutive quiet samples and a short
run has no cruise between its departure and deceleration ramps.

WHY THIS NEEDS A CHECK RATHER THAN AN EYEBALL. It logged as

    FSM: Arrival (polled), peak 0.458

which is indistinguishable from a healthy release. The beacon was actually
freed by an unrelated disturbance (doors, or someone moving in the cab) 78 s
after the stop. Every historical per-run summary could contain this and read
as a clean release, so the two signatures below are tested for explicitly.

  ARMING FAILURE -- pk == 0.00 while the raw deviation is well above
    ARRIVAL_QUIET_MSS, AND late enough in the run that the departure cannot
    explain it. The window matters: pk is DELIBERATELY 0.00 through the
    departure ramp, which is the arming gate working correctly, and a naive
    "pk==0 during an excursion" test fires on every healthy run. Departure
    ramps measured up to 3.3 s at 300 fpm, so only excursions later than
    ARM_GRACE_MS after the latch count as evidence of a stuck gate.

  LATE RELEASE -- the run's last sustained one-signed excursion (the
    deceleration that physically brought the car to rest) ended long before
    the release fired.

    Two rejected approaches, both wrong for reasons worth keeping:

      "quiet immediately before the release" -- defeated by the very
      disturbance that caused the release. The 0.458 spike that finally freed
      the beacon is itself out of band, so the walk-back stops after one
      sample and measures nothing.

      "longest in-band stretch in the run" -- cannot work at all. At constant
      velocity z reads 1 g exactly as it does parked (§3), so a long quiet
      stretch is equally consistent with cruising and with being stopped.
      That is the same trap that got the stillness backstop banned.

    Keying on the deceleration ramp avoids both: a sustained one-signed
    excursion is a real, dated, physical event, and once the car has shed its
    speed it IS stopped. Everything after that ramp is a stationary car.

Both are reported per run, and neither changes any existing output.
"""

ARM_QUIET_MSS   = 0.15      # ARRIVAL_QUIET_MSS in main.cpp
LATE_QUIET_MS   = 8000      # gap after the decel ramp that means "already parked"

# What counts as deceleration-ramp content in the decimated log. 0.30 sits
# above cartop cruise peaks (0.07-0.28) and well under the measured plateau
# (0.49-0.61); 1.5 s is half the shortest ramp measured, so a ramp cannot be
# missed while a single vibration spike cannot qualify.
RAMP_DEV_MSS    = 0.30
RAMP_MIN_MS     = 1500

# How long after the departure latch a pk==0.00 excursion is still explained
# by the departure ramp itself. 5 s against measured ramps of up to 3.3 s.
ARM_GRACE_MS    = 5000


def check_release(cap, calib, i0, i1):
    """Return (arming_failed, quiet_ms_before_release) for one run.

    i0/i1 are sample indices bounding the run. Returns (None, None) when the
    capture lacks pk= (pre-2026-08-10 firmware), so old logs stay silent
    rather than reporting a defect the firmware could not have had.
    """
    if not cap.pk or i1 is None or i0 is None:
        return (None, None)

    seg = [(cap.pk[i], cap.accel[i], cap.t_ms[i])
           for i in range(i0, min(i1 + 1, len(cap.pk)))
           if cap.pk[i] is not None and cap.accel[i] is not None]
    if not seg:
        return (None, None)

    # 1. Arming failure: a real excursion the peak collector reported as zero,
    #    late enough that the departure ramp cannot account for it.
    t_start = seg[0][2]
    blind = 0.0
    if t_start is not None:
        blind = max((abs(a - calib) for pk, a, t in seg
                     if pk == 0.0 and t is not None
                     and (t - t_start) > ARM_GRACE_MS), default=0.0)
    arming_failed = blind > ARM_QUIET_MSS * 2

    # 2. Late release: how long after the last deceleration ramp did the
    #    release fire? A ramp is a run of same-signed samples above
    #    RAMP_DEV_MSS lasting at least RAMP_MIN_MS. The log is decimated 8:1,
    #    so a 3 s ramp is ~10 logged samples -- coarse but unambiguous.
    #    THE DEPARTURE RAMP MUST BE EXCLUDED. It is a sustained one-signed
    #    excursion of exactly the same shape, and on a healthy fast stop the
    #    FSM releases on the DECELERATION'S LEADING EDGE -- so the
    #    deceleration lies mostly after the release and the only complete
    #    ramp inside the run window is the departure. Measuring from that
    #    reports every healthy run as "released 11 s late". Only ramps ending
    #    beyond the departure grace count.
    late_ms = None
    t_rel = seg[-1][2]
    ramp_end = None
    cur_sign, cur_start, cur_last = 0, None, None

    def close(end):
        if cur_start is None or end is None:
            return None
        if end - cur_start < RAMP_MIN_MS:
            return None
        if t_start is None or (end - t_start) <= ARM_GRACE_MS:
            return None        # the departure ramp itself
        return end

    for pk, a, t in seg:
        if t is None:
            continue
        dev = a - calib
        sign = 1 if dev > RAMP_DEV_MSS else (-1 if dev < -RAMP_DEV_MSS else 0)
        if sign != 0 and sign == cur_sign:
            cur_last = t
        elif sign != 0:
            ramp_end = close(cur_last) or ramp_end
            cur_sign, cur_start, cur_last = sign, t, t
        else:
            ramp_end = close(cur_last) or ramp_end
            cur_sign, cur_start, cur_last = 0, None, None
    ramp_end = close(cur_last) or ramp_end

    if ramp_end is not None and t_rel is not None:
        late_ms = t_rel - ramp_end

    return (arming_failed, late_ms)


def report_runs(cap):
    """Per-run summary of the latched FSM (roadmap item 8).

    One line per trip: how long the alarm held, which path released it, and
    the two numbers that decide an arrival -- quiet-window any-motion edges,
    and the peak polled delta. Cruise edge rate is printed alongside because
    that is what differs between speeds and what the 2-edge clustering rule
    is sensitive to.
    """
    if not any(k in ("departure", "dep_move") for _, k, _ in cap.latch):
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
            open_run = {"start": t, "release": None, "detail": None,
                        "ramp_obs": [], "i0": idx, "i1": None,
                        "via": "any-motion"}
        elif kind == "dep_move":
            # Normal runs print the departure line and THEN transition, so a
            # run is already open here and this is the follow-on. If nothing is
            # open, the departure was silent -- the polled backstop.
            if open_run is None:
                open_run = {"start": t, "release": None, "detail": None,
                            "ramp_obs": [], "i0": idx, "i1": None,
                            "via": "POLLED (silent)"}
        elif kind == "ramp_obs" and open_run:
            # Unarmed ramp verdict: informational, does not close the run.
            open_run["ramp_obs"].append((t, detail))
        elif kind in ("arr_int", "arr_poll", "arr_ramp", "jog_rel",
                      "stillness", "xy_rel", "failsafe") and open_run:
            open_run["release"] = t
            open_run["detail"] = (kind, detail)
            open_run["i1"] = idx
            runs.append(open_run)
            open_run = None
    if open_run:
        open_run["release"] = None
        runs.append(open_run)

    LABEL = {"arr_int": "any-motion", "arr_poll": "polled",
             "arr_ramp": "RAMP mean/dir=", "jog_rel": "jog verdict",
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
        # Unarmed ramp verdicts observed during the run (log-only path).
        for rt, rd in r.get("ramp_obs", []):
            ts = ("t=%d " % rt) if rt is not None else ""
            how += "  [ramp obs %smean/dir=%s]" % (ts, rd)

        # Arming / late-release check -- see the block above check_release().
        arming_failed, late_ms = check_release(cap, calib,
                                               r.get("i0"), r.get("i1"))
        warn = []
        if arming_failed:
            warn.append("** DETECTOR NEVER ARMED: pk=0.00 across a real"
                        " excursion well after the departure. Both the peak"
                        " and ramp paths were OFF for this run -- no cruise"
                        " gap to arm on. **")
        if late_ms is not None and late_ms >= LATE_QUIET_MS:
            warn.append("** LATE RELEASE: fired %.1f s AFTER the last"
                        " deceleration ramp ended. The car was already"
                        " stopped, so this did not detect the arrival --"
                        " read it as a false beacon of that length. **"
                        % (late_ms / 1000.0))

        # Any-motion edges inside the run, split by buzzer state.
        #
        # Only st=4 (STATE_MOVING) counts: notify_any_motion() ignores edges in
        # every other state, so edges raised during MOVEMENT_DETECTED or
        # DECELERATING must not be included or the clustering analysis will not
        # match what the firmware actually saw.
        i0, i1 = r.get("i0"), r.get("i1")
        span = [e for e in cap.edges
                if i0 is not None and len(e) > 3 and e[3] >= i0
                and (i1 is None or e[3] <= i1) and e[1] == 4]
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
        def in_run(lo_t, hi_t):
            """Sample indices inside THIS run whose t falls in [lo_t, hi_t].

            Index-bounded first. See the note on cap.edges: t is not unique
            across boots, so a t-only filter can pull samples from a different
            boot entirely.
            """
            out = []
            if i0 is None:
                return out
            hi_i = i1 if i1 is not None else len(cap.t_ms) - 1
            for k in range(max(0, i0), min(hi_i + 1, len(cap.t_ms))):
                t = cap.t_ms[k]
                if t is None:
                    continue
                if (lo_t is None or t >= lo_t) and (hi_t is None or t <= hi_t):
                    out.append(k)
            return out

        def peak_between(lo, hi):
            best = 0.0
            for k in in_run(lo, hi):
                av = cap.avg[k]
                if av is not None:
                    best = max(best, abs(av - calib))
            return best

        # SESSION D -- the two halves of the arrival gate, from the fields
        # B2/B4 added on 2026-08-19.
        #
        # cp= is the RETIRED arrival bucket, so it lags pk= by one full 1 s
        # bucket. That lag is the whole point: pk= is max(cur, prev) and has
        # already absorbed the arrival transient by the time the car stops,
        # which is why the cruise ceiling could never be read from it. Take cp=
        # from the CRUISE window only.
        def field_between(vals, lo, hi):
            best = None
            for k in in_run(lo, hi):
                v = vals[k] if k < len(vals) else None
                if v is not None:
                    best = v if best is None else max(best, v)
            return best

        cruise_ceiling = field_between(cap.cp, cru_lo, cru_hi)
        arrival_peak = field_between(cap.pk, cru_hi, t1)

        # peak polled delta during the run
        peak = 0.0
        for k in in_run(t0, t1):
            av = cap.avg[k]
            if av is not None:
                peak = max(peak, abs(av - calib))

        via = r.get("via", "?")
        print("    run %d  held %-8s  released by %-18s  departed via %s"
              % (i, held, how, via))
        if via.startswith("POLLED"):
            print("           ** SILENT DEPARTURE: no latch line, no burst,"
                  " no jog verdict. Reconstructed from the state"
                  " transition. Item B3. **")
        for w in warn:
            print("           %s" % w)
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

        # --- session D: gate margin ---------------------------------------
        if cruise_ceiling is not None or arrival_peak is not None:
            cc = "%.3f" % cruise_ceiling if cruise_ceiling is not None else "n/a"
            ap = "%.3f" % arrival_peak if arrival_peak is not None else "n/a"
            print("           GATE    cruise ceiling cp=%s   arrival peak pk=%s"
                  "   (gate %.2f)" % (cc, ap, ARRIVAL_PEAK_VALUE))
            if arrival_peak is not None:
                x = arrival_peak / ARRIVAL_PEAK_VALUE
                flag = "  <-- ON THE GATE" if x < 1.10 else ""
                print("                   arrival is %.2fx the gate%s" % (x, flag))
            if cruise_ceiling and arrival_peak is not None and cruise_ceiling > 0:
                sep = arrival_peak / cruise_ceiling
                if sep < SEPARATION_MIN:
                    note = ("  <-- BELOW %.1fx: no single gate serves this"
                            " direction and position. Second axis, not a new"
                            " constant." % SEPARATION_MIN)
                else:
                    note = ""
                print("                   separation %.2fx%s" % (sep, note))

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
    # Non-ASCII in the report aborts on a cp1252 Windows console. Force UTF-8.
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(encoding='utf-8')
        except (AttributeError, ValueError):
            pass

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
