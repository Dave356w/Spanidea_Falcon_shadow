"""
velocity_replay.py -- set and check the thresholds in falcon_srcs/src/velocity.h
against a real capture.

WHY THIS EXISTS

velocity.h derives every gate from VEL_PARKED_PEAK, the largest |w| a PARKED
device produces. That number is a property of the sample rate, not of the
algorithm, so it must be re-measured after any change to the ISR rate, the
sensor ODR, or the filter bandwidth -- and the module must not be armed until
it has been.

This script is a line-for-line mirror of the VelocityWindow class. Keep the two
in step: if velocity.cpp changes, change this, or the numbers it produces stop
describing the firmware.

USAGE

    python velocity_replay.py soak   <log>...   # false-departure rate
    python velocity_replay.py scale  <log>...   # is more sample rate worth it?
    python velocity_replay.py runs   <log>...   # departure/arrival integrals

WHAT "PARKED" MEANS HERE

Only a stretch that is entirely STATE_MONITORING with no FSM event nearby. On a
cartop capture most MONITORING time is NOT parked -- the unit is being ridden
and handled, and a velocity excursion there may be real motion the FSM failed
to latch, which is the bug this project exists to fix rather than a false
positive. The soak mode picks the longest genuinely quiet stretch and reports
how much it found, so a capture with no real parked time says so instead of
producing a meaningless number.
"""

import re
import sys
import statistics

# --- constants, from velocity.h -------------------------------------------
VEL_WINDOW_MS = 5000
VEL_BINS = 10
VEL_BIN_MS = VEL_WINDOW_MS // VEL_BINS
VEL_MAX_DT_MS = 400
VEL_MIN_COVERAGE_MS = 1800
VEL_PRIME_SAMPLES = 32
VEL_BASELINE_TAU_MS = 120000
VEL_BASELINE_FREEZE_MPS = 0.030
VEL_SIGMA_MSS = 0.023
VEL_PARKED_PEAK = 0.1274
VEL_DEPART_THRESHOLD = 2.0 * VEL_PARKED_PEAK
VEL_ARRIVE_FRACTION = 0.50
VEL_ARRIVE_MIN = 2.0 * VEL_PARKED_PEAK

STATE_MONITORING = 2
STATE_MOVING = 4
FPM = 0.00508


class VelocityWindow:
    def __init__(self):
        self.bin_v = [0.0] * VEL_BINS
        self.bin_cov = [0] * VEL_BINS
        self.cur = 0
        self.cur_bin_start = 0
        self.last_sample_ms = 0
        self.have_last = False
        self.base = 0.0
        self.prime_count = 0
        self.track_baseline = True
        self.w_cache = 0.0
        self.total_cov = 0

    def reset(self, now_ms):
        self.bin_v = [0.0] * VEL_BINS
        self.bin_cov = [0] * VEL_BINS
        self.cur = 0
        self.cur_bin_start = now_ms
        self.w_cache = 0.0
        self.total_cov = 0

    def advance_to(self, now_ms):
        elapsed = now_ms - self.cur_bin_start
        if elapsed < VEL_BIN_MS:
            return
        steps = elapsed // VEL_BIN_MS
        if steps >= VEL_BINS:
            self.bin_v = [0.0] * VEL_BINS
            self.bin_cov = [0] * VEL_BINS
            self.cur = 0
            self.cur_bin_start = now_ms
            return
        for _ in range(steps):
            self.cur = (self.cur + 1) % VEL_BINS
            self.bin_v[self.cur] = 0.0
            self.bin_cov[self.cur] = 0
        self.cur_bin_start += steps * VEL_BIN_MS

    def recompute(self):
        self.w_cache = sum(self.bin_v)
        cov = sum(self.bin_cov)
        self.total_cov = 0xFFFF if cov > 0xFFFF else cov

    def tick(self, now_ms):
        self.advance_to(now_ms)
        self.recompute()

    def add(self, accel_mss, now_ms):
        if self.prime_count == 0:
            self.base = accel_mss
            self.prime_count = 1
            self.last_sample_ms = now_ms
            self.have_last = True
            self.cur_bin_start = now_ms
            return
        if self.prime_count < VEL_PRIME_SAMPLES:
            self.prime_count += 1
        if not self.have_last:
            self.last_sample_ms = now_ms
            self.have_last = True
            return
        dt = now_ms - self.last_sample_ms
        self.last_sample_ms = now_ms
        if dt == 0:
            return
        if dt > VEL_MAX_DT_MS:
            dt = VEL_MAX_DT_MS
        self.advance_to(now_ms)
        self.bin_v[self.cur] += (accel_mss - self.base) * (dt / 1000.0)
        self.bin_cov[self.cur] += dt
        self.recompute()
        if self.track_baseline and abs(self.w_cache) < VEL_BASELINE_FREEZE_MPS:
            alpha = dt / VEL_BASELINE_TAU_MS
            if alpha > 0.05:
                alpha = 0.05
            self.base += alpha * (accel_mss - self.base)

    def w(self):
        return self.w_cache

    def valid(self):
        return (self.prime_count >= VEL_PRIME_SAMPLES and
                self.total_cov >= VEL_MIN_COVERAGE_MS)


# --- log replay ------------------------------------------------------------
SAMPLE = re.compile(r"t=(\d+)\s+a=(-?\d+\.?\d*)\s+avg=(-?\d+\.?\d*)\s+st=(\d+)")
ERRSAMP = re.compile(r"t=(\d+)\s+a=ERR")
FSM = re.compile(r"FSM:\s*(.+)")
BOOT = "Device Booted"


def sections(path):
    out, ev, sm = [], [], []
    for line in open(path, encoding="utf8", errors="replace"):
        if BOOT in line:
            if sm:
                out.append((sm, ev))
            sm, ev = [], []
        m = SAMPLE.search(line)
        if m:
            sm.append((int(m.group(1)), float(m.group(2)), int(m.group(4))))
            continue
        if ERRSAMP.search(line):
            continue
        m = FSM.search(line)
        if m and sm:
            ev.append((sm[-1][0], m.group(1).strip()))
    if sm:
        out.append((sm, ev))
    return out



# --- analysis modes --------------------------------------------------------

def longest_parked(path, min_s=600):
    """Longest run of consecutive MONITORING samples with no FSM event near."""
    best = []
    for smp, ev in sections(path):
        ev_t = sorted(t for t, _ in ev)
        run = []
        for (t, a, st) in smp:
            near = any(abs(t - e) < 15000 for e in ev_t)
            if st == STATE_MONITORING and not near:
                run.append((t, a, st))
            else:
                if len(run) > len(best):
                    best = run
                run = []
        if len(run) > len(best):
            best = run
    if not best or (best[-1][0] - best[0][0]) / 1000.0 < min_s:
        return None
    return best


def mode_soak(path):
    """False-departure rate over genuinely parked time."""
    seg = longest_parked(path)
    if not seg:
        print("  no parked stretch >= 600 s -- this capture cannot set a gate")
        return

    span = (seg[-1][0] - seg[0][0]) / 1000.0
    vw = VelocityWindow()
    peak, crossings, ws, armed_from = 0.0, 0, [], None

    for (t, a, st) in seg:
        vw.tick(t)
        vw.track_baseline = True
        vw.add(a, t)
        if not vw.valid():
            continue
        if armed_from is None:
            armed_from = t
        ws.append(abs(vw.w()))
        peak = max(peak, abs(vw.w()))
        if abs(vw.w()) > VEL_DEPART_THRESHOLD:
            crossings += 1

    if not ws:
        print("  window never became valid")
        return
    ws_sorted = sorted(ws)
    sd = statistics.pstdev(ws)
    armed_s = (seg[-1][0] - armed_from) / 1000.0

    print(f"  parked stretch    : {span:.0f} s, {len(seg)} samples")
    print(f"  window armed      : {armed_s:.0f} s, {len(ws)} evaluations")
    print(f"  |w| median / p99  : {ws_sorted[len(ws)//2]:.4f} / "
          f"{ws_sorted[int(len(ws)*0.99)]:.4f} m/s")
    print(f"  |w| PEAK          : {peak:.4f} m/s   <-- VEL_PARKED_PEAK")
    print(f"  implied sigma     : {sd:.4f}  (peak is {peak/sd:.1f} sigma -- if")
    print(f"                      that is well over 4, the tail is heavy and a")
    print(f"                      sigma-based gate is NOT safe)")
    print(f"  current gate      : {VEL_DEPART_THRESHOLD:.4f} m/s "
          f"({VEL_DEPART_THRESHOLD/FPM:.0f} fpm)")
    print(f"  FALSE DEPARTURES  : {crossings}")
    if crossings:
        print(f"  ** gate is too low. Set VEL_PARKED_PEAK to {peak:.4f} "
              f"and re-run. **")


def mode_scale(path):
    """Does a higher sample rate actually help? Decimate and watch sigma_w."""
    seg = longest_parked(path)
    if not seg:
        print("  no parked stretch >= 600 s")
        return
    dt = statistics.median(b[0] - a[0] for a, b in zip(seg, seg[1:]))
    print(f"  parked {len(seg)} samples at {1000/dt:.2f} Hz")
    print(f"  {'decim':>6} {'eff Hz':>7} {'sigma_w':>9} {'peak':>8} "
          f"{'vs 1x':>7} {'sqrt(k)':>8}")
    base = None
    for k in (1, 2, 4, 8):
        dec = seg[::k]
        if len(dec) < 100:
            continue
        vw = VelocityWindow()
        ws = []
        for (t, a, st) in dec:
            vw.tick(t)
            vw.track_baseline = True
            vw.add(a, t)
            if vw.valid():
                ws.append(abs(vw.w()))
        if len(ws) < 50:
            continue
        sd = statistics.pstdev(ws)
        base = base or sd
        print(f"  {k:>6} {1000/(dt*k):>7.2f} {sd:>9.4f} {max(ws):>8.4f} "
              f"{sd/base:>6.2f}x {math.sqrt(k):>7.2f}x")
    print("  If 'vs 1x' tracks sqrt(k) the noise is white, the sample rate is")
    print("  the binding constraint, and raising it buys margin proportionally.")
    print()
    print("  Read the PEAK column, not sigma. The gate derives from the peak,")
    print("  and on the 2026-08-07 soak the two behave differently: peak scaled")
    print("  1.41x under 2x decimation (exactly sqrt(2)) while sigma did not")
    print("  move. That is what a heavy tail looks like -- the bulk is set by")
    print("  something else, the excursions that matter are white.")
    print()
    print("  Rows stop early by design. dt is clamped to VEL_MAX_DT_MS, so below")
    print("  ~1.5 Hz a 5 s window cannot reach VEL_MIN_COVERAGE_MS and valid()")
    print("  never goes true. Only one octave is probeable from a 3.13 Hz")
    print("  capture; treat any extrapolation past that as a hypothesis.")


def mode_runs(path):
    """Departure and arrival integrals for each latched run."""
    print(f"  {'dep w':>9} {'arr w':>9} {'arr/dep':>8}  release")
    for smp, ev in sections(path):
        vw = VelocityWindow()
        marks = {t: x for t, x in ev}
        dep_w, in_run = 0.0, False
        for (t, a, st) in smp:
            vw.tick(t)
            vw.track_baseline = (st == STATE_MONITORING)
            vw.add(a, t)
            if t in marks:
                x = marks[t]
                if "Departure latched" in x:
                    dep_w = vw.w() if vw.valid() else 0.0
                    in_run = True
                elif in_run and ("Arrival" in x or "FAILSAFE" in x):
                    arr = vw.w() if vw.valid() else float('nan')
                    ratio = abs(arr) / abs(dep_w) if dep_w else float('nan')
                    print(f"  {dep_w:>+9.3f} {arr:>+9.3f} {ratio:>8.2f}  {x[:38]}")
                    in_run = False
            if st == STATE_MOVEMENT_DETECTED and vw.valid() and \
               abs(vw.w()) > abs(dep_w):
                dep_w = vw.w()


import math

MODES = {"soak": mode_soak, "scale": mode_scale, "runs": mode_runs}
STATE_MOVEMENT_DETECTED = 3

if __name__ == "__main__":
    if len(sys.argv) < 3 or sys.argv[1] not in MODES:
        print(__doc__)
        sys.exit(1)
    fn = MODES[sys.argv[1]]
    for p in sys.argv[2:]:
        print(f"=== {p.replace(chr(92), '/').split('/')[-1]}")
        fn(p)
        print()
