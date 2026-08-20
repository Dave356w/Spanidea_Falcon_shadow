#!/usr/bin/env python3
"""
ramp_priority_replay.py -- does giving the RAMP verdict priority over the
polled peak prevent the 2026-08-20 false release, and what does it cost?

THE QUESTION. falcon_false_release_2026-08-20.md established that no value of
ARRIVAL_PEAK_VALUE separates a stop from cruise on that machine: the false
release measured 0.472 and sits INSIDE the real-arrival population
(0.450-0.511), one of which landed exactly on the 0.45 gate. So the arrival
gate cannot be retuned, and the remaining candidate is to release on something
other than magnitude -- the ramp detector's sustained one-signed test.
falcon_START_HERE_2026-08-20.md §4 names extending this replay as the next step.

WHY THIS IS NOT A BURST REPLAY. arming_replay.py works on 80-sample BURST
windows and can only ask "would this rule have armed". It cannot ask when the
ramp verdict would have LANDED relative to the peak, because that needs the
continuous stream across the whole run, which does not exist off the device.

But it does not have to be replayed, because IT WAS ALREADY MEASURED. Since
2026-08-12 the firmware prints `RAMP latched mean= dir=` from loop() the moment
the verdict latches, REGARDLESS OF FSM STATE (main.cpp emit_ramp_log, added
precisely because the ramp was invisible where it mattered). The accumulator
keeps running after the peak has already released the latch, so every run on
file since then carries a direct answer to "when would the ramp have fired if
the peak had not gone first". This tool reads those lines. Nothing here is
simulated except the choice of which verdict to obey.

================================================================================
THE GROUND TRUTH, AND ITS ONE BLIND SPOT -- read before quoting any number
================================================================================

A release is only correct if the car had actually stopped, and the log cannot
say so directly: `FSM: Arrival (polled), peak 0.472` is byte-identical on a
healthy release and on the false one. The 2026-08-20 event was identified only
because Dave was inside the car.

So the verdict comes from the LATERAL channel, which the false-release note
already established as the discriminating evidence: `m` reads 0.005-0.05 at
rest and 0.175-0.884 on a moving car, a 10x-170x separation that needs no
observer. The instrument is ONE function -- `verdict_at(t)`, the median of `m`
over the window [t+5 s, t+11 s] -- asked at whatever instant a rule would have
dropped the latch. A rule that releases later is judged on a correspondingly
later window, which is the point: releasing 2 s after a car has stopped is
correct, and the instrument has to be able to say so.

Calibrated against device-monitor-260820-142248.log, where one release is known
false from Dave's own words: that window reads 0.322 for the false release and
0.018-0.095 for all 31 others. The separation is real but it is not a wide moat
-- treat a margin under 2x as unresolved.

  ! BLIND SPOT: this detects a release that is EARLY ENOUGH that the car is
    still moving 5-11 s later. A release 1-2 s early sits inside the settling
    tail of a correct one and is NOT separable this way. Every "released while
    moving" count here is therefore a LOWER BOUND, and a zero in that column
    means "none this instrument can see", never "none".

  ! The window is truncated at the next departure latch, so a release whose
    next trip starts too soon to leave a usable window is reported UNKNOWN and
    excluded from the scoring rather than guessed at. UNKNOWN is per RULE, not
    per run -- a deferred release can fall off the end of a window the
    shipping release fitted inside.

  ! An earlier draft of this tool derived a per-run `t_stop` (first sample
    after which `m` stayed quiet for 2 s) and scored releases against it. It
    is not in here because it disagreed with the calibrated window on its own
    data -- runs it called "early by 21 s" read 0.018, dead at rest, in the
    verdict window -- and it put the SHIPPING build at 50 early releases in
    219, which the 30+ clean live runs on file flatly contradict. A settling
    tail is not a stop time. Do not reintroduce it without calibrating it
    against the one release whose truth is known independently.

WHAT THE RULES MEAN. All four arm identically -- this changes only which
verdict is allowed to RELEASE, never when the detectors start looking.

  peak          shipping. First peak crossing after arming releases.
  ramp_only     the peak never releases. Ramp verdict or nothing.
  ramp_pri(W)   the peak's release is HELD W ms. If the ramp latches inside
                that window the ramp releases; otherwise the peak releases
                late, at t_peak + W.
  ramp_veto(W)  as above, but on W expiring with no ramp the peak's release is
                DISCARDED -- the beacon stays on.

`ramp_pri` cannot prevent a release that no ramp ever confirms; it can only
delay it by W. It is included because that delay is itself the safety margin on
a slow stop, and because the difference between it and `ramp_veto` is exactly
the price of refusing to release without confirmation.

Usage:
    python ramp_priority_replay.py [--logs DIR] [--hold MS] [--verbose] [--sweep]
"""

import argparse
import glob
import os
import re
import statistics
import sys

# -- Ground-truth oracle constants, calibrated in the header ------------------
REST_M = 0.12          # lateral m below this is "at rest"; rest 0.005-0.05,
                       # moving 0.175-0.884, so this sits in the gap
ORACLE_LO_MS = 5000    # the calibrated verdict window, relative to a release
ORACLE_HI_MS = 11000
ORACLE_MIN_N = 3       # fewer samples than this in the window -> UNKNOWN

# -- Candidate rule parameters -----------------------------------------------
DEFAULT_HOLD_MS = 2000  # W: the ramp needs RAMP_BLOCKS*RAMP_BLOCK_N samples,
                        # ~1.44 s at 25 Hz, so W must exceed that to be a fair
                        # test of the ramp at all

KV = re.compile(r'(\w+)=(-?[\d.]+)')
ARR_RE = re.compile(
    r'^FSM: Arrival \((?P<path>polled|any-motion)\)(?:, edges \d+)?,? peak '
    r'(?P<peak>[\d.]+)')
RAMP_RE = re.compile(r'^RAMP latched mean=(?P<mean>-?\d+) dir=(?P<dir>-?\d+)')
ARM_RE = re.compile(r'^ARM q=(?P<q>\d+) a=(?P<a>\d+) v=(?P<v>\d+) '
                    r'g=(?P<g>\d+) ro=(?P<ro>\d+)')
DEP_RE = re.compile(r'^FSM: Departure latched \((?P<path>[\w-]+)\)')


def ramp_instrumented(path, saw_ramp):
    """
    Can this capture answer the ramp question at all?

    emit_ramp_log() was added 2026-08-12, PART WAY THROUGH that day -- after
    the first two automatic cab runs. A capture without it prints no
    `RAMP latched` line however hard the ramp fires, so scoring a ramp rule on
    one manufactures a NO RELEASE out of missing instrumentation. That is the
    single easiest way to get a wrong answer out of this tool, and it is worth
    more than a hundred extra runs of corpus.

    A capture qualifies if it actually printed a ramp latch, or if it is dated
    after 2026-08-12, in which case a silent ramp is a real decline -- which is
    the expected and load-bearing behaviour on inspection brake sets.
    """
    if saw_ramp:
        return True
    m = re.search(r'(\d{6})-\d{6}', os.path.basename(path))
    return bool(m) and int(m.group(1)) > 260812


class Run:
    """One STATE_MOVING episode: its release and its ramp verdict."""

    def __init__(self, log, dep_line, dep_t):
        self.log = log
        self.dep_line = dep_line
        self.dep_t = dep_t
        self.rel_line = None      # peak release
        self.rel_t = None
        self.peak = None
        self.rel_path = None
        self.ramp_t = None        # ramp verdict, if it ever latched
        self.ramp_mean = None
        self.ramp_dir = None
        self.arm = {}
        self.next_dep_t = None
        self.next_dep_line = None
        self.tail = ()            # (t, |m|) from the release to the next
                                  # departure, in FILE order -- see build_tail
        self.instrumented = False

    @property
    def name(self):
        return f"{os.path.basename(self.log)}:{self.rel_line or self.dep_line}"

    def build_tail(self, samples):
        """
        The samples this run is allowed to be judged on: from the release line
        to the next departure line, IN FILE ORDER.

        ! Selecting these by timestamp instead is wrong, and wrong in the
          direction that hides the failure this tool exists to find. The device
          `millis()` RESETS on a brownout or watchdog -- 260820-142248 carries a
          `Reset cause: 0x2` mid-capture -- so `t` is not monotonic across a
          file and t=161054 occurs twice, 6300 lines apart. A timestamp filter
          silently pulls the parked-car samples from before the reboot into the
          verdict window of a run after it. That is exactly what made an
          earlier draft score the KNOWN false release as 0.0435 STOPPED.
          File order is the only ordering that survives a reset.
        """
        end = self.next_dep_line if self.next_dep_line is not None else 1 << 62
        tail = [(t, m) for ln, t, m in samples if self.rel_line <= ln < end]
        # A reset inside the tail truncates it; nothing after is comparable.
        for i in range(1, len(tail)):
            if tail[i][0] < tail[i - 1][0]:
                tail = tail[:i]
                break
        self.tail = tail

    def verdict_at(self, t):
        """
        Was the car still moving when the latch dropped at `t`?

        The ONE instrument. Median lateral |m| over [t+5 s, t+11 s], cut off at
        the next departure latch so a genuine new trip is never mistaken for a
        car that never stopped. Returns (verdict, median) with verdict in
        MOVING / STOPPED / UNKNOWN.
        """
        if t is None:
            return 'UNKNOWN', None
        win = [m for tt, m in self.tail
               if t + ORACLE_LO_MS <= tt <= t + ORACLE_HI_MS]
        if len(win) < ORACLE_MIN_N:
            return 'UNKNOWN', None
        med = statistics.median(win)
        return ('MOVING' if med >= REST_M else 'STOPPED'), med


def parse_log(path):
    """
    Segment a raw device-monitor capture into runs.

    Sample lines are matched generically as key=value because the field set has
    changed repeatedly across builds (tk/et/tw on 2026-08-18, cp/bz later) and a
    fixed pattern silently drops whole sessions. Mount orientation also flips
    the sign of `a`, so nothing here may assume it is positive.

    ! Sample lines LAG FSM lines: the ring is drained one entry per loop()
      pass (falcon_START_HERE_2026-08-20 §7). Every event timestamp here is
      therefore the nearest FOLLOWING sample's `t`, and is good to about one
      sample. Do not read sub-100 ms differences out of this tool.
    """
    samples = []      # (line, t, m)
    events = []       # (line, kind, payload)
    for ln, raw in enumerate(open(path, encoding='utf-8', errors='replace')):
        line = raw.strip()
        if line.startswith('t=') and ' a=' in line:
            d = dict(KV.findall(line))
            if 't' in d and 'm' in d:
                try:
                    samples.append((ln, int(float(d['t'])), abs(float(d['m']))))
                except ValueError:
                    pass
            continue
        if line.startswith('FSM: Departure latched'):
            m = DEP_RE.match(line)
            if m:
                events.append((ln, 'dep', m.group('path')))
        elif line.startswith('FSM: Arrival'):
            m = ARR_RE.match(line)
            if m:
                events.append((ln, 'arr', (float(m.group('peak')),
                                           m.group('path'))))
        elif line.startswith('RAMP latched'):
            m = RAMP_RE.match(line)
            if m:
                events.append((ln, 'ramp', (int(m.group('mean')),
                                            int(m.group('dir')))))
        elif line.startswith('ARM q='):
            m = ARM_RE.match(line)
            if m:
                events.append((ln, 'arm', {k: int(v)
                                           for k, v in m.groupdict().items()}))

    if not samples:
        return []

    # Event timestamps come from the first sample at or after the event line.
    # Walk both lists once rather than scanning per event -- some captures carry
    # 10k sample lines and the quadratic version dominated the runtime.
    sample_lines = [s[0] for s in samples]

    def t_at(line):
        import bisect
        i = bisect.bisect_left(sample_lines, line)
        return samples[i][1] if i < len(samples) else None

    runs = []
    cur = None
    for ln, kind, payload in events:
        if kind == 'dep':
            t = t_at(ln)
            if cur is not None:
                cur.next_dep_t = t
                cur.next_dep_line = ln
            cur = Run(path, ln, t)
            runs.append(cur)
        elif cur is None:
            continue
        elif kind == 'arr' and cur.rel_line is None:
            cur.rel_line, cur.rel_t = ln, t_at(ln)
            cur.peak, cur.rel_path = payload
        elif kind == 'arm' and not cur.arm:
            cur.arm = payload
        elif kind == 'ramp' and cur.ramp_t is None:
            # The ramp accumulator is cleared on entry to STATE_MOVING, so a
            # latch printed before the NEXT departure still belongs to this run
            # -- that is the whole point, it is the verdict the peak pre-empted.
            cur.ramp_t = t_at(ln)
            cur.ramp_mean, cur.ramp_dir = payload

    instrumented = ramp_instrumented(path, any(k == 'ramp' for _, k, _ in events))
    for r in runs:
        r.instrumented = instrumented
        if r.rel_line is not None:
            r.build_tail(samples)
    return runs


# -- The rules ---------------------------------------------------------------
def release_time(run, rule, hold_ms=DEFAULT_HOLD_MS):
    """When does `rule` drop the latch? None means it never does."""
    if run.rel_t is None:
        return None
    if rule == 'peak':
        return run.rel_t
    if rule == 'ramp_only':
        return run.ramp_t
    confirmed = run.ramp_t is not None and run.ramp_t <= run.rel_t + hold_ms
    if rule == 'ramp_pri':
        return run.ramp_t if confirmed else run.rel_t + hold_ms
    if rule == 'ramp_veto':
        return run.ramp_t if confirmed else None
    raise ValueError(rule)


def classify(run, rule, hold_ms=DEFAULT_HOLD_MS):
    """
    MOVING  -- released with the car still moving: silence while moving, the
               only catastrophic failure (main.cpp §14.4).
    OK      -- released with the car stopped.
    NONE    -- never released at all. The beacon is held over a stopped car to
               LATCH_FAILSAFE_MS (600 s): a position lie, bad but not silent.
    UNKNOWN -- no usable verdict window; excluded from the scoring.

    Returns (verdict, median_m, t_release).
    """
    t = release_time(run, rule, hold_ms)
    if t is None:
        return 'NONE', None, None
    v, med = run.verdict_at(t)
    return ('MOVING' if v == 'MOVING' else
            'OK' if v == 'STOPPED' else 'UNKNOWN'), med, t


SELFTEST_LOG = 'device-monitor-260820-142248.log'
SELFTEST_LINE = 6504     # `FSM: Arrival (polled), peak 0.472`


def selftest(logdir):
    """
    The corpus contains exactly ONE release whose truth is known from outside
    the log -- Dave, in the car, 2026-08-20: "released in travel". Every number
    this tool prints rests on the instrument agreeing with him about that one.

    Two separate defects during development both showed up here and NOWHERE
    else in the output -- a `t_stop` estimator that disagreed with its own
    calibration, and a timestamp filter that walked through a `millis()` reset
    and scored this release 0.0435 STOPPED. In both cases the pooled tables
    stayed plausible. This is the assertion that catches the third one.
    """
    path = os.path.join(logdir, SELFTEST_LOG)
    if not os.path.exists(path):
        print(f"SKIP: {SELFTEST_LOG} not in {logdir}", file=sys.stderr)
        return 77
    run = next((r for r in parse_log(path) if r.rel_line == SELFTEST_LINE), None)
    checks = []
    if run is None:
        checks.append(('the known false release is parsed at all', False, '-'))
    else:
        v, med, _ = classify(run, 'peak')
        checks.append(('peak released it on a MOVING car',
                       v == 'MOVING', f'{v} lateral={med}'))
        checks.append(('and by a clear margin, not a borderline',
                       med is not None and med >= 2 * REST_M,
                       f'{med:.3f} vs {2*REST_M:.3f}'))
        v2, _, _ = classify(run, 'ramp_veto')
        checks.append(('ramp_veto prevents it', v2 == 'NONE', v2))
        v3, _, _ = classify(run, 'ramp_pri')
        checks.append(('ramp_pri does NOT prevent it -- the finding',
                       v3 == 'MOVING', v3))
    ok = True
    for label, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}   ({detail})")
        ok &= passed
    print('selftest', 'PASSED' if ok else 'FAILED')
    return 0 if ok else 1


def main():
    for s in (sys.stdout, sys.stderr):
        try:
            s.reconfigure(encoding='utf-8')
        except (AttributeError, ValueError):
            pass

    ap = argparse.ArgumentParser()
    ap.add_argument('--logs', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'logs'))
    ap.add_argument('--hold', type=int, default=DEFAULT_HOLD_MS)
    ap.add_argument('--sweep', action='store_true')
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--selftest', action='store_true',
                    help='assert the calibration still catches the one release '
                         'whose truth is known independently')
    args = ap.parse_args()

    if args.selftest:
        return selftest(args.logs)

    runs = []
    for p in sorted(glob.glob(os.path.join(args.logs, '*.log'))):
        runs.extend(parse_log(p))
    runs = [r for r in runs if r.rel_t is not None]
    if not runs:
        print('no runs found', file=sys.stderr)
        return 1

    pool = [r for r in runs if r.instrumented]
    ramped = [r for r in pool if r.ramp_t is not None]
    print(f"Loaded {len(runs)} released runs from {args.logs}")
    print(f"  ramp-instrumented (2026-08-12 onward): {len(pool)}/{len(runs)}")
    print(f"  the other {len(runs) - len(pool)} predate emit_ramp_log and are")
    print(f"  EXCLUDED -- they cannot print a ramp latch, so scoring a ramp")
    print(f"  rule on them would invent a NO RELEASE out of a missing print.")
    print(f"  Of the pool, {len(ramped)}/{len(pool)} carry a ramp verdict.")

    # -- 1. the race the shipping firmware runs -------------------------------
    print("\n" + "=" * 78)
    print("1. THE RACE: when does the ramp verdict land, relative to the peak?")
    print("=" * 78)
    print("   START_HERE §4: 'the polled peak always wins the race'. Measured:\n")
    deltas = sorted(r.ramp_t - r.rel_t for r in ramped)
    if deltas:
        print(f"   ramp minus peak, ms:  min {deltas[0]}  median "
              f"{int(statistics.median(deltas))}  max {deltas[-1]}")
        print(f"   ramp landed FIRST on {sum(1 for d in deltas if d < 0)}"
              f"/{len(deltas)} runs")
        print(f"   ramp landed within {args.hold} ms of the peak on "
              f"{sum(1 for d in deltas if 0 <= d <= args.hold)}/{len(deltas)}")

    # -- 2. what each rule would have done ------------------------------------
    print("\n" + "=" * 78)
    print("2. OUTCOMES against the lateral verdict")
    print("=" * 78)
    print("   MOVING is the catastrophic column -- silence while moving, and a")
    print("   LOWER BOUND (see the blind spot in the header).")
    print("   NONE is the beacon held to the 600 s failsafe: a position lie.")
    print(f"   Pool: {len(pool)} ramp-instrumented runs.\n")
    print(f"   {'rule':<14} {'MOVING':>8} {'OK':>6} {'NONE':>6} {'UNKNOWN':>8} "
          f"{'median delay':>13}")
    print("   " + "-" * 60)
    for rule in ('peak', 'ramp_only', 'ramp_pri', 'ramp_veto'):
        tally = {'MOVING': 0, 'OK': 0, 'NONE': 0, 'UNKNOWN': 0}
        delays = []
        for r in pool:
            v, _, t = classify(r, rule, args.hold)
            tally[v] += 1
            if t is not None:
                delays.append(t - r.rel_t)
        flag = '  <- CATASTROPHIC' if tally['MOVING'] else '  ok'
        med = f'{int(statistics.median(delays))} ms' if delays else '-'
        print(f"   {rule:<14} {tally['MOVING']:>8} {tally['OK']:>6} "
              f"{tally['NONE']:>6} {tally['UNKNOWN']:>8} {med:>13}{flag}")
    print("\n   median delay is release latency added versus the shipping peak.")

    # -- 2b. the disaggregation the whole decision turns on --------------------
    print("\n" + "=" * 78)
    print("2b. THE SAME TABLE PER CAPTURE -- read this before section 2")
    print("=" * 78)
    print("   The pooled NONE rate is meaningless on its own, because the ramp")
    print("   DECLINING is correct behaviour on an inspection brake set and a")
    print("   failure on a contract-speed stop, and both are in the pool. A")
    print("   capture with 0 ramp verdicts is an inspection population; one")
    print("   with a ramp on nearly every run is automatic operation. What the")
    print("   decision needs is the veto's cost where the ramp IS available.\n")
    print(f"   {'capture':<38} {'runs':>5} {'ramp':>5} "
          f"{'peak MOVING':>12} {'veto NONE':>10} {'veto MOVING':>12}")
    print("   " + "-" * 88)
    for log in sorted({r.log for r in pool}):
        rs = [r for r in pool if r.log == log]
        nramp = sum(1 for r in rs if r.ramp_t is not None)
        pmv = sum(1 for r in rs if classify(r, 'peak', args.hold)[0] == 'MOVING')
        vn = sum(1 for r in rs
                 if classify(r, 'ramp_veto', args.hold)[0] == 'NONE')
        vm = sum(1 for r in rs
                 if classify(r, 'ramp_veto', args.hold)[0] == 'MOVING')
        print(f"   {os.path.basename(log):<38} {len(rs):>5} {nramp:>5} "
              f"{pmv:>12} {vn:>10} {vm:>12}")

    # -- 3. the runs that matter ----------------------------------------------
    print("\n" + "=" * 78)
    print("3. EVERY RELEASE THE SHIPPING PEAK MADE ON A MOVING CAR")
    print("=" * 78)
    print("   This is the population the change exists for. For each one, what")
    print("   the candidates would have done instead.\n")
    bad = [r for r in pool if classify(r, 'peak', args.hold)[0] == 'MOVING']
    if not bad:
        print("   none")
    for r in bad:
        _, med, _ = classify(r, 'peak', args.hold)
        rt = (f"ramp +{r.ramp_t - r.rel_t} ms" if r.ramp_t is not None
              else "NO RAMP VERDICT EVER")
        # A verdict inside 2x of the rest threshold is not a finding -- the
        # calibration separates 0.322 from 0.018-0.095, and nothing in that
        # gap has ever been confirmed against an observer in the car.
        margin = med / REST_M
        note = '' if margin >= 2.0 else \
               f"   [UNRESOLVED: only {margin:.2f}x the rest threshold]"
        print(f"   {r.name:<44} peak {r.peak:.3f}  lateral {med:.3f}{note}")
        print(f"       g={r.arm.get('g','?')} ro={r.arm.get('ro','?')}  {rt}")
        for rule in ('ramp_only', 'ramp_pri', 'ramp_veto'):
            v, m2, t = classify(r, rule, args.hold)
            d = f"  (+{t - r.rel_t} ms, lateral {m2:.3f})" if m2 is not None \
                else ""
            mark = '  <- still catastrophic' if v == 'MOVING' else \
                   '  <- prevented' if v == 'NONE' else ''
            print(f"       {rule:<12} -> {v}{d}{mark}")

    if args.verbose:
        print("\n" + "=" * 78)
        print("4. ALL RAMP-INSTRUMENTED RUNS")
        print("=" * 78)
        print(f"   {'run':<42} {'peak':>6} {'g':>2} {'d_ramp':>7} "
              f"{'lateral':>8} {'peak rule':>10}")
        print("   " + "-" * 82)
        for r in pool:
            d = r.ramp_t - r.rel_t if r.ramp_t is not None else None
            v, med, _ = classify(r, 'peak', args.hold)
            om = ('%.3f' % med) if med is not None else '-'
            print(f"   {r.name:<42} {r.peak:>6.3f} {r.arm.get('g','?'):>2} "
                  f"{str(d):>7} {om:>8} {v:>10}")

    if args.sweep:
        print("\n" + "=" * 78)
        print("5. SWEEP: the hold window W")
        print("=" * 78)
        print("   W must exceed ~1.44 s for the ramp to be able to answer at")
        print("   all. Longer W buys confirmation and costs release latency.\n")
        print("   A long W on ramp_pri is the 'fall back to the peak eventually'")
        print("   design -- it needs W to outlast the false-release event, and")
        print("   the instrument LOSES the ability to score it there, because")
        print("   the verdict window runs past the next departure. Rising")
        print("   pri UNKNOWN is that limit, not a clean bill.\n")
        print(f"   {'W ms':>6} {'pri MOVING':>11} {'pri OK':>7} {'pri UNK':>8} "
              f"{'veto MOVING':>12} {'veto OK':>8} {'veto NONE':>10}")
        print("   " + "-" * 68)
        for w in (1500, 2000, 2500, 3000, 4000, 6000, 8000, 12000):
            pm = po = pu = vm = vo = vn = 0
            for r in pool:
                v, _, _ = classify(r, 'ramp_pri', w)
                pm += v == 'MOVING'
                po += v == 'OK'
                pu += v == 'UNKNOWN'
                v, _, _ = classify(r, 'ramp_veto', w)
                vm += v == 'MOVING'
                vo += v == 'OK'
                vn += v == 'NONE'
            print(f"   {w:>6} {pm:>11} {po:>7} {pu:>8} {vm:>12} {vo:>8} "
                  f"{vn:>10}")

    print("\nDone. The blind spot in the header is not optional reading.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
