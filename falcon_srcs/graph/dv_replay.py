"""
dv_replay.py -- can the firmware tell AUTOMATIC operation from INSPECTION,
                using only the departure burst it already records?

WHY THIS EXISTS. falcon_ramp_priority_2026-08-20 §5 ends on a MODE question,
not a threshold question. A ramp-VETO -- refusing to release without ramp
confirmation -- is the only rule measured to prevent the 2026-08-20 release on
a moving car, and it is unusable because it declines 100% of inspection stops,
which carry 0 ramp verdicts in 79 runs. Declining a brake set is CORRECT on
inspection and CATASTROPHIC at contract speed. If the firmware could tell the
two apart at runtime, the veto ships behind that flag and S9 closes.

falcon_fsm_logic_and_shortcomings_2026-08-21 §4.3 proposes an answer needing no
new sensor: the departure burst is 80 signed samples at 25 Hz -- 3.2 s of the
departure at full rate -- so INTEGRATING IT GIVES dv, and dv is the regime.
300 fpm is 1.52 m/s; 19 fpm is 0.097 m/s. That is 16x, on a quantity already
in RAM.

This scores that proposal over every departure burst in the corpus.

⚠️ THE GROUND TRUTH IS NOT THE RAMP. Labelling a capture "automatic" because it
carries ramp verdicts and then showing dv predicts it would be circular -- ramp
availability is what dv is meant to replace. Every mode label in MODE below
comes from a session note describing WHAT DAVE DID, and carries the sentence it
came from. Captures no note describes are `unknown` and are never folded into
either population.

⚠️ A CAPTURE IS NOT A POPULATION. An automatic session still contains
repositioning moves and deliberate jogs; an inspection session contains long
runs. So the per-burst labels from `datasets/session_g_labels.csv` are joined
on top, using session_g.py's OWN run indexing (increment on the STATE_MOVING
transition), and the headline comparison is `run`-labelled bursts only. The
unlabelled remainder is reported beside it, never inside it.

⚠️ SAMPLING BIAS, carried from S10/B3. A `BURST k=dep` is emitted only by the
any-motion branch, so a run latched by the polled path alone contributes no
burst -- 26 bursts against 33 latches in 260820-150000. n here is bursts, not
runs.

usage:
    python dv_replay.py                        # datasets/, the versioned corpus
    python dv_replay.py --logs ../logs         # raw captures, if present
    python dv_replay.py --window 60            # post-trigger samples to integrate
    python dv_replay.py --no-zero              # skip the pre-trigger zero fit
"""

import argparse
import csv
import glob
import os
import re
import statistics as st

SAMPLE_DT = 0.04            # 25 Hz; BURST_N 80 = 3.2 s (main.cpp:710)
FPM_TO_MS = 0.00508         # 1 ft/min in m/s

RE_BURST_DEP = re.compile(
    r'^BURST\s+k=dep\s+pre=(?P<pre>\d+)\s+n=(?P<n>\d+)\s+'
    r'signed_mmss=(?P<vals>[-\d ]+)')
TO_MOVING = 'Transitioned to STATE_MOVING'      # session_g.py's run boundary

# ---------------------------------------------------------------------------
# MODE LABELS -- one row per capture holding departure bursts. `basis` is the
# sentence that establishes it. If you add a capture and cannot name the
# evidence, the label is `unknown`: a valid answer, and the count of unknowns
# is a result rather than a failure (datasets/README.md, the session G rule).
# ---------------------------------------------------------------------------
MODE = {
 '260811-134652': ('automatic', 350, 'falcon_350fpm_automatic_2026-08-11: "Four full express runs on automatic operation at 350 fpm" (runs 1-3)'),
 '260811-140425': ('automatic', 350, 'falcon_350fpm_automatic_2026-08-11: same session, "runs 4-5, fresh boot"'),
 '260811-144408': ('inspection', None, 'falcon_jog_verdict_2026-08-11: "four jogs and three slow departures, captured from the cartop"'),
 '260811-152209': ('inspection', None, 'falcon_jog_verdict_2026-08-11 §6.1, same cartop tethered session'),
 '260811-154018': ('inspection', None, 'falcon_jog_verdict_2026-08-11 §7, same cartop tethered session'),
 '260812-095101': ('inspection', None, 'falcon_cartop_2026-08-12: "inspection runs, jogs"; "Dave driving, cartop mounting"'),
 '260812-104647': ('inspection', None, 'falcon_cartop_2026-08-12: same session, "everything after"'),
 '260812-112254': ('automatic', 300, 'falcon_cab_automatic_2026-08-12: "Dave inside the cab, automatic operation at 300 fpm"'),
 '260812-140913': ('automatic', 300, 'falcon_ramp_armed_2026-08-12: "all four car runs", continuing the 300 fpm cab session'),
 '260813-103846': ('automatic', 500, 'falcon_500fpm_ui_2026-08-13 §5: "a different building -- 8 floors, 500 fpm"'),
 '260813-110300': ('automatic', 500, 'falcon_500fpm_ui_2026-08-13 §5: same building and speed'),
 '260818-105317': ('inspection', 20, 'falcon_departure_detection_2026-08-18: cartop rig, slow up runs; ramp_priority §4 labels it "inspection"'),
 '260818-133242': ('inspection', 20, 'falcon_departure_detection_2026-08-18: same session, right-side-up rig'),
 '260820-113000': ('bench', None, 'falcon_b1_2026-08-20: the bench cascade -- taps on a desk, not a car'),
 '260820-120250': ('inspection', 19, 'falcon_cartop_2026-08-20: "Cartop, inspection, 19 fpm, Dave driving"'),
 '260820-122600': ('inspection', 19, 'falcon_cartop_2026-08-20: same session'),
 '260820-150000': ('automatic', 300, 'falcon_automatic_2026-08-20: first automatic operation at contract speed, 300 fpm'),
 '260820-152131': ('automatic', 300, 'falcon_ramp_priority_2026-08-20 §7: "the 1-4 / 4-1 sequence", 11 runs, same installation'),
 '260821-101915': ('automatic', 350, 'falcon_350fpm_2026-08-21: 14 automatic runs at 350 fpm, cab, Dave driving -- the out-of-sample test of this tool'),
 '260821-105230': ('automatic', 350, 'falcon_START_HERE_2026-08-21 §3: the B3 verification runs, same machine and session as 260821-101915, build 67ca85b'),
 '260811-122214': ('unknown', None, 'named in no session note'),
}


# ---------------------------------------------------------------------------
# ⛔ CONTAMINATION CUTS. A raw capture in logs/ is not necessarily all car data.
#
# `260820-152131` was started 2026-08-20 15:45 and was STILL RUNNING on 08-21,
# so it also contains that day's bench work -- hand taps on a desk, which the
# device latches and releases exactly like a car. Scored raw, the first pass of
# this tool reported a third release-on-a-moving-car in it (line 17768,
# lateral 0.671). It is a bench tap: it sits past the pre-flash boundary at
# line 12718 and it carries an `SL: held` line, which only the 08-21 firmware
# emits. falcon_ramp_priority_2026-08-20 §7 scored the same capture at 11 runs;
# everything after the cut is bench.
#
# ⚠️ A bench tap is a large lateral event with a real release, so it scores
# MOVING and it inflates exactly the number this tool exists to measure.
# ---------------------------------------------------------------------------
CUT = {
 '260820-152131': (12718, 'still-running capture; 08-21 bench taps begin here '
                          '(pre-flash offset, and SL: lines start after it)'),
}


class Burst(object):
    __slots__ = ('log', 'run', 'pre', 'n', 'a', 'mode', 'fpm', 'label',
                 'zero', 'dv', 'dv_raw', 'peak')


def load_labels(path):
    if not os.path.exists(path):
        return {}
    out = {}
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            stem = os.path.splitext(row['log'])[0]
            try:
                out[(stem, int(row['run']))] = row['label']
            except (ValueError, KeyError):
                pass
    return out


def load(paths, labels):
    """Walk each capture the way session_g.py does, so run indices join."""
    out = []
    for p in sorted(paths):
        stem = os.path.basename(p).replace('device-monitor-', '')
        stem = os.path.splitext(stem)[0]
        mode, fpm, _ = MODE.get(stem, ('unknown', None, 'not in the MODE table'))
        idx = 0
        with open(p, 'r', errors='replace') as fh:
            for line in fh:
                if TO_MOVING in line:
                    idx += 1
                    continue
                m = RE_BURST_DEP.match(line)
                if not m:
                    continue
                vals = [int(v) for v in m.group('vals').split()]
                b = Burst()
                b.log, b.run = stem, idx
                b.pre, b.n = int(m.group('pre')), int(m.group('n'))
                b.a = vals
                b.mode, b.fpm = mode, fpm
                b.label = labels.get((stem, idx), '')
                if len(vals) >= b.pre + 2:
                    out.append(b)
    return out


def score(b, window, zero_n):
    """Integrate the post-trigger samples to a velocity change, in m/s."""
    #
    # The burst is (a - zero) on the VERTICAL channel, so a standing zero error
    # integrates linearly: 0.05 m/s^2 of offset over 2.4 s is 0.12 m/s, LARGER
    # than a whole 19 fpm departure. Removing it is not a nicety.
    #
    # ⚠️ BUT ONLY THE FIRST FEW PRE-TRIGGER SAMPLES ARE AT REST. Any-motion
    # fires part-way up the acceleration ramp, so the LATER pre-trigger samples
    # already carry signal -- on a 300 fpm departure the last four read -175,
    # -205, -263, -296 against a rest level of tens. Fitting the zero across all
    # 20 subtracts the departure from itself: it drags the median "offset" from
    # ~9 mm/s^2 to 69 and collapses the separation from 2.1x to 1.1x. §6 sweeps
    # this, and it is the single most load-bearing choice in the tool.
    #
    # ⚠️ MEDIAN, not mean. One disturbed pre-sample wrecks a mean:
    # 260818-105317 run 17 opens [40, 60, 133, -697, -206, 3077], where a mean
    # puts the zero at 401 mm/s^2 and manufactures 1.03 m/s of dv over 2.4 s out
    # of a burst whose post-trigger samples never exceed 460. That one artefact
    # WAS the worst false-automatic in the corpus; the median of the same six
    # samples reads 50 and it disappears. The dangerous tail of the inspection
    # population was zero-fit failure, not motion.
    pre = b.a[:min(zero_n, b.pre)]
    b.zero = st.median(pre) if pre else 0.0
    post = b.a[b.pre:b.pre + window]
    b.dv = sum((v - b.zero) for v in post) / 1000.0 * SAMPLE_DT
    b.dv_raw = sum(post) / 1000.0 * SAMPLE_DT
    b.peak = max(abs(v - b.zero) for v in post) / 1000.0 if post else 0.0
    return b


def q(xs, p):
    xs = sorted(xs)
    i = max(0, min(len(xs) - 1, int(round(p * (len(xs) - 1)))))
    return xs[i]


def dist(name, xs, width=22):
    if not xs:
        print(f"   {name:<{width}} (none)")
        return
    print(f"   {name:<{width}} n={len(xs):<4} min {min(xs):7.3f}  p10 {q(xs,.10):7.3f}"
          f"  med {st.median(xs):7.3f}  p90 {q(xs,.90):7.3f}  max {max(xs):7.3f}")


def errors(auto, insp, thr):
    ma = sum(1 for v in auto if v < thr)     # automatic scored as inspection
    fa = sum(1 for v in insp if v >= thr)    # inspection scored as automatic
    return ma, fa


def veto_section(logdir, thr, window, zero_n, hold):
    """
    The question §4.3 actually asks: does gating the ramp-VETO on dv make it
    shippable? The veto is the only rule measured to prevent the 2026-08-20
    release on a moving car, and it is dead because it declines 100% of
    inspection stops. Gate it on dv and it should decline none of them.

    This borrows ramp_priority_replay's parser and its verdicts wholesale --
    same runs, same tail-based MOVING/OK/NONE, same instrumented-pool
    exclusion -- and only adds the gate, so any difference from that tool's
    published numbers is the gate and nothing else.

    ⚠️ NEEDS THE RAW CAPTURES. The verdict is computed from post-release sample
    lines, and datasets/ strips them. Point --veto at logs/.
    """
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        import ramp_priority_replay as rp
    except Exception as exc:                      # pragma: no cover
        print(f'\n(--veto needs ramp_priority_replay.py: {exc})')
        return

    runs, cut_note = [], []
    for p in sorted(glob.glob(os.path.join(logdir, '*.log'))):
        stem = os.path.splitext(os.path.basename(p).replace('device-monitor-', ''))[0]
        cut, why = CUT.get(stem, (None, None))
        rs = rp.parse_log(p)
        if cut is not None:
            before = len(rs)
            rs = [r for r in rs if r.dep_line <= cut]
            cut_note.append((stem, before, len(rs), why))
        bursts = {}
        with open(p, 'r', errors='replace') as fh:
            for ln, line in enumerate(fh, 1):
                m = RE_BURST_DEP.match(line.strip())
                if not m:
                    continue
                vals = [int(v) for v in m.group('vals').split()]
                pre_n = int(m.group('pre'))
                pre = vals[:min(zero_n, pre_n)]
                z = st.median(pre) if pre else 0.0
                post = vals[pre_n:pre_n + window]
                bursts[ln] = abs(sum(v - z for v in post) / 1000.0 * SAMPLE_DT)
        for r in rs:
            r.dv = None
            cand = [(ln, v) for ln, v in bursts.items()
                    if ln >= r.dep_line and
                    (r.next_dep_line is None or ln < r.next_dep_line)]
            if cand:
                r.dv = min(cand)[1]              # first burst after the departure
        runs.extend(rs)

    pool = [r for r in runs if r.rel_t is not None and r.instrumented]
    if not pool:
        print(f'\n(--veto found no instrumented runs in {logdir})')
        return

    print('\n' + '=' * 78)
    print('8. THE POINT OF ALL THIS -- a ramp-VETO GATED ON dv')
    print('=' * 78)
    print(f'   raw captures : {os.path.abspath(logdir)}')
    print(f'   pool         : {len(pool)} instrumented released runs, '
          f'{sum(1 for r in pool if r.dv is not None)} carry a departure burst')
    print(f'   gate         : veto applied only when |dv| >= {thr:.2f} m/s')
    print('   no burst     : NO veto -- falls back to the shipping peak. A')
    print('                  polled-only latch emits no burst (S10/B3), and the')
    print('                  safe default there is today\'s behaviour.')
    for stem, before, after, why in cut_note:
        print(f'   ⛔ CUT       : {stem} {before} -> {after} runs; {why}')
    print()

    def verdicts(rule):
        out = {'MOVING': 0, 'OK': 0, 'NONE': 0, 'UNKNOWN': 0}
        for r in pool:
            if rule == 'dv_veto':
                use = 'ramp_veto' if (r.dv is not None and r.dv >= thr) else 'peak'
            else:
                use = rule
            out[rp.classify(r, use, hold)[0]] += 1
        return out

    print(f"   {'rule':<12} {'MOVING':>8} {'OK':>6} {'NONE':>6} {'UNKNOWN':>8}   what it means")
    print('   ' + '-' * 74)
    rows = [('peak', 'what ships today'),
            ('ramp_veto', 'safe, and refuses every inspection stop'),
            ('dv_veto', 'the candidate')]
    for rule, what in rows:
        v = verdicts(rule)
        print(f"   {rule:<12} {v['MOVING']:>8} {v['OK']:>6} {v['NONE']:>6} "
              f"{v['UNKNOWN']:>8}   {what}")
    print('\n   MOVING = released on a moving car, the catastrophic direction.')
    print('   NONE   = never released; beacon held to the 600 s failsafe.')

    print('\n   per capture, NONE count -- the cost the veto was killed for:\n')
    print(f"   {'capture':<34} {'runs':>5} {'dv>=thr':>8} "
          f"{'veto NONE':>10} {'dv_veto NONE':>13}")
    print('   ' + '-' * 74)
    for log in sorted({r.log for r in pool}):
        rs = [r for r in pool if r.log == log]
        gated = sum(1 for r in rs if r.dv is not None and r.dv >= thr)
        vn = sum(1 for r in rs if rp.classify(r, 'ramp_veto', hold)[0] == 'NONE')
        dn = sum(1 for r in rs
                 if rp.classify(r, 'ramp_veto' if (r.dv is not None and r.dv >= thr)
                                else 'peak', hold)[0] == 'NONE')
        print(f"   {os.path.basename(log):<34} {len(rs):>5} {gated:>8} "
              f"{vn:>10} {dn:>13}")


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--logs', default=os.path.join(here, '..', 'datasets'))
    ap.add_argument('--labels',
                    default=os.path.join(here, '..', 'datasets',
                                         'session_g_labels.csv'))
    ap.add_argument('--window', type=int, default=60,
                    help='post-trigger samples to integrate '
                         '(default 60 = the whole BURST_POST_DEP window, 2.4 s)')
    ap.add_argument('--zero-n', type=int, default=8,
                    help='pre-trigger samples used to fit the zero (default 6; '
                         'the later ones already carry the ramp -- see section 6)')
    ap.add_argument('--veto', metavar='LOGDIR', default=None,
                    help='also replay a dv-gated ramp-veto over the RAW '
                         'captures in LOGDIR (needs the sample lines)')
    ap.add_argument('--dv-thr', type=float, default=0.75,
                    help='dv threshold for the gate, m/s (default 0.75)')
    ap.add_argument('--hold', type=int, default=2500,
                    help="ramp_priority_replay's W, ms (default 2500)")
    ap.add_argument('--no-zero', action='store_true',
                    help='skip the pre-trigger zero fit entirely')
    a = ap.parse_args()

    labels = load_labels(a.labels)
    zn = 0 if a.no_zero else a.zero_n
    bursts = [score(b, a.window, zn)
              for b in load(glob.glob(os.path.join(a.logs, '*.log')), labels)]
    if not bursts:
        print('no departure bursts found in', a.logs)
        return

    print('=' * 78)
    print('dv FROM THE DEPARTURE BURST -- automatic vs inspection')
    print('=' * 78)
    print(f'   corpus   : {os.path.abspath(a.logs)}')
    print(f'   bursts   : {len(bursts)} k=dep   ({sum(1 for b in bursts if b.label)} carry a session-G label)')
    print(f'   window   : {a.window} post-trigger samples = {a.window*SAMPLE_DT:.2f} s')
    print(f'   zero fit : {"OFF" if not zn else f"median of the first {zn} pre-trigger samples"}')

    # -- 1. validity ---------------------------------------------------------
    print('\n' + '=' * 78)
    print('1. VALIDITY -- does the integral track the KNOWN car speed?')
    print('=' * 78)
    print('   If it does not, nothing below means anything. Nominal is the speed')
    print('   the session note records. Only `run`-labelled or unlabelled bursts')
    print('   are used here; jogs are excluded because a jog is not a run.\n')
    print(f"   {'fpm':>6} {'nominal m/s':>12} {'n':>5} {'median |dv|':>12} {'captured':>9}")
    print('   ' + '-' * 50)
    for fpm in sorted({b.fpm for b in bursts if b.fpm}):
        grp = [abs(b.dv) for b in bursts
               if b.fpm == fpm and b.label in ('', 'run')]
        if not grp:
            continue
        nom = fpm * FPM_TO_MS
        print(f"   {fpm:>6} {nom:>12.3f} {len(grp):>5} {st.median(grp):>12.3f} "
              f"{st.median(grp)/nom:>8.0%}")
    print('\n   ⚠️ The high-speed rows are TRUNCATED, not wrong. A 300 fpm car is')
    print('   still accelerating when the 2.4 s window closes, so dv is the')
    print('   velocity gained by +2.4 s -- a LOWER BOUND on line speed. That is')
    print('   monotone in speed, which is all a discriminator needs.')

    # -- 2. populations ------------------------------------------------------
    print('\n' + '=' * 78)
    print('2. THE POPULATIONS, |dv| in m/s')
    print('=' * 78)
    print('   Headline pair is `run`-labelled only -- an automatic SESSION still')
    print('   contains repositioning moves and deliberate jogs.\n')

    def sel(mode, lab=None):
        return [abs(b.dv) for b in bursts
                if b.mode == mode and (lab is None or b.label == lab)]

    ar, ir = sel('automatic', 'run'), sel('inspection', 'run')
    dist('automatic  run', ar)
    dist('inspection run', ir)
    print()
    dist('automatic  jog', sel('automatic', 'jog'))
    dist('inspection jog', sel('inspection', 'jog'))
    dist('any disturbance', [abs(b.dv) for b in bursts if b.label == 'disturbance'])
    print()
    dist('automatic  unlabelled', sel('automatic', ''))
    dist('inspection unlabelled', sel('inspection', ''))
    dist('bench (taps)', sel('bench'))
    dist('unknown capture', sel('unknown'))

    if not ar or not ir:
        print('\n   a labelled population is empty -- nothing to separate')
        return

    print('\n   --- the headline pair ---')
    print(f"   weakest automatic run  : {min(ar):.3f} m/s")
    print(f"   strongest inspection run: {max(ir):.3f} m/s")
    if min(ar) > max(ir):
        print(f"   *** SEPARATED, gap {min(ar)-max(ir):.3f} m/s "
              f"({min(ar)/max(ir):.1f}x) ***")
    else:
        ma, fa = errors(ar, ir, (min(ar) + max(ir)) / 2)
        print(f"   *** OVERLAP -- {sum(1 for v in ar if v <= max(ir))} automatic "
              f"at or below the strongest inspection ***")

    # -- 3. the error table --------------------------------------------------
    print('\n' + '=' * 78)
    print('3. THE ERROR TABLE -- every candidate threshold, both costs')
    print('=' * 78)
    print('   MISSED AUTO = automatic scored as inspection. The veto is NOT')
    print('                 applied, i.e. exactly today\'s behaviour. Benign.')
    print('   FALSE AUTO  = inspection scored as automatic. The veto IS applied,')
    print('                 the stop does not release, and the beacon is held')
    print('                 over a stopped car to the 600 s failsafe.\n')
    print('   Scored on `run` labels; the unlabelled column pools every burst in')
    print('   a capture of that mode, which is the pessimistic reading.\n')
    print(f"   {'thr m/s':>8} {'missed auto':>13} {'false auto':>13} "
          f"{'| all-burst missed':>19} {'all-burst false':>17}")
    print('   ' + '-' * 76)
    aa, ia = sel('automatic'), sel('inspection')
    for thr in [0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00]:
        ma, fa = errors(ar, ir, thr)
        mA, fA = errors(aa, ia, thr)
        flag = '   <- clean on labels' if (ma == 0 and fa == 0) else ''
        print(f"   {thr:>8.2f} {ma:>8}/{len(ar):<4} {fa:>8}/{len(ir):<4} "
              f"{mA:>12}/{len(aa):<4} {fA:>12}/{len(ia):<4}{flag}")

    # -- 4. per capture ------------------------------------------------------
    print('\n' + '=' * 78)
    print('4. PER CAPTURE -- a pooled number can hide one bad installation')
    print('=' * 78)
    print(f"   {'capture':<16} {'mode':<11} {'fpm':>5} {'n':>4} {'runs':>5} "
          f"{'min |dv|':>9} {'med':>8} {'max':>8}")
    print('   ' + '-' * 76)
    for log in sorted({b.log for b in bursts}):
        g = [b for b in bursts if b.log == log]
        v = [abs(b.dv) for b in g]
        r = [abs(b.dv) for b in g if b.label == 'run']
        fpm = g[0].fpm if g[0].fpm else ''
        print(f"   {log:<16} {g[0].mode:<11} {str(fpm):>5} {len(g):>4} {len(r):>5} "
              f"{min(v):>9.3f} {st.median(v):>8.3f} {max(v):>8.3f}")

    # -- 5. how early can the flag be set? -----------------------------------
    print('\n' + '=' * 78)
    print('5. HOW EARLY -- separation as a function of the integration window')
    print('=' * 78)
    print('   The flag is only useful if it is set before the release decision.')
    print('   On 260820-150000 the false release crossed the peak gate at +5.4 s.\n')
    print(f"   {'window':>8} {'seconds':>8} {'weakest auto':>14} "
          f"{'strongest insp':>15} {'ratio':>8}")
    print('   ' + '-' * 58)
    for w in (10, 15, 20, 25, 30, 40, 50, 60):
        A = [abs(score(b, w, zn).dv) for b in bursts
             if b.mode == 'automatic' and b.label == 'run']
        I = [abs(score(b, w, zn).dv) for b in bursts
             if b.mode == 'inspection' and b.label == 'run']
        if not A or not I:
            continue
        ratio = min(A) / max(I) if max(I) > 0 else float('inf')
        mark = '  ok' if min(A) > max(I) else '  OVERLAP'
        print(f"   {w:>8} {w*SAMPLE_DT:>8.2f} {min(A):>14.3f} {max(I):>15.3f} "
              f"{ratio:>8.2f}{mark}")
    for b in bursts:                        # restore the requested window
        score(b, a.window, zn)

    # -- 6. the zero fit, which is the load-bearing choice --------------------
    print('\n' + '=' * 78)
    print('6. THE ZERO FIT -- the most load-bearing choice in this tool')
    print('=' * 78)
    print('   How many of the 20 pre-trigger samples estimate the zero. Any-motion')
    print('   fires PART-WAY UP the ramp, so the later ones already carry signal')
    print('   and fitting across all 20 subtracts the departure from itself.\n')
    print('   The estimator is a MEDIAN, not a mean -- see score().')
    print(f"   {'zero_n':>7} {'weakest auto':>14} {'strongest insp':>15} "
          f"{'ratio':>7} {'median |zero|':>14}")
    print('   ' + '-' * 62)
    for zt in (0, 2, 4, 6, 8, 10, 12, 16, 20):
        A, I, Z = [], [], []
        for b in bursts:
            score(b, a.window, zt)
            Z.append(abs(b.zero))
            if b.label == 'run':
                if b.mode == 'automatic':
                    A.append(abs(b.dv))
                elif b.mode == 'inspection':
                    I.append(abs(b.dv))
        if not A or not I:
            continue
        mark = '  <- default' if zt == zn else ''
        print(f"   {zt:>7} {min(A):>14.3f} {max(I):>15.3f} "
              f"{min(A)/max(I):>7.2f} {st.median(Z):>14.1f}{mark}")
    for b in bursts:
        score(b, a.window, zn)

    # -- 7. the cases that decide it -----------------------------------------
    print('\n' + '=' * 78)
    print('7. THE BOUNDARY CASES -- the only ones that matter')
    print('=' * 78)
    print('   Weakest AUTOMATIC. Scoring one of these as inspection is BENIGN:')
    print('   the veto is simply not applied, which is today\'s behaviour.')
    for b in sorted([x for x in bursts if x.mode == 'automatic'],
                    key=lambda x: abs(x.dv))[:6]:
        print(f"     {b.log} run {b.run:<3} label={b.label or '-':<12} "
              f"|dv|={abs(b.dv):6.3f}  peak={b.peak:6.3f}")
    print('\n   Strongest INSPECTION. Scoring one of these as automatic applies')
    print('   the veto to a brake set, and the stop may not release at all.')
    for b in sorted([x for x in bursts if x.mode == 'inspection'],
                    key=lambda x: -abs(x.dv))[:6]:
        print(f"     {b.log} run {b.run:<3} label={b.label or '-':<12} "
              f"|dv|={abs(b.dv):6.3f}  peak={b.peak:6.3f}")

    if a.veto:
        veto_section(a.veto, a.dv_thr, a.window, zn, a.hold)

    print('\n' + '=' * 78)
    print('WHAT THIS DOES NOT SETTLE')
    print('=' * 78)
    print('   - A burst comes only from the any-motion branch (S10/B3), so runs')
    print('     latched by the polled path alone are absent. n is bursts, not runs.')
    print('   - Label coverage is partial. The unlabelled columns in §3 are the')
    print('     pessimistic reading and they are the ones to argue with.')
    print('   - Nothing here is a fix. It is the FLAG a ramp-veto would need;')
    print('     the veto itself still has to be replayed behind it, and the')
    print('     standing instruction not to rely on the beacon is unchanged.')


if __name__ == '__main__':
    main()
