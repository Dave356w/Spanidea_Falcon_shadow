#!/usr/bin/python3
"""
session_g.py -- corpus census and labelling worksheet (test plan session G).

Session G exists because four candidate discrimination rules could not be
scored: nothing in the corpus records whether a departure was a real run, a
jog, or a disturbance. This tool does the mechanical half -- find every
departure, attach the context a human needs to judge it, and report what the
corpus cannot answer at all.

It does NOT label. Labelling is the human half and the whole point.

Usage:
    ./session_g.py                          # census only, default corpus
    ./session_g.py --worksheet out.csv      # census + blank worksheet
    ./session_g.py path/to/*.log            # explicit files

⚠️  span_ms IS NOT A RUN DURATION -- do not label from it. The corpus carries
    no periodic sample line (`t= a= avg= ...`), so timestamps come only from
    edge-triggered ACC-INT lines. A departure raises a cluster of edges and
    cruise is quiet by construction, so in most runs BOTH edges sit at the
    departure: 2-edge runs have a median span of 1146 ms, which is not a 1.1 s
    ride. The census prints the full distribution.

⚠️  Departure bursts fire on the ANY-MOTION path only (movement_service.cpp,
    the burst_trigger call under `if (any_motion_pending)`). A departure the
    polled or velocity path caught produces no burst. `dep_path` reports this
    per row and the census totals it -- it is the corpus's structural blind
    spot, not a logging gap.
"""

import csv
import glob
import os
import re
import sys

# ---------------------------------------------------------------- patterns

RE_ACC_INT   = re.compile(r'^ACC-INT\s+n=(\d+)\s+t=(\d+)\s+st=(\d+)\s+bz=(\d+)')
RE_BURST_DEP = re.compile(r'^BURST\s+k=dep\b')
RE_BURST_OLD = re.compile(r'^BURST\s+pre=')          # pre-k= format, 2026-08-11
RE_JOGV      = re.compile(
    r'^JOGV\s+pos=(-?\d+)\s+neg=(-?\d+)\s+ratio=(-?\d+)\s+opk=(-?\d+)\s+verdict=(\w+)')
RE_ARRIVAL   = re.compile(r'^FSM:\s+Arrival\s+\((\S+?)\)')
RE_PEAK      = re.compile(r'peak\s+([0-9.]+)')
RE_RESET     = re.compile(r'^Reset cause')

LATCH_ANYMOTION = 'FSM: Departure latched'
TO_MOVING       = 'Transitioned to STATE_MOVING'
TO_DECEL        = 'Transitioned to STATE_DECELERATING'
FAILSAFE        = 'FSM: FAILSAFE'
JOG_RELEASE     = 'FSM: Release (jog verdict)'

FIELDS = [
    'log', 'boot', 'run', 'dep_path', 'has_burst',
    'edges', 't_first_ms', 't_last_ms', 'span_ms',
    'jogv_verdict', 'jogv_opk', 'jogv_ratio', 'jogv_pos', 'jogv_neg',
    'end_path', 'arr_peak', 'buzzer_edges',
    'label', 'basis',
]


class Run(object):
    def __init__(self, log, boot, idx):
        self.log      = log
        self.boot     = boot
        self.idx      = idx
        self.latched   = False
        self.pre_latch = False   # capture predates the latched FSM entirely
        self.burst    = False
        self.jogv     = None
        self.edges    = 0
        self.bz_edges = 0
        self.t_first  = None
        self.t_last   = None
        self.end      = ''
        self.peak     = ''

    def note_edge(self, t_ms, bz):
        self.edges += 1
        if bz:
            self.bz_edges += 1
        if self.t_first is None:
            self.t_first = t_ms
        self.t_last = t_ms

    def row(self):
        span = ''
        if self.t_first is not None and self.t_last is not None:
            if self.t_last >= self.t_first:
                span = self.t_last - self.t_first
        j = self.jogv or {}
        return {
            'log':          self.log,
            'boot':         self.boot,
            'run':          self.idx,
            'dep_path':     ('any-motion' if self.latched
                             else ('pre-latch-build' if self.pre_latch
                                   else 'polled/silent')),
            'has_burst':    'y' if self.burst else 'n',
            'edges':        self.edges,
            't_first_ms':   '' if self.t_first is None else self.t_first,
            't_last_ms':    '' if self.t_last is None else self.t_last,
            'span_ms':      span,
            'jogv_verdict': j.get('verdict', ''),
            'jogv_opk':     j.get('opk', ''),
            'jogv_ratio':   j.get('ratio', ''),
            'jogv_pos':     j.get('pos', ''),
            'jogv_neg':     j.get('neg', ''),
            'end_path':     self.end,
            'arr_peak':     self.peak,
            'buzzer_edges': self.bz_edges,
            'label':        '',      # run | jog | disturbance | unknown
            'basis':        '',      # WHY -- see the procedure note
        }


def parse_log(path):
    """Return (runs, stats) for one capture."""
    name  = os.path.basename(path)
    runs  = []
    stats = {'moving': 0, 'latched': 0, 'orphan_burst': 0, 'boots': 1}

    boot        = 1
    idx         = 0
    cur         = None
    last_t      = None
    latch_armed = False          # latch line seen, MOVING not yet reached

    with open(path, 'r', errors='replace') as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue

            if RE_RESET.match(line):
                boot += 1
                stats['boots'] = boot
                last_t = None
                continue

            m = RE_ACC_INT.match(line)
            if m:
                t_ms = int(m.group(2))
                bz   = int(m.group(4))
                # t= restarts on reboot; a backwards jump is a new boot.
                if last_t is not None and t_ms + 1000 < last_t:
                    boot += 1
                    stats['boots'] = boot
                last_t = t_ms
                if cur is not None:
                    cur.note_edge(t_ms, bz)
                continue

            if line.startswith(LATCH_ANYMOTION):
                latch_armed = True
                stats['latched'] += 1
                continue

            if TO_MOVING in line:
                stats['moving'] += 1
                idx += 1
                cur = Run(name, boot, idx)
                cur.latched = latch_armed
                latch_armed = False
                runs.append(cur)
                continue

            if RE_BURST_DEP.match(line) or RE_BURST_OLD.match(line):
                if cur is not None:
                    cur.burst = True
                else:
                    stats['orphan_burst'] += 1
                continue

            m = RE_JOGV.match(line)
            if m and cur is not None:
                cur.jogv = {
                    'pos':     m.group(1), 'neg':   m.group(2),
                    'ratio':   m.group(3), 'opk':   m.group(4),
                    'verdict': m.group(5),
                }
                continue

            m = RE_ARRIVAL.match(line)
            if m and cur is not None:
                cur.end = 'arrival-' + m.group(1)
                p = RE_PEAK.search(line)
                if p:
                    cur.peak = p.group(1)
                continue

            if line.startswith(FAILSAFE) and cur is not None:
                cur.end = 'failsafe'
                continue

            if line.startswith(JOG_RELEASE) and cur is not None:
                cur.end = 'jog-verdict-release'
                continue

            if TO_DECEL in line and cur is not None:
                if not cur.end:
                    cur.end = 'decel-no-cause-logged'
                cur = None
                continue

    for r in runs:
        if not r.end:
            r.end = 'log-ended-mid-run'

    # A capture with NO latch line at all predates the latched FSM (landed
    # 2026-08-07); its departures came through the old polled path and are not
    # B3 silent departures. Conflating the two over-attributes to B3.
    if stats['latched'] == 0:
        for r in runs:
            r.pre_latch = True

    return runs, stats


def census(all_runs, totals, files):
    def pct(n, d):
        return '  --' if not d else '%4.1f%%' % (100.0 * n / d)

    n        = len(all_runs)
    latched   = sum(1 for r in all_runs if r.latched)
    pre_latch = sum(1 for r in all_runs if r.pre_latch)
    silent    = n - latched - pre_latch
    bursts   = sum(1 for r in all_runs if r.burst)
    no_burst = n - bursts

    print('CORPUS CENSUS -- session G')
    print('  captures scanned      %d' % len(files))
    print('  departures (MOVING)   %d' % n)
    print('')
    print('DEPARTURE PATH -- what triggered the latch')
    print('  any-motion            %4d   %s' % (latched, pct(latched, n)))
    print('  polled / SILENT       %4d   %s   <- true B3: capture has latch'
          % (silent, pct(silent, n)))
    print('                                       lines, this departure had none')
    print('  pre-latch build       %4d   %s   <- capture has NO latch line at'
          % (pre_latch, pct(pre_latch, n)))
    print('                                       all; predates the latched FSM')
    print('')
    print('BURST COVERAGE -- what can carry a shape rule at all')
    print('  departure burst       %4d   %s' % (bursts, pct(bursts, n)))
    print('  NO burst              %4d   %s   <- unscoreable for any shape rule'
          % (no_burst, pct(no_burst, n)))
    print('')

    print('BURST COVERAGE BY DATE')
    by_date = {}
    for r in all_runs:
        d = r.log[:6]
        a, b = by_date.get(d, (0, 0))
        by_date[d] = (a + 1, b + (1 if r.burst else 0))
    for d in sorted(by_date):
        tot, bst = by_date[d]
        print('  %s   %3d departures   %3d bursts   %s'
              % (d, tot, bst, pct(bst, tot)))
    print('')

    print('HOW RUNS ENDED')
    ends = {}
    for r in all_runs:
        ends[r.end] = ends.get(r.end, 0) + 1
    for k in sorted(ends, key=lambda x: -ends[x]):
        print('  %-24s %4d   %s' % (k, ends[k], pct(ends[k], n)))
    print('')

    print('⚠️  span_ms IS NOT A RUN DURATION. Measured over this corpus:')
    print('      1 edge   n=44   span always 0')
    print('      2 edges  n=95   median span 1146 ms')
    print('      3-5      n=66   median span 4915 ms')
    print('    ACC-INT is edge-triggered and a departure raises a cluster of')
    print('    edges; cruise is quiet by construction (a car at constant')
    print('    velocity reads 1 g). So in most runs BOTH edges sit at the')
    print('    departure and the span measures that cluster, not the trip.')
    print('    Do not label from it.')
    print('')
    buckets = [('0 edges', 0, 0), ('1-2 edges', 1, 2),
               ('3-9 edges', 3, 9), ('10+ edges', 10, 10 ** 9)]
    for lbl, lo, hi in buckets:
        c = sum(1 for r in all_runs if lo <= r.edges <= hi)
        print('  %-12s %4d   %s' % (lbl, c, pct(c, n)))
    print('')

    print('FIRMWARE JOG VERDICT -- population needing independent labels')
    print('  ⚠️  DO NOT label from this field. It is opk-thresholded by')
    print('      definition, so any rule scored against it separates perfectly')
    print('      and means nothing.')
    verds = {}
    for r in all_runs:
        v = (r.jogv or {}).get('verdict', '(none)')
        verds[v] = verds.get(v, 0) + 1
    for k in sorted(verds, key=lambda x: -verds[x]):
        print('  %-12s %4d   %s' % (k, verds[k], pct(verds[k], n)))
    print('')

    print('CEILING ON THIS SESSION')
    print('  shape-scoreable (has a departure burst)   %d of %d  (%s)'
          % (bursts, n, pct(bursts, n)))
    print('  ...carrying independent evidence of WHAT the event was, from')
    print('     the log alone:                         0')
    print('')
    print('  Nothing in a capture says run vs jog. end_path cannot: a jog')
    print('  produces an arrival transient too, and jog-verdict-release IS')
    print('  the opk rule, so scoring against it is circular. span_ms cannot,')
    print('  per the note above. That leaves the dated session notes as the')
    print('  only independent source, and they are per-SESSION narrative,')
    print('  not per-RECORD.')
    print('')
    print('  So the binding constraint is notes coverage, not burst coverage.')
    print('  Work the per-capture table below against Eng_Notes and mark only')
    print('  what the notes actually pin down. If that count comes out low,')
    print("  the session's conclusion is the one its exit criterion already")
    print('  anticipated: future rules need labels collected AT THE CAR, not')
    print('  recovered afterwards.')
    print('')

    print('PER-CAPTURE WORKLIST -- take these to the dated session notes')
    per = {}
    for r in all_runs:
        a, b = per.get(r.log, (0, 0))
        per[r.log] = (a + 1, b + (1 if r.burst else 0))
    print('  %-24s %5s %7s' % ('capture', 'deps', 'bursts'))
    for k in sorted(per):
        tot, bst = per[k]
        if bst:
            print('  %-24s %5d %7d' % (k, tot, bst))


def load_labels(path):
    """
    Read a labels file: log,run,label,basis. Keyed on (log, run) because those
    are the two columns the worksheet exposes and a human can transcribe. The
    file is the durable artefact -- the worksheet is regenerated from it, so
    re-running the census never loses a label.
    """
    labels = {}
    with open(path, newline='', encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            log = (row.get('log') or '').strip()
            run = (row.get('run') or '').strip()
            if not log or not run:
                continue
            labels[(log, run)] = ((row.get('label') or '').strip(),
                                  (row.get('basis') or '').strip())
    return labels


def label_census(all_runs, labels):
    """Report the exit criterion: counts per label, with named evidence."""
    order = ['run', 'jog', 'disturbance', 'unknown']
    counts = dict((k, 0) for k in order)
    other  = {}
    no_basis = []
    labelled_keys = set()

    for r in all_runs:
        key = (r.log, str(r.idx))
        lab, basis = labels.get(key, ('', ''))
        if not lab:
            continue
        labelled_keys.add(key)
        if lab in counts:
            counts[lab] += 1
        else:
            other[lab] = other.get(lab, 0) + 1
        if not basis:
            no_basis.append('%s r%s' % (r.log, r.idx))

    total = len(all_runs)
    named = counts['run'] + counts['jog'] + counts['disturbance']

    print('')
    print('LABEL CENSUS -- the session exit criterion')
    for k in order:
        print('  %-14s %5d   %5.1f%%' % (k, counts[k], 100.0 * counts[k] / total))
    for k, v in sorted(other.items()):
        print('  %-14s %5d   (not one of the four valid labels)' % (k, v))
    print('  %-14s %5d   %5.1f%%   <- no label row at all' %
          ('unlabelled', total - len(labelled_keys),
           100.0 * (total - len(labelled_keys)) / total))
    print('  %-14s %5d   %5.1f%%   <- run/jog/disturbance with named evidence' %
          ('SCOREABLE', named, 100.0 * named / total))

    if no_basis:
        print('')
        print('  ⚠️  labelled with NO basis -- not admissible, fix or drop:')
        for n in no_basis:
            print('       %s' % n)

    stray = sorted(set(labels) - labelled_keys)
    if stray:
        print('')
        print('  ⚠️  label rows matching no record (typo in log or run?):')
        for lg, rn in stray:
            print('       %s r%s' % (lg, rn))


def main():
    # The census prints non-ASCII warning marks. On Windows the console
    # defaults to cp1252 and the first one aborts the run, so force UTF-8.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding='utf-8')
        except (AttributeError, ValueError):
            pass

    args  = [a for a in sys.argv[1:] if not a.startswith('--')]
    sheet = None
    labels_path = None
    for flag in ('--worksheet', '--labels'):
        if flag in sys.argv:
            i = sys.argv.index(flag)
            if i + 1 < len(sys.argv):
                val = sys.argv[i + 1]
                if val in args:
                    args.remove(val)
                if flag == '--worksheet':
                    sheet = val
                else:
                    labels_path = val

    if args:
        files = []
        for a in args:
            files.extend(sorted(glob.glob(a)))
    else:
        here  = os.path.dirname(os.path.abspath(__file__))
        files = sorted(glob.glob(os.path.join(here, '..', 'datasets', '*.log')))

    if not files:
        print('no logs found', file=sys.stderr)
        return 1

    all_runs, totals = [], {}
    for f in files:
        runs, stats = parse_log(f)
        all_runs.extend(runs)
        for k, v in stats.items():
            totals[k] = totals.get(k, 0) + v

    census(all_runs, totals, files)

    labels = load_labels(labels_path) if labels_path else {}
    if labels:
        label_census(all_runs, labels)

    if sheet:
        with open(sheet, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            for r in all_runs:
                row = r.row()
                lab, basis = labels.get((r.log, str(r.idx)), ('', ''))
                row['label'] = lab
                row['basis'] = basis
                w.writerow(row)
        print('')
        print('worksheet: %s  (%d rows, %d labelled)' %
              (sheet, len(all_runs), sum(1 for r in all_runs
                                         if (r.log, str(r.idx)) in labels)))

    return 0


if __name__ == '__main__':
    sys.exit(main())
