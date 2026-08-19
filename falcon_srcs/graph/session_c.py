"""Session C -- re-arm regression check, and sizing MONITOR_REARM_QUIET.

Two questions, one capture.

1. THE REGRESSION. Does an alarm begin on a car that has already stopped? The
   signature is a departure latch shortly after STATE_MONITORING is entered,
   which does NOT go on to become a real run. The 2026-08-07 instance re-latched
   3.5 s after MONITORING and held for 73 s on a stationary car.

   A short interval alone is NOT the regression. Measured 2026-08-19, a genuine
   jog departed 2801 ms after MONITORING and ran a full cycle to a polled
   arrival. What separates them is whether the latch leads to a real run, so
   this reports the outcome of every re-latch rather than just its timing.

2. SIZING MONITOR_REARM_QUIET. The re-arm blank clears when
   `elapsed >= MONITOR_REARM_MIN_MS && quiet_run() >= MONITOR_REARM_QUIET`.
   The floor cannot solve this on its own: real departures were seen at 2.8 s
   and the false re-latch at 3.5 s, so the populations overlap and no floor
   value separates them. The quiet requirement CAN separate them, because a
   ringing terminal-floor brake set holds quiet_run() down while an ordinary
   settled stop does not.

   So the number wanted is: how long does the ringing keep quiet_run()
   suppressed on a TERMINAL stop, versus an ordinary one? This prints time-to-
   reach for a range of quiet thresholds, per stop. Pick the threshold that
   ordinary stops clear quickly and terminal stops do not.

Reference: MONITOR_REARM_MS 6000 (cap), MONITOR_REARM_MIN_MS 2500 (floor, was
1500 until 2026-08-19), MONITOR_REARM_QUIET 8. STOP_CONFIRM_MS 5000 is the
PRIMARY defence -- the blank is explicitly the backstop, not the fix.
"""
import io
import re
import sys

QUIET_PROBES = (8, 16, 32, 64, 128)
REARM_MIN_MS = 2500
REARM_CAP_MS = 6000


def parse(path):
    """Return per-MONITORING-episode records.

    Bounded by sample INDEX, never by t= alone: t restarts at zero on every
    boot and a capture with two boots overlaps in t.
    """
    samples = []          # (idx, t, st, q)
    events = []           # (idx, kind, detail)
    for ln in io.open(path, encoding="utf-8", errors="replace"):
        ln = ln.strip()
        if ln.startswith("t=") and " ov=" in ln:
            f = {}
            for p in ln.split():
                if "=" in p:
                    k, v = p.split("=", 1)
                    f[k] = v
            try:
                samples.append((len(samples), int(f["t"]), int(f.get("st", -1)),
                                int(f.get("q", -1))))
            except ValueError:
                pass
            continue
        idx = len(samples)
        if "Transitioned to STATE_MONITORING" in ln:
            events.append((idx, "monitor", None))
        elif "STATE_MOVEMENT_DETECTED" in ln:
            events.append((idx, "depart", None))
        elif "Arrival (" in ln:
            events.append((idx, "arrival", ln))
        elif "re-arm blank cleared early" in ln:
            m = re.search(r"settled at (\d+) ms", ln)
            events.append((idx, "settled", int(m.group(1)) if m else None))
        elif "Device Booted" in ln:
            events.append((idx, "boot", None))
        elif "FAILSAFE" in ln:
            events.append((idx, "failsafe", None))
        elif "Release" in ln or "Released" in ln:
            events.append((idx, "release", ln))
    return samples, events


def main():
    path = sys.argv[1]
    samples, events = parse(path)
    if not samples:
        print("  no sample lines")
        return
    t_of = {i: t for i, t, st, q in samples}

    print("  Session C -- re-arm regression + quiet sizing      %s" % path)
    print("  floor %d ms   cap %d ms\n" % (REARM_MIN_MS, REARM_CAP_MS))

    stops = []
    for n, (idx, kind, detail) in enumerate(events):
        if kind != "monitor":
            continue
        settled = None
        nxt = None
        outcome = "no further departure"
        for idx2, kind2, detail2 in events[n + 1:]:
            if kind2 == "settled" and settled is None:
                settled = detail2
            elif kind2 == "depart":
                nxt = idx2
                # what happened after this departure?
                for idx3, kind3, detail3 in events[n + 1:]:
                    if idx3 <= idx2:
                        continue
                    if kind3 == "arrival":
                        outcome = "REAL RUN (reached arrival)"
                        break
                    if kind3 in ("failsafe",):
                        outcome = "** FAILSAFE -- 600 s alarm **"
                        break
                    if kind3 == "release":
                        outcome = "released without an arrival: %s" % detail3[:44]
                        break
                    if kind3 in ("monitor", "boot"):
                        outcome = "** NO ARRIVAL -- returned to MONITORING **"
                        break
                break
            elif kind2 in ("monitor", "boot"):
                break
        stops.append((idx, settled, nxt, outcome))

    print("  stop  settled_at   next departure   outcome")
    print("  ----  ----------   --------------   -------")
    suspicious = 0
    for i, (idx, settled, nxt, outcome) in enumerate(stops, 1):
        gap = ""
        if nxt is not None and idx in t_of and nxt in t_of:
            d = t_of[nxt] - t_of[idx]
            gap = "%6d ms" % d
            if d < REARM_CAP_MS and "REAL RUN" not in outcome:
                suspicious += 1
                gap += " ⚠"
        print("  %4d  %-10s   %-14s   %s"
              % (i, ("%d ms" % settled) if settled else "not cleared",
                 gap or "none", outcome))

    print("\n  ---- quiet_run() profile after each stop ----")
    print("  what MONITOR_REARM_QUIET would cost: ms from MONITORING until")
    print("  quiet_run() first reaches each threshold.\n")
    hdr = "  stop  " + "".join("q>=%-3d   " % p for p in QUIET_PROBES)
    print(hdr)
    print("  ----  " + "".join("-------  " for _ in QUIET_PROBES))
    for i, (idx, settled, nxt, outcome) in enumerate(stops, 1):
        end = nxt if nxt is not None else len(samples)
        seg = [(t, q) for j, t, st, q in samples if idx <= j < end and q >= 0]
        if not seg:
            continue
        t0 = seg[0][0]
        row = "  %4d  " % i
        for p in QUIET_PROBES:
            hit = next((t - t0 for t, q in seg if q >= p), None)
            row += ("%5d    " % hit) if hit is not None else "  --     "
        print(row)

    print("\n  ---- verdict ----")
    if suspicious:
        print("  ** %d re-latch(es) inside the %d ms cap that did NOT become a real"
              " run. **" % (suspicious, REARM_CAP_MS))
        print("  That is the regression. Record the interval; it is the number that")
        print("  decides the fix. Do NOT set MONITOR_REARM_QUIET to 0 -- that")
        print("  SHORTENS the blank to the floor (corrected 2026-08-19).")
    else:
        print("  No re-latch inside the cap failed to become a real run.")
        print("  Exit criterion wants ten terminal stops; this capture has %d."
              % len(stops))


if __name__ == "__main__":
    main()
