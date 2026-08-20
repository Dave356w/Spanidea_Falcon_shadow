#!/usr/bin/env python3
"""
capture.py -- straight serial-to-file capture of the Falcon log UART.

WHY THIS EXISTS. `pio device monitor` fights programmatic capture: it owns the
port, reformats, and cannot be started or stopped from a script, so repeatable
A/B runs were impossible until this replaced it (falcon_START_HERE_2026-08-20
§7). It writes bytes straight through, line-buffered, and never exits on its
own -- the capture must outlive every run in the session, because a capture
stopped at "write it up" is how the 4-minute 18 fpm run was lost.

Usage:
    python capture.py [--port COM5] [--baud 62500] [--out DIR]
"""
import argparse, datetime, os, sys, time, serial

ap = argparse.ArgumentParser()
ap.add_argument('--port', default='COM5')       # CP210x = log UART; CH340 = ISP
ap.add_argument('--baud', type=int, default=62500)   # platformio.ini monitor_speed
ap.add_argument('--out', default=os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'logs'))
a = ap.parse_args()

stamp = datetime.datetime.now().strftime('%y%m%d-%H%M%S')
path = os.path.abspath(os.path.join(a.out, f'device-monitor-{stamp}.log'))
os.makedirs(a.out, exist_ok=True)

#
# Wait the port out rather than failing. A capture that refuses to start
# because the previous session's reader has not exited yet is a capture that
# is not running when the car moves. Retry until it opens, then hold it for
# the whole session -- there is no timeout here on purpose.
#
ser = None
while ser is None:
    try:
        ser = serial.Serial(a.port, a.baud, timeout=1)
    except serial.SerialException as exc:
        if 'Access is denied' not in str(exc) and 'PermissionError' not in str(exc):
            raise
        print(f'{a.port} busy, waiting...', flush=True)
        time.sleep(1.0)
print(f'capturing {a.port} @ {a.baud} -> {path}', flush=True)
n = 0
with open(path, 'wb', buffering=0) as fh:
    while True:
        chunk = ser.read(4096)
        if chunk:
            fh.write(chunk)
            n += chunk.count(b'\n')
