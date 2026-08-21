#!/usr/bin/env python3
"""
capture.py -- straight serial-to-file capture of the Falcon log UART.

WHY THIS EXISTS. `pio device monitor` fights programmatic capture: it owns the
port, reformats, and cannot be started or stopped from a script, so repeatable
A/B runs were impossible until this replaced it (falcon_START_HERE_2026-08-20
§7). It writes bytes straight through, unbuffered, and never exits on its own --
the capture must outlive every run in the session, because a capture stopped at
"write it up" is how the 4-minute 18 fpm run was lost.

WHY IT NEVER GIVES UP ON THE PORT. Two different failures have each killed a
capture mid-session, and both are silent until you go looking:

  1. The port is still held by the PREVIOUS session's reader when this one
     starts. Failing at startup means not capturing when the car moves.
  2. The CP210x RE-ENUMERATES mid-session -- pyserial raises
     `ClearCommError failed (Access is denied)` from read() and the process
     dies. This killed the 2026-08-20 15:21 capture at 15:44 with the car
     still running. falcon_START_HERE_2026-08-20 §1: "the programmer
     re-enumerates constantly."

So both the open and the read are retried forever, and everything appends to
ONE file across reconnects, because a session split into fragments by a USB
glitch is a session someone has to reassemble by hand later. A `---- capture
reconnected ----` marker is written on each recovery so the seam is visible;
nothing in graph/ parses it, since every parser there matches on line prefixes.

Usage:
    python capture.py [--port COM5] [--baud 62500] [--out DIR] [--append FILE]
"""
import argparse
import datetime
import os
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument('--port', default='COM5')            # CP210x = log UART
ap.add_argument('--baud', type=int, default=62500)   # platformio monitor_speed
ap.add_argument('--out', default=os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'logs'))
ap.add_argument('--append', default=None,
                help='continue an existing capture file instead of opening a '
                     'new one -- use this after a crash so the session stays '
                     'in one piece')
a = ap.parse_args()

os.makedirs(a.out, exist_ok=True)
if a.append:
    path = os.path.abspath(a.append)
else:
    stamp = datetime.datetime.now().strftime('%y%m%d-%H%M%S')
    path = os.path.abspath(os.path.join(a.out, f'device-monitor-{stamp}.log'))

print(f'capture -> {path}', flush=True)
fh = open(path, 'ab', buffering=0)
first = True

while True:
    ser = None
    while ser is None:
        try:
            ser = serial.Serial(a.port, a.baud, timeout=1)
        except serial.SerialException as exc:
            if 'denied' not in str(exc).lower() and \
               'could not open' not in str(exc).lower():
                raise
            print(f'{a.port} unavailable, waiting...', flush=True)
            time.sleep(1.0)

    if not first:
        stamp = datetime.datetime.now().strftime('%H:%M:%S')
        fh.write(f'---- capture reconnected {stamp} ----\r\n'.encode())
    first = False
    print(f'capturing {a.port} @ {a.baud}', flush=True)

    try:
        while True:
            chunk = ser.read(4096)
            if chunk:
                fh.write(chunk)
    except (serial.SerialException, OSError) as exc:
        print(f'read failed ({exc}); reopening', flush=True)
        try:
            ser.close()
        except Exception:
            pass
        time.sleep(1.0)
