# Falcon EFT — Reflash and Test Session Report

**Date:** 2026-07-15 **Firmware:** V1.2, branch `Falcon_Rel_EFT` (commit `b795dd2`) **Tester scope:** firmware reflash, serial logging setup, instrumented hoistway runs

## Summary

Firmware was rebuilt from the `Falcon_Rel_EFT` branch and flashed successfully; upload verified byte-for-byte against the device. Serial logging was established after correcting a documentation error (actual baud is 9600, not the documented 115200). Eight instrumented runs were completed at four speeds in both directions. Movement detection and alarm functioned on departure in 7 of 8 runs; observed behaviors of note: the alarm consistently stopped after roughly 10 seconds while the car was still traveling at constant velocity, re-alarming on brake set at arrival, and the 18 fpm down run produced no alarm at departure at all.

## Flashing procedure (as performed)

Project source: branch ZIP extracted; the PlatformIO project is the `falcon_srcs/` folder — open that folder in VS Code, not the repo root. PlatformIO auto-configured the toolchain and libraries on first open (requires internet; a transient IntelliSense compilerPath error appeared during download and self-resolved).

Before flashing, `upload_port` in `platformio.ini` was updated from the stale `COM7` to the actual programmer port. Procedure: Clean → Build → Upload from the ATmega328PB project tasks. Build result: flash 20,478 / 32,256 bytes (63.5%), RAM 1,631 / 2,048 bytes (79.6%). Upload over stk500; avrdude reported device signature `0x1e9516 (probably m328pb)` and verified all 20,478 bytes.

Documentation corrections encountered during setup, for whoever maintains the Readme: the Readme and `commands.sh` reference ATmega328P / `-p m328p`, but the device reports m328pb and the project builds for ATmega328PB — manual avrdude commands need `-p m328pb`. The `upload_port` value in the ini must be updated to the current port each session (check Device Manager; ports renumber on reconnection).

## Serial logging setup (as performed)

Hardware this session: COM4 \= CH340 (programmer, TAG connector); COM5 \= CP2102 (log cable, TX/RX crossed, common ground).

PuTTY settings that worked: Serial, COM5, **9600 baud**, 8 data bits, 1 stop bit, no parity, flow control **None**, Session → Logging → All session output. The Readme documents 115200 baud — at that setting the terminal showed nothing (blank window, cable activity LED flashing); at 9600 output was clean. Readme needs correcting.

Note for anyone reading captures: PuTTY connects silently — a blank window with a cursor means the port is open and waiting. Reset or power-cycle the device with the window open to capture boot output.

## Test session — hoistway runs

Eight runs: up and down at 18, 58, 107, and \~348/350 fpm. One PuTTY log per run. Logs contain no timestamps, so event timing within a run relies on operator observation; noting the wall-clock time of each motion command (or adding timestamps to the log output) would improve the next session.

| Run | Alarm at departure | Observed behavior |
| :---- | :---- | :---- |
| up 18 | Yes | Alarm stopped \~10 s into travel; re-alarmed at brake set |
| up 58 | Yes | Same pattern |
| up 107 | Yes | Same pattern |
| up 348 | Yes | Trip under 10 s (limited rise); single alarm cycle covered the run |
| dwn 18 | **No** | No alarm at departure; alarmed only at brake set on arrival |
| dwn 58 | Yes | Alarm stopped \~10 s into travel; re-alarmed at brake set |
| dwn 107 | Yes | Same pattern |
| dwn 350 | Yes | Trip under 10 s (limited rise) |

## Observations

1. **Alarm cutoff during travel.** At 18, 58, and 107 fpm in both directions, the alarm stopped after roughly 10 seconds while the car was still moving at constant velocity, then re-alarmed when the brake set at arrival. The device alarms on the start and stop transients, not for the duration of travel.  
2. **No detection at 18 fpm down.** The slowest downward run produced no alarm at departure — the car traveled the full run unalarmed until brake set. The corresponding upward run at the same speed did alarm at departure.  
3. **High-speed steady state not covered.** The 348/350 fpm trips were shorter than 10 seconds due to the available rise, so those runs do not show whether the alarm would persist or cut off during sustained travel at rated speed. Testing that requires a longer hoistway or an express run.  
4. **Corrupted characters in logs.** Dropped characters appear in 7 of 8 log files (e.g. "STA E\_MONITORING", "ST TE\_"), consistently within the burst of status lines printed around the point the buzzer switches off. Recurs across runs; noted for the hardware/firmware team.  
5. **Battery voltage readings** appeared periodically in all logs (2224–2242 mV range this session), with no low-battery alarms triggered.

## Log files

Eight PuTTY captures retained, named `falcon_log_{up|dwn}_{speed}_fpm`. One file (`up_348`) contains two stray non-printable bytes; content is otherwise intact and readable.  
