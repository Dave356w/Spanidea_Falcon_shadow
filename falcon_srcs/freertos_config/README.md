# FreeRTOS configuration (ATmega328PB — 2 KB SRAM)

`FreeRTOSConfig.h` here is the **tuned** configuration for this project. The
feilipu/FreeRTOS library keeps its own copy at
`.pio/libdeps/ATmega328PB/FreeRTOS/src/FreeRTOSConfig.h`, and because the kernel
includes it with a quoted path (`#include "FreeRTOSConfig.h"`) that resolves to
the library's own directory first, a project-level override is **not** picked up
automatically.

**Important:** `.pio/` is regenerated. Any `pio pkg install`, `pio pkg update`,
or a clean checkout will restore the library's *stock* config and silently undo
these tunings. After any such operation, re-copy this file over the library's:

```
cp freertos_config/FreeRTOSConfig.h .pio/libdeps/ATmega328PB/FreeRTOS/src/FreeRTOSConfig.h
```

(For a permanent fix, vendor the FreeRTOS library under `lib/` and drop it from
`lib_deps`, or add a PlatformIO `extra_scripts` pre-build hook that performs the
copy.)

## Why these settings differ from stock

SRAM is only 2 KB and this AVR port `malloc()`s every task stack + TCB at
runtime (heap_3). The existing firmware already uses ~1.6 KB of static RAM, so
the scheduler must fit in the remainder. Changes from stock:

| Setting | Stock | Here | Reason |
|---|---|---|---|
| `configUSE_TIMERS` | 1 | 0 | No software timers used; frees the daemon task stack + TCB + command queue (~250 B heap). |
| `configMINIMAL_STACK_SIZE` | 192 | 128 | Idle task stack; idle hook is trivial. |
| `configMAX_PRIORITIES` | 4 | 2 | Only idle (0) and the app task (1) exist; fewer ready-lists = less static RAM. |
| `configMAX_TASK_NAME_LEN` | 16 | 8 | Shorter name field shrinks each TCB. |

`configCHECK_FOR_STACK_OVERFLOW` and `configUSE_MALLOC_FAILED_HOOK` are left
enabled — on the dev board a **slow** LED blink signals a stack overflow and a
**fast** blink signals a heap malloc() failure. Watch for these on first bring-up:
the runtime RAM margin is very small.
