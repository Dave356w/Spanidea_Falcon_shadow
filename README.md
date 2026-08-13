# Falcon — elevator counterweight movement beacon

This branch is compiled for the ATmega328PB part in version 2 of the hardware
designed by Voguvant.

A battery-powered beacon placed on an elevator counterweight during maintenance.
While the counterweight moves it sounds a piezo and runs a chase-LED sweep so a
mechanic in the hoistway can locate it. When it stops, the beacon must stop.

## Start here

| document | what it is |
|---|---|
| **[`Falcon_Product_Document.md`](Falcon_Product_Document.md)** | **Start here.** Product definition, implemented state logic, development practice, test procedures. Written to be read cold. |
| [`Eng_Notes/falcon_state_of_project_2026-08-13.md`](Eng_Notes/falcon_state_of_project_2026-08-13.md) | Current status: what is proven, what is not, what is open |
| [`Eng_Notes/falcon_reference_2026-08-12.md`](Eng_Notes/falcon_reference_2026-08-12.md) | Orientation doc — full legends for every function, log field, variable and live threshold. Kept current |
| `Eng_Notes/` | Dated engineering session notes. Chronology is listed in the reference doc |

## Build

```
cd falcon_srcs
pio run                        # build
pio run -t upload              # flash (programmer port)
pio device monitor -b 62500    # capture (logging port)
```

⚠️ Two things that will catch you out:

- **`falcon_srcs/lib/Wire` and `lib/Wire1` are vendored and deliberately
  modified.** Do not replace them with the framework copies — see
  Product Document §5.3.
- **The safety axiom:** a detector may fail toward alarming, never toward
  silence. Every threshold carries its reasoning in a comment at its `#define`.
  Read the comment before changing the number.
