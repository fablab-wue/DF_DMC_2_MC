# DF_DMC_2_MC

**Dragonframe DMC v2 USB bridge for DIY motorized camera sliders** — C++ / PlatformIO on a **Waveshare RP2040-Zero**, talking [SliderMC](https://github.com/fablab-wue/SliderMC) ASCII over UART.

## About

**DF_DMC_2_MC** lets [Dragonframe](https://www.dragonframe.com/) Arc drive the same motion board as the physical panel ([SliderCtrl](https://github.com/fablab-wue/SliderCtrl)). The PC speaks binary **DMC v2** on USB CDC; this Zero translates millimetre moves, path play, GIO, camera, and live DMX onto SliderMC’s UART contract (`VH`, `CG`, `MT`, `PG`, …).

Use it **instead of** a UIC on that UART, not in parallel. Rail, motor, and housing stay yours.

Docs and manuals: **[SliderDoc](https://github.com/fablab-wue/SliderDoc)**.

> Documentation: [SliderDoc](https://github.com/fablab-wue/SliderDoc)

```text
Dragonframe (PC)
        USB CDC  — binary DMC v2  (device type dmc-lite)
DF_DMC_2_MC  (RP2040-Zero)
        UART 115200  GP12 TX / GP13 RX
SliderMC
```

---

## Features

- **Connect as dmc-lite** — hello name `SliderCtrl MC V1 (dmc-lite)`; Arc **steps per unit = 1000**
- **Units** — 1000 DMC steps = 1 mm or 1 deg (same for speed/accel)
- **Live GIO** — 4 out (GP1–4) / 4 in (GP5–8) on this Zero, not MC extenders
- **DMX512** — 512 channels on GP0 (PIO UART + MAX485 for a real universe). Channels 1–6 also drive DMX1–DMX6 PWM at 18 kHz on GP29, GP28, GP27, GP26, GP15, GP14 (high-active, duty = level/255)
- **MOVE** — GP11 high while the verbose status letter is `M`, `A`, `B`, `H`, or `P`
- **Camera / bloop** — GP9 shutter + MC `CT`; GP10 buzzer + MC `BE`
- **Path upload** — DF frames → SliderMC `PD` / `PG` (0-based inclusive; reverse if start > end)
- **Simulator** — `-DSIMULATE` (default): 1-axis stand-in if no MC answers

---

## Quick start (VS Code)

1. Install [VS Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.
2. **File → Open Folder** → this repository (`DF_DMC_2_MC`).
3. PlatformIO: **Build** / **Upload**.
4. Dragonframe: Scene → Connections → device type **dmc-lite** → this COM port → Connect.
5. In Arc, set **steps per unit = 1000**.

Shared DMC framing, the path upload table, DMX, and GIO come from the sibling checkout [DF_DMC_Common](https://github.com/fablab-wue/DF_DMC_Common) (`../DF_DMC_Common`). Clone it next to this repo before building.

USB CDC is binary DMC — do not use the PlatformIO serial monitor as a console. Details: [dmc/build.md](https://github.com/fablab-wue/SliderDoc/blob/main/dmc/build.md).

PC smoke test (Dragonframe disconnected): `python pc_dmc_test.py COM21 --sequence hi` — see [dmc/build.md](https://github.com/fablab-wue/SliderDoc/blob/main/dmc/build.md).

---

## Documentation (SliderDoc)

| Topic | Document |
|-------|----------|
| Overview / Connect / units | [dmc/overview.md](https://github.com/fablab-wue/SliderDoc/blob/main/dmc/overview.md) |
| Build / flash / PC test | [dmc/build.md](https://github.com/fablab-wue/SliderDoc/blob/main/dmc/build.md) |
| GPIO / UART / wiring ASCII | [dmc/pins.md](https://github.com/fablab-wue/SliderDoc/blob/main/dmc/pins.md) |
| DMC opcode → MC map | [dmc/mapping.md](https://github.com/fablab-wue/SliderDoc/blob/main/dmc/mapping.md) |
| SliderMC UART | [contract/protocol.md](https://github.com/fablab-wue/SliderDoc/blob/main/contract/protocol.md) |
| Official DMC v2 (Dragonframe) | [DMC-Protocol-2024-08-13.pdf](https://www.dragonframe.com/download/dmcproto/DMC-Protocol-2024-08-13.pdf) |

dmc-lite sketches ship **with Dragonframe**, not this repo: [where to find dmc-lite](https://www.dragonframe.com/ufaqs/where-do-i-find-the-dmc-lite-arduino-sketch/).

---

## Repository layout

```text
src/config.h         Pins, limits, 1000 steps = 1 mm/deg
src/path_store       DF upload → PD samples (positions in DF_DMC_Common)
src/mc_client        SliderMC UART + simulator
src/status_led       WS2812 / classic LED
src/bridge           USB DMC dispatch
src/main.cpp         setup() / loop()
pc_dmc_test.py       Host DMC smoke test
```

---

## License

Copyright (c) 2026 Jochen Krapf \<jk@nerd2nerd.org\>

Licensed under the [MIT License](LICENSE).

Company names and product names mentioned in this project are trademarks or registered trademarks of their respective owners. Use here is for identification only.
