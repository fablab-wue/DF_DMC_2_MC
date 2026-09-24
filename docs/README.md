# DF_DMC_2_MC firmware docs

DragonFrame **DMC v2** USB bridge to **SliderMC** ASCII UART. Shipping board: **Waveshare RP2040-Zero**.

**Start here:** [overview.md](overview.md)

| Document | Topic |
|----------|-------|
| [overview.md](overview.md) | What it is, Connect, units, simulator, LED |
| [build.md](build.md) | PlatformIO, flags, `pc_dmc_test.py` |
| [pins.md](pins.md) | GPIO map, UART to MC, wiring ASCII |
| [mapping.md](mapping.md) | DMC opcodes → MC / local GPIO |

**Code repo:** [DF_DMC_2_MC](https://github.com/fablab-wue/DF_DMC_2_MC)

**Official DMC protocol (Dragonframe, not this project):** [DMC-Protocol-2024-08-13.pdf](https://www.dragonframe.com/download/dmcproto/DMC-Protocol-2024-08-13.pdf)

**Library:** [DF_DMC_Common](https://github.com/fablab-wue/DF_DMC_Common). Connect steps shared with other boards: [dragonframe.md](https://github.com/fablab-wue/DF_DMC_Common/blob/main/docs/dragonframe.md).

**Slider stack:** [SliderMC](https://github.com/fablab-wue/SliderDoc/blob/main/mc/README.md) · [UART contract](https://github.com/fablab-wue/SliderDoc/blob/main/contract/protocol.md) · [architecture](https://github.com/fablab-wue/SliderDoc/blob/main/architecture/overview.md)
