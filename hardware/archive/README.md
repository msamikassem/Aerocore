# Archive

`v1_Aerocore.kicad_pcb` is the first PCB layout attempt — kept here for reference, not used for manufacturing.

This version had the battery power plane split into disconnected copper islands and severe current-path bottlenecks (down to ~0.4mm wide in places, where 15A per motor really needs closer to 8mm on 1oz copper). See [../../docs/design-log.md](../../docs/design-log.md) for the full story of what was wrong and how I found it.

The schematic didn't change between v1 and v2 — only the PCB layout was redone, so there's only one schematic file in `hardware/kicad/`.
