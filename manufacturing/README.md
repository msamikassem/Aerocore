# Manufacturing files

These are the files actually used to order the board from JLCPCB.

- `bom.csv` — Bill of materials. Only lists real placeable parts (things with an LCSC part number).
- `cpl.csv` — component position file.
- `DRC.rpt` — Design rule check report from KiCad before ordering. 4 real "error" severity items (all on the USB-C connector's mechanical mounting holes) plus some cosmetic silkscreen warnings, all reviewed and accepted/fixed before ordering.

Gerbers aren't included here since they're a direct export of the PCB file in `hardware/kicad/`, regenerate with **File>Fabrication Outputs>Gerbers** in KiCAD.
