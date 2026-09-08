# Design Log

This is a rough chronological writeup of how this board actually got built, including the stuff that went wrong. I think that's the more useful part to write down, honestly, the schematic and PCB files show what I ended up with, but not how many times I almost shipped something broken.

## Why I started this

I wanted a flight controller that I actually understood, top to bottom, instead of just buying one off the shelf. I read enough about 3" quads to know roughly what a Flight controller needs to do (IMU, MCU, ESC outputs, receiver link), so I figured designing one myself was a good way to actually learn PCB design and embedded systems properly rather than just reading about it.

## Component choices

- **STM32F411CEU6** — Cortex-M4 at 100MHz, cheap, well documented, so there's plenty of reference material if I get stuck.
- **BMI270** — modern 6-axis IMU,cheap, SPI interface, low noise. I only use the main SPI interface (SDO/SDX/SCX/CSB), the auxiliary SPI/OIS pins (ASDX, ASCX, OCSB, OSDO) aren't used since I'm not attaching an external magnetometer, so those are marked no-connect.
- **LMR51430** buck converter for the 5V rail, **TLV75533** LDO for 3.3V off the 5V rail rather than straight off the battery,simpler to design and the LDO doesn't need to drop much voltage that way.
- USB-C instead of micro-USB, mostly because it's 2024+ and there's no excuse not to at this point.

## Mistake #1: crystal load capacitors

Early on I had my 16MHz crystal's load caps (C9/C10 at the time) set to **4.7µF**. Turns out I'd basically copy-pasted the value from a decoupling cap elsewhere in the BOM without thinking about it. Load caps for a crystal oscillator need to be in the **picofarad** range, not microfarad. If I'd sent that to fab as is, the oscillator simply wouldn't start, and the whole board would be dead on arrival with no obvious reason why from just looking at it. Caught this before ordering and fixed it to 10pF, which is a safe standard value for a 16MHz crystal in this kind of layout.

Lesson: double check *every* passive value that isn't obviously self-checking, especially anything copy-pasted between components that look similar in the schematic.

## Mistake #2: the first PCB layout (the big one)

This is the part I think is most worth documenting.

I have a 4S LiPo feeding this board, and each of the 4 motors can pull up to 15A, so worst case the input rail sees 60A. My plan was to just make the entire bottom copper layer a big `+BATT` zone and let the copper handle it. Sounded reasonable at the time.

After actually checking the filled copper geometry, I found two real probleRedesign PCB layout to fix power distribution issuesms:

1. **The battery plane was split into two disconnected islands.** My buck converter sat in the middle of the board, between the battery input and the ESC connectors, and its own footprint clearances plus a couple of GND cutouts ended up slicing the `+BATT` pour into two separate pieces. The regulator's own VIN pins were sitting on an island bridged back to the main plane by about a 0.3mm trace. That's fine for the regulator's own (small) current draw, but it meant the "big 60A plane" wasn't actually one continuous plane at all.

2. **Severe pinch points between the battery input and the ESC pads.** I measured the actual minimum copper width along the path from the battery connector to each ESC power pad and got numbers like 0.36mm–1.77mm in places. For reference, using the standard IPC-2221 trace sizing rule, you need roughly **8mm of width** on 1oz copper to carry 15A continuously without excessive heating. A 0.4mm neck is nowhere close, and it would've meant serious local heating right where a battery is plugged into a device with spinning props.

Neither of these show up as an obvious visual problem when you're just looking at "yeah there's a big copper zone there", you actually have to check the result and think about the current path, not just the outline.

Given how central the layout mistake was (the whole middle of the board was in the way of the power path), I decided patching it wasn't worth it and redid the layout from scratch, this time placing the battery input and ESC power pads first and building the power plane around them then placing the regulator on **TOP** layer.

## Redesign (v2 — what's actually in `hardware/kicad/` now)

- Rechecked the filled copper after placement and got like a single, fully connected `+BATT` region, with the narrowest points now in the 3–10mm range depending on which ESC, a big improvement, though I'd still like the tightest ones a bit wider if I revise this again.

- Decided to place the buck converter and LDO on the top layer, allowing the bottom layer to be only a region of `+BATT` and `GND`.

- Soldering pads instead of pin headers for the ST-LINK connector. I was running out of space! This actually has the added feature of not having ugly looking pins sticking out, meaning I could solder and desolder connections any time.

## Bottom Layer /L4

Made three 'islands' the main one being `+BATT` and three `GND` regions, two at the corners connecting each ESC's ground and a small zone around the `GND` of the Lipo soldering Pad. 

## Stitching (Vias)

I utilized the method of stitching, using several vias in a close region, to safely conduct the high current (60A worst case scenario). Nothing random was made here, using the KiCAD via size calculator, I knew a 0.6mm diameter via is capable of an estimated ampacity of about 4A. I made 18 vias inside each GND zone, and made sure they were close to the soldering pads.

## USB differential pair

I wanted the USB D+/D- pair impedance-matched to 90Ω since that's the USB2.0 target. Rather than trust a rough hand calculation, I used JLCPCB's own impedance calculator against their actual **JLC04161H-7628** stackup, which gave me the trace width and gap to use. This matters because a stackup's exact dielectric thickness changes the required geometry a lot, and a generic formula without the real numbers can be off by a meaningful margin.

## DRC and manufacturing file issues

Running an actual DRC check turned up a few things:

- 4 hole-clearance flags on the USB-C connector, between its mechanical alignment holes and its own ground pads. These turned out to be inherent to that specific connector's footprint accepted these as known and moved on rather than trying to force a fix that isn't really a fix.
- A couple of cosmetic silkscreen issues, fixed those since they're easy and free.

Separately from DRC, my exported BOM and CPL (pick-and-place) files both got rejected by JLCPCB's uploader on the first try:

- The BOM's header row had one more column (`DNP`) than the actual data rows did, which shifted every part's LCSC number into the wrong column as far as the parser was concerned.


## Current status

Board is ordered from JLCPCB (4-layer, HASL decision made based on assembly method, both sides populated since the power stage lives on the bottom layer). Once it's back, next steps are bring-up testing (power rails first, then IMU communication) before touching flight firmware.

## Things I'd do differently next time

- Check filled copper geometry for any high-current net considering a layout "done," not as an afterthought.
- Pick a standard mounting hole pattern from the start instead of placing holes arbitrarily and discovering the mismatch later.
- Keep passive component values in a spreadsheet with datasheet references while designing, instead of trusting memory/copy paste for things like crystal load caps.
