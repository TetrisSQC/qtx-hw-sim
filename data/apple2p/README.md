# Apple II+ board data (source material)

Exported 2026-11-18 with KiCad 10.0.8 (kicad-cli) from C:\Projekte\C64Sim\AppleII (since deleted: this export is now the source; KiCad 5 schematic,
"Captured from the Apple II Reference Manual (1979)", converted to .kicad_sch by opening and saving it in KiCad).

- apple2p.net.xml  netlist (kicadxml): 172 parts, 428 nets, 2037 pin connections, incl. lib parts with pin names
- apple2p.bom.csv  bill of materials
- apple2p.erc.txt  ERC report
- apple2p.pdf      the schematic, for reference

Schematic capture errors found and fixed in tools/gen_apple2.py:
- RAM rows: only C3/D3/E3 had their row's RA6 and /CAS; all chips of row X get XRA6 and ~{XCAS}.
- B8 (RAM latch, high nibble) drove LD0-LD3 like B5: it drives LD4-LD7.
- C13 (74LS51, SYNC): H3 on pins 10 and 11, one of them is H2 (SYNC = /(B14-1 B11-8 B14-13 + H2 H3 HBL), Gayler).
- ~{INH} had no pull-up: RA1 pin 7.
- Memory configuration blocks D1/E1/F1 (16K x 3): jumpers 12-8, 13-9, 14-10 (in hwsim.apple2).
- Q5 (reset) has E and C swapped in the symbol (modeled as a switch pulling /RESET low).

Known schematic capture issues (original list):
- Labels differing only in case are separate nets: SOFT5/Soft5, ~{IO-SELECT-n}/~{IO-Select-n} (n=1..7) -> merge.
- Unused gate units and the power units of many 74xx are not placed (missing_unit / missing_power_pin):
  their VCC/GND pins are absent -> tie chips to the rails in the generator.
- Ref "B80" looks like a typo (check against the board grid).
- 95 single-pin nets: mostly unused pins / slot pins, check the ones on chips.
- No PCB geometry: place parts from the Apple grid refs (A1..J14).
