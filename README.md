# qtx-hw-sim

Pin-level simulators of the C64, ZX Spectrum, Apple II+ and LC-80 boards in Quartex Pascal.

Based on **[c64-hw-simulator](https://github.com/antirez/c64-hw-simulator)** by Salvatore Sanfilippo
(antirez), a pin-level simulator of the C64 and ZX Spectrum motherboards in C. This repository is an
**altered version**, not the original: the simulator ported to Quartex Pascal (running in the browser
and in node.js) and extended. See LICENSE: the original's license, which applies here too, and the
parts that keep their own licenses.

Pin-level board simulators in Quartex Pascal, started as a port of antirez's c64-hw-simulator:
C64 ASSY 250407, ZX Spectrum 48K Issue 3B, Apple II+ (1979 schematic) and LC-80 (G-5403.500,
netlist captured from a schematic scan). While no fault is set and no pin is on the scope, every
machine runs in a fast functional mode (cycle exact against the pin level for C64, ZX and LC-80);
a fault switches to the pin-level board. See PORTING.md for design notes, compiler pitfalls,
status and the TODO list.

- `src/`            shared Pascal units (engine, parts, boards, CPU/VIC/CIA/SID/Z80/ULA/PIO/CTC models, web UI);
                    `hwsim.roms.pas` holds the ROM images (C64, ZX, LC-80, cartridges; no ROM files at run time).
                    The Apple II+ ROMs may not be published: `hwsim.roms_local.pas` is an empty stub (the Apple then
                    runs its built-in test ROM); `gen_data.py --rom apple2p_main=FILE --rom apple2p_char=FILE`
                    writes the filled unit to `src/local/` (not checked in), which build.py uses instead
- `hwsim_headless/` node.js project: `node --stack-size=8000 index.js --headless 200 --text [--machine zx48|apple2p|lc80]
                    [--fault U5:removed] [--cart deadtest-781220.crt]`; `PINLEVEL=1` forces pin level
- `hwsim_web/`      browser project: serve the folder (`python -m http.server 8765`) and open http://127.0.0.1:8765
- `tools/`          generators (board data/ROMs, Apple II and LC-80 netlists, 6502 and Z80 decoders), build.py
- `data/`           netlists: `apple2p/` (from the KiCad schematic), `lc80/lc80.pins.txt` (hand-captured)
- `tests/`          C reference harnesses and compare_faults.sh (C vs Pascal at pin level)

Build: `python tools/build.py hwsim_headless --run ...` or `python tools/build.py hwsim_web`
(needs the Quartex IDE running with its MCP server on port 3030).
ROMs: `python tools/gen_data.py ../c64-hw-simulator-main src [--rom KEY=FILE]` keeps the images already embedded.
The generated units (src/hwsim.bd_*.pas, src/hwsim.roms*.pas) are checked in, so building needs neither
c64-hw-simulator-main nor any ROM file; only regenerating them does.
