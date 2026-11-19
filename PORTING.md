# C64/ZX hw-sim -> Quartex Pascal port: working notes

Source: ../c64-hw-simulator-main (C, antirez). Target: Quartex Pascal (DWScript dialect -> JS).

## Tooling
- Quartex IDE MCP on http://localhost:3030/mcp (not registered in Claude Code).
  Helper scripts in session scratchpad: qx.py <tool> '<json>' | @argsfile ; qxput.py <local> [remote].
  Better: register permanently: `claude mcp add --transport http quartex http://localhost:3030/mcp`.
- Project hwsim_headless (node target) at qtx/hwsim_headless. Compile via MCP `compile`, run `node index.js`.
  app.config.ini: Obfuscation=0 (readable JS).
- C reference build (msys64 ucrt64 gcc):
  `gcc -O2 -std=gnu11 -Isrc -Ichips -Ithird_party -DSOKOL_GLCORE src/*.c -o c64-hw-sim.exe -lopengl32 -lgdi32 -lole32 -lkernel32 -luser32 -lshell32`
  `./c64-hw-sim.exe --headless 200 --text` -> READY screen (C64), `--machine zx48` -> (c) 1982 Sinclair.
  Runs at ~6% real time on this box (~60 s for 200 frames).

## Language facts (verified by probe)
- Integer = JS number. `and/or/xor` -> JS `& | ^` (signed 32-bit result!). `shr` -> `>>>` (unsigned). `shl` -> `<<`.
- $FFFFFFFF literal = 4294967295 (double). Records are COPIED on assignment (Copy$ helper) -> use classes for mutable state.
- Static arrays -> JS arrays. JUint8Array etc. available (new JUint8Array(n)), wraps like C uint8.
- Procedural types (procedure(...)) work, plain function refs.
- asm ... end; inline JS, pascal vars referenced with @name.
- Loop perf ~ native-ish (50M simple iters in 87 ms).

- COMPILER BUG: `b <> (x = y)` (Boolean compared with an equality) is emitted as `b != x == y` (wrong
  JS precedence). Use temp Booleans / xor. Relational inner ops (<, <=) happen to work.
- COMPILER BUG: float literals are emitted with 15 significant digits (9007199254740992.0 -> 9.00719925474099E15).
  Keep literals short / exact (powers of two in small steps).
- Case-insensitive names: locals must not shadow functions/fields (kmod/KMod, chr/Chr, PINS/pins, Drive/DRIVE).

## Design decisions
- 64-bit values:
  - cycle counters -> Float (double, exact to 2^53). UINT64_MAX sentinel -> constant 1e300 (CYCLE_NEVER).
  - bus bitmap (C64 44 bits, ZX 47 bits) -> two 32-bit words lo/hi; a bus never straddles a word
    (bit allocator skips to 32 if the bus doesn't fit).
  - RNG xorshift64 -> lo/hi pair, exact emulation (keeps determinism vs C).
  - chip core pin masks: m6502 bits 0..30 fit 32 bits; 6510 port P0..P5 (bits 32..37) kept in separate field.
    Other cores: check each (m6569, m6526, m6581, z80).
- C structs with mutable state -> Pascal classes. part_fn -> procedural type TPartFn = procedure(s: TSim; p: TPart).
- void* state -> TObject subclasses.

## Build layout
- qtx/src/*.pas: shared Pascal sources. tools/build.py copies them into a project dir, compiles via MCP, runs node.
  (IDE ignores app.config.ini search paths; it also rewrites the ini when the project is closed.)
- tools/gen_data.py: board data + ROMs (base64). tools/gen_m6502.py (+ m6502.pas.tmpl, cxlate.py): 6502 core.
- tools/cxlate.py: C subset -> Pascal translator (macro expansion, masks stores to field widths).
- tests/cpu6502_ref.c: C reference trace (flat RAM + ROMs); Pascal equivalent in app.body Test6502. Hash matched.

## Status
- [x] board data + ROM generators (python -> pascal units)
- [x] sim engine (hwsim.sim)
- [x] parts (hwsim.parts)
- [x] board (hwsim.board) - TBoard abstract, machines subclass it
- [x] m6502 core, verified bit-exact vs C (2M cycles)
- [x] m6569, m6526, m6581 cores (hand ports)
- [x] c64 machine, cart, harness (hwsim.c64)
- [x] headless driver (hwsim_headless/app.body.pas): boots to READY, identical to C.
      tests/compare_faults.sh: 7/8 fault scenarios identical; basic-removed differed only by
      register A -> found 2^53 literal bug in Noise, fixed (rerun pending).
- [x] z80 (tools/gen_z80.py + z80.pas.tmpl): registers in ArrayBuffer views, address bus in field
      'ab', control pins bits 0..15. Verified bit-exact vs tests/z80_ref.c (4M edges).
      Translator: gotos -> label variable; code after a conditional goto moved into else
      (no 'break' in case -> DWScript emits a real JS switch instead of an if-chain).
- [x] ula, zx48 (snapshots .sna/.z80, ROM cartridges). tests/compare_faults.sh 200 zx48: 5/5 identical.
- [x] browser UI: hwsim_web (src/hwsim.webui.pas, DOM via Variants, rAF time slicing).
      Tested in headless Chrome via CDP (tools: scratchpad cdp.mjs): C64 READY, ZX (c) 1982, Dead Test cart.
- Known deviation: C64 with BASIC ROM removed ends with register A different (screen identical);
  floating-input path (exp() drift + noise thresholds), likely libm vs V8 exp() last-ulp.
- Not ported: PNG screenshots, --record, ULA gate log (ulacheck), --flashes, inspector (gate drawings),
  file browser (replaced by file input + built-in cartridges), SID audio output (core runs, audio=false).

## TODO (later)
LC-80
- [ ] Tape: ST (save) / LD (load) - model the B861 input amplifier and the monitor's tape format,
      a WAV file as the tape (TAPE OUT is already sampled for the speaker).
- [ ] PIO interrupts (mode 3 bit match, mode 1/2 strobes); CTC counter mode and the TRG inputs.
- [ ] Not captured from the scan: power supply, bus connectors X1 (user bus) / X2 (CPU bus), the tape and
      speaker circuit (V220, B861). Check the netlist's interpretations (header of data/lc80/lc80.pins.txt)
      against a second source if one turns up.
- [ ] The other ROM sets are embedded but unused: lc80a_2716 (2K board), lc80_2 (LC-80.2), lc80e_* (LC-80e chess).
Sound
- [ ] Use TBoard.audio for the other machines: C64 SID (core runs, output unused), ZX beeper/EAR, Apple speaker ($C030).
Apple II+
- [ ] Disk II in slot 6 (13-sector ROMs in data/apple2roms), paddles/joystick (NE558), cassette.
- [ ] Fast mode: floating bus (slots and $C0xx return 0), colour (the monitor is monochrome).
ZX Spectrum
- [ ] Fast mode: floating bus (unused ports and INTA return $FF); tape loading through EAR.
Housekeeping
- [x] Removed 2026-11-19: hwsim_web/apple2/ (Apple ROM copies from the old fetch), forms/ (unused
      Quartex template), tools/__pycache__, stale tests/out summaries.
- [ ] Next machine: IBM PC 5150 at pin level (see memory / plan).

## Apple II+ (2026-11-19)
- data/apple2p: netlist from the KiCad 5 schematic (see its README). tools/gen_apple2.py -> src/hwsim.bd_apple2p.pas
  fixes: B5+B50/B8+B80 merged, case-only label duplicates merged, RAM rows' RA6//CAS per row, ~{INH} pull-up on RA1.7,
  TTL power pins tied to rails; geometry from the Apple grid refs.
- src/hwsim.ttl.pas: new TTL/MOS models (LS02..LS283, 8T97, 8T28, 9316 ROM, 2513, 555, NE558 stub).
- src/hwsim.apple2.pas: machine. sim.cycle = 14M half periods; oscillator driven on Q2 collector (into B2);
  6502 ticked on PHI0 falling edge; video sampled from VIDEO_DATA (B10.5) and SYNC (C13.8);
  memory config blocks D1/E1/F1 jumpered 12-8, 13-9, 14-10 (16K x 3); RA1 pull-ups; Q5 NPN reset; built-in test ROM.
- 2026-11-19: WORKS. Boots the real Apple II+ ROM (data/apple2roms/apple.rom + 3410036.bin, user-supplied) to
  "APPLE ][" and the ] prompt, headless and in the browser.
- ROMs built in: tools/gen_data.py embeds data/apple2roms/apple.rom + 3410036.bin into src/hwsim.roms.pas
  (ROM_APPLE2P_MAIN/CHAR; empty if the files are missing -> built-in test ROM). --rom / file drop still override.
  Capture errors found while debugging: B8 outputs (LD4-7), C13 pin 11 = H2. 74LS51 pinout fixed (TI).
  CPU data latched while PHI0 high (selects turn off at the falling edge). Pin level ~1.3% of real time.
- ROM files are the user's: never publish data/apple2roms, hwsim_web/apple2, src/hwsim.roms.pas or the built index.js.
- FAST MODE (Apple): while healthy (no pin/part/resistor fault; RESET ok) and no scope probe, TApple2.RunFast runs the
  6502 functionally on a flat 48K RAM + ROM + $C0xx (kbd, $C010, $C030, F14 soft switches) and RenderFast draws the
  picture from RAM into raw (~1000% of real time headless; the web UI throttles to real time). On a fault:
  FastToChips (RAM -> 48 x 4116 cells, soft -> F14.q, strobe -> B10.q2, kbd lines), back: ChipsToFast.
  Address -> cell mapping (CellOf) and RAM chip order were measured by logging the pin-level DRAM writes of a boot:
  bank $0000/$4000/$8000 = rows C/D/E, chip 3+b holds bit b; cell = col bits A6,A11,A9,A5,A4,A10,A13 | row bits
  A1,A8,A3,A0,A2,A7,A12 << 8. Picture calibrated pixel-exact against pin level: text at raw x 180/y 31,
  hi-res one dot (2 samples) right of text, lo-res one sample right, its first sample the previous block's colour.
  Board pins stay frozen in fast mode. Headless: A2PINS=1 forces pin level; UI checkbox does the same.
  Test: --at N:J14.1:open (harmless fault) switches to pin level mid-run, --at M:repair back.
- Speed: TSim.edge_commits (Apple only): edge_commit parts (4116, LS74/161/166/174/175/194/195) commit only
  when an input net changed (TNet.committers -> cdirty); full commit while a lifted MOS leg is noisy.

## ZX Spectrum fast mode (2026-11-19)
- TZX48.RunFast: same Z80 core on a flat 48K (fram) + ROM/cart; the ULA reduced to the moments its CPU clock can
  change: rising edge at h0 of an odd tick, falling edge at h1 of an even tick unless the contention gates of
  ula_step (mem/io with mreqt2/ioreqt2 latched while the clock is low) hold it high. INT = line 248, tick < 64.
  FastEdge answers the bus cycle put out at the previous edge (memory, port $FE: keyboard from key_matrix, EAR;
  INTA -> $FF; other ports $FF) then ticks the core. Picture: FastLine per line from RAM, the border per tick
  (border_shown = border at c - (c and 7) - 3).
- Verified cycle exact: every M1 fetch (address, line, tick) identical to pin level over 200 frames of boot
  (1.5M fetches), fault/repair round trip identical, .sna load identical. ~350% of real time headless.
- force_pins is now in TBoard (UI checkbox, headless PINLEVEL=1 or A2PINS=1). Harmless test fault: KB1.1:open.
- MakeCells: address -> 4116/4532 cell tables from the mux wiring traced once (DramCell traced it per call).

## C64 fast mode (2026-11-19)
- TC64.FastCycle = BoardCycle without the netlist: same 6510/VIC/CIA/SID cores in the same order; memory map
  from the PLA's own terms (pla_logic, tabulated over all 64K inputs; /CAS low, CPU on the bus, BA counts:
  an I/O read while BA is low selects RAM, as on the real board); the data bus as one byte fbus that keeps
  its last value (colour RAM upper nibble, $DE00-$DFFF, writes to $00/$01). CIA port inputs only sampled on
  CS cycles (as CiaTick does); keys joined by union of PA/PB lines; CIA2 PA6/PA7 read back PA4/PA5 through
  the 7406. 556: RESTORE pulse and power-on reset kept in its own state. Colour RAM = the 2114's cells.
- Not in fast mode: harness plugged, check mode, any fault. FastToChips settles the board before the CPU
  looks (else the 556's stale /RESET reset the CPU on the switch).
- Verified cycle exact: CPU input pins every cycle identical to pin level: boot 3.9M cycles, typing + BASIC
  program (SID/CIA/colour RAM/border) 17.6M with a 40-frame pin-level stretch in the middle, Dead Test
  (Ultimax) 2.9M, PRG load 5.8M. ~250% of real time headless, ~100% (throttled) in the browser.
- VIC: DecodePixels skips SunitDecode when no sprite is displayed (it does nothing then): +20%.
- tests/compare_faults.sh now exports PINLEVEL=1. Results: c64 7/8 (basic-removed: known noise diff), zx 5/5.

## ROMs not from hw-sim (2026-11-19)
- src/hwsim.roms.pas is their home. tools/gen_data.py reads a file of EXTRA_ROMS only when it exists and
  otherwise keeps the image already embedded, so the binaries can be deleted. One-off:
  python tools/gen_data.py ../c64-hw-simulator-main src --rom KEY=FILE  (KEY must be in EXTRA_ROMS).
- Public vs private (2026-11-19): Apple II+ ROMs (PRIVATE_ROMS) go into hwsim.roms_local: src/hwsim.roms_local.pas
  is an empty stub (checked in), the filled unit is src/local/hwsim.roms_local.pas (.gitignore'd); build.py copies
  src/local/*.pas over the stubs. Everything else (C64, ZX, LC-80, carts) is in hwsim.roms.pas and public.
- Embedded: Apple II+ (apple2p_main/char, private); LC-80: lc80_d202 + lc80_d203 (U505 1K each, the schematic's board),
  lc80a_2716 (2K), lc80_2 (4K), lc80e_0000/1000/c000 (LC-80e chess).

## LC-80 (2026-11-19)
- Source: lc80/schaltplan_lc80.jpg (scan, no PCB; the scan and the ROM files have since been deleted:
  data/lc80/lc80.pins.txt and the images in src/hwsim.roms.pas are the sources now). Netlist captured by hand into data/lc80/lc80.pins.txt (its header
  lists every interpretation); tools/gen_lc80.py -> src/hwsim.bd_lc80.pas (synthesized geometry, key table).
- src/hwsim.lc80.pas: U880 (Z80 core), U505D 1K ROM, Z80 PIO (modes 0-3, control words; no PIO interrupts),
  Z80 CTC (timers with prescaler, vector, interrupts, daisy chain IEI/IEO, RETI decoded from the bus;
  counter mode/triggers not modelled), VQE23 displays sampled by the machine (segment anode high while the
  digit's cathode is low, brightness per frame). Glue: DS8205 = PT_74LS138, DL014 = PT_74LS04, DL000 = 7400,
  U214D = PT_2114. New elements: EL_PNP (segment drivers), EL_PNP_SINK (digit drivers, drawn as PNP emitter
  followers: B low selects a digit, as the monitor expects).
- Found while bringing it up: the monitor's texts are stored right to left, so system PIO B2 is the rightmost
  digit; the digit table at $0A07 confirms the segment order b f a g dp c e d.
- Works: RAM test + ROM checksum pass, power-on melody with the running text "HALLO USEr OF LC-80", then
  " LC-80"; ADR 2 0 0 1 shows "2001 FF". Keys: 0-9 A-F + -, F1 ADR, F2 DAT, F3 ST, F4 LD, Enter EX, Esc RES, F5 NMI.
- Pin level 30-50% of real time. Tape (B861) not modelled (TAPE IN pulled up); the speaker is heard (see Sound).
- Fast mode (TLc80.RunFast): same Z80 core, the ROMs' data, the 2114s' own cells, the same TPioState/TCtcState
  objects and register functions (pio_read/pio_write, ctc_write/ack/reti/clock shared with the pin models),
  peripherals acting right after the edge as their commit does. Verified: every M1 fetch identical to pin level
  over the 30 s power-on (2.56M fetches), over a key session (2.78M), and with a fault/repair round trip.
  ~800% headless, real time in the browser.
- RETI is recognised when /M1 rises after an opcode fetch (/RD rises at the same edge, so "RD rising while M1
  low" never saw it).
- Monitor use (tested): ADR + 4 digits = address; DAT + 2 digits = byte there; + / - next/previous address;
  EX = run from the address; NMI breaks a running program ("Int"); RES = reset. ST/LD are the tape functions.
- Web UI: on-screen keypad (the schematic's key table) under the display for the LC-80; .bin (to $2000) and
  Intel .hex load into the RAM (TLc80.Load, the 2114 cells, both modes).
- Sound: TBoard.audio_rate/audio/AudioCycle (samples of a 0/1 line), played by the web UI through WebAudio
  (StartAudio on the first click/key, PlayAudio queues each frame ~50 ms ahead; checkbox "Sound").
  LC-80: TAPE OUT (system PIO B1, also the speaker via V220) at 22050 Hz. C64 SID / ZX beeper could use it.

