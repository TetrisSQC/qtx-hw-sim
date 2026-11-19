unit hwsim.apple2;

{ The Apple II+ motherboard, simulated at its pins.

  HOW TIME MOVES

  Everything is timed by the 14.318 MHz master oscillator (Y1 with Q1/Q2,
  squared up by a gate of B2). We simulate every half period of it: the
  oscillator changes level, the board settles, the 6502 sees an edge of
  PHI0 if the timing chain made one (it is clocked by the board, not the
  other way round), then the flip-flops, counters and memories see their
  clocks (a commit) and the board settles again. sim.cycle counts these
  half periods.

  What the timing chain makes (B1, C2, D11-D14, E11-E13...) is not known to
  this code: PHI0, /RAS, /CAS, the video counters, the dot clock and the
  shift register loads all come from the netlist and the TTL models.

  The video is read where the board makes it: VIDEO_DATA (the B10 flip-flop
  clocked by 14M) and SYNC (C13), sampled at every rising edge of 14M.

  FAST MODE

  Pin level costs about 28 board steps per CPU cycle, ~1% of real time.
  While the board is healthy (no fault, RESET aside, no scope probe) the
  machine runs functionally instead: the same 6502 core on a flat 48K RAM,
  the ROMs, the soft switches, the keyboard latch, the picture drawn from
  RAM once per frame. On a fault it hands its state to the chips (RAM into
  the 48 4116 cells, soft switches into F14, the keyboard strobe into B10)
  and the board takes over from there; back to healthy, it reads them back.
  The address -> 4116 cell mapping was taken from the pin-level board
  (logging the RAM writes of a boot).

  Board: the Apple II schematic captured from the Apple II Reference Manual
  (1979), see data/apple2p/README.md for what was corrected. }

interface

uses
  qtx.sysutils,
  hwsim.util, hwsim.boarddata, hwsim.bd_apple2p, hwsim.roms, hwsim.roms_local, hwsim.sim, hwsim.parts, hwsim.board,
  hwsim.m6502, hwsim.kbd, hwsim.ttl;

const
  A2_OSC_HZ = 14318180;
  A2_STEPS_HZ = 2 * 14318180;        { sim.cycle counts oscillator half periods }
  A2_LINE = 912;                     { 14M periods per line (65 CPU cycles, one long) }
  A2_LINES = 262;
  A2_FRAME_STEPS = 2 * 912 * 262;
  A2_RAW_W = 912;
  A2_RAW_H = 262;
  A2_SCREEN_W = 560;
  A2_SCREEN_H = 384;
  A2_CPU_FRAME = 17030;              { CPU cycles per frame (65 x 262) }
  A2_MODE_NONE = 0;
  A2_MODE_PINS = 1;
  A2_MODE_FAST = 2;
  { alignment of the fast picture with the board's (measured) }
  A2_FAST_X = 180;
  A2_FAST_Y = 31;
  A2_TEXT_SH = 6;                    { char dots: bit 6 the leftmost of 7 }
  A2_LORES_PH = 2;

type
  TApple2 = class(TBoard)
  private
    function PinOf(const ref: String; number: Integer): Integer;
    procedure SetElements(const ref: String; const pairs: array of Integer; typ: Integer);
    procedure Step;
    procedure CpuEdge;
    procedure VideoSample;
    procedure RunFast(cycles: Integer);
    function FastRead(addr: Integer): Integer;
    procedure FastWrite(addr, v: Integer);
    procedure FastIO(addr: Integer);
    function Healthy: Boolean;
    procedure ChipsToFast;
    procedure FastToChips;
  protected
    procedure Describe; override;
    procedure MachineCreate; override;
    procedure MachinePowerOn; override;
  public
    cpu: TM6502;
    cpu_pins: Integer;
    p_cpu, p_reset555: Integer;
    pin_osc, pin_phi0, pin_video, pin_sync: Integer;
    osc: Integer;
    phi0: Integer;
    cpu_running, in_reset: Boolean;
    last_edge: Float;
    cache_addr, cache_data: Integer;
    { keyboard: data pins of A7 (bit 0..6) and the strobe }
    pin_kbd: array[0..6] of Integer;
    pin_strobe: Integer;
    strobe_left: Integer;
    kbd_queue: array of Integer;
    { video: raw picture, one sample per 14M period }
    raw: TBytes;
    vx, vy, sync_len, last_sync: Integer;
    rom_main: TBytes;        { $D000-$FFFF, 12K }
    rom_char: TBytes;        { 2513 image, 512 bytes (char * 8 + row, 5 dots, bit 4 leftmost) }
    rom_name: String;
    n_phi0, n_lines, n_vsync: Integer;
    runlen, runlvl: Integer;
    trace_left, trace_on: Integer;
    data_latch: Integer;
    dbg_line: Integer;
    dbg: String;   { the data bus as the CPU saw it while PHI0 was high }
    hist: array of String;   { counters, for checking the timing }
    { fast mode }
    mode: Integer;           { A2_MODE_* }
    ram: TBytes;             { 48K, $0000-$BFFF }
    soft: Integer;           { F14: bit 0 TEXT, 1 MIX, 2 PG2, 3 HIRES, 4-7 ANN0-3 }
    kbd_val, kbd_strobe: Integer;
    cpu_acc, frame_acc: Float;
    n_frames: Integer;
    speaker: Integer;
    healthy_epoch: Integer;
    healthy_cache: Boolean;

    constructor Create; override;
    procedure Run(cycles: Integer); override;
    procedure InstallRoms;
    procedure KeyDown(key: Integer); override;
    procedure KeyUp(key: Integer); override;
    procedure FrameUpdate(us: Integer); override;
    function HostKey(const code: String): Integer; override;
    function HostChar(c: Integer): Integer; override;
    procedure Monitor(outp: TUInts); override;
    function FileKind(const path: String; const head: TBytes; var info: String; var bad: Boolean): TFileKind; override;
    function Load(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean; override;
    function CpuStatus: String; override;
    function Find(const ref: String): TPart;
    function Fast: Boolean;
    procedure RenderFast;
    function CpuRegs: String; override;
    function ScreenText: array of String; override;
  end;

var
  PT_6502: TPartType;

implementation

const
  CPU_ADDR_PINS: array[0..15] of Integer = [9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25];
  CPU_DATA_PINS: array[0..7] of Integer = [33, 32, 31, 30, 29, 28, 27, 26];
  GK_A2_DATA = GK_MACHINE + 300;

{ ------------------------------------------------------------------------ }
{ The test ROM: until a real ROM set is loaded. Text mode, the screen       }
{ filled with every character code, then each key typed goes to the top     }
{ left corner. Written for this simulator (not Apple's code).               }
{ ------------------------------------------------------------------------ }

const
  TEST_CODE: array[0..37] of Integer = [
    $AD, $51, $C0,        { F800 LDA $C051   text }
    $AD, $54, $C0,        { F803 LDA $C054   page 1 }
    $A2, $00,             { F806 LDX #0 }
    $8A,                  { F808 TXA }
    $9D, $00, $04,        { F809 STA $0400,X }
    $9D, $00, $05,        { F80C STA $0500,X }
    $9D, $00, $06,        { F80F STA $0600,X }
    $9D, $00, $07,        { F812 STA $0700,X }
    $E8,                  { F815 INX }
    $D0, $F0,             { F816 BNE $F808 }
    $AD, $00, $C0,        { F818 LDA $C000 }
    $10, $FB,             { F81B BPL $F818 }
    $8D, $00, $04,        { F81D STA $0400 }
    $8D, $10, $C0,        { F820 STA $C010 }
    $4C, $18, $F8];       { F823 JMP $F818 }

function TestMainRom: TBytes;
var i: Integer;
begin
  Result := NewBytes(12288);
  for i := 0 to 12287 do Result[i] := $EA;
  for i := 0 to TEST_CODE.High do Result[$2800 + i] := TEST_CODE[i];
  { vectors at $FFFA: NMI, RESET, IRQ -> $F800 }
  for i := 0 to 2 do begin
    Result[$2FFA + i * 2] := $00;
    Result[$2FFB + i * 2] := $F8;
  end;
end;

{ A test character set: the 64 glyphs of the 2513 are, in this order,
  @A..Z[\]^_ and space!"#..? - the same order as the first 64 glyphs of the
  C64 character ROM, whose middle five columns are used here. }
function TestCharRom: TBytes;
var
  c, r: Integer;
  src: TBytes;
begin
  src := ROM_DUMP_C64_CHAR_BIN;
  Result := NewBytes(512);
  for c := 0 to 63 do
    for r := 0 to 7 do
      Result[c * 8 + r] := (src[c * 8 + r] shr 2) and $1F;
end;

{ ------------------------------------------------------------------------ }
{ 6502 (U3)                                                                 }
{ ------------------------------------------------------------------------ }

procedure cpu_eval(s: TSim; p: TPart);
var
  a: TApple2;
  pins: Integer;
  write, ph: Boolean;
begin
  a := TApple2(p.user);
  pins := a.cpu_pins;
  ph := s.PinIn(p, 37) <> 0;
  s.PinOut(p, 3, DRV_BIT(not ph));           { Phi1 out }
  s.PinOut(p, 39, DRV_BIT(ph));              { Phi2 out }
  a.cache_addr := part_drive_bus_cached(s, p, CPU_ADDR_PINS, 16, M6502_GET_ADDR(pins), True, a.cache_addr);
  write := (pins and M6502_RW) = 0;
  s.PinOut(p, 34, DRV_BIT(not write));
  a.cache_data := part_drive_bus_cached(s, p, CPU_DATA_PINS, 8, M6502_GET_DATA(pins), write and ph, a.cache_data);
end;

{ An edge of PHI0. Falling: the end of phi2, the CPU latches the data bus
  (on a read) and starts the next cycle: the core ticks and puts out the
  new address. }
procedure TApple2.CpuEdge;
var
  s: TSim;
  p: TPart;
  pins: Integer;
begin
  s := sim;
  p := s.parts[p_cpu];
  last_edge := s.cycle;
  if p.dead then exit;
  pins := cpu_pins;
  if (pins and M6502_RW) <> 0 then pins := M6502_SET_DATA(pins, data_latch);
  if s.PinIn(p, 40) = 0 then begin
    in_reset := True;
    cpu_pins := pins;
    exit;
  end;
  if in_reset then begin
    in_reset := False;
    pins := pins or M6502_RES;
    trace_left := trace_on;
  end;
  if trace_left > 0 then begin
    Dec(trace_left);
    if (pins and M6502_RW) <> 0 then
      WriteLn('cpu ' + HexW(M6502_GET_ADDR(pins)) + ' rd ' + HexB(M6502_GET_DATA(pins)) + '  PC=' + HexW(cpu.PC))
    else
      WriteLn('cpu ' + HexW(M6502_GET_ADDR(pins)) + ' WR ' + HexB(M6502_GET_DATA(pins)) + '  PC=' + HexW(cpu.PC));
  end;
  pins := pins and not (M6502_IRQ or M6502_NMI or M6502_RDY);
  if s.PinIn(p, 4) = 0 then pins := pins or M6502_IRQ;
  if s.PinIn(p, 6) = 0 then pins := pins or M6502_NMI;
  if s.PinIn(p, 2) = 0 then pins := pins or M6502_RDY;
  cpu_pins := cpu.Tick(pins);
  s.MarkDirty(p_cpu);
end;

{ ------------------------------------------------------------------------ }
{ Time                                                                      }
{ ------------------------------------------------------------------------ }

procedure TApple2.VideoSample;
var
  s: TSim;
  v, sy: Integer;
begin
  s := sim;
  v := s.Read(pin_video);
  sy := s.Read(pin_sync);
  if sy = runlvl then Inc(runlen) else begin
    if hist.Length < 40 then hist.Add(IntToStr(runlvl) + ':' + IntToStr(runlen));
    runlvl := sy; runlen := 1;
  end;
  { SYNC is low during a sync pulse: a pulse starts a line; a long one
    (vertical sync, several lines) starts the frame. }
  if sy = 0 then begin
    Inc(sync_len);
    if (last_sync <> 0) then begin
      vx := 0;
      Inc(vy);
      Inc(n_lines);
    end;
    if sync_len = 200 then begin vy := -4; Inc(n_vsync); end;
  end else
    sync_len := 0;
  last_sync := sy;
  if (vx < A2_RAW_W) and (vy >= 0) and (vy < A2_RAW_H) then raw[vy * A2_RAW_W + vx] := v;
  if (dbg_line > 0) and (vy = dbg_line) and (vx >= 230) and (vx < 290) then
    dbg := dbg + IntToStr(s.Read(PinOf('B12', 8))) + IntToStr(s.Read(PinOf('A3', 15))) + IntToStr(s.Read(PinOf('B1', 15))) +
      IntToStr(s.Read(PinOf('A10', 12))) + IntToStr(s.Read(PinOf('A3', 13))) + IntToStr(s.Read(PinOf('B2', 11))) + IntToStr(v) + ' ';
  Inc(vx);
end;

procedure TApple2.Step;
var
  s: TSim;
  ph: Integer;
begin
  s := sim;
  s.cycle := s.cycle + 1;
  osc := osc xor 1;
  s.Drive(pin_osc, DRV_BIT(osc));
  s.Settle;
  ph := s.Read(pin_phi0);
  { The 6502 takes the data bus at the end of phi2: keep the last value seen
    while PHI0 is high (the falling edge itself turns the selects off). }
  if (ph <> 0) and ((cpu_pins and M6502_RW) <> 0) then
    data_latch := part_read_bus(s, s.parts[p_cpu], GK_A2_DATA, CPU_DATA_PINS, 8);
  if ph <> phi0 then begin
    phi0 := ph;
    if ph = 0 then begin
      Inc(n_phi0);
      CpuEdge;
      s.Settle;
    end;
  end;
  s.Commit;
  s.Settle;
  if osc <> 0 then VideoSample;
  Sample;
end;

procedure TApple2.Run(cycles: Integer);
var
  i, k, want: Integer;
begin
  if (not force_pins) and (scope_pin < 0) and Healthy then want := A2_MODE_FAST else want := A2_MODE_PINS;
  if want <> mode then begin
    if want = A2_MODE_FAST then ChipsToFast
    else if mode = A2_MODE_FAST then FastToChips;
    mode := want;
  end;
  if mode = A2_MODE_FAST then begin RunFast(cycles); exit; end;
  for i := 0 to cycles - 1 do begin
    Step;
    if strobe_left > 0 then begin
      Dec(strobe_left);
      if strobe_left = 0 then sim.Drive(pin_strobe, DRV_LO);
    end else if (kbd_queue.Length > 0) and ((sim.cycle mod 4096) = 0) then begin
      { the keyboard encoder: 7 data lines, then a strobe pulse }
      for k := 0 to 6 do sim.Drive(pin_kbd[k], DRV_BIT((kbd_queue[0] shr k) and 1));
      sim.Drive(pin_strobe, DRV_HI);
      strobe_left := 512;
      kbd_queue.Delete(0);
    end;
  end;
  cpu_running := sim.cycle - last_edge < 1000;
end;

{ ------------------------------------------------------------------------ }
{ Fast mode                                                                 }
{ ------------------------------------------------------------------------ }

var
  CellOf: array of Integer;          { address and $3FFF -> 4116 cell }

procedure MakeCellOf;
const
  { cell bit -> address bit: column (cell bits 0-6), row (cell bits 8-14) }
  COLB: array[0..6] of Integer = [6, 11, 9, 5, 4, 10, 13];
  ROWB: array[0..6] of Integer = [1, 8, 3, 0, 2, 7, 12];
var a, k, c: Integer;
begin
  if CellOf.Length > 0 then exit;
  CellOf.SetLength(16384);
  for a := 0 to 16383 do begin
    c := 0;
    for k := 0 to 6 do begin
      c := c or (((a shr COLB[k]) and 1) shl k);
      c := c or (((a shr ROWB[k]) and 1) shl (k + 8));
    end;
    CellOf[a] := c;
  end;
end;

function TApple2.Fast: Boolean;
begin
  Result := mode = A2_MODE_FAST;
end;

{ Healthy enough for fast mode: no pin fault, nothing removed, no resistor
  fault. (RESET held is fine: fast mode has it.) }
function TApple2.Healthy: Boolean;
var i: Integer;
begin
  if sim.epoch = healthy_epoch then exit(healthy_cache);
  healthy_epoch := sim.epoch;
  healthy_cache := False;
  for i := 0 to sim.npins - 1 do
    if sim.pins[i].fault <> PF_OK then exit(False);
  for i := 0 to sim.nparts - 1 do
    if sim.parts[i].removed or sim.parts[i].dead or (sim.parts[i].rfault <> RF_OK) then exit(False);
  healthy_cache := True;
  Result := True;
end;

{ The RAM chip holding bit b of bank k (0: $0000, 1: $4000, 2: $8000). }
function RamChip(a: TApple2; bank, b: Integer): TPart;
begin
  if bank = 0 then Result := a.Find('C' + IntToStr(3 + b))
  else if bank = 1 then Result := a.Find('D' + IntToStr(3 + b))
  else Result := a.Find('E' + IntToStr(3 + b));
end;

procedure TApple2.ChipsToFast;
var
  bank, b, x, v: Integer;
  p: TPart;
  cells: TBytes;
begin
  MakeCellOf;
  if ram = nil then ram := NewBytes($C000);
  ram.fill(0);
  for bank := 0 to 2 do
    for b := 0 to 7 do begin
      p := RamChip(Self, bank, b);
      if (p = nil) or (p.state = nil) then continue;
      cells := TDramState(p.state).cells;
      for x := 0 to 16383 do begin
        v := cells[CellOf[x]];
        if v <> 0 then ram[bank * 16384 + x] := ram[bank * 16384 + x] or (1 shl b);
      end;
    end;
  p := Find('F14');
  if (p <> nil) and (p.state <> nil) then soft := TRegState(p.state).q and $FF;
  p := Find('B10');
  if (p <> nil) and (p.state <> nil) then kbd_strobe := TRegState(p.state).q2 and 1;
  cpu_acc := 0;
end;

procedure TApple2.FastToChips;
var
  bank, b, x: Integer;
  p: TPart;
  cells: TBytes;
begin
  MakeCellOf;
  for bank := 0 to 2 do
    for b := 0 to 7 do begin
      p := RamChip(Self, bank, b);
      if (p = nil) or (p.state = nil) then continue;
      cells := TDramState(p.state).cells;
      for x := 0 to 16383 do cells[CellOf[x]] := (ram[bank * 16384 + x] shr b) and 1;
      sim.MarkDirty(p.index);
    end;
  p := Find('F14');
  if (p <> nil) and (p.state <> nil) then begin TRegState(p.state).q := soft; sim.MarkDirty(p.index); end;
  p := Find('B10');
  if (p <> nil) and (p.state <> nil) then begin TRegState(p.state).q2 := kbd_strobe; sim.MarkDirty(p.index); end;
  { the keyboard encoder's lines keep the last key }
  for b := 0 to 6 do sim.Drive(pin_kbd[b], DRV_BIT((kbd_val shr b) and 1));
  strobe_left := 0;
  sim.Drive(pin_strobe, DRV_LO);
end;

{ $C000-$C0FF: what F13 decodes. Reads and writes act alike. }
procedure TApple2.FastIO(addr: Integer);
var bit: Integer;
begin
  case (addr shr 4) and 15 of
    1: kbd_strobe := 0;                                   { $C010 clear strobe }
    3: speaker := speaker xor 1;                          { $C030 }
    5: begin
         bit := 1 shl ((addr shr 1) and 7);
         if (addr and 1) <> 0 then soft := soft or bit else soft := soft and not bit;
       end;
  end;
end;

function TApple2.FastRead(addr: Integer): Integer;
begin
  if addr < $C000 then exit(ram[addr]);
  if addr >= $D000 then exit(rom_main[addr - $D000]);
  if addr < $C100 then begin
    FastIO(addr);
    if addr < $C010 then exit(kbd_val or (kbd_strobe shl 7));
    if addr < $C020 then exit(kbd_val);
  end;
  Result := 0;     { nothing in the slots; the floating bus is not modelled }
end;

procedure TApple2.FastWrite(addr, v: Integer);
begin
  if addr < $C000 then ram[addr] := v
  else if addr < $C100 then FastIO(addr);
end;

procedure TApple2.RunFast(cycles: Integer);
var
  s: TSim;
  pins, addr, k: Integer;
  reset_low: Boolean;
  st: TTimer555State;
begin
  s := sim;
  s.cycle := s.cycle + cycles;
  { the keyboard encoder: the next key once the last one was taken }
  if (kbd_queue.Length > 0) and (kbd_strobe = 0) then begin
    kbd_val := kbd_queue[0];
    kbd_queue.Delete(0);
    kbd_strobe := 1;
    for k := 0 to 6 do s.Drive(pin_kbd[k], DRV_BIT((kbd_val shr k) and 1));
  end;
  st := TTimer555State(s.parts[p_reset555].state);
  reset_low := reset_button or ((st <> nil) and (s.cycle < st.until_cycle));
  cpu_acc := cpu_acc + cycles * A2_CPU_FRAME;
  while cpu_acc >= A2_FRAME_STEPS do begin
    cpu_acc := cpu_acc - A2_FRAME_STEPS;
    if reset_low then begin in_reset := True; continue; end;
    pins := cpu_pins;
    if (pins and M6502_RW) <> 0 then pins := M6502_SET_DATA(pins, data_latch);
    if in_reset then begin in_reset := False; pins := pins or M6502_RES; end;
    pins := pins and not (M6502_IRQ or M6502_NMI or M6502_RDY);
    pins := cpu.Tick(pins);
    cpu_pins := pins;
    addr := M6502_GET_ADDR(pins);
    if (pins and M6502_RW) <> 0 then data_latch := FastRead(addr)
    else FastWrite(addr, M6502_GET_DATA(pins));
  end;
  last_edge := s.cycle;
  cpu_running := True;
  frame_acc := frame_acc + cycles;
  if frame_acc >= A2_FRAME_STEPS then begin
    frame_acc := frame_acc - A2_FRAME_STEPS;
    Inc(n_frames);
    RenderFast;
  end;
end;

{ The picture from RAM, into the raw buffer where the board's video lands:
  192 lines of 40 x 14 samples (7 dots of two 14M periods each). }
procedure TApple2.RenderFast;
var
  y, col, k, a, v, row, c, dots, base, nib, x0, prev: Integer;
  gr, hires, mixed, flash_on, txt: Boolean;
begin
  gr := (soft and 1) = 0;           { Q0 high: text ($C051) }
  mixed := (soft and 2) <> 0;
  hires := (soft and 8) <> 0;
  flash_on := ((n_frames shr 4) and 1) <> 0;
  for y := 0 to 191 do begin
    base := (A2_FAST_Y + y) * A2_RAW_W + A2_FAST_X;
    prev := 0;
    txt := (not gr) or (mixed and (y >= 160));
    for col := 0 to 39 do begin
      x0 := base + col * 14;
      if (not txt) and hires then begin
        a := $2000 + ((soft and 4) shl 11) + (y and 7) * $400 + ((y shr 3) and 7) * $80 + (y shr 6) * $28 + col;
        v := ram[a];
        { 7 dots, bit 0 first; bit 7 delays them by half a dot }
        for k := 0 to 13 do begin
          if (v and $80) <> 0 then begin
            if k = 0 then dots := prev else dots := (v shr ((k - 1) shr 1)) and 1;
          end else
            dots := (v shr (k shr 1)) and 1;
          raw[x0 + 2 + k] := dots;      { the board: one dot right of text }
        end;
        if col = 0 then begin raw[x0] := 0; raw[x0 + 1] := 0; end;
        prev := (v shr 6) and 1;
      end else begin
        row := y shr 3;
        a := $400 + ((soft and 4) shl 8) + (row and 7) * $80 + (row shr 3) * $28 + col;
        v := ram[a];
        if not txt then begin
          { lo-res: the colour nibble shifted out as a 4-period pattern }
          if (y and 7) < 4 then nib := v and 15 else nib := v shr 4;
          { (the board puts lo-res blocks one 14M period right of text) }
          { and its first period still shows the previous block's colour }
          if col = 0 then begin raw[x0] := 0; prev := 0; end;
          raw[x0 + 1] := (prev shr ((col * 14 + 1 + A2_LORES_PH) and 3)) and 1;
          for k := 1 to 13 do raw[x0 + 1 + k] := (nib shr ((col * 14 + k + 1 + A2_LORES_PH) and 3)) and 1;
          prev := nib;
        end else begin
          c := v and $3F;
          dots := rom_char[c * 8 + (y and 7)] and $1F;
          if (v < $40) or ((v < $80) and flash_on) then dots := (not dots) and $7F;
          for k := 0 to 13 do raw[x0 + k] := (dots shr (A2_TEXT_SH - (k shr 1))) and 1;
        end;
      end;
    end;
  end;
end;

{ ------------------------------------------------------------------------ }
{ The board                                                                 }
{ ------------------------------------------------------------------------ }

function a2_rail_of(const name: String): Integer;
begin
  if (name = 'GND') or (name = '-5V') or (name = '-12V') then exit(RAIL_GND);
  if (name = '+5V') or (name = '+12V') then exit(RAIL_VCC);
  Result := RAIL_NONE;
end;

function a2_kind_of(const ref: String): Integer;
var c1: String;
begin
  c1 := Copy(ref, 2, 1);
  if (ref = 'U3') or (Copy(ref, 1, 4) = 'ROM-') then exit(PART_IC);
  { grid refs that are not chips }
  if (ref = 'A7') or (ref = 'J14') or (ref = 'J18') then exit(PART_CONNECTOR);
  if ref = 'C15' then exit(PART_OTHER);
  if (Copy(ref, 1, 4) = 'SLOT') or (ref = 'J0') or (ref = 'J2') or (ref = 'K14') or (ref = 'CP0') or
     (ref = 'SPKR1') then exit(PART_CONNECTOR);
  if (Length(ref) >= 2) and (Pos(Copy(ref, 1, 1), 'ABCDEFGHJ') > 0) and (c1 >= '0') and (c1 <= '9') then exit(PART_IC);
  if (Copy(ref, 1, 1) = 'R') then exit(PART_RESISTOR);
  if Copy(ref, 1, 2) = 'JP' then exit(PART_LINK);
  Result := PART_OTHER;
end;

constructor TApple2.Create;
begin
  inherited Create;
  cpu := TM6502.Create;
end;

function RoleOf(const v: String): Integer;
begin
  if v = '4116' then exit(ROLE_RAM);
  if (v = 'ROM_Chip') or (Pos('2513', v) > 0) then exit(ROLE_ROM);
  if v = 'MOS6502' then exit(ROLE_CHIP);
  if (v = '74LS161') or (v = '74LS175') or (v = '74LS195') then exit(ROLE_CLOCK);
  Result := ROLE_GLUE;
end;

procedure TApple2.Describe;
var
  bd: TBoardData;
  i: Integer;
  ref, v: String;
  t: TPartType;
begin
  id := 'apple2p';
  name := 'Apple II+';
  board_name := 'Apple II+ (schematic of the 1979 Reference Manual)';
  data := BoardApple2p;
  simdesc.rail_of := a2_rail_of;
  simdesc.kind_of := a2_kind_of;
  simdesc.cycles_per_us := A2_STEPS_HZ / 1000000;
  cpu_hz := A2_STEPS_HZ;
  cycles_per_frame := A2_FRAME_STEPS;
  steps_per_cycle := 2;
  screen_w := A2_SCREEN_W;
  screen_h := A2_SCREEN_H;
  ready_cycles := 0.3 * A2_STEPS_HZ;
  link_resistors := 'R14';
  crystals := 'Y1';
  keys_help := 'Keys: the Apple II+ keyboard (upper case); ESC, RETURN, arrows';
  reset_help := 'Hold to press RESET';
  files_title := 'Open a ROM set (12K $D000-$FFFF) or a character ROM (2513 image)';
  cart_slot := '';
  bd := data;
  for i := 0 to bd.ncomps - 1 do begin
    ref := bd.comps[i].ref;
    v := bd.comps[i].value;
    t := nil;
    if v = '74LS00' then t := PT_74LS00x
    else if v = '74LS02' then t := PT_74LS02
    else if v = '74LS04' then t := PT_74LS04
    else if v = '74LS08' then t := PT_74LS08x
    else if v = '74LS11' then t := PT_74LS11
    else if v = '74LS20' then t := PT_74LS20
    else if v = '74LS32' then t := PT_74LS32x
    else if v = '74LS51' then t := PT_74LS51
    else if v = '74LS74' then t := PT_74LS74x
    else if v = '74LS86' then t := PT_74LS86
    else if v = '74LS138' then t := PT_74LS138
    else if v = '74LS139' then t := PT_74LS139x
    else if v = '74LS151' then t := PT_74LS151
    else if v = '74LS153' then t := PT_74LS153
    else if v = '74LS161' then t := PT_74LS161
    else if v = '74LS166' then t := PT_74LS166
    else if v = '74LS174' then t := PT_74LS174
    else if v = '74LS175' then t := PT_74LS175
    else if v = '74LS194' then t := PT_74LS194
    else if v = '74LS195' then t := PT_74LS195
    else if v = '74LS251' then t := PT_74LS251
    else if v = '74LS257' then t := PT_74LS257x
    else if v = '74LS259' then t := PT_74LS259
    else if v = '74LS283' then t := PT_74LS283
    else if v = '8T97' then t := PT_8T97
    else if v = '8T28' then t := PT_8T28
    else if v = '4116' then t := PT_4116
    else if v = 'ROM_Chip' then t := PT_2316
    else if Pos('2513', v) > 0 then t := PT_2513
    else if v = 'MOS6502' then t := PT_6502
    else if v = '555' then t := PT_555A
    else if v = 'NE558' then t := PT_NE558;
    if t <> nil then
      part_defs.Add(TPartDef.Create(ref, t, ref, RoleOf(v), v, v + ' at ' + ref));
  end;
  conn_defs.Add(TConnDef.Create('A7', 'KEYBOARD', '1=+5V 2=STROBE 3=/RESET 5=B5 6=B4 7=B6 8=GND 10=B2 11=B3 12=B0 13=B1 15=-12V'));
  conn_defs.Add(TConnDef.Create('J14', 'GAME I/O', '1=+5V 2=SW0 3=SW1 4=SW2 5=/STB 6=PDL0 7=PDL2 8=GND 10=PDL1 11=PDL3 12=AN3 13=AN2 14=AN1 15=AN0'));
  conn_defs.Add(TConnDef.Create('K14', 'VIDEO OUT', ''));
  conn_defs.Add(TConnDef.Create('J18', 'AUX VIDEO', ''));
  conn_defs.Add(TConnDef.Create('J0', 'CASSETTE IN', ''));
  conn_defs.Add(TConnDef.Create('J2', 'CASSETTE OUT', ''));
  conn_defs.Add(TConnDef.Create('CP0', 'POWER', ''));
  conn_defs.Add(TConnDef.Create('SPKR1', 'SPEAKER', ''));
end;

function TApple2.Find(const ref: String): TPart;
var i: Integer;
begin
  i := sim.FindPart(ref);
  if i >= 0 then Result := sim.parts[i] else Result := nil;
end;

function TApple2.PinOf(const ref: String; number: Integer): Integer;
var p: TPart;
begin
  p := Find(ref);
  if p <> nil then Result := p.Pin(number) else Result := -1;
end;

{ Replace the elements of a passive part (pairs of pin numbers). }
procedure TApple2.SetElements(const ref: String; const pairs: array of Integer; typ: Integer);
var
  p: TPart;
  st: TResistorState;
  e: TElement;
  i: Integer;
begin
  p := Find(ref);
  if p = nil then exit;
  st := TResistorState(p.state);
  if st = nil then begin
    st := TResistorState.Create;
    p.state := st;
  end;
  p.kind := PART_RESISTOR;
  st.el.Clear;
  st.nel := 0;
  i := 0;
  while i + 1 <= pairs.High do begin
    e := TElement.Create;
    e.pin_a := p.Pin(pairs[i]);
    e.pin_b := p.Pin(pairs[i + 1]);
    e.typ := EL_ANALOG;
    if (e.pin_a >= 0) and (e.pin_b >= 0) then begin
      st.el.Add(e);
      Inc(st.nel);
      if typ = EL_PULLUP then begin
        { pin_a is the pulled line, the rail side doesn't matter }
        e.typ := EL_PULLUP;
        p.eval := resistor_eval;
      end else if typ = EL_LINK then
        e.typ := EL_LINK
      else
        element_set_type(sim, p, st.nel - 1, typ);
    end;
    i := i + 2;
  end;
end;

procedure TApple2.MachineCreate;
var
  s: TSim;
  i, bit, k, n: Integer;
  prefixes: array[0..3] of String;
  counts: array[0..3] of Integer;
  p: TPart;
begin
  s := sim;
  s.edge_commits := True;
  p_cpu := s.FindPart('U3');
  p_reset555 := s.FindPart('A15');
  pin_phi0 := s.parts[p_cpu].Pin(37);
  pin_osc := PinOf('Q2', 2);           { the oscillator's collector, into B2 }
  pin_video := PinOf('B10', 5);        { VIDEO_DATA }
  pin_sync := PinOf('C13', 8);         { SYNC }
  for k := 0 to 6 do pin_kbd[k] := -1;
  pin_kbd[0] := PinOf('A7', 12); pin_kbd[1] := PinOf('A7', 13); pin_kbd[2] := PinOf('A7', 10);
  pin_kbd[3] := PinOf('A7', 11); pin_kbd[4] := PinOf('A7', 6); pin_kbd[5] := PinOf('A7', 5);
  pin_kbd[6] := PinOf('A7', 7);
  pin_strobe := PinOf('A7', 2);
  { The memory configuration blocks, wired for 16K chips in all three rows
    (48K): /CAS (F1), RA6 (E1) and the RAM select (D1) of each row. }
  SetElements('D1', [12, 8, 13, 9, 14, 10], EL_LINK);
  SetElements('E1', [12, 8, 13, 9, 14, 10], EL_LINK);
  SetElements('F1', [12, 8, 13, 9, 14, 10], EL_LINK);
  for i := 0 to 2 do begin
    if i = 0 then p := Find('D1') else if i = 1 then p := Find('E1') else p := Find('F1');
    if p <> nil then p.kind := PART_LINK;
  end;
  { Pull-ups: RA1 (pin 1 to +5V, pins 2-8 the lines), RA22. }
  SetElements('RA1', [2, 1, 3, 1, 4, 1, 5, 1, 6, 1, 7, 1, 8, 1], EL_PULLUP);
  SetElements('RA22', [1, 2], EL_PULLUP);
  { RA2/RA3 terminate the RAM address lines (to +5V and to GND): analog. }
  SetElements('RA2', [], EL_ANALOG);
  SetElements('RA3', [], EL_ANALOG);
  { The power-on reset: A15 (a 555) through R14 into Q5, which pulls
    /RESET low. Q5's symbol has emitter and collector swapped. }
  SetElements('R14', [1, 2], EL_LINK);
  SetElements('Q5', [2, 1], EL_NPN);
  { The 50/60 Hz solder jumpers as fitted (60 Hz). }
  SetElements('JP11', [1, 2], EL_LINK);
  SetElements('JP12', [1, 2], EL_LINK);
  SetElements('JP0', [], EL_ANALOG);
  SetElements('JP10', [], EL_ANALOG);
  SetElements('JP13', [], EL_ANALOG);
  SetElements('JP-User1', [], EL_ANALOG);
  { Buses for the bus bitmap. }
  prefixes[0] := 'A'; counts[0] := 16;
  prefixes[1] := 'D'; counts[1] := 8;
  prefixes[2] := 'CPU_D'; counts[2] := 8;
  prefixes[3] := 'RA'; counts[3] := 6;
  bit := 0;
  for k := 0 to 3 do begin
    n := counts[k];
    if (bit < 32) and (bit + n > 32) then bit := 32;
    for i := 0 to n - 1 do begin
      s.SetBusBit(s.FindNet(prefixes[k] + IntToStr(i)), bit);
      Inc(bit);
    end;
  end;
  raw := NewBytes(A2_RAW_W * A2_RAW_H);
  InstallRoms;
  pin_reset := PinOf('A7', 3);
  pin_reset_gnd := PinOf('A7', 8);
end;

{ 341-0036 dumps: 8 rows per character, the 5 dots in bits 5..1 (bit 5 the
  leftmost); the model takes bits 4..0 (bit 4 the leftmost) }
function CharRomFrom(const bytes: TBytes): TBytes;
var i: Integer;
begin
  Result := NewBytes(512);
  for i := 0 to 511 do Result[i] := (bytes[i] shr 1) and $1F;
end;

procedure TApple2.InstallRoms;
const
  NAMES: array[0..5] of String = ['ROM-D0', 'ROM-D8', 'ROM-E0', 'ROM-E8', 'ROM-F0', 'ROM-F8'];
var
  i, k: Integer;
  img: TBytes;
  p: TPart;
begin
  if rom_main = nil then begin
    rom_main := ROM_APPLE2P_MAIN;      { built in (hwsim.roms), if it was there at generation }
    rom_name := 'Apple II+ ROM';
    if rom_main.Length <> 12288 then begin rom_main := TestMainRom; rom_name := 'test ROM'; end;
  end;
  if rom_char = nil then begin
    if ROM_APPLE2P_CHAR.Length >= 512 then rom_char := CharRomFrom(ROM_APPLE2P_CHAR)
    else rom_char := TestCharRom;
  end;
  for i := 0 to 5 do begin
    p := Find(NAMES[i]);
    if p = nil then continue;
    img := NewBytes(2048);
    for k := 0 to 2047 do img[k] := rom_main[i * 2048 + k];
    TRom2316State(p.state).data := img;
  end;
  p := Find('A5');
  if p <> nil then TRom2316State(p.state).data := rom_char;
end;

procedure TApple2.MachinePowerOn;
var p: TPart;
begin
  cpu_pins := cpu.Init(False, nil, nil, Self, 0, 0);
  in_reset := True;
  mode := A2_MODE_NONE;          { Run picks it, reading the RAM chips }
  healthy_epoch := -1;
  soft := 0; kbd_val := 0; kbd_strobe := 0; cpu_acc := 0; frame_acc := 0; data_latch := 0;
  osc := 0;
  phi0 := 0;
  last_edge := 0;
  vx := 0; vy := 0; sync_len := 0; last_sync := 1;
  if raw <> nil then raw.fill(0);
  kbd_queue.Clear;
  strobe_left := 0;
  p := sim.parts[p_reset555];
  { the 555 holds /RESET low for ~0.2 s after power on }
  if p.state <> nil then TTimer555State(p.state).until_cycle := 0.2 * A2_STEPS_HZ;
end;

{ ------------------------------------------------------------------------ }
{ Keyboard, video, files                                                    }
{ ------------------------------------------------------------------------ }

procedure TApple2.KeyDown(key: Integer);
begin
  if key > 0 then kbd_queue.Add(key and $7F);
end;

procedure TApple2.KeyUp(key: Integer);
begin
end;

procedure TApple2.FrameUpdate(us: Integer);
begin
end;

function TApple2.HostKey(const code: String): Integer;
begin
  if code = 'Enter' then exit(13);
  if code = 'Backspace' then exit(8);
  if code = 'ArrowLeft' then exit(8);
  if code = 'ArrowRight' then exit(21);
  if code = 'Escape' then exit(27);
  Result := 0;
end;

function TApple2.HostChar(c: Integer): Integer;
begin
  if (c >= Ord('a')) and (c <= Ord('z')) then exit(c - 32);
  Result := c;
end;

{ The monitor: a window of the raw picture (14M periods x lines), every
  line drawn twice. White on black: a monochrome monitor. }
const
  VIEW_X = 182;
  VIEW_Y = 30;

procedure TApple2.Monitor(outp: TUInts);
var
  x, y, v, c: Integer;
begin
  for y := 0 to A2_SCREEN_H - 1 do
    for x := 0 to A2_SCREEN_W - 1 do begin
      v := 0;
      if (VIEW_Y + (y shr 1) < A2_RAW_H) and (VIEW_X + x < A2_RAW_W) then
        v := raw[(VIEW_Y + (y shr 1)) * A2_RAW_W + VIEW_X + x];
      if v <> 0 then c := U32($FF60FF60) else c := U32($FF000000);
      outp[y * A2_SCREEN_W + x] := c;
    end;
end;

function TApple2.FileKind(const path: String; const head: TBytes; var info: String; var bad: Boolean): TFileKind;
var
  n: Integer;
  lp: String;
begin
  lp := LowerCase(path);
  if (Pos('.rom', lp) = 0) and (Pos('.bin', lp) = 0) then exit(nil);
  if head <> nil then n := head.length else n := 0;
  bad := False;
  if (n = 12288) or (n = 16384) or (n = 20480) then info := 'Apple II+ ROM set'
  else if (n = 512) or (n = 2048) then info := 'character ROM'
  else begin info := IntToStr(n) + ' bytes: not an Apple II+ ROM'; bad := True; end;
  Result := TFileKind.Create('ROM', 'Install the ROMs', False, ROLE_ROM);
end;

function TApple2.Load(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean;
var
  n, i, base: Integer;
begin
  n := bytes.length;
  if (n = 12288) or (n = 16384) or (n = 20480) then begin
    base := n - 12288;                     { the last 12K is $D000-$FFFF }
    rom_main := NewBytes(12288);
    for i := 0 to 12287 do rom_main[i] := bytes[base + i];
    rom_name := path;
  end else if (n = 512) or (n = 2048) then begin
    rom_char := CharRomFrom(bytes);
  end else begin
    msg := path + ': ' + IntToStr(n) + ' bytes, not an Apple II+ ROM';
    exit(False);
  end;
  InstallRoms;
  PowerOn;
  msg := path + ' installed, switched on again';
  Result := True;
end;

function TApple2.CpuStatus: String;
begin
  if not cpu_running then Result := 'stopped (no clock)'
  else if in_reset then Result := 'in reset'
  else if Fast then Result := 'running (fast mode)'
  else Result := 'running';
end;

function TApple2.CpuRegs: String;
var st: String;
begin
  if not cpu_running then st := '(no clock)' else if in_reset then st := '(in reset)' else st := '';
  Result := 'PC=' + UpperCase(HexW(cpu.PC)) + ' A=' + UpperCase(HexB(cpu.A)) + ' X=' + UpperCase(HexB(cpu.X)) +
    ' Y=' + UpperCase(HexB(cpu.Y)) + ' S=' + UpperCase(HexB(cpu.S)) + ' ' + st;
end;

{ The raw picture as text art (for --text): one character per 7 dots of a
  line, '#' where any dot is lit, every 8th line. }
function TApple2.ScreenText: array of String;
var
  y, x, k, lit: Integer;
  line: String;
begin
  y := 0;
  while y < A2_RAW_H do begin
    line := '';
    x := 0;
    while x + 7 <= A2_RAW_W do begin
      lit := 0;
      for k := 0 to 6 do lit := lit or raw[y * A2_RAW_W + x + k];
      if lit <> 0 then line := line + '#' else line := line + ' ';
      x := x + 7;
    end;
    Result.Add(TrimRight(line));
    y := y + 2;
  end;
end;

initialization
  PT_6502 := TPartType.Create('6502 CPU',
    'VSS:P RDY:m PHI1:O /IRQ:m NC:O /NMI:m SYNC:O VCC:P A0:O A1:O A2:O A3:O A4:O A5:O A6:O A7:O ' +
    'A8:O A9:O A10:O A11:O VSS:P A12:O A13:O A14:O A15:O D7:B D6:B D5:B D4:B D3:B D2:B D1:B D0:B ' +
    'R/W:O NC:O NC:O PHI0:M SO:m PHI2:O /RES:m', cpu_eval, nil, nil, nil, TECH_NMOS);
end.
