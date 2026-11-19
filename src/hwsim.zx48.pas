unit hwsim.zx48;

{ The Sinclair ZX Spectrum 48K, board Issue 3B (port of zx48.h / zx48.c).

  HOW TIME MOVES

  Everything on this board is timed by the ULA: its 14 MHz oscillator (X1)
  is the board clock. Every half period of it is simulated (8 per CPU T
  state): the ULA advances, the board settles, the Z80 sees an edge of its
  clock if there was one (the ULA makes the clock, and stops it to keep the
  CPU off the lower RAM while it reads the screen), then the memories see
  their strobes (a commit) and the board settles again.

  The CPU and the ULA share the lower 16K (eight 4116 DRAMs) through 470 ohm
  resistors on the data bus and 330 ohm resistors after the 74LS157 address
  multiplexers; the upper 32K (eight 4532) has its own glue (IC23-IC26) with
  two RC delays timing its RAS and CAS; the ROM is selected by the ULA. The
  keyboard half rows are the address lines A8-A15, through D1-D8.

  FAST MODE

  While the board is healthy (no fault; keys and RESET are fine) and no
  pin is on the scope, the machine runs functionally: the same Z80 core on
  a flat 48K RAM, and the ULA reduced to the moments its CPU clock can
  change, with the contention gates of ula_step; the picture is drawn a
  line at a time from RAM. On a fault the RAM goes into the DRAM cells and
  the board takes over (and back when repaired).

  Board: liveboxandy's KiCad Issue 3B (CC0, from PABB's gerbers). }

interface

uses
  qtx.sysutils,
  hwsim.util, hwsim.boarddata, hwsim.bd_zx48, hwsim.roms, hwsim.sim, hwsim.parts, hwsim.board,
  hwsim.z80, hwsim.kbd, hwsim.ula;

const
  ZX_FREQ = 3500000;
  ZX_CYCLES_PER_FRAME = 224 * 312;
  ZX_STEPS_PER_CYCLE = 8;
  ZX_MODE_NONE = 0;
  ZX_MODE_PINS = 1;
  ZX_MODE_FAST = 2;

type
  TZXCart = class
  public
    name: String;
    rom: TBytes;
    pin_romcs, pin_mreq, pin_rd: Integer;
    pin_a: array[0..15] of Integer;
    pin_d: array[0..7] of Integer;
  end;

  TZX48 = class(TBoard)
  private
    FCellsLo, FCellsHi: TInts;
    function Find(const ref: String): TPart;
    function PinOf(const ref: String; number: Integer): Integer;
    procedure MakeElement(const ref: String; pin_a, pin_b, typ: Integer);
    procedure SetupEdge;
    procedure InitKeymap;
    function Broken(const ref: String): Boolean;
    procedure Power;
    procedure ZxStep;
    procedure ResetTick;
    procedure CpuEdge(clk: Integer);
    function AddrBitOfNet(net: Integer): Integer;
    function DramCell(part, addr: Integer): Integer;
    procedure CpuStart(pc: Integer);
    function LoadSna(const d: TBytes; var msg: String): Boolean;
    function LoadZ80(const d: TBytes; var msg: String): Boolean;
    function EdgePin(const net: String): Integer;
    procedure CartPins(c: TZXCart; plugged: Boolean);
    function PlugRom(const path: String; const d: TBytes; var msg: String): Boolean;
    function Healthy: Boolean;
    procedure MakeCells;
    procedure ChipsToFast;
    procedure FastToChips;
    function FastPortIn(addr: Integer): Integer;
    function FastMemRead(addr: Integer): Integer;
    procedure FastEdge(level: Integer; irq: Boolean);
    procedure FastLine(v: Integer);
    procedure RunFast(cycles: Integer);
  protected
    procedure Describe; override;
    procedure MachineCreate; override;
    procedure MachinePowerOn; override;
    procedure MachineJoins; override;
  public
    cpu: TZ80;
    cpu_pins, cpu_ab: Integer;
    p_cpu, p_ula, p_rom: Integer;
    pin_clk: Integer;
    clk: Integer;
    in_reset, cpu_running: Boolean;
    last_edge: Float;
    cache_addr, cache_data: Integer;
    changes_seen: Integer;
    lram_part, uram_part: array[0..7] of Integer;
    kbd: TKbd;
    key_matrix: array[0..4] of Integer;
    pin_kb_row: array[0..7] of Integer;
    pin_kb_line: array[0..4] of Integer;
    pin_9v, pin_12v, pin_m5v: Integer;
    cart: TZXCart;
    pin_reset_cap: Integer;
    reset_until: Float;
    monitor_seed: Integer;
    { fast mode }
    mode: Integer;
    fram: TBytes;              { $4000-$FFFF }
    romdata: TBytes;
    fclk, fdata: Integer;
    freset: Boolean;
    bt_cur, bt_prev: TInts;    { the border at every tick of this and the last line }
    healthy_epoch: Integer;
    healthy_cache: Boolean;

    constructor Create; override;
    procedure Run(cycles: Integer); override;
    function Ula: TUlaState;
    procedure DramPinBits(pin: Integer; var row, col: Integer);
    procedure RamPoke(addr, v: Integer);
    function RamPeek(addr: Integer): Integer;
    procedure SyncKeys;

    procedure KeyDown(key: Integer); override;
    procedure KeyUp(key: Integer); override;
    procedure FrameUpdate(us: Integer); override;
    function HostKey(const code: String): Integer; override;
    function HostChar(c: Integer): Integer; override;
    procedure Monitor(outp: TUInts); override;
    function FileKind(const path: String; const head: TBytes; var info: String; var bad: Boolean): TFileKind; override;
    function Load(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean; override;
    function CartName: String; override;
    procedure RemoveCart; override;
    function CpuStatus: String; override;
    function Fast: Boolean;
    function CpuRegs: String; override;
    function ScreenText: array of String; override;
  end;

var
  PT_Z80: TPartType;

implementation

const
  Z80_ADDR_PINS: array[0..15] of Integer = [30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 1, 2, 3, 4, 5];
  Z80_DATA_PINS: array[0..7] of Integer = [14, 15, 12, 8, 7, 9, 10, 13];
  Z_CLK = 6; Z_INT = 16; Z_NMI = 17; Z_HALT = 18; Z_MREQ = 19; Z_IORQ = 20; Z_RD = 21; Z_WR = 22;
  Z_BUSAK = 23; Z_WAIT = 24; Z_BUSRQ = 25; Z_RESET = 26; Z_M1 = 27; Z_RFSH = 28;
  GK_Z80_DATA = GK_MACHINE + 20;
  DRAM_ADDR_PINS_ZX: array[0..7] of Integer = [5, 7, 6, 12, 11, 10, 13, 9];

{ ------------------------------------------------------------------------ }
{ Z80 (IC2)                                                                 }
{ ------------------------------------------------------------------------ }

procedure cpu_eval(s: TSim; p: TPart);
var
  z: TZX48;
  pins: Integer;
  aoe: Boolean;
begin
  z := TZX48(p.user);
  pins := z.cpu_pins;
  aoe := (pins and Z80_AOE) <> 0;
  z.cache_addr := part_drive_bus_cached(s, p, Z80_ADDR_PINS, 16, z.cpu_ab, aoe, z.cache_addr);
  if aoe then begin
    s.PinOut(p, Z_MREQ, DRV_BIT((pins and Z80_MREQ) = 0));
    s.PinOut(p, Z_IORQ, DRV_BIT((pins and Z80_IORQ) = 0));
    s.PinOut(p, Z_RD, DRV_BIT((pins and Z80_RD) = 0));
    s.PinOut(p, Z_WR, DRV_BIT((pins and Z80_WR) = 0));
  end else begin
    s.PinOut(p, Z_MREQ, DRV_Z);
    s.PinOut(p, Z_IORQ, DRV_Z);
    s.PinOut(p, Z_RD, DRV_Z);
    s.PinOut(p, Z_WR, DRV_Z);
  end;
  s.PinOut(p, Z_M1, DRV_BIT((pins and Z80_M1) = 0));
  s.PinOut(p, Z_RFSH, DRV_BIT((pins and Z80_RFSH) = 0));
  s.PinOut(p, Z_HALT, DRV_BIT((pins and Z80_HALT) = 0));
  s.PinOut(p, Z_BUSAK, DRV_BIT((pins and Z80_BUSAK) = 0));
  z.cache_data := part_drive_bus_cached(s, p, Z80_DATA_PINS, 8, (pins shr 16) and $FF,
    (pins and Z80_DOE) <> 0, z.cache_data);
end;

procedure TZX48.CpuEdge(clk: Integer);
var
  s: TSim;
  p: TPart;
  pins, outp: Integer;
begin
  s := sim;
  p := s.parts[p_cpu];
  last_edge := s.cycle;
  if p.dead then exit;
  if clk <> 0 then pins := Z80_CLK else pins := 0;
  pins := pins or (part_read_bus(s, p, GK_Z80_DATA, Z80_DATA_PINS, 8) shl 16);
  if s.PinIn(p, Z_INT) = 0 then pins := pins or Z80_INT;
  if s.PinIn(p, Z_NMI) = 0 then pins := pins or Z80_NMI;
  if s.PinIn(p, Z_WAIT) = 0 then pins := pins or Z80_WAIT;
  if s.PinIn(p, Z_RESET) = 0 then pins := pins or Z80_RES;
  if s.PinIn(p, Z_BUSRQ) = 0 then pins := pins or Z80_BUSRQ;
  in_reset := (pins and Z80_RES) <> 0;
  outp := cpu.Tick(pins);
  if (outp <> cpu_pins) or (cpu.ab <> cpu_ab) then begin
    cpu_pins := outp;
    cpu_ab := cpu.ab;
    s.MarkDirty(p_cpu);
  end;
end;

{ ------------------------------------------------------------------------ }
{ Time                                                                      }
{ ------------------------------------------------------------------------ }

function TZX48.Ula: TUlaState;
begin
  Result := TUlaState(sim.parts[p_ula].state);
end;

procedure TZX48.ZxStep;
var
  s: TSim;
  c: Integer;
begin
  s := sim;
  ula_step(s, s.parts[p_ula]);
  s.Settle;
  c := s.Read(pin_clk);
  if c <> clk then begin
    clk := c;
    CpuEdge(c);
    s.Settle;
  end;
  { The memories and delays only act on edges. }
  if s.changes <> changes_seen then begin
    s.Commit;
    s.Settle;
    changes_seen := s.changes;
  end;
  Sample;
end;

procedure TZX48.ResetTick;
begin
  if pin_reset_cap < 0 then exit;
  if sim.cycle < reset_until then sim.Drive(pin_reset_cap, DRV_LO)
  else sim.Drive(pin_reset_cap, DRV_Z);
end;

procedure TZX48.Run(cycles: Integer);
var
  s: TSim;
  i, k, want: Integer;
begin
  s := sim;
  if (not force_pins) and (scope_pin < 0) and Healthy then want := ZX_MODE_FAST else want := ZX_MODE_PINS;
  if want <> mode then begin
    if want = ZX_MODE_FAST then ChipsToFast
    else if mode = ZX_MODE_FAST then FastToChips;
    mode := want;
  end;
  if mode = ZX_MODE_FAST then begin RunFast(cycles); exit; end;
  for i := 0 to cycles - 1 do begin
    s.cycle := s.cycle + 1;
    ResetTick;
    for k := 0 to ZX_STEPS_PER_CYCLE - 1 do ZxStep;
    cpu_running := s.cycle - last_edge < 1000;
  end;
end;

{ ------------------------------------------------------------------------ }
{ Fast mode                                                                 }
{ ------------------------------------------------------------------------ }

function TZX48.Fast: Boolean;
begin
  Result := mode = ZX_MODE_FAST;
end;

{ Healthy enough for fast mode: no pin fault, nothing removed or dead, no
  resistor fault. (Pressed keys and RESET are fine: fast mode has them.) }
function TZX48.Healthy: Boolean;
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

{ The address -> cell tables, from the wiring of one chip of each half
  (DramCell traces the multiplexers for every bit: do that once). }
procedure CellTable(z: TZX48; part, base, n: Integer; t: TInts);
var
  p: TPart;
  bits, i, a, rb, cb, row, col: Integer;
  rbs, cbs: array[0..7] of Integer;
begin
  p := z.sim.parts[part];
  if p.name = '4116' then bits := 7 else bits := 8;
  for i := 0 to bits - 1 do begin
    z.DramPinBits(p.Pin(DRAM_ADDR_PINS_ZX[i]), rb, cb);
    rbs[i] := rb;
    cbs[i] := cb;
  end;
  for a := 0 to n - 1 do begin
    row := 0;
    col := 0;
    for i := 0 to bits - 1 do begin
      if rbs[i] >= 0 then row := row or ((((base + a) shr rbs[i]) and 1) shl i)
      else if rbs[i] = -3 then row := row or (1 shl i);
      if cbs[i] >= 0 then col := col or ((((base + a) shr cbs[i]) and 1) shl i)
      else if cbs[i] = -3 then col := col or (1 shl i);
    end;
    t[a] := (row shl 8) or col;
  end;
end;

procedure TZX48.MakeCells;
begin
  if FCellsLo <> nil then exit;
  FCellsLo := NewInts(16384);
  CellTable(Self, lram_part[0], $4000, 16384, FCellsLo);
  FCellsHi := NewInts(32768);
  CellTable(Self, uram_part[0], $8000, 32768, FCellsHi);
end;

procedure TZX48.ChipsToFast;
var
  a, bit: Integer;
  cells: TBytes;
begin
  MakeCells;
  if fram = nil then fram := NewBytes($C000);
  fram.fill(0);
  for bit := 0 to 7 do begin
    cells := TDramState(sim.parts[lram_part[bit]].state).cells;
    for a := 0 to 16383 do
      if cells[FCellsLo[a]] <> 0 then fram[a] := fram[a] or (1 shl bit);
    cells := TDramState(sim.parts[uram_part[bit]].state).cells;
    for a := 0 to 32767 do
      if cells[FCellsHi[a]] <> 0 then fram[$4000 + a] := fram[$4000 + a] or (1 shl bit);
  end;
  fclk := clk;
  if bt_cur = nil then begin bt_cur := NewInts(ULA_TICKS_PER_LINE); bt_prev := NewInts(ULA_TICKS_PER_LINE); end;
  bt_cur.fill(Ula.border);
  bt_prev.fill(Ula.border);
end;

procedure TZX48.FastToChips;
var
  a, bit: Integer;
  cells: TBytes;
  u: TUlaState;
begin
  MakeCells;
  for bit := 0 to 7 do begin
    cells := TDramState(sim.parts[lram_part[bit]].state).cells;
    for a := 0 to 16383 do cells[FCellsLo[a]] := (fram[a] shr bit) and 1;
    cells := TDramState(sim.parts[uram_part[bit]].state).cells;
    for a := 0 to 32767 do cells[FCellsHi[a]] := (fram[$4000 + a] shr bit) and 1;
    sim.MarkDirty(lram_part[bit]);
    sim.MarkDirty(uram_part[bit]);
  end;
  { The ULA goes on from where fast mode left it: the half period after the
    last one acted on (h0 of an odd tick, h1 of an even one). }
  u := Ula;
  if (u.c and 1) <> 0 then u.half := 0 else u.half := 1;
  u.cpuclk := fclk <> 0;
  clk := fclk;
  sim.MarkDirty(p_ula);
  sim.MarkDirty(p_cpu);
end;

{ Port 0xFE (A0 low): keyboard half rows A8-A15 on D0-D4, EAR on D6. }
function TZX48.FastPortIn(addr: Integer): Integer;
var
  l, c, lowbits: Integer;
  u: TUlaState;
begin
  if (addr and 1) <> 0 then exit($FF);          { nothing else on the bus }
  lowbits := 0;
  for l := 0 to 4 do
    for c := 0 to 7 do
      if (((key_matrix[l] shr c) and 1) <> 0) and (((addr shr (8 + c)) and 1) = 0) then
        lowbits := lowbits or (1 shl l);
  u := Ula;
  if (not u.ear_in) and (not u.speaker) then lowbits := lowbits or $40;
  Result := $FF and not lowbits;
end;

function TZX48.FastMemRead(addr: Integer): Integer;
begin
  if addr >= $4000 then exit(fram[addr - $4000]);
  if cart <> nil then exit(cart.rom[addr]);
  Result := romdata[addr];
end;

{ One edge of the CPU clock: the board answers the bus cycle the CPU put out
  at the last edge, then the CPU ticks. }
procedure TZX48.FastEdge(level: Integer; irq: Boolean);
var
  pins, inp, a: Integer;
  u: TUlaState;
begin
  pins := cpu_pins;
  a := cpu_ab;
  if (pins and Z80_MREQ) <> 0 then begin
    if (pins and Z80_RD) <> 0 then fdata := FastMemRead(a)
    else if ((pins and Z80_WR) <> 0) and (a >= $4000) then fram[a - $4000] := (pins shr 16) and $FF;
  end else if (pins and Z80_IORQ) <> 0 then begin
    if (pins and Z80_M1) <> 0 then fdata := $FF                     { interrupt acknowledge }
    else if (pins and Z80_RD) <> 0 then fdata := FastPortIn(a)
    else if ((pins and Z80_WR) <> 0) and ((a and 1) = 0) then begin
      u := Ula;
      u.border := (pins shr 16) and 7;
      u.mic := ((pins shr 19) and 1) <> 0;
      u.speaker := ((pins shr 20) and 1) <> 0;
    end;
  end;
  if (pins and Z80_DOE) <> 0 then fdata := (pins shr 16) and $FF;
  if level <> 0 then inp := Z80_CLK else inp := 0;
  inp := inp or (fdata shl 16);
  if irq then inp := inp or Z80_INT;
  if freset then inp := inp or Z80_RES;
  in_reset := freset;
  cpu_pins := cpu.Tick(inp);
  cpu_ab := cpu.ab;
end;

{ The pixels of line v as put_pixel draws them, from the RAM at the end of
  the line and the border as it was at every tick. }
procedure TZX48.FastLine(v: Integer);
var
  u: TUlaState;
  x, y, c, px, k, a, b, t, pix, bw, base: Integer;
  ink: Boolean;
begin
  u := Ula;
  if u.fb = nil then exit;
  if v < 248 then y := v + 48 else y := v - 264;
  if (y < 0) or (y >= ULA_SCREEN_H) then exit;
  for x := 0 to ULA_SCREEN_W - 1 do begin
    c := (x + ULA_TICKS_PER_LINE - 32) mod ULA_TICKS_PER_LINE;
    if (v < 192) and (c >= 16) and (c < 272) then begin
      px := c - 16;
      k := px shr 3;
      base := ((v and $C0) shl 5) or ((v and 7) shl 8) or ((v and $38) shl 2) or k;
      b := fram[base];
      a := fram[$1800 or ((v and $F8) shl 2) or k];
      ink := ((b shr (7 - (px and 7))) and 1) <> 0;
      if ((a and $80) <> 0) and (((u.frames shr 4) and 1) <> 0) then ink := not ink;
      if ink then pix := (a and 7) or ((a and $40) shr 3)
      else pix := ((a shr 3) and 7) or ((a and $40) shr 3);
    end else begin
      { border_shown takes border_ao at c and 7 = 0, which took the border
        at c and 7 = 5 }
      t := c - (c and 7) - 3;
      if t >= 0 then bw := bt_cur[t] else bw := bt_prev[t + ULA_TICKS_PER_LINE];
      pix := bw;
    end;
    u.fb[y * ULA_SCREEN_W + x] := ula_color(pix and 7, (pix and 8) <> 0);
  end;
end;

{ The ULA, functionally, at the moments its clock output can change:
  the rising edge at half 0 of an odd tick, the falling one at half 1 of an
  even tick unless contention holds it high (the gates of ula_step). }
procedure TZX48.RunFast(cycles: Integer);
var
  s: TSim;
  n, c: Integer;
  u: TUlaState;
  screen, c23, iorq, mreq, a14, a15, w362, mem, io, irq: Boolean;
  tmp: TInts;
begin
  s := sim;
  u := Ula;
  for n := 1 to cycles * 2 do begin
    { time: a T state is two ticks }
    if (n and 1) <> 0 then begin
      s.cycle := s.cycle + 1;
      freset := reset_button or (s.cycle < reset_until);
    end;
    Inc(u.c);
    if u.c = ULA_TICKS_PER_LINE then begin
      FastLine(u.v);
      tmp := bt_prev; bt_prev := bt_cur; bt_cur := tmp;
      u.c := 0;
      Inc(u.v);
      if u.v = ULA_LINES then begin
        u.v := 0;
        u.frames := u.frames + 1;
      end;
    end;
    c := u.c;
    irq := (u.v = 248) and (c < 64);
    if (c and 1) <> 0 then begin
      if fclk = 0 then begin fclk := 1; FastEdge(1, irq); end;
    end else if fclk <> 0 then begin
      screen := (c < 256) and (u.v < 192);
      c23 := (c and 12) <> 0;
      mreq := (cpu_pins and Z80_MREQ) <> 0;
      iorq := (cpu_pins and Z80_IORQ) <> 0;
      a14 := (cpu_ab and $4000) <> 0;
      a15 := (cpu_ab and $8000) <> 0;
      w362 := (not u.ioreqt2) and screen and (not u.mreqt2);
      mem := (a14 or iorq) and (iorq or not a15) and c23 and w362;
      io := screen and iorq and c23 and not u.ioreqt2;
      if not (mem or io) then begin
        fclk := 0;
        FastEdge(0, irq);
        u.mreqt2 := (cpu_pins and Z80_MREQ) <> 0;
        u.ioreqt2 := (cpu_pins and Z80_IORQ) <> 0;
      end;
    end;
    bt_cur[c] := u.border;
  end;
  last_edge := s.cycle;
  cpu_running := True;
end;

{ ------------------------------------------------------------------------ }
{ The board                                                                 }
{ ------------------------------------------------------------------------ }

function zx_rail_of(const name: String): Integer;
begin
  if name = 'GND' then exit(RAIL_GND);
  if name = '+5V' then exit(RAIL_VCC);
  Result := RAIL_NONE;
end;

function zx_kind_of(const ref: String): Integer;
var c0, c1: String;
begin
  c0 := Copy(ref, 1, 1);
  c1 := Copy(ref, 2, 1);
  if Copy(ref, 1, 2) = 'IC' then exit(PART_IC);
  if Copy(ref, 1, 2) = 'TV' then exit(PART_CONNECTOR);
  if Copy(ref, 1, 2) = 'TR' then exit(PART_OTHER);
  if (c0 = 'R') and (c1 >= '0') and (c1 <= '9') and (c1 <> '') then exit(PART_RESISTOR);
  if (Copy(ref, 1, 4) = 'JROM') or (Copy(ref, 1, 2) = 'TI') or (Copy(ref, 1, 3) = 'OKI') then exit(PART_LINK);
  if (Copy(ref, 1, 2) = 'KB') or (Copy(ref, 1, 3) = 'EAR') or (Copy(ref, 1, 3) = 'MIC') or (ref = 'E1') or
     (c0 = 'J') then exit(PART_CONNECTOR);
  Result := PART_OTHER;
end;

constructor TZX48.Create;
begin
  inherited Create;
  cpu := TZ80.Create;
  kbd := TKbd.Create;
  monitor_seed := 12345;
end;

procedure TZX48.Describe;
var i: Integer;
begin
  id := 'zx48';
  name := 'ZX Spectrum 48K';
  board_name := 'ZX Spectrum 48K Issue 3B';
  data := BoardZX48;
  simdesc.rail_of := zx_rail_of;
  simdesc.kind_of := zx_kind_of;
  simdesc.cycles_per_us := 3.5;
  cpu_hz := ZX_FREQ;
  cycles_per_frame := ZX_CYCLES_PER_FRAME;
  steps_per_cycle := ZX_STEPS_PER_CYCLE;
  screen_w := ULA_SCREEN_W;
  screen_h := ULA_SCREEN_H;
  ready_cycles := 3 * ZX_FREQ;
  link_resistors := '';
  crystals := 'X1 REG1 TR4 TR5 T1 D11 D12 D15 D16 R55 R58 R59 R60 R61';
  keys_help := 'Keys: ESC=BREAK  TAB=SYMBOL SHIFT  left SHIFT=CAPS SHIFT';
  reset_help := 'Hold to press RESET: shorts /RESET to GND on the edge connector';
  files_title := 'Open a snapshot (.sna, .z80) or a ROM (.rom)';
  cart_slot := 'E1';
  part_defs.Add(TPartDef.Create('IC1', PT_ULA, 'ULA', ROLE_CHIP, '6C001'#10'video, I/O',
    'Ferranti ULA 6C001: video, CPU clock and contention, lower RAM control, port 0xFE'));
  part_defs.Add(TPartDef.Create('IC2', PT_Z80, 'CPU', ROLE_CHIP, 'Z80A', 'Z80A CPU, 3.5 MHz'));
  part_defs.Add(TPartDef.Create('IC5', PT_23128, 'ROM', ROLE_ROM, '23128'#10'$0000-$3FFF', '23128 ROM, $0000-$3FFF'));
  for i := 0 to 7 do
    part_defs.Add(TPartDef.Create('IC' + IntToStr(6 + i), PT_4116, 'LRAM' + IntToStr(i), ROLE_VRAM,
      '4116'#10'bit D' + IntToStr(i) + #10'$4000-$7FFF',
      '4116 DRAM, lower RAM: data bit D' + IntToStr(i) + ' of $4000-$7FFF (the screen is here)'));
  for i := 0 to 7 do
    part_defs.Add(TPartDef.Create('IC' + IntToStr(15 + i), PT_4532, 'URAM' + IntToStr(i), ROLE_RAM,
      '4532'#10'bit D' + IntToStr(i) + #10'$8000-$FFFF',
      '4532 DRAM, upper RAM: data bit D' + IntToStr(i) + ' of $8000-$FFFF'));
  part_defs.Add(TPartDef.Create('IC3', PT_74LS157, 'MUX', ROLE_GLUE, '74LS157',
    '74LS157 lower RAM address mux: CPU A0-A3 (row), A7-A10 (column), switched by /RAS'));
  part_defs.Add(TPartDef.Create('IC4', PT_74LS157, 'MUX', ROLE_GLUE, '74LS157',
    '74LS157 lower RAM address mux: CPU A4-A6 (row), A11-A13 (column), switched by /RAS'));
  part_defs.Add(TPartDef.Create('IC25', PT_74LS157, 'MUX', ROLE_GLUE, '74LS157', '74LS157 upper RAM address mux (row, column)'));
  part_defs.Add(TPartDef.Create('IC26', PT_74LS157, 'MUX', ROLE_GLUE, '74LS157',
    '74LS157 upper RAM address mux (row, column; A7 set by the TI/OKI links)'));
  part_defs.Add(TPartDef.Create('IC23', PT_74LS32, 'OR', ROLE_GLUE, '74LS32', '74LS32 OR gates: upper RAM /RAS, /CAS, mux select, /WRITE'));
  part_defs.Add(TPartDef.Create('IC24', PT_74LS00, 'NAND', ROLE_GLUE, '74LS00', '74LS00 NAND gates: upper RAM select (A15 and RD or WR)'));
  part_defs.Add(TPartDef.Create('IC14', PT_LM1889, 'PAL', ROLE_OTHER, 'LM1889'#10'not'#10'simulated',
    'LM1889 PAL video modulator (analog, not simulated: the monitor shows the ULA''s picture)'));
  conn_defs.Add(TConnDef.Create('KB1', 'KEYBOARD LINES', '1=KB0 2=KB1 3=KB2 4=KB3 5=KB4'));
  conn_defs.Add(TConnDef.Create('KB2', 'KEYBOARD HALF ROWS', '1=A11 2=A10 3=A9 4=A12 5=A13 6=A8 7=A14 8=A15'));
  conn_defs.Add(TConnDef.Create('EAR1', 'EAR', '1=GND 2=EAR_IN'));
  conn_defs.Add(TConnDef.Create('MIC1', 'MIC', '1=GND 2=MIC_OUT'));
  conn_defs.Add(TConnDef.Create('TV1', 'TV', '1=RF_SUPPLY 2=VIDEO 3=GND'));
  conn_defs.Add(TConnDef.Create('J7', '9V DC', '1=GND 2=+9V 3=NC'));
  conn_defs.Add(TConnDef.Create('E1', 'EDGE CONNECTOR', ''));
end;

function TZX48.Find(const ref: String): TPart;
var i: Integer;
begin
  i := sim.FindPart(ref);
  if i >= 0 then Result := sim.parts[i] else Result := nil;
end;

function TZX48.PinOf(const ref: String; number: Integer): Integer;
var p: TPart;
begin
  p := Find(ref);
  if p <> nil then Result := p.Pin(number) else Result := -1;
end;

{ Turn a two pin part into one active element (diodes, transistors). }
procedure TZX48.MakeElement(const ref: String; pin_a, pin_b, typ: Integer);
var
  p: TPart;
  st: TResistorState;
  e: TElement;
begin
  p := Find(ref);
  if (p = nil) or (p.Pin(pin_a) < 0) or (p.Pin(pin_b) < 0) then exit;
  st := TResistorState(p.state);
  if st = nil then begin
    st := TResistorState.Create;
    p.state := st;
  end;
  p.kind := PART_RESISTOR;
  e := TElement.Create;
  e.pin_a := p.Pin(pin_a);
  e.pin_b := p.Pin(pin_b);
  e.typ := EL_ANALOG;
  st.el.Clear;
  st.el.Add(e);
  st.nel := 1;
  element_set_type(sim, p, 0, typ);
end;

procedure TZX48.SetupEdge;
var
  e1: TPart;
  k, pin, net: Integer;
  n: String;
begin
  e1 := Find('E1');
  if e1 = nil then exit;
  for k := 0 to e1.npins - 1 do begin
    pin := e1.first_pin + k;
    net := sim.pins[pin].net;
    if (net < 0) or (pin_func[pin] <> '') then continue;
    n := sim.nets[net].name;
    if (Copy(n, 1, 5) <> 'Net-(') and (Copy(n, 1, 11) <> 'unconnected') then pin_func[pin] := n;
  end;
end;

procedure TZX48.InitKeymap;
const
  KEYMAP = ' zxcv' + 'asdfg' + 'qwert' + '12345' + '09876' + 'poiuy' + ' lkjh' + '  mnb' +
           ' ZXCV' + 'ASDFG' + 'QWERT' + '     ' + '     ' + 'POIUY' + ' LKJH' + '  MNB' +
           ' : ?/' + '     ' + '   <>' + '!@#$%' + '_)(''&' + '";   ' + ' =+-^' + '  .,*';
var
  layer, column, line, c, m: Integer;
begin
  kbd.Init(1);
  kbd.RegisterModifier(0, 0, 0);     { caps shift }
  kbd.RegisterModifier(1, 7, 1);     { symbol shift }
  for layer := 0 to 2 do
    for column := 0 to 7 do
      for line := 0 to 4 do begin
        c := Ord(KEYMAP[layer * 40 + column * 5 + line + 1]);
        if c <> Ord(' ') then begin
          if layer > 0 then m := 1 shl (layer - 1) else m := 0;
          kbd.RegisterKey(c, column, line, m);
        end;
      end;
  kbd.RegisterKey(Ord(' '), 7, 0, 0);
  kbd.RegisterKey($0F, 7, 1, 0);    { symbol shift }
  kbd.RegisterKey($0E, 0, 0, 0);    { caps shift }
  kbd.RegisterKey($08, 3, 4, 1);    { cursor left: caps shift + 5 }
  kbd.RegisterKey($0A, 4, 4, 1);    { down: caps shift + 6 }
  kbd.RegisterKey($0B, 4, 3, 1);    { up: caps shift + 7 }
  kbd.RegisterKey($09, 4, 2, 1);    { right: caps shift + 8 }
  kbd.RegisterKey($07, 3, 0, 1);    { edit: caps shift + 1 }
  kbd.RegisterKey($0C, 4, 0, 1);    { delete: caps shift + 0 }
  kbd.RegisterKey($03, 7, 0, 1);    { break: caps shift + space }
  kbd.RegisterKey($0D, 6, 0, 0);    { enter }
end;

const
  SERIES: array[0..18] of String = ['R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7', 'R8', 'R17', 'R18', 'R19', 'R20',
    'R21', 'R22', 'R23', 'R26', 'R27', 'R33', 'R57'];
  LINKS_OFF: array[0..3] of String = ['TI-4', 'OKI1', 'OKI-L1', 'OKI-H1'];
  DELAYS: array[0..2] of String = ['R32', 'R70', 'R71'];

procedure TZX48.MachineCreate;
var
  s: TSim;
  i, c, l, k, a, j, net, bit, n, pin: Integer;
  p, kb2, e1: TPart;
  st: TResistorState;
  e: TElement;
  nm, an: String;
  fmts: array[0..4] of String;
  counts: array[0..4] of Integer;
begin
  s := sim;
  p_cpu := s.FindPart('IC2');
  p_ula := s.FindPart('IC1');
  p_rom := s.FindPart('IC5');
  pin_clk := s.parts[p_cpu].Pin(Z_CLK);
  romdata := ROM_ZX48_ROM;
  TRomState(s.parts[p_rom].state).data := romdata;
  Ula.fb := screen;
  { Resistors and links that do logic. }
  for i := 0 to SERIES.High do begin
    p := Find(SERIES[i]);
    if (p <> nil) and (p.state <> nil) and (TResistorState(p.state).nel > 0) then element_set_type(s, p, 0, EL_SERIES);
  end;
  p := Find('R24');                  { /PHICPU to TR3's base }
  if (p <> nil) and (p.state <> nil) then TResistorState(p.state).el[0].typ := EL_LINK;
  { RC delays, from pin 2 to pin 1. }
  for i := 0 to 2 do begin
    p := Find(DELAYS[i]);
    if (p = nil) or (p.state = nil) then continue;
    e := TResistorState(p.state).el[0];
    e.pin_a := p.Pin(2);
    e.pin_b := p.Pin(1);
    element_set_type(s, p, 0, EL_DELAY);
  end;
  for i := 0 to LINKS_OFF.High do begin
    p := Find(LINKS_OFF[i]);
    if (p <> nil) and (p.state <> nil) then TResistorState(p.state).nel := 0;
  end;
  p := Find('JROM1');
  if (p <> nil) and (p.state <> nil) then begin
    st := TResistorState(p.state);
    st.el.Clear;
    e := TElement.Create; e.pin_a := p.Pin(1); e.pin_b := p.Pin(2); e.typ := EL_LINK; st.el.Add(e);
    e := TElement.Create; e.pin_a := p.Pin(3); e.pin_b := p.Pin(4); e.typ := EL_LINK; st.el.Add(e);
    st.nel := 2;
  end;
  { The keyboard diodes, TR3 (the clock inverter) and TR6 (emitter follower). }
  for i := 1 to 8 do MakeElement('D' + IntToStr(i), 2, 1, EL_DIODE);
  MakeElement('TR3', 2, 1, EL_NPN);
  MakeElement('TR6', 2, 3, EL_FOLLOWER);
  { Which DRAM holds which bit. }
  for i := 0 to s.nparts - 1 do begin
    p := s.parts[i];
    if p.Pin(2) >= 0 then net := s.pins[p.Pin(2)].net else net := -1;
    if net >= 0 then nm := s.nets[net].name else nm := '';
    if (p.name = '4532') and (Copy(nm, 1, 1) = 'D') and (Length(nm) >= 2) then
      uram_part[Ord(nm[2]) - 48] := i;
    if (p.name = '4116') and (Copy(nm, 1, 10) = 'Net-(IC1-D') then
      lram_part[Ord(nm[11]) - 48] := i;
  end;
  { Keyboard pins. }
  kb2 := Find('KB2');
  for c := 0 to 7 do begin
    pin_kb_row[c] := -1;
    an := 'A' + IntToStr(8 + c);
    for i := 1 to 8 do begin
      k := PinOf('D' + IntToStr(i), 1);
      a := PinOf('D' + IntToStr(i), 2);
      if (k < 0) or (a < 0) or (s.pins[k].net < 0) or (s.nets[s.pins[k].net].name <> an) then continue;
      if kb2 <> nil then
        for j := 0 to kb2.npins - 1 do
          if (s.pins[kb2.first_pin + j].net = s.pins[a].net) and (AtoI(s.bd.pads[kb2.first_pin + j].name) < 10) then
            pin_kb_row[c] := kb2.first_pin + j;
    end;
  end;
  for l := 0 to 4 do pin_kb_line[l] := PinOf('KB1', l + 1);
  InitKeymap;
  { Buses, for the bus bitmap (speed only); a bus never straddles a word. }
  fmts[0] := 'A%d'; counts[0] := 16;
  fmts[1] := 'D%d'; counts[1] := 8;
  fmts[2] := 'LA%d'; counts[2] := 7;
  fmts[3] := 'UA%d'; counts[3] := 8;
  fmts[4] := 'Net-(IC1-D%d)'; counts[4] := 8;
  bit := 0;
  for k := 0 to 4 do begin
    n := counts[k];
    if (bit < 32) and (bit + n > 32) then bit := 32;
    for i := 0 to n - 1 do begin
      s.SetBusBit(s.FindNet(StrReplace(fmts[k], '%d', IntToStr(i))), bit);
      Inc(bit);
    end;
  end;
  pin_9v := PinOf('J7', 2);
  pin_12v := PinOf('D15', 1);
  pin_m5v := PinOf('D12', 2);
  pin_reset_cap := PinOf('C27', 1);
  e1 := Find('E1');
  if e1 <> nil then
    for k := 0 to e1.npins - 1 do begin
      pin := e1.first_pin + k;
      net := s.pins[pin].net;
      if net < 0 then continue;
      if (s.nets[net].name = '/RESET') and (pin_reset < 0) then pin_reset := pin;
      if (s.nets[net].name = 'GND') and (pin_reset_gnd < 0) then pin_reset_gnd := pin;
    end;
  SetupEdge;
end;

procedure TZX48.MachinePowerOn;
begin
  cpu_pins := cpu.Init;
  cpu_ab := cpu.ab;
  in_reset := True;
  mode := ZX_MODE_NONE;            { Run picks it, reading the RAM chips }
  healthy_epoch := -1;
  fdata := $FF;
  clk := 0;
  last_edge := 0;
  reset_until := Trunc(ZX_FREQ * 7 / 100);
  if screen <> nil then screen.fill(0);
  sim.Drive(pin_9v, MkDrive(S_SUPPLY, 1));
end;

function TZX48.Broken(const ref: String): Boolean;
var p: TPart;
begin
  p := Find(ref);
  Result := (p <> nil) and (p.removed or (p.rfault <> RF_OK));
end;

procedure TZX48.Power;
var
  s: TSim;
  v5, osc, p12, m5: Boolean;
  i: Integer;
  p: TPart;
begin
  s := sim;
  v5 := not Broken('REG1');
  osc := v5 and not Broken('TR4') and not Broken('TR5') and not Broken('T1') and not Broken('R58') and
         not Broken('R59') and not Broken('R60') and not Broken('R61');
  p12 := osc and not Broken('D15');
  m5 := osc and not Broken('D11') and not Broken('D12') and not Broken('D16') and not Broken('R55');
  if p12 then s.Drive(pin_12v, MkDrive(S_SUPPLY, 1)) else s.Drive(pin_12v, DRV_Z);
  if m5 then s.Drive(pin_m5v, MkDrive(S_SUPPLY, 0)) else s.Drive(pin_m5v, DRV_Z);
  for i := 0 to s.nparts - 1 do begin
    p := s.parts[i];
    if p.kind <> PART_IC then continue;
    p.no_supply := (not v5) or ((p.name = '4116') and ((not p12) or (not m5)));
  end;
  Ula.running := not Broken('X1');
end;

procedure TZX48.MachineJoins;
var
  s: TSim;
  l, c, r, k: Integer;
begin
  s := sim;
  Power;
  for l := 0 to 4 do
    for c := 0 to 7 do begin
      if ((key_matrix[l] shr c) and 1) = 0 then continue;
      r := pin_kb_row[c];
      k := pin_kb_line[l];
      if (r < 0) or (k < 0) or (s.pins[r].fault = PF_OPEN) or (s.pins[k].fault = PF_OPEN) then continue;
      s.Join(s.pins[r].net, s.pins[k].net);
    end;
end;

procedure TZX48.SyncKeys;
var
  l, m: Integer;
  changed: Boolean;
begin
  changed := False;
  for l := 0 to 4 do begin
    m := kbd.scanout_column_masks[l];
    if m <> key_matrix[l] then begin
      key_matrix[l] := m;
      changed := True;
    end;
  end;
  if changed then Rebuild;
end;

procedure TZX48.KeyDown(key: Integer);
begin
  kbd.KeyDown(key);
  SyncKeys;
end;

procedure TZX48.KeyUp(key: Integer);
begin
  kbd.KeyUp(key);
  SyncKeys;
end;

procedure TZX48.FrameUpdate(us: Integer);
begin
  kbd.Update(us);
  SyncKeys;
end;

function TZX48.HostKey(const code: String): Integer;
begin
  if code = 'Enter' then exit($0D);
  if code = 'Backspace' then exit($0C);
  if code = 'ArrowLeft' then exit($08);
  if code = 'ArrowRight' then exit($09);
  if code = 'ArrowUp' then exit($0B);
  if code = 'ArrowDown' then exit($0A);
  if code = 'Escape' then exit($03);
  if code = 'Tab' then exit($0F);
  if code = 'Shift' then exit($0E);
  Result := 0;
end;

function TZX48.HostChar(c: Integer): Integer;
begin
  Result := c;
end;

procedure TZX48.Monitor(outp: TUInts);
var
  i, c, g: Integer;
  dead: Boolean;
begin
  dead := sim.parts[p_ula].dead;
  for i := 0 to ULA_SCREEN_W * ULA_SCREEN_H - 1 do begin
    c := screen[i];
    if dead then begin
      monitor_seed := U32(Imul(monitor_seed, 1103515245) + 12345);
      g := (((monitor_seed shr 24) and $FF) * 3) div 4;
      c := U32($FF000000 or (g shl 16) or (g shl 8) or g);
    end;
    outp[i] := c;
  end;
end;

{ ------------------------------------------------------------------------ }
{ Loading snapshots                                                         }
{ ------------------------------------------------------------------------ }

function TZX48.AddrBitOfNet(net: Integer): Integer;
var
  root, i: Integer;
  n: String;
begin
  root := sim.nets[net].root;
  for i := 0 to sim.nnets - 1 do begin
    if sim.nets[i].root <> root then continue;
    n := sim.nets[i].name;
    if (Length(n) >= 2) and (n[1] = 'A') and (n[2] >= '0') and (n[2] <= '9') then
      exit(AtoI(Copy(n, 2, Length(n))));
    if n = 'GND' then exit(-2);
    if n = '+5V' then exit(-3);
  end;
  Result := -1;
end;

const
  MUX_SEC_YAB: array[0..11] of Integer = [4, 2, 3, 7, 5, 6, 9, 11, 10, 12, 14, 13];

procedure TZX48.DramPinBits(pin: Integer; var row, col: Integer);
var
  s: TSim;
  net, hop, next, i, k: Integer;
  p: TPart;
  e: TElement;
begin
  s := sim;
  row := -1;
  col := -1;
  net := s.pins[pin].net;
  hop := 0;
  while (hop < 2) and (net >= 0) do begin
    next := -1;
    for i := 0 to s.npins - 1 do begin
      if s.pins[i].net <> net then continue;
      p := s.parts[s.pins[i].part];
      if p.name = '74LS157' then
        for k := 0 to 3 do
          if p.Pin(MUX_SEC_YAB[k * 3]) = i then begin
            col := AddrBitOfNet(s.pins[p.Pin(MUX_SEC_YAB[k * 3 + 1])].net);
            row := AddrBitOfNet(s.pins[p.Pin(MUX_SEC_YAB[k * 3 + 2])].net);
            exit;
          end;
      if (p.kind = PART_RESISTOR) and (p.state is TResistorState) and (TResistorState(p.state).nel > 0) then begin
        e := TResistorState(p.state).el[0];
        if e.typ = EL_SERIES then begin
          if e.pin_a = i then next := s.pins[e.pin_b].net else next := s.pins[e.pin_a].net;
        end;
      end;
    end;
    net := next;
    Inc(hop);
  end;
end;


function TZX48.DramCell(part, addr: Integer): Integer;
var
  p: TPart;
  bits, row, col, i, rb, cb, r, c: Integer;
begin
  p := sim.parts[part];
  if p.name = '4116' then bits := 7 else bits := 8;
  row := 0;
  col := 0;
  for i := 0 to bits - 1 do begin
    DramPinBits(p.Pin(DRAM_ADDR_PINS_ZX[i]), rb, cb);
    if rb >= 0 then r := (addr shr rb) and 1 else r := Ord(rb = -3);
    if cb >= 0 then c := (addr shr cb) and 1 else c := Ord(cb = -3);
    row := row or (r shl i);
    col := col or (c shl i);
  end;
  Result := (row shl 8) or col;
end;

procedure TZX48.RamPoke(addr, v: Integer);
var
  upper, a, cell, bit: Integer;
begin
  if addr < $4000 then exit;
  if addr >= $8000 then upper := 1 else upper := 0;
  MakeCells;
  if upper = 1 then cell := FCellsHi[addr - $8000] else cell := FCellsLo[addr - $4000];
  for bit := 0 to 7 do
    if upper = 1 then TDramState(sim.parts[uram_part[bit]].state).cells[cell] := (v shr bit) and 1
    else TDramState(sim.parts[lram_part[bit]].state).cells[cell] := (v shr bit) and 1;
end;

function TZX48.RamPeek(addr: Integer): Integer;
var cell, bit, part: Integer;
begin
  if addr < $4000 then exit(ROM_ZX48_ROM[addr]);
  MakeCells;
  if addr >= $8000 then cell := FCellsHi[addr - $8000] else cell := FCellsLo[addr - $4000];
  Result := 0;
  for bit := 0 to 7 do begin
    if addr >= $8000 then part := uram_part[bit] else part := lram_part[bit];
    Result := Result or (TDramState(sim.parts[part].state).cells[cell] shl bit);
  end;
end;

procedure TZX48.CpuStart(pc: Integer);
begin
  cpu_pins := cpu.Prefetch(pc);
  sim.MarkDirty(p_cpu);
end;

function LE16(const d: TBytes; o: Integer): Integer;
begin
  Result := d[o] or (d[o + 1] shl 8);
end;

function TZX48.LoadSna(const d: TBytes; var msg: String): Boolean;
var
  a, sp, pc: Integer;
  c: TZ80;
begin
  if d.length <> 27 + 49152 then begin
    msg := 'not a 48K .sna (' + IntToStr(d.length) + ' bytes)';
    exit(False);
  end;
  for a := 0 to 49151 do RamPoke($4000 + a, d[27 + a]);
  c := cpu;
  c.r8[19] := d[0];                         { i }
  c.hl2 := LE16(d, 1); c.de2 := LE16(d, 3); c.bc2 := LE16(d, 5); c.af2 := LE16(d, 7);
  c.r16[4] := LE16(d, 9); c.r16[3] := LE16(d, 11); c.r16[2] := LE16(d, 13);
  c.r16[6] := LE16(d, 15); c.r16[5] := LE16(d, 17);
  c.iff1 := (d[19] and 4) <> 0;
  c.iff2 := c.iff1;
  c.r8[18] := d[20];                        { r }
  c.r16[1] := LE16(d, 21);
  sp := LE16(d, 23);
  c.im := d[25] and 3;
  Ula.border := d[26] and 7;
  pc := RamPeek(sp) or (RamPeek((sp + 1) and $FFFF) shl 8);
  c.r16[8] := (sp + 2) and $FFFF;
  CpuStart(pc);
  Result := True;
end;

function Z80Unpack(const src: TBytes; at, n: Integer; dst: TBytes; dbase, max: Integer; v1: Boolean): Integer;
var i, o, k: Integer;
begin
  i := 0;
  o := 0;
  while (i < n) and (o < max) do begin
    if v1 and (i + 3 < n) and (src[at + i] = 0) and (src[at + i + 1] = $ED) and (src[at + i + 2] = $ED) and
       (src[at + i + 3] = 0) then break;
    if (i + 3 < n) and (src[at + i] = $ED) and (src[at + i + 1] = $ED) then begin
      k := 0;
      while (k < src[at + i + 2]) and (o < max) do begin
        dst[dbase + o] := src[at + i + 3];
        Inc(o);
        Inc(k);
      end;
      i := i + 4;
    end else begin
      dst[dbase + o] := src[at + i];
      Inc(o);
      Inc(i);
    end;
  end;
  Result := o;
end;

function TZX48.LoadZ80(const d: TBytes; var msg: String): Boolean;
var
  mem: TBytes;
  n, pc, flags, got, hlen, at, hw, pages, len, page, inl, base, a, i: Integer;
  v2: Boolean;
  c: TZ80;
begin
  n := d.length;
  if n < 30 then begin msg := 'too short for a .z80'; exit(False); end;
  mem := NewBytes(49152);
  pc := LE16(d, 6);
  if d[12] = 255 then flags := 1 else flags := d[12];
  if pc <> 0 then begin
    if (flags and $20) <> 0 then got := Z80Unpack(d, 30, n - 30, mem, 0, 49152, True)
    else begin
      if n - 30 < 49152 then got := n - 30 else got := 49152;
      for i := 0 to got - 1 do mem[i] := d[30 + i];
    end;
    if got <> 49152 then begin
      msg := '.z80: memory is ' + IntToStr(got) + ' bytes, not 48K';
      exit(False);
    end;
  end else begin
    hlen := LE16(d, 30);
    at := 32 + hlen;
    pc := LE16(d, 32);
    hw := d[34];
    v2 := hlen = 23;
    if (v2 and (hw > 1)) or ((not v2) and (hw <> 0) and (hw <> 1) and (hw <> 3)) then begin
      msg := '.z80 for another machine (hardware ' + IntToStr(hw) + '): only the 48K';
      exit(False);
    end;
    pages := 0;
    while at + 3 <= n do begin
      len := LE16(d, at);
      page := d[at + 2];
      at := at + 3;
      if len = $FFFF then inl := 16384 else inl := len;
      if at + inl > n then break;
      if page = 8 then base := 0 else if page = 4 then base := 16384 else if page = 5 then base := 32768 else base := -1;
      if base >= 0 then begin
        if len = $FFFF then for i := 0 to 16383 do mem[base + i] := d[at + i]
        else Z80Unpack(d, at, inl, mem, base, 16384, False);
        Inc(pages);
      end;
      at := at + inl;
    end;
    if pages <> 3 then begin
      msg := '.z80: ' + IntToStr(pages) + ' pages of the 48K found, not 3';
      exit(False);
    end;
  end;
  for a := 0 to 49151 do RamPoke($4000 + a, mem[a]);
  c := cpu;
  c.r8[3] := d[0]; c.r8[2] := d[1];                   { a, f }
  c.r16[2] := LE16(d, 2); c.r16[4] := LE16(d, 4); c.r16[8] := LE16(d, 8);
  c.r8[19] := d[10];                                  { i }
  c.r8[18] := (d[11] and $7F) or ((flags and 1) shl 7);  { r }
  c.r16[3] := LE16(d, 13); c.bc2 := LE16(d, 15); c.de2 := LE16(d, 17); c.hl2 := LE16(d, 19);
  c.af2 := (d[21] shl 8) or d[22];
  c.r16[6] := LE16(d, 23); c.r16[5] := LE16(d, 25);
  c.iff1 := d[27] <> 0;
  c.iff2 := d[28] <> 0;
  c.im := d[29] and 3;
  Ula.border := (flags shr 1) and 7;
  CpuStart(pc);
  Result := True;
end;

{ ------------------------------------------------------------------------ }
{ ROM cartridges                                                            }
{ ------------------------------------------------------------------------ }

procedure zxcart_eval(s: TSim; p: TPart);
var
  c: TZXCart;
  sel: Boolean;
  a, i: Integer;
begin
  c := TZXCart(p.state);
  sel := (s.Read(c.pin_mreq) = 0) and (s.Read(c.pin_rd) = 0) and (s.Read(c.pin_a[14]) = 0) and (s.Read(c.pin_a[15]) = 0);
  a := 0;
  if sel then for i := 0 to 13 do a := a or (s.Read(c.pin_a[i]) shl i);
  for i := 0 to 7 do
    if sel then s.Drive(c.pin_d[i], DRV_BIT((c.rom[a] shr i) and 1)) else s.Drive(c.pin_d[i], DRV_Z);
  s.Drive(c.pin_romcs, DRV_HI);
end;

function TZX48.EdgePin(const net: String): Integer;
var
  e1: TPart;
  k, pin: Integer;
begin
  e1 := Find('E1');
  if e1 <> nil then
    for k := 0 to e1.npins - 1 do begin
      pin := e1.first_pin + k;
      if (sim.pins[pin].net >= 0) and (sim.nets[sim.pins[pin].net].name = net) then exit(pin);
    end;
  Result := -1;
end;

procedure TZX48.CartPins(c: TZXCart; plugged: Boolean);
var
  ins, outs: array of Integer;
  i: Integer;
begin
  ins.Add(c.pin_mreq);
  ins.Add(c.pin_rd);
  for i := 0 to 15 do ins.Add(c.pin_a[i]);
  for i := 0 to ins.High do
    if ins[i] >= 0 then begin
      if plugged then sim.pins[ins[i]].flags := PIN_IN or PIN_SENSE else sim.pins[ins[i]].flags := 0;
    end;
  outs.Add(c.pin_romcs);
  for i := 0 to 7 do outs.Add(c.pin_d[i]);
  for i := 0 to outs.High do
    if outs[i] >= 0 then begin
      if plugged then sim.pins[outs[i]].flags := PIN_OUT else sim.pins[outs[i]].flags := 0;
      sim.pins[outs[i]].drive := DRV_Z;
    end;
end;

procedure TZX48.RemoveCart;
var e1: TPart;
begin
  e1 := Find('E1');
  if (cart = nil) or (e1 = nil) then exit;
  CartPins(cart, False);
  cart := nil;
  e1.state := nil;
  e1.eval := nil;
  Rebuild;
  PowerOn;
end;

function TZX48.CartName: String;
begin
  if cart <> nil then Result := cart.name else Result := '';
end;

function TZX48.PlugRom(const path: String; const d: TBytes; var msg: String): Boolean;
var
  e1: TPart;
  c: TZXCart;
  i: Integer;
  nm: String;
begin
  e1 := Find('E1');
  if (d.length <> 16384) or (e1 = nil) then begin
    msg := 'a ROM cartridge is 16384 bytes, this is ' + IntToStr(d.length);
    exit(False);
  end;
  RemoveCart;
  c := TZXCart.Create;
  nm := path;
  while Pos('/', nm) > 0 do nm := Copy(nm, Pos('/', nm) + 1, Length(nm));
  c.name := nm;
  c.rom := NewBytes(16384);
  for i := 0 to 16383 do c.rom[i] := d[i];
  c.pin_romcs := EdgePin('/ROMCS');
  c.pin_mreq := EdgePin('/MREQ');
  c.pin_rd := EdgePin('/RD');
  for i := 0 to 15 do c.pin_a[i] := EdgePin('A' + IntToStr(i));
  for i := 0 to 7 do c.pin_d[i] := EdgePin('D' + IntToStr(i));
  CartPins(c, True);
  cart := c;
  e1.state := c;
  e1.eval := zxcart_eval;
  Rebuild;
  PowerOn;
  msg := 'ROM cartridge ' + c.name + ' plugged into the edge connector';
  Result := True;
end;

function TZX48.FileKind(const path: String; const head: TBytes; var info: String; var bad: Boolean): TFileKind;
var
  lp: String;
  sna, z80, rom, v2, ok: Boolean;
  size, hlen, hw: Integer;
begin
  lp := LowerCase(path);
  sna := (Length(lp) > 4) and (Copy(lp, Length(lp) - 3, 4) = '.sna');
  z80 := (Length(lp) > 4) and (Copy(lp, Length(lp) - 3, 4) = '.z80');
  rom := (Length(lp) > 4) and (Copy(lp, Length(lp) - 3, 4) = '.rom');
  if not sna and not z80 and not rom then exit(nil);
  if head <> nil then size := head.length else size := 0;
  bad := False;
  if rom then begin
    if size = 16384 then info := '16K ROM for the edge connector'
    else begin info := IntToStr(size) + ' bytes: not a 16K ROM'; bad := True; end;
    exit(TFileKind.Create('ROM cartridge', 'Plug the ROM', False, ROLE_ROM));
  end;
  Result := TFileKind.Create('snapshot', 'Load the snapshot', False, -1);
  if sna then begin
    if size = 27 + 49152 then info := '48K snapshot'
    else begin info := IntToStr(size) + ' bytes: not a 48K snapshot'; bad := True; end;
    exit;
  end;
  if size < 35 then begin info := 'too short for a snapshot'; bad := True; exit; end;
  if (head[6] or head[7]) <> 0 then begin info := '48K snapshot (version 1)'; exit; end;
  hlen := head[30] or (head[31] shl 8);
  hw := head[34];
  v2 := hlen = 23;
  if v2 then ok := hw <= 1 else ok := (hw = 0) or (hw = 1) or (hw = 3);
  if ok then begin
    if v2 then info := '48K snapshot (version 2)' else info := '48K snapshot (version 3)';
  end else begin
    info := 'for another model (hardware ' + IntToStr(hw) + '): only the 48K';
    bad := True;
  end;
end;

function TZX48.Load(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean;
var lp: String;
begin
  lp := LowerCase(path);
  if (Length(lp) > 4) and (Copy(lp, Length(lp) - 3, 4) = '.rom') then exit(PlugRom(path, bytes, msg));
  if (Length(lp) > 4) and (Copy(lp, Length(lp) - 3, 4) = '.sna') then Result := LoadSna(bytes, msg)
  else if (Length(lp) > 4) and (Copy(lp, Length(lp) - 3, 4) = '.z80') then Result := LoadZ80(bytes, msg)
  else begin
    msg := path + ': the Spectrum takes .sna and .z80 snapshots, .rom cartridges';
    exit(False);
  end;
  if Result then msg := 'snapshot ' + path + ' loaded';
  if Result and (mode = ZX_MODE_FAST) then mode := ZX_MODE_NONE;   { fast mode reads the new RAM }
end;

function TZX48.CpuStatus: String;
begin
  if not cpu_running then Result := 'stopped (no clock)'
  else if in_reset then Result := 'in reset'
  else if Fast then Result := 'running (fast mode)'
  else Result := 'running';
end;

function TZX48.CpuRegs: String;
var st: String;
begin
  if not cpu_running then st := '(no clock)'
  else if in_reset then st := '(in reset)'
  else st := '';
  Result := 'PC=' + UpperCase(HexW(cpu.PC)) + ' AF=' + UpperCase(HexW(cpu.AF)) + ' BC=' + UpperCase(HexW(cpu.BC)) +
    ' DE=' + UpperCase(HexW(cpu.DE)) + ' HL=' + UpperCase(HexW(cpu.HL)) + ' SP=' + UpperCase(HexW(cpu.SP)) +
    ' IR=' + UpperCase(HexW(cpu.IR)) + ' IM' + IntToStr(cpu.im) + ' ' + st;
end;

function TZX48.ScreenText: array of String;
var
  px: TUInts;
  rom: TBytes;
  row, col, x0, y0, bg, x, y, ch, c: Integer;
  cell: array[0..7] of Integer;
  same, inv: Boolean;
  line: String;
begin
  px := screen;
  rom := ROM_ZX48_ROM;
  for row := 0 to 23 do begin
    line := '';
    for col := 0 to 31 do begin
      x0 := 48 + col * 8;
      y0 := 48 + row * 8;
      bg := px[y0 * ULA_SCREEN_W + x0];
      for y := 0 to 7 do begin
        cell[y] := 0;
        for x := 0 to 7 do
          if px[(y0 + y) * ULA_SCREEN_W + x0 + x] <> bg then cell[y] := cell[y] or ($80 shr x);
      end;
      ch := Ord('?');
      for c := 32 to 127 do begin
        same := True;
        inv := True;
        for y := 0 to 7 do begin
          if rom[$3D00 + (c - 32) * 8 + y] <> cell[y] then same := False;
          if ((not rom[$3D00 + (c - 32) * 8 + y]) and $FF) <> cell[y] then inv := False;
        end;
        if same or inv then begin ch := c; break; end;
      end;
      if ch = $7F then line := line + '(c)'
      else line := line + Chr(ch);
    end;
    Result.Add(TrimRight(line));
  end;
end;

initialization
  PT_Z80 := TPartType.Create('Z80A',
    'A11:O A12:O A13:O A14:O A15:O CLK:m D4:B D3:B D5:B D6:B VCC:P D2:B D7:B D0:B D1:B /INT:m /NMI:m ' +
    '/HALT:O /MREQ:O /IORQ:O /RD:O /WR:O /BUSAK:O /WAIT:m /BUSRQ:m /RESET:m /M1:O /RFSH:O GND:P A0:O ' +
    'A1:O A2:O A3:O A4:O A5:O A6:O A7:O A8:O A9:O A10:O', cpu_eval, nil, nil, nil, TECH_NMOS);
end.
