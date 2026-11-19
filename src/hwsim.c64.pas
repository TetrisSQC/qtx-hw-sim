unit hwsim.c64;

{ The Commodore 64, board C64 ASSY 250407 (port of c64.c, cart.c, harness.c).

  FAST MODE

  While the board is healthy (no fault, no harness; keys, RESTORE and RESET
  are fine) and no pin is on the scope, FastCycle does what BoardCycle does
  without the netlist: the same 6510, VIC-II, CIA and SID cores ticked in the
  same order, the PLA's memory map, the data bus as one byte that keeps its
  last value (the "floating bus"). On a fault the RAM goes into the eight
  4164s and the board takes over; the colour RAM is the 2114's own cells.

  HOW A BUS CYCLE IS SIMULATED

  A C64 cycle (about 1 microsecond) has two halves. In the first half (phi1,
  PHI0 low) the VIC-II reads memory; in the second half (phi2, PHI0 high) the
  CPU does, unless the VIC "steals" it (badlines, sprites). In both halves
  the VIC runs a DRAM cycle: row address, /RAS low, column address, /CAS
  low. Each half is simulated in three steps, letting the whole netlist
  settle after each one:

    STEP_ROW  /RAS and /CAS high, row address on the multiplexed lines
    STEP_RAS  /RAS falls: the DRAMs and the 74LS373 latch the row
    STEP_CAS  /CAS falls: column address, the PLA asserts /CASRAM if the
              DRAM is selected, ROMs and RAMs put data on the bus

  At the end of phi2 the chips with a core (VIC registers, CIAs, SID) see
  the bus, perform register reads/writes and advance one clock; finally the
  CPU samples the data bus. Nothing here knows the C64 memory map: chip
  selects, banking, the address multiplexing, everything comes from the
  netlist and the glue chip models. }

interface

uses
  qtx.sysutils,
  hwsim.util, hwsim.boarddata, hwsim.bd_c64, hwsim.roms, hwsim.sim, hwsim.parts, hwsim.board,
  hwsim.m6502, hwsim.m6569, hwsim.m6526, hwsim.m6581, hwsim.kbd;

const
  C64_FREQ = 985248;
  C64_CYCLES_PER_FRAME = 63 * 312;
  C64_SCREEN_W = 392;
  C64_SCREEN_H = 272;
  C64_STEPS_PER_CYCLE = 6;

  STEP_ROW = 0; STEP_RAS = 1; STEP_CAS = 2;

  C64_KEY_SPACE = $20;
  C64_KEY_CSRLEFT = $08;
  C64_KEY_CSRRIGHT = $09;
  C64_KEY_CSRDOWN = $0A;
  C64_KEY_CSRUP = $0B;
  C64_KEY_DEL = $01;
  C64_KEY_INST = $10;
  C64_KEY_HOME = $0C;
  C64_KEY_CLR = $02;
  C64_KEY_RETURN = $0D;
  C64_KEY_CTRL = $0E;
  C64_KEY_CBM = $0F;
  C64_KEY_RESTORE = $FF;
  C64_KEY_STOP = $03;
  C64_KEY_F1 = $F1;
  C64_KEY_F3 = $F3;
  C64_KEY_F5 = $F5;
  C64_KEY_F7 = $F7;

  DOUT_VIC = 0; DOUT_CIA1 = 1; DOUT_CIA2 = 2; DOUT_SID = 3;

  C64_MODE_NONE = 0; C64_MODE_PINS = 1; C64_MODE_FAST = 2;
  C64_NMI_CYCLES = 1000;            { the 556's RESTORE pulse (hwsim.parts) }
  FR_OPEN = 0; FR_RAM = 1; FR_BASIC = 2; FR_KERNAL = 3; FR_CHAR = 4; FR_IO = 5; FR_ROML = 6; FR_ROMH = 7;

type
  TCart = class
  public
    name: String;
    exrom, game: Boolean;
    has_roml, has_romh: Boolean;
    roml, romh: TBytes;
    pin_roml, pin_romh, pin_game, pin_exrom: Integer;
    pin_a: array[0..12] of Integer;
    pin_d: array[0..7] of Integer;
  end;

  TC64 = class(TBoard)
  protected
    procedure Describe; override;
    procedure MachineCreate; override;
    procedure MachinePowerOn; override;
    procedure MachineJoins; override;
  public
    cpu: TM6502;
    cpu_pins: Integer;
    p_cpu: Integer;
    vic: TM6569;
    p_vic: Integer;
    cia: array[0..1] of TM6526;
    p_cia: array[0..1] of Integer;
    sid: TM6581;
    p_sid: Integer;
    p_u31, p_y1, p_cn1: Integer;
    dram_part: array[0..7] of Integer;
    pin_cn1_pa, pin_cn1_pb: array[0..7] of Integer;
    pin_restore, net_gnd: Integer;
    phase, step: Integer;
    vic_owns: Boolean;
    vic_addr: Integer;
    vic_fetches: Integer;
    vic_steal: Boolean;
    ba_streak: Integer;
    vic_ba_low, vic_irq: Boolean;
    vic_refresh: Integer;
    vic_reg: Integer;
    dout_en: array[0..3] of Boolean;
    dout_val: array[0..3] of Integer;
    cia_irq: array[0..1] of Boolean;
    cia_outs: array[0..1] of Integer;
    cpu_ticked: Boolean;
    cache_cpu_addr, cache_cpu_data, cache_cpu_port, cache_vic_mux, cache_vic_addr, cache_vic_data: Integer;
    cache_cia_pa, cache_cia_pb, cache_cia_data: array[0..1] of Integer;
    cache_sid_data: Integer;
    seen_phi0, seen_dot, seen_color: array[0..1] of Integer;
    seen_phi2: array[0..5] of Integer;   { [i*2 + h] }
    cpu_running, vic_running, color_ok, cpu_in_reset: Boolean;
    vic_last_tick: Float;
    kbd: TKbd;
    key_matrix: array[0..8] of Integer;
    pin_mains, p_r5, p_r100, pin_motor, pin_motor_ctl: Integer;
    mains_next, mains_edges: Float;
    harness, harness_write: Boolean;
    pin_hmotor, pin_hwrite, p_u28: Integer;
    cart: TCart;
    p_cn6: Integer;
    monitor_seed: Integer;
    { fast mode }
    mode: Integer;
    fram: TBytes;
    fcolor, fbasic, fkernal, fchar: TBytes;
    ft556: TTimer556State;
    fbus: Integer;
    f_res_low, f_nmi_low, f_tod: Boolean;
    healthy_epoch: Integer;
    healthy_cache: Boolean;

    constructor Create; override;
    procedure Run(cycles: Integer); override;
    procedure BoardCycle;
    procedure BusStep(astep: Integer);
    procedure BusHalf(aphase: Integer; owns: Boolean; addr: Integer);
    procedure SetDout(idx, part, val: Integer);
    procedure CpuTick;
    procedure CpuSample;
    procedure VicTick;
    procedure VicFinish;
    function VicFetch(addr: Integer): Integer;
    procedure CiaTick(i: Integer);
    procedure SidTick;
    procedure MainsTick;
    procedure MotorTick;
    function DoutPart(i: Integer): Integer;
    function Cn1PinOnNetOf(part, number: Integer): Integer;
    procedure InitKeymap;
    procedure SyncKeys;
    procedure RamPoke(addr, v: Integer);
    function DramPeek(addr: Integer): Integer;
    function LoadPrg(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean;
    { cartridges }
    function InsertCart(const path: String; const bytes: TBytes; var msg: String): Boolean;
    procedure Plug(c: TCart);
    { harness }
    function FindPin(const spec: String): Integer;
    procedure JoinPins(const a, c: String);
    procedure HarnessJoins;
    procedure HarnessTick;
    procedure SetHarness(plugged: Boolean);
    function PotValue(sid_pin: Integer): Integer;
    { fast mode }
    function Fast: Boolean;
    function Healthy: Boolean;
    procedure ChipsToFast;
    procedure FastToChips;
    function FastPortPins: Integer;
    function FastVicRead(addr: Integer): Integer;
    function FastVicFetch(addr: Integer): Integer;
    function FastRegion(addr: Integer; read: Boolean): Integer;
    procedure FastCiaInputs(i: Integer; var pa, pb: Integer);
    procedure FastCycle;
    function FastRestoreWired: Boolean;
    procedure RunFast(cycles: Integer);

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
    function CpuRegs: String; override;
    function ScreenText: array of String; override;
  end;

implementation

const
  CPU_ADDR_PINS: array[0..15] of Integer = [7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23];
  CPU_DATA_PINS: array[0..7] of Integer = [37, 36, 35, 34, 33, 32, 31, 30];
  VIC_MUX_PINS: array[0..5] of Integer = [24, 25, 26, 27, 28, 29];
  VIC_DATA_PINS: array[0..11] of Integer = [7, 6, 5, 4, 3, 2, 1, 39, 38, 37, 36, 35];
  VIC_COLOR_PINS: array[0..3] of Integer = [38, 37, 36, 35];
  VIC_A67_PINS: array[0..1] of Integer = [30, 31];
  VIC_A811_PINS: array[0..3] of Integer = [32, 33, 34, 23];
  CIA_DATA_PINS: array[0..7] of Integer = [33, 32, 31, 30, 29, 28, 27, 26];
  CIA_RS_PINS: array[0..3] of Integer = [38, 37, 36, 35];
  SID_ADDR_PINS: array[0..4] of Integer = [9, 10, 11, 12, 13];
  SID_DATA_PINS: array[0..7] of Integer = [15, 16, 17, 18, 19, 20, 21, 22];

  GK_CPU_DATA = GK_MACHINE + 1;
  GK_VIC_MUX = GK_MACHINE + 2;
  GK_VIC_DATA = GK_MACHINE + 3;
  GK_VIC_COLOR = GK_MACHINE + 4;
  GK_CIA_RS = GK_MACHINE + 5;
  GK_CIA_DATA = GK_MACHINE + 6;
  GK_SID_ADDR = GK_MACHINE + 7;
  GK_SID_DATA = GK_MACHINE + 8;

  MAINS_HZ = 50;
  PADDLE_OHM = 110e3;

{ ------------------------------------------------------------------------ }
{ Part models of the big chips                                              }
{ ------------------------------------------------------------------------ }

procedure cpu_eval(s: TSim; p: TPart);
var
  c: TC64;
  port_key, i, pins, addr: Integer;
  bus, write: Boolean;
begin
  c := TC64(p.user);
  { PHI2 output follows the PHI0 input: it clocks the CIAs and the SID. }
  s.PinOut(p, 39, DRV_BIT(s.PinIn(p, 1)));
  { I/O port P0..P5 (pins 29..24): driven where the DDR says output. }
  port_key := (c.cpu.io_ddr shl 8) or c.cpu.io_out;
  if c.cache_cpu_port <> port_key + 1 then begin
    c.cache_cpu_port := port_key + 1;
    for i := 0 to 5 do
      if ((c.cpu.io_ddr shr i) and 1) <> 0 then s.PinOut(p, 29 - i, DRV_BIT((c.cpu.io_out shr i) and 1))
      else s.PinOut(p, 29 - i, DRV_Z);
  end;
  { Address, R/W and data only while AEC is high. }
  bus := s.PinIn(p, 5) <> 0;
  pins := c.cpu_pins;
  addr := M6502_GET_ADDR(pins);
  write := (pins and M6502_RW) = 0;
  c.cache_cpu_addr := part_drive_bus_cached(s, p, CPU_ADDR_PINS, 16, addr, bus, c.cache_cpu_addr);
  if bus then s.PinOut(p, 38, DRV_BIT(not write)) else s.PinOut(p, 38, DRV_Z);
  { Writes to $00/$01 go to the internal port: the 6510 doesn't drive the bus. }
  c.cache_cpu_data := part_drive_bus_cached(s, p, CPU_DATA_PINS, 8, M6502_GET_DATA(pins),
    bus and write and (addr > 1), c.cache_cpu_data);
end;

function cpu_port_in(user: TObject): Integer;
var
  c: TC64;
  i: Integer;
  p: TPart;
begin
  c := TC64(user);
  if c.Fast then exit(c.FastPortPins);
  p := c.sim.parts[c.p_cpu];
  Result := 0;
  for i := 0 to 5 do Result := Result or (c.sim.PinIn(p, 29 - i) shl i);
end;

procedure cpu_port_out(data: Integer; user: TObject);
begin
end;

procedure vic_eval(s: TSim; p: TPart);
var
  c: TC64;
  run, own: Boolean;
  a, mux, key: Integer;
begin
  c := TC64(p.user);
  run := c.vic_running;
  a := c.vic_addr;
  own := run and c.vic_owns;
  if run then begin
    s.PinOut(p, 17, DRV_BIT(c.phase = 2));          { PHI0 }
    s.PinOut(p, 18, DRV_BIT(c.step = STEP_ROW));    { /RAS }
    s.PinOut(p, 19, DRV_BIT(c.step <> STEP_CAS));   { /CAS }
    s.PinOut(p, 16, DRV_BIT(not c.vic_owns));       { AEC }
    s.PinOut(p, 12, DRV_BIT(not c.vic_ba_low));     { BA }
    if c.vic_irq then s.PinOut(p, 8, DRV_LO) else s.PinOut(p, 8, DRV_Z);
  end else begin
    s.PinOut(p, 17, DRV_Z);
    s.PinOut(p, 18, DRV_Z);
    s.PinOut(p, 19, DRV_Z);
    s.PinOut(p, 16, DRV_Z);
    s.PinOut(p, 12, DRV_Z);
    s.PinOut(p, 8, DRV_Z);
  end;
  { Multiplexed address: row (A0-A5) then column (A8-A13). }
  if c.step = STEP_CAS then mux := (a shr 8) and 63 else mux := a and 63;
  c.cache_vic_mux := part_drive_bus_cached(s, p, VIC_MUX_PINS, 6, mux, own, c.cache_vic_mux);
  { Static address pins A6-A7 (to U14) and A8-A11 (on the address bus). }
  if own then key := ((a shr 6) and 63) + 1 else key := -1;
  if c.cache_vic_addr <> key then begin
    c.cache_vic_addr := key;
    part_drive_bus(s, p, VIC_A67_PINS, 2, (a shr 6) and 3, own);
    part_drive_bus(s, p, VIC_A811_PINS, 4, (a shr 8) and 15, own);
  end;
  { Register read: data on D0-D7. }
  c.cache_vic_data := part_drive_bus_cached(s, p, VIC_DATA_PINS, 8, c.dout_val[DOUT_VIC],
    c.dout_en[DOUT_VIC], c.cache_vic_data);
end;

procedure vic_commit(s: TSim; p: TPart);
var c: TC64;
begin
  c := TC64(p.user);
  if (c.step = STEP_RAS) and not c.vic_owns then
    c.vic_reg := part_read_bus(s, p, GK_VIC_MUX, VIC_MUX_PINS, 6);
end;

function vic_fetch_cb(addr: Integer; user: TObject): Integer;
begin
  if TC64(user).Fast then exit(TC64(user).FastVicFetch(addr));
  Result := TC64(user).VicFetch(addr);
end;

procedure cia_eval(s: TSim; p: TPart);
var
  b: TC64;
  i, port, bit, lows: Integer;
  c: TM6526;
  sp_out: Boolean;
begin
  b := TC64(p.user);
  if p.index = b.p_cia[0] then i := 0 else i := 1;
  c := b.cia[i];
  { Port lines: an output 0 pulls down strongly, everything else is a weak pull-up. }
  for port := 0 to 1 do begin
    if port = 0 then lows := (c.pa.ddr and not c.pa.reg) and $FF
    else lows := (c.pb.ddr and not c.pb.reg) and $FF;
    if port = 0 then begin
      if b.cache_cia_pa[i] = lows + 1 then continue;
      b.cache_cia_pa[i] := lows + 1;
    end else begin
      if b.cache_cia_pb[i] = lows + 1 then continue;
      b.cache_cia_pb[i] := lows + 1;
    end;
    for bit := 0 to 7 do
      if ((lows shr bit) and 1) <> 0 then s.PinOut(p, 2 + port * 8 + bit, DRV_LO)
      else s.PinOut(p, 2 + port * 8 + bit, DRV_PULLUP);
  end;
  if b.cia_irq[i] then s.PinOut(p, 21, DRV_LO) else s.PinOut(p, 21, DRV_Z);   { /IRQ }
  if c.pc_low then s.PinOut(p, 18, DRV_LO) else s.PinOut(p, 18, DRV_HI);       { /PC }
  sp_out := (c.ta.cr and (1 shl 6)) <> 0;
  if sp_out then begin
    s.PinOut(p, 39, DRV_BIT(c.sp_sp_out));
    s.PinOut(p, 40, DRV_BIT(c.sp_cnt_out));
  end else begin
    s.PinOut(p, 39, DRV_Z);
    s.PinOut(p, 40, DRV_Z);
  end;
  b.cache_cia_data[i] := part_drive_bus_cached(s, p, CIA_DATA_PINS, 8, b.dout_val[DOUT_CIA1 + i],
    b.dout_en[DOUT_CIA1 + i], b.cache_cia_data[i]);
end;

procedure sid_eval(s: TSim; p: TPart);
var c: TC64;
begin
  c := TC64(p.user);
  c.cache_sid_data := part_drive_bus_cached(s, p, SID_DATA_PINS, 8, c.dout_val[DOUT_SID],
    c.dout_en[DOUT_SID], c.cache_sid_data);
end;

{ The cartridge in the expansion port: the ROM selected by the PLA drives
  the data bus; the address is A0-A12 (8K). }
procedure cart_eval(s: TSim; p: TPart);
var
  c: TCart;
  rom: TBytes;
  a, v, i: Integer;
begin
  c := TCart(p.state);
  rom := nil;
  if c.has_roml and (s.Read(c.pin_roml) = 0) then rom := c.roml
  else if c.has_romh and (s.Read(c.pin_romh) = 0) then rom := c.romh;
  v := 0;
  if rom <> nil then begin
    a := 0;
    for i := 0 to 12 do a := a or (s.Read(c.pin_a[i]) shl i);
    v := rom[a];
  end;
  for i := 0 to 7 do
    if rom <> nil then s.Drive(c.pin_d[i], DRV_BIT((v shr i) and 1))
    else s.Drive(c.pin_d[i], DRV_Z);
end;

{ ------------------------------------------------------------------------ }
{ Setup                                                                     }
{ ------------------------------------------------------------------------ }

function c64_rail_of(const name: String): Integer;
begin
  if name = 'Earth' then exit(RAIL_GND);
  if (name = '+5V') or (name = '+Vc') or (name = 'CAN+5V') or (name = '+12V') or
     (name = '5V_joyport') or (name = '5V_FB3') then exit(RAIL_VCC);
  Result := RAIL_NONE;
end;

function c64_kind_of(const ref: String): Integer;
var c0, c1: String;
begin
  c0 := Copy(ref, 1, 1);
  c1 := Copy(ref, 2, 1);
  if c0 = 'U' then exit(PART_IC);
  if c0 = 'R' then exit(PART_RESISTOR);
  if (Copy(ref, 1, 2) = 'FB') or (c0 = 'L') or (c0 = 'F') or
     ((c0 = 'E') and (c1 >= '0') and (c1 <= '9') and (c1 <> '')) then exit(PART_LINK);
  if Copy(ref, 1, 2) = 'CN' then exit(PART_CONNECTOR);
  Result := PART_OTHER;
end;

constructor TC64.Create;
begin
  inherited Create;
  cpu := TM6502.Create;
  vic := TM6569.Create;
  cia[0] := TM6526.Create;
  cia[1] := TM6526.Create;
  sid := TM6581.Create;
  kbd := TKbd.Create;
  monitor_seed := 12345;
end;

procedure TC64.Describe;
begin
  id := 'c64';
  name := 'Commodore 64';
  board_name := 'C64 ASSY 250407';
  data := BoardC64;
  simdesc.rail_of := c64_rail_of;
  simdesc.kind_of := c64_kind_of;
  simdesc.cycles_per_us := 1.0;
  cpu_hz := C64_FREQ;
  cycles_per_frame := C64_CYCLES_PER_FRAME;
  steps_per_cycle := C64_STEPS_PER_CYCLE;
  screen_w := C64_SCREEN_W;
  screen_h := C64_SCREEN_H;
  ready_cycles := 3 * C64_FREQ;
  link_resistors := 'RP1 RP2 R42';
  crystals := 'Y1';
  keys_help := 'Keys: ESC=RUN/STOP  PgUp=RESTORE  TAB=CTRL  ALT=C=';
  reset_help := 'Hold to press RESET: shorts /RESET to GND (user port pin 3 to pin 1)';
  files_title := 'Open a program (.prg) or a cartridge (.crt)';
  cart_slot := 'CN6';
  part_defs.Add(TPartDef.Create('U21', PT_4164, 'RAM1', ROLE_RAM, '4164'#10'bit D0'#10'$0000-$FFFF', '4164 DRAM, holds data bit D0 of every byte, $0000-$FFFF'));
  part_defs.Add(TPartDef.Create('U9', PT_4164, 'RAM2', ROLE_RAM, '4164'#10'bit D1'#10'$0000-$FFFF', '4164 DRAM, holds data bit D1 of every byte, $0000-$FFFF'));
  part_defs.Add(TPartDef.Create('U22', PT_4164, 'RAM3', ROLE_RAM, '4164'#10'bit D2'#10'$0000-$FFFF', '4164 DRAM, holds data bit D2 of every byte, $0000-$FFFF'));
  part_defs.Add(TPartDef.Create('U10', PT_4164, 'RAM4', ROLE_RAM, '4164'#10'bit D3'#10'$0000-$FFFF', '4164 DRAM, holds data bit D3 of every byte, $0000-$FFFF'));
  part_defs.Add(TPartDef.Create('U23', PT_4164, 'RAM5', ROLE_RAM, '4164'#10'bit D4'#10'$0000-$FFFF', '4164 DRAM, holds data bit D4 of every byte, $0000-$FFFF'));
  part_defs.Add(TPartDef.Create('U11', PT_4164, 'RAM6', ROLE_RAM, '4164'#10'bit D5'#10'$0000-$FFFF', '4164 DRAM, holds data bit D5 of every byte, $0000-$FFFF'));
  part_defs.Add(TPartDef.Create('U24', PT_4164, 'RAM7', ROLE_RAM, '4164'#10'bit D6'#10'$0000-$FFFF', '4164 DRAM, holds data bit D6 of every byte, $0000-$FFFF'));
  part_defs.Add(TPartDef.Create('U12', PT_4164, 'RAM8', ROLE_RAM, '4164'#10'bit D7'#10'$0000-$FFFF', '4164 DRAM, holds data bit D7 of every byte, $0000-$FFFF'));
  part_defs.Add(TPartDef.Create('U6', PT_2114, 'VRAM', ROLE_VRAM, '2114'#10'color RAM'#10'$D800-$DBFF',
    '2114 color RAM (video), $D800-$DBFF, 4 bits, read by the VIC every character'));
  part_defs.Add(TPartDef.Create('U3', PT_2364, 'BASIC', ROLE_ROM, '2364 ROM'#10'$A000-$BFFF', '2364 BASIC ROM, $A000-$BFFF'));
  part_defs.Add(TPartDef.Create('U4', PT_2364, 'KERNAL', ROLE_ROM, '2364 ROM'#10'$E000-$FFFF', '2364 KERNAL ROM, $E000-$FFFF'));
  part_defs.Add(TPartDef.Create('U5', PT_2332, 'CHAR', ROLE_ROM, '2332 ROM'#10'$D000-$DFFF',
    '2332 character ROM, $D000-$DFFF (the VIC sees it at $1000 and $9000)'));
  part_defs.Add(TPartDef.Create('U17', PT_82S100, 'PLA', ROLE_PLA, '906114-01'#10'memory map',
    '906114-01 PLA (82S100): the memory map, decodes all the chip selects'));
  part_defs.Add(TPartDef.Create('U7', PT_6510, 'CPU', ROLE_CHIP, '6510', '6510 CPU'));
  part_defs.Add(TPartDef.Create('U19', PT_6569, 'VIC', ROLE_CHIP, '6569 VIC-II'#10'$D000-$D3FF',
    '6569 VIC-II video chip, $D000-$D3FF (PAL); also makes the system clock'));
  part_defs.Add(TPartDef.Create('U18', PT_6581, 'SID', ROLE_CHIP, '6581'#10'$D400-$D7FF', '6581 SID sound chip, $D400-$D7FF'));
  part_defs.Add(TPartDef.Create('U1', PT_6526, 'CIA1', ROLE_CHIP, '6526'#10'$DC00-$DCFF', '6526 CIA #1, $DC00-$DCFF: keyboard, joysticks, IRQ'));
  part_defs.Add(TPartDef.Create('U2', PT_6526, 'CIA2', ROLE_CHIP, '6526'#10'$DD00-$DDFF',
    '6526 CIA #2, $DD00-$DDFF: serial bus, user port, VIC bank, NMI'));
  part_defs.Add(TPartDef.Create('U25', PT_74LS257, 'MUX', ROLE_GLUE, '74LS257', '74LS257 DRAM address mux: CPU A0-A3 (row), A8-A11 (column)'));
  part_defs.Add(TPartDef.Create('U13', PT_74LS257, 'MUX', ROLE_GLUE, '74LS257', '74LS257 DRAM address mux: CPU A4-A7 (row), A12-A15 (column)'));
  part_defs.Add(TPartDef.Create('U14', PT_74LS258, 'MUX', ROLE_GLUE, '74LS258', '74LS258 VIC address mux: VIC A6-A7 (row), bank bits VA14-VA15 (column)'));
  part_defs.Add(TPartDef.Create('U26', PT_74LS373, 'LATCH', ROLE_GLUE, '74LS373', '74LS373 latch: puts the VIC address on A0-A7'));
  part_defs.Add(TPartDef.Create('U15', PT_74LS139, 'I/O', ROLE_GLUE, '74LS139',
    '74LS139 I/O decoder: VIC, SID, color RAM, CIA1, CIA2, I/O1, I/O2'));
  part_defs.Add(TPartDef.Create('U27', PT_74LS08, 'AND', ROLE_GLUE, '74LS08', '74LS08 AND gates: CPU AEC, RDY, color RAM select, TOD clock'));
  part_defs.Add(TPartDef.Create('U8', PT_7406, 'INV', ROLE_GLUE, '7406', '7406 open collector inverters: /AEC, /RESET, /NMI, serial bus'));
  part_defs.Add(TPartDef.Create('U16', PT_4066, 'SW', ROLE_GLUE, '4066', '4066 switch: connects the color RAM to D0-D3 in CPU cycles'));
  part_defs.Add(TPartDef.Create('U28', PT_4066, 'SW', ROLE_GLUE, '4066', '4066 switch: selects the paddles read by the SID'));
  part_defs.Add(TPartDef.Create('U20', PT_556, 'RST', ROLE_GLUE, '556', '556 timer: power-on reset, RESTORE key NMI'));
  part_defs.Add(TPartDef.Create('U31', PT_74LS629, 'OSC', ROLE_CLOCK, '74LS629', '74LS629 oscillators: dot clock and color clock'));
  part_defs.Add(TPartDef.Create('U29', PT_74LS74, 'PLL', ROLE_CLOCK, '74LS74', '74LS74, part of the clock PLL (not simulated)'));
  part_defs.Add(TPartDef.Create('U30', PT_74LS193, 'PLL', ROLE_CLOCK, '74LS193', '74LS193, part of the clock PLL (not simulated)'));
  part_defs.Add(TPartDef.Create('U32', PT_MC4044, 'PLL', ROLE_CLOCK, 'MC4044', 'MC4044 phase detector, part of the clock PLL (not simulated)'));
  conn_defs.Add(TConnDef.Create('CN1', 'KEYBOARD', '1=GND 3=RESTORE 4=+5V 5=ROW3 6=ROW6 7=ROW5 8=ROW4 9=ROW7 10=ROW2 ' +
    '11=ROW1 12=ROW0 13=COL0 14=COL6 15=COL5 16=COL4 17=COL3 18=COL2 19=COL1 20=COL7'));
  conn_defs.Add(TConnDef.Create('CN2', 'USER PORT', '1=GND 2=+5V 3=/RESET 4=CNT1 5=SP1 6=CNT2 7=SP2 8=/PC2 9=SERIAL_ATN ' +
    '10=9V_AC 11=9V_AC 12=GND A=GND B=/FLAG2 C=PB0 D=PB1 E=PB2 F=PB3 H=PB4 J=PB5 K=PB6 L=PB7 M=PA2 N=GND'));
  conn_defs.Add(TConnDef.Create('CN3', 'CASSETTE', 'A=GND B=+5V C=MOTOR D=READ E=WRITE F=SENSE'));
  conn_defs.Add(TConnDef.Create('CN4', 'SERIAL', '1=SRQ_IN 2=GND 3=ATN 4=CLK 5=DATA 6=/RESET'));
  conn_defs.Add(TConnDef.Create('CN5', 'AUDIO/VIDEO', '1=LUMINANCE 2=GND 3=AUDIO_OUT 4=VIDEO_OUT 5=AUDIO_IN 6=CHROMA ' +
    '7=NOT_CONNECTED 8=NOT_CONNECTED'));
  conn_defs.Add(TConnDef.Create('CN6', 'EXPANSION PORT', '1=GND 2=+5V 3=+5V 4=/IRQ 5=R/W 6=DOT_CLOCK 7=/I/O1 8=/GAME ' +
    '9=/EXROM 10=/I/O2 11=/ROML 12=BA 13=/DMA 14=D7 15=D6 16=D5 17=D4 18=D3 19=D2 20=D1 21=D0 ' +
    '22=GND A=GND B=/ROMH C=/RESET D=/NMI E=PHI2 F=A15 H=A14 J=A13 K=A12 L=A11 M=A10 N=A9 ' +
    'P=A8 R=A7 S=A6 T=A5 U=A4 V=A3 W=A2 X=A1 Y=A0 Z=GND'));
  conn_defs.Add(TConnDef.Create('CN7', 'POWER', '1=GND 2=GND 3=GND 4=+5V 5=+5V 6=9V_AC 7=9V_AC'));
  conn_defs.Add(TConnDef.Create('CN8', 'CONTROL PORT 2', '1=UP 2=DOWN 3=LEFT 4=RIGHT 5=POT_Y 6=FIRE 7=+5V 8=GND 9=POT_X'));
  conn_defs.Add(TConnDef.Create('CN9', 'CONTROL PORT 1', '1=UP 2=DOWN 3=LEFT 4=RIGHT 5=POT_Y 6=FIRE/LIGHT_PEN 7=+5V 8=GND ' +
    '9=POT_X'));
  conn_defs.Add(TConnDef.Create('CN10', 'POWER LED', '1=LED+ 2=LED- 3=LED+'));
end;

function TC64.Cn1PinOnNetOf(part, number: Integer): Integer;
var
  net, i: Integer;
  cn1: TPart;
begin
  net := sim.pins[sim.parts[part].Pin(number)].net;
  cn1 := sim.parts[p_cn1];
  for i := 0 to cn1.npins - 1 do
    if sim.pins[cn1.first_pin + i].net = net then exit(cn1.first_pin + i);
  Result := -1;
end;

procedure TC64.MachineCreate;
var
  s: TSim;
  i, net, bit, k, q3, user_port, n: Integer;
  nm: String;
  t: TTimer556State;
  prefixes: array[0..3] of String;
  counts: array[0..3] of Integer;
begin
  s := sim;
  p_cpu := s.FindPart('U7');
  p_vic := s.FindPart('U19');
  p_cia[0] := s.FindPart('U1');
  p_cia[1] := s.FindPart('U2');
  p_sid := s.FindPart('U18');
  p_u31 := s.FindPart('U31');
  p_y1 := s.FindPart('Y1');
  p_cn1 := s.FindPart('CN1');
  net_gnd := s.FindNet('Earth');
  s.parts[p_cpu].eval := cpu_eval;
  s.parts[p_vic].eval := vic_eval;
  s.parts[p_vic].commit := vic_commit;
  s.parts[p_cia[0]].eval := cia_eval;
  s.parts[p_cia[1]].eval := cia_eval;
  s.parts[p_sid].eval := sid_eval;
  TRomState(s.parts[s.FindPart('U3')].state).data := ROM_DUMP_C64_BASIC_BIN;
  TRomState(s.parts[s.FindPart('U4')].state).data := ROM_DUMP_C64_KERNALV3_BIN;
  TRomState(s.parts[s.FindPart('U5')].state).data := ROM_DUMP_C64_CHAR_BIN;
  { Which DRAM holds which data bit: the one whose D pin is on net Dn. }
  for i := 0 to s.nparts - 1 do begin
    if not (s.parts[i].state is TDramState) then continue;
    net := s.pins[s.parts[i].Pin(2)].net;
    if net >= 0 then nm := s.nets[net].name else nm := '';
    if (Length(nm) = 2) and (nm[1] = 'D') and (nm[2] >= '0') and (nm[2] <= '7') then
      dram_part[Ord(nm[2]) - 48] := i;
  end;
  for i := 0 to 7 do begin
    pin_cn1_pa[i] := Cn1PinOnNetOf(p_cia[0], 2 + i);
    pin_cn1_pb[i] := Cn1PinOnNetOf(p_cia[0], 10 + i);
  end;
  pin_restore := s.parts[p_cn1].Pin(3);
  t := TTimer556State(s.parts[s.FindPart('U20')].state);
  t.restore_net := s.pins[pin_restore].net;
  InitKeymap;
  { The bus lines get consecutive bits in the bus bitmap (speed only). A bus
    never straddles the two 32-bit words of the bitmap. }
  prefixes[0] := 'A'; counts[0] := 16;
  prefixes[1] := 'D'; counts[1] := 8;
  prefixes[2] := 'VA'; counts[2] := 8;
  prefixes[3] := 'MA'; counts[3] := 8;
  bit := 0;
  for k := 0 to 3 do begin
    n := counts[k];
    if (bit < 32) and (bit + n > 32) then bit := 32;
    for i := 0 to n - 1 do begin
      s.SetBusBit(s.FindNet(prefixes[k] + IntToStr(i)), bit);
      Inc(bit);
    end;
  end;
  { The color RAM nibble (D8-D11 of the VIC). }
  if (bit < 32) and (bit + 4 > 32) then bit := 32;
  for i := 0 to 3 do begin
    s.SetBusBit(s.pins[s.parts[p_vic].Pin(38 - i)].net, bit);
    Inc(bit);
  end;
  p_r5 := s.FindPart('R5');
  p_r100 := s.FindPart('R100');
  if (p_r5 >= 0) and (p_r100 >= 0) then pin_mains := s.parts[p_r100].Pin(2) else pin_mains := -1;
  p_pla := s.FindPart('U17');
  p_cn6 := s.FindPart('CN6');
  p_u28 := s.FindPart('U28');
  q3 := s.FindPart('Q3');
  if q3 >= 0 then pin_motor := s.parts[q3].Pin(3) else pin_motor := -1;
  pin_motor_ctl := s.parts[p_cpu].Pin(24);
  pin_hmotor := -1;
  pin_hwrite := -1;
  { The RESET button sits on the user port, like a reset cartridge. }
  user_port := s.FindPart('CN2');
  pin_reset := PadPin(user_port, '3');
  pin_reset_gnd := PadPin(user_port, '1');
end;

procedure TC64.MachinePowerOn;
var i: Integer;
begin
  mains_next := 0;
  mode := C64_MODE_NONE;           { Run picks it, reading the RAM chips }
  healthy_epoch := -1;
  cpu_pins := cpu.Init(False, cpu_port_in, cpu_port_out, Self, 0, $C0);
  cpu_in_reset := True;
  vic.Init(vic_fetch_cb, Self, 64, 24, C64_SCREEN_W, C64_SCREEN_H, screen);
  cia[0].Init;
  cia[1].Init;
  sid.Init(C64_FREQ, 44100, 1.0);
  if screen <> nil then screen.fill(0);
  for i := 0 to 3 do dout_en[i] := False;
  cia_irq[0] := False;
  cia_irq[1] := False;
  vic_ba_low := False;
  vic_irq := False;
  ba_streak := 0;
  seen_phi0[0] := 0; seen_dot[0] := 0; seen_color[0] := 0;
  seen_phi0[1] := 1; seen_dot[1] := 1; seen_color[1] := 1;
  for i := 0 to 2 do begin seen_phi2[i * 2] := 0; seen_phi2[i * 2 + 1] := 1; end;
end;

procedure TC64.MachineJoins;
var
  s: TSim;
  line, col, pa, pb: Integer;
  vco: TVcoState;
begin
  s := sim;
  for line := 0 to 7 do
    for col := 0 to 7 do begin
      if ((key_matrix[line] shr col) and 1) = 0 then continue;
      pa := pin_cn1_pa[line];
      pb := pin_cn1_pb[col];
      if (pa < 0) or (pb < 0) or (s.pins[pa].fault = PF_OPEN) or (s.pins[pb].fault = PF_OPEN) then continue;
      s.Join(s.pins[pa].net, s.pins[pb].net);
    end;
  if ((key_matrix[8] and 1) <> 0) and (pin_restore >= 0) and (s.pins[pin_restore].fault <> PF_OPEN) then
    s.Join(s.pins[pin_restore].net, net_gnd);
  HarnessJoins;
  vco := TVcoState(s.parts[p_u31].state);
  vco.crystal_ok := (p_y1 < 0) or (s.parts[p_y1].rfault <> RF_OPEN);
end;

procedure TC64.InitKeymap;
const
  KEYMAP = '        3WA4ZSE 5RD6CFTX7YG8BHUV9IJ0MKON+PL-.:@,~*;  = /1  2  Q ' +
           '        #wa$zse %rd&cftx''yg(bhuv)ij0mkon pl >[ <$ ]    ?!  "  q ';
var
  shift, column, line, c: Integer;
begin
  kbd.Init(2);
  kbd.RegisterModifier(0, 7, 1);    { shift }
  kbd.RegisterModifier(1, 2, 7);    { ctrl }
  for shift := 0 to 1 do
    for column := 0 to 7 do
      for line := 0 to 7 do begin
        c := Ord(KEYMAP[shift * 64 + line * 8 + column + 1]);
        if c <> $20 then kbd.RegisterKey(c, column, line, shift);
      end;
  kbd.RegisterKey(C64_KEY_SPACE, 4, 7, 0);
  kbd.RegisterKey(C64_KEY_CSRLEFT, 2, 0, 1);
  kbd.RegisterKey(C64_KEY_CSRRIGHT, 2, 0, 0);
  kbd.RegisterKey(C64_KEY_CSRDOWN, 7, 0, 0);
  kbd.RegisterKey(C64_KEY_CSRUP, 7, 0, 1);
  kbd.RegisterKey(C64_KEY_DEL, 0, 0, 0);
  kbd.RegisterKey(C64_KEY_INST, 0, 0, 1);
  kbd.RegisterKey(C64_KEY_HOME, 3, 6, 0);
  kbd.RegisterKey(C64_KEY_CLR, 3, 6, 1);
  kbd.RegisterKey(C64_KEY_RETURN, 1, 0, 0);
  kbd.RegisterKey(C64_KEY_CTRL, 2, 7, 0);
  kbd.RegisterKey(C64_KEY_CBM, 5, 7, 0);
  kbd.RegisterKey(C64_KEY_RESTORE, 0, 8, 0);
  kbd.RegisterKey(C64_KEY_STOP, 7, 7, 0);
  kbd.RegisterKey(C64_KEY_F1, 4, 0, 0);
  kbd.RegisterKey(C64_KEY_F3, 5, 0, 0);
  kbd.RegisterKey(C64_KEY_F5, 6, 0, 0);
  kbd.RegisterKey(C64_KEY_F7, 3, 0, 0);
end;

{ ------------------------------------------------------------------------ }
{ CPU                                                                       }
{ ------------------------------------------------------------------------ }

procedure TC64.CpuTick;
var
  s: TSim;
  p: TPart;
  pins: Integer;
begin
  s := sim;
  p := s.parts[p_cpu];
  cpu_ticked := False;
  { No PHI0 clock from the VIC: the CPU is frozen. }
  cpu_running := (not p.dead) and (seen_phi0[0] = 0) and (seen_phi0[1] = 1);
  if not cpu_running then exit;
  { /RES is a level. }
  if s.PinIn(p, 40) = 0 then begin cpu_in_reset := True; exit; end;
  if cpu_in_reset then begin
    cpu_in_reset := False;
    cpu_pins := cpu_pins or M6502_RES;
    cpu.io_ddr := 0;
    cpu.io_out := 0;
  end;
  pins := cpu_pins and not (M6502_IRQ or M6502_NMI or M6502_RDY);
  if s.PinIn(p, 3) = 0 then pins := pins or M6502_IRQ;
  if s.PinIn(p, 4) = 0 then pins := pins or M6502_NMI;
  if s.PinIn(p, 2) = 0 then pins := pins or M6502_RDY;
  cpu_pins := cpu.Tick(pins);
  cpu_ticked := True;
  s.MarkDirty(p_cpu);
end;

function TC64.DramPeek(addr: Integer): Integer;
var cell, bit: Integer;
begin
  cell := ((addr and $FF) shl 8) or (addr shr 8);
  Result := 0;
  for bit := 0 to 7 do
    Result := Result or (TDramState(sim.parts[dram_part[bit]].state).cells[cell] shl bit);
end;

procedure TC64.CpuSample;
var
  s: TSim;
  pins, a, want: Integer;
begin
  if not cpu_ticked then exit;
  s := sim;
  pins := cpu_pins;
  if (pins and M6502_RW) <> 0 then
    pins := M6502_SET_DATA(pins, part_read_bus(s, s.parts[p_cpu], GK_CPU_DATA, CPU_DATA_PINS, 8));
  if M6510_CHECK_IO(pins) then pins := cpu.IORQ(pins);
  if check and not vic_steal and (cart = nil) then begin
    a := M6502_GET_ADDR(pins);
    if ((pins and M6502_RW) <> 0) and (a >= $E000) and ((cpu.io_out and cpu.io_ddr and 2) <> 0) then begin
      want := ROM_DUMP_C64_KERNALV3_BIN[a - $E000];
      if M6502_GET_DATA(pins) <> want then begin
        if check_errors < 10 then
          WriteLn('CHECK: CPU $' + HexW(a) + ' read $' + HexB(M6502_GET_DATA(pins)) + ', expected $' + HexB(want));
        Inc(check_errors);
      end;
    end;
  end;
  cpu_pins := pins;
  s.MarkDirty(p_cpu);
end;

{ ------------------------------------------------------------------------ }
{ Bus half cycles                                                           }
{ ------------------------------------------------------------------------ }

procedure TC64.BusStep(astep: Integer);
var s: TSim;
begin
  s := sim;
  step := astep;
  if phase = 2 then s.clock_level := 1 else s.clock_level := 0;
  s.MarkDirty(p_vic);
  if astep = STEP_ROW then s.MarkDirty(p_u31);
  s.Settle;
  s.Commit;
  s.Settle;
  Sample;
end;

function TC64.DoutPart(i: Integer): Integer;
begin
  if i = DOUT_VIC then exit(p_vic);
  if i = DOUT_SID then exit(p_sid);
  Result := p_cia[i - DOUT_CIA1];
end;

procedure TC64.BusHalf(aphase: Integer; owns: Boolean; addr: Integer);
var
  s: TSim;
  i, h: Integer;
begin
  s := sim;
  phase := aphase;
  vic_owns := owns;
  vic_addr := addr;
  for i := 0 to 3 do begin
    if not dout_en[i] then continue;
    dout_en[i] := False;
    s.MarkDirty(DoutPart(i));
  end;
  BusStep(STEP_ROW);
  BusStep(STEP_RAS);
  BusStep(STEP_CAS);
  h := aphase - 1;
  seen_phi0[h] := s.PinIn(s.parts[p_cpu], 1);
  seen_dot[h] := s.PinIn(s.parts[p_vic], 22);
  seen_color[h] := s.PinIn(s.parts[p_vic], 21);
  seen_phi2[0 * 2 + h] := s.PinIn(s.parts[p_cia[0]], 25);
  seen_phi2[1 * 2 + h] := s.PinIn(s.parts[p_cia[1]], 25);
  seen_phi2[2 * 2 + h] := s.PinIn(s.parts[p_sid], 6);
end;

procedure TC64.SetDout(idx, part, val: Integer);
begin
  dout_en[idx] := True;
  dout_val[idx] := val;
  sim.MarkDirty(part);
end;

{ ------------------------------------------------------------------------ }
{ VIC                                                                       }
{ ------------------------------------------------------------------------ }

function TC64.VicFetch(addr: Integer): Integer;
var
  ph, a, want, got: Integer;
  p: TPart;
begin
  if (vic_fetches > 0) and vic_steal then ph := 2 else ph := 1;
  Inc(vic_fetches);
  BusHalf(ph, True, addr and $3FFF);
  p := sim.parts[p_vic];
  Result := part_read_bus(sim, p, GK_VIC_DATA, VIC_DATA_PINS, 8) or
            (part_read_bus(sim, p, GK_VIC_COLOR, VIC_COLOR_PINS, 4) shl 8);
  if check and (cart = nil) then begin
    a := addr and $3FFF;
    got := Result and $FF;
    if (a and $3000) = $1000 then want := ROM_DUMP_C64_CHAR_BIN[a and $FFF] else want := DramPeek(a);
    if got <> want then begin
      if check_errors < 10 then
        WriteLn('CHECK: VIC $' + HexW(a) + ' read $' + HexB(got) + ', expected $' + HexB(want));
      Inc(check_errors);
    end;
  end;
end;

procedure TC64.VicTick;
var
  s: TSim;
  p: TPart;
  pins: Integer;
begin
  s := sim;
  p := s.parts[p_vic];
  vic_fetches := 0;
  vic_running := (not p.dead) and (seen_dot[0] <> seen_dot[1]);
  vic_steal := vic_running and vic_ba_low and (ba_streak >= 3);
  if vic_running then begin
    pins := vic.TickFetch(0);     { calls VicFetch() }
    vic_ba_low := (pins and M6569_BA) <> 0;
    vic_irq := (pins and M6569_IRQ) <> 0;
    vic_last_tick := s.cycle;
  end else begin
    vic_ba_low := False;
    vic_irq := False;
  end;
  if vic_ba_low then Inc(ba_streak) else ba_streak := 0;
  vic_steal := vic_ba_low and (ba_streak >= 4);
  s.MarkDirty(p_vic);
  { Every phi1 has a VIC access: otherwise a DRAM refresh ($3Fxx). }
  if vic_fetches = 0 then begin
    BusHalf(1, vic_running, $3F00 or vic_refresh);
    vic_refresh := (vic_refresh - 1) and $FF;
  end;
end;

procedure TC64.VicFinish;
var
  s: TSim;
  p: TPart;
  pins: Integer;
begin
  if not vic_running then exit;
  s := sim;
  p := s.parts[p_vic];
  pins := 0;
  if s.PinIn(p, 10) = 0 then begin                 { /CS: $D000-$D3FF }
    pins := pins or M6569_CS or (vic_reg and 63);
    if s.PinIn(p, 11) <> 0 then pins := pins or M6569_RW
    else pins := (pins and not $FF0000) or (part_read_bus(s, p, GK_VIC_DATA, VIC_DATA_PINS, 8) shl 16);
  end;
  pins := vic.TickFinish(pins);
  if (pins and (M6569_CS or M6569_RW)) = (M6569_CS or M6569_RW) then
    SetDout(DOUT_VIC, p_vic, (pins shr 16) and $FF);
end;

{ ------------------------------------------------------------------------ }
{ CIAs and SID                                                              }
{ ------------------------------------------------------------------------ }

procedure TC64.CiaTick(i: Integer);
var
  s: TSim;
  p: TPart;
  c: TM6526;
  pins, pa, pb, bit, outs: Integer;
begin
  s := sim;
  p := s.parts[p_cia[i]];
  if p.dead or not ((seen_phi2[i * 2] = 0) and (seen_phi2[i * 2 + 1] = 1)) then exit;
  c := cia[i];
  if s.PinIn(p, 34) = 0 then begin                 { /RES held low }
    c.Reset;
    cia_irq[i] := False;
    s.MarkDirty(p_cia[i]);
    exit;
  end;
  pins := 0;
  if s.PinIn(p, 23) = 0 then begin                 { /CS: a register access }
    pins := M6526_CS or part_read_bus(s, p, GK_CIA_RS, CIA_RS_PINS, 4);
    if s.PinIn(p, 22) <> 0 then pins := pins or M6526_RW;
    pins := M6526_SET_DATA(pins, part_read_bus(s, p, GK_CIA_DATA, CIA_DATA_PINS, 8));
    pa := 0; pb := 0;
    for bit := 0 to 7 do begin
      pa := pa or (s.PinIn(p, 2 + bit) shl bit);
      pb := pb or (s.PinIn(p, 10 + bit) shl bit);
    end;
  end else begin
    pa := c.pa.inp;
    pb := c.pb.inp;
  end;
  if s.PinIn(p, 24) = 0 then pins := pins or M6526_FLAG;
  if s.PinIn(p, 19) <> 0 then pins := pins or M6526_TOD;
  if s.PinIn(p, 39) <> 0 then pins := pins or M6526_SP;
  if s.PinIn(p, 40) <> 0 then pins := pins or M6526_CNT;
  pins := c.Tick(pins, pa, pb);
  outs := Ord(c.pc_low) or (Ord(c.sp_sp_out) shl 1) or (Ord(c.sp_cnt_out) shl 2) or (((c.ta.cr shr 6) and 1) shl 3);
  if outs <> cia_outs[i] then begin
    cia_outs[i] := outs;
    s.MarkDirty(p_cia[i]);
  end;
  cia_irq[i] := (pins and M6526_IRQ) <> 0;
  if (pins and (M6526_CS or M6526_RW)) = (M6526_CS or M6526_RW) then
    SetDout(DOUT_CIA1 + i, p_cia[i], M6526_GET_DATA(pins));
  s.MarkDirty(p_cia[i]);
end;

procedure TC64.SidTick;
var
  s: TSim;
  p: TPart;
  pins, reg: Integer;
begin
  s := sim;
  p := s.parts[p_sid];
  if p.dead or not ((seen_phi2[4] = 0) and (seen_phi2[5] = 1)) then exit;
  if s.PinIn(p, 5) = 0 then begin sid.Reset; exit; end;
  pins := 0;
  if s.PinIn(p, 8) = 0 then begin                  { /CS }
    pins := M6581_CS or part_read_bus(s, p, GK_SID_ADDR, SID_ADDR_PINS, 5);
    if s.PinIn(p, 7) <> 0 then pins := pins or M6581_RW;
    pins := (pins and not $FF0000) or (part_read_bus(s, p, GK_SID_DATA, SID_DATA_PINS, 8) shl 16);
  end;
  if (pins and (M6581_CS or M6581_RW)) = (M6581_CS or M6581_RW) then begin
    reg := pins and M6581_ADDR_MASK;
    if reg = M6581_POT_X then sid.pot[0] := PotValue(p.Pin(24));
    if reg = M6581_POT_Y then sid.pot[1] := PotValue(p.Pin(23));
  end;
  pins := sid.Tick(pins);
  if (pins and (M6581_CS or M6581_RW)) = (M6581_CS or M6581_RW) then
    SetDout(DOUT_SID, p_sid, (pins shr 16) and $FF);
end;

{ ------------------------------------------------------------------------ }
{ One cycle                                                                 }
{ ------------------------------------------------------------------------ }

procedure TC64.MainsTick;
var
  s: TSim;
  on: Boolean;
  level: Integer;
begin
  s := sim;
  if pin_mains < 0 then exit;
  if s.cycle >= mains_next then begin
    mains_edges := Trunc((s.cycle * (2 * MAINS_HZ)) / C64_FREQ) + 1;
    mains_next := Trunc((mains_edges * C64_FREQ + 2 * MAINS_HZ - 1) / (2 * MAINS_HZ));
  end;
  on := (s.parts[p_r5].rfault <> RF_OPEN) and (s.parts[p_r100].rfault <> RF_OPEN);
  if (Trunc(mains_edges - 1) and 1) <> 0 then level := 1 else level := 0;
  if on then s.Drive(pin_mains, MkDrive(S_STRONG, level)) else s.Drive(pin_mains, DRV_Z);
end;

procedure TC64.MotorTick;
var
  s: TSim;
  c: TPin;
  p5_high: Boolean;
begin
  s := sim;
  if (pin_motor < 0) or (pin_motor_ctl < 0) then exit;
  c := s.pins[pin_motor_ctl];
  p5_high := (c.root >= 0) and (s.nets[c.root].strength <> S_Z) and (s.nets[c.root].level <> 0);
  if p5_high then s.Drive(pin_motor, DRV_Z) else s.Drive(pin_motor, DRV_HI);
end;

procedure TC64.BoardCycle;
var s: TSim;
begin
  s := sim;
  s.cycle := s.cycle + 1;
  MainsTick;
  MotorTick;
  HarnessTick;
  CpuTick;
  VicTick;                                   { phi1 (and maybe a stolen phi2) }
  if not vic_steal then BusHalf(2, False, 0)  { phi2: the CPU's access }
  else if vic_fetches < 2 then BusHalf(2, True, $3FFF);
  VicFinish;
  CiaTick(0);
  CiaTick(1);
  SidTick;
  s.Settle;
  CpuSample;
end;

procedure TC64.Run(cycles: Integer);
var i, want: Integer;
begin
  if (not force_pins) and (scope_pin < 0) and Healthy then want := C64_MODE_FAST else want := C64_MODE_PINS;
  if want <> mode then begin
    if want = C64_MODE_FAST then ChipsToFast
    else if mode = C64_MODE_FAST then FastToChips;
    mode := want;
  end;
  if mode = C64_MODE_FAST then begin RunFast(cycles); exit; end;
  for i := 0 to cycles - 1 do BoardCycle;
end;

{ ------------------------------------------------------------------------ }
{ Fast mode                                                                 }
{ ------------------------------------------------------------------------ }

function TC64.Fast: Boolean;
begin
  Result := mode = C64_MODE_FAST;
end;

{ Healthy enough for fast mode: no pin fault, nothing removed or dead, no
  resistor fault, no harness. (Keys, RESTORE and RESET are fine.) }
function TC64.Healthy: Boolean;
var i: Integer;
begin
  if sim.epoch = healthy_epoch then exit(healthy_cache);
  healthy_epoch := sim.epoch;
  healthy_cache := False;
  if harness or check then exit(False);
  for i := 0 to sim.npins - 1 do
    if sim.pins[i].fault <> PF_OK then exit(False);
  for i := 0 to sim.nparts - 1 do
    if sim.parts[i].removed or sim.parts[i].dead or (sim.parts[i].rfault <> RF_OK) then exit(False);
  healthy_cache := True;
  Result := True;
end;

procedure TC64.ChipsToFast;
var
  a, bit: Integer;
  cells: TBytes;
begin
  if fram = nil then fram := NewBytes(65536);
  fram.fill(0);
  for bit := 0 to 7 do begin
    cells := TDramState(sim.parts[dram_part[bit]].state).cells;
    for a := 0 to 65535 do
      if cells[((a and $FF) shl 8) or (a shr 8)] <> 0 then fram[a] := fram[a] or (1 shl bit);
  end;
  fcolor := TSramState(sim.parts[sim.FindPart('U6')].state).cells;
  fbasic := TRomState(sim.parts[sim.FindPart('U3')].state).data;
  fkernal := TRomState(sim.parts[sim.FindPart('U4')].state).data;
  fchar := TRomState(sim.parts[sim.FindPart('U5')].state).data;
  ft556 := TTimer556State(sim.parts[sim.FindPart('U20')].state);
  fbus := (sim.busLevelLo shr 16) and $FF;
  { the lines as the board left them }
  f_res_low := sim.PinIn(sim.parts[p_cpu], 40) = 0;
  f_nmi_low := sim.PinIn(sim.parts[p_cpu], 4) = 0;
  f_tod := sim.PinIn(sim.parts[p_cia[0]], 19) <> 0;
end;

procedure TC64.FastToChips;
var
  a, bit, i: Integer;
  cells: TBytes;
begin
  for bit := 0 to 7 do begin
    cells := TDramState(sim.parts[dram_part[bit]].state).cells;
    for a := 0 to 65535 do cells[((a and $FF) shl 8) or (a shr 8)] := (fram[a] shr bit) and 1;
    sim.MarkDirty(dram_part[bit]);
  end;
  for i := 0 to 3 do dout_en[i] := False;
  cache_cpu_port := -1;
  cache_cia_pa[0] := -1; cache_cia_pa[1] := -1;
  cache_cia_pb[0] := -1; cache_cia_pb[1] := -1;
  cia_outs[0] := -1; cia_outs[1] := -1;
  for i := 0 to sim.nparts - 1 do
    if Assigned(sim.parts[i].eval) then sim.MarkDirty(i);
  { let every output (the 556's /RESET too) catch up before the CPU looks }
  sim.Settle;
end;

{ The 6510 port lines as they read: outputs, else the pull-ups. }
function TC64.FastPortPins: Integer;
begin
  Result := ((cpu.io_out and cpu.io_ddr) or (not cpu.io_ddr and $3F)) and $3F;
end;

{ What the VIC reads at a 14 bit address (its bank from CIA2 PA0/PA1). }
function TC64.FastVicRead(addr: Integer): Integer;
var
  pa, bank, a: Integer;
  ultimax: Boolean;
begin
  pa := (cia[1].pa.reg and cia[1].pa.ddr) or (not cia[1].pa.ddr and $FF);
  bank := 3 - (pa and 3);
  a := addr and $3FFF;
  ultimax := (cart <> nil) and cart.game and not cart.exrom;
  if ultimax and ((a and $3000) = $3000) then begin
    if cart.has_romh then Result := cart.romh[$1000 or (a and $FFF)] else Result := fbus;
  end else if (not ultimax) and ((bank and 1) = 0) and ((a and $3000) = $1000) then
    Result := fchar[a and $FFF]
  else
    Result := fram[(bank shl 14) or a];
end;

function TC64.FastVicFetch(addr: Integer): Integer;
var v: Integer;
begin
  Inc(vic_fetches);
  v := FastVicRead(addr);
  fbus := v;
  Result := v or ((fcolor[addr and $3FF] and 15) shl 8);
end;

{ The CPU's view of an address: FR_* (what the PLA selects, from its own
  terms: in phi2 with the CPU on the bus, /CAS low; BA counts for I/O reads). }
var
  PlaTable: TBytes;

function TC64.FastRegion(addr: Integer; read: Boolean): Integer;
var
  port, inp, outv, i: Integer;
begin
  if PlaTable = nil then begin
    PlaTable := NewBytes(65536);
    for i := 0 to 65535 do PlaTable[i] := pla_logic(i);
  end;
  port := FastPortPins;
  inp := (((addr shr 13) and 1) shl PI_A13) or (((addr shr 14) and 1) shl PI_A14) or
         (((addr shr 15) and 1) shl PI_A15) or (((addr shr 12) and 1) shl PI_A12) or (1 shl PI_VA14);
  if (port and 4) <> 0 then inp := inp or (1 shl PI_CHAREN);
  if (port and 2) <> 0 then inp := inp or (1 shl PI_HIRAM);
  if (port and 1) <> 0 then inp := inp or (1 shl PI_LORAM);
  if not ((cart <> nil) and cart.game) then inp := inp or (1 shl PI_GAME);
  if not ((cart <> nil) and cart.exrom) then inp := inp or (1 shl PI_EXROM);
  if read then inp := inp or (1 shl PI_RW);
  if not vic_ba_low then inp := inp or (1 shl PI_BA);
  outv := PlaTable[inp];                     { active low, but /CASRAM }
  if ((outv shr PO_IO) and 1) = 0 then exit(FR_IO);
  if ((outv shr PO_KERNAL) and 1) = 0 then exit(FR_KERNAL);
  if ((outv shr PO_BASIC) and 1) = 0 then exit(FR_BASIC);
  if ((outv shr PO_CHAROM) and 1) = 0 then exit(FR_CHAR);
  if ((outv shr PO_ROML) and 1) = 0 then exit(FR_ROML);
  if ((outv shr PO_ROMH) and 1) = 0 then exit(FR_ROMH);
  if ((outv shr PO_CASRAM) and 1) = 0 then exit(FR_RAM);
  Result := FR_OPEN;
end;

{ CIA port inputs: the lines with their pull-ups, the outputs pulling low,
  the keys joining CIA1 PA lines to PB lines (also through each other). }
procedure TC64.FastCiaInputs(i: Integer; var pa, pb: Integer);
var
  lowa, lowb, line, col, k, ga, gb: Integer;
  grp: array[0..15] of Integer;
  glow: array[0..15] of Boolean;
begin
  lowa := cia[i].pa.ddr and not cia[i].pa.reg and $FF;
  lowb := cia[i].pb.ddr and not cia[i].pb.reg and $FF;
  if i = 1 then begin
    { the serial bus: PA4 (CLK out) and PA5 (DATA out) pull the lines low
      through the 7406; PA6/PA7 read them back }
    pa := not lowa and $FF;
    if ((cia[1].pa.ddr and cia[1].pa.reg and $10) <> 0) then pa := pa and not $40;
    if ((cia[1].pa.ddr and cia[1].pa.reg and $20) <> 0) then pa := pa and not $80;
    pb := not lowb and $FF;
    exit;
  end;
  for k := 0 to 15 do grp[k] := k;
  for line := 0 to 7 do
    for col := 0 to 7 do
      if ((key_matrix[line] shr col) and 1) <> 0 then begin
        ga := grp[line];
        gb := grp[8 + col];
        if ga <> gb then
          for k := 0 to 15 do
            if grp[k] = gb then grp[k] := ga;
      end;
  for k := 0 to 15 do glow[k] := False;
  for k := 0 to 7 do begin
    if ((lowa shr k) and 1) <> 0 then glow[grp[k]] := True;
    if ((lowb shr k) and 1) <> 0 then glow[grp[8 + k]] := True;
  end;
  pa := 0;
  pb := 0;
  for k := 0 to 7 do begin
    if not glow[grp[k]] then pa := pa or (1 shl k);
    if not glow[grp[8 + k]] then pb := pb or (1 shl k);
  end;
end;

procedure TC64.FastCycle;
var
  s: TSim;
  pins, a, rw, region, d, reg, pa, pb, outs, i, level: Integer;
  read, io_cycle, res_low, irq_low, restore_lv: Boolean;
  c: TM6526;
begin
  s := sim;
  s.cycle := s.cycle + 1;
  { the lines as the last cycle left them }
  res_low := f_res_low;
  irq_low := vic_irq or cia_irq[0];
  { CPU }
  if res_low then
    cpu_in_reset := True
  else begin
    if cpu_in_reset then begin
      cpu_in_reset := False;
      cpu_pins := cpu_pins or M6502_RES;
      cpu.io_ddr := 0;
      cpu.io_out := 0;
    end;
    pins := cpu_pins and not (M6502_IRQ or M6502_NMI or M6502_RDY);
    if irq_low then pins := pins or M6502_IRQ;
    if f_nmi_low then pins := pins or M6502_NMI;
    if vic_ba_low then pins := pins or M6502_RDY;
    cpu_pins := cpu.Tick(pins);
  end;
  cpu_ticked := not res_low;
  { VIC, phi1 }
  vic_fetches := 0;
  pins := vic.TickFetch(0);                  { calls FastVicFetch }
  vic_ba_low := (pins and M6569_BA) <> 0;
  vic_irq := (pins and M6569_IRQ) <> 0;
  vic_last_tick := s.cycle;
  if vic_ba_low then Inc(ba_streak) else ba_streak := 0;
  vic_steal := vic_ba_low and (ba_streak >= 4);
  if vic_fetches = 0 then begin
    fbus := FastVicRead($3F00 or vic_refresh);
    vic_refresh := (vic_refresh - 1) and $FF;
  end;
  { phi2: the CPU's access, unless the VIC takes it }
  a := M6502_GET_ADDR(cpu_pins);
  read := (cpu_pins and M6502_RW) <> 0;
  region := FR_OPEN;
  io_cycle := False;
  if vic_steal then begin
    if vic_fetches < 2 then fbus := FastVicRead($3FFF);
  end else begin
    region := FastRegion(a, read);
    io_cycle := region = FR_IO;
    if not read then begin
      d := M6502_GET_DATA(cpu_pins);
      if a <= 1 then d := fbus;              { the 6510 doesn't drive the bus for its port }
      fbus := d;
      if region = FR_RAM then
        fram[a] := d
      else if region = FR_IO then begin
        if (a and $0C00) = $0800 then fcolor[a and $3FF] := d and 15;
      end;
    end else begin
      case region of
        FR_RAM: fbus := fram[a];
        FR_BASIC: fbus := fbasic[a and $1FFF];
        FR_KERNAL: fbus := fkernal[a and $1FFF];
        FR_CHAR: fbus := fchar[a and $FFF];
        FR_ROML: if cart.has_roml then fbus := cart.roml[a and $1FFF];
        FR_ROMH: if cart.has_romh then fbus := cart.romh[a and $1FFF];
        FR_IO: if (a and $0C00) = $0800 then fbus := (fbus and $F0) or (fcolor[a and $3FF] and 15);
      end;
    end;
  end;
  { VIC registers }
  pins := 0;
  if io_cycle and ((a and $0C00) = 0) then begin
    pins := M6569_CS or (a and 63);
    if read then pins := pins or M6569_RW
    else pins := (pins and not $FF0000) or (fbus shl 16);
  end;
  pins := vic.TickFinish(pins);
  if (pins and (M6569_CS or M6569_RW)) = (M6569_CS or M6569_RW) then fbus := (pins shr 16) and $FF;
  { CIAs }
  for i := 0 to 1 do begin
    c := cia[i];
    if res_low then begin
      c.Reset;
      cia_irq[i] := False;
      continue;
    end;
    pins := 0;
    if io_cycle and ((a and $0F00) = $0C00 + i * $100) then begin
      pins := M6526_CS or (a and 15);
      if read then pins := pins or M6526_RW;
      pins := M6526_SET_DATA(pins, fbus);
      FastCiaInputs(i, pa, pb);
    end else begin
      pa := c.pa.inp;
      pb := c.pb.inp;
    end;
    if f_tod then pins := pins or M6526_TOD;
    if (c.ta.cr and (1 shl 6)) <> 0 then begin
      if c.sp_sp_out then pins := pins or M6526_SP;
      if c.sp_cnt_out then pins := pins or M6526_CNT;
    end else
      pins := pins or M6526_SP or M6526_CNT;
    pins := c.Tick(pins, pa, pb);
    outs := Ord(c.pc_low) or (Ord(c.sp_sp_out) shl 1) or (Ord(c.sp_cnt_out) shl 2) or (((c.ta.cr shr 6) and 1) shl 3);
    cia_outs[i] := outs;
    cia_irq[i] := (pins and M6526_IRQ) <> 0;
    if (pins and (M6526_CS or M6526_RW)) = (M6526_CS or M6526_RW) then fbus := M6526_GET_DATA(pins);
  end;
  { SID }
  if res_low then
    sid.Reset
  else begin
    pins := 0;
    if io_cycle and ((a and $0C00) = $0400) then begin
      pins := M6581_CS or (a and 31);
      if read then pins := pins or M6581_RW;
      pins := (pins and not $FF0000) or (fbus shl 16);
    end;
    if (pins and (M6581_CS or M6581_RW)) = (M6581_CS or M6581_RW) then begin
      reg := pins and M6581_ADDR_MASK;
      if reg = M6581_POT_X then sid.pot[0] := PotValue(s.parts[p_sid].Pin(24));
      if reg = M6581_POT_Y then sid.pot[1] := PotValue(s.parts[p_sid].Pin(23));
    end;
    pins := sid.Tick(pins);
    if (pins and (M6581_CS or M6581_RW)) = (M6581_CS or M6581_RW) then fbus := (pins shr 16) and $FF;
  end;
  { the 556: RESTORE -> NMI pulse, power-on reset; the lines for the next cycle }
  restore_lv := ((key_matrix[8] and 1) = 0) or not FastRestoreWired;
  if restore_lv then level := 1 else level := 0;
  if (ft556.restore_prev <> 0) and (level = 0) then ft556.nmi_until := s.cycle + C64_NMI_CYCLES;
  ft556.restore_prev := level;
  ft556.last_cycle := s.cycle;
  f_res_low := reset_button or (s.cycle < ft556.reset_until);
  f_nmi_low := cia_irq[1] or (s.cycle < ft556.nmi_until);
  { TOD: the 50 Hz from the mains }
  if s.cycle >= mains_next then begin
    mains_edges := Trunc((s.cycle * (2 * MAINS_HZ)) / C64_FREQ) + 1;
    mains_next := Trunc((mains_edges * C64_FREQ + 2 * MAINS_HZ - 1) / (2 * MAINS_HZ));
  end;
  f_tod := (Trunc(mains_edges - 1) and 1) <> 0;
  { the CPU samples the bus }
  if cpu_ticked then begin
    pins := cpu_pins;
    if read then pins := M6502_SET_DATA(pins, fbus);
    if M6510_CHECK_IO(pins) then pins := cpu.IORQ(pins);
    cpu_pins := pins;
  end;
end;

function TC64.FastRestoreWired: Boolean;
begin
  Result := True;
end;

procedure TC64.RunFast(cycles: Integer);
var i: Integer;
begin
  for i := 0 to cycles - 1 do FastCycle;
  cpu_running := True;
  vic_running := True;
end;


{ ------------------------------------------------------------------------ }
{ Keyboard, PRG loading, video                                              }
{ ------------------------------------------------------------------------ }

procedure TC64.SyncKeys;
var
  line, m: Integer;
  changed: Boolean;
begin
  changed := False;
  for line := 0 to 8 do begin
    m := kbd.scanout_column_masks[line];
    if m <> key_matrix[line] then begin
      key_matrix[line] := m;
      changed := True;
    end;
  end;
  if changed then Rebuild;
end;

procedure TC64.KeyDown(key: Integer);
begin
  kbd.KeyDown(key);
  SyncKeys;
end;

procedure TC64.KeyUp(key: Integer);
begin
  kbd.KeyUp(key);
  SyncKeys;
end;

procedure TC64.FrameUpdate(us: Integer);
begin
  kbd.Update(us);
  SyncKeys;
end;

procedure TC64.RamPoke(addr, v: Integer);
var cell, bit: Integer;
begin
  cell := ((addr and $FF) shl 8) or ((addr shr 8) and $FF);
  for bit := 0 to 7 do
    TDramState(sim.parts[dram_part[bit]].state).cells[cell] := (v shr bit) and 1;
end;

function TC64.LoadPrg(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean;
const
  PTRS: array[0..4] of Integer = [$2D, $2F, $31, $33, $AE];
var
  n, start, en, i: Integer;
begin
  n := bytes.length;
  if n < 3 then begin
    msg := path + ' is too short';
    exit(False);
  end;
  start := bytes[0] or (bytes[1] shl 8);
  en := (start + n - 2) and $FFFF;
  for i := 2 to n - 1 do RamPoke((start + i - 2) and $FFFF, bytes[i]);
  for i := 0 to 4 do begin
    RamPoke(PTRS[i], en and $FF);
    RamPoke(PTRS[i] + 1, en shr 8);
  end;
  if run then begin
    RamPoke($277, Ord('R'));
    RamPoke($278, Ord('U'));
    RamPoke($279, Ord('N'));
    RamPoke($27A, 13);
    RamPoke($C6, 4);
  end;
  msg := 'loaded ' + path + ' at $' + HexW(start) + '-$' + HexW(en);
  Result := True;
end;

procedure TC64.Monitor(outp: TUInts);
var
  s: TSim;
  vp: TPart;
  sync, color: Boolean;
  i, c, g, r, gg, bl, y: Integer;
begin
  s := sim;
  vp := s.parts[p_vic];
  sync := (not vp.dead) and (s.cycle - vic_last_tick < 20000) and (s.pins[vp.Pin(15)].fault = PF_OK);
  color := (seen_color[0] <> seen_color[1]) and (s.pins[vp.Pin(14)].fault = PF_OK);
  for i := 0 to C64_SCREEN_W * C64_SCREEN_H - 1 do begin
    c := screen[i];
    if not sync then begin
      monitor_seed := U32(Imul(monitor_seed, 1103515245) + 12345);
      g := (monitor_seed shr 24) and $FF;
      g := (g * 3) div 4;
      outp[i] := U32($FF000000 or (g shl 16) or (g shl 8) or g);
    end else if not color then begin
      r := c and $FF;
      gg := (c shr 8) and $FF;
      bl := (c shr 16) and $FF;
      y := (r * 299 + gg * 587 + bl * 114) div 1000;
      outp[i] := U32($FF000000 or (y shl 16) or (y shl 8) or y);
    end else
      outp[i] := c;
  end;
end;

function TC64.FileKind(const path: String; const head: TBytes; var info: String; var bad: Boolean): TFileKind;
var
  lp, nm, kind: String;
  prg, crt, exrom, game: Boolean;
  n, addr, typ, i: Integer;
begin
  lp := LowerCase(path);
  prg := (Length(lp) > 4) and (Copy(lp, Length(lp) - 3, 4) = '.prg');
  crt := (Length(lp) > 4) and (Copy(lp, Length(lp) - 3, 4) = '.crt');
  if not prg and not crt then exit(nil);
  if head <> nil then n := head.length else n := 0;
  bad := False;
  if prg then begin
    if n >= 2 then addr := head[0] or (head[1] shl 8) else addr := 0;
    if n < 3 then info := 'too short for a program'
    else if addr = $0801 then info := 'BASIC program'
    else info := 'machine code at $' + HexW(addr);
    exit(TFileKind.Create('program', 'Load and RUN', True, -1));
  end;
  Result := TFileKind.Create('cartridge', 'Plug the cartridge', False, ROLE_ROM);
  nm := '';
  if n >= 64 then for i := 0 to 15 do nm := nm + Chr(head[i]);
  if (n < 64) or (nm <> 'C64 CARTRIDGE   ') then begin
    info := 'not a cartridge file';
    bad := True;
    exit;
  end;
  nm := '';
  for i := 0 to 31 do if head[$20 + i] <> 0 then nm := nm + Chr(head[$20 + i]) else break;
  nm := TrimRight(nm);
  typ := (head[$16] shl 8) or head[$17];
  exrom := head[$18] = 0;
  game := head[$19] = 0;
  if typ <> 0 then begin
    info := 'type ' + IntToStr(typ) + ' (bank switching): not supported';
    bad := True;
    exit;
  end;
  if game and not exrom then kind := 'Ultimax'
  else if game then kind := '16K'
  else if exrom then kind := '8K'
  else kind := 'no ROM lines';
  if nm <> '' then info := kind + ' cartridge: ' + nm else info := kind + ' cartridge';
end;

function TC64.Load(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean;
var lp: String;
begin
  lp := LowerCase(path);
  if (Length(lp) > 4) and (Copy(lp, Length(lp) - 3, 4) = '.crt') then
    Result := InsertCart(path, bytes, msg)
  else begin
    if Fast then FastToChips;
    Result := LoadPrg(path, bytes, run, msg);
    if mode = C64_MODE_FAST then mode := C64_MODE_NONE;   { fast mode reads the new RAM }
  end;
end;

function TC64.CpuStatus: String;
begin
  if not cpu_running then Result := 'stopped (no clock)'
  else if cpu_in_reset then Result := 'in reset'
  else if Fast then Result := 'running (fast mode)'
  else Result := 'running';
end;

function TC64.CpuRegs: String;
var st: String;
begin
  if not cpu_running then st := '(no clock)'
  else if cpu_in_reset then st := '(in reset)'
  else st := '';
  Result := 'PC=' + UpperCase(HexW(cpu.PC)) + ' A=' + UpperCase(HexB(cpu.A)) + ' X=' + UpperCase(HexB(cpu.X)) +
    ' Y=' + UpperCase(HexB(cpu.Y)) + ' S=' + UpperCase(HexB(cpu.S)) + ' ' + st;
end;

{ Read the text on the monitor picture, like a person would: match every
  8x8 cell of the text area against the character ROM glyphs. }
function MatchCell(const px: TUInts; x0, y0, bg: Integer; const chrom: TBytes): Integer;
var
  cell: array[0..7] of Integer;
  x, y, c: Integer;
  same, inv: Boolean;
begin
  for y := 0 to 7 do begin
    cell[y] := 0;
    for x := 0 to 7 do
      if px[(y0 + y) * C64_SCREEN_W + x0 + x] <> bg then cell[y] := cell[y] or ($80 shr x);
  end;
  for c := 0 to 127 do begin
    same := True;
    inv := True;
    for y := 0 to 7 do begin
      if chrom[c * 8 + y] <> cell[y] then same := False;
      if ((not chrom[c * 8 + y]) and $FF) <> cell[y] then inv := False;
    end;
    if same or inv then exit(c);
  end;
  Result := -1;
end;

function TC64.ScreenText: array of String;
const
  TAB = '[\]^_ !"#$%&''()*+,-./0123456789:;<=>?';
var
  px: TUInts;
  chrom: TBytes;
  border, ox, oy, x, y, bg, row, col, code: Integer;
  line, ch: String;
begin
  px := NewUInts(C64_SCREEN_W * C64_SCREEN_H);
  Monitor(px);
  chrom := ROM_DUMP_C64_CHAR_BIN;
  border := px[2 * C64_SCREEN_W + 2];
  ox := 32; oy := 36;
  for y := 20 to 59 do
    if px[y * C64_SCREEN_W + C64_SCREEN_W div 2] <> border then begin oy := y; break; end;
  for x := 16 to 59 do
    if px[(oy + 100) * C64_SCREEN_W + x] <> border then begin ox := x; break; end;
  bg := px[(oy + 100) * C64_SCREEN_W + ox + 4];
  for row := 0 to 24 do begin
    line := '';
    for col := 0 to 39 do begin
      code := MatchCell(px, ox + col * 8, oy + row * 8, bg, chrom);
      ch := '?';
      if code = 0 then ch := '@'
      else if (code >= 1) and (code <= 26) then ch := Chr(Ord('A') + code - 1)
      else if (code >= 27) and (code < 64) then ch := Copy(TAB, code - 27 + 1, 1)
      else if code >= 64 then ch := '#';
      line := line + ch;
    end;
    Result.Add(TrimRight(line));
  end;
end;

{ ------------------------------------------------------------------------ }
{ The host keyboard (DOM KeyboardEvent.key names)                           }
{ ------------------------------------------------------------------------ }

function TC64.HostKey(const code: String): Integer;
begin
  if code = 'Enter' then exit(C64_KEY_RETURN);
  if code = 'Backspace' then exit(C64_KEY_DEL);
  if code = 'ArrowLeft' then exit(C64_KEY_CSRLEFT);
  if code = 'ArrowRight' then exit(C64_KEY_CSRRIGHT);
  if code = 'ArrowUp' then exit(C64_KEY_CSRUP);
  if code = 'ArrowDown' then exit(C64_KEY_CSRDOWN);
  if code = 'Home' then exit(C64_KEY_HOME);
  if code = 'Escape' then exit(C64_KEY_STOP);
  if code = 'PageUp' then exit(C64_KEY_RESTORE);
  if code = 'Tab' then exit(C64_KEY_CTRL);
  if code = 'Alt' then exit(C64_KEY_CBM);
  if code = 'F1' then exit(C64_KEY_F1);
  if code = 'F3' then exit(C64_KEY_F3);
  if code = 'F5' then exit(C64_KEY_F5);
  if code = 'F7' then exit(C64_KEY_F7);
  Result := 0;
end;

function TC64.HostChar(c: Integer): Integer;
begin
  if (c >= Ord('a')) and (c <= Ord('z')) then exit(c - 32);
  if (c >= Ord('A')) and (c <= Ord('Z')) then exit(c + 32);
  Result := c;
end;

{ ------------------------------------------------------------------------ }
{ Cartridges (cart.c)                                                       }
{ ------------------------------------------------------------------------ }

const
  PAD_A: array[0..12] of String = ['Y', 'X', 'W', 'V', 'U', 'T', 'S', 'R', 'P', 'N', 'M', 'L', 'K'];
  PAD_D: array[0..7] of String = ['21', '20', '19', '18', '17', '16', '15', '14'];

function BE16(const b: TBytes; o: Integer): Integer;
begin
  Result := (b[o] shl 8) or b[o + 1];
end;

function BE32(const b: TBytes; o: Integer): Integer;
begin
  Result := U32((b[o] shl 24) or (b[o + 1] shl 16) or (b[o + 2] shl 8) or b[o + 3]);
end;

function CrtLoad(const path: String; const buf: TBytes; var msg: String): TCart;
var
  n, typ, off, len, load, size, m, i, base, data: Integer;
  nm, hdr, fname: String;
  c: TCart;
begin
  Result := nil;
  n := buf.length;
  hdr := '';
  if n >= $40 then for i := 0 to 15 do hdr := hdr + Chr(buf[i]);
  if (n < $40) or (hdr <> 'C64 CARTRIDGE   ') then begin
    msg := path + ' is not a .CRT cartridge';
    exit;
  end;
  typ := BE16(buf, $16);
  nm := '';
  for i := 0 to 31 do if buf[$20 + i] <> 0 then nm := nm + Chr(buf[$20 + i]) else break;
  if typ <> 0 then begin
    msg := nm + ': cartridge type ' + IntToStr(typ) +
      ' has bank switching, not supported (only plain 8K/16K/Ultimax ones)';
    exit;
  end;
  c := TCart.Create;
  c.roml := NewBytes(8192);
  c.romh := NewBytes(8192);
  c.name := nm;
  if c.name = '' then begin
    fname := path;
    while Pos('/', fname) > 0 do fname := Copy(fname, Pos('/', fname) + 1, Length(fname));
    if Pos('.', fname) > 0 then fname := Copy(fname, 1, Pos('.', fname) - 1);
    c.name := fname;
  end;
  c.exrom := buf[$18] = 0;
  c.game := buf[$19] = 0;
  off := BE32(buf, $10);
  while (off + 16 <= n) and (buf[off] = Ord('C')) and (buf[off + 1] = Ord('H')) and
        (buf[off + 2] = Ord('I')) and (buf[off + 3] = Ord('P')) do begin
    len := BE32(buf, off + 4);
    load := BE16(buf, off + 12);
    size := BE16(buf, off + 14);
    if (len < 16) or (off + 16 + size > n) then break;
    data := off + 16;
    if size < 8192 then m := size else m := 8192;
    if (m <> 0) and (load = $8000) then begin
      for i := 0 to 8191 do c.roml[i] := buf[data + (i mod m)];
      c.has_roml := True;
      if size > 8192 then begin
        for i := 0 to 8191 do c.romh[i] := buf[data + 8192 + (i mod (size - 8192))];
        c.has_romh := True;
      end;
    end else if (m <> 0) and ((load = $A000) or (load >= $E000)) then begin
      if load >= $E000 then base := load - $E000 else base := 0;
      for i := 0 to 8191 do c.romh[i] := buf[data + ((i + 8192 - base) mod m)];
      c.has_romh := True;
    end;
    off := off + len;
  end;
  if not c.has_roml and not c.has_romh then begin
    msg := c.name + ': no ROM in the file';
    exit;
  end;
  Result := c;
end;

procedure TC64.Plug(c: TCart);
var
  s: TSim;
  p: TPart;
  k: TCart;
  ins, outs: array of Integer;
  i: Integer;
begin
  s := sim;
  p := s.parts[p_cn6];
  if c <> nil then k := c else k := TCart(p.state);
  if k = nil then exit;
  ins.Add(k.pin_roml);
  ins.Add(k.pin_romh);
  for i := 0 to 12 do ins.Add(k.pin_a[i]);
  for i := 0 to 7 do outs.Add(k.pin_d[i]);
  outs.Add(k.pin_game);
  outs.Add(k.pin_exrom);
  for i := 0 to ins.High do
    if ins[i] >= 0 then begin
      if c <> nil then s.pins[ins[i]].flags := PIN_IN or PIN_SENSE else s.pins[ins[i]].flags := 0;
    end;
  for i := 0 to outs.High do
    if outs[i] >= 0 then begin
      if c <> nil then s.pins[outs[i]].flags := PIN_OUT else s.pins[outs[i]].flags := 0;
      s.pins[outs[i]].drive := DRV_Z;
    end;
  if (c <> nil) and c.game and (c.pin_game >= 0) then s.pins[c.pin_game].drive := MkDrive(S_SUPPLY, 0);
  if (c <> nil) and c.exrom and (c.pin_exrom >= 0) then s.pins[c.pin_exrom].drive := MkDrive(S_SUPPLY, 0);
  p.state := c;
  if c <> nil then p.eval := cart_eval else p.eval := nil;
  cart := c;
end;

function TC64.InsertCart(const path: String; const bytes: TBytes; var msg: String): Boolean;
var
  c: TCart;
  i: Integer;
  kind: String;
begin
  c := CrtLoad(path, bytes, msg);
  if c = nil then exit(False);
  c.pin_roml := PadPin(p_cn6, '11');
  c.pin_romh := PadPin(p_cn6, 'B');
  c.pin_game := PadPin(p_cn6, '8');
  c.pin_exrom := PadPin(p_cn6, '9');
  for i := 0 to 12 do c.pin_a[i] := PadPin(p_cn6, PAD_A[i]);
  for i := 0 to 7 do c.pin_d[i] := PadPin(p_cn6, PAD_D[i]);
  RemoveCart;
  Plug(c);
  Rebuild;
  PowerOn;
  if c.game and not c.exrom then kind := 'Ultimax' else if c.game then kind := '16K' else kind := '8K';
  msg := 'cartridge "' + c.name + '": ' + kind;
  if c.has_roml then msg := msg + ', ROML';
  if c.has_romh then msg := msg + ', ROMH';
  Result := True;
end;

procedure TC64.RemoveCart;
begin
  if cart = nil then exit;
  Plug(nil);
  cart := nil;
  sim.parts[p_cn6].state := nil;
  sim.parts[p_cn6].eval := nil;
  Rebuild;
  PowerOn;
end;

function TC64.CartName: String;
begin
  if cart <> nil then Result := cart.name else Result := '';
end;

{ ------------------------------------------------------------------------ }
{ The 586220 diagnostic harness (harness.c)                                 }
{ ------------------------------------------------------------------------ }

const
  WIRES: array[0..27] of String = [
    'CN2.B', 'CN2.8', 'CN2.C', 'CN2.H', 'CN2.D', 'CN2.J', 'CN2.E', 'CN2.K', 'CN2.F', 'CN2.L',
    'CN2.4', 'CN2.6', 'CN2.5', 'CN2.7', 'CN2.9', 'CN2.M', 'CN3.D', 'CN3.F', 'CN4.1', 'CN4.5',
    'CN4.3', 'CN4.4', 'CN1.7', 'CN1.15', 'CN1.6', 'CN1.14', 'CN1.9', 'CN1.20'];
  JOY_SWITCHES: array[0..9] of String = [
    'CN9.1', 'CN8.1', 'CN9.2', 'CN8.2', 'CN9.3', 'CN8.3', 'CN9.4', 'CN8.4', 'CN9.6', 'CN8.6'];
  KBD_SWITCHES: array[0..9] of String = [
    'CN1.13', 'CN1.12', 'CN1.19', 'CN1.11', 'CN1.18', 'CN1.10', 'CN1.17', 'CN1.5', 'CN1.16', 'CN1.8'];
  PADDLES: array[0..3] of String = ['CN9.9', 'CN9.5', 'CN8.9', 'CN8.5'];

function TC64.FindPin(const spec: String): Integer;
var dot: Integer;
begin
  dot := Pos('.', spec);
  Result := PadPin(sim.FindPart(Copy(spec, 1, dot - 1)), Copy(spec, dot + 1, Length(spec)));
end;

procedure TC64.JoinPins(const a, c: String);
var pa, pc: Integer;
begin
  pa := FindPin(a);
  pc := FindPin(c);
  if (pa < 0) or (pc < 0) or (sim.pins[pa].fault = PF_OPEN) or (sim.pins[pc].fault = PF_OPEN) then exit;
  sim.Join(sim.pins[pa].net, sim.pins[pc].net);
end;

procedure TC64.HarnessJoins;
var i: Integer;
begin
  if not harness then exit;
  for i := 0 to 13 do JoinPins(WIRES[i * 2], WIRES[i * 2 + 1]);
  if harness_write then
    for i := 0 to 4 do JoinPins(JOY_SWITCHES[i * 2], JOY_SWITCHES[i * 2 + 1])
  else
    for i := 0 to 4 do JoinPins(KBD_SWITCHES[i * 2], KBD_SWITCHES[i * 2 + 1]);
end;

procedure TC64.HarnessTick;
var
  s: TSim;
  m, w: TPin;
  motor, write: Boolean;
begin
  if (not harness) or (pin_hmotor < 0) or (pin_hwrite < 0) then exit;
  s := sim;
  m := s.pins[pin_hmotor];
  motor := (m.root >= 0) and (m.fault <> PF_OPEN) and (s.nets[m.root].strength <> S_Z) and (s.nets[m.root].level <> 0);
  s.Drive(pin_hwrite, MkDrive(S_SUPPLY, Ord(motor)));
  w := s.pins[pin_hwrite];
  write := (w.root >= 0) and (s.nets[w.root].level <> 0);
  if write <> harness_write then begin
    harness_write := write;
    Rebuild;
  end;
end;

procedure TC64.SetHarness(plugged: Boolean);
begin
  if harness = plugged then exit;
  pin_hmotor := FindPin('CN3.C');
  pin_hwrite := FindPin('CN3.E');
  harness := plugged;
  harness_write := False;
  if pin_hwrite >= 0 then sim.pins[pin_hwrite].drive := DRV_Z;
  Rebuild;
  PowerOn;
end;

const
  U28_SW: array[0..11] of Integer = [1, 2, 13, 4, 3, 5, 8, 9, 6, 11, 10, 12];

function ParseFloatPrefix(const s: String): Float;
var
  i: Integer;
  t: String;
begin
  t := '';
  i := 1;
  while (i <= Length(s)) and (((s[i] >= '0') and (s[i] <= '9')) or (s[i] = '.') or (s[i] = '-') or (s[i] = '+')) do begin
    t := t + s[i];
    Inc(i);
  end;
  if (t = '') or (t = '.') then exit(0);
  Result := StrToFloat(t);
end;

function TC64.PotValue(sid_pin: Integer): Integer;
var
  s: TSim;
  root, k, i, a, c, ctl, other, pp, u28: Integer;
  cap, g, v: Float;
  n: TNet;
  p: TPart;
  pc: TPin;
begin
  s := sim;
  root := s.pins[sid_pin].root;
  if (root < 0) or (s.pins[sid_pin].fault = PF_OPEN) then exit(255);
  cap := 0;
  n := s.nets[root];
  for k := 0 to n.pins.High do begin
    p := s.parts[s.pins[n.pins[k]].part];
    if (Copy(p.ref, 1, 1) <> 'C') or (p.kind <> PART_OTHER) then continue;
    v := ParseFloatPrefix(p.name);
    if Pos('pF', p.name) > 0 then cap := cap + v * 1e-12
    else cap := cap + v * 1e-6;
  end;
  g := 0;
  u28 := p_u28;
  if harness and (u28 >= 0) and not s.parts[u28].dead then begin
    p := s.parts[u28];
    for i := 0 to 3 do begin
      a := p.Pin(U28_SW[i * 3]);
      c := p.Pin(U28_SW[i * 3 + 1]);
      ctl := p.Pin(U28_SW[i * 3 + 2]);
      if (a < 0) or (c < 0) or (ctl < 0) then continue;
      pc := s.pins[ctl];
      if (pc.root < 0) or (s.nets[pc.root].level = 0) or (s.nets[pc.root].strength = S_Z) then continue;
      if s.pins[a].root = root then other := c
      else if s.pins[c].root = root then other := a
      else other := -1;
      if (other < 0) or (s.pins[a].fault = PF_OPEN) or (s.pins[c].fault = PF_OPEN) then continue;
      for k := 0 to 3 do begin
        pp := FindPin(PADDLES[k]);
        if (pp >= 0) and (s.pins[pp].root = s.pins[other].root) and (s.pins[pp].fault <> PF_OPEN) then
          g := g + 1.0 / PADDLE_OHM;
      end;
    end;
  end;
  if (g <= 0) or (cap <= 0) then exit(255);
  v := 255.0 * (cap / g) / 4.7e-4;
  if v > 255 then Result := 255 else Result := Trunc(v);
end;

end.
