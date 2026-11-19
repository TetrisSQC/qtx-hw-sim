unit hwsim.lc80;

{ The LC-80 (Lerncomputer G-5403.500, DDR 1984), simulated at its pins.

  The board: a U880 (Z80) at about 900 kHz from an RC oscillator, 2K ROM
  (two U505D, 1K each, at $0000 and $0800), 1K RAM (two U214D = 2114 at
  $2000), a system PIO (port A: the segments through eight PNP drivers,
  port B: tape in/out and the six digit drivers, which are also the
  keyboard's scan lines), a user PIO (port B4-B7: the keyboard's return
  lines) and a CTC; the interrupt chain is CTC -> user PIO -> system PIO.
  The netlist was captured by hand from the schematic scan, see
  data/lc80/lc80.pins.txt for what was read how.

  HOW TIME MOVES

  Every half period of the oscillator is simulated: the machine drives the
  oscillator's node (OSC, the input of the Schmitt trigger D211 13), the
  board settles, the CPU sees an edge of its clock if the inverters made
  one, then the clocked parts (RAM writes, PIO/CTC registers) commit.

  FAST MODE: while the board is healthy (keys and RES are fine) and no pin
  is on the scope, the same Z80 core runs on the ROMs, the 2114s' own cells
  and the same PIO/CTC states and register functions, without the netlist.

  The display is multiplexed by the monitor program: the machine samples
  which segment anodes are driven high while their digit's cathode is
  pulled low and draws each segment as bright as it was lit. }

interface

uses
  qtx.sysutils,
  hwsim.util, hwsim.boarddata, hwsim.bd_lc80, hwsim.roms, hwsim.sim, hwsim.parts, hwsim.board,
  hwsim.z80, hwsim.ttl;

const
  LC80_FREQ = 900000;
  LC80_CYCLES_PER_FRAME = 18000;       { 50 frames a second }
  LC80_SCREEN_W = 480;
  LC80_SCREEN_H = 150;
  LC80_DIGITS = 6;
  LC80_MODE_NONE = 0; LC80_MODE_PINS = 1; LC80_MODE_FAST = 2;

  { keys that are not characters (KeyDown codes) }
  LC80_KEY_ADR = 1001; LC80_KEY_DAT = 1002; LC80_KEY_ST = 1003; LC80_KEY_LD = 1004;
  LC80_KEY_EX = 1005; LC80_KEY_RES = 1006; LC80_KEY_NMI = 1007;

type
  TPioPort = class
  public
    mode: Integer;        { 0 output, 1 input, 2 bidirectional, 3 bit control }
    outr: Integer;
    iosel: Integer;       { mode 3: 1 = input }
    vector: Integer;
    icw: Integer;
    mask: Integer;
    expect_io, expect_mask: Boolean;
  end;

  { What the Z80 peripherals watch on the bus (both the PIO and the CTC). }
  TZ80Bus = class
  public
    iorq_prev, rd_prev, m1_prev: Integer;
    wr_on, inta_on, m1_seen_rdio: Boolean;
    wr_data, wr_reg: Integer;
    fetch, op_prev: Integer;
    m1_rd: Boolean;         { this M1 cycle was an opcode fetch (RD low) }
    reti: Boolean;          { set for one commit after ED 4D was fetched }
  end;

  TPioState = class
  public
    port: array[0..1] of TPioPort;
    bus: TZ80Bus;
    data_cache: Integer;
    constructor Create;
  end;

  TCtcState = class
  public
    ctl, tc, cnt, presc: array[0..3] of Integer;
    running, want_tc, pending, inservice: array[0..3] of Boolean;
    vector: Integer;
    clk_prev: Integer;
    bus: TZ80Bus;
    data_cache: Integer;
    constructor Create;
  end;

  TLc80 = class(TBoard)
  private
    function PinOf(const ref: String; number: Integer): Integer;
    procedure SetElements(const ref: String; const pairs: array of Integer; typ: Integer);
    procedure CpuEdge(c: Integer);
    procedure Step;
    procedure SampleDisplay;
    function Healthy: Boolean;
    procedure ChipsToFast;
    procedure FastToChips;
    function FastPortPins(user: Boolean; port: Integer): Integer;
    function FastDigitLow(d: Integer): Boolean;
    function FastSegOn(k: Integer): Boolean;
    function FastTapeOut: Integer;
    function FastIoRead(a: Integer): Integer;
    procedure FastIoWrite(a, d: Integer);
    procedure FastEdge(level: Integer);
    procedure FastSample;
    procedure RunFast(cycles: Integer);
  protected
    procedure Describe; override;
    procedure MachineCreate; override;
    procedure MachinePowerOn; override;
    procedure MachineJoins; override;
  public
    cpu: TZ80;
    cpu_pins, cpu_ab: Integer;
    p_cpu: Integer;
    cache_addr, cache_data: Integer;
    pin_osc, pin_clk, pin_reskey, pin_nmikey, net_gnd: Integer;
    osc, clk: Integer;
    in_reset, cpu_running: Boolean;
    last_edge: Float;
    reset_until: Float;
    changes_seen: Integer;
    { display: nets of the 8 segment lines and 6 cathodes, light per frame }
    net_seg: array[0..7] of Integer;
    net_cath: array[0..LC80_DIGITS - 1] of Integer;
    lit: array[0..LC80_DIGITS * 8 - 1] of Integer;
    bright: array[0..LC80_DIGITS * 8 - 1] of Float;
    samples, sample_div: Integer;
    halt_lit, halt_samples: Integer;
    halt_bright: Float;
    net_haltled, net_tapeout: Integer;
    { keyboard }
    net_dig: array[0..5] of Integer;
    net_kin: array[0..3] of Integer;
    key_down: array of Boolean;      { by LC80_KEYS index, then RES, NMI }
    key_hold: array of Integer;      { frames a key stays down at least (the monitor scans slowly) }
    key_up_wanted: array of Boolean;
    { fast mode }
    mode: Integer;
    healthy_epoch: Integer;
    healthy_cache: Boolean;
    f_ramlo, f_ramhi, f_rom1, f_rom2: TBytes;
    f_spio, f_upio: TPioState;
    f_ctc: TCtcState;
    f_data, f_fetch, f_op_prev, f_wr_addr, f_wr_data: Integer;
    f_res, f_nmi, f_wr_on, f_inta_on, f_m1_rd: Boolean;

    constructor Create; override;
    function Find(const ref: String): TPart;
    procedure Run(cycles: Integer); override;
    procedure KeyDown(key: Integer); override;
    procedure KeyUp(key: Integer); override;
    procedure FrameUpdate(us: Integer); override;
    function HostKey(const code: String): Integer; override;
    function HostChar(c: Integer): Integer; override;
    procedure Monitor(outp: TUInts); override;
    function FileKind(const path: String; const head: TBytes; var info: String; var bad: Boolean): TFileKind; override;
    function Load(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean; override;
    function CpuStatus: String; override;
    function CpuRegs: String; override;
    function ScreenText: array of String; override;
    function DisplayText: String;
    function RamPoke(addr, v: Integer): Boolean;
    function Fast: Boolean;
  end;

var
  PT_U880, PT_U505, PT_PIO, PT_CTC, PT_VQE23: TPartType;

implementation

const
  Z80_ADDR_PINS: array[0..15] of Integer = [30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 1, 2, 3, 4, 5];
  Z80_DATA_PINS: array[0..7] of Integer = [14, 15, 12, 8, 7, 9, 10, 13];
  Z_CLK = 6; Z_INT = 16; Z_NMI = 17; Z_HALT = 18; Z_MREQ = 19; Z_IORQ = 20; Z_RD = 21; Z_WR = 22;
  Z_BUSAK = 23; Z_WAIT = 24; Z_BUSRQ = 25; Z_RESET = 26; Z_M1 = 27; Z_RFSH = 28;

  U505_ADDR_PINS: array[0..9] of Integer = [8, 7, 6, 5, 4, 3, 2, 1, 23, 22];
  U505_DATA_PINS: array[0..7] of Integer = [9, 10, 11, 13, 14, 15, 16, 17];

  PIO_DATA_PINS: array[0..7] of Integer = [19, 20, 1, 40, 39, 38, 3, 2];
  PIO_A_PINS: array[0..7] of Integer = [15, 14, 13, 12, 10, 9, 8, 7];
  PIO_B_PINS: array[0..7] of Integer = [27, 28, 29, 30, 31, 32, 33, 34];
  P_BA = 6; P_CD = 5; P_CE = 4; P_M1 = 37; P_IORQ = 36; P_RD = 35; P_INT = 23; P_IEI = 24; P_IEO = 22;

  CTC_DATA_PINS: array[0..7] of Integer = [25, 26, 27, 28, 1, 2, 3, 4];
  CTC_ZC_PINS: array[0..2] of Integer = [7, 8, 9];
  C_CS0 = 18; C_CS1 = 19; C_CE = 16; C_M1 = 14; C_IORQ = 10; C_RD = 6; C_CLK = 15; C_RESET = 17;
  C_INT = 12; C_IEI = 13; C_IEO = 11;

  GK_LC_CPU = GK_MACHINE + 400;
  GK_LC_ROM = GK_MACHINE + 401;
  GK_LC_PIO = GK_MACHINE + 402;
  GK_LC_CTC = GK_MACHINE + 403;

  { segment line n (PIO A bit n) -> segment: b f a g dp c e d; as bits a..g, dp = 0..7 }
  SEG_OF_LINE: array[0..7] of Integer = [1, 5, 0, 6, 7, 2, 4, 3];

{ ------------------------------------------------------------------------ }
{ Z80 (D201)                                                                }
{ ------------------------------------------------------------------------ }

procedure cpu_eval(s: TSim; p: TPart);
var
  m: TLc80;
  pins: Integer;
  aoe: Boolean;
begin
  m := TLc80(p.user);
  pins := m.cpu_pins;
  aoe := (pins and Z80_AOE) <> 0;
  m.cache_addr := part_drive_bus_cached(s, p, Z80_ADDR_PINS, 16, m.cpu_ab, aoe, m.cache_addr);
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
  m.cache_data := part_drive_bus_cached(s, p, Z80_DATA_PINS, 8, (pins shr 16) and $FF,
    (pins and Z80_DOE) <> 0, m.cache_data);
end;

procedure TLc80.CpuEdge(c: Integer);
var
  s: TSim;
  p: TPart;
  pins, outp: Integer;
begin
  s := sim;
  p := s.parts[p_cpu];
  last_edge := s.cycle;
  if p.dead then exit;
  if c <> 0 then pins := Z80_CLK else pins := 0;
  pins := pins or (part_read_bus(s, p, GK_LC_CPU, Z80_DATA_PINS, 8) shl 16);
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
{ U505D: 1K mask ROM (D202, D203)                                           }
{ ------------------------------------------------------------------------ }

procedure u505_eval(s: TSim; p: TPart);
var
  st: TRomState;
  sel: Boolean;
  addr, v: Integer;
begin
  st := TRomState(p.state);
  sel := s.PinIn(p, 20) = 0;
  if sel then addr := part_read_bus(s, p, GK_LC_ROM, U505_ADDR_PINS, 10) else addr := 0;
  if (st.data <> nil) and (addr < st.data.Length) then v := st.data[addr] else v := $FF;
  st.out_cache := part_drive_bus_cached(s, p, U505_DATA_PINS, 8, v, sel, st.out_cache);
end;

function NewRom: TObject;
begin
  Result := TRomState.Create;
end;

{ ------------------------------------------------------------------------ }
{ What a Z80 peripheral sees of the bus cycles                              }
{ ------------------------------------------------------------------------ }

{ Called at every commit. ce: the chip's select. Returns via the fields:
  wr_on/wr_data/wr_reg (an I/O write to this chip just ended: perform it),
  inta_end (an interrupt acknowledge just ended), reti, m1 reset. }
procedure bus_watch(s: TSim; p: TPart; b: TZ80Bus; pin_m1, pin_iorq, pin_rd: Integer;
  ce: Boolean; reg: Integer; const data_pins: array of Integer; key: Integer;
  var wr_done, inta_done, m1_reset: Boolean);
var
  m1, iorq, rd, d: Integer;
begin
  wr_done := False;
  inta_done := False;
  m1_reset := False;
  b.reti := False;
  m1 := s.PinIn(p, pin_m1);
  iorq := s.PinIn(p, pin_iorq);
  rd := s.PinIn(p, pin_rd);
  { I/O write to this chip: /IORQ low, /RD and /M1 high; the data is taken
    while /IORQ is low, the register written when it rises. }
  if (iorq = 0) and (rd <> 0) and (m1 <> 0) and ce then begin
    b.wr_on := True;
    b.wr_data := part_read_bus(s, p, key, data_pins, 8);
    b.wr_reg := reg;
  end;
  if (iorq = 0) and (m1 = 0) then b.inta_on := True;
  if (b.iorq_prev = 0) and (iorq <> 0) then begin
    if b.wr_on then wr_done := True;
    if b.inta_on then inta_done := True;
  end;
  if iorq <> 0 then begin
    if not wr_done then b.wr_on := False;
    if not inta_done then b.inta_on := False;
  end;
  { opcode fetches: the byte on the bus while /RD is low in an M1 cycle;
    looked at when /M1 rises (/RD rises with it) }
  if (m1 = 0) and (rd = 0) and (iorq <> 0) then begin
    b.fetch := part_read_bus(s, p, key, data_pins, 8);
    b.m1_rd := True;
  end;
  if (m1 = 0) and ((rd = 0) or (iorq = 0)) then b.m1_seen_rdio := True;
  if (b.m1_prev = 0) and (m1 <> 0) then begin
    if b.m1_rd then begin
      if (b.op_prev = $ED) and (b.fetch = $4D) then b.reti := True;
      b.op_prev := b.fetch;
    end;
    b.m1_rd := False;
    { M1 without RD or IORQ: the PIO's reset }
    if not b.m1_seen_rdio then m1_reset := True;
    b.m1_seen_rdio := False;
  end;
  b.iorq_prev := iorq;
  b.rd_prev := rd;
  b.m1_prev := m1;
end;

{ ------------------------------------------------------------------------ }
{ UD855D: Z80 PIO (D206, D207)                                              }
{ ------------------------------------------------------------------------ }

constructor TPioState.Create;
var i: Integer;
begin
  inherited Create;
  for i := 0 to 1 do begin
    port[i] := TPioPort.Create;
    port[i].mode := 1;
    port[i].iosel := $FF;
  end;
  bus := TZ80Bus.Create;
  bus.iorq_prev := 1; bus.rd_prev := 1; bus.m1_prev := 1;
  bus.m1_seen_rdio := True;
  data_cache := -1;
end;

function NewPio: TObject;
begin
  Result := TPioState.Create;
end;

procedure pio_reset_ports(st: TPioState);
var i: Integer;
begin
  for i := 0 to 1 do begin
    st.port[i].mode := 1;
    st.port[i].outr := 0;
    st.port[i].iosel := $FF;
    st.port[i].icw := 0;
    st.port[i].mask := $FF;
    st.port[i].expect_io := False;
    st.port[i].expect_mask := False;
  end;
end;

{ the pins of a port as they read }
function pio_pins(s: TSim; p: TPart; i: Integer): Integer;
var k, v: Integer;
begin
  v := 0;
  for k := 0 to 7 do
    if i = 0 then v := v or (s.PinIn(p, PIO_A_PINS[k]) shl k)
    else v := v or (s.PinIn(p, PIO_B_PINS[k]) shl k);
  Result := v;
end;

{ a register write: reg bit 0 = B/A, bit 1 = C/D }
procedure pio_write(st: TPioState; reg, d: Integer);
var pt: TPioPort;
begin
  pt := st.port[reg and 1];
  if (reg and 2) = 0 then
    pt.outr := d                                     { data }
  else if pt.expect_io then begin
    pt.iosel := d; pt.expect_io := False;
  end else if pt.expect_mask then begin
    pt.mask := d; pt.expect_mask := False;
  end else if (d and 1) = 0 then
    pt.vector := d
  else if (d and $0F) = $0F then begin
    pt.mode := (d shr 6) and 3;
    if pt.mode = 3 then pt.expect_io := True;
  end else if (d and $0F) = $07 then begin
    pt.icw := d;
    if (d and $10) <> 0 then pt.expect_mask := True;
  end else if (d and $0F) = $03 then
    pt.icw := (pt.icw and $7F) or (d and $80);
end;

{ a data register read, given the port's pins }
function pio_read(st: TPioState; reg, pins: Integer): Integer;
var pt: TPioPort;
begin
  Result := $FF;
  if reg >= 2 then exit;
  pt := st.port[reg];
  case pt.mode of
    0: Result := pt.outr;
    3: Result := (pins and pt.iosel) or (pt.outr and not pt.iosel);
  else
    Result := pins;
  end;
  Result := Result and $FF;
end;

procedure pio_eval(s: TSim; p: TPart);
var
  st: TPioState;
  i, k, outmask, v, reg: Integer;
  pt: TPioPort;
  rd: Boolean;
begin
  st := TPioState(p.state);
  for i := 0 to 1 do begin
    pt := st.port[i];
    case pt.mode of
      0, 2: outmask := $FF;
      3: outmask := (not pt.iosel) and $FF;
    else
      outmask := 0;
    end;
    for k := 0 to 7 do begin
      if ((outmask shr k) and 1) <> 0 then v := DRV_BIT((pt.outr shr k) and 1) else v := DRV_Z;
      if i = 0 then s.PinOut(p, PIO_A_PINS[k], v) else s.PinOut(p, PIO_B_PINS[k], v);
    end;
  end;
  { register read: /CE, /IORQ, /RD low, /M1 high }
  rd := (s.PinIn(p, P_CE) = 0) and (s.PinIn(p, P_IORQ) = 0) and (s.PinIn(p, P_RD) = 0) and (s.PinIn(p, P_M1) <> 0);
  v := $FF;
  if rd then begin
    reg := s.PinIn(p, P_BA) or (s.PinIn(p, P_CD) shl 1);
    if reg < 2 then v := pio_read(st, reg, pio_pins(s, p, reg));
  end;
  st.data_cache := part_drive_bus_cached(s, p, PIO_DATA_PINS, 8, v and $FF, rd, st.data_cache);
  { no interrupts from the PIOs in this model: /INT off, IEO follows IEI }
  s.PinOut(p, P_INT, DRV_Z);
  s.PinOut(p, P_IEO, DRV_BIT(s.PinIn(p, P_IEI) <> 0));
end;

procedure pio_commit(s: TSim; p: TPart);
var
  st: TPioState;
  wr_done, inta_done, m1_reset: Boolean;
  reg, d: Integer;
  pt: TPioPort;
begin
  st := TPioState(p.state);
  reg := s.PinIn(p, P_BA) or (s.PinIn(p, P_CD) shl 1);
  bus_watch(s, p, st.bus, P_M1, P_IORQ, P_RD, s.PinIn(p, P_CE) = 0, reg, PIO_DATA_PINS, GK_LC_PIO,
    wr_done, inta_done, m1_reset);
  if m1_reset then begin
    pio_reset_ports(st);
    s.MarkDirty(p.index);
  end;
  if not wr_done then exit;
  pio_write(st, st.bus.wr_reg, st.bus.wr_data);
  st.bus.wr_on := False;
  s.MarkDirty(p.index);
end;

{ ------------------------------------------------------------------------ }
{ UD857D: Z80 CTC (D208)                                                    }
{ ------------------------------------------------------------------------ }

constructor TCtcState.Create;
var i: Integer;
begin
  inherited Create;
  for i := 0 to 3 do begin ctl[i] := 0; tc[i] := 256; cnt[i] := 0; presc[i] := 0; end;
  bus := TZ80Bus.Create;
  bus.iorq_prev := 1; bus.rd_prev := 1; bus.m1_prev := 1;
  bus.m1_seen_rdio := True;
  clk_prev := 0;
  data_cache := -1;
end;

function NewCtc: TObject;
begin
  Result := TCtcState.Create;
end;

procedure ctc_reset(st: TCtcState);
var i: Integer;
begin
  for i := 0 to 3 do begin
    st.ctl[i] := 0;
    st.running[i] := False;
    st.want_tc[i] := False;
    st.pending[i] := False;
    st.inservice[i] := False;
  end;
end;

{ the channel whose interrupt is acknowledged next (-1: none) }
function ctc_top(st: TCtcState): Integer;
var i: Integer;
begin
  for i := 0 to 3 do begin
    if st.inservice[i] then exit(-1);
    if st.pending[i] then exit(i);
  end;
  Result := -1;
end;

procedure ctc_eval(s: TSim; p: TPart);
var
  st: TCtcState;
  rd, inta, iei, busy: Boolean;
  v, ch, i: Integer;
begin
  st := TCtcState(p.state);
  iei := s.PinIn(p, C_IEI) <> 0;
  rd := (s.PinIn(p, C_CE) = 0) and (s.PinIn(p, C_IORQ) = 0) and (s.PinIn(p, C_RD) = 0) and (s.PinIn(p, C_M1) <> 0);
  inta := (s.PinIn(p, C_IORQ) = 0) and (s.PinIn(p, C_M1) = 0) and iei and (ctc_top(st) >= 0);
  v := $FF;
  if rd then begin
    ch := s.PinIn(p, C_CS0) or (s.PinIn(p, C_CS1) shl 1);
    v := st.cnt[ch] and $FF;
  end else if inta then
    v := (st.vector and $F8) or (ctc_top(st) shl 1);
  st.data_cache := part_drive_bus_cached(s, p, CTC_DATA_PINS, 8, v, rd or inta, st.data_cache);
  if iei and (ctc_top(st) >= 0) then s.PinOut(p, C_INT, DRV_LO) else s.PinOut(p, C_INT, DRV_Z);
  busy := False;
  for i := 0 to 3 do
    if st.inservice[i] or st.pending[i] then busy := True;
  s.PinOut(p, C_IEO, DRV_BIT(iei and not busy));
  for i := 0 to 2 do s.PinOut(p, CTC_ZC_PINS[i], DRV_LO);
end;

procedure ctc_write(st: TCtcState; ch, d: Integer);
begin
  if st.want_tc[ch] then begin
    if d = 0 then st.tc[ch] := 256 else st.tc[ch] := d;
    st.cnt[ch] := st.tc[ch];
    st.want_tc[ch] := False;
    { a timer without a trigger runs from now on; a counter counts edges }
    st.running[ch] := ((st.ctl[ch] and $40) = 0) and ((st.ctl[ch] and $08) = 0);
    st.presc[ch] := 0;
  end else if (d and 1) <> 0 then begin
    st.ctl[ch] := d;
    if (d and 2) <> 0 then st.running[ch] := False;
    if (d and 4) <> 0 then st.want_tc[ch] := True;
    if (d and $80) = 0 then st.pending[ch] := False;
  end else if ch = 0 then
    st.vector := d and $F8;
end;

{ an interrupt acknowledge ended: the top channel goes in service }
function ctc_ack(st: TCtcState): Boolean;
var i: Integer;
begin
  i := ctc_top(st);
  Result := i >= 0;
  if Result then begin st.pending[i] := False; st.inservice[i] := True; end;
end;

{ RETI seen with IEI high: the channel in service is done }
function ctc_reti(st: TCtcState): Boolean;
var i: Integer;
begin
  Result := False;
  for i := 0 to 3 do
    if st.inservice[i] then begin st.inservice[i] := False; exit(True); end;
end;

{ a rising edge of the system clock: the timers through their prescalers }
function ctc_clock(st: TCtcState): Boolean;
var i, pre: Integer;
begin
  Result := False;
  for i := 0 to 3 do begin
    if not st.running[i] then continue;
    if (st.ctl[i] and $20) <> 0 then pre := 256 else pre := 16;
    Inc(st.presc[i]);
    if st.presc[i] < pre then continue;
    st.presc[i] := 0;
    Dec(st.cnt[i]);
    if st.cnt[i] <= 0 then begin
      st.cnt[i] := st.tc[i];
      if (st.ctl[i] and $80) <> 0 then begin st.pending[i] := True; Result := True; end;
    end;
  end;
end;

procedure ctc_commit(s: TSim; p: TPart);
var
  st: TCtcState;
  wr_done, inta_done, m1_reset, changed: Boolean;
  ch, d, c, i, pre: Integer;
begin
  st := TCtcState(p.state);
  changed := False;
  if s.PinIn(p, C_RESET) = 0 then begin
    ctc_reset(st);
    s.MarkDirty(p.index);
  end;
  ch := s.PinIn(p, C_CS0) or (s.PinIn(p, C_CS1) shl 1);
  bus_watch(s, p, st.bus, C_M1, C_IORQ, C_RD, s.PinIn(p, C_CE) = 0, ch, CTC_DATA_PINS, GK_LC_CTC,
    wr_done, inta_done, m1_reset);
  if inta_done and (s.PinIn(p, C_IEI) <> 0) then
    if ctc_ack(st) then changed := True;
  if st.bus.reti and (s.PinIn(p, C_IEI) <> 0) then
    if ctc_reti(st) then changed := True;
  if wr_done then begin
    ctc_write(st, st.bus.wr_reg, st.bus.wr_data);
    st.bus.wr_on := False;
    changed := True;
  end;
  { the timers: the system clock through the prescaler (16 or 256) }
  c := s.PinIn(p, C_CLK);
  if (st.clk_prev = 0) and (c <> 0) then
    if ctc_clock(st) then changed := True;
  st.clk_prev := c;
  if changed then s.MarkDirty(p.index);
end;

{ ------------------------------------------------------------------------ }
{ The board                                                                 }
{ ------------------------------------------------------------------------ }

function lc_rail_of(const name: String): Integer;
begin
  if name = 'GND' then exit(RAIL_GND);
  if name = '+5V' then exit(RAIL_VCC);
  Result := RAIL_NONE;
end;

function lc_kind_of(const ref: String): Integer;
var c0: String;
begin
  c0 := Copy(ref, 1, 1);
  if (c0 = 'D') or (c0 = 'H') then exit(PART_IC);
  if c0 = 'R' then exit(PART_RESISTOR);
  Result := PART_OTHER;
end;

constructor TLc80.Create;
begin
  inherited Create;
  cpu := TZ80.Create;
end;

procedure TLc80.Describe;
var i: Integer;
begin
  id := 'lc80';
  name := 'LC-80';
  board_name := 'LC-80 (G-5403.500, schematic scan)';
  data := BoardLc80;
  simdesc.rail_of := lc_rail_of;
  simdesc.kind_of := lc_kind_of;
  simdesc.cycles_per_us := LC80_FREQ / 1000000;
  cpu_hz := LC80_FREQ;
  cycles_per_frame := LC80_CYCLES_PER_FRAME;
  steps_per_cycle := 2;
  screen_w := LC80_SCREEN_W;
  screen_h := LC80_SCREEN_H;
  ready_cycles := 0.5 * LC80_FREQ;
  audio_rate := 22050;
  link_resistors := 'R275 R276 R297 R298 R299 R300 R301 R302 R303 R304 R305 R306 R307 R308 R309 R310 R311 R312';
  crystals := '';
  keys_help := 'Keys: 0-9 A-F + -   F1=ADR F2=DAT F3=ST F4=LD Enter=EX   Esc=RES F5=NMI';
  reset_help := 'Hold to press RES';
  files_title := 'Open a program: .bin (to $2000) or Intel .hex';
  cart_slot := '';
  part_defs.Add(TPartDef.Create('D201', PT_U880, 'CPU', ROLE_CHIP, 'U880D'#10'Z80', 'U880D CPU (Z80), about 900 kHz'));
  part_defs.Add(TPartDef.Create('D202', PT_U505, 'ROM1', ROLE_ROM, 'U505D'#10'$0000', 'U505D 1K ROM, monitor part 1, $0000-$07FF'));
  part_defs.Add(TPartDef.Create('D203', PT_U505, 'ROM2', ROLE_ROM, 'U505D'#10'$0800', 'U505D 1K ROM, monitor part 2, $0800-$0FFF'));
  part_defs.Add(TPartDef.Create('D204', PT_2114, 'RAM', ROLE_RAM, 'U214D'#10'D0-D3', 'U214D (2114) 1K x 4 RAM, D0-D3 of $2000-$23FF'));
  part_defs.Add(TPartDef.Create('D205', PT_2114, 'RAM', ROLE_RAM, 'U214D'#10'D4-D7', 'U214D (2114) 1K x 4 RAM, D4-D7 of $2000-$23FF'));
  part_defs.Add(TPartDef.Create('D206', PT_PIO, 'PIO', ROLE_CHIP, 'UD855D'#10'SYS PIO', 'UD855D (Z80 PIO), system PIO $F4-$F7: segments, digits, tape'));
  part_defs.Add(TPartDef.Create('D207', PT_PIO, 'PIO', ROLE_CHIP, 'UD855D'#10'USER PIO', 'UD855D (Z80 PIO), user PIO $F8-$FB: keyboard lines B4-B7'));
  part_defs.Add(TPartDef.Create('D208', PT_CTC, 'CTC', ROLE_CHIP, 'UD857D'#10'CTC', 'UD857D (Z80 CTC), $EC-$EF'));
  part_defs.Add(TPartDef.Create('D209', PT_74LS138, 'DEC', ROLE_GLUE, 'DS8205'#10'ROM sel', 'DS8205 (74LS138): ROM selects, 2K each from $0000'));
  part_defs.Add(TPartDef.Create('D210', PT_74LS138, 'DEC', ROLE_GLUE, 'DS8205'#10'RAM sel', 'DS8205 (74LS138): RAM selects, 1K each from $2000'));
  part_defs.Add(TPartDef.Create('D211', PT_74LS04, 'INV', ROLE_GLUE, 'DL014'#10'7414', 'DL014 (7414) Schmitt inverters: clock, RES and NMI keys'));
  part_defs.Add(TPartDef.Create('D212', PT_74LS00x, 'NAND', ROLE_GLUE, 'DL000'#10'7400', 'DL000 (7400) NAND: /RESET, /NMI, M1 for PIO/CTC'));
  for i := 0 to 2 do
    part_defs.Add(TPartDef.Create('H32' + IntToStr(2 + i), PT_VQE23, 'LED', ROLE_OTHER, 'VQE23',
      'VQE23 two-digit 7-segment display, digits ' + IntToStr(i * 2 + 1) + ' and ' + IntToStr(i * 2 + 2)));
end;

function TLc80.Find(const ref: String): TPart;
var i: Integer;
begin
  i := sim.FindPart(ref);
  if i >= 0 then Result := sim.parts[i] else Result := nil;
end;

function TLc80.PinOf(const ref: String; number: Integer): Integer;
var p: TPart;
begin
  p := Find(ref);
  if p <> nil then Result := p.Pin(number) else Result := -1;
end;

{ A transistor as an element (pins: base, collector). }
procedure TLc80.SetElements(const ref: String; const pairs: array of Integer; typ: Integer);
var
  p: TPart;
  st: TResistorState;
  e: TElement;
begin
  p := Find(ref);
  if p = nil then exit;
  st := TResistorState.Create;
  p.state := st;
  p.kind := PART_RESISTOR;
  e := TElement.Create;
  e.pin_a := p.Pin(pairs[0]);
  e.pin_b := p.Pin(pairs[1]);
  st.el.Add(e);
  st.nel := 1;
  element_set_type(sim, p, 0, typ);
end;

procedure TLc80.MachineCreate;
var
  s: TSim;
  i: Integer;
  p: TPart;
begin
  s := sim;
  p_cpu := s.FindPart('D201');
  TRomState(Find('D202').state).data := ROM_LC80_D202;
  TRomState(Find('D203').state).data := ROM_LC80_D203;
  for i := 0 to 7 do SetElements('V' + IntToStr(221 + i), [2, 3], EL_PNP);
  { the digit drivers are PNPs too, emitter followers to GND: B low selects }
  for i := 0 to 5 do SetElements('V' + IntToStr(229 + i), [2, 3], EL_PNP_SINK);
  SetElements('V219', [2, 3], EL_PNP);
  pin_osc := PinOf('D211', 13);
  pin_clk := PinOf('D201', Z_CLK);
  pin_reskey := PinOf('D211', 5);
  pin_nmikey := PinOf('D211', 3);
  net_gnd := s.FindNet('GND');
  for i := 0 to 7 do net_seg[i] := s.FindNet('SEG' + IntToStr(i));
  for i := 0 to LC80_DIGITS - 1 do net_cath[i] := s.FindNet('CATH' + IntToStr(i));
  for i := 0 to 5 do net_dig[i] := s.FindNet('DIG' + IntToStr(i));
  for i := 0 to 3 do net_kin[i] := s.FindNet('KIN' + IntToStr(i));
  net_haltled := s.FindNet('HALTLED');
  net_tapeout := s.FindNet('TAPEOUT');
  key_down.SetLength(LC80_KEYS.Length + 2);
  key_hold.SetLength(LC80_KEYS.Length + 2);
  key_up_wanted.SetLength(LC80_KEYS.Length + 2);
  for i := 0 to key_down.High do begin key_down[i] := False; key_hold[i] := 0; key_up_wanted[i] := False; end;
  { the RES key on the connector, for the RESET button }
  pin_reset := pin_reskey;
  pin_reset_gnd := PinOf('D211', 7);
  sample_div := 0;
end;

procedure TLc80.MachinePowerOn;
var i: Integer;
begin
  cpu_pins := cpu.Init;
  cpu_ab := cpu.ab;
  in_reset := True;
  osc := 0;
  clk := 0;
  last_edge := 0;
  { C243 (10 uF) charging through R271 (10k) holds RES low for a while }
  reset_until := 0.1 * LC80_FREQ;
  mode := LC80_MODE_NONE;
  healthy_epoch := -1;
  for i := 0 to lit.High do begin lit[i] := 0; bright[i] := 0; end;
  samples := 0;
  halt_lit := 0; halt_samples := 0; halt_bright := 0;
  if screen <> nil then screen.fill(0);
end;

procedure TLc80.MachineJoins;
var
  s: TSim;
  i: Integer;
begin
  s := sim;
  for i := 0 to LC80_KEYS.High do
    if key_down[i] then
      s.Join(net_dig[LC80_KEYS[i].sysb - 2], net_kin[LC80_KEYS[i].userb - 4]);
  if key_down[LC80_KEYS.Length] then s.Join(s.pins[pin_reskey].net, net_gnd);
  if key_down[LC80_KEYS.Length + 1] then s.Join(s.pins[pin_nmikey].net, net_gnd);
end;

{ ------------------------------------------------------------------------ }
{ Time                                                                      }
{ ------------------------------------------------------------------------ }

function net_high(s: TSim; n: Integer): Boolean;
var r: TNet;
begin
  if n < 0 then exit(False);
  r := s.nets[s.nets[n].root];
  Result := (r.strength <> S_Z) and (r.level <> 0);
end;

function net_low(s: TSim; n: Integer): Boolean;
var r: TNet;
begin
  if n < 0 then exit(False);
  r := s.nets[s.nets[n].root];
  Result := (r.strength <> S_Z) and (r.level = 0);
end;

procedure TLc80.Step;
var
  s: TSim;
  c: Integer;
begin
  s := sim;
  osc := osc xor 1;
  s.Drive(pin_osc, MkDrive(S_STRONG, osc));
  s.Settle;
  c := s.Read(pin_clk);
  if c <> clk then begin
    clk := c;
    CpuEdge(c);
    s.Settle;
  end;
  if s.changes <> changes_seen then begin
    s.Commit;
    s.Settle;
    changes_seen := s.changes;
  end;
  Sample;
end;

procedure TLc80.Run(cycles: Integer);
var
  s: TSim;
  i, want: Integer;
begin
  s := sim;
  if (not force_pins) and (scope_pin < 0) and Healthy then want := LC80_MODE_FAST else want := LC80_MODE_PINS;
  if want <> mode then begin
    if want = LC80_MODE_FAST then ChipsToFast
    else if mode = LC80_MODE_FAST then FastToChips;
    mode := want;
  end;
  if mode = LC80_MODE_FAST then begin RunFast(cycles); exit; end;
  for i := 0 to cycles - 1 do begin
    s.cycle := s.cycle + 1;
    { power-on reset: C243 discharged }
    if s.cycle < reset_until then s.Drive(pin_reskey, DRV_LO) else s.Drive(pin_reskey, DRV_Z);
    Step;
    Step;
    if net_tapeout >= 0 then AudioCycle(Ord(net_high(s, net_tapeout)));
    Inc(sample_div);
    if sample_div >= 4 then begin
      sample_div := 0;
      SampleDisplay;
    end;
  end;
  cpu_running := s.cycle - last_edge < 1000;
end;

{ ------------------------------------------------------------------------ }
{ Fast mode                                                                 }
{ ------------------------------------------------------------------------ }

function TLc80.Fast: Boolean;
begin
  Result := mode = LC80_MODE_FAST;
end;

function TLc80.Healthy: Boolean;
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

procedure TLc80.ChipsToFast;
begin
  f_ramlo := TSramState(Find('D204').state).cells;
  f_ramhi := TSramState(Find('D205').state).cells;
  f_rom1 := TRomState(Find('D202').state).data;
  f_rom2 := TRomState(Find('D203').state).data;
  f_spio := TPioState(Find('D206').state);
  f_upio := TPioState(Find('D207').state);
  f_ctc := TCtcState(Find('D208').state);
  f_data := (sim.busLevelLo shr 16) and $FF;
  f_res := sim.PinIn(sim.parts[p_cpu], Z_RESET) = 0;
  f_wr_on := False;
  f_inta_on := False;
end;

procedure TLc80.FastToChips;
var i: Integer;
begin
  for i := 0 to sim.nparts - 1 do
    if Assigned(sim.parts[i].eval) then sim.MarkDirty(i);
  sim.Settle;
end;

{ the pins of a PIO port as the board makes them read (fast mode) }
function TLc80.FastPortPins(user: Boolean; port: Integer): Integer;
var
  i, u, d: Integer;
begin
  Result := 0;
  if not user then begin
    { system PIO: B0 = TAPE IN (pulled up), the rest are its own outputs }
    if port = 1 then Result := 1;
    exit;
  end;
  if port = 0 then exit;
  { user PIO B4-B7: pulled up, low through a key on a digit line that is low }
  for u := 0 to 3 do begin
    d := 1;
    for i := 0 to LC80_KEYS.High do
      if key_down[i] and (LC80_KEYS[i].userb = 4 + u) and FastDigitLow(LC80_KEYS[i].sysb - 2) then d := 0;
    Result := Result or (d shl (4 + u));
  end;
end;

{ a system PIO B line (digit d = B(2+d)) driven low }
function TLc80.FastDigitLow(d: Integer): Boolean;
var pt: TPioPort;
begin
  pt := f_spio.port[1];
  Result := False;
  if pt.mode = 3 then begin
    if ((pt.iosel shr (2 + d)) and 1) <> 0 then exit;
  end else if pt.mode <> 0 then exit;
  Result := ((pt.outr shr (2 + d)) and 1) = 0;
end;

{ a segment line (system PIO A bit k) high: its PNP's base driven low }
function TLc80.FastSegOn(k: Integer): Boolean;
var pt: TPioPort;
begin
  pt := f_spio.port[0];
  Result := False;
  if pt.mode = 3 then begin
    if ((pt.iosel shr k) and 1) <> 0 then exit;
  end else if pt.mode = 1 then exit;
  Result := ((pt.outr shr k) and 1) = 0;
end;

{ TAPE OUT (system PIO B1), which also drives the speaker through V220 }
function TLc80.FastTapeOut: Integer;
var pt: TPioPort;
begin
  pt := f_spio.port[1];
  Result := 0;
  if (pt.mode = 3) and ((pt.iosel and 2) <> 0) then exit;
  if pt.mode = 1 then exit;
  Result := (pt.outr shr 1) and 1;
end;

function TLc80.FastIoRead(a: Integer): Integer;
var reg: Integer;
begin
  reg := a and 3;
  if (a and 8) = 0 then exit(pio_read(f_spio, reg, FastPortPins(False, reg and 1)));
  if (a and 4) = 0 then exit(pio_read(f_upio, reg, FastPortPins(True, reg and 1)));
  if (a and 16) = 0 then exit(f_ctc.cnt[reg] and $FF);
  Result := f_data;                         { nobody drives the bus }
end;

procedure TLc80.FastIoWrite(a, d: Integer);
var reg: Integer;
begin
  reg := a and 3;
  if (a and 8) = 0 then pio_write(f_spio, reg, d);
  if (a and 4) = 0 then pio_write(f_upio, reg, d);
  if (a and 16) = 0 then ctc_write(f_ctc, reg, d);
end;

{ one edge of the CPU clock: the board answers the bus cycle the CPU put out
  at the last edge, then the CPU ticks }
procedure TLc80.FastEdge(level: Integer);
var
  pins, a, inp, sel: Integer;
begin
  pins := cpu_pins;
  a := cpu_ab;
  if (pins and Z80_MREQ) <> 0 then begin
    if (a and $2000) = 0 then begin
      sel := (a shr 11) and 3;               { D209 }
      if (pins and Z80_RD) <> 0 then begin
        if sel = 0 then f_data := f_rom1[a and $3FF]
        else if sel = 1 then f_data := f_rom2[a and $3FF];
      end;
    end else if ((a shr 10) and 3) = 0 then begin   { D210: RAM1 }
      if (pins and Z80_RD) <> 0 then
        f_data := (f_ramlo[a and $3FF] and 15) or ((f_ramhi[a and $3FF] and 15) shl 4)
      else if (pins and Z80_WR) <> 0 then begin
        f_ramlo[a and $3FF] := (pins shr 16) and 15;
        f_ramhi[a and $3FF] := (pins shr 20) and 15;
      end;
    end;
    { opcode fetches, for RETI }
    if ((pins and Z80_M1) <> 0) and ((pins and Z80_RD) <> 0) then f_fetch := f_data;
  end else if (pins and Z80_IORQ) <> 0 then begin
    if (pins and Z80_M1) <> 0 then begin
      f_inta_on := True;
      if ctc_top(f_ctc) >= 0 then f_data := (f_ctc.vector and $F8) or (ctc_top(f_ctc) shl 1);
    end else if (pins and Z80_RD) <> 0 then
      f_data := FastIoRead(a)
    else if (pins and Z80_WR) <> 0 then begin
      f_wr_on := True;
      f_wr_addr := a;
      f_wr_data := (pins shr 16) and $FF;
    end;
  end;
  if (pins and Z80_DOE) <> 0 then f_data := (pins shr 16) and $FF;
  if level <> 0 then inp := Z80_CLK else inp := 0;
  inp := inp or (f_data shl 16);
  if ctc_top(f_ctc) >= 0 then inp := inp or Z80_INT;
  if f_nmi then inp := inp or Z80_NMI;
  if f_res then inp := inp or Z80_RES;
  in_reset := f_res;
  cpu_pins := cpu.Tick(inp);
  cpu_ab := cpu.ab;
  { what the peripherals do right after the edge (their commit on the board) }
  if (cpu_pins and Z80_IORQ) = 0 then begin
    if f_wr_on then begin FastIoWrite(f_wr_addr, f_wr_data); f_wr_on := False; end;
    if f_inta_on then begin ctc_ack(f_ctc); f_inta_on := False; end;
  end;
  if ((pins and Z80_M1) <> 0) and ((pins and (Z80_MREQ or Z80_RD)) = (Z80_MREQ or Z80_RD)) then f_m1_rd := True;
  if ((pins and Z80_M1) <> 0) and ((cpu_pins and Z80_M1) = 0) then begin
    if f_m1_rd then begin
      if (f_op_prev = $ED) and (f_fetch = $4D) then ctc_reti(f_ctc);
      f_op_prev := f_fetch;
    end;
    f_m1_rd := False;
  end;
end;

procedure TLc80.FastSample;
var d, k: Integer;
begin
  Inc(samples);
  for d := 0 to LC80_DIGITS - 1 do begin
    if not FastDigitLow(d) then continue;
    for k := 0 to 7 do
      if FastSegOn(k) then Inc(lit[d * 8 + SEG_OF_LINE[k]]);
  end;
  Inc(halt_samples);
  if (cpu_pins and Z80_HALT) <> 0 then Inc(halt_lit);
end;

procedure TLc80.RunFast(cycles: Integer);
var
  s: TSim;
  i: Integer;
  res: Boolean;
begin
  s := sim;
  for i := 0 to cycles - 1 do begin
    s.cycle := s.cycle + 1;
    res := (s.cycle < reset_until) or reset_button or key_down[LC80_KEYS.Length];
    if f_res and not res then begin
      { M1 for the PIOs (M1 and RESET) rises without RD or IORQ: their reset }
      pio_reset_ports(f_spio);
      pio_reset_ports(f_upio);
    end;
    f_res := res;
    f_nmi := key_down[LC80_KEYS.Length + 1];
    FastEdge(1);
    if f_res then ctc_reset(f_ctc) else ctc_clock(f_ctc);
    FastEdge(0);
    AudioCycle(FastTapeOut);
    Inc(sample_div);
    if sample_div >= 4 then begin
      sample_div := 0;
      FastSample;
    end;
  end;
  last_edge := s.cycle;
  cpu_running := True;
end;

{ ------------------------------------------------------------------------ }
{ Display, keys                                                             }
{ ------------------------------------------------------------------------ }

procedure TLc80.SampleDisplay;
var
  s: TSim;
  d, k: Integer;
begin
  s := sim;
  Inc(samples);
  for d := 0 to LC80_DIGITS - 1 do begin
    if not net_low(s, net_cath[d]) then continue;
    for k := 0 to 7 do
      if net_high(s, net_seg[k]) then Inc(lit[d * 8 + SEG_OF_LINE[k]]);
  end;
  Inc(halt_samples);
  if net_high(s, net_haltled) then Inc(halt_lit);
end;

procedure TLc80.FrameUpdate(us: Integer);
var
  i: Integer;
  changed: Boolean;
begin
  changed := False;
  for i := 0 to key_hold.High do begin
    if key_hold[i] > 0 then Dec(key_hold[i]);
    if (key_hold[i] = 0) and key_up_wanted[i] and key_down[i] then begin
      key_down[i] := False;
      key_up_wanted[i] := False;
      changed := True;
    end;
  end;
  if changed then Rebuild;
  if samples > 0 then begin
    { a digit is lit 1/6 of the time when the monitor multiplexes }
    for i := 0 to lit.High do begin
      bright[i] := lit[i] * LC80_DIGITS / samples;
      if bright[i] > 1 then bright[i] := 1;
      lit[i] := 0;
    end;
    samples := 0;
  end;
  if halt_samples > 0 then begin
    halt_bright := halt_lit / halt_samples;
    halt_lit := 0;
    halt_samples := 0;
  end;
end;

function TLc80.HostKey(const code: String): Integer;
begin
  if code = 'F1' then exit(LC80_KEY_ADR);
  if code = 'F2' then exit(LC80_KEY_DAT);
  if code = 'F3' then exit(LC80_KEY_ST);
  if code = 'F4' then exit(LC80_KEY_LD);
  if code = 'Enter' then exit(LC80_KEY_EX);
  if code = 'Escape' then exit(LC80_KEY_RES);
  if code = 'F5' then exit(LC80_KEY_NMI);
  Result := 0;
end;

function TLc80.HostChar(c: Integer): Integer;
begin
  if (c >= Ord('a')) and (c <= Ord('f')) then exit(c - 32);
  Result := c;
end;

function key_index(key: Integer): Integer;
var
  i: Integer;
  nm: String;
begin
  case key of
    LC80_KEY_ADR: nm := 'ADR';
    LC80_KEY_DAT: nm := 'DAT';
    LC80_KEY_ST: nm := 'ST';
    LC80_KEY_LD: nm := 'LD';
    LC80_KEY_EX: nm := 'EX';
    LC80_KEY_RES: exit(LC80_KEYS.Length);
    LC80_KEY_NMI: exit(LC80_KEYS.Length + 1);
  else
    if (key > 32) and (key < 127) then nm := Chr(key) else exit(-1);
  end;
  for i := 0 to LC80_KEYS.High do
    if LC80_KEYS[i].name = nm then exit(i);
  Result := -1;
end;

procedure TLc80.KeyDown(key: Integer);
var i: Integer;
begin
  i := key_index(key);
  if i < 0 then exit;
  key_up_wanted[i] := False;
  key_hold[i] := 4;
  if key_down[i] then exit;
  key_down[i] := True;
  Rebuild;
end;

procedure TLc80.KeyUp(key: Integer);
var i: Integer;
begin
  i := key_index(key);
  if (i < 0) or not key_down[i] then exit;
  key_up_wanted[i] := True;
  if key_hold[i] > 0 then exit;
  key_down[i] := False;
  key_up_wanted[i] := False;
  Rebuild;
end;

{ The display: six red 7-segment digits, each segment as bright as it was lit. }
procedure TLc80.Monitor(outp: TUInts);
const
  DW = 56; DH = 100; GAP = 14; T = 8;
var
  i, d, x0, y0, x, y, seg, c: Integer;
  b: Float;

  procedure Box(x1, y1, x2, y2, col: Integer);
  var xx, yy: Integer;
  begin
    for yy := y1 to y2 - 1 do
      for xx := x1 to x2 - 1 do
        if (xx >= 0) and (xx < LC80_SCREEN_W) and (yy >= 0) and (yy < LC80_SCREEN_H) then
          outp[yy * LC80_SCREEN_W + xx] := col;
  end;

  function Col(v: Float): Integer;
  var r, g: Integer;
  begin
    r := 40 + Round(215 * v);
    g := 8 + Round(40 * v);
    Result := U32($FF000000 or (g shl 8) or r);
  end;

begin
  for i := 0 to LC80_SCREEN_W * LC80_SCREEN_H - 1 do outp[i] := U32($FF101010);
  for d := 0 to LC80_DIGITS - 1 do begin
    { cathode 0 (B2) is the rightmost digit }
    x0 := 24 + (LC80_DIGITS - 1 - d) * (DW + GAP) + ((LC80_DIGITS - 1 - d) div 2) * 8;
    y0 := 18;
    for seg := 0 to 7 do begin
      b := bright[d * 8 + seg];
      c := Col(b);
      case seg of
        0: Box(x0 + T, y0, x0 + DW - T, y0 + T, c);                                   { a }
        1: Box(x0 + DW - T, y0 + T, x0 + DW, y0 + DH div 2, c);                       { b }
        2: Box(x0 + DW - T, y0 + DH div 2, x0 + DW, y0 + DH - T, c);                  { c }
        3: Box(x0 + T, y0 + DH - T, x0 + DW - T, y0 + DH, c);                         { d }
        4: Box(x0, y0 + DH div 2, x0 + T, y0 + DH - T, c);                            { e }
        5: Box(x0, y0 + T, x0 + T, y0 + DH div 2, c);                                 { f }
        6: Box(x0 + T, y0 + DH div 2 - T div 2, x0 + DW - T, y0 + DH div 2 + T - T div 2, c);  { g }
        7: Box(x0 + DW + 2, y0 + DH - T, x0 + DW + 2 + T, y0 + DH, c);               { dp }
      end;
    end;
  end;
  { the HALT lamp }
  x := LC80_SCREEN_W - 40;
  y := LC80_SCREEN_H - 22;
  Box(x, y, x + 14, y + 14, Col(halt_bright));
end;

{ The digits as text: a hex digit where the segments make one, '-' for g
  alone, ' ' when dark, '?' otherwise; '.' after a lit point. }
function TLc80.DisplayText: String;
const
  PAT: array[0..30] of Integer = [$3F, $06, $5B, $4F, $66, $6D, $7D, $07, $7F, $6F, $77, $7C, $39, $5E, $79, $71, $40, $00,
    $76, $38, $50, $3E, $73, $54, $5C, $1C, $1E, $78, $6E, $58, $74];
  CH: String = '0123456789ABCDEF- HLrUPnouJtych';
var
  d, k, m, j: Integer;
  c: String;
begin
  Result := '';
  for d := LC80_DIGITS - 1 downto 0 do begin
    m := 0;
    for k := 0 to 6 do
      if bright[d * 8 + k] > 0.3 then m := m or (1 shl k);
    c := '?';
    for j := 0 to PAT.High do
      if PAT[j] = m then c := Copy(CH, j + 1, 1);
    Result := Result + c;
    if bright[d * 8 + 7] > 0.3 then Result := Result + '.';
  end;
end;

function TLc80.ScreenText: array of String;
var
  i, k, st_i: Integer;
  st: TPioState;
  line: String;
begin
  Result.Add('[' + DisplayText + ']');
  line := 'lines on (PIO A bit order, left to right):';
  for i := LC80_DIGITS - 1 downto 0 do begin
    k := 0;
    for st_i := 0 to 7 do
      if bright[i * 8 + SEG_OF_LINE[st_i]] > 0.3 then k := k or (1 shl st_i);
    line := line + ' ' + HexB(k);
  end;
  Result.Add(line);
  for i := 0 to 1 do begin
    st := TPioState(Find('D20' + IntToStr(6 + i)).state);
    line := 'D20' + IntToStr(6 + i) + ':';
    for k := 0 to 1 do
      line := line + ' ' + Copy('AB', k + 1, 1) + ' mode ' + IntToStr(st.port[k].mode) + ' out ' + HexB(st.port[k].outr) +
        ' io ' + HexB(st.port[k].iosel);
    Result.Add(line);
  end;
end;

{ ------------------------------------------------------------------------ }
{ Programs                                                                  }
{ ------------------------------------------------------------------------ }

{ A byte into the RAM (the two 2114s' cells: they are the RAM in both modes).
  False: not RAM ($2000-$23FF; A12, A14, A15 are not decoded). }
function TLc80.RamPoke(addr, v: Integer): Boolean;
var a: Integer;
begin
  Result := ((addr and $2000) <> 0) and (((addr shr 10) and 3) = 0);
  if not Result then exit;
  a := addr and $3FF;
  TSramState(Find('D204').state).cells[a] := v and 15;
  TSramState(Find('D205').state).cells[a] := (v shr 4) and 15;
  sim.MarkDirty(sim.FindPart('D204'));
  sim.MarkDirty(sim.FindPart('D205'));
end;

function HexVal(const t: String; pos, n: Integer): Integer;
var
  i, c: Integer;
begin
  Result := 0;
  for i := 0 to n - 1 do begin
    c := Ord(t[pos + i]);
    if (c >= 48) and (c <= 57) then c := c - 48
    else if (c >= 65) and (c <= 70) then c := c - 55
    else if (c >= 97) and (c <= 102) then c := c - 87
    else exit(-1);
    Result := Result * 16 + c;
  end;
end;

function TLc80.FileKind(const path: String; const head: TBytes; var info: String; var bad: Boolean): TFileKind;
var lp: String;
begin
  lp := LowerCase(path);
  bad := False;
  if Pos('.hex', lp) > 0 then begin
    info := 'Intel HEX';
    exit(TFileKind.Create('program', 'Load into RAM', True, ROLE_RAM));
  end;
  if Pos('.bin', lp) > 0 then begin
    info := 'binary at $2000';
    if (head <> nil) and (head.Length > 1024) then begin
      info := IntToStr(head.Length) + ' bytes: the RAM is 1K ($2000-$23FF)';
      bad := True;
    end;
    exit(TFileKind.Create('program', 'Load into RAM at $2000', True, ROLE_RAM));
  end;
  Result := nil;
end;

function TLc80.Load(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean;
var
  t, line: String;
  i, n, len, addr, typ, lo, hi, k, v, outside: Integer;
begin
  Result := False;
  lo := $FFFF; hi := -1; outside := 0;
  if Pos('.hex', LowerCase(path)) > 0 then begin
    t := '';
    for i := 0 to bytes.Length - 1 do t := t + Chr(bytes[i]);
    t := StrReplace(t, #13, '');
    for line in t.Split(#10) do begin
      line := Trim(line);
      if (Length(line) < 11) or (line[1] <> ':') then continue;
      len := HexVal(line, 2, 2);
      addr := HexVal(line, 4, 4);
      typ := HexVal(line, 8, 2);
      if (len < 0) or (addr < 0) or (typ < 0) then begin msg := path + ': not Intel HEX'; exit; end;
      if typ = 1 then break;
      if typ <> 0 then continue;
      for k := 0 to len - 1 do begin
        v := HexVal(line, 10 + k * 2, 2);
        if v < 0 then begin msg := path + ': bad hex digit'; exit; end;
        if RamPoke(addr + k, v) then begin
          if addr + k < lo then lo := addr + k;
          if addr + k > hi then hi := addr + k;
        end else Inc(outside);
      end;
    end;
  end else begin
    n := bytes.Length;
    if n > 1024 then begin msg := path + ': ' + IntToStr(n) + ' bytes, the RAM is 1K'; exit; end;
    for i := 0 to n - 1 do RamPoke($2000 + i, bytes[i]);
    lo := $2000; hi := $2000 + n - 1;
  end;
  if hi < 0 then begin msg := path + ': nothing for the RAM ($2000-$23FF)'; exit; end;
  msg := path + ' loaded at $' + UpperCase(HexW(lo)) + '-$' + UpperCase(HexW(hi)) +
    ': ADR ' + UpperCase(HexW(lo)) + ', EX runs it';
  if outside > 0 then msg := msg + ' (' + IntToStr(outside) + ' bytes outside the RAM left out)';
  Result := True;
end;

function TLc80.CpuStatus: String;
begin
  if not cpu_running then Result := 'stopped (no clock)'
  else if in_reset then Result := 'in reset'
  else if Fast then Result := 'running (fast mode)'
  else Result := 'running';
end;

function TLc80.CpuRegs: String;
begin
  Result := 'PC=' + UpperCase(HexW(cpu.PC)) + ' AF=' + UpperCase(HexW(cpu.AF)) + ' BC=' + UpperCase(HexW(cpu.BC)) +
    ' DE=' + UpperCase(HexW(cpu.DE)) + ' HL=' + UpperCase(HexW(cpu.HL)) + ' SP=' + UpperCase(HexW(cpu.SP)) +
    ' IR=' + UpperCase(HexW(cpu.IR)) + ' IM' + IntToStr(cpu.im);
end;

initialization
  PT_U880 := TPartType.Create('U880D',
    'A11:O A12:O A13:O A14:O A15:O CLK:m D4:B D3:B D5:B D6:B VCC:P D2:B D7:B D0:B D1:B /INT:m /NMI:m ' +
    '/HALT:O /MREQ:O /IORQ:O /RD:O /WR:O /BUSAK:O /WAIT:m /BUSRQ:m /RESET:m /M1:O /RFSH:O GND:P A0:O ' +
    'A1:O A2:O A3:O A4:O A5:O A6:O A7:O A8:O A9:O A10:O', cpu_eval, nil, nil, nil, TECH_NMOS);
  PT_U505 := TPartType.Create('U505D',
    'A7:M A6:M A5:M A4:M A3:M A2:M A1:M A0:M D0:O D1:O D2:O GND:P D3:O D4:O D5:O D6:O D7:O PGM:m A10:m ' +
    '/CS:M VCC:P A9:M A8:M VCC:P', u505_eval, nil, nil, NewRom, TECH_NMOS);
  PT_PIO := TPartType.Create('UD855D',
    'D2:B D7:B D6:B /CE:M C/D:M B/A:M A7:X A6:X A5:X A4:X GND:P A3:X A2:X A1:X A0:X /ASTB:m ARDY:O ' +
    'D0:B D1:B /BSTB:m BRDY:O IEO:O /INT:O IEI:M CLK:m VCC:P B0:X B1:X B2:X B3:X B4:X B5:X B6:X B7:X ' +
    '/RD:M /IORQ:M /M1:M D5:B D4:B D3:B', pio_eval, pio_commit, nil, NewPio, TECH_NMOS);
  PT_CTC := TPartType.Create('UD857D',
    'D4:B D5:B D6:B D7:B GND:P /RD:M ZC0:O ZC1:O ZC2:O /IORQ:M IEO:O /INT:O IEI:M /M1:M CLK:m /CE:M ' +
    '/RESET:m CS0:M CS1:M TRG3:m TRG2:m TRG1:m TRG0:m VCC:P D0:B D1:B D2:B D3:B', ctc_eval, ctc_commit, nil,
    NewCtc, TECH_NMOS);
  PT_VQE23 := TPartType.Create('VQE23',
    'C1:O E1:O D1:O K1:O K2:O D2:O E2:O C2:O DP2:O G2:O A2:O F2:O B2:O B1:O F1:O A1:O G1:O DP1:O',
    nil, nil, nil, nil, TECH_NMOS);
end.
