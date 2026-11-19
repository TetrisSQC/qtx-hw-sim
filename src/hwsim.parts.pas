unit hwsim.parts;

{ Part types: pin descriptions and pin-level models of the glue logic and
  memories (port of parts.h / parts.c). The big chips (6510, VIC-II, CIAs,
  SID, Z80, ULA) are wrapped by the machines around their cores.

  Every model follows the same pattern:
    eval()   reads input pins and drives output pins, no other side effects.
    commit() is called after the board settled, at every bus step: latches
             and memories store what they see (clock edges).
  Pin numbers always follow the chip datasheet. }

interface

uses
  hwsim.util, hwsim.sim;

type
  TStateFactory = function: TObject;

  TPartType = class
  public
    name: String;     { label drawn on the chip, e.g. "74LS08" }
    { Pins in order 1..N as "NAME:K", K = kind:
        T  TTL input, re-evaluate on change (floating reads 1)
        M  MOS input, re-evaluate on change
        m  MOS input sampled only at clock edges / by the chip core
        O  output (or analog pin not simulated)
        B  bidirectional MOS pin, sampled by the core
        X  bidirectional pin that re-evaluates the part when it changes
        P  power pin
        S  analog switch terminal (4066) }
    pins: String;
    eval, commit, reset: TPartFn;
    factory: TStateFactory;
    tech: Integer;
    edge_commit: Boolean;   { commit reacts only to input changes (no time, no noise) }
    edge_pins: array of Integer;   { the inputs it reacts to (empty: all) }
    constructor Create(const aname, apins: String; aeval, acommit, areset: TPartFn;
      afactory: TStateFactory; atech: Integer);
  end;

  TRomState = class
  public
    data: TBytes;
    out_cache: Integer;
  end;

  TDramState = class
  public
    cells: TBytes;
    row, col: Integer;
    ras_prev, cas_prev: Integer;
    out_en, out_bit: Integer;
    constructor Create;
  end;

  TSramState = class
  public
    cells: TBytes;
    out_cache: Integer;
    constructor Create;
  end;

  TTimer556State = class
  public
    reset_until: Float;
    nmi_until: Float;
    restore_net: Integer;
    restore_prev: Integer;
    last_cycle: Float;
  end;

  TVcoState = class
  public
    crystal_ok: Boolean;
  end;

  TLs373State = class
  public
    latch: Integer;
    out_cache: Integer;
  end;

const
  { Resistors, links and diodes: what each element does electrically. }
  EL_PULLUP = 0; EL_PULLDOWN = 1; EL_LINK = 2; EL_ANALOG = 3; EL_SERIES = 4;
  EL_DIODE = 5; EL_DELAY = 6; EL_NPN = 7; EL_FOLLOWER = 8;
  EL_PNP = 9;      { pin_a base, pin_b collector, emitter on +5V: base low pulls the collector high }
  EL_PNP_SINK = 10;  { pin_a base, pin_b emitter, collector on GND: base low pulls the emitter low }

type
  TElement = class
  public
    pin_a, pin_b, typ: Integer;
  end;

  TResistorState = class
  public
    el: array of TElement;
    nel: Integer;
    delayed: array[0..15] of Integer;
  end;

const
  { 82S100 PLA inputs (bit PI_x set when that input pin is high) and outputs. }
  PI_A13 = 0; PI_A14 = 1; PI_A15 = 2; PI_VA14 = 3; PI_CHAREN = 4; PI_HIRAM = 5;
  PI_LORAM = 6; PI_CAS = 7; PI_VA12 = 8; PI_VA13 = 9; PI_GAME = 10; PI_EXROM = 11;
  PI_RW = 12; PI_AEC = 13; PI_BA = 14; PI_A12 = 15; PLA_NIN = 16;
  PO_ROMH = 0; PO_ROML = 1; PO_IO = 2; PO_GRW = 3; PO_CHAROM = 4; PO_KERNAL = 5;
  PO_BASIC = 6; PO_CASRAM = 7; PLA_NOUT = 8;

  PLA_IN_PIN: array[0..15] of Integer = [2, 3, 4, 5, 6, 7, 8, 9, 20, 21, 22, 23, 24, 25, 26, 27];
  PLA_OUT_PIN: array[0..7] of Integer = [10, 11, 12, 13, 15, 16, 17, 18];

type
  TPlaTerm = class
  public
    mask, value: Integer;   { true when (inputs and mask) = value }
    outputs: Integer;       { bit PO_x set: the term feeds that output }
  end;

var
  pla_terms: array of TPlaTerm;
  pla_nterms: Integer;

  PT_6510, PT_6569, PT_6526, PT_6581: TPartType;
  PT_82S100, PT_2364, PT_2332, PT_2114, PT_4164: TPartType;
  PT_74LS08, PT_7406, PT_74LS139, PT_74LS257, PT_74LS258: TPartType;
  PT_74LS373, PT_4066, PT_556, PT_74LS629: TPartType;
  PT_74LS74, PT_74LS193, PT_MC4044: TPartType;
  PT_4116, PT_4532, PT_23128, PT_74LS157, PT_74LS32, PT_74LS00, PT_LM1889: TPartType;

function pla_read_inputs(s: TSim; p: TPart): Integer;
function pla_logic(inputs: Integer): Integer;

{ Attach a type to a part: pin names/flags, functions, state. }
procedure part_set_type(s: TSim; part: Integer; t: TPartType);
{ Read/drive a group of pins as a binary number (pins[0] is bit 0). }
function part_read_bus(s: TSim; p: TPart; key: Integer; const pins: array of Integer; n: Integer): Integer;
procedure part_drive_bus(s: TSim; p: TPart; const pins: array of Integer; n, v: Integer; enable: Boolean);
{ Same, but does nothing if value and enable are the same as the last time
  this cache was used. Returns the new cache value. }
function part_drive_bus_cached(s: TSim; p: TPart; const pins: array of Integer; n, v: Integer;
  enable: Boolean; cache: Integer): Integer;

procedure resistor_eval(s: TSim; p: TPart);
procedure resistor_commit(s: TSim; p: TPart);
procedure element_set_type(s: TSim; p: TPart; index, typ: Integer);

{ Group keys for part_read_bus (any unique value per pin array). }
const
  GK_ROM_ADDR = 1;
  GK_SRAM_ADDR = 2;
  GK_SRAM_IO = 3;
  GK_DRAM_ADDR = 4;
  GK_LS373_D = 5;
  GK_ROM23128_ADDR = 6;
  GK_MACHINE = 100;   { machines use keys from here }

implementation

constructor TPartType.Create(const aname, apins: String; aeval, acommit, areset: TPartFn;
  afactory: TStateFactory; atech: Integer);
begin
  inherited Create;
  name := aname;
  pins := apins;
  eval := aeval;
  commit := acommit;
  reset := areset;
  factory := afactory;
  tech := atech;
end;

constructor TDramState.Create;
begin
  inherited Create;
  cells := NewBytes(65536);
end;

constructor TSramState.Create;
begin
  inherited Create;
  cells := NewBytes(1024);
end;

{ ------------------------------------------------------------------------ }
{ Setup                                                                     }
{ ------------------------------------------------------------------------ }

procedure part_set_type(s: TSim; part: Integer; t: TPartType);
var
  p: TPart;
  toks: array of String;
  tok, nm: String;
  i, number, flags, colon, pi: Integer;
  k: String;
begin
  p := s.parts[part];
  p.name := t.name;
  p.eval := t.eval;
  p.commit := t.commit;
  p.edge_commit := t.edge_commit;
  p.edge_pins := t.edge_pins;
  p.reset := t.reset;
  p.tech := t.tech;
  if Assigned(t.factory) then p.state := t.factory();
  toks := t.pins.Split(' ');
  number := 1;
  for i := 0 to toks.High do begin
    tok := toks[i];
    if tok = '' then continue;
    colon := Pos(':', tok);
    flags := 0;
    nm := tok;
    if colon > 0 then begin
      nm := Copy(tok, 1, colon - 1);
      k := Copy(tok, colon + 1, 1);
      if k = 'T' then flags := PIN_TTL or PIN_SENSE or PIN_IN
      else if k = 'M' then flags := PIN_SENSE or PIN_IN
      else if k = 'm' then flags := PIN_IN
      else if k = 'O' then flags := PIN_OUT
      else if k = 'B' then flags := PIN_IN or PIN_OUT
      else if k = 'X' then flags := PIN_SENSE or PIN_IN or PIN_OUT
      else if k = 'P' then flags := PIN_POWER
      else if k = 'S' then flags := PIN_SENSE or PIN_SWITCH or PIN_IN or PIN_OUT;
    end;
    if number < 48 then begin
      pi := p.pinmap[number];
      if pi >= 0 then begin
        s.pin_names[pi] := nm;
        s.pins[pi].flags := flags;
      end;
    end;
    Inc(number);
  end;
end;

function part_read_bus(s: TSim; p: TPart; key: Integer; const pins: array of Integer; n: Integer): Integer;
begin
  Result := s.ReadGroup(p, key, pins, n);
end;

procedure part_drive_bus(s: TSim; p: TPart; const pins: array of Integer; n, v: Integer; enable: Boolean);
var i: Integer;
begin
  for i := 0 to n - 1 do
    if enable then s.Drive(p.pinmap[pins[i]], DRV_BIT((v shr i) and 1))
    else s.Drive(p.pinmap[pins[i]], DRV_Z);
end;

function part_drive_bus_cached(s: TSim; p: TPart; const pins: array of Integer; n, v: Integer;
  enable: Boolean; cache: Integer): Integer;
begin
  if enable then Result := v + 1 else Result := -1;
  if cache = Result then exit;
  part_drive_bus(s, p, pins, n, v, enable);
end;

{ ------------------------------------------------------------------------ }
{ The big chips: only pin descriptions here                                 }
{ ------------------------------------------------------------------------ }

{ ------------------------------------------------------------------------ }
{ 82S100 PLA (U17)                                                          }
{ ------------------------------------------------------------------------ }

function H(x: Integer): Integer;   { input x high }
begin
  Result := (1 shl x) or (1 shl (x + 16));
end;

function L(x: Integer): Integer;   { input x low }
begin
  Result := 1 shl x;
end;

function PO(x: Integer): Integer;
begin
  Result := 1 shl x;
end;

procedure TERM(spec, outs: Integer);
var t: TPlaTerm;
begin
  t := TPlaTerm.Create;
  t.mask := spec and $FFFF;
  t.value := (spec shr 16) and $FFFF;
  t.outputs := outs and $FF;
  pla_terms.Add(t);
end;

procedure InitPla;
var T_CPU, T_VIC, T_IO: Integer;
begin
  T_CPU := L(PI_AEC);
  T_VIC := H(PI_AEC);
  T_IO := H(PI_A15) or H(PI_A14) or L(PI_A13) or H(PI_A12);
  { /BASIC }
  TERM(H(PI_LORAM) or H(PI_HIRAM) or H(PI_A15) or L(PI_A14) or H(PI_A13) or T_CPU or H(PI_RW) or H(PI_GAME), PO(PO_BASIC) or PO(PO_CASRAM));
  { /KERNAL }
  TERM(H(PI_HIRAM) or H(PI_A15) or H(PI_A14) or H(PI_A13) or T_CPU or H(PI_RW) or H(PI_GAME), PO(PO_KERNAL) or PO(PO_CASRAM));
  TERM(H(PI_HIRAM) or H(PI_A15) or H(PI_A14) or H(PI_A13) or T_CPU or H(PI_RW) or L(PI_EXROM) or L(PI_GAME), PO(PO_KERNAL) or PO(PO_CASRAM));
  { /CHAROM }
  TERM(H(PI_HIRAM) or L(PI_CHAREN) or T_IO or T_CPU or H(PI_RW) or H(PI_GAME), PO(PO_CHAROM) or PO(PO_CASRAM));
  TERM(H(PI_LORAM) or L(PI_CHAREN) or T_IO or T_CPU or H(PI_RW) or H(PI_GAME), PO(PO_CHAROM) or PO(PO_CASRAM));
  TERM(H(PI_HIRAM) or L(PI_CHAREN) or T_IO or T_CPU or H(PI_RW) or L(PI_EXROM) or L(PI_GAME), PO(PO_CHAROM) or PO(PO_CASRAM));
  TERM(H(PI_VA14) or T_VIC or H(PI_GAME) or L(PI_VA13) or H(PI_VA12), PO(PO_CHAROM) or PO(PO_CASRAM));
  TERM(H(PI_VA14) or T_VIC or L(PI_EXROM) or L(PI_GAME) or L(PI_VA13) or H(PI_VA12), PO(PO_CHAROM) or PO(PO_CASRAM));
  { /IO }
  TERM(H(PI_HIRAM) or H(PI_CHAREN) or T_IO or T_CPU or H(PI_BA) or H(PI_RW) or H(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  TERM(H(PI_HIRAM) or H(PI_CHAREN) or T_IO or T_CPU or L(PI_RW) or H(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  TERM(H(PI_LORAM) or H(PI_CHAREN) or T_IO or T_CPU or H(PI_BA) or H(PI_RW) or H(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  TERM(H(PI_LORAM) or H(PI_CHAREN) or T_IO or T_CPU or L(PI_RW) or H(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  TERM(H(PI_HIRAM) or H(PI_CHAREN) or T_IO or T_CPU or H(PI_BA) or H(PI_RW) or L(PI_EXROM) or L(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  TERM(H(PI_HIRAM) or H(PI_CHAREN) or T_IO or T_CPU or L(PI_RW) or L(PI_EXROM) or L(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  TERM(H(PI_LORAM) or H(PI_CHAREN) or T_IO or T_CPU or H(PI_BA) or H(PI_RW) or L(PI_EXROM) or L(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  TERM(H(PI_LORAM) or H(PI_CHAREN) or T_IO or T_CPU or L(PI_RW) or L(PI_EXROM) or L(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  TERM(T_IO or T_CPU or H(PI_BA) or H(PI_RW) or H(PI_EXROM) or L(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  TERM(T_IO or T_CPU or L(PI_RW) or H(PI_EXROM) or L(PI_GAME), PO(PO_IO) or PO(PO_CASRAM));
  { GR/W }
  TERM(L(PI_CAS) or T_IO or T_CPU or L(PI_RW), PO(PO_GRW));
  { /ROML }
  TERM(H(PI_LORAM) or H(PI_HIRAM) or H(PI_A15) or L(PI_A14) or L(PI_A13) or T_CPU or H(PI_RW) or L(PI_EXROM), PO(PO_ROML) or PO(PO_CASRAM));
  TERM(H(PI_A15) or L(PI_A14) or L(PI_A13) or T_CPU or H(PI_EXROM) or L(PI_GAME), PO(PO_ROML) or PO(PO_CASRAM));
  { /ROMH }
  TERM(H(PI_HIRAM) or H(PI_A15) or L(PI_A14) or H(PI_A13) or T_CPU or H(PI_RW) or L(PI_EXROM) or L(PI_GAME), PO(PO_ROMH) or PO(PO_CASRAM));
  TERM(H(PI_A15) or H(PI_A14) or H(PI_A13) or T_CPU or H(PI_EXROM) or L(PI_GAME), PO(PO_ROMH) or PO(PO_CASRAM));
  TERM(T_VIC or H(PI_EXROM) or L(PI_GAME) or H(PI_VA13) or H(PI_VA12), PO(PO_ROMH) or PO(PO_CASRAM));
  { Ultimax holes }
  TERM(H(PI_EXROM) or L(PI_GAME) or L(PI_A15) or L(PI_A14) or H(PI_A12), PO(PO_CASRAM));
  TERM(H(PI_EXROM) or L(PI_GAME) or L(PI_A15) or L(PI_A14) or H(PI_A13), PO(PO_CASRAM));
  TERM(H(PI_EXROM) or L(PI_GAME) or L(PI_A15) or H(PI_A14), PO(PO_CASRAM));
  TERM(H(PI_EXROM) or L(PI_GAME) or H(PI_A15) or L(PI_A14) or H(PI_A13), PO(PO_CASRAM));
  TERM(H(PI_EXROM) or L(PI_GAME) or H(PI_A15) or H(PI_A14) or L(PI_A13) or L(PI_A12), PO(PO_CASRAM));
  { RAM only while /CAS is low }
  TERM(H(PI_CAS), PO(PO_CASRAM));
  pla_nterms := pla_terms.Length;
end;

function pla_read_inputs(s: TSim; p: TPart): Integer;
var k: Integer;
begin
  Result := 0;
  for k := 0 to PLA_NIN - 1 do Result := Result or (s.PinIn(p, PLA_IN_PIN[k]) shl k);
end;

function pla_logic(inputs: Integer): Integer;
var
  any, t: Integer;
  term: TPlaTerm;
begin
  any := 0;
  for t := 0 to pla_nterms - 1 do begin
    term := pla_terms[t];
    if (inputs and term.mask) = term.value then any := any or term.outputs;
  end;
  { The selects are active low; /CASRAM is high when RAM is NOT selected. }
  Result := (any xor (not (1 shl PO_CASRAM))) and $FF;
end;

procedure pla_eval(s: TSim; p: TPart);
var
  outv, k: Integer;
  en: Boolean;
begin
  outv := pla_logic(pla_read_inputs(s, p));
  en := s.PinIn(p, 19) = 0;     { /OE, tied to GND }
  for k := 0 to PLA_NOUT - 1 do
    if en then s.PinOut(p, PLA_OUT_PIN[k], DRV_BIT((outv shr k) and 1))
    else s.PinOut(p, PLA_OUT_PIN[k], DRV_Z);
end;

{ ------------------------------------------------------------------------ }
{ ROMs: 2364 (8K) and 2332 (4K)                                             }
{ ------------------------------------------------------------------------ }

const
  ROM_ADDR_PINS: array[0..12] of Integer = [8, 7, 6, 5, 4, 3, 2, 1, 23, 22, 19, 18, 21];
  ROM_DATA_PINS: array[0..7] of Integer = [9, 10, 11, 13, 14, 15, 16, 17];

function NewRomState: TObject;
begin
  Result := TRomState.Create;
end;

procedure rom2364_eval(s: TSim; p: TPart);
var
  st: TRomState;
  selected: Boolean;
  addr, v: Integer;
begin
  st := TRomState(p.state);
  selected := s.PinIn(p, 20) = 0;
  if selected then addr := part_read_bus(s, p, GK_ROM_ADDR, ROM_ADDR_PINS, 13) else addr := 0;
  if st.data <> nil then v := st.data[addr] else v := $FF;
  st.out_cache := part_drive_bus_cached(s, p, ROM_DATA_PINS, 8, v, selected, st.out_cache);
end;

procedure rom2332_eval(s: TSim; p: TPart);
var
  st: TRomState;
  selected: Boolean;
  addr, v: Integer;
begin
  st := TRomState(p.state);
  selected := (s.PinIn(p, 20) = 0) and (s.PinIn(p, 21) <> 0);   { CS1 low, CS2 high }
  if selected then addr := part_read_bus(s, p, GK_ROM_ADDR, ROM_ADDR_PINS, 12) else addr := 0;
  if st.data <> nil then v := st.data[addr] else v := $FF;
  st.out_cache := part_drive_bus_cached(s, p, ROM_DATA_PINS, 8, v, selected, st.out_cache);
end;

{ ------------------------------------------------------------------------ }
{ 2114 static RAM 1K x 4 (color RAM)                                        }
{ ------------------------------------------------------------------------ }

const
  SRAM_ADDR_PINS: array[0..9] of Integer = [5, 6, 7, 4, 3, 2, 1, 17, 16, 15];
  SRAM_IO_PINS: array[0..3] of Integer = [14, 13, 12, 11];

function NewSramState: TObject;
begin
  Result := TSramState.Create;
end;

procedure sram_eval(s: TSim; p: TPart);
var
  st: TSramState;
  rd: Boolean;
  addr: Integer;
begin
  st := TSramState(p.state);
  rd := (s.PinIn(p, 8) = 0) and (s.PinIn(p, 10) <> 0);    { /CS low, /WE high }
  if rd then addr := part_read_bus(s, p, GK_SRAM_ADDR, SRAM_ADDR_PINS, 10) else addr := 0;
  st.out_cache := part_drive_bus_cached(s, p, SRAM_IO_PINS, 4, st.cells[addr], rd, st.out_cache);
end;

procedure sram_commit(s: TSim; p: TPart);
var st: TSramState;
begin
  st := TSramState(p.state);
  if (s.PinIn(p, 8) = 0) and (s.PinIn(p, 10) = 0) then    { /CS low, /WE low }
    st.cells[part_read_bus(s, p, GK_SRAM_ADDR, SRAM_ADDR_PINS, 10)] :=
      part_read_bus(s, p, GK_SRAM_IO, SRAM_IO_PINS, 4);
end;

procedure sram_reset(s: TSim; p: TPart);
var
  st: TSramState;
  i: Integer;
begin
  st := TSramState(p.state);
  for i := 0 to 1023 do st.cells[i] := (i * 7 + (i shr 4)) and 15;
end;

{ ------------------------------------------------------------------------ }
{ 4164 dynamic RAM 64K x 1                                                  }
{ ------------------------------------------------------------------------ }

const
  DRAM_ADDR_PINS: array[0..7] of Integer = [5, 7, 6, 12, 11, 10, 13, 9];

function NewDramState: TObject;
begin
  Result := TDramState.Create;
end;

procedure dram_eval(s: TSim; p: TPart);
var st: TDramState;
begin
  st := TDramState(p.state);
  if st.out_en <> 0 then s.PinOut(p, 14, DRV_BIT(st.out_bit))
  else s.PinOut(p, 14, DRV_Z);
end;

procedure dram_cycle(s: TSim; p: TPart; bits: Integer; bad_row7: Boolean);
var
  st: TDramState;
  ras, cas, out_en, out_bit, cell: Integer;
  bad: Boolean;
begin
  st := TDramState(p.state);
  ras := s.PinIn(p, 4);
  cas := s.PinIn(p, 15);
  out_en := st.out_en;
  out_bit := st.out_bit;
  if (st.ras_prev <> 0) and (ras = 0) then
    st.row := part_read_bus(s, p, GK_DRAM_ADDR, DRAM_ADDR_PINS, bits);
  if (st.cas_prev <> 0) and (cas = 0) and (ras = 0) then begin
    st.col := part_read_bus(s, p, GK_DRAM_ADDR, DRAM_ADDR_PINS, bits);
    cell := (st.row shl 8) or st.col;
    bad := bad_row7 and ((st.row and $80) <> 0);
    if s.PinIn(p, 3) = 0 then begin     { /WE low before /CAS: early write }
      if not bad then st.cells[cell] := s.PinIn(p, 2);
      st.out_en := 0;
    end else begin                      { read: Q drives until /CAS rises }
      st.out_en := 1;
      if bad then st.out_bit := U32(Imul(cell, 2654435761)) shr 31
      else st.out_bit := st.cells[cell];
    end;
  end;
  { Q is valid until /CAS rises. }
  if cas <> 0 then st.out_en := 0;
  st.ras_prev := ras;
  st.cas_prev := cas;
  if (st.out_en <> out_en) or (st.out_bit <> out_bit) then s.MarkDirty(p.index);
end;

procedure dram_commit(s: TSim; p: TPart);
begin
  dram_cycle(s, p, 8, False);
end;

procedure dram_reset(s: TSim; p: TPart);
var
  st: TDramState;
  row, col, addr: Integer;
begin
  st := TDramState(p.state);
  for row := 0 to 255 do
    for col := 0 to 255 do begin
      addr := (col shl 8) or row;
      if (addr and 64) <> 0 then st.cells[(row shl 8) or col] := 1
      else st.cells[(row shl 8) or col] := 0;
    end;
  st.ras_prev := 1;
  st.cas_prev := 1;
  st.out_en := 0;
end;

{ ------------------------------------------------------------------------ }
{ TTL glue                                                                  }
{ ------------------------------------------------------------------------ }

const
  QUAD_GATES: array[0..11] of Integer = [1, 2, 3, 4, 5, 6, 9, 10, 8, 12, 13, 11];

procedure ls08_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 3 do
    s.PinOut(p, QUAD_GATES[i * 3 + 2],
      DRV_BIT((s.PinIn(p, QUAD_GATES[i * 3]) <> 0) and (s.PinIn(p, QUAD_GATES[i * 3 + 1]) <> 0)));
end;

const
  LS06_G: array[0..11] of Integer = [1, 2, 3, 4, 5, 6, 9, 8, 11, 10, 13, 12];

procedure ls06_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 5 do
    if s.PinIn(p, LS06_G[i * 2]) <> 0 then s.PinOut(p, LS06_G[i * 2 + 1], DRV_LO)
    else s.PinOut(p, LS06_G[i * 2 + 1], DRV_Z);
end;

const
  LS139_HALF: array[0..13] of Integer = [1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9];

procedure ls139_eval(s: TSim; p: TPart);
var
  h, b, sel, y: Integer;
  enabled: Boolean;
begin
  for h := 0 to 1 do begin
    b := h * 7;
    enabled := s.PinIn(p, LS139_HALF[b]) = 0;
    sel := s.PinIn(p, LS139_HALF[b + 1]) or (s.PinIn(p, LS139_HALF[b + 2]) shl 1);
    for y := 0 to 3 do
      s.PinOut(p, LS139_HALF[b + 3 + y], DRV_BIT(not (enabled and (sel = y))));
  end;
end;

const
  MUX_SEC: array[0..11] of Integer = [2, 3, 4, 5, 6, 7, 11, 10, 9, 14, 13, 12];
  MUX_OUTPUTS: array[0..3] of Integer = [4, 7, 9, 12];
  MUX_PINS = 'S:T 1A:T 1B:T 1Y:O 2A:T 2B:T 2Y:O GND:P 3Y:O 3B:T 3A:T 4Y:O 4B:T 4A:T /G:T VCC:P';

procedure mux_eval(s: TSim; p: TPart; invert: Integer);
var
  enabled: Boolean;
  sel, v, i, pin: Integer;
begin
  enabled := s.PinIn(p, 15) = 0;
  sel := s.PinIn(p, 1);
  v := 0;
  for i := 0 to 3 do begin
    if sel <> 0 then pin := MUX_SEC[i * 3 + 1] else pin := MUX_SEC[i * 3];
    v := v or ((s.PinIn(p, pin) xor invert) shl i);
  end;
  part_drive_bus(s, p, MUX_OUTPUTS, 4, v, enabled);
end;

procedure ls257_eval(s: TSim; p: TPart);
begin
  mux_eval(s, p, 0);
end;

procedure ls258_eval(s: TSim; p: TPart);
begin
  mux_eval(s, p, 1);
end;

const
  LS373_D: array[0..7] of Integer = [3, 4, 7, 8, 13, 14, 17, 18];
  LS373_Q: array[0..7] of Integer = [2, 5, 6, 9, 12, 15, 16, 19];

function NewLs373State: TObject;
begin
  Result := TLs373State.Create;
end;

procedure ls373_eval(s: TSim; p: TPart);
var
  st: TLs373State;
  enabled: Boolean;
  v: Integer;
begin
  st := TLs373State(p.state);
  enabled := s.PinIn(p, 1) = 0;
  if enabled and (s.PinIn(p, 11) <> 0) then v := part_read_bus(s, p, GK_LS373_D, LS373_D, 8)
  else v := st.latch;
  st.out_cache := part_drive_bus_cached(s, p, LS373_Q, 8, v, enabled, st.out_cache);
end;

procedure ls373_commit(s: TSim; p: TPart);
var st: TLs373State;
begin
  st := TLs373State(p.state);
  if s.PinIn(p, 11) <> 0 then st.latch := part_read_bus(s, p, GK_LS373_D, LS373_D, 8);
end;

const
  CD4066_SW: array[0..11] of Integer = [1, 2, 13, 4, 3, 5, 8, 9, 6, 11, 10, 12];

procedure cd4066_eval(s: TSim; p: TPart);
var
  i, a, b, from_a, from_b: Integer;
begin
  for i := 0 to 3 do begin
    a := p.Pin(CD4066_SW[i * 3]);
    b := p.Pin(CD4066_SW[i * 3 + 1]);
    if (a < 0) or (b < 0) then continue;
    if s.PinIn(p, CD4066_SW[i * 3 + 2]) <> 0 then begin
      from_a := s.DriveExcept(a, a);
      from_b := s.DriveExcept(b, b);
      s.Drive(a, from_b);
      s.Drive(b, from_a);
    end else begin
      s.Drive(a, DRV_Z);
      s.Drive(b, DRV_Z);
    end;
  end;
end;

{ 556 dual timer (U20): power-on reset and the RESTORE debouncer. }
const
  RESET_CYCLES = 50000;   { ~50 ms }
  NMI_CYCLES = 1000;      { ~1 ms }

function NewTimer556State: TObject;
var st: TTimer556State;
begin
  st := TTimer556State.Create;
  st.restore_net := -1;
  Result := st;
end;

procedure t556_eval(s: TSim; p: TPart);
var st: TTimer556State;
begin
  st := TTimer556State(p.state);
  s.PinOut(p, 5, DRV_BIT(s.cycle < st.nmi_until));
  s.PinOut(p, 9, DRV_BIT(s.cycle < st.reset_until));
end;

procedure t556_commit(s: TSim; p: TPart);
var
  st: TTimer556State;
  restore: Integer;
  out1, out2, hi1, hi2: Boolean;
begin
  st := TTimer556State(p.state);
  if st.last_cycle = s.cycle then exit;     { nothing new in this cycle }
  st.last_cycle := s.cycle;
  restore := 1;
  if st.restore_net >= 0 then restore := s.nets[s.nets[st.restore_net].root].level;
  if (st.restore_prev <> 0) and (restore = 0) then st.nmi_until := s.cycle + NMI_CYCLES;
  st.restore_prev := restore;
  out1 := s.cycle < st.nmi_until;
  out2 := s.cycle < st.reset_until;
  { (two steps: Quartex drops the parentheses of "b <> (x = y)") }
  hi1 := s.pins[p.Pin(5)].drive = DRV_HI;
  hi2 := s.pins[p.Pin(9)].drive = DRV_HI;
  if (out1 xor hi1) or (out2 xor hi2) then
    s.MarkDirty(p.index);
end;

procedure t556_reset(s: TSim; p: TPart);
var st: TTimer556State;
begin
  st := TTimer556State(p.state);
  st.reset_until := s.cycle + RESET_CYCLES;
  st.nmi_until := 0;
  st.restore_prev := 1;
  st.last_cycle := CYCLE_NEVER;
end;

{ 74LS629: dot clock and color clock oscillator. }
function NewVcoState: TObject;
begin
  Result := TVcoState.Create;
end;

procedure ls629_eval(s: TSim; p: TPart);
var st: TVcoState;
begin
  st := TVcoState(p.state);
  s.PinOut(p, 7, DRV_BIT(s.clock_level));
  if st.crystal_ok then s.PinOut(p, 10, DRV_BIT(s.clock_level))
  else s.PinOut(p, 10, DRV_LO);
end;

{ ------------------------------------------------------------------------ }
{ Resistors                                                                 }
{ ------------------------------------------------------------------------ }

{ What a series resistor passes from one side to the other: the drive on
  that side (its own drive excluded), one step weaker. }
function through(s: TSim; pin: Integer): Integer;
var d, st: Integer;
begin
  d := s.DriveExcept(pin, pin);
  st := d shr 1;
  if st >= S_STRONG then exit(MkDrive(S_RES, d and 1));
  if st >= S_PULL then exit(MkDrive(S_PULL, d and 1));
  Result := DRV_Z;
end;

procedure resistor_eval(s: TSim; p: TPart);
var
  st: TResistorState;
  e: TElement;
  i, level, d: Integer;
  a, b, k, bp: TPin;
  off, on: Boolean;
begin
  st := TResistorState(p.state);
  for i := 0 to st.nel - 1 do begin
    e := st.el[i];
    case e.typ of
      EL_PULLUP, EL_PULLDOWN: begin
        if e.typ = EL_PULLUP then level := 1 else level := 0;
        if p.rfault = RF_OPEN then d := DRV_Z
        else if p.rfault = RF_SHORT then d := MkDrive(S_SUPPLY, level)
        else d := MkDrive(S_PULL, level);
        s.Drive(e.pin_a, d);
      end;
      EL_SERIES: begin
        a := s.pins[e.pin_a];
        b := s.pins[e.pin_b];
        off := (p.rfault <> RF_OK) or (a.root < 0) or (b.root < 0) or (a.root = b.root) or
               (a.fault = PF_OPEN) or (b.fault = PF_OPEN);
        if off then begin
          s.Drive(e.pin_b, DRV_Z);
          s.Drive(e.pin_a, DRV_Z);
        end else begin
          s.Drive(e.pin_b, through(s, e.pin_a));
          s.Drive(e.pin_a, through(s, e.pin_b));
        end;
      end;
      EL_DIODE: begin
        k := s.pins[e.pin_b];
        on := (p.rfault = RF_OK) and (k.root >= 0) and (k.fault <> PF_OPEN) and
              (s.nets[k.root].strength <> S_Z) and (s.nets[k.root].level = 0);
        if p.rfault = RF_SHORT then on := (k.root >= 0) and (s.nets[k.root].level = 0);
        if on then s.Drive(e.pin_a, MkDrive(S_RES, 0)) else s.Drive(e.pin_a, DRV_Z);
      end;
      EL_DELAY: begin
        if p.rfault = RF_OPEN then s.Drive(e.pin_b, DRV_Z)
        else s.Drive(e.pin_b, DRV_BIT(st.delayed[i]));
      end;
      EL_NPN, EL_FOLLOWER: begin
        bp := s.pins[e.pin_a];
        on := (p.rfault = RF_SHORT) or
              ((p.rfault = RF_OK) and (bp.root >= 0) and (bp.fault <> PF_OPEN) and
               (s.nets[bp.root].strength <> S_Z) and (s.nets[bp.root].level <> 0));
        if not on then s.Drive(e.pin_b, DRV_Z)
        else if e.typ = EL_NPN then s.Drive(e.pin_b, DRV_LO)
        else s.Drive(e.pin_b, DRV_HI);
      end;
      EL_PNP: begin
        bp := s.pins[e.pin_a];
        on := (p.rfault = RF_SHORT) or
              ((p.rfault = RF_OK) and (bp.root >= 0) and (bp.fault <> PF_OPEN) and
               (s.nets[bp.root].strength <> S_Z) and (s.nets[bp.root].level = 0));
        if on then s.Drive(e.pin_b, DRV_HI) else s.Drive(e.pin_b, DRV_Z);
      end;
      EL_PNP_SINK: begin
        bp := s.pins[e.pin_a];
        on := (p.rfault = RF_SHORT) or
              ((p.rfault = RF_OK) and (bp.root >= 0) and (bp.fault <> PF_OPEN) and
               (s.nets[bp.root].strength <> S_Z) and (s.nets[bp.root].level = 0));
        if on then s.Drive(e.pin_b, DRV_LO) else s.Drive(e.pin_b, DRV_Z);
      end;
    end;
  end;
end;

procedure resistor_commit(s: TSim; p: TPart);
var
  st: TResistorState;
  i, v: Integer;
begin
  st := TResistorState(p.state);
  for i := 0 to st.nel - 1 do begin
    if st.el[i].typ <> EL_DELAY then continue;
    v := s.Read(st.el[i].pin_a);
    if v <> st.delayed[i] then begin
      st.delayed[i] := v;
      s.MarkDirty(p.index);
    end;
  end;
end;

procedure element_set_type(s: TSim; p: TPart; index, typ: Integer);
var
  st: TResistorState;
  e: TElement;
  a, b: TPin;
begin
  st := TResistorState(p.state);
  e := st.el[index];
  e.typ := typ;
  a := s.pins[e.pin_a];
  b := s.pins[e.pin_b];
  case typ of
    EL_SERIES: begin
      a.flags := PIN_SENSE or PIN_SWITCH or PIN_IN or PIN_OUT;
      b.flags := PIN_SENSE or PIN_SWITCH or PIN_IN or PIN_OUT;
    end;
    EL_DIODE: begin
      a.flags := PIN_OUT;
      b.flags := PIN_SENSE or PIN_IN;
    end;
    EL_NPN, EL_FOLLOWER, EL_PNP, EL_PNP_SINK: begin
      a.flags := PIN_SENSE or PIN_IN;
      b.flags := PIN_OUT;
    end;
    EL_DELAY: begin
      a.flags := PIN_IN;
      b.flags := PIN_OUT;
      p.commit := resistor_commit;
    end;
  end;
  p.eval := resistor_eval;
end;

{ ------------------------------------------------------------------------ }
{ ZX Spectrum 48K: memories and glue logic                                  }
{ ------------------------------------------------------------------------ }

procedure dram4116_commit(s: TSim; p: TPart);
begin
  dram_cycle(s, p, 7, False);
end;

procedure dram4532_commit(s: TSim; p: TPart);
begin
  dram_cycle(s, p, 8, True);
end;

const
  ROM23128_ADDR_PINS: array[0..13] of Integer = [10, 9, 8, 7, 6, 5, 4, 3, 25, 24, 21, 23, 2, 26];
  ROM23128_DATA_PINS: array[0..7] of Integer = [11, 12, 13, 15, 16, 17, 18, 19];

procedure rom23128_eval(s: TSim; p: TPart);
var
  st: TRomState;
  selected: Boolean;
  addr, v: Integer;
begin
  st := TRomState(p.state);
  selected := (s.PinIn(p, 20) = 0) and (s.PinIn(p, 22) = 0) and (s.PinIn(p, 27) = 0);
  if selected then addr := part_read_bus(s, p, GK_ROM23128_ADDR, ROM23128_ADDR_PINS, 14) else addr := 0;
  if st.data <> nil then v := st.data[addr] else v := $FF;
  st.out_cache := part_drive_bus_cached(s, p, ROM23128_DATA_PINS, 8, v, selected, st.out_cache);
end;

procedure ls157_eval(s: TSim; p: TPart);
var
  enabled: Boolean;
  sel, i, pin: Integer;
begin
  enabled := s.PinIn(p, 15) = 0;
  sel := s.PinIn(p, 1);
  for i := 0 to 3 do begin
    if sel <> 0 then pin := MUX_SEC[i * 3 + 1] else pin := MUX_SEC[i * 3];
    s.PinOut(p, MUX_SEC[i * 3 + 2], DRV_BIT(enabled and (s.PinIn(p, pin) <> 0)));
  end;
end;

procedure ls32_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 3 do
    s.PinOut(p, QUAD_GATES[i * 3 + 2],
      DRV_BIT((s.PinIn(p, QUAD_GATES[i * 3]) <> 0) or (s.PinIn(p, QUAD_GATES[i * 3 + 1]) <> 0)));
end;

procedure ls00_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 3 do
    s.PinOut(p, QUAD_GATES[i * 3 + 2],
      DRV_BIT(not ((s.PinIn(p, QUAD_GATES[i * 3]) <> 0) and (s.PinIn(p, QUAD_GATES[i * 3 + 1]) <> 0))));
end;

{ ------------------------------------------------------------------------ }
{ Type table                                                                }
{ ------------------------------------------------------------------------ }

procedure InitTypes;
begin
  PT_6510 := TPartType.Create('6510 CPU',
    'PHI0:M RDY:m /IRQ:m /NMI:m AEC:M VCC:P A0:O A1:O A2:O A3:O A4:O A5:O A6:O A7:O ' +
    'A8:O A9:O A10:O A11:O A12:O A13:O GND:P A14:O A15:O P5:B P4:B P3:B P2:B P1:B P0:B ' +
    'D7:B D6:B D5:B D4:B D3:B D2:B D1:B D0:B R/W:O PHI2:O /RES:m', nil, nil, nil, nil, TECH_NMOS);
  PT_6569 := TPartType.Create('6569 VIC-II',
    'D6:B D5:B D4:B D3:B D2:B D1:B D0:B /IRQ:O LP:m /CS:m R/W:m BA:O VDD:P COLOR:O ' +
    'S/LUM:O AEC:O PHI0:O /RAS:O /CAS:O GND:P PHICOL:m PHIIN:m A11:O A0/A8:B A1/A9:B ' +
    'A2/A10:B A3/A11:B A4/A12:B A5/A13:B A6:O A7:O A8:O A9:O A10:O D11:B D10:B D9:B ' +
    'D8:B D7:B VCC:P', nil, nil, nil, nil, TECH_NMOS);
  PT_6526 := TPartType.Create('6526 CIA',
    'GND:P PA0:B PA1:B PA2:B PA3:B PA4:B PA5:B PA6:B PA7:B PB0:B PB1:B PB2:B PB3:B ' +
    'PB4:B PB5:B PB6:B PB7:B /PC:O TOD:m VCC:P /IRQ:O R/W:m /CS:m /FLAG:m PHI2:m ' +
    'D7:B D6:B D5:B D4:B D3:B D2:B D1:B D0:B /RES:m RS3:m RS2:m RS1:m RS0:m SP:O CNT:O',
    nil, nil, nil, nil, TECH_NMOS);
  PT_6581 := TPartType.Create('6581 SID',
    'CAP1A:O CAP1B:O CAP2A:O CAP2B:O /RES:m PHI2:m R/W:m /CS:m A0:m A1:m A2:m A3:m A4:m ' +
    'GND:P D0:B D1:B D2:B D3:B D4:B D5:B D6:B D7:B POTY:m POTX:m VCC:P EXTIN:m ' +
    'AUDIO:O VDD:P', nil, nil, nil, nil, TECH_NMOS);
  PT_82S100 := TPartType.Create('82S100 PLA',
    'FE:O A13:T A14:T A15:T /VA14:T /CHAREN:T /HIRAM:T /LORAM:T /CAS:T /ROMH:O /ROML:O ' +
    '/IO:O GR/W:O GND:P /CHAROM:O /KERNAL:O /BASIC:O /CASRAM:O /OE:T VA12:T VA13:T ' +
    '/GAME:T /EXROM:T R/W:T /AEC:T BA:T A12:T VCC:P', pla_eval, nil, nil, nil, TECH_TTL);
  PT_2364 := TPartType.Create('2364 ROM',
    'A7:M A6:M A5:M A4:M A3:M A2:M A1:M A0:M D0:O D1:O D2:O GND:P D3:O D4:O D5:O D6:O ' +
    'D7:O A11:M A10:M /CS:M A12:M A9:M A8:M VCC:P', rom2364_eval, nil, nil, NewRomState, TECH_NMOS);
  PT_2332 := TPartType.Create('2332 ROM',
    'A7:M A6:M A5:M A4:M A3:M A2:M A1:M A0:M D0:O D1:O D2:O GND:P D3:O D4:O D5:O D6:O ' +
    'D7:O A11:M A10:M /CS1:M CS2:M A9:M A8:M VCC:P', rom2332_eval, nil, nil, NewRomState, TECH_NMOS);
  PT_2114 := TPartType.Create('2114 SRAM',
    'A6:M A5:M A4:M A3:M A0:M A1:M A2:M /CS:M GND:P /WE:M IO4:B IO3:B IO2:B IO1:B ' +
    'A9:M A8:M A7:M VCC:P', sram_eval, sram_commit, sram_reset, NewSramState, TECH_NMOS);
  PT_4164 := TPartType.Create('4164',
    'NC:O D:m /WE:m /RAS:m A0:m A2:m A1:m VCC:P A7:m A5:m A4:m A3:m A6:m Q:O /CAS:m GND:P',
    dram_eval, dram_commit, dram_reset, NewDramState, TECH_NMOS);
  PT_74LS08 := TPartType.Create('74LS08',
    '1A:T 1B:T 1Y:O 2A:T 2B:T 2Y:O GND:P 3Y:O 3A:T 3B:T 4Y:O 4A:T 4B:T VCC:P',
    ls08_eval, nil, nil, nil, TECH_TTL);
  PT_7406 := TPartType.Create('7406',
    '1A:T 1Y:O 2A:T 2Y:O 3A:T 3Y:O GND:P 4Y:O 4A:T 5Y:O 5A:T 6Y:O 6A:T VCC:P',
    ls06_eval, nil, nil, nil, TECH_TTL_OC);
  PT_74LS139 := TPartType.Create('74LS139',
    '1/G:T 1A:T 1B:T 1Y0:O 1Y1:O 1Y2:O 1Y3:O GND:P 2Y3:O 2Y2:O 2Y1:O 2Y0:O 2B:T 2A:T ' +
    '2/G:T VCC:P', ls139_eval, nil, nil, nil, TECH_TTL);
  PT_74LS257 := TPartType.Create('74LS257', MUX_PINS, ls257_eval, nil, nil, nil, TECH_TTL);
  PT_74LS258 := TPartType.Create('74LS258', MUX_PINS, ls258_eval, nil, nil, nil, TECH_TTL);
  PT_74LS373 := TPartType.Create('74LS373',
    '/OE:T 1Q:O 1D:T 2D:T 2Q:O 3Q:O 3D:T 4D:T 4Q:O GND:P LE:T 5Q:O 5D:T 6D:T 6Q:O 7Q:O ' +
    '7D:T 8D:T 8Q:O VCC:P', ls373_eval, ls373_commit, nil, NewLs373State, TECH_TTL);
  PT_4066 := TPartType.Create('4066',
    '1A:S 1B:S 2B:S 2A:S C2:M C3:M VSS:P 3A:S 3B:S 4B:S 4A:S C4:M C1:M VDD:P',
    cd4066_eval, nil, nil, nil, TECH_CMOS);
  PT_556 := TPartType.Create('556 TIMER',
    'DIS1:O THR1:O CV1:O /RES1:m OUT1:O TRIG1:m GND:P TRIG2:m OUT2:O /RES2:m CV2:O THR2:O ' +
    'DIS2:O VCC:P', t556_eval, t556_commit, t556_reset, NewTimer556State, TECH_TTL);
  PT_74LS629 := TPartType.Create('74LS629 VCO',
    '2FC:O 1FC:O 1RNG:O 1CX:O 1CX:O /1EN:m 1Y:O OSCGND:P GND:P 2Y:O /2EN:m 2CX:O 2CX:O ' +
    '2RNG:O OSCVCC:O VCC:P', ls629_eval, nil, nil, NewVcoState, TECH_TTL);
  PT_74LS74 := TPartType.Create('74LS74',
    '/1CLR:m 1D:m 1CK:m /1PR:m 1Q:O /1Q:O GND:P /2Q:O 2Q:O /2PR:m 2CK:m 2D:m /2CLR:m VCC:P',
    nil, nil, nil, nil, TECH_TTL);
  PT_74LS193 := TPartType.Create('74LS193',
    'B:m QB:O QA:O DOWN:m UP:m QC:O QD:O GND:P D:m C:m /LOAD:m /CO:O /BO:O CLR:m A:m VCC:P',
    nil, nil, nil, nil, TECH_TTL);
  PT_MC4044 := TPartType.Create('MC4044 PLL',
    'R:m 2:O V:m 4:O 5:O 6:O GND:P 8:O 9:O 10:O 11:O 12:O 13:O VCC:P', nil, nil, nil, nil, TECH_TTL);
  PT_4116 := TPartType.Create('4116',
    'VBB:P D:m /WE:m /RAS:m A0:m A2:m A1:m VDD:P VCC:P A5:m A4:m A3:m A6:m Q:O /CAS:m VSS:P',
    dram_eval, dram4116_commit, dram_reset, NewDramState, TECH_NMOS);
  PT_4116.edge_commit := True;
  PT_4116.edge_pins := [4, 15];          { /RAS, /CAS }
  PT_4532 := TPartType.Create('4532',
    'NC:O D:m /WE:m /RAS:m A0:m A2:m A1:m VCC:P A7:m A5:m A4:m A3:m A6:m Q:O /CAS:m GND:P',
    dram_eval, dram4532_commit, dram_reset, NewDramState, TECH_NMOS);
  PT_23128 := TPartType.Create('23128 ROM',
    'NC:O A12:M A7:M A6:M A5:M A4:M A3:M A2:M A1:M A0:M D0:O D1:O D2:O GND:P D3:O D4:O D5:O D6:O ' +
    'D7:O /CE:M A10:M /OE:M A11:M A9:M A8:M A13:M OE1:M VCC:P', rom23128_eval, nil, nil, NewRomState,
    TECH_NMOS);
  PT_74LS157 := TPartType.Create('74LS157', MUX_PINS, ls157_eval, nil, nil, nil, TECH_TTL);
  PT_74LS32 := TPartType.Create('74LS32',
    '1A:T 1B:T 1Y:O 2A:T 2B:T 2Y:O GND:P 3Y:O 3A:T 3B:T 4Y:O 4A:T 4B:T VCC:P', ls32_eval, nil, nil, nil, TECH_TTL);
  PT_74LS00 := TPartType.Create('74LS00',
    '1A:T 1B:T 1Y:O 2A:T 2B:T 2Y:O GND:P 3Y:O 3A:T 3B:T 4Y:O 4A:T 4B:T VCC:P', ls00_eval, nil, nil, nil, TECH_TTL);
  PT_LM1889 := TPartType.Create('LM1889',
    'CHROMA_LEAD:O R-Y_IN:O CHROMA_BIAS:O B-Y_IN:O GND:P TANK_B1:O TANK_B2:O TANK_A1:O TANK_A2:O ' +
    'OUT_B:O OUT_A:O VIDEO_IN:O SUBCARRIER:O VCC:P VCC:P VCC:P OSC_OUT:O CHROMA_LAG:O', nil, nil, nil, nil, TECH_NMOS);
end;

initialization
  InitPla;
  InitTypes;
end.
