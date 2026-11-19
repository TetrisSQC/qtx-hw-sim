unit hwsim.ttl;

{ Pin-level models of the TTL and MOS parts of the Apple II+ motherboard.
  Same pattern as hwsim.parts: eval() drives outputs from inputs and state;
  edge triggered parts remember their clock level and sample their inputs
  in commit() when it rises (all flip-flops of the board see the same edge:
  commit reads pins, which still show the levels before the edge).
  Pin numbers follow the datasheets (they match the schematic symbols). }

interface

uses
  hwsim.util, hwsim.sim, hwsim.parts;

type
  { Generic state for clocked parts. }
  TRegState = class
  public
    q: Integer;          { register contents / counter value }
    q2: Integer;         { a second register (dual flip-flops) }
    clk: Integer;        { last clock level }
    clk2: Integer;
    out_cache: Integer;
  end;

  TRom2316State = class
  public
    data: TBytes;        { 2K }
    out_cache: Integer;
  end;

  TTimer555State = class
  public
    until_cycle: Float;  { output high until this cycle (power-on one-shot) }
    period: Float;       { astable: 0 = one-shot }
  end;

var
  PT_74LS02, PT_74LS04, PT_74LS11, PT_74LS20, PT_74LS51, PT_74LS86, PT_74LS74x: TPartType;
  PT_74LS138, PT_74LS139x, PT_74LS151, PT_74LS153, PT_74LS161, PT_74LS166, PT_74LS174, PT_74LS175: TPartType;
  PT_74LS194, PT_74LS195, PT_74LS251, PT_74LS257x, PT_74LS259, PT_74LS283: TPartType;
  PT_8T97, PT_8T28, PT_2513, PT_2316, PT_4116A, PT_555A, PT_NE558, PT_MEMBLOCK: TPartType;
  PT_74LS00x, PT_74LS08x, PT_74LS32x: TPartType;

const
  GK_TTL = GK_MACHINE + 200;

implementation

function NewReg: TObject;
begin
  Result := TRegState.Create;
end;

function B(v: Integer): Boolean;
begin
  Result := v <> 0;
end;

{ ------------------------------------------------------------------------ }
{ Gates                                                                     }
{ ------------------------------------------------------------------------ }

const
  { 74LS02: Y, A, B per gate }
  G02: array[0..11] of Integer = [1, 2, 3, 4, 5, 6, 10, 8, 9, 13, 11, 12];
  { 74LS04: A, Y per inverter }
  G04: array[0..11] of Integer = [1, 2, 3, 4, 5, 6, 9, 8, 11, 10, 13, 12];
  { 74LS00/08/32/86: A, B, Y }
  GQ: array[0..11] of Integer = [1, 2, 3, 4, 5, 6, 9, 10, 8, 12, 13, 11];
  { 74LS11: A, B, C, Y }
  G11: array[0..11] of Integer = [1, 2, 13, 12, 3, 4, 5, 6, 9, 10, 11, 8];
  { 74LS20: A, B, C, D, Y }
  G20: array[0..9] of Integer = [1, 2, 4, 5, 6, 9, 10, 12, 13, 8];

procedure ls02_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 3 do
    s.PinOut(p, G02[i * 3], DRV_BIT((s.PinIn(p, G02[i * 3 + 1]) or s.PinIn(p, G02[i * 3 + 2])) = 0));
end;

procedure ls04_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 5 do
    s.PinOut(p, G04[i * 2 + 1], DRV_BIT(s.PinIn(p, G04[i * 2]) = 0));
end;

procedure ls00_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 3 do
    s.PinOut(p, GQ[i * 3 + 2], DRV_BIT((s.PinIn(p, GQ[i * 3]) and s.PinIn(p, GQ[i * 3 + 1])) = 0));
end;

procedure ls08_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 3 do
    s.PinOut(p, GQ[i * 3 + 2], DRV_BIT(s.PinIn(p, GQ[i * 3]) and s.PinIn(p, GQ[i * 3 + 1])));
end;

procedure ls32_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 3 do
    s.PinOut(p, GQ[i * 3 + 2], DRV_BIT(s.PinIn(p, GQ[i * 3]) or s.PinIn(p, GQ[i * 3 + 1])));
end;

procedure ls86_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 3 do
    s.PinOut(p, GQ[i * 3 + 2], DRV_BIT(s.PinIn(p, GQ[i * 3]) xor s.PinIn(p, GQ[i * 3 + 1])));
end;

procedure ls11_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 2 do
    s.PinOut(p, G11[i * 4 + 3], DRV_BIT(s.PinIn(p, G11[i * 4]) and s.PinIn(p, G11[i * 4 + 1]) and s.PinIn(p, G11[i * 4 + 2])));
end;

procedure ls20_eval(s: TSim; p: TPart);
var i: Integer;
begin
  for i := 0 to 1 do
    s.PinOut(p, G20[i * 5 + 4], DRV_BIT((s.PinIn(p, G20[i * 5]) and s.PinIn(p, G20[i * 5 + 1]) and
      s.PinIn(p, G20[i * 5 + 2]) and s.PinIn(p, G20[i * 5 + 3])) = 0));
end;

{ 74LS51 (TI pinout): 1Y = /(1A 1B 1C + 1D 1E 1F) with 1A 1, 1B 13, 1C 12,
  1D 11, 1E 10, 1F 9; 2Y = /(2A 2B + 2C 2D), pins 2 3 4 5. }
procedure ls51_eval(s: TSim; p: TPart);
var a, b: Integer;
begin
  a := s.PinIn(p, 1) and s.PinIn(p, 13) and s.PinIn(p, 12);
  b := s.PinIn(p, 11) and s.PinIn(p, 10) and s.PinIn(p, 9);
  s.PinOut(p, 8, DRV_BIT((a or b) = 0));
  a := s.PinIn(p, 2) and s.PinIn(p, 3);
  b := s.PinIn(p, 4) and s.PinIn(p, 5);
  s.PinOut(p, 6, DRV_BIT((a or b) = 0));
end;

{ ------------------------------------------------------------------------ }
{ Decoders and multiplexers                                                 }
{ ------------------------------------------------------------------------ }

const
  LS138_OUT: array[0..7] of Integer = [15, 14, 13, 12, 11, 10, 9, 7];

procedure ls138_eval(s: TSim; p: TPart);
var
  en: Boolean;
  sel, y: Integer;
begin
  en := (s.PinIn(p, 4) = 0) and (s.PinIn(p, 5) = 0) and (s.PinIn(p, 6) <> 0);
  sel := s.PinIn(p, 1) or (s.PinIn(p, 2) shl 1) or (s.PinIn(p, 3) shl 2);
  for y := 0 to 7 do s.PinOut(p, LS138_OUT[y], DRV_BIT(not (en and (sel = y))));
end;

const
  LS139_HALF: array[0..13] of Integer = [1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9];

procedure ls139_eval(s: TSim; p: TPart);
var
  h, b, sel, y: Integer;
  en: Boolean;
begin
  for h := 0 to 1 do begin
    b := h * 7;
    en := s.PinIn(p, LS139_HALF[b]) = 0;
    sel := s.PinIn(p, LS139_HALF[b + 1]) or (s.PinIn(p, LS139_HALF[b + 2]) shl 1);
    for y := 0 to 3 do s.PinOut(p, LS139_HALF[b + 3 + y], DRV_BIT(not (en and (sel = y))));
  end;
end;

const
  MUX8_IN: array[0..7] of Integer = [4, 3, 2, 1, 15, 14, 13, 12];

{ 74LS151: 8 to 1 multiplexer, /E low enables; Z and /Z. }
procedure ls151_eval(s: TSim; p: TPart);
var
  sel, v: Integer;
begin
  if s.PinIn(p, 7) <> 0 then v := 0
  else begin
    sel := s.PinIn(p, 11) or (s.PinIn(p, 10) shl 1) or (s.PinIn(p, 9) shl 2);
    v := s.PinIn(p, MUX8_IN[sel]);
  end;
  s.PinOut(p, 5, DRV_BIT(v));
  s.PinOut(p, 6, DRV_BIT(v = 0));
end;

{ 74LS251: the same with three-state outputs, /OE low enables. }
procedure ls251_eval(s: TSim; p: TPart);
var
  sel, v: Integer;
begin
  if s.PinIn(p, 7) <> 0 then begin
    s.PinOut(p, 5, DRV_Z);
    s.PinOut(p, 6, DRV_Z);
    exit;
  end;
  sel := s.PinIn(p, 11) or (s.PinIn(p, 10) shl 1) or (s.PinIn(p, 9) shl 2);
  v := s.PinIn(p, MUX8_IN[sel]);
  s.PinOut(p, 5, DRV_BIT(v));
  s.PinOut(p, 6, DRV_BIT(v = 0));
end;

{ 74LS153: dual 4 to 1, common selects S0 (14) S1 (2), /Ea (1) /Eb (15). }
const
  LS153_A: array[0..3] of Integer = [6, 5, 4, 3];
  LS153_B: array[0..3] of Integer = [10, 11, 12, 13];

procedure ls153_eval(s: TSim; p: TPart);
var sel: Integer;
begin
  sel := s.PinIn(p, 14) or (s.PinIn(p, 2) shl 1);
  if s.PinIn(p, 1) <> 0 then s.PinOut(p, 7, DRV_LO) else s.PinOut(p, 7, DRV_BIT(s.PinIn(p, LS153_A[sel])));
  if s.PinIn(p, 15) <> 0 then s.PinOut(p, 9, DRV_LO) else s.PinOut(p, 9, DRV_BIT(s.PinIn(p, LS153_B[sel])));
end;

{ 74LS257: quad 2 to 1, S low selects I0; OE (15) high: outputs off. }
const
  LS257: array[0..11] of Integer = [2, 3, 4, 5, 6, 7, 11, 10, 9, 14, 13, 12];

procedure ls257_eval(s: TSim; p: TPart);
var
  i, sel: Integer;
  en: Boolean;
begin
  en := s.PinIn(p, 15) = 0;
  sel := s.PinIn(p, 1);
  for i := 0 to 3 do
    if not en then s.PinOut(p, LS257[i * 3 + 2], DRV_Z)
    else s.PinOut(p, LS257[i * 3 + 2], DRV_BIT(s.PinIn(p, LS257[i * 3 + sel])));
end;

{ 74LS283: 4 bit adder. }
procedure ls283_eval(s: TSim; p: TPart);
var a, bb, sum: Integer;
begin
  a := s.PinIn(p, 5) or (s.PinIn(p, 3) shl 1) or (s.PinIn(p, 14) shl 2) or (s.PinIn(p, 12) shl 3);
  bb := s.PinIn(p, 6) or (s.PinIn(p, 2) shl 1) or (s.PinIn(p, 15) shl 2) or (s.PinIn(p, 11) shl 3);
  sum := a + bb + s.PinIn(p, 7);
  s.PinOut(p, 4, DRV_BIT(sum and 1));
  s.PinOut(p, 1, DRV_BIT((sum shr 1) and 1));
  s.PinOut(p, 13, DRV_BIT((sum shr 2) and 1));
  s.PinOut(p, 10, DRV_BIT((sum shr 3) and 1));
  s.PinOut(p, 9, DRV_BIT((sum shr 4) and 1));
end;

{ ------------------------------------------------------------------------ }
{ Flip-flops, latches, counters, shift registers                            }
{ ------------------------------------------------------------------------ }

{ 74LS74: dual D flip-flop, asynchronous /R and /S (both low: Q and /Q high). }
const
  LS74: array[0..11] of Integer = [1, 2, 3, 4, 5, 6, 13, 12, 11, 10, 9, 8];   { /R D C /S Q /Q }

procedure ls74_eval(s: TSim; p: TPart);
var
  st: TRegState;
  i, r, sset, q: Integer;
begin
  st := TRegState(p.state);
  for i := 0 to 1 do begin
    if i = 0 then q := st.q else q := st.q2;
    r := s.PinIn(p, LS74[i * 6]);
    sset := s.PinIn(p, LS74[i * 6 + 3]);
    if (r = 0) and (sset = 0) then begin
      s.PinOut(p, LS74[i * 6 + 4], DRV_HI);
      s.PinOut(p, LS74[i * 6 + 5], DRV_HI);
      continue;
    end;
    if r = 0 then q := 0 else if sset = 0 then q := 1;
    s.PinOut(p, LS74[i * 6 + 4], DRV_BIT(q));
    s.PinOut(p, LS74[i * 6 + 5], DRV_BIT(q = 0));
  end;
end;

procedure ls74_commit(s: TSim; p: TPart);
var
  st: TRegState;
  i, c, q, prev, r, sset: Integer;
  changed: Boolean;
begin
  st := TRegState(p.state);
  changed := False;
  for i := 0 to 1 do begin
    c := s.PinIn(p, LS74[i * 6 + 2]);
    if i = 0 then begin prev := st.clk; q := st.q; end else begin prev := st.clk2; q := st.q2; end;
    r := s.PinIn(p, LS74[i * 6]);
    sset := s.PinIn(p, LS74[i * 6 + 3]);
    if r = 0 then q := 0
    else if sset = 0 then q := 1
    else if (prev = 0) and (c <> 0) then q := s.PinIn(p, LS74[i * 6 + 1]);
    if i = 0 then begin
      if q <> st.q then changed := True;
      st.q := q; st.clk := c;
    end else begin
      if q <> st.q2 then changed := True;
      st.q2 := q; st.clk2 := c;
    end;
  end;
  if changed then s.MarkDirty(p.index);
end;

{ 74LS174: hex D flip-flop with /MR (1), clock 9. }
const
  LS174_D: array[0..5] of Integer = [3, 4, 6, 11, 13, 14];
  LS174_Q: array[0..5] of Integer = [2, 5, 7, 10, 12, 15];

procedure ls174_eval(s: TSim; p: TPart);
var
  st: TRegState;
  i, q: Integer;
begin
  st := TRegState(p.state);
  if s.PinIn(p, 1) = 0 then q := 0 else q := st.q;
  for i := 0 to 5 do s.PinOut(p, LS174_Q[i], DRV_BIT((q shr i) and 1));
end;

procedure ls174_commit(s: TSim; p: TPart);
var
  st: TRegState;
  c, q, i: Integer;
begin
  st := TRegState(p.state);
  c := s.PinIn(p, 9);
  q := st.q;
  if s.PinIn(p, 1) = 0 then q := 0
  else if (st.clk = 0) and (c <> 0) then begin
    q := 0;
    for i := 0 to 5 do q := q or (s.PinIn(p, LS174_D[i]) shl i);
  end;
  st.clk := c;
  if q <> st.q then begin st.q := q; s.MarkDirty(p.index); end;
end;

{ 74LS175: quad D flip-flop with true and complement outputs. }
const
  LS175_D: array[0..3] of Integer = [4, 5, 12, 13];
  LS175_Q: array[0..3] of Integer = [2, 7, 10, 15];
  LS175_NQ: array[0..3] of Integer = [3, 6, 11, 14];

procedure ls175_eval(s: TSim; p: TPart);
var
  st: TRegState;
  i, q: Integer;
begin
  st := TRegState(p.state);
  if s.PinIn(p, 1) = 0 then q := 0 else q := st.q;
  for i := 0 to 3 do begin
    s.PinOut(p, LS175_Q[i], DRV_BIT((q shr i) and 1));
    s.PinOut(p, LS175_NQ[i], DRV_BIT(((q shr i) and 1) = 0));
  end;
end;

procedure ls175_commit(s: TSim; p: TPart);
var
  st: TRegState;
  c, q, i: Integer;
begin
  st := TRegState(p.state);
  c := s.PinIn(p, 9);
  q := st.q;
  if s.PinIn(p, 1) = 0 then q := 0
  else if (st.clk = 0) and (c <> 0) then begin
    q := 0;
    for i := 0 to 3 do q := q or (s.PinIn(p, LS175_D[i]) shl i);
  end;
  st.clk := c;
  if q <> st.q then begin st.q := q; s.MarkDirty(p.index); end;
end;

{ 74LS161: 4 bit synchronous counter. /MR (1) asynchronous clear, CP 2,
  /PE 9 synchronous load, CEP 7, CET 10, TC 15 = Q=15 and CET. }
procedure ls161_eval(s: TSim; p: TPart);
var
  st: TRegState;
  q: Integer;
begin
  st := TRegState(p.state);
  if s.PinIn(p, 1) = 0 then q := 0 else q := st.q;
  s.PinOut(p, 14, DRV_BIT(q and 1));
  s.PinOut(p, 13, DRV_BIT((q shr 1) and 1));
  s.PinOut(p, 12, DRV_BIT((q shr 2) and 1));
  s.PinOut(p, 11, DRV_BIT((q shr 3) and 1));
  s.PinOut(p, 15, DRV_BIT((q = 15) and (s.PinIn(p, 10) <> 0)));
end;

procedure ls161_commit(s: TSim; p: TPart);
var
  st: TRegState;
  c, q: Integer;
begin
  st := TRegState(p.state);
  c := s.PinIn(p, 2);
  q := st.q;
  if s.PinIn(p, 1) = 0 then q := 0
  else if (st.clk = 0) and (c <> 0) then begin
    if s.PinIn(p, 9) = 0 then
      q := s.PinIn(p, 3) or (s.PinIn(p, 4) shl 1) or (s.PinIn(p, 5) shl 2) or (s.PinIn(p, 6) shl 3)
    else if (s.PinIn(p, 7) <> 0) and (s.PinIn(p, 10) <> 0) then
      q := (q + 1) and 15;
  end;
  st.clk := c;
  if q <> st.q then begin st.q := q; s.MarkDirty(p.index); end;
end;

{ 74LS166: 8 bit parallel-in serial-out shift register. Clock 7 (inhibited
  while CE 6 is high), /PE 15 low loads A..H, Clr 9 low clears, Ds 1 serial
  in, Qh 13 out. Bit 7 (H) is the output: A=bit0 .. H=bit7, shifts toward H. }
const
  LS166_IN: array[0..7] of Integer = [2, 3, 4, 5, 10, 11, 12, 14];

procedure ls166_eval(s: TSim; p: TPart);
var st: TRegState;
begin
  st := TRegState(p.state);
  if s.PinIn(p, 9) = 0 then s.PinOut(p, 13, DRV_LO)
  else s.PinOut(p, 13, DRV_BIT((st.q shr 7) and 1));
end;

procedure ls166_commit(s: TSim; p: TPart);
var
  st: TRegState;
  c, q, i: Integer;
begin
  st := TRegState(p.state);
  c := s.PinIn(p, 7);
  q := st.q;
  if s.PinIn(p, 9) = 0 then q := 0
  else if (st.clk = 0) and (c <> 0) and (s.PinIn(p, 6) = 0) then begin
    if s.PinIn(p, 15) = 0 then begin
      q := 0;
      for i := 0 to 7 do q := q or (s.PinIn(p, LS166_IN[i]) shl i);
    end else
      q := ((q shl 1) or s.PinIn(p, 1)) and $FF;
  end;
  st.clk := c;
  if q <> st.q then begin st.q := q; s.MarkDirty(p.index); end;
end;

{ 74LS194: 4 bit bidirectional universal shift register. Mr 1 (active low
  clear), Dsr 2, P0-P3 3-6, Dsl 7, S0 9, S1 10, Cp 11, Q0-Q3 15,14,13,12.
  S1 S0: 00 hold, 01 shift right (Q0 <- Dsr), 10 shift left (Q3 <- Dsl), 11 load. }
procedure ls194_eval(s: TSim; p: TPart);
var
  st: TRegState;
  q: Integer;
begin
  st := TRegState(p.state);
  if s.PinIn(p, 1) = 0 then q := 0 else q := st.q;
  s.PinOut(p, 15, DRV_BIT(q and 1));
  s.PinOut(p, 14, DRV_BIT((q shr 1) and 1));
  s.PinOut(p, 13, DRV_BIT((q shr 2) and 1));
  s.PinOut(p, 12, DRV_BIT((q shr 3) and 1));
end;

procedure ls194_commit(s: TSim; p: TPart);
var
  st: TRegState;
  c, q, mode: Integer;
begin
  st := TRegState(p.state);
  c := s.PinIn(p, 11);
  q := st.q;
  if s.PinIn(p, 1) = 0 then q := 0
  else if (st.clk = 0) and (c <> 0) then begin
    mode := s.PinIn(p, 9) or (s.PinIn(p, 10) shl 1);
    case mode of
      1: q := ((q shl 1) or s.PinIn(p, 2)) and 15;
      2: q := (q shr 1) or (s.PinIn(p, 7) shl 3);
      3: q := s.PinIn(p, 3) or (s.PinIn(p, 4) shl 1) or (s.PinIn(p, 5) shl 2) or (s.PinIn(p, 6) shl 3);
    end;
  end;
  st.clk := c;
  if q <> st.q then begin st.q := q; s.MarkDirty(p.index); end;
end;

{ 74LS195: 4 bit shift register with J/K serial input. /MR 1, J 2, /K 3,
  P0-P3 4-7, /PE 9 (low: load), CP 10, Q0-Q3 15,14,13,12, /Q3 11. }
procedure ls195_eval(s: TSim; p: TPart);
var
  st: TRegState;
  q: Integer;
begin
  st := TRegState(p.state);
  if s.PinIn(p, 1) = 0 then q := 0 else q := st.q;
  s.PinOut(p, 15, DRV_BIT(q and 1));
  s.PinOut(p, 14, DRV_BIT((q shr 1) and 1));
  s.PinOut(p, 13, DRV_BIT((q shr 2) and 1));
  s.PinOut(p, 12, DRV_BIT((q shr 3) and 1));
  s.PinOut(p, 11, DRV_BIT(((q shr 3) and 1) = 0));
end;

procedure ls195_commit(s: TSim; p: TPart);
var
  st: TRegState;
  c, q, j, k, q0: Integer;
begin
  st := TRegState(p.state);
  c := s.PinIn(p, 10);
  q := st.q;
  if s.PinIn(p, 1) = 0 then q := 0
  else if (st.clk = 0) and (c <> 0) then begin
    if s.PinIn(p, 9) = 0 then
      q := s.PinIn(p, 4) or (s.PinIn(p, 5) shl 1) or (s.PinIn(p, 6) shl 2) or (s.PinIn(p, 7) shl 3)
    else begin
      j := s.PinIn(p, 2);
      k := s.PinIn(p, 3);       { /K }
      { J /K: 00 -> 0, 11 -> 1, 10 -> toggle, 01 -> hold }
      if (j = 0) and (k = 0) then q0 := 0
      else if (j <> 0) and (k <> 0) then q0 := 1
      else if j <> 0 then q0 := (q and 1) xor 1
      else q0 := q and 1;
      q := ((q shl 1) and 14) or q0;
    end;
  end;
  st.clk := c;
  if q <> st.q then begin st.q := q; s.MarkDirty(p.index); end;
end;

{ 74LS259: 8 bit addressable latch. A0-A2 1-3, D 13, /E 14 (low: the
  addressed latch follows D), /Clr 15 (low: all clear while /E high;
  with /E low a demultiplexer), Q0-Q7 4-7, 9-12. Level sensitive. }
const
  LS259_Q: array[0..7] of Integer = [4, 5, 6, 7, 9, 10, 11, 12];

procedure ls259_eval(s: TSim; p: TPart);
var
  st: TRegState;
  q, a, i: Integer;
  en, clr: Boolean;
begin
  st := TRegState(p.state);
  en := s.PinIn(p, 14) = 0;
  clr := s.PinIn(p, 15) = 0;
  a := s.PinIn(p, 1) or (s.PinIn(p, 2) shl 1) or (s.PinIn(p, 3) shl 2);
  q := st.q;
  if clr then begin
    if en then q := s.PinIn(p, 13) shl a else q := 0;
  end else if en then
    q := (q and not (1 shl a)) or (s.PinIn(p, 13) shl a);
  st.q := q;
  for i := 0 to 7 do s.PinOut(p, LS259_Q[i], DRV_BIT((q shr i) and 1));
end;

{ ------------------------------------------------------------------------ }
{ Bus buffers                                                               }
{ ------------------------------------------------------------------------ }

{ 8T97: hex three-state buffer, non-inverting. DIS1 (1) high disables
  buffers 1-4, DIS2 (15) buffers 5-6. }
const
  T97: array[0..11] of Integer = [2, 3, 4, 5, 6, 7, 10, 9, 12, 11, 14, 13];

procedure t8t97_eval(s: TSim; p: TPart);
var
  i: Integer;
  dis: Boolean;
begin
  for i := 0 to 5 do begin
    if i < 4 then dis := s.PinIn(p, 1) <> 0 else dis := s.PinIn(p, 15) <> 0;
    if dis then s.PinOut(p, T97[i * 2 + 1], DRV_Z)
    else s.PinOut(p, T97[i * 2 + 1], DRV_BIT(s.PinIn(p, T97[i * 2])));
  end;
end;

{ 8T28: quad bidirectional bus transceiver, non-inverting. Driver: DriverIn
  -> Bus while DE (15) is high; receiver: Bus -> Out while /RE (1) is low. }
const
  T28: array[0..11] of Integer = [4, 3, 2, 7, 6, 5, 9, 10, 11, 12, 13, 14];   { DriverIn, Bus, Out }

procedure t8t28_eval(s: TSim; p: TPart);
var
  i: Integer;
  de, re: Boolean;
begin
  de := s.PinIn(p, 15) <> 0;
  re := s.PinIn(p, 1) = 0;
  for i := 0 to 3 do begin
    if de then s.PinOut(p, T28[i * 3 + 1], DRV_BIT(s.PinIn(p, T28[i * 3])))
    else s.PinOut(p, T28[i * 3 + 1], DRV_Z);
    if re then s.PinOut(p, T28[i * 3 + 2], DRV_BIT(s.PinIn(p, T28[i * 3 + 1])))
    else s.PinOut(p, T28[i * 3 + 2], DRV_Z);
  end;
end;

{ ------------------------------------------------------------------------ }
{ Memories                                                                  }
{ ------------------------------------------------------------------------ }

{ 2316/9316 ROM (2K x 8): A0-A10, CS2 (18) high, /CS1 (20) and /CS3 (21) low. }
const
  R2316_A: array[0..10] of Integer = [8, 7, 6, 5, 4, 3, 2, 1, 23, 22, 19];
  R2316_D: array[0..7] of Integer = [9, 10, 11, 13, 14, 15, 16, 17];

function NewRom2316: TObject;
begin
  Result := TRom2316State.Create;
end;

procedure rom2316_eval(s: TSim; p: TPart);
var
  st: TRom2316State;
  sel: Boolean;
  addr, v: Integer;
begin
  st := TRom2316State(p.state);
  sel := (s.PinIn(p, 18) <> 0) and (s.PinIn(p, 20) = 0) and (s.PinIn(p, 21) = 0);
  if sel then addr := part_read_bus(s, p, GK_TTL + 1, R2316_A, 11) else addr := 0;
  if (st.data <> nil) and (addr < st.data.length) then v := st.data[addr] else v := $FF;
  st.out_cache := part_drive_bus_cached(s, p, R2316_D, 8, v, sel, st.out_cache);
end;

{ 2513 character generator: 64 characters x 8 rows x 5 dots. A1-A3 (14-16)
  the row, A4-A9 (17-22) the character, /CE (11) low, O1-O5 (4-8). The
  image holds a byte per row (bits 0-4 the dots, O1 = bit 0). }
const
  C2513_A: array[0..8] of Integer = [14, 15, 16, 17, 18, 19, 20, 21, 22];
  C2513_O: array[0..4] of Integer = [4, 5, 6, 7, 8];

procedure c2513_eval(s: TSim; p: TPart);
var
  st: TRom2316State;
  sel: Boolean;
  addr, v: Integer;
begin
  st := TRom2316State(p.state);
  sel := s.PinIn(p, 11) = 0;
  addr := part_read_bus(s, p, GK_TTL + 2, C2513_A, 9);
  if (st.data <> nil) and (addr < st.data.length) then v := st.data[addr] else v := $1F;
  st.out_cache := part_drive_bus_cached(s, p, C2513_O, 5, v, sel, st.out_cache);
end;

{ ------------------------------------------------------------------------ }
{ Timers                                                                    }
{ ------------------------------------------------------------------------ }

{ 555 as the Apple's power-on reset: the output is high while the timing
  capacitor charges after power on, then low. Modeled by what it does. }
function NewTimer555: TObject;
begin
  Result := TTimer555State.Create;
end;

procedure t555_eval(s: TSim; p: TPart);
var st: TTimer555State;
begin
  st := TTimer555State(p.state);
  if s.PinIn(p, 4) = 0 then s.PinOut(p, 3, DRV_LO)
  else s.PinOut(p, 3, DRV_BIT(s.cycle < st.until_cycle));
end;

procedure t555_commit(s: TSim; p: TPart);
var
  st: TTimer555State;
  hi, isHi: Boolean;
begin
  st := TTimer555State(p.state);
  hi := s.cycle < st.until_cycle;
  isHi := s.pins[p.Pin(3)].drive = DRV_HI;      { (Quartex: no "b <> (x = y)") }
  if hi xor isHi then s.MarkDirty(p.index);
end;

procedure t555_reset(s: TSim; p: TPart);
begin
  TTimer555State(p.state).until_cycle := s.cycle + 0;   { set by the machine }
end;

{ NE558 quad timer (game paddles): with nothing plugged into the game port
  the timing capacitors never charge: the outputs stay as triggered. Not
  simulated beyond keeping the outputs low. }
procedure ne558_eval(s: TSim; p: TPart);
begin
  s.PinOut(p, 1, DRV_LO);
  s.PinOut(p, 8, DRV_LO);
  s.PinOut(p, 9, DRV_LO);
  s.PinOut(p, 16, DRV_LO);
end;

initialization
  PT_74LS00x := TPartType.Create('74LS00', '1A:T 1B:T 1Y:O 2A:T 2B:T 2Y:O GND:P 3Y:O 3A:T 3B:T 4Y:O 4A:T 4B:T VCC:P',
    ls00_eval, nil, nil, nil, TECH_TTL);
  PT_74LS02 := TPartType.Create('74LS02', '1Y:O 1A:T 1B:T 2Y:O 2A:T 2B:T GND:P 3A:T 3B:T 3Y:O 4A:T 4B:T 4Y:O VCC:P',
    ls02_eval, nil, nil, nil, TECH_TTL);
  PT_74LS04 := TPartType.Create('74LS04', '1A:T 1Y:O 2A:T 2Y:O 3A:T 3Y:O GND:P 4Y:O 4A:T 5Y:O 5A:T 6Y:O 6A:T VCC:P',
    ls04_eval, nil, nil, nil, TECH_TTL);
  PT_74LS08x := TPartType.Create('74LS08', '1A:T 1B:T 1Y:O 2A:T 2B:T 2Y:O GND:P 3Y:O 3A:T 3B:T 4Y:O 4A:T 4B:T VCC:P',
    ls08_eval, nil, nil, nil, TECH_TTL);
  PT_74LS11 := TPartType.Create('74LS11', '1A:T 1B:T 2A:T 2B:T 2C:T 2Y:O GND:P 3Y:O 3A:T 3B:T 3C:T 1Y:O 1C:T VCC:P',
    ls11_eval, nil, nil, nil, TECH_TTL);
  PT_74LS20 := TPartType.Create('74LS20', '1A:T 1B:T NC:O 1C:T 1D:T 1Y:O GND:P 2Y:O 2A:T 2B:T NC:O 2C:T 2D:T VCC:P',
    ls20_eval, nil, nil, nil, TECH_TTL);
  PT_74LS32x := TPartType.Create('74LS32', '1A:T 1B:T 1Y:O 2A:T 2B:T 2Y:O GND:P 3Y:O 3A:T 3B:T 4Y:O 4A:T 4B:T VCC:P',
    ls32_eval, nil, nil, nil, TECH_TTL);
  PT_74LS51 := TPartType.Create('74LS51', '1A:T 2A:T 2B:T 2C:T 2D:T 2Y:O GND:P 1Y:O 1F:T 1E:T 1D:T 1C:T 1B:T VCC:P',
    ls51_eval, nil, nil, nil, TECH_TTL);
  PT_74LS86 := TPartType.Create('74LS86', '1A:T 1B:T 1Y:O 2A:T 2B:T 2Y:O GND:P 3Y:O 3A:T 3B:T 4Y:O 4A:T 4B:T VCC:P',
    ls86_eval, nil, nil, nil, TECH_TTL);
  PT_74LS74x := TPartType.Create('74LS74', '/1R:T 1D:T 1C:T /1S:T 1Q:O /1Q:O GND:P /2Q:O 2Q:O /2S:T 2C:T 2D:T /2R:T VCC:P',
    ls74_eval, ls74_commit, nil, NewReg, TECH_TTL);
  PT_74LS138 := TPartType.Create('74LS138', 'A0:T A1:T A2:T /E1:T /E2:T E3:T O7:O GND:P O6:O O5:O O4:O O3:O O2:O O1:O O0:O VCC:P',
    ls138_eval, nil, nil, nil, TECH_TTL);
  PT_74LS139x := TPartType.Create('74LS139', '1/E:T 1A0:T 1A1:T 1O0:O 1O1:O 1O2:O 1O3:O GND:P 2O3:O 2O2:O 2O1:O 2O0:O 2A1:T 2A0:T 2/E:T VCC:P',
    ls139_eval, nil, nil, nil, TECH_TTL);
  PT_74LS151 := TPartType.Create('74LS151', 'I3:T I2:T I1:T I0:T Z:O /Z:O /E:T GND:P S2:T S1:T S0:T I7:T I6:T I5:T I4:T VCC:P',
    ls151_eval, nil, nil, nil, TECH_TTL);
  PT_74LS153 := TPartType.Create('74LS153', '/Ea:T S1:T I3a:T I2a:T I1a:T I0a:T Za:O GND:P Zb:O I0b:T I1b:T I2b:T I3b:T S0:T /Eb:T VCC:P',
    ls153_eval, nil, nil, nil, TECH_TTL);
  PT_74LS161 := TPartType.Create('74LS161', '/MR:T CP:T D0:T D1:T D2:T D3:T CEP:T GND:P /PE:T CET:T Q3:O Q2:O Q1:O Q0:O TC:O VCC:P',
    ls161_eval, ls161_commit, nil, NewReg, TECH_TTL);
  PT_74LS166 := TPartType.Create('74LS166', 'DS:T A:T B:T C:T D:T CE:T CLK:T GND:P /CLR:T E:T F:T G:T QH:O H:T /PE:T VCC:P',
    ls166_eval, ls166_commit, nil, NewReg, TECH_TTL);
  PT_74LS174 := TPartType.Create('74LS174', '/MR:T Q0:O D0:T D1:T Q1:O D2:T Q2:O GND:P CP:T Q3:O D3:T Q4:O D4:T D5:T Q5:O VCC:P',
    ls174_eval, ls174_commit, nil, NewReg, TECH_TTL);
  PT_74LS175 := TPartType.Create('74LS175', '/CLR:T 1Q:O /1Q:O 1D:T 2D:T /2Q:O 2Q:O GND:P CLK:T 3Q:O /3Q:O 3D:T 4D:T /4Q:O 4Q:O VCC:P',
    ls175_eval, ls175_commit, nil, NewReg, TECH_TTL);
  PT_74LS194 := TPartType.Create('74LS194', '/MR:T DSR:T P0:T P1:T P2:T P3:T DSL:T GND:P S0:T S1:T CP:T Q3:O Q2:O Q1:O Q0:O VCC:P',
    ls194_eval, ls194_commit, nil, NewReg, TECH_TTL);
  PT_74LS195 := TPartType.Create('74LS195', '/MR:T J:T /K:T P0:T P1:T P2:T P3:T GND:P /PE:T CP:T /Q3:O Q3:O Q2:O Q1:O Q0:O VCC:P',
    ls195_eval, ls195_commit, nil, NewReg, TECH_TTL);
  PT_74LS251 := TPartType.Create('74LS251', 'I3:T I2:T I1:T I0:T Y:O /Y:O /OE:T GND:P S2:T S1:T S0:T I7:T I6:T I5:T I4:T VCC:P',
    ls251_eval, nil, nil, nil, TECH_TTL);
  PT_74LS257x := TPartType.Create('74LS257', 'S:T I0a:T I1a:T Za:O I0b:T I1b:T Zb:O GND:P Zd:O I1d:T I0d:T Zc:O I1c:T I0c:T OE:T VCC:P',
    ls257_eval, nil, nil, nil, TECH_TTL);
  PT_74LS259 := TPartType.Create('74LS259', 'A0:T A1:T A2:T Q0:O Q1:O Q2:O Q3:O GND:P Q4:O Q5:O Q6:O Q7:O D:T /E:T /CLR:T VCC:P',
    ls259_eval, nil, nil, NewReg, TECH_TTL);
  PT_74LS283 := TPartType.Create('74LS283', 'S2:O B2:T A2:T S1:O A1:T B1:T C0:T GND:P C4:O S4:O B4:T A4:T S3:O A3:T B3:T VCC:P',
    ls283_eval, nil, nil, nil, TECH_TTL);
  PT_8T97 := TPartType.Create('8T97', 'DIS1:T I1:T Y1:O I2:T Y2:O I3:T Y3:O GND:P Y4:O I4:T Y5:O I5:T Y6:O I6:T DIS2:T VCC:P',
    t8t97_eval, nil, nil, nil, TECH_TTL);
  PT_8T28 := TPartType.Create('8T28', '/RE:T AOUT:O ABUS:X ADIN:T BOUT:O BBUS:X BDIN:T GND:P CDIN:T CBUS:X COUT:O DDIN:T DBUS:X DOUT:O DE:T VCC:P',
    t8t28_eval, nil, nil, nil, TECH_TTL);
  PT_2316 := TPartType.Create('9316B ROM',
    'A7:M A6:M A5:M A4:M A3:M A2:M A1:M A0:M D0:O D1:O D2:O GND:P D3:O D4:O D5:O D6:O D7:O CS2:M A10:M /CS1:M /CS3:M A9:M A8:M VCC:P',
    rom2316_eval, nil, nil, NewRom2316, TECH_NMOS);
  PT_2513 := TPartType.Create('2513 CHAR ROM',
    'VGG:P NC:O NC:O O1:O O2:O O3:O O4:O O5:O NC:O GND:P /CE:M VDD:P NC:O A1:M A2:M A3:M A4:M A5:M A6:M A7:M A8:M A9:M NC:O VCC:P',
    c2513_eval, nil, nil, NewRom2316, TECH_NMOS);
  PT_555A := TPartType.Create('555', 'GND:P TRIG:m OUT:O /RESET:M CONT:O THRES:m DISCH:O VCC:P',
    t555_eval, t555_commit, nil, NewTimer555, TECH_TTL);
  PT_NE558 := TPartType.Create('NE558', 'OUTA:O TA:O TRA:m CV:O VCC:P TRB:m TB:O OUTB:O OUTC:O TC:O TRC:m GND:P /RESET:m TRD:m TD:O OUTD:O',
    ne558_eval, nil, nil, nil, TECH_TTL);
  { clocked parts whose commit only looks at their inputs (no time, no noise) }
  PT_74LS74x.edge_commit := True; PT_74LS74x.edge_pins := [1, 3, 4, 10, 11, 13];
  PT_74LS161.edge_commit := True; PT_74LS161.edge_pins := [1, 2];
  PT_74LS166.edge_commit := True; PT_74LS166.edge_pins := [7, 9];
  PT_74LS174.edge_commit := True; PT_74LS174.edge_pins := [1, 9];
  PT_74LS175.edge_commit := True; PT_74LS175.edge_pins := [1, 9];
  PT_74LS194.edge_commit := True; PT_74LS194.edge_pins := [1, 11];
  PT_74LS195.edge_commit := True; PT_74LS195.edge_pins := [1, 10];
end.
