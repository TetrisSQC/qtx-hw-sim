unit hwsim.sim;

{ The electrical net engine (port of sim.h / sim.c).

  The board is a set of PARTS (chips, resistors, connectors...). Every part
  has PINS, and every pin sits on a NET (a piece of copper). At any moment a
  pin may DRIVE its net: low or high, with a strength. The engine RESOLVES
  each net from all the drives on it:

    SUPPLY  shorts to GND/+5V and the power rails themselves
    STRONG  normal chip outputs
    RES     an output seen through a series resistor
    PULL    pull-up resistors, weak NMOS outputs (CIA port lines)
    Z       nobody drives: the net is FLOATING and keeps its last level

  The strongest drive wins. Two drives of the same strength fighting with
  different levels is a CONTENTION: low wins and the net is flagged.

  What a chip READS from a floating net depends on its input technology:
  TTL inputs read 1, MOS inputs read the charge left on the net.

  Parts are evaluated only when one of their input nets changes, in a loop
  that runs until the whole board is stable (settle). Parts that have memory
  sample their inputs in a separate commit pass, after the board settled.

  Nets can be joined (links, pressed keys) with a union-find: all the
  electrical state lives in the ROOT of a merged net.

  Port notes: 64-bit values of the C code are represented as
  - cycle counters: Float (exact up to 2^53);
  - the bus bitmap (at most 64 lines): two 32-bit words, lo (bits 0..31)
    and hi (bits 32..63). A group of pins read together must sit in one word;
  - the noise generator (xorshift64): a lo/hi pair, bit exact with the C. }

interface

uses
  hwsim.util, hwsim.boarddata;

const
  { Drive strengths. }
  S_Z = 0; S_PULL = 1; S_RES = 2; S_STRONG = 3; S_SUPPLY = 4;

  { A drive packs strength and level in one value: (strength shl 1) or level. }
  DRV_Z = 0;
  DRV_LO = 6;       { MkDrive(S_STRONG, 0) }
  DRV_HI = 7;       { MkDrive(S_STRONG, 1) }
  DRV_PULLUP = 3;   { MkDrive(S_PULL, 1) }

  { Pin faults. }
  PF_OK = 0; PF_OPEN = 1; PF_GND = 2; PF_VCC = 3;

  { Pin flags. }
  PIN_TTL = 1;      { bipolar input: floating reads as 1 }
  PIN_SENSE = 2;    { re-evaluate the part when this net changes }
  PIN_POWER = 4;    { VCC/GND pin: if open, the chip is dead }
  PIN_SWITCH = 8;   { analog switch pin: re-evaluate on any drive change }
  PIN_OUT = 16;     { the chip can drive this pin }
  PIN_IN = 32;      { the chip reads this pin }

  { Chip technology. }
  TECH_NMOS = 0; TECH_TTL = 1; TECH_TTL_OC = 2; TECH_CMOS = 3;

  { Power state of a part. }
  PWR_OK = 0; PWR_OFF = 1; PWR_NO_VCC = 2; PWR_NO_GND = 3;

  { Part kinds. }
  PART_IC = 0; PART_RESISTOR = 1; PART_LINK = 2; PART_CONNECTOR = 3; PART_OTHER = 4;

  { Resistor / link faults. }
  RF_OK = 0; RF_OPEN = 1; RF_SHORT = 2;

  { Rails. }
  RAIL_NONE = 0; RAIL_GND = 1; RAIL_VCC = 2;

  PART_GROUPS = 4;
  SIM_MAX_PARTS = 256;
  SIM_QUEUE_SIZE = 512;

type
  TSim = class;
  TPart = class;

  TPartFn = procedure(s: TSim; p: TPart);

  TPin = class
  public
    part: Integer;      { owning part index }
    number: Integer;    { pin number (1..N), 0 if the pad name isn't a number }
    net: Integer;       { net index, -1 if unconnected }
    root: Integer;      { root of the merged net }
    drive: Integer;     { what the part currently drives on this pin }
    eff: Integer;       { the drive actually counted on the net }
    fault: Integer;     { PF_* }
    flags: Integer;     { PIN_* }
    held: Integer;      { level a MOS input had when its leg was lifted }
    locked: Boolean;    { the drive can't reach the net }
  end;

  TNet = class
  public
    count: array[0..9] of Integer;  { drives of each kind, index = drive value }
    level: Integer;
    strength: Integer;
    contention: Integer;
    rail: Integer;
    readers: array of Integer;
    committers: array of Integer;   { edge_commit parts with an input on the net }
    switches: array of Integer;
    busLo, busHi: Integer;          { bus bitmap bits merged into this root }
    float_since: Float;
    hold_until: Float;
    ttl_loaded: Integer;
    root: Integer;
    busbit: Integer;
    pins: array of Integer;
    name: String;
  end;

  { A group of pins read together, cached per part. }
  TGroup = class
  public
    key: Integer;       { identifies the group (0: unused slot) }
    n: Integer;
    epoch: Integer;
    first: Integer;     { bus bitmap bit of the group's first pin, or -1 }
    word: Integer;      { 0: lo word, 1: hi word }
    shift: Integer;     { first mod 32 }
    mask, ttl: Integer;
  end;

  TPart = class
  public
    index: Integer;
    ref: String;
    name: String;
    kind: Integer;
    first_pin, npins: Integer;
    pinmap: array[0..47] of Integer;
    removed: Boolean;
    no_supply: Boolean;
    tech: Integer;
    power: Integer;
    dead: Boolean;
    rfault: Integer;
    eval: TPartFn;
    commit: TPartFn;
    edge_commit: Boolean;       { commit only acts on input changes (see TSim.edge_commits) }
    edge_pins: array of Integer;  { ... on these pins (clocks, async clears); empty: all inputs }
    reset: TPartFn;
    state: TObject;
    user: TObject;
    groups: array[0..PART_GROUPS - 1] of TGroup;
    constructor Create;
    function Pin(number: Integer): Integer;
  end;

  TRailOfFn = function(const net: String): Integer;
  TKindOfFn = function(const ref: String): Integer;

  TSimDesc = class
  public
    rail_of: TRailOfFn;
    kind_of: TKindOfFn;
    cycles_per_us: Float;
  end;

  TSim = class
  private
    FJoinA, FJoinB: array of Integer;
    function FindRoot(n: Integer): Integer;
    procedure Resolve(root, cause_part: Integer);
    procedure AddCommitter(pin, part: Integer);
    procedure MirrorAll;
    procedure ComputeHoldTimes;
    procedure SetHold(n: TNet);
    function PhantomPowered(p: TPart): Boolean;
    procedure GroupSetup(p: TPart; g: TGroup; const numbers: array of Integer; n: Integer);
  public
    bd: TBoardData;
    time_scale: Float;
    pins: array of TPin;
    npins: Integer;
    pin_names: array of String;
    nets: array of TNet;
    nnets: Integer;
    nread: TBytes;              { per net: bit 0 level, bit 1 floating, bit 2 erratic }
    busLevelLo, busLevelHi: Integer;
    busFloatLo, busFloatHi: Integer;
    busErrLo, busErrHi: Integer;
    epoch: Integer;
    parts: array of TPart;
    nparts: Integer;
    commit_list: array of Integer;
    ncommit: Integer;
    dirty: TBytes;
    { Speed-up: commit an edge_commit part only when one of its inputs changed
      since its last commit (its commit is then a no-op anyway). Off while a
      lifted MOS leg makes inputs noisy. }
    edge_commits: Boolean;
    cdirty: TBytes;
    queue: TInts;
    qhead, qtail: Integer;
    cycle: Float;               { board clock, in CPU cycles }
    changes: Integer;           { counts net changes }
    clock_level: Integer;       { PHI0 level of the current half cycle }
    rngLo, rngHi: Integer;      { xorshift64 state }
    hold_time: array[0..3] of Float;   { [ttl_loaded * 2 + level] }
    open_since: TFloats;        { per pin: cycle its leg was lifted }
    noisy: array of Integer;
    nnoisy: Integer;

    constructor Create(abd: TBoardData; desc: TSimDesc);
    procedure Rebuild;
    procedure JoinBegin;
    procedure Join(a, b: Integer);
    procedure Discharge;
    procedure Seed(aseed: Integer);
    procedure WakeErratic;
    function Noise: Float;
    procedure SetBusBit(net, bit: Integer);

    procedure Drive(pin, d: Integer);
    procedure DriveChanged(p: TPin);
    function Read(pin: Integer): Integer;
    function ReadFloating(pin: Integer): Integer;
    function ReadOpen(pin: Integer): Integer;
    { Read a group of pins of a part (pin numbers in 'numbers', bit 0 first)
      as a binary number. 'key' must be unique for the pin number array. }
    function ReadGroup(p: TPart; key: Integer; const numbers: array of Integer; n: Integer): Integer;

    procedure SetPinFault(pin, fault: Integer);
    function DriveExcept(pin_on_net, except_pin: Integer): Integer;
    function PinDrive(p: TPin): Integer;

    procedure MarkDirty(part: Integer);
    procedure Settle;
    procedure Commit;
    procedure UpdatePartPower(part: Integer);
    function FindPart(const ref: String): Integer;
    function FindNet(const name: String): Integer;

    { pin_out / pin_in of the C code. }
    procedure PinOut(p: TPart; number, d: Integer);
    function PinIn(p: TPart; number: Integer): Integer;
  end;

function MkDrive(strength, level: Integer): Integer;
function DRV_BIT(v: Integer): Integer; overload;
function DRV_BIT(v: Boolean): Integer; overload;
function Threshold(pin: Integer): Float;

implementation

const
  V_HIGH = 5.0;
  V_TH = 1.4;
  V_TH_SPREAD = 0.25;
  V_NOISE = 0.08;
  V_BAND = 0.25 + 4 * 0.08;
  V_TTL_REST = 1.4;
  TAU_TTL = 2.0;
  TAU_MOS = 300000.0;
  V_OPEN_REST = 1.4;
  TAU_OPEN = 5000.0;
  K_CROSSTALK = 0.12;

function MkDrive(strength, level: Integer): Integer;
begin
  Result := (strength shl 1) or (level and 1);
end;

function DRV_BIT(v: Integer): Integer;
begin
  if v <> 0 then Result := DRV_HI else Result := DRV_LO;
end;

function DRV_BIT(v: Boolean): Integer;
begin
  if v then Result := DRV_HI else Result := DRV_LO;
end;

{ ------------------------------------------------------------------------ }
{ TPart                                                                     }
{ ------------------------------------------------------------------------ }

constructor TPart.Create;
var i: Integer;
begin
  inherited Create;
  for i := 0 to 47 do pinmap[i] := -1;
  for i := 0 to PART_GROUPS - 1 do groups[i] := TGroup.Create;
end;

function TPart.Pin(number: Integer): Integer;
begin
  Result := pinmap[number];
end;

{ ------------------------------------------------------------------------ }
{ Setup                                                                     }
{ ------------------------------------------------------------------------ }

constructor TSim.Create(abd: TBoardData; desc: TSimDesc);
var
  i, c, k, pi: Integer;
  n: TNet;
  p: TPart;
  pin: TPin;
begin
  inherited Create;
  bd := abd;
  time_scale := desc.cycles_per_us;
  npins := bd.npads;
  nnets := bd.nnets;
  pins.SetLength(npins);
  pin_names.SetLength(npins);
  for i := 0 to npins - 1 do begin
    pins[i] := TPin.Create;
    pin_names[i] := '';
  end;
  nets.SetLength(nnets);
  nread := NewBytes(nnets);
  open_since := NewFloats(npins);
  dirty := NewBytes(SIM_MAX_PARTS);
  cdirty := NewBytes(SIM_MAX_PARTS);
  queue := NewInts(SIM_QUEUE_SIZE);
  Seed(1);
  ComputeHoldTimes;
  for i := 0 to nnets - 1 do begin
    n := TNet.Create;
    nets[i] := n;
    n.name := bd.nets[i];
    n.root := i;
    n.busbit := -1;
    n.rail := desc.rail_of(bd.nets[i]);
    if n.rail = RAIL_VCC then n.level := 1 else n.level := 0;
  end;
  { One part per footprint, one pin per pad. }
  nparts := 0;
  c := 0;
  while (c < bd.ncomps) and (nparts < SIM_MAX_PARTS) do begin
    p := TPart.Create;
    p.index := nparts;
    p.ref := bd.comps[c].ref;
    p.name := bd.comps[c].value;
    p.kind := desc.kind_of(p.ref);
    p.first_pin := bd.comps[c].first_pad;
    p.npins := bd.comps[c].npads;
    for k := 0 to p.npins - 1 do begin
      pi := p.first_pin + k;
      pin := pins[pi];
      pin.part := nparts;
      pin.net := bd.pads[pi].net;
      pin.root := pin.net;
      pin.number := AtoI(bd.pads[pi].name);
      if (pin.number > 0) and (pin.number < 48) then p.pinmap[pin.number] := pi;
    end;
    parts.Add(p);
    Inc(nparts);
    Inc(c);
  end;
  Rebuild;
end;

procedure TSim.SetBusBit(net, bit: Integer);
begin
  if (net >= 0) and (bit >= 0) and (bit < 64) then nets[net].busbit := bit;
end;

procedure TSim.MirrorAll;
var
  i, floating: Integer;
  n: TNet;
begin
  busLevelLo := 0; busLevelHi := 0;
  busFloatLo := 0; busFloatHi := 0;
  busErrLo := 0; busErrHi := 0;
  for i := 0 to nnets - 1 do begin
    n := nets[i];
    if n.root <> i then continue;
    if n.strength = S_Z then floating := 1 else floating := 0;
    nread[i] := n.level or (floating shl 1);
    if n.level <> 0 then begin
      busLevelLo := busLevelLo or n.busLo;
      busLevelHi := busLevelHi or n.busHi;
    end;
    if floating <> 0 then begin
      busFloatLo := busFloatLo or n.busLo;
      busFloatHi := busFloatHi or n.busHi;
    end;
  end;
end;

procedure TSim.Discharge;
var i: Integer;
begin
  for i := 0 to nnets - 1 do begin
    if nets[i].rail = RAIL_VCC then nets[i].level := 1 else nets[i].level := 0;
    nets[i].float_since := CYCLE_NEVER;   { restarted by the next rebuild }
  end;
  { Lifted MOS legs lose their charge too. }
  for i := 0 to npins - 1 do begin
    pins[i].held := 0;
    open_since[i] := 0;
  end;
  MirrorAll;
end;

function TSim.FindPart(const ref: String): Integer;
var i: Integer;
begin
  for i := 0 to nparts - 1 do
    if parts[i].ref = ref then exit(i);
  Result := -1;
end;

function TSim.FindNet(const name: String): Integer;
var i: Integer;
begin
  for i := 0 to nnets - 1 do
    if nets[i].name = name then exit(i);
  Result := -1;
end;

{ ------------------------------------------------------------------------ }
{ Merging nets (union-find)                                                 }
{ ------------------------------------------------------------------------ }

procedure TSim.JoinBegin;
begin
  FJoinA.Clear;
  FJoinB.Clear;
end;

procedure TSim.Join(a, b: Integer);
begin
  if (a < 0) or (b < 0) then exit;
  FJoinA.Add(a);
  FJoinB.Add(b);
end;

function TSim.FindRoot(n: Integer): Integer;
begin
  while nets[n].root <> n do begin
    nets[n].root := nets[nets[n].root].root;   { path halving }
    n := nets[n].root;
  end;
  Result := n;
end;

{ ------------------------------------------------------------------------ }
{ Floating inputs                                                           }
{ ------------------------------------------------------------------------ }

procedure TSim.Seed(aseed: Integer);
var
  a0, a1, a2, a3, b0, b1, b2, b3: Integer;
  r: array[0..7] of Integer;
  i, j, carry, t: Integer;
begin
  { rng = seed * 0x9E3779B97F4A7C15 + 1 (mod 2^64), with 16-bit limbs. }
  a0 := $7C15; a1 := $7F4A; a2 := $79B9; a3 := $9E37;
  b0 := aseed and $FFFF; b1 := (aseed shr 16) and $FFFF; b2 := 0; b3 := 0;
  for i := 0 to 7 do r[i] := 0;
  var av: array[0..3] of Integer := [a0, a1, a2, a3];
  var bv: array[0..3] of Integer := [b0, b1, b2, b3];
  for i := 0 to 3 do begin
    carry := 0;
    for j := 0 to 3 - i do begin
      t := r[i + j] + av[j] * bv[i] + carry;
      r[i + j] := t mod 65536;
      carry := (t - r[i + j]) div 65536;
    end;
  end;
  { + 1 }
  carry := 1;
  for i := 0 to 3 do begin
    t := r[i] + carry;
    r[i] := t mod 65536;
    carry := t div 65536;
  end;
  rngLo := (r[0] or (r[1] shl 16)) shr 0;
  rngHi := (r[2] or (r[3] shl 16)) shr 0;
end;

function TSim.Noise: Float;
var
  sum: Float;
  i, lo, hi, nlo, nhi: Integer;
begin
  sum := 0;   { sum of 4 uniforms: close enough to a gaussian }
  lo := rngLo; hi := rngHi;
  for i := 0 to 3 do begin
    { rng ^= rng << 13 }
    nhi := hi xor ((hi shl 13) or (lo shr 19));
    nlo := lo xor (lo shl 13);
    lo := nlo; hi := nhi;
    { rng ^= rng >> 7 }
    nlo := lo xor ((lo shr 7) or (hi shl 25));
    nhi := hi xor (hi shr 7);
    lo := nlo; hi := nhi;
    { rng ^= rng << 17 }
    nhi := hi xor ((hi shl 17) or (lo shr 15));
    nlo := lo xor (lo shl 17);
    lo := nlo; hi := nhi;
    { (rng >> 11) / 2^53 }
    { (2^53 in two exact steps: Quartex prints float literals with 15 digits) }
    sum := sum + (U32(hi) * 2097152.0 + (lo shr 11)) / 4294967296.0 / 2097152.0;
  end;
  rngLo := lo; rngHi := hi;
  Result := (sum - 2.0) * 1.732 * V_NOISE;
end;

{ The threshold of a pin: fixed, but different from pin to pin. }
function Threshold(pin: Integer): Float;
var h: Integer;
begin
  h := U32(Imul(pin, 2654435761));
  h := U32(h xor (h shr 15));
  Result := V_TH + ((h and 1023) / 1023.0 * 2 - 1) * V_TH_SPREAD;
end;

function Drift(v0, rest, tau, age: Float): Float;
begin
  Result := rest + (v0 - rest) * Exp(-age / tau);
end;

procedure TSim.ComputeHoldTimes;
var
  t, l: Integer;
  v0, rest, tau, lo, hi, d, lim: Float;
  inf: Boolean;
begin
  for t := 0 to 1 do
    for l := 0 to 1 do begin
      if l <> 0 then v0 := V_HIGH else v0 := 0;
      if t <> 0 then rest := V_TTL_REST else rest := 0;
      if t <> 0 then tau := TAU_TTL * time_scale else tau := TAU_MOS * time_scale;
      lo := V_TH - V_BAND; hi := V_TH + V_BAND;
      inf := False;
      d := 0;
      if (v0 >= lo) and (v0 <= hi) then d := 0
      else if ((v0 < lo) and (rest < lo)) or ((v0 > hi) and (rest > hi)) then inf := True
      else begin
        if v0 < lo then lim := lo else lim := hi;
        d := tau * Ln((v0 - rest) / (lim - rest));
      end;
      if inf then hold_time[t * 2 + l] := CYCLE_NEVER
      else hold_time[t * 2 + l] := Trunc(d);
    end;
end;

procedure TSim.SetHold(n: TNet);
var d: Float;
begin
  d := hold_time[n.ttl_loaded * 2 + n.level];
  if d >= CYCLE_NEVER then n.hold_until := CYCLE_NEVER
  else n.hold_until := n.float_since + d;
end;

function TSim.ReadFloating(pin: Integer): Integer;
var
  n: TNet;
  v, v0, rest, tau: Float;
begin
  n := nets[pins[pin].root];
  if n.level <> 0 then v0 := V_HIGH else v0 := 0;
  if n.ttl_loaded <> 0 then begin rest := V_TTL_REST; tau := TAU_TTL; end
  else begin rest := 0; tau := TAU_MOS; end;
  v := Drift(v0, rest, tau * time_scale, cycle - n.float_since);
  if v + Noise > Threshold(pin) then Result := 1 else Result := 0;
end;

function TSim.ReadOpen(pin: Integer): Integer;
var
  p: TPin;
  part: TPart;
  v, v0: Float;
  half, num, d, nb, pi: Integer;
begin
  p := pins[pin];
  part := parts[p.part];
  if p.held <> 0 then v0 := V_HIGH else v0 := 0;
  v := Drift(v0, V_OPEN_REST, TAU_OPEN * time_scale, cycle - open_since[pin]);
  { Crosstalk from the neighbour pins on the same side of the package. }
  half := part.npins div 2;
  num := p.number;
  d := -1;
  while d <= 1 do begin
    nb := num + d;
    if not ((num <= 0) or (nb < 1) or (nb > part.npins) or ((num <= half) <> (nb <= half))) then begin
      pi := part.pinmap[nb];
      if not ((pi < 0) or ((pins[pi].flags and PIN_POWER) <> 0) or (pins[pi].root < 0)) then begin
        if (nread[pins[pi].root] and 1) <> 0 then v := v + K_CROSSTALK * (V_HIGH / 2)
        else v := v - K_CROSSTALK * (V_HIGH / 2);
      end;
    end;
    d := d + 2;
  end;
  if v + Noise > Threshold(pin) then Result := 1 else Result := 0;
end;

procedure TSim.WakeErratic;
var
  i, k: Integer;
  n: TNet;
begin
  for i := 0 to nnets - 1 do begin
    n := nets[i];
    if (n.root <> i) or (n.strength <> S_Z) or (cycle < n.hold_until) then continue;
    nread[i] := nread[i] or 4;
    busErrLo := busErrLo or n.busLo;
    busErrHi := busErrHi or n.busHi;
    for k := 0 to n.readers.High do MarkDirty(n.readers[k]);
    for k := 0 to n.committers.High do cdirty[n.committers[k]] := 1;
  end;
  for i := 0 to nnoisy - 1 do
    if Assigned(parts[noisy[i]].eval) then MarkDirty(noisy[i]);
end;

procedure TSim.AddCommitter(pin, part: Integer);
var
  p: TPin;
  n: TNet;
  k: Integer;
begin
  if pin < 0 then exit;
  p := pins[pin];
  if (p.root < 0) or ((p.flags and PIN_POWER) <> 0) then exit;
  n := nets[p.root];
  for k := 0 to n.committers.High do
    if n.committers[k] = part then exit;
  n.committers.Add(part);
end;

{ Rebuild the merged nets: roots, pin lists and reader lists. }
procedure TSim.Rebuild;
var
  i, j, k, ra, rb, t, bit: Integer;
  n: TNet;
  p: TPin;
  part: TPart;
  dup: Boolean;
begin
  for i := 0 to nnets - 1 do begin
    n := nets[i];
    n.root := i;
    n.pins.Clear;
    n.readers.Clear;
    n.committers.Clear;
    n.switches.Clear;
  end;
  for j := 0 to FJoinA.High do begin
    ra := FindRoot(FJoinA[j]);
    rb := FindRoot(FJoinB[j]);
    if ra = rb then continue;
    { Keep a rail as root, so the merged net stays a rail. }
    if (nets[rb].rail <> 0) and (nets[ra].rail = 0) then begin t := ra; ra := rb; rb := t; end;
    nets[rb].root := ra;
  end;
  for i := 0 to nnets - 1 do nets[i].root := FindRoot(i);
  for i := 0 to nnets - 1 do begin nets[i].busLo := 0; nets[i].busHi := 0; end;
  for i := 0 to nnets - 1 do begin
    bit := nets[i].busbit;
    if bit >= 0 then begin
      n := nets[nets[i].root];
      if bit < 32 then n.busLo := n.busLo or (1 shl bit)
      else n.busHi := n.busHi or (1 shl (bit - 32));
    end;
  end;

  { Pin lists. }
  for i := 0 to npins - 1 do begin
    p := pins[i];
    if p.net >= 0 then p.root := nets[p.net].root else p.root := -1;
    if p.root < 0 then continue;
    n := nets[p.root];
    n.pins.Add(i);
    if ((p.flags and PIN_SWITCH) <> 0) and (p.fault <> PF_OPEN) then
      n.switches.Add(p.part);
    if ((p.flags and PIN_SENSE) <> 0) and (p.fault <> PF_OPEN) and Assigned(parts[p.part].eval) then begin
      dup := False;
      for k := 0 to n.readers.High do
        if n.readers[k] = p.part then dup := True;
      if not dup then n.readers.Add(p.part);
    end;
  end;
  for i := 0 to nparts - 1 do begin
    part := parts[i];
    if not part.edge_commit then continue;
    if part.edge_pins.Length = 0 then begin
      for k := 0 to part.npins - 1 do
        AddCommitter(part.first_pin + k, i);
    end else
      for k := 0 to part.edge_pins.High do
        AddCommitter(part.pinmap[part.edge_pins[k]], i);
  end;
  for i := 0 to nparts - 1 do cdirty[i] := 1;
  { Commit list (parts with memory). }
  commit_list.Clear;
  for i := 0 to nparts - 1 do
    if Assigned(parts[i].commit) then commit_list.Add(i);
  ncommit := commit_list.Length;

  { Count the drives on every net, then resolve everything. }
  for i := 0 to nnets - 1 do
    for k := 0 to 9 do nets[i].count[k] := 0;
  for i := 0 to nnets - 1 do
    if nets[i].rail <> 0 then begin
      if nets[i].rail = RAIL_VCC then k := MkDrive(S_SUPPLY, 1) else k := MkDrive(S_SUPPLY, 0);
      n := nets[nets[i].root];
      n.count[k] := n.count[k] + 1;
    end;
  for i := 0 to npins - 1 do begin
    p := pins[i];
    part := parts[p.part];
    p.locked := (p.root < 0) or (p.fault <> PF_OK) or (part.power = PWR_OFF) or
                ((part.power <> PWR_OK) and (part.tech <> TECH_CMOS));
    if p.root >= 0 then p.eff := PinDrive(p) else p.eff := DRV_Z;
    if (p.eff shr 1) <> 0 then begin
      n := nets[p.root];
      n.count[p.eff] := n.count[p.eff] + 1;
    end;
  end;
  { Nets with a TTL input on them. }
  for i := 0 to nnets - 1 do nets[i].ttl_loaded := 0;
  for i := 0 to npins - 1 do begin
    p := pins[i];
    if (p.root >= 0) and ((p.flags and PIN_TTL) <> 0) and (p.fault <> PF_OPEN) and
       (parts[p.part].power <> PWR_OFF) then
      nets[p.root].ttl_loaded := 1;
  end;
  for i := 0 to nnets - 1 do
    if nets[i].root = i then Resolve(i, -1);
  { Floating nets: (re)compute how long their charge clearly holds. }
  for i := 0 to nnets - 1 do begin
    n := nets[i];
    if (n.root <> i) or (n.strength <> S_Z) then continue;
    if n.float_since > cycle then n.float_since := cycle;
    SetHold(n);
  end;
  { Parts with a lifted MOS input leg: their input is noisy. }
  noisy.Clear;
  for i := 0 to nparts - 1 do begin
    part := parts[i];
    if part.dead or (not Assigned(part.eval) and not Assigned(part.commit)) then continue;
    for k := 0 to part.npins - 1 do begin
      p := pins[part.first_pin + k];
      if (p.fault = PF_OPEN) and ((p.flags and PIN_IN) <> 0) and ((p.flags and PIN_TTL) = 0) then begin
        noisy.Add(i);
        break;
      end;
    end;
  end;
  nnoisy := noisy.Length;
  MirrorAll;
  Inc(epoch);   { pin groups must be looked at again }
  for i := 0 to nparts - 1 do
    if Assigned(parts[i].eval) then MarkDirty(i);
end;

{ ------------------------------------------------------------------------ }
{ Resolution                                                                }
{ ------------------------------------------------------------------------ }

function TSim.PinDrive(p: TPin): Integer;
var part: TPart;
begin
  case p.fault of
    PF_GND: exit(MkDrive(S_SUPPLY, 0));
    PF_VCC: exit(MkDrive(S_SUPPLY, 1));
    PF_OPEN: exit(DRV_Z);
  end;
  part := parts[p.part];
  if (part.power = PWR_OK) or (part.tech = TECH_CMOS) then begin
    if part.power = PWR_OFF then exit(DRV_Z) else exit(p.drive);
  end;
  if (part.power = PWR_OFF) or ((p.flags and PIN_POWER) <> 0) then exit(DRV_Z);
  if part.power = PWR_NO_VCC then exit(DRV_Z);
  { PWR_NO_GND }
  if part.tech = TECH_TTL_OC then exit(DRV_Z);
  if (p.flags and PIN_OUT) <> 0 then Result := MkDrive(S_PULL, 1) else Result := DRV_Z;
end;

procedure TSim.Resolve(root, cause_part: Integer);
var
  n: TNet;
  d, st, level, lo, hi, i, floating: Integer;
begin
  n := nets[root];
  { combine(): the strongest wins, equal ones fighting: low wins. }
  d := DRV_Z;
  n.contention := 0;
  st := S_SUPPLY;
  while st >= S_PULL do begin
    lo := n.count[st * 2];
    hi := n.count[st * 2 + 1];
    if (lo <> 0) or (hi <> 0) then begin
      if (lo <> 0) and (hi <> 0) then n.contention := 1;
      if lo <> 0 then d := st * 2 else d := st * 2 + 1;
      break;
    end;
    Dec(st);
  end;
  st := d shr 1;
  if st = S_Z then level := n.level else level := d and 1;
  for i := 0 to n.switches.High do
    if n.switches[i] <> cause_part then MarkDirty(n.switches[i]);
  if (level = n.level) and ((st = S_Z) = (n.strength = S_Z)) then begin
    n.strength := st;
    exit;
  end;
  if (st = S_Z) and (n.strength <> S_Z) then begin   { nobody drives it anymore }
    n.float_since := cycle;
    n.level := level;
    SetHold(n);
  end;
  n.level := level;
  n.strength := st;
  Inc(changes);
  if st = S_Z then floating := 1 else floating := 0;
  nread[root] := level or (floating shl 1);
  for i := 0 to n.committers.High do cdirty[n.committers[i]] := 1;
  if (n.busLo or n.busHi) <> 0 then begin
    if level <> 0 then begin
      busLevelLo := busLevelLo or n.busLo; busLevelHi := busLevelHi or n.busHi;
    end else begin
      busLevelLo := busLevelLo and not n.busLo; busLevelHi := busLevelHi and not n.busHi;
    end;
    if floating <> 0 then begin
      busFloatLo := busFloatLo or n.busLo; busFloatHi := busFloatHi or n.busHi;
    end else begin
      busFloatLo := busFloatLo and not n.busLo; busFloatHi := busFloatHi and not n.busHi;
    end;
    busErrLo := busErrLo and not n.busLo;
    busErrHi := busErrHi and not n.busHi;
  end;
  for i := 0 to n.readers.High do MarkDirty(n.readers[i]);
end;

procedure TSim.Drive(pin, d: Integer);
var p: TPin;
begin
  if pin < 0 then exit;
  p := pins[pin];
  if p.drive = d then exit;
  p.drive := d;
  if not p.locked then DriveChanged(p);
end;

procedure TSim.DriveChanged(p: TPin);
var
  eff: Integer;
  n: TNet;
begin
  eff := p.drive;
  if eff = p.eff then exit;
  n := nets[p.root];
  if (p.eff shr 1) <> 0 then n.count[p.eff] := n.count[p.eff] - 1;
  if (eff shr 1) <> 0 then n.count[eff] := n.count[eff] + 1;
  p.eff := eff;
  Resolve(p.root, p.part);
end;

function TSim.Read(pin: Integer): Integer;
var
  p: TPin;
  ttl, v: Integer;
begin
  if pin < 0 then exit(1);
  p := pins[pin];
  ttl := p.flags and PIN_TTL;
  { Lifted leg: a TTL input reads high, a MOS input floats on its own. }
  if (p.fault = PF_OPEN) or (p.root < 0) then begin
    if ttl <> 0 then exit(1);
    exit(ReadOpen(pin));
  end;
  v := nread[p.root];
  if ((v and 4) <> 0) and (ttl = 0) then exit(ReadFloating(pin));
  Result := (v and 1) or ((v shr 1) and ttl);
end;

procedure TSim.PinOut(p: TPart; number, d: Integer);
begin
  Drive(p.pinmap[number], d);
end;

function TSim.PinIn(p: TPart; number: Integer): Integer;
begin
  Result := Read(p.pinmap[number]);
end;

procedure TSim.SetPinFault(pin, fault: Integer);
var p: TPin;
begin
  p := pins[pin];
  if (fault = PF_OPEN) and (p.fault <> PF_OPEN) then begin
    p.held := Read(pin);
    open_since[pin] := cycle;
  end;
  p.fault := fault;
end;

function TSim.DriveExcept(pin_on_net, except_pin: Integer): Integer;
var
  p, x: TPin;
  n: TNet;
  skip, st, lo, hi: Integer;
begin
  p := pins[pin_on_net];
  if (p.root < 0) or (p.fault = PF_OPEN) then exit(DRV_Z);
  n := nets[p.root];
  x := pins[except_pin];
  if x.root = p.root then skip := x.eff else skip := DRV_Z;
  st := S_SUPPLY;
  while st >= S_PULL do begin
    lo := n.count[st * 2];
    hi := n.count[st * 2 + 1];
    if skip = st * 2 then Dec(lo);
    if skip = st * 2 + 1 then Dec(hi);
    if (lo <> 0) or (hi <> 0) then begin
      if lo <> 0 then exit(st * 2) else exit(st * 2 + 1);
    end;
    Dec(st);
  end;
  Result := DRV_Z;
end;

{ ------------------------------------------------------------------------ }
{ Group reads                                                               }
{ ------------------------------------------------------------------------ }

procedure TSim.GroupSetup(p: TPart; g: TGroup; const numbers: array of Integer; n: Integer);
var
  i, pi, bit, first: Integer;
  pin: TPin;
begin
  g.epoch := epoch;
  g.first := -1;
  g.mask := 0;
  g.ttl := 0;
  first := -1;
  for i := 0 to n - 1 do begin
    pi := p.pinmap[numbers[i]];
    if pi < 0 then exit;
    pin := pins[pi];
    if (pin.fault = PF_OPEN) or (pin.root < 0) then exit;
    bit := nets[pin.net].busbit;
    if (bit < 0) or ((i > 0) and (bit <> first + i)) then exit;
    if i = 0 then first := bit;
    if (pin.flags and PIN_TTL) <> 0 then g.ttl := g.ttl or (1 shl i);
  end;
  { The group must sit in one 32-bit word of the bitmap. }
  if (first div 32) <> ((first + n - 1) div 32) then exit;
  g.first := first;
  g.word := first div 32;
  g.shift := first mod 32;
  if n >= 32 then g.mask := -1 else g.mask := (1 shl n) - 1;
end;

function TSim.ReadGroup(p: TPart; key: Integer; const numbers: array of Integer; n: Integer): Integer;
var
  g, c: TGroup;
  i, level, floating, err, v: Integer;
begin
  g := nil;
  for i := 0 to PART_GROUPS - 1 do begin
    c := p.groups[i];
    if (c.key = key) and (c.n = n) then begin g := c; break; end;
    if c.key = 0 then begin
      g := c; c.key := key; c.n := n; c.epoch := epoch - 1;
      break;
    end;
  end;
  if (g <> nil) and (g.epoch <> epoch) then GroupSetup(p, g, numbers, n);
  if (g <> nil) and (g.first >= 0) then begin
    if g.word = 0 then begin
      level := busLevelLo shr g.shift;
      floating := busFloatLo shr g.shift;
      err := busErrLo shr g.shift;
    end else begin
      level := busLevelHi shr g.shift;
      floating := busFloatHi shr g.shift;
      err := busErrHi shr g.shift;
    end;
    if (err and g.mask and not g.ttl) = 0 then
      exit((level and g.mask) or (floating and g.ttl));
  end;
  v := 0;
  for i := 0 to n - 1 do v := v or (Read(p.pinmap[numbers[i]]) shl i);
  Result := v;
end;

{ ------------------------------------------------------------------------ }
{ Evaluation                                                                }
{ ------------------------------------------------------------------------ }

procedure TSim.MarkDirty(part: Integer);
begin
  if dirty[part] <> 0 then exit;
  dirty[part] := 1;
  queue[qtail and (SIM_QUEUE_SIZE - 1)] := part;
  qtail := (qtail + 1) and $3FFFFFFF;
end;

function TSim.PhantomPowered(p: TPart): Boolean;
var
  k, want: Integer;
begin
  if p.power = PWR_NO_VCC then want := 1 else want := 0;
  for k := 0 to p.npins - 1 do begin
    if (pins[p.first_pin + k].flags and PIN_POWER) <> 0 then continue;
    if Read(p.first_pin + k) = want then exit(True);
  end;
  for k := 0 to p.npins - 1 do
    if (pins[p.first_pin + k].flags and PIN_POWER) = 0 then Drive(p.first_pin + k, DRV_Z);
  Result := False;
end;

procedure TSim.Settle;
var
  budget, part: Integer;
  p: TPart;
begin
  { Evaluate dirty parts until the board is stable. A board with a fault can
    oscillate, so give up after a large number of evaluations. }
  budget := 20000;
  while (qhead <> qtail) and (budget > 0) do begin
    Dec(budget);
    part := queue[qhead and (SIM_QUEUE_SIZE - 1)];
    qhead := (qhead + 1) and $3FFFFFFF;
    dirty[part] := 0;
    p := parts[part];
    if p.dead then continue;
    if (p.power <> PWR_OK) and not PhantomPowered(p) then continue;
    p.eval(Self, p);
  end;
  while qhead <> qtail do begin     { oscillating: drop pending work }
    dirty[queue[qhead and (SIM_QUEUE_SIZE - 1)]] := 0;
    qhead := (qhead + 1) and $3FFFFFFF;
  end;
end;

procedure TSim.Commit;
var
  i: Integer;
  p: TPart;
begin
  if edge_commits and (nnoisy = 0) then begin
    for i := 0 to ncommit - 1 do begin
      p := parts[commit_list[i]];
      if p.dead then continue;
      if p.edge_commit then begin
        if cdirty[p.index] = 0 then continue;
        cdirty[p.index] := 0;
      end;
      p.commit(Self, p);
    end;
    exit;
  end;
  for i := 0 to ncommit - 1 do begin
    p := parts[commit_list[i]];
    if not p.dead then p.commit(Self, p);
  end;
end;

procedure TSim.UpdatePartPower(part: Integer);
var
  p: TPart;
  no_vcc, no_gnd: Boolean;
  i, pi: Integer;
  name: String;
begin
  p := parts[part];
  no_vcc := False;
  no_gnd := False;
  for i := 0 to p.npins - 1 do begin
    pi := p.first_pin + i;
    if ((pins[pi].flags and PIN_POWER) = 0) or (pins[pi].fault <> PF_OPEN) then continue;
    name := pin_names[pi];
    if (name <> '') and ((Pos('GND', name) > 0) or (name = 'VSS')) then no_gnd := True
    else no_vcc := True;
  end;
  if p.removed or p.no_supply or (no_vcc and no_gnd) then p.power := PWR_OFF
  else if no_vcc then p.power := PWR_NO_VCC
  else if no_gnd then p.power := PWR_NO_GND
  else p.power := PWR_OK;
  p.dead := (p.power = PWR_OFF) or ((p.power <> PWR_OK) and (p.tech <> TECH_CMOS));
end;

end.
