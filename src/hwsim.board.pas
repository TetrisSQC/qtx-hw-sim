unit hwsim.board;

{ A board: the netlist of a machine and its parts, and what every machine
  has in common: faults, logic probe, oscilloscope, labels, the RESET
  button (port of board.h / board.c). What makes a machine (the cores of its
  big chips, its bus cycle, keyboard, files, video) lives in a subclass:
  TC64 (hwsim.c64) and TZX48 (hwsim.zx48). }

interface

uses
  qtx.sysutils,
  hwsim.util, hwsim.boarddata, hwsim.sim, hwsim.parts;

const
  SCOPE_SAMPLES = 65536;

  { What a pin's net looked like over the last frame (logic probe colors). }
  NET_LOW = 0; NET_HIGH = 1; NET_PULSING = 2; NET_FLOATING = 3; NET_CONTENTION = 4; NET_NONE = 5;
  { Scope sample bits. }
  SC_LEVEL = 1; SC_FLOATING = 2; SC_FIGHT = 4;

  { The role of a chip, to color its name. }
  ROLE_RAM = 0; ROLE_VRAM = 1; ROLE_ROM = 2; ROLE_PLA = 3; ROLE_CHIP = 4; ROLE_GLUE = 5;
  ROLE_CLOCK = 6; ROLE_OTHER = 7;

type
  { A chip of the board: model, short name drawn on it, detail lines shown
    when zooming in (#10 separated), role, one line description. }
  TPartDef = class
  public
    ref: String;
    typ: TPartType;
    &label: String;
    role: Integer;
    info: String;
    detail: String;
    constructor Create(const aref: String; atyp: TPartType; const alabel: String; arole: Integer;
      const ainfo, adetail: String);
  end;

  { A connector: its name and what each pin is for, "PAD=FUNCTION ...". }
  TConnDef = class
  public
    ref, name, pins: String;
    constructor Create(const aref, aname, apins: String);
  end;

  TFileKind = class
  public
    typ: String;        { "program", "cartridge", "snapshot" }
    action: String;     { "Load and RUN" }
    norun: Boolean;
    role: Integer;
    constructor Create(const atyp, aaction: String; anorun: Boolean; arole: Integer);
  end;

  TBoard = class
  protected
    procedure SetupPassive(idx: Integer);
    procedure SetupConnectors;
    { Machine description, set by the subclass constructor before Init. }
    procedure Describe; virtual; abstract;
    { Set up the cores, models and wiring. }
    procedure MachineCreate; virtual; abstract;
    { Switched on: the cores start over. }
    procedure MachinePowerOn; virtual; abstract;
    { Joins of nets for Rebuild: pressed keys, dongles. }
    procedure MachineJoins; virtual;
  public
    sim: TSim;
    { Machine description. }
    id: String;
    name: String;
    board_name: String;
    data: TBoardData;
    simdesc: TSimDesc;
    cpu_hz: Float;
    cycles_per_frame: Integer;
    steps_per_cycle: Integer;
    screen_w, screen_h: Integer;
    ready_cycles: Float;
    part_defs: array of TPartDef;
    conn_defs: array of TConnDef;
    link_resistors: String;
    crystals: String;
    keys_help: String;
    reset_help: String;
    files_title: String;
    cart_slot: String;

    { Video, as the machine's video chip drew it (screen_w x screen_h, ABGR). }
    screen: TUInts;
    { Oscilloscope: one sample per bus step for the selected pin. }
    scope_pin: Integer;
    scope: TBytes;
    scope_pos: Integer;
    { Self-check oracle for tests. }
    check: Boolean;
    check_errors: Integer;
    { Logic probe statistics per net, reset by the UI every frame. }
    st_hi, st_lo, st_float, st_fight: TUInts;
    st_countdown: Integer;
    pin_stats: Boolean;
    pst_hi, pst_lo, pst_off, pst_weak: TUInts;
    p_pla: Integer;
    pla_hits: TUInts;
    pla_samples: Integer;
    { The RESET button. }
    reset_button: Boolean;
    force_pins: Boolean;        { no fast mode (machines that have one) }
    { Sound: samples (-1..1) at audio_rate, taken by the UI (audio_n so far); rate 0: no sound }
    audio_rate: Integer;
    audio: TFloats;
    audio_n: Integer;
    audio_acc: Float;
    pin_reset, pin_reset_gnd: Integer;
    { Connectors: what each pin is for ("ATN"), '' for other parts. }
    pin_func: array of String;

    constructor Create; virtual;
    { Build everything: call once after Create. }
    procedure Init;
    procedure PowerOn;
    procedure Rebuild;
    procedure Run(cycles: Integer); virtual; abstract;
    procedure Sample;

    { Faults. }
    procedure SetPinFault(pin, fault: Integer);
    procedure SetRemoved(part: Integer; removed: Boolean);
    procedure SetRFault(part, rfault: Integer);
    procedure ClearFaults;
    function ApplyFaultSpec(const spec: String): Boolean;
    procedure SetResetButton(pressed: Boolean);
    { one CPU cycle of a 0/1 sound line: a sample when one is due }
    procedure AudioCycle(level: Integer);
    function PartSimulated(part: Integer): Boolean;
    function PartDef(part: Integer): TPartDef;
    function PartLabel(part: Integer): String;
    function PartInfo(part: Integer): String;
    function PartDetail(part: Integer): String;
    function PartRole(part: Integer): Integer;
    function PadPin(part: Integer; const pad: String): Integer;
    function ConnectorName(part: Integer): String;
    function PinFunc(pin: Integer): String;
    function NetSimulated(net: Integer): Boolean;

    { Keyboard, files, video. }
    procedure KeyDown(key: Integer); virtual; abstract;
    procedure KeyUp(key: Integer); virtual; abstract;
    procedure FrameUpdate(us: Integer); virtual; abstract;
    function HostKey(const code: String): Integer; virtual; abstract;
    function HostChar(c: Integer): Integer; virtual; abstract;
    procedure Monitor(outp: TUInts); virtual; abstract;
    function FileKind(const path: String; const head: TBytes; var info: String; var bad: Boolean): TFileKind; virtual; abstract;
    function Load(const path: String; const bytes: TBytes; run: Boolean; var msg: String): Boolean; virtual; abstract;
    function CartName: String; virtual;
    procedure RemoveCart; virtual;
    function CpuStatus: String; virtual; abstract;
    function CpuRegs: String; virtual; abstract;
    { Read the screen text (--text). }
    function ScreenText: array of String; virtual; abstract;

    { Logic probe. }
    function NetState(net: Integer): Integer;
    procedure StatsReset;
    function PinDriveState(pin: Integer): Integer;
  end;

  TBoardClass = class of TBoard;

{ Does a pad name match? The C64 cassette port pads are named after both
  sides of the edge connector ("D,4"): either name matches. }
function PadMatches(const name, pad: String): Boolean;

implementation

constructor TPartDef.Create(const aref: String; atyp: TPartType; const alabel: String; arole: Integer;
  const ainfo, adetail: String);
begin
  inherited Create;
  ref := aref; typ := atyp; &label := alabel; role := arole; info := ainfo; detail := adetail;
end;

constructor TConnDef.Create(const aref, aname, apins: String);
begin
  inherited Create;
  ref := aref; name := aname; pins := apins;
end;

constructor TFileKind.Create(const atyp, aaction: String; anorun: Boolean; arole: Integer);
begin
  inherited Create;
  typ := atyp; action := aaction; norun := anorun; role := arole;
end;

{ ------------------------------------------------------------------------ }

constructor TBoard.Create;
begin
  inherited Create;
  simdesc := TSimDesc.Create;
  scope_pin := -1;
  p_pla := -1;
  pin_reset := -1;
  pin_reset_gnd := -1;
end;

procedure TBoard.MachineJoins;
begin
end;

function TBoard.CartName: String;
begin
  Result := '';
end;

procedure TBoard.RemoveCart;
begin
end;

procedure TBoard.Init;
var
  i, idx: Integer;
begin
  Describe;
  sim := TSim.Create(data, simdesc);
  for i := 0 to sim.nparts - 1 do sim.parts[i].user := Self;
  for i := 0 to part_defs.High do begin
    idx := sim.FindPart(part_defs[i].ref);
    if idx >= 0 then part_set_type(sim, idx, part_defs[i].typ);
  end;
  for i := 0 to sim.nparts - 1 do
    if (sim.parts[i].kind = PART_RESISTOR) or (sim.parts[i].kind = PART_LINK) then SetupPassive(i);
  screen := NewUInts(screen_w * screen_h);
  scope := NewBytes(SCOPE_SAMPLES);
  st_hi := NewUInts(sim.nnets);
  st_lo := NewUInts(sim.nnets);
  st_float := NewUInts(sim.nnets);
  st_fight := NewUInts(sim.nnets);
  pst_hi := NewUInts(sim.npins);
  pst_lo := NewUInts(sim.npins);
  pst_off := NewUInts(sim.npins);
  pst_weak := NewUInts(sim.npins);
  pla_hits := NewUInts(64);
  SetupConnectors;
  MachineCreate;
  PowerOn;
end;

{ ------------------------------------------------------------------------ }
{ Scope and statistics                                                      }
{ ------------------------------------------------------------------------ }

procedure TBoard.Sample;
var
  s: TSim;
  pin: TPin;
  n: TNet;
  v, i, d, inp, t: Integer;
  pla: TPart;
begin
  s := sim;
  if scope_pin >= 0 then begin
    pin := s.pins[scope_pin];
    v := 0;
    if pin.root >= 0 then begin
      n := s.nets[pin.root];
      if n.level <> 0 then v := SC_LEVEL;
      if n.strength = S_Z then v := v or SC_FLOATING;
      if n.contention <> 0 then v := v or SC_FIGHT;
    end;
    scope[scope_pos and (SCOPE_SAMPLES - 1)] := v;
    scope_pos := (scope_pos + 1) and $3FFFFFFF;
  end;
  { Logic probe statistics: sample every net once in a while. }
  Dec(st_countdown);
  if st_countdown > 0 then exit;
  st_countdown := 97;
  { Inputs reading an erratic level can change without any event. }
  s.WakeErratic;
  for i := 0 to s.nnets - 1 do begin
    n := s.nets[s.nets[i].root];
    if n.contention <> 0 then st_fight[i] := st_fight[i] + 1
    else if n.strength = S_Z then st_float[i] := st_float[i] + 1
    else if n.level <> 0 then st_hi[i] := st_hi[i] + 1
    else st_lo[i] := st_lo[i] + 1;
  end;
  if not pin_stats then exit;
  for i := 0 to s.npins - 1 do begin
    d := s.pins[i].drive;
    if (d shr 1) = S_Z then pst_off[i] := pst_off[i] + 1
    else if (d and 1) <> 0 then pst_hi[i] := pst_hi[i] + 1
    else pst_lo[i] := pst_lo[i] + 1;
    if (d shr 1) = S_PULL then pst_weak[i] := pst_weak[i] + 1;
  end;
  if p_pla < 0 then exit;
  pla := s.parts[p_pla];
  if not pla.dead then begin
    inp := pla_read_inputs(s, pla);
    for t := 0 to pla_nterms - 1 do
      if (inp and pla_terms[t].mask) = pla_terms[t].value then pla_hits[t] := pla_hits[t] + 1;
    Inc(pla_samples);
  end;
end;

{ ------------------------------------------------------------------------ }
{ Labels                                                                    }
{ ------------------------------------------------------------------------ }

function TBoard.PartDef(part: Integer): TPartDef;
var
  ref: String;
  i: Integer;
begin
  ref := sim.parts[part].ref;
  for i := 0 to part_defs.High do
    if part_defs[i].ref = ref then exit(part_defs[i]);
  Result := nil;
end;

function TBoard.PartLabel(part: Integer): String;
var d: TPartDef;
begin
  d := PartDef(part);
  if d <> nil then Result := d.&label else Result := sim.parts[part].name;
end;

function TBoard.PartDetail(part: Integer): String;
var d: TPartDef;
begin
  d := PartDef(part);
  if d <> nil then Result := d.detail else Result := sim.parts[part].name;
end;

function TBoard.PartInfo(part: Integer): String;
var d: TPartDef;
begin
  d := PartDef(part);
  if d <> nil then Result := d.info else Result := '';
end;

function TBoard.PartRole(part: Integer): Integer;
var d: TPartDef;
begin
  d := PartDef(part);
  if d <> nil then Result := d.role else Result := ROLE_OTHER;
end;

{ ------------------------------------------------------------------------ }
{ Passive parts                                                             }
{ ------------------------------------------------------------------------ }

procedure TBoard.SetupPassive(idx: Integer);
var
  s: TSim;
  p: TPart;
  st: TResistorState;
  pa, pb: array of Integer;
  bussed, isolated, series: Boolean;
  n, i, a, c, ra, rc, t: Integer;
  e: TElement;
begin
  s := sim;
  p := s.parts[idx];
  st := TResistorState.Create;
  p.state := st;
  bussed := StrContains(p.name, 'Bussed');
  isolated := StrContains(p.name, 'isolated');
  if bussed then begin
    n := 2;
    while (n < 48) and (pa.Length < 16) do begin
      if p.pinmap[n] >= 0 then begin pa.Add(p.pinmap[n]); pb.Add(p.pinmap[1]); end;
      Inc(n);
    end;
  end else if isolated then begin
    n := 1;
    while (n + 1 < 48) and (pa.Length < 16) do begin
      if (p.pinmap[n] >= 0) and (p.pinmap[n + 1] >= 0) then begin
        pa.Add(p.pinmap[n]); pb.Add(p.pinmap[n + 1]);
      end;
      n := n + 2;
    end;
  end else if (p.pinmap[1] >= 0) and (p.pinmap[2] >= 0) then begin
    pa.Add(p.pinmap[1]); pb.Add(p.pinmap[2]);
  end;
  { A "dashed" jumper is an option not fitted on this board. }
  if StrContains(p.name, 'DASHED') then begin pa.Clear; pb.Clear; end;
  series := InList(link_resistors, p.ref);
  for i := 0 to pa.High do begin
    e := TElement.Create;
    st.el.Add(e);
    Inc(st.nel);
    a := pa[i]; c := pb[i];
    if s.pins[a].net >= 0 then ra := s.nets[s.pins[a].net].rail else ra := RAIL_NONE;
    if s.pins[c].net >= 0 then rc := s.nets[s.pins[c].net].rail else rc := RAIL_NONE;
    if (ra <> 0) and (rc = 0) then begin
      t := a; a := c; c := t;
      t := ra; ra := rc; rc := t;
    end;
    e.pin_a := a; e.pin_b := c;
    if p.kind = PART_LINK then e.typ := EL_LINK
    else if (rc = RAIL_VCC) and (ra = 0) then e.typ := EL_PULLUP
    else if (rc = RAIL_GND) and (ra = 0) then e.typ := EL_PULLDOWN
    else if series then e.typ := EL_LINK
    else e.typ := EL_ANALOG;
    if (e.typ = EL_PULLUP) or (e.typ = EL_PULLDOWN) then p.eval := resistor_eval;
  end;
end;

function TBoard.PartSimulated(part: Integer): Boolean;
var
  p: TPart;
  st: TResistorState;
  i: Integer;
begin
  p := sim.parts[part];
  if (p.kind = PART_IC) or (p.kind = PART_CONNECTOR) then exit(True);
  if InList(crystals, p.ref) then exit(True);
  if (p.kind = PART_RESISTOR) or (p.kind = PART_LINK) then begin
    st := TResistorState(p.state);
    if st <> nil then
      for i := 0 to st.nel - 1 do
        if st.el[i].typ <> EL_ANALOG then exit(True);
  end;
  Result := False;
end;

{ Rebuild the electrical network after a change: links, keys, faults. }
procedure TBoard.Rebuild;
var
  s: TSim;
  i, k: Integer;
  p: TPart;
  st: TResistorState;
  e: TElement;
begin
  s := sim;
  s.JoinBegin;
  for i := 0 to s.nparts - 1 do begin
    p := s.parts[i];
    if ((p.kind <> PART_RESISTOR) and (p.kind <> PART_LINK)) or (p.state = nil) then continue;
    if p.rfault = RF_OPEN then continue;
    st := TResistorState(p.state);
    for k := 0 to st.nel - 1 do begin
      e := st.el[k];
      if (e.typ <> EL_LINK) and not ((e.typ = EL_SERIES) and (p.rfault = RF_SHORT)) then continue;
      if (s.pins[e.pin_a].fault = PF_OPEN) or (s.pins[e.pin_b].fault = PF_OPEN) then continue;
      s.Join(s.pins[e.pin_a].net, s.pins[e.pin_b].net);
    end;
  end;
  MachineJoins;
  if reset_button and (pin_reset >= 0) and (pin_reset_gnd >= 0) and
     (s.pins[pin_reset].fault <> PF_OPEN) and (s.pins[pin_reset_gnd].fault <> PF_OPEN) then
    s.Join(s.pins[pin_reset].net, s.pins[pin_reset_gnd].net);
  for i := 0 to s.nparts - 1 do s.UpdatePartPower(i);
  s.Rebuild;
end;

{ ------------------------------------------------------------------------ }
{ Connectors                                                                }
{ ------------------------------------------------------------------------ }

function PadMatches(const name, pad: String): Boolean;
var
  parts: array of String;
  i: Integer;
begin
  parts := name.Split(',');
  for i := 0 to parts.High do
    if parts[i] = pad then exit(True);
  Result := False;
end;

function TBoard.PadPin(part: Integer; const pad: String): Integer;
var
  p: TPart;
  k: Integer;
begin
  if part < 0 then exit(-1);
  p := sim.parts[part];
  for k := 0 to p.npins - 1 do
    if PadMatches(sim.bd.pads[p.first_pin + k].name, pad) then exit(p.first_pin + k);
  Result := -1;
end;

procedure TBoard.SetupConnectors;
var
  c, part, k, eq: Integer;
  toks: array of String;
  tok, pad, func: String;
  p: TPart;
  t: Integer;
begin
  pin_func.SetLength(sim.npins);
  for k := 0 to sim.npins - 1 do pin_func[k] := '';
  for c := 0 to conn_defs.High do begin
    part := sim.FindPart(conn_defs[c].ref);
    if part < 0 then continue;
    toks := conn_defs[c].pins.Split(' ');
    for t := 0 to toks.High do begin
      tok := toks[t];
      eq := Pos('=', tok);
      if eq <= 0 then continue;
      pad := Copy(tok, 1, eq - 1);
      func := StrReplace(Copy(tok, eq + 1, Length(tok)), '_', ' ');
      p := sim.parts[part];
      for k := 0 to p.npins - 1 do
        if PadMatches(sim.bd.pads[p.first_pin + k].name, pad) then pin_func[p.first_pin + k] := func;
    end;
  end;
end;

function TBoard.ConnectorName(part: Integer): String;
var c: Integer;
begin
  for c := 0 to conn_defs.High do
    if conn_defs[c].ref = sim.parts[part].ref then exit(conn_defs[c].name);
  Result := '';
end;

function TBoard.PinFunc(pin: Integer): String;
begin
  if pin >= 0 then Result := pin_func[pin] else Result := '';
end;

function TBoard.NetSimulated(net: Integer): Boolean;
var
  n: TNet;
  k, part: Integer;
begin
  if net < 0 then exit(False);
  n := sim.nets[sim.nets[net].root];
  if n.rail <> 0 then exit(True);
  for k := 0 to n.pins.High do begin
    part := sim.pins[n.pins[k]].part;
    if (sim.parts[part].kind = PART_IC) and PartSimulated(part) then exit(True);
  end;
  Result := False;
end;

{ ------------------------------------------------------------------------ }
{ Life                                                                      }
{ ------------------------------------------------------------------------ }

{ Cold start: like switching the machine on. Faults stay where they are. }
procedure TBoard.PowerOn;
var i: Integer;
begin
  sim.cycle := 0;
  MachinePowerOn;
  sim.Discharge;
  sim.Seed(1);
  for i := 0 to sim.nparts - 1 do
    if Assigned(sim.parts[i].reset) then sim.parts[i].reset(sim, sim.parts[i]);
  Rebuild;
end;

{ ------------------------------------------------------------------------ }
{ Faults                                                                    }
{ ------------------------------------------------------------------------ }

procedure TBoard.SetPinFault(pin, fault: Integer);
begin
  sim.SetPinFault(pin, fault);
  Rebuild;
end;

procedure TBoard.SetRemoved(part: Integer; removed: Boolean);
begin
  sim.parts[part].removed := removed;
  Rebuild;
end;

procedure TBoard.SetRFault(part, rfault: Integer);
begin
  sim.parts[part].rfault := rfault;
  Rebuild;
end;

procedure TBoard.AudioCycle(level: Integer);
begin
  audio_acc := audio_acc + audio_rate;
  if audio_acc < cpu_hz then exit;
  audio_acc := audio_acc - cpu_hz;
  if audio = nil then audio := NewFloats(16384);
  if audio_n >= 16384 then exit;              { nobody takes them (headless, muted) }
  audio[audio_n] := level - 0.5;
  Inc(audio_n);
end;

procedure TBoard.SetResetButton(pressed: Boolean);
begin
  if reset_button = pressed then exit;
  reset_button := pressed;
  Rebuild;
end;

{ Text form of a fault, used by the command line:
    U17:removed  U17.18:gnd  U17.18:vcc  U17.18:open  R51:open  R51:short
  and the RESET button: RESET:pressed  RESET:released }
function TBoard.ApplyFaultSpec(const spec: String): Boolean;
var
  colon, dot, part, pi, f: Integer;
  head, what, ref, pad: String;
begin
  if (spec = 'RESET:pressed') or (spec = 'RESET:released') then begin
    SetResetButton(spec = 'RESET:pressed');
    exit(True);
  end;
  colon := Pos(':', spec);
  if colon <= 1 then exit(False);
  head := Copy(spec, 1, colon - 1);
  what := Copy(spec, colon + 1, Length(spec));
  if what = '' then exit(False);
  dot := Pos('.', head);
  if dot > 0 then begin
    ref := Copy(head, 1, dot - 1);
    pad := Copy(head, dot + 1, Length(head));
  end else begin
    ref := head;
    pad := '';
  end;
  part := sim.FindPart(ref);
  if part < 0 then exit(False);
  if pad <> '' then begin
    pi := PadPin(part, pad);
    if pi < 0 then exit(False);
    if what = 'gnd' then f := PF_GND
    else if what = 'vcc' then f := PF_VCC
    else if what = 'open' then f := PF_OPEN
    else f := PF_OK;
    SetPinFault(pi, f);
  end else if (what = 'removed') or (what = 'present') then
    SetRemoved(part, what = 'removed')
  else if what = 'open' then SetRFault(part, RF_OPEN)
  else if what = 'short' then SetRFault(part, RF_SHORT)
  else SetRFault(part, RF_OK);
  Result := True;
end;

procedure TBoard.ClearFaults;
var i: Integer;
begin
  for i := 0 to sim.npins - 1 do sim.SetPinFault(i, PF_OK);
  for i := 0 to sim.nparts - 1 do begin
    sim.parts[i].removed := False;
    sim.parts[i].rfault := RF_OK;
  end;
  Rebuild;
end;

{ ------------------------------------------------------------------------ }
{ Logic probe                                                               }
{ ------------------------------------------------------------------------ }

function TBoard.NetState(net: Integer): Integer;
var
  hi, lo, fl, fi, tot: Integer;
  n: TNet;
begin
  if net < 0 then exit(NET_NONE);
  hi := st_hi[net]; lo := st_lo[net]; fl := st_float[net]; fi := st_fight[net];
  tot := hi + lo + fl + fi;
  if tot = 0 then begin
    n := sim.nets[sim.nets[net].root];
    if n.contention <> 0 then exit(NET_CONTENTION);
    if n.strength = S_Z then exit(NET_FLOATING);
    if n.level <> 0 then exit(NET_HIGH) else exit(NET_LOW);
  end;
  if fi * 20 > tot then exit(NET_CONTENTION);
  if fl = tot then exit(NET_FLOATING);
  if (hi <> 0) and (lo <> 0) then exit(NET_PULSING);
  if hi <> 0 then begin
    if fl <> 0 then exit(NET_PULSING) else exit(NET_HIGH);
  end;
  if lo <> 0 then begin
    if fl <> 0 then exit(NET_PULSING) else exit(NET_LOW);
  end;
  Result := NET_FLOATING;
end;

procedure TBoard.StatsReset;
begin
  st_hi.fill(0); st_lo.fill(0); st_float.fill(0); st_fight.fill(0);
  pst_hi.fill(0); pst_lo.fill(0); pst_off.fill(0); pst_weak.fill(0);
  pla_hits.fill(0);
  pla_samples := 0;
end;

function TBoard.PinDriveState(pin: Integer): Integer;
var hi, lo, off, d: Integer;
begin
  hi := pst_hi[pin]; lo := pst_lo[pin]; off := pst_off[pin];
  if hi + lo + off = 0 then begin
    d := sim.pins[pin].drive;
    if (d shr 1) = S_Z then exit(NET_FLOATING);
    if (d and 1) <> 0 then exit(NET_HIGH) else exit(NET_LOW);
  end;
  if ((hi <> 0) and (lo <> 0)) or (((hi <> 0) or (lo <> 0)) and (off <> 0)) then exit(NET_PULSING);
  if hi <> 0 then exit(NET_HIGH);
  if lo <> 0 then exit(NET_LOW);
  Result := NET_FLOATING;
end;

end.
