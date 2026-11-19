unit hwsim.webui;

{ The browser interface (the counterpart of ui.c).

  Left: the motherboard, drawn from the KiCad geometry. Chips are dark
  boxes with their name, pins are small squares colored like a logic probe
  would show them. Mouse wheel zooms, dragging pans. Clicking a pin opens a
  menu to break it (lift the leg, short to GND/+5V) or to put it on the
  oscilloscope; clicking a chip pulls it out of its socket; clicking a
  resistor, a link or the crystal opens or shorts it.

  Right: what the monitor shows (click it to type on the machine's
  keyboard), the oscilloscope, the faults and the controls.

  The DOM is used through Variants (late bound calls compile to plain JS).
  The simulation runs in slices inside requestAnimationFrame. }

interface

uses
  qtx.sysutils,
  hwsim.util, hwsim.boarddata, hwsim.sim, hwsim.parts, hwsim.board, hwsim.roms, hwsim.c64, hwsim.zx48, hwsim.apple2, hwsim.lc80;

type
  TSimUI = class
  private
    FBoard: TBoard;
    doc, win: Variant;
    boardCv, boardCtx, monCv, monCtx, monImg, scopeCv, scopeCtx: Variant;
    statusEl, infoEl, faultsEl, menuEl, machineSel, fileSel, harnessChk, pinsChk, pauseBtn, speedEl,
      timebaseSel, resetBtn, fileInput, cpuEl, keypadEl: Variant;
    monPix: TUInts;
    monBuf: TUInts;
    { board view: screen = o + mm * zoom }
    zoom, ox, oy: Float;
    fitted: Boolean;
    bx0, by0, bx1, by1: TFloats;
    hoverPin, hoverPart: Integer;
    dragging, dragMoved: Boolean;
    dragX, dragY, mouseX, mouseY: Float;
    netState: TBytes;
    paused: Boolean;
    cycleInFrame: Integer;
    emuFrames: Integer;
    speedAcc, speedReal: Float;
    lastSliceMs, speedT0: Float;
    audioCtx: Variant;
    audioTime: Float;
    soundChk: Variant;
    speedPct: Float;
    timebase: Integer;
    pendingLoad: String;
    pendingBytes: TBytes;
    lastFaultSig: String;
    procedure Build;
    function El(const tag, cls: String; parent: Variant): Variant;
    function Button(const text: String; parent: Variant; handler: procedure): Variant;
    procedure Attach(b: TBoard);
    procedure BuildKeypad;
    procedure PlayAudio;
    procedure StartAudio;
    procedure SwitchMachine(const id: String);
    procedure ComputeBoxes;
    procedure FitView;
    function PickPin(mx, my: Float): Integer;
    function PickPart(mx, my: Float): Integer;
    function PinInteractive(pin: Integer): Boolean;
    procedure Frame(ts: Float);
    procedure RunSlice;
    procedure DrawBoard;
    procedure DrawPart(i: Integer);
    procedure DrawPins;
    procedure DrawMonitor;
    procedure DrawScope;
    procedure UpdateFaults;
    procedure UpdateInfo;
    procedure OpenMenu(pin, part: Integer; x, y: Float);
    procedure CloseMenu;
    procedure MenuItem(const text: String; handler: procedure);
    procedure SelectScopePin(pin: Integer);
    procedure LoadBytes(const name: String; const bytes: TBytes);
    procedure LoadBuiltin(const name: String);
    procedure OnMouseDown(e: Variant);
    procedure OnMouseMove(e: Variant);
    procedure OnMouseUp(e: Variant);
    procedure OnWheel(e: Variant);
    procedure OnKey(e: Variant; down: Boolean);
    procedure OnResize;
    procedure Status(const msg: String);
  public
    constructor Create;
  end;

implementation

const
  COL_PCB = '#1c5834';
  COL_PCB_EDGE = '#0a2d19';
  COL_CHIP = '#1a1a1c';
  COL_CHIP_EDGE = '#64646c';
  COL_SOCKET = '#c8a55a';
  COL_RES = '#cdb68c';
  COL_LINK = '#46464c';
  COL_OTHER = '#3c7850';
  COL_TEXT = '#ebebe1';
  COL_DIMTEXT = '#96a096';
  COL_HIGH = '#f0503c';
  COL_LOW = '#468cff';
  COL_PULSE = '#fad23c';
  COL_FLOAT = '#a0a0a0';
  COL_FIGHT = '#ff3cff';
  COL_NONE = '#464646';
  COL_FAULT = '#ff9600';
  COL_HOVER = '#00ffff';
  COL_TRACE = '#5aff78';
  COL_HARNESS = '#96a0ff';

  ROLE_COLORS: array[0..7] of String = ['#78c8ff', '#e18cff', '#ffc864', '#ff7878', '#ebebe1', '#a0dca0',
    '#dcdc78', '#b4b4b4'];
  TIMEBASES: array[0..4] of Integer = [16, 64, 256, 1024, 4096];

function NowMs: Float;
begin
  asm @Result = performance.now(); end;
end;

function StateColor(st: Integer): String;
begin
  case st of
    NET_LOW: Result := COL_LOW;
    NET_HIGH: Result := COL_HIGH;
    NET_PULSING: Result := COL_PULSE;
    NET_FLOATING: Result := COL_FLOAT;
    NET_CONTENTION: Result := COL_FIGHT;
  else
    Result := COL_NONE;
  end;
end;

function StateName(st: Integer): String;
begin
  case st of
    NET_LOW: Result := 'LOW';
    NET_HIGH: Result := 'HIGH';
    NET_PULSING: Result := 'PULSING';
    NET_FLOATING: Result := 'FLOATING';
    NET_CONTENTION: Result := 'CONTENTION';
  else
    Result := '-';
  end;
end;

{ ------------------------------------------------------------------------ }
{ Setup                                                                     }
{ ------------------------------------------------------------------------ }

constructor TSimUI.Create;
begin
  inherited Create;
  asm @doc = document; @win = window; end;
  hoverPin := -1;
  hoverPart := -1;
  timebase := 2;
  Build;
  SwitchMachine('c64');
  win.requestAnimationFrame(procedure(ts: Float) begin Frame(ts); end);
end;

function TSimUI.El(const tag, cls: String; parent: Variant): Variant;
begin
  Result := doc.createElement(tag);
  if cls <> '' then Result.className := cls;
  if parent <> nil then parent.appendChild(Result);
end;

function TSimUI.Button(const text: String; parent: Variant; handler: procedure): Variant;
begin
  Result := El('button', '', parent);
  Result.textContent := text;
  Result.onclick := procedure(e: Variant) begin handler(); end;
end;

procedure TSimUI.Build;
var
  app, left, right, bar, row, legend, item, sw, opt, monWrap, names, opts: Variant;
  i: Integer;
  st: array[0..4] of Integer;
  lbls: array of String;
begin
  doc.body.innerHTML := '';
  app := El('div', 'app', doc.body);
  left := El('div', 'left', app);
  boardCv := El('canvas', 'board', left);
  boardCtx := boardCv.getContext('2d');
  infoEl := El('div', 'info', left);
  menuEl := El('div', 'menu', doc.body);
  menuEl.style.display := 'none';

  right := El('div', 'right', app);
  bar := El('div', 'bar', right);
  machineSel := El('select', '', bar);
  opt := El('option', '', machineSel); opt.value := 'c64'; opt.textContent := 'Commodore 64 (ASSY 250407)';
  opt := El('option', '', machineSel); opt.value := 'zx48'; opt.textContent := 'ZX Spectrum 48K (Issue 3B)';
  opt := El('option', '', machineSel); opt.value := 'apple2p'; opt.textContent := 'Apple II+';
  opt := El('option', '', machineSel); opt.value := 'lc80'; opt.textContent := 'LC-80';
  machineSel.onchange := procedure(e: Variant) begin SwitchMachine(machineSel.value); end;
  Button('Power cycle', bar, procedure begin FBoard.PowerOn; cycleInFrame := 0; Status('switched off and on'); end);
  pauseBtn := Button('Pause', bar, procedure
    begin
      paused := not paused;
      if paused then pauseBtn.textContent := 'Run' else pauseBtn.textContent := 'Pause';
    end);
  resetBtn := El('button', '', bar);
  resetBtn.textContent := 'RESET';
  resetBtn.onmousedown := procedure(e: Variant) begin FBoard.SetResetButton(True); end;
  resetBtn.onmouseup := procedure(e: Variant) begin FBoard.SetResetButton(False); end;
  resetBtn.onmouseleave := procedure(e: Variant) begin FBoard.SetResetButton(False); end;

  monWrap := El('div', 'monwrap', right);
  monCv := El('canvas', 'monitor', monWrap);
  monCv.tabIndex := 0;
  monCtx := monCv.getContext('2d');
  keypadEl := El('div', 'keypad', right);
  keypadEl.style.display := 'none';
  speedEl := El('div', 'speed', right);
  cpuEl := El('div', 'cpu', right);

  row := El('div', 'bar', right);
  El('span', 'lbl', row).textContent := 'Scope';
  timebaseSel := El('select', '', row);
  for i := 0 to 4 do begin
    opt := El('option', '', timebaseSel);
    opt.value := IntToStr(i);
    opt.textContent := IntToStr(TIMEBASES[i]) + ' cycles';
  end;
  timebaseSel.value := IntToStr(timebase);
  timebaseSel.onchange := procedure(e: Variant) begin timebase := StrToInt(timebaseSel.value); end;
  scopeCv := El('canvas', 'scope', right);
  scopeCv.width := 440;
  scopeCv.height := 120;
  scopeCtx := scopeCv.getContext('2d');

  row := El('div', 'bar', right);
  fileSel := El('select', '', row);
  opt := El('option', '', fileSel); opt.value := ''; opt.textContent := 'Built-in cartridges / ROMs...';
  names := BuiltinFileNames;
  for i := 0 to names.length - 1 do begin
    opt := El('option', '', fileSel);
    opt.value := names[i];
    opt.textContent := names[i];
  end;
  fileSel.onchange := procedure(e: Variant)
    begin
      if fileSel.value <> '' then LoadBuiltin(fileSel.value);
      fileSel.value := '';
    end;
  fileInput := El('input', '', row);
  fileInput.&type := 'file';
  fileInput.onchange := procedure(e: Variant)
    var f, rd: Variant;
    begin
      f := fileInput.files[0];
      if f = nil then exit;
      asm @rd = new FileReader(); end;
      rd.onload := procedure(ev: Variant)
        var bytes: TBytes;
        begin
          asm @bytes = new Uint8Array((@rd).result); end;
          LoadBytes(f.name, bytes);
          fileInput.value := '';
        end;
      rd.readAsArrayBuffer(f);
    end;
  row := El('div', 'bar', right);
  harnessChk := El('input', '', row);
  harnessChk.&type := 'checkbox';
  harnessChk.id := 'harness';
  item := El('label', '', row);
  item.htmlFor := 'harness';
  item.textContent := '586220 test harness (C64)';
  harnessChk.onchange := procedure(e: Variant)
    begin
      if FBoard is TC64 then TC64(FBoard).SetHarness(harnessChk.checked)
      else harnessChk.checked := False;
    end;
  Button('Repair all', row, procedure begin FBoard.ClearFaults; Status('all faults repaired'); end);
  row := El('div', 'bar', right);
  pinsChk := El('input', '', row);
  pinsChk.&type := 'checkbox';
  pinsChk.id := 'pinlevel';
  item := El('label', '', row);
  item.htmlFor := 'pinlevel';
  item.textContent := 'Always at pin level (slow; otherwise only while a fault is set)';
  pinsChk.onchange := procedure(e: Variant) begin FBoard.force_pins := pinsChk.checked; end;
  row := El('div', 'bar', right);
  soundChk := El('input', '', row);
  soundChk.&type := 'checkbox';
  soundChk.id := 'sound';
  soundChk.checked := True;
  item := El('label', '', row);
  item.htmlFor := 'sound';
  item.textContent := 'Sound (machines that have it: LC-80)';
  soundChk.onchange := procedure(e: Variant) begin StartAudio; end;
  { browsers start audio only after the user did something }
  doc.addEventListener('pointerdown', procedure(e: Variant) begin StartAudio; end);
  doc.addEventListener('keydown', procedure(e: Variant) begin StartAudio; end);

  statusEl := El('div', 'status', right);
  El('div', 'lbl', right).textContent := 'Faults';
  faultsEl := El('div', 'faults', right);

  legend := El('div', 'legend', right);
  st[0] := NET_HIGH; st[1] := NET_LOW; st[2] := NET_PULSING; st[3] := NET_FLOATING; st[4] := NET_CONTENTION;
  for i := 0 to 4 do begin
    item := El('span', 'leg', legend);
    sw := El('span', 'sw', item);
    sw.style.background := StateColor(st[i]);
    El('span', '', item).textContent := StateName(st[i]);
  end;
  El('div', 'help', right).innerHTML :=
    'Wheel: zoom &middot; drag: pan &middot; click a pin or a part to break it. ' +
    'Click the monitor to type on the machine.';

  boardCv.onmousedown := procedure(e: Variant) begin OnMouseDown(e); end;
  win.addEventListener('mousemove', procedure(e: Variant) begin OnMouseMove(e); end);
  win.addEventListener('mouseup', procedure(e: Variant) begin OnMouseUp(e); end);
  opts := nil;
  asm @opts = { passive: false }; end;
  boardCv.addEventListener('wheel', procedure(e: Variant) begin OnWheel(e); end, opts);
  monCv.addEventListener('keydown', procedure(e: Variant) begin OnKey(e, True); end);
  monCv.addEventListener('keyup', procedure(e: Variant) begin OnKey(e, False); end);
  win.addEventListener('resize', procedure(e: Variant) begin OnResize; end);
  OnResize;
end;

procedure TSimUI.Status(const msg: String);
begin
  statusEl.textContent := msg;
end;

procedure TSimUI.OnResize;
var r: Variant;
begin
  r := boardCv.parentNode.getBoundingClientRect();
  boardCv.width := Trunc(r.width);
  boardCv.height := Trunc(r.height) - 28;
  fitted := False;
end;

{ The LC-80's keyboard (board 15403.500-1902), as on the schematic's table:
  click or hold a key. Shown for the LC-80 only. }
procedure TSimUI.BuildKeypad;
const
  LABELS: array[0..24] of String = ['RES', 'ADR', 'DAT', '+', '-', 'NMI', 'C', 'D', 'E', 'F',
    'ST', '8', '9', 'A', 'B', 'LD', '4', '5', '6', '7', 'EX', '0', '1', '2', '3'];
var
  i, code: Integer;
  b: Variant;
  lab: String;
begin
  keypadEl.innerHTML := '';
  if not (FBoard is TLc80) then begin
    keypadEl.style.display := 'none';
    exit;
  end;
  keypadEl.style.display := 'grid';
  for i := 0 to 24 do begin
    lab := LABELS[i];
    if lab = 'RES' then code := LC80_KEY_RES
    else if lab = 'NMI' then code := LC80_KEY_NMI
    else if lab = 'ADR' then code := LC80_KEY_ADR
    else if lab = 'DAT' then code := LC80_KEY_DAT
    else if lab = 'ST' then code := LC80_KEY_ST
    else if lab = 'LD' then code := LC80_KEY_LD
    else if lab = 'EX' then code := LC80_KEY_EX
    else code := Ord(lab[1]);
    b := El('button', '', keypadEl);
    b.textContent := lab;
    if (lab = 'RES') or (lab = 'NMI') then b.className := 'red';
    b.setAttribute('data-code', IntToStr(code));
    b.onmousedown := procedure(e: Variant)
      var c: Integer;
      begin
        c := StrToInt(e.currentTarget.getAttribute('data-code'));
        FBoard.KeyDown(c);
        e.preventDefault();
      end;
    b.onmouseup := procedure(e: Variant)
      var c: Integer;
      begin
        c := StrToInt(e.currentTarget.getAttribute('data-code'));
        FBoard.KeyUp(c);
      end;
    b.onmouseleave := procedure(e: Variant)
      var c: Integer;
      begin
        c := StrToInt(e.currentTarget.getAttribute('data-code'));
        FBoard.KeyUp(c);
      end;
  end;
end;

procedure TSimUI.Attach(b: TBoard);
begin
  FBoard := b;
  bx0 := NewFloats(b.sim.nparts);
  by0 := NewFloats(b.sim.nparts);
  bx1 := NewFloats(b.sim.nparts);
  by1 := NewFloats(b.sim.nparts);
  netState := NewBytes(b.sim.nnets);
  ComputeBoxes;
  fitted := False;
  monCv.width := b.screen_w;
  monCv.height := b.screen_h;
  monImg := monCtx.createImageData(b.screen_w, b.screen_h);
  asm @monPix = new Uint32Array((@monImg).data.buffer); end;
  monBuf := NewUInts(b.screen_w * b.screen_h);
  cycleInFrame := 0;
  emuFrames := 0;
  hoverPin := -1;
  hoverPart := -1;
  b.StatsReset;
  CloseMenu;
  lastFaultSig := '?';
  if b is TC64 then harnessChk.checked := TC64(b).harness else harnessChk.checked := False;
  b.force_pins := pinsChk.checked;
  BuildKeypad;
end;

procedure TSimUI.SwitchMachine(const id: String);
var b: TBoard;
begin
  if id = 'zx48' then b := TZX48.Create
  else if id = 'apple2p' then b := TApple2.Create
  else if id = 'lc80' then b := TLc80.Create
  else b := TC64.Create;
  b.Init;
  machineSel.value := id;
  Attach(b);
  Status(b.name + ' switched on');
end;

{ ------------------------------------------------------------------------ }
{ Geometry                                                                  }
{ ------------------------------------------------------------------------ }

procedure TSimUI.ComputeBoxes;
var
  s: TSim;
  i, k: Integer;
  p: TPart;
  x0, y0, x1, y1, pad, m, inset: Float;
  bd: TBdPad;
begin
  s := FBoard.sim;
  for i := 0 to s.nparts - 1 do begin
    p := s.parts[i];
    x0 := 1e9; y0 := 1e9; x1 := -1e9; y1 := -1e9; pad := 0;
    for k := 0 to p.npins - 1 do begin
      bd := s.bd.pads[p.first_pin + k];
      if bd.x < x0 then x0 := bd.x;
      if bd.y < y0 then y0 := bd.y;
      if bd.x > x1 then x1 := bd.x;
      if bd.y > y1 then y1 := bd.y;
      if bd.w > pad then pad := bd.w;
      if bd.h > pad then pad := bd.h;
    end;
    if p.kind = PART_IC then begin
      if x1 - x0 >= y1 - y0 then begin
        bx0[i] := x0 - 1.4; by0[i] := y0 + 0.9; bx1[i] := x1 + 1.4; by1[i] := y1 - 0.9;
      end else begin
        bx0[i] := x0 + 0.9; by0[i] := y0 - 1.4; bx1[i] := x1 - 0.9; by1[i] := y1 + 1.4;
      end;
    end else begin
      m := pad / 2 + 0.4;
      bx0[i] := x0 - m; by0[i] := y0 - m; bx1[i] := x1 + m; by1[i] := y1 + m;
    end;
    inset := 0.4;
    if bx0[i] < s.bd.outline[0] + inset then bx0[i] := s.bd.outline[0] + inset;
    if by0[i] < s.bd.outline[1] + inset then by0[i] := s.bd.outline[1] + inset;
    if bx1[i] > s.bd.outline[2] - inset then bx1[i] := s.bd.outline[2] - inset;
    if by1[i] > s.bd.outline[3] - inset then by1[i] := s.bd.outline[3] - inset;
  end;
end;

procedure TSimUI.FitView;
var bw, bh, w, h: Float;
begin
  w := boardCv.width;
  h := boardCv.height;
  bw := FBoard.sim.bd.outline[2] - FBoard.sim.bd.outline[0];
  bh := FBoard.sim.bd.outline[3] - FBoard.sim.bd.outline[1];
  zoom := w / bw;
  if h / bh < zoom then zoom := h / bh;
  zoom := zoom * 0.98;
  ox := (w - bw * zoom) / 2 - FBoard.sim.bd.outline[0] * zoom;
  oy := (h - bh * zoom) / 2 - FBoard.sim.bd.outline[1] * zoom;
  fitted := True;
end;

function TSimUI.PinInteractive(pin: Integer): Boolean;
var p: TPart;
begin
  p := FBoard.sim.parts[FBoard.sim.pins[pin].part];
  Result := (p.kind = PART_IC) or (p.kind = PART_CONNECTOR);
end;

function TSimUI.PickPin(mx, my: Float): Integer;
var
  i: Integer;
  bd: TBdPad;
  hw, hh: Float;
begin
  for i := 0 to FBoard.sim.npins - 1 do begin
    if not PinInteractive(i) then continue;
    bd := FBoard.sim.bd.pads[i];
    hw := bd.w / 2; if hw < 0.7 then hw := 0.7;
    hh := bd.h / 2; if hh < 0.7 then hh := 0.7;
    if (Abs(mx - bd.x) <= hw) and (Abs(my - bd.y) <= hh) then exit(i);
  end;
  Result := -1;
end;

function TSimUI.PickPart(mx, my: Float): Integer;
var pass, i: Integer;
begin
  for pass := 0 to 1 do
    for i := 0 to FBoard.sim.nparts - 1 do begin
      if (FBoard.sim.parts[i].kind = PART_IC) <> (pass = 0) then continue;
      if not FBoard.PartSimulated(i) then continue;
      if (mx >= bx0[i]) and (mx <= bx1[i]) and (my >= by0[i]) and (my <= by1[i]) then exit(i);
    end;
  Result := -1;
end;

{ ------------------------------------------------------------------------ }
{ Main loop                                                                 }
{ ------------------------------------------------------------------------ }

procedure TSimUI.RunSlice;
var
  t0, t1, budget, limit, ran: Float;
  chunk, n: Integer;
  msg: String;
begin
  if paused then begin lastSliceMs := 0; exit; end;
  budget := 28;
  t0 := NowMs;
  { never faster than the real machine (only the Apple's fast mode gets there) }
  if lastSliceMs = 0 then lastSliceMs := t0 - 17;
  limit := (t0 - lastSliceMs) * FBoard.cpu_hz / 1000;
  if limit > FBoard.cpu_hz / 10 then limit := FBoard.cpu_hz / 10;
  lastSliceMs := t0;
  ran := 0;
  chunk := FBoard.cycles_per_frame div 32;
  repeat
    n := chunk;
    if cycleInFrame + n > FBoard.cycles_per_frame then n := FBoard.cycles_per_frame - cycleInFrame;
    FBoard.Run(n);
    cycleInFrame := cycleInFrame + n;
    speedAcc := speedAcc + n;
    if cycleInFrame >= FBoard.cycles_per_frame then begin
      cycleInFrame := 0;
      Inc(emuFrames);
      FBoard.FrameUpdate(20000);
      if (pendingLoad <> '') and (FBoard.sim.cycle >= FBoard.ready_cycles) then begin
        FBoard.Load(pendingLoad, pendingBytes, True, msg);
        Status(msg);
        pendingLoad := '';
      end;
    end;
    t1 := NowMs;
    ran := ran + n;
  until (t1 - t0 >= budget) or (ran >= limit);
  { wall time (with the throttle, time spent outside the slices counts too) }
  if speedT0 = 0 then speedT0 := t0;
  speedReal := t1 - speedT0;
  if speedReal >= 1000 then begin
    speedT0 := t1;
    speedPct := 100 * (speedAcc / FBoard.cpu_hz) / (speedReal / 1000);
    speedAcc := 0;
    speedReal := 0;
  end;
end;

procedure TSimUI.StartAudio;
var ctx: Variant;
begin
  if not soundChk.checked then exit;
  ctx := audioCtx;
  asm
    if (!@ctx) { var AC = window.AudioContext || window.webkitAudioContext; if (AC) @ctx = new AC(); }
    if (@ctx && (@ctx).state === 'suspended') (@ctx).resume();
  end;
  audioCtx := ctx;
end;

{ The samples the machine made since the last frame, queued behind the ones
  already playing (about 50 ms ahead). }
procedure TSimUI.PlayAudio;
var
  n, rate: Integer;
  buf: TFloats;
  ctx: Variant;
  t: Float;
begin
  n := FBoard.audio_n;
  rate := FBoard.audio_rate;
  FBoard.audio_n := 0;
  if (rate = 0) or (n = 0) or not soundChk.checked then exit;
  ctx := audioCtx;
  buf := FBoard.audio;
  t := audioTime;
  asm
    if (@ctx && (@ctx).state === 'running') {
      var b = (@ctx).createBuffer(1, @n, @rate);
      var d = b.getChannelData(0);
      for (var i = 0; i < @n; i++) d[i] = (@buf)[i] * 0.3;
      var src = (@ctx).createBufferSource();
      src.buffer = b;
      src.connect((@ctx).destination);
      if (@t < (@ctx).currentTime + 0.02 || @t > (@ctx).currentTime + 0.3) @t = (@ctx).currentTime + 0.05;
      src.start(@t);
      @t = @t + @n / @rate;
    }
  end;
  audioTime := t;
end;

procedure TSimUI.Frame(ts: Float);
var i: Integer;
begin
  RunSlice;
  PlayAudio;
  for i := 0 to FBoard.sim.nnets - 1 do netState[i] := FBoard.NetState(i);
  FBoard.StatsReset;
  DrawBoard;
  DrawMonitor;
  DrawScope;
  UpdateFaults;
  UpdateInfo;
  speedEl.textContent := 'Speed: ' + IntToStr(Round(speedPct)) + '% of real time   ' +
    'Emulated: ' + FloatToStr(Round(FBoard.sim.cycle / FBoard.cpu_hz * 10) / 10) + ' s';
  cpuEl.textContent := 'CPU ' + FBoard.CpuStatus + ': ' + FBoard.CpuRegs;
  win.requestAnimationFrame(procedure(t: Float) begin Frame(t); end);
end;

{ ------------------------------------------------------------------------ }
{ Drawing                                                                   }
{ ------------------------------------------------------------------------ }

procedure TSimUI.DrawPart(i: Integer);
var
  s: TSim;
  p: TPart;
  x, y, w, h, px, py, cx, cy, fs, ty: Float;
  sim, mem, dongle, vertical: Boolean;
  role, p1, k, n: Integer;
  lab, cart, cname: String;
  ctx: Variant;
begin
  s := FBoard.sim;
  ctx := boardCtx;
  p := s.parts[i];
  x := ox + bx0[i] * zoom;
  y := oy + by0[i] * zoom;
  w := (bx1[i] - bx0[i]) * zoom;
  h := (by1[i] - by0[i]) * zoom;
  sim := FBoard.PartSimulated(i);
  case p.kind of
    PART_IC: begin
      role := FBoard.PartRole(i);
      if p.removed then begin
        ctx.strokeStyle := COL_SOCKET;
        ctx.lineWidth := 2;
        ctx.strokeRect(x, y, w, h);
      end else begin
        mem := (role = ROLE_RAM) or (role = ROLE_VRAM) or (role = ROLE_ROM) or (role = ROLE_PLA);
        ctx.fillStyle := COL_CHIP;
        ctx.fillRect(x, y, w, h);
        if mem then begin ctx.strokeStyle := ROLE_COLORS[role]; ctx.lineWidth := 2; end
        else begin ctx.strokeStyle := COL_CHIP_EDGE; ctx.lineWidth := 1; end;
        ctx.strokeRect(x, y, w, h);
      end;
      p1 := p.Pin(1);
      if p1 >= 0 then begin
        px := ox + s.bd.pads[p1].x * zoom;
        py := oy + s.bd.pads[p1].y * zoom;
        cx := px; if cx < x + 6 then cx := x + 6; if cx > x + w - 6 then cx := x + w - 6;
        cy := py; if cy < y + 6 then cy := y + 6; if cy > y + h - 6 then cy := y + h - 6;
        if p.removed then ctx.fillStyle := COL_SOCKET else ctx.fillStyle := COL_CHIP_EDGE;
        ctx.beginPath();
        ctx.arc(cx, cy, 3, 0, 6.2832);
        ctx.fill();
      end;
      { the name: horizontal if it fits, else stacked letters }
      lab := FBoard.PartLabel(i);
      if p.removed then ctx.fillStyle := COL_SOCKET else ctx.fillStyle := ROLE_COLORS[role];
      fs := zoom * 3.2;
      if fs > 16 then fs := 16;
      if fs < 8 then fs := 8;
      begin
        ctx.font := 'bold ' + IntToStr(Trunc(fs)) + 'px sans-serif';
        ctx.textAlign := 'center';
        ctx.textBaseline := 'middle';
        vertical := (h > w) and (ctx.measureText(lab).width > w - 4);
        if (not vertical) and (ctx.measureText(lab).width > w - 4) then begin
          { too wide for a flat part (e.g. a 1x16 connector): shrink to fit,
            and fall back to the reference if that gets unreadable }
          fs := fs * (w - 4) / ctx.measureText(lab).width;
          if fs < 7 then begin
            lab := p.ref;
            fs := zoom * 3.2;
            if fs > 16 then fs := 16;
            if fs < 8 then fs := 8;
            ctx.font := 'bold ' + IntToStr(Trunc(fs)) + 'px sans-serif';
            if ctx.measureText(lab).width > w - 4 then
              fs := fs * (w - 4) / ctx.measureText(lab).width;
          end;
          if fs < 6 then fs := 6;
          ctx.font := 'bold ' + IntToStr(Trunc(fs)) + 'px sans-serif';
        end;
        if not vertical then begin
          ctx.fillText(lab, x + w / 2, y + h / 2);
          if (zoom >= 6) and (h > fs * 3) then begin
            ctx.font := IntToStr(Trunc(fs * 0.7)) + 'px sans-serif';
            ctx.fillStyle := COL_DIMTEXT;
            ctx.fillText(p.ref, x + w / 2, y + h / 2 + fs);
          end;
        end else begin
          n := Length(lab);
          ty := y + h / 2 - (n - 1) * fs * 0.55;
          for k := 1 to n do begin
            ctx.fillText(Copy(lab, k, 1), x + w / 2, ty);
            ty := ty + fs * 1.1;
          end;
        end;
      end;
    end;
    PART_RESISTOR, PART_LINK: begin
      if not sim then begin
        ctx.strokeStyle := COL_OTHER;
        ctx.lineWidth := 1;
        ctx.strokeRect(x, y, w, h);
        exit;
      end;
      if p.kind = PART_LINK then ctx.fillStyle := COL_LINK else ctx.fillStyle := COL_RES;
      ctx.fillRect(x, y, w, h);
      if p.rfault <> RF_OK then begin
        ctx.strokeStyle := COL_FAULT;
        ctx.lineWidth := 2;
        ctx.strokeRect(x, y, w, h);
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + w, y + h);
        ctx.stroke();
      end;
    end;
    PART_CONNECTOR: begin
      cart := '';
      if (FBoard.cart_slot <> '') and (p.ref = FBoard.cart_slot) then cart := FBoard.CartName;
      dongle := (FBoard is TC64) and TC64(FBoard).harness and
                ((p.ref = 'CN1') or (p.ref = 'CN2') or (p.ref = 'CN3') or (p.ref = 'CN4') or
                 (p.ref = 'CN8') or (p.ref = 'CN9'));
      if dongle then begin
        ctx.strokeStyle := COL_HARNESS;
        ctx.lineWidth := 2;
        ctx.strokeRect(x - 4, y - 4, w + 8, h + 8);
      end;
      if cart <> '' then begin
        ctx.fillStyle := '#3c3228';
        ctx.fillRect(x - 6, y - 6, w + 12, h + 12);
        ctx.strokeStyle := COL_SOCKET;
        ctx.lineWidth := 2;
        ctx.strokeRect(x - 6, y - 6, w + 12, h + 12);
      end;
      ctx.strokeStyle := COL_DIMTEXT;
      ctx.lineWidth := 1;
      ctx.strokeRect(x, y, w, h);
      cname := FBoard.ConnectorName(i);
      if cart <> '' then lab := p.ref + ' cartridge: ' + cart
      else if dongle then lab := p.ref + ' + harness'
      else lab := Trim(p.ref + ' ' + cname);
      ctx.font := '11px sans-serif';
      ctx.textAlign := 'center';
      ctx.textBaseline := 'bottom';
      if cart <> '' then ctx.fillStyle := COL_SOCKET
      else if dongle then ctx.fillStyle := COL_HARNESS
      else ctx.fillStyle := COL_DIMTEXT;
      if (cart <> '') or dongle or (ctx.measureText(lab).width < w + 30) then
        ctx.fillText(lab, x + w / 2, y - 3)
      else if FBoard is TApple2 then begin
        { Apple II connectors sit alone in their row: the name may run right }
        ctx.textAlign := 'left';
        ctx.fillText(lab, x, y - 3);
      end;
    end;
  else
    if sim then begin
      ctx.fillStyle := '#aaaab4';
      ctx.fillRect(x, y, w, h);
      if p.rfault <> RF_OK then begin
        ctx.strokeStyle := COL_FAULT;
        ctx.lineWidth := 2;
        ctx.strokeRect(x, y, w, h);
      end;
    end else begin
      ctx.strokeStyle := COL_OTHER;
      ctx.lineWidth := 1;
      ctx.strokeRect(x, y, w, h);
    end;
  end;
end;

procedure TSimUI.DrawPins;
var
  s: TSim;
  ctx: Variant;
  i, hoverRoot: Integer;
  bd: TBdPad;
  p: TPin;
  sz, x, y, m: Float;
  removed: Boolean;
  c: String;
begin
  s := FBoard.sim;
  ctx := boardCtx;
  if hoverPin >= 0 then hoverRoot := s.pins[hoverPin].root else hoverRoot := -2;
  ctx.textAlign := 'center';
  ctx.textBaseline := 'middle';
  for i := 0 to s.npins - 1 do begin
    bd := s.bd.pads[i];
    p := s.pins[i];
    m := bd.w; if bd.h < m then m := bd.h;
    sz := m * zoom * 0.85;
    if sz < 2 then sz := 2;
    x := ox + bd.x * zoom;
    y := oy + bd.y * zoom;
    if not PinInteractive(i) then begin
      ctx.fillStyle := '#968c6e';
      ctx.fillRect(x - sz / 2, y - sz / 2, sz, sz);
      continue;
    end;
    removed := s.parts[p.part].removed;
    if p.net < 0 then c := COL_NONE else c := StateColor(netState[p.net]);
    if removed then c := '#5a503c';
    ctx.fillStyle := c;
    ctx.fillRect(x - sz / 2, y - sz / 2, sz, sz);
    if p.fault <> PF_OK then begin
      ctx.strokeStyle := COL_FAULT;
      ctx.lineWidth := 2;
      ctx.strokeRect(x - sz / 2 - 2, y - sz / 2 - 2, sz + 4, sz + 4);
      if p.fault = PF_OPEN then begin
        ctx.strokeStyle := '#000';
        ctx.lineWidth := 1.5;
        ctx.beginPath();
        ctx.moveTo(x - sz / 2, y - sz / 2); ctx.lineTo(x + sz / 2, y + sz / 2);
        ctx.moveTo(x + sz / 2, y - sz / 2); ctx.lineTo(x - sz / 2, y + sz / 2);
        ctx.stroke();
      end;
    end;
    if (not removed) and (sz >= 11) then begin
      ctx.fillStyle := '#000';
      ctx.font := IntToStr(Trunc(sz * 0.6)) + 'px sans-serif';
      ctx.fillText(bd.name, x, y + 0.5);
    end;
    if (p.root = hoverRoot) and (p.root >= 0) then begin
      ctx.strokeStyle := COL_HOVER;
      ctx.lineWidth := 1.5;
      ctx.strokeRect(x - sz / 2 - 1, y - sz / 2 - 1, sz + 2, sz + 2);
    end;
    if i = FBoard.scope_pin then begin
      ctx.strokeStyle := COL_TRACE;
      ctx.lineWidth := 2;
      ctx.strokeRect(x - sz / 2 - 3, y - sz / 2 - 3, sz + 6, sz + 6);
    end;
  end;
end;

procedure TSimUI.DrawBoard;
var
  ctx: Variant;
  o: array[0..3] of Float;
  i: Integer;
begin
  if not fitted then FitView;
  ctx := boardCtx;
  ctx.fillStyle := '#101512';
  ctx.fillRect(0, 0, boardCv.width, boardCv.height);
  o[0] := FBoard.sim.bd.outline[0]; o[1] := FBoard.sim.bd.outline[1];
  o[2] := FBoard.sim.bd.outline[2]; o[3] := FBoard.sim.bd.outline[3];
  ctx.fillStyle := COL_PCB;
  ctx.fillRect(ox + o[0] * zoom, oy + o[1] * zoom, (o[2] - o[0]) * zoom, (o[3] - o[1]) * zoom);
  ctx.strokeStyle := COL_PCB_EDGE;
  ctx.lineWidth := 2;
  ctx.strokeRect(ox + o[0] * zoom, oy + o[1] * zoom, (o[2] - o[0]) * zoom, (o[3] - o[1]) * zoom);
  { passives first, chips and connectors on top }
  for i := 0 to FBoard.sim.nparts - 1 do
    if FBoard.sim.parts[i].kind <> PART_IC then DrawPart(i);
  for i := 0 to FBoard.sim.nparts - 1 do
    if FBoard.sim.parts[i].kind = PART_IC then DrawPart(i);
  DrawPins;
end;

procedure TSimUI.DrawMonitor;
var i, n: Integer;
begin
  FBoard.Monitor(monBuf);
  n := FBoard.screen_w * FBoard.screen_h;
  for i := 0 to n - 1 do monPix[i] := monBuf[i];
  monCtx.putImageData(monImg, 0, 0);
end;

procedure TSimUI.DrawScope;
var
  ctx: Variant;
  w, h, n, i, v, pos, yhi, ylo: Integer;
  x0, x1, y: Float;
  pname: String;
begin
  ctx := scopeCtx;
  w := scopeCv.width;
  h := scopeCv.height;
  ctx.fillStyle := '#06120a';
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle := '#14301c';
  ctx.lineWidth := 1;
  for i := 1 to 9 do begin
    ctx.beginPath();
    ctx.moveTo(i * w / 10, 0);
    ctx.lineTo(i * w / 10, h);
    ctx.stroke();
  end;
  if FBoard.scope_pin < 0 then begin
    ctx.fillStyle := COL_DIMTEXT;
    ctx.font := '12px sans-serif';
    ctx.textAlign := 'center';
    ctx.textBaseline := 'middle';
    ctx.fillText('click a pin and choose "Scope" to probe it', w / 2, h / 2);
    exit;
  end;
  n := TIMEBASES[timebase] * FBoard.steps_per_cycle;
  if n > SCOPE_SAMPLES then n := SCOPE_SAMPLES;
  yhi := 22;
  ylo := h - 14;
  pos := FBoard.scope_pos;
  for i := 0 to n - 1 do begin
    v := FBoard.scope[(pos - n + i) and (SCOPE_SAMPLES - 1)];
    x0 := i * w / n;
    x1 := (i + 1) * w / n;
    if (v and SC_LEVEL) <> 0 then y := yhi else y := ylo;
    if (v and SC_FIGHT) <> 0 then ctx.strokeStyle := COL_FIGHT
    else if (v and SC_FLOATING) <> 0 then ctx.strokeStyle := COL_FLOAT
    else ctx.strokeStyle := COL_TRACE;
    ctx.lineWidth := 2;
    ctx.beginPath();
    ctx.moveTo(x0, y);
    ctx.lineTo(x1, y);
    if i + 1 < n then begin
      if (FBoard.scope[(pos - n + i + 1) and (SCOPE_SAMPLES - 1)] and SC_LEVEL) <> (v and SC_LEVEL) then begin
        ctx.moveTo(x1, yhi);
        ctx.lineTo(x1, ylo);
      end;
    end;
    ctx.stroke();
  end;
  pname := FBoard.sim.parts[FBoard.sim.pins[FBoard.scope_pin].part].ref + '.' +
    FBoard.sim.bd.pads[FBoard.scope_pin].name + ' ' + FBoard.sim.pin_names[FBoard.scope_pin];
  ctx.fillStyle := COL_TEXT;
  ctx.font := '12px sans-serif';
  ctx.textAlign := 'left';
  ctx.textBaseline := 'top';
  ctx.fillText(pname + '   (' + IntToStr(TIMEBASES[timebase]) + ' cycles)', 6, 4);
end;

procedure TSimUI.UpdateFaults;
var
  s: TSim;
  i: Integer;
  sig, txt: String;
  items: array of String;
  specs: array of String;
  row, b: Variant;
begin
  s := FBoard.sim;
  for i := 0 to s.npins - 1 do
    if s.pins[i].fault <> PF_OK then begin
      case s.pins[i].fault of
        PF_OPEN: txt := 'open';
        PF_GND: txt := 'gnd';
      else
        txt := 'vcc';
      end;
      items.Add(s.parts[s.pins[i].part].ref + '.' + s.bd.pads[i].name + ' (' + s.pin_names[i] + '): ' + txt);
      specs.Add(s.parts[s.pins[i].part].ref + '.' + s.bd.pads[i].name + ':ok');
    end;
  for i := 0 to s.nparts - 1 do begin
    if s.parts[i].removed then begin
      items.Add(s.parts[i].ref + ' (' + FBoard.PartLabel(i) + '): removed');
      specs.Add(s.parts[i].ref + ':present');
    end;
    if s.parts[i].rfault <> RF_OK then begin
      if s.parts[i].rfault = RF_OPEN then txt := 'open' else txt := 'short';
      items.Add(s.parts[i].ref + ': ' + txt);
      specs.Add(s.parts[i].ref + ':ok');
    end;
  end;
  if FBoard.reset_button then begin
    items.Add('RESET button pressed');
    specs.Add('RESET:released');
  end;
  sig := '';
  for i := 0 to items.High do sig := sig + items[i] + '|';
  if sig = lastFaultSig then exit;
  lastFaultSig := sig;
  faultsEl.innerHTML := '';
  if items.Length = 0 then begin
    El('div', 'nofault', faultsEl).textContent := 'none: a healthy board';
    exit;
  end;
  for i := 0 to items.High do begin
    row := El('div', 'fault', faultsEl);
    El('span', '', row).textContent := items[i];
    b := El('button', 'small', row);
    b.textContent := 'repair';
    b.dataset.spec := specs[i];
    b.onclick := procedure(e: Variant)
      begin
        FBoard.ApplyFaultSpec(e.target.dataset.spec);
      end;
  end;
end;

procedure TSimUI.UpdateInfo;
var
  s: TSim;
  pin, part, net: Integer;
  txt, func: String;
begin
  s := FBoard.sim;
  txt := FBoard.board_name + ' — ' + FBoard.keys_help;
  if hoverPin >= 0 then begin
    pin := hoverPin;
    part := s.pins[pin].part;
    net := s.pins[pin].net;
    txt := s.parts[part].ref + ' pin ' + s.bd.pads[pin].name;
    if s.pin_names[pin] <> '' then txt := txt + ' (' + s.pin_names[pin] + ')';
    func := FBoard.PinFunc(pin);
    if func <> '' then txt := txt + ' ' + func;
    if net >= 0 then txt := txt + ' — net ' + s.nets[net].name + ': ' + StateName(netState[net])
    else txt := txt + ' — not connected';
    txt := txt + ' — ' + FBoard.PartInfo(part);
  end else if hoverPart >= 0 then begin
    txt := s.parts[hoverPart].ref + ' ' + FBoard.PartInfo(hoverPart);
    if txt = s.parts[hoverPart].ref + ' ' then txt := s.parts[hoverPart].ref + ' ' + s.parts[hoverPart].name;
  end;
  infoEl.textContent := txt;
end;

{ ------------------------------------------------------------------------ }
{ Fault menu                                                                }
{ ------------------------------------------------------------------------ }

procedure TSimUI.CloseMenu;
begin
  menuEl.style.display := 'none';
  menuEl.innerHTML := '';
end;

procedure TSimUI.MenuItem(const text: String; handler: procedure);
var b: Variant;
begin
  b := El('button', 'mitem', menuEl);
  b.textContent := text;
  b.onclick := procedure(e: Variant)
    begin
      CloseMenu;
      handler();
    end;
end;

procedure TSimUI.SelectScopePin(pin: Integer);
begin
  FBoard.scope_pin := pin;
  FBoard.scope_pos := 0;
  FBoard.scope.fill(0);
end;

procedure TSimUI.OpenMenu(pin, part: Integer; x, y: Float);
var
  s: TSim;
  title: String;
  p: TPart;
begin
  s := FBoard.sim;
  menuEl.innerHTML := '';
  if pin >= 0 then begin
    title := s.parts[s.pins[pin].part].ref + ' pin ' + s.bd.pads[pin].name;
    if s.pin_names[pin] <> '' then title := title + ' ' + s.pin_names[pin];
    El('div', 'mtitle', menuEl).textContent := title;
    MenuItem('Put on the oscilloscope', procedure begin SelectScopePin(pin); end);
    MenuItem('OK (repair)', procedure begin FBoard.SetPinFault(pin, PF_OK); end);
    MenuItem('Lift the leg (open)', procedure begin FBoard.SetPinFault(pin, PF_OPEN); end);
    MenuItem('Short to GND', procedure begin FBoard.SetPinFault(pin, PF_GND); end);
    MenuItem('Short to +5V', procedure begin FBoard.SetPinFault(pin, PF_VCC); end);
  end else begin
    p := s.parts[part];
    El('div', 'mtitle', menuEl).textContent := p.ref + ' ' + FBoard.PartLabel(part);
    if p.kind = PART_IC then begin
      if p.removed then MenuItem('Put it back in its socket', procedure begin FBoard.SetRemoved(part, False); end)
      else MenuItem('Pull it out of its socket', procedure begin FBoard.SetRemoved(part, True); end);
    end else begin
      MenuItem('OK (repair)', procedure begin FBoard.SetRFault(part, RF_OK); end);
      MenuItem('Open', procedure begin FBoard.SetRFault(part, RF_OPEN); end);
      MenuItem('Short', procedure begin FBoard.SetRFault(part, RF_SHORT); end);
    end;
  end;
  MenuItem('Cancel', procedure begin end);
  menuEl.style.left := FloatToStr(x) + 'px';
  menuEl.style.top := FloatToStr(y) + 'px';
  menuEl.style.display := 'block';
end;

{ ------------------------------------------------------------------------ }
{ Input                                                                     }
{ ------------------------------------------------------------------------ }

procedure TSimUI.OnMouseDown(e: Variant);
begin
  if e.button <> 0 then exit;
  dragging := True;
  dragMoved := False;
  dragX := e.clientX;
  dragY := e.clientY;
  e.preventDefault();
end;

procedure TSimUI.OnMouseMove(e: Variant);
var
  r: Variant;
  mx, my: Float;
begin
  r := boardCv.getBoundingClientRect();
  mouseX := e.clientX - r.left;
  mouseY := e.clientY - r.top;
  if dragging then begin
    if (Abs(e.clientX - dragX) > 3) or (Abs(e.clientY - dragY) > 3) or dragMoved then begin
      dragMoved := True;
      ox := ox + (e.clientX - dragX);
      oy := oy + (e.clientY - dragY);
      dragX := e.clientX;
      dragY := e.clientY;
    end;
    exit;
  end;
  if (mouseX < 0) or (mouseY < 0) or (mouseX > boardCv.width) or (mouseY > boardCv.height) then begin
    hoverPin := -1;
    hoverPart := -1;
    exit;
  end;
  mx := (mouseX - ox) / zoom;
  my := (mouseY - oy) / zoom;
  hoverPin := PickPin(mx, my);
  if hoverPin < 0 then hoverPart := PickPart(mx, my) else hoverPart := -1;
end;

procedure TSimUI.OnMouseUp(e: Variant);
var
  r: Variant;
  mx, my: Float;
  pin, part: Integer;
begin
  if not dragging then exit;
  dragging := False;
  if dragMoved then exit;
  r := boardCv.getBoundingClientRect();
  mx := (e.clientX - r.left - ox) / zoom;
  my := (e.clientY - r.top - oy) / zoom;
  pin := PickPin(mx, my);
  part := -1;
  if pin < 0 then part := PickPart(mx, my);
  if (pin < 0) and (part < 0) then begin CloseMenu; exit; end;
  OpenMenu(pin, part, e.pageX + 4, e.pageY + 4);
end;

procedure TSimUI.OnWheel(e: Variant);
var f, mx, my: Float;
begin
  e.preventDefault();
  if e.deltaY < 0 then f := 1.2 else f := 1 / 1.2;
  mx := (mouseX - ox) / zoom;
  my := (mouseY - oy) / zoom;
  zoom := zoom * f;
  if zoom < 0.5 then zoom := 0.5;
  if zoom > 80 then zoom := 80;
  ox := mouseX - mx * zoom;
  oy := mouseY - my * zoom;
end;

procedure TSimUI.OnKey(e: Variant; down: Boolean);
var
  key: String;
  code: Integer;
begin
  key := e.key;
  if Length(key) = 1 then code := FBoard.HostChar(Ord(key[1]))
  else code := FBoard.HostKey(key);
  if code = 0 then exit;
  e.preventDefault();
  if down then begin
    if e.&repeat then exit;
    FBoard.KeyDown(code);
  end else
    FBoard.KeyUp(code);
end;

{ ------------------------------------------------------------------------ }
{ Files                                                                     }
{ ------------------------------------------------------------------------ }

procedure TSimUI.LoadBytes(const name: String; const bytes: TBytes);
var
  lp, msg, info: String;
  bad: Boolean;
  k: TFileKind;
begin
  lp := LowerCase(name);
  { the machine that takes this kind of file (ROMs for the Apple while it is on the bench) }
  if (FBoard is TApple2) and ((Pos('.rom', lp) > 0) or (Pos('.bin', lp) > 0)) then
  else if (FBoard is TLc80) and ((Pos('.bin', lp) > 0) or (Pos('.hex', lp) > 0)) then
  else if (Pos('.prg', lp) > 0) or (Pos('.crt', lp) > 0) then begin
    if not (FBoard is TC64) then SwitchMachine('c64');
  end else if (Pos('.sna', lp) > 0) or (Pos('.z80', lp) > 0) or (Pos('.rom', lp) > 0) then begin
    if not (FBoard is TZX48) then SwitchMachine('zx48');
  end;
  k := FBoard.FileKind(name, bytes, info, bad);
  if k = nil then begin Status(name + ': not a file this machine takes'); exit; end;
  if bad then begin Status(name + ': ' + info); exit; end;
  if (FBoard is TC64) and (Pos('.prg', lp) > 0) and (FBoard.sim.cycle < FBoard.ready_cycles) then begin
    pendingLoad := name;
    pendingBytes := bytes;
    Status(name + ' (' + info + ') will be loaded once BASIC is ready');
    exit;
  end;
  FBoard.Load(name, bytes, True, msg);
  Status(msg);
  if FBoard is TC64 then harnessChk.checked := TC64(FBoard).harness;
end;

procedure TSimUI.LoadBuiltin(const name: String);
begin
  if name = 'diag-586220.crt' then begin
    if not (FBoard is TC64) then SwitchMachine('c64');
    TC64(FBoard).SetHarness(True);
    harnessChk.checked := True;
  end;
  LoadBytes(name, BuiltinFile(name));
end;

end.

