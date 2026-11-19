unit app.body;

{ Headless driver (node.js), like 'c64-hw-sim --headless FRAMES':

    node index.js --headless 200 [--machine c64|zx48] [--text] [--fault SPEC]...
                  [--at FRAME:SPEC]... [--cart FILE] [--harness] [--nets] [prg]
}

interface

uses
  qtx.sysutils,
  qtx.classes,
  qtx.components,
  qtx.node.application,
  hwsim.util, hwsim.sim, hwsim.parts, hwsim.board, hwsim.roms,
  hwsim.c64, hwsim.z80, hwsim.zx48, hwsim.apple2, hwsim.lc80;

type
  TNodeApplication = class(TQTXNodeApplication)
  public
    procedure DoExecute; override;
  end;

implementation

function NodeArgs: array of String;
begin
  asm @Result = process.argv.slice(2); end;
end;

function NowMs: Float;
begin
  asm @Result = Date.now(); end;
end;

function ReadFileBytes(const path: String): TBytes;
begin
  Result := BuiltinFile(path);
  if Result <> nil then exit;
  asm
    var fs = require('fs');
    var b = fs.readFileSync(@path);
    @Result = new Uint8Array(b.buffer, b.byteOffset, b.length);
  end;
end;

function Fix2(v: Float): String;
begin
  asm @Result = (@v).toFixed(2); end;
end;

procedure SavePng(const path: String; px: TUInts; w, h: Integer);
begin
  asm
    var zlib = require('zlib'), fs = require('fs');
    var raw = Buffer.alloc((@w * 4 + 1) * @h);
    for (var y = 0; y < @h; y++) {
      raw[y * (@w * 4 + 1)] = 0;
      for (var x = 0; x < @w; x++) {
        var c = (@px)[y * @w + x], o = y * (@w * 4 + 1) + 1 + x * 4;
        raw[o] = c & 255; raw[o + 1] = (c >>> 8) & 255; raw[o + 2] = (c >>> 16) & 255; raw[o + 3] = 255;
      }
    }
    function crc32(b) { var c, crc = -1; for (var i = 0; i < b.length; i++) { c = (crc ^ b[i]) & 255;
      for (var k = 0; k < 8; k++) c = c & 1 ? (c >>> 1) ^ 0xEDB88320 : c >>> 1; crc = (crc >>> 8) ^ c; } return (crc ^ -1) >>> 0; }
    function chunk(t, d) { var l = Buffer.alloc(4); l.writeUInt32BE(d.length); var td = Buffer.concat([Buffer.from(t), d]);
      var c = Buffer.alloc(4); c.writeUInt32BE(crc32(td)); return Buffer.concat([l, td, c]); }
    var ih = Buffer.alloc(13); ih.writeUInt32BE(@w, 0); ih.writeUInt32BE(@h, 4); ih[8] = 8; ih[9] = 6;
    fs.writeFileSync(@path, Buffer.concat([Buffer.from([137,80,78,71,13,10,26,10]), chunk('IHDR', ih),
      chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]));
  end;
end;

function Env(const name: String): String;
begin
  asm @Result = process.env[@name] || ''; end;
end;

procedure TestZ80(n: Integer);
var
  mem, rom: TBytes;
  cpu: TZ80;
  pins, i, a, d, ctl, h: Integer;
begin
  mem := NewBytes(65536);
  rom := ROM_ZX48_ROM;
  for i := 0 to 16383 do mem[i] := rom[i];
  for i := 16384 to 65535 do mem[i] := (i * 7) and $FF;
  cpu := TZ80.Create;
  pins := cpu.Init;
  h := 0;
  for i := 0 to n - 1 do begin
    pins := pins xor Z80_CLK;
    if (i mod 140000) < 64 then pins := pins or Z80_INT else pins := pins and not Z80_INT;
    pins := cpu.Tick(pins);
    a := cpu.ab;
    d := 0;
    if ((pins and Z80_MREQ) <> 0) and ((pins and Z80_RD) <> 0) then begin
      d := mem[a]; pins := (pins and not $FF0000) or (d shl 16);
    end else if ((pins and Z80_MREQ) <> 0) and ((pins and Z80_WR) <> 0) then begin
      d := (pins shr 16) and $FF;
      if a >= 16384 then mem[a] := d;
    end else if ((pins and Z80_IORQ) <> 0) and ((pins and Z80_RD) <> 0) then begin
      d := ($BF xor (i shr 8)) and $FF; pins := (pins and not $FF0000) or (d shl 16);
    end else if (pins and Z80_IORQ) <> 0 then begin
      d := $FF; pins := (pins and not $FF0000) or (d shl 16);
    end;
    ctl := 0;
    if (pins and Z80_M1) <> 0 then ctl := ctl or 1;
    if (pins and Z80_MREQ) <> 0 then ctl := ctl or 2;
    if (pins and Z80_IORQ) <> 0 then ctl := ctl or 4;
    if (pins and Z80_RD) <> 0 then ctl := ctl or 8;
    if (pins and Z80_WR) <> 0 then ctl := ctl or 16;
    if (pins and Z80_HALT) <> 0 then ctl := ctl or 32;
    if (pins and Z80_RFSH) <> 0 then ctl := ctl or 64;
    h := U32(Imul(h, 31) + a + d * 65536 + Imul(ctl, $1000000) + cpu.PC * 3 + cpu.AF * 5 + cpu.BC * 7 + cpu.DE * 11 + cpu.HL * 13 + cpu.SP * 17);
    if (i and $7FFFF) = $7FFFF then
      WriteLn(IntToStr(i) + ' PC=' + HexW(cpu.PC) + ' AF=' + HexW(cpu.AF) + ' BC=' + HexW(cpu.BC) + ' DE=' + HexW(cpu.DE) +
        ' HL=' + HexW(cpu.HL) + ' SP=' + HexW(cpu.SP) + ' IX=' + HexW(cpu.IX) + ' IY=' + HexW(cpu.IY) + ' h=' + IntToHex(h, 8));
  end;
  WriteLn('final h=' + IntToHex(h, 8));
end;

procedure TNodeApplication.DoExecute;
var
  args, faults, atSpecs, lines, probes, roms: array of String;
  atFrames: array of Integer;
  i, f, frames, ready, k, typed, next_type, c: Integer;
  machine, cartFile, prg, msg, a, typ, shot: String;
  text, harnessOpt, nets: Boolean;
  b: TBoard;
  t0, secs, emulated: Float;
  n: TNet;
  tot: Integer;
begin
  args := NodeArgs;
  frames := 0;
  machine := 'c64';
  text := False;
  harnessOpt := False;
  nets := False;
  i := 0;
  while i <= args.High do begin
    a := args[i];
    if (a = '--headless') and (i < args.High) then begin Inc(i); frames := StrToInt(args[i]); end
    else if (a = '--machine') and (i < args.High) then begin Inc(i); machine := args[i]; end
    else if a = '--text' then text := True
    else if a = '--nets' then nets := True
    else if (a = '--shot') and (i < args.High) then begin Inc(i); shot := args[i]; end
    else if a = '--harness' then harnessOpt := True
    else if (a = '--probe') and (i < args.High) then begin Inc(i); probes.Add(args[i]); end
    else if (a = '--fault') and (i < args.High) then begin Inc(i); faults.Add(args[i]); end
    else if (a = '--cart') and (i < args.High) then begin Inc(i); cartFile := args[i]; end
    else if (a = '--rom') and (i < args.High) then begin Inc(i); roms.Add(args[i]); end
    else if (a = '--at') and (i < args.High) then begin
      Inc(i);
      k := Pos(':', args[i]);
      atFrames.Add(StrToInt(Copy(args[i], 1, k - 1)));
      atSpecs.Add(Copy(args[i], k + 1, Length(args[i])));
    end else prg := a;
    Inc(i);
  end;
  if frames <= 0 then frames := 200;
  if machine = 'c64' then b := TC64.Create
  else if machine = 'zx48' then b := TZX48.Create
  else if machine = 'apple2p' then b := TApple2.Create
  else if machine = 'lc80' then b := TLc80.Create
  else begin
    WriteLn('unknown machine: ' + machine);
    exit;
  end;
  if Env('TESTZ80') <> '' then begin TestZ80(4000000); exit; end;
  if Env('DBGRNG') <> '' then begin
    b.Init;
    b.sim.Seed(1);
    WriteLn('seed ' + IntToHex(U32(b.sim.rngHi), 8) + IntToHex(U32(b.sim.rngLo), 8));
    for i := 0 to 4 do WriteLn('noise ' + FloatToStr(b.sim.Noise));
    WriteLn('rng ' + IntToHex(U32(b.sim.rngHi), 8) + IntToHex(U32(b.sim.rngLo), 8));
    i := 0;
    while i < 2000 do begin WriteLn('th ' + IntToStr(i) + ' ' + FloatToStr(Threshold(i))); i := i + 397; end;
    WriteLn('drift ' + FloatToStr(0 + (5.0 - 0) * Exp(-123456.0 / 300000.0)));
    exit;
  end;
  b.check := Env('C64HW_CHECK') <> '';
  b.Init;
  if (b is TApple2) and (Env('A2TRACE') <> '') then TApple2(b).trace_on := StrToInt(Env('A2TRACE'));
  if (b is TApple2) and (Env('A2LINE') <> '') then TApple2(b).dbg_line := StrToInt(Env('A2LINE'));
  if (Env('A2PINS') <> '') or (Env('PINLEVEL') <> '') then b.force_pins := True;
  for i := 0 to roms.High do begin b.Load(roms[i], ReadFileBytes(roms[i]), False, msg); WriteLn(msg); end;
  if harnessOpt and (b is TC64) then TC64(b).SetHarness(True);
  if cartFile <> '' then begin
    b.Load(cartFile, ReadFileBytes(cartFile), False, msg);
    WriteLn(msg);
  end;
  for i := 0 to faults.High do
    if not b.ApplyFaultSpec(faults[i]) then WriteLn('bad fault spec: ' + faults[i]);
  typ := Env('C64HW_TYPE');
  typed := 0;
  next_type := 0;
  ready := Trunc(b.ready_cycles / b.cycles_per_frame);
  t0 := NowMs;
  for f := 0 to frames - 1 do begin
    for i := 0 to atFrames.High do begin
      if atFrames[i] <> f then continue;
      if atSpecs[i] = 'repair' then b.ClearFaults
      else if not b.ApplyFaultSpec(atSpecs[i]) then WriteLn('bad fault spec: ' + atSpecs[i]);
    end;
    b.Run(b.cycles_per_frame);
    b.FrameUpdate(20000);
    if (prg <> '') and (f = ready) then begin
      b.Load(prg, ReadFileBytes(prg), True, msg);
      WriteLn(msg);
    end;
    if (typ <> '') and (f >= ready + 10) and (f >= next_type) and ((f mod 4) = 0) and (typed < Length(typ)) then begin
      Inc(typed);
      c := Ord(typ[typed]);
      if (typed < Length(typ)) and (Ord(typ[typed + 1]) = c) then next_type := f + 12 else next_type := f + 4;
      if c = 10 then c := b.HostKey('Enter')
      else if (c >= Ord('a')) and (c <= Ord('z')) then c := b.HostChar(c);
      b.KeyDown(c);
      b.KeyUp(c);
    end;
  end;
  secs := (NowMs - t0) / 1000;
  emulated := frames * b.cycles_per_frame / b.cpu_hz;
  WriteLn('emulated ' + Fix2(emulated) + ' s in ' + Fix2(secs) +
    ' s (' + IntToStr(Round(100 * emulated / secs)) + '% of real time)');
  WriteLn('CPU: ' + b.CpuRegs);
  if b.check then WriteLn('self-check errors: ' + IntToStr(b.check_errors));
  if text then begin
    lines := b.ScreenText;
    for i := 0 to lines.High do WriteLn('|' + lines[i]);
  end;
  if b is TApple2 then WriteLn('apple2: phi0 falls ' + IntToStr(TApple2(b).n_phi0) + ' sync pulses ' + IntToStr(TApple2(b).n_lines) + ' vsyncs ' + IntToStr(TApple2(b).n_vsync) + ' steps ' + FloatToStr(b.sim.cycle));
  if (b is TApple2) and (TApple2(b).dbg_line > 0) then WriteLn('dbg (LD194 /LDPS 7M A3-1 Qh TextVideo VIDEO): ' + Copy(TApple2(b).dbg, Length(TApple2(b).dbg) - 60 * 8 + 1, 60 * 8));
  if b is TApple2 then begin msg := ''; for i := 0 to TApple2(b).hist.High do msg := msg + TApple2(b).hist[i] + ' '; WriteLn(msg); end;
  for i := 0 to probes.High do begin
    k := Pos('.', probes[i]);
    c := b.PadPin(b.sim.FindPart(Copy(probes[i], 1, k - 1)), Copy(probes[i], k + 1, 9));
    if c < 0 then begin WriteLn('no pin ' + probes[i]); continue; end;
    n := b.sim.nets[b.sim.pins[c].root];
    WriteLn(probes[i] + ' (' + b.sim.pin_names[c] + ') net ' + b.sim.nets[b.sim.pins[c].net].name + ' root ' + n.name +
      ' level ' + IntToStr(n.level) + ' strength ' + IntToStr(n.strength) + ' read ' + IntToStr(b.sim.Read(c)) +
      ' fault ' + IntToStr(b.sim.pins[c].fault) + ' probe ' + IntToStr(b.NetState(b.sim.pins[c].net)) + ' (0 lo 1 hi 2 puls 3 float 4 fight)');
    for k := 0 to n.pins.High do
      if b.sim.pins[n.pins[k]].eff <> 0 then
        WriteLn('   driver ' + b.sim.parts[b.sim.pins[n.pins[k]].part].ref + '.' + IntToStr(b.sim.pins[n.pins[k]].number) +
          ' (' + b.sim.pin_names[n.pins[k]] + ') eff ' + IntToStr(b.sim.pins[n.pins[k]].eff) + ' drive ' + IntToStr(b.sim.pins[n.pins[k]].drive));
  end;
  if Env('DBG556') <> '' then begin
    var t := TTimer556State(b.sim.parts[b.sim.FindPart('U20')].state);
    WriteLn('556 reset_until ' + FloatToStr(t.reset_until) + ' last ' + FloatToStr(t.last_cycle) + ' cycle ' + FloatToStr(b.sim.cycle) +
      ' ncommit ' + IntToStr(b.sim.ncommit) + ' dead ' + BoolToStr(b.sim.parts[b.sim.FindPart('U20')].dead));
  end;
  if shot <> '' then begin
    var px := NewUInts(b.screen_w * b.screen_h);
    b.Monitor(px);
    SavePng(shot, px, b.screen_w, b.screen_h);
    WriteLn('screen saved to ' + shot);
  end;
  if nets then
    for i := 0 to b.sim.nnets - 1 do begin
      tot := b.st_hi[i] + b.st_lo[i] + b.st_float[i] + b.st_fight[i];
      n := b.sim.nets[i];
      if (b.st_fight[i] = 0) or (n.root <> i) then continue;
      WriteLn('contention ' + IntToStr((100 * b.st_fight[i]) div tot) + '% on net ' + n.name);
    end;
end;

end.
