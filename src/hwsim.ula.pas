unit hwsim.ula;

{ The Ferranti ULA 6C001 (Mark 7) of the ZX Spectrum 48K, at its pins
  (port of ula.h / ula.c).

  The ULA makes the video picture, fetching the screen from the lower 16K
  of RAM (the 4116s) through its own address and RAS/CAS pins, makes the
  CPU clock and stops it (contention) when the CPU wants that RAM while the
  screen is being fetched, decodes the CPU's accesses to the lower 16K and
  the ROM, and is the I/O port 0xFE: keyboard, border, MIC, EAR.

  The logic is that of the 6C001 as recovered from its die by the
  emu-russia/ula project (CC0); the timing of every pin was taken from a
  gate level simulation of that netlist, written here in half periods of
  the 14 MHz oscillator.

  Time: a pixel tick (7 MHz) is four half periods; C counts the ticks of a
  line (0..447), V the lines of a frame (0..311). The screen is fetched
  during C 0..255 of lines 0..191; the interrupt is at line 248.

  The screen fetch: in every block of 16 ticks, ticks 8..15 fetch two
  character cells, bitmap and attribute of cell 2k at ticks 8..11, of cell
  2k+1 at 12..15, each pair one /RAS cycle with two /CAS cycles:

    tick       8      9      10     11     (half periods 0..3 of each)
    /RAS       -___   ____   ____   _---
    /CAS       --__   ___-   -___   ___-
    A0-A6      RR CC  CCCC   CAAA   ARRR

  The CPU clock is high for 5 half periods and lowbits for 3. Contention: when
  the clock is about to fall in ticks 4..15 of a block on a screen line,
  and the CPU puts out an address of 0x4000-0x7FFF without /MREQ yet or
  starts an I/O cycle, the ULA keeps the clock high until the block ends. }

interface

uses
  hwsim.util, hwsim.sim, hwsim.parts;

const
  ULA_TICKS_PER_LINE = 448;
  ULA_LINES = 312;
  ULA_SCREEN_W = 352;
  ULA_SCREEN_H = 296;

type
  TUlaState = class
  public
    c, v, half: Integer;
    frames: Integer;
    data, attr: Integer;
    line_bitmap, line_attr: array[0..31] of Integer;
    border, border_ao, border_shown: Integer;
    mic, speaker: Boolean;
    mreqt2, ioreqt2: Boolean;
    cpuclk, ras, cas, ras_oe, ae, we, romcs, irq: Boolean;
    addr: Integer;
    dlow: Integer;
    cpu_cas, cpu_cas_req: Boolean;
    running: Boolean;
    ear_in: Boolean;
    fb: TUInts;
    pix: Integer;
  end;

var
  PT_ULA: TPartType;

{ One half period of the 14 MHz oscillator (8 per CPU T-state). }
procedure ula_step(s: TSim; p: TPart);
function ula_color(index: Integer; bright: Boolean): Integer;

implementation

const
  P_CAS = 1; P_WR = 2; P_RD = 3; P_WE = 4; P_A0 = 5; P_INT = 12; P_D0 = 18; P_KB0 = 19; P_KB1 = 20;
  P_D1 = 21; P_D2 = 22; P_KB2 = 23; P_KB3 = 24; P_D3 = 25; P_KB4 = 26; P_D4 = 27; P_SOUND = 28;
  P_D5 = 29; P_D6 = 30; P_D7 = 31; P_PHICPU = 32; P_IOREQ = 33; P_ROMCS = 34; P_RAS = 35; P_A14 = 36;
  P_A15 = 37; P_MREQ = 38; P_OSC = 39;

  ADDR_PINS: array[0..6] of Integer = [5, 6, 7, 8, 9, 10, 11];
  DATA_PINS: array[0..7] of Integer = [P_D0, P_D1, P_D2, P_D3, P_D4, P_D5, P_D6, P_D7];
  KB_PINS: array[0..4] of Integer = [P_KB0, P_KB1, P_KB2, P_KB3, P_KB4];

  GK_ULA_DATA = GK_MACHINE + 50;

function bitmap_addr(v, k: Integer): Integer;
begin
  Result := ((v and $C0) shl 5) or ((v and 7) shl 8) or ((v and $38) shl 2) or k;
end;

function attr_addr(v, k: Integer): Integer;
begin
  Result := $1800 or ((v and $F8) shl 2) or k;
end;

function on_screen(u: TUlaState): Boolean;
begin
  Result := (u.c < 256) and (u.v < 192);
end;

function NewUlaState: TObject;
begin
  Result := TUlaState.Create;
end;

procedure ula_eval(s: TSim; p: TPart);
var
  u: TUlaState;
  mreq, a14, a15, rd, wr, iorq, ram16, ras, ras_oe, port_rd: Boolean;
  lowbits, i: Integer;
begin
  u := TUlaState(p.state);
  mreq := s.PinIn(p, P_MREQ) = 0;
  a14 := s.PinIn(p, P_A14) <> 0;
  a15 := s.PinIn(p, P_A15) <> 0;
  rd := s.PinIn(p, P_RD) = 0;
  wr := s.PinIn(p, P_WR) = 0;
  iorq := s.PinIn(p, P_IOREQ) = 0;
  ram16 := a14 and not a15 and mreq;          { the CPU wants 0x4000-0x7FFF }
  ras := u.ras or ram16;
  { /RAS is let go when the CPU accesses the ROM in the border. }
  ras_oe := not ((not a15) and (not a14) and (not on_screen(u)) and mreq);
  if ras_oe then s.PinOut(p, P_RAS, DRV_BIT(not ras)) else s.PinOut(p, P_RAS, DRV_Z);
  s.PinOut(p, P_CAS, DRV_BIT(not (u.cas or u.cpu_cas)));
  s.PinOut(p, P_WE, DRV_BIT(not (ram16 and wr)));
  s.PinOut(p, P_ROMCS, DRV_BIT(a15 or a14));
  s.PinOut(p, P_INT, DRV_BIT(not u.irq));
  { The CPU clock: an inverting open collector output. }
  if u.cpuclk then s.PinOut(p, P_PHICPU, DRV_LO) else s.PinOut(p, P_PHICPU, DRV_Z);
  part_drive_bus(s, p, ADDR_PINS, 7, u.addr, u.ae);
  s.PinOut(p, P_SOUND, DRV_BIT(u.speaker));
  { Port 0xFE read: keyboard on D0-D4, EAR on D6 (open collector). }
  port_rd := iorq and rd and (s.PinIn(p, P_A0) = 0) and u.ioreqt2;
  lowbits := 0;
  if port_rd then begin
    for i := 0 to 4 do
      if s.PinIn(p, KB_PINS[i]) = 0 then lowbits := lowbits or (1 shl i);
    if (not u.ear_in) and (s.PinIn(p, P_SOUND) = 0) then lowbits := lowbits or $40;
  end;
  for i := 0 to 7 do
    if ((lowbits shr i) and 1) <> 0 then s.PinOut(p, DATA_PINS[i], DRV_LO)
    else s.PinOut(p, DATA_PINS[i], DRV_Z);
  u.dlow := lowbits;
end;

function ula_color(index: Integer; bright: Boolean): Integer;
var lvl, r, g, b: Integer;
begin
  if bright then lvl := $FF else lvl := $D7;
  if (index and 2) <> 0 then r := lvl else r := 0;
  if (index and 4) <> 0 then g := lvl else g := 0;
  if (index and 1) <> 0 then b := lvl else b := 0;
  Result := U32($FF000000 or (b shl 16) or (g shl 8) or r);
end;

procedure put_pixel(u: TUlaState);
var
  px, k, a, b, x, y: Integer;
  ink: Boolean;
begin
  if (u.c and 7) = 0 then u.border_shown := u.border_ao;
  if (u.v < 192) and (u.c >= 16) and (u.c < 272) then begin
    px := u.c - 16;
    k := px shr 3;
    a := u.line_attr[k];
    b := u.line_bitmap[k];
    ink := ((b shr (7 - (px and 7))) and 1) <> 0;
    if ((a and $80) <> 0) and (((u.frames shr 4) and 1) <> 0) then ink := not ink;    { FLASH }
    if ink then u.pix := (a and 7) or ((a and $40) shr 3)
    else u.pix := ((a shr 3) and 7) or ((a and $40) shr 3);
  end else
    u.pix := u.border_shown;
  x := (u.c - 16 + 48 + ULA_TICKS_PER_LINE) mod ULA_TICKS_PER_LINE;
  if u.v < 248 then y := u.v + 48 else y := u.v - 264;
  if (u.fb = nil) or (x >= ULA_SCREEN_W) or (y < 0) or (y >= ULA_SCREEN_H) then exit;
  u.fb[y * ULA_SCREEN_W + x] := ula_color(u.pix and 7, (u.pix and 8) <> 0);
end;

procedure ula_step(s: TSim; p: TPart);
var
  u: TUlaState;
  c, h, blk, k, sub, row, bcol, acol, addr, d: Integer;
  screen, mreq, iorq, rd, wr, a14, a15, ras, cas, ae: Boolean;
  border, c23, base, w362, mem, io, cpuclk, irq, cpu_cas: Boolean;
begin
  u := TUlaState(p.state);
  if p.dead or not u.running then exit;
  { Time. }
  u.half := (u.half + 1) and 3;
  if u.half = 0 then begin
    Inc(u.c);
    if u.c = ULA_TICKS_PER_LINE then begin
      u.c := 0;
      Inc(u.v);
      if u.v = ULA_LINES then begin
        u.v := 0;
        u.frames := u.frames + 1;
      end;
    end;
    put_pixel(u);
  end;
  c := u.c;
  h := u.half;
  blk := c and 15;
  screen := on_screen(u);
  { Inputs, as the board settled them. }
  mreq := s.PinIn(p, P_MREQ) = 0;
  iorq := s.PinIn(p, P_IOREQ) = 0;
  rd := s.PinIn(p, P_RD) = 0;
  wr := s.PinIn(p, P_WR) = 0;
  a14 := s.PinIn(p, P_A14) <> 0;
  a15 := s.PinIn(p, P_A15) <> 0;
  { The screen fetch. }
  ras := False;
  cas := False;
  ae := False;
  addr := u.addr;
  if (blk = 0) and (h = 0) and (c > 0) and (c <= 256) and (u.v < 192) then ae := True;
  if screen and ((blk >= 8) or ((blk = 7) and (h >= 1))) then begin
    k := ((c shr 4) shl 1) or Ord(blk >= 12);
    sub := blk and 3;
    row := bitmap_addr(u.v, k) and $7F;
    bcol := bitmap_addr(u.v, k) shr 7;
    acol := attr_addr(u.v, k) shr 7;
    ae := True;
    if blk = 7 then
      addr := row
    else begin
      ras := ((sub = 0) and (h >= 1)) or (sub = 1) or (sub = 2) or ((sub = 3) and (h = 0));
      cas := ((sub = 0) and (h >= 2)) or ((sub = 1) and (h <= 2)) or ((sub = 2) and (h >= 1)) or ((sub = 3) and (h <= 2));
      if sub = 0 then begin
        if h <= 1 then addr := row else addr := bcol;
      end else if sub = 1 then
        addr := bcol
      else if sub = 2 then begin
        if h = 0 then addr := bcol else addr := acol;
      end else if h = 0 then
        addr := acol
      else
        addr := bitmap_addr(u.v, k xor 1) and $7F;   { the row of the other cell of the pair }
      addr := addr and $FF;
      { The latches close just before /CAS rises. }
      if (h = 3) and ((sub = 1) or (sub = 3)) then begin
        d := part_read_bus(s, p, GK_ULA_DATA, DATA_PINS, 8);
        if sub = 1 then u.data := d
        else begin
          u.attr := d;
          u.line_bitmap[k] := u.data;
          u.line_attr[k] := u.attr;
        end;
      end;
    end;
  end;
  { Port 0xFE write: the latch is open while the CPU writes. }
  if iorq and wr and (s.PinIn(p, P_A0) = 0) and u.ioreqt2 then begin
    d := part_read_bus(s, p, GK_ULA_DATA, DATA_PINS, 8);
    u.border := d and 7;
    u.mic := ((d shr 3) and 1) <> 0;
    if u.speaker <> (((d shr 4) and 1) <> 0) then begin
      u.speaker := ((d shr 4) and 1) <> 0;
      s.MarkDirty(p.index);
    end;
  end;
  if (c and 7) = 5 then u.border_ao := u.border;
  { The CPU clock and contention (gates g383..g405 of the netlist). }
  border := not screen;
  c23 := (c and 12) <> 0;
  base := ((c and 1) <> 0) or (h = 0);
  w362 := (not u.ioreqt2) and (not border) and u.cpuclk and (not u.mreqt2);
  mem := (a14 or iorq) and (iorq or not a15) and c23 and w362;
  io := (not border) and u.cpuclk and iorq and c23 and not u.ioreqt2;
  cpuclk := base or mem or io;
  if not cpuclk then begin
    u.mreqt2 := mreq;
    u.ioreqt2 := iorq;
  end;
  { /INT: 64 ticks at the start of line 248, half a period behind. }
  irq := (u.v = 248) and (((c = 0) and (h >= 1)) or ((c >= 1) and (c < 64)) or ((c = 64) and (h = 0)));
  cpu_cas := u.cpu_cas_req;
  u.cpu_cas_req := (rd or wr) and a14 and (not a15) and mreq;
  if (cpuclk xor u.cpuclk) or (ras xor u.ras) or (cas xor u.cas) or (ae xor u.ae) or (addr <> u.addr) or
     (irq xor u.irq) or (cpu_cas xor u.cpu_cas) then begin
    u.cpuclk := cpuclk;
    u.ras := ras;
    u.cas := cas;
    u.ae := ae;
    u.addr := addr;
    u.irq := irq;
    u.cpu_cas := cpu_cas;
    s.MarkDirty(p.index);
  end;
end;

procedure ula_reset(s: TSim; p: TPart);
var
  u, n: TUlaState;
begin
  u := TUlaState(p.state);
  n := TUlaState.Create;
  n.fb := u.fb;
  n.ear_in := u.ear_in;
  n.running := u.running;
  n.cpuclk := True;
  p.state := n;
end;

initialization
  PT_ULA := TPartType.Create('ULA 6C001',
    '/CAS:O /WR:T /RD:T /WE:O A0:X A1:O A2:O A3:O A4:O A5:O A6:O /INT:O VCC2:P VCC1:P U:O V:O /Y:O ' +
    'D0:B KB0:T KB1:T D1:B D2:B KB2:T KB3:T D3:B KB4:T D4:B SOUND:X D5:m D6:B D7:m /PHICPU:O ' +
    '/IOREQ:T /ROMCS:O /RAS:O A14:T A15:T /MREQ:T OSC:m GND:P',
    ula_eval, nil, ula_reset, NewUlaState, TECH_TTL);
end.
