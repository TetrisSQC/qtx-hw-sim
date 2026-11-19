unit hwsim.m6526;

{ MOS 6526 CIA, port of floooh's chips/m6526.h (zlib license, Copyright (c)
  2018 Andre Weissflog) with the c64-hw-sim additions: time of day clock,
  serial port, /PC.

  Pins (bits of an Integer): RS0-RS3 0..3, CS 4, FLAG 5, PC 6, SP 7, TOD 8,
  CNT 9, D0-D7 16..23, RW 24, IRQ 26. The C code has CS..CNT at bits 40..45
  and the ports at 48..63: here the port lines are the arguments pa/pb of
  Tick, and the port outputs are pa_pins/pb_pins. }

interface

const
  M6526_RS = $0F;
  M6526_CS = 1 shl 4;
  M6526_FLAG = 1 shl 5;
  M6526_PC = 1 shl 6;
  M6526_SP = 1 shl 7;
  M6526_TOD = 1 shl 8;
  M6526_CNT = 1 shl 9;
  M6526_RW = 1 shl 24;
  M6526_IRQ = 1 shl 26;

  M6526_REG_PRA = 0; M6526_REG_PRB = 1; M6526_REG_DDRA = 2; M6526_REG_DDRB = 3;
  M6526_REG_TALO = 4; M6526_REG_TAHI = 5; M6526_REG_TBLO = 6; M6526_REG_TBHI = 7;
  M6526_REG_TOD10TH = 8; M6526_REG_TODSEC = 9; M6526_REG_TODMIN = 10; M6526_REG_TODHR = 11;
  M6526_REG_SDR = 12; M6526_REG_ICR = 13; M6526_REG_CRA = 14; M6526_REG_CRB = 15;

type
  TM6526Port = class
  public
    reg, ddr, inp, pins: Integer;
  end;

  TM6526Timer = class
  public
    latch, counter, cr: Integer;
    t_bit, t_out: Boolean;
    pip: Integer;
  end;

  TM6526 = class
  private
    procedure InitPort(p: TM6526Port);
    procedure InitTimer(t: TM6526Timer);
    procedure InitInterrupt;
    function MergePB67(data: Integer): Integer;
    procedure WriteIcr(data: Integer);
    function ReadIcr: Integer;
    procedure TodCheckAlarm;
    procedure TickTod(apins: Integer);
    function ReadTod(addr: Integer): Integer;
    procedure WriteTod(addr, data: Integer);
    procedure TickSp(apins: Integer);
    function UpdateIrq(apins: Integer): Integer;
    procedure TickTimer(t: TM6526Timer);
    procedure TickPipeline;
    procedure WriteCr(t: TM6526Timer; data: Integer);
    function RegRead(addr: Integer): Integer;
    procedure RegWrite(addr, data: Integer);
  public
    pa, pb: TM6526Port;
    ta, tb: TM6526Timer;
    { interrupt }
    imr, imr1, icr, int_pip: Integer;
    int_flag: Boolean;
    { time of day }
    tod_time, tod_alarm, tod_latch: array[0..3] of Integer;
    tod_latched, tod_stopped, tod_match, tod_irq: Boolean;
    tod_prescaler: Integer;
    tod_pin: Boolean;
    { serial port }
    sp_sdr, sp_shift, sp_bits: Integer;
    sp_busy, sp_loaded, sp_cnt_out, sp_sp_out, sp_cnt_in, sp_irq: Boolean;
    pc_low, pc_next: Boolean;
    pins: Integer;
    constructor Create;
    procedure Init;
    procedure Reset;
    { One clock. pa/pb: the levels of the port lines. }
    function Tick(apins, apa, apb: Integer): Integer;
  end;

function M6526_GET_DATA(p: Integer): Integer;
function M6526_SET_DATA(p, d: Integer): Integer;

implementation

const
  PIP_TIMER_COUNT = 0;
  PIP_TIMER_ONESHOT = 8;
  PIP_TIMER_LOAD = 16;
  PIP_IRQ = 0;
  PIP_READ_ICR = 8;

function M6526_GET_DATA(p: Integer): Integer;
begin
  Result := (p shr 16) and $FF;
end;

function M6526_SET_DATA(p, d: Integer): Integer;
begin
  Result := (p and not $FF0000) or ((d and $FF) shl 16);
end;

function PipTest(pip, offset, pos: Integer): Boolean;
begin
  Result := (pip and (1 shl (offset + pos))) <> 0;
end;

constructor TM6526.Create;
begin
  inherited Create;
  pa := TM6526Port.Create;
  pb := TM6526Port.Create;
  ta := TM6526Timer.Create;
  tb := TM6526Timer.Create;
end;

procedure TM6526.InitPort(p: TM6526Port);
begin
  p.reg := 0; p.ddr := 0; p.inp := 0; p.pins := 0;
end;

procedure TM6526.InitTimer(t: TM6526Timer);
begin
  t.latch := $FFFF;
  t.counter := 0;
  t.cr := 0;
  t.t_bit := False;
  t.t_out := False;
  t.pip := 0;
end;

procedure TM6526.InitInterrupt;
begin
  imr := 0; imr1 := 0; icr := 0; int_pip := 0;
  int_flag := False;
end;

procedure TM6526.Init;
begin
  Reset;
end;

procedure TM6526.Reset;
var i: Integer;
begin
  InitPort(pa);
  InitPort(pb);
  InitTimer(ta);
  InitTimer(tb);
  InitInterrupt;
  for i := 0 to 3 do begin tod_time[i] := 0; tod_alarm[i] := 0; tod_latch[i] := 0; end;
  tod_latched := False; tod_stopped := False; tod_match := False; tod_irq := False;
  tod_prescaler := 0; tod_pin := False;
  sp_sdr := 0; sp_shift := 0; sp_bits := 0;
  sp_busy := False; sp_loaded := False; sp_irq := False;
  sp_cnt_out := True; sp_sp_out := True;
  sp_cnt_in := True;
  pc_low := False; pc_next := False;
  pins := 0;
end;

{--- port ---}

function TM6526.MergePB67(data: Integer): Integer;
begin
  if (ta.cr and 2) <> 0 then begin
    data := data and not (1 shl 6);
    if (ta.cr and 4) <> 0 then begin
      if ta.t_bit then data := data or (1 shl 6);
    end else begin
      if ta.t_out then data := data or (1 shl 6);
    end;
  end;
  if (tb.cr and 2) <> 0 then begin
    data := data and not (1 shl 7);
    if (tb.cr and 4) <> 0 then begin
      if tb.t_bit then data := data or (1 shl 7);
    end else begin
      if tb.t_out then data := data or (1 shl 7);
    end;
  end;
  Result := data and $FF;
end;

{--- interrupt ---}

procedure TM6526.WriteIcr(data: Integer);
begin
  if (data and $80) <> 0 then imr1 := imr1 or (data and $1F)
  else imr1 := imr1 and not (data and $1F);
  if (icr and imr1) <> 0 then int_pip := int_pip or (1 shl (PIP_IRQ + 1));
end;

function TM6526.ReadIcr: Integer;
begin
  Result := icr;
  icr := 0;
  int_pip := int_pip and not ($FF shl PIP_IRQ);
  int_pip := int_pip or (1 shl PIP_READ_ICR);
end;

{--- time of day ---}

function BcdInc(v: Integer): Integer;
begin
  if (v and $0F) = 9 then Result := ((v and $F0) + $10) and $FF
  else Result := (v + 1) and $FF;
end;

procedure TM6526.TodCheckAlarm;
var match: Boolean;
begin
  match := (tod_time[0] = tod_alarm[0]) and (tod_time[1] = tod_alarm[1]) and
           (tod_time[2] = tod_alarm[2]) and (tod_time[3] = tod_alarm[3]);
  if match and not tod_match then tod_irq := True;
  tod_match := match;
end;

procedure TM6526.TickTod(apins: Integer);
var
  pin, rise: Boolean;
  lim, h, pm: Integer;
begin
  pin := (apins and M6526_TOD) <> 0;
  rise := pin and not tod_pin;
  tod_pin := pin;
  if (not rise) or tod_stopped then exit;
  tod_prescaler := (tod_prescaler + 1) and $FF;
  if (ta.cr and $80) <> 0 then lim := 5 else lim := 6;
  if tod_prescaler < lim then exit;
  tod_prescaler := 0;
  if tod_time[0] <> 9 then
    tod_time[0] := (tod_time[0] + 1) and $0F
  else begin
    tod_time[0] := 0;
    if tod_time[1] <> $59 then
      tod_time[1] := BcdInc(tod_time[1]) and $7F
    else begin
      tod_time[1] := 0;
      if tod_time[2] <> $59 then
        tod_time[2] := BcdInc(tod_time[2]) and $7F
      else begin
        tod_time[2] := 0;
        h := tod_time[3] and $1F;
        pm := tod_time[3] and $80;
        if h = $11 then begin
          h := $12;
          pm := pm xor $80;
        end else if h = $12 then
          h := $01
        else
          h := BcdInc(h) and $1F;
        tod_time[3] := pm or h;
      end;
    end;
  end;
  TodCheckAlarm;
end;

function TM6526.ReadTod(addr: Integer): Integer;
var i: Integer;
begin
  i := addr - M6526_REG_TOD10TH;
  if (i = 3) and not tod_latched then begin
    tod_latch[0] := tod_time[0]; tod_latch[1] := tod_time[1];
    tod_latch[2] := tod_time[2]; tod_latch[3] := tod_time[3];
    tod_latched := True;
  end;
  if tod_latched then Result := tod_latch[i] else Result := tod_time[i];
  if i = 0 then tod_latched := False;
end;

const
  TOD_MASK: array[0..3] of Integer = [$0F, $7F, $7F, $9F];

procedure TM6526.WriteTod(addr, data: Integer);
var i: Integer;
begin
  i := addr - M6526_REG_TOD10TH;
  data := data and TOD_MASK[i];
  if (tb.cr and $80) <> 0 then
    tod_alarm[i] := data
  else begin
    if i = 3 then begin
      if (data and $1F) = $12 then data := data xor $80;
      tod_stopped := True;
    end else if i = 0 then begin
      tod_stopped := False;
      tod_prescaler := 0;
    end;
    tod_time[i] := data;
  end;
  TodCheckAlarm;
end;

{--- serial port ---}

procedure TM6526.TickSp(apins: Integer);
var cnt: Boolean;
begin
  cnt := (apins and M6526_CNT) <> 0;
  if (ta.cr and $40) <> 0 then begin
    if (not sp_busy) and sp_loaded then begin
      sp_shift := sp_sdr;
      sp_loaded := False;
      sp_busy := True;
      sp_bits := 0;
    end;
    if sp_busy and ta.t_out then begin
      if sp_cnt_out then begin
        sp_cnt_out := False;
        sp_sp_out := (sp_shift and $80) <> 0;
        sp_shift := (sp_shift shl 1) and $FF;
      end else begin
        sp_cnt_out := True;
        sp_bits := (sp_bits + 1) and $FF;
        if sp_bits = 8 then begin
          sp_irq := True;
          sp_busy := False;
        end;
      end;
    end;
  end else if cnt and not sp_cnt_in then begin
    sp_shift := ((sp_shift shl 1) or Ord((apins and M6526_SP) <> 0)) and $FF;
    sp_bits := (sp_bits + 1) and $FF;
    if sp_bits = 8 then begin
      sp_sdr := sp_shift;
      sp_bits := 0;
      sp_irq := True;
    end;
  end;
  sp_cnt_in := cnt;
end;

function TM6526.UpdateIrq(apins: Integer): Integer;
begin
  if ta.t_out then icr := icr or 1;
  if tb.t_out then
    if not PipTest(int_pip, PIP_READ_ICR, 0) then icr := icr or 2;
  if ((apins and M6526_FLAG) <> 0) and not int_flag then icr := icr or $10;
  int_flag := (apins and M6526_FLAG) <> 0;
  if tod_irq then begin
    icr := icr or 4;
    tod_irq := False;
  end;
  if sp_irq then begin
    icr := icr or 8;
    sp_irq := False;
  end;
  if PipTest(int_pip, PIP_IRQ, 0) then icr := icr or $80;
  if (icr and $80) <> 0 then Result := apins or M6526_IRQ
  else Result := apins and not M6526_IRQ;
end;

{--- timers ---}

procedure TM6526.TickTimer(t: TM6526Timer);
begin
  if PipTest(t.pip, PIP_TIMER_COUNT, 0) then t.counter := (t.counter - 1) and $FFFF;
  t.t_out := (t.counter = 0) and PipTest(t.pip, PIP_TIMER_COUNT, 1);
  if t.t_out then begin
    t.t_bit := not t.t_bit;
    if ((t.cr and 8) <> 0) or PipTest(t.pip, PIP_TIMER_ONESHOT, 0) then t.cr := t.cr and not 1;
    t.pip := t.pip or (1 shl PIP_TIMER_LOAD);
  end;
  if PipTest(t.pip, PIP_TIMER_LOAD, 0) then begin
    t.counter := t.latch;
    t.pip := t.pip and not (1 shl (PIP_TIMER_COUNT + 1));
  end;
end;

procedure TM6526.TickPipeline;
var tb_active: Boolean;
begin
  { timer A counter pipeline }
  if (ta.cr and $20) = 0 then ta.pip := ta.pip or (1 shl (PIP_TIMER_COUNT + 2));
  if (ta.cr and 1) = 0 then ta.pip := ta.pip and not (1 shl (PIP_TIMER_COUNT + 2));
  if (ta.cr and $10) <> 0 then begin
    ta.pip := ta.pip or (1 shl (PIP_TIMER_LOAD + 1));
    ta.cr := ta.cr and not $10;
  end;
  if (ta.cr and 8) <> 0 then ta.pip := ta.pip or (1 shl (PIP_TIMER_ONESHOT + 1));
  { timer B counter pipeline }
  tb_active := False;
  case tb.cr and $60 of
    $00: tb_active := True;
    $40: tb_active := ta.t_out;
    $20: tb_active := False;
    $60: tb_active := ta.t_out;
  end;
  if tb_active then tb.pip := tb.pip or (1 shl (PIP_TIMER_COUNT + 2))
  else tb.pip := tb.pip and not (1 shl (PIP_TIMER_COUNT + 2));
  if (tb.cr and 1) = 0 then tb.pip := tb.pip and not (1 shl (PIP_TIMER_COUNT + 2));
  if (tb.cr and $10) <> 0 then begin
    tb.pip := tb.pip or (1 shl (PIP_TIMER_LOAD + 1));
    tb.cr := tb.cr and not $10;
  end;
  if (tb.cr and 8) <> 0 then tb.pip := tb.pip or (1 shl (PIP_TIMER_ONESHOT + 1));
  { interrupt pipeline }
  if (icr and imr) <> 0 then int_pip := int_pip or (1 shl (PIP_IRQ + 1));
  imr := imr1;
  { tick pipelines forward }
  ta.pip := (ta.pip shr 1) and $7F7F7F7F;
  tb.pip := (tb.pip shr 1) and $7F7F7F7F;
  int_pip := (int_pip shr 1) and $7F7F7F7F;
end;

procedure TM6526.WriteCr(t: TM6526Timer; data: Integer);
begin
  if ((t.cr and 1) = 0) and ((data and 1) <> 0) then t.t_bit := True;
  t.cr := data;
end;

function TM6526.RegRead(addr: Integer): Integer;
begin
  Result := $FF;
  case addr of
    M6526_REG_PRA: Result := pa.inp;
    M6526_REG_PRB: Result := MergePB67(pb.inp);
    M6526_REG_DDRA: Result := pa.ddr;
    M6526_REG_DDRB: Result := pb.ddr;
    M6526_REG_TALO: Result := ta.counter and $FF;
    M6526_REG_TAHI: Result := ta.counter shr 8;
    M6526_REG_TBLO: Result := tb.counter and $FF;
    M6526_REG_TBHI: Result := tb.counter shr 8;
    M6526_REG_ICR: Result := ReadIcr;
    M6526_REG_CRA: Result := ta.cr and not $10;
    M6526_REG_CRB: Result := tb.cr and not $10;
    M6526_REG_TOD10TH, M6526_REG_TODSEC, M6526_REG_TODMIN, M6526_REG_TODHR: Result := ReadTod(addr);
    M6526_REG_SDR: Result := sp_sdr;
  end;
end;

procedure TM6526.RegWrite(addr, data: Integer);
begin
  case addr of
    M6526_REG_PRA: pa.reg := data;
    M6526_REG_PRB: pb.reg := data;
    M6526_REG_DDRA: pa.ddr := data;
    M6526_REG_DDRB: pb.ddr := data;
    M6526_REG_TALO: ta.latch := (ta.latch and $FF00) or data;
    M6526_REG_TAHI: begin
      ta.latch := (data shl 8) or (ta.latch and $00FF);
      if (ta.cr and 1) = 0 then ta.pip := ta.pip or (1 shl (PIP_TIMER_LOAD + 1));
    end;
    M6526_REG_TBLO: tb.latch := (tb.latch and $FF00) or data;
    M6526_REG_TBHI: begin
      tb.latch := (data shl 8) or (tb.latch and $00FF);
      if (tb.cr and 1) = 0 then tb.pip := tb.pip or (1 shl (PIP_TIMER_LOAD + 1));
    end;
    M6526_REG_ICR: WriteIcr(data);
    M6526_REG_CRA: begin
      if ((ta.cr xor data) and $40) <> 0 then begin
        sp_busy := False;
        sp_loaded := False;
        sp_bits := 0;
        sp_cnt_out := True;
      end;
      WriteCr(ta, data);
    end;
    M6526_REG_CRB: WriteCr(tb, data);
    M6526_REG_TOD10TH, M6526_REG_TODSEC, M6526_REG_TODMIN, M6526_REG_TODHR: WriteTod(addr, data);
    M6526_REG_SDR: begin
      sp_sdr := data;
      if (ta.cr and $40) <> 0 then sp_loaded := True;
    end;
  end;
end;

function TM6526.Tick(apins, apa, apb: Integer): Integer;
var
  p, addr: Integer;
begin
  { c64-hw-sim: /PC is low for one cycle following an access to port B }
  pc_low := pc_next;
  pc_next := False;
  p := apins;
  pa.inp := apa and $FF;
  pb.inp := apb and $FF;
  TickTimer(ta);
  TickTimer(tb);
  TickTod(p);
  TickSp(p);
  p := UpdateIrq(p);
  pa.pins := (pa.reg or (pa.inp and not pa.ddr)) and $FF;
  pb.pins := MergePB67(pb.reg or (pb.inp and not pb.ddr));
  TickPipeline;
  if (p and M6526_CS) <> 0 then begin
    addr := p and M6526_RS;
    if addr = M6526_REG_PRB then pc_next := True;
    if (p and M6526_RW) <> 0 then
      p := M6526_SET_DATA(p, RegRead(addr))
    else
      RegWrite(addr, M6526_GET_DATA(p));
  end;
  pins := p;
  Result := p;
end;

end.
