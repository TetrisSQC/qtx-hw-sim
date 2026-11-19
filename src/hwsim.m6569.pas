unit hwsim.m6569;

{ MOS 6569 VIC-II (PAL), port of floooh's chips/m6569.h (zlib license,
  Copyright (c) 2019 Andre Weissflog), with the c64-hw-sim changes:
  the tick is split in TickFetch (memory fetches, BA, IRQ) and TickFinish
  (register write, border, pixels, register read).

  Pins (bits of an Integer): A0-A13 0..13, D0-D7 16..23, RW 24, IRQ 26,
  BA 28, AEC 29, CS 30 (bit 40 in the C code). }

interface

uses
  hwsim.util;

const
  M6569_HTOTAL = 63;
  M6569_VTOTAL = 312;

  M6569_RW = 1 shl 24;
  M6569_IRQ = 1 shl 26;
  M6569_BA = 1 shl 28;
  M6569_AEC = 1 shl 29;
  M6569_CS = 1 shl 30;
  M6569_REG_MASK = 63;

  M6569_CTRL1_RST8 = 1 shl 7;
  M6569_CTRL1_ECM = 1 shl 6;
  M6569_CTRL1_BMM = 1 shl 5;
  M6569_CTRL1_DEN = 1 shl 4;
  M6569_CTRL1_RSEL = 1 shl 3;
  M6569_CTRL2_RES = 1 shl 5;
  M6569_CTRL2_MCM = 1 shl 4;
  M6569_CTRL2_CSEL = 1 shl 3;
  M6569_CTRL2_XSCROLL = 7;
  M6569_INT_IRQ = 1 shl 7;
  M6569_INT_ILP = 1 shl 3;
  M6569_INT_IMMC = 1 shl 2;
  M6569_INT_IMBC = 1 shl 1;
  M6569_INT_IRST = 1;

  { Register indices. }
  R_MX8 = $10; R_CTRL1 = $11; R_RASTER = $12; R_ME = $15; R_CTRL2 = $16; R_MYE = $17;
  R_MEMPTRS = $18; R_INTLATCH = $19; R_INTMASK = $1A; R_MDP = $1B; R_MMC = $1C; R_MXE = $1D;
  R_MCM = $1E; R_MCD = $1F; R_EC = $20;

type
  TM6569FetchFn = function(addr: Integer; user: TObject): Integer;

  TM6569 = class
  private
    procedure IoUpdateIrqLine(ctrl_1, rast: Integer);
    procedure IoUpdateMemoryUnit(mem_ptrs, ctrl_1: Integer);
    procedure IoUpdateBorderRsel(ctrl_1: Integer);
    procedure IoUpdateBorderCsel(ctrl_2: Integer);
    procedure IoUpdateGunitMode(ctrl_1, ctrl_2: Integer);
    procedure IoUpdateSunit(i, mx, my, mx8, mxe, mye: Integer);
    function RegRead(pins: Integer): Integer;
    procedure RegWrite(pins: Integer);
    procedure ResetBorderUnit;
    procedure GunitRewind(xscroll: Integer);
    procedure GunitTick(gd: Integer);
    function GunitDecode(mode: Integer): Integer;
    procedure SunitStart;
    procedure SunitUpdateMcDispEnable;
    procedure SunitUpdateMcbase;
    function SunitDecode(hpos: Integer): Integer;
    procedure DecodePixels(x, y, gd, hpos: Integer);
    procedure RsNextRasterline;
    procedure RsUpdateBadline;
    procedure RsUpdateDisplayState;
    procedure RsRewindVcVmliRc;
    procedure CrtNextCrtline;
    procedure BunitLeft(hpos: Integer);
    procedure BunitEnd;
    procedure CAccess;
    function IAccess: Integer;
    function GIAccess: Integer;
    procedure PAccess(i: Integer);
    procedure SAccess(i: Integer);
    function SIAccess(i: Integer): Integer;
    function DmaBA(i, pins: Integer): Integer;
    function DmaAEC(i, pins: Integer): Integer;
  public
    regs: array[0..63] of Integer;
    { crt }
    crt_x, crt_y: Integer;
    vis_x0, vis_y0, vis_x1, vis_y1, vis_w, vis_h: Integer;
    { border unit }
    brd_left, brd_right, brd_top, brd_bottom: Integer;
    brd_main, brd_vert: Boolean;
    brd_bc: Integer;
    { raster unit }
    h_count, v_count, v_irqline, vc, vc_base, rc: Integer;
    display_state, badline, frame_badlines_enabled: Boolean;
    { memory unit }
    c_addr_or, g_addr_and, g_addr_or, i_addr, p_addr_or: Integer;
    fetch_cb: TM6569FetchFn;
    user_data: TObject;
    { graphics unit }
    g_enabled: Boolean;
    g_mode, g_count, g_shift, g_outp, g_outp2, g_c_data, g_last_bg: Integer;
    g_bg: array[0..3] of Integer;
    { sprite unit }
    s_h_first, s_h_last, s_h_offset, s_p_data: array[0..7] of Integer;
    s_dma_enabled, s_disp_enabled, s_expand: array[0..7] of Boolean;
    s_mc, s_mc_base, s_delay_count, s_outp2_count, s_xexp_count: array[0..7] of Integer;
    s_shift, s_outp, s_outp2: array[0..7] of Integer;
    s_colors: array[0..31] of Integer;   { [i*4 + k] }
    { video matrix }
    vmli, next_vmli: Integer;
    vm_line: array[0..63] of Integer;
    pins: Integer;
    g_data: Integer;
    { framebuffer the picture goes to (fb_w x fb_h), may be nil }
    fb: TUInts;
    fb_w, fb_h: Integer;

    procedure Init(afetch: TM6569FetchFn; user: TObject; sx, sy, sw, sh: Integer; afb: TUInts);
    procedure Reset;
    function TickFetch(apins: Integer): Integer;
    function TickFinish(apins: Integer): Integer;
  end;

const
  M6569_COLORS: array[0..15] of Integer = [
    $FF000000, $FFFFFFFF, $FF383381, $FFC8CE75, $FF973C8E, $FF4DAC56, $FF9B2C2E, $FF71F1ED,
    $FF29508E, $FF003855, $FF716CC4, $FF4A4A4A, $FF7B7B7B, $FF9FFFA9, $FFEB6D70, $FFB2B2B2];

  REG_MASK: array[0..63] of Integer = [
    $FF, $FF, $FF, $FF, $FF, $FF, $FF, $FF,
    $FF, $FF, $FF, $FF, $FF, $FF, $FF, $FF,
    $FF, $FF, $FF, $FF, $FF, $FF,
    $3F, $FF, $FE, $8F, $0F,
    $FF, $FF, $FF, $FF, $FF,
    $0F,
    $0F, $0F, $0F, $0F,
    $0F, $0F,
    $0F, $0F, $0F, $0F, $0F, $0F, $0F, $0F,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0];

implementation

const
  VRETRACEPOS = 303;
  RSEL1_BORDER_TOP = 51;
  RSEL1_BORDER_BOTTOM = 251;
  RSEL0_BORDER_TOP = 55;
  RSEL0_BORDER_BOTTOM = 247;
  CSEL1_BORDER_LEFT = 16;
  CSEL1_BORDER_RIGHT = 56;
  CSEL0_BORDER_LEFT = 17;
  CSEL0_BORDER_RIGHT = 55;

procedure TM6569.Init(afetch: TM6569FetchFn; user: TObject; sx, sy, sw, sh: Integer; afb: TUInts);
begin
  Reset;
  crt_x := 0; crt_y := 0;
  vis_x0 := sx div 8;
  vis_y0 := sy;
  vis_w := sw div 8;
  vis_h := sh;
  vis_x1 := vis_x0 + vis_w;
  vis_y1 := vis_y0 + vis_h;
  fetch_cb := afetch;
  user_data := user;
  fb := afb;
  fb_w := sw;
  fb_h := sh;
  pins := 0;
  g_data := 0;
end;

procedure TM6569.ResetBorderUnit;
begin
  brd_main := False;
  brd_vert := False;
  { c64-hw-sim: the border comparators are wired to RSEL/CSEL, valid from power on. }
  brd_top := RSEL0_BORDER_TOP;
  brd_bottom := RSEL0_BORDER_BOTTOM;
  brd_left := CSEL0_BORDER_LEFT;
  brd_right := CSEL0_BORDER_RIGHT;
end;

procedure TM6569.Reset;
var i: Integer;
begin
  for i := 0 to 63 do regs[i] := 0;
  h_count := 0; v_count := 0; v_irqline := 0; vc := 0; vc_base := 0; rc := 0;
  display_state := False; badline := False; frame_badlines_enabled := False;
  crt_x := 0; crt_y := 0;
  ResetBorderUnit;
  brd_bc := 0;
  c_addr_or := 0; g_addr_and := 0; g_addr_or := 0; i_addr := 0; p_addr_or := 0;
  vmli := 0; next_vmli := 0;
  for i := 0 to 63 do vm_line[i] := 0;
  g_enabled := False;
  g_mode := 0; g_count := 0; g_shift := 0; g_outp := 0; g_outp2 := 0; g_c_data := 0; g_last_bg := 0;
  for i := 0 to 3 do g_bg[i] := 0;
  for i := 0 to 7 do begin
    s_h_first[i] := 0; s_h_last[i] := 0; s_h_offset[i] := 0; s_p_data[i] := 0;
    s_dma_enabled[i] := False; s_disp_enabled[i] := False; s_expand[i] := False;
    s_mc[i] := 0; s_mc_base[i] := 0; s_delay_count[i] := 0; s_outp2_count[i] := 0; s_xexp_count[i] := 0;
    s_shift[i] := 0; s_outp[i] := 0; s_outp2[i] := 0;
  end;
  for i := 0 to 31 do s_colors[i] := 0;
end;

{--- register read/writes ---------------------------------------------------}

procedure TM6569.IoUpdateIrqLine(ctrl_1, rast: Integer);
begin
  v_irqline := ((ctrl_1 and M6569_CTRL1_RST8) shl 1) or rast;
end;

procedure TM6569.IoUpdateMemoryUnit(mem_ptrs, ctrl_1: Integer);
begin
  c_addr_or := (mem_ptrs and $F0) shl 6;
  g_addr_or := (mem_ptrs and $0E) shl 10;
  g_addr_and := $FFFF;
  i_addr := $3FFF;
  p_addr_or := ((mem_ptrs and $F0) shl 6) or $3F8;
  if (ctrl_1 and M6569_CTRL1_ECM) <> 0 then begin
    g_addr_and := g_addr_and and not ((1 shl 10) or (1 shl 9));
    i_addr := i_addr and not ((1 shl 10) or (1 shl 9));
  end;
end;

procedure TM6569.IoUpdateBorderRsel(ctrl_1: Integer);
begin
  if (ctrl_1 and M6569_CTRL1_RSEL) <> 0 then begin
    brd_top := RSEL1_BORDER_TOP;
    brd_bottom := RSEL1_BORDER_BOTTOM;
  end else begin
    brd_top := RSEL0_BORDER_TOP;
    brd_bottom := RSEL0_BORDER_BOTTOM;
  end;
end;

procedure TM6569.IoUpdateBorderCsel(ctrl_2: Integer);
begin
  if (ctrl_2 and M6569_CTRL2_CSEL) <> 0 then begin
    brd_left := CSEL1_BORDER_LEFT;
    brd_right := CSEL1_BORDER_RIGHT;
  end else begin
    brd_left := CSEL0_BORDER_LEFT;
    brd_right := CSEL0_BORDER_RIGHT;
  end;
end;

procedure TM6569.IoUpdateGunitMode(ctrl_1, ctrl_2: Integer);
begin
  g_mode := ((ctrl_1 and (M6569_CTRL1_ECM or M6569_CTRL1_BMM)) or (ctrl_2 and M6569_CTRL2_MCM)) shr 4;
end;

procedure TM6569.IoUpdateSunit(i, mx, my, mx8, mxe, mye: Integer);
var xpos, w: Integer;
begin
  xpos := ((mx8 and (1 shl i)) shl (8 - i)) or mx;
  { The PAL X counter wraps after 503: 504..511 are never reached. }
  if xpos < 504 then s_h_first[i] := ((xpos div 8) + 13) mod 63 else s_h_first[i] := $FF;
  s_h_offset[i] := xpos and 7;
  if (mxe and (1 shl i)) <> 0 then w := 5 else w := 2;
  if s_h_offset[i] > 0 then Inc(w);
  s_h_last[i] := (s_h_first[i] + w) and $FF;
  if (mye and (1 shl i)) = 0 then s_expand[i] := True;
end;

function TM6569.RegRead(pins: Integer): Integer;
var r_addr, data: Integer;
begin
  r_addr := pins and M6569_REG_MASK;
  case r_addr of
    $11: data := (regs[R_CTRL1] and $7F) or ((v_count and $100) shr 1);
    $12: data := v_count and $FF;
    $1E, $1F: begin
      data := regs[r_addr];
      regs[r_addr] := 0;
    end;
  else
    data := (regs[r_addr] or not REG_MASK[r_addr]) and $FF;
  end;
  Result := (pins and not $FF0000) or ((data and $FF) shl 16);
end;

procedure TM6569.RegWrite(pins: Integer);
var
  r_addr, data, i, mc, base: Integer;
  write: Boolean;
begin
  r_addr := pins and M6569_REG_MASK;
  data := ((pins shr 16) and $FF) and REG_MASK[r_addr];
  write := True;
  case r_addr of
    $00..$0F: begin
      i := r_addr shr 1;
      if (r_addr and 1) = 0 then
        IoUpdateSunit(i, data, regs[i * 2 + 1], regs[R_MX8], regs[R_MXE], regs[R_MYE])
      else
        IoUpdateSunit(i, regs[i * 2], data, regs[R_MX8], regs[R_MXE], regs[R_MYE]);
    end;
    $10:
      for i := 0 to 7 do IoUpdateSunit(i, regs[i * 2], regs[i * 2 + 1], data, regs[R_MXE], regs[R_MYE]);
    $11: begin
      IoUpdateIrqLine(data, regs[R_RASTER]);
      IoUpdateBorderRsel(data);
      IoUpdateMemoryUnit(regs[R_MEMPTRS], data);
      IoUpdateGunitMode(data, regs[R_CTRL2]);
    end;
    $12: IoUpdateIrqLine(regs[R_CTRL1], data);
    $16: begin
      IoUpdateBorderCsel(data);
      IoUpdateGunitMode(regs[R_CTRL1], data);
    end;
    $17:
      for i := 0 to 7 do begin
        if ((data and (1 shl i)) = 0) and not s_expand[i] and (h_count = 15) then begin
          { Clearing Y expansion during cycle 15 mixes MC and MCBASE. }
          mc := s_mc[i];
          base := s_mc_base[i];
          s_mc[i] := ((mc and base) and $2A) or ((mc or base) and $15);
        end;
        IoUpdateSunit(i, regs[i * 2], regs[i * 2 + 1], regs[R_MX8], regs[R_MXE], data);
      end;
    $18: IoUpdateMemoryUnit(data, regs[R_CTRL1]);
    $19: begin
      regs[R_INTLATCH] := (regs[R_INTLATCH] and not data) and REG_MASK[$19];
      write := False;
    end;
    $1D:
      for i := 0 to 7 do IoUpdateSunit(i, regs[i * 2], regs[i * 2 + 1], regs[R_MX8], data, regs[R_MYE]);
    $1E, $1F: write := False;
    $20: brd_bc := data and $F;
    $21, $22: g_bg[r_addr - $21] := data and $F;
    $23, $24: g_bg[r_addr - $21] := $FF00 or (data and $F);
    $25: for i := 0 to 7 do s_colors[i * 4 + 1] := data and $F;
    $26: for i := 0 to 7 do s_colors[i * 4 + 3] := data and $F;
    $27..$2E: s_colors[(r_addr - $27) * 4 + 2] := data and $F;
  end;
  if write then regs[r_addr] := data;
end;

{--- graphics sequencer -----------------------------------------------------}

procedure TM6569.GunitRewind(xscroll: Integer);
begin
  g_count := xscroll;
  g_shift := 0;
  g_outp := 0;
  g_outp2 := 0;
  g_c_data := 0;
end;

procedure TM6569.GunitTick(gd: Integer);
begin
  if g_count = 0 then begin
    g_count := 7;
    g_shift := g_shift or gd;
    if g_enabled then g_c_data := vm_line[vmli] else g_c_data := 0;
  end else
    Dec(g_count);
  g_outp := g_shift;
  if (g_count and 1) <> 0 then g_outp2 := g_shift;
  g_shift := (g_shift shl 1) and $FF;
end;

function TM6569.GunitDecode(mode: Integer): Integer;
var fg, bits: Integer;
begin
  case mode of
    0:
      if (g_outp and $80) <> 0 then Result := $FF00 or ((g_c_data shr 8) and $F)
      else Result := g_bg[0];
    1: begin
      fg := (g_c_data shr 8) and 7;
      if (g_c_data and (1 shl 11)) <> 0 then begin
        bits := (g_outp2 shr 6) and 3;
        if bits = 3 then Result := $FF00 or fg else Result := g_bg[bits];
      end else begin
        if (g_outp and $80) <> 0 then Result := $FF00 or fg else Result := g_bg[0];
      end;
    end;
    2:
      if (g_outp and $80) <> 0 then Result := $FF00 or ((g_c_data shr 4) and $F)
      else Result := g_c_data and $F;
    3:
      case (g_outp2 shr 6) and 3 of
        0: Result := g_bg[0];
        1: Result := (g_c_data shr 4) and $F;
        2: Result := $FF00 or (g_c_data and $F);
      else
        Result := $FF00 or ((g_c_data shr 8) and $F);
      end;
    4:
      if (g_outp and $80) <> 0 then Result := $FF00 or ((g_c_data shr 8) and $F)
      else Result := g_bg[(g_c_data shr 6) and 3];
    5:
      if (g_c_data and (1 shl 11)) <> 0 then begin
        if ((g_outp2 shr 6) and 3) >= 2 then Result := $FF00 else Result := 0;
      end else begin
        if (g_outp and $80) <> 0 then Result := $FF00 else Result := 0;
      end;
    6:
      if (g_outp and $80) <> 0 then Result := $FF00 else Result := 0;
  else
    if ((g_outp2 shr 6) and 3) >= 2 then Result := $FF00 else Result := 0;
  end;
end;

{--- sprite sequencer -------------------------------------------------------}

procedure TM6569.SunitStart;
var
  me, mye, i, mask: Integer;
begin
  me := regs[R_ME];
  mye := regs[R_MYE];
  for i := 0 to 7 do begin
    mask := 1 shl i;
    if ((me and mask) <> 0) and ((v_count and $FF) = regs[i * 2 + 1]) then begin
      if not s_dma_enabled[i] then begin
        s_dma_enabled[i] := True;
        s_mc_base[i] := 0;
        s_expand[i] := True;
      end;
    end;
    if (h_count = 56) and s_dma_enabled[i] and ((mye and mask) <> 0) then
      s_expand[i] := not s_expand[i];
  end;
end;

procedure TM6569.SunitUpdateMcDispEnable;
var i: Integer;
begin
  for i := 0 to 7 do begin
    s_mc[i] := s_mc_base[i];
    if not s_dma_enabled[i] then s_disp_enabled[i] := False
    else if ((regs[R_ME] and (1 shl i)) <> 0) and ((v_count and $FF) = regs[i * 2 + 1]) then
      s_disp_enabled[i] := True;
  end;
end;

procedure TM6569.SunitUpdateMcbase;
var i: Integer;
begin
  for i := 0 to 7 do
    if s_expand[i] then begin
      s_mc_base[i] := s_mc[i];
      if s_mc_base[i] = $3F then s_dma_enabled[i] := False;
    end;
end;

function TM6569.DmaBA(i, pins: Integer): Integer;
begin
  if s_dma_enabled[i] then Result := pins or M6569_BA else Result := pins;
end;

function TM6569.DmaAEC(i, pins: Integer): Integer;
begin
  if s_dma_enabled[i] then Result := pins or M6569_AEC else Result := pins;
end;

function TM6569.SunitDecode(hpos: Integer): Integer;
var
  c, mxe, mmc, i, ci: Integer;
  collision: Boolean;
begin
  c := 0;
  collision := False;
  mxe := regs[R_MXE];
  mmc := regs[R_MMC];
  for i := 0 to 7 do begin
    if s_disp_enabled[i] and (s_h_first[i] <> $FF) and
       (((hpos + 63 - s_h_first[i]) mod 63) <= s_h_last[i] - s_h_first[i]) then begin
      if s_delay_count[i] = 0 then begin
        ci := s_xexp_count[i];
        s_xexp_count[i] := (ci + 1) and $FF;
        if ((ci and 1) = 0) or ((mxe and (1 shl i)) = 0) then begin
          s_outp[i] := s_shift[i];
          ci := s_outp2_count[i];
          s_outp2_count[i] := (ci + 1) and $FF;
          if (ci and 1) = 0 then s_outp2[i] := s_shift[i];
          s_shift[i] := s_shift[i] shl 1;
        end;
        if (mmc and (1 shl i)) <> 0 then begin
          ci := (s_outp2[i] shr 30) and 3;
          if ci <> 0 then begin
            if c = 0 then c := s_colors[i * 4 + ci]
            else collision := True;
            c := c or (1 shl (8 + i));
          end;
        end else begin
          if (s_outp[i] and $80000000) <> 0 then begin
            if c = 0 then c := s_colors[i * 4 + 2]   { the (1 shl (24+i)) of the C is lost in uint16 }
            else collision := True;
            c := c or (1 shl (8 + i));
          end;
        end;
      end else
        Dec(s_delay_count[i]);
    end;
  end;
  if collision then begin
    regs[R_MCM] := (regs[R_MCM] or (c shr 8)) and $FF;
    regs[R_INTLATCH] := regs[R_INTLATCH] or M6569_INT_IMMC;
  end;
  Result := c;
end;

procedure TM6569.DecodePixels(x, y, gd, hpos: Integer);
var
  i, mdp, mode, bmc, sc, c, idx: Integer;
  visible, main_brd, vert_brd, sprites: Boolean;
begin
  sprites := False;
  for i := 0 to 7 do
    if s_disp_enabled[i] then sprites := True;
  for i := 0 to 7 do
    if s_disp_enabled[i] and (hpos = s_h_first[i]) then begin
      s_delay_count[i] := s_h_offset[i];
      s_outp2_count[i] := 0;
      s_xexp_count[i] := 0;
    end;
  visible := (x >= 0) and (x < vis_w * 8) and (y >= 0) and (y < vis_h);
  main_brd := brd_main;
  vert_brd := brd_vert;
  mdp := regs[R_MDP];
  mode := g_mode;
  bmc := 0;
  for i := 0 to 7 do begin
    if sprites then sc := SunitDecode(hpos) else sc := 0;   { (no sprite shown: nothing to do) }
    GunitTick(gd);
    if vert_brd then
      bmc := g_last_bg
    else begin
      bmc := GunitDecode(mode);
      if (bmc and $FF00) = 0 then g_last_bg := bmc and $FF;
    end;
    if not vert_brd then begin
      { mob-data collision }
      if (sc and bmc and $FF00) <> 0 then begin
        regs[R_MCD] := (regs[R_MCD] or (sc shr 8)) and $FF;
        regs[R_INTLATCH] := regs[R_INTLATCH] or M6569_INT_IMBC;
      end;
    end;
    if main_brd then c := brd_bc
    else begin
      { color multiplexer }
      if sc = 0 then c := bmc
      else if ((sc shr 8) and mdp) <> 0 then begin
        if (bmc and $FF00) = 0 then c := sc else c := bmc;
      end else c := sc;
      c := c and $FF;
    end;
    if visible and (fb <> nil) then begin
      { crt_set_pixel of c64.c: clipped to the screen }
      if (x + i < fb_w) and (y < fb_h) then begin
        idx := y * fb_w + x + i;
        fb[idx] := M6569_COLORS[c and 15];
      end;
    end;
  end;
end;

{--- raster unit ------------------------------------------------------------}

procedure TM6569.RsNextRasterline;
begin
  if v_count = M6569_VTOTAL - 1 then begin
    v_count := 0;
    vc_base := 0;
  end else
    Inc(v_count);
end;

procedure TM6569.RsUpdateBadline;
begin
  if (v_count >= 48) and (v_count <= 247) then begin
    if (v_count = 48) and ((regs[R_CTRL1] and M6569_CTRL1_DEN) <> 0) then
      frame_badlines_enabled := True;
    badline := frame_badlines_enabled and ((v_count and 7) = (regs[R_CTRL1] and 7));
  end else begin
    frame_badlines_enabled := False;
    badline := False;
  end;
  if badline then display_state := True;
end;

procedure TM6569.RsUpdateDisplayState;
begin
  if rc = 7 then begin
    vc_base := vc;
    if not badline then display_state := False;
  end;
  if display_state then rc := (rc + 1) and 7;
end;

procedure TM6569.RsRewindVcVmliRc;
begin
  vc := vc_base;
  vmli := 0;
  next_vmli := 0;
  if badline then rc := 0;
end;

procedure TM6569.CrtNextCrtline;
begin
  crt_x := 0;
  if v_count = VRETRACEPOS then crt_y := 0
  else crt_y := (crt_y + 1) and $FFFF;
end;

procedure TM6569.BunitLeft(hpos: Integer);
begin
  if hpos = brd_left then begin
    if v_count = brd_bottom then brd_vert := True
    else if (v_count = brd_top) and ((regs[R_CTRL1] and M6569_CTRL1_DEN) <> 0) then brd_vert := False;
    if not brd_vert then brd_main := False;
  end;
end;

procedure TM6569.BunitEnd;
begin
  if v_count = brd_bottom then brd_vert := True
  else if (v_count = brd_top) and ((regs[R_CTRL1] and M6569_CTRL1_DEN) <> 0) then brd_vert := False;
end;

{--- memory accesses --------------------------------------------------------}

procedure TM6569.CAccess;
begin
  if badline then
    vm_line[vmli] := fetch_cb(vc or c_addr_or, user_data) and $0FFF;
end;

function TM6569.IAccess: Integer;
begin
  Result := fetch_cb(i_addr, user_data) and $FF;
end;

function TM6569.GIAccess: Integer;
var addr: Integer;
begin
  if display_state then begin
    if (regs[R_CTRL1] and M6569_CTRL1_BMM) <> 0 then begin
      addr := ((vc shl 3) or rc) and $FFFF;
      addr := (addr or (g_addr_or and (1 shl 13))) and g_addr_and;
    end else begin
      addr := (((vm_line[vmli] and $FF) shl 3) or rc) and $FFFF;
      addr := (addr or g_addr_or) and g_addr_and;
    end;
    vc := (vc + 1) and $3FF;
    next_vmli := (vmli + 1) and $3F;
    Result := fetch_cb(addr, user_data) and $FF;
  end else
    Result := IAccess;
end;

procedure TM6569.PAccess(i: Integer);
begin
  s_p_data[i] := fetch_cb((p_addr_or + i) and $FFFF, user_data) and $FF;
end;

procedure TM6569.SAccess(i: Integer);
var s_data: Integer;
begin
  if s_dma_enabled[i] then begin
    s_data := fetch_cb(((s_p_data[i] shl 6) or s_mc[i]) and $FFFF, user_data) and $FF;
    s_shift[i] := (s_shift[i] shl 8) or (s_data shl 8);
    s_mc[i] := (s_mc[i] + 1) and $3F;
  end;
end;

function TM6569.SIAccess(i: Integer): Integer;
begin
  if s_dma_enabled[i] then begin
    SAccess(i);
    Result := 0;
  end else
    Result := IAccess;
end;

{--- tick -------------------------------------------------------------------}

function TM6569.TickFetch(apins: Integer): Integer;
var
  p, gd: Integer;
begin
  p := apins and not M6569_BA;
  gd := 0;
  Inc(h_count);
  crt_x := (crt_x + 1) and $FFFF;
  if h_count = 1 then begin
    RsNextRasterline;
    if v_count = v_irqline then regs[R_INTLATCH] := regs[R_INTLATCH] or M6569_INT_IRST;
  end;
  RsUpdateBadline;
  case h_count of
    1: begin
      PAccess(3); SAccess(3);
      p := DmaAEC(3, p); p := DmaBA(3, p); p := DmaBA(4, p);
    end;
    2: begin
      gd := SIAccess(3); SAccess(3);
      p := DmaAEC(3, p); p := DmaBA(3, p); p := DmaBA(4, p); p := DmaBA(5, p);
    end;
    3: begin
      PAccess(4); SAccess(4);
      p := DmaAEC(4, p); p := DmaBA(4, p); p := DmaBA(5, p);
    end;
    4: begin
      CrtNextCrtline;
      gd := SIAccess(4); SAccess(4);
      p := DmaAEC(4, p); p := DmaBA(4, p); p := DmaBA(5, p); p := DmaBA(6, p);
    end;
    5: begin
      PAccess(5); SAccess(5);
      p := DmaAEC(5, p); p := DmaBA(5, p); p := DmaBA(6, p);
    end;
    6: begin
      gd := SIAccess(5); SAccess(5);
      p := DmaAEC(5, p); p := DmaBA(5, p); p := DmaBA(6, p); p := DmaBA(7, p);
    end;
    7: begin
      PAccess(6); SAccess(6);
      p := DmaAEC(6, p); p := DmaBA(6, p); p := DmaBA(7, p);
    end;
    8: begin
      gd := SIAccess(6); SAccess(6);
      p := DmaAEC(6, p); p := DmaBA(6, p); p := DmaBA(7, p);
    end;
    9: begin
      PAccess(7); SAccess(7);
      p := DmaAEC(7, p); p := DmaBA(7, p);
    end;
    10: begin
      gd := SIAccess(7); SAccess(7);
      p := DmaAEC(7, p); p := DmaBA(7, p);
    end;
    11: ;
    12, 13, 14:
      if badline then p := p or M6569_BA;
    15: begin
      if badline then p := p or M6569_BA;
      p := p or M6569_AEC;
      RsRewindVcVmliRc;
    end;
    16: begin
      if badline then p := p or M6569_BA;
      p := p or M6569_AEC;
      g_enabled := display_state;
      GunitRewind(regs[R_CTRL2] and M6569_CTRL2_XSCROLL);
      SunitUpdateMcbase;
      CAccess;
      gd := GIAccess;
    end;
    17..54: begin
      if badline then p := p or M6569_BA;
      p := p or M6569_AEC;
      g_enabled := display_state;
      CAccess;
      gd := GIAccess;
    end;
    55: begin
      p := p or M6569_AEC;
      g_enabled := display_state;
      CAccess;
      gd := GIAccess;
      SunitStart;
      p := DmaBA(0, p);
    end;
    56: begin
      g_enabled := False;
      SunitStart;
      gd := IAccess;
      p := DmaBA(0, p);
    end;
    57: begin
      gd := IAccess;
      p := DmaBA(0, p); p := DmaBA(1, p);
    end;
    58: begin
      SunitUpdateMcDispEnable;
      PAccess(0); SAccess(0);
      p := DmaAEC(0, p); p := DmaBA(0, p); p := DmaBA(1, p);
    end;
    59: begin
      RsUpdateDisplayState;
      gd := SIAccess(0); SAccess(0);
      p := DmaAEC(0, p); p := DmaBA(0, p); p := DmaBA(1, p); p := DmaBA(2, p);
    end;
    60: begin
      PAccess(1); SAccess(1);
      p := DmaAEC(1, p); p := DmaBA(1, p); p := DmaBA(2, p);
    end;
    61: begin
      gd := SIAccess(1); SAccess(1);
      p := DmaAEC(1, p); p := DmaBA(1, p); p := DmaBA(2, p); p := DmaBA(3, p);
    end;
    62: begin
      PAccess(2); SAccess(2);
      p := DmaAEC(2, p); p := DmaBA(2, p); p := DmaBA(3, p);
    end;
    63: begin
      h_count := 0;
      gd := SIAccess(2); SAccess(2);
      p := DmaAEC(2, p); p := DmaBA(2, p); p := DmaBA(3, p); p := DmaBA(4, p);
      BunitEnd;
    end;
  end;
  { main interrupt bit }
  if (regs[R_INTLATCH] and regs[R_INTMASK] and $0F) <> 0 then
    regs[R_INTLATCH] := regs[R_INTLATCH] or M6569_INT_IRQ
  else
    regs[R_INTLATCH] := regs[R_INTLATCH] and not M6569_INT_IRQ;
  if (regs[R_INTLATCH] and M6569_INT_IRQ) <> 0 then p := p or M6569_IRQ;
  g_data := gd;
  Result := p;
end;

function TM6569.TickFinish(apins: Integer): Integer;
var p: Integer;
begin
  p := apins;
  if (p and (M6569_CS or M6569_RW)) = M6569_CS then RegWrite(p);
  if (h_count = 16) or (h_count = 17) then BunitLeft(h_count);
  if ((h_count = 55) or (h_count = 56)) and (h_count = brd_right) then brd_main := True;
  DecodePixels((crt_x - vis_x0) * 8, crt_y - vis_y0, g_data, h_count);
  vmli := next_vmli;
  if (p and (M6569_CS or M6569_RW)) = (M6569_CS or M6569_RW) then p := RegRead(p);
  pins := p;
  Result := p;
end;

end.
