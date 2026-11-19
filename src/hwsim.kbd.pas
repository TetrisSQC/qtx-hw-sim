unit hwsim.kbd;

{ Keyboard matrix helper, port of floooh's chips/kbd.h (zlib license,
  Copyright (c) 2018 Andre Weissflog). Keys are registered with a column,
  a line and modifier layers; the matrix (which lines a column closes) is
  computed from the pressed keys. Times are in microseconds (emulated). }

interface

const
  KBD_MAX_COLUMNS = 12;
  KBD_MAX_LINES = 12;
  KBD_MAX_MOD_KEYS = 4;
  KBD_MAX_KEYS = 256;
  KBD_MAX_PRESSED_KEYS = 4;

type
  TKbd = class
  private
    function TestLines(column_mask: Integer): Integer;
    function TestColumns(line_mask: Integer): Integer;
    procedure UpdateScanoutMasks;
  public
    cur_time: Float;
    sticky_time: Integer;
    key_masks: array[0..KBD_MAX_KEYS - 1] of Integer;
    mod_masks: array[0..KBD_MAX_MOD_KEYS - 1] of Integer;
    kb_key: array[0..KBD_MAX_PRESSED_KEYS - 1] of Integer;
    kb_mask: array[0..KBD_MAX_PRESSED_KEYS - 1] of Integer;
    kb_pressed_time: array[0..KBD_MAX_PRESSED_KEYS - 1] of Float;
    kb_released: array[0..KBD_MAX_PRESSED_KEYS - 1] of Boolean;
    scanout_column_masks: array[0..KBD_MAX_LINES - 1] of Integer;
    scanout_line_masks: array[0..KBD_MAX_COLUMNS - 1] of Integer;
    procedure Init(sticky_frames: Integer);
    procedure Update(frame_time_us: Integer);
    procedure RegisterModifier(layer, column, line: Integer);
    procedure RegisterModifierLine(layer, line: Integer);
    procedure RegisterKey(key, column, line, mod_mask: Integer);
    procedure KeyDown(key: Integer);
    procedure KeyUp(key: Integer);
  end;

implementation

function KColumns(m: Integer): Integer;
begin
  Result := (m shr KBD_MAX_LINES) and ((1 shl KBD_MAX_COLUMNS) - 1);
end;

function KLines(m: Integer): Integer;
begin
  Result := m and ((1 shl KBD_MAX_LINES) - 1);
end;

function KMod(m: Integer): Integer;
begin
  Result := m and (((1 shl KBD_MAX_MOD_KEYS) - 1) shl (KBD_MAX_COLUMNS + KBD_MAX_LINES));
end;

procedure TKbd.Init(sticky_frames: Integer);
var i: Integer;
begin
  cur_time := 0;
  sticky_time := sticky_frames * 16667;
  for i := 0 to KBD_MAX_KEYS - 1 do key_masks[i] := 0;
  for i := 0 to KBD_MAX_MOD_KEYS - 1 do mod_masks[i] := 0;
  for i := 0 to KBD_MAX_PRESSED_KEYS - 1 do begin
    kb_key[i] := 0; kb_mask[i] := 0; kb_pressed_time[i] := 0; kb_released[i] := False;
  end;
  for i := 0 to KBD_MAX_LINES - 1 do scanout_column_masks[i] := 0;
  for i := 0 to KBD_MAX_COLUMNS - 1 do scanout_line_masks[i] := 0;
end;

procedure TKbd.RegisterModifier(layer, column, line: Integer);
begin
  mod_masks[layer] := (1 shl (layer + KBD_MAX_COLUMNS + KBD_MAX_LINES)) or (1 shl (column + KBD_MAX_LINES)) or (1 shl line);
end;

procedure TKbd.RegisterModifierLine(layer, line: Integer);
begin
  mod_masks[layer] := (1 shl (layer + KBD_MAX_COLUMNS + KBD_MAX_LINES)) or (1 shl line);
end;

procedure TKbd.RegisterKey(key, column, line, mod_mask: Integer);
begin
  key_masks[key] := (mod_mask shl (KBD_MAX_COLUMNS + KBD_MAX_LINES)) or (1 shl (column + KBD_MAX_LINES)) or (1 shl line);
end;

function TKbd.TestLines(column_mask: Integer): Integer;
var
  ki, mi, km, kc, kmodm, mm, mc: Integer;
begin
  Result := 0;
  for ki := 0 to KBD_MAX_PRESSED_KEYS - 1 do begin
    km := kb_mask[ki];
    if km = 0 then continue;
    kc := KColumns(km);
    if (kc and column_mask) = kc then Result := Result or KLines(km);
    kmodm := KMod(km);
    if kmodm <> 0 then
      for mi := 0 to KBD_MAX_MOD_KEYS - 1 do begin
        mm := mod_masks[mi];
        if (mm and kmodm) <> 0 then begin
          mc := KColumns(mm);
          if mc <> 0 then begin
            if (mc and column_mask) = mc then Result := Result or KLines(mm);
          end else
            Result := Result or KLines(mm);
        end;
      end;
  end;
end;

function TKbd.TestColumns(line_mask: Integer): Integer;
var
  ki, mi, km, kl, kmodm, mm, ml: Integer;
begin
  Result := 0;
  for ki := 0 to KBD_MAX_PRESSED_KEYS - 1 do begin
    km := kb_mask[ki];
    if km = 0 then continue;
    kl := KLines(km);
    if (kl and line_mask) = kl then Result := Result or KColumns(km);
    kmodm := KMod(km);
    if kmodm <> 0 then
      for mi := 0 to KBD_MAX_MOD_KEYS - 1 do begin
        mm := mod_masks[mi];
        if (mm and kmodm) <> 0 then begin
          ml := KLines(mm);
          if ml <> 0 then begin
            if (ml and line_mask) = ml then Result := Result or KColumns(mm);
          end else
            Result := Result or KColumns(mm);
        end;
      end;
  end;
end;

procedure TKbd.UpdateScanoutMasks;
var i: Integer;
begin
  for i := 0 to KBD_MAX_LINES - 1 do scanout_column_masks[i] := TestColumns(1 shl i);
  for i := 0 to KBD_MAX_COLUMNS - 1 do scanout_line_masks[i] := TestLines(1 shl i);
end;

procedure TKbd.Update(frame_time_us: Integer);
var i: Integer;
begin
  for i := 0 to KBD_MAX_PRESSED_KEYS - 1 do
    if kb_released[i] then
      if (cur_time < kb_pressed_time[i]) or (cur_time > kb_pressed_time[i] + sticky_time) then begin
        kb_mask[i] := 0;
        kb_key[i] := 0;
        kb_pressed_time[i] := 0;
        kb_released[i] := False;
      end;
  cur_time := cur_time + frame_time_us;
  UpdateScanoutMasks;
end;

procedure TKbd.KeyDown(key: Integer);
var i: Integer;
begin
  for i := 0 to KBD_MAX_PRESSED_KEYS - 1 do
    if kb_key[i] = key then begin
      kb_pressed_time[i] := cur_time;
      UpdateScanoutMasks;
      exit;
    end;
  for i := 0 to KBD_MAX_PRESSED_KEYS - 1 do
    if kb_mask[i] = 0 then begin
      kb_key[i] := key;
      kb_mask[i] := key_masks[key];
      kb_pressed_time[i] := cur_time;
      kb_released[i] := False;
      UpdateScanoutMasks;
      exit;
    end;
end;

procedure TKbd.KeyUp(key: Integer);
var i: Integer;
begin
  for i := 0 to KBD_MAX_PRESSED_KEYS - 1 do
    if kb_key[i] = key then kb_released[i] := True;
  UpdateScanoutMasks;
end;

end.
