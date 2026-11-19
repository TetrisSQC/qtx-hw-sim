unit hwsim.util;

{ Small helpers shared by all units of the simulator: byte buffers, 32-bit
  integer arithmetic (JavaScript numbers are doubles, bitwise operators work
  on signed 32-bit values) and a few string helpers. }

interface

uses
  qtx.sysutils;

type
  TBytes = JUint8Array;
  TWords = JUint16Array;
  TInts = JInt32Array;
  TUInts = JUint32Array;
  TFloats = JFloat64Array;

const
  { "Never": stands in for UINT64_MAX cycle counts of the C code. }
  CYCLE_NEVER = 1e300;

function NewBytes(n: Integer): TBytes;
function NewWords(n: Integer): TWords;
function NewInts(n: Integer): TInts;
function NewUInts(n: Integer): TUInts;
function NewFloats(n: Integer): TFloats;

{ 32-bit integer multiply with wrap-around (C: uint32 * uint32). The result
  is signed; use U32() to make it unsigned. }
function Imul(a, b: Integer): Integer;
{ Reinterpret as unsigned 32-bit. }
function U32(a: Integer): Integer;

function Base64ToBytes(const s: String): TBytes;

{ Is 'word' one of the space separated words of 'list'? }
function InList(const list, word: String): Boolean;
{ C atoi(): leading integer of a string, 0 if none. }
function AtoI(const s: String): Integer;
function StrContains(const s, sub: String): Boolean;
function HexB(v: Integer): String;
function HexW(v: Integer): String;

implementation

function NewBytes(n: Integer): TBytes;
begin
  Result := new JUint8Array(n);
end;

function NewWords(n: Integer): TWords;
begin
  Result := new JUint16Array(n);
end;

function NewInts(n: Integer): TInts;
begin
  Result := new JInt32Array(n);
end;

function NewUInts(n: Integer): TUInts;
begin
  Result := new JUint32Array(n);
end;

function NewFloats(n: Integer): TFloats;
begin
  Result := new JFloat64Array(n);
end;

function Imul(a, b: Integer): Integer;
begin
  asm @Result = Math.imul(@a, @b); end;
end;

function U32(a: Integer): Integer;
begin
  asm @Result = (@a) >>> 0; end;
end;

function Base64ToBytes(const s: String): TBytes;
const
  ALPHA = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
var
  lut: array[0..127] of Integer;
  i, n, len, acc, bits, o, c: Integer;
begin
  for i := 0 to 127 do lut[i] := -1;
  for i := 1 to 64 do lut[Ord(ALPHA[i])] := i - 1;
  len := Length(s);
  n := 0;
  for i := 1 to len do
    if (Ord(s[i]) < 128) and (lut[Ord(s[i])] >= 0) then Inc(n);
  Result := NewBytes((n * 6) div 8);
  acc := 0; bits := 0; o := 0;
  for i := 1 to len do begin
    c := Ord(s[i]);
    if (c >= 128) or (lut[c] < 0) then continue;
    acc := ((acc shl 6) or lut[c]) and $FFFFFF;
    bits := bits + 6;
    if bits >= 8 then begin
      bits := bits - 8;
      if o < Result.length then Result[o] := (acc shr bits) and 255;
      Inc(o);
    end;
  end;
end;

function InList(const list, word: String): Boolean;
var
  parts: array of String;
  i: Integer;
begin
  Result := False;
  if (list = '') or (word = '') then exit;
  parts := list.Split(' ');
  for i := 0 to parts.High do
    if parts[i] = word then exit(True);
end;

function AtoI(const s: String): Integer;
var
  i, sign: Integer;
begin
  Result := 0;
  sign := 1;
  i := 1;
  while (i <= Length(s)) and (s[i] = ' ') do Inc(i);
  if (i <= Length(s)) and ((s[i] = '-') or (s[i] = '+')) then begin
    if s[i] = '-' then sign := -1;
    Inc(i);
  end;
  while (i <= Length(s)) and (s[i] >= '0') and (s[i] <= '9') do begin
    Result := Result * 10 + (Ord(s[i]) - 48);
    Inc(i);
  end;
  Result := Result * sign;
end;

function StrContains(const s, sub: String): Boolean;
begin
  Result := Pos(sub, s) > 0;
end;

function HexB(v: Integer): String;
begin
  Result := IntToHex(v and $FF, 2);
end;

function HexW(v: Integer): String;
begin
  Result := IntToHex(v and $FFFF, 4);
end;

end.
