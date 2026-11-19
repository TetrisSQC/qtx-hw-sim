unit hwsim.m6581;

{ MOS 6581 SID, port of floooh's chips/m6581.h (zlib license, Copyright (c)
  2018 Andre Weissflog), with the c64-hw-sim POT inputs.

  Pins (bits of an Integer): A0-A4 0..4, D0-D7 16..23, RW 24, CS 30,
  SAMPLE 31 (CS/SAMPLE are bits 40/41 in the C code).

  The audio resampler (a 1023 tap FIR, the heaviest part) only runs while
  'audio' is true: it doesn't change anything the CPU can read. }

interface

uses
  hwsim.util;

const
  M6581_RW = 1 shl 24;
  M6581_CS = 1 shl 30;
  M6581_SAMPLE = 1 shl 31;
  M6581_ADDR_MASK = $1F;
  M6581_POT_X = 25;
  M6581_POT_Y = 26;
  M6581_OSC3RAND = 27;
  M6581_ENV3 = 28;
  M6581_SAMPLE_BUFFER_SIZE = 1024;

type
  TM6581Voice = class
  public
    muted: Boolean;
    freq, pulse_width, ctrl: Integer;
    sync: Boolean;
    noise_shift, noise_latch, noise_reset, noise_delay: Integer;
    wav_accum, wav_output, pulse_output: Integer;
    env_state: Integer;
    env_attack_add, env_decay_sub, env_sustain_level, env_release_sub: Integer;
    env_cur_level, env_counter, env_exp_counter, env_counter_compare, env_exp_period: Integer;
    env_hold_zero: Boolean;
    procedure Init;
  end;

  TM6581 = class
  private
    procedure InitFilter;
    procedure InitSampling;
    procedure SetFilterCutoff;
    procedure SetResonance;
    procedure SetCtrl(v: TM6581Voice; data: Integer);
    procedure VoiceTick(i: Integer);
    procedure VoiceOutput(i: Integer);
    function Triangle(v, vs: TM6581Voice): Integer;
    function Pulse(v: TM6581Voice): Integer;
    function FilterOutput(vi: Float): Float;
    function SampleOut(sample: Float): Boolean;
    function RegRead(apins: Integer): Integer;
    procedure RegWrite(apins: Integer);
  public
    sound_hz, tick_hz: Integer;
    bus_value, bus_decay: Integer;
    pot: array[0..1] of Integer;
    voice: array[0..2] of TM6581Voice;
    { filter }
    f_cutoff, f_resonance, f_voices, f_mode, f_volume: Integer;
    f_resonance_coeff, f_w0, f_v_hp, f_v_bp, f_v_lp: Float;
    { sampling }
    audio: Boolean;
    sample_phase, sample_pos: Integer;
    sample_buffer: TFloats;
    sample_filter: TFloats;
    ext_lp, ext_hp, ext_lp_coeff, ext_hp_coeff, sample_mag, sample: Float;
    pins: Integer;
    constructor Create;
    procedure Init(atick_hz, asound_hz: Integer; magnitude: Float);
    procedure Reset;
    function Tick(apins: Integer): Integer;
  end;

implementation

const
  ENV_FROZEN = 0; ENV_ATTACK = 1; ENV_DECAY = 2; ENV_RELEASE = 3;
  CTRL_GATE = 1; CTRL_SYNC = 2; CTRL_RINGMOD = 4; CTRL_TEST = 8; CTRL_SAWTOOTH = $20;
  FILTER_LP = 1; FILTER_BP = 2; FILTER_HP = 4; FILTER_3OFF = 8;
  DCWAVE = 0.40;
  DCMIXER = -0.04;
  DCVOICE = 0.26;
  RATE_COUNT_PERIOD: array[0..15] of Integer = [
    $7F00, $0006, $003C, $0330, $20C0, $6755, $3800, $500E,
    $1212, $0222, $1848, $59B8, $3840, $77E2, $7625, $0A93];

function BIT(val, nr: Integer): Integer;
begin
  Result := (val shr nr) and 1;
end;

procedure TM6581Voice.Init;
begin
  muted := False;
  freq := 0; pulse_width := 0; ctrl := 0; sync := False;
  noise_shift := $007FFFFC; noise_latch := 0; noise_reset := 0; noise_delay := 0;
  wav_accum := 0; wav_output := 0; pulse_output := 0;
  env_state := ENV_RELEASE;
  env_attack_add := 0; env_decay_sub := 0; env_sustain_level := 0; env_release_sub := 0;
  env_cur_level := 0; env_exp_counter := 0; env_counter_compare := 0;
  env_exp_period := 1;
  env_hold_zero := True;
  env_counter := $7FFF;
end;

constructor TM6581.Create;
var i: Integer;
begin
  inherited Create;
  for i := 0 to 2 do voice[i] := TM6581Voice.Create;
  sample_buffer := NewFloats(M6581_SAMPLE_BUFFER_SIZE * 2);
  sample_filter := NewFloats(M6581_SAMPLE_BUFFER_SIZE - 1);
end;

procedure TM6581.SetFilterCutoff;
var x, cf: Float;
begin
  x := f_cutoff / 8.0;
  cf := -0.0156 * x * x + 48.473 * x - 45.074;
  if cf < 30.0 then cf := 30.0;
  f_w0 := 6.28318530718 * cf / tick_hz;
end;

procedure TM6581.SetResonance;
begin
  f_resonance_coeff := 1.0 / (0.707 + 1.9 * f_resonance / 15.0);
end;

procedure TM6581.InitFilter;
begin
  f_cutoff := 0; f_resonance := 0; f_voices := 0; f_mode := 0; f_volume := 0;
  f_v_hp := 0; f_v_bp := 0; f_v_lp := 0;
  SetFilterCutoff;
  SetResonance;
end;

procedure TM6581.InitSampling;
var
  n, i: Integer;
  cutoff, sum, x, sinc, phase, window: Float;
begin
  n := M6581_SAMPLE_BUFFER_SIZE - 1;
  cutoff := 0.4 * sound_hz / tick_hz;
  sum := 0;
  for i := 0 to n - 1 do begin
    x := i - (n - 1) * 0.5;
    if x = 0 then sinc := 2.0 * cutoff else sinc := Sin(2.0 * PI * cutoff * x) / (PI * x);
    phase := 2.0 * PI * i / (n - 1);
    window := 0.42 - 0.5 * Cos(phase) + 0.08 * Cos(2.0 * phase);
    sample_filter[i] := sinc * window;
    sum := sum + sinc * window;
  end;
  for i := 0 to n - 1 do sample_filter[i] := sample_filter[i] / sum;
  ext_lp_coeff := 1.0 - Exp(-2.0 * PI * 16000.0 / tick_hz);
  ext_hp_coeff := 1.0 - Exp(-2.0 * PI * 16.0 / tick_hz);
end;

procedure TM6581.Init(atick_hz, asound_hz: Integer; magnitude: Float);
var i: Integer;
begin
  sound_hz := asound_hz;
  tick_hz := atick_hz;
  sample_mag := magnitude;
  bus_value := 0;
  bus_decay := 0;
  pot[0] := 0; pot[1] := 0;
  for i := 0 to 2 do voice[i].Init;
  InitFilter;
  sample_phase := 0;
  sample_pos := 0;
  sample := 0;
  ext_lp := 0; ext_hp := 0;
  sample_buffer.fill(0);
  InitSampling;
  pins := 0;
end;

procedure TM6581.Reset;
var i: Integer;
begin
  bus_value := 0;
  bus_decay := $2000;
  for i := 0 to 2 do voice[i].Init;
  InitFilter;
  sample_phase := 0;
  sample := 0;
  sample_pos := 0;
  sample_buffer.fill(0);
  ext_lp := 0; ext_hp := 0;
  pins := 0;
end;

{--- voices ---}

procedure TM6581.SetCtrl(v: TM6581Voice; data: Integer);
var bit0: Integer;
begin
  if ((data and CTRL_TEST) <> 0) and ((v.ctrl and CTRL_TEST) = 0) then begin
    v.wav_accum := 0;
    v.sync := False;
    v.noise_delay := 0;
    v.noise_reset := $8000;
  end else if ((data and CTRL_TEST) = 0) and ((v.ctrl and CTRL_TEST) <> 0) then begin
    bit0 := 1 xor ((v.noise_shift shr 17) and 1);
    v.noise_shift := ((v.noise_shift shl 1) or bit0) and $007FFFFF;
    v.noise_reset := 0;
  end;
  if ((data and CTRL_GATE) <> 0) and ((v.ctrl and CTRL_GATE) = 0) then begin
    v.env_state := ENV_ATTACK;
    v.env_hold_zero := False;
    v.env_counter_compare := v.env_attack_add;
  end else if ((data and CTRL_GATE) = 0) and ((v.ctrl and CTRL_GATE) <> 0) then begin
    v.env_state := ENV_RELEASE;
    v.env_counter_compare := v.env_release_sub;
  end;
  v.ctrl := data;
end;

function TM6581.Triangle(v, vs: TM6581Voice): Integer;
var msb: Integer;
begin
  if (v.ctrl and CTRL_RINGMOD) <> 0 then msb := (v.wav_accum xor (not vs.wav_accum)) and $00800000
  else msb := v.wav_accum and $00800000;
  if (v.ctrl and CTRL_SAWTOOTH) <> 0 then msb := 0;
  if msb <> 0 then Result := ((not v.wav_accum) shr 11) and $0FFE
  else Result := (v.wav_accum shr 11) and $0FFE;
end;

function TM6581.Pulse(v: TM6581Voice): Integer;
begin
  if (v.ctrl and CTRL_TEST) <> 0 then Result := $0FFF else Result := v.pulse_output;
end;

procedure TM6581.VoiceTick(i: Integer);
var
  v: TM6581Voice;
  s, prev_accum, lfsr, feedback: Integer;
begin
  v := voice[i];
  v.sync := False;
  if (v.ctrl and CTRL_TEST) <> 0 then begin
    if v.noise_reset <> 0 then begin
      Dec(v.noise_reset);
      if v.noise_reset = 0 then v.noise_shift := $007FFFFF;
    end;
  end else begin
    if v.noise_delay <> 0 then begin
      Dec(v.noise_delay);
      if v.noise_delay = 1 then begin
        s := v.noise_shift;
        v.noise_latch := ((s shl 1) or (((s shr 22) xor (s shr 17)) and 1)) and $007FFFFF;
      end else
        v.noise_shift := v.noise_latch;
    end;
    prev_accum := v.wav_accum;
    v.wav_accum := (v.wav_accum + v.freq) and $00FFFFFF;
    if ((v.wav_accum and $00080000) <> 0) and ((prev_accum and $00080000) = 0) then v.noise_delay := 2;
    v.sync := ((v.wav_accum and $00800000) <> 0) and ((prev_accum and $00800000) = 0);
  end;
  { envelope generator }
  lfsr := v.env_counter;
  if lfsr <> RATE_COUNT_PERIOD[v.env_counter_compare and $0F] then begin
    feedback := ((lfsr shr 14) xor (lfsr shr 13)) and 1;
    lfsr := ((lfsr shl 1) or feedback) and $7FFF;
    v.env_counter := lfsr;
  end else begin
    v.env_counter := $7FFF;
    if v.env_state <> ENV_ATTACK then v.env_exp_counter := v.env_exp_counter + 1;
    if (v.env_state = ENV_ATTACK) or (v.env_exp_counter = v.env_exp_period) then begin
      v.env_exp_counter := 0;
      if v.env_hold_zero then exit;
      case v.env_state of
        ENV_ATTACK: begin
          v.env_cur_level := (v.env_cur_level + 1) and $FF;
          if v.env_cur_level = $FF then begin
            v.env_state := ENV_DECAY;
            v.env_counter_compare := v.env_decay_sub;
          end;
        end;
        ENV_DECAY:
          if v.env_cur_level <> v.env_sustain_level then
            v.env_cur_level := (v.env_cur_level - 1) and $FF;
        ENV_RELEASE: v.env_cur_level := (v.env_cur_level - 1) and $FF;
        ENV_FROZEN: v.env_cur_level := 0;
      end;
      case v.env_cur_level of
        $FF: v.env_exp_period := 1;
        $5D: v.env_exp_period := 2;
        $36: v.env_exp_period := 4;
        $1A: v.env_exp_period := 8;
        $0E: v.env_exp_period := 16;
        $06: v.env_exp_period := 30;
        $00: begin
          v.env_exp_period := 1;
          v.env_hold_zero := True;
        end;
      end;
    end;
  end;
end;

procedure TM6581.VoiceOutput(i: Integer);
var
  v, vs: TM6581Voice;
  wave, sm, noise, mask: Integer;
begin
  v := voice[i];
  vs := voice[(i + 2) mod 3];
  wave := v.ctrl shr 4;
  case wave and 7 of
    0: if wave <> 0 then sm := $0FFF else sm := v.wav_output;
    1: sm := Triangle(v, vs);
    2: sm := v.wav_accum shr 12;
    3: begin sm := Triangle(v, vs) and (v.wav_accum shr 12); sm := (sm shr 1) and (sm shl 1); end;
    4: sm := Pulse(v);
    5: begin sm := Triangle(v, vs) and Pulse(v); sm := (sm shr 1) and (sm shl 1); end;
    6: begin sm := (v.wav_accum shr 12) and Pulse(v); sm := (sm shr 1) and (sm shl 1); end;
  else
    sm := Triangle(v, vs) and (v.wav_accum shr 12) and Pulse(v);
    sm := (sm shr 1) and (sm shl 1);
  end;
  if (wave and 8) <> 0 then begin
    noise := (BIT(v.noise_shift, 22) shl 11) or (BIT(v.noise_shift, 20) shl 10) or
             (BIT(v.noise_shift, 16) shl 9) or (BIT(v.noise_shift, 13) shl 8) or
             (BIT(v.noise_shift, 11) shl 7) or (BIT(v.noise_shift, 7) shl 6) or
             (BIT(v.noise_shift, 4) shl 5) or (BIT(v.noise_shift, 2) shl 4);
    sm := sm and noise;
    if (wave and 7) <> 0 then begin
      mask := ($007FFFFF xor ((1 shl 22) or (1 shl 20) or (1 shl 16) or (1 shl 13) or
               (1 shl 11) or (1 shl 7) or (1 shl 4) or (1 shl 2))) or
              (BIT(sm, 11) shl 22) or (BIT(sm, 10) shl 20) or
              (BIT(sm, 9) shl 16) or (BIT(sm, 8) shl 13) or
              (BIT(sm, 7) shl 11) or (BIT(sm, 6) shl 7) or
              (BIT(sm, 5) shl 4) or (BIT(sm, 4) shl 2);
      v.noise_shift := v.noise_shift and mask;
      v.noise_latch := v.noise_latch and mask;
    end;
  end;
  v.wav_output := sm;
  if (v.wav_accum shr 12) >= v.pulse_width then v.pulse_output := $0FFF else v.pulse_output := 0;
end;

{--- filter ---}

function TM6581.FilterOutput(vi: Float): Float;
begin
  f_v_lp := f_v_lp - f_w0 * f_v_bp;
  f_v_bp := f_v_bp - f_w0 * f_v_hp;
  f_v_hp := f_v_bp * f_resonance_coeff - f_v_lp - vi;
  Result := 0;
  if (f_mode and FILTER_LP) <> 0 then Result := Result + f_v_lp;
  if (f_mode and FILTER_BP) <> 0 then Result := Result + f_v_bp;
  if (f_mode and FILTER_HP) <> 0 then Result := Result + f_v_hp;
end;

function TM6581.SampleOut(sample: Float): Boolean;
var
  frac, outv: Float;
  i, base: Integer;
begin
  ext_lp := ext_lp + ext_lp_coeff * (sample - ext_lp);
  ext_hp := ext_hp + ext_hp_coeff * (ext_lp - ext_hp);
  sample := ext_lp - ext_hp;
  sample_buffer[sample_pos] := sample;
  sample_buffer[sample_pos + M6581_SAMPLE_BUFFER_SIZE] := sample;
  sample_pos := (sample_pos + 1) and (M6581_SAMPLE_BUFFER_SIZE - 1);
  sample_phase := sample_phase + sound_hz;
  if sample_phase < tick_hz then exit(False);
  sample_phase := sample_phase - tick_hz;
  frac := sample_phase / sound_hz;
  base := sample_pos;
  outv := 0;
  for i := 0 to M6581_SAMPLE_BUFFER_SIZE - 2 do
    outv := outv + sample_filter[i] *
      (sample_buffer[base + i + 1] + frac * (sample_buffer[base + i] - sample_buffer[base + i + 1]));
  Self.sample := sample_mag * outv;
  Result := True;
end;

function TM6581.RegRead(apins: Integer): Integer;
var reg, data: Integer;
begin
  reg := apins and M6581_ADDR_MASK;
  case reg of
    M6581_POT_X, M6581_POT_Y: data := pot[reg - M6581_POT_X];
    M6581_OSC3RAND: data := (voice[2].wav_output shr 4) and $FF;
    M6581_ENV3: data := voice[2].env_cur_level;
  else
    data := bus_value;
  end;
  if (reg >= M6581_POT_X) and (reg <= M6581_ENV3) then begin
    bus_value := data;
    bus_decay := $2000;
  end;
  Result := (apins and not $FF0000) or ((data and $FF) shl 16);
end;

procedure TM6581.RegWrite(apins: Integer);
var
  reg, data: Integer;
  v: TM6581Voice;
begin
  reg := apins and M6581_ADDR_MASK;
  data := (apins shr 16) and $FF;
  bus_value := data;
  bus_decay := $2000;
  if reg < 21 then begin
    v := voice[reg div 7];
    case reg mod 7 of
      0: v.freq := (v.freq and $FF00) or data;
      1: v.freq := (data shl 8) or (v.freq and $00FF);
      2: v.pulse_width := (v.pulse_width and $0F00) or data;
      3: v.pulse_width := ((data and $0F) shl 8) or (v.pulse_width and $00FF);
      4: SetCtrl(v, data);
      5: begin
        v.env_attack_add := data shr 4;
        v.env_decay_sub := data and $0F;
        if v.env_state = ENV_ATTACK then v.env_counter_compare := v.env_attack_add
        else if v.env_state = ENV_DECAY then v.env_counter_compare := v.env_decay_sub;
      end;
      6: begin
        v.env_sustain_level := (data shr 4) * $11;
        v.env_release_sub := data and $0F;
        if v.env_state = ENV_RELEASE then v.env_counter_compare := v.env_release_sub;
      end;
    end;
    exit;
  end;
  case reg of
    21: if ((data xor f_cutoff) and 7) <> 0 then begin
          f_cutoff := (f_cutoff and $7F8) or (data and 7);
          SetFilterCutoff;
        end;
    22: begin
      f_cutoff := ((data shl 3) or (f_cutoff and 7)) and $FFFF;
      SetFilterCutoff;
    end;
    23: begin
      f_voices := data and 7;
      f_resonance := data shr 4;
      SetResonance;
    end;
    24: begin
      f_volume := data and $0F;
      f_mode := data shr 4;
      voice[2].muted := (f_mode and FILTER_3OFF) <> 0;
    end;
  end;
end;

function TM6581.Tick(apins: Integer): Integer;
var
  p, i: Integer;
  v: TM6581Voice;
  sum_f, sum_o, wav_out, env_out, smp: Float;
begin
  p := apins;
  if bus_decay > 0 then begin
    Dec(bus_decay);
    if bus_decay = 0 then bus_value := 0;
  end;
  for i := 0 to 2 do VoiceTick(i);
  for i := 0 to 2 do
    if voice[(i + 2) mod 3].sync and ((voice[i].ctrl and CTRL_SYNC) <> 0) then voice[i].wav_accum := 0;
  for i := 0 to 2 do VoiceOutput(i);
  if audio then begin
    sum_f := 0;
    sum_o := 0;
    for i := 0 to 2 do begin
      v := voice[i];
      wav_out := v.wav_output * (1.05 / 4095.0);
      env_out := v.env_cur_level * (1.0 / 255.0);
      if (f_voices and (1 shl i)) <> 0 then
        sum_f := sum_f + (wav_out - DCWAVE) * env_out + DCVOICE
      else if not v.muted then
        sum_o := sum_o + (wav_out - DCWAVE) * env_out + DCVOICE;
    end;
    smp := (sum_o + FilterOutput(sum_f) + DCMIXER) * f_volume * (1.0 / (15.0 * 4.5));
    if SampleOut(smp) then p := p or M6581_SAMPLE else p := p and not M6581_SAMPLE;
  end;
  if (p and M6581_CS) <> 0 then begin
    if (p and M6581_RW) <> 0 then p := RegRead(p)
    else RegWrite(p);
  end;
  pins := p;
  Result := p;
end;

end.
