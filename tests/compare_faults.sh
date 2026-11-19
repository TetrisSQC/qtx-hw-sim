#!/bin/sh
# Compare C reference and Pascal port on fault scenarios: tests/compare_faults.sh [frames] [machine]
F=${1:-200}; M=${2:-c64}
cd "$(dirname "$0")"
mkdir -p out
export PATH=/c/msys64/ucrt64/bin:$PATH
C=../../c64-hw-simulator-main/c64-hw-sim.exe
J=../hwsim_headless/index.js
export PINLEVEL=1   # the port at pin level (fast mode is checked against it separately)
if [ "$M" = c64 ]; then
  LIST=${LIST:-"healthy:  char:U5:removed basic:U3:removed cia1:U1:removed dram0:U21:removed pla18gnd:U17.18:gnd colorram:U6:removed vic:U19:removed"}
else
  LIST=${LIST:-"healthy:  ula:IC1:removed rom:IC5:removed ram0:IC6:removed lram5v:IC6.9:open"}
fi
for item in $LIST; do
  name=${item%%:*}; spec=${item#*:}
  args=""; [ -n "$spec" ] && args="--fault $spec"
  ( $C --machine $M --headless $F --text $args > out/$M-$name.c.txt 2>&1 ) &
  ( node --stack-size=4000 $J --machine $M --headless $F --text $args > out/$M-$name.js.txt 2>&1 ) &
done
wait
for item in $LIST; do
  name=${item%%:*}
  if diff <(grep -v "^emulated" out/$M-$name.c.txt) <(grep -v "^emulated" out/$M-$name.js.txt) >/dev/null; then echo "SAME  $name"; else echo "DIFF  $name"; fi
done
