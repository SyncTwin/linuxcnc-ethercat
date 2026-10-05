#!/bin/bash
# run_a.sh -- UNPATCHED cia402 (PR #534 head), CSV 10 rev/s forward until 0x6064 wraps
# past +2^31, then stop. pos-fb is only read here (no position loop), so the
# 2^32-count jump of pos-fb is harmless on the shaft.
cd ~/unwrap/run
g(){ halcmd getp $1; }
./up.sh cia402_unw_a csv
(halsampler -t > trace_a.txt 2> halsampler_a.err &)
sleep 0.3
off(){
  halcmd setp vgen.in 0; sleep 1.0
  halcmd setp cia402.0.enable 0; sleep 0.5
  echo "T $(date +%s.%3N) disabled cw=$(g cia402.0.controlword) sw=$(g cia402.0.statusword) raw=$(g cia402.0.drv-actual-position) pos-fb=$(g cia402.0.pos-fb)"
  sleep 0.3; pkill -f halsampler; sleep 0.2
  halrun -U >/dev/null 2>&1
  echo "T $(date +%s.%3N) halrun unloaded; 0x6041=$(ethercat upload -p4 -t uint16 0x6041 0)"
}
trap off EXIT
for k in $(seq 200); do [ "$(g lcec.0.d4.slave-state-op)" = TRUE ] && break; sleep 0.1; done
sleep 1
echo "T $(date +%s.%3N) d4-op=$(g lcec.0.d4.slave-state-op)"
[ "$(g lcec.0.d4.slave-state-op)" = TRUE ] || exit 5
halcmd setp vgen.in 0; halcmd setp vgen.load 1; sleep 0.05; halcmd setp vgen.load 0
echo "T $(date +%s.%3N) start raw=$(g cia402.0.drv-actual-position) pos-fb=$(g cia402.0.pos-fb)"
halcmd setp cia402.0.enable 1
for k in $(seq 40); do [ "$(g cia402.0.stat-op-enabled)" = TRUE ] && break; sleep 0.05; done
echo "T $(date +%s.%3N) op=$(g cia402.0.stat-op-enabled) cw=$(g cia402.0.controlword) sw=$(g cia402.0.statusword) omd=$(g cia402.0.opmode-display)"
[ "$(g cia402.0.stat-op-enabled)" = TRUE ] || exit 1
halcmd setp vgen.in 10
t0=$(date +%s); prev=$(g cia402.0.drv-actual-position)
while :; do
  now=$(date +%s); a=$(g cia402.0.drv-actual-position)
  [ "$(g cia402.0.stat-fault)" = TRUE ] && { echo "T fault sw=$(g cia402.0.statusword)"; exit 2; }
  if [ $((now - t0)) -ge 3 ]; then
    v=$(g cia402.0.velocity-fb); python3 -c "import sys; sys.exit(0 if $v > 5 else 1)" || { echo "T no motion v=$v"; exit 3; }
  fi
  [ $((now - t0)) -ge 40 ] && { echo "T timeout"; exit 4; }
  if [ "$a" -lt 0 ] && [ "$prev" -gt 1073741824 ]; then
    sleep 0.3; echo "T $(date +%s.%3N) wrapped: raw $prev -> $a, now raw=$(g cia402.0.drv-actual-position) pos-fb=$(g cia402.0.pos-fb)"; break
  fi
  prev=$a; sleep 0.02
done
