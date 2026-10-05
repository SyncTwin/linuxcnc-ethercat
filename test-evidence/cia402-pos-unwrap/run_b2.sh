#!/bin/bash
# run_b.sh <home_raw> -- PATCHED cia402, CSP 5 rev/s: up through the wrap,
# back down through it, then to the original raw position.
cd ~/unwrap/run
HOME_RAW=$1
g(){ halcmd getp $1; }
./up.sh cia402_unw_b csp
(halsampler -t > trace_b.txt 2> halsampler_b.err &)
off(){
  halcmd setp cia402.0.enable 0; sleep 0.5
  echo "T $(date +%s.%3N) disabled cw=$(g cia402.0.controlword) sw=$(g cia402.0.statusword) raw=$(g cia402.0.drv-actual-position) pos-fb=$(g cia402.0.pos-fb)"
  sleep 0.3; pkill -f halsampler; sleep 0.2
  halrun -U >/dev/null 2>&1
  echo "T $(date +%s.%3N) halrun unloaded; 0x6041=$(ethercat upload -p4 -t uint16 0x6041 0) 0x6064=$(ethercat upload -p4 -t int32 0x6064 0)"
}
trap off EXIT
for k in $(seq 200); do [ "$(g lcec.0.d4.slave-state-op)" = TRUE ] && break; sleep 0.1; done
sleep 1
[ "$(g lcec.0.d4.slave-state-op)" = TRUE ] || exit 5
halcmd setp pgen.maxv 5
p0=$(g cia402.0.pos-fb)
halcmd setp pgen.in $p0; halcmd setp pgen.load 1; sleep 0.05; halcmd setp pgen.load 0
echo "T $(date +%s.%3N) start raw=$(g cia402.0.drv-actual-position) pos-fb=$p0 pos-cmd=$(g pgen.out) tgt=$(g cia402.0.drv-target-position)"
halcmd setp cia402.0.enable 1
for k in $(seq 40); do [ "$(g cia402.0.stat-op-enabled)" = TRUE ] && break; sleep 0.05; done
echo "T $(date +%s.%3N) op=$(g cia402.0.stat-op-enabled) cw=$(g cia402.0.controlword) sw=$(g cia402.0.statusword) omd=$(g cia402.0.opmode-display)"
[ "$(g cia402.0.stat-op-enabled)" = TRUE ] || exit 1
goto(){
  local tgt=$1 t0=$(date +%s) lim
  lim=$(python3 -c "print(int(abs($tgt-($(g cia402.0.pos-fb)))/5)+6)")
  halcmd setp pgen.in $tgt
  while :; do
    [ "$(g cia402.0.stat-fault)" = TRUE ] && { echo "T fault sw=$(g cia402.0.statusword)"; exit 2; }
    [ "$(g cia402.0.stat-op-enabled)" = TRUE ] || { echo "T dropped sw=$(g cia402.0.statusword)"; exit 2; }
    python3 -c "import sys; sys.exit(0 if abs($tgt-($(g cia402.0.pos-fb)))<0.01 else 1)" && break
    if [ $(( $(date +%s) - t0 )) -ge 2 ]; then python3 -c "import sys; sys.exit(0 if abs($(g cia402.0.velocity-fb))>0.5 or abs($tgt-($(g pgen.out)))<0.5 else 1)" || { echo "T no motion"; exit 3; }; fi
    [ $(( $(date +%s) - t0 )) -ge $lim ] && { echo "T timeout to $tgt"; exit 4; }
    sleep 0.05
  done
  echo "T $(date +%s.%3N) at $tgt raw=$(g cia402.0.drv-actual-position) pos-fb=$(g cia402.0.pos-fb) tgt=$(g cia402.0.drv-target-position) sw=$(g cia402.0.statusword)"
}
goto $(python3 -c "print($p0+13)")
goto $p0
raw=$(g cia402.0.drv-actual-position)
goto $(python3 -c "import ctypes; print($(g cia402.0.pos-fb)+ctypes.c_int32($HOME_RAW-($raw)).value/8388608)")
