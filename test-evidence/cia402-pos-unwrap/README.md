# cia402 position across the 0x6064 rollover: hardware test

Test data for `fix(cia402): keep position continuous across the 0x6064 rollover`
(commit 233207e on top of a3d3cba, PR #534 head). Not for merging.

## Setup

- Drive: Inovance SV660N (ESI `SV660_1Axis_00912`, 0x100A `V1.0`, H01-00 9023,
  H01-01 9029), bus position 4, motor MS1H1-75B30CB, 23-bit encoder, no load
  on the shaft. 0x6091 = 1:1, so 0x6064/0x607A are raw encoder counts and
  2^31 counts = 256 motor turns. Drive software limit off (200A-02h = 0).
- IS620N at position 0 is configured only as the DC reference clock; it stays
  disabled. Other slaves are not configured.
- LinuxCNC 2.10.0~pre2 (uspace, PREEMPT_RT 6.12), IgH EtherCAT master 1.6.10,
  servo thread 1 ms.
- lcec and cia402 built from a3d3cba; the patched cia402 differs only in
  `src/cia402.c` (233207e). Modules were copied to the rtlib directory under
  temporary names (`lcec_unw`, `cia402_unw_a` = a3d3cba, `cia402_unw_b` =
  233207e) and removed after the test.
- pos-scale = velo-scale = 8388608 (units: motor turns).

## Runs

| run | module | mode | motion |
|---|---|---|---|
| a | a3d3cba (unpatched) | CSV, 10 rev/s | up through +2^31; pos-fb only read |
| b1 | 233207e (patched) | CSP, 5 rev/s | down through the wrap, 13 turns |
| b2 | 233207e (patched) | CSP, 5 rev/s | up through the wrap 13 turns, back down, then 250 turns to the start position |

The unpatched module was not run in CSP across the wrap: its target jumps
by 2^32 there.

b1 ended on the script's own stall check while settling 0.005 turn before
its target (`log_b1.txt`); the crossing is complete in the trace. The check
was relaxed for b2.

## Results (`python3 check.py`)

| | 0x6064 at the wrap | pos-fb step at the wrap | 0x607A step (mod 2^32) | max \|0x607A - 0x6064\| |
|---|---|---|---|---|
| a, unpatched | 2147467625 -> -2147415810 | -511.9900 turns | - | - |
| b1, patched, down | -2147463018 -> 2147462233 | -0.0050 | -41943 | 1297703 (0.155 turn) |
| b2, patched, up | 2147471173 -> -2147454308 | +0.0050 | +41943 | 1297564 (0.155 turn) |
| b2, patched, down | -2147464662 -> 2147460565 | -0.0050 | -41943 | same run |

0.0050 turn is one 1 ms cycle at 5 rev/s. The following error at the wrap is
the same as anywhere else at 5 rev/s. No fault bit in any run.

## Files

- `a_unpatched_csv.csv`, `b1_patched_csp.csv`, `b2_patched_csp.csv`: every
  servo cycle, from halsampler (`t_ms` is the sample number at 1 ms):
  0x6064, pos-fb, pos-cmd, 0x607A, controlword, statusword, velocity-fb,
  0x6061, and 0x607A - 0x6064 mod 2^32 for CSP.
- `*_wrap_window.csv`: 500 cycles each side of every 0x6064 wrap.
- `ethercat-conf.xml`, `body_csv.hal`, `body_csp.hal`, `up.sh`: the HAL setup.
- `run_a.sh`, `run_b1.sh`, `run_b2.sh`, `log_*.txt`: motion scripts and output.

To repeat: build lcec at the commit under test, copy `lcec.so` and `cia402.so`
into the rtlib directory as `lcec_unw.so` and `cia402_unw_a.so` /
`cia402_unw_b.so`, adjust the lcec_conf path in `up.sh`, then
`./run_a.sh` and `./run_b2.sh <0x6064 to return to>`.
