# Manual audit: Mitsubishi MR-J4-TM (EtherCAT) vs. `lcec_mitsubishi.c`

Audit date: 2026-10-03. Driver at `synctwin/v1.46` (`src/devices/lcec_mitsubishi.c`, CiA 402 class `src/devices/lcec_class_cia402*.{c,h}`). Driver code is unchanged by this audit.

Reference: Mitsubishi *MR-J4-_TM_ Servo Amplifier Instruction Manual*, SH(NA)030193-E (2016-06), 604 pages, read cover to cover. Page numbers below are PDF pages, with the printed page in brackets. Only facts are recorded here, not text from the manual.

**Scope limit.** SH-030193 is the general hardware manual shared by the EtherCAT, EtherNet/IP and PROFINET variants. The EtherCAT details are in a separate manual, *MR-J4-_TM_ … (EtherCAT)* SH(NA)030208, which we do not have (p. 8 [A-7]). That covers the object dictionary, PDO mapping limits, DC/Sync0, minimum cycle time, ESM behaviour and the bit-level controlword/statusword. Rows marked "not in manual" stay open until SH-030208 or the ESI is available.

## 1. Comparison table

| # | Topic | Manual (page) | Driver (file:line) | Verdict |
|---|---|---|---|---|
| 1 | EtherCAT slave hardware | The network interface is a plug-in HMS Anybus CompactCom M40 module, ABCC-M40-ECT (AB6916-C or -B) (p. 19 [1-2]) | Not mentioned; `lcec_mitsubishi.c:19-34` describes the drive as a single unit | Gap (documentation only) |
| 2 | Vendor / product ID | Not in manual | `lcec.h:72` VID 0x00000a1e; `lcec_mitsubishi.c:80` PID 0x00000201, read from a live drive | Not verifiable from this manual |
| 3 | ESI requirement | ESI is needed when the amplifier firmware is ≥ B0 and the module firmware is ≤ 1.11.01, because Get OD List then returns only 710 objects (p. 19 [1-2]) | `lcec_mitsubishi.c:25-33`: no ESI available; SDO-Info returned 809 objects | Consistent (809 > 710 suggests a newer module). No impact on IgH, which does not need the OD list |
| 4 | Supported modes | PA01 `___0` (default) or `___1` gives csp/csv/cst + hm; `___2` gives pp/pv/tq + hm. **Both groups are never available at the same time** (p. 154 [5-15]) | `lcec_mitsubishi.c:34` reads 0x6502 = 0x3AD (all seven); `:108-112` enables csp, csv, cst | Matches for the default PA01. The driver comment does not mention the group restriction |
| 5 | Position unit in cyclic sync | Pulse only (PT01 `_3__`); anything else raises AL.37 (p. 212 [5-73]). Encoder resolution is 4 194 304 pulse/rev (p. 18 [1-1]) | Not handled; the scale is left to the HAL configuration | Matches (no conflict), but undocumented |
| 6 | Electronic gear 0x6091 | Must be 1/1 in cyclic sync, otherwise AL.37 (p. 157 [5-18]) | Not written | Matches. Do not expose a gear modParam for csp |
| 7 | DC / Sync0 | Not in manual (SH-030208) | `lcec_mitsubishi.c:48-51`: DC deliberately not preset, no `<dcConf>` | **Open: prime OP suspect** (see §3, H1) |
| 8 | PDO assignment / limits | Not in manual | `lcec_mitsubishi.c:36-42`, `:104-105`: limits 12/14 taken from the factory map | Open. The class rewrites 0x1600/0x1A00 with its own, shorter list (see §2) |
| 9 | RxPDO content | Factory RxPDO includes 0x2D01–0x2D03 (Control DI) and 0x2D20 (velocity limit for tq/cst; PT67 default 500.00 r/min) (p. 228 [5-89]) | Class maps 0x6040, 0x6060, 0x607A, 0x60FF, 0x6071 (`lcec_class_cia402.c:223-236` plus the enables at `lcec_mitsubishi.c:108-112`) | **Diverges**: 0x2D20 and Control DI are dropped. In cst the speed limit then stays at the PT67 default of 500 r/min |
| 10 | TxPDO content | Factory TxPDO includes Status DO 0x2D11–0x2D13 and touch probe (driver comment) | Class maps 0x6041, 0x6061, 0x6064, 0x606C, 0x6077, 0x60F4 (`lcec_class_cia402.c:269-283`, `lcec_mitsubishi.c:108-114`) | Diverges from factory (by design). 0x603F is not mapped (`lcec_mitsubishi.c:53-56`) |
| 11 | Following error 0x6065/0x6066 | Active in pp **and csp**, sets statusword bit 13. Default window conflicts inside the manual: 0 or 0x00C00000 pulse (p. 144 [5-5] vs p. 192-193 [5-53..54]); 0xFFFFFFFF disables it | Optional SDO pins exist in the class (`lcec_class_cia402_opt.h:528-529`), not enabled | Open. Read the actual value on the drive |
| 12 | Quick stop / forced stop decel 0x6085 | Equals PC24, default 100 ms. Also used on network loss and EM2 (p. 186 [5-47]; p. 107 [3-28]) | Not exposed | Gap (low) |
| 13 | Torque limits 0x60E0/0x60E1 | Equal PA11/PA12, default 1000.0 %; 0 means no torque (p. 161-162 [5-22..23]) | Not exposed. The class only knows 0x6072 (`lcec_class_cia402_opt.h:89`), which the manual does not list | Gap |
| 14 | Homing 0x6098 | PT45, default 37 (data set). Supported: −1…−11 (vendor), −33…−43, CiA 3–8, 11, 12, 19–24, 27, 28, 33–35, 37. Other values raise AL.37 (p. 218-219 [5-79..80]) | `enable_hm` is not set | Matches (hm off). If enabled later, validate the method against this list |
| 15 | Network-loss behaviour | Master power-off while in OP raises AL.86.1; communication loss triggers forced stop deceleration (p. 132 [4-11]; p. 110-111 [3-31..32]) | `lcec_mitsubishi.c:130,140`: no I/O outside OP | Matches |
| 16 | Alarm reset classes | AL.86 clears with fault reset or a power cycle, not with a communication reset. AL.37 clears with a communication reset (ESM re-init) or a power cycle, **not** with fault reset. AL.84/85 (network module) clear only with a power cycle (p. 293-295 [8-4..8-6]) | Not documented | Gap (documentation) |
| 17 | Test switch SW1-1 | When ON, the network is blocked for this axis and every axis downstream; with PA01 `___0` it also raises AL.37 (p. 135 [4-14]) | Not documented | Gap (bench checklist) |
| 18 | Node address | SW2/SW3 rotary switches; 00h means PN01 is used (EtherCAT only, power cycle) (p. 129 [4-8]; p. 230 [5-91]) | Not used (position addressing) | No conflict |
| 19 | Servo-on preconditions | Main power present (else AL.E9), EM2 closed (else AL.E6), LSP/LSN closed or PD01 auto-on (else AL.99), STO wired or shorting plug in CN8; servo-on accepted 3–4 s after main power (p. 97-104 [3-18..3-25]; p. 337 [11-2]) | Not documented | Gap (bench checklist) |
| 20 | No motor on the bench | Without a motor/encoder: AL.16 at power-on. Motor-less mode PC05 `___1` suppresses AL.16/1E/1F/20/21/25/92/9F (p. 138-139 [4-17..18]) | `lcec_mitsubishi.c:21-22`: "No motor on our bench" | **Relevant**: the bench drive sat in AL.16 unless PC05 was set |
| 21 | Display codes | `Ab` init not finished, `AC` init in progress, `AA` communication lost, `b/C/d` ready-off / ready-on / servo-on (p. 133 [4-12]; p. 302 [8-13]) | Not documented | Gap (diagnostics) |
| 22 | Torque polarity | PC29 `x___` (default 1) ignores PA14 for 0x6071/0x6074/0x6077/0x60E0/0x60E1 in torque mode (p. 189 [5-50]) | Not documented | Gap: cst direction can differ from csp/csv when PA14 = 1 |

## 2. What the driver sends during PREOP→SAFEOP (derived from the code)

- SM2 output, PDO 0x1600: 0x6040:00/16, 0x6060:00/8, 0x607A:00/32, 0x60FF:00/32, 0x6071:00/16. That is 5 entries.
- SM3 input, PDO 0x1A00: 0x6041:00/16, 0x6061:00/8, 0x6064:00/32, 0x606C:00/32, 0x6077:00/16, 0x60F4:00/32. That is 6 entries.
- No DC, default SM watchdog, no startup SDOs except reading 0x6502 (`lcec_class_cia402.c:435`).

IgH reconfigures 0x1C12/0x1C13 and 0x1600/0x1A00 over SDO in PREOP. Whether the Anybus module allows the assignment or mapping to be rewritten is not stated in SH-030193.

## 3. Hypotheses: why the drive did not reach OP (ranked)

| # | Hypothesis | Evidence | How to check |
|---|---|---|---|
| H1 | **The drive requires DC Sync0 for the cyclic synchronous modes, and the driver configures none**, so SAFEOP→OP is refused (AL status 0x0030/0x002C or similar) | PA01 default selects csp/csv/cst for EtherCAT (p. 154); `lcec_mitsubishi.c:48-51` sets no DC; the manual warns about gains at cycles ≥ 2 ms (p. 238 [6-5]), which implies a synchronized cycle | Read the AL status code (`ethercat slaves -v`, 0x0134). Retry with `<dcConf assignActivate="300" sync0Cycle="*1" .../>`. Get AssignActivate and the allowed cycles from the ESI or SH-030208 |
| H2 | PDO remap is rejected (fixed mapping, or 0x6060/0x60FF/0x6071 not allowed in 0x1600), so PREOP→SAFEOP fails | The factory map is a single fixed-looking 12/14-entry PDO; the manual says nothing about flexible mapping | Check the IgH log for an SDO abort on 0x1C12/0x1600. Retry with the factory map (no reassignment) |
| H3 | Bench state: **no motor connected, so AL.16**; the drive was also in alarm or held by AL.37 | `lcec_mitsubishi.c:21-22`; p. 127 [4-6], p. 293 [8-4] | Read the 7-segment display. Set PC05 `___1` (motor-less) and power-cycle |
| H4 | SW1-1 left ON after an MR Configurator2 session blocks the network (and raises AL.37 with PA01 `___0`) | p. 135 [4-14], p. 302 [8-13] | Display shows `b##.` (with a dot); set SW1-1 OFF and power-cycle |
| H5 | AL.37 after parameter writes cannot be cleared by fault reset, only by an ESM re-init or a power cycle; a stuck AL.37 looks like "won't enable" | p. 293 [8-4] | Check the display and the alarm number |
| H6 | Network module firmware/ESI mismatch (≤ 1.11.01 with amplifier ≥ B0) | p. 19 [1-2] | Read the module firmware (SII/0x100A) and get the matching ESI |

H1 and H2 match how the drive behaved: identity and PDO map read fine in PREOP, then it stopped. Check them first.

## 4. Proposed changes (priority order; not applied)

| Priority | Change | Where |
|---|---|---|
| P1 | Get SH(NA)030208 and the ESI (Mitsubishi / HMS AB6916). Confirm the DC requirement, AssignActivate, allowed cycles and PDO mapping rules | — |
| P1 | On the bench: record the AL status code when OP fails; test with `<dcConf assignActivate="300">` and a sync0 cycle equal to the servo period | XML / bench |
| P1 | If DC is required, preset it in the driver (as `lcec_leadshine_stepper.c:238-241` sets AssignActivate) and document the allowed cycle times | `lcec_mitsubishi.c:48-51`, type list `:80` |
| P2 | If remapping is restricted, keep the factory PDO (map 0x2D01–0x2D03/0x2D11–0x2D13/0x2D20) instead of the class-built list | `lcec_mitsubishi.c:103-119` |
| P2 | Expose 0x2D20 (velocity limit in cst, default 500 r/min) as a pin or modParam when cst is enabled | `lcec_mitsubishi.c:111` |
| P2 | Expose 0x60E0/0x60E1 (torque limits) and 0x6085 (quick stop time) as SDO pins | class or driver |
| P3 | Document the bench checklist in the driver header or DEVICES.md: SW1-1 off, EM2/LSP/LSN/STO, PC05 motor-less without a motor, display codes, which reset clears which alarm | `lcec_mitsubishi.c:19-56`, `documentation/DEVICES.md:255` |
| P3 | Document the cyclic-sync constraints: pulse units (4 194 304/rev), gear 1/1, PA01 selects one mode group, PC29 torque polarity | same |
| P3 | Map 0x603F once the mapping rules are known, for alarm visibility | `lcec_mitsubishi.c:53-56` |
