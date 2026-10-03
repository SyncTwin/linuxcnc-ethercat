# Omron NX-ECC202 through the generic MDP coupler driver (#497): gap analysis

Status: analysis only, no driver change.  Written on top of upstream PR #497
(`lcec_mdp_coupler.c` + `scripts/esi2coupler.py`) to answer whether the
SyncTwin `omron_nx` driver (branch `synctwin/v1.46`,
`src/devices/lcec_omron_nx.c`) can be replaced by an ESI-generated family
table.

**Short answer: not as #497 stands.**  The NX units' own process data fits
the MDP model, but the NX-ECC202 station image also carries coupler-level
PDOs (diagnostics, status, alignment) that #497 has no way to declare, and
dropping the alignment entries silently shifts every input by eight bits.

## Why no table was generated

`esi2coupler.py` needs the vendor ESI (Omron `NX-ECC20x` XML, with the NX
unit `<Module>` list).  It is not in this repository and could not be fetched
from this build environment.  Hand-writing a "generated" header or an ESI
fragment from memory would put unverified module idents and PDO indices into
a table that the driver hands to the master, so it was not done.  Everything
below is taken from `omron_nx` and its hardware captures (2026-08-21 SDO
upload, 2026-09-06 OP run: 3x DO16 + 1x DI4).

## omron_nx feature -> #497 equivalent

| omron_nx (synctwin/v1.46) | #497 MDP coupler | Status |
|---|---|---|
| Identity `0x83` / `0xa6`, type `NX-ECC202` | `vid`/`pid` from ESI `<Device>`; type name = `--family` | OK once ESI is available |
| Lineup **read from the device** (0x1c12/0x1c13 + 0x16xx/0x1axx) in PREOP | Lineup **declared** in XML: `<subModule id ident name>` | Different model; user must list units and their idents |
| Unit data at 0x6000/0x7000 + 0x20 per slot | `slot_index_incr` from ESI `SlotIndexIncrement` + `DependOnSlot` | Fits (0x20 checked for DO16/DI only) |
| Unit PDO index per slot (input 0x1a0c for slot 4) | `slot_pdo_incr` from ESI `SlotPdoIncrement` | Fits if ESI declares it (4 implied by capture) |
| Coupler TxPDO 0x1bf8 (0x3003:04, 0x3006:04, 2x128 bit) | none: generator reads only `<Module>` PDOs, not the coupler `<Device>` PDOs | **Missing** |
| Coupler TxPDO 0x1bff (0x2002:01 status, 8 bit) | none | **Missing** |
| Alignment PDOs 0x1bf4 (8 bit) before units, 0x1bf6 (12 bit) after | `padding` flag exists per entry, but only inside module PDOs; nothing places station-level gap PDOs whose width depends on the whole lineup | **Missing** (critical: wrong bits, no error) |
| `slot<S>.dout-<b>` / `slot<S>.din-<b>`, S 1-based | `<name>-dout-<n>` / `<name>-din-<n>`, name from `<subModule name>`, id 0-based | Renamed; `name="slot1"` gives `slot1-dout-0` (dash, not dot) |
| 1-bit entry -> 1 pin; wide entry -> 1 pin per bit, LSB first | same (`_packed` helpers for 8/16/32 bit) | OK |
| Pins only for 0x6000..0x7fff | `io` flag per PDO, same 0x6000..0x7fff rule | OK |
| `coupler-io-active`, `slot<S>.io-active` from 0x3006:04 | no coupler-level pins at all | **Missing** |
| Analog units refused (ambiguous in discovered map) | AIN/AOUT from ESI `ModuleClass`, 16-bit values | Better in #497 (ESI removes the ambiguity) |
| No 0xF030 write | writes 0xF030 if ESI `DownloadModuleIdentList` | Unknown for NX-ECC202; needs ESI + hardware test |
| No DC (AL 0x0034 with dcConf) | no DC configured | OK |
| No 0x1c12/0x1c13 writes | default none; `EXPLICIT_SM_ASSIGN` / `NO_PDO_ASSIGN` quirks | OK (registration with quirks = 0) |
| Up to 16 PDOs per direction | `LCEC_MAX_PDO_INFO_COUNT` 16 per sync | Same limit; coupler PDOs would count against it |

## What #497 would need for the NX-ECC202

1. **Coupler-level PDOs**: emit the coupler `<Device>`'s own default
   `TxPdo`/`RxPdo` (0x1bf8, 0x1bff) into the family table and have
   `lcec_mdp_build_syncs()` place them before the slot PDOs, in the order the
   coupler assigns them.
2. **Station alignment PDOs**: a family hook that inserts the 0x1bf4/0x1bf6
   gap entries with the bit width the coupler computes (8 bit before the unit
   inputs, 12 bit after in the captured station).  The rule behind the widths
   is not documented here; without it the safe route is a quirk that reads
   0x1c12/0x1c13 back in PREOP and uses the slave's own assignment, i.e. what
   `omron_nx` does today.
3. **Coupler-level pins**: a way to register pins outside any slot
   (`coupler-io-active`, per-slot `io-active` from 0x3006:04).
4. **The Omron ESI** in `scripts/esi/` and a `check-generated-headers.sh`
   entry, plus a hardware run to settle `DownloadModuleIdentList` / 0xF030.

Until then `omron_nx` stays the driver for NX-ECC202 stations.
