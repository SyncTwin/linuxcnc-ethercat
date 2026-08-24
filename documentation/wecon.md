# Wecon VD3E Servo Drives

The `wecon` driver supports Wecon's VD3E single-axis EtherCAT servo
drive.

## Setup

In your XML file, you should have an entry somewhat like this:

```xml
<masters>
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
    <slave idx="0" type="VD3E" name="s"/>
  </master>
</masters>
```

See the [CiA 402](cia402.md) documentation for additional details
about how to configure CiA 402 devices in LinuxCNC.  At a minimum, you
will need to include the `cia402` HAL component.

Note the `refClockSyncCycles="-1"`; see the distributed-clocks caveat
below before changing it.

## Devices

- [VD3E](https://docs.we-con.com.cn/bin/view/Servo/) single-axis
  EtherCAT servo drive, vendor ID `0x00000eff`, product code
  `0x0d3e0001`.  CoE only (no FoE, no EoE).

The identity was read from live hardware and matches the vendor ESI
"Wecon VD3E EtherCAT Servo V1.15.0.xml" (revision `0x00000073`).

The related VD5 and VD5L drives share one product code (`0x0d510001`)
and differ only by revision, so supporting them needs revision
matching, not just another product code.  Neither has been on our
bench, and neither is supported by this driver yet.

### Testing status

The VD3E has been tested on live hardware: driven to OP on a
five-slave bus (three Inovance IS620N axes, the VD3E, and an Omron
NX-ECC202 coupler), where it runs the spindle of a 3-axis mill in
cyclic velocity mode.  Object 0x6502 reads `0x3AD` (`pp`, `pv`, `tq`,
`hm`, `csp`, `csv`, `cst`), and position, velocity, torque and
following-error feedback are live.

### Caveats

- **The drive assigns exactly one RxPDO and one TxPDO, and every
  mapping object holds at most 10 entries.**  Objects 0x1C12 and
  0x1C13 each hold a single entry (defaulting to 0x1701 and 0x1B01),
  and the mapping objects (0x1600, 0x1701, 0x1702 for outputs; 0x1A00,
  0x1B01 for inputs) accept at most 10 entries each.  This is a
  property of the device, not a tuning knob: a longer mapping is
  rejected and the slave never reaches OP.  The driver keeps within
  these limits; keep them in mind if you enable enough optional cia402
  features to need more PDO entries, or if you write your own PDO
  config for this drive.
- **The bus cycle time should be 1 ms.**  The vendor manual recommends 1 ms
  and does not guarantee stability below it, so a 500 us or 250 us
  application period is outside what this drive is specified for even
  if your master keeps up.
- **Objects 0x608F (position encoder resolution) and 0x6092 (feed
  constant) do not exist on this drive** — the SDO reply is "object
  does not exist", and neither appears in the vendor manual.  A
  master cannot learn the encoder resolution through CiA 402; it is
  2^23 counts per revolution (23-bit absolute encoder), confirmed by
  hand on the shaft.  The manual's own route is the manufacturer
  object **U0-52 = 0x201E:0x34**, which reports the number of encoder
  BITS (resolution = `1 << bits`).  It reads 23 on our bench — 8388608
  counts per revolution, matching the shaft measurement — and it is a
  UINT16, not a UINT32.  Feed constant has no substitute:
  only the electronic gear 0x6091 exists, and it defaults to 1:1
  (confirmed on the bench: both subindices read 1), so out of the box
  0x6064 carries raw encoder counts and all mm scaling is the master's
  job.
- **Drive temperature is not exposed in any object** — it is in neither
  the manual nor the dictionary walked with `ethercat sdos`
  — do not go looking for it.  DC-bus voltage is **U0-31 =
  0x201E:0x1F**, a UINT16; with mains applied it read 3157 on our
  bench, which puts the unit at 0.1 V (315.7 V, the expected rectified
  220 V).  That scale is *inferred* from the agreement, not stated by
  the manual — check it against a meter before showing volts to an
  operator.
- **0x603F carries a vendor code, not a CiA 402 one.**  Read as
  DECIMAL, its value is the digits of the code on the drive's own
  display: 34 is `Er.34` (motor overload), 82 is `A-82`.  Formatting
  the same value as hex points the operator at a different fault
  entirely — `0x22` would read as `Er.22`, a DC-bus overvoltage.  The
  full table of codes, causes and remedies is chapter 11.2 of the
  vendor manual.
- **Keep the firmware and the ESI in step.**  The ESI file states the
  firmware version it describes.  The manual points at **U2-04** for
  the drive's own version, but no manufacturer object is needed: the
  standard **0x100A** ("Manufacturer Software version") returns it as a
  string — `V1.15` on our bench, alongside `0x1009` = `VD3E V1.1.0.0.0`
  and revision `0x1018:03` = `0x73`, which is what the ESI declares.  A
  drive older than the ESI needs the vendor's firmware upgrade tool;
  1.15 is the latest at the time of writing.
- **SDO Info works on this drive.**  The whole object dictionary can be
  walked with `ethercat sdos`, which is not true of the Inovance or
  Schneider drives on the same bench, and the manufacturer groups are
  named there (`0x201E` = "U0 Group monitor", `0x2020` = "U2 Group
  Monitor", `0x2000`..`0x200D` = the `P0`..`P13` parameter groups).
  Useful when you need a vendor object and only have the manual's
  `Pxx-yy` / `Uxx-yy` numbering.
- **SDO writes must not use complete access.**  The drive NAKs
  complete-access SDO writes.  Other drives on the same bench (e.g. a
  Mitsubishi MR-J4) accept it, so this is per vendor, not per bus.
- **Distributed clocks converge only with master-to-slaves
  distribution (`refClockSyncCycles="-1"`).**  On the test bench,
  `dc-sync-diff` settled at about 103 ns after 40 s with `-1`; with a
  positive value the clocks did not converge at all.  The driver
  enables DC by default (`assignActivate` 0x300, SYNC0 at the
  application cycle time, from the ESI DC OpMode "DC", shifted by half
  the cycle); the drive also offers Free Run.  Override with a
  `<dcConf>` element if needed.  The half-cycle SYNC0 shift is the
  manual's recommendation — advance the PDI by 50% of the SYNC0 period
  — and their fault `E.101` ("ECAT sync error") names an unreasonable
  master SYNC Shift Time as its first cause.  Measured on the bench at
  a 1 ms period: with no shift, `dc-sync-diff` starts at 129 us and
  needs ~20 s to converge; with the half-cycle shift it is already
  within 1 ns at 10 s.  Both settle in the single-digit-ns range, so
  the shift buys startup convergence rather than a steadier lock, and
  no `E.101` appeared in either case.
- **Touch probe is not enabled.**  The probe objects (0x60B8..0x60BD)
  exist, and 0x60B8 even sits in the factory RxPDO, but the driver
  does not map them: they have not been run on hardware yet, and a
  wrong mapping costs the OP transition.

## Configuration

There are no Wecon-specific `<modParam>` options; the standard
[`cia402` modParams](cia402.md) apply.  The driver enables the
following cia402 features on top of the base set: opmode selection,
CSP, CSV, CST, target torque, actual torque, actual following error,
and the error code (0x603F ships in the factory TxPDO, so exporting it
costs nothing).

To override the default distributed-clock settings, use a `<dcConf>`
element on the slave as usual.

## Pins

The driver exports the standard cia402 pin set, prefixed with
`lcec.<master>.<slave-name>.srv-`, for example:

- `srv-cia-controlword`, `srv-cia-statusword`
- `srv-opmode`, `srv-opmode-display`
- `srv-actual-position`, `srv-actual-velocity`, `srv-actual-torque`
- `srv-target-position`, `srv-target-velocity`, `srv-target-torque`
- `srv-actual-following-error`, `srv-error-code`
- `srv-supported-modes`, `srv-supports-mode-{pp,pv,tq,hm,csp,csv,cst}`

See the [CiA 402 documentation](cia402.md) for the full list and
semantics.
