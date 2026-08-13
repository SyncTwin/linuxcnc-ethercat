# Mitsubishi MR-J4-TM Servo Drives

The `mitsubishi` driver supports Mitsubishi Electric MR-J4 single-axis
servo drives with the TM (multi-network) interface running EtherCAT.

## Setup

In your XML file, you should have an entry somewhat like this:

```xml
<masters>
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
    <slave idx="0" type="MR-J4-TM" name="x"/>
  </master>
</masters>
```

See the [CiA 402](cia402.md) documentation for additional details
about how to configure CiA 402 devices in LinuxCNC.  At a minimum, you
will need to include the `cia402` HAL component.

## Devices

- [MR-J4-TM](https://www.mitsubishielectric.com/fa/products/drv/servo/)
  single-axis servo drive, EtherCAT network type.  Vendor ID
  `0x00000a1e`, product code `0x00000201`.

The identity above was read from a live MR-J4-20TM (revision
`0x00020002`) on our bench; we had no ESI file for this drive, so
everything the driver knows comes from the bus itself rather than from
vendor XML.  The drive supports CoE and FoE, and reports its mode set
in object 0x6502 (`0x03ad`: `pp`, `pv`, `tq`, `hm`, `csp`, `csv`,
`cst`).

Two things stand out compared to the other CiA 402 drives we have
looked at:

- **SDO-Info works and the object dictionary is unusually rich.**  The
  drive answers an SDO-Info scan with 809 objects — by far the largest
  dictionary of the four vendors on our bench (for comparison: Wecon
  answers with 109, Inovance with none).
- **The factory PDO mapping is fuller than usual.**  Mitsubishi maps
  its own vendor objects alongside the standard CiA 402 set:
  0x2d01..0x2d03 "Control DI", 0x2d11..0x2d13 "Status DO" and 0x2d20
  "Velocity limit value".

### Testing status

The MR-J4-TM has only been *identified* on a live bus (2026-08): its
vendor and product codes, PDO layout and supported-mode set were read
from real hardware, but the drive has **not been driven to OP with
this driver yet**.  The driver follows the same pattern as
bench-verified drivers for other vendors, but treat it as untested.

### Caveats

- **PDO limits are observed counts, not datasheet values.**  The drive
  assigns exactly one RxPDO and one TxPDO (0x1C12 and 0x1C13 each hold
  a single entry, 0x1600 and 0x1A00).  As shipped they carry 12 and 14
  entries respectively, and the driver uses those counts as its
  limits.  Without an ESI we could not confirm the true maxima, so
  treat the limits as a floor: enabling enough optional cia402
  features to outgrow them may or may not work on real hardware.
- **Distributed clocks are deliberately not preset.**  The drive
  reports a system time transmission delay of 0 ns and we have no ESI
  to read `AssignActivate` from, so the driver does not guess.  If
  your installation needs DC, configure it explicitly with a
  `<dcConf>` element on the slave.
- **Error code (0x603f) is not mapped.**  It is absent from the
  factory TxPDO, so `enable_error_code` is left off; turning it on
  grows the mapping past what the drive ships with, and that is
  exactly the kind of change that should be tested on hardware first.
- **Touch probes are not enabled.**  The full touch probe set
  (0x60b8..0x60bd) is present in the factory TxPDO, but probe support
  is untested on this hardware and a wrong mapping costs the
  transition to OP.

## Configuration

There are no Mitsubishi-specific `<modParam>` options; the standard
[`cia402` modParams](cia402.md) apply.  The driver enables the
following cia402 features on top of the base set: opmode selection,
CSP, CSV, CST, target torque, actual torque, and actual following
error.

## Pins

The driver exports the standard cia402 pin set, prefixed with
`lcec.<master>.<slave-name>.srv-`, for example:

- `srv-cia-controlword`, `srv-cia-statusword`
- `srv-opmode`, `srv-opmode-display`
- `srv-actual-position`, `srv-actual-velocity`, `srv-actual-torque`
- `srv-target-position`, `srv-target-velocity`, `srv-target-torque`
- `srv-actual-following-error`
- `srv-supported-modes`, `srv-supports-mode-{pp,pv,hm,csp,csv,cst}`

See the [CiA 402 documentation](cia402.md) for the full list and
semantics.
