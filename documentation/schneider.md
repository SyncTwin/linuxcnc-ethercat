# Schneider Electric Lexium 28 (LXM28E) Servo Drives

The `schneider` driver supports Schneider Electric's Lexium 28
(LXM28E) single-axis EtherCAT servo drive.

## Setup

In your XML file, you should have an entry somewhat like this:

```xml
<masters>
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
    <slave idx="0" type="LXM28E" name="x"/>
  </master>
</masters>
```

See the [CiA 402](cia402.md) documentation for additional details
about how to configure CiA 402 devices in LinuxCNC.  At a minimum, you
will need to include the `cia402` HAL component.

## Devices

- [LXM28E](https://www.se.com/ww/en/product-range/62092-lexium-28/)
  single-axis EtherCAT servo drive, vendor ID `0x0800005a`, product
  code `0x00096030`.

The identity (vendor ID, product code, revision `0x00030006`), the PDO
map and the supported-mode object were all read from a live LXM28E on
our bench.  No ESI file was available for this drive, and it does not
answer SDO-Info (see caveats), so those direct reads are the only
source for the facts on this page.

### Testing status

The LXM28E has only been *identified* on a live bus: its vendor and
product codes, PDO layout and mode set were read from real hardware,
but it has **not been driven to OP with this driver yet**.  The driver
follows the pattern of drives that are bench-verified, so it is
expected to work, but treat it as untested.

### Caveats

- **SDO-Info is not supported.**  Unlike the Mitsubishi drive on the
  same bench, the LXM28E does not answer SDO-Info requests: the object
  dictionary reads back empty, so an EtherCAT master cannot enumerate
  it over the wire.  Individual SDO uploads/downloads work normally.
- **The mode set differs from most CiA 402 servo drives.**  Object
  0x6502 reads `0x000000ed`: `pp`, `pv`, `tq`, `hm`, `ip` and `csp` —
  there is **no csv and no cst**.  The Inovance, Wecon and Mitsubishi
  drives on the same bench all report `0x000003ad`, so it is tempting
  to assume that set is universal; it is not.  This driver enables
  `csp` only among the cyclic modes, because enabling csv/cst would
  map objects the hardware does not claim to implement.
- **PDO entry limits are observed counts, not documented maxima.**
  The drive ships with one RxPDO and one TxPDO (0x1600/0x1A00, a
  single entry each in 0x1C12/0x1C13), carrying 5 and 6 entries
  respectively.  The map is pure CiA 402 with no vendor-specific
  objects.
- **Distributed clocks are not preset.**  With no ESI file there is no
  AssignActivate value to trust, so the driver does not guess one.
  Use a `<dcConf>` element in the XML to configure DC.  The drive
  reports a DC system time transmission delay of 1140 ns.

## Configuration

There are no Schneider-specific `<modParam>` options; the standard
[`cia402` modParams](cia402.md) apply.  The driver enables the
following cia402 features on top of the base set: opmode selection,
CSP, actual torque, and actual following error.

## Pins

The driver exports the standard cia402 pin set, prefixed with
`lcec.<master>.<slave-name>.srv-`, for example:

- `srv-cia-controlword`, `srv-cia-statusword`
- `srv-opmode`, `srv-opmode-display`
- `srv-actual-position`, `srv-actual-velocity`, `srv-actual-torque`
- `srv-target-position`
- `srv-actual-following-error`
- `srv-supported-modes`

See the [CiA 402 documentation](cia402.md) for the full list and
semantics.
