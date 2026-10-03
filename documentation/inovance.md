# Inovance IS620N and SV660 Servo Drives

The `inovance` driver supports Inovance's IS620N and SV660 single-axis
EtherCAT servo drives.

## Setup

In your XML file, you should have an entry somewhat like this:

```xml
<masters>
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
    <slave idx="0" type="IS620N" name="x"/>
    <slave idx="1" type="IS620N" name="y"/>
    <slave idx="2" type="IS620N" name="z"/>
  </master>
</masters>
```

See the [CiA 402](cia402.md) documentation for additional details
about how to configure CiA 402 devices in LinuxCNC.  At a minimum, you
will need to include the `cia402` HAL component.

## Devices

- [IS620N](https://www.inovance.com/) 1-axis EtherCAT servo drive,
  product code `0x000c0108`.
- [SV660](https://www.inovance.com/) 1-axis EtherCAT servo drive,
  product code `0x000c010d`.  Sold as SV660N in the EtherCAT variant.

Both are tested on hardware.  IS620N: 3 drives on one bus, all reaching
OP and running a mill in CSP under load.  SV660: one drive reaching OP
and turning 30 revolutions each way in CSP at 60 rpm, returning to the
start position.  Both report `0x3ad` in 0x6502 (pp, pv, tq, hm, csp,
csv, cst) and share one parameter architecture.

I had no ESI file for either drive, so everything here was read off the
bus.

### Caveats

- These drives only assign one PDO per direction: 0x1600 for RxPDO and
  0x1A00 for TxPDO.  0x1C13:01 takes either 0x1A00 (variable) or one of
  the fixed maps 0x1B01..0x1B04.  There is no 0x1A01; writing it is an
  SDO error and the slave stays out of OP.
- Both 0x1600 and 0x1A00 hold 10 entries at most, which is what
  `rxpdolimit` and `txpdolimit` are set to.  Neither drive answers SDO
  information requests (`Enable SDO Info: no` in `ethercat slaves -v`),
  so `ethercat sdos` returns nothing and the object description is not
  readable; reading a sub-index above the current entry count aborts
  with 0x06090011 as well, so the count in :00 is all a read gives you.
  The limit was measured instead: with :00 set to 0, sub-indices 0x01
  through 0x0A accept a mapping, and 0x0B aborts with 0x06090011
  ("subindex does not exist").  Same on IS620N and SV660.
- Match the whole product code.  Inovance uses vendor ID 0x00100000
  across drive families, and the IS620N and SV660 codes differ in the
  last byte only.
- DC is on by default (`assignActivate` 0x300, SYNC0 at the
  application cycle time).  The reference clock lives on the first
  DC-capable slave in the chain, so if some other DC-capable device
  sits ahead of the drives, it needs DC configured too or the clocks
  never settle.

## Configuration

There are no Inovance-specific `<modParam>` options; the standard
[`cia402` modParams](cia402.md) apply and are passed to the cia402
class.  The driver turns on opmode selection, CSP, CSV, CST, target
torque, actual torque, actual following error, digital inputs
(0x60FD) and error code (0x603F).  That is 5 RxPDO entries (13 bytes)
and 8 TxPDO entries (23 bytes), inside the 10-entry limit of 0x1600
and 0x1A00.

### Homing mode (HM)

CiA 402 homing mode is off by default.  Turn it on per drive with:

```xml
<slave idx="0" type="SV660" name="x">
  <modParam name="enableHM" value="true"/>
</slave>
```

This adds the standard cia402 homing pins: `srv-home-method` (0x6098),
`srv-home-velocity-fast` (0x6099:01) and `srv-home-velocity-slow`
(0x6099:02).  The cia402 class writes these objects with SDO requests
when the pins change, not through PDOs, so the PDO mapping does not
grow: still 5 RxPDO and 8 TxPDO entries.  The homing acceleration
(0x609A) can be added with `enableHomeAccel`, also over SDO.  To home,
set `srv-opmode` to 6 and use the controlword/statusword pins of the
`cia402` component.  `homeMethod` and `homeOffset` modParams write
0x6098 and 0x607C once at startup.

Homing mode is not yet tested on hardware with these drives.

Use `<dcConf>` on the slave to override the distributed-clock defaults.

Vendor parameters ("H codes" in the Inovance manuals) are readable and
writable over CoE, and can be set at startup with `<sdoConfig>`:
`Hgg-nn` is index `0x2000 + 0xgg`; the subindex is the decimal
parameter number `nn` plus one, written in hex.  H02-01 is
`0x2002:02`, H02-10 is `0x2002:0B`, H0E-01 is `0x200E:02`.

## Pins

The driver exports the standard cia402 pin set, prefixed with
`lcec.<master>.<slave-name>.srv-`.  See the [CiA 402
documentation](cia402.md) for the list.
