# Omron NX EtherCAT Couplers

The `omron_nx` driver supports the Omron NX-ECC202, the EtherCAT coupler at
the head of a modular Omron NX I/O station.

## Setup

```xml
<masters>
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
    <slave idx="0" type="NX-ECC202" name="io"/>
  </master>
</masters>
```

Nothing about the station lineup goes into the XML.  The driver reads the
coupler's own PDO assignment over CoE while the slave is in PREOP and builds
the sync configuration and the HAL pins from what it finds.

## Why the lineup is not in the config

The coupler is the only EtherCAT slave on the wire.  The NX units clipped onto
it are not slaves; their process data is aggregated into the coupler's process
image.  The identity is therefore the same for every station: vendor ID
`0x00000083`, product code `0x000000a6`, no matter what is mounted.

Unlike an EL2008, where the device name implies the pinout, "NX-ECC202" says
nothing about the process image.  A driver with a hardcoded object list would
be right for one assembled station and silently wrong for every other, so the
map is read from the device instead:

- `0x1c12:00..N` -- assigned RxPDOs (outputs, `0x16xx`)
- `0x1c13:00..N` -- assigned TxPDOs (inputs, `0x1axx`)
- each `0x16xx` / `0x1axx:00..N` -- one u32 per mapping entry, packed as
  `(index << 16) | (subindex << 8) | bitlength`

If the coupler does not answer those reads, the driver refuses to load rather
than publish a plausible-looking pinout; use `type="generic"` with an explicit
map in that case.

## Padding entries

The input assignment is not just the modules.  On a live NX-ECC202 with three
16-point output units and one 4-point input unit, `0x1c13` assigns five PDOs,
in this order:

| PDO      | entries | contents                                | meaning              |
|----------|---------|-----------------------------------------|----------------------|
| `0x1bf8` | 2       | `0x3003:04` (128 bit), `0x3006:04` (128) | station diagnostics  |
| `0x1bff` | 1       | `0x2002:01` (8 bit)                      | station status       |
| `0x1bf4` | 1       | `0x0000:00` (8 bit)                      | alignment            |
| `0x1a0c` | 4       | `0x6060:01..04` (1 bit each)             | the input unit       |
| `0x1bf6` | 1       | `0x0000:00` (12 bit)                     | alignment            |

The two alignment entries map object `0x0000` and carry no data, but they take
up bits in the process image and must be declared in order.  Dropping the
first one shifts every input behind it by eight bits, and that does not fail
visibly: the station still reaches OP, and the pins simply read the wrong
terminals.  The driver passes every entry it read to the master, and creates
no HAL pin for the ones that carry no I/O.

## Pins

Pins are chosen by object index range:

- `0x6000..0x6fff` -- station inputs, `din-N`
- `0x7000..0x7fff` -- station outputs, `dout-N`
- everything else (alignment, `0x2002` status, `0x3xxx` diagnostics) is
  declared but not exposed

A one-bit entry becomes one pin.  A wider entry is treated as a packed word
and becomes one pin per bit, LSB first.  Numbering runs across the whole
station in assignment order, so the first output unit's first terminal is
`dout-0`.  The station above yields `dout-0`..`dout-47` and `din-0`..`din-3`.

Analog NX units are not supported.  A 16-bit mapping entry from a digital
output unit and one from an analog output unit are indistinguishable in the
mapping, and this driver assumes digital.  Stations with analog units need
`type="generic"`.

## Distributed clocks

Do not enable DC on this coupler.  With a `dcConf` it fails with
`AL 0x0034 DC Sync Timeout` and never leaves SAFEOP; freerun is what its ESI
describes, and digital I/O does not need the shared clock.

## Devices

- [NX-ECC202](https://industrial.omron.eu/en/products/NX-ECC202) EtherCAT
  coupler for the NX I/O series, vendor ID `0x00000083`, product code
  `0x000000a6`.

The NX-ECC201 and NX-ECC203 are the same family and would very likely work
with this driver, but their product codes have not been read off hardware
here and are not guessed.

### Testing status

The PDO map above was read from a live NX-ECC202 on 2026-08-21 with
`ethercat upload`, with the master idle and the slave in PREOP.  That station
has been running in production through a hand-written `type="generic"`
configuration built from the same map.

The driver's discovery has been checked against those recorded SDO answers
offline: it produces exactly the twelve mapping entries listed above, in the
same order, including both alignment entries, and 48 output plus 4 input pins.
