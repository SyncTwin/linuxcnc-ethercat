//
//    Copyright (C) 2026 SyncTwin <info@synctwin.ru>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

/// @brief Driver for Wecon CiA 402 servo drives (VD3E).
///
/// Single-axis EtherCAT servo drive, CoE only (no FoE, no EoE).
/// Identity read from live hardware and confirmed against the vendor ESI
/// "Wecon VD3E EtherCAT Servo V1.15.0.xml":
///   - VD3E, product code 0x0d3e0001, revision 0x00000073
///
/// PDO layout: the drive assigns exactly ONE RxPDO and ONE TxPDO —
/// 0x1c12 and 0x1c13 each hold a single entry, defaulting to 0x1701 and
/// 0x1b01. The mapping itself is variable (ESI declares PdoAssign and
/// PdoConfig), and every mapping object holds at most 10 entries
/// (subindex 0 default 0x0a). That limit is a property of the device, not
/// a tuning knob: a longer mapping is rejected and the slave never
/// reaches OP. Measured on the bench, not read out of the ESI: raising the
/// declared default did not help. Available mapping objects are
/// 0x1600, 0x1701, 0x1702 for outputs and 0x1a00, 0x1b01 for inputs.
///
/// Cycle time: the vendor manual recommends 1 ms and does not guarantee
/// stability below it, so 500 us / 250 us bus periods are outside what
/// this drive is specified for, whatever the master can do.
///
/// All CiA 402 modes are supported per the vendor manual: pp, pv, tq, hm,
/// csp, csv, cst. The option set below covers the cyclic modes a CNC
/// actually drives, and fits the 10-entry limit with room to spare
/// (5 output entries, 7 input entries).
///
/// Distributed clocks: assignActivate 0x300 with sync0 at the master
/// period, taken from the ESI DC OpMode "DC". SYNC0 is shifted by half the
/// cycle: the vendor manual recommends advancing the PDI by 50% of the SYNC0
/// period, and their own fault E.101 ("ECAT sync error") names an
/// unreasonable master SYNC Shift Time as its first cause. Bench-measured
/// at a 1 ms period: without the shift dc-sync-diff starts at 129 us and
/// takes ~20 s to converge, with it the master is within 1 ns by 10 s.
/// Both end up in the single-digit-ns range, so the shift buys startup
/// convergence, not a tighter steady-state lock. The drive also offers
/// Free Run. Override either with <dcConf> in the XML config.
///
/// Notes for anyone extending this driver:
///   - Touch probe (0x60b8..0x60bd) is present and 0x60b8 even sits in the
///     factory RxPDO, but it is not enabled here: it has not been run on
///     hardware yet, and a wrong mapping costs the OP transition.
///   - 0x608f (position encoder resolution) and 0x6092 (feed constant) do
///     NOT exist on this drive — the SDO reply is "object does not exist",
///     and neither appears in the vendor manual. A master cannot learn
///     the resolution through CiA 402; it is 2^23 counts per revolution
///     (23-bit absolute encoder), confirmed by hand on the shaft. The
///     manual's own route is the manufacturer object U0-52 = 0x201e:0x34,
///     which reports the number of ENCODER BITS (resolution = 1 << bits).
///     Verified on the bench: it reads 23, i.e. 8388608 counts/rev, matching
///     the shaft measurement. Note it is a UINT16, not a UINT32.
///     Feed constant has no substitute at all: only the electronic gear
///     0x6091 exists, and it defaults to 1:1 — so out of the box 0x6064
///     carries raw encoder counts, and all mm scaling is the master's job.
///     Bench reading confirms both subindices are 1.
///   - Drive temperature is NOT exposed in any object — neither the manual
///     nor a full SDO Info walk of the dictionary turns one up;
///     do not go looking for it. DC-bus voltage is U0-31 = 0x201e:0x1f, a
///     UINT16; on the bench it read 3157 with mains applied, which puts the
///     unit at 0.1 V (315.7 V, the expected rectified 220 V). The scale is
///     INFERRED from that agreement, not stated in the manual — confirm it
///     against a meter before showing volts to an operator.
///   - 0x603f carries a VENDOR code, not a CiA 402 one: read as DECIMAL it
///     is the digits of the drive's own Er.xx / A-xx display code (so 34 is
///     Er.34, motor overload). The full table lives in chapter 11.2 of the
///     vendor manual; formatting the value as hex points the operator at a
///     different fault entirely.
///   - Firmware and ESI must match: the ESI file states the firmware it
///     belongs to. The manual points at U2-04 for the drive's own version,
///     but there is no need to go to a manufacturer object — the standard
///     0x100a ("Manufacturer Software version") reads it as a string:
///     "V1.15" on our bench, against hardware 0x1009 "VD3E V1.1.0.0.0" and
///     revision 0x1018:03 = 0x73, which is what the ESI declares. A drive
///     older than the ESI needs the vendor's upgrade tool. Latest at the
///     time of writing: 1.15.
///   - SDO Info IS supported: the full object dictionary can be walked
///     (`ethercat sdos`), unlike the Inovance and Schneider drives on the
///     same bench. Manufacturer groups are named there — 0x201e is "U0
///     Group monitor", 0x2020 is "U2 Group Monitor", and so on.
///   - SDO writes must NOT use complete access; the drive NAKs it. The
///     Mitsubishi MR-J4 on the same bench accepts it, so this is per
///     vendor, not per bus.
///   - VD5 and VD5L share product code 0x0d510001 and differ only by
///     revision, so adding them needs revision matching, not a second
///     product code. Neither has been on our bench.

#include "../lcec.h"
#include "lcec_class_cia402.h"

static int lcec_wecon_init(int comp_id, lcec_slave_t *slave);

static const lcec_modparam_desc_t modparams_perchannel[] = {
    {NULL},
};

static const lcec_modparam_desc_t modparams_base[] = {
    {NULL},
};

static const lcec_modparam_doc_t chan_docs[] = {
    {NULL},
};

static const lcec_modparam_doc_t base_docs[] = {
    {NULL},
};

static lcec_typelist_t types[] = {
    {"VD3E", LCEC_WECON_VID, 0x0d3e0001, 0, NULL, lcec_wecon_init, NULL, 0},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, 1, modparams_perchannel, modparams_base, chan_docs, base_docs)

static void lcec_wecon_read(lcec_slave_t *slave, long period);
static void lcec_wecon_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
} lcec_wecon_data_t;

static int lcec_wecon_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_wecon_data_t *hal_data;

  slave->proc_read = lcec_wecon_read;
  slave->proc_write = lcec_wecon_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_wecon_data_t);
  slave->hal_data = hal_data;

  // Apply default Distributed Clock settings if not already set. Values
  // come from the ESI DC OpMode "DC"; the SYNC0 shift follows the
  // manual's recommendation of advancing the PDI by 50% of the SYNC0
  // period (see the file comment on E.101).
  if (slave->dc_conf == NULL) {
    lcec_slave_dc_t *dc = LCEC_HAL_ALLOCATE(lcec_slave_dc_t);
    dc->assignActivate = 0x300;
    dc->sync0Cycle = master->app_time_period;
    dc->sync0Shift = master->app_time_period / 2;
    slave->dc_conf = dc;
  }

  lcec_class_cia402_options_t *options = lcec_cia402_options();

  // One RxPDO and one TxPDO, at most 10 entries each — see the file comment.
  options->rxpdolimit = 10;
  options->txpdolimit = 10;
  options->channels = 1;

  options->channel[0]->enable_opmode = 1;
  options->channel[0]->enable_csp = 1;
  options->channel[0]->enable_csv = 1;
  options->channel[0]->enable_cst = 1;
  options->channel[0]->enable_target_torque = 1;
  options->channel[0]->enable_actual_torque = 1;
  options->channel[0]->enable_actual_following_error = 1;
  // 0x603f ships in the factory TxPDO 0x1b01, so exporting it costs nothing.
  options->channel[0]->enable_error_code = 1;

  lcec_syncs_t *syncs = lcec_cia402_init_sync(slave, options);
  lcec_cia402_add_output_sync(slave, syncs, options);
  lcec_cia402_add_input_sync(slave, syncs, options);
  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);
  hal_data->cia402->channels[0] = lcec_cia402_register_channel(slave, 0x6000, options->channel[0]);

  return 0;
}

static void lcec_wecon_read(lcec_slave_t *slave, long period) {
  lcec_wecon_data_t *hal_data = (lcec_wecon_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_read_all(slave, hal_data->cia402);
}

static void lcec_wecon_write(lcec_slave_t *slave, long period) {
  lcec_wecon_data_t *hal_data = (lcec_wecon_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_write_all(slave, hal_data->cia402);
}
