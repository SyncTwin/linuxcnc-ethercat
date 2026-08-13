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

/// @brief Driver for Mitsubishi MR-J4 CiA 402 servo drives (TM network type).
///
/// Single-axis EtherCAT servo drive. Everything below was read off a live
/// MR-J4-20TM on our bench in 2026-08; we have no ESI file for this drive, so
/// the values come from the bus itself rather than from vendor XML.
///
///   - MR-J4-20TM, vendor 0x00000a1e, product code 0x00000201, revision
///     0x00020002
///   - Mailbox: CoE and FoE
///   - SDO-Info works: the object dictionary answers with 809 objects, which is
///     by far the richest of the four vendors on our bench (Wecon 109,
///     Inovance 0).
///   - 0x6502 reads 0x000003ad: pp, pv, tq, hm, csp, csv, cst.
///
/// PDO layout: one RxPDO and one TxPDO (0x1c12 and 0x1c13 each hold a single
/// entry, 0x1600 and 0x1a00). As shipped, the RxPDO carries 12 entries and the
/// TxPDO 14 — noticeably fuller than other vendors, because Mitsubishi maps its
/// own objects alongside the standard ones: 0x2d01..0x2d03 "Control DI",
/// 0x2d11..0x2d13 "Status DO" and 0x2d20 "Velocity limit value". The limits
/// below are those observed counts. We could not confirm the true maximum
/// without an ESI, so treat them as a floor, not a datasheet value.
///
/// The full touch probe set (0x60b8..0x60bd) is in the factory TxPDO. It is not
/// enabled here for the same reason as in the other drivers: untested on
/// hardware, and a wrong mapping costs the transition to OP.
///
/// Distributed clocks: deliberately NOT preset. The drive reports a system time
/// transmission delay of 0 ns and we have no ESI to read AssignActivate from,
/// so inventing a value would be a guess. Configure DC through <dcConf> in the
/// XML if your installation needs it.
///
/// Note that 0x603f (error code) is absent from the factory TxPDO, so
/// enable_error_code is left off — turning it on grows the mapping past what
/// the drive ships with, and that is exactly the kind of change that should be
/// tested on hardware first.

#include "../lcec.h"
#include "lcec_class_cia402.h"

static int lcec_mitsubishi_init(int comp_id, lcec_slave_t *slave);

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
    {"MR-J4-TM", LCEC_MITSUBISHI_VID, 0x00000201, 0, NULL, lcec_mitsubishi_init, NULL, 0},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, 1, modparams_perchannel, modparams_base, chan_docs, base_docs)

static void lcec_mitsubishi_read(lcec_slave_t *slave, long period);
static void lcec_mitsubishi_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
} lcec_mitsubishi_data_t;

static int lcec_mitsubishi_init(int comp_id, lcec_slave_t *slave) {
  lcec_mitsubishi_data_t *hal_data;

  slave->proc_read = lcec_mitsubishi_read;
  slave->proc_write = lcec_mitsubishi_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_mitsubishi_data_t);
  slave->hal_data = hal_data;

  lcec_class_cia402_options_t *options = lcec_cia402_options();

  // Entry counts as shipped by the drive — see the file comment.
  options->rxpdolimit = 12;
  options->txpdolimit = 14;
  options->channels = 1;

  options->channel[0]->enable_opmode = 1;
  options->channel[0]->enable_csp = 1;
  options->channel[0]->enable_csv = 1;
  options->channel[0]->enable_cst = 1;
  options->channel[0]->enable_target_torque = 1;
  options->channel[0]->enable_actual_torque = 1;
  options->channel[0]->enable_actual_following_error = 1;

  lcec_syncs_t *syncs = lcec_cia402_init_sync(slave, options);
  lcec_cia402_add_output_sync(slave, syncs, options);
  lcec_cia402_add_input_sync(slave, syncs, options);
  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);
  hal_data->cia402->channels[0] = lcec_cia402_register_channel(slave, 0x6000, options->channel[0]);

  return 0;
}

static void lcec_mitsubishi_read(lcec_slave_t *slave, long period) {
  lcec_mitsubishi_data_t *hal_data = (lcec_mitsubishi_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_read_all(slave, hal_data->cia402);
}

static void lcec_mitsubishi_write(lcec_slave_t *slave, long period) {
  lcec_mitsubishi_data_t *hal_data = (lcec_mitsubishi_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_write_all(slave, hal_data->cia402);
}
