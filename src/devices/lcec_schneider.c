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

/// @brief Driver for Schneider Electric Lexium 28 (LXM28E) CiA 402 servo drives.
///
/// Single-axis EtherCAT servo drive. Read off a live LXM28E on our bench in
/// 2026-08. We have no ESI file for it, and — unlike the Mitsubishi on the same
/// bus — this drive does not answer SDO-Info at all, so the object dictionary
/// cannot be enumerated over the wire. Everything below therefore comes from
/// the PDO map and from direct SDO uploads.
///
///   - LXM28E, vendor 0x0800005a, product code 0x00096030, revision 0x00030006
///   - Mailbox: CoE and FoE
///   - SDO-Info: not supported (the dictionary reads back empty)
///   - DC system time transmission delay: 1140 ns
///
/// ⚠ Mode set differs from every other drive we have seen. 0x6502 reads
/// 0x000000ed, which is pp, pv, tq, hm, ip and csp — there is **no csv and no
/// cst**. Three other vendors on the same bench (Inovance, Wecon, Mitsubishi)
/// all answer 0x000003ad, so it is tempting to assume that set is universal.
/// It is not. This driver therefore enables csp only among the cyclic modes;
/// asking for csv or cst here would map objects the drive does not implement.
///
/// PDO layout: one RxPDO and one TxPDO (0x1c12/0x1c13 hold a single entry each,
/// 0x1600 and 0x1a00), carrying 5 and 6 entries as shipped. Unlike the
/// Mitsubishi, the map is pure CiA 402 with no vendor-specific objects. The
/// limits below are the observed counts, not a documented maximum.
///
/// Distributed clocks are not preset: no ESI means no AssignActivate value, and
/// guessing one would be worse than leaving it to <dcConf> in the XML.

#include "../lcec.h"
#include "lcec_class_cia402.h"

static int lcec_schneider_init(int comp_id, lcec_slave_t *slave);

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
    {"LXM28E", LCEC_SCHNEIDER_VID, 0x00096030, 0, NULL, lcec_schneider_init, NULL, 0},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, 1, modparams_perchannel, modparams_base, chan_docs, base_docs)

static void lcec_schneider_read(lcec_slave_t *slave, long period);
static void lcec_schneider_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
} lcec_schneider_data_t;

static int lcec_schneider_init(int comp_id, lcec_slave_t *slave) {
  lcec_schneider_data_t *hal_data;

  slave->proc_read = lcec_schneider_read;
  slave->proc_write = lcec_schneider_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_schneider_data_t);
  slave->hal_data = hal_data;

  lcec_class_cia402_options_t *options = lcec_cia402_options();

  // Entry counts as shipped by the drive — see the file comment.
  options->rxpdolimit = 5;
  options->txpdolimit = 6;
  options->channels = 1;

  // csp only: 0x6502 says this drive has no csv and no cst.
  options->channel[0]->enable_opmode = 1;
  options->channel[0]->enable_csp = 1;
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

static void lcec_schneider_read(lcec_slave_t *slave, long period) {
  lcec_schneider_data_t *hal_data = (lcec_schneider_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_read_all(slave, hal_data->cia402);
}

static void lcec_schneider_write(lcec_slave_t *slave, long period) {
  lcec_schneider_data_t *hal_data = (lcec_schneider_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_write_all(slave, hal_data->cia402);
}
