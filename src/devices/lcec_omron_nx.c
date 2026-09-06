//
//    Copyright (C) 2026 SyncTwin GmbH
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

/// @file
/// @brief Driver for Omron NX-series EtherCAT couplers (NX-ECC202)
///
/// The NX-ECC202 is not an I/O module.  It is the EtherCAT coupler at the head
/// of a modular Omron NX station: the coupler is the only slave on the wire,
/// and the NX units clipped onto it are not visible as separate slaves.  Their
/// process data is aggregated into the coupler's own process image.
///
/// That has a consequence for a named driver: the vendor ID and product code
/// are the same for every station, whatever units are mounted.  Unlike an
/// EL2008, where the name implies the pinout, the string "NX-ECC202" implies
/// nothing about the process image.  A driver with a hardcoded object list
/// would be correct for exactly one assembled station and silently wrong for
/// every other one -- and a wrong output bit on an NX station moves real
/// hardware.
///
/// So this driver reads the coupler's actual PDO assignment over CoE during
/// `_init` (in PREOP, where the mailbox is already up):
///
///   - 0x1c12:00..N  -> assigned RxPDOs (outputs, 0x16xx)
///   - 0x1c13:00..N  -> assigned TxPDOs (inputs, 0x1axx)
///   - each 0x16xx / 0x1axx:00..N -> mapping entries, one u32 per subindex,
///     packed as (index << 16) | (subindex << 8) | bitlength
///
/// The discovered mapping is handed to the master as the slave's sync config,
/// and HAL pins are created for the entries that carry I/O.  The station
/// lineup therefore never has to be repeated in the XML config.
///
/// # Padding entries must be kept
///
/// The assignment is not just the modules.  On a live NX-ECC202 the input
/// assignment 0x1c13 lists five PDOs, in this order: station diagnostics,
/// a station status byte, an alignment entry, then the module inputs, then a
/// second alignment entry.  The alignment entries map object 0x0000:00 and
/// carry no data at all.
///
/// They still have to be declared, in order, because they occupy bits in the
/// process image.  Dropping the first one shifts every input that follows it
/// by eight bits, and that does not show up as a failure: the station reaches
/// OP and the pins simply read the wrong terminals.  Keeping them costs
/// nothing -- an entry with no HAL pin just takes up its place in the map.
///
/// # Which entries get pins
///
/// Pins are created by object index range, which is what tells apart module
/// data from the coupler's own housekeeping:
///
///   - 0x6000..0x6fff -> station inputs   -> `din-N`
///   - 0x7000..0x7fff -> station outputs  -> `dout-N`
///   - everything else (0x0000 padding, the 0x2002 status byte, the 0x3xxx
///     diagnostics blocks) is declared but not exposed.
///
/// A one-bit entry becomes one pin; a wider entry is treated as a packed word
/// and becomes one pin per bit, LSB first.  Numbering runs across the whole
/// station in assignment order, so the first output unit's first terminal is
/// `dout-0`.
///
/// Analog NX units are not supported: a 16-bit mapping entry from a digital
/// output unit and a 16-bit entry from an analog output unit look identical in
/// the mapping, and the driver assumes digital.  Stations with analog units
/// need `type="generic"` with an explicit map for now.

#include <stdio.h>

#include "../lcec.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

#define NX_RXPDO_ASSIGN 0x1c12  ///< Output (master -> slave) PDO assignment.
#define NX_TXPDO_ASSIGN 0x1c13  ///< Input (slave -> master) PDO assignment.

#define NX_MAX_PDOS     16  ///< Assigned PDOs we will look at, per direction.
#define NX_MAX_ENTRIES  64  ///< Mapping entries we will look at, per direction.
#define NX_MAX_PER_PDO  32  ///< Mapping entries in a single 0x16xx/0x1axx.
#define NX_PIN_NAME_LEN 16  ///< Enough for "dout-<n>".

/// @brief One PDO mapping entry, as decoded from a 0x16xx/0x1axx subindex.
typedef struct {
  uint16_t idx;    ///< CoE object index, 0 for an alignment entry.
  uint8_t sidx;    ///< CoE object subindex.
  uint8_t bitlen;  ///< Width in bits.
} lcec_omron_nx_entry_t;

typedef struct {
  lcec_class_din_channels_t *din;
  lcec_class_dout_channels_t *dout;
} lcec_omron_nx_data_t;

static int lcec_omron_nx_init(int comp_id, lcec_slave_t *slave);
static void lcec_omron_nx_read(lcec_slave_t *slave, long period);
static void lcec_omron_nx_write(lcec_slave_t *slave, long period);

static lcec_typelist_t types[] = {
    // NX-ECC201 and NX-ECC203 are the same family and would very likely work
    // with this code, but their product codes have not been read off hardware
    // here, and guessing an identity is how a config ends up pointing at the
    // wrong slave.  Add them once they have been seen.
    {"NX-ECC202", LCEC_OMRON_VID, 0x000000a6, 0, NULL, lcec_omron_nx_init, NULL, 0},
    {NULL},
};
ADD_TYPES(types);

/// @brief Read one sync manager's PDO assignment.
///
/// @param slave      The slave, from `_init`.
/// @param assign_idx 0x1c12 for outputs, 0x1c13 for inputs.
/// @param out        Caller-supplied array of at least `max` PDO indices.
/// @param max        Capacity of `out`.
/// @return Number of assigned PDOs, or a negative errno.
static int lcec_omron_nx_read_assign(lcec_slave_t *slave, uint16_t assign_idx, uint16_t *out, int max) {
  uint8_t count;
  int n = 0;

  if (lcec_read_sdo8(slave, assign_idx, 0x00, &count) != 0) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: failed reading PDO assignment 0x%04x:00\n", slave->master->name, slave->name, assign_idx);
    return -EIO;
  }

  for (uint8_t i = 1; i <= count; i++) {
    uint16_t pdo;

    if (lcec_read_sdo16(slave, assign_idx, i, &pdo) != 0) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: failed reading 0x%04x:%02x\n", slave->master->name, slave->name, assign_idx, i);
      return -EIO;
    }
    if (pdo == 0) continue;

    if (n >= max) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: 0x%04x assigns more than %d PDOs\n", slave->master->name, slave->name, assign_idx, max);
      return -E2BIG;
    }
    out[n++] = pdo;
  }

  return n;
}

/// @brief Read the mapping entries of a single 0x16xx / 0x1axx object.
///
/// Alignment entries (object 0x0000) are kept: they take up space in the
/// process image, and dropping them shifts everything behind them.
///
/// @param slave       The slave, from `_init`.
/// @param mapping_idx The 0x16xx or 0x1axx object to read.
/// @param out         Caller-supplied array of at least `max` entries.
/// @param max         Capacity of `out`.
/// @return Number of entries, or a negative errno.
static int lcec_omron_nx_read_mapping(lcec_slave_t *slave, uint16_t mapping_idx, lcec_omron_nx_entry_t *out, int max) {
  uint8_t count;

  if (lcec_read_sdo8(slave, mapping_idx, 0x00, &count) != 0) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: failed reading PDO mapping 0x%04x:00\n", slave->master->name, slave->name, mapping_idx);
    return -EIO;
  }

  if (count > max) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: mapping 0x%04x has %u entries, maximum is %d\n", slave->master->name,
        slave->name, mapping_idx, (unsigned int)count, max);
    return -E2BIG;
  }

  for (uint8_t i = 0; i < count; i++) {
    uint32_t raw;

    if (lcec_read_sdo32(slave, mapping_idx, i + 1, &raw) != 0) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: failed reading 0x%04x:%02x\n", slave->master->name, slave->name, mapping_idx, i + 1);
      return -EIO;
    }

    out[i].idx = (uint16_t)(raw >> 16);
    out[i].sidx = (uint8_t)(raw >> 8);
    out[i].bitlen = (uint8_t)(raw & 0xff);
  }

  return count;
}

/// @brief Read a whole direction: every assigned PDO and all of its entries.
///
/// @param entries     Flat output array of all entries, in image order.
/// @param pdos        Output array of the assigned PDO indices.
/// @param pdo_first   Output: index into `entries` where each PDO starts.
/// @param pdo_count   Output: number of entries in each PDO.
/// @return Total number of entries, or a negative errno.
static int lcec_omron_nx_scan(
    lcec_slave_t *slave, uint16_t assign_idx, lcec_omron_nx_entry_t *entries, uint16_t *pdos, int *pdo_first, int *pdo_count, int *n_pdos) {
  lcec_omron_nx_entry_t tmp[NX_MAX_PER_PDO];
  int n_entries = 0;
  int n, i, j;

  n = lcec_omron_nx_read_assign(slave, assign_idx, pdos, NX_MAX_PDOS);
  if (n < 0) return n;
  *n_pdos = n;

  for (i = 0; i < n; i++) {
    int m = lcec_omron_nx_read_mapping(slave, pdos[i], tmp, NX_MAX_PER_PDO);

    if (m < 0) return m;
    if (n_entries + m > NX_MAX_ENTRIES) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: more than %d mapping entries on 0x%04x\n", slave->master->name, slave->name,
          NX_MAX_ENTRIES, assign_idx);
      return -E2BIG;
    }

    pdo_first[i] = n_entries;
    pdo_count[i] = m;
    for (j = 0; j < m; j++) entries[n_entries++] = tmp[j];
  }

  return n_entries;
}

/// @brief Is this entry station I/O, as opposed to padding or housekeeping?
static int lcec_omron_nx_is_io(const lcec_omron_nx_entry_t *e, uint16_t base) {
  return e->idx >= base && e->idx < (uint16_t)(base + 0x1000) && e->sidx > 0 && e->bitlen > 0;
}

/// @brief Count the HAL pins an entry list will produce.
static int lcec_omron_nx_count_pins(const lcec_omron_nx_entry_t *entries, int count, uint16_t base) {
  int pins = 0;

  for (int i = 0; i < count; i++) {
    if (lcec_omron_nx_is_io(&entries[i], base)) pins += entries[i].bitlen;
  }
  return pins;
}

/// @brief Allocate a pin name that outlives `_init`.
///
/// The din/dout classes keep the pointer they are given, so a stack buffer
/// will not do.
static char *lcec_omron_nx_pin_name(const char *prefix, int id) {
  char *name = LCEC_HAL_ALLOCATE_STRING(NX_PIN_NAME_LEN);

  if (name == NULL) return NULL;
  snprintf(name, NX_PIN_NAME_LEN, "%s-%d", prefix, id);
  return name;
}

static int lcec_omron_nx_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_omron_nx_entry_t out_entries[NX_MAX_ENTRIES], in_entries[NX_MAX_ENTRIES];
  uint16_t out_pdos[NX_MAX_PDOS], in_pdos[NX_MAX_PDOS];
  int out_first[NX_MAX_PDOS], out_count[NX_MAX_PDOS];
  int in_first[NX_MAX_PDOS], in_count[NX_MAX_PDOS];
  int n_out_pdos = 0, n_in_pdos = 0;
  int n_out, n_in, n_dout, n_din;
  lcec_omron_nx_data_t *hal_data;
  lcec_syncs_t *syncs;
  int i, j, id;

  n_out = lcec_omron_nx_scan(slave, NX_RXPDO_ASSIGN, out_entries, out_pdos, out_first, out_count, &n_out_pdos);
  if (n_out < 0) goto no_map;
  n_in = lcec_omron_nx_scan(slave, NX_TXPDO_ASSIGN, in_entries, in_pdos, in_first, in_count, &n_in_pdos);
  if (n_in < 0) goto no_map;

  n_dout = lcec_omron_nx_count_pins(out_entries, n_out, 0x7000);
  n_din = lcec_omron_nx_count_pins(in_entries, n_in, 0x6000);

  rtapi_print_msg(RTAPI_MSG_INFO,
      LCEC_MSG_PFX
      "slave %s.%s: NX station has %d output and %d input mapping entries, "
      "giving %d dout and %d din pins; all I/O entries are treated as digital\n",
      master->name, slave->name, n_out, n_in, n_dout, n_din);

  // Hand the discovered map to the master exactly as it was read, padding
  // included.  Anything left out here shifts the entries behind it.
  syncs = LCEC_HAL_ALLOCATE(lcec_syncs_t);
  if (syncs == NULL) return -ENOMEM;

  lcec_syncs_init(slave, syncs);
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DISABLE);  // SM0, mailbox out
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);   // SM1, mailbox in

  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_ENABLE);  // SM2, outputs
  for (i = 0; i < n_out_pdos; i++) {
    lcec_syncs_add_pdo_info(syncs, out_pdos[i]);
    for (j = 0; j < out_count[i]; j++) {
      const lcec_omron_nx_entry_t *e = &out_entries[out_first[i] + j];
      lcec_syncs_add_pdo_entry(syncs, e->idx, e->sidx, e->bitlen);
    }
  }

  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);  // SM3, inputs
  for (i = 0; i < n_in_pdos; i++) {
    lcec_syncs_add_pdo_info(syncs, in_pdos[i]);
    for (j = 0; j < in_count[i]; j++) {
      const lcec_omron_nx_entry_t *e = &in_entries[in_first[i] + j];
      lcec_syncs_add_pdo_entry(syncs, e->idx, e->sidx, e->bitlen);
    }
  }

  slave->sync_info = syncs->syncs;

  hal_data = LCEC_HAL_ALLOCATE(lcec_omron_nx_data_t);
  if (hal_data == NULL) return -ENOMEM;
  hal_data->din = NULL;
  hal_data->dout = NULL;
  slave->hal_data = hal_data;

  if (n_dout > 0) {
    hal_data->dout = lcec_dout_allocate_channels(n_dout);
    if (hal_data->dout == NULL) return -ENOMEM;

    id = 0;
    for (i = 0; i < n_out; i++) {
      const lcec_omron_nx_entry_t *e = &out_entries[i];

      if (!lcec_omron_nx_is_io(e, 0x7000)) continue;

      if (e->bitlen == 1) {
        hal_data->dout->channels[id] = lcec_dout_register_channel(slave, id, e->idx, e->sidx);
        if (hal_data->dout->channels[id] == NULL) return -EIO;
        id++;
        continue;
      }

      for (int bit = 0; bit < e->bitlen; bit++) {
        char *name = lcec_omron_nx_pin_name("dout", id);

        if (name == NULL) return -ENOMEM;
        hal_data->dout->channels[id] = lcec_dout_register_channel_packed(slave, e->idx, e->sidx, bit, name);
        if (hal_data->dout->channels[id] == NULL) return -EIO;
        id++;
      }
    }
    slave->proc_write = lcec_omron_nx_write;
  }

  if (n_din > 0) {
    hal_data->din = lcec_din_allocate_channels(n_din);
    if (hal_data->din == NULL) return -ENOMEM;

    id = 0;
    for (i = 0; i < n_in; i++) {
      const lcec_omron_nx_entry_t *e = &in_entries[i];

      if (!lcec_omron_nx_is_io(e, 0x6000)) continue;

      if (e->bitlen == 1) {
        hal_data->din->channels[id] = lcec_din_register_channel(slave, id, e->idx, e->sidx);
        if (hal_data->din->channels[id] == NULL) return -EIO;
        id++;
        continue;
      }

      for (int bit = 0; bit < e->bitlen; bit++) {
        char *name = lcec_omron_nx_pin_name("din", id);

        if (name == NULL) return -ENOMEM;
        hal_data->din->channels[id] = lcec_din_register_channel_packed(slave, e->idx, e->sidx, bit, name);
        if (hal_data->din->channels[id] == NULL) return -EIO;
        id++;
      }
    }
    slave->proc_read = lcec_omron_nx_read;
  }

  return 0;

no_map:
  // Without the station's own map there is nothing honest to publish.  Refuse
  // instead of inventing a plausible pinout: on an NX station a wrong output
  // bit moves real hardware.
  rtapi_print_msg(RTAPI_MSG_ERR,
      LCEC_MSG_PFX
      "slave %s.%s: could not read the station's PDO map over CoE; "
      "use type=\"generic\" with an explicit map instead\n",
      master->name, slave->name);
  return -EIO;
}

static void lcec_omron_nx_read(lcec_slave_t *slave, long period) {
  lcec_omron_nx_data_t *hal_data = (lcec_omron_nx_data_t *)slave->hal_data;

  if (!slave->state.operational) return;
  lcec_din_read_all(slave, hal_data->din);
}

static void lcec_omron_nx_write(lcec_slave_t *slave, long period) {
  lcec_omron_nx_data_t *hal_data = (lcec_omron_nx_data_t *)slave->hal_data;

  if (!slave->state.operational) return;
  lcec_dout_write_all(slave, hal_data->dout);
}
