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
/// The NX-ECC202 is the EtherCAT coupler at the head of a modular Omron NX
/// station: the coupler is the only slave on the wire, and the NX units
/// clipped onto it are not separate slaves.  Their process data is aggregated
/// into the coupler's own process image, so vendor ID and product code say
/// nothing about the pinout.
///
/// # The driver knows the coupler, the config knows the units
///
/// No NX unit is known to this file.  Every unit is described in the XML
/// config, generic style, under the coupler's `<slave>`: the unit's PDOs with
/// their entries, `halPin` and `halType` (printed from the vendor ESI, one
/// catalog row per unit):
///
///     <slave idx="4" type="NX-ECC202">
///       <syncManager idx="2" dir="out">
///         <pdo idx="0x1600">                         <!-- slot 1 -->
///           <pdoEntry idx="0x7002" subIdx="1" bitLen="16" halPin="slot1.dout" halType="bit"/>
///         </pdo>
///       </syncManager>
///       ...
///     </slave>
///
/// Only unit PDOs are declared (0x1600..0x16ff out, 0x1a00..0x1aff in, four
/// per slot).  The coupler's own PDOs and the alignment entries are not: the
/// driver knows those, below.
///
/// # The process image comes from the coupler
///
/// In `_init` (PREOP, mailbox up) the driver reads the coupler's actual PDO
/// assignment over CoE -- 0x1c12/0x1c13 and every assigned 0x16xx/0x1axx --
/// and hands exactly that map to the master, padding included.  On a live
/// NX-ECC202 0x1c13 lists station diagnostics, a status byte, an alignment
/// entry, the unit inputs and a second alignment entry; dropping an alignment
/// entry shifts every input behind it and still reaches OP.
///
/// # Declared = read, or no station
///
/// Before anything is registered the declared units are checked against the
/// read map, PDO by PDO and entry by entry (index, subindex, bit length):
///
///   - a declared unit PDO the coupler does not assign -> refused;
///   - a unit PDO the coupler assigns that the config does not declare -> refused;
///   - a declared PDO outside the unit ranges -> refused.
///
/// A unit in a slot that differs from the config therefore stops the station
/// at init, naming the slot, instead of running with pins on other terminals.
/// Two units with the same PDO layout (an NPN and a PNP DO16) are not told
/// apart by this check.
///
/// # Unit data validity: `io-active`
///
/// Object 0x3006:04, "NX Unit I/O Data Active Status 125", 128 bits, bit n =
/// NX Unit n, bit 0 = the coupler (W519-E1-15, A-7-6 p. A-45, p. 9-18), is in
/// TxPDO 0x1bf8 of a stock NX-ECC202.  When it is in the read map the driver
/// publishes `coupler-io-active` (bit 0) and `slot<S>.io-active` (bit S) for
/// every declared slot.  If it is not mapped, no io-active pins are created.

#include <stdio.h>

#include "../lcec.h"
#include "lcec_generic.h"

#define NX_RXPDO_ASSIGN 0x1c12  ///< Output (master -> slave) PDO assignment.
#define NX_TXPDO_ASSIGN 0x1c13  ///< Input (slave -> master) PDO assignment.

#define NX_MAX_PDOS     80  ///< Assigned PDOs we will look at, per direction.
#define NX_MAX_ENTRIES  512 ///< Mapping entries we will look at, per direction.
#define NX_MAX_PER_PDO  64  ///< Mapping entries in a single 0x16xx/0x1axx.
#define NX_UNIT_RXPDO   0x1600  ///< First unit RxPDO (slot 1).
#define NX_UNIT_TXPDO   0x1a00  ///< First unit TxPDO (slot 1).
#define NX_UNIT_PDO_SPAN 0x100  ///< Unit PDO range per direction.
#define NX_PDO_SLOT_STEP 4      ///< PDOs per slot (ESI DependOnSlot, NX-ECC20x).
#define NX_MAX_SLOTS (NX_UNIT_PDO_SPAN / NX_PDO_SLOT_STEP)  ///< Slots one range holds.
#define NX_IO_ACTIVE_IDX  0x3006  ///< NX Unit I/O Data Active Status (W519 A-7-6).
#define NX_IO_ACTIVE_SIDX 0x04    ///< Subindex 04: 125 units, 128 bits, bit n = unit n.

/// @brief One PDO mapping entry, as decoded from a 0x16xx/0x1axx subindex.
typedef struct {
  uint16_t idx;    ///< CoE object index, 0 for an alignment entry.
  uint8_t sidx;    ///< CoE object subindex.
  uint8_t bitlen;  ///< Width in bits.
} lcec_omron_nx_entry_t;

/// @brief One direction of the coupler's map, as read over CoE.
typedef struct {
  lcec_omron_nx_entry_t entries[NX_MAX_ENTRIES];
  uint16_t pdos[NX_MAX_PDOS];
  int first[NX_MAX_PDOS];
  int count[NX_MAX_PDOS];
  int n_pdos;
  int n_entries;
} lcec_omron_nx_map_t;

typedef struct {
  lcec_generic_pin_t *pins;                    ///< Declared unit entries (from the config).
  int n_pins;
  int io_active_mapped;                        ///< 0x3006:04 is in the input map.
  unsigned int io_active_os;                   ///< Byte offset of 0x3006:04.
  unsigned int io_active_bp;                   ///< Bit position of 0x3006:04.
  int io_active_bits;                          ///< Width of the mapped entry.
  hal_bit_t *io_active[NX_MAX_SLOTS + 1];      ///< [0] coupler, [S] slot S; NULL if no pin.
} lcec_omron_nx_data_t;

static int lcec_omron_nx_init(int comp_id, lcec_slave_t *slave);
static void lcec_omron_nx_read(lcec_slave_t *slave, long period);
static void lcec_omron_nx_write(lcec_slave_t *slave, long period);

static lcec_typelist_t types[] = {
    // NX-ECC201 and NX-ECC203 are the same family, but their coupler PDOs
    // (diagnostics, io-active) have not been read off hardware here.
    {.name = "NX-ECC202", .vid = LCEC_OMRON_VID, .pid = 0x000000a6, .proc_init = lcec_omron_nx_init, .conf_pdos = 1},
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
static int lcec_omron_nx_scan(lcec_slave_t *slave, uint16_t assign_idx, lcec_omron_nx_map_t *map) {
  lcec_omron_nx_entry_t tmp[NX_MAX_PER_PDO];
  int n, i, j;

  map->n_entries = 0;
  n = lcec_omron_nx_read_assign(slave, assign_idx, map->pdos, NX_MAX_PDOS);
  if (n < 0) return n;
  map->n_pdos = n;

  for (i = 0; i < n; i++) {
    int m = lcec_omron_nx_read_mapping(slave, map->pdos[i], tmp, NX_MAX_PER_PDO);

    if (m < 0) return m;
    if (map->n_entries + m > NX_MAX_ENTRIES) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: more than %d mapping entries on 0x%04x\n", slave->master->name,
          slave->name, NX_MAX_ENTRIES, assign_idx);
      return -E2BIG;
    }
    map->first[i] = map->n_entries;
    map->count[i] = m;
    for (j = 0; j < m; j++) map->entries[map->n_entries++] = tmp[j];
  }
  return map->n_entries;
}

/// @brief Base of the unit PDO range of a direction, 0 if `pdo` is not a unit PDO.
static uint16_t lcec_omron_nx_unit_base(uint16_t pdo) {
  if (pdo >= NX_UNIT_RXPDO && pdo < NX_UNIT_RXPDO + NX_UNIT_PDO_SPAN) return NX_UNIT_RXPDO;
  if (pdo >= NX_UNIT_TXPDO && pdo < NX_UNIT_TXPDO + NX_UNIT_PDO_SPAN) return NX_UNIT_TXPDO;
  return 0;
}

/// @brief Slot (1-based) of a unit PDO.
static int lcec_omron_nx_slot(uint16_t pdo) {
  return (pdo - lcec_omron_nx_unit_base(pdo)) / NX_PDO_SLOT_STEP + 1;
}

/// @brief Check the declared unit PDOs of one direction against the read map.
///
/// @return 0 when they are the same set with the same entries, -EINVAL otherwise.
static int lcec_omron_nx_verify(lcec_slave_t *slave, const ec_sync_info_t *sm, const lcec_omron_nx_map_t *map, int *has_slot) {
  lcec_master_t *master = slave->master;
  int i, j, k, declared = 0;

  for (i = 0; sm != NULL && i < (int)sm->n_pdos; i++) {
    const ec_pdo_info_t *pdo = &sm->pdos[i];

    if (lcec_omron_nx_unit_base(pdo->index) == 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: PDO 0x%04x is not a unit PDO; the coupler's own PDOs are read off it, not declared\n",
          master->name, slave->name, pdo->index);
      return -EINVAL;
    }
    for (k = 0; k < map->n_pdos && map->pdos[k] != pdo->index; k++);
    if (k == map->n_pdos) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: slot %d: declared PDO 0x%04x is not assigned by the coupler -- the unit in the slot is not the one in the config\n",
          master->name, slave->name, lcec_omron_nx_slot(pdo->index), pdo->index);
      return -EINVAL;
    }
    if ((int)pdo->n_entries != map->count[k]) goto differs;
    for (j = 0; j < map->count[k]; j++) {
      const lcec_omron_nx_entry_t *e = &map->entries[map->first[k] + j];
      const ec_pdo_entry_info_t *d = &pdo->entries[j];

      if (d->index != e->idx || d->subindex != e->sidx || d->bit_length != e->bitlen) goto differs;
    }
    has_slot[lcec_omron_nx_slot(pdo->index)] = 1;
    declared++;
    continue;

  differs:
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: slot %d: PDO 0x%04x maps other entries on the coupler than in the config -- the unit in the slot is not the one in the config\n",
        master->name, slave->name, lcec_omron_nx_slot(pdo->index), pdo->index);
    return -EINVAL;
  }

  for (k = 0; k < map->n_pdos; k++) {
    if (lcec_omron_nx_unit_base(map->pdos[k]) == 0) continue;
    for (i = 0; sm != NULL && i < (int)sm->n_pdos && sm->pdos[i].index != map->pdos[k]; i++);
    if (sm == NULL || i == (int)sm->n_pdos) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: slot %d: the coupler assigns PDO 0x%04x, the config declares no unit there\n",
          master->name, slave->name, lcec_omron_nx_slot(map->pdos[k]), map->pdos[k]);
      return -EINVAL;
    }
  }
  return declared;
}

/// @brief The declared sync manager of a direction, NULL if none.
static const ec_sync_info_t *lcec_omron_nx_declared(const lcec_slave_t *slave, ec_direction_t dir) {
  const ec_sync_info_t *sm;

  // the config's sync managers, terminated by index 0xff
  for (sm = slave->generic_sync_managers; sm != NULL && sm->index != 0xff; sm++) {
    if (sm->dir == dir) return sm;
  }
  return NULL;
}

/// @brief Publish 0x3006:04 as `coupler-io-active` and `slot<S>.io-active`.
static int lcec_omron_nx_init_io_active(lcec_slave_t *slave, lcec_omron_nx_data_t *hal_data, const lcec_omron_nx_map_t *in, const int *has_slot) {
  lcec_master_t *master = slave->master;
  int i;

  for (i = 0; i < in->n_entries; i++) {
    if (in->entries[i].idx == NX_IO_ACTIVE_IDX && in->entries[i].sidx == NX_IO_ACTIVE_SIDX) break;
  }
  if (i == in->n_entries) {
    rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "slave %s.%s: 0x%04x:%02x not mapped, no io-active pins\n", master->name, slave->name,
        NX_IO_ACTIVE_IDX, NX_IO_ACTIVE_SIDX);
    return 0;
  }

  hal_data->io_active_bits = in->entries[i].bitlen;
  if (lcec_pdo_init(slave, NX_IO_ACTIVE_IDX, NX_IO_ACTIVE_SIDX, &hal_data->io_active_os, &hal_data->io_active_bp) != 0) return -EIO;
  hal_data->io_active_mapped = 1;

  if (lcec_pin_newf(HAL_BIT, HAL_OUT, (void **)&hal_data->io_active[0], "%s.%s.%s.coupler-io-active", LCEC_MODULE_NAME, master->name,
          slave->name) != 0)
    return -EIO;

  for (int s = 1; s <= NX_MAX_SLOTS && s < hal_data->io_active_bits; s++) {
    if (!has_slot[s]) continue;
    if (lcec_pin_newf(HAL_BIT, HAL_OUT, (void **)&hal_data->io_active[s], "%s.%s.%s.slot%d.io-active", LCEC_MODULE_NAME, master->name,
            slave->name, s) != 0)
      return -EIO;
  }
  return 0;
}

/// @brief Append one read direction to the master's sync config.
static void lcec_omron_nx_add_sync(lcec_syncs_t *syncs, ec_direction_t dir, ec_watchdog_mode_t wd, const lcec_omron_nx_map_t *map) {
  lcec_syncs_add_sync(syncs, dir, wd);
  for (int i = 0; i < map->n_pdos; i++) {
    lcec_syncs_add_pdo_info(syncs, map->pdos[i]);
    for (int j = 0; j < map->count[i]; j++) {
      const lcec_omron_nx_entry_t *e = &map->entries[map->first[i] + j];
      lcec_syncs_add_pdo_entry(syncs, e->idx, e->sidx, e->bitlen);
    }
  }
}

static int lcec_omron_nx_init(int comp_id, lcec_slave_t *slave) {
  (void)comp_id;
  lcec_master_t *master = slave->master;
  static lcec_omron_nx_map_t out, in;  // _init runs once per slave, sequentially, in non-RT context
  lcec_omron_nx_data_t *hal_data;
  lcec_syncs_t *syncs;
  int has_slot[NX_MAX_SLOTS + 1] = {0};
  int n_out, n_in, err;

  if (lcec_omron_nx_scan(slave, NX_RXPDO_ASSIGN, &out) < 0 || lcec_omron_nx_scan(slave, NX_TXPDO_ASSIGN, &in) < 0) {
    // Without the station's own map there is nothing honest to hand the
    // master: on an NX station a wrong output bit moves real hardware.
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s: could not read the station's PDO map over CoE\n", master->name, slave->name);
    return -EIO;
  }

  n_out = lcec_omron_nx_verify(slave, lcec_omron_nx_declared(slave, EC_DIR_OUTPUT), &out, has_slot);
  if (n_out < 0) return n_out;
  n_in = lcec_omron_nx_verify(slave, lcec_omron_nx_declared(slave, EC_DIR_INPUT), &in, has_slot);
  if (n_in < 0) return n_in;
  rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "slave %s.%s: NX station matches the config: %d output and %d input unit PDOs\n",
      master->name, slave->name, n_out, n_in);

  // Hand the read map to the master exactly as it was read, padding included.
  syncs = LCEC_HAL_ALLOCATE(lcec_syncs_t);
  if (syncs == NULL) return -ENOMEM;
  lcec_syncs_init(slave, syncs);
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DISABLE);  // SM0, mailbox out
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);   // SM1, mailbox in
  lcec_omron_nx_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_ENABLE, &out);  // SM2, outputs
  lcec_omron_nx_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE, &in);   // SM3, inputs
  slave->sync_info = syncs->syncs;

  hal_data = LCEC_HAL_ALLOCATE(lcec_omron_nx_data_t);
  if (hal_data == NULL) return -ENOMEM;
  memset(hal_data, 0, sizeof(*hal_data));
  hal_data->pins = (lcec_generic_pin_t *)slave->hal_data;  // the config's pins, allocated by lcec_main
  hal_data->n_pins = slave->generic_pdo_entry_count;
  slave->hal_data = hal_data;

  err = lcec_generic_init_pins(slave, hal_data->pins, hal_data->n_pins);
  if (err != 0) return err;
  err = lcec_omron_nx_init_io_active(slave, hal_data, &in, has_slot);
  if (err != 0) return err;

  slave->proc_read = lcec_omron_nx_read;
  slave->proc_write = lcec_omron_nx_write;
  return 0;
}

static void lcec_omron_nx_read(lcec_slave_t *slave, long period) {
  lcec_omron_nx_data_t *hal_data = (lcec_omron_nx_data_t *)slave->hal_data;
  uint8_t *pd = lcec_slave_pd(slave);

  if (!slave->state.operational) return;
  lcec_generic_read_pins(slave, hal_data->pins, hal_data->n_pins);

  if (hal_data->io_active_mapped) {
    for (int s = 0; s <= NX_MAX_SLOTS; s++) {
      unsigned int bit = hal_data->io_active_bp + s;

      if (hal_data->io_active[s] == NULL) continue;
      LCEC_PIN_BIT_SET(hal_data->io_active[s], EC_READ_BIT(&pd[hal_data->io_active_os + (bit >> 3)], bit & 7));
    }
  }
}

static void lcec_omron_nx_write(lcec_slave_t *slave, long period) {
  lcec_omron_nx_data_t *hal_data = (lcec_omron_nx_data_t *)slave->hal_data;

  if (!slave->state.operational) return;
  lcec_generic_write_pins(slave, hal_data->pins, hal_data->n_pins);
}
