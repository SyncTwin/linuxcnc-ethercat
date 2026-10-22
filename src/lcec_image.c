//
//    Copyright (C) 2026 SyncTwin
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//

/// @file
/// @brief Process image ring: the writer side (layout in lcec_image.h).
///
/// lcec_image_init()      rtapi_app_main, after PDO registration of every master:
///                        sizes, catalog, one shared memory segment.
/// lcec_image_activated() master activation (RT context on the initf path):
///                        checks the domain sizes against the catalog, no allocation.
/// lcec_image_publish()   every lcec_write_master() cycle, before the frame is sent:
///                        one memcpy per Sync Unit domain into the next slot.
/// lcec_image_exit()      lcec_clear_config(): marks the masters stopped, removes the key.

#include "lcec_image.h"

#include "lcec.h"

#ifndef __KERNEL__
#include <sys/ipc.h>
#include <sys/shm.h>
#endif

/// Ring depth in milliseconds of master cycles; 0 disables the ring.
/// Owner decision 20.10.2026: 20 s = the scope window (docs/reference/lcec_process_image_ring_2026_10_20.md).
/// The decimation for the scope is the reader's business: the writer stores every cycle.
int image_depth_ms = 20000;
RTAPI_MP_INT(image_depth_ms, "process image ring depth in ms of master cycles, 0 = off");

extern int lcec_comp_id;

static int image_shmem_id = -1;
static lcec_image_header_t *image_hdr = NULL;

#define IMAGE_ALIGN8(x) (((x) + 7u) & ~(uint64_t)7u)

static void image_name(char *dst, const char *src) {
  rtapi_snprintf(dst, LCEC_IMAGE_NAME_LEN, "%s", src);
}

/// Bit length and direction of a registered entry, from the PDO configuration the
/// slave was given (driver sync_info or the generic slave's XML). 0 = not configured
/// there (the slave keeps its SII default mapping).
static void image_entry_info(const lcec_slave_t *slave, uint16_t index, uint8_t subindex, uint16_t *bit_length, uint8_t *dir) {
  const ec_sync_info_t *sm;
  unsigned int p, e;

  *bit_length = 0;
  *dir = LCEC_IMAGE_DIR_UNKNOWN;
  if (slave->sync_info == NULL) return;
  for (sm = slave->sync_info; sm->index != 0xff; sm++) {
    for (p = 0; p < sm->n_pdos && sm->pdos != NULL; p++) {
      const ec_pdo_info_t *pdo = &sm->pdos[p];
      for (e = 0; e < pdo->n_entries && pdo->entries != NULL; e++) {
        if (pdo->entries[e].index == index && pdo->entries[e].subindex == subindex) {
          *bit_length = pdo->entries[e].bit_length;
          *dir = (sm->dir == EC_DIR_OUTPUT) ? LCEC_IMAGE_DIR_OUTPUT : (sm->dir == EC_DIR_INPUT) ? LCEC_IMAGE_DIR_INPUT : LCEC_IMAGE_DIR_UNKNOWN;
          return;
        }
      }
    }
  }
}

static int image_slave_entry_count(const lcec_slave_t *slave) { return (slave->regs != NULL) ? slave->regs->current : 0; }

/// Remove a segment left by an earlier lcec under our key: readers still attached keep
/// their mapping (state STOPPED tells them), the key is free for the new layout.
static void image_drop_key(void) {
#ifndef __KERNEL__
  int id = shmget((key_t)LCEC_IMAGE_SHMEM_KEY, 0, 0);
  if (id >= 0) shmctl(id, IPC_RMID, NULL);
#endif
}

int lcec_image_init(lcec_master_t *first) {
  lcec_master_t *master;
  lcec_sync_unit_t *sync_unit;
  lcec_slave_t *slave;
  uint32_t master_count = 0, i;
  uint64_t off, total;
  void *ptr;

  for (master = first; master != NULL; master = master->next) master_count++;
  if (master_count == 0) return 0;

  // pass 1: sizes; pass 2: fill. Both walk the same lists in the same order.
  off = IMAGE_ALIGN8(sizeof(lcec_image_header_t)) + IMAGE_ALIGN8((uint64_t)master_count * sizeof(lcec_image_master_t));
  for (master = first; master != NULL; master = master->next) {
    uint64_t image_len = 0, depth = 0;
    int entries = 0;
    for (sync_unit = master->first_sync_unit; sync_unit != NULL; sync_unit = sync_unit->next) image_len += (uint64_t)ecrt_domain_size(sync_unit->domain);
    for (slave = master->first_slave; slave != NULL; slave = slave->next) entries += image_slave_entry_count(slave);
    if (image_depth_ms > 0 && master->app_time_period > 0 && master->sync_unit_count <= LCEC_IMAGE_MAX_UNITS) {
      depth = ((uint64_t)image_depth_ms * 1000000ull + master->app_time_period - 1) / master->app_time_period;
    }
    off += IMAGE_ALIGN8((uint64_t)master->sync_unit_count * sizeof(lcec_image_unit_t));
    off += IMAGE_ALIGN8((uint64_t)entries * sizeof(lcec_image_entry_t));
    off += depth * IMAGE_ALIGN8(sizeof(lcec_image_slot_t) + image_len);
  }
  total = off;
  if (total > 0x7fffffffull) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "process image ring: %llu bytes do not fit one segment, ring off (lower image_depth_ms)\n",
        (unsigned long long)total);
    return 0;
  }

  image_drop_key();
  image_shmem_id = rtapi_shmem_new(LCEC_IMAGE_SHMEM_KEY, lcec_comp_id, (unsigned long)total);
  if (image_shmem_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "process image ring: shmem of %llu bytes failed, ring off\n", (unsigned long long)total);
    image_shmem_id = -1;
    return 0;
  }
  if (lcec_rtapi_shmem_getptr(image_shmem_id, &ptr) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "process image ring: shmem pointer failed, ring off\n");
    rtapi_shmem_delete(image_shmem_id, lcec_comp_id);
    image_shmem_id = -1;
    return 0;
  }
  memset(ptr, 0, total);
  image_hdr = (lcec_image_header_t *)ptr;

  image_hdr->version = LCEC_IMAGE_VERSION;
  image_hdr->total_size = total;
  image_hdr->created_ns = rtapi_get_time();
  image_hdr->master_count = master_count;
  image_hdr->masters_off = IMAGE_ALIGN8(sizeof(lcec_image_header_t));
  image_hdr->depth_ms = image_depth_ms > 0 ? image_depth_ms : 0;

  off = image_hdr->masters_off + IMAGE_ALIGN8((uint64_t)master_count * sizeof(lcec_image_master_t));
  for (master = first, i = 0; master != NULL; master = master->next, i++) {
    lcec_image_master_t *m = (lcec_image_master_t *)((uint8_t *)ptr + image_hdr->masters_off) + i;
    lcec_image_unit_t *units;
    lcec_image_entry_t *ent;
    uint32_t u, image_len = 0;
    int entries = 0;

    image_name(m->name, master->name);
    m->index = master->index;
    m->period_ns = master->app_time_period;
    m->unit_count = master->sync_unit_count;
    m->units_off = off;
    units = (lcec_image_unit_t *)((uint8_t *)ptr + off);
    for (sync_unit = master->first_sync_unit, u = 0; sync_unit != NULL; sync_unit = sync_unit->next, u++) {
      image_name(units[u].name, sync_unit->name);
      units[u].image_off = image_len;
      units[u].len = (uint32_t)ecrt_domain_size(sync_unit->domain);
      units[u].cycle_ns = sync_unit->cycle_time;
      units[u].divider = sync_unit->cycle_divider;
      units[u].phase = sync_unit->phase;
      image_len += units[u].len;
    }
    off += IMAGE_ALIGN8((uint64_t)m->unit_count * sizeof(lcec_image_unit_t));

    // catalog: every registered entry, offsets as ecrt_domain_reg_pdo_entry_list filled them
    m->entries_off = off;
    ent = (lcec_image_entry_t *)((uint8_t *)ptr + off);
    for (slave = master->first_slave; slave != NULL; slave = slave->next) {
      int r, n = image_slave_entry_count(slave);
      uint32_t unit_idx = 0;
      lcec_sync_unit_t *su;
      for (su = master->first_sync_unit; su != NULL && su != slave->sync_unit; su = su->next) unit_idx++;
      for (r = 0; r < n; r++) {
        const ec_pdo_entry_reg_t *reg = &slave->regs->pdo_entry_regs[r];
        image_name(ent->slave_name, slave->name);
        ent->slave_pos = reg->position;
        ent->unit = unit_idx;
        ent->vid = reg->vendor_id;
        ent->pid = reg->product_code;
        ent->index = reg->index;
        ent->subindex = reg->subindex;
        ent->byte_off = units[unit_idx].image_off + (reg->offset != NULL ? *reg->offset : 0);
        ent->bit_pos = (reg->bit_position != NULL) ? *reg->bit_position : 0;
        image_entry_info(slave, reg->index, reg->subindex, &ent->bit_length, &ent->dir);
        ent++;
        entries++;
      }
    }
    m->entry_count = entries;
    off += IMAGE_ALIGN8((uint64_t)entries * sizeof(lcec_image_entry_t));

    m->image_len = image_len;
    m->slot_size = IMAGE_ALIGN8(sizeof(lcec_image_slot_t) + image_len);
    m->ring_off = off;
    if (image_depth_ms <= 0) {
      m->state = LCEC_IMAGE_STATE_OFF;
      m->off_reason = LCEC_IMAGE_OFF_DISABLED;
    } else if (master->app_time_period == 0) {
      m->state = LCEC_IMAGE_STATE_OFF;
      m->off_reason = LCEC_IMAGE_OFF_NO_PERIOD;
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "master %s: no appTimePeriod in the XML, process image ring off\n", master->name);
    } else if (master->sync_unit_count > LCEC_IMAGE_MAX_UNITS) {
      m->state = LCEC_IMAGE_STATE_OFF;
      m->off_reason = LCEC_IMAGE_OFF_TOO_MANY_UNITS;
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "master %s: more than %d Sync Units, process image ring off\n", master->name,
          LCEC_IMAGE_MAX_UNITS);
    } else {
      m->depth = ((uint64_t)image_depth_ms * 1000000ull + master->app_time_period - 1) / master->app_time_period;
      m->state = LCEC_IMAGE_STATE_ARMED;
      master->image = m;
    }
    off += (uint64_t)m->depth * m->slot_size;
    rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "master %s: process image ring %u slots x %u bytes, %u entries\n", master->name, m->depth,
        m->slot_size, m->entry_count);
  }

  // magic last: a reader that sees it sees the whole catalog
  __atomic_store_n(&image_hdr->magic, LCEC_IMAGE_MAGIC, __ATOMIC_RELEASE);
  return 0;
}

void lcec_image_activated(lcec_master_t *master) {
  lcec_image_master_t *m = master->image;
  lcec_image_unit_t *units;
  lcec_sync_unit_t *sync_unit;
  uint32_t u;

  if (m == NULL) return;
  units = (lcec_image_unit_t *)((uint8_t *)image_hdr + m->units_off);
  for (sync_unit = master->first_sync_unit, u = 0; sync_unit != NULL; sync_unit = sync_unit->next, u++) {
    if ((uint32_t)sync_unit->process_data_len != units[u].len) {
      __atomic_store_n(&m->off_reason, LCEC_IMAGE_OFF_SIZE_CHANGED, __ATOMIC_RELAXED);
      __atomic_store_n(&m->state, LCEC_IMAGE_STATE_OFF, __ATOMIC_RELEASE);
      master->image = NULL;
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "master %s syncUnit %s: domain size %d at activation, %u in the catalog, process image ring off\n",
          master->name, sync_unit->name, sync_unit->process_data_len, units[u].len);
      return;
    }
  }
}

void lcec_image_publish(lcec_master_t *master, int64_t tick, uint64_t app_time, long long now) {
  lcec_image_master_t *m = master->image;
  lcec_image_unit_t *units;
  lcec_image_slot_t *slot;
  lcec_sync_unit_t *sync_unit;
  uint64_t seq;
  uint32_t u, fresh = 0;
  uint8_t *data;

  if (m == NULL) return;
  units = (lcec_image_unit_t *)((uint8_t *)image_hdr + m->units_off);

  // the only writer: head is ours to read without ordering
  seq = m->head + 1;
  slot = (lcec_image_slot_t *)((uint8_t *)image_hdr + m->ring_off + ((seq - 1) % m->depth) * (uint64_t)m->slot_size);

  __atomic_store_n(&slot->seq, 0, __ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_RELEASE);

  data = LCEC_IMAGE_SLOT_DATA(slot);
  for (sync_unit = master->first_sync_unit, u = 0; sync_unit != NULL; sync_unit = sync_unit->next, u++) {
    memcpy(data + units[u].image_off, sync_unit->process_data, units[u].len);
    if (sync_unit->process) fresh |= 1u << u;
  }
  slot->tick = tick;
  slot->app_time = app_time;
  slot->mono_ns = now;
  slot->wkc = master->image_wkc;
  slot->fresh_mask = fresh;

  __atomic_store_n(&slot->seq, seq, __ATOMIC_RELEASE);
  __atomic_store_n(&m->head, seq, __ATOMIC_RELEASE);
  if (m->state != LCEC_IMAGE_STATE_RUNNING) __atomic_store_n(&m->state, LCEC_IMAGE_STATE_RUNNING, __ATOMIC_RELEASE);
}

void lcec_image_exit(lcec_master_t *first) {
  lcec_master_t *master;
  uint32_t i;

  for (master = first; master != NULL; master = master->next) master->image = NULL;
  if (image_hdr != NULL) {
    lcec_image_master_t *m = (lcec_image_master_t *)((uint8_t *)image_hdr + image_hdr->masters_off);
    for (i = 0; i < image_hdr->master_count; i++) __atomic_store_n(&m[i].state, LCEC_IMAGE_STATE_STOPPED, __ATOMIC_RELEASE);
    image_hdr = NULL;
  }
  if (image_shmem_id >= 0) {
    // detach (removes the segment when no reader is attached), then free the key a reader still holds
    rtapi_shmem_delete(image_shmem_id, lcec_comp_id);
    image_shmem_id = -1;
    image_drop_key();
  }
}
