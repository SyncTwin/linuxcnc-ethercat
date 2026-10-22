//
//    Copyright (C) 2026 SyncTwin
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//

/// @file
/// @brief Process image ring: layout of the shared memory segment.
///
/// Every master cycle lcec copies the process image of each master (all of its
/// Sync Unit domains, concatenated) into a ring of slots in one SysV shared
/// memory segment, the way a TwinCAT Scope records the process image of a task.
/// The bus thread is the only writer and never waits; any number of user
/// processes read the segment without stealing rows from each other.
///
/// This header is the one home of the layout: the writer (lcec_image.c) and
/// every reader include it. It only uses fixed-size types, so a reader does
/// not need lcec.h, HAL or ecrt.h.
///
/// Segment:
///
///   lcec_image_header_t
///   lcec_image_master_t  [master_count]           at header.masters_off
///   per master:
///     lcec_image_unit_t  [unit_count]             at master.units_off
///     lcec_image_entry_t [entry_count]            at master.entries_off
///     slot               [depth] of slot_size     at master.ring_off
///       slot = lcec_image_slot_t + image_len bytes of image
///
/// All offsets are bytes from the start of the segment.
///
/// Reading (seqlock per slot, no locks shared with the bus thread):
///
///   1. attach: shmget(LCEC_IMAGE_SHMEM_KEY, 0, 0), shmat(SHM_RDONLY);
///      check magic, version, total_size.
///   2. head = __atomic_load_n(&m->head, ACQUIRE): sequence number of the last
///      published slot (0: nothing published yet). Slot of sequence s is
///      (s - 1) % depth. The oldest readable sequence is head - depth + 1.
///   3. s1 = __atomic_load_n(&slot->seq, ACQUIRE); s1 must equal the wanted s;
///      copy the slot; __atomic_thread_fence(ACQUIRE);
///      s2 = __atomic_load_n(&slot->seq, RELAXED); the copy is valid iff
///      s1 == s2 == s. Otherwise the writer lapped the reader: skip the row.
///   4. state != LCEC_IMAGE_STATE_RUNNING or a new shmid for the key means the
///      writer is gone or reloaded (lcec reload removes the key first): re-attach
///      and re-read the catalog.
///
/// Sequence numbers count master cycles without gaps while the master runs;
/// tick is the master's DC grid index of the cycle (lcec_master_t.next_tick).

#ifndef _LCEC_IMAGE_H_
#define _LCEC_IMAGE_H_

#include <stdint.h>

#define LCEC_IMAGE_SHMEM_KEY 0xACB572C8  ///< Next to LCEC_CONF_SHMEM_KEY (0xACB572C7).
#define LCEC_IMAGE_MAGIC 0x4C434952      ///< "LCIR"
#define LCEC_IMAGE_VERSION 1
#define LCEC_IMAGE_NAME_LEN 48  ///< = LCEC_CONF_STR_MAXLEN: names of the XML are not cut.
#define LCEC_IMAGE_MAX_UNITS 32  ///< fresh_mask has one bit per Sync Unit.

/// Ring state of one master.
enum {
  LCEC_IMAGE_STATE_OFF = 0,      ///< No ring for this master; see off_reason.
  LCEC_IMAGE_STATE_ARMED = 1,    ///< Catalog written, master not yet cycling.
  LCEC_IMAGE_STATE_RUNNING = 2,  ///< Slots are being published.
  LCEC_IMAGE_STATE_STOPPED = 3,  ///< lcec unloaded; the segment is a leftover.
};

/// Why a master has no ring (state OFF).
enum {
  LCEC_IMAGE_OFF_NONE = 0,
  LCEC_IMAGE_OFF_DISABLED = 1,      ///< image_depth_ms=0.
  LCEC_IMAGE_OFF_NO_PERIOD = 2,     ///< The XML gives no appTimePeriod: depth in slots unknown.
  LCEC_IMAGE_OFF_TOO_MANY_UNITS = 3,
  LCEC_IMAGE_OFF_SIZE_CHANGED = 4,  ///< Domain size at activation differs from the catalog.
};

/// Direction of a PDO entry, as ec_direction_t of ecrt.h.
enum {
  LCEC_IMAGE_DIR_UNKNOWN = 0,
  LCEC_IMAGE_DIR_OUTPUT = 1,  ///< RxPDO of the slave: master -> slave.
  LCEC_IMAGE_DIR_INPUT = 2,   ///< TxPDO of the slave: slave -> master.
};

typedef struct {
  uint32_t magic;          ///< LCEC_IMAGE_MAGIC
  uint32_t version;        ///< LCEC_IMAGE_VERSION
  uint64_t total_size;     ///< Bytes of the whole segment.
  uint64_t created_ns;     ///< rtapi_get_time() when lcec created the segment.
  uint32_t master_count;   ///< Entries of lcec_image_master_t.
  uint32_t masters_off;    ///< Offset of lcec_image_master_t[master_count].
  uint32_t depth_ms;       ///< Requested depth (modparam image_depth_ms).
  uint32_t reserved[7];
} lcec_image_header_t;

typedef struct lcec_image_master_s {
  char name[LCEC_IMAGE_NAME_LEN];  ///< Master name of the XML (pins lcec.<name>.*).
  int32_t index;                   ///< IgH master index.
  uint32_t state;                  ///< LCEC_IMAGE_STATE_*
  uint32_t off_reason;             ///< LCEC_IMAGE_OFF_* when state is OFF.
  uint32_t period_ns;              ///< Master cycle (appTimePeriod): one slot per period.
  uint32_t unit_count;
  uint32_t units_off;
  uint32_t entry_count;
  uint32_t entries_off;
  uint32_t image_len;   ///< Bytes of image in a slot: all unit domains, concatenated.
  uint32_t slot_size;   ///< sizeof(lcec_image_slot_t) + image_len, rounded up to 8.
  uint32_t depth;       ///< Slots in the ring.
  uint32_t reserved0;
  uint64_t ring_off;    ///< Offset of slot 0.
  uint64_t head;        ///< Sequence of the last published slot, 0 = none. Atomic.
  uint32_t reserved[4];
} lcec_image_master_t;

typedef struct {
  char name[LCEC_IMAGE_NAME_LEN];  ///< Sync Unit name ("" for the master's default unit).
  uint32_t image_off;              ///< Offset of this unit's domain inside the slot image.
  uint32_t len;                    ///< Domain size in bytes.
  uint32_t cycle_ns;               ///< Unit cycle.
  uint32_t divider;                ///< Master cycles per unit cycle.
  uint32_t phase;                  ///< Master cycle within the unit cycle the domain is sent on.
  uint32_t reserved[3];
} lcec_image_unit_t;

/// One registered PDO entry (the entries lcec drivers map to HAL pins).
typedef struct {
  char slave_name[LCEC_IMAGE_NAME_LEN];  ///< Slave name of the XML (pins lcec.<m>.<name>.*).
  uint16_t slave_pos;                    ///< Ring position of the slave.
  uint16_t unit;                         ///< Index into the master's units.
  uint32_t vid;
  uint32_t pid;
  uint16_t index;       ///< Object index (e.g. 0x6064).
  uint8_t subindex;
  uint8_t dir;          ///< LCEC_IMAGE_DIR_*
  uint32_t byte_off;    ///< Offset inside the slot image (unit image_off + domain offset).
  uint8_t bit_pos;      ///< Bit inside the byte (0 for byte-aligned entries).
  uint8_t reserved0;
  uint16_t bit_length;  ///< 0 = not in the slave's PDO configuration (SII default mapping).
  uint32_t reserved[2];
} lcec_image_entry_t;

typedef struct {
  uint64_t seq;         ///< Sequence of the cycle; 0 while being written. Atomic.
  int64_t tick;         ///< DC grid index of the cycle (master next_tick).
  uint64_t app_time;    ///< DC application time sent this cycle (ns).
  int64_t mono_ns;      ///< rtapi_get_time() sampled with app_time.
  uint32_t wkc;         ///< Sum of the unit domains' working counters at the last receive.
  uint32_t fresh_mask;  ///< Bit u: unit u received inputs in this cycle (others hold their last image).
} lcec_image_slot_t;

#define LCEC_IMAGE_SLOT_DATA(slot) ((uint8_t *)(slot) + sizeof(lcec_image_slot_t))

#ifdef __cplusplus
static_assert(sizeof(lcec_image_header_t) == 64, "lcec_image_header_t layout");
static_assert(sizeof(lcec_image_master_t) == 128, "lcec_image_master_t layout");
static_assert(sizeof(lcec_image_unit_t) == 80, "lcec_image_unit_t layout");
static_assert(sizeof(lcec_image_entry_t) == 80, "lcec_image_entry_t layout");
static_assert(sizeof(lcec_image_slot_t) == 40, "lcec_image_slot_t layout");
#else
_Static_assert(sizeof(lcec_image_header_t) == 64, "lcec_image_header_t layout");
_Static_assert(sizeof(lcec_image_master_t) == 128, "lcec_image_master_t layout");
_Static_assert(sizeof(lcec_image_unit_t) == 80, "lcec_image_unit_t layout");
_Static_assert(sizeof(lcec_image_entry_t) == 80, "lcec_image_entry_t layout");
_Static_assert(sizeof(lcec_image_slot_t) == 40, "lcec_image_slot_t layout");
#endif

#endif
