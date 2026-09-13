//
// flash_map.h - the top-of-flash region map, in ONE place.
//
// Regions are anchored to the TOP of flash and grow DOWNWARD, so every region
// is a potential neighbour of the next one down. Keeping the map split across
// files is how a capability constant raised in one place silently overruns a
// layout somewhere else. Everything that claims a sector declares it here, and
// the static_asserts below are the enforcement.
//
//   -1   legacy config location (superseded; left blank)
//   -2   legacy slots location (superseded; left blank)
//   -3   ACTIVE CONFIG
//   -4 .. -19   SLOT RESERVATION (SLOT_SECTORS_RESERVED)
//   -20  MACRO TABLE, growing downward if it ever needs a second sector
//   -22/-21  btstack TLV BANK (pinned in CMakeLists - see the trap below)
//
// THE BTSTACK BANK TRAP. The SDK default for PICO_FLASH_BANK_STORAGE_OFFSET is
// PICO_FLASH_SIZE_BYTES - PICO_FLASH_BANK_TOTAL_SIZE on RP2040, but
// PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE - PICO_FLASH_BANK_TOTAL_SIZE on
// RP2350 - one sector lower, which lands the bank (two sectors) exactly on the
// ACTIVE CONFIG sector above. btstack erases a whole bank sector per TLV write,
// so on a Pico 2 W every controller connect / disconnect / re-pair wiped the
// saved settings, while the save itself verified perfectly against flash. That is
// why the bank is pinned explicitly in CMakeLists.txt below every region here,
// and why the assert at the bottom of this file holds it there.
//

#ifndef DS5_BRIDGE_FLASH_MAP_H
#define DS5_BRIDGE_FLASH_MAP_H

#include "config.h"          // SLOT_COUNT, SLOTS_PER_SECTOR
#include "hardware/flash.h"  // FLASH_SECTOR_SIZE, FLASH_PAGE_SIZE

// Slot sector 0 stays at -4 forever so pre-existing slots are preserved in
// place; extra sectors grow downward.
static constexpr uint32_t slot_sector_offset_at(uint8_t sector) {
    return PICO_FLASH_SIZE_BYTES - (4u + sector) * FLASH_SECTOR_SIZE;
}

// --- Slot growth reservation -------------------------------------------------
// At SLOT_COUNT=24 the last slot sector is -6; at 32 it would be -7, at 64 -11.
// Anything parked immediately below the last slot sector is overrun the moment
// SLOT_COUNT is raised, so reserve the window up front and place every later
// region BELOW it. Raising SLOT_COUNT then stays a one-constant change.
//
// The reservation is free: the firmware image is ~723 KB, leaving ~837 spare
// sectors on the 4 MB Pico 2 W and far more on the 16 MB Waveshare.
//
// DO NOT anchor a neighbouring region to the CURRENT slot count (e.g.
// slot_sector_offset_at(SLOT_COUNT / SLOTS_PER_SECTOR)). It looks
// self-maintaining but moves the neighbour on every SLOT_COUNT change,
// orphaning its old contents AND letting slot writes land on top of them.
constexpr uint8_t SLOT_SECTORS_RESERVED = 16; // headroom for SLOT_COUNT up to 128

static_assert(SLOT_COUNT % SLOTS_PER_SECTOR == 0, "whole sectors only");
static_assert(SLOT_COUNT / SLOTS_PER_SECTOR <= SLOT_SECTORS_RESERVED,
              "SLOT_COUNT raised past its reservation - slot sectors would overrun "
              "the macro table. Raising SLOT_SECTORS_RESERVED MOVES the macro sector, "
              "which is a flash migration, not a constant bump.");

// --- Macro table -------------------------------------------------------------
constexpr uint32_t MACRO_FLASH_OFFSET =
    PICO_FLASH_SIZE_BYTES - (4u + SLOT_SECTORS_RESERVED) * FLASH_SECTOR_SIZE;

static_assert(MACRO_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0);
static_assert(MACRO_FLASH_OFFSET < slot_sector_offset_at(SLOT_SECTORS_RESERVED - 1),
              "macro sector must sit strictly below every reserved slot sector");

// --- btstack TLV bank --------------------------------------------------------
// Pinned in CMakeLists.txt (PICO_FLASH_BANK_STORAGE_OFFSET) instead of taking the
// SDK default, which collides with the active-config sector on RP2350. Declared
// here so the whole map is in one place and the collision cannot come back.
constexpr uint32_t BT_BANK_FLASH_OFFSET = PICO_FLASH_SIZE_BYTES - 22 * FLASH_SECTOR_SIZE;
constexpr uint32_t BT_BANK_SIZE         = 2 * FLASH_SECTOR_SIZE;  // two banks, one sector each
static_assert(BT_BANK_FLASH_OFFSET + BT_BANK_SIZE <= MACRO_FLASH_OFFSET,
              "btstack's TLV bank must sit strictly below the macro table");
static_assert(BT_BANK_FLASH_OFFSET != PICO_FLASH_SIZE_BYTES - 3 * FLASH_SECTOR_SIZE,
              "btstack's TLV bank must never share the active-config sector");

#endif // DS5_BRIDGE_FLASH_MAP_H
