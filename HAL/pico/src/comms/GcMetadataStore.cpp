#include "comms/GcMetadataStore.hpp"

#include <Arduino.h>
#include <gamecube_definitions.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <pico/platform.h>
#include <string.h>

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

static constexpr uint32_t metadata_magic = 0x4842594d; // "MYBH"

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t chunks;
    uint8_t data[gc_metadata_max_size];
} persisted_gc_metadata_t;

static_assert(
    sizeof(persisted_gc_metadata_t) <= FLASH_SECTOR_SIZE,
    "metadata persistence exceeds one flash sector"
);

static constexpr uint32_t metadata_flash_offset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
static const persisted_gc_metadata_t *const metadata_flash =
    (const persisted_gc_metadata_t *)(XIP_BASE + metadata_flash_offset);

// Keep flash staging buffer out of stack. A 4KB stack allocation in the commit
// path can overflow on RP2040/Arduino and crash after a seemingly successful
// save.
static uint8_t metadata_sector_buf[FLASH_SECTOR_SIZE];

void gc_metadata_build_default(uint8_t *metadata, uint8_t &chunks) {
    memset(metadata, 0, gc_metadata_max_size);

    uint8_t *p = metadata;
    int idx = 0;
    const char *key = "firmware";
    const char *value = "HayBox";

    // UBJSON: { U <len_key> <key> S U <len_val> <value> }
    p[idx++] = 0x7B;
    p[idx++] = 'U';
    p[idx++] = (uint8_t)strlen(key);
    memcpy(&p[idx], key, strlen(key));
    idx += (int)strlen(key);

    p[idx++] = 'S';
    p[idx++] = 'U';
    p[idx++] = (uint8_t)strlen(value);
    memcpy(&p[idx], value, strlen(value));
    idx += (int)strlen(value);

    p[idx++] = 0x7D;
    chunks = 1;
}

bool gc_metadata_load(uint8_t *metadata, uint8_t &chunks) {
    if (metadata_flash->magic != metadata_magic) {
        return false;
    }
    if (metadata_flash->chunks == 0 || metadata_flash->chunks > gc_metadata_max_chunks) {
        return false;
    }
    chunks = metadata_flash->chunks;
    memcpy(metadata, metadata_flash->data, gc_metadata_max_size);
    return true;
}

static void __no_inline_not_in_flash_func(program_persisted_metadata)(
    const uint8_t *metadata,
    uint8_t chunks
) {
    memset(metadata_sector_buf, 0xFF, sizeof(metadata_sector_buf));

    persisted_gc_metadata_t persisted;
    persisted.magic = metadata_magic;
    persisted.chunks = chunks;
    memcpy(persisted.data, metadata, gc_metadata_max_size);
    memcpy(metadata_sector_buf, &persisted, sizeof(persisted));

    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(metadata_flash_offset, FLASH_SECTOR_SIZE);
    flash_range_program(metadata_flash_offset, metadata_sector_buf, FLASH_SECTOR_SIZE);
    restore_interrupts(interrupts);
}

void gc_metadata_save(const uint8_t *metadata, uint8_t chunks) {
    if (chunks == 0) {
        return;
    }
    // Skip the erase/program if flash already matches.
    if (metadata_flash->magic == metadata_magic && metadata_flash->chunks == chunks &&
        memcmp(metadata_flash->data, metadata, gc_metadata_max_size) == 0) {
        return;
    }
    // Park the other core for the flash erase/program window, then write.
    rp2040.idleOtherCore();
    program_persisted_metadata(metadata, chunks);
    rp2040.resumeOtherCore();
}
