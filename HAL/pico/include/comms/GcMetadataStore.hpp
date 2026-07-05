#ifndef _COMMS_GCMETADATASTORE_HPP
#define _COMMS_GCMETADATASTORE_HPP

#include <stdint.h>

// Shared flash-backed persistence for GameCube controller metadata.
//
// The same on-flash sector and record format are used by both GamecubeBackend
// (physical Joybus personality) and WebUSBBackend (USB personality), so the two
// interoperate on a single persisted metadata object. Keep this the one place
// that knows the magic/offset/layout.

// Load persisted metadata into `out` (must be gc_metadata_max_size bytes).
// Returns false if flash holds no valid record; `chunks` is written only on
// success.
bool gc_metadata_load(uint8_t *out, uint8_t &chunks);

// Fill `out` (gc_metadata_max_size bytes) with the default metadata object
// ({"firmware":"HayBox"}) and set `chunks` to its chunk count.
void gc_metadata_build_default(uint8_t *out, uint8_t &chunks);

// Persist `in` (gc_metadata_max_size bytes). No-op if chunks == 0 or if flash
// already matches. Parks the other core across the erase/program window, so it
// must run on core0 outside any USB/PIO interrupt context.
void gc_metadata_save(const uint8_t *in, uint8_t chunks);

#endif
