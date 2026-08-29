/* ================================================================================================
 * File: heap.h
 * Brief: C/C++ memory allocation and stats.
 *        NOTE: Shared header between C and C++.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#ifndef PS2_SYSTEM_HEAP_H
#define PS2_SYSTEM_HEAP_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// NOTE: Be sure to update s_memTagNames[] in heap.cpp when changing this enum!
typedef enum
{
    MEMTAG_ELF_SYS,    // Memory used by the system/kernel and the estimate size of the ELF executable.
    MEMTAG_OPNEW,      // C++ operator new/new[].
    MEMTAG_QUAKE,      // Game allocations: Z_Malloc/Z_TagMalloc/etc.
    MEMTAG_RENDERER,   // Things related to rendering / the refresh module.
    MEMTAG_TEXIMAGE,   // Allocs related to images/textures/palettes.
    MEMTAG_MDL_ALIAS,  // MD2/Alias models.
    MEMTAG_MDL_SPRITE, // Sprite models.
    MEMTAG_MDL_WORLD,  // World geometry.
    MEMTAG_LIGHTMAP,   // Lightmap atlas buffers (see renderer/lightmap.cpp).
    MEMTAG_AUDIO,      // Decoded sound cache. Its own tag because it is one of the largest pools in the game.
    MEMTAG_COUNT,      // Number of entries in this enum. Internal use only.
} PS2MemTag;

void * PS2_MemAlloc(size_t sizeBytes, PS2MemTag tag);
void * PS2_MemAllocAligned(size_t alignment, size_t sizeBytes, PS2MemTag tag);
void PS2_MemFree(void * ptr, size_t sizeBytes, PS2MemTag tag);
void PS2_TagsAddMem(PS2MemTag tag, size_t sizeBytes);

// Total EE RAM as reported by the kernel (32MB on a retail console).
size_t PS2_GetTotalMemBytes();

// EE RAM still available to the program right now: the gap between the current
// program break and the ceiling the kernel set up for the heap.
size_t PS2_GetAvailableMemBytes();

// Books the RAM that the game can never allocate - EE kernel, the loaded ELF
// image (text/data/bss), the main thread stack - into MEMTAG_ELF_SYS. Call once,
// as early as possible in main().
void PS2_TagsAddSystemMem();

// Formats a byte count as "12.34 MB" (abbreviated) or "12.34 Megabytes" into the
// caller's buffer, and returns it so the call can be nested in a printf argument
// list. Takes the buffer rather than owning a static one so two calls in the same
// format string don't overwrite each other.
enum { PS2_MEMUNIT_STR_SIZE = 32 };
const char * PS2_FormatMemoryUnit(size_t memorySizeInBytes, int abbreviated,
                                  char * outBuffer, size_t outBufferSize);

typedef struct
{
    size_t totalBytes;
    size_t peakBytes; // High-water mark of totalBytes. See PS2_GetPeakMemBytes.
    size_t totalAllocs;
    size_t totalFrees;
    size_t smallestAlloc;
    size_t largestAlloc;
} PS2MemStats;

const PS2MemStats * PS2_GetStatsForMemTag(PS2MemTag tag);
const char * PS2_GetNameForMemTag(PS2MemTag tag);

// High-water mark of the sum of every tag - the largest the game was ever holding
// at one time. Not the same as adding up the per-tag peaks: those happen at
// different moments. This is the number that says whether a map change fits, since
// the transient (old map still resident while the new one loads) never shows up in
// a steady-state reading.
size_t PS2_GetPeakMemBytes();

// Renders the whole memory-tag table into the caller's buffer and returns it.
// Truncates rather than overrunning if the buffer is short; PS2_MEMTAGS_DUMP_SIZE
// is big enough for the full table.
enum { PS2_MEMTAGS_DUMP_SIZE = 2048 };
const char * PS2_DumpMemTags(char * outBuffer, size_t outBufferSize);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // PS2_SYSTEM_HEAP_H
