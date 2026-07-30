/* ================================================================================================
 * File: heap.cpp
 * Brief: C++ side of the program-wide dlmalloc heap. Provides operator new/delete
 *        (routed to the dlmalloc-backed global malloc from dlmalloc.c) and the
 *        tag-accounting layer, which common.c's Z_Malloc and the renderer allocate through.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/system/heap.h"
#include "ps2/common.h" // Sys_Error, etc

#include <new>
#include <cstdio>  // snprintf
#include <cstring> // memset
#include <cstdint> // uintptr_t
#include <unistd.h> // sbrk

#include <kernel.h> // EndOfHeap, GetMemorySize

// dlmalloc
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow" // issue with mallinfo
extern "C" {
    #define USE_DL_PREFIX 1
    #include "dlmalloc/malloc.h"
}
#pragma GCC diagnostic pop

// ------------------------------------------------------------------------------------------------
// operator new / delete -> dlmalloc global heap
//
// Built with -fno-exceptions, so we cannot throw std::bad_alloc; a failed
// allocation is a fatal Sys_Error instead. delete is always noexcept.
// ------------------------------------------------------------------------------------------------

void * operator new(std::size_t n)   { return PS2_MemAlloc(n, MEMTAG_OPNEW); }
void * operator new[](std::size_t n) { return PS2_MemAlloc(n, MEMTAG_OPNEW); }

void * operator new(std::size_t n, const std::nothrow_t &)   noexcept { return PS2_MemAlloc(n, MEMTAG_OPNEW); }
void * operator new[](std::size_t n, const std::nothrow_t &) noexcept { return PS2_MemAlloc(n, MEMTAG_OPNEW); }

void * operator new(std::size_t n, std::align_val_t al)   { return PS2_MemAllocAligned(static_cast<std::size_t>(al), n, MEMTAG_OPNEW); }
void * operator new[](std::size_t n, std::align_val_t al) { return PS2_MemAllocAligned(static_cast<std::size_t>(al), n, MEMTAG_OPNEW); }

void operator delete(void * p)   noexcept { PS2_MemFree(p, dlmalloc_usable_size(p), MEMTAG_OPNEW); }
void operator delete[](void * p) noexcept { PS2_MemFree(p, dlmalloc_usable_size(p), MEMTAG_OPNEW); }

void operator delete(void * p, std::size_t n)   noexcept { PS2_MemFree(p, n, MEMTAG_OPNEW); }
void operator delete[](void * p, std::size_t n) noexcept { PS2_MemFree(p, n, MEMTAG_OPNEW); }

void operator delete(void * p, const std::nothrow_t &)   noexcept { PS2_MemFree(p, dlmalloc_usable_size(p), MEMTAG_OPNEW); }
void operator delete[](void * p, const std::nothrow_t &) noexcept { PS2_MemFree(p, dlmalloc_usable_size(p), MEMTAG_OPNEW); }

void operator delete(void * p, std::align_val_t)   noexcept { PS2_MemFree(p, dlmalloc_usable_size(p), MEMTAG_OPNEW); }
void operator delete[](void * p, std::align_val_t) noexcept { PS2_MemFree(p, dlmalloc_usable_size(p), MEMTAG_OPNEW); }

void operator delete(void * p, std::size_t n, std::align_val_t)   noexcept { PS2_MemFree(p, n, MEMTAG_OPNEW); }
void operator delete[](void * p, std::size_t n, std::align_val_t) noexcept { PS2_MemFree(p, n, MEMTAG_OPNEW); }

// ------------------------------------------------------------------------------------------------
// Tagged allocation + memory accounting.
// C linkage (declared extern "C" in heap.h) so common.c can call these.
// ------------------------------------------------------------------------------------------------

static PS2MemStats s_memTagCounts[MEMTAG_COUNT] = {};

// NOTE: These should match the PS2MemTag declaration order!
static const char * const s_memTagNames[MEMTAG_COUNT] = {
    "Misc",
    "OpNew",
    "Quake",
    "Renderer",
    "TexImage",
    "Mdl_Alias",
    "Mdl_Sprite",
    "Mdl_World",
};

static inline size_t MemTagToIndex(PS2MemTag tag)
{
    const int t = static_cast<int>(tag);
    return (t >= 0 && t < MEMTAG_COUNT) ? static_cast<size_t>(t) : static_cast<size_t>(MEMTAG_MISC);
}

static inline void AccountAlloc(PS2MemTag tag, size_t bytes)
{
    PS2MemStats * c = &s_memTagCounts[MemTagToIndex(tag)];
    c->totalBytes  += bytes;
    c->totalAllocs += 1u;
    if (c->smallestAlloc == 0u || bytes < c->smallestAlloc) { c->smallestAlloc = bytes; }
    if (bytes > c->largestAlloc) { c->largestAlloc = bytes; }
}

// ------------------------------------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------------------------------------

extern "C" {

void * PS2_MemAlloc(size_t sizeBytes, PS2MemTag tag)
{
    const size_t n = (sizeBytes != 0u ? sizeBytes : 1u);
    void * p = dlmalloc(n);
    if (p == nullptr)
    {
        Sys_Error("PS2_MemAlloc: failed to allocate %zu bytes (%s)\n"
                  "%s", n, s_memTagNames[MemTagToIndex(tag)], PS2_DumpMemTags());
    }
    AccountAlloc(tag, n);
    return p;
}

void * PS2_MemAllocAligned(size_t alignment, size_t sizeBytes, PS2MemTag tag)
{
    const size_t n = (sizeBytes != 0u ? sizeBytes : 1u);
    void * p = dlmemalign(alignment, n);
    if (p == nullptr)
    {
        Sys_Error("PS2_MemAllocAligned: failed to allocate %zu bytes (align %zu, %s)\n"
                  "%s", n, alignment, s_memTagNames[MemTagToIndex(tag)], PS2_DumpMemTags());
    }
    AccountAlloc(tag, n);
    return p;
}

void PS2_MemFree(void * ptr, size_t sizeBytes, PS2MemTag tag)
{
    if (ptr == nullptr) { return; }
    PS2MemStats * c = &s_memTagCounts[MemTagToIndex(tag)];
    c->totalFrees += 1u;
    if (sizeBytes != 0u)
    {
        c->totalBytes = (c->totalBytes >= sizeBytes) ? (c->totalBytes - sizeBytes) : 0u;
    }
    dlfree(ptr);
}

void PS2_TagsAddMem(PS2MemTag tag, size_t sizeBytes)
{
    s_memTagCounts[MemTagToIndex(tag)].totalBytes += sizeBytes;
}

size_t PS2_GetTotalMemBytes()
{
    // GetMemorySize() is an EE kernel syscall returning the installed RAM in
    // bytes. Retail consoles answer 32MB; fall back to that if it ever fails.
    const s32 memSize = GetMemorySize();
    return (memSize > 0) ? static_cast<size_t>(memSize) : (32u * 1024u * 1024u);
}

size_t PS2_GetAvailableMemBytes()
{
    // sbrk(0) hands back the current program break without moving it. crt0 hands
    // the kernel [_end, all remaining RAM) via SetupHeap, so the break starts at
    // _end (just past our bss) and only ever grows - and since dlmalloc.c routes
    // every allocator in the link through dlmalloc, dlmalloc is the sole caller
    // of sbrk. The gap up to EndOfHeap() is therefore exactly the RAM nothing has
    // claimed yet. Note this is *unclaimed* memory: blocks dlmalloc has already
    // sbrk'd and since freed sit in its free lists and do not show up here.
    const std::uintptr_t brk = reinterpret_cast<std::uintptr_t>(sbrk(0));
    const std::uintptr_t top = reinterpret_cast<std::uintptr_t>(EndOfHeap());

    if (brk == 0u || brk == ~static_cast<std::uintptr_t>(0) || top <= brk)
    {
        return 0u; // sbrk failed, or the heap is exhausted
    }
    return static_cast<size_t>(top - brk);
}

void PS2_TagsAddSystemMem()
{
    const size_t totalBytes = PS2_GetTotalMemBytes();
    const size_t availBytes = PS2_GetAvailableMemBytes();

    // Everything the game will never get to allocate: the first megabyte of RAM
    // reserved for the EE kernel, our ELF image (text/data/bss), and the stack
    // the kernel carved out above the heap ceiling.
    if (totalBytes > availBytes)
    {
        PS2_TagsAddMem(MEMTAG_MISC, totalBytes - availBytes);
    }
}

const char * PS2_FormatMemoryUnit(size_t memorySizeInBytes, int abbreviated)
{
    static char s_str[64];
    const char * unit;
    double value;

    if (memorySizeInBytes >= (1024u * 1024u * 1024u))
    {
        unit  = abbreviated ? "GB" : "Gigabytes";
        value = static_cast<double>(memorySizeInBytes) / (1024.0 * 1024.0 * 1024.0);
    }
    else if (memorySizeInBytes >= (1024u * 1024u))
    {
        unit  = abbreviated ? "MB" : "Megabytes";
        value = static_cast<double>(memorySizeInBytes) / (1024.0 * 1024.0);
    }
    else if (memorySizeInBytes >= 1024u)
    {
        unit  = abbreviated ? "KB" : "Kilobytes";
        value = static_cast<double>(memorySizeInBytes) / 1024.0;
    }
    else
    {
        unit  = abbreviated ? "B" : "Bytes";
        value = static_cast<double>(memorySizeInBytes);
    }

    std::snprintf(s_str, sizeof(s_str), "%.2f %s", value, unit);
    return s_str;
}

const PS2MemStats * PS2_GetStatsForMemTag(PS2MemTag tag)
{
    return &s_memTagCounts[MemTagToIndex(tag)];
}

const char * PS2_GetNameForMemTag(PS2MemTag tag)
{
    return s_memTagNames[MemTagToIndex(tag)];
}

const char * PS2_DumpMemTags()
{
    static char s_memTagsDumpBuff[2048];
    char * ptr = s_memTagsDumpBuff;
    size_t memTotal = 0;

    ptr += std::sprintf(ptr, "--------------------------- MEMTAGS ---------------------------\n");
    ptr += std::sprintf(ptr, "Tag Name          Bytes      Allocs  Frees   Small    Large\n");

    for (int i = 0; i < MEMTAG_COUNT; ++i)
    {
        memTotal += s_memTagCounts[i].totalBytes;
        const char * const totalStr = PS2_FormatMemoryUnit(s_memTagCounts[i].totalBytes, true);

        ptr += std::sprintf(ptr, "%-17s %-10s %-7zu %-7zu %-8zu %-8zu\n",
                            s_memTagNames[i], totalStr,
                            s_memTagCounts[i].totalAllocs,
                            s_memTagCounts[i].totalFrees,
                            s_memTagCounts[i].smallestAlloc,
                            s_memTagCounts[i].largestAlloc);
    }

    ptr += std::sprintf(ptr, "\nTOTAL MEM: %s\n", PS2_FormatMemoryUnit(memTotal, true));
    ptr += std::sprintf(ptr, "FREE MEM (sbrk):  %s\n", PS2_FormatMemoryUnit(PS2_GetAvailableMemBytes(), true));
    ptr += std::sprintf(ptr, "--------------------------- MEMTAGS ---------------------------");

    s_memTagsDumpBuff[sizeof(s_memTagsDumpBuff) - 1] = '\0';
    return s_memTagsDumpBuff;
}

} // extern "C"
