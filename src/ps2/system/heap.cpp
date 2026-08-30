/* ================================================================================================
 * File: heap.cpp
 * Brief: C++ side of the program-wide dlmalloc heap. Provides operator new/delete
 *        (routed to the dlmalloc-backed global malloc from dlmalloc.c) and the
 *        tag-accounting layer, which common.c's Z_Malloc and the renderer allocate through.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/common.h" // Sys_Error, etc
#include "ps2/system/heap.h"
#include "ps2/debug/stack_trace.h" // PrintStackTrace

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

    // malloc.h declares this one unprefixed even under USE_DL_PREFIX (both arms of
    // its #ifndef say "mallinfo"), so the name it actually exports goes undeclared.
    // malloc.c does define dlmallinfo - see public_mALLINFo - so declare it here.
    struct mallinfo dlmallinfo(void);
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
    "ELF_Sys",
    "OpNew",
    "Quake",
    "Renderer",
    "TexImage",
    "Alias",
    "Sprite",
    "World",
    "Lightmap",
    "Audio",
};

static inline size_t MemTagToIndex(PS2MemTag tag)
{
    const int t = static_cast<int>(tag);
    return (t >= 0 && t < MEMTAG_COUNT) ? static_cast<size_t>(t) : static_cast<size_t>(MEMTAG_ELF_SYS);
}

// Running sum of every tag's totalBytes, and the largest it has ever been. Kept
// incrementally rather than summed on demand so the peak is sampled at every
// allocation - the map-change transient we care about lasts milliseconds and would
// be missed by anything that only looks when asked.
static size_t s_liveTotalBytes = 0;
static size_t s_peakTotalBytes = 0;

static inline void AccountAlloc(PS2MemTag tag, size_t bytes)
{
    PS2MemStats * c = &s_memTagCounts[MemTagToIndex(tag)];
    c->totalBytes  += bytes;
    c->totalAllocs += 1u;

    if (c->smallestAlloc == 0u || bytes < c->smallestAlloc) { c->smallestAlloc = bytes; }
    if (bytes > c->largestAlloc) { c->largestAlloc = bytes; }
    if (c->totalBytes > c->peakBytes) { c->peakBytes = c->totalBytes; }

    s_liveTotalBytes += bytes;
    if (s_liveTotalBytes > s_peakTotalBytes) { s_peakTotalBytes = s_liveTotalBytes; }
}

// ------------------------------------------------------------------------------------------------
// Allocator dump
// ------------------------------------------------------------------------------------------------

// The largest block dlmalloc would actually hand out right now, found by probing
// and immediately giving back. mallinfo reports free bytes and free chunk count
// but not the largest run, and the largest run is the number that decides whether
// a big contiguous request can be served.
//
// Only ever called on the way to Sys_Error. It allocates, so it perturbs the heap
// it is measuring - acceptable exactly once, at the point where the program is
// already terminating.
__attribute__((cold, noinline))
static size_t LargestAllocatableBlock(const size_t upperBound)
{
    if (upperBound < 16u) { return 0u; }

    // Binary search the largest size that succeeds. ~24 iterations for a 32 MB
    // span, each one a malloc/free pair.
    size_t lo = 0u;         // known to succeed (trivially)
    size_t hi = upperBound; // known to fail (the caller just proved it)

    while (hi - lo > 4096u)
    {
        const size_t mid = lo + ((hi - lo) / 2u);
        void * const p = dlmalloc(mid);
        if (p != nullptr)
        {
            dlfree(p);
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    return lo;
}

__attribute__((cold, noinline))
static void PrintDlmallocStats(const size_t failedRequest)
{
    const struct mallinfo mi = dlmallinfo();

    // mallinfo's fields are ints over unsigned-long bookkeeping, so they can read
    // negative once the arena passes 2 GB. Never on a 32 MB console, but clamp
    // rather than print something absurd if it ever does.
    const auto Clamp = [](int v) -> size_t { return (v < 0) ? 0u : static_cast<size_t>(v); };

    const size_t arena    = Clamp(mi.arena);
    const size_t inUse    = Clamp(mi.uordblks);
    const size_t freeTot  = Clamp(mi.fordblks);
    const size_t freeChks = Clamp(mi.ordblks);
    const size_t keepCost = Clamp(mi.keepcost);
    const size_t largest  = LargestAllocatableBlock(failedRequest);

    char a[PS2_MEMUNIT_STR_SIZE], b[PS2_MEMUNIT_STR_SIZE], c[PS2_MEMUNIT_STR_SIZE];

    std::printf("-------------------------- DLMALLOC ---------------------------\n");
    std::printf("Arena (sbrk'd)   : %s\n", PS2_FormatMemoryUnit(arena, true, a, sizeof(a)));
    std::printf("In use           : %s\n", PS2_FormatMemoryUnit(inUse, true, a, sizeof(a)));
    std::printf("Free total       : %s  in %zu chunks (avg %s)\n",
                PS2_FormatMemoryUnit(freeTot, true, a, sizeof(a)), freeChks,
                PS2_FormatMemoryUnit((freeChks != 0u) ? (freeTot / freeChks) : 0u, true, b, sizeof(b)));
    std::printf("Top releasable   : %s\n", PS2_FormatMemoryUnit(keepCost, true, a, sizeof(a)));
    std::printf("Largest free blk : %s   (the failed request wanted %s)\n",
                PS2_FormatMemoryUnit(largest, true, a, sizeof(a)),
                PS2_FormatMemoryUnit(failedRequest, true, b, sizeof(b)));

    // The verdict, spelled out, so the log answers the question without arithmetic.
    if (freeTot >= failedRequest)
    {
        std::printf("VERDICT: FRAGMENTATION. %s free in total, but the largest single run is\n"
                    "         only %s. The bytes exist; they are not adjacent.\n",
                    PS2_FormatMemoryUnit(freeTot, true, a, sizeof(a)),
                    PS2_FormatMemoryUnit(largest, true, c, sizeof(c)));
    }
    else
    {
        std::printf("VERDICT: EXHAUSTION. Only %s free in total, less than the request.\n",
                    PS2_FormatMemoryUnit(freeTot, true, a, sizeof(a)));
    }
    std::printf("-------------------------- DLMALLOC ---------------------------\n");
    std::fflush(stdout);
}

__attribute__((cold, noinline))
static void OutOfMemory(const size_t requestSize, const PS2MemTag tag, const char * const funcName)
{
    // The call stack goes to stdout, not to Sys_Error: the panic screen it
    // paints has 24 lines to spend on the message and the memtag table, and
    // stdout is where the PCSX2/ps2client log goes.
    std::printf("%s: failed to allocate %zu bytes (tag: %s)\n",
                funcName, requestSize, s_memTagNames[MemTagToIndex(tag)]);

    PrintDlmallocStats(requestSize);
    ps2::debug::PrintStackTrace();

    char dump[PS2_MEMTAGS_DUMP_SIZE];
    Sys_Error("%s: failed to allocate %zu bytes (%s)\n%s",
              funcName, requestSize, s_memTagNames[MemTagToIndex(tag)],
              PS2_DumpMemTags(dump, sizeof(dump)));
}

// ------------------------------------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------------------------------------

extern "C" {

void * PS2_MemAlloc(size_t sizeBytes, PS2MemTag tag)
{
    const size_t n = (sizeBytes != 0u ? sizeBytes : 1u);
    void * p = dlmalloc(n);

    if (p == nullptr) [[unlikely]]
    {
        OutOfMemory(sizeBytes, tag, "PS2_MemAlloc");
    }

    AccountAlloc(tag, n);
    return p;
}

void * PS2_MemAllocAligned(size_t alignment, size_t sizeBytes, PS2MemTag tag)
{
    const size_t n = (sizeBytes != 0u ? sizeBytes : 1u);
    void * p = dlmemalign(alignment, n);

    if (p == nullptr) [[unlikely]]
    {
        OutOfMemory(sizeBytes, tag, "PS2_MemAllocAligned");
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
        const size_t taken = (c->totalBytes >= sizeBytes) ? sizeBytes : c->totalBytes;
        c->totalBytes     -= taken;
        s_liveTotalBytes  -= (s_liveTotalBytes >= taken) ? taken : s_liveTotalBytes;
    }
    dlfree(ptr);
}

void PS2_TagsAddMem(PS2MemTag tag, size_t sizeBytes)
{
    AccountAlloc(tag, sizeBytes);
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
        const size_t totalUsedBytes = totalBytes - availBytes;

        Sys_ConsoleOutput(va("RAM available: %.2f MB, used by ELF + system: %.2f MB\n",
                             static_cast<double>(availBytes) / 1024.0 / 1024.0,
                             static_cast<double>(totalUsedBytes) / 1024.0 / 1024.0));

        PS2_TagsAddMem(MEMTAG_ELF_SYS, totalUsedBytes);
    }
}

const char * PS2_FormatMemoryUnit(size_t memorySizeInBytes, int abbreviated,
                                  char * outBuffer, size_t outBufferSize)
{
    const char * unit;
    double value;

    if (outBuffer == nullptr || outBufferSize == 0u)
    {
        return "";
    }

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

    std::snprintf(outBuffer, outBufferSize, "%.2f %s", value, unit);
    return outBuffer;
}

const PS2MemStats * PS2_GetStatsForMemTag(PS2MemTag tag)
{
    return &s_memTagCounts[MemTagToIndex(tag)];
}

const char * PS2_GetNameForMemTag(PS2MemTag tag)
{
    return s_memTagNames[MemTagToIndex(tag)];
}

size_t PS2_GetPeakMemBytes()
{
    return s_peakTotalBytes;
}

const char * PS2_DumpMemTags(char * outBuffer, size_t outBufferSize)
{
    char unitStr[PS2_MEMUNIT_STR_SIZE];
    char peakStr[PS2_MEMUNIT_STR_SIZE];
    char * const end = outBuffer + outBufferSize;
    size_t memTotal = 0;

    if (outBuffer == nullptr || outBufferSize == 0u)
    {
        return "";
    }

    // snprintf clamps and reports the *untruncated* length, so advance by
    // whichever is smaller: a dump that outgrows the caller's buffer stops
    // filling it instead of walking off the end.
    char * ptr = outBuffer;
    const auto append = [&ptr, end](const char * fmt, auto... args)
    {
        if (ptr >= end) { return; }
        const size_t left = static_cast<size_t>(end - ptr);
        const int n = std::snprintf(ptr, left, fmt, args...);
        ptr += (n < 0) ? 0 : ((static_cast<size_t>(n) < left) ? n : static_cast<int>(left - 1u));
    };

    append("%s", "--------------------------- MEMTAGS ---------------------------\n");
    append("%s", "Tag      Total     Peak      Allocs  Frees   Small    Large\n");

    for (int i = 0; i < MEMTAG_COUNT; ++i)
    {
        memTotal += s_memTagCounts[i].totalBytes;

        append("%-8s %-9s %-9s %-7zu %-7zu %-8zu %-8zu\n",
               s_memTagNames[i],
               PS2_FormatMemoryUnit(s_memTagCounts[i].totalBytes, true, unitStr, sizeof(unitStr)),
               PS2_FormatMemoryUnit(s_memTagCounts[i].peakBytes,  true, peakStr, sizeof(peakStr)),
               s_memTagCounts[i].totalAllocs,
               s_memTagCounts[i].totalFrees,
               s_memTagCounts[i].smallestAlloc,
               s_memTagCounts[i].largestAlloc);
    }

    // PEAK MEM is the high-water of the sum, not the sum of the per-tag peaks:
    // those happen at different moments, so adding them up would over-report.
    append("\nTOTAL MEM: %s", PS2_FormatMemoryUnit(memTotal, true, unitStr, sizeof(unitStr)));
    append("   PEAK MEM: %s\n", PS2_FormatMemoryUnit(s_peakTotalBytes, true, peakStr, sizeof(peakStr)));
    append("FREE MEM (sbrk): %s\n", PS2_FormatMemoryUnit(PS2_GetAvailableMemBytes(), true, unitStr, sizeof(unitStr)));
    append("%s", "--------------------------- MEMTAGS ---------------------------");

    outBuffer[outBufferSize - 1u] = '\0';
    return outBuffer;
}

} // extern "C"
