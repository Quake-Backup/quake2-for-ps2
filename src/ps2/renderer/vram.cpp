/* ================================================================================================
 * File: vram.cpp
 * Brief: GS VRAM texture heap. See vram.h.
 *
 *  The heap is tracked as a small, address-ordered array of blocks, each either
 *  free or owned by one texture (modelled on gsKit's TexManager block list).
 *  Allocation is first-fit, splitting off the free remainder; when nothing fits,
 *  the least-recently-bound texture is evicted and its block coalesced with free
 *  neighbours until the request can be satisfied. Eviction is "two-level"
 *  (ps2gl's trick): it only marks the victim non-resident - the victim's pixels
 *  stay in EE RAM and re-upload transparently the next time it is bound.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/common.h"
#include "ps2/renderer/vram.h"
#include "ps2/renderer/texture.h"

#include <tamtypes.h>
#include <gs_psm.h>

namespace ps2::vram {
namespace {

constexpr int kVramTotalWords = 1024 * 1024; // 4 MB of GS VRAM, in 32-bit words.

// One entry per contiguous VRAM range. Every block spans at least one GS page
// (TextureFootprintWords is page-granular), so the ~1.27 MB heap left after
// the framebuffers/z-buffer (~161 pages) can never fragment into more blocks
// than this, no matter how many textures the cache holds.
constexpr int kMaxBlocks = 176;

// Debug knob: nonzero clamps the heap to this many words so eviction can be
// exercised without loading more textures than VRAM holds. Keep 0 normally.
constexpr int kDebugHeapLimitWords = 0;

struct Block
{
    Address              addrWords;      // absolute GS VRAM word address
    int                  sizeWords;
    const tex::Texture * owner;          // nullptr = free block
    u32                  lastBoundFrame; // LRU stamp; valid while owned
};

static Block s_blocks[kMaxBlocks];
static int   s_blockCount = 0;
static u32   s_frame      = 0;

// Debug-overlay stats: the heap's total size and this frame's upload count.
static int s_heapTotalWords    = 0;
static int s_uploadsThisFrame  = 0;

void InsertBlockAt(int index)
{
    PS2_AssertMsg(s_blockCount < kMaxBlocks, "Out of VRAM block descriptors!");
    for (int i = s_blockCount; i > index; --i)
    {
        s_blocks[i] = s_blocks[i - 1];
    }
    ++s_blockCount;
}

void RemoveBlockAt(int index)
{
    for (int i = index; i < s_blockCount - 1; ++i)
    {
        s_blocks[i] = s_blocks[i + 1];
    }
    --s_blockCount;
}

// Merges the free block at 'index' with free neighbours, keeping the invariant
// that no two adjacent blocks are both free.
void CoalesceFreeAt(int index)
{
    PS2_Assert(s_blocks[index].owner == nullptr);

    if (index + 1 < s_blockCount && s_blocks[index + 1].owner == nullptr)
    {
        s_blocks[index].sizeWords += s_blocks[index + 1].sizeWords;
        RemoveBlockAt(index + 1);
    }
    if (index > 0 && s_blocks[index - 1].owner == nullptr)
    {
        s_blocks[index - 1].sizeWords += s_blocks[index].sizeWords;
        RemoveBlockAt(index);
    }
}

// Enum names for the debug dump below.
const char * ImageTypeName(tex::ImageType type)
{
    switch (type)
    {
    case tex::ImageType::Null   : return "null";
    case tex::ImageType::Pic    : return "pic";
    case tex::ImageType::Skin   : return "skin";
    case tex::ImageType::Sprite : return "sprite";
    case tex::ImageType::Wall   : return "wall";
    case tex::ImageType::Sky    : return "sky";
    }
    return "???"; // Unreachable; keeps GCC's -Wreturn-type happy.
}

const char * PixelFormatName(tex::PixelFormat format)
{
    switch (format)
    {
    case tex::PixelFormat::RGBA32   : return "rgba32";
    case tex::PixelFormat::RGB16    : return "rgb16";
    case tex::PixelFormat::Palette8 : return "pal8";
    }
    return "???"; // Unreachable; keeps GCC's -Wreturn-type happy.
}

// Prints the whole block list plus the current Stats to stdout, so a failed
// allocation can be diagnosed from the EE emulog: which textures were holding
// VRAM, how much each took and how recently they were bound. Blocks marked
// [pinned] were bound this frame and so cannot be evicted; when every used
// block is pinned the frame's working set simply does not fit. A large free
// total next to a small largest-free-block means fragmentation instead.
void DumpAllBlocks()
{
    Com_Printf("---- GS VRAM texture heap dump (frame %u) ----\n", s_frame);
    Com_Printf("idx  addrWords  sizeWords  sizeKB  lastBound  texture\n");

    int usedBlocks       = 0;
    int pinnedBlocks     = 0;
    int largestFreeWords = 0;

    for (int i = 0; i < s_blockCount; ++i)
    {
        const Block & block = s_blocks[i];
        const int addrWords = static_cast<int>(block.addrWords);
        const int sizeKb    = block.sizeWords * 4 / 1024;

        if (block.owner == nullptr)
        {
            if (block.sizeWords > largestFreeWords)
            {
                largestFreeWords = block.sizeWords;
            }

            Com_Printf("%3d  %9d  %9d  %6d  %9s  <free>\n",
                       i, addrWords, block.sizeWords, sizeKb, "-");
            continue;
        }

        ++usedBlocks;

        const bool pinned = (block.lastBoundFrame == s_frame);
        if (pinned)
        {
            ++pinnedBlocks;
        }

        const tex::Texture & texture = *block.owner;
        Com_Printf("%3d  %9d  %9d  %6d  %9u  %s (%dx%d, %s, %s)%s%s\n",
                   i, addrWords, block.sizeWords, sizeKb, block.lastBoundFrame,
                   texture.name, texture.width, texture.height,
                   PixelFormatName(texture.format), ImageTypeName(texture.type),
                   pinned ? " [pinned]" : "", texture.dirtyPixels ? " [dirty]" : "");
    }

    const Stats stats = GetStats();

    Com_Printf("Blocks   : %d used (%d pinned this frame), %d free, %d of %d descriptors\n",
               usedBlocks,
               pinnedBlocks,
               s_blockCount - usedBlocks,
               s_blockCount,
               kMaxBlocks);

    Com_Printf("Heap     : %d KB total, %d KB used, %d KB free (largest free block %d KB)\n",
               stats.totalWords * 4 / 1024,
               (stats.totalWords - stats.freeWords) * 4 / 1024,
               stats.freeWords * 4 / 1024,
               largestFreeWords * 4 / 1024);

    Com_Printf("Textures : %d resident, %d uploads this frame\n",
               stats.residentTextures,
               stats.uploadsThisFrame);
}

} // namespace

// ------------------------------------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------------------------------------

void Init(int heapBaseWords)
{
    PS2_AssertMsg(s_blockCount == 0, "vram::Init called twice!");
    PS2_Assert(heapBaseWords > 0 && heapBaseWords < kVramTotalWords);

    int heapEndWords = kVramTotalWords;
    if constexpr (kDebugHeapLimitWords != 0)
    {
        heapEndWords = heapBaseWords + kDebugHeapLimitWords;
        PS2_Assert(heapEndWords <= kVramTotalWords);
    }

    s_blocks[0] = { Address(heapBaseWords), heapEndWords - heapBaseWords, nullptr, 0 };
    s_blockCount = 1;
    s_heapTotalWords = s_blocks[0].sizeWords;

    Com_Printf("GS texture heap: %d KB of VRAM.\n", s_blocks[0].sizeWords * 4 / 1024);
}

void BeginFrame()
{
    ++s_frame;
    s_uploadsThisFrame = 0;
}

void EndFrame()
{
    // Nothing for now.
}

int TextureFootprintWords(int width, int height, int psm)
{
    PS2_Assert(width > 0 && height > 0);

    // A texture occupies every GS page its pixel rectangle touches: pages tile
    // the *texture space* in fixed pixel dimensions, and the swizzled layout
    // scatters texels across the whole page grid. libgraph's graph_vram_size
    // counts linear width*height words instead, which undercounts textures with
    // non-page-multiple dimensions and would let the next allocation overlap.
    int pageWidth, pageHeight;
    switch (psm)
    {
    case GS_PSM_32:
        pageWidth  = 64;
        pageHeight = 32;
        break;
    case GS_PSM_16:
    case GS_PSM_16S:
        pageWidth  = 64;
        pageHeight = 64;
        break;
    case GS_PSM_8:
        pageWidth  = 128;
        pageHeight = 64;
        break;
    default:
        PS2_AssertMsg(false, "Unsupported texture PSM!");
        return 0;
    }

    const int pagesX = (width  + pageWidth  - 1) / pageWidth;
    const int pagesY = (height + pageHeight - 1) / pageHeight;
    return pagesX * pagesY * 2048; // one GS page = 8 KB = 2048 words
}

Address Allocate(const tex::Texture & texture, int sizeWords, bool * outEvicted)
{
    PS2_AssertMsg(s_blockCount > 0, "vram::Init not called!");
    PS2_AssertMsg(texture.vramAddr == tex::Texture::kNotResident, "Texture already resident!");
    PS2_Assert(sizeWords > 0 && outEvicted != nullptr);

    *outEvicted = false;

    for (;;)
    {
        // First fit among the free blocks.
        for (int i = 0; i < s_blockCount; ++i)
        {
            if (s_blocks[i].owner != nullptr || s_blocks[i].sizeWords < sizeWords)
            {
                continue;
            }

            if (s_blocks[i].sizeWords > sizeWords)
            {
                // Split off the free remainder.
                InsertBlockAt(i + 1);
                s_blocks[i + 1] = {
                    Address(static_cast<int>(s_blocks[i].addrWords) + sizeWords),
                    s_blocks[i].sizeWords - sizeWords,
                    nullptr,
                    0
                };
                s_blocks[i].sizeWords = sizeWords;
            }

            s_blocks[i].owner = &texture;
            s_blocks[i].lastBoundFrame = s_frame;
            return s_blocks[i].addrWords;
        }

        // Nothing fits: evict the least-recently-bound texture and retry.
        // Textures bound this frame are off-limits - their draws may still be
        // queued in the frame packet or in flight on the GS.
        int victim = -1;
        for (int i = 0; i < s_blockCount; ++i)
        {
            if (s_blocks[i].owner == nullptr || s_blocks[i].lastBoundFrame == s_frame)
            {
                continue;
            }
            if (victim < 0 || s_blocks[i].lastBoundFrame < s_blocks[victim].lastBoundFrame)
            {
                victim = i;
            }
        }

        if (victim < 0)
        {
            DumpAllBlocks();
            Sys_Error("GS texture heap too small for this frame's working set! Failed request of %d words.", sizeWords);
        }

        Com_DPrintf("VRAM: evicting '%s' (%d KB)\n",
                    s_blocks[victim].owner->name, s_blocks[victim].sizeWords * 4 / 1024);

        s_blocks[victim].owner->vramAddr = tex::Texture::kNotResident;
        s_blocks[victim].owner = nullptr;
        *outEvicted = true;
        CoalesceFreeAt(victim);
    }
}

void Touch(const tex::Texture & texture)
{
    PS2_AssertMsg(texture.vramAddr != tex::Texture::kNotResident, "Touch on a non-resident texture!");

    for (int i = 0; i < s_blockCount; ++i)
    {
        if (s_blocks[i].owner == &texture)
        {
            s_blocks[i].lastBoundFrame = s_frame;
            return;
        }
    }

    PS2_AssertMsg(false, "Resident texture has no VRAM block!");
}

bool BoundThisFrame(const tex::Texture & texture)
{
    PS2_AssertMsg(texture.vramAddr != tex::Texture::kNotResident, "BoundThisFrame on a non-resident texture!");

    for (int i = 0; i < s_blockCount; ++i)
    {
        if (s_blocks[i].owner == &texture)
        {
            return s_blocks[i].lastBoundFrame == s_frame;
        }
    }

    PS2_AssertMsg(false, "Resident texture has no VRAM block!");
    return false;
}

void Free(const tex::Texture & texture)
{
    if (texture.vramAddr == tex::Texture::kNotResident)
    {
        return;
    }

    for (int i = 0; i < s_blockCount; ++i)
    {
        if (s_blocks[i].owner == &texture)
        {
            texture.vramAddr  = tex::Texture::kNotResident;
            s_blocks[i].owner = nullptr;
            CoalesceFreeAt(i);
            return;
        }
    }

    PS2_AssertMsg(false, "Resident texture has no VRAM block!");
}

void NoteTextureUpload()
{
    ++s_uploadsThisFrame;
}

Stats GetStats()
{
    Stats stats = {};
    stats.totalWords       = s_heapTotalWords;
    stats.uploadsThisFrame = s_uploadsThisFrame;

    for (int i = 0; i < s_blockCount; ++i)
    {
        if (s_blocks[i].owner == nullptr)
        {
            stats.freeWords += s_blocks[i].sizeWords;
        }
        else
        {
            ++stats.residentTextures;
        }
    }

    return stats;
}

} // namespace ps2::vram
