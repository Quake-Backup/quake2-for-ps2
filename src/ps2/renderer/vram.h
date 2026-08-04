#pragma once
/* ================================================================================================
 * File: vram.h
 * Brief: GS VRAM texture heap: tracks which textures are resident in the VRAM left
 *        over after the framebuffers and z-buffer, handing out space on demand and
 *        evicting the least-recently-bound textures when full. Pure bookkeeping -
 *        the DMA uploads and GS synchronisation stay with gs::EnsureTextureResident.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

namespace ps2::tex { struct Texture; }

namespace ps2::vram {

// 32-bits type to represent VRAM addresses.
enum struct Address : int
{
    Invalid = -1,
};

// Takes ownership of GS VRAM from 'heapBaseWords' (a word address) up to the 4 MB
// end. Call once, from gs::Init(), after the framebuffer/z-buffer allocations.
void Init(int heapBaseWords);

// Advances the LRU clock. Call once per frame, from gs::BeginFrame()/EndFrame().
// BeginFrame() also resets the per-frame texture-upload counter (see GetStats).
void BeginFrame();
void EndFrame();

// VRAM words the texture occupies: the whole GS page grid it covers. libgraph's
// graph_vram_size undercounts here - see the .cpp for why.
int TextureFootprintWords(int width, int height, int psm);

// Allocates 'sizeWords' for 'texture', evicting least-recently-bound textures as
// needed (never ones bound this frame; asserts if the frame's working set cannot
// fit). Evicted textures get vramAddr = kNotResident and self-heal on their next
// bind. Returns the block's word address; sets *outEvicted when anything was
// evicted - the caller must sync the GS before writing over reused VRAM.
Address Allocate(const tex::Texture & texture, int sizeWords, bool * outEvicted);

// Marks the (resident) texture as bound this frame, protecting it from eviction
// until the next frame.
void Touch(const tex::Texture & texture);

// True when the (resident) texture was already bound this frame - its draws may
// still be queued in the unsent frame packet, so overwriting its VRAM (dynamic
// texture re-upload) must sync the GS first.
bool BoundThisFrame(const tex::Texture & texture);

// Returns the texture's block to the heap and marks it non-resident (no-op when
// not resident). The freed range may be handed out without an eviction, so the
// caller must treat it like evicted VRAM: sync the GS before writing over it.
void Free(const tex::Texture & texture);

// Compacts the heap the cheap way: evicts every resident texture and remakes it
// as one contiguous free range. Nothing is lost - as with any eviction the pixels
// stay in EE RAM and re-upload on the next bind, which repacks the live set from
// the heap base with no holes, and only for the textures actually bound again.
// Returns true when anything was evicted; the caller must then sync the GS before
// the recycled VRAM is written, exactly like Allocate's *outEvicted.
//
// Meant for level changes: freeing the previous level's textures leaves holes
// between the surviving textures, and the new level's first frame binds its whole
// working set at once - every block pinned, nothing evictable - so an allocation
// can fail there with plenty of free VRAM left, just not contiguous.
bool Defragment();

// Records one texture DMA upload for the per-frame counter. Called by
// gs::EnsureTextureResident each time it transfers a texture's pixels into VRAM
// (a first upload or a dirty re-upload); reset each frame by BeginFrame().
void NoteTextureUpload();

// Live snapshot of the texture heap, for the ref.cpp debug overlay.
struct Stats
{
    int freeWords;        // uncommitted VRAM: total size of the free blocks
    int totalWords;       // heap size handed to Init()
    int residentTextures; // textures currently holding a block
    int uploadsThisFrame; // texture DMA uploads since the last BeginFrame()
};

// Computes the current stats (cheap; walks the block list).
Stats GetStats();

} // namespace ps2::vram
