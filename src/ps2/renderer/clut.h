#pragma once
/* ================================================================================================
 * File: clut.h
 * Brief: The 256-entry Color Lookup Tables the GS's indexed pixel formats sample
 *        through. Both of ours are built once at startup and live at fixed VRAM
 *        addresses outside the texture heap - see gs.cpp, which owns the instances
 *        and their upload; this header is just their layout and the entry-order
 *        arithmetic the GS imposes.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/renderer/vram.h"
#include <tamtypes.h>

namespace ps2::tex {

// A CLUT: 256 RGBA entries plus where they live in GS VRAM.
//
// The entries are held in the arrangement the GS *reads* them in, not the
// arrangement they are written in - see Csm1Index - so the buffer is ready to
// upload verbatim and an index into it is not the palette index. Build it
// through the helpers below rather than assigning entries directly.
struct Clut final
{
    static constexpr int kNumEntries = 256;

    // A CLUT uploads as a 16x16 PSMCT32 image (256 words = 4 GS blocks), with
    // the transfer's destination buffer at the minimum TBW granularity.
    static constexpr int kImageWidth    = 16;
    static constexpr int kImageHeight   = 16;
    static constexpr int kTransferWidth = 64;

    alignas(16) u32 entries[kNumEntries];
    vram::Address vramAddr = vram::Address::Invalid;

    // Where the GS reads palette entry 'index' from in CSM1 storage mode: within
    // each 32-entry group the two middle 8-entry blocks swap, i.e. index bits 3
    // and 4 exchange (see ps2stuff GS::ReorderClut).
    static constexpr int Csm1Index(int index)
    {
        return (index & ~0x18) | ((index & 0x08) << 1) | ((index & 0x10) >> 1);
    }

    // Fills from 256 linear RGBA entries - a palette in the order everything
    // outside the GS thinks of it in.
    void BuildFromPalette(const u32 * palette)
    {
        for (int i = 0; i < kNumEntries; ++i)
        {
            entries[Csm1Index(i)] = palette[i];
        }
    }

    // Fills from the same palette with ref_gl's 'intensity' multiplied into the
    // colour first. This is the compensation that keeps a texture from going
    // dim the moment something multiplies it back down - a lightmap over a
    // wall, an entity's shade colour over a skin. Quake's baked lightmaps are
    // dark (a median luxel across the retail maps is 46 of 255, and none of
    // them exceed 196), so without it a lit surface draws at a fraction of the
    // brightness it was authored for.
    //
    // Alpha is left alone; only the three colour channels scale, and each
    // clamps at full rather than wrapping - exactly ref_gl's intensitytable.
    void BuildFromPaletteScaled(const u32 * palette, const float scale)
    {
        // 256 entries share 256 possible channel values, so the scale only has
        // to be worked out once per value rather than once per channel.
        u8 ramp[kNumEntries];
        for (int i = 0; i < kNumEntries; ++i)
        {
            const float scaled = static_cast<float>(i) * scale;
            ramp[i] = static_cast<u8>((scaled >= 255.0f) ? 255.0f
                                    : (scaled <= 0.0f)   ? 0.0f
                                                         : scaled);
        }

        for (int i = 0; i < kNumEntries; ++i)
        {
            const u32 entry = palette[i];
            entries[Csm1Index(i)] =  static_cast<u32>(ramp[ entry        & 0xFFu])
                                  | (static_cast<u32>(ramp[(entry >>  8) & 0xFFu]) <<  8)
                                  | (static_cast<u32>(ramp[(entry >> 16) & 0xFFu]) << 16)
                                  | (entry & 0xFF000000u);
        }
    }

    // Fills with the alpha ramp sampled by PixelFormat::Alpha8: the index *is*
    // the alpha, and the colour is pinned at the GS modulate identity (128) so
    // the texture leaves the primitive's own colour untouched. Alpha 0x80 is 1.0
    // on the GS, so the ramp tops out at 128 rather than 255. Index 0 maps to
    // alpha 0 on purpose - the batch alpha test drops those texels, which is what
    // cuts the particle images out; callers that must not be cut out (the
    // lightmap atlases) clamp their stored index to 1.
    void BuildAlphaRamp()
    {
        for (int i = 0; i < kNumEntries; ++i)
        {
            const u32 alpha = static_cast<u32>((i + 1) >> 1);
            entries[Csm1Index(i)] = 128u | (128u << 8) | (128u << 16) | (alpha << 24);
        }
    }
};

} // namespace ps2::tex
