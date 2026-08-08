#pragma once
/* ================================================================================================
 * File: lightmap.h
 * Brief: World lightmaps: the atlases the BSP's baked light samples are packed into,
 *        and the per-frame rebuilds that fold in animated light styles and dynamic
 *        lights. Owned by an internal manager (see lightmap.cpp); this is its API.
 *
 *  Build time: the BSP face loader brackets its loop with BeginBuildingLightmaps /
 *  EndBuildingLightmaps and calls CreateSurfaceLightmap per lit face, which packs the
 *  face's luxels into an atlas and records where they landed on the surface
 *  (light_s/light_t/lightmapTextureNum). The polygon builder then bakes those into
 *  the vertices' second UV set, so nothing has to be recomputed at draw time.
 *
 *  Frame time: the view renderer calls ChainSurface for every visible lit surface,
 *  which rebuilds the ones whose lighting moved and threads each onto its atlas's
 *  draw chain. The lightmap pass then walks one chain per atlas (AtlasTexture /
 *  AtlasChain) and clears them with ClearChains.
 *
 *  The atlases are Alpha8: the GS blend unit can only scale the framebuffer by a
 *  scalar alpha, never by a second colour, so what the lightmap pass multiplies in
 *  is luxel *intensity*. The colour of each luxel is kept in EE RAM alongside it
 *  (see lightmap.cpp) for the paths that will want it later.
 *
 *  TODO (coloured lighting): lighting is monochrome for exactly that reason - the
 *  hardware cannot express diffuse x coloured-lightmap per pixel in one blend. The
 *  colour is loaded, built, animated and stored; only the last step of getting it
 *  onto the screen is missing, so this is where an accessor for it belongs. Every
 *  site that has to change is marked with this same tag - here, in lightmap.cpp
 *  (the mirror buffer and StoreLightmap) and in render_view.cpp (ClipVertex,
 *  GatherPolyTriangles, EmitScratchVertex and SurfaceDrawState).
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/common.h"

namespace ps2::tex { struct Texture; }
namespace ps2::mod { struct ModelSurface; }

namespace ps2::lm {

// Atlas dimensions, in luxels. The model loader normalises the lightmap UVs
// against these, so the two must agree - it takes them from here.
constexpr int kLightmapTextureWidth  = 256;
constexpr int kLightmapTextureHeight = 256;

// World units one luxel covers. Baked into the BSP format rather than chosen:
// surface extents are snapped to this grid, which is what makes a surface's
// luxel count (extents >> 4) + 1.
constexpr int kLuxelSizeUnits = 16;

// Atlases a single map may use. Measured over all 39 retail maps: they need 2
// (fact3) to 6 (bunk1, power2, space, ...), packing 58-91% full, so this is
// roughly double the worst case. Atlases are allocated on demand, so the extra
// headroom costs nothing until a map actually reaches for it; running out is a
// Sys_Error telling you to raise it.
//
// Six atlases is ~384 KB of GS VRAM out of a ~1.27 MB heap, which is the real
// budget to watch - see the note on vram::Allocate failures in lightmap.cpp.
constexpr int kMaxLightmapTextures = 12;

// Registers the lightmap cvars. Call once, after tex::Init().
void Init();

// ------------------------------------------------------------------------------------------------
// Level build - bracket the BSP face loop (ref_gl's GL_Begin/EndBuildingLightmaps)
// ------------------------------------------------------------------------------------------------

// Releases the previous map's atlases (EE RAM and GS VRAM) and resets the packer.
void BeginBuildingLightmaps();

// Packs the surface's luxels into an atlas and bakes its static lighting there,
// filling in surf.light_s / light_t / lightmapTextureNum. Must run before the
// surface's polygon is built - the lightmap UVs bake light_s/light_t in - and
// only for lit faces: the caller skips sky, turbulent and translucent ones,
// which keep mod::kNotLightmapped.
void CreateSurfaceLightmap(mod::ModelSurface & surf);

// Closes the atlas being filled. Nothing may be packed until the next
// BeginBuildingLightmaps.
void EndBuildingLightmaps();

// ------------------------------------------------------------------------------------------------
// Frame time
// ------------------------------------------------------------------------------------------------

// Resets the per-frame debug counters. Call once at the top of the frame.
void BeginFrame();

// Rebuilds the surface's luxels if its lighting changed since they were baked -
// an animated light style, or a dynamic light touching it this frame - and
// threads it onto its atlas's draw chain. Call once per visible lit surface,
// before the lightmap pass draws. Surfaces with mod::kNotLightmapped are not
// valid arguments; the caller filters them out.
void ChainSurface(mod::ModelSurface & surf, const refdef_t & viewDef, int frameCount);

// Atlases in use by the current map, and the texture to bind for one. Indices
// are the surfaces' lightmapTextureNum.
int NumAtlases();
const tex::Texture & AtlasTexture(int index);

// Head of the atlas's draw chain, as built by ChainSurface this frame; null when
// nothing visible uses it. Walk it through ModelSurface::lightmapChain.
const mod::ModelSurface * AtlasChain(int index);

// Empties every atlas draw chain. Call right after drawing them, so the next
// caller (a brush model entity, or the next frame's world) starts clean.
void ClearChains();

// Per-frame counters for the ps2_show_drawstats overlay. Cleared by BeginFrame.
struct Stats
{
    int atlases;        // Atlases the current map packed into (not per frame).
    int styleUpdates;   // Surfaces rebuilt because an animated light style moved.
    int dynamicUpdates; // Surfaces rebuilt with a dynamic light folded in.
    int restoreUpdates; // Surfaces rebuilt back to their static lighting.
};

Stats GetStats();

} // namespace ps2::lm
