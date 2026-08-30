#pragma once
/* ================================================================================================
 * File: model_load.h
 * Brief: Loaders for the Quake 2 on-disk model formats (world map, sprite, md2).
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include <cstdio> // FILE

namespace ps2::mod {

struct ModelInstance;

// Reserves the block the world hunk and the streamed loader's lump scratch are
// carved out of, for the life of the program. Call once at renderer init, before
// any map loads. Neither of those two is ever handed back to the general heap:
// they are the largest and most frequently recycled allocations in the game, and
// leaving them to a non-moving allocator fragments it until no contiguous run big
// enough survives.
void ReserveWorldArena();

// True if 'ptr' is the base of the reserved arena, i.e. memory that must never be
// passed to PS2_MemFree. ModelCache::Unload checks this before releasing a hunk.
bool IsWorldArenaBlock(const void * ptr);

// All three take an open file positioned at the model's first byte, and none of
// them closes it - the caller opened it to read the format tag and owns it.
//
// None of these formats needs a decode pass on the EE: a sprite and an MD2 are
// stored in the hunk exactly as they sit on disk, and a .bsp is read lump by lump
// into a hunk laid out up front. So every one of them reads straight into its
// final destination. Nothing here ever holds a whole model file and a copy of it
// at the same time, which for the biggest MD2 in pak0 would be 2 ~MB.
bool LoadBrushModel(ModelInstance & outModel, FILE * file, const char * fileName);
bool LoadSpriteModel(ModelInstance & outModel, FILE * file, int fileLen);
bool LoadAliasMD2Model(ModelInstance & outModel, FILE * file, int fileLen);

} // namespace ps2::mod
