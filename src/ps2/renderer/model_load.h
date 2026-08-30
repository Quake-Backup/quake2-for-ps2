#pragma once
/* ================================================================================================
 * File: model_load.h
 * Brief: Loaders for the Quake 2 on-disk model formats (world map, sprite, md2).
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

namespace ps2::mod {

struct ModelInstance;

// LoadBrushModel takes the file name rather than a loaded buffer: a world .bsp
// is 2-3 MB and the hunk built from it another 4-7 MB, so it streams the lumps
// through a small scratch buffer instead of holding both at once. See BspFileReader.
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

bool LoadBrushModel(ModelInstance & outModel, const char * fileName);
bool LoadSpriteModel(ModelInstance & outModel, const void * modelData, int dataLenBytes);
bool LoadAliasMD2Model(ModelInstance & outModel, const void * modelData, int dataLenBytes);

} // namespace ps2::mod
