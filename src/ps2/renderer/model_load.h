#pragma once
/* ================================================================================================
 * File: model_load.h
 * Brief: Loaders for the Quake 2 on-disk model formats (world map, sprite, md2).
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

namespace ps2::mod {

struct ModelInstance;

// Takes the file name rather than a loaded buffer: a world .bsp is 2-3 MB and the
// hunk built from it another 4-7 MB, so this streams the lumps through a small
// scratch buffer instead of holding both at once. See BspFileReader in the .cpp.
bool LoadBrushModel(ModelInstance & outModel, const char * fileName);
bool LoadSpriteModel(ModelInstance & outModel, const void * modelData, int dataLenBytes);
bool LoadAliasMD2Model(ModelInstance & outModel, const void * modelData, int dataLenBytes);

} // namespace ps2::mod
