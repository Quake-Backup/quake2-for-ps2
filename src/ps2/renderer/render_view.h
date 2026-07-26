#pragma once
/* ================================================================================================
 * File: render_view.h
 * Brief: View/3D frame rendering helpers.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/common.h"

namespace ps2::view
{

// Resets the cached view clusters. Call when a new map loads
// (PS2_BeginRegistration) so stale PVS state cannot leak across maps.
void BeginRegistration();

// Draws the 3D scene described by 'viewDef': the world's visible BSP geometry
// (PVS + frustum culled), submitted per texture through vu1::DrawTriangles.
// Call between gs::Begin/EndFrame.
void RenderFrame(const refdef_t & viewDef);

} // namespace ps2::view
