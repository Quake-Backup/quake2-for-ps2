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

// Performance counters for one RenderFrame, tracking what the 3D view walked,
// culled, clipped and submitted. Feeds the ps2_show_drawstats debug overlay.
struct DrawStats
{
    int nodesWalked;   // BSP nodes + leafs visited by the world walk.
    int surfaces;      // Opaque world surfaces drawn.
    int surfacesAlpha; // Translucent surfaces deferred.
    int trisDrawn;     // Triangles submitted to VU1 (after EE clipping).
    int trisClipped;   // Triangles re-cut against the VU clip volume.
    int trisCulled;    // Triangles dropped whole, entirely outside the view volume.
    int boxesCulled;   // Whole meshes culled via bounding box checks.
    int drawBatches;   // vu1::DrawTriangles calls (one or more per texture).
};

// Stats of the most recent RenderFrame; all zeros before the first 3D frame.
const DrawStats & GetDrawStats();

// Resets the cached view clusters. Call when a new map loads
// (PS2_BeginRegistration) so stale PVS state cannot leak across maps.
void BeginRegistration();

// Draws the 3D scene described by 'viewDef': the world's visible BSP geometry
// (PVS + frustum culled), submitted per texture through vu1::DrawTriangles.
// Call between gs::Begin/EndFrame.
void RenderFrame(const refdef_t & viewDef);

} // namespace ps2::view
