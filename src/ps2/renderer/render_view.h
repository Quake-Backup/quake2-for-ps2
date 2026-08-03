#pragma once
/* ================================================================================================
 * File: render_view.h
 * Brief: View/3D frame rendering helpers.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/common.h"
#include "ps2/math/vec_mat.h"

namespace ps2::view
{

void InitViewRendering();

// Performance counters for one RenderFrame, tracking what the 3D view walked,
// culled, clipped and submitted. Feeds the ps2_show_drawstats debug overlay.
struct DrawStats
{
    int nodesWalked;    // BSP nodes + leafs visited by the world walk.
    int surfaces;       // Opaque world surfaces drawn.
    int surfacesAlpha;  // Translucent surfaces deferred to the final alpha pass.
    int trisDrawn;      // Triangles submitted to VU1 (after EE clipping).
    int trisClipped;    // Triangles re-cut against the VU clip volume.
    int trisCulled;     // Triangles dropped whole, entirely outside the view volume.
    int trisBackFacing; // Triangles dropped by the world back-face test, before clipping.
    int boxesCulled;    // Whole meshes culled via bounding box checks.
    int drawBatches;    // vu1::DrawTriangles calls (one or more per texture).
    int entities;       // Entity models drawn (after frustum culling).
    int particles;      // Particle billboards drawn.
    int dlights;        // Dynamic light flares drawn.
};

// Stats of the most recent RenderFrame; all zeros before the first 3D frame.
DrawStats & GetDrawStats();

// Entity angles + origin as a world transform, in the row-vector convention
// (rotations apply first, then the translation). 'flipPitchAngle' picks the sign
// of the pitch rotation: ref_gl's R_RotateForEntity applies -pitch, which is
// what brush models get, but gl_mesh.c negates the angle around that call for
// meshes ("e->angles[PITCH] = -e->angles[PITCH];"), so alias models end
// up with +pitch. Only shows on entities that actually pitch - most brush
// models never leave yaw.
math::Mat4 MakeEntityMatrix(const entity_t & entity, bool flipPitchAngle);

// True when some frustum side plane has every one of the points on its
// outside - the conservative cull test for rotated (non-axis-aligned)
// bounding boxes, fed their world-space corners (ref_gl's R_CullAliasModel).
bool FrustumCullsPoints(const vec3_t * points, int numPoints);

// Samples the world's static lighting at 'point' - a lightmap trace straight
// down (ref_gl's R_LightPoint) with lightstyle scaling applied and the frame's
// dynamic lights added on top. Also returns where the trace hit ('outLightSpot',
// the ground anchor for projected shadows). All-ones when the world has no
// light data; black when the trace leaves the world. Used to shade entity
// models, which have no lightmaps of their own.
void CalcPointLightColor(const refdef_t & viewDef, const vec3_t point,
                         vec3_t outColor, vec3_t outLightSpot);

// Resets the cached view clusters. Call when a new map loads
// (PS2_BeginRegistration) so stale PVS state cannot leak across maps.
void BeginRegistration();

// Draws the 3D scene described by 'viewDef': the world's visible BSP geometry
// (PVS + frustum culled), submitted per texture through vu1::DrawTriangles.
// Call between gs::Begin/EndFrame.
void RenderFrame(const refdef_t & viewDef);

} // namespace ps2::view
