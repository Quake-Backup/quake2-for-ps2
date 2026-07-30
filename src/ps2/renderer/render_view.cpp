/* ================================================================================================
 * File: render_view.cpp
 * Brief: View/3D frame rendering: the world geometry pass behind PS2_RenderFrame.
 *
 *  RenderFrame walks the world BSP for the refdef's camera: MarkLeaves stamps the
 *  nodes reachable from the current PVS cluster, RecursiveWorldNode descends the
 *  tree front-to-back culling against the view frustum and threads every visible
 *  opaque surface onto its texture's draw chain, and DrawTextureChains then
 *  gathers each chain's triangles into a scratch buffer and submits them through
 *  vu1::DrawTriangles - one synchronous batch per texture. Sky and translucent
 *  surfaces are routed aside for later passes (skybox / alpha-blend milestones).
 *
 *  Camera mapping: Quake is Z-up with AngleVectors giving forward/right/up; those
 *  feed math::LookAt directly (its right = cross(up, -forward) lands on Quake's
 *  own right vector) and PerspectiveProjection's Y-flip puts +up up on screen, so
 *  no axis juggling is needed between the engine and the GS.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/common.h"
#include "ps2/renderer/render_view.h"
#include "ps2/renderer/render_md2.h"
#include "ps2/renderer/texture.h"
#include "ps2/renderer/model.h"
#include "ps2/renderer/vu1.h"
#include "ps2/renderer/gs.h"
#include "ps2/math/vec_mat.h"

#include <cstring>

namespace ps2::view {
namespace {

// Depth range for the world projection (ref_gl's values).
constexpr float kZNear = 4.0f;
constexpr float kZFar  = 4096.0f;

// Vertex colour for the not-yet-lit world: GS modulate 128 = texels unchanged.
constexpr u32 kFullBright = vu1::PackColorRGBA(128, 128, 128, 0x80);

// ------------------------------------------------------------------------------------------------
// Frame state
// ------------------------------------------------------------------------------------------------

// Frame counters used to mark drawable surfaces:
static int s_frameCount    = 0; // Bumped per RenderFrame; stamps surfaces marked for draw.
static int s_visFrameCount = 0; // Bumped when the PVS changes; stamps reachable nodes.

// Quake 2 view clusters; BeginRegistration() resets them for a new map.
constexpr int kInvalidCluster = -1;
static int s_viewCluster      = kInvalidCluster;
static int s_viewCluster2     = kInvalidCluster;
static int s_oldViewCluster   = kInvalidCluster;
static int s_oldViewCluster2  = kInvalidCluster;

// Scene camera basis for the frame (Quake coordinates, from AngleVectors).
static vec3_t s_forwardVec = {};
static vec3_t s_rightVec   = {};
static vec3_t s_upVec      = {};

// World-to-clip transform for the frame (world geometry draws in world space).
static math::Mat4 s_viewProjMatrix = {};

// View frustum side planes (left, right, bottom, top) for bounding-box culling.
static cplane_t s_frustum[4] = {};

// Wall texture animation frame (viewDef.time * 2, as in ref_gl).
static int s_textureAnimFrame = 0;

// Scratch for one decompressed cluster PVS, and for the two-cluster union used
// when the camera straddles a solid water boundary. Static: 8 KB each.
alignas(16) static u8 s_dvisPvs[MAX_MAP_LEAFS / 8];
alignas(16) static u8 s_fatPvs[MAX_MAP_LEAFS / 8];

// Textures that received surfaces this frame; DrawTextureChains draws and
// resets exactly these. Sized to the texture cache capacity.
constexpr int kMaxChainTextures = 1024;
static const tex::Texture * s_chainTextures[kMaxChainTextures];
static int s_chainTextureCount = 0;

// World surfaces with transparency (glass/water), chained back-to-front while
// walking the BSP. Collected so they stay out of the opaque chains; the pass
// that draws them lands with the alpha-blend milestone.
static const mod::ModelSurface * s_alphaSurfaces = nullptr;

// Triangle gather buffer: texture chains append here and flush through
// vu1::DrawTriangles when full. DrawTriangles is synchronous, so one buffer
// serves every batch in turn, referenced in place by the DMA chain.
constexpr int kScratchMaxVerts = 3072; // whole triangles (3 * 1024); 96 KB
alignas(16) static vu1::DrawVertex s_scratchVerts[kScratchMaxVerts];
static int s_scratchVertCount = 0;

// Performance counters for the frame, reset by RenderFrame and read through
// GetDrawStats() by the ps2_show_drawstats overlay.
static DrawStats s_drawStats = {};

// ------------------------------------------------------------------------------------------------
// Frame setup: camera matrices and frustum
// ------------------------------------------------------------------------------------------------

inline int SignBitsForPlane(const cplane_t & plane)
{
    // Sign bits are used for fast box-on-plane-side tests.
    int bits = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (plane.normal[i] < 0.0f)
        {
            bits |= (1 << i);
        }
    }
    return bits;
}

// Builds the four frustum side planes by rotating the view direction around
// the up/right axes by half the FOV (ref_gl's R_SetFrustum construction).
void SetUpFrustum(const refdef_t & viewDef)
{
    RotatePointAroundVector(s_frustum[0].normal, s_upVec,    s_forwardVec, -(90.0f - viewDef.fov_x * 0.5f));
    RotatePointAroundVector(s_frustum[1].normal, s_upVec,    s_forwardVec,  (90.0f - viewDef.fov_x * 0.5f));
    RotatePointAroundVector(s_frustum[2].normal, s_rightVec, s_forwardVec,  (90.0f - viewDef.fov_y * 0.5f));
    RotatePointAroundVector(s_frustum[3].normal, s_rightVec, s_forwardVec, -(90.0f - viewDef.fov_y * 0.5f));

    for (cplane_t & plane : s_frustum)
    {
        plane.type     = PLANE_ANYZ;
        plane.dist     = DotProduct(viewDef.vieworg, plane.normal);
        plane.signbits = static_cast<byte>(SignBitsForPlane(plane));
    }
}

// True when the box is completely outside the frustum and must not draw.
inline bool ShouldCullBBox(float * mins, float * maxs)
{
    for (cplane_t & plane : s_frustum)
    {
        if (BOX_ON_PLANE_SIDE(mins, maxs, &plane) == 2)
        {
            ++s_drawStats.boxesCulled;
            return true;
        }
    }
    return false;
}

void SetupFrame(const refdef_t & viewDef)
{
    ++s_frameCount;

    // Animated walls flip frames at 2 Hz of game time (as in ref_gl).
    s_textureAnimFrame = static_cast<int>(viewDef.time * 2.0f);

    // Camera basis vectors from the view angles.
    AngleVectors(viewDef.viewangles, s_forwardVec, s_rightVec, s_upVec);

    const math::Vec3 eye    = { viewDef.vieworg[0], viewDef.vieworg[1], viewDef.vieworg[2] };
    const math::Vec3 target = { eye.x + s_forwardVec[0], eye.y + s_forwardVec[1], eye.z + s_forwardVec[2] };
    const math::Vec3 up     = { s_upVec[0], s_upVec[1], s_upVec[2] };

    const math::Mat4 view = math::LookAt(eye, target, up);
    const math::Mat4 proj = math::PerspectiveProjection(
        math::DegToRad(viewDef.fov_y),
        static_cast<float>(viewDef.width) / static_cast<float>(viewDef.height),
        static_cast<float>(gs::Width()), static_cast<float>(gs::Height()),
        kZNear, kZFar);

    s_viewProjMatrix = view * proj;

    SetUpFrustum(viewDef);
}

// ------------------------------------------------------------------------------------------------
// PVS / visibility
// ------------------------------------------------------------------------------------------------

const mod::ModelLeaf * FindLeafNodeForPoint(const float * point, const mod::ModelInstance & model)
{
    PS2_AssertMsg(model.nodes != nullptr, "World model has no nodes!");

    const mod::ModelNode * node = model.nodes;
    for (;;)
    {
        if (node->contents != -1)
        {
            return reinterpret_cast<const mod::ModelLeaf *>(node);
        }

        const cplane_t * const plane = node->plane;
        const float d = DotProduct(point, plane->normal) - plane->dist;
        node = (d > 0.0f) ? node->children[0] : node->children[1];
    }
}

// Decompresses a cluster's RLE visibility row into 'out' (zero runs are
// length-encoded). A null 'in' (no vis data) decompresses to all-visible.
const u8 * DecompressModelVis(u8 * out, const u8 * in, const mod::ModelInstance & model)
{
    const auto * vis = static_cast<const dvis_t *>(model.vis);
    int row = (vis->numclusters + 7) >> 3;
    u8 * dst = out;

    if (in == nullptr)
    {
        while (row-- > 0)
        {
            *dst++ = 0xFF;
        }
        return out;
    }

    do
    {
        if (*in != 0)
        {
            *dst++ = *in++;
            continue;
        }

        int c = in[1];
        in += 2;
        while (c-- > 0)
        {
            *dst++ = 0;
        }
    } while (dst - out < row);

    return out;
}

// Returns the decompressed PVS row for 'cluster' (shared s_dvisPvs buffer, so
// don't hold on to it across calls).
const u8 * GetClusterPVS(int cluster, const mod::ModelInstance & model)
{
    if (cluster == kInvalidCluster || model.vis == nullptr)
    {
        std::memset(s_dvisPvs, 0xFF, sizeof(s_dvisPvs)); // All visible.
        return s_dvisPvs;
    }

    const auto * vis = static_cast<const dvis_t *>(model.vis);
    const u8 * compressed = static_cast<const u8 *>(model.vis) + vis->bitofs[cluster][DVIS_PVS];
    return DecompressModelVis(s_dvisPvs, compressed, model);
}

// Finds the clusters the camera sees from this frame. Two clusters when near a
// solid water surface, so crossing it doesn't draw wrong (checked by sampling a
// second leaf 16 units above/below the eye).
void SetUpViewClusters(const refdef_t & viewDef, const mod::ModelInstance & world)
{
    const mod::ModelLeaf * leaf = FindLeafNodeForPoint(viewDef.vieworg, world);

    s_oldViewCluster  = s_viewCluster;
    s_oldViewCluster2 = s_viewCluster2;
    s_viewCluster = s_viewCluster2 = leaf->cluster;

    vec3_t temp;
    VectorCopy(viewDef.vieworg, temp);
    temp[2] += (leaf->contents == 0) ? -16.0f : 16.0f;

    leaf = FindLeafNodeForPoint(temp, world);
    if (!(leaf->contents & CONTENTS_SOLID) && (leaf->cluster != s_viewCluster2))
    {
        s_viewCluster2 = leaf->cluster;
    }
}

// Stamps the leafs in the current clusters' PVS - and the node chains above
// them - with the new vis frame count. Skipped entirely while the camera stays
// in the same cluster(s), which is the common case.
void MarkLeaves(const mod::ModelInstance & world)
{
    if (s_oldViewCluster  == s_viewCluster  &&
        s_oldViewCluster2 == s_viewCluster2 &&
        s_viewCluster != kInvalidCluster)
    {
        return; // Same clusters as the previous frame; marks still valid.
    }

    ++s_visFrameCount;
    s_oldViewCluster  = s_viewCluster;
    s_oldViewCluster2 = s_viewCluster2;

    if (s_viewCluster == kInvalidCluster || world.vis == nullptr)
    {
        // Outside the map or no PVS data: mark everything visible.
        for (int i = 0; i < world.numLeafs; ++i)
        {
            world.leafs[i].visFrame = s_visFrameCount;
        }
        for (int i = 0; i < world.numNodes; ++i)
        {
            world.nodes[i].visFrame = s_visFrameCount;
        }
        return;
    }

    const u8 * vis = GetClusterPVS(s_viewCluster, world);

    // May have to combine two clusters because of solid water boundaries:
    if (s_viewCluster2 != s_viewCluster)
    {
        std::memcpy(s_fatPvs, vis, static_cast<size_t>((world.numLeafs + 7) / 8));
        vis = GetClusterPVS(s_viewCluster2, world);

        // Both buffers are 16-byte aligned, so OR them a word at a time.
        u32 * fat = static_cast<u32 *>(static_cast<void *>(s_fatPvs));
        const u32 * add = static_cast<const u32 *>(static_cast<const void *>(vis));

        const int words = (world.numLeafs + 31) / 32;
        for (int i = 0; i < words; ++i)
        {
            fat[i] |= add[i];
        }
        vis = s_fatPvs;
    }

    mod::ModelLeaf * leaf = world.leafs;
    for (int i = 0; i < world.numLeafs; ++i, ++leaf)
    {
        const int cluster = leaf->cluster;
        if (cluster == kInvalidCluster)
        {
            continue;
        }

        if (vis[cluster >> 3] & (1 << (cluster & 7)))
        {
            auto * node = reinterpret_cast<mod::ModelNode *>(leaf);
            do
            {
                if (node->visFrame == s_visFrameCount)
                {
                    break; // This branch is already marked up to the root.
                }
                node->visFrame = s_visFrameCount;
                node = node->parent;
            } while (node != nullptr);
        }
    }
}

// ------------------------------------------------------------------------------------------------
// World BSP walk and texture chains
// ------------------------------------------------------------------------------------------------

// Returns the texture a surface draws with this frame, following the animation
// chain for animated walls (torches, screens). Never null: the model loader
// substitutes the debug checkerboard for missing wall textures.
const tex::Texture * TextureAnimation(const mod::ModelTexInfo * texInfo)
{
    PS2_Assert(texInfo != nullptr && texInfo->texture != nullptr);

    if (texInfo->next == nullptr)
    {
        return texInfo->texture; // Not animated.
    }

    int c = s_textureAnimFrame % texInfo->numFrames;
    while (c-- > 0)
    {
        texInfo = texInfo->next;
    }
    return texInfo->texture;
}

// Recursively marks and chains the visible world surfaces: walks the BSP
// front-to-back, culling nodes against the PVS marks and the view frustum,
// and threads each drawable surface onto its texture's chain so the next
// DrawTextureChains() call renders what was collected here.
void RecursiveWorldNode(const refdef_t & viewDef, const mod::ModelInstance & world, mod::ModelNode * node)
{
    if (node->contents == CONTENTS_SOLID)
    {
        return;
    }
    if (node->visFrame != s_visFrameCount)
    {
        return; // Not reachable from the current PVS cluster.
    }
    if (ShouldCullBBox(node->minmaxs, node->minmaxs + 3))
    {
        return; // Entirely outside the view frustum.
    }

    ++s_drawStats.nodesWalked;

    // Leaf: stamp its surfaces as drawable this frame.
    if (node->contents != -1)
    {
        auto * leaf = reinterpret_cast<mod::ModelLeaf *>(node);

        // Check for door-connected areas:
        if (viewDef.areabits != nullptr)
        {
            if (!(viewDef.areabits[leaf->area >> 3] & (1 << (leaf->area & 7))))
            {
                return; // Not visible.
            }
        }

        mod::ModelSurface ** mark = leaf->firstMarkSurface;
        for (int i = 0; i < leaf->numMarkSurfaces; ++i, ++mark)
        {
            (*mark)->visFrame = s_frameCount;
        }
        return;
    }

    // Decision node: find which side of its plane the camera is on.
    float dot;
    const cplane_t * const plane = node->plane;
    switch (plane->type)
    {
    case PLANE_X:
        dot = viewDef.vieworg[0] - plane->dist;
        break;
    case PLANE_Y:
        dot = viewDef.vieworg[1] - plane->dist;
        break;
    case PLANE_Z:
        dot = viewDef.vieworg[2] - plane->dist;
        break;
    default:
        dot = DotProduct(viewDef.vieworg, plane->normal) - plane->dist;
        break;
    }

    const int  side          = (dot >= 0.0f) ? 0 : 1;
    const bool cameraOnBack  = (side == 1);

    // Recurse down the camera side first (front-to-back order)...
    RecursiveWorldNode(viewDef, world, node->children[side]);

    // ...then chain this node's surfaces that face the camera...
    mod::ModelSurface * surf = world.surfaces + node->firstSurface;
    for (int i = 0; i < node->numSurfaces; ++i, ++surf)
    {
        if (surf->visFrame != s_frameCount)
        {
            continue; // Not in a visible leaf.
        }
        if (HasFlag(surf->flags, mod::SurfaceFlags::PlaneBack) != cameraOnBack)
        {
            continue; // Facing away from the camera.
        }

        const int texFlags = surf->texInfo->flags;
        if (texFlags & SURF_SKY)
        {
            // TODO: Sky surfaces feed the skybox bounds (skybox milestone).
            continue;
        }

        if (texFlags & (SURF_TRANS33 | SURF_TRANS66 | SURF_WARP))
        {
            // Translucent/warped: kept for a later back-to-front alpha pass.
            surf->textureChain = s_alphaSurfaces;
            s_alphaSurfaces = surf;
            ++s_drawStats.surfacesAlpha;
            continue;
        }

        // Opaque: thread onto its texture's draw chain.
        ++s_drawStats.surfaces;
        const tex::Texture * texture = TextureAnimation(surf->texInfo);
        if (texture->textureChain == nullptr)
        {
            // First surface for this texture this frame; remember the chain.
            PS2_AssertMsg(s_chainTextureCount < kMaxChainTextures, "Out of texture chain slots!");
            s_chainTextures[s_chainTextureCount++] = texture;
        }
        surf->textureChain    = texture->textureChain;
        texture->textureChain = surf;
    }

    // ...and finally recurse down the far side.
    RecursiveWorldNode(viewDef, world, node->children[side ^ 1]);
}

// ------------------------------------------------------------------------------------------------
// Triangle gathering and submission
// ------------------------------------------------------------------------------------------------

// Sends the gathered scratch triangles as one batch and empties the buffer.
inline void FlushScratch(const tex::Texture & texture)
{
    if (s_scratchVertCount > 0)
    {
        ++s_drawStats.drawBatches;
        vu1::DrawTriangles(s_viewProjMatrix, texture, s_scratchVerts, s_scratchVertCount);
        s_scratchVertCount = 0;
    }
}

// ------------------------------------------------------------------------------------------------
// Triangle clipping against the VU clip volume
//
// The VU microprogram does not clip: a triangle with any vertex outside its
// clip volume is rejected whole (ADC bit), so world triangles must be
// pre-clipped here on the EE against all six planes the VU judges - near and
// far (z is judged exactly), and the four guard-band side planes. The sides
// matter just as much as near: a floor polygon clipped at the near plane
// right under the camera lands at tiny w and enormous |x/w|, far outside any
// band the GS 12.4 coordinates could hold. Clipping runs in clip space - a
// vertex is inside while w-z >= 0 (near; also excludes everything behind the
// camera), w+z >= 0 (far), and G*w +/- x/y >= 0 (sides, G = the VU guard-band
// limit). Everything a vertex carries - world position, UVs, and the six
// distances themselves - is linear under a plane cut, so a split interpolates
// the whole ClipVertex as four quadword lerps on VU0, no scalar float math.
// Whole-triangle-inside is the common case and skips all of it.
// ------------------------------------------------------------------------------------------------

constexpr int kNumClipPlanes = 6;

// Clip a hair early so the VU's judgement never flags a vertex this clipper
// just placed on the boundary (clip-space units, i.e. ~world units here).
constexpr float kClipEpsilon = 0.01f;

// Signed distances to the clip planes; >= 0 is inside. Held as two whole quadwords
// (six planes, two spare lanes) so the whole set interpolates with two vector lerps
// while the per-plane tests still read it as a plain float array.
union ClipDists
{
    math::Vec4 q[2];
    float f[8];
};

// Everything a clipped vertex carries, laid out as four quadwords so a plane cut
// interpolates it with four aligned vector lerps and no scalar float math at all.
struct alignas(16) ClipVertex
{
    math::Vec4 pos; // world position, w = 1
    math::Vec4 st;  // diffuse texture coords in xy; zw unused
    ClipDists  d;
};
static_assert(sizeof(ClipVertex) == 64, "ClipVertex must be exactly four quadwords");

// Sutherland-Hodgman pass of a convex polygon against one plane. 'out' must
// hold inCount + 1 vertexes. Returns the clipped vertex count.
int ClipAgainstPlane(const ClipVertex * in, const int inCount, ClipVertex * out, const int plane)
{
    int outCount = 0;
    for (int i = 0; i < inCount; ++i)
    {
        const ClipVertex & a = in[i];
        const ClipVertex & b = in[(i + 1 == inCount) ? 0 : i + 1];

        if (a.d.f[plane] >= 0.0f)
        {
            out[outCount++] = a;
        }
        if ((a.d.f[plane] >= 0.0f) != (b.d.f[plane] >= 0.0f)) // Edge crosses the plane.
        {
            const float t = a.d.f[plane] / (a.d.f[plane] - b.d.f[plane]);
            ClipVertex & o = out[outCount++];
            math::LerpTo(o.pos,    a.pos,    b.pos,    t);
            math::LerpTo(o.st,     a.st,     b.st,     t);
            math::LerpTo(o.d.q[0], a.d.q[0], b.d.q[0], t);
            math::LerpTo(o.d.q[1], a.d.q[1], b.d.q[1], t);
        }
    }
    return outCount;
}

inline void EmitScratchVertex(const ClipVertex & v)
{
    vu1::DrawVertex & dst = s_scratchVerts[s_scratchVertCount++];
    dst.x    = v.pos.x;
    dst.y    = v.pos.y;
    dst.z    = v.pos.z;
    dst.w    = 1.0f;
    dst.rgba = kFullBright;
    dst.s    = v.st.x;
    dst.t    = v.st.y;
    dst.q    = 1.0f;
}

// Appends a polygon's triangles to the scratch buffer, clipping the ones that
// cross the VU clip volume and flushing when full.
void GatherPolyTriangles(const mod::ModelPoly & poly, const tex::Texture & texture)
{
    const int numTriangles = poly.numVerts - 2;
    for (int t = 0; t < numTriangles; ++t)
    {
        const mod::ModelTriangle & tri = poly.triangles[t];

        // Polygons the triangulation couldn't complete leave zeroed
        // (degenerate) triangles behind; skip them.
        if (tri.vertexes[0] == tri.vertexes[1])
        {
            continue;
        }

        // Clip-space plane distances of the three corners (world geometry
        // draws untransformed, so the point transform is the view-projection).
        ClipVertex corners[3];
        int insidePerPlane[kNumClipPlanes] = {};
        for (int v = 0; v < 3; ++v)
        {
            const mod::PolyVertex & src = poly.vertexes[tri.vertexes[v]];
            ClipVertex & c = corners[v];

            c.pos = { src.position.x, src.position.y, src.position.z, 1.0f };
            c.st  = { src.texture_s, src.texture_t, 0.0f, 0.0f };

            const math::Vec4 clip = math::Transform(c.pos, s_viewProjMatrix);
            const float gw = vu1::kGuardBandNdcLimit * clip.w;
            c.d.f[0] = (clip.w - clip.z) - kClipEpsilon; // near (and behind-camera)
            c.d.f[1] = (clip.w + clip.z) - kClipEpsilon; // far
            c.d.f[2] = (gw - clip.x) - kClipEpsilon;     // guard band sides
            c.d.f[3] = (gw + clip.x) - kClipEpsilon;
            c.d.f[4] = (gw - clip.y) - kClipEpsilon;
            c.d.f[5] = (gw + clip.y) - kClipEpsilon;
            c.d.f[6] = 0.0f; // Spare lanes: never read as planes, but they ride
            c.d.f[7] = 0.0f; // along through the lerps, so keep them finite.

            for (int p = 0; p < kNumClipPlanes; ++p)
            {
                insidePerPlane[p] += (c.d.f[p] >= 0.0f);
            }
        }

        int insideTotal = 0;
        bool outsideAny = false;
        for (int p = 0; p < kNumClipPlanes; ++p)
        {
            insideTotal += insidePerPlane[p];
            outsideAny  |= (insidePerPlane[p] == 0);
        }

        if (outsideAny)
        {
            ++s_drawStats.trisCulled;
            continue; // All three corners outside one plane: entirely out.
        }

        if (insideTotal == 3 * kNumClipPlanes)
        {
            // Whole triangle inside: the common case, no clipping.
            ++s_drawStats.trisDrawn;
            if (s_scratchVertCount + 3 > kScratchMaxVerts)
            {
                FlushScratch(texture);
            }
            for (const ClipVertex & c : corners)
            {
                EmitScratchVertex(c);
            }
            continue;
        }

        // Straddling triangle: clip against every plane in turn. Each pass
        // can add one vertex (3 -> at most 9); the survivors fan-triangulate.
        ++s_drawStats.trisClipped;
        ClipVertex bufferA[3 + kNumClipPlanes];
        ClipVertex bufferB[3 + kNumClipPlanes];

        const ClipVertex * in = corners;
        ClipVertex * out = bufferA;
        int count = 3;
        for (int p = 0; p < kNumClipPlanes && count >= 3; ++p)
        {
            if (insidePerPlane[p] == 3)
            {
                continue; // All corners inside this plane: every clipped
                          // vertex is a convex mix of them, so none can
                          // cross it either.
            }
            count = ClipAgainstPlane(in, count, out, p);
            in  = out;
            out = (out == bufferA) ? bufferB : bufferA;
        }
        if (count < 3)
        {
            continue;
        }

        // TODO: Cull and reject back-facing triangles.

        if (s_scratchVertCount + (count - 2) * 3 > kScratchMaxVerts)
        {
            FlushScratch(texture);
        }
        for (int v = 1; v < count - 1; ++v)
        {
            EmitScratchVertex(in[0]);
            EmitScratchVertex(in[v]);
            EmitScratchVertex(in[v + 1]);
        }
        s_drawStats.trisDrawn += count - 2;
    }
}

// Draws every texture chain built by RecursiveWorldNode and resets them.
void DrawTextureChains()
{
    for (int i = 0; i < s_chainTextureCount; ++i)
    {
        const tex::Texture * texture = s_chainTextures[i];

        for (const mod::ModelSurface * surf = texture->textureChain; surf != nullptr; surf = surf->textureChain)
        {
            for (const mod::ModelPoly * poly = surf->polys; poly != nullptr; poly = poly->next)
            {
                if (poly->numVerts >= 3) // Need at least one triangle.
                {
                    GatherPolyTriangles(*poly, *texture);
                }
            }
        }
        FlushScratch(*texture);

        texture->textureChain = nullptr; // Reset for the next frame.
    }
    s_chainTextureCount = 0;
}

// ------------------------------------------------------------------------------------------------
// World model pass
// ------------------------------------------------------------------------------------------------

void RenderWorldModel(const refdef_t & viewDef)
{
    static const cvar_t * s_skipWorld = Cvar_Get("ps2_skip_world", "0", 0);

    s_alphaSurfaces = nullptr;

    if (viewDef.rdflags & RDF_NOWORLDMODEL)
    {
        return; // Menu/loading screens render no world.
    }
    if (s_skipWorld->value != 0.0f)
    {
        return; // Debug: skip the world pass entirely.
    }

    const mod::ModelInstance * world = mod::GetWorldModel();
    PS2_AssertMsg(world != nullptr, "RenderFrame without a world model!");

    SetUpViewClusters(viewDef, *world);
    MarkLeaves(*world);
    RecursiveWorldNode(viewDef, *world, world->nodes);
    DrawTextureChains();
}

// ------------------------------------------------------------------------------------------------
// Point lighting (world lightmap sampling for entity models)
// ------------------------------------------------------------------------------------------------

enum LightSampleResult : int { NoHit = -1, Hit = 0, HitColorSampled = 1 };

// Descends the BSP along the start->end segment looking for the first lit
// surface it crosses (ref_gl's RecursiveLightPoint): finds the node plane the
// segment straddles, recurses the near side, and on the way back samples the
// lightmap of the node surface containing the crossing point - each style's
// sample scaled by its current lightstyle. Returns -1 for no hit, 0 for a hit
// with no light data, 1 for a sampled hit ('outColor' and 'outLightSpot' set).
LightSampleResult RecursiveLightPoint(const mod::ModelInstance & world, const mod::ModelNode * node,
                                      const lightstyle_t * lightstyles, const vec3_t start, const vec3_t end,
                                      vec3_t outColor, vec3_t outLightSpot)
{
    if (node->contents != -1)
    {
        return NoHit; // Leaf: didn't hit anything on the way down.
    }

    // Which side(s) of this node's plane does the segment touch?
    const cplane_t * const plane = node->plane;
    const float front = DotProduct(start, plane->normal) - plane->dist;
    const float back  = DotProduct(end,   plane->normal) - plane->dist;
    const int   side  = (front < 0.0f);

    if ((back < 0.0f) == side)
    {
        // Whole segment on one side; no surface of this node can be crossed.
        return RecursiveLightPoint(world, node->children[side], lightstyles, start, end, outColor, outLightSpot);
    }

    // The segment crosses the plane at 'mid': trace the near half first.
    const float frac = front / (front - back);

    vec3_t mid;
    mid[0] = start[0] + (end[0] - start[0]) * frac;
    mid[1] = start[1] + (end[1] - start[1]) * frac;
    mid[2] = start[2] + (end[2] - start[2]) * frac;

    const auto r = RecursiveLightPoint(world, node->children[side], lightstyles, start, mid, outColor, outLightSpot);
    if (r >= Hit)
    {
        return r; // Hit something nearer.
    }

    VectorCopy(mid, outLightSpot);

    // Check the crossing point against this node's surfaces.
    const int numSurfaces = node->numSurfaces;
    const mod::ModelSurface * surf = world.surfaces + node->firstSurface;
    for (int i = 0; i < numSurfaces; ++i, ++surf)
    {
        if (HasFlag(surf->flags, mod::SurfaceFlags::DrawTurb | mod::SurfaceFlags::DrawSky))
        {
            continue; // No lightmaps on water/sky.
        }

        const mod::ModelTexInfo * tex = surf->texInfo;

        const int s = static_cast<int>(DotProduct(mid, tex->vecs[0]) + tex->vecs[0][3]);
        const int t = static_cast<int>(DotProduct(mid, tex->vecs[1]) + tex->vecs[1][3]);

        if (s < surf->textureMins[0] || t < surf->textureMins[1])
        {
            continue;
        }

        int ds = s - surf->textureMins[0];
        int dt = t - surf->textureMins[1];

        if (ds > surf->extents[0] || dt > surf->extents[1])
        {
            continue;
        }

        if (surf->samples == nullptr)
        {
            return Hit; // Hit, but the surface carries no light data.
        }

        // 16-texel lightmap granularity, one RGB triplet per luxel, one
        // whole map per active style.
        ds >>= 4;
        dt >>= 4;

        VectorClear(outColor);

        const u8 * lightmap = surf->samples;
        lightmap += 3 * (dt * ((surf->extents[0] >> 4) + 1) + ds);

        for (int map = 0; map < mod::kMaxLightmaps && surf->styles[map] != 255; ++map)
        {
            const float * styleRGB = lightstyles[surf->styles[map]].rgb;

            outColor[0] += lightmap[0] * styleRGB[0] * (1.0f / 255.0f);
            outColor[1] += lightmap[1] * styleRGB[1] * (1.0f / 255.0f);
            outColor[2] += lightmap[2] * styleRGB[2] * (1.0f / 255.0f);

            lightmap += 3 * ((surf->extents[0] >> 4) + 1) * ((surf->extents[1] >> 4) + 1);
        }

        return HitColorSampled;
    }

    // Nothing on this node; carry on down the far half of the segment.
    return RecursiveLightPoint(world, node->children[!side], lightstyles, mid, end, outColor, outLightSpot);
}

// ------------------------------------------------------------------------------------------------
// Entity pass
// ------------------------------------------------------------------------------------------------

void RenderEntities(const refdef_t & viewDef, const bool translucentPass)
{
    static const cvar_t * s_skipEntities = Cvar_Get("ps2_skip_entities", "0", 0);

    if (s_skipEntities->value != 0.0f)
    {
        return; // Debug: skip all entity models.
    }

    const int numEntities = viewDef.num_entities;
    for (int e = 0; e < numEntities; ++e)
    {
        const entity_t & entity = viewDef.entities[e];

        if (entity.flags & RF_BEAM)
        {
            continue; // TODO: beam cylinders (effects milestone).
        }

        // Translucents draw after every solid, so they blend over a finished
        // opaque scene rather than whatever happened to be drawn so far.
        const bool translucent = (entity.flags & RF_TRANSLUCENT) != 0;
        if (translucent != translucentPass)
        {
            continue;
        }

        // entity_t::model is opaque outside the refresh module, hence the cast.
        const auto * model = reinterpret_cast<const mod::ModelInstance *>(entity.model);
        if (model == nullptr)
        {
            // TODO: null-model placeholder (entity models milestone).
            continue;
        }

        switch (model->type)
        {
        case mod::ModelType::AliasMD2:
            DrawAliasMD2Entity(viewDef, entity, s_viewProjMatrix);
            break;

        case mod::ModelType::Brush:
        case mod::ModelType::Sprite:
            // TODO: brush and sprite entity models (entity models milestone).
            break;
        }
    }
}

} // namespace

// ------------------------------------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------------------------------------

void BeginRegistration()
{
    s_drawStats = {};

    // New map: forget the previous map's clusters so the first frame re-marks.
    s_viewCluster     = kInvalidCluster;
    s_viewCluster2    = kInvalidCluster;
    s_oldViewCluster  = kInvalidCluster;
    s_oldViewCluster2 = kInvalidCluster;
}

DrawStats & GetDrawStats()
{
    return s_drawStats;
}

void CalcPointLightColor(const refdef_t & viewDef, const vec3_t point,
                         vec3_t outColor, vec3_t outLightSpot)
{
    const mod::ModelInstance * world = mod::GetWorldModel();

    if (world == nullptr || world->lightData == nullptr)
    {
        // No world or a map compiled without light data: fullbright.
        VectorSet(outColor, 1.0f, 1.0f, 1.0f);
        return;
    }

    // Trace straight down; 2048 units reaches the floor from anywhere sane.
    const vec3_t endPoint = { point[0], point[1], point[2] - 2048.0f };

    vec3_t sampled = {};
    const auto r = RecursiveLightPoint(*world, world->nodes, viewDef.lightstyles,
                                       point, endPoint, sampled, outLightSpot);
    if (r == NoHit)
    {
        VectorClear(outColor); // Left the world without hitting anything.
    }
    else
    {
        VectorCopy(sampled, outColor);
    }

    // Add the frame's dynamic lights, falling off linearly with distance.
    const dlight_t * dl = viewDef.dlights;
    const int numDlights = viewDef.num_dlights;
    for (int i = 0; i < numDlights; ++i, ++dl)
    {
        vec3_t dist;
        VectorSubtract(point, dl->origin, dist);

        const float add = (dl->intensity - VectorLength(dist)) * (1.0f / 256.0f);
        if (add > 0.0f)
        {
            outColor[0] += add * dl->color[0];
            outColor[1] += add * dl->color[1];
            outColor[2] += add * dl->color[2];
        }
    }
}

bool FrustumCullsPoints(const vec3_t * points, int numPoints)
{
    // Each point's mask has one bit per side plane it is outside of; the AND
    // across all points is nonzero exactly when one plane excludes them all.
    // (Conservative: a box crossing a frustum corner passes and draws.)
    u32 aggregate = ~0u;
    for (int i = 0; i < numPoints; ++i)
    {
        u32 mask = 0;
        for (int p = 0; p < 4; ++p)
        {
            if (DotProduct(points[i], s_frustum[p].normal) - s_frustum[p].dist < 0.0f)
            {
                mask |= (1u << p);
            }
        }

        aggregate &= mask;
        if (aggregate == 0)
        {
            return false; // No plane excludes every point seen so far.
        }
    }
    return true;
}

void RenderFrame(const refdef_t & viewDef)
{
    PS2_Assert(viewDef.width > 0 && viewDef.height > 0);

    s_drawStats = {};

    SetupFrame(viewDef);

    RenderWorldModel(viewDef);
    RenderEntities(viewDef, /*translucentPass=*/false);
    RenderEntities(viewDef, /*translucentPass=*/true);

    // Later milestones continue here: translucent world surfaces,
    // particles, dynamic lights.
}

} // namespace ps2::view
