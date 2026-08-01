/* ================================================================================================
 * File: render_view.cpp
 * Brief: View/3D frame rendering: the world geometry pass behind PS2_RenderFrame.
 *
 *  RenderFrame walks the world BSP for the refdef's camera: MarkLeaves stamps the
 *  nodes reachable from the current PVS cluster, RecursiveWorldNode descends the
 *  tree front-to-back culling against the view frustum and threads every visible
 *  opaque surface onto its texture's draw chain, and DrawTextureChains then
 *  gathers each chain's triangles into a scratch buffer and submits them through
 *  vu1::DrawTriangles - one synchronous batch per texture. Translucent surfaces
 *  are routed aside and drawn back-to-front at the end of the frame by
 *  RenderAlphaSurfaces; sky still awaits the skybox milestone.
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
#include "ps2/builtin/builtin.h" // global_palette (beam and particle colours)

#include <cmath>
#include <cstring>

namespace ps2::view {
namespace {

// Depth range for the world projection (ref_gl's values).
constexpr float kZNear = 4.0f;
constexpr float kZFar  = 4096.0f;

// Vertex colour for the not-yet-lit world: GS modulate 128 = texels unchanged.
constexpr u32 kFullBright = vu1::PackColorRGBA(128, 128, 128, 0x80);

// Render view cvars:
static const cvar_t * s_backFaceCull      = nullptr;
static const cvar_t * s_skipWorld         = nullptr;
static const cvar_t * s_skipAlphaSurfaces = nullptr;
static const cvar_t * s_skipBrushModels   = nullptr;
static const cvar_t * s_skipSprites       = nullptr;
static const cvar_t * s_skipEntities      = nullptr;
static const cvar_t * s_skipParticles     = nullptr;
static const cvar_t * s_hdParticles       = nullptr;
static const cvar_t * s_forceNullModels   = nullptr;
static const cvar_t * s_skipWeaponModel   = nullptr;

// Not ours to read: SetLightLevel writes the sampled light level back into it
// every frame for the game code (hence non-const). Registered by the client
// (cl_main.c), which forwards it to the server in each usercmd.
static cvar_t * s_lightLevel = nullptr;

// ------------------------------------------------------------------------------------------------
// Frame state
// ------------------------------------------------------------------------------------------------

// Game time of the frame, in seconds; drives the water warp animation.
static float s_frameTime = 0.0f;

// Frame counters used to mark drawable surfaces:
static int s_frameCount    = 0; // Bumped per RenderFrame; stamps surfaces marked for draw.
static int s_visFrameCount = 0; // Bumped when the PVS changes; stamps reachable nodes.

// Quake 2 view clusters; BeginRegistration() resets them for a new map.
constexpr int kInvalidCluster = -1;
static int s_viewCluster      = kInvalidCluster;
static int s_viewCluster2     = kInvalidCluster;
static int s_oldViewCluster   = kInvalidCluster;
static int s_oldViewCluster2  = kInvalidCluster;

// World-space camera position for the frame, in the format the VU0 back-face
// helper wants (16-byte aligned, w = 1).
static math::Vec4 s_eyePosition = {};

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

// Surfaces with transparency (glass, water, lava, slime), collected while
// walking the BSP and while drawing brush model entities, then drawn last by
// RenderAlphaSurfaces over the finished opaque scene.
//
// Recorded here rather than threaded through ModelSurface::textureChain like
// the opaque chains because a brush model's surfaces have to remember the
// entity transform they were collected under. ref_gl loses it - its
// R_DrawAlphaSurfaces reloads the world matrix, so a moving submodel's water
// draws back at the map's rest position - and the records cost little.
struct AlphaSurface
{
    const mod::ModelSurface * surf;
    const tex::Texture *      texture; // Resolved on collection: the animation frame is the entity's.
    const math::Mat4 *        mvp;     // &s_viewProjMatrix, or into s_alphaEntityMatrices below.
};

constexpr int kMaxAlphaSurfaces = 1024;
static AlphaSurface s_alphaSurfaces[kMaxAlphaSurfaces];
static int s_alphaSurfaceCount = 0;

// One transform per brush model entity that contributed a translucent surface;
// DrawBrushModelEntity's own is a local, long gone by the time the alpha pass
// runs. MAX_ENTITIES is the hard ceiling on contributors, so it cannot overflow.
alignas(16) static math::Mat4 s_alphaEntityMatrices[MAX_ENTITIES];
static int s_alphaEntityMatrixCount = 0;

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
// Translucent surface collection
// ------------------------------------------------------------------------------------------------

// Defers one translucent surface to the RenderAlphaSurfaces pass at the end of
// the frame, remembering the texture (the animation frame is the caller's) and
// the transform it draws under.
void PushAlphaSurface(const mod::ModelSurface & surf, const tex::Texture & texture, const math::Mat4 & mvp)
{
    PS2_AssertMsg(s_alphaSurfaceCount < kMaxAlphaSurfaces, "Out of alpha surface slots!");
    if (s_alphaSurfaceCount == kMaxAlphaSurfaces)
    {
        Com_DPrintf("Out of alpha surface slots!\n");
        return; // Fail and drop the overflow surface on no-asserts build.
    }

    AlphaSurface & entry = s_alphaSurfaces[s_alphaSurfaceCount++];
    entry.surf    = &surf;
    entry.texture = &texture;
    entry.mvp     = &mvp;

    ++s_drawStats.surfacesAlpha;
}

// Parks a brush model entity's transform where the deferred pass can still
// reach it. One slot per contributing entity - callers hold on to the returned
// pointer for the rest of their surfaces. Null only if the table is full.
const math::Mat4 * StoreAlphaEntityMatrix(const math::Mat4 & mvp)
{
    PS2_AssertMsg(s_alphaEntityMatrixCount < MAX_ENTITIES, "Out of alpha entity matrix slots!");
    if (s_alphaEntityMatrixCount == MAX_ENTITIES)
    {
        Com_DPrintf("Out of alpha entity matrix slots!\n");
        return nullptr;
    }

    math::Mat4 & slot = s_alphaEntityMatrices[s_alphaEntityMatrixCount++];
    slot = mvp;
    return &slot;
}

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

// Whether the per-triangle back-face test runs at all. Note the world and
// brush model passes already reject whole surfaces on the same side test
// (their triangles are coplanar with the surface), so enabling this test
// actually doesn't gain us anything. Left as a reference, disabled by default.
inline bool WorldBackFaceCullEnabled()
{
    return s_backFaceCull->value != 0.0f;
}

void SetupFrame(const refdef_t & viewDef)
{
    ++s_frameCount;

    s_eyePosition = { viewDef.vieworg[0], viewDef.vieworg[1], viewDef.vieworg[2], 1.0f };

    // Animated walls flip frames at 2 Hz of game time (as in ref_gl).
    s_frameTime        = viewDef.time;
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

// Returns the texture a surface draws with, following the animation chain for
// animated walls (torches, screens). Never null: the model loader substitutes
// the debug checkerboard for missing wall textures. World surfaces step the
// chain on game time; brush model entities step it on entity.frame instead,
// which is how the game scripts a door or button changing its own texture.
const tex::Texture * TextureAnimation(const mod::ModelTexInfo * texInfo, int animFrame)
{
    PS2_Assert(texInfo != nullptr && texInfo->texture != nullptr);

    if (texInfo->next == nullptr)
    {
        return texInfo->texture; // Not animated.
    }

    int c = animFrame % texInfo->numFrames;
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
            // Translucent or turbulent: deferred to the back-to-front pass at
            // the end of the frame. World geometry draws in world space, so
            // the plain view-projection is transform enough.
            PushAlphaSurface(*surf, *TextureAnimation(surf->texInfo, s_textureAnimFrame), s_viewProjMatrix);
            continue;
        }

        // Opaque: thread onto its texture's draw chain.
        ++s_drawStats.surfaces;
        const tex::Texture * texture = TextureAnimation(surf->texInfo, s_textureAnimFrame);
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

// Everything the gather path needs beyond the geometry itself. The world pass
// draws untransformed, fullbright and opaque; a brush model entity carries its
// own model-view-projection and may blend, so the same clipper and scratch
// buffer serve both.
struct SurfaceDrawState
{
    const math::Mat4 * mvp;   // Clips and draws with this; the world's is the plain view-projection.
    math::Vec4         eye;   // Camera in the same space as the triangles, for the back-face test.
    u32                rgba;  // Packed vertex colour (GS modulate: 128 = unchanged, alpha 0x80 = 1.0).
    vu1::DrawFlags     flags; // Batch flags, i.e. whether the submission blends.
    bool               cullBackFaces;
};

// Sends the gathered scratch triangles as one batch and empties the buffer.
inline void FlushScratch(const tex::Texture & texture, const SurfaceDrawState & state)
{
    if (s_scratchVertCount > 0)
    {
        ++s_drawStats.drawBatches;
        vu1::DrawTriangles(*state.mvp, texture, s_scratchVerts, s_scratchVertCount,
                           state.flags);
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

inline void EmitScratchVertex(const ClipVertex & v, u32 rgba)
{
    vu1::DrawVertex & dst = s_scratchVerts[s_scratchVertCount++];
    dst.x    = v.pos.x;
    dst.y    = v.pos.y;
    dst.z    = v.pos.z;
    dst.w    = 1.0f;
    dst.rgba = rgba;
    dst.s    = v.st.x;
    dst.t    = v.st.y;
    dst.q    = 1.0f;
}

// Fills in a corner's six clip-plane distances from its position. The vertex
// draws untransformed and the VU applies 'mvp', so the judgement has to happen
// in that same clip space.
inline void SetClipDists(ClipVertex & c, const math::Mat4 & mvp)
{
    const math::Vec4 clip = math::Transform(c.pos, mvp);
    const float gw = vu1::kGuardBandNdcLimit * clip.w;
    c.d.f[0] = (clip.w - clip.z) - kClipEpsilon; // near (and behind-camera)
    c.d.f[1] = (clip.w + clip.z) - kClipEpsilon; // far
    c.d.f[2] = (gw - clip.x) - kClipEpsilon;     // guard band sides
    c.d.f[3] = (gw + clip.x) - kClipEpsilon;
    c.d.f[4] = (gw - clip.y) - kClipEpsilon;
    c.d.f[5] = (gw + clip.y) - kClipEpsilon;
    c.d.f[6] = 0.0f; // Spare lanes: never read as planes, but they ride
    c.d.f[7] = 0.0f; // along through the lerps, so keep them finite.
}

// Clips one triangle against the VU clip volume and appends the survivors to
// the scratch buffer, flushing it when full. The corners arrive with their
// position and UVs set; their clip distances are computed here.
void GatherTriangle(ClipVertex (&corners)[3],
                    const tex::Texture & texture,
                    const SurfaceDrawState & state)
{
    // Reject a triangle facing away from the camera before any clipping work.
    // The test is cheaper than the six plane distances below, and a rejected
    // triangle costs the clipper, the scratch buffer and the VU nothing.
    if (state.cullBackFaces &&
        math::CullBackFacingTriangle(state.eye, corners[0].pos, corners[1].pos, corners[2].pos))
    {
        ++s_drawStats.trisBackFacing;
        return;
    }

    int insidePerPlane[kNumClipPlanes] = {};
    for (ClipVertex & c : corners)
    {
        SetClipDists(c, *state.mvp);
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
        return; // All three corners outside one plane: entirely out.
    }

    if (insideTotal == 3 * kNumClipPlanes)
    {
        // Whole triangle inside: the common case, no clipping.
        ++s_drawStats.trisDrawn;
        if (s_scratchVertCount + 3 > kScratchMaxVerts)
        {
            FlushScratch(texture, state);
        }
        for (const ClipVertex & c : corners)
        {
            EmitScratchVertex(c, state.rgba);
        }
        return;
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
        return;
    }

    if (s_scratchVertCount + (count - 2) * 3 > kScratchMaxVerts)
    {
        FlushScratch(texture, state);
    }
    for (int v = 1; v < count - 1; ++v)
    {
        EmitScratchVertex(in[0],     state.rgba);
        EmitScratchVertex(in[v],     state.rgba);
        EmitScratchVertex(in[v + 1], state.rgba);
    }
    s_drawStats.trisDrawn += count - 2;
}

// Appends a polygon's triangles to the scratch buffer, clipping the ones that
// cross the VU clip volume and flushing when full.
void GatherPolyTriangles(const mod::ModelPoly & poly,
                         const tex::Texture & texture,
                         const SurfaceDrawState & state)
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

        ClipVertex corners[3];
        for (int v = 0; v < 3; ++v)
        {
            const mod::PolyVertex & src = poly.vertexes[tri.vertexes[v]];
            corners[v].pos = { src.position.x, src.position.y, src.position.z, 1.0f };
            corners[v].st  = { src.texture_s, src.texture_t, 0.0f, 0.0f };
        }

        GatherTriangle(corners, texture, state);
    }
}

// ------------------------------------------------------------------------------------------------
// Turbulent (warped) surfaces: water, lava, slime
// ------------------------------------------------------------------------------------------------

// ref_gl's r_turbsin lookup (gl_warp.c, values in warpsin.h): 8*sin(i*2pi/256),
// halved once at startup by R_Init, so the effective amplitude is 4 texels.
// Built at init rather than copied in - it is a pure function of the index,
// and this is more accurate than the 6-significant-digit literals ref_gl ships.
constexpr int kTurbSinSize = 256;
constexpr float kTurbSinAmplitude = 4.0f;
static float s_turbSin[kTurbSinSize];

// Phase to table index: the table spans exactly one period (ref_gl's TURBSCALE).
constexpr float kTurbScale = static_cast<float>(kTurbSinSize) / (2.0f * math::kPI);

// A subdivided warp polygon is a fan: a centre vertex, the ring, then a
// duplicate of the first to close it. SubdividePolygon splits at 64-unit
// boundaries, so a leaf never exceeds that many ring vertices.
constexpr int kMaxWarpPolyVerts = 64 + 2;

inline float TurbSin(const float phase)
{
    // Truncation toward zero and a two's complement mask, exactly as ref_gl
    // indexes the table - negative phases included.
    const int index = static_cast<int>(phase * kTurbScale) & (kTurbSinSize - 1);
    return s_turbSin[index];
}

// Gathers a turbulent surface with ref_gl's warp animation (EmitWaterPolys):
// every vertex's texture coordinates are pushed around by a sine of the
// *other* axis plus time, which is what makes the surface ripple while the
// geometry stays put.
//
// These polygons are shaped differently from ordinary ones, so this cannot go
// through GatherPolyTriangles: the loader's SubdivideSurface leaves each as a
// fan with no triangle list at all (reading poly.triangles would dereference
// null), and with texture coordinates still in raw texel units. Both follow
// ref_gl, whose GL_SubdivideSurface builds fans for glBegin(GL_TRIANGLE_FAN)
// and leaves the texel-to-image division to EmitWaterPolys.
//
// TODO: Consider moving the polygon warping work to the VU1.
void DrawAnimatedWaterPolys(const mod::ModelSurface & surf,
                            const tex::Texture & texture,
                            const SurfaceDrawState & state)
{
    // SURF_FLOWING drifts the surface along S by a whole 64-texel tile every
    // two seconds. (ref_gl truncates with an int cast; the time is never
    // negative, so this is the same fractional part.)
    float scroll = 0.0f;
    if (surf.texInfo->flags & SURF_FLOWING)
    {
        const float halfTime = s_frameTime * 0.5f;
        scroll = -64.0f * (halfTime - std::floor(halfTime));
    }

    // ref_gl divides by a hardcoded 64 at this point. Every warp texture in
    // pak0 is 64x64, so these agree on the real data, and this one stays
    // right if some mod ships another size.
    const float invWidth  = 1.0f / static_cast<float>(texture.width);
    const float invHeight = 1.0f / static_cast<float>(texture.height);

    for (const mod::ModelPoly * poly = surf.polys; poly != nullptr; poly = poly->next)
    {
        if (poly->numVerts < 3) // Need at least one triangle.
        {
            continue;
        }
        PS2_AssertMsg(poly->numVerts <= kMaxWarpPolyVerts, "Warp polygon larger than a subdivision leaf!");

        // Warped once per vertex: the fan below reads each of them twice.
        float warpedS[kMaxWarpPolyVerts];
        float warpedT[kMaxWarpPolyVerts];
        for (int i = 0; i < poly->numVerts; ++i)
        {
            const float os = poly->vertexes[i].texture_s;
            const float ot = poly->vertexes[i].texture_t;

            warpedS[i] = (os + TurbSin((ot * 0.125f) + s_frameTime) + scroll) * invWidth;
            warpedT[i] = (ot + TurbSin((os * 0.125f) + s_frameTime)) * invHeight;
        }

        for (int v = 1; v < poly->numVerts - 1; ++v)
        {
            const int fan[3] = { 0, v, v + 1 };

            ClipVertex corners[3];
            for (int c = 0; c < 3; ++c)
            {
                const mod::PolyVertex & src = poly->vertexes[fan[c]];
                corners[c].pos = { src.position.x, src.position.y, src.position.z, 1.0f };
                corners[c].st  = { warpedS[fan[c]], warpedT[fan[c]], 0.0f, 0.0f };
            }

            GatherTriangle(corners, texture, state);
        }
    }
}

// Draws every texture chain built by RecursiveWorldNode and resets them.
void DrawTextureChains()
{
    // World geometry sits in world space already, so its "model" transform is
    // the plain view-projection and the back-face test takes the world camera.
    const SurfaceDrawState state = {
        .mvp   = &s_viewProjMatrix,
        .eye   = s_eyePosition,
        .rgba  = kFullBright,
        .flags = vu1::DrawFlags::None,
        .cullBackFaces = WorldBackFaceCullEnabled()
    };

    for (int i = 0; i < s_chainTextureCount; ++i)
    {
        const tex::Texture * texture = s_chainTextures[i];

        for (const mod::ModelSurface * surf = texture->textureChain; surf != nullptr; surf = surf->textureChain)
        {
            for (const mod::ModelPoly * poly = surf->polys; poly != nullptr; poly = poly->next)
            {
                if (poly->numVerts >= 3) // Need at least one triangle.
                {
                    GatherPolyTriangles(*poly, *texture, state);
                }
            }
        }
        FlushScratch(*texture, state);

        texture->textureChain = nullptr; // Reset for the next frame.
    }
    s_chainTextureCount = 0;
}

// ------------------------------------------------------------------------------------------------
// Light level readback
// ------------------------------------------------------------------------------------------------

// Quake 2's channel for telling the game code how brightly lit the player is:
// the client reads r_lightlevel back out of the cvar system every frame and
// packs it into the usercmd (cl_input.c), where the server uses it to decide
// how visible the player is. Nothing about it is graphics - the renderer just
// happens to be the only thing that can sample the lightmaps.
//
// The value is the largest of the three sampled colour components scaled by
// 150, which is what the software renderer's mono light value worked out to.
// Written straight into cvar_t::value, as ref_gl's R_SetLightLevel does: this
// runs every frame and the cvar's string form is never read.
void SetLightLevel(const refdef_t & viewDef)
{
    if (viewDef.rdflags & RDF_NOWORLDMODEL)
    {
        return; // No world to sample; leave the last value alone (ref_gl does).
    }

    vec3_t shadeLight = { 1.0f, 1.0f, 1.0f };
    vec3_t lightSpot  = {}; // Unused here; the shadow anchor is for entity models.
    CalcPointLightColor(viewDef, viewDef.vieworg, shadeLight, lightSpot);

    float brightest = shadeLight[0];
    if (shadeLight[1] > brightest) { brightest = shadeLight[1]; }
    if (shadeLight[2] > brightest) { brightest = shadeLight[2]; }

    s_lightLevel->value = 150.0f * brightest;
}

// ------------------------------------------------------------------------------------------------
// Translucent surface pass
// ------------------------------------------------------------------------------------------------

// Vertex colour for a deferred surface: ref_gl's R_DrawAlphaSurfaces alphas on
// the GS's 0x80 = 1.0 scale. Surfaces that are turbulent but not explicitly
// translucent (lava, slime) still go through the blend at full opacity, as
// they do there.
inline u32 AlphaSurfaceColor(const int texFlags)
{
    u32 alpha = 0x80; // 1.0
    if (texFlags & SURF_TRANS33)
    {
        alpha = 42; // 0.33
    }
    else if (texFlags & SURF_TRANS66)
    {
        alpha = 84; // 0.66
    }
    return vu1::PackColorRGBA(128, 128, 128, alpha);
}

// Draws everything RecursiveWorldNode and DrawBrushModelEntity set aside -
// glass, water, lava, slime - blended over the finished opaque scene. Called
// last in the frame, where ref_gl calls R_DrawAlphaSurfaces.
//
// Walked in reverse of collection order, which is back to front: the BSP walk
// that collected them ran front to back (ref_gl reaches the same order by
// prepending to a linked list). The entries are depth-ordered rather than
// grouped by texture, so unlike the opaque pass the batch has to break
// whenever the texture, the transform or the alpha changes.
//
// Brush model surfaces were collected after the whole world walk, so reversing
// puts them ahead of even the farthest world surface. That is a real mis-sort
// - and the same one ref_gl has - but it only shows when a translucent
// submodel sits behind translucent world geometry. Sorting the entries by
// view distance would fix it, at the price of the depth order the BSP walk
// hands us for free.
void RenderAlphaSurfaces()
{
    if (s_alphaSurfaceCount == 0 || s_skipAlphaSurfaces->value != 0.0f)
    {
        s_alphaSurfaceCount      = 0;
        s_alphaEntityMatrixCount = 0;
        return;
    }

    SurfaceDrawState state = {
        .mvp   = nullptr, // Per entry, below; no entry ever carries null, so the first always switches.
        .eye   = s_eyePosition,
        .rgba  = 0,
        .flags = vu1::DrawFlags::Blended,
        // Both collectors already dropped the surfaces facing away from the
        // camera - the world walk by plane side, brush models by the same
        // test in model space - so the triangle test has nothing left to find here.
        .cullBackFaces = false
    };

    const tex::Texture * batchTexture = nullptr;

    for (int i = s_alphaSurfaceCount - 1; i >= 0; --i)
    {
        const AlphaSurface & entry = s_alphaSurfaces[i];

        const int texFlags = entry.surf->texInfo->flags;
        const u32 rgba     = AlphaSurfaceColor(texFlags);

        if (entry.texture != batchTexture || entry.mvp != state.mvp || rgba != state.rgba)
        {
            if (batchTexture != nullptr)
            {
                FlushScratch(*batchTexture, state); // Still the outgoing state: flush before switching.
            }
            batchTexture = entry.texture;
            state.mvp    = entry.mvp;
            state.rgba   = rgba;
        }

        if (texFlags & SURF_WARP)
        {
            // Turbulent: its own fan walk, its own animated coordinates.
            DrawAnimatedWaterPolys(*entry.surf, *entry.texture, state);
            continue;
        }

        for (const mod::ModelPoly * poly = entry.surf->polys; poly != nullptr; poly = poly->next)
        {
            if (poly->numVerts >= 3) // Need at least one triangle.
            {
                GatherPolyTriangles(*poly, *entry.texture, state);
            }
        }
    }

    if (batchTexture != nullptr)
    {
        FlushScratch(*batchTexture, state);
    }

    s_alphaSurfaceCount      = 0;
    s_alphaEntityMatrixCount = 0;
}

// ------------------------------------------------------------------------------------------------
// World model pass
// ------------------------------------------------------------------------------------------------

void RenderWorldModel(const refdef_t & viewDef)
{
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
// Brush model entities (inline BSP submodels: doors, plats, trains, buttons)
// ------------------------------------------------------------------------------------------------

// A surface is drawn when the camera is on its front side by more than this,
// in world units (ref_gl's BACKFACE_EPSILON). Surfaces the camera is level
// with are edge-on and contribute nothing.
constexpr float kBackFaceEpsilon = 0.01f;

// An inline submodel's surfaces live in the world model's surface array but
// hang off the submodel's own BSP subtree, so the world walk never reaches
// them - they are drawn here instead, transformed by the entity that carries
// them. There is no PVS or texture chaining involved: the surface count is
// small and the transform is per-entity, so each one is gathered immediately
// and batched only across runs of the same texture.
void DrawBrushModelEntity(const refdef_t & viewDef, const entity_t & entity)
{
    if (s_skipBrushModels->value != 0.0f)
    {
        return; // Debug: skip brush model entities.
    }

    const auto * model = reinterpret_cast<const mod::ModelInstance *>(entity.model);
    PS2_Assert(model != nullptr);

    if (model->numModelSurfaces == 0)
    {
        return; // Submodel with no faces of its own (a pure clip brush).
    }

    // Frustum cull. A rotated submodel has no axis-aligned box that survives
    // the rotation, so it falls back to the model's bounding sphere.
    const bool rotated = (entity.angles[0] != 0.0f ||
                          entity.angles[1] != 0.0f ||
                          entity.angles[2] != 0.0f);
    vec3_t mins, maxs;
    if (rotated)
    {
        for (int i = 0; i < 3; ++i)
        {
            mins[i] = entity.origin[i] - model->radius;
            maxs[i] = entity.origin[i] + model->radius;
        }
    }
    else
    {
        const float * const modelMins = &model->mins.x;
        const float * const modelMaxs = &model->maxs.x;
        for (int i = 0; i < 3; ++i)
        {
            mins[i] = entity.origin[i] + modelMins[i];
            maxs[i] = entity.origin[i] + modelMaxs[i];
        }
    }
    if (ShouldCullBBox(mins, maxs))
    {
        return;
    }

    ++s_drawStats.entities;

    // The per-surface side test runs in model space, so the camera goes there
    // rather than every surface plane coming out - one transform instead of N.
    vec3_t modelOrigin;
    VectorSubtract(viewDef.vieworg, entity.origin, modelOrigin);
    if (rotated)
    {
        vec3_t temp, forward, right, up;
        VectorCopy(modelOrigin, temp);
        AngleVectors(entity.angles, forward, right, up);

        modelOrigin[0] =  DotProduct(temp, forward);
        modelOrigin[1] = -DotProduct(temp, right);
        modelOrigin[2] =  DotProduct(temp, up);
    }

    const math::Mat4 mvp = MakeEntityMatrix(entity, /*flipPitchAngle=*/false) * s_viewProjMatrix;

    // ref_gl draws translucent brush models at a flat quarter alpha rather
    // than the entity's own (glColor4f(1,1,1,0.25) in R_DrawBrushModel).
    const bool translucent = (entity.flags & RF_TRANSLUCENT) != 0;
    const SurfaceDrawState state = {
        .mvp   = &mvp,
        // The surfaces are model-space, so the back-face test takes the same
        // model-space camera the plane-side test above uses.
        .eye   = { modelOrigin[0], modelOrigin[1], modelOrigin[2], 1.0f },
        .rgba  = translucent ? vu1::PackColorRGBA(128, 128, 128, 0x80 / 4) : kFullBright,
        .flags = translucent ? vu1::DrawFlags::Blended : vu1::DrawFlags::None,
        .cullBackFaces = WorldBackFaceCullEnabled()
    };

    // Consecutive surfaces usually share a texture, so the batch only breaks
    // when it actually changes.
    const tex::Texture * batchTexture = nullptr;

    // This entity's transform, parked for the deferred alpha pass on the first
    // translucent surface that needs it (most models have none at all).
    const math::Mat4 * alphaMvp = nullptr;

    mod::ModelSurface * surf = model->surfaces + model->firstModelSurface;
    for (int i = 0; i < model->numModelSurfaces; ++i, ++surf)
    {
        const cplane_t & plane = *surf->plane;
        const float dot = DotProduct(modelOrigin, plane.normal) - plane.dist;

        const bool planeBack = HasFlag(surf->flags, mod::SurfaceFlags::PlaneBack);
        if (!(( planeBack && dot < -kBackFaceEpsilon) ||
              (!planeBack && dot >  kBackFaceEpsilon)))
        {
            continue; // Facing away from the camera.
        }

        if (surf->texInfo->flags & (SURF_TRANS33 | SURF_TRANS66 | SURF_WARP))
        {
            // Deferred to the back-to-front alpha pass, along with this
            // entity's transform: unlike ref_gl's, that pass draws a moving
            // submodel's water/glass where the submodel actually is rather
            // than at the map's rest position. Parked on the first such
            // surface; every later one shares the slot.
            if (alphaMvp == nullptr)
            {
                alphaMvp = StoreAlphaEntityMatrix(mvp);
            }
            if (alphaMvp != nullptr)
            {
                PushAlphaSurface(*surf, *TextureAnimation(surf->texInfo, entity.frame), *alphaMvp);
            }
            continue;
        }

        const tex::Texture * texture = TextureAnimation(surf->texInfo, entity.frame);
        if (texture != batchTexture)
        {
            if (batchTexture != nullptr)
            {
                FlushScratch(*batchTexture, state);
            }
            batchTexture = texture;
        }

        ++s_drawStats.surfaces;
        for (const mod::ModelPoly * poly = surf->polys; poly != nullptr; poly = poly->next)
        {
            if (poly->numVerts >= 3)
            {
                GatherPolyTriangles(*poly, *texture, state);
            }
        }
    }

    if (batchTexture != nullptr)
    {
        FlushScratch(*batchTexture, state);
    }
}

// ------------------------------------------------------------------------------------------------
// Sprite entities (.sp2: explosions, flashes, the power-up glows)
// ------------------------------------------------------------------------------------------------

// A sprite is one camera-facing quad, sized and anchored by its frame: the
// frame's origin_x/origin_y say where in the image the entity's origin sits,
// so the quad hangs off the view's right/up vectors from there. No culling -
// it is two triangles, and the clipper rejects it if it is off screen anyway.
void DrawSpriteEntity(const entity_t & entity)
{
    if (s_skipSprites->value != 0.0f)
    {
        return; // Debug: skip sprite entities.
    }

    const auto * model = reinterpret_cast<const mod::ModelInstance *>(entity.model);
    PS2_Assert(model != nullptr && model->hunkBase != nullptr);

    // The sprite hunk holds the SP2 file image verbatim (see model_load.cpp).
    const auto * sprite = static_cast<const dsprite_t *>(model->hunkBase);
    if (sprite->numframes <= 0)
    {
        return;
    }

    // The engine cycles entity.frame freely and expects the sprite to wrap.
    const int frameNum = (entity.frame >= 0) ? (entity.frame % sprite->numframes) : 0;
    const dsprframe_t & frame = sprite->frames[frameNum];

    const tex::Texture * skin = model->skins[frameNum];
    if (skin == nullptr)
    {
        skin = &tex::DebugTexture(); // Frame's .pcx failed to load.
    }

    ++s_drawStats.entities;

    // Sprite images are rarely power-of-two (48x48, 144x144), and normalized
    // ST spans the power-of-two TEX0 extent, not the image.
    float stScaleS, stScaleT;
    tex::StScaleFor(*skin, &stScaleS, &stScaleT);

    // Clamped because the alpha is whatever the game code put on the entity;
    // on the GS 0x80 is 1.0 and anything above it reads as overbright.
    float alpha = (entity.flags & RF_TRANSLUCENT) ? entity.alpha : 1.0f;
    alpha = (alpha < 0.0f) ? 0.0f : ((alpha > 1.0f) ? 1.0f : alpha);

    const SurfaceDrawState state = {
        .mvp   = &s_viewProjMatrix, // The quad is built in world space already.
        .eye   = s_eyePosition,
        .rgba  = vu1::PackColorRGBA(128, 128, 128, static_cast<u32>(alpha * 128.0f)),
        .flags = (alpha < 1.0f) ? vu1::DrawFlags::Blended : vu1::DrawFlags::None,
        // Never back-face culled: the quad is built from the camera's own
        // right/up vectors, so it cannot face away and the test would be useless.
        .cullBackFaces = false
    };

    // The four corners, in ref_gl's order: bottom-left, top-left, top-right,
    // bottom-right, with T running down the image the way the pixels are
    // stored. 'right' and 'up' are the camera's, which is what makes the quad
    // face it.
    const float leftOffset   = -static_cast<float>(frame.origin_x);
    const float rightOffset  =  static_cast<float>(frame.width - frame.origin_x);
    const float bottomOffset = -static_cast<float>(frame.origin_y);
    const float topOffset    =  static_cast<float>(frame.height - frame.origin_y);

    const struct { float along, up, s, t; } layout[4] = {
        { leftOffset,  bottomOffset, 0.0f, 1.0f },
        { leftOffset,  topOffset,    0.0f, 0.0f },
        { rightOffset, topOffset,    1.0f, 0.0f },
        { rightOffset, bottomOffset, 1.0f, 1.0f },
    };

    ClipVertex quad[4];
    for (int i = 0; i < 4; ++i)
    {
        quad[i].pos = {
            entity.origin[0] + (s_rightVec[0] * layout[i].along) + (s_upVec[0] * layout[i].up),
            entity.origin[1] + (s_rightVec[1] * layout[i].along) + (s_upVec[1] * layout[i].up),
            entity.origin[2] + (s_rightVec[2] * layout[i].along) + (s_upVec[2] * layout[i].up),
            1.0f
        };
        quad[i].st = { layout[i].s * stScaleS, layout[i].t * stScaleT, 0.0f, 0.0f };
    }

    ClipVertex triangle[3] = { quad[0], quad[1], quad[2] };
    GatherTriangle(triangle, *skin, state);

    triangle[0] = quad[0];
    triangle[1] = quad[2];
    triangle[2] = quad[3];
    GatherTriangle(triangle, *skin, state);

    FlushScratch(*skin, state);
}

// ------------------------------------------------------------------------------------------------
// Beam entities (RF_BEAM: the lightning/railgun cylinders)
// ------------------------------------------------------------------------------------------------

// A beam is a cylinder spanning entity.origin -> entity.oldorigin, built as a
// ring of segments around that axis. It has no model and no texture: the
// colour is a palette index in skinnum and the diameter is entity.frame
// (ref_gl's R_DrawBeam).
constexpr int kNumBeamSegs = 6;

void DrawBeamEntity(const entity_t & entity)
{
    vec3_t direction, normalizedDirection;
    VectorSubtract(entity.oldorigin, entity.origin, direction);
    VectorCopy(direction, normalizedDirection);

    if (VectorNormalize(normalizedDirection) == 0.0f)
    {
        return; // Zero length. Also how the client's ex_flash explosions - which
                // borrow RF_BEAM purely as a "don't draw me" marker, comment and
                // all - end up drawing nothing.
    }

    vec3_t perpVec;
    PerpendicularVector(perpVec, normalizedDirection);
    VectorScale(perpVec, static_cast<float>(entity.frame) / 2.0f, perpVec);

    ++s_drawStats.entities;

    const float alpha = (entity.alpha > 0.0f && entity.alpha <= 1.0f) ? entity.alpha : 1.0f;

    const SurfaceDrawState state = {
        .mvp   = &s_viewProjMatrix, // Built in world space.
        .eye   = s_eyePosition,
        .rgba  = (global_palette[entity.skinnum & 0xFF] & 0x00FFFFFF) |
                 (static_cast<u32>(alpha * 128.0f) << 24),
        .flags = vu1::DrawFlags::Blended | vu1::DrawFlags::Untextured,
        // A cylinder does have a far half, but keeping it only makes the blend
        // slightly denser - whereas culling it with the ring's winding guessed
        // wrong would turn the beam inside out.
        .cullBackFaces = false
    };

    math::Vec4 startPoints[kNumBeamSegs];
    math::Vec4 endPoints[kNumBeamSegs];

    for (int i = 0; i < kNumBeamSegs; ++i)
    {
        vec3_t start;
        RotatePointAroundVector(start, normalizedDirection, perpVec,
                                (360.0f / kNumBeamSegs) * static_cast<float>(i));
        VectorAdd(start, entity.origin, start);

        startPoints[i] = { start[0], start[1], start[2], 1.0f };
        endPoints[i]   = { start[0] + direction[0],
                           start[1] + direction[1],
                           start[2] + direction[2], 1.0f };
    }

    // Untextured, but a batch still binds one.
    const tex::Texture & texture = tex::DebugTexture();
    const math::Vec4 zero = { 0.0f, 0.0f, 0.0f, 0.0f };

    // ref_gl walks the ring as one triangle strip of (start[i], end[i],
    // start[i+1], end[i+1]) groups; expanded to the triangle lists the VU
    // path takes, each group is the two triangles closing one wall quad.
    for (int i = 0; i < kNumBeamSegs; ++i)
    {
        const int next = (i + 1) % kNumBeamSegs;

        ClipVertex quad[4];
        quad[0].pos = startPoints[i];
        quad[1].pos = endPoints[i];
        quad[2].pos = startPoints[next];
        quad[3].pos = endPoints[next];
        for (ClipVertex & c : quad)
        {
            c.st = zero;
        }

        ClipVertex triangle[3] = { quad[0], quad[1], quad[2] };
        GatherTriangle(triangle, texture, state);

        triangle[0] = quad[2];
        triangle[1] = quad[1];
        triangle[2] = quad[3];
        GatherTriangle(triangle, texture, state);
    }

    FlushScratch(texture, state);
}

// ------------------------------------------------------------------------------------------------
// Null models (placeholder for an entity whose model is missing)
// ------------------------------------------------------------------------------------------------

// ref_gl's R_DrawNullModel: a small octahedron lit by the world at the
// entity's position, so a model that failed to load is loudly visible instead
// of silently absent.
void DrawNullModelEntity(const refdef_t & viewDef, const entity_t & entity)
{
    vec3_t color = { 1.0f, 1.0f, 1.0f };
    if (!(entity.flags & RF_FULLBRIGHT))
    {
        vec3_t lightSpot = {};
        CalcPointLightColor(viewDef, entity.origin, color, lightSpot);
    }

    ++s_drawStats.entities;

    const auto channel = [](float c) -> u32
    {
        const float scaled = c * 128.0f; // 128 = the GS modulate identity.
        return (scaled >= 255.0f) ? 255u : ((scaled <= 0.0f) ? 0u : static_cast<u32>(scaled));
    };

    // Null models take the brush convention: ref_gl calls R_RotateForEntity
    // directly here, without the pitch flip the alias path wraps it in.
    const math::Mat4 mvp = MakeEntityMatrix(entity, /*flipPitchAngle=*/false) * s_viewProjMatrix;

    const SurfaceDrawState state = {
        .mvp   = &mvp,
        .eye   = s_eyePosition, // Unused; the octahedron is not culled (below).
        .rgba  = vu1::PackColorRGBA(channel(color[0]), channel(color[1]), channel(color[2]), 0x80),
        .flags = vu1::DrawFlags::None,
        // Eight triangles for a debug marker: not worth risking the fans
        // coming out inside-out and hiding the very thing they exist to show.
        .cullBackFaces = false
    };

    constexpr float kRadius = 16.0f;
    constexpr float kApex   = 16.0f;

    // The square ring in the entity's XY plane both fans close over.
    math::Vec4 ring[5];
    for (int i = 0; i <= 4; ++i)
    {
        const float angle = static_cast<float>(i) * math::kHalfPI;
        ring[i] = { kRadius * math::Cosf(angle), kRadius * math::Sinf(angle), 0.0f, 1.0f };
    }

    // The pink checkerboard doubles as the "this model is missing" signal;
    // ref_gl draws the octahedron untextured, in flat shadelight.
    const tex::Texture & texture = tex::DebugTexture();

    for (int half = 0; half < 2; ++half)
    {
        ClipVertex apex;
        apex.pos = { 0.0f, 0.0f, (half == 0) ? -kApex : kApex, 1.0f };
        apex.st  = { 0.5f, 0.5f, 0.0f, 0.0f };

        for (int i = 0; i < 4; ++i)
        {
            // The top half walks the ring backwards, so both cones wind the
            // same way seen from outside.
            const int a = (half == 0) ? i : (4 - i);
            const int b = (half == 0) ? (i + 1) : (3 - i);

            ClipVertex triangle[3];
            triangle[0] = apex;
            triangle[1].pos = ring[a];
            triangle[1].st  = { 0.0f, 1.0f, 0.0f, 0.0f };
            triangle[2].pos = ring[b];
            triangle[2].st  = { 1.0f, 1.0f, 0.0f, 0.0f };

            GatherTriangle(triangle, texture, state);
        }
    }

    FlushScratch(texture, state);
}

// ------------------------------------------------------------------------------------------------
// Particles
// ------------------------------------------------------------------------------------------------

// The client's particle list as camera-facing billboards. Two shapes, picked
// by ps2_hd_particles:
//
//  - The classic Quake 2 particle is a SINGLE triangle whose UVs run
//    0.0625 .. 1.0625 across an 8x8 image with a dot in its top-left corner.
//    The triangle covers exactly the half of the image holding the dot, and
//    the half-texel offset centres it. Three vertices per particle.
//  - The HD particle is a full quad over the soft round sprite: six vertices
//    and twice the fill, but no hard pixel edges when it scales up close.
//
// Both are blended, and DrawFlags::Blended masks depth writes, which is what
// keeps a cloud of them from z-fighting itself (ref_gl's glDepthMask(FALSE)).
//
// TODO: Potential candidate to be moved to the VU1. Could submit a single
// vertex with particle colour and expand to a triangle/quad on the VU.
void RenderParticles(const refdef_t & viewDef)
{
    const int numParticles = viewDef.num_particles;
    if (numParticles <= 0 || s_skipParticles->value != 0.0f)
    {
        return;
    }

    const bool highQuality = (s_hdParticles->value != 0.0f);
    const tex::Texture & texture = tex::ParticleTexture(highQuality);

    // ref_gl's 1.5x blow-up of the camera basis.
    vec3_t up, right;
    VectorScale(s_upVec,    1.5f, up);
    VectorScale(s_rightVec, 1.5f, right);

    SurfaceDrawState state = {
        .mvp   = &s_viewProjMatrix, // Billboards are built in world space.
        .eye   = s_eyePosition,
        .rgba  = 0, // Per particle; filled in below.
        .flags = vu1::DrawFlags::Blended,
        .cullBackFaces = false // Camera-facing, like sprites: nothing to cull.
    };

    for (int i = 0; i < numParticles; ++i)
    {
        const particle_t & p = viewDef.particles[i];

        // ref_gl's "hack a scale up to keep particles from disappearing":
        // past 20 units the billboard grows with distance so it stays wide
        // enough to cover a pixel.
        const float distance = ((p.origin[0] - viewDef.vieworg[0]) * s_forwardVec[0]) +
                               ((p.origin[1] - viewDef.vieworg[1]) * s_forwardVec[1]) +
                               ((p.origin[2] - viewDef.vieworg[2]) * s_forwardVec[2]);
        const float scale = (distance < 20.0f) ? 1.0f : (1.0f + (distance * 0.004f));

        const float alpha = (p.alpha < 0.0f) ? 0.0f : ((p.alpha > 1.0f) ? 1.0f : p.alpha);
        state.rgba = (global_palette[p.color & 0xFF] & 0x00FFFFFF) |
                     (static_cast<u32>(alpha * 128.0f) << 24);

        ++s_drawStats.particles;

        if (highQuality)
        {
            // Corner offsets along (up, right), and the matching UVs.
            static const struct { float u, r, s, t; } kQuad[4] = {
                { 0.0f, 0.0f, 0.0f, 0.0f },
                { 1.0f, 0.0f, 0.0f, 1.0f },
                { 1.0f, 1.0f, 1.0f, 1.0f },
                { 0.0f, 1.0f, 1.0f, 0.0f },
            };

            ClipVertex quad[4];
            for (int c = 0; c < 4; ++c)
            {
                quad[c].pos = {
                    p.origin[0] + (((up[0] * kQuad[c].u) + (right[0] * kQuad[c].r)) * scale),
                    p.origin[1] + (((up[1] * kQuad[c].u) + (right[1] * kQuad[c].r)) * scale),
                    p.origin[2] + (((up[2] * kQuad[c].u) + (right[2] * kQuad[c].r)) * scale),
                    1.0f
                };
                quad[c].st = { kQuad[c].s, kQuad[c].t, 0.0f, 0.0f };
            }

            ClipVertex triangle[3] = { quad[0], quad[1], quad[2] };
            GatherTriangle(triangle, texture, state);

            triangle[0] = quad[2];
            triangle[1] = quad[3];
            triangle[2] = quad[0];
            GatherTriangle(triangle, texture, state);
        }
        else
        {
            constexpr float kUvLo = 0.0625f; // Half a texel into the 8x8 image...
            constexpr float kUvHi = 1.0625f; // ...and one whole image across.

            ClipVertex triangle[3];
            triangle[0].pos = { p.origin[0], p.origin[1], p.origin[2], 1.0f };
            triangle[0].st  = { kUvLo, kUvLo, 0.0f, 0.0f };

            triangle[1].pos = { p.origin[0] + (up[0] * scale),
                                p.origin[1] + (up[1] * scale),
                                p.origin[2] + (up[2] * scale), 1.0f };
            triangle[1].st  = { kUvHi, kUvLo, 0.0f, 0.0f };

            triangle[2].pos = { p.origin[0] + (right[0] * scale),
                                p.origin[1] + (right[1] * scale),
                                p.origin[2] + (right[2] * scale), 1.0f };
            triangle[2].st  = { kUvLo, kUvHi, 0.0f, 0.0f };

            GatherTriangle(triangle, texture, state);
        }
    }

    FlushScratch(texture, state);
}

// ------------------------------------------------------------------------------------------------
// Entity pass
// ------------------------------------------------------------------------------------------------

template<bool isTranslucentPass>
void RenderEntities(const refdef_t & viewDef)
{
    if (s_skipEntities->value != 0.0f)
    {
        return; // Debug: skip all entity models.
    }

    const int numEntities = viewDef.num_entities;
    for (int e = 0; e < numEntities; ++e)
    {
        const entity_t & entity = viewDef.entities[e];

        // Translucents draw after every solid, so they blend over a finished
        // opaque scene rather than whatever happened to be drawn so far.
        const bool translucent = (entity.flags & RF_TRANSLUCENT) != 0;
        if (translucent != isTranslucentPass)
        {
            continue;
        }

        // Debug: Skip drawing the weapon model.
        if ((entity.flags & RF_WEAPONMODEL) && s_skipWeaponModel->value != 0.0f)
        {
            continue;
        }

        // RF_BEAM wins over whatever model the entity carries - the client
        // attaches one to its ex_flash explosions and still expects a beam
        // (a degenerate, invisible one) rather than that model.
        if (entity.flags & RF_BEAM)
        {
            DrawBeamEntity(entity);
            continue;
        }

        // entity_t::model is opaque outside the refresh module, hence the cast.
        const auto * model = reinterpret_cast<const mod::ModelInstance *>(entity.model);
        if (model == nullptr || s_forceNullModels->value != 0.0f)
        {
            DrawNullModelEntity(viewDef, entity);
            continue;
        }

        switch (model->type)
        {
        case mod::ModelType::AliasMD2:
            DrawAliasMD2Entity(viewDef, entity, s_viewProjMatrix);
            break;

        case mod::ModelType::Brush:
            DrawBrushModelEntity(viewDef, entity);
            break;

        case mod::ModelType::Sprite:
            DrawSpriteEntity(entity);
            break;
        }
    }
}

} // namespace

// ------------------------------------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------------------------------------

void InitViewRendering()
{
    s_backFaceCull      = Cvar_Get("ps2_backface_cull",       "0", 0); // NOTE: Off by default. BSP already culls backfacing surfaces.
    s_skipWorld         = Cvar_Get("ps2_skip_world",          "0", 0);
    s_skipAlphaSurfaces = Cvar_Get("ps2_skip_alpha_surfaces", "0", 0); // Debug: drop the translucent glass/water pass.
    s_skipBrushModels   = Cvar_Get("ps2_skip_brushmodels",    "0", 0);
    s_skipSprites       = Cvar_Get("ps2_skip_sprites",        "0", 0);
    s_skipEntities      = Cvar_Get("ps2_skip_entities",       "0", 0);
    s_skipParticles     = Cvar_Get("ps2_skip_particles",      "0", 0);
    s_hdParticles       = Cvar_Get("ps2_hd_particles",        "1", 0); // 1 = soft round quads instead of the classic dot.
    s_forceNullModels   = Cvar_Get("ps2_force_null_models",   "0", 0); // Debug: draw every entity as the octahedron placeholder.
    s_skipWeaponModel   = Cvar_Get("ps2_skip_weapon_model",   "0", 0);

    // Already registered by the client; this just resolves the same object.
    s_lightLevel = Cvar_Get("r_lightlevel", "0", 0);

    // ref_gl's r_turbsin, at full precision (see kTurbSinAmplitude).
    constexpr float kRadiansPerStep = (2.0f * math::kPI) / static_cast<float>(kTurbSinSize);
    for (int i = 0; i < kTurbSinSize; ++i)
    {
        s_turbSin[i] = kTurbSinAmplitude * std::sin(static_cast<float>(i) * kRadiansPerStep);
    }
}

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

math::Mat4 MakeEntityMatrix(const entity_t & entity, const bool flipPitchAngle)
{
    const float pitch = flipPitchAngle ? entity.angles[PITCH] : -entity.angles[PITCH];

    return math::RotationX(math::DegToRad(-entity.angles[ROLL])) *
           math::RotationY(math::DegToRad(pitch))                *
           math::RotationZ(math::DegToRad(entity.angles[YAW]))   *
           math::Translation(entity.origin[0], entity.origin[1], entity.origin[2]);
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

    s_drawStats              = {};
    s_alphaSurfaceCount      = 0;
    s_alphaEntityMatrixCount = 0;

    SetupFrame(viewDef);

    RenderWorldModel(viewDef);
    RenderEntities</*isTranslucentPass=*/false>(viewDef);
    RenderEntities</*isTranslucentPass=*/true>(viewDef);

    // Particles next, as ref_gl does: they are blended and depth-write
    // masked, so they need the opaque scene already laid down behind them.
    RenderParticles(viewDef);

    // Then the translucent world and brush model surfaces the passes above
    // set aside, blended over the whole finished scene.
    RenderAlphaSurfaces();

    // Nothing to draw: hands the light at the camera back to the game code.
    // Where ref_gl's R_RenderFrame calls R_SetLightLevel.
    SetLightLevel(viewDef);

    // TODO: Later milestones continue here: dynamic lights, lightmapped surfaces.
}

} // namespace ps2::view
