/* ================================================================================================
 * File: render_md2.cpp
 * Brief: MD2 "alias" entity model rendering.
 *
 *  An MD2 pose is two keyframes of byte-quantized vertices interpolated by the
 *  entity's backlerp: position = move + vCur * frontv + vOld * backv, where the
 *  three uniform vectors fold together the frames' decode scale/translate, the
 *  lerp factors and the entity's origin delta. The mesh itself arrives as the
 *  file's "glcmds" - tristrips and trifans of (s, t, index) records - which
 *  ExpandGLCmds flattens into the plain triangle lists the VU1 path draws.
 *  The model hunk holds the MD2 file image verbatim (see model_load.cpp), so
 *  frames and glcmds are read straight out of it.
 *
 *  The interpolation itself runs on VU1 (lerped_triangles.vcl): the expansion
 *  streams the two keyframes' dtrivertx_t bytes verbatim and the microprogram
 *  converts and lerps them, with the pose's uniform 'move' translation folded
 *  into the MVP's row 3 so only the two scale vectors ride with each batch.
 *  A scalar EE path (ref_gl's shape: lerp into s_lerpedPositions[], draw
 *  through the plain textured program) is kept behind ps2_md2_vu_lerp=0 as
 *  the A/B debug path, and carries the powersuit-shell models, whose
 *  per-vertex normal extrusion needs an EE-side table lookup.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/common.h"
#include "ps2/renderer/render_md2.h"
#include "ps2/renderer/render_view.h"
#include "ps2/renderer/texture.h"
#include "ps2/renderer/model.h"
#include "ps2/renderer/vu1.h"
#include "ps2/math/vec_mat.h"

namespace ps2::view {
namespace {

// ------------------------------------------------------------------------------------------------
// Vertex lighting tables
// ------------------------------------------------------------------------------------------------

constexpr int kShadeDotQuant    = 16;
constexpr int kNumVertexNormals = 162;

// Pre-calculated dot products of the 162 MD2 vertex normals against the fixed
// shade direction, for 16 quantized model yaws (ref_gl's anormtab; the values
// bake in ref_gl's shading ramp, hence some exceed 1.0 - clamp after scaling),
// and the normals themselves, which the powersuit shell extrudes along.
// Both tables are kept verbatim from id's ref_gl, whose double literals lack
// the .f suffix - hence the -Wfloat-conversion waiver.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-conversion"
static const float s_vertexNormalDots[kShadeDotQuant][256] = {
    #include "client/anormtab.h"
};
static const float s_vertexNormals[kNumVertexNormals][3] = {
    #include "client/anorms.h"
};
#pragma GCC diagnostic pop

// The shade-dot row for the entity's yaw.
inline const float * GetShadeDotsForEntity(const entity_t & entity)
{
    const u32 row = static_cast<u32>(static_cast<int>(
        entity.angles[YAW] * (kShadeDotQuant / 360.0f))) & (kShadeDotQuant - 1);
    return s_vertexNormalDots[row];
}

// ------------------------------------------------------------------------------------------------
// MD2 file data accessors (the model hunk holds the file verbatim)
// ------------------------------------------------------------------------------------------------

inline const dmdl_t * GetAliasHeader(const mod::ModelInstance & model)
{
    PS2_Assert(model.type == mod::ModelType::AliasMD2 && model.hunkBase != nullptr);
    return static_cast<const dmdl_t *>(model.hunkBase);
}

inline const daliasframe_t * GetAliasFrame(const dmdl_t * hdr, const int frameIndex)
{
    const u8 * bytes = static_cast<const u8 *>(static_cast<const void *>(hdr));
    return static_cast<const daliasframe_t *>(
        static_cast<const void *>(bytes + hdr->ofs_frames + (frameIndex * hdr->framesize)));
}

inline const s32 * GetAliasGLCmds(const dmdl_t * hdr)
{
    const u8 * bytes = static_cast<const u8 *>(static_cast<const void *>(hdr));
    return static_cast<const s32 *>(static_cast<const void *>(bytes + hdr->ofs_glcmds));
}

// ------------------------------------------------------------------------------------------------
// Frame state / scratch buffers
// ------------------------------------------------------------------------------------------------

// Triangle gather buffers, flushed when full (referenced in place by DMA).
// The VU lerp path fills the byte-position and attribute streams; the EE
// lerp path fills the plain DrawVertex buffer. One count serves whichever
// path is active. The +1 on the positions is the DrawLerpedTriangles pad
// element for odd flush counts (transferred, never read).
constexpr int kScratchMaxVerts = 3 * 512;
alignas(16) static vu1::DrawVertex      s_scratchVerts[kScratchMaxVerts];
alignas(16) static vu1::LerpVertexBytes s_scratchPosBytes[kScratchMaxVerts + 1];
alignas(16) static vu1::LerpDrawAttrib  s_scratchAttribs[kScratchMaxVerts];
static int s_scratchVertCount = 0;

// Sends the gathered scratch triangles as one batch and empties the buffer.
inline void FlushScratch(const math::Mat4 & mvp, const tex::Texture & texture,
                         const vu1::DrawFlags flags = vu1::DrawFlags::None)
{
    if (s_scratchVertCount > 0)
    {
        ++GetDrawStats().drawBatches;
        vu1::DrawTriangles(mvp, texture, s_scratchVerts, s_scratchVertCount, flags);
        s_scratchVertCount = 0;
    }
}

// The VU-lerp equivalent of FlushScratch, submitting the two SoA streams.
inline void FlushScratchVULerp(const math::Mat4 & mvp, const tex::Texture & texture,
                               const math::Vec3 & frontv, const math::Vec3 & backv,
                               const vu1::FaceCull faceCull, const vu1::DrawFlags flags = vu1::DrawFlags::None)
{
    if (s_scratchVertCount > 0)
    {
        ++GetDrawStats().drawBatches;
        vu1::DrawLerpedTriangles(mvp, texture, frontv, backv,
                                 s_scratchPosBytes, s_scratchAttribs, s_scratchVertCount,
                                 faceCull, flags);
        s_scratchVertCount = 0;
    }
}

// ------------------------------------------------------------------------------------------------
// Entity transform and frustum cull
// ------------------------------------------------------------------------------------------------

// The entity transform is shared with the brush model path;
// alias models are the ones that take +pitch.
inline math::Mat4 MakeAliasMatrix(const entity_t & entity)
{
    return MakeEntityMatrix(entity, /*flipPitchAngle=*/true);
}

// RF_DEPTHHACK: squeeze the entity's whole depth range into the slice
// nearest the camera, so the view weapon can never poke through a wall it
// is visually in front of. ref_gl does it with glDepthRange(0, 0.3); here
// the equivalent is a post-multiply remapping clip z, which the projection
// maps to NDC +1 at the near plane and -1 at the far one:
//     z' = kDepthHackScale * z + (1 - kDepthHackScale) * w
// i.e. NDC z spanning [1 - 2s, 1]. Only rows' z columns change, and the
// scale rides on w, so it composes after everything else.
constexpr float kDepthHackScale = 0.15f;

inline void ApplyDepthHack(math::Mat4 & m)
{
    for (int row = 0; row < 4; ++row)
    {
        m.m[row][2] = (m.m[row][2] * kDepthHackScale) +
                      (m.m[row][3] * (1.0f - kDepthHackScale));
    }
}

// Conservative frustum cull (ref_gl's R_CullAliasModel): the model-space
// bounds of both keyframes, unioned, rotated to world and tested corner-wise.
// MD2 frames carry no explicit bounds - a frame's box is its byte-decode
// transform, translate + scale * [0, 255].
bool ShouldCullEntity(const entity_t & entity, const daliasframe_t * frame, const daliasframe_t * oldFrame)
{
    vec3_t mins, maxs;
    for (int i = 0; i < 3; ++i)
    {
        const float minCur = frame->translate[i];
        const float maxCur = minCur + (frame->scale[i] * 255.0f);
        const float minOld = oldFrame->translate[i];
        const float maxOld = minOld + (oldFrame->scale[i] * 255.0f);

        mins[i] = (minCur < minOld) ? minCur : minOld;
        maxs[i] = (maxCur > maxOld) ? maxCur : maxOld;
    }

    // Rotate the 8 corners into world space - note the same yaw negation and
    // Y flip the engine uses whenever it runs a basis through AngleVectors.
    vec3_t angles, vectors[3];
    VectorCopy(entity.angles, angles);
    angles[YAW] = -angles[YAW];
    AngleVectors(angles, vectors[0], vectors[1], vectors[2]);

    vec3_t corners[8];
    for (int i = 0; i < 8; ++i)
    {
        vec3_t tmp;
        tmp[0] = (i & 1) ? mins[0] : maxs[0];
        tmp[1] = (i & 2) ? mins[1] : maxs[1];
        tmp[2] = (i & 4) ? mins[2] : maxs[2];

        corners[i][0] =  DotProduct(vectors[0], tmp);
        corners[i][1] = -DotProduct(vectors[1], tmp);
        corners[i][2] =  DotProduct(vectors[2], tmp);
        VectorAdd(corners[i], entity.origin, corners[i]);
    }

    return FrustumCullsPoints(corners, ArrayLength(corners));
}

// ------------------------------------------------------------------------------------------------
// Entity shading
// ------------------------------------------------------------------------------------------------

constexpr int kShellFlags = RF_SHELL_RED | RF_SHELL_GREEN | RF_SHELL_BLUE |
                            RF_SHELL_DOUBLE | RF_SHELL_HALF_DAM;

// The uniform shade colour for the entity: shell colours for powersuit
// shells, all-ones for fullbright, the world lighting at the entity origin
// otherwise; then the minlight floor, the bonus-item pulse and the IR goggles
// override on top. Also yields the lightmap trace's ground hit ('outLightSpot',
// where the projected shadow anchors). Based on ref_gl.
math::Vec3 ShadeEntity(const refdef_t & viewDef, const entity_t & entity, vec3_t outLightSpot)
{
    vec3_t color = { 1.0f, 1.0f, 1.0f };

    if (entity.flags & kShellFlags)
    {
        // special case for godmode
        if ((entity.flags & RF_SHELL_RED) && (entity.flags & RF_SHELL_BLUE) && (entity.flags & RF_SHELL_GREEN))
        {
            VectorSet(color, 1.0f, 1.0f, 1.0f);
        }
        else if (entity.flags & (RF_SHELL_RED | RF_SHELL_BLUE | RF_SHELL_DOUBLE))
        {
            VectorClear(color);

            if (entity.flags & RF_SHELL_RED)
            {
                color[0] = 1.0f;
                if (entity.flags & (RF_SHELL_BLUE | RF_SHELL_DOUBLE))
                {
                    color[2] = 1.0f;
                }
            }
            else if (entity.flags & RF_SHELL_BLUE)
            {
                if (entity.flags & RF_SHELL_DOUBLE)
                {
                    color[1] = 1.0f;
                    color[2] = 1.0f;
                }
                else
                {
                    color[2] = 1.0f;
                }
            }
            else if (entity.flags & RF_SHELL_DOUBLE)
            {
                color[0] = 0.9f;
                color[1] = 0.7f;
            }
        }
        else if (entity.flags & (RF_SHELL_HALF_DAM | RF_SHELL_GREEN))
        {
            VectorClear(color);

            if (entity.flags & RF_SHELL_HALF_DAM)
            {
                VectorSet(color, 0.56f, 0.59f, 0.45f);
            }
            if (entity.flags & RF_SHELL_GREEN)
            {
                color[1] = 1.0f;
            }
        }
    }
    else if (entity.flags & RF_FULLBRIGHT)
    {
        VectorSet(color, 1.0f, 1.0f, 1.0f);
    }
    else
    {
        CalcPointLightColor(viewDef, entity.origin, color, outLightSpot);
    }

    if (entity.flags & RF_MINLIGHT)
    {
        int i;
        for (i = 0; i < 3; ++i)
        {
            if (color[i] > 0.1f)
            {
                break;
            }
        }
        if (i == 3)
        {
            VectorSet(color, 0.1f, 0.1f, 0.1f);
        }
    }

    if (entity.flags & RF_GLOW)
    {
        // Bonus items pulse with time.
        const float scale = 0.1f * math::Sinf(viewDef.time * 7.0f);
        for (int i = 0; i < 3; ++i)
        {
            const float min = color[i] * 0.8f;
            color[i] += scale;
            if (color[i] < min)
            {
                color[i] = min;
            }
        }
    }

    // IR goggles color override
    if ((viewDef.rdflags & RDF_IRGOGGLES) && (entity.flags & RF_IR_VISIBLE))
    {
        VectorSet(color, 1.0f, 0.0f, 0.0f);
    }

    return { color[0], color[1], color[2] };
}

const u32 * BuildColorLUT(const entity_t & entity, const math::Vec3 & shadeLight, const float alpha)
{
    // Per-entity packed vertex colours, indexed by the current frame's
    // lightnormalindex: min(shadeDots[n] * shadeLight * 128, 255) per channel
    // (128 = unmodulated texels on the GS; the dots exceed 1.0 by design).
    // Sized 256, not 162: the loader never validates lightnormalindex, so a
    // malformed model must land inside the table rather than past its end.
    static u32 s_colorLUT[256];

    const u32 a = static_cast<u32>(alpha * 128.0f);

    if (entity.flags & kShellFlags)
    {
        // Shells shade flat - no normal-based modulation (ref_gl draws them
        // untextured in pure shell colour; the full effect lands with the
        // powersuit step).
        const auto clamp255 = [](float c) { return (c > 255.0f) ? 255u : static_cast<u32>(c); };
        const u32 packed = vu1::PackColorRGBA(clamp255(shadeLight.x * 128.0f),
                                              clamp255(shadeLight.y * 128.0f),
                                              clamp255(shadeLight.z * 128.0f), a);
        for (u32 & entry : s_colorLUT)
        {
            entry = packed;
        }
        return s_colorLUT;
    }

    const float * const shadeDots = GetShadeDotsForEntity(entity);
    for (int i = 0; i < ArrayLength(s_colorLUT); ++i)
    {
        const float l = shadeDots[i] * 128.0f;
        float r = l * shadeLight.x;
        float g = l * shadeLight.y;
        float b = l * shadeLight.z;
        if (r > 255.0f) { r = 255.0f; }
        if (g > 255.0f) { g = 255.0f; }
        if (b > 255.0f) { b = 255.0f; }
        s_colorLUT[i] = vu1::PackColorRGBA(static_cast<u32>(r),
                                           static_cast<u32>(g),
                                           static_cast<u32>(b), a);
    }

    return s_colorLUT;
}

// ------------------------------------------------------------------------------------------------
// Keyframe interpolation
// ------------------------------------------------------------------------------------------------

// The uniform terms of the pose lerp: position = move + vCur*frontv + vOld*backv
// over the byte-quantized frame vertices.
struct LerpConsts
{
    math::Vec3 move;   // both frames' translate terms + the origin delta, pre-lerped
    math::Vec3 frontv; // current frame decode scale * (1 - backlerp)
    math::Vec3 backv;  // old frame decode scale * backlerp
};

LerpConsts SetUpLerp(const entity_t & entity, const daliasframe_t * frame,
                     const daliasframe_t * oldFrame, const float backlerp)
{
    const float frontlerp = 1.0f - backlerp;

    // The world-space step back to where the previous frame was rendered,
    // expressed in model space (hence the basis projection), so a moving
    // entity's pose interpolates along its own motion.
    vec3_t delta, vectors[3];
    VectorSubtract(entity.oldorigin, entity.origin, delta);
    AngleVectors(entity.angles, vectors[0], vectors[1], vectors[2]);

    vec3_t move;
    move[0] =  DotProduct(delta, vectors[0]); // forward
    move[1] = -DotProduct(delta, vectors[1]); // left
    move[2] =  DotProduct(delta, vectors[2]); // up
    VectorAdd(move, oldFrame->translate, move);

    return {
        .move = {
            (backlerp * move[0]) + (frontlerp * frame->translate[0]),
            (backlerp * move[1]) + (frontlerp * frame->translate[1]),
            (backlerp * move[2]) + (frontlerp * frame->translate[2])
        },
        .frontv = {
            frontlerp * frame->scale[0],
            frontlerp * frame->scale[1],
            frontlerp * frame->scale[2]
        },
        .backv = {
            backlerp * oldFrame->scale[0],
            backlerp * oldFrame->scale[1],
            backlerp * oldFrame->scale[2]
        }
    };
}

// EE-side pose lerp into s_lerpedPositions[]. This is the ps2_md2_vu_lerp=0
// debug path, and the only path for the powersuit-shell models: pushing
// each vertex out along its own normal is a per-vertex table lookup,
// what the VU cannot index cheaply.
const math::Vec3 * LerpVertsEE(const dtrivertx_t * verts, const dtrivertx_t * oldVerts,
                               const int numVerts, const LerpConsts & lc, const bool powersuit)
{
    // Lerped model-space positions of one entity's pose, indexed by the glcmds'
    // vertex index. 24 KB; draws are synchronous, so every entity reuses it in turn.
    static math::Vec3 s_lerpedPositions[MAX_VERTS];

    PS2_Assert(numVerts < ArrayLength(s_lerpedPositions));

    for (int i = 0; i < numVerts; ++i)
    {
        s_lerpedPositions[i] = {
            lc.move.x + (oldVerts[i].v[0] * lc.backv.x) + (verts[i].v[0] * lc.frontv.x),
            lc.move.y + (oldVerts[i].v[1] * lc.backv.y) + (verts[i].v[1] * lc.frontv.y),
            lc.move.z + (oldVerts[i].v[2] * lc.backv.z) + (verts[i].v[2] * lc.frontv.z)
        };

        if (powersuit)
        {
            // Inflate the mesh along its normals so the shell surrounds the
            // model instead of z-fighting it.
            const float * normal = s_vertexNormals[verts[i].lightnormalindex % kNumVertexNormals];
            s_lerpedPositions[i].x += normal[0] * POWERSUIT_SCALE;
            s_lerpedPositions[i].y += normal[1] * POWERSUIT_SCALE;
            s_lerpedPositions[i].z += normal[2] * POWERSUIT_SCALE;
        }
    }

    return s_lerpedPositions;
}

// ------------------------------------------------------------------------------------------------
// glcmds expansion
// ------------------------------------------------------------------------------------------------

// Walks the model's glcmds - tristrips (count > 0) and trifans (count < 0)
// of (float s, float t, s32 index) records, zero-terminated - expanded into
// plain triangles: fans pivot on their first vertex, strips flip every odd
// triangle so the whole strip keeps one facing. Calls emit(s, t, index)
// three times per triangle.
template<typename EmitFn>
void ExpandGLCmds(const dmdl_t * hdr, EmitFn && emit)
{
    const s32 * order = GetAliasGLCmds(hdr);
    for (;;)
    {
        s32 count = *order++;
        if (count == 0)
        {
            break; // End of the command list.
        }

        const bool isFan = (count < 0);
        if (isFan)
        {
            count = -count;
        }
        PS2_Assert(count >= 3);

        const s32 * const verts = order; // 3 words per record
        order += count * 3;

        const auto emitRecord = [&](s32 v)
        {
            const s32 * record = verts + (v * 3);
            const float s   = *static_cast<const float *>(static_cast<const void *>(&record[0]));
            const float t   = *static_cast<const float *>(static_cast<const void *>(&record[1]));
            const s32 index = record[2];
            PS2_Assert(index >= 0 && index < hdr->num_xyz);
            emit(s, t, index);
        };

        if (isFan)
        {
            for (s32 i = 1; i < count - 1; ++i)
            {
                emitRecord(0);
                emitRecord(i);
                emitRecord(i + 1);
            }
        }
        else
        {
            for (s32 i = 0; i < count - 2; ++i)
            {
                if (i & 1) // Odd strip triangles flip to preserve the facing.
                {
                    emitRecord(i + 1);
                    emitRecord(i);
                    emitRecord(i + 2);
                }
                else
                {
                    emitRecord(i);
                    emitRecord(i + 1);
                    emitRecord(i + 2);
                }
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Projected shadow
// ------------------------------------------------------------------------------------------------

// The shadow's attribute stream: flat black at half alpha, identical for
// every vertex of every shadow - filled once, referenced forever. Alpha
// 0x40 = 0.5.
alignas(16) static vu1::LerpDrawAttrib s_shadowAttribs[kScratchMaxVerts];

// Draws the entity's planar projected shadow: the same keyframe byte streams
// the model just drew, run through the same VU1 lerp, with the flattening
// projection premultiplied into the matrix - the reference implementation's
// per-vertex squash (DrawAliasMD2Shadow) is affine, so it rides the MVP for
// free and no EE-side positions are ever needed. 'lightSpot' is where the
// shading lightmap trace hit the ground below the entity.
void DrawAliasMD2Shadow(const entity_t & entity, const dmdl_t * hdr,
                        const daliasframe_t * frame, const daliasframe_t * oldFrame,
                        const LerpConsts & lc, const math::Mat4 & viewProj,
                        const tex::Texture & skin, const vec3_t lightSpot,
                        const vu1::FaceCull faceCull)
{
    // The reference's squash, in matrix form (row-vector convention):
    // x' = x - sv.x * (z + lheight), y' likewise, z' = height - i.e. model
    // z collapses onto the ground plane, sheared along the yaw-unrotated
    // shade vector. Model-space, applied before the entity transform.
    const float lheight = entity.origin[2] - lightSpot[2];
    const float height  = -lheight + 1.0f;
    const float angle   = math::DegToRad(entity.angles[YAW]);
    const math::Vec3 sv = math::Normalize({ math::Cosf(-angle), math::Sinf(-angle), 1.0f });

    const math::Mat4 shadowProj = {{
        { 1.0f,             0.0f,             0.0f,   0.0f },
        { 0.0f,             1.0f,             0.0f,   0.0f },
        { -sv.x,           -sv.y,             0.0f,   0.0f },
        { -sv.x * lheight, -sv.y * lheight,   height, 1.0f } }};

    // Compose first, fold second: the pose's 'move' must translate before
    // the squash, so its fold goes into the full shadow composite.
    math::Mat4 mvp = shadowProj * MakeAliasMatrix(entity) * viewProj;
    const math::Vec4 row3 = math::Transform(math::Vec4{ lc.move.x, lc.move.y, lc.move.z, 1.0f }, mvp);

    mvp.m[3][0] = row3.x;
    mvp.m[3][1] = row3.y;
    mvp.m[3][2] = row3.z;
    mvp.m[3][3] = row3.w;

    const auto flush = [&]()
    {
        if (s_scratchVertCount > 0)
        {
            ++GetDrawStats().drawBatches;
            vu1::DrawLerpedTriangles(mvp, skin, lc.frontv, lc.backv,
                                     s_scratchPosBytes, s_shadowAttribs, s_scratchVertCount,
                                     faceCull, vu1::DrawFlags::Blended | vu1::DrawFlags::Untextured);
            s_scratchVertCount = 0;
        }
    };

    ExpandGLCmds(hdr, [&](float, float, s32 index)
    {
        if (s_scratchVertCount == kScratchMaxVerts)
        {
            flush();
        }
        const int i = s_scratchVertCount++;
        s_scratchPosBytes[i].cur = bits_to_u32(frame->verts[index]);
        s_scratchPosBytes[i].old = bits_to_u32(oldFrame->verts[index]);
    });
    flush();
}

// ------------------------------------------------------------------------------------------------
// Skin selection
// ------------------------------------------------------------------------------------------------

const tex::Texture & SkinForEntity(const entity_t & entity, const mod::ModelInstance & model)
{
    // A custom skin (player models) overrides the model's own list.
    if (entity.skin != nullptr)
    {
        return *reinterpret_cast<const tex::Texture *>(entity.skin);
    }

    const tex::Texture * skin = nullptr;
    if (entity.skinnum >= 0 && entity.skinnum < mod::kMaxMD2Skins)
    {
        skin = model.skins[entity.skinnum];
    }
    if (skin == nullptr)
    {
        skin = model.skins[0];
    }
    if (skin == nullptr)
    {
        return tex::DebugTexture(); // Model has no usable skin at all.
    }
    return *skin;
}

// ------------------------------------------------------------------------------------------------
// Config Cvars
// ------------------------------------------------------------------------------------------------

static const cvar_t * s_lerpModels = nullptr;
static const cvar_t * s_vuLerp     = nullptr;
static const cvar_t * s_cullFace   = nullptr;
static const cvar_t * s_shadows    = nullptr;

} // namespace

// ------------------------------------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------------------------------------

void InitEntityRendering()
{
    s_lerpModels = Cvar_Get("ps2_md2_lerp_on",  "1", 0);
    s_vuLerp     = Cvar_Get("ps2_md2_vu_lerp",  "1", 0);
    s_cullFace   = Cvar_Get("ps2_md2_cullface", "1", 0);
    s_shadows    = Cvar_Get("ps2_md2_shadows",  "1", 0);

    for (vu1::LerpDrawAttrib & attrib : s_shadowAttribs)
    {
        attrib = { vu1::PackColorRGBA(0, 0, 0, 0x40), 0.0f, 0.0f, 1.0f };
    }
}

void DrawAliasMD2Entity(const refdef_t & viewDef, const entity_t & entity, const math::Mat4 & viewProj)
{
    const auto * model = reinterpret_cast<const mod::ModelInstance *>(entity.model);
    PS2_Assert(model != nullptr);

    const dmdl_t * hdr = GetAliasHeader(*model);

    // Bad animation frames render as frame 0 (ref_gl behaviour), not an error.
    int frameIndex    = entity.frame;
    int oldFrameIndex = entity.oldframe;

    if (frameIndex < 0 || frameIndex >= hdr->num_frames)
    {
        Com_DPrintf("DrawAliasMD2Entity %s: no such frame %d\n", model->name, frameIndex);
        frameIndex    = 0;
        oldFrameIndex = 0;
    }
    if (oldFrameIndex < 0 || oldFrameIndex >= hdr->num_frames)
    {
        Com_DPrintf("DrawAliasMD2Entity %s: no such oldframe %d\n", model->name, oldFrameIndex);
        frameIndex    = 0;
        oldFrameIndex = 0;
    }

    const daliasframe_t * frame    = GetAliasFrame(hdr, frameIndex);
    const daliasframe_t * oldFrame = GetAliasFrame(hdr, oldFrameIndex);

    // The view weapon hugs the near plane and never leaves the view; the
    // corner test would false-cull it.
    if ((entity.flags & RF_WEAPONMODEL) == 0)
    {
        if (ShouldCullEntity(entity, frame, oldFrame))
        {
            ++GetDrawStats().boxesCulled;
            return;
        }
    }

    PS2_Assert(hdr->num_xyz > 0 && hdr->num_xyz <= MAX_VERTS);
    ++GetDrawStats().entities;

    // Shade colour and per-normal-index vertex colours. The normal index is
    // read from the *current* frame only - the pose interpolates, the
    // lighting does not (ref_gl behaviour). 'lightSpot' anchors the shadow.
    vec3_t lightSpot = {};
    const math::Vec3 shadeLight = ShadeEntity(viewDef, entity, lightSpot);
    const float alpha = (entity.flags & RF_TRANSLUCENT) ? entity.alpha : 1.0f;
    const u32 * const colorLUT = BuildColorLUT(entity, shadeLight, alpha);

    const float backlerp = (s_lerpModels->value != 0.0f) ? entity.backlerp : 0.0f;
    const LerpConsts lc = SetUpLerp(entity, frame, oldFrame, backlerp);
    const tex::Texture & skin = SkinForEntity(entity, *model);

    // The glcmds' coordinates are normalized against the skin image, but the
    // GS spreads normalized ST over the power-of-two TEX0 extent - and model
    // skins essentially never are one (276x194 is the common size). Without
    // this the skin samples squashed into a corner of a larger virtual image.
    float stScaleS, stScaleT;
    tex::StScaleFor(skin, &stScaleS, &stScaleT);

    math::Mat4 mvp = MakeAliasMatrix(entity) * viewProj;
    if (entity.flags & RF_DEPTHHACK)
    {
        ApplyDepthHack(mvp);
    }

    const bool vuLerp = (s_vuLerp->value != 0.0f) && !(entity.flags & kShellFlags);
    const auto faceCull = static_cast<vu1::FaceCull>(static_cast<u32>(s_cullFace->value) % 3u);

    // Translucent entities blend over the finished opaque scene (the entity
    // pass draws them last) with alpha already folded into the colour LUT.
    const auto blendFlags = (entity.flags & RF_TRANSLUCENT)
                          ? vu1::DrawFlags::Blended
                          : vu1::DrawFlags::None;

    // Expand the glcmds over the pose. Note MD2 triangles are not near-plane
    // clipped like the world's: the VU rejects straddlers whole (guard band),
    // which would only show when the camera is inside a model.
    int emittedVerts = 0;
    if (vuLerp)
    {
        // The pose lerp runs on VU1: fold the uniform 'move' term into the matrix.
        const math::Vec4 row3 = math::Transform(
            math::Vec4{ lc.move.x, lc.move.y, lc.move.z, 1.0f }, mvp);

        mvp.m[3][0] = row3.x;
        mvp.m[3][1] = row3.y;
        mvp.m[3][2] = row3.z;
        mvp.m[3][3] = row3.w;

        ExpandGLCmds(hdr, [&](float s, float t, s32 index)
        {
            if (s_scratchVertCount == kScratchMaxVerts)
            {
                FlushScratchVULerp(mvp, skin, lc.frontv, lc.backv, faceCull, blendFlags);
            }

            const int i = s_scratchVertCount++;
            s_scratchPosBytes[i].cur = bits_to_u32(frame->verts[index]);
            s_scratchPosBytes[i].old = bits_to_u32(oldFrame->verts[index]);
            s_scratchAttribs[i] = {
                .rgba = colorLUT[frame->verts[index].lightnormalindex],
                .s = s * stScaleS,
                .t = t * stScaleT,
                .q = 1.0f
            };
            ++emittedVerts;
        });
        FlushScratchVULerp(mvp, skin, lc.frontv, lc.backv, faceCull, blendFlags);
    }
    else
    {
        // Shells draw as a flat-coloured inflated silhouette: no skin, and
        // always blended, whether or not the client tagged them translucent.
        const bool powersuit = (entity.flags & kShellFlags) != 0;
        const auto flags = powersuit
                         ? (vu1::DrawFlags::Blended | vu1::DrawFlags::Untextured)
                         : blendFlags;

        const math::Vec3 * const lerpedPositions =
            LerpVertsEE(frame->verts, oldFrame->verts, hdr->num_xyz, lc, powersuit);

        ExpandGLCmds(hdr, [&](float s, float t, s32 index)
        {
            if (s_scratchVertCount == kScratchMaxVerts)
            {
                FlushScratch(mvp, skin, flags); // Capacity is a triangle multiple,
            }                                   // so this only fires between them.

            vu1::DrawVertex  & dst = s_scratchVerts[s_scratchVertCount++];
            const math::Vec3 & pos = lerpedPositions[index];
            dst.x    = pos.x;
            dst.y    = pos.y;
            dst.z    = pos.z;
            dst.w    = 1.0f;
            dst.rgba = colorLUT[frame->verts[index].lightnormalindex];
            dst.s    = powersuit ? 0.0f : (s * stScaleS);
            dst.t    = powersuit ? 0.0f : (t * stScaleT);
            dst.q    = 1.0f;
            ++emittedVerts;
        });
        FlushScratch(mvp, skin, flags);
    }

    GetDrawStats().trisDrawn += emittedVerts / 3;

    // The projected blob shadow. Skipped for the view weapon,
    // translucents and shells/fullbright entities.
    if (s_shadows->value != 0.0f &&
        !(entity.flags & (RF_TRANSLUCENT | RF_WEAPONMODEL | RF_FULLBRIGHT | kShellFlags)))
    {
        DrawAliasMD2Shadow(entity, hdr, frame, oldFrame, lc, viewProj, skin, lightSpot, faceCull);
    }
}

} // namespace ps2::view
