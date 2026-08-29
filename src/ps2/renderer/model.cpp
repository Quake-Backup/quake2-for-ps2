/* ================================================================================================
 * File: model.cpp
 * Brief: Quake 2 3D model format caching. Loading itself lives in model_load.cpp;
 *        this file owns the pool of loaded models, the name lookup, and the
 *        level registration lifecycle (mirrors the TextureCache in texture.cpp).
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/hash_map.h"
#include "ps2/renderer/model.h"
#include "ps2/renderer/model_load.h"
#include "ps2/renderer/texture.h"
#include "ps2/small_pool.h"
#include "ps2/hash.h"
#include "ps2/common.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
    #include "common/q_files.h" // IDBSPHEADER / dsprite_t / dmdl_t / MAX_SKINNAME
}

namespace ps2::mod {
namespace {

// Extra debug printing for cache hits / evictions.
constexpr bool kVerboseModelCache = false;

PS2MemTag MemTagForType(ModelType type)
{
    switch (type)
    {
    case ModelType::Brush    : return MEMTAG_MDL_WORLD;
    case ModelType::Sprite   : return MEMTAG_MDL_SPRITE;
    case ModelType::AliasMD2 : return MEMTAG_MDL_ALIAS;
    }
    return MEMTAG_MDL_WORLD; // Unreachable; keeps GCC's -Wreturn-type happy.
}

// Owns the model pool and the name lookup. Internal singleton (s_cache);
// the module API at the bottom of the file is the public face.
class ModelCache final
{
public:
    void Init();

    void BeginRegistration(const char * mapName);
    void EndRegistration();
    bool ReleaseWorldModel(const char * fullName);

    const ModelInstance * Find(const char * name);
    const ModelInstance * WorldModel() { return m_worldModel; }

private:
    const ModelInstance * LoadModel(const char * name);
    void LoadWorldModel(const char * mapName);

    const ModelInstance * FindInlineModel(const char * name);
    void SetUpInlineModels(ModelInstance & world);

    void ReferenceAllTextures(ModelInstance & mdl);
    void Unload(u16 slot);

    // A level references the world plus a few hundred entity/sprite models.
    // The protocol caps model configstrings at MAX_MODELS (256); the slack on
    // top covers view-weapon and per-client player models, which are resolved
    // by name outside that table. Fixed-size pool: running out is a Sys_Error
    // telling you to bump this.
    static constexpr u32 kMaxModels = 320;
    using ModelPool = SmallPool<ModelInstance, kMaxModels>;

    ModelPool m_modelPool;

    // Inline (*N) brush submodels of the current map. They alias the world
    // model's geometry and are set up on each world load, so they live outside
    // the pool and are never looked up by name. Bounds submodels per map, so it
    // mirrors cmodel.c's own cap rather than standing on its own.
    static constexpr u32 kMaxInlineModels = MAX_MAP_MODELS;
    ModelInstance m_inlineModels[kMaxInlineModels] = {};

    // Lookup: FNV-1a hash of the model path -> pool slot.
    HashMap<kMaxModels> m_lookup;

    // Level load/change cycle counter; models stamped with an older value are
    // the ones EndRegistration() frees.
    // Starts at 1, not 0, so a freshly zeroed ModelInstance (regSequence 0) never
    // looks like it was registered this cycle. Init() applies that, rather than a
    // default member initializer here: this cache is a file-level static, and one
    // non-zero word in it is enough to move the whole ~200 KB object out of .bss
    // and into .data, where the zeros cost ELF file size for nothing.
    u32 m_regSequence = 0;

    // Currently loaded world map (a pointer into m_modelPool).
    const ModelInstance * m_worldModel = nullptr;
};

void ModelCache::Init()
{
    m_modelPool.Init(); // One-shot; asserts if called twice.
    m_regSequence = 1;  // See the member declaration for why it starts here.
    Com_Printf("Model cache initialised.\n");
}

const ModelInstance * ModelCache::Find(const char * const name)
{
    PS2_Assert(name != nullptr && *name != '\0');

    // Inline models come from the world's submodels, not the pool.
    if (name[0] == '*')
    {
        return FindInlineModel(name);
    }

    const u16 slot = m_lookup.Find(HashStr64(name));
    if (slot != m_lookup.kInvalidValue)
    {
        ModelInstance & mdl = m_modelPool.Slot(slot);

        // 64-bit FNV-1a collisions are vanishingly rare, but verify the name.
        PS2_AssertMsg(std::strcmp(mdl.name, name) == 0, "Model lookup hash collision!");

        if (kVerboseModelCache)
        {
            Com_DPrintf("Model '%s' already in cache.\n", name);
        }

        mdl.regSequence = m_regSequence; // Still referenced this cycle.
        ReferenceAllTextures(mdl);       // Keep its textures alive too.
        return &mdl;
    }

    return LoadModel(name);
}

// Reads just the 4-byte format id, without loading the file. Brush models are
// streamed lump by lump (LoadBrushModel opens the file itself), so the format has
// to be known before deciding whether to pull the whole thing into memory.
static bool PeekModelType(const char * const name, ModelType & outType)
{
    FILE * file = nullptr;
    if (FS_FOpenFile(name, &file) < static_cast<int>(sizeof(u32)) || file == nullptr)
    {
        if (file != nullptr) { FS_FCloseFile(file); }
        return false;
    }

    u32 id = 0;
    FS_Read(&id, static_cast<int>(sizeof(id)), file);
    FS_FCloseFile(file);

    switch (id)
    {
    case IDBSPHEADER    : outType = ModelType::Brush;    return true;
    case IDSPRITEHEADER : outType = ModelType::Sprite;   return true;
    case IDALIASHEADER  : outType = ModelType::AliasMD2; return true;
    default :
        Com_Printf("ERROR: ModelCache: Unknown file id (0x%X) for '%s'!\n", id, name);
        return false;
    }
}

const ModelInstance * ModelCache::LoadModel(const char * const name)
{
    ModelType type;
    if (!PeekModelType(name, type))
    {
        Com_Printf("WARNING: Unable to load model '%s'! Failed to open file.\n", name);
        return nullptr;
    }

    const u16 slot = m_modelPool.Alloc();
    if (slot == ModelPool::kInvalidIndex) [[unlikely]]
    {
        Sys_Error("Out of model cache slots for '%s'! Bump ModelCache::kMaxModels (%u).", name, kMaxModels);
    }

    ModelInstance & mdl = m_modelPool.Slot(slot);
    std::snprintf(mdl.name, sizeof(mdl.name), "%s", name);
    mdl.type        = type;
    mdl.regSequence = m_regSequence;

    bool ok = false;
    if (type == ModelType::Brush)
    {
        // Streams the file itself - never holds the whole .bsp.
        ok = LoadBrushModel(mdl, name);
        if (ok) { SetUpInlineModels(mdl); }
    }
    else
    {
        // Sprites and MD2s are copied into their hunk verbatim, so the loaded
        // file and the hunk are the same size; loading whole file is fine for those.
        void * fileData = nullptr;
        const int fileLen = FS_LoadFile(name, &fileData);
        if (fileData == nullptr || fileLen <= 0)
        {
            Com_Printf("WARNING: Unable to load model '%s'! Failed to read file.\n", name);
            if (fileData != nullptr) { FS_FreeFile(fileData); }
            Unload(slot);
            return nullptr;
        }

        ok = (type == ModelType::Sprite) ? LoadSpriteModel(mdl, fileData, fileLen)
                                         : LoadAliasMD2Model(mdl, fileData, fileLen);
        FS_FreeFile(fileData);
    }

    if (!ok)
    {
        Unload(slot); // Frees any hunk and returns the slot to the pool.
        return nullptr;
    }

    const bool inserted = m_lookup.Insert(HashStr64(name), slot);
    PS2_AssertMsg(inserted, "Duplicate model name!");

    if (kVerboseModelCache)
    {
        Com_DPrintf("Loaded model '%s'.\n", name);
    }
    return &mdl;
}

const ModelInstance * ModelCache::FindInlineModel(const char * const name)
{
    const int idx = std::atoi(name + 1);
    if (idx < 1 || idx >= static_cast<int>(kMaxInlineModels) ||
        m_worldModel == nullptr || idx >= m_worldModel->numSubModels)
    {
        Com_Printf("ERROR: ModelCache: Bad inline model number (%i) or null world model.\n", idx);
        return nullptr;
    }
    return &m_inlineModels[idx];
}

void ModelCache::SetUpInlineModels(ModelInstance & world)
{
    if (world.numSubModels > static_cast<int>(kMaxInlineModels)) [[unlikely]]
    {
        Sys_Error("Map '%s' has too many submodels (%i)! Bump ModelCache::kMaxInlineModels (%u).",
                  world.name, world.numSubModels, kMaxInlineModels);
    }

    for (int i = 0; i < world.numSubModels; ++i)
    {
        const SubModelInfo & sm = world.subModels[i];
        ModelInstance & inl = m_inlineModels[i];

        // Alias the world's geometry, then override the per-submodel bounds and
        // surface/node range. Inline models never own the hunk (the world does),
        // so clear it to avoid a double free.
        inl = world;
        inl.hunkBase = nullptr;
        inl.hunkSize = 0;
        inl.isInline = true;

        inl.firstModelSurface = sm.firstFace;
        inl.numModelSurfaces  = sm.numFaces;
        inl.firstNode         = sm.headNode;
        inl.mins              = sm.mins;
        inl.maxs              = sm.maxs;
        inl.radius            = sm.radius;

        // Quake 2's on-disk dmodel_t carries no leaf count, and inline models
        // never walk the leaf array; only the world's LoadLeafs count matters.
        inl.numLeafs = 0;

        if (inl.firstNode >= world.numNodes) [[unlikely]]
        {
            Sys_Error("Inline model %i of '%s' has a bad first node!", i, world.name);
        }

        // Submodel 0 is the world itself; fold its ranges back into the world.
        // numLeafs stays untouched: the world keeps the full count from
        // LoadLeafs, or MarkLeaves would have no leafs to stamp visible.
        if (i == 0)
        {
            world.firstModelSurface = sm.firstFace;
            world.numModelSurfaces  = sm.numFaces;
            world.firstNode         = sm.headNode;
            world.mins              = sm.mins;
            world.maxs              = sm.maxs;
            world.radius            = sm.radius;
        }
    }
}

void ModelCache::ReferenceAllTextures(ModelInstance & mdl)
{
    switch (mdl.type)
    {
    case ModelType::Brush:
        // Re-stamp the wall textures so EndRegistration keeps them (no reload).
        for (int i = 0; i < mdl.numTexInfos; ++i)
        {
            if (mdl.texInfos[i].texture != nullptr)
            {
                tex::TouchTexture(*mdl.texInfos[i].texture);
            }
        }
        break;

    case ModelType::Sprite:
        {
            const auto * sprite = static_cast<const dsprite_t *>(mdl.hunkBase);
            for (int i = 0; i < sprite->numframes; ++i)
            {
                mdl.skins[i] = tex::Find(sprite->frames[i].name, tex::ImageType::Sprite);
            }
            break;
        }

    case ModelType::AliasMD2:
        {
            const auto * md2 = static_cast<const dmdl_t *>(mdl.hunkBase);
            for (int i = 0; i < md2->num_skins; ++i)
            {
                const char * skinName = reinterpret_cast<const char *>(md2) + md2->ofs_skins + (i * MAX_SKINNAME);
                mdl.skins[i] = tex::Find(skinName, tex::ImageType::Skin);
            }
            mdl.numFrames = md2->num_frames;
            break;
        }
    }
}

void ModelCache::Unload(u16 slot)
{
    ModelInstance & mdl = m_modelPool.Slot(slot);
    if (mdl.hunkBase != nullptr)
    {
        PS2_MemFree(mdl.hunkBase, mdl.hunkSize, MemTagForType(mdl.type));
    }
    m_modelPool.Free(slot); // Zeroes the slot.
}

void ModelCache::BeginRegistration(const char * const mapName)
{
    PS2_Assert(mapName != nullptr && *mapName != '\0');

    // Bump first, so everything found or loaded this cycle is stamped current
    // and survives EndRegistration().
    ++m_regSequence;
    LoadWorldModel(mapName);
}

// Frees the resident world, unless it is already 'fullName' - a null or empty
// name releases whatever is loaded. Splitting this out of LoadWorldModel lets the
// server call it before it builds the next map's collision model: the old world
// hunk is the single largest allocation in the game (7.1 MB on 'power2' map) and
// it was otherwise held for the whole of server init, which is exactly when the next
// map's BSP needs the room.
//
// Returns whether it actually freed anything. Callers need that: the lightmap
// atlases have the same lifetime as the world model and are reachable only
// through ModelSurface::lightmapTextureNum, so they may be released exactly when
// the surfaces indexing them are - never on the keep-it path below.
bool ModelCache::ReleaseWorldModel(const char * const fullName)
{
    if (m_worldModel == nullptr)
    {
        return false;
    }
    if (fullName != nullptr && std::strcmp(m_worldModel->name, fullName) == 0)
    {
        // Same map (a restart or savegame load). Keeping it means Find() will hit
        // the cache in LoadWorldModel, which means no reload - and so no
        // lm::BeginBuildingLightmaps() to rebuild atlases either.
        return false;
    }

    if (kVerboseModelCache)
    {
        Com_DPrintf("Unloading current map '%s'...\n", m_worldModel->name);
    }

    const u64 key  = HashStr64(m_worldModel->name);
    const u16 slot = m_lookup.Find(key);
    if (slot != m_lookup.kInvalidValue)
    {
        Unload(slot);
        m_lookup.Remove(key);
    }
    m_worldModel = nullptr;
    return true;
}

void ModelCache::LoadWorldModel(const char * const mapName)
{
    char fullName[MAX_QPATH];
    std::snprintf(fullName, sizeof(fullName), "maps/%s.bsp", mapName);

    // Free the old map up front if we are switching to a different one. This
    // guarantees the world's inline models are rebuilt against fresh geometry.
    // Normally a no-op by now: the server released it before it loaded the new
    // map's collision data (see ps2::ReleaseWorldModel).
    ReleaseWorldModel(fullName);

    const ModelInstance * const world = Find(fullName);
    if (world == nullptr) [[unlikely]]
    {
        Sys_Error("ModelCache: Unable to load level map '%s'!", fullName);
    }
    m_worldModel = world;
}

void ModelCache::EndRegistration()
{
    // Free the models this cycle no longer references.
    const int freedCount = static_cast<int>(m_lookup.RemoveIf([this](u64, u16 slot) {
        ModelInstance & mdl = m_modelPool.Slot(slot);
        if (mdl.regSequence == m_regSequence)
        {
            return false;
        }

        if (kVerboseModelCache)
        {
            Com_DPrintf("Freeing unused model '%s'\n", mdl.name);
        }

        if (&mdl == m_worldModel)
        {
            m_worldModel = nullptr;
        }

        Unload(slot);
        return true;
    }));

    if (freedCount > 0)
    {
        Com_DPrintf("Model cache: freed %d unused models.\n", freedCount);
    }
}

static ModelCache s_cache;

} // namespace

// ------------------------------------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------------------------------------

void Init()
{
    s_cache.Init();
}

void BeginRegistration(const char * mapName)
{
    s_cache.BeginRegistration(mapName);
}

void EndRegistration()
{
    s_cache.EndRegistration();
}

bool ReleaseWorldModel(const char * fullName)
{
    return s_cache.ReleaseWorldModel(fullName);
}

const ModelInstance * Find(const char * name)
{
    return s_cache.Find(name);
}

const ModelInstance * GetWorldModel()
{
    return s_cache.WorldModel();
}

} // namespace ps2::mod
