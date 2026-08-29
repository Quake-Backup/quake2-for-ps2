/* ================================================================================================
 * File: map_cycle.cpp
 * Brief: Map cycling memory smoke test. See map_cycle.h.
 *
 *  Drives the real console command ("map <name>") through the command buffer rather than
 *  calling into the server directly, so the sequence the test exercises is byte for byte
 *  the one a player produces. Cbuf_AddText also defers execution out of the middle of the
 *  frame we are in, which matters: SV_SpawnServer frees the resident world model, and the
 *  renderer is on the stack right now.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#if PS2_QUAKE_DEBUG
#include "ps2/common.h"
#include "ps2/tests/map_cycle.h"
#include "ps2/renderer/model.h"

#include <cstdio>
#include <cstring>

namespace ps2::test {
namespace {

// Every map in pak0, in single-player unit order (verified against the pak: 39
// maps, no gaps, no duplicates). Order matters - the test is about the cost of
// each transition, and this is the sequence a real playthrough produces.
constexpr const char * kMaps[] = {
    // Unit 1 - outer base
    "base1", "base2", "base3", "train",
    // Unit 2 - installation
    "bunk1", "ware1", "ware2",
    // Unit 3 - jail
    "jail1", "jail2", "jail3", "jail4", "jail5", "security",
    // Unit 4 - mine
    "mintro", "mine1", "mine2", "mine3", "mine4",
    // Unit 5 - factory
    "fact1", "fact3", "fact2",
    // Unit 6 - power plant
    "power1", "power2", "cool1",
    // Unit 7 - biggun
    "waste1", "waste2", "waste3", "biggun",
    // Unit 8 - hangar
    "hangar1", "hangar2", "space",
    // Unit 9 - research lab
    "lab", "boss1",
    // Unit 10 - city
    "city1", "city2", "city3", "strike",
    // Unit 11 - final
    "command", "boss2",
};

enum class State
{
    Idle,    // Nothing issued yet; kick off the next map.
    Loading, // Command issued, waiting for the world to come up.
    Dwelling // Map is up; stay in it so it actually renders.
};

// A map that never comes up is a failed test, not a reason to hang forever. The
// slowest stock load measured is ~8 s (power2 from host:), so this is generous.
constexpr int kLoadTimeoutMs = 90 * 1000;

// The world has to be resident for this many frames before we call it loaded.
// Guards the window between Cbuf_AddText and Cbuf_Execute, where the *previous*
// map is still up and would otherwise match if it happened to be the target.
constexpr int kFramesToConfirm = 2;

static State  s_state         = State::Idle;
static int    s_nextMap       = 0;
static bool   s_done          = false;
static int    s_issuedAtMs    = 0;
static int    s_dwellUntilMs  = 0;
static int    s_confirmFrames = 0;
static int    s_skipped       = 0;
static int    s_failed        = 0;
static size_t s_peakBeforeMap = 0;
static char   s_targetBsp[MAX_QPATH] = {};

bool MapFileExists(const char * const bspName)
{
    FILE * file = nullptr;
    if (FS_FOpenFile(bspName, &file) < 0 || file == nullptr)
    {
        if (file != nullptr) { FS_FCloseFile(file); }
        return false;
    }
    FS_FCloseFile(file);
    return true;
}

bool TargetWorldIsResident()
{
    const mod::ModelInstance * const world = mod::GetWorldModel();
    return world != nullptr && std::strcmp(world->name, s_targetBsp) == 0;
}

size_t TagBytes(const PS2MemTag tag)
{
    return PS2_GetStatsForMemTag(tag)->totalBytes;
}

size_t LiveTotalBytes()
{
    size_t total = 0;
    for (int i = 0; i < MEMTAG_COUNT; ++i)
    {
        total += TagBytes(static_cast<PS2MemTag>(i));
    }
    return total;
}

// One line per map, with the tags that actually move between levels. The peak is
// the global high-water (PS2_GetPeakMemBytes), so "NEW PEAK" marks the transition
// that cost the most - which is the number the whole test exists to find.
void ReportMap(const char * const name, const int index)
{
    char world[PS2_MEMUNIT_STR_SIZE], audio[PS2_MEMUNIT_STR_SIZE];
    char tex[PS2_MEMUNIT_STR_SIZE],   alias[PS2_MEMUNIT_STR_SIZE];
    char total[PS2_MEMUNIT_STR_SIZE], peak[PS2_MEMUNIT_STR_SIZE];
    char freeMem[PS2_MEMUNIT_STR_SIZE];

    const size_t peakNow = PS2_GetPeakMemBytes();

    Com_Printf("MapCycle [%2d/%2d] %-9s World %-9s Audio %-9s Tex %-9s Mdl %-9s "
               "TOTAL %-9s PEAK %-9s FREE %-9s%s\n",
               index + 1, ArrayLength(kMaps), name,
               PS2_FormatMemoryUnit(TagBytes(MEMTAG_MDL_WORLD), true, world, sizeof(world)),
               PS2_FormatMemoryUnit(TagBytes(MEMTAG_AUDIO),     true, audio, sizeof(audio)),
               PS2_FormatMemoryUnit(TagBytes(MEMTAG_TEXIMAGE),  true, tex,   sizeof(tex)),
               PS2_FormatMemoryUnit(TagBytes(MEMTAG_MDL_ALIAS), true, alias, sizeof(alias)),
               PS2_FormatMemoryUnit(LiveTotalBytes(),           true, total, sizeof(total)),
               PS2_FormatMemoryUnit(peakNow,                    true, peak,  sizeof(peak)),
               PS2_FormatMemoryUnit(PS2_GetAvailableMemBytes(), true, freeMem, sizeof(freeMem)),
               (peakNow > s_peakBeforeMap) ? "  <- NEW PEAK" : "");
}

void Finish()
{
    char peak[PS2_MEMUNIT_STR_SIZE], total[PS2_MEMUNIT_STR_SIZE];

    Com_Printf("MapCycle: pass complete - %d loaded, %d skipped (not in pak), %d timed out.\n",
               ArrayLength(kMaps) - s_skipped - s_failed, s_skipped, s_failed);
    Com_Printf("MapCycle: worst moment across the whole run was %s of %s installed.\n",
               PS2_FormatMemoryUnit(PS2_GetPeakMemBytes(), true, peak, sizeof(peak)),
               PS2_FormatMemoryUnit(PS2_GetTotalMemBytes(), true, total, sizeof(total)));
    s_done = true;
}

// Issues the next map, skipping any that are not in the pak. Returns false when
// the list is exhausted.
bool StartNextMap()
{
    while (s_nextMap < ArrayLength(kMaps))
    {
        const char * const name = kMaps[s_nextMap];
        std::snprintf(s_targetBsp, sizeof(s_targetBsp), "maps/%s.bsp", name);

        if (!MapFileExists(s_targetBsp))
        {
            Com_Printf("MapCycle: skipping '%s' (not in the pak).\n", name);
            ++s_skipped;
            ++s_nextMap;
            continue;
        }

        // Sampled before the load so ReportMap can tell whether *this* transition
        // set a new high-water, rather than just echoing the running maximum.
        s_peakBeforeMap = PS2_GetPeakMemBytes();

        Cbuf_AddText(va("map %s\n", name));

        s_issuedAtMs    = Sys_Milliseconds();
        s_confirmFrames = 0;
        s_state         = State::Loading;
        return true;
    }
    return false;
}

} // namespace

void RunMapCycle()
{
    static const cvar_t * s_enabled = Cvar_Get("ps2_testmaps",       "0", 0);
    static const cvar_t * s_dwell   = Cvar_Get("ps2_testmaps_dwell", "8", 0);

    if (s_enabled->value == 0.0f || s_done)
    {
        return;
    }

    switch (s_state)
    {
    case State::Idle:
        if (!StartNextMap())
        {
            Finish();
        }
        break;

    case State::Loading:
        if (TargetWorldIsResident() && ++s_confirmFrames >= kFramesToConfirm)
        {
            const int dwellMs = static_cast<int>(s_dwell->value * 1000.0f);
            s_dwellUntilMs = Sys_Milliseconds() + ((dwellMs > 0) ? dwellMs : 1);
            s_state = State::Dwelling;
            break;
        }
        if ((Sys_Milliseconds() - s_issuedAtMs) > kLoadTimeoutMs)
        {
            Com_Printf("MapCycle: '%s' never came up after %d seconds - moving on.\n",
                       kMaps[s_nextMap], kLoadTimeoutMs / 1000);
            ++s_failed;
            ++s_nextMap;
            s_state = State::Idle;
        }
        break;

    case State::Dwelling:
        if (Sys_Milliseconds() >= s_dwellUntilMs)
        {
            ReportMap(kMaps[s_nextMap], s_nextMap);
            ++s_nextMap;
            s_state = State::Idle;
        }
        break;
    }
}

} // namespace ps2::test
#endif // PS2_QUAKE_DEBUG
