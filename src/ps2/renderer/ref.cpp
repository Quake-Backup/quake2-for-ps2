/* ================================================================================================
 * File: ref.cpp
 * Brief: The refexport_t implementation - the functions the Quake II client calls
 *        to draw. Implements the full 2D overlay path (console, HUD, menus) -
 *        pics, glyphs, tile fills, solid fills and fades - plus cinematic
 *        playback (cinematic.cpp) and the image/model registration cycle
 *        (assets load from disk on first use and are freed when a level stops
 *        referencing them). RenderFrame draws the 3D world geometry (render_view.cpp)
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/common.h"
#include "ps2/renderer/gs.h"
#include "ps2/renderer/vram.h"
#include "ps2/renderer/vu1.h"
#include "ps2/renderer/model.h"
#include "ps2/renderer/texture.h"
#include "ps2/renderer/cinematic.h"
#include "ps2/renderer/render_view.h"
#include "ps2/renderer/tests/draw_cube.h"
#include "ps2/renderer/tests/cinematics.h"
#include "ps2/builtin/builtin.h"

#include <cstdio>

namespace {

// Size in pixels of one console font glyph (conchars is a 16x16 grid of these).
constexpr int kGlyphSize = 8;

// Vertex colour applied to textured 2D (GS modulate: 128 = texels unchanged).
constexpr u8 kUiBrightness = 128;

static const cvar_t * s_showFpsCount = nullptr;
static const cvar_t * s_showMemStats = nullptr;
static const cvar_t * s_showVramStats = nullptr;

// Built-ins used every frame, cached at init to skip the name lookup.
static const ps2::tex::Texture * s_texConchars = nullptr;
static const ps2::tex::Texture * s_texBacktile = nullptr;

// A missing image draws as the pink/black checkerboard instead of crashing or
// silently vanishing - obvious on screen, and callers get sane dimensions.
const ps2::tex::Texture & FindTextureOrPlaceholder(const char * name, const ps2::tex::ImageType type)
{
    const ps2::tex::Texture * texture = ps2::tex::Find(name, type);
    if (texture == nullptr)
    {
        Com_DPrintf("Missing texture '%s', using placeholder.\n", name);
        texture = &ps2::tex::DebugTexture();
    }
    return *texture;
}

void DrawGlyph(int x, int y, int c)
{
    // Draws one 8x8 graphics character with 0 being transparent.
    // It can be clipped to the top of the screen to allow the console
    // to be smoothly scrolled off. Based on Draw_Char() from ref_gl.

    c &= 255;

    if ((c & 127) == ' ')
    {
        return; // Whitespace
    }
    if (y <= -kGlyphSize)
    {
        return; // Totally off screen
    }

    const int row = (c >> 4) * kGlyphSize;
    const int col = (c & 15) * kGlyphSize;

    ps2::gs::SetTextureFor2D(*s_texConchars);
    ps2::gs::DrawTexturedRect(x, y, kGlyphSize, kGlyphSize,
                              col, row, col + kGlyphSize, row + kGlyphSize, kUiBrightness);
}

void DrawInternalString(int x, int y, const char * str)
{
    const int initialX = x;
    for (; *str != '\0'; ++str)
    {
        DrawGlyph(x, y, *str);
        x += kGlyphSize;
        if (*str == '\n')
        {
            y += kGlyphSize + 2; // 2 pixels of spacing between lines.
            x = initialX;
        }
    }
}

// Frames-per-second counter at the top-right corner of the screen.
// Averages a few frames to smooth changes out a bit.
void DrawFpsCounter()
{
    if (s_showFpsCount->value == 0.0f)
    {
        return;
    }

    constexpr int kMaxFpsHist = 4;
    static struct
    {
        int index;
        int count;
        int previousTime;
        int timesHist[kMaxFpsHist];
    } s_fps;

    const int timeMillisec = Sys_Milliseconds(); // Real time clock
    const int frameTime = timeMillisec - s_fps.previousTime;

    s_fps.timesHist[s_fps.index++] = frameTime;
    s_fps.previousTime = timeMillisec;

    if (s_fps.index == kMaxFpsHist)
    {
        int total = 0;
        for (int i = 0; i < kMaxFpsHist; ++i)
        {
            total += s_fps.timesHist[i];
        }
        if (total == 0)
        {
            total = 1;
        }
        s_fps.count = ((1000 * kMaxFpsHist) + (total / 2)) / total;
        s_fps.index = 0;
    }

    char text[32];
    std::snprintf(text, sizeof(text), "FPS %d", s_fps.count);

    // A black background to give the text more contrast.
    ps2::gs::FillRect(viddef.width - 68, 2, 64, 12, 0, 0, 0, 255);
    DrawInternalString(viddef.width - 64, 4, text);
}

// Memory usage overlay in the lower-right corner: one line per PS2MemTag with
// its running byte total, followed by the grand total across all tags.
void DrawMemUsageOverlay()
{
    if (s_showMemStats->value == 0.0f)
    {
        return;
    }

    constexpr int kLineHeight = kGlyphSize + 2;   // Matches DrawInternalString spacing.
    constexpr int kNumLines   = MEMTAG_COUNT + 2; // Header + one per tag + total.
    constexpr int kPanelWidth = 176;
    constexpr int kPadding    = 4;

    const int panelHeight = (kNumLines * kLineHeight) + (kPadding * 2);
    const int panelX = viddef.width  - kPanelWidth;
    const int panelY = viddef.height - panelHeight;

    // A black background to give the text more contrast.
    ps2::gs::FillRect(panelX, panelY, kPanelWidth, panelHeight, 0, 0, 0, 255);

    const int textX = panelX + kPadding;
    int textY = panelY + kPadding;

    DrawInternalString(textX, textY, "MEM USAGE");
    textY += kLineHeight;

    char line[64];
    size_t totalBytes = 0;
    for (int i = 0; i < MEMTAG_COUNT; ++i)
    {
        const PS2MemTag tag = static_cast<PS2MemTag>(i);
        const size_t tagBytes = PS2_GetStatsForMemTag(tag)->totalBytes;
        totalBytes += tagBytes;

        // PS2_FormatMemoryUnit returns a shared static buffer, so it must be
        // consumed by this one snprintf before the next iteration reuses it.
        std::snprintf(line, sizeof(line), "%-10s %s",
                      PS2_GetNameForMemTag(tag),
                      PS2_FormatMemoryUnit(tagBytes, true));

        DrawInternalString(textX, textY, line);
        textY += kLineHeight;
    }

    std::snprintf(line, sizeof(line), "%-10s %s", "Total", PS2_FormatMemoryUnit(totalBytes, true));
    DrawInternalString(textX, textY, line);
}

// GS VRAM texture-heap overlay in the lower-left corner: free/uncommitted heap
// space, the number of resident textures and the texture uploads done so far
// this frame (streaming pressure - high or spiking means the heap is thrashing).
void DrawVramUsageOverlay()
{
    if (s_showVramStats->value == 0.0f)
    {
        return;
    }

    constexpr int kLineHeight = kGlyphSize + 2; // Matches DrawInternalString spacing.
    constexpr int kNumLines   = 4;              // Header + three stats.
    constexpr int kPanelWidth = 174;
    constexpr int kPadding    = 4;

    const ps2::vram::Stats stats = ps2::vram::GetStats();

    const int panelHeight = (kNumLines * kLineHeight) + (kPadding * 2);
    const int panelX = 0;                            // flush to the left edge
    const int panelY = viddef.height - panelHeight;  // ...and the bottom

    // A black background to give the text more contrast.
    ps2::gs::FillRect(panelX, panelY, kPanelWidth, panelHeight, 0, 0, 0, 255);

    const int textX = panelX + kPadding;
    int textY = panelY + kPadding;

    DrawInternalString(textX, textY, "VRAM USAGE");
    textY += kLineHeight;

    char line[64];

    // PS2_FormatMemoryUnit returns a shared static buffer, so it must be consumed
    // by this one snprintf before anything else reuses it.
    std::snprintf(line, sizeof(line), "%-10s %s", "Free",
                  PS2_FormatMemoryUnit(static_cast<size_t>(stats.freeWords) * 4u, true));
    DrawInternalString(textX, textY, line);
    textY += kLineHeight;

    std::snprintf(line, sizeof(line), "%-10s %d", "Resident", stats.residentTextures);
    DrawInternalString(textX, textY, line);
    textY += kLineHeight;

    std::snprintf(line, sizeof(line), "%-10s %d", "Uploads", stats.uploadsThisFrame);
    DrawInternalString(textX, textY, line);
}

} // namespace

extern "C" {

// ------------------------------------------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------------------------------------------

qboolean PS2_RefInit(void * hinstance, void * wndproc)
{
    (void)hinstance;
    (void)wndproc;

    ps2::gs::Init();
    ps2::tex::Init();
    ps2::vu1::Init();
    ps2::mod::Init();

    s_showFpsCount = Cvar_Get("ps2_show_fps", "1", 0);
    s_showMemStats = Cvar_Get("ps2_show_memstats", "1", 0);
    s_showVramStats = Cvar_Get("ps2_show_vramstats", "1", 0);

    s_texConchars = ps2::tex::Find("conchars", ps2::tex::ImageType::Pic);
    s_texBacktile = ps2::tex::Find("backtile", ps2::tex::ImageType::Pic);
    PS2_Assert(s_texConchars != nullptr && s_texBacktile != nullptr);

    // Seed the cinematic palette from the game palette (the engine normally
    // sets a real one before the first frame; this covers stray draws).
    ps2::cin::SetPalette(nullptr);

    viddef.width  = ps2::gs::Width();
    viddef.height = ps2::gs::Height();

    Com_Printf("PS2 refresh initialised: %dx%d\n", viddef.width, viddef.height);
    return true;
}

void PS2_RefShutdown() {}
void PS2_AppActivate(qboolean activate) { (void)activate; }

// ------------------------------------------------------------------------------------------------
// Registration: textures load from disk on first use (PCX/WAL/TGA); the
// Begin/End pair brackets a level change and frees the level assets it no
// longer references. Same for models.
// ------------------------------------------------------------------------------------------------

void PS2_BeginRegistration(const char * mapName)
{
    ps2::tex::BeginRegistration();
    ps2::mod::BeginRegistration(mapName);
    ps2::view::BeginRegistration();
}

void PS2_EndRegistration()
{
    ps2::mod::EndRegistration();
    ps2::tex::EndRegistration();
}

void PS2_SetSky(const char * name, float rotate, vec3_t axis)
{
    (void)name; (void)rotate; (void)axis;
    // TODO
}

struct model_s * PS2_RegisterModel(const char * name)
{
    return const_cast<struct model_s*>(
        reinterpret_cast<const struct model_s *>(ps2::mod::Find(name)));
}

struct image_s * PS2_RegisterSkin(const char * name)
{
    return const_cast<struct image_s*>(
        reinterpret_cast<const struct image_s *>(&FindTextureOrPlaceholder(name, ps2::tex::ImageType::Skin)));
}

struct image_s * PS2_RegisterPic(const char * name)
{
    return const_cast<struct image_s*>(
        reinterpret_cast<const struct image_s *>(&FindTextureOrPlaceholder(name, ps2::tex::ImageType::Pic)));
}

// ------------------------------------------------------------------------------------------------
// 2D overlay
// ------------------------------------------------------------------------------------------------

void PS2_DrawGetPicSize(int * w, int * h, const char * name)
{
    // Callable outside Begin/EndFrame. Placeholder dimensions keep the
    // callers' centering math sane when the pic is missing.
    const ps2::tex::Texture & texture = FindTextureOrPlaceholder(name, ps2::tex::ImageType::Pic);
    *w = texture.width;
    *h = texture.height;
}

void PS2_DrawStretchPic(int x, int y, int w, int h, const char * name)
{
    const ps2::tex::Texture & texture = FindTextureOrPlaceholder(name, ps2::tex::ImageType::Pic);
    ps2::gs::SetTextureFor2D(texture);
    ps2::gs::DrawTexturedRect(x, y, w, h, 0, 0, texture.width, texture.height, kUiBrightness);
}

void PS2_DrawPic(int x, int y, const char * name)
{
    const ps2::tex::Texture & texture = FindTextureOrPlaceholder(name, ps2::tex::ImageType::Pic);
    ps2::gs::SetTextureFor2D(texture);
    ps2::gs::DrawTexturedRect(x, y, texture.width, texture.height,
                              0, 0, texture.width, texture.height, kUiBrightness);
}

void PS2_DrawChar(int x, int y, int c)
{
    DrawGlyph(x, y, c);
}

void PS2_DrawTileClear(int x, int y, int w, int h, const char * name)
{
    // Tiles the image over the given screen rectangle: texels are addressed in
    // screen space and wrap via the REPEAT mode set up in gs::Init().
    (void)name; // Quake only ever tiles "backtile" here.
    ps2::gs::SetTextureFor2D(*s_texBacktile);
    ps2::gs::DrawTexturedRect(x, y, w, h, x, y, x + w, y + h, kUiBrightness);
}

void PS2_DrawFill(int x, int y, int w, int h, int c)
{
    const u32 p = global_palette[c & 0xFF];
    const u8  r = static_cast<u8>(p & 0xFFu);
    const u8  g = static_cast<u8>((p >> 8) & 0xFFu);
    const u8  b = static_cast<u8>((p >> 16) & 0xFFu);
    ps2::gs::FillRect(x, y, w, h, r, g, b, 255);
}

void PS2_DrawFadeScreen()
{
    ps2::gs::FillRect(0, 0, ps2::gs::Width(), ps2::gs::Height(), 0, 0, 0, 128);
}

// ------------------------------------------------------------------------------------------------
// Cinematics
// ------------------------------------------------------------------------------------------------

void PS2_DrawStretchRaw(int x, int y, int w, int h, int cols, int rows, const byte * data)
{
    // Called every frame while a cinematic plays; the movie quad is a 2D draw,
    // so it joins the deferred overlay batch (opened lazily) like any other.
    ps2::cin::DrawFrame(x, y, w, h, cols, rows, data);
}

void PS2_CinematicSetPalette(const unsigned char * palette)
{
    ps2::cin::SetPalette(palette);
}

// ------------------------------------------------------------------------------------------------
// Frame rendering
// ------------------------------------------------------------------------------------------------

void PS2_BeginFrame(float cameraSeparation)
{
    (void)cameraSeparation;
    ps2::gs::BeginFrame();
    // 2D and 3D now draw freely between here and PS2_EndFrame: 2D primitives
    // open the deferred overlay batch lazily and it flushes automatically at
    // each 2D->3D boundary and in gs::EndFrame() - no explicit bracket here.
}

void PS2_EndFrame()
{
    // Cinematic playback test: the movie quad is a 2D draw, run at frame's end
    // so it lands over the fullscreen console but under the FPS counter. Enable
    // with cvar "ps2_testcin 1".
    ps2::test::RunCinematics();

    // VU1 bring-up scene (cvar "ps2_testcube 1"): a 3D draw, so it flushes the
    // 2D overlay accumulated above and lands on top - staying visible over the
    // fullscreen console Quake forces while disconnected (its batch programs
    // its own z-test). gs::EndFrame() then sends any remaining 2D and flips.
    ps2::test::DrawRotatingCube();

    DrawFpsCounter();
    DrawMemUsageOverlay();
    DrawVramUsageOverlay();

    ps2::gs::EndFrame();
}

void PS2_RenderFrame(refdef_t * viewDef)
{
    PS2_Assert(viewDef != nullptr);
    ps2::view::RenderFrame(*viewDef);
}

} // extern "C"
