
# Quake II port for the PlayStation 2

## Overview

This is an unofficial fan-made port, targeting the PlayStation 2 console, of the original
[Quake II source code released by id Software][link_id_repo].

The port relies solely on the free [PS2DEV SDK][link_ps2_dev] to provide rendering, input,
audio and system services for the Quake II engine — no official Sony SDK, no proprietary
libraries. The long-term goal is a fully functional and playable single-player Quake II
on the PS2, built with freely available tools only.

The engine, game and server code under [src/client/](src/client/), [src/common/](src/common/),
[src/game/](src/game/) and [src/server/](src/server/) is id's original C, kept as close to
untouched as possible. Everything specific to the console lives in [src/ps2/](src/ps2/) and is
written in modern C++ (C++20, no exceptions, no RTTI, warnings-as-errors). The two halves meet
at a small number of well defined seams — `refexport_t`, `SNDDMA_*`, `IN_*`, `Sys_*`, `NET_*`
— exactly where the original win32/linux backends plugged in.

### What works today

- **A complete rendered frame.** World BSP geometry with textures and lightmaps, brush model
  entities, MD2 alias models (monsters, items, the view weapon), sprites, the skybox,
  particles, beams, dynamic light flares, translucent surfaces (water/glass/lava), the
  fullscreen damage/powerup blend, and the whole 2D layer (console, HUD, menus, cinematics).
- **VU1-accelerated rendering.** Vertex transform, guard-band clip judgement and GS packet
  building all run on VU1 microprograms written in VCL; MD2 keyframe interpolation runs there
  too. EE-side triangle clipping uses VU0 macro mode for the vector math.
- **Coloured lightmaps**, including animated light styles and dynamic lights folded in at
  frame time.
- **Sound.** The stock portable Quake mixer painting into a ring buffer that is streamed to
  the SPU2 through the `audsrv` IOP driver, including cinematic audio.
- **Input.** DualShock gamepad (analog sticks + full button mapping) and an optional USB
  keyboard through `ps2kbd`, usable simultaneously.
- **Memory.** A `dlmalloc`-backed program-wide heap with per-subsystem tag accounting, and a
  GS VRAM texture heap with LRU eviction and defragmentation.
- Runs on both the **PCSX2 emulator** (game data over `host:`) and **real hardware**
  (game data from USB mass storage).

---

## Getting started

### Prerequisites

1. **The ps2dev toolchain and PS2SDK.** Build/install from [ps2dev][link_ps2_dev]
   (`ps2toolchain` + `ps2sdk`). The build expects these on `PATH`:

   | Tool | Comes from | Typical location |
   | --- | --- | --- |
   | `mips64r5900el-ps2-elf-gcc` / `-g++` | ps2toolchain | `$PS2DEV/ee/bin` |
   | `vclpp`, `openvcl` | openvcl | `$PS2DEV/bin` |
   | `dvp-as` | ps2toolchain (dvp) | `$PS2DEV/dvp/bin` |
   | `bin2c` | ps2sdk | `$PS2SDK/bin` |

   Note the toolchain prefix is `mips64r5900el-ps2-elf-*`, not the older `ee-gcc`.

   A `~/.zshenv` (or equivalent) along these lines is what the Makefile and the VSCode
   tasks expect:

   ```sh
   export PS2DEV=$HOME/ps2dev
   export PS2SDK=$PS2DEV/ps2sdk
   export PATH=$PS2DEV/bin:$PS2DEV/ee/bin:$PS2DEV/dvp/bin:$PS2SDK/bin:$PATH
   ```

   Both `PS2DEV` and `PS2SDK` can also be overridden on the `make` command line.

2. **PCSX2**, if you want to run it on the emulator, plus a PS2 BIOS image for it.
   The Makefile's `run` target defaults to `/Applications/PCSX2.app/Contents/MacOS/PCSX2`;
   override `PCSX2=` for other platforms/locations.

3. **The Quake II game data.** Not included here and not redistributable — copy it from
   your own retail install. Place it in a `baseq2/` directory at the root of
   the repository (it is `.gitignore`d):

   ```
   quake2-ps2/
     baseq2/
       pak0.pak
       players/      (optional, loose files)
       video/        (optional, .cin cinematics)
       config.cfg
   ```

   A loose-file tree works as well as the `.pak`; [src/tools/unpak.c](src/tools/unpak.c)
   builds a small extractor for that.

### Building

```sh
make            # build build/quake2.elf (+ host tools)
make run        # build, then launch it in PCSX2
```

Header dependencies are tracked automatically (`-MMD`), so editing a header rebuilds only
the affected objects.

#### Makefile targets

| Target | What it does |
| --- | --- |
| `make` / `make all` | Builds `build/quake2.elf` and the host tools. |
| `make run` | Builds, symlinks `build/baseq2` → `baseq2/`, and launches PCSX2 with the ELF. |
| `make tools` | Builds the host-side command line tools into `build/tools/` (see below). |
| `make compiledb` | Regenerates `compile_commands.json` from the Makefile, for IntelliSense/clangd. Run after adding or removing source files. |
| `make clean` | Removes all build artifacts. |
| `make clean_vu` | Removes only the assembled VU microprograms (`build/vu/`). |

Useful variable overrides: `PS2DEV=`, `PS2SDK=`, `PCSX2=`, `HOST_CC=`, and
`-DPS2_FS_BASE_PATH=\"host:.\"` in `COMMON_DEFS` to pin the game data path and skip
autodetection.

#### Host tools

Built with the *host* compiler, not the EE toolchain:

- `build/tools/unpak` — extracts a Quake II `.pak` archive into a normal directory.
- `build/tools/imgdump` — dumps `.pcx` images into C byte arrays (RGB or raw 8-bit
  palettized), which is how the built-in console font, console background, HUD backtile and
  the global palette under [src/ps2/builtin/](src/ps2/builtin/) were generated. Those are
  compiled into the ELF so the executable can boot and print errors before any game data
  is found.

### Running in PCSX2

`make run` handles the launch (`PCSX2 -batch -elf build/quake2.elf`), but PCSX2 itself needs
a couple of settings that are **off by default**. Edit
`PCSX2.ini` **while PCSX2 is closed** — it rewrites the file on exit and will clobber your
changes. On macOS it lives at `~/Library/Application Support/PCSX2/inis/PCSX2.ini`.

#### 1. Enable the host filesystem (`host:`)

```ini
[EmuCore]
HostFs = true
```

PCSX2 maps `host:` to the directory the ELF was loaded from — that is `build/`, and the
`run` target symlinks `build/baseq2` back to the repo's `baseq2/`. Without `HostFs`, the boot
probe finds nothing and the game halts with *"No game data found"*. The equivalent UI toggle
(Settings → Advanced → Enable Host Filesystem) is hidden unless advanced settings are shown.

#### 2. Enable the IOP console, to see the game's stdout

```ini
[Logging]
EnableIOPConsole = true
EnableFileLogging = true
```

The game's `Com_Printf` output goes to stdout, and ps2sdk routes stdout through the IOP fio
path — so the EE console setting alone shows nothing. With both flags on, the log appears in
the PCSX2 log window and in `logs/emulog.txt`, which can be followed live. All engine output
is prefixed with `[Q2]` so it can be separated from the emulator's own chatter:

```sh
tail -f ~/Library/Application\ Support/PCSX2/logs/emulog.txt | grep '\[Q2\]'
```

#### 3. Optional: a virtual USB keyboard

```ini
[USB1]
Type = hidkbd
```

This attaches a HID keyboard that passes host keystrokes straight through — no per-key
bindings to set up. Set `in_keyboard 1` in the game to bring the driver up. Two quirks worth
knowing:

- The emulated device reports itself as a JIS keyboard, so at boot it logs a batch of
  harmless `Missing host mapping for QKey '<name>'` warnings for JIS-only keys.
- It sends USB HID usage `0x34` (the *apostrophe* usage) for the host's `` ` ``/`~` key,
  and never sends `0x35` (grave). The port maps `0x34` to `` ` `` so the console toggle
  works under the emulator; on real hardware, hand it back with `in_keyboardmap 0x34 '`.

### Running on real hardware

The boot path probes `host:` first (which fails instantly on a console), then falls back to
USB mass storage: full IOP reset, sbv patches, and the embedded BDM/USB driver stack, waiting
for the drive to enumerate. Put `baseq2/` in the root of a FAT-formatted USB stick and load
`quake2.elf` with your launcher of choice. The BDM, USB, keyboard and sound IRX modules are
all embedded in the ELF by the Makefile's `bin2c` rule, so nothing else has to be on the
drive.

---

## Editor setup (VSCode)

The repo ships `.vscode/tasks.json` (Shift+Cmd+B → build) and `.vscode/launch.json`
(F5 → `make run`). Those inherit the toolchain environment from your shell profile, so they
need no configuration.

IntelliSense does need one edit: it is served by the C/C++ extension, not by a shell, so it
cannot read your `~/.zshenv`. `.vscode/c_cpp_properties.json` declares the SDK location
itself, and `PS2DEV` is the single literal path in the whole file — point it at your install:

```jsonc
{
  "configurations": [
    // ...
  ],
  "env": {
    "PS2DEV": "/Users/you/ps2dev", // <-- change this
    "PS2SDK": "${PS2DEV}/ps2sdk"
  }
}
```

`compile_commands.json` is per-machine and `.gitignore`d; generate it with `make compiledb`.
It carries the exact per-file flags (language standard, `-isystem` paths, warning set), so
both the C/C++ extension and clangd see what the compiler really sees. Regenerate it whenever
source files are added or removed.

---

## Source layout

```
src/
  client/ common/ game/ server/   id's original Quake II C code
  null/                           cd_null.c - the only remaining null stub (no CD audio)
  tools/                          host-side command line tools (imgdump, unpak)
  ps2/                            the PS2 backend - all new C++ code
    system/                       main() entry point, Sys_* seam, IOP boot, dlmalloc heap
    renderer/                     GS front-end, VRAM heap, textures, models, VU1 path
      vu1progs/                   VU1 microprograms (.vcl)
      tests/                      standalone bring-up scenes (test cube, cinematics)
    audio/                        SNDDMA_* seam, audsrv device, mix ring
    input/                        IN_* seam, DualShock pad, USB keyboard
    math/                         vector/matrix math for the renderer
    net/                          NET_* seam (loopback only)
    builtin/                      images baked into the ELF (font, palette, HUD tiles)
    debug/                        simple on-screen printing for fatal errors
```

Every file carries a header comment explaining what it does and *why* it does it that way;
those comments are the real documentation and this section is only a map to them.
[src/ps2/common.h](src/ps2/common.h) is the single seam between the C++ backend and the C
engine — backend sources usually include it rather than reaching into engine headers directly.

---

## Architecture

### Rendering

The renderer implements `refexport_t` in [ps2/renderer/ref.cpp](src/ps2/renderer/ref.cpp) —
the same interface `ref_gl` and `ref_soft` implemented, so the client above it is unmodified.

**Video mode and VRAM budget.** 640x448, NTSC/PAL auto-detected, double-buffered using both
GS drawing contexts. The framebuffer format is chosen by `ps2_fb_16bit` (default on): 16-bit
buffers cost 560 KB each instead of 1120 KB, which is where most of the texture heap's
headroom comes from, and halve the GS's colour write and blend-read bandwidth — at the cost
of 5:5:5 colour, which `ps2_fb_dither` can smooth over. Depth is 16-bit either way (the GS
requires colour and depth to share a page layout). What is left after the two framebuffers,
the z-buffer and two CLUTs — about 1.27 MB in the 16-bit configuration — becomes the
streamed texture heap.

**Frame structure.** `BeginFrame()` clears colour and depth; 2D and 3D then draw in any
order; `EndFrame()` flushes, waits for vsync and flips. 2D primitives accumulate into a
deferred batch with an always-pass z-test so they land on top, and that batch is flushed
automatically at each 2D→3D boundary and again at end of frame. Flushing at the boundary is
not just about layering: it consumes the deferred draws before a later 3D texture upload can
evict the VRAM they sample.

**VRAM texture heap** ([vram.h](src/ps2/renderer/vram.h)). Textures stream in on first bind;
while resident, binding one is just a `TEX0`/`TEX1` register write — no DMA, no pipeline
flush. When the heap fills, the least-recently-bound textures are evicted, never ones bound
in the current frame (their draws may still be in flight). Allocation failure is recoverable
rather than fatal: drain the GS, unpin, retry, defragment, retry. The heap is also
defragmented on level change.

**Textures and CLUTs** ([texture.h](src/ps2/renderer/texture.h)). Most world and model
textures stay 8-bit (`PSMT8`) and sample Quake's shared 256-entry palette uploaded once as a
CLUT, which keeps them a quarter the size of RGBA. A second CLUT is an alpha ramp used by
images that carry only a coverage signal — particles and lightmap atlases — where the index
*is* the alpha and the colour comes from the primitive.

**2D scrap atlases** ([scrap_atlas.h](src/ps2/renderer/scrap_atlas.h)). A texture occupies
every GS page its pixels touch, so a 24x24 HUD icon would otherwise cost a full 8 KB page.
The small pics are packed into shared 256x256 atlases instead — baseq2's 89 such pics drop
from 712 KB of VRAM to a fraction of that, and a HUD frame binds one texture instead of
fifteen.

**Lightmaps** ([lightmap.h](src/ps2/renderer/lightmap.h)). The BSP's baked light samples are
packed into 256x256 atlases at load time and baked into a second UV set on the vertices.
Surfaces whose lighting moved (animated light styles, dynamic lights) are rebuilt at frame
time and chained per atlas. Colour is the interesting part: the GS blend unit computes
`(A - B) * C + D` where `C` is a scalar alpha and never a second colour, so
`diffuse x coloured-lightmap` is not expressible as a single blend. So a luxel is split — the
atlases the hardware samples are alpha-only and carry the luxel's *intensity*, multiplied in
per pixel, while its chroma rides in an EE-RAM mirror sampled per vertex and folded into the
vertex colour the wall texture is modulated by. Intensity times chroma reconstructs the
luxel: full resolution in the term that varies per pixel, vertex resolution in the one that
barely varies at all.

**VU1 path** ([vu1.h](src/ps2/renderer/vu1.h), [vu1progs/](src/ps2/renderer/vu1progs/)).
Triangle batches are submitted as VIF1 source chains: frame constants (MVP, GS screen
mapping, clip scale) unpacked to fixed low VU addresses, then chunks of up to 96 vertices
unpacked into a double buffer with `MSCAL` to run the microprogram, which transforms,
judges the clip volume, builds the GS packet in place and `XGKICK`s it over PATH1. `XTOP`
flips on every kick, so the VIF unpacks the next chunk while VU1 still works on the current
one. There are two microprograms, written in VCL:

- `textured_triangles.vcl` — gouraud-shaded textured triangle lists.
- `lerped_triangles.vcl` — the same, but positions are interpolated between two
  byte-quantized MD2 keyframes on the VU, with an optional back-face cull by screen area.

Each batch's A+D block programs `TEST`, `ALPHA` and `ZBUF` alongside `TEX0`/`TEX1`, so a
batch draws with the right z-test, blend equation and depth-write mask regardless of what
the surrounding 2D packets left behind.

**Clipping** ([clip.h](src/ps2/renderer/clip.h)). The microprogram does not clip — a triangle
with any vertex outside its guard band is rejected whole via the ADC bit — so geometry that
can cross those planes is cut on the EE first, against the same six planes the VU judges.
Because everything a vertex carries is linear under a plane cut, a split is five quadword
lerps on VU0 rather than scalar float math, and the common whole-triangle-inside case copies
nothing.

**Frame pass order** ([render_view.cpp](src/ps2/renderer/render_view.cpp)), following
`ref_gl`'s `R_RenderView`: PVS + frustum culled world surfaces and the skybox, opaque
entities, translucent entities, dynamic light flares, particles, deferred translucent world
and brush surfaces, then the fullscreen polyblend.

### Sound

The mixing is entirely id's portable code — `client/snd_dma.c`, `snd_mem.c`, `snd_mix.c` are
in the build unchanged. The backend ([ps2/audio/](src/ps2/audio/)) only implements the five
`SNDDMA_*` entry points, exactly as `win32/snd_win.c` did:

- [`AudsrvDevice`](src/ps2/audio/audsrv_device.h) owns the IOP-side bring-up (`libsd.irx` +
  `audsrv.irx`, both embedded in the ELF) and the streaming session, exposing audsrv's queue
  as "how many bytes fit right now" and "take these". The SPU2 is only reachable from the
  IOP — there is no EE-side mapping of its registers — so every mixed byte crosses SIF.
  Output is 16-bit stereo at 22050 Hz by default (`s_khz`; 11025 and 44100 also work),
  chosen because it matches most of the game's WAVs *and* the audio in the `.cin` videos,
  which avoids a `snd_restart` whenever a cinematic plays.
- [`MixRing`](src/ps2/audio/mix_ring.h) is the 64 KB buffer the mixer paints into and the
  bookkeeping around it. The play position reported back to the engine is how far we have
  *submitted*, not where the SPU2 actually is — the same trick the waveOut backend used.
  Since submission is paced by the device's free space, that cursor advances at exactly the
  playback rate and is monotonic by construction, which is what `GetSoundtime()` needs, and
  it means a stall long enough to drain the queue (a level load) recovers on its own with no
  resync logic.

Audio bring-up failure is not fatal: a missing IOP driver just costs sound. `ps2_disable_sound 1`
turns it off deliberately.

### Input

[ps2/input/](src/ps2/input/) implements the `IN_*` seam over libpad and, optionally, the
`ps2kbd` driver. Both devices can be used at the same time.

**Gamepad.** [`GamePad`](src/ps2/input/pad.h) owns the connection state machine (connect →
request analog mode → ready) and per-frame polling, exposing the button mask and sticks
normalised to [-1, +1]. [input.cpp](src/ps2/input/input.cpp) maps that onto the engine: the
right stick rotates the camera and the left stick moves the player, with the same
sensitivity/threshold cvars (`joy_yawsensitivity` and friends) the original win32 joystick
code used — negate a sensitivity to invert that axis.

Buttons send **one of two keys depending on where input focus is**, so menus and the console
stay navigable without stealing gameplay binds. The gameplay keys are ordinary rebindable
`JOY`/`AUX` keys, given default binds at startup only if the user's config has not bound
them already:

| Button | Menu / console | In game (default bind) |
| --- | --- | --- |
| D-pad up / down | Up / Down | `invuse` / `inven` |
| D-pad left / right | Left / Right | `invprev` / `invnext` |
| Cross | Enter | `+moveup` (jump) |
| Circle | Escape | `+movedown` (crouch) |
| Square | Enter | `+use` |
| Triangle | Escape | `cmd help` |
| L1 / R1 | (same key as in game) | `weapnext` / `weapprev` |
| L2 / R2 | PgUp / PgDn | `+speed` / `+attack` |
| L3 / R3 | (same key as in game) | unbound / `centerview` |
| Start | Escape | Escape (menu toggle) |
| Select | `` ` `` | `` ` `` (console toggle) |

**Keyboard.** [`Keyboard`](src/ps2/input/keyboard.h) starts `usbd.irx` + `ps2kbd.irx` on
demand and reads the driver in raw scan-code mode, translating USB HID usages into Quake key
events, so the stock `default.cfg` binds work as they do on a PC. It is gated by
`in_keyboard` (a one-shot bring-up — the IOP modules must not be loaded twice), traced by
`in_keyboarddebug 1`, and individual usages can be remapped at runtime with
`in_keyboardmap <usage> <key>` since keyboards disagree on which usage a physical key sends.

### System and memory

[main.cpp](src/ps2/system/main.cpp) locates the game data (`host:` first, then the USB/BDM
bring-up) *before* `Qcommon_Init`, because `FS_InitFilesystem` opens pak files during init
while the pad driver loads its `rom0:` modules later — after the IOP reset, which is the
required order.

Memory is a single program-wide `dlmalloc` heap ([heap.h](src/ps2/system/heap.h)) with
`operator new`/`delete` and the engine's `Z_Malloc` routed through a tag-accounting layer
(`MEMTAG_QUAKE`, `MEMTAG_RENDERER`, `MEMTAG_TEXIMAGE`, `MEMTAG_MDL_*`, `MEMTAG_LIGHTMAP`, …).
The RAM the game can never allocate — EE kernel, ELF image, stack — is booked against
`MEMTAG_MISC` at startup so the tags add up to a faithful picture of the console's 32 MB.
Built with `-fno-exceptions`, so a failed allocation is a fatal `Sys_Error`, not a throw.

Networking is loopback only ([net/net.cpp](src/ps2/net/net.cpp)) — enough for a local
single-player/listen-server game; remote sends are dropped.

---

## Debugging aids

All of these are cvars unless noted. The four overlays default to on; everything else
defaults to the normal rendering path.

**Overlays:** `ps2_show_fps`, `ps2_show_memstats` (per-tag heap usage), `ps2_show_vramstats`
(texture heap occupancy and per-frame uploads), `ps2_show_drawstats` (nodes walked, surfaces,
triangles drawn/clipped/culled, batches, entities, particles, dlights).

**Isolating parts of the frame:** `ps2_skip_world`, `ps2_skip_entities`, `ps2_skip_sky`,
`ps2_skip_particles`, `ps2_skip_sprites`, `ps2_skip_brushmodels`, `ps2_skip_alpha_surfaces`,
`ps2_skip_weapon_model`, `ps2_force_null_models`.

**Renderer toggles:** `ps2_fb_16bit`, `ps2_fb_dither`, `ps2_lightmaps`, `ps2_lightmap_only`,
`ps2_lightmap_color`, `ps2_lightmap_modulate`, `ps2_dynamic_lightmaps`, `ps2_backface_cull`,
`ps2_md2_lerp_on`, `ps2_md2_vu_lerp`, `ps2_md2_cullface`, `ps2_md2_shadows`,
`ps2_md2_clip_weapon`, `ps2_hd_particles`, `ps2_polyblend`, `ps2_skymip`,
`ps2_sky_full_bounds`.

**Bring-up scenes:** `ps2_testcube 1` (VU1 path smoke test, with `ps2_testcube_tess`,
`ps2_testcube_vulerp`, `ps2_testcube_vram_tex_eviction`), `ps2_testcin 1` (cinematic
playback test).

**Commands:** `ps2_dump_iop_mods` lists the currently loaded IOP modules;
`in_keyboardmap <usage> <key>` remaps a USB scan code.

**Loading a quiet map for renderer work.** `deathmatch 1` is the switch that frees every monster at spawn. Both it and `cheats` are latched, so they must be set *before* `map`:

```
killserver ; deathmatch 1 ; cheats 1 ; map base1
```

---

## Pending work

**Rendering**

- Non-power-of-two wall textures sample incorrectly and need resampling at load time
  ([image_load.cpp:153](src/ps2/renderer/image_load.cpp#L153)).
- No texture mipmaps ([texture.h:119](src/ps2/renderer/texture.h#L119)); minification
  aliasing is visible on distant world geometry.
- Water/turbulent surface warping is still done on the EE and is a good candidate to move to
  VU1 ([render_view.cpp:889](src/ps2/renderer/render_view.cpp#L889)), as is particle
  billboard generation ([render_view.cpp:2077](src/ps2/renderer/render_view.cpp#L2077)).
- `TextureTransfer` has no `EnsureSpace` and works off a fixed 128-qword scratch packet
  ([gs.cpp:633](src/ps2/renderer/gs.cpp#L633)); CLUT reloads could also be skipped with
  `CLUT_COMPARE_CBP0` ([gs.cpp:723](src/ps2/renderer/gs.cpp#L723)).
- The 2D path still uses the old `libpacket`; `libpacket2` (already used by the VU1 path)
  would remove that dependency
  ([render_packet.h:37](src/ps2/renderer/render_packet.h#L37)).
- General performance work — the target is a solid 60 fps "performance mode" in real gameplay, and that hasn't been profiled seriously yet.

**Engine features**

- **Background music.** There is no CDVD path at all, so `null/cd_null.c` still stands in for
  CD audio. Streaming OGG/ADPCM tracks off the game data would be the way in.
- **Save games** have not been exercised — the filesystem write path on `mass:`/`host:`
  needs checking.
- **Memory card support** for configs and saves.

**Build and project**

- Separate **debug and release targets**: release should disable asserts
  (`PS2_QUAKE_ASSERTS`), drop the debug overlays and strip the ELF
  ([Makefile:140](Makefile#L140)).
- Package a ready-to-run **`.iso`/ELF release** so it can be tried without a toolchain.
- Real-hardware testing; hardware-only issues (timing, IOP module quirks, USB enumeration) are the most likely place for surprises.

Contributions are welcome. The one style rule worth stating up front: the backend is held to
`-Werror` with a strict warning set (`-Wconversion`, `-Wsign-conversion`, `-Wshadow`,
`-Wdouble-promotion`, `-Wcast-align` and more), so conversions must be intentional and
explicit. The engine's original C is deliberately exempt from all of it. Formatting follows
the `_clang-format` file at the root of the repo.

---

## License

Quake II was originally released as GPL, and it remains as such. New code written for the PS2
port and any changes made to the original source code are also released under the GNU General
Public License version 2. See the accompanying LICENSE file for details.

You can also find a copy of the GPL version 2 [in here][link_gpl_v2].

[link_id_repo]: https://github.com/id-Software/Quake-2
[link_ps2_dev]: https://github.com/ps2dev
[link_gpl_v2]:  https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html
