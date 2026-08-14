/* ================================================================================================
 * File: audsrv_device.cpp
 * Brief: PCM output device over the audsrv IOP driver. See audsrv_device.h.
 *
 *  audsrv keeps a ring buffer on the IOP sized to ten of its 512-sample feed blocks -
 *  9400 bytes, about 100ms, at 22050Hz/16bit/stereo - and a thread that drains it into
 *  the SPU2 93.75 times a second, upsampling to the SPU2's native 48kHz on the way.
 *  Two of its quirks shape everything below:
 *
 *   - audsrv_play_audio() copies only as much as the ring has room for, but the EE-side
 *     wrapper advances its source pointer by the size it was *asked* for. Submitting
 *     more than audsrv_available() therefore drops audio silently.
 *   - The ring reports "write cursor == read cursor" as empty rather than full, so
 *     filling it right to the last byte makes a full queue read back as drained.
 *
 *  FreeBytes() answers both: it is the only size Enqueue() may ever be handed.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#include "ps2/audio/audsrv_device.h"
#include "ps2/system/iop_boot.h"
#include "ps2/common.h"

#include <audsrv.h>

// IRX module images embedded by the Makefile's bin2c rule (IRX_FILES). libsd is
// ps2sdk's open-source reimplementation of Sony's sound driver (freesd.irx is a
// symlink to the same binary); audsrv imports its sceSd* entry points, so it has
// to be running first.
extern "C" {
extern unsigned char libsd_irx[];
extern unsigned int  size_libsd_irx;
extern unsigned char audsrv_irx[];
extern unsigned int  size_audsrv_irx;
}

namespace ps2::audio {
namespace {

// Set once the IOP modules are up and audsrv_init() has bound the RPC channel.
// The whole sequence is one-shot: SifExecModuleBuffer fails on an already resident
// module, and a second audsrv_init() would leak the EE-side RPC thread it spawns.
static bool s_iopStarted = false;

} // namespace

bool AudsrvDevice::StartIopSide()
{
    if (s_iopStarted)
    {
        return true;
    }

    if (!ps2::sys::StartIopModuleFromBuffer("libsd", libsd_irx, size_libsd_irx))
    {
        return false;
    }

    if (!ps2::sys::StartIopModuleFromBuffer("audsrv", audsrv_irx, size_audsrv_irx))
    {
        return false;
    }

    const int err = audsrv_init();
    if (err != AUDSRV_ERR_NOERROR)
    {
        Com_Printf("WARNING: audsrv_init failed (%d: %s)\n", err, audsrv_get_error_string());
        return false;
    }

    s_iopStarted = true;
    return true;
}

bool AudsrvDevice::Init(const int sampleRateHz)
{
    m_ready = false;

    if (!StartIopSide())
    {
        return false;
    }

    audsrv_fmt_t format;
    format.freq     = sampleRateHz;
    format.bits     = kSampleBits;
    format.channels = kChannels;

    const int err = audsrv_set_format(&format);
    if (err != AUDSRV_ERR_NOERROR)
    {
        Com_Printf("WARNING: audsrv_set_format(%dHz, %d bit, %d ch) failed (%d: %s)\n",
                   sampleRateHz, kSampleBits, kChannels, err, audsrv_get_error_string());
        return false;
    }

    // Quake scales every sample by s_volume as it mixes (snd_scaletable / snd_vol),
    // so the hardware side stays wide open.
    audsrv_set_volume(MAX_VOLUME);

    m_ready = true;
    return true;
}

void AudsrvDevice::Shutdown()
{
    if (!m_ready)
    {
        return;
    }

    // Deliberately not audsrv_quit(): leaving the IOP modules and the audsrv EE RPC
    // thread up makes snd_restart - which cl_main.c triggers on s_khz changes and
    // cinematics - a plain audsrv_set_format() instead of a full teardown. Nothing
    // on the PS2 ever returns from main() anyway.
    audsrv_stop_audio();
    m_ready = false;
}

int AudsrvDevice::FreeBytes() const
{
    if (!m_ready)
    {
        return 0;
    }

    // Hold one frame back so the write cursor can never land exactly on the read
    // cursor, which audsrv reports as an empty ring rather than a full one.
    const int freeBytes = audsrv_available() - kFrameBytes;
    if (freeBytes < kFrameBytes)
    {
        return 0;
    }

    // Whole stereo frames only: audsrv steps its read cursor by four bytes, so a
    // misaligned write cursor would swap left and right for the rest of the session.
    return freeBytes & ~(kFrameBytes - 1);
}

int AudsrvDevice::Enqueue(const u8 * const data, const int sizeBytes)
{
    if (!m_ready || sizeBytes <= 0)
    {
        return 0;
    }

    const int sent = audsrv_play_audio(reinterpret_cast<const char *>(data), sizeBytes);
    return (sent > 0) ? sent : 0;
}

} // namespace ps2::audio
