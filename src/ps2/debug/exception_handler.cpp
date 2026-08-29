/* ================================================================================================
 * File: exception_handler.cpp
 * Brief: EE CPU exception post-mortem. See exception_handler.h.
 *
 *  Built on ps2sdk's libeedebug, which owns the hard part: it replaces the EE's level 1
 *  exception vectors with an assembly stub that spills the full register set into an
 *  EE_RegFrame and dispatches to a C handler per cause. All this file adds is the handler
 *  that prints the frame and unwinds the stack behind it.
 *
 * This source code is released under the GNU GPL v2 license.
 * ================================================================================================ */

#if PS2_QUAKE_DEBUG
#include "ps2/debug/exception_handler.h"
#include "ps2/debug/stack_trace.h"

#include <cstdio>
#include <tamtypes.h>
#include <ee_debug.h>

// Bounds of our own .text, from the ps2sdk linkfile. Used to tell a fault inside
// the program from one the kernel took on our behalf - the two want unwinding
// from different places.
extern "C" {
    extern u8 _ftext[];
    extern u8 _etext[];
}

namespace ps2::debug {
namespace {

// MIPS ExcCode values, indexed by the cause field libeedebug dispatches on.
// Only the ones it routes to a level 1 handler are ever seen here; the gaps are
// syscall and breakpoint, which the kernel keeps for itself.
const char * CauseName(const int cause)
{
    switch (cause)
    {
    case 1  : return "TLB modified";
    case 2  : return "TLB refill (load/fetch)";
    case 3  : return "TLB refill (store)";
    case 4  : return "Address error (load/fetch)";
    case 5  : return "Address error (store)";
    case 6  : return "Bus error (instruction fetch)";
    case 7  : return "Bus error (data)";
    case 10 : return "Reserved instruction";
    case 11 : return "Coprocessor unusable";
    case 12 : return "Arithmetic overflow";
    case 13 : return "Trap";
    default : return "Unknown";
    }
}

// A fault inside the handler would re-enter it and spin, which is exactly the
// double fault this exists to replace. One pass only: after that, stop.
static volatile bool s_handlingException = false;

// The low 32 bits of a saved 128-bit EE register.
inline u32 Reg(const u32 (&r)[4]) { return r[0]; }

inline bool InOurText(const u32 addr)
{
    return addr >= reinterpret_cast<u32>(_ftext) && addr < reinterpret_cast<u32>(_etext);
}

// Prints one unwind. 'pc' must be an address the scanner can walk back from -
// see the note at the call sites about which one to hand it.
void PrintUnwind(const char * const what, const u32 pc, const u32 sp)
{
    u32 frames[kStackTraceMaxFrames];
    const int count = detail::WalkStack(pc, sp, frames, kStackTraceMaxFrames);

    std::printf("----------------- STACK TRACE (%s) -----------------\n", what);
    for (int i = 0; i < count; ++i)
    {
        std::printf("#%-2d 0x%08x\n", i, frames[i]);
    }
    if (count == 0)
    {
        std::printf("%s", "<unavailable - could not unwind>\n");
    }
    else if (count == kStackTraceMaxFrames)
    {
        std::printf("%s", "... (truncated)\n");
    }
    std::fflush(stdout);
}

int OnException(EE_RegFrame * const frame)
{
    if (s_handlingException)
    {
        std::printf("\n*** Fault inside the exception handler - stopping. ***\n");
        std::fflush(stdout);
        for (;;) {} // Nothing safe left to do; hang here rather than loop the vector.
    }
    s_handlingException = true;

    const int cause = static_cast<int>((frame->cause >> 2) & 0x1F);

    std::printf("\n"
                "=============== EE CPU EXCEPTION ===============\n"
                "Cause    : %d (%s)\n"
                "EPC      : 0x%08x   <- the faulting instruction\n"
                "BadVAddr : 0x%08x   <- the address it touched\n"
                "Status   : 0x%08x   Cause raw: 0x%08x\n"
                "ra       : 0x%08x   sp: 0x%08x   fp: 0x%08x   gp: 0x%08x\n",
                cause, CauseName(cause),
                frame->epc, frame->badvaddr, frame->status, frame->cause,
                Reg(frame->ra), Reg(frame->sp), Reg(frame->fp), Reg(frame->gp));

    // The argument registers are worth having: a null pointer handed to a callee
    // is the common shape of this fault, and $a0-$a3 usually still hold it.
    std::printf("a0-a3    : 0x%08x 0x%08x 0x%08x 0x%08x\n"
                "v0-v1    : 0x%08x 0x%08x\n",
                Reg(frame->a0), Reg(frame->a1), Reg(frame->a2), Reg(frame->a3),
                Reg(frame->v0), Reg(frame->v1));

    // Flush before unwinding. Printing from exception context goes out over the
    // same path the fault may have interrupted; if the unwind below wedges, the
    // four lines above are the ones actually worth having.
    std::fflush(stdout);

    // Where to unwind from depends on where the fault landed.
    //
    // Inside our own text, EPC names the faulting instruction and the scanner can
    // walk back from it to the function prologue. Outside it - a kernel routine
    // handed a bad pointer, which is what "pc=0x82000 addr=0x0" was - EPC is in
    // code the scanner cannot read prologues for, and unwinding from it produces
    // fiction. There $ra is the useful number: it points back into whichever of
    // our functions made the call.
    const u32 epc = frame->epc;
    const u32 ra  = Reg(frame->ra);
    const u32 sp  = Reg(frame->sp);

    if (InOurText(epc))
    {
        PrintUnwind("from EPC", epc, sp);
    }
    else
    {
        std::printf("EPC 0x%08x is outside this program's text (0x%08x-0x%08x): the fault was\n"
                    "taken in kernel or library code we were calling. Unwinding from $ra instead.\n",
                    epc, reinterpret_cast<u32>(_ftext), reinterpret_cast<u32>(_etext));

        if (InOurText(ra))
        {
            PrintUnwind("from $ra", ra, sp);
        }
        else
        {
            std::printf("$ra 0x%08x is outside our text too - the call chain is gone, which\n"
                        "usually means a smashed stack or a jump through a corrupt pointer.\n", ra);
        }
    }

    std::printf("%s", "Resolve with: mips64r5900el-ps2-elf-addr2line -f -C -e "
                      "build/<config>/quake2_unstripped.elf <addr>\n");
    std::printf("%s", "================================================\n");
    std::fflush(stdout);

    // Returning would resume at EPC and fault again forever. Sys_Error is not an
    // option either - it draws, and the renderer's state is whatever the fault
    // left it. Hang, with everything already printed.
    for (;;) {}
}

} // namespace

void InstallExceptionHandlers()
{
    // Level 1 only. Level 2 is the debug/counter vector, used by hardware
    // breakpoints; nothing here sets any, and installing it would replace the
    // vector a real debugger wants.
    if (ee_dbg_install(1) < 0)
    {
        std::printf("WARNING: could not install EE exception handlers.\n");
        return;
    }

    // The causes libeedebug routes to level 1 handlers: TLB refill (1-3),
    // address and bus errors (4-7), and the instruction-level faults (10-13).
    // Syscall (8) and breakpoint (9) are deliberately absent - the kernel and a
    // debugger own those.
    for (int cause = 1; cause <= 13; ++cause)
    {
        if (cause == 8 || cause == 9) { continue; }
        ee_dbg_set_level1_handler(cause, OnException);
    }

    std::printf("EE exception handlers installed.\n");
}

} // namespace ps2::debug
#endif // PS2_QUAKE_DEBUG
