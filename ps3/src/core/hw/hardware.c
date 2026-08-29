/* hardware.c — device set construction and reset. */
#include "hardware.h"
#include "../ppc/interp/interp.h"
#include "gx_fifo.h"
#include "../core_timing.h"
#include "../mem/memmap.h"
#include "../../common/log.h"

void pi_attach_cpu(PPCState *s);

void hw_init(PPCState *state, int wii_mode)
{
    /* The recompiler reads const_one and the quantise/dequantise scale tables
     * straight out of the state; they are data, so nothing faults if they are
     * zero -- scaled psq loads just silently produce 0.0. Fill them at the one
     * point every harness and the console all pass through. */
    ppc_init_constants(state);

    /* Mid-slice clocks read this CPU's downcount to see how far the current
     * slice has run. */
    timing_bind_cpu(state);

    mmio_reset();

    /* The CPU pointer is attached before any device exists, so a device that
     * signals during its own initialization cannot find a half-built machine. */
    pi_attach_cpu(state);
    pi_init();

    /* NTSC by default. The standard is really a property of the title and the
     * console's region setting; until a loader supplies one, the more common
     * of the two is the better default. */
    vi_init(VI_NTSC);

    /* The command processor after PI, because its interrupt goes through PI,
     * and after the CPU is attached, because the write-gather pipe stages into
     * PPCState. */
    gxfifo_attach_cpu(state);
    gxfifo_init();

    /* The IPC mailbox to IOS exists only on the Wii. Without it a Wii title
     * spins forever at its first IOS call. */
    if (wii_mode)
        ipc_init();

    /* The DSP exists on both consoles; a title resets and hand-shakes with it
     * during early init regardless of whether it ends up producing sound. */
    dsp_init();
    ai_init();
    pe_init();

    LOG_INFO(LOG_CORE, "hardware: PI + VI + CP%s online (%s)",
             wii_mode ? " + IPC" : "", wii_mode ? "Wii" : "GameCube");
}

void hw_reset(void)
{
    pi_reset();
    vi_reset();
    gxfifo_reset();
}

void hw_shutdown(void)
{
    mmio_reset();
}
