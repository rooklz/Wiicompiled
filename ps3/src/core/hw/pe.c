/* pe.c — the pixel engine's completion signalling.
 *
 * The GPU tells the CPU it has finished work in two ways, and a title's render
 * loop is built on both. `GXSetDrawDone` writes BP register 0x45 and then waits
 * for the *finish* interrupt; `GXSetDrawSync` writes a token through BP 0x47/
 * 0x48 and waits for the *token* interrupt. A frame that issues either and
 * never hears back simply stops -- and because a title's loading work is
 * usually sequenced behind "the previous frame is done", everything downstream
 * of it stops too: no assets requested, no disc reads, every thread idle.
 *
 * Registers and semantics follow Dolphin's VideoCommon/PixelEngine (the control
 * register's enable/acknowledge bits and UpdateInterrupts).
 */
#include "hardware.h"
#include "../mem/memmap.h"
#include "../gx/bp.h"
#include "../../common/log.h"

#define PE_BASE          0xCC001000u
#define PE_TOKEN_REG     0x0E    /* last token written by the GPU     */
#define PE_CTRL_REGISTER 0x0A    /* enables and acknowledgements      */

/* PE_CTRL_REGISTER bits (Dolphin UPECtrlReg). */
#define PE_CTRL_TOKEN_ENABLE  0x0001
#define PE_CTRL_FINISH_ENABLE 0x0002
#define PE_CTRL_TOKEN_ACK     0x0004
#define PE_CTRL_FINISH_ACK    0x0008

static u16 s_control;
static u16 s_token;
static int s_token_pending, s_finish_pending;

static void pe_update_irq(void)
{
    pi_set_interrupt(PI_INT_PE_TOKEN,
                     (s_token_pending && (s_control & PE_CTRL_TOKEN_ENABLE)) ? 1 : 0);
    pi_set_interrupt(PI_INT_PE_FINISH,
                     (s_finish_pending && (s_control & PE_CTRL_FINISH_ENABLE)) ? 1 : 0);
}

/* Called when the command stream reaches BP 0x45: the GPU has drained the work
 * queued before it. Real hardware takes time to get here; we signal as soon as
 * the parser sees it, which is early but never late. */
void pe_signal_finish(void)
{
    s_finish_pending = 1;
    pe_update_irq();
}

/* BP 0x47/0x48: a token value the title polls or takes an interrupt on. */
void pe_signal_token(u16 token, int with_interrupt)
{
    s_token = token;
    if (with_interrupt) {
        s_token_pending = 1;
        pe_update_irq();
    }
}

static u32 pe_read(u32 addr, unsigned size, void *ctx)
{
    (void)size; (void)ctx;
    /* A frame-done wait polls these registers call-free; the pump hook is
     * the one delivery point inside such a spin (same pattern as ipc_read:
     * guest-thread + EE + non-reentrant guards live inside the pump). */
    {   extern void wc_pump_from_mmio(void);
        wc_pump_from_mmio();
    }
    switch (addr - PE_BASE) {
    case PE_TOKEN_REG:     return s_token;
    case PE_CTRL_REGISTER: return s_control;
    default:               return 0;
    }
}

static void pe_write(u32 addr, u32 value, unsigned size, void *ctx)
{
    (void)size; (void)ctx;
    if ((addr - PE_BASE) == PE_CTRL_REGISTER) {
        /* The acknowledge bits clear a pending signal rather than being stored;
         * the enable bits are state. */
        if (value & PE_CTRL_TOKEN_ACK)  s_token_pending = 0;
        if (value & PE_CTRL_FINISH_ACK) s_finish_pending = 0;
        s_control = (u16)(value & (PE_CTRL_TOKEN_ENABLE | PE_CTRL_FINISH_ENABLE));
        pe_update_irq();
    }
}

void pe_reset(void)
{
    s_control = 0;
    s_token = 0;
    s_token_pending = s_finish_pending = 0;
}

/* Rescue diagnostics: the four internals that decide the draw-sync verdict. */
void pe_debug_state(unsigned out[4])
{
    out[0] = s_control;
    out[1] = s_token;
    out[2] = (unsigned)s_token_pending;
    out[3] = (unsigned)s_finish_pending;
}

void pe_init(void)
{
    pe_reset();
    bp_set_pe_hooks(pe_signal_finish, pe_signal_token);
    mmio_register(PE_BASE, 0x100, pe_read, pe_write, NULL, "PE");
}
