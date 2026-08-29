/* pi.c — processor interface: interrupt aggregation and machine identity.
 *
 * Every interrupt source in the machine reports here, and this is the only
 * place that decides whether the CPU can currently see one. That decision is a
 * three-way conjunction -- a cause bit is set, the corresponding mask bit is
 * set, and MSR[EE] is on -- and keeping it in one function is what prevents the
 * two classic failures: an interrupt that is raised twice because two devices
 * both poked the CPU, and one that never arrives because a device checked the
 * mask before the guest had written it.
 */
#include <stdlib.h>
#include "hardware.h"
#include "../mem/memmap.h"
#include "../ppc/interp/interp.h"
#include "../../common/log.h"

/* Register offsets from HW_PI_BASE. */
#define PI_INTSR        0x00    /* interrupt cause (write 1 to clear) */
#define PI_INTMR        0x04    /* interrupt mask                     */
#define PI_FIFO_BASE    0x0C
#define PI_FIFO_END     0x10
#define PI_FIFO_WPTR    0x14
#define PI_RESET        0x24
#define PI_REVISION     0x2C

/* Console identity. Titles read this to distinguish hardware revisions; the
 * value here is the retail Wii's, which is what a Wii title expects to see. */
#define PI_REVISION_WII 0x00000011u

typedef struct {
    u32 intsr;
    u32 intmr;
    u32 fifo_base;
    u32 fifo_end;
    u32 fifo_wptr;
    u32 reset;
} PIState;

static PIState    s_pi;
static PPCState  *s_cpu;

/* Escape hatch for the early-exit above, armed by dolphin-noirqexit.txt so a
 * single build can be A/B'd on hardware without a rebuild. */
int g_pi_no_irq_exit;

/* Off-console A/B: the PS3 build arms g_pi_no_irq_exit from a flag file, but
 * the qemu harnesses have no such path, so honour an environment variable too.
 * Read once -- this sits on the interrupt path. */
static int pi_irq_exit_disabled(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("PI_NO_IRQ_EXIT");
        v = (e && *e && *e != '0') ? 1 : 0;
    }
    return v || g_pi_no_irq_exit;
}

/* ------------------------------------------------------------------ */

/* Re-evaluate whether the CPU should be seeing an external interrupt.
 *
 * Called after *any* change to cause, mask, or MSR[EE]. Raising is edge-like
 * from the CPU's point of view but the condition is level-based here, which is
 * the correct model: a device that stays asserted while the guest unmasks it
 * must produce an interrupt at the moment of unmasking, not be lost. */
static void pi_update(void)
{
    int pending = (s_pi.intsr & s_pi.intmr) != 0;

    if (!s_cpu)
        return;

    /* Model the interrupt *line level*, not an edge, and do NOT gate it on
     * MSR[EE] here: a device that asserts while the guest has interrupts masked
     * off must stay asserted so it is taken the moment the guest re-enables
     * them. Delivery is gated on MSR[EE] in ppc_deliver_exception; withdrawing
     * the exception here instead loses every interrupt raised inside a critical
     * section -- which is exactly how an IPC reply asserted with EE=0 never
     * reached the handler. */
    if (pending) {
        s_cpu->exceptions |= EXC_EXTERNAL_INT;

        /* End the slice now, so the guest vectors to its handler within a
         * block or two instead of after the rest of the grant.
         *
         * Exceptions are delivered once, at the top of jit_run -- deliberately,
         * because delivering mid-block raced StaticR.rel's context swap (see
         * jit.c). That is architecturally right but it makes interrupt latency
         * a whole slice, ~7,400 guest instructions at the measured rate. A
         * title that blocks on IOS pays that twice per transaction, once for
         * the ack and once for the reply, and spins in its wait loop for every
         * instruction of it. MKWii's RFL (Mii) loader does exactly this: the
         * on-console block profiler measured its poll loop at 0x800bbe80 as
         * 52% of ALL executed guest instructions.
         *
         * Requesting an exit does not change *where* the exception is taken --
         * still only at a slice boundary, still gated on MSR[EE] -- it just
         * brings the boundary forward. The scheduler charges only what actually
         * ran, because timing_advance derives consumed time from the leftover
         * downcount, so emulated time stays correct.
         *
         * Gated on MSR[EE]: with interrupts masked the guest cannot take it
         * yet, and cutting the slice short would burn dispatches for nothing.
         * The line stays latched either way. */
        if ((s_cpu->msr & MSR_EE) && !pi_irq_exit_disabled())
            s_cpu->exit_requested = 1;
    } else {
        s_cpu->exceptions &= ~EXC_EXTERNAL_INT;
    }
}

static volatile u32 s_pi_raises;

/* Record an interrupt-worthy event on a line that may already be asserted.
 * A device with more than one event behind one cause bit (IPC: acknowledge and
 * reply) calls this so the port can deliver one handler run per event rather
 * than one per rising edge of the shared line. */
void pi_note_event(void) { __atomic_add_fetch(&s_pi_raises, 1u, __ATOMIC_SEQ_CST); }

/* How many times a cause line has been asserted, ever. */
u32 pi_raise_seq(void) { return s_pi_raises; }

int pi_interrupt_pending(void)
{
    return (s_pi.intsr & s_pi.intmr) != 0;
}

void pi_set_interrupt(PIInterrupt which, int asserted)
{
    u32 bit = 1u << (unsigned)which;

    if (asserted) {
        /* Count the RISING EDGE. On the console an interrupt is taken once per
         * assertion and the handler clears it; the port cannot rely on the
         * guest handler always clearing the line, so it delivers one interrupt
         * per assertion counted here instead of one per poll of a level that
         * may stay high. Delivering per poll ran the guest's IPC handler
         * several times for a single reply and walked its queue counters past
         * the requests it had actually enqueued. */
        u32 old = __atomic_fetch_or(&s_pi.intsr, bit, __ATOMIC_SEQ_CST);
        if (!(old & bit)) __atomic_add_fetch(&s_pi_raises, 1u, __ATOMIC_SEQ_CST);
    } else
        __atomic_fetch_and(&s_pi.intsr, ~bit, __ATOMIC_SEQ_CST);

    pi_update();
}

/* ------------------------------------------------------------------ */
/* MMIO                                                                 */
/* ------------------------------------------------------------------ */

static u32 pi_read(u32 addr, unsigned size, void *ctx)
{
    u32 off = addr - HW_PI_BASE;
    (void)size; (void)ctx;

    switch (off) {
    case PI_INTSR:     return s_pi.intsr;
    case PI_INTMR:     return s_pi.intmr;
    case PI_FIFO_BASE: return s_pi.fifo_base;
    case PI_FIFO_END:  return s_pi.fifo_end;
    case PI_FIFO_WPTR: return s_pi.fifo_wptr;
    case PI_RESET:     return s_pi.reset;
    case PI_REVISION:  return PI_REVISION_WII;
    default:
        LOG_WARN_ONCE(LOG_CORE, "PI: read from unmapped +%03x", off);
        return 0;
    }
}

static void pi_write(u32 addr, u32 value, unsigned size, void *ctx)
{
    u32 off = addr - HW_PI_BASE;
    (void)size; (void)ctx;

    switch (off) {
    case PI_INTSR:
        /* Write-one-to-clear. Writing the cause register does not *set*
         * anything -- only devices do that -- so a guest acknowledging an
         * interrupt cannot accidentally raise another. */
        __atomic_fetch_and(&s_pi.intsr, ~value, __ATOMIC_SEQ_CST);
        pi_update();
        return;

    case PI_INTMR:
        s_pi.intmr = value;
        pi_update();
        return;

    /* The FIFO window the write-gather pipe bursts into. Addresses are
     * physical and 32-byte aligned by construction. */
    case PI_FIFO_BASE: s_pi.fifo_base = value & ~31u; return;
    case PI_FIFO_END:  s_pi.fifo_end  = value & ~31u; return;
    case PI_FIFO_WPTR: s_pi.fifo_wptr = value & ~31u; return;

    case PI_RESET:
        s_pi.reset = value;
        if (value & 1u)
            LOG_INFO(LOG_CORE, "PI: guest requested system reset");
        return;

    default:
        LOG_WARN_ONCE(LOG_CORE, "PI: write to unmapped +%03x = %08x", off, value);
        return;
    }
}

/* ------------------------------------------------------------------ */

void pi_init(void)
{
    pi_reset();
    mmio_register(HW_PI_BASE, 0x100, pi_read, pi_write, NULL, "PI");
}

void pi_reset(void)
{
    s_pi.intsr = 0;
    s_pi.intmr = 0;
    s_pi.fifo_base = 0;
    s_pi.fifo_end = 0;
    s_pi.fifo_wptr = 0;
    s_pi.reset = 0;
}

/* The write-gather pipe bursts into this window.
 *
 * The registers live in PI because the *CPU* side of the ring is PI's -- the
 * GPU side (read pointer, distance, watermarks) belongs to CP. Two devices
 * addressing one ring from opposite ends is the actual hardware arrangement,
 * not a modelling artifact, and keeping each pointer with its owner is what
 * makes the GP-link behaviour fall out naturally instead of needing a special
 * case. */
void pi_fifo_window(u32 *base, u32 *end, u32 *wptr)
{
    *base = s_pi.fifo_base;
    *end  = s_pi.fifo_end;
    *wptr = s_pi.fifo_wptr;
}

void pi_fifo_set_write_pointer(u32 wptr)
{
    s_pi.fifo_wptr = wptr;
}

/* Called by hw_init; kept out of pi_init so the CPU pointer is established
 * before any device can signal. */
void pi_attach_cpu(PPCState *s)
{
    s_cpu = s;
}

/* The guest can enable interrupts with a device already asserting, so MSR
 * changes have to re-run the same decision. */
void pi_msr_changed(void)
{
    pi_update();
}

/* Raw cause and mask, for the port's interrupt diagnostics. Reading them
 * through the MMIO path would be indistinguishable from the guest doing so and
 * would clear-on-read where that applies. */
u32 pi_intsr_raw(void) { return s_pi.intsr; }
u32 pi_intmr_raw(void) { return s_pi.intmr; }
