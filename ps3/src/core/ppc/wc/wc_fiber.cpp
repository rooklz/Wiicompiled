/* wc_fiber.cpp -- guest-thread fibers for the native port.
 *
 * One host thread runs ALL guest threads; each guest OSThread owns a fiber
 * (a host stack + a FiberCtx). The guest's own scheduler decides who runs;
 * this file only performs the switch it asks for. Single-threaded by
 * construction -- every function here runs on the guest execution thread --
 * so there are no locks and no races, exactly like the hardware this mirrors.
 *
 * The resume protocol (shared by first runs and re-runs): a fiber that gains
 * the processor seeds gpr[3] with ITS OWN OSContext and runs the translated
 * OSLoadContext body, which restores the guest register mirror exactly as
 * hardware's rfi would; MSR comes from the context's SRR1. After that, a
 * resumed fiber simply unwinds its frozen host frames; a first-run fiber
 * invokes the entry from SRR0 and, if the entry returns, hands the return
 * value to the translated OSExitThread -- the system software itself retires the thread.
 */
#include "fiber.h"
extern "C" {
#include "../../../common/log.h"
}
#include "ppc_runtime.h"
#include "gen/wc_calls.h"
#include "memory.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>

extern "C" void func_801A1EB8(CpuContext *);   /* OSLoadContext (translated) */
extern "C" void func_801AA050(CpuContext *);   /* OSExitThread  (translated) */
extern "C" void wc_note_guest_thread(void);
extern "C" int t_in_irq_probe(void);
extern "C" int g_wc_in_irq_probe_val(void);
extern "C" unsigned ipc_backlog_probe(void);

namespace {

enum {
    kMaxFibers   = 16,
    kFiberStack  = 128 * 1024,   /* twice the old per-thread host stacks    */
    kOsSrr0      = 0x198,
    kOsSrr1      = 0x19C,
};

struct GuestFiber {
    uint32_t  osthread;   /* guest OSThread == its OSContext address */
    FiberCtx  fc;
    char     *stack;      /* nullptr for the adopted root fiber      */
    int       used;
    int       started;    /* first run already happened              */
    int       pumping;    /* this fiber is inside an interrupt delivery */
};

GuestFiber  s_fibers[kMaxFibers];
int         s_cur = -1;              /* index of the running fiber      */
CpuContext *s_ctx;                   /* the one shared register mirror  */

int find_fiber(uint32_t osthread)
{
    for (int i = 0; i < kMaxFibers; i++)
        if (s_fibers[i].used && s_fibers[i].osthread == osthread) return i;
    return -1;
}

int alloc_fiber(uint32_t osthread)
{
    for (int i = 0; i < kMaxFibers; i++) {
        if (!s_fibers[i].used) {
            std::memset(&s_fibers[i], 0, sizeof s_fibers[i]);
            s_fibers[i].osthread = osthread;
            s_fibers[i].used = 1;
            return i;
        }
    }
    return -1;
}

/* Restore the guest register mirror for the running fiber's own context --
 * the tail half of every switch, identical for resumes and first runs. */
void load_own_context(void)
{
    uint32_t osctx = s_fibers[s_cur].osthread;
    s_ctx->gpr[3] = osctx;
    func_801A1EB8(s_ctx);
    {   uint32_t s1 = MemoryInline::Load<uint32_t>(osctx + kOsSrr1);
        if (s1) s_ctx->msr = s1;
    }
    /* Post-restore sanity: a context whose saved LR or SP is not a guest
     * address was stale or cleared when we restored it -- the exact ingress
     * of the lr=0x18 mirror poisoning. Name it at the moment it happens. */
    {   uint32_t lr = (uint32_t)s_ctx->lr, sp = (uint32_t)s_ctx->gpr[1];
        if ((lr && (lr < 0x80004000u || lr >= 0x80290000u)) ||
            sp < 0x80003000u || sp >= 0x94000000u) {
            static unsigned n;
            if (n < 8u) { n++;
                LOG_WARN(LOG_CORE, "WCF: STALE restore f%d osctx=%08x lr=%08x sp=%08x srr0=%08x",
                         s_cur, osctx, lr, sp,
                         MemoryInline::Load<uint32_t>(osctx + kOsSrr0)); }
        }
    }
}

/* First run of a created fiber: land, restore context, run the entry from
 * SRR0, and retire through the game's own OSExitThread if it ever returns. */
void fiber_main(void *arg)
{
    int me = (int)(intptr_t)arg;
    s_cur = me;
    s_fibers[me].started = 1;
    {   uint32_t osctx = s_fibers[me].osthread;
        uint32_t entry = MemoryInline::Load<uint32_t>(osctx + kOsSrr0);
        load_own_context();
        LOG_WARN(LOG_CORE, "WCF: fiber %d starts os=%08x entry=%08x",
                 me, (unsigned)osctx, (unsigned)entry);
        InvokeIndirectCpu(entry, s_ctx);
        /* Entry returned: r3 is the exit value; the translated OSExitThread
         * does the moribund/join/reschedule work and never comes back. */
        LOG_WARN(LOG_CORE, "WCF: fiber %d entry returned (r3=%08x), exiting",
                 me, (unsigned)s_ctx->gpr[3]);
        func_801AA050(s_ctx);
    }
    LOG_ERROR(LOG_CORE, "WCF: OSExitThread returned on fiber %d", me);
    for (;;) { /* unreachable; fiber_boot traps if fiber_main returns */
        break;
    }
}

} /* namespace */

void fiber_prime(FiberCtx *ctx, void *stack_base, size_t stack_size,
                 void (*entry)(void *), void *arg)
{
    std::memset(ctx, 0, sizeof *ctx);
    /* Top of stack, 16-aligned, with a null back chain and headroom for the
     * trampoline's frame. */
    uintptr_t top = ((uintptr_t)stack_base + stack_size - 256) & ~(uintptr_t)15;
    *(uint64_t *)top = 0;                       /* null back chain terminator */
    ctx->sp = (uint64_t)top;
    /* fiber_boot's CODE address comes from its descriptor (ELFv1: a function
     * symbol's C address IS the descriptor; word 0 is the entry point). */
    ctx->toc = ((const uint64_t *)(const void *)&fiber_boot)[1];
    ctx->lr  = ((const uint64_t *)(const void *)&fiber_boot)[0];
    ctx->gpr[0] = (uint64_t)(uintptr_t)(const void *)entry;  /* r14: descriptor */
    ctx->gpr[1] = (uint64_t)(uintptr_t)arg;                  /* r15: argument   */
}

extern "C" int wcf_register_root(uint32_t osthread, CpuContext *ctx)
{
    int i = find_fiber(osthread);
    if (i >= 0) return i;
    i = alloc_fiber(osthread);
    if (i < 0) return -1;
    s_fibers[i].started = 1;      /* it is running right now, on this stack */
    s_cur = i;
    s_ctx = ctx;
    wc_note_guest_thread();
    /* Host mode stays OFF: the synthesis that works is HLE'd sync-IPC +
     * DVDLow in-call completion + CLASSIC interrupt-driven async
     * completion through the guest IPC handler (which does its own
     * ProfReply bookkeeping, mailbox acks and reschedule -- the exact
     * fidelity BTE needs; both models completed Bluetooth this way). */
    LOG_WARN(LOG_CORE, "WCF: root fiber %d adopts os=%08x", i, (unsigned)osthread);
    return i;
}

extern "C" int wcf_create(uint32_t osthread)
{
    int i = find_fiber(osthread);
    if (i >= 0) {
        /* The game reuses OSThread storage; retire the stale fiber. */
        LOG_WARN(LOG_CORE, "WCF: recreate os=%08x (purging fiber %d)",
                 (unsigned)osthread, i);
        std::free(s_fibers[i].stack);
        s_fibers[i].used = 0;
    }
    i = alloc_fiber(osthread);
    if (i < 0) { LOG_ERROR(LOG_CORE, "WCF: out of fibers"); return -1; }
    s_fibers[i].stack = (char *)std::malloc(kFiberStack);
    if (!s_fibers[i].stack) { s_fibers[i].used = 0; return -1; }
    fiber_prime(&s_fibers[i].fc, s_fibers[i].stack, kFiberStack,
                fiber_main, (void *)(intptr_t)i);
    return i;
}

extern "C" void wcf_purge(uint32_t osthread)
{
    int i = find_fiber(osthread);
    if (i < 0 || i == s_cur) return;
    std::free(s_fibers[i].stack);
    s_fibers[i].used = 0;
}

extern "C" CpuContext *wcf_ctx(void) { return s_ctx; }
extern "C" void *wcf_ctx_raw(void) { return (void *)s_ctx; }
extern "C" unsigned wcf_ctx_gpr(int i) { return s_ctx ? (unsigned)s_ctx->gpr[i] : 0u; }
extern "C" unsigned wcf_ctx_lr(void)  { return s_ctx ? (unsigned)s_ctx->lr : 0u; }

extern "C" int wcf_pump_enter(void)
{
    if (s_cur < 0) return 1;                  /* no fiber system yet: allow */
    if (s_fibers[s_cur].pumping) {            /* nested on THIS fiber: skip */
        static unsigned n;
        if (n < 8u) { n++;
            LOG_WARN(LOG_CORE, "WCF: pump skipped, flag held by fiber %d (os=%08x)",
                     s_cur, (unsigned)s_fibers[s_cur].osthread); }
        return 0;
    }
    s_fibers[s_cur].pumping = 1;
    return 1;
}

/* Reaching the scheduler's IDLE LOOP means any delivery in flight on this
 * fiber has architecturally completed its interrupt work -- and the idle
 * loop is the only runnable context left, so a stuck flag here is a
 * deadlock by construction (measured: task fiber exits, parks in idle,
 * pump refused forever under an asserted VI line). Clear it. */
extern "C" void wcf_pump_force_clear(void)
{
    if (s_cur >= 0 && s_fibers[s_cur].pumping) {
        static unsigned n;
        if (n < 8u) { n++;
            LOG_WARN(LOG_CORE, "WCF: idle clears stuck pump flag of fiber %d", s_cur); }
        s_fibers[s_cur].pumping = 0;
    }
    /* Reaching the scheduler's idle branch means no delivery can still be
     * architecturally in flight on the RUNNING context; a leaked in-irq
     * state here starves every future delivery (measured composite: g=1,
     * a fiber parked mid-pump, mirror poisoned, machine asleep). */
    {   extern void wc_irq_state_force_clear(void);
        wc_irq_state_force_clear();
    }
}

extern "C" void wcf_pump_exit(void)
{
    if (s_cur >= 0) s_fibers[s_cur].pumping = 0;
}

extern "C" uint32_t wcf_fiber_osthread(int i)
{
    return (i >= 0 && i < kMaxFibers && s_fibers[i].used)
         ? s_fibers[i].osthread : 0u;
}

extern "C" uint32_t wcf_current_osthread(void)
{
    return (s_cur >= 0) ? s_fibers[s_cur].osthread : 0u;
}

/* The switch OSLoadContext asks for. Returns 1 for a self-load (context
 * restored in place, no stack switch), 0 after a cross-switch has completed
 * and this fiber has been resumed, -1 for a context that has no fiber (the
 * caller runs the translated load in place -- correct for the pre-ThreadInit
 * default context, which is the same underlying thread continuing). */
extern "C" int wcf_switch(uint32_t osctx, CpuContext *ctx)
{
    if (!s_ctx) s_ctx = ctx;
    int to = find_fiber(osctx);
    if (to < 0) return -1;
    if (to == s_cur) {
        load_own_context();
        return 1;
    }
    int from = s_cur;
    s_cur = to;
    fiber_swap(&s_fibers[from].fc, &s_fibers[to].fc);
    /* Resumed: someone loaded OUR context. s_cur was set back to `from` by
     * the switch that resumed us (below), or by fiber_main for first runs. */
    s_cur = from;
    load_own_context();
    return 0;
}

/* Diagnostics for the rescue listener. */
extern "C" int wcf_report(char *out, int cap)
{
    extern volatile unsigned g_wcs_early0, g_wcs_save1, g_wcs_idle,
                             g_wcs_self, g_wcs_sw, g_wcs_skipsave;
    extern volatile unsigned g_mp_calls, g_mp_line0, g_mp_thread, g_mp_ee0, g_mp_hit;
    extern volatile int g_host_site;
    extern volatile unsigned g_ipc_backlog, g_ipc_pushed, g_drain_total;
    extern volatile unsigned g_bdrain_hits;
    extern volatile unsigned g_bta_tick_n;
    int used = 0;
    {
        used += snprintf(out + used, (size_t)(cap - used),
                     "ipc: backlog=%u pushed=%u drained=%u bhits=%u in_irq(t)=%d g=%d\n",
                     ipc_backlog_probe(), g_ipc_pushed, g_drain_total,
                     g_bdrain_hits, t_in_irq_probe(), g_wc_in_irq_probe_val());
    {   extern volatile unsigned g_wc_gki_send_n, g_wc_gki_read_n;
        used += snprintf(out + used, (size_t)(cap - used),
                     "bta_ticks=%u gki_send=%u gki_read=%u\n",
                     g_bta_tick_n, g_wc_gki_send_n, g_wc_gki_read_n);
    }
    }
    used += snprintf(out + used, (size_t)(cap - used),
                     "host_site=%d (0=guest 1=idle 2=irqwait 3=ipc2sync 4=ipc2cb)\n",
                     g_host_site);
    used += snprintf(out + used, (size_t)(cap - used),
                     "mmio-pump: calls=%u line0=%u thread=%u ee0=%u hit=%u\n",
                     g_mp_calls, g_mp_line0, g_mp_thread, g_mp_ee0, g_mp_hit);
    used += snprintf(out + used, (size_t)(cap - used),
                     "sched: early0=%u save1=%u idle=%u self=%u sw=%u skipsave=%u\n",
                     g_wcs_early0, g_wcs_save1, g_wcs_idle,
                     g_wcs_self, g_wcs_sw, g_wcs_skipsave);
    used += snprintf(out + used, (size_t)(cap - used),
                     "fibers: cur=%d os=%08x\n", s_cur,
                     (unsigned)((s_cur >= 0) ? s_fibers[s_cur].osthread : 0u));
    for (int i = 0; i < kMaxFibers; i++) {
        if (!s_fibers[i].used) continue;
        used += snprintf(out + used, (size_t)(cap - used),
                         "  f%d os=%08x started=%d root=%d pump=%d\n", i,
                         (unsigned)s_fibers[i].osthread, s_fibers[i].started,
                         s_fibers[i].stack == nullptr, s_fibers[i].pumping);
    }
    return used;
}
