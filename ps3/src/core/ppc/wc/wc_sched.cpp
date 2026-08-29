/* wc_sched.cpp -- the guest scheduler for the fiber build.
 *
 * SelectThread, ported 1:1 from the WiiCompiled runtime's fiber-aware HLE
 * (its own solution for this exact game, a sibling build) and cross-checked
 * line by line against the NTSC translated body func_801A9B68 -- they agree
 * on every offset and step. The one intentional divergence: the idle loop
 * pumps THIS port's event sources (PI interrupts, the decrementer) instead of
 * the runtime's VI/audio polls, because this port's device layer already
 * works and stays authoritative.
 *
 * Everything else about threading remains translated guest code. The HLE
 * boundary is exactly: SelectThread (the idle loop needs host code),
 * OSLoadContext (the stack switch), OSCreateThread/__OSThreadInit (fiber
 * bookkeeping). See docs/PORT_SCHEDULER_REDESIGN.md.
 */
extern "C" {
#include "../../../common/log.h"
}
#include "ppc_runtime.h"
#include "gen/wc_calls.h"
#include "memory.h"
#include "fiber.h"

#include <unistd.h>

extern "C" void func_801A9B68(CpuContext *);   /* SelectThread (translated)  */
extern "C" void func_801A1E38(CpuContext *);   /* OSSaveContext              */
extern "C" void func_801A1EB8(CpuContext *);   /* OSLoadContext              */
extern "C" void func_801A1DD0(CpuContext *);   /* OSSetCurrentContext        */
extern "C" void func_801A1FF8(CpuContext *);   /* OSClearContext             */
extern "C" void func_801A9DE4(CpuContext *);   /* OSCreateThread             */
extern "C" void func_801A957C(CpuContext *);   /* __OSThreadInit             */

extern "C" int  wcf_switch(uint32_t osctx, CpuContext *ctx);
extern "C" { extern volatile unsigned g_wcf_poison_save; }
extern "C" int  wcf_create(uint32_t osthread);
extern "C" int  wcf_register_root(uint32_t osthread, CpuContext *ctx);
extern "C" void wcf_purge(uint32_t osthread);
extern "C" void wcf_pump_force_clear(void);
extern "C" void wc_ios_drain_replies(CpuContext *);
extern "C" uint32_t wcf_current_osthread(void);

extern "C" void wc_irq_pump(CpuContext *ctx);  /* wc_os.cpp: PI + dec        */
extern "C" int  pi_interrupt_pending(void);
extern "C" unsigned int pi_intsr_raw(void);
extern "C" unsigned int pi_intmr_raw(void);
extern "C" int  wc_dec_due(void);

namespace {

/* NTSC-mkwii scheduler globals, extracted from the translated source
 * (r13 = 0x80388880) and verified live. */
enum : uint32_t {
    kDisableCount = 0x80382598u,   /* OSDisableScheduler nesting            */
    kPendingMask  = 0x803825A0u,   /* run-queue pending bitmask             */
    kReschedFlag  = 0x8038259Cu,   /* reschedule counter                    */
    kSwitchCbPtr  = 0x80381760u,   /* switch-thread callback pointer        */
    kRunQueueBase = 0x80343430u,   /* 32 x {head, tail}                     */
    kIdleCtx      = 0x80343530u,
    kCurCtxAddr   = 0x800000D4u,
    kRunCtxAddr   = 0x800000E4u,
    kOffState     = 0x2C8u,        /* u16 */
    kOffPrio      = 0x2D0u,
    kOffQueue     = 0x2DCu,
    kOffNext      = 0x2E0u,
    kOffPrev      = 0x2E4u,
    kOffMode      = 0x1A2u,        /* u16, OSContext state flags            */
    kOffSrr1      = 0x19Cu,
};

inline uint32_t rd32(uint32_t a)            { return MemoryInline::Load<uint32_t>(a); }
inline void     wr32(uint32_t a, uint32_t v){ MemoryInline::Store<uint32_t>(a, v); }
inline uint16_t rd16(uint32_t a)            { return MemoryInline::Load<uint16_t>(a); }
inline void     wr16(uint32_t a, uint16_t v){ MemoryInline::Store<uint16_t>(a, v); }

/* Decision counters: defined in BOTH builds (the fibers report links them
 * unconditionally); only the fiber scheduler increments them. */
extern "C" {
volatile unsigned g_wcs_early0, g_wcs_save1, g_wcs_idle,
                  g_wcs_self, g_wcs_sw, g_wcs_skipsave;
}

#ifdef WC_FIBER_SCHED

void invoke_switch_callback(CpuContext *ctx, uint32_t oldctx, uint32_t newctx)
{
    uint32_t cb = rd32(kSwitchCbPtr);
    if (!cb) return;
    ctx->gpr[3] = oldctx;
    ctx->gpr[4] = newctx;
    InvokeIndirectCpu(cb, ctx);
}

/* Tail-enqueue on the run queue for `prio` and publish the pending bit --
 * the translated loc_801A9BE8..C54 sequence, exactly. */
void requeue_running(uint32_t thread, uint32_t prio)
{
    uint32_t q = kRunQueueBase + prio * 8u;
    uint32_t tail = rd32(q + 4u);
    if (tail == 0) wr32(q, thread);
    else           wr32(tail + kOffNext, thread);
    wr32(thread + kOffPrev, tail);
    wr32(thread + kOffNext, 0);
    wr32(q + 4u, thread);
    wr32(thread + kOffQueue, q);
    wr32(kPendingMask, rd32(kPendingMask) | (1u << (31u - prio)));
    wr32(kReschedFlag, 1u);
}

#endif /* WC_FIBER_SCHED */

} /* namespace */

extern "C" {

/* The stack switch OSLoadContext asks for. Self-load restores in place (the
 * rfi / immediate-rewake); cross-load switches fibers; a context without a
 * fiber (pre-ThreadInit default context: the same underlying thread) runs
 * the translated load in place. Loading the idle context restores nothing --
 * it happens only as a handler's rfi inside the idle pump, which resumes
 * pumping when the hook returns. */
void wcf_hle_OSLoadContext(CpuContext *ctx)
{
    uint32_t osctx = ctx->gpr[3];
    if (!osctx) return;
    if (osctx == kIdleCtx) return;
    int r = wcf_switch(osctx, ctx);
    if (r < 0) {
        static unsigned n;
        if (n < 8u) { n++;
            LOG_WARN(LOG_CORE, "WCF: in-place load of unregistered ctx %08x",
                     (unsigned)osctx); }
        func_801A1EB8(ctx);
        {   uint32_t s1 = rd32(osctx + kOffSrr1);
            if (s1) ctx->msr = s1;
        }
    }
}

void wcf_hle_OSCreateThread(CpuContext *ctx)
{
    uint32_t thread = ctx->gpr[3];             /* r3 AT ENTRY: the OSThread */
    func_801A9DE4(ctx);                        /* the SDK's own creation    */
    if (thread && ctx->gpr[3])                 /* r3 after: BOOL success    */
        wcf_create(thread);
}

void wcf_hle_OSThreadInit(CpuContext *ctx)
{
    func_801A957C(ctx);                        /* the SDK's own init        */
    uint32_t cur = rd32(kRunCtxAddr);
    if (!cur) {
        LOG_ERROR(LOG_CORE, "WCF: __OSThreadInit left no current thread");
        return;
    }
    /* The adopted boot thread's srr1 is zero (nothing wrote it); seed EE|FP
     * exactly as OSInitContext would, so resumes restore a sane MSR. */
    if (!rd32(cur + kOffSrr1))
        wr32(cur + kOffSrr1, 0x0000A000u);
    wcf_register_root(cur, ctx);
}

/* SelectThread. Non-fiber builds forward to the translated body -- this
 * symbol is bound at 0x801A9B68 in both builds. */
void wc_hle_SelectThread(CpuContext *ctx)
{
#ifndef WC_FIBER_SCHED
    func_801A9B68(ctx);
#else
    uint32_t yield = ctx->gpr[3];

    wc_ios_drain_replies(ctx);

    if (rd32(kDisableCount) != 0) { g_wcs_early0++; ctx->gpr[3] = 0; return; }

    uint32_t curctx = rd32(kCurCtxAddr);
    uint32_t runctx = rd32(kRunCtxAddr);
    if (curctx == 0 && runctx == 0) { ctx->gpr[3] = 0; return; }
    if (curctx != runctx)           { ctx->gpr[3] = 0; return; }

    if (runctx != 0) {
        uint16_t state = rd16(runctx + kOffState);
        if (state == 2u) {                                   /* RUNNING */
            if (!yield) {
                uint32_t prio = rd32(runctx + kOffPrio);
                uint32_t pend = rd32(kPendingMask);
                uint32_t clz  = pend ? (uint32_t)__builtin_clz(pend) : 32u;
                if ((int32_t)prio <= (int32_t)clz) { ctx->gpr[3] = 0; return; }
            }
            wr16(runctx + kOffState, 1u);                    /* READY */
            requeue_running(runctx, rd32(runctx + kOffPrio));
        }
        /* Not preemptible-marked: save, and when the save "returns 1" this
         * thread has been resumed -- SelectThread returns 0 to its caller
         * with the caller's world restored. The translated OSSaveContext
         * stores r3=1 and SRR0 into the context; our resume protocol runs
         * the translated load body, so gpr[3]==1 lands here. */
        if (!(rd16(runctx + kOffMode) & 2u)) {
            ctx->gpr[3] = runctx;
            func_801A1E38(ctx);
            if (ctx->gpr[3] != 0) { g_wcs_save1++; ctx->gpr[3] = 0; return; }
            {   /* poison bracket, save side: the mirror we just archived */
                uint32_t r1v = (uint32_t)ctx->gpr[1], r31v = (uint32_t)ctx->gpr[31];
                if (r1v < 0x80000000u || r1v >= 0x81800000u) {
                    g_wcf_poison_save++;
                    static unsigned n;
                    if (n < 8u) { n++;
                        LOG_WARN(LOG_CORE, "POISONSAVE[%u] run=%08x r1=%08x r31=%08x lr=%08x",
                                 n, (unsigned)runctx, r1v, r31v, (unsigned)ctx->lr); } }
            }
        } else {
            g_wcs_skipsave++;
            {   static unsigned n;
                if (n < 8u) { n++;
                    LOG_WARN(LOG_CORE, "WCS: save SKIPPED run=%08x mode=%04x st=%u",
                             (unsigned)runctx, rd16(runctx + kOffMode),
                             rd16(runctx + kOffState)); } }
        }
    }

    for (;;) {
        uint32_t pend = rd32(kPendingMask);
        if (pend == 0) {
            /* Hardware's idle loop, as host code: current context becomes
             * the idle context, EE opens, and the CPU takes interrupts until
             * something is runnable. Handlers run as nested calls right
             * here on the current fiber; one that reschedules switches away
             * mid-handler and this loop continues when we are resumed. */
            invoke_switch_callback(ctx, rd32(kRunCtxAddr), 0);
            wr32(kRunCtxAddr, 0);
            ctx->gpr[3] = kIdleCtx;
            func_801A1DD0(ctx);                /* OSSetCurrentContext(idle) */
            g_wcs_idle++;
            wcf_pump_force_clear();
            {   uint32_t saved_msr = ctx->msr;
                ctx->msr |= 0x8000u;           /* EE on, as the spin does   */
                {   extern volatile int g_host_site;
                    g_host_site = 1;
                }
                while (rd32(kPendingMask) == 0) {
                    if ((pi_intsr_raw() & pi_intmr_raw()) || wc_dec_due())
                        wc_irq_pump(ctx);
                    if (rd32(kPendingMask) != 0) break;
                    usleep(200);
                }
                {   extern volatile int g_host_site;
                    g_host_site = 0;
                }
                /* The translated spin exits through OSDisableInterrupts:
                 * EE off, everything else as it was. */
                ctx->msr = saved_msr & ~0x8000u;
            }
            ctx->gpr[3] = kIdleCtx;
            func_801A1FF8(ctx);                /* OSClearContext(idle)      */
            pend = rd32(kPendingMask);
            if (pend == 0) continue;
        }

        wr32(kReschedFlag, 0);

        uint32_t prio = (uint32_t)__builtin_clz(pend);
        uint32_t q    = kRunQueueBase + prio * 8u;
        uint32_t head = rd32(q);
        if (head == 0) {
            /* Pending bit with an empty queue: clear and rescan (runtime's
             * safety; never observed on hardware-faithful state). */
            LOG_WARN_ONCE(LOG_CORE, "WCF: pending bit %u with empty queue", prio);
            wr32(kPendingMask, pend & ~(1u << (31u - prio)));
            continue;
        }

        {   uint16_t st = rd16(head + kOffState);
            if (st == 0u || st == 8u) {        /* dead thread on a run queue */
                uint32_t nxt = rd32(head + kOffNext);
                if (nxt) wr32(nxt + kOffPrev, 0);
                else     wr32(q + 4u, 0);
                wr32(q, nxt);
                if (!nxt) wr32(kPendingMask, rd32(kPendingMask) & ~(1u << (31u - prio)));
                wr32(head + kOffQueue, 0);
                wcf_purge(head);
                continue;
            }
        }

        /* Pop the head. */
        {   uint32_t nxt = rd32(head + kOffNext);
            if (nxt) wr32(nxt + kOffPrev, 0);
            else     wr32(q + 4u, 0);
            wr32(q, nxt);
            if (!nxt) wr32(kPendingMask, rd32(kPendingMask) & ~(1u << (31u - prio)));
        }
        wr32(head + kOffQueue, 0);
        wr16(head + kOffState, 2u);            /* RUNNING */

        invoke_switch_callback(ctx, rd32(kRunCtxAddr), head);
        wr32(kRunCtxAddr, head);
        ctx->gpr[3] = head;
        func_801A1DD0(ctx);                    /* OSSetCurrentContext(next) */

        {   static unsigned n;
            if (n < 12u) { n++;
                LOG_WARN(LOG_CORE, "WCS: pick[%u] head=%08x prio=%u cur=%08x",
                         n, (unsigned)head, prio, (unsigned)runctx); } }
        if (wcf_current_osthread() == head) g_wcs_self++; else g_wcs_sw++;
        ctx->gpr[3] = head;
    {   static unsigned s_npick;
        if (s_npick < 14u) { s_npick++;
            LOG_WARN(LOG_CORE, "PICK[%u] os=%08x prio=%u hint=%08x",
                     s_npick, (unsigned)ctx->gpr[3],
                     (unsigned)MemoryInline::Load<uint32_t>(ctx->gpr[3] + 0x2D0u),
                     0u); } }

        wcf_hle_OSLoadContext(ctx);            /* the switch (or self-load) */

        ctx->gpr[3] = head;
        return;
    }
#endif /* WC_FIBER_SCHED */
}

} /* extern "C" */
