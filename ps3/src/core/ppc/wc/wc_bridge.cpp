/* wc_bridge.cpp -- the hybrid runtime's two crossings.
 *
 * The whole-game native link measures 154 MB against an ~85 MB image budget
 * (the console gives the process ~214 MB and guest RAM + arenas hold ~130),
 * so StaticR.rel cannot ship fully native. The game itself loads the REL
 * into guest RAM; everything needed to EXECUTE guest bytes -- the JIT, the
 * interpreter, the device model -- is already resident. So:
 *
 *   native -> jit   An indirect call whose target is not in the linked set
 *                   (WcUnresolvedCall) runs under the JIT until the guest
 *                   returns. The return is caught by an AOT entry at a
 *                   sentinel address planted in LR: hardware-exact semantics,
 *                   since AOT dispatch happens at every block boundary.
 *
 *   jit -> native   Inside a bridge stint, every dispatch first consults the
 *                   linked table (wc_table_lookup): DOL calls made by REL
 *                   code land back in the translated natives -- and, above
 *                   all, in the 49 HLE overrides. Without this the JIT would
 *                   execute the guest's own IPC/DVD/OS code against a device
 *                   model that HLEs them, and diverge.
 *
 * Interrupts: between slices the mirror is synchronized and wc_irq_poll runs
 * exactly as at a native call boundary -- deliveries, mid-handler parks and
 * fiber switches all work unchanged, because they only ever see the mirror.
 *
 * Paired singles cross these boundaries as ps0 scalars, the same contract
 * the native call layer already has (CpuContext carries scalar FPRs).
 */
#include <cstring>
extern "C" {
#include "../../../common/log.h"
#include "../gekko.h"
#include "../jit/jit.h"
}
#include "ppc_runtime.h"
#include "memory.h"

extern "C" u64  timing_timebase(void);
extern "C" void interp_run(PPCState *);

#define WC_JIT_SENTINEL 0x7E000000u

extern "C" { int g_wc_bridge_depth = 0; }
static int s_bridge_ready = 0;
extern "C" { volatile unsigned g_wc_bridge_calls, g_wc_bridge_slices; }

static u32 aot_bridge_return(PPCState *s)
{
    s->exit_requested = 1;          /* pc stays at the sentinel: the flag */
    return 0;
}

static void st_from_ctx(PPCState *s, const CpuContext *c)
{
    for (unsigned i = 0; i < 32; i++) s->gpr[i] = c->gpr[i];
    s->cr = c->cr; s->lr = c->lr; s->ctr = c->ctr;
    ppc_set_xer(s, c->xer);
    s->fpscr = c->fpscr; s->msr = c->msr; s->hid2 = c->hid2;
    for (unsigned i = 0; i < 8; i++) s->gqr[i] = c->gqr[i];
    for (unsigned i = 0; i < 32; i++) s->ps[i].ps0.u = c->fpr[i].raw;
}

static void ctx_from_st(CpuContext *c, const PPCState *s)
{
    for (unsigned i = 0; i < 32; i++) c->gpr[i] = s->gpr[i];
    c->cr = s->cr; c->lr = s->lr; c->ctr = s->ctr;
    c->xer = ppc_get_xer(s);
    c->fpscr = s->fpscr; c->msr = s->msr; c->hid2 = s->hid2;
    c->pc = s->pc;
    for (unsigned i = 0; i < 8; i++) c->gqr[i] = s->gqr[i];
    for (unsigned i = 0; i < 32; i++) c->fpr[i].raw = s->ps[i].ps0.u;
}

/* jit -> native: dispatch hook, called from jit_dispatch_c inside a stint. */
extern "C" void (*wc_table_lookup(uint32_t target))(CpuContext *);

extern "C" int wc_native_dispatch(PPCState *s)
{
    void (*fn)(CpuContext *) = wc_table_lookup(s->pc);
    if (!fn) return 0;
    CpuContext c;
    std::memset(&c, 0, sizeof c);
    ctx_from_st(&c, s);
    c.srr0 = s->pc; c.srr1 = s->msr;
    u32 retpc = s->lr;
    CpuContext *saved = wc_current_ctx;
    wc_current_ctx = &c;
    fn(&c);
    wc_current_ctx = saved;
    u32 pc_keep = retpc & ~3u;
    st_from_ctx(s, &c);
    s->pc = pc_keep;
    s->downcount -= (s32)c.insn_count;
    if (s->downcount < 0) s->downcount = 0;
    return 1;
}

/* native -> jit: run guest code at `target` until it returns. Re-entrant. */
extern "C" void wc_jit_bridge(uint32_t target, CpuContext *ctx)
{
    if (!s_bridge_ready || !ctx) return;          /* pre-init: old skip */
    g_wc_bridge_calls++;
    PPCState s;
    std::memset(&s, 0, sizeof s);
    st_from_ctx(&s, ctx);
    s.pc = target;
    s.lr = WC_JIT_SENTINEL;
    u32 saved_lr = ctx->lr;
    g_wc_bridge_depth++;
    for (;;) {
        s.downcount = 50000;
        s.exit_slack = 0;
        s.exit_requested = 0;
        s.tb = timing_timebase();
        jit_run(&s);
        g_wc_bridge_slices++;
        if (s.pc == WC_JIT_SENTINEL) break;
        /* Slice edge: the same delivery point a native call boundary is.
         * The mirror is authoritative for delivery; sync, poll, resync. */
        ctx_from_st(ctx, &s);
        wc_irq_poll(ctx);
        st_from_ctx(&s, ctx);
    }
    g_wc_bridge_depth--;
    ctx_from_st(ctx, &s);
    ctx->lr = saved_lr;
    {   static unsigned n;
        if (n < 8u) { n++;
            LOG_WARN(LOG_CORE, "BRIDGE[%u] ran %08x r3=%08x slices~%u",
                     n, target, (unsigned)ctx->gpr[3], g_wc_bridge_slices); } }
}

extern "C" void wc_bridge_init(void)
{
    jit_aot_register_key(WC_JIT_SENTINEL, aot_bridge_return, 0, 0);
    jit_aot_enable_all();
    s_bridge_ready = 1;
    LOG_WARN(LOG_CORE, "WC: jit bridge armed (sentinel %08x)", WC_JIT_SENTINEL);
}
