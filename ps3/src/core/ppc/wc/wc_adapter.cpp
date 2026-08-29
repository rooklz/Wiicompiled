/* wc_adapter.cpp -- run WiiCompiled-translated guest functions from the
 * emulator's AOT dispatcher.
 *
 * The dispatcher's contract (tools/rec/aot_fns.c): entered with s->pc at the
 * guest function's entry, execute the whole function natively, leave s->pc at
 * the guest return target, return the number of guest instructions retired.
 * A translated function instead takes a CpuContext by pointer, reads the
 * registers its ABI trailer names, writes back what it changed, and returns
 * as C -- the guest `blr` is the C return. The adapter is the bridge, and it
 * is deliberately dumb: every GPR and SPR is copied both ways. Forty loads
 * and stores are nothing next to a function body that runs thousands of
 * instructions, and the ABI masks can trim it once the number is in. */
extern "C" {
#include "../../../common/log.h"
#include "../../hw/gx_fifo.h"
#include "../gekko.h"
#include "../../mem/memmap.h"
}
#include "ppc_runtime.h"
#include "memory.h"
#include "wc_ps.h"
#include <cstring>

/* Context of the translated function currently running, for the C entry
 * points (PPC_PsqL/PsqSt) that take a GQR index rather than a context. */
/* Deliberately a plain global, NOT __thread.
 *
 * Only the quantised paired-single paths read it, for the GQR the instruction
 * names. Under the one-runner model exactly one guest thread is running, and
 * only the running thread writes it, so those readers always see the context
 * whose code is executing.
 *
 * The interrupt thread is the one host thread that runs guest code without
 * owning the runner, and it deliberately does NOT write this pointer: it only
 * ever runs while the guest sits in its idle loop, which executes no quantised
 * access at all, and the GQRs it would read are the ones the system software programmed at
 * init and never changes per thread. Leaving the pointer alone therefore keeps
 * the handler correct and keeps the game thread's pointer valid underneath it.
 *
 * (Making it __thread is the textbook answer and it does not fit: the symbol's
 * storage class has to match in all 13,675 translated objects, so it cannot be
 * changed without recompiling every one of them.) */
CpuContext* wc_current_ctx = nullptr;

extern "C" void func_801B5AD4(CpuContext* ctx);      /* __THPHuffDecodeDCTCompY */

static void wc_context_in(CpuContext &c, const PPCState *s)
{
    for (unsigned i = 0; i < 32; i++) c.gpr[i] = s->gpr[i];
    c.cr = s->cr; c.lr = s->lr; c.ctr = s->ctr;
    c.xer = ppc_get_xer(s);
    c.fpscr = s->fpscr; c.pc = s->pc; c.msr = s->msr; c.hid2 = s->hid2;
    for (unsigned i = 0; i < 8; i++) c.gqr[i] = s->gqr[i];
    /* Scalar representation of every FPR: ps0 as the double it is. A
     * function whose trailer names FPRs will need the paired form for the
     * registers it treats as pairs; none of the functions wired so far do. */
    for (unsigned i = 0; i < 32; i++) c.fpr[i].raw = s->ps[i].ps0.u;
    c.insn_count = 0;
}

static void wc_context_out(PPCState *s, const CpuContext &c)
{
    for (unsigned i = 0; i < 32; i++) s->gpr[i] = c.gpr[i];
    s->cr = c.cr; s->lr = c.lr; s->ctr = c.ctr;
    ppc_set_xer(s, c.xer);
    for (unsigned i = 0; i < 32; i++) s->ps[i].ps0.u = c.fpr[i].raw;
}

extern "C" u32 aot_wc_801b5ad4(PPCState *s)
{
    CpuContext c;
    wc_context_in(c, s);
    wc_current_ctx = &c;
    func_801B5AD4(&c);
    wc_current_ctx = nullptr;
    wc_context_out(s, c);
    s->pc = s->lr & ~3u;
    return c.insn_count;
}

/* The arena base the translated code indexes off, published once. In the
 * emulator build the JIT is already using it; a standalone port sets it from
 * its own allocation. */
uint8_t *g_wc_arena;

extern "C" void wc_memory_init(void)
{
    g_wc_arena = (uint8_t *)mem_base();
}

/* dcbz: zero one 32-byte Gekko cache line. */
extern "C" int32_t memset_zero_32(int32_t address)
{
    u32 ea = (u32)address & ~31u;
    if (g_wc_arena && !MemoryInline::IsMmio(ea))
        std::memset(g_wc_arena + MemoryInline::Fold(ea), 0, 32);
    else
        for (unsigned i = 0; i < 32; i += 4) mem_write32(ea + i, 0);
    return 0;
}

/* A computed branch whose target is not a translated function.
 *
 * Two causes, and they need different answers. A target that is a function
 * START simply was not discovered: the recursive walk follows direct calls and
 * the symbol map, and a function only ever reached through a pointer appears in
 * neither. A target INSIDE a function is a jump table -- the guest computed a
 * label within its own body -- and C cannot jump into the middle of another
 * function, so that one has to become its own entry point and be translated
 * from there.
 *
 * Both are fixed the same way: feed the address back to the translator as a
 * discovery seed. So rather than dying on the first one, every distinct target
 * is recorded and reported, and one run collects the whole set for a single
 * re-translation. Execution cannot meaningfully continue past one -- the guest
 * called something and got nothing -- so the run stops, loudly, rather than
 * carrying on with corrupt state and blaming the corruption later. */
#define WC_MAX_UNRESOLVED 256
static u32 s_unresolved[WC_MAX_UNRESOLVED];
static unsigned s_unresolved_n;

extern "C" void wc_report_unresolved(void)
{
    unsigned i;
    if (!s_unresolved_n) return;
    LOG_WARN(LOG_JIT, "WC: %u distinct untranslated indirect target(s):",
             s_unresolved_n);
    for (i = 0; i < s_unresolved_n; i++)
        LOG_WARN(LOG_JIT, "WC:   %08x", s_unresolved[i]);
}

void WcUnresolvedCall(uint32_t target, CpuContext *ctx)
{
    unsigned i;
    (void)ctx;
    for (i = 0; i < s_unresolved_n; i++)
        if (s_unresolved[i] == target) return;
    if (s_unresolved_n < WC_MAX_UNRESOLVED)
        s_unresolved[s_unresolved_n++] = target;
    LOG_WARN(LOG_JIT, "WC: indirect target %08x is not translated", target);
}


/* ------------------------------------------------------------------ */
/* GX bridge                                                            */
/*                                                                      */
/* Translated GX code does not touch hardware: it builds display lists   */
/* by writing through these five hooks, which on real hardware is the    */
/* write-gather pipe. We already have a parser for exactly that stream   */
/* (src/core/hw/gx_fifo.c -> src/core/gx/gx_parse.c) driving a working   */
/* RSX backend, so the whole GX surface -- 139 of the 256 native entry   */
/* points -- comes up by forwarding these, instead of reimplementing     */
/* GXBegin/GXSetBlendMode/... one by one against libgcm.                 */
/*                                                                      */
/* The parser wants the CPU whose downcount and interrupt state a FIFO   */
/* write can affect. In the hybrid build that is the live emulator CPU.  */
/* ------------------------------------------------------------------ */
extern "C" {

static PPCState *wc_fifo_cpu(void)
{
    extern PPCState *g_live_cpu;
    return g_live_cpu;
}

void GX_HLE_FIFO_Write8(uint8_t v)
{
    PPCState *s = wc_fifo_cpu();
    if (s) gxfifo_gather_write(s, 0, v, 1);
}
void GX_HLE_FIFO_Write16(uint16_t v)
{
    PPCState *s = wc_fifo_cpu();
    if (s) gxfifo_gather_write(s, 0, v, 2);
}
void GX_HLE_FIFO_Write32(uint32_t v)
{
    PPCState *s = wc_fifo_cpu();
    if (s) gxfifo_gather_write(s, 0, v, 4);
}
void GX_HLE_FIFO_WriteFloat(float v)
{
    uint32_t bits;
    std::memcpy(&bits, &v, sizeof bits);
    GX_HLE_FIFO_Write32(bits);
}
/* A burst is the vertex data behind a GXBegin: byte-for-byte the same stream
 * the gather pipe would have carried, so it goes through the same path rather
 * than a second one that could drift from it. */
void GX_HLE_FIFO_WriteBurst(const uint8_t *data, uint32_t sizeBytes)
{
    PPCState *s = wc_fifo_cpu();
    uint32_t i = 0;
    if (!s) return;
    for (; i + 4u <= sizeBytes; i += 4u) {
        uint32_t w = ((uint32_t)data[i] << 24) | ((uint32_t)data[i+1] << 16) |
                     ((uint32_t)data[i+2] << 8) | (uint32_t)data[i+3];
        gxfifo_gather_write(s, 0, w, 4);
    }
    for (; i < sizeBytes; i++)
        gxfifo_gather_write(s, 0, data[i], 1);
}

} /* extern "C" */

/* Instructions the translator could not model. Loud: a silent wrong answer is
 * worse than a stop. */
void PPC_Undefined(uint32_t pc, uint32_t raw, const char *details)
{
    LOG_ERROR(LOG_JIT, "WC: undefined guest instruction pc=%08x raw=%08x %s",
              pc, raw, details ? details : "");
    for (;;) { }
}

/* Registration, behind its own boot flag so an A/B can hold everything
 * else constant (see main.c). */
extern "C" void jit_aot_register(u32 guest_pc, u32 (*fn)(PPCState *));
extern "C" void wc_register_all(void)
{
    wc_memory_init();
    jit_aot_register(0x801b5ad4u, aot_wc_801b5ad4);
}
