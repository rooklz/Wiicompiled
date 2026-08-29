/* wc_hle_os.cpp -- native implementations of guest OS entry points.
 *
 * These replace translated guest functions at specific addresses, bound in
 * gen/wc_calls.cpp through the override map. A function is only worth
 * replacing for one of two reasons: the translated version cannot work
 * (anything that switches PowerPC context, because after static recompilation
 * the guest's registers are C locals), or the host can do the job far better
 * (diagnostics, cache maintenance).
 *
 * Everything else stays translated. The system software talks to hardware through loads
 * and stores, and the device model answers -- so most of it needs no help.
 */
extern "C" {
#include "../../../common/log.h"
#include "../gekko.h"
#include "../../mem/memmap.h"
}
#include "ppc_runtime.h"
#include "memory.h"
#include "gen/wc_calls.h"
#include <unistd.h>
#include <cstring>
#include <cstdio>

namespace {

/* Read a NUL-terminated guest string into a host buffer. Bounded: a guest
 * pointer at this stage of bring-up is as likely to be wrong as right, and a
 * diagnostic path must never be the thing that crashes. */
unsigned guest_str(uint32_t addr, char *out, unsigned cap)
{
    unsigned n = 0;
    while (n + 1 < cap) {
        uint8_t c = MemoryInline::FlatRead8(addr + n);
        if (!c) break;
        out[n++] = (char)c;
    }
    out[n] = 0;
    return n;
}

/* The guest's printf, with its arguments where the PowerPC ABI puts them:
 * r3 is the format, r4-r10 the remaining integer arguments, f1-f8 the floating
 * ones. Only the conversions MKWii's own diagnostics use are handled; anything
 * else is emitted literally so a message is never silently dropped. */
void format_report(CpuContext *ctx, char *out, unsigned cap)
{
    char fmt[512];
    unsigned oi = 0, gi = 4, fi = 1, i;
    unsigned n = guest_str(ctx->gpr[3], fmt, sizeof fmt);

    for (i = 0; i < n && oi + 1 < cap; i++) {
        if (fmt[i] != '%') { out[oi++] = fmt[i]; continue; }
        if (i + 1 >= n) break;
        /* Skip flags, width and precision; MKWii's messages use plain forms. */
        {   unsigned j = i + 1;
            char conv;
            while (j < n && (fmt[j] == '-' || fmt[j] == '+' || fmt[j] == ' ' ||
                             fmt[j] == '#' || fmt[j] == '0' || fmt[j] == '.' ||
                             (fmt[j] >= '1' && fmt[j] <= '9') || fmt[j] == 'l')) j++;
            if (j >= n) break;
            conv = fmt[j];
            char tmp[128];
            tmp[0] = 0;
            switch (conv) {
            case '%': tmp[0] = '%'; tmp[1] = 0; break;
            case 'd': case 'i':
                snprintf(tmp, sizeof tmp, "%d", (int)(gi < 11 ? ctx->gpr[gi++] : 0)); break;
            case 'u':
                snprintf(tmp, sizeof tmp, "%u", (unsigned)(gi < 11 ? ctx->gpr[gi++] : 0)); break;
            case 'x': case 'X': case 'p':
                snprintf(tmp, sizeof tmp, "%08x", (unsigned)(gi < 11 ? ctx->gpr[gi++] : 0)); break;
            case 'c':
                tmp[0] = (char)(gi < 11 ? ctx->gpr[gi++] : '?'); tmp[1] = 0; break;
            case 's': {
                char sbuf[192];
                guest_str(gi < 11 ? ctx->gpr[gi++] : 0, sbuf, sizeof sbuf);
                snprintf(tmp, sizeof tmp, "%s", sbuf);
                break;
            }
            case 'f': case 'g': case 'e': {
                double v = (fi < 9) ? ctx->fpr[fi++].d : 0.0;
                snprintf(tmp, sizeof tmp, "%g", v);
                break;
            }
            default:
                snprintf(tmp, sizeof tmp, "%%%c", conv); break;
            }
            {   unsigned k = 0;
                while (tmp[k] && oi + 1 < cap) out[oi++] = tmp[k++];
            }
            i = j;
        }
    }
    out[oi] = 0;
}

} /* namespace */

extern "C" {

/* OSReport: the game's own diagnostics. Translated, this writes to a debug
 * UART that does not exist here and the message is lost. Natively it is the
 * single most useful thing the port can print -- it is the game explaining its
 * own boot, in its own words. */

/* ------------------------------------------------------------------ */
/* Native printf family.                                                */
/*                                                                      */
/* WHY NATIVE: the translated MSL formatter diverges from the           */
/* interpreter (difftest: _pformatter helper FAILS deterministically),  */
/* and its first casualty was visible on console: IUSB_OpenDeviceIds    */
/* snprintf'd "/dev/usb/%s/%x/%x" into an all-zero buffer, the USB      */
/* stack opened "" -> -106, and the whole Wiimote bring-up slept        */
/* forever. Ninety-seven call sites share that formatter. One correct   */
/* native implementation replaces them all, exactly as OSReport already */
/* formats guest varargs natively.                                      */
/*                                                                      */
/* Argument sources follow the PPC EABI the game was compiled for:      */
/* integer varargs in r-regs then the caller's overflow area at         */
/* old-r1+8; floats in f1..f8. The va_list struct is MSL's              */
/* { u8 gpr; u8 fpr; u16 pad; u32 overflow; u32 regsave; }.             */
/* ------------------------------------------------------------------ */

struct WcArgSrc {
    CpuContext *ctx;
    int   use_va;
    int   gi;          /* next integer reg index (direct mode)      */
    int   fi;          /* next float reg index (1-based, direct)    */
    u32   overflow;    /* guest addr of next stacked arg            */
    /* va_list mode */
    u32   va_gpr, va_fpr, va_over, va_save;
};

static u32 wc_arg_u32(WcArgSrc *a)
{
    if (a->use_va) {
        u32 v;
        if (a->va_gpr < 8u) { v = mem_read32(a->va_save + 4u * a->va_gpr); a->va_gpr++; }
        else { v = mem_read32(a->va_over); a->va_over += 4u; }
        return v;
    }
    if (a->gi <= 10) return a->ctx->gpr[a->gi++];
    {   u32 v = mem_read32(a->overflow); a->overflow += 4u; return v; }
}

static double wc_arg_f64(WcArgSrc *a)
{
    if (a->use_va) {
        double v;
        if (a->va_fpr < 8u) {
            union { u64 u; double d; } c;
            c.u = ((u64)mem_read32(a->va_save + 32u + 8u * a->va_fpr) << 32)
                |  mem_read32(a->va_save + 36u + 8u * a->va_fpr);
            a->va_fpr++; v = c.d;
        } else {
            union { u64 u; double d; } c;
            a->va_over = (a->va_over + 7u) & ~7u;
            c.u = ((u64)mem_read32(a->va_over) << 32) | mem_read32(a->va_over + 4u);
            a->va_over += 8u; v = c.d;
        }
        return v;
    }
    if (a->fi <= 8) return a->ctx->fpr[a->fi++].d;
    return 0.0;
}

/* Format the guest format string at fmt_addr into `out` (host). Returns the
 * full untruncated length, C99-style. */
static unsigned wc_guest_vformat(u32 fmt_addr, WcArgSrc *a, char *out, unsigned cap)
{
    unsigned oi = 0, total = 0, k;
    char tmp[320], spec[16];
    u32 p = fmt_addr;
    for (;;) {
        u8 c = mem_read8(p++);
        if (!c) break;
        if (c != '%') {
            if (oi + 1 < cap) out[oi++] = (char)c;
            total++;
            continue;
        }
        /* collect %[flags][width][.prec][l]conv */
        {   unsigned si = 0;
            spec[si++] = '%';
            for (;;) {
                u8 d = mem_read8(p);
                if (si < sizeof spec - 3 &&
                    (d == '-' || d == '0' || d == '+' || d == ' ' || d == '#' ||
                     (d >= '1' && d <= '9') || d == '.' ||
                     (si > 1 && d >= '0' && d <= '9'))) { spec[si++] = (char)d; p++; }
                else break;
            }
            {   u8 d = mem_read8(p);
                if (d == 'l' || d == 'h') { p++; if (mem_read8(p) == d) p++; }
            }
            {   u8 conv = mem_read8(p++);
                tmp[0] = 0;
                switch (conv) {
                case '%': snprintf(tmp, sizeof tmp, "%%"); break;
                case 'c': { spec[si++]='c'; spec[si]=0;
                            snprintf(tmp, sizeof tmp, spec, (int)(char)wc_arg_u32(a)); } break;
                case 'd': case 'i': { spec[si++]='d'; spec[si]=0;
                            snprintf(tmp, sizeof tmp, spec, (int)wc_arg_u32(a)); } break;
                case 'u': { spec[si++]='u'; spec[si]=0;
                            snprintf(tmp, sizeof tmp, spec, (unsigned)wc_arg_u32(a)); } break;
                case 'x': case 'X': case 'o': case 'p':
                          { spec[si++] = (conv=='p') ? 'x' : (char)conv; spec[si]=0;
                            snprintf(tmp, sizeof tmp, spec, (unsigned)wc_arg_u32(a)); } break;
                case 'f': case 'g': case 'G': case 'e': case 'E':
                          { spec[si++]=(char)conv; spec[si]=0;
                            snprintf(tmp, sizeof tmp, spec, wc_arg_f64(a)); } break;
                case 's': { u32 sp2 = wc_arg_u32(a); unsigned j = 0; char sb[512];
                            if (sp2) { while (j < sizeof sb - 1) { u8 ch = mem_read8(sp2 + j);
                                       if (!ch) break; sb[j++] = (char)ch; } }
                            sb[j] = 0; spec[si++]='s'; spec[si]=0;
                            snprintf(tmp, sizeof tmp, spec, sb); } break;
                case 'n': { u32 np = wc_arg_u32(a); mem_write32(np, total); } break;
                default:  snprintf(tmp, sizeof tmp, "%%%c", (char)conv); break;
                }
            }
        }
        for (k = 0; tmp[k]; k++) { if (oi + 1 < cap) out[oi++] = tmp[k]; total++; }
    }
    out[oi] = 0;
    return total;
}

/* Copy `src` into the guest buffer with vsnprintf's truncation contract:
 * at most size-1 bytes plus NUL when size > 0; r3 = untruncated length. */
static void wc_fmt_to_guest(CpuContext *ctx, u32 buf, u32 size,
                            const char *src, unsigned full)
{
    unsigned i, lim;
    if (buf && size) {
        lim = full < size - 1u ? full : size - 1u;
        for (i = 0; i < lim; i++) mem_write8(buf + i, (u8)src[i]);
        mem_write8(buf + lim, 0);
    }
    ctx->gpr[3] = full;
}

extern "C" void wc_hle_snprintf(CpuContext *ctx)      /* r3 buf r4 size r5 fmt */
{
    char out[1024]; WcArgSrc a; std::memset(&a, 0, sizeof a);
    a.ctx = ctx; a.gi = 6; a.fi = 1; a.overflow = ctx->gpr[1] + 8u;
    wc_fmt_to_guest(ctx, ctx->gpr[3], ctx->gpr[4], out,
                    wc_guest_vformat(ctx->gpr[5], &a, out, sizeof out));
}

/* The internal helper at 0x80010DD8 shares snprintf's register contract. */
extern "C" void wc_hle_vsnprintf_helper(CpuContext *ctx)
{
    wc_hle_snprintf(ctx);
}

extern "C" void wc_hle_sprintf(CpuContext *ctx)       /* r3 buf r4 fmt */
{
    char out[1024]; WcArgSrc a; std::memset(&a, 0, sizeof a);
    a.ctx = ctx; a.gi = 5; a.fi = 1; a.overflow = ctx->gpr[1] + 8u;
    wc_fmt_to_guest(ctx, ctx->gpr[3], 0xFFFFFFFFu, out,
                    wc_guest_vformat(ctx->gpr[4], &a, out, sizeof out));
}

static void wc_read_valist(WcArgSrc *a, CpuContext *ctx, u32 vap)
{
    std::memset(a, 0, sizeof *a);
    a->ctx = ctx; a->use_va = 1;
    a->va_gpr  = mem_read8(vap + 0);
    a->va_fpr  = mem_read8(vap + 1);
    a->va_over = mem_read32(vap + 4);
    a->va_save = mem_read32(vap + 8);
}

extern "C" void wc_hle_vsnprintf(CpuContext *ctx)     /* r3 buf r4 size r5 fmt r6 va */
{
    char out[1024]; WcArgSrc a;
    wc_read_valist(&a, ctx, ctx->gpr[6]);
    wc_fmt_to_guest(ctx, ctx->gpr[3], ctx->gpr[4], out,
                    wc_guest_vformat(ctx->gpr[5], &a, out, sizeof out));
}

extern "C" void wc_hle_vsprintf(CpuContext *ctx)      /* r3 buf r4 fmt r5 va */
{
    char out[1024]; WcArgSrc a;
    wc_read_valist(&a, ctx, ctx->gpr[5]);
    wc_fmt_to_guest(ctx, ctx->gpr[3], 0xFFFFFFFFu, out,
                    wc_guest_vformat(ctx->gpr[4], &a, out, sizeof out));
}

/* 800207D8: an unnamed 16-byte list push in the TRK map cluster --
 * node->next = head; node->fn = r4; node->arg = r3; head = node, with the
 * head in small data at r13-27500. It is the newest crumb in every low-mem
 * canary trip, so every invocation is logged (first 12) while the stomp is
 * hunted: if a call arrives with r5 (node) == 0, its first store IS the
 * corruption at folded 0x80000000. Semantics replicated exactly. */
extern "C" void func_80226744(CpuContext *);
extern "C" void func_801A1DD0(CpuContext *);   /* OSSetCurrentContext */

void wc_hle_OSSetCurrentContext(CpuContext *ctx)
{
    {   static unsigned n;
        if (n < 40u && (ctx->gpr[3] < 0x80000000u || ctx->gpr[3] > 0x94000000u)) {
            n++;
            LOG_WARN(LOG_CORE, "SETCC[%u] INSANE r3=%08x lr=%08x",
                     n, (unsigned)ctx->gpr[3], (unsigned)ctx->lr);
        }
    }
    func_801A1DD0(ctx);
}

extern "C" void func_8000B49C(CpuContext *);   /* ProcessRipRequest */

extern "C" { volatile int g_rip_inflight; volatile unsigned g_taskq_polls; }

void wc_hle_ProcessRipRequest(CpuContext *ctx)
{
    if (g_rip_inflight <= 0) g_rip_inflight = 1;   /* post-side arm normally did this */
    uint32_t req = ctx->gpr[3];
    {   static unsigned n;
        if (n < 4u) { n++;
            LOG_WARN(LOG_CORE, "RIPTASK[%u] enter req=%08x", n, req); } }
    func_8000B49C(ctx);
    {   static unsigned n2;
        if (n2 < 4u) { n2++;
            LOG_WARN(LOG_CORE, "RIPTASK[%u] exit result=%08x [req+80]=%08x",
                     n2, (unsigned)ctx->gpr[3],
                     req ? MemoryInline::Load<uint32_t>(req + 80u) : 0u); } }
    if (g_rip_inflight > 0) g_rip_inflight--;
}

extern "C" void func_80241DDC(CpuContext *);   /* TaskThread idle query */
extern "C" void func_80241D5C(CpuContext *);   /* TaskThread post */

/* Arm the in-flight guard at the POST, not at fn entry: the task thread
 * dequeues (flipping the idle query) before it calls the fn, and the strap
 * wait can slip out in that window. */
extern "C" void func_8000B2D0(CpuContext *);
#define WCH_CRUMB_MASK (WC_CRUMB_N - 1u)

void wc_hle_RipFromDisc1(CpuContext *ctx)
{
    static unsigned n;
    unsigned polls0 = g_taskq_polls;
    uint32_t req0 = ctx->gpr[3];
    if (n < 4u) LOG_WARN(LOG_CORE, "RIP1[%u] enter r3=%08x r4=%08x r8=%08x",
                         n + 1u, (unsigned)ctx->gpr[3], (unsigned)ctx->gpr[4],
                         (unsigned)ctx->gpr[8]);
    unsigned c0 = g_wc_calls;
    func_8000B2D0(ctx);
    {   unsigned c1 = g_wc_calls, span = c1 - c0, k, i;
        if (n < 2u) {
            char buf[220]; int off = 0;
            unsigned show = span > 20u ? 20u : span;
            for (k = 0; k < show; k++) {
                i = (c1 - show + k) & WCH_CRUMB_MASK;
                off += snprintf(buf + off, sizeof(buf) - (size_t)off, " %08x",
                                (unsigned)g_wc_crumb[i]);
                if (off > 200) break;
            }
            LOG_WARN(LOG_CORE, "RIP1 crumbs span=%u last:%s", span, buf);
        } }
    if (n < 4u) { n++;
        LOG_WARN(LOG_CORE, "RIP1[%u] exit r3=%08x polls=%u inflight=%d req180=%02x lr=%08x",
                 n, (unsigned)ctx->gpr[3], g_taskq_polls - polls0,
                 (int)g_rip_inflight,
                 req0 ? MemoryInline::Load<uint8_t>(req0 + 180u) : 0u,
                 (unsigned)ctx->lr); }
}

void wc_hle_TaskThreadPost(CpuContext *ctx)
{
    if (ctx->gpr[4] == 0x8000B49Cu) {
        g_rip_inflight++;
        static unsigned n;
        if (n < 4u) { n++;
            LOG_WARN(LOG_CORE, "RIPPOST[%u] armed fn=%08x arg=%08x", n,
                     (unsigned)ctx->gpr[4], (unsigned)ctx->gpr[5]); }
    }
    func_80241D5C(ctx);
}

/* The strap wait loops while this query returns 0, then reads the result
 * the rip task wrote. The query can flip in the window between the task
 * thread DEQUEUEING the request and ProcessRipRequest WRITING the result
 * (queue-empty is true while the work is mid-flight) -- the scene then
 * exits with a stale NULL and decodes zeros. Hold the query at 0 while the
 * task body is actually executing. */
void wc_hle_TaskThreadIdleQuery(CpuContext *ctx)
{
    uint32_t call_lr = ctx->lr;
    g_taskq_polls++;
    func_80241DDC(ctx);
    {   static unsigned np;
        if (np < 8u) { np++;
            LOG_WARN(LOG_CORE, "TASKQPOLL[%u] lr=%08x r3=%08x inflight=%d",
                     np, (unsigned)call_lr, (unsigned)ctx->gpr[3],
                     (int)g_rip_inflight); } }
    /* Nonzero = tasks outstanding (loop continues); zero = done (the wait
     * exits and reads the result). The race window is the query hitting 0
     * between the task thread dequeuing the request and ProcessRipRequest
     * writing the result -- so while the rip is in flight, stay "busy". */
    if (ctx->gpr[3] == 0 && g_rip_inflight > 0) {
        static unsigned n;
        if (n < 6u) { n++;
            LOG_WARN(LOG_CORE, "TASKQ[%u] held busy: rip in flight", n); }
        ctx->gpr[3] = 1;
    }
}

extern "C" void func_80218B04(CpuContext *);   /* EGG::Decomp::decode */

void wc_hle_EggDecompDecode(CpuContext *ctx)
{
    uint32_t src = ctx->gpr[3], dst = ctx->gpr[4];
    uint32_t magic = src ? MemoryInline::Load<uint32_t>(src) : 0u;
    {   static unsigned n;
        if (n < 8u) { n++;
            LOG_WARN(LOG_CORE, "DECOMP[%u] src=%08x dst=%08x magic=%08x",
                     n, src, dst, magic); } }
    func_80218B04(ctx);
}

extern "C" void func_80222448(CpuContext *);   /* EGG::DvdFile::open */

void wc_hle_EggDvdFileOpen(CpuContext *ctx)
{
    uint32_t path = ctx->gpr[4];
    char buf[64]; int i;
    for (i = 0; i < 63 && path; i++) {
        buf[i] = (char)MemoryInline::Load<uint8_t>(path + (uint32_t)i);
        if (!buf[i]) break;
    }
    buf[i] = 0;
    func_80222448(ctx);
    {   static unsigned n;
        if (n < 12u) { n++;
            LOG_WARN(LOG_CORE, "DVDOPEN[%u] \"%s\" -> %d",
                     n, buf, (int)ctx->gpr[3]); } }
}

extern "C" void func_8015E794(CpuContext *);   /* DVDReadPrio */

void wc_hle_DVDReadPrio(CpuContext *ctx)
{
    uint32_t len = ctx->gpr[5], off = ctx->gpr[6];
    func_8015E794(ctx);
    {   static unsigned n;
        if (n < 8u) { n++;
            LOG_WARN(LOG_CORE, "DVDRD[%u] len=%x off=%x -> ret=%d",
                     n, len, off, (int)ctx->gpr[3]); } }
}

extern "C" void func_80131748(CpuContext *);   /* BTA tick callback */
extern "C" { volatile unsigned g_bta_tick_n; }

void wc_hle_BtaTick(CpuContext *ctx)
{
    g_bta_tick_n++;
    {   static unsigned n;
        if (n < 8u) { n++;
            LOG_WARN(LOG_CORE, "BTATICK[%u]", n); } }
    func_80131748(ctx);
}

extern "C" void func_80162AB0(CpuContext *);   /* DVDGetDriveStatus */

/* THE LATCH TIMELINE. The disc-error screen flag (+81) is set by
 * DvdThread_main when it observes a drive status outside {0,1} while the
 * watch (+80) is armed. Every hypothesis about WHICH transient does it has
 * died on measurement, so record the timeline itself: every status this
 * poller sees, with the DVD state words and the flag bytes, interleaved in
 * the log with the DI command lines. */
void wc_hle_DVDGetDriveStatus(CpuContext *ctx)
{
    func_80162AB0(ctx);
    {   static unsigned n;
        if (n < 28u) {
            uint32_t obj = MemoryInline::Load<uint32_t>(0x80381C40u);
            n++;
            LOG_WARN(LOG_CORE, "DVDST[%u] ret=%d fatal=%08x st8=%08x cmd=%08x "
                     "watch=%u flag=%u",
                     n, (int)ctx->gpr[3],
                     MemoryInline::Load<uint32_t>(0x803822ECu),
                     MemoryInline::Load<uint32_t>(0x803822E8u),
                     MemoryInline::Load<uint32_t>(0x80382370u),
                     obj ? MemoryInline::Load<uint8_t>(obj + 80u) : 0u,
                     obj ? MemoryInline::Load<uint8_t>(obj + 81u) : 0u);
        }
    }
}

extern "C" void func_80194EF4(CpuContext *);   /* IPCiProfQueueReq */
extern "C" void wc_irq_pump(CpuContext *);
extern "C" unsigned int pi_intsr_raw(void);
extern "C" unsigned int pi_intmr_raw(void);

/* __ios_Ipc2 (0x80193500), AS HOST CODE. The translated body stages the
 * request, submits over MMIO inside an interrupts-off bracket, and waits in
 * loops that make no guest calls -- there is no legal point at which a
 * single-threaded port can deliver the completion interrupt the wait needs
 * (measured: gate counters hit=0/ee0=105, the same DI reply released four
 * times and never consumed). The WiiCompiled runtime answered this exact
 * problem by HLE-ing its IOS layer; this is that answer for this port.
 *
 * Contract (from the translated source): r3 = request block (virtual),
 * r4 = completion mode -- zero means synchronous (the body clears the
 * callback words at +44/+48 and waits itself), nonzero means asynchronous
 * (callback in the block fires from the IPC handler later). Return in r3:
 * the command result for sync, 0 ("queued") for async. The device engine
 * answers most requests instantly; a parked request (async device work)
 * completes via ipc_queue_reply, so sync waits watch the block's result
 * word with a generous timeout. */
extern "C" { s32 ios_dispatch_hle_direct(u32 req, s32 *result); }
extern "C" int  ios_take_reply_for(u32 req_virt);
extern "C" int  ios_fd_is_di(u32 fdn);
extern "C" u32  ipc_pop_reply_virt(void);
extern "C" void wc_invoke_ios_cb(CpuContext *, u32, u32, u32);
extern "C" void wc_ios_prof_reply(CpuContext *, u32);
extern "C" { extern u32 g_wc_cb_req; }
extern "C" void dev_lock(void);
extern "C" void dev_unlock(void);
extern "C" void ipc_queue_reply_virt(u32 req_virt);

void wc_hle_iosIpc2(CpuContext *ctx)
{
    {   static int tagged;
        if (!tagged) { tagged = 1;
            LOG_WARN(LOG_CORE, "IPC2 build " __DATE__ " " __TIME__); } }
    u32 req  = ctx->gpr[3];
    {   /* True block layout, once and for all: 12 words of the request. */
        static unsigned nd;
        if (nd < 4u && req) { nd++;
            LOG_WARN(LOG_CORE, "IPC2HDR[%u] %08x: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x",
                nd, req,
                MemoryInline::Load<uint32_t>(req+0u),  MemoryInline::Load<uint32_t>(req+4u),
                MemoryInline::Load<uint32_t>(req+8u),  MemoryInline::Load<uint32_t>(req+12u),
                MemoryInline::Load<uint32_t>(req+16u), MemoryInline::Load<uint32_t>(req+20u),
                MemoryInline::Load<uint32_t>(req+24u), MemoryInline::Load<uint32_t>(req+28u),
                MemoryInline::Load<uint32_t>(req+32u), MemoryInline::Load<uint32_t>(req+36u),
                MemoryInline::Load<uint32_t>(req+40u), MemoryInline::Load<uint32_t>(req+44u)); }
    }
    u32 mode = ctx->gpr[4];
    if (!req) { ctx->gpr[3] = (u32)-4; return; }
    if (!mode) {
        MemoryInline::Store<uint32_t>(req + 44u, 0u);
        MemoryInline::Store<uint32_t>(req + 48u, 0u);
    }
    {   s32 result = 0;
        s32 st = ios_dispatch_hle_direct(req, &result);
        static unsigned n;
        if (n < 60u) { n++;
            LOG_WARN(LOG_CORE, "IPC2[%u] req=%08x cmd=%u fd=%u a0=%08x mode=%08x st=%d res=%d",
                     n, req, (unsigned)MemoryInline::Load<uint32_t>(req),
                     (unsigned)MemoryInline::Load<uint32_t>(req + 8u),
                     (unsigned)MemoryInline::Load<uint32_t>(req + 12u),
                     (unsigned)mode, (int)st, (int)result); }
        if (st == 1) {                                /* replied now */
            if (!mode) {
                wc_ios_prof_reply(ctx, req);
                ctx->gpr[3] = (u32)result;
                return;
            }
            /* async: queue the completion, then RUN THE HANDLER before
             * returning. Every system software wait for an async completion is
             * potentially a call-free RAM spin (measured twice: __ios_Ipc2's
             * ack wait, then DVDLowInit spinning on its callback flag with
             * the reply queued and nobody to deliver it). Delivering at this
             * boundary -- inside a dispatched call, the exact context
             * handlers run in -- lands the callback before the caller can
             * begin waiting for it. IOS completing before the submit call
             * returns is hardware-legal; the system software arms its callback state
             * before issuing. */
            /* ALL async completions go through the DRAIN. Direct invocation
             * at issue-time ran each device's completion callback INSIDE the
             * submitting call; every device that chains its next submission
             * from that callback (DI first, then the BT control path, i.e.
             * all of them) mis-sequenced. The drain fires at the next
             * dispatch boundary -- after the issuing call returns, in plain
             * call context: hardware ordering for everything. */
            {   uint32_t fdw = MemoryInline::Load<uint32_t>(req + 8u);
                uint32_t cb  = MemoryInline::Load<uint32_t>(req + 32u);
                uint32_t usr = MemoryInline::Load<uint32_t>(req + 36u);
                uint32_t res = MemoryInline::Load<uint32_t>(req + 4u);
                {   static unsigned n;
                    if (n < 10u) { n++;
                        LOG_WARN(LOG_CORE, "IPC2DI[%u] fdw=%u is_di=%d cb=%08x",
                                 n, fdw, ios_fd_is_di(fdw), cb); } }
                /* The in-call completion applies ONLY to the DVDLow
                 * command family, whose caller enters a call-free spin the
                 * moment this returns. Keyed by the callback's text range:
                 * DVDLow lives in 0x8016_0000..0x8016_8000 (801645e4
                 * measured); the generic IOS completion shim (80169c54) and
                 * everything else stays on the drain -- widening this to
                 * the shim re-broke the Bluetooth sequencing. */
                if (cb >= 0x80160000u && cb < 0x80168000u) {
                    /* DI: the caller spins call-free the instant this call
                     * returns -- no drain can ever reach it. Real latency,
                     * then the callback under the full-save discipline. */
                    usleep(150);
                    g_wc_cb_req = req;
                    wc_invoke_ios_cb(ctx, cb, res, usr);
                    g_wc_cb_req = 0;
                    ctx->gpr[3] = 0;
                    return;
                }
            }
            /* CLASSIC async completion: into the IPC reply protocol; the
             * guest handler (pumped at boundaries/idle) consumes with full
             * system software fidelity. Merely re-enabling this path sent 0x1009/0x1003
             * and cleared the BT wall. */
            {   extern void ipc_queue_reply(u32 req_phys);
                dev_lock();
                ipc_queue_reply(req - 0x80000000u);
                dev_unlock();
            }
            ctx->gpr[3] = 0;
            return;
            {   int spin;
                uint32_t saved = ctx->msr;
                ctx->msr |= 0x8000u;
                {   extern volatile int g_host_site; g_host_site = 4; }
                for (spin = 0; spin < 200; spin++) {   /* <= ~20 ms */
                    extern unsigned int pi_intsr_raw(void);
                    extern unsigned int pi_intmr_raw(void);
                    if (pi_intsr_raw() & pi_intmr_raw()) {
                        wc_irq_pump(ctx);
                        if (!(pi_intsr_raw() & pi_intmr_raw())) break;
                    }
                    usleep(100);       /* device loop releases the reply */
                }
                ctx->msr = saved;
                {   extern volatile int g_host_site; g_host_site = 0; }
                {   static unsigned n;
                    if (n < 10u) { n++;
                        LOG_WARN(LOG_CORE, "IPC2CB[%u] req=%08x delivered after %d spins",
                                 n, req, spin); } }
            }
            ctx->gpr[3] = 0;
            return;
        }
        /* Parked: a device will complete it later via ipc_queue_reply. */
        if (mode) { ctx->gpr[3] = 0; return; }
        /* Sync on a parked request: wait host-side for the device to write
         * the block's result word (IOS semantics: +4 holds the command
         * result once complete). */
        {   int i;
            {   static unsigned n;
                if (n < 8u) { n++;
                    LOG_WARN(LOG_CORE, "IPC2WAIT[%u] req=%08x parked-sync begins", n, req); } }
            {   extern volatile int g_host_site; g_host_site = 3; }
            for (i = 0; i < 20000; i++) {             /* <= 2 s */
                if (ios_take_reply_for(req)) break;
                usleep(100);
            }
            {   extern volatile int g_host_site; g_host_site = 0; }
            {   static unsigned n2;
                if (n2 < 8u) { n2++;
                    LOG_WARN(LOG_CORE, "IPC2WAIT[%u] req=%08x %s after %d iters res=%08x",
                             n2, req, i < 20000 ? "completed" : "TIMEOUT", i,
                             (unsigned)MemoryInline::Load<uint32_t>(req + 4u)); } }
            wc_ios_prof_reply(ctx, req);
            ctx->gpr[3] = MemoryInline::Load<uint32_t>(req + 4u);
        }
    }
}

/* Invoke an IOS completion callback with the FULL volatile set saved --
 * the caller may be mid-boundary with staged argument registers, and the
 * callback chain may nest further IOS calls. One discipline for every
 * completion site. */
extern "C" void func_80194F84(CpuContext *);   /* IPCiProfReply */

/* The system software's completion bookkeeping: IPCiProfReply(req, [req+8]) dequeues the
 * request from the outstanding table and decrements the outstanding count.
 * EVERY completion path must run it -- the guest handler always did. Without
 * it the pool exhausted after ~7 async exchanges (IOS_IoctlvAsync refused
 * further submissions; Bluetooth froze between 0x1001 and 0x1009) and every
 * host-answered SYNC call leaked a slot too. Caller holds the volatile save. */
static void wc_ios_prof_reply_unsafe(CpuContext *ctx, u32 req)
{
    ctx->gpr[3] = req;
    ctx->gpr[4] = MemoryInline::Load<uint32_t>(req + 8u);
    func_80194F84(ctx);
}

extern "C" void wc_ios_prof_reply(CpuContext *ctx, u32 req)
{
    uint32_t s3 = ctx->gpr[3], s4 = ctx->gpr[4], s5 = ctx->gpr[5];
    uint32_t s0 = ctx->gpr[0], s6 = ctx->gpr[6], s7 = ctx->gpr[7];
    uint32_t slr = (uint32_t)ctx->lr, sctr = (uint32_t)ctx->ctr;
    uint32_t scr = ctx->cr, sxer = ctx->xer;
    wc_ios_prof_reply_unsafe(ctx, req);
    ctx->gpr[0] = s0; ctx->gpr[3] = s3; ctx->gpr[4] = s4;
    ctx->gpr[5] = s5; ctx->gpr[6] = s6; ctx->gpr[7] = s7;
    ctx->lr = slr; ctx->ctr = sctr; ctx->cr = scr; ctx->xer = sxer;
}

extern "C" { u32 g_wc_cb_req; }   /* request for the invoker's bookkeeping */

extern "C" void wc_invoke_ios_cb(CpuContext *ctx, u32 cb, u32 res, u32 usr)
{
    uint32_t sg[13]; PPC_FPR sf[14];
    uint32_t slr = (uint32_t)ctx->lr, sctr = (uint32_t)ctx->ctr;
    uint32_t scr = ctx->cr, sxer = ctx->xer, smsr = ctx->msr;
    int k;
    sg[0] = ctx->gpr[0];
    for (k = 0; k < 10; k++) sg[k + 1] = ctx->gpr[3 + k];
    for (k = 0; k < 14; k++) sf[k] = ctx->fpr[k];
    if (g_wc_cb_req) wc_ios_prof_reply_unsafe(ctx, g_wc_cb_req);
    ctx->gpr[3] = res;
    ctx->gpr[4] = usr;
    InvokeIndirectCpu(cb, ctx);
    ctx->gpr[0] = sg[0];
    for (k = 0; k < 10; k++) ctx->gpr[3 + k] = sg[k + 1];
    for (k = 0; k < 14; k++) ctx->fpr[k] = sf[k];
    ctx->lr = slr; ctx->ctr = sctr;
    ctx->cr = scr; ctx->xer = sxer; ctx->msr = smsr;
}

/* Drain device-completed (parked) replies as host-side callback calls.
 * Invoked at SelectThread entry: every sleep and wake passes there, so
 * completions land promptly, in a plain-call context. */
extern "C" { volatile unsigned g_drain_total; }
extern "C" void wc_ios_drain_replies(CpuContext *ctx)
{
    /* Retired: the guest IPC handler owns the reply queue again; a live
     * drain raced it for the same entries. */
    (void)ctx;
    return;
    int guard = 0;
    u32 req;
    while (guard++ < 8 && (req = ipc_pop_reply_virt()) != 0) {
        uint32_t cb  = MemoryInline::Load<uint32_t>(req + 32u);
        uint32_t usr = MemoryInline::Load<uint32_t>(req + 36u);
        uint32_t res = MemoryInline::Load<uint32_t>(req + 4u);
        static unsigned n;
        g_drain_total++;
        if (n < 48u) { n++;
            LOG_WARN(LOG_CORE, "IOSDRAIN[%u] req=%08x cb=%08x res=%d",
                     n, req, cb, (int)res); }
        if (cb >= 0x80004000u && cb < 0x80290000u) {
            g_wc_cb_req = req;
            wc_invoke_ios_cb(ctx, cb, res, usr);
            g_wc_cb_req = 0;
        } else {
            wc_ios_prof_reply(ctx, req);   /* no callback: still dequeue */
        }
    }
}

/* THE SYNC-IPC EDGE. __ios_Ipc2 queues a request and then waits for the
 * completion CALL-FREE: the sync-with-queue variant restores EE and polls,
 * the ack variant polls the ack counter -- neither makes another guest call,
 * so the single-thread fiber build has no boundary at which to deliver the
 * IPC interrupt the wait needs (measured: boot frozen at DVDLowInit's first
 * DI ioctl, calls static at 418, intsr=0x4000 held high forever). On
 * hardware the line traps the instant the guest's inline EE-restore executes
 * -- a few READ-ONLY instructions after this function returns. Delivering at
 * this boundary is that edge, a handful of instructions early, across which
 * the guest touches nothing the handler touches. */
void wc_hle_IPCiProfQueueReq(CpuContext *ctx)
{
    unsigned before, iters = 0;
    func_80194EF4(ctx);
    before = pi_intsr_raw() & pi_intmr_raw();
    /* The device side may ack/reply on its own thread: WAIT for the line,
     * bounded (openers completed in well under a millisecond; 20 ms covers
     * every device model's latency without harming async callers -- an early
     * callback is hardware-legal). */
    for (int i = 0; i < 100; i++) {
        if (pi_intsr_raw() & pi_intmr_raw()) {
            /* The guest re-enables EE a few READ-ONLY instructions after
             * this returns, then polls RAM call-free. Deliver at that edge,
             * a hair early, under the restored-EE view. */
            uint32_t saved = ctx->msr;
            ctx->msr |= 0x8000u;
            wc_irq_pump(ctx);
            ctx->msr = saved | (ctx->msr & ~0x8000u & 0u) | (saved & 0xFFFFFFFFu);
            ctx->msr = saved;
            iters++;
            if (!(pi_intsr_raw() & pi_intmr_raw())) break;
        }
        usleep(200);
    }
    {   static unsigned n;
        if (n < 10u) { n++;
            LOG_WARN(LOG_CORE, "IPCQ[%u] before=%04x iters=%u after=%04x",
                     n, before, iters,
                     pi_intsr_raw() & pi_intmr_raw()); } }
}

extern "C" void func_801AA918(CpuContext *);   /* OSSleepThread    */
extern "C" void func_801AAA04(CpuContext *);   /* OSWakeupThread   */
extern "C" void func_801A72BC(CpuContext *);   /* OSSendMessage    */
extern "C" void func_801A7384(CpuContext *);   /* OSReceiveMessage */

/* MESSAGE-QUEUE TRACE. The boot parks the MAIN thread on an EGG::ProcessMeter
 * worker's own task queue (verified live: queue 0x80425170, usedCount=0, main
 * linked as tail behind the ProcessMeter thread) and nothing ever sends. Who
 * called receive with that queue, and who was supposed to send, are the two
 * facts that decide the fix -- so log both sides with the caller's return
 * address rather than inferring from stale stack walks. */
/* Who enqueued MAIN on a worker's wait queue? The message tracers proved it
 * was not OSReceiveMessage (3 receives, none from main, zero sends), so the
 * enqueue came through OSSleepThread directly -- log the queue, the caller,
 * and the OS's idea of the current thread at that instant. A mismatch between
 * `cur` and the thread actually executing is the whole hypothesis: OSSleepThread
 * enqueues OS_CURRENT_THREAD, so a stale current pointer parks the WRONG
 * thread, which is exactly the shape of this defect. */
/* Rolling ring of the last 16 sleeps. Printing only the FIRST n missed the
 * one that matters -- the boot performs thousands of healthy IPC sleeps before
 * the one that parks main forever. The ring is read on demand from the rescue
 * listener ("sleeps"), so the wedge reports its own final moments. */
u32 g_sleep_q[16], g_sleep_lr[16], g_sleep_cur[16];
volatile unsigned g_sleep_n;

/* PER-THREAD last sleep. A plain ring is useless here: the display thread
 * sleeps on the retrace queue 60x a second and buries the one sleep that
 * matters (14,594 sleeps in a 40 s boot, the last 16 all retrace). Keyed by
 * the OS's current-thread pointer, this keeps the most recent sleep for every
 * thread -- so the thread that never woke still reports exactly which queue
 * it went down on and from where. */
extern "C" {
u32 g_rcv_thr[8], g_rcv_q[8], g_rcv_lr[8];
volatile unsigned g_rcv_cnt[8];
u32 g_snd_q[16], g_snd_msg[16], g_snd_lr[16], g_snd_cur[16];
volatile unsigned g_snd_n;
}
u32 g_slp_thr[8], g_slp_q[8], g_slp_lr[8];
volatile unsigned g_slp_cnt[8];

void wc_hle_OSSleepThread(CpuContext *ctx)
{
    u32 cur = MemoryInline::FlatRead32(0x800000E4u);
    unsigned i = g_sleep_n & 15u, k;
    g_sleep_q[i]   = ctx->gpr[3];
    g_sleep_lr[i]  = ctx->lr;
    g_sleep_cur[i] = cur;
    g_sleep_n++;
    for (k = 0; k < 8u; k++) {
        if (g_slp_thr[k] == cur || g_slp_thr[k] == 0) {
            g_slp_thr[k] = cur;
            g_slp_q[k]   = ctx->gpr[3];
            g_slp_lr[k]  = ctx->lr;
            g_slp_cnt[k]++;
            break;
        }
    }
    func_801AA918(ctx);
}

void wc_hle_OSWakeupThread(CpuContext *ctx)
{
    static unsigned n;
    if (n < 24u) {
        n++;
        LOG_WARN(LOG_CORE, "WAKE[%u] q=%08x head=%08x lr=%08x cur=%08x",
                 n, (unsigned)ctx->gpr[3],
                 ctx->gpr[3] ? (unsigned)MemoryInline::FlatRead32(ctx->gpr[3]) : 0u,
                 (unsigned)ctx->lr,
                 (unsigned)MemoryInline::FlatRead32(0x800000E4u));
    }
    func_801AAA04(ctx);
}

void wc_hle_OSReceiveMessage(CpuContext *ctx)
{
    u32 cur = MemoryInline::FlatRead32(0x800000E4u);
    unsigned k;
    for (k = 0; k < 8u; k++) {
        if (g_rcv_thr[k] == cur || g_rcv_thr[k] == 0) {
            g_rcv_thr[k] = cur;
            g_rcv_q[k]   = ctx->gpr[3];
            g_rcv_lr[k]  = ctx->lr;
            g_rcv_cnt[k]++;
            break;
        }
    }
    func_801A7384(ctx);
}

void wc_hle_OSSendMessage(CpuContext *ctx)
{
    unsigned i = g_snd_n & 15u;
    g_snd_q[i]   = ctx->gpr[3];
    g_snd_msg[i] = ctx->gpr[4];
    g_snd_lr[i]  = ctx->lr;
    g_snd_cur[i] = MemoryInline::FlatRead32(0x800000E4u);
    g_snd_n++;
    func_801A72BC(ctx);
}

/* __OSInitPlayTime: the Wii play-time/parental accounting. Long instrumented
 * boots WROTE a play-time record into the port's NAND with a TB-epoch
 * timestamp; every later boot then read it back as a play-limit (type==1,
 * remaining<=0) and OSPanic'd "Expired" at calls~918 -- a self-inflicted
 * time bomb. The feature is meaningless for the port: initialize nothing,
 * return cleanly. */
void wc_hle_OSInitPlayTime(CpuContext *ctx)
{
    (void)ctx;
}

/* EGG::ExpHeap::create(size, heap, opt) -- the recurring intermittent freeze
 * parks a thread at RKSystem::initialize+0x364 with this call in flight and
 * zero further dispatches. Log the entry, the allocator vtable and the exact
 * indirect target about to be invoked, and the return, for the first few
 * calls: the freeze then names its own mechanism. */
void wc_hle_EggExpHeapCreate(CpuContext *ctx)
{
    static unsigned n;
    u32 r3 = ctx->gpr[3], r4 = ctx->gpr[4], r5 = ctx->gpr[5];
    if (n < 8u) {
        u32 vt = 0, fn20 = 0, fn36 = 0;
        u32 alloc_obj = r4 ? r4 : MemoryInline::FlatRead32(0x80388880u - 23904u);
        if (alloc_obj) {
            vt   = MemoryInline::FlatRead32(alloc_obj);
            fn20 = vt ? MemoryInline::FlatRead32(vt + 20u) : 0;
            fn36 = vt ? MemoryInline::FlatRead32(vt + 36u) : 0;
        }
        n++;
        LOG_WARN(LOG_CORE, "EGGCREATE[%u] size=%08x heap=%08x opt=%08x "
                 "obj=%08x vt=%08x vt+20=%08x vt+36=%08x",
                 n, r3, r4, r5, alloc_obj, vt, fn20, fn36);
    }
    func_80226744(ctx);
    if (n <= 8u)
        LOG_WARN(LOG_CORE, "EGGCREATE ret=%08x", (unsigned)ctx->gpr[3]);
}

void wc_hle_TrkListPush(CpuContext *ctx)
{
    u32 r3 = ctx->gpr[3], r4 = ctx->gpr[4], r5 = ctx->gpr[5];
    u32 r13 = ctx->gpr[13];
    u32 head = MemoryInline::FlatRead32(r13 - 27500u);
    static int n;
    if (n < 12) {
        n++;
        LOG_WARN(LOG_CORE, "PUSH[%d] node=%08x fn=%08x arg=%08x r13=%08x head=%08x",
                 n, r5, r4, r3, r13, head);
    }
    MemoryInline::FlatWrite32(r5, head);
    MemoryInline::FlatWrite32(r5 + 4u, r4);
    MemoryInline::FlatWrite32(r5 + 8u, r3);
    MemoryInline::FlatWrite32(r13 - 27500u, r5);
    ctx->gpr[0] = head;   /* translated body leaves old head in r0 */
}

void wc_hle_OSReport(CpuContext *ctx)
{
    char msg[768];
    format_report(ctx, msg, sizeof msg);
    /* One-shot caller identification for the module-load failure: the format
     * string could not be traced to its function statically (indirect
     * reference), so name the caller from the live crumb ring the moment the
     * message passes through. */
    if (std::strstr(msg, "Load Module")) {
        static int once;
        if (!once) {
            extern volatile unsigned g_wc_calls;
            extern uint32_t g_wc_crumb[];
            unsigned c2 = g_wc_calls, k;
            char line[200]; int used = 0;
            once = 1;
            for (k = 16; k >= 1; k--)
                used += snprintf(line + used, sizeof line - (size_t)used,
                                 " %08x", (unsigned)g_wc_crumb[(c2 - k) & 63u]);
            LOG_WARN(LOG_CORE, "WC: module-fail caller trail:%s", line);
        }
    }
    /* The guest supplies its own newlines; strip a trailing one so the log
     * stays one message per line. */
    {   size_t l = std::strlen(msg);
        while (l && (msg[l - 1] == '\n' || msg[l - 1] == '\r')) msg[--l] = 0;
    }
    if (msg[0]) LOG_INFO(LOG_CORE, "GAME: %s", msg);
}

/* OSPanic / OSFatal: the game has decided it cannot continue. Print what it
 * says and stop -- continuing past a panic produces damage that gets blamed on
 * whatever runs next. */
void wc_hle_OSPanic(CpuContext *ctx)
{
    char file[192], msg[512];
    guest_str(ctx->gpr[3], file, sizeof file);
    {   CpuContext tmp = *ctx;
        tmp.gpr[3] = ctx->gpr[5];          /* the message is the third argument */
        tmp.gpr[4] = ctx->gpr[6];
        format_report(&tmp, msg, sizeof msg);
    }
    LOG_ERROR(LOG_CORE, "GAME PANIC at %s:%u: %s", file, (unsigned)ctx->gpr[4], msg);
    for (;;) { }
}

void wc_hle_OSFatal(CpuContext *ctx)
{
    char msg[512];
    guest_str(ctx->gpr[5], msg, sizeof msg);
    LOG_ERROR(LOG_CORE, "GAME FATAL: %s", msg);
    for (;;) { }
}

} /* extern "C" */

/* ------------------------------------------------------------------ */
/* Cache maintenance                                                    */
/*                                                                      */
/* Guest RAM is ordinary host memory here, and the RSX reads it          */
/* coherently, so a flush or an invalidate has nothing to do. That is    */
/* not a shortcut: on the console these exist to make the CPU's view and */
/* the GPU's view agree, and on this host they already do.               */
/*                                                                      */
/* dcbz is the exception and must really zero -- the game uses it to     */
/* clear structures cheaply, and a no-op there leaves live garbage.      */
/* ------------------------------------------------------------------ */
extern "C" {

void wc_hle_DCFlushRange(CpuContext *)      { }
void wc_hle_DCStoreRange(CpuContext *)      { }
void wc_hle_DCInvalidateRange(CpuContext *) { }

void wc_hle_DCZeroRange(CpuContext *ctx)
{
    /* r3 = address, r4 = length. The system software requires both to be 32-byte aligned
     * and the hardware zeroes whole cache lines, so round the way it does
     * rather than the way the arguments read. */
    uint32_t a = ctx->gpr[3] & ~31u;
    uint32_t n = (ctx->gpr[4] + (ctx->gpr[3] & 31u) + 31u) & ~31u;
    uint32_t i;
    for (i = 0; i < n; i += 4) MemoryInline::FlatWrite32(a + i, 0);
}

/* Gekko's locked cache: 16 KiB of L1 turned into a scratchpad at 0xE0000000,
 * with a DMA engine that moves 32-byte blocks between it and main memory. The
 * emulator already backs that address range, so these are copies -- and being
 * synchronous, the queue is always empty and a wait always returns at once.
 * That is stronger than the hardware, never weaker: code that waits for the
 * DMA finds it already done. */
void wc_hle_LCLoadBlocks(CpuContext *ctx)
{
    uint32_t dst = ctx->gpr[3];          /* locked-cache destination */
    uint32_t src = ctx->gpr[4];          /* main memory source       */
    uint32_t blocks = ctx->gpr[5] & 0x7Fu;   /* 0 means 128          */
    uint32_t n = (blocks ? blocks : 128u) * 32u, i;
    for (i = 0; i < n; i += 4)
        MemoryInline::FlatWrite32(dst + i, MemoryInline::FlatRead32(src + i));
}

void wc_hle_LCStoreBlocks(CpuContext *ctx)
{
    uint32_t dst = ctx->gpr[3];          /* main memory destination  */
    uint32_t src = ctx->gpr[4];          /* locked-cache source      */
    uint32_t blocks = ctx->gpr[5] & 0x7Fu;
    uint32_t n = (blocks ? blocks : 128u) * 32u, i;
    for (i = 0; i < n; i += 4)
        MemoryInline::FlatWrite32(dst + i, MemoryInline::FlatRead32(src + i));
}

void wc_hle_LCStoreData(CpuContext *ctx)
{
    uint32_t dst = ctx->gpr[3], src = ctx->gpr[4], n = ctx->gpr[5], i;
    for (i = 0; i + 3 < n; i += 4)
        MemoryInline::FlatWrite32(dst + i, MemoryInline::FlatRead32(src + i));
    for (; i < n; i++)
        MemoryInline::FlatWrite8(dst + i, MemoryInline::FlatRead8(src + i));
    ctx->gpr[3] = 0;
}

void wc_hle_LCQueueLength(CpuContext *ctx) { ctx->gpr[3] = 0; }   /* always drained */
void wc_hle_LCQueueWait(CpuContext *)      { }                    /* already done   */

/* PPCSync is a `sync` instruction: an ordering barrier. The host has one and
 * it is the right thing to emit -- the device model and the RSX are read by
 * other threads. */
void wc_hle_PPCSync(CpuContext *)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

/* HID2 carries PSE and LSQE, which the port holds in the context. */
void wc_hle_PPCMfhid2(CpuContext *ctx) { ctx->gpr[3] = ctx->hid2; }
void wc_hle_PPCMthid2(CpuContext *ctx) { ctx->hid2 = ctx->gpr[3]; }

/* The decrementer drives the guest's own scheduling tick. The device model
 * owns time here, so setting it is accepted and ignored rather than allowed to
 * write a register that nothing reads. */
void wc_hle_PPCMtdec(CpuContext *ctx)
{
    /* The written value is the alarm system's entire timing source: OSSetAlarm
     * arms the decrementer, the DEC exception runs AlarmHandler, AlarmHandler
     * re-arms it. Discarding the value (this was a no-op) silences every
     * OSAlarm in the game -- audio DMA pacing and the game's own timers.
     * Record it; the interrupt poller turns expiry into a delivered DEC
     * exception through the same machinery external interrupts use. */
    extern void wc_dec_write(uint32_t value);
    wc_dec_write(ctx->gpr[3]);
}

/* OSGetTime: the 64-bit timebase, which is what the guest's clock is built on.
 * Returned in r3:r4, high word first, as the ABI does for a 64-bit value. */
void wc_hle_OSGetSystemTime(CpuContext *ctx)
{
    uint64_t tb = ((uint64_t)PPC_Mftbu() << 32) | PPC_Mftb();
    ctx->gpr[3] = (uint32_t)(tb >> 32);
    ctx->gpr[4] = (uint32_t)tb;
}

/* A retail console, booted normally: no reset button, no development kit. */
void wc_hle_OSGetResetCode(CpuContext *ctx)  { ctx->gpr[3] = 0u; }
void wc_hle_OSGetConsoleType(CpuContext *ctx) { ctx->gpr[3] = 0x00000002u; } /* retail */

} /* extern "C" */

/* ------------------------------------------------------------------ */
/* Interrupts                                                           */
/*                                                                      */
/* The guest masks and unmasks interrupts by writing MSR[EE] and the     */
/* interrupt controller. Translated, the MSR write goes nowhere: the     */
/* port has no MSR, because after static recompilation there is no       */
/* PowerPC state machine to hold one -- the "registers" are C locals.    */
/* So the mask lives here instead, and the device model consults it      */
/* before delivering.                                                    */
/*                                                                      */
/* These return the PREVIOUS state, which is what the game's callers      */
/* store and hand back to OSRestoreInterrupts. Getting that backwards    */
/* would leave interrupts off after the first critical section and the   */
/* game would stop at its first wait, looking like a hang.               */
/* ------------------------------------------------------------------ */
extern "C" {

uint32_t wc_os_disable_interrupts(void);
uint32_t wc_os_enable_interrupts(void);
uint32_t wc_os_restore_interrupts(uint32_t level);

void wc_hle_OSDisableInterrupts(CpuContext *ctx) { ctx->gpr[3] = wc_os_disable_interrupts(); }
void wc_hle_OSEnableInterrupts(CpuContext *ctx)  { ctx->gpr[3] = wc_os_enable_interrupts(); }
void wc_hle_OSRestoreInterrupts(CpuContext *ctx) { ctx->gpr[3] = wc_os_restore_interrupts(ctx->gpr[3]); }

/* __init_hardware runs before anything else and programs the memory
 * controller, the caches and the exception vectors. On a port every one of
 * those belongs to the host: the arena exists, the caches are the PPE's, and
 * there are no guest exception vectors to install because there is no guest
 * exception mechanism. Letting the translated version run would have it write
 * hardware registers that model something this machine does not have.
 *
 * The guest-visible state it leaves behind -- the low-memory globals a game
 * reads back -- is already set up by mkw_setup_globals before the port starts,
 * for the same reason and from the same values. */
void wc_hle_init_hardware(CpuContext *) { }

} /* extern "C" */

/* One-shot instrumentation on the SZS decompressor: the low-memory wipe
 * happened during decodeSZS -> ResFile::Init, and the fastest attribution is
 * the decoder's own arguments -- source, destination, and the first bytes of
 * each. A dest pointing at (or folding into) low MEM1 is the smoking gun;
 * a source without the Yaz0 magic points back at disc content instead. */
extern "C" void func_80218B8C(CpuContext *);
extern "C" void wc_hle_decodeSZS(CpuContext *ctx)
{
    static unsigned n;
    if (n < 6u) {
        n++;
        uint32_t src = ctx->gpr[3], dst = ctx->gpr[4];
        LOG_WARN(LOG_CORE, "SZS[%u] src=%08x dst=%08x src0=%02x%02x%02x%02x "
                 "srclen~=%02x%02x%02x%02x", n, src, dst,
                 mem_read8(src+0), mem_read8(src+1), mem_read8(src+2),
                 mem_read8(src+3), mem_read8(src+4), mem_read8(src+5),
                 mem_read8(src+6), mem_read8(src+7));
    }
    func_80218B8C(ctx);
}

/* Null-destination trap on the g3d 32-byte copier: the canary proved a
 * near-zero dst wrote a bss pointer over the disc ID at calls=1266. The
 * caller's identity is in the crumb ring at entry. */
extern "C" void func_80060DA0(CpuContext *);
extern "C" void wc_hle_Copy32ByteBlocks(CpuContext *ctx)
{
    /* FOLD-AWARE: the canary tripped while this trap stayed silent -- the
     * destination reaches low memory through a MIRROR (0x00/0x40/0xC0
     * prefixes all fold to the same arena bytes as 0x80000000). Match on the
     * folded offset, and also log the first two calls unconditionally so the
     * normal dst shape is on record. */
    {   static unsigned seen;
        if (seen < 2u) { seen++;
            LOG_WARN(LOG_CORE, "COPY32[%u] dst=%08x src=%08x n=%08x",
                     seen, ctx->gpr[3], ctx->gpr[4], ctx->gpr[5]); }
    }
    if ((ctx->gpr[3] & 0x3FFFFFFFu) < 0x4000u) {
        static unsigned n;
        if (n < 4u) {
            extern volatile unsigned g_wc_calls;
            extern uint32_t g_wc_crumb[];
            unsigned c2 = g_wc_calls, k;
            char lb[120]; int used = 0;
            n++;
            for (k = 8; k >= 1; k--)
                used += snprintf(lb + used, sizeof lb - (size_t)used,
                                 " %08x", (unsigned)g_wc_crumb[(c2 - k) & 63u]);
            LOG_WARN(LOG_CORE, "NULLCOPY[%u] dst=%08x src=%08x n=%08x "
                     "callers:%s", n, ctx->gpr[3], ctx->gpr[4],
                     ctx->gpr[5], lb);
        }
    }
    func_80060DA0(ctx);
}
