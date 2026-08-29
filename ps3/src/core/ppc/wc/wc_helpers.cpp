/* wc_helpers.cpp -- out-of-line PowerPC helpers the translated game calls.
 *
 * The translator emits a call rather than inline code wherever an instruction
 * has state or a side effect it will not model inline: SPR access, the time
 * base, FPSCR, reservations, byte-reversed memory, the paired-single forms
 * whose lane bookkeeping is not worth duplicating at every site.
 *
 * Every one of these mirrors the interpreter (src/core/ppc/interp) so that a
 * translated function and the interpreter agree bit for bit -- which is what
 * tools/rec/wc_difftest.c checks. Where the interpreter reaches for emulator
 * state (the timebase, a reservation), so does this.
 */
extern "C" {
#include "../../../common/log.h"
#include "../gekko.h"
#include "../../mem/memmap.h"
#include "../../core_timing.h"
}
#include "ppc_runtime.h"
#include "memory.h"
#include "wc_ps.h"
#include <cstring>

extern CpuContext g_wc_ctx;

namespace {
/* One reservation, as the hardware has: lwarx sets it, stwcx. consumes it, and
 * any other write to the line breaks it. Single-runner threading means no other
 * guest thread can break it under us. */
uint32_t g_reserve_addr;
int      g_reserve_valid;
}

extern "C" {

/* ---- time base ---------------------------------------------------------
 * The guest reads a 64-bit counter at the bus clock through two 32-bit halves.
 * Serving both from one read of the emulator's timebase keeps them coherent;
 * reading twice could straddle a carry and hand the guest a value that never
 * existed. */
static uint64_t wc_timebase(void) { return timing_timebase(); }
uint32_t PPC_Mftb(void)   { return (uint32_t)(wc_timebase() & 0xFFFFFFFFu); }
uint32_t PPC_Mftbu(void)  { return (uint32_t)(wc_timebase() >> 32); }

/* ---- SPRs ---------------------------------------------------------------
 * Only the ones the game writes matter. GQRs decide how psq_l/psq_st quantise
 * and are read on every access, so they live in the context. HID2 carries
 * PSE/LSQE. Anything else is reported once and ignored, which is what the
 * interpreter does for an unmodelled SPR. */
extern "C" void wc_dec_write(uint32_t value);
extern "C" uint32_t wc_dec_remaining(void);

uint32_t PPC_ReadSpr(uint32_t spr)
{
    if (spr == 22u) return wc_dec_remaining();
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    if (spr >= 912u && spr <= 919u) return c->gqr[spr - 912u];
    switch (spr) {
    case 920u: return c->hid2;
    case 8u:   return c->lr;
    case 9u:   return c->ctr;
    case 1u:   return c->xer;
    case 268u: return PPC_Mftb();
    case 269u: return PPC_Mftbu();
    default:   return 0u;
    }
}
/* SPR 22 is the decrementer, and it matters: the translator INLINES the SDK's
 * PPCMtdec into its callers, so the address-level HLE on PPCMtdec never runs
 * and every write lands here instead. Ignoring it silenced every OSAlarm --
 * measured on console as the Bluetooth stack polling BTA_DmIsDeviceUp seven
 * million times a second, waiting for a GKI timer tick that could never come:
 * the BTE HCI state machine advances on alarms. */

void PPC_WriteSpr(uint32_t spr, uint32_t value)
{
    if (spr == 22u) {
        static unsigned logged;
        if (logged < 12u) { logged++;
            /* value/60750 = milliseconds at the Broadway decrementer rate:
             * the requested alarm interval, in units a human can read. The
             * GKI/BT tick pacing bug showed up as ~27 s intervals where
             * 10 ms belonged -- this line is what catches that class. */
            LOG_INFO(LOG_CORE, "WC: mtdec #%u = %u ticks (%u.%03u ms)",
                     logged, value, value / 60750u,
                     (value % 60750u) * 1000u / 60750u); }
        wc_dec_write(value); return;
    }
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    if (spr >= 912u && spr <= 919u) { c->gqr[spr - 912u] = value; return; }
    switch (spr) {
    case 920u: c->hid2 = value; break;
    case 8u:   c->lr   = value; break;
    case 9u:   c->ctr  = value; break;
    case 1u:   c->xer  = value; break;
    default:   break;
    }
}

uint32_t OSSystemCall(void) { return 0u; }

/* ---- FPSCR --------------------------------------------------------------
 * mffs delivers FPSCR in the low word with the high word all ones, which is
 * what the hardware leaves and what stfd/lwz sequences read back. */
double PPC_Mffs(void)
{
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    uint64_t bits = 0xFFF8000000000000ull | c->fpscr;
    double d; std::memcpy(&d, &bits, 8); return d;
}
void PPC_Mtfsf(uint32_t fm, double value)
{
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    uint64_t bits; uint32_t mask = 0; unsigned i;
    std::memcpy(&bits, &value, 8);
    for (i = 0; i < 8; i++) if (fm & (0x80u >> i)) mask |= 0xFu << (28 - 4 * i);
    c->fpscr = (c->fpscr & ~mask) | ((uint32_t)bits & mask);
}
uint32_t PPC_Mcrxr(uint32_t crField)
{
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    uint32_t f = (c->xer >> 28) & 0xFu, sh = (7u - (crField & 7u)) * 4u;
    c->cr = (c->cr & ~(0xFu << sh)) | (f << sh);
    c->xer &= 0x0FFFFFFFu;                      /* mcrxr clears XER[0..3] */
    return c->cr;
}
uint32_t PPC_Mcrfs(uint32_t dst, uint32_t src)
{
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    uint32_t ss = (7u - (src & 7u)) * 4u, ds = (7u - (dst & 7u)) * 4u;
    uint32_t f = (c->fpscr >> ss) & 0xFu;
    c->cr = (c->cr & ~(0xFu << ds)) | (f << ds);
    return c->cr;
}
uint32_t PPC_CrSetBit(uint32_t bitIndex, uint32_t value)
{
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    c->cr = PpcCrSetBitResident(c->cr, bitIndex, value);
    return c->cr;
}
uint32_t PPC_CrLogical(uint32_t op, uint32_t bt, uint32_t ba, uint32_t bb)
{
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    c->cr = PpcCrLogicalResident(c->cr, op, bt, ba, bb);
    return c->cr;
}
uint32_t PPC_Mcrf(uint32_t dst, uint32_t src)
{
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    c->cr = PpcMcrfResident(c->cr, dst, src);
    return c->cr;
}

/* ---- byte-reversed memory ----------------------------------------------
 * lwbrx/stwbrx and the halfword forms. Both guest and host are big-endian, so
 * "reversed" genuinely is a swap here rather than a no-op. */
uint32_t PPC_LoadWordByteReverse(uint32_t a)          { return __builtin_bswap32(MemoryInline::FlatRead32(a)); }
void     PPC_StoreWordByteReverse(uint32_t a, uint32_t v)  { MemoryInline::FlatWrite32(a, __builtin_bswap32(v)); }
uint32_t PPC_LoadHalfwordByteReverse(uint32_t a)      { return __builtin_bswap16(MemoryInline::FlatRead16(a)); }
void     PPC_StoreHalfwordByteReverse(uint32_t a, uint32_t v) { MemoryInline::FlatWrite16(a, __builtin_bswap16((uint16_t)v)); }

/* ---- reservations ------------------------------------------------------- */
uint32_t PPC_Lwarx(uint32_t addr)
{
    g_reserve_addr = addr & ~3u; g_reserve_valid = 1;
    return MemoryInline::FlatRead32(addr);
}
uint32_t PPC_Stwcx(uint32_t addr, uint32_t value)
{
    if (!g_reserve_valid || g_reserve_addr != (addr & ~3u)) { g_reserve_valid = 0; return 0u; }
    MemoryInline::FlatWrite32(addr, value);
    g_reserve_valid = 0;
    return 1u;                                   /* CR0[EQ] set on success */
}

/* ---- traps and odd stores ---------------------------------------------- */
void PPC_TrapWord(uint32_t opts, uint32_t lhs, uint32_t rhs)
{
    int32_t a = (int32_t)lhs, b = (int32_t)rhs;
    int fire = ((opts & 0x10) && a < b) || ((opts & 0x08) && a > b) ||
               ((opts & 0x04) && a == b) ||
               ((opts & 0x02) && lhs < rhs) || ((opts & 0x01) && lhs > rhs);
    if (fire) LOG_WARN(LOG_CORE, "WC: guest trap %08x vs %08x (opts %x)", lhs, rhs, opts);
}
void PPC_Stfiwx(uint32_t addr, double value)
{
    uint64_t b; std::memcpy(&b, &value, 8);
    MemoryInline::FlatWrite32(addr, (uint32_t)b);
}
void PPC_Fcmp(uint32_t crField, double a, double b)
{
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    SetCRFloatResident(c->cr, (int)crField, a, b);
}

/* ---- paired-single out-of-line forms ------------------------------------ */
double PPC_PsSum0(double a, double b, double c) { return PPC_PsSum0Inline(a, b, c); }
double PPC_PsSum1(double a, double b, double c) { return PPC_PsSum1Inline(a, b, c); }
double PPC_PsMuls0(double a, double c)          { return PPC_PsMuls0Inline(a, c); }
double PPC_PsMuls1(double a, double c)          { return PPC_PsMuls1Inline(a, c); }
double PPC_PsAbs(double v)                      { return PPC_PsAbsInline(v); }
double PPC_PsNabs(double v)                     { return PPC_PsNabsInline(v); }
double PPC_PsMr(double v)                       { return PPC_PsMrInline(v); }
double PPC_PsRes(double v)                      { return PPC_PsResInline(v); }
double PPC_PsRsqrte(double v)                   { return PPC_PsRsqrteInline(v); }
static void ps_cmp(uint32_t crf, double a, double b, int lane)
{
    CpuContext *c = wc_current_ctx ? wc_current_ctx : &g_wc_ctx;
    float x = lane ? PpcGetPs1Inline(a) : PpcGetPs0Inline(a);
    float y = lane ? PpcGetPs1Inline(b) : PpcGetPs0Inline(b);
    SetCRFloatResident(c->cr, (int)crf, (double)x, (double)y);
}
void PPC_PsCmpo0(uint32_t crf, double a, double b) { ps_cmp(crf, a, b, 0); }
void PPC_PsCmpu0(uint32_t crf, double a, double b) { ps_cmp(crf, a, b, 0); }
void PPC_PsCmpo1(uint32_t crf, double a, double b) { ps_cmp(crf, a, b, 1); }
void PPC_PsCmpu1(uint32_t crf, double a, double b) { ps_cmp(crf, a, b, 1); }

/* ---- integer forms with overflow -------------------------------------- */
uint32_t PPC_Divwo(uint32_t a, uint32_t b)       { return (uint32_t)PPC_Divw((int32_t)a, (int32_t)b); }
uint32_t PPC_Divwuo(uint32_t a, uint32_t b)      { return PPC_Divwu(a, b); }
uint32_t PPC_Mullwo(uint32_t a, uint32_t b)      { return a * b; }
uint32_t PPC_Nego(uint32_t v)                    { return (uint32_t)(-(int32_t)v); }

/* Cache maintenance. Guest RAM is plain host memory here and the RSX reads it
 * coherently, so a flush has nothing to do -- but dcbz DOES have to zero, and
 * the guest uses it to clear structures cheaply. That one is real
 * (memset_zero_32 in wc_adapter.cpp); these are not. */
void DCFlushRange(uint32_t, uint32_t) {}
void DCStoreRange(uint32_t, uint32_t) {}
void DCInvalidateRange(uint32_t, uint32_t) {}

} /* extern "C" */
