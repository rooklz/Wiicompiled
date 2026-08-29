/* ppc_runtime.h -- PPE-side shim for WiiCompiled-translated functions.
 *
 * The static recompiler (external/Wiicompiled, GPLv3) emits one C++ function
 * per guest function against a small runtime surface: a CpuContext, flat
 * guest-memory accessors, and inline helpers for the handful of PowerPC
 * semantics C cannot express directly. Its own runtime is x86-64/Windows and
 * C++20; this header is the same surface for gcc 7 on the Cell PPE, where
 * the host is big-endian and most of what the x86 runtime has to emulate is
 * simply native. Semantics of every helper mirror runtime/include/isa/ (the *.h files). */
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>

#define MKW_RESTRICT __restrict
#define MKW_PPC_FORCE_INLINE inline __attribute__((always_inline))

union PPC_FPR {
    uint64_t raw;
    double   d;
    struct { float ps0; float ps1; } paired;    /* big-endian: ps0 is the high word */
};

struct CpuContext {
    uint32_t gpr[32];
    uint32_t cr, lr, ctr, xer, fpscr, pc;
    PPC_FPR  fpr[32];
    uint32_t gqr[8];
    uint32_t hid0, hid1, hid2, srr0, srr1, msr;
    /* Added for the PS3 dispatcher: guest instructions retired by this call,
     * maintained by the counting pass tools/wc_postprocess.py inserts. */
    uint32_t insn_count;
};

/* The context of the translated function currently running. Set by whatever
 * enters translated code (wc_boot, or the emulator's AOT adapter), and read by
 * the few helpers whose guest instruction names state -- FPSCR bits, SPRs --
 * without being handed the context. */
extern CpuContext *wc_current_ctx;

/* CALL BREADCRUMB
 *
 * The port runs native code, so when the game stops there is no guest pc to
 * read and no interpreter loop to ask -- the first hang produced exactly two
 * SDK banners and nothing else, with no way to tell which of 13,675 functions
 * it was sitting in.
 *
 * Every guest call passes through one of the dispatch specializations, and all
 * of those live in a single generated file, so recording the address there
 * costs one store and one increment per call and needs no change to the
 * translated bodies themselves.
 *
 * The counter doubles as the ring index and as the watchdog's progress signal:
 * g_wc_calls existed but nothing ever incremented it, which is why every hang
 * so far reported "calls=0" whether the game was working or not. */
#define WC_CRUMB_N 64u

/* Back-edge delivery points. Call-free guest loops (a goto with no call
 * between label and jump) are unpreemptible here even though hardware's
 * decrementer preempts them; the device loop arms this flag when a line is
 * live or the decrementer is due, and the injected poll at each such back
 * edge becomes the delivery point. Locals live in host registers across it
 * (no spill/reload is emitted at a non-call site), so a delivery -- or a
 * fiber switch inside one -- preserves the interrupted loop exactly. */
#ifdef __cplusplus
extern "C" {
#endif
extern volatile unsigned g_wc_backedge_arm;
void wc_backedge_service(CpuContext *ctx);
#ifdef __cplusplus
}
#endif
static inline void WcBackedgePoll(CpuContext *ctx)
{
    if (__builtin_expect(g_wc_backedge_arm != 0, 0))
        wc_backedge_service(ctx);
}
/* Probe payload: names the call-free loop the guest is spinning in.
 * Two volatile stores; no delivery, no mirror interaction. */
#ifdef __cplusplus
extern "C" {
#endif
extern volatile unsigned WcBackedgePoll_site, WcBackedgePoll_n;
#ifdef __cplusplus
}
#endif
extern uint32_t          g_wc_crumb[WC_CRUMB_N];
extern volatile unsigned g_wc_calls;
#define WC_CRUMB(a) \
    do { g_wc_crumb[g_wc_calls++ & (WC_CRUMB_N - 1u)] = (uint32_t)(a); } while (0)

/* INTERRUPT DELIVERY AT A CALL BOUNDARY
 *
 * The guest does not only sleep while it waits -- it also BUSY-POLLS. SCInit
 * kicks off a NAND read and then spins on SCCheckStatus until it completes, so
 * a port that only delivers interrupts when the guest goes idle never delivers
 * this one: the guest never goes idle, and the read it is waiting for is
 * finished in the device model with nobody to say so. That loop ran 750
 * million calls deep without advancing.
 *
 * A guest call boundary is the natural safe point. The caller has already
 * written back every register the callee can see, the handler runs on its own
 * CpuContext, and it is on the GAME thread -- so unlike delivery from the
 * interrupt thread there is no concurrency with the code being interrupted.
 *
 * The cost is a load and a branch per guest call, against a flag that shares a
 * cache line with the call counter this macro already writes. Interrupts are
 * delivered only with the guest's own MSR[EE] set, so a critical section that
 * masks them is still respected exactly as the hardware would. */
extern volatile int g_wc_irq_pending;
extern volatile int g_wc_in_irq;
void wc_irq_deliver(CpuContext *ctx);
/* Blocks while ANOTHER thread is inside the guest's interrupt handler. Not
 * inline: it has to consult a thread-local to know whether the caller is the
 * thread running the handler, and that is the rare path. */
void wc_irq_wait(void);

/* MSR[EE], spelled out rather than pulled in: this header is included by all
 * 13,675 translated units and gekko.h is a C header from the emulator side. */
#define WC_MSR_EE 0x00008000u

/* Zero-lag low-memory canary (state in wc_os.cpp). Non-null only while armed;
 * one extra load+branch per guest call while hunting the ctor-pass stomp. */
extern "C" {
extern volatile const uint32_t *g_wc_canary_ptr;
void wc_canary_trip(void);
void wc_irq_pump(CpuContext *);
void wc_e4_stomp_check2(CpuContext *);
void wc_ios_drain_replies(struct CpuContext *);
int  t_in_irq_probe(void);
unsigned ipc_backlog_probe(void);
int  pi_interrupt_pending(void);
unsigned int pi_intsr_raw(void);
unsigned int pi_intmr_raw(void);
int  wc_dec_due(void);
}

inline void wc_irq_poll(CpuContext *ctx)
{
#ifdef WC_FIBER_SCHED
    {   extern void wc_e4_stomp_check2(CpuContext *);
        wc_e4_stomp_check2(ctx);
    }
    {   /* Device-completed IPC replies: drain at the dispatch boundary.
         * Counter proves the path is compiled and reached. */
        extern volatile unsigned g_bdrain_hits;
        if (ipc_backlog_probe()) {
            g_bdrain_hits++;
            if (!t_in_irq_probe())
                wc_ios_drain_replies(ctx);
        }
    }
    if ((ctx->msr & 0x8000u) &&
        ((pi_intsr_raw() & pi_intmr_raw()) || wc_dec_due()))
        wc_irq_pump(ctx);
    return;
#endif
    if (__builtin_expect(g_wc_canary_ptr != nullptr, 0) &&
        *g_wc_canary_ptr != 0x524D4345u)
        wc_canary_trip();
    /* SERIALISE AGAINST A HANDLER RUNNING ON THE INTERRUPT THREAD.
     *
     * When the guest goes idle it spins without making calls, so its interrupt
     * has to be delivered from the interrupt thread. That handler wakes a
     * thread PART WAY THROUGH its work -- OSWakeupThread sets the run-queue
     * hint about half way down -- and the moment it does, the idle spin exits
     * and the game thread starts executing guest code beside a handler that is
     * still running guest code.
     *
     * They collide on exactly the state the handler was servicing. The visible
     * result was an IPC request block being dispatched a second time, after
     * IOS had already written its reply marker into it: the model answered the
     * stale block with "bad command 8" -> EINVAL, the guest read that -4 as the
     * result of its own /dev/di open, and MKWii put up the Wii system error
     * screen and called OSFatal.
     *
     * A guest call cannot start while another thread is inside the handler. */
    if (__builtin_expect(g_wc_in_irq != 0, 0))
        wc_irq_wait();
    if (__builtin_expect(g_wc_irq_pending != 0, 0) && (ctx->msr & WC_MSR_EE))
        wc_irq_deliver(ctx);
}

extern "C" { extern volatile unsigned g_wc_dispatch_total; }   /* never rewound: watchdog metric */
extern "C" void wc_dispatch_guard(uint32_t a);   /* delivery-thread SelectThread escape */
#define WC_DISPATCH(a, ctx) do { WC_CRUMB(a); g_wc_dispatch_total++; wc_dispatch_guard(a); wc_irq_poll(ctx); } while (0)


/* ---- condition register ------------------------------------------------ */
inline void SetCRResident(uint32_t& cr, uint32_t xer, int field, int32_t a, int32_t b) noexcept {
    uint32_t value = (a < b ? 0x8u : 0u) | (a > b ? 0x4u : 0u) | (a == b ? 0x2u : 0u) | ((xer >> 31) & 1u);
    const int shift = (7 - field) * 4;
    cr = (cr & ~(0xFu << shift)) | (value << shift);
}
inline void SetCRResident(uint32_t& cr, uint32_t xer, int field, uint32_t a, uint32_t b) noexcept {
    uint32_t value = (a < b ? 0x8u : 0u) | (a > b ? 0x4u : 0u) | (a == b ? 0x2u : 0u) | ((xer >> 31) & 1u);
    const int shift = (7 - field) * 4;
    cr = (cr & ~(0xFu << shift)) | (value << shift);
}
inline void SetCRFloatResident(uint32_t& cr, int field, double a, double b) noexcept {
    uint32_t value = (std::isnan(a) || std::isnan(b)) ? 0x1u :
        ((a < b ? 0x8u : 0u) | (a > b ? 0x4u : 0u) | (a == b ? 0x2u : 0u));
    const int shift = (7 - field) * 4;
    cr = (cr & ~(0xFu << shift)) | (value << shift);
}
inline bool GetCRBitResident(uint32_t cr, int field, int bit) noexcept {
    const int shift = (7 - field) * 4 + (3 - bit);
    return ((cr >> shift) & 1u) != 0u;
}
inline uint32_t PpcCrSetBitResident(uint32_t cr, uint32_t bitIndex, uint32_t value) noexcept {
    const uint32_t mask = 1u << (31u - (bitIndex & 31u));
    return (value & 1u) != 0 ? (cr | mask) : (cr & ~mask);
}
inline uint32_t PpcCrLogicalResident(uint32_t cr, uint32_t op, uint32_t bt, uint32_t ba, uint32_t bb) noexcept {
    const uint32_t a = (cr >> (31u - (ba & 31u))) & 1u;
    const uint32_t b = (cr >> (31u - (bb & 31u))) & 1u;
    uint32_t result = 0;
    switch (op & 7u) {
        case 0: result = ~(a | b) & 1u; break;
        case 1: result = a & (~b & 1u); break;
        case 2: result = a ^ b; break;
        case 3: result = ~(a & b) & 1u; break;
        case 4: result = a & b; break;
        case 5: result = ~(a ^ b) & 1u; break;
        case 6: result = (~a & 1u) | b; break;
        case 7: result = a | b; break;
    }
    return PpcCrSetBitResident(cr, bt, result);
}
inline uint32_t PpcMcrfResident(uint32_t cr, uint32_t dstField, uint32_t srcField) noexcept {
    dstField &= 7u; srcField &= 7u;
    const uint32_t dstShift = (7u - dstField) * 4u, srcShift = (7u - srcField) * 4u;
    const uint32_t field = (cr >> srcShift) & 0xFu;
    return (cr & ~(0xFu << dstShift)) | (field << dstShift);
}

/* ---- integer ------------------------------------------------------------ */
inline uint32_t PpcRotl32Inline(uint32_t value, uint32_t shift) {
    shift &= 31u;
    return shift ? ((value << shift) | (value >> (32u - shift))) : value;
}
MKW_PPC_FORCE_INLINE uint32_t PPC_CntlzwInline(uint32_t value) {
    return value == 0 ? 32u : static_cast<uint32_t>(__builtin_clz(value));
}
inline uint32_t PPC_Slw(uint32_t value, uint32_t amount) { return (amount & 0x20u) ? 0u : value << (amount & 0x1Fu); }
inline uint32_t PPC_Srw(uint32_t value, uint32_t amount) { return (amount & 0x20u) ? 0u : value >> (amount & 0x1Fu); }
inline uint32_t PPC_Sraw(uint32_t value, uint32_t amount) {
    if (amount & 0x20u) return (value & 0x80000000u) ? 0xFFFFFFFFu : 0u;
    return static_cast<uint32_t>(static_cast<int32_t>(value) >> (amount & 0x1Fu));
}
inline uint32_t PPC_Divwu(uint32_t a, uint32_t b) { return b == 0 ? 0u : a / b; }
inline int32_t  PPC_Divw(int32_t a, int32_t b) {
    if (b == 0 || (a == std::numeric_limits<int32_t>::min() && b == -1)) return a < 0 ? -1 : 0;
    return a / b;
}
template <typename T> inline int32_t CompareUnsigned(T a, T b) {
    uint32_t ua = static_cast<uint32_t>(a), ub = static_cast<uint32_t>(b);
    return ua < ub ? -1 : (ua > ub ? 1 : 0);
}
template <int Bits> inline int32_t SignExtend(uint32_t val) { struct { int32_t x : Bits; } s; s.x = val; return s.x; }
inline int32_t ArithmeticShiftRight(int32_t val, int amount) { return val >> amount; }

/* dcbz: zero one 32-byte Gekko cache line at the (aligned) guest address. */
extern "C" int32_t memset_zero_32(int32_t address);

/* ------------------------------------------------------------------ */
/* Scalar FP entry points                                               */
/*                                                                      */
/* The translator emits WiiCompiled's names; the semantics are the ones  */
/* interp_fputil.h defines (paired-lane fill, the Gekko estimate tables, */
/* 25-bit rounding of a single multiply's C operand, NaN-preserving      */
/* negation). Routing both engines through the same helpers is what      */
/* keeps a translated function bit-exact against the interpreter, which  */
/* tools/rec/aot_difftest.c checks on every trial.                       */
/* ------------------------------------------------------------------ */
extern "C" {
#include "../interp/interp_fputil.h"
}

inline uint32_t PPC_FprLowWordInline(double v) { uint64_t b; std::memcpy(&b, &v, 8); return static_cast<uint32_t>(b); }
inline double   PpcBitCastToDoubleInline(uint64_t b) { double d; std::memcpy(&d, &b, 8); return d; }
inline uint64_t PpcBitCastToU64Inline(double v) { uint64_t b; std::memcpy(&b, &v, 8); return b; }

/* Single-precision scalar arithmetic: compute, then round once to single. */
inline double PpcFaddsInline(double a, double b) { return static_cast<double>(static_cast<float>(gekko_add_single(a, b))); }
inline double PpcFsubsInline(double a, double b) { return static_cast<double>(static_cast<float>(gekko_add_single(a, -b))); }
inline double PpcFmulsInline(double a, double c) { return static_cast<double>(static_cast<float>(gekko_mul_single(a, gekko_force25(c)))); }
inline double PpcFdivsInline(double a, double b) { return static_cast<double>(static_cast<float>(gekko_div_single(a, b))); }
inline double PpcFmaddsInline (double a, double c, double b) { return static_cast<double>(static_cast<float>(gekko_madd_single(a, gekko_force25(c),  b))); }
inline double PpcFmsubsInline (double a, double c, double b) { return static_cast<double>(static_cast<float>(gekko_madd_single(a, gekko_force25(c), -b))); }
inline double PpcFnmaddsInline(double a, double c, double b) { return gekko_neg_unless_nan(PpcFmaddsInline(a, c, b)); }
inline double PpcFnmsubsInline(double a, double c, double b) { return gekko_neg_unless_nan(PpcFmsubsInline(a, c, b)); }
inline double PpcForce25BitInline(double v) { return gekko_force25(v); }

extern "C" {
inline double PPC_Fadds(double a, double b) { return PpcFaddsInline(a, b); }
inline double PPC_Fsubs(double a, double b) { return PpcFsubsInline(a, b); }
inline double PPC_Fmuls(double a, double c) { return PpcFmulsInline(a, c); }
inline double PPC_Fdivs(double a, double b) { return PpcFdivsInline(a, b); }
inline double PPC_Fmadds (double a, double c, double b) { return PpcFmaddsInline(a, c, b); }
inline double PPC_Fmsubs (double a, double c, double b) { return PpcFmsubsInline(a, c, b); }
inline double PPC_Fnmadds(double a, double c, double b) { return PpcFnmaddsInline(a, c, b); }
inline double PPC_Fnmsubs(double a, double c, double b) { return PpcFnmsubsInline(a, c, b); }
/* Double-precision fused forms are the host's own, exactly. */
inline double PPC_Fmadd (double a, double c, double b) { return __builtin_fma(a, c,  b); }
inline double PPC_Fmsub (double a, double c, double b) { return __builtin_fma(a, c, -b); }
inline double PPC_Fnmadd(double a, double c, double b) { return gekko_neg_unless_nan(__builtin_fma(a, c,  b)); }
inline double PPC_Fnmsub(double a, double c, double b) { return gekko_neg_unless_nan(__builtin_fma(a, c, -b)); }
inline double PPC_Fres   (double v) { return gekko_fres(v); }
inline double PPC_Frsqrte(double v) { return gekko_frsqrte(v); }
inline double PPC_Fsel(double a, double c, double b) { return (a >= 0.0) ? c : b; }

/* fctiwz: round toward zero into the LOW 32 bits of the FPR. Saturating, and
 * NaN yields 0x80000000 -- interp_float.c's fctiw_common. */
inline int32_t PpcClampIntegerWordInline(double v) {
    if (v != v) return static_cast<int32_t>(0x80000000u);
    if (v >  2147483647.0) return 2147483647;
    if (v < -2147483648.0) return static_cast<int32_t>(0x80000000u);
    return static_cast<int32_t>(v);
}
inline double PPC_Fctiwz(double v) { return PpcBitCastToDoubleInline(static_cast<uint32_t>(PpcClampIntegerWordInline(v))); }
inline double PPC_Fctiw (double v) { return PPC_Fctiwz(__builtin_nearbyint(v)); }
inline uint32_t PPC_Cntlzw(uint32_t v) { return PPC_CntlzwInline(v); }
}

/* State-free ABI: the translator's alternative calling convention, which this
 * port does not use -- every function takes a CpuContext. */
inline constexpr bool MkwStateFreeAbiEnabled(uint32_t) noexcept { return false; }

/* ------------------------------------------------------------------ */
/* Remaining translator-emitted surface                                 */
/* ------------------------------------------------------------------ */

/* Inlining hints the emitter puts on generated bodies. WiiCompiled's are MSVC
 * spellings; these are the gcc ones. */
#ifndef MKW_PPC_NO_INLINE
#define MKW_PPC_NO_INLINE            __attribute__((noinline))
#endif
#ifndef MKW_PPC_ALWAYS_INLINE_BODY
#define MKW_PPC_ALWAYS_INLINE_BODY   __attribute__((always_inline))
#endif
#ifndef MKW_PPC_COLD
#define MKW_PPC_COLD                 __attribute__((cold))
#endif
#ifndef MKW_PPC_INTERNAL_CALL
#define MKW_PPC_INTERNAL_CALL
#endif

/* A state-free function returning two register-width values.
 *
 * The translator writes `return { a, b };`, which clang accepts for its
 * ext_vector_type(2) but gcc 7 refuses for a vector_size type. A plain
 * two-field struct takes the brace form and costs nothing: the PowerPC ELF
 * ABI returns a 16-byte aggregate in a register pair, which is exactly what
 * the vector would have done. operator[] is provided for any site that
 * indexes the result instead of naming the fields. */
struct MkwStateFreeResult2 {
    uint64_t v0, v1;
    uint64_t &operator[](unsigned i) { return i ? v1 : v0; }
    const uint64_t &operator[](unsigned i) const { return i ? v1 : v0; }
};

/* Time base. The PPE's is 64-bit at the bus clock; the guest reads it as a
 * pair of 32-bit halves through mftb/mftbu. */
extern "C" {
uint32_t PPC_Mftb(void);
uint32_t PPC_Mftbu(void);
/* SPR access the translator could not resolve statically. GQRs and HID are
 * the ones the game actually writes; everything else is reported and ignored,
 * which is what the interpreter does for an unmodelled SPR. */
uint32_t PPC_ReadSpr(uint32_t spr);
void     PPC_WriteSpr(uint32_t spr, uint32_t value);
uint32_t OSSystemCall(void);
/* FPSCR. The guest reads it for rounding/exception state; NI is the only bit
 * with an effect here, and interp_fputil.h owns that policy. */
double   PPC_Mffs(void);
void     PPC_Mtfsf(uint32_t fm, double value);
uint32_t PPC_Mcrxr(uint32_t crField);
uint32_t PPC_Mcrfs(uint32_t dst, uint32_t src);
uint32_t PPC_CrSetBit(uint32_t bitIndex, uint32_t value);
uint32_t PPC_CrLogical(uint32_t op, uint32_t bt, uint32_t ba, uint32_t bb);
uint32_t PPC_Mcrf(uint32_t dst, uint32_t src);
/* lwbrx / stwbrx and friends: byte-reversed access. The guest is big-endian
 * and so is this host, so "reversed" really is a swap here. */
uint32_t PPC_LoadWordByteReverse(uint32_t addr);
void     PPC_StoreWordByteReverse(uint32_t addr, uint32_t value);
uint32_t PPC_LoadHalfwordByteReverse(uint32_t addr);
void     PPC_StoreHalfwordByteReverse(uint32_t addr, uint32_t value);
uint32_t PPC_Lwarx(uint32_t addr);
uint32_t PPC_Stwcx(uint32_t addr, uint32_t value);
void     PPC_TrapWord(uint32_t opts, uint32_t lhs, uint32_t rhs);
void     PPC_Stfiwx(uint32_t addr, double value);
void     PPC_Fcmp(uint32_t crField, double a, double b);
}

/* An instruction the translator could not model. Loud by design: a silent
 * wrong answer here is far worse than a stop. */
[[noreturn]] void PPC_Undefined(uint32_t pc, uint32_t raw, const char *details);
#define UNDEFINED(pc, raw, details) PPC_Undefined((pc), (raw), (details))

/* Double-precision fused forms, inline. The scalar single ones above go
 * through gekko_madd_single; these are the host's own FMA, which is exactly
 * the guest's for double. */
inline double PpcFmaddInline (double a, double c, double b) { return __builtin_fma(a, c,  b); }
inline double PpcFmsubInline (double a, double c, double b) { return __builtin_fma(a, c, -b); }
inline double PpcFnmaddInline(double a, double c, double b) { return gekko_neg_unless_nan(__builtin_fma(a, c,  b)); }
inline double PpcFnmsubInline(double a, double c, double b) { return gekko_neg_unless_nan(__builtin_fma(a, c, -b)); }

/* mtfsb0/mtfsb1: set or clear one FPSCR bit, numbered MSB-first. Bit 29 is
 * NI (non-IEEE), which the guest sets to get flush-to-zero -- the one bit here
 * with an effect on results, and interp_fputil.h owns that policy. */
inline void PPC_Mtfsb0(uint32_t bit) {
    CpuContext *c = wc_current_ctx;
    if (c) c->fpscr &= ~(0x80000000u >> (bit & 31u));
}
inline void PPC_Mtfsb1(uint32_t bit) {
    CpuContext *c = wc_current_ctx;
    if (c) c->fpscr |= (0x80000000u >> (bit & 31u));
}
inline void PPC_Mtfsfi(uint32_t crf, uint32_t imm) {
    CpuContext *c = wc_current_ctx;
    uint32_t sh = 28u - 4u * (crf & 7u);
    if (c) c->fpscr = (c->fpscr & ~(0xFu << sh)) | ((imm & 0xFu) << sh);
}
