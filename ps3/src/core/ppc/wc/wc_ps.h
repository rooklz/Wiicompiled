/* wc_ps.h -- paired singles and quantised loads/stores for translated code,
 * PPE edition.
 *
 * WiiCompiled keeps a paired register as two packed floats in the 64-bit FPR
 * (ps0 in the high word) and computes lanes with SSE. Here each lane is a
 * float computed through the same gekko_* helpers the interpreter uses
 * (interp_fputil.h), so a translated function and the interpreter agree bit
 * for bit -- which tools/rec/aot_difftest.c checks. Argument order of every
 * helper is WiiCompiled's (runtime/include/isa/ppc_isa_float.h); the lane
 * semantics are Gekko's as interp_paired.c states them. */
#pragma once
#include "ppc_runtime.h"
#include "memory.h"

inline float PpcBitCastToFloatInline(uint32_t v) { float f; std::memcpy(&f, &v, 4); return f; }
inline uint32_t PpcBitCastToU32Inline(float f) { uint32_t v; std::memcpy(&v, &f, 4); return v; }
inline double PpcPackPairedInline(float ps0, float ps1) { PPC_FPR r; r.paired.ps0 = ps0; r.paired.ps1 = ps1; return r.d; }
inline float  PpcGetPs0Inline(double v) { PPC_FPR r; r.d = v; return r.paired.ps0; }
inline float  PpcGetPs1Inline(double v) { PPC_FPR r; r.d = v; return r.paired.ps1; }
inline void   PpcSetPairedFprInline(PPC_FPR& fpr, double packed) { fpr.d = packed; }
inline PPC_FPR PpcMakePairedResultInline(float ps0, float ps1) { PPC_FPR r; r.paired.ps0 = ps0; r.paired.ps1 = ps1; return r; }
inline float  PpcForceSingleValueInline(double v) { return static_cast<float>(v); }
inline double PPC_PsFromScalarInline(double v) { const float s = static_cast<float>(v); return PpcPackPairedInline(s, s); }
inline double PPC_PsToScalarInline(double v) { return static_cast<double>(PpcGetPs0Inline(v)); }

/* Lane arithmetic -- and the single most important performance decision in the
 * port.
 *
 * A paired-single lane IS a float, and every PowerPC single-precision op
 * computes in double and rounds once to single. That is Gekko's rule and it is
 * also the PPE's, bit for bit: `ps_mul` and the PPE's `fmuls` are the same
 * operation. So plain float arithmetic in C compiles to exactly the instruction
 * the guest executed -- one `fmuls`, one `fadds`, one `fmadds` -- and is exact
 * by construction rather than by reconstruction.
 *
 * The first version of this file did not do that. It computed in double and
 * then reproduced Gekko's rounding by hand, with tie detection and error-free
 * transforms, because that is what the *interpreter* has to do on a host whose
 * FP is not Gekko's. Here it was ~20 instructions per lane where one would do,
 * and it showed: PSMTXConcat, a 4x4 matrix multiply, compiled to 4,547
 * instructions.
 *
 * Force25 is absent for the same reason it is absent in the JIT: it rounds a
 * multiply's C operand to 25 bits, and a lane already holds a float, so it is
 * the identity here. It stays in the scalar path (ppc_runtime.h), where the
 * operand can be a full double.
 *
 * The fused forms use __builtin_fmaf, which is `fmadds` -- one rounding, as
 * ps_madd specifies. Writing a*c+b instead would round twice and be wrong. */
#define WC_LANES(v) const float v##0 = PpcGetPs0Inline(v), v##1 = PpcGetPs1Inline(v)
inline float wc_add(float a, float b) { return a + b; }
inline float wc_mul(float a, float c) { return a * c; }
inline float wc_div(float a, float b) { return a / b; }
inline float wc_madd(float a, float c, float b) { return __builtin_fmaf(a, c, b); }
inline float wc_nun(float v) { return (v != v) ? v : -v; }     /* negate unless NaN */

inline double PPC_PsAddInline(double a, double b) { WC_LANES(a); WC_LANES(b); return PpcPackPairedInline(wc_add(a0, b0), wc_add(a1, b1)); }
inline double PPC_PsSubInline(double a, double b) { WC_LANES(a); WC_LANES(b); return PpcPackPairedInline(wc_add(a0, -b0), wc_add(a1, -b1)); }
inline double PPC_PsMulInline(double a, double c) { WC_LANES(a); WC_LANES(c); return PpcPackPairedInline(wc_mul(a0, c0), wc_mul(a1, c1)); }
inline double PPC_PsDivInline(double a, double b) { WC_LANES(a); WC_LANES(b); return PpcPackPairedInline(wc_div(a0, b0), wc_div(a1, b1)); }
inline double PPC_PsMuls0Inline(double a, double c) { WC_LANES(a); const float c0 = PpcGetPs0Inline(c); return PpcPackPairedInline(wc_mul(a0, c0), wc_mul(a1, c0)); }
inline double PPC_PsMuls1Inline(double a, double c) { WC_LANES(a); const float c1 = PpcGetPs1Inline(c); return PpcPackPairedInline(wc_mul(a0, c1), wc_mul(a1, c1)); }
inline double PPC_PsMaddInline (double a, double c, double b) { WC_LANES(a); WC_LANES(c); WC_LANES(b); return PpcPackPairedInline(wc_madd(a0, c0, b0), wc_madd(a1, c1, b1)); }
inline double PPC_PsMsubInline (double a, double c, double b) { WC_LANES(a); WC_LANES(c); WC_LANES(b); return PpcPackPairedInline(wc_madd(a0, c0, -b0), wc_madd(a1, c1, -b1)); }
inline double PPC_PsNmaddInline(double a, double c, double b) { WC_LANES(a); WC_LANES(c); WC_LANES(b); return PpcPackPairedInline(wc_nun(wc_madd(a0, c0, b0)), wc_nun(wc_madd(a1, c1, b1))); }
inline double PPC_PsNmsubInline(double a, double c, double b) { WC_LANES(a); WC_LANES(c); WC_LANES(b); return PpcPackPairedInline(wc_nun(wc_madd(a0, c0, -b0)), wc_nun(wc_madd(a1, c1, -b1))); }
inline double PPC_PsMadds0Inline(double a, double c, double b) { WC_LANES(a); WC_LANES(b); const float c0 = PpcGetPs0Inline(c); return PpcPackPairedInline(wc_madd(a0, c0, b0), wc_madd(a1, c0, b1)); }
inline double PPC_PsMadds1Inline(double a, double c, double b) { WC_LANES(a); WC_LANES(b); const float c1 = PpcGetPs1Inline(c); return PpcPackPairedInline(wc_madd(a0, c1, b0), wc_madd(a1, c1, b1)); }
inline double PPC_PsSum0Inline(double a, double b, double c) { return PpcPackPairedInline(wc_add(PpcGetPs0Inline(a), PpcGetPs1Inline(b)), PpcGetPs1Inline(c)); }
inline double PPC_PsSum1Inline(double a, double b, double c) { return PpcPackPairedInline(PpcGetPs0Inline(c), wc_add(PpcGetPs0Inline(a), PpcGetPs1Inline(b))); }
inline double PPC_PsMerge00Inline(double a, double b) { return PpcPackPairedInline(PpcGetPs0Inline(a), PpcGetPs0Inline(b)); }
inline double PPC_PsMerge01Inline(double a, double b) { return PpcPackPairedInline(PpcGetPs0Inline(a), PpcGetPs1Inline(b)); }
inline double PPC_PsMerge10Inline(double a, double b) { return PpcPackPairedInline(PpcGetPs1Inline(a), PpcGetPs0Inline(b)); }
inline double PPC_PsMerge11Inline(double a, double b) { return PpcPackPairedInline(PpcGetPs1Inline(a), PpcGetPs1Inline(b)); }
inline double PPC_PsSelInline(double lhs, double control, double rhs) {   /* (FRC, FRA, FRB) */
    WC_LANES(lhs); WC_LANES(control); WC_LANES(rhs);
    return PpcPackPairedInline(control0 >= -0.0f ? lhs0 : rhs0, control1 >= -0.0f ? lhs1 : rhs1);
}
inline double PPC_PsNegInline(double v)  { PPC_FPR r; r.d = v; r.raw ^= 0x8000000080000000ull; return r.d; }
inline double PPC_PsAbsInline(double v)  { PPC_FPR r; r.d = v; r.raw &= 0x7FFFFFFF7FFFFFFFull; return r.d; }
inline double PPC_PsNabsInline(double v) { PPC_FPR r; r.d = v; r.raw |= 0x8000000080000000ull; return r.d; }
inline double PPC_PsMrInline(double v)   { return v; }
inline double PPC_PsResInline(double b)  { WC_LANES(b); return PpcPackPairedInline(static_cast<float>(gekko_fres(b0)), static_cast<float>(gekko_fres(b1))); }
inline double PPC_PsRsqrteInline(double b) { WC_LANES(b); return PpcPackPairedInline(static_cast<float>(gekko_frsqrte(b0)), static_cast<float>(gekko_frsqrte(b1))); }

/* ---- psq_l / psq_st ------------------------------------------------------
 * Mirrors interp_paired.c dequantize()/quantize(): the GQR scale is a 6-bit
 * signed exponent; loads dequantise then round to single; stores scale,
 * saturate, truncate. The float encoding (type 0 and the reserved codes) is
 * a raw single, no scale. */
namespace WcPsq {
inline int32_t ScaleExp(uint32_t s) { return (s & 0x20u) ? static_cast<int32_t>(s) - 64 : static_cast<int32_t>(s); }
inline float Dequant(uint32_t type, int32_t exp, int64_t raw) {
    double v;
    switch (type) {
        case 4: v = static_cast<double>(static_cast<uint8_t>(raw));  break;
        case 6: v = static_cast<double>(static_cast<int8_t>(raw));   break;
        case 5: v = static_cast<double>(static_cast<uint16_t>(raw)); break;
        default: v = static_cast<double>(static_cast<int16_t>(raw)); break;   /* 7 */
    }
    return static_cast<float>(std::ldexp(v, -exp));
}
inline uint32_t QuantBits(uint32_t type, int32_t exp, float value, unsigned& size) {
    double s = std::ldexp(static_cast<double>(value), exp);
    switch (type) {
        case 4: size = 1; if (!(s >= 0.0)) s = 0.0; if (s > 255.0) s = 255.0; return static_cast<uint8_t>(s);
        case 6: size = 1; if (!(s >= -128.0)) s = -128.0; if (s > 127.0) s = 127.0; return static_cast<uint8_t>(static_cast<int8_t>(s));
        case 5: size = 2; if (!(s >= 0.0)) s = 0.0; if (s > 65535.0) s = 65535.0; return static_cast<uint16_t>(s);
        default: size = 2; if (!(s >= -32768.0)) s = -32768.0; if (s > 32767.0) s = 32767.0; return static_cast<uint16_t>(static_cast<int16_t>(s));
    }
}
inline unsigned TypeSize(uint32_t type) { return (type == 4u || type == 6u) ? 1u : 2u; }

/* The quantised encodings, out of line and cold.
 *
 * These are the whole reason a psq op looks expensive: the type switch, the
 * scale exponent, ldexp and the saturation bounds. Inlined at every call site
 * they cost ~220 instructions each -- PSMTXConcat, nineteen psq ops in a 167
 * line function, compiled to 4,191 instructions because of it.
 *
 * The float encoding (type 0, and the three reserved codes hardware treats the
 * same) is what the SDK leaves in GQR0 and what almost every access uses. That
 * one stays inline and is two loads; everything else goes through here. */
MKW_PPC_NO_INLINE inline double LoadQuantised(uint32_t gqr, uint32_t addr, uint32_t w)
{
    const uint32_t type = (gqr >> 16) & 7u;
    const int32_t  exp  = ScaleExp((gqr >> 24) & 0x3Fu);
    const unsigned sz   = TypeSize(type);
    const int64_t  r0   = sz == 1 ? MemoryInline::Load<uint8_t>(addr)
                                  : MemoryInline::Load<uint16_t>(addr);
    const float a = Dequant(type, exp, r0);
    if (w) return PpcPackPairedInline(a, 1.0f);
    const int64_t r1 = sz == 1 ? MemoryInline::Load<uint8_t>(addr + 1)
                               : MemoryInline::Load<uint16_t>(addr + 2);
    return PpcPackPairedInline(a, Dequant(type, exp, r1));
}

MKW_PPC_NO_INLINE inline void StoreQuantised(uint32_t gqr, uint32_t addr, double value, uint32_t w)
{
    const uint32_t type = gqr & 7u;
    const int32_t  exp  = ScaleExp((gqr >> 8) & 0x3Fu);
    WC_LANES(value);
    unsigned sz;
    const uint32_t q0 = QuantBits(type, exp, value0, sz);
    if (sz == 1) MemoryInline::Store<uint8_t>(addr, static_cast<uint8_t>(q0));
    else         MemoryInline::Store<uint16_t>(addr, static_cast<uint16_t>(q0));
    if (w) return;
    const uint32_t q1 = QuantBits(type, exp, value1, sz);
    if (sz == 1) MemoryInline::Store<uint8_t>(addr + 1, static_cast<uint8_t>(q1));
    else         MemoryInline::Store<uint16_t>(addr + 2, static_cast<uint16_t>(q1));
}

/* psq_l. Float encoding inline (two loads); anything else out of line. */
inline double Load(uint32_t gqr, uint32_t addr, uint32_t w)
{
    if (__builtin_expect(((gqr >> 16) & 4u) == 0u, 1)) {   /* type 0..3 = float */
        const float a = PpcBitCastToFloatInline(MemoryInline::Load<uint32_t>(addr));
        const float b = w ? 1.0f : PpcBitCastToFloatInline(MemoryInline::Load<uint32_t>(addr + 4));
        return PpcPackPairedInline(a, b);
    }
    return LoadQuantised(gqr, addr, w);
}

/* psq_st, same split. */
inline void Store(uint32_t gqr, uint32_t addr, double value, uint32_t w)
{
    if (__builtin_expect((gqr & 4u) == 0u, 1)) {           /* type 0..3 = float */
        WC_LANES(value);
        MemoryInline::Store<uint32_t>(addr, PpcBitCastToU32Inline(value0));
        if (!w) MemoryInline::Store<uint32_t>(addr + 4, PpcBitCastToU32Inline(value1));
        return;
    }
    StoreQuantised(gqr, addr, value, w);
}
} /* namespace WcPsq */


/* Template entry points in WiiCompiled's shapes. The resolved forms get a host
 * pointer for the range; ours are arena pointers, so host + offset is exactly
 * base + Fold(addr) and the generic path can be used with the address. */
template <uint32_t W, uint32_t I>
inline double PPC_PsqLResolvedInline(CpuContext* cpu, uint8_t* host, uint32_t off, uint32_t addr) {
    (void)host; (void)off; return WcPsq::Load(cpu->gqr[I], addr, W);
}
template <uint32_t W, uint32_t I, uint32_t GQR>
inline double PPC_PsqLKnownResolvedInline(CpuContext*, uint8_t* host, uint32_t off, uint32_t addr) {
    (void)host; (void)off; return WcPsq::Load(GQR, addr, W);
}
template <uint32_t W, uint32_t I>
inline double PPC_PsqLGqrInline(CpuContext*, uint32_t gqrValue, uint32_t addr) {
    return WcPsq::Load(gqrValue, addr, W);
}
template <uint32_t W, uint32_t I>
inline void PPC_PsqStResolvedInline(CpuContext* cpu, uint8_t* host, uint32_t off, uint32_t addr, double v) {
    (void)host; (void)off; WcPsq::Store(cpu->gqr[I], addr, v, W);
}
template <uint32_t W, uint32_t I, uint32_t GQR>
inline void PPC_PsqStKnownResolvedInline(CpuContext*, uint8_t* host, uint32_t off, uint32_t addr, double v) {
    (void)host; (void)off; WcPsq::Store(GQR, addr, v, W);
}
template <uint32_t W, uint32_t I>
inline void PPC_PsqStGqrInline(CpuContext*, uint32_t gqrValue, uint32_t addr, double v) {
    WcPsq::Store(gqrValue, addr, v, W);
}
template <uint32_t W, uint32_t I>
inline void PPC_PsqStStackInline(CpuContext* cpu, uint32_t addr, double v) {
    WcPsq::Store(cpu->gqr[I], addr, v, W);
}
inline double PPC_PsqL(uint32_t addr, uint32_t w, uint32_t i) { return WcPsq::Load(wc_current_ctx->gqr[i], addr, w); }
inline void   PPC_PsqSt(uint32_t addr, double v, uint32_t w, uint32_t i) { WcPsq::Store(wc_current_ctx->gqr[i], addr, v, w); }

/* Stack-addressed psq: the translator proves the address is the guest stack,
 * which is always RAM. Same maths, no MMIO test to skip -- Load/Store already
 * take the fast path for RAM. */
template <uint32_t W, uint32_t I>
inline double PPC_PsqLStackInline(CpuContext* cpu, uint32_t addr) { return WcPsq::Load(cpu->gqr[I], addr, W); }
template <uint32_t W, uint32_t I, uint32_t GQR>
inline double PPC_PsqLKnownStackInline(CpuContext*, uint32_t addr) { return WcPsq::Load(GQR, addr, W); }
template <uint32_t W, uint32_t I, uint32_t GQR>
inline void PPC_PsqStKnownStackInline(CpuContext*, uint32_t addr, double v) { WcPsq::Store(GQR, addr, v, W); }
template <uint32_t W, uint32_t I>
inline double PPC_PsqLKnownInline(CpuContext* cpu, uint32_t addr) { return WcPsq::Load(cpu->gqr[I], addr, W); }
template <uint32_t W, uint32_t I, uint32_t GQR>
inline void PPC_PsqStKnownInline(CpuContext*, uint32_t addr, double v) { WcPsq::Store(GQR, addr, v, W); }
/* The explicit-state forms take the GQR by value (hoisted out of the context by
 * the translator) and a Stack flag saying the address is provably the guest
 * stack. Stack only lets WiiCompiled skip its MMIO page test; ours is already a
 * single mask-and-compare that the RAM path takes anyway, so both specialise to
 * the same code here. */
template <uint32_t W, uint32_t I, bool Stack>
inline double PPC_PsqLStateInline(uint32_t gqr, uint32_t addr) { return WcPsq::Load(gqr, addr, W); }
template <uint32_t W, uint32_t I, bool Stack>
inline void PPC_PsqStStateInline(uint32_t gqr, uint32_t addr, double v) { WcPsq::Store(gqr, addr, v, W); }

/* Context-taking and context-free forms of the plain psq pair. */
template <uint32_t W, uint32_t I>
inline double PPC_PsqLInline(CpuContext* cpu, uint32_t addr) { return WcPsq::Load(cpu->gqr[I], addr, W); }
template <uint32_t W, uint32_t I>
inline double PPC_PsqLInline(uint32_t addr) { return WcPsq::Load(wc_current_ctx->gqr[I], addr, W); }
template <uint32_t W, uint32_t I>
inline void PPC_PsqStInline(CpuContext* cpu, uint32_t addr, double v) { WcPsq::Store(cpu->gqr[I], addr, v, W); }
template <uint32_t W, uint32_t I>
inline void PPC_PsqStInline(uint32_t addr, double v) { WcPsq::Store(wc_current_ctx->gqr[I], addr, v, W); }

/* Out-of-line spellings of the paired forms, for the sites the translator does
 * not inline. Operand order matches PPC_PsSelInline: (FRC, FRA, FRB) -- the
 * control is the middle argument, as ps_sel selects on frA. */
inline double PPC_PsSel(double lhs, double control, double rhs) { return PPC_PsSelInline(lhs, control, rhs); }
inline double PPC_PsAdd(double a, double b) { return PPC_PsAddInline(a, b); }
inline double PPC_PsSub(double a, double b) { return PPC_PsSubInline(a, b); }
inline double PPC_PsMul(double a, double c) { return PPC_PsMulInline(a, c); }
inline double PPC_PsDiv(double a, double b) { return PPC_PsDivInline(a, b); }
inline double PPC_PsMadd (double a, double c, double b) { return PPC_PsMaddInline(a, c, b); }
inline double PPC_PsMsub (double a, double c, double b) { return PPC_PsMsubInline(a, c, b); }
inline double PPC_PsNmadd(double a, double c, double b) { return PPC_PsNmaddInline(a, c, b); }
inline double PPC_PsNmsub(double a, double c, double b) { return PPC_PsNmsubInline(a, c, b); }
inline double PPC_PsMerge00(double a, double b) { return PPC_PsMerge00Inline(a, b); }
inline double PPC_PsMerge01(double a, double b) { return PPC_PsMerge01Inline(a, b); }
inline double PPC_PsMerge10(double a, double b) { return PPC_PsMerge10Inline(a, b); }
inline double PPC_PsMerge11(double a, double b) { return PPC_PsMerge11Inline(a, b); }
inline double PPC_PsNeg(double v) { return PPC_PsNegInline(v); }
inline double PPC_PsFromScalar(double v) { return PPC_PsFromScalarInline(v); }
inline double PPC_PsToScalar(double v) { return PPC_PsToScalarInline(v); }
