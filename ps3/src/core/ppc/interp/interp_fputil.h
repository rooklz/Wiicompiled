/* interp_fputil.h — single/double conversion and FP result classification.
 *
 * The two conversion routines below implement the algorithms given in the
 * PowerPC Programming Environments Manual for `lfs` (single -> double on load)
 * and `stfs` (double -> single on store). They are *not* equivalent to a C
 * `(double)(float)` cast: the manual's algorithm has specific behaviour for
 * subnormals and preserves NaN payloads, and titles do observe the difference.
 *
 * Worth stating plainly, because it is one of the clearest illustrations of why
 * this port is cheap where others are expensive:
 *
 *   On x86 or ARM these must be emulated in software on *every single-precision
 *   load and store* -- which, in Wii game code, is most floating-point traffic.
 *   On the Cell PPE, `lfs` and `stfs` are these algorithms, in hardware, in one
 *   instruction. The JIT therefore emits one instruction and gets bit-exactness
 *   for free; this software version exists for the interpreter, which must stay
 *   portable so the verification harness can run it on a workstation.
 */
#ifndef DOLPHIN_CORE_PPC_INTERP_FPUTIL_H
#define DOLPHIN_CORE_PPC_INTERP_FPUTIL_H

#include "../gekko.h"

#define DOUBLE_SIGN 0x8000000000000000ull
#define DOUBLE_EXP  0x7FF0000000000000ull
#define DOUBLE_FRAC 0x000FFFFFFFFFFFFFull
#define DOUBLE_QBIT 0x0008000000000000ull

#define SINGLE_SIGN 0x80000000u
#define SINGLE_EXP  0x7F800000u
#define SINGLE_FRAC 0x007FFFFFu

/* single -> double, as performed by `lfs`. */
DOL_INLINE u64 ppc_convert_to_double(u32 value)
{
    u64 x    = value;
    u64 exp  = (x >> 23) & 0xFFu;
    u64 frac = x & 0x007FFFFFu;

    if (exp > 0 && exp < 255) {
        /* Normal: copy sign and the top two exponent bits, then replicate the
         * inverted high exponent bit into the three bits the wider double
         * exponent adds. */
        u64 y = !(exp >> 7);
        u64 z = (y << 61) | (y << 60) | (y << 59);
        return ((x & 0xC0000000ull) << 32) | z | ((x & 0x3FFFFFFFull) << 29);
    } else if (exp == 0 && frac != 0) {
        /* Subnormal single: renormalize into the double's wider exponent
         * range, where it is an ordinary normal number. */
        exp = 1023 - 126;
        do {
            frac <<= 1;
            exp -= 1;
        } while ((frac & 0x00800000ull) == 0);
        return ((x & 0x80000000ull) << 32) | (exp << 52) |
               ((frac & 0x007FFFFFull) << 29);
    }
    /* Zero, infinity and NaN.
     *
     * The ISA's single-to-double conversion sets frD[2:4] = frS[1] replicated
     * in this case, just as the normal case above sets them to the INVERTED
     * bit. Omitting them is invisible for zero, where frS[1] is 0 anyway -- and
     * wrong for infinity and NaN, where frS[1] is 1 and those three bits are
     * exactly what carries the exponent to all-ones. Without them a NaN single
     * converted to a double with exponent 0x47F instead of 0x7FF: a finite,
     * enormous number where hardware gives a NaN.
     *
     * This affects `lfs` as well as `psq_l`, and the differential fuzzer found
     * it the moment its state enabled paired singles (docs/PLAN.md §16). */
    {
        u64 y = (exp >> 7) & 1u;        /* frS[1]: 1 for Inf/NaN, 0 for zero */
        u64 z = (y << 61) | (y << 60) | (y << 59);
        return ((x & 0xC0000000ull) << 32) | z | ((x & 0x3FFFFFFFull) << 29);
    }
}

/* double -> single, as performed by `stfs`. */
DOL_INLINE u32 ppc_convert_to_single(u64 x)
{
    u32 exp = (u32)((x >> 52) & 0x7FFu);

    if (exp > 896 || (x & ~DOUBLE_SIGN) == 0) {
        return (u32)(((x >> 32) & 0xC0000000ull) | ((x >> 29) & 0x3FFFFFFFull));
    } else if (exp >= 874) {
        /* Representable only as a subnormal single: shift the implicit 1 back
         * in and denormalize by the exponent difference. */
        u32 t = (u32)(0x80000000u | ((x & DOUBLE_FRAC) >> 21));
        t >>= (905 - exp);
        t |= (u32)((x >> 32) & 0x80000000ull);
        return t;
    }
    /* Underflows even a subnormal single. Hardware's result here is not
     * meaningfully defined; match the truncating path. */
    return (u32)(((x >> 32) & 0xC0000000ull) | ((x >> 29) & 0x3FFFFFFFull));
}

/* ------------------------------------------------------------------ */
/* Classification                                                       */
/* ------------------------------------------------------------------ */

DOL_INLINE int ppc_is_nan(u64 x)
{
    return (x & DOUBLE_EXP) == DOUBLE_EXP && (x & DOUBLE_FRAC) != 0;
}

DOL_INLINE int ppc_is_snan(u64 x)
{
    return ppc_is_nan(x) && !(x & DOUBLE_QBIT);
}

DOL_INLINE int ppc_is_inf(u64 x)
{
    return (x & DOUBLE_EXP) == DOUBLE_EXP && (x & DOUBLE_FRAC) == 0;
}

DOL_INLINE int ppc_is_zero(u64 x)
{
    return (x & ~DOUBLE_SIGN) == 0;
}

/* FPRF: the 5-bit result-class field FPSCR carries, used by fcmpu/fcmpo and by
 * `mcrfs`. Titles rarely read it, but the differential harness compares full
 * FPSCR state, so it has to be right. */
DOL_INLINE u32 ppc_compute_fprf(u64 x)
{
    const u32 C = 0x10u, FL = 0x08u, FG = 0x04u, FE = 0x02u, FU = 0x01u;
    if (ppc_is_nan(x))  return C | FU;
    if (ppc_is_inf(x))  return (x & DOUBLE_SIGN) ? (FL | FU) : (FG | FU);
    if (ppc_is_zero(x)) return (x & DOUBLE_SIGN) ? (C | FE) : FE;
    /* Subnormal has the C bit set alongside the sign-derived comparison bit. */
    if ((x & DOUBLE_EXP) == 0)
        return (x & DOUBLE_SIGN) ? (C | FL) : (C | FG);
    return (x & DOUBLE_SIGN) ? FL : FG;
}

DOL_INLINE void ppc_set_fprf(PPCState *s, u64 result)
{
    s->fpscr = (s->fpscr & ~FPSCR_FPRF_MASK) |
               ((ppc_compute_fprf(result) << 12) & FPSCR_FPRF_MASK);
    /* This write is authoritative, so any value compiled code deferred is
     * superseded. See PPCState.fprf_src. */
    s->fprf_src = s->fprf_ack = FPRF_SRC_NONE;
}

/* Fold a deferred FPRF (recorded by compiled code as a raw result value) into
 * `fpscr`. Call this before ANY read of FPSCR[FPRF] and before any partial
 * write of `fpscr` that must preserve FPRF bits it does not itself write.
 *
 * Reads of FPSCR outside the FPRF field -- update_cr1's FX/FEX/VX/OX summary,
 * ppc_set_fpscr_exception's read-modify-write -- do not need it: the deferred
 * value never affects those bits, and leaving it pending across them is
 * correct. */
DOL_INLINE void ppc_fprf_sync(PPCState *s)
{
    if (s->fprf_src != s->fprf_ack) {
        s->fpscr = (s->fpscr & ~FPSCR_FPRF_MASK) |
                   ((ppc_compute_fprf(s->fprf_src) << 12) & FPSCR_FPRF_MASK);
        s->fprf_src = s->fprf_ack = FPRF_SRC_NONE;
    }
}

/* Setting an exception bit also sets the summary bits. FX is "sticky-on-
 * transition": it is set only when the bit was not already set. */
DOL_INLINE void ppc_set_fpscr_exception(PPCState *s, u32 bits)
{
    if ((s->fpscr & bits) != bits)
        s->fpscr |= FPSCR_FX;
    s->fpscr |= bits;
    if (s->fpscr & FPSCR_VX_ANY)
        s->fpscr |= FPSCR_VX;
}


/* ------------------------------------------------------------------ */
/* Gekko/Broadway-specific FP semantics                                 */
/*                                                                      */
/* These are the places where the guest FPU is *not* generic PowerPC, and    */
/* where "compute the exact value" is the wrong answer. Every rule here is    */
/* hardware-derived: the estimate tables and the 25-bit operand rounding come */
/* from Dolphin's Source/Core/Common/FloatUtils.cpp and Interpreter_*.cpp     */
/* (GPLv2+), which took them from hardware tests; WiiCompiled's static        */
/* recompiler ships the identical tables and proves ghost-exact physics with  */
/* them. Before this file existed the emulator computed 1/x exactly, rounded   */
/* nothing to 25 bits, and left ps1 stale after single-precision results --   */
/* three ways to diverge from a real Wii that no differential test between    */
/* our own two engines could ever show, because both engines shared them.     */
/* ------------------------------------------------------------------ */

#include <math.h>

typedef struct { s32 base, dec; } GekkoEstEntry;

/* fres / ps_res: 32 segments over the mantissa's top 5 bits, linear inside. */
static const GekkoEstEntry k_gekko_fres[32] = {
    {0x7ff800, 0x3e1}, {0x783800, 0x3a7}, {0x70ea00, 0x371}, {0x6a0800, 0x340},
    {0x638800, 0x313}, {0x5d6200, 0x2ea}, {0x579000, 0x2c4}, {0x520800, 0x2a0},
    {0x4cc800, 0x27f}, {0x47ca00, 0x261}, {0x430800, 0x245}, {0x3e8000, 0x22a},
    {0x3a2c00, 0x212}, {0x360800, 0x1fb}, {0x321400, 0x1e5}, {0x2e4a00, 0x1d1},
    {0x2aa800, 0x1be}, {0x272c00, 0x1ac}, {0x23d600, 0x19b}, {0x209e00, 0x18b},
    {0x1d8800, 0x17c}, {0x1a9000, 0x16e}, {0x17ae00, 0x15b}, {0x14f800, 0x15b},
    {0x124400, 0x143}, {0x0fbe00, 0x143}, {0x0d3800, 0x12d}, {0x0ade00, 0x12d},
    {0x088400, 0x11a}, {0x065000, 0x11a}, {0x041c00, 0x108}, {0x020c00, 0x106},
};

/* frsqrte / ps_rsqrte: indexed by the exponent's low bit and the mantissa's
 * top 4 bits, so odd and even exponents get separate halves of the table. */
static const GekkoEstEntry k_gekko_frsqrte[32] = {
    {0x1a7e800, -0x568}, {0x17cb800, -0x4f3}, {0x1552800, -0x48d}, {0x130c000, -0x435},
    {0x10f2000, -0x3e7}, {0x0eff000, -0x3a2}, {0x0d2e000, -0x365}, {0x0b7c000, -0x32e},
    {0x09e5000, -0x2fc}, {0x0867000, -0x2d0}, {0x06ff000, -0x2a8}, {0x05ab800, -0x283},
    {0x046a000, -0x261}, {0x0339800, -0x243}, {0x0218800, -0x226}, {0x0105800, -0x20b},
    {0x3ffa000, -0x7a4}, {0x3c29000, -0x700}, {0x38aa000, -0x670}, {0x3572000, -0x5f2},
    {0x3279000, -0x584}, {0x2fb7000, -0x524}, {0x2d26000, -0x4cc}, {0x2ac0000, -0x47e},
    {0x2881000, -0x43a}, {0x2665000, -0x3fa}, {0x2468000, -0x3c2}, {0x2287000, -0x38e},
    {0x20c1000, -0x35e}, {0x1f12000, -0x332}, {0x1d79000, -0x30a}, {0x1bf4000, -0x2e6},
};

DOL_INLINE u64 gk_bits(f64 v) { union { f64 f; u64 u; } x; x.f = v; return x.u; }
DOL_INLINE f64 gk_dbl(u64 u)  { union { f64 f; u64 u; } x; x.u = u; return x.f; }

/* The hardware reciprocal estimate, bit-exact. */
DOL_INLINE f64 gekko_fres(f64 v)
{
    u64 in = gk_bits(v);
    u64 mant = in & 0x000FFFFFFFFFFFFFull;
    u64 sign = in & 0x8000000000000000ull;
    u64 exp  = in & 0x7FF0000000000000ull;
    int i;
    s64 est;

    if (mant == 0 && exp == 0)                  /* +-0 -> +-Inf */
        return gk_dbl(sign | 0x7FF0000000000000ull);
    if (exp == 0x7FF0000000000000ull) {
        if (mant == 0) return gk_dbl(sign);     /* +-Inf -> +-0 */
        return gk_dbl(in | DOUBLE_QBIT);        /* NaN quieted   */
    }
    if (exp < ((u64)895 << 52))                 /* tiny -> +-FLT_MAX */
        return sign ? -3.4028234663852886e38 : 3.4028234663852886e38;
    if (exp >= ((u64)1149 << 52))               /* huge -> +-0 */
        return gk_dbl(sign);

    exp = ((u64)0x7FD << 52) - exp;
    i   = (int)(mant >> 37);
    est = (s64)k_gekko_fres[i / 1024].base -
          ((s64)k_gekko_fres[i / 1024].dec * (i % 1024) + 1) / 2;
    return gk_dbl(sign | exp | ((u64)est << 29));
}

/* The hardware reciprocal-square-root estimate, bit-exact. */
DOL_INLINE f64 gekko_frsqrte(f64 v)
{
    u64 in = gk_bits(v);
    s64 mant = (s64)(in & 0x000FFFFFFFFFFFFFull);
    u64 sign = in & 0x8000000000000000ull;
    s64 exp  = (s64)(in & 0x7FF0000000000000ull);
    s64 exp_lsb;
    int i;
    s64 est;

    if (mant == 0 && exp == 0)                  /* +-0 -> +-Inf */
        return gk_dbl(sign | 0x7FF0000000000000ull);
    if ((u64)exp == 0x7FF0000000000000ull) {
        if (mant == 0)                          /* +Inf -> 0, -Inf -> NaN */
            return sign ? gk_dbl(0x7FF8000000000000ull) : 0.0;
        return gk_dbl(in | DOUBLE_QBIT);
    }
    if (sign)                                   /* negative -> default QNaN */
        return gk_dbl(0x7FF8000000000000ull);
    if (exp == 0) {                             /* normalise a denormal */
        do {
            exp  -= (s64)1 << 52;
            mant <<= 1;
        } while (!(mant & ((s64)1 << 52)));
        mant &= ((s64)1 << 52) - 1;
        exp  += (s64)1 << 52;
    }
    exp_lsb = exp & ((s64)1 << 52);
    exp = (((s64)0x3FF << 52) - ((exp - ((s64)0x3FE << 52)) / 2)) &
          (s64)0x7FF0000000000000ll;
    i   = (int)((u64)(exp_lsb | mant) >> 37);
    est = (s64)k_gekko_frsqrte[i / 2048].base +
          (s64)k_gekko_frsqrte[i / 2048].dec * (i % 2048);
    return gk_dbl(sign | (u64)exp | ((u64)est << 26));
}

/* Single-precision multiplies (fmuls, the fused singles, every paired-single
 * multiply) round operand C to 25 significant bits *before* multiplying. A
 * single already has 24, so this only bites when C holds a full double -- an
 * lfd-loaded constant, or the result of double arithmetic -- which is exactly
 * where a title's numbers silently drift from the console's. */
DOL_INLINE f64 gekko_force25(f64 v)
{
    u64 x = gk_bits(v);
    x = (x & 0xFFFFFFFFF8000000ull) + (x & 0x0000000008000000ull);
    return gk_dbl(x);
}

/* The fused single forms round the *exact* a*c+b once, to single. Computing
 * fma() in double and then rounding to single is a double rounding, and it is
 * wrong precisely when the double lands on a single's tie: recover the
 * discarded error with error-free transforms and nudge across the tie. */
/* A finite double whose discarded-by-single bits are exactly a half: the one
 * place where "round to double, then to single" can disagree with the
 * hardware's single rounding of the exact value. */
DOL_INLINE int gekko_on_single_tie(f64 r)
{
    u64 rb = gk_bits(r);
    return (rb & 0x000000001FFFFFFFull) == 0x0000000010000000ull &&
           ((rb >> 52) & 0x7FFull) != 0x7FFull;
}

/* err is the sign of (exact - r). One ulp toward the exact value takes r off
 * the tie, after which rounding to single lands where the hardware does. */
DOL_INLINE f64 gekko_tie_nudge(f64 r, f64 err)
{
    u64 rb = gk_bits(r);
    if (err == 0.0 || err != err)
        return r;
    return gk_dbl(((err > 0.0) == (r > 0.0)) ? rb + 1 : rb - 1);
}

DOL_INLINE f64 gekko_add_single(f64 a, f64 b)
{
    f64 s = a + b;
    if (gekko_on_single_tie(s)) {
        f64 bb = s - a;                 /* TwoSum: exact (a + b) - s */
        s = gekko_tie_nudge(s, (a - (s - bb)) + (b - bb));
    }
    return s;
}

DOL_INLINE f64 gekko_mul_single(f64 a, f64 c25)
{
    f64 r = a * c25;
    if (gekko_on_single_tie(r))
        r = gekko_tie_nudge(r, fma(a, c25, -r));    /* exact a*c - r */
    return r;
}

DOL_INLINE f64 gekko_div_single(f64 a, f64 b)
{
    f64 r = a / b;
    if (gekko_on_single_tie(r)) {
        f64 rem = fma(-r, b, a);        /* exact a - r*b; (exact - r) = rem/b */
        r = gekko_tie_nudge(r, (b > 0.0) ? rem : -rem);
    }
    return r;
}

DOL_INLINE f64 gekko_madd_single(f64 a, f64 c, f64 b)
{
    f64 r = fma(a, c, b);
    if (gekko_on_single_tie(r)) {
        f64 ap = b - r;                 /* a*c ~= -ap, both reachable exactly */
        f64 bp = r + ap;
        f64 da = fma(a, c, ap);         /* exact residual of the product part */
        f64 db = b - bp;                /* exact residual of the addend part  */
        r = gekko_tie_nudge(r, da + db);
    }
    return r;
}

/* fnmadd/fnmsub and ps_nmadd/ps_nmsub negate the result -- unless it is a
 * NaN, whose sign the hardware leaves alone. */
DOL_INLINE f64 gekko_neg_unless_nan(f64 v)
{
    return (v != v) ? v : -v;
}

#endif /* DOLPHIN_CORE_PPC_INTERP_FPUTIL_H */
