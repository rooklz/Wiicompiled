/* interp_float.c — scalar floating point.
 *
 * A note on fidelity, since this is the oracle the JIT is measured against:
 *
 * PowerPC's single-precision forms (`fadds` and friends) are defined as
 * "compute to infinite precision, then round once to single". C gives us
 * `(float)(a + b)`, which rounds to double and then to single -- a double
 * rounding that can differ in the last bit for operands that are themselves
 * full doubles. When the operands are single-precision values, which is the
 * case throughout paired-single game code, the intermediate double is exact and
 * the two formulations agree bit for bit.
 *
 * The JIT does not have this problem at all: it emits the hardware `fadds`,
 * which *is* the correct single rounding. So the residual risk is confined to
 * the interpreter, and the honest statement is that for full-double operands
 * the hardware test ROMs -- not this file -- are the authority. This is called
 * out in docs/ARCHITECTURE.md §8.1 rather than left as a silent assumption.
 */
#include "interp.h"
#include "interp_fputil.h"
#include "../../../common/log.h"

DOL_INLINE f64 rsingle(f64 v) { return (f64)(f32)v; }

DOL_INLINE f64 ps0f(const PPCState *s, u32 r) { return s->ps[r].ps0.f; }
DOL_INLINE u64 ps0u(const PPCState *s, u32 r) { return s->ps[r].ps0.u; }

DOL_INLINE void set_ps0(PPCState *s, u32 r, f64 v)
{
    s->ps[r].ps0.f = v;
}

/* Gekko writes a single-precision *result* to both halves of the pair: fadds,
 * fsubs, fmuls, fdivs, the fused singles, fres and frsp all Fill (Dolphin's
 * word, from its hardware tests). Double results, frsqrte, fsel and the moves
 * touch ps0 only. Getting this wrong leaves ps1 holding whatever the register
 * last held, and the next ps_sum0/ps_merge1x/ps_muls1 that reads lane 1 turns
 * that stale value into an answer -- or a NaN. */
DOL_INLINE void set_fill(PPCState *s, u32 r, f64 v)
{
    s->ps[r].ps0.f = v;
    s->ps[r].ps1.f = v;
}

DOL_INLINE void set_result(PPCState *s, u32 r, f64 v, int single)
{
    if (single) set_fill(s, r, rsingle(v));
    else        set_ps0(s, r, v);
}

DOL_INLINE void set_nan(PPCState *s, u32 r, u64 nan, int single)
{
    s->ps[r].ps0.u = nan;
    if (single) s->ps[r].ps1.u = nan;
}

/* CR1 mirrors the FPSCR exception summary for Rc=1 floating-point forms. */
static void update_cr1(PPCState *s, u32 op)
{
    if (RC_BIT(op)) {
        u32 f = (s->fpscr >> 28) & 0xFu;    /* FX FEX VX OX */
        s->cr = cr_set_field(s->cr, 1, f);
    }
}

/* PowerPC NaN precedence: a signalling NaN operand wins (quieted), otherwise
 * the first quiet NaN in operand order. */
static int nan_result(u64 a, u64 b, u64 c, int have_c, u64 *out)
{
    if (ppc_is_snan(a)) { *out = a | DOUBLE_QBIT; return 1; }
    if (ppc_is_snan(b)) { *out = b | DOUBLE_QBIT; return 1; }
    if (have_c && ppc_is_snan(c)) { *out = c | DOUBLE_QBIT; return 1; }
    if (ppc_is_nan(a)) { *out = a; return 1; }
    if (ppc_is_nan(b)) { *out = b; return 1; }
    if (have_c && ppc_is_nan(c)) { *out = c; return 1; }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Arithmetic                                                           */
/* ------------------------------------------------------------------ */

#define FP_ARITH2(name, opexpr, single)                                     \
    void ppc_##name(PPCState *s, u32 op)                                    \
    {                                                                       \
        u64 ua = ps0u(s, FRA(op)), ub = ps0u(s, FRB(op)), nan;              \
        f64 a = ps0f(s, FRA(op)), b = ps0f(s, FRB(op)), r;                  \
        if (nan_result(ua, ub, 0, 0, &nan)) {                               \
            set_nan(s, FRT(op), nan, single);                                     \
            if (ppc_is_snan(ua) || ppc_is_snan(ub))                         \
                ppc_set_fpscr_exception(s, FPSCR_VXSNAN);                   \
        } else {                                                            \
            r = (opexpr);                                                   \
            set_result(s, FRT(op), r, single);                   \
        }                                                                   \
        ppc_set_fprf(s, ps0u(s, FRT(op)));                                  \
        update_cr1(s, op);                                                  \
    }

FP_ARITH2(fadd,  a + b, 0)
FP_ARITH2(fadds, gekko_add_single(a, b), 1)
FP_ARITH2(fsub,  a - b, 0)
FP_ARITH2(fsubs, gekko_add_single(a, -b), 1)
FP_ARITH2(fdiv,  a / b, 0)
FP_ARITH2(fdivs, gekko_div_single(a, b), 1)
#undef FP_ARITH2

/* Multiply takes its second operand from the FRC slot, not FRB. */
#define FP_MUL(name, single)                                                \
    void ppc_##name(PPCState *s, u32 op)                                    \
    {                                                                       \
        f64 cv = single ? gekko_force25(ps0f(s, FRC(op))) : ps0f(s, FRC(op)); \
        u64 ua = ps0u(s, FRA(op)), uc = gk_bits(cv), nan;                    \
        f64 r;                                                              \
        if (nan_result(ua, uc, 0, 0, &nan)) {                               \
            set_nan(s, FRT(op), nan, single);                               \
            if (ppc_is_snan(ua) || ppc_is_snan(uc))                         \
                ppc_set_fpscr_exception(s, FPSCR_VXSNAN);                   \
        } else {                                                            \
            r = single ? gekko_mul_single(ps0f(s, FRA(op)), cv)             \
                       : ps0f(s, FRA(op)) * cv;                             \
            set_result(s, FRT(op), r, single);                              \
        }                                                                   \
        ppc_set_fprf(s, ps0u(s, FRT(op)));                                  \
        update_cr1(s, op);                                                  \
    }

FP_MUL(fmul,  0)
FP_MUL(fmuls, 1)
#undef FP_MUL

/* Multiply-add. These are *fused* on PowerPC: the product is not rounded before
 * the addition. C's fma() has exactly that contract, which is why it is used
 * here rather than a * c + b. */
#include <math.h>

#define FP_MADD(name, negate, subtract, single)                             \
    void ppc_##name(PPCState *s, u32 op)                                    \
    {                                                                       \
        f64 cv = single ? gekko_force25(ps0f(s, FRC(op))) : ps0f(s, FRC(op)); \
        u64 ua = ps0u(s, FRA(op)), ub = ps0u(s, FRB(op)), uc = gk_bits(cv), nan; \
        f64 r;                                                              \
        if (nan_result(ua, ub, uc, 1, &nan)) {                              \
            set_nan(s, FRT(op), nan, single);                               \
            if (ppc_is_snan(ua) || ppc_is_snan(ub) || ppc_is_snan(uc))      \
                ppc_set_fpscr_exception(s, FPSCR_VXSNAN);                   \
        } else {                                                            \
            f64 a = ps0f(s, FRA(op)), b = ps0f(s, FRB(op));                 \
            if (subtract) b = -b;                                           \
            r = single ? gekko_madd_single(a, cv, b) : fma(a, cv, b);       \
            if (negate) r = gekko_neg_unless_nan(r);                        \
            set_result(s, FRT(op), r, single);                              \
        }                                                                   \
        ppc_set_fprf(s, ps0u(s, FRT(op)));                                  \
        update_cr1(s, op);                                                  \
    }

FP_MADD(fmadd,   0, 0, 0)  FP_MADD(fmadds,   0, 0, 1)
FP_MADD(fmsub,   0, 1, 0)  FP_MADD(fmsubs,   0, 1, 1)
FP_MADD(fnmadd,  1, 0, 0)  FP_MADD(fnmadds,  1, 0, 1)
FP_MADD(fnmsub,  1, 1, 0)  FP_MADD(fnmsubs,  1, 1, 1)
#undef FP_MADD

void ppc_fsqrt(PPCState *s, u32 op)
{
    set_ps0(s, FRT(op), sqrt(ps0f(s, FRB(op))));
    ppc_set_fprf(s, ps0u(s, FRT(op)));
    update_cr1(s, op);
}

/* fres and frsqrte are the hardware's *estimates*, reproduced bit-exactly
 * from the tables in interp_fputil.h. Titles feed them into normalisation and
 * lighting, and the physics of a title that syncs ghost replays across
 * consoles depends on every bit. The JIT hands both to this file. */
void ppc_fres(PPCState *s, u32 op)
{
    set_fill(s, FRT(op), gekko_fres(ps0f(s, FRB(op))));
    ppc_set_fprf(s, ps0u(s, FRT(op)));
    update_cr1(s, op);
}

void ppc_frsqrte(PPCState *s, u32 op)
{
    set_ps0(s, FRT(op), gekko_frsqrte(ps0f(s, FRB(op))));
    ppc_set_fprf(s, ps0u(s, FRT(op)));
    update_cr1(s, op);
}

/* fsel is a branchless select: >= 0 picks FRC. Note it tests >= 0, so -0.0
 * selects FRC, and NaN selects FRB. */
void ppc_fsel(PPCState *s, u32 op)
{
    f64 a = ps0f(s, FRA(op));
    set_ps0(s, FRT(op), (a >= 0.0) ? ps0f(s, FRC(op)) : ps0f(s, FRB(op)));
    update_cr1(s, op);
}

/* ------------------------------------------------------------------ */
/* Moves and sign manipulation — these do not touch FPSCR              */
/* ------------------------------------------------------------------ */

void ppc_fmr(PPCState *s, u32 op)
{ s->ps[FRT(op)].ps0.u = ps0u(s, FRB(op)); update_cr1(s, op); }

void ppc_fneg(PPCState *s, u32 op)
{ s->ps[FRT(op)].ps0.u = ps0u(s, FRB(op)) ^ DOUBLE_SIGN; update_cr1(s, op); }

void ppc_fabs(PPCState *s, u32 op)
{ s->ps[FRT(op)].ps0.u = ps0u(s, FRB(op)) & ~DOUBLE_SIGN; update_cr1(s, op); }

void ppc_fnabs(PPCState *s, u32 op)
{ s->ps[FRT(op)].ps0.u = ps0u(s, FRB(op)) | DOUBLE_SIGN; update_cr1(s, op); }

void ppc_frsp(PPCState *s, u32 op)
{
    set_fill(s, FRT(op), rsingle(ps0f(s, FRB(op))));
    ppc_set_fprf(s, ps0u(s, FRT(op)));
    update_cr1(s, op);
}

/* ------------------------------------------------------------------ */
/* Conversion to integer                                                */
/*                                                                      */
/* The result lands in the *low half* of the FPR's bit pattern, which is why    */
/* stfiwx exists to store it. Out-of-range and NaN inputs saturate.             */
/* ------------------------------------------------------------------ */

static void fctiw_common(PPCState *s, u32 op, int round_to_zero)
{
    f64 b = ps0f(s, FRB(op));
    u64 ub = ps0u(s, FRB(op));
    s32 r;

    if (ppc_is_nan(ub)) {
        r = (s32)0x80000000;
        ppc_set_fpscr_exception(s, FPSCR_VXCVI);
    } else if (b >  2147483647.0) {
        r = 0x7FFFFFFF;
        ppc_set_fpscr_exception(s, FPSCR_VXCVI);
    } else if (b < -2147483648.0) {
        r = (s32)0x80000000;
        ppc_set_fpscr_exception(s, FPSCR_VXCVI);
    } else {
        r = round_to_zero ? (s32)b : (s32)nearbyint(b);
    }
    /* The high half is undefined on hardware; zeroing keeps runs reproducible
     * for the differential harness. */
    s->ps[FRT(op)].ps0.u = (u64)(u32)r;
    update_cr1(s, op);
}

void ppc_fctiw (PPCState *s, u32 op) { fctiw_common(s, op, 0); }
void ppc_fctiwz(PPCState *s, u32 op) { fctiw_common(s, op, 1); }

/* ------------------------------------------------------------------ */
/* Compare                                                              */
/* ------------------------------------------------------------------ */

static void fcmp_common(PPCState *s, u32 op, int ordered)
{
    u64 ua = ps0u(s, FRA(op)), ub = ps0u(s, FRB(op));
    f64 a = ps0f(s, FRA(op)), b = ps0f(s, FRB(op));
    u32 f;

    if (ppc_is_nan(ua) || ppc_is_nan(ub)) {
        f = CR_SO;                              /* unordered */
        if (ppc_is_snan(ua) || ppc_is_snan(ub))
            ppc_set_fpscr_exception(s, FPSCR_VXSNAN);
        /* fcmpo additionally flags an invalid compare for quiet NaNs. */
        if (ordered)
            ppc_set_fpscr_exception(s, FPSCR_VXVC);
    } else {
        f = (a < b) ? CR_LT : ((a > b) ? CR_GT : CR_EQ);
    }
    s->cr = cr_set_field(s->cr, CRFD(op), f);
    s->fpscr = (s->fpscr & ~FPSCR_FPRF_MASK) | ((f & 0xF) << 12);
    /* Authoritative: supersedes anything compiled code deferred. The JIT's
     * compile_fcmp emits the same pair of writes inline. */
    s->fprf_src = s->fprf_ack = FPRF_SRC_NONE;
}

void ppc_fcmpu(PPCState *s, u32 op) { fcmp_common(s, op, 0); }
void ppc_fcmpo(PPCState *s, u32 op) { fcmp_common(s, op, 1); }

/* ------------------------------------------------------------------ */
/* FPSCR access                                                         */
/* ------------------------------------------------------------------ */

void ppc_mffs(PPCState *s, u32 op)
{
    ppc_fprf_sync(s);
    s->ps[FRT(op)].ps0.u = 0xFFF8000000000000ull | s->fpscr;
    update_cr1(s, op);
}

void ppc_mtfsf(PPCState *s, u32 op)
{
    u32 fm = FM(op), mask = 0, i;
    ppc_fprf_sync(s);   /* FPRF bits this mask does not cover must survive */
    for (i = 0; i < 8; i++)
        if (fm & (0x80u >> i))
            mask |= 0xFu << (28 - 4 * i);
    s->fpscr = (s->fpscr & ~mask) | ((u32)ps0u(s, FRB(op)) & mask);
    update_cr1(s, op);
}

void ppc_mtfsb0(PPCState *s, u32 op)
{ ppc_fprf_sync(s); s->fpscr &= ~(0x80000000u >> CRBD(op)); update_cr1(s, op); }

void ppc_mtfsb1(PPCState *s, u32 op)
{ ppc_fprf_sync(s); s->fpscr |= (0x80000000u >> CRBD(op)); update_cr1(s, op); }

void ppc_mtfsfi(PPCState *s, u32 op)
{
    u32 shift = 28 - 4 * CRFD(op);
    u32 v = (op >> 12) & 0xFu;
    ppc_fprf_sync(s);
    s->fpscr = (s->fpscr & ~(0xFu << shift)) | (v << shift);
    update_cr1(s, op);
}

void ppc_mcrfs(PPCState *s, u32 op)
{
    u32 shift = 28 - 4 * CRFS(op);
    u32 v;
    ppc_fprf_sync(s);
    v = (s->fpscr >> shift) & 0xFu;
    s->cr = cr_set_field(s->cr, CRFD(op), v);
    /* Reading a field clears its exception bits. */
    s->fpscr &= ~(0xFu << shift);
}
