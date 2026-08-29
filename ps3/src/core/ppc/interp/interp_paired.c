/* interp_paired.c — Gekko paired singles and quantized load/store.
 *
 * This is the instruction group that makes Gekko not-quite-a-750: two
 * single-precision lanes packed into one FPR, plus load/store instructions that
 * quantize to and from 8- and 16-bit integer formats using a scale factor held
 * in one of eight GQR registers.
 *
 * Every lane operation below is "compute, then round to single". On the Cell
 * PPE that is precisely what `fadds`/`fmuls`/`fmadds` do, so the JIT emits two
 * native instructions per paired op and is bit-exact by construction -- no
 * conversion, no flush-to-zero mismatch, no vector/scalar shuffling. It is the
 * single clearest reason this host suits this guest (docs/HARDWARE.md §1.4).
 */
#include "interp.h"
#include "interp_fputil.h"
#include "../../mem/memmap.h"
#include "../../../common/log.h"

#include <math.h>

DOL_INLINE f64 rsingle(f64 v) { return (f64)(f32)v; }

DOL_INLINE f64 A0(const PPCState *s, u32 op) { return s->ps[FRA(op)].ps0.f; }
DOL_INLINE f64 A1(const PPCState *s, u32 op) { return s->ps[FRA(op)].ps1.f; }
DOL_INLINE f64 B0(const PPCState *s, u32 op) { return s->ps[FRB(op)].ps0.f; }
DOL_INLINE f64 B1(const PPCState *s, u32 op) { return s->ps[FRB(op)].ps1.f; }
DOL_INLINE f64 C0(const PPCState *s, u32 op) { return s->ps[FRC(op)].ps0.f; }
DOL_INLINE f64 C1(const PPCState *s, u32 op) { return s->ps[FRC(op)].ps1.f; }

DOL_INLINE void put(PPCState *s, u32 op, f64 v0, f64 v1)
{
    s->ps[FRT(op)].ps0.f = v0;
    s->ps[FRT(op)].ps1.f = v1;
}

/* Executing a paired-single opcode with HID2[PSE] clear is a program
 * exception, not a silently-executed instruction. */
static int ps_enabled(PPCState *s)
{
    if (LIKELY(ppc_paired_single_enabled(s)))
        return 1;
    ppc_raise(s, EXC_PROGRAM);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Lane arithmetic                                                      */
/* ------------------------------------------------------------------ */

#define PS_OP2(name, e0, e1)                                                \
    void ppc_##name(PPCState *s, u32 op)                                    \
    {                                                                       \
        if (!ps_enabled(s)) return;                                         \
        put(s, op, rsingle(e0), rsingle(e1));                               \
    }

PS_OP2(ps_add, gekko_add_single(A0(s,op), B0(s,op)), gekko_add_single(A1(s,op), B1(s,op)))
PS_OP2(ps_sub, gekko_add_single(A0(s,op), -B0(s,op)), gekko_add_single(A1(s,op), -B1(s,op)))
PS_OP2(ps_mul, gekko_mul_single(A0(s,op), gekko_force25(C0(s,op))), gekko_mul_single(A1(s,op), gekko_force25(C1(s,op))))
PS_OP2(ps_div, gekko_div_single(A0(s,op), B0(s,op)), gekko_div_single(A1(s,op), B1(s,op)))

/* The "s0"/"s1" forms broadcast one lane of FRC to both multiplies -- the
 * scalar-times-vector primitive that makes these useful for transforms. */
PS_OP2(ps_muls0, gekko_mul_single(A0(s,op), gekko_force25(C0(s,op))), gekko_mul_single(A1(s,op), gekko_force25(C0(s,op))))
PS_OP2(ps_muls1, gekko_mul_single(A0(s,op), gekko_force25(C1(s,op))), gekko_mul_single(A1(s,op), gekko_force25(C1(s,op))))

/* Fused, like their scalar counterparts. */
#define MADD(a, c, b) gekko_madd_single((a), gekko_force25(c), (b))
PS_OP2(ps_madd,   MADD(A0(s,op), C0(s,op),  B0(s,op)),
                  MADD(A1(s,op), C1(s,op),  B1(s,op)))
PS_OP2(ps_msub,   MADD(A0(s,op), C0(s,op), -B0(s,op)),
                  MADD(A1(s,op), C1(s,op), -B1(s,op)))
PS_OP2(ps_nmadd,  gekko_neg_unless_nan(MADD(A0(s,op), C0(s,op),  B0(s,op))),
                  gekko_neg_unless_nan(MADD(A1(s,op), C1(s,op),  B1(s,op))))
PS_OP2(ps_nmsub,  gekko_neg_unless_nan(MADD(A0(s,op), C0(s,op), -B0(s,op))),
                  gekko_neg_unless_nan(MADD(A1(s,op), C1(s,op), -B1(s,op))))
PS_OP2(ps_madds0, MADD(A0(s,op), C0(s,op),  B0(s,op)),
                  MADD(A1(s,op), C0(s,op),  B1(s,op)))
PS_OP2(ps_madds1, MADD(A0(s,op), C1(s,op),  B0(s,op)),
                  MADD(A1(s,op), C1(s,op),  B1(s,op)))
#undef MADD

/* Cross-lane sums: the building block for dot products. */
PS_OP2(ps_sum0, A0(s,op) + B1(s,op), C1(s,op))
PS_OP2(ps_sum1, C0(s,op),            A0(s,op) + B1(s,op))
#undef PS_OP2

void ppc_ps_res(PPCState *s, u32 op)
{
    if (!ps_enabled(s)) return;
    put(s, op, gekko_fres(B0(s, op)), gekko_fres(B1(s, op)));
}

void ppc_ps_rsqrte(PPCState *s, u32 op)
{
    if (!ps_enabled(s)) return;
    put(s, op, rsingle(gekko_frsqrte(B0(s, op))), rsingle(gekko_frsqrte(B1(s, op))));
}

void ppc_ps_sel(PPCState *s, u32 op)
{
    if (!ps_enabled(s)) return;
    put(s, op,
        (A0(s, op) >= 0.0) ? C0(s, op) : B0(s, op),
        (A1(s, op) >= 0.0) ? C1(s, op) : B1(s, op));
}

/* ------------------------------------------------------------------ */
/* Lane movement — bit operations, so they go through the raw patterns  */
/* ------------------------------------------------------------------ */

#define PS_MOVE(name, u0, u1)                                               \
    void ppc_##name(PPCState *s, u32 op)                                    \
    {                                                                       \
        u64 a0 = s->ps[FRA(op)].ps0.u, a1 = s->ps[FRA(op)].ps1.u;           \
        u64 b0 = s->ps[FRB(op)].ps0.u, b1 = s->ps[FRB(op)].ps1.u;           \
        u64 r0, r1;                                                         \
        (void)a0; (void)a1; (void)b0; (void)b1;                             \
        if (!ps_enabled(s)) return;                                         \
        r0 = (u0); r1 = (u1);                                               \
        s->ps[FRT(op)].ps0.u = r0;                                          \
        s->ps[FRT(op)].ps1.u = r1;                                          \
    }

PS_MOVE(ps_merge00, a0, b0)
PS_MOVE(ps_merge01, a0, b1)
PS_MOVE(ps_merge10, a1, b0)
PS_MOVE(ps_merge11, a1, b1)
PS_MOVE(ps_mr,      b0, b1)
PS_MOVE(ps_neg,     b0 ^ DOUBLE_SIGN,  b1 ^ DOUBLE_SIGN)
PS_MOVE(ps_abs,     b0 & ~DOUBLE_SIGN, b1 & ~DOUBLE_SIGN)
PS_MOVE(ps_nabs,    b0 | DOUBLE_SIGN,  b1 | DOUBLE_SIGN)
#undef PS_MOVE

/* ------------------------------------------------------------------ */
/* Lane compare                                                         */
/* ------------------------------------------------------------------ */

static void ps_cmp(PPCState *s, u32 op, int lane, int ordered)
{
    f64 a = lane ? A1(s, op) : A0(s, op);
    f64 b = lane ? B1(s, op) : B0(s, op);
    u64 ua = lane ? s->ps[FRA(op)].ps1.u : s->ps[FRA(op)].ps0.u;
    u64 ub = lane ? s->ps[FRB(op)].ps1.u : s->ps[FRB(op)].ps0.u;
    u32 f;

    if (ppc_is_nan(ua) || ppc_is_nan(ub)) {
        f = CR_SO;
        if (ppc_is_snan(ua) || ppc_is_snan(ub))
            ppc_set_fpscr_exception(s, FPSCR_VXSNAN);
        if (ordered)
            ppc_set_fpscr_exception(s, FPSCR_VXVC);
    } else {
        f = (a < b) ? CR_LT : ((a > b) ? CR_GT : CR_EQ);
    }
    s->cr = cr_set_field(s->cr, CRFD(op), f);
}

void ppc_ps_cmpu0(PPCState *s, u32 op) { ps_cmp(s, op, 0, 0); }
void ppc_ps_cmpo0(PPCState *s, u32 op) { ps_cmp(s, op, 0, 1); }
void ppc_ps_cmpu1(PPCState *s, u32 op) { ps_cmp(s, op, 1, 0); }
void ppc_ps_cmpo1(PPCState *s, u32 op) { ps_cmp(s, op, 1, 1); }

/* ------------------------------------------------------------------ */
/* Quantized load / store                                               */
/*                                                                      */
/* The GQR scale is a 6-bit *signed* exponent. Dequantizing multiplies by       */
/* 2^-scale and quantizing by 2^+scale, with saturation to the target range.    */
/* ------------------------------------------------------------------ */

static f64 dequantize(u32 ea, u32 type, s32 exp, unsigned index)
{
    f64 v;

    switch (type) {
    case QUANT_U8:  v = (f64)(u8) mem_read8 (ea + index);      break;
    case QUANT_S8:  v = (f64)(s8) mem_read8 (ea + index);      break;
    case QUANT_U16: v = (f64)(u16)mem_read16(ea + index * 2);  break;
    case QUANT_S16: v = (f64)(s16)mem_read16(ea + index * 2);  break;
    default: {
        /* f32 -- and the three reserved type encodings, which hardware treats
         * as f32. No scaling is applied, which is why this case is both the
         * most common and the cheapest: on the JIT path it is a single `lfs`. */
        FPReg r;
        r.u = ppc_convert_to_double(mem_read32(ea + index * 4));
        return r.f;
    }
    }
    /* Round the scaled result to SINGLE precision.
     *
     * psq_l dequantizes to single -- the paired-single register holds two
     * singles -- so a scale that pushes the value past the single range must
     * saturate to infinity, exactly as it does on hardware. Returning the
     * unrounded double kept values just above 3.4e38 finite here while the
     * compiled path (which converts through a single) produced infinity: a
     * divergence that only existed because this side was the more precise one.
     *
     * Found by the differential fuzzer the moment HID2[LSQE] was set in its
     * state, which is what finally let it compile a quantized load at all
     * (docs/PLAN.md §16). */
    return (f64)(f32)ldexp(v, -exp);
}

static void quantize(u32 ea, u32 type, s32 exp, unsigned index, f64 value)
{
    f64 scaled;

    if (type < QUANT_U8) {          /* f32 and the reserved encodings */
        FPReg r;
        r.f = value;
        mem_write32(ea + index * 4, ppc_convert_to_single(r.u));
        return;
    }

    scaled = ldexp(value, exp);
    switch (type) {
    case QUANT_U8:
        if (!(scaled >= 0.0)) scaled = 0.0;          /* also catches NaN */
        if (scaled > 255.0)   scaled = 255.0;
        mem_write8(ea + index, (u8)scaled);
        break;
    case QUANT_S8:
        if (!(scaled >= -128.0)) scaled = -128.0;
        if (scaled > 127.0)      scaled = 127.0;
        mem_write8(ea + index, (u8)(s8)scaled);
        break;
    case QUANT_U16:
        if (!(scaled >= 0.0))   scaled = 0.0;
        if (scaled > 65535.0)   scaled = 65535.0;
        mem_write16(ea + index * 2, (u16)scaled);
        break;
    case QUANT_S16:
        if (!(scaled >= -32768.0)) scaled = -32768.0;
        if (scaled > 32767.0)      scaled = 32767.0;
        mem_write16(ea + index * 2, (u16)(s16)scaled);
        break;
    default:
        break;
    }
}

static void psq_load(PPCState *s, u32 op, u32 ea, u32 gqr_index, u32 width_one)
{
    u32 gqr  = s->gqr[gqr_index];
    u32 type = gqr_ld_type(gqr);
    s32 exp  = gqr_scale_exp(gqr_ld_scale(gqr));

    if (!ps_enabled(s))
        return;

    if (width_one) {
        /* W=1 loads a single value into ps0 and forces ps1 to 1.0, which is
         * what makes psq_l useful for loading 3-component vectors. */
        s->ps[FRT(op)].ps0.f = dequantize(ea, type, exp, 0);
        s->ps[FRT(op)].ps1.f = 1.0;
    } else {
        /* Both lanes are read before either is written: FRT may alias the
         * register the address came from. */
        f64 v0 = dequantize(ea, type, exp, 0);
        f64 v1 = dequantize(ea, type, exp, 1);
        s->ps[FRT(op)].ps0.f = v0;
        s->ps[FRT(op)].ps1.f = v1;
    }
}

static void psq_store(PPCState *s, u32 op, u32 ea, u32 gqr_index, u32 width_one)
{
    u32 gqr  = s->gqr[gqr_index];
    u32 type = gqr_st_type(gqr);
    s32 exp  = gqr_scale_exp(gqr_st_scale(gqr));

    if (!ps_enabled(s))
        return;

    quantize(ea, type, exp, 0, s->ps[FRT(op)].ps0.f);
    if (!width_one)
        quantize(ea, type, exp, 1, s->ps[FRT(op)].ps1.f);
}

void ppc_psq_l(PPCState *s, u32 op)
{
    u32 ea = ea_ra_or_0(s, op) + (u32)PS_D(op);
    psq_load(s, op, ea, PS_I(op), PS_W(op));
}

void ppc_psq_lu(PPCState *s, u32 op)
{
    u32 ea = s->gpr[RA(op)] + (u32)PS_D(op);
    psq_load(s, op, ea, PS_I(op), PS_W(op));
    s->gpr[RA(op)] = ea;
}

void ppc_psq_lx(PPCState *s, u32 op)
{
    u32 ea = ea_x(s, op);
    psq_load(s, op, ea, PSX_I(op), PSX_W(op));
}

void ppc_psq_lux(PPCState *s, u32 op)
{
    u32 ea = s->gpr[RA(op)] + s->gpr[RB(op)];
    psq_load(s, op, ea, PSX_I(op), PSX_W(op));
    s->gpr[RA(op)] = ea;
}

void ppc_psq_st(PPCState *s, u32 op)
{
    u32 ea = ea_ra_or_0(s, op) + (u32)PS_D(op);
    psq_store(s, op, ea, PS_I(op), PS_W(op));
}

void ppc_psq_stu(PPCState *s, u32 op)
{
    u32 ea = s->gpr[RA(op)] + (u32)PS_D(op);
    psq_store(s, op, ea, PS_I(op), PS_W(op));
    s->gpr[RA(op)] = ea;
}

void ppc_psq_stx(PPCState *s, u32 op)
{
    u32 ea = ea_x(s, op);
    psq_store(s, op, ea, PSX_I(op), PSX_W(op));
}

void ppc_psq_stux(PPCState *s, u32 op)
{
    u32 ea = s->gpr[RA(op)] + s->gpr[RB(op)];
    psq_store(s, op, ea, PSX_I(op), PSX_W(op));
    s->gpr[RA(op)] = ea;
}
