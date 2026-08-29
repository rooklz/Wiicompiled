/* interp_integer.c — integer, logical, rotate, shift and compare instructions.
 *
 * Written to mirror the Gekko manual's own descriptions rather than to be
 * fast. Carry and overflow in particular are computed the long way, through a
 * 64-bit intermediate, because that formulation is the one that is obviously
 * right for every operand -- including the boundary cases (0, -1, INT_MIN) that
 * the differential tests hammer.
 */
#include "interp.h"
#include "../../../common/log.h"

/* ------------------------------------------------------------------ */
/* Shared condition/flag helpers                                        */
/* ------------------------------------------------------------------ */

void ppc_update_cr0_from(PPCState *s, u32 result)
{
    u32 f = ((s32)result < 0) ? CR_LT : ((result == 0) ? CR_EQ : CR_GT);
    if (s->xer_so)
        f |= CR_SO;
    s->cr = cr_set_field(s->cr, 0, f);
}

/* XER[OV] is set per-instruction; XER[SO] is *sticky* and only cleared by an
 * explicit write to XER. Forgetting the stickiness is a classic emulator bug:
 * it shows up as rare, title-specific arithmetic drift. */
void ppc_set_ov_so(PPCState *s, int overflow)
{
    s->xer_ov = overflow ? 1u : 0u;
    if (overflow)
        s->xer_so = 1u;
}

/* PowerPC MASK(mb, me): bits mb..me inclusive, MSB-first. When mb > me the
 * range wraps around the end of the word. */
static u32 ppc_mask32(u32 mb, u32 me)
{
    u32 from_mb = 0xFFFFFFFFu >> mb;
    u32 to_me   = (me >= 31) ? 0xFFFFFFFFu : ~(0xFFFFFFFFu >> (me + 1));
    return (mb <= me) ? (from_mb & to_me) : (from_mb | to_me);
}

static void finish(PPCState *s, u32 op, u32 ra_index, u32 result)
{
    s->gpr[ra_index] = result;
    if (RC_BIT(op))
        ppc_update_cr0_from(s, result);
}

/* ------------------------------------------------------------------ */
/* Add / subtract                                                       */
/* ------------------------------------------------------------------ */

/* Signed overflow for a + b -> r: both operands agree in sign and the result
 * disagrees with them. */
static int add_overflowed(u32 a, u32 b, u32 r)
{
    return (int)(((a ^ r) & (b ^ r)) >> 31);
}

void ppc_addi(PPCState *s, u32 op)
{
    u32 a = RA(op);
    s->gpr[RT(op)] = (a ? s->gpr[a] : 0u) + (u32)SIMM(op);
}

void ppc_addis(PPCState *s, u32 op)
{
    u32 a = RA(op);
    s->gpr[RT(op)] = (a ? s->gpr[a] : 0u) + (UIMM(op) << 16);
}

void ppc_addic(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)];
    u64 t = (u64)a + (u64)(u32)SIMM(op);
    s->gpr[RT(op)] = (u32)t;
    s->xer_ca = (u32)(t >> 32);
}

void ppc_addic_rc(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)];
    u64 t = (u64)a + (u64)(u32)SIMM(op);
    s->gpr[RT(op)] = (u32)t;
    s->xer_ca = (u32)(t >> 32);
    ppc_update_cr0_from(s, (u32)t);
}

void ppc_add(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], b = s->gpr[RB(op)];
    u32 r = a + b;
    if (OE_BIT(op))
        ppc_set_ov_so(s, add_overflowed(a, b, r));
    finish(s, op, RT(op), r);
}

void ppc_addc(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], b = s->gpr[RB(op)];
    u64 t = (u64)a + (u64)b;
    s->xer_ca = (u32)(t >> 32);
    if (OE_BIT(op))
        ppc_set_ov_so(s, add_overflowed(a, b, (u32)t));
    finish(s, op, RT(op), (u32)t);
}

void ppc_adde(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], b = s->gpr[RB(op)], c = s->xer_ca;
    u64 t = (u64)a + (u64)b + (u64)c;
    s->xer_ca = (u32)(t >> 32);
    if (OE_BIT(op))
        ppc_set_ov_so(s, add_overflowed(a, b, (u32)t));
    finish(s, op, RT(op), (u32)t);
}

void ppc_addme(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], c = s->xer_ca;
    u64 t = (u64)a + 0xFFFFFFFFull + (u64)c;
    s->xer_ca = (u32)(t >> 32);
    if (OE_BIT(op))
        ppc_set_ov_so(s, add_overflowed(a, 0xFFFFFFFFu, (u32)t));
    finish(s, op, RT(op), (u32)t);
}

void ppc_addze(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], c = s->xer_ca;
    u64 t = (u64)a + (u64)c;
    s->xer_ca = (u32)(t >> 32);
    if (OE_BIT(op))
        ppc_set_ov_so(s, add_overflowed(a, 0, (u32)t));
    finish(s, op, RT(op), (u32)t);
}

/* subf computes RB - RA, which the ISA defines as ~RA + RB + 1. Keeping that
 * spelling makes the carry-out fall straight out of the 64-bit sum. */
void ppc_subf(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], b = s->gpr[RB(op)];
    u32 r = b - a;
    if (OE_BIT(op))
        ppc_set_ov_so(s, (int)(((b ^ a) & (b ^ r)) >> 31));
    finish(s, op, RT(op), r);
}

void ppc_subfc(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], b = s->gpr[RB(op)];
    u64 t = (u64)(u32)~a + (u64)b + 1ull;
    s->xer_ca = (u32)(t >> 32);
    if (OE_BIT(op))
        ppc_set_ov_so(s, (int)(((b ^ a) & (b ^ (u32)t)) >> 31));
    finish(s, op, RT(op), (u32)t);
}

void ppc_subfe(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], b = s->gpr[RB(op)], c = s->xer_ca;
    u64 t = (u64)(u32)~a + (u64)b + (u64)c;
    s->xer_ca = (u32)(t >> 32);
    if (OE_BIT(op))
        ppc_set_ov_so(s, (int)(((b ^ a) & (b ^ (u32)t)) >> 31));
    finish(s, op, RT(op), (u32)t);
}

void ppc_subfic(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], imm = (u32)SIMM(op);
    u64 t = (u64)(u32)~a + (u64)imm + 1ull;
    s->gpr[RT(op)] = (u32)t;
    s->xer_ca = (u32)(t >> 32);
}

void ppc_subfme(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], c = s->xer_ca;
    u64 t = (u64)(u32)~a + 0xFFFFFFFFull + (u64)c;
    s->xer_ca = (u32)(t >> 32);
    if (OE_BIT(op))
        ppc_set_ov_so(s, (int)(((0xFFFFFFFFu ^ a) & (0xFFFFFFFFu ^ (u32)t)) >> 31));
    finish(s, op, RT(op), (u32)t);
}

void ppc_subfze(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], c = s->xer_ca;
    u64 t = (u64)(u32)~a + (u64)c;
    s->xer_ca = (u32)(t >> 32);
    if (OE_BIT(op))
        ppc_set_ov_so(s, (int)((a & (u32)t) >> 31));
    finish(s, op, RT(op), (u32)t);
}

void ppc_neg(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)];
    u32 r = ~a + 1u;
    /* The single overflow case: negating INT_MIN. */
    if (OE_BIT(op))
        ppc_set_ov_so(s, a == 0x80000000u);
    finish(s, op, RT(op), r);
}

/* ------------------------------------------------------------------ */
/* Multiply / divide                                                    */
/* ------------------------------------------------------------------ */

void ppc_mulli(PPCState *s, u32 op)
{
    s->gpr[RT(op)] = (u32)((s32)s->gpr[RA(op)] * SIMM(op));
}

void ppc_mullw(PPCState *s, u32 op)
{
    s64 t = (s64)(s32)s->gpr[RA(op)] * (s64)(s32)s->gpr[RB(op)];
    if (OE_BIT(op))
        ppc_set_ov_so(s, t != (s64)(s32)t);
    finish(s, op, RT(op), (u32)t);
}

void ppc_mulhw(PPCState *s, u32 op)
{
    s64 t = (s64)(s32)s->gpr[RA(op)] * (s64)(s32)s->gpr[RB(op)];
    finish(s, op, RT(op), (u32)((u64)t >> 32));
}

void ppc_mulhwu(PPCState *s, u32 op)
{
    u64 t = (u64)s->gpr[RA(op)] * (u64)s->gpr[RB(op)];
    finish(s, op, RT(op), (u32)(t >> 32));
}

/* Division by zero, and INT_MIN / -1, leave RT *undefined* on hardware and set
 * OV. Dolphin and real titles both rely on the emulator not trapping here, so
 * we produce a defined value and carry on. */
void ppc_divw(PPCState *s, u32 op)
{
    s32 a = (s32)s->gpr[RA(op)], b = (s32)s->gpr[RB(op)];
    u32 r;
    int ov = 0;

    if (b == 0 || (a == (s32)0x80000000 && b == -1)) {
        ov = 1;
        r = (a < 0) ? 0xFFFFFFFFu : 0u;
    } else {
        r = (u32)(a / b);
    }
    if (OE_BIT(op))
        ppc_set_ov_so(s, ov);
    finish(s, op, RT(op), r);
}

void ppc_divwu(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], b = s->gpr[RB(op)];
    u32 r;
    int ov = 0;

    if (b == 0) { ov = 1; r = 0; }
    else        { r = a / b; }

    if (OE_BIT(op))
        ppc_set_ov_so(s, ov);
    finish(s, op, RT(op), r);
}

/* ------------------------------------------------------------------ */
/* Logical                                                              */
/*                                                                      */
/* Note the operand direction: these write RA from RS and RB, the reverse of    */
/* the arithmetic instructions above.                                           */
/* ------------------------------------------------------------------ */

#define LOGICAL_RR(name, expr)                                              \
    void ppc_##name(PPCState *s, u32 op)                                    \
    {                                                                       \
        u32 rs = s->gpr[RS(op)], rb = s->gpr[RB(op)];                       \
        u32 r = (expr);                                                     \
        s->gpr[RA(op)] = r;                                                 \
        if (RC_BIT(op)) ppc_update_cr0_from(s, r);                          \
    }

LOGICAL_RR(and,   rs & rb)
LOGICAL_RR(andc,  rs & ~rb)
LOGICAL_RR(nand, ~(rs & rb))
LOGICAL_RR(or,    rs | rb)
LOGICAL_RR(orc,   rs | ~rb)
LOGICAL_RR(nor,  ~(rs | rb))
LOGICAL_RR(xor,   rs ^ rb)
LOGICAL_RR(eqv,  ~(rs ^ rb))
#undef LOGICAL_RR

void ppc_andi_rc(PPCState *s, u32 op)
{
    u32 r = s->gpr[RS(op)] & UIMM(op);
    s->gpr[RA(op)] = r;
    ppc_update_cr0_from(s, r);
}

void ppc_andis_rc(PPCState *s, u32 op)
{
    u32 r = s->gpr[RS(op)] & (UIMM(op) << 16);
    s->gpr[RA(op)] = r;
    ppc_update_cr0_from(s, r);
}

void ppc_ori(PPCState *s, u32 op)
{ s->gpr[RA(op)] = s->gpr[RS(op)] | UIMM(op); }
void ppc_oris(PPCState *s, u32 op)
{ s->gpr[RA(op)] = s->gpr[RS(op)] | (UIMM(op) << 16); }
void ppc_xori(PPCState *s, u32 op)
{ s->gpr[RA(op)] = s->gpr[RS(op)] ^ UIMM(op); }
void ppc_xoris(PPCState *s, u32 op)
{ s->gpr[RA(op)] = s->gpr[RS(op)] ^ (UIMM(op) << 16); }

void ppc_extsb(PPCState *s, u32 op)
{
    u32 r = (u32)(s32)(s8)s->gpr[RS(op)];
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

void ppc_extsh(PPCState *s, u32 op)
{
    u32 r = (u32)(s32)(s16)s->gpr[RS(op)];
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

void ppc_cntlzw(PPCState *s, u32 op)
{
    u32 r = dol_clz32(s->gpr[RS(op)]);
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

/* ------------------------------------------------------------------ */
/* Shifts                                                               */
/*                                                                      */
/* PowerPC shifts use a *6-bit* count, so a shift of 32..63 produces zero (or a */
/* full sign fill) rather than the undefined behaviour C would give.            */
/* ------------------------------------------------------------------ */

void ppc_slw(PPCState *s, u32 op)
{
    u32 n = s->gpr[RB(op)] & 0x3Fu;
    u32 r = (n < 32) ? (s->gpr[RS(op)] << n) : 0u;
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

void ppc_srw(PPCState *s, u32 op)
{
    u32 n = s->gpr[RB(op)] & 0x3Fu;
    u32 r = (n < 32) ? (s->gpr[RS(op)] >> n) : 0u;
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

/* sraw sets CA when the value was negative *and* any 1 bit was shifted out --
 * the rule that makes `srawi; addze` a correct signed divide-by-power-of-two. */
void ppc_sraw(PPCState *s, u32 op)
{
    u32 n  = s->gpr[RB(op)] & 0x3Fu;
    s32 rs = (s32)s->gpr[RS(op)];
    u32 r;

    if (n >= 32) {
        r = (rs < 0) ? 0xFFFFFFFFu : 0u;
        s->xer_ca = (rs < 0) ? 1u : 0u;
    } else {
        r = (u32)(rs >> n);
        s->xer_ca = (rs < 0 && (s->gpr[RS(op)] & ((1u << n) - 1u)) != 0) ? 1u : 0u;
    }
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

void ppc_srawi(PPCState *s, u32 op)
{
    u32 n  = SH(op);
    s32 rs = (s32)s->gpr[RS(op)];
    u32 r  = (u32)(rs >> n);

    s->xer_ca = (rs < 0 && (s->gpr[RS(op)] & ((1u << n) - 1u)) != 0) ? 1u : 0u;
    if (n == 0)
        s->xer_ca = 0;
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

/* ------------------------------------------------------------------ */
/* Rotate and mask                                                      */
/* ------------------------------------------------------------------ */

void ppc_rlwinm(PPCState *s, u32 op)
{
    u32 m = ppc_mask32(MB(op), ME(op));
    u32 r = dol_rotl32(s->gpr[RS(op)], SH(op)) & m;
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

void ppc_rlwnm(PPCState *s, u32 op)
{
    u32 m = ppc_mask32(MB(op), ME(op));
    u32 r = dol_rotl32(s->gpr[RS(op)], s->gpr[RB(op)] & 31u) & m;
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

void ppc_rlwimi(PPCState *s, u32 op)
{
    u32 m = ppc_mask32(MB(op), ME(op));
    u32 r = (dol_rotl32(s->gpr[RS(op)], SH(op)) & m) | (s->gpr[RA(op)] & ~m);
    s->gpr[RA(op)] = r;
    if (RC_BIT(op)) ppc_update_cr0_from(s, r);
}

/* ------------------------------------------------------------------ */
/* Compare                                                              */
/* ------------------------------------------------------------------ */

static void set_cr_cmp(PPCState *s, u32 crf, int lt, int gt)
{
    u32 f = lt ? CR_LT : (gt ? CR_GT : CR_EQ);
    if (s->xer_so)
        f |= CR_SO;
    s->cr = cr_set_field(s->cr, crf, f);
}

void ppc_cmp(PPCState *s, u32 op)
{
    s32 a = (s32)s->gpr[RA(op)], b = (s32)s->gpr[RB(op)];
    set_cr_cmp(s, CRFD(op), a < b, a > b);
}

void ppc_cmpi(PPCState *s, u32 op)
{
    s32 a = (s32)s->gpr[RA(op)], b = SIMM(op);
    set_cr_cmp(s, CRFD(op), a < b, a > b);
}

void ppc_cmpl(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], b = s->gpr[RB(op)];
    set_cr_cmp(s, CRFD(op), a < b, a > b);
}

void ppc_cmpli(PPCState *s, u32 op)
{
    u32 a = s->gpr[RA(op)], b = UIMM(op);
    set_cr_cmp(s, CRFD(op), a < b, a > b);
}

/* ------------------------------------------------------------------ */
/* Traps                                                                */
/*                                                                      */
/* Debug builds of retail titles do use these, and a `trap` that silently does  */
/* nothing turns an assertion failure into a mysterious hang later.             */
/* ------------------------------------------------------------------ */

static int trap_taken(u32 to, s32 a, s32 b)
{
    if ((to & 0x10) && a <  b) return 1;
    if ((to & 0x08) && a >  b) return 1;
    if ((to & 0x04) && a == b) return 1;
    if ((to & 0x02) && (u32)a <  (u32)b) return 1;
    if ((to & 0x01) && (u32)a >  (u32)b) return 1;
    return 0;
}

void ppc_tw(PPCState *s, u32 op)
{
    if (trap_taken(RT(op), (s32)s->gpr[RA(op)], (s32)s->gpr[RB(op)]))
        ppc_raise(s, EXC_PROGRAM);
}

void ppc_twi(PPCState *s, u32 op)
{
    if (trap_taken(RT(op), (s32)s->gpr[RA(op)], SIMM(op)))
        ppc_raise(s, EXC_PROGRAM);
}
