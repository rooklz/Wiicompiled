/* interp_branch.c — branches, condition-register logic, and system call.
 *
 * The BO field encoding is the fiddliest part of the PowerPC branch unit and
 * the place emulators most often go subtly wrong, so it is decoded once here,
 * bit by named bit, and every branch form goes through it.
 */
#include "interp.h"
#include "../../../common/log.h"

/* ------------------------------------------------------------------ */
/* BO decoding                                                          */
/*                                                                      */
/*   bit 0 (0x10)  ignore the CR condition                              */
/*   bit 1 (0x08)  the CR value to test for                             */
/*   bit 2 (0x04)  do not decrement CTR                                 */
/*   bit 3 (0x02)  test CTR == 0 (rather than != 0)                     */
/*   bit 4 (0x01)  static prediction hint; no architectural effect      */
/* ------------------------------------------------------------------ */

#define BO_IGNORE_CR   0x10u
#define BO_CR_VALUE    0x08u
#define BO_NO_CTR_DEC  0x04u
#define BO_CTR_IS_ZERO 0x02u

/* CR bits are numbered MSB-first: bit n is (cr >> (31 - n)) & 1. */
DOL_INLINE u32 cr_bit(const PPCState *s, u32 n)
{
    return (s->cr >> (31u - n)) & 1u;
}

DOL_INLINE void cr_set_bit(PPCState *s, u32 n, u32 v)
{
    u32 mask = 1u << (31u - n);
    s->cr = v ? (s->cr | mask) : (s->cr & ~mask);
}

/* Evaluates the branch condition and applies the CTR decrement as a side
 * effect, which is architecturally part of the test. */
static int branch_taken(PPCState *s, u32 bo, u32 bi)
{
    int ctr_ok, cond_ok;

    if (!(bo & BO_NO_CTR_DEC))
        s->ctr--;

    ctr_ok = (bo & BO_NO_CTR_DEC) ||
             ((s->ctr != 0) ^ ((bo & BO_CTR_IS_ZERO) != 0));

    cond_ok = (bo & BO_IGNORE_CR) ||
              (cr_bit(s, bi) == ((bo & BO_CR_VALUE) >> 3));

    return ctr_ok && cond_ok;
}

/* ------------------------------------------------------------------ */
/* Branch forms                                                         */
/* ------------------------------------------------------------------ */

void ppc_b(PPCState *s, u32 op)
{
    s32 disp = LI(op);
    u32 target = AA_BIT(op) ? (u32)disp : (s->pc + (u32)disp);

    if (LK_BIT(op))
        s->lr = s->pc + 4;
    s->npc = target;
}

void ppc_bc(PPCState *s, u32 op)
{
    s32 disp = BD(op);
    u32 target = AA_BIT(op) ? (u32)disp : (s->pc + (u32)disp);

    /* The link register is written whether or not the branch is taken. */
    if (LK_BIT(op))
        s->lr = s->pc + 4;

    if (branch_taken(s, BO(op), BI(op)))
        s->npc = target;
}

void ppc_bclr(PPCState *s, u32 op)
{
    /* Sample LR before the link update: `bclrl` returns to the *old* LR. */
    u32 target = s->lr & ~3u;
    int taken = branch_taken(s, BO(op), BI(op));

    if (LK_BIT(op))
        s->lr = s->pc + 4;
    if (taken) {
        if (target == 0)
            LOG_WARN(LOG_INTERP, "blr to 0 FROM pc=%08x sp=%08x r3=%08x r13=%08x",
                     s->pc, s->gpr[1], s->gpr[3], s->gpr[13]);
        s->npc = target;
    }
}

void ppc_bcctr(PPCState *s, u32 op)
{
    u32 target = s->ctr & ~3u;
    /* bcctr must not decrement CTR -- it is the branch target. Encodings that
     * request it are invalid; hardware ignores the decrement. */
    int taken = (BO(op) & BO_IGNORE_CR) ||
                (cr_bit(s, BI(op)) == ((BO(op) & BO_CR_VALUE) >> 3));

    if (LK_BIT(op))
        s->lr = s->pc + 4;
    if (taken) {
        if (target == 0)
            LOG_WARN(LOG_INTERP, "bctr to 0 FROM pc=%08x sp=%08x r3=%08x",
                     s->pc, s->gpr[1], s->gpr[3]);
        s->npc = target;
    }
}

void ppc_sc(PPCState *s, u32 op)
{
    (void)op;
    ppc_raise(s, EXC_SYSCALL);
}

/* ------------------------------------------------------------------ */
/* Condition register logic                                             */
/* ------------------------------------------------------------------ */

#define CR_LOGICAL(name, expr)                                              \
    void ppc_##name(PPCState *s, u32 op)                                    \
    {                                                                       \
        u32 a = cr_bit(s, CRBA(op)), b = cr_bit(s, CRBB(op));               \
        cr_set_bit(s, CRBD(op), (expr) & 1u);                               \
    }

CR_LOGICAL(crand,   a & b)
CR_LOGICAL(cror,    a | b)
CR_LOGICAL(crxor,   a ^ b)
CR_LOGICAL(crnand, ~(a & b))
CR_LOGICAL(crnor,  ~(a | b))
CR_LOGICAL(creqv,  ~(a ^ b))
CR_LOGICAL(crandc,  a & ~b)
CR_LOGICAL(crorc,   a | ~b)
#undef CR_LOGICAL

void ppc_mcrf(PPCState *s, u32 op)
{
    s->cr = cr_set_field(s->cr, CRFD(op), cr_get_field(s->cr, CRFS(op)));
}

/* isync flushes the guest's pipeline. We have no speculative guest state, but
 * it is a hint that instruction memory may have changed, which the JIT uses as
 * a cue to check for invalidated blocks. */
void ppc_isync(PPCState *s, u32 op)
{
    (void)s; (void)op;
}
