/* difftest.c — run guest programs through both engines and compare state.
 *
 * Each case is a short guest program assembled with the shared emitter (which
 * is itself verified against llvm-mc). It is executed from identical starting
 * state by the interpreter and by the JIT, and every architecturally visible
 * register is compared afterwards. Comparing the *whole* state rather than a
 * chosen result is deliberate: recompiler bugs characteristically show up in
 * something the test author was not looking at -- a carry bit, a condition
 * field, a register the block spilled incorrectly.
 */
#include "difftest.h"
#include "interp/interp.h"
#include "interp/interp_fputil.h"
#include "jit/jit.h"
#include "jit/ppc_emitter.h"
#include "../mem/memmap.h"
#include "../../common/log.h"

#include <stdio.h>
#include <string.h>

#define DT_CODE_BASE 0x80300000u
#define DT_DATA_BASE 0x80310000u

typedef struct {
    const char *name;
    void (*build)(PPCEmitter *e);
    void (*setup)(PPCState *s);
    unsigned steps;
} DiffCase;

/* ------------------------------------------------------------------ */
/* Guest programs                                                       */
/* ------------------------------------------------------------------ */

static void b_arith(PPCEmitter *e)
{
    e_add(e, 5, 3, 4);
    e_subf(e, 6, 3, 4);
    e_or(e, 7, 5, 6);
    e_and(e, 8, 5, 6);
    e_xor(e, 9, 7, 8);
    e_addi(e, 10, 9, 0x123);
    e_mullw(e, 11, 10, 3);
    e_neg(e, 12, 11);
    e_blr(e);
}
static void s_arith(PPCState *s)
{
    s->gpr[3] = 0x12345678u;
    s->gpr[4] = 0x0BADF00Du;
}

static void b_shifts(PPCEmitter *e)
{
    e_slw(e, 5, 3, 4);
    e_srw(e, 6, 3, 4);
    e_rlwinm(e, 7, 3, 8, 0, 31);
    e_rlwinm(e, 8, 3, 16, 24, 31);
    e_rlwimi(e, 9, 3, 4, 8, 15);
    e_extsb(e, 10, 3);
    e_extsh(e, 11, 3);
    e_cntlzw(e, 12, 3);
    e_blr(e);
}
static void s_shifts(PPCState *s)
{
    s->gpr[3] = 0xDEADBEEFu;
    s->gpr[4] = 5;
    s->gpr[9] = 0xFFFFFFFFu;
}

/* Condition-register traffic: the guest CR lives in the host CR inside a
 * block, so this exercises the load/spill boundary as much as the compares. */
static void b_compare(PPCEmitter *e)
{
    e_cmpw(e, 0, 3, 4);
    e_cmplw(e, 1, 3, 4);
    e_cmpwi(e, 2, 3, -1);
    e_cmplwi(e, 3, 4, 0x8000);
    e_and_(e, 5, 3, 4);         /* Rc=1: writes CR0 */
    e_blr(e);
}
static void s_compare(PPCState *s)
{
    s->gpr[3] = 0xFFFFFFFFu;    /* -1 signed, huge unsigned */
    s->gpr[4] = 0x00000001u;
}

/* Memory: exercises the address fold and the hoisted base pointer. */
static void b_memory(PPCEmitter *e)
{
    e_lwz(e, 5, 0, 3);
    e_lwz(e, 6, 4, 3);
    e_lbz(e, 7, 8, 3);
    e_lhz(e, 8, 10, 3);
    e_lha(e, 9, 10, 3);
    e_add(e, 10, 5, 6);
    e_stw(e, 10, 16, 3);
    e_stb(e, 7, 20, 3);
    e_sth(e, 8, 22, 3);
    e_lwz(e, 11, 16, 3);        /* read back what we stored */
    e_blr(e);
}
static void s_memory(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE;
    mem_write32(DT_DATA_BASE + 0,  0x11223344u);
    mem_write32(DT_DATA_BASE + 4,  0x55667788u);
    mem_write32(DT_DATA_BASE + 8,  0x99AABBCCu);
    mem_write32(DT_DATA_BASE + 12, 0xDDEEFF00u);
}

/* The update forms, which write the effective address back to RA.
 *
 * `stwu r1,-N(r1)` is the standard PowerPC function prologue and the update
 * loads are how the compiler walks arrays, so these decide whether ordinary
 * function entry stays in compiled code. Both the loaded/stored value and the
 * updated pointer matter, so each pointer is left live in its own register and
 * the stored bytes are read back. Negative displacements are included because
 * the prologue's is negative. */
static void b_dform_update(PPCEmitter *e)
{
    e_mr(e, 4, 3);
    e_lwzu(e, 5, 4, 4);         /* r5 = [base+4],  r4 = base+4  */
    e_lwzu(e, 6, 4, 4);         /* r6 = [base+8],  r4 = base+8  */
    e_lwzu(e, 7, -8, 4);        /* r7 = [base+0],  r4 = base+0  */
    e_lbzu(e, 8, 9, 4);         /* r8 = byte,      r4 = base+9  */
    e_lhzu(e, 9, 1, 4);         /* r9 = half,      r4 = base+10 */
    e_lhau(e, 10, 0, 4);        /* r10 = signed half, r4 unchanged in value */

    /* Stores, then read the same bytes back through a second pointer. */
    e_mr(e, 11, 3);
    e_stwu(e, 5, 32, 11);       /* [base+32] = r5, r11 = base+32 */
    e_stbu(e, 8, 4, 11);        /* [base+36] = byte, r11 = base+36 */
    e_sthu(e, 9, 2, 11);        /* [base+38] = half, r11 = base+38 */
    e_mr(e, 12, 3);
    e_lwzu(e, 13, 32, 12);
    e_lbzu(e, 14, 4, 12);
    e_lhzu(e, 15, 2, 12);

    /* The prologue shape itself: a negative update, work, then the unwind. */
    e_mr(e, 16, 3);
    e_stwu(e, 3, -16, 16);      /* r16 = base-16, [base-16] = base */
    e_lwz(e, 17, 0, 16);
    e_addi(e, 16, 16, 16);      /* epilogue: undo the frame */

    /* Float update forms, on their own pointer so every address stays
     * naturally aligned: Gekko raises an alignment exception for misaligned
     * floating-point accesses, and that belongs in its own test rather than
     * being stumbled into here. */
    e_mr(e, 18, 3);
    e_lfsu(e, 1, 128, 18);      /* r18 = base+128 */
    e_lfdu(e, 2, 8, 18);        /* r18 = base+136 */
    e_stfsu(e, 1, 8, 18);       /* r18 = base+144 */
    e_stfdu(e, 2, 8, 18);       /* r18 = base+152 */
    e_blr(e);
}
static void s_dform_update(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE + 64;
    mem_write32(DT_DATA_BASE + 64 + 0, 0x11223344u);
    mem_write32(DT_DATA_BASE + 64 + 4, 0x55667788u);
    mem_write32(DT_DATA_BASE + 64 + 8, 0x99AABBCCu);
    mem_write32(DT_DATA_BASE + 64 - 16, 0xDEADBEEFu);
    mem_write32(DT_DATA_BASE + 64 + 128, 0x3F800000u);   /* 1.0f       */
    mem_write32(DT_DATA_BASE + 64 + 136, 0x40091EB8u);   /* ~3.14, hi  */
    mem_write32(DT_DATA_BASE + 64 + 140, 0x51EB851Fu);   /* ~3.14, lo  */
}

/* addic/addic./subfic/mulli, whose carry rules are the easiest to get subtly
 * wrong: the immediate is sign-extended but the carry is defined on the
 * 32-bit unsigned addition, and a negative immediate makes it a *borrow*.
 * XER[CA] is only compared at the end, so each intermediate carry is captured
 * into its own register with addze. */
static void b_imm_carry(PPCEmitter *e)
{
    e_li(e, 3, 1);
    e_addic(e, 4, 3, 5);        /* 6, no carry out */
    e_li(e, 5, 0);
    e_addze(e, 5, 5);           /* -> 0 */

    e_lis(e, 6, 0xFFFF);
    e_ori(e, 6, 6, 0xFFFF);     /* r6 = 0xFFFFFFFF */
    e_addic(e, 7, 6, 1);        /* wraps to 0, carry out */
    e_li(e, 8, 0);
    e_addze(e, 8, 8);           /* -> 1 */

    e_li(e, 9, 10);
    e_addic(e, 10, 9, -3);      /* 7, no borrow -> CA=1 */
    e_li(e, 11, 0);
    e_addze(e, 11, 11);         /* -> 1 */

    e_li(e, 12, 0);
    e_addic(e, 13, 12, -1);     /* 0xFFFFFFFF, borrows -> CA=0 */
    e_li(e, 14, 0);
    e_addze(e, 14, 14);         /* -> 0 */

    e_li(e, 15, 5);
    e_addic_(e, 16, 15, -5);    /* 0: sets CR0 EQ as well as CA */

    e_li(e, 17, 3);
    e_subfic(e, 18, 17, 10);    /* 7, 10 >= 3 -> CA=1 */
    e_li(e, 19, 0);
    e_addze(e, 19, 19);

    e_li(e, 20, 10);
    e_subfic(e, 21, 20, 3);     /* 3 < 10 -> CA=0 */
    e_li(e, 22, 0);
    e_addze(e, 22, 22);

    e_li(e, 23, 7);
    e_subfic(e, 24, 23, -2);    /* 0xFFFFFFFE - 7 */

    e_li(e, 25, 7);
    e_mulli(e, 26, 25, 100);
    e_mulli(e, 27, 25, -3);
    e_blr(e);
}
static void s_imm_carry(PPCState *s) { (void)s; }

/* mflr/mtlr/mfctr/mtctr. Every non-leaf function opens with `mflr r0` and
 * closes with `mtlr r0`, so these decide whether function entry and exit stay
 * in compiled code. The SPR number is encoded with its two halves swapped,
 * which is the easy thing to get wrong -- and getting it wrong would silently
 * address the wrong field rather than fault. */
static void b_spr_move(PPCEmitter *e)
{
    e_li(e, 3, 0x1234);
    e_mtlr(e, 3);
    e_mflr(e, 4);               /* r4 = 0x1234 */
    e_li(e, 5, 0x5678);
    e_mtctr(e, 5);
    e_mfctr(e, 6);              /* r6 = 0x5678 */
    e_mtlr(e, 6);
    e_mflr(e, 7);               /* r7 = 0x5678 */
    e_add(e, 8, 4, 7);
    e_li(e, 9, 0);
    e_mtlr(e, 9);               /* leave LR where the harness found it */
    e_blr(e);
}
static void s_spr_move(PPCState *s) { (void)s; }

/* CTR-decrementing branches -- bdnz and bdz. The back-edge of every counting
 * loop the compiler emits is a bdnz, so this is the branch that decides whether
 * loops stay in compiled code. The first loop also stores inside the body, to
 * exercise the register flush that must happen before the branch; the second
 * is a bdz countdown. CTR is read back so a wrong final counter is visible. */
static void b_ctr_loop(PPCEmitter *e)
{
    /* bdnz: r5 += 3, seven times. */
    e_li(e, 5, 0);
    e_li(e, 6, 7);
    e_mtctr(e, 6);
    e_addi(e, 5, 5, 3);             /* loop body */
    e_bc(e, BO_DNZ, 0, -4);         /* bdnz back to the addi -> r5 = 21 */
    e_mfctr(e, 9);                  /* CTR decremented to 0 */

    /* bdnz with a store in the body, so the flush-before-branch path runs. */
    e_li(e, 7, 4);
    e_mtctr(e, 7);
    e_li(e, 10, 100);
    e_addi(e, 10, 10, 5);
    e_stw(e, 10, 0, 3);
    e_bc(e, BO_DNZ, 0, -8);         /* r10 = 120, [data] = 120 */
    e_lwz(e, 11, 0, 3);

    /* bdz countdown: body runs (ctr-1) times. */
    e_li(e, 12, 3);
    e_mtctr(e, 12);
    e_li(e, 13, 0);
    e_bc(e, BO_DZ, 0, 12);          /* bdz exit (skip addi + b) when ctr==0 */
    e_addi(e, 13, 13, 1);
    e_b(e, -8);                     /* loop back to the bdz -> r13 = 2 */
    e_mfctr(e, 14);                 /* 0 */
    e_blr(e);
}
static void s_ctr_loop(PPCState *s) { s->gpr[3] = DT_DATA_BASE; }

/* A self-loop whose body begins at the block entry, so the bdnz targets the
 * entry -- exactly the shape loop register retention compiles: the guest
 * registers and CTR are kept live across the back-edge, spilled only on exit.
 * Sums an array through a moving pointer, so the retained pointer, accumulator
 * and a per-iteration memory load are all exercised, and the sum is stored and
 * read back so the exit path's spill is checked too. */
static void b_retain_loop(PPCEmitter *e)
{
    e_lwz(e, 5, 0, 3);          /* body: load [cursor]    */
    e_add(e, 6, 6, 5);          /* accumulate             */
    e_addi(e, 3, 3, 4);         /* advance the cursor      */
    e_bc(e, BO_DNZ, 0, -12);    /* bdnz back to the lwz    */
    e_stw(e, 6, 0, 4);          /* loop done: store sum    */
    e_lwz(e, 7, 0, 4);          /* read it back            */
    e_blr(e);
}
static void s_retain_loop(PPCState *s)
{
    unsigned i;
    s->gpr[3] = DT_DATA_BASE;
    s->gpr[6] = 0;
    s->gpr[4] = DT_DATA_BASE + 128;
    s->ctr    = 8;
    for (i = 0; i < 8; i++)
        mem_write32(DT_DATA_BASE + i * 4, (i + 1) * 10);   /* sum = 360 */
}

/* A cmp/bc self-loop from the block entry -- the shape loop retention now
 * compiles for compare-driven loops: registers stay live across the back-edge,
 * the CR is preloaded once and spilled per iteration. Walks a pointer to an
 * end address, summing, exactly like the pointer-comparison loops compilers
 * emit for iterator-style code. */
static void b_retain_cmp(PPCEmitter *e)
{
    e_lwz(e, 5, 0, 3);          /* body: load [cursor]        */
    e_add(e, 6, 6, 5);          /* accumulate                 */
    e_addi(e, 3, 3, 4);         /* advance                    */
    e_cmpw(e, 0, 3, 4);         /* cursor vs end              */
    e_bc(e, BO_FALSE, BI_EQ(0), -16);   /* loop while cursor != end */
    e_stw(e, 6, 0, 7);          /* store the sum              */
    e_lwz(e, 8, 0, 7);          /* read it back               */
    e_blr(e);
}
static void s_retain_cmp(PPCState *s)
{
    unsigned i;
    s->gpr[3] = DT_DATA_BASE;
    s->gpr[4] = DT_DATA_BASE + 6 * 4;   /* six elements */
    s->gpr[6] = 0;
    s->gpr[7] = DT_DATA_BASE + 128;
    for (i = 0; i < 6; i++)
        mem_write32(DT_DATA_BASE + i * 4, (i + 1) * 100);  /* sum = 2100 */
}

/* Multiply-high (mulhw/mulhwu) and the carry-extend tail (addze/subfze). GCC
 * emits exactly this shape for division by a constant: a reciprocal multiply,
 * the high word, then a carry fix-up. Real integer code is dense with it. */
static void b_mulhigh(PPCEmitter *e)
{
    e_mulhw(e, 5, 3, 4);            /* signed high word   */
    e_mulhwu(e, 6, 3, 4);          /* unsigned high word */
    e_mulhw(e, 7, 4, 4);
    e_mulhwu(e, 8, 3, 3);

    /* Produce a carry, then consume it with addze and subfze. */
    e_lis(e, 9, 0xFFFF);
    e_ori(e, 9, 9, 0xFFFF);        /* r9 = 0xFFFFFFFF */
    e_addic(e, 10, 9, 1);          /* wraps -> CA = 1 */
    e_li(e, 11, 40);
    e_addze(e, 12, 11);            /* 41 */
    e_li(e, 13, 40);
    e_subfze(e, 14, 13);           /* ~40 + CA */
    e_blr(e);
}
static void s_mulhigh(PPCState *s)
{
    s->gpr[3] = 0x12345678u;
    s->gpr[4] = 0xFFFFFFF0u;       /* negative signed, large unsigned */
}

/* Conditional return (bclr with a CR condition) -- how the compiler emits an
 * early return. Exercises both the not-taken path (bnelr falls through) and
 * the taken path (bnelr returns to LR), so a wrong condition sense shows up. */
static void b_cond_return(PPCEmitter *e)
{
    e_li(e, 5, 0);                 /*  0 */
    e_li(e, 6, 0);                 /*  4 */
    e_cmpw(e, 0, 3, 4);            /*  8: r3 == r4 */
    e_bclr(e, BO_FALSE, BI_EQ(0)); /* 12: bnelr -- equal, so NOT taken */
    e_addi(e, 5, 5, 1);            /* 16: runs -> r5 = 1 */
    e_cmpw(e, 0, 3, 7);            /* 20: r3 != r7 */
    e_bclr(e, BO_FALSE, BI_EQ(0)); /* 24: bnelr -- unequal, so taken -> LR */
    e_addi(e, 6, 6, 1);            /* 28: skipped -> r6 stays 0 */
    e_blr(e);                      /* 32 */
}
static void s_cond_return(PPCState *s)
{
    s->gpr[3] = 5;
    s->gpr[4] = 5;
    s->gpr[7] = 9;
    s->lr = DT_CODE_BASE + 32;     /* both returns converge on the final blr */
}

/* Stack-relative traffic through r1. The recompiler elides the MMIO guard on
 * the stack pointer (it is RAM by the ABI), so this confirms guarded and
 * unguarded bases produce identical results. Mixes r1 (elided) with r3
 * (guarded) in the same block. */
static void b_stack(PPCEmitter *e)
{
    e_stw(e, 3, 8, 1);          /* spill via the stack pointer */
    e_stw(e, 4, 12, 1);
    e_sth(e, 3, 18, 1);
    e_stb(e, 4, 23, 1);
    e_lwz(e, 5, 8, 1);          /* read them back */
    e_lwz(e, 6, 12, 1);
    e_lhz(e, 7, 18, 1);
    e_lbz(e, 8, 23, 1);
    e_add(e, 9, 5, 6);
    e_stw(e, 9, 0, 3);          /* r3 base stays guarded */
    e_lwz(e, 10, 0, 3);
    e_blr(e);
}
static void s_stack(PPCState *s)
{
    s->gpr[1] = DT_DATA_BASE + 128;     /* stack area */
    s->gpr[3] = DT_DATA_BASE;
    s->gpr[4] = 0x0BADF00Du;
}

/* Indexed addressing, which cannot use the hoisted base. */
static void b_memory_x(PPCEmitter *e)
{
    e_lwzx(e, 5, 3, 4);
    e_lbzx(e, 6, 3, 4);
    e_lhzx(e, 7, 3, 4);
    e_stwx(e, 5, 3, 6);
    e_blr(e);
}
static void s_memory_x(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE;
    s->gpr[4] = 8;
    s->gpr[6] = 32;
    mem_write32(DT_DATA_BASE + 8, 0xCAFEBABEu);
}

/* A taken branch, a not-taken branch and a loop: the paths block linking
 * rewrites. */
static void b_branches(PPCEmitter *e)
{
    e_li(e, 5, 0);
    e_li(e, 6, 4);
    /* loop: r5 += 3; r6 -= 1; if r6 != 0 loop */
    e_addi(e, 5, 5, 3);
    e_addic_(e, 6, 6, -1);
    e_bc(e, BO_FALSE, BI_EQ(0), -8);
    e_cmpwi(e, 0, 5, 12);
    e_bc(e, BO_TRUE, BI_EQ(0), 8);
    e_li(e, 7, 111);            /* skipped when the loop ran correctly */
    e_li(e, 8, 222);
    e_blr(e);
}
static void s_branches(PPCState *s) { (void)s; }

/* Scalar floating point, including the single<->double conversions. */
static void b_float(PPCEmitter *e)
{
    e_lfs(e, 1, 0, 3);
    e_lfs(e, 2, 4, 3);
    e_lfd(e, 3, 8, 3);
    e_fadds(e, 4, 1, 2);
    e_fmuls(e, 5, 1, 2);
    e_fmadds(e, 6, 1, 2, 4);
    e_fsubs(e, 7, 6, 5);
    e_fmr(e, 8, 7);
    e_fneg(e, 9, 8);
    e_fabs(e, 10, 9);
    e_stfs(e, 6, 16, 3);
    e_stfd(e, 3, 24, 3);
    e_blr(e);
}
static void s_float(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE;
    s->msr |= MSR_FP;
    mem_write32(DT_DATA_BASE + 0, 0x40490FDBu);   /* 3.14159f */
    mem_write32(DT_DATA_BASE + 4, 0x3F800000u);   /* 1.0f     */
    mem_write64(DT_DATA_BASE + 8, 0x400921FB54442D18ull); /* pi as double */
}

/* Floating-point comparison, and the state it leaves behind.
 *
 * fcmpu/fcmpo are the obvious next candidates for native compilation: each is
 * one host instruction for the comparison itself. What makes that unsafe
 * without a test is the *other* thing they write -- FPSCR's FPRF field, and the
 * invalid-operation flags when an operand is NaN. The host instruction updates
 * the host's FPSCR, not the emulated one, so a naive compilation would leave
 * the guest's FPSCR stale and only diverge in code that reads it afterwards.
 * This case reads it afterwards, on ordinary values, on a quiet NaN and on a
 * signalling NaN, so any such attempt has to be exactly right to pass. */
static void b_fcmp(PPCEmitter *e)
{
    e_lfd(e, 1, 0, 3);              /* 1.0                 */
    e_lfd(e, 2, 8, 3);              /* 2.0                 */
    e_lfd(e, 3, 16, 3);             /* quiet NaN           */
    e_lfd(e, 4, 24, 3);             /* signalling NaN      */

    e_fcmpu(e, 0, 1, 2);            /* 1 < 2  -> LT        */
    e_mffs(e, 5);                   /* capture FPSCR       */
    e_stfd(e, 5, 32, 3);

    e_fcmpu(e, 1, 2, 1);            /* 2 > 1  -> GT        */
    e_mffs(e, 6);
    e_stfd(e, 6, 40, 3);

    e_fcmpu(e, 2, 1, 1);            /* equal  -> EQ        */
    e_mffs(e, 7);
    e_stfd(e, 7, 48, 3);

    e_fcmpu(e, 3, 1, 3);            /* quiet NaN -> SO     */
    e_mffs(e, 8);
    e_stfd(e, 8, 56, 3);

    e_fcmpo(e, 4, 1, 3);            /* ordered vs qNaN: VXVC   */
    e_mffs(e, 9);
    e_stfd(e, 9, 64, 3);

    e_fcmpu(e, 5, 1, 4);            /* signalling NaN: VXSNAN  */
    e_mffs(e, 10);
    e_stfd(e, 10, 72, 3);

    /* Fold every comparison's CR bits into one register so a wrong CR field is
     * a wrong GPR, which the differential comparison reports directly. */
    e_mfcr(e, 4);
    e_blr(e);
}
static void s_fcmp(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE;
    s->msr |= MSR_FP;
    mem_write64(DT_DATA_BASE +  0, 0x3FF0000000000000ull);   /* 1.0        */
    mem_write64(DT_DATA_BASE +  8, 0x4000000000000000ull);   /* 2.0        */
    mem_write64(DT_DATA_BASE + 16, 0x7FF8000000000000ull);   /* quiet NaN  */
    mem_write64(DT_DATA_BASE + 24, 0x7FF4000000000000ull);   /* signalling */
}

/* Paired singles -- the port's signature path. */
static void ps_aform(PPCEmitter *e, u32 xo5, u32 d, u32 a, u32 b, u32 cq)
{
    emit_word(e, (4u << 26) | (d << 21) | (a << 16) | (b << 11) |
                 (cq << 6) | (xo5 << 1));
}

static void b_paired(PPCEmitter *e)
{
    e_lfs(e, 1, 0, 3);          /* lfs fills both halves */
    e_lfs(e, 2, 4, 3);
    ps_aform(e, 21, 4, 1, 2, 0);        /* ps_add  */
    ps_aform(e, 20, 5, 1, 2, 0);        /* ps_sub  */
    ps_aform(e, 25, 6, 1, 0, 2);        /* ps_mul  */
    ps_aform(e, 29, 7, 1, 4, 2);        /* ps_madd */
    ps_aform(e, 12, 8, 1, 0, 2);        /* ps_muls0 */
    e_blr(e);
}
static void s_paired(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE;
    s->msr |= MSR_FP;
    s->hid2 = HID2_PSE | HID2_LSQE;
    mem_write32(DT_DATA_BASE + 0, 0x40000000u);   /* 2.0f */
    mem_write32(DT_DATA_BASE + 4, 0x40A00000u);   /* 5.0f */
}

/* Quantized load/store. The JIT specializes on the GQR value seen at compile
 * time and guards it at runtime, so this covers both halves: the specialized
 * f32 path, and (via the second GQR) a format it must decline and hand to the
 * interpreter. Both engines must agree either way. */
static void psq_dform(PPCEmitter *e, u32 opcd, u32 frt, u32 ra, s32 d,
                      u32 i, u32 w)
{
    emit_word(e, (opcd << 26) | (frt << 21) | (ra << 16) | (w << 15) |
                 (i << 12) | ((u32)d & 0xFFFu));
}

static void b_quantized(PPCEmitter *e)
{
    psq_dform(e, 56, 1, 3, 0,  0, 0);   /* psq_l  f1, 0(r3),  w=0, GQR0 (f32)  */
    psq_dform(e, 56, 2, 3, 8,  0, 1);   /* psq_l  f2, 8(r3),  w=1 -> ps1 = 1.0 */
    psq_dform(e, 56, 3, 3, 16, 1, 0);   /* psq_l  f3, 16(r3), GQR1 (u8) -> slow */
    ps_aform(e, 21, 4, 1, 2, 0);        /* ps_add on the loaded values         */
    psq_dform(e, 60, 4, 3, 32, 0, 0);   /* psq_st f4, 32(r3), f32              */
    e_blr(e);
}
static void s_quantized(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE;
    s->msr |= MSR_FP;
    s->hid2 = HID2_PSE | HID2_LSQE;
    /* GQR0: f32, no scaling -- the case the JIT specializes.
     * GQR1: u8 with a scale -- the case it must decline. */
    s->gqr[0] = 0;
    s->gqr[1] = (QUANT_U8 << 16) | (3u << 24) | QUANT_U8 | (3u << 8);
    mem_write32(DT_DATA_BASE + 0,  0x40000000u);   /* 2.0f  */
    mem_write32(DT_DATA_BASE + 4,  0x40A00000u);   /* 5.0f  */
    mem_write32(DT_DATA_BASE + 8,  0x41200000u);   /* 10.0f */
    mem_write32(DT_DATA_BASE + 16, 0x20401020u);   /* bytes for the u8 path */
}

/* Instructions with no native path: verifies the interpreter-fallback plumbing
 * -- register spill, call, and cache invalidation on return. */
static void b_fallback(PPCEmitter *e)
{
    e_addc(e, 5, 3, 4);
    e_adde(e, 6, 3, 4);
    e_srawi(e, 7, 3, 4);
    e_sraw(e, 8, 3, 4);
    e_add(e, 9, 5, 6);          /* native op after a fallback */
    e_blr(e);
}
static void s_fallback(PPCState *s)
{
    s->gpr[3] = 0xFFFFFFFFu;
    s->gpr[4] = 0x00000002u;
}

/* A device (MMIO) access. The compiled code cannot perform this itself -- lv2
 * has no usable fault handler -- so it must detect the address, abandon the
 * block, and let the interpreter execute the instruction. Both engines must
 * still agree on the resulting state, which is what proves the bail-out path
 * restores registers correctly rather than merely not crashing. */
static void b_mmio(PPCEmitter *e)
{
    e_lwz(e, 5, 0, 3);          /* RAM   -- fast path                */
    e_lwz(e, 6, 0, 4);          /* MMIO  -- guard fires, deopts      */
    e_add(e, 7, 5, 6);          /* must resume correctly afterwards  */
    e_stw(e, 7, 4, 3);
    e_blr(e);
}
static void s_mmio(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE;
    s->gpr[4] = 0xCC000000u;    /* Video Interface register block */
    mem_write32(DT_DATA_BASE, 0x0000BEEFu);
}

/* More live guest registers than the cache has host slots, which forces
 * eviction and exercises the LRU and pinning logic. */
static void b_pressure(PPCEmitter *e)
{
    unsigned i;
    for (i = 3; i < 25; i++)
        e_addi(e, i, 3, (s32)(i * 7));
    for (i = 3; i < 24; i++)
        e_add(e, i, i, i + 1);
    e_blr(e);
}
static void s_pressure(PPCState *s) { s->gpr[3] = 0x1000; }

/* A conditional branch reached with registers still cached and dirty.
 *
 * "branches" above cannot cover this: it uses `addic.`, which the recompiler
 * does not compile, so the block boundary lands immediately before its `bc`
 * and the branch is always compiled into a fresh block with nothing dirty to
 * write back. A branch-codegen bug that discards the register flush on one
 * exit is therefore invisible to it -- which is exactly what happened.
 *
 * This case uses only instructions the recompiler handles, so the dirty
 * registers and the branch land in the *same* block, and both exits are
 * exercised: the compare below is arranged so the branch is NOT taken, which
 * is the path a forward-branch-to-the-fall-through-exit layout is most likely
 * to get wrong. r8/r9 then read the values back, so a lost writeback shows up
 * in more than one register.
 */
static void b_branch_exit(PPCEmitter *e)
{
    e_li(e, 5, 0);
    e_li(e, 6, 0);
    e_li(e, 7, 0);
    e_addi(e, 5, 5, 11);        /* r5, r6, r7 now cached and dirty */
    e_addi(e, 6, 6, 22);
    e_addi(e, 7, 7, 33);
    e_cmpwi(e, 0, 5, 11);       /* equal, so BO_FALSE does not branch */
    e_bc(e, BO_FALSE, BI_EQ(0), 8);
    e_add(e, 8, 5, 6);          /* 33 only if the writeback survived */
    e_add(e, 9, 7, 8);          /* 66 */
    /* Now the taken direction, still with dirty registers live. */
    e_addi(e, 10, 6, 5);
    e_cmpwi(e, 0, 6, 22);
    e_bc(e, BO_TRUE, BI_EQ(0), 8);
    e_li(e, 11, 999);           /* skipped when the branch is taken */
    e_addi(e, 12, 10, 1);
    e_blr(e);
}
static void s_branch_exit(PPCState *s) { (void)s; }

/* rlwimi patching a pointer that already has a hoisted address base.
 *
 * emit_addr_d caches MEMBASE + (RA & ARENA_MASK) in a dedicated host register
 * so a run of accesses off one pointer pays the fold and the MMIO guard once.
 * Every other path that writes a GPR drops that cache (gpr_write / gpr_dest,
 * and the psq update form by hand); rlwimi writes RA without going through
 * either, and for a long time did not.
 *
 * rlwimi IS the PowerPC bitfield-insert instruction, so "patch the low bits of
 * a pointer and dereference it" is not a contrived sequence -- it is what a
 * compiler emits for banked/aligned addressing and what hand-written pointer
 * tagging does. The consequence of a stale base is silent: a load reads the
 * OLD address, and a store WRITES to it.
 *
 * Everything here compiles natively on purpose. A single interpreter fallback
 * would wipe the whole register cache (rc_invalidate_all) and take the stale
 * slot with it, hiding the bug. */
static void b_rlwimi_base(PPCEmitter *e)
{
    e_lwz(e, 5, 0, 3);              /* hoists the base for r3            */
    e_lwz(e, 6, 4, 3);              /* second access reuses it (the point) */
    e_li(e, 7, 0x40);
    e_rlwimi(e, 3, 7, 0, 20, 31);   /* r3.low12 <- 0x040: r3 now +0x40   */
    e_lwz(e, 8, 0, 3);              /* must see the NEW address          */
    e_lwz(e, 9, 4, 3);
    e_or(e, 4, 3, 3);               /* an independent (fresh) base       */
    e_li(e, 10, 0x5A5A);
    e_stw(e, 10, 8, 3);             /* store through the patched pointer */
    e_lwz(e, 11, 8, 4);             /* read it back through the fresh one */
    e_blr(e);
}
static void s_rlwimi_base(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE;
    /* Distinct content at the old and the new address, so a stale base is a
     * wrong register value rather than a coincidence. */
    mem_write32(DT_DATA_BASE + 0x00, 0xA0A0A0A0u);
    mem_write32(DT_DATA_BASE + 0x04, 0xA1A1A1A1u);
    mem_write32(DT_DATA_BASE + 0x08, 0xA2A2A2A2u);
    mem_write32(DT_DATA_BASE + 0x40, 0xB0B0B0B0u);
    mem_write32(DT_DATA_BASE + 0x44, 0xB1B1B1B1u);
    mem_write32(DT_DATA_BASE + 0x48, 0xB2B2B2B2u);
}

/* FPSCR[FPRF] after ordinary FP arithmetic.
 *
 * The interpreter sets the 5-bit result class on every arithmetic result
 * (ppc_set_fprf). Compiled code emits the bare host instruction and records
 * the result for deferred classification; the classification itself happens in
 * ppc_fprf_sync, which every FPSCR reader calls. This case reads FPRF back
 * through both readers the architecture has -- mffs, into an FPR that the
 * state comparison covers, and mcrfs, into CR -- after results in four
 * different classes (negative normal, positive zero, positive infinity,
 * positive normal).
 *
 * The tail is the case a naive implementation gets wrong: an intervening
 * fcmpu makes FPRF authoritative again, and then the SAME arithmetic result
 * is produced a second time. A deferred scheme that only records "the last
 * value" and compares it against "the last value consumed" would decide
 * nothing had changed and leave the compare's FPCC in place. */
static void b_fprf(PPCEmitter *e)
{
    e_lfd(e, 1, 0, 3);              /*  1.5  */
    e_lfd(e, 2, 8, 3);              /* -2.5  */
    e_lfd(e, 3, 16, 3);             /*  0.0  */

    e_fadd(e, 4, 1, 2);             /* -1.0     -> FL           */
    e_mffs(e, 20);
    e_fsub(e, 5, 1, 1);             /* +0.0     -> FE           */
    e_mffs(e, 21);
    e_fdiv(e, 6, 1, 3);             /* +inf     -> FG|FU        */
    e_mffs(e, 22);
    e_fmul(e, 7, 2, 2);             /* +6.25    -> FG           */
    e_mffs(e, 23);
    e_frsp(e, 8, 2);                /* -2.5     -> FL           */
    e_mffs(e, 24);
    e_fmadd(e, 9, 1, 2, 3);         /* -3.75    -> FL           */
    e_mffs(e, 25);

    e_fadd(e, 10, 1, 2);            /* -1.0                     */
    e_fcmpu(e, 0, 1, 2);            /* FPRF <- the compare (GT) */
    e_mffs(e, 26);                  /* ... which mffs must see  */
    e_fadd(e, 11, 1, 2);            /* -1.0 again -> FL again   */
    e_mffs(e, 27);

    /* mcrfs 2,4: FPCC into CR2. Folded into a GPR so a wrong CR field is a
     * reported register difference. */
    emit_word(e, (63u << 26) | (2u << 23) | (4u << 18) | (64u << 1));
    e_mfcr(e, 4);
    e_blr(e);
}
static void s_fprf(PPCState *s)
{
    s->gpr[3] = DT_DATA_BASE;
    s->msr |= MSR_FP;
    mem_write64(DT_DATA_BASE +  0, 0x3FF8000000000000ull);   /*  1.5 */
    mem_write64(DT_DATA_BASE +  8, 0xC004000000000000ull);   /* -2.5 */
    mem_write64(DT_DATA_BASE + 16, 0x0000000000000000ull);   /*  0.0 */
}

static const DiffCase k_cases[] = {
    { "arith",     b_arith,     s_arith,     64 },
    { "shifts",    b_shifts,    s_shifts,    64 },
    { "compare",   b_compare,   s_compare,   64 },
    { "memory",    b_memory,    s_memory,    64 },
    { "memory-x",  b_memory_x,  s_memory_x,  64 },
    { "stack",     b_stack,     s_stack,     64 },
    { "d-update",  b_dform_update, s_dform_update, 128 },
    { "imm-carry", b_imm_carry, s_imm_carry, 128 },
    { "spr-move",  b_spr_move,  s_spr_move,  64 },
    { "ctr-loop",  b_ctr_loop,  s_ctr_loop, 128 },
    { "retain-loop", b_retain_loop, s_retain_loop, 128 },
    { "retain-cmp", b_retain_cmp, s_retain_cmp, 128 },
    { "mulhigh",   b_mulhigh,   s_mulhigh,   64 },
    { "cond-ret",  b_cond_return, s_cond_return, 64 },
    { "branches",  b_branches,  s_branches, 128 },
    { "branch-exit", b_branch_exit, s_branch_exit, 128 },
    { "float",     b_float,     s_float,     64 },
    { "fcmp",      b_fcmp,      s_fcmp,      64 },
    { "paired",    b_paired,    s_paired,    64 },
    { "fallback",  b_fallback,  s_fallback,  64 },
    { "mmio",      b_mmio,      s_mmio,      64 },
    { "quantized", b_quantized, s_quantized, 64 },
    { "pressure",  b_pressure,  s_pressure, 128 },
    { "rlwimi-base", b_rlwimi_base, s_rlwimi_base, 128 },
    { "fprf",      b_fprf,      s_fprf,     128 },
};

/* ------------------------------------------------------------------ */
/* State comparison                                                     */
/* ------------------------------------------------------------------ */

static int compare_state(const PPCState *a, const PPCState *b,
                         DiffOutFn out, void *ctx, const char *name)
{
    char line[160];
    int bad = 0;
    unsigned i;

#define REPORT(fmt, ...)                                                     \
    do {                                                                     \
        snprintf(line, sizeof line, "    %s: " fmt, name, __VA_ARGS__);      \
        out(ctx, line);                                                      \
        bad++;                                                               \
    } while (0)

    for (i = 0; i < 32; i++)
        if (a->gpr[i] != b->gpr[i])
            REPORT("r%u interp=%08x jit=%08x", i,
                   (unsigned)a->gpr[i], (unsigned)b->gpr[i]);

    for (i = 0; i < 32; i++) {
        if (a->ps[i].ps0.u != b->ps[i].ps0.u)
            REPORT("f%u.ps0 interp=%08x%08x jit=%08x%08x", i,
                   (unsigned)(a->ps[i].ps0.u >> 32), (unsigned)a->ps[i].ps0.u,
                   (unsigned)(b->ps[i].ps0.u >> 32), (unsigned)b->ps[i].ps0.u);
        if (a->ps[i].ps1.u != b->ps[i].ps1.u)
            REPORT("f%u.ps1 interp=%08x%08x jit=%08x%08x", i,
                   (unsigned)(a->ps[i].ps1.u >> 32), (unsigned)a->ps[i].ps1.u,
                   (unsigned)(b->ps[i].ps1.u >> 32), (unsigned)b->ps[i].ps1.u);
    }

    if (a->pc  != b->pc)  REPORT("pc interp=%08x jit=%08x",  (unsigned)a->pc,  (unsigned)b->pc);
    if (a->cr  != b->cr)  REPORT("cr interp=%08x jit=%08x",  (unsigned)a->cr,  (unsigned)b->cr);
    if (a->lr  != b->lr)  REPORT("lr interp=%08x jit=%08x",  (unsigned)a->lr,  (unsigned)b->lr);
    if (a->ctr != b->ctr) REPORT("ctr interp=%08x jit=%08x", (unsigned)a->ctr, (unsigned)b->ctr);
    if (a->xer_ca != b->xer_ca)
        REPORT("XER[CA] interp=%u jit=%u", (unsigned)a->xer_ca, (unsigned)b->xer_ca);
    if (a->xer_so != b->xer_so)
        REPORT("XER[SO] interp=%u jit=%u", (unsigned)a->xer_so, (unsigned)b->xer_so);
    /* FPSCR in full, FPRF included. Both states were folded by ppc_fprf_sync
     * before this call, which is exactly what a guest read of FPSCR does. */
    if (a->fpscr != b->fpscr)
        REPORT("fpscr interp=%08x jit=%08x",
               (unsigned)a->fpscr, (unsigned)b->fpscr);

#undef REPORT
    return bad;
}

/* ------------------------------------------------------------------ */

static void install(const DiffCase *c, u32 *scratch, unsigned words_max)
{
    PPCEmitter e;
    unsigned n, i;

    emit_init(&e, scratch, words_max * 4);
    c->build(&e);
    n = (unsigned)(emit_size(&e) / 4);

    /* Written through the guest accessors so the program lands in guest byte
     * order regardless of the host's. */
    for (i = 0; i < n; i++)
        mem_write32(DT_CODE_BASE + i * 4, scratch[i]);
}

static void init_state(PPCState *s, const DiffCase *c)
{
    memset(s, 0, sizeof *s);
    s->msr = MSR_FP;
    /* All of the JIT's in-state constants, not just const_one: the scale
     * tables too. Setting only const_one here is exactly the bug that made
     * every scaled quantised load multiply by 0.0 -- in this harness *and* on
     * the console, since nothing anywhere called ppc_init_constants. */
    ppc_init_constants(s);
    s->pc  = DT_CODE_BASE;
    c->setup(s);
    s->downcount = (s32)c->steps;
    s->exit_requested = 0;
}

/* ---------------------------------------------------------------- fuzzing
 *
 * Random short integer programs, both engines, full-state compare. Exists
 * because a real miscompile shipped anyway: the THP video decoder produced
 * all-zero coefficients under the JIT and correct output under the
 * interpreter, so the curated cases above provably do not cover the decoder's
 * instruction mix. Random programs over that mix hunt the divergence
 * deterministically (fixed seed = reproducible failure). */
static u32 fz_state;
static u32 fz(void) { fz_state = fz_state * 1664525u + 1013904223u; return fz_state; }

static u32 fuzz_op(void)
{
    static const u16 xo31[] = { 24, 28, 60, 124, 284, 316, 412, 444, 476, 536,
                                792, 824, 922, 954, 26, 8, 10, 40, 138, 234,
                                202, 136, 232, 104, 235, 75, 11, 459, 491,
                                266, 40 };
    u32 rd = 3u + fz() % 9u, ra = 3u + fz() % 9u, rb = 3u + fz() % 9u;
    u32 rc = fz() & 1u;
    switch (fz() % 19u) {
    case 14: {  /* paired-single arithmetic (op4). Estimates (ps_res,
                 * ps_rsqrte) excluded: host vs soft precision differs
                 * legitimately. Rc=0 always (cr1 from FPSCR untested). */
        static const u16 pxo[] = { 21, 20, 25, 10, 11, 8, 40, 72, 264,
                                   528, 560, 592, 624, 12, 14, 15 };
        u32 fd = fz() % 8u, fa = fz() % 8u, fb2 = fz() % 8u, fc = fz() % 8u;
        u16 xo = pxo[fz() % (sizeof pxo / sizeof pxo[0])];
        if (xo == 12 || xo == 14 || xo == 15) /* ps_muls0/madds0/madds1 use A-form */
            return (4u<<26)|(fd<<21)|(fa<<16)|(fb2<<11)|(fc<<6)|((u32)xo<<1);
        if (xo == 25 || xo == 20 || xo == 21 || xo == 10 || xo == 11)
            /* A-form arith: ps_mul(25) fc in slot; ps_add/sub/sel/sum via fb */
            return (4u<<26)|(fd<<21)|(fa<<16)|(fb2<<11)|(fc<<6)|((u32)xo<<1);
        return (4u<<26)|(fd<<21)|(fa<<16)|(fb2<<11)|((u32)xo<<1);
    }
    case 15: {  /* psq_l / psq_st through r30 with small offsets, GQR0-3 */
        u32 fd = fz() % 8u, gq = fz() % 4u, w = fz() & 1u;
        u32 off = (fz() % 0x38u) & ~7u;
        u32 opc = (fz() & 1u) ? 56u : 60u;   /* psq_l : psq_st */
        return (opc<<26)|(fd<<21)|(30u<<16)|(w<<15)|(gq<<12)|off;
    }
    case 18: {  /* scalar FP loads and stores through r30, both signs of
                 * displacement.
                 *
                 * The fixed suite exercises lfs/lfd/stfs with a handful of
                 * values; the fuzzer generated none, so anything value- or
                 * displacement-dependent in the single<->double conversion had
                 * no oracle. That conversion has already been wrong once (the
                 * Inf/NaN case, §17). */
        static const u16 fop[] = { 48, 50, 52, 54 };  /* lfs lfd stfs stfd */
        u32 fd = fz() % 8u;
        /* NON-NEGATIVE, and inside the window the harness saves and restores
         * (DT_DATA_BASE .. +0x100, and r30 is DT_DATA_BASE itself). A negative
         * displacement reaches below that window: the interpreter pass then
         * leaves stores there that the restore never undoes, and the compiled
         * pass starts from different memory. That produced a "divergence" in
         * f6 that was entirely an artefact of this generator -- the harness,
         * not the recompiler. 0xC0 leaves room for an 8-byte stfd. */
        u32 d = (fz() % 0xC0u) & ~7u;
        return ((u32)fop[fz() % 4u] << 26) | (fd << 21) | (30u << 16) | d;
    }
    case 17: {  /* CR logic: crand/cror/crxor/crnand/crnor/creqv/crandc/crorc
                 * and mcrf.
                 *
                 * The fuzzer generated none of these, so the whole family was
                 * unverified -- and `cror` alone is 437,668 EXECUTED fallbacks
                 * in a racing interval, the largest single decline left. It is
                 * also the family most likely to interact badly with the
                 * guard-CR-field machinery, which is exactly why it needs the
                 * oracle before it gets an implementation. */
        static const u16 cxo[] = { 257, 449, 193, 225, 33, 289, 129, 417 };
        u32 bt = fz() % 32u, ba = fz() % 32u, bb = fz() % 32u;
        if ((fz() % 8u) == 0)                       /* mcrf now and then */
            return (19u<<26)|((fz()%8u)<<23)|((fz()%8u)<<18);
        return (19u<<26)|(bt<<21)|(ba<<16)|(bb<<11)|
               ((u32)cxo[fz() % 8u]<<1);
    }
    case 16: {  /* double/single FP: fadd/fsub/fmul/fmadd/frsp/fmr/fneg/fabs */
        /* fres (24) and frsqrte (26) added.
         *
         * These are ESTIMATES on Gekko with their own tables, and the two
         * halves of this emulator disagree about them: interp_float.c computes
         * the exact 1/x and 1/sqrt(x), while the recompiler emits the host's
         * estimate instruction -- whose table is the PPE's, not Broadway's. So
         * neither matches the machine and they do not match each other. The
         * comment in interp_float.c says the harness would surface that; the
         * harness never generated either instruction. */
        static const u16 fxo[] = { 21, 20, 25, 18, 29, 28, 24, 26 };
        u32 fd = fz() % 8u, fa = fz() % 8u, fb2 = fz() % 8u, fc = fz() % 8u;
        unsigned pick = fz() % 10u;
        if (pick < 8) {
            u16 xo = fxo[pick];
            if (xo == 18) xo = 21;           /* skip fdiv: precision */
            return (63u<<26)|(fd<<21)|(fa<<16)|(fb2<<11)|(fc<<6)|((u32)xo<<1);
        }
        if (pick == 6)  /* frsp */
            return (63u<<26)|(fd<<21)|(fb2<<11)|(12u<<1);
        return (63u<<26)|(fd<<21)|(fb2<<11)|(72u<<1);  /* fmr */
    }
    case 10:  /* dynamic shifts: slw/srw/sraw with shift amounts 0..63 in rb */
        { static const u16 sxo[] = { 24, 536, 792 };
          return (31u<<26)|(rd<<21)|(ra<<16)|(rb<<11)|
                 ((u32)sxo[fz()%3u]<<1)|rc; }
    case 11:  /* carry chain: addic(.)/subfic then adde/subfe/addme/addze */
        { switch (fz() % 4u) {
          case 0: return (12u<<26)|(rd<<21)|(ra<<16)|(fz()&0xFFFFu);   /* addic */
          case 1: return (13u<<26)|(rd<<21)|(ra<<16)|(fz()&0xFFFFu);   /* addic. */
          case 2: return (8u<<26)|(rd<<21)|(ra<<16)|(fz()&0xFFFFu);    /* subfic */
          default: { static const u16 cxo[]={138,234,200,202,232,136};
            return (31u<<26)|(rd<<21)|(ra<<16)|(rb<<11)|((u32)cxo[fz()%6u]<<1)|rc; } } }
    case 12:  /* indexed loads/stores */
        { static const u16 lxo[] = { 23, 87, 279, 343, 151, 407, 215 };
          /* rb must stay small so ra+rb lands in the data window: use r29
           * preloaded with a small offset */
          return (31u<<26)|(rd<<21)|(30u<<16)|(29u<<11)|
                 ((u32)lxo[fz()%7u]<<1); }
    case 13:  /* neg/divw/divwu, incl. overflow-prone operands */
        { static const u16 dxo[] = { 104, 491, 459, 104 };
          return (31u<<26)|(rd<<21)|(ra<<16)|(rb<<11)|
                 ((u32)dxo[fz()%4u]<<1)|rc; }
    case 0:   /* rlwinm */
        return (21u<<26)|(rd<<21)|(ra<<16)|((fz()%32u)<<11)|((fz()%32u)<<6)|((fz()%32u)<<1)|rc;
    case 1:   /* rlwimi */
        return (20u<<26)|(rd<<21)|(ra<<16)|((fz()%32u)<<11)|((fz()%32u)<<6)|((fz()%32u)<<1)|rc;
    case 2:   /* srawi */
        return (31u<<26)|(rd<<21)|(ra<<16)|((fz()%32u)<<11)|(824u<<1)|rc;
    case 3:   /* addi/addis */
        return (((fz()&1u)?14u:15u)<<26)|(rd<<21)|(ra<<16)|(fz()&0xFFFFu);
    case 4:   /* andi./oris/xoris */
        return (((28u + fz()%3u))<<26)|(ra<<21)|(rd<<16)|(fz()&0xFFFFu);
    case 5:   /* lwz/lhz/lbz/lha from the data window */
        { static const u8 op[]={32,40,34,42};
          return ((u32)op[fz()%4u]<<26)|(rd<<21)|(30u<<16)|(fz()%0xE0u); }
    case 6:   /* stw/sth/stb into the data window */
        { static const u8 op[]={36,44,38};
          return ((u32)op[fz()%3u]<<26)|(rd<<21)|(30u<<16)|(fz()%0xE0u); }
    case 7:   /* mulli */
        return (7u<<26)|(rd<<21)|(ra<<16)|(fz()&0xFFFFu);
    default:  /* 31-form arithmetic/logical from the table */
        return (31u<<26)|(rd<<21)|(ra<<16)|(rb<<11)|
               ((u32)xo31[fz() % (sizeof xo31/sizeof xo31[0])]<<1)|rc;
    }
}

int difftest_fuzz(DiffOutFn out, void *ctx, DiffResults *res,
                  u32 seed, unsigned iters, unsigned oplen)
{
    static PPCState sa, sb;
    char line[200];
    unsigned it, i;

    memset(res, 0, sizeof *res);
#if defined(__powerpc64__) || defined(__PPC64__)
    res->jit_executed = 1;
#endif
    for (it = 0; it < iters; it++) {
        u32 ops[64];
        unsigned n = oplen ? oplen : 12u;
        int bad;
        if (n > 60) n = 60;
        fz_state = seed + it * 2654435761u;
        for (i = 0; i < n; i++) ops[i] = fuzz_op();
        for (i = 0; i < n; i++) mem_write32(DT_CODE_BASE + i*4, ops[i]);
        /* terminate with a self-branch the runner's budget expires on */
        mem_write32(DT_CODE_BASE + n*4, (18u<<26) | 0u | 2u); /* b . */

        memset(&sa, 0, sizeof sa);
        sa.msr = MSR_FP; ppc_init_constants(&sa);
        sa.pc = DT_CODE_BASE;
        for (i = 0; i < 32; i++) sa.gpr[i] = fz();
        sa.gpr[30] = DT_DATA_BASE;
        sa.gpr[29] = fz() % 0xB0u;           /* small index for x-form */
        {   /* Tame float values in the low FPRs and quantized types in
             * GQR0-3, so the PS ops above have realistic inputs. */
            unsigned q;
            for (q = 0; q < 8; q++) {
                union { double d; u64 u; } dv;
                /* An adversarial spread, not just the small tidy range.
                 *
                 * The old seeding produced -125..125 in both lanes: every
                 * value comfortably inside the U8/U16/S8/S16 quantisation
                 * ranges, so a store's saturation arms were never taken and
                 * neither were the NaN paths. Quantised stores clamp, convert
                 * and truncate, and all three behave differently at the
                 * boundaries -- which is precisely the code compiled in §18.
                 * One in four values is now huge, tiny, NaN or infinite. */
                unsigned k6 = fz() % 8u;
                switch (k6) {
                case 0: dv.u = 0x7FF8000000000000ull; break;  /* NaN        */
                case 1: dv.u = 0x7FF0000000000000ull; break;  /* +Inf       */
                case 2: dv.u = 0xFFF0000000000000ull; break;  /* -Inf       */
                case 3: dv.d = (double)(fz() % 200000u) - 100000.0; break;
                case 4: dv.u = (u64)(fz() & 0xFFFFu);         break; /* denormal */
                default:
                    dv.d = ((double)(fz() % 4000u) - 2000.0) / 16.0; break;
                }
                sa.ps[q].ps0.u = dv.u;
                switch (fz() % 8u) {
                case 0: dv.u = 0x7FF8000000000000ull; break;
                case 1: dv.d = (double)(fz() % 200000u) - 100000.0; break;
                case 2: dv.u = (u64)(fz() & 0xFFFFu); break;
                default:
                    dv.d = ((double)(fz() % 4000u) - 2000.0) / 32.0; break;
                }
                sa.ps[q].ps1.u = dv.u;
            }
            sa.spr[912] = 0;                       /* GQR0 float */
            sa.spr[913] = 0x00040004u;             /* u8  (type 4) */
            sa.spr[914] = 0x00050005u;             /* u16 (type 5) */
            sa.spr[915] = 0x20062006u;             /* s8  (type 6), scaled */
            sa.msr |= MSR_FP;
            /* Paired singles ENABLED.
             *
             * Without this the GQRs above were decoration: compile_psq
             * declines every quantized access when HID2[LSQE] is clear, so the
             * JIT fell back to the interpreter and the comparison was between
             * the interpreter and itself. Forty thousand cases passed while
             * three quarters of the quantisation formats went uncompiled --
             * which is how a swapped u16/s8 row in the JIT's format table
             * survived to hang the game (see docs/PLAN.md §16). */
            sa.hid2 |= HID2_PSE | HID2_LSQE;
        }
        sa.gpr[1]  = DT_DATA_BASE + 0x800;
        /* bias some registers toward shift-interesting values */
        sa.gpr[4] = fz() % 70u;
        sa.gpr[7] = 31u + fz() % 5u;
        for (i = 0; i < 0x100; i += 4) mem_write32(DT_DATA_BASE + i, fz());
        sb = sa;
        {   /* both engines run the same budget */
            u32 datacopy[0x40];
            for (i = 0; i < 0x40; i++) datacopy[i] = mem_read32(DT_DATA_BASE + i*4);
            sa.downcount = (s32)(n + 2); sa.exit_requested = 0;
            interp_run(&sa);
            for (i = 0; i < 0x40; i++) mem_write32(DT_DATA_BASE + i*4, datacopy[i]);
            sb.downcount = (s32)(n + 2); sb.exit_requested = 0;
            jit_flush_all();
            jit_run(&sb);
        }
        ppc_fprf_sync(&sa); ppc_fprf_sync(&sb);
        bad = compare_state(&sa, &sb, out, ctx, "fuzz");
        res->cases_run++;
        if (bad) {
            res->cases_failed++;
            snprintf(line, sizeof line, "  FUZZ FAIL seed=%08x it=%u ops:",
                     seed, it);
            out(ctx, line);
            for (i = 0; i < n; i++) {
                snprintf(line, sizeof line, "    op[%u]=%08x", i, ops[i]);
                out(ctx, line);
            }
            if (res->cases_failed >= 5) break;
        }
    }
    snprintf(line, sizeof line, "%s: fuzz %u/%u matched",
             res->cases_failed ? "FAIL" : "PASS",
             res->cases_run - res->cases_failed, res->cases_run);
    out(ctx, line);
    return res->cases_failed ? -1 : 0;
}

int difftest_run_all(DiffOutFn out, void *ctx, DiffResults *res)
{
    static PPCState sa, sb;
    static u32 scratch[256];
    char line[160];
    unsigned i;

    memset(res, 0, sizeof *res);
#if defined(__powerpc64__) || defined(__PPC64__)
    res->jit_executed = 1;
#endif

    out(ctx, res->jit_executed
        ? "differential: interpreter vs RECOMPILED code (JIT executing)"
        : "differential: interpreter vs interpreter "
          "(host is not PowerPC; JIT compiles but cannot execute)");

    for (i = 0; i < sizeof k_cases / sizeof k_cases[0]; i++) {
        const DiffCase *c = &k_cases[i];
        int bad;

        /* Each engine gets a pristine data area as well as pristine registers,
         * so a store in one run cannot influence the other. */
        install(c, scratch, 256);

        init_state(&sa, c);
        interp_run(&sa);

        install(c, scratch, 256);
        init_state(&sb, c);
        jit_flush_all();            /* compile fresh: exercises the compiler */
        jit_run(&sb);

        /* FPSCR[FPRF] is maintained lazily: the interpreter writes it
         * eagerly, compiled code records the source value and classifies it
         * when the guest reads FPSCR. Folding both states here is the same
         * step a guest `mffs` performs, so the comparison below is over the
         * architecturally visible value in both engines. */
        ppc_fprf_sync(&sa);
        ppc_fprf_sync(&sb);

        bad = compare_state(&sa, &sb, out, ctx, c->name);
        res->cases_run++;
        if (bad) {
            res->cases_failed++;
            res->state_mismatches += (unsigned)bad;
            snprintf(line, sizeof line, "  FAIL %-10s (%d differences)", c->name, bad);
        } else {
            snprintf(line, sizeof line, "  ok   %-10s", c->name);
        }
        out(ctx, line);
    }

    snprintf(line, sizeof line, "%s: %u/%u cases matched",
             res->cases_failed ? "FAIL" : "PASS",
             res->cases_run - res->cases_failed, res->cases_run);
    out(ctx, line);

    return res->cases_failed ? -1 : 0;
}
