/* ppc_emitter.h — PowerPC64 instruction emitter for the recompiler backend.
 *
 * This emits *host* (Cell PPE, 64-bit) instructions. It is deliberately a thin,
 * total-coverage assembler rather than a clever one: correctness of encodings is
 * verified mechanically against llvm-objdump by tools/emitter_selftest, so the
 * JIT above it can be written without second-guessing bit layouts.
 *
 * Naming follows the PowerPC ISA exactly (e.g. `rlwinm_`, trailing underscore =
 * Rc bit set / "dot" form), so JIT source reads like the assembly it produces
 * and can be checked line-by-line against the Gekko manual.
 *
 * Field-numbering caution: PowerPC numbers bits MSB-first. Mask and shift
 * arguments (MB/ME/SH) are taken here in *ISA* numbering, and the encoders do
 * the bit gymnastics — notably MD-form, whose 6-bit mask field is stored with
 * its low 5 bits and high bit swapped.
 */
#ifndef DOLPHIN_CORE_PPC_JIT_EMITTER_H
#define DOLPHIN_CORE_PPC_JIT_EMITTER_H

#include "../../../common/types.h"

/* ------------------------------------------------------------------ */
/* Host register allocation — see ARCHITECTURE.md §4.1                  */
/* ------------------------------------------------------------------ */

/* Fixed by the 64-bit PowerPC ELF ABI. */
#define H_R0        0       /* reads as literal 0 in the `ra` slot of d-form */
#define H_SP        1
#define H_TOC       2
#define H_TLS       13

/* Pinned emulator state. Chosen from the ABI's non-volatile set (r14-r31) so
 * they survive calls into C helpers without save/restore at every call site. */
#define H_MEMBASE   14      /* guest memory arena base                     */
#define H_STATE     15      /* &PPCState                                   */
#define H_DOWNCOUNT 16      /* cycles until the scheduler must run         */
#define H_DISPATCH  17      /* block-lookup table base                     */

/* r18..r31 form the guest-GPR cache (14 registers). */
#define H_GPRCACHE_FIRST 18
#define H_GPRCACHE_LAST  31
#define H_GPRCACHE_COUNT (H_GPRCACHE_LAST - H_GPRCACHE_FIRST + 1)

/* r3..r12 are volatile: scratch, and the ABI's argument registers. */
#define H_SCRATCH0  3
#define H_SCRATCH1  4
#define H_SCRATCH2  5
#define H_SCRATCH3  6

/* Guest XER[CA], kept in a register across a block.
 *
 * Carry is the hottest piece of state outside the GPRs: a 64-bit add on a
 * 32-bit machine is an addc/adde pair, and GCC emits those constantly. Round-
 * tripping it through PPCState for every one turns a four-instruction sequence
 * into a six-instruction one with two memory accesses in the dependency chain.
 * Volatile under the ABI, which is fine -- it is flushed before any helper call
 * and reloaded after. */
#define H_CARRY     8

/* Each holds `arena_base + fold(guest_base)` for one guest register in use as
 * an address base, so a run of accesses off one pointer -- struct fields, array
 * walks -- costs one instruction each after the first instead of three.
 *
 * There are three because compiled code alternates between a stack pointer, a
 * frame pointer and a data pointer; with a single slot every switch re-ran the
 * MMIO guard and re-folded the base. Volatile, so they are discarded across
 * helper calls. */
#define H_ADDRBASE  12
#define H_ADDRBASE1 11
#define H_ADDRBASE2 10
#define JIT_ADDR_BASE_SLOTS 3
#define H_ARG0      3
#define H_ARG1      4
#define H_ARG2      5
#define H_ARG3      6
#define H_RET0      3

/* f14..f31 are non-volatile FPRs: the guest-FPR cache. f0..f13 are scratch. */
#define H_FPRCACHE_FIRST 14
#define H_FPRCACHE_LAST  31

/* v20..v31 are the non-volatile VMX registers; v0..v19 scratch. */
#define H_VRCACHE_FIRST  20

/* Condition-register field reserved for the JIT's own tests (the MMIO check),
 * so they cannot disturb the guest CR that lives in the host CR for the
 * duration of a block. Guest CR7 is kept in memory instead; compilers target
 * CR0 overwhelmingly, so the trade costs almost nothing. */
#define H_CR_JIT     7

/* ------------------------------------------------------------------ */
/* Emitter state                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    u32 *base;      /* start of the buffer                        */
    u32 *cur;       /* next word to write                         */
    u32 *limit;     /* one past the last writable word            */
    int  overflow;  /* set once the buffer is exhausted           */
} PPCEmitter;

DOL_INLINE void emit_init(PPCEmitter *e, void *buf, size_t bytes)
{
    e->base = (u32 *)buf;
    e->cur  = (u32 *)buf;
    e->limit = (u32 *)((u8 *)buf + (bytes & ~(size_t)3));
    e->overflow = 0;
}

DOL_INLINE size_t emit_size(const PPCEmitter *e)
{
    return (size_t)((u8 *)e->cur - (u8 *)e->base);
}

DOL_INLINE u32 *emit_mark(const PPCEmitter *e) { return e->cur; }

/* Every encoder funnels through here. Overflow is sticky and checked once at
 * block end rather than being tested per instruction — a block that overruns is
 * simply discarded and recompiled with a larger buffer. */
DOL_INLINE void emit_word(PPCEmitter *e, u32 insn)
{
    if (UNLIKELY(e->cur >= e->limit)) { e->overflow = 1; return; }
    *e->cur++ = insn;
}

/* ------------------------------------------------------------------ */
/* Instruction form encoders                                            */
/* ------------------------------------------------------------------ */

DOL_INLINE u32 enc_d(u32 op, u32 rt, u32 ra, s32 d)
{ return (op << 26) | (rt << 21) | (ra << 16) | ((u32)d & 0xFFFFu); }

DOL_INLINE u32 enc_ds(u32 op, u32 rt, u32 ra, s32 d, u32 xo)
{ return (op << 26) | (rt << 21) | (ra << 16) | ((u32)d & 0xFFFCu) | xo; }

DOL_INLINE u32 enc_x(u32 op, u32 rt, u32 ra, u32 rb, u32 xo, u32 rc)
{ return (op << 26) | (rt << 21) | (ra << 16) | (rb << 11) | (xo << 1) | rc; }

DOL_INLINE u32 enc_xo(u32 op, u32 rt, u32 ra, u32 rb, u32 oe, u32 xo, u32 rc)
{ return (op << 26) | (rt << 21) | (ra << 16) | (rb << 11) | (oe << 10) | (xo << 1) | rc; }

DOL_INLINE u32 enc_m(u32 op, u32 rs, u32 ra, u32 sh, u32 mb, u32 me, u32 rc)
{ return (op << 26) | (rs << 21) | (ra << 16) | (sh << 11) | (mb << 6) | (me << 1) | rc; }

/* MD-form stores the 6-bit mask with bit 5 moved to the low position. */
DOL_INLINE u32 enc_md(u32 op, u32 rs, u32 ra, u32 sh, u32 mb, u32 xo, u32 rc)
{
    u32 mb6 = ((mb & 0x1Fu) << 1) | ((mb >> 5) & 1u);
    return (op << 26) | (rs << 21) | (ra << 16) | ((sh & 0x1Fu) << 11) |
           (mb6 << 5) | (xo << 2) | (((sh >> 5) & 1u) << 1) | rc;
}

DOL_INLINE u32 enc_a(u32 op, u32 frt, u32 fra, u32 frb, u32 frc, u32 xo, u32 rc)
{ return (op << 26) | (frt << 21) | (fra << 16) | (frb << 11) | (frc << 6) | (xo << 1) | rc; }

DOL_INLINE u32 enc_xl(u32 op, u32 bt, u32 ba, u32 bb, u32 xo, u32 lk)
{ return (op << 26) | (bt << 21) | (ba << 16) | (bb << 11) | (xo << 1) | lk; }

/* XFX: the 10-bit SPR number is stored with its two 5-bit halves swapped. */
DOL_INLINE u32 enc_xfx(u32 op, u32 rt, u32 spr, u32 xo)
{
    u32 s = ((spr & 0x1Fu) << 5) | ((spr >> 5) & 0x1Fu);
    return (op << 26) | (rt << 21) | (s << 11) | (xo << 1);
}

DOL_INLINE u32 enc_va(u32 vd, u32 va, u32 vb, u32 vc, u32 xo)
{ return (4u << 26) | (vd << 21) | (va << 16) | (vb << 11) | (vc << 6) | xo; }

DOL_INLINE u32 enc_vx(u32 vd, u32 va, u32 vb, u32 xo)
{ return (4u << 26) | (vd << 21) | (va << 16) | (vb << 11) | xo; }

/* ------------------------------------------------------------------ */
/* Branch condition encoding                                            */
/* ------------------------------------------------------------------ */

#define BO_FALSE        4u    /* branch if CR[bi] == 0        */
#define BO_TRUE         12u   /* branch if CR[bi] == 1        */
#define BO_ALWAYS       20u
#define BO_DNZ          16u   /* decrement CTR, branch if != 0 */
#define BO_DZ           18u   /* decrement CTR, branch if == 0 */
#define BO_HINT         1u    /* OR into BO to invert the static prediction */

#define BI_LT(crf)      ((crf) * 4 + 0)
#define BI_GT(crf)      ((crf) * 4 + 1)
#define BI_EQ(crf)      ((crf) * 4 + 2)
#define BI_SO(crf)      ((crf) * 4 + 3)

/* ------------------------------------------------------------------ */
/* Integer — register/register                                          */
/* ------------------------------------------------------------------ */

#define X31_RR(name, xo)                                                    \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 ra, u32 rs, u32 rb)         \
    { emit_word(e, enc_x(31, rs, ra, rb, xo, 0)); }                         \
    DOL_INLINE void e_##name##_(PPCEmitter *e, u32 ra, u32 rs, u32 rb)      \
    { emit_word(e, enc_x(31, rs, ra, rb, xo, 1)); }

/* Note the PowerPC operand order for logicals: `and ra, rs, rb`. */
X31_RR(and,   28)
X31_RR(andc,  60)
X31_RR(nand, 476)
X31_RR(or,   444)
X31_RR(orc,  412)
X31_RR(nor,  124)
X31_RR(xor,  316)
X31_RR(eqv,  284)
X31_RR(slw,   24)
X31_RR(srw,  536)
X31_RR(sraw, 792)
X31_RR(sld,   27)
X31_RR(srd,  539)
X31_RR(srad, 794)
#undef X31_RR

DOL_INLINE void e_mr(PPCEmitter *e, u32 rt, u32 rs)
{ emit_word(e, enc_x(31, rs, rt, rs, 444, 0)); }          /* or rt, rs, rs  */

DOL_INLINE void e_nop(PPCEmitter *e)
{ emit_word(e, enc_d(24, 0, 0, 0)); }                     /* ori 0,0,0      */

#define X31_UN(name, xo)                                                    \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 ra, u32 rs)                 \
    { emit_word(e, enc_x(31, rs, ra, 0, xo, 0)); }                          \
    DOL_INLINE void e_##name##_(PPCEmitter *e, u32 ra, u32 rs)              \
    { emit_word(e, enc_x(31, rs, ra, 0, xo, 1)); }

X31_UN(extsb,  954)
X31_UN(extsh,  922)
X31_UN(extsw,  986)
X31_UN(cntlzw,  26)
X31_UN(cntlzd,  58)
#undef X31_UN

DOL_INLINE void e_srawi(PPCEmitter *e, u32 ra, u32 rs, u32 sh)
{ emit_word(e, enc_x(31, rs, ra, sh, 824, 0)); }
DOL_INLINE void e_srawi_(PPCEmitter *e, u32 ra, u32 rs, u32 sh)
{ emit_word(e, enc_x(31, rs, ra, sh, 824, 1)); }

/* sradi is XS-form: the 6-bit shift is split like MD-form's. */
DOL_INLINE void e_sradi(PPCEmitter *e, u32 ra, u32 rs, u32 sh)
{
    emit_word(e, (31u << 26) | (rs << 21) | (ra << 16) | ((sh & 0x1Fu) << 11) |
                 (413u << 2) | (((sh >> 5) & 1u) << 1));
}

/* ------------------------------------------------------------------ */
/* Integer — XO-form arithmetic                                         */
/* ------------------------------------------------------------------ */

#define XO_RRR(name, xo)                                                    \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 rt, u32 ra, u32 rb)         \
    { emit_word(e, enc_xo(31, rt, ra, rb, 0, xo, 0)); }                     \
    DOL_INLINE void e_##name##_(PPCEmitter *e, u32 rt, u32 ra, u32 rb)      \
    { emit_word(e, enc_xo(31, rt, ra, rb, 0, xo, 1)); }                     \
    DOL_INLINE void e_##name##o(PPCEmitter *e, u32 rt, u32 ra, u32 rb)      \
    { emit_word(e, enc_xo(31, rt, ra, rb, 1, xo, 0)); }

XO_RRR(add,    266)
XO_RRR(addc,    10)
XO_RRR(adde,   138)
XO_RRR(subf,    40)
XO_RRR(subfc,    8)
XO_RRR(subfe,  136)
XO_RRR(mullw,  235)
XO_RRR(mulhw,   75)
XO_RRR(mulhwu,  11)
XO_RRR(divw,   491)
XO_RRR(divwu,  459)
XO_RRR(mulld,  233)
XO_RRR(divd,   489)
XO_RRR(divdu,  457)
#undef XO_RRR

#define XO_RR(name, xo)                                                     \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 rt, u32 ra)                 \
    { emit_word(e, enc_xo(31, rt, ra, 0, 0, xo, 0)); }                      \
    DOL_INLINE void e_##name##_(PPCEmitter *e, u32 rt, u32 ra)              \
    { emit_word(e, enc_xo(31, rt, ra, 0, 0, xo, 1)); }

XO_RR(neg,    104)
XO_RR(addme,  234)
XO_RR(addze,  202)
XO_RR(subfme, 232)
XO_RR(subfze, 200)
#undef XO_RR

/* ------------------------------------------------------------------ */
/* Integer — immediate                                                  */
/* ------------------------------------------------------------------ */

DOL_INLINE void e_addi(PPCEmitter *e, u32 rt, u32 ra, s32 si)
{ emit_word(e, enc_d(14, rt, ra, si)); }
DOL_INLINE void e_addis(PPCEmitter *e, u32 rt, u32 ra, s32 si)
{ emit_word(e, enc_d(15, rt, ra, si)); }
DOL_INLINE void e_addic(PPCEmitter *e, u32 rt, u32 ra, s32 si)
{ emit_word(e, enc_d(12, rt, ra, si)); }
DOL_INLINE void e_addic_(PPCEmitter *e, u32 rt, u32 ra, s32 si)
{ emit_word(e, enc_d(13, rt, ra, si)); }
DOL_INLINE void e_subfic(PPCEmitter *e, u32 rt, u32 ra, s32 si)
{ emit_word(e, enc_d(8, rt, ra, si)); }
DOL_INLINE void e_mulli(PPCEmitter *e, u32 rt, u32 ra, s32 si)
{ emit_word(e, enc_d(7, rt, ra, si)); }

DOL_INLINE void e_andi_(PPCEmitter *e, u32 ra, u32 rs, u32 ui)
{ emit_word(e, enc_d(28, rs, ra, (s32)(ui & 0xFFFFu))); }
DOL_INLINE void e_andis_(PPCEmitter *e, u32 ra, u32 rs, u32 ui)
{ emit_word(e, enc_d(29, rs, ra, (s32)(ui & 0xFFFFu))); }
DOL_INLINE void e_ori(PPCEmitter *e, u32 ra, u32 rs, u32 ui)
{ emit_word(e, enc_d(24, rs, ra, (s32)(ui & 0xFFFFu))); }
DOL_INLINE void e_oris(PPCEmitter *e, u32 ra, u32 rs, u32 ui)
{ emit_word(e, enc_d(25, rs, ra, (s32)(ui & 0xFFFFu))); }
DOL_INLINE void e_xori(PPCEmitter *e, u32 ra, u32 rs, u32 ui)
{ emit_word(e, enc_d(26, rs, ra, (s32)(ui & 0xFFFFu))); }
DOL_INLINE void e_xoris(PPCEmitter *e, u32 ra, u32 rs, u32 ui)
{ emit_word(e, enc_d(27, rs, ra, (s32)(ui & 0xFFFFu))); }

DOL_INLINE void e_li(PPCEmitter *e, u32 rt, s32 v)
{ emit_word(e, enc_d(14, rt, 0, v)); }                    /* addi rt, 0, v  */
DOL_INLINE void e_lis(PPCEmitter *e, u32 rt, s32 v)
{ emit_word(e, enc_d(15, rt, 0, v)); }

/* Materialize a full 32-bit constant, zero-extended (the guest-GPR invariant).
 * Emits one instruction when the value fits 16 bits, two otherwise. */
DOL_INLINE void e_load_imm32(PPCEmitter *e, u32 rt, u32 v)
{
    if (v <= 0x7FFFu) {
        e_li(e, rt, (s32)v);
    } else if ((v & 0xFFFFu) == 0) {
        e_lis(e, rt, (s32)(s16)(v >> 16));
        if (v & 0x80000000u)                 /* lis sign-extends; re-clear */
            emit_word(e, enc_md(30, rt, rt, 0, 32, 0, 0));  /* rldicl rt,rt,0,32 */
    } else {
        e_lis(e, rt, (s32)(s16)(v >> 16));
        e_ori(e, rt, rt, v & 0xFFFFu);
        if (v & 0x80000000u)
            emit_word(e, enc_md(30, rt, rt, 0, 32, 0, 0));
    }
}

/* Materialize a 32-bit constant for a consumer that reads only the low word --
 * a 32-bit store (`stw` of a guest pc or LR value) being the important case.
 * Skips e_load_imm32's high-half cleanup, so a bit-31 constant (every guest
 * text address: 0x80xxxxxx) is two instructions instead of three. The upper
 * half holds lis's sign extension, which such a consumer never sees. */
DOL_INLINE void e_load_imm32_lo(PPCEmitter *e, u32 rt, u32 v)
{
    if ((u32)(v + 0x8000u) <= 0xFFFFu) {  /* fits addi's sign-extended s16 */
        e_li(e, rt, (s32)(s16)v);
    } else if ((v & 0xFFFFu) == 0) {
        e_lis(e, rt, (s32)(s16)(v >> 16));
    } else {
        e_lis(e, rt, (s32)(s16)(v >> 16));
        e_ori(e, rt, rt, v & 0xFFFFu);
    }
}

/* ------------------------------------------------------------------ */
/* Compare                                                              */
/* ------------------------------------------------------------------ */

/* L=0 selects a 32-bit compare, which on a 64-bit implementation compares the
 * low word *sign-extended* — exactly the guest's semantics, and therefore valid
 * regardless of what the upper 32 bits of the host register happen to hold. */
DOL_INLINE void e_cmpw(PPCEmitter *e, u32 crf, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, crf << 2, ra, rb, 0, 0)); }
DOL_INLINE void e_cmplw(PPCEmitter *e, u32 crf, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, crf << 2, ra, rb, 32, 0)); }
DOL_INLINE void e_cmpd(PPCEmitter *e, u32 crf, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, (crf << 2) | 1, ra, rb, 0, 0)); }
DOL_INLINE void e_cmpld(PPCEmitter *e, u32 crf, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, (crf << 2) | 1, ra, rb, 32, 0)); }

DOL_INLINE void e_cmpwi(PPCEmitter *e, u32 crf, u32 ra, s32 si)
{ emit_word(e, enc_d(11, crf << 2, ra, si)); }
DOL_INLINE void e_cmplwi(PPCEmitter *e, u32 crf, u32 ra, u32 ui)
{ emit_word(e, enc_d(10, crf << 2, ra, (s32)(ui & 0xFFFFu))); }

/* ------------------------------------------------------------------ */
/* Rotate and mask                                                      */
/* ------------------------------------------------------------------ */

DOL_INLINE void e_rlwinm(PPCEmitter *e, u32 ra, u32 rs, u32 sh, u32 mb, u32 me)
{ emit_word(e, enc_m(21, rs, ra, sh, mb, me, 0)); }
DOL_INLINE void e_rlwinm_(PPCEmitter *e, u32 ra, u32 rs, u32 sh, u32 mb, u32 me)
{ emit_word(e, enc_m(21, rs, ra, sh, mb, me, 1)); }
DOL_INLINE void e_rlwnm(PPCEmitter *e, u32 ra, u32 rs, u32 rb, u32 mb, u32 me)
{ emit_word(e, enc_m(23, rs, ra, rb, mb, me, 0)); }
DOL_INLINE void e_rlwimi(PPCEmitter *e, u32 ra, u32 rs, u32 sh, u32 mb, u32 me)
{ emit_word(e, enc_m(20, rs, ra, sh, mb, me, 0)); }

DOL_INLINE void e_rldicl(PPCEmitter *e, u32 ra, u32 rs, u32 sh, u32 mb)
{ emit_word(e, enc_md(30, rs, ra, sh, mb, 0, 0)); }
DOL_INLINE void e_rldicr(PPCEmitter *e, u32 ra, u32 rs, u32 sh, u32 me)
{ emit_word(e, enc_md(30, rs, ra, sh, me, 1, 0)); }
DOL_INLINE void e_rldic(PPCEmitter *e, u32 ra, u32 rs, u32 sh, u32 mb)
{ emit_word(e, enc_md(30, rs, ra, sh, mb, 2, 0)); }
DOL_INLINE void e_rldimi(PPCEmitter *e, u32 ra, u32 rs, u32 sh, u32 mb)
{ emit_word(e, enc_md(30, rs, ra, sh, mb, 3, 0)); }

/* The two workhorses of the 32-bit-in-64-bit invariant (ARCHITECTURE.md §3.3). */
DOL_INLINE void e_clrldi(PPCEmitter *e, u32 ra, u32 rs, u32 n)
{ e_rldicl(e, ra, rs, 0, n); }                 /* clear the high n bits    */
DOL_INLINE void e_zext32(PPCEmitter *e, u32 ra, u32 rs)
{ e_rldicl(e, ra, rs, 0, 32); }                /* re-establish invariant   */
/* Extract host bit 32 (the carry out of a 32-bit add) into bit 0. */
DOL_INLINE void e_extract_carry32(PPCEmitter *e, u32 ra, u32 rs)
{ e_rldicl(e, ra, rs, 32, 63); }

/* ------------------------------------------------------------------ */
/* Load / store                                                         */
/* ------------------------------------------------------------------ */

#define LS_D(name, op)                                                      \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 rt, s32 d, u32 ra)          \
    { emit_word(e, enc_d(op, rt, ra, d)); }

LS_D(lbz, 34) LS_D(lhz, 40) LS_D(lha, 42) LS_D(lwz, 32)
LS_D(stb, 38) LS_D(sth, 44) LS_D(stw, 36)
/* Update forms. The recompiler builds guest addresses through its own base
 * register and writes RA back separately, so it never emits these as *host*
 * instructions -- but the guest has them (a function prologue is `stwu`), and
 * the test harnesses assemble guest code with this same emitter. */
LS_D(lwzu, 33) LS_D(stwu, 37)
LS_D(lbzu, 35) LS_D(lhzu, 41) LS_D(lhau, 43)
LS_D(stbu, 39) LS_D(sthu, 45)
LS_D(lfsu, 49) LS_D(lfdu, 51) LS_D(stfsu, 53) LS_D(stfdu, 55)
LS_D(lfs, 48) LS_D(lfd, 50) LS_D(stfs, 52) LS_D(stfd, 54)
#undef LS_D

DOL_INLINE void e_ld(PPCEmitter *e, u32 rt, s32 d, u32 ra)
{ emit_word(e, enc_ds(58, rt, ra, d, 0)); }
DOL_INLINE void e_lwa(PPCEmitter *e, u32 rt, s32 d, u32 ra)
{ emit_word(e, enc_ds(58, rt, ra, d, 2)); }
DOL_INLINE void e_std(PPCEmitter *e, u32 rs, s32 d, u32 ra)
{ emit_word(e, enc_ds(62, rs, ra, d, 0)); }

#define LS_X(name, xo)                                                      \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 rt, u32 ra, u32 rb)         \
    { emit_word(e, enc_x(31, rt, ra, rb, xo, 0)); }

/* The fastmem workhorses: `e_lwzx(rt, H_MEMBASE, rAddr)` *is* a guest load. */
LS_X(lbzx,  87) LS_X(lhzx, 279) LS_X(lhax, 343) LS_X(lwzx,  23)
LS_X(stbx, 215) LS_X(sthx, 407) LS_X(stwx, 151)
LS_X(ldx,   21) LS_X(stdx, 149) LS_X(lwax, 341)
LS_X(lfsx, 535) LS_X(lfdx, 599) LS_X(stfsx, 663) LS_X(stfdx, 727)
/* Byte-reversed forms. Present for completeness and for the guest's own
 * lwbrx/stwbrx opcodes — never needed for ordinary guest memory access, which
 * is the entire point of this port. */
LS_X(lwbrx, 534) LS_X(stwbrx, 662) LS_X(lhbrx, 790) LS_X(sthbrx, 918)
LS_X(lwarx,  20)
#undef LS_X

DOL_INLINE void e_stwcx_(PPCEmitter *e, u32 rs, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, rs, ra, rb, 150, 1)); }

DOL_INLINE void e_dcbz(PPCEmitter *e, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, 0, ra, rb, 1014, 0)); }
DOL_INLINE void e_dcbt(PPCEmitter *e, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, 0, ra, rb, 278, 0)); }
DOL_INLINE void e_dcbf(PPCEmitter *e, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, 0, ra, rb, 86, 0)); }
DOL_INLINE void e_dcbst(PPCEmitter *e, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, 0, ra, rb, 54, 0)); }
DOL_INLINE void e_icbi(PPCEmitter *e, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, 0, ra, rb, 982, 0)); }
DOL_INLINE void e_sync(PPCEmitter *e)  { emit_word(e, enc_x(31, 0, 0, 0, 598, 0)); }
DOL_INLINE void e_isync(PPCEmitter *e) { emit_word(e, enc_xl(19, 0, 0, 0, 150, 0)); }


/* ------------------------------------------------------------------ */
/* Floating point                                                       */
/* ------------------------------------------------------------------ */

/* Opcode 63 = double precision, 59 = single precision. The single-precision
 * forms are what paired-single arithmetic compiles to: `fadds` rounds to single
 * and stores a double, which is bit-for-bit Gekko's ps_add lane semantic. */
#define FP_AB(name, op, xo)                                                 \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 frt, u32 fra, u32 frb)      \
    { emit_word(e, enc_a(op, frt, fra, frb, 0, xo, 0)); }

FP_AB(fadd,  63, 21) FP_AB(fsub,  63, 20) FP_AB(fdiv,  63, 18)
FP_AB(fadds, 59, 21) FP_AB(fsubs, 59, 20) FP_AB(fdivs, 59, 18)
#undef FP_AB

/* Multiply takes its second operand in the frc slot. */
#define FP_AC(name, op, xo)                                                 \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 frt, u32 fra, u32 frc)      \
    { emit_word(e, enc_a(op, frt, fra, 0, frc, xo, 0)); }

FP_AC(fmul,  63, 25)
FP_AC(fmuls, 59, 25)
#undef FP_AC


#define FP_ABC(name, op, xo)                                                \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 frt, u32 fra, u32 frc, u32 frb) \
    { emit_word(e, enc_a(op, frt, fra, frb, frc, xo, 0)); }

FP_ABC(fmadd,   63, 29) FP_ABC(fmsub,   63, 28)
FP_ABC(fnmadd,  63, 31) FP_ABC(fnmsub,  63, 30)
FP_ABC(fmadds,  59, 29) FP_ABC(fmsubs,  59, 28)
FP_ABC(fnmadds, 59, 31) FP_ABC(fnmsubs, 59, 30)
FP_ABC(fsel,    63, 23)
#undef FP_ABC

DOL_INLINE void e_fres(PPCEmitter *e, u32 frt, u32 frb)
{ emit_word(e, enc_a(59, frt, 0, frb, 0, 24, 0)); }
DOL_INLINE void e_frsqrte(PPCEmitter *e, u32 frt, u32 frb)
{ emit_word(e, enc_a(63, frt, 0, frb, 0, 26, 0)); }
DOL_INLINE void e_fsqrt(PPCEmitter *e, u32 frt, u32 frb)
{ emit_word(e, enc_a(63, frt, 0, frb, 0, 22, 0)); }

#define FP_UN(name, xo)                                                     \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 frt, u32 frb)               \
    { emit_word(e, enc_x(63, frt, 0, frb, xo, 0)); }

FP_UN(fmr,    72) FP_UN(fneg,   40) FP_UN(fabs,  264) FP_UN(fnabs, 136)
FP_UN(frsp,   12) FP_UN(fctiw,  14) FP_UN(fctiwz, 15)
FP_UN(fctid, 814) FP_UN(fctidz,815) FP_UN(fcfid, 846)
#undef FP_UN

DOL_INLINE void e_fcmpu(PPCEmitter *e, u32 crf, u32 fra, u32 frb)
{ emit_word(e, enc_x(63, crf << 2, fra, frb, 0, 0)); }
DOL_INLINE void e_fcmpo(PPCEmitter *e, u32 crf, u32 fra, u32 frb)
{ emit_word(e, enc_x(63, crf << 2, fra, frb, 32, 0)); }

DOL_INLINE void e_mffs(PPCEmitter *e, u32 frt)
{ emit_word(e, enc_x(63, frt, 0, 0, 583, 0)); }
DOL_INLINE void e_mtfsf(PPCEmitter *e, u32 flm, u32 frb)
{ emit_word(e, (63u << 26) | (flm << 17) | (frb << 11) | (711u << 1)); }

/* ------------------------------------------------------------------ */
/* Branches                                                             */
/* ------------------------------------------------------------------ */

/* Relative branch by a byte displacement. The caller is responsible for range
 * (+-32 MiB for b, +-32 KiB for bc); the JIT's block allocator keeps related
 * code within range, and out-of-range targets go through CTR. */
DOL_INLINE void e_b(PPCEmitter *e, s32 byte_disp)
{ emit_word(e, (18u << 26) | ((u32)byte_disp & 0x03FFFFFCu)); }
DOL_INLINE void e_bl(PPCEmitter *e, s32 byte_disp)
{ emit_word(e, (18u << 26) | ((u32)byte_disp & 0x03FFFFFCu) | 1u); }

DOL_INLINE void e_bc(PPCEmitter *e, u32 bo, u32 bi, s32 byte_disp)
{ emit_word(e, (16u << 26) | (bo << 21) | (bi << 16) | ((u32)byte_disp & 0xFFFCu)); }

DOL_INLINE void e_bclr(PPCEmitter *e, u32 bo, u32 bi)
{ emit_word(e, enc_xl(19, bo, bi, 0, 16, 0)); }
DOL_INLINE void e_bcctr(PPCEmitter *e, u32 bo, u32 bi)
{ emit_word(e, enc_xl(19, bo, bi, 0, 528, 0)); }

DOL_INLINE void e_blr(PPCEmitter *e)   { e_bclr(e, BO_ALWAYS, 0); }
DOL_INLINE void e_bctr(PPCEmitter *e)  { e_bcctr(e, BO_ALWAYS, 0); }
DOL_INLINE void e_bctrl(PPCEmitter *e) { emit_word(e, enc_xl(19, BO_ALWAYS, 0, 0, 528, 1)); }

/* ------------------------------------------------------------------ */
/* Forward-reference patching                                           */
/*                                                                      */
/* The JIT emits forward branches before their targets are known. A "fixup" is  */
/* the address of the branch word; resolving it computes and patches the        */
/* displacement in place.                                                       */
/* ------------------------------------------------------------------ */

typedef u32 *PPCFixup;

DOL_INLINE PPCFixup e_bc_fwd(PPCEmitter *e, u32 bo, u32 bi)
{
    PPCFixup f = e->cur;
    e_bc(e, bo, bi, 0);
    return f;
}

DOL_INLINE PPCFixup e_b_fwd(PPCEmitter *e)
{
    PPCFixup f = e->cur;
    e_b(e, 0);
    return f;
}

/* Patch a previously-emitted branch to land at the current emit position. */
DOL_INLINE void e_patch_here(PPCEmitter *e, PPCFixup f)
{
    if (UNLIKELY(!f || e->overflow)) return;
    {
        s32 disp = (s32)((u8 *)e->cur - (u8 *)f);
        u32 insn = *f;
        u32 op   = insn >> 26;
        if (op == 16u)                      /* B-form: 14-bit displacement */
            *f = (insn & ~0xFFFCu) | ((u32)disp & 0xFFFCu);
        else                                /* I-form: 24-bit displacement */
            *f = (insn & ~0x03FFFFFCu) | ((u32)disp & 0x03FFFFFCu);
    }
}

/* ------------------------------------------------------------------ */
/* CR and SPR movement                                                  */
/* ------------------------------------------------------------------ */

DOL_INLINE void e_mfcr(PPCEmitter *e, u32 rt)
{ emit_word(e, enc_x(31, rt, 0, 0, 19, 0)); }
DOL_INLINE void e_mtcrf(PPCEmitter *e, u32 mask, u32 rs)
{ emit_word(e, (31u << 26) | (rs << 21) | (mask << 12) | (144u << 1)); }
/* mtocrf: mtcrf with bit 11 set, writing exactly ONE CR field.
 *
 * The distinction is a performance cliff, not a style choice. Per the PPE
 * manual (Table 10-1) `mtcrf` with bit 11 clear is MICROCODED: >=11 cycles of
 * ROM latency, executed atomically, and it stalls BOTH SMT threads at
 * dispatch. `mtocrf` is 1 cycle. Restoring all eight fields as eight mtocrf
 * is therefore ~8 cycles and thread-friendly, against ~11+ cycles of
 * microcode that also stops the other thread dead. */
DOL_INLINE void e_mtocrf(PPCEmitter *e, u32 field, u32 rs)
{ emit_word(e, (31u << 26) | (rs << 21) | (1u << 20) |
               ((1u << (7u - (field & 7u))) << 12) | (144u << 1)); }
DOL_INLINE void e_mcrf(PPCEmitter *e, u32 crd, u32 crs)
{ emit_word(e, enc_xl(19, crd << 2, crs << 2, 0, 0, 0)); }

#define CR_LOGIC(name, xo)                                                  \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 bt, u32 ba, u32 bb)         \
    { emit_word(e, enc_xl(19, bt, ba, bb, xo, 0)); }

CR_LOGIC(crand,  257) CR_LOGIC(cror,   449) CR_LOGIC(crxor,  193)
CR_LOGIC(crnand, 225) CR_LOGIC(crnor,   33) CR_LOGIC(creqv, 289)
CR_LOGIC(crandc, 129) CR_LOGIC(crorc,  417)
#undef CR_LOGIC

DOL_INLINE void e_mfspr(PPCEmitter *e, u32 rt, u32 spr)
{ emit_word(e, enc_xfx(31, rt, spr, 339)); }
DOL_INLINE void e_mtspr(PPCEmitter *e, u32 spr, u32 rs)
{ emit_word(e, enc_xfx(31, rs, spr, 467)); }

DOL_INLINE void e_mflr(PPCEmitter *e, u32 rt)  { e_mfspr(e, rt, 8); }
DOL_INLINE void e_mtlr(PPCEmitter *e, u32 rs)  { e_mtspr(e, 8, rs); }
DOL_INLINE void e_mfctr(PPCEmitter *e, u32 rt) { e_mfspr(e, rt, 9); }
DOL_INLINE void e_mtctr(PPCEmitter *e, u32 rs) { e_mtspr(e, 9, rs); }

/* ------------------------------------------------------------------ */
/* VMX / AltiVec                                                        */
/*                                                                      */
/* Used narrowly and deliberately: quantized load/store (psq_l / psq_st), where  */
/* vcfux/vctsxs perform an integer<->float conversion *with a power-of-two       */
/* scale* in one instruction — precisely the GQR semantic. VMX is NOT used for   */
/* paired-single arithmetic; it lacks IEEE compliance and doubles, so the scalar */
/* FPU is both faster and more correct there (ARCHITECTURE.md §2.4).             */
/* ------------------------------------------------------------------ */

DOL_INLINE void e_lvx(PPCEmitter *e, u32 vd, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, vd, ra, rb, 103, 0)); }
DOL_INLINE void e_stvx(PPCEmitter *e, u32 vs, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, vs, ra, rb, 231, 0)); }
DOL_INLINE void e_lvsl(PPCEmitter *e, u32 vd, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, vd, ra, rb, 6, 0)); }
DOL_INLINE void e_lvsr(PPCEmitter *e, u32 vd, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, vd, ra, rb, 38, 0)); }
DOL_INLINE void e_lvewx(PPCEmitter *e, u32 vd, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, vd, ra, rb, 71, 0)); }
DOL_INLINE void e_stvewx(PPCEmitter *e, u32 vs, u32 ra, u32 rb)
{ emit_word(e, enc_x(31, vs, ra, rb, 199, 0)); }

#define VX_AB(name, xo)                                                     \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 vd, u32 va, u32 vb)         \
    { emit_word(e, enc_vx(vd, va, vb, xo)); }

VX_AB(vaddfp,   10) VX_AB(vsubfp,   74)
VX_AB(vand,   1028) VX_AB(vandc,  1092) VX_AB(vor,    1156)
VX_AB(vxor,   1220) VX_AB(vnor,   1284)
VX_AB(vmrghw,  140) VX_AB(vmrglw,  396)
VX_AB(vmaxfp, 1034) VX_AB(vminfp, 1098)
VX_AB(vpkuhum,  14) VX_AB(vpkuwum,  78)
VX_AB(vpkshss, 398) VX_AB(vpkswss, 462) VX_AB(vpkuhus, 142) VX_AB(vpkuwus, 206)
#undef VX_AB

/* The quantization primitives. `uimm` is the power-of-two scale exponent, which
 * maps directly onto the GQR scale field. */
DOL_INLINE void e_vcfux(PPCEmitter *e, u32 vd, u32 vb, u32 uimm)
{ emit_word(e, enc_vx(vd, uimm, vb, 778)); }
DOL_INLINE void e_vcfsx(PPCEmitter *e, u32 vd, u32 vb, u32 uimm)
{ emit_word(e, enc_vx(vd, uimm, vb, 842)); }
DOL_INLINE void e_vctuxs(PPCEmitter *e, u32 vd, u32 vb, u32 uimm)
{ emit_word(e, enc_vx(vd, uimm, vb, 906)); }
DOL_INLINE void e_vctsxs(PPCEmitter *e, u32 vd, u32 vb, u32 uimm)
{ emit_word(e, enc_vx(vd, uimm, vb, 970)); }

DOL_INLINE void e_vspltisw(PPCEmitter *e, u32 vd, s32 simm)
{ emit_word(e, enc_vx(vd, (u32)simm & 0x1Fu, 0, 908)); }
DOL_INLINE void e_vspltw(PPCEmitter *e, u32 vd, u32 vb, u32 uimm)
{ emit_word(e, enc_vx(vd, uimm, vb, 652)); }

#define VX_UN(name, xo)                                                     \
    DOL_INLINE void e_##name(PPCEmitter *e, u32 vd, u32 vb)                 \
    { emit_word(e, enc_vx(vd, 0, vb, xo)); }
VX_UN(vupkhsb, 526) VX_UN(vupklsb, 654) VX_UN(vupkhsh, 590) VX_UN(vupklsh, 718)
VX_UN(vrefp,   266) VX_UN(vrsqrtefp, 330)
#undef VX_UN

DOL_INLINE void e_vperm(PPCEmitter *e, u32 vd, u32 va, u32 vb, u32 vc)
{ emit_word(e, enc_va(vd, va, vb, vc, 43)); }
DOL_INLINE void e_vsel(PPCEmitter *e, u32 vd, u32 va, u32 vb, u32 vc)
{ emit_word(e, enc_va(vd, va, vb, vc, 42)); }
/* SHB is a 4-bit field sitting in the low bits of the VA-form `vc` slot, so it
 * is passed through unshifted. (Verified against llvm-mc; shifting it here is
 * an easy and silent mistake.) */
DOL_INLINE void e_vsldoi(PPCEmitter *e, u32 vd, u32 va, u32 vb, u32 shb)
{ emit_word(e, enc_va(vd, va, vb, shb & 0xFu, 44)); }
DOL_INLINE void e_vmaddfp(PPCEmitter *e, u32 vd, u32 va, u32 vc, u32 vb)
{ emit_word(e, enc_va(vd, va, vb, vc, 46)); }
DOL_INLINE void e_vnmsubfp(PPCEmitter *e, u32 vd, u32 va, u32 vc, u32 vb)
{ emit_word(e, enc_va(vd, va, vb, vc, 47)); }

/* ------------------------------------------------------------------ */
/* Instruction-cache maintenance                                        */
/*                                                                      */
/* The PPE has no coherent instruction cache: freshly emitted code is invisible  */
/* to the fetch unit until the affected lines are flushed from the data cache    */
/* and invalidated in the instruction cache. Omitting this is the classic JIT    */
/* bug that manifests as executing stale or garbage code, so it lives here       */
/* rather than at call sites.                                                    */
/* ------------------------------------------------------------------ */

DOL_INLINE void ppc_flush_icache(const void *addr, size_t len)
{
#if defined(__powerpc64__) || defined(__PPC64__) || defined(__powerpc__)
    const size_t line = 128;                 /* PPE L1 line size */
    u8 *p   = (u8 *)((size_t)addr & ~(line - 1));
    u8 *end = (u8 *)addr + len;
    for (; p < end; p += line)
        __asm__ __volatile__ ("dcbst 0,%0" :: "r"(p) : "memory");
    __asm__ __volatile__ ("sync" ::: "memory");
    p = (u8 *)((size_t)addr & ~(line - 1));
    for (; p < end; p += line)
        __asm__ __volatile__ ("icbi 0,%0" :: "r"(p) : "memory");
    __asm__ __volatile__ ("sync; isync" ::: "memory");
#else
    /* Workstation build: emitted code is inspected and disassembled but never
     * executed (the host is not a PowerPC), so there is no cache to reconcile. */
    (void)addr; (void)len;
#endif
}

#endif /* DOLPHIN_CORE_PPC_JIT_EMITTER_H */
