/* ppe_model.h — a machine model of the Cell PPE, over the words the JIT emits.
 *
 * Two consumers, one source of truth:
 *
 *   1. ppe_stall.h, the static stall analyser. It walks a compiled block's
 *      host words and reports where an in-order PPE would lose cycles, by
 *      cause. That is the instrument this whole exercise needed: the PPE has
 *      performance counters, but we cannot run on the console from here, and
 *      qemu wall-clock is worthless for the question -- qemu is an
 *      interpreter with no pipeline, so it charges the same time to a
 *      perfectly scheduled block and a maximally stalled one.
 *
 *   2. ppe_sched.h, the post-pass list scheduler that reorders those words.
 *      It needs exactly the same dependence and latency facts, so they live
 *      here rather than being written twice and drifting apart.
 *
 * What the model is, precisely:
 *
 *   - A *decoder* for the host PowerPC64 subset ppc_emitter.h can produce:
 *     which registers each word reads and writes (GPR, FPR, VR, CR field,
 *     LR/CTR/XER), which execution unit it uses, and its result latency.
 *     Anything it does not recognise decodes as PI_UNKNOWN, which every
 *     consumer treats as a full barrier that reads and writes everything.
 *     Being wrong in that direction costs modelled cycles and forbidden
 *     motion, never correctness.
 *
 *   - An *issue model*: in-order, two instructions per cycle, a pair only
 *     from different units, no dependent pair, and nothing pairs with a
 *     microcoded or serialising instruction.
 *
 * Every latency below is from docs/HARDWARE.md §1 (which in turn cites the
 * CBE handbook), so the model and the documented machine cannot drift:
 *
 *     integer ALU            2 cycles
 *     integer multiply      11
 *     L1 load                5   (+1 for a load that targets an FPR)
 *     FP (double)           10
 *     branch mispredict     24
 *
 * The numbers the model produces are *relative*. They are not a prediction of
 * console wall-clock -- no cache miss model, no SMT partner, no fetch-group
 * effects -- and they are not used as one. They are used to rank causes, and
 * to compare the same corpus of blocks before and after an emitter change,
 * which is a comparison the missing effects do not move.
 */
#ifndef DOLPHIN_CORE_PPC_JIT_PPE_MODEL_H
#define DOLPHIN_CORE_PPC_JIT_PPE_MODEL_H

#include "../../../common/types.h"

/* Plain static inline: these are large enough that forcing them inline at
 * every call site (DOL_INLINE) would bloat the compiler for no gain. */
#define PPE_INLINE static inline

/* ------------------------------------------------------------------ */
/* Execution units                                                      */
/* ------------------------------------------------------------------ */
enum {
    PU_FXU = 0,     /* fixed-point arithmetic, logic, shifts, compares  */
    PU_LSU,         /* loads and stores                                 */
    PU_FPU,         /* scalar floating point                            */
    PU_VSU,         /* VMX                                              */
    PU_BRU,         /* branches                                         */
    PU_CRU,         /* CR logical ops, mcrf                             */
    PU_MCU,         /* microcoded / serialising (mfcr, multi-field mtcrf) */
    PU_COUNT
};

/* ------------------------------------------------------------------ */
/* Instruction properties                                               */
/* ------------------------------------------------------------------ */
#define PI_LOAD     0x0001u
#define PI_STORE    0x0002u
#define PI_BRANCH   0x0004u
#define PI_BARRIER  0x0008u /* sync/isync/icbi/lwarx/stwcx.: pin in place  */
#define PI_SERIAL   0x0010u /* microcoded: cannot pair, drains the pipe    */
#define PI_UNKNOWN  0x0020u /* decoder did not recognise it                */
#define PI_LINK     0x0040u /* sets LR (lk=1)                              */
#define PI_CALL     0x0080u /* bl / bctrl: clobbers the volatile registers */
#define PI_CTRDEC   0x0100u /* branch form that decrements CTR             */
#define PI_MULDIV   0x0200u /* long-latency fixed point                    */
#define PI_CMP      0x0400u /* writes a CR field as its whole job          */

/* SPR bits, for def_spr / use_spr. */
#define PS_LR       0x01u
#define PS_CTR      0x02u
#define PS_XER      0x04u
#define PS_FPSCR    0x08u

typedef struct {
    u8  unit;
    u8  lat;            /* cycles before a dependent instruction may issue */
    u16 flags;
    u32 def_gpr, use_gpr;
    u32 def_fpr, use_fpr;
    u32 def_vr,  use_vr;
    u8  def_crf, use_crf;   /* bitmask over CR fields 0..7 */
    u8  def_spr, use_spr;
    s32 mem_disp;           /* d/ds-form displacement (loads/stores)       */
    u8  mem_base;           /* base GPR of a load/store, 32 if literal 0   */
    u8  mem_class;          /* PM_*: which object the access can touch     */
} PpeInsn;

/* Memory disambiguation, by base register.
 *
 * The recompiler's own calling convention makes this exact rather than
 * heuristic. Guest memory is only ever reached through the pinned arena base
 * (r14) or one of the three folded address-base registers (r10-r12), which
 * hold `arena + fold(guest_base)`; PPCState -- the register cache's home, the
 * downcount, the pc, the GQRs, the FP constants -- is only ever reached
 * through r15. They are different objects in different mappings and cannot
 * alias, so a register-cache reload may cross a guest store and vice versa.
 * That one fact is what lets the scheduler lift a cache load out of the middle
 * of a copy loop instead of leaving it pinned behind the store above it. Any
 * other base -- a scratch holding a lookup-table address, say -- is PM_UNKNOWN
 * and conflicts with everything, which is the safe answer. */
#define PM_UNKNOWN  0
#define PM_STATE    1
#define PM_GUEST    2

PPE_INLINE u8 ppe_mem_class(u32 base)
{
    if (base == 15) return PM_STATE;                        /* H_STATE     */
    if (base == 14 || base == 12 || base == 11 || base == 10)
        return PM_GUEST;                                    /* arena bases */
    return PM_UNKNOWN;
}

/* Latencies. Named so the source reads as the table in docs/HARDWARE.md. */
#define PPE_LAT_ALU     2
#define PPE_LAT_MUL     11
#define PPE_LAT_DIV     40
#define PPE_LAT_LOAD    5
#define PPE_LAT_LOADF   6       /* a load whose target is an FPR */
#define PPE_LAT_FPU     10
#define PPE_LAT_VSU     4
#define PPE_LAT_VSUF    12
#define PPE_LAT_CR      2       /* compare / Rc form -> CR field */
#define PPE_LAT_SPR     6       /* mtctr -> bctr, mtlr -> blr    */
#define PPE_LAT_SERIAL  5       /* mfcr, multi-field mtcrf       */
#define PPE_MISPREDICT  24

#define PPE_GPRBIT(r)  (1u << ((r) & 31u))
#define PPE_CRFBIT(f)  ((u8)(1u << ((f) & 7u)))

/* ------------------------------------------------------------------ */
/* Field accessors (host encoding; deliberately local so this header is  */
/* independent of the guest-side macros in gekko.h)                      */
/* ------------------------------------------------------------------ */
#define PPE_OPCD(w)  (((w) >> 26) & 0x3Fu)
#define PPE_RT(w)    (((w) >> 21) & 0x1Fu)
#define PPE_RA(w)    (((w) >> 16) & 0x1Fu)
#define PPE_RB(w)    (((w) >> 11) & 0x1Fu)
#define PPE_RC(w)    (((w) >>  6) & 0x1Fu)
#define PPE_XO10(w)  (((w) >>  1) & 0x3FFu)
#define PPE_XO9(w)   (((w) >>  1) & 0x1FFu)
#define PPE_XO5(w)   (((w) >>  1) & 0x1Fu)
#define PPE_RCBIT(w) ((w) & 1u)
#define PPE_SIMM(w)  ((s32)(s16)((w) & 0xFFFFu))

DOL_INLINE void ppe_clear(PpeInsn *o)
{
    o->unit = PU_FXU; o->lat = PPE_LAT_ALU; o->flags = 0;
    o->def_gpr = o->use_gpr = 0;
    o->def_fpr = o->use_fpr = 0;
    o->def_vr  = o->use_vr  = 0;
    o->def_crf = o->use_crf = 0;
    o->def_spr = o->use_spr = 0;
    o->mem_disp = 0; o->mem_base = 32; o->mem_class = PM_UNKNOWN;
}

/* Everything reads and writes everything: the safe answer for a word the
 * decoder does not know, and the one that makes an unrecognised encoding show
 * up as a cost rather than as silent miscompilation. */
PPE_INLINE void ppe_unknown(PpeInsn *o)
{
    ppe_clear(o);
    o->flags = PI_UNKNOWN | PI_BARRIER | PI_SERIAL;
    o->unit  = PU_MCU;
    o->lat   = PPE_LAT_SERIAL;
    o->def_gpr = o->use_gpr = 0xFFFFFFFFu;
    o->def_fpr = o->use_fpr = 0xFFFFFFFFu;
    o->def_vr  = o->use_vr  = 0xFFFFFFFFu;
    o->def_crf = o->use_crf = 0xFFu;
    o->def_spr = o->use_spr = 0x0Fu;
}

/* Common shapes. */
DOL_INLINE void ppe_ld(PpeInsn *o, u32 w, int to_fpr, int indexed)
{
    ppe_clear(o);
    o->unit  = PU_LSU;
    o->flags = PI_LOAD;
    o->lat   = to_fpr ? PPE_LAT_LOADF : PPE_LAT_LOAD;
    if (to_fpr) o->def_fpr = PPE_GPRBIT(PPE_RT(w));
    else        o->def_gpr = PPE_GPRBIT(PPE_RT(w));
    if (PPE_RA(w)) { o->use_gpr |= PPE_GPRBIT(PPE_RA(w)); o->mem_base = (u8)PPE_RA(w); }
    if (indexed)   o->use_gpr |= PPE_GPRBIT(PPE_RB(w));
    else           o->mem_disp = PPE_SIMM(w);
    o->mem_class = ppe_mem_class(o->mem_base);
}

DOL_INLINE void ppe_st(PpeInsn *o, u32 w, int from_fpr, int indexed)
{
    ppe_clear(o);
    o->unit  = PU_LSU;
    o->flags = PI_STORE;
    o->lat   = 0;
    if (from_fpr) o->use_fpr = PPE_GPRBIT(PPE_RT(w));
    else          o->use_gpr = PPE_GPRBIT(PPE_RT(w));
    if (PPE_RA(w)) { o->use_gpr |= PPE_GPRBIT(PPE_RA(w)); o->mem_base = (u8)PPE_RA(w); }
    if (indexed)   o->use_gpr |= PPE_GPRBIT(PPE_RB(w));
    else           o->mem_disp = PPE_SIMM(w);
    o->mem_class = ppe_mem_class(o->mem_base);
}

/* ra <- f(rs, rb): the PowerPC logical/shift operand order. */
DOL_INLINE void ppe_xrr(PpeInsn *o, u32 w, int uses_rb)
{
    ppe_clear(o);
    o->def_gpr = PPE_GPRBIT(PPE_RA(w));
    o->use_gpr = PPE_GPRBIT(PPE_RT(w));
    if (uses_rb) o->use_gpr |= PPE_GPRBIT(PPE_RB(w));
    if (PPE_RCBIT(w)) { o->def_crf = PPE_CRFBIT(0); }
}

/* rt <- f(ra, rb): the arithmetic operand order. */
DOL_INLINE void ppe_xorr(PpeInsn *o, u32 w, int uses_rb)
{
    ppe_clear(o);
    o->def_gpr = PPE_GPRBIT(PPE_RT(w));
    o->use_gpr = PPE_GPRBIT(PPE_RA(w));
    if (uses_rb) o->use_gpr |= PPE_GPRBIT(PPE_RB(w));
    if (PPE_RCBIT(w)) o->def_crf = PPE_CRFBIT(0);
    if (w & (1u << 10)) o->def_spr |= PS_XER;      /* OE */
}

/* ------------------------------------------------------------------ */
/* The decoder                                                          */
/*                                                                      */
/* Returns 1 when the word was recognised, 0 when it decoded as unknown. */
/* ------------------------------------------------------------------ */
PPE_INLINE int ppe_decode(u32 w, PpeInsn *o)
{
    u32 op = PPE_OPCD(w);

    switch (op) {

    /* ---- d-form arithmetic / logic ------------------------------- */
    case 14: case 15:                       /* addi / addis (li/lis)   */
        ppe_clear(o);
        o->def_gpr = PPE_GPRBIT(PPE_RT(w));
        if (PPE_RA(w)) o->use_gpr = PPE_GPRBIT(PPE_RA(w));
        return 1;
    case 7:                                 /* mulli                   */
        ppe_clear(o);
        o->def_gpr = PPE_GPRBIT(PPE_RT(w));
        o->use_gpr = PPE_GPRBIT(PPE_RA(w));
        o->lat = PPE_LAT_MUL; o->flags = PI_MULDIV;
        return 1;
    case 8:                                 /* subfic                  */
        ppe_clear(o);
        o->def_gpr = PPE_GPRBIT(PPE_RT(w));
        o->use_gpr = PPE_GPRBIT(PPE_RA(w));
        o->def_spr = PS_XER;
        return 1;
    case 12: case 13:                       /* addic / addic.          */
        ppe_clear(o);
        o->def_gpr = PPE_GPRBIT(PPE_RT(w));
        o->use_gpr = PPE_GPRBIT(PPE_RA(w));
        o->def_spr = PS_XER;
        if (op == 13) o->def_crf = PPE_CRFBIT(0);
        return 1;
    case 10: case 11:                       /* cmpli / cmpi            */
        ppe_clear(o);
        o->use_gpr = PPE_GPRBIT(PPE_RA(w));
        o->def_crf = PPE_CRFBIT(PPE_RT(w) >> 2);
        o->lat = PPE_LAT_CR; o->flags = PI_CMP;
        return 1;
    case 24: case 25: case 26: case 27:     /* ori/oris/xori/xoris     */
        ppe_clear(o);
        o->def_gpr = PPE_GPRBIT(PPE_RA(w));
        o->use_gpr = PPE_GPRBIT(PPE_RT(w));
        return 1;
    case 28: case 29:                       /* andi. / andis.          */
        ppe_clear(o);
        o->def_gpr = PPE_GPRBIT(PPE_RA(w));
        o->use_gpr = PPE_GPRBIT(PPE_RT(w));
        o->def_crf = PPE_CRFBIT(0);
        return 1;

    /* ---- rotates ------------------------------------------------- */
    case 21:                                /* rlwinm                  */
        ppe_xrr(o, w, 0);
        return 1;
    case 23:                                /* rlwnm                   */
        ppe_xrr(o, w, 1);
        return 1;
    case 20:                                /* rlwimi: reads RA too    */
        ppe_xrr(o, w, 0);
        o->use_gpr |= PPE_GPRBIT(PPE_RA(w));
        return 1;
    case 30: {                              /* MD/MDS: rldicl/r/ic/imi */
        u32 md = (w >> 2) & 7u;
        ppe_xrr(o, w, 0);
        if (md == 3) o->use_gpr |= PPE_GPRBIT(PPE_RA(w));   /* rldimi  */
        if (md >= 8) o->use_gpr |= PPE_GPRBIT(PPE_RB(w));   /* MDS     */
        return 1;
    }

    /* ---- d-form loads and stores --------------------------------- */
    case 32: case 34: case 40: case 42:     /* lwz lbz lhz lha         */
        ppe_ld(o, w, 0, 0); return 1;
    case 33: case 35: case 41: case 43:     /* update forms            */
        ppe_ld(o, w, 0, 0); o->def_gpr |= PPE_GPRBIT(PPE_RA(w)); return 1;
    case 36: case 38: case 44:              /* stw stb sth             */
        ppe_st(o, w, 0, 0); return 1;
    case 37: case 39: case 45:              /* update forms            */
        ppe_st(o, w, 0, 0); o->def_gpr |= PPE_GPRBIT(PPE_RA(w)); return 1;
    case 48: case 50:                       /* lfs lfd                 */
        ppe_ld(o, w, 1, 0); return 1;
    case 49: case 51:
        ppe_ld(o, w, 1, 0); o->def_gpr |= PPE_GPRBIT(PPE_RA(w)); return 1;
    case 52: case 54:                       /* stfs stfd               */
        ppe_st(o, w, 1, 0); return 1;
    case 53: case 55:
        ppe_st(o, w, 1, 0); o->def_gpr |= PPE_GPRBIT(PPE_RA(w)); return 1;
    case 58:                                /* ld / lwa (DS-form)      */
        ppe_ld(o, w, 0, 0); o->mem_disp &= ~3; return 1;
    case 62:                                /* std (DS-form)           */
        ppe_st(o, w, 0, 0); o->mem_disp &= ~3; return 1;

    /* ---- branches ------------------------------------------------ */
    case 16: {                              /* bc                      */
        u32 bo = PPE_RT(w);
        ppe_clear(o);
        o->unit = PU_BRU; o->lat = 0; o->flags = PI_BRANCH;
        if (!(bo & 0x10u)) o->use_crf = PPE_CRFBIT(PPE_RA(w) >> 2);
        if (!(bo & 0x04u)) {
            o->flags |= PI_CTRDEC;
            o->use_spr |= PS_CTR; o->def_spr |= PS_CTR;
        }
        if (w & 1u) { o->flags |= PI_LINK; o->def_spr |= PS_LR; }
        return 1;
    }
    case 18:                                /* b / bl                  */
        ppe_clear(o);
        o->unit = PU_BRU; o->lat = 0; o->flags = PI_BRANCH;
        if (w & 1u) { o->flags |= PI_LINK | PI_CALL; o->def_spr |= PS_LR; }
        return 1;
    case 19: {
        u32 xo = PPE_XO10(w);
        if (xo == 16 || xo == 528) {        /* bclr / bcctr            */
            u32 bo = PPE_RT(w);
            ppe_clear(o);
            o->unit = PU_BRU; o->lat = 0; o->flags = PI_BRANCH;
            o->use_spr |= (xo == 16) ? PS_LR : PS_CTR;
            if (!(bo & 0x10u)) o->use_crf = PPE_CRFBIT(PPE_RA(w) >> 2);
            if (!(bo & 0x04u)) { o->flags |= PI_CTRDEC;
                                 o->use_spr |= PS_CTR; o->def_spr |= PS_CTR; }
            if (w & 1u) { o->flags |= PI_LINK | PI_CALL; o->def_spr |= PS_LR; }
            return 1;
        }
        if (xo == 150) {                    /* isync                   */
            ppe_clear(o);
            o->unit = PU_MCU; o->flags = PI_BARRIER | PI_SERIAL;
            o->lat = PPE_LAT_SERIAL;
            return 1;
        }
        if (xo == 0) {                      /* mcrf                    */
            ppe_clear(o);
            o->unit = PU_CRU; o->lat = PPE_LAT_CR;
            o->def_crf = PPE_CRFBIT(PPE_RT(w) >> 2);
            o->use_crf = PPE_CRFBIT(PPE_RA(w) >> 2);
            return 1;
        }
        /* CR logical: crand/cror/crxor/crnand/crnor/creqv/crandc/crorc */
        if (xo == 257 || xo == 449 || xo == 193 || xo == 225 ||
            xo ==  33 || xo == 289 || xo == 129 || xo == 417) {
            ppe_clear(o);
            o->unit = PU_CRU; o->lat = PPE_LAT_CR;
            o->def_crf = PPE_CRFBIT(PPE_RT(w) >> 2);
            o->use_crf = (u8)(PPE_CRFBIT(PPE_RA(w) >> 2) |
                              PPE_CRFBIT(PPE_RB(w) >> 2));
            return 1;
        }
        ppe_unknown(o);
        return 0;
    }

    /* ---- opcode 31 ----------------------------------------------- */
    case 31: {
        u32 xo = PPE_XO10(w);
        switch (xo) {
        /* logic and shifts: ra <- rs op rb */
        case 28: case 60: case 476: case 444: case 412: case 124:
        case 316: case 284: case 24: case 536: case 792: case 27:
        case 539: case 794:
            ppe_xrr(o, w, 1);
            if (xo == 792 || xo == 794) o->def_spr |= PS_XER;  /* sraw(d) */
            return 1;
        /* unary: ra <- f(rs) */
        case 954: case 922: case 986: case 26: case 58:
            ppe_xrr(o, w, 0);
            return 1;
        case 824:                            /* srawi  (writes XER[CA]) */
            ppe_xrr(o, w, 0); o->def_spr |= PS_XER; return 1;
        case 826: case 827:                  /* sradi (XS-form)         */
            ppe_xrr(o, w, 0); o->def_spr |= PS_XER; return 1;

        /* compares */
        case 0: case 32:
            ppe_clear(o);
            o->use_gpr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RB(w));
            o->def_crf = PPE_CRFBIT(PPE_RT(w) >> 2);
            o->lat = PPE_LAT_CR; o->flags = PI_CMP;
            return 1;

        /* loads, GPR */
        case 87: case 279: case 343: case 23: case 21: case 341:
        case 534: case 790:
            ppe_ld(o, w, 0, 1); return 1;
        /* loads, FPR */
        case 535: case 599:
            ppe_ld(o, w, 1, 1); return 1;
        /* stores, GPR */
        case 215: case 407: case 151: case 149: case 662: case 918:
            ppe_st(o, w, 0, 1); return 1;
        /* stores, FPR */
        case 663: case 727:
            ppe_st(o, w, 1, 1); return 1;

        case 20:                             /* lwarx                   */
            ppe_ld(o, w, 0, 1);
            o->flags |= PI_BARRIER; o->unit = PU_MCU; return 1;
        case 150:                            /* stwcx.                  */
            ppe_st(o, w, 0, 1);
            o->flags |= PI_BARRIER; o->unit = PU_MCU;
            o->def_crf = PPE_CRFBIT(0); return 1;

        /* cache ops: treated as stores (dcbz writes a line) or barriers */
        case 1014:                           /* dcbz                    */
            ppe_clear(o);
            o->unit = PU_LSU; o->flags = PI_STORE; o->lat = 0;
            if (PPE_RA(w)) o->use_gpr |= PPE_GPRBIT(PPE_RA(w));
            o->use_gpr |= PPE_GPRBIT(PPE_RB(w));
            return 1;
        case 278: case 86: case 54:          /* dcbt / dcbf / dcbst     */
            ppe_clear(o);
            o->unit = PU_LSU; o->flags = PI_LOAD; o->lat = 0;
            if (PPE_RA(w)) o->use_gpr |= PPE_GPRBIT(PPE_RA(w));
            o->use_gpr |= PPE_GPRBIT(PPE_RB(w));
            return 1;
        case 982:                            /* icbi                    */
        case 598:                            /* sync                    */
            ppe_clear(o);
            o->unit = PU_MCU; o->flags = PI_BARRIER | PI_SERIAL;
            o->lat = PPE_LAT_SERIAL;
            if (xo == 982) {
                if (PPE_RA(w)) o->use_gpr |= PPE_GPRBIT(PPE_RA(w));
                o->use_gpr |= PPE_GPRBIT(PPE_RB(w));
            }
            return 1;

        /* CR / SPR movement — the microcoded corner this exercise is about */
        case 19:                             /* mfcr                    */
            ppe_clear(o);
            o->unit = PU_MCU; o->flags = PI_SERIAL; o->lat = PPE_LAT_SERIAL;
            o->def_gpr = PPE_GPRBIT(PPE_RT(w));
            o->use_crf = 0xFFu;
            return 1;
        case 144: {                          /* mtcrf / mtocrf          */
            u32 mask = (w >> 12) & 0xFFu;
            int one  = mask && !(mask & (mask - 1u));
            ppe_clear(o);
            o->use_gpr = PPE_GPRBIT(PPE_RT(w));
            o->def_crf = (u8)mask;
            if (one && (w & (1u << 20))) {   /* mtocrf: single field, fast */
                o->unit = PU_FXU; o->lat = PPE_LAT_CR;
            } else {
                o->unit = PU_MCU; o->flags = PI_SERIAL; o->lat = PPE_LAT_SERIAL;
            }
            return 1;
        }
        case 339: {                          /* mfspr                   */
            u32 spr = ((w >> 16) & 0x1Fu) | (((w >> 11) & 0x1Fu) << 5);
            ppe_clear(o);
            o->def_gpr = PPE_GPRBIT(PPE_RT(w));
            o->lat = PPE_LAT_SPR;
            if (spr == 8) o->use_spr = PS_LR;
            else if (spr == 9) o->use_spr = PS_CTR;
            else if (spr == 1) o->use_spr = PS_XER;
            else { o->unit = PU_MCU; o->flags = PI_SERIAL; }   /* mftb etc */
            return 1;
        }
        case 467: {                          /* mtspr                   */
            u32 spr = ((w >> 16) & 0x1Fu) | (((w >> 11) & 0x1Fu) << 5);
            ppe_clear(o);
            o->use_gpr = PPE_GPRBIT(PPE_RT(w));
            o->lat = PPE_LAT_SPR;
            if (spr == 8) o->def_spr = PS_LR;
            else if (spr == 9) o->def_spr = PS_CTR;
            else if (spr == 1) o->def_spr = PS_XER;
            else { o->unit = PU_MCU; o->flags = PI_SERIAL; }
            return 1;
        }

        /* VMX load/store */
        case 103: case 6: case 38: case 71:
            ppe_clear(o);
            o->unit = PU_LSU; o->flags = PI_LOAD; o->lat = PPE_LAT_LOADF;
            o->def_vr = PPE_GPRBIT(PPE_RT(w));
            if (PPE_RA(w)) o->use_gpr |= PPE_GPRBIT(PPE_RA(w));
            o->use_gpr |= PPE_GPRBIT(PPE_RB(w));
            o->mem_class = ppe_mem_class(PPE_RA(w));
            return 1;
        case 231: case 199:
            ppe_clear(o);
            o->unit = PU_LSU; o->flags = PI_STORE; o->lat = 0;
            o->use_vr = PPE_GPRBIT(PPE_RT(w));
            if (PPE_RA(w)) o->use_gpr |= PPE_GPRBIT(PPE_RA(w));
            o->use_gpr |= PPE_GPRBIT(PPE_RB(w));
            o->mem_class = ppe_mem_class(PPE_RA(w));
            return 1;

        default:
            break;
        }
        /* XO-form arithmetic: nine-bit XO with OE in bit 10. */
        switch (PPE_XO9(w)) {
        case 266: case 40:                    /* add / subf             */
            ppe_xorr(o, w, 1); return 1;
        case 10: case 8:                      /* addc / subfc           */
            ppe_xorr(o, w, 1); o->def_spr |= PS_XER; return 1;
        case 138: case 136:                   /* adde / subfe           */
            ppe_xorr(o, w, 1); o->def_spr |= PS_XER; o->use_spr |= PS_XER;
            return 1;
        case 235: case 75: case 11: case 233:  /* mullw mulhw mulhwu mulld */
            ppe_xorr(o, w, 1);
            o->lat = PPE_LAT_MUL; o->flags |= PI_MULDIV; return 1;
        case 491: case 459: case 489: case 457: /* divw divwu divd divdu */
            ppe_xorr(o, w, 1);
            o->lat = PPE_LAT_DIV; o->flags |= PI_MULDIV;
            o->unit = PU_MCU; o->flags |= PI_SERIAL; return 1;
        case 104:                             /* neg                    */
            ppe_xorr(o, w, 0); return 1;
        case 234: case 202: case 232: case 200: /* addme addze subfme subfze */
            ppe_xorr(o, w, 0);
            o->def_spr |= PS_XER; o->use_spr |= PS_XER; return 1;
        default:
            break;
        }
        ppe_unknown(o);
        return 0;
    }

    /* ---- floating point ------------------------------------------ */
    case 59: {                                  /* single precision      */
        u32 xo5 = PPE_XO5(w);
        ppe_clear(o);
        o->unit = PU_FPU; o->lat = PPE_LAT_FPU;
        o->def_fpr = PPE_GPRBIT(PPE_RT(w));
        o->def_spr = PS_FPSCR;
        switch (xo5) {
        case 21: case 20: case 18:              /* fadds fsubs fdivs     */
            o->use_fpr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RB(w));
            if (xo5 == 18) { o->lat = PPE_LAT_DIV; o->unit = PU_FPU; }
            break;
        case 25:                                /* fmuls                 */
            o->use_fpr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RC(w));
            break;
        case 29: case 28: case 31: case 30:     /* fmadds family         */
            o->use_fpr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RB(w)) |
                         PPE_GPRBIT(PPE_RC(w));
            break;
        case 24:                                /* fres                  */
            o->use_fpr = PPE_GPRBIT(PPE_RB(w));
            break;
        default:
            ppe_unknown(o); return 0;
        }
        if (PPE_RCBIT(w)) o->def_crf |= PPE_CRFBIT(1);
        return 1;
    }
    case 63: {
        u32 xo = PPE_XO10(w);
        /* X-form members first: none of their XOs has an A-form value in its
         * low five bits, so the two decodes cannot collide (the same argument
         * jit_compile.c relies on for the guest side). */
        switch (xo) {
        case 0: case 32:                        /* fcmpu / fcmpo         */
            ppe_clear(o);
            o->unit = PU_FPU; o->lat = PPE_LAT_FPU;
            o->use_fpr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RB(w));
            o->def_crf = PPE_CRFBIT(PPE_RT(w) >> 2);
            o->def_spr = PS_FPSCR; o->flags = PI_CMP;
            return 1;
        case 72: case 40: case 264: case 136:   /* fmr fneg fabs fnabs   */
            ppe_clear(o);
            o->unit = PU_FPU; o->lat = PPE_LAT_FPU;
            o->def_fpr = PPE_GPRBIT(PPE_RT(w));
            o->use_fpr = PPE_GPRBIT(PPE_RB(w));
            if (PPE_RCBIT(w)) o->def_crf |= PPE_CRFBIT(1);
            return 1;
        case 12: case 14: case 15: case 814: case 815: case 846:
            ppe_clear(o);                        /* frsp fctiw(z) fctid(z) fcfid */
            o->unit = PU_FPU; o->lat = PPE_LAT_FPU;
            o->def_fpr = PPE_GPRBIT(PPE_RT(w));
            o->use_fpr = PPE_GPRBIT(PPE_RB(w));
            o->def_spr = PS_FPSCR;
            if (PPE_RCBIT(w)) o->def_crf |= PPE_CRFBIT(1);
            return 1;
        case 583:                                /* mffs                  */
            ppe_clear(o);
            o->unit = PU_MCU; o->flags = PI_SERIAL; o->lat = PPE_LAT_SERIAL;
            o->def_fpr = PPE_GPRBIT(PPE_RT(w));
            o->use_spr = PS_FPSCR;
            return 1;
        case 711:                                /* mtfsf                 */
            ppe_clear(o);
            o->unit = PU_MCU; o->flags = PI_SERIAL | PI_BARRIER;
            o->lat = PPE_LAT_SERIAL;
            o->use_fpr = PPE_GPRBIT(PPE_RB(w));
            o->def_spr = PS_FPSCR;
            return 1;
        default:
            break;
        }
        {
            u32 xo5 = PPE_XO5(w);
            ppe_clear(o);
            o->unit = PU_FPU; o->lat = PPE_LAT_FPU;
            o->def_fpr = PPE_GPRBIT(PPE_RT(w));
            o->def_spr = PS_FPSCR;
            switch (xo5) {
            case 21: case 20: case 18:
                o->use_fpr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RB(w));
                if (xo5 == 18) o->lat = PPE_LAT_DIV;
                break;
            case 25:
                o->use_fpr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RC(w));
                break;
            case 29: case 28: case 31: case 30: case 23:  /* madds, fsel  */
                o->use_fpr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RB(w)) |
                             PPE_GPRBIT(PPE_RC(w));
                if (xo5 == 23) o->def_spr = 0;            /* fsel: no FPSCR */
                break;
            case 22: case 26:                             /* fsqrt frsqrte */
                o->use_fpr = PPE_GPRBIT(PPE_RB(w));
                o->lat = PPE_LAT_DIV;
                break;
            default:
                ppe_unknown(o); return 0;
            }
            if (PPE_RCBIT(w)) o->def_crf |= PPE_CRFBIT(1);
            return 1;
        }
    }

    /* ---- VMX ------------------------------------------------------ */
    case 4: {
        u32 va = w & 0x3Fu;
        ppe_clear(o);
        o->unit = PU_VSU; o->lat = PPE_LAT_VSU;
        o->def_vr = PPE_GPRBIT(PPE_RT(w));
        if (va >= 42 && va <= 47) {              /* VA-form: vd,va,vb,vc  */
            o->use_vr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RB(w)) |
                        PPE_GPRBIT(PPE_RC(w));
            if (va == 46 || va == 47) o->lat = PPE_LAT_VSUF;
            return 1;
        }
        {   /* VX-form */
            u32 xo = w & 0x7FFu;
            o->use_vr = PPE_GPRBIT(PPE_RA(w)) | PPE_GPRBIT(PPE_RB(w));
            switch (xo) {
            case 10: case 74: case 1034: case 1098:      /* float ALU     */
            case 266: case 330:
                o->lat = PPE_LAT_VSUF; break;
            case 778: case 842: case 906: case 970:      /* conversions   */
                o->lat = PPE_LAT_VSUF;
                o->use_vr = PPE_GPRBIT(PPE_RB(w));       /* uimm in va    */
                break;
            case 908:                                     /* vspltisw     */
                o->use_vr = 0; break;
            case 652:                                     /* vspltw       */
                o->use_vr = PPE_GPRBIT(PPE_RB(w)); break;
            case 526: case 654: case 590: case 718:       /* vupk*        */
                o->use_vr = PPE_GPRBIT(PPE_RB(w)); break;
            case 1028: case 1092: case 1156: case 1220: case 1284:
            case 140: case 396: case 14: case 78: case 398: case 462:
            case 142: case 206:
                break;
            default:
                ppe_unknown(o); return 0;
            }
            return 1;
        }
    }

    default:
        ppe_unknown(o);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Dependence between two decoded words, a before b                     */
/* ------------------------------------------------------------------ */

/* True if b must not be hoisted above a (RAW, WAR or WAW on any resource,
 * or a memory ordering constraint). Memory is disambiguated the only way it
 * safely can be here: two loads are independent of each other, everything
 * else that touches memory is ordered. Nothing in the emitted code depends
 * on load-load order -- there is no other agent writing guest memory inside a
 * block -- while a load moved above a store, or any store reordered, could
 * change what the guest observes. */
PPE_INLINE int ppe_depends(const PpeInsn *a, const PpeInsn *b)
{
    if ((a->flags | b->flags) & (PI_BARRIER | PI_UNKNOWN))
        return 1;
    if ((a->def_gpr & (b->use_gpr | b->def_gpr)) ||
        (b->def_gpr &  a->use_gpr))
        return 1;
    if ((a->def_fpr & (b->use_fpr | b->def_fpr)) ||
        (b->def_fpr &  a->use_fpr))
        return 1;
    if ((a->def_vr  & (b->use_vr  | b->def_vr )) ||
        (b->def_vr  &  a->use_vr ))
        return 1;
    if ((a->def_crf & (b->use_crf | b->def_crf)) ||
        (b->def_crf &  a->use_crf))
        return 1;
    if ((a->def_spr & (b->use_spr | b->def_spr)) ||
        (b->def_spr &  a->use_spr))
        return 1;
    {
        u32 am = a->flags & (PI_LOAD | PI_STORE);
        u32 bm = b->flags & (PI_LOAD | PI_STORE);
        if (am && bm) {
            /* Two loads never conflict: nothing else writes guest memory
             * while a block runs. Otherwise they conflict unless they
             * provably address different objects (see PM_* above). */
            if (!(am == PI_LOAD && bm == PI_LOAD)) {
                if (a->mem_class == PM_UNKNOWN || b->mem_class == PM_UNKNOWN ||
                    a->mem_class == b->mem_class)
                    return 1;
            }
        }
    }
    return 0;
}

/* True if b reads something a produced (a RAW edge only) — the edge that
 * carries a latency, as opposed to the ordering-only WAR/WAW edges. */
PPE_INLINE int ppe_raw(const PpeInsn *a, const PpeInsn *b)
{
    if ((a->def_gpr & b->use_gpr) || (a->def_fpr & b->use_fpr) ||
        (a->def_vr  & b->use_vr ) || (a->def_crf & b->use_crf) ||
        (a->def_spr & b->use_spr))
        return 1;
    return 0;
}

/* The PPE issues at most two instructions per cycle, and only from different
 * units. Microcoded instructions issue alone. */
PPE_INLINE int ppe_can_pair(const PpeInsn *a, const PpeInsn *b)
{
    if ((a->flags | b->flags) & (PI_SERIAL | PI_UNKNOWN))
        return 0;
    if (a->unit == b->unit)
        return 0;
    if (a->unit == PU_BRU)          /* a branch is the end of its pair */
        return 0;
    if (ppe_depends(a, b))
        return 0;
    return 1;
}

#endif /* DOLPHIN_CORE_PPC_JIT_PPE_MODEL_H */
