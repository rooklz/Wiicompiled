/* ppe_stall.h — a static stall analyser for the code the JIT emits.
 *
 * WHY THIS EXISTS, AND WHY IT IS NOT A TIMER
 *
 * The arithmetic that motivates it: the recompiler expands guest code by
 * about 5.6-6 host words per guest instruction, and a 3.2 GHz in-order PPE
 * retiring 1 instruction per cycle at 6x expansion would run ~530 M guest
 * inst/s.  Measured on the console: ~248 M.  Roughly half the gap is not
 * instruction count -- it is the pipeline standing still.
 *
 * That half cannot be measured from here.  The PPE has performance counters,
 * but we have no console to run them on; and qemu wall-clock is NOT a proxy,
 * not even a bad one.  qemu-ppc64 is an interpreter: it has no issue width,
 * no latencies, no branch predictor.  A block that stalls for 40 cycles and a
 * block that never stalls take the same time under qemu, so any "it got
 * faster under qemu" claim about scheduling is measuring the host x86, not
 * the PPE.  The only honest instrument available is a model of the machine
 * applied to the exact words the JIT emits -- this file.
 *
 * WHAT IT COMPUTES
 *
 * For one block's hot path (the words before the cold tail, i.e. the ones
 * actually fetched when nothing bails out), it runs the ppe_model.h issue
 * rules forward and charges every cycle in which the machine could not issue
 * to a cause:
 *
 *   load-use      a load's result was not ready for its consumer
 *   alu-dep       a 2-cycle fixed-point result was consumed too soon
 *   mul/div       an 11- or 40-cycle fixed-point result
 *   fp-dep        a 10-cycle FPU result
 *   vmx-dep       a VMX result
 *   cr-dep        a CR field consumed by non-branch code too soon
 *   cr->branch    a CR field consumed by the branch that tests it
 *   spr-dep       mtctr/mtlr feeding a bctr/blr
 *   microcode     mfcr / multi-field mtcrf / divide, which issue alone
 *
 * plus, separately and clearly labelled, a *modelled* branch-mispredict term.
 * That one is the only number here that rests on an assumption rather than on
 * a dependence: it needs a taken-rate, which static analysis cannot know.  It
 * is therefore reported apart from the pipeline-stall ranking and never mixed
 * into it.
 *
 * WHAT IT IS NOT
 *
 * Not a cycle-accurate simulator.  No cache-miss model (every load hits L1),
 * no SMT partner, no fetch-group or i-cache effects, no store-queue.  The
 * absolute cycle counts it prints are therefore lower bounds and should never
 * be quoted as a predicted frame time.  Its job is to rank causes, and to
 * compare the same corpus of blocks before and after an emitter change --
 * comparisons the omitted effects do not move, because they are identical on
 * both sides.
 */
#ifndef DOLPHIN_CORE_PPC_JIT_PPE_STALL_H
#define DOLPHIN_CORE_PPC_JIT_PPE_STALL_H

#include "ppe_model.h"

enum {
    SC_LOADUSE = 0,
    SC_ALUDEP,
    SC_MULDIV,
    SC_FPDEP,
    SC_VSUDEP,
    SC_CRDEP,
    SC_CR2BR,
    SC_SPRDEP,
    SC_SERIAL,
    SC_COUNT
};

/* Mispredict rates.  The model's only assumptions, isolated here so they can
 * be argued with in one place -- which is why the mispredict total is
 * reported apart from the dependence-derived stall ranking and never folded
 * into it.
 *
 * A conditional branch is charged the rate of one whose static hint agrees
 * with the direction the emitter lays out as common, because the emitter now
 * makes that true by construction (see BO_LIKELY_TAKEN in jit_compile.c).
 * Static analysis cannot do better than that: scoring prediction accuracy
 * needs a taken-rate, which only execution knows.
 *
 * Indirect branches are where the real modelled cost is.  `bctr` back to the
 * dispatcher has a target no predictor can learn; `blr` is covered by the
 * link stack. */
#define PPE_P_HINT_OK    0.02
#define PPE_P_CTR        0.60
#define PPE_P_LR         0.05

typedef struct {
    u32    cycles;              /* modelled issue cycles, one traversal   */
    u32    words;               /* words modelled                          */
    u32    dual;                /* instructions that paired                */
    u32    unknown;             /* words the decoder did not recognise     */
    u32    stall[SC_COUNT];
    u32    loop_from, loop_to;  /* an internal back edge, if the path has  */
                                /* one: [from, to) is the loop body, which */
                                /* runs once per ITERATION rather than once */
                                /* per dispatch                             */
    u32    conds;               /* conditional branches on the hot path    */
    u32    hinted;              /* ...of which carry an explicit y hint     */
    u32    indirect;            /* blr/bctr: the dispatcher round trip      */
    double mispredict;          /* modelled, fractional                    */
} PpeStallResult;

/* Which cause class an instruction's *result* belongs to, for attribution. */
PPE_INLINE u8 ppe_cause_of(const PpeInsn *p)
{
    if (p->flags & PI_LOAD)                       return SC_LOADUSE;
    if (p->flags & PI_MULDIV)                     return SC_MULDIV;
    if (p->flags & (PI_SERIAL | PI_UNKNOWN))      return SC_SERIAL;
    if (p->unit == PU_FPU)                        return SC_FPDEP;
    if (p->unit == PU_VSU)                        return SC_VSUDEP;
    if (p->unit == PU_CRU)                        return SC_CRDEP;
    if (p->def_spr & (PS_LR | PS_CTR))            return SC_SPRDEP;
    if (p->flags & PI_CMP)                        return SC_CRDEP;
    return SC_ALUDEP;
}

/* Per-cause attribution of *where* the stall came from, so a ranking can be
 * turned into an edit.  Keyed by the producing and consuming encodings,
 * compressed to (opcd, xo) pairs. */
#define PPE_SITE_SLOTS 2048
typedef struct {
    u32 key;                    /* 0 = empty                              */
    u32 cause;
    u32 pclass;                 /* producer's PM_* memory class, if any   */
    u64 cycles;
    u64 count;
} PpeSite;

PPE_INLINE u32 ppe_site_key(u32 prod, u32 cons)
{
    u32 po = PPE_OPCD(prod), co = PPE_OPCD(cons);
    u32 a = (po << 10) | ((po == 31 || po == 63 || po == 19) ? PPE_XO10(prod) : 0u);
    u32 b = (co << 10) | ((co == 31 || co == 63 || co == 19) ? PPE_XO10(cons) : 0u);
    u32 k = (a << 16) ^ b;
    return k ? k : 1u;
}

PPE_INLINE void ppe_site_add(PpeSite *tab, u32 key, u32 cause, u32 pclass,
                             u64 cycles)
{
    u32 i = ((key ^ (pclass * 0x9E3779B1u)) * 2654435761u) & (PPE_SITE_SLOTS - 1u);
    u32 n = 0;
    for (; n < PPE_SITE_SLOTS; n++) {
        if (tab[i].key == 0) { tab[i].key = key; tab[i].cause = cause;
                               tab[i].pclass = pclass; }
        if (tab[i].key == key && tab[i].cause == cause &&
            tab[i].pclass == pclass) {
            tab[i].cycles += cycles; tab[i].count++;
            return;
        }
        i = (i + 1u) & (PPE_SITE_SLOTS - 1u);
    }
}

/* ------------------------------------------------------------------ */
/* The simulation                                                       */
/*                                                                      */
/* One pass over the hot path, assuming the fall-through direction at    */
/* every conditional branch: that is the path a guarded access takes     */
/* when it does not bail out and an inlined conditional takes when it    */
/* is not taken, which is by construction the common one.                */
/* ------------------------------------------------------------------ */
PPE_INLINE void ppe_stall_analyse(const u32 *code, u32 n_words,
                                  PpeStallResult *r, PpeSite *sites)
{
    /* Ready time and producing word index, per architected resource. */
    u32 rd_gpr[32], rd_fpr[32], rd_vr[32], rd_crf[8], rd_spr[4];
    u32 pw_gpr[32], pw_fpr[32], pw_vr[32], pw_crf[8], pw_spr[4];
    u32 cycle = 0, in_cycle = 0, i, k;
    PpeInsn prev;
    int have_prev = 0;

    for (k = 0; k < 32; k++) { rd_gpr[k] = rd_fpr[k] = rd_vr[k] = 0;
                               pw_gpr[k] = pw_fpr[k] = pw_vr[k] = 0xFFFFFFFFu; }
    for (k = 0; k < 8;  k++) { rd_crf[k] = 0; pw_crf[k] = 0xFFFFFFFFu; }
    for (k = 0; k < 4;  k++) { rd_spr[k] = 0; pw_spr[k] = 0xFFFFFFFFu; }

    for (k = 0; k < SC_COUNT; k++) r->stall[k] = 0;
    r->cycles = r->words = r->dual = r->unknown = 0;
    r->conds = r->hinted = r->indirect = 0;
    r->loop_from = r->loop_to = 0;
    r->mispredict = 0.0;

    for (i = 0; i < n_words; i++) {
        PpeInsn cur;
        u32 dep = 0, dep_from = 0xFFFFFFFFu;
        u8  cause = SC_ALUDEP;

        if (!ppe_decode(code[i], &cur))
            r->unknown++;

        /* Earliest cycle the operands allow, and which producer set it. */
#define PPE_DEP(readyarr, prodarr, mask, count)                              \
        do { u32 q;                                                          \
             for (q = 0; q < (count); q++)                                   \
                 if (((mask) >> q) & 1u)                                     \
                     if ((readyarr)[q] > dep) {                              \
                         dep = (readyarr)[q]; dep_from = (prodarr)[q];        \
                     }                                                       \
        } while (0)
        PPE_DEP(rd_gpr, pw_gpr, cur.use_gpr, 32);
        PPE_DEP(rd_fpr, pw_fpr, cur.use_fpr, 32);
        PPE_DEP(rd_vr,  pw_vr,  cur.use_vr,  32);
        PPE_DEP(rd_crf, pw_crf, cur.use_crf, 8);
        PPE_DEP(rd_spr, pw_spr, cur.use_spr, 4);
#undef PPE_DEP

        if (dep_from != 0xFFFFFFFFu && dep_from < i) {
            PpeInsn prod;
            ppe_decode(code[dep_from], &prod);
            cause = ppe_cause_of(&prod);
            /* A CR field consumed by the branch that tests it is its own
             * cause: it is the one the emitter fixes by separating the
             * compare from the branch, not by breaking an ALU chain. */
            if (cause == SC_CRDEP && (cur.flags & PI_BRANCH))
                cause = SC_CR2BR;
        }

        /* In-order issue. */
        for (;;) {
            if (in_cycle >= 2) { cycle++; in_cycle = 0; have_prev = 0; continue; }
            if (in_cycle == 1 && (!have_prev || !ppe_can_pair(&prev, &cur))) {
                cycle++; in_cycle = 0; have_prev = 0; continue;
            }
            if (dep > cycle) {
                u32 lost = dep - cycle;
                r->stall[cause] += lost;
                if (sites && dep_from != 0xFFFFFFFFu && dep_from < i) {
                    PpeInsn pd;
                    ppe_decode(code[dep_from], &pd);
                    ppe_site_add(sites, ppe_site_key(code[dep_from], code[i]),
                                 cause,
                                 (pd.flags & (PI_LOAD | PI_STORE)) ? pd.mem_class : 0u,
                                 lost);
                }
                cycle = dep; in_cycle = 0; have_prev = 0;
                continue;
            }
            break;
        }

        if (in_cycle == 1) r->dual++;
        in_cycle++;
        prev = cur; have_prev = 1;
        r->words++;

        /* Publish results. */
        {
            u32 ready = cycle + cur.lat;
            for (k = 0; k < 32; k++) {
                if ((cur.def_gpr >> k) & 1u) { rd_gpr[k] = ready; pw_gpr[k] = i; }
                if ((cur.def_fpr >> k) & 1u) { rd_fpr[k] = ready; pw_fpr[k] = i; }
                if ((cur.def_vr  >> k) & 1u) { rd_vr[k]  = ready; pw_vr[k]  = i; }
            }
            for (k = 0; k < 8; k++)
                if ((cur.def_crf >> k) & 1u) { rd_crf[k] = ready; pw_crf[k] = i; }
            for (k = 0; k < 4; k++)
                if ((cur.def_spr >> k) & 1u) { rd_spr[k] = ready; pw_spr[k] = i; }
        }

        /* A microcoded instruction occupies the machine alone. */
        if (cur.flags & (PI_SERIAL | PI_UNKNOWN))
            in_cycle = 2;

        /* Branches: the modelled mispredict term.
         *
         * Static analysis cannot score prediction *accuracy* -- that needs a
         * taken-rate, which only execution knows -- so this term deliberately
         * does not try. Every conditional branch is charged the rate of a
         * branch whose static hint agrees with the layout, because the
         * emitter now makes that true by construction (jit_compile.c carries
         * the guest compiler's own y bit across, and hints the guard and
         * exception skips taken). What remains, and what dominates, is the
         * indirect branch: the `bctr` that returns to the dispatcher has an
         * unpredictable target, and there are as many of them as there are
         * block exits that do not resolve to a direct link. */
        if (cur.flags & PI_BRANCH) {
            u32 opc = PPE_OPCD(code[i]);
            if (opc == 16) {
                s32 bd = (s32)(s16)(code[i] & 0xFFFCu);
                if (PPE_RT(code[i]) & 1u) r->hinted++;
                r->mispredict += PPE_MISPREDICT * PPE_P_HINT_OK;
                r->conds++;
                /* A conditional branch backwards inside the modelled path is
                 * a loop back edge -- a retained bdnz loop's budget branch, or
                 * a warm self-loop's. Its body runs once per ITERATION, and
                 * there can be thousands of iterations per dispatch, so it has
                 * to be weighted separately or every loop in the corpus is
                 * counted once. (An idle-skip block deliberately has no back
                 * edge: it zeroes the downcount and yields, which is exactly
                 * why its enormous attributed instruction count must NOT be
                 * read as iterations.) */
                if (bd < 0 && !(code[i] & 2u)) {
                    s32 t = (s32)i + bd / 4;
                    if (t >= 0) { r->loop_from = (u32)t; r->loop_to = i + 1u; }
                }
            } else if (opc == 19) {
                u32 xo = PPE_XO10(code[i]);
                r->mispredict += PPE_MISPREDICT *
                    (xo == 16 ? PPE_P_LR : PPE_P_CTR);
                r->indirect++;
            }
        }

        /* The hot path ends at the first unconditional transfer that is not a
         * call.
         *
         * This matters most at a linked block exit: the emitted tail is
         *
         *     addi/cmpwi/bc  -> dispatcher side
         *     b              <- LINK SITE, patched to the next block
         *     <dispatcher tail: stores, mtctr, bctr>
         *
         * and once the link resolves -- which is the design, and what the
         * console runs -- control leaves at the `b` and the dispatcher tail
         * is never fetched. Modelling past it would charge every block exit
         * for a round trip it does not make. Exits that genuinely cannot be
         * linked (a guest `blr`, a computed target, an AOT-owned pc) have no
         * link site at all: they end in `mtctr`/`bctr`, which this loop does
         * model, and where the mtctr-to-bctr latency is therefore real. */
        {
            u32 opc = PPE_OPCD(code[i]);
            u32 bo  = PPE_RT(code[i]);
            int uncond = 0;
            if (code[i] & 1u) {
                uncond = 0;                     /* a call returns */
            } else if (opc == 18) {
                uncond = 1;
            } else if (opc == 16 || (opc == 19 &&
                       (PPE_XO10(code[i]) == 16 || PPE_XO10(code[i]) == 528))) {
                uncond = (bo & 0x14u) == 0x14u;
            }
            if (uncond)
                break;
        }
    }

    r->cycles = cycle + (in_cycle ? 1u : 0u);
}

PPE_INLINE const char *ppe_cause_name(unsigned c)
{
    static const char *const k[SC_COUNT] = {
        "load-use", "alu-dep", "mul/div", "fp-dep", "vmx-dep",
        "cr-dep", "cr->branch", "spr-dep", "microcode"
    };
    return (c < SC_COUNT) ? k[c] : "?";
}

#endif /* DOLPHIN_CORE_PPC_JIT_PPE_STALL_H */
