/* ppe_sched.h — schedule a compiled block's hot path for the in-order PPE.
 *
 * The recompiler emits code in the order it thinks about it: form an address,
 * use it; load a register, consume it; compare, branch. On an out-of-order
 * host that order is irrelevant. On the PPE it is most of the cost. The
 * machine issues two instructions per cycle, strictly in order, and every
 * dependence is a hole: a load's result is five cycles away, an ALU result
 * two, a `mtctr` six cycles from the `bctr` that reads it. The hardware will
 * not rescue a badly ordered pair -- it simply stops.
 *
 * The static model in ppe_stall.h measured that on a real Mario Kart Wii boot
 * and found 55.7% of modelled issue cycles were stalls -- load-use 47% of
 * them, dependent ALU chains 23%, `mtctr`-to-`bctr` at block exits 10%. This pass is the
 * general answer: after a block is compiled, its hot path is cut into
 * straight-line regions and each is list-scheduled against the same machine
 * model, longest-dependence-path first.
 *
 * WHAT MAKES THIS SAFE
 *
 *   Regions never cross a branch or a branch target. A region begins at the
 *   block entry, at a warm-loop entry, or at any word some branch in the hot
 *   path targets; it ends at (and includes) the next branch. Branches
 *   themselves never move, so every recorded word offset -- link sites, warm
 *   entries, the fixups emit_cold_tail has already patched -- still points at
 *   the same instruction afterwards. Scheduling is a permutation *inside* a
 *   region, so the architectural state at the region's end, which is the only
 *   state anything outside it can observe (including a guard's deferred cold
 *   bail-out, which is entered from the region's terminating branch), is
 *   unchanged.
 *
 *   Dependences are taken from ppe_model.h and include WAR and WAW, not just
 *   RAW: the register cache reuses host registers heavily and there is no
 *   renaming here, so anti-dependences are real. Memory is disambiguated the
 *   only way it safely can be: loads may be reordered with respect to one
 *   another (nothing else writes guest memory inside a block), and every
 *   other pairing that touches memory keeps its order. Anything the decoder
 *   does not recognise is a full barrier.
 *
 *   The escape and exit tails are never scheduled. They are not on the fast
 *   path, and one of them contains the one thing that must not be treated as
 *   an instruction: the inline guest-pc data word that follows the shared
 *   escape tail's `bl`.
 *
 *   A promoted REGION is scheduled, and is passed here as its own range. A
 *   region is hot code -- the taken side of an inlined conditional branch --
 *   that trace formation emits after the unit's entry region, interleaved
 *   with those tails, so it cannot be reached by extending the entry range;
 *   jit_compile.c records each region's extent and calls this once per
 *   region. A region is entered only at its first word (the guard branch that
 *   patched to it), which is exactly the contract this function's `code[0]`
 *   label already expresses.
 *
 * WHAT IT CANNOT DO
 *
 *   It cannot move work across a branch, so a stall whose producer and
 *   consumer sit on opposite sides of a guard has to be fixed where it is
 *   emitted (jit_compile.c does exactly that for the MMIO guard, whose
 *   address fold is now formed before the guard rather than after it).
 */
#ifndef DOLPHIN_CORE_PPC_JIT_PPE_SCHED_H
#define DOLPHIN_CORE_PPC_JIT_PPE_SCHED_H

#include "ppe_model.h"

/* JIT_TRACE_HARD_WORDS comes from jit.h, which every user of this header
 * includes first; the fallback keeps the header self-contained. */
#ifndef JIT_TRACE_HARD_WORDS
#define JIT_TRACE_HARD_WORDS 4500u
#endif

/* Largest region scheduled in one go. A longer straight-line run is cut into
 * chunks at this boundary, which costs only the scheduling freedom across the
 * cut -- never correctness. Real regions in emitted code are far shorter:
 * guards, exits and guest branches punctuate the stream every few words. */
#define PPE_SCHED_MAX 160

/* Working storage. The recompiler is single-threaded (one compile at a time,
 * called from the dispatcher), so this is file-static rather than 12 KiB of
 * stack on a console thread. */
typedef struct {
    u32     word[PPE_SCHED_MAX];
    PpeInsn dec[PPE_SCHED_MAX];
    u64     pred[PPE_SCHED_MAX][(PPE_SCHED_MAX + 63) / 64];
    u64     rawp[PPE_SCHED_MAX][(PPE_SCHED_MAX + 63) / 64];
    s32     height[PPE_SCHED_MAX];
    s32     earliest[PPE_SCHED_MAX];
    u8      done[PPE_SCHED_MAX];
    u32     out[PPE_SCHED_MAX];
} PpeSchedWork;

#define PPE_BIT_SET(a, i)  ((a)[(i) >> 6] |= (u64)1 << ((i) & 63))
#define PPE_BIT_GET(a, i)  (((a)[(i) >> 6] >> ((i) & 63)) & 1u)

PPE_INLINE int ppe_is_branch_word(u32 w)
{
    u32 op = PPE_OPCD(w);
    if (op == 16 || op == 18)
        return 1;
    if (op == 19) {
        u32 xo = PPE_XO10(w);
        return xo == 16 || xo == 528;
    }
    return 0;
}

/* Schedule words [0, n) of one region in place. */
PPE_INLINE void ppe_sched_region(PpeSchedWork *W, u32 *code, u32 n)
{
    u32 i, j, k, nw = (n + 63) / 64;
    u32 scheduled = 0, cycle = 0, in_cycle = 0;
    int have_prev = 0, pinned_last = 0;
    PpeInsn prev;
    u32 guard;

    if (n < 3)
        return;

    for (i = 0; i < n; i++) {
        W->word[i] = code[i];
        ppe_decode(code[i], &W->dec[i]);
        for (k = 0; k < nw; k++) { W->pred[i][k] = 0; W->rawp[i][k] = 0; }
        W->height[i] = 0;
        W->earliest[i] = 0;
        W->done[i] = 0;
    }

    /* A terminating branch stays last: everything that records a word offset
     * (link sites, the cold tail's already-patched fixups) points at it. */
    if (W->dec[n - 1].flags & PI_BRANCH)
        pinned_last = 1;

    /* Dependence edges, both directions of anti-dependence included. */
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++)
            if (ppe_depends(&W->dec[i], &W->dec[j])) {
                PPE_BIT_SET(W->pred[j], i);
                if (ppe_raw(&W->dec[i], &W->dec[j]))
                    PPE_BIT_SET(W->rawp[j], i);
            }
    if (pinned_last)
        for (i = 0; i + 1 < n; i++)
            PPE_BIT_SET(W->pred[n - 1], i);

    /* Critical-path height, so the longest dependence chain starts first. */
    for (i = n; i-- > 0; ) {
        for (j = i + 1; j < n; j++)
            if (PPE_BIT_GET(W->pred[j], i)) {
                s32 lat = PPE_BIT_GET(W->rawp[j], i) ? (s32)W->dec[i].lat : 1;
                if (W->height[j] + lat > W->height[i])
                    W->height[i] = W->height[j] + lat;
            }
    }

    /* List scheduling under the PPE issue rules. */
    {
        u64 smask[(PPE_SCHED_MAX + 63) / 64];
        for (k = 0; k < nw; k++) smask[k] = 0;
        guard = 0;
        while (scheduled < n && guard++ < PPE_SCHED_MAX * 64u) {
            int best = -1;
            for (i = 0; i < n; i++) {
                int ok = 1;
                if (W->done[i]) continue;
                for (k = 0; k < nw; k++)
                    if (W->pred[i][k] & ~smask[k]) { ok = 0; break; }
                if (!ok) continue;
                if (W->earliest[i] > (s32)cycle) continue;
                if (in_cycle == 1 &&
                    (!have_prev || !ppe_can_pair(&prev, &W->dec[i])))
                    continue;
                if (best < 0 || W->height[i] > W->height[best])
                    best = (int)i;
            }
            if (best < 0) {
                if (in_cycle > 0) { in_cycle = 0; have_prev = 0; }
                cycle++;
                continue;
            }
            W->done[best] = 1;
            W->out[scheduled++] = (u32)best;
            PPE_BIT_SET(smask, (u32)best);
            for (j = (u32)best + 1; j < n; j++)
                if (PPE_BIT_GET(W->rawp[j], (u32)best)) {
                    s32 t = (s32)cycle + (s32)W->dec[best].lat;
                    if (t > W->earliest[j]) W->earliest[j] = t;
                }
            if (W->dec[best].flags & (PI_SERIAL | PI_UNKNOWN)) {
                in_cycle = 0; have_prev = 0; cycle++;
            } else if (in_cycle == 1) {
                in_cycle = 0; have_prev = 0; cycle++;
            } else {
                in_cycle = 1; prev = W->dec[best]; have_prev = 1;
            }
        }
    }

    /* A scheduling failure must never produce wrong code: leave the region
     * exactly as emitted. */
    if (scheduled != n)
        return;

    for (i = 0; i < n; i++)
        code[i] = W->word[W->out[i]];
}

/* Schedule one entered-at-word-0 range of a finished unit: the entry region
 * ([0, hot_words)), or one promoted region ([from, to) of the cold tail).
 *
 * `extra_label` (or 0) is a word offset that must start a region even though
 * no branch in the range targets it -- the warm self-loop entry, which the
 * dispatcher and the back edge both reach directly. Regions have none.
 */
PPE_INLINE void ppe_sched_block(PpeSchedWork *W, u32 *code, u32 hot_words,
                                u32 extra_label)
{
    /* One byte per word of the range being scheduled. Sized from the trace
     * budget (jit.h): no single region of a unit can exceed
     * JIT_TRACE_HARD_WORDS, so nothing is ever silently skipped for want of
     * room here. The largest entry region seen over a full boot is 712 words.
     * Anything larger than the table is left unscheduled rather than
     * mis-scheduled. */
    static u8 label[JIT_TRACE_HARD_WORDS + 64u];
    u32 i, s;

    if (hot_words < 4 || hot_words > sizeof label)
        return;

    for (i = 0; i < hot_words; i++) label[i] = 0;
    label[0] = 1;
    if (extra_label && extra_label < hot_words) label[extra_label] = 1;

    /* Every branch target inside the range is an entry point. Only this range
     * is scanned: a branch leaving it (a guard branching out to its cold slot,
     * an exit) lands past `hot_words` and is simply not a label here, and the
     * words past the range include a data word that must never be decoded.
     * Nothing outside a range branches into its interior -- a cold slot's
     * fixup is patched to the FIRST word of what follows it, which is either a
     * tail or a region's own word 0. */
    for (i = 0; i < hot_words; i++) {
        u32 w = code[i];
        u32 op = PPE_OPCD(w);
        s32 d;
        if (op == 16)      d = (s32)(s16)(w & 0xFFFCu);
        else if (op == 18) d = (s32)(((s32)(w << 6)) >> 6) & ~3;
        else continue;
        if (w & 2u)        continue;            /* absolute (AA=1) */
        {
            s32 t = (s32)i + d / 4;
            if (t >= 0 && t < (s32)hot_words) label[t] = 1;
        }
    }

    i = 0;
    while (i < hot_words) {
        s = i;
        while (i < hot_words) {
            if (i > s && label[i]) break;
            if (i - s >= PPE_SCHED_MAX) break;
            if (ppe_is_branch_word(code[i])) { i++; break; }
            i++;
        }
        ppe_sched_region(W, code + s, i - s);
    }
}

#endif /* DOLPHIN_CORE_PPC_JIT_PPE_SCHED_H */
