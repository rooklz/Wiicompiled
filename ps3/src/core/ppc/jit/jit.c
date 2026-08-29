/* jit.c — code cache, block lookup, invalidation and the dispatch loop.
 *
 * Execution model: `jit_enter` (jit_entry.S) saves the non-volatile registers
 * once, pins the arena base and PPCState pointer, and then loops. Blocks are
 * jumped *into*, and leave by jumping back to the dispatch label held in r17 --
 * no per-block prologue, epilogue, stack frame or link-register traffic. The
 * 18 non-volatile registers are saved once per `jit_run` call rather than once
 * per block, which matters because the PPE's write-through L1 makes every one
 * of those stores an L2 write (docs/HARDWARE.md §1.2).
 */
#include "jit.h"
#include "../interp/interp.h"
#if !defined(__PS3__) && !defined(__lv2ppu__)
/* Analysis only: the static PPE stall model (JIT_SCHED=1). Never built
 * into the console image -- it exists to rank stall causes on the
 * workstation, where the console's performance counters cannot be read. */
#  include "ppe_stall.h"
#endif
#include "../../mem/memmap.h"
#include "../../../common/log.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#if defined(__PS3__) || defined(__lv2ppu__)
#  include <sys/memory.h>
#  include "../../mem/mem_platform.h"
#else
#  include <sys/mman.h>
#  ifndef MAP_ANONYMOUS
#    define MAP_ANONYMOUS MAP_ANON
#  endif
#endif

/* Only a PowerPC host can execute what we emit. Elsewhere the compiler still
 * runs (and is verified by disassembly), but execution uses the interpreter. */
#if defined(__powerpc64__) || defined(__PPC64__)
#  define JIT_CAN_EXECUTE 1
#else
#  define JIT_CAN_EXECUTE 0
#endif

/* ------------------------------------------------------------------ */

#define JIT_HASH_BITS  16
#define JIT_HASH_SIZE  (1u << JIT_HASH_BITS)
#define JIT_HASH_MASK  (JIT_HASH_SIZE - 1u)

/* Guest pages tracked for invalidation. The guest's instruction cache line is
 * 32 bytes, but tracking at that granularity would need a huge table; 4 KiB
 * pages keep the map small while making self-modifying code cheap to detect. */
#define JIT_PAGE_SHIFT 12
#define JIT_PAGE_COUNT (0x40000000u >> JIT_PAGE_SHIFT)   /* over the folded arena */

typedef struct {
    u8       *code;
    size_t    size;
    size_t    used;
    void     *handle;   /* opaque, owned by the platform allocator (lv2 only) */
} CodeBuffer;

static CodeBuffer  s_code;
static JitBlock  **s_hash;
static JitBlock  **s_pages;
static JitBlock   *s_blocks;
static u32         s_block_count;
static u32         s_block_capacity;
static int         s_inited;

/* Link sites whose target block did not exist yet when they were emitted. */
#define JIT_MAX_UNRESOLVED 4096

typedef struct {
    JitBlock *from;
    u8        index;
} UnresolvedLink;

static UnresolvedLink s_unresolved[JIT_MAX_UNRESOLVED];
static u32            s_unresolved_count;

/* Defined with the AOT table below; the linker must know which pcs the
 * table owns so it never wires a direct branch past the dispatcher. */
static int jit_aot_owns(u32 pc);

JitStats g_jit_stats;

/* Owned by jit_compile.c: the post-pass PPE instruction scheduler's
 * on/off switch. Always 1 on the console. */
extern int g_jit_sched_enable;
extern int g_jit_sched_regions;

/* Instrumentation: how many times an emitted guard actually escaped to the
 * interpreter -- the execution frequency of the cold path this work moved. */
u64 g_jit_force_interp_hits;

/* Instrumentation (compile-time accounting only): host words that end up in
 * the hot part of a block vs in its cold tail, summed over every block ever
 * compiled. Reported by the JIT_HIST dump. */
u64 g_jit_hot_words;
u64 g_jit_cold_words;
u64 g_jit_blocks_with_cold;

/* Compilation-unit size distribution, in guest instructions. Trace formation
 * is a policy over unit SIZE, so the distribution -- not the mean -- is what
 * says whether a change did what it claimed. Buckets are 1, 2, 3-4, 5-8, ...
 * up to >256. Compile-time accounting only. */
u64 g_jit_bsize[10];
u64 g_jit_bsize_insts[10];

#ifdef JIT_WORDPROF
/* The PPCState the recompiler last ran for, so the JIT_HIST dump can read the
 * exact executed-word counter the emitted code maintains (jit_compile.c,
 * "Exact executed-word accounting"). */
static PPCState *s_wp_state;
#endif

/* ------------------------------------------------------------------ */
/* Execution profiling (JIT_PROFILE=1 in the environment).              */
/*                                                                      */
/* When enabled, block linking is suppressed so every block transition   */
/* passes through jit_dispatch_c, and the guest instructions executed    */
/* since the previous dispatch -- the downcount delta, which the emitted */
/* code charges exactly, including per-iteration inside retained loops   */
/* and the whole-slice charge of an idle skip -- are attributed to the   */
/* block dispatched then. Counts live in a pc-keyed side table so they   */
/* survive jit_flush_all. Costs nothing measurable when the env var is   */
/* unset: one predicted branch per dispatch.                             */
/* ------------------------------------------------------------------ */

typedef struct {
    u32 pc;             /* guest pc of the block (0 = empty slot)       */
    u32 guest_insts;    /* block length last seen at this pc            */
    u32 hot_words;      /* host words before the cold tail, last seen   */
    u32 warm_words;     /* warm self-loop fast path words (0 = none)    */
    u64 execs;          /* dispatches                                   */
    u64 insts;          /* guest instructions attributed by downcount   */
    u64 real;           /* guest instructions ACTUALLY executed.
                         * `insts` is charged from the downcount delta, which
                         * an IDLE SKIP inflates by a whole slice: the OS idle
                         * loop shows 35.8 BILLION "instructions" while being
                         * correctly skipped. Ranking hot code by `insts` is
                         * therefore meaningless. `real` counts only what the
                         * block genuinely retired (execs * guest_insts), so
                         * it is the number to optimise against. */
#if !defined(__PS3__) && !defined(__lv2ppu__)
    /* JIT_SCHED=1: the static PPE stall model for this block's hot path,
     * filled in at compile time. Weighted by execs/insts at dump time, which
     * is what turns a per-block model into a whole-corpus ranking. */
    u8  sched_done;
    PpeStallResult sched;        /* one traversal of the entry region      */
    PpeStallResult sched_loop;   /* ...of the loop body inside it, if any  */
    /* Promoted regions: hot code the cold-tail sweep emitted past
     * hot_words. Summed over every region of the unit -- a traversal enters
     * at most one of them, so this is an upper bound per dispatch and is
     * reported separately from `sched` for that reason. */
    PpeStallResult sched_reg;
    u32 sched_reg_n;
#endif
} JitProfEntry;

#define JIT_PROF_BITS 18
#define JIT_PROF_SIZE (1u << JIT_PROF_BITS)
#define JIT_PROF_DUMP 4096u

static JitProfEntry *s_prof;        /* table + 1 shared overflow bucket */
static JitProfEntry *s_prof_sort;   /* preallocated: dump may run in a signal */
static u32           s_prof_used;
static JitProfEntry *s_prof_last;   /* entry of the block last dispatched */
static s32           s_prof_last_dc;
static int           s_prof_enabled;

static JitProfEntry *prof_entry(u32 pc)
{
    u32 i = ((pc >> 2) * 2654435761u) >> (32 - JIT_PROF_BITS);
    for (;;) {
        JitProfEntry *e = &s_prof[i];
        if (e->pc == pc)
            return e;
        if (e->pc == 0) {
            if (s_prof_used >= JIT_PROF_SIZE - (JIT_PROF_SIZE >> 2))
                return &s_prof[JIT_PROF_SIZE];   /* table full: overflow */
            e->pc = pc;
            s_prof_used++;
            return e;
        }
        i = (i + 1) & (JIT_PROF_SIZE - 1u);
    }
}

static int prof_cmp(const void *a, const void *b)
{
    const JitProfEntry *x = (const JitProfEntry *)a;
    const JitProfEntry *y = (const JitProfEntry *)b;
    if (x->real != y->real)
        return (x->real < y->real) ? 1 : -1;
    return 0;
}

#if !defined(__PS3__) && !defined(__lv2ppu__)
/* ------------------------------------------------------------------ */
/* JIT_SCHED=1: the static PPE stall model over the blocks a real boot   */
/* compiles, weighted by the blocks a real boot actually executes.       */
/*                                                                      */
/* Needs JIT_PROFILE=1 as well: the per-block execution weights live in  */
/* that table, and an unweighted static ranking over compiled code       */
/* answers the wrong question (most compiled blocks run once).           */
/* ------------------------------------------------------------------ */
static int       s_sched_enabled;
static PpeSite   s_sched_sites[PPE_SITE_SLOTS];
static u64       s_sched_blocks;

static void jit_sched_init(void)
{
    const char *p = getenv("JIT_SCHED");
    if (!p || !*p || *p == '0')
        return;
    s_sched_enabled = 1;
    fprintf(stderr, "[sched] static PPE stall model armed\n");
}

/* Analyse one freshly compiled block. Called from jit_compile_block. */
static void jit_sched_record(const JitBlock *b, const JitContext *c)
{
    JitProfEntry *e;
    unsigned r, k;
    if (!s_sched_enabled || !s_prof || !b || !b->hot_words)
        return;
    e = prof_entry(b->guest_pc);
    /* A pc can be recompiled (cache flush, MSR change). Model the latest
     * shape, exactly as hot_words/warm_words already do. */
    ppe_stall_analyse(b->code, b->hot_words, &e->sched, s_sched_sites);
    if (e->sched.loop_to > e->sched.loop_from)
        ppe_stall_analyse(b->code + e->sched.loop_from,
                          e->sched.loop_to - e->sched.loop_from,
                          &e->sched_loop, NULL);
    else
        e->sched_loop.cycles = e->sched_loop.words = 0;

    /* Trace formation promotes a deferred conditional branch to a REGION of
     * the same unit, emitted after hot_words by the cold-tail sweep. That
     * code is hot -- the taken side of an inlined branch -- so a model that
     * stops at hot_words no longer sees a growing share of what executes.
     * Summed here, and reported on its own line. */
    e->sched_reg.cycles = e->sched_reg.words = e->sched_reg.dual = 0;
    e->sched_reg.unknown = e->sched_reg.conds = 0;
    e->sched_reg.hinted = e->sched_reg.indirect = 0;
    e->sched_reg.mispredict = 0.0;
    for (k = 0; k < SC_COUNT; k++) e->sched_reg.stall[k] = 0;
    e->sched_reg_n = 0;
    for (r = 0; c && r < c->sched_span_count; r++) {
        PpeStallResult t;
        u32 from = c->sched_span[r].from, to = c->sched_span[r].to;
        if (to <= from || to > b->code_words)
            continue;
        ppe_stall_analyse(b->code + from, to - from, &t, s_sched_sites);
        e->sched_reg.cycles += t.cycles;
        e->sched_reg.words  += t.words;
        e->sched_reg.dual   += t.dual;
        e->sched_reg.unknown += t.unknown;
        e->sched_reg.conds  += t.conds;
        e->sched_reg.hinted += t.hinted;
        e->sched_reg.indirect += t.indirect;
        e->sched_reg.mispredict += t.mispredict;
        for (k = 0; k < SC_COUNT; k++) e->sched_reg.stall[k] += t.stall[k];
        e->sched_reg_n++;
    }
    e->sched_done = 1;
    s_sched_blocks++;
}

static int sched_site_cmp(const void *a, const void *b)
{
    const PpeSite *x = (const PpeSite *)a, *y = (const PpeSite *)b;
    if (x->cycles != y->cycles) return (x->cycles < y->cycles) ? 1 : -1;
    return 0;
}

static const char *sched_opname(u32 key)
{
    /* key = (opcd<<10 | xo). Only the encodings that actually show up in the
     * emitted corpus need names; anything else prints numerically. */
    u32 opcd = key >> 10, xo = key & 0x3FFu;
    switch (opcd) {
    case 7:  return "mulli";   case 8:  return "subfic";
    case 10: return "cmplwi";  case 11: return "cmpwi";
    case 12: return "addic";   case 13: return "addic.";
    case 14: return "addi";    case 15: return "addis";
    case 16: return "bc";      case 18: return "b/bl";
    case 20: return "rlwimi";  case 21: return "rlwinm";
    case 23: return "rlwnm";   case 24: return "ori";
    case 25: return "oris";    case 26: return "xori";
    case 27: return "xoris";   case 28: return "andi.";
    case 29: return "andis.";  case 30: return "rld*";
    case 32: return "lwz";     case 34: return "lbz";
    case 36: return "stw";     case 38: return "stb";
    case 40: return "lhz";     case 42: return "lha";
    case 44: return "sth";     case 48: return "lfs";
    case 50: return "lfd";     case 52: return "stfs";
    case 54: return "stfd";    case 58: return "ld";
    case 62: return "std";
    case 19:
        return (xo == 16) ? "blr" : (xo == 528) ? "bctr" : "cr-op";
    case 59: return "fp-single";
    case 4:  return "vmx";
    case 31:
        switch (xo) {
        case 444: return "or/mr";     case 28:  return "and";
        case 316: return "xor";       case 124: return "nor";
        case 24:  return "slw";       case 536: return "srw";
        case 792: return "sraw";      case 824: return "srawi";
        case 266: case 778: return "add";
        case 40:  case 552: return "subf";
        case 23:  return "lwzx";      case 151: return "stwx";
        case 87:  return "lbzx";      case 215: return "stbx";
        case 279: return "lhzx";      case 407: return "sthx";
        case 535: return "lfsx";      case 663: return "stfsx";
        case 599: return "lfdx";      case 727: return "stfdx";
        case 0:   return "cmpw";      case 32:  return "cmplw";
        case 19:  return "mfcr";      case 144: return "mtcrf";
        case 339: return "mfspr";     case 467: return "mtspr";
        case 235: return "mullw";     case 491: return "divw";
        case 26:  return "cntlzw";    case 922: return "extsh";
        case 954: return "extsb";     case 986: return "extsw";
        case 1014:return "dcbz";
        default:  return "op31";
        }
    default: return "op";
    }
}

static void jit_sched_dump(void)
{
    static volatile int done;
    u32 i;
    double w_disp[SC_COUNT], w_inst[SC_COUNT];
    double cyc_disp = 0, cyc_inst = 0, mis_disp = 0, mis_inst = 0;
    double words_disp = 0, words_inst = 0;
    double stall_disp = 0, stall_inst = 0;
    u64 unknown = 0, dual = 0, modelled_words = 0;
    double conds = 0, hinted = 0, indirect = 0;
    double reg_cyc = 0, reg_words = 0, reg_stall = 0;
    double w_reg[SC_COUNT];
    u64 reg_blocks = 0, reg_count = 0, reg_modelled_words = 0;
    unsigned c;

    if (!s_sched_enabled || !s_prof || done)
        return;
    done = 1;

    for (c = 0; c < SC_COUNT; c++) w_disp[c] = w_inst[c] = w_reg[c] = 0.0;

    for (i = 0; i <= JIT_PROF_SIZE; i++) {
        const JitProfEntry *e = &s_prof[i];
        double wd, wi;
        if (!e->pc || !e->sched_done || !e->sched.words)
            continue;
        /* Two weightings, exactly the ones jit_profile_dump already uses for
         * executed host words, so the two reports are commensurable.
         *   dispatch-weighted: one hot-path traversal per dispatch (exact for
         *     straight-line blocks, an underestimate inside retained loops)
         *   insts-weighted:    every attributed guest instruction cost its
         *     block's mean rate (overweights idle-skipped slices)
         * The truth is between them; both are computed identically on every
         * build being compared, which is what makes A/B valid. */
        /* Two weightings.
         *
         * `wd` (dispatch) charges one hot-path traversal per dispatch. Exact
         * for a straight-line block, and exact for the idle-skip block, whose
         * enormous attributed instruction count is *charged* virtual time,
         * not executed work.
         *
         * `wl` (loop-aware) adds, for a block whose hot path contains a back
         * edge, one loop-body traversal per iteration beyond the first: a
         * retained or warm self-loop runs its body thousands of times per
         * dispatch, and counting it once would hide exactly the code the
         * emulator spends most of its time in. Iterations come from the
         * downcount-attributed instruction count divided by the block's guest
         * length, which is what the profile already uses for executed words.
         *
         * The loop-aware figure is the primary one; the dispatch figure is
         * kept because it depends on nothing but the model. */
        wd = (double)e->execs;
        wi = wd;
        if (e->sched_loop.cycles && e->guest_insts) {
            double iters = (double)e->insts / (double)e->guest_insts;
            if (iters > wd) wi = iters;
        }
        for (c = 0; c < SC_COUNT; c++) {
            w_disp[c] += wd * (double)e->sched.stall[c];
            w_inst[c] += wd * (double)e->sched.stall[c] +
                         (wi - wd) * (double)e->sched_loop.stall[c];
        }
        cyc_disp += wd * (double)e->sched.cycles;
        cyc_inst += wd * (double)e->sched.cycles +
                    (wi - wd) * (double)e->sched_loop.cycles;
        mis_disp += wd * e->sched.mispredict;
        mis_inst += wd * e->sched.mispredict +
                    (wi - wd) * e->sched_loop.mispredict;
        words_disp += wd * (double)e->sched.words;
        words_inst += wd * (double)e->sched.words +
                      (wi - wd) * (double)e->sched_loop.words;
        unknown += e->sched.unknown;
        dual    += e->sched.dual;
        modelled_words += e->sched.words;
        conds    += wd * (double)e->sched.conds;
        hinted   += wd * (double)e->sched.hinted;
        indirect += wd * (double)e->sched.indirect;
        if (e->sched_reg.words) {
            reg_cyc   += wd * (double)e->sched_reg.cycles;
            reg_words += wd * (double)e->sched_reg.words;
            for (c = 0; c < SC_COUNT; c++)
                w_reg[c] += wd * (double)e->sched_reg.stall[c];
            reg_modelled_words += e->sched_reg.words;
            reg_count += e->sched_reg_n;
            reg_blocks++;
        }
    }
    for (c = 0; c < SC_COUNT; c++) reg_stall += w_reg[c];
    for (c = 0; c < SC_COUNT; c++) { stall_disp += w_disp[c]; stall_inst += w_inst[c]; }

    fprintf(stderr, "[sched] ==== static PPE stall model ====\n");
    fprintf(stderr, "[sched] blocks modelled=%llu words=%llu undecoded=%llu"
            " (%.4f%%)\n",
            (unsigned long long)s_sched_blocks,
            (unsigned long long)modelled_words,
            (unsigned long long)unknown,
            modelled_words ? 100.0 * (double)unknown / (double)modelled_words : 0.0);
    fprintf(stderr, "[sched] weighted: exec_words=%.0f issue_cycles=%.0f"
            "  stall_cycles=%.0f  stall_share=%.2f%%\n",
            words_disp, cyc_disp, stall_disp,
            cyc_disp > 0 ? 100.0 * stall_disp / cyc_disp : 0.0);
    fprintf(stderr, "[sched] weighted(loop-aware): exec_words=%.0f issue_cycles=%.0f"
            "  stall_cycles=%.0f  stall_share=%.2f%%\n",
            words_inst, cyc_inst, stall_inst,
            cyc_inst > 0 ? 100.0 * stall_inst / cyc_inst : 0.0);
    fprintf(stderr, "[sched] modelled IPC=%.3f  (words/cycle, dispatch-weighted)"
            "  dual-issued %llu of %llu compiled words (%.1f%%)\n",
            cyc_disp > 0 ? words_disp / cyc_disp : 0.0,
            (unsigned long long)dual, (unsigned long long)modelled_words,
            modelled_words ? 100.0 * (double)dual / (double)modelled_words : 0.0);
    fprintf(stderr, "[sched] ---- stall cycles by cause (dispatch-weighted) ----\n");
    {
        unsigned order[SC_COUNT], j, k2;
        for (j = 0; j < SC_COUNT; j++) order[j] = j;
        for (j = 0; j < SC_COUNT; j++)
            for (k2 = j + 1; k2 < SC_COUNT; k2++)
                if (w_disp[order[k2]] > w_disp[order[j]]) {
                    unsigned t = order[j]; order[j] = order[k2]; order[k2] = t;
                }
        for (j = 0; j < SC_COUNT; j++) {
            unsigned p = order[j];
            fprintf(stderr, "[sched]   %-12s %14.0f  %6.2f%% of stalls"
                    "  %6.2f%% of cycles   [loop-w %.0f]\n",
                    ppe_cause_name(p), w_disp[p],
                    stall_disp > 0 ? 100.0 * w_disp[p] / stall_disp : 0.0,
                    cyc_disp > 0 ? 100.0 * w_disp[p] / cyc_disp : 0.0,
                    w_inst[p]);
        }
    }
    fprintf(stderr, "[sched] branches on the modelled path (dispatch-weighted):"
            " conditional=%.0f of which hinted=%.0f (%.1f%%), indirect=%.0f\n",
            conds, hinted, conds > 0 ? 100.0 * hinted / conds : 0.0, indirect);
    fprintf(stderr, "[sched] MODELLED branch mispredict (assumption-driven,"
            " reported apart): %.0f cycles = %.2f%% of issue cycles"
            "  [insts-w %.0f]\n",
            mis_disp, cyc_disp > 0 ? 100.0 * mis_disp / cyc_disp : 0.0, mis_inst);
    fprintf(stderr, "[sched] headroom if every modelled stall vanished:"
            " dispatch-w %.3fx, loop-aware %.3fx\n",
            (cyc_disp - stall_disp) > 0 ? cyc_disp / (cyc_disp - stall_disp) : 0.0,
            (cyc_inst - stall_inst) > 0 ? cyc_inst / (cyc_inst - stall_inst) : 0.0);
    fprintf(stderr, "[sched] A/B KEY NUMBERS  issue_cycles: dispatch-w %.0f"
            "  loop-aware %.0f\n", cyc_disp, cyc_inst);

    /* Promoted regions, reported apart. A dispatch enters at most one of a
     * unit's regions, so weighting every region of a unit by that unit's
     * dispatch count is an upper bound, not a traversal -- but it is the same
     * upper bound on both sides of an A/B, which is what it is for. */
    fprintf(stderr, "[sched] ---- promoted regions (hot code past hot_words),"
            " dispatch-weighted upper bound ----\n");
    fprintf(stderr, "[sched]   units_with_regions=%llu regions=%llu"
            " compiled_words=%llu\n",
            (unsigned long long)reg_blocks, (unsigned long long)reg_count,
            (unsigned long long)reg_modelled_words);
    fprintf(stderr, "[sched]   exec_words=%.0f issue_cycles=%.0f"
            " stall_cycles=%.0f stall_share=%.2f%%\n",
            reg_words, reg_cyc, reg_stall,
            reg_cyc > 0 ? 100.0 * reg_stall / reg_cyc : 0.0);
    {
        unsigned order[SC_COUNT], j, k2;
        for (j = 0; j < SC_COUNT; j++) order[j] = j;
        for (j = 0; j < SC_COUNT; j++)
            for (k2 = j + 1; k2 < SC_COUNT; k2++)
                if (w_reg[order[k2]] > w_reg[order[j]]) {
                    unsigned t = order[j]; order[j] = order[k2]; order[k2] = t;
                }
        for (j = 0; j < SC_COUNT; j++) {
            unsigned p = order[j];
            if (w_reg[p] <= 0) break;
            fprintf(stderr, "[sched]   %-12s %14.0f  %6.2f%% of region stalls\n",
                    ppe_cause_name(p), w_reg[p],
                    reg_stall > 0 ? 100.0 * w_reg[p] / reg_stall : 0.0);
        }
    }
    fprintf(stderr, "[sched] A/B KEY NUMBERS(entry+regions) issue_cycles=%.0f"
            " stall_cycles=%.0f\n", cyc_disp + reg_cyc, stall_disp + reg_stall);

    /* Where, in the emitted code, the stalls actually are: the pair of
     * encodings whose dependence cost the cycles. This is the line that turns
     * "load-use is 40%" into an edit in the emitter. Unweighted (compile-time
     * frequency), which is enough to identify a *shape*. */
    {
        static PpeSite sorted[PPE_SITE_SLOTS];
        u32 n = 0;
        for (i = 0; i < PPE_SITE_SLOTS; i++)
            if (s_sched_sites[i].key && s_sched_sites[i].cycles)
                sorted[n++] = s_sched_sites[i];
        qsort(sorted, n, sizeof *sorted, sched_site_cmp);
        fprintf(stderr, "[sched] ---- top stalling (producer -> consumer) pairs,"
                " compile-time weight ----\n");
        for (i = 0; i < n && i < 24u; i++)
            fprintf(stderr, "[sched]   %-11s %-9s%-7s -> %-9s  %10llu cyc  %8llu sites\n",
                    ppe_cause_name(sorted[i].cause),
                    sched_opname((sorted[i].key >> 16) & 0xFFFFu),
                    sorted[i].pclass == PM_STATE ? "[state]"
                      : sorted[i].pclass == PM_GUEST ? "[guest]" : "",
                    sched_opname(sorted[i].key & 0xFFFFu),
                    (unsigned long long)sorted[i].cycles,
                    (unsigned long long)sorted[i].count);
    }
    fflush(stderr);
}
#endif /* host-only stall model */

/* Console-facing: top-N hottest blocks through a line callback, callable any
 * time, repeatable. The stderr dump below stays host-only. */
void jit_profile_reset(void)
{
    /* Zero the per-block counters so the next report is a clean window --
     * cumulative-since-boot numbers repeatedly mis-attributed menu/attract
     * work to in-race phases. Entries stay allocated (pc/guest_insts keep
     * their identity); only the accumulators clear. */
    u32 i;
    if (!s_prof) return;
    for (i = 0; i <= JIT_PROF_SIZE; i++) {
        s_prof[i].insts = 0;
        s_prof[i].execs = 0;
        s_prof[i].real  = 0;
    }
    s_prof_last = NULL;
}

void jit_profile_report(void (*out)(const char *), unsigned topn)
{
    char ln[128];
    /* Say this every single time. Profiling suppresses block linking (see
     * link_or_defer), so a profiled run is SLOWER than production by however
     * much linking was worth -- and the dispatch counters are correspondingly
     * inflated. Mistaking a profiled run for a production one cost a whole
     * investigation into "JIT throughput" that was really just this flag. */
    out("  jitprof: NOTE -- profiling DISABLES BLOCK LINKING; fps and");
    out("  jitprof:         dispatch_lookups from this run are NOT production.");
    u64 total = 0;
    u32 n = 0, i, cap;
    if (!s_prof || !s_prof_sort) { out("  jitprof: not armed"); return; }
    for (i = 0; i <= JIT_PROF_SIZE; i++)
        if (s_prof[i].pc && (s_prof[i].execs | s_prof[i].insts)) {
            s_prof_sort[n++] = s_prof[i];
            total += s_prof[i].real;
        }
    qsort(s_prof_sort, n, sizeof *s_prof_sort, prof_cmp);
    cap = n < topn ? n : topn;
    snprintf(ln, sizeof ln, "  jitprof: blocks=%u total_real=%llu",
             n, (unsigned long long)total);
    out(ln);
    for (i = 0; i < cap; i++) {
        snprintf(ln, sizeof ln,
                 "  jp pc=%08x real=%llu execs=%llu gi=%u virt=%llu",
                 s_prof_sort[i].pc,
                 (unsigned long long)s_prof_sort[i].real,
                 (unsigned long long)s_prof_sort[i].execs,
                 s_prof_sort[i].guest_insts,
                 (unsigned long long)s_prof_sort[i].insts);
        out(ln);
    }
}

static void jit_profile_dump(void)
{
    static volatile int dumped;
    u64 total = 0, shown = 0;
    u32 n = 0, i, cap;

    if (!s_prof || !s_prof_sort || dumped)
        return;
    dumped = 1;

    for (i = 0; i <= JIT_PROF_SIZE; i++)
        if (s_prof[i].pc && (s_prof[i].execs | s_prof[i].insts)) {
            s_prof_sort[n++] = s_prof[i];
            total += s_prof[i].insts;
        }
    qsort(s_prof_sort, n, sizeof *s_prof_sort, prof_cmp);
    cap = n < JIT_PROF_DUMP ? n : JIT_PROF_DUMP;

    fprintf(stderr, "[jitprof] total_insts=%llu blocks=%u dumped=%u\n",
            (unsigned long long)total, n, cap);
    for (i = 0; i < cap; i++) {
        fprintf(stderr, "[jitprof] pc=%08x insts=%llu execs=%llu ginsts=%u hw=%u ww=%u\n",
                s_prof_sort[i].pc,
                (unsigned long long)s_prof_sort[i].insts,
                (unsigned long long)s_prof_sort[i].execs,
                s_prof_sort[i].guest_insts,
                s_prof_sort[i].hot_words,
                s_prof_sort[i].warm_words);
        shown += s_prof_sort[i].insts;
    }
    fprintf(stderr, "[jitprof] dumped_insts=%llu of %llu (%.2f%%)\n",
            (unsigned long long)shown, (unsigned long long)total,
            total ? 100.0 * (double)shown / (double)total : 0.0);
    {
        /* Executed-host-word estimates. insts-weighted assumes each attributed
         * guest instruction cost its block's mean hot expansion (overweights
         * idle-skipped slices); dispatch-weighted counts one hot-path traversal
         * per dispatch (underweights retained loops). The truth sits between;
         * both are computed the same way on every build being compared. */
        double ew_insts = 0.0, ew_disp = 0.0;
        for (i = 0; i <= JIT_PROF_SIZE; i++) {
            const JitProfEntry *e = &s_prof[i];
            if (!e->pc || !e->hot_words)
                continue;
            if (e->guest_insts) {
                /* Warm self-loop blocks: each attributed iteration executes
                 * only the warm fast path; the prologue and the exits run
                 * once per dispatch. The formula reduces to the plain one
                 * when warm_words is 0, so builds stay comparable. */
                if (e->warm_words && e->warm_words <= e->hot_words)
                    ew_insts += ((double)e->insts / (double)e->guest_insts) *
                                    (double)e->warm_words +
                                (double)e->execs *
                                    (double)(e->hot_words - e->warm_words);
                else
                    ew_insts += (double)e->insts * (double)e->hot_words /
                                (double)e->guest_insts;
            }
            ew_disp += (double)e->execs * (double)e->hot_words;
        }
        fprintf(stderr, "[jitprof] est_exec_host_words insts_weighted=%.0f "
                "dispatch_weighted=%.0f\n", ew_insts, ew_disp);
    }
#if !defined(__PS3__) && !defined(__lv2ppu__)
    jit_sched_dump();
#endif
    fflush(stderr);
}

#if !defined(__PS3__) && !defined(__lv2ppu__)
static void jit_profile_on_signal(int sig)
{
    jit_profile_dump();
    _exit(128 + sig);
}
#endif

static void jit_profile_init(void)
{
    const char *p = getenv("JIT_PROFILE");
    if (s_prof_enabled || s_prof)
        return;
#if defined(__PS3__) || defined(__lv2ppu__)
    {   /* The console has no environment; a file is the switch. Costs the
         * block-linking suppression only when armed, and it is the only way
         * to see WHICH guest code eats a CPU-bound phase on hardware -- the
         * attract movie runs 24M guest instructions a frame and the phase
         * table cannot say where. */
        FILE *jf = fopen("/dev_hdd0/tmp/dolphin-jitprof.txt", "r");
        if (!jf) return;
        fclose(jf);
        p = "1";
    }
#endif
    if (!p || !*p || *p == '0')
        return;
    s_prof      = (JitProfEntry *)calloc(JIT_PROF_SIZE + 1, sizeof *s_prof);
    s_prof_sort = (JitProfEntry *)calloc(JIT_PROF_SIZE + 1, sizeof *s_prof_sort);
    if (!s_prof || !s_prof_sort) {
        free(s_prof);      s_prof = NULL;
        free(s_prof_sort); s_prof_sort = NULL;
        return;
    }
    s_prof[JIT_PROF_SIZE].pc = 0xFFFFFFFFu;   /* the shared overflow bucket */
    s_prof_enabled = 1;
#if !defined(__PS3__) && !defined(__lv2ppu__)
    atexit(jit_profile_dump);
    signal(SIGTERM, jit_profile_on_signal);
    signal(SIGINT,  jit_profile_on_signal);
#endif
    fprintf(stderr, "[jitprof] enabled: linking suppressed, counting dispatches\n");
}

/* JIT_HIST: dump the per-opcode emission histogram at process exit. The
 * histogram itself (g_jit_cost / g_jit_count, one bucket per guest primary
 * opcode) is maintained unconditionally by jit_attribute() in jit_compile.c;
 * this only adds the env-var-gated dump, so the qemu boot harness can report
 * where the emitted host words actually went for the code a real boot
 * compiles. Host-only, like JIT_PROFILE. */
extern u64 g_jit_cost[64];
extern u64 g_jit_count[64];
extern u64 g_jit_guards_kept, g_jit_guards_elided, g_jit_mmio_direct;
extern u64 g_jit_cold_mmio_words, g_jit_cold_mmio_count;
extern u64 g_jit_esc_sites, g_jit_esc_tramp_words;
extern u64 g_jit_cold_site[4];
extern u64 g_jit_cold_branch_words, g_jit_cold_branch_count;
extern u64 g_jit_inline_bail_words, g_jit_inline_bail_count;
extern u64 g_jit_regions;

#if !defined(__PS3__) && !defined(__lv2ppu__)
static void jit_hist_dump(void)
{
    unsigned i;
    u64 words = 0, insts = 0;
    for (i = 0; i < 64; i++) { words += g_jit_cost[i]; insts += g_jit_count[i]; }
    if (!insts)
        return;
    fprintf(stderr, "[jithist] guest=%llu attributed=%llu emitted=%llu "
            "expansion=%.2fx\n",
            (unsigned long long)insts, (unsigned long long)words,
            (unsigned long long)g_jit_stats.host_insts_emitted,
            jit_expansion_ratio());
    for (i = 0; i < 64; i++)
        if (g_jit_count[i])
            fprintf(stderr, "[jithist] op%02u count=%llu words=%llu each=%.2f\n",
                    i, (unsigned long long)g_jit_count[i],
                    (unsigned long long)g_jit_cost[i],
                    (double)g_jit_cost[i] / (double)g_jit_count[i]);
    fprintf(stderr, "[jithist] guards kept=%llu elided=%llu direct_mmio=%llu\n",
            (unsigned long long)g_jit_guards_kept,
            (unsigned long long)g_jit_guards_elided,
            (unsigned long long)g_jit_mmio_direct);
    fprintf(stderr, "[jithist] words hot=%llu cold=%llu cold_pct=%.2f "
            "blocks_with_cold=%llu\n",
            (unsigned long long)g_jit_hot_words,
            (unsigned long long)g_jit_cold_words,
            (g_jit_hot_words + g_jit_cold_words)
              ? 100.0 * (double)g_jit_cold_words /
                (double)(g_jit_hot_words + g_jit_cold_words) : 0.0,
            (unsigned long long)g_jit_blocks_with_cold);
    fprintf(stderr, "[jithist] cold mmio=%llu/%lluw branch=%llu/%lluw "
            "inline=%llu/%lluw\n",
            (unsigned long long)g_jit_cold_mmio_count,
            (unsigned long long)g_jit_cold_mmio_words,
            (unsigned long long)g_jit_cold_branch_count,
            (unsigned long long)g_jit_cold_branch_words,
            (unsigned long long)g_jit_inline_bail_count,
            (unsigned long long)g_jit_inline_bail_words);
    fprintf(stderr, "[jithist] regions_in_unit=%llu\n",
            (unsigned long long)g_jit_regions);
    fprintf(stderr, "[jithist] coldsite mmio=%llu fcmp=%llu gqr=%llu branch=%llu\n",
            (unsigned long long)g_jit_cold_site[0],
            (unsigned long long)g_jit_cold_site[1],
            (unsigned long long)g_jit_cold_site[2],
            (unsigned long long)g_jit_cold_site[3]);
    fprintf(stderr, "[jithist] esc sites=%llu tramp_words=%llu "
            "escapes_taken=%llu\n",
            (unsigned long long)g_jit_esc_sites,
            (unsigned long long)g_jit_esc_tramp_words,
            (unsigned long long)g_jit_force_interp_hits);
    fprintf(stderr, "[jithist] cache blocks=%llu flushes=%llu code_bytes=%llu\n",
            (unsigned long long)g_jit_stats.blocks_compiled,
            (unsigned long long)g_jit_stats.cache_flushes,
            (unsigned long long)g_jit_stats.code_bytes_used);
    {   static const char *const k_bname[10] = {
            "1", "2", "3-4", "5-8", "9-16", "17-32", "33-64", "65-128",
            "129-256", ">256" };
        u64 nb = 0, ni = 0;
        unsigned k;
        for (k = 0; k < 10; k++) { nb += g_jit_bsize[k]; ni += g_jit_bsize_insts[k]; }
        fprintf(stderr, "[jithist] unit size: blocks=%llu mean_guest_insts=%.2f\n",
                (unsigned long long)nb, nb ? (double)ni / (double)nb : 0.0);
        for (k = 0; k < 10; k++)
            if (g_jit_bsize[k])
                fprintf(stderr, "[jithist] unitsize %-8s blocks=%llu (%.1f%%) "
                        "guest_insts=%llu\n", k_bname[k],
                        (unsigned long long)g_jit_bsize[k],
                        nb ? 100.0 * (double)g_jit_bsize[k] / (double)nb : 0.0,
                        (unsigned long long)g_jit_bsize_insts[k]);
    }
#ifdef JIT_WORDPROF
    fprintf(stderr, "[jithist] EXEC host words=%llu\n",
            s_wp_state ? (unsigned long long)s_wp_state->jit_prof_words : 0ull);
#endif
    fflush(stderr);
}
#endif

static void jit_hist_init(void)
{
#if !defined(__PS3__) && !defined(__lv2ppu__)
    static int armed;
    const char *p = getenv("JIT_HIST");
    if (armed || !p || !*p || *p == '0')
        return;
    armed = 1;
    atexit(jit_hist_dump);
#endif
}

/* Declared in jit_compile.c. */
void jit_compile_into(JitContext *c, PPCState *s, u32 pc);

/* Implemented in jit_entry.S on PowerPC. */
#if JIT_CAN_EXECUTE
extern void jit_enter(PPCState *s, u8 *membase);
#endif

/* ------------------------------------------------------------------ */
/* Code buffer                                                          */
/* ------------------------------------------------------------------ */

#if defined(__PS3__) || defined(__lv2ppu__)
/* The JIT code cache, reserved inside the executable text segment.
 *
 * The PS3 refuses to execute code written at run time into heap, mapper, or
 * .bss memory -- retail firmware enforces W^X in userland, confirmed directly
 * by an on-console probe that ran from .text but faulted from every runtime
 * allocation. The one region that executes runtime-written code is the loadable
 * text segment, which the same probe also showed to be writable in practice.
 *
 * So the cache is a static array forced into .text. `used` keeps the linker
 * from discarding it; it is zero-filled in the image (a fixed cost in the
 * binary), which is the price of the only memory the console will run. A
 * cleaner NOBITS reservation would need a bespoke executable segment in the
 * linker script, which the toolchain's header layout would not accommodate
 * without overflowing the reserved program-header space -- a refinement for
 * later, not a correctness requirement. */
/* Sized from measurement, not convenience. At 8 MiB the cache held roughly
 * 12,800 compiled blocks and overflowed about once a second in-race; because
 * a flush discards the whole cache, all 12,800 were then recompiled from
 * cold, costing 52 ms per frame -- 23% of the frame -- inside the compiler.
 * The working set sat permanently just above the cache, so it never
 * converged. 24 MiB gives roughly three times the measured working set,
 * which takes the steady-state flush rate to zero.
 *
 * The cost is paid in the executable: this array is zero-filled in the image,
 * so the EBOOT grows by the same amount and the loader reads it all at
 * startup. That is the price of the only memory the console will execute
 * runtime-written code from. */
#define PS3_JITCACHE_BYTES (24u << 20)
__attribute__((section(".text"), aligned(4096), used))
static u8 s_ps3_jitcache[PS3_JITCACHE_BYTES];
#endif

static int code_buffer_alloc(size_t bytes)
{
#if defined(__PS3__) || defined(__lv2ppu__)
    if (bytes > sizeof s_ps3_jitcache) {
        LOG_ERROR(LOG_JIT, "code cache: asked for %u KiB, .text reserves %u KiB",
                  (unsigned)(bytes >> 10),
                  (unsigned)(sizeof s_ps3_jitcache >> 10));
        return -1;
    }
    s_code.code   = s_ps3_jitcache;
    s_code.handle = NULL;               /* nothing to free: it is in the image */
#else
    /* Only ask for PROT_EXEC where the code will actually run. macOS refuses
     * anonymous RWX mappings under the hardened runtime, and the workstation
     * build only ever compiles and disassembles blocks -- it cannot execute
     * PowerPC anyway. */
    int prot = PROT_READ | PROT_WRITE;
#if JIT_CAN_EXECUTE
    prot |= PROT_EXEC;
#endif
    void *p = mmap(NULL, bytes, prot, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED)
        return -1;
    s_code.code = (u8 *)p;
#endif
    s_code.size = bytes;
    s_code.used = 0;
    return 0;
}

static void code_buffer_free(void)
{
    if (!s_code.code)
        return;
#if defined(__PS3__) || defined(__lv2ppu__)
    /* The cache is a NOBITS region of the process image, not an allocation;
     * there is nothing to return to the system. */
#else
    munmap(s_code.code, s_code.size);
#endif
    s_code.code = NULL;
    s_code.size = s_code.used = 0;
}

/* ------------------------------------------------------------------ */
/* Shared escape tail                                                   */
/*                                                                      */
/* One copy, at the head of the code cache, of the nine words every       */
/* guarded access used to emit for itself. See the commentary in jit.h.   */
/* ------------------------------------------------------------------ */

/* ---- cold arena -------------------------------------------------------
 *
 * The cache is split in two. Hot code allocates upward from the bottom; the
 * MMIO bail-out tails allocate upward from `s_cold_base`. The point is
 * locality, not capacity: hot code from consecutive blocks ends up adjacent
 * instead of separated by ~130 words of tail that is branched over.
 *
 * The split fraction follows the measured mix (hot is ~36% of emitted words),
 * biased toward hot so the arena that fills first is the one whose size was
 * chosen deliberately. Off by default until measured on hardware -- see
 * g_jit_cold_split. */
int    g_jit_cold_split = 0;
static size_t s_cold_base;      /* offset of the cold arena */
static size_t s_cold_used;      /* bytes used within it     */

size_t jit_cold_used(void)  { return s_cold_used; }

static u32 *s_esc_stub;         /* the tail's address, or NULL */

void *jit_esc_stub(void) { return (void *)s_esc_stub; }

/* Emit the tail at the current head of the code cache. Called at jit_init
 * and again after every flush. Keeping it *inside* the cache is what lets a
 * one-word `bl` reach it from any site: `bl` spans +-32 MiB and the whole
 * cache is 8 MiB on the console.
 *
 * On entry LR points at the caller's `.long guest_pc`, and every register
 * the block was caching has already been written back by the site itself. */
static void esc_stub_emit(void)
{
    PPCEmitter e;
    size_t bytes;

    s_esc_stub = NULL;
    if (!s_code.code || s_code.size - s_code.used < 256)
        return;

    emit_init(&e, s_code.code + s_code.used, s_code.size - s_code.used);

    e_mflr(&e, H_SCRATCH0);
    e_lwz(&e, H_SCRATCH1, 0, H_SCRATCH0);            /* the site's guest pc */
    e_stw(&e, H_SCRATCH1, (s32)offsetof(PPCState, pc), H_STATE);
    e_li(&e, H_SCRATCH1, 1);
    e_stw(&e, H_SCRATCH1, (s32)offsetof(PPCState, force_interp), H_STATE);
    /* The site already charged the instructions it completed against the
     * pinned budget; only the memory copy the dispatcher reads is left. */
    e_stw(&e, H_DOWNCOUNT, (s32)offsetof(PPCState, downcount), H_STATE);
    e_mtctr(&e, H_DISPATCH);
    e_bctr(&e);

    if (e.overflow)
        return;
    bytes = emit_size(&e);
    s_esc_stub = (u32 *)(s_code.code + s_code.used);
    s_code.used += bytes;
    ppc_flush_icache(s_esc_stub, bytes);
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                            */
/* ------------------------------------------------------------------ */

int jit_init(size_t code_bytes)
{
    if (s_inited)
        return 0;

    memset(&g_jit_stats, 0, sizeof g_jit_stats);

    if (code_buffer_alloc(code_bytes) != 0) {
        LOG_ERROR(LOG_JIT, "could not allocate a %u MiB code cache",
                  (unsigned)(code_bytes >> 20));
        return -1;
    }

    /* One slot per 512 bytes of cache. Measured in-race, a compiled block
     * averages ~650 bytes, so the cache -- the thing whose size was chosen
     * deliberately -- runs out first and the table keeps roughly 25% spare
     * slots. The older 1-per-256 rule doubled a table that is ~150 bytes per
     * entry, which at a 24 MiB cache is 15 MiB of index for a capacity that
     * can never be reached. */
#if !defined(__PS3__) && !defined(__lv2ppu__)
    {   /* Bisection handle, same pattern as JIT_NOSCHED. */
        const char *cs = getenv("JIT_COLD_SPLIT");
        if (cs && *cs && *cs != '0') g_jit_cold_split = 1;
    }
#else
    {   /* On console the flag files are how experiments are armed, because
         * there is no environment to read. */
        FILE *f = fopen("/dev_hdd0/tmp/dolphin-coldsplit.txt", "rb");
        if (f) { g_jit_cold_split = 1; fclose(f); }
    }
#endif
    /* Hot gets 45%: measured emitted words are ~36% hot, so this leaves the
     * hot arena a little slack and lets the cold arena be the one that
     * usually runs out -- and a cold overflow is the cheaper of the two to
     * handle, because it only ever costs a flush. */
    s_cold_base = g_jit_cold_split ? (code_bytes * 45u) / 100u : code_bytes;
    s_cold_used = 0;

    s_block_capacity = (u32)(code_bytes / 512);
    s_blocks = (JitBlock *)calloc(s_block_capacity, sizeof *s_blocks);
    s_hash   = (JitBlock **)calloc(JIT_HASH_SIZE, sizeof *s_hash);
    s_pages  = (JitBlock **)calloc(JIT_PAGE_COUNT, sizeof *s_pages);

    if (!s_blocks || !s_hash || !s_pages) {
        jit_shutdown();
        return -1;
    }

    esc_stub_emit();

    s_inited = 1;
    jit_profile_init();
    jit_hist_init();
#if defined(__PS3__) || defined(__lv2ppu__)
    /* The same bisection handles, armed by file because the console has no
     * environment. They are not a debugging luxury: the recompiler is now
     * ~79% of the frame and 35 cycles per guest instruction, and the only
     * honest way to find out what any one of its features is worth is to turn
     * that feature off on the hardware and measure the frame. Guessing has
     * been wrong twice. */
    {   extern int g_jit_prefetch, g_jit_prefetch_dist;
        FILE *pf = fopen("/dev_hdd0/tmp/dolphin-prefetch.txt", "rb");
        FILE *pa = fopen("/dev_hdd0/tmp/dolphin-prefetch-all.txt", "rb");
        {   FILE *p3 = fopen("/dev_hdd0/tmp/dolphin-prefetch-probe.txt", "rb");
            if (p3) { fclose(p3); g_jit_prefetch = 3; }
        }
        if (pa) { fclose(pa); if (!pf && g_jit_prefetch != 3) g_jit_prefetch = 2; }
        if (pf) {
            int d = 0;
            g_jit_prefetch = pa ? 2 : 1;
            if (fscanf(pf, "%d", &d) == 1 && d >= 32 && d <= 4096)
                g_jit_prefetch_dist = d;
            fclose(pf);
            LOG_INFO(LOG_JIT, "software prefetch ON, %d bytes ahead",
                     g_jit_prefetch_dist);
        }
    }
    {   FILE *f = fopen("/dev_hdd0/tmp/dolphin-nosched.txt", "rb");
        if (f) { fclose(f); g_jit_sched_enable = 0;
                 LOG_INFO(LOG_JIT, "PPE instruction scheduling DISABLED (flag)"); }
        f = fopen("/dev_hdd0/tmp/dolphin-nosched-regions.txt", "rb");
        if (f) { fclose(f); g_jit_sched_regions = 0;
                 LOG_INFO(LOG_JIT, "region scheduling DISABLED (flag)"); }
    }
#endif
#if !defined(__PS3__) && !defined(__lv2ppu__)
    jit_sched_init();
    {   /* Bisection handle for the instruction scheduler. */
        {   extern int g_jit_prefetch, g_jit_prefetch_dist;
            const char *pf = getenv("JIT_PREFETCH");
            if (pf && *pf && *pf != '0') {
                g_jit_prefetch = 1;
                if (atoi(pf) >= 32) g_jit_prefetch_dist = atoi(pf);
            if (getenv("JIT_PREFETCH_ALL")) g_jit_prefetch = 2;
            if (getenv("JIT_PREFETCH_PROBE")) g_jit_prefetch = 3;
                fprintf(stderr, "[jit] software prefetch ON, %d bytes ahead\n",
                        g_jit_prefetch_dist);
            }
        }
        const char *ns = getenv("JIT_NOSCHED");
        if (ns && *ns && *ns != '0') {
            g_jit_sched_enable = 0;
            fprintf(stderr, "[sched] PPE instruction scheduling DISABLED\n");
        }
        ns = getenv("JIT_NOSCHED_REGIONS");
        if (ns && *ns && *ns != '0') {
            g_jit_sched_regions = 0;
            fprintf(stderr, "[sched] region scheduling DISABLED\n");
        }
    }
#endif
    LOG_INFO(LOG_JIT, "code cache %u MiB, %u block slots%s",
             (unsigned)(code_bytes >> 20), s_block_capacity,
             JIT_CAN_EXECUTE ? "" : " (compile-only: host is not PowerPC)");
    return 0;
}

void jit_shutdown(void)
{
    code_buffer_free();
    s_esc_stub = NULL;
    free(s_blocks); s_blocks = NULL;
    free(s_hash);   s_hash   = NULL;
    free(s_pages);  s_pages  = NULL;
    s_block_count = 0;
    s_inited = 0;
}

void jit_flush_all(void)
{
    if (!s_inited)
        return;
    memset(s_hash,  0, JIT_HASH_SIZE  * sizeof *s_hash);
    memset(s_pages, 0, JIT_PAGE_COUNT * sizeof *s_pages);
    s_block_count = 0;
    s_code.used = 0;
    s_unresolved_count = 0;
    /* The tail lives in the cache, so it went with it, and so did every site
     * that branched to it. Re-emit before any block is compiled. */
    esc_stub_emit();
    s_cold_used = 0;
    g_jit_stats.cache_flushes++;
    LOG_DEBUG(LOG_JIT, "code cache flushed");
}

/* ------------------------------------------------------------------ */
/* Lookup                                                               */
/* ------------------------------------------------------------------ */

/* Blocks are keyed on the machine state they were compiled under. HID2[PSE]
 * belongs in the key alongside the MSR bits: paired-single instructions are
 * compiled natively only when it is set, so reusing such a block with it clear
 * would silently skip the program exception hardware would raise.
 *
 * HID2[LSQE] belongs for the same reason and was missing. `compile_psq`
 * refuses every quantised load and store when it is clear:
 *
 *     if (!(c->state->hid2 & HID2_LSQE)) return 0;
 *
 * so a block first translated before the title enables paired-single
 * load/store bakes that refusal in -- and without the bit in the key, the same
 * block is then reused for the rest of the run once LSQE is set. Every psq_l
 * and psq_st in it keeps taking the interpreter fallback forever: a full GPR,
 * FPR and CR flush, a 64-bit call out and a reload, per access, for code the
 * recompiler is perfectly able to emit.
 *
 * Found by reading the host code emitted for a block that still contained a
 * `psq_st` fallback (0x80036f68) long after §18 taught the compiler to emit
 * quantised stores. LSQE flips once during startup, so the extra key bit costs
 * one extra translation of whatever was compiled before it. */
static u32 block_key(const PPCState *s)
{
    return (s->msr & MSR_JIT_KEY_MASK) |
           ((s->hid2 & HID2_PSE)  ? 0x80000000u : 0u) |
           ((s->hid2 & HID2_LSQE) ? 0x40000000u : 0u);
}

static u32 hash_pc(u32 pc, u32 msr_key)
{
    /* pc is 4-byte aligned, so the low two bits carry nothing. Mixing in the
     * MSR key keeps blocks compiled with the MMU on from being reused with it
     * off, which would be silently wrong. */
    u32 h = (pc >> 2) ^ (msr_key * 0x9E3779B9u);
    return (h ^ (h >> JIT_HASH_BITS)) & JIT_HASH_MASK;
}

static JitBlock *lookup(u32 pc, u32 msr_key)
{
    JitBlock *b = s_hash[hash_pc(pc, msr_key)];
    for (; b; b = b->hash_next)
        if (b->guest_pc == pc && b->msr_key == msr_key)
            return b;
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Block linking                                                        */
/*                                                                      */
/* A link site starts life as a branch to the following instruction (a no-op)   */
/* and is rewritten into a direct branch once its target has been compiled.     */
/* Targets are frequently compiled *after* the block that branches to them, so  */
/* sites that cannot be resolved immediately are parked here and patched when   */
/* their target appears.                                                        */
/* ------------------------------------------------------------------ */

static void patch_link(JitBlock *from, u32 idx, const JitBlock *to)
{
    u32 *site = from->code + from->links[idx].word_offset;
    s32  disp = (s32)((const u8 *)to->code - (const u8 *)site);

    /* The code cache is far smaller than the +-32 MiB reach of a `b`, so a
     * link can never be out of range. */
    *site = (18u << 26) | ((u32)disp & 0x03FFFFFCu);

    /* The PPE's instruction cache is not coherent with stores: without this the
     * fetch unit would keep executing the old no-op indefinitely. */
    ppc_flush_icache(site, 4);
}

static void link_or_defer(JitBlock *from, u32 idx)
{
    JitBlock *to;

    /* Profiling counts block executions in the dispatcher; a patched link
     * would let blocks jump to each other without being counted. Leaving the
     * site unpatched is the compiled code's initial state: slower, never
     * wrong. */
    if (UNLIKELY(s_prof_enabled))
        return;

    /* An AOT-owned pc must always come back to the dispatcher: a patched
     * direct branch would jump into the JIT-compiled block and the AOT
     * function would never be consulted (a `bl` to a registered function
     * is a statically known exit target, so without this every direct
     * call would bypass the table). Leaving the site unlinked is the
     * compiled code's initial state: slower, never wrong. With an empty
     * table this is one load and a predicted branch. */
    if (UNLIKELY(jit_aot_owns(from->links[idx].target_pc)))
        return;

    to = lookup(from->links[idx].target_pc, from->msr_key);
    g_jit_stats.links_emitted++;
    if (to) {
        patch_link(from, idx, to);
        g_jit_stats.links_resolved++;
        return;
    }
    if (s_unresolved_count < JIT_MAX_UNRESOLVED) {
        s_unresolved[s_unresolved_count].from  = from;
        s_unresolved[s_unresolved_count].index = (u8)idx;
        s_unresolved_count++;
    }
    /* If the table is full the site simply stays unlinked -- slower, never
     * wrong. Silently dropping *correctness* would be another matter. */
}

/* Called once a new block is registered: anything waiting on this address can
 * now be wired straight through. */
static void resolve_links_to(JitBlock *nb)
{
    u32 i = 0;

    if (UNLIKELY(s_prof_enabled))
        return;

    /* Same rule as link_or_defer, from the other side: sites can be
     * parked before the target pc is AOT-registered, and must not be
     * patched when its block is finally compiled (e.g. after an
     * estimator declined a dispatch there). */
    if (UNLIKELY(jit_aot_owns(nb->guest_pc)))
        return;

    while (i < s_unresolved_count) {
        UnresolvedLink *u = &s_unresolved[i];
        if (u->from->links[u->index].target_pc == nb->guest_pc &&
            u->from->msr_key == nb->msr_key) {
            patch_link(u->from, u->index, nb);
            g_jit_stats.links_resolved++;
            *u = s_unresolved[--s_unresolved_count];   /* swap with last */
            continue;
        }
        i++;
    }
}

/* The PPE time base, for the compile-cost counters in JitStats. Reading it
 * is a move-from-SPR; on anything that is not a 64-bit PowerPC there is no
 * such register, so the counters stay zero and say so. */
static u64 jit_tb(void)
{
#if defined(__powerpc64__) || defined(__PPC64__)
    u64 v;
    __asm__ __volatile__ ("mftb %0" : "=r"(v));
    return v;
#else
    return 0;
#endif
}

JitBlock *jit_compile_block(PPCState *s, u32 pc)
{
    JitContext ctx;
    JitBlock  *b;
    size_t     avail;
    u32        msr_key = block_key(s);
    u64        t_start = jit_tb();

    if (s_block_count >= s_block_capacity ||
        s_code.used + 4096 > s_cold_base ||
        (g_jit_cold_split && s_cold_base + s_cold_used + 4096 > s_code.size)) {
        jit_flush_all();
        if (s_code.used + 4096 > s_cold_base) {
            LOG_ERROR(LOG_JIT, "code cache too small for a single block");
            return NULL;
        }
    }

#if JIT_BLOCK_ALIGN
    /* Align every unit entry to a 32-byte boundary.
     *
     * The PPE fetches instructions in aligned groups and issues in pairs; a
     * unit whose first instruction lands in the middle of a group wastes
     * part of the first fetch, and the first fetch of a unit is never
     * covered by anything else -- the dispatcher or a patched link has just
     * branched here. The padding is *between* units, never inside one, so it
     * costs zero executed instructions.
     *
     * OFF BY DEFAULT, and the reason is trace formation, not the fetch
     * argument. `jit_code_pressure()` is `s_code.used` over the cache size,
     * and `trace_have_room()` gates every trace-formation decision on it --
     * whether a `b` is followed, whether a `bc` is inlined, whether a
     * deferred branch is promoted to a region. Padding inflates `used`, so
     * it does not merely reorder instructions: it changes WHICH guest code
     * is compiled into which unit, and where the pressure gate trips. See
     * docs/ARCHITECTURE.md, "In-order scheduling". */
    s_code.used = (s_code.used + 31u) & ~(size_t)31u;
    if (s_code.used + 4096 > s_code.size) {
        jit_flush_all();
        s_code.used = (s_code.used + 31u) & ~(size_t)31u;
    }
#endif

    avail = s_cold_base - s_code.used;
    emit_init(&ctx.e, s_code.code + s_code.used, avail);
    /* The cold arena, when split. Otherwise `ce` aliases `e` so every emit
     * site can write through it unconditionally and the two layouts share one
     * code path -- the difference is where the words land, nothing else. */
    ctx.cold_split = g_jit_cold_split;
    if (ctx.cold_split)
        emit_init(&ctx.ce, s_code.code + s_cold_base + s_cold_used,
                  s_code.size - s_cold_base - s_cold_used);

    /* Try loop register retention first. If a disqualifying condition turns up
     * mid-compile (a fallback, register-file exhaustion), retain_aborted is set
     * and the block is recompiled the ordinary way onto the same cache slot --
     * so the worst case is a wasted first compile, never wrong code. */
    ctx.want_retain = 1;
    ctx.want_warm   = 0;
    jit_compile_into(&ctx, s, pc);
    if (ctx.retain_aborted) {
        emit_init(&ctx.e, s_code.code + s_code.used, avail);
        if (ctx.cold_split)
            emit_init(&ctx.ce, s_code.code + s_cold_base + s_cold_used,
                      s_code.size - s_cold_base - s_cold_used);
        ctx.want_retain = 0;
        jit_compile_into(&ctx, s, pc);
    }

    /* The probe pass found a general self-loop (a conditional terminator
     * branching to this block's own start) that retention had refused:
     * recompile with a warm prologue so the back edge keeps the observed
     * register set live (see jit_compile.c, "Warm self-loop continuity").
     * Any surprise -- pin exhaustion, overflow -- falls back to one more
     * plain compile, the known-correct path. */
    if (!ctx.failed && !ctx.e.overflow && ctx.warm_candidate) {
        emit_init(&ctx.e, s_code.code + s_code.used, avail);
        if (ctx.cold_split)
            emit_init(&ctx.ce, s_code.code + s_cold_base + s_cold_used,
                      s_code.size - s_cold_base - s_cold_used);
        ctx.want_retain = 0;
        ctx.want_warm   = 1;
        jit_compile_into(&ctx, s, pc);
        if (ctx.retain_aborted || ctx.failed || ctx.e.overflow) {
            emit_init(&ctx.e, s_code.code + s_code.used, avail);
            if (ctx.cold_split)
                emit_init(&ctx.ce, s_code.code + s_cold_base + s_cold_used,
                          s_code.size - s_cold_base - s_cold_used);
            ctx.want_retain = 0;
            ctx.want_warm   = 0;
            jit_compile_into(&ctx, s, pc);
        }
    }

    if (ctx.failed || ctx.e.overflow ||
        (ctx.cold_split && ctx.ce.overflow)) {
        LOG_WARN(LOG_JIT, "block at %08x did not fit; flushing cache", pc);
        jit_flush_all();
        return NULL;
    }
    /* Claim the cold words this block wrote. Done here, after the block is
     * known good, so an abandoned compile attempt leaves the arena untouched
     * -- the same rule the hot cursor already follows. */
    if (ctx.cold_split)
        s_cold_used += emit_size(&ctx.ce);

    b = &s_blocks[s_block_count++];
    b->guest_pc    = pc;
    b->guest_end   = ctx.pc;
    b->msr_key     = msr_key;
    b->code        = (u32 *)(s_code.code + s_code.used);
    b->code_words  = (u32)(emit_size(&ctx.e) / 4);
    /* If no bail-out was deferred the whole block is hot. */
    b->hot_words   = ctx.hot_words ? ctx.hot_words : b->code_words;
    /* Nonzero only when the warm back edge was actually emitted. */
    b->warm_words  = ctx.warm_words;
    b->guest_insts = ctx.trace_insts;
    b->hits        = 0;
    {   /* Size distribution bucket: 1, 2, 3-4, 5-8, ... , >256. */
        u32 gi = ctx.trace_insts, k = 0;
        while (k < 9 && gi > (1u << k))
            k++;
        g_jit_bsize[k]++;
        g_jit_bsize_insts[k] += gi;
    }
    g_jit_hot_words  += b->hot_words;
    g_jit_cold_words += b->code_words - b->hot_words;
    if (b->code_words != b->hot_words) g_jit_blocks_with_cold++;
    b->link_count  = ctx.link_count;
    memcpy(b->links, ctx.link, sizeof b->links);

    s_code.used += emit_size(&ctx.e);

    /* The PPE has no coherent instruction cache: freshly written code is
     * invisible to the fetch unit until the lines are flushed from the data
     * cache and invalidated in the instruction cache. Omitting this is the
     * classic JIT bug that surfaces as executing stale bytes. */
    ppc_flush_icache(b->code, b->code_words * 4);

    {
        u32 h = hash_pc(pc, msr_key);
        b->hash_next = s_hash[h];
        s_hash[h] = b;
    }
    {
        u32 page = mem_fold(pc) >> JIT_PAGE_SHIFT;
        if (page < JIT_PAGE_COUNT) {
            b->page_next = s_pages[page];
            s_pages[page] = b;
        }
    }

    /* Wire this block's exits to their targets where possible, then satisfy
     * anything that was waiting on this block's address. */
    {
        u32 i;
        for (i = 0; i < b->link_count; i++)
            link_or_defer(b, i);
        resolve_links_to(b);
    }

    g_jit_stats.blocks_compiled++;
    g_jit_stats.guest_insts_compiled += ctx.trace_insts;
    g_jit_stats.host_insts_emitted   += b->code_words;
    g_jit_stats.fallback_insts       += ctx.fallbacks;
    g_jit_stats.code_bytes_used       = s_code.used;

    {
        u64 dt = jit_tb() - t_start;
        g_jit_stats.compile_ticks += dt;
        if (dt > g_jit_stats.compile_ticks_max)
            g_jit_stats.compile_ticks_max = dt;
    }

    LOG_TRACE(LOG_JIT, "block %08x..%08x: %u guest -> %u host (%u fallback)",
              pc, ctx.pc, ctx.trace_insts, b->code_words, ctx.fallbacks);
#if !defined(__PS3__) && !defined(__lv2ppu__)
    jit_sched_record(b, &ctx);
#endif
    return b;
}

unsigned jit_code_pressure(void)
{
    if (!s_inited || !s_code.size)
        return 100u;
    return (unsigned)((s_code.used * 100u) / s_code.size);
}

int jit_block_compiled(const PPCState *s, u32 pc)
{
    return s_inited && lookup(pc, block_key(s)) != NULL;
}

/* PC ranges the JIT must refuse, forcing those blocks through the
 * interpreter. File-armed ("dolphin-jitskip.txt": "start end" hex pairs, one
 * per line): the bisection tool that finds WHICH block the JIT miscompiles on
 * hardware, by narrowing the skipped range until the symptom moves. */
static struct { u32 lo, hi; } s_jit_skip[8];
static int s_jit_skip_n = -1;

static int jit_skip_pc(u32 pc)
{
    int i;
    if (s_jit_skip_n < 0) {
        FILE *f = fopen("/dev_hdd0/tmp/dolphin-jitskip.txt", "r");
        s_jit_skip_n = 0;
        if (f) {
            unsigned a2, b2;
            while (s_jit_skip_n < 8 &&
                   fscanf(f, "%x %x", &a2, &b2) == 2) {
                s_jit_skip[s_jit_skip_n].lo = a2;
                s_jit_skip[s_jit_skip_n].hi = b2;
                s_jit_skip_n++;
            }
            fclose(f);
            LOG_INFO(LOG_JIT, "jit: %d skip range(s) armed", s_jit_skip_n);
        }
    }
    for (i = 0; i < s_jit_skip_n; i++)
        if (pc >= s_jit_skip[i].lo && pc < s_jit_skip[i].hi)
            return 1;
    return 0;
}

JitBlock *jit_get_block(PPCState *s, u32 pc)
{
    u32 msr_key = block_key(s);
    JitBlock *b;

    g_jit_stats.dispatch_lookups++;
    if (jit_skip_pc(pc))
        return NULL;

    b = lookup(pc, msr_key);
    if (b) {
        b->hits++;
        return b;
    }
#ifdef JIT_AOT_TRACE
    {
        extern void jit_trace_compile_begin(void);
        extern void jit_trace_compile_end(void);
        JitBlock *nb;
        jit_trace_compile_begin();
        nb = jit_compile_block(s, pc);
        jit_trace_compile_end();
        return nb;
    }
#else
    return jit_compile_block(s, pc);
#endif
}

/* ------------------------------------------------------------------ */
/* Invalidation                                                         */
/*                                                                      */
/* Titles do modify code: overlays are loaded over one another, and the IPL and */
/* some engines patch branches at runtime. Getting this wrong produces a hang   */
/* that looks nothing like its cause, so the page granularity is deliberately   */
/* coarse and conservative -- dropping too many blocks costs recompilation,     */
/* while dropping too few costs correctness.                                    */
/* ------------------------------------------------------------------ */

void jit_invalidate_range(u32 addr, u32 len)
{
    if (!s_inited || len == 0)
        return;

    /* Blocks are linked to each other with direct branches, so a block that is
     * merely unhooked from the lookup table can still be *jumped into* by a
     * neighbour that was linked to it. Unpicking those back-references needs a
     * reverse edge list per block; until that exists, invalidation drops the
     * whole cache.
     *
     * This is the deliberately conservative choice: too many blocks discarded
     * costs recompilation, too few costs correctness, and self-modifying code
     * is rare enough in practice that the trade is worth taking until it shows
     * up in a profile. `cache_flushes` in JitStats is what will say when.
     */
    (void)addr;
    jit_flush_all();
}

/* ------------------------------------------------------------------ */
/* AOT dispatch table                                                   */
/*                                                                      */
/* A handful of statically recompiled guest functions (tools/rec),       */
/* consulted by pc before block lookup. Open-addressed and tiny: it      */
/* holds well under 32 entries, so 64 slots capped at three-quarters     */
/* full keeps every probe short and guarantees a miss terminates at an   */
/* empty slot. The table is deliberately independent of the block cache: */
/* jit_flush_all and jit_shutdown do not touch it, because the native    */
/* functions it points at are part of this binary, not the code buffer.  */
/* ------------------------------------------------------------------ */

typedef struct {
    u32         pc;         /* guest entry address (0 = empty slot) */
    JitAotFn    fn;
    JitAotEstFn est;        /* optional slice-cap estimator (may be NULL) */
    u32         key_mask;   /* block_key bits this fn is sensitive to */
    u32         key_want;   /* required value of those bits           */
    u64         hits;       /* dispatches into fn (instrumentation)  */
    u64         declined;   /* estimator refusals (instrumentation)  */
} JitAotEntry;

static JitAotEntry s_aot[JIT_AOT_SLOTS];
static u32         s_aot_count;     /* stored entries                     */
static u32         s_aot_enabled;   /* jit_aot_enable_all has been called */
/* The one word the dispatcher reads: nonzero iff enabled AND non-empty,
 * so the no-AOT path costs a single load and a predicted branch. */
static u32         s_aot_active;

/* Guest instructions an AOT function executed past the end of its slice.
 * A JIT block overshoots the downcount by at most one block, and
 * timing_advance clamps the overshoot away -- a loss too small to
 * observe. A whole function can overshoot by orders of magnitude more,
 * and letting timing_advance clamp that would hand the guest large
 * amounts of free work, visibly skewing its clock against a non-AOT
 * run. So the overshoot is carried here and charged against the
 * following slice(s) before any code runs: the work was real, and this
 * is when the base JIT would have executed it. */
static s32         s_aot_debt;

/* The machine state an AOT function is valid under, expressed as a
 * (mask, want) pair over block_key bits -- the same bits the block hash
 * keys on: MSR IR/DR/PR/FP plus the HID2[PSE] bit at 0x80000000.
 *
 * The default (jit_aot_register) is the strict full key: translation
 * on, supervisor, FP available, paired singles enabled. That is what an
 * FP/paired-single function needs -- entering one with MSR[FP] clear
 * would skip the FP-unavailable trap the interpreter would take, and
 * with it the OS's lazy FP context switching.
 *
 * Integer-only functions are gated more loosely via
 * jit_aot_register_key: MKWii runs FP disabled on most threads (lazy
 * per-thread FP), so demanding MSR[FP] there would turn the table off
 * exactly where it is hottest. An integer function's semantics do not
 * depend on FP or PSE, so masking those bits out stays bit-exact. */
#define JIT_AOT_KEY_MASK ((MSR_IR | MSR_DR | MSR_PR | MSR_FP) | 0x80000000u)
#define JIT_AOT_KEY      ((MSR_IR | MSR_DR | MSR_FP) | 0x80000000u)

static u32 aot_slot(u32 pc)
{
    /* pc is 4-byte aligned; same multiplicative mix as prof_entry. */
    return ((pc >> 2) * 2654435761u) >> 26;   /* 32 - log2(JIT_AOT_SLOTS) */
}

static JitAotEntry *aot_find(u32 pc)
{
    u32 i = aot_slot(pc);
    u32 probes;

    for (probes = 0; probes < JIT_AOT_SLOTS; probes++) {
        JitAotEntry *e = &s_aot[i];
        if (e->pc == pc)
            return e;
        if (e->pc == 0)
            return NULL;
        i = (i + 1) & (JIT_AOT_SLOTS - 1u);
    }
    return NULL;    /* unreachable: registration keeps empty slots */
}

/* Whether the AOT table has an entry at pc -- registered, enabled or
 * not. Linking keys refusal on registration rather than on the enable
 * flag so a stored entry can never be linked past between registration
 * and enable_all. Key gating is deliberately ignored: refusing a link
 * the dispatcher would not have taken is slower, never wrong. */
static int jit_aot_owns(u32 pc)
{
    return s_aot_count != 0 && aot_find(pc) != NULL;
}

/* The compiler's form of the same query (jit_compile.c must not follow
 * an unconditional branch into an owned pc during superblock formation:
 * a followed branch never reaches the dispatcher at all, which is even
 * more thorough a bypass than a patched link). */
int jit_aot_owns_pc(u32 pc)
{
    return jit_aot_owns(pc);
}

void jit_aot_register_est(u32 guest_pc, JitAotFn fn, u32 key_mask, u32 key_want,
                          JitAotEstFn est)
{
    u32 i = aot_slot(guest_pc);
    u32 probes;

    if (guest_pc == 0 || (guest_pc & 3u) != 0 || fn == NULL) {
        LOG_WARN(LOG_JIT, "AOT: rejecting bogus registration pc=%08x fn=%p",
                 guest_pc, (void *)fn);
        return;
    }

    for (probes = 0; probes < JIT_AOT_SLOTS; probes++) {
        JitAotEntry *e = &s_aot[i];
        if (e->pc == guest_pc) {            /* re-registration replaces */
            e->fn       = fn;
            e->est      = est;
            e->key_mask = key_mask;
            e->key_want = key_want;
            return;
        }
        if (e->pc == 0) {
            if (s_aot_count >= JIT_AOT_MAX_ENTRIES)
                break;                      /* keep the table sparse */
            e->pc       = guest_pc;
            e->fn       = fn;
            e->est      = est;
            e->key_mask = key_mask;
            e->key_want = key_want;
            e->hits     = 0;
            e->declined = 0;
            s_aot_count++;
            s_aot_active = s_aot_enabled ? s_aot_count : 0;
            /* Blocks compiled while the table was live may hold direct
             * links to this pc (jit_aot_owns said no at the time); they
             * would bypass the new entry forever. Rebuild them. */
            if (s_aot_enabled)
                jit_flush_all();
            return;
        }
        i = (i + 1) & (JIT_AOT_SLOTS - 1u);
    }
    /* Dropping a registration is slower, never wrong: the JIT simply
     * keeps compiling that function itself. */
    LOG_WARN(LOG_JIT, "AOT table full: dropping %08x", guest_pc);
}

void jit_aot_register_key(u32 guest_pc, JitAotFn fn, u32 key_mask, u32 key_want)
{
    jit_aot_register_est(guest_pc, fn, key_mask, key_want, NULL);
}

void jit_aot_register(u32 guest_pc, JitAotFn fn)
{
    jit_aot_register_key(guest_pc, fn, JIT_AOT_KEY_MASK, JIT_AOT_KEY);
}

void jit_aot_enable_all(void)
{
    s_aot_enabled = 1;
    s_aot_active  = s_aot_count;
    /* Blocks compiled before this point were linked under the "no AOT"
     * rule: a direct branch to a now-registered pc jumps straight into
     * the successor block and never consults the dispatcher. Flushing
     * rebuilds everything under jit_aot_owns link refusal, which is
     * what makes direct `bl`s to registered functions actually reach
     * the table. Costs one cold recompile, never correctness. */
    if (s_aot_count != 0)
        jit_flush_all();
    LOG_INFO(LOG_JIT, "AOT enabled: %u function(s)", s_aot_count);
}

void jit_aot_disable(void)
{
    s_aot_enabled = 0;
    s_aot_active  = 0;
    /* Symmetric with jit_aot_enable_all: blocks compiled while AOT was live
     * were linked under the AOT ownership rule, and some branch straight into
     * a recompiled body. Without a flush, "disabled" would still execute AOT
     * code and an A/B against the interpreter path would be measuring
     * nothing. */
    if (s_aot_count != 0)
        jit_flush_all();
    LOG_INFO(LOG_JIT, "AOT disabled: %u function(s) parked", s_aot_count);
}

JitAotFn jit_aot_lookup(u32 guest_pc)
{
    JitAotEntry *e;
    if (s_aot_active == 0)
        return NULL;
    e = aot_find(guest_pc);
    return e ? e->fn : NULL;
}

u64 jit_aot_hit_count(u32 guest_pc)
{
    const JitAotEntry *e = aot_find(guest_pc);
    return e ? e->hits : 0;
}

u64 jit_aot_declined_count(u32 guest_pc)
{
    const JitAotEntry *e = aot_find(guest_pc);
    return e ? e->declined : 0;
}

s32 jit_aot_debt_pending(void)
{
    return s_aot_debt;
}

/* ------------------------------------------------------------------ */
/* Dispatch                                                             */
/* ------------------------------------------------------------------ */

/* jit_entry.S hardcodes this offset; fail the build rather than mis-read the
 * downcount if PPCState is ever reordered. */
DOL_STATIC_ASSERT(offsetof(PPCState, downcount) == 0x0A8, jit_entry_downcount);

/* Called from the assembly dispatch loop. Returns the host code address of the
 * next block, or NULL to leave the loop. The termination checks live here
 * rather than in assembly so that jit_entry.S needs almost no knowledge of
 * PPCState's layout. */
void *jit_dispatch_c(PPCState *s)
{
    /* A loop, not straight-line code: an AOT hit executes a whole guest
     * function right here in C and leaves s->pc at the guest return target,
     * after which dispatch re-evaluates from the top -- the new pc may be
     * another AOT hit (a chain of returns through recompiled callers), a
     * compiled block, or code that still needs compiling. Looping instead of
     * recursing keeps a long AOT chain off the host stack. */
    static u32 s_last_disp_pc;
    for (;;) {
        JitBlock *b;

        if (UNLIKELY(s->pc == 0x80500000u)) {
            static unsigned n5;
            if (s_last_disp_pc >= 0x80300000u && s_last_disp_pc < 0x80340000u)
                ;
            else if (n5 < 6) {
                LOG_WARN(LOG_JIT, "dispatch 80500000! prev=%08x lr=%08x sp=%08x "
                         "r3=%08x r31=%08x",
                         s_last_disp_pc, s->lr, s->gpr[1], s->gpr[3],
                         s->gpr[31]);
                n5++;
            }
        }
        if (UNLIKELY(s->pc == 0)) {
            static unsigned n0;
            /* The boot difftest/realtest suites end every case with a branch
             * to 0 by design (prev block 0x8030xxxx-0x8033xxxx); don't let
             * that noise consume the log budget for the real thing. */
            if (s_last_disp_pc >= 0x80300000u && s_last_disp_pc < 0x80340000u)
                ;
            else if (n0 < 8)
                LOG_WARN(LOG_JIT, "dispatch pc=0! prev block pc=%08x lr=%08x "
                         "sp=%08x r3=%08x r13=%08x",
                         s_last_disp_pc, s->lr, s->gpr[1], s->gpr[3],
                         s->gpr[13]);
            n0++;
        }
        s_last_disp_pc = s->pc;

#ifdef JIT_TRACE_BLOCKS
        {
            extern void jit_trace_hook(const PPCState *);
            jit_trace_hook(s);
        }
#endif

        /* Attribute the guest instructions executed since the previous dispatch
         * (the downcount delta) to the block dispatched then. The delta is exact:
         * every exit path, retained-loop back edge and idle skip charges the
         * downcount for precisely the instructions it completed. */
        if (UNLIKELY(s_prof_enabled) && s_prof_last) {
            /* exit_slack is budget a forced exit destroyed, not instructions
             * executed -- timing_advance refunds it, so the profile must too. */
            s32 d = s_prof_last_dc - s->downcount - s->exit_slack;
            if (d > 0)
                s_prof_last->insts += (u64)(u32)d;
                /* Real work retired by this dispatch: an idle-skipped block
                 * runs its body once and then zeroes the budget, so cap the
                 * attribution at the block's own length. */
                {
                    u64 gi = s_prof_last->guest_insts;
                    u64 dd = (u64)(u32)d;
                    s_prof_last->real += (gi && dd > gi) ? gi : dd;
                }
            s_prof_last = NULL;
        }

        /* Pay down AOT overshoot from a previous slice before running
         * anything: these cycles were already executed ahead of the clock. */
        if (UNLIKELY(s_aot_debt > 0) && s->downcount > 0) {
            s32 d = (s->downcount < s_aot_debt) ? s->downcount : s_aot_debt;
            s->downcount -= d;
            s_aot_debt   -= d;
        }

        if (s->downcount <= 0 || s->exit_requested)
            return NULL;

        /* Compiled code asked for one interpreter step (an MMIO access it could
         * not perform itself). Returning NULL drops back to jit_run, which runs
         * that single instruction and re-enters. */
        if (s->force_interp)
            return NULL;

        /* AOT: a statically recompiled function registered for this pc runs
         * here, natively, and dispatch continues at the guest return target.
         * With an empty table this is one load and one predicted branch. The
         * key compare replicates block keying: an AOT function is only valid
         * under the machine state it was recompiled for, exactly as a block is
         * only found under the key it was compiled with. */
        if (s_aot_active != 0) {
            JitAotEntry *ae = aot_find(s->pc);
            if (ae && (block_key(s) & ae->key_mask) == ae->key_want) {
                u32 entry_pc = s->pc;
                u32 n;

                /* Slice cap. An AOT function cannot be suspended
                 * mid-body, so a function with a registered estimator
                 * runs only when its conservative cost fits in the
                 * remaining downcount (downcount > 0 was established
                 * above, so the cast is safe). Otherwise this one
                 * invocation falls through to the ordinary JIT path
                 * below, which executes the same guest code and yields
                 * at block boundaries -- always correct. With a
                 * conservative (>= actual) estimate, an accepted
                 * function also finishes inside the slice, so it never
                 * parks debt and never displaces the once-per-slice
                 * interrupt delivery relative to the base JIT. */
                if (UNLIKELY(ae->est != NULL) &&
                    ae->est(s) > (u32)s->downcount) {
                    ae->declined++;
                    goto aot_declined;
                }

                ae->hits++;
#ifdef JIT_AOT_TRACE
                {
                    extern void jit_aot_trace_begin(u32 pc, s32 dc);
                    jit_aot_trace_begin(entry_pc, s->downcount);
                }
#endif
                n = ae->fn(s);
#ifdef JIT_AOT_TRACE
                {
                    extern void jit_aot_trace_end(u32 pc, u32 n);
                    jit_aot_trace_end(entry_pc, n);
                }
#endif
                s->downcount -= (s32)n;
                /* Whole-function overshoot becomes debt for the next
                 * slice rather than clamped-away free work (s_aot_debt
                 * above). Zeroing the downcount here and carrying the
                 * remainder keeps timing_advance's accounting exact. */
                if (s->downcount < 0) {
                    s_aot_debt  += -s->downcount;
                    s->downcount = 0;
                }
                if (UNLIKELY(s_prof_enabled)) {
                    /* Keep the profile's total truthful: attribute the whole
                     * native execution to the function's entry pc. */
                    JitProfEntry *e = prof_entry(entry_pc);
                    e->execs++;
                    e->insts += n;
                }
                continue;
            }
        }
aot_declined:

        /* Hybrid bridge stint: linked native functions (and through them the
         * HLE overrides) take precedence over compiling guest DOL bytes. */
        {   extern int g_wc_bridge_depth;
            extern int wc_native_dispatch(PPCState *);
            if (UNLIKELY(g_wc_bridge_depth > 0) && wc_native_dispatch(s))
                continue;
        }

        b = jit_get_block(s, s->pc);
        if (b && UNLIKELY(s_prof_enabled)) {
            JitProfEntry *e = prof_entry(b->guest_pc);
            e->execs++;
            e->guest_insts = b->guest_insts;
            e->hot_words   = b->hot_words;
            e->warm_words  = b->warm_words;
            s_prof_last    = e;
            s_prof_last_dc = s->downcount;
        }
        return b ? (void *)b->code : NULL;
    }
}

void jit_run(PPCState *s)
{
#if JIT_CAN_EXECUTE
    /* Compiled code addresses guest memory as an offset from a pinned arena
     * base. Without the arena there is no such base, so the only correct thing
     * to do is interpret. */
    if (!g_mem.fastmem_ok) {
        interp_run(s);
        return;
    }

    /* Asynchronous exceptions are delivered ONCE, on entry -- not at every
     * block boundary inside the slice. Devices only change their interrupt
     * lines between slices (timing_advance), so per-boundary checks add no
     * responsiveness -- what they add is *amplification of real races*: MKWii's
     * StaticR.rel swaps the OS current-context pointers (0xD4 effective, then
     * 0xC0 physical) with interrupts enabled, a two-instruction window that
     * real hardware essentially never hits. A block boundary between those two
     * stores turned that race into a deterministic jump through a
     * half-switched context (srr0=0) on every boot. Delivering at the slice
     * edge keeps the same architectural behaviour -- pending lines stay
     * latched, MSR[EE] still gates -- with hardware-like probability. */
    if (UNLIKELY(s->exceptions))
        ppc_deliver_exception(s);

#ifdef JIT_WORDPROF
    s_wp_state = s;
#endif

    while (s->downcount > 0 && !s->exit_requested) {
        jit_enter(s, mem_base());

        /* A device access the compiled code deliberately declined: run exactly
         * that instruction in the interpreter, which dispatches to the device
         * model, then resume compiled execution. */
        if (s->force_interp) {
            g_jit_force_interp_hits++;
            s->force_interp = 0;
            interp_step(s);
            continue;
        }

        /* jit_enter also returns when the downcount expires, an exit is
         * requested, or a block could not be compiled. Only the last needs
         * handling: interpret one instruction so progress is guaranteed and we
         * cannot spin re-attempting the same failed compilation. */
        if (s->downcount > 0 && !s->exit_requested && !jit_get_block(s, s->pc))
            interp_step(s);
    }
#else
    /* The host is not a PowerPC, so emitted code cannot run here. Compiling is
     * still exercised by the verification tools; execution falls back to the
     * interpreter so the rest of the emulator can be developed and tested. */
    interp_run(s);
#endif
}

/* Map a host code address back to the guest block that emitted it.
 *
 * Used only by the rescue thread's `ctx` command: when the emulator wedges
 * inside jit_run, sysDbgReadPPUThreadContext yields a host pc, and the only
 * way to turn that into something meaningful is a linear walk of the block
 * table. Cost does not matter -- the caller runs once, on a thread that is
 * asking why the other one stopped. */
int jit_block_at_host(void *host_pc, u32 *guest_pc, u32 *guest_end, u32 *word)
{
    u32 *p = (u32 *)host_pc;
    u32 i;

    for (i = 0; i < s_block_count; i++) {
        JitBlock *b = &s_blocks[i];
        if (!b->code || !b->code_words)
            continue;
        if (p >= b->code && p < b->code + b->code_words) {
            if (guest_pc)  *guest_pc  = b->guest_pc;
            if (guest_end) *guest_end = b->guest_end;
            if (word)      *word      = (u32)(p - b->code);
            return 1;
        }
    }
    return 0;
}
