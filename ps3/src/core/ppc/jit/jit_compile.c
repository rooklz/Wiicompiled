/* jit_compile.c — the Gekko -> Cell PPE block compiler.
 *
 * ============================ THE 32-IN-64 RULE ============================
 *
 * Guest GPRs are 32-bit; host GPRs are 64-bit. The obvious approach is to keep
 * every guest value zero-extended and re-truncate after each operation, but
 * that costs an extra instruction on nearly every arithmetic op and would put
 * the expansion ratio around 2.0.
 *
 * It is unnecessary. On a 64-bit PowerPC the "word" instructions the guest uses
 * are *defined* on bits 32:63 and ignore the upper half:
 *
 *   add/and/or/xor/mullw     operate on the low word; upper bits are junk we
 *                            never read back (stw stores the low word).
 *   cmpw/cmplw               compare bits 32:63 only -- correct regardless.
 *   rlwinm/slw/srw/cntlzw    are defined on the low word and *produce* a clean
 *                            zero-extended result, so they launder values.
 *   lwz                      zero-extends, so values enter clean.
 *   stw                      stores the low word, so they leave clean.
 *
 * So the invariant is deliberately weak: **only the low 32 bits of a cached
 * guest register are meaningful.** Most guest instructions compile 1:1.
 *
 * The exceptions are instructions whose result depends on bit 32 crossing into
 * bit 63 -- the carry-generating group (addc/adde/subfc/...) and anything doing
 * a 64-bit compare. Those explicitly clean their inputs first. They are a small
 * fraction of the instruction mix, so paying 2-3 instructions there and 1
 * everywhere else is much better than paying 2 everywhere.
 *
 * ============================== MEMORY =====================================
 *
 * A guest access folds the effective address with one `rlwinm` and then uses a
 * single indexed instruction against the pinned arena base (ARCHITECTURE.md
 * §3.2). `rlwinm` conveniently *also* truncates to 32 bits, so the fold
 * re-establishes a clean address regardless of what junk the EA arithmetic left
 * in the upper half.
 *
 * MMIO cannot be trapped on this platform. lv2's exception handler is read-only
 * -- it cannot set a register or advance the program counter -- so the standard
 * trap-and-emulate approach is unavailable (docs/ARCHITECTURE.md §3.2.1).
 * Device addresses are therefore recognized by an explicit guard in the emitted
 * code, hoisted onto the base pointer so RAM code pays for it once per base and
 * nothing per access. On a match the block bails out and the interpreter
 * executes that one instruction.
 *
 * ============================= COMPLETENESS ================================
 *
 * Anything without a native path becomes a call to the interpreter handler for
 * that opcode. The JIT is therefore complete and correct from the first block;
 * native paths are an optimization layered on top, not a prerequisite.
 */
#include "jit.h"
#include "../interp/interp.h"
#include "ppe_sched.h"
#include "../../mem/memmap.h"
#include "../../core_timing.h"
#include "../../../common/log.h"

#include <string.h>

/* Feature gate. Compile-time, so the shipped recompiler carries no runtime
 * test: -DJIT_TRACE=0 rebuilds it with the pre-trace unit-formation rules and
 * is how the before/after in this file was measured with byte-identical
 * instrumentation on both sides. */
#ifndef JIT_TRACE
#define JIT_TRACE 1
#endif


/* g_jit_stats is defined once, in jit.c; jit_compile.c only updates it. Having
 * a second definition here was tolerated by the workstation's clang but is a
 * genuine ODR violation that the big-endian validation compiler (GCC with
 * -fno-common) correctly rejects. */

/* PPC64 ELFv1 -- which is what lv2 uses -- represents a function pointer as a
 * descriptor {code, toc, env}, not as a code address. JIT'd code must branch to
 * the *code* word. Our generated blocks never touch r2, so the module's TOC
 * stays valid across a helper call and only the code address is needed. */
static void *fn_code_ptr(const void *fn)
{
#if defined(_CALL_ELF) && _CALL_ELF == 2
    return (void *)fn;
#else
    return ((void *const *)fn)[0];
#endif
}


/* Where the host instructions actually go.
 *
 * Expansion is an average, and an average hides the thing worth fixing: 6.11x
 * overall could be every instruction costing six, or most costing two and a few
 * costing sixty. Those call for completely different work, so attribute each
 * guest instruction to its primary opcode and record what it cost to emit.
 * Compile-time only -- nothing here runs on the console. */
u64 g_jit_cost[64];     /* host words emitted, by primary opcode  */
u64 g_jit_count[64];    /* guest instructions seen, by opcode     */

/* Guard placement accounting, for the same audit: how many MMIO guards were
 * emitted, how many a compiler could prove unnecessary, and how many accesses
 * were proven to *be* MMIO and compiled straight to the escape. On a compiler
 * without address provenance only `kept` ever moves. Compile-time only. */
u64 g_jit_cold_site[4];   /* 0=mmio 1=fcmp 2=gqr 3=branch */
u64 g_jit_esc_sites;          /* escapes that took the shared tail        */
u64 g_jit_esc_tramp_words;    /* words those escapes cost, in total       */
u64 g_jit_cold_mmio_words;    /* words in deferred MMIO escapes           */
u64 g_jit_cold_mmio_count;
static PPCFixup e_bc_cold(JitContext *c, u32 bo, u32 bi);  /* defined below */

u64 g_jit_cold_branch_words;  /* words in deferred taken-branch exits      */
u64 g_jit_cold_branch_count;
u64 g_jit_inline_bail_words;  /* words in inline (slot-exhausted) bail-outs */
u64 g_jit_inline_bail_count;
u64 g_jit_regions;            /* deferred branches compiled in-unit       */
/* Post-pass instruction scheduling for the in-order PPE (ppe_sched.h).
 * On by default. The kill switch exists so a divergence can be bisected
 * between "the scheduler reordered something wrongly" and "an emitter
 * change is wrong" without rebuilding; jit_init clears it from the
 * environment on the host, and nothing clears it on the console. */
int g_jit_sched_enable = 1;
/* Software prefetch: off until measured. See emit_prefetch. */
int g_jit_prefetch = 0;
int g_jit_prefetch_dist = 128;
/* Second, finer switch: schedule the promoted REGIONS as well as the unit's
 * entry region. Regions are hot code emitted in the cold tail, so they are
 * not covered by c->hot_words; separating the two lets a divergence be
 * bisected to one side or the other without rebuilding. */
int g_jit_sched_regions = 1;
static PpeSchedWork s_sched_work;

u64 g_jit_guards_kept;
u64 g_jit_guards_elided;
u64 g_jit_mmio_direct;

void jit_attribute(u32 op, u32 words)
{
    unsigned primary = (op >> 26) & 63u;
    g_jit_cost[primary]  += words;
    g_jit_count[primary] += 1;
}

/* ------------------------------------------------------------------ */
/* Exact executed-word accounting (JIT_WORDPROF measurement builds)      */
/*                                                                       */
/* "How many host words does a boot actually execute?" stopped being      */
/* answerable from block metadata once compilation units grew internal    */
/* control flow: hot_words then counts code that a given traversal        */
/* branches over, so both of the profiler's estimates (insts-weighted and */
/* dispatch-weighted) drift with block SHAPE, which is exactly the        */
/* variable under test. Estimating the effect of trace formation with a   */
/* metric that trace formation biases is worthless.                       */
/*                                                                        */
/* So the emitted code counts itself, exactly. The hot region of a unit is */
/* laid out as one straight line: every conditional the compiler inlines   */
/* branches OUT of it, to the cold tail. A traversal is therefore a prefix */
/* of that line plus, if it left early, one cold tail. Both lengths are    */
/* known at compile time, so three host words at each point where control  */
/* leaves a straight-line run record the exact count.                      */
/*                                                                         */
/* Present only when JIT_WORDPROF is defined; the shipped recompiler emits  */
/* not one instruction of it.                                              */
/* ------------------------------------------------------------------ */

#ifdef JIT_WORDPROF
static u32 wp_here(JitContext *c)
{
    return (u32)(emit_mark(&c->e) - c->e.base);
}

/* Start a fresh straight-line run here, having already executed `already`
 * words to reach it. */
static void wp_begin(JitContext *c, s32 already)
{
    c->wp_mark  = wp_here(c);
    c->wp_carry = already;
}

/* Add exactly `n` words, without disturbing the run accounting. Used where
 * the length is known but the emit offsets are not a straight subtraction --
 * a loop back edge, whose next traversal re-enters above this point. */
static void wp_add(JitContext *c, s32 n)
{
    if (n <= 0)
        return;
    e_ld(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, jit_prof_words), H_STATE);
    e_addi(&c->e, H_SCRATCH0, H_SCRATCH0, n);
    e_std(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, jit_prof_words), H_STATE);
}

/* Account everything executed since the run began, plus `trailing` words that
 * follow this checkpoint and are certain to run on the path being counted.
 * If the path continues past them to another checkpoint, the negative carry
 * left behind cancels the double count. H_SCRATCH0 must be dead here, which
 * it is at every exit: the register caches have been flushed and any address
 * in flight has been consumed. */
static void wp_point(JitContext *c, s32 trailing)
{
    wp_add(c, c->wp_carry + (s32)(wp_here(c) - c->wp_mark) + trailing);
    c->wp_mark  = wp_here(c);
    c->wp_carry = -trailing;
}

/* Words executed to reach the cold slot just recorded, carried with it so its
 * tail can account the whole path in one place. */
static void wp_cold(JitContext *c, unsigned n)
{
    c->cold[n].wp = c->wp_carry + (s32)(wp_here(c) - c->wp_mark);
}

/* Resume accounting for the path that reaches cold slot `n`. */
static void wp_resume(JitContext *c, unsigned n)
{
    wp_begin(c, c->cold[n].wp);
}

/* A loop back edge. One iteration -- warm entry through the branch -- is
 * accounted here, because the next traversal re-enters ABOVE this point and
 * a subtraction of emit offsets would not describe it. Call immediately
 * before the two-word budget test that precedes the branch. */
static void wp_backedge(JitContext *c, u32 entry_off)
{
    wp_add(c, (s32)(wp_here(c) - entry_off) + 2);
}

/* Code the straight-line path branches over (an inline bail-out, the
 * exception check's exit arm) must not be charged to it, and any checkpoint
 * inside it must not move the run's origin. */
#define WP_SKIP_BEGIN(c)  s32 wp_sc = (c)->wp_carry +                       \
                              (s32)(wp_here(c) - (c)->wp_mark)
#define WP_SKIP_END(c)    do { (c)->wp_carry = wp_sc;                         \
                               (c)->wp_mark  = wp_here(c); } while (0)
#else
#define wp_here(c)          ((u32)0)
#define wp_begin(c, a)      ((void)0)
#define wp_add(c, n)        ((void)0)
#define wp_point(c, t)      ((void)0)
#define wp_cold(c, n)       ((void)0)
#define wp_resume(c, n)     ((void)0)
#define wp_backedge(c, o)   ((void)0)
#define WP_SKIP_BEGIN(c)    ((void)0)
#define WP_SKIP_END(c)      ((void)0)
#endif

/* ------------------------------------------------------------------ */
/* Constant address provenance                                          */
/*                                                                      */
/* A per-block abstract value per guest GPR (jit.h: ProvSlot). The point is    */
/* guard elision: 38.6% of all emitted words were cold MMIO bail-out tails    */
/* that can never run, plus a hot 3-word guard per base, and the addresses     */
/* behind most of them are compile-time constants built by lis/addi/ori        */
/* chains. Tracking those chains proves, per access, whether the guard can     */
/* fire at all.                                                                */
/*                                                                             */
/* Soundness rules, all enforced below:                                        */
/*   - block entry state is all-unknown (nothing crosses block boundaries);    */
/*   - every write path through the register cache (gpr_dest/gpr_write) kills  */
/*     the destination's provenance; the few writes that bypass the cache      */
/*     helpers (rlwimi, psq update) kill explicitly;                           */
/*   - an interpreter fallback may change any register, so it wipes the lot;   */
/*   - the modelled transfer functions (prov_transfer, further down) are       */
/*     computed from the pre-instruction state and applied only after the      */
/*     native path for that instruction was actually emitted;                  */
/*   - the only in-block back edge is loop retention's warm entry, which sits  */
/*     before the first guest instruction where the lattice is still all-     */
/*     unknown, so everything derived inside the body is re-established by     */
/*     the body itself on every iteration.                                     */
/* ------------------------------------------------------------------ */

static void prov_reset(JitContext *c)
{
    memset(c->prov, 0, sizeof c->prov);         /* PROV_UNKNOWN */
}

static void prov_kill(JitContext *c, u32 g)
{
    c->prov[g].kind = PROV_UNKNOWN;
}

static void prov_set(JitContext *c, u32 g, u8 kind, u32 value, u32 span)
{
    c->prov[g].kind  = kind;
    c->prov[g].value = (kind == PROV_UPPER) ? (value & 0xFFFF0000u) : value;
    c->prov[g].span  = (kind == PROV_RANGE) ? span : 0;
}

/* The value interval a register is known to lie in, or 0 if unknown. The
 * interval is contiguous in 64 bits; the register's actual value is any
 * member reduced mod 2^32, which does not change bits 24..31 -- all the
 * guard classification below ever reads. */
static int prov_bounds(const JitContext *c, u32 g, u64 *lo, u64 *hi)
{
    switch (c->prov[g].kind) {
    case PROV_CONST:
        *lo = *hi = c->prov[g].value;
        return 1;
    case PROV_UPPER:
        *lo = c->prov[g].value;
        *hi = (u64)c->prov[g].value | 0xFFFFu;
        return 1;
    case PROV_RANGE:
        *lo = c->prov[g].value;
        *hi = (u64)c->prov[g].value + c->prov[g].span;
        return 1;
    default:
        return 0;
    }
}

/* What the guard would decide for every address in [lo, hi] (64-bit, so a sum
 * of two 32-bit intervals stays contiguous; dropping bit 32 afterwards does
 * not change bits 24..31, which are all the guard reads).
 *
 * The emitted guard is `rlwinm 8,26,30; cmplwi FOLD_MMIO>>24`, i.e. it fires
 * iff ((addr >> 24) & 0x3E) == 0x0C -- a pure function of addr >> 25. An
 * interval shorter than 2^25 spans at most two consecutive values of
 * addr >> 25, so if both endpoints agree the whole interval does. */
#define EA_UNKNOWN 0            /* keep the guard          */
#define EA_SAFE    1            /* guard can never fire    */
#define EA_MMIO    2            /* guard would always fire */

static int ea_is_mmio_word(u64 x)
{
    return ((x >> 24) & 0x3Eu) == (u64)(FOLD_MMIO >> 24);
}

static int ea_classify(u64 lo, u64 hi)
{
    if (hi - lo >= 0x02000000ull)
        return EA_UNKNOWN;
    if (ea_is_mmio_word(lo) != ea_is_mmio_word(hi))
        return EA_UNKNOWN;
    return ea_is_mmio_word(lo) ? EA_MMIO : EA_SAFE;
}

/* Classification of a single base register (the d-form guard tests the raw
 * base, hoisted, with the displacement never part of the test). */
static int prov_classify_reg(const JitContext *c, u32 g)
{
    u64 lo, hi;
    if (!prov_bounds(c, g, &lo, &hi))
        return EA_UNKNOWN;
    return ea_classify(lo, hi);
}

/* Classification of ra+rb (the indexed-form guard tests the full sum). */
static int prov_classify_sum(const JitContext *c, u32 ga, u32 gb)
{
    u64 alo, ahi, blo, bhi;
    if (!prov_bounds(c, ga, &alo, &ahi) || !prov_bounds(c, gb, &blo, &bhi))
        return EA_UNKNOWN;
    return ea_classify(alo + blo, ahi + bhi);
}

/* ------------------------------------------------------------------ */
/* Register cache                                                       */
/* ------------------------------------------------------------------ */

static void addr_base_drop(JitContext *c, u32 g);
static void addr_base_invalidate_all(JitContext *c);
static void rc_reset(JitContext *c)
{
    unsigned i;
    for (i = 0; i < 32; i++) {
        c->gpr[i].host = -1;
        c->gpr[i].dirty = 0;
        c->gpr[i].loaded = 0;
        c->gpr[i].lru = 0;
        c->host_taken[i] = 0;
    }
    c->lru_clock = 0;
    addr_base_invalidate_all(c);
}

/* Which guest register currently owns a host register, or -1. */
static int host_owner(JitContext *c, int host)
{
    unsigned g;
    for (g = 0; g < 32; g++)
        if (c->gpr[g].host == host)
            return (int)g;
    return -1;
}

static void gpr_writeback(JitContext *c, u32 g)
{
    if (c->gpr[g].host >= 0 && c->gpr[g].dirty) {
        e_stw(&c->e, (u32)c->gpr[g].host,
              (s32)(offsetof(PPCState, gpr) + 4 * g), H_STATE);
        c->gpr[g].dirty = 0;
    }
}

/* Free a host register, writing back whatever guest register owned it. The
 * evicted guest register's value no longer lives in any host register, so it
 * must be marked not-loaded: a later use allocates a fresh host slot and has to
 * reload from memory. Forgetting this leaves the stale `loaded` flag set and
 * the reload is skipped -- reintroducing exactly the uninitialized-register
 * bug, but only on registers unlucky enough to be evicted. */
static void evict_host(JitContext *c, int host)
{
    int g = host_owner(c, host);
    if (g >= 0) {
        gpr_writeback(c, (u32)g);
        c->gpr[g].host = -1;
        c->gpr[g].loaded = 0;
    }
    c->host_taken[host] = 0;
}

static int alloc_host(JitContext *c)
{
    int r, victim = H_GPRCACHE_FIRST;
    u32 oldest = 0xFFFFFFFFu;

    for (r = H_GPRCACHE_FIRST; r <= H_GPRCACHE_LAST; r++) {
        if (!c->host_taken[r]) {
            c->host_taken[r] = 1;
            return r;
        }
    }

    /* All fourteen are in use, so something must go. The victim must be chosen
     * by *least recent use*, not by a fixed rule: a block that touches more
     * than fourteen guest registers usually keeps hammering a few of them (a
     * loop cursor, a base pointer) while cycling through the rest. Always
     * evicting the same slot lands on the hot register and produces a
     * store/reload on nearly every subsequent access -- measured at roughly
     * twice the emitted code for a 21-instruction block before this was fixed.
     */
    victim = -1;
    for (r = H_GPRCACHE_FIRST; r <= H_GPRCACHE_LAST; r++) {
        int g;
        u32 age;
        if ((c->pin_gpr | c->retain_pin) & (1u << r))
            continue;               /* in use now, or pinned for the whole loop */
        g   = host_owner(c, r);
        age = (g >= 0) ? c->gpr[g].lru : 0u;
        if (victim < 0 || age < oldest) {
            oldest = age;
            victim = r;
        }
    }

    if (victim < 0) {
        /* Everything is pinned -- only reachable under loop retention when the
         * body needs more live temporaries than the reserve left free. Abort
         * retention; jit_compile_block recompiles the block without it, which
         * is the known-correct path. Return a scratch so the aborted compile
         * does not also dereference -1; its output is discarded. */
        c->retain_aborted = 1;
        return H_GPRCACHE_FIRST;
    }

    evict_host(c, victim);
    c->host_taken[victim] = 1;
    return victim;
}

/* Ensure guest register g occupies a host register that holds its *current
 * value*, emitting a load if the slot is fresh or was allocated for a
 * write that has not happened yet. This is the read path, and it is where the
 * validity of the value is guaranteed.
 *
 * The subtlety this fixes: a destination is frequently allocated (by gpr_def)
 * before the same instruction reads it -- `addi r3, r3, 1` writes and reads r3
 * -- and a write-only allocation leaves the host register holding garbage. A
 * later read of that same guest register must load it, which the `loaded` flag
 * makes observable. Missing this produced a value silently short by exactly the
 * register's initial contents, and was caught only once the JIT actually
 * executed on real hardware. */
static int gpr_read(JitContext *c, u32 g)
{
    c->gpr[g].lru = ++c->lru_clock;

    if (c->gpr[g].host < 0)
        c->gpr[g].host = (s8)alloc_host(c);

    if (!c->gpr[g].loaded) {
        e_lwz(&c->e, (u32)c->gpr[g].host,
              (s32)(offsetof(PPCState, gpr) + 4 * g), H_STATE);
        c->gpr[g].loaded = 1;
    }
    c->pin_gpr |= 1u << (unsigned)c->gpr[g].host;
    return c->gpr[g].host;
}

/* Destination allocation that is correct regardless of whether the same
 * instruction also reads the register. It ensures the current value is present
 * (a read-modify destination such as `addi r3, r3, 1` therefore gets its real
 * contents) and then marks the slot dirty.
 *
 * The cost is one load for a genuinely pure definition (`li`, or an `add` whose
 * destination it does not read). That is a known, bounded inefficiency and a
 * later optimization can skip it by reading sources before defining
 * destinations; it is emphatically not worth trading for the write-only
 * shortcut that caused the uninitialized-register bug the hardware differential
 * test exposed. Correctness first. */
/* Drop any address-base slot folded from guest register g: the fold is of that
 * register's old value and is stale the moment it is written. */
static void addr_base_drop(JitContext *c, u32 g)
{
    int slot;
    for (slot = 0; slot < JIT_ADDR_BASE_SLOTS; slot++)
        if (c->addr_base_guest[slot] == (s8)g)
            c->addr_base_guest[slot] = -1;
}

static void addr_base_invalidate_all(JitContext *c)
{
    int slot;
    for (slot = 0; slot < JIT_ADDR_BASE_SLOTS; slot++) {
        c->addr_base_guest[slot] = -1;
        c->addr_base_lru[slot] = 0;
    }
    c->addr_base_clock = 0;
}

static int gpr_write(JitContext *c, u32 g)
{
    int h = gpr_read(c, g);     /* ensures loaded */
    c->gpr[g].dirty = 1;
    addr_base_drop(c, g);
    prov_kill(c, g);
    return h;
}

/* Allocate a host register for a guest register that is about to be written in
 * full, *without* first loading the old value from state.
 *
 * gpr_write is gpr_read plus a dirty flag, which is what a read-modify-write
 * form needs but is wasteful for the far commoner destination-only case: the
 * emitted code read a value from PPCState and then immediately overwrote it,
 *
 *      lwz 22, 36(15)          <- dead: nothing ever reads this
 *      lwz 22, 8(12)           <- the actual guest load
 *
 * one wasted load for every destination register not already cached.
 *
 * Precondition: every source register of the instruction has already been read.
 * That makes this safe even when the destination *is* one of the sources --
 * the register is then already loaded, and marking it loaded is a no-op. Call
 * it before reading the sources and it would suppress a load that is needed. */
static int gpr_dest(JitContext *c, u32 g)
{
    c->gpr[g].lru = ++c->lru_clock;

    if (c->gpr[g].host < 0)
        c->gpr[g].host = (s8)alloc_host(c);

    c->gpr[g].loaded = 1;       /* the instruction itself supplies the value */
    c->gpr[g].dirty  = 1;
    addr_base_drop(c, g);
    prov_kill(c, g);
    c->pin_gpr |= 1u << (unsigned)c->gpr[g].host;
    return c->gpr[g].host;
}

/* ------------------------------------------------------------------ */
/* Carry cache                                                          */
/*                                                                      */
/* Same arrangement as the GPR cache, for the same reason: a long-arithmetic    */
/* chain is a run of adde/subfe whose only shared state is the carry, and       */
/* spilling it between each pair puts two memory accesses in a dependency chain */
/* that would otherwise be pure register arithmetic.                            */
/* ------------------------------------------------------------------ */

static int carry_reg(JitContext *c)
{
    if (!c->carry_loaded) {
        e_lwz(&c->e, H_CARRY, (s32)offsetof(PPCState, xer_ca), H_STATE);
        c->carry_loaded = 1;
    }
    return H_CARRY;
}

/* The new carry is already in H_CARRY; just record that memory is now stale. */
static void carry_produced(JitContext *c)
{
    c->carry_loaded = 1;
    c->carry_dirty  = 1;
}

static void carry_flush(JitContext *c)
{
    if (!c->carry_dirty)
        return;
    e_stw(&c->e, H_CARRY, (s32)offsetof(PPCState, xer_ca), H_STATE);
    c->carry_dirty = 0;
}

static void carry_invalidate(JitContext *c)
{
    c->carry_loaded = 0;
    c->carry_dirty  = 0;
}

static void rc_flush_all(JitContext *c)
{
    unsigned g;
    for (g = 0; g < 32; g++)
        gpr_writeback(c, g);
    carry_flush(c);
}

/* Drop every cached register. Used after a helper call, which may have changed
 * PPCState.gpr behind our back. */
static void rc_invalidate_all(JitContext *c)
{
    unsigned i;
    for (i = 0; i < 32; i++) {
        c->gpr[i].host = -1;
        c->gpr[i].dirty = 0;
        c->gpr[i].loaded = 0;
        c->gpr[i].lru = 0;
    }
    for (i = 0; i < 32; i++)
        c->host_taken[i] = 0;
    addr_base_invalidate_all(c);
    carry_invalidate(c);
}

/* ------------------------------------------------------------------ */
/* Floating-point register cache                                        */
/*                                                                      */
/* Gekko's FPRs are pairs (ps0, ps1) and the halves are cached independently,   */
/* because scalar code touches only ps0 and caching whole pairs would waste     */
/* half of a scarce resource.                                                   */
/*                                                                              */
/* This is where the port earns its keep. Gekko's paired-single operations      */
/* round each lane to single precision while storing a double -- which is       */
/* *precisely* the semantics of PowerPC `fadds`/`fmuls`/`fmadds`. So `ps_add`   */
/* compiles to two native, bit-exact instructions with no format conversion, no */
/* vector/scalar shuffling, and no flush-to-zero or NaN-propagation mismatch.   */
/* An x86 backend must cvtpd2ps, operate, cvtps2pd, and still fight the         */
/* differences. Deliberately *not* using VMX here: it has no doubles and is not */
/* IEEE-compliant, so the scalar FPU is both faster and more correct.           */
/* ------------------------------------------------------------------ */

static s32 fpr_state_offset(u32 slot)
{
    u32 f = slot >> 1, half = slot & 1u;
    return (s32)(offsetof(PPCState, ps) + f * sizeof(PairedSingle) + half * 8u);
}

static int fpr_host_owner(JitContext *c, int host)
{
    unsigned s;
    for (s = 0; s < 64; s++)
        if (c->fpr[s].host == host)
            return (int)s;
    return -1;
}

static void fpr_writeback(JitContext *c, u32 slot)
{
    if (c->fpr[slot].host >= 0 && c->fpr[slot].dirty) {
        e_stfd(&c->e, (u32)c->fpr[slot].host, fpr_state_offset(slot), H_STATE);
        c->fpr[slot].dirty = 0;
    }
}

static void fpr_evict(JitContext *c, int host)
{
    int s = fpr_host_owner(c, host);
    if (s >= 0) {
        fpr_writeback(c, (u32)s);
        c->fpr[s].host = -1;
    }
    c->host_fpr_taken[host] = 0;
}

static int fpr_alloc_host(JitContext *c)
{
    int r, victim = -1;
    u32 oldest = 0xFFFFFFFFu;

    for (r = H_FPRCACHE_FIRST; r <= H_FPRCACHE_LAST; r++)
        if (!c->host_fpr_taken[r]) {
            c->host_fpr_taken[r] = 1;
            return r;
        }

    for (r = H_FPRCACHE_FIRST; r <= H_FPRCACHE_LAST; r++) {
        int s;
        u32 age;
        if (c->pin_fpr & (1u << r))
            continue;
        s   = fpr_host_owner(c, r);
        age = (s >= 0) ? c->fpr[s].lru : 0u;
        if (age < oldest) {
            oldest = age;
            victim = r;
        }
    }
    if (victim < 0)
        victim = H_FPRCACHE_FIRST;      /* cannot happen: ops pin at most 4 */

    fpr_evict(c, victim);
    c->host_fpr_taken[victim] = 1;
    return victim;
}

static int fpr_slot_host(JitContext *c, u32 slot, int for_write_only)
{
    c->fpr[slot].lru = ++c->fpr_lru_clock;

    if (c->fpr[slot].host < 0) {
        c->fpr[slot].host = (s8)fpr_alloc_host(c);
        c->fpr[slot].single = 0;            /* provenance unknown */
        if (!for_write_only)
            e_lfd(&c->e, (u32)c->fpr[slot].host, fpr_state_offset(slot), H_STATE);
    }
    c->pin_fpr |= 1u << (unsigned)c->fpr[slot].host;
    return c->fpr[slot].host;
}

static int fpr_read(JitContext *c, u32 f, u32 half)
{
    return fpr_slot_host(c, (u32)FSLOT(f, half), 0);
}

static int fpr_write(JitContext *c, u32 f, u32 half)
{
    int h = fpr_slot_host(c, (u32)FSLOT(f, half), 1);
    c->fpr[FSLOT(f, half)].dirty  = 1;
    c->fpr[FSLOT(f, half)].single = 0;      /* the producer marks it */
    return h;
}

DOL_INLINE void fpr_mark_single(JitContext *c, u32 f, u32 half)
{
    c->fpr[FSLOT(f, half)].single = 1;
}

DOL_INLINE void fpr_mark_both(JitContext *c, u32 f)
{
    c->fpr[FSLOT(f, PS0)].single = 1;
    c->fpr[FSLOT(f, PS1)].single = 1;
}

/* Gekko writes single-precision scalar results to BOTH halves of the pair
 * (Dolphin's Fill, from hardware tests): fadds fsubs fmuls fdivs, the fused
 * singles, fres and frsp. ps1 left stale here is read back by the next
 * ps_sum0 / ps_merge1x / ps_muls1 -- the exact route by which one stall in
 * this title arrived at a NaN angle. d0 is pinned by fpr_slot_host, so the
 * PS1 allocation cannot evict it. */
static void fpr_fill_single(JitContext *c, u32 f, int d0)
{
    int d1 = fpr_write(c, f, PS1);
    e_fmr(&c->e, (u32)d1, (u32)d0);
    fpr_mark_both(c, f);
}

/* The C operand of every single-precision multiply is rounded to 25
 * significant bits before the multiply -- a Gekko FPU property the PPE does
 * not share. Identity for a value already known single; otherwise exact
 * integer arithmetic on the bit pattern, since Force25(x) ==
 * (x + 2^27) & ~(2^28 - 1) for every pattern, NaN payloads included. A
 * floating-point splitting trick would be cheaper but wrong on Inf/NaN and on
 * the huge magnitudes the fuzzer seeds. `scratch` is f6 or f7: outside the
 * guest cache, and not f5 (merge staging) or f0/f1 (psq scratch). */
static int fpr_read_c25(JitContext *c, u32 f, u32 half, u32 scratch)
{
    int m = fpr_read(c, f, half);
    if (c->fpr[FSLOT(f, half)].single)
        return m;
    e_stfd (&c->e, (u32)m, (s32)offsetof(PPCState, quant_tmp), H_STATE);
    e_ld   (&c->e, H_SCRATCH0, (s32)offsetof(PPCState, quant_tmp), H_STATE);
    e_addis(&c->e, H_SCRATCH0, H_SCRATCH0, 0x0800);      /* + 2^27          */
    e_rldicr(&c->e, H_SCRATCH0, H_SCRATCH0, 0, 35);      /* clear low 28    */
    e_std  (&c->e, H_SCRATCH0, (s32)offsetof(PPCState, quant_tmp), H_STATE);
    e_lfd  (&c->e, scratch, (s32)offsetof(PPCState, quant_tmp), H_STATE);
    return (int)scratch;
}

static void fpr_flush_all(JitContext *c)
{
    unsigned s;
    for (s = 0; s < 64; s++)
        fpr_writeback(c, s);
}

static void fpr_invalidate_all(JitContext *c)
{
    unsigned i;
    for (i = 0; i < 64; i++) {
        c->fpr[i].host = -1;
        c->fpr[i].dirty = 0;
        c->fpr[i].single = 0;
        c->fpr[i].lru = 0;
    }
    for (i = 0; i < 32; i++)
        c->host_fpr_taken[i] = 0;
    c->fpr_lru_clock = 0;
}

/* ------------------------------------------------------------------ */
/* Condition register                                                   */
/*                                                                      */
/* The guest CR lives in the *host* CR for the duration of a block. This is the */
/* single largest structural advantage over an x86 backend, which has no        */
/* equivalent register and must synthesize each field. `cmpw` writes it and     */
/* `bc` reads it, exactly as on the guest.                                      */
/* ------------------------------------------------------------------ */

/* Mirror the guest CR into the host CR, once, on first use. */
static void cr_ensure(JitContext *c)
{
    if (c->cr_loaded)
        return;
    e_lwz(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, cr), H_STATE);
    e_mtcrf(&c->e, 0xFF, H_SCRATCH0);   /* BISECT: mtocrf loop reverted */
    c->cr_loaded = 1;
    /* The full reload just rewrote every host field from the authoritative
     * memory copy, including the guard field a guard may have clobbered. */
    c->guard_crf_stale = 0;
}

/* A guest CR write updates one 4-bit field and leaves the other seven alone,
 * so the whole register has to be present before we touch any of it. */
static void cr_touch(JitContext *c)
{
    cr_ensure(c);
    c->cr_dirty = 1;
}

/* Spill the guest CR to memory through a caller-chosen scratch register. The
 * scratch matters: the MMIO/GQR guards spill the CR while an effective address
 * they must preserve is sitting in H_SCRATCH0, so they pass a different one.
 * Using H_SCRATCH0 unconditionally silently clobbered the address that a
 * `stwx` was about to use -- the bug that corrupted an insertion sort. */
static void cr_store_via(JitContext *c, int scratch)
{
    if (!c->cr_dirty)
        return;
    e_mfcr(&c->e, (u32)scratch);
    if (c->guard_crf_stale) {
        /* An MMIO/GQR guard has compared into host field guard_crf without
         * spilling first -- legal because the pre-scan proved no native code
         * in this block reads or writes that field, so the memory copy in
         * PPCState.cr still holds its authoritative guest value (only a
         * helper call can change it, and helpers write memory directly).
         * `mfcr` picked up the guard's junk in that field; re-insert the
         * memory copy's four bits over it before storing, so the guest CR
         * written back is exact at every exit.
         *
         * The second scratch is safe at every merging call site: block
         * exits and helper-call sites have no effective address in flight,
         * and a guard's policy spill (which passes H_SCRATCH3) holds its
         * address in H_SCRATCH0 or a cache register, never H_SCRATCH1/2. */
        int s2 = (scratch == H_SCRATCH1) ? H_SCRATCH2 : H_SCRATCH1;
        u32 mb = (u32)c->guard_crf * 4u;
        e_lwz(&c->e, (u32)s2, (s32)offsetof(PPCState, cr), H_STATE);
        e_rlwimi(&c->e, (u32)scratch, (u32)s2, 0, mb, mb + 3);
    }
    e_stw(&c->e, (u32)scratch, (s32)offsetof(PPCState, cr), H_STATE);
    c->cr_dirty = 0;
}

static void cr_store(JitContext *c)
{
    cr_store_via(c, H_SCRATCH0);
}

/* After a helper call the interpreter may have rewritten PPCState.cr, so the
 * host copy is stale and must be reloaded on next use. */
static void cr_invalidate(JitContext *c)
{
    c->cr_loaded = 0;
    c->cr_dirty  = 0;
}

/* ------------------------------------------------------------------ */
/* Effective address                                                    */
/* ------------------------------------------------------------------ */

/* Defined with the other block-exit machinery below. */
static void emit_downcount(JitContext *c, u32 insts);

/* ------------------------------------------------------------------ */
/* MMIO detection                                                       */
/*                                                                      */
/* lv2 provides no way to emulate a faulting instruction and step over it, so   */
/* device accesses cannot be trapped the way every desktop emulator traps them  */
/* (docs/ARCHITECTURE.md §3.2.1). They have to be recognized in the emitted     */
/* code instead.                                                                */
/*                                                                              */
/* A folded MMIO address always has 0x0C in its top byte and no RAM region      */
/* does -- MEM1 is 0x00-0x01, MEM2 0x10-0x13, locked cache 0x20 -- so one       */
/* mask-and-compare separates them. On a match we abandon the block and let the */
/* interpreter execute that one instruction, which dispatches to the device     */
/* model correctly.                                                             */
/* ------------------------------------------------------------------ */

/* The deferred form of the escape: keep the write-backs, share the
 * boilerplate.
 *
 * The register write-backs stay here because they are exactly what differs
 * between one guarded access and the next -- and because they are what the
 * escape actually runs. Everything after them was byte-for-byte identical at
 * every site in every block, so it lives once at the head of the code cache
 * and the site reaches it with a `bl`, which conveniently leaves LR pointing
 * at the one datum the shared code still needs: the faulting guest pc,
 * written inline as the word after the branch.
 *
 * Cost at the site: the same stores as before, one `addi`, one `bl`, one data
 * word -- against the same stores plus nine. Cost when it runs: one extra
 * instruction, the `bl`.
 *
 * Returns 0 if the shared tail is absent or out of reach, in which case the
 * caller emits the old self-contained tail. Correct either way; only bigger. */
static int emit_esc_tail(JitContext *c, u32 guest_pc, u32 insts)
{
    u32 *stub = (u32 *)jit_esc_stub();
    ptrdiff_t disp;
    size_t at;

    if (!stub || c->e.overflow)
        return 0;
    /* Reach is checked before the write-backs are emitted, since they move
     * the branch further from the target -- generously, because a full cache
     * flush is at most a few dozen words of them. */
    disp = (u8 *)stub - (u8 *)c->e.cur;
    if (disp > 0x01FF0000 || disp < -0x01FF0000)     /* `bl` spans +-32 MiB */
        return 0;

    at = emit_size(&c->e);

    rc_flush_all(c);
    fpr_flush_all(c);
    cr_store(c);
    /* Charge only the instructions completed before this one. The store of
     * the pinned budget is the shared tail's job. */
    if (insts != 0)
        e_addi(&c->e, H_DOWNCOUNT, H_DOWNCOUNT, -(s32)insts);

    /* One `bl` plus the eight words of the shared tail; the inline
     * `.long guest_pc` is data and never fetched. */
    wp_point(c, 1 + 8);
    disp = (u8 *)stub - (u8 *)c->e.cur;
    e_bl(&c->e, (s32)disp);
    emit_word(&c->e, guest_pc);

    g_jit_esc_sites++;
    g_jit_esc_tramp_words += (u64)((emit_size(&c->e) - at) / 4);
    return 1;
}

/* Emit the escape sequence for a guarded access: hand this one instruction to
 * the interpreter and return to the dispatcher.
 *
 * Split out because it is emitted twice -- deferred to the block's cold tail
 * where there is room to record one, and inline where there is not. */
static void emit_mmio_escape(JitContext *c, u32 guest_pc, u32 insts)
{
    rc_flush_all(c);
    fpr_flush_all(c);
    cr_store(c);

    e_load_imm32_lo(&c->e, H_SCRATCH0, guest_pc);
    e_stw(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, pc), H_STATE);
    e_li(&c->e, H_SCRATCH0, 1);
    e_stw(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, force_interp), H_STATE);
    /* Charge only the instructions completed before this one. */
    emit_downcount(c, insts);
    wp_point(c, 2);
    e_mtctr(&c->e, H_DISPATCH);
    e_bctr(&c->e);
}

/* Static branch-prediction hints.
 *
 * BO bit 0 (`y`) is architecturally a hint and nothing else: it inverts the
 * default static prediction, which is "backward taken, forward not-taken".
 * Setting it changes no semantics whatsoever, costs no instruction, and on a
 * core with a 24-cycle mispredict penalty and a BHT shared between two SMT
 * threads it is the cheapest performance bit in the instruction set.
 *
 * Every conditional branch the JIT emits is *forward* -- guards branch out to
 * a cold tail, conditional exits branch over the straight-line path -- so the
 * default is "not taken", and BO_HINT is applied exactly at the sites whose
 * common direction is taken. */
#define BO_LIKELY_TAKEN(bo)   ((bo) | BO_HINT)

static void emit_mmio_guard(JitContext *c, int host_addr_raw, u32 guest_pc)
{
    PPCFixup to_cold;
    u32 crf;

    /* This guard needs a host CR field for its compare, and the guest CR
     * lives in the host CR. Clobbering a live guest field silently corrupts
     * control flow: exactly the bug that once made an insertion sort loop
     * forever.
     *
     * Fast case: the pre-scan found a CR field this block's guest code
     * provably never touches natively (guard_crf). Comparing into that field
     * disturbs nothing the block can observe, so the guest CR stays live in
     * the host CR across the guard -- no spill, no reload. The one debt is
     * settled at the exits: cr_store re-inserts that field's authoritative
     * memory copy over the guard's junk (see cr_store_via). In CR-live code
     * this turns 4 executed words per guard (mfcr+stw before, lwz+mtcrf at
     * the next CR use) into 0, at +2 words per CR store emitted after the
     * first guard.
     *
     * Fallback (no free field): compare into the reserved cr7 as before --
     * spill the guest CR to memory and mark it not-loaded before touching
     * it. The next guest CR access reloads it (every conditional branch does
     * cr_ensure first), and because this guard is hoisted onto the base
     * pointer the spill is paid once per base, not per access. */
    if (c->guard_crf >= 0) {
        crf = (u32)c->guard_crf;
        /* Spill-vs-merge policy. If guest CR activity remains ahead of this
         * point (the scan's last_cr_idx), keep the dirty CR live: the exits
         * pay one 2-word merge, cheaper than the spill+reload the old scheme
         * forced. If nothing ahead ever touches CR again, the dirty value
         * can only flow to a store -- spill it here (host CR stays intact,
         * so nothing reloads) and the exit stores stay plain. Either way the
         * choice affects word count only, never the stored value. */
        if (c->cr_dirty && c->guest_insts >= c->guard_last_cr_idx)
            cr_store_via(c, H_SCRATCH3);   /* H_SCRATCH1/2 are dead here */
        c->guard_crf_stale = 1;
    } else {
        crf = H_CR_JIT;
        cr_store_via(c, H_SCRATCH3);   /* not H_SCRATCH0: it holds the address */
        cr_invalidate(c);
    }

    /* Top byte of the folded address, straight from the unfolded value: the
     * fold clears the top two bits, so a 6-bit extract is the same thing.
     *
     * The mask drops the top byte's LOW bit (ME=30, not 31), so 0x0C and 0x0D
     * both extract to 0x0C: GameCube MMIO folds to top byte 0x0C, the Wii
     * Hollywood window (0xCD000000 / physical 0x0D000000, holding the IPC block
     * and the rest) folds to 0x0D, and they differ only in that bit. No RAM
     * region maps to either 0x0C or 0x0D, so one compare against 0x0C safely
     * diverts both apertures to the MMIO/interpreter slow path. Without this a
     * Hollywood access on the console is an *unrecoverable* fastmem fault --
     * that address is inside the arena reservation but has no backing page. */
    e_rlwinm(&c->e, H_SCRATCH1, (u32)host_addr_raw, 8, 26, 30);
    e_cmplwi(&c->e, crf, H_SCRATCH1, FOLD_MMIO >> 24);

    c->mmio_checks++;
    g_jit_guards_kept++;

    if (c->cold_count < JIT_MAX_COLD_PER_BLOCK) {
        /* Branch *to* the bail-out, which is emitted after the block's hot
         * code. The fast path then falls straight through with nothing but the
         * three instructions above -- and, more importantly, nothing cold
         * between it and the next real instruction. */
        unsigned n = c->cold_count++;
        to_cold = e_bc_cold(c, BO_TRUE, BI_EQ(crf));

        c->cold[n].from     = to_cold;
        c->cold[n].kind     = COLD_MMIO;
        g_jit_cold_site[0]++;
        c->cold[n].guest_pc = guest_pc;
        c->cold[n].insts    = c->guest_insts;
        /* The bail-out spills whatever was live *here*, so the state it was
         * generated under has to travel with it. */
        memcpy(c->cold[n].gpr, c->gpr, sizeof c->gpr);
        memcpy(c->cold[n].fpr, c->fpr, sizeof c->fpr);
        memcpy(c->cold[n].host_taken, c->host_taken, sizeof c->host_taken);
        c->cold[n].cr_loaded    = c->cr_loaded;
        c->cold[n].cr_dirty     = c->cr_dirty;
        c->cold[n].guard_crf_stale = c->guard_crf_stale;
        c->cold[n].carry_loaded = c->carry_loaded;
        c->cold[n].carry_dirty  = c->carry_dirty;
        c->cold[n].lr_known = c->lr_known;
        c->cold[n].lr_const = c->lr_const;
        wp_cold(c, n);
        return;
    }

    /* Out of deferred slots: fall back to an inline bail-out, branched over.
     * Correct, just denser than we would like. */
    {
        PPCFixup to_ram;
        u8 saved_gpr_dirty[32], saved_fpr_dirty[64], saved_cr_dirty;
        u8 saved_carry_dirty;
        unsigned i;

        size_t w0;
        /* Branches over the inline bail-out, i.e. taken on every access that
         * is not MMIO -- which is nearly all of them. Hint it taken. */
        to_ram = e_bc_fwd(&c->e, BO_LIKELY_TAKEN(BO_FALSE), BI_EQ(crf));
        WP_SKIP_BEGIN(c);
        w0 = emit_size(&c->e);

        for (i = 0; i < 32; i++) saved_gpr_dirty[i] = c->gpr[i].dirty;
        for (i = 0; i < 64; i++) saved_fpr_dirty[i] = c->fpr[i].dirty;
        saved_cr_dirty    = c->cr_dirty;
        saved_carry_dirty = c->carry_dirty;

        emit_mmio_escape(c, guest_pc, c->guest_insts);

        for (i = 0; i < 32; i++) c->gpr[i].dirty = saved_gpr_dirty[i];
        for (i = 0; i < 64; i++) c->fpr[i].dirty = saved_fpr_dirty[i];
        c->cr_dirty    = saved_cr_dirty;
        c->carry_dirty = saved_carry_dirty;
        g_jit_inline_bail_words += (u64)((emit_size(&c->e) - w0) / 4);
        g_jit_inline_bail_count++;

        e_patch_here(&c->e, to_ram);
        WP_SKIP_END(c);
    }
}

/* The access's base provably addresses the MMIO aperture: the guard would
 * fire on every execution, so its cold tail is the only path that can run.
 * Compile the escape unconditionally -- no guard, no fastmem access -- and
 * end the block; the dispatcher re-enters at the following pc after the
 * interpreter has performed the device access, exactly as it does today when
 * the guard fires. The caller still emits its (now unreachable) access
 * sequence after the escape's bctr; those few dead words are never fetched
 * and the block ends here, so nothing later depends on the compile-time
 * cache state they were emitted under. */
static void emit_mmio_direct(JitContext *c)
{
    g_jit_mmio_direct++;
    emit_mmio_escape(c, c->pc, c->guest_insts);
    c->ended = 1;
}

static void emit_exit_tail(JitContext *c, u32 next_pc, u32 insts);
static void compile_region(JitContext *c);
static void rc_reset(JitContext *c);
static int  trace_have_room(void);

/* Would the deferred branch in cold slot `n` be better compiled as another
 * REGION of this unit than as an exit from it?
 *
 * The taken side of an inlined conditional is deferred, not cold-in-fact: the
 * compiler has no idea which way the branch goes, and MKWii's hot loop takes
 * the deferred side of two branches on every one of its nineteen million
 * iterations. Compiling that side into the same unit removes the asymmetry
 * entirely -- neither side pays an exit tail, a pc materialization, a
 * dispatcher hash lookup or a second block's prologue -- without needing to
 * know which is hot. Only the seam is asymmetric, and a seam is a register
 * flush the exit was going to pay anyway.
 *
 * Refused where growth would be unbounded or unsafe: past the word budgets
 * (which also keep every guard branch inside its 14-bit displacement), past
 * the unit's instruction budget, and at an AOT-owned target, which must reach
 * the dispatcher exactly as link_or_defer insists. */
static int region_wanted(const JitContext *c, unsigned n)
{
    if (!JIT_TRACE)
        return 0;
    if (c->cold[n].kind != COLD_BRANCH)
        return 0;
    if (c->retaining || c->warm_active)
        return 0;   /* the loop machinery owns this unit's entry contract */
    if (emit_size(&c->e) / 4 >= JIT_TRACE_REGION_WORDS)
        return 0;
    if (c->trace_insts >= JIT_TRACE_MAX_INSTS)
        return 0;
    if (c->region_count >= JIT_TRACE_MAX_REGIONS)
        return 0;
    if (!trace_have_room())
        return 0;
    if (!mem_is_ram(c->cold[n].guest_pc) ||
        jit_aot_owns_pc(c->cold[n].guest_pc))
        return 0;
    /* Inline once. If this target is already a compiled block, the exit links
     * to it in four words; duplicating it here would buy the same traversal
     * at the price of a second copy in a cache that has 8 MiB and flushes
     * everything when it overflows. Measured: without this the boot's peak
     * code footprint went from 6.7 MiB to 10.3 MiB and the console-sized
     * cache flushed; with it the footprint stays under the limit and the
     * executed-word saving is essentially unchanged, because the FIRST unit
     * to reach a target is the one that inlines it, and that is the unit the
     * hot path goes through. */
    return !JIT_TRACE_INLINE_ONCE ||
           !jit_block_compiled(c->state, c->cold[n].guest_pc);
}

/* Turn a deferred conditional branch into the entry of a new region.
 *
 * Everything live is made memory-authoritative and the caches are emptied, so
 * the region begins in exactly the state a UNIT entry begins in. That is the
 * whole correctness argument for regions: the two paths meet in the state the
 * compiler already knows how to start from, so nothing has to be reconciled
 * between them. Provenance, the once-per-unit GQR guard suppression and the
 * guard-field staleness all reset with it, because each is an assertion about
 * a path this one did not take.
 *
 * The cycle budget is charged HERE, for the path that reached the branch: the
 * instructions of the region the branch sits in, plus the branch itself. The
 * region then restarts its own instruction count from zero, so every exit and
 * every interior escape below charges exactly what its path executed -- the
 * correction is applied to the pinned register at the seam, once, and
 * everything downstream nets out without knowing about it.
 *
 * The known-LR state travels with the branch rather than being discarded, so a
 * region that ends in a callee's `blr` still returns into its caller inline. */
static void emit_region_seam(JitContext *c, unsigned n)
{
    u32 pc = c->cold[n].guest_pc;

    rc_flush_all(c);
    fpr_flush_all(c);
    cr_store(c);
    e_addi(&c->e, H_DOWNCOUNT, H_DOWNCOUNT, -(s32)(c->cold[n].insts + 1));

    rc_reset(c);
    fpr_invalidate_all(c);
    cr_invalidate(c);
    carry_invalidate(c);
    prov_reset(c);
    c->gqr_guarded     = 0;
    c->guard_crf_stale = 0;
    c->lr_known    = c->cold[n].lr_known;
    c->lr_const    = c->cold[n].lr_const;
    c->guest_insts = 0;
    c->ended       = 0;
    c->pc          = pc;
    c->region      = 1;
    c->region_count++;
    g_jit_regions++;
}

/* Emit every deferred path, after the unit's entry region. Each is generated
 * under the register-cache state that was live where its guard sits, which is
 * why that state is installed before it rather than using the region's final
 * one.
 *
 * The loop index deliberately re-reads cold_count every iteration: a region
 * compiled here defers conditional branches of its own, which land further
 * down the same array and are picked up by this same sweep. */
/* Branch to a deferred bail-out tail.
 *
 * Inline, the tail is a few dozen words further down the same buffer and one
 * conditional branch reaches it. Split into its own arena it can be megabytes
 * away -- past the +/-32 KB a conditional branch encodes -- so the condition
 * instead skips over an unconditional branch, which spans the whole cache.
 * One extra hot word per bail-out site, against ~26 cold words lifted out of
 * the instruction stream. */
static PPCFixup e_bc_cold(JitContext *c, u32 bo, u32 bi)
{
    if (!c->cold_split)
        return e_bc_fwd(&c->e, bo, bi);
    {
        PPCFixup over = e_bc_fwd(&c->e, bo ^ 8u, bi);   /* BO_TRUE <-> BO_FALSE */
        PPCFixup far  = e_b_fwd(&c->e);
        e_patch_here(&c->e, over);
        return far;
    }
}

static void emit_cold_tail(JitContext *c)
{
    unsigned n;
    PPCEmitter hot_saved;

    /* Everything up to here is hot. */
    c->hot_words = (u32)(emit_size(&c->e) / 4);

    /* Only the MMIO tails are exiled, and the distinction is not cosmetic.
     * Measured over real game code, MMIO bail-outs are 476k words across 64k
     * sites -- 7.4 words each, and reached only a few hundred times a frame
     * because a provably-MMIO base already compiles to a direct path. The
     * deferred taken-branch exits are 2.12M words across 48k sites and run
     * every time a guest conditional branch is taken; they are cold only in
     * the sense that they were deferred, and moving them megabytes away would
     * wreck the locality of an ordinary taken branch.
     *
     * The branch form has to agree: `e_bc_cold` emits the two-word reaching
     * sequence exactly at the sites whose tails land in the other arena. When
     * that agreement was missing, a 14-bit displacement silently truncated
     * into a wild branch -- caught by the differential fuzzer under qemu. */
    for (n = 0; n < c->cold_count; n++) {
        const int exile = c->cold_split && c->cold[n].kind == COLD_MMIO;
        GprSlot saved_gpr[32];
        FprSlot saved_fpr[64];
        u8 saved_taken[32];
        u8 s_cr_l, s_cr_d, s_ca_l, s_ca_d, s_gcs;

        if (exile) { hot_saved = c->e; c->e = c->ce; }

        memcpy(saved_gpr, c->gpr, sizeof saved_gpr);
        memcpy(saved_fpr, c->fpr, sizeof saved_fpr);
        memcpy(saved_taken, c->host_taken, sizeof saved_taken);
        s_cr_l = c->cr_loaded; s_cr_d = c->cr_dirty;
        s_ca_l = c->carry_loaded; s_ca_d = c->carry_dirty;
        s_gcs  = c->guard_crf_stale;

        memcpy(c->gpr, c->cold[n].gpr, sizeof c->gpr);
        memcpy(c->fpr, c->cold[n].fpr, sizeof c->fpr);
        memcpy(c->host_taken, c->cold[n].host_taken, sizeof c->host_taken);
        c->cr_loaded    = c->cold[n].cr_loaded;
        c->cr_dirty     = c->cold[n].cr_dirty;
        c->guard_crf_stale = c->cold[n].guard_crf_stale;
        c->carry_loaded = c->cold[n].carry_loaded;
        c->carry_dirty  = c->cold[n].carry_dirty;

        e_patch_here(&c->e, c->cold[n].from);
        wp_resume(c, n);
        { size_t w0 = emit_size(&c->e);
        if (region_wanted(c, n)) {
            emit_region_seam(c, n);
            compile_region(c);
            /* A region is hot code that happens to be emitted down here.
             * Record its extent so the PPE scheduler can reach it without
             * touching the escape tails around it (jit_compile_into). */
            if (c->sched_span_count < JIT_TRACE_MAX_REGIONS) {
                c->sched_span[c->sched_span_count].from = (u32)(w0 / 4);
                c->sched_span[c->sched_span_count].to =
                    (u32)(emit_size(&c->e) / 4);
                c->sched_span_count++;
            }
        } else if (c->cold[n].kind == COLD_MMIO &&
            emit_esc_tail(c, c->cold[n].guest_pc, c->cold[n].insts)) {
            /* Write-backs plus three words; the rest is shared. */
        } else if (c->cold[n].kind == COLD_BRANCH) {
            /* The taken side of an inlined conditional branch. Everything the
             * hot path kept live gets written back here, once, on the path
             * that leaves -- and the exit tail gives the taken side a link
             * site, so a hot taken branch still chains directly to its target
             * block after resolution. The +1 charges the branch itself, which
             * completed by being taken. */
            rc_flush_all(c);
            fpr_flush_all(c);
            cr_store(c);
            emit_exit_tail(c, c->cold[n].guest_pc, c->cold[n].insts + 1);
        } else {
            emit_mmio_escape(c, c->cold[n].guest_pc, c->cold[n].insts);
        }
        { u64 w = (u64)((emit_size(&c->e) - w0) / 4);
          if (c->cold[n].kind == COLD_BRANCH) {
              g_jit_cold_branch_words += w; g_jit_cold_branch_count++;
          } else {
              g_jit_cold_mmio_words += w; g_jit_cold_mmio_count++;
          } } }

        memcpy(c->gpr, saved_gpr, sizeof c->gpr);
        memcpy(c->fpr, saved_fpr, sizeof c->fpr);
        memcpy(c->host_taken, saved_taken, sizeof c->host_taken);
        c->cr_loaded = s_cr_l; c->cr_dirty = s_cr_d;
        c->guard_crf_stale = s_gcs;
        c->carry_loaded = s_ca_l; c->carry_dirty = s_ca_d;

        if (exile) { c->ce = c->e; c->e = hot_saved; }
    }
    c->cold_count = 0;
}

/* Prepare addressing for a d-form access.
 *
 * Rather than folding a fresh effective address for every access -- three
 * instructions each -- we materialize `arena_base + fold(guest_base)` once in
 * H_ADDRBASE and then use the guest displacement directly. Consecutive
 * accesses off the same pointer, which is what struct field access and array
 * walks look like, cost **one instruction** each after the first.
 *
 * Validity: this computes (base & mask) + d where the guest computes
 * (base + d) & mask. Those differ only if base + d crosses a 0x4000_0000
 * boundary. Guest displacements are +-32 KiB, and every address a title can
 * legitimately name folds into MEM1 (0x0000_0000) or MEM2 (0x1000_0000) --
 * both more than 32 KiB clear of the next boundary. An address that could
 * cross is by construction not backed by any page, so both formulations fault;
 * only the reported address differs, and the fault handler prints it.
 *
 * Returns the host base register; *disp receives the displacement to use.
 */
static const u8 k_addr_base_reg[JIT_ADDR_BASE_SLOTS] = {
    H_ADDRBASE, H_ADDRBASE1, H_ADDRBASE2
};

/* Pick the slot to (re)build in: a free one if there is any, else the least
 * recently used. Evicting the LRU matters for the alternating-pointer case --
 * a round-robin victim would evict the pointer about to be used next. */
static int addr_base_victim(JitContext *c)
{
    int slot, best = 0;
    for (slot = 0; slot < JIT_ADDR_BASE_SLOTS; slot++) {
        if (c->addr_base_guest[slot] < 0)
            return slot;
        if (c->addr_base_lru[slot] < c->addr_base_lru[best])
            best = slot;
    }
    return best;
}

static int emit_addr_d(JitContext *c, u32 op, s32 *disp)
{
    u32 ra = RA(op);
    s32 d  = SIMM(op);
    int slot;

    if (ra == 0) {
        /* Absolute address: fold the constant at compile time and index off
         * the arena base directly. */
        slot = addr_base_victim(c);
        e_load_imm32(&c->e, H_SCRATCH0, ((u32)d) & ARENA_MASK);
        e_add(&c->e, k_addr_base_reg[slot], H_MEMBASE, H_SCRATCH0);
        c->addr_base_guest[slot] = -1;  /* not tied to any guest register */
        c->addr_base_lru[slot] = ++c->addr_base_clock;
        *disp = 0;
        return k_addr_base_reg[slot];
    }

    for (slot = 0; slot < JIT_ADDR_BASE_SLOTS; slot++) {
        if (c->addr_base_guest[slot] == (s8)ra) {
            c->addr_base_lru[slot] = ++c->addr_base_clock;
            *disp = d;
            return k_addr_base_reg[slot];
        }
    }

    {
        int h, cls;

        /* Constant provenance decides the guard. The guard tests the raw
         * base register (hoisted; displacements are never part of the test),
         * so the classification is over the base's possible values alone.
         * A provably-MMIO base skips the access entirely -- the guard would
         * fire every time, so compile its only reachable path. Inside a
         * retained loop keep the ordinary guard instead: the retention
         * machinery relies on the block keeping its scanned shape. */
        cls = prov_classify_reg(c, ra);
        if (cls == EA_MMIO && !c->retaining) {
            emit_mmio_direct(c);
            *disp = 0;
            return k_addr_base_reg[0];  /* unreachable access; dead words */
        }

        h = gpr_read(c, ra);
        slot = addr_base_victim(c);
        /* Form the folded base BEFORE the guard, not after it.
         *
         * The two are independent -- the guard extracts the top byte of the
         * raw base, the fold clears its top two bits into a different
         * register -- but emission order decides which side of the guard's
         * branch they land on, and the scheduler cannot move work across a
         * branch. Emitted after the guard, the fold's `rlwinm`, the `add`
         * that follows it and the access that uses the result are a
         * three-deep dependence chain starting cold: two ALU stalls and then
         * the load's own five cycles, every time. Emitted before it, they sit
         * in the same region as the guard's extract and compare, which are
         * independent of them, so the scheduler interleaves the two chains
         * and the address is ready when the branch retires.
         *
         * Safe: the guard's cold bail-out replays the GPR/FPR cache, the CR
         * and the carry -- never the address-base registers (r10-r12), which
         * are ABI-volatile scratch the escape path re-derives. Executing the
         * fold on a path that then bails out writes a register nothing
         * downstream reads. */
        e_rlwinm(&c->e, k_addr_base_reg[slot], (u32)h, 0, 2, 31); /* fold */
        e_add(&c->e, k_addr_base_reg[slot], H_MEMBASE,
              k_addr_base_reg[slot]);
        /* Guarding the *base* rather than each access is the whole reason
         * hoisting matters here: MMIO is reached through a dedicated base
         * register holding 0xCC00_0000, so one test diverts an entire device
         * sequence, while ordinary RAM code pays three instructions per base
         * pointer and nothing per access.
         *
         * Three base registers are exempt, because the ABI fixes them to main
         * memory and they never address the MMIO aperture:
         *   r1  the stack pointer   -- every local, spill, prologue, epilogue
         *   r2  read-only small data -- const globals
         *   r13 small data area      -- read/write globals
         * These carry an enormous share of real memory traffic, and their guard
         * can never fire, so dropping it (and its cold bail-out) is among the
         * cheapest large wins available.
         *
         * Beyond those, a base whose provenance proves it outside the MMIO
         * aperture -- a lis/addi/ori-built constant, which is how the guest
         * addresses every global -- needs no guard either, and loses the cold
         * bail-out tail with it. Every unknown base still gets the guard, so
         * a genuinely computed MMIO pointer is still diverted correctly. */
        if (ra != 1 && ra != 2 && ra != 13) {
            if (cls == EA_SAFE)
                g_jit_guards_elided++;
            else
                emit_mmio_guard(c, h, c->pc);
        }
        c->addr_base_guest[slot] = (s8)ra;
        c->addr_base_lru[slot] = ++c->addr_base_clock;
    }
    *disp = d;
    return k_addr_base_reg[slot];
}

/* Indexed addressing cannot be hoisted -- the index changes per access -- so it
 * keeps the three-instruction form: add, fold, indexed access. */
static int emit_ea_x(JitContext *c, u32 op, int scratch);

static int emit_ea_x(JitContext *c, u32 op, int scratch)
{
    u32 ra = RA(op), rb = RB(op);
    int hb, cls;

    /* The indexed guard tests the full ra+rb sum, so classify that. Two
     * constants classify exactly; a constant plus a known-upper-half value
     * still bounds the sum tightly enough to decide (ea_classify). */
    cls = (ra == 0) ? prov_classify_reg(c, rb) : prov_classify_sum(c, ra, rb);
    if (cls == EA_MMIO && !c->retaining) {
        emit_mmio_direct(c);
        return scratch;                 /* unreachable access; dead words */
    }

    hb = gpr_read(c, rb);
    if (ra == 0)
        e_mr(&c->e, (u32)scratch, (u32)hb);
    else
        e_add(&c->e, (u32)scratch, (u32)gpr_read(c, ra), (u32)hb);

    /* The index changes per access, so unlike the d-form case this guard
     * cannot be hoisted -- but a provably in-range sum needs none at all.
     *
     * The fold comes first here too, and guarding the *folded* address is
     * exactly equivalent to guarding the raw one: the guard's extract is
     * `rlwinm rX, addr, 8, 26, 30`, whose mask keeps only bits 2..6 of the
     * original top byte, and the fold clears bits 0..1 of it. 0xCC and 0x0C
     * both extract to 0x0C. Doing it in this order puts the fold on the
     * guard's own side of the branch, where the scheduler can overlap it
     * with the compare instead of leaving it to start cold afterwards. */
    e_rlwinm(&c->e, (u32)scratch, (u32)scratch, 0, 2, 31);
    if (cls == EA_SAFE)
        g_jit_guards_elided++;
    else
        emit_mmio_guard(c, scratch, c->pc);
    return scratch;
}

/* ------------------------------------------------------------------ */
/* Interpreter fallback                                                 */
/* ------------------------------------------------------------------ */

/* Which guest opcodes the recompiler declines, counted at COMPILE time.
 *
 * Every fallback is an instruction the interpreter runs one at a time, with a
 * full register spill and reload around it -- one to two orders of magnitude
 * dearer than compiled code. There are 64 places compile_* can decline, and
 * which of them a real title actually reaches was never measured; without that
 * the list is just a list. Indexed by primary opcode, with the extended-opcode
 * families that matter separated out by the caller. */
u64 g_jit_fallback_op[64];
u64 g_jit_fallback_total;
int g_no_psq_store;      /* wiicompiled-nopsqst.txt: decline compiled psq_st */
/* Self-loops with interior branch exits keep their registers live across the
 * back edge. This was refused until measured: the refusal was a compile-time
 * guess tuned on decodeSZS, whose back edge is cold, while the in-race hot
 * block 801b5e3c is the opposite shape (89.9M iterations of one body). In-race
 * the allow-case wins 7.033 -> 7.698 fps, 24.592 -> 23.577 cycles/guest inst.
 * wiicompiled-nowarmcold.txt restores the old refusal. */
int g_warm_no_cold;
u32 g_warm_captured;    /* blocks given warm continuity              */
u32 g_warm_had_cold;    /* ...of which had an interior branch exit   */
u32 g_jit_fallback_xo[3][1024];   /* [0]=op31 [1]=op19 [2]=op4, by XO */

static void emit_fallback(JitContext *c, u32 op)
{
    g_jit_fallback_op[OPCD(op) & 63u]++;
    g_jit_fallback_total++;
    /* Extended-opcode breakdown for the three primaries that dominate the
     * EXECUTED fallback count (31 integer X-form, 19 branch/CR, 4 paired
     * singles). Compile-time only, so it costs nothing at run time, and it is
     * what turns "opcode 31 is expensive" into a list of instructions to
     * write. */
    {
        unsigned pri = OPCD(op) & 63u;
        if (pri == 31 || pri == 19 || pri == 4) {
            unsigned slot = (pri == 31) ? 0u : (pri == 19 ? 1u : 2u);
            g_jit_fallback_xo[slot][XO10(op) & 1023u]++;
        }
    }

    InterpFn fn = interp_decode(op);
    void *code = fn_code_ptr((const void *)fn);

    /* Count this fallback when it EXECUTES, not when it compiles. The static
     * decline count says which opcodes the recompiler refuses; it says nothing
     * about how often a title runs them, and one refused instruction inside a
     * hot loop outweighs hundreds that compile once and never run. Three
     * instructions off the state pointer, which is already live. */
    {   s32 slot = (s32)(offsetof(PPCState, fallback_by_op) +
                         4 * (s32)(OPCD(op) & 63u));
        e_lwz(&c->e, H_SCRATCH1, slot, H_STATE);
        e_addi(&c->e, H_SCRATCH1, H_SCRATCH1, 1);
        e_stw(&c->e, H_SCRATCH1, slot, H_STATE);
    }

    /* The interpreter reads and writes PPCState directly, so everything we are
     * holding in registers must be visible to it first. */
    rc_flush_all(c);
    fpr_flush_all(c);
    cr_store(c);

    /* The pinned cycle budget has to be visible too. A handler that calls
     * ppc_request_exit reads it to work out how much of the slice it is about
     * to destroy, and the register is the only place the live value exists --
     * the memory copy is written only at block exit. Without this spill the
     * refund is computed from a stale number and the emulated clock drifts. */
    e_stw(&c->e, H_DOWNCOUNT, (s32)offsetof(PPCState, downcount), H_STATE);

    /* The interpreter advances pc itself from state->pc, so it must be
     * accurate at the call. */
    e_load_imm32_lo(&c->e, H_SCRATCH1, c->pc);
    e_stw(&c->e, H_SCRATCH1, (s32)offsetof(PPCState, pc), H_STATE);
    e_load_imm32_lo(&c->e, H_SCRATCH1, c->pc + 4);
    e_stw(&c->e, H_SCRATCH1, (s32)offsetof(PPCState, npc), H_STATE);

    e_mr(&c->e, H_ARG0, H_STATE);
    e_load_imm32(&c->e, H_ARG1, op);
    /* Materialize the 64-bit code address. Helpers live in our module, so the
     * TOC in r2 is already correct and only the entry word is needed. */
    {
        u64 addr = (u64)(size_t)code;
        e_lis(&c->e, H_SCRATCH2, (s32)(s16)(addr >> 48));
        e_ori(&c->e, H_SCRATCH2, H_SCRATCH2, (u32)((addr >> 32) & 0xFFFF));
        e_rldicr(&c->e, H_SCRATCH2, H_SCRATCH2, 32, 31);
        e_oris(&c->e, H_SCRATCH2, H_SCRATCH2, (u32)((addr >> 16) & 0xFFFF));
        e_ori(&c->e, H_SCRATCH2, H_SCRATCH2, (u32)(addr & 0xFFFF));
    }
#if defined(_CALL_ELF) && _CALL_ELF == 2
    /* ELFv2 (the big-endian validation build): a called function derives its
     * TOC from r12, so r12 must hold the callee's global entry address. lv2 is
     * ELFv1 and needs none of this -- there the descriptor already carries the
     * TOC and r2 is preserved across the call. */
    e_mr(&c->e, 12, H_SCRATCH2);
#endif
    e_mtctr(&c->e, H_SCRATCH2);
    e_bctrl(&c->e);

    /* The call clobbered the volatile registers and may have changed any part
     * of PPCState, so nothing cached survives. */
    /* The helper may have changed the cycle budget -- ppc_request_exit drives
     * it negative to force the scheduler to run -- so the pinned copy is
     * stale. */
    e_lwz(&c->e, H_DOWNCOUNT, (s32)offsetof(PPCState, downcount), H_STATE);

    rc_invalidate_all(c);
    fpr_invalidate_all(c);
    cr_invalidate(c);
    prov_reset(c);      /* the helper may have written any guest register */
    c->lr_known = 0;    /* ...including the link register */
    c->had_fallback = 1;/* warm continuity cannot survive the wipe */
    c->fallbacks++;

    /* Bucket the fallback so the self-test can name exactly which opcodes to
     * implement next, rather than reporting only that some fraction fell back. */
    g_jit_stats.fallback_by_opcd[OPCD(op)]++;
    if (OPCD(op) == 31)
        g_jit_stats.fallback_x31_xo[XO10(op)]++;
    else if (OPCD(op) == 63)
        g_jit_stats.fallback_x63_xo[XO10(op)]++;
}

/* ------------------------------------------------------------------ */
/* Compact helper calls                                                 */
/*                                                                      */
/* The full fallback above must assume the interpreter handler can read or     */
/* write *any* guest state, so it spills and invalidates everything -- and the */
/* audit showed that spill/reload storm, not the call itself, is most of a     */
/* fallback's cost. A helper that touches only named PPCState fields (the      */
/* time-base read) needs none of that: the register caches live in host        */
/* r18-r31 / f14-f31, which the C ABI preserves, so only the state the ABI can */
/* clobber is flushed -- the carry (r8), the address bases (r10-r12), the      */
/* guest CR mirror (host CR fields 0,1,5-7 are volatile), and the memory copy  */
/* of the downcount, which the extrapolating clock derives mid-slice progress  */
/* from (core_timing.c, now_cycles). The GPR/FPR caches stay live and no       */
/* post-call reload is ever emitted.                                           */
/* ------------------------------------------------------------------ */

/* The derived time base, exactly as the interpreter computes it:
 * tb_now(s) = timing_timebase() + s->tb_offset (interp_system.c:29-32),
 * split by ppc_mftb / ppc_mfspr into halves (interp_system.c:48-49,126-130).
 * These run through the same timing_timebase() the handlers use, so the
 * value observed is bit-identical for identical downcount state. */
static u32 jit_tb_lower_helper(PPCState *s)
{ return (u32)(timing_timebase() + s->tb_offset); }
static u32 jit_tb_upper_helper(PPCState *s)
{ return (u32)((timing_timebase() + s->tb_offset) >> 32); }

/* Call u32 fn(PPCState *) with the register caches kept live. The result is
 * in H_RET0 when this returns. The callee must not read or write s->gpr,
 * s->ps, s->cr, s->xer_ca, or s->downcount (it may read downcount). */
static void emit_compact_call(JitContext *c, const void *fn)
{
    void *code = fn_code_ptr(fn);

    carry_flush(c);
    carry_invalidate(c);        /* H_CARRY (r8) is ABI-volatile */
    cr_store(c);
    cr_invalidate(c);           /* host CR fields 0,1,5-7 are ABI-volatile */
    addr_base_invalidate_all(c);/* r10-r12 are ABI-volatile */

    /* The extrapolating clock reads the memory downcount for mid-slice
     * progress; the pinned register is the only live copy. Same spill the
     * full fallback performs, for the same reason. */
    e_stw(&c->e, H_DOWNCOUNT, (s32)offsetof(PPCState, downcount), H_STATE);

    e_mr(&c->e, H_ARG0, H_STATE);
    {
        u64 addr = (u64)(size_t)code;
        e_lis(&c->e, H_SCRATCH2, (s32)(s16)(addr >> 48));
        e_ori(&c->e, H_SCRATCH2, H_SCRATCH2, (u32)((addr >> 32) & 0xFFFF));
        e_rldicr(&c->e, H_SCRATCH2, H_SCRATCH2, 32, 31);
        e_oris(&c->e, H_SCRATCH2, H_SCRATCH2, (u32)((addr >> 16) & 0xFFFF));
        e_ori(&c->e, H_SCRATCH2, H_SCRATCH2, (u32)(addr & 0xFFFF));
    }
#if defined(_CALL_ELF) && _CALL_ELF == 2
    e_mr(&c->e, 12, H_SCRATCH2);
#endif
    e_mtctr(&c->e, H_SCRATCH2);
    e_bctrl(&c->e);
}

/* ------------------------------------------------------------------ */
/* Block termination                                                    */
/* ------------------------------------------------------------------ */

/* Write everything back, record the next guest pc, and return to the
 * dispatcher. Blocks do not use LR for their own return -- they jump to the
 * dispatcher address pinned in r17 -- which keeps helper calls from having to
 * preserve it and removes all prologue/epilogue stack traffic. */
/* Charge the guest instructions this path executed against the scheduler's
 * budget. The downcount lives in a pinned register for the duration of the
 * block, so this is two instructions rather than a read-modify-write of
 * memory -- worth having on a core where every store goes to L2. */
static void emit_downcount(JitContext *c, u32 insts)
{
    /* A bail-out taken before the block's first instruction has completed
     * charges nothing, and `addi rX, rX, 0` is pure waste. */
    if (insts != 0)
        e_addi(&c->e, H_DOWNCOUNT, H_DOWNCOUNT, -(s32)insts);
    e_stw(&c->e, H_DOWNCOUNT, (s32)offsetof(PPCState, downcount), H_STATE);
}

/* Exit to a statically known guest address, with a patchable link site.
 *
 * The tail emitted here is:
 *
 *      addi  r16, r16, -insts   ; charge the budget, in-register
 *      cmpwi cr7, r16, 0        ; any cycle budget left?
 *      ble   cr7, .Ldisp        ; no -> let the scheduler run
 *      b     .Ldisp             ; LINK SITE: patched to `b target_block`
 *  .Ldisp:
 *      stw   r16, downcount     ; memory copies, for the dispatcher only
 *      lis/ori r3, next_pc
 *      stw   r3, pc
 *      mtctr r17
 *      bctr                     ; dispatcher
 *
 * Unpatched, the link site is a branch to the next instruction and the block
 * behaves exactly as before. Patched, it becomes a direct branch to the next
 * block -- removing a C call and a hash lookup from the hot path, and (because
 * the downcount/pc stores live on the dispatcher side of the link site)
 * costing four instructions rather than ten. The memory pc and downcount are
 * stale while linked blocks chain, which is already the contract: helpers and
 * escapes spill both themselves, and nothing else runs mid-chain.
 *
 * The downcount test is what keeps linking safe: without it, a chain of linked
 * blocks would never return to the scheduler. Anything else that needs the
 * dispatcher's attention (a raised exception, an MSR change) drives the
 * downcount non-positive through ppc_request_exit, so this one test covers
 * every reason to stop rather than needing a second check. */
/* Everything about a block exit that depends on *where* it goes.
 *
 * Split from the register writebacks because a conditional branch has two
 * exits which differ only in their target: the writebacks are identical on both
 * paths, and emitting them twice cost thirteen stores per branch in a realistic
 * block -- around an eighth of the whole thing, and the single largest item in
 * it after the memory accesses themselves. */
static void emit_exit_tail(JitContext *c, u32 next_pc, u32 insts)
{
    /* Charge the cycle budget in-register first; the *memory* copies of the
     * downcount and pc are needed only by the dispatcher and scheduler, so
     * their stores sit on the dispatcher side of the link site. A patched
     * link then transfers control in four instructions -- addi, cmpwi, bc,
     * b -- instead of also paying two dead stores and a pc materialization
     * on every block-to-block transition. The next block neither reads the
     * memory pc nor the memory downcount: every path that reaches C code
     * (dispatcher exit, interpreter fallback, MMIO escape) stores both
     * itself, exactly as before. */
    /* Everything up to and including the link site: what a PATCHED exit
     * executes before it is gone. insts==0 drops the addi. */
    wp_point(c, (c->link_count < JIT_MAX_LINKS_PER_BLOCK ? 3 : 0) +
                (insts != 0 ? 1 : 0));

    if (insts != 0)
        e_addi(&c->e, H_DOWNCOUNT, H_DOWNCOUNT, -(s32)insts);

    if (c->link_count < JIT_MAX_LINKS_PER_BLOCK) {
        e_cmpwi(&c->e, 7, H_DOWNCOUNT, 0);
        e_bc(&c->e, BO_FALSE, BI_GT(7), 8);     /* not > 0 -> dispatcher */

        c->link[c->link_count].word_offset =
            (u32)(emit_mark(&c->e) - c->e.base);
        c->link[c->link_count].target_pc = next_pc;
        c->link_count++;
        e_b(&c->e, 4);                          /* placeholder: fall through */
    }

    e_stw(&c->e, H_DOWNCOUNT, (s32)offsetof(PPCState, downcount), H_STATE);
    e_load_imm32_lo(&c->e, H_SCRATCH0, next_pc);
    e_stw(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, pc), H_STATE);
    /* Reached only when the link site was NOT patched away. */
    wp_point(c, 2);
    e_mtctr(&c->e, H_DISPATCH);
    e_bctr(&c->e);
}

/* The ordinary block exit: write back everything live, then go. */
static void emit_exit_to(JitContext *c, u32 next_pc, u32 insts)
{
    rc_flush_all(c);
    fpr_flush_all(c);
    cr_store(c);
    emit_exit_tail(c, next_pc, insts);
}

/* Exit where the next pc is already in a host register.
 *
 * The store of the target pc must come *first*. cr_store() below needs a
 * scratch register, and the obvious ordering (flush, then store pc) lets
 * `mfcr` clobber the very value we are about to branch to -- a bug that
 * survives every straight-line test and only shows up as a wild jump out of
 * a function return. */
/* The register-target exit without the register/CR flush, for a branch that has
 * already flushed once before splitting into two exits. host_pc must not be
 * H_SCRATCH0's transient use inside a flush -- callers load it after flushing. */
static void emit_exit_reg_tail(JitContext *c, int host_pc, u32 insts)
{
    e_stw(&c->e, (u32)host_pc, (s32)offsetof(PPCState, pc), H_STATE);
    emit_downcount(c, insts);
    wp_point(c, 2);
    e_mtctr(&c->e, H_DISPATCH);
    e_bctr(&c->e);
}

static void emit_exit_reg(JitContext *c, int host_pc, u32 insts)
{
    e_stw(&c->e, (u32)host_pc, (s32)offsetof(PPCState, pc), H_STATE);
    rc_flush_all(c);
    fpr_flush_all(c);
    cr_store(c);
    emit_downcount(c, insts);
    wp_point(c, 2);
    e_mtctr(&c->e, H_DISPATCH);
    e_bctr(&c->e);
}

/* After a fallback that might have raised: if the interpreter handler set
 * exit_requested, leave the block through npc; otherwise fall through and keep
 * executing. The register cache is preserved for the fall-through path with the
 * same save/restore-dirty dance the MMIO and GQR guards use, because
 * rc_flush_all clears the dirty flags as a side effect. */
static void emit_exit_if_requested(JitContext *c)
{
    PPCFixup keep_going;
    u8 saved_gpr_dirty[32], saved_fpr_dirty[64], saved_cr_dirty;
    unsigned i;

    e_lwz(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, exit_requested), H_STATE);
    e_cmpwi(&c->e, H_CR_JIT, H_SCRATCH0, 0);
    /* exit_requested is zero on all but a handful of fallbacks in a whole
     * boot, so this branch is taken essentially always. */
    keep_going = e_bc_fwd(&c->e, BO_LIKELY_TAKEN(BO_TRUE),
                          BI_EQ(H_CR_JIT));   /* == 0: continue */
    WP_SKIP_BEGIN(c);

    for (i = 0; i < 32; i++) saved_gpr_dirty[i] = c->gpr[i].dirty;
    for (i = 0; i < 64; i++) saved_fpr_dirty[i] = c->fpr[i].dirty;
    saved_cr_dirty = c->cr_dirty;

    /* The handler advanced npc; leave through it. Instructions completed so far
     * in this block are charged, this one included. */
    e_lwz(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, npc), H_STATE);
    emit_exit_reg(c, H_SCRATCH0, c->guest_insts + 1);

    for (i = 0; i < 32; i++) c->gpr[i].dirty = saved_gpr_dirty[i];
    for (i = 0; i < 64; i++) c->fpr[i].dirty = saved_fpr_dirty[i];
    c->cr_dirty = saved_cr_dirty;

    e_patch_here(&c->e, keep_going);
    WP_SKIP_END(c);
}

/* ------------------------------------------------------------------ */
/* Instruction compilation                                              */
/* ------------------------------------------------------------------ */

/* Rc=1 forms compare the result against zero into CR0. Because the guest CR is
 * the host CR, this is a single native compare. */
static void maybe_rc(JitContext *c, u32 op, int host_result)
{
    if (RC_BIT(op)) {
        cr_touch(c);
        e_cmpwi(&c->e, 0, (u32)host_result, 0);
    }
}

/* ------------------------------------------------------------------ */
/* Floating point and paired singles                                    */
/* ------------------------------------------------------------------ */

/* Record an FP result as the deferred source of FPSCR[FPRF].
 *
 * The interpreter -- the oracle -- sets FPRF on every arithmetic result
 * (interp_float.c's ppc_set_fprf). Reproducing that inline would mean getting
 * the double into a GPR (an FPR has no other route to the integer side than a
 * store and a reload, with the load-hit-store penalty an in-order PPE cannot
 * hide) and then a dozen instructions of classification -- on EVERY `fadd`,
 * for a field almost nothing reads.
 *
 * So the class is not computed here at all: one `stfd` records the value it
 * would be computed from, and ppc_fprf_sync (interp_fputil.h) does the
 * classification if and when the guest reads FPSCR. Everything that reads
 * FPRF goes through an interpreter handler -- the JIT compiles no FPSCR
 * reader -- so there is no path that can observe the field without syncing.
 *
 * Emitted for exactly the forms whose interpreter handler calls
 * ppc_set_fprf: the arithmetic and multiply-add families, and frsp. NOT for
 * fsel/fmr/fneg/fabs/fnabs (architecturally FPSCR-neutral), not for the
 * paired-single ops (interp_paired.c leaves FPRF alone), and not for the
 * forms that fall back anyway (Rc=1, fsqrt, fres, frsqrte, fctiw). */
static void emit_fprf_defer(JitContext *c, int host_fpr)
{
    e_stfd(&c->e, (u32)host_fpr, (s32)offsetof(PPCState, fprf_src), H_STATE);
}

/* Scalar FP arithmetic. Touches ps0 only; ps1 is architecturally unchanged,
 * which the split register cache represents naturally at no cost.
 *
 * Rc=1 forms write CR1 from FPSCR summary bits, which the fast path does not
 * model -- those fall back. They are rare in compiled game code. */
static int compile_fp_arith(JitContext *c, u32 op, int single)
{
    u32 xo = XO5(op);
    int d, a, b, m;

    if (RC_BIT(op))
        return 0;

    switch (xo) {
    case 21:    /* fadd / fadds */
        a = fpr_read(c, FRA(op), PS0); b = fpr_read(c, FRB(op), PS0);
        d = fpr_write(c, FRT(op), PS0);
        if (single) e_fadds(&c->e, (u32)d, (u32)a, (u32)b);
        else        e_fadd (&c->e, (u32)d, (u32)a, (u32)b);
        emit_fprf_defer(c, d);
        if (single) fpr_fill_single(c, FRT(op), d);
        return 1;

    case 20:    /* fsub / fsubs */
        a = fpr_read(c, FRA(op), PS0); b = fpr_read(c, FRB(op), PS0);
        d = fpr_write(c, FRT(op), PS0);
        if (single) e_fsubs(&c->e, (u32)d, (u32)a, (u32)b);
        else        e_fsub (&c->e, (u32)d, (u32)a, (u32)b);
        emit_fprf_defer(c, d);
        if (single) fpr_fill_single(c, FRT(op), d);
        return 1;

    case 18:    /* fdiv / fdivs */
        a = fpr_read(c, FRA(op), PS0); b = fpr_read(c, FRB(op), PS0);
        d = fpr_write(c, FRT(op), PS0);
        if (single) e_fdivs(&c->e, (u32)d, (u32)a, (u32)b);
        else        e_fdiv (&c->e, (u32)d, (u32)a, (u32)b);
        emit_fprf_defer(c, d);
        if (single) fpr_fill_single(c, FRT(op), d);
        return 1;

    case 25:    /* fmul / fmuls -- second operand comes from FRC, not FRB */
        a = fpr_read(c, FRA(op), PS0);
        m = single ? fpr_read_c25(c, FRC(op), PS0, 6) : fpr_read(c, FRC(op), PS0);
        d = fpr_write(c, FRT(op), PS0);
        if (single) e_fmuls(&c->e, (u32)d, (u32)a, (u32)m);
        else        e_fmul (&c->e, (u32)d, (u32)a, (u32)m);
        emit_fprf_defer(c, d);
        if (single) fpr_fill_single(c, FRT(op), d);
        return 1;

    /* The fused multiply-adds. PowerPC does not round the product before the
     * add, and neither does the host instruction -- so these are exact. */
    case 29:    /* fmadd  */
    case 28:    /* fmsub  */
    case 31:    /* fnmadd */
    case 30:    /* fnmsub */
        a = fpr_read(c, FRA(op), PS0);
        b = fpr_read(c, FRB(op), PS0);
        m = single ? fpr_read_c25(c, FRC(op), PS0, 6) : fpr_read(c, FRC(op), PS0);
        d = fpr_write(c, FRT(op), PS0);
        if (single) {
            if (xo == 29) e_fmadds (&c->e, (u32)d, (u32)a, (u32)m, (u32)b);
            if (xo == 28) e_fmsubs (&c->e, (u32)d, (u32)a, (u32)m, (u32)b);
            if (xo == 31) e_fnmadds(&c->e, (u32)d, (u32)a, (u32)m, (u32)b);
            if (xo == 30) e_fnmsubs(&c->e, (u32)d, (u32)a, (u32)m, (u32)b);
        } else {
            if (xo == 29) e_fmadd (&c->e, (u32)d, (u32)a, (u32)m, (u32)b);
            if (xo == 28) e_fmsub (&c->e, (u32)d, (u32)a, (u32)m, (u32)b);
            if (xo == 31) e_fnmadd(&c->e, (u32)d, (u32)a, (u32)m, (u32)b);
            if (xo == 30) e_fnmsub(&c->e, (u32)d, (u32)a, (u32)m, (u32)b);
        }
        emit_fprf_defer(c, d);
        if (single) fpr_fill_single(c, FRT(op), d);
        return 1;

    case 23: {  /* fsel */
        int ss;
        a = fpr_read(c, FRA(op), PS0);
        m = fpr_read(c, FRC(op), PS0);
        b = fpr_read(c, FRB(op), PS0);
        ss = c->fpr[FSLOT(FRC(op), PS0)].single && c->fpr[FSLOT(FRB(op), PS0)].single;
        d = fpr_write(c, FRT(op), PS0);
        e_fsel(&c->e, (u32)d, (u32)a, (u32)m, (u32)b);
        c->fpr[FSLOT(FRT(op), PS0)].single = (u8)ss;
        return 1;
    }

    default:
        return 0;
    }
}

/* Opcode 63 X-form: moves and sign manipulation. These do not touch FPSCR. */
static int compile_fp_move(JitContext *c, u32 op)
{
    u32 xo = XO10(op);
    int b, d;

    if (RC_BIT(op))
        return 0;

    switch (xo) {
    case 72: case 40: case 264: case 136: case 12: {
        int bs;
        b = fpr_read(c, FRB(op), PS0);
        bs = c->fpr[FSLOT(FRB(op), PS0)].single;    /* before d aliases it */
        d = fpr_write(c, FRT(op), PS0);
        switch (xo) {
        case 72:  e_fmr  (&c->e, (u32)d, (u32)b); break;
        case 40:  e_fneg (&c->e, (u32)d, (u32)b); break;
        case 264: e_fabs (&c->e, (u32)d, (u32)b); break;
        case 136: e_fnabs(&c->e, (u32)d, (u32)b); break;
        case 12:  e_frsp (&c->e, (u32)d, (u32)b); break;
        }
        /* Of this group only frsp produces a rounded FP result, and it is the
         * only one whose interpreter handler sets FPRF. */
        if (xo == 12) {
            emit_fprf_defer(c, d);
            fpr_fill_single(c, FRT(op), d);     /* frsp Fills */
        } else {
            c->fpr[FSLOT(FRT(op), PS0)].single = (u8)bs;
        }
        return 1;
    }
    default:
        return 0;
    }
}

/* fcmpu / fcmpo (opcode 63, XO 0 / 32), mirroring fcmp_common
 * (interp_float.c:236-257).
 *
 * The host's own fcmpu computes precisely the guest's CR field: LT/GT/EQ for
 * ordered operands and the SO position for unordered -- the same bit values
 * (8/4/2/1) cr_set_field writes. So the compare is one native instruction
 * into the mirrored guest CR, plus the FPSCR[FPRF] update the handler also
 * performs: fpscr = (fpscr & ~0x0001F000) | (f << 12), which clears the C
 * bit and deposits the 4-bit compare result (interp_float.c:253-254).
 *
 * The NaN arm (interp_float.c:241-248) additionally sets FPSCR exception
 * state -- VXSNAN for signalling NaNs, VXVC for fcmpo, with the FX/VX
 * summary cascade -- which is not worth inlining: measured across a full
 * boot, ZERO of the 3.18M executed fcmp instructions saw a NaN. The SO bit
 * of the freshly computed field identifies that case exactly, so it escapes
 * to the interpreter through the same deferred force-interp bail-out MMIO
 * guards use; the handler recomputes the compare idempotently and applies
 * the full FPSCR semantics. Host fcmpu is emitted for guest fcmpo too: the
 * CR result is identical and the guest-visible fcmpo difference (VXVC) only
 * exists on the NaN path, which never runs natively.
 *
 * Guest fcmp encodings have no Rc bit; the interpreter dispatches on the
 * 10-bit XO alone, and so does this. */
static int compile_fcmp(JitContext *c, u32 op)
{
    u32 crfd = CRFD(op);
    int a, b;
    unsigned n;

    if (c->cold_count >= JIT_MAX_COLD_PER_BLOCK)
        return 0;

    a = fpr_read(c, FRA(op), PS0);
    b = fpr_read(c, FRB(op), PS0);
    cr_touch(c);
    e_fcmpu(&c->e, crfd, (u32)a, (u32)b);

    /* Unordered -> the interpreter's NaN arm, via a deferred cold escape. */
    n = c->cold_count++;
    c->cold[n].from     = e_bc_cold(c, BO_TRUE, BI_SO(crfd));
    c->cold[n].kind     = COLD_MMIO;
    g_jit_cold_site[1]++;
    c->cold[n].guest_pc = c->pc;
    c->cold[n].insts    = c->guest_insts;
    memcpy(c->cold[n].gpr, c->gpr, sizeof c->gpr);
    memcpy(c->cold[n].fpr, c->fpr, sizeof c->fpr);
    memcpy(c->cold[n].host_taken, c->host_taken, sizeof c->host_taken);
    c->cold[n].cr_loaded    = c->cr_loaded;
    c->cold[n].cr_dirty     = c->cr_dirty;
    /* Merge-integration: like every other cold-slot creation site, the
     * deferred escape runs under the guard-field staleness of THIS point,
     * so snapshot it -- without this the NaN escape's cr_store could skip
     * (or wrongly perform) the guard-field merge. */
    c->cold[n].guard_crf_stale = c->guard_crf_stale;
    c->cold[n].carry_loaded = c->carry_loaded;
    c->cold[n].carry_dirty  = c->carry_dirty;
    c->cold[n].lr_known = c->lr_known;
    c->cold[n].lr_const = c->lr_const;
    wp_cold(c, n);

    /* FPSCR[FPRF] <- 0b0 || f: extract the just-written CR field, clear all
     * five FPRF bits (the C bit stays 0, as the handler leaves it), deposit
     * the four FPCC bits. Field 7 is already the low nibble, hence the &31. */
    e_mfcr(&c->e, H_SCRATCH0);
    e_rlwinm(&c->e, H_SCRATCH0, H_SCRATCH0, (4u * (crfd + 1u)) & 31u, 28, 31);
    e_lwz(&c->e, H_SCRATCH1, (s32)offsetof(PPCState, fpscr), H_STATE);
    e_rlwinm(&c->e, H_SCRATCH1, H_SCRATCH1, 0, 20, 14);   /* ~0x0001F000 */
    e_rlwimi(&c->e, H_SCRATCH1, H_SCRATCH0, 12, 16, 19);  /* f << 12 */
    e_stw(&c->e, H_SCRATCH1, (s32)offsetof(PPCState, fpscr), H_STATE);

    /* That write is authoritative, so a value an earlier FP result deferred
     * into fprf_src must be retired -- otherwise the next FPSCR reader would
     * classify the stale result and overwrite the compare's FPCC. Retire it
     * to FPRF_SRC_NONE (gekko.h) rather than merely equalising the pair: the
     * marker is a signalling NaN, which no FP arithmetic result can equal, so
     * a later `stfd` still registers as pending even if it repeats a value
     * seen before this compare. Both halves of the pair, three instructions
     * to materialize 0x7FF0_0001_0000_0000, on an instruction that is 0.02%
     * of a boot's dynamic count. */
    e_lis(&c->e, H_SCRATCH0, 0x7FF0);
    e_ori(&c->e, H_SCRATCH0, H_SCRATCH0, 1);
    e_rldicr(&c->e, H_SCRATCH0, H_SCRATCH0, 32, 31);
    e_std(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, fprf_src), H_STATE);
    e_std(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, fprf_ack), H_STATE);
    return 1;
}

/* Paired singles.
 *
 * The whole argument for this port in two instructions: each lane is a native
 * single-precision operation with identical rounding to the guest's, so a
 * paired op is two instructions and bit-exact.
 *
 * Lane-crossing hazard: writing (FRT, ps0) can clobber a source that the ps1
 * computation still needs, but only for the *merge* forms, which read across
 * lanes. Ordinary arithmetic reads lane 1 sources exclusively from ps1 slots,
 * which (FRT, ps0) can never alias -- so arithmetic is unconditionally safe. */
static int compile_paired(JitContext *c, u32 op)
{
    u32 xo5 = XO5(op), xo10 = XO10(op);
    int a0, a1, b0, b1, m0, m1, d0, d1;

    if (RC_BIT(op))
        return 0;

    switch (xo5) {
    case 21:    /* ps_add */
    case 20:    /* ps_sub */
    case 18:    /* ps_div */
        a0 = fpr_read(c, FRA(op), PS0); a1 = fpr_read(c, FRA(op), PS1);
        b0 = fpr_read(c, FRB(op), PS0); b1 = fpr_read(c, FRB(op), PS1);
        d0 = fpr_write(c, FRT(op), PS0); d1 = fpr_write(c, FRT(op), PS1);
        if (xo5 == 21) {
            e_fadds(&c->e, (u32)d0, (u32)a0, (u32)b0);
            e_fadds(&c->e, (u32)d1, (u32)a1, (u32)b1);
        } else if (xo5 == 20) {
            e_fsubs(&c->e, (u32)d0, (u32)a0, (u32)b0);
            e_fsubs(&c->e, (u32)d1, (u32)a1, (u32)b1);
        } else {
            e_fdivs(&c->e, (u32)d0, (u32)a0, (u32)b0);
            e_fdivs(&c->e, (u32)d1, (u32)a1, (u32)b1);
        }
        fpr_mark_both(c, FRT(op));
        return 1;

    case 25:    /* ps_mul   */
    case 12:    /* ps_muls0 -- broadcast FRC lane 0 to both multiplies */
    case 13:    /* ps_muls1 -- broadcast FRC lane 1 */
        a0 = fpr_read(c, FRA(op), PS0); a1 = fpr_read(c, FRA(op), PS1);
        m0 = m1 = -1;
        if (xo5 != 13) m0 = fpr_read_c25(c, FRC(op), PS0, 6);
        if (xo5 != 12) m1 = fpr_read_c25(c, FRC(op), PS1, 7);
        d0 = fpr_write(c, FRT(op), PS0); d1 = fpr_write(c, FRT(op), PS1);
        if (xo5 == 25) {
            e_fmuls(&c->e, (u32)d0, (u32)a0, (u32)m0);
            e_fmuls(&c->e, (u32)d1, (u32)a1, (u32)m1);
        } else if (xo5 == 12) {
            e_fmuls(&c->e, (u32)d0, (u32)a0, (u32)m0);
            e_fmuls(&c->e, (u32)d1, (u32)a1, (u32)m0);
        } else {
            e_fmuls(&c->e, (u32)d0, (u32)a0, (u32)m1);
            e_fmuls(&c->e, (u32)d1, (u32)a1, (u32)m1);
        }
        fpr_mark_both(c, FRT(op));
        return 1;

    case 29:    /* ps_madd    */
    case 28:    /* ps_msub    */
    case 31:    /* ps_nmadd   */
    case 30:    /* ps_nmsub   */
    case 14:    /* ps_madds0  */
    case 15:    /* ps_madds1  */
        a0 = fpr_read(c, FRA(op), PS0); a1 = fpr_read(c, FRA(op), PS1);
        b0 = fpr_read(c, FRB(op), PS0); b1 = fpr_read(c, FRB(op), PS1);
        m0 = m1 = -1;
        if (xo5 != 15) m0 = fpr_read_c25(c, FRC(op), PS0, 6);
        if (xo5 != 14) m1 = fpr_read_c25(c, FRC(op), PS1, 7);
        d0 = fpr_write(c, FRT(op), PS0); d1 = fpr_write(c, FRT(op), PS1);
        switch (xo5) {
        case 29:
            e_fmadds(&c->e, (u32)d0, (u32)a0, (u32)m0, (u32)b0);
            e_fmadds(&c->e, (u32)d1, (u32)a1, (u32)m1, (u32)b1);
            break;
        case 28:
            e_fmsubs(&c->e, (u32)d0, (u32)a0, (u32)m0, (u32)b0);
            e_fmsubs(&c->e, (u32)d1, (u32)a1, (u32)m1, (u32)b1);
            break;
        case 31:
            e_fnmadds(&c->e, (u32)d0, (u32)a0, (u32)m0, (u32)b0);
            e_fnmadds(&c->e, (u32)d1, (u32)a1, (u32)m1, (u32)b1);
            break;
        case 30:
            e_fnmsubs(&c->e, (u32)d0, (u32)a0, (u32)m0, (u32)b0);
            e_fnmsubs(&c->e, (u32)d1, (u32)a1, (u32)m1, (u32)b1);
            break;
        case 14:    /* madds0: FRC lane 0 broadcast */
            e_fmadds(&c->e, (u32)d0, (u32)a0, (u32)m0, (u32)b0);
            e_fmadds(&c->e, (u32)d1, (u32)a1, (u32)m0, (u32)b1);
            break;
        case 15:    /* madds1: FRC lane 1 broadcast */
            e_fmadds(&c->e, (u32)d0, (u32)a0, (u32)m1, (u32)b0);
            e_fmadds(&c->e, (u32)d1, (u32)a1, (u32)m1, (u32)b1);
            break;
        }
        fpr_mark_both(c, FRT(op));
        return 1;

    case 23: {  /* ps_sel */
        int s0f, s1f;
        a0 = fpr_read(c, FRA(op), PS0); a1 = fpr_read(c, FRA(op), PS1);
        b0 = fpr_read(c, FRB(op), PS0); b1 = fpr_read(c, FRB(op), PS1);
        m0 = fpr_read(c, FRC(op), PS0); m1 = fpr_read(c, FRC(op), PS1);
        s0f = c->fpr[FSLOT(FRC(op), PS0)].single && c->fpr[FSLOT(FRB(op), PS0)].single;
        s1f = c->fpr[FSLOT(FRC(op), PS1)].single && c->fpr[FSLOT(FRB(op), PS1)].single;
        d0 = fpr_write(c, FRT(op), PS0); d1 = fpr_write(c, FRT(op), PS1);
        e_fsel(&c->e, (u32)d0, (u32)a0, (u32)m0, (u32)b0);
        e_fsel(&c->e, (u32)d1, (u32)a1, (u32)m1, (u32)b1);
        c->fpr[FSLOT(FRT(op), PS0)].single = (u8)s0f;
        c->fpr[FSLOT(FRT(op), PS1)].single = (u8)s1f;
        return 1;
    }

    default:
        break;
    }

    /* Lane-crossing moves. Compiled natively only when the destination aliases
     * neither source, because otherwise writing lane 0 can destroy a value the
     * lane 1 move still needs. The aliasing cases fall back rather than being
     * handled with extra shuffling -- they are uncommon, and a wrong answer
     * here would be a very quiet corruption of vector math. */
    switch (xo10) {
    case 528: case 560: case 592: case 624: case 72: case 40: case 264: case 136: {
        u32 t = FRT(op), fa = FRA(op), fb = FRB(op);
        int s0, s1;

        a0 = fpr_read(c, fa, PS0); a1 = fpr_read(c, fa, PS1);
        b0 = fpr_read(c, fb, PS0); b1 = fpr_read(c, fb, PS1);

        /* Which source lane feeds each destination lane. */
        {
            u8 *F = &c->fpr[0].single;      /* provenance travels with the lane */
            int ss0, ss1;
            (void)F;
        switch (xo10) {
        case 528: s0 = a0; s1 = b0; ss0 = c->fpr[FSLOT(fa,PS0)].single; ss1 = c->fpr[FSLOT(fb,PS0)].single; break;
        case 560: s0 = a0; s1 = b1; ss0 = c->fpr[FSLOT(fa,PS0)].single; ss1 = c->fpr[FSLOT(fb,PS1)].single; break;
        case 592: s0 = a1; s1 = b0; ss0 = c->fpr[FSLOT(fa,PS1)].single; ss1 = c->fpr[FSLOT(fb,PS0)].single; break;
        case 624: s0 = a1; s1 = b1; ss0 = c->fpr[FSLOT(fa,PS1)].single; ss1 = c->fpr[FSLOT(fb,PS1)].single; break;
        default:  s0 = b0; s1 = b1; ss0 = c->fpr[FSLOT(fb,PS0)].single; ss1 = c->fpr[FSLOT(fb,PS1)].single; break;
        }

        d0 = fpr_write(c, t, PS0); d1 = fpr_write(c, t, PS1);
        c->fpr[FSLOT(t, PS0)].single = (u8)ss0;
        c->fpr[FSLOT(t, PS1)].single = (u8)ss1;
        }

        /* The destination may alias a source -- `ps_merge01 f3,f3,f4` is how a
         * vector is built in place, and it is anything but rare: measured on
         * hardware, opcode 4 was 6.8 million EXECUTED fallbacks in one
         * interval, and this decline was most of it. The old code refused the
         * aliasing case on the grounds that writing lane 0 can destroy a value
         * lane 1 still needs, which is true and costs exactly one extra move
         * to avoid: stage the lane-1 source first, then write lane 0.
         *
         * f0..f13 are outside the guest FPR cache (H_FPRCACHE_FIRST is 14). */
        if (s1 == d0) {
            e_fmr(&c->e, 5, (u32)s1);
            s1 = 5;
        }

        switch (xo10) {
        case 528: case 560: case 592: case 624: case 72:
            e_fmr(&c->e, (u32)d0, (u32)s0);
            e_fmr(&c->e, (u32)d1, (u32)s1);
            break;
        case 40:
            e_fneg(&c->e, (u32)d0, (u32)s0);
            e_fneg(&c->e, (u32)d1, (u32)s1);
            break;
        case 264:
            e_fabs(&c->e, (u32)d0, (u32)s0);
            e_fabs(&c->e, (u32)d1, (u32)s1);
            break;
        case 136:
            e_fnabs(&c->e, (u32)d0, (u32)s0);
            e_fnabs(&c->e, (u32)d1, (u32)s1);
            break;
        }
        return 1;
    }
    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Quantized load/store (psq_l / psq_st)                                */
/*                                                                      */
/* These read a GQR to decide the element format and a power-of-two scale, so   */
/* a general implementation is a chain of tests around a conversion -- roughly  */
/* twenty-five instructions through the interpreter.                            */
/*                                                                              */
/* But the format is *runtime state that never changes inside a block*, and in  */
/* practice it is overwhelmingly plain f32 with no scaling, for which the whole */
/* instruction collapses to two `lfs`. So the block is compiled against the GQR */
/* value observed at compile time and guarded by a runtime comparison. The      */
/* guard is emitted once per GQR per block: nothing within a block can change   */
/* one, because `mtspr` exits to the interpreter.                               */
/* ------------------------------------------------------------------ */

/* Emit a check that the GQR still holds the value this code was compiled for,
 * bailing out to the interpreter if a title has reconfigured it. */
static void emit_gqr_guard(JitContext *c, u32 gqr_index, u32 expect, u32 mask)
{
    PPCFixup ok;
    u8 saved_gpr_dirty[32], saved_fpr_dirty[64], saved_cr_dirty;
    u8 saved_carry_dirty;
    unsigned i;
    u32 crf;

    if (c->gqr_guarded & (1u << gqr_index))
        return;

    /* Same CR-field discipline as emit_mmio_guard: use the block's free
     * field when the pre-scan found one (no spill, no reload; cr_store
     * settles the clobbered field at the exits), else spill and use the
     * reserved cr7. */
    if (c->guard_crf >= 0) {
        crf = (u32)c->guard_crf;
        /* Same spill-vs-merge policy as emit_mmio_guard. */
        if (c->cr_dirty && c->guest_insts >= c->guard_last_cr_idx)
            cr_store_via(c, H_SCRATCH3);
        c->guard_crf_stale = 1;
    } else {
        crf = H_CR_JIT;
        cr_store_via(c, H_SCRATCH3);
        cr_invalidate(c);
    }

    e_lwz(&c->e, H_SCRATCH1,
          (s32)(offsetof(PPCState, gqr) + 4 * gqr_index), H_STATE);
    e_load_imm32(&c->e, H_SCRATCH2, mask);
    e_and(&c->e, H_SCRATCH1, H_SCRATCH1, H_SCRATCH2);
    e_load_imm32(&c->e, H_SCRATCH2, expect & mask);
    e_cmplw(&c->e, crf, H_SCRATCH1, H_SCRATCH2);

    /* This escape is the same shape as an MMIO bail-out and fires just as
     * rarely -- only a title reconfiguring a GQR mid-stream reaches it -- so
     * it belongs in the block's cold tail, not inline between the two halves
     * of a quantized load. Inline it put eleven words of never-executed code
     * into the hot instruction stream at every first use of a GQR in a block.
     *
     * The polarity flips with the move: the fast path now falls through and
     * the *mismatch* branches away. */
    if (c->cold_count < JIT_MAX_COLD_PER_BLOCK && jit_esc_stub()) {
        unsigned n = c->cold_count++;
        c->cold[n].from     = e_bc_cold(c, BO_FALSE, BI_EQ(crf));
        c->cold[n].kind     = COLD_MMIO;
        g_jit_cold_site[2]++;
        c->cold[n].guest_pc = c->pc;
        c->cold[n].insts    = c->guest_insts;
        memcpy(c->cold[n].gpr, c->gpr, sizeof c->gpr);
        memcpy(c->cold[n].fpr, c->fpr, sizeof c->fpr);
        memcpy(c->cold[n].host_taken, c->host_taken, sizeof c->host_taken);
        c->cold[n].cr_loaded       = c->cr_loaded;
        c->cold[n].cr_dirty        = c->cr_dirty;
        c->cold[n].guard_crf_stale = c->guard_crf_stale;
        c->cold[n].carry_loaded    = c->carry_loaded;
        c->cold[n].carry_dirty     = c->carry_dirty;
        c->cold[n].lr_known = c->lr_known;
        c->cold[n].lr_const = c->lr_const;
        wp_cold(c, n);
        c->gqr_guarded |= (u8)(1u << gqr_index);
        return;
    }

    /* No cold slot, or no shared tail: the old inline escape, branched over. */
    /* Taken whenever the GQR still holds the format the site was compiled
     * for, which is the case on every execution but a reconfiguration. */
    ok = e_bc_fwd(&c->e, BO_LIKELY_TAKEN(BO_TRUE), BI_EQ(crf));

    for (i = 0; i < 32; i++) saved_gpr_dirty[i] = c->gpr[i].dirty;
    for (i = 0; i < 64; i++) saved_fpr_dirty[i] = c->fpr[i].dirty;
    saved_cr_dirty    = c->cr_dirty;
    saved_carry_dirty = c->carry_dirty;

    rc_flush_all(c);
    fpr_flush_all(c);
    cr_store(c);
    e_load_imm32_lo(&c->e, H_SCRATCH0, c->pc);
    e_stw(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, pc), H_STATE);
    e_li(&c->e, H_SCRATCH0, 1);
    e_stw(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, force_interp), H_STATE);
    emit_downcount(c, c->guest_insts);
    e_mtctr(&c->e, H_DISPATCH);
    e_bctr(&c->e);

    for (i = 0; i < 32; i++) c->gpr[i].dirty = saved_gpr_dirty[i];
    for (i = 0; i < 64; i++) c->fpr[i].dirty = saved_fpr_dirty[i];
    c->cr_dirty = saved_cr_dirty;
    /* The carry belongs in that restore too: rc_flush_all cleared it, but the
     * store it emitted is on the branched-over side, so the hot path still
     * holds an unspilled XER[CA]. (Unreachable now that the deferred arm
     * above is the normal one, but the omission was real.) */
    c->carry_dirty = saved_carry_dirty;

    e_patch_here(&c->e, ok);
    c->gqr_guarded |= (u8)(1u << gqr_index);
}

/* opcodes 56/57 (psq_l, psq_lu) and 60/61 (psq_st, psq_stu). */
static int compile_psq(JitContext *c, u32 op)
{
    u32 opcd  = OPCD(op);
    int store = (opcd == 60 || opcd == 61);
    int upd   = (opcd == 57 || opcd == 61);
    u32 idx   = PS_I(op), w = PS_W(op);
    u32 gqr   = c->state->gqr[idx];
    u32 type  = store ? gqr_st_type(gqr)  : gqr_ld_type(gqr);
    u32 scale = store ? gqr_st_scale(gqr) : gqr_ld_scale(gqr);
    u32 mask  = store ? 0x00003F07u : 0x3F070000u;   /* the half we depend on */
    s32 d     = PS_D(op);
    int ra    = (int)RA(op);
    int base, f0, f1;

    if (!(c->state->hid2 & HID2_LSQE))
        return 0;

    /* f32 needs no conversion at all and is the cheapest case. The integer
     * formats need a widen, an integer-to-float conversion and a scale
     * multiply -- half a dozen instructions, against a fallback that spills
     * every GPR, every FPR and CR before calling the interpreter. Compiling
     * them is the difference between a handful of instructions and tens of
     * them, and these are the formats a title uses for compact vertex data.
     *
     * Stores stay with the interpreter for now: they need saturation to the
     * format's range, which is several compares per lane. */
    if (type != QUANT_F32 && store)
        return 0;

    /* Update forms need a base register, and RA=0 is invalid for them. */
    if (upd && ra == 0)
        return 0;

    emit_gqr_guard(c, idx, gqr, mask);

    /* Reuse the ordinary d-form addressing path, which brings the hoisted base
     * pointer and the MMIO guard with it. The displacement field is 12-bit
     * here rather than 16-bit, so it is handled directly. */
    {
        u32 fake = (op & 0xFFFF0000u) | ((u32)d & 0xFFFFu);
        base = emit_addr_d(c, fake, &d);
    }

    if (store && type == QUANT_F32) {
        f0 = fpr_read(c, FRT(op), PS0);
        e_stfs(&c->e, (u32)f0, d, (u32)base);
        if (!w) {
            f1 = fpr_read(c, FRT(op), PS1);
            e_stfs(&c->e, (u32)f1, d + 4, (u32)base);
        }
    } else if (store && g_no_psq_store) {
        /* A/B: fall the quantised store back to the interpreter, so a stall
         * that survives can be attributed to (or cleared of) the compiled
         * store added in §18. The idiom at the stall is a psq_st/psq_l round
         * trip through U16, and that store is the newest code in the path. */
        return 0;
    } else if (store) {
        /* Quantised store. f0..f13 are outside the guest-FPR cache
         * (H_FPRCACHE_FIRST is 14), so they are free scratch. */
        static const int k_ssize[4] = { 1, 2, 1, 2 };   /* U8 U16 S8 S16 */
        unsigned fi2 = (unsigned)type - QUANT_U8;
        int      sz  = k_ssize[fi2];
        unsigned lane;

        e_lfd(&c->e, 1, (s32)(offsetof(PPCState, quant_scale) +
                              (u32)scale * (u32)sizeof(f64)), H_STATE);
        e_lfd(&c->e, 2, (s32)(offsetof(PPCState, quant_lo) +
                              fi2 * (u32)sizeof(f64)), H_STATE);
        e_lfd(&c->e, 3, (s32)(offsetof(PPCState, quant_hi) +
                              fi2 * (u32)sizeof(f64)), H_STATE);

        for (lane = 0; lane < (w ? 1u : 2u); lane++) {
            int fsrc = fpr_read(c, FRT(op), lane == 0 ? PS0 : PS1);
            e_fmul(&c->e, 0, (u32)fsrc, 1);      /* f0 = value * 2^exp     */
            e_fsub(&c->e, 4, 0, 2);              /* f4 = f0 - lo           */
            e_fsel(&c->e, 0, 4, 0, 2);           /* max(f0,lo); NaN -> lo  */
            e_fsub(&c->e, 4, 3, 0);              /* f4 = hi - f0           */
            e_fsel(&c->e, 0, 4, 0, 3);           /* min(f0,hi)             */
            e_fctiwz(&c->e, 0, 0);               /* truncate toward zero   */
            /* The only route from an FPR to a GPR is through memory, and the
             * integer is the LOW word of the double: offset +4. */
            e_stfd(&c->e, 0, (s32)offsetof(PPCState, quant_tmp), H_STATE);
            e_lwz(&c->e, H_SCRATCH1,
                  (s32)offsetof(PPCState, quant_tmp) + 4, H_STATE);
            if (sz == 1)
                e_stb(&c->e, H_SCRATCH1, d + (s32)lane, (u32)base);
            else
                e_sth(&c->e, H_SCRATCH1, d + (s32)(lane * 2u), (u32)base);
        }
    } else if (type != QUANT_F32) {
        /* One lane: widen the integer into a GPR, pass it through memory (the
         * only route from an integer register to an FPR), convert, and scale.
         * f0/f1 are outside the guest-FPR cache, so they are free scratch. */
        /* Indexed by `type - QUANT_U8`, so the ROWS MUST FOLLOW THE HARDWARE
         * TYPE ENCODING, which is 4=U8, 5=U16, 6=S8, 7=S16 -- not the
         * u8/s8/u16/s16 an eye expects. Ordered the intuitive way, a U16 load
         * decoded as one signed byte and an S8 load as two unsigned bytes:
         * wrong width AND wrong sign, silently, only for those two formats,
         * and only in compiled code (interp_paired.c switches on the type and
         * was always right).
         *
         * Titles use U16 for compact vertex data, so this fed garbage floats
         * into the guest. One consequence was a hang: a range-reduction loop
         * (`fsubs`/`fcmpu`/branch-back) reached a modulus of 1.58e-319 -- the
         * integer 32000 sitting raw in a mantissa -- and a loop that subtracts
         * a value indistinguishable from zero never terminates. */
        static const struct { int size; int is_signed; } k_fmt[4] = {
            { 1, 0 },   /* QUANT_U8  = 4 */
            { 2, 0 },   /* QUANT_U16 = 5 */
            { 1, 1 },   /* QUANT_S8  = 6 */
            { 2, 1 },   /* QUANT_S16 = 7 */
        };
        unsigned fi = (unsigned)type - QUANT_U8;
        int size = k_fmt[fi].size, sgn = k_fmt[fi].is_signed;
        unsigned lane;

        /* The scale is fixed for this block (emit_gqr_guard has already
         * checked the GQR still holds what we compiled for), so it is a
         * constant load rather than a computation. */
        e_lfd(&c->e, 1, (s32)(offsetof(PPCState, dequant_scale) +
                              (u32)scale * (u32)sizeof(f64)), H_STATE);

        for (lane = 0; lane < (w ? 1u : 2u); lane++) {
            s32 off = d + (s32)(lane * (unsigned)size);
            int fd = fpr_write(c, FRT(op), lane == 0 ? PS0 : PS1);

            if (size == 1) {
                e_lbz(&c->e, H_SCRATCH1, off, (u32)base);
                if (sgn) e_extsb(&c->e, H_SCRATCH1, H_SCRATCH1);
            } else if (sgn) {
                /* lha is an algebraic load and therefore microcoded on the
                 * PPE; lhz + extsh is two non-microcoded instructions. */
                e_lhz(&c->e, H_SCRATCH1, off, (u32)base);
                e_extsh(&c->e, H_SCRATCH1, H_SCRATCH1);
            } else {
                e_lhz(&c->e, H_SCRATCH1, off, (u32)base);
            }
            e_std(&c->e, H_SCRATCH1, (s32)offsetof(PPCState, quant_scratch),
                  H_STATE);
            e_lfd(&c->e, 0, (s32)offsetof(PPCState, quant_scratch), H_STATE);
            e_fcfid(&c->e, 0, 0);                  /* integer -> double     */
            e_fmul(&c->e, (u32)fd, 0, 1);          /* apply 2^-scale        */
            fpr_mark_single(c, FRT(op), lane == 0 ? PS0 : PS1);
        }
        if (w) {
            f1 = fpr_write(c, FRT(op), PS1);
            e_lfd(&c->e, (u32)f1, (s32)offsetof(PPCState, const_one), H_STATE);
            fpr_mark_single(c, FRT(op), PS1);
        }
    } else {
        f0 = fpr_write(c, FRT(op), PS0);
        e_lfs(&c->e, (u32)f0, d, (u32)base);
        fpr_mark_single(c, FRT(op), PS0);
        f1 = fpr_write(c, FRT(op), PS1);
        if (w) {
            /* Width-1 loads force the second lane to 1.0, which is what makes
             * psq_l useful for three-component vectors. */
            e_lfd(&c->e, (u32)f1, (s32)offsetof(PPCState, const_one), H_STATE);
            fpr_mark_single(c, FRT(op), PS1);
        } else {
            e_lfs(&c->e, (u32)f1, d + 4, (u32)base);
            fpr_mark_single(c, FRT(op), PS1);
        }
    }

    if (upd) {
        int h = gpr_read(c, (u32)ra);
        e_addi(&c->e, (u32)h, (u32)h, PS_D(op));
        c->gpr[ra].dirty = 1;
        addr_base_drop(c, (u32)ra);
        prov_kill(c, (u32)ra);  /* written without going through gpr_write */
    }
    return 1;
}

/* Floating-point loads and stores.
 *
 * `lfs` and `stfs` are the standout case: the single<->double conversion the
 * PowerPC manual specifies is *the hardware instruction* here, so each is one
 * instruction and bit-exact. An x86 backend must call a software routine that
 * special-cases subnormals and NaN payloads -- on every single-precision access,
 * which in Wii game code is most floating-point traffic. */
/* Write the effective address back to RA, for the update forms. RA holds a
 * guest pointer, so this is a plain 32-bit add; gpr_write also drops the
 * cached address base, which was derived from RA's old value. */
static void emit_ra_update(JitContext *c, u32 op, s32 disp)
{
    int ra = gpr_write(c, RA(op));
    e_addi(&c->e, (u32)ra, (u32)ra, disp);
}

static int compile_fp_loadstore(JitContext *c, u32 op)
{
    s32 disp;
    int base, d, sreg;
    /* The update forms differ only in writing the address back to RA. */
    u32 opcd   = OPCD(op);
    int update = (opcd == 49 || opcd == 51 || opcd == 53 || opcd == 55);

    if (update) {
        if (RA(op) == 0)
            return 0;               /* invalid form: leave it alone */
        opcd -= 1;                  /* lfsu->lfs, lfdu->lfd, stfsu->stfs... */
    }

    switch (opcd) {
    case 48:    /* lfs -- fills *both* paired-single halves on Gekko */
        base = emit_addr_d(c, op, &disp);
        d = fpr_write(c, FRT(op), PS0);
        e_lfs(&c->e, (u32)d, disp, (u32)base);
        {
            int d1 = fpr_write(c, FRT(op), PS1);
            e_fmr(&c->e, (u32)d1, (u32)d);
            fpr_mark_both(c, FRT(op));
        }
        break;

    case 50:    /* lfd -- writes ps0 only */
        base = emit_addr_d(c, op, &disp);
        d = fpr_write(c, FRT(op), PS0);
        e_lfd(&c->e, (u32)d, disp, (u32)base);
        break;

    case 52:    /* stfs */
        base = emit_addr_d(c, op, &disp);
        sreg = fpr_read(c, FRT(op), PS0);
        e_stfs(&c->e, (u32)sreg, disp, (u32)base);
        break;

    case 54:    /* stfd */
        base = emit_addr_d(c, op, &disp);
        sreg = fpr_read(c, FRT(op), PS0);
        e_stfd(&c->e, (u32)sreg, disp, (u32)base);
        break;

    default:
        return 0;
    }

    if (update)
        emit_ra_update(c, op, disp);
    return 1;
}

/* ------------------------------------------------------------------ */
/* Carry                                                                */
/*                                                                      */
/* The guest keeps XER[CA] as its own word (0 or 1) rather than packed into a   */
/* status register, so producing it means materialising a 0 or 1 and consuming  */
/* it means reading one. Two different mechanisms do that here, and which is    */
/* cheaper depends on the instruction.                                          */
/*                                                                              */
/* For shifts, the *host's* carry is already right: `sraw`/`srawi` are 32-bit   */
/* forms whose XER[CA] is defined over the low 32 bits, which is the guest's    */
/* definition exactly. Lifting it out costs `li` + `addze`.                     */
/*                                                                              */
/* For add and subtract it is not. The host is 64-bit and the guest is 32-bit,  */
/* so a host `addc` of two zero-extended values never carries out of bit 63 and */
/* its XER[CA] is always zero. That sounds like an obstacle and is actually the */
/* cheaper path: because the inputs are zero-extended, **bit 32 of the 64-bit   */
/* sum is the 32-bit carry out**. A plain `add` and a shift produce it, with no */
/* access to the XER at all and no dependency on a serialising status register. */
/* ------------------------------------------------------------------ */

/* Lift the host's XER[CA] into the guest's carry register. */
static void emit_store_host_ca(JitContext *c)
{
    e_li(&c->e, H_CARRY, 0);
    e_addze(&c->e, H_CARRY, H_CARRY);
    carry_produced(c);
}

/* Guest values reach host registers zero-extended (they are loaded with `lwz`),
 * but an earlier guest `add` that overflowed 32 bits leaves rubbish above bit
 * 31 -- harmless for ordinary arithmetic, since only the low word is ever
 * stored, and fatal here, because the carry *is* bit 32. So carry-form
 * instructions clean their own inputs rather than requiring every other
 * instruction to keep registers tidy. */
static void emit_clean32(JitContext *c, int dst, int src)
{
    e_clrldi(&c->e, (u32)dst, (u32)src, 32);
}

/* addc / adde: RT = RA + RB (+ CA), carry out to xer_ca. */
static int emit_carry_add(JitContext *c, u32 op, int use_carry_in, int unused)
{
    int a = gpr_read(c, RA(op)), b = gpr_read(c, RB(op));
    int d;
    (void)unused;

    emit_clean32(c, H_SCRATCH0, a);
    emit_clean32(c, H_SCRATCH1, b);
    e_add(&c->e, H_SCRATCH0, H_SCRATCH0, H_SCRATCH1);

    if (use_carry_in) {
        /* Adding the incoming carry cannot produce a second carry out: the
         * largest possible sum is 2*(2^32 - 1) + 1, which still fits below
         * 2^33, so bit 32 remains the only carry bit. */
        e_add(&c->e, H_SCRATCH0, H_SCRATCH0, (u32)carry_reg(c));
    }

    /* srdi H_CARRY, H_SCRATCH0, 32 -- the carry out is bit 32 of the sum. */
    e_rldicl(&c->e, H_CARRY, H_SCRATCH0, 32, 32);
    carry_produced(c);

    d = gpr_dest(c, RT(op));
    e_mr(&c->e, (u32)d, H_SCRATCH0);
    maybe_rc(c, op, d);
    return 1;
}

/* subfc / subfe: RT = ~RA + RB + (1 or CA). PowerPC defines subtraction this
 * way rather than as RB - RA, and the difference is exactly the carry: the
 * complement-and-add form produces a carry out on "no borrow", which is the
 * convention the guest's own subfe chains depend on. */
static int emit_carry_sub(JitContext *c, u32 op, int use_carry_in)
{
    int a = gpr_read(c, RA(op)), b = gpr_read(c, RB(op));
    int d;

    emit_clean32(c, H_SCRATCH0, a);
    e_nor(&c->e, H_SCRATCH0, H_SCRATCH0, H_SCRATCH0);   /* ~a, 64-bit      */
    emit_clean32(c, H_SCRATCH0, H_SCRATCH0);            /* ~a in 32 bits   */
    emit_clean32(c, H_SCRATCH1, b);
    e_add(&c->e, H_SCRATCH0, H_SCRATCH0, H_SCRATCH1);

    if (use_carry_in) {
        e_add(&c->e, H_SCRATCH0, H_SCRATCH0, (u32)carry_reg(c));
    } else {
        e_addi(&c->e, H_SCRATCH0, H_SCRATCH0, 1);
    }

    e_rldicl(&c->e, H_CARRY, H_SCRATCH0, 32, 32);
    carry_produced(c);

    d = gpr_dest(c, RT(op));
    e_mr(&c->e, (u32)d, H_SCRATCH0);
    maybe_rc(c, op, d);
    return 1;
}


static int compile_op31(JitContext *c, u32 op)
{
    u32 xo = XO10(op);

    /* OE forms need overflow tracking; leave them to the interpreter.
     *
     * Testing instruction bit 10 alone does not identify them. That bit is the
     * top of the ten-bit extended opcode, so *every* X-form instruction whose
     * XO is 512 or more has it set -- `srw` (536), `sraw` (792), `srawi` (824),
     * `extsb` (954), `extsh` (922), `extsw` (986). A blanket test therefore
     * rejects a set of extremely common instructions that have perfectly good
     * handlers below, and does it silently: the interpreter fallback is
     * correct, merely some seventeen times slower. Nothing fails, the emulator
     * is just quietly slow, which is the hardest kind of bug to notice.
     *
     * OE only exists on the XO-form arithmetic, all of which has a base opcode
     * below 512; with OE set the field reads base + 512. So an instruction is
     * an OE form precisely when its XO is 512 or more *and* subtracting 512
     * lands on one of those arithmetic opcodes. */
    if (xo >= 512) {
        switch (xo - 512) {
        case 8: case 10: case 40: case 104: case 136: case 138:
        case 200: case 202: case 232: case 234: case 235:
        case 459: case 491:
            return 0;           /* genuinely an OE form */
        default:
            break;              /* the XO simply has its top bit set */
        }
    }

    switch (xo) {
    /* Cache management: dcbst, dcbf, dcbtst, dcbt, dcbi, icbi.
     *
     * The interpreter maps every one of these to ppc_cache_nop -- there is no
     * guest cache to flush, and emulator memory is coherent -- so the compiled
     * form is nothing at all. They were reaching the interpreter fallback
     * instead, which flushes every GPR, every FPR and CR, spills the
     * downcount, materialises a 64-bit callee address and calls out, all to
     * run an empty function.
     *
     * That was not a corner case. Measured in a race, dcbi and dcbf were the
     * two largest EXECUTED fallbacks in the machine -- 723,725 and 582,624 in
     * one interval -- because the title's DCFlushRange/DCInvalidateRange walk
     * a cache line at a time in a tight bdnz loop. Roughly 1.3 million
     * fallbacks an interval, each costing 50-100 instructions, to do nothing.
     *
     * icbi is a no-op here on the interpreter's authority, and the compiled
     * form must agree with it or the two diverge; code invalidation is not
     * driven from this instruction. */
    case 54: case 86: case 246: case 278: case 470: case 982:
        return 1;

    case 266: {     /* add */
        int a = gpr_read(c, RA(op)), b = gpr_read(c, RB(op));
        int d = gpr_dest(c, RT(op));
        e_add(&c->e, (u32)d, (u32)a, (u32)b);
        maybe_rc(c, op, d);
        return 1;
    }
    case 40: {      /* subf: RT = RB - RA */
        int a = gpr_read(c, RA(op)), b = gpr_read(c, RB(op));
        int d = gpr_dest(c, RT(op));
        e_subf(&c->e, (u32)d, (u32)a, (u32)b);
        maybe_rc(c, op, d);
        return 1;
    }
    case 104: {     /* neg */
        int a = gpr_read(c, RA(op));
        int d = gpr_dest(c, RT(op));
        e_neg(&c->e, (u32)d, (u32)a);
        maybe_rc(c, op, d);
        return 1;
    }
    case 235: {     /* mullw */
        int a = gpr_read(c, RA(op)), b = gpr_read(c, RB(op));
        int d = gpr_dest(c, RT(op));
        e_mullw(&c->e, (u32)d, (u32)a, (u32)b);
        maybe_rc(c, op, d);
        return 1;
    }

    /* Multiply-high. The host is itself PowerPC, and its mulhw/mulhwu are
     * 32-bit instructions that read only the low words of their operands and
     * produce the high word of the 32x32 product -- identical to the guest's,
     * so they map one-to-one. These are how the compiler implements division by
     * a constant (the reciprocal-multiply trick), so real integer code is full
     * of them. */
    case 75: {      /* mulhw */
        int a = gpr_read(c, RA(op)), b = gpr_read(c, RB(op));
        int d = gpr_dest(c, RT(op));
        e_mulhw(&c->e, (u32)d, (u32)a, (u32)b);
        maybe_rc(c, op, d);
        return 1;
    }
    case 11: {      /* mulhwu */
        int a = gpr_read(c, RA(op)), b = gpr_read(c, RB(op));
        int d = gpr_dest(c, RT(op));
        e_mulhwu(&c->e, (u32)d, (u32)a, (u32)b);
        maybe_rc(c, op, d);
        return 1;
    }

    /* Add/subtract the carry to zero: the tail of the division-by-constant and
     * multi-word sequences the multiply-high forms above begin. Both consume
     * and produce XER[CA]; bit 32 of the 64-bit sum is the 32-bit carry out,
     * the same identity addc uses. */
    case 202: {     /* addze: RT = RA + CA */
        int a = gpr_read(c, RA(op));
        int d;
        emit_clean32(c, H_SCRATCH0, a);
        e_add(&c->e, H_SCRATCH0, H_SCRATCH0, (u32)carry_reg(c));
        e_extract_carry32(&c->e, H_CARRY, H_SCRATCH0);
        carry_produced(c);
        d = gpr_dest(c, RT(op));
        e_mr(&c->e, (u32)d, H_SCRATCH0);
        maybe_rc(c, op, d);
        return 1;
    }
    case 200: {     /* subfze: RT = ~RA + CA */
        int a = gpr_read(c, RA(op));
        int d;
        emit_clean32(c, H_SCRATCH0, a);
        e_xori(&c->e, H_SCRATCH0, H_SCRATCH0, 0xFFFF);
        e_xoris(&c->e, H_SCRATCH0, H_SCRATCH0, 0xFFFF);   /* ~RA in low 32 */
        e_clrldi(&c->e, H_SCRATCH0, H_SCRATCH0, 32);      /* keep it 32-bit */
        e_add(&c->e, H_SCRATCH0, H_SCRATCH0, (u32)carry_reg(c));
        e_extract_carry32(&c->e, H_CARRY, H_SCRATCH0);
        carry_produced(c);
        d = gpr_dest(c, RT(op));
        e_mr(&c->e, (u32)d, H_SCRATCH0);
        maybe_rc(c, op, d);
        return 1;
    }

    /* Logicals. Note the reversed operand direction: RA <- RS op RB. */
#define LOGIC(xoval, emitter)                                               \
    case xoval: {                                                           \
        int sreg = gpr_read(c, RS(op)), b = gpr_read(c, RB(op));            \
        int d = gpr_dest(c, RA(op));                                       \
        emitter(&c->e, (u32)d, (u32)sreg, (u32)b);                          \
        maybe_rc(c, op, d);                                                 \
        return 1;                                                           \
    }
    LOGIC(28,  e_and)
    LOGIC(60,  e_andc)
    LOGIC(476, e_nand)
    LOGIC(444, e_or)
    LOGIC(412, e_orc)
    LOGIC(124, e_nor)
    LOGIC(316, e_xor)
    LOGIC(284, e_eqv)
    LOGIC(24,  e_slw)
    LOGIC(536, e_srw)
#undef LOGIC

    /* Shift-right-algebraic writes XER[CA], and the host's own instruction
     * computes it correctly: `sraw`/`srawi` are 32-bit forms whose carry is
     * defined over the low 32 bits, which is exactly the guest's definition.
     * So the shift is one instruction and the carry costs two more to lift out
     * of the host XER -- against roughly thirty-five for the interpreter
     * fallback this replaces. */
    case 792: {     /* sraw  */
        int sreg = gpr_read(c, RS(op)), b = gpr_read(c, RB(op));
        int d = gpr_dest(c, RA(op));
        e_sraw(&c->e, (u32)d, (u32)sreg, (u32)b);
        emit_store_host_ca(c);
        maybe_rc(c, op, d);
        return 1;
    }
    case 824: {     /* srawi */
        int sreg = gpr_read(c, RS(op));
        int d = gpr_dest(c, RA(op));
        e_srawi(&c->e, (u32)d, (u32)sreg, SH(op));
        emit_store_host_ca(c);
        maybe_rc(c, op, d);
        return 1;
    }

    /* The carry-generating add/subtract group. See emit_carry_add. */
    case 10:  return emit_carry_add(c, op, 0, 0);   /* addc  */
    case 138: return emit_carry_add(c, op, 1, 0);   /* adde  */
    case 8:   return emit_carry_sub(c, op, 0);      /* subfc */
    case 136: return emit_carry_sub(c, op, 1);      /* subfe */

    /* mfspr, expanded from the original LR/CTR pair to every SPR whose read
     * is a plain PPCState load in the interpreter -- mirroring ppc_mfspr
     * (interp_system.c:38-66) case for case. The audit measured mfspr-other
     * at ~2.8M executed fallbacks per boot, almost all of them GQR reads
     * (the OS context switcher saves all eight) and SRR0/SRR1/SPRG reads in
     * the interrupt path.
     *
     * The SPR field is encoded with its two halves swapped. SPRN() is 10
     * bits, so every value is inside spr[SPR_COUNT] and the interpreter's
     * out-of-range warning branch (interp_system.c:59-62) is unreachable.
     * Three reads are not plain loads and keep their handler:
     *   XER        -- composed from xer_so/ov/ca/count (ppc_get_xer)
     *   TBL/TBU    -- the derived clock; a compact call below (48-49). */
    case 339: {     /* mfspr */
        u32 spr = SPRN(op);
        int d;

        if (spr == SPR_TBL_R || spr == SPR_TBU_R) {
            /* interp_system.c:48-49: v = tb_lower/tb_upper(s). One compact
             * call; the register caches stay live across it. */
            emit_compact_call(c, (spr == SPR_TBU_R)
                                     ? (const void *)jit_tb_upper_helper
                                     : (const void *)jit_tb_lower_helper);
            d = gpr_dest(c, RT(op));
            e_mr(&c->e, (u32)d, H_RET0);
            return 1;
        }
        if (spr == SPR_XER)
            return 0;                   /* interp_system.c:44, composed */

        d = gpr_dest(c, RT(op));
        if (spr == SPR_LR)              /* :45 */
            e_lwz(&c->e, (u32)d, (s32)offsetof(PPCState, lr), H_STATE);
        else if (spr == SPR_CTR)        /* :46 */
            e_lwz(&c->e, (u32)d, (s32)offsetof(PPCState, ctr), H_STATE);
        else if (spr == SPR_DEC)        /* :47 -- the stored value, untick'd */
            e_lwz(&c->e, (u32)d, (s32)offsetof(PPCState, dec), H_STATE);
        else if (spr == SPR_PVR)        /* :50 -- constant */
            e_load_imm32(&c->e, (u32)d, GEKKO_PVR);
        else if (spr == SPR_HID2)       /* :51 */
            e_lwz(&c->e, (u32)d, (s32)offsetof(PPCState, hid2), H_STATE);
        else if (spr >= SPR_GQR0 && spr <= SPR_GQR0 + 7)    /* :52-55 */
            e_lwz(&c->e, (u32)d,
                  (s32)(offsetof(PPCState, gqr) + 4 * (spr - SPR_GQR0)),
                  H_STATE);
        else                            /* :56-58 -- v = s->spr[n] */
            e_lwz(&c->e, (u32)d,
                  (s32)(offsetof(PPCState, spr) + 4 * spr), H_STATE);
        return 1;
    }

    /* mtspr, likewise expanded to every SPR whose write is a plain store in
     * ppc_mtspr (interp_system.c:68-123). The audit measured ~5.3M executed
     * mtspr fallbacks per boot: GQR writes (context switch, ~1.65M), SRR0/
     * SRR1 (0.9M), DEC (230K), SPRG0 (214K), HID0 (63K).
     *
     * Kept on the fallback:
     *   XER          -- decomposed into xer_* fields (ppc_set_xer, :74),
     *                   and it feeds the JIT's own carry cache
     *   TBL_W/TBU_W  -- rewrites tb_offset against the derived clock (:85-96)
     * PVR is read-only (:113-114): architecturally a no-op, zero code. */
    case 467: {     /* mtspr */
        u32 spr = SPRN(op);
        int sreg;

        if (spr == SPR_XER || spr == SPR_TBL_W || spr == SPR_TBU_W)
            return 0;
        if (spr == SPR_PVR)
            return 1;                   /* :113-114 -- no effect */

        sreg = gpr_read(c, RS(op));
        if (spr == SPR_LR) {            /* :76 */
            e_stw(&c->e, (u32)sreg, (s32)offsetof(PPCState, lr), H_STATE);
            /* The guest just chose its own return address; whatever this
             * unit knew about LR is void (jit.h, lr_known). trace_kills_lr
             * mirrors this test for the pre-scan. */
            c->lr_known = 0;
        }
        else if (spr == SPR_CTR)        /* :77 */
            e_stw(&c->e, (u32)sreg, (s32)offsetof(PPCState, ctr), H_STATE);
        else if (spr == SPR_DEC) {
            /* :79-83 -- s->dec = v; s->dec_write_tb = s->tb. The handler
             * samples the *slice-boundary* tb field, not the derived clock,
             * so this is a plain 64-bit field copy. */
            e_stw(&c->e, (u32)sreg, (s32)offsetof(PPCState, dec), H_STATE);
            e_ld(&c->e, H_SCRATCH1, (s32)offsetof(PPCState, tb), H_STATE);
            e_std(&c->e, H_SCRATCH1,
                  (s32)offsetof(PPCState, dec_write_tb), H_STATE);
        } else if (spr == SPR_HID2) {   /* :98-102 -- store + debug log only */
            e_stw(&c->e, (u32)sreg, (s32)offsetof(PPCState, hid2), H_STATE);
        } else if (spr >= SPR_GQR0 && spr <= SPR_GQR0 + 7) {
            /* :104-110 -- s->gqr[n] = v. The store itself is plain; the
             * JIT-visible consequence is that any psq_l/psq_st guard already
             * emitted in THIS block validated the old value, so drop the
             * once-per-block guard suppression and let a later quantized
             * access in the block re-check the register. (The runtime guard
             * always reads state->gqr, so blocks entered after this one are
             * already safe.) */
            e_stw(&c->e, (u32)sreg,
                  (s32)(offsetof(PPCState, gqr) + 4 * (spr - SPR_GQR0)),
                  H_STATE);
            c->gqr_guarded &= (u8)~(1u << (spr - SPR_GQR0));
        } else if (spr == SPR_DMAL) {
            /* Locked-cache DMA trigger: real work in the interpreter handler
             * (the THP video decoder moves every frame through it). A plain
             * store here silently discarded the transfer. */
            return 0;                   /* fall back to ppc_mtspr */
        } else {                        /* :116-121 -- s->spr[n] = v */
            e_stw(&c->e, (u32)sreg,
                  (s32)(offsetof(PPCState, spr) + 4 * spr), H_STATE);
        }
        return 1;
    }

    /* mfmsr: the interpreter handler is a single field read
     * (interp_system.c:164-167), so the native path is a single lwz --
     * against ~33 executed words plus the handler body for the fallback.
     * Measured at 6.3M executions per boot (OSDisableInterrupts reads it on
     * every critical section entry). */
    case 83: {      /* mfmsr */
        int d = gpr_dest(c, RT(op));
        e_lwz(&c->e, (u32)d, (s32)offsetof(PPCState, msr), H_STATE);
        return 1;
    }

    /* mtmsr, mirroring ppc_mtmsr (interp_system.c:169-175) exactly:
     *
     *     s->msr = s->gpr[RS];
     *     if ((s->msr & MSR_EE) && s->exceptions)
     *         ppc_request_exit(s);
     *
     * The store is unconditional. The EE-and-pending test is the rare path
     * (measured: 15.6K of 6.0M executions); when it fires, the inline code
     * replicates ppc_request_exit (interp_core.c:317-332) on the *live*
     * downcount register -- the same value the fallback spilled for the
     * handler to read -- and leaves the block through pc+4 charged with
     * guest_insts+1, exactly as the old fallback + emit_exit_if_requested
     * pair did. npc is stored for parity with the fallback's pre-call state.
     *
     * MSR[EE] disable/restore brackets every OS critical section, which is
     * why this was the single largest fallback in the audit (10.6M weighted;
     * 6.0M raw executions per boot). */
    case 146: {     /* mtmsr */
        int h = gpr_read(c, RS(op));
        PPCFixup no_ee, no_exc, no_slack;
        u8 sg[32], sf[64], sca;
        unsigned i;

        e_stw(&c->e, (u32)h, (s32)offsetof(PPCState, msr), H_STATE);

        /* The tests below use host cr7, which the guest CR mirror also
         * occupies -- same discipline as emit_mmio_guard. */
        cr_store_via(c, H_SCRATCH3);
        cr_invalidate(c);

        e_rlwinm(&c->e, H_SCRATCH1, (u32)h, 17, 31, 31);    /* MSR[EE] */
        e_cmpwi(&c->e, H_CR_JIT, H_SCRATCH1, 0);
        no_ee = e_bc_fwd(&c->e, BO_TRUE, BI_EQ(H_CR_JIT));
        e_lwz(&c->e, H_SCRATCH1, (s32)offsetof(PPCState, exceptions), H_STATE);
        e_cmpwi(&c->e, H_CR_JIT, H_SCRATCH1, 0);
        no_exc = e_bc_fwd(&c->e, BO_TRUE, BI_EQ(H_CR_JIT));

        /* Rare path: interrupts enabled with one pending. Flush, request
         * the exit, and leave through pc+4. The flushes clear the dirty
         * flags, which the fall-through path still needs -- the same
         * save/restore dance as emit_exit_if_requested. */
        for (i = 0; i < 32; i++) sg[i] = c->gpr[i].dirty;
        for (i = 0; i < 64; i++) sf[i] = c->fpr[i].dirty;
        sca = c->carry_dirty;

        rc_flush_all(c);
        fpr_flush_all(c);

        /* ppc_request_exit: if (downcount > 0) { exit_slack = downcount+1;
         * downcount = -1; } exit_requested = 1. On the live register. */
        e_cmpwi(&c->e, H_CR_JIT, H_DOWNCOUNT, 0);
        no_slack = e_bc_fwd(&c->e, BO_FALSE, BI_GT(H_CR_JIT));
        e_addi(&c->e, H_SCRATCH1, H_DOWNCOUNT, 1);
        e_stw(&c->e, H_SCRATCH1, (s32)offsetof(PPCState, exit_slack), H_STATE);
        e_li(&c->e, H_DOWNCOUNT, -1);
        e_patch_here(&c->e, no_slack);
        e_li(&c->e, H_SCRATCH1, 1);
        e_stw(&c->e, H_SCRATCH1,
              (s32)offsetof(PPCState, exit_requested), H_STATE);
        e_load_imm32_lo(&c->e, H_SCRATCH1, c->pc + 4);
        e_stw(&c->e, H_SCRATCH1, (s32)offsetof(PPCState, npc), H_STATE);
        emit_exit_reg_tail(c, H_SCRATCH1, c->guest_insts + 1);

        for (i = 0; i < 32; i++) c->gpr[i].dirty = sg[i];
        for (i = 0; i < 64; i++) c->fpr[i].dirty = sf[i];
        c->carry_dirty = sca;

        e_patch_here(&c->e, no_ee);
        e_patch_here(&c->e, no_exc);
        return 1;
    }

    /* mftb / mftbu, mirroring ppc_mftb (interp_system.c:126-130): TBU when
     * the swapped SPR field names it, TBL for every other value. The time
     * base is *derived* (scheduler clock + tb_offset), so a C call remains
     * -- but a compact one: the register caches live in ABI-preserved host
     * registers and survive it, where the fallback's full spill-and-
     * invalidate forced a reload of everything it had cached. 2.4M
     * executions per boot. */
    case 371: {     /* mftb */
        int d;
        emit_compact_call(c, (SPRN(op) == SPR_TBU_R)
                                 ? (const void *)jit_tb_upper_helper
                                 : (const void *)jit_tb_lower_helper);
        d = gpr_dest(c, RT(op));
        e_mr(&c->e, (u32)d, H_RET0);
        return 1;
    }

    /* dcbz, mirroring ppc_dcbz (interp_loadstore.c:194-200): zero the
     * 32-byte guest cache line at (EA & ~31). The guest line is 32 bytes
     * and the host PPE's is 128, so the host's own dcbz must NOT be used
     * (docs/HARDWARE.md §5.4); four zero doubleword stores through the
     * arena replace the interpreter's mem_write_block. The EA is formed and
     * MMIO-guarded exactly like every other indexed access; the fold and
     * the 32-byte alignment collapse into one rlwinm (mask 0x3FFFFFE0).
     * 2.4M executions per boot -- memset/allocator clears sit on it. */
    case 1014: {    /* dcbz */
        u32 ra = RA(op), rb = RB(op);
        int hb = gpr_read(c, rb);

        if (ra == 0)
            e_mr(&c->e, H_SCRATCH0, (u32)hb);
        else
            e_add(&c->e, H_SCRATCH0, (u32)gpr_read(c, ra), (u32)hb);
        emit_mmio_guard(c, H_SCRATCH0, c->pc);
        e_rlwinm(&c->e, H_SCRATCH0, H_SCRATCH0, 0, 2, 26); /* fold + align */
        e_add(&c->e, H_SCRATCH0, H_MEMBASE, H_SCRATCH0);
        e_li(&c->e, H_SCRATCH1, 0);
        e_std(&c->e, H_SCRATCH1, 0,  H_SCRATCH0);
        e_std(&c->e, H_SCRATCH1, 8,  H_SCRATCH0);
        e_std(&c->e, H_SCRATCH1, 16, H_SCRATCH0);
        e_std(&c->e, H_SCRATCH1, 24, H_SCRATCH0);
        return 1;
    }

    case 954: {     /* extsb */
        int sreg = gpr_read(c, RS(op));
        int d = gpr_dest(c, RA(op));
        e_extsb(&c->e, (u32)d, (u32)sreg);
        maybe_rc(c, op, d);
        return 1;
    }
    case 922: {     /* extsh */
        int sreg = gpr_read(c, RS(op));
        int d = gpr_dest(c, RA(op));
        e_extsh(&c->e, (u32)d, (u32)sreg);
        maybe_rc(c, op, d);
        return 1;
    }
    case 26: {      /* cntlzw */
        int sreg = gpr_read(c, RS(op));
        int d = gpr_dest(c, RA(op));
        e_cntlzw(&c->e, (u32)d, (u32)sreg);
        maybe_rc(c, op, d);
        return 1;
    }

    case 0:         /* cmp  */
    case 32: {      /* cmpl */
        int a = gpr_read(c, RA(op)), b = gpr_read(c, RB(op));
        cr_touch(c);
        if (xo == 0) e_cmpw (&c->e, CRFD(op), (u32)a, (u32)b);
        else         e_cmplw(&c->e, CRFD(op), (u32)a, (u32)b);
        return 1;
    }

    /* Indexed loads and stores: fold, then one indexed access. */
#define LOAD_X(xoval, emitter)                                              \
    case xoval: {                                                           \
        int ea = emit_ea_x(c, op, H_SCRATCH0);                              \
        int d  = gpr_dest(c, RT(op));                                      \
        emitter(&c->e, (u32)d, H_MEMBASE, (u32)ea);                         \
        return 1;                                                           \
    }
    LOAD_X(23,  e_lwzx)
    LOAD_X(87,  e_lbzx)
    LOAD_X(279, e_lhzx)
    LOAD_X(343, e_lhax)
#undef LOAD_X

#define STORE_X(xoval, emitter)                                             \
    case xoval: {                                                           \
        int ea = emit_ea_x(c, op, H_SCRATCH0);                              \
        int sreg = gpr_read(c, RS(op));                                     \
        emitter(&c->e, (u32)sreg, H_MEMBASE, (u32)ea);                      \
        return 1;                                                           \
    }
    STORE_X(151, e_stwx)
    STORE_X(215, e_stbx)
    STORE_X(407, e_sthx)
#undef STORE_X

    /* Indexed floating-point accesses. These are the same one-instruction
     * accesses as their d-form counterparts -- the single<->double conversion
     * is the hardware instruction here -- and they are common enough in the
     * title's code to be worth compiling: stfsx alone was the single largest
     * remaining fallback. lfsx fills both paired-single halves, exactly as lfs
     * does on Gekko. */
    case 535: {     /* lfsx */
        int ea = emit_ea_x(c, op, H_SCRATCH0);
        int d  = fpr_write(c, FRT(op), PS0);
        e_lfsx(&c->e, (u32)d, H_MEMBASE, (u32)ea);
        {
            int d1 = fpr_write(c, FRT(op), PS1);
            e_fmr(&c->e, (u32)d1, (u32)d);
            fpr_mark_both(c, FRT(op));
        }
        return 1;
    }
    case 599: {     /* lfdx -- ps0 only */
        int ea = emit_ea_x(c, op, H_SCRATCH0);
        int d  = fpr_write(c, FRT(op), PS0);
        e_lfdx(&c->e, (u32)d, H_MEMBASE, (u32)ea);
        return 1;
    }
    case 663: {     /* stfsx */
        int ea = emit_ea_x(c, op, H_SCRATCH0);
        int sreg = fpr_read(c, FRT(op), PS0);
        e_stfsx(&c->e, (u32)sreg, H_MEMBASE, (u32)ea);
        return 1;
    }
    case 727: {     /* stfdx */
        int ea = emit_ea_x(c, op, H_SCRATCH0);
        int sreg = fpr_read(c, FRT(op), PS0);
        e_stfdx(&c->e, (u32)sreg, H_MEMBASE, (u32)ea);
        return 1;
    }

    default:
        return 0;
    }
}

/* Returns 1 if a native path was emitted, 0 to fall back. */
/* Software prefetch for guest loads inside a retained loop.
 *
 * The console says the recompiler is not issue-bound: a menu runs at 1.0
 * cycles per guest instruction, an in-race frame at ~35, with only 4x more
 * compiled code between them. The static PPE issue model puts the hot path at
 * 1.64 cycles per guest instruction, so ~95% of in-race time is the guest's
 * data missing a 512 KB L2 into memory several hundred cycles away. Nothing on
 * the issue side can touch that -- turning the instruction scheduler off on
 * hardware measured very slightly FASTER.
 *
 * `dcbt` is the documented lever: Cell puts dcbt-driven streaming at 5.8 GB/s
 * against 1.0 GB/s for demand loads. It is a hint with no architectural
 * effect, so unlike a codegen change it cannot produce wrong results -- the
 * only risk is the scratch register, and H_SCRATCH0 is dead once
 * emit_addr_d has returned its folded base.
 *
 * Restricted to retained loops because that is where strided access lives and
 * where two extra words per load can be amortised; scattered loads would pay
 * the issue cost for nothing. Distance is one 128-byte line ahead by default,
 * which suits the walk-an-array shape without running far off the end. */
static void emit_prefetch(JitContext *c, int base)
{
    if (!g_jit_prefetch)
        return;
    /* Mode 1 is loop-only, which reaches almost nothing: measured over 20,000
     * real blocks it put a dcbt in 17 of them. That is not a test of whether
     * prefetching helps, it is a test of how rare retained loops are. Mode 2
     * prefetches every d-form load, which costs two words per load (~36% more
     * hot code) and is far too blunt to ship -- but issue cost is only ~5% of
     * the frame, so paying 36% of 5% to find out whether the other 95% is
     * recoverable is a cheap ceiling experiment. */
    if (g_jit_prefetch == 1 && !(c->retaining || c->warm_active))
        return;
    if (g_jit_prefetch == 3) {
        /* Discriminating probe. One instruction, no scratch register touched,
         * and it prefetches the very line the load is about to read -- so it
         * can neither corrupt state nor fetch anything the access would not
         * have fetched a cycle later. If mode 3 also collapses performance the
         * cause is `dcbt` itself on this core; if it does not, mode 2's
         * regression was the H_SCRATCH0 clobber, not the hardware. */
        e_dcbt(&c->e, 0, (u32)base);
        return;
    }
    /* H_SCRATCH1, not H_SCRATCH0.
     *
     * H_SCRATCH0 still holds the access address at this point -- emit_mmio_guard
     * says so in as many words ("not H_SCRATCH0: it holds the address"), and it
     * is what the guard's bail-out path depends on. Writing it here cost a 7x
     * slowdown on the console, which was originally, and wrongly, written up as
     * a D-ERAT property of the hardware. The scratch-free probe (mode 3) ran at
     * full speed and settled it: `dcbt` is nearly free here, the clobber was
     * mine. Same source states H_SCRATCH1/2 are dead at this point. */
    e_addi(&c->e, H_SCRATCH1, (u32)base, (s32)g_jit_prefetch_dist);
    e_dcbt(&c->e, 0, H_SCRATCH1);
}

static int compile_one(JitContext *c, u32 op)
{
    switch (OPCD(op)) {

    case 19: {      /* condition-register logic, and mcrf */
        /* The guest CR is mirrored into the whole host CR, so these are
         * literally the same instruction on both sides, with the same bit
         * numbering. They were reaching the interpreter fallback: `cror`
         * alone was 437,668 EXECUTED fallbacks in a racing interval, the
         * largest single decline left after the cache no-ops.
         *
         * compile_branch also has a `case 19`, but it only ever sees the
         * branch forms -- it is called from the terminator path. Putting
         * these there compiled nothing at all, which the fallback counter
         * said plainly: op19 did not move.
         *
         * cr_touch mirrors the register in if it is not already there and
         * marks it dirty so the exits spill it. The guard-field picker
         * already counts these as touching the fields holding BD, BA and BB
         * (cr_fields_touched, case 19), so it will not hand a guard a field
         * one of them writes. */
        u32 xo = XO10(op);
        switch (xo) {
        case 257: case 449: case 193: case 225:
        case 33:  case 289: case 129: case 417: {
            u32 bt = CRBD(op), ba = CRBA(op), bb = CRBB(op);
            cr_touch(c);
            switch (xo) {
            case 257: e_crand (&c->e, bt, ba, bb); break;
            case 449: e_cror  (&c->e, bt, ba, bb); break;
            case 193: e_crxor (&c->e, bt, ba, bb); break;
            case 225: e_crnand(&c->e, bt, ba, bb); break;
            case 33:  e_crnor (&c->e, bt, ba, bb); break;
            case 289: e_creqv (&c->e, bt, ba, bb); break;
            case 129: e_crandc(&c->e, bt, ba, bb); break;
            case 417: e_crorc (&c->e, bt, ba, bb); break;
            }
            return 1;
        }
        case 0:                                 /* mcrf */
            cr_touch(c);
            e_mcrf(&c->e, CRFD(op), CRFS(op));
            return 1;
        case 150:                               /* isync */
            /* The interpreter treats it as a no-op; compiled code must agree
             * or the two diverge. */
            return 1;
        default:
            return 0;                           /* rfi and friends */
        }
    }

    case 14: {      /* addi (and li when RA=0) */
        /* The source is read before the destination is allocated, so that
         * gpr_dest can skip the load of a value about to be overwritten. */
        int a = RA(op) ? gpr_read(c, RA(op)) : -1;
        int d = gpr_dest(c, RT(op));
        if (a < 0)
            /* One host instruction, exactly the guest's own encoding. A
             * negative immediate leaves lis/addi sign-extension in the upper
             * half -- which the 32-in-64 invariant explicitly permits: only
             * the low word of a cached guest register is meaningful, and the
             * consumers that need clean inputs (carry forms, address folds)
             * already clean them. Materializing a zero-extended constant here
             * cost three instructions for every `li rN, -1`. */
            e_li(&c->e, (u32)d, SIMM(op));
        else
            e_addi(&c->e, (u32)d, (u32)a, SIMM(op));
        return 1;
    }
    case 15: {      /* addis (and lis) */
        int a = RA(op) ? gpr_read(c, RA(op)) : -1;
        int d = gpr_dest(c, RT(op));
        (void)a;
        if (RA(op) == 0)
            /* `lis rN, 0x8034` -- how the guest builds every MEM1/MEM2
             * address -- is one instruction, not lis+rldicl. Same invariant
             * argument as `li` above. */
            e_lis(&c->e, (u32)d, (s32)(s16)UIMM(op));
        else
            e_addis(&c->e, (u32)d, (u32)gpr_read(c, RA(op)), (s32)(s16)UIMM(op));
        return 1;
    }

#define IMM_LOGIC(opc, emitter, shifted)                                    \
    case opc: {                                                             \
        int sreg = gpr_read(c, RS(op));                                     \
        int d = gpr_dest(c, RA(op));                                       \
        emitter(&c->e, (u32)d, (u32)sreg, UIMM(op));                        \
        (void)shifted;                                                      \
        return 1;                                                           \
    }
    IMM_LOGIC(24, e_ori,   0)
    IMM_LOGIC(25, e_oris,  1)
    IMM_LOGIC(26, e_xori,  0)
    IMM_LOGIC(27, e_xoris, 1)
#undef IMM_LOGIC

    case 28: {      /* andi. -- always sets CR0 */
        int sreg = gpr_read(c, RS(op));
        int d = gpr_dest(c, RA(op));
        cr_touch(c);
        /* andi. is microcoded (>=11 cycles, both threads stall) and the ISA
         * has no non-recording andi. Build it from an immediate load, a
         * non-recording `and`, and a compare: three instructions, ~5 cycles,
         * none microcoded. */
        e_load_imm32(&c->e, H_SCRATCH0, UIMM(op));
        e_and(&c->e, (u32)d, (u32)sreg, H_SCRATCH0);
        e_cmpwi(&c->e, 0, (u32)d, 0);
        return 1;
    }
    case 29: {      /* andis. */
        int sreg = gpr_read(c, RS(op));
        int d = gpr_dest(c, RA(op));
        cr_touch(c);
        /* andis. can set bit 31 (immediate bit 15), and the native record
         * form compares the 64-bit zero-extended result -- same class of bug
         * as rlwinm.; the cmpwi overwrites CR0 with the 32-bit answer. */
        e_andis_(&c->e, (u32)d, (u32)sreg, UIMM(op));
        e_cmpwi(&c->e, 0, (u32)d, 0);
        return 1;
    }

    case 11: {      /* cmpi */
        int a = gpr_read(c, RA(op));
        cr_touch(c);
        e_cmpwi(&c->e, CRFD(op), (u32)a, SIMM(op));
        return 1;
    }
    case 10: {      /* cmpli */
        int a = gpr_read(c, RA(op));
        cr_touch(c);
        e_cmplwi(&c->e, CRFD(op), (u32)a, UIMM(op));
        return 1;
    }

    case 21: {      /* rlwinm */
        int sreg = gpr_read(c, RS(op));
        int d = gpr_dest(c, RA(op));
        /* NOT the native record form. On a 64-bit host `rlwinm.` compares
         * the ZERO-EXTENDED 64-bit result, which is never negative -- CR0
         * came out GT for every nonzero result whose bit 31 the guest
         * expects to mean LT. The differential fuzzer caught it; the THP
         * video decoder had been silently miscompiled by it. */
        e_rlwinm(&c->e, (u32)d, (u32)sreg, SH(op), MB(op), ME(op));
        maybe_rc(c, op, d);
        return 1;
    }
    case 23: {      /* rlwnm */
        int sreg = gpr_read(c, RS(op)), b = gpr_read(c, RB(op));
        int d = gpr_dest(c, RA(op));
        e_rlwnm(&c->e, (u32)d, (u32)sreg, (u32)b, MB(op), ME(op));
        maybe_rc(c, op, d);
        return 1;
    }
    case 20: {      /* rlwimi: reads RA as well as writing it */
        int sreg = gpr_read(c, RS(op));
        int d = gpr_read(c, RA(op));
        c->gpr[RA(op)].dirty = 1;
        /* Written without going through gpr_write, so both of the things a
         * GPR write must kill have to be killed by hand. Dropping the
         * address-base slot is not optional: emit_addr_d caches
         * MEMBASE + (RA & ARENA_MASK) in a dedicated host register, and that
         * fold is of RA's OLD value. A guest that patches a pointer's low
         * bits with rlwimi and then accesses through it -- which is exactly
         * what rlwimi is for -- would otherwise load or STORE at the stale
         * address. */
        addr_base_drop(c, RA(op));
        prov_kill(c, RA(op));
        e_rlwimi(&c->e, (u32)d, (u32)sreg, SH(op), MB(op), ME(op));
        maybe_rc(c, op, d);
        return 1;
    }

    /* d-form loads and stores. */
#define LOAD_D(opc, emitter)                                                \
    case opc: {                                                             \
        s32 disp;                                                           \
        int base = emit_addr_d(c, op, &disp);                               \
        int d;                                                              \
        emit_prefetch(c, base);                                             \
        d = gpr_dest(c, RT(op));                                            \
        emitter(&c->e, (u32)d, disp, (u32)base);                            \
        return 1;                                                           \
    }
    LOAD_D(32, e_lwz)
    LOAD_D(34, e_lbz)
    LOAD_D(40, e_lhz)
    LOAD_D(42, e_lha)
#undef LOAD_D

#define STORE_D(opc, emitter)                                               \
    case opc: {                                                             \
        s32 disp;                                                           \
        int base = emit_addr_d(c, op, &disp);                               \
        int sreg = gpr_read(c, RS(op));                                     \
        emitter(&c->e, (u32)sreg, disp, (u32)base);                         \
        return 1;                                                           \
    }
    STORE_D(36, e_stw)
    STORE_D(38, e_stb)
    STORE_D(44, e_sth)
#undef STORE_D

    /* Update forms. These are not a rare corner: `stwu r1,-N(r1)` is the
     * standard PowerPC function prologue, so every non-leaf function in a game
     * starts with one, and the update loads are how the compiler walks arrays.
     * Leaving them to the interpreter truncated a block at every function
     * entry and every pointer-walking loop.
     *
     * The address is formed exactly as for the non-update form -- through the
     * same MMIO-guarded base -- and the writeback is a separate 32-bit add on
     * the guest register. gpr_write drops the cached address base, which was
     * derived from RA's old value and is stale the moment RA changes. */
#define LOAD_DU(opc, emitter)                                               \
    case opc: {                                                             \
        s32 disp;                                                           \
        int base, d;                                                        \
        if (RA(op) == 0 || RA(op) == RT(op))                                \
            return 0;               /* invalid form: leave it alone */      \
        base = emit_addr_d(c, op, &disp);                                   \
        d = gpr_dest(c, RT(op));                                           \
        emitter(&c->e, (u32)d, disp, (u32)base);                            \
        emit_ra_update(c, op, disp);                                        \
        return 1;                                                           \
    }
    LOAD_DU(33, e_lwz)
    LOAD_DU(35, e_lbz)
    LOAD_DU(41, e_lhz)
    LOAD_DU(43, e_lha)
#undef LOAD_DU

#define STORE_DU(opc, emitter)                                              \
    case opc: {                                                             \
        s32 disp;                                                           \
        int base, sreg;                                                     \
        if (RA(op) == 0)                                                    \
            return 0;               /* invalid form: leave it alone */      \
        base = emit_addr_d(c, op, &disp);                                   \
        sreg = gpr_read(c, RS(op));                                         \
        emitter(&c->e, (u32)sreg, disp, (u32)base);                         \
        emit_ra_update(c, op, disp);                                        \
        return 1;                                                           \
    }
    STORE_DU(37, e_stw)
    STORE_DU(39, e_stb)
    STORE_DU(45, e_sth)
#undef STORE_DU

    case 7: {       /* mulli */
        int a = gpr_read(c, RA(op));
        int d = gpr_dest(c, RT(op));
        /* The low 32 bits of a product depend only on the low 32 bits of the
         * operands, so the host's 64-bit multiply needs no cleaning. */
        e_mulli(&c->e, (u32)d, (u32)a, SIMM(op));
        return 1;
    }

    case 8: {       /* subfic */
        int a = gpr_read(c, RA(op));
        int d;
        /* RT = SIMM - RA, and CA is "no borrow", which for the 32-bit
         * operation is exactly (u32)SIMM >= (u32)RA. Working in 64 bits with
         * both operands zero-extended makes the difference exact, so the
         * borrow is just whether it went negative. */
        emit_clean32(c, H_SCRATCH0, a);
        e_load_imm32(&c->e, H_SCRATCH1, (u32)SIMM(op));
        e_zext32(&c->e, H_SCRATCH1, H_SCRATCH1);
        e_subf(&c->e, H_SCRATCH0, H_SCRATCH0, H_SCRATCH1);
        e_rldicl(&c->e, H_CARRY, H_SCRATCH0, 1, 63);    /* sign bit */
        e_xori(&c->e, H_CARRY, H_CARRY, 1);             /* no borrow */
        carry_produced(c);
        d = gpr_dest(c, RT(op));
        e_mr(&c->e, (u32)d, H_SCRATCH0);
        return 1;
    }

    case 12:        /* addic  */
    case 13: {      /* addic. -- the dot is in the opcode, not an Rc bit */
        int a = gpr_read(c, RA(op));
        s32 imm = SIMM(op);
        int d;

        emit_clean32(c, H_SCRATCH0, a);
        e_addi(&c->e, H_SCRATCH0, H_SCRATCH0, imm);
        if (imm >= 0) {
            /* Both operands are non-negative and the sum is below 2^33, so
             * bit 32 is the 32-bit carry out -- the same identity addc uses. */
            e_extract_carry32(&c->e, H_CARRY, H_SCRATCH0);
        } else {
            /* A negative immediate is a subtraction, whose carry is "no
             * borrow". The 64-bit sum is exact, so that is simply whether it
             * stayed non-negative: take the sign bit and invert it. */
            e_rldicl(&c->e, H_CARRY, H_SCRATCH0, 1, 63);
            e_xori(&c->e, H_CARRY, H_CARRY, 1);
        }
        carry_produced(c);

        d = gpr_dest(c, RT(op));
        e_mr(&c->e, (u32)d, H_SCRATCH0);
        if (OPCD(op) == 13) {
            cr_touch(c);
            e_cmpwi(&c->e, 0, (u32)d, 0);
        }
        return 1;
    }

    case 4:
        /* Paired singles are only architecturally present when HID2[PSE] is
         * set. The block key includes that bit, so a block compiled with them
         * enabled can never be reused with them disabled. */
        if (!(c->state->hid2 & HID2_PSE))
            return 0;
        return compile_paired(c, op);

    case 48: case 50: case 52: case 54:
    case 49: case 51: case 53: case 55:     /* update forms */
        return compile_fp_loadstore(c, op);

    case 56: case 57: case 60: case 61:
        return compile_psq(c, op);

    case 59:
        return compile_fp_arith(c, op, 1);

    case 63:
        /* Opcode 63 carries both A-form (5-bit XO) arithmetic and X-form
         * (10-bit XO) moves. The two encodings provably do not collide -- no
         * X-form XO has an A-form value in its low five bits -- so trying
         * arithmetic first and moves second is unambiguous. The compares are
         * X-form XOs 0 and 32; both have A-form low bits 0, which no A-form
         * arithmetic uses, so testing them first is equally unambiguous. */
        if (XO10(op) == 0 || XO10(op) == 32)
            return compile_fcmp(c, op);
        if (compile_fp_arith(c, op, 0))
            return 1;
        return compile_fp_move(c, op);

    case 31:
        return compile_op31(c, op);

    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Warm self-loop continuity                                            */
/*                                                                      */
/* Loop retention (further down) keeps registers live across a self-loop's     */
/* back edge, but only for a whitelisted body: no carry, no Rc, no FP, no      */
/* interior branches, no fallbacks. The loops that whitelist rejects --        */
/* __fill_mem's addic.-counted word loop, memcpy's lbzu/stbu byte loop,        */
/* decodeSZS's outer state machine -- are exactly the audit's giant: their     */
/* back edge is a patched link to the block's own cold entry, so every         */
/* iteration re-loads every register it uses and writes every one back.       */
/*                                                                             */
/* This section removes that per-iteration traffic for ANY body, using two     */
/* entry points instead of a whitelist:                                        */
/*                                                                             */
/*   cold entry (block->code): the dispatcher/link contract is unchanged --    */
/*     memory is authoritative. A short prologue loads the loop's registers    */
/*     and falls into the body.                                                */
/*   warm entry (just past the prologue): reachable ONLY from this block's     */
/*     own back edge, where the register state equals the entry state by       */
/*     construction -- no cross-block mapping agreement is ever needed.        */
/*                                                                             */
/* Which registers, and in which host slots? The block itself already knows:   */
/* a probe compile (the ordinary pass) is run first, and the cache mapping     */
/* observed at its back edge -- guest register, host slot, and whether the     */
/* body writes it -- is recorded verbatim as the warm contract. The recompile  */
/* preloads exactly those pairs, pinned, so the body reproduces the mapping.   */
/*                                                                             */
/* Dirtiness is the load-bearing subtlety. The body is compiled ONCE, but on   */
/* iterations past the first its code runs with registers carrying values      */
/* memory does not have. Every interior sync point -- a guard's cold escape,   */
/* an exception check -- spills the set that is compile-time dirty AT THAT     */
/* POINT, so a register dirtied later in program order than the sync point     */
/* would be spilled as clean while runtime-dirty from the previous iteration.  */
/* The fix is the entry convention: every register the body writes ANYWHERE    */
/* is marked dirty from the block's first instruction. Then every snapshot,    */
/* every escape and both loop exits already include the full loop-carried      */
/* set; on the first iteration the extra writebacks store the values just      */
/* loaded -- redundant, never wrong. Registers the body only reads stay        */
/* clean: their memory copy is authoritative on every iteration.               */
/*                                                                             */
/* Everything OUTSIDE the warm set keeps the cold contract at the back edge:   */
/* dirty stragglers are written back, carry/CR/FPRs spilled, so the next       */
/* iteration's lazy reloads (compiled against the empty entry state) read      */
/* authoritative memory. The back edge charges the cycle budget and tests it   */
/* exactly as a linked exit does; on exhaustion it spills the warm set and     */
/* yields to the dispatcher at the loop top, whose cold prologue rebuilds      */
/* everything next slice.                                                      */
/*                                                                             */
/* Degradation is always available and always correct: a fallback wipes the   */
/* cache mid-body, the back edge notices the mapping no longer matches the    */
/* contract, and emits the ordinary linked exit instead. An AOT-owned loop    */
/* head refuses warm treatment for the same reason link_or_defer refuses to   */
/* patch one: the dispatcher must stay in the loop.                           */
/* ------------------------------------------------------------------ */

static int warm_member_index(const JitContext *c, u32 g)
{
    unsigned i;
    for (i = 0; i < c->warm_count; i++)
        if (c->warm_guest[i] == g)
            return (int)i;
    return -1;
}

/* The back edge may stay warm only if the compile-time cache state still
 * matches the recorded contract exactly: every preloaded register in its
 * prologue slot, current, with the dirtiness the body was compiled under. */
static int warm_state_matches(const JitContext *c)
{
    unsigned i;
    for (i = 0; i < c->warm_count; i++) {
        const GprSlot *sl = &c->gpr[c->warm_guest[i]];
        if (sl->host != (s8)c->warm_hostr[i] || !sl->loaded ||
            (sl->dirty != 0) != (c->warm_dirtyf[i] != 0))
            return 0;
    }
    return 1;
}

/* The back edge is a `bc`, and its displacement field is fourteen bits --
 * +-32 KiB. Retention's whitelisted bodies could never approach that; a
 * general body has no such bound (a block with more guarded accesses than
 * the deferred-cold budget keeps its bail-outs INLINE, and each of those is
 * a full spill of every live register). An out-of-range displacement is not
 * diagnosed by the emitter -- e_bc masks it into the 14-bit field -- so it
 * would become a silent forward branch inside the block. Measure the distance
 * instead and decline warm treatment when it will not fit, which costs the
 * optimization and nothing else. The margin covers the words
 * emit_warm_backedge itself emits ahead of the branch: at most one store per
 * cached GPR and FPR plus the carry, CR and counter sequences, ~110. */
#define JIT_WARM_MAX_BACK_WORDS 7900u

static int warm_backedge_in_range(const JitContext *c)
{
    u32 here = (u32)(emit_mark(&c->e) - c->e.base);
    return here >= c->warm_off &&
           (here - c->warm_off) < JIT_WARM_MAX_BACK_WORDS;
}

/* Probe-pass discovery, called at a conditional self-loop back edge before
 * the generic path's flush clears the dirty flags: record the mapping the
 * block naturally chose as the warm entry contract for the recompile. */
static void warm_capture(JitContext *c)
{
    unsigned g, n = 0;

    if (c->retaining || c->want_warm || c->had_fallback || c->warm_candidate)
        return;
    if (jit_aot_owns_pc(c->start_pc))
        return;             /* mirrors link_or_defer's AOT refusal */

    /* Bodies with interior branch exits are refused: when an inlined forward
     * branch leaves the block mid-body -- which profiling shows is the
     * DOMINANT exit for state-machine loops like decodeSZS's outer loop --
     * the back edge is cold, and warm treatment inverts into a loss twice
     * over: the eager prologue loads registers the short path never touches,
     * and the dirty-from-entry convention makes every interior exit write
     * the whole loop-carried set back where the plain block wrote only what
     * that execution dirtied. Straight-line bodies (every memcpy/__fill_mem
     * shape, and every loop retention already declines for carry/Rc reasons)
     * run the body in full on every pass, so neither penalty exists. MMIO
     * bail-outs stay eligible -- they are never executed. */
    {   int had_cold = 0;
        for (g = 0; g < c->cold_count; g++)
            if (c->cold[g].kind == COLD_BRANCH) { had_cold = 1; break; }
        if (had_cold) {
            if (g_warm_no_cold)
                return;
            g_warm_had_cold++;
        }
    }

    for (g = 0; g < 32 && n < H_GPRCACHE_COUNT; g++) {
        if (c->gpr[g].host < 0 || !c->gpr[g].loaded)
            continue;
        c->warm_guest[n]  = (u8)g;
        c->warm_hostr[n]  = (u8)c->gpr[g].host;
        c->warm_dirtyf[n] = c->gpr[g].dirty;
        n++;
    }
    /* Leave at least two cache registers unpinned for the recompiled body's
     * transients; with everything pinned the first eviction would abort the
     * warm pass and waste the compile. Drop the least recently used. */
    while (n > (unsigned)(H_GPRCACHE_COUNT - 2)) {
        unsigned i, victim = 0;
        u32 oldest = 0xFFFFFFFFu;
        for (i = 0; i < n; i++) {
            u32 age = c->gpr[c->warm_guest[i]].lru;
            if (age < oldest) { oldest = age; victim = i; }
        }
        c->warm_guest[victim]  = c->warm_guest[n - 1];
        c->warm_hostr[victim]  = c->warm_hostr[n - 1];
        c->warm_dirtyf[victim] = c->warm_dirtyf[n - 1];
        n--;
    }
    if (n == 0)
        return;
    c->warm_count = (u8)n;
    c->warm_candidate = 1;
    g_warm_captured++;
}

/* Emit the warm back edge for a conditional self-loop terminator (CTR form
 * or CR form, per the op). The warm set stays in registers; everything else
 * is made memory-authoritative first. Both cold outcomes -- loop finished,
 * budget expired -- leave through ordinary exits whose flush writes the warm
 * set back (it is compile-time dirty exactly where the body writes it). */
static int emit_warm_backedge(JitContext *c, u32 op)
{
    u32 bo = BO(op);
    PPCFixup loop_done;
    u8 sgpr[32], sfpr[64];
    unsigned i;
    u32 here;
    s32 disp;
    int ctr_kind = !(bo & 0x04);

    for (i = 0; i < 32; i++)
        if (c->gpr[i].host >= 0 && c->gpr[i].dirty &&
            warm_member_index(c, i) < 0)
            gpr_writeback(c, i);
    carry_flush(c);
    fpr_flush_all(c);
    if (ctr_kind) {
        cr_store(c);        /* body Rc/cmp results, if any */
        cr_invalidate(c);
        /* CTR stays memory-resident -- a general body may use it --
         * decremented in place exactly as the generic path does. */
        e_lwz(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, ctr), H_STATE);
        e_addi(&c->e, H_SCRATCH0, H_SCRATCH0, -1);
        e_stw(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, ctr), H_STATE);
        e_cmplwi(&c->e, H_CR_JIT, H_SCRATCH0, 0);
    } else {
        cr_ensure(c);       /* the tested bit must be live in the host CR */
        cr_store(c);        /* mfcr reads it without disturbing it */
    }

    /* Charge this iteration (body + branch) once; every exit below passes 0.
     * Identical accounting to a retained loop's back edge. */
    e_addi(&c->e, H_DOWNCOUNT, H_DOWNCOUNT, -(s32)(c->guest_insts + 1));

    /* Loop finished? bdnz is terminal on CTR==0 (EQ), bdz on CTR!=0; the CR
     * form ends when the guest branch is NOT taken (inverse sense). */
    if (ctr_kind)
        loop_done = e_bc_fwd(&c->e, (bo & 0x02) ? BO_FALSE : BO_TRUE,
                             BI_EQ(H_CR_JIT));
    else
        loop_done = e_bc_fwd(&c->e, (bo & 0x08) ? BO_FALSE : BO_TRUE, BI(op));
    WP_SKIP_BEGIN(c);

    /* Loop continues: back to the warm entry while the budget lasts -- the
     * same test every linked exit performs, and for the same reason. */
    wp_backedge(c, c->warm_off);
    e_cmpwi(&c->e, H_CR_JIT, H_DOWNCOUNT, 0);
    here = (u32)(emit_mark(&c->e) - c->e.base);
    disp = ((s32)c->warm_off - (s32)here) * 4;
    e_bc(&c->e, BO_TRUE, BI_GT(H_CR_JIT), disp);
    wp_begin(c, 0);

    /* Per-iteration fast-path words, for the executed-word estimate. */
    c->warm_words = (u32)(emit_mark(&c->e) - c->e.base) - c->warm_off;

    /* Budget spent: spill the warm set and yield at the loop top; the
     * dispatcher re-enters through the cold prologue next slice. Dirty
     * flags are saved so the loop-finished exit spills the same set. */
    for (i = 0; i < 32; i++) sgpr[i] = c->gpr[i].dirty;
    for (i = 0; i < 64; i++) sfpr[i] = c->fpr[i].dirty;
    emit_exit_to(c, c->start_pc, 0);
    for (i = 0; i < 32; i++) c->gpr[i].dirty = sgpr[i];
    for (i = 0; i < 64; i++) c->fpr[i].dirty = sfpr[i];

    e_patch_here(&c->e, loop_done);
    WP_SKIP_END(c);
    emit_exit_to(c, c->pc + 4, 0);
    return 1;
}

/* ------------------------------------------------------------------ */
/* Branches — these terminate a block                                   */
/* ------------------------------------------------------------------ */

/* Returns 1 if the instruction ended the block. */
static int compile_branch(JitContext *c, u32 op)
{
    switch (OPCD(op)) {

    case 18: {      /* b / bl / ba / bla */
        u32 target = AA_BIT(op) ? (u32)LI(op) : (c->pc + (u32)LI(op));
        if (LK_BIT(op)) {
            e_load_imm32_lo(&c->e, H_SCRATCH0, c->pc + 4);
            e_stw(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, lr), H_STATE);
        }
        emit_exit_to(c, target, c->guest_insts + 1);
        return 1;
    }

    case 16: {      /* bc */
        u32 target = AA_BIT(op) ? (u32)BD(op) : (c->pc + (u32)BD(op));
        u32 bo = BO(op), bi = BI(op);
        PPCFixup not_taken;

        /* LR is written whether or not the branch is taken. */
        if (LK_BIT(op)) {
            e_load_imm32_lo(&c->e, H_SCRATCH0, c->pc + 4);
            e_stw(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, lr), H_STATE);
        }

        /* CTR-decrementing branches: bdnz / bdz and the combined forms.
         *
         * This is the single most important branch to compile. The back-edge
         * of every counting loop GCC emits is a `bdnz`, so leaving these to the
         * interpreter meant every iteration of every loop paid a full fallback
         * -- a state spill, an indirect call, and a reload -- which on the
         * console showed up as real code running at a small fraction of the
         * speed the expansion ratio predicted. */
        if (!(bo & 0x04)) {
            /* The combined CTR-and-CR forms (bdnzt, bdzf, ...) also test a CR
             * bit, which has just been spilled to memory here; reloading it is
             * more than these rare forms are worth, so leave them to the
             * interpreter. The common forms ignore CR (BO bit 0x10 set). */
            if (!(bo & 0x10))
                return 0;

            if (c->retaining) {
                /* Retained back-edge. CTR is in retain_ctr_host and the loop's
                 * guest registers are already live, so the common path spills
                 * nothing. Decrement CTR and the cycle budget in-register, then
                 * pick among three outcomes:
                 *   - CTR terminal        -> loop done, exit to pc+4
                 *   - budget expired      -> yield to the loop top so the
                 *                            scheduler runs and the loop resumes
                 *   - otherwise           -> branch back to the warm entry
                 * The budget check is not optional: the interpreter stops the
                 * instant downcount hits zero, so a retained loop that ran to
                 * completion would diverge from it. */
                PPCFixup ctr_done;
                u8 sgpr[32], sfpr[64], scr;
                u32 spin, here;
                s32 disp;
                unsigned i;

                e_addi(&c->e, (u32)c->retain_ctr_host,
                       (u32)c->retain_ctr_host, -1);
                e_addi(&c->e, H_DOWNCOUNT, H_DOWNCOUNT,
                       -(s32)(c->guest_insts + 1));
                e_cmplwi(&c->e, H_CR_JIT, (u32)c->retain_ctr_host, 0);

                /* CTR terminal -> fall into the "done" exit. bdnz is terminal on
                 * CTR==0 (EQ); bdz on CTR!=0. */
                ctr_done = e_bc_fwd(&c->e,
                                    (bo & 0x02) ? BO_FALSE : BO_TRUE,
                                    BI_EQ(H_CR_JIT));
                WP_SKIP_BEGIN(c);

                /* CTR continues: loop iff the budget is not spent. */
                wp_backedge(c, c->retain_warm_off);
                e_cmpwi(&c->e, H_CR_JIT, H_DOWNCOUNT, 0);
                here = (u32)(emit_mark(&c->e) - c->e.base);
                disp = ((s32)c->retain_warm_off - (s32)here) * 4;
                e_bc(&c->e, BO_TRUE, BI_GT(H_CR_JIT), disp);
                wp_begin(c, 0);

                /* Budget spent: yield to the loop top. Save the dirty flags so
                 * the "done" exit below still spills the same registers. CTR is
                 * charged in-register already, so both exits pass 0. */
                for (i = 0; i < 32; i++) sgpr[i] = c->gpr[i].dirty;
                for (i = 0; i < 64; i++) sfpr[i] = c->fpr[i].dirty;
                scr = c->cr_dirty; spin = c->retain_pin;

                e_stw(&c->e, (u32)c->retain_ctr_host,
                      (s32)offsetof(PPCState, ctr), H_STATE);
                c->retain_pin = 0;
                emit_exit_to(c, c->start_pc, 0);

                for (i = 0; i < 32; i++) c->gpr[i].dirty = sgpr[i];
                for (i = 0; i < 64; i++) c->fpr[i].dirty = sfpr[i];
                c->cr_dirty = scr;
                c->retain_pin = spin;

                e_patch_here(&c->e, ctr_done);
                WP_SKIP_END(c);
                e_stw(&c->e, (u32)c->retain_ctr_host,
                      (s32)offsetof(PPCState, ctr), H_STATE);
                c->retain_pin = 0;
                emit_exit_to(c, c->pc + 4, 0);
                return 1;
            }

            /* Warm self-loop back edge (general bodies retention refused):
             * emit on the warm pass if the register contract held; record
             * the candidate on the probe pass. */
            if (target == c->start_pc && c->region == 0) {
                if (c->warm_active && warm_state_matches(c) &&
                    warm_backedge_in_range(c))
                    return emit_warm_backedge(c, op);
                warm_capture(c);
            }

            rc_flush_all(c);
            fpr_flush_all(c);
            cr_store(c);            /* guest CR consistent at the boundary */
            cr_invalidate(c);       /* free to clobber cr7 below */

            /* Decrement the 32-bit CTR in place, then test it. cmplwi looks at
             * the low word -- the guest counter -- and the reserved cr7 keeps
             * the test clear of the guest condition register. A counter of 0
             * decrements to 0xFFFFFFFF, still non-zero, matching the guest's
             * 32-bit wrap. */
            e_lwz(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, ctr), H_STATE);
            e_addi(&c->e, H_SCRATCH0, H_SCRATCH0, -1);
            e_stw(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, ctr), H_STATE);
            e_cmplwi(&c->e, H_CR_JIT, H_SCRATCH0, 0);

            /* Lay the taken path straight, branching to the fall-through on the
             * not-taken condition. bdnz (BO bit 0x02 clear) takes the branch
             * while CTR != 0, so it falls through on CTR == 0 (EQ); bdz is the
             * reverse. */
            not_taken = e_bc_fwd(&c->e,
                                 (bo & 0x02) ? BO_FALSE : BO_TRUE,
                                 BI_EQ(H_CR_JIT));

            emit_exit_tail(c, target, c->guest_insts + 1);
            e_patch_here(&c->e, not_taken);
            emit_exit_tail(c, c->pc + 4, c->guest_insts + 1);
            return 1;
        }

        if (bo & 0x10) {
            /* Unconditional after all (BO ignores CR). */
            emit_exit_to(c, target, c->guest_insts + 1);
            return 1;
        }

        if (c->retaining && c->retain_kind == 1 && target == c->start_pc) {
            /* Retained cmp/bc back-edge. The registers stay live; the guest CR
             * is spilled with mfcr+stw each iteration (a store, not a
             * load-hit-store) *before* the budget compare below clobbers cr7 --
             * cr7 is usually exactly where GCC put the guest's condition. The
             * body recomputes its compare field every iteration, so the
             * clobber is invisible to the loop and the memory copy is current
             * for both exits. Budget yield as in the bdnz form. */
            PPCFixup loop_done;
            u8 sgpr[32], sfpr[64];
            u32 spin, here;
            s32 disp;
            unsigned i2;
            u32 bcrf = H_CR_JIT;

            /* The budget compare below clobbers its host CR field every
             * iteration, and with a free guard field in play nothing reloads
             * the host CR between iterations any more (the body's guards no
             * longer invalidate it -- that reload was exactly what used to
             * launder the clobber before the next iteration's spill). So aim
             * the budget compare at the guard field too, and mark it stale
             * *before* emitting the spill: the spill then re-inserts that
             * field's authoritative memory copy each iteration, and the junk
             * never reaches the guest CR. Without a free field the body
             * guards keep their spill+invalidate, whose per-iteration reload
             * launders host cr7 exactly as before.
             *
             * The idle-skip shape emits no budget compare at all (it zeroes
             * the downcount and yields), so it has nothing to launder and
             * skips this -- body guards, if any, set the stale flag
             * themselves. */
            if (c->guard_crf >= 0 && !c->retain_idle) {
                bcrf = (u32)c->guard_crf;
                c->guard_crf_stale = 1;
            }

            cr_store_via(c, H_SCRATCH3);
            e_addi(&c->e, H_DOWNCOUNT, H_DOWNCOUNT,
                   -(s32)(c->guest_insts + 1));

            /* Loop exit when the guest branch is NOT taken: the same inverse
             * sense as the plain conditional path below. Guest-taken means
             * "branch back, continue the loop". */
            loop_done = e_bc_fwd(&c->e,
                                 (bo & 0x08) ? BO_FALSE : BO_TRUE, bi);
            WP_SKIP_BEGIN(c);

            if (c->retain_idle) {
                /* Idle skip. The scanner proved this body is pure
                 * load-and-compare: it cannot change its own exit condition,
                 * and the interrupt handler that can only runs between
                 * scheduler slices. Every iteration inside this slice would
                 * therefore see the same flag -- so the loop's remaining spin
                 * *is* the rest of the slice, and it is charged in one move:
                 * zero the downcount and yield. The scheduler advances
                 * virtual time over the skipped iterations, delivers whatever
                 * interrupt the loop is waiting on, and re-enters at the loop
                 * head for one real check per slice instead of thousands.
                 *
                 * MKWii's boot idles exactly here -- lwz/cmpwi/beq on a flag
                 * at 801a9cb4 -- for most of its 17 billion instructions. */
                /* Record what the skip discarded before discarding it. The
                 * instruction counter credits `grant - downcount`, so without
                 * this the skipped spin is indistinguishable from executed
                 * work and every derived throughput figure is inflated. One
                 * store, on a path that runs once per slice. */
                e_stw(&c->e, H_DOWNCOUNT,
                      (s32)offsetof(PPCState, idle_skipped_last), H_STATE);
                e_li(&c->e, H_DOWNCOUNT, 0);
                (void)here; (void)disp;
            } else {
            /* Continue while the budget lasts. */
            wp_backedge(c, c->retain_warm_off);
            e_cmpwi(&c->e, bcrf, H_DOWNCOUNT, 0);
            here = (u32)(emit_mark(&c->e) - c->e.base);
            disp = ((s32)c->retain_warm_off - (s32)here) * 4;
            e_bc(&c->e, BO_TRUE, BI_GT(bcrf), disp);
            wp_begin(c, 0);
            }

            /* Budget spent: yield to the loop top. */
            for (i2 = 0; i2 < 32; i2++) sgpr[i2] = c->gpr[i2].dirty;
            for (i2 = 0; i2 < 64; i2++) sfpr[i2] = c->fpr[i2].dirty;
            spin = c->retain_pin;
            c->retain_pin = 0;
            emit_exit_to(c, c->start_pc, 0);
            for (i2 = 0; i2 < 32; i2++) c->gpr[i2].dirty = sgpr[i2];
            for (i2 = 0; i2 < 64; i2++) c->fpr[i2].dirty = sfpr[i2];
            c->retain_pin = spin;

            /* Loop finished. */
            e_patch_here(&c->e, loop_done);
            WP_SKIP_END(c);
            c->retain_pin = 0;
            emit_exit_to(c, c->pc + 4, 0);
            return 1;
        }

        /* Warm self-loop back edge, CR-conditional form: same discovery and
         * emission as the CTR form above. */
        if (target == c->start_pc && c->region == 0) {
            if (c->warm_active && warm_state_matches(c) &&
                warm_backedge_in_range(c))
                return emit_warm_backedge(c, op);
            warm_capture(c);
        }

        /* Both exits write back the same registers -- the values computed in
         * this block do not depend on which way the branch goes -- so the
         * flush is emitted once, *above the branch*, and each exit gets only
         * its own target. That also removes the save-and-restore of the dirty
         * flags that the duplicated version needed: there is now exactly one
         * flush, so nothing can be cleared before a second one.
         *
         * The flush must precede the branch, not merely be written once in
         * the source. Emitting it after `e_bc_fwd` puts it on the fall-through
         * path only, so the not-taken exit jumps clean over every writeback
         * and discards the block's results -- a loop counter cached in a host
         * register never reaches memory, and the loop runs forever. That is
         * exactly what this ordering originally got wrong.
         *
         * Ordering within the flush: gpr/fpr writeback and carry_flush emit
         * only stores, and cr_store reads the CR with `mfcr` without
         * disturbing it, so the branch below still sees the guest condition
         * that cr_ensure put there. */
        rc_flush_all(c);
        fpr_flush_all(c);
        cr_ensure(c);
        cr_store(c);

        /* Emit the *inverse* condition as a forward branch to the fall-through
         * exit, so the taken path is the straight-line one. On a core with a
         * 24-cycle mispredict penalty, laying the likely path out straight is
         * worth more than the instruction it saves. */
        /* This host branch is taken when the *guest* branch is not, so its
         * hint is the inverse of the guest's prediction. The guest predicts
         * taken when (backward XOR y); the host branch is forward, so it
         * needs its y bit set to predict taken, i.e. set it when the guest
         * predicts NOT taken. Without this the layout silently assumed every
         * conditional exit was taken. */
        not_taken = e_bc_fwd(&c->e,
                             ((bo & 0x08) ? BO_FALSE : BO_TRUE) |
                               ((((target < c->pc) ? 1u : 0u) ^ (bo & 1u))
                                  ? 0u : BO_HINT),
                             bi);

        emit_exit_tail(c, target, c->guest_insts + 1);
        e_patch_here(&c->e, not_taken);
        emit_exit_tail(c, c->pc + 4, c->guest_insts + 1);
        return 1;
    }

    case 19: {
        u32 xo = XO10(op);

        if (xo == 16 || xo == 528) {        /* bclr / bcctr */
            u32 bo = BO(op), bi = BI(op);
            s32 tgt_off = (xo == 16) ? (s32)offsetof(PPCState, lr)
                                     : (s32)offsetof(PPCState, ctr);

            /* CTR-decrementing forms (bclr only; bcctr must not per the ISA)
             * are rare -- leave them to the interpreter rather than reload the
             * counter and combine conditions here. */
            if (!(bo & 0x04))
                return 0;

            if (bo & 0x10) {
                /* Unconditional: blr at the end of every function, bctr for
                 * jump tables and virtual calls. The overwhelming majority. */
                e_lwz(&c->e, H_SCRATCH0, tgt_off, H_STATE);
                e_rlwinm(&c->e, H_SCRATCH0, H_SCRATCH0, 0, 0, 29);
                if (LK_BIT(op)) {
                    e_load_imm32_lo(&c->e, H_SCRATCH1, c->pc + 4);
                    e_stw(&c->e, H_SCRATCH1,
                          (s32)offsetof(PPCState, lr), H_STATE);
                }
                emit_exit_reg(c, H_SCRATCH0, c->guest_insts + 1);
                return 1;
            }

            /* CR-conditional register branch: beqlr / bnelr and the like, which
             * are how the compiler emits an early return. Two exits -- to the
             * register target if taken, to the next instruction if not -- so it
             * flushes once, then splits, exactly like the conditional bc. */
            {
                PPCFixup not_taken;

                rc_flush_all(c);
                fpr_flush_all(c);
                cr_ensure(c);       /* the guest CR bit must be live to test */
                cr_store(c);        /* mfcr reads it without disturbing it */

                /* Read the target before LK overwrites LR. cr_store's transient
                 * use of H_SCRATCH0 is already done, so it is free now. */
                e_lwz(&c->e, H_SCRATCH0, tgt_off, H_STATE);
                e_rlwinm(&c->e, H_SCRATCH0, H_SCRATCH0, 0, 0, 29);
                if (LK_BIT(op)) {
                    e_load_imm32_lo(&c->e, H_SCRATCH1, c->pc + 4);
                    e_stw(&c->e, H_SCRATCH1,
                          (s32)offsetof(PPCState, lr), H_STATE);
                }

                /* Same inversion as the direct conditional exit above. A
                 * register-target branch (beqlr, the compiler's early
                 * return) has no static direction of its own, so the guest's
                 * y bit is the whole signal: y set means the guest predicted
                 * the return taken, which means this host branch is
                 * predicted not taken. */
                not_taken = e_bc_fwd(&c->e,
                                     ((bo & 0x08) ? BO_FALSE : BO_TRUE) |
                                       ((bo & 1u) ? 0u : BO_HINT),
                                     bi);
                emit_exit_reg_tail(c, H_SCRATCH0, c->guest_insts + 1);
                e_patch_here(&c->e, not_taken);
                emit_exit_tail(c, c->pc + 4, c->guest_insts + 1);
                return 1;
            }
        }
        return 0;
    }

    default:
        return 0;
    }
}

static int is_block_terminator(u32 op)
{
    switch (OPCD(op)) {
    case 16:            /* bc  */
    case 18:            /* b   */
    case 17:            /* sc  */
        return 1;
    case 19: {
        u32 xo = XO10(op);
        /* Branches, plus the condition-register logic and mcrf, which compile
         * to the identical host instruction. */
        return xo == 16 || xo == 528 || xo == 50;   /* bclr, bcctr, rfi */
    }
    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Loop retention                                                       */
/*                                                                      */
/* A self-looping block normally reloads its guest registers from PPCState at    */
/* the top and spills them at the bottom on every iteration. On the PPE that      */
/* store-then-reload to the same addresses is a load-hit-store stall (~40         */
/* cycles), and it is the dominant cost of a tight loop.                          */
/*                                                                                */
/* For the common shape -- a `bdnz` counted loop with an integer body that        */
/* touches neither the condition register, XER[CA], nor the FPU -- the registers  */
/* and CTR are loaded once at the block's cold entry and kept live across the     */
/* back-edge. The back-edge branches to a "warm entry" just after the preload,    */
/* with everything already in host registers, and only the loop *exit* spills.    */
/*                                                                                */
/* It is deliberately conservative: anything outside a small whitelist            */
/* disqualifies the loop (retain_safe_op), and any surprise during compilation    */
/* -- a fallback, register-file exhaustion -- sets retain_aborted, on which        */
/* jit_compile_block simply recompiles the block the ordinary way. So the worst   */
/* case is a wasted first compile, never wrong code. Correctness rests on the      */
/* big-endian differential suite, which executes these loops on real PowerPC.     */
/* ------------------------------------------------------------------ */

/* An instruction that reads/writes only GPRs -- no CR (so no Rc=1 forms, no       */
/* compares), no carry, no FP. Only these may appear in a retained loop body.      */
/* Compares are handled separately: a cmp/bc loop needs exactly the compare its    */
/* back-edge tests, and nothing else that touches CR.                              */
static int retain_compare_op(u32 op)
{
    switch (OPCD(op)) {
    case 11: case 10:                       /* cmpi, cmpli */
        return 1;
    case 31: {
        u32 xo = XO10(op);
        return (xo == 0 || xo == 32) && !(op & 1u);   /* cmp, cmpl */
    }
    default:
        return 0;
    }
}

static int retain_safe_op(u32 op)
{
    u32 opcd = OPCD(op);
    switch (opcd) {
    case 14: case 15:                       /* addi, addis           */
    case 24: case 25: case 26: case 27:     /* ori oris xori xoris   */
    case 32: case 34: case 40: case 42:     /* lwz lbz lhz lha       */
    case 36: case 38: case 44:              /* stw stb sth           */
    case 33: case 35: case 41: case 43:     /* lwzu lbzu lhzu lhau   */
    case 37: case 39: case 45:              /* stwu stbu sthu        */
        return 1;
    case 20: case 21: case 23:              /* rlwimi rlwinm rlwnm   */
        return !(op & 1u);                  /* Rc=0 only             */
    case 31: {
        u32 xo = XO10(op);
        if (op & 1u) return 0;              /* Rc=1 writes CR        */
        switch (xo) {
        case 266: case 40: case 104:                /* add subf neg  */
        case 235: case 75: case 11:                 /* mullw mulhw mulhwu */
        case 24: case 536:                          /* slw srw       */
        case 28: case 444: case 316:                /* and or xor    */
        case 476: case 124: case 60: case 412:      /* nand nor andc orc */
        case 954: case 922: case 26:                /* extsb extsh cntlzw */
        case 23: case 87: case 279: case 343:       /* lwzx lbzx lhzx lhax */
        case 151: case 215: case 407:               /* stwx stbx sthx */
            return 1;
        default:
            return 0;                       /* sraw/srawi set CA; carry forms; etc. */
        }
    }
    default:
        return 0;
    }
}

/* Add the register operands of a whitelisted instruction to the mask. Only the
 * fields that are genuinely registers for the form, so an immediate is never
 * mistaken for a register number. */
static void retain_collect(u32 op, u32 *mask)
{
    u32 opcd = OPCD(op);
    *mask |= (1u << RT(op)) | (1u << RA(op));    /* RT/RS and RA in every form */
    if (opcd == 31 || opcd == 23)                /* X-form and rlwnm use RB    */
        *mask |= (1u << RB(op));
}

/* Decide whether the block at pc is a retention candidate and, if so, return the
 * set of guest registers it touches. */
static int loop_retention_scan(PPCState *s, u32 pc, u32 *out_mask, u8 *out_kind,
                               u8 *out_idle)
{
    u32 mask = 0, p = pc;
    int n = 0, cnt, g;
    int has_compare = 0;
    /* Idle until proven otherwise: a loop stays "idle" while its body is
     * nothing but non-update loads and compares -- code that reads a flag and
     * tests it, with no store, no arithmetic that accumulates, no side effect
     * of any kind. Such a loop cannot change its own exit condition; only an
     * interrupt handler can, and handlers only run between scheduler slices.
     * Spinning it for the rest of a slice is therefore *exactly* equivalent
     * to skipping the rest of the slice, which is what the back-edge does
     * with the classification made here. */
    int idle = 1;

    (void)s;
    for (;;) {
        u32 op = mem_read32_for_fetch(p);
        if (n++ > 48)
            return 0;                       /* too long to be worth retaining */

        if (!is_block_terminator(op)) {
            unsigned pr = OPCD(op);
            int is_load = (pr == 32 || pr == 34 || pr == 40 || pr == 42);
            int is_cmp  = (pr == 11 || pr == 10) ||
                          (pr == 31 && (XO10(op) == 0 || XO10(op) == 32));
            if (!is_load && !is_cmp)
                idle = 0;
        }

        if (is_block_terminator(op)) {
            u32 bo = BO(op);
            if (OPCD(op) != 16) return 0;               /* not bc            */
            if (AA_BIT(op))     return 0;               /* absolute target   */
            if ((u32)(p + (u32)BD(op)) != pc) return 0; /* not a self-loop   */
            if (!(bo & 0x04)) {
                /* CTR-decrementing back-edge: bdnz/bdz, CR ignored. */
                if (!(bo & 0x10)) return 0;
                if (has_compare)  return 0;     /* CR written but never read */
                *out_kind = 0;
            } else if (!(bo & 0x10)) {
                /* CR-conditional back-edge: needs exactly one live compare. */
                if (!has_compare) return 0;
                *out_kind = 1;
                *out_idle = (u8)idle;
            } else {
                return 0;                       /* unconditional: not a loop */
            }
            break;
        }
        if (retain_compare_op(op)) {
            has_compare = 1;
            retain_collect(op, &mask);
            p += 4;
            continue;
        }
        if (!retain_safe_op(op))
            return 0;
        retain_collect(op, &mask);
        p += 4;
    }

    /* Leave headroom in the 14-register cache for address bases and temporaries;
     * a loop touching more than ten distinct GPRs is not worth the risk. */
    for (cnt = 0, g = 0; g < 32; g++)
        if (mask & (1u << g)) cnt++;
    if (cnt == 0 || cnt > 10)
        return 0;

    *out_mask = mask;
    return 1;
}

/* ------------------------------------------------------------------ */
/* Block compilation                                                    */
/* ------------------------------------------------------------------ */


/* ------------------------------------------------------------------ */
/* Provenance transfer functions                                        */
/*                                                                      */
/* prov_transfer computes, from the PRE-instruction lattice, the provenance    */
/* of the (at most two: destination, and an update-form's base) guest GPRs     */
/* the instruction writes. The caller applies the result only after            */
/* compile_one actually took the native path -- a fallback wipes the whole     */
/* lattice instead, and gpr_dest/gpr_write have already returned every         */
/* written register to unknown, so an instruction this function does not       */
/* model is conservative by construction.                                      */
/*                                                                             */
/* Every rule models GUEST semantics (the interpreter's), not the emitted      */
/* host code; the two are equivalent on the low 32 bits, which is all a        */
/* ProvSlot describes.                                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    u8  reg;
    u8  kind;
    u32 value;
    u32 span;
} ProvUpdate;

static u32 prov_rotl32(u32 v, u32 n)
{
    n &= 31u;
    return n ? ((v << n) | (v >> (32u - n))) : v;
}

/* The rlwinm/rlwimi mask, PowerPC bit numbering (bit 0 = MSB). */
static u32 prov_mask32(u32 mb, u32 me)
{
    u32 a = 0xFFFFFFFFu >> mb;
    u32 b = 0xFFFFFFFFu << (31u - me);
    return (mb <= me) ? (a & b) : (a | b);
}

static unsigned prov_transfer(const JitContext *c, u32 op, ProvUpdate out[2])
{
    const ProvSlot *P = c->prov;
    unsigned n = 0;
    u32 rs = RS(op), ra = RA(op), rb = RB(op);

#define PROV_OUT(r, k, v) \
    do { out[n].reg = (u8)(r); out[n].kind = (u8)(k); out[n].value = (v); \
         out[n].span = 0; n++; } while (0)
#define PROV_OUT_RANGE(r, lov, spanv) \
    do { out[n].reg = (u8)(r); out[n].kind = PROV_RANGE; \
         out[n].value = (lov); out[n].span = (spanv); n++; } while (0)

    switch (OPCD(op)) {
    case 14:        /* addi (li when RA=0) */
        if (ra == 0)
            PROV_OUT(RT(op), PROV_CONST, (u32)SIMM(op));
        else if (P[ra].kind == PROV_CONST)
            PROV_OUT(RT(op), PROV_CONST, P[ra].value + (u32)SIMM(op));
        else if (P[ra].kind == PROV_RANGE)
            /* the interval shifts whole; the span is unchanged */
            PROV_OUT_RANGE(RT(op), P[ra].value + (u32)SIMM(op), P[ra].span);
        else if (P[ra].kind == PROV_UPPER)
            PROV_OUT_RANGE(RT(op), P[ra].value + (u32)SIMM(op), 0xFFFFu);
        break;

    case 15:        /* addis (lis when RA=0): adds imm<<16, low half untouched,
                     * so a known upper half stays known. */
        if (ra == 0)
            PROV_OUT(RT(op), PROV_CONST, UIMM(op) << 16);
        else if (P[ra].kind == PROV_CONST)
            PROV_OUT(RT(op), PROV_CONST, P[ra].value + (UIMM(op) << 16));
        else if (P[ra].kind == PROV_UPPER)
            PROV_OUT(RT(op), PROV_UPPER, P[ra].value + (UIMM(op) << 16));
        else if (P[ra].kind == PROV_RANGE)
            PROV_OUT_RANGE(RT(op), P[ra].value + (UIMM(op) << 16),
                           P[ra].span);
        break;

    case 24:        /* ori: low half only; upper half survives */
        if (P[rs].kind == PROV_CONST)
            PROV_OUT(ra, PROV_CONST, P[rs].value | UIMM(op));
        else if (P[rs].kind == PROV_UPPER)
            PROV_OUT(ra, PROV_UPPER, P[rs].value);
        break;

    case 26:        /* xori: low half only; upper half survives */
        if (P[rs].kind == PROV_CONST)
            PROV_OUT(ra, PROV_CONST, P[rs].value ^ UIMM(op));
        else if (P[rs].kind == PROV_UPPER)
            PROV_OUT(ra, PROV_UPPER, P[rs].value);
        break;

    case 25:        /* oris: upper half only */
        if (P[rs].kind == PROV_CONST)
            PROV_OUT(ra, PROV_CONST, P[rs].value | (UIMM(op) << 16));
        else if (P[rs].kind == PROV_UPPER)
            PROV_OUT(ra, PROV_UPPER, P[rs].value | (UIMM(op) << 16));
        break;

    case 27:        /* xoris: upper half only */
        if (P[rs].kind == PROV_CONST)
            PROV_OUT(ra, PROV_CONST, P[rs].value ^ (UIMM(op) << 16));
        else if (P[rs].kind == PROV_UPPER)
            PROV_OUT(ra, PROV_UPPER, P[rs].value ^ (UIMM(op) << 16));
        break;

    case 28:        /* andi.: the result is a subset of the mask bits,
                     * and a bit-subset of a constant never exceeds it */
        if (P[rs].kind == PROV_CONST)
            PROV_OUT(ra, PROV_CONST, P[rs].value & UIMM(op));
        else
            PROV_OUT_RANGE(ra, 0, UIMM(op));
        break;

    case 29:        /* andis.: the low half is forced to zero, so a known
                     * upper half yields a full constant. CONST and UPPER
                     * only: a RANGE source's upper half VARIES across the
                     * interval, so treating its base as the upper half was
                     * unsound (found in review). */
        if (P[rs].kind == PROV_CONST || P[rs].kind == PROV_UPPER)
            PROV_OUT(ra, PROV_CONST, P[rs].value & (UIMM(op) << 16));
        break;

    case 21: {      /* rlwinm */
        u32 m = prov_mask32(MB(op), ME(op));
        if (P[rs].kind == PROV_CONST) {
            PROV_OUT(ra, PROV_CONST, prov_rotl32(P[rs].value, SH(op)) & m);
            break;
        }
        if (P[rs].kind == PROV_UPPER && SH(op) == 0) {
            /* No rotation: upper stays where it was, masked. If the mask
             * also clears the whole low half the result is fully known. */
            PROV_OUT(ra, (m & 0xFFFFu) ? PROV_UPPER : PROV_CONST,
                     P[rs].value & m);
            break;
        }
        /* A masked value is a bit-subset of the mask, and a bit-subset of a
         * constant never exceeds it: result in [0, m] for any source. A
         * bounded source that the rotation cannot wrap (an unwrapped
         * interval starting at 0 -- a zero-extended or masked index -- whose
         * top value shifted left still fits in 32 bits) tightens that to
         * [0, min(m, hi << sh)]: the scaled-index idiom, lhz;slwi;lwzx. */
        {
            u32 bound = m;
            u64 lo, hi;
            if (prov_bounds(c, rs, &lo, &hi) && lo == 0 &&
                SH(op) < 32 && (hi << SH(op)) <= 0xFFFFFFFFull) {
                u32 shifted = (u32)(hi << SH(op));
                if (shifted < bound)
                    bound = shifted;
            }
            PROV_OUT_RANGE(ra, 0, bound);
        }
        break;
    }

    case 23:        /* rlwnm: variable rotation, but the mask bounds it */
        if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST) {
            u32 m = prov_mask32(MB(op), ME(op));
            PROV_OUT(ra, PROV_CONST,
                     prov_rotl32(P[rs].value, P[rb].value & 0x1Fu) & m);
        } else
            PROV_OUT_RANGE(ra, 0, prov_mask32(MB(op), ME(op)));
        break;

    case 20: {      /* rlwimi (its unknown case was killed at the site) */
        if (P[rs].kind == PROV_CONST && P[ra].kind == PROV_CONST) {
            u32 m = prov_mask32(MB(op), ME(op));
            PROV_OUT(ra, PROV_CONST,
                     (prov_rotl32(P[rs].value, SH(op)) & m) |
                     (P[ra].value & ~m));
        }
        break;
    }

    case 34:                    /* lbz: zero-extended byte */
        PROV_OUT_RANGE(RT(op), 0, 0xFFu);
        break;
    case 40:                    /* lhz: zero-extended halfword */
        PROV_OUT_RANGE(RT(op), 0, 0xFFFFu);
        break;
    case 42:                    /* lha: sign-extended halfword -- a wrapped
                                 * interval, which RANGE represents exactly */
        PROV_OUT_RANGE(RT(op), 0xFFFF8000u, 0xFFFFu);
        break;

    case 35:                    /* lbzu */
        PROV_OUT_RANGE(RT(op), 0, 0xFFu);
        goto d_update;
    case 41:                    /* lhzu */
        PROV_OUT_RANGE(RT(op), 0, 0xFFFFu);
        goto d_update;
    case 43:                    /* lhau */
        PROV_OUT_RANGE(RT(op), 0xFFFF8000u, 0xFFFFu);
        goto d_update;
    case 33:                    /* lwzu (destination stays unknown) */
    case 37: case 39: case 45:  /* stwu, stbu, sthu */
    case 49: case 51: case 53: case 55: /* lfsu, lfdu, stfsu, stfdu */
d_update:
        if (P[ra].kind == PROV_CONST)
            PROV_OUT(ra, PROV_CONST, P[ra].value + (u32)SIMM(op));
        else if (P[ra].kind == PROV_RANGE)
            PROV_OUT_RANGE(ra, P[ra].value + (u32)SIMM(op), P[ra].span);
        break;

    case 57: case 61:           /* psq_lu, psq_stu: 12-bit displacement */
        if (P[ra].kind == PROV_CONST)
            PROV_OUT(ra, PROV_CONST, P[ra].value + (u32)PS_D(op));
        break;

    case 7:         /* mulli: low 32 bits of the product */
        if (P[ra].kind == PROV_CONST)
            PROV_OUT(RT(op), PROV_CONST, P[ra].value * (u32)SIMM(op));
        break;

    case 8:         /* subfic (its carry is not tracked here) */
        if (P[ra].kind == PROV_CONST)
            PROV_OUT(RT(op), PROV_CONST, (u32)SIMM(op) - P[ra].value);
        break;

    case 12: case 13:   /* addic, addic. */
        if (P[ra].kind == PROV_CONST)
            PROV_OUT(RT(op), PROV_CONST, P[ra].value + (u32)SIMM(op));
        break;

    case 31:
        switch (XO10(op)) {
        case 266:   /* add: interval addition, spans capped so the
                     * classifier's width check stays meaningful */
            if (P[ra].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(RT(op), PROV_CONST, P[ra].value + P[rb].value);
            else {
                u64 alo, ahi, blo, bhi, span;
                if (prov_bounds(c, ra, &alo, &ahi) &&
                    prov_bounds(c, rb, &blo, &bhi)) {
                    span = (ahi - alo) + (bhi - blo);
                    if (span <= 0xFFFFFFFFull)
                        PROV_OUT_RANGE(RT(op), (u32)(alo + blo), (u32)span);
                }
            }
            break;
        case 40:    /* subf: RB - RA */
            if (P[ra].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(RT(op), PROV_CONST, P[rb].value - P[ra].value);
            break;
        case 104:   /* neg */
            if (P[ra].kind == PROV_CONST)
                PROV_OUT(RT(op), PROV_CONST, 0u - P[ra].value);
            break;
        case 235:   /* mullw: low 32 bits */
            if (P[ra].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(RT(op), PROV_CONST, P[ra].value * P[rb].value);
            break;
        case 444:   /* or -- including mr, which copies provenance whole */
            if (rs == rb) {
                /* Copy the WHOLE slot: PROV_OUT zeroes the span, which for a
                 * RANGE source silently narrowed the interval to its low end
                 * -- an unsound claim of exactness (found in review). */
                if (P[rs].kind == PROV_RANGE)
                    PROV_OUT_RANGE(ra, P[rs].value, P[rs].span);
                else if (P[rs].kind != PROV_UNKNOWN)
                    PROV_OUT(ra, P[rs].kind, P[rs].value);
            } else if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(ra, PROV_CONST, P[rs].value | P[rb].value);
            break;
        case 28: {  /* and: the result never exceeds either operand, so any
                     * operand with a non-wrapped interval caps it */
            u64 lo, hi, bound = 0xFFFFFFFFull;
            if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST) {
                PROV_OUT(ra, PROV_CONST, P[rs].value & P[rb].value);
                break;
            }
            if (prov_bounds(c, rs, &lo, &hi) && hi < bound)
                bound = hi;
            if (prov_bounds(c, rb, &lo, &hi) && hi < bound)
                bound = hi;
            if (bound < 0xFFFFFFFFull)
                PROV_OUT_RANGE(ra, 0, (u32)bound);
            break;
        }
        case 316:   /* xor */
            if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(ra, PROV_CONST, P[rs].value ^ P[rb].value);
            break;
        case 60:    /* andc */
            if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(ra, PROV_CONST, P[rs].value & ~P[rb].value);
            break;
        case 124:   /* nor */
            if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(ra, PROV_CONST, ~(P[rs].value | P[rb].value));
            break;
        case 284:   /* eqv */
            if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(ra, PROV_CONST, ~(P[rs].value ^ P[rb].value));
            break;
        case 412:   /* orc */
            if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(ra, PROV_CONST, P[rs].value | ~P[rb].value);
            break;
        case 476:   /* nand */
            if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST)
                PROV_OUT(ra, PROV_CONST, ~(P[rs].value & P[rb].value));
            break;
        case 24:    /* slw: n = RB & 0x3F, zero when n > 31 */
            if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST) {
                u32 sh = P[rb].value & 0x3Fu;
                PROV_OUT(ra, PROV_CONST,
                         (sh < 32) ? (P[rs].value << sh) : 0u);
            }
            break;
        case 536:   /* srw */
            if (P[rs].kind == PROV_CONST && P[rb].kind == PROV_CONST) {
                u32 sh = P[rb].value & 0x3Fu;
                PROV_OUT(ra, PROV_CONST,
                         (sh < 32) ? (P[rs].value >> sh) : 0u);
            }
            break;
        case 954:   /* extsb */
            if (P[rs].kind == PROV_CONST)
                PROV_OUT(ra, PROV_CONST, (u32)(s32)(s8)P[rs].value);
            else
                PROV_OUT_RANGE(ra, 0xFFFFFF80u, 0xFFu);
            break;
        case 922:   /* extsh */
            if (P[rs].kind == PROV_CONST)
                PROV_OUT(ra, PROV_CONST, (u32)(s32)(s16)P[rs].value);
            else
                PROV_OUT_RANGE(ra, 0xFFFF8000u, 0xFFFFu);
            break;
        case 87:    /* lbzx */
            PROV_OUT_RANGE(RT(op), 0, 0xFFu);
            break;
        case 279:   /* lhzx */
            PROV_OUT_RANGE(RT(op), 0, 0xFFFFu);
            break;
        case 343:   /* lhax */
            PROV_OUT_RANGE(RT(op), 0xFFFF8000u, 0xFFFFu);
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }
#undef PROV_OUT
    return n;
}

#define JIT_MAX_GUEST_INSTS 256

/* ------------------------------------------------------------------ */
/* Trace formation policy                                               */
/*                                                                      */
/* Where a compilation unit stops is the single largest structural cost in    */
/* the recompiler. The measured composition of EXECUTED host words puts exit  */
/* tails at ~21%, register-cache reloads at 9.2% and writebacks at 8.8% --    */
/* all three per-UNIT, none of them per-instruction. So the lever is not to   */
/* make the code cheaper, it is to need fewer units.                          */
/*                                                                            */
/* Every rule below is a pure function of the guest instruction stream, and   */
/* every one lives in exactly one place, because it has TWO consumers that    */
/* must agree:                                                                */
/*                                                                            */
/*   jit_compile_into  -- decides where compilation actually goes;            */
/*   guard_crf_pick    -- pre-scans the same code to find a host CR field no   */
/*                        guest instruction in the unit touches.              */
/*                                                                            */
/* The second is a correctness dependency, not an optimization one: a guard   */
/* comparing into a field some later instruction reads would corrupt guest    */
/* control flow. The pre-scan must therefore cover AT LEAST every instruction */
/* the compiler will process. Sharing the predicates is what makes that       */
/* checkable by inspection instead of by hoping two hand-written walks stay   */
/* in step. Where the compiler consults state the scan cannot see (cold-slot  */
/* budget, register-file exhaustion) the compiler only ever STOPS EARLIER,    */
/* which keeps its instruction set a subset of the scan's.                    */
/* ------------------------------------------------------------------ */

#if JIT_TRACE
/* Instructions scanned from a call target looking for its return. Larger than
 * the original 24 because the scan now walks *through* the shapes the compiler
 * walks through -- linker veneers and inlined forward conditionals -- and
 * those consume scan steps without lengthening the leaf's straight-line body. */
#ifndef JIT_TRACE_LEAF_SCAN
#define JIT_TRACE_LEAF_SCAN   24
#endif
/* Unconditional-branch hops the leaf scan will chase before giving up. A `b`
 * at a call target is a linker branch island or a tail call to the real body;
 * one or two hops is what the toolchain emits, and bounding it keeps the scan
 * finite without needing a visited set. */
#ifndef JIT_TRACE_LEAF_HOPS
#define JIT_TRACE_LEAF_HOPS   2
#endif
#define JIT_TRACE_LEAF_BC     1   /* step over inlinable conditionals    */
#define JIT_TRACE_RETURNS     1   /* continue at a known return address  */
#else
#undef  JIT_TRACE_LEAF_SCAN
#undef  JIT_TRACE_LEAF_HOPS
#define JIT_TRACE_LEAF_SCAN   24
#define JIT_TRACE_LEAF_HOPS   0
#define JIT_TRACE_LEAF_BC     0
#define JIT_TRACE_RETURNS     0
#endif

/* Is there still room in the code cache to spend on trace formation?
 *
 * Every rule in this section trades code SIZE for executed words: a followed
 * call duplicates its callee, a followed return duplicates its caller's
 * suffix, a region duplicates its target. On the console the cache is 8 MiB
 * of .text and an overflow flushes EVERYTHING, and the unmodified recompiler
 * already fills 6.7 MiB of it during a boot -- so the trade is affordable
 * only while there is headroom, and past that point the right shape is the
 * pre-trace one. Consulted by the compiler and by guard_crf_pick alike, and
 * constant for the duration of one compile (the emitter's bump pointer is
 * committed only when a unit succeeds), so the two cannot disagree.
 *
 * The effect is a recompiler that traces aggressively into a cold cache and
 * stops growing units as it fills, instead of one that flushes and starts
 * over. Measured on the boot: it is the difference between one full flush and
 * none. */
static int trace_have_room(void)
{
    return JIT_TRACE && jit_code_pressure() < JIT_TRACE_PRESSURE_PCT;
}

/* `blr` with BO ignoring both CR and CTR, and no link: a plain function
 * return, which is what all but a handful of op19 branches are. */
static int trace_is_uncond_blr(u32 w)
{
    return OPCD(w) == 19 && XO10(w) == 16 &&
           (BO(w) & 0x14) == 0x14 && !LK_BIT(w);
}

static int trace_call_is_leaf(u32 target);

/* Would a conditional branch be compiled with its fall-through inline and its
 * taken side deferred to a cold slot?
 *
 * Only *forward* targets: a backward `bc` is a loop back edge, where the taken
 * side is the hot one and the retained/warm self-loop machinery already
 * compiles the hottest shapes specially. Skipped too: bcl (rare, needs the LR
 * store), the CTR-decrementing forms, and BO patterns that ignore CR (those
 * are unconditional, and the op18 path covers the idea). */
static int trace_inline_bc(u32 pc, u32 op, u32 *out_target)
{
    u32 bo = BO(op);
    u32 target = AA_BIT(op) ? (u32)BD(op) : (pc + (u32)BD(op));

    *out_target = target;
    if (LK_BIT(op))
        return 0;
    if (!(bo & 0x04) || (bo & 0x10))
        return 0;
    return target > pc && mem_is_ram(target);
}

/* Would compilation continue THROUGH the unconditional direct branch `op` at
 * `pc`, and where?
 *
 * An AOT-owned target must not be followed: a followed branch reaches the
 * target without ever touching the dispatcher, so the registered function
 * would be bypassed at every direct call site. Compiling the branch as an
 * ordinary exit instead keeps those calls on the dispatcher path; its link
 * site is then refused by link_or_defer for the same reason. */
static int trace_follow_b(u32 pc, u32 op, u32 *out_target)
{
    u32 target = AA_BIT(op) ? (u32)LI(op) : (pc + (u32)LI(op));

    *out_target = target;
    if (target == pc || !mem_is_ram(target) || jit_aot_owns_pc(target))
        return 0;
    /* A call is followed only into a small leaf. Unrestricted call-following
     * inlines every callee at every call site; on the console that filled the
     * whole 8 MiB code cache and flushed it 31 times in one boot, and the
     * recompile storms ate more than the inlining saved. */
    if (LK_BIT(op) && !trace_call_is_leaf(target))
        return 0;
    return 1;
}

/* Would compilation continue at a return whose LR is the known constant
 * `lr`? Same AOT rule as every other statically known target. */
static int trace_follow_blr(u32 lr)
{
    return JIT_TRACE_RETURNS && trace_have_room() &&
           mem_is_ram(lr) && !jit_aot_owns_pc(lr);
}

/* Does this instruction destroy the compiler's knowledge of the guest LR?
 * Deliberately opcode-only, so the compiler and the pre-scan reach the same
 * answer. The compiler additionally forgets LR at an interpreter fallback,
 * which the scan cannot predict -- that direction is safe, because it only
 * makes the compiler stop where the scan kept going. */
static int trace_kills_lr(u32 op)
{
    return OPCD(op) == 31 && XO10(op) == 467 && SPRN(op) == SPR_LR;
}

/* Is `target` a small leaf: straight-line code reaching a plain `blr` within
 * the scan window, chasing veneers and stepping over inlinable forward
 * conditionals exactly as compilation will? A large or branching callee keeps
 * the classic exit-and-link, whose cost is one dispatch, not a cache flush. */
static int trace_call_is_leaf(u32 target)
{
    u32 p = target;
    unsigned k, hops = 0;

    for (k = 0; k < JIT_TRACE_LEAF_SCAN; k++) {
        u32 w = mem_read32_for_fetch(p), t;

        if (trace_is_uncond_blr(w))
            return 1;
        if (OPCD(w) == 18) {
            /* A veneer or tail branch. The compiler follows it, so the scan
             * must too, or a call whose body sits one indirection away would
             * never be inlined -- and MKWii's hot path calls exactly such a
             * thunk 19 million times per boot. A nested CALL is a different
             * matter and still disqualifies the leaf. The follow test is
             * spelled out rather than delegated to trace_follow_b, which
             * would recurse straight back into this function. */
            if (LK_BIT(w) || hops >= JIT_TRACE_LEAF_HOPS ||
                !trace_have_room())
                return 0;
            t = AA_BIT(w) ? (u32)LI(w) : (p + (u32)LI(w));
            if (t == p || !mem_is_ram(t) || jit_aot_owns_pc(t))
                return 0;
            hops++;
            p = t;
            continue;
        }
        if (OPCD(w) == 16) {
            if (!JIT_TRACE_LEAF_BC || !trace_have_room() ||
                !trace_inline_bc(p, w, &t))
                return 0;
            p += 4;
            continue;
        }
        if (is_block_terminator(w))
            return 0;
        p += 4;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Guard CR field selection                                             */
/*                                                                      */
/* The guest CR is mirrored into the whole host CR, so the MMIO/GQR guards'   */
/* compares historically had to spill it (mfcr+stw) and let the next guest    */
/* CR use reload it (lwz+mtcrf) -- 4 executed words per guard in CR-live      */
/* code, and the measured audit showed hot decompression loops (decodeSZS)    */
/* paying it three times per 16-instruction iteration.                        */
/*                                                                            */
/* But a guard only needs SOME host CR field, and most blocks leave several   */
/* guest fields completely untouched: GCC concentrates on cr0 (Rc) and cr7    */
/* (compares). A field the block's native code provably neither reads nor     */
/* writes can be clobbered freely: nothing in the block observes the host     */
/* copy, and the memory copy in PPCState.cr keeps the authoritative guest     */
/* value (only helper calls change it, and those write memory directly).      */
/* The single debt is at CR stores: `mfcr` reads all eight fields, so         */
/* cr_store_via re-inserts the clobbered field's memory copy before the       */
/* `stw` (one lwz + one rlwimi) -- making the guest-visible CR at every       */
/* exit bit-identical to the spill/reload scheme.                             */
/*                                                                            */
/* cr_fields_touched is deliberately a SUPERSET of what jit_compile emits     */
/* native CR accesses for: it marks every field a guest instruction could     */
/* touch, native or helper. Marking too much only costs the optimization;     */
/* marking too little would be a correctness bug, so anything ambiguous       */
/* (mfcr, mtcrf, unknown op19 forms) marks all fields. Host cr7 (H_CR_JIT)    */
/* stays reserved for the JIT's own internal tests and is never a candidate.  */
/* ------------------------------------------------------------------ */

static u8 cr_fields_touched(u32 op)
{
    switch (OPCD(op)) {
    case 10: case 11:                       /* cmpli, cmpi          */
        return (u8)(1u << CRFD(op));
    case 13:                                /* addic.               */
    case 28: case 29:                       /* andi., andis.        */
        return 0x01;
    case 20: case 21: case 23:              /* rlwimi rlwinm rlwnm  */
        return (u8)((op & 1u) ? 0x01 : 0);  /* Rc -> cr0            */
    case 16:                                /* bc                   */
        return (BO(op) & 0x10) ? 0 : (u8)(1u << (BI(op) >> 2));
    case 19: {
        u32 xo = XO10(op);
        if (xo == 16 || xo == 528)          /* bclr, bcctr          */
            return (BO(op) & 0x10) ? 0 : (u8)(1u << (BI(op) >> 2));
        if (xo == 0)                        /* mcrf                 */
            return (u8)((1u << CRFD(op)) | (1u << CRFS(op)));
        if (xo == 33 || xo == 129 || xo == 193 || xo == 225 ||
            xo == 257 || xo == 289 || xo == 417 || xo == 449)
            /* crnor crandc crxor crnand crand creqv crorc cror */
            return (u8)((1u << (CRBD(op) >> 2)) | (1u << (CRBA(op) >> 2)) |
                        (1u << (CRBB(op) >> 2)));
        if (xo == 50 || xo == 150)          /* rfi, isync           */
            return 0;
        return 0xFF;                        /* anything else: be safe */
    }
    case 31: {
        u32 xo = XO10(op);
        u8 m = (u8)((op & 1u) ? 0x01 : 0);  /* Rc (incl. stwcx.) -> cr0 */
        if (xo == 0 || xo == 32 || xo == 512)   /* cmp, cmpl, mcrxr */
            m |= (u8)(1u << CRFD(op));
        if (xo == 19 || xo == 144)          /* mfcr, mtcrf          */
            m = 0xFF;
        return m;
    }
    case 4: {                               /* paired singles       */
        u32 xo = XO10(op);
        u8 m = (u8)((op & 1u) ? 0x02 : 0);  /* Rc -> cr1            */
        if (xo == 0 || xo == 32 || xo == 64 || xo == 96)  /* ps_cmp* */
            m |= (u8)(1u << CRFD(op));
        return m;
    }
    case 59:
        return (u8)((op & 1u) ? 0x02 : 0);  /* Rc -> cr1            */
    case 63: {
        u32 xo = XO10(op);
        u8 m = (u8)((op & 1u) ? 0x02 : 0);  /* Rc -> cr1            */
        if (xo == 0 || xo == 32 || xo == 64)    /* fcmpu, fcmpo, mcrfs */
            m |= (u8)(1u << CRFD(op));
        return m;
    }
    default:
        return 0;       /* loads, stores, plain arithmetic: no CR anywhere */
    }
}

/* Walk the code jit_compile_into is about to compile and pick a host CR
 * field (0..6) no guest instruction in the unit touches, or -1.
 *
 * The traversal must cover AT LEAST every instruction the compiler will
 * process, and it does so by consulting the SAME trace-formation predicates
 * the compiler consults (trace_follow_b / trace_inline_bc / trace_follow_blr
 * / trace_kills_lr). Where the compiler additionally depends on compile-time
 * state this scan cannot see -- the cold-slot budget, register-file pressure,
 * an interpreter fallback forgetting the return address -- the effect is
 * always that the compiler stops EARLIER, so its instruction set stays a
 * subset of this one.
 *
 * Retained self-loops are automatically exact: their whitelisted bodies
 * contain no interior branches, so the scan stops at the back-edge.
 *
 * Field preference is highest-first among 6..0: GCC's guest code leans on
 * cr0 and cr7, so the high middle fields are the ones most often free. */
/* Scan steps across the whole unit. A trace with regions is a small CFG, not
 * a line, so the walk needs a budget; exceeding it declines the optimization
 * (guards then spill and reload the guest CR, which is merely slower). */
#define JIT_TRACE_SCAN_MAX 4096u

static int guard_crf_pick(u32 pc, u32 *last_cr_idx)
{
    /* Work list of region entries still to scan. One slot per cold slot the
     * compiler could spend on a deferred branch, which is the same bound the
     * compiler works to; overflowing it declines the optimization rather than
     * leaving part of the unit unscanned. */
    struct { u32 pc; u32 lr; u8 lr_known; } q[JIT_MAX_COLD_PER_BLOCK];
    unsigned qn = 0, qi = 0, steps = 0;
    u8 touched = 0;
    int f, entry = 1;

    *last_cr_idx = 0;
    q[qn].pc = pc; q[qn].lr = 0; q[qn].lr_known = 0; qn++;

    while (qi < qn) {
        u32 p = q[qi].pc, lr = q[qi].lr;
        unsigned lr_known = q[qi].lr_known, n = 0;
        qi++;

        while (n < JIT_MAX_GUEST_INSTS) {
            u32 op, target;
            u8 t;

            if (steps++ >= JIT_TRACE_SCAN_MAX)
                return -1;

            op = mem_read32_for_fetch(p);
            t  = cr_fields_touched(op);
            if (t) {
                touched |= t;
                /* Only the entry region's index is meaningful: a region
                 * restarts guest_insts, which is what this is compared
                 * against. The comparison chooses between spilling the guest
                 * CR at a guard and merging it at the exits -- both exact,
                 * differing only in word count -- so an approximate index
                 * here cannot be wrong, only suboptimal. */
                if (entry)
                    *last_cr_idx = n;
            }
            if ((touched & 0x7F) == 0x7F)
                return -1;
            if (n + 1 >= JIT_MAX_GUEST_INSTS)
                break;

            if (OPCD(op) == 18) {
                if (!trace_follow_b(p, op, &target))
                    break;
                if (LK_BIT(op)) { lr = p + 4; lr_known = 1; }
                n++;
                p = target;
                continue;
            }

            if (OPCD(op) == 16) {
                if (!trace_inline_bc(p, op, &target))
                    break;
#if JIT_TRACE
                {   /* The deferred side may be compiled as a region of this
                     * same unit, so it must be scanned too. Whether the
                     * compiler actually opens one depends on budgets this
                     * scan cannot see -- it only ever opens FEWER, so
                     * scanning them all keeps this a superset. */
                    unsigned k;
                    for (k = 0; k < qn; k++)
                        if (q[k].pc == target && q[k].lr_known == lr_known &&
                            (!lr_known || q[k].lr == lr))
                            break;
                    if (k == qn) {
                        if (qn >= JIT_MAX_COLD_PER_BLOCK)
                            return -1;
                        q[qn].pc = target; q[qn].lr = lr;
                        q[qn].lr_known = (u8)lr_known; qn++;
                    }
                }
#endif
                n++;
                p += 4;
                continue;
            }

            if (trace_is_uncond_blr(op)) {
                if (!lr_known || !trace_follow_blr(lr))
                    break;
                n++;
                p = lr;
                lr_known = 0;
                continue;
            }

            if (is_block_terminator(op))
                break;

            if (trace_kills_lr(op))
                lr_known = 0;

            n++;
            p += 4;
        }
        entry = 0;
    }

    for (f = 6; f >= 0; f--)
        if (!(touched & (1u << f)))
            return f;
    return -1;
}

void jit_compile_into(JitContext *c, PPCState *s, u32 pc)
{
    c->state     = s;
    c->start_pc  = pc;
    c->pc        = pc;
    c->guest_insts = 0;
    c->fallbacks = 0;
    c->ended     = 0;
    c->failed    = 0;
    rc_reset(c);
    prov_reset(c);      /* provenance never crosses a block boundary */
    fpr_invalidate_all(c);
    cr_invalidate(c);
    /* Carry must start unloaded in every block. H_CARRY is a host register with
     * no meaning across a block boundary -- another block, or the dispatcher,
     * may have used it -- so a stale "loaded" flag would make the first adde in
     * a block read whatever happened to be there instead of the guest's actual
     * carry. */
    carry_invalidate(c);
    c->pin_gpr = c->pin_fpr = 0;
    c->link_count = 0;
    c->mmio_checks = 0;
    c->cold_count = 0;
    c->hot_words = 0;
    c->gqr_guarded = 0;
    c->guard_crf = (s8)guard_crf_pick(pc, &c->guard_last_cr_idx);
    c->guard_crf_stale = 0;
    c->lr_known  = 0;   /* nothing is known about LR at a unit's entry */
    c->trace_insts = 0;
    c->region      = 0;
    c->region_count = 0;
    c->sched_span_count = 0;
    wp_begin(c, 0);

    c->retaining       = 0;
    c->retain_idle     = 0;
    c->retain_aborted  = 0;
    c->retain_pin      = 0;
    c->retain_warm_off = 0;
    c->retain_ctr_host = -1;

    /* Warm continuity per-pass state. want_warm and the warm_guest/hostr/
     * dirtyf arrays with warm_count are caller-owned: the probe pass fills
     * them (warm_capture) and jit_compile_block feeds them back unchanged
     * into the warm pass. */
    c->warm_active    = 0;
    c->warm_candidate = 0;
    c->warm_off       = 0;
    c->warm_words     = 0;
    c->had_fallback   = 0;

    /* Loop retention preload: pull the loop's guest registers and CTR into the
     * register file once, pin them for the whole block, and mark the warm entry
     * just past here. The back-edge (below, in the bdnz handler) branches back
     * to it with everything already live. H_CARRY is free to hold CTR because a
     * retained loop is carry-free by construction (retain_safe_op). */
    if (c->want_retain) {
        u32 mask = 0;
        u8 kind = 0, idle = 0;
        if (loop_retention_scan(s, pc, &mask, &kind, &idle)) {
            unsigned g;
            c->retaining = 1;
            c->retain_kind = kind;
            c->retain_idle = idle;
            for (g = 0; g < 32 && !c->retain_aborted; g++) {
                if (!(mask & (1u << g)))
                    continue;
                (void)gpr_read(c, g);       /* load into a cache register */
                if (c->gpr[g].host >= 0)
                    c->retain_pin |= 1u << (unsigned)c->gpr[g].host;
            }
            if (kind == 0) {
                /* bdnz loop: the counter lives in a register for the whole
                 * loop. H_CARRY is free because the body is carry-free. */
                c->retain_ctr_host = H_CARRY;
                e_lwz(&c->e, H_CARRY, (s32)offsetof(PPCState, ctr), H_STATE);
            } else {
                /* cmp/bc loop: the condition register is loaded once here so
                 * the body's compare emits no per-iteration reload. The body
                 * writes CR before the back-edge reads it, and contains no
                 * other CR readers (the scan guarantees it), so the host CR
                 * stays live across the back-edge. */
                cr_ensure(c);
            }
            wp_point(c, 0);     /* the preload runs once per dispatch */
            c->retain_warm_off = (u32)(emit_mark(&c->e) - c->e.base);
            if (c->retain_aborted)
                return;                     /* caller recompiles plain */
        }
    }

    /* Warm self-loop prologue (the probe pass found the back edge; see the
     * "Warm self-loop continuity" section above). Load the recorded register
     * set into the exact host slots the body was observed to choose, pinned
     * for the whole block. Registers the body writes are marked dirty FROM
     * ENTRY: on iterations past the first they arrive at every interior sync
     * point carrying the previous iteration's values, so every compiled
     * spill set must already include them. On the first entry those extra
     * writebacks store the value just loaded -- redundant, never wrong. The
     * warm entry sits before the first guest instruction, where the
     * provenance lattice is still all-unknown: the same soundness argument
     * as retention's warm entry. */
    if (!c->retaining && c->want_warm && c->warm_count) {
        unsigned i;
        for (i = 0; i < c->warm_count; i++) {
            u32 g = c->warm_guest[i];
            int h = (int)c->warm_hostr[i];
            c->gpr[g].host   = (s8)h;
            c->gpr[g].loaded = 1;
            c->gpr[g].dirty  = c->warm_dirtyf[i];
            c->gpr[g].lru    = ++c->lru_clock;
            c->host_taken[h] = 1;
            c->retain_pin   |= 1u << (unsigned)h;
            e_lwz(&c->e, (u32)h, (s32)(offsetof(PPCState, gpr) + 4 * g),
                  H_STATE);
        }
        wp_point(c, 0);         /* the preload runs once per dispatch */
        c->warm_off    = (u32)(emit_mark(&c->e) - c->e.base);
        c->warm_active = 1;
    }

    compile_region(c);

    /* The deferred paths go after everything else -- and a deferred branch may
     * be compiled as another region of this same unit, which is why this is a
     * driver rather than a tail. The unit always ends in a branch, so nothing
     * falls into it. */
    emit_cold_tail(c);
    if (c->e.overflow)
        c->failed = 1;

    /* Finally, schedule the hot path for the in-order PPE.
     *
     * Everything above emits in the order the compiler *thinks* -- form an
     * address then use it, load a register then consume it, compare then
     * branch. That order costs nothing on an out-of-order host and is most of
     * the cost on this one: the static model (ppe_stall.h) measured most of
     * the modelled issue cycles lost to stalls over a real Mario Kart Wii
     * boot, more than half of them a load or an ALU result consumed before it
     * exists. This pass permutes each straight-line region against the
     * machine model; it moves no branch, changes no word count, and therefore
     * leaves every recorded offset -- link sites, warm entries, the cold
     * tail's patched fixups -- pointing where it did.
     *
     * It runs last so that it sees the final shape, cold tail included, and
     * it deliberately schedules only the words a traversal of the unit can
     * reach without leaving it: the unit's entry region (c->hot_words) and
     * each promoted REGION recorded during emit_cold_tail. The escape and
     * exit tails in between are never on the fast path and contain the one
     * thing that must not be decoded as an instruction -- the shared escape
     * stub's inline guest-pc data word. */
    if (g_jit_sched_enable && !c->failed && !c->e.overflow) {
        unsigned r;
        ppe_sched_block(&s_sched_work, c->e.base, c->hot_words,
                        c->warm_off ? c->warm_off : c->retain_warm_off);
        for (r = 0; g_jit_sched_regions && r < c->sched_span_count; r++)
            ppe_sched_block(&s_sched_work,
                            c->e.base + c->sched_span[r].from,
                            c->sched_span[r].to - c->sched_span[r].from, 0);
    }
}

/* One straight-line region of a compilation unit: the entry region, or the
 * target of a deferred conditional branch. Entered with the register caches
 * empty and memory authoritative, and with c->pc / c->guest_insts set. */
static void compile_region(JitContext *c)
{
    u32 window = c->region ? JIT_TRACE_REGION_INSTS : JIT_MAX_GUEST_INSTS;

    while (!c->ended && c->guest_insts < window &&
           c->trace_insts < JIT_TRACE_MAX_INSTS) {
        u32 op = mem_read32_for_fetch(c->pc);
        u32 *op_start = c->e.cur;   /* host words this guest insn costs */

        /* Hard size stop. Beyond this a cold slot's 14-bit guard branch could
         * no longer reach its tail (jit.h, trace formation budgets), and
         * e_patch_here truncates rather than diagnosing. */
        if (emit_size(&c->e) / 4 >= JIT_TRACE_HARD_WORDS)
            break;

        /* Pins only protect registers for the instruction being emitted. The
         * loop-retained registers stay pinned through retain_pin, not here. */
        c->pin_gpr = c->pin_fpr = 0;

        /* Trace formation, unconditional half: a direct branch does not end
         * the unit. The target is known at compile time, so compilation
         * simply *continues there*, and the branch itself costs nothing but
         * its LR store -- no register-cache flush, no exit tail, no
         * dispatcher round trip at run time. This is the largest single lever
         * on the expansion ratio: before it, `b` averaged 16 host words and
         * `bc` 27, and together they were half of all emitted code.
         *
         * Correctness notes, each of which is load-bearing:
         *   - The register cache stays live across the seam, which is the
         *     entire point. Nothing architectural happens at a branch.
         *   - guest_insts++ charges the branch against the downcount exactly
         *     as an exit would have.
         *   - A self-branch (`b .`) is the guest idling and must remain an
         *     ordinary block so the scheduler keeps running; following it
         *     would unroll 256 copies of nothing.
         *   - Backward targets unroll the loop body into the block, bounded
         *     by JIT_MAX_GUEST_INSTS. That is fewer dispatches, not more.
         *   - Blocks may now span pages; invalidation already drops the whole
         *     cache, so page-granular tracking is not relied on.
         *   - Retained-loop recompiles keep the old shape: the retention
         *     scanner's offsets assume it. */
        if (OPCD(op) == 18 && !c->retaining) {
            u32 target;
            if (c->guest_insts + 1 < JIT_MAX_GUEST_INSTS &&
                trace_follow_b(c->pc, op, &target)) {
                if (LK_BIT(op)) {
                    e_load_imm32_lo(&c->e, H_SCRATCH0, c->pc + 4);
                    e_stw(&c->e, H_SCRATCH0,
                          (s32)offsetof(PPCState, lr), H_STATE);
                    /* The store just made PPCState.lr equal to a constant
                     * this compilation unit knows, which is what lets the
                     * callee's `blr` below become a direct branch. */
                    c->lr_known = 1;
                    c->lr_const = c->pc + 4;
                }
                jit_attribute(op, (u32)(c->e.cur - op_start));
                c->guest_insts++;
                c->trace_insts++;
                c->pc = target;
                continue;
            }
        }

        /* Trace formation, return half: a `blr` whose LR the compiler has
         * been tracking as a compile-time constant is a direct branch in
         * disguise, so compilation continues at the return site.
         *
         * Following a call into a small leaf (above) removes the call's exit;
         * this removes the matching return's, which is the more expensive of
         * the two. An indirect exit cannot be block-linked, so every inlined
         * call still paid a full register flush, a pc store and a dispatcher
         * hash lookup to get back to its caller -- measured at 13.5 host
         * words per op19 across a boot, the most expensive guest instruction
         * in the histogram. Together the two halves put caller prefix,
         * callee body and caller suffix in ONE unit with ONE register-cache
         * lifetime and ONE exit.
         *
         * lr_const is exact, not predicted: it is set only where this unit
         * itself emitted the LR store, and cleared by anything that could
         * write LR afterwards. Branching to it is precisely what the guest's
         * `blr` would do -- and it costs no host instructions at all, since
         * LR's memory copy is already current and the constant is already
         * word-aligned (the guest's blr masks the low two bits; pc+4 has
         * them clear).
         *
         * The knowledge is consumed once. Keeping it across the return would
         * be equally correct -- LR really does still hold that value -- but a
         * function that returned twice without an intervening mtlr would then
         * walk the same code forever until the instruction budget stopped it,
         * emitting a pile of unreachable words for nothing. */
        if (trace_is_uncond_blr(op) && c->lr_known && !c->retaining &&
            trace_follow_blr(c->lr_const) &&
            c->guest_insts + 1 < JIT_MAX_GUEST_INSTS) {
            jit_attribute(op, (u32)(c->e.cur - op_start));
            c->guest_insts++;
            c->trace_insts++;
            c->pc = c->lr_const;
            c->lr_known = 0;
            continue;
        }

        /* Superblock formation, conditional half: a forward `bc` that only
         * tests CR keeps compiling its fall-through *inline*, with the taken
         * side deferred to a cold exit exactly the way MMIO bail-outs are.
         * The hot path pays the CR test and one conditional branch -- two or
         * three words -- where it used to pay a full double exit of ~27.
         *
         * Only *forward* targets: a backward `bc` is a loop back-edge, where
         * the taken side is the hot one and the retained-loop machinery
         * already compiles the hottest shapes specially. Skipped too: bcl
         * (rare, needs the LR store), CTR forms, and BO patterns that ignore
         * CR (those are unconditional; op18's path covers the idea).
         *
         * cr_ensure comes before the snapshot on purpose -- the cold path
         * inherits cr_loaded/cr_dirty as they stand at the branch, and the
         * fall-through continues with the same guest CR live in the host CR,
         * untouched by the test.
         *
         * "Deferred" is not "cold". The compiler has no idea which way the
         * branch goes, and the deferred side is taken every time round MKWii's
         * hot loop. So the slot recorded here is a candidate for becoming a
         * REGION of this same unit rather than an exit from it -- see
         * emit_region_seam, which is where that is decided, once the unit's
         * final size is known. */
        if (OPCD(op) == 16 && !c->retaining) {
            u32 bo = BO(op), bi = BI(op);
            u32 target;
            if (trace_inline_bc(c->pc, op, &target) &&
                c->cold_count < JIT_MAX_COLD_PER_BLOCK &&
                c->guest_insts + 1 < JIT_MAX_GUEST_INSTS) {
                unsigned n = c->cold_count++;

                cr_ensure(c);
                /* Carry the guest compiler's own prediction across. The host
                 * branch is taken exactly when the guest's is, and the target
                 * here is always forward in guest space, so the guest hints
                 * "taken" precisely when its y bit is set -- and the host
                 * branch, also forward, needs its own y bit set to agree.
                 * Dropping this bit made every hinted-taken guest branch a
                 * mispredict on the fall-through superblock layout. */
                c->cold[n].from = e_bc_fwd(&c->e,
                                           ((bo & 0x08) ? BO_TRUE : BO_FALSE)
                                             | (bo & BO_HINT),
                                           bi);
                c->cold[n].kind     = COLD_BRANCH;
                g_jit_cold_site[3]++;
                c->cold[n].guest_pc = target;
                c->cold[n].insts    = c->guest_insts;
                memcpy(c->cold[n].gpr, c->gpr, sizeof c->gpr);
                memcpy(c->cold[n].fpr, c->fpr, sizeof c->fpr);
                memcpy(c->cold[n].host_taken, c->host_taken,
                       sizeof c->host_taken);
                c->cold[n].cr_loaded    = c->cr_loaded;
                c->cold[n].cr_dirty     = c->cr_dirty;
                c->cold[n].guard_crf_stale = c->guard_crf_stale;
                c->cold[n].carry_loaded = c->carry_loaded;
                c->cold[n].carry_dirty  = c->carry_dirty;
                c->cold[n].lr_known = c->lr_known;
                c->cold[n].lr_const = c->lr_const;
                wp_cold(c, n);

                jit_attribute(op, (u32)(c->e.cur - op_start));
                c->guest_insts++;
                c->trace_insts++;
                c->pc += 4;
                continue;
            }
        }

        if (is_block_terminator(op)) {
            if (!compile_branch(c, op)) {
                if (c->retaining) { c->retain_aborted = 1; return; }
                emit_fallback(c, op);
                /* The interpreter set npc; leave through it. */
                e_lwz(&c->e, H_SCRATCH0, (s32)offsetof(PPCState, npc), H_STATE);
                emit_exit_reg(c, H_SCRATCH0, c->guest_insts + 1);
            }
            if (c->retain_aborted) return;
            jit_attribute(op, (u32)(c->e.cur - op_start));
            c->guest_insts++;
            c->trace_insts++;
            c->pc += 4;
            c->ended = 1;
            break;
        }

        {
            /* Provenance: computed from the pre-instruction lattice, applied
             * only if the native path is actually taken below. gpr_dest and
             * gpr_write have already killed whatever the instruction writes,
             * so not committing (fallback path) is conservative, and
             * emit_fallback wipes the lattice anyway -- the interpreter may
             * write any register. */
            ProvUpdate pu[2];
            unsigned pu_n = prov_transfer(c, op, pu);
            if (compile_one(c, op)) {
                unsigned k;
                for (k = 0; k < pu_n; k++)
                    prov_set(c, pu[k].reg, pu[k].kind, pu[k].value,
                             pu[k].span);
                goto compiled_native;
            }
        }
        {
            /* A retained loop must be fallback-free -- a fallback invalidates
             * the whole register cache, destroying the live retained state. The
             * scan should have excluded this, but honour it defensively: abort
             * and let jit_compile_block recompile the block without retention. */
            if (c->retaining) { c->retain_aborted = 1; return; }
            emit_fallback(c, op);

            /* A fallback runs an interpreter handler, which may raise an
             * exception, request a syscall, or otherwise ask to leave the
             * block *in the middle of it* -- an alignment fault, an FP
             * exception, a program exception on an illegal opcode. The
             * interpreter notices at its next step boundary; a compiled block
             * has no such boundary and would keep executing the following
             * instructions. So after every fallback, check whether the handler
             * asked to stop and, if so, leave through npc.
             *
             * Missing this is invisible in most code -- the following
             * instructions usually just re-raise or compute dead values -- but
             * it is a real divergence from the interpreter, and it is exactly
             * what the big-endian differential harness caught. */
            emit_exit_if_requested(c);
        }
compiled_native:

        if (c->retain_aborted) return;      /* allocator ran out under pinning */

        jit_attribute(op, (u32)(c->e.cur - op_start));
        c->guest_insts++;
        c->trace_insts++;
        c->pc += 4;

        if (c->e.overflow) {
            c->failed = 1;
            return;
        }
    }

    /* Ran off the end of the window without hitting a branch. */
    if (!c->ended)
        emit_exit_to(c, c->pc, c->guest_insts);
}

double jit_expansion_ratio(void)
{
    if (g_jit_stats.guest_insts_compiled == 0)
        return 0.0;
    return (double)g_jit_stats.host_insts_emitted /
           (double)g_jit_stats.guest_insts_compiled;
}
