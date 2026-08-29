/* jit.h — PPC32 (Gekko) -> PPC64 (Cell PPE) block recompiler.
 *
 * The design follows directly from docs/HARDWARE.md, and three of its numbers
 * dominate every decision here:
 *
 *   L1 is write-through, no-write-allocate  -> every store costs an L2 write
 *                                              (41 cycles), so a guest register
 *                                              is loaded once and stored once
 *                                              per *block*, never per
 *                                              instruction, and never spilled.
 *   Branch mispredict is 24 cycles          -> blocks are linked with direct
 *                                              branches; indirect dispatch is
 *                                              the exception, not the rule.
 *   The core is in-order, 2-issue           -> instruction order matters; the
 *                                              hardware will not rescue a
 *                                              badly scheduled dependent pair.
 *
 * Completeness strategy: every instruction the compiler does not have a native
 * path for falls back to a call into the interpreter. That makes the JIT
 * *correct and complete from the first block*, and lets native paths be added
 * where profiling says they matter rather than all at once. A fallback is slow
 * but never wrong, which is the right way round.
 */
#ifndef DOLPHIN_CORE_PPC_JIT_H
#define DOLPHIN_CORE_PPC_JIT_H

#include "../gekko.h"
#include "ppc_emitter.h"

/* ------------------------------------------------------------------ */
/* Blocks                                                              */
/* ------------------------------------------------------------------ */

/* A place in a block's code where control leaves for another block at a
 * statically known guest address. Initially a fall-through into the dispatcher;
 * patched into a direct branch once the target has been compiled. Removing that
 * dispatcher round trip -- a C call plus a hash lookup -- is the single largest
 * performance item in the recompiler, and it matters doubly on a core whose
 * branch mispredict costs 24 cycles. */
#define JIT_MAX_LINKS_PER_BLOCK 12

typedef struct {
    u32 word_offset;        /* index into block->code of the branch  */
    u32 target_pc;          /* guest address it goes to              */
} JitLinkSite;

typedef struct JitBlock {
    u32  guest_pc;          /* address this block starts at            */
    u32  guest_end;         /* one past the last guest instruction     */
    u32  msr_key;           /* MSR bits the code was compiled under    */
    u32 *code;              /* host code                               */
    u32  code_words;
    /* Words before the cold tail: the part actually fetched on the fast path.
     * MMIO bail-outs are branched over, so total size overstates the cost. */
    u32  hot_words;
    /* Warm self-loop blocks only: host words on the per-iteration fast path
     * (warm entry through the back-edge branch). 0 for ordinary blocks. Used
     * by the JIT_PROFILE executed-word estimate, which would otherwise charge
     * every iteration the full block including the one-time prologue/exits. */
    u32  warm_words;
    u32  guest_insts;       /* for the expansion-ratio metric          */
    u32  hits;              /* execution count, for profiling          */

    JitLinkSite links[JIT_MAX_LINKS_PER_BLOCK];
    u8   link_count;

    struct JitBlock *hash_next;
    struct JitBlock *page_next;   /* chain of blocks on one guest page */
} JitBlock;

/* ------------------------------------------------------------------ */
/* Statistics — the raw material for the "is it at the machine's limit"  */
/* question (docs/ARCHITECTURE.md §8.2). Cheap enough to always collect. */
/* ------------------------------------------------------------------ */

typedef struct {
    u64 blocks_compiled;
    u64 guest_insts_compiled;
    u64 host_insts_emitted;
    u64 fallback_insts;         /* compiled as an interpreter call */
    u64 dispatch_lookups;
    u64 cache_flushes;
    u64 code_bytes_used;
    u64 links_emitted;
    u64 links_resolved;   /* patched to a direct branch */

    /* Time spent *compiling*, in PPE time-base ticks (mftb, 79.8 MHz on
     * retail hardware; zero on hosts that have no time base). Compilation
     * is invisible to the phase profiler -- it is charged to PH_JIT along
     * with execution -- yet it arrives in bursts: entering a new scene
     * after a load compiles thousands of cold blocks inside a handful of
     * slices, and a burst that takes longer than a frame is a hitch the
     * player sees. These two counters are what turn "compilation might be
     * hurting" into a measured number of microseconds per boot. */
    u64 compile_ticks;
    u64 compile_ticks_max;      /* the single longest compile */

    /* Fallbacks bucketed by guest primary opcode (0..63), and separately by the
     * extended opcode of the two primaries that pack many instructions -- 31
     * (integer/system) and 63 (double float). This is what turns "something
     * falls back" into "implement exactly these next", with no disassembler and
     * no console debugger. */
    u64 fallback_by_opcd[64];
    u64 fallback_x31_xo[1024];
    u64 fallback_x63_xo[1024];
} JitStats;

extern JitStats g_jit_stats;

/* Mean host instructions emitted per guest instruction. The theoretical floor
 * is 1.0; anything much above ~1.4 means the fast paths are not covering the
 * instruction mix this title actually executes. */
double jit_expansion_ratio(void);

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

int  jit_init(size_t code_bytes);
void jit_shutdown(void);

/* Drop every compiled block. Triggered by a code-cache overflow, or by the
 * guest invalidating instruction memory on a scale that makes per-page
 * invalidation pointless. */
void jit_flush_all(void);

/* Lay the MMIO bail-out tails in a separate arena at the far end of the code
 * cache instead of inline after each block. Set before jit_init; changing it
 * afterwards has no effect until the next init. See the commentary on
 * JitContext::ce for why. */
extern int g_jit_cold_split;
size_t jit_cold_used(void);

/* Invalidate blocks overlapping [addr, addr+len). Called when the guest writes
 * to memory that has been compiled -- self-modifying code, overlays, and DMA
 * into code pages all rely on this being right. */
void jit_invalidate_range(u32 addr, u32 len);

/* Find the block for the current PC/MSR, compiling it if necessary.
 * Returns NULL only if compilation failed, in which case the caller should run
 * the interpreter for one instruction and try again. */
JitBlock *jit_get_block(PPCState *s, u32 pc);

/* Compile a single block starting at `pc`. Exposed for the verification tools,
 * which compile blocks and disassemble them without executing. */
JitBlock *jit_compile_block(PPCState *s, u32 pc);

/* Run compiled code until the downcount expires or an exit is requested. */
void jit_run(PPCState *s);

/* The dispatch loop's C half: returns the host code address of the next block,
 * or NULL to leave the loop. Called from jit_entry.S; declared here so the
 * host-side tests can drive it directly. */
void *jit_dispatch_c(PPCState *s);

/* ------------------------------------------------------------------ */
/* AOT — statically recompiled guest functions                         */
/*                                                                     */
/* An AOT function is a guest function recompiled to native code ahead */
/* of time (tools/rec). Signature and contract:                        */
/*                                                                     */
/*     u32 aot_<hexaddr>(PPCState *s);                                 */
/*                                                                     */
/* Entered with s->pc at the function's guest entry; runs the whole    */
/* guest function natively, internal branches included; at every guest */
/* exit (blr) it leaves s->pc at the guest return target and returns   */
/* the number of GUEST instructions executed. It must be bit-exact     */
/* with the interpreter for any initial state.                         */
/*                                                                     */
/* The dispatcher consults the AOT table before block lookup, so a     */
/* registered entry pre-empts JIT compilation at that pc. Interrupts   */
/* are delivered once per slice at jit_run entry, so whole-function    */
/* execution without mid-function interrupt checks is equivalent to a  */
/* JIT superblock by design.                                           */
/* ------------------------------------------------------------------ */

typedef u32 (*JitAotFn)(PPCState *s);

/* Optional cost estimator, consulted at DISPATCH time, before the
 * function runs. It must return a CONSERVATIVE (>= actual) estimate of
 * the guest instructions the function would execute, derived only from
 * the entry state -- argument registers, and guest memory read without
 * side effects (RAM only; anything unreadable must estimate UINT32_MAX
 * and thereby decline). The dispatcher enters the AOT function only
 * when the estimate fits in the remaining downcount; otherwise it
 * falls through to the ordinary JIT path for that invocation, which
 * yields at block boundaries -- always correct, merely slower. This is
 * the slice cap: an AOT function cannot be suspended mid-body, so a
 * function whose work would cross the slice edge must not start. */
typedef u32 (*JitAotEstFn)(const PPCState *s);

/* Capacity of the open-addressed table. Registrations are capped below
 * this (JIT_AOT_MAX_ENTRIES) so a lookup miss always terminates at an
 * empty slot after a short probe. */
#define JIT_AOT_SLOTS       64
#define JIT_AOT_MAX_ENTRIES (JIT_AOT_SLOTS - (JIT_AOT_SLOTS / 4))

/* Store an entry. Storage only: nothing is dispatched until
 * jit_aot_enable_all(). Re-registering a pc replaces its function; a
 * full table drops the registration with a warning (slower, never
 * wrong). May be called before jit_init — the table is independent of
 * the code cache and survives jit_flush_all/jit_shutdown. */
void jit_aot_register(u32 guest_pc, JitAotFn fn);

/* Like jit_aot_register, but with an explicit machine-state gate: the
 * dispatcher enters the function only when
 *     (block_key(s) & key_mask) == key_want
 * where block_key is MSR & (IR|DR|PR|FP) plus 0x80000000 for
 * HID2[PSE]. jit_aot_register uses the strict full-state key (an FP
 * function entered with MSR[FP] clear would skip the FP-unavailable
 * trap); integer-only functions can mask FP and PSE out, which is what
 * makes them reachable on MKWii's FP-disabled threads. */
void jit_aot_register_key(u32 guest_pc, JitAotFn fn, u32 key_mask, u32 key_want);

/* Like jit_aot_register_key, with a cost estimator (may be NULL, which
 * is exactly jit_aot_register_key). Functions whose run time is
 * caller-controlled (memcpy-alikes, decompressors) must register with
 * an estimator so a long invocation is capped at dispatch time instead
 * of running many slices' worth of work in one dispatch. */
void jit_aot_register_est(u32 guest_pc, JitAotFn fn, u32 key_mask, u32 key_want,
                          JitAotEstFn est);

/* The code-integrity gate. Registration merely stores; the embedder
 * calls this AFTER the guest code the functions were recompiled from
 * has been loaded, at which point every stored entry goes live. Also
 * flushes the code cache: blocks compiled (and block-to-block linked)
 * before enabling were built under the "no AOT" linking rule and would
 * jump straight past the dispatcher; rebuilding them under the
 * link-refusal rule (jit_compile: exits targeting an AOT pc stay on
 * the dispatcher path) is what makes direct calls reach AOT. */
void jit_aot_enable_all(void);

/* Top-N hottest blocks via callback (armed by JIT_PROFILE / the console file). */
void jit_profile_report(void (*out)(const char *), unsigned topn);
void jit_profile_reset(void);

/* Kill switch: stop dispatching to AOT functions (e.g. a disc-DMA
 * overwrite of guest code would invalidate what they were compiled
 * from). Entries stay stored; jit_aot_enable_all() re-arms them. */
void jit_aot_disable(void);

/* The function registered for guest_pc, or NULL if absent or AOT is
 * not enabled. Mirrors exactly what the dispatcher would call. */
JitAotFn jit_aot_lookup(u32 guest_pc);

/* Whether a compiled block already exists for `pc` under the machine state
 * `s` is in. The compiler's question, at trace-formation time: a deferred
 * branch target that is ALREADY a block does not need to be duplicated into
 * this unit -- the exit can link straight to the existing code for four
 * words. Inlining it anyway is what makes regions expensive in the 8 MiB
 * console code cache, because a target reached from many places is otherwise
 * copied into every unit that reaches it. */
int jit_block_compiled(const PPCState *s, u32 pc);

/* How full the code cache is, 0..100. Trace formation trades code size for
 * executed words -- a region is a second copy of code that exists elsewhere --
 * and on the console the cache is 8 MiB of .text whose overflow flushes
 * EVERYTHING. So the trade is made only while there is room: past the
 * threshold, deferred branches go back to being ordinary exits and units stop
 * growing, which keeps regions from ever being the reason the cache overflows.
 * Blocks compiled under pressure are correct and merely shaped like the
 * pre-trace ones. */
unsigned jit_code_pressure(void);

/* Whether the table has an entry registered at guest_pc, enabled or
 * not. This is the compiler's query, not the dispatcher's: jit_compile
 * must neither follow an unconditional branch into such a pc (superblock
 * formation) nor patch a block-to-block link at one, or direct calls
 * would bypass the dispatcher and the AOT function would never run.
 * Registration-keyed (unlike jit_aot_lookup) so code compiled between
 * registration and jit_aot_enable_all already obeys the rule. With an
 * empty table this is one load and a predicted branch. */
int jit_aot_owns_pc(u32 guest_pc);

/* Instrumentation: how many times the dispatcher entered the AOT
 * function registered at guest_pc (0 if absent). Counts dispatches,
 * not guest instructions. */
u64 jit_aot_hit_count(u32 guest_pc);

/* Instrumentation: how many times the estimator at guest_pc declined a
 * dispatch (estimate did not fit the remaining downcount), sending that
 * invocation down the ordinary JIT path instead. 0 if absent. */
u64 jit_aot_declined_count(u32 guest_pc);

/* Guest instructions an AOT function executed past the end of its
 * slice, not yet charged to the clock. A whole-function execution can
 * overshoot the downcount by far more than a JIT block would; the
 * dispatcher zeroes the downcount, parks the overshoot here, and pays
 * it down at the start of the following slice(s) so AOT work is never
 * clamped away as free cycles by timing_advance. */
s32 jit_aot_debt_pending(void);

/* ------------------------------------------------------------------ */
/* Shared escape tail                                                   */
/*                                                                      */
/* Every guarded memory access, every GQR-format guard and the fcmp NaN   */
/* arm needs a way out to the interpreter. Emitting that way out per site */
/* is what made bail-out tails 38.6% of all compiled words: twelve host   */
/* words each, of which nine are the *same nine every time* -- store the  */
/* faulting pc, raise force_interp, spill the cycle budget, jump to the   */
/* dispatcher. They are never executed, but they pad every block, they    */
/* push blocks out of the code cache (each overflow flushes ALL compiled  */
/* code), and they cost memory bandwidth to write.                       */
/*                                                                       */
/* So those nine words are emitted ONCE for the whole code cache and a    */
/* site's tail becomes                                                   */
/*                                                                       */
/*     <the register write-backs this point actually needs>              */
/*     addi  rDOWNCOUNT, rDOWNCOUNT, -insts                              */
/*     bl    escape_tail          ; LR now points at the next word       */
/*     .long guest_pc             ; data, read through LR, never fetched */
/*                                                                       */
/* The write-backs stay in the site because they are what differs between */
/* sites, and because they are what the escape actually *executes* -- an  */
/* MMIO-heavy title bails out millions of times per boot, so the escape   */
/* path must not get slower. It does not: the site executes exactly the   */
/* stores it did before, plus one `bl`. Everything removed is the         */
/* boilerplate that was identical everywhere.                            */
/*                                                                       */
/* (A denser variant was built and measured first: two words per site,    */
/* with a stub that dumped the whole register cache blind and a C helper  */
/* that moved the live entries per a side-table descriptor. It reached    */
/* 16.1% of all emitted words instead of 9.2% -- and cost 4.6% of boot    */
/* wall-clock, because 16.7M escapes each paid ~45 extra instructions.    */
/* Static size is not worth executed time.) */
/* ------------------------------------------------------------------ */

/* Address of the shared tail in the code cache, or NULL if it could not be
 * emitted. Re-emitted at the head of the cache after every flush, because a
 * flush resets the bump pointer and takes the tail with it. */
void *jit_esc_stub(void);

/* ------------------------------------------------------------------ */
/* Register cache                                                      */
/*                                                                     */
/* A guest register is loaded into a host register on first use in a block and  */
/* written back at block end if modified. Because the PPE's L1 does not absorb  */
/* writes, this single decision is worth more than any peephole optimization    */
/* elsewhere in the compiler.                                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    s8  host;               /* host GPR holding it, or -1              */
    u8  dirty;              /* written since being loaded              */
    u8  loaded;             /* host register holds the current value   */
    u32 lru;                /* logical time of last access             */
} GprSlot;

/* Floating-point cache slot. Gekko's FPRs are *pairs* (ps0, ps1), and the two
 * halves are cached independently: scalar code touches only ps0, so caching
 * whole pairs would waste half of a scarce resource. Indexed by
 * (guest_fpr * 2 + half). */
typedef struct {
    s8  host;               /* host FPR holding it, or -1        */
    u8  dirty;
    /* Value is known to be exactly a single (<= 24 significant bits), so the
     * Gekko's 25-bit rounding of a multiply's C operand is the identity and
     * can be skipped. Set by everything that produces a single -- lfs, psq_l,
     * every paired op, single-precision scalar results, frsp -- inherited by
     * moves, and cleared by any write of unknown provenance, including a
     * (re)load from PPCState at block entry. Dolphin's fprIsSingle. */
    u8  single;
    u32 lru;
} FprSlot;

/* Guarded accesses per block. A block that exceeds this keeps its remaining
 * bail-outs inline, which is slower but correct -- never a compilation
 * failure. */
#define JIT_MAX_COLD_PER_BLOCK 24

/* What a deferred cold path is for. MMIO escapes hand one instruction to the
 * interpreter; branch exits are the taken side of a conditional branch whose
 * fall-through continues inline in the block (superblock formation). */
#define COLD_MMIO   0
#define COLD_BRANCH 1

/* Trace formation budgets.
 *
 * A deferred conditional branch does not have to leave the compilation unit.
 * Its target can be compiled as a further REGION of the same unit, reached by
 * the guard branch that already exists -- no exit tail, no pc store, no
 * dispatcher round trip, no second block with its own prologue. What the
 * region does pay is a canonical entry: the register caches are flushed and
 * emptied at the seam, so the region compiles against the same all-in-memory
 * state a block entry has. That is the whole correctness argument, and it is
 * why regions need no state reconciliation.
 *
 * The bounds exist for one hard reason beyond code-cache pressure: a cold
 * slot's guard branch is a B-form `bc`, whose displacement field is 14 bits
 * (+-32 KiB), and e_patch_here truncates silently rather than diagnosing. So
 * the unit is kept small enough that the LAST cold tail is still in range of
 * the FIRST guard branch. Worst case with these numbers is
 * 4500 (hard stop) + one instruction (<64) + 24 tails * 120 words = 7440
 * words = 29.8 KiB, comfortably inside the field. */
#ifndef JIT_TRACE_REGION_WORDS
#define JIT_TRACE_REGION_WORDS  1200u   /* do not open a region past this  */
#endif
#ifndef JIT_TRACE_HARD_WORDS
#define JIT_TRACE_HARD_WORDS    4500u   /* end the current region here     */
#endif
#ifndef JIT_TRACE_MAX_INSTS
#define JIT_TRACE_MAX_INSTS     320u    /* guest instructions per unit     */
#endif
#ifndef JIT_TRACE_REGION_INSTS
/* Guest instructions a single region may compile. The entry region gets the
 * unit's full window; a region is a DUPLICATE of code that exists (or will
 * exist) elsewhere, so it is worth exactly as much as the exit it removes
 * plus the seam it saves -- a bound that does not grow with its length. */
#define JIT_TRACE_REGION_INSTS  64u
#endif
#ifndef JIT_TRACE_INLINE_ONCE
/* Refuse a region whose target is ALREADY a compiled block, on the theory
 * that the exit can link to the existing code for four words instead of
 * duplicating it. Measured on the boot, and off by default because the
 * measurement says so: it saves 0.3 MiB of the 10.6 MiB footprint and costs
 * 1.1 points of the executed-word reduction (14.2% instead of 15.3% at the
 * console's cache size). Left as a knob because that trade inverts the moment
 * code size, rather than executed words, is the binding constraint. */
#define JIT_TRACE_INLINE_ONCE   0
#endif
#ifndef JIT_TRACE_PRESSURE_PCT
/* Code-cache occupancy past which no new region is opened. */
#define JIT_TRACE_PRESSURE_PCT  70u
#endif
#ifndef JIT_TRACE_MAX_REGIONS
/* Regions duplicate: a target reached from several places is compiled into
 * each unit that reaches it. That is the trade -- one traversal costs fewer
 * words, the cache holds fewer distinct traversals -- and the code cache is
 * 8 MiB of .text on the console, where an overflow flushes EVERYTHING and
 * pays for it in recompilation. So fan-out is capped per unit, which bounds
 * duplication where it grows fastest without touching the common shapes
 * (one or two deferred branches). */
#define JIT_TRACE_MAX_REGIONS   6u
#endif

/* Pad each unit entry to a 32-byte fetch group. Off: see jit.c for why
 * (it perturbs jit_code_pressure and therefore trace formation). */
#ifndef JIT_BLOCK_ALIGN
#define JIT_BLOCK_ALIGN 0
#endif

#define PS0 0
#define PS1 1
#define FSLOT(fpr, half) ((fpr) * 2 + (half))

/* Constant address provenance: what the compiler knows about the low 32 bits
 * of a guest GPR at the current point in the block, tracked as a small
 * abstract-value lattice and updated as instructions compile.
 *
 *   PROV_CONST   the full 32-bit value is a compile-time constant
 *   PROV_UPPER   the upper 16 bits are a known constant, the low 16 unknown
 *   PROV_RANGE   the value is (value + k) mod 2^32 for some k in [0, span] --
 *                zero-extended loads, mask results, sign-extended halfwords
 *                (a wrapped interval), bounded index arithmetic
 *   PROV_UNKNOWN nothing is known
 *
 * The only consumer is guard placement at guarded memory accesses: the MMIO
 * guard tests bits 25..29 of the unfolded address, so a base whose possible
 * values all land outside the aperture needs no guard and no cold bail-out
 * tail, and one whose values all land inside it can skip the test and go
 * straight to the escape. Block entry state is all-unknown -- provenance never
 * crosses a block boundary -- and any write the compiler does not model
 * returns the register to unknown. */
#define PROV_UNKNOWN 0
#define PROV_CONST   1
#define PROV_UPPER   2
#define PROV_RANGE   3

typedef struct {
    u8  kind;               /* PROV_* */
    u32 value;              /* CONST: the value. UPPER: low 16 bits zero.
                             * RANGE: the interval's low end. */
    u32 span;               /* RANGE only: interval length (hi - lo) */
} ProvSlot;

typedef struct {
    PPCEmitter  e;

    /* Second emitter for the cold bail-out tails.
     *
     * Measured over 20,000 blocks of Mario Kart Wii's real .text: the hot path
     * is a healthy 1.79 host instructions per guest instruction, but the cold
     * MMIO tails are 129.8 words per block -- 64% of everything the recompiler
     * emits. Laid out inline they sit between one block's hot code and the
     * next's, so the PPE's 32 KB instruction cache spends most of its capacity
     * on words that are branched over, and the 512 KB L2 (which also has to
     * hold the guest's working set) carries them too.
     *
     * They are rare at run time -- a provably-MMIO base already compiles to a
     * direct path, so bail-outs run only a few hundred times a frame -- which
     * is exactly what makes them worth exiling rather than shrinking first.
     * `ce` writes into a separate arena at the far end of the code cache; hot
     * code from consecutive blocks then lies contiguous. */
    PPCEmitter  ce;
    int         cold_split;     /* ce is a real second arena, not an alias */
    PPCState   *state;

    u32         start_pc;
    u32         pc;             /* guest pc being compiled       */
    u32         guest_insts;
    u32         fallbacks;

    GprSlot     gpr[32];
    ProvSlot    prov[32];       /* constant provenance, per guest GPR */
    u8          host_taken[32]; /* host GPRs currently allocated */
    u32         lru_clock;      /* monotonic tick for LRU eviction */

    FprSlot     fpr[64];            /* (guest_fpr, half) -> host FPR   */
    u8          host_fpr_taken[32];
    u32         fpr_lru_clock;

    /* Host registers that must not be evicted while the current guest
     * instruction is being emitted. Without this, allocating a destination can
     * evict one of the sources whose host register number the caller is still
     * holding -- producing an instruction that reads the wrong value. Reachable
     * whenever a block touches more registers than the cache has slots.
     * Cleared before each guest instruction. */
    u32         pin_gpr;
    u32         pin_fpr;

    u32         mmio_checks;    /* MMIO guards emitted, for statistics */

    /* Deferred MMIO bail-outs.
     *
     * A guarded access branches over its bail-out on the fast path, so the
     * bail-out costs no executed instructions -- but it sits in the middle of
     * hot code, and it is large: register spills, the escape sequence and the
     * dispatch. Across a realistic block those bail-outs are roughly half the
     * emitted code, which is half the instruction cache filled with lines that
     * never execute. The PPE has 32 KiB of it and no second chance.
     *
     * So each bail-out is recorded here and emitted after the block's hot code,
     * with the guard's branch patched to reach it. Emitting it later means
     * replaying the register-cache state it was generated under, which is why
     * the snapshot is taken rather than just the address. */
    struct {
        PPCFixup from;          /* the guard's branch, to be patched      */
        u8       kind;          /* COLD_MMIO or COLD_BRANCH               */
        u32      guest_pc;      /* MMIO: faulting pc.  Branch: target pc  */
        u32      insts;         /* guest instructions completed before it */
        GprSlot  gpr[32];
        FprSlot  fpr[64];
        u8       host_taken[32];
        u8       cr_loaded, cr_dirty;
        u8       guard_crf_stale;
        u8       carry_loaded, carry_dirty;
        /* Trace formation: the known-LR state at the branch, so a region
         * compiled from this slot can still turn its callee's `blr` into a
         * direct branch. */
        u8       lr_known;
        u32      lr_const;
#ifdef JIT_WORDPROF
        s32      wp;            /* host words executed to reach this site */
#endif
    }           cold[JIT_MAX_COLD_PER_BLOCK];
    u8          cold_count;

    /* Words emitted before the cold tail begins -- the part of the block that
     * is actually fetched and executed. Total emitted size stopped being a
     * useful cost measure once bail-outs moved out of line: it counts code that
     * never runs. This is the number to optimise against. */
    u32         hot_words;

    /* GQRs already validated in this block. Quantized load/store is compiled
     * against the GQR value seen at compile time, guarded by a runtime check;
     * the check only has to happen once per GQR per block because nothing
     * inside a block can change one (mtspr exits to the interpreter). */
    u8          gqr_guarded;

    /* Link sites emitted while compiling; copied into the finished block. */
    JitLinkSite link[JIT_MAX_LINKS_PER_BLOCK];
    u8          link_count;

    /* The guest CR is mirrored into the *host* CR, but only for blocks that
     * actually use it. Loading and spilling it unconditionally costs four
     * instructions on every block, which on short blocks is a large fraction
     * of the total -- and a great many blocks never touch a condition code. */
    u8          cr_loaded;      /* host CR currently mirrors the guest CR  */
    u8          cr_dirty;       /* host CR holds changes not yet spilled   */

    /* Host CR field the MMIO/GQR guards may compare into without spilling
     * the guest CR first, or -1 if every field is (potentially) live.
     *
     * Chosen once per block by a pre-scan (guard_crf_pick): a field the
     * block's guest code provably neither reads nor writes natively can be
     * clobbered freely, so a guard in CR-live code costs zero extra words
     * instead of the mfcr+stw spill before it and the lwz+mtcrf reload at
     * the next guest CR use. guard_crf_used records that a guard has
     * actually been emitted this way, at which point cr_store must merge
     * the authoritative memory copy of that one field over the junk the
     * guard left in the host CR (see cr_store_via).
     *
     * guard_crf_stale narrows the merge to where it is actually needed: it
     * is set when a guard clobbers the field and cleared when cr_ensure's
     * full `mtcrf 0xFF` reload overwrites the whole host CR with the memory
     * copy -- after that the host field is correct again and a plain store
     * suffices. Snapshotted per cold exit, since a deferred bail-out runs
     * under the staleness of its guard site, not of the block's end. */
    s8          guard_crf;
    u8          guard_crf_stale;
    /* Traversal index (in guest instructions, same counting as guest_insts)
     * of the last instruction in the block that touches CR at all. A guard
     * emitted at or past this index knows the dirty CR can only be stored
     * from here on, never rewritten -- so spilling it at the guard (host CR
     * left intact, no reload ever needed) keeps the block's exit stores
     * plain instead of merged. */
    u32         guard_last_cr_idx;

    /* Guest XER[CA], cached in H_CARRY across the block. A long-arithmetic
     * chain is a run of adde/subfe sharing only the carry, so spilling it
     * between each pair would put two memory accesses into what is otherwise
     * pure register arithmetic. */
    u8          carry_loaded;
    u8          carry_dirty;

    /* Guest registers whose folded host addresses are currently live in the
     * address-base registers, or -1. A slot is invalidated when its guest
     * register is written or a helper call clobbers the volatile registers.
     *
     * There is more than one slot because a single one thrashes: ordinary
     * compiled code alternates between the stack pointer, the frame pointer
     * and a data pointer, and with one slot every switch re-ran the MMIO guard
     * and re-folded the base -- five instructions each, and in a plain
     * function prologue/epilogue that was 40% of the emitted block. */
    s8          addr_base_guest[JIT_ADDR_BASE_SLOTS];
    u8          addr_base_lru[JIT_ADDR_BASE_SLOTS];
    u8          addr_base_clock;

    int         ended;          /* a terminating instruction was emitted */
    int         failed;

    /* Trace formation: the guest LR as a compile-time constant.
     *
     * Set where this compilation unit itself emitted the LR store for a call
     * it decided to follow, and cleared by anything that could write LR
     * afterwards -- a native mtlr, or an interpreter fallback, which may
     * write any state. While it holds, the callee's `blr` is a direct branch
     * to a known address, so compilation simply continues at the return site
     * and the return costs no host instructions at all. Without it, inlining
     * a call removed only the cheaper half of the round trip: the return
     * stayed an indirect exit, which cannot be block-linked and therefore
     * paid a dispatcher hash lookup on every call. */
    u8          lr_known;
    u32         lr_const;

    /* Trace formation across regions. guest_insts is per-region (it is what
     * the downcount charges are measured in, and a region starts its own
     * charge); trace_insts is the whole unit, for the budget and the stats. */
    u32         trace_insts;
    u8          region;         /* 0 = the unit's entry region */
    u8          region_count;   /* regions opened past the entry one */

#ifdef JIT_WORDPROF
    /* Exact executed-word accounting (measurement builds only). wp_mark is
     * the emit offset the current straight-line run started accounting from;
     * wp_carry is the word count already established for that run before
     * wp_mark (a cold tail inherits the hot prefix that reached it, and a
     * checkpoint carries a negative correction for words it counted ahead of
     * itself). */
    u32         wp_mark;
    s32         wp_carry;
#endif

    /* Loop register retention (bdnz counted loops). When a block is a self-loop
     * whose back-edge is a bdnz, its guest registers and CTR are loaded once at
     * a cold entry and kept live across the back-edge -- no per-iteration spill
     * and reload, which on the PPE is a load-hit-store stall storm. See the
     * "Loop retention" section in jit_compile.c. */
    u8          want_retain;    /* caller asked to try retention */
    u8          retaining;      /* retention is active for this compile */
    u8          retain_aborted; /* a disqualifying event -> recompile plain */
    u32         retain_pin;     /* host GPRs pinned for the whole block */
    u32         retain_warm_off;/* word offset of the warm loop entry */
    s8          retain_ctr_host;/* host reg holding CTR, or -1 */
    u8          retain_kind;    /* 0 = bdnz loop, 1 = cmp/bc loop */
    u8          retain_idle;    /* body is pure load+compare: skippable */

    /* Warm self-loop continuity — the general-body successor to retention.
     * A block whose terminator is a conditional branch back to its own start
     * is given a second entry point past a preload prologue, and the back
     * edge branches there with the guest registers still live: no
     * per-iteration reload or writeback for the preloaded set. Discovered by
     * a probe compile (pass 1 records the cache mapping at the back edge),
     * then recompiled with `want_warm` and the recorded set. See the
     * "Warm self-loop continuity" section in jit_compile.c. */
    u8          want_warm;      /* caller asks for a warm-prologue compile */
    u8          warm_active;    /* prologue emitted; back edge may go warm */
    u8          warm_candidate; /* probe pass: warm-eligible self-loop seen */
    u8          warm_count;     /* preloaded (guest, host) pairs */
    u8          warm_guest[H_GPRCACHE_COUNT];  /* guest GPR number        */
    u8          warm_hostr[H_GPRCACHE_COUNT];  /* host GPR holding it     */
    u8          warm_dirtyf[H_GPRCACHE_COUNT]; /* dirty-from-entry flag   */
    u8          had_fallback;   /* an interpreter fallback wiped the cache */
    u32         warm_off;       /* word offset of the warm entry           */
    u32         warm_words;     /* fast-path words, warm entry..back edge  */

    /* Where each promoted REGION landed, in words from the unit's base.
     * Regions are emitted by the cold-tail sweep but are ordinary hot code:
     * a taken conditional branch enters one and never returns to the entry
     * region. The PPE scheduler needs their extents because the words
     * between them -- escape and exit tails -- must not be scheduled (the
     * shared escape tail is followed by an inline guest-pc DATA word). One
     * slot per region the unit may open. */
    u8          sched_span_count;
    struct { u32 from, to; } sched_span[JIT_TRACE_MAX_REGIONS];
} JitContext;

/* Host code address -> emitting guest block. For the rescue thread's wedged-
 * thread report; linear walk, not for use on any hot path. */
int jit_block_at_host(void *host_pc, u32 *guest_pc, u32 *guest_end, u32 *word);

#endif /* DOLPHIN_CORE_PPC_JIT_H */
