/* gekko.h — architectural state of the Gekko/Broadway CPU.
 *
 * PPCState is the single most performance-sensitive structure in the emulator.
 * Its field order is deliberate, not incidental:
 *
 *   - The Cell PPE has 128-byte cache lines and *no out-of-order engine*. A
 *     miss on this structure cannot be hidden by speculation, so the fields the
 *     JIT touches every block are packed into the first two lines.
 *   - Every field must sit within a signed 16-bit displacement of the pinned
 *     state pointer (r15) so the JIT can address it with one d-form
 *     instruction: `lwz rD, offsetof(field)(r15)`. The struct is far smaller
 *     than 32 KiB, so this holds by construction (asserted below).
 *   - `ps[]` is 16-byte aligned so VMX `lvx`/`stvx` can address it directly for
 *     the quantized load/store paths.
 *
 * Representation decisions worth stating explicitly, because they differ from
 * every x86 Dolphin backend:
 *
 * CR is stored *packed*, exactly as hardware lays it out. On x86, PowerPC's
 * 8-field condition register has no analogue, so Dolphin keeps eight separate
 * lazily-evaluated values. Here the host CR *is* the guest CR: `cmpw` writes it
 * and `bc` reads it, 1:1. Within a compiled block the guest CR lives in the
 * host CR and is only spilled to this field at block boundaries and helper
 * calls.
 *
 * Paired singles are stored as two IEEE doubles (ps0, ps1). This mirrors the
 * hardware, where an FPR holds a double and the paired-single ops round each
 * lane to single precision. PowerPC's `fadds`/`fmuls`/`fmadds` have *precisely*
 * that semantic, so paired-single arithmetic is two native, bit-exact scalar
 * instructions with no format conversion anywhere. See ARCHITECTURE.md §2.4.
 */
#ifndef DOLPHIN_CORE_PPC_GEKKO_H
#define DOLPHIN_CORE_PPC_GEKKO_H

#include "../../common/types.h"

/* ------------------------------------------------------------------ */
/* Floating-point register representation                               */
/* ------------------------------------------------------------------ */

typedef union {
    u64 u;
    f64 f;
} FPReg;

/* One Gekko FPR: the scalar double lives in ps0; paired-single ops use both.
 * 16 bytes exactly, so ps[i] is at byte offset i*16 and ps1 at i*16+8 — both
 * reachable by a single d-form FP load. */
typedef struct {
    FPReg ps0;
    FPReg ps1;
} PairedSingle;

DOL_STATIC_ASSERT(sizeof(PairedSingle) == 16, paired_single_size);

/* ------------------------------------------------------------------ */
/* Condition register                                                   */
/* ------------------------------------------------------------------ */

/* CR field n occupies bits [4n .. 4n+3] counting MSB-first, i.e. CR0 is the
 * *high* nibble. Shift for field n is therefore 28 - 4n. */
#define CR_FIELD_SHIFT(n)   (28 - 4 * (n))
#define CR_FIELD_MASK(n)    (0xFu << CR_FIELD_SHIFT(n))

#define CR_LT   0x8u    /* negative / less than    */
#define CR_GT   0x4u    /* positive / greater than */
#define CR_EQ   0x2u    /* zero / equal            */
#define CR_SO   0x1u    /* summary overflow copy   */

DOL_INLINE u32 cr_get_field(u32 cr, unsigned n)
{
    return (cr >> CR_FIELD_SHIFT(n)) & 0xFu;
}

DOL_INLINE u32 cr_set_field(u32 cr, unsigned n, u32 v)
{
    return (cr & ~CR_FIELD_MASK(n)) | ((v & 0xFu) << CR_FIELD_SHIFT(n));
}

/* ------------------------------------------------------------------ */
/* FPSCR                                                                */
/* ------------------------------------------------------------------ */

#define FPSCR_FX        0x80000000u  /* exception summary            */
#define FPSCR_FEX       0x40000000u  /* enabled exception summary    */
#define FPSCR_VX        0x20000000u  /* invalid operation summary    */
#define FPSCR_OX        0x10000000u  /* overflow                     */
#define FPSCR_UX        0x08000000u  /* underflow                    */
#define FPSCR_ZX        0x04000000u  /* zero divide                  */
#define FPSCR_XX        0x02000000u  /* inexact                      */
#define FPSCR_VXSNAN    0x01000000u
#define FPSCR_VXISI     0x00800000u  /* inf - inf                    */
#define FPSCR_VXIDI     0x00400000u  /* inf / inf                    */
#define FPSCR_VXZDZ     0x00200000u  /* 0 / 0                        */
#define FPSCR_VXIMZ     0x00100000u  /* inf * 0                      */
#define FPSCR_VXVC      0x00080000u  /* invalid compare              */
#define FPSCR_FR        0x00040000u  /* fraction rounded             */
#define FPSCR_FI        0x00020000u  /* fraction inexact             */
#define FPSCR_FPRF_MASK 0x0001F000u  /* result class + <>=?          */
#define FPSCR_VXSOFT    0x00000400u
#define FPSCR_VXSQRT    0x00000200u
#define FPSCR_VXCVI     0x00000100u
#define FPSCR_VE        0x00000080u  /* invalid-op exception enable  */
#define FPSCR_OE        0x00000040u
#define FPSCR_UE        0x00000020u
#define FPSCR_ZE        0x00000010u
#define FPSCR_XE        0x00000008u
#define FPSCR_NI        0x00000004u  /* non-IEEE mode                */
#define FPSCR_RN_MASK   0x00000003u  /* rounding mode                */

#define FPSCR_VX_ANY   (FPSCR_VXSNAN | FPSCR_VXISI | FPSCR_VXIDI | FPSCR_VXZDZ | \
                        FPSCR_VXIMZ  | FPSCR_VXVC  | FPSCR_VXSOFT | FPSCR_VXSQRT | \
                        FPSCR_VXCVI)

/* "No deferred FPRF pending" marker for PPCState.fprf_src / fprf_ack (see the
 * struct). A signalling NaN: IEEE requires every arithmetic operation to quiet
 * a signalling NaN operand and none can create one, so no value the JIT ever
 * hands to `stfd fprf_src` can collide with it. */
#define FPRF_SRC_NONE   0x7FF0000100000000ull

/* ------------------------------------------------------------------ */
/* XER                                                                  */
/* ------------------------------------------------------------------ */

#define XER_SO_BIT      0x80000000u
#define XER_OV_BIT      0x40000000u
#define XER_CA_BIT      0x20000000u
#define XER_COUNT_MASK  0x0000007Fu

/* ------------------------------------------------------------------ */
/* MSR                                                                  */
/* ------------------------------------------------------------------ */

#define MSR_POW     0x00040000u  /* power management enable   */
#define MSR_ILE     0x00010000u  /* interrupt little-endian   */
#define MSR_EE      0x00008000u  /* external interrupt enable */
#define MSR_PR      0x00004000u  /* problem (user) state      */
#define MSR_FP      0x00002000u  /* FP available              */
#define MSR_ME      0x00001000u  /* machine check enable      */
#define MSR_FE0     0x00000800u
#define MSR_SE      0x00000400u  /* single step               */
#define MSR_BE      0x00000200u  /* branch trace              */
#define MSR_FE1     0x00000100u
#define MSR_IP      0x00000040u  /* exception prefix          */
#define MSR_IR      0x00000020u  /* instruction relocate (MMU)*/
#define MSR_DR      0x00000010u  /* data relocate (MMU)       */
#define MSR_PM      0x00000004u  /* performance monitor mark  */
#define MSR_RI      0x00000002u  /* recoverable interrupt     */
#define MSR_LE      0x00000001u  /* little-endian             */

/* Bits that participate in JIT block identity: code compiled with the MMU on
 * is not valid with it off, and user/supervisor changes fault differently. */
#define MSR_JIT_KEY_MASK  (MSR_IR | MSR_DR | MSR_PR | MSR_FP)

/* ------------------------------------------------------------------ */
/* HID2 — Gekko-specific, gates paired singles and the locked cache      */
/* ------------------------------------------------------------------ */

#define HID2_LSQE   0x80000000u  /* load/store quantized enable */
#define HID2_WPE    0x40000000u  /* write pipe enable           */
#define HID2_PSE    0x20000000u  /* paired single enable        */
#define HID2_LCE    0x10000000u  /* locked cache enable         */

/* ------------------------------------------------------------------ */
/* Exceptions — held as a bitmask so the JIT can test them with one andi. */
/* ------------------------------------------------------------------ */

#define EXC_DECREMENTER     0x00000001u
#define EXC_EXTERNAL_INT    0x00000002u
#define EXC_PERF_MONITOR    0x00000004u
#define EXC_DSI             0x00000008u
#define EXC_ISI             0x00000010u
#define EXC_ALIGNMENT       0x00000020u
#define EXC_PROGRAM         0x00000040u
#define EXC_FPU_UNAVAILABLE 0x00000080u
#define EXC_SYSCALL         0x00000100u
#define EXC_FLOATING_POINT  0x00000200u
#define EXC_TRACE           0x00000400u
#define EXC_THERMAL         0x00000800u

/* Exceptions caused by the instruction executing them, as opposed to ones that
 * merely interrupt the flow. The split decides both *when* an exception may be
 * taken and *what* goes in SRR0, so it belongs beside the bits themselves
 * rather than in whichever file happened to need it first. */
#define EXC_SYNCHRONOUS (EXC_DSI | EXC_ISI | EXC_ALIGNMENT | EXC_PROGRAM | \
                         EXC_FPU_UNAVAILABLE | EXC_SYSCALL | EXC_TRACE)

/* Vector offsets (physical, relative to the 0x00000000 / 0xFFF00000 prefix). */
#define VEC_SYSTEM_RESET    0x00100
#define VEC_MACHINE_CHECK   0x00200
#define VEC_DSI             0x00300
#define VEC_ISI             0x00400
#define VEC_EXTERNAL_INT    0x00500
#define VEC_ALIGNMENT       0x00600
#define VEC_PROGRAM         0x00700
#define VEC_FPU_UNAVAILABLE 0x00800
#define VEC_DECREMENTER     0x00900
#define VEC_SYSCALL         0x00C00
#define VEC_TRACE           0x00D00
#define VEC_PERF_MONITOR    0x00F00
#define VEC_THERMAL         0x01700

/* ------------------------------------------------------------------ */
/* SPR numbers                                                          */
/* ------------------------------------------------------------------ */

#define SPR_XER     1
#define SPR_LR      8
#define SPR_CTR     9
#define SPR_DSISR   18
#define SPR_DAR     19
#define SPR_DEC     22
#define SPR_SDR1    25
#define SPR_SRR0    26
#define SPR_SRR1    27
#define SPR_SPRG0   272
#define SPR_SPRG1   273
#define SPR_SPRG2   274
#define SPR_SPRG3   275
#define SPR_EAR     282
#define SPR_TBL_R   268   /* time base, read  */
#define SPR_TBU_R   269
#define SPR_TBL_W   284   /* time base, write */
#define SPR_TBU_W   285
#define SPR_PVR     287
#define SPR_IBAT0U  528
#define SPR_IBAT0L  529
#define SPR_IBAT3L  535
#define SPR_DBAT0U  536
#define SPR_DBAT0L  537
#define SPR_DBAT3L  543
/* Gekko adds a second set of four BATs (IBAT4-7 / DBAT4-7). */
#define SPR_IBAT4U  560
#define SPR_IBAT7L  567
#define SPR_DBAT4U  568
#define SPR_DBAT7L  575
#define SPR_GQR0    912   /* .. 919 */
#define SPR_HID2    920
#define SPR_WPAR    921   /* write gather pipe address */
#define SPR_DMAU    922   /* locked-cache DMA upper    */
#define SPR_DMAL    923   /* locked-cache DMA lower    */
#define SPR_MMCR0   952
#define SPR_PMC1    953
#define SPR_PMC2    954
#define SPR_SIA     955
#define SPR_MMCR1   956
#define SPR_PMC3    957
#define SPR_PMC4    958
#define SPR_HID0    1008
#define SPR_HID1    1009
#define SPR_IABR    1010
#define SPR_HID4    1011
#define SPR_DABR    1013
#define SPR_L2CR    1017
#define SPR_ICTC    1019
#define SPR_THRM1   1020
#define SPR_THRM3   1022

#define SPR_COUNT   1024

/* Gekko's PVR. Broadway reports the same major revision; titles that branch on
 * it are checking for "Gekko-class", not for a specific console. */
#define GEKKO_PVR   0x00083214u

/* ------------------------------------------------------------------ */
/* Quantized load/store (psq_l / psq_st)                                */
/* ------------------------------------------------------------------ */

typedef enum {
    QUANT_F32 = 0,
    QUANT_RSVD1 = 1,
    QUANT_RSVD2 = 2,
    QUANT_RSVD3 = 3,
    QUANT_U8  = 4,
    QUANT_U16 = 5,
    QUANT_S8  = 6,
    QUANT_S16 = 7
} QuantType;

/* GQR layout. Scale fields are 6-bit *signed* exponents: a load computes
 * f = (f32)i * 2^-scale and a store computes i = (int)(f * 2^scale). */
DOL_INLINE u32 gqr_st_type(u32 gqr)  { return  gqr        & 0x7u;  }
DOL_INLINE u32 gqr_st_scale(u32 gqr) { return (gqr >> 8)  & 0x3Fu; }
DOL_INLINE u32 gqr_ld_type(u32 gqr)  { return (gqr >> 16) & 0x7u;  }
DOL_INLINE u32 gqr_ld_scale(u32 gqr) { return (gqr >> 24) & 0x3Fu; }

/* Sign-extend a 6-bit GQR scale to a usable exponent. */
DOL_INLINE s32 gqr_scale_exp(u32 scale6) { return dol_sext32(scale6, 6); }

/* ------------------------------------------------------------------ */
/* BAT entry (decoded)                                                  */
/* ------------------------------------------------------------------ */

typedef struct {
    u32 bepi;       /* block effective page index      */
    u32 bl;         /* block length mask               */
    u32 brpn;       /* block real page number          */
    u8  vs, vp;     /* valid in supervisor / problem   */
    u8  wimg;       /* cache control bits              */
    u8  pp;         /* protection                      */
} BATEntry;

/* ------------------------------------------------------------------ */
/* The architectural state                                              */
/* ------------------------------------------------------------------ */

typedef struct DOL_CACHE_ALIGN PPCState {
    /* --- cache line 0: the general purpose registers ------------------
     * Exactly 128 bytes: one full PPE cache line, never sharing with
     * anything else. */
    u32 gpr[32];                                        /* 0x000 */

    /* --- cache line 1: everything the dispatcher and every block touch --- */
    u32 pc;                                             /* 0x080 */
    u32 npc;                                            /* 0x084 */
    u32 cr;                                             /* 0x088 packed, host-CR mirrored */
    u32 msr;                                            /* 0x08C */
    u32 lr;                                             /* 0x090 */
    u32 ctr;                                            /* 0x094 */

    /* XER is split into its independently-updated pieces. Carry is by far the
     * hottest (every addc/adde/subfe chain) and lives in its own word so the
     * JIT can keep it in a pinned host register and never read-modify-write a
     * packed XER. */
    u32 xer_ca;                                         /* 0x098 0 or 1  */
    u32 xer_so;                                         /* 0x09C 0 or 1  */
    u32 xer_ov;                                         /* 0x0A0 0 or 1  */
    u32 xer_count;                                      /* 0x0A4 lswx/stswx */

    /* Cycles remaining before the scheduler must run. The JIT decrements this
     * in a pinned register and branches to the dispatcher when it goes
     * negative — the only per-block bookkeeping we cannot avoid. */
    s32 downcount;                                      /* 0x0A8 */
    u32 exceptions;                                     /* 0x0AC pending mask */

    u32 fpscr;                                          /* 0x0B0 */
    u32 hid2;                                           /* 0x0B4 PSE/LSQE gate paired singles */
    u32 gqr[8];                                         /* 0x0B8 .. 0x0D7 */

    /* Reservation for lwarx/stwcx. */
    u32 reserve_addr;                                   /* 0x0D8 */
    u32 reserve_valid;                                  /* 0x0DC */

    /* --- floating point: 16-byte aligned for VMX quantized paths ------- */
    PairedSingle ps[32] DOL_ALIGN(16);                  /* 0x0E0 .. 0x2DF */

    /* Deferred FPSCR[FPRF].
     *
     * FPRF is the 5-bit class of the last floating-point *result*, and the
     * interpreter maintains it on every arithmetic op (interp_float.c calls
     * ppc_set_fprf). Computing that class in compiled code would cost an FPR
     * round trip through memory plus a dozen integer instructions on every
     * `fadd` -- for a field that essentially nothing reads. So compiled code
     * instead records the *source value* here with one `stfd`, and the class
     * is computed only when something actually reads FPSCR (`mffs`, `mcrfs`,
     * or an FPSCR-modifying form), by ppc_fprf_sync.
     *
     * The pair is a pending-flag, not two values: FPSCR[FPRF] as stored in
     * `fpscr` is authoritative exactly when `fprf_src == fprf_ack`; otherwise
     * the true FPRF is ppc_compute_fprf(fprf_src). Everything that writes FPRF
     * authoritatively (the interpreter's own handlers, and compile_fcmp's
     * inline sequence) sets BOTH to FPRF_SRC_NONE, a signalling NaN -- a bit
     * pattern no PowerPC floating-point arithmetic can ever produce, so a
     * later `stfd` of a genuine result is always recognised as pending even
     * when it repeats a value seen before. A zeroed PPCState has both zero,
     * i.e. "authoritative", which is what a reset state means -- and every
     * constructor (ppc_init_constants, dol_setup_boot_state) additionally
     * seeds the pair with FPRF_SRC_NONE, because 0 is also the bit pattern of
     * +0.0 and a first-ever result of exactly +0.0 would otherwise compare
     * equal to the zeroed `fprf_ack` and not register. */
    u64 fprf_src;
    u64 fprf_ack;

    /* --- colder state: touched by mtspr/mtsr and the MMU, not per block -- */
    u32 spr[SPR_COUNT];
    u32 sr[16];                     /* segment registers */

    BATEntry ibat[8];               /* Gekko has 8 IBATs (4 base + 4 extended) */
    BATEntry dbat[8];

    /* Time base, maintained as a 64-bit counter and split on read. */
    u64 tb;
    u64 tb_offset;      /* guest TB writes, relative to the derived clock */

    /* Decrementer is sampled against the time base rather than ticked, so a
     * long-running block does not have to touch it. */
    u32 dec;
    u64 dec_write_tb;

    /* Write-gather pipe (0xCC008000): Gekko streams 32-byte bursts to the GX
     * FIFO through this. Modelled as a 32-byte staging buffer. */
    u8  gather_pipe[32] DOL_ALIGN(32);
    u32 gather_pipe_count;

    /* Set when the JIT must return to the dispatcher at the next safe point:
     * exception pending, block invalidated, or the frontend asked us to stop. */
    volatile u32 exit_requested;

    /* Cycle budget destroyed by a forced exit, so the scheduler can give it
     * back.
     *
     * ppc_request_exit drives downcount negative to make linked blocks bail --
     * that single test is what keeps a chain of linked blocks cheap. The cost is
     * that the remaining budget is gone, and the scheduler derives elapsed time
     * by subtracting downcount from the grant, so an early exit would charge the
     * *entire* slice. Since exits are not rare (every device access, every
     * exception), that is not a rounding error; it is emulated time running fast
     * by a large and load-dependent factor. Recording what was destroyed costs
     * one word and makes the clock exact. */
    s32 exit_slack;

    /* Set by compiled code when it detects an access it cannot perform itself
     * -- an MMIO address reached through a register it could not prove. The
     * dispatcher runs exactly one interpreter step, which handles the device
     * access correctly, and then resumes compiled execution.
     *
     * This exists because lv2 offers no way to emulate a faulting instruction
     * and step over it (docs/ARCHITECTURE.md §3.2.1), so MMIO must be detected
     * in the emitted code rather than trapped. */
    u32 force_interp;

#ifdef JIT_WORDPROF
    /* Exact executed-host-word counter (measurement builds only).
     *
     * Block-size distributions and the profiler's insts/dispatch-weighted
     * estimates both stop being comparable once a compilation unit contains
     * internal branches: `hot_words` then counts code some traversals skip.
     * So the emitted code counts itself. Every point where control leaves a
     * straight-line run -- each exit, each escape, each loop back edge --
     * adds the exact number of host words that run executed. Three words per
     * such point, present only when JIT_WORDPROF is defined, so the shipped
     * recompiler is byte-for-byte unaffected. */
    u64 jit_prof_words;
#endif

    /* Constant 1.0, so compiled code can materialize the ps1 lane of a
     * width-1 quantized load with a single lfd rather than a constant pool. */
    f64 const_one;

    /* Working area for the quantised load/store paths. Converting an integer
     * lane to a float needs it to pass through memory once (there is no
     * register-to-register integer-to-float move), and the scale is a constant
     * the compiled code loads rather than computes: dequant[i] is 2^-e and
     * quant[i] is 2^e for the 6-bit scale field i read as signed. Having both
     * tables here means a quantised access is a handful of instructions instead
     * of a call into the interpreter. */
    u64 quant_scratch;
    f64 dequant_scale[64];
    f64 quant_scale[64];
    /* Saturation bounds for the four quantised store formats, indexed by
     * `type - QUANT_U8` in HARDWARE order (4=U8, 5=U16, 6=S8, 7=S16), and a
     * scratch double for moving an fctiwz result into a GPR. Held in state so
     * the compiled store can reach them off the state pointer instead of
     * materialising constants. */
    f64 quant_lo[4];
    f64 quant_hi[4];
    f64 quant_tmp;

    /* Instructions the guest did NOT execute because an idle loop was skipped.
     *
     * The scheduler charges a skipped spin as the whole remaining slice --
     * that is the point of the optimisation -- but the instruction counter is
     * `grant - downcount`, so those uncounted spins land in it as though they
     * had run. MKWii idles heavily (the boot flag loop alone accounts for most
     * of its instruction total), so any "guest instructions per second" or
     * "cycles per instruction" figure derived from that counter is inflated by
     * an unknown amount, and flatters the recompiler.
     *
     * The idle-skip site adds the discarded downcount here before zeroing it,
     * which costs three instructions once per slice and makes the real figure
     * recoverable: real = credited - idle_skipped. Appended at the end of the
     * struct on purpose -- the offsets above are quoted in the JIT.
     *
     * The JIT only STORES here, one instruction, because an idle skip ends the
     * slice: at most one can happen before C regains control. C zeroes it
     * before each slice and accumulates it afterwards. */
    u32 idle_skipped_last;
    /* Executed interpreter fallbacks. Counted here, not in a global, because
     * the state pointer is already live in a register at the fallback site --
     * three instructions against the five a global address would cost. Appended
     * at the end so no offset the JIT quotes moves. */
    u32 fallback_hits;
    /* Executed fallbacks per primary opcode. The emitting side knows the
     * opcode as a constant, so it addresses one slot directly and the count
     * costs the same three instructions as a single total would. Static
     * decline counts rank what the recompiler REFUSES; this ranks what a
     * title actually RUNS, which is the only ordering worth acting on. */
    u32 fallback_by_op[64];
} PPCState;

/* The JIT addresses every one of these with a single d-form instruction off the
 * pinned state pointer, which requires each offset to fit a signed 16-bit
 * displacement. Assert the whole structure stays inside that window. */
DOL_STATIC_ASSERT(sizeof(PPCState) < 32768, ppcstate_fits_dform);

/* Hot-field offsets are contractual: the JIT emitter hardcodes them and the
 * assembly dispatcher references them. Pin them so a careless field insertion
 * fails the build instead of silently costing performance or correctness. */
DOL_STATIC_ASSERT(offsetof(PPCState, gpr) == 0x000, off_gpr);
DOL_STATIC_ASSERT(offsetof(PPCState, pc)  == 0x080, off_pc);
DOL_STATIC_ASSERT(offsetof(PPCState, cr)  == 0x088, off_cr);
DOL_STATIC_ASSERT(offsetof(PPCState, downcount) == 0x0A8, off_downcount);
DOL_STATIC_ASSERT(offsetof(PPCState, ps)  == 0x0E0, off_ps);
/* GPRs must occupy exactly one cache line, alone. */
DOL_STATIC_ASSERT(sizeof(((PPCState *)0)->gpr) == DOL_CACHELINE, gpr_one_line);
/* VMX lvx/stvx ignore the low 4 address bits; ps[] must be truly 16-aligned. */
DOL_STATIC_ASSERT((offsetof(PPCState, ps) & 15) == 0, ps_vmx_aligned);
/* compile_fcmp retires the deferred FPRF with `std`, whose DS-form
 * displacement has no room for the low two bits. */
DOL_STATIC_ASSERT((offsetof(PPCState, fprf_src) & 3) == 0, fprf_src_ds_form);
DOL_STATIC_ASSERT((offsetof(PPCState, fprf_ack) & 3) == 0, fprf_ack_ds_form);

/* ------------------------------------------------------------------ */
/* Accessors shared by interpreter, JIT helpers and the debugger        */
/* ------------------------------------------------------------------ */

DOL_INLINE u32 ppc_get_xer(const PPCState *s)
{
    return (s->xer_so ? XER_SO_BIT : 0u) |
           (s->xer_ov ? XER_OV_BIT : 0u) |
           (s->xer_ca ? XER_CA_BIT : 0u) |
           (s->xer_count & XER_COUNT_MASK);
}

DOL_INLINE void ppc_set_xer(PPCState *s, u32 v)
{
    s->xer_so    = (v & XER_SO_BIT) ? 1u : 0u;
    s->xer_ov    = (v & XER_OV_BIT) ? 1u : 0u;
    s->xer_ca    = (v & XER_CA_BIT) ? 1u : 0u;
    s->xer_count =  v & XER_COUNT_MASK;
}

/* Paired singles are only architecturally visible when HID2[PSE] is set;
 * executing a PS opcode with it clear raises a program exception. */
DOL_INLINE int ppc_paired_single_enabled(const PPCState *s)
{
    return (s->hid2 & HID2_PSE) != 0;
}

DOL_INLINE int ppc_quantized_ls_enabled(const PPCState *s)
{
    return (s->hid2 & HID2_LSQE) != 0;
}

/* Compare-and-set CR0 from a 32-bit signed result, as Rc=1 instructions do. */
DOL_INLINE void ppc_update_cr0(PPCState *s, u32 result)
{
    u32 f = ((s32)result < 0) ? CR_LT : ((result == 0) ? CR_EQ : CR_GT);
    if (s->xer_so)
        f |= CR_SO;
    s->cr = cr_set_field(s->cr, 0, f);
}

#endif /* DOLPHIN_CORE_PPC_GEKKO_H */
