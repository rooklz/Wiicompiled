/* interp_system.c — SPRs, MSR, segment registers, exception return.
 *
 * Most of the register file is plumbing, but three items here are load-bearing
 * for real titles and are called out where they appear:
 *   GQR0-7   — quantization control; every psq_l/psq_st reads one.
 *   HID2     — gates paired singles and the locked cache.
 *   WPAR/HID2— the write-gather pipe, which is how the CPU feeds the GPU FIFO.
 */
#include <stdio.h>
#include <stdlib.h>
#include "interp.h"
#include "../../core_timing.h"
#include "../../mem/memmap.h"
#include "../../../common/log.h"

/* ------------------------------------------------------------------ */
/* Time base                                                            */
/*                                                                      */
/* Gekko's time base runs at the bus clock / 4. Titles use it for timing and    */
/* for seeding RNGs, so it must advance monotonically and at a plausible rate;  */
/* the scheduler updates s->tb from the emulated cycle count.                   */
/* ------------------------------------------------------------------ */

/* The time base is *derived*, not stored: reading it mid-slice must see time
 * moving (MKWii times single AI-counter ticks with back-to-back mftb pairs,
 * and with a slice-frozen clock both rates measured identically and its
 * sanity check span forever). Route reads through the scheduler's
 * extrapolating clock; tb_offset carries guest writes. */
DOL_INLINE u64 tb_now(const PPCState *s)
{ return timing_timebase() + s->tb_offset; }
DOL_INLINE u32 tb_lower(const PPCState *s) { return (u32)tb_now(s); }
DOL_INLINE u32 tb_upper(const PPCState *s) { return (u32)(tb_now(s) >> 32); }

/* ------------------------------------------------------------------ */
/* mfspr / mtspr                                                        */
/* ------------------------------------------------------------------ */

void ppc_mfspr(PPCState *s, u32 op)
{
    u32 n = SPRN(op);
    u32 v;

    switch (n) {
    case SPR_XER:   v = ppc_get_xer(s);          break;
    case SPR_LR:    v = s->lr;                   break;
    case SPR_CTR:   v = s->ctr;                  break;
    case SPR_DEC:   v = s->dec;                  break;
    case SPR_TBL_R: v = tb_lower(s);             break;
    case SPR_TBU_R: v = tb_upper(s);             break;
    case SPR_PVR:   v = GEKKO_PVR;               break;
    case SPR_HID2:  v = s->hid2;                 break;
    case SPR_GQR0: case SPR_GQR0+1: case SPR_GQR0+2: case SPR_GQR0+3:
    case SPR_GQR0+4: case SPR_GQR0+5: case SPR_GQR0+6: case SPR_GQR0+7:
        v = s->gqr[n - SPR_GQR0];
        break;
    default:
        if (n < SPR_COUNT) {
            v = s->spr[n];
        } else {
            LOG_WARN_ONCE(LOG_INTERP, "mfspr from out-of-range SPR %u", n);
            v = 0;
        }
        break;
    }
    s->gpr[RT(op)] = v;
}

u64 g_lcdma_loads, g_lcdma_stores, g_lcdma_last_mem;

void ppc_mtspr(PPCState *s, u32 op)
{
    u32 n = SPRN(op);
    u32 v = s->gpr[RS(op)];

    switch (n) {
    case SPR_XER:   ppc_set_xer(s, v);   return;
    case SPR_LR:    s->lr  = v;          return;
    case SPR_CTR:   s->ctr = v;          return;

    case SPR_DEC:
        /* Writing DEC restarts the countdown; the scheduler samples it against
         * the time base rather than ticking it, so record when it was set. */
        s->dec = v;
        s->dec_write_tb = s->tb;
        return;

    case SPR_TBL_W: {
        /* A write sets the *offset* between the guest's TB and the derived
         * clock, so subsequent reads keep advancing from the written value. */
        u64 cur = tb_now(s);
        s->tb_offset += ((cur & 0xFFFFFFFF00000000ull) | v) - cur;
        return;
    }
    case SPR_TBU_W: {
        u64 cur = tb_now(s);
        s->tb_offset += ((cur & 0x00000000FFFFFFFFull) | ((u64)v << 32)) - cur;
        return;
    }

    case SPR_HID2:
        s->hid2 = v;
        LOG_DEBUG(LOG_INTERP, "HID2 = %08x (PSE=%d LSQE=%d LCE=%d)", v,
                  !!(v & HID2_PSE), !!(v & HID2_LSQE), !!(v & HID2_LCE));
        return;

    case SPR_GQR0: case SPR_GQR0+1: case SPR_GQR0+2: case SPR_GQR0+3:
    case SPR_GQR0+4: case SPR_GQR0+5: case SPR_GQR0+6: case SPR_GQR0+7:
        /* A GQR write changes the meaning of every psq_l/psq_st compiled
         * against it, so the JIT's specialized quantized paths must be
         * re-validated. The guard those blocks carry handles it; this is only
         * a note for anyone reading the JIT's deopt path. */
        s->gqr[n - SPR_GQR0] = v;
        return;

    case SPR_PVR:
        return;                                  /* read-only */

    case SPR_DMAL: {
        /* Gekko/Broadway locked-cache DMA. Undocumented in this codebase
         * until now and load-bearing for real titles: Mario Kart Wii's THP
         * video decoder writes its IDCT output through LC DMA, so with the
         * trigger ignored every decoded frame vanished -- the attract movie
         * rendered as a flat colour over zeroed YUV planes while the DVD
         * streamed the file at full rate.
         *
         * Layout (LSB): DMAL = F | T<<1 | LEN_LO(2)<<2 | LD<<4 | LC_ADDR&~1F.
         *               DMAU = LEN_HI(5) | MEM_ADDR&~1F.
         * Length is in 32-byte lines, 0 meaning 128. LD=1 loads main->LC,
         * LD=0 stores LC->main. The transfer is modelled as instantaneous. */
        s->spr[SPR_DMAL] = v;
        if (v & 2u) {
            extern u64 g_lcdma_loads, g_lcdma_stores, g_lcdma_last_mem;
            if (v & 0x10u) g_lcdma_loads++; else { g_lcdma_stores++;
                g_lcdma_last_mem = s->spr[SPR_DMAU] & ~0x1Fu; }                            /* DMA_T: trigger */
            u32 dmau  = s->spr[SPR_DMAU];
            u32 mem_a = dmau & ~0x1Fu;
            u32 lc_a  = v & ~0x1Fu;
            u32 lines = ((dmau & 0x1Fu) << 2) | ((v >> 2) & 3u);
            u32 bytes = (lines ? lines : 128u) * 32u;
            u32 i2;
            /* Polarity is NOT a guess -- Broadway User Manual, Table 2-18
             * (DMAL Bit Settings), bit 27 DMA_LD:
             *     0 = Store: transfer from locked cache to external memory
             *     1 = Load:  transfer from external memory to locked cache
             * Bit 27 in the manual's MSB numbering is mask 0x10 here. An
             * earlier empirical "fix" inverted this from a misread of the
             * load/store counters and broke both directions. */
            if (v & 0x10u) {                     /* load: main -> LC */
                for (i2 = 0; i2 < bytes; i2 += 4)
                    mem_write32(lc_a + i2, mem_read32(mem_a + i2));
            } else {                             /* store: LC -> main */
                for (i2 = 0; i2 < bytes; i2 += 4)
                    mem_write32(mem_a + i2, mem_read32(lc_a + i2));
            }
            s->spr[SPR_DMAL] = v & ~2u;          /* transfer complete */
        }
        return;
    }

    default:
        if (n < SPR_COUNT)
            s->spr[n] = v;
        else
            LOG_WARN_ONCE(LOG_INTERP, "mtspr to out-of-range SPR %u", n);
        return;
    }
}

/* mftb is architecturally distinct from mfspr but reads the same counter. */
void ppc_mftb(PPCState *s, u32 op)
{
    u32 n = SPRN(op);
    s->gpr[RT(op)] = (n == SPR_TBU_R) ? tb_upper(s) : tb_lower(s);
}

/* ------------------------------------------------------------------ */
/* Condition register / XER movement                                    */
/* ------------------------------------------------------------------ */

void ppc_mfcr(PPCState *s, u32 op)
{
    s->gpr[RT(op)] = s->cr;
}

void ppc_mtcrf(PPCState *s, u32 op)
{
    u32 crm = CRM(op), v = s->gpr[RS(op)], mask = 0, i;
    for (i = 0; i < 8; i++)
        if (crm & (0x80u >> i))
            mask |= 0xFu << CR_FIELD_SHIFT(i);
    s->cr = (s->cr & ~mask) | (v & mask);
}

/* mcrxr moves XER's summary bits into a CR field and *clears* them. The clear
 * is easy to miss and titles do depend on it. */
void ppc_mcrxr(PPCState *s, u32 op)
{
    u32 f = (s->xer_so ? CR_LT : 0u) | (s->xer_ov ? CR_GT : 0u) |
            (s->xer_ca ? CR_EQ : 0u);
    s->cr = cr_set_field(s->cr, CRFD(op), f);
    s->xer_so = s->xer_ov = s->xer_ca = 0;
}

/* ------------------------------------------------------------------ */
/* MSR and exception return                                             */
/* ------------------------------------------------------------------ */

void ppc_mfmsr(PPCState *s, u32 op)
{
    s->gpr[RT(op)] = s->msr;
}

void ppc_mtmsr(PPCState *s, u32 op)
{
    s->msr = s->gpr[RS(op)];
    /* Enabling external interrupts can expose an already-pending one. */
    if ((s->msr & MSR_EE) && s->exceptions)
        ppc_request_exit(s);
}

void ppc_rfi(PPCState *s, u32 op)
{
    (void)op;
    /* SRR1 supplies the restored MSR bits; the mask is the set the ISA says
     * transfers. */
    const u32 mask = 0x87C0FF73u;
    s->msr = (s->msr & ~mask) | (s->spr[SPR_SRR1] & mask);
    s->msr &= ~0x00040000u;                     /* POW is always cleared */
    if (UNLIKELY((s->spr[SPR_SRR0] & ~3u) == 0x80500000u)) {
        static unsigned n_s5;
        if (n_s5 < 6)
            LOG_WARN(LOG_INTERP, "rfi to 80500000! ctx=%08x from pc=%08x lr=%08x",
                     mem_read32(0x800000D4u), s->pc, s->lr);
        n_s5++;
    }
    if (UNLIKELY((s->spr[SPR_SRR0] & ~3u) == 0)) {
        static unsigned n_z;
        if (n_z < 8)
            LOG_WARN(LOG_INTERP, "rfi to 0! ctx=%08x lr=%08x sp=%08x from pc=%08x",
                     mem_read32(0x800000D4u), s->lr, s->gpr[1], s->pc);
        n_z++;
    }
    s->npc = s->spr[SPR_SRR0] & ~3u;
    ppc_request_exit(s);                        /* leave the block; MSR changed */
}

/* ------------------------------------------------------------------ */
/* Exception delivery                                                   */
/*                                                                      */
/* ppc_raise only records that an exception is pending. This is where one is    */
/* actually taken: state saved into SRR0/SRR1, MSR reduced to its exception     */
/* form, and execution vectored. Splitting the two is what lets recompiled      */
/* code raise an exception from the middle of a block and have it delivered at  */
/* a boundary where all guest state is coherent.                                */
/*                                                                              */
/* The distinction that matters most here is synchronous versus asynchronous.   */
/* A synchronous exception belongs to the instruction that caused it, so SRR0   */
/* must name *that* instruction and it must be taken before the next one runs.  */
/* An asynchronous one merely interrupts the flow, so SRR0 names whatever comes */
/* next. Getting this backwards produces a title that runs for minutes and then */
/* returns from a handler into the wrong instruction, which is close to         */
/* undiagnosable after the fact.                                                */
/* ------------------------------------------------------------------ */

/* Of those, the two whose SRR0 is the *following* instruction: the ISA defines
 * sc and the trace exception as completing before the exception is taken. */
#define EXC_SYNC_AFTER  (EXC_SYSCALL | EXC_TRACE)

/* Post-mortem record of the last synchronous fault. A game's own exception
 * handler runs and obscures the original cause; this captures it at the source
 * so a diagnostic tool can report what actually faulted. */
u32 g_fault_vector, g_fault_srr0, g_fault_srr1, g_fault_dar, g_fault_dsisr;
unsigned g_fault_count;

static void deliver(PPCState *s, u32 which, u32 vector, u32 srr0, u32 srr1_extra)
{
    /* Post-mortem visibility: while the diagnostic breakpoints are armed,
     * print every exception taken -- which, from where, under what MSR. */
    if (g_bp_pc[0]) {
        static int t = -1;
        if (t < 0) t = getenv("DSP_TRACE") != NULL;
        if (t)
            fprintf(stderr, "[exc] vec=%03x srr0=%08x msr=%08x "
                    "ctxD4=%08x ctxC0=%08x\n",
                    (unsigned)vector, (unsigned)srr0, (unsigned)s->msr,
                    (unsigned)mem_read32(0x800000D4u),
                    (unsigned)mem_read32(0x800000C0u));
    }

    /* SRR1 keeps MSR's low half plus the exception-specific bits; the mask is
     * the one the 750 documents and the one Dolphin uses, so a title that
     * inspects SRR1 sees the same value on both. */
    s->spr[SPR_SRR0] = srr0;
    s->spr[SPR_SRR1] = (s->msr & 0x87C0FFFFu) | srr1_extra;

    if (vector == VEC_DSI || vector == VEC_ISI || vector == VEC_PROGRAM ||
        vector == VEC_ALIGNMENT) {
        g_fault_vector = vector;
        g_fault_srr0   = srr0;
        g_fault_srr1   = s->spr[SPR_SRR1];
        g_fault_dar    = s->spr[SPR_DAR];
        g_fault_dsisr  = s->spr[SPR_DSISR];
        g_fault_count++;
    }

    /* Everything the hardware clears on entry: translation off, interrupts off,
     * supervisor mode, FP off. A handler runs on physical addresses. */
    s->msr &= ~0x04EF36u;

    /* MSR[IP] chooses between the low and high vector pages. The Wii runs with
     * it clear, but honouring it costs one test and makes early IPL code --
     * which does run with it set -- behave. */
    s->pc = s->npc = ((s->msr & MSR_IP) ? 0xFFF00000u : 0x00000000u) + vector;

    s->exceptions &= ~which;
    ppc_request_exit(s);
}

/* s->pc carries the right address for both kinds, because the two are taken at
 * different moments. A synchronous exception is delivered from ppc_raise, in
 * the middle of the instruction that caused it -- where s->pc is still that
 * instruction, in the interpreter and equally in the JIT's fallback path, which
 * stores the fallback instruction's address before calling the handler. An
 * asynchronous one is delivered at a boundary, where s->pc has already advanced
 * to whatever would have run next. Both want s->pc; neither wants npc. */
int ppc_deliver_exception(PPCState *s)
{
    u32 p = s->exceptions;

    if (!p)
        return 0;

    /* Priority order is the 750's. Synchronous first: they are already the
     * consequence of the instruction just executed, so nothing else can be
     * more urgent. */
    if (p & EXC_ISI) {
        deliver(s, EXC_ISI, VEC_ISI, s->pc, 0);
        return 1;
    }
    if (p & EXC_PROGRAM) {
        /* SRR1[11] distinguishes an illegal instruction from a trap or a
         * privilege violation. We only raise it for illegal opcodes and for
         * paired-single use with HID2[PSE] clear, both of which are "illegal". */
        {   /* Rate-limited: name the faulting pc/op -- the race-load wedge is
             * an exception storm at 0x700 and this is the line that says from
             * WHERE. */
            static unsigned n_pe;
            if (n_pe < 16 || (n_pe & 0x3FFu) == 0) {
                u32 sp2 = s->gpr[1]; unsigned d2;
                LOG_WARN(LOG_INTERP, "PROGRAM exc #%u at pc=%08x op=%08x lr=%08x sp=%08x",
                         n_pe, s->pc, mem_read32(s->pc), s->lr, sp2);
                /* Return-address chain: names the caller that jumped into
                 * zeroed memory, which the wedge's lr (== pc) cannot. */
                for (d2 = 0; d2 < 8 && sp2 >= 0x80000000u && sp2 < 0x81800000u;
                     d2++) {
                    u32 up = mem_read32(sp2), ra = mem_read32(sp2 + 4);
                    LOG_WARN(LOG_INTERP, "  bt[%u] sp=%08x ra=%08x", d2, sp2, ra);
                    if (up <= sp2) break;
                    sp2 = up;
                }
            }
            n_pe++;
        }
        deliver(s, EXC_PROGRAM, VEC_PROGRAM, s->pc, 0x00080000u);
        return 1;
    }
    if (p & EXC_FPU_UNAVAILABLE) {
        deliver(s, EXC_FPU_UNAVAILABLE, VEC_FPU_UNAVAILABLE, s->pc, 0);
        return 1;
    }
    if (p & EXC_DSI) {
        deliver(s, EXC_DSI, VEC_DSI, s->pc, 0);
        return 1;
    }
    if (p & EXC_ALIGNMENT) {
        deliver(s, EXC_ALIGNMENT, VEC_ALIGNMENT, s->pc, 0);
        return 1;
    }
    if (p & EXC_SYSCALL) {
        deliver(s, EXC_SYSCALL, VEC_SYSCALL, s->pc + 4, 0);
        return 1;
    }
    if (p & EXC_TRACE) {
        deliver(s, EXC_TRACE, VEC_TRACE, s->pc + 4, 0);
        return 1;
    }

    /* Asynchronous exceptions record s->pc, not s->npc: at every point this is
     * called -- after an interpreter step, or at a compiled block's exit --
     * s->pc already names the instruction that would have run next, and s->npc
     * is only meaningful mid-instruction.
     *
     * Only with external interrupts enabled. A device that
     * asserts while MSR[EE] is clear stays asserted -- pi.c models the line
     * level rather than an edge -- so nothing is lost by declining here. */
    if (!(s->msr & MSR_EE))
        return 0;

    /* External before decrementer: a device waiting on a frame deadline is more
     * time-critical than the scheduler tick, and this is the order hardware
     * uses. */
    if (p & EXC_EXTERNAL_INT) {
        deliver(s, EXC_EXTERNAL_INT, VEC_EXTERNAL_INT, s->pc, 0);
        return 1;
    }
    if (p & EXC_DECREMENTER) {
        deliver(s, EXC_DECREMENTER, VEC_DECREMENTER, s->pc, 0);
        return 1;
    }
    if (p & EXC_PERF_MONITOR) {
        deliver(s, EXC_PERF_MONITOR, VEC_PERF_MONITOR, s->pc, 0);
        return 1;
    }
    if (p & EXC_THERMAL) {
        deliver(s, EXC_THERMAL, VEC_THERMAL, s->pc, 0);
        return 1;
    }

    return 0;
}

int ppc_exception_pending(const PPCState *s)
{
    if (s->exceptions & EXC_SYNCHRONOUS)
        return 1;
    return (s->msr & MSR_EE) && s->exceptions != 0;
}

/* ------------------------------------------------------------------ */
/* Segment registers and TLB                                            */
/*                                                                      */
/* Present for completeness. Retail titles run with BAT-mapped memory and       */
/* almost never touch these, but the IPL does during early boot.                */
/* ------------------------------------------------------------------ */

void ppc_mtsr(PPCState *s, u32 op)   { s->sr[(op >> 16) & 0xF] = s->gpr[RS(op)]; }
void ppc_mfsr(PPCState *s, u32 op)   { s->gpr[RT(op)] = s->sr[(op >> 16) & 0xF]; }
void ppc_mtsrin(PPCState *s, u32 op) { s->sr[(s->gpr[RB(op)] >> 28) & 0xF] = s->gpr[RS(op)]; }
void ppc_mfsrin(PPCState *s, u32 op) { s->gpr[RT(op)] = s->sr[(s->gpr[RB(op)] >> 28) & 0xF]; }

void ppc_tlbie(PPCState *s, u32 op)   { (void)s; (void)op; }
void ppc_tlbsync(PPCState *s, u32 op) { (void)s; (void)op; }

/* eieio / sync order guest memory accesses. We execute guest memory operations
 * in program order already, so there is nothing to enforce. */
void ppc_sync_nop(PPCState *s, u32 op) { (void)s; (void)op; }
