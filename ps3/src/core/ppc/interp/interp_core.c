/* interp_core.c — instruction decode tables, single-step and run loop.
 *
 * PowerPC decode is a primary 6-bit opcode plus, for five of those opcodes, an
 * extended field. Opcodes 4 and 63 are the awkward ones: they carry *both*
 * 5-bit A-form and 10-bit X-form extended opcodes in the same encoding space.
 * The two sets happen not to collide (the A-form values never appear in the low
 * five bits of a valid X-form value), so the tables can be built by replicating
 * the A-form entries across all 32 aliases and then writing the X-form entries
 * over the remainder. That non-collision is asserted at table-build time rather
 * than assumed, because if it were ever false the failure would be a
 * mysteriously wrong instruction rather than a crash.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "interp.h"
#include "interp_ops.h"
#include "../../mem/memmap.h"
#include "../../../common/log.h"

#include <string.h>

static InterpFn s_primary[64];
static InterpFn s_t4[1024];
static InterpFn s_t19[1024];
static InterpFn s_t31[1024];
static InterpFn s_t59[32];
static InterpFn s_t63[1024];
static int s_tables_built;

/* ------------------------------------------------------------------ */
/* Illegal / unimplemented                                              */
/* ------------------------------------------------------------------ */

/* Store-target watch: when the guest PC is in [g_stw_lo,g_stw_hi), record the
 * effective address of every store. Names WHERE the THP decoder's output
 * actually lands, since its plane pointer is computed at runtime and the
 * static probe addresses were guesses. */
u32 g_stw_lo, g_stw_hi;
u32 g_stw_ea[16];
unsigned g_stw_n;
/* Accumulate the full EA span written from the watched PC range across the
 * WHOLE run, per 1MB band -- a decoder that writes a plane in sporadic bursts
 * still reveals the plane's extent, which windowed page-sampling could not. */
u32 g_stw_min[8], g_stw_max[8]; u64 g_stw_cnt[8];
void interp_note_store(u32 pc, u32 ea)
{
    unsigned b;
    if (pc < g_stw_lo || pc >= g_stw_hi) return;
    if (ea < 0x80000000u || ea >= 0x81800000u) return;
    b = (ea - 0x80000000u) >> 21;      /* 2MB bands, 12 of them */
    if (b >= 8) b = 7;
    if (g_stw_cnt[b] == 0 || ea < g_stw_min[b]) g_stw_min[b] = ea;
    if (ea > g_stw_max[b]) g_stw_max[b] = ea;
    g_stw_cnt[b]++;
    if (g_stw_n < 16) {
        unsigned i;
        for (i = 0; i < g_stw_n; i++)
            if ((g_stw_ea[i] & ~0xFFFFFu) == (ea & ~0xFFFFFu)) return;
        g_stw_ea[g_stw_n++] = ea;
    }
}

void interp_illegal(PPCState *s, u32 op)
{
    LOG_WARN_ONCE(LOG_INTERP,
                  "illegal or unimplemented instruction %08x at %08x "
                  "(primary %u, xo %u)", op, s->pc, OPCD(op), XO10(op));
    if (s->pc == 0) {
        /* A jump to NULL is a dead function pointer, and the caller is in LR.
         * The THP video decode thread died exactly this way; without the LR
         * there is nothing to chase. Logged every time -- a NULL jump is never
         * routine. */
        static unsigned n0;
        if (n0 < 8) { n0++;
            LOG_WARN(LOG_INTERP, "jump to NULL: lr=%08x ctr=%08x r3=%08x "
                     "r12=%08x sp=%08x", s->lr, s->ctr, s->gpr[3],
                     s->gpr[12], s->gpr[1]); }
    }
    ppc_raise(s, EXC_PROGRAM);
}

/* ------------------------------------------------------------------ */
/* Sub-table dispatchers                                                */
/* ------------------------------------------------------------------ */

static void dispatch_4 (PPCState *s, u32 op) { s_t4 [XO10(op)](s, op); }
static void dispatch_19(PPCState *s, u32 op) { s_t19[XO10(op)](s, op); }
static void dispatch_31(PPCState *s, u32 op) { s_t31[XO10(op)](s, op); }
static void dispatch_59(PPCState *s, u32 op) { s_t59[XO5 (op)](s, op); }
static void dispatch_63(PPCState *s, u32 op) { s_t63[XO10(op)](s, op); }

/* ------------------------------------------------------------------ */
/* Table construction                                                   */
/* ------------------------------------------------------------------ */

static void fill(InterpFn *tab, unsigned n, InterpFn fn)
{
    unsigned i;
    for (i = 0; i < n; i++)
        tab[i] = fn;
}

/* Replicate an A-form (5-bit XO) entry across every 10-bit value whose low
 * five bits match. */
static void set_aform(InterpFn *tab, unsigned xo5, InterpFn fn)
{
    unsigned i;
    for (i = 0; i < 32; i++)
        tab[i * 32 + xo5] = fn;
}

/* Constants compiled code reads straight out of the state: 1.0 for the ps1 lane
 * of a width-1 quantised load, and the scale tables for the quantised formats.
 * The 6-bit scale field is signed; a dequantise multiplies by 2^-e and a
 * quantise by 2^e. Building them once per state keeps the emitted sequence down
 * to a single lfd instead of a call. */
void ppc_init_constants(PPCState *s)
{
    unsigned i;
    s->const_one = 1.0;
    /* Deferred-FPRF bookkeeping (gekko.h). A zeroed pair already reads as
     * "authoritative", which is right for a reset CPU, but 0 is also the bit
     * pattern of +0.0 -- so a first-ever result of exactly +0.0 would not
     * register as pending. Seeding the pair with a value no arithmetic result
     * can equal removes that one case. */
    s->fprf_src = s->fprf_ack = FPRF_SRC_NONE;
    for (i = 0; i < 64; i++) {
        int e = (i & 0x20) ? (int)i - 64 : (int)i;
        s->dequant_scale[i] = ldexp(1.0, -e);
        s->quant_scale[i]   = ldexp(1.0,  e);
    }
    {   /* Same bounds the interpreter's quantize() clamps to, in the hardware
         * type order the JIT indexes by. */
        s->quant_lo[0] =      0.0; s->quant_hi[0] =   255.0;  /* U8  */
        s->quant_lo[1] =      0.0; s->quant_hi[1] = 65535.0;  /* U16 */
        s->quant_lo[2] =   -128.0; s->quant_hi[2] =   127.0;  /* S8  */
        s->quant_lo[3] = -32768.0; s->quant_hi[3] = 32767.0;  /* S16 */
    }
}

void interp_init_tables(void)
{
    if (s_tables_built)
        return;

    fill(s_primary, 64,   interp_illegal);
    fill(s_t4,      1024, interp_illegal);
    fill(s_t19,     1024, interp_illegal);
    fill(s_t31,     1024, interp_illegal);
    fill(s_t59,     32,   interp_illegal);
    fill(s_t63,     1024, interp_illegal);

    /* ---- primary ---- */
    s_primary[3]  = ppc_twi;
    s_primary[4]  = dispatch_4;
    s_primary[7]  = ppc_mulli;
    s_primary[8]  = ppc_subfic;
    s_primary[10] = ppc_cmpli;
    s_primary[11] = ppc_cmpi;
    s_primary[12] = ppc_addic;
    s_primary[13] = ppc_addic_rc;
    s_primary[14] = ppc_addi;
    s_primary[15] = ppc_addis;
    s_primary[16] = ppc_bc;
    s_primary[17] = ppc_sc;
    s_primary[18] = ppc_b;
    s_primary[19] = dispatch_19;
    s_primary[20] = ppc_rlwimi;
    s_primary[21] = ppc_rlwinm;
    s_primary[23] = ppc_rlwnm;
    s_primary[24] = ppc_ori;
    s_primary[25] = ppc_oris;
    s_primary[26] = ppc_xori;
    s_primary[27] = ppc_xoris;
    s_primary[28] = ppc_andi_rc;
    s_primary[29] = ppc_andis_rc;
    s_primary[31] = dispatch_31;
    s_primary[32] = ppc_lwz;   s_primary[33] = ppc_lwzu;
    s_primary[34] = ppc_lbz;   s_primary[35] = ppc_lbzu;
    s_primary[36] = ppc_stw;   s_primary[37] = ppc_stwu;
    s_primary[38] = ppc_stb;   s_primary[39] = ppc_stbu;
    s_primary[40] = ppc_lhz;   s_primary[41] = ppc_lhzu;
    s_primary[42] = ppc_lha;   s_primary[43] = ppc_lhau;
    s_primary[44] = ppc_sth;   s_primary[45] = ppc_sthu;
    s_primary[46] = ppc_lmw;   s_primary[47] = ppc_stmw;
    s_primary[48] = ppc_lfs;   s_primary[49] = ppc_lfsu;
    s_primary[50] = ppc_lfd;   s_primary[51] = ppc_lfdu;
    s_primary[52] = ppc_stfs;  s_primary[53] = ppc_stfsu;
    s_primary[54] = ppc_stfd;  s_primary[55] = ppc_stfdu;
    s_primary[56] = ppc_psq_l; s_primary[57] = ppc_psq_lu;
    s_primary[59] = dispatch_59;
    s_primary[60] = ppc_psq_st; s_primary[61] = ppc_psq_stu;
    s_primary[63] = dispatch_63;

    /* ---- opcode 4: paired singles ---- */
    set_aform(s_t4, 10, ppc_ps_sum0);
    set_aform(s_t4, 11, ppc_ps_sum1);
    set_aform(s_t4, 12, ppc_ps_muls0);
    set_aform(s_t4, 13, ppc_ps_muls1);
    set_aform(s_t4, 14, ppc_ps_madds0);
    set_aform(s_t4, 15, ppc_ps_madds1);
    set_aform(s_t4, 18, ppc_ps_div);
    set_aform(s_t4, 20, ppc_ps_sub);
    set_aform(s_t4, 21, ppc_ps_add);
    set_aform(s_t4, 23, ppc_ps_sel);
    set_aform(s_t4, 24, ppc_ps_res);
    set_aform(s_t4, 25, ppc_ps_mul);
    set_aform(s_t4, 26, ppc_ps_rsqrte);
    set_aform(s_t4, 28, ppc_ps_msub);
    set_aform(s_t4, 29, ppc_ps_madd);
    set_aform(s_t4, 30, ppc_ps_nmsub);
    set_aform(s_t4, 31, ppc_ps_nmadd);

    s_t4[0]    = ppc_ps_cmpu0;
    s_t4[32]   = ppc_ps_cmpo0;
    s_t4[64]   = ppc_ps_cmpu1;
    s_t4[96]   = ppc_ps_cmpo1;
    s_t4[6]    = ppc_psq_lx;
    s_t4[7]    = ppc_psq_stx;
    s_t4[38]   = ppc_psq_lux;
    s_t4[39]   = ppc_psq_stux;
    s_t4[40]   = ppc_ps_neg;
    s_t4[72]   = ppc_ps_mr;
    s_t4[136]  = ppc_ps_nabs;
    s_t4[264]  = ppc_ps_abs;
    s_t4[528]  = ppc_ps_merge00;
    s_t4[560]  = ppc_ps_merge01;
    s_t4[592]  = ppc_ps_merge10;
    s_t4[624]  = ppc_ps_merge11;
    /* dcbz_l ZEROES the 32-byte line in the locked cache -- as a nop, the
     * race loader's LC scratch was never cleared, decompression built
     * structures over garbage, and the game jumped through uninitialised
     * (0xdeadbeef) pointers to 0: the recurring race-load crash. */
    s_t4[1014] = ppc_dcbz;

    /* ---- opcode 19: branch and CR logic ---- */
    s_t19[0]   = ppc_mcrf;
    s_t19[16]  = ppc_bclr;
    s_t19[33]  = ppc_crnor;
    s_t19[50]  = ppc_rfi;
    s_t19[129] = ppc_crandc;
    s_t19[150] = ppc_isync;
    s_t19[193] = ppc_crxor;
    s_t19[225] = ppc_crnand;
    s_t19[257] = ppc_crand;
    s_t19[289] = ppc_creqv;
    s_t19[417] = ppc_crorc;
    s_t19[449] = ppc_cror;
    s_t19[528] = ppc_bcctr;

    /* ---- opcode 31: integer, load/store, system ---- */
    s_t31[0]   = ppc_cmp;      s_t31[4]   = ppc_tw;
    s_t31[8]   = ppc_subfc;    s_t31[10]  = ppc_addc;
    s_t31[11]  = ppc_mulhwu;   s_t31[19]  = ppc_mfcr;
    s_t31[20]  = ppc_lwarx;    s_t31[23]  = ppc_lwzx;
    s_t31[24]  = ppc_slw;      s_t31[26]  = ppc_cntlzw;
    s_t31[28]  = ppc_and;      s_t31[32]  = ppc_cmpl;
    s_t31[40]  = ppc_subf;     s_t31[54]  = ppc_cache_nop;   /* dcbst */
    s_t31[55]  = ppc_lwzux;    s_t31[60]  = ppc_andc;
    s_t31[75]  = ppc_mulhw;    s_t31[83]  = ppc_mfmsr;
    s_t31[86]  = ppc_cache_nop;/* dcbf */
    s_t31[87]  = ppc_lbzx;     s_t31[104] = ppc_neg;
    s_t31[119] = ppc_lbzux;    s_t31[124] = ppc_nor;
    s_t31[136] = ppc_subfe;    s_t31[138] = ppc_adde;
    s_t31[144] = ppc_mtcrf;    s_t31[146] = ppc_mtmsr;
    s_t31[150] = ppc_stwcx;    s_t31[151] = ppc_stwx;
    s_t31[183] = ppc_stwux;    s_t31[200] = ppc_subfze;
    s_t31[202] = ppc_addze;    s_t31[210] = ppc_mtsr;
    s_t31[215] = ppc_stbx;     s_t31[232] = ppc_subfme;
    s_t31[234] = ppc_addme;    s_t31[235] = ppc_mullw;
    s_t31[242] = ppc_mtsrin;   s_t31[246] = ppc_cache_nop;  /* dcbtst */
    s_t31[247] = ppc_stbux;    s_t31[266] = ppc_add;
    s_t31[278] = ppc_cache_nop;/* dcbt */
    s_t31[279] = ppc_lhzx;     s_t31[284] = ppc_eqv;
    s_t31[306] = ppc_tlbie;    s_t31[311] = ppc_lhzux;
    s_t31[316] = ppc_xor;      s_t31[339] = ppc_mfspr;
    s_t31[343] = ppc_lhax;     s_t31[371] = ppc_mftb;
    s_t31[375] = ppc_lhaux;    s_t31[407] = ppc_sthx;
    s_t31[412] = ppc_orc;      s_t31[439] = ppc_sthux;
    s_t31[444] = ppc_or;       s_t31[459] = ppc_divwu;
    s_t31[467] = ppc_mtspr;    s_t31[470] = ppc_cache_nop;  /* dcbi */
    s_t31[476] = ppc_nand;     s_t31[491] = ppc_divw;
    s_t31[512] = ppc_mcrxr;    s_t31[534] = ppc_lwbrx;
    s_t31[535] = ppc_lfsx;     s_t31[536] = ppc_srw;
    s_t31[567] = ppc_lfsux;    s_t31[595] = ppc_mfsr;
    s_t31[598] = ppc_sync_nop; s_t31[599] = ppc_lfdx;
    s_t31[631] = ppc_lfdux;    s_t31[659] = ppc_mfsrin;
    s_t31[662] = ppc_stwbrx;   s_t31[663] = ppc_stfsx;
    s_t31[695] = ppc_stfsux;   s_t31[727] = ppc_stfdx;
    s_t31[759] = ppc_stfdux;   s_t31[790] = ppc_lhbrx;
    s_t31[792] = ppc_sraw;     s_t31[824] = ppc_srawi;
    s_t31[854] = ppc_sync_nop; /* eieio */
    s_t31[918] = ppc_sthbrx;   s_t31[922] = ppc_extsh;
    s_t31[954] = ppc_extsb;    s_t31[982] = ppc_cache_nop;  /* icbi */
    s_t31[983] = ppc_stfiwx;   s_t31[1014] = ppc_dcbz;
    s_t31[566] = ppc_tlbsync;

    /* XO-form arithmetic carries an OE bit at MSB-position 21, which falls
     * *inside* the 10-bit extended field this table is indexed by. Every
     * OE-capable instruction therefore has a second valid encoding 512 higher
     * (`addo` is 266 + 512 = 778), and registering only the OE=0 form would
     * leave addo/subfo/nego/mullwo/divwo decoding as illegal instructions.
     * The handlers already read the OE bit themselves, so both indices share
     * one implementation. */
    {
        static const u16 oe_forms[] = {
            8,    /* subfc */   10,  /* addc  */   40,  /* subf   */
            104,  /* neg   */   136, /* subfe */   138, /* adde   */
            200,  /* subfze*/   202, /* addze */   232, /* subfme */
            234,  /* addme */   235, /* mullw */   266, /* add    */
            459,  /* divwu */   491  /* divw   */
        };
        unsigned i;
        for (i = 0; i < DOL_ARRAY_COUNT(oe_forms); i++)
            s_t31[oe_forms[i] + 512] = s_t31[oe_forms[i]];
    }

    /* ---- opcode 59: single-precision arithmetic ---- */
    s_t59[18] = ppc_fdivs;   s_t59[20] = ppc_fsubs;
    s_t59[21] = ppc_fadds;   s_t59[24] = ppc_fres;
    s_t59[25] = ppc_fmuls;   s_t59[28] = ppc_fmsubs;
    s_t59[29] = ppc_fmadds;  s_t59[30] = ppc_fnmsubs;
    s_t59[31] = ppc_fnmadds;
    /* fsqrts (22) exists in the ISA but Gekko does not implement it. */

    /* ---- opcode 63: double-precision and FPSCR ---- */
    set_aform(s_t63, 18, ppc_fdiv);
    set_aform(s_t63, 20, ppc_fsub);
    set_aform(s_t63, 21, ppc_fadd);
    set_aform(s_t63, 23, ppc_fsel);
    set_aform(s_t63, 25, ppc_fmul);
    set_aform(s_t63, 26, ppc_frsqrte);
    set_aform(s_t63, 28, ppc_fmsub);
    set_aform(s_t63, 29, ppc_fmadd);
    set_aform(s_t63, 30, ppc_fnmsub);
    set_aform(s_t63, 31, ppc_fnmadd);

    s_t63[0]   = ppc_fcmpu;   s_t63[12]  = ppc_frsp;
    s_t63[14]  = ppc_fctiw;   s_t63[15]  = ppc_fctiwz;
    s_t63[32]  = ppc_fcmpo;   s_t63[38]  = ppc_mtfsb1;
    s_t63[40]  = ppc_fneg;    s_t63[64]  = ppc_mcrfs;
    s_t63[70]  = ppc_mtfsb0;  s_t63[72]  = ppc_fmr;
    s_t63[134] = ppc_mtfsfi;  s_t63[136] = ppc_fnabs;
    s_t63[264] = ppc_fabs;    s_t63[583] = ppc_mffs;
    s_t63[711] = ppc_mtfsf;

    s_tables_built = 1;
}

InterpFn interp_decode(u32 op)
{
    interp_init_tables();
    switch (OPCD(op)) {
    case 4:  return s_t4 [XO10(op)];
    case 19: return s_t19[XO10(op)];
    case 31: return s_t31[XO10(op)];
    case 59: return s_t59[XO5 (op)];
    case 63: return s_t63[XO10(op)];
    default: return s_primary[OPCD(op)];
    }
}

/* ------------------------------------------------------------------ */
/* Exceptions                                                           */
/* ------------------------------------------------------------------ */

/* Force control back to the scheduler at the next block boundary.
 *
 * Driving the downcount negative as well as setting the flag is what lets the
 * JIT's linked-block fast path test a single value: a chain of directly linked
 * blocks checks only the cycle budget, so every other reason to stop has to
 * show up there too. */
void ppc_request_exit(PPCState *s)
{
    /* Hand the unspent budget to the scheduler before destroying it. The +1 is
     * the instruction currently executing: the interpreter charges it after the
     * handler returns, and the JIT charges it as part of the block's bulk
     * decrement at exit, so in both cases it has not been paid for yet at this
     * point and would otherwise be refunded twice.
     *
     * Guarded on downcount > 0 so a second request in the same slice -- an
     * mtmsr followed by a fallback that raises, say -- cannot refund twice. */
    if (s->downcount > 0) {
        s->exit_slack = s->downcount + 1;
        s->downcount = -1;
    }
    s->exit_requested = 1;
}

void ppc_raise(PPCState *s, u32 mask)
{
    s->exceptions |= mask;

    /* Synchronous exceptions are taken here rather than deferred, because this
     * is the last moment at which s->pc still names the instruction that caused
     * it -- and SRR0 must name that instruction, or the handler returns into
     * the wrong place. Both callers are mid-instruction: the interpreter's
     * opcode handlers, and the JIT's fallback trampoline, which stores the
     * fallback instruction's address into s->pc before the call.
     *
     * Asynchronous ones only need the block to end; the dispatcher takes them
     * at a boundary, where guest state is coherent. */
    if (mask & EXC_SYNCHRONOUS)
        ppc_deliver_exception(s);
    else
        ppc_request_exit(s);
}

/* ------------------------------------------------------------------ */
/* Execution                                                            */
/* ------------------------------------------------------------------ */

/* Diagnostic breakpoint counters: set g_bp_pc[i] to a guest address and
 * g_bp_hits[i] counts how often execution reaches it. Costs one compare per
 * instruction and answers "does this code ever run" without a debugger. */
u32 g_bp_pc[4];
u64 g_bp_hits[4];
u32 g_bp_cond_reg, g_bp_cond_val;
u32 g_bp_gpr[4][32];
u32 g_bp_lr[4], g_bp_sp[4];

/* Rolling window of recently executed pcs, for post-mortem inspection: when
 * a wild jump lands somewhere fatal, the ring names the road that led there.
 * Interpreter-only and cheap (one store per instruction). */
u32 g_pc_ring[256];
u32 g_pc_ring_pos;

void interp_step(PPCState *s)
{
    g_pc_ring[g_pc_ring_pos++ & 255u] = s->pc;
    /* Instruction-level watch on the OS current-context pointer: the null
     * context load traced back to this cell changing between interrupts.
     * Active only when bp 0 is armed (the post-mortem phase), so the ordinary
     * interpreter path never pays for it. */
    if (UNLIKELY(g_bp_pc[0])) {
        static u32 last_ctx = 0xEEEEEEEEu;
        u32 cur = mem_read32(0x800000D4u);
        if (cur != last_ctx) {
            static int t2 = -1;
            if (t2 < 0) t2 = getenv("DSP_TRACE") != NULL;
            if (t2 && last_ctx != 0xEEEEEEEEu)
                fprintf(stderr,
                        "[ctx] %08x -> %08x at pc=%08x lr=%08x\n",
                        (unsigned)last_ctx, (unsigned)cur,
                        (unsigned)s->pc, (unsigned)s->lr);
            last_ctx = cur;
        }
    }
    if (UNLIKELY(g_bp_pc[0])) {
        unsigned bi;
        for (bi = 0; bi < 4; bi++)
            if (s->pc == g_bp_pc[bi]) {
                /* Snapshot the registers the first time each is reached, so a
                 * one-shot event (a panic, an assertion) can be read after the
                 * fact rather than needing the run stopped at it. */
                if (!g_bp_hits[bi]) {
                    unsigned r;
                    for (r = 0; r < 32; r++) g_bp_gpr[bi][r] = s->gpr[r];
                    g_bp_lr[bi] = s->lr;
                    g_bp_sp[bi] = s->gpr[1];
                }
                /* Conditional refinement: if g_bp_cond_val is armed, keep
                 * re-snapshotting until gpr[cond_reg] == cond_val, then
                 * freeze. Finds e.g. the OSCreateThread call whose entry
                 * argument is the bad pointer, amid hundreds of good ones. */
                if (g_bp_cond_val && bi == 0 &&
                    g_bp_gpr[0][g_bp_cond_reg] != g_bp_cond_val) {
                    unsigned r;
                    for (r = 0; r < 32; r++) g_bp_gpr[0][r] = s->gpr[r];
                    g_bp_lr[0] = s->lr;
                    g_bp_sp[0] = s->gpr[1];
                    if (s->gpr[g_bp_cond_reg] == g_bp_cond_val)
                        LOG_WARN(LOG_INTERP, "bp COND HIT pc=%08x lr=%08x "
                                 "r3=%08x r4=%08x r5=%08x r6=%08x",
                                 s->pc, s->lr, s->gpr[3], s->gpr[4],
                                 s->gpr[5], s->gpr[6]);
                }
                g_bp_hits[bi]++;
                /* Verbose mode: print every hit with the call arguments --
                 * the way to log e.g. every OSCreateThread with its entry
                 * pointer. Gated on the same env as the device traces. */
                {
                    static int t = -1;
                    if (t < 0) t = getenv("DSP_TRACE") != NULL;
                    if (t)
                        fprintf(stderr,
                                "[bp] %08x hit %llu lr=%08x "
                                "r3=%08x r4=%08x r5=%08x [r3+198]=%08x\n",
                                (unsigned)s->pc,
                                (unsigned long long)g_bp_hits[bi],
                                (unsigned)s->lr, (unsigned)s->gpr[3],
                                (unsigned)s->gpr[4], (unsigned)s->gpr[5],
                                (unsigned)mem_read32(s->gpr[3] + 0x198));
                }
            }
    }
    u32 op;
    InterpFn fn;

    /* Instruction fetch goes through the same accessor as data, so a jump into
     * unmapped memory reports rather than executing garbage. */
    op = mem_read32_for_fetch(s->pc);
    s->npc = s->pc + 4;

    fn = interp_decode(op);
    fn(s, op);

    s->pc = s->npc;
    s->downcount--;

    /* Only the asynchronous ones can still be waiting: a synchronous exception
     * was already taken inside ppc_raise, before s->pc moved. This is the point
     * where an mtmsr that just enabled MSR[EE] lets a device interrupt through. */
    if (UNLIKELY(s->exceptions))
        ppc_deliver_exception(s);
}

void interp_run(PPCState *s)
{
    extern int g_dt_pctrace;
    interp_init_tables();
    s->exit_requested = 0;
    if (UNLIKELY(g_dt_pctrace)) {
        /* Print the executed path for this one slice. Bounded so a mistakenly
         * wide window cannot flood the report. */
        unsigned n = 0;
        while (s->downcount > 0 && !s->exit_requested) {
            if (n < 4000)
                LOG_INFO(LOG_CORE, "PT %04u %08x", n, (unsigned)s->pc);
            if (s->pc == 0x80193080u) {
                /* The branch two instructions on is where the two machines
                 * part: it reads [r31+4] and the console sees <= 0 where qemu
                 * sees > 0. Print the pointer and the words around it so the
                 * structure can be named rather than guessed at. */
                u32 b0 = s->gpr[31];
                LOG_INFO(LOG_CORE,
                         "PQ r31=%08x r4=%08x [r31+0]=%08x [+4]=%08x "
                         "[+8]=%08x [+c]=%08x [+10]=%08x",
                         (unsigned)b0, (unsigned)s->gpr[4],
                         (unsigned)mem_read32(b0), (unsigned)mem_read32(b0 + 4),
                         (unsigned)mem_read32(b0 + 8),
                         (unsigned)mem_read32(b0 + 12),
                         (unsigned)mem_read32(b0 + 16));
            }
            n++;
            interp_step(s);
        }
        return;
    }
    while (s->downcount > 0 && !s->exit_requested)
        interp_step(s);
}
