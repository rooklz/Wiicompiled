/* interp_loadstore.c — load, store, cache and reservation instructions.
 *
 * Guest memory goes through the slow accessors in memmap.c here. The JIT does
 * not: it emits a folded indexed access directly against the arena
 * (ARCHITECTURE.md §3.2). Keeping the interpreter on the slow path is
 * deliberate -- it makes the oracle independent of the fastmem machinery, so a
 * bug in one cannot hide a bug in the other.
 */
#include "interp.h"
#include "interp_fputil.h"
#include "../../mem/memmap.h"
#include "../../../common/log.h"

/* ------------------------------------------------------------------ */
/* Alignment                                                            */
/*                                                                      */
/* Gekko raises an alignment exception for misaligned floating-point and        */
/* multiple-word accesses, but handles misaligned integer accesses in hardware. */
/* Titles do rely on the latter working.                                        */
/* ------------------------------------------------------------------ */

static int fp_misaligned(PPCState *s, u32 ea, unsigned align)
{
    if (LIKELY((ea & (align - 1)) == 0))
        return 0;
    s->spr[SPR_DAR] = ea;
    ppc_raise(s, EXC_ALIGNMENT);
    return 1;
}

/* ------------------------------------------------------------------ */
/* Integer loads                                                        */
/* ------------------------------------------------------------------ */

#define LOAD_D(name, expr)                                                  \
    void ppc_##name(PPCState *s, u32 op)                                    \
    { u32 ea = ea_d(s, op); s->gpr[RT(op)] = (expr); }

#define LOAD_D_U(name, expr)                                                \
    void ppc_##name(PPCState *s, u32 op)                                    \
    { u32 ea = ea_d(s, op); s->gpr[RT(op)] = (expr); s->gpr[RA(op)] = ea; }

#define LOAD_X(name, expr)                                                  \
    void ppc_##name(PPCState *s, u32 op)                                    \
    { u32 ea = ea_x(s, op); s->gpr[RT(op)] = (expr); }

#define LOAD_X_U(name, expr)                                                \
    void ppc_##name(PPCState *s, u32 op)                                    \
    { u32 ea = ea_x(s, op); s->gpr[RT(op)] = (expr); s->gpr[RA(op)] = ea; }

LOAD_D  (lbz,   mem_read8(ea))
LOAD_D_U(lbzu,  mem_read8(ea))
LOAD_X  (lbzx,  mem_read8(ea))
LOAD_X_U(lbzux, mem_read8(ea))

LOAD_D  (lhz,   mem_read16(ea))
LOAD_D_U(lhzu,  mem_read16(ea))
LOAD_X  (lhzx,  mem_read16(ea))
LOAD_X_U(lhzux, mem_read16(ea))

LOAD_D  (lha,   (u32)(s32)(s16)mem_read16(ea))
LOAD_D_U(lhau,  (u32)(s32)(s16)mem_read16(ea))
LOAD_X  (lhax,  (u32)(s32)(s16)mem_read16(ea))
LOAD_X_U(lhaux, (u32)(s32)(s16)mem_read16(ea))

LOAD_D  (lwz,   mem_read32(ea))
LOAD_D_U(lwzu,  mem_read32(ea))
LOAD_X  (lwzx,  mem_read32(ea))
LOAD_X_U(lwzux, mem_read32(ea))

/* Byte-reversed forms. These exist in the guest ISA for little-endian data and
 * are genuinely byte-swapping here -- unlike ordinary guest access, which needs
 * no swap at all on this host. */
LOAD_X(lhbrx, (u32)DOL_SWAP16((u16)((mem_read8(ea) << 8) | mem_read8(ea + 1))))
LOAD_X(lwbrx, ((u32)mem_read8(ea)) | ((u32)mem_read8(ea + 1) << 8) |
              ((u32)mem_read8(ea + 2) << 16) | ((u32)mem_read8(ea + 3) << 24))

#undef LOAD_D
#undef LOAD_D_U
#undef LOAD_X
#undef LOAD_X_U

/* ------------------------------------------------------------------ */
/* Integer stores                                                       */
/* ------------------------------------------------------------------ */

extern u32 g_stw_lo, g_stw_hi;
void interp_note_store(u32 pc, u32 ea);

#define STORE_D(name, stmt)                                                 \
    void ppc_##name(PPCState *s, u32 op)                                    \
    { u32 ea = ea_d(s, op); u32 v = s->gpr[RS(op)]; (void)v; stmt;          \
      if (UNLIKELY(g_stw_hi)) interp_note_store(s->pc, ea); }

#define STORE_D_U(name, stmt)                                               \
    void ppc_##name(PPCState *s, u32 op)                                    \
    { u32 ea = ea_d(s, op); u32 v = s->gpr[RS(op)]; (void)v; stmt;          \
      s->gpr[RA(op)] = ea; }

#define STORE_X(name, stmt)                                                 \
    void ppc_##name(PPCState *s, u32 op)                                    \
    { u32 ea = ea_x(s, op); u32 v = s->gpr[RS(op)]; (void)v; stmt;          \
      if (UNLIKELY(g_stw_hi)) interp_note_store(s->pc, ea); }

#define STORE_X_U(name, stmt)                                               \
    void ppc_##name(PPCState *s, u32 op)                                    \
    { u32 ea = ea_x(s, op); u32 v = s->gpr[RS(op)]; (void)v; stmt;          \
      s->gpr[RA(op)] = ea; }

STORE_D  (stb,   mem_write8(ea, (u8)v))
STORE_D_U(stbu,  mem_write8(ea, (u8)v))
STORE_X  (stbx,  mem_write8(ea, (u8)v))
STORE_X_U(stbux, mem_write8(ea, (u8)v))

STORE_D  (sth,   mem_write16(ea, (u16)v))
STORE_D_U(sthu,  mem_write16(ea, (u16)v))
STORE_X  (sthx,  mem_write16(ea, (u16)v))
STORE_X_U(sthux, mem_write16(ea, (u16)v))

STORE_D  (stw,   mem_write32(ea, v))
STORE_D_U(stwu,  mem_write32(ea, v))
STORE_X  (stwx,  mem_write32(ea, v))
STORE_X_U(stwux, mem_write32(ea, v))

STORE_X(sthbrx, mem_write8(ea, (u8)v); mem_write8(ea + 1, (u8)(v >> 8)))
STORE_X(stwbrx, mem_write8(ea, (u8)v); mem_write8(ea + 1, (u8)(v >> 8));
                mem_write8(ea + 2, (u8)(v >> 16)); mem_write8(ea + 3, (u8)(v >> 24)))

#undef STORE_D
#undef STORE_D_U
#undef STORE_X
#undef STORE_X_U

/* ------------------------------------------------------------------ */
/* Multiple word                                                        */
/* ------------------------------------------------------------------ */

void ppc_lmw(PPCState *s, u32 op)
{
    u32 ea = ea_d(s, op), r;
    if (fp_misaligned(s, ea, 4)) return;
    for (r = RT(op); r < 32; r++, ea += 4)
        s->gpr[r] = mem_read32(ea);
}

void ppc_stmw(PPCState *s, u32 op)
{
    u32 ea = ea_d(s, op), r;
    if (fp_misaligned(s, ea, 4)) return;
    for (r = RS(op); r < 32; r++, ea += 4)
        mem_write32(ea, s->gpr[r]);
}

/* ------------------------------------------------------------------ */
/* Reservation (lwarx / stwcx.)                                         */
/*                                                                      */
/* Single-CPU guest, so the reservation only ever loses to itself. Modelling it */
/* properly still matters: titles use it for lock-free queues shared with the   */
/* DSP and DMA engines, and a stwcx. that always succeeds turns a retry loop    */
/* into a silently wrong fast path.                                             */
/* ------------------------------------------------------------------ */

void ppc_lwarx(PPCState *s, u32 op)
{
    u32 ea = ea_x(s, op);
    if (fp_misaligned(s, ea, 4)) return;
    s->gpr[RT(op)]     = mem_read32(ea);
    s->reserve_addr    = ea;
    s->reserve_valid   = 1;
}

void ppc_stwcx(PPCState *s, u32 op)
{
    u32 ea = ea_x(s, op);
    u32 f;

    if (fp_misaligned(s, ea, 4)) return;

    if (s->reserve_valid && s->reserve_addr == ea) {
        mem_write32(ea, s->gpr[RS(op)]);
        s->reserve_valid = 0;
        f = CR_EQ;
    } else {
        s->reserve_valid = 0;
        f = 0;
    }
    if (s->xer_so)
        f |= CR_SO;
    s->cr = cr_set_field(s->cr, 0, f);
}

/* ------------------------------------------------------------------ */
/* Cache control                                                        */
/* ------------------------------------------------------------------ */

/* The guest's cache line is 32 bytes. The host PPE's is 128. Forwarding this
 * to a host `dcbz` would clear four times too much memory, so it is emulated
 * explicitly -- see docs/HARDWARE.md §5.4. */
void ppc_dcbz(PPCState *s, u32 op)
{
    u32 ea = ea_x(s, op) & ~31u;
    u8 zero[32];
    memset(zero, 0, sizeof zero);
    mem_write_block(ea, zero, sizeof zero);
}

/* Cache hints with no architectural effect on an emulator that has no guest
 * cache to maintain. They are listed explicitly rather than falling through to
 * the illegal-instruction handler, because they are extremely common and
 * treating them as illegal would be catastrophic. */
void ppc_cache_nop(PPCState *s, u32 op) { (void)s; (void)op; }

/* ------------------------------------------------------------------ */
/* Floating-point loads                                                 */
/*                                                                      */
/* Note `Fill`: a single-precision load writes *both* paired-single halves.     */
/* That is Gekko-specific behaviour and a common source of subtle bugs.         */
/* ------------------------------------------------------------------ */

#define LFS_BODY(eacalc, doupdate)                                          \
    u32 ea = eacalc;                                                        \
    u64 v;                                                                  \
    if (fp_misaligned(s, ea, 4)) return;                                    \
    v = ppc_convert_to_double(mem_read32(ea));                              \
    s->ps[FRT(op)].ps0.u = v;                                               \
    s->ps[FRT(op)].ps1.u = v;                                               \
    if (doupdate) s->gpr[RA(op)] = ea;

void ppc_lfs   (PPCState *s, u32 op) { LFS_BODY(ea_d(s, op), 0) }
void ppc_lfsu  (PPCState *s, u32 op) { LFS_BODY(ea_d(s, op), 1) }
void ppc_lfsx  (PPCState *s, u32 op) { LFS_BODY(ea_x(s, op), 0) }
void ppc_lfsux (PPCState *s, u32 op) { LFS_BODY(ea_x(s, op), 1) }
#undef LFS_BODY

/* A double-precision load writes ps0 only; ps1 is architecturally unchanged. */
#define LFD_BODY(eacalc, doupdate)                                          \
    u32 ea = eacalc;                                                        \
    if (fp_misaligned(s, ea, 4)) return;   /* word, not doubleword */       \
    s->ps[FRT(op)].ps0.u = mem_read64(ea);                                  \
    if (doupdate) s->gpr[RA(op)] = ea;

void ppc_lfd   (PPCState *s, u32 op) { LFD_BODY(ea_d(s, op), 0) }
void ppc_lfdu  (PPCState *s, u32 op) { LFD_BODY(ea_d(s, op), 1) }
void ppc_lfdx  (PPCState *s, u32 op) { LFD_BODY(ea_x(s, op), 0) }
void ppc_lfdux (PPCState *s, u32 op) { LFD_BODY(ea_x(s, op), 1) }
#undef LFD_BODY

/* ------------------------------------------------------------------ */
/* Floating-point stores                                                */
/* ------------------------------------------------------------------ */

#define STFS_BODY(eacalc, doupdate)                                         \
    u32 ea = eacalc;                                                        \
    if (fp_misaligned(s, ea, 4)) return;                                    \
    mem_write32(ea, ppc_convert_to_single(s->ps[FRT(op)].ps0.u));           \
    if (doupdate) s->gpr[RA(op)] = ea;

void ppc_stfs  (PPCState *s, u32 op) { STFS_BODY(ea_d(s, op), 0) }
void ppc_stfsu (PPCState *s, u32 op) { STFS_BODY(ea_d(s, op), 1) }
void ppc_stfsx (PPCState *s, u32 op) { STFS_BODY(ea_x(s, op), 0) }
void ppc_stfsux(PPCState *s, u32 op) { STFS_BODY(ea_x(s, op), 1) }
#undef STFS_BODY

#define STFD_BODY(eacalc, doupdate)                                         \
    u32 ea = eacalc;                                                        \
    if (fp_misaligned(s, ea, 4)) return;   /* word, not doubleword */       \
    mem_write64(ea, s->ps[FRT(op)].ps0.u);                                  \
    if (doupdate) s->gpr[RA(op)] = ea;

void ppc_stfd  (PPCState *s, u32 op) { STFD_BODY(ea_d(s, op), 0) }
void ppc_stfdu (PPCState *s, u32 op) { STFD_BODY(ea_d(s, op), 1) }
void ppc_stfdx (PPCState *s, u32 op) { STFD_BODY(ea_x(s, op), 0) }
void ppc_stfdux(PPCState *s, u32 op) { STFD_BODY(ea_x(s, op), 1) }
#undef STFD_BODY

/* Stores the low 32 bits of the FPR's bit pattern without conversion -- the
 * companion to fctiw, which leaves an integer in the low half of a double. */
void ppc_stfiwx(PPCState *s, u32 op)
{
    u32 ea = ea_x(s, op);
    if (fp_misaligned(s, ea, 4)) return;
    mem_write32(ea, (u32)s->ps[FRT(op)].ps0.u);
}
