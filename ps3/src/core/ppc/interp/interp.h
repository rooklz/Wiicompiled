/* interp.h — Gekko interpreter: instruction fields, decode, dispatch.
 *
 * The interpreter is not the fast path and is not trying to be. Its job is to
 * be *obviously correct*, because it is the oracle the JIT is validated
 * against (docs/ARCHITECTURE.md §8.1). Every opcode here is written the way the
 * Gekko manual describes it, even where a cleverer formulation exists.
 *
 * It also serves as the JIT's fallback for instructions not worth recompiling
 * and for pages the JIT must not compile (self-modifying code, MMU-translated
 * code).
 */
#ifndef DOLPHIN_CORE_PPC_INTERP_H
#define DOLPHIN_CORE_PPC_INTERP_H

#include "../gekko.h"

/* ------------------------------------------------------------------ */
/* Instruction field extraction                                         */
/*                                                                      */
/* PowerPC numbers bits MSB-first, so field positions read "backwards" as shifts.*/
/* These are written once, here, and never open-coded elsewhere -- getting one   */
/* wrong produces an instruction that silently does something plausible.         */
/* ------------------------------------------------------------------ */

DOL_INLINE u32 OPCD(u32 op) { return (op >> 26) & 0x3Fu; }
DOL_INLINE u32 RS  (u32 op) { return (op >> 21) & 0x1Fu; }
DOL_INLINE u32 RT  (u32 op) { return (op >> 21) & 0x1Fu; }  /* same field */
DOL_INLINE u32 RA  (u32 op) { return (op >> 16) & 0x1Fu; }
DOL_INLINE u32 RB  (u32 op) { return (op >> 11) & 0x1Fu; }
DOL_INLINE u32 FRT (u32 op) { return (op >> 21) & 0x1Fu; }
DOL_INLINE u32 FRA (u32 op) { return (op >> 16) & 0x1Fu; }
DOL_INLINE u32 FRB (u32 op) { return (op >> 11) & 0x1Fu; }
DOL_INLINE u32 FRC (u32 op) { return (op >>  6) & 0x1Fu; }

DOL_INLINE s32 SIMM(u32 op) { return (s32)(s16)(op & 0xFFFFu); }
DOL_INLINE u32 UIMM(u32 op) { return op & 0xFFFFu; }

DOL_INLINE u32 RC_BIT(u32 op) { return op & 1u; }
DOL_INLINE u32 OE_BIT(u32 op) { return (op >> 10) & 1u; }
DOL_INLINE u32 LK_BIT(u32 op) { return op & 1u; }
DOL_INLINE u32 AA_BIT(u32 op) { return (op >> 1) & 1u; }

DOL_INLINE u32 BO(u32 op) { return (op >> 21) & 0x1Fu; }
DOL_INLINE u32 BI(u32 op) { return (op >> 16) & 0x1Fu; }
DOL_INLINE s32 BD(u32 op) { return (s32)(s16)(op & 0xFFFCu); }
/* LI is a 24-bit signed word displacement occupying bits 6..29. */
DOL_INLINE s32 LI(u32 op) { return ((s32)(op << 6)) >> 6 & ~3; }

DOL_INLINE u32 SH(u32 op) { return (op >> 11) & 0x1Fu; }
DOL_INLINE u32 MB(u32 op) { return (op >>  6) & 0x1Fu; }
DOL_INLINE u32 ME(u32 op) { return (op >>  1) & 0x1Fu; }

DOL_INLINE u32 CRFD(u32 op) { return (op >> 23) & 7u; }
DOL_INLINE u32 CRFS(u32 op) { return (op >> 18) & 7u; }
DOL_INLINE u32 CRBD(u32 op) { return (op >> 21) & 0x1Fu; }
DOL_INLINE u32 CRBA(u32 op) { return (op >> 16) & 0x1Fu; }
DOL_INLINE u32 CRBB(u32 op) { return (op >> 11) & 0x1Fu; }
DOL_INLINE u32 CRM (u32 op) { return (op >> 12) & 0xFFu; }
DOL_INLINE u32 FM  (u32 op) { return (op >> 17) & 0xFFu; }

/* The SPR field is stored with its two 5-bit halves swapped. */
DOL_INLINE u32 SPRN(u32 op)
{ return ((op >> 16) & 0x1Fu) | ((op >> 6) & 0x3E0u); }

DOL_INLINE u32 XO10(u32 op) { return (op >> 1) & 0x3FFu; }
DOL_INLINE u32 XO5 (u32 op) { return (op >> 1) & 0x1Fu;  }

/* Paired-single specific fields (opcode 4 / psq_l / psq_st). */
DOL_INLINE u32 PS_I(u32 op)  { return (op >> 12) & 7u; }     /* GQR index   */
DOL_INLINE u32 PS_W(u32 op)  { return (op >> 15) & 1u; }     /* width flag  */
DOL_INLINE s32 PS_D(u32 op)  { return dol_sext32(op & 0xFFFu, 12); }
DOL_INLINE u32 PSX_I(u32 op) { return (op >> 7) & 7u; }      /* indexed form */
DOL_INLINE u32 PSX_W(u32 op) { return (op >> 10) & 1u; }

/* ------------------------------------------------------------------ */
/* Effective address                                                    */
/* ------------------------------------------------------------------ */

/* PowerPC's "RA or 0": register 0 in the RA slot reads as literal zero for
 * address formation, which is not the same as reading GPR0. */
DOL_INLINE u32 ea_ra_or_0(const PPCState *s, u32 op)
{ u32 a = RA(op); return a ? s->gpr[a] : 0u; }

DOL_INLINE u32 ea_d(const PPCState *s, u32 op)
{ return ea_ra_or_0(s, op) + (u32)SIMM(op); }

DOL_INLINE u32 ea_x(const PPCState *s, u32 op)
{ return ea_ra_or_0(s, op) + s->gpr[RB(op)]; }

/* ------------------------------------------------------------------ */
/* Dispatch                                                             */
/* ------------------------------------------------------------------ */

typedef void (*InterpFn)(PPCState *s, u32 op);

/* Build the decode tables. Idempotent; safe to call more than once. */
void interp_init_tables(void);

/* Fill the constants compiled code reads from the state (1.0 and the quantised
 * scale tables). Call once per PPCState before running anything. */
void ppc_init_constants(PPCState *s);

/* Decode one instruction to its handler. Never returns NULL: unknown encodings
 * resolve to a handler that raises a program exception, exactly as hardware
 * does, rather than being silently skipped. */
InterpFn interp_decode(u32 op);

/* Execute a single instruction at s->pc. Advances pc/npc. */
void interp_step(PPCState *s);

/* Run until downcount is exhausted or an exit is requested. */
void interp_run(PPCState *s);

/* Raise a pending exception; the dispatcher services it at the next boundary. */
void ppc_raise(PPCState *s, u32 exception_mask);

/* Take the highest-priority pending exception, if one may be taken now.
 *
 * Synchronous exceptions are delivered by ppc_raise the moment they are raised,
 * because that is the only point at which s->pc still names the instruction
 * responsible. Callers therefore only need this for the asynchronous ones, at a
 * boundary. Returns 1 if an exception was taken, in which case pc and npc now
 * point at the handler. */
int ppc_deliver_exception(PPCState *s);

/* Whether ppc_deliver_exception would take something. Cheap enough to test on
 * every dispatch, which is where it is used. */
int ppc_exception_pending(const PPCState *s);

/* Force a return to the scheduler at the next block boundary. */
void ppc_request_exit(PPCState *s);

/* Shared helpers used across the opcode groups. */
void ppc_update_cr0_from(PPCState *s, u32 result);
void ppc_set_ov_so(PPCState *s, int overflow);

/* Handler for anything the decoder does not recognize. */
void interp_illegal(PPCState *s, u32 op);


/* Last synchronous fault captured at delivery (interp_system.c), for diagnostics. */
extern u32 g_pc_ring[256];
extern u32 g_pc_ring_pos;
extern u32 g_bp_pc[4];
extern u64 g_bp_hits[4];
extern u32 g_bp_cond_reg, g_bp_cond_val;
extern u32 g_bp_gpr[4][32];
extern u32 g_bp_lr[4], g_bp_sp[4];
extern u32 g_fault_vector, g_fault_srr0, g_fault_srr1, g_fault_dar, g_fault_dsisr;
extern unsigned g_fault_count;

#endif /* DOLPHIN_CORE_PPC_INTERP_H */
