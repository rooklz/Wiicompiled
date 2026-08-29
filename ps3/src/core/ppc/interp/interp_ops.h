/* interp_ops.h — declarations for every interpreter opcode handler.
 * Split out so interp_core.c can build the decode tables without dragging in
 * the implementation files' internals. */
#ifndef DOLPHIN_CORE_PPC_INTERP_OPS_H
#define DOLPHIN_CORE_PPC_INTERP_OPS_H

#include "interp.h"

#define OP(name) void ppc_##name(PPCState *s, u32 op)

/* integer */
OP(addi); OP(addis); OP(addic); OP(addic_rc); OP(add); OP(addc); OP(adde);
OP(addme); OP(addze); OP(subf); OP(subfc); OP(subfe); OP(subfic); OP(subfme);
OP(subfze); OP(neg); OP(mulli); OP(mullw); OP(mulhw); OP(mulhwu); OP(divw);
OP(divwu); OP(and); OP(andc); OP(nand); OP(or); OP(orc); OP(nor); OP(xor);
OP(eqv); OP(andi_rc); OP(andis_rc); OP(ori); OP(oris); OP(xori); OP(xoris);
OP(extsb); OP(extsh); OP(cntlzw); OP(slw); OP(srw); OP(sraw); OP(srawi);
OP(rlwinm); OP(rlwnm); OP(rlwimi); OP(cmp); OP(cmpi); OP(cmpl); OP(cmpli);
OP(tw); OP(twi);

/* load/store */
OP(lbz); OP(lbzu); OP(lbzx); OP(lbzux); OP(lhz); OP(lhzu); OP(lhzx); OP(lhzux);
OP(lha); OP(lhau); OP(lhax); OP(lhaux); OP(lwz); OP(lwzu); OP(lwzx); OP(lwzux);
OP(lhbrx); OP(lwbrx); OP(stb); OP(stbu); OP(stbx); OP(stbux); OP(sth);
OP(sthu); OP(sthx); OP(sthux); OP(stw); OP(stwu); OP(stwx); OP(stwux);
OP(sthbrx); OP(stwbrx); OP(lmw); OP(stmw); OP(lwarx); OP(stwcx); OP(dcbz);
OP(cache_nop); OP(lfs); OP(lfsu); OP(lfsx); OP(lfsux); OP(lfd); OP(lfdu);
OP(lfdx); OP(lfdux); OP(stfs); OP(stfsu); OP(stfsx); OP(stfsux); OP(stfd);
OP(stfdu); OP(stfdx); OP(stfdux); OP(stfiwx);

/* branch / CR */
OP(b); OP(bc); OP(bclr); OP(bcctr); OP(sc); OP(crand); OP(cror); OP(crxor);
OP(crnand); OP(crnor); OP(creqv); OP(crandc); OP(crorc); OP(mcrf); OP(isync);

/* system */
OP(mfspr); OP(mtspr); OP(mftb); OP(mfcr); OP(mtcrf); OP(mcrxr); OP(mfmsr);
OP(mtmsr); OP(rfi); OP(mtsr); OP(mfsr); OP(mtsrin); OP(mfsrin); OP(tlbie);
OP(tlbsync); OP(sync_nop);

/* floating point */
OP(fadd); OP(fadds); OP(fsub); OP(fsubs); OP(fdiv); OP(fdivs); OP(fmul);
OP(fmuls); OP(fmadd); OP(fmadds); OP(fmsub); OP(fmsubs); OP(fnmadd);
OP(fnmadds); OP(fnmsub); OP(fnmsubs); OP(fsqrt); OP(fres); OP(frsqrte);
OP(fsel); OP(fmr); OP(fneg); OP(fabs); OP(fnabs); OP(frsp); OP(fctiw);
OP(fctiwz); OP(fcmpu); OP(fcmpo); OP(mffs); OP(mtfsf); OP(mtfsb0);
OP(mtfsb1); OP(mtfsfi); OP(mcrfs);

/* paired singles */
OP(ps_add); OP(ps_sub); OP(ps_mul); OP(ps_div); OP(ps_muls0); OP(ps_muls1);
OP(ps_madd); OP(ps_msub); OP(ps_nmadd); OP(ps_nmsub); OP(ps_madds0);
OP(ps_madds1); OP(ps_sum0); OP(ps_sum1); OP(ps_res); OP(ps_rsqrte);
OP(ps_sel); OP(ps_merge00); OP(ps_merge01); OP(ps_merge10); OP(ps_merge11);
OP(ps_mr); OP(ps_neg); OP(ps_abs); OP(ps_nabs); OP(ps_cmpu0); OP(ps_cmpo0);
OP(ps_cmpu1); OP(ps_cmpo1); OP(psq_l); OP(psq_lu); OP(psq_lx); OP(psq_lux);
OP(psq_st); OP(psq_stu); OP(psq_stx); OP(psq_stux);

#undef OP

#endif /* DOLPHIN_CORE_PPC_INTERP_OPS_H */
