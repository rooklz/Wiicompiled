/* jit_entry.S — the trampoline between C and recompiled guest code.
 *
 * The shape of this file is a direct consequence of the PPE's write-through L1
 * (docs/HARDWARE.md §1.2): every store is an L2 write, so saving the eighteen
 * non-volatile registers must happen *once per jit_run*, not once per block.
 * Hence the dispatch loop lives here, in the same frame, and compiled blocks
 * are jumped into and jump back -- they have no prologue, no epilogue, no stack
 * frame and no link-register traffic of their own.
 *
 * Register contract with compiled code:
 *
 *   r14  guest memory arena base
 *   r15  &PPCState
 *   r16  downcount (spilled to PPCState at every block exit)
 *   r17  address of .Ldispatch -- how a block returns
 *   r18..r31  guest GPR cache, freely clobbered by blocks
 *
 * lv2 uses the ELFv1 ABI, in which a function symbol names a descriptor
 * {code, toc, env} rather than code, so the entry point needs an .opd entry.
 */

	.machine "ppc64"

/* Must match offsetof(PPCState, downcount). A static assertion in jit.c
 * fails the build if the structure ever drifts from this. */
#define PPCSTATE_DOWNCOUNT	0x0A8

/* The entry symbol is declared two different ways depending on the ABI. lv2
 * uses ELFv1, in which a function symbol names a descriptor {code, toc, env}
 * placed in .opd. The big-endian PPC64 validation environment (musl + qemu)
 * uses ELFv2, in which the symbol is the code address directly, with a global
 * and a local entry point. The body below is identical; only the framing
 * differs, and the emitted blocks it enters are ABI-agnostic. */
#if defined(_CALL_ELF) && _CALL_ELF == 2

	.section ".text"
	.align 2
	.globl jit_enter
	.type  jit_enter,@function
jit_enter:
	/* Global entry: set up r2 from r12, then a local entry one instruction
	 * in. In a static binary everything shares one TOC, so this is cheap. */
	addis	2, 12, .TOC.-jit_enter@ha
	addi	2, 2, .TOC.-jit_enter@l
	.localentry jit_enter, . - jit_enter
.L.jit_enter:

#else

	.section ".opd","aw"
	.align 3
	.globl jit_enter
	.type  jit_enter,@function
jit_enter:
	.quad	.L.jit_enter, .TOC.@tocbase, 0
	.size	jit_enter, 24

	.section ".text"
	.align 2

/* void jit_enter(PPCState *s, u8 *membase)
 *   r3 = state, r4 = arena base */
.L.jit_enter:

#endif
	mflr	0
	std	0, 16(1)		/* LR into the caller's linkage area */
	stdu	1, -288(1)
	std	2, 40(1)		/* TOC save slot                     */

	/* Save the non-volatile GPRs the guest register cache will clobber. */
	std	14, 112(1)
	std	15, 120(1)
	std	16, 128(1)
	std	17, 136(1)
	std	18, 144(1)
	std	19, 152(1)
	std	20, 160(1)
	std	21, 168(1)
	std	22, 176(1)
	std	23, 184(1)
	std	24, 192(1)
	std	25, 200(1)
	std	26, 208(1)
	std	27, 216(1)
	std	28, 224(1)
	std	29, 232(1)
	std	30, 240(1)
	std	31, 248(1)

	mr	15, 3			/* r15 = PPCState*      */
	mr	14, 4			/* r14 = arena base     */
	lwz	16, PPCSTATE_DOWNCOUNT(15)	/* r16 = downcount */

	/* Put the address of .Ldispatch in r17. Done with a local `bl` so the
	 * code stays position-independent without needing a TOC entry. */
	bl	.Lhere
.Lhere:
	mflr	17
	addi	17, 17, (.Ldispatch - .Lhere)

.Ldispatch:
	/* jit_dispatch_c checks the downcount and exit flag itself and returns
	 * either the host code address of the next block, or NULL to stop.
	 * Keeping those checks in C means this file needs no PPCState offsets
	 * beyond the downcount. */
	mr	3, 15
	bl	jit_dispatch_c
	nop				/* ELFv1 TOC-restore slot for the linker */
	ld	2, 40(1)
	/* jit_dispatch_c may have executed guest instructions itself (an AOT
	 * function) and charged them to s->downcount. r16 is the live copy the
	 * next block decrements and spills, so it must be refreshed here or
	 * that charge would be silently undone at the next block exit. On the
	 * common no-AOT path this reloads the value the last block just
	 * spilled -- one L1 hit. */
	lwz	16, PPCSTATE_DOWNCOUNT(15)
	cmpdi	3, 0
	beq	.Ldone

	mtctr	3
	bctr				/* enter the block; it jumps back to r17 */

.Ldone:
	ld	14, 112(1)
	ld	15, 120(1)
	ld	16, 128(1)
	ld	17, 136(1)
	ld	18, 144(1)
	ld	19, 152(1)
	ld	20, 160(1)
	ld	21, 168(1)
	ld	22, 176(1)
	ld	23, 184(1)
	ld	24, 192(1)
	ld	25, 200(1)
	ld	26, 208(1)
	ld	27, 216(1)
	ld	28, 224(1)
	ld	29, 232(1)
	ld	30, 240(1)
	ld	31, 248(1)

	addi	1, 1, 288
	ld	0, 16(1)
	mtlr	0
	blr

	.size	.L.jit_enter, . - .L.jit_enter
