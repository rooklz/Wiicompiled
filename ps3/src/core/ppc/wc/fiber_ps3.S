/* fiber_ps3.S -- the cooperative context switch, PS3 PPU (64-bit, ELFv1-style
 * function descriptors). Offsets match struct FiberCtx in fiber.h exactly.
 *
 * fiber_swap(FiberCtx *from [r3], FiberCtx *to [r4]):
 *   save the callee-saved machine state into *from, load it from *to, and
 *   return through *to's lr. A later swap back into *from returns from THIS
 *   call with everything intact. Leaf function: no stack frame of its own.
 *
 * fiber_boot: landing point for a fiber's first run. fiber_prime() seeds
 *   lr = the code address of fiber_boot, r14 = descriptor of the C entry,
 *   r15 = its argument, sp = a prepared top-of-stack with a null back chain.
 *   ELFv1 indirect-call discipline: entry at 0(desc), TOC at 8(desc), env at
 *   16(desc) into r11.
 */

#define CTX_SP      0x000
#define CTX_TOC     0x008
#define CTX_LR      0x010
#define CTX_CR      0x018
#define CTX_GPR     0x020   /* r14..r31, 8 bytes each                */
#define CTX_FPR     0x0B0   /* f14..f31, 8 bytes each                */
#define CTX_VRSAVE  0x140
#define CTX_VR      0x150   /* v20..v31, 16 bytes each               */

	.section ".opd", "aw"
	.align	3
	.globl	fiber_swap
fiber_swap:
	.quad	.fiber_swap, .TOC.@tocbase, 0
	.globl	fiber_boot
fiber_boot:
	.quad	.fiber_boot, .TOC.@tocbase, 0

	.text
	.align	2
	.globl	.fiber_swap
.fiber_swap:
	/* ---- save into *from (r3) ---- */
	std	1,  CTX_SP(3)
	std	2,  CTX_TOC(3)
	mflr	0
	std	0,  CTX_LR(3)
	mfcr	0
	std	0,  CTX_CR(3)
	std	14, CTX_GPR+0x00(3)
	std	15, CTX_GPR+0x08(3)
	std	16, CTX_GPR+0x10(3)
	std	17, CTX_GPR+0x18(3)
	std	18, CTX_GPR+0x20(3)
	std	19, CTX_GPR+0x28(3)
	std	20, CTX_GPR+0x30(3)
	std	21, CTX_GPR+0x38(3)
	std	22, CTX_GPR+0x40(3)
	std	23, CTX_GPR+0x48(3)
	std	24, CTX_GPR+0x50(3)
	std	25, CTX_GPR+0x58(3)
	std	26, CTX_GPR+0x60(3)
	std	27, CTX_GPR+0x68(3)
	std	28, CTX_GPR+0x70(3)
	std	29, CTX_GPR+0x78(3)
	std	30, CTX_GPR+0x80(3)
	std	31, CTX_GPR+0x88(3)
	stfd	14, CTX_FPR+0x00(3)
	stfd	15, CTX_FPR+0x08(3)
	stfd	16, CTX_FPR+0x10(3)
	stfd	17, CTX_FPR+0x18(3)
	stfd	18, CTX_FPR+0x20(3)
	stfd	19, CTX_FPR+0x28(3)
	stfd	20, CTX_FPR+0x30(3)
	stfd	21, CTX_FPR+0x38(3)
	stfd	22, CTX_FPR+0x40(3)
	stfd	23, CTX_FPR+0x48(3)
	stfd	24, CTX_FPR+0x50(3)
	stfd	25, CTX_FPR+0x58(3)
	stfd	26, CTX_FPR+0x60(3)
	stfd	27, CTX_FPR+0x68(3)
	stfd	28, CTX_FPR+0x70(3)
	stfd	29, CTX_FPR+0x78(3)
	stfd	30, CTX_FPR+0x80(3)
	stfd	31, CTX_FPR+0x88(3)
	mfspr	0, 256			/* VRSAVE */
	std	0,  CTX_VRSAVE(3)
	addi	5,  3, CTX_VR
	stvx	20, 0, 5
	addi	5,  5, 16
	stvx	21, 0, 5
	addi	5,  5, 16
	stvx	22, 0, 5
	addi	5,  5, 16
	stvx	23, 0, 5
	addi	5,  5, 16
	stvx	24, 0, 5
	addi	5,  5, 16
	stvx	25, 0, 5
	addi	5,  5, 16
	stvx	26, 0, 5
	addi	5,  5, 16
	stvx	27, 0, 5
	addi	5,  5, 16
	stvx	28, 0, 5
	addi	5,  5, 16
	stvx	29, 0, 5
	addi	5,  5, 16
	stvx	30, 0, 5
	addi	5,  5, 16
	stvx	31, 0, 5

	/* ---- restore from *to (r4) ---- */
	ld	1,  CTX_SP(4)
	ld	2,  CTX_TOC(4)
	ld	0,  CTX_LR(4)
	mtlr	0
	ld	0,  CTX_CR(4)
	mtcr	0
	ld	14, CTX_GPR+0x00(4)
	ld	15, CTX_GPR+0x08(4)
	ld	16, CTX_GPR+0x10(4)
	ld	17, CTX_GPR+0x18(4)
	ld	18, CTX_GPR+0x20(4)
	ld	19, CTX_GPR+0x28(4)
	ld	20, CTX_GPR+0x30(4)
	ld	21, CTX_GPR+0x38(4)
	ld	22, CTX_GPR+0x40(4)
	ld	23, CTX_GPR+0x48(4)
	ld	24, CTX_GPR+0x50(4)
	ld	25, CTX_GPR+0x58(4)
	ld	26, CTX_GPR+0x60(4)
	ld	27, CTX_GPR+0x68(4)
	ld	28, CTX_GPR+0x70(4)
	ld	29, CTX_GPR+0x78(4)
	ld	30, CTX_GPR+0x80(4)
	ld	31, CTX_GPR+0x88(4)
	lfd	14, CTX_FPR+0x00(4)
	lfd	15, CTX_FPR+0x08(4)
	lfd	16, CTX_FPR+0x10(4)
	lfd	17, CTX_FPR+0x18(4)
	lfd	18, CTX_FPR+0x20(4)
	lfd	19, CTX_FPR+0x28(4)
	lfd	20, CTX_FPR+0x30(4)
	lfd	21, CTX_FPR+0x38(4)
	lfd	22, CTX_FPR+0x40(4)
	lfd	23, CTX_FPR+0x48(4)
	lfd	24, CTX_FPR+0x50(4)
	lfd	25, CTX_FPR+0x58(4)
	lfd	26, CTX_FPR+0x60(4)
	lfd	27, CTX_FPR+0x68(4)
	lfd	28, CTX_FPR+0x70(4)
	lfd	29, CTX_FPR+0x78(4)
	lfd	30, CTX_FPR+0x80(4)
	lfd	31, CTX_FPR+0x88(4)
	ld	0,  CTX_VRSAVE(4)
	mtspr	256, 0
	addi	5,  4, CTX_VR
	lvx	20, 0, 5
	addi	5,  5, 16
	lvx	21, 0, 5
	addi	5,  5, 16
	lvx	22, 0, 5
	addi	5,  5, 16
	lvx	23, 0, 5
	addi	5,  5, 16
	lvx	24, 0, 5
	addi	5,  5, 16
	lvx	25, 0, 5
	addi	5,  5, 16
	lvx	26, 0, 5
	addi	5,  5, 16
	lvx	27, 0, 5
	addi	5,  5, 16
	lvx	28, 0, 5
	addi	5,  5, 16
	lvx	29, 0, 5
	addi	5,  5, 16
	lvx	30, 0, 5
	addi	5,  5, 16
	lvx	31, 0, 5
	blr

	.align	2
	.globl	.fiber_boot
.fiber_boot:
	/* First run of a primed fiber. r14 = descriptor of the C entry,
	 * r15 = argument, r1 = prepared stack top (null back chain in place).
	 * Build the ABI-minimum frame and make a proper descriptor call. */
	stdu	1, -112(1)
	mr	3, 15
	ld	0,  0(14)		/* entry address  */
	ld	2,  8(14)		/* entry TOC      */
	ld	11, 16(14)		/* environment    */
	mtctr	0
	bctrl
	/* The fiber entry must never return: it ends by switching away for
	 * good (guest OSExitThread path). If it does return, stop hard. */
1:	trap
	b	1b
