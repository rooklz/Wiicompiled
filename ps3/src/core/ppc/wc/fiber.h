/* fiber.h -- cooperative context switch for the native port.
 *
 * The Wii runs every guest thread on ONE CPU, switching only at explicit
 * yield points. The translated scheduler (SelectThread's run queues, thread
 * states) is native guest code and the single source of truth; the only thing
 * translated code cannot express is the host-stack switch that OSLoadContext
 * implies, because translated code nests one host C frame per guest call.
 * That is a fiber: cooperative switch with a separate host stack per guest
 * thread, all on one host thread.
 *
 * ABI (proven by disassembly of this exact toolchain, not assumed): 64-bit
 * ELF with ELFv1-style function descriptors (.opd {entry, TOC, env}); LR save
 * at 16(r1), TOC save at 40(r1), back chain at 0(r1), minimum frame 112
 * bytes, 16-byte stack alignment. Callee-saved set: r1, r2, LR, CR, r14-r31,
 * f14-f31, v20-v31, VRSAVE. r13 is host-TLS and constant on the one guest
 * host thread, so it is not part of the swap. FPSCR is shared -- one CPU.
 * Saves are 64-bit doublewords: the compiler emits 64-bit register ops
 * (std/ld in every prologue), so newlib's 32-bit setjmp layout is NOT the
 * model to copy.
 */
#ifndef WC_FIBER_H
#define WC_FIBER_H

#include <cstdint>
#include <cstddef>

extern "C" {

/* One saved execution context. Layout is shared with fiber_ps3.S -- the
 * offsets there are generated from this order and MUST match:
 *
 *   0x000 sp        r1
 *   0x008 toc       r2  (constant program-wide in this static ELF; saved to
 *                        be safe and to seed new fibers)
 *   0x010 lr        resume address (code address, NOT a descriptor)
 *   0x018 cr        full CR (CR2-CR4 are the callee-saved fields)
 *   0x020 gpr[18]   r14..r31
 *   0x0B0 fpr[18]   f14..f31 (raw 64-bit)
 *   0x140 vrsave
 *   0x148 pad       (keeps vr[] 16-aligned)
 *   0x150 vr[12]    v20..v31, 16 bytes each
 *   0x210 total size
 */
struct FiberCtx {
    uint64_t sp;
    uint64_t toc;
    uint64_t lr;
    uint64_t cr;
    uint64_t gpr[18];
    uint64_t fpr[18];
    uint64_t vrsave;
    uint64_t pad;
    uint8_t  vr[12][16];
} __attribute__((aligned(16)));

/* Swap execution: save the current context into `from`, restore `to`, and
 * continue at `to`'s saved lr. When something later swaps back into `from`,
 * this call returns to its own caller with the full callee-saved state
 * intact. Defined in fiber_ps3.S. */
void fiber_swap(FiberCtx *from, FiberCtx *to);

/* Assembly trampoline: the first swap into a primed fiber "returns" here.
 * It builds a minimal ABI frame on the primed stack and calls the C function
 * whose DESCRIPTOR address was primed into gpr[0] (r14), passing the value
 * primed into gpr[1] (r15) as the argument. If that function ever returns,
 * it loops on a trap. Referenced by wc_fiber.cpp via fiber_boot_entry(),
 * which resolves the descriptor to the raw code address for ctx->lr. */
void fiber_boot(void);

} /* extern "C" */

/* Prime `ctx` so the first fiber_swap into it runs entry(arg) on the given
 * stack (host memory, 16-byte aligned internally). Implemented in
 * wc_fiber.cpp on top of the two assembly routines. */
void fiber_prime(FiberCtx *ctx, void *stack_base, size_t stack_size,
                 void (*entry)(void *), void *arg);

#endif /* WC_FIBER_H */
