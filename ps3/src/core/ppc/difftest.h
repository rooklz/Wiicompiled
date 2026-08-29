/* difftest.h — differential testing of the JIT against the interpreter.
 *
 * The verification that actually matters. The emitter is checked against LLVM
 * and the emitted code is read by eye, but "the emitted code is correct" and
 * "the emitted code *runs* correctly" are different claims. This runs the same
 * guest program twice from identical state -- once through the interpreter,
 * once through the recompiler -- and compares the entire architectural state
 * afterwards. Any disagreement is a JIT bug, localized to one program.
 *
 * It is written to run anywhere so the same tests execute on the workstation
 * (where the JIT compiles but the interpreter executes, so it validates the
 * harness) and on the PS3 (where the JIT genuinely executes, so it validates
 * the recompiler). Output goes through a caller-supplied sink because the
 * console has nowhere convenient to print to.
 */
#ifndef DOLPHIN_CORE_PPC_DIFFTEST_H
#define DOLPHIN_CORE_PPC_DIFFTEST_H

#include "gekko.h"

typedef void (*DiffOutFn)(void *ctx, const char *line);

typedef struct {
    unsigned cases_run;
    unsigned cases_failed;
    unsigned state_mismatches;
    int      jit_executed;      /* 0 if the host cannot run emitted code */
} DiffResults;

/* Runs the whole suite. Returns 0 when every case matched. */
int difftest_fuzz(DiffOutFn out, void *ctx, DiffResults *res, u32 seed, unsigned iters, unsigned oplen);
int difftest_run_all(DiffOutFn out, void *ctx, DiffResults *results);

#endif /* DOLPHIN_CORE_PPC_DIFFTEST_H */
