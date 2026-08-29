/* realtest.h — run real compiler output through the emulator.
 *
 * Companion to difftest: where that compares the two engines on instructions
 * chosen by hand, this runs ordinary C compiled by GCC at -O2 and checks the
 * results against the same source compiled natively. It therefore covers
 * whatever the compiler decided to emit rather than whatever the author thought
 * to test.
 */
#ifndef DOLPHIN_CORE_PPC_REALTEST_H
#define DOLPHIN_CORE_PPC_REALTEST_H

#include "../../common/types.h"

typedef void (*RealTestOutFn)(void *ctx, const char *line);

/* Returns 0 when every check passed. */
int realtest_run_all(RealTestOutFn out, void *ctx, int *checks, int *failures);

/* Run a representative slice of that same compiler output `reps` times and
 * report how many guest instructions were executed.
 *
 * This exists because the synthetic benchmark loop measures an instruction mix
 * *I* chose, and the number that matters -- how fast a game runs -- depends on
 * the mix a compiler chooses. This workload is a sort (branch-heavy, memory
 * bound), a float transform, a dot product and 64-bit arithmetic on a 32-bit
 * machine: the shapes real title code is made of, including the carry chains
 * and division sequences the recompiler still falls back on. Returns 0 on
 * success. */
int realtest_benchmark(int use_jit, unsigned reps, u64 *guest_insts);

#endif
