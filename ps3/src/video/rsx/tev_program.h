/* tev_program.h — TEV configuration to RSX fragment program.
 *
 * This is the translation the whole graphics plan rests on. Hollywood's TEV is
 * 16 configurable stages, each computing
 *
 *     out = (d ± lerp(a, b, c)) * scale + bias
 *
 * over four colour registers, eight texture units and eight rasterised
 * channels. RSX has no such unit; it has a programmable fragment pipeline. So
 * every distinct TEV configuration becomes a small program, generated once and
 * cached against a hash of the state that produced it (ARCHITECTURE.md §6.1).
 *
 * Why generate rather than interpret with an ubershader: NV4x branches in the
 * fragment unit very slowly, and its register file falls off a cliff -- two
 * vec4 temporaries run at full rate, three or four halve throughput. An
 * ubershader would need every stage's inputs live at once and would branch per
 * stage, losing on both counts at the same time. A specialised program uses
 * only the registers its configuration actually needs.
 *
 * The generated code is checked two ways, because encoding and lowering fail
 * differently. Each *instruction* is a verified encoding (fp_emitter.h, diffed
 * against cgcomp). Each *program* is executed by a small interpreter and
 * compared against a direct implementation of the TEV formula
 * (tests/test_tev.c) -- which is the only way to catch a lowering that emits
 * perfectly valid instructions computing the wrong thing.
 */
#ifndef DOLPHIN_VIDEO_RSX_TEV_PROGRAM_H
#define DOLPHIN_VIDEO_RSX_TEV_PROGRAM_H

#include "fp_emitter.h"
#include "../../core/gx/bp.h"

/* ------------------------------------------------------------------ */
/* TEV field layout                                                     */
/*                                                                      */
/* The colour and alpha combiners pack their operand selectors differently --   */
/* colour uses four bits and has sixteen sources, alpha uses three and has      */
/* eight -- which is the single easiest thing to get wrong here, because the    */
/* two registers otherwise look identical.                                      */
/* ------------------------------------------------------------------ */

/* Colour combiner (BP 0xC0 + 2n) */
#define TEV_CC_D_SHIFT      0
#define TEV_CC_C_SHIFT      4
#define TEV_CC_B_SHIFT      8
#define TEV_CC_A_SHIFT      12
#define TEV_CC_ARG_BITS     4
#define TEV_CC_BIAS_SHIFT   16
#define TEV_CC_SUB_SHIFT    18
#define TEV_CC_CLAMP_SHIFT  19
#define TEV_CC_SCALE_SHIFT  20
#define TEV_CC_DEST_SHIFT   22

/* Alpha combiner (BP 0xC1 + 2n) */
#define TEV_AC_D_SHIFT      4
#define TEV_AC_C_SHIFT      7
#define TEV_AC_B_SHIFT      10
#define TEV_AC_A_SHIFT      13
#define TEV_AC_ARG_BITS     3
#define TEV_AC_BIAS_SHIFT   16
#define TEV_AC_SUB_SHIFT    18
#define TEV_AC_CLAMP_SHIFT  19
#define TEV_AC_SCALE_SHIFT  20
#define TEV_AC_DEST_SHIFT   22

/* Colour operand sources. */
typedef enum {
    TEV_CC_PREV_RGB = 0,  TEV_CC_PREV_A  = 1,
    TEV_CC_C0_RGB   = 2,  TEV_CC_C0_A    = 3,
    TEV_CC_C1_RGB   = 4,  TEV_CC_C1_A    = 5,
    TEV_CC_C2_RGB   = 6,  TEV_CC_C2_A    = 7,
    TEV_CC_TEX_RGB  = 8,  TEV_CC_TEX_A   = 9,
    TEV_CC_RAS_RGB  = 10, TEV_CC_RAS_A   = 11,
    TEV_CC_ONE      = 12, TEV_CC_HALF    = 13,
    TEV_CC_KONST    = 14, TEV_CC_ZERO    = 15
} TevColorArg;

/* Alpha operand sources. Note these are *not* the colour values shifted -- the
 * two enumerations are independent. */
typedef enum {
    TEV_AC_PREV  = 0, TEV_AC_C0 = 1, TEV_AC_C1 = 2, TEV_AC_C2 = 3,
    TEV_AC_TEX   = 4, TEV_AC_RAS = 5, TEV_AC_KONST = 6, TEV_AC_ZERO = 7
} TevAlphaArg;

/* Bias 3 does not add anything: it switches the stage into a comparison mode
 * where `scale` selects the comparison. Treating it as a bias produces a stage
 * that is quietly wrong rather than obviously broken. */
#define TEV_BIAS_ZERO       0
#define TEV_BIAS_PLUS_HALF  1
#define TEV_BIAS_MINUS_HALF 2
#define TEV_BIAS_COMPARE    3

#define TEV_SCALE_1   0
#define TEV_SCALE_2   1
#define TEV_SCALE_4   2
#define TEV_SCALE_HALF 3

#define TEV_DEST_PREV 0
#define TEV_DEST_C0   1
#define TEV_DEST_C1   2
#define TEV_DEST_C2   3

/* ------------------------------------------------------------------ */
/* Register assignment                                                  */
/*                                                                      */
/* Fixed rather than allocated, for now. NV4x's cliff is at two vec4           */
/* temporaries and any real TEV chain exceeds that, so allocation cannot make   */
/* the difference between fast and slow here -- it can only reduce how far past */
/* the cliff we are. Correctness first; the cost is measured before it is       */
/* optimised, which is the same order the CPU work followed.                    */
/* ------------------------------------------------------------------ */

#define TEV_REG_PREV   0
#define TEV_REG_C0     1
#define TEV_REG_C1     2
#define TEV_REG_C2     3
#define TEV_REG_TEX    4    /* this stage's texture sample */
#define TEV_REG_TMP0   5    /* lerp scratch               */
#define TEV_REG_TMP1   6
/* A fragment instruction carries exactly one literal slot, so it can reference
 * exactly one constant. TEV routinely wants two in a single expression -- a
 * stage computing `1 - 1` or scaling by a constant while adding another is
 * ordinary. Extra constants are hoisted into these with a MOV first; without
 * that, both operands silently read the *same* literal and the stage computes
 * something plausible and wrong. */
#define TEV_REG_IMM0   7
#define TEV_REG_IMM1   8
#define TEV_REG_COUNT  9

/* The indirect unit's registers, all above the ones a program without an
 * indirect stage uses -- so such a program allocates exactly what it allocated
 * before this existed, and the register-count cliff is paid only by the
 * materials that ask for it.
 *
 * One register per indirect LOOKUP (there are at most four, and several TEV
 * stages routinely share one), plus the running texture coordinate, which has
 * to survive a stage because a stage can add the previous stage's coordinate
 * to its own. */
/* Ordered by how often a material touches them, because the highest register
 * TOUCHED is what the program declares and what the register-count cliff is
 * paid against: a material with one lookup and no bump reaches register 10 and
 * declares eleven, not sixteen. */
#define TEV_REG_COORD  9u           /* the coordinate a stage samples with */
#define TEV_REG_IND(n) (10u + (n))  /* 10..13: the four indirect samples   */
#define TEV_REG_ICRD   14u          /* the sample as three signed numbers  */
#define TEV_REG_BUMP   15u          /* the indirect unit's alpha bump      */
#define TEV_REG_COUNT_IND 16

/* What the generator needs beyond BP state: which konst values are selected,
 * and the rasterised colours, are supplied as fragment inputs and constants. */
typedef struct {
    u32 instructions;   /* instructions emitted (excluding literal slots) */
    u32 words;          /* total 32-bit words, literals included          */
    u32 temps_used;
    int truncated;      /* ran out of program space                       */
} TevProgramInfo;

/* Generate a fragment program implementing the pixel engine's current TEV
 * configuration. Returns 0 on success. */
int tev_generate(const BPState *bp, FPEmitter *e, TevProgramInfo *info);

/* Which texture-coordinate interpolants the generated fragment program reads,
 * as a bit per coordinate. The RSX routes only the coordinates named here. */
u32 tev_texcoord_mask(const BPState *bp);

/* A hash of every BP field the generated program depends on -- and nothing
 * else. This is the cache key, so it has to be exact in both directions: a
 * field that affects codegen but is left out gives a stale program, and a
 * field included that does not affect codegen just costs a recompile. */
u64 tev_state_hash(const BPState *bp);

#endif /* DOLPHIN_VIDEO_RSX_TEV_PROGRAM_H */
