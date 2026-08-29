/* fp_emitter.h — RSX fragment-program encoder.
 *
 * The GPU counterpart to ppc_emitter.h, and it exists for the same reason. TEV
 * is 16 configurable stages, and the only way to run that on an NV4x part at
 * full rate is to generate a *specialised* fragment program per unique TEV
 * state (ARCHITECTURE.md §6.1). Generating programs means encoding microcode,
 * and a wrong encoding here produces the same class of bug a wrong PowerPC
 * encoding does: not a crash, but a picture that is subtly wrong with nothing
 * pointing back at the cause.
 *
 * So the same discipline applies. Every encoding this header can produce is
 * compared against `cgcomp`, PSL1GHT's shader compiler, by
 * `tools/fp_emitter_selftest.c` -- the role `llvm-mc` plays for the CPU.
 *
 * Two properties of the format are worth stating up front because both are
 * invisible until something is already wrong:
 *
 * **Words are stored with their two 16-bit halves swapped.** `0x02003e00`, a
 * perfectly ordinary MUL, is written to memory as `0x3e000200`. Every field
 * below is defined against the *logical* word and the swap happens once, at
 * emit time, so nothing else has to think about it.
 *
 * **A constant is a whole extra instruction slot.** An instruction referencing
 * an immediate is followed by a 16-byte slot holding the four floats. The
 * program counter therefore does not advance uniformly, which matters the
 * moment anything branches.
 */
#ifndef DOLPHIN_VIDEO_RSX_FP_EMITTER_H
#define DOLPHIN_VIDEO_RSX_FP_EMITTER_H

#include "../../common/types.h"

/* ------------------------------------------------------------------ */
/* Opcodes                                                              */
/* ------------------------------------------------------------------ */

#define FP_OP_NOP    0x00
#define FP_OP_MOV    0x01
#define FP_OP_MUL    0x02
#define FP_OP_ADD    0x03
#define FP_OP_MAD    0x04
#define FP_OP_DP3    0x05
#define FP_OP_DP4    0x06
#define FP_OP_DST    0x07
#define FP_OP_MIN    0x08
#define FP_OP_MAX    0x09
#define FP_OP_SLT    0x0A
#define FP_OP_SGE    0x0B
#define FP_OP_SLE    0x0C
#define FP_OP_SGT    0x0D
#define FP_OP_SNE    0x0E
#define FP_OP_SEQ    0x0F
#define FP_OP_FRC    0x10
#define FP_OP_FLR    0x11
#define FP_OP_KIL    0x12
#define FP_OP_PK4B   0x13
#define FP_OP_UP4B   0x14
#define FP_OP_DDX    0x15
#define FP_OP_DDY    0x16
#define FP_OP_TEX    0x17
#define FP_OP_TXP    0x18
#define FP_OP_TXD    0x19
#define FP_OP_RCP    0x1A
#define FP_OP_RSQ    0x1B
#define FP_OP_EX2    0x1C
#define FP_OP_LG2    0x1D
#define FP_OP_LIT    0x1E
#define FP_OP_LRP    0x1F
#define FP_OP_STR    0x20
#define FP_OP_SFL    0x21
#define FP_OP_COS    0x22
#define FP_OP_SIN    0x23
#define FP_OP_PK2H   0x24
#define FP_OP_UP2H   0x25
#define FP_OP_POW    0x26
#define FP_OP_PK4UB  0x27
#define FP_OP_UP4UB  0x28
#define FP_OP_PK2US  0x29
#define FP_OP_UP2US  0x2A
#define FP_OP_DP2A   0x2B
#define FP_OP_TXL    0x2C
#define FP_OP_TXB    0x31
#define FP_OP_DIV    0x3A

/* ------------------------------------------------------------------ */
/* Word 0: destination, mask, precision, opcode                         */
/* ------------------------------------------------------------------ */

#define FP_PROGRAM_END        (1u << 0)
#define FP_OUT_REG_SHIFT      1
#define FP_OUT_REG_HALF       (1u << 7)
#define FP_COND_WRITE_ENABLE  (1u << 8)
#define FP_OUTMASK_SHIFT      9
#define FP_INPUT_SRC_SHIFT    13
#define FP_TEX_UNIT_SHIFT     17
#define FP_PRECISION_SHIFT    22
#define FP_OUT_SAT            (1u << 31)
#define FP_OPCODE_SHIFT       24
#define FP_OUT_NONE           (1u << 30)

/* Precision. The Wii's TEV works in roughly 8-bit fixed point, so most
 * generated stages fit the narrow forms -- and on NV4x the narrow forms are
 * what keep a long TEV chain under the register-count cliff. */
#define FP_PREC_FP32  0
#define FP_PREC_FP16  1
#define FP_PREC_FX12  2

/* Write mask, one bit per component. */
#define FP_MASK_X  0x1u
#define FP_MASK_Y  0x2u
#define FP_MASK_Z  0x4u
#define FP_MASK_W  0x8u
#define FP_MASK_XYZ  (FP_MASK_X | FP_MASK_Y | FP_MASK_Z)
#define FP_MASK_ALL  (FP_MASK_XYZ | FP_MASK_W)

/* ------------------------------------------------------------------ */
/* Source operands (words 1..3)                                         */
/* ------------------------------------------------------------------ */

#define FP_REG_TYPE_TEMP    0u
#define FP_REG_TYPE_INPUT   1u
#define FP_REG_TYPE_CONST   2u

#define FP_REG_TYPE_SHIFT   0
#define FP_REG_SRC_SHIFT    2
#define FP_REG_SRC_HALF     (1u << 8)
#define FP_REG_SWZ_X_SHIFT  9
#define FP_REG_SWZ_Y_SHIFT  11
#define FP_REG_SWZ_Z_SHIFT  13
#define FP_REG_SWZ_W_SHIFT  15
#define FP_REG_NEGATE       (1u << 17)

/* `abs` sits in a different bit for source 0 than for 1 and 2 -- the encoding
 * ran out of room in word 1. */
#define FP_SRC0_ABS         (1u << 29)
#define FP_SRC12_ABS        (1u << 18)

/* Word 1 is not only source 0. Its upper half carries the instruction's
 * condition code and that code's swizzle, which every instruction has whether
 * or not it uses one. Leaving them zero encodes "condition false", so the
 * hardware discards the write and the shader silently computes nothing --
 * which is exactly the sort of wrong-but-valid encoding this file exists to
 * avoid, and it is what the cgcomp comparison caught. */
#define FP_COND_SHIFT       18
#define FP_COND_FL          0u      /* never */
#define FP_COND_TR          7u      /* always -- the default */
#define FP_COND_SWZ_X_SHIFT 21
#define FP_COND_SWZ_Y_SHIFT 23
#define FP_COND_SWZ_Z_SHIFT 25
#define FP_COND_SWZ_W_SHIFT 27

#define FP_COND_DEFAULT \
    ((FP_COND_TR << FP_COND_SHIFT) | \
     (0u << FP_COND_SWZ_X_SHIFT) | (1u << FP_COND_SWZ_Y_SHIFT) | \
     (2u << FP_COND_SWZ_Z_SHIFT) | (3u << FP_COND_SWZ_W_SHIFT))

/* Word 3 carries a texture-address field that must be set whenever *any*
 * source reads a texture coordinate -- not only on a texture fetch. It is a
 * fixed value for non-indexed access. */
#define FP_ADDR_INDEX_SHIFT 19
#define FP_ADDR_DIRECT      (0x7FCu << FP_ADDR_INDEX_SHIFT)

/* Fragment inputs, by their hardware index. */
#define FP_IN_WPOS   0
#define FP_IN_COL0   1
#define FP_IN_COL1   2
#define FP_IN_FOGC   3
#define FP_IN_TEX(n) (4 + (n))

typedef struct {
    u32 word;       /* logical, pre-swap */
    int is_const;   /* needs a following literal slot */
    float imm[4];
} FPSrc;

typedef struct {
    u32 *code;
    u32  capacity;  /* in 32-bit words */
    u32  used;      /* in 32-bit words */
    u32  last_insn; /* word index of the last instruction emitted */
    u32  num_regs;  /* highest temporary touched, + 1 */
    int  overflow;
} FPEmitter;

/* ------------------------------------------------------------------ */
/* Operand construction                                                 */
/* ------------------------------------------------------------------ */

DOL_INLINE u32 fp_swz(unsigned x, unsigned y, unsigned z, unsigned w)
{
    return (x << FP_REG_SWZ_X_SHIFT) | (y << FP_REG_SWZ_Y_SHIFT) |
           (z << FP_REG_SWZ_Z_SHIFT) | (w << FP_REG_SWZ_W_SHIFT);
}

#define FP_SWZ_XYZW  fp_swz(0, 1, 2, 3)
#define FP_SWZ_XXXX  fp_swz(0, 0, 0, 0)
#define FP_SWZ_WWWW  fp_swz(3, 3, 3, 3)

DOL_INLINE FPSrc fp_temp(unsigned index)
{
    FPSrc s;
    s.word = (FP_REG_TYPE_TEMP << FP_REG_TYPE_SHIFT) |
             (index << FP_REG_SRC_SHIFT) | FP_SWZ_XYZW;
    s.is_const = 0;
    s.imm[0] = s.imm[1] = s.imm[2] = s.imm[3] = 0.0f;
    return s;
}

/* An input's *index* does not live in the source word -- it goes in word 0,
 * which is why only one input can be read per instruction. The encoder puts it
 * there; this just marks the type. */
DOL_INLINE FPSrc fp_input(unsigned index)
{
    FPSrc s;
    s.word = (FP_REG_TYPE_INPUT << FP_REG_TYPE_SHIFT) | FP_SWZ_XYZW;
    s.is_const = 0;
    s.imm[0] = s.imm[1] = s.imm[2] = s.imm[3] = 0.0f;
    /* Stash the index in a spare high bit range for the emitter to lift out;
     * it is masked off before the word is written. */
    s.word |= (index & 0x1Fu) << 24;
    return s;
}

DOL_INLINE FPSrc fp_imm(float x, float y, float z, float w)
{
    FPSrc s;
    s.word = (FP_REG_TYPE_CONST << FP_REG_TYPE_SHIFT) | FP_SWZ_XYZW;
    s.is_const = 1;
    s.imm[0] = x; s.imm[1] = y; s.imm[2] = z; s.imm[3] = w;
    return s;
}

DOL_INLINE FPSrc fp_none(void)
{
    FPSrc s;
    s.word = (FP_REG_TYPE_TEMP << FP_REG_TYPE_SHIFT) | FP_SWZ_XYZW;
    s.is_const = 0;
    s.imm[0] = s.imm[1] = s.imm[2] = s.imm[3] = 0.0f;
    return s;
}

DOL_INLINE FPSrc fp_swizzle(FPSrc s, unsigned x, unsigned y, unsigned z, unsigned w)
{
    s.word &= ~(0xFFu << FP_REG_SWZ_X_SHIFT);
    s.word |= fp_swz(x, y, z, w);
    return s;
}

DOL_INLINE FPSrc fp_negate(FPSrc s) { s.word |= FP_REG_NEGATE; return s; }

/* ------------------------------------------------------------------ */
/* Emission                                                             */
/* ------------------------------------------------------------------ */

DOL_INLINE void fp_init(FPEmitter *e, u32 *buffer, u32 capacity_words)
{
    e->code = buffer;
    e->capacity = capacity_words;
    e->used = 0;
    e->last_insn = 0;
    e->num_regs = 1;
    e->overflow = 0;
}

/* The one place the half-swap happens. Everything above works in logical
 * words, so no caller can forget it. */
DOL_INLINE u32 fp_pack(u32 logical)
{
    return (logical << 16) | (logical >> 16);
}

DOL_INLINE void fp_emit_raw(FPEmitter *e, u32 w0, u32 w1, u32 w2, u32 w3)
{
    if (e->used + 4 > e->capacity) { e->overflow = 1; return; }
    e->last_insn = e->used;
    e->code[e->used++] = fp_pack(w0);
    e->code[e->used++] = fp_pack(w1);
    e->code[e->used++] = fp_pack(w2);
    e->code[e->used++] = fp_pack(w3);
}

/* A literal slot is four floats in the SAME half-swapped storage as the
 * instructions.
 *
 * This was originally written the other way ("it is data, not an
 * instruction") and that reasoning was wrong. The authority is nouveau's
 * driver for the same silicon: on a big-endian host it patches constants into
 * the instruction array and then applies the halfword swap to every word of
 * the upload, literals included -- the fetch path does not distinguish data
 * words from instruction words. cgcomp could never catch this because its
 * assembler silently refuses literal syntax, making this the one encoder path
 * with no reference to diff against. On hardware the raw-float version
 * produced saturated garbage where a constant colour should have been. */
DOL_INLINE void fp_emit_literal(FPEmitter *e, const float v[4])
{
    unsigned i;
    if (e->used + 4 > e->capacity) { e->overflow = 1; return; }
    for (i = 0; i < 4; i++) {
        u32 bits;
        float f = v[i];
        __builtin_memcpy(&bits, &f, sizeof bits);
        e->code[e->used++] = fp_pack(bits);
    }
}

DOL_INLINE void fp_emit(FPEmitter *e, unsigned op, unsigned dst, unsigned mask,
                        unsigned precision, int saturate,
                        FPSrc a, FPSrc b, FPSrc c)
{
    u32 w0, w1, w2, w3;
    const FPSrc *srcs[3];
    unsigned i;
    int have_const = 0;
    float lit[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    w0 = ((u32)op << FP_OPCODE_SHIFT) |
         ((u32)mask << FP_OUTMASK_SHIFT) |
         ((u32)precision << FP_PRECISION_SHIFT) |
         ((u32)dst << FP_OUT_REG_SHIFT);
    if (saturate)
        w0 |= FP_OUT_SAT;

    srcs[0] = &a; srcs[1] = &b; srcs[2] = &c;

    /* An input's index belongs to word 0, shared by all three operands: the
     * hardware can read exactly one input per instruction. */
    for (i = 0; i < 3; i++) {
        if ((srcs[i]->word & 3u) == FP_REG_TYPE_INPUT)
            w0 |= ((srcs[i]->word >> 24) & 0xFu) << FP_INPUT_SRC_SHIFT;
    }

    w1 = (a.word & 0x0003FFFFu) | FP_COND_DEFAULT;
    w2 = b.word & 0x0003FFFFu;
    w3 = c.word & 0x0003FFFFu;

    /* Reading a texture coordinate -- from any operand -- requires the address
     * field, because the coordinate is fetched through the texture address
     * unit rather than read as a plain varying. */
    for (i = 0; i < 3; i++) {
        if ((srcs[i]->word & 3u) == FP_REG_TYPE_INPUT &&
            ((srcs[i]->word >> 24) & 0x1Fu) >= (u32)FP_IN_TEX(0)) {
            w3 |= FP_ADDR_DIRECT;
            break;
        }
    }

    for (i = 0; i < 3; i++) {
        if (srcs[i]->is_const && !have_const) {
            have_const = 1;
            lit[0] = srcs[i]->imm[0]; lit[1] = srcs[i]->imm[1];
            lit[2] = srcs[i]->imm[2]; lit[3] = srcs[i]->imm[3];
        }
    }

    if (dst + 1 > e->num_regs)
        e->num_regs = dst + 1;

    fp_emit_raw(e, w0, w1, w2, w3);
    if (have_const)
        fp_emit_literal(e, lit);
}

/* A texture fetch names its unit in word 0 and its coordinate as source 0. */
DOL_INLINE void fp_emit_tex(FPEmitter *e, unsigned op, unsigned dst,
                            unsigned mask, unsigned precision, int saturate,
                            unsigned unit, FPSrc coord)
{
    u32 w0, w1;

    w0 = ((u32)op << FP_OPCODE_SHIFT) |
         ((u32)mask << FP_OUTMASK_SHIFT) |
         ((u32)precision << FP_PRECISION_SHIFT) |
         ((u32)dst << FP_OUT_REG_SHIFT) |
         ((u32)unit << FP_TEX_UNIT_SHIFT);
    if (saturate)
        w0 |= FP_OUT_SAT;
    /* Four bits, not five: bit 17 upwards is the texture unit. There are
     * thirteen inputs, so five bits was never needed and the extra one would
     * corrupt the unit number for any input index of 16 or more. */
    if ((coord.word & 3u) == FP_REG_TYPE_INPUT)
        w0 |= ((coord.word >> 24) & 0xFu) << FP_INPUT_SRC_SHIFT;

    w1 = (coord.word & 0x0003FFFFu) | FP_COND_DEFAULT;

    if (dst + 1 > e->num_regs)
        e->num_regs = dst + 1;

    fp_emit_raw(e, w0, w1, fp_none().word & 0x0003FFFFu,
                (fp_none().word & 0x0003FFFFu) | FP_ADDR_DIRECT);
}

/* Mark the program's last instruction. The end flag is a bit on that
 * instruction rather than an instruction of its own, which is why a program's
 * instruction count is one lower than the number of source lines written. */
DOL_INLINE void fp_finish(FPEmitter *e)
{
    if (e->used >= 4)
        e->code[e->last_insn] |= fp_pack(FP_PROGRAM_END);
}

#endif /* DOLPHIN_VIDEO_RSX_FP_EMITTER_H */
