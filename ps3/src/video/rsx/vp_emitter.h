/* vp_emitter.h — RSX vertex-program encoder.
 *
 * The counterpart to fp_emitter.h. Gekko's transform unit (XF) does position
 * and normal transforms, texture-coordinate generation and up to eight lights;
 * RSX does all of that in a programmable vertex pipeline, so the same
 * generate-and-cache approach applies. Unlike the fragment unit, the vertex
 * unit has no register-pressure cliff, so the generated code can be
 * straightforward.
 *
 * The *encoding*, however, is considerably nastier than the fragment one, in
 * three specific ways that are all invisible until something renders wrong:
 *
 * **Every instruction has two operation slots.** A vector unit and a scalar
 * unit share one four-word instruction, each with its own opcode, write mask
 * and destination. Emitting into one slot means explicitly marking the other's
 * destination as unused -- leaving it zero names temporary register 0 and
 * silently corrupts it.
 *
 * **Source operands straddle word boundaries.** A source descriptor is 17 bits;
 * source 0 is split across words 1 and 2, source 2 across words 2 and 3. The
 * split is not at a byte boundary and the two halves are shifted differently.
 *
 * **Words are plain big-endian**, unlike fragment programs, which store their
 * two 16-bit halves swapped. Applying the fragment rule here produces valid
 * instructions that do the wrong thing.
 *
 * Every encoding this header can produce is diffed against `cgcomp -a -v` by
 * `tools/vp_verify.sh`, for the same reason the fragment encoder is.
 */
#ifndef DOLPHIN_VIDEO_RSX_VP_EMITTER_H
#define DOLPHIN_VIDEO_RSX_VP_EMITTER_H

#include "../../common/types.h"

/* ------------------------------------------------------------------ */
/* Opcodes                                                              */
/*                                                                      */
/* The two units have separate opcode spaces, which is why they are named       */
/* apart: VEC_MOV and SCA_MOV are different numbers in different fields.        */
/* ------------------------------------------------------------------ */

#define VP_VEC_NOP   0x00
#define VP_VEC_MOV   0x01
#define VP_VEC_MUL   0x02
#define VP_VEC_ADD   0x03
#define VP_VEC_MAD   0x04
#define VP_VEC_DP3   0x05
#define VP_VEC_DPH   0x06
#define VP_VEC_DP4   0x07
#define VP_VEC_DST   0x08
#define VP_VEC_MIN   0x09
#define VP_VEC_MAX   0x0A
#define VP_VEC_SLT   0x0B
#define VP_VEC_SGE   0x0C
#define VP_VEC_ARL   0x0D
#define VP_VEC_FRC   0x0E
#define VP_VEC_FLR   0x0F
#define VP_VEC_SEQ   0x10
#define VP_VEC_SFL   0x11
#define VP_VEC_SGT   0x12
#define VP_VEC_SLE   0x13
#define VP_VEC_SNE   0x14
#define VP_VEC_STR   0x15
#define VP_VEC_SSG   0x16
#define VP_VEC_ARR   0x17
#define VP_VEC_ARA   0x18
#define VP_VEC_TXL   0x19

#define VP_SCA_NOP   0x00
#define VP_SCA_MOV   0x01
#define VP_SCA_RCP   0x02
#define VP_SCA_RCC   0x03
#define VP_SCA_RSQ   0x04
#define VP_SCA_EXP   0x05
#define VP_SCA_LOG   0x06
#define VP_SCA_LIT   0x07
#define VP_SCA_BRA   0x09
#define VP_SCA_RET   0x0D
#define VP_SCA_EX2   0x0E
#define VP_SCA_LG2   0x0F
#define VP_SCA_SIN   0x10
#define VP_SCA_COS   0x11

/* ------------------------------------------------------------------ */
/* Field positions                                                      */
/* ------------------------------------------------------------------ */

/* Word 0 */
#define VP_SATURATE            (1u << 26)
/* Set when the vector slot writes an output rather than a temporary. The
 * destination index alone does not say which -- both live in the same field --
 * so this bit is what distinguishes "output 1" from "temporary 1". */
#define VP_VEC_RESULT          (1u << 30)
#define VP_VEC_DEST_TEMP_SHIFT 15
#define VP_VEC_DEST_TEMP_MASK  (0x3Fu << 15)

/* Word 1 */
#define VP_VEC_OPCODE_SHIFT    22
#define VP_SCA_OPCODE_SHIFT    27
#define VP_CONST_SRC_SHIFT     12
#define VP_INPUT_SRC_SHIFT     8
#define VP_SRC0H_SHIFT         0

/* Word 2 */
#define VP_SRC0L_SHIFT         23
#define VP_SRC1_SHIFT          6
#define VP_SRC2H_SHIFT         0

/* Word 3 */
#define VP_SRC2L_SHIFT         21
#define VP_SCA_WRITEMASK_SHIFT 17
#define VP_VEC_WRITEMASK_SHIFT 13
#define VP_SCA_DEST_TEMP_SHIFT 7
#define VP_SCA_DEST_TEMP_MASK  (0x3Fu << 7)
#define VP_DEST_SHIFT          2
#define VP_DEST_MASK           (31u << 2)
/* Note this is *not* the same bit as the vector unit's result flag: the vector
 * one is bit 30 of word 0, this is bit 12 of word 3. Two flags meaning the same
 * thing for the two slots, in unrelated places. */
#define VP_SCA_RESULT          (1u << 12)
#define VP_LAST                (1u << 0)

/* Source descriptor, 17 bits, assembled before being split across words. */
#define VP_SRC_REG_TYPE_SHIFT  0
#define VP_SRC_REG_TYPE_TEMP   1u
#define VP_SRC_REG_TYPE_INPUT  2u
#define VP_SRC_REG_TYPE_CONST  3u
#define VP_SRC_TEMP_SRC_SHIFT  2
#define VP_SRC_SWZ_W_SHIFT     8
#define VP_SRC_SWZ_Z_SHIFT     10
#define VP_SRC_SWZ_Y_SHIFT     12
#define VP_SRC_SWZ_X_SHIFT     14
#define VP_SRC_NEGATE          (1u << 16)

/* The split. Source 0's low nine bits go to word 2, its high eight to word 1;
 * source 2's low eleven go to word 3 and its high six to word 2. */
#define VP_SRC0_HIGH_SHIFT     9
#define VP_SRC0_HIGH_MASK      0x0001FE00u
#define VP_SRC0_LOW_MASK       0x000001FFu
#define VP_SRC2_HIGH_SHIFT     11
#define VP_SRC2_HIGH_MASK      0x0001F800u
#define VP_SRC2_LOW_MASK       0x000007FFu

/* Condition code. Present on every instruction whether or not one is used, and
 * zero means "condition false" -- the same trap the fragment encoder has, with
 * the same consequence: the hardware discards the write and the instruction
 * silently does nothing. */
#define VP_COND_SHIFT        10
#define VP_COND_TR           7u
#define VP_COND_SWZ_X_SHIFT  8
#define VP_COND_SWZ_Y_SHIFT  6
#define VP_COND_SWZ_Z_SHIFT  4
#define VP_COND_SWZ_W_SHIFT  2

#define VP_COND_DEFAULT \
    ((VP_COND_TR << VP_COND_SHIFT) | \
     (0u << VP_COND_SWZ_X_SHIFT) | (1u << VP_COND_SWZ_Y_SHIFT) | \
     (2u << VP_COND_SWZ_Z_SHIFT) | (3u << VP_COND_SWZ_W_SHIFT))

/* Write mask, one bit per component. Note the bit order is the reverse of the
 * fragment unit's: x is the *high* bit here. */
#define VP_MASK_X  0x8u
#define VP_MASK_Y  0x4u
#define VP_MASK_Z  0x2u
#define VP_MASK_W  0x1u
#define VP_MASK_ALL (VP_MASK_X | VP_MASK_Y | VP_MASK_Z | VP_MASK_W)

/* Vertex inputs, by attribute index. */
#define VP_IN_POS     0
#define VP_IN_WEIGHT  1
#define VP_IN_NORMAL  2
#define VP_IN_COL0    3
#define VP_IN_COL1    4
#define VP_IN_FOGC    5
#define VP_IN_TEX(n)  (8 + (n))

/* Vertex outputs, as the *instruction's* destination index. */
#define VP_OUT_POS    0
#define VP_OUT_COL0   1
#define VP_OUT_COL1   2
#define VP_OUT_BFC0   3
#define VP_OUT_BFC1   4
#define VP_OUT_FOGC   5
#define VP_OUT_PSZ    6
#define VP_OUT_TEX(n) (7 + (n))

/* The result-enable mask uses a *different* numbering from the destination
 * index above, and the two are easy to conflate because both are small integers
 * naming the same outputs.
 *
 * The mask is handed straight to NV40TCL_VP_RESULT_EN, which decides which
 * interpolants are routed to the fragment stage. Writing `1 << dst` produces a
 * mask that is well-formed, accepted by the hardware, and enables the wrong
 * interpolant: a program writing COL0 (destination 1) would enable bit 1, which
 * is front *specular*, leaving diffuse unrouted. The fragment program then
 * reads a constant instead of the interpolated colour, and the triangle comes
 * out a flat, plausible colour with nothing anywhere reporting an error.
 *
 * Position is absent from the mask deliberately: it is always emitted.
 *
 * This is exactly the kind of mistake the cgcomp comparison cannot catch --
 * that check verifies *instruction encodings*, and this is a header field. */
DOL_INLINE u32 vp_result_bit(unsigned dst)
{
    switch (dst) {
    case VP_OUT_POS:  return 0;                 /* always emitted; no bit */
    /* Diffuse and specular each enable *two* bits: front and back. cgcomp
     * emits 0x5 for a program writing o[COL0] and 0xA for o[COL1], and
     * matching the reference matters -- enabling only the front bit leaves the
     * interpolant unrouted on the path the rasteriser actually takes, and the
     * fragment program reads a constant. Derived by compiling one-line
     * programs with cgcomp and reading the mask it produced, rather than from
     * the constant names, which suggest one bit each. */
    case VP_OUT_COL0: return (1u << 0) | (1u << 2);
    case VP_OUT_COL1: return (1u << 1) | (1u << 3);
    case VP_OUT_BFC0: return 1u << 2;
    case VP_OUT_BFC1: return 1u << 3;
    case VP_OUT_FOGC: return 1u << 4;
    case VP_OUT_PSZ:  return 1u << 5;
    default:
        /* TEX0..TEX7 are destinations 7..14 and mask bits 14..21. */
        if (dst >= VP_OUT_TEX(0) && dst <= VP_OUT_TEX(7))
            return 1u << (14 + (dst - VP_OUT_TEX(0)));
        return 0;
    }
}

/* Which hardware source slots an opcode reads.
 *
 * This is the detail that makes the vertex encoder genuinely different from the
 * fragment one. Operands are not assigned to slots 0, 1, 2 in order: ADD reads
 * slots **0 and 2**, while MUL reads 0 and 1, and the single-operand scalar
 * operations all read slot 2. Placing an ADD's second operand in slot 1
 * produces a well-formed instruction that adds whatever slot 2 happens to hold
 * -- which, for a freshly written program, is a plausible-looking zero.
 *
 * Returns the hardware slot for logical operand `n`, or -1 if the opcode does
 * not use it. */
DOL_INLINE int vp_vec_slot(unsigned op, unsigned n)
{
    switch (op) {
    case VP_VEC_MOV: case VP_VEC_FRC: case VP_VEC_FLR:
    case VP_VEC_ARL: case VP_VEC_ARR: case VP_VEC_ARA:
        return n == 0 ? 0 : -1;
    case VP_VEC_ADD:
        return n == 0 ? 0 : (n == 1 ? 2 : -1);
    case VP_VEC_MAD:
        return n < 3 ? (int)n : -1;
    default:
        /* MUL, DP3, DP4, DPH, DST, MIN, MAX and the comparisons. */
        return n < 2 ? (int)n : -1;
    }
}

typedef struct {
    u32 word;       /* the 17-bit descriptor, unsplit */
    int is_input;
    unsigned input_index;
    int is_const;
    unsigned const_index;
} VPSrc;

typedef struct {
    u32 *code;
    u32  capacity;
    u32  used;
    u32  last_insn;
    u32  num_regs;
    u32  input_mask;
    u32  output_mask;
    int  overflow;
} VPEmitter;

/* ------------------------------------------------------------------ */
/* Operands                                                             */
/* ------------------------------------------------------------------ */

DOL_INLINE u32 vp_swz(unsigned x, unsigned y, unsigned z, unsigned w)
{
    return (x << VP_SRC_SWZ_X_SHIFT) | (y << VP_SRC_SWZ_Y_SHIFT) |
           (z << VP_SRC_SWZ_Z_SHIFT) | (w << VP_SRC_SWZ_W_SHIFT);
}

#define VP_SWZ_XYZW  vp_swz(0, 1, 2, 3)

DOL_INLINE VPSrc vp_temp(unsigned index)
{
    VPSrc s;
    s.word = (VP_SRC_REG_TYPE_TEMP << VP_SRC_REG_TYPE_SHIFT) |
             (index << VP_SRC_TEMP_SRC_SHIFT) | VP_SWZ_XYZW;
    s.is_input = 0; s.input_index = 0;
    s.is_const = 0; s.const_index = 0;
    return s;
}

/* An input's index lives in word 1, not in the descriptor -- so an instruction
 * can read only one distinct input, exactly as the fragment unit can. */
DOL_INLINE VPSrc vp_input(unsigned index)
{
    VPSrc s;
    s.word = (VP_SRC_REG_TYPE_INPUT << VP_SRC_REG_TYPE_SHIFT) | VP_SWZ_XYZW;
    s.is_input = 1; s.input_index = index;
    s.is_const = 0; s.const_index = 0;
    return s;
}

/* Likewise a constant's index is in word 1: one constant per instruction. */
DOL_INLINE VPSrc vp_const(unsigned index)
{
    VPSrc s;
    s.word = (VP_SRC_REG_TYPE_CONST << VP_SRC_REG_TYPE_SHIFT) | VP_SWZ_XYZW;
    s.is_input = 0; s.input_index = 0;
    s.is_const = 1; s.const_index = index;
    return s;
}

/* An unused operand still has to name a type; zero is not a valid one. */
DOL_INLINE VPSrc vp_none(void)
{
    VPSrc s;
    s.word = (VP_SRC_REG_TYPE_INPUT << VP_SRC_REG_TYPE_SHIFT) | VP_SWZ_XYZW;
    s.is_input = 0; s.input_index = 0;
    s.is_const = 0; s.const_index = 0;
    return s;
}

DOL_INLINE VPSrc vp_swizzle(VPSrc s, unsigned x, unsigned y, unsigned z,
                            unsigned w)
{
    s.word &= ~(0xFFu << VP_SRC_SWZ_W_SHIFT);
    s.word |= vp_swz(x, y, z, w);
    return s;
}

DOL_INLINE VPSrc vp_negate(VPSrc s) { s.word |= VP_SRC_NEGATE; return s; }

/* ------------------------------------------------------------------ */
/* Emission                                                             */
/* ------------------------------------------------------------------ */

DOL_INLINE void vp_init(VPEmitter *e, u32 *buffer, u32 capacity_words)
{
    e->code = buffer;
    e->capacity = capacity_words;
    e->used = 0;
    e->last_insn = 0;
    e->num_regs = 1;
    e->input_mask = 0;
    e->output_mask = 0;
    e->overflow = 0;
}

/* Emit a vector-unit instruction.
 *
 * `dst_out` selects an output register when `to_output` is set, otherwise a
 * temporary. The scalar slot is left idle, which means its destination must be
 * explicitly marked unused -- the whole point of the two-slot encoding is that
 * both halves are always present. */
DOL_INLINE void vp_emit_vec(VPEmitter *e, unsigned op, unsigned dst,
                            int to_output, unsigned mask, int saturate,
                            VPSrc a, VPSrc b, VPSrc c)
{
    u32 w[4] = { 0, 0, 0, 0 };
    VPSrc logical[3];
    VPSrc slot[3];
    unsigned i;

    if (e->used + 4 > e->capacity) { e->overflow = 1; return; }

    logical[0] = a; logical[1] = b; logical[2] = c;
    slot[0] = vp_none(); slot[1] = vp_none(); slot[2] = vp_none();

    /* Place each operand in the slot this opcode reads it from. */
    for (i = 0; i < 3; i++) {
        int hw_slot = vp_vec_slot(op, i);
        if (hw_slot >= 0)
            slot[hw_slot] = logical[i];
    }

    w[0] |= VP_COND_DEFAULT;
    w[1] |= (u32)op << VP_VEC_OPCODE_SHIFT;
    w[3] |= (u32)mask << VP_VEC_WRITEMASK_SHIFT;
    if (saturate)
        w[0] |= VP_SATURATE;

    /* The idle scalar slot. */
    w[3] |= VP_SCA_DEST_TEMP_MASK;

    if (to_output) {
        w[3] |= ((u32)dst << VP_DEST_SHIFT) & VP_DEST_MASK;
        w[0] |= VP_VEC_RESULT;              /* the destination is an output */
        w[0] |= VP_VEC_DEST_TEMP_MASK;      /* and not a temporary          */
        e->output_mask |= vp_result_bit(dst);
    } else {
        w[3] |= VP_DEST_MASK;               /* not an output   */
        w[0] |= (u32)dst << VP_VEC_DEST_TEMP_SHIFT;
        if (dst + 1 > e->num_regs)
            e->num_regs = dst + 1;
    }

    for (i = 0; i < 3; i++) {
        if (slot[i].is_input) {
            w[1] |= slot[i].input_index << VP_INPUT_SRC_SHIFT;
            e->input_mask |= 1u << slot[i].input_index;
        }
        if (slot[i].is_const)
            w[1] |= slot[i].const_index << VP_CONST_SRC_SHIFT;
    }

    /* The split. Assembling each descriptor whole and then cutting it is the
     * only formulation where the boundary is visible; folding the shifts into
     * the operand construction hides a 17-bit field inside two expressions
     * that look unrelated. */
    w[1] |= ((slot[0].word & VP_SRC0_HIGH_MASK) >> VP_SRC0_HIGH_SHIFT) << VP_SRC0H_SHIFT;
    w[2] |= (slot[0].word & VP_SRC0_LOW_MASK) << VP_SRC0L_SHIFT;
    w[2] |= slot[1].word << VP_SRC1_SHIFT;
    w[2] |= ((slot[2].word & VP_SRC2_HIGH_MASK) >> VP_SRC2_HIGH_SHIFT) << VP_SRC2H_SHIFT;
    w[3] |= (slot[2].word & VP_SRC2_LOW_MASK) << VP_SRC2L_SHIFT;

    e->last_insn = e->used;
    e->code[e->used++] = w[0];
    e->code[e->used++] = w[1];
    e->code[e->used++] = w[2];
    e->code[e->used++] = w[3];
}

/* Emit a scalar-unit instruction. Mirrors the above with the slots reversed. */
DOL_INLINE void vp_emit_sca(VPEmitter *e, unsigned op, unsigned dst,
                            int to_output, unsigned mask, int saturate,
                            VPSrc a)
{
    u32 w[4] = { 0, 0, 0, 0 };

    if (e->used + 4 > e->capacity) { e->overflow = 1; return; }

    w[0] |= VP_COND_DEFAULT;
    w[1] |= (u32)op << VP_SCA_OPCODE_SHIFT;
    w[3] |= (u32)mask << VP_SCA_WRITEMASK_SHIFT;
    if (saturate)
        w[0] |= VP_SATURATE;

    /* The idle vector slot. */
    w[0] |= VP_VEC_DEST_TEMP_MASK;

    if (to_output) {
        w[3] |= ((u32)dst << VP_DEST_SHIFT) & VP_DEST_MASK;
        w[3] |= VP_SCA_RESULT;
        w[3] |= VP_SCA_DEST_TEMP_MASK;
        e->output_mask |= vp_result_bit(dst);
    } else {
        w[3] |= VP_DEST_MASK;
        w[3] |= (u32)dst << VP_SCA_DEST_TEMP_SHIFT;
        if (dst + 1 > e->num_regs)
            e->num_regs = dst + 1;
    }

    if (a.is_input) {
        w[1] |= a.input_index << VP_INPUT_SRC_SHIFT;
        e->input_mask |= 1u << a.input_index;
    }
    if (a.is_const)
        w[1] |= a.const_index << VP_CONST_SRC_SHIFT;

    /* A scalar operation reads source 2, not source 0. */
    {
        VPSrc none = vp_none();
        w[1] |= ((none.word & VP_SRC0_HIGH_MASK) >> VP_SRC0_HIGH_SHIFT) << VP_SRC0H_SHIFT;
        w[2] |= (none.word & VP_SRC0_LOW_MASK) << VP_SRC0L_SHIFT;
        w[2] |= none.word << VP_SRC1_SHIFT;
        w[2] |= ((a.word & VP_SRC2_HIGH_MASK) >> VP_SRC2_HIGH_SHIFT) << VP_SRC2H_SHIFT;
        w[3] |= (a.word & VP_SRC2_LOW_MASK) << VP_SRC2L_SHIFT;
    }

    e->last_insn = e->used;
    e->code[e->used++] = w[0];
    e->code[e->used++] = w[1];
    e->code[e->used++] = w[2];
    e->code[e->used++] = w[3];
}

DOL_INLINE void vp_finish(VPEmitter *e)
{
    if (e->used >= 4)
        e->code[e->last_insn + 3] |= VP_LAST;
}

#endif /* DOLPHIN_VIDEO_RSX_VP_EMITTER_H */
