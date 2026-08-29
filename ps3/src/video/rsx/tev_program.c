/* tev_program.c — lowering TEV stages to RSX fragment microcode.
 *
 * See tev_program.h for why this generates rather than interprets.
 *
 * The lowering is deliberately built from instructions the encoder has been
 * verified on (MOV, MUL, ADD, MAD, TEX), rather than the shortest possible
 * sequence. NV40 has an LRP that would collapse the interpolation to one
 * instruction, but cgcomp expands LRP into two MADs rather than emitting it,
 * which means there is no reference encoding to check ours against -- and an
 * unverified instruction in the middle of every TEV stage is exactly the kind
 * of thing that produces a subtly wrong picture with no way to localise it.
 * Two verified instructions beat one unverified one until an oracle exists.
 */
#include "tev_program.h"
#include "gx_features.h"

#include <string.h>

#define BITS(v, lsb, width) (((u32)(v) >> (lsb)) & ((1u << (width)) - 1u))

/* ------------------------------------------------------------------ */
/* Operand sourcing                                                     */
/*                                                                      */
/* Every TEV source resolves to a register plus a swizzle. Alpha sources are    */
/* the .w of the same registers, which is what lets the colour and alpha        */
/* combiners share one lowering.                                                */
/* ------------------------------------------------------------------ */

/* The rasterised-colour selector is not an index into the colour channels.
 * Channels 0 and 1 are the two lit channels; 5 and 6 select the indirect
 * unit's "alpha bump"; 7 selects a constant zero, which is what a title writes
 * when a stage's formula has no use for a vertex colour at all.
 *
 * This used to be `FP_IN_COL0 + (channel & 1)`, which sends channel 7 to COL1
 * -- an interpolant the vertex program does not write unless the title has
 * configured two colour channels, so the stage read whatever the rasteriser
 * had left in it. On the Mario Kart Wii title screen 1221 stages select
 * channel 7, all of them in the full-screen overlay pass; they were being
 * combined with an undefined colour. */
static FPSrc ras_source(unsigned channel)
{
    switch (channel & 7u) {
    case 0:  return fp_input(FP_IN_COL0);
    case 1:  return fp_input(FP_IN_COL1);
    default: return fp_imm(0.0f, 0.0f, 0.0f, 0.0f);
    }
}

/* ------------------------------------------------------------------ */
/* Konst                                                                */
/*                                                                      */
/* A stage's constant is not one of the four TEV registers: it is whatever the  */
/* KSEL register selects, which is either one of eight fixed fractions, one of  */
/* the four konst *colours* (a different bank from the four TEV registers), or  */
/* a single channel of one of them broadcast to all four components.            */
/*                                                                              */
/* Until now TEV_CC_KONST resolved to TEV_REG_C2 -- the third ordinary register, */
/* which is neither the right bank nor the right selection. That is a colour     */
/* that is plausible rather than absent, so nothing about the picture says       */
/* "konst": over the 396 course and kart materials the game defines, 173     */
/* alpha operands and 51 colour operands select a konst, and all 224 of them     */
/* were reading whatever the last stage happened to leave in C2.                 */
/* ------------------------------------------------------------------ */

/* The eight fixed fractions, in KSEL order. */
static float konst_fraction(unsigned sel)
{
    static const float k[8] = {
        1.0f, 7.0f/8.0f, 3.0f/4.0f, 5.0f/8.0f,
        0.5f, 3.0f/8.0f, 1.0f/4.0f, 1.0f/8.0f
    };
    return k[sel & 7u];
}

/* Resolve a konst selection to the literal the generated program will carry.
 * `alpha` picks the alpha combiner's encoding, which differs from the colour
 * one in exactly one place: selections 8..15 are invalid for it (the four
 * whole-colour forms 12..15 are colour-only) and read as zero. */
static void konst_value(const BPState *bp, unsigned sel, int alpha, float out[4])
{
    unsigned k;
    out[0] = out[1] = out[2] = out[3] = 0.0f;
    sel &= 31u;

    if (sel < 8) {
        float v = konst_fraction(sel);
        out[0] = out[1] = out[2] = out[3] = v;
        return;
    }
    if (sel < 12)
        return;                         /* invalid in both encodings: zero */
    if (sel < 16) {
        if (alpha)
            return;                     /* whole-colour forms are colour only */
        k = sel - 12u;
        out[0] = bp->tev_konst[k][0];
        out[1] = bp->tev_konst[k][1];
        out[2] = bp->tev_konst[k][2];
        out[3] = bp->tev_konst[k][3];
        return;
    }
    /* 16..31: konst k's r, g, b or a broadcast to every component. */
    k = (sel - 16u) & 3u;
    {
        float v = bp->tev_konst[k][(sel - 16u) >> 2];
        out[0] = out[1] = out[2] = out[3] = v;
    }
}

/* ------------------------------------------------------------------ */
/* Swap tables                                                          */
/*                                                                      */
/* Each stage names one swap table for the rasterised colour and one for the    */
/* texture sample; a table says which input channel each output channel comes   */
/* from. It costs nothing to honour -- a source operand already carries a        */
/* swizzle -- and getting it wrong is a channel permutation, which reads as a    */
/* wrong colour rather than as missing geometry.                                 */
/*                                                                              */
/* Measured: rswap and tswap are 0 on all 612 TEV stages in the six courses and  */
/* two karts read off the disc, and table 0 is the identity in all of them, so   */
/* this is presently a no-op on the data that matters. It is implemented anyway  */
/* because it is three lines and because a table that is silently ignored is the */
/* kind of thing that is discovered from a screenshot months later.              */
/* ------------------------------------------------------------------ */

static FPSrc swap_rgb(FPSrc base, const unsigned sw[4])
{
    return fp_swizzle(base, sw[0], sw[1], sw[2], sw[3]);
}

static FPSrc swap_alpha(FPSrc base, const unsigned sw[4])
{
    return fp_swizzle(base, sw[3], sw[3], sw[3], sw[3]);
}

/* One stage's operand sources, resolved once and passed to both combiners.
 * Building them here rather than inside the switch is what makes the swap
 * tables and the konst selection free: they are properties of the stage, not
 * of the individual operand. */
typedef struct {
    FPSrc ras_rgb, ras_a;   /* rasterised colour, already swapped */
    FPSrc tex_rgb, tex_a;   /* texture sample, already swapped    */
    FPSrc konst_rgb;        /* the colour combiner's constant     */
    FPSrc konst_a;          /* the alpha combiner's constant      */
} TevStageSrc;

static FPSrc color_arg(unsigned arg, const TevStageSrc *src)
{
    switch (arg) {
    case TEV_CC_PREV_RGB: return fp_temp(TEV_REG_PREV);
    case TEV_CC_PREV_A:   return fp_swizzle(fp_temp(TEV_REG_PREV), 3, 3, 3, 3);
    case TEV_CC_C0_RGB:   return fp_temp(TEV_REG_C0);
    case TEV_CC_C0_A:     return fp_swizzle(fp_temp(TEV_REG_C0), 3, 3, 3, 3);
    case TEV_CC_C1_RGB:   return fp_temp(TEV_REG_C1);
    case TEV_CC_C1_A:     return fp_swizzle(fp_temp(TEV_REG_C1), 3, 3, 3, 3);
    case TEV_CC_C2_RGB:   return fp_temp(TEV_REG_C2);
    case TEV_CC_C2_A:     return fp_swizzle(fp_temp(TEV_REG_C2), 3, 3, 3, 3);
    case TEV_CC_TEX_RGB:  return src->tex_rgb;
    case TEV_CC_TEX_A:    return src->tex_a;
    case TEV_CC_RAS_RGB:  return src->ras_rgb;
    case TEV_CC_RAS_A:    return src->ras_a;
    case TEV_CC_ONE:      return fp_imm(1.0f, 1.0f, 1.0f, 1.0f);
    case TEV_CC_HALF:     return fp_imm(0.5f, 0.5f, 0.5f, 0.5f);
    case TEV_CC_KONST:    return src->konst_rgb;
    case TEV_CC_ZERO:
    default:              return fp_imm(0.0f, 0.0f, 0.0f, 0.0f);
    }
}

static FPSrc alpha_arg(unsigned arg, const TevStageSrc *src)
{
    switch (arg) {
    case TEV_AC_PREV:  return fp_swizzle(fp_temp(TEV_REG_PREV), 3, 3, 3, 3);
    case TEV_AC_C0:    return fp_swizzle(fp_temp(TEV_REG_C0), 3, 3, 3, 3);
    case TEV_AC_C1:    return fp_swizzle(fp_temp(TEV_REG_C1), 3, 3, 3, 3);
    case TEV_AC_C2:    return fp_swizzle(fp_temp(TEV_REG_C2), 3, 3, 3, 3);
    case TEV_AC_TEX:   return src->tex_a;
    case TEV_AC_RAS:   return src->ras_a;
    case TEV_AC_KONST: return src->konst_a;
    case TEV_AC_ZERO:
    default:           return fp_imm(0.0f, 0.0f, 0.0f, 0.0f);
    }
}

static unsigned dest_reg(unsigned dest)
{
    switch (dest) {
    case TEV_DEST_C0: return TEV_REG_C0;
    case TEV_DEST_C1: return TEV_REG_C1;
    case TEV_DEST_C2: return TEV_REG_C2;
    case TEV_DEST_PREV:
    default:          return TEV_REG_PREV;
    }
}

static float bias_value(unsigned bias)
{
    switch (bias) {
    case TEV_BIAS_PLUS_HALF:  return  0.5f;
    case TEV_BIAS_MINUS_HALF: return -0.5f;
    default:                  return  0.0f;
    }
}

static float scale_value(unsigned scale)
{
    switch (scale) {
    case TEV_SCALE_2:    return 2.0f;
    case TEV_SCALE_4:    return 4.0f;
    case TEV_SCALE_HALF: return 0.5f;
    default:             return 1.0f;
    }
}

/* ------------------------------------------------------------------ */
/* One combiner                                                         */
/*                                                                      */
/* out = ((d +/- lerp(a, b, c)) + bias) * scale, written as:                    */
/*                                                                             */
/*   ADD  tmp0 = b - a                                                         */
/*   MAD  tmp1 = c * tmp0 + a          ; lerp(a, b, c)                         */
/*   ADD  tmp1 = d +/- tmp1                                                    */
/*   ADD  tmp1 = tmp1 + bias           ; only when bias is not zero            */
/*   MUL  dst  = tmp1 * scale          ; only when scale is not one            */
/*                                                                             */
/* Bias and scale are separate instructions rather than one MAD because an      */
/* instruction carries a single literal slot and that would need two. Whichever */
/* instruction ends up last takes the saturate flag, so clamping is free rather */
/* than a min/max pair. A stage at the defaults -- which is most of them --     */
/* costs three instructions.                                                    */
/* ------------------------------------------------------------------ */

/* Emit one instruction, hoisting constants until at most one remains.
 *
 * The hardware gives an instruction a single literal slot. Passing two
 * different immediates does not fail -- both operands read whichever literal
 * was emitted, so the stage computes a wrong value with no diagnostic
 * anywhere. This is the wrapper that makes that unrepresentable. */
static void emit_op(FPEmitter *e, unsigned op, unsigned dst, unsigned mask,
                    unsigned prec, int sat, FPSrc a, FPSrc b, FPSrc c)
{
    FPSrc *ops[3];
    unsigned spill[2] = { TEV_REG_IMM0, TEV_REG_IMM1 };
    unsigned spilled = 0;
    unsigned seen = 0;
    unsigned i;

    ops[0] = &a; ops[1] = &b; ops[2] = &c;

    for (i = 0; i < 3; i++) {
        if (!ops[i]->is_const)
            continue;
        if (seen == 0) { seen = 1; continue; }   /* the one we can keep */
        if (spilled < 2) {
            fp_emit(e, FP_OP_MOV, spill[spilled], FP_MASK_ALL, prec, 0,
                    *ops[i], fp_none(), fp_none());
            *ops[i] = fp_temp(spill[spilled]);
            spilled++;
        }
    }

    fp_emit(e, op, dst, mask, prec, sat, a, b, c);
}

/* "Unclamped" is a misnomer: the TEV output register is SIGNED 11 BITS, so
 * hardware clamps to -1024..1023 either way. Normalised that is
 * -1024/255 = -4.0157 .. 1023/255 = +4.0117. Letting an unclamped chain run
 * free in fp16 does not merely exceed the range, it loses mantissa precision
 * as the exponent grows, so long unclamped stage chains drift. Two extra
 * instructions, and only on stages that actually ask for no clamp. */
static void tev_clamp_unclamped(FPEmitter *e, unsigned dst, unsigned mask,
                                int clamp)
{
    if (clamp)
        return;                     /* saturate already did 0..1 */
    emit_op(e, FP_OP_MAX, dst, mask, FP_PREC_FP16, 0,
            fp_temp(dst), fp_imm(-4.0156863f, -4.0156863f,
                                 -4.0156863f, -4.0156863f), fp_none());
    emit_op(e, FP_OP_MIN, dst, mask, FP_PREC_FP16, 0,
            fp_temp(dst), fp_imm(4.0117648f, 4.0117648f,
                                 4.0117648f, 4.0117648f), fp_none());
}

/* TEV COMPARE MODE.
 *
 * When bias == 3 the stage is not an arithmetic combine at all, and the `op`
 * and `scale` fields CHANGE MEANING (BPMemory.h:453-459):
 *     bit 18     op    -> comparison   (0 = GT, 1 = EQ)
 *     bits 20-21 scale -> compare_mode (0 = R8, 1 = GR16, 2 = BGR24, 3 = RGB8/A8)
 * Decoding them as op/scale computes d +/- lerp(a,b,c) scaled by 1/2/4/0.5 --
 * plausible-looking garbage rather than an obvious failure, which is why this
 * survived every test until the register semantics were checked against
 * hardware. The correct result is:
 *     dest = d + (compare(a,b) ? c : 0)
 * R8/GR16/BGR24 make ONE scalar decision broadcast to rgb; RGB8 (and A8)
 * compare PER COMPONENT. That asymmetry is real hardware behaviour.
 *
 * GR16 and BGR24 form dot(rgb, (1,256,65536)), which reaches 16.7M: exactly
 * representable in fp32's 24-bit mantissa and NOT in fp16's 11-bit one, so
 * those two widths are forced to fp32. */
static void emit_compare(FPEmitter *e, unsigned dst, unsigned mask,
                         FPSrc a, FPSrc b, FPSrc c, FPSrc d,
                         unsigned cmp_mode, int eq, int clamp)
{
    /* GR16/BGR24 form dot(rgb,(1,256,65536)) which reaches 16.7M: exact in
     * fp32's 24-bit mantissa, NOT in fp16's 11-bit one. */
    unsigned prec = (cmp_mode == 1u || cmp_mode == 2u) ? FP_PREC_FP32
                                                       : FP_PREC_FP16;
    unsigned cmpop = eq ? FP_OP_SEQ : FP_OP_SGT;

    if (cmp_mode == 3u) {
        /* RGB8 (and A8 on the alpha combiner): PER-COMPONENT decision. */
        emit_op(e, cmpop, TEV_REG_TMP0, mask, prec, 0, a, b, fp_none());
    } else if (cmp_mode == 0u) {
        /* R8: one decision from the red channel, broadcast. */
        emit_op(e, cmpop, TEV_REG_TMP0, FP_MASK_ALL, prec, 0,
                fp_swizzle(a, 0, 0, 0, 0), fp_swizzle(b, 0, 0, 0, 0),
                fp_none());
    } else {
        /* GR16 / BGR24: one decision from a weighted sum, broadcast. The
         * channels are 0..1 here, so the hardware weights (1,256,65536) are
         * pre-multiplied by 255 to put the comparison in integer units. */
        static const float w16[4] = { 255.0f, 255.0f*256.0f, 0.0f, 0.0f };
        static const float w24[4] = { 255.0f, 255.0f*256.0f, 255.0f*65536.0f,
                                      0.0f };
        const float *w = (cmp_mode == 2u) ? w24 : w16;
        emit_op(e, FP_OP_DP3, TEV_REG_TMP0, FP_MASK_ALL, prec, 0,
                a, fp_imm(w[0], w[1], w[2], w[3]), fp_none());
        emit_op(e, FP_OP_DP3, TEV_REG_TMP1, FP_MASK_ALL, prec, 0,
                b, fp_imm(w[0], w[1], w[2], w[3]), fp_none());
        emit_op(e, cmpop, TEV_REG_TMP0, FP_MASK_ALL, prec, 0,
                fp_temp(TEV_REG_TMP0), fp_temp(TEV_REG_TMP1), fp_none());
    }
    /* dest = d + (decision ? c : 0) */
    emit_op(e, FP_OP_MAD, dst, mask, prec, clamp,
            fp_temp(TEV_REG_TMP0), c, d);
    tev_clamp_unclamped(e, dst, mask, clamp);
}

static void emit_combiner(FPEmitter *e, unsigned dst, unsigned mask,
                          FPSrc a, FPSrc b, FPSrc c, FPSrc d,
                          unsigned bias, unsigned scale,
                          int subtract, int clamp)
{
    float k_scale, k_bias;

    if (bias == TEV_BIAS_COMPARE) {
        /* `subtract` carries bit 18 (comparison) and `scale` bits 20-21
         * (compare_mode) in this encoding -- see emit_compare. */
        emit_compare(e, dst, mask, a, b, c, d, scale, subtract, clamp);
        return;
    }
    k_scale = scale_value(scale);
    k_bias  = bias_value(bias);

    /* b - a */
    emit_op(e, FP_OP_ADD, TEV_REG_TMP0, mask, FP_PREC_FP16, 0,
            b, fp_negate(a), fp_none());

    /* a + c * (b - a) */
    emit_op(e, FP_OP_MAD, TEV_REG_TMP1, mask, FP_PREC_FP16, 0,
            c, fp_temp(TEV_REG_TMP0), a);

    /* d +/- lerp. Whether this is the final instruction -- and so whether it
     * carries the saturate flag -- depends on there being no bias or scale
     * left to apply. */
    {
        int last = (k_bias == 0.0f && k_scale == 1.0f);
        emit_op(e, FP_OP_ADD, last ? dst : TEV_REG_TMP1, mask, FP_PREC_FP16,
                last ? clamp : 0,
                d, subtract ? fp_negate(fp_temp(TEV_REG_TMP1))
                            : fp_temp(TEV_REG_TMP1),
                fp_none());
        if (last) {
            tev_clamp_unclamped(e, dst, mask, clamp);
            return;
        }
    }

    /* Bias is applied *before* scale: TEV computes ((d +/- lerp) + bias) *
     * scale, not the other way round. Getting that order wrong is invisible
     * whenever either is at its default, which is most of the time -- so it
     * survives casual testing and then quietly darkens or blows out exactly
     * the stages that use both. */
    if (k_bias != 0.0f) {
        int last = (k_scale == 1.0f);
        emit_op(e, FP_OP_ADD, last ? dst : TEV_REG_TMP1, mask, FP_PREC_FP16,
                last ? clamp : 0,
                fp_temp(TEV_REG_TMP1),
                fp_imm(k_bias, k_bias, k_bias, k_bias), fp_none());
        if (last) {
            tev_clamp_unclamped(e, dst, mask, clamp);
            return;
        }
    }

    emit_op(e, FP_OP_MUL, dst, mask, FP_PREC_FP16, clamp,
            fp_temp(TEV_REG_TMP1),
            fp_imm(k_scale, k_scale, k_scale, k_scale), fp_none());
    tev_clamp_unclamped(e, dst, mask, clamp);
}

/* ------------------------------------------------------------------ */
/* Indirect texturing                                                   */
/*                                                                      */
/* A second, smaller texture pipeline that runs BEFORE the ordinary one: it     */
/* samples a texture, reads the texel as three signed numbers, transforms them  */
/* by a 2x3 matrix and adds the result to the coordinate the stage was about to */
/* sample with. That is how a race gets its heat haze, its water, its road      */
/* distortion and its shadow lookups -- 1,813,066 of the 9,482,453 draws in a   */
/* full race (19.1%) configure one, and until now the generator ignored every   */
/* one of them and produced an undistorted picture.                             */
/*                                                                              */
/* THE ARITHMETIC. The hardware works in a fixed-point coordinate space of      */
/* 1/128 texel, which is where the constants below come from. Dolphin's         */
/* pixel-shader generator writes it as integers; this generator has floats and  */
/* no integer ops at all, so the same chain is folded into per-program          */
/* constants instead:                                                           */
/*                                                                              */
/*   offset_fixed = ((M . c) >> 3) >> (17 - scale)      the hardware            */
/*   offset_texel = offset_fixed / 128                                          */
/*   offset_uv    = offset_texel / texture_size                                 */
/*                                                                              */
/* and with M stored as an 11-bit value with ten fractional bits (so the        */
/* register holds 1024*m), the three shifts collapse into one factor:           */
/*                                                                              */
/*   offset_uv = (m . c) * 2^(scale - 17) / texture_size                        */
/*                                                                              */
/* where c is the texel's ALPHA, BLUE and GREEN channels -- not red, green,     */
/* blue, which is the single easiest thing here to get wrong and produces an    */
/* offset of the right shape from the wrong data -- as integers 0..255, shifted */
/* down by the format and then biased.                                          */
/*                                                                              */
/* WHAT MARIO KART WII ACTUALLY ASKS FOR, measured over a complete race         */
/* (332,505 TEV stages carrying a non-zero indirect command):                   */
/*                                                                              */
/*   format         ITF_8 on all 332,505                                        */
/*   bias           STU on 332,493, none on 12                                  */
/*   bump alpha     off on all of them; no stage selects rasterised channel     */
/*                  5 or 6 anywhere in the race                                 */
/*   matrix id      the ordinary matrix on all of them; neither dynamic form    */
/*   matrix index   1 on 309,323, 2 on 11,591, 3 on 11,591                      */
/*   wrap           off on 297,732, "replace with zero" on 34,773; the five     */
/*                  modulo settings never appear                                */
/*   coord scale    zero on every indirect stage                                */
/*   add previous   12 stages (four draws)                                      */
/*                                                                              */
/* So the paths that matter are short, and the rest is implemented because it   */
/* is a handful of instructions each and a silently ignored field is the kind   */
/* of thing that is found in a screenshot months later. The unit tests in       */
/* tests/test_tev.c exercise all of it, exercised-by-the-game or not.           */
/*                                                                              */
/* WHAT IS NOT IMPLEMENTED, deliberately and with the reason:                   */
/*                                                                              */
/*  - `lb_utclod` selects whether the LOD is computed from the modified or the  */
/*    unmodified coordinate. This backend does not compute an LOD at all (there */
/*    are no mipmaps yet), so the bit has nothing to change. No stage in the    */
/*    race sets it.                                                             */
/*  - The hardware truncates the accumulated coordinate to a signed 24-bit      */
/*    fixed-point value, i.e. it wraps at +/-65536 texels. Reproducing that in  */
/*    floats costs two instructions per stage to emulate an overflow no draw    */
/*    can reach: a coordinate that large has already left the texture.          */
/*  - The coordinate's fixed-point scale is taken to be the size of the texture */
/*    being sampled. The hardware has a separate per-coordinate scale register  */
/*    (SU_SSIZE/SU_TSIZE, BP 0x30..0x3F) and GX sets it to the texture size,    */
/*    which is the only case that matters; the two are only distinguishable by  */
/*    a title that sets them apart on purpose.                                  */
/*  - The third and fourth components of the computed coordinate are not        */
/*    maintained. Every fetch this backend issues is 2D and reads s and t only, */
/*    which is already true of the plain path (it hands the fetch an            */
/*    interpolant whose third component is the texgen's q).                     */
/*  - The whole unit is gated on the guest's indirect STAGE COUNT rather than   */
/*    on the per-stage command, which is where this differs from Dolphin: with  */
/*    the count at zero Dolphin still applies a stale command's wrapping and    */
/*    its add-previous, and this does not. Measured over a complete race that   */
/*    is 19,758 draws of 9,482,453 -- draws where the guest has told the        */
/*    hardware the indirect unit is off but has not written GXSetTevDirect over */
/*    the command. Following the count is the conservative reading and keeps    */
/*    those draws generating exactly the code they generated before.            */
/* ------------------------------------------------------------------ */

/* Powers of two without <math.h>: this file has no other need for libm and the
 * exponents involved are small and bounded (the scale field is five bits). */
static float tev_pow2(int e)
{
    float v = 1.0f;
    for (; e > 0; e--) v *= 2.0f;
    for (; e < 0; e++) v *= 0.5f;
    return v;
}

/* How far the texel is shifted down before it is used as a coordinate: ITF_8
 * uses the whole byte, the narrow formats keep the top bits and leave the rest
 * of the byte for the alpha bump. */
static unsigned ind_fmt_shift(unsigned fmt)
{
    static const unsigned k[4] = { 0u, 3u, 4u, 5u };
    return k[fmt & 3u];
}

/* The bias field selects WHICH axes are offset, not by how much: ITF_8 is
 * signed about the middle of the byte and biases by -128, the narrow formats
 * bias by +1. */
static float ind_bias_add(unsigned fmt)
{
    return (fmt & 3u) == 0u ? -128.0f : 1.0f;
}

/* What a wrap selector does to one axis of the regular coordinate. Returns the
 * modulus in TEXELS; 0 means "leave the axis alone" and a negative result means
 * "replace it with zero" (selectors 6 and 7, the second of which is not a legal
 * value but behaves like the first). */
static float ind_wrap_modulus(unsigned w)
{
    static const float k[6] = { 0.0f, 256.0f, 128.0f, 64.0f, 32.0f, 16.0f };
    return (w >= 6u) ? -1.0f : k[w];
}

/* What the whole program needs from the indirect unit, decided once so a
 * material that does not use it emits nothing at all. */
typedef struct {
    int      live;      /* the coordinate pipeline is emitted at all         */
    int      persist;   /* some stage adds the previous stage's coordinate   */
    unsigned lookups;   /* bit per indirect lookup that is actually fetched  */
} TevIndPlan;

/* A stage offsets its coordinate when it names a matrix AND the lookup it
 * names exists. Referencing a lookup above the configured count is undefined
 * on hardware (it produces a noise pattern); skipping the offset is the
 * closest defined behaviour and is what Dolphin settled on. */
static int ind_stage_offsets(const BPState *bp, unsigned n)
{
    const BPTevIndirect *ti = &bp->tevind[n];
    return ti->matrix_index != 0u &&
           ti->ind_stage < bp->genmode.num_indstages;
}

/* The alpha bump is only observable through a stage that selects rasterised
 * channel 5 or 6, so it is generated only then -- three instructions saved on
 * every stage that configures one and reads a real colour instead. */
static int ind_stage_bumps(const BPState *bp, unsigned n)
{
    const BPTevIndirect *ti = &bp->tevind[n];
    unsigned ras = bp->tev[n].ras_channel & 7u;
    return ti->bump_alpha != 0u &&
           ti->ind_stage < bp->genmode.num_indstages &&
           (ras == 5u || ras == 6u);
}

static void ind_plan(const BPState *bp, TevIndPlan *p)
{
    unsigned stages = bp_tev_stage_count(bp), i;

    p->live = 0; p->persist = 0; p->lookups = 0;
    if (!(g_gx_state_mask & GX_STATE_INDIRECT))
        return;
    /* GXSetNumIndStages(0) does not clear the per-stage commands, so a
     * material that has switched the unit off can still have a non-zero
     * command left in a register. The stage count is the guest's own "the
     * indirect unit is in use" switch and is what the audit counts, so it
     * gates the whole thing here too. */
    if (bp->genmode.num_indstages == 0u)
        return;

    for (i = 0; i < stages && i < BP_MAX_TEV_STAGES; i++) {
        if (!bp->tevind[i].raw)
            continue;
        p->live = 1;
        if (bp->tevind[i].add_prev)
            p->persist = 1;
        if (ind_stage_offsets(bp, i) || ind_stage_bumps(bp, i))
            p->lookups |= 1u << bp->tevind[i].ind_stage;
    }
}

/* Which interpolant a lookup reads.
 *
 * Naming a coordinate above the texgen count is undefined on hardware and
 * Dolphin substitutes coordinate 0 for it. This does not, deliberately: the
 * generator already lets a TEV STAGE name a coordinate that does not exist and
 * reads it anyway, so applying the substitution to lookups alone would make
 * the two paths disagree about the same situation -- and the substitution
 * depends on BP's texgen count, which is a second opinion about a number the
 * transform unit owns. tev_texcoord_mask declares whatever is named, so the
 * interpolant at least exists; what the vertex program left in it is the same
 * question the stage path already answers this way. */
static unsigned ind_lookup_coord(const BPState *bp, unsigned i)
{
    return bp->ind_stage[i].coord & 7u;
}

/* The indirect lookups themselves, emitted once before any stage uses them --
 * several TEV stages routinely share one lookup, and sampling it per stage
 * would pay for the fetch again each time. */
static void emit_ind_fetches(FPEmitter *e, const BPState *bp, unsigned lookups)
{
    unsigned i;
    for (i = 0; i < 4u; i++) {
        unsigned map, coord;
        FPSrc c;
        if (!(lookups & (1u << i)))
            continue;
        map   = bp->ind_stage[i].map & 7u;
        coord = ind_lookup_coord(bp, i);
        c = fp_input(FP_IN_TEX(coord));
        /* The lookup's own coordinate is divided by a power of two first,
         * which is how a title samples a small distortion map with a
         * coordinate generated for a much larger surface texture. */
        if (bp->ind_stage[i].scale_s || bp->ind_stage[i].scale_t) {
            float fs = tev_pow2(-(int)bp->ind_stage[i].scale_s);
            float ft = tev_pow2(-(int)bp->ind_stage[i].scale_t);
            fp_emit(e, FP_OP_MUL, TEV_REG_IND(i), FP_MASK_ALL, FP_PREC_FP32, 0,
                    c, fp_imm(fs, ft, 1.0f, 1.0f), fp_none());
            c = fp_temp(TEV_REG_IND(i));
        }
        fp_emit_tex(e, FP_OP_TEX, TEV_REG_IND(i), FP_MASK_ALL, FP_PREC_FP32, 0,
                    map, c);
    }
}

/* The indirect sample as the three signed numbers the offset matrix
 * multiplies.
 *
 * `*fold` comes back as the factor still owed to whatever consumes the result:
 * a texel's integer value is 255 times the sampled float, and when the format
 * needs no shift and nothing else has to happen to the vector, that multiply
 * is free because it folds into a constant the matrix already carries. `*biased`
 * says whether the bias has been applied here or is still owed to the caller
 * (where it is a compile-time constant added to the finished offset).
 *
 * `need_reg` forces the materialised form, which the two dynamic matrix modes
 * need because they read a single component of it twice. */
static FPSrc emit_ind_cvec(FPEmitter *e, const BPState *bp, unsigned n,
                           int need_reg, float *fold, int *biased)
{
    const BPTevIndirect *ti = &bp->tevind[n];
    /* S, T and U are the texel's alpha, blue and green. */
    FPSrc s = fp_swizzle(fp_temp(TEV_REG_IND(ti->ind_stage)), 3, 2, 1, 1);
    unsigned shift = ind_fmt_shift(ti->format);
    float scale;

    if (!need_reg && shift == 0u) {
        *fold = 255.0f;
        *biased = 0;
        return s;
    }

    scale = 255.0f / tev_pow2((int)shift);
    fp_emit(e, FP_OP_MUL, TEV_REG_ICRD, FP_MASK_XYZ, FP_PREC_FP32, 0,
            s, fp_imm(scale, scale, scale, scale), fp_none());
    /* The shift is an integer one on hardware, so what is left of the byte
     * below it is discarded rather than rounded in. */
    if (shift != 0u)
        fp_emit(e, FP_OP_FLR, TEV_REG_ICRD, FP_MASK_XYZ, FP_PREC_FP32, 0,
                fp_temp(TEV_REG_ICRD), fp_none(), fp_none());
    if (ti->bias) {
        float add = ind_bias_add(ti->format);
        fp_emit(e, FP_OP_ADD, TEV_REG_ICRD, FP_MASK_XYZ, FP_PREC_FP32, 0,
                fp_temp(TEV_REG_ICRD),
                fp_imm((ti->bias & 1u) ? add : 0.0f,
                       (ti->bias & 2u) ? add : 0.0f,
                       (ti->bias & 4u) ? add : 0.0f, 0.0f), fp_none());
    }
    *fold = 1.0f;
    *biased = 1;
    return fp_temp(TEV_REG_ICRD);
}

/* This stage's coordinate offset, left in `dst`.xy. */
static void emit_ind_offset(FPEmitter *e, const BPState *bp, unsigned n,
                            unsigned dst, FPSrc uv, float tw, float th)
{
    const BPTevIndirect *ti = &bp->tevind[n];
    const BPIndMatrix *m = &bp->ind_mtx[(ti->matrix_index - 1u) % 3u];
    float k = tev_pow2((int)m->scale - 17);
    float fold;
    int biased;

    if (ti->matrix_id == 0u) {
        FPSrc c = emit_ind_cvec(e, bp, n, 0, &fold, &biased);
        float a[3], b[3], ka = 0.0f, kb = 0.0f;
        unsigned j;

        for (j = 0; j < 3; j++) {
            a[j] = (float)m->m[0][j] / 1024.0f * k / tw;
            b[j] = (float)m->m[1][j] / 1024.0f * k / th;
        }
        /* When the bias was not applied to the vector it is applied to the
         * finished offset instead, where it is a constant the compiler can
         * work out: the matrix is linear, so M.(255s + bias) is
         * 255*(M.s) + M.bias and the second term never changes. */
        if (!biased && ti->bias) {
            float add = ind_bias_add(ti->format);
            for (j = 0; j < 3; j++)
                if (ti->bias & (1u << j)) { ka += a[j] * add; kb += b[j] * add; }
        }
        for (j = 0; j < 3; j++) { a[j] *= fold; b[j] *= fold; }

        fp_emit(e, FP_OP_DP3, dst, FP_MASK_X, FP_PREC_FP32, 0,
                c, fp_imm(a[0], a[1], a[2], 0.0f), fp_none());
        fp_emit(e, FP_OP_DP3, dst, FP_MASK_Y, FP_PREC_FP32, 0,
                c, fp_imm(b[0], b[1], b[2], 0.0f), fp_none());
        if (ka != 0.0f || kb != 0.0f)
            fp_emit(e, FP_OP_ADD, dst, FP_MASK_X | FP_MASK_Y, FP_PREC_FP32, 0,
                    fp_temp(dst), fp_imm(ka, kb, 0.0f, 0.0f), fp_none());
        return;
    }

    /* The two dynamic forms do not transform the sample at all: they scale the
     * stage's OWN coordinate by one component of it, which is how a title
     * makes a coordinate shrink towards the origin rather than slide. The
     * texture size cancels out of this one -- the offset is a fraction of the
     * coordinate rather than a number of texels -- and the selected matrix
     * still supplies the scale exponent even though none of its elements are
     * read. */
    {
        FPSrc c = emit_ind_cvec(e, bp, n, 1, &fold, &biased);
        float kk = tev_pow2((int)m->scale - 25);
        unsigned comp = (ti->matrix_id == 1u) ? 0u : 1u;

        fp_emit(e, FP_OP_MUL, dst, FP_MASK_X | FP_MASK_Y, FP_PREC_FP32, 0,
                uv, fp_imm(kk, kk, 0.0f, 0.0f), fp_none());
        fp_emit(e, FP_OP_MUL, dst, FP_MASK_X | FP_MASK_Y, FP_PREC_FP32, 0,
                fp_temp(dst), fp_swizzle(c, comp, comp, comp, comp), fp_none());
    }
}

/* The coordinate this stage samples with: its own interpolant, wrapped, plus
 * the indirect offset, plus -- if the stage asks for it -- everything the
 * previous stages accumulated. */
static FPSrc emit_ind_coord(FPEmitter *e, const BPState *bp, unsigned n,
                            const TevIndPlan *plan)
{
    const BPTevStage *st = &bp->tev[n];
    const BPTevIndirect *ti = &bp->tevind[n];
    const BPTexture *tx = &bp->tex[st->tex_map & 7u];
    FPSrc uv = fp_input(FP_IN_TEX(st->tex_coord & 7u));
    float tw = tx->width  ? (float)tx->width  : 1.0f;
    float th = tx->height ? (float)tx->height : 1.0f;
    float mod_s = ind_wrap_modulus(ti->wrap_s);
    float mod_t = ind_wrap_modulus(ti->wrap_t);
    int offsets = ind_stage_offsets(bp, n);
    int wrap_is_uv   = (mod_s == 0.0f && mod_t == 0.0f);
    int wrap_is_zero = (mod_s <  0.0f && mod_t <  0.0f);
    /* Where this stage's own coordinate is built. When some stage adds the
     * previous stage's coordinate the accumulator has to survive the stage, so
     * the stage's own value is built somewhere else and folded in afterwards. */
    unsigned build = plan->persist ? TEV_REG_TMP1 : (unsigned)TEV_REG_COORD;
    int own_is_uv = 0, own_is_zero = 0;

    if (wrap_is_uv && !offsets) {
        own_is_uv = 1;
    } else if (wrap_is_zero && !offsets) {
        own_is_zero = 1;
    } else {
        int wrapped_in_build = 0;

        /* The wrap is a modulo in TEXELS, so it is expressed as a fraction of
         * the wrap distance and turned back into a normalised coordinate:
         * frac(uv * size / N) * N / size. Taking the fractional part is the
         * same operation the hardware's bit mask is, including for a negative
         * coordinate, where both produce a positive result. An axis that is
         * "replaced with zero" falls out of the same two constants being zero,
         * and an axis with no wrap out of both being one. */
        if (!wrap_is_uv && !wrap_is_zero) {
            float pre_s = 1.0f, pre_t = 1.0f, post_s = 1.0f, post_t = 1.0f;
            unsigned frc = 0;

            if (mod_s < 0.0f)      { pre_s = 0.0f; post_s = 0.0f; }
            else if (mod_s > 0.0f) { pre_s = tw / mod_s; post_s = mod_s / tw;
                                     frc |= FP_MASK_X; }
            if (mod_t < 0.0f)      { pre_t = 0.0f; post_t = 0.0f; }
            else if (mod_t > 0.0f) { pre_t = th / mod_t; post_t = mod_t / th;
                                     frc |= FP_MASK_Y; }

            fp_emit(e, FP_OP_MUL, build, FP_MASK_ALL, FP_PREC_FP32, 0,
                    uv, fp_imm(pre_s, pre_t, 1.0f, 1.0f), fp_none());
            if (frc)
                fp_emit(e, FP_OP_FRC, build, frc, FP_PREC_FP32, 0,
                        fp_temp(build), fp_none(), fp_none());
            fp_emit(e, FP_OP_MUL, build, FP_MASK_ALL, FP_PREC_FP32, 0,
                    fp_temp(build), fp_imm(post_s, post_t, 1.0f, 1.0f),
                    fp_none());
            wrapped_in_build = 1;
        }

        if (offsets) {
            unsigned oreg = wrapped_in_build ? (unsigned)TEV_REG_TMP0 : build;
            emit_ind_offset(e, bp, n, oreg, uv, tw, th);
            if (wrapped_in_build)
                fp_emit(e, FP_OP_ADD, build, FP_MASK_X | FP_MASK_Y,
                        FP_PREC_FP32, 0, fp_temp(build), fp_temp(oreg),
                        fp_none());
            else if (wrap_is_uv)
                fp_emit(e, FP_OP_ADD, build, FP_MASK_X | FP_MASK_Y,
                        FP_PREC_FP32, 0, fp_temp(build), uv, fp_none());
            /* wrap_is_zero: `build` already holds the offset on its own. */
        }
    }

    if (!plan->persist) {
        if (own_is_uv)
            return uv;
        if (own_is_zero) {
            fp_emit(e, FP_OP_MOV, TEV_REG_COORD, FP_MASK_ALL, FP_PREC_FP32, 0,
                    fp_imm(0.0f, 0.0f, 0.0f, 0.0f), fp_none(), fp_none());
            return fp_temp(TEV_REG_COORD);
        }
        return fp_temp(TEV_REG_COORD);
    }

    /* The accumulator. A stage that does not add to it replaces it, because
     * the next stage that does add reads whatever this one left there. */
    if (ti->add_prev) {
        if (!own_is_zero)
            fp_emit(e, FP_OP_ADD, TEV_REG_COORD, FP_MASK_X | FP_MASK_Y,
                    FP_PREC_FP32, 0, fp_temp(TEV_REG_COORD),
                    own_is_uv ? uv : fp_temp(build), fp_none());
    } else if (own_is_zero) {
        fp_emit(e, FP_OP_MOV, TEV_REG_COORD, FP_MASK_ALL, FP_PREC_FP32, 0,
                fp_imm(0.0f, 0.0f, 0.0f, 0.0f), fp_none(), fp_none());
    } else {
        fp_emit(e, FP_OP_MOV, TEV_REG_COORD, FP_MASK_ALL, FP_PREC_FP32, 0,
                own_is_uv ? uv : fp_temp(build), fp_none(), fp_none());
    }
    return fp_temp(TEV_REG_COORD);
}

/* The alpha bump: five bits taken out of the indirect texel and offered to the
 * combiners as a rasterised colour.
 *
 * libogc documents the value as five bits read from the part of the byte the
 * coordinate does not use -- the low bits for the narrow formats, the top ones
 * for ITF_8 -- and always delivered in the top of an eight-bit field, so it
 * runs 0..248 in steps of eight. Channel 6 is the same value stretched to
 * reach 255, which the hardware does by folding the top three bits back in at
 * the bottom; a multiply by 255/248 lands on the same numbers to well inside
 * a least significant bit. */
static void emit_ind_bump(FPEmitter *e, const BPState *bp, unsigned n)
{
    const BPTevIndirect *ti = &bp->tevind[n];
    static const unsigned k_chan[3] = { 3u, 2u, 1u };   /* S, T, U */
    unsigned ch = k_chan[(ti->bump_alpha - 1u) & 3u];
    FPSrc s = fp_swizzle(fp_temp(TEV_REG_IND(ti->ind_stage)), ch, ch, ch, ch);
    unsigned shift = ind_fmt_shift(ti->format);
    float norm = ((bp->tev[n].ras_channel & 7u) == 6u) ? 255.0f / 248.0f : 1.0f;
    float pre, post;

    if (shift == 0u) {
        /* The top five bits of the byte: divide by eight, drop the remainder,
         * put it back. */
        pre  = 255.0f / 8.0f;
        post = 8.0f / 255.0f * norm;
        fp_emit(e, FP_OP_MUL, TEV_REG_BUMP, FP_MASK_ALL, FP_PREC_FP32, 0,
                s, fp_imm(pre, pre, pre, pre), fp_none());
        fp_emit(e, FP_OP_FLR, TEV_REG_BUMP, FP_MASK_ALL, FP_PREC_FP32, 0,
                fp_temp(TEV_REG_BUMP), fp_none(), fp_none());
    } else {
        /* The low 8 - shift bits, moved up to the top of the field. */
        float m = tev_pow2((int)(8u - shift));
        pre  = 255.0f / m;
        post = 256.0f / 255.0f * norm;
        fp_emit(e, FP_OP_MUL, TEV_REG_BUMP, FP_MASK_ALL, FP_PREC_FP32, 0,
                s, fp_imm(pre, pre, pre, pre), fp_none());
        fp_emit(e, FP_OP_FRC, TEV_REG_BUMP, FP_MASK_ALL, FP_PREC_FP32, 0,
                fp_temp(TEV_REG_BUMP), fp_none(), fp_none());
    }
    fp_emit(e, FP_OP_MUL, TEV_REG_BUMP, FP_MASK_ALL, FP_PREC_FP32, 0,
            fp_temp(TEV_REG_BUMP), fp_imm(post, post, post, post), fp_none());
}

/* ------------------------------------------------------------------ */
/* Fog                                                                  */
/*                                                                      */
/* GX fogs in two halves and it matters which half goes where. The *coordinate* */
/* -- "how far away is this fragment, in the units the fog curve is written in" */
/* -- is an affine function of eye-space depth, so it interpolates exactly and   */
/* is computed once per vertex (xf_program.c) rather than once per fragment.     */
/* What is left here is the part that is genuinely per fragment: clamping the    */
/* coordinate, bending it through the selected curve, and blending the colour.   */
/*                                                                              */
/* The fog colour and the curve are BP state and are therefore literals in the   */
/* program, which is why tev_state_hash covers them. The distances are not:      */
/* they are in the vertex program's constants and can change every draw without  */
/* recompiling anything.                                                         */
/*                                                                              */
/* Measured on the disc: 375 of the 396 course and kart materials name a fog     */
/* set, and the type the engine configures is perspective linear with the range  */
/* adjustment explicitly disabled (EGG::Fog::SetGX ends in a literal             */
/* GXSetFogRangeAdj(0, 0, NULL)). The exponential curves are implemented anyway  */
/* because they are two instructions each.                                       */
/* ------------------------------------------------------------------ */

/* The interpolant the vertex program leaves the fog coordinate in.
 *
 * Texture coordinate 7 rather than the dedicated fog interpolant: GX allows
 * eight texgens and Mario Kart Wii's materials use at most four, so 7 is free,
 * and a texture coordinate is a path this backend has already got right. The
 * fog interpolant on NV4x is entangled with the fixed-function fog unit, which
 * is a second thing to be wrong about for no gain. */
#define TEV_FOG_COORD  7

static void emit_fog(FPEmitter *e, const BPState *bp)
{
    unsigned fsel = bp->fog.fsel;
    FPSrc coord = fp_swizzle(fp_input(FP_IN_TEX(TEV_FOG_COORD)), 0, 0, 0, 0);

    if (fsel == 0)
        return;

    /* fog = saturate(coordinate). The saturate flag does the clamp GX's
     * "clamp(ze - C, 0, 1)" asks for, so it costs no instruction. */
    fp_emit(e, FP_OP_MOV, TEV_REG_TMP0, FP_MASK_X, FP_PREC_FP32, 1,
            coord, fp_none(), fp_none());

    /* The curve. Linear (2) is the identity on the clamped coordinate; the
     * exponentials are Dolphin's, which are in turn the hardware's:
     *     exp      1 - 2^(-8 f)
     *     exp2     1 - 2^(-8 f^2)
     *     rev exp      2^(-8 (1-f))
     *     rev exp2     2^(-8 (1-f)^2)
     * EX2 is a scalar-ish operation here and writes one component, which is
     * all the blend below reads. */
    if (fsel >= 4) {
        int backwards = (fsel >= 6);
        int squared   = (fsel == 5 || fsel == 7);

        if (backwards)                          /* f = 1 - f */
            fp_emit(e, FP_OP_ADD, TEV_REG_TMP0, FP_MASK_X, FP_PREC_FP32, 0,
                    fp_imm(1.0f, 1.0f, 1.0f, 1.0f),
                    fp_negate(fp_temp(TEV_REG_TMP0)), fp_none());
        if (squared)                            /* f = f * f */
            fp_emit(e, FP_OP_MUL, TEV_REG_TMP0, FP_MASK_X, FP_PREC_FP32, 0,
                    fp_temp(TEV_REG_TMP0), fp_temp(TEV_REG_TMP0), fp_none());

        fp_emit(e, FP_OP_MUL, TEV_REG_TMP0, FP_MASK_X, FP_PREC_FP32, 0,
                fp_temp(TEV_REG_TMP0),
                fp_imm(-8.0f, -8.0f, -8.0f, -8.0f), fp_none());
        fp_emit(e, FP_OP_EX2, TEV_REG_TMP0, FP_MASK_X, FP_PREC_FP32, 0,
                fp_swizzle(fp_temp(TEV_REG_TMP0), 0, 0, 0, 0),
                fp_none(), fp_none());
        if (!backwards)                         /* f = 1 - 2^(...) */
            fp_emit(e, FP_OP_ADD, TEV_REG_TMP0, FP_MASK_X, FP_PREC_FP32, 0,
                    fp_imm(1.0f, 1.0f, 1.0f, 1.0f),
                    fp_negate(fp_temp(TEV_REG_TMP0)), fp_none());
    }

    /* prev.rgb = prev.rgb + f * (fogcolour - prev.rgb).
     *
     * Written as a subtract and a MAD rather than an LRP for the reason the
     * rest of this file gives: LRP has no reference encoding to check against.
     * The colour is the one literal each instruction is allowed. */
    fp_emit(e, FP_OP_ADD, TEV_REG_TMP1, FP_MASK_XYZ, FP_PREC_FP32, 0,
            fp_imm(bp->fog.color[0], bp->fog.color[1], bp->fog.color[2], 0.0f),
            fp_negate(fp_temp(TEV_REG_PREV)), fp_none());
    fp_emit(e, FP_OP_MAD, TEV_REG_PREV, FP_MASK_XYZ, FP_PREC_FP32, 0,
            fp_swizzle(fp_temp(TEV_REG_TMP0), 0, 0, 0, 0),
            fp_temp(TEV_REG_TMP1), fp_temp(TEV_REG_PREV));
}

/* Which texture-coordinate interpolants a generated program reads. The RSX
 * routes only the ones named here, and an unnamed coordinate is left in its
 * default 2D configuration -- two live components and two frozen ones. */
u32 tev_texcoord_mask(const BPState *bp)
{
    unsigned stages = bp_tev_stage_count(bp);
    unsigned i;
    u32 mask = 0;

    for (i = 0; i < stages && i < BP_MAX_TEV_STAGES; i++)
        if (bp->tev[i].tex_enable)
            mask |= 1u << (bp->tev[i].tex_coord & 7u);
    if ((g_gx_state_mask & GX_STATE_FOG) && bp->fog.fsel)
        mask |= 1u << TEV_FOG_COORD;
    /* An indirect lookup reads a coordinate of its own, which is routinely not
     * one any TEV stage samples with -- an undeclared one arrives with only its
     * first two components live, and the distortion map is then fetched from a
     * coordinate the vertex program never wrote. */
    {
        TevIndPlan plan;
        unsigned k;
        ind_plan(bp, &plan);
        for (k = 0; k < 4u; k++)
            if (plan.lookups & (1u << k))
                mask |= 1u << ind_lookup_coord(bp, k);
        /* A stage that does not sample still reads its interpolant when the
         * indirect unit is carrying a coordinate forward through it. */
        if (plan.live)
            for (i = 0; i < stages && i < BP_MAX_TEV_STAGES; i++)
                mask |= 1u << (bp->tev[i].tex_coord & 7u);
    }
    /* Coordinates 0 and 1 have always been declared unconditionally and the
     * console is happy with that; keeping them in avoids making "fewer
     * interpolants" a variable in the first hardware run that has fog. */
    return mask | 0x3u;
}

/* ------------------------------------------------------------------ */

int tev_generate(const BPState *bp, FPEmitter *e, TevProgramInfo *info)
{
    unsigned stages = bp_tev_stage_count(bp);
    unsigned i;
    u32 before_words = e->used;
    unsigned instructions = 0;
    int konst_on = (g_gx_state_mask & GX_STATE_KONST) != 0;
    TevIndPlan plan;

    if (info)
        memset(info, 0, sizeof *info);

    ind_plan(bp, &plan);

    /* Materialise the TEV register colours before any stage reads them. On
     * hardware these are persistent pipeline registers loaded by BP writes;
     * in a generated program they are plain temporaries, and the first frame
     * of the menus taught us what uninitialised ones look like: every stage
     * that multiplies by C0 goes to zero, and the whole screen with it. The
     * hash below covers the colours, so a program is regenerated when its
     * material changes. */
    for (i = 0; i < 4; i++) {
        static const unsigned reg_for[4] = {
            TEV_REG_PREV, TEV_REG_C0, TEV_REG_C1, TEV_REG_C2
        };
        fp_emit(e, FP_OP_MOV, reg_for[i], FP_MASK_ALL, FP_PREC_FP32, 0,
                fp_imm(bp->tev_reg[i][0], bp->tev_reg[i][1],
                       bp->tev_reg[i][2], bp->tev_reg[i][3]),
                fp_none(), fp_none());
    }

    /* The indirect lookups, before any stage that reads one. The running
     * coordinate starts at zero because the first stage that ADDS to it must
     * not read whatever the register happened to hold. */
    if (plan.live) {
        emit_ind_fetches(e, bp, plan.lookups);
        if (plan.persist)
            fp_emit(e, FP_OP_MOV, TEV_REG_COORD, FP_MASK_ALL, FP_PREC_FP32, 0,
                    fp_imm(0.0f, 0.0f, 0.0f, 0.0f), fp_none(), fp_none());
    }

    for (i = 0; i < stages && i < BP_MAX_TEV_STAGES; i++) {
        const BPTevStage *st = &bp->tev[i];
        u32 cc = st->color_env;
        u32 ac = st->alpha_env;
        int bumped = plan.live && ind_stage_bumps(bp, i);
        /* The indirect unit runs whether or not the stage samples anything:
         * it can still produce the alpha bump, and it still carries the
         * coordinate forward for a later stage to add to. */
        FPSrc coord = plan.live ? emit_ind_coord(e, bp, i, &plan)
                                : fp_input(FP_IN_TEX(st->tex_coord & 7u));
        FPSrc ras;
        u32 words_before = e->used;
        TevStageSrc src;
        /* Which swap table this stage selects for each of its two swappable
         * inputs. Both fields live in the *alpha* environment register, which
         * is the easiest place to forget to look. */
        static const unsigned k_identity[4] = { 0, 1, 2, 3 };
        const unsigned *rsw = konst_on ? bp->tev_swap[ac & 3u] : k_identity;
        const unsigned *tsw = konst_on ? bp->tev_swap[(ac >> 2) & 3u]
                                       : k_identity;

        /* Sample this stage's texture, if it uses one. A stage with texturing
         * disabled must not fetch: an unbound unit returns undefined data, and
         * a stage that ignores it would still pay the fetch. */
        if (bumped)
            emit_ind_bump(e, bp, i);
        ras = bumped ? fp_temp(TEV_REG_BUMP) : ras_source(st->ras_channel);

        if (st->tex_enable) {
            const BPTexture *tx = &bp->tex[st->tex_map & 7u];
            unsigned wrap_s = tx->mode0 & 3u, wrap_t = (tx->mode0 >> 2) & 3u;

            /* GX_REPEAT, in the program rather than only on the sampler.
             *
             * Every texture this backend uploads is a *linear* RSX texture --
             * it has to be, because Wii art is routinely a size the swizzled
             * layout cannot express (233x167, 831x316, 8x167 on the title
             * screen alone). The NV4x sampler treats a linear texture as a
             * rectangle texture, and a rectangle texture clamps: the wrap mode
             * written to the unit is accepted and then ignored. A title that
             * tiles a 32x32 pattern thirty times down a full-screen quad --
             * which is exactly what draw 3 of the Mario Kart Wii title screen
             * does -- gets one stretched copy instead.
             *
             * Taking the fractional part of the coordinate first reproduces
             * the repeat for the sampled texel, needs nothing from the
             * hardware but FRC, and is harmless if the sampler honours the
             * wrap after all: the coordinate is already inside [0,1) by then.
             * The seam it leaves is one texel wide at each tile edge, where
             * filtering clamps instead of wrapping -- which against thirty
             * missing tiles is not a trade worth agonising over.
             *
             * GX_MIRROR is left to the sampler: it costs three more
             * instructions to fold and no draw in the title screen uses it. */
            if (wrap_s == 1u || wrap_t == 1u) {
                fp_emit(e, FP_OP_MOV, TEV_REG_TMP0, FP_MASK_ALL, FP_PREC_FP32,
                        0, coord, fp_none(), fp_none());
                if (wrap_s == 1u)
                    fp_emit(e, FP_OP_FRC, TEV_REG_TMP0, FP_MASK_X,
                            FP_PREC_FP32, 0, fp_temp(TEV_REG_TMP0),
                            fp_none(), fp_none());
                if (wrap_t == 1u)
                    fp_emit(e, FP_OP_FRC, TEV_REG_TMP0, FP_MASK_Y,
                            FP_PREC_FP32, 0, fp_temp(TEV_REG_TMP0),
                            fp_none(), fp_none());
                coord = fp_temp(TEV_REG_TMP0);
            }

            fp_emit_tex(e, FP_OP_TEX, TEV_REG_TEX, FP_MASK_ALL, FP_PREC_FP32, 0,
                        st->tex_map & 7u, coord);
        }

        /* This stage's operand sources. The rasterised colour is only swapped
         * when it is a real interpolant: for channels 2..7 it is a literal
         * zero, and permuting the components of zero is both pointless and a
         * second literal in an instruction that can hold one. */
        {
            float kc[4], ka[4];
            src.ras_rgb = (st->ras_channel & 7u) < 2 ? swap_rgb(ras, rsw) : ras;
            src.ras_a   = (st->ras_channel & 7u) < 2 ? swap_alpha(ras, rsw)
                                                     : ras;
            src.tex_rgb = swap_rgb(fp_temp(TEV_REG_TEX), tsw);
            src.tex_a   = swap_alpha(fp_temp(TEV_REG_TEX), tsw);
            if (konst_on) {
                konst_value(bp, st->konst_color, 0, kc);
                konst_value(bp, st->konst_alpha, 1, ka);
                src.konst_rgb = fp_imm(kc[0], kc[1], kc[2], kc[3]);
                src.konst_a   = fp_imm(ka[0], ka[1], ka[2], ka[3]);
            } else {
                /* The pre-konst behaviour, kept reachable so the console can
                 * step the group off and get the previous picture back. */
                src.konst_rgb = fp_temp(TEV_REG_C2);
                src.konst_a   = fp_swizzle(fp_temp(TEV_REG_C2), 3, 3, 3, 3);
            }
        }

        /* Colour: .xyz only, so the alpha combiner's result is not disturbed. */
        emit_combiner(e, dest_reg(BITS(cc, TEV_CC_DEST_SHIFT, 2)), FP_MASK_XYZ,
                      color_arg(BITS(cc, TEV_CC_A_SHIFT, TEV_CC_ARG_BITS), &src),
                      color_arg(BITS(cc, TEV_CC_B_SHIFT, TEV_CC_ARG_BITS), &src),
                      color_arg(BITS(cc, TEV_CC_C_SHIFT, TEV_CC_ARG_BITS), &src),
                      color_arg(BITS(cc, TEV_CC_D_SHIFT, TEV_CC_ARG_BITS), &src),
                      BITS(cc, TEV_CC_BIAS_SHIFT, 2),
                      BITS(cc, TEV_CC_SCALE_SHIFT, 2),
                      (int)BITS(cc, TEV_CC_SUB_SHIFT, 1),
                      (int)BITS(cc, TEV_CC_CLAMP_SHIFT, 1));

        /* Alpha: .w only, and its own independent operand encoding. */
        emit_combiner(e, dest_reg(BITS(ac, TEV_AC_DEST_SHIFT, 2)), FP_MASK_W,
                      alpha_arg(BITS(ac, TEV_AC_A_SHIFT, TEV_AC_ARG_BITS), &src),
                      alpha_arg(BITS(ac, TEV_AC_B_SHIFT, TEV_AC_ARG_BITS), &src),
                      alpha_arg(BITS(ac, TEV_AC_C_SHIFT, TEV_AC_ARG_BITS), &src),
                      alpha_arg(BITS(ac, TEV_AC_D_SHIFT, TEV_AC_ARG_BITS), &src),
                      BITS(ac, TEV_AC_BIAS_SHIFT, 2),
                      BITS(ac, TEV_AC_SCALE_SHIFT, 2),
                      (int)BITS(ac, TEV_AC_SUB_SHIFT, 1),
                      (int)BITS(ac, TEV_AC_CLAMP_SHIFT, 1));

        (void)words_before;
        if (e->overflow)
            break;
    }

    /* Fog last: GX applies it to the pipeline's output, after every stage and
     * before the blend unit sees the fragment. Doing it earlier would fog a
     * value a later stage then multiplies, which is a different picture. */
    if (g_gx_state_mask & GX_STATE_FOG)
        emit_fog(e, bp);

    /* The pipeline's result is whatever the last stage left in prev. */
    {   /* Debug (file-armed on the console): output APREV broadcast as
         * grayscale instead of the colour, making the fragment ALPHA the
         * pipeline actually computed visible in a screenshot. Splits "the
         * program computes the wrong alpha on the RSX" from "the alpha is
         * right and the blend unit misuses it" -- the two remaining suspects
         * for the title-screen bars, after the simulator proved the generated
         * IR yields srcA=0 for the quad's captured state. */
        extern int g_tev_show_alpha;
        extern int g_tev_show_red;
        if (g_tev_show_red) {
            /* Diagnostic: solid red from every fragment program. If geometry
             * rasterises at all it shows as red silhouettes; separates
             * "no fragments" from "fragments coloured black". */
            fp_emit(e, FP_OP_MOV, 0, FP_MASK_ALL, FP_PREC_FP32, 0,
                    fp_imm(1.0f, 0.0f, 0.0f, 1.0f),
                    fp_none(), fp_none());
        } else if (g_tev_show_alpha)
            fp_emit(e, FP_OP_MOV, 0, FP_MASK_ALL, FP_PREC_FP32, 0,
                    fp_swizzle(fp_temp(TEV_REG_PREV), 3, 3, 3, 3),
                    fp_none(), fp_none());
        else
            fp_emit(e, FP_OP_MOV, 0, FP_MASK_ALL, FP_PREC_FP32, 0,
                    fp_temp(TEV_REG_PREV), fp_none(), fp_none());
    }
    fp_finish(e);

    /* Counted rather than accumulated, because an instruction that takes a
     * literal occupies two slots and a count kept by hand drifts from the
     * truth exactly when a program starts using constants. */
    {
        u32 w;
        for (w = before_words; w < e->used; w += 4) {
            u32 logical = (e->code[w] << 16) | (e->code[w] >> 16);
            unsigned op = (logical >> FP_OPCODE_SHIFT) & 0x3Fu;
            instructions++;
            /* A literal slot follows an instruction that referenced one; it is
             * data, and skipping it is what keeps the walk aligned. Detected
             * by the source type, and for EVERY opcode rather than for the
             * four that used to be the only ones the generator gave a literal
             * to: the indirect unit's DP3 carries the offset matrix in exactly
             * the same slot, and a walk that does not skip it decodes four
             * floats as an instruction and counts nonsense from there on. */
            (void)op;
            {
                u32 w1 = (e->code[w + 1] << 16) | (e->code[w + 1] >> 16);
                u32 w2 = (e->code[w + 2] << 16) | (e->code[w + 2] >> 16);
                u32 w3 = (e->code[w + 3] << 16) | (e->code[w + 3] >> 16);
                if ((w1 & 3u) == FP_REG_TYPE_CONST ||
                    (w2 & 3u) == FP_REG_TYPE_CONST ||
                    (w3 & 3u) == FP_REG_TYPE_CONST)
                    w += 4;
            }
        }
    }

    if (info) {
        info->instructions = instructions;
        info->words = e->used - before_words;
        info->temps_used = e->num_regs;
        info->truncated = e->overflow;
    }
    return e->overflow ? -1 : 0;
}

/* ------------------------------------------------------------------ */

int g_tev_show_alpha;
int g_tev_show_red;

u64 tev_state_hash(const BPState *bp)
{
    /* FNV-1a: cheap, and good enough for a cache key whose collisions would
     * merely produce a recompile rather than a wrong program -- because the
     * cache stores the state alongside the program and compares it. */
    u64 h = 1469598103934665603ull;
    unsigned stages = bp_tev_stage_count(bp);
    unsigned i;

    #define MIX(v) do { h ^= (u64)(v); h *= 1099511628211ull; } while (0)

    MIX(stages);
    MIX(bp->genmode.num_texgens);
    MIX(bp->genmode.num_colorchans);

    /* The register colours are baked into the program as literals, so they
     * are part of its identity. */
    {
        unsigned r5, c5;
        union { float f; u32 u; } cv5;
        for (r5 = 0; r5 < 4; r5++)
            for (c5 = 0; c5 < 4; c5++) {
                cv5.f = bp->tev_reg[r5][c5];
                MIX(cv5.u);
            }
    }

    for (i = 0; i < stages && i < BP_MAX_TEV_STAGES; i++) {
        MIX(bp->tev[i].color_env);
        MIX(bp->tev[i].alpha_env);
        MIX(bp->tev[i].tex_map);
        MIX(bp->tev[i].tex_coord);
        MIX(bp->tev[i].tex_enable);
        MIX(bp->tev[i].ras_channel);
        /* The wrap mode is part of the *program* now, not only of the sampler
         * state, so two draws that differ only in it are two programs. */
        MIX(bp->tex[bp->tev[i].tex_map & 7u].mode0 & 0xFu);
        /* Konst is resolved to a literal at generation time, so both the
         * selection and the bank it names belong to the program's identity.
         * Leaving the selection out would hand a stage the previous
         * material's constant, which is the failure the konst work fixes. */
        MIX(bp->tev[i].konst_color);
        MIX(bp->tev[i].konst_alpha);
    }

    /* The konst colours themselves, and the swap tables, are literals and
     * swizzles baked into the code. Only mixed in when the group is on, so
     * stepping it off from the pad reproduces the previous cache behaviour
     * exactly rather than merely the previous pictures. */
    if (g_gx_state_mask & GX_STATE_KONST) {
        unsigned r6, c6, t6;
        union { float f; u32 u; } cv6;
        for (r6 = 0; r6 < 4; r6++)
            for (c6 = 0; c6 < 4; c6++) {
                cv6.f = bp->tev_konst[r6][c6];
                MIX(cv6.u);
            }
        for (t6 = 0; t6 < 4; t6++)
            for (c6 = 0; c6 < 4; c6++)
                MIX(bp->tev_swap[t6][c6]);
    }

    /* Fog: the curve and the colour are compiled in; the distances are vertex
     * constants and deliberately are not. */
    if (g_gx_state_mask & GX_STATE_FOG) {
        unsigned c7;
        union { float f; u32 u; } cv7;
        MIX(bp->fog.fsel);
        for (c7 = 0; c7 < 3; c7++) { cv7.f = bp->fog.color[c7]; MIX(cv7.u); }
    }

    /* Indirect texturing. The matrices, the coordinate scales and the texture
     * sizes are all folded into literals in the generated code, so all of them
     * are part of the program's identity -- a material whose distortion matrix
     * is animated is a new program every time it changes, which is what the
     * cache is for. Only mixed in when the group is on, so stepping it off from
     * the pad reproduces the previous cache behaviour exactly. */
    if (g_gx_state_mask & GX_STATE_INDIRECT) {
        MIX(bp->genmode.num_indstages);
        if (bp->genmode.num_indstages) {
            unsigned k8, j8;
            for (i = 0; i < stages && i < BP_MAX_TEV_STAGES; i++) {
                MIX(bp->tevind[i].raw);
                if (bp->tevind[i].raw) {
                    /* The offset is in texels and the coordinate is not, so
                     * the sampled texture's size is a constant in the code. */
                    MIX(bp->tex[bp->tev[i].tex_map & 7u].width);
                    MIX(bp->tex[bp->tev[i].tex_map & 7u].height);
                }
            }
            for (k8 = 0; k8 < 4; k8++) {
                MIX(bp->ind_stage[k8].map);
                MIX(bp->ind_stage[k8].coord);
                MIX(bp->ind_stage[k8].scale_s);
                MIX(bp->ind_stage[k8].scale_t);
            }
            for (k8 = 0; k8 < 3; k8++) {
                MIX(bp->ind_mtx[k8].scale);
                for (j8 = 0; j8 < 3; j8++) {
                    MIX((u32)bp->ind_mtx[k8].m[0][j8]);
                    MIX((u32)bp->ind_mtx[k8].m[1][j8]);
                }
            }
        }
    }

    /* Alpha test is compiled into the program (a KIL), so it belongs to the
     * key. Depth and blend state do not -- they are pipeline configuration the
     * same program runs under, and including them would multiply the cache for
     * no benefit. */
    MIX(bp->alpha_test.comp0);
    MIX(bp->alpha_test.comp1);
    MIX(bp->alpha_test.logic);
    MIX(bp->alpha_test.ref0);
    MIX(bp->alpha_test.ref1);

    #undef MIX
    return h;
}
