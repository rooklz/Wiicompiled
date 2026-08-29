/* xf_program.c — lowering the transform unit to a vertex program.
 *
 * See xf_program.h. Position, colour channels, texture coordinates and fog,
 * generated per XF configuration and cached against its shape.
 *
 * The vertex unit has no register-pressure cliff to design around, so the code
 * below is written to be obvious rather than short. What it is *not* free to be
 * is approximate: everything here decides where geometry lands, and a transform
 * that is nearly right produces a picture that is recognisably the game and
 * wrong in a way no counter reports.
 *
 * Three groups are behind feature bits (gx_features.h) so a console session can
 * step them the way it stepped the depth test:
 *
 *   GX_STATE_TEXGEN    texgen source rows, ST vs STQ, the dual transform
 *   GX_STATE_LIGHTING  per-channel material/ambient sources and up to 8 lights
 *   GX_STATE_FOG       the fog coordinate (the curve itself is in tev_program.c)
 *
 * With all three clear the generated program is instruction-for-instruction
 * what it was before any of this existed, which is what makes "the title screen
 * still works" a check rather than a hope.
 */
#include "xf_program.h"
#include "gx_features.h"

#include <string.h>
#include <math.h>

#define BITSX(v, lsb, width) (((u32)(v) >> (lsb)) & ((1u << (width)) - 1u))

/* ------------------------------------------------------------------ */
/* Scratch registers                                                    */
/*                                                                      */
/* Fixed rather than allocated. The vertex unit has 32 temporaries and the       */
/* longest program this file can emit -- eight lights on two channels -- uses    */
/* six of them, so allocation would buy nothing and cost a class of bug.         */
/* ------------------------------------------------------------------ */

#define XR_POS      0   /* position in view space (lighting)            */
#define XR_NRM      1   /* normal in view space                         */
#define XR_ACC      2   /* light accumulator for the channel in hand    */
#define XR_LDIR     3   /* direction to the light                       */
#define XR_TMP      4
#define XR_TMP2     5
#define XR_COORD    6   /* texgen input coordinate                      */
#define XR_TCRES    7   /* texgen result before the post transform      */
/* The channel's material colour, which has to survive every light in the
 * channel's mask -- it is multiplied in only at the very end. It was briefly
 * kept in XR_TMP2, which the per-light normalisation uses as scratch: the
 * result was a material whose red channel came from a reciprocal square root,
 * and only red, because that is the one component the scratch touches. */
#define XR_MAT      8

/* ------------------------------------------------------------------ */

/* Transform a vertex attribute by a 4x4 matrix held in four consecutive
 * constant registers, writing one component per dot product.
 *
 * Four instructions rather than one, because the hardware has no matrix
 * multiply: a DP4 produces a scalar, and the write mask is what places it. The
 * masks must name a single component each -- a shared mask would have every
 * dot product overwrite the last. */
static void emit_transform(VPEmitter *e, unsigned out, int to_output,
                           VPSrc src, unsigned const_base)
{
    static const unsigned k_mask[4] = {
        VP_MASK_X, VP_MASK_Y, VP_MASK_Z, VP_MASK_W
    };
    unsigned i;

    for (i = 0; i < 4; i++)
        vp_emit_vec(e, VP_VEC_DP4, out, to_output, k_mask[i], 0,
                    src, vp_const(const_base + i), vp_none());
}

/* Three dot products into x, y, z of a temporary -- a 3x4 matrix applied to a
 * homogeneous input. The fourth component is left alone, which is what lets
 * the caller keep a 1 in w for the row that follows. */
static void emit_transform3(VPEmitter *e, unsigned dst_temp, VPSrc src,
                            unsigned const_base, unsigned rows)
{
    static const unsigned k_mask[3] = { VP_MASK_X, VP_MASK_Y, VP_MASK_Z };
    unsigned i;
    for (i = 0; i < rows && i < 3; i++)
        vp_emit_vec(e, VP_VEC_DP4, dst_temp, 0, k_mask[i], 0,
                    src, vp_const(const_base + i), vp_none());
}

/* The two numbers generated code needs constantly. Sourced from a constant
 * register rather than from an immediate because a vertex instruction may name
 * only one constant register: writing them this way costs one instruction and
 * leaves the *other* operand free to be a matrix row or a light colour. */
static VPSrc vp_zero_src(void) { return vp_swizzle(vp_const(XF_CONST_ONE), 0, 0, 0, 0); }
static VPSrc vp_one_src(void)  { return vp_swizzle(vp_const(XF_CONST_ONE), 1, 1, 1, 1); }

static void emit_set_one(VPEmitter *e, unsigned reg, unsigned mask)
{
    if (!mask) return;
    vp_emit_vec(e, VP_VEC_MOV, reg, 0, mask, 0, vp_one_src(), vp_none(),
                vp_none());
}

/* dst = normalize(src.xyz). RSQ is a scalar operation and writes one component,
 * so the reciprocal square root is computed into a scratch component and then
 * broadcast -- three instructions, and the shape the encoder has been verified
 * on. */
static void emit_normalize(VPEmitter *e, unsigned dst, VPSrc src,
                           unsigned scratch)
{
    vp_emit_vec(e, VP_VEC_DP3, scratch, 0, VP_MASK_X, 0,
                src, src, vp_none());
    vp_emit_sca(e, VP_SCA_RSQ, scratch, 0, VP_MASK_X,  0,
                vp_swizzle(vp_temp(scratch), 0, 0, 0, 0));
    vp_emit_vec(e, VP_VEC_MUL, dst, 0, VP_MASK_X | VP_MASK_Y | VP_MASK_Z, 0,
                src, vp_swizzle(vp_temp(scratch), 0, 0, 0, 0), vp_none());
}

/* ------------------------------------------------------------------ */
/* Lighting                                                             */
/*                                                                      */
/* GX lights per *channel*, and a channel is a colour and an alpha that are     */
/* configured independently: a title can light the colour from four lights and  */
/* take the alpha straight from the vertex. The two halves therefore share this */
/* code and differ only in which components they write.                          */
/*                                                                              */
/* The formula is Dolphin's, which is the hardware's:                            */
/*                                                                              */
/*     mat  = matsource ? vertex colour : material register                      */
/*     lacc = lit ? (ambsource ? vertex colour : ambient register) : 1           */
/*     for each light in the mask:                                              */
/*         lacc += attenuate(light) * diffuse(light) * light colour             */
/*     out  = mat * clamp(lacc, 0, 1)                                            */
/*                                                                              */
/* Measured on the disc: every lit channel in the six courses and two karts read */
/* out of Mario Kart Wii asks for diffuse function "clamp" and attenuation       */
/* "spot"; none asks for the specular form. All four are generated anyway --     */
/* the light mask is runtime state the model files do not contain, so the        */
/* measurement bounds what the *materials* ask for and not what the engine will  */
/* have configured by the time the frame is drawn.                               */
/* ------------------------------------------------------------------ */

#define LIT_MATSRC(v)   BITSX(v, 0, 1)      /* 1 = vertex colour        */
#define LIT_ENABLE(v)   BITSX(v, 1, 1)
#define LIT_AMBSRC(v)   BITSX(v, 6, 1)      /* 1 = vertex colour        */
#define LIT_DIFFUSE(v)  BITSX(v, 7, 2)      /* 0 none, 1 sign, 2 clamp  */
#define LIT_ATTN(v)     BITSX(v, 9, 2)      /* 0 none, 1 spec, 2 dir, 3 spot */
#define LIT_MASK(v)     (BITSX(v, 2, 4) | (BITSX(v, 11, 4) << 4))

/* One light's contribution, accumulated into XR_ACC under `mask`.
 *
 * `mask` is VP_MASK_X|Y|Z for a colour channel and VP_MASK_W for an alpha one;
 * the alpha case broadcasts the light's own alpha rather than its colour, which
 * is why the source swizzle is a parameter. */
static void emit_light(VPEmitter *e, unsigned light, unsigned attn,
                       unsigned diffuse, unsigned mask, int alpha)
{
    unsigned base = XF_CONST_LIGHT(light);
    VPSrc lcol = vp_const(base + XF_LIGHT_COLOR);
    unsigned rgbmask = VP_MASK_X | VP_MASK_Y | VP_MASK_Z;

    if (alpha)
        lcol = vp_swizzle(lcol, 3, 3, 3, 3);

    /* Direction from the vertex to the light, in view space. */
    vp_emit_vec(e, VP_VEC_ADD, XR_LDIR, 0, rgbmask, 0,
                vp_const(base + XF_LIGHT_POS),
                vp_negate(vp_temp(XR_POS)), vp_none());

    if (attn == 3) {
        /* Spot: an angular polynomial over the cosine to the spot axis,
         * divided by a distance polynomial. Both are quadratics evaluated as
         * a DP3 against (1, t, t^2), which is why the vector is assembled
         * component by component -- there is no instruction that squares into
         * a neighbouring lane. */
        vp_emit_vec(e, VP_VEC_DP3, XR_TMP, 0, VP_MASK_X, 0,
                    vp_temp(XR_LDIR), vp_temp(XR_LDIR), vp_none());   /* d^2 */
        vp_emit_sca(e, VP_SCA_RSQ, XR_TMP, 0, VP_MASK_Y, 0,
                    vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0));         /* 1/d */
        vp_emit_vec(e, VP_VEC_MUL, XR_LDIR, 0, rgbmask, 0,
                    vp_temp(XR_LDIR),
                    vp_swizzle(vp_temp(XR_TMP), 1, 1, 1, 1), vp_none());
        vp_emit_vec(e, VP_VEC_MUL, XR_TMP, 0, VP_MASK_Z, 0,
                    vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0),
                    vp_swizzle(vp_temp(XR_TMP), 1, 1, 1, 1), vp_none()); /* d */

        /* (1, d, d^2) . distatt */
        emit_set_one(e, XR_TMP2, VP_MASK_X);
        vp_emit_vec(e, VP_VEC_MOV, XR_TMP2, 0, VP_MASK_Y, 0,
                    vp_swizzle(vp_temp(XR_TMP), 2, 2, 2, 2), vp_none(),
                    vp_none());
        vp_emit_vec(e, VP_VEC_MOV, XR_TMP2, 0, VP_MASK_Z, 0,
                    vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0), vp_none(),
                    vp_none());
        vp_emit_vec(e, VP_VEC_DP3, XR_TMP, 0, VP_MASK_W, 0,
                    vp_const(base + XF_LIGHT_DISTATT), vp_temp(XR_TMP2),
                    vp_none());

        /* a = max(0, ldir . spot axis), then (1, a, a^2) . cosatt */
        vp_emit_vec(e, VP_VEC_DP3, XR_TMP2, 0, VP_MASK_W, 0,
                    vp_temp(XR_LDIR), vp_const(base + XF_LIGHT_DIR), vp_none());
        vp_emit_vec(e, VP_VEC_MAX, XR_TMP2, 0, VP_MASK_W, 0,
                    vp_temp(XR_TMP2), vp_zero_src(), vp_none());
        emit_set_one(e, XR_TMP2, VP_MASK_X);
        vp_emit_vec(e, VP_VEC_MOV, XR_TMP2, 0, VP_MASK_Y, 0,
                    vp_swizzle(vp_temp(XR_TMP2), 3, 3, 3, 3), vp_none(),
                    vp_none());
        vp_emit_vec(e, VP_VEC_MUL, XR_TMP2, 0, VP_MASK_Z, 0,
                    vp_swizzle(vp_temp(XR_TMP2), 3, 3, 3, 3),
                    vp_swizzle(vp_temp(XR_TMP2), 3, 3, 3, 3), vp_none());
        vp_emit_vec(e, VP_VEC_DP3, XR_TMP2, 0, VP_MASK_W, 0,
                    vp_const(base + XF_LIGHT_COSATT), vp_temp(XR_TMP2),
                    vp_none());
        vp_emit_vec(e, VP_VEC_MAX, XR_TMP2, 0, VP_MASK_W, 0,
                    vp_temp(XR_TMP2), vp_zero_src(), vp_none());

        vp_emit_sca(e, VP_SCA_RCP, XR_TMP, 0, VP_MASK_W, 0,
                    vp_swizzle(vp_temp(XR_TMP), 3, 3, 3, 3));
        vp_emit_vec(e, VP_VEC_MUL, XR_TMP, 0, VP_MASK_X, 0,
                    vp_swizzle(vp_temp(XR_TMP2), 3, 3, 3, 3),
                    vp_swizzle(vp_temp(XR_TMP), 3, 3, 3, 3), vp_none());
    } else {
        /* None and Dir behave identically here: full strength, direction from
         * the vertex to the light. The specular form is folded onto them
         * rather than approximated -- no material measured on the disc selects
         * it, and a highlight computed wrongly is worse than no highlight. */
        emit_normalize(e, XR_LDIR, vp_temp(XR_LDIR), XR_TMP2);
        emit_set_one(e, XR_TMP, VP_MASK_X);
    }

    /* The diffuse term, into XR_TMP.y. */
    switch (diffuse) {
    case 0:     /* none: the light contributes its full colour */
        emit_set_one(e, XR_TMP, VP_MASK_Y);
        break;
    case 1:     /* sign: the raw dot product, negatives included */
        vp_emit_vec(e, VP_VEC_DP3, XR_TMP, 0, VP_MASK_Y, 0,
                    vp_temp(XR_LDIR), vp_temp(XR_NRM), vp_none());
        break;
    default:    /* clamp */
        vp_emit_vec(e, VP_VEC_DP3, XR_TMP, 0, VP_MASK_Y, 0,
                    vp_temp(XR_LDIR), vp_temp(XR_NRM), vp_none());
        vp_emit_vec(e, VP_VEC_MAX, XR_TMP, 0, VP_MASK_Y, 0,
                    vp_temp(XR_TMP), vp_zero_src(), vp_none());
        break;
    }

    /* acc += attn * diffuse * light colour. */
    vp_emit_vec(e, VP_VEC_MUL, XR_TMP, 0, VP_MASK_X, 0,
                vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0),
                vp_swizzle(vp_temp(XR_TMP), 1, 1, 1, 1), vp_none());
    vp_emit_vec(e, VP_VEC_MAD, XR_ACC, 0, mask, 0,
                vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0), lcol, vp_temp(XR_ACC));
}

/* One colour channel: colour components and alpha are separate LitChannel
 * registers and are generated one after the other into the same output. */
static void emit_channel(VPEmitter *e, const GXState *g, unsigned chan,
                         unsigned *lights_used)
{
    u32 ccol = g->xf.mem[0x100E + chan];
    u32 calp = g->xf.mem[0x1010 + chan];
    unsigned out = chan ? VP_OUT_COL1 : VP_OUT_COL0;
    unsigned vin = chan ? VP_IN_COL1 : VP_IN_COL0;
    unsigned rgbmask = VP_MASK_X | VP_MASK_Y | VP_MASK_Z;
    unsigned i;

    /* XR_MAT holds the material colour: the vertex's, or the register's. */
    vp_emit_vec(e, VP_VEC_MOV, XR_MAT, 0, rgbmask, 0,
                LIT_MATSRC(ccol) ? vp_input(vin)
                                 : vp_const(XF_CONST_MATERIAL(chan)),
                vp_none(), vp_none());
    vp_emit_vec(e, VP_VEC_MOV, XR_MAT, 0, VP_MASK_W, 0,
                LIT_MATSRC(calp) ? vp_input(vin)
                                 : vp_const(XF_CONST_MATERIAL(chan)),
                vp_none(), vp_none());

    /* The accumulator starts at the ambient term for a lit channel and at one
     * for an unlit one -- an unlit channel is the material colour unchanged,
     * and expressing that as "accumulate nothing onto a white ambient" keeps
     * one code path instead of two. */
    if (LIT_ENABLE(ccol))
        vp_emit_vec(e, VP_VEC_MOV, XR_ACC, 0, rgbmask, 0,
                    LIT_AMBSRC(ccol) ? vp_input(vin)
                                     : vp_const(XF_CONST_AMBIENT(chan)),
                    vp_none(), vp_none());
    else
        emit_set_one(e, XR_ACC, rgbmask);
    if (LIT_ENABLE(calp))
        vp_emit_vec(e, VP_VEC_MOV, XR_ACC, 0, VP_MASK_W, 0,
                    LIT_AMBSRC(calp) ? vp_input(vin)
                                     : vp_const(XF_CONST_AMBIENT(chan)),
                    vp_none(), vp_none());
    else
        emit_set_one(e, XR_ACC, VP_MASK_W);

    if (LIT_ENABLE(ccol)) {
        unsigned mask = LIT_MASK(ccol);
        for (i = 0; i < 8; i++)
            if (mask & (1u << i)) {
                emit_light(e, i, LIT_ATTN(ccol), LIT_DIFFUSE(ccol), rgbmask, 0);
                *lights_used |= 1u << i;
            }
    }
    if (LIT_ENABLE(calp)) {
        unsigned mask = LIT_MASK(calp);
        for (i = 0; i < 8; i++)
            if (mask & (1u << i)) {
                emit_light(e, i, LIT_ATTN(calp), LIT_DIFFUSE(calp),
                           VP_MASK_W, 1);
                *lights_used |= 1u << i;
            }
    }

    /* out = material * clamp(acc). The saturate flag does the clamp. */
    vp_emit_vec(e, VP_VEC_MOV, XR_ACC, 0, VP_MASK_ALL, 1,
                vp_temp(XR_ACC), vp_none(), vp_none());
    vp_emit_vec(e, VP_VEC_MUL, out, 1, VP_MASK_ALL, 0,
                vp_temp(XR_MAT), vp_temp(XR_ACC), vp_none());
}

/* ------------------------------------------------------------------ */
/* Texture coordinate generation                                        */
/* ------------------------------------------------------------------ */

#define TG_PROJ(v)      BITSX(v, 1, 1)      /* 0 = ST (2 rows), 1 = STQ (3) */
#define TG_INPUTFORM(v) BITSX(v, 2, 1)      /* 0 = AB11, 1 = ABC1           */
#define TG_TYPE(v)      BITSX(v, 4, 3)      /* 0 regular, 1 emboss, 2/3 col */
#define TG_SRCROW(v)    BITSX(v, 7, 5)      /* 0 geom, 1 normal, 5.. tex n  */

/* The projective divide: the sampler is handed s/q and t/q, not s and t.
 *
 * Emitted for every texgen, not only the ones the projection bit calls STQ.
 * The bit selects whether the *texture matrix* has three rows; the third
 * component that ends up dividing can also come from the dual transform, whose
 * matrix is three rows either way. Gating on the bit therefore leaves a
 * dual-transformed ST texgen dividing by a q it should have divided by -- and
 * for a texgen with neither, the generated code sets q to the literal 1 and
 * this is a divide by one, exact and invisible. Dolphin divides unconditionally
 * in the pixel shader for the same reason.
 *
 * Per vertex rather than per fragment, which is the one place this differs
 * from the hardware. The two agree exactly whenever q is constant across a
 * primitive, and they differ by the usual affine-versus-perspective error when
 * it is not. The trade is deliberate: the fragment unit is the one with the
 * register cliff this whole backend is designed around, and an unconditional
 * per-fragment divide would cost two fragment instructions on every texture
 * fetch in the game -- including the 174,014 title-screen fetches whose q is
 * exactly 1. If a reflective surface ever looks subtly skewed, this is the
 * line to revisit; the exact fix is an RCP and a MUL in tev_program.c's fetch.
 *
 * q == 0 divides by one instead, which is what the hardware does and what
 * Dolphin reproduces. SEQ gives that without a branch: it produces 1 exactly
 * when q is zero, and adding it leaves every other q alone. */
static void emit_project(VPEmitter *e, unsigned reg)
{
    vp_emit_vec(e, VP_VEC_SEQ, XR_TMP2, 0, VP_MASK_X, 0,
                vp_swizzle(vp_temp(reg), 2, 2, 2, 2), vp_zero_src(), vp_none());
    vp_emit_vec(e, VP_VEC_ADD, XR_TMP2, 0, VP_MASK_X, 0,
                vp_swizzle(vp_temp(reg), 2, 2, 2, 2), vp_temp(XR_TMP2),
                vp_none());
    vp_emit_sca(e, VP_SCA_RCP, XR_TMP2, 0, VP_MASK_Y, 0,
                vp_swizzle(vp_temp(XR_TMP2), 0, 0, 0, 0));
    vp_emit_vec(e, VP_VEC_MUL, reg, 0, VP_MASK_X | VP_MASK_Y, 0,
                vp_temp(reg), vp_swizzle(vp_temp(XR_TMP2), 1, 1, 1, 1),
                vp_none());
    emit_set_one(e, reg, VP_MASK_Z | VP_MASK_W);
}

static void emit_texgen(VPEmitter *e, const GXState *g, unsigned n,
                        unsigned *post_used, unsigned *uses_normal,
                        unsigned *uses_posmtx)
{
    u32 info = g->xf.mem[0x1040 + n];
    unsigned type = TG_TYPE(info);
    unsigned srcrow = TG_SRCROW(info);
    unsigned rows = TG_PROJ(info) ? 3u : 2u;
    unsigned dual = g->xf.mem[0x1012] & 1u;
    u32 post = g->xf.mem[0x1050 + n];
    unsigned rgbmask = VP_MASK_X | VP_MASK_Y | VP_MASK_Z;

    /* Types 2 and 3 route a *lit colour* into the coordinate: (r, g, 1) of
     * colour channel 0 or 1, which is how a title indexes a ramp texture by
     * its own lighting. The third component has to be the literal 1 rather
     * than the channel's blue, or the divide by q below scales the lookup by
     * a colour.
     *
     * Type 1 is emboss bump mapping, which needs the binormal and the tangent
     * the vertex loader does not deliver; it is deliberately *not* special
     * cased here and falls through to the regular path, which transforms its
     * declared source row and produces a coordinate that is wrong in the same
     * way it was before this file understood texgen types at all. No material
     * in the six courses and two karts read off the disc uses it -- all 612
     * texgens are regular -- so inventing a lowering for it would be
     * untestable code on a path nothing takes. */
    if (type == 2 || type == 3) {
        unsigned src = (type == 2) ? VP_IN_COL0 : VP_IN_COL1;
        vp_emit_vec(e, VP_VEC_MOV, XR_TCRES, 0, VP_MASK_X | VP_MASK_Y, 0,
                    vp_input(src), vp_none(), vp_none());
        emit_set_one(e, XR_TCRES, VP_MASK_Z | VP_MASK_W);
        vp_emit_vec(e, VP_VEC_MOV, VP_OUT_TEX(n), 1, VP_MASK_ALL, 0,
                    vp_temp(XR_TCRES), vp_none(), vp_none());
        return;
    }

    /* The input row. The hardware feeds the matrix a four-component vector
     * whose fourth element is 1; the AB11 input form additionally forces the
     * third to 1, which is what makes a two-component texture coordinate pick
     * up the matrix's translation column. */
    switch (srcrow) {
    case 0:     /* geometry: the raw model-space position */
        vp_emit_vec(e, VP_VEC_MOV, XR_COORD, 0, rgbmask, 0,
                    vp_input(VP_IN_POS), vp_none(), vp_none());
        break;
    case 1:     /* normal -- reflection and environment mapping */
        vp_emit_vec(e, VP_VEC_MOV, XR_COORD, 0, rgbmask, 0,
                    vp_input(VP_IN_NORMAL), vp_none(), vp_none());
        *uses_normal = 1;
        break;
    default: {  /* 5..12: texture coordinate 0..7 */
        unsigned tex = (srcrow >= 5 && srcrow <= 12) ? srcrow - 5u : n;
        vp_emit_vec(e, VP_VEC_MOV, XR_COORD, 0, rgbmask, 0,
                    vp_input(VP_IN_TEX(tex)), vp_none(), vp_none());
        break;
    }
    }
    /* w = 1 always; z = 1 for the AB11 input form. The third component is what
     * multiplies a texture matrix's translation column, so forcing it to 1 is
     * exactly what makes an atlas sub-rectangle offset apply. */
    emit_set_one(e, XR_COORD, VP_MASK_W | (TG_INPUTFORM(info) ? 0u : VP_MASK_Z));

    (void)uses_posmtx;

    /* The texture matrix. Two rows for ST, three for STQ; the unwritten third
     * component must be 1 so that a later divide by q is a no-op. */
    emit_transform3(e, XR_TCRES, vp_temp(XR_COORD), XF_CONST_TEXMTX(n), rows);
    emit_set_one(e, XR_TCRES, VP_MASK_W | (rows == 3 ? 0u : VP_MASK_Z));

    if (!dual) {
        emit_project(e, XR_TCRES);
        vp_emit_vec(e, VP_VEC_MOV, VP_OUT_TEX(n), 1, VP_MASK_ALL, 0,
                    vp_temp(XR_TCRES), vp_none(), vp_none());
        return;
    }

    /* Dual transform. The post matrix is 3 rows of 4 and its fourth column is
     * a translation, so the input's w has to be 1 -- which is why it is set
     * again after the normalisation, which only touches xyz. Normalising first is what turns a transformed normal into a
     * unit vector to index an environment map with; 167 of the 612 texgens
     * measured on the disc ask for it and every one of them sources the
     * normal. */
    if (BITSX(post, 8, 1))
        emit_normalize(e, XR_TCRES, vp_temp(XR_TCRES), XR_TMP);
    emit_set_one(e, XR_TCRES, VP_MASK_W);
    emit_transform3(e, XR_TMP, vp_temp(XR_TCRES), XF_CONST_POSTMTX(n), 3);
    emit_set_one(e, XR_TMP, VP_MASK_W);
    emit_project(e, XR_TMP);
    vp_emit_vec(e, VP_VEC_MOV, VP_OUT_TEX(n), 1, VP_MASK_ALL, 0,
                vp_temp(XR_TMP), vp_none(), vp_none());
    *post_used |= 1u << n;
}

/* ------------------------------------------------------------------ */
/* Fog coordinate                                                       */
/*                                                                      */
/* GX defines the fog distance against the *screen* depth, not against eye z:    */
/*                                                                              */
/*     ze  = A / (B - (zs >> b_shift))                                          */
/*     fog = ze - C                                                             */
/*                                                                              */
/* which is affine in eye-space depth and therefore interpolates exactly, so it  */
/* is computed here and the fragment program only bends and blends it. Deriving  */
/* it from the screen depth rather than from eye z is deliberate: the depth row  */
/* of the combined matrix is the one the console has already agreed with, so the */
/* fog inherits that agreement instead of re-deriving near and far from the      */
/* projection coefficients.                                                     */
/*                                                                              */
/* XF_CONST_FOG carries (-2^-b_shift, B, A, -C).                                */
/* ------------------------------------------------------------------ */

static void emit_fog_coord(VPEmitter *e, const GXState *g)
{
    VPSrc f = vp_const(XF_CONST_FOG);

    /* zc = depth row . position ; wc = w row . position */
    vp_emit_vec(e, VP_VEC_DP4, XR_TMP, 0, VP_MASK_X, 0,
                vp_input(VP_IN_POS), vp_const(XF_CONST_PROJ + 2), vp_none());
    vp_emit_vec(e, VP_VEC_DP4, XR_TMP, 0, VP_MASK_Y, 0,
                vp_input(VP_IN_POS), vp_const(XF_CONST_PROJ + 3), vp_none());
    vp_emit_sca(e, VP_SCA_RCP, XR_TMP, 0, VP_MASK_Z, 0,
                vp_swizzle(vp_temp(XR_TMP), 1, 1, 1, 1));
    vp_emit_vec(e, VP_VEC_MUL, XR_TMP, 0, VP_MASK_X, 0,
                vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0),
                vp_swizzle(vp_temp(XR_TMP), 2, 2, 2, 2), vp_none());  /* zs */

    if (g->bp.fog.projection) {
        /* Orthographic: ze = A * zs, with no reciprocal at all. */
        vp_emit_vec(e, VP_VEC_MAD, VP_OUT_TEX(XF_FOG_TEXCOORD), 1,
                    VP_MASK_ALL, 0,
                    vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0),
                    vp_swizzle(f, 2, 2, 2, 2), vp_swizzle(f, 3, 3, 3, 3));
        return;
    }

    /* Perspective: ze = A / (B - zs * 2^-b_shift), then subtract C. */
    vp_emit_vec(e, VP_VEC_MAD, XR_TMP, 0, VP_MASK_X, 0,
                vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0),
                vp_swizzle(f, 0, 0, 0, 0), vp_swizzle(f, 1, 1, 1, 1));
    vp_emit_sca(e, VP_SCA_RCP, XR_TMP, 0, VP_MASK_X, 0,
                vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0));
    vp_emit_vec(e, VP_VEC_MAD, VP_OUT_TEX(XF_FOG_TEXCOORD), 1, VP_MASK_ALL, 0,
                vp_swizzle(vp_temp(XR_TMP), 0, 0, 0, 0),
                vp_swizzle(f, 2, 2, 2, 2), vp_swizzle(f, 3, 3, 3, 3));
}

/* ------------------------------------------------------------------ */
/* Constant unpacking                                                   */
/*                                                                      */
/* See xf_program.h for why these are here and not in the backend.              */
/* ------------------------------------------------------------------ */

static f32 xf_float(u32 raw)
{
    union { u32 u; f32 f; } cv;
    cv.u = raw;
    return cv.f;
}

void xf_light_constants(const XFState *xf, unsigned light, f32 out[20])
{
    const u32 *raw = &xf->mem[XF_LIGHT_BASE + (light & 7u) * 16u];
    unsigned k;

    /* The colour is packed ABGR -- the *opposite* of the material and ambient
     * registers below, which are RGBA. This is the single most consequential
     * byte order in this file, and the reason it is here rather than inline in
     * the backend is that here it can be tested. */
    out[0] = (f32)( raw[3]        & 0xFFu) / 255.0f;
    out[1] = (f32)((raw[3] >>  8) & 0xFFu) / 255.0f;
    out[2] = (f32)((raw[3] >> 16) & 0xFFu) / 255.0f;
    out[3] = (f32)((raw[3] >> 24) & 0xFFu) / 255.0f;

    for (k = 0; k < 3; k++) out[4 + k]  = xf_float(raw[4 + k]);   /* cosatt  */
    out[7] = 0.0f;
    for (k = 0; k < 3; k++) out[8 + k]  = xf_float(raw[7 + k]);   /* distatt */
    /* An all-zero distance attenuation would divide the spot term by zero.
     * The hardware does not, so the constant term is nudged rather than the
     * shader being made to test for it every vertex. */
    if (out[8] > -1e-5f && out[8] < 1e-5f &&
        out[9] > -1e-5f && out[9] < 1e-5f &&
        out[10] > -1e-5f && out[10] < 1e-5f)
        out[8] = 1e-5f;
    out[11] = 0.0f;
    for (k = 0; k < 3; k++) out[12 + k] = xf_float(raw[10 + k]);  /* position */
    out[15] = 1.0f;
    for (k = 0; k < 3; k++) out[16 + k] = xf_float(raw[13 + k]);  /* direction */
    out[19] = 0.0f;

    /* The spot axis is normalised once per draw rather than once per vertex:
     * it is a constant of the draw, and doing it in the program would be three
     * instructions per light per vertex to compute the same number. */
    {
        f32 n2 = out[16] * out[16] + out[17] * out[17] + out[18] * out[18];
        if (n2 > 1e-12f) {
            f32 inv = 1.0f / (f32)sqrt((double)n2);
            out[16] *= inv; out[17] *= inv; out[18] *= inv;
        }
    }
}

void xf_material_constants(const XFState *xf, f32 material[8], f32 ambient[8])
{
    unsigned c;
    for (c = 0; c < 2; c++) {
        u32 m = xf->mem[0x100C + c];        /* material: RGBA, top byte first */
        u32 a = xf->mem[0x100A + c];        /* ambient:  RGBA                 */
        material[c * 4 + 0] = (f32)((m >> 24) & 0xFFu) / 255.0f;
        material[c * 4 + 1] = (f32)((m >> 16) & 0xFFu) / 255.0f;
        material[c * 4 + 2] = (f32)((m >>  8) & 0xFFu) / 255.0f;
        material[c * 4 + 3] = (f32)( m        & 0xFFu) / 255.0f;
        ambient[c * 4 + 0]  = (f32)((a >> 24) & 0xFFu) / 255.0f;
        ambient[c * 4 + 1]  = (f32)((a >> 16) & 0xFFu) / 255.0f;
        ambient[c * 4 + 2]  = (f32)((a >>  8) & 0xFFu) / 255.0f;
        ambient[c * 4 + 3]  = (f32)( a        & 0xFFu) / 255.0f;
    }
}

void xf_normal_matrix(const XFState *xf, unsigned posmtx_idx, f32 out[12])
{
    /* Three rows of *three* floats, packed without padding -- unlike every
     * other matrix in XF memory, and indexed by the position matrix's index
     * masked to 32 rather than by an index of its own. Reading it as rows of
     * four gives row 1 a mixture of rows 1 and 2, which tilts every normal by
     * an amount that reads as a lighting bug rather than a layout one. */
    const u32 *raw = &xf->mem[0x0400u + 3u * (posmtx_idx & 31u)];
    unsigned r, c;
    for (r = 0; r < 3; r++) {
        for (c = 0; c < 3; c++)
            out[r * 4 + c] = xf_float(raw[r * 3 + c]);
        out[r * 4 + 3] = 0.0f;
    }
}

void xf_fog_constants(const BPState *bp, f32 out[4])
{
    /* (-2^-b_shift, B, A, -C). B arrives as a 24-bit integer magnitude and the
     * shift as an exponent, so the normalisation happens once per draw here
     * rather than as a divide per vertex in the program. */
    out[0] = -1.0f / (f32)(1u << (bp->fog.b_shift & 31u));
    out[1] = (f32)bp->fog.b_magnitude / 16777216.0f;
    out[2] = bp->fog.a;
    out[3] = -bp->fog.c;
}

/* ------------------------------------------------------------------ */

int xf_generate(const GXState *g, VPEmitter *e, XFProgramInfo *info)
{
    unsigned texgens = g->xf.num_texgens;
    unsigned colors  = g->xf.num_colorchans;
    unsigned n;
    u32 before = e->used;
    int lighting_on = (g_gx_state_mask & GX_STATE_LIGHTING) != 0;
    int texgen_on   = (g_gx_state_mask & GX_STATE_TEXGEN) != 0;
    int fog_on      = (g_gx_state_mask & GX_STATE_FOG) != 0 && g->bp.fog.fsel;
    unsigned lights_used = 0, post_used = 0, uses_normal = 0, uses_posmtx = 0;
    unsigned uses_channels = 0;

    if (info)
        memset(info, 0, sizeof *info);

    if (texgens > GX_NUM_TEXCOORD)
        texgens = GX_NUM_TEXCOORD;
    if (colors > 2)
        colors = 2;

    /* Position: the only thing every vertex program must do. */
    emit_transform(e, VP_OUT_POS, 1, vp_input(VP_IN_POS), XF_CONST_PROJ);

    /* Colour channels.
     *
     * COL0 is written whatever the channel count says. A vertex program that
     * leaves an output alone does not leave the interpolant alone: the
     * rasteriser hands the fragment stage whatever was last routed there, and
     * a TEV stage reading the rasterised colour then reads the previous draw's
     * material. Three quarters of the draws on the Mario Kart Wii title screen
     * configure zero colour channels, so this was a live hazard rather than a
     * theoretical one. */
    if (lighting_on && colors >= 1) {
        /* View-space position and normal, which every light needs. Emitted
         * once per program rather than once per light. */
        int any_lit = 0;
        for (n = 0; n < 2; n++)
            if (LIT_ENABLE(g->xf.mem[0x100E + n]) ||
                LIT_ENABLE(g->xf.mem[0x1010 + n]))
                any_lit = 1;
        if (any_lit) {
            emit_transform3(e, XR_POS, vp_input(VP_IN_POS),
                            XF_CONST_POSMTX, 3);
            emit_transform3(e, XR_NRM, vp_input(VP_IN_NORMAL),
                            XF_CONST_NRMMTX, 3);
            emit_normalize(e, XR_NRM, vp_temp(XR_NRM), XR_TMP);
            uses_normal = 1;
            uses_posmtx = 1;
        }
        emit_channel(e, g, 0, &lights_used);
        if (colors >= 2)
            emit_channel(e, g, 1, &lights_used);
        uses_channels = 1;
    } else {
        /* No colour channel configured -- or the group switched off. The
         * channel-control registers still hold whatever the last material
         * left in them, and evaluating them would take the material colour
         * from a register the title has no reason to have written. 126,733 of
         * the 174,282 draws in a Mario Kart Wii boot configure zero colour
         * channels, so this is the common case and not a corner. */
        /* ZERO, not the vertex colour. With numChans == 0 the hardware
         * rasterises zero (Dolphin does the same), and titles depend on it:
         * MKWii's title-screen pattern quad draws with SRCALPHA blending, no
         * colour channel and no colour array -- srcA must come out 0 so the
         * quad is invisible. Passing the loader's 1.0 fallback instead gave
         * srcA=1 and painted the pattern at full amplitude: the horizontal
         * bars. */
        if (colors == 0) {
            /* numChans == 0 rasterises ZERO on hardware (Dolphin agrees), and
             * titles depend on it: MKWii's title pattern quad uses SRCALPHA
             * blending with no channel and no colour array -- srcA must be 0
             * so the quad is invisible. The 1.0 fallback painted it at full
             * amplitude: the horizontal bars. */
            vp_emit_vec(e, VP_VEC_MOV, VP_OUT_COL0, 1, VP_MASK_ALL, 0,
                        vp_zero_src(), vp_none(), vp_none());
        } else {
            /* Channels exist but the lighting group is off: pass the vertex
             * colour through, which is the debug-mask semantic the tests pin. */
            vp_emit_vec(e, VP_VEC_MOV, VP_OUT_COL0, 1, VP_MASK_ALL, 0,
                        vp_input(VP_IN_COL0), vp_none(), vp_none());
        }
        if (colors >= 2)
            vp_emit_vec(e, VP_VEC_MOV, VP_OUT_COL1, 1, VP_MASK_ALL, 0,
                        vp_input(VP_IN_COL1), vp_none(), vp_none());
    }

    /* Texture coordinates. */
    for (n = 0; n < texgens; n++) {
        if (texgen_on)
            emit_texgen(e, g, n, &post_used, &uses_normal, &uses_posmtx);
        else
            emit_transform(e, VP_OUT_TEX(n), 1, vp_input(VP_IN_TEX(n)),
                           XF_CONST_TEXMTX(n));
    }

    if (fog_on)
        emit_fog_coord(e, g);

    vp_finish(e);

    if (info) {
        info->words = e->used - before;
        info->instructions = info->words / 4;
        info->input_mask = e->input_mask;
        info->output_mask = e->output_mask;
        info->truncated = e->overflow;
        info->uses_normal = uses_normal;
        info->uses_posmtx = uses_posmtx;
        info->texgens = texgens;
        info->lights_used = lights_used;
        info->uses_channels = uses_channels;
        info->post_used = post_used;
        info->fog = fog_on ? 1u : 0u;
    }
    return e->overflow ? -1 : 0;
}

/* ------------------------------------------------------------------ */

u64 xf_state_hash(const GXState *g)
{
    u64 h = 1469598103934665603ull;

    #define MIX(v) do { h ^= (u64)(v); h *= 1099511628211ull; } while (0)

    /* Only the shape. The projection and texture matrices live in XF memory and
     * change constantly -- per object, sometimes per frame -- and they are
     * uploaded as constants rather than compiled in. Hashing them would
     * recompile the program every time anything moved, which is the exact
     * failure the cache exists to prevent. */
    MIX(g->xf.num_texgens);
    MIX(g->xf.num_colorchans);
    MIX(g->xf.projection_orthographic);

    /* The vertex descriptor decides which inputs exist, so a format change is a
     * different program even at the same texgen count. */
    MIX(g->parser.cp.vcd_lo);
    MIX(g->parser.cp.vcd_hi);

    /* Texgen configuration words: which row feeds the coordinate, whether the
     * matrix is 2 rows or 3, and whether a post transform follows. All of it
     * is generated code, none of it is a constant. */
    if (g_gx_state_mask & GX_STATE_TEXGEN) {
        unsigned n;
        MIX(g->xf.mem[0x1012]);                 /* dual transform enable */
        for (n = 0; n < g->xf.num_texgens && n < GX_NUM_TEXCOORD; n++) {
            MIX(g->xf.mem[0x1040 + n]);
            MIX(g->xf.mem[0x1050 + n] & 0x1FFu);   /* index and normalize */
        }
    }

    /* Lighting: the four channel-control words are the program's shape --
     * which lights, which functions, where the material comes from. The light
     * *data* is constants and deliberately is not hashed. */
    if ((g_gx_state_mask & GX_STATE_LIGHTING) && g->xf.num_colorchans >= 1) {
        MIX(g->xf.mem[0x100E]); MIX(g->xf.mem[0x100F]);
        MIX(g->xf.mem[0x1010]); MIX(g->xf.mem[0x1011]);
    }

    /* Fog: only whether a coordinate is generated at all, and by which of the
     * two formulas. A and B and C are constants. */
    if (g_gx_state_mask & GX_STATE_FOG) {
        MIX(g->bp.fog.fsel ? 1u : 0u);
        MIX(g->bp.fog.projection);
    }

    #undef MIX
    return h;
}
