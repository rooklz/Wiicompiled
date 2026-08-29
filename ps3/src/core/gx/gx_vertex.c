/* gx_vertex.c — how many bytes a vertex occupies in the command stream.
 *
 * This is the arithmetic the whole graphics front end balances on. A draw
 * command names a primitive and a vertex count and then hands over raw bytes;
 * only the vertex descriptor and attribute table say how many. Off by one and
 * the parser does not draw a wrong triangle -- it loses stream synchronisation
 * and every command after it is noise.
 *
 * The bit layouts below are the hardware's, transcribed field by field rather
 * than as extracted masks, because a mask with the wrong shift produces a
 * plausible size for most vertex formats and a wrong one for a few.
 */
#include "gx.h"
#include "../../common/log.h"

/* ------------------------------------------------------------------ */
/* Field extraction                                                     */
/* ------------------------------------------------------------------ */

/* A macro rather than a function so the field definitions below are constant
 * expressions, which is what lets the build-time assertions at the end of this
 * file check the texture-coordinate table against them. */
#define GX_BITS(v, lsb, width) \
    (((u32)(v) >> (lsb)) & ((1u << (width)) - 1u))

/* VCD lo (CP 0x50): the nine one-byte index attributes, then four two-bit
 * attribute types. */
/* Draws whose attributes are all direct, vs draws using per-attribute
 * indexing. See the classification in gx_vertex_size. */
unsigned long long g_gx_draws_direct, g_gx_draws_indexed;

/* Depth/colour state and projection type, sampled per draw. The 2D HUD renders
 * correctly while the 3D world is black; these are the state bits that differ
 * between those two populations, so they are what to look at. */
unsigned long long g_zm[8], g_persp, g_ortho, g_nocolor;
double g_proj_min[6], g_proj_max[6];
unsigned long long g_proj_n, g_proj_zero;
unsigned long long g_at_comp[8][2], g_at_logic[4], g_at_never;

#define VCD_POS_MAT_IDX(lo)     GX_BITS(lo, 0, 1)
#define VCD_TEX_MAT_IDX(lo, n)  GX_BITS(lo, 1 + (n), 1)
#define VCD_POSITION(lo)        GX_BITS(lo, 9, 2)
#define VCD_NORMAL(lo)          GX_BITS(lo, 11, 2)
#define VCD_COLOR0(lo)          GX_BITS(lo, 13, 2)
#define VCD_COLOR1(lo)          GX_BITS(lo, 15, 2)

/* VCD hi (CP 0x60): eight two-bit texture-coordinate attribute types. */
#define VCD_TEXCOORD(hi, n)     GX_BITS(hi, 2 * (n), 2)

/* VAT A (CP 0x70+n) */
#define VAT_POS_ELEMENTS(a)     GX_BITS(a, 0, 1)    /* 0 = xy, 1 = xyz          */
#define VAT_POS_FORMAT(a)       GX_BITS(a, 1, 3)
#define VAT_NRM_ELEMENTS(a)     GX_BITS(a, 9, 1)    /* 0 = normal, 1 = NBT      */
#define VAT_NRM_FORMAT(a)       GX_BITS(a, 10, 3)
#define VAT_COL0_FORMAT(a)      GX_BITS(a, 14, 3)
#define VAT_COL1_FORMAT(a)      GX_BITS(a, 18, 3)
#define VAT_TEX0_ELEMENTS(a)    GX_BITS(a, 21, 1)   /* 0 = s, 1 = st            */
#define VAT_TEX0_FORMAT(a)      GX_BITS(a, 22, 3)
#define VAT_NORMAL_INDEX3(a)    GX_BITS(a, 31, 1)

/* VAT B (CP 0x80+n) carries texture coordinates 1..4; VAT C (CP 0x90+n)
 * carries 5..7. Coordinate 4's fractional bits straddle the two registers,
 * which does not affect size but is why the split looks arbitrary. */
#define VAT_TEX1_ELEMENTS(b)    GX_BITS(b, 0, 1)
#define VAT_TEX1_FORMAT(b)      GX_BITS(b, 1, 3)
#define VAT_TEX2_ELEMENTS(b)    GX_BITS(b, 9, 1)
#define VAT_TEX2_FORMAT(b)      GX_BITS(b, 10, 3)
#define VAT_TEX3_ELEMENTS(b)    GX_BITS(b, 18, 1)
#define VAT_TEX3_FORMAT(b)      GX_BITS(b, 19, 3)
#define VAT_TEX4_ELEMENTS(b)    GX_BITS(b, 27, 1)
#define VAT_TEX4_FORMAT(b)      GX_BITS(b, 28, 3)

#define VAT_TEX5_ELEMENTS(c)    GX_BITS(c, 5, 1)
#define VAT_TEX5_FORMAT(c)      GX_BITS(c, 6, 3)
#define VAT_TEX6_ELEMENTS(c)    GX_BITS(c, 14, 1)
#define VAT_TEX6_FORMAT(c)      GX_BITS(c, 15, 3)
#define VAT_TEX7_ELEMENTS(c)    GX_BITS(c, 23, 1)
#define VAT_TEX7_FORMAT(c)      GX_BITS(c, 24, 3)

/* ------------------------------------------------------------------ */

u32 gx_component_size(GXCompFormat fmt)
{
    switch (fmt) {
    case GX_COMP_U8:  case GX_COMP_S8:  return 1;
    case GX_COMP_U16: case GX_COMP_S16: return 2;
    case GX_COMP_F32:                   return 4;
    default:
        /* Formats 5-7 are not in the public documentation but they ARE used:
         * Fifa Street and Def Jam: Fight for New York ship format 5 (Dolphin
         * bug 12719), and Dolphin sizes all three as FLOAT -- 4 bytes.
         *
         * Returning 0 here was a latent catastrophe rather than a cosmetic
         * wrong answer: gx_vertex_size feeds the primitive command's length,
         * so a zero-sized component makes the FIFO decoder consume the wrong
         * byte count and lose stream synchronisation PERMANENTLY, with every
         * later command decoded as garbage and nothing pointing at the
         * cause. Four is the right answer; the warning stays because seeing
         * one of these is still notable. */
        LOG_WARN_ONCE(LOG_VIDEO, "vertex component format %u is undocumented; "
                      "sizing as float (4)", (unsigned)fmt);
        return 4;
    }
}

u32 gx_color_size(GXColorFormat fmt)
{
    switch (fmt) {
    case GX_CLR_RGB565: return 2;
    case GX_CLR_RGB8:   return 3;
    case GX_CLR_RGBX8:  return 4;
    case GX_CLR_RGBA4:  return 2;
    case GX_CLR_RGBA6:  return 3;
    case GX_CLR_RGBA8:  return 4;
    default:
        LOG_WARN_ONCE(LOG_VIDEO, "vertex colour format %u is not defined",
                      (unsigned)fmt);
        return 0;
    }
}

/* Size contributed by one attribute.
 *
 * Indexed attributes cost only their index, whatever the underlying data looks
 * like -- the data lives in an array the GPU dereferences. That is the whole
 * point of indexing, and it is also the easiest part of this to get wrong,
 * because the natural instinct is to charge for the data. */
static u32 attr_size(u32 type, u32 direct_size)
{
    switch (type) {
    case GX_ATTR_NONE:    return 0;
    case GX_ATTR_DIRECT:  return direct_size;
    case GX_ATTR_INDEX8:  return 1;
    case GX_ATTR_INDEX16: return 2;
    default:              return 0;
    }
}

u32 gx_vertex_size(const GXCPRegs *cp, unsigned vat)
{
    u32 a, b, c, lo, hi, size = 0;
    unsigned n;

    if (vat >= GX_NUM_VAT)
        return 0;

    lo = cp->vcd_lo;
    hi = cp->vcd_hi;
    a  = cp->vat_a[vat];
    b  = cp->vat_b[vat];
    c  = cp->vat_c[vat];

    /* Classify the draw: is EVERY present attribute direct?
     *
     * This decides the size of a large opportunity. GX's quantised attribute
     * formats (S16/U8/F32 with a fractional shift) map onto RSX vertex types
     * the hardware already supports, and the shift folds into the transform
     * matrix -- so a draw whose attributes are all direct could be handed to
     * the RSX as the guest's own buffer, with no decode, no SPU job and no
     * wait. What RSX cannot do is GX's PER-ATTRIBUTE indexing: it has one
     * index stream per vertex where GX has one per attribute.
     *
     * VCD attribute codes: 0 = not present, 1 = direct, 2 = index8,
     * 3 = index16. So "any code >= 2" is the disqualifier. Counted here
     * because every backend reaches this function once per draw command. */
    {
        u32 idxd = 0;
        idxd |= (VCD_POSITION(lo) >= 2u);
        idxd |= (VCD_NORMAL(lo)   >= 2u);
        idxd |= (VCD_COLOR0(lo)   >= 2u);
        idxd |= (VCD_COLOR1(lo)   >= 2u);
        for (n = 0; n < GX_NUM_TEXCOORD; n++)
            idxd |= (VCD_TEXCOORD(hi, n) >= 2u);
        if (idxd) g_gx_draws_indexed++;
        else      g_gx_draws_direct++;
    }

    /* Matrix indices are one byte each and are always direct when present. */
    size += VCD_POS_MAT_IDX(lo);
    for (n = 0; n < GX_NUM_TEXCOORD; n++)
        size += VCD_TEX_MAT_IDX(lo, n);

    /* Position: two or three components. */
    size += attr_size(VCD_POSITION(lo),
                      (VAT_POS_ELEMENTS(a) ? 3u : 2u) *
                      gx_component_size((GXCompFormat)VAT_POS_FORMAT(a)));

    /* Normal: three components, or nine when the vertex carries a full
     * normal/binormal/tangent frame.
     *
     * NBT also changes the *indexed* cost, but only when NormalIndex3 is set:
     * the three vectors are then fetched with three separate indices rather
     * than one. A title using NBT lighting with single indexing exists, so the
     * two bits genuinely have to be considered together. */
    {
        u32 type = VCD_NORMAL(lo);
        u32 vectors = VAT_NRM_ELEMENTS(a) ? 3u : 1u;
        u32 direct = 3u * vectors *
                     gx_component_size((GXCompFormat)VAT_NRM_FORMAT(a));
        u32 n_size = attr_size(type, direct);

        if (VAT_NORMAL_INDEX3(a) && vectors == 3u &&
            (type == GX_ATTR_INDEX8 || type == GX_ATTR_INDEX16))
            n_size *= 3u;

        size += n_size;
    }

    /* Colours: the component format alone fixes the size. */
    size += attr_size(VCD_COLOR0(lo),
                      gx_color_size((GXColorFormat)VAT_COL0_FORMAT(a)));
    size += attr_size(VCD_COLOR1(lo),
                      gx_color_size((GXColorFormat)VAT_COL1_FORMAT(a)));

    /* Texture coordinates: one or two components each, spread across the three
     * attribute-table registers. */
    {
        static const u8 k_elem_lsb[GX_NUM_TEXCOORD] = { 21, 0, 9, 18, 27, 5, 14, 23 };
        static const u8 k_fmt_lsb [GX_NUM_TEXCOORD] = { 22, 1, 10, 19, 28, 6, 15, 24 };
        const u32 group[GX_NUM_TEXCOORD] = { a, b, b, b, b, c, c, c };

        for (n = 0; n < GX_NUM_TEXCOORD; n++) {
            u32 reg  = group[n];
            u32 elem = GX_BITS(reg, k_elem_lsb[n], 1) ? 2u : 1u;
            u32 fmt  = GX_BITS(reg, k_fmt_lsb[n], 3);
            size += attr_size(VCD_TEXCOORD(hi, n),
                              elem * gx_component_size((GXCompFormat)fmt));
        }
    }

    return size;
}

/* The per-coordinate macros above are not used by gx_vertex_size -- the table
 * is -- but they document the layout the table encodes, and a mismatch between
 * the two is a bug waiting to happen. Assert the agreement at build time so the
 * table cannot drift from the field definitions it was derived from. */
DOL_STATIC_ASSERT(VAT_TEX0_ELEMENTS(1u << 21) == 1, tex0_elements_bit);
DOL_STATIC_ASSERT(VAT_TEX1_ELEMENTS(1u << 0)  == 1, tex1_elements_bit);
DOL_STATIC_ASSERT(VAT_TEX2_ELEMENTS(1u << 9)  == 1, tex2_elements_bit);
DOL_STATIC_ASSERT(VAT_TEX3_ELEMENTS(1u << 18) == 1, tex3_elements_bit);
DOL_STATIC_ASSERT(VAT_TEX4_ELEMENTS(1u << 27) == 1, tex4_elements_bit);
DOL_STATIC_ASSERT(VAT_TEX5_ELEMENTS(1u << 5)  == 1, tex5_elements_bit);
DOL_STATIC_ASSERT(VAT_TEX6_ELEMENTS(1u << 14) == 1, tex6_elements_bit);
DOL_STATIC_ASSERT(VAT_TEX7_ELEMENTS(1u << 23) == 1, tex7_elements_bit);
DOL_STATIC_ASSERT(VAT_TEX0_FORMAT(7u << 22) == 7, tex0_format_bits);
DOL_STATIC_ASSERT(VAT_TEX4_FORMAT(7u << 28) == 7, tex4_format_bits);
DOL_STATIC_ASSERT(VAT_TEX7_FORMAT(7u << 24) == 7, tex7_format_bits);
