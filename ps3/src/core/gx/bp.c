/* bp.c — pixel engine register decode.
 *
 * See bp.h for why every register is stored raw and only some are decoded.
 *
 * The bit positions below are transcribed from the hardware's field layout one
 * field at a time. That is deliberate: BP registers pack four to nine fields
 * into 24 bits, and a mask with the right width but the wrong shift produces
 * values that are individually plausible — a cull mode of 1 instead of 2, a
 * blend factor of 4 instead of 5 — so the resulting image is wrong rather than
 * absent, and nothing points back here.
 */
#include "bp.h"
#include "../../common/log.h"

#include <string.h>

/* The pixel engine is signalled through hooks rather than called directly, so
 * this parser stays usable on its own (the shader tests link it without any
 * hardware). hw/pe.c installs them at init. */
static void (*s_pe_finish)(void);
static void (*s_pe_token)(u16 token, int with_interrupt);

void bp_set_pe_hooks(void (*finish)(void),
                     void (*token)(u16 token, int with_interrupt))
{
    s_pe_finish = finish;
    s_pe_token  = token;
}


#define BITS(v, lsb, width) (((u32)(v) >> (lsb)) & ((1u << (width)) - 1u))

/* ------------------------------------------------------------------ */

void bp_reset(BPState *bp)
{
    memset(bp, 0, sizeof *bp);

    /* Out of reset the pipeline draws nothing useful, but the fields a renderer
     * reads before the first draw must still be coherent. One TEV stage is the
     * minimum the hardware can be configured for -- zero stages is not a legal
     * state, and a renderer that loops `for (i = 0; i < stages; i++)` on a
     * zeroed struct would silently produce untextured geometry. */
    bp->genmode.num_tev_stages = 1;
    bp->zmode.update_enable = 1;
    bp->blend.color_update = 1;
    bp->blend.alpha_update = 1;

    /* The identity swap table. A zeroed table would route every output channel
     * to the input's red, so a title that never writes KSEL -- and the whole
     * boot sequence up to the first material does not -- would get greyscale
     * textures out of a decoder that is working perfectly. */
    {
        unsigned t, c;
        for (t = 0; t < 4; t++)
            for (c = 0; c < 4; c++)
                bp->tev_swap[t][c] = c;
    }
}

unsigned bp_tev_stage_count(const BPState *bp)
{
    return bp->genmode.num_tev_stages;
}

/* ------------------------------------------------------------------ */
/* Individual registers                                                 */
/* ------------------------------------------------------------------ */

static void decode_genmode(BPState *bp, u32 v)
{
    /* Three of these five counts are stored as "count - 1" and two are not,
     * which is exactly the kind of inconsistency that has to be written down
     * rather than remembered. The colour-channel count is three bits, not
     * five: bits 7 and 8 are an unused bit and the flat-shading flag, and
     * reading them as part of the count turned "one colour channel, flat
     * shaded" into "seventeen colour channels". Nothing consumed the value
     * except a cache key, so it cost cache entries rather than pixels -- but
     * a count that can read 17 is a trap for the first consumer that trusts
     * it. */
    bp->genmode.num_texgens    = BITS(v, 0, 4);        /* plain count      */
    bp->genmode.num_colorchans = BITS(v, 4, 3);        /* plain count      */
    bp->genmode.num_tev_stages = BITS(v, 10, 4) + 1u;  /* stored count - 1 */
    bp->genmode.cull           = (BPCullMode)BITS(v, 14, 2);
    bp->genmode.num_indstages  = BITS(v, 16, 3);       /* plain count      */
    bp->genmode.zfreeze        = BITS(v, 19, 1);
}

static void decode_zmode(BPState *bp, u32 v)
{
    bp->zmode.enable        = BITS(v, 0, 1);
    bp->zmode.func          = BITS(v, 1, 3);
    bp->zmode.update_enable = BITS(v, 4, 1);
}

/* Distinct values written to PE_CMODE0, for diagnosis. Colour writes are
 * coming out disabled on essentially every perspective draw, which renders the
 * 3D world invisible while the orthographic HUD draws fine. Either the title
 * really writes that, or the value reaching here is not the one it wrote. */
unsigned long long g_cmode0_vals[8], g_cmode0_n;

static void decode_blend(BPState *bp, u32 v)
{
    {   unsigned q; int seen = 0;
        for (q = 0; q < g_cmode0_n && q < 8; q++)
            if (g_cmode0_vals[q] == v) { seen = 1; break; }
        if (!seen && g_cmode0_n < 8) g_cmode0_vals[g_cmode0_n++] = v;
    }
    bp->blend.blend_enable = BITS(v, 0, 1);
    bp->blend.logic_enable = BITS(v, 1, 1);
    bp->blend.dither       = BITS(v, 2, 1);
    bp->blend.color_update = BITS(v, 3, 1);
    bp->blend.alpha_update = BITS(v, 4, 1);
    bp->blend.dst_factor   = BITS(v, 5, 3);
    bp->blend.src_factor   = BITS(v, 8, 3);
    bp->blend.subtract     = BITS(v, 11, 1);
    bp->blend.logic_op     = BITS(v, 12, 4);
}

static void decode_alpha_test(BPState *bp, u32 v)
{
    bp->alpha_test.ref0  = BITS(v, 0, 8);
    bp->alpha_test.ref1  = BITS(v, 8, 8);
    bp->alpha_test.comp0 = BITS(v, 16, 3);
    bp->alpha_test.comp1 = BITS(v, 19, 3);
    bp->alpha_test.logic = BITS(v, 22, 2);
}

/* TEV "order" registers pack two stages each: 0x28 covers stages 0 and 1, 0x29
 * covers 2 and 3, and so on. The even stage is in the low 12 bits. */
static void decode_tev_order(BPState *bp, unsigned pair, u32 v)
{
    unsigned s0 = pair * 2, s1 = s0 + 1;
    if (s1 >= BP_MAX_TEV_STAGES)
        return;

    bp->tev[s0].tex_map     = BITS(v, 0, 3);
    bp->tev[s0].tex_coord   = BITS(v, 3, 3);
    bp->tev[s0].tex_enable  = BITS(v, 6, 1);
    bp->tev[s0].ras_channel = BITS(v, 7, 3);

    bp->tev[s1].tex_map     = BITS(v, 12, 3);
    bp->tev[s1].tex_coord   = BITS(v, 15, 3);
    bp->tev[s1].tex_enable  = BITS(v, 18, 1);
    bp->tev[s1].ras_channel = BITS(v, 19, 3);
}

/* ------------------------------------------------------------------ */
/* Indirect texturing                                                   */
/*                                                                      */
/* Four registers' worth of state, none of which was decoded until the           */
/* fragment-program generator learned to act on it. All of it is small and       */
/* bit-packed, and three of the four have a field that is easy to lose:          */
/* the matrix's shared exponent is split two bits per register, the command      */
/* has two adjacent two-bit fields that mean different things, and the           */
/* coordinate scale packs two indirect stages into one register.                 */
/* ------------------------------------------------------------------ */

/* One TEV stage's indirect command, BP 0x10 + n. */
static void decode_tevind(BPState *bp, unsigned stage, u32 v)
{
    BPTevIndirect *ti;
    if (stage >= BP_MAX_TEV_STAGES)
        return;
    ti = &bp->tevind[stage];
    ti->raw            = v;
    ti->ind_stage      = BITS(v, 0, 2);
    ti->format         = BITS(v, 2, 2);
    ti->bias           = BITS(v, 4, 3);
    ti->bump_alpha     = BITS(v, 7, 2);
    ti->matrix_index   = BITS(v, 9, 2);
    ti->matrix_id      = BITS(v, 11, 2);
    ti->wrap_s         = BITS(v, 13, 3);
    ti->wrap_t         = BITS(v, 16, 3);
    ti->lod_unmodified = BITS(v, 19, 1);
    ti->add_prev       = BITS(v, 20, 1);
}

/* One column of one indirect matrix, BP 0x06 + 3m + c.
 *
 * "Column" is the hardware's word and it is the confusing one: a register
 * carries one column of a 2x3 matrix, i.e. one element of each ROW. So
 * register 0x06 holds m[0][0] and m[1][0], not the first row.
 *
 * Each register also carries two bits of the matrix's shared scale exponent,
 * in ascending significance across the three registers. Dolphin's notes record
 * that hardware ignores the sixth bit even though the SDK writes it, which is
 * why only five are kept here -- an exponent read two bits too wide scales
 * every offset by a factor of a million and produces a texture lookup
 * somewhere else entirely. */
static void decode_ind_mtx(BPState *bp, unsigned reg, u32 v)
{
    unsigned m = reg / 3u, col = reg % 3u;
    int a, b;
    if (m >= 3)
        return;

    /* 11-bit two's complement, ten fractional bits. */
    a = (int)BITS(v, 0, 11);  if (a & 0x400) a -= 0x800;
    b = (int)BITS(v, 11, 11); if (b & 0x400) b -= 0x800;
    bp->ind_mtx[m].m[0][col] = a;
    bp->ind_mtx[m].m[1][col] = b;

    {
        unsigned bits = BITS(v, 22, col == 2 ? 1u : 2u);
        unsigned shift = col * 2u;
        unsigned width = (col == 2) ? 1u : 2u;
        unsigned mask  = ((1u << width) - 1u) << shift;
        bp->ind_mtx[m].scale = (bp->ind_mtx[m].scale & ~mask) | (bits << shift);
    }
}

/* RAS1_IREF (0x27): which texture map and which texture coordinate each of the
 * four indirect lookups uses, three bits each. */
static void decode_ind_ref(BPState *bp, u32 v)
{
    unsigned i;
    for (i = 0; i < 4; i++) {
        bp->ind_stage[i].map   = BITS(v, i * 6u, 3);
        bp->ind_stage[i].coord = BITS(v, i * 6u + 3u, 3);
    }
}

/* RAS1_SS0/SS1 (0x25, 0x26): the indirect lookup's coordinate is divided by
 * 2^scale before the fetch. Two indirect stages per register. */
static void decode_ind_scale(BPState *bp, unsigned which, u32 v)
{
    unsigned i = which * 2u;
    bp->ind_stage[i].scale_s     = BITS(v, 0, 4);
    bp->ind_stage[i].scale_t     = BITS(v, 4, 4);
    bp->ind_stage[i + 1].scale_s = BITS(v, 8, 4);
    bp->ind_stage[i + 1].scale_t = BITS(v, 12, 4);
}

/* Texture registers come in two banks of four, at 0x80 and 0xA0, covering
 * textures 0-3 and 4-7. Mapping both banks through one function is what keeps
 * the second bank from being quietly forgotten -- it is the same hardware. */
static void decode_texture(BPState *bp, unsigned unit, unsigned which, u32 v)
{
    BPTexture *t;
    if (unit >= BP_NUM_TEXTURES)
        return;
    t = &bp->tex[unit];

    switch (which) {
    case 0: t->mode0 = v; break;
    case 1: t->mode1 = v; break;
    case 2:
        t->image0 = v;
        /* Dimensions are stored 1-based minus one, i.e. a 64-pixel texture
         * stores 63. */
        t->width  = BITS(v, 0, 10) + 1u;
        t->height = BITS(v, 10, 10) + 1u;
        t->format = BITS(v, 20, 4);
        break;
    case 3: t->image1 = v; break;   /* TMEM layout: even/odd cache lines   */
    case 4: t->image2 = v; break;
    case 5:
        t->image3 = v;
        /* The address register holds a 24-bit value in 32-byte units, which is
         * how a 21-bit field addresses all of MEM1. */
        t->address = (v & 0x00FFFFFFu) << 5;
        break;
    case 6: t->tlut = v; break;
    default: break;
    }
}

/* KSEL: konst selection for two stages, and one row of one swap table.
 *
 * The register does two unrelated jobs at once, which is the trap. Bits 0..3
 * are a *swap table* entry -- register 0xF6+2t carries table t's red and green
 * sources, 0xF6+2t+1 carries its blue and alpha -- while bits 4..23 are the
 * konst selections for TEV stages 2n and 2n+1. A decoder that treats the
 * register as "the konst selectors for a stage pair" silently drops the swap
 * tables, and one that treats it as a swap table drops the constants. */
static void decode_ksel(BPState *bp, unsigned n, u32 v)
{
    unsigned s0 = n * 2, s1 = s0 + 1;
    unsigned table = n >> 1;

    if (s1 < BP_MAX_TEV_STAGES) {
        bp->tev[s0].konst_color = BITS(v, 4, 5);
        bp->tev[s0].konst_alpha = BITS(v, 9, 5);
        bp->tev[s1].konst_color = BITS(v, 14, 5);
        bp->tev[s1].konst_alpha = BITS(v, 19, 5);
    }

    /* The even register of a pair names red and green, the odd one blue and
     * alpha -- the field is in the same two bit positions either way, and only
     * the register's parity says which channels it is about. */
    if (n & 1u) {
        bp->tev_swap[table][2] = BITS(v, 0, 2);   /* blue  */
        bp->tev_swap[table][3] = BITS(v, 2, 2);   /* alpha */
    } else {
        bp->tev_swap[table][0] = BITS(v, 0, 2);   /* red   */
        bp->tev_swap[table][1] = BITS(v, 2, 2);   /* green */
    }
}

/* The fog registers hold two 11-bit floats in a format of their own: sign,
 * 8-bit exponent, 11-bit mantissa, which is an IEEE single with the bottom
 * twelve mantissa bits cut off. Reconstructing it by shifting the mantissa
 * back up is exact; computing it as mant/2048 * 2^(exp-127) is not, and the
 * error lands in the fog's distance scale where it looks like a slightly wrong
 * draw distance rather than a decoding bug. */
static float fog_float(u32 v)
{
    union { u32 u; float f; } cv;
    cv.u = (BITS(v, 19, 1) << 31) | (BITS(v, 11, 8) << 23) | (BITS(v, 0, 11) << 12);
    return cv.f;
}

static void decode_fog(BPState *bp)
{
    u32 p3 = bp->raw[BP_FOGPARAM3];
    unsigned i;

    bp->fog.fsel        = BITS(p3, 21, 3);
    bp->fog.projection  = BITS(p3, 20, 1);
    bp->fog.a           = fog_float(bp->raw[BP_FOGPARAM0]);
    bp->fog.c           = fog_float(p3);
    bp->fog.b_magnitude = bp->raw[BP_FOGBMAGNITUDE] & 0xFFFFFFu;
    bp->fog.b_shift     = bp->raw[BP_FOGBEXPONENT] & 0x1Fu;

    bp->fog.color[0] = (float)BITS(bp->raw[BP_FOGCOLOR], 16, 8) / 255.0f;
    bp->fog.color[1] = (float)BITS(bp->raw[BP_FOGCOLOR],  8, 8) / 255.0f;
    bp->fog.color[2] = (float)BITS(bp->raw[BP_FOGCOLOR],  0, 8) / 255.0f;

    /* The range-adjustment table: five registers of two 12-bit entries, and
     * the centre carries the same +342 rasteriser bias the scissor does. */
    bp->fog.range_enable = BITS(bp->raw[BP_FOGRANGE], 10, 1);
    /* Signed, and the rasteriser's +342 bias removed, exactly as the
     * scissor registers are -- so every consumer sees screen space and none
     * of them has to remember. */
    bp->fog.range_center = (int)BITS(bp->raw[BP_FOGRANGE], 0, 10) - 342;
    for (i = 0; i < 5; i++) {
        u32 k = bp->raw[BP_FOGRANGE + 1 + i];
        bp->fog.range_k[i * 2 + 0] = (float)BITS(k, 12, 12) / 256.0f;
        bp->fog.range_k[i * 2 + 1] = (float)BITS(k,  0, 12) / 256.0f;
    }
}

/* An EFB copy is the only BP register with a side effect rather than a state
 * change, so it is the only one that reads other registers to do its work. */
static void do_efb_copy(BPState *bp, u32 v)
{
    BPCopy *c = &bp->copy;
    u32 tl = bp->raw[BP_EFB_TL];
    u32 wh = bp->raw[BP_EFB_WH];

    c->src_x  = BITS(tl, 0, 10);
    c->src_y  = BITS(tl, 10, 10);
    /* Width and height are also stored one less than their real value; a copy
     * that is one pixel short on each axis leaves a visible seam. */
    c->width  = BITS(wh, 0, 10) + 1u;
    c->height = BITS(wh, 10, 10) + 1u;

    c->dest_addr   = (bp->raw[BP_EFB_DEST_ADDR] & 0x00FFFFFFu) << 5;
    c->dest_stride = (bp->raw[BP_EFB_STRIDE] & 0x3FFu) << 5;

    /* The trigger word's layout, field by field, because three of these were
     * previously read off the wrong bits and a copy register read one bit to
     * the left is not a wrong picture -- it is a wrong *decision* about what
     * kind of copy this is:
     *
     *   0     clamp top          8:7   gamma (0=1.0, 1=1.7, 2=2.2)
     *   1     clamp bottom       9     half scale (mipmap: 2x2 -> 1)
     *   2     YUV                10    scale invert (Y-scale reciprocal)
     *   6:3   copy format        11    clear the EFB after copying
     *                            13:12 frame-to-field
     *                            14    copy to the XFB
     *                            15    intensity (luminance) format
     */
    c->clear        = BITS(v, 11, 1);
    c->to_xfb       = BITS(v, 14, 1);
    c->half_scale   = BITS(v, 9, 1);
    c->scale_invert = BITS(v, 10, 1);
    /* The format field is stored with its bits rotated: the hardware's
     * encoding is (raw >> 1) | ((raw & 1) << 3), a one-place cycling right
     * shift. Reading it raw is not a wrong colour, it is a wrong *texture
     * encoding*, and therefore a wrong size: Mario Kart Wii's render-to-
     * texture copies write raw 0xA, which is RGB5A3 (16 bits per texel, and a
     * stride of 1024 bytes for the 128-wide copy the game actually issues) --
     * read raw it decodes as "B8", an 8-bit format whose stride would be 512,
     * so every byte of every copy after the first row would land in the wrong
     * place. `format_raw` keeps the register's own value for anyone reading
     * this next to a FIFO capture. */
    c->format_raw   = BITS(v, 3, 4);
    c->format       = (c->format_raw >> 1) | ((c->format_raw & 1u) << 3);
    c->intensity    = BITS(v, 15, 1);
    c->gamma        = BITS(v, 7, 2);

    /* The clear colour arrives as two registers holding alpha/red and
     * green/blue, which is a legacy of the copy pipeline's byte lanes. */
    c->clear_color = ((bp->raw[BP_CLEAR_AR] & 0xFF00u) << 16) |  /* A */
                     ((bp->raw[BP_CLEAR_AR] & 0x00FFu) << 16) |  /* R */
                     ((bp->raw[BP_CLEAR_GB] & 0xFF00u) >> 0)  |  /* G */
                      (bp->raw[BP_CLEAR_GB] & 0x00FFu);          /* B */
    c->clear_z = bp->raw[BP_CLEAR_Z] & 0x00FFFFFFu;

    bp->copies++;
}

/* ------------------------------------------------------------------ */

void bp_write(BPState *bp, u8 reg, u32 value)
{
    /* TEV register colours: 0xE0..0xE7, lo/hi pairs per register. The lo word
     * carries red (0:10) and alpha (12:22); the hi word blue and green. Bit 23
     * distinguishes the konstant bank.
     *
     * The fields are S11 -- ELEVEN BITS SIGNED, and the sign is load-bearing.
     * GXSetTevColorS10 exists precisely so a title can park negative
     * constants in a TEV register, and the YUV->RGB conversion every THP
     * movie uses does exactly that: Nintendo's own THPDraw.c sets
     * TEVREG0 = {-90, 0, -114, 135}. Reading the field unsigned turned -90
     * into 1958, clamped it to 255, and inverted the colour matrix -- which
     * is why every video in the game rendered magenta.
     *
     * The konstant bank is genuinely 0..255 unsigned (GXSetTevKColor takes
     * a GXColor), so only the register bank is sign-extended. */
    if (reg >= 0xE0 && reg <= 0xE7) {
        unsigned rn   = (reg - 0xE0u) >> 1;
        int      hi   = reg & 1;
        int      kon  = (value >> 23) & 1;
        float  (*bank)[4] = kon ? bp->tev_konst : bp->tev_reg;
        unsigned a4 = value & 0x7FFu, b4 = (value >> 12) & 0x7FFu;
        float fa, fb;
        if (kon) {
            fa = (a4 > 255 ? 255 : a4) / 255.0f;
            fb = (b4 > 255 ? 255 : b4) / 255.0f;
        } else {
            int sa = (int)(a4 << 21) >> 21;     /* sign-extend 11 -> 32 */
            int sb = (int)(b4 << 21) >> 21;
            fa = (float)sa / 255.0f;
            fb = (float)sb / 255.0f;
        }
        if (hi) { bank[rn][2] = fa; bank[rn][1] = fb; }
        else    { bank[rn][0] = fa; bank[rn][3] = fb; }
    }

    value &= 0x00FFFFFFu;

    /* Record first, decode second. The raw file is the source of truth, so a
     * register this function does not understand is still not lost. */
    bp->raw[reg] = value;

    /* TLUT LOAD (0x64 = source and count, 0x65 = destination). The pair
     * copies palette entries from main memory into TLUT memory: 0x64 carries
     * the source address in 32-byte units, 0x65 the TMEM destination in
     * 32-byte units plus the entry count. Hardware performs the copy when
     * 0x65 is written, so 0x64 is only latched. */
    if (reg == 0x64) { bp->tlut_src = (value & 0x00FFFFFFu) << 5; return; }
    if (reg == 0x65) {
        u32 dst   = (value & 0x3FFu) << 5;          /* TMEM byte offset */
        u32 count = ((value >> 10) & 0x3FFu) << 4;  /* entries (16-bit) */
        u32 i;
        if (count == 0) count = 16;
        /* bp.c models registers and deliberately does not reach into guest
         * memory; the embedder supplies the reader. */
        if (bp->read16) {
            for (i = 0; i < count; i++) {
                u32 di = (dst >> 1) + i;
                if (di >= (u32)(sizeof bp->tlut_mem /
                                sizeof bp->tlut_mem[0])) break;
                bp->tlut_mem[di] = bp->read16(bp->tlut_src + i * 2u);
            }
            bp->tlut_loaded = 1;
        }
        return;
    }

    switch (reg) {
    case BP_GENMODE:      decode_genmode(bp, value); return;
    case BP_ZMODE:        decode_zmode(bp, value);   return;
    case BP_ZCOMPARE:
        bp->pe_control = value;
        break;

    case BP_BLENDMODE:    decode_blend(bp, value);   return;
    case BP_ALPHACOMPARE: decode_alpha_test(bp, value); return;

    /* Scissor coordinates are biased by +342 -- an artifact of the rasteriser's
     * internal origin, which sits well outside the visible area so that guard
     * banding works without signed coordinates. The bias is removed here so
     * every consumer sees screen space; leaving it in place would mean each
     * one has to remember, and the one that forgets clips a band off the
     * left edge of every frame. The offset register carries the same bias
     * and is additionally stored halved. */
    case BP_SCISSOR_TL:
        bp->scissor_top  = BITS(value, 0, 11)  - 342;
        bp->scissor_left = BITS(value, 12, 11) - 342;
        return;
    case BP_SCISSOR_BR:
        bp->scissor_bottom = BITS(value, 0, 11)  - 342;
        bp->scissor_right  = BITS(value, 12, 11) - 342;
        return;
    case BP_SCISSOR_OFFSET:
        bp->scissor_offset_x = (int)BITS(value, 0, 10) * 2 - 342;
        bp->scissor_offset_y = (int)BITS(value, 10, 10) * 2 - 342;
        return;

    case BP_TRIGGER_EFB_COPY:
        do_efb_copy(bp, value);
        return;

    case BP_SET_DRAWDONE:
        /* Value 0x02 means "signal when the pipeline drains", which is how a
         * title learns the GPU finished the frame. Counting it is not enough:
         * the title waits on the pixel engine's finish interrupt, so the
         * hardware has to be told as well. */
        if ((value & 0xFFu) == 0x02u) {
            bp->draw_done++;
            if (s_pe_finish) s_pe_finish();
        }
        return;

    case BP_PE_TOKEN:
    case BP_PE_TOKEN_INT:
        bp->last_token = (u16)value;
        bp->tokens++;
        /* 0x48 is the interrupting form; 0x47 only updates the register a
         * title may poll. */
        if (s_pe_token) s_pe_token((u16)value, reg == BP_PE_TOKEN_INT);
        return;

    case BP_TEX_INVALIDATE:
        return;                     /* nothing is cached yet */

    case BP_RAS1_SS0:     decode_ind_scale(bp, 0, value); return;
    case BP_RAS1_SS1:     decode_ind_scale(bp, 1, value); return;
    case BP_RAS1_IREF:    decode_ind_ref(bp, value);      return;

    default:
        break;
    }

    /* Ranges. */
    if (reg >= BP_IND_MTX && reg < BP_IND_MTX + 9) {
        decode_ind_mtx(bp, reg - BP_IND_MTX, value);
        return;
    }
    if (reg >= BP_IND_CMD && reg < BP_IND_CMD + BP_MAX_TEV_STAGES) {
        decode_tevind(bp, reg - BP_IND_CMD, value);
        return;
    }
    if (reg >= BP_TEV_REF && reg < BP_TEV_REF + 8) {
        decode_tev_order(bp, reg - BP_TEV_REF, value);
        return;
    }
    if (reg >= BP_TEV_COLOR_ENV && reg < BP_TEV_COLOR_ENV + 32) {
        /* Colour and alpha environments alternate, two registers per stage. */
        unsigned idx = reg - BP_TEV_COLOR_ENV;
        unsigned stage = idx >> 1;
        if (stage < BP_MAX_TEV_STAGES) {
            if (idx & 1u) bp->tev[stage].alpha_env = value;
            else          bp->tev[stage].color_env = value;
        }
        return;
    }
    if (reg >= BP_TX_SETMODE0 && reg < BP_TX_SETMODE0 + 0x20) {
        unsigned which = (reg - BP_TX_SETMODE0) >> 2;
        unsigned unit  = (reg - BP_TX_SETMODE0) & 3u;
        decode_texture(bp, unit, which, value);
        return;
    }
    if (reg >= BP_TX_SETMODE0_4 && reg < BP_TX_SETMODE0_4 + 0x20) {
        unsigned which = (reg - BP_TX_SETMODE0_4) >> 2;
        unsigned unit  = 4u + ((reg - BP_TX_SETMODE0_4) & 3u);
        decode_texture(bp, unit, which, value);
        return;
    }
    if (reg >= BP_TEV_KSEL && reg < BP_TEV_KSEL + 8) {
        decode_ksel(bp, reg - BP_TEV_KSEL, value);
        return;
    }
    if (reg >= BP_FOGRANGE && reg <= BP_FOGCOLOR) {
        decode_fog(bp);
        return;
    }

    /* Everything else is recorded and not yet acted on. Deliberately not
     * warned about: a title writes dozens of these per frame and the noise
     * would bury the warnings that matter. The raw file has them when a
     * renderer is ready to read them. */
}
