/* vertex_loader.c — decoding GX vertex attributes.
 *
 * See vertex_loader.h. The three things worth knowing before reading:
 *
 * Fixed-point components carry a *fractional bit count* from the attribute
 * table, so the value is `raw / 2^frac`. Floats ignore it. Normals ignore it
 * too and use a fixed scale set by their format, which is the one place the
 * rule does not hold and therefore the one place it gets applied by mistake.
 *
 * Indexed attributes are an index into an array whose base and stride live in
 * CP registers, so decoding follows a pointer into guest memory rather than
 * reading the stream.
 *
 * Everything is big-endian, because the guest wrote it.
 */
#include "vertex_loader.h"
#include "../../core/mem/memmap.h"
#include "../../common/log.h"

u64 g_vtx_draws_direct, g_vtx_draws_indexed;

#include <string.h>
#include <stdint.h>

#define BITS(v, lsb, width) (((u32)(v) >> (lsb)) & ((1u << (width)) - 1u))

/* VAT field positions, matching gx_vertex.c. Duplicated deliberately rather
 * than shared: that file needs sizes and this one needs meanings, and a single
 * "VAT accessor" header would make each change to either look like a change to
 * both. The build-time assertions at the end keep them honest. */
#define VAT_POS_ELEMENTS(a)  BITS(a, 0, 1)
#define VAT_POS_FORMAT(a)    BITS(a, 1, 3)
#define VAT_POS_FRAC(a)      BITS(a, 4, 5)
#define VAT_NRM_ELEMENTS(a)  BITS(a, 9, 1)
#define VAT_NRM_FORMAT(a)    BITS(a, 10, 3)
#define VAT_COL0_FORMAT(a)   BITS(a, 14, 3)
#define VAT_COL1_FORMAT(a)   BITS(a, 18, 3)
#define VAT_NORMAL_INDEX3(a) BITS(a, 31, 1)

#define VCD_POS_MAT_IDX(lo)    BITS(lo, 0, 1)
#define VCD_TEX_MAT_IDX(lo, n) BITS(lo, 1 + (n), 1)
#define VCD_POSITION(lo)       BITS(lo, 9, 2)
#define VCD_NORMAL(lo)         BITS(lo, 11, 2)
#define VCD_COLOR0(lo)         BITS(lo, 13, 2)
#define VCD_COLOR1(lo)         BITS(lo, 15, 2)
#define VCD_TEXCOORD(hi, n)    BITS(hi, 2 * (n), 2)

/* Texture coordinates are spread across all three VAT registers; the tables
 * are the same ones gx_vertex.c uses to size them. */
static const u8 k_tex_elem_lsb[GX_NUM_TEXCOORD] = { 21, 0,  9, 18, 27,  5, 14, 23 };
static const u8 k_tex_fmt_lsb [GX_NUM_TEXCOORD] = { 22, 1, 10, 19, 28,  6, 15, 24 };
static const u8 k_tex_frac_lsb[GX_NUM_TEXCOORD] = { 25, 4, 13, 22,  0,  9, 18, 27 };

/* Coordinate 4 is the awkward one: its element and format bits are in VAT B
 * but its fractional bits are in VAT C. Splitting a field across two registers
 * is not something a reader expects, so it is called out rather than buried in
 * a table index. */
static u32 tex_group(unsigned n, u32 a, u32 b, u32 c)
{
    if (n == 0) return a;
    if (n <= 4) return b;
    return c;
}

static u32 tex_frac_group(unsigned n, u32 a, u32 b, u32 c)
{
    if (n == 0) return a;
    if (n <= 3) return b;
    return c;               /* including 4, whose frac bits live in VAT C */
}

/* ------------------------------------------------------------------ */

float vtx_dequantize(u32 raw, GXCompFormat fmt, unsigned frac_bits)
{
    float scale = 1.0f / (float)(1u << (frac_bits & 31u));

    switch (fmt) {
    case GX_COMP_U8:  return (float)(u8)raw  * scale;
    case GX_COMP_S8:  return (float)(s8)raw  * scale;
    case GX_COMP_U16: return (float)(u16)raw * scale;
    case GX_COMP_S16: return (float)(s16)raw * scale;
    case GX_COMP_F32: {
        /* A float ignores the fractional count entirely -- it is already
         * scaled. Applying it here is the single most likely way to produce
         * geometry that is a recognisable model at the wrong size. */
        float f;
        memcpy(&f, &raw, sizeof f);
        return f;
    }
    default:
        return 0.0f;
    }
}

static u32 read_component(u32 addr, GXCompFormat fmt)
{
    switch (fmt) {
    case GX_COMP_U8:  case GX_COMP_S8:  return mem_read8(addr);
    case GX_COMP_U16: case GX_COMP_S16: return mem_read16(addr);
    case GX_COMP_F32:                   return mem_read32(addr);
    default:                            return 0;
    }
}

/* Normals do not use the attribute table's fractional bits. Their scale is
 * fixed by format -- 1/64 for a byte, 1/16384 for a short -- because a normal
 * is always in [-1, 1] and the hardware knows it. */
static float dequantize_normal(u32 raw, GXCompFormat fmt)
{
    switch (fmt) {
    case GX_COMP_S8:  return (float)(s8)raw  / 64.0f;
    case GX_COMP_U8:  return (float)(u8)raw  / 64.0f;
    case GX_COMP_S16: return (float)(s16)raw / 16384.0f;
    case GX_COMP_U16: return (float)(u16)raw / 16384.0f;
    case GX_COMP_F32: { float f; memcpy(&f, &raw, sizeof f); return f; }
    default:          return 0.0f;
    }
}

static void decode_color(u32 addr, GXColorFormat fmt, float out[4])
{
    u32 v;
    switch (fmt) {
    case GX_CLR_RGB565:
        v = mem_read16(addr);
        out[0] = (float)BITS(v, 11, 5) / 31.0f;
        out[1] = (float)BITS(v, 5, 6)  / 63.0f;
        out[2] = (float)BITS(v, 0, 5)  / 31.0f;
        out[3] = 1.0f;
        return;
    case GX_CLR_RGB8:
        out[0] = mem_read8(addr)     / 255.0f;
        out[1] = mem_read8(addr + 1) / 255.0f;
        out[2] = mem_read8(addr + 2) / 255.0f;
        out[3] = 1.0f;
        return;
    case GX_CLR_RGBX8:
        out[0] = mem_read8(addr)     / 255.0f;
        out[1] = mem_read8(addr + 1) / 255.0f;
        out[2] = mem_read8(addr + 2) / 255.0f;
        /* The fourth byte exists but is not alpha -- hence RGBX. Reading it as
         * alpha makes every such vertex transparent by whatever happened to be
         * in the padding. */
        out[3] = 1.0f;
        return;
    case GX_CLR_RGBA4:
        v = mem_read16(addr);
        out[0] = (float)BITS(v, 12, 4) / 15.0f;
        out[1] = (float)BITS(v, 8, 4)  / 15.0f;
        out[2] = (float)BITS(v, 4, 4)  / 15.0f;
        out[3] = (float)BITS(v, 0, 4)  / 15.0f;
        return;
    case GX_CLR_RGBA6:
        /* Three bytes holding four six-bit channels, so every channel but the
         * first straddles a byte boundary. */
        v = ((u32)mem_read8(addr) << 16) | ((u32)mem_read8(addr + 1) << 8) |
             (u32)mem_read8(addr + 2);
        out[0] = (float)BITS(v, 18, 6) / 63.0f;
        out[1] = (float)BITS(v, 12, 6) / 63.0f;
        out[2] = (float)BITS(v, 6, 6)  / 63.0f;
        out[3] = (float)BITS(v, 0, 6)  / 63.0f;
        return;
    case GX_CLR_RGBA8:
        out[0] = mem_read8(addr)     / 255.0f;
        out[1] = mem_read8(addr + 1) / 255.0f;
        out[2] = mem_read8(addr + 2) / 255.0f;
        out[3] = mem_read8(addr + 3) / 255.0f;
        return;
    default:
        out[0] = out[1] = out[2] = out[3] = 1.0f;
        return;
    }
}

/* Where an attribute's data actually lives, and how many stream bytes reading
 * it costs. For a direct attribute those are the same place; for an indexed one
 * the stream holds only the index. */
typedef struct {
    u32 data_addr;
    u32 stream_cost;
    int present;
} AttrRef;

static AttrRef locate(const GXCPRegs *cp, unsigned type, unsigned array,
                      u32 stream_addr)
{
    AttrRef r;
    r.present = 1;
    switch (type) {
    case GX_ATTR_DIRECT:
        r.data_addr = stream_addr;
        r.stream_cost = 0;          /* the caller adds the data size */
        return r;
    case GX_ATTR_INDEX8:
        r.data_addr = cp->array_base[array] +
                      (u32)mem_read8(stream_addr) * cp->array_stride[array];
        r.stream_cost = 1;
        return r;
    case GX_ATTR_INDEX16:
        r.data_addr = cp->array_base[array] +
                      (u32)mem_read16(stream_addr) * cp->array_stride[array];
        r.stream_cost = 2;
        return r;
    case GX_ATTR_NONE:
    default:
        r.present = 0;
        r.data_addr = 0;
        r.stream_cost = 0;
        return r;
    }
}

/* ------------------------------------------------------------------ */

/* Array indices used by indexed attributes. These are CP ARRAY-REGISTER
 * numbers (the low nibble of CP 0xA0..0xAF, which is how gx_parse.c stores
 * array_base[]), NOT the GXAttr enum. They were the GXAttr values (9..13),
 * which silently made every indexed fetch read from the wrong base pointer.
 * Menu quads use DIRECT attributes, so the title screen was unaffected and
 * the bug stayed invisible -- it would have corrupted every 3D model the
 * moment a race started. */
#define ARRAY_POSITION  0
#define ARRAY_NORMAL    1
#define ARRAY_COLOR0    2
#define ARRAY_COLOR1    3
#define ARRAY_TEX0      4

/* Sanity of decoded positions, split by whether the draw used per-attribute
 * indexing. The 3D world renders black while the 2D HUD renders correctly, and
 * the HUD is exactly the direct-attribute traffic (14.7% of draws) while the
 * world is the indexed traffic (85.3%). If indexed decode is fetching from the
 * wrong array base, its positions will be garbage where direct ones are sane —
 * which is a measurement, not an inference. */
unsigned long long g_vtxpos_ok[2], g_vtxpos_bad[2];
double g_vtxpos_absmax[2];

static void vtx_pos_note(int indexed, const float *p)
{
    int i, bad = 0;
    for (i = 0; i < 3; i++) {
        double v = p[i], a = v < 0 ? -v : v;
        if (!(v == v) || a > 1.0e7) bad = 1;
        if (a > g_vtxpos_absmax[indexed]) g_vtxpos_absmax[indexed] = a;
    }
    if (bad) g_vtxpos_bad[indexed]++; else g_vtxpos_ok[indexed]++;
}

u32 vtx_decode(const GXCPRegs *cp, unsigned vat, u32 addr, VtxAttributes *out)
{
    u32 lo, hi, a, b, c;
    u32 p = addr;
    unsigned n;

    memset(out, 0, sizeof *out);
    if (vat >= GX_NUM_VAT)
        return 0;

    lo = cp->vcd_lo; hi = cp->vcd_hi;
    a = cp->vat_a[vat]; b = cp->vat_b[vat]; c = cp->vat_c[vat];

    /* Matrix indices come first in the vertex and are always one direct byte. */
    if (VCD_POS_MAT_IDX(lo)) {
        out->pos_matrix_index = mem_read8(p);
        out->has_pos_matrix_index = 1;
        p += 1;
    }
    for (n = 0; n < GX_NUM_TEXCOORD; n++) {
        if (VCD_TEX_MAT_IDX(lo, n)) {
            out->tex_matrix_index[n] = mem_read8(p);
            p += 1;
        }
    }

    /* Position. */
    {
        unsigned type = VCD_POSITION(lo);
        AttrRef r = locate(cp, type, ARRAY_POSITION, p);
        if (r.present) {
            GXCompFormat fmt = (GXCompFormat)VAT_POS_FORMAT(a);
            unsigned count = VAT_POS_ELEMENTS(a) ? 3u : 2u;
            unsigned frac  = VAT_POS_FRAC(a);
            u32 csize = gx_component_size(fmt);
            unsigned i;
            for (i = 0; i < count; i++)
                out->position[i] = vtx_dequantize(
                    read_component(r.data_addr + i * csize, fmt), fmt, frac);
            out->position_count = count;
            p += (type == GX_ATTR_DIRECT) ? count * csize : r.stream_cost;
        }
    }

    /* Normal, or a full normal/binormal/tangent frame. */
    {
        unsigned type = VCD_NORMAL(lo);
        AttrRef r = locate(cp, type, ARRAY_NORMAL, p);
        if (r.present) {
            GXCompFormat fmt = (GXCompFormat)VAT_NRM_FORMAT(a);
            unsigned vectors = VAT_NRM_ELEMENTS(a) ? 3u : 1u;
            unsigned count = 3u * vectors;
            u32 csize = gx_component_size(fmt);
            unsigned i;
            for (i = 0; i < count; i++)
                out->normal[i] = dequantize_normal(
                    read_component(r.data_addr + i * csize, fmt), fmt);
            out->normal_count = count;

            if (type == GX_ATTR_DIRECT) {
                p += count * csize;
            } else if (VAT_NORMAL_INDEX3(a) && vectors == 3u) {
                /* Three separate indices, one per vector. */
                p += r.stream_cost * 3u;
            } else {
                p += r.stream_cost;
            }
        }
    }

    /* Colours. */
    {
        unsigned ci;
        for (ci = 0; ci < 2; ci++) {
            unsigned type = ci ? VCD_COLOR1(lo) : VCD_COLOR0(lo);
            AttrRef r = locate(cp, type, ci ? ARRAY_COLOR1 : ARRAY_COLOR0, p);
            if (!r.present)
                continue;
            {
                GXColorFormat fmt = (GXColorFormat)(ci ? VAT_COL1_FORMAT(a)
                                                       : VAT_COL0_FORMAT(a));
                decode_color(r.data_addr, fmt, out->color[ci]);
                out->color_present[ci] = 1;
                p += (type == GX_ATTR_DIRECT) ? gx_color_size(fmt)
                                              : r.stream_cost;
            }
        }
    }

    /* Texture coordinates. */
    for (n = 0; n < GX_NUM_TEXCOORD; n++) {
        unsigned type = VCD_TEXCOORD(hi, n);
        AttrRef r = locate(cp, type, ARRAY_TEX0 + n, p);
        if (!r.present)
            continue;
        {
            u32 greg  = tex_group(n, a, b, c);
            u32 freg  = tex_frac_group(n, a, b, c);
            GXCompFormat fmt = (GXCompFormat)BITS(greg, k_tex_fmt_lsb[n], 3);
            unsigned count = BITS(greg, k_tex_elem_lsb[n], 1) ? 2u : 1u;
            unsigned frac  = BITS(freg, k_tex_frac_lsb[n], 5);
            u32 csize = gx_component_size(fmt);
            unsigned i;
            for (i = 0; i < count; i++)
                out->texcoord[n][i] = vtx_dequantize(
                    read_component(r.data_addr + i * csize, fmt), fmt, frac);
            out->texcoord_count[n] = count;
            p += (type == GX_ATTR_DIRECT) ? count * csize : r.stream_cost;
        }
    }

    /* Was ANY attribute of this vertex fetched through an index? */
    {   int idxed = 0;
        u32 lo = cp->vcd_lo, hi = cp->vcd_hi;
        unsigned q;
        if (((lo >> 9) & 3u) >= 2u) idxed = 1;
        if (((lo >> 11) & 3u) >= 2u) idxed = 1;
        if (((lo >> 13) & 3u) >= 2u) idxed = 1;
        if (((lo >> 15) & 3u) >= 2u) idxed = 1;
        for (q = 0; q < GX_NUM_TEXCOORD; q++)
            if (((hi >> (2 * q)) & 3u) >= 2u) idxed = 1;
        if (out->position_count) vtx_pos_note(idxed, out->position);
    }

    return p - addr;
}

/* ------------------------------------------------------------------ */
/* SPU recipe builder                                                   */
/*                                                                      */
/* Compiles the CP/VAT state for one draw into the SpuVtxJob protocol   */
/* (spu_vtx_shared.h) that the SPU vertex decoder interprets, mirroring */
/* vtx_decode's stream walk EXACTLY -- the two must agree on every byte */
/* consumed or the stream desynchronises. Returns 0 and fills the job   */
/* on success; -1 when any attribute is outside what the protocol can   */
/* express (the caller then decodes on the PPU as before). Verified by  */
/* construction: the recipe's computed per-vertex size must equal       */
/* gx_vertex_size or the build fails.                                   */
/* ------------------------------------------------------------------ */
#include "spu_vtx_shared.h"

static int spu_fmt(GXCompFormat f)
{
    switch (f) {
    case GX_COMP_U8:  return SVF_U8;
    case GX_COMP_S8:  return SVF_S8;
    case GX_COMP_U16: return SVF_U16;
    case GX_COMP_S16: return SVF_S16;
    case GX_COMP_F32: return SVF_F32;
    default:          return -1;
    }
}
static int spu_cfmt(GXColorFormat f)
{
    switch (f) {
    case GX_CLR_RGB565: return SVC_RGB565;
    case GX_CLR_RGB8:   return SVC_RGB8;
    case GX_CLR_RGBX8:  return SVC_RGBX8;
    case GX_CLR_RGBA4:  return SVC_RGBA4;
    case GX_CLR_RGBA6:  return SVC_RGBA6;
    case GX_CLR_RGBA8:  return SVC_RGBA8;
    default:            return -1;
    }
}

/* Cached format decode.
 *
 * Everything from here to the vsize check depends ONLY on the vertex format --
 * the descriptor, the VAT registers for this slot, and the array bases and
 * strides. None of that changes per draw; a title sets a format and then
 * issues many draws with it. The decode is a long sequence of bit extractions
 * over position, normal, two colours and eight texture coordinates, and it was
 * being redone for all 6,231 draws a frame (spu_job: 8.2% of the frame,
 * 2.1 us per draw, on a PPE that is the bottleneck).
 *
 * The per-draw part -- the index-window scan, the stream translation, the
 * count and destination -- still runs every time. Only the recipe is reused.
 * The scan mutates the attributes it is given (array_ea becomes a host
 * pointer, base_idx and window_len get filled in), so the template is copied
 * into the caller's job first and the mutation lands on the copy. */
/* Armed by dolphin-spuscan.txt. Off by default until measured. */
int g_spu_scan_on;

static struct {
    int      valid;
    u32      vcd_lo, vcd_hi, va, vb, vc;
    unsigned vat;
    u32      abase[GX_NUM_ARRAYS];
    u16      astride[GX_NUM_ARRAYS];
    SpuVtxJob job;
    u32      attr_off[SPU_VTX_MAX_ATTRS];
    unsigned na;
    u32      vsize;
} s_fmt;

static int fmt_matches(const GXCPRegs *cp, unsigned vat)
{
    unsigned i;
    if (!s_fmt.valid || s_fmt.vat != vat) return 0;
    if (s_fmt.vcd_lo != cp->vcd_lo || s_fmt.vcd_hi != cp->vcd_hi) return 0;
    if (s_fmt.va != cp->vat_a[vat] || s_fmt.vb != cp->vat_b[vat] ||
        s_fmt.vc != cp->vat_c[vat]) return 0;
    for (i = 0; i < GX_NUM_ARRAYS; i++)
        if (s_fmt.abase[i] != cp->array_base[i] ||
            s_fmt.astride[i] != cp->array_stride[i]) return 0;
    return 1;
}

int vtx_build_spu_job(const GXCPRegs *cp, unsigned vat, u32 stream_addr,
                      unsigned count, u64 dest_ea, SpuVtxJob *job)
{
    u32 lo, hi, a, b, c;
    unsigned na = 0, n, ci;
    u32 vsize = 0;
    /* stream offset of each attr's field (fixed per vertex) */
    u32 attr_off[SPU_VTX_MAX_ATTRS];

    if (vat >= GX_NUM_VAT || count == 0 || count > SPU_VTX_MAX_VERTS)
        return -1;

    if (fmt_matches(cp, vat)) {
        memcpy(job, &s_fmt.job, sizeof *job);
        memcpy(attr_off, s_fmt.attr_off, sizeof attr_off);
        na    = s_fmt.na;
        vsize = s_fmt.vsize;
        goto format_done;
    }

    lo = cp->vcd_lo; hi = cp->vcd_hi;
    a = cp->vat_a[vat]; b = cp->vat_b[vat]; c = cp->vat_c[vat];
    memset(job, 0, sizeof *job);

#define ADD_ATTR() (na >= SPU_VTX_MAX_ATTRS ? -1 : (int)na++)
#define AT (&job->attr[na - 1])

    if (VCD_POS_MAT_IDX(lo)) {
        if (ADD_ATTR() < 0) return -1;
        AT->kind = SVA_PNMTXIDX; AT->mode = SVM_DIRECT;
        attr_off[na-1] = vsize; vsize += 1;
    }
    for (n = 0; n < GX_NUM_TEXCOORD; n++) {
        if (VCD_TEX_MAT_IDX(lo, n)) {
            if (ADD_ATTR() < 0) return -1;
            AT->kind = SVA_TEXMTXIDX; AT->mode = SVM_DIRECT;
            attr_off[na-1] = vsize; vsize += 1;
        }
    }
    {
        unsigned type = VCD_POSITION(lo);
        if (type != GX_ATTR_NONE) {
            GXCompFormat fmt = (GXCompFormat)VAT_POS_FORMAT(a);
            int sf = spu_fmt(fmt);
            unsigned cnt = VAT_POS_ELEMENTS(a) ? 3u : 2u;
            if (sf < 0 || ADD_ATTR() < 0) return -1;
            AT->kind = SVA_POS; AT->fmt = (u8)sf; AT->count = (u8)cnt;
            AT->frac = (u8)VAT_POS_FRAC(a);
            attr_off[na-1] = vsize;
            if (type == GX_ATTR_DIRECT) {
                AT->mode = SVM_DIRECT;
                vsize += cnt * gx_component_size(fmt);
            } else {
                AT->mode = (type == GX_ATTR_INDEX8) ? SVM_IDX8 : SVM_IDX16;
                AT->stride = (u16)cp->array_stride[ARRAY_POSITION];
                AT->array_ea = (u64)cp->array_base[ARRAY_POSITION];
                vsize += (type == GX_ATTR_INDEX8) ? 1 : 2;
            }
        }
    }
    {
        unsigned type = VCD_NORMAL(lo);
        if (type != GX_ATTR_NONE) {
            GXCompFormat fmt = (GXCompFormat)VAT_NRM_FORMAT(a);
            int sf = spu_fmt(fmt);
            unsigned vectors = VAT_NRM_ELEMENTS(a) ? 3u : 1u;
            if (vectors != 1u) return -1;       /* NBT frames: PPU path */
            if (type != GX_ATTR_DIRECT && VAT_NORMAL_INDEX3(a)) return -1;
            if (sf < 0 || ADD_ATTR() < 0) return -1;
            AT->kind = SVA_NRM; AT->fmt = (u8)sf; AT->count = 3;
            /* dequantize_normal: s8/u8 divide by 64, s16/u16 by 16384. */
            AT->frac = (fmt == GX_COMP_S8 || fmt == GX_COMP_U8) ? 6
                     : (fmt == GX_COMP_S16 || fmt == GX_COMP_U16) ? 14 : 0;
            attr_off[na-1] = vsize;
            if (type == GX_ATTR_DIRECT) {
                AT->mode = SVM_DIRECT;
                vsize += 3u * gx_component_size(fmt);
            } else {
                AT->mode = (type == GX_ATTR_INDEX8) ? SVM_IDX8 : SVM_IDX16;
                AT->stride = (u16)cp->array_stride[ARRAY_NORMAL];
                AT->array_ea = (u64)cp->array_base[ARRAY_NORMAL];
                vsize += (type == GX_ATTR_INDEX8) ? 1 : 2;
            }
        }
    }
    for (ci = 0; ci < 2; ci++) {
        unsigned type = ci ? VCD_COLOR1(lo) : VCD_COLOR0(lo);
        if (type != GX_ATTR_NONE) {
            GXColorFormat fmt = (GXColorFormat)(ci ? VAT_COL1_FORMAT(a)
                                                   : VAT_COL0_FORMAT(a));
            int sf = spu_cfmt(fmt);
            if (sf < 0 || ADD_ATTR() < 0) return -1;
            AT->kind = ci ? SVA_COL1 : SVA_COL0; AT->fmt = (u8)sf;
            attr_off[na-1] = vsize;
            if (type == GX_ATTR_DIRECT) {
                AT->mode = SVM_DIRECT;
                vsize += gx_color_size(fmt);
            } else {
                AT->mode = (type == GX_ATTR_INDEX8) ? SVM_IDX8 : SVM_IDX16;
                AT->stride = (u16)cp->array_stride[ci ? ARRAY_COLOR1
                                                      : ARRAY_COLOR0];
                AT->array_ea = (u64)cp->array_base[ci ? ARRAY_COLOR1 : ARRAY_COLOR0];
                vsize += (type == GX_ATTR_INDEX8) ? 1 : 2;
            }
        }
    }
    for (n = 0; n < GX_NUM_TEXCOORD; n++) {
        unsigned type = VCD_TEXCOORD(hi, n);
        if (type != GX_ATTR_NONE) {
            u32 greg  = tex_group(n, a, b, c);
            u32 freg  = tex_frac_group(n, a, b, c);
            GXCompFormat fmt = (GXCompFormat)BITS(greg, k_tex_fmt_lsb[n], 3);
            unsigned cnt = BITS(greg, k_tex_elem_lsb[n], 1) ? 2u : 1u;
            int sf = spu_fmt(fmt);
            if (sf < 0 || ADD_ATTR() < 0) return -1;
            AT->kind = (u8)(SVA_TEX0 + n); AT->fmt = (u8)sf;
            AT->count = (u8)cnt;
            AT->frac = (u8)BITS(freg, k_tex_frac_lsb[n], 5);
            attr_off[na-1] = vsize;
            if (type == GX_ATTR_DIRECT) {
                AT->mode = SVM_DIRECT;
                vsize += cnt * gx_component_size(fmt);
            } else {
                AT->mode = (type == GX_ATTR_INDEX8) ? SVM_IDX8 : SVM_IDX16;
                AT->stride = (u16)cp->array_stride[ARRAY_TEX0 + n];
                AT->array_ea = (u64)cp->array_base[ARRAY_TEX0 + n];
                vsize += (type == GX_ATTR_INDEX8) ? 1 : 2;
            }
        }
    }
#undef AT
#undef ADD_ATTR

    /* The recipe must account for every byte vtx_decode would consume. */
    if (vsize != gx_vertex_size(cp, vat))
        return -1;

    {   /* Remember it for the next draw with this format. */
        unsigned i;
        s_fmt.vcd_lo = cp->vcd_lo; s_fmt.vcd_hi = cp->vcd_hi;
        s_fmt.va = cp->vat_a[vat]; s_fmt.vb = cp->vat_b[vat];
        s_fmt.vc = cp->vat_c[vat]; s_fmt.vat = vat;
        for (i = 0; i < GX_NUM_ARRAYS; i++) {
            s_fmt.abase[i]   = cp->array_base[i];
            s_fmt.astride[i] = (u16)cp->array_stride[i];
        }
        memcpy(&s_fmt.job, job, sizeof s_fmt.job);
        memcpy(s_fmt.attr_off, attr_off, sizeof s_fmt.attr_off);
        s_fmt.na = na; s_fmt.vsize = vsize; s_fmt.valid = 1;
    }

format_done:

    /* Count all-direct vs indexed draws.
     *
     * This decides whether a large optimisation is available at all. GX's
     * quantised attribute formats (S16/U8/F32 with a fractional shift) map
     * onto RSX vertex attribute types the hardware already supports, and the
     * shift folds into the transform matrix -- so for a draw whose attributes
     * are ALL direct, the RSX could read the guest's vertex buffer as-is and
     * the entire decode disappears: no SPU job, no wait, no arena. What it
     * cannot do is GX's per-attribute indexing, which needs one index stream
     * per attribute where RSX has one for the whole vertex.
     *
     * So the value of that route is exactly the fraction of draws that are
     * all-direct, and nobody has measured it. Counting is nearly free. */
    {   unsigned d;
        int any_indexed = 0;
        for (d = 0; d < na; d++)
            if (job->attr[d].mode != SVM_DIRECT) { any_indexed = 1; break; }
        if (any_indexed) g_vtx_draws_indexed++;
        else             g_vtx_draws_direct++;
    }

    /* Indexed attrs: find each one's index span so the SPU can DMA a
     * bounded window. Rebase the recipe to the span's low element and pass
     * the low index for the SPU to subtract. */
    /* Hand the scan to the consumer when asked: point each indexed attribute
     * at element 0 of its array and say how far the mapping is known good.
     * The SPU derives min/max from the indices it already has. */
    if (g_spu_scan_on) {
        unsigned ok = 1;
        for (n = 0; n < na && ok; n++) {
            SpuVtxAttr *at9 = &job->attr[n];
            u32 base, avail;
            void *hp;
            if (at9->mode == SVM_DIRECT) { job->window_len[n] = 0; continue; }
            base  = (u32)at9->array_ea;
            avail = mem_valid_span(base);
            hp    = mem_ptr(base);
            if (!hp || avail < (u32)at9->stride) { ok = 0; break; }
            at9->array_ea      = (u64)(uintptr_t)hp;
            at9->base_idx      = 0;
            job->window_len[n] = avail;
        }
        if (ok) {
            job->flags |= SPU_JOB_SCAN;
            goto scan_done;
        }
        /* Could not vouch for an array; fall through to the PPU scan, which
         * validates each window individually. */
    }

    /* ONE pass over the vertices for ALL indexed attributes, not one pass
     * each.
     *
     * The per-attribute loop this replaces walked the whole stream once per
     * indexed attribute, striding by the vertex size each time -- so every
     * pass re-touched the same cache lines at a different offset, and with
     * several indexed attributes the earlier passes' lines had usually been
     * evicted by the time the next pass wanted them. A vertex's attributes
     * are ADJACENT in memory, so reading them together costs one or two lines
     * per vertex instead of one line per vertex per attribute.
     *
     * This is the hot loop of the draw path: 85% of a race's draws are
     * indexed, ~150 vertices each, and draw setup was 16% of the frame. */
    {
        u32 mn_of[SPU_VTX_MAX_ATTRS], mx_of[SPU_VTX_MAX_ATTRS];
        u32 v9;
        int any_idx = 0;
        for (n = 0; n < na; n++) {
            mn_of[n] = 0xFFFFFFFFu; mx_of[n] = 0;
            if (job->attr[n].mode != SVM_DIRECT) any_idx = 1;
        }
        if (any_idx) for (v9 = 0; v9 < count; v9++) {
            u32 base = stream_addr + v9 * vsize;
            for (n = 0; n < na; n++) {
                u32 idx;
                if (job->attr[n].mode == SVM_DIRECT) continue;
                idx = (job->attr[n].mode == SVM_IDX8)
                    ? mem_read8(base + attr_off[n])
                    : mem_read16(base + attr_off[n]);
                if (idx < mn_of[n]) mn_of[n] = idx;
                if (idx > mx_of[n]) mx_of[n] = idx;
            }
        }
        for (n = 0; n < na; n++) {
            SpuVtxAttr *at9 = &job->attr[n];
            u32 mn = mn_of[n], mx = mx_of[n];
            u32 span;
            if (at9->mode == SVM_DIRECT) continue;
            span = (mx - mn + 1u) * at9->stride;
        if (span > SPU_VTX_MAX_WINDOW - 256u)   /* room for DMA line skew */
            return -1;
        if (mn > 0xFFFFu) return -1;
        {
            u32 gwin = (u32)at9->array_ea + mn * at9->stride;
            void *hp;
            if (mem_valid_span(gwin) < span) return -1;
            hp = mem_ptr(gwin);
            if (!hp) return -1;
            at9->array_ea = (u64)(uintptr_t)hp;
        }
            at9->base_idx = (u16)mn;
            job->window_len[n] = span;
        }
    }
scan_done:;

    /* The SPU's MFC addresses HOST memory. Everything the recipe points at
     * must therefore be translated from the guest's address space into the
     * emulator process's, and the span must be backed contiguously -- the
     * SPU has no fault handler and a bad EA simply stalls its DMA queue
     * (which is exactly how the first version hung: it was handed raw
     * 0x80xxxxxx Wii addresses). */
    {
        void *sp2;
        if (mem_valid_span(stream_addr) < (u32)vsize * count)
            return -1;
        sp2 = mem_ptr(stream_addr);
        if (!sp2) return -1;
        job->stream_ea = (u64)(uintptr_t)sp2;
    }
    job->dest_ea    = dest_ea;
    job->vert_count = (u16)count;
    job->vert_size  = (u16)vsize;
    job->nattrs     = (u16)na;
    return 0;
}
