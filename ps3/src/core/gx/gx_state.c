/* gx_state.c — what the command stream means.
 *
 * See gx_state.h. This file is the sink the parser calls into: it applies XF
 * and BP writes, keeps the decoded state a renderer reads, and forwards draws
 * and framebuffer copies to a backend.
 */
#include "gx_state.h"
#include "../hw/gx_fifo.h"
#include "../mem/memmap.h"
#include "../../common/log.h"

#include <string.h>

static GXState s_gx;

GXState *gx_state(void) { return &s_gx; }

/* ------------------------------------------------------------------ */
/* XF                                                                   */
/* ------------------------------------------------------------------ */

/* XF values are IEEE floats in the command stream, delivered as raw words. The
 * reinterpretation goes through memcpy rather than a pointer cast: the JIT is
 * built with -fno-strict-aliasing but this file is not required to be, and a
 * cast here is exactly the kind of thing that works until an optimiser
 * notices. */
static float word_to_float(u32 v)
{
    float f;
    memcpy(&f, &v, sizeof f);
    return f;
}

static void xf_decode(XFState *xf, u16 addr)
{
    if (addr >= XF_VIEWPORT && addr < XF_VIEWPORT + 6) {
        unsigned i;
        for (i = 0; i < 3; i++) {
            xf->viewport_scale[i]  = word_to_float(xf->mem[XF_VIEWPORT + i]);
            xf->viewport_offset[i] = word_to_float(xf->mem[XF_VIEWPORT + 3 + i]);
        }
        return;
    }
    if (addr >= XF_PROJECTION && addr < XF_PROJECTION + 7) {
        unsigned i;
        for (i = 0; i < 6; i++)
            xf->projection[i] = word_to_float(xf->mem[XF_PROJECTION + i]);
        /* The seventh word is not a coefficient: it selects between an
         * orthographic and a perspective matrix, which changes what the six
         * mean. Reading it as a float would give a denormal and no warning. */
        xf->projection_orthographic = xf->mem[XF_PROJECTION + 6] & 1u;
        return;
    }
    if (addr == XF_NUMTEXGENS) {
        xf->num_texgens = xf->mem[XF_NUMTEXGENS] & 0xFu;
        return;
    }
    if (addr == XF_NUMCOLORS) {
        xf->num_colorchans = xf->mem[XF_NUMCOLORS] & 0x3u;
        return;
    }
}

/* ------------------------------------------------------------------ */
/* Parser sink                                                          */
/* ------------------------------------------------------------------ */

u32 g_xf_generation;   /* bumped on every XF memory write; lets the render
                        * backend skip re-uploading unchanged matrices */

static void on_load_xf(void *ctx, u16 addr, const u32 *values, unsigned count)
{
    g_xf_generation++;
    GXState *g = (GXState *)ctx;
    unsigned i;

    for (i = 0; i < count; i++) {
        u16 a = (u16)(addr + i);
        if (a >= XF_MEM_SIZE) {
            /* Beyond what we model. Reported once rather than per write,
             * because a title that does this does it every frame. */
            LOG_WARN_ONCE(LOG_VIDEO, "XF write to %04x is outside modelled memory",
                          a);
            continue;
        }
        g->xf.mem[a] = values[i];
        xf_decode(&g->xf, a);
    }
}

/* Bumped on every BP write, the counterpart of g_xf_generation.
 *
 * The renderer's shader lookup hashes the whole XF and TEV state on EVERY
 * draw, and the hash walk -- not the table probe -- is what it costs. A title
 * sets a material and issues many draws with it, so the hash almost always
 * produces the answer it produced last time. With both generations available
 * the lookup can tell "nothing has changed" in two compares. */
u32 g_bp_generation;

static void on_load_bp(void *ctx, u8 reg, u32 value)
{
    g_bp_generation++;
    GXState *g = (GXState *)ctx;
    u64 before = g->bp.copies;

    bp_write(&g->bp, reg, value);

    /* An EFB copy is the only BP write with a side effect, and it is the one
     * that makes anything visible, so it is forwarded rather than left for the
     * backend to notice by polling. */
    if (g->bp.copies != before && g->backend.efb_copy)
        g->backend.efb_copy(g->backend.ctx, g, &g->bp.copy);
}

static void on_draw(void *ctx, GXPrimitive prim, unsigned vat,
                    u16 vertex_count, u32 data_addr, u32 vertex_size)
{
    GXState *g = (GXState *)ctx;

    if (!g->backend.draw) {
        g->draws_dropped++;
        return;
    }
    g->backend.draw(g->backend.ctx, g, prim, vat, vertex_count,
                    data_addr, vertex_size);
}

static void on_load_index(void *ctx, u8 which, u16 index, u16 xf_addr, u8 count)
{
    g_xf_generation++;
    GXState *g = (GXState *)ctx;
    const GXCPRegs *cp = &g->parser.cp;
    unsigned array;
    u32 src;
    unsigned i;

    /* The four index commands name four different arrays. Their numbering is
     * the hardware's and does not follow from the opcode arithmetically, which
     * is why it is a table rather than a shift. */
    switch (which) {
    /* CP arrays 12-15, NOT 0-3. Arrays 0-11 are the vertex ATTRIBUTE arrays
     * (positions, normals, colours, texcoords); the four indexed-XF commands
     * have their own bases at 12-15 (Dolphin: CPArray::XF_A..XF_D). Reading
     * from 0-3 fetched "matrices" out of the position/normal vertex data --
     * every skinned or matrix-palette object (i.e. ALL 3D models) transformed
     * by garbage and left the screen: menus survived because their direct
     * XF loads use matrix 0 via the command stream. */
    case GX_LOAD_INDX_A: array = 12; break;  /* position matrices */
    case GX_LOAD_INDX_B: array = 13; break;  /* normal matrices   */
    case GX_LOAD_INDX_C: array = 14; break;  /* texture matrices  */
    case GX_LOAD_INDX_D: array = 15; break;  /* light objects     */
    default: return;
    }

    src = cp->array_base[array] + (u32)index * cp->array_stride[array];

    for (i = 0; i < count; i++) {
        u16 a = (u16)(xf_addr + i);
        if (a >= XF_MEM_SIZE)
            break;
        g->xf.mem[a] = mem_read32(src + i * 4u);
        xf_decode(&g->xf, a);
    }
}

/* ------------------------------------------------------------------ */

static u16 gx_guest_read16(u32 addr) { return mem_read16(addr); }

void gx_state_init(GXState *g, const GXBackend *backend)
{
    GXSink sink;

    memset(g, 0, sizeof *g);
    /* After the wipe, or it would be cleared: the palette loader needs a way
     * to read guest memory and gx_state is the layer that legitimately has
     * one. */
    g->bp.read16 = gx_guest_read16;
    if (backend)
        g->backend = *backend;

    memset(&sink, 0, sizeof sink);
    sink.ctx        = g;
    sink.load_xf    = on_load_xf;
    sink.load_bp    = on_load_bp;
    sink.load_index = on_load_index;
    sink.draw       = on_draw;
    /* No call_list hook: the parser recurses into display lists itself, which
     * is what the hardware does. A backend that wants to cache them can set one
     * later. */

    gx_parser_init(&g->parser, &sink);
    bp_reset(&g->bp);
}

void gx_state_reset(GXState *g)
{
    GXBackend saved = g->backend;
    gx_state_init(g, &saved);
}

u32 gx_state_run(GXState *g)
{
    return gx_parser_run(&g->parser);
}

u32 gx_state_run_list(GXState *g, u32 addr, u32 size)
{
    return gx_parser_run_memory(&g->parser, addr, size);
}
