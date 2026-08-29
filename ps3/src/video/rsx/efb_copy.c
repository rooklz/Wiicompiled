/* efb_copy.c — see efb_copy.h.
 *
 * Deliberately free of any GPU dependency: this is address arithmetic and a
 * sixteen-entry table, so it links into the qemu harness and the host tests
 * exactly as it links into the console build. The RSX side -- allocating the
 * surface and resolving the render target into it -- lives in gx_render.c,
 * which keeps its surfaces in an array indexed by efb_copy_index().
 */
#include "efb_copy.h"
#include "../../core/mem/memmap.h"

#include <stdio.h>
#include <string.h>

EfbCopyStats g_efb_copy;

static EfbCopyTarget s_targets[EFB_COPY_TARGETS];
static u32           s_serial;

/* ------------------------------------------------------------------ */

u8 efb_copy_texture_format(unsigned copy_fmt, unsigned intensity)
{
    /* The copy format field names *channel selections*, not texture
     * encodings: 8 is "the red channel as one byte", 0xC is "green and blue
     * as two". What the pixel engine writes to memory is nevertheless always
     * one of the ordinary texture encodings, and this is the mapping between
     * them. It matters because the destination's tile geometry -- and
     * therefore how many bytes the copy touches -- comes from the texture
     * format, not from the copy format.
     *
     * `intensity` selects luminance arithmetic on the way out; it does not
     * change which encoding is written, so it does not change the geometry.
     * It is taken as a parameter anyway because a caller that has the bit
     * should not have to know that. */
    (void)intensity;
    switch (copy_fmt & 0xFu) {
    case 0x0: return (u8)GX_TF_I4;      /* R4    */
    case 0x1: return (u8)GX_TF_I8;      /* R8 (the "_1" alias) */
    case 0x2: return (u8)GX_TF_IA4;     /* RA4   */
    case 0x3: return (u8)GX_TF_IA8;     /* RA8   */
    case 0x4: return (u8)GX_TF_RGB565;
    case 0x5: return (u8)GX_TF_RGB5A3;
    case 0x6: return (u8)GX_TF_RGBA8;
    case 0x7: return (u8)GX_TF_I8;      /* A8    */
    case 0x8: return (u8)GX_TF_I8;      /* R8    */
    case 0x9: return (u8)GX_TF_I8;      /* G8    */
    case 0xA: return (u8)GX_TF_I8;      /* B8    */
    case 0xB: return (u8)GX_TF_IA8;     /* RG8   */
    case 0xC: return (u8)GX_TF_IA8;     /* GB8   */
    default:  return 0xFFu;             /* 0xD-0xF: no texture equivalent   */
    }
}

/* Destination dimensions. `half_scale` is the pixel engine's mipmap path: the
 * copy averages 2x2 EFB pixels into one destination texel, so the destination
 * is half the size of the source rectangle in each axis. Getting this wrong
 * does not corrupt the image, it mis-sizes the allocation and the extent --
 * which is worse, because the extent is what a later texture bind is matched
 * against. */
static void dest_size(const BPCopy *c, unsigned *w, unsigned *h)
{
    if (c->half_scale) {
        *w = c->width  >> 1;
        *h = c->height >> 1;
        if (*w == 0) *w = 1;
        if (*h == 0) *h = 1;
    } else {
        *w = c->width;
        *h = c->height;
    }
}

u32 efb_copy_dest_bytes(const BPCopy *c)
{
    u8 fmt = efb_copy_texture_format(c->format, c->intensity);
    unsigned bh, w, h, rows;

    if (fmt == 0xFFu)
        return 0;
    bh = tex_block_height((GXTextureFormat)fmt);
    if (!bh)
        return 0;

    dest_size(c, &w, &h);
    rows = (h + bh - 1u) / bh;

    /* The stride register is bytes per *row of tiles*, which is why it is
     * multiplied by the number of tile rows rather than by the pixel height.
     * A title that leaves the stride at zero is asking for the tightly packed
     * layout, so fall back to computing it. */
    if (c->dest_stride)
        return rows * c->dest_stride;
    return tex_image_bytes((GXTextureFormat)fmt, w, h);
}

/* Read through mem_ptr rather than mem_read32: an address that is not RAM
 * must read as nothing rather than as an MMIO access with side effects, and
 * a destination that is not mapped at all should turn the guard into a
 * harmless no-op instead of a stream of "read from unmapped" warnings. */
static u32 guard_word(u32 a)
{
    const void *p = mem_ptr(a);
    return p ? dol_be32(p) : 0u;
}

/* Sample the guard words. Spread across the range rather than clustered at
 * the start: a title that only rewrites the first tile would otherwise go
 * unnoticed, and a title that only rewrites the last one likewise. */
static void guard_sample(EfbCopyTarget *e)
{
    unsigned i;
    u32 step = e->bytes / EFB_COPY_GUARDS;
    for (i = 0; i < EFB_COPY_GUARDS; i++) {
        u32 a = e->addr + (u32)i * step;
        a &= ~3u;
        if (a + 4u > e->addr + e->bytes)
            a = (e->addr + e->bytes - 4u) & ~3u;
        e->guard_at[i] = a;
        e->guard[i]    = guard_word(a);
    }
}

static int guard_intact(const EfbCopyTarget *e)
{
    unsigned i;
    for (i = 0; i < EFB_COPY_GUARDS; i++)
        if (guard_word(e->guard_at[i]) != e->guard[i])
            return 0;
    return 1;
}

static int fmt_has_alpha(u8 fmt)
{
    switch ((GXTextureFormat)fmt) {
    case GX_TF_IA4: case GX_TF_IA8: case GX_TF_RGB5A3: case GX_TF_RGBA8:
        return 1;
    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ */

EfbCopyTarget *efb_copy_note(const BPCopy *c)
{
    u8  fmt   = efb_copy_texture_format(c->format, c->intensity);
    u32 bytes = efb_copy_dest_bytes(c);
    unsigned w, h, i;
    EfbCopyTarget *e = NULL;

    if (fmt == 0xFFu || bytes == 0 || c->dest_addr == 0) {
        g_efb_copy.copies_unmodelled++;
        return NULL;
    }

    dest_size(c, &w, &h);

    /* An exact destination match is a *replacement*, not a second entry: the
     * licence screen re-renders the same Mii into the same buffer every time
     * the selection moves, and keeping both would leave the stale one to be
     * found first. */
    for (i = 0; i < EFB_COPY_TARGETS; i++)
        if (s_targets[i].valid && s_targets[i].addr == c->dest_addr) {
            e = &s_targets[i];
            break;
        }
    if (!e)
        for (i = 0; i < EFB_COPY_TARGETS; i++)
            if (!s_targets[i].valid) { e = &s_targets[i]; break; }
    if (!e) {
        /* Full: evict the entry written longest ago.
         *
         * Refusing instead would be the safer-sounding choice and is the wrong
         * one, because the destination addresses are not stable. Mario Kart
         * Wii allocates a fresh pair of buffers for every Mii it renders
         * (0x1213f980, then 0x1211e020, then 0x124c8940 in one boot), so a
         * table that refuses fills up with dead addresses and then drops
         * exactly the copies the game is about to sample. Least-recently-
         * written is the right victim for the same reason: a destination that
         * has not been re-copied in a long time is the one the title has
         * finished with. The cost of being wrong is one frame of a texture
         * that has moved on, not a decode of memory nothing ever wrote. */
        unsigned oldest = 0;
        for (i = 1; i < EFB_COPY_TARGETS; i++)
            if (s_targets[i].serial < s_targets[oldest].serial)
                oldest = i;
        e = &s_targets[oldest];
        memset(e, 0, sizeof *e);
        g_efb_copy.evictions++;
    }

    /* Everything is overwritten unconditionally, including the geometry: the
     * backend compares its surface against the new size and grows it if the
     * copy got bigger. */
    e->addr      = c->dest_addr;
    e->bytes     = bytes;
    e->stride    = c->dest_stride;
    e->width     = (u16)w;
    e->height    = (u16)h;
    e->src_x     = (u16)c->src_x;
    e->src_y     = (u16)c->src_y;
    e->fmt       = fmt;
    e->has_alpha = (u8)fmt_has_alpha(fmt);
    e->intensity = (u8)(c->intensity ? 1 : 0);
    e->valid     = 1;
    e->copies++;
    e->serial    = ++s_serial;
    guard_sample(e);
    g_efb_copy.copies_texture++;
    return e;
}

EfbCopyTarget *efb_copy_find(u32 addr, unsigned width, unsigned height,
                             unsigned fmt)
{
    unsigned i;
    EfbCopyTarget *contains = NULL;

    (void)fmt;
    if (addr == 0)
        return NULL;

    for (i = 0; i < EFB_COPY_TARGETS; i++) {
        EfbCopyTarget *e = &s_targets[i];
        if (!e->valid)
            continue;
        if (e->addr == addr) {
            /* An exact address match is the normal case. The declared size is
             * not required to agree: a title may bind a 64x64 copy target as
             * 64x64 (it does) but is also free to declare a smaller mip level
             * at the same address, and serving level 0 for it is far closer to
             * right than decoding bytes nothing wrote. */
            (void)width; (void)height;
            if (!guard_intact(e)) {
                e->valid = 0;
                g_efb_copy.guard_invalidations++;
                return NULL;
            }
            return e;
        }
        if (addr > e->addr && addr < e->addr + e->bytes)
            contains = e;
    }
    if (contains && !guard_intact(contains)) {
        contains->valid = 0;
        g_efb_copy.guard_invalidations++;
        return NULL;
    }
    return contains;
}

void efb_copy_invalidate_range(u32 addr, u32 bytes)
{
    unsigned i;
    u32 end = addr + bytes;
    for (i = 0; i < EFB_COPY_TARGETS; i++) {
        EfbCopyTarget *e = &s_targets[i];
        if (!e->valid)
            continue;
        if (addr < e->addr + e->bytes && end > e->addr) {
            e->valid = 0;
            g_efb_copy.cpu_invalidations++;
        }
    }
}

void efb_copy_drop(EfbCopyTarget *e)
{
    if (e) {
        memset(e, 0, sizeof *e);
    }
}

EfbCopyTarget *efb_copy_entry(unsigned i)
{
    return (i < EFB_COPY_TARGETS) ? &s_targets[i] : NULL;
}

unsigned efb_copy_index(const EfbCopyTarget *e)
{
    return (unsigned)(e - s_targets);
}

void efb_copy_reset(void)
{
    memset(s_targets, 0, sizeof s_targets);
    memset(&g_efb_copy, 0, sizeof g_efb_copy);
    s_serial = 0;
}

char *efb_copy_stats_line(char *buf, unsigned len)
{
    unsigned i, live = 0;
    for (i = 0; i < EFB_COPY_TARGETS; i++)
        if (s_targets[i].valid) live++;
    snprintf(buf, len,
             "EFBC %u/%u  tex %llu  bind %llu  stale %llu  cpuw %llu  "
             "evict %llu  unk %llu",
             live, (unsigned)EFB_COPY_TARGETS,
             (unsigned long long)g_efb_copy.copies_texture,
             (unsigned long long)g_efb_copy.binds_resolved,
             (unsigned long long)g_efb_copy.binds_stale,
             (unsigned long long)g_efb_copy.guard_invalidations,
             (unsigned long long)g_efb_copy.evictions,
             (unsigned long long)g_efb_copy.copies_unmodelled);
    return buf;
}
