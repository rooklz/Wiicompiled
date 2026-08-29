/* texture_decode.c — GX texture formats to RGBA8.
 *
 * See texture_decode.h for why tiling is the thing to be careful about.
 *
 * The decoders are written as "for each tile, for each pixel in the tile",
 * rather than "for each output pixel, compute its source address". Both work;
 * the first is the one where the tile geometry appears once per format instead
 * of being folded into an address expression, which is what makes an error
 * visible when reading the code rather than only when looking at the picture.
 */
#include "texture_decode.h"
#include "../../core/mem/memmap.h"
#include "../../common/log.h"

/* ------------------------------------------------------------------ */
/* Geometry                                                             */
/*                                                                      */
/* Tile size follows bit depth, not format: 4 bits per pixel gives 8x8, 8 bits  */
/* gives 8x4, 16 bits gives 4x4. Every tile is therefore 32 bytes, except       */
/* RGBA8 (two 32-byte halves) and CMPR (four compressed 4x4 blocks).            */
/* ------------------------------------------------------------------ */

unsigned tex_block_width(GXTextureFormat fmt)
{
    switch (fmt) {
    case GX_TF_I4: case GX_TF_C4: case GX_TF_CMPR: return 8;
    case GX_TF_I8: case GX_TF_IA4: case GX_TF_C8:  return 8;
    case GX_TF_IA8: case GX_TF_RGB565: case GX_TF_RGB5A3:
    case GX_TF_RGBA8: case GX_TF_C14X2:            return 4;
    default:                                       return 0;
    }
}

unsigned tex_block_height(GXTextureFormat fmt)
{
    switch (fmt) {
    case GX_TF_I4: case GX_TF_C4: case GX_TF_CMPR: return 8;
    case GX_TF_I8: case GX_TF_IA4: case GX_TF_C8:  return 4;
    case GX_TF_IA8: case GX_TF_RGB565: case GX_TF_RGB5A3:
    case GX_TF_RGBA8: case GX_TF_C14X2:            return 4;
    default:                                       return 0;
    }
}

unsigned tex_block_bytes(GXTextureFormat fmt)
{
    switch (fmt) {
    case GX_TF_RGBA8: return 64;    /* two 32-byte halves: AR then GB */
    case GX_TF_CMPR:  return 32;    /* four 8-byte DXT1 blocks         */
    case GX_TF_I4: case GX_TF_C4:
    case GX_TF_I8: case GX_TF_IA4: case GX_TF_C8:
    case GX_TF_IA8: case GX_TF_RGB565: case GX_TF_RGB5A3:
    case GX_TF_C14X2: return 32;
    default:          return 0;
    }
}

u32 tex_image_bytes(GXTextureFormat fmt, unsigned width, unsigned height)
{
    unsigned bw = tex_block_width(fmt), bh = tex_block_height(fmt);
    unsigned cols, rows;

    if (!bw || !bh)
        return 0;

    /* Rounded up to whole tiles. A title is free to declare a 33 x 17 texture,
     * and the storage is still whole tiles -- computing size from the declared
     * dimensions alone under-reads the last row and column. */
    cols = (width  + bw - 1) / bw;
    rows = (height + bh - 1) / bh;
    return (u32)cols * rows * tex_block_bytes(fmt);
}

/* ------------------------------------------------------------------ */
/* Colour conversion                                                    */
/* ------------------------------------------------------------------ */

/* 5- and 6-bit channels are expanded by replicating their high bits into the
 * low ones, not by shifting left. Shifting maps full-scale 31 to 248 rather
 * than 255, so every white in every texture comes out slightly grey -- visible
 * as a wash over the whole frame, and easy to mistake for a gamma problem. */
static u32 expand5(u32 v) { return (v << 3) | (v >> 2); }
static u32 expand6(u32 v) { return (v << 2) | (v >> 4); }
static u32 expand4(u32 v) { return (v << 4) | v; }
static u32 expand3(u32 v) { return (v << 5) | (v << 2) | (v >> 1); }

static u32 rgba(u32 r, u32 g, u32 b, u32 a)
{
    return (r << 24) | (g << 16) | (b << 8) | a;
}

u32 tex_rgb565_to_rgba8(u16 v)
{
    return rgba(expand5((v >> 11) & 0x1Fu),
                expand6((v >> 5)  & 0x3Fu),
                expand5(v & 0x1Fu), 255);
}

/* RGB5A3 is two formats sharing an encoding, selected by the top bit: set means
 * opaque RGB555, clear means ARGB3444. Treating it as one format gives either
 * uniformly wrong colours or uniformly wrong alpha, depending which half is
 * assumed. */
u32 tex_rgb5a3_to_rgba8(u16 v)
{
    if (v & 0x8000u)
        return rgba(expand5((v >> 10) & 0x1Fu),
                    expand5((v >> 5)  & 0x1Fu),
                    expand5(v & 0x1Fu), 255);

    return rgba(expand4((v >> 8) & 0x0Fu),
                expand4((v >> 4) & 0x0Fu),
                expand4(v & 0x0Fu),
                expand3((v >> 12) & 0x07u));
}

/* 3/8 v1 + 5/8 v2, the blend Flipper's CMPR unit performs where DXT1 would
 * interpolate exactly. Written as a named function because "(v1 * 3 + v2 * 5)
 * >> 3" inline in a palette expression is indistinguishable from a typo. */
static u32 blend38(u32 v1, u32 v2) { return (v1 * 3u + v2 * 5u) >> 3; }

static u32 intensity(u32 i)      { return rgba(i, i, i, i); }
static u32 intensity_a(u32 i, u32 a) { return rgba(i, i, i, a); }

/* ------------------------------------------------------------------ */
/* Decoders                                                             */
/* ------------------------------------------------------------------ */

/* A function, not a macro, and that distinction is load-bearing.
 *
 * The callers pass `mem_read8(p++)` as the value. Inside a macro that tests the
 * bounds first, the read -- and the increment -- only happen for pixels that
 * land inside the image, so a tile hanging over the right edge silently skips
 * its padding bytes and every row after it is shifted. The result is a sheared
 * texture, on exactly the images whose width is not a multiple of the tile
 * width, which is most user-interface art. A function call evaluates its
 * arguments whatever the bounds turn out to be. */
static void put(u32 *out, unsigned width, unsigned height,
                unsigned x, unsigned y, u32 v)
{
    if (x < width && y < height)
        out[y * width + x] = v;
}

#define PUT(x, y, v) put(out, width, height, (x), (y), (v))

/* Tiles that fall partly outside the image are decoded and clipped rather than
 * skipped: their in-range pixels are real image data, and their padding still
 * occupies bytes that must be stepped over. */

static void decode_i4(u32 addr, unsigned width, unsigned height, u32 *out)
{
    unsigned ty, tx, py, px;
    u32 p = addr;
    for (ty = 0; ty < height; ty += 8)
        for (tx = 0; tx < width; tx += 8)
            for (py = 0; py < 8; py++)
                for (px = 0; px < 8; px += 2) {
                    u8 b = mem_read8(p++);
                    PUT(tx + px,     ty + py, intensity(expand4(b >> 4)));
                    PUT(tx + px + 1, ty + py, intensity(expand4(b & 0xFu)));
                }
}

static void decode_i8(u32 addr, unsigned width, unsigned height, u32 *out)
{
    unsigned ty, tx, py, px;
    u32 p = addr;
    for (ty = 0; ty < height; ty += 4)
        for (tx = 0; tx < width; tx += 8)
            for (py = 0; py < 4; py++)
                for (px = 0; px < 8; px++)
                    PUT(tx + px, ty + py, intensity(mem_read8(p++)));
}

static void decode_ia4(u32 addr, unsigned width, unsigned height, u32 *out)
{
    unsigned ty, tx, py, px;
    u32 p = addr;
    for (ty = 0; ty < height; ty += 4)
        for (tx = 0; tx < width; tx += 8)
            for (py = 0; py < 4; py++)
                for (px = 0; px < 8; px++) {
                    u8 b = mem_read8(p++);
                    /* Alpha is the *high* nibble. */
                    PUT(tx + px, ty + py,
                        intensity_a(expand4(b & 0xFu), expand4(b >> 4)));
                }
}

static void decode_ia8(u32 addr, unsigned width, unsigned height, u32 *out)
{
    unsigned ty, tx, py, px;
    u32 p = addr;
    for (ty = 0; ty < height; ty += 4)
        for (tx = 0; tx < width; tx += 4)
            for (py = 0; py < 4; py++)
                for (px = 0; px < 4; px++) {
                    u16 v = mem_read16(p); p += 2;
                    /* Alpha in the high byte, intensity in the low. */
                    PUT(tx + px, ty + py,
                        intensity_a(v & 0xFFu, v >> 8));
                }
}

static void decode_16bit(u32 addr, unsigned width, unsigned height, u32 *out,
                         int is_5a3)
{
    unsigned ty, tx, py, px;
    u32 p = addr;
    for (ty = 0; ty < height; ty += 4)
        for (tx = 0; tx < width; tx += 4)
            for (py = 0; py < 4; py++)
                for (px = 0; px < 4; px++) {
                    u16 v = mem_read16(p); p += 2;
                    PUT(tx + px, ty + py,
                        is_5a3 ? tex_rgb5a3_to_rgba8(v)
                               : tex_rgb565_to_rgba8(v));
                }
}

/* RGBA8's tile is 64 bytes in two halves: the first 32 hold alpha and red
 * interleaved, the second 32 hold green and blue. Reading it as 32-bit pixels
 * gives an image whose colours are drawn from two different parts of the tile
 * -- recognisable, and completely wrong. */
static void decode_rgba8(u32 addr, unsigned width, unsigned height, u32 *out)
{
    unsigned ty, tx, py, px;
    u32 p = addr;
    for (ty = 0; ty < height; ty += 4)
        for (tx = 0; tx < width; tx += 4) {
            u32 ar = p, gb = p + 32;
            for (py = 0; py < 4; py++)
                for (px = 0; px < 4; px++) {
                    unsigned i = py * 4 + px;
                    u32 a = mem_read8(ar + i * 2);
                    u32 r = mem_read8(ar + i * 2 + 1);
                    u32 g = mem_read8(gb + i * 2);
                    u32 b = mem_read8(gb + i * 2 + 1);
                    PUT(tx + px, ty + py, rgba(r, g, b, a));
                }
            p += 64;
        }
}

/* CMPR is DXT1 with two differences that both matter: the 8x8 tile holds four
 * 4x4 sub-blocks, and the 16-bit endpoints and index word are big-endian.
 * Feeding the bytes to a stock DXT1 decoder gets both wrong. */
static void decode_cmpr_block(u32 addr, unsigned x0, unsigned y0,
                              unsigned width, unsigned height, u32 *out)
{
    u16 c0 = mem_read16(addr);
    u16 c1 = mem_read16(addr + 2);
    u32 bits = mem_read32(addr + 4);
    u32 palette[4];
    unsigned py, px;

    palette[0] = tex_rgb565_to_rgba8(c0);
    palette[1] = tex_rgb565_to_rgba8(c1);

    {
        u32 r0 = (palette[0] >> 24) & 0xFF, g0 = (palette[0] >> 16) & 0xFF,
            b0 = (palette[0] >> 8) & 0xFF;
        u32 r1 = (palette[1] >> 24) & 0xFF, g1 = (palette[1] >> 16) & 0xFF,
            b1 = (palette[1] >> 8) & 0xFF;

    /* Which of the two interpolation modes applies is decided by comparing the
     * raw 16-bit endpoints, not the expanded colours: two distinct 565 values
     * can expand to the same RGBA8, and comparing after expansion would then
     * pick the wrong mode. */
    if (c0 > c1) {
        /* Four opaque colours. The middle two are *not* at 1/3 and 2/3:
         * Flipper's decompressor blends 5/8 and 3/8, an approximation to a
         * third that it never corrects. Decoding with the exact thirds a DXT1
         * decoder uses shifts every interpolated texel by up to 4% of the
         * endpoint separation -- which is a wash over gradients and skies
         * rather than a localised artefact, and therefore reads as "our
         * colours are slightly off" with nothing to point at. */
        palette[2] = rgba(blend38(r1, r0), blend38(g1, g0), blend38(b1, b0), 255);
        palette[3] = rgba(blend38(r0, r1), blend38(g0, g1), blend38(b0, b1), 255);
    } else {
        /* Three colours and a transparent index. The transparent one is the
         * *midpoint colour* with zero alpha, not transparent black -- this is
         * where Flipper differs from DXT1, and it matters as soon as anything
         * filters the texture: a bilinear tap that straddles a cut-out edge
         * mixes the invisible texel's RGB into a visible pixel, so transparent
         * black draws a dark fringe around every alpha-tested sprite. */
        u32 mr = (r0 + r1) / 2, mg = (g0 + g1) / 2, mb = (b0 + b1) / 2;
        palette[2] = rgba(mr, mg, mb, 255);
        palette[3] = rgba(mr, mg, mb, 0);
    }
    }

    for (py = 0; py < 4; py++)
        for (px = 0; px < 4; px++) {
            /* Two bits per pixel, most significant pair first. */
            unsigned shift = 30 - (py * 4 + px) * 2;
            unsigned idx = (bits >> shift) & 3u;
            PUT(x0 + px, y0 + py, palette[idx]);
        }
}

static void decode_cmpr(u32 addr, unsigned width, unsigned height, u32 *out)
{
    unsigned ty, tx;
    u32 p = addr;
    for (ty = 0; ty < height; ty += 8)
        for (tx = 0; tx < width; tx += 8) {
            /* The four sub-blocks are in raster order within the tile. */
            decode_cmpr_block(p,      tx,     ty,     width, height, out);
            decode_cmpr_block(p + 8,  tx + 4, ty,     width, height, out);
            decode_cmpr_block(p + 16, tx,     ty + 4, width, height, out);
            decode_cmpr_block(p + 24, tx + 4, ty + 4, width, height, out);
            p += 32;
        }
}

/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* Paletted formats                                                     */
/*                                                                      */
/* C4, C8 and C14X2 store an INDEX per texel; the colour comes from the
 * palette a title loaded into TLUT memory. The palette's own format is
 * chosen by the texture's TLUT register: IA8, RGB565 or RGB5A3.
 * These were previously refused outright, so anything paletted drew as a
 * flat error colour.                                                    */

static u32 tlut_lookup(const u16 *pal, unsigned n, unsigned idx, unsigned fmt)
{
    u16 v = (idx < n) ? pal[idx] : 0;
    switch (fmt) {
    case 0: {   /* IA8: intensity in the low byte, alpha in the high byte */
        u32 i = v & 0xFFu, a = (v >> 8) & 0xFFu;
        return (i << 24) | (i << 16) | (i << 8) | a;
    }
    case 1:     /* RGB565 */
        return tex_rgb565_to_rgba8(v);
    default:    /* RGB5A3 */
        return tex_rgb5a3_to_rgba8(v);
    }
}

static void decode_c4(u32 addr, unsigned width, unsigned height, u32 *out,
                      const u16 *pal, unsigned pn, unsigned pfmt)
{
    unsigned ty, tx, py, px;
    u32 p = addr;
    /* 8x8 tiles, two texels per byte, high nibble first. */
    for (ty = 0; ty < height; ty += 8)
        for (tx = 0; tx < width; tx += 8)
            for (py = 0; py < 8; py++)
                for (px = 0; px < 8; px += 2) {
                    u8 b = mem_read8(p++);
                    unsigned x0 = tx + px, x1 = tx + px + 1, y = ty + py;
                    if (x0 < width && y < height)
                        out[y * width + x0] = tlut_lookup(pal, pn, b >> 4, pfmt);
                    if (x1 < width && y < height)
                        out[y * width + x1] = tlut_lookup(pal, pn, b & 0xF, pfmt);
                }
}

static void decode_c8(u32 addr, unsigned width, unsigned height, u32 *out,
                      const u16 *pal, unsigned pn, unsigned pfmt)
{
    unsigned ty, tx, py, px;
    u32 p = addr;
    for (ty = 0; ty < height; ty += 4)          /* 8x4 tiles */
        for (tx = 0; tx < width; tx += 8)
            for (py = 0; py < 4; py++)
                for (px = 0; px < 8; px++) {
                    u8 b = mem_read8(p++);
                    unsigned x = tx + px, y = ty + py;
                    if (x < width && y < height)
                        out[y * width + x] = tlut_lookup(pal, pn, b, pfmt);
                }
}

static void decode_c14x2(u32 addr, unsigned width, unsigned height, u32 *out,
                         const u16 *pal, unsigned pn, unsigned pfmt)
{
    unsigned ty, tx, py, px;
    u32 p = addr;
    for (ty = 0; ty < height; ty += 4)          /* 4x4 tiles, 16 bits each */
        for (tx = 0; tx < width; tx += 4)
            for (py = 0; py < 4; py++)
                for (px = 0; px < 4; px++) {
                    u16 v = mem_read16(p); p += 2;
                    unsigned x = tx + px, y = ty + py;
                    if (x < width && y < height)
                        out[y * width + x] =
                            tlut_lookup(pal, pn, v & 0x3FFFu, pfmt);
                }
}

int tex_decode_paletted(GXTextureFormat fmt, u32 addr, unsigned width,
                        unsigned height, u32 *out, const u16 *pal,
                        unsigned pal_entries, unsigned pal_fmt)
{
    if (!pal) return -1;
    switch (fmt) {
    case GX_TF_C4:     decode_c4(addr, width, height, out, pal, pal_entries, pal_fmt); return 0;
    case GX_TF_C8:     decode_c8(addr, width, height, out, pal, pal_entries, pal_fmt); return 0;
    case GX_TF_C14X2:  decode_c14x2(addr, width, height, out, pal, pal_entries, pal_fmt); return 0;
    default: return -1;
    }
}

/* Which GX texture formats the title actually uses, and how many texels of
 * each. The point is the same question the vertex counter answered: how much
 * of this work the RSX could take natively. CMPR is a pure bit repack into
 * DXT1 and RGB565 is bit-identical to R5G6B5, so those need no conversion at
 * all; the paletted formats (C4/C8/C14X2) must be expanded because NV40
 * deleted paletted texture hardware; the rest are per-texel conversions. */
unsigned long long g_tex_fmt_texels[16], g_tex_fmt_calls[16];

int tex_decode(GXTextureFormat fmt, u32 addr, unsigned width, unsigned height,
               u32 *out)
{
    if (!width || !height || !out)
        return -1;
    if ((unsigned)fmt < 16) {
        g_tex_fmt_calls[(unsigned)fmt]++;
        g_tex_fmt_texels[(unsigned)fmt] += (unsigned long long)width * height;
    }

    switch (fmt) {
    case GX_TF_I4:     decode_i4(addr, width, height, out);         return 0;
    case GX_TF_I8:     decode_i8(addr, width, height, out);         return 0;
    case GX_TF_IA4:    decode_ia4(addr, width, height, out);        return 0;
    case GX_TF_IA8:    decode_ia8(addr, width, height, out);        return 0;
    case GX_TF_RGB565: decode_16bit(addr, width, height, out, 0);   return 0;
    case GX_TF_RGB5A3: decode_16bit(addr, width, height, out, 1);   return 0;
    case GX_TF_RGBA8:  decode_rgba8(addr, width, height, out);      return 0;
    case GX_TF_CMPR:   decode_cmpr(addr, width, height, out);       return 0;
    default:
        /* Palettised formats need the TLUT, which is loaded separately and is
         * not modelled yet. Reported rather than silently producing black, so a
         * title using them is identifiable. */
        LOG_WARN_ONCE(LOG_VIDEO, "texture format %u is not decoded yet",
                      (unsigned)fmt);
        return -1;
    }
}
