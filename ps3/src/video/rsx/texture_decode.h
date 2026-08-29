/* texture_decode.h — GX texture formats to RGBA8.
 *
 * Every texture the Wii samples is stored *tiled*: the image is cut into small
 * rectangles and those rectangles are laid out one after another, so
 * consecutive bytes in memory are not consecutive pixels on a row. The tile
 * size depends on the bit depth, which means each format has its own
 * address arithmetic.
 *
 * That is the thing to be careful about here, and the reason for the shape of
 * the tests. A wrong tile size does not produce noise — it produces the right
 * image with its blocks shuffled, which still looks like a texture, still has
 * the right colours and histogram, and is obvious only if you are looking at
 * it. There is no assertion a decoder can make about its own output that would
 * catch it. So the tests build images whose every pixel encodes its own
 * coordinates, and check each pixel individually: a shuffled block then fails
 * loudly at the first pixel that moved.
 *
 * Output is straight RGBA8, one word per pixel, top row first. Converting that
 * to whatever the RSX wants is the backend's problem; this file is pure
 * arithmetic over memory and is therefore testable anywhere.
 */
#ifndef DOLPHIN_VIDEO_RSX_TEXTURE_DECODE_H
#define DOLPHIN_VIDEO_RSX_TEXTURE_DECODE_H

#include "../../common/types.h"

/* Texture formats, by their hardware number (BP TX_SETIMAGE0 bits 20-23). */
typedef enum {
    GX_TF_I4     = 0x0,
    GX_TF_I8     = 0x1,
    GX_TF_IA4    = 0x2,
    GX_TF_IA8    = 0x3,
    GX_TF_RGB565 = 0x4,
    GX_TF_RGB5A3 = 0x5,
    GX_TF_RGBA8  = 0x6,
    GX_TF_C4     = 0x8,
    GX_TF_C8     = 0x9,
    GX_TF_C14X2  = 0xA,
    GX_TF_CMPR   = 0xE
} GXTextureFormat;

/* Tile dimensions for a format. Returns 0 for a format that is not modelled,
 * which the caller must treat as "cannot decode" rather than "zero-sized". */
unsigned tex_block_width(GXTextureFormat fmt);
unsigned tex_block_height(GXTextureFormat fmt);

/* Bytes one tile occupies. */
unsigned tex_block_bytes(GXTextureFormat fmt);

/* Total bytes a width x height image occupies, including the padding implied by
 * rounding up to whole tiles -- which is where a "how big is this texture"
 * calculation usually goes wrong, because the dimensions a title sets are not
 * required to be multiples of the tile size. */
u32 tex_image_bytes(GXTextureFormat fmt, unsigned width, unsigned height);

/* Decode into `out`, which must hold width * height u32s. Returns 0 on success,
 * non-zero for a format that is not modelled. Reads guest memory. */
int tex_decode(GXTextureFormat fmt, u32 addr, unsigned width, unsigned height,
               u32 *out);

/* The paletted formats (C4, C8, C14X2) need the palette a title loaded into
 * TLUT memory, so they take it explicitly. pal_fmt is the TLUT format field:
 * 0 = IA8, 1 = RGB565, 2 = RGB5A3. Returns 0 on success. */
int tex_decode_paletted(GXTextureFormat fmt, u32 addr, unsigned width,
                        unsigned height, u32 *out, const u16 *pal,
                        unsigned pal_entries, unsigned pal_fmt);

/* The two 16-bit colour encodings, exposed because both have a case that is
 * easy to get subtly wrong and worth testing directly: RGB5A3 switches meaning
 * on its top bit, and RGB565's green has one more bit than red or blue. */
u32 tex_rgb565_to_rgba8(u16 v);
u32 tex_rgb5a3_to_rgba8(u16 v);

#endif /* DOLPHIN_VIDEO_RSX_TEXTURE_DECODE_H */
