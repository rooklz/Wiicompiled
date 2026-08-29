/* xfb_present.c -- display the video interface's framebuffer.
 *
 * A Wii title does not always render through GX. Video playback (MKWii's
 * attract movie is THP) decodes frames on the CPU and writes them straight
 * into the external framebuffer, then points the video interface at it; the
 * 3D pipeline is not involved at all. This emulator only ever presented what
 * GX drew, so those frames were never displayed -- which is why the movie
 * appeared as whatever happened to be in the render target.
 *
 * The XFB is NOT RGB. It is YUV 4:2:2 in the pattern Y0 Cb Y1 Cr: two pixels
 * per four bytes, sharing one chroma sample. Presenting those bytes as RGB is
 * exactly the sort of thing that produces a uniform magenta screen.
 *
 * Conversion follows the ITU-R BT.601 relation the console's video encoder
 * uses, with the studio-swing ranges GX writes (Y 16..235, C 16..240).
 */

#include <string.h>

#include "xfb_present.h"
#include "rsx_video.h"
#include "../../core/mem/memmap.h"
#include "../../common/log.h"

static int clamp255(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }

/* One YUV pair -> two RGBA pixels. */
static void yuv_pair_to_rgba(int y0, int cb, int y1, int cr, u32 *out)
{
    /* BT.601, studio swing: scale luma by 255/219 and chroma by 255/224. */
    int c0 = y0 - 16, c1 = y1 - 16, d = cb - 128, e = cr - 128;
    int r0 = (298 * c0 + 409 * e + 128) >> 8;
    int g0 = (298 * c0 - 100 * d - 208 * e + 128) >> 8;
    int b0 = (298 * c0 + 516 * d + 128) >> 8;
    int r1 = (298 * c1 + 409 * e + 128) >> 8;
    int g1 = (298 * c1 - 100 * d - 208 * e + 128) >> 8;
    int b1 = (298 * c1 + 516 * d + 128) >> 8;
    out[0] = 0xFF000000u | ((u32)clamp255(r0) << 16) |
             ((u32)clamp255(g0) << 8) | (u32)clamp255(b0);
    out[1] = 0xFF000000u | ((u32)clamp255(r1) << 16) |
             ((u32)clamp255(g1) << 8) | (u32)clamp255(b1);
}

int xfb_present(u32 xfb_addr, unsigned width, unsigned height,
                u32 *dst, unsigned dst_pitch_words,
                unsigned dst_width, unsigned dst_height)
{
    unsigned y, x;
    u32 rgba[2];

    if (!xfb_addr || !dst || width < 2 || height < 2)
        return -1;
    if (width > 1024 || height > 1024)
        return -1;

    for (y = 0; y < dst_height; y++) {
        /* Nearest-neighbour scale: the XFB is 640x480-ish and the display is
         * 1920x1080. A filtered scale belongs on the GPU, not here. */
        unsigned sy = (unsigned)((u64)y * height / dst_height);
        u32 row = xfb_addr + sy * width * 2u;
        u32 *o  = dst + (size_t)y * dst_pitch_words;
        for (x = 0; x + 1 < dst_width; x += 2) {
            unsigned sx = (unsigned)((u64)x * width / dst_width) & ~1u;
            u32 p = row + sx * 2u;
            yuv_pair_to_rgba(mem_read8(p), mem_read8(p + 1),
                             mem_read8(p + 2), mem_read8(p + 3), rgba);
            o[x]     = rgba[0];
            o[x + 1] = rgba[1];
        }
    }
    return 0;
}
