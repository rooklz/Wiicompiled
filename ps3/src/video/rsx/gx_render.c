/* gx_render.c — see gx_render.h. */
#include "gx_render.h"
#include "rsx_video.h"
#include "rsx_shader.h"
#include "vertex_loader.h"
#include "spu_vtx_shared.h"
int vtx_build_spu_job(const GXCPRegs *cp, unsigned vat, u32 stream_addr,
                      unsigned count, u64 dest_ea, SpuVtxJob *job);
#include "tev_program.h"
#include "xf_program.h"
#include "vp_emitter.h"
#include "fp_emitter.h"
#include "../../common/log.h"
/* PHASE PROFILE: the renderer's own CPU cost, split four ways.  See main.c. */
#include "../../common/phase_prof.h"

#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <rsx/rsx.h>
#include <altivec.h>

extern gcmContextData *rsx_context(void);

GXRenderStats g_gx_render;
static u32 s_tex_frame;    /* bumped per presented frame for hash throttling */

/* ------------------------------------------------------------------ */
/* Texture cache                                                        */
/*                                                                      */
/* GX textures decode to RGBA8 (texture_decode.c) into RSX-visible memory,      */
/* once per distinct (address, format, size) -- menu art is loaded once and     */
/* sampled for thousands of frames, so decode cost is irrelevant and lookup     */
/* cost is a hash probe. The decoded bytes are R,G,B,A in memory; the sampler   */
/* reads A8R8G8B8, so a channel remap on the unit puts each component back.     */
/*                                                                              */
/* Invalidation is deliberately absent for now: nothing evicts, and CPU writes  */
/* to texture memory between draws are not tracked. Menus survive this;         */
/* render-to-texture effects will need an efb-copy hook when they matter.       */
/* ------------------------------------------------------------------ */

#include "texture_decode.h"
#include "efb_copy.h"

static u32 tex_swizzle_index(u32 x, u32 y, u32 w, u32 h);
static void tex_halve(const u32 *src, u32 w, u32 h, u32 *dst);

#define TEXCACHE_SLOTS 512

typedef struct {
    u32   addr;
    u16   width, height;
    u8    fmt;
    u8    valid;
    u32   offset;               /* RSX offset of the decoded pixels */
    u32   pitch;                /* bytes per row, 64-byte aligned      */
    u32  *pixels;               /* CPU view, for the debug thumbnails */
    u8    mips;                 /* uploaded mip levels (1 = no chain)  */
    u32   chash;                /* sampled content hash of the guest bytes */
    u32   hash_frame;           /* frame the hash was last verified on */
    u32   tlut;                 /* palette selector -- same address + different
                                 * palette is a DIFFERENT texture (character
                                 * icons visibly duplicated without this) */
} TexEntry;

/* A linear texture's pitch is a hardware field with an alignment requirement:
 * the RSX reads rows 64 bytes at a time and a pitch that is not a multiple of
 * 64 is not a slightly-slow texture, it is a texture whose rows start in the
 * wrong place -- the image shears progressively down the screen. Most Wii art
 * is an awkward width (233x167 buttons, 831x316 panels, 8x167 slivers: three
 * of the five distinct textures on the title screen), so this is the common
 * case rather than the exotic one. The decoded image is written into a
 * padded buffer; the padding is never sampled. */
static u32 tex_pitch_for(unsigned width)
{
    return ((u32)width * 4u + 63u) & ~63u;
}

static TexEntry s_texcache[TEXCACHE_SLOTS];

/* Sampled hash of the texture's guest bytes. The cache was keyed by address,
 * format and size alone, so a texture the CPU rewrites in place -- the THP
 * video player uploads new frames to the SAME address every frame -- kept
 * serving its first decode forever: the attract movie rendered as a flat
 * magenta wash (the YUV->RGB TEV fed stale luma). 64 strided samples catch a
 * rewritten frame with near certainty at negligible bind cost. */
static u32 tex_content_hash(const BPTexture *t)
{
    u32 bytes = tex_image_bytes((GXTextureFormat)t->format, t->width, t->height);
    u32 step = bytes > 256u ? bytes / 64u : 4u;
    u32 h = 2166136261u, off;
    if (!bytes) return 0;
    step &= ~3u;
    if (!step) step = 4u;
    for (off = 0; off + 4 <= bytes; off += step) {
        h ^= mem_read32(t->address + off);
        h *= 16777619u;
    }
    return h;
}

static TexEntry *texcache_get(const BPTexture *t, const BPState *bp)
{
    u32 h = (t->address >> 5) * 2654435761u ^ (t->format * 97u)
          ^ (t->width * 31u) ^ t->height ^ (t->tlut * 131u);
    unsigned probe;

    if (t->address == 0 || t->width == 0 || t->height == 0)
        return NULL;

    for (probe = 0; probe < 8; probe++) {
        TexEntry *e = &s_texcache[(h + probe) % TEXCACHE_SLOTS];
        if (e->valid && e->addr == t->address && e->fmt == t->format &&
            e->width == t->width && e->height == t->height &&
            e->tlut == t->tlut) {
            u32 ch;
            /* Revalidate the content AT MOST once per frame per entry. The
             * hash is 64 strided guest reads; at the race's ~15k binds/frame
             * hashing on every bind was ~1M guest reads a frame -- a fifth of
             * the whole frame time -- to catch rewrites (THP frames) that can
             * only happen between frames anyway. */
            extern u64 g_draw_frame_base;
            if (e->hash_frame == s_tex_frame)
                return e;
            ch = tex_content_hash(t);
            e->hash_frame = s_tex_frame;
            if (ch == e->chash)
                return e;
            /* Content changed under the same key: drop and re-decode. The
             * pixels the RSX may still be sampling stay allocated (same
             * quarantine reasoning as the shader cache). */
            e->valid = 0;
            g_gx_render.texture_evictions++;
            continue;
        }
        if (!e->valid) {
            u32 pitch = tex_pitch_for(t->width);
            u32 *pixels = (u32 *)rsxMemalign(128, pitch * t->height);
            u32 *tight;
            if (!pixels)
                return NULL;
            /* tex_decode writes a tightly packed image, which is the right
             * shape for a decoder and the wrong one for the sampler. Decode
             * into scratch and restripe into the padded buffer; this happens
             * once per distinct texture, so a second pass over the pixels is
             * not a cost anything can measure. */
            tight = (u32 *)malloc((size_t)t->width * t->height * 4u);
            if (!tight) { rsxFree(pixels); return NULL; }
            int drc;
            if (t->format == GX_TF_C4 || t->format == GX_TF_C8 ||
                t->format == GX_TF_C14X2) {
                /* TLUT register: bits 0..9 are the palette's TMEM offset in
                 * 32-byte units, bits 10..11 its format (0 IA8, 1 RGB565,
                 * 2 RGB5A3). C4 has 16 entries, C8 256, C14X2 16384. */
                unsigned poff = (t->tlut & 0x3FFu) << 4;   /* in 16-bit units */
                unsigned pfmt = (t->tlut >> 10) & 3u;
                unsigned pn   = (t->format == GX_TF_C4)  ? 16u
                              : (t->format == GX_TF_C8)  ? 256u : 16384u;
                unsigned avail = (unsigned)(sizeof bp->tlut_mem /
                                            sizeof bp->tlut_mem[0]);
                if (poff >= avail) poff = 0;
                if (pn > avail - poff) pn = avail - poff;
                drc = tex_decode_paletted((GXTextureFormat)t->format,
                                          t->address, t->width, t->height,
                                          tight, bp->tlut_mem + poff, pn, pfmt);
            } else {
                drc = tex_decode((GXTextureFormat)t->format, t->address,
                                 t->width, t->height, tight);
            }
            if (drc != 0) {
                /* The leak was NOT bounded: the slot is never marked valid,
                 * so the next draw binding this same texture arrives here and
                 * allocates again -- video memory bled at DRAW RATE for every
                 * paletted or unmodelled format, and PSL1GHT's first-fit
                 * allocator then walks a free list that grows without limit
                 * (measured: 9 -> 2,755 nodes, 1 -> 2,755 steps per alloc).
                 * That is the progressive slowdown. */
                free(tight);
                rsxFree(pixels);
                return NULL;
            }
            {
                unsigned row;
                for (row = 0; row < t->height; row++)
                    memcpy((u8 *)pixels + (size_t)row * pitch,
                           tight + (size_t)row * t->width,
                           (size_t)t->width * 4u);
            }
            e->mips = 1;
            /* Power-of-two textures get a swizzled mip chain, generated by box
             * filter from level 0 (RSX LINEAR textures cannot carry mips, so
             * the chain is swizzled). A generated chain is visually equivalent
             * to the game's own mip data and needs no per-format walking. */
            if (t->width >= 2 && t->height >= 2 &&
                (t->width & (t->width - 1)) == 0 &&
                (t->height & (t->height - 1)) == 0 &&
                t->width <= 1024 && t->height <= 1024) {
                u32 w = t->width, hgt = t->height, total = 0, nlev = 0;
                for (;;) {
                    total += w * hgt; nlev++;
                    if (w == 1 && hgt == 1) break;
                    w = w > 1 ? w >> 1 : 1; hgt = hgt > 1 ? hgt >> 1 : 1;
                }
                {
                    u32 *sw = (u32 *)rsxMemalign(128, total * 4u);
                    u32 *lvl = (u32 *)malloc((size_t)t->width * t->height * 4u);
                    if (sw && lvl) {
                        u32 off = 0, x, y, li;
                        memcpy(lvl, tight, (size_t)t->width * t->height * 4u);
                        w = t->width; hgt = t->height;
                        for (li = 0; li < nlev; li++) {
                            for (y = 0; y < hgt; y++)
                                for (x = 0; x < w; x++)
                                    sw[off + tex_swizzle_index(x, y, w, hgt)] =
                                        lvl[y * w + x];
                            off += w * hgt;
                            if (w == 1 && hgt == 1) break;
                            if (w > 1 && hgt > 1) {
                                tex_halve(lvl, w, hgt, lvl);
                            } else {
                                /* one axis already 1: average pairs along the
                                 * remaining axis */
                                u32 n2 = (w > 1 ? w >> 1 : 1) *
                                         (hgt > 1 ? hgt >> 1 : 1), i2;
                                for (i2 = 0; i2 < n2; i2++) {
                                    u32 a2 = lvl[i2*2], b2 = lvl[i2*2+1];
                                    u32 r=((((a2>>24)&0xFFu)+((b2>>24)&0xFFu))>>1);
                                    u32 g=((((a2>>16)&0xFFu)+((b2>>16)&0xFFu))>>1);
                                    u32 b3=((((a2>>8)&0xFFu)+((b2>>8)&0xFFu))>>1);
                                    u32 a3=(((a2&0xFFu)+(b2&0xFFu))>>1);
                                    lvl[i2]=(r<<24)|(g<<16)|(b3<<8)|a3;
                                }
                            }
                            w = w > 1 ? w >> 1 : 1; hgt = hgt > 1 ? hgt >> 1 : 1;
                        }
                        if (rsxAddressToOffset(sw, &e->offset) == 0) {
                            rsxFree(pixels);
                            pixels = sw; sw = NULL;
                            e->mips = (u8)nlev;
                            pitch = 0;
                        }
                    }
                    if (sw) rsxFree(sw);
                    free(lvl);
                }
            }
            free(tight);
            e->pitch = pitch;
            if (e->mips <= 1 &&
                rsxAddressToOffset(pixels, &e->offset) != 0)
                return NULL;
            /* The RSX texture cache does not watch memory: without an
             * invalidate, the samplers keep serving whatever they cached
             * before this upload existed -- which rendered as a full screen
             * of flat white while the thumbnails (CPU reads of the same
             * pixels) showed perfect art. The known-good PSL1GHT sample
             * invalidates before every bind; once per new upload is the
             * cheap version of the same correctness. */
            rsxInvalidateTextureCache(rsx_context(), GCM_INVALIDATE_TEXTURE);
            e->pixels = pixels;
            e->chash  = tex_content_hash(t);
            e->hash_frame = s_tex_frame;
            e->tlut   = t->tlut;
            e->addr   = t->address;
            e->fmt    = (u8)t->format;
            e->width  = (u16)t->width;
            e->height = (u16)t->height;
            e->valid  = 1;
            g_gx_render.textures_decoded++;
            return e;
        }
    }
    return NULL;                    /* all probes busy: draw untextured */
}

/* Debug: blit up to `count` decoded textures as thumbnails into a CPU
 * framebuffer -- one glance at the screen says whether decode works. */
void gx_render_debug_textures(u32 *fb, int pitch_words, int y0, int thumb,
                              int count)
{
    int drawn = 0, i;
    for (i = 0; i < TEXCACHE_SLOTS && drawn < count; i++) {
        TexEntry *e = &s_texcache[i];
        int x, y;
        if (!e->valid || !e->pixels) continue;
        for (y = 0; y < thumb; y++) {
            for (x = 0; x < thumb; x++) {
                u32 sx = (u32)x * e->width  / (u32)thumb;
                u32 sy = (u32)y * e->height / (u32)thumb;
                u32 px = *(const u32 *)((const u8 *)e->pixels
                                        + (size_t)sy * e->pitch + sx * 4u);
                /* decoded layout R,G,B,A (big-endian u32) -> frame ARGB */
                fb[(y0 + y) * pitch_words + drawn * (thumb + 4) + 8 + x] =
                    0xFF000000u | (px >> 8);
            }
        }
        drawn++;
    }
}

/* The EFB rectangle the title presents, learned from its own XFB copies. It is
 * the reference frame for every EFB-to-screen conversion: the guest viewport,
 * the guest scissor and the source rectangle of a copy are all in EFB pixels,
 * and the render target is the television. Mario Kart Wii presents 608x456;
 * 640x480 is the safe default until the first frame ends. */
static u32 s_efb_w = 640, s_efb_h = 480;

static float efb_sx(void) { return (float)g_rsx.width  / (float)s_efb_w; }
static float efb_sy(void) { return (float)g_rsx.height / (float)s_efb_h; }

/* ------------------------------------------------------------------ */
/* Viewport and scissor                                                 */
/*                                                                      */
/* Until now every draw was rasterised into the whole display buffer, which is  */
/* right for exactly one thing a title does -- render a full-screen frame --    */
/* and wrong for the other one: render a small image into a corner of the EFB   */
/* and copy it out as a texture. A 128x128 Mii head drawn with the viewport     */
/* ignored covers the television, and the copy that follows then takes the      */
/* top-left 128x128 EFB pixels of it, which is a magnified fragment of an ear.  */
/*                                                                              */
/* So the guest viewport and scissor are honoured, in EFB pixels scaled to the  */
/* render target. The measurement that says this is safe: over a complete       */
/* Mario Kart Wii boot (tools/efb_audit.sh), 174,186 of 174,282 draws ask for   */
/* a viewport that starts at the EFB origin and covers the whole presented      */
/* frame, with a scissor that does not cut into it -- and the frame the title   */
/* presents is 608x456, exactly the viewport those draws ask for, so the        */
/* mapping is arithmetically the identity and not one pixel moves. The 96 draws */
/* that are left are precisely the render-to-texture batches: viewport 608x448  */
/* with the scissor closed down to (0,0)-(127,127).                             */
/*                                                                              */
/* GX stores the viewport as scale/offset with the rasteriser's +342 bias in    */
/* the offset, and its y scale is negative because GX counts y downward from    */
/* the top of the EFB:                                                          */
/*     w =  2*scale.x            x = offset.x - 342 - scale.x                   */
/*     h = -2*scale.y            y = offset.y - 342 + scale.y                   */
/* ------------------------------------------------------------------ */

static struct {
    int valid;
    int vx, vy, vw, vh;
    int sx0, sy0, sw, sh;
} s_view;

/* Called whenever something else has written the viewport or the scissor, so
 * the next draw re-establishes its own. */
static void gx_view_state_dirty(void) { s_view.valid = 0; }

static int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

GxVpCensus g_vp_census[GX_VP_CENSUS_MAX];
unsigned   g_vp_census_n;
static int s_vp_cur = -1;   /* census slot the live rectangle belongs to */

/* Bumped once per frame. The constant caches below key on it as well as on
 * their own state, so nothing they hold can outlive a frame -- the flip
 * reinitialises the RSX context, and a cache that assumed otherwise would
 * serve constants the GPU no longer has. */
static u32 s_const_epoch;
static unsigned s_cmat_logged;

static void apply_view_state(gcmContextData *c, const GXState *g)
{
    const XFState *xf = &g->xf;
    const BPState *bp = &g->bp;
    float fw = xf->viewport_scale[0] * 2.0f;
    float fh = xf->viewport_scale[1] * -2.0f;
    float fx = xf->viewport_offset[0] - 342.0f - xf->viewport_scale[0];
    float fy = xf->viewport_offset[1] - 342.0f + xf->viewport_scale[1];
    float sx = efb_sx(), sy = efb_sy();
    int vx, vy, vw, vh, sx0, sy0, sw, sh;

    /* Mask 0 is the "write no state at all" baseline, and that has to keep
     * meaning exactly what it meant: the frame-wide viewport and scissor
     * rsx_frame_begin set, untouched. */
    if (g_gx_state_mask == 0)
        return;

    /* A viewport that has never been written, or a degenerate one, means the
     * title is not asking for anything: leave the frame-wide setup in place
     * rather than rasterising into a zero-sized rectangle. */
    if (!(fw > 0.5f && fh > 0.5f))
        return;

    vx = clampi((int)(fx * sx + 0.5f), 0, (int)g_rsx.width);
    vy = clampi((int)(fy * sy + 0.5f), 0, (int)g_rsx.height);
    vw = clampi((int)(fw * sx + 0.5f), 1, (int)g_rsx.width  - vx);
    vh = clampi((int)(fh * sy + 0.5f), 1, (int)g_rsx.height - vy);

    /* The scissor offset register shifts the rasteriser origin; both it and
     * the scissor corners have already had the +342 bias removed by bp.c, so
     * the effective rectangle is one subtraction. Right/bottom are inclusive. */
    {
        int l = bp->scissor_left   - bp->scissor_offset_x;
        int t = bp->scissor_top    - bp->scissor_offset_y;
        int r = bp->scissor_right  - bp->scissor_offset_x;
        int b = bp->scissor_bottom - bp->scissor_offset_y;
        if (r < l || b < t) { l = t = 0; r = b = -1; }
        sx0 = clampi((int)((float)l * sx + 0.5f), 0, (int)g_rsx.width);
        sy0 = clampi((int)((float)t * sy + 0.5f), 0, (int)g_rsx.height);
        sw  = clampi((int)((float)(r - l + 1) * sx + 0.5f), 0,
                     (int)g_rsx.width  - sx0);
        sh  = clampi((int)((float)(b - t + 1) * sy + 0.5f), 0,
                     (int)g_rsx.height - sy0);
    }

    if (!(g_gx_state_mask & GX_STATE_VIEWPORT)) {
        vx = vy = 0; vw = (int)g_rsx.width; vh = (int)g_rsx.height;
    }
    if (!(g_gx_state_mask & GX_STATE_SCISSOR)) {
        sx0 = sy0 = 0; sw = (int)g_rsx.width; sh = (int)g_rsx.height;
    }

    if (s_view.valid && s_view.vx == vx && s_view.vy == vy &&
        s_view.vw == vw && s_view.vh == vh && s_view.sx0 == sx0 &&
        s_view.sy0 == sy0 && s_view.sw == sw && s_view.sh == sh)
        return;                     /* nothing changed: no command words */

    {
        f32 scale[4], offset[4];
        scale[0]  =  (f32)vw * 0.5f;
        /* Still negated: the RSX's window origin is top-left and clip space is
         * bottom-up, exactly as in the frame-wide setup this replaces. */
        scale[1]  = -(f32)vh * 0.5f;
        scale[2]  =  0.5f;
        scale[3]  =  0.0f;
        offset[0] =  (f32)vx + (f32)vw * 0.5f;
        offset[1] =  (f32)vy + (f32)vh * 0.5f;
        offset[2] =  0.5f;
        offset[3] =  0.0f;
        rsxSetViewport(c, (u16)vx, (u16)vy, (u16)vw, (u16)vh, 0.0f, 1.0f,
                       scale, offset);
        rsxSetScissor(c, (u16)sx0, (u16)sy0, (u16)sw, (u16)sh);
    }

    s_view.valid = 1;
    s_view.vx = vx; s_view.vy = vy; s_view.vw = vw; s_view.vh = vh;
    s_view.sx0 = sx0; s_view.sy0 = sy0; s_view.sw = sw; s_view.sh = sh;

    /* Census of the rectangles actually programmed. In-race the world renders
     * into a small corner of the EFB while the HUD draws at full size, and a
     * sampled log cannot tell whether that is the viewport, the scissor, or
     * the copy. This counts distinct rectangles with the draw count each one
     * received, which answers it directly. */
    {
        unsigned i;
        for (i = 0; i < g_vp_census_n; i++) {
            if (g_vp_census[i].vw == (u16)vw && g_vp_census[i].vh == (u16)vh &&
                g_vp_census[i].vx == (u16)vx && g_vp_census[i].vy == (u16)vy &&
                g_vp_census[i].sw == (u16)sw && g_vp_census[i].sh == (u16)sh) {
                g_vp_census[i].uses++;
                s_vp_cur = (int)i;
                return;
            }
        }
        if (g_vp_census_n < GX_VP_CENSUS_MAX) {
            i = g_vp_census_n++;
            g_vp_census[i].vx = (u16)vx; g_vp_census[i].vy = (u16)vy;
            g_vp_census[i].vw = (u16)vw; g_vp_census[i].vh = (u16)vh;
            g_vp_census[i].sx = (u16)sx0; g_vp_census[i].sy = (u16)sy0;
            g_vp_census[i].sw = (u16)sw; g_vp_census[i].sh = (u16)sh;
            g_vp_census[i].uses = 1;
            g_vp_census[i].raw_sc[0]  = xf->viewport_scale[0];
            g_vp_census[i].raw_sc[1]  = xf->viewport_scale[1];
            g_vp_census[i].raw_sc[2]  = xf->viewport_scale[2];
            g_vp_census[i].raw_off[0] = xf->viewport_offset[0];
            g_vp_census[i].raw_off[1] = xf->viewport_offset[1];
            g_vp_census[i].raw_off[2] = xf->viewport_offset[2];
            s_vp_cur = (int)i;
        }
    }
}

/* ------------------------------------------------------------------ */
/* EFB copies that become textures (render to texture)                  */
/*                                                                      */
/* A GX title renders a small thing -- a Mii head, a mirror, a shadow map --    */
/* into a corner of the embedded framebuffer and then copies that corner out    */
/* as a texture it samples a few draws later. Until now this backend acted on   */
/* the copies that end a frame and dropped the rest on the floor, so those      */
/* textures were sampled from guest memory that nothing had ever written: the   */
/* scan lines and garbage on Mario Kart Wii's licence-selection screen.         */
/*                                                                              */
/* WHY A BLIT AT COPY TIME rather than rendering into a texture surface.        */
/*                                                                              */
/* The other option is to point the RSX's colour surface at a texture           */
/* allocation for the draws that feed the copy. It cannot be done here, and     */
/* the reason is in the command stream rather than in the GPU: the copy         */
/* trigger arrives AFTER the draws it copies. Nothing in the FIFO announces     */
/* that a render-to-texture batch is starting, so the surface would have to be  */
/* retargeted on a guess -- and a guess that is wrong in the "yes" direction    */
/* sends a whole frame into a 128x128 buffer. Retargeting also means            */
/* re-establishing the surface, and with it the depth buffer and the tile and   */
/* zcull configuration, several times a frame, which is one of the more         */
/* expensive things this GPU can be asked to do.                                */
/*                                                                              */
/* The RSX's 2D engine, on the other hand, does exactly the operation the       */
/* pixel engine is performing: read a rectangle of one surface, scale it, write */
/* it to another. It is twenty-six command words (NV3089 scaled-image-from-     */
/* memory into an NV3062 2D surface), GDDR3 to GDDR3, with no shader, no        */
/* surface change and no disturbance to the 3D state -- and the scaling is not  */
/* an extra: because this backend renders at the television's resolution rather */
/* than the EFB's, every copy to a texture is inherently a downscale, and the   */
/* filter the 2D engine applies on the way is a fair analogue of GX's own copy  */
/* filter, which averages EFB rows into each destination row.                   */
/*                                                                              */
/* WHAT ONLY THE CONSOLE CAN CONFIRM: that the 2D engine reads the pixels the   */
/* 3D pipe has just written (the wait-for-idle below is there for exactly that  */
/* ordering), and that the resulting texture is the right way up and the right  */
/* colour. Everything above that -- which copies the game issues, what geometry */
/* they have, which draws resolve to them -- is checked off hardware by         */
/* tools/efb_audit.sh and tests/test_efbcopy.c.                                 */
/* ------------------------------------------------------------------ */

/* One resolved surface per registry slot. Allocated on first use, grown if a
 * later copy to that slot is bigger, never freed while the emulator runs --
 * see the note on efb_copy_index() for why freeing is the dangerous option. */
typedef struct {
    void *mem;
    u32   offset;
    u32   pitch;
    u32   bytes;
    u32   serial;       /* the copy generation these pixels came from */
    u16   width, height;
    u16   store_w, store_h;   /* fb-density store size (see resolve)  */
} EfbSurface;

static EfbSurface s_efb_surf[EFB_COPY_TARGETS];

/* Resolve one EFB-to-texture copy into its slot's surface. */
static void efb_copy_resolve(const BPCopy *copy)
{
    gcmContextData *c = rsx_context();
    EfbCopyTarget *e;
    EfbSurface *sf;
    gcmTransferScale sc;
    gcmTransferSurface ds;
    float sx, sy;
    float src_x, src_y, src_w, src_h;
    u32 want_pitch, want_bytes;

    if (!c || !g_rsx.inited)
        return;

    e = efb_copy_note(copy);
    if (!e)
        return;                     /* geometry we cannot model; counted */

    sf = &s_efb_surf[efb_copy_index(e)];

    /* Store the copy at the RENDER TARGET's pixel density, not the EFB's.
     * Resolving fb-scale pixels DOWN to GX-native size and then drawing them
     * back UP resamples every render-to-texture twice per hop -- the title
     * screen art goes through such a copy, and the double LINEAR resample is
     * the blur. At fb density the blit is 1:1 and the later draw is 1:1. */
    {
        float sxd = efb_sx(), syd = efb_sy();
        u32 dw = (u32)((float)e->width  * sxd + 0.5f);
        u32 dh = (u32)((float)e->height * syd + 0.5f);
        if (dw < 1) dw = 1;
        if (dh < 1) dh = 1;
        if (dw > 2048) dw = 2048;
        if (dh > 2048) dh = 2048;
        sf->store_w = dw; sf->store_h = dh;
    }
    want_pitch = (sf->store_w * 4u + 63u) & ~63u;
    want_bytes = want_pitch * sf->store_h;
    if (!sf->mem || sf->bytes < want_bytes) {
        /* Grow rather than reallocate per copy: the same slot is written every
         * time the title re-renders that Mii, and an allocator round trip per
         * copy is how the texture cache used to bleed video memory. */
        void *m = rsxMemalign(128, want_bytes);
        u32 off;
        if (!m)
            { e->valid = 0; return; }
        if (rsxAddressToOffset(m, &off) != 0)
            { rsxFree(m); e->valid = 0; return; }
        /* The old allocation is deliberately NOT freed: the RSX may still be
         * executing a draw that samples it. Sixteen slots, each grown at most
         * to its largest copy, is a bounded leak measured in kilobytes; a
         * use-after-free here is a corrupted frame with no way to see why. */
        sf->mem = m; sf->offset = off; sf->bytes = want_bytes;
    }
    sf->pitch  = want_pitch;
    sf->width  = sf->store_w;
    sf->height = sf->store_h;

    /* Source rectangle, EFB pixels -> render-target pixels. */
    sx = efb_sx(); sy = efb_sy();
    src_x = (float)copy->src_x * sx;
    src_y = (float)copy->src_y * sy;
    src_w = (float)copy->width  * sx;
    src_h = (float)copy->height * sy;
    if (src_x < 0.0f) src_x = 0.0f;
    if (src_y < 0.0f) src_y = 0.0f;
    if (src_x + src_w > (float)g_rsx.width)  src_w = (float)g_rsx.width  - src_x;
    if (src_y + src_h > (float)g_rsx.height) src_h = (float)g_rsx.height - src_y;
    if (src_w <= 0.0f || src_h <= 0.0f)
        { e->valid = 0; return; }

    /* The 3D pipe is deeply pipelined and the 2D engine is a different object
     * on the same channel: without this, the blit is free to read the render
     * target before the draws that filled it have retired. It costs a drain,
     * which is why it is here and not in the draw path -- a title issues a
     * handful of these a frame, not hundreds. */
#ifdef __PS3__
    {   extern void spu_vtx_join(void);
        spu_vtx_join();
    }
#endif
    rsxSetWaitForIdle(c);

    memset(&sc, 0, sizeof sc);
    memset(&ds, 0, sizeof ds);

    ds.format = GCM_TRANSFER_SURFACE_FORMAT_A8R8G8B8;
    ds.pitch  = (u16)sf->pitch;
    ds.offset = sf->offset;

    sc.conversion = GCM_TRANSFER_CONVERSION_TRUNCATE;
    sc.format     = GCM_TRANSFER_SCALE_FORMAT_A8R8G8B8;
    sc.operation  = GCM_TRANSFER_OPERATION_SRCCOPY;
    sc.clipX = 0; sc.clipY = 0;
    sc.clipW = (u16)sf->store_w; sc.clipH = (u16)sf->store_h;
    sc.outX  = 0; sc.outY  = 0;
    sc.outW  = (u16)sf->store_w; sc.outH  = (u16)sf->store_h;
    sc.ratioX = rsxGetFixedSint32(src_w / (float)sf->store_w);
    sc.ratioY = rsxGetFixedSint32(src_h / (float)sf->store_h);
    sc.inW    = (u16)g_rsx.width;
    sc.inH    = (u16)g_rsx.height;
    sc.pitch  = (u16)g_rsx.pitch;
    sc.origin = GCM_TRANSFER_ORIGIN_CORNER;
    /* Linear, because this is always a downscale: point-sampling a 2x
     * reduction throws away three quarters of the pixels the title drew, and
     * the aliasing is exactly what a Mii head shows worst. */
    /* NEAREST when the blit is 1:1 (it is, now that surfaces store at fb
     * density). The title re-blits its own frame through this path EVERY
     * frame -- scene -> copy -> redraw -> copy -- and with LINEAR any
     * sub-texel phase error compounds per generation into a screen-wide blur
     * that Dolphin (pixel-exact copies) never shows. */
    sc.interp = (sf->store_w == (u32)(src_w + 0.5f) &&
                 sf->store_h == (u32)(src_h + 0.5f))
              ? GCM_TRANSFER_INTERPOLATOR_NEAREST
              : GCM_TRANSFER_INTERPOLATOR_LINEAR;
    /* Source is the surface being rendered into, which is the dedicated EFB
     * when one is bound and the display buffer otherwise. */
    sc.offset = rsx_render_target_offset();
    sc.inX    = rsxGetFixedUint16(src_x);
    sc.inY    = rsxGetFixedUint16(src_y);

    rsxSetTransferScaleMode(c, GCM_TRANSFER_LOCAL_TO_LOCAL, GCM_TRANSFER_SURFACE);
    rsxSetTransferScaleSurface(c, &sc, &ds);

    /* The sampler caches by address, not by content: without this the next
     * bind serves whatever it read before the blit existed. Same reason the
     * decode path invalidates after an upload. */
    rsxInvalidateTextureCache(c, GCM_INVALIDATE_TEXTURE);

    sf->serial = e->serial;
    g_gx_render.efb_copies_resolved++;

    /* Once per new destination. A console has no debugger, and the one thing
     * worth knowing from the sofa is whether the copies the qemu audit
     * predicted are the copies the console sees -- address, size and format,
     * three numbers that identify the effect exactly. */
    if (e->copies == 1)
        LOG_INFO(LOG_VIDEO,
                 "efb copy -> texture: %08x %ux%u fmt %u stride %u "
                 "from EFB (%u,%u) %ux%u  clear=%u",
                 (unsigned)e->addr, (unsigned)e->width, (unsigned)e->height,
                 (unsigned)e->fmt, (unsigned)e->stride,
                 (unsigned)copy->src_x, (unsigned)copy->src_y,
                 (unsigned)copy->width, (unsigned)copy->height,
                 (unsigned)copy->clear);
}

/* GX clears the EFB rectangle as part of a copy when the clear bit is set, and
 * it matters for more than tidiness. Mario Kart Wii's Mii render is bracketed
 * by two clearing copies; the first one is what puts the transparent
 * background under the head, so without it the copied texture comes out with
 * the frame's own opaque clear colour behind the face -- and the render-to-
 * texture pixels also stay in the frame the title is still building, as a
 * bright rectangle in the corner of the screen. */
static void efb_copy_clear(const BPCopy *copy)
{
    gcmContextData *c = rsx_context();
    float sx, sy;
    int x, y, w, h;

    if (!c || !g_rsx.inited)
        return;

    sx = efb_sx(); sy = efb_sy();
    x = (int)((float)copy->src_x * sx);
    y = (int)((float)copy->src_y * sy);
    w = (int)((float)copy->width  * sx + 0.5f);
    h = (int)((float)copy->height * sy + 0.5f);
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x + w > (int)g_rsx.width)  w = (int)g_rsx.width  - x;
    if (y + h > (int)g_rsx.height) h = (int)g_rsx.height - y;
    if (w <= 0 || h <= 0)
        return;

    rsxSetScissor(c, (u16)x, (u16)y, (u16)w, (u16)h);
    /* DEPTH AND STENCIL ONLY -- the colour clear is deliberately not issued.
     *
     * On the real machine a copy's clear empties the EMBEDDED framebuffer, a
     * surface the video interface never scans; the XFB it just copied to keeps
     * the picture. This backend has no separate EFB -- it renders straight
     * into the buffer being displayed -- so clearing the copy's rect here
     * erases pixels that have already been presented.
     *
     * In a race that is fatal rather than cosmetic. MKWii's post-processing
     * takes 18,702 32x32 copies, 1,561 at 256x256 and 648 at 128x128 in a
     * single measured interval, each one clearing its rect; together they wipe
     * the frame, and the only draws left standing are the ones issued after
     * the last copy -- the HUD. That is exactly the reported symptom: HUD and
     * minimap over a black world. Suppressing the colour clear (verified live
     * on hardware by clearing GX_STATE_EFB_CLEAR) brings the track, the
     * grandstand, the karts and the smoke back.
     *
     * Depth still has to be cleared: the title renders into that rect again
     * immediately, and leaving stale Z there would reject the new geometry.
     * The frame-wide colour clear that rsx_frame_begin/rsx_clear issue after
     * every flip is what actually prepares the next frame's colour.
     *
     * The complete fix is a dedicated EFB surface with a blit to the display
     * buffer on the present copy, which would also stop discarding the EFB
     * rows below 456 that the census shows the title rendering into. This is
     * the correct behaviour for the surface we actually have today. */
    rsxSetClearDepthStencil(c, copy->clear_z << 8);
    rsxSetDepthWriteEnable(c, GCM_TRUE);        /* or the clear is a no-op */
    if (g_rsx.efb) {
        /* With a real EFB bound this clear can do its actual job: it empties
         * the embedded framebuffer, which is not what the display scans. */
        rsxSetClearColor(c, copy->clear_color);
        rsxSetColorMask(c, GCM_COLOR_MASK_R | GCM_COLOR_MASK_G |
                           GCM_COLOR_MASK_B | GCM_COLOR_MASK_A);
        rsxClearSurface(c, GCM_CLEAR_R | GCM_CLEAR_G | GCM_CLEAR_B |
                           GCM_CLEAR_A | GCM_CLEAR_Z | GCM_CLEAR_S);
    } else {
        rsxClearSurface(c, GCM_CLEAR_Z | GCM_CLEAR_S);
    }
    pipeline_state_cache_invalidate();  /* masks/depth-write were poked raw */
    /* The scissor is now wrong for the next draw; the per-draw cache is told
     * so rather than restoring it here, so the cost is one re-emit on the next
     * draw instead of a second full state block. */
    gx_view_state_dirty();
}

int g_tex_force_clamp;
int g_tex_force_trilinear;
u32 g_draw_win_min, g_draw_win_max;
u32 g_draw_info_idx; int g_draw_info_arm; u32 g_draw_info_minv;
u32 g_draw_minverts;
u32 g_census_agg[4];
u64 g_addr_flips;
u64 g_draw_frame_base;

/* GX wrap: 0 clamp, 1 repeat, 2 mirror. */
static u8 gx_wrap_to_gcm(unsigned w)
{
    switch (w & 3u) {
    case 1:  return GCM_TEXTURE_REPEAT;
    case 2:  return GCM_TEXTURE_MIRRORED_REPEAT;
    default: return GCM_TEXTURE_CLAMP_TO_EDGE;
    }
}

/* Morton order for RSX swizzled textures: interleave x and y bits, the larger
 * dimension contributing its remaining bits linearly once the smaller is
 * exhausted. Swizzled layout is required because RSX LINEAR textures cannot
 * carry mipmaps, and mipmaps are the fix for the title-screen banding: the
 * hardware samples a minified mip whose average is what Dolphin shows flat,
 * while sampling mip0 shows the full-amplitude gradient as bars. */
static u32 tex_swizzle_index(u32 x, u32 y, u32 w, u32 h)
{
    u32 idx = 0, shift = 0;
    while (w > 1 || h > 1) {
        if (w > 1) { idx |= (x & 1u) << shift; x >>= 1; w >>= 1; shift++; }
        if (h > 1) { idx |= (y & 1u) << shift; y >>= 1; h >>= 1; shift++; }
    }
    return idx;
}

/* Box-filtered half-scale in place: src WxH RGBA8 -> dst (W/2)x(H/2). */
static void tex_halve(const u32 *src, u32 w, u32 h, u32 *dst)
{
    u32 x, y, nw = w >> 1, nh = h >> 1;
    for (y = 0; y < nh; y++)
        for (x = 0; x < nw; x++) {
            const u32 p[4] = { src[(2*y)*w + 2*x],   src[(2*y)*w + 2*x+1],
                               src[(2*y+1)*w + 2*x], src[(2*y+1)*w + 2*x+1] };
            u32 r=0,g=0,b=0,a=0; unsigned i;
            for (i = 0; i < 4; i++) {
                r += (p[i] >> 24) & 0xFF; g += (p[i] >> 16) & 0xFF;
                b += (p[i] >> 8) & 0xFF;  a += p[i] & 0xFF;
            }
            dst[y*nw + x] = ((r>>2)<<24)|((g>>2)<<16)|((b>>2)<<8)|(a>>2);
        }
}

/* Sampler state that is the same whichever memory the texels came from. */
/* Emit a texture binding only when it differs from what is already on the unit.
 *
 * All the validation above still runs -- the EFB-copy serial check, the
 * texcache probe -- so a surface that changed under the same address is still
 * caught. What is skipped is the COMMAND EMISSION: rsxLoadTexture writes a
 * descriptor into the ring and the sampler state follows it, and a title
 * sorted by material rebinds the same texture to the same unit across long
 * runs of draws.
 *
 * Bounded by the frame epoch for the same reason the constant caches are: the
 * flip reinitialises the RSX context, so nothing may be assumed to survive it.
 */
static gcmTexture s_unit_tex[8];
static u8         s_unit_valid[8];
static u32        s_unit_epoch[8];
static u8         s_unit_mips[8];

static int tex_bind_unchanged(unsigned unit, const gcmTexture *gt, u8 mips)
{
    if (unit >= 8) return 0;
    if (!s_unit_valid[unit] || s_unit_epoch[unit] != s_const_epoch) return 0;
    if (s_unit_mips[unit] != mips) return 0;
    return memcmp(&s_unit_tex[unit], gt, sizeof *gt) == 0;
}

static void tex_bind_remember(unsigned unit, const gcmTexture *gt, u8 mips)
{
    if (unit >= 8) return;
    s_unit_tex[unit]   = *gt;
    s_unit_valid[unit] = 1;
    s_unit_epoch[unit] = s_const_epoch;
    s_unit_mips[unit]  = mips;
}

static void tex_apply_sampler(gcmContextData *c, unsigned unit,
                              const BPTexture *t, unsigned mips)
{
    rsxTextureControl(c, (u8)unit, GCM_TRUE, 0 << 8,
                      (mips ? mips : 12u) << 8, 0);
    /* GX's filter fields, rather than linear unconditionally. A title that
     * asks for point sampling is usually asking because the art depends on it
     * -- a pixel-aligned UI element or a palette strip read as a lookup -- and
     * filtering it produces a soft edge exactly where the artist wanted a hard
     * one. Mip filtering is not selected here because only level 0 is
     * uploaded. */
    {   /* GX min_filter (mode0 bits 5..7): 0 near, 1 near_mip_near,
         * 2 near_mip_lin, 4 lin, 5 lin_mip_near, 6 lin_mip_lin. Mip variants
         * only when a chain was actually uploaded. */
        static const u8 k_min_mip[8] = {
            GCM_TEXTURE_NEAREST, GCM_TEXTURE_NEAREST_MIPMAP_NEAREST,
            GCM_TEXTURE_NEAREST_MIPMAP_LINEAR, GCM_TEXTURE_NEAREST,
            GCM_TEXTURE_LINEAR, GCM_TEXTURE_LINEAR_MIPMAP_NEAREST,
            GCM_TEXTURE_LINEAR_MIPMAP_LINEAR, GCM_TEXTURE_LINEAR };
        unsigned mf = (t->mode0 >> 5) & 7u;
        u8 minf = (mips > 1) ? k_min_mip[mf]
                 : ((mf >= 4u) ? GCM_TEXTURE_LINEAR : GCM_TEXTURE_NEAREST);
        {   /* Diagnostic (file-armed): trilinear on every mipped texture, to
             * separate "filter selection wrong" from "LOD/chain wrong". */
            extern int g_tex_force_trilinear;
u32 g_draw_win_min, g_draw_win_max;
u32 g_draw_info_idx; int g_draw_info_arm;
u64 g_draw_frame_base;
            if (g_tex_force_trilinear && mips > 1)
                minf = GCM_TEXTURE_LINEAR_MIPMAP_LINEAR;
        }
        rsxTextureFilter(c, (u8)unit, 0, minf,
                         ((t->mode0 >> 4) & 1u) ? GCM_TEXTURE_LINEAR
                                                : GCM_TEXTURE_NEAREST,
                         GCM_TEXTURE_CONVOLUTION_QUINCUNX);
    }
    {   /* Diagnostic override: force clamping on both axes.
         *
         * The background bands have a period that is CONSTANT in EFB space
         * (~16 rows, measured at 480p AND 720p output), and the cache holds a
         * 16x16 texture. A 16-tall image repeating with a smooth ramp and a
         * hard drop at each boundary is a tile edge. If clamping removes the
         * bands, the fault is the wrap mode or the coordinates feeding it; if
         * it does not, the repeat is what the title asked for and the bands
         * come from somewhere else. Either answer is worth having, and this
         * costs one file to find out. */
        extern int g_tex_force_clamp;
int g_tex_force_trilinear;
u32 g_draw_win_min, g_draw_win_max;
u32 g_draw_info_idx; int g_draw_info_arm;
u64 g_draw_frame_base;
        u8 ws = g_tex_force_clamp ? GCM_TEXTURE_CLAMP_TO_EDGE
                                  : gx_wrap_to_gcm(t->mode0 & 3u);
        u8 wt = g_tex_force_clamp ? GCM_TEXTURE_CLAMP_TO_EDGE
                                  : gx_wrap_to_gcm((t->mode0 >> 2) & 3u);
        rsxTextureWrapMode(c, (u8)unit, ws, wt,
                           GCM_TEXTURE_CLAMP_TO_EDGE, 0, 0, 0);
    }
}

/* Bind a resolved EFB copy. The pixels came out of the render target, so they
 * are already in the sampler's own A8R8G8B8 order -- no channel remap, unlike
 * the decode path, whose output is R,G,B,A in memory. */
static void efb_surface_bind(gcmContextData *c, unsigned unit,
                             const BPTexture *t, const EfbCopyTarget *e,
                             const EfbSurface *sf)
{
    gcmTexture gt;
    u32 alpha_type = e->has_alpha ? GCM_TEXTURE_REMAP_TYPE_REMAP
                                  : GCM_TEXTURE_REMAP_TYPE_ONE;

    memset(&gt, 0, sizeof gt);
    gt.format    = GCM_TEXTURE_FORMAT_A8R8G8B8 | GCM_TEXTURE_FORMAT_LIN;
    gt.mipmap    = 1;
    gt.dimension = GCM_TEXTURE_DIMS_2D;
    gt.cubemap   = GCM_FALSE;
    /* Identity, except that a destination format without an alpha channel must
     * read as opaque: on the hardware those bits do not exist, and letting the
     * render target's own alpha through would make a blended draw fade an
     * image the title expects to be solid. */
    gt.remap =
        (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_B_SHIFT) |
        (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_G_SHIFT) |
        (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_R_SHIFT) |
        (alpha_type                   << GCM_TEXTURE_REMAP_TYPE_A_SHIFT) |
        (GCM_TEXTURE_REMAP_COLOR_B << GCM_TEXTURE_REMAP_COLOR_B_SHIFT) |
        (GCM_TEXTURE_REMAP_COLOR_G << GCM_TEXTURE_REMAP_COLOR_G_SHIFT) |
        (GCM_TEXTURE_REMAP_COLOR_R << GCM_TEXTURE_REMAP_COLOR_R_SHIFT) |
        (GCM_TEXTURE_REMAP_COLOR_A << GCM_TEXTURE_REMAP_COLOR_A_SHIFT);
    gt.width     = sf->width;
    gt.height    = sf->height;
    gt.depth     = 1;
    gt.location  = GCM_LOCATION_RSX;
    gt.pitch     = sf->pitch;
    gt.offset    = sf->offset;

    if (tex_bind_unchanged(unit, &gt, 1))
        return;
    tex_bind_remember(unit, &gt, 1);
    rsxLoadTexture(c, (u8)unit, &gt);
    tex_apply_sampler(c, unit, t, 1);
    /* Same reasoning as the blit: a copy the same size as the framebuffer is
     * the title's own frame coming back; sample it point-exact or the
     * roundtrip blurs a little more every generation. */
    if (sf->width == (u16)g_rsx.width && sf->height == (u16)g_rsx.height)
        rsxTextureFilter(c, (u8)unit, 0, GCM_TEXTURE_NEAREST,
                         GCM_TEXTURE_NEAREST,
                         GCM_TEXTURE_CONVOLUTION_QUINCUNX);
}

static void texcache_bind(gcmContextData *c, unsigned unit, const BPTexture *t,
                          const BPState *bp)
{
    TexEntry *e;
    gcmTexture gt;

    /* A texture whose address is an EFB copy destination must come from the
     * copy, not from a decode: the guest bytes at that address are whatever
     * was there before the title ever rendered into it, because the copy
     * happened on the GPU and never touched guest memory. Decoding them is how
     * the licence screen ends up drawing scan lines. */
    if (g_gx_state_mask & GX_STATE_EFB_COPY) {
        EfbCopyTarget *ct = efb_copy_find(t->address, t->width, t->height,
                                          t->format);
        if (ct) {
            EfbSurface *sf = &s_efb_surf[efb_copy_index(ct)];
            if (sf->mem && sf->serial == ct->serial) {
                efb_surface_bind(c, unit, t, ct, sf);
                ct->binds++;
                g_efb_copy.binds_resolved++;
                if (ct->width != t->width || ct->height != t->height)
                    g_gx_render.efb_bind_resized++;
                return;
            }
            /* The address is a copy destination but no surface backs it -- the
             * allocation failed, or the copy could not be modelled. Fall
             * through to the decode, which is wrong, and say so. */
            g_efb_copy.binds_stale++;
        }
    }

    e = texcache_get(t, bp);
    if (!e) {
        rsxTextureControl(c, (u8)unit, GCM_FALSE, 0, 0, 0);
        return;
    }

    memset(&gt, 0, sizeof gt);
    gt.format    = GCM_TEXTURE_FORMAT_A8R8G8B8 |
                   ((e->mips > 1) ? 0u : GCM_TEXTURE_FORMAT_LIN);
    gt.mipmap    = (e->mips > 1) ? e->mips : 1;
    gt.dimension = GCM_TEXTURE_DIMS_2D;
    gt.cubemap   = GCM_FALSE;
    /* Memory holds R,G,B,A; the A8R8G8B8 sampler reads byte0 as A. Remap each
     * output back to the true component. */
    gt.remap =
        (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_B_SHIFT) |
        (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_G_SHIFT) |
        (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_R_SHIFT) |
        (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_A_SHIFT) |
        (GCM_TEXTURE_REMAP_COLOR_G << GCM_TEXTURE_REMAP_COLOR_B_SHIFT) |
        (GCM_TEXTURE_REMAP_COLOR_R << GCM_TEXTURE_REMAP_COLOR_G_SHIFT) |
        (GCM_TEXTURE_REMAP_COLOR_A << GCM_TEXTURE_REMAP_COLOR_R_SHIFT) |
        (GCM_TEXTURE_REMAP_COLOR_B << GCM_TEXTURE_REMAP_COLOR_A_SHIFT);
    gt.width     = e->width;
    gt.height    = e->height;
    gt.depth     = 1;
    gt.location  = GCM_LOCATION_RSX;
    gt.pitch     = e->pitch;
    gt.offset    = e->offset;

    if (tex_bind_unchanged(unit, &gt, (u8)e->mips))
        return;
    tex_bind_remember(unit, &gt, (u8)e->mips);
    rsxLoadTexture(c, (u8)unit, &gt);
    tex_apply_sampler(c, unit, t, e->mips);
}

/* ------------------------------------------------------------------ */
/* Vertex arena                                                         */
/*                                                                      */
/* Decoded vertices are written into one RSX-visible arena per frame and drawn  */
/* from it. A bump allocator rather than a buffer per draw: a Wii frame issues  */
/* hundreds of small draws, and an allocation per draw would cost more than the */
/* drawing. It resets at frame start, which is safe because the RSX has been    */
/* waited on by the flip.                                                       */
/* ------------------------------------------------------------------ */

/* Position, normal, colour and four texture coordinates -- the shape every
 * generated program agrees on. Fixed layout so the attribute binding is
 * constant.
 *
 * The normal and the third and fourth coordinates are what a 3D scene needs and
 * a menu does not. Both were measured before they were added: of the 396 course
 * and kart materials read off the disc, 167 texgens take the *normal* as their
 * source row (that is what an environment map is), and 70 materials configure
 * more than two coordinates. Four rather than GX's eight because no material in
 * six courses and two karts asks for more than four, and each unused coordinate
 * is twelve bytes of vertex bandwidth on every vertex of every draw.
 *
 * The arena grows with the vertex so the *number* of vertices a frame can hold
 * does not shrink: it is the same 80k vertices it was at 52 bytes each. */
typedef struct {
    f32 pos[3];
    f32 nrm[3];
    f32 col[4];
    /* Three components: GX texgen input for an ST coordinate is (s, t, 1) --
     * the third component multiplies the matrix's translation column. Sending
     * (s, t, 0) dropped every atlas offset, which showed as glyphs clipped at
     * their right edges: the sub-rectangle translation never applied. */
    f32 tex[4][3];
} RenderVertex;

#define RENDER_VERTEX_TEXCOORDS 4

#define ATTR_POS  0
#define ATTR_NRM  2
#define ATTR_COL  3
#define ATTR_TEX(n) (8 + (n))

/* Port mode pays 45 MiB for the translated game before this is asked for, so
 * the emulator's worst-case headroom does not fit alongside it. 24 MiB is
 * still three times the measured in-race working set; the sizing loop below
 * falls back further if even that is unavailable. */
extern int g_wc_running;

#define VERTEX_ARENA_BYTES (g_wc_running ? (16 * 1024 * 1024) : (48 * 1024 * 1024)) /* 7MB = 83K verts/frame overflowed in-race (overflow=110417): the whole track scene was dropped after the arena filled -- black 3D world, HUD only */

static RenderVertex *s_arena;
static u32           s_arena_offset;
static u32           s_arena_used;      /* in vertices */
static u32           s_arena_capacity;

/* ------------------------------------------------------------------ */
/* Shader cache                                                         */
/* ------------------------------------------------------------------ */

#define SHADER_CACHE_SIZE 64
/* Sized for the longest program the generators can now emit, not for the
 * longest one the title screen needs. A menu's vertex program is nine
 * instructions; a channel lit by eight lights is twenty-two instructions per
 * light, and GX allows eight lights on each of two colour channels and their
 * two alpha channels independently. Overflow is not a wrong picture -- the
 * draw is dropped and `programs_failed` counts it -- but it is a whole object
 * missing, and 512 words was two lights away from that.
 *
 * 2048 words is 512 instructions, which is about where NV4x's own vertex
 * program limit sits: a configuration that does not fit here would not fit in
 * the hardware either, so this is the right place for the cap rather than an
 * arbitrary one. Measured: nine instructions for a menu draw, twenty for an
 * environment-mapped texgen, sixty-three for a channel lit by one spot light
 * on both its colour and its alpha. */
#define VP_WORDS_MAX 2048
#define FP_WORDS_MAX 2048

typedef struct {
    int  used;
    u64  key;
    RsxVertProgram vp;
    RsxFragProgram fp;
    u32  vp_code[VP_WORDS_MAX];
    /* What the generated vertex program reads out of the constant file. The
     * uploads below are driven from this rather than from the live state, so a
     * program and its constants can never disagree about, say, whether a
     * post-transform matrix is in play. */
    XFProgramInfo xinfo;
} ShaderEntry;

static ShaderEntry s_cache[SHADER_CACHE_SIZE];


/* One-line renderer statistics for the on-screen overlay. Deltas between
 * successive prints are what matter: a growing texture-cache occupancy or a
 * shader-cache miss count that never settles is the signature of the cache
 * thrash and video-memory leak that made the frame rate sag over minutes. */
/* Publish cache occupancy into the shared statistics block.
 *
 * texture_entries, program_entries and texture_bytes_live were declared but
 * never assigned, so every benchmark record reported zero for all three while
 * the same run showed textures decoded and hundreds of shader-cache HITS -- a
 * cache with no entries cannot serve hits, so the zeros were a reporting gap,
 * not a finding. They are the counters that say whether a washed-out frame is
 * textures failing to bind, which makes them worth having right. */
/* List the live texture cache: address, size, format and wrap mode.
 *
 * The background bands have a period that is CONSTANT in EFB space (~16 rows,
 * measured at both 480p and 720p output), which means a 16-texel-tall image is
 * repeating where it should be stretched. That is either a texture whose
 * height we decoded wrongly or one sampled with REPEAT where the title asked
 * for CLAMP -- and both are visible from here. */
void gx_render_dump_textures(void (*out)(const char *))
{
    unsigned i, n = 0;
    char line[160];
    for (i = 0; i < TEXCACHE_SLOTS && n < 40; i++) {
        TexEntry *e = &s_texcache[i];
        if (!e->valid) continue;
        n++;
        snprintf(line, sizeof line,
                 "  tex[%02u] addr=%08x %ux%u fmt=%u pitch=%u mips=%u",
                 i, (unsigned)e->addr, (unsigned)e->width, (unsigned)e->height,
                 (unsigned)e->fmt, (unsigned)e->pitch, (unsigned)e->mips);
        out(line);
    }
    snprintf(line, sizeof line, "  %u live texture(s)", n);
    out(line);
}

void gx_render_sample_caches(void)
{
    unsigned i, tex = 0, shd = 0;
    u32 bytes = 0;
    for (i = 0; i < TEXCACHE_SLOTS; i++)
        if (s_texcache[i].valid) {
            tex++;
            bytes += s_texcache[i].pitch * s_texcache[i].height;
        }
    for (i = 0; i < SHADER_CACHE_SIZE; i++)
        if (s_cache[i].used) shd++;
    g_gx_render.texture_entries   = tex;
    g_gx_render.program_entries   = shd;
    g_gx_render.texture_bytes_live = bytes;
}

char *gx_render_stats_line(char *buf, unsigned len)
{
    unsigned i, tex = 0;
    for (i = 0; i < TEXCACHE_SLOTS; i++)
        if (s_texcache[i].valid) tex++;
    {
        unsigned shd = 0, k;
        for (k = 0; k < SHADER_CACHE_SIZE; k++)
            if (s_cache[k].used) shd++;
        unsigned efb = 0, m;
        for (m = 0; m < EFB_COPY_TARGETS; m++) {
            EfbCopyTarget *ct = efb_copy_entry(m);
            if (ct && ct->valid) efb++;
        }
        snprintf(buf, len,
                 "TEX %u/%u  SHD %u/%u  DRAW %u  RTT %u/%u %llu/%llu  GEN!%llu",
                 tex, (unsigned)TEXCACHE_SLOTS,
                 shd, (unsigned)SHADER_CACHE_SIZE,
                 (unsigned)g_gx_render.draws,
                 efb, (unsigned)EFB_COPY_TARGETS,
                 (unsigned long long)g_gx_render.efb_copies_resolved,
                 (unsigned long long)g_efb_copy.binds_resolved,
                 (unsigned long long)g_gx_render.programs_failed);
        /* Only while Zcull is actually running, so the line is unchanged on a
         * build with the tiling group switched off. */
        if (rsx_zcull_active()) {
            size_t ln = strlen(buf);
            if (ln < len)
                snprintf(buf + ln, len - ln, "  ZK%llu",
                         (unsigned long long)g_gx_render.zcull_invalidating);
        }
    }
    return buf;
}

int gx_render_init(void)
{
    memset(&g_gx_render, 0, sizeof g_gx_render);
    memset(s_cache, 0, sizeof s_cache);
    memset(s_efb_surf, 0, sizeof s_efb_surf);
    efb_copy_reset();

    /* Take the largest arena RSX memory will give us, rather than demanding
     * one size and failing.
     *
     * This asked for 48 MiB and, when that allocation failed, returned -1 --
     * which leaves `s_arena` NULL, and the very first test in the draw path is
     * `if (!c || !s_arena || ...) return;`. So a failed allocation did not
     * degrade the renderer, it silently disabled it: every GX draw returned
     * immediately, the counters read "0 draws, 0 vertices, 0 programs", and
     * the console showed a black 3D world. That is the same picture the arena
     * was enlarged to fix, arrived at from the opposite direction.
     *
     * A smaller arena overflows in heavy scenes and drops the draws that do
     * not fit, which is bad; no arena drops all of them, which is worse. So
     * walk down until something succeeds and say which size won. */
    {
        static const size_t k_try[] = {
            48u << 20, 32u << 20, 24u << 20, 16u << 20, 8u << 20, 4u << 20
        };
        size_t i;
        /* Release the previous session's arena first. The main loop restarts
         * sessions in place, and this used to just overwrite the pointer: five
         * restarts leaked 48 MiB apiece until the allocator had nothing left
         * and the renderer came up disabled -- "NO vertex arena at any size"
         * on a console with a perfectly healthy 256 MiB of RSX memory. */
        if (s_arena) { rsxFree(s_arena); s_arena = NULL; s_arena_capacity = 0; }
        for (i = 0; i < sizeof k_try / sizeof k_try[0]; i++) {
            s_arena = (RenderVertex *)rsxMemalign(128, k_try[i]);
            if (!s_arena)
                continue;
            if (rsxAddressToOffset(s_arena, &s_arena_offset) != 0) {
                rsxFree(s_arena);
                s_arena = NULL;
                continue;
            }
            s_arena_capacity = k_try[i] / sizeof(RenderVertex);
            LOG_INFO(LOG_CORE, "gx_render: vertex arena %u MiB (%u verts)",
                     (unsigned)(k_try[i] >> 20), (unsigned)s_arena_capacity);
            break;
        }
        if (!s_arena) {
            LOG_ERROR(LOG_VIDEO, "gx_render: NO vertex arena at any size");
            return -1;
        }
    }
    s_arena_used = 0;
    return 0;
}

void gx_render_shutdown(void)
{
    unsigned i;
    for (i = 0; i < SHADER_CACHE_SIZE; i++)
        if (s_cache[i].used)
            rsx_fp_destroy(&s_cache[i].fp);
    memset(s_cache, 0, sizeof s_cache);
    {
        unsigned k;
        for (k = 0; k < EFB_COPY_TARGETS; k++)
            if (s_efb_surf[k].mem) { rsxFree(s_efb_surf[k].mem);
                                     s_efb_surf[k].mem = NULL; }
    }
    efb_copy_reset();
    if (s_arena) { rsxFree(s_arena); s_arena = NULL; }
}

void gx_render_frame_begin(void)
{
    s_tex_frame++;
#ifdef __PS3__
    {   extern void spu_vtx_arena_reset(void);
        extern void spu_vtx_join(void);
        spu_vtx_join();          /* prior frame fully decoded */
        spu_vtx_arena_reset();
    }
#endif
    pipeline_state_cache_invalidate();   /* one full emit per frame: self-heals
                                          * any out-of-band RSX state pokes */
    gcmContextData *c = rsx_context();

    prof_enter(PH_CMD);                 /* PHASE PROFILE */
    s_arena_used = 0;
    s_const_epoch++;                    /* invalidate the constant caches */
    {   extern void rsx_shader_new_frame(void);
        rsx_shader_new_frame();         /* and the program binding */
    }
    /* rsx_frame_begin has just written a frame-wide viewport and scissor, so
     * whatever the per-draw cache thinks is in the GPU is no longer true. */
    gx_view_state_dirty();

    /* Start every *title* frame with a fresh depth buffer.
     *
     * The platform only re-establishes the surface (and clears) on the frames
     * it actually presents, which at a reduced present rate is one title frame
     * in several. Colour survives that happily -- the next frame paints over
     * it -- but depth does not: now that the depth test is honoured, a stale
     * buffer from the previous title frame rejects the new frame's geometry
     * wherever it happens to sit further away. Clearing here rather than in
     * the present path ties the reset to the thing it belongs to, which is the
     * guest's frame rather than the display's. */
    if (c) {
        /* The scissor bounds the clear, and draws now set their own -- a
         * render-to-texture batch leaves it closed down to a 128x128 corner.
         * Without reopening it here the depth clear silently applies to that
         * corner only, on every title frame the platform does not present (it
         * re-establishes the surface only on the frames it flips). Cheap, once
         * per title frame, and the per-draw cache is told the GPU moved. */
        rsxSetScissor(c, 0, 0, (u16)g_rsx.width, (u16)g_rsx.height);
        gx_view_state_dirty();
        rsxSetClearDepthStencil(c, 0xffffff00u);
        rsxSetDepthWriteEnable(c, GCM_TRUE);   /* or the clear is a no-op */
        rsxClearSurface(c, GCM_CLEAR_Z | GCM_CLEAR_S);
        /* This clear is also what makes the Zcull region valid again, so the
         * Zcull surface is cleared and the unit re-enabled on the back of it.
         * It has to happen here and not only in rsx_clear: the platform
         * re-establishes the surface once per PRESENTED frame while this runs
         * once per TITLE frame, so at a reduced present rate most title frames
         * reach the guest's geometry through this path alone -- and a Zcull
         * region that is never revalidated after a depth clear rejects
         * fragments against the previous frame's depth bounds.
         *
         * The control registers this pairs with are emitted in
         * rsx_frame_begin. They survive from there to here because only a flip
         * resets the RSX context, and a flip is always followed by
         * rsx_frame_begin before anything else draws. */
        rsx_zcull_after_depth_clear();
    }
    prof_exit();                        /* PHASE PROFILE */
}
void gx_render_frame_end(void)   { }

/* ------------------------------------------------------------------ */
/* Shader generation                                                    */
/* ------------------------------------------------------------------ */

/* Build (or find) the program pair for the current GX state. Returns NULL if
 * the pair could not be produced, in which case the draw is skipped rather
 * than rendered wrongly. */
int g_shader_flush_req;
int g_fp_dump_req;

static ShaderEntry *shaders_for_state(const GXState *g)
{
    if (g_shader_flush_req) {
        unsigned fi;
        g_shader_flush_req = 0;
        for (fi = 0; fi < SHADER_CACHE_SIZE; fi++)
            if (s_cache[fi].used) rsx_fp_destroy(&s_cache[fi].fp);
        memset(s_cache, 0, sizeof s_cache);
        LOG_INFO(LOG_CORE, "shader cache flushed by request");
    }
    /* Skip the hash entirely when neither the XF nor the BP state has been
     * written since the last lookup. The two generation counters are bumped by
     * the parser on every write, so this is exact, not a guess -- and it turns
     * a full walk of the transform and TEV state into two compares for the
     * long runs of draws that share a material. */
    {
        extern u32 g_xf_generation, g_bp_generation;
        static u32 last_xf, last_bp;
        static ShaderEntry *last_e;
        if (last_e && last_xf == g_xf_generation && last_bp == g_bp_generation) {
            g_gx_render.program_hits++;
            return last_e;
        }
        last_xf = g_xf_generation; last_bp = g_bp_generation;
        {
            u64 key2 = xf_state_hash(g) ^
                       (tev_state_hash(&g->bp) * 0x9E3779B97F4A7C15ull);
            unsigned slot2 = (unsigned)(key2 % SHADER_CACHE_SIZE);
            ShaderEntry *e2 = &s_cache[slot2];
            if (e2->used && e2->key == key2) {
                g_gx_render.program_hits++;
                last_e = e2;
                return e2;
            }
            last_e = NULL;      /* a miss regenerates below; do not memoize it
                                 * until the entry is built and valid */
        }
    }

    u64 key = xf_state_hash(g) ^ (tev_state_hash(&g->bp) * 0x9E3779B97F4A7C15ull);
    unsigned slot = (unsigned)(key % SHADER_CACHE_SIZE);
    ShaderEntry *e = &s_cache[slot];

    if (e->used && e->key == key) {
        g_gx_render.program_hits++;
        return e;
    }

    /* Direct-mapped: a collision evicts. A title's hot set is a handful of
     * material configurations, so this is nearly always a hit; the cost of a
     * miss is regenerating two short programs, not a stall. */
    if (e->used)
        rsx_fp_destroy(&e->fp);
    memset(e, 0, sizeof *e);

    {
        VPEmitter ve;
        FPEmitter fe;
        XFProgramInfo xinfo;
        TevProgramInfo tinfo;
        static u32 fp_code[FP_WORDS_MAX];

        vp_init(&ve, e->vp_code, VP_WORDS_MAX);
        if (xf_generate(g, &ve, &xinfo) != 0 || ve.overflow || xinfo.truncated) {
            /* Counted, not only logged: a program that does not fit means the
             * draw is skipped, and a missing object with a warning buried in a
             * log the console cannot show is indistinguishable from geometry
             * that was never submitted. */
            g_gx_render.programs_failed++;
            LOG_WARN(LOG_VIDEO, "gx_render: vertex program generation failed");
            return NULL;
        }

        fp_init(&fe, fp_code, FP_WORDS_MAX);
        if (tev_generate(&g->bp, &fe, &tinfo) != 0 || fe.overflow ||
            tinfo.truncated) {
            g_gx_render.programs_failed++;
            LOG_WARN(LOG_VIDEO, "gx_render: fragment program generation failed");
            return NULL;
        }

        rsx_vp_create(&e->vp, e->vp_code, ve.used, ve.used / 4,
                      ve.input_mask, ve.output_mask);
        e->xinfo = xinfo;
        /* Every coordinate the generated TEV program reads is declared -- an
         * undeclared coordinate interpolates only its first two components,
         * which is how the fog coordinate would arrive frozen. */
        if (rsx_fp_create(&e->fp, fp_code, fe.used, tinfo.temps_used,
                          tev_texcoord_mask(&g->bp)) != 0)
            return NULL;
    }

    e->key  = key;
    e->used = 1;
    g_gx_render.programs_built++;
    return e;
}

/* ------------------------------------------------------------------ */
/* Pipeline state                                                       */
/*                                                                      */
/* Everything below is state the pixel engine holds in BP registers and the RSX */
/* holds in its own. Until this existed the RSX simply kept whatever the        */
/* previous owner of the GPU had left: no blending, no alpha test, no depth     */
/* test. That is not a subtle difference. Measured over a real Mario Kart Wii   */
/* title screen, 267065 draws in 268362 ask for GX_BM_BLEND with               */
/* (SRC_ALPHA, INV_SRC_ALPHA) and 31705 ask for an alpha test -- so essentially */
/* every transparent thing on the screen was being composited as though it were */
/* opaque, and every cut-out texel was being written instead of discarded.      */
/*                                                                              */
/* The state is written per draw rather than tracked and diffed. A Wii frame is */
/* a few hundred draws and these are two-word methods; the bookkeeping to skip  */
/* the redundant ones would cost more than the words it saved, and a cache that */
/* goes stale is a class of bug this file does not need.                        */
/* ------------------------------------------------------------------ */

/* GX's compare functions are numbered exactly as OpenGL's, and GCM's constants
 * are the OpenGL enumerants. So the mapping is arithmetic rather than a table
 * -- but it is asserted rather than assumed, because a table that happens to be
 * the identity is indistinguishable from one that was never checked. */
static u32 gx_compare_to_gcm(unsigned func)
{
    static const u32 k[8] = {
        GCM_NEVER, GCM_LESS, GCM_EQUAL, GCM_LEQUAL,
        GCM_GREATER, GCM_NOTEQUAL, GCM_GEQUAL, GCM_ALWAYS
    };
    return k[func & 7u];
}

/* The source and destination factor fields are *different enumerations* that
 * share their first two and last four entries: at index 2 and 3 a source
 * factor means the destination colour and a destination factor means the
 * source colour. One table for both would be wrong for exactly the two entries
 * a menu is least likely to use, which is the worst place for it to be wrong. */
static u16 gx_src_factor_to_gcm(unsigned f)
{
    static const u16 k[8] = {
        GCM_ZERO, GCM_ONE, GCM_DST_COLOR, GCM_ONE_MINUS_DST_COLOR,
        GCM_SRC_ALPHA, GCM_ONE_MINUS_SRC_ALPHA,
        GCM_DST_ALPHA, GCM_ONE_MINUS_DST_ALPHA
    };
    return k[f & 7u];
}

static u16 gx_dst_factor_to_gcm(unsigned f)
{
    static const u16 k[8] = {
        GCM_ZERO, GCM_ONE, GCM_SRC_COLOR, GCM_ONE_MINUS_SRC_COLOR,
        GCM_SRC_ALPHA, GCM_ONE_MINUS_SRC_ALPHA,
        GCM_DST_ALPHA, GCM_ONE_MINUS_DST_ALPHA
    };
    return k[f & 7u];
}

/* For the alpha channel a colour factor and the matching alpha factor are the
 * same number, so the colour forms are folded onto the alpha forms. Dolphin
 * does the same substitution and for the same reason: it lets one blend unit
 * serve both channels. */
static unsigned src_factor_alpha(unsigned f)
{
    if (f == 2) return 6;       /* dst colour -> dst alpha      */
    if (f == 3) return 7;       /* 1-dst colour -> 1-dst alpha  */
    return f;
}

static unsigned dst_factor_alpha(unsigned f)
{
    if (f == 2) return 4;       /* src colour -> src alpha      */
    if (f == 3) return 5;       /* 1-src colour -> 1-src alpha  */
    return f;
}

/* A destination-alpha factor only means anything if the framebuffer format the
 * title selected actually has alpha. GX_PF_RGB8_Z24 has none, and reading it
 * back gives 1.0 -- so the factors degenerate rather than sampling garbage. */
static unsigned remove_dst_alpha(unsigned f)
{
    if (f == 6) return 1;       /* dst alpha   -> one   */
    if (f == 7) return 0;       /* 1-dst alpha -> zero  */
    return f;
}

/* GX's alpha test is two comparisons combined by a logic op; the RSX has one.
 * Rather than approximate, the cases that *are* one comparison are reduced to
 * it exactly and everything else is left disabled and counted -- a test that is
 * not applied shows too much, which is recoverable and visible, where a test
 * applied wrongly shows too little and looks like missing geometry.
 *
 * On the Mario Kart Wii title screen every configuration reduces: 31608 draws
 * ask for (GREATER ref) AND ALWAYS, 97 for (GREATER ref) OR NEVER, and the
 * rest are ALWAYS/ALWAYS. */
static int alpha_test_to_gcm(const BPAlphaTest *at, u32 *func, u32 *ref)
{
    unsigned c0 = at->comp0 & 7u, c1 = at->comp1 & 7u;
    unsigned r0 = at->ref0 & 0xFFu, r1 = at->ref1 & 0xFFu;
    unsigned logic = at->logic & 3u;

    /* AND: an ALWAYS operand contributes nothing, a NEVER operand decides. */
    if (logic == 0) {
        if (c0 == 7 && c1 == 7) return 0;              /* always passes  */
        if (c0 == 0 || c1 == 0) { *func = GCM_NEVER; *ref = 0; return 1; }
        if (c1 == 7) { *func = gx_compare_to_gcm(c0); *ref = r0; return 1; }
        if (c0 == 7) { *func = gx_compare_to_gcm(c1); *ref = r1; return 1; }
    }
    /* OR: a NEVER operand contributes nothing, an ALWAYS operand decides. */
    if (logic == 1) {
        if (c0 == 7 || c1 == 7) return 0;              /* always passes  */
        if (c0 == 0 && c1 == 0) { *func = GCM_NEVER; *ref = 0; return 1; }
        if (c1 == 0) { *func = gx_compare_to_gcm(c0); *ref = r0; return 1; }
        if (c0 == 0) { *func = gx_compare_to_gcm(c1); *ref = r1; return 1; }
    }
    /* Both halves identical: the logic op cannot change the answer. */
    if (c0 == c1 && r0 == r1 && logic <= 1) {
        *func = gx_compare_to_gcm(c0); *ref = r0; return 1;
    }
    g_gx_render.alpha_test_unmapped++;
    return 0;
}

/* Runtime-selectable pipeline-state groups. Writing GX render state to the
 * RSX is correct in principle -- 99.5% of title-screen draws ask for alpha
 * blending -- but it regressed the console (blurred image, scan lines, half
 * the frame rate), and the console is the only oracle. Rather than spend the
 * user's launches on guesses, each group is switchable from the pad at run
 * time: mask 0 reproduces the previously-good build exactly, and stepping up
 * adds one group at a time so a single session names the offender.
 *   bit0 blend   bit1 alpha test   bit2 depth   bit3 masks/cull/logic
 *   bit4 viewport   bit5 scissor   bit6 EFB copy to texture   bit7 copy clear
 *
 * The four render-to-texture groups are on by default, because the first two
 * are the identity for every full-screen draw (174,186 of the 174,282 draws in
 * a complete Mario Kart Wii boot -- see the viewport section) and the last two
 * only act on copies this backend previously ignored altogether. If hardware
 * disagrees, the pad steps them off one at a time exactly as it did for the
 * depth test. */
/* Hardware verdict: blending is REQUIRED (mask 0 draws opaque white blocks
 * over the title screen; mask 1 renders the "Press the A Button" text
 * correctly), and DEPTH TESTING is what produced the scan lines and blurred
 * image -- every mask including bit2 broke the picture, every mask without it
 * was clean. Default to blend + alpha test; depth stays off until the depth
 * buffer is set up and cleared properly.
 *
 * UPDATE: both depth defects are now fixed -- the window mapping was putting
 * the NEAR plane at the maximum depth value (so GX_LEQUAL kept the FARTHEST
 * fragment of every pixel), and the per-frame Z clear was silently gated off
 * by the depth write mask left behind by the previous frame's blended UI
 * draws. Depth is therefore ON by default now; it is required for any 3D
 * scene, and L2 still steps the groups back if hardware disagrees. */
/* The mask itself is defined in gx_features.c, so the shader generators can
 * read it without linking any of the RSX. */

/* Everything apply_pipeline_state emits is a pure function of these raw BP
 * registers plus the debug toggles. At the race's ~11k draws/frame the
 * pipeline state barely changes between consecutive draws, so re-emitting the
 * full blend/depth/cull/alpha block per draw was pure command-buffer and CPU
 * overhead. The key is compared per draw; the block is emitted only when it
 * actually changed. Anything that pokes the RSX state OUTSIDE this function
 * (the copy-clear path, session restarts) must call
 * pipeline_state_cache_invalidate(). */
static u32 s_pipe_key[8];
static int s_pipe_key_valid;
void pipeline_state_cache_invalidate(void) { s_pipe_key_valid = 0; }

static void apply_pipeline_state(gcmContextData *c, const BPState *bp)
{
    if (g_gx_state_mask == 0) return;   /* known-good baseline */
    {
        extern int g_tev_show_alpha; extern int g_tev_show_red;
        u32 key[8];
        key[0] = bp->raw[0x40]; key[1] = bp->raw[0x41];
        key[2] = bp->raw[0x42]; key[3] = bp->raw[BP_ZCOMPARE];
        key[4] = bp->raw[0xF3];              /* alpha test func/refs */
        key[5] = bp->raw[0x00];              /* genmode: cull mode   */
        key[6] = g_gx_state_mask;
        key[7] = ((u32)(g_tev_show_alpha & 1) << 1) | (u32)(g_tev_show_red & 1);
        if (s_pipe_key_valid && memcmp(key, s_pipe_key, sizeof key) == 0)
            return;
        memcpy(s_pipe_key, key, sizeof key);
        s_pipe_key_valid = 1;
    }
    /* GX_PF_RGBA6_Z24 is the only pixel format with a destination alpha
     * channel; the field is BP 0x43 bits 0-2. */
    int has_dst_alpha = ((bp->raw[BP_ZCOMPARE] & 7u) == 1u);
    u32 afunc, aref;

    /* --- blending -------------------------------------------------- */
    {   /* Alpha-view debugging writes the fragment alpha as colour; blending
         * must be off or the view is composited into uselessness. */
        extern int g_tev_show_alpha;
        if (g_tev_show_alpha) {
            rsxSetBlendEnable(c, GCM_FALSE);
            rsxSetLogicOpEnable(c, GCM_FALSE);
            goto after_blend;
        }
    }
    if (bp->blend.blend_enable) {
        unsigned s = bp->blend.src_factor, d = bp->blend.dst_factor;
        if (bp->blend.subtract) {
            /* GX's subtract mode ignores the factor fields entirely and
             * computes dst - src. Feeding the factors through anyway would
             * scale a subtraction that the hardware does not scale. */
            if (!(g_gx_state_mask & 1)) return;
            rsxSetBlendEnable(c, GCM_TRUE);
            rsxSetBlendFunc(c, GCM_ONE, GCM_ONE, GCM_ONE, GCM_ONE);
            rsxSetBlendEquation(c, GCM_FUNC_REVERSE_SUBTRACT,
                                GCM_FUNC_REVERSE_SUBTRACT);
        } else {
            if (!has_dst_alpha) {
                s = remove_dst_alpha(s);
                d = remove_dst_alpha(d);
            }
            rsxSetBlendEnable(c, GCM_TRUE);
            rsxSetBlendFunc(c, gx_src_factor_to_gcm(s), gx_dst_factor_to_gcm(d),
                            gx_src_factor_to_gcm(src_factor_alpha(s)),
                            gx_dst_factor_to_gcm(dst_factor_alpha(d)));
            rsxSetBlendEquation(c, GCM_FUNC_ADD, GCM_FUNC_ADD);
        }
        rsxSetLogicOpEnable(c, GCM_FALSE);
    } else if (bp->blend.logic_enable) {
        /* On GX the logic op replaces blending rather than following it, so
         * the two are never both live. GX numbers its sixteen operations the
         * way OpenGL does, and GCM takes the OpenGL enumerants. */
        rsxSetBlendEnable(c, GCM_FALSE);
        rsxSetLogicOp(c, 0x1500u + (bp->blend.logic_op & 15u));
        rsxSetLogicOpEnable(c, GCM_TRUE);
    } else {
        rsxSetBlendEnable(c, GCM_FALSE);
        rsxSetLogicOpEnable(c, GCM_FALSE);
    }

after_blend:
    /* --- alpha test ------------------------------------------------ */
    if (alpha_test_to_gcm(&bp->alpha_test, &afunc, &aref)) {
        rsxSetAlphaFunc(c, afunc, aref);
        if (g_gx_state_mask & 2) rsxSetAlphaTestEnable(c, GCM_TRUE);
    } else {
        if (g_gx_state_mask & 2) rsxSetAlphaTestEnable(c, GCM_FALSE);
    }

    /* --- depth ----------------------------------------------------- */
    if (g_gx_state_mask & 4)
        rsxSetDepthFunc(c, gx_compare_to_gcm(bp->zmode.func));
    if (g_gx_state_mask & 4)
        rsxSetDepthTestEnable(c, bp->zmode.enable ? GCM_TRUE : GCM_FALSE);
    if (g_gx_state_mask & 4)
        rsxSetDepthWriteEnable(c, bp->zmode.update_enable ? GCM_TRUE : GCM_FALSE);
    /* Counted, not prevented. The depth function is the guest's and cannot be
     * overridden without rendering the wrong picture; what a count buys is the
     * ability to tell "Zcull is configured and rejecting nothing" apart from
     * "Zcull is configured and there is nothing to reject". GX compare codes:
     * 4 GREATER, 5 NOTEQUAL, 6 GEQUAL, 7 ALWAYS -- all on the far side of a
     * LESS-direction Zcull, which permits only NEVER, LESS, EQUAL and LEQUAL
     * (codes 0..3). A draw with the depth TEST off and depth writes on counts
     * too: the hardware then writes depth unconditionally, which is ALWAYS by
     * another name. */
    if (bp->zmode.update_enable &&
        (!bp->zmode.enable || (bp->zmode.func & 7u) >= 4u))
        g_gx_render.zcull_invalidating++;

    /* --- write masks ----------------------------------------------- */
    {
        u32 mask = 0;
        if (bp->blend.color_update)
            mask |= GCM_COLOR_MASK_R | GCM_COLOR_MASK_G | GCM_COLOR_MASK_B;
        if (bp->blend.alpha_update && has_dst_alpha)
            mask |= GCM_COLOR_MASK_A;
        if (g_gx_state_mask & 8) rsxSetColorMask(c, mask);
    }

    /* --- culling ---------------------------------------------------- */
    /* GX's rule is known exactly. Dolphin's software clipper calls a triangle
     * front-facing when the signed area of its clip-space projection is
     * positive, and this backend's clip-to-window mapping is bit-for-bit GX's
     * (both put y_ndc = +1 at the top of the viewport), so a triangle that is
     * front-facing on the Wii is front-facing here in exactly the same sense.
     *
     * What is *not* known off the console is which of GCM's two winding names
     * describes that sense once the viewport's negative y scale has reversed
     * the orientation: NV4x states its rule against its own raster space and
     * nothing here says which handedness that is. So both answers are
     * reachable -- GX_STATE_CULL alone, and GX_STATE_CULL|GX_STATE_CULL_FLIP
     * -- and one console launch settles it. Getting it backwards shows the
     * inside of every kart, which is obvious and recoverable; it is not a
     * black screen, because GX_CULL_ALL is handled in the draw path and never
     * reaches the rasteriser.
     *
     * And the title screen is a stronger test than a race would be, which is
     * the reason this group is off by default rather than on: 173,061 of the
     * 174,282 draws in a complete boot ask for GX_CULL_BACK and only 1,221 ask
     * for nothing, so the winding is not a subtlety that shows up later -- it
     * decides whether the screen the console already renders survives. */
    if (g_gx_state_mask & GX_STATE_CULL) {
        /* Settled from the RSX Users Manual rather than by experiment. The
         * hardware screen space has its ORIGIN AT THE UPPER LEFT with +y
         * DOWNWARD (3.2.1), and the facing test is applied AFTER the viewport
         * transform (3.3.1-3.3.2); the manual notes explicitly that a
         * lower-left-origin API must negate scale.y and that doing so INVERTS
         * front and back faces. Our NDC has y = +1 at the top and rsx_video.c
         * negates scale.y, so a triangle the guest treats as front-facing
         * arrives counter-clockwise in raster space: CCW is the correct
         * default, and CULL_FLIP now selects the disproved answer. */
        unsigned front = (g_gx_state_mask & GX_STATE_CULL_FLIP)
                       ? GCM_FRONTFACE_CW : GCM_FRONTFACE_CCW;
        switch (bp->genmode.cull) {
        case BP_CULL_BACK:
            rsxSetFrontFace(c, front);
            rsxSetCullFace(c, GCM_CULL_BACK);
            rsxSetCullFaceEnable(c, GCM_TRUE);
            break;
        case BP_CULL_FRONT:
            rsxSetFrontFace(c, front);
            rsxSetCullFace(c, GCM_CULL_FRONT);
            rsxSetCullFaceEnable(c, GCM_TRUE);
            break;
        case BP_CULL_ALL:       /* the draw path already dropped it */
        case BP_CULL_NONE:
        default:
            rsxSetCullFaceEnable(c, GCM_FALSE);
            break;
        }
    } else {
        if (bp->genmode.cull != BP_CULL_NONE)
            g_gx_render.cull_unmapped++;
        if (g_gx_state_mask & GX_STATE_MASKS) rsxSetCullFaceEnable(c, GCM_FALSE);
    }

    /* --- what is still missing ------------------------------------- */
    /* Indirect texturing is generated into the fragment program now, so this
     * only counts what the feature mask is *refusing* to generate. */
    if (bp->genmode.num_indstages && !(g_gx_state_mask & GX_STATE_INDIRECT))
        g_gx_render.indirect_unmapped++;
    if (bp->genmode.num_texgens > RENDER_VERTEX_TEXCOORDS)
        g_gx_render.texcoords_unmapped++;
    if (bp->fog.fsel && bp->fog.range_enable)
        g_gx_render.fog_range_unmapped++;
}

/* GX stores six projection parameters, not a matrix: the hardware expands them
 * on the fly, and the two cases put the same six numbers in different places.
 * Row-major, because the generated vertex program transforms with one DP4 per
 * row. Getting the perspective w-row wrong (it is -1, not 1) yields geometry
 * that is present, correctly shaped, and never visible -- so it is written out
 * explicitly rather than left to a memset. */
static void gx_projection_matrix(const XFState *xf, f32 m[16])
{
    const float *r = xf->projection;
    unsigned i;
    for (i = 0; i < 16; i++) m[i] = 0.0f;

    if (xf->projection_orthographic) {
        m[0]  = r[0];  m[3]  = r[1];
        m[5]  = r[2];  m[7]  = r[3];
        m[10] = r[4];  m[11] = r[5];
        m[15] = 1.0f;
    } else {
        m[0]  = r[0];  m[2]  = r[1];
        m[5]  = r[2];  m[6]  = r[3];
        m[10] = r[4];  m[11] = r[5];
        m[14] = -1.0f;
    }

    /* Depth, and the direction of it.
     *
     * Gekko's clip space puts z in [-w, 0]: the near plane is 0 and the far
     * plane is -w. Every GPU since maps [0, w] (or [-w, w]) instead, so the
     * projection above hands the rasteriser a depth that runs *backwards* --
     * near geometry ends up with the larger depth value. That is invisible
     * while nothing tests depth, which is exactly how it survived: the frame
     * looks right and the buffer is upside down. Turn GX_LEQUAL on top of it
     * and the pipeline keeps the furthest fragment of every pixel, which is
     * the whole frame drawn back to front.
     *
     * The fix is one negation, folded into the matrix rather than the viewport
     * so it is independent of which normalised-device convention the RSX is
     * configured for -- either way the result is monotonic in Gekko's depth and
     * inside the buffer's range. The transform is Dolphin's:
     *
     *     z' = w * (1 - farZ/2^24) - z * (zRange/2^24)
     *
     * with the viewport's own depth range folded in. GX_SetViewport's defaults
     * give zRange = farZ = 2^24 - 1, which reduces to a plain z' = -z. */
    {
        /* Direct GX semantics, measured on hardware rather than derived:
         *
         *   screenZ/2^24 = vo2/2^24 + (vs2/2^24) * z_clip,  z_clip in [-1,0]
         *
         * (smaller screenZ = nearer; GX clears to far and compares LEQUAL).
         * The previous fold used far_n = 1 - vo2/2^24 with a negated range --
         * for the title screen's viewport (vs2=0.1*2^24, vo2=0.11*2^24) that
         * REVERSED near and far, the frame-composite quads landed in front of
         * the scene, and every later draw failed the depth test: the whole
         * screen showed the previous frame's blurred copy. Depth off produced
         * a pixel-perfect title screen, which is what pinned it here.
         *
         * The RSX rasterises ndc z through its own 0.5/0.5 window transform,
         * so the matrix must hand it ndc = 2*screen - 1. */
        /* The depth-range fold is applied ONLY for ORTHO. For an ortho
         * projection the w row is the constant (0,0,0,1), so folding the
         * viewport depth range into the z row via the w column is exact -- and
         * it is what makes the title screen's 2D composite depth-order
         * correctly (pixel-perfect, verified).
         *
         * For PERSPECTIVE the w row is (0,0,-1,0) = -z_eye, so the same fold
         * corrupts depth: after the perspective divide,
         *   screen_z = (foldedZ)/(-z_eye) = 0.82 + 1.60/z_eye,
         * which is ~-15 for near geometry -> clipped -> the whole 3D track
         * rendered BLACK while the ortho HUD was fine. The raw GC perspective
         * projection already yields ndc_z in [-1,0] (near plane ~ z=-1, far at
         * infinity), which the RSX's 0.5/0.5 window transform maps into a
         * valid depth range, so perspective needs NO fold. Captured in-race:
         * proj={1.81,0,2.41,0,-0.0001,-1.0001}, vs2=0.80, vo2=0.91. */
        if (xf->projection_orthographic) {
            const float k_max = 16777215.0f;
            float range2 = xf->viewport_scale[2] / k_max;
            float far2   = xf->viewport_offset[2] / k_max;
            unsigned c;
            if (!(range2 > 0.0009765625f && range2 <= 1.0f) ||
                !(far2 >= 0.0f && far2 <= 1.0f)) {
                range2 = 1.0f; far2 = 1.0f;
            }
            for (c = 0; c < 4; c++)
                m[8 + c] = (2.0f * far2 - 1.0f) * m[12 + c]
                         + 2.0f * range2 * m[8 + c];
        }
    }
}

/* ------------------------------------------------------------------ */
/* Draw                                                                 */
/* ------------------------------------------------------------------ */

static u32 rsx_primitive(GXPrimitive p)
{
    switch (p) {
    case GX_POINTS:         return GCM_TYPE_POINTS;
    case GX_LINES:          return GCM_TYPE_LINES;
    case GX_LINE_STRIP:     return GCM_TYPE_LINE_STRIP;
    case GX_TRIANGLES:      return GCM_TYPE_TRIANGLES;
    case GX_TRIANGLE_STRIP: return GCM_TYPE_TRIANGLE_STRIP;
    case GX_TRIANGLE_FAN:   return GCM_TYPE_TRIANGLE_FAN;
    /* The Wii's quads have no RSX equivalent, but a quad list is exactly a
     * triangle fan per four vertices; RSX QUADS does the same decomposition. */
    case GX_QUADS:
    case GX_QUADS_2:        return GCM_TYPE_QUADS;
    default:                return GCM_TYPE_TRIANGLES;
    }
}

static void on_draw(void *ctx, const struct GXState *state, GXPrimitive prim,
                    unsigned vat, u16 vertex_count, u32 data_addr,
                    u32 vertex_size)
{
    const GXState *g = state;
    gcmContextData *c = rsx_context();
    ShaderEntry *sh;
    RenderVertex *out;
    u32 base_vertex, addr;
    unsigned i, t;

    (void)ctx;
    g_gx_render.draws++;

    {   /* Draw-window bisect (devlink "dwin <min> <max>"): render only draws
         * whose within-frame index lies in [min,max). Identifying WHICH draw
         * paints an artefact beats theorising about it; the counter resets in
         * rsx_frame_begin via g_draw_frame_base. */
        extern u32 g_draw_win_min, g_draw_win_max;
extern u32 g_draw_info_idx; extern int g_draw_info_arm; extern u32 g_draw_info_minv;
        extern u64 g_draw_frame_base;
        if (g_draw_win_max) {
            u64 idx = g_gx_render.draws - 1 - g_draw_frame_base;
            if (idx < (u64)g_draw_win_min || idx >= (u64)g_draw_win_max)
                return;
        }
        {   /* One-shot state dump for a chosen within-frame draw index
             * (devlink "dinfo <n>"). What blend, TEV count and texture a draw
             * uses is a fact, not a debate. */
            extern u32 g_draw_info_idx; extern int g_draw_info_arm;
            u64 idx = g_gx_render.draws - 1 - g_draw_frame_base;
            extern u32 g_draw_info_minv;
            extern u32 g_census_agg[4];   /* draws, verts, in-frustum, wneg */
            int hit = g_draw_info_arm &&
                      (g_draw_info_minv ? (vertex_count >= g_draw_info_minv)
                                        : (idx == (u64)g_draw_info_idx));
            /* Aggregate mode: minv armed counts EVERY matching draw's frustum
             * stats for one frame instead of dumping the first. */
            if (g_draw_info_arm && g_draw_info_minv && vertex_count >= g_draw_info_minv) {
                unsigned inf9 = 0, wn9 = 0, n9c = 0, v9c;
                f32 P9[16];
                union { u32 u; f32 f; } cv9;
                u32 a10 = data_addr;
                gx_projection_matrix(&g->xf, P9);
                for (v9c = 0; v9c < vertex_count && v9c < 16; v9c++) {
                    VtxAttributes vh; f32 M9[16], cl[4], in9[4], ey[4];
                    unsigned rr, cc9;
                    u32 c10 = vtx_decode(&g->parser.cp, vat, a10, &vh);
                    u32 pidx = vh.has_pos_matrix_index
                             ? (vh.pos_matrix_index & 0x3Fu)
                             : (g->xf.mem[0x1018] & 0x3Fu);
                    if (!c10) break;
                    a10 += c10; n9c++;
                    for (rr = 0; rr < 3; rr++)
                        for (cc9 = 0; cc9 < 4; cc9++) {
                            cv9.u = g->xf.mem[pidx*4u + rr*4 + cc9];
                            M9[rr*4+cc9] = cv9.f;
                        }
                    in9[0]=vh.position[0]; in9[1]=vh.position[1];
                    in9[2]=(vh.position_count>=3)?vh.position[2]:0.0f;
                    for (rr = 0; rr < 3; rr++)
                        ey[rr] = M9[rr*4]*in9[0]+M9[rr*4+1]*in9[1]
                               + M9[rr*4+2]*in9[2]+M9[rr*4+3];
                    ey[3]=1.0f;
                    for (rr = 0; rr < 4; rr++)
                        cl[rr] = P9[rr*4]*ey[0]+P9[rr*4+1]*ey[1]
                               + P9[rr*4+2]*ey[2]+P9[rr*4+3]*ey[3];
                    if (cl[3] <= 0.0f) { wn9++; continue; }
                    if (cl[0] >= -cl[3] && cl[0] <= cl[3] &&
                        cl[1] >= -cl[3] && cl[1] <= cl[3]) inf9++;
                }
                g_census_agg[0]++; g_census_agg[1]+=n9c;
                g_census_agg[2]+=inf9; g_census_agg[3]+=wn9;
                if (g_census_agg[0] >= 40) {
                    LOG_INFO(LOG_CORE, "DAGG draws=%u verts=%u infrustum=%u wneg=%u",
                             g_census_agg[0], g_census_agg[1],
                             g_census_agg[2], g_census_agg[3]);
                    g_draw_info_arm = 0; g_draw_info_minv = 0;
                    g_census_agg[0]=g_census_agg[1]=g_census_agg[2]=g_census_agg[3]=0;
                }
                hit = 0;
            }
            if (hit) {
                const BPTexture *t0 = &g->bp.tex[g->bp.tev[0].tex_map & 7u];
                g_draw_info_arm = 0;
                LOG_INFO(LOG_CORE, "DINFO draw=%llu verts=%u stages=%u "
                         "blend=%u sub=%u src=%u dst=%u dither=%u "
                         "tex0=%08x %ux%u fmt=%u mode0=%06x logic=%u",
                         (unsigned long long)idx, (unsigned)vertex_count,
                         g->bp.genmode.num_tev_stages,
                         g->bp.blend.blend_enable, g->bp.blend.subtract,
                         g->bp.blend.src_factor, g->bp.blend.dst_factor,
                         g->bp.blend.dither,
                         (unsigned)t0->address, t0->width, t0->height,
                         t0->format, (unsigned)t0->mode0,
                         g->bp.blend.logic_enable);
                {
                    unsigned r6;
                    for (r6 = 0; r6 < 4; r6++)
                        LOG_INFO(LOG_CORE,
                            "DTEV reg%u std={%.3f %.3f %.3f %.3f} "
                            "konst={%.3f %.3f %.3f %.3f}", r6,
                            g->bp.tev_reg[r6][0], g->bp.tev_reg[r6][1],
                            g->bp.tev_reg[r6][2], g->bp.tev_reg[r6][3],
                            g->bp.tev_konst[r6][0], g->bp.tev_konst[r6][1],
                            g->bp.tev_konst[r6][2], g->bp.tev_konst[r6][3]);
                }
                {
                    u32 mi = g->xf.mem[0x1018];
                    u32 ti = (mi >> 6) & 0x3Fu;
                    const u32 *tr = &g->xf.mem[ti * 4u];
                    union { u32 u; float f; } c9;
                    float row0[4], row1[4]; unsigned q9;
                    for (q9 = 0; q9 < 4; q9++) { c9.u = tr[q9]; row0[q9]=c9.f; }
                    for (q9 = 0; q9 < 4; q9++) { c9.u = tr[4+q9]; row1[q9]=c9.f; }
                    {
                        const u32 *pq = &g->xf.mem[0x0500u + 61u * 4u];
                        union { u32 u; float f; } c8;
                        float p0[4]; unsigned q8;
                        for (q8 = 0; q8 < 4; q8++) { c8.u = pq[q8]; p0[q8]=c8.f; }
                        LOG_INFO(LOG_CORE, "DVPXY vp={%g %g %g %g} sc={%d %d %d %d}",
                             g->xf.viewport_scale[0], g->xf.viewport_scale[1],
                             g->xf.viewport_offset[0], g->xf.viewport_offset[1],
                             g->bp.scissor_left, g->bp.scissor_top,
                             g->bp.scissor_right, g->bp.scissor_bottom);
                        LOG_INFO(LOG_CORE, "DVP vs2=%g vo2=%g zmode_raw=%06x "
                             "ortho=%u proj={%g %g %g %g %g %g}",
                             g->xf.viewport_scale[2], g->xf.viewport_offset[2],
                             g->bp.raw[0x40], g->xf.projection_orthographic,
                             g->xf.projection[0], g->xf.projection[1],
                             g->xf.projection[2], g->xf.projection[3],
                             g->xf.projection[4], g->xf.projection[5]);
                        {   /* The full combined-matrix z and w rows AFTER the
                             * depth fold -- the numbers the RSX actually
                             * rasterises. For perspective the w row is not
                             * constant, so this is where a wrong depth fold
                             * shows up as z that clips or inverts. */
                            f32 mm[16];
                            gx_projection_matrix(&g->xf, mm);
                            LOG_INFO(LOG_CORE, "DMZ z={%g %g %g %g} w={%g %g %g %g}",
                                     mm[8],mm[9],mm[10],mm[11],
                                     mm[12],mm[13],mm[14],mm[15]);
                        }
                    LOG_INFO(LOG_CORE, "DTG2 dualflag=%u post61r0={%g %g %g %g}",
                                 g->xf.mem[0x1012] & 1u, p0[0], p0[1], p0[2], p0[3]);
                    }
                    LOG_INFO(LOG_CORE, "DTG spec=%08x dual=%08x mtxA=%08x "
                             "ti=%u r0={%g %g %g %g} r1={%g %g %g %g}",
                             g->xf.mem[0x1040], g->xf.mem[0x1050], mi, ti,
                             row0[0],row0[1],row0[2],row0[3],
                             row1[0],row1[1],row1[2],row1[3]);
                }
                {   /* All four vertices' raw UV0 and position: the flat
                     * Dolphin output means its sampled coordinate is constant,
                     * so if OUR decoded UVs vary across these verts the fault
                     * is the vertex decode; if they match, it is the texgen
                     * (post-matrix) path. */
                    u32 a9 = data_addr; unsigned v9;
                    for (v9 = 0; v9 < vertex_count && v9 < 4; v9++) {
                        VtxAttributes va9;
                        u32 c9 = vtx_decode(&g->parser.cp, vat, a9, &va9);
                        LOG_INFO(LOG_CORE, "DVTX%u tc0={%g %g}(n=%u) "
                                 "pos={%g %g %g}", v9,
                                 va9.texcoord[0][0], va9.texcoord[0][1],
                                 va9.texcoord_count[0],
                                 va9.position[0], va9.position[1],
                                 va9.position[2]);
                        if (!c9) break;
                        a9 += c9;
                    }
                }
                {   /* Frustum census: run up to 32 vertices through the
                     * combined matrix, EACH WITH ITS OWN pos-matrix index
                     * (skinned models select per-vertex). Prints how many land
                     * in-frustum -- the direct answer to "should this draw be
                     * visible", per vertex rather than per draw. */
                    unsigned inf = 0, wneg = 0, n9 = 0, v9c;
                    f32 P9[16];
                    union { u32 u; f32 f; } cv9;
                    u32 a10 = data_addr;
                    gx_projection_matrix(&g->xf, P9);
                    for (v9c = 0; v9c < vertex_count && v9c < 32; v9c++) {
                        VtxAttributes vh; f32 M9[16], cl[4], in9[4];
                        unsigned rr, cc9, kk;
                        u32 c10 = vtx_decode(&g->parser.cp, vat, a10, &vh);
                        u32 pidx = vh.has_pos_matrix_index
                                 ? (vh.pos_matrix_index & 0x3Fu)
                                 : (g->xf.mem[0x1018] & 0x3Fu);
                        if (!c10) break;
                        a10 += c10; n9++;
                        for (rr = 0; rr < 3; rr++)
                            for (cc9 = 0; cc9 < 4; cc9++) {
                                cv9.u = g->xf.mem[pidx * 4u + rr * 4 + cc9];
                                M9[rr * 4 + cc9] = cv9.f;
                            }
                        in9[0]=vh.position[0]; in9[1]=vh.position[1];
                        in9[2]=(vh.position_count>=3)?vh.position[2]:0.0f;
                        in9[3]=1.0f;
                        {   f32 ey[4];
                            for (rr = 0; rr < 3; rr++)
                                ey[rr] = M9[rr*4+0]*in9[0]+M9[rr*4+1]*in9[1]
                                       + M9[rr*4+2]*in9[2]+M9[rr*4+3];
                            ey[3]=1.0f;
                            for (rr = 0; rr < 4; rr++)
                                cl[rr] = P9[rr*4+0]*ey[0]+P9[rr*4+1]*ey[1]
                                       + P9[rr*4+2]*ey[2]+P9[rr*4+3]*ey[3];
                        }
                        if (v9c < 2) {
                            LOG_INFO(LOG_CORE, "DCLIP v%u pidx=%u pos={%g %g %g} clip={%g %g %g %g}",
                                     v9c, pidx, in9[0], in9[1], in9[2],
                                     cl[0], cl[1], cl[2], cl[3]);
                            if (v9c == 0)
                                LOG_INFO(LOG_CORE,
                                    "DM r0={%g %g %g %g} r1={%g %g %g %g} r2={%g %g %g %g}",
                                    M9[0],M9[1],M9[2],M9[3],
                                    M9[4],M9[5],M9[6],M9[7],
                                    M9[8],M9[9],M9[10],M9[11]);
                        }
                        if (cl[3] <= 0.0f) { wneg++; continue; }
                        if (cl[0] >= -cl[3] && cl[0] <= cl[3] &&
                            cl[1] >= -cl[3] && cl[1] <= cl[3])
                            inf++;
                    }
                    LOG_INFO(LOG_CORE, "DCENSUS %u/%u in-frustum, %u w<=0",
                             inf, n9, wneg);
                }
                LOG_INFO(LOG_CORE, "DINFO2 pe_control=%06x aup=%u cup=%u",
                         g->bp.pe_control, g->bp.blend.alpha_update,
                         g->bp.blend.color_update);
                {   /* Full nonzero BP register dump: everything needed to
                     * hand-execute the TEV offline against Dolphin's decode. */
                    unsigned r;
                    for (r = 0; r < 256; r += 4) {
                        u32 a4=g->bp.raw[r],b4=g->bp.raw[r+1],
                            c4=g->bp.raw[r+2],d4=g->bp.raw[r+3];
                        if (a4|b4|c4|d4)
                            LOG_INFO(LOG_CORE,
                                     "DREG %02x: %06x %06x %06x %06x",
                                     r, a4, b4, c4, d4);
                    }
                }
            }
        }
    }

    if (!c || !s_arena || vertex_count == 0)
        return;

    /* GX_CULL_ALL discards both faces, which is a title's way of saying "draw
     * nothing" without unbuilding the display list. It is the one culling mode
     * whose meaning does not depend on a winding convention, so it is the one
     * this backend can honour exactly. */
    if (g->bp.genmode.cull == BP_CULL_ALL) {
        g_gx_render.cull_unmapped++;
        return;
    }

    {   /* Draw-size filter: render ONLY draws with >= g_draw_minverts
         * vertices (devlink `dbig N`). Isolates the 3D models from the UI
         * unambiguously -- index windows drift with per-frame draw counts. */
        extern u32 g_draw_minverts;
        if (g_draw_minverts && vertex_count < g_draw_minverts)
            return;
    }
    if (s_arena_used + vertex_count > s_arena_capacity) {
        g_gx_render.overflow++;
        return;
    }

    prof_enter(PH_SHADER);              /* PHASE PROFILE */
    sh = shaders_for_state(g);
    prof_exit();
    if (!sh)
        return;

    prof_enter(PH_CMD);                 /* PHASE PROFILE */
    apply_pipeline_state(c, &g->bp);
    apply_view_state(c, g);
    prof_exit();

    /* Attribute this draw to the rectangle it rendered under, and note
     * whether it is a perspective (world) draw or an orthographic (UI) one.
     * This is what separates "the world was never drawn" from "the world was
     * drawn into a rectangle nothing presents". */
    if (s_vp_cur >= 0 && (unsigned)s_vp_cur < g_vp_census_n) {
        g_vp_census[s_vp_cur].draws++;
        if (!g->xf.projection_orthographic)
            g_vp_census[s_vp_cur].draws_persp++;
    }

    prof_enter(PH_TEX);                 /* PHASE PROFILE: bind + any decode */
    {
        /* Bind every texture the generated TEV program samples. */
        unsigned stages = bp_tev_stage_count(&g->bp), st;
        for (st = 0; st < stages && st < BP_MAX_TEV_STAGES; st++) {
            const BPTevStage *ts = &g->bp.tev[st];
            if (ts->tex_enable)
                texcache_bind(c, ts->tex_map & 7u,
                              &g->bp.tex[ts->tex_map & 7u], &g->bp);
        }
    }
    prof_exit();

    base_vertex = s_arena_used;
    out  = &s_arena[base_vertex];
    addr = data_addr;
    {   /* SPU fast-path bookkeeping (harmless on host builds) */
    }
    u32 spu_draw_io_off = 0;
    int spu_draw_active = 0;

    u32 posmtx_idx = (g->xf.mem[0x1018] & 0x3Fu);   /* global default */
    /* Texture-coordinate matrix indices. MATRIXINDEX_A carries the position
     * matrix and coordinates 0..3 in six-bit fields; MATRIXINDEX_B carries
     * 4..7. Reading only A -- which is what this did while the vertex carried
     * two coordinates -- silently gives coordinates 4..7 the position matrix. */
    u32 texmtx_idx[GX_NUM_TEXCOORD];
    {
        unsigned k;
        for (k = 0; k < 4; k++)
            texmtx_idx[k] = (g->xf.mem[0x1018] >> (6 + k * 6)) & 0x3Fu;
        for (k = 4; k < GX_NUM_TEXCOORD; k++)
            texmtx_idx[k] = (g->xf.mem[0x1019] >> ((k - 4) * 6)) & 0x3Fu;
    }

#ifdef __PS3__
    /* --- SPU fast path -------------------------------------------------
     * If the SPU pipeline is up and the draw's format is expressible, hand
     * the whole vertex decode to the SPU and skip the PPU loop entirely.
     * The RSX then fetches this draw's vertices from the SPU's main-memory
     * arena (GCM_LOCATION_CELL) instead of the local-memory arena. */
    {
        extern int  spu_vtx_active(void);
        extern u8  *spu_vtx_reserve(u32 cnt, u32 *io_off);
        extern int  spu_vtx_submit(const SpuVtxJob *j);
        extern u64  g_spu_fallbacks;
        extern u64  g_spu_too_big;
        /* A draw with more vertices than one job can carry never even reaches
         * the recipe builder, so it was invisible in the fallback count while
         * still costing a full PPU decode. Count it separately. */
        if (spu_vtx_active() && vertex_count > SPU_VTX_MAX_VERTS)
            g_spu_too_big++;
        if (spu_vtx_active() && vertex_count <= SPU_VTX_MAX_VERTS) {
            SpuVtxJob job; u32 io_off;
            u8 *dst = spu_vtx_reserve(vertex_count, &io_off);
            int jb;
            prof_enter(PH_SPUJOB);
            jb = dst && vtx_build_spu_job(&g->parser.cp, vat, data_addr,
                                          vertex_count, (u64)(uintptr_t)dst,
                                          &job) == 0;
            prof_exit();
            if (jb) {
                /* First vertex's matrix indices: the PPU still needs them for
                 * the constant uploads; they are the first byte(s). */
                u32 mp = data_addr;
                if (g->parser.cp.vcd_lo & 1u) {   /* VCD_POS_MAT_IDX */
                    posmtx_idx = mem_read8(mp) & 0x3Fu;
                    mp += 1;
                }
                {   unsigned k9;
                    for (k9 = 0; k9 < GX_NUM_TEXCOORD; k9++)
                        if ((g->parser.cp.vcd_lo >> (1 + k9)) & 1u) {
                            texmtx_idx[k9] = mem_read8(mp) & 0x3Fu;
                            mp += 1;
                        }
                }
                spu_vtx_submit(&job);
                spu_draw_io_off = io_off;
                spu_draw_active = 1;
                goto decode_done;
            }
            g_spu_fallbacks++;
        }
    }
#endif
    prof_enter(PH_VTX);                 /* PHASE PROFILE: decode */
    for (i = 0; i < vertex_count; i++) {
        VtxAttributes a;
        u32 consumed = vtx_decode(&g->parser.cp, vat, addr, &a);

        /* The decoder and gx_vertex_size compute the stride independently. If
         * they disagree the stream is being walked wrongly, and continuing
         * would render garbage from misaligned data -- better to stop the draw
         * than to draw nonsense. */
        if (consumed != vertex_size) {
            LOG_WARN(LOG_VIDEO,
                     "gx_render: vertex stride %u != expected %u; draw dropped",
                     (unsigned)consumed, (unsigned)vertex_size);
            prof_exit();                /* PHASE PROFILE: keep the stack even */
            return;
        }
        addr += consumed;

        if (a.position_count == 0) {
            g_gx_render.skipped_no_pos++;
            prof_exit();                /* PHASE PROFILE: keep the stack even */
            return;
        }

        /* Per-vertex matrix selection: honour the first vertex's index for
         * the draw. (Distinct indices within one draw would need a matrix
         * palette in the vertex program; menu UI never does that.) */
        if (i == 0 && a.has_pos_matrix_index) {
            unsigned k;
            posmtx_idx = a.pos_matrix_index & 0x3Fu;
            for (k = 0; k < GX_NUM_TEXCOORD; k++)
                texmtx_idx[k] = a.tex_matrix_index[k] & 0x3Fu;
        }

        out[i].pos[0] = a.position[0];
        out[i].pos[1] = a.position[1];
        out[i].pos[2] = (a.position_count >= 3) ? a.position[2] : 0.0f;

        /* The normal, when the vertex has one. A lit draw whose vertices do
         * not carry a normal is not an error -- the transform unit lights it
         * against whatever the normal register holds -- but a zero normal
         * would make every diffuse term zero and the object black, so the
         * fallback points straight at the eye. */
        if (a.normal_count >= 3) {
            out[i].nrm[0] = a.normal[0];
            out[i].nrm[1] = a.normal[1];
            out[i].nrm[2] = a.normal[2];
        } else {
            out[i].nrm[0] = 0.0f;
            out[i].nrm[1] = 0.0f;
            out[i].nrm[2] = 1.0f;
        }

        if (a.color_present[0]) {
            out[i].col[0] = a.color[0][0];
            out[i].col[1] = a.color[0][1];
            out[i].col[2] = a.color[0][2];
            out[i].col[3] = a.color[0][3];
        } else {
            out[i].col[0] = out[i].col[1] = out[i].col[2] = out[i].col[3] = 1.0f;
        }

        for (t = 0; t < RENDER_VERTEX_TEXCOORDS; t++) {
            if (a.texcoord_count[t] >= 1) {
                out[i].tex[t][0] = a.texcoord[t][0];
                out[i].tex[t][1] = (a.texcoord_count[t] >= 2)
                                 ? a.texcoord[t][1] : 0.0f;
            } else {
                out[i].tex[t][0] = out[i].tex[t][1] = 0.0f;
            }
            out[i].tex[t][2] = 1.0f;    /* texgen's implicit third input */
        }
    }

    prof_exit();                        /* PHASE PROFILE: end decode */

#ifdef __PS3__
decode_done:
#endif
    if (!spu_draw_active)
        s_arena_used += vertex_count;
    g_gx_render.vertices += vertex_count;

    prof_enter(PH_CMD);                 /* PHASE PROFILE */
    rsx_bind_programs(&sh->vp, &sh->fp);
    prof_exit();

    /* PHASE PROFILE: transform maths.  The three rsx_vp_constants uploads in
     * here are twelve command words and are counted with it rather than with
     * PH_CMD -- they are the transform's output, not general state. */
    prof_enter(PH_VTX);
    /* The projection is uploaded as the 4x4 matrix the generated program
     * reads. GX stores six parameters rather than a matrix -- the same
     * frustum/ortho values the hardware expands -- so it is expanded here,
     * once per draw, into the fixed constant slots xf_program.h defines. */
    {
        /* The slot's name has always said "combined projection x position";
         * this is the multiply it promised. Skipping the position (modelview)
         * matrix worked for bring-up scenes whose vertices were authored in
         * viewport space -- and made every real scene transform to nowhere:
         * MKWii's menus put vertices at +-750 in model space, and projection
         * alone maps those far off screen. Black screen, healthy counters.
         *
         * The position matrix is 3 rows of 4 in XF memory, at the index the
         * draw selected (per-vertex attribute, or the MATRIXINDEX_A register);
         * the fourth row is implicit {0,0,0,1}. */
        /* 16-byte aligned: the combine below is done with VMX, and vec_ld
             * ignores the low four address bits. */
        f32 P[16] __attribute__((aligned(16)));
        f32 M[16] __attribute__((aligned(16)));
        f32 C[16] __attribute__((aligned(16)));
        const u32 *raw = &g->xf.mem[posmtx_idx * 4u];
        unsigned r2, c2, k2;
        union { u32 u; f32 f; } cv;

        /* Matrix-upload cache: the combined matrix, the texgen matrices and
         * the pos/normal pair are pure functions of XF memory (generation-
         * counted), the selected indices and the viewport depth fold inputs.
         * At ~11k draws/frame most consecutive draws share all of it; the
         * constant uploads (100-200 bytes of inline transfer each) were a
         * measurable slice of the frame. */
        {
            extern u32 g_xf_generation;
            static u32 lk[8]; static int lk_valid;
            u32 k2v[8];
            k2v[0]=g_xf_generation ^ (s_const_epoch << 16); k2v[1]=posmtx_idx;
            k2v[2]=texmtx_idx[0] | (texmtx_idx[1]<<8) |
                   (texmtx_idx[2]<<16) | (texmtx_idx[3]<<24);
            k2v[3]=texmtx_idx[4] | (texmtx_idx[5]<<8) |
                   (texmtx_idx[6]<<16) | (texmtx_idx[7]<<24);
            k2v[4]=sh->xinfo.texgens | ((u32)sh->xinfo.uses_posmtx<<8);
            k2v[5]=g->xf.mem[0x101A]; /* viewport z scale raw */
            k2v[6]=g->xf.mem[0x101D]; /* viewport z offset raw */
            k2v[7]=0;
            if (lk_valid && memcmp(k2v, lk, sizeof lk) == 0)
                goto matrices_done;
            memcpy(lk, k2v, sizeof lk); lk_valid = 1;
        }

        gx_projection_matrix(&g->xf, P);
        for (r2 = 0; r2 < 3; r2++)
            for (c2 = 0; c2 < 4; c2++) {
                cv.u = raw[r2 * 4 + c2];
                M[r2 * 4 + c2] = cv.f;
            }
        M[12] = 0.0f; M[13] = 0.0f; M[14] = 0.0f; M[15] = 1.0f;

        /* C = P x M on the vector unit.
         *
         * The PPE has VMX and this port had never used it for anything but
         * alignment. A row-major 4x4 combine is the case it is built for: each
         * result row is P[r][0]*M0 + P[r][1]*M1 + P[r][2]*M2 + P[r][3]*M3, so
         * four splat-and-multiply-add steps replace sixteen scalar
         * multiply-adds, and the whole 64-operation combine becomes 16 vector
         * operations. It runs for every draw that changes a matrix, on the
         * side of the machine that is the bottleneck.
         *
         * vec_madd is a fused multiply-add, so the rounding differs very
         * slightly from the separate multiply and add it replaces. That is
         * safe here: these are host render-transform values assembled for the
         * RSX, never guest-visible state, and the guest cannot observe them. */
        {
            vector float m0 = vec_ld(0,  M);
            vector float m1 = vec_ld(16, M);
            vector float m2 = vec_ld(32, M);
            vector float m3 = vec_ld(48, M);
            vector float zero = (vector float){0.0f, 0.0f, 0.0f, 0.0f};
            for (r2 = 0; r2 < 4; r2++) {
                vector float p = vec_ld((int)(r2 * 16), P);
                vector float acc;
                acc = vec_madd(vec_splat(p, 0), m0, zero);
                acc = vec_madd(vec_splat(p, 1), m1, acc);
                acc = vec_madd(vec_splat(p, 2), m2, acc);
                acc = vec_madd(vec_splat(p, 3), m3, acc);
                vec_st(acc, (int)(r2 * 16), C);
            }
            (void)c2; (void)k2;
        }
        rsx_vp_constants(XF_CONST_PROJ, 4, C);

        /* The combined projection x position matrix actually handed to the
         * vertex program, for the first few perspective draws. Everything up
         * to here has been verified off-console (geometry submitted, programs
         * generated, viewport full-screen, culling ruled out), so if the world
         * is invisible the transform is the last thing that can be wrong -- and
         * only hardware runs it. Logged as scaled integers: LOG has no %f. */
        if (!g->xf.projection_orthographic && s_cmat_logged < 4) {
            s_cmat_logged++;
            LOG_INFO(LOG_CORE,
                     "CMAT[%u] pm=%u r0=%d,%d,%d,%d r1=%d,%d,%d,%d",
                     s_cmat_logged, posmtx_idx,
                     (int)(C[0]*1000.f),  (int)(C[1]*1000.f),
                     (int)(C[2]*1000.f),  (int)(C[3]*1000.f),
                     (int)(C[4]*1000.f),  (int)(C[5]*1000.f),
                     (int)(C[6]*1000.f),  (int)(C[7]*1000.f));
            LOG_INFO(LOG_CORE,
                     "CMAT[%u] r2=%d,%d,%d,%d r3=%d,%d,%d,%d",
                     s_cmat_logged,
                     (int)(C[8]*1000.f),  (int)(C[9]*1000.f),
                     (int)(C[10]*1000.f), (int)(C[11]*1000.f),
                     (int)(C[12]*1000.f), (int)(C[13]*1000.f),
                     (int)(C[14]*1000.f), (int)(C[15]*1000.f));
            LOG_INFO(LOG_CORE,
                     "CMAT[%u] posmtx r0=%d,%d,%d,%d",
                     s_cmat_logged,
                     (int)(M[0]*1000.f), (int)(M[1]*1000.f),
                     (int)(M[2]*1000.f), (int)(M[3]*1000.f));
        }

        /* Texture-coordinate matrices: same story as the position matrix --
         * the program has always multiplied by these constant slots, and
         * nothing ever wrote them. An all-zero matrix maps every UV to the
         * atlas corner texel, which painted the whole UI flat white (or flat
         * black while the TEV colours were also zero).
         *
         * Three rows, not two. A texgen configured for STQ divides by the
         * third row, and 172 of the 612 texgens on the disc are -- with two
         * rows uploaded and the third left as an identity, every one of them
         * divides by 1 and the projection quietly does not happen.
         *
         * And one matrix per *texgen the program has*, not a fixed two. GX
         * allows eight; the vertex carries four texture coordinates, but a
         * texgen whose source row is the normal or the geometry needs its
         * matrix regardless of how many coordinates the vertex has. Driving
         * the loop from the generated program's texgen count also means a
         * draw with one coordinate uploads one matrix rather than two. */
        for (r2 = 0; r2 < sh->xinfo.texgens && r2 < GX_NUM_TEXCOORD; r2++) {
            f32 T[16];
            const u32 *traw = &g->xf.mem[texmtx_idx[r2] * 4u];
            unsigned rr, cc4;
            for (rr = 0; rr < 3; rr++)
                for (cc4 = 0; cc4 < 4; cc4++) {
                    cv.u = traw[rr * 4 + cc4];
                    T[rr * 4 + cc4] = cv.f;
                }
            T[12] = 0; T[13] = 0; T[14] = 0; T[15] = 1;
            rsx_vp_constants(XF_CONST_TEXMTX(r2), 4, T);
        }

        /* The model-view matrix on its own, and the normal matrix, for
         * lighting and for a texgen whose source row is the geometry or the
         * normal. Uploaded only when the generated program reads them, and
         * unpacked by xf_program.c rather than here: every one of these is a
         * byte order or a memory layout, and xf_program.c is the half of the
         * pipeline that a test can link. */
        if (sh->xinfo.uses_posmtx) {
            f32 N[12];
            rsx_vp_constants(XF_CONST_POSMTX, 3, M);
            xf_normal_matrix(&g->xf, posmtx_idx, N);
            rsx_vp_constants(XF_CONST_NRMMTX, 3, N);
        }
    matrices_done: ;

        /* Lights, material and the post-transform matrices all come out of XF
         * memory, and g_xf_generation already tracks every write to it. They
         * were being re-uploaded on EVERY draw regardless -- up to eight
         * lights at five rows each, plus material and ambient -- because they
         * sit past the matrix cache's `goto`, whose key includes the position
         * and texture matrix indices and therefore misses constantly while the
         * lighting has not changed at all.
         *
         * Keyed on the XF generation and on which of them the generated
         * program actually reads, so a draw that changes only its object
         * matrix no longer re-sends the scene's lighting. */
        {
            extern u32 g_xf_generation;
            static u32 lk3[4]; static int lk3_valid;
            u32 k3[4];
            k3[0] = g_xf_generation ^ (s_const_epoch << 16);
            k3[1] = sh->xinfo.lights_used;
            k3[2] = sh->xinfo.uses_channels;
            k3[3] = sh->xinfo.post_used;
            if (lk3_valid && memcmp(k3, lk3, sizeof lk3) == 0)
                goto lights_done;
            memcpy(lk3, k3, sizeof lk3); lk3_valid = 1;
        }

        /* Post-transform ("dual") matrices, for the texgens that select one.
         * Three rows of four out of the post-matrix memory at 0x0500. */
        if (sh->xinfo.post_used) {
            unsigned n2;
            for (n2 = 0; n2 < RENDER_VERTEX_TEXCOORDS; n2++) {
                f32 Q[12];
                unsigned idx, rr, cc4;
                const u32 *praw;
                if (!(sh->xinfo.post_used & (1u << n2)))
                    continue;
                idx = g->xf.mem[0x1050 + n2] & 0x3Fu;
                praw = &g->xf.mem[0x0500u + idx * 4u];
                for (rr = 0; rr < 3; rr++)
                    for (cc4 = 0; cc4 < 4; cc4++) {
                        cv.u = praw[rr * 4 + cc4];
                        Q[rr * 4 + cc4] = cv.f;
                    }
                rsx_vp_constants(XF_CONST_POSTMTX(n2), 3, Q);
            }
        }

        /* Lights, for the ones the generated program actually reads. */
        if (sh->xinfo.lights_used) {
            unsigned li;
            for (li = 0; li < 8; li++) {
                f32 L[20];
                if (!(sh->xinfo.lights_used & (1u << li)))
                    continue;
                xf_light_constants(&g->xf, li, L);
                rsx_vp_constants(XF_CONST_LIGHT(li), 5, L);
            }
        }
        if (sh->xinfo.uses_channels) {
            f32 MC[8], AC[8];
            xf_material_constants(&g->xf, MC, AC);
            rsx_vp_constants(XF_CONST_MATERIAL(0), 2, MC);
            rsx_vp_constants(XF_CONST_AMBIENT(0), 2, AC);
        }

    lights_done: ;

        /* The small-numbers register the generated code reaches for whenever
         * it needs a plain 0 or 1. It is four literal constants and it was
         * being re-sent on every draw; it cannot change, so it goes once. */
        {
            static u32 one_epoch;
            if (one_epoch != s_const_epoch + 1u) {
                f32 K[4];
                K[0] = 0.0f; K[1] = 1.0f; K[2] = 0.5f; K[3] = 2.0f;
                rsx_vp_constants(XF_CONST_ONE, 1, K);
                one_epoch = s_const_epoch + 1u;
            }
        }
        if (sh->xinfo.fog) {
            f32 F[4];
            xf_fog_constants(&g->bp, F);
            rsx_vp_constants(XF_CONST_FOG, 1, F);
        }
    }

    prof_exit();                        /* PHASE PROFILE: end transform */

#ifdef __PS3__
    /* NO join here any more.
     *
     * The RSX must not fetch vertices the SPU has not finished writing -- but
     * "has not finished" only matters at the moment the RSX actually starts
     * reading, and that is the flush, not the point where we write the draw
     * method into the command buffer. Joining per draw made the PPU wait on
     * SPU latency 94.9% of the time (~1130 spin iterations a draw) for an
     * ordering guarantee it did not need yet.
     *
     * The fence now lives at every point where the RSX can begin consuming:
     * every rsxFlushBuffer (see rsx_flush_fenced in rsx_video.c), the ring-wrap
     * callback that flushes when the command buffer fills mid-frame, the
     * WaitForIdle before an EFB blit, and the start of the next frame before
     * the vertex arena is reused. Between those the SPU runs freely, up to the
     * ring's 64 jobs ahead of the PPU.
     *
     * Still checked: a dead SPU. Its data will never land, so the draw would
     * read stale arena contents. */
    if (spu_draw_active) {
        extern int spu_vtx_active(void);
        extern int g_spu_fence_per_draw;
        extern int rsx_ring_low(void);
        if (g_spu_fence_per_draw) {
            extern void spu_vtx_join(void);
            /* Opt-in only (wiicompiled-fenceperdraw.txt). The per-draw join is
             * redundant, and it was expensive: measured in-race it ran once
             * per draw -- 7,059 joins a frame, 93% of them actually blocking,
             * ~600 spin iterations each, 10.3% of the frame asleep -- while
             * the SPU sat idle 71 us for every 3.45 us of work, because the
             * PPE dispatched a job and immediately waited on it.
             *
             * It is redundant because the RSX cannot observe a command until
             * that command is FLUSHED, and every flush goes through
             * rsx_flush_fenced(), which joins the SPUs first. The vertex arena
             * is reset per frame and drops on overflow rather than wrapping,
             * so nothing is overwritten mid-frame either. The flush fence is
             * therefore the whole guarantee, and this only re-proved it.
             *
             * The old condition also tested rsx_ring_low(), which turned out
             * to be degenerate: 4,209,552 of 4,209,552 tests fired, minimum
             * headroom 0 KiB, because end-current is not the free space in
             * this context. That is what kept the join firing every draw even
             * after the per-draw flag was cleared. */
            spu_vtx_join();
        }
        if (!spu_vtx_active())
            return;
    }
#endif
    prof_enter(PH_CMD);                 /* PHASE PROFILE: binds + the draw */
    {
        u32 off = spu_draw_active ? spu_draw_io_off
                : s_arena_offset + base_vertex * sizeof(RenderVertex);
        u8  loc = spu_draw_active ? GCM_LOCATION_CELL : GCM_LOCATION_RSX;
        unsigned n3;
        /* Offsets taken from the struct rather than written out, because the
         * layout has changed once already and a hand-counted offset that is
         * twelve bytes stale reads the next field as a position. */
        rsxBindVertexArrayAttrib(c, ATTR_POS, 0,
                                 off + (u32)offsetof(RenderVertex, pos),
                                 sizeof(RenderVertex), 3,
                                 GCM_VERTEX_DATA_TYPE_F32, loc);
        rsxBindVertexArrayAttrib(c, ATTR_NRM, 0,
                                 off + (u32)offsetof(RenderVertex, nrm),
                                 sizeof(RenderVertex), 3,
                                 GCM_VERTEX_DATA_TYPE_F32, loc);
        rsxBindVertexArrayAttrib(c, ATTR_COL, 0,
                                 off + (u32)offsetof(RenderVertex, col),
                                 sizeof(RenderVertex), 4,
                                 GCM_VERTEX_DATA_TYPE_F32, loc);
        for (n3 = 0; n3 < RENDER_VERTEX_TEXCOORDS; n3++)
            rsxBindVertexArrayAttrib(c, ATTR_TEX(n3), 0,
                                     off + (u32)offsetof(RenderVertex, tex) +
                                     (u32)(n3 * sizeof ((RenderVertex *)0)->tex[0]),
                                     sizeof(RenderVertex), 3,
                                     GCM_VERTEX_DATA_TYPE_F32, loc);

        rsxDrawVertexArray(c, rsx_primitive(prim), 0, vertex_count);
    }
    prof_exit();                        /* PHASE PROFILE */
}

/* The state tracker already owns the parser and applies every CP/XF/BP write,
 * so the backend only has to render. That is the whole point of the split: the
 * front end is useful (and testable) with no backend attached at all. */
static void (*s_frame_fn)(void *ctx);
static void  *s_frame_ctx;

void gx_render_set_frame_handler(void (*fn)(void *ctx), void *ctx)
{
    s_frame_fn  = fn;
    s_frame_ctx = ctx;
}

/* The EFB copy is the only moment pixels leave the embedded framebuffer, and
 * it has two entirely different jobs depending on one bit.
 *
 * to_xfb: the title has finished a frame and the video interface is about to
 * scan it out. Here that maps to a flip -- present what the RSX drew and start
 * the next frame. The copy's rectangle is also the best available statement of
 * how big the title's EFB is (Mario Kart Wii: 608x456), which is what every
 * EFB-to-screen conversion in this file is scaled by.
 *
 * Everything else: the title has rendered something into a corner of the EFB
 * and wants it back as a texture. That is resolved into a surface of its own
 * and registered under the destination address, so the next draw that samples
 * that address gets the pixels rather than a decode of untouched memory. */
GxCpCensus g_cp_census[GX_CP_CENSUS_MAX];
unsigned   g_cp_census_n;

static void on_efb_copy(void *ctx, const struct GXState *state,
                        const BPCopy *copy)
{
    (void)ctx; (void)state;

    {   /* In-race MKWii presents with the to_xfb BIT CLEAR: the copy is a
         * plain EFB copy whose DESTINATION is the framebuffer the VI is
         * scanning. Menus set bit14; races do not -- flips froze the moment a
         * race started (counter pinned at 3840 while the game ran on). Treat
         * "destination == the VI's current XFB base" as a present regardless
         * of the bit. */
        extern u32 vi_current_xfb(void);
        static u32 s_vi_bases[2];
        u32 vixfb = vi_current_xfb() & 0x1FFFFFFFu;
        u32 dst = copy->dest_addr & 0x1FFFFFFFu;
        if (vixfb && vixfb != s_vi_bases[0]) {
            /* Double-buffered XFBs: the copy lands in the buffer the VI is
             * NOT scanning, so remember the last two bases and match either. */
            s_vi_bases[1] = s_vi_bases[0];
            s_vi_bases[0] = vixfb;
        }
        {   /* Diagnostic: name the copies the presenter sees, so the race's
             * actual present mechanism stops being guesswork. */
            static unsigned n_cp;
            if (n_cp < 24 || (n_cp & 0x1FFu) == 0)
                LOG_INFO(LOG_CORE, "COPY dst=%08x %ux%u xfbbit=%u clear=%u "
                         "vi=%08x/%08x", dst, copy->width, copy->height,
                         copy->to_xfb, copy->clear,
                         s_vi_bases[0], s_vi_bases[1]);
            n_cp++;
        }
        {   /* Same census for copies: size, to_xfb bit, and whether the
             * destination matched a VI base (i.e. whether we treated it as a
             * present). A sampled log cannot show the distribution. */
            unsigned i;
            int matched = (vixfb && (dst == s_vi_bases[0] ||
                                     dst == s_vi_bases[1])) ? 1 : 0;
            for (i = 0; i < g_cp_census_n; i++)
                if (g_cp_census[i].w == copy->width &&
                    g_cp_census[i].h == copy->height &&
                    g_cp_census[i].to_xfb == copy->to_xfb &&
                    g_cp_census[i].matched == matched) break;
            if (i == g_cp_census_n && g_cp_census_n < GX_CP_CENSUS_MAX) {
                g_cp_census_n++;
                g_cp_census[i].w = (u16)copy->width;
                g_cp_census[i].h = (u16)copy->height;
                g_cp_census[i].to_xfb = (u8)copy->to_xfb;
                g_cp_census[i].matched = (u8)matched;
                g_cp_census[i].uses = 0;
            }
            if (i < GX_CP_CENSUS_MAX) g_cp_census[i].uses++;
        }
        if (!copy->to_xfb && vixfb &&
            (dst == s_vi_bases[0] || dst == s_vi_bases[1])) {
            extern u64 g_addr_flips; g_addr_flips++;
            g_gx_render.efb_copies_xfb++;
            if (copy->width >= 320 && copy->width <= 1024 &&
                copy->height >= 200 && copy->height <= 1024) {
                s_efb_w = copy->width;
                s_efb_h = copy->height;
            }
            if (s_frame_fn) {
#ifdef __PS3__
                extern void spu_vtx_join(void);
                spu_vtx_join();
#endif
                rsx_efb_to_display();   /* EFB -> display buffer, then flip */
                s_frame_fn(s_frame_ctx);
            }
            return;
        }
    }
    if (copy->to_xfb) {
        /* Count it. A to_xfb copy is the title saying "this frame was RENDERED
         * by GX"; video played from a THP stream reaches the XFB by a
         * completely different route (the CPU writes decoded YUV straight into
         * it) and produces no copy at all. That distinction is what tells the
         * presenter which of the two it is looking at. */
        g_gx_render.efb_copies_xfb++;
        /* Ignore an implausible rectangle rather than rescaling the whole
         * frame from it: a half-height field copy or a stray trigger would
         * otherwise double every coordinate on screen. */
        if (copy->width >= 320 && copy->width <= 1024 &&
            copy->height >= 200 && copy->height <= 1024) {
            s_efb_w = copy->width;
            s_efb_h = copy->height;
        }
        if (s_frame_fn) {
#ifdef __PS3__
            extern void spu_vtx_join(void);
            spu_vtx_join();     /* every queued vertex must land before the
                                 * frame's commands reach the GPU */
#endif
            rsx_efb_to_display();   /* EFB -> display buffer, then flip */
            s_frame_fn(s_frame_ctx);
        }
        return;
    }

    g_gx_render.efb_copies_texture++;
    if (!(g_gx_state_mask & GX_STATE_EFB_COPY))
        return;

    prof_enter(PH_TEX);                 /* PHASE PROFILE: with the textures */
    efb_copy_resolve(copy);
    if (copy->clear && (g_gx_state_mask & GX_STATE_EFB_CLEAR))
        efb_copy_clear(copy);
    prof_exit();
}

void gx_render_bind(GXBackend *backend)
{
    memset(backend, 0, sizeof *backend);
    backend->ctx      = NULL;
    backend->draw     = on_draw;
    backend->efb_copy = on_efb_copy;
}
