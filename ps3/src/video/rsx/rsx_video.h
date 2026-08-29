/* rsx_video.h — RSX display output: video mode, framebuffers, frame lifecycle.
 *
 * The layer between the emulator and the console's actual screen. Everything
 * above it (the GX front end, the TEV/XF lowerings, the shader encoders) has
 * been verified without a GPU; this is what finally puts their results on a
 * television.
 */
#ifndef DOLPHIN_VIDEO_RSX_VIDEO_H
#define DOLPHIN_VIDEO_RSX_VIDEO_H

#include "../../common/types.h"

/* Double buffering. Two is enough: the emulator waits for flip completion
 * before drawing again, so a third buffer would only add latency. */
/* Triple buffered. With two buffers the emulator had to wait for the flip to
 * retire before drawing again, which the phase profiler measured at 35.6% of
 * every frame spent asleep in the vsync wait. A third buffer means there is
 * always one free to draw into. */
#define RSX_BUFFERS 3

typedef struct {
    int  inited;
    u32  width;
    u32  height;
    u32  pitch;                 /* bytes per scanline, 4 bytes per pixel */
    u32 *buffer[RSX_BUFFERS];   /* CPU-visible addresses                 */
    u32  offset[RSX_BUFFERS];   /* RSX-visible offsets                   */
    u32 *depth;                 /* depth buffer, CPU-visible             */
    u32  depth_offset;
    u32  depth_pitch;
    /* Optional dedicated embedded-framebuffer surface. On the real machine the
     * EFB is a separate memory the video interface never scans, so a GX copy's
     * clear cannot damage the picture already presented. Rendering straight
     * into the display buffer makes every render-to-texture clear erase
     * presented pixels; with this bound, clears hit the EFB and the present
     * copy blits EFB -> display. Null when the flag is not armed. */
    u32 *efb;
    u32  efb_offset;
    int  current;               /* buffer being drawn into               */
    int  last_queued;           /* newest completed pixels: what a
                                 * screenshot must read. (current-1)%N was
                                 * a guess that sometimes named a slot no
                                 * longer in rotation -- captures then
                                 * returned a byte-identical stale frame
                                 * across sessions and were analysed as
                                 * fresh, twice. */
    u64  frames;
    u64  flip_timeouts;         /* flips that never retired -- see rsx_frame_end */

    /* Tiled regions and Zcull (GX_STATE_RSX_TILE). All of this is decided
     * once, in rsx_video_init, and none of it can be changed afterwards: tile
     * and Zcull binds are privileged calls into the memory controller that
     * require a completely idle RSX, which only holds before the first
     * gcmSetDisplayBuffer. `tiled` and `zcull` record what was actually
     * achieved rather than what was asked for, so a partial success (tiles
     * bound, Zcull refused) is visible instead of assumed. */
    int  tiled;                 /* tile regions bound over every surface  */
    int  zcull;                 /* a Zcull region is bound over the depth */
    u32  buffer_bytes;          /* per colour buffer, incl. the tile tail */
    u32  depth_bytes;
    u32  zcull_width;           /* 64-aligned, as passed to gcmBindZcull  */
    u32  zcull_height;
} RsxVideo;

extern RsxVideo g_rsx;

/* Configure the display, create the RSX context, allocate the buffers.
 * Returns 0 on success. Safe to call twice; the second call is a no-op. */
int  rsx_video_init(void);

/* Begin a frame: point the RSX at the back buffer and set the viewport. */
void rsx_frame_begin(void);

/* Clear the back buffer. Colour is 0xAARRGGBB. */
void rsx_clear(u32 argb);

/* Finish a frame: flush the command buffer, flip, and wait for the flip to
 * retire. Blocking, because the emulator has nothing else to do with the
 * time and it keeps the two buffers strictly alternating. */
void rsx_frame_end(void);

/* Release the buffers and the context. */
void rsx_video_shutdown(void);

/* Is a Zcull region bound AND still switched on in the feature mask?
 *
 * Anything that clears depth has to ask, because a Zcull region only becomes
 * *valid* -- i.e. actually able to reject anything -- by riding along with a
 * hardware fast clear, and it goes back to doing nothing the moment its
 * control registers are rewritten. Callers outside rsx_video.c own their own
 * depth clears (gx_render.c clears once per title frame), so they have to
 * perform the same small ritual around them. */
int  rsx_zcull_active(void);

/* Emit the Zcull programming that belongs BEFORE a depth clear: direction,
 * encoding, limits, the (disabled) stencil-cull criterion and the all-or-
 * nothing stencil mask a hardware fast clear requires. These writes invalidate
 * the Zcull region, which is precisely why they go before the clear that
 * revalidates it.
 *
 * rsx_frame_begin already calls this after rsxSetSurface. It is exported so
 * that a per-frame state block re-established after a flip has one place to
 * call -- and so that such a block never re-emits the raw ZCULL_CONTROL0 /
 * ZCULL_CONTROL1 / SCULL_CONTROL triple (method 0x1ea4) itself, which after
 * the clear would kill the region for the rest of the frame. Safe to call when
 * Zcull is inactive; it emits nothing. */
void rsx_zcull_before_clear(void);

/* Emit the Zcull programming that belongs immediately AFTER a depth clear:
 * clear the Zcull surface so its contents agree with the depth values just
 * written, then switch the unit on. Safe to call when Zcull is inactive; it
 * emits nothing. */
void rsx_zcull_after_depth_clear(void);

/* Diagnostics, used by the console self-test.
 *
 * rsx_probe_alive: does the RSX execute commands at all? Writes a label from
 * the GPU's back end and waits (bounded) for it to appear in memory. This is
 * independent of the display engine, so it separates "the GPU is not running
 * our command buffer" from "the GPU runs but flips do not retire".
 * Returns 1 if the label landed, 0 on timeout.
 *
 * rsx_fill_cpu: fill a framebuffer from the CPU, with no GPU involvement, so
 * the display path can be tested on its own. */
int  rsx_probe_alive(void);

/* Block until the GPU has retired everything submitted so far. Bounded.
 * Returns 1 if it went idle, 0 on timeout. */
int  rsx_wait_idle(void);

/* Count sampled pixels in `buffer` whose colour differs from `argb`, and
 * optionally return the centre pixel. This is how a draw is verified without
 * relying on someone watching the television. */
u32  rsx_count_differing(int buffer, u32 argb, u32 *sample_centre);

/* Read one pixel, in normalised screen coordinates (0..1). Sampling several
 * points is how "is anything being interpolated" gets answered without a
 * frame grabber. */
u32  rsx_sample(int buffer, float u, float v);

/* Command-stream capture: remember the ring position, then copy out everything
 * emitted since. For offline decoding against the NV40 method database. */
void rsx_cmd_mark(void);
u32  rsx_cmd_since_mark(u32 *out, u32 max_words);
void rsx_fill_cpu(int buffer, u32 argb);
int  rsx_present_cpu(int buffer);

/* CPU-side text blitter: draw status text into a framebuffer (ARGB8888),
 * then rsx_present_cpu() it. scale multiplies the 8x8 glyphs. */
int rsx_video_screenshot(const char *path, unsigned step, unsigned which);
void rsx_draw_char(u32 *fb, int pitch, int x, int y, u32 color, int scale, char c);
void rsx_draw_text_scaled(u32 *fb, int pitch, int x, int y, u32 color, int scale, const char *s);
void rsx_draw_text(u32 *fb, int pitch, int x, int y, u32 color, const char *s);

int rsx_efb_surface_wanted(void);
u32 rsx_render_target_offset(void);
void rsx_efb_to_display(void);

#endif
int rsx_xmb_menu_open(void);
