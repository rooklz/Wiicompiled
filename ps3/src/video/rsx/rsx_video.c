/* rsx_video.c — RSX display output.
 *
 * Sequence, and why it is in this order:
 *
 *   1. Ask the console what video mode it is already in and keep it. A console
 *      is attached to a television whose capabilities we cannot query
 *      usefully; the mode the dashboard is running in is known to work on this
 *      set, and forcing 1080p onto an SDTV shows nothing at all.
 *   2. Create the RSX context. Its command buffer lives in main memory mapped
 *      into the RSX's address space, which is what `rsxInit`'s io area is.
 *   3. Allocate the framebuffers from RSX-visible memory and register them
 *      with the display engine.
 *
 * The depth buffer is allocated even though the first users only clear colour:
 * a Wii title's very first draw sets a depth function, and having the surface
 * already bound avoids a special case there.
 */
#include <stdio.h>
#include <stdlib.h>
#include "rsx_video.h"
/* PHASE PROFILE: the two places the PPE goes to sleep on the GPU.  See main.c
 * and src/common/phase_prof.h. */
#include "../../common/phase_prof.h"
#include "../../common/log.h"
/* GX_STATE_RSX_TILE: whether the surfaces below are laid out in tiled regions
 * with a Zcull region over the depth buffer. */
#include "gx_features.h"

#include <string.h>
#include <malloc.h>
#include <unistd.h>

#include <rsx/rsx.h>
#include <sysutil/video.h>

RsxVideo g_rsx;

static gcmContextData *s_ctx;
/* Provided by the platform (src/platform/ps3/main.c). A weak default here
 * silently WON the link on this toolchain, so the flip wait never learned the
 * XMB menu was up and the console locked when the PS button was pressed. */
int rsx_xmb_menu_open(void);

/* ================= NON-BLOCKING PRESENTATION (BEGIN) =================
 *
 * Source: libgcm Overview ch.5 "Flips" (5.1 Flip Methods, 5.2 Flip Completion
 * Checks) and the libgcm Reference entries for cellGcmSetPrepareFlip,
 * cellGcmSetFlipImmediate, cellGcmSetFlipHandler and cellGcmSetVBlankHandler.
 * The structure below follows the standard RSX immediate-flip setup.
 *
 * WHAT THE OLD PATH DID WRONG.  cellGcmGetFlipStatus is a single global flag
 * ("Gets the flip status", libgcm Reference) -- it can say *a* flip retired,
 * never *which of our buffers* is free.  With three surfaces that forces the
 * caller to guess, and the guess this file used was to assume everything
 * outstanding retired together and to sleep in a usleep poll until the flag
 * cleared.  The phase profiler measured that sleep at 35-46% of every frame.
 *
 * WHAT REPLACES IT.  cellGcmSetFlipHandler's documented purpose is exactly
 * this problem: "the application is able to know through an interrupt when a
 * screen flip completes, and thus the display buffer, for example, can be
 * freed efficiently".  So buffer ownership is tracked in plain PPU memory,
 * written by the flip interrupt and read by the frame loop as an ordinary
 * cached load.  Nothing in the frame loop polls the GPU, and nothing reads
 * RSX local memory (RSX_SOL 1.2.14 / RSX_Tips 2.1.3: Cell reads of local
 * memory run at ~15.6 MB/s and block the accesses the RSX driver itself
 * needs).  The one local-memory load that remains is a single word inside the
 * VBlank interrupt, 60 times a second, which is the standard approach.
 *
 * THE FLOW, per frame:
 *   rsx_frame_end   PPU: gcmSetPrepareFlip(cur) -> queue id.  Then
 *                   rsxSetWriteBackendLabel(PREPARED, (cur<<8)|qid), which the
 *                   RSX writes only once its render back end has flushed this
 *                   frame's pixels -- so the label doubles as "buffer cur is
 *                   finished".  Flush.  No wait of any kind.
 *   VBlank IRQ      reads that label; if it names a buffer other than the one
 *                   on screen, issues gcmSetFlipImmediate(qid).  The flip
 *                   decision lives in the interrupt, not in the frame loop.
 *   Flip IRQ        marks every buffer from the one that *was* on screen up to
 *                   the one just flipped to as free.  Frames the display
 *                   skipped are released here rather than never.
 *
 * WHY THIS IS NOT QUANTISED TO 60/k.  The old code spent a whole scanout
 * interval asleep per frame, so a 20 ms frame became 33.3 ms of wall clock and
 * the emulator ran at 30 Hz.  Here the loop never waits while a buffer is
 * free, so distinct frames arrive at the emulator's own rate and each is shown
 * at the next V after it completes: 20 ms frames present at ~48 Hz, not 30.
 * The only remaining cap is the one Application Requirements (Graphics),
 * "Note on Flips" says cannot be removed -- the system enforces a minimum
 * interval between flips from the TV refresh rate.
 *
 * XMB.  The in-game menu composites over our output and holds the flip, so no
 * flip retires and no buffer is freed.  Every wait below is bounded and every
 * one of them is skipped outright while rsx_xmb_menu_open() is true, so the
 * worst case is a dropped present rather than a console that needs a power
 * cycle.
 */

/* Label indices.  65 / 66.. mirror flip_immediate/main.cpp's 0x41 / 0x42.., and
 * stay clear of 255, which rsx_probe_alive and rsx_wait_idle already own. */
#define RSX_LABEL_PREPARED   65
#define RSX_LABEL_BUFSTATUS  66     /* 66 .. 66 + RSX_BUFFERS - 1 (mirror only) */

/* Written by the interrupt handlers, read by the frame loop.  Ordinary main
 * memory: no syscall, no GPU access, no lock. */
static volatile u32 s_buf_on_display;              /* index being scanned out  */
static volatile u32 s_buf_flipped;                 /* index of flip in flight  */
static volatile u32 s_flip_in_flight;
static volatile u32 s_buf_busy[RSX_BUFFERS];       /* 1 = queued or on screen  */
static volatile u64 s_flip_irqs;
static volatile u64 s_vblank_irqs;
static volatile u32 *s_label_prepared;             /* cached once at init      */
static u64 s_present_waits;                        /* frames that had to wait  */
static u64 s_present_drops;                        /* flip queue full / no buf */
static int s_handlers_up;

static void rsx_flip_handler(const u32 head)
{
    u32 v = s_buf_flipped, i;
    (void)head;
    /* Retire the buffer that was on screen and any the display skipped over.
     * flip_immediate/main.cpp:159-166 does exactly this walk, and it is what
     * lets a run-ahead frame be dropped cleanly instead of stalling. */
    for (i = s_buf_on_display; i != v; i = (i + 1u) % RSX_BUFFERS)
        s_buf_busy[i] = 0u;
    s_buf_on_display = v;
    s_buf_busy[v]    = 1u;          /* the new one is now being scanned out */
    s_flip_in_flight = 0u;
    s_flip_irqs++;
}

static void rsx_vblank_handler(const u32 head)
{
    u32 data, buf, qid;
    (void)head;
    s_vblank_irqs++;

    if (!s_label_prepared)   return;
    if (s_flip_in_flight)    return;    /* "when this function is called twice
                                         * successively, the second call may be
                                         * ignored" -- SetFlipImmediate ref */
    data = *s_label_prepared;
    buf  = (data >> 8) & 0xffu;
    qid  =  data       & 0x07u;

    if (buf >= (u32)RSX_BUFFERS)   return;   /* label not published yet */
    if (buf == s_buf_on_display)   return;   /* nothing newer is ready  */

    if (gcmSetFlipImmediate((u8)qid) != 0)
        return;                              /* stale id: try again next V */
    s_buf_flipped    = buf;
    s_flip_in_flight = 1u;
}

/* Give up on the flips that are not coming: release every buffer except the one
 * genuinely on screen and hand back the first of them.  Used only when the
 * ownership model has stopped being fed -- the XMB holding the flip, or a flip
 * that never retired.  Without this the busy flags would stay set for ever and
 * every subsequent frame would pay the full timeout. */
static int rsx_release_stuck(int start)
{
    int i, pick = -1;
    for (i = 0; i < RSX_BUFFERS; i++) {
        int b = (start + i) % RSX_BUFFERS;
        if ((u32)b == s_buf_on_display)
            continue;
        s_buf_busy[b] = 0u;
        if (pick < 0) pick = b;
    }
    return pick >= 0 ? pick : start;
}

/* Pick the next buffer to draw into.  Returns immediately whenever any buffer
 * is free, which is the common case; only a loop that has run ahead of the
 * display ever sleeps, and that sleep is the display's pacing, not a poll. */
static int rsx_acquire_buffer(void)
{
    int  start = (g_rsx.current + 1) % RSX_BUFFERS;
    unsigned waits = 0;
    /* ~250 ms.  A frame is 16.7 ms, so anything approaching this bound means
     * the flip is never coming, not that we are early.  Bounded because on a
     * console an unbounded wait is indistinguishable from a crash except that
     * it also requires power-cycling the machine. */
    const unsigned k_max_waits = 1250;

    for (;;) {
        int i;
        for (i = 0; i < RSX_BUFFERS; i++) {
            int b = (start + i) % RSX_BUFFERS;
            if (!s_buf_busy[b])
                return b;
        }

        /* The XMB owns the flip while its menu is up: no flip will retire and
         * no buffer will come free.  Never wait here. */
        if (rsx_xmb_menu_open()) {
            s_present_drops++;
            return rsx_release_stuck(start);
        }
        if (waits >= k_max_waits) {
            g_rsx.flip_timeouts++;
            /* If the flip interrupt has never fired even once, the interrupt
             * path is not working on this machine and no amount of waiting
             * will help.  Retreat permanently to the plain command-stream flip
             * rather than leave a black screen: a slower picture beats none,
             * and this is the only failure mode of the new path that a console
             * could not otherwise recover from. */
            if (s_flip_irqs == 0 && g_rsx.flip_timeouts >= 3) {
                s_handlers_up = 0;
                LOG_ERROR(LOG_VIDEO,
                          "rsx: flip interrupt never fired (%llu vblank irqs); "
                          "falling back to gcmSetFlip",
                          (unsigned long long)s_vblank_irqs);
            }
            return rsx_release_stuck(start);
        }
        prof_enter(PH_WAITFLIP);
        usleep(200);
        prof_exit();
        waits++;
        if (waits == 1) s_present_waits++;
    }
}
/* ================== NON-BLOCKING PRESENTATION (END) ================== */

/* Other RSX modules submit through the same context; exposing it as a call
 * rather than a global keeps the ownership in one place. */
gcmContextData *rsx_context(void) { return s_ctx; }

/* ============ PER-FRAME RSX CONTEXT RESET (BEGIN) ============
 *
 * libgcm Overview ch.5, opening paragraph: "the RSX(TM) context (the rendering
 * context on RSX(TM)) is not saved and restored for flips; instead, the initial
 * settings are restored.  It is therefore necessary for the application to
 * reset the RSX(TM) context every frame."  5.3 "Resetting the RSX(TM) Context"
 * then prints the full default list, and cellGcmSetPrepareFlip's Reference
 * entry repeats it: "RSX(TM) contexts will be re-initialized when this command
 * is executed".  Overview 4 (command buffer layout) says how: the first 4 KB of
 * the default command buffer is an initialisation buffer that SetFlip /
 * SetFlipWithWaitLabel / SetPrepareFlip *Call* into.
 *
 * Consequence for this file: anything written once at rsx_video_init and never
 * again is only correct until the first flip.  The registers below are exactly
 * that -- raw NV40 methods PSL1GHT has no wrapper for, so nothing else in the
 * process re-establishes them.  They are now emitted from rsx_frame_begin as
 * well, which is 19 command words per frame.
 *
 * The one that matters is the interpolant routing at 0x1fc4/0x1fc8/0x1fd0/
 * 0x1fd4: it maps vertex-program outputs onto rasteriser interpolants, and a
 * scrambled routing lets a program write COL0 forever while the fragment stage
 * reads an unrouted constant -- position still works, because position does not
 * route through it.  That is the failure this display path spent a long time
 * chasing, and leaving it to survive a context reset by luck is not a thing to
 * rely on. */
static const struct { u16 method; u16 count; u32 value[3]; } k_ctx_init[] = {
    { 0x1450, 1, { 0x00000004, 0, 0 } },
    /* ZCULL_CONTROL0 / ZCULL_CONTROL1 / SCULL_CONTROL. Decoded: zcullformat
     * LONES with direction LESS, move-forward and push-back limits 0x100, and
     * a stencil-cull criterion of GEQUAL/0x80/0xff.
     *
     * THIS ENTRY IS ONLY SAFE WHERE IT IS. rsx_zcull_before_clear() programs
     * the same three registers through the PSL1GHT wrappers and is called at
     * the END of rsx_frame_begin, so it lands after this table and before the
     * clear -- which is the only order that works, because a write to
     * ZCULL_CONTROL0 or SCULL_CONTROL invalidates the Zcull region and only a
     * clear can make it valid again. If rsx_emit_raw_defaults() is ever moved
     * to a point after the clear, DELETE THIS ENTRY: leaving it would
     * invalidate the region every frame and Zcull would silently reject
     * nothing, with no visible symptom to notice it by. The first two values
     * already agree with what the wrappers write; the third is inert because
     * rsxSetZCullEnable asks for depth culling only. */
    { 0x1ea4, 3, { 0x00000010, 0x01000100, 0xff800006 } }, /* zcull */
    { 0x1fc4, 1, { 0x06144321, 0, 0 } },  /* interpolant routing */
    { 0x1fc8, 2, { 0xedcba987, 0x0000006f } },
    { 0x1fd0, 1, { 0x00171615, 0, 0 } },
    { 0x1fd4, 1, { 0x001b1a19, 0, 0 } },
    { 0x1ef8, 1, { 0x0020ffff, 0, 0 } },  /* transform timeout */
    { 0x1d64, 1, { 0x01d300d4, 0, 0 } },
};

static void rsx_emit_raw_defaults(void)
{
    unsigned n, j, words = 0;

    if (!s_ctx)
        return;

    for (n = 0; n < sizeof k_ctx_init / sizeof k_ctx_init[0]; n++)
        words += 1u + k_ctx_init[n].count;

    /* These are raw methods, so the ring-wrap callback PSL1GHT's wrappers run
     * for us has to be provoked by hand.  rsxSetNopCommand reserves `words`
     * words (invoking the callback if the ring is short) and fills them with
     * NOPs; rewinding `current` over them and writing the real methods into
     * the space it just guaranteed is the only way to get that guarantee from
     * outside <rsx/commands.h>.  The previous code simply skipped the block
     * when the ring was short, which at init was harmless and once per frame
     * would not have been. */
    rsxSetNopCommand(s_ctx, words);
    s_ctx->current -= words;

    for (n = 0; n < sizeof k_ctx_init / sizeof k_ctx_init[0]; n++) {
        *s_ctx->current++ = ((u32)k_ctx_init[n].count << 18) | k_ctx_init[n].method;
        for (j = 0; j < k_ctx_init[n].count; j++)
            *s_ctx->current++ = k_ctx_init[n].value[j];
    }
}
/* ============= PER-FRAME RSX CONTEXT RESET (END) ============= */

/* The command buffer and the RSX-visible main-memory window. 1 MiB of command
 * buffer is far more than a frame of Wii geometry needs; the io area has to be
 * a multiple of 1 MiB and is where rsxMemalign allocates from. */
static gcmContextCallback s_ring_cb_prev;

/* Set when the ring-wrap fence could not be installed: the draw path then
 * restores the old per-draw join, which is correct but slow. */
int g_spu_fence_per_draw;

/* See rsx_video_init: the SPUs must be caught up before PSL1GHT's wrap handler
 * lets the RSX loose on the commands already in the ring. */
static s32 rsx_ring_wrap_fenced(gcmContextData *c, u32 count)
{
#ifdef __PS3__
    {   extern void spu_vtx_join(void);
        spu_vtx_join();
    }
#endif
    return s_ring_cb_prev ? s_ring_cb_prev(c, count) : -1;
}

/* How close the command ring is to wrapping.
 *
 * The wrap flush is the one the renderer does not initiate: PSL1GHT's handler
 * fires when `current` reaches `end`, advances PUT and waits for space, and
 * from that moment the RSX is reading commands we have already written. Rather
 * than interpose on that handler (see rsx_video_init), the draw path asks this
 * before each draw and fences itself when the ring is nearly full. Same
 * guarantee, no call into someone else's function pointer. */
u64 g_ring_low_hits, g_ring_low_tests;
u32 g_ring_span_min = 0xFFFFFFFFu;

int rsx_ring_low(void)
{
    size_t left;
    if (!s_ctx) return 0;
    left = (size_t)((u8 *)s_ctx->end - (u8 *)s_ctx->current);
    g_ring_low_tests++;
    if ((u32)left < g_ring_span_min) g_ring_span_min = (u32)left;
    if (left < (64u * 1024u)) { g_ring_low_hits++; return 1; }
    return 0;
}

/* Flush, but never ahead of the SPUs.
 *
 * A flush is the moment the RSX may begin consuming commands already written
 * into the ring, and some of those commands point at vertex buffers an SPE is
 * still filling. The draw path deliberately no longer waits per draw (see
 * gx_render.c), so the ordering guarantee has to be re-established exactly
 * here instead -- at every point where the GPU can start reading.
 *
 * Costs one compare when nothing is outstanding, which is the common case. */
static void rsx_flush_fenced(gcmContextData *c)
{
#ifdef __PS3__
    {   extern void spu_vtx_join(void);
        spu_vtx_join();
    }
#endif
    rsxFlushBuffer(c);
}

/* 2 MiB.
 *
 * This was raised to 8 MiB for an unvalidated performance experiment (fewer
 * ring wraps, so the SPU fence fires less often). That experiment was never
 * measured, and the space is not free: the vertex arena competes for the same
 * memory, and an arena that fails to allocate disables the renderer outright.
 * Correct rendering outranks an unmeasured ring-size tweak, so most of it goes
 * back. The original note follows.
 *
 * WAS: 8 MiB, not 1.
 *
 * A heavy Mario Kart frame writes well over a megabyte of RSX methods, so a
 * 1 MiB ring wrapped several times per frame. Every wrap is a flush we do not
 * control -- the RSX starts reading commands already written -- so the vertex
 * fence had to be taken before nearly every draw, which is exactly the
 * per-draw wait the SPU pipeline was changed to avoid. Sized to hold a whole
 * frame, the ring wraps rarely and the fence collapses back to the handful of
 * real flush points. Carved from the 32 MiB host area, which has the room. */
#define CB_SIZE   (2 * 1024 * 1024)
#define HOST_SIZE (32 * 1024 * 1024)

/* ------------------------------------------------------------------ */
/* Tiled regions and Zcull                                              */
/*                                                                      */
/* Region indices, derived from RSX_BUFFERS rather than written out, because
 * the buffer count is a presentation decision that belongs to the flip path --
 * three today, four if the flip queue ever wants jitter slack -- and a
 * hardcoded region list would silently overlap or leave a hole the day it
 * changes.
 *
 * The memory controller has 15 tiled regions (0..14) and 8 Zcull regions
 * (0..7). Depth takes region 0 and colour takes 1..RSX_BUFFERS, so a texture
 * arena or an EFB-copy arena can still be tiled later without running out:
 * even at four buffers this is 5 of 15.
 *
 * Depth is region 0 and colour follows it because the depth region is the only
 * compressed one and the compression tag window is allocated from base 0
 * upwards -- keeping the compressed region first keeps that window trivially
 * disjoint from anything added afterwards. */
#define TILE_REGION_DEPTH    0u
#define TILE_REGION_COLOR0   (TILE_REGION_DEPTH + 1u)
#define TILE_REGIONS_USED    (TILE_REGION_COLOR0 + (u32)RSX_BUFFERS)
#define TILE_REGION_COUNT    15u
#define ZCULL_REGION         0u
#define ZCULL_REGION_COUNT   8u

/* Compile-time, because running out of regions is not something to discover on
 * a television: a bind past the last index is refused and the surface is then
 * tiled in part and linear in part, which is the classic blocky corruption. */
typedef char rsx_tile_region_budget_check[
    (TILE_REGIONS_USED <= TILE_REGION_COUNT &&
     ZCULL_REGION < ZCULL_REGION_COUNT) ? 1 : -1];

/* Bank sense. Adjacent frame-buffer tiles land in different DRAM banks, and
 * the bank field shifts that mapping per region so that the read-Z / write-Z /
 * write-colour sequence of a single fragment does not keep landing on the same
 * DRAM page in two different regions. The manual is explicit that the right
 * value is empirical and that 0 for colour with a non-zero depth is the
 * convention; 2 is what the PSL1GHT tiled sample ships, so that is the
 * starting point rather than a guess of our own. */
#define TILE_BANK_COLOR     0u
#define TILE_BANK_DEPTH     2u

/* Compression tag RAM: 0x800 units of 64 KiB, i.e. 128 MiB of local memory may
 * be compressed at once, and a region consumes base .. base + size/0x10000.
 * Only the depth region is compressed here, so it takes the window at 0. */
#define TILE_TAG_BASE_DEPTH 0u

/* Label index used by the bounded "has the GPU got here yet" waits below. The
 * tile setup needs one before the first frame exists, so it is defined here
 * rather than next to rsx_wait_idle. 255 is deliberately clear of the
 * presentation path's labels (65 and 66..66+RSX_BUFFERS-1). */
#define RSX_LABEL_INDEX 255

static u32 align_up_u32(u32 v, u32 a) { return (v + (a - 1u)) & ~(a - 1u); }

/* Bounded proof that the RSX has drained everything submitted so far.
 *
 * gcmSetTileInfo/gcmBindTile/gcmBindZcull reprogram the memory controller, and
 * the documented precondition is that the RSX is COMPLETELY idle -- pipeline
 * drained, no Cell traffic to the region, region not being scanned out.
 * Violating it gives a mosaic of tiles on screen or wedges the GPU. Two of the
 * three conditions hold for free at init (nothing else has touched these pages,
 * and gcmSetDisplayBuffer has deliberately not been called yet, so no region
 * is scanned out); this establishes the third. A back-end label is the right
 * primitive because it retires only after the render back end has flushed, and
 * the 200 us poll interval is comfortably over the 30 us minimum the hardware
 * erratum on cache-inhibited PPU loads requires. */
static int rsx_tile_wait_idle(gcmContextData *ctx)
{
    volatile u32 *label;
    static u32 seq = 0x2000;
    u32 want = ++seq;
    unsigned polls = 0;

    label = (volatile u32 *)gcmGetLabelAddress(RSX_LABEL_INDEX);
    if (!label)
        return 0;
    *label = 0;

    rsxSetWriteBackendLabel(ctx, RSX_LABEL_INDEX, want);
    rsx_flush_fenced(ctx);

    while (*label != want && polls++ < 5000)
        usleep(200);
    return *label == want;
}

int rsx_video_init(void)
{
    /* RSX is a PROCESS-level resource: rsxInit() cannot be run a second time
     * in the same process, and trying returns 0x80010004. The session
     * supervisor restarts the emulator in-process, and it called
     * rsx_video_shutdown() then rsx_video_init() around that -- so every
     * restart failed here and the emulator exited to the XMB, costing a
     * manual relaunch each time. On this console nothing can start a title
     * remotely (six webMAN endpoints all answer 200 without launching, and
     * setup.ps3 discards remote submissions), so a needless exit is expensive.
     *
     * Initialising once and treating later calls as satisfied keeps the
     * display context alive across session restarts, which is what makes one
     * manual start last indefinitely. */
    static int s_inited;
    if (s_inited) {
        LOG_INFO(LOG_VIDEO, "rsx: already initialised, reusing the context");
        return 0;
    }
    {
    videoState        state;
    videoConfiguration cfg;
    videoResolution   res;
    void             *host;
    int               i;
    s32               rc;
    /* Latched here and nowhere else: see the comment on the bit in
     * gx_features.h. Everything downstream reads g_rsx.tiled/g_rsx.zcull, so
     * a later change to the mask cannot leave the allocation and the tile
     * registers disagreeing. */
    int               want_tile = (g_gx_state_mask & GX_STATE_RSX_TILE) != 0;
    u32               cb_h, cb_size, depth_h, depth_size;

    if (g_rsx.inited)
        return 0;

    host = memalign(1024 * 1024, HOST_SIZE);
    if (!host) {
        LOG_ERROR(LOG_VIDEO, "rsx: could not reserve the %u MiB host area",
                  (unsigned)(HOST_SIZE >> 20));
        return -1;
    }

    s_ctx = NULL;
    rc = rsxInit(&s_ctx, CB_SIZE, HOST_SIZE, host);
    if (rc != 0 || !s_ctx) {
        LOG_ERROR(LOG_VIDEO, "rsx: rsxInit failed: %08x", (unsigned)rc);
        return -1;
    }

    /* Interpose on the ring-wrap callback.
     *
     * This is the one flush we do not ask for: when `current` reaches `end`,
     * PSL1GHT's callback advances PUT and waits for the RSX to free ring space
     * -- which lets the GPU consume every command written so far. A frame of
     * Mario Kart writes well over the 1 MiB ring, so it fires mid-frame,
     * routinely. Without the fence here, dropping the per-draw join would let
     * the RSX read vertex buffers the SPEs had not finished writing, and the
     * corruption would appear only in heavy scenes -- the worst kind to chase.
     *
     * Chaining rather than replacing: PSL1GHT's own callback does the actual
     * wrap, and reimplementing it would be a second copy of something we do
     * not own. */
    /* Chaining to PSL1GHT's wrap handler is opt-in until proven safe.
     *
     * gcmContextData::callback is declared ATTRIBUTE_PRXPTR, which means it is
     * not necessarily an ordinary function-descriptor pointer -- calling it
     * from our code may not be a plain indirect call at all. The emulator
     * wedges in present/flip at ~10 frames with the hook installed, which is
     * about when the 1 MiB command ring first wraps, and it does so with the
     * SPU path disabled entirely. That is the shape of a bad call, not of a
     * missing fence. Off by default until the alternative below is measured. */
    {   FILE *hf = fopen("/dev_hdd0/tmp/wiicompiled-ringhook.txt", "rb");
        if (hf) fclose(hf);
        else {
            LOG_INFO(LOG_RSX, "rsx: ring-wrap hook disabled (default); "
                              "headroom fence in the draw path instead");
            s_ctx = s_ctx;   /* nothing to install */
            goto ring_hook_done;
        }
    }
    if (s_ctx->callback) {
        s_ring_cb_prev = s_ctx->callback;
        s_ctx->callback = rsx_ring_wrap_fenced;
        LOG_INFO(LOG_RSX, "rsx: ring-wrap fence installed");
    } else {
        /* No callback to chain to means we cannot make the wrap safe, and
         * replacing it with one that only fences would break the wrap itself.
         * Say so loudly and leave the draw path to fence per draw instead --
         * slower, but never wrong. */
        /* No callback to chain to. That used to mean fencing on EVERY draw,
         * which is correct but ruinous: measured in-race, 10,817,855 joins of
         * which 93% actually blocked, 504 spin iterations each, 8.8% of the
         * frame spent asleep in spu_vtx_join -- while the SPU itself was idle
         * 71 us per 3.45 us job. The PPE dispatched a job and immediately
         * waited for it, so nothing ever overlapped.
         *
         * The draw path already tests rsx_ring_low() beside the fence, and
         * that test is a sufficient substitute: it fences when fewer than
         * 64 KiB of ring remain, and a single draw emits far less than that,
         * so the SPUs are always caught up BEFORE the wrap the callback
         * existed to guard. Fencing again on every draw only re-proves it.
         *
         * wiicompiled-fenceperdraw.txt restores the old behaviour for A/B. */
        LOG_WARN(LOG_RSX, "rsx: no ring-wrap callback to chain; "
                          "relying on the ring-headroom fence instead");
        {   FILE *ff = fopen("/dev_hdd0/tmp/wiicompiled-fenceperdraw.txt", "r");
            if (ff) { fclose(ff); g_spu_fence_per_draw = 1; }
        }
        LOG_INFO(LOG_CORE, "rsx: SPU fence mode = %s",
                 g_spu_fence_per_draw ? "PER-DRAW (flag)" : "ring headroom");
    }
ring_hook_done:;

    rc = videoGetState(0, 0, &state);
    if (rc != 0) {
        LOG_ERROR(LOG_VIDEO, "rsx: videoGetState failed: %08x", (unsigned)rc);
        return -1;
    }
    if (state.state != 0) {
        /* 0 means "enabled". Anything else means no display is connected in a
         * usable state, and configuring would fail. */
        LOG_ERROR(LOG_VIDEO, "rsx: display not ready (state %u)",
                  (unsigned)state.state);
        return -1;
    }

    /* Prefer the console's NATIVE 480p over whatever mode the dashboard is
     * in.
     *
     * The Wii's highest progressive output is 480p, and we render what the
     * title draws -- a 608x456 EFB for Mario Kart Wii. Presenting that on a
     * 1080p display means a 2.37x vertical upscale, and a non-integer vertical
     * scale is exactly what produces regular horizontal banding (rows land on
     * uneven source lines) and an overall softness that no amount of internal
     * resolution fixes. Asking the PS3 for 480p instead makes the vertical
     * scale 456 -> 480, essentially 1:1, and lets the TV do the only scaling
     * -- which is what it is designed for and what a real Wii would have fed
     * it.
     *
     * This is a standard video mode, not a CFW trick, and availability is
     * asked rather than assumed: a display that will not take 480p keeps the
     * current mode. A file can force a different choice for comparison. */
    {
        u32 want = VIDEO_RESOLUTION_480;
        FILE *rf = fopen("/dev_hdd0/tmp/wiicompiled-res.txt", "r");
        if (rf) {
            unsigned v = 0;
            if (fscanf(rf, "%u", &v) == 1 && v) want = v;
            fclose(rf);
        }
        if (want != state.displayMode.resolution &&
            videoGetResolutionAvailability(0, want, VIDEO_ASPECT_AUTO, 0)) {
            LOG_INFO(LOG_VIDEO, "rsx: switching display mode %u -> %u "
                     "(native, no upscale)",
                     (unsigned)state.displayMode.resolution, (unsigned)want);
            state.displayMode.resolution = want;
        } else if (want != state.displayMode.resolution) {
            LOG_INFO(LOG_VIDEO, "rsx: mode %u unavailable, keeping %u",
                     (unsigned)want, (unsigned)state.displayMode.resolution);
        }
    }

    rc = videoGetResolution(state.displayMode.resolution, &res);
    if (rc != 0) {
        LOG_ERROR(LOG_VIDEO, "rsx: videoGetResolution failed: %08x",
                  (unsigned)rc);
        return -1;
    }

    memset(&cfg, 0, sizeof cfg);
    cfg.resolution = state.displayMode.resolution;   /* keep the current mode */
    cfg.format     = VIDEO_BUFFER_FORMAT_XRGB;
    cfg.aspect     = state.displayMode.aspect;
    /* A tiled pitch is NOT an alignment: the memory controller factorises the
     * pitch into (1|3|5|7|13) * 2^k to work out tiles-per-row, and a pitch it
     * cannot factorise cannot be tiled at all. The legal set is a finite
     * hardware list, so ask the library rather than rounding: 1280*4 = 5120 is
     * already legal and nothing moves at 720p, but 1920*4 = 7680 is not and
     * becomes 8192, and 720*4 = 2880 becomes 3072.
     *
     * The SCANOUT pitch has to be the tiled pitch too. gcmSetDisplayBuffer's
     * pitch must agree with videoConfigure's and with the tile region's, and a
     * disagreement is a distinct RSX graphics error rather than a soft
     * failure -- which is why this is the first line of the change and not an
     * afterthought once the surfaces are tiled. */
    cfg.pitch      = want_tile ? gcmGetTiledPitchSize(res.width * 4)
                               : res.width * 4;

    rc = videoConfigure(0, &cfg, NULL, 0);
    if (rc != 0) {
        LOG_ERROR(LOG_VIDEO, "rsx: videoConfigure failed: %08x", (unsigned)rc);
        return -1;
    }

    g_rsx.width  = res.width;
    g_rsx.height = res.height;
    g_rsx.pitch  = cfg.pitch;

    /* On NV4x colour pitch and depth pitch are separate registers, so the
     * depth buffer gets its own legal pitch rather than borrowing colour's.
     * Z24S8 is 4 bytes per pixel, so at every resolution we support the two
     * come out equal -- but they are computed separately because nothing
     * guarantees that, and a depth pitch that is not on the legal list is a
     * silently untiled depth buffer. */
    g_rsx.depth_pitch = want_tile ? gcmGetTiledPitchSize(g_rsx.width * 4)
                                  : g_rsx.width * 4;

    /* Buffer height rounds to 32 lines in local memory -- that is the height
     * of one 8 KiB frame-buffer tile -- and the whole region then rounds up to
     * a multiple of 64 KiB. The depth buffer additionally rounds to 64 lines,
     * because a Zcull region's height must be a multiple of 64 and it is
     * cleaner for the depth surface to genuinely cover the region the Zcull
     * unit believes it owns than to reason about a 32-line overhang.
     *
     * The rounded-up tail is real memory that the RSX, the PPU and the SPUs
     * must all leave alone: touching it raises no error, it simply corrupts.
     * Nothing here ever addresses past `height` rows, so that holds.
     *
     * At 1920x1080 this is 8,912,896 bytes per colour buffer and the same for
     * depth: 34.0 MiB for three buffers plus depth, 42.5 MiB for four, against
     * 31.6 MiB for the untiled three-buffer layout it replaces. At 1280x720 it
     * is 14.6 / 18.2 MiB. Local memory is 256 MiB, so the buffer count is not
     * constrained by this. */
    cb_h       = want_tile ? align_up_u32(g_rsx.height, GCM_TILE_LOCAL_ALIGN_HEIGHT)
                           : g_rsx.height;
    cb_size    = want_tile ? align_up_u32(g_rsx.pitch * cb_h, GCM_TILE_ALIGN_SIZE)
                           : g_rsx.pitch * g_rsx.height;
    depth_h    = want_tile ? align_up_u32(g_rsx.height, GCM_ZCULL_ALIGN_HEIGHT)
                           : g_rsx.height;
    depth_size = want_tile ? align_up_u32(g_rsx.depth_pitch * depth_h,
                                          GCM_TILE_ALIGN_SIZE)
                           : g_rsx.depth_pitch * g_rsx.height;
    g_rsx.buffer_bytes = cb_size;
    g_rsx.depth_bytes  = depth_size;

    /* Allocate everything before binding anything, and do not register a
     * display buffer yet: the documented idle requirement for touching tile
     * registers includes "the region is not being scanned out", and the only
     * moment that is free is before the first gcmSetDisplayBuffer. */
    for (i = 0; i < RSX_BUFFERS; i++) {
        g_rsx.buffer[i] = (u32 *)rsxMemalign(
            want_tile ? GCM_TILE_ALIGN_OFFSET : 64, cb_size);
        if (!g_rsx.buffer[i]) {
            LOG_ERROR(LOG_VIDEO, "rsx: framebuffer %d allocation failed", i);
            return -1;
        }
        if (rsxAddressToOffset(g_rsx.buffer[i], &g_rsx.offset[i]) != 0) {
            LOG_ERROR(LOG_VIDEO, "rsx: framebuffer %d has no RSX offset", i);
            return -1;
        }
        if (want_tile && (g_rsx.offset[i] & (GCM_TILE_ALIGN_OFFSET - 1u))) {
            /* A 64 KiB-aligned CPU address whose RSX offset is not 64 KiB
             * aligned would mean the local-memory window does not start on a
             * 64 KiB boundary. gcmSetTileInfo rejects it, so drop tiling
             * rather than press on and get an alignment error per region. */
            LOG_ERROR(LOG_VIDEO,
                      "rsx: framebuffer %d offset %08x is not 64K aligned; "
                      "tiling disabled", i, (unsigned)g_rsx.offset[i]);
            want_tile = 0;
        }
    }

    /* The dedicated EFB surface: same geometry, pitch and format as a display
     * buffer, so nothing about the EFB->screen coordinate mapping changes --
     * only WHERE draws and copy-clears land. Armed by wiicompiled-efbsurf.txt; a
     * failed allocation is not fatal, it just leaves the old behaviour. */
    if (rsx_efb_surface_wanted()) {
        g_rsx.efb = (u32 *)rsxMemalign(
            want_tile ? GCM_TILE_ALIGN_OFFSET : 64, cb_size);
        if (!g_rsx.efb ||
            rsxAddressToOffset(g_rsx.efb, &g_rsx.efb_offset) != 0) {
            LOG_ERROR(LOG_VIDEO, "rsx: EFB surface allocation failed; "
                                 "rendering into the display buffer");
            g_rsx.efb = NULL; g_rsx.efb_offset = 0;
        } else {
            LOG_INFO(LOG_CORE, "rsx: EFB surface at offset %08x (%u bytes)",
                     (unsigned)g_rsx.efb_offset, (unsigned)cb_size);
        }
    }

    g_rsx.depth = (u32 *)rsxMemalign(
        want_tile ? GCM_TILE_ALIGN_OFFSET : 64, depth_size);
    if (!g_rsx.depth ||
        rsxAddressToOffset(g_rsx.depth, &g_rsx.depth_offset) != 0) {
        LOG_ERROR(LOG_VIDEO, "rsx: depth buffer allocation failed");
        return -1;
    }
    if (want_tile && (g_rsx.depth_offset & (GCM_TILE_ALIGN_OFFSET - 1u))) {
        LOG_ERROR(LOG_VIDEO,
                  "rsx: depth offset %08x is not 64K aligned; tiling disabled",
                  (unsigned)g_rsx.depth_offset);
        want_tile = 0;
    }

    /* --- tile regions and Zcull ------------------------------------- */
    if (want_tile) {
        u32 zc_w = align_up_u32(g_rsx.width,  GCM_ZCULL_ALIGN_WIDTH);
        u32 zc_h = align_up_u32(g_rsx.height, GCM_ZCULL_ALIGN_HEIGHT);

        if (!rsx_tile_wait_idle(s_ctx))
            LOG_ERROR(LOG_VIDEO,
                      "rsx: RSX did not report idle before the tile binds; "
                      "binding anyway");

        /* Zcull first, then the tiles, then (below) the display buffers, which
         * is the standard zcull setup order.
         *
         * Constraints, all satisfied by construction above: the depth offset
         * is 64 KiB aligned and therefore also the required 4 KiB aligned; the
         * depth buffer is in LOCAL memory; width and height are multiples of
         * 64 (the hardware packs them as w>>6 and h>>6, so this is structural
         * rather than advisory); cullStart is 0, which is 4 KiB aligned;
         * zFormat matches the surface's Z24S8 and aaFormat matches its
         * CENTER_1, both of which a region must match exactly or it never
         * becomes active and silently culls nothing.
         *
         * Direction LESS is forced by our depth setup rather than chosen: the
         * viewport maps near to window z 0 and far to 1 and the GX path emits
         * LESS/LEQUAL, and LESS is also the case in which LONES (count leading
         * ones) is the correct 12-bit compression of the 24-bit Z range.
         *
         * Stencil culling is switched off rather than configured: sFunc
         * ALWAYS with sRef and sMask 0 cannot reject anything, and
         * rsxSetZCullEnable asks for depth culling only. Nothing in the GX
         * path drives stencil yet, and a stencil cull criterion that does not
         * match the (unused) stencil state is a way to lose geometry for no
         * gain. */
        if ((u64)zc_w * (u64)zc_h > (u64)GCM_ZCULL_RAM_SIZE_MAX) {
            LOG_ERROR(LOG_VIDEO,
                      "rsx: zcull region %ux%u exceeds the %u byte on-chip "
                      "RAM; zcull disabled",
                      (unsigned)zc_w, (unsigned)zc_h,
                      (unsigned)GCM_ZCULL_RAM_SIZE_MAX);
        } else {
            rc = gcmBindZcull((u8)ZCULL_REGION, g_rsx.depth_offset,
                              zc_w, zc_h, 0 /* cullStart */,
                              GCM_ZCULL_Z24S8, GCM_SURFACE_CENTER_1,
                              GCM_ZCULL_LESS, GCM_ZCULL_LONES,
                              GCM_SCULL_SFUNC_ALWAYS, 0 /* sRef */,
                              0 /* sMask */);
            if (rc != 0) {
                LOG_ERROR(LOG_VIDEO, "rsx: gcmBindZcull failed: %08x",
                          (unsigned)rc);
            } else {
                g_rsx.zcull        = 1;
                g_rsx.zcull_width  = zc_w;
                g_rsx.zcull_height = zc_h;
            }
        }

        /* Depth: its own region, compressed. Every constraint on Z
         * compression is met -- local memory, Z24S8 (Z16 has no compression
         * mode on this chip at all), colour format is 32bpp so compression is
         * not silently skipped the way it is at 128bpp, and colour and depth
         * are in SEPARATE regions, which is required: a region holding both
         * cannot compress at all. The mode has to match the surface's
         * antialias mode, and CENTER_1 selects _REGULAR; _DIAGONAL and
         * _ROTATED are the 2x and 4x MSAA variants and we have no MSAA.
         *
         * Tag RAM: this region consumes depth_size/64K units from base 0 --
         * 60 of the 2048 available at 720p, 136 at 1080p. It is the only
         * compressed region, so no other region can share those entries --
         * which matters, because two live regions sharing tag entries is
         * corruption rather than an error. */
        rc = gcmSetTileInfo((u8)TILE_REGION_DEPTH, GCM_LOCATION_RSX,
                            g_rsx.depth_offset, depth_size, g_rsx.depth_pitch,
                            GCM_COMPMODE_Z32_SEPSTENCIL_REGULAR,
                            (u16)TILE_TAG_BASE_DEPTH, (u8)TILE_BANK_DEPTH);
        if (rc == 0)
            rc = gcmBindTile((u8)TILE_REGION_DEPTH);
        if (rc != 0) {
            LOG_ERROR(LOG_VIDEO, "rsx: depth tile bind failed: %08x",
                      (unsigned)rc);
            want_tile = 0;
        }

        /* Colour: one region per buffer, uncompressed. There is no legal
         * colour compression here and that is not a tuning decision -- the two
         * colour modes are the 2-sample and 4-sample multisample formats, and
         * this surface is single sampled. Tiling it is still worth it on its
         * own: it is the bank/row access pattern the ROPs and the scanout see,
         * and a linear colour buffer is measured at half the throughput of a
         * tiled one under a ROP-bound load. */
        for (i = 0; want_tile && i < RSX_BUFFERS; i++) {
            rc = gcmSetTileInfo((u8)(TILE_REGION_COLOR0 + (u32)i),
                                GCM_LOCATION_RSX, g_rsx.offset[i], cb_size,
                                g_rsx.pitch, GCM_COMPMODE_DISABLED,
                                0 /* base: no tag RAM when uncompressed */,
                                (u8)TILE_BANK_COLOR);
            if (rc == 0)
                rc = gcmBindTile((u8)(TILE_REGION_COLOR0 + (u32)i));
            if (rc != 0) {
                LOG_ERROR(LOG_VIDEO, "rsx: colour tile %d bind failed: %08x",
                          i, (unsigned)rc);
                want_tile = 0;
            }
        }
        g_rsx.tiled = want_tile;
        /* A Zcull region over an UNTILED depth buffer cannot receive ROP
         * feedback at all, so it is not merely less effective, it is dead
         * weight. If the tile binds failed, take the Zcull region back down
         * with them rather than leaving the per-frame programming running for
         * nothing. */
        if (!want_tile && g_rsx.zcull) {
            gcmUnbindZcull((u8)ZCULL_REGION);
            g_rsx.zcull = 0;
        }
    }

    for (i = 0; i < RSX_BUFFERS; i++) {
        if (gcmSetDisplayBuffer((u8)i, g_rsx.offset[i], g_rsx.pitch,
                                g_rsx.width, g_rsx.height) != 0) {
            LOG_ERROR(LOG_VIDEO, "rsx: gcmSetDisplayBuffer(%d) failed", i);
            return -1;
        }
    }

    /* GPU context defaults that PSL1GHT never sets.
     *
     * The registers below come from nouveau's context initialisation for this
     * exact GPU family (nv30_screen.c) -- the sequence a real driver writes at
     * screen creation before anything renders. PSL1GHT has no wrappers for
     * them and never touches them, so their content here is whatever the
     * dashboard left. The 0x1fc4..0x1fd4 group is the routing table mapping
     * vertex-program outputs to rasteriser interpolants: with it scrambled, a
     * program can write COL0 forever and the fragment stage will read an
     * unrouted constant -- while the position path still works, because
     * position does not route through it. That is precisely the symptom set
     * this display path exhibited, through every shader-side hypothesis.
     *
     * Written raw because no wrapper exists; the values are nouveau's, kept
     * verbatim. The table itself now lives at file scope (k_ctx_init) because
     * a flip reinitialises the RSX context and every one of these has to be
     * re-issued each frame -- see rsx_emit_raw_defaults above. */
    rsx_emit_raw_defaults();
    rsx_flush_fenced(s_ctx);

    /* Without a flip mode the display engine never retires a queued flip, so
     * gcmGetFlipStatus never clears and anything waiting on it waits forever.
     * That is not a subtle performance detail: omitting this line is what made
     * the first console run hang on a black screen. VSYNC is also what an
     * emulator wants -- the Wii's own framebuffer swap is tied to scanout. */
    /* libgcm Reference, cellGcmSetFlipMode: "When using cellVideoOutConfigure(),
     * the settings made by this function may return to CELL_GCM_DISPLAY_VSYNC.
     * After using cellVideoOutConfigure(), make sure to renew settings" -- which
     * is why this is here and not before videoConfigure.
     *
     * VSYNC rather than HSYNC.  HSYNC lets the flip land at the next horizontal
     * sync instead of the next V, which removes the last ~8 ms of average
     * present latency, but the Reference warns a flip issued during VBLANK then
     * lands after the first scan line so the previous buffer shows on rows 0-1
     * (flip_immediate/main.cpp works around it with cellGcmSetScissor(0,2,...)).
     * That is a visible-behaviour change no test here can judge, so it is a
     * one-line switch and not the default. */
    gcmSetFlipMode(GCM_FLIP_VSYNC);
    gcmResetFlipStatus();

    /* ---- NON-BLOCKING PRESENTATION: arm the interrupt path ---- */
    {
        u8 cur = 0;
        int b;
        if (gcmGetCurrentDisplayBufferId(&cur) != 0 || cur >= RSX_BUFFERS)
            cur = 0;
        s_buf_on_display = cur;
        s_buf_flipped    = cur;
        s_flip_in_flight = 0;
        for (b = 0; b < RSX_BUFFERS; b++)
            s_buf_busy[b] = ((u32)b == s_buf_on_display) ? 1u : 0u;

        /* The VBlank handler reads this word every V from the moment it is
         * installed, so it has to say "the buffer already on screen" before
         * that happens or the first V issues a flip to a buffer nothing has
         * drawn.  flip_immediate/main.cpp:423 seeds it the same way. */
        s_label_prepared = (volatile u32 *)gcmGetLabelAddress(RSX_LABEL_PREPARED);
        if (s_label_prepared)
            *s_label_prepared = (s_buf_on_display << 8);
        /* The per-buffer status labels are seeded too.  We do not make the RSX
         * wait on them -- the PPU-side gate in rsx_acquire_buffer is what keeps
         * us off the scanned-out surface, and a GPU-side wait label would turn
         * "the XMB is holding the flip" into a wedged command processor.  They
         * are kept consistent so a future rsxSetWaitLabel can be dropped in. */
        for (b = 0; b < RSX_BUFFERS; b++) {
            volatile u32 *l =
                (volatile u32 *)gcmGetLabelAddress(RSX_LABEL_BUFSTATUS + b);
            if (l) *l = s_buf_busy[b];
        }

        gcmSetFlipHandler(rsx_flip_handler);
        gcmSetVBlankHandler(rsx_vblank_handler);
        s_handlers_up = (s_label_prepared != NULL);
    }

    g_rsx.current = (int)((s_buf_on_display + 1u) % RSX_BUFFERS);
    g_rsx.frames  = 0;
    g_rsx.inited  = 1;
    g_rsx.flip_timeouts = 0;

    LOG_INFO(LOG_VIDEO, "rsx: %ux%u, pitch %u", (unsigned)g_rsx.width,
             (unsigned)g_rsx.height, (unsigned)g_rsx.pitch);
    LOG_INFO(LOG_VIDEO,
             "rsx: present=prepareflip+vblank-irq buffers=%d flipmode=VSYNC "
             "handlers=%s label=%p on_display=%u",
             (int)RSX_BUFFERS, s_handlers_up ? "flip+vblank" : "NONE",
             (void *)s_label_prepared, (unsigned)s_buf_on_display);
    /* A flip reinitialises the whole RSX context (libgcm Overview 5.3), so this
     * is the list a console run can check against what it sees on screen: every
     * item here is re-issued once per presented frame, after the flip that
     * reset it.  Anything NOT in this list and not re-issued per draw is
     * running on the power-on default. */
    LOG_INFO(LOG_VIDEO,
             "rsx: per-frame context reset (libgcm Overview 5.3): surface, "
             "raw{0x1450,zcull 0x1ea4,interp 0x1fc4/1fc8/1fd0/1fd4,"
             "xform-timeout 0x1ef8,0x1d64}, depthformat, scissor, shademodel, "
             "colormaskmrt, zminmax, viewportclip[0-7], userclipplane, "
             "frontface, colormask, viewport");
    LOG_INFO(LOG_VIDEO,
             "rsx: per-draw (gx_render.c): blend, logicop, alphatest, depth "
             "func/test/write, colormask, cull+frontface, viewport+scissor, "
             "vp/fp programs, vp constants, texture bind+sampler, vertex "
             "attribute arrays");
    /* What was actually configured, not what was asked for. A console run has
     * no debugger and the difference between "tiled" and "asked for tiled and
     * the memory controller refused" is invisible on screen, so it is printed:
     * pitches, region indices, compression modes and every Zcull parameter. */
    LOG_INFO(LOG_VIDEO,
             "rsx: tiling %s  colour pitch %u (%u linear)  depth pitch %u  "
             "colour %u B/buf  depth %u B",
             g_rsx.tiled ? "ON" : "off",
             (unsigned)g_rsx.pitch, (unsigned)(g_rsx.width * 4u),
             (unsigned)g_rsx.depth_pitch,
             (unsigned)g_rsx.buffer_bytes, (unsigned)g_rsx.depth_bytes);
    if (g_rsx.tiled) {
        LOG_INFO(LOG_VIDEO,
                 "rsx: using %u of %u tile regions (1 depth + %u colour) and "
                 "%u of %u zcull regions",
                 (unsigned)TILE_REGIONS_USED, (unsigned)TILE_REGION_COUNT,
                 (unsigned)RSX_BUFFERS, g_rsx.zcull ? 1u : 0u,
                 (unsigned)ZCULL_REGION_COUNT);
        LOG_INFO(LOG_VIDEO,
                 "rsx: tile region %u = depth off %08x comp %u (Z32_SEPSTENCIL"
                 "_REGULAR) tagbase %u bank %u",
                 (unsigned)TILE_REGION_DEPTH, (unsigned)g_rsx.depth_offset,
                 (unsigned)GCM_COMPMODE_Z32_SEPSTENCIL_REGULAR,
                 (unsigned)TILE_TAG_BASE_DEPTH, (unsigned)TILE_BANK_DEPTH);
        for (i = 0; i < RSX_BUFFERS; i++)
            LOG_INFO(LOG_VIDEO,
                     "rsx: tile region %u = colour%d off %08x comp %u "
                     "(DISABLED, no MSAA) bank %u",
                     (unsigned)(TILE_REGION_COLOR0 + (u32)i), i,
                     (unsigned)g_rsx.offset[i],
                     (unsigned)GCM_COMPMODE_DISABLED, (unsigned)TILE_BANK_COLOR);
    }
    LOG_INFO(LOG_VIDEO,
             "rsx: zcull %s region %u off %08x %ux%u cullStart 0 "
             "fmt Z24S8 aa CENTER_1 dir LESS enc LONES scull ALWAYS/0/0 "
             "(%u of %u bytes of on-chip RAM)",
             g_rsx.zcull ? "ON" : "off", (unsigned)ZCULL_REGION,
             (unsigned)g_rsx.depth_offset, (unsigned)g_rsx.zcull_width,
             (unsigned)g_rsx.zcull_height,
             (unsigned)(g_rsx.zcull_width * g_rsx.zcull_height),
             (unsigned)GCM_ZCULL_RAM_SIZE_MAX);
    s_inited = 1;
    return 0;
    }
}

int rsx_zcull_active(void)
{
    /* Both halves: the region has to be bound (an init-time fact) and the
     * feature bit still set (a run-time one, so the pad bisect can switch the
     * Zcull programming off without a rebuild). */
    return g_rsx.inited && g_rsx.zcull &&
           (g_gx_state_mask & GX_STATE_RSX_TILE) != 0;
}

/* The Zcull control registers, re-emitted once per frame.
 *
 * THIS IS NOT AN OPTIMISATION THAT COULD BE HOISTED TO INIT, for exactly the
 * reason the per-frame context-reset block above exists: a flip reinitialises
 * the whole RSX context, and ZCULL_CONTROL0/1, SCULL_CONTROL and ZCULL_EN are
 * ordinary NV4x methods that go with it. (The tile and Zcull *region* binds
 * are not -- those are privileged calls into the memory controller rather than
 * context state, which is why they can be done once and only once.)
 *
 * Ordering is load-bearing in one direction: writing ZCullControl or
 * SCullControl INVALIDATES the region, even when the value written is
 * identical to the value already there, and a region stays dead until the next
 * clear revalidates it. So the control registers go before the clear and the
 * enable goes after it. ZCullLimit does not invalidate and could go anywhere;
 * it is here to keep the three together.
 *
 * INVARIANT WITH k_ctx_init. That table also carries the same three registers
 * raw, as `{ 0x1ea4, 3, ... }`, with values that happen to agree on
 * ZCULL_CONTROL0/1 (LONES/LESS, limits 0x100/0x100) and differ only on
 * SCULL_CONTROL, which is inert because rsxSetZCullEnable below asks for depth
 * culling only. That is safe ONLY because rsx_emit_raw_defaults() runs earlier
 * in rsx_frame_begin than this function does. If the raw block is ever moved
 * to after the clear, it must lose its 0x1ea4 entry, or it will invalidate the
 * region every frame and Zcull will silently do nothing at all. */
void rsx_zcull_before_clear(void)
{
    if (!rsx_zcull_active())
        return;
    rsxSetZCullControl(s_ctx, GCM_ZCULL_LESS, GCM_ZCULL_LONES);
    rsxSetZCullLimit(s_ctx, 0x100, 0x100);
    rsxSetSCullControl(s_ctx, GCM_SCULL_SFUNC_ALWAYS, 0, 0);
    /* A Zcull region only becomes valid off the back of a HARDWARE FAST
     * clear, never off the quad-clear fallback, and one of the fast-clear
     * preconditions is that the stencil mask is all-or-nothing. 0xff is the
     * post-context-reset default, so this is redundant today and stops being
     * redundant the moment the GX path starts driving stencil -- at which
     * point the failure would be a silent loss of all culling rather than
     * anything visible. */
    rsxSetStencilMask(s_ctx, 0xff);
}

void rsx_zcull_after_depth_clear(void)
{
    if (!rsx_zcull_active())
        return;
    /* Clear the Zcull RAM from the same clear-depth value the surface clear
     * just used, so the hierarchical representation and the depth buffer agree
     * rather than the former holding whatever the previous frame left. */
    rsxSetClearZCullSurface(s_ctx, GCM_TRUE, GCM_TRUE);
    /* Depth culling only. Stencil culling is deliberately off: see the
     * gcmBindZcull call. */
    rsxSetZCullEnable(s_ctx, GCM_TRUE, GCM_FALSE);
}

/* Armed by a flag file so the change can be measured against the known-good
 * path on hardware without a rebuild, and disabled from FTP if it misbehaves. */
int rsx_efb_surface_wanted(void)
{
    static int on = -1;
    if (on < 0) {
        FILE *f = fopen("/dev_hdd0/tmp/wiicompiled-efbsurf.txt", "r");
        on = 0;
        if (f) { fclose(f); on = 1; }
        LOG_INFO(LOG_CORE, "rsx: dedicated EFB surface %s",
                 on ? "REQUESTED" : "off");
    }
    return on;
}

u32 rsx_render_target_offset(void)
{
    return g_rsx.efb ? g_rsx.efb_offset : g_rsx.offset[g_rsx.current];
}

/* Copy the embedded framebuffer to the display buffer. This is the moment the
 * title's present copy becomes visible; everything drawn since the last one
 * lives in the EFB, where copy-clears cannot reach the presented image. */
u64 g_efb_blits, g_efb_blit_skips;

void rsx_efb_to_display(void)
{
    gcmContextData *c = s_ctx;
    gcmTransferScale sc;
    gcmTransferSurface ds;

    if (!g_rsx.inited || !g_rsx.efb || !c) {
        g_efb_blit_skips++;
        return;
    }
    g_efb_blits++;

    rsxSetWaitForIdle(c);
    memset(&sc, 0, sizeof sc);
    memset(&ds, 0, sizeof ds);

    ds.format = GCM_TRANSFER_SURFACE_FORMAT_A8R8G8B8;
    ds.pitch  = (u16)g_rsx.pitch;
    ds.offset = g_rsx.offset[g_rsx.current];

    sc.conversion = GCM_TRANSFER_CONVERSION_TRUNCATE;
    sc.format     = GCM_TRANSFER_SCALE_FORMAT_A8R8G8B8;
    sc.operation  = GCM_TRANSFER_OPERATION_SRCCOPY;
    sc.clipX = 0; sc.clipY = 0;
    sc.clipW = (u16)g_rsx.width; sc.clipH = (u16)g_rsx.height;
    sc.outX  = 0; sc.outY  = 0;
    sc.outW  = (u16)g_rsx.width; sc.outH  = (u16)g_rsx.height;
    /* 1:1 -- the EFB surface has the display's exact geometry, so this is a
     * straight copy and must not resample. */
    sc.ratioX = rsxGetFixedSint32(1.0f);
    sc.ratioY = rsxGetFixedSint32(1.0f);
    sc.inW    = (u16)g_rsx.width;
    sc.inH    = (u16)g_rsx.height;
    sc.pitch  = (u16)g_rsx.pitch;
    sc.origin = GCM_TRANSFER_ORIGIN_CORNER;
    sc.interp = GCM_TRANSFER_INTERPOLATOR_NEAREST;
    sc.offset = g_rsx.efb_offset;
    sc.inX    = rsxGetFixedUint16(0.0f);
    sc.inY    = rsxGetFixedUint16(0.0f);

    rsxSetTransferScaleMode(c, GCM_TRANSFER_LOCAL_TO_LOCAL,
                            GCM_TRANSFER_SURFACE);
    rsxSetTransferScaleSurface(c, &sc, &ds);
}

void rsx_frame_begin(void)
{
    /* NO WAIT HERE.  The buffer this frame draws into was chosen in
     * rsx_frame_end from state the flip interrupt maintains; by the time
     * control reaches here it is already known free.  What used to be at the
     * top of this function -- a usleep poll on the single global
     * gcmGetFlipStatus flag -- is gone entirely. */
    gcmSurface s;
    int b = g_rsx.current;

    if (!g_rsx.inited)
        return;

    memset(&s, 0, sizeof s);
    /* GCM_SURFACE_TYPE_LINEAR (1), *not* GCM_TEXTURE_LINEAR (2). The texture
     * constant of the same name has a different value, and 2 in this field
     * means SWIZZLE -- telling the RSX a linear framebuffer is swizzle-tiled,
     * which wedges it on the first draw. Flips then never retire, which looks
     * exactly like a broken flip path and is not one. */
    s.type              = GCM_SURFACE_TYPE_LINEAR;
    s.antiAlias         = GCM_SURFACE_CENTER_1;
    s.colorFormat       = GCM_SURFACE_A8R8G8B8;
    s.colorTarget       = GCM_SURFACE_TARGET_0;
    s.colorLocation[0]  = GCM_LOCATION_RSX;
    s.colorOffset[0]    = g_rsx.efb ? g_rsx.efb_offset : g_rsx.offset[b];
    s.colorPitch[0]     = g_rsx.pitch;
    /* Targets 1-3 are unused, but the pitch fields must still be legal: the
     * RSX rejects a surface whose unused pitches are zero. */
    s.colorLocation[1] = s.colorLocation[2] = s.colorLocation[3] = GCM_LOCATION_RSX;
    s.colorOffset[1] = s.colorOffset[2] = s.colorOffset[3] = 0;
    s.colorPitch[1] = s.colorPitch[2] = s.colorPitch[3] = 64;

    s.depthFormat   = GCM_SURFACE_ZETA_Z24S8;
    s.depthLocation = GCM_LOCATION_RSX;
    s.depthOffset   = g_rsx.depth_offset;
    s.depthPitch    = g_rsx.depth_pitch;

    s.width  = (u16)g_rsx.width;
    s.height = (u16)g_rsx.height;
    s.x = s.y = 0;

    rsxSetSurface(s_ctx, &s);

    /* ---- PER-FRAME RSX CONTEXT RESET ----
     * The flip that ended the previous frame reinitialised the RSX context
     * (libgcm Overview 5.3 "Resetting the RSX(TM) Context"; cellGcmSetPrepareFlip
     * Reference: "RSX(TM) contexts will be re-initialized when this command is
     * executed"), so the raw methods below have reverted to whatever libgcm's
     * 4 KB init buffer leaves and must be re-issued here.  They used to be
     * written once in rsx_video_init and never again. */
    rsx_emit_raw_defaults();

    /* Depth buffer interpretation.  libgcm Reference cellGcmSetDepthFormat:
     * "Initial value is CELL_GCM_DEPTH_FORMAT_FIXED", which is what a Z24S8
     * surface wants, so this is not a behaviour change -- it makes the
     * dependency explicit and, via the PSL1GHT wrapper, sets the 0x00100000 bit
     * that NV40TCL_CONTROL0 carries unconditionally and that nothing else in
     * this process writes.  Emitted here, before any per-draw depth state, so a
     * draw's own depth func / test / write always wins. */
    rsxSetDepthFormat(s_ctx, GCM_DEPTH_FORMAT_FIXED);

    /* The scissor bounds the clear as well as drawing, so it has to cover the
     * frame or a clear silently does nothing. The viewport transform maps
     * normalised device coordinates onto the buffer; it is set here so a draw
     * needs only its own state. */
    {
        f32 scale[4], offset[4];
        f32 w = (f32)g_rsx.width, h = (f32)g_rsx.height;

        rsxSetScissor(s_ctx, 0, 0, (u16)g_rsx.width, (u16)g_rsx.height);
        /* Smooth shading, explicitly. A flat shade model takes the whole
         * primitive's colour from one vertex, which renders a perfectly
         * uniform triangle -- indistinguishable from an interpolant that is
         * not varying, and not something any counter reports. */
        rsxSetShadeModel(s_ctx, GCM_SHADE_MODEL_SMOOTH);

        /* The rest of the state block every working PSL1GHT sample sets each
         * frame (rsxtest's setDrawEnv/drawFrame). The GPU state after the
         * dashboard hands over is whatever the dashboard left, and nothing in
         * rsxInit resets it -- the lv2 driver only builds the command ring.
         * Every register here is one the samples consider necessary, so all of
         * them are set rather than guessing which one matters. */
        rsxSetColorMaskMrt(s_ctx, 0);
        rsxSetZMinMaxControl(s_ctx, GCM_FALSE, GCM_TRUE, GCM_FALSE);
        {
            unsigned cl;
            for (cl = 0; cl < 8; cl++)
                rsxSetViewportClip(s_ctx, (u8)cl,
                                   (u16)g_rsx.width, (u16)g_rsx.height);
        }
        rsxSetUserClipPlaneControl(s_ctx,
                                   GCM_USER_CLIP_PLANE_DISABLE,
                                   GCM_USER_CLIP_PLANE_DISABLE,
                                   GCM_USER_CLIP_PLANE_DISABLE,
                                   GCM_USER_CLIP_PLANE_DISABLE,
                                   GCM_USER_CLIP_PLANE_DISABLE,
                                   GCM_USER_CLIP_PLANE_DISABLE);
        rsxSetFrontFace(s_ctx, GCM_FRONTFACE_CCW);
        rsxSetColorMask(s_ctx, GCM_COLOR_MASK_R | GCM_COLOR_MASK_G |
                               GCM_COLOR_MASK_B | GCM_COLOR_MASK_A);

        /* Y is flipped: the RSX's origin is top-left, clip space is bottom-up. */
        scale[0]  =  w * 0.5f;
        scale[1]  = -h * 0.5f;
        /* GX clip space is near=-1, far=0, and our projection fold already
         * negates it, so NDC z arrives as +1 at the near plane and 0 at the
         * far plane. The stock 0.5/0.5 window mapping then put NEAR at the
         * MAXIMUM depth value: GX_LEQUAL kept the FARTHEST fragment of every
         * pixel and half the 24-bit range was thrown away. Inverting the
         * window mapping makes window_z equal the GX screen depth exactly,
         * over the full range. */
        scale[2]  = -1.0f;
        scale[3]  =  0.0f;
        offset[0] =  w * 0.5f;
        offset[1] =  h * 0.5f;
        offset[2] =  1.0f;
        offset[3] =  0.0f;
        rsxSetViewport(s_ctx, 0, 0, (u16)g_rsx.width, (u16)g_rsx.height,
                       0.0f, 1.0f, scale, offset);
    }

    /* Zcull, last in the block and therefore AFTER rsx_emit_raw_defaults()
     * above -- which matters, because that table's `{ 0x1ea4, 3, ... }` entry
     * writes the same three registers raw and whichever write lands last is
     * the one that counts. After rsxSetSurface, because a Zcull region is only
     * *active* while the bound surface matches it; before the clear, because
     * these writes invalidate it and the clear is what makes it valid. */
    rsx_zcull_before_clear();
}

void rsx_clear(u32 argb)
{
    if (!g_rsx.inited)
        return;
    rsxSetClearColor(s_ctx, argb);
    rsxSetClearDepthStencil(s_ctx, 0xffffff00);
    /* A surface clear is gated by the write masks. The colour mask is
     * re-armed just above for exactly this reason; depth needs the same or
     * the Z clear silently does nothing. */
    rsxSetDepthWriteEnable(s_ctx, GCM_TRUE);
    /* Every colour channel is in the mask and the surface is a tiled, linear,
     * 32bpp buffer in local memory, which is the whole fast-clear precondition
     * list. That matters beyond the clear's own speed: it is what makes the
     * Zcull region valid. A clear that falls back to drawing a quad clears the
     * pixels and leaves Zcull dead. */
    rsxClearSurface(s_ctx, GCM_CLEAR_R | GCM_CLEAR_G | GCM_CLEAR_B |
                           GCM_CLEAR_A | GCM_CLEAR_Z | GCM_CLEAR_S);
    rsx_zcull_after_depth_clear();
}

void rsx_frame_end(void)
{
    s32 qid;

    if (!g_rsx.inited)
        return;

    /* If the interrupt path never came up (gcmGetLabelAddress failed), fall
     * back to the old command-stream flip.  Losing the picture because a label
     * address was unavailable would be a poor trade. */
    if (!s_handlers_up) {
        if (gcmSetFlip(s_ctx, (u8)g_rsx.current) == 0) {
            s_buf_flipped = (u32)g_rsx.current;
            g_rsx.last_queued = g_rsx.current;
            /* An RSX-side wait, not a PPU one: "This function blocks RSX(TM)
             * processing until the flip completes ... the execution of PPU
             * threads will not be blocked" (cellGcmSetWaitFlip).  It is what
             * keeps this degraded path off the displayed surface without
             * reintroducing a CPU stall. */
            gcmSetWaitFlip(s_ctx);
            rsx_flush_fenced(s_ctx);
        }
        g_rsx.current = (g_rsx.current + 1) % RSX_BUFFERS;
        g_rsx.frames++;
        return;
    }

    /* cellGcmSetPrepareFlip: "generates a command to carry out preprocessing
     * for buffer display output".  It does NOT flip -- gcmSetFlipImmediate(qid)
     * does, from the VBlank interrupt, once this command has been executed.
     * The library queue is 8 deep and a full queue is an error, not a stall. */
    g_rsx.last_queued = g_rsx.current;   /* the buffer with the newest pixels */
    qid = (s32)gcmSetPrepareFlip(s_ctx, (u8)g_rsx.current);
    if (qid < 0 || qid > 7) {
        /* Queue full.  Drop this present rather than sleep: the next frame's
         * PrepareFlip will succeed once the interrupt has drained one. */
        rsx_flush_fenced(s_ctx);
        s_present_drops++;
        g_rsx.frames++;
        return;
    }

    /* Publish "buffer N is finished and flippable" with a BACK END write.  The
     * distinction matters: cellGcmSetWriteCommandLabel fires when the front end
     * parses the command, which retires far too early; the back-end label fires
     * only after the render back end has flushed this frame's pixels.  That is
     * the guarantee cellGcmSetFlipImmediate demands -- "before issuing a flip
     * instruction using this function, make sure to confirm that the command
     * execution set by the corresponding cellGcmSetPrepareFlip() has been
     * completed".  PSL1GHT's wrapper pre-swaps bytes 0 and 2 to cancel the
     * hardware's own swap, so the word read back is the word written. */
    rsxSetWriteBackendLabel(s_ctx, RSX_LABEL_PREPARED,
                            ((u32)g_rsx.current << 8) | (u32)qid);
    rsx_flush_fenced(s_ctx);

    s_buf_busy[g_rsx.current] = 1u;     /* queued: not ours to draw into again */
    g_rsx.frames++;

    /* Choose the next buffer from state the flip interrupt maintains.  Returns
     * without sleeping whenever any buffer is free. */
    g_rsx.current = rsx_acquire_buffer();

    /* One line every ten seconds of presented frames, so a console run can see
     * the interrupts are actually firing and how often the loop had to wait. */
    if ((g_rsx.frames % 600u) == 0u)
        LOG_INFO(LOG_VIDEO,
                 "rsx: frames %llu flipirq %llu vblankirq %llu waits %llu "
                 "drops %llu timeouts %llu on_display %u",
                 (unsigned long long)g_rsx.frames,
                 (unsigned long long)s_flip_irqs,
                 (unsigned long long)s_vblank_irqs,
                 (unsigned long long)s_present_waits,
                 (unsigned long long)s_present_drops,
                 (unsigned long long)g_rsx.flip_timeouts,
                 (unsigned)s_buf_on_display);
}

/* ------------------------------------------------------------------ */
/* Diagnostics                                                          */
/* ------------------------------------------------------------------ */

int rsx_probe_alive(void)
{
    volatile u32 *label;
    const u32 want = 0x5A5A0001u;
    unsigned polls = 0;

    if (!g_rsx.inited)
        return 0;

    label = (volatile u32 *)gcmGetLabelAddress(RSX_LABEL_INDEX);
    if (!label)
        return 0;
    *label = 0;

    /* The back end writes the label only after it has actually retired the
     * command, so observing the value proves the RSX consumed our buffer. */
    rsxSetWriteBackendLabel(s_ctx, RSX_LABEL_INDEX, want);
    rsx_flush_fenced(s_ctx);

    while (*label != want && polls++ < 2000)
        usleep(200);

    return *label == want;
}

void rsx_fill_cpu(int buffer, u32 argb)
{
    u32 *p;
    u32  x, y;

    if (!g_rsx.inited || buffer < 0 || buffer >= RSX_BUFFERS)
        return;

    /* Written through the CPU's view of the framebuffer, so this exercises the
     * display engine alone: videoConfigure, gcmSetDisplayBuffer and scanout,
     * with the RSX not involved at all. If this shows and GPU rendering does
     * not, the fault is in command submission rather than the display. */
    p = g_rsx.buffer[buffer];
    for (y = 0; y < g_rsx.height; y++)
        for (x = 0; x < g_rsx.width; x++)
            p[y * (g_rsx.pitch / 4) + x] = argb;
}

/* ------------------------------------------------------------------ */
/* CPU text blitter: draw status text straight into the framebuffer,   */
/* with no GPU/shader involvement -- the same proven path as            */
/* rsx_fill_cpu. Used to show boot progress on the TV.                  */
/* ------------------------------------------------------------------ */

/* 8x8 status font (public-domain style, generated). Index = char-0x20;
 * one byte per row, LSB = leftmost column. Undefined glyphs are blank. */
static const unsigned char k_font8x8[96][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /*   */
    {0x04,0x04,0x04,0x04,0x00,0x04,0x00,0x00}, /* ! */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* " */
    {0x0A,0x3F,0x0A,0x3F,0x0A,0x00,0x00,0x00}, /* # */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* $ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* % */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* & */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ' */
    {0x08,0x04,0x04,0x04,0x04,0x08,0x00,0x00}, /* ( */
    {0x02,0x04,0x04,0x04,0x04,0x02,0x00,0x00}, /* ) */
    {0x00,0x15,0x0E,0x1F,0x0E,0x15,0x00,0x00}, /* * */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* + */
    {0x00,0x00,0x00,0x00,0x04,0x04,0x02,0x00}, /* , */
    {0x00,0x00,0x3F,0x00,0x00,0x00,0x00,0x00}, /* - */
    {0x00,0x00,0x00,0x00,0x04,0x04,0x00,0x00}, /* . */
    {0x10,0x08,0x04,0x04,0x02,0x01,0x00,0x00}, /* / */
    {0x1E,0x31,0x29,0x25,0x23,0x1E,0x00,0x00}, /* 0 */
    {0x04,0x06,0x04,0x04,0x04,0x0E,0x00,0x00}, /* 1 */
    {0x1E,0x21,0x10,0x0C,0x02,0x3F,0x00,0x00}, /* 2 */
    {0x1F,0x10,0x0C,0x10,0x21,0x1E,0x00,0x00}, /* 3 */
    {0x18,0x14,0x12,0x3F,0x10,0x10,0x00,0x00}, /* 4 */
    {0x3F,0x01,0x1F,0x10,0x21,0x1E,0x00,0x00}, /* 5 */
    {0x1E,0x01,0x1F,0x21,0x21,0x1E,0x00,0x00}, /* 6 */
    {0x3F,0x10,0x08,0x04,0x02,0x02,0x00,0x00}, /* 7 */
    {0x1E,0x21,0x1E,0x21,0x21,0x1E,0x00,0x00}, /* 8 */
    {0x1E,0x21,0x21,0x3E,0x10,0x1E,0x00,0x00}, /* 9 */
    {0x00,0x04,0x04,0x00,0x04,0x04,0x00,0x00}, /* : */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ; */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* < */
    {0x00,0x3F,0x00,0x3F,0x00,0x00,0x00,0x00}, /* = */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* > */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ? */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* @ */
    {0x0C,0x12,0x21,0x21,0x3F,0x21,0x21,0x00}, /* A */
    {0x1F,0x21,0x1F,0x21,0x21,0x1F,0x00,0x00}, /* B */
    {0x1E,0x21,0x01,0x01,0x21,0x1E,0x00,0x00}, /* C */
    {0x0F,0x11,0x21,0x21,0x11,0x0F,0x00,0x00}, /* D */
    {0x3F,0x01,0x1F,0x01,0x01,0x3F,0x00,0x00}, /* E */
    {0x3F,0x01,0x1F,0x01,0x01,0x01,0x00,0x00}, /* F */
    {0x1E,0x21,0x01,0x39,0x21,0x1E,0x00,0x00}, /* G */
    {0x21,0x21,0x3F,0x21,0x21,0x21,0x00,0x00}, /* H */
    {0x0E,0x04,0x04,0x04,0x04,0x0E,0x00,0x00}, /* I */
    {0x38,0x10,0x10,0x11,0x11,0x0E,0x00,0x00}, /* J */
    {0x11,0x09,0x07,0x09,0x11,0x21,0x00,0x00}, /* K */
    {0x01,0x01,0x01,0x01,0x01,0x3F,0x00,0x00}, /* L */
    {0x21,0x33,0x2D,0x21,0x21,0x21,0x00,0x00}, /* M */
    {0x21,0x23,0x25,0x29,0x31,0x21,0x00,0x00}, /* N */
    {0x1E,0x21,0x21,0x21,0x21,0x1E,0x00,0x00}, /* O */
    {0x1F,0x21,0x1F,0x01,0x01,0x01,0x00,0x00}, /* P */
    {0x1E,0x21,0x21,0x29,0x11,0x2E,0x00,0x00}, /* Q */
    {0x1F,0x21,0x1F,0x09,0x11,0x21,0x00,0x00}, /* R */
    {0x1E,0x01,0x1E,0x40,0x21,0x1E,0x00,0x00}, /* S */
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x00,0x00}, /* T */
    {0x21,0x21,0x21,0x21,0x21,0x1E,0x00,0x00}, /* U */
    {0x21,0x21,0x21,0x21,0x12,0x0C,0x00,0x00}, /* V */
    {0x21,0x21,0x21,0x2D,0x33,0x21,0x00,0x00}, /* W */
    {0x21,0x12,0x0C,0x0C,0x12,0x21,0x00,0x00}, /* X */
    {0x21,0x12,0x0C,0x04,0x04,0x04,0x00,0x00}, /* Y */
    {0x3F,0x10,0x08,0x04,0x02,0x3F,0x00,0x00}, /* Z */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* [ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* \ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ] */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ^ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* _ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ` */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* a */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* b */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* c */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* d */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* e */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* f */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* g */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* h */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* i */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* j */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* k */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* l */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* m */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* n */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* o */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* p */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* q */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* r */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* s */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* t */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* u */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* v */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* w */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* x */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* y */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* z */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* { */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* | */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* } */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ~ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /*   */
};

void rsx_draw_char(u32 *fb, int pitch, int x, int y, u32 color, int scale,
                   char c)
{
    const unsigned char *g;
    int row, col, sx, sy, stride = pitch / 4;
    unsigned uc = (unsigned char)c;

    if (uc >= 'a' && uc <= 'z') uc -= 32;    /* font is uppercase */
    if (uc < 0x20 || uc > 0x7E) return;
    if (scale < 1) scale = 1;
    g = k_font8x8[uc - 0x20];

    for (row = 0; row < 8; row++) {
        unsigned char bits = g[row];
        for (col = 0; col < 8; col++) {
            if (!(bits & (1u << col))) continue;   /* LSB = leftmost */
            for (sy = 0; sy < scale; sy++) {
                int py = y + row * scale + sy;
                if (py < 0 || py >= (int)g_rsx.height) continue;
                for (sx = 0; sx < scale; sx++) {
                    int px = x + col * scale + sx;
                    if (px < 0 || px >= (int)g_rsx.width) continue;
                    fb[py * stride + px] = color;
                }
            }
        }
    }
}

void rsx_draw_text_scaled(u32 *fb, int pitch, int x, int y, u32 color,
                          int scale, const char *s)
{
    int cx = x;
    if (scale < 1) scale = 1;
    for (; *s; s++) {
        if (*s == '\n') { y += 8 * scale + scale * 2; cx = x; continue; }
        rsx_draw_char(fb, pitch, cx, y, color, scale, *s);
        cx += 8 * scale;
    }
}

void rsx_draw_text(u32 *fb, int pitch, int x, int y, u32 color, const char *s)
{
    rsx_draw_text_scaled(fb, pitch, x, y, color, 2, s);
}


int rsx_present_cpu(int buffer)
{
    unsigned polls = 0;
    if (!g_rsx.inited)
        return 0;
    /* Boot/diagnostic path: a self-contained gcmSetFlip, still with its own
     * blocking wait because these callers want the picture on screen before
     * they return.  The flip interrupt fires for this flip too, so tell it
     * which buffer is arriving or it would retire the wrong ones. */
    s_buf_flipped = (u32)buffer;
    s_buf_busy[buffer] = 1u;
    if (gcmSetFlip(s_ctx, (u8)buffer) != 0)
        return 0;
    rsx_flush_fenced(s_ctx);
    gcmSetWaitFlip(s_ctx);
    while (gcmGetFlipStatus() != 0 && polls++ < 1000)
        usleep(200);
    if (polls >= 1000)
        return 0;
    gcmResetFlipStatus();
    /* Keep the frame loop's next choice consistent with what is now on screen;
     * the flip interrupt has already updated s_buf_on_display. */
    g_rsx.current = (int)((s_buf_on_display + 1u) % RSX_BUFFERS);
    return 1;
}

int rsx_wait_idle(void)
{
    volatile u32 *label;
    static u32 seq = 0x1000;
    u32 want = ++seq;
    unsigned polls = 0;

    if (!g_rsx.inited)
        return 0;

    label = (volatile u32 *)gcmGetLabelAddress(RSX_LABEL_INDEX);
    if (!label)
        return 0;
    *label = 0;

    rsxSetWriteBackendLabel(s_ctx, RSX_LABEL_INDEX, want);
    rsx_flush_fenced(s_ctx);

    /* PHASE PROFILE: asleep waiting for the RSX to drain the frame it was
     * given.  Non-zero here means GPU bound, not vsync bound. */
    while (*label != want && polls++ < 5000) {
        prof_enter(PH_WAITGPU);
        usleep(200);
        prof_exit();
    }
    return *label == want;
}

u32 rsx_count_differing(int buffer, u32 argb, u32 *sample_centre)
{
    const u32 *p;
    u32 x, y, stride, n = 0;

    if (!g_rsx.inited || buffer < 0 || buffer >= RSX_BUFFERS)
        return 0;

    /* Reading what the GPU wrote is the only way to know a draw produced
     * pixels. A frame can be submitted, accepted and flipped while rendering
     * nothing at all -- a shader that writes no output, geometry rejected by
     * the viewport transform, a state bit left wrong -- and every counter in
     * the command path still says success. Counting pixels that differ from
     * the clear colour turns "the GPU accepted it" into "the GPU drew it".
     *
     * Sampled on a grid rather than exhaustively: at 1920x1080 a full scan is
     * two million uncached reads over a bus tuned for the other direction, and
     * a triangle covering a fifth of the screen is not going to hide from a
     * grid this dense. */
    p      = g_rsx.buffer[buffer];
    stride = g_rsx.pitch / 4;

    for (y = 0; y < g_rsx.height; y += 4)
        for (x = 0; x < g_rsx.width; x += 4)
            if ((p[y * stride + x] & 0x00FFFFFFu) != (argb & 0x00FFFFFFu))
                n++;

    if (sample_centre)
        *sample_centre = p[(g_rsx.height / 2) * stride + (g_rsx.width / 2)];
    return n;
}

u32 rsx_sample(int buffer, float u, float v)
{
    u32 x, y;
    if (!g_rsx.inited || buffer < 0 || buffer >= RSX_BUFFERS)
        return 0;
    x = (u32)(u * (float)(g_rsx.width  - 1));
    y = (u32)(v * (float)(g_rsx.height - 1));
    return g_rsx.buffer[buffer][y * (g_rsx.pitch / 4) + x];
}

/* Command-stream capture. The command ring lives in our own host memory, so
 * the exact words handed to the GPU can be copied out and decoded offline --
 * which turns "guess which method is missing" into "read the stream". */
static u32 *s_cmd_mark;

void rsx_cmd_mark(void)
{
    s_cmd_mark = s_ctx ? s_ctx->current : NULL;
}

u32 rsx_cmd_since_mark(u32 *out, u32 max_words)
{
    u32 n = 0;
    const u32 *p;
    if (!s_ctx || !s_cmd_mark)
        return 0;
    /* If the ring wrapped through the callback, current is behind the mark and
     * the segment is not contiguous; report nothing rather than garbage. */
    if (s_ctx->current < s_cmd_mark)
        return 0;
    for (p = s_cmd_mark; p < s_ctx->current && n < max_words; p++)
        out[n++] = *p;
    return n;
}

/* Write the buffer most recently presented to a file, as a binary PPM.
 *
 * Graphics faults have been the one class that could not be diagnosed
 * remotely: everything else leaves evidence in the log, but "the picture is
 * wrong" needs the picture. Dumping the framebuffer makes rendering bugs as
 * inspectable as the rest, and removes the need to ask someone to describe
 * what they are looking at.
 *
 * Downscaled by `step` so a 1080p frame is a few hundred KB rather than 8 MB,
 * which matters when it is being pulled off the console over FTP. */
int g_shot_alpha;   /* write the ALPHA channel as grayscale instead of RGB */

int rsx_video_screenshot(const char *path, unsigned step, unsigned which)
{
    FILE *f;
    unsigned x, y, w, h;
    const u32 *src;
    unsigned char *row;

    if (!g_rsx.inited) return -1;
    if (step == 0) step = 1;
    /* Which buffer is on screen depends on where the flip chain happens to
     * be, so the caller names one rather than this guessing -- guessing gave a
     * uniformly blank frame the first time. `which` is an index; anything out
     * of range falls back to the one not being drawn into. */
    if (which < RSX_BUFFERS && g_rsx.buffer[which]) src = g_rsx.buffer[which];
    else src = g_rsx.buffer[g_rsx.last_queued % RSX_BUFFERS];
    if (!src) src = g_rsx.buffer[g_rsx.current];
    if (!src) return -1;

    w = g_rsx.width / step;
    h = g_rsx.height / step;
    f = fopen(path, "wb");
    if (!f) return -1;
    fprintf(f, "P6\n%u %u\n255\n", w, h);

    row = (unsigned char *)malloc((size_t)w * 3);
    if (!row) { fclose(f); return -1; }
    for (y = 0; y < h; y++) {
        const u32 *sl = src + (size_t)(y * step) * (g_rsx.pitch / 4);
        for (x = 0; x < w; x++) {
            u32 p = sl[x * step];               /* ARGB8 in memory */
            if (g_shot_alpha) {
                unsigned char a2 = (unsigned char)((p >> 24) & 0xFF);
                row[x * 3 + 0] = row[x * 3 + 1] = row[x * 3 + 2] = a2;
            } else {
            row[x * 3 + 0] = (unsigned char)((p >> 16) & 0xFF);
            row[x * 3 + 1] = (unsigned char)((p >> 8) & 0xFF);
            row[x * 3 + 2] = (unsigned char)(p & 0xFF);
            }
        }
        fwrite(row, 1, (size_t)w * 3, f);
    }
    free(row);
    fclose(f);
    return 0;
}

void rsx_video_shutdown(void)
{
    /* Deliberately does NOT tear the RSX context down. It cannot be brought
     * back (see rsx_video_init), and the only thing that follows a real
     * shutdown here is process exit, which releases it anyway. */
    LOG_INFO(LOG_VIDEO, "rsx: shutdown requested; keeping the context alive");
    return;
#if 0   /* retained for reference; see above */
    {
    int i;
    if (!g_rsx.inited)
        return;
    /* "The registered callback can be canceled if NULL is specified for
     * handler" -- cellGcmSetFlipHandler / cellGcmSetVBlankHandler.  Cancel them
     * before the buffers go away, or a VBlank a microsecond later issues a flip
     * to freed memory. */
    gcmSetVBlankHandler(NULL);
    gcmSetFlipHandler(NULL);
    s_handlers_up = 0;
    s_label_prepared = NULL;
    /* Unbind before freeing, and with the RSX idle, for the same reason the
     * binds needed an idle GPU: a tile region left pointing at memory the
     * allocator has handed to something else is corruption with no error
     * attached to it. This runs on the exit path (and before a relaunch),
     * which is exactly when the next process would inherit them. */
    if (g_rsx.tiled || g_rsx.zcull) {
        rsx_tile_wait_idle(s_ctx);
        if (g_rsx.zcull)
            gcmUnbindZcull((u8)ZCULL_REGION);
        if (g_rsx.tiled) {
            gcmUnbindTile((u8)TILE_REGION_DEPTH);
            for (i = 0; i < RSX_BUFFERS; i++)
                gcmUnbindTile((u8)(TILE_REGION_COLOR0 + (u32)i));
        }
    }
    for (i = 0; i < RSX_BUFFERS; i++)
        if (g_rsx.buffer[i]) rsxFree(g_rsx.buffer[i]);
    if (g_rsx.depth) rsxFree(g_rsx.depth);
    memset(&g_rsx, 0, sizeof g_rsx);
    }
#endif
}
