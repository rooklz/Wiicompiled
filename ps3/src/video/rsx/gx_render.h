/* gx_render.h — the GX sink that actually renders, on the RSX.
 *
 * This is where the two halves of the port meet. Everything above it is the
 * guest's world: a FIFO of GX commands, CP/XF/BP registers, vertices in the
 * Wii's quantised formats. Everything below it is the RSX: vertex programs,
 * fragment programs, attribute arrays and draw calls. The pieces on both sides
 * were verified separately without a console (the parser and decoders against
 * hand-computed values, the shader encoders against cgcomp); this joins them.
 *
 * The design follows from one measurement in docs/HARDWARE.md: the RSX loses
 * half its fragment throughput past two vec4 temporaries, so an "ubershader"
 * that interprets TEV state at run time is the wrong shape for this GPU.
 * Instead each distinct TEV/XF *configuration* compiles to a specialised
 * program, cached by a hash of exactly the state the program depends on --
 * which is why xf_state_hash deliberately excludes matrix values: those are
 * constants the program reads, and including them would recompile every time
 * an object moved.
 */
#ifndef DOLPHIN_VIDEO_RSX_GX_RENDER_H
#define DOLPHIN_VIDEO_RSX_GX_RENDER_H

#include "../../core/gx/gx.h"
#include "../../core/gx/gx_state.h"
#include "gx_features.h"

typedef struct {
    u64 draws;              /* draw commands received                */
    u64 vertices;           /* vertices decoded                      */
    u64 programs_built;     /* shader pairs generated                */
    u64 programs_failed;    /* pairs that would not fit: draws dropped */
    u64 program_hits;       /* cache hits                            */
    u64 skipped_no_pos;     /* draws with no position attribute      */
    u64 overflow;           /* draws dropped: vertex buffer exhausted */
    u64 textures_decoded;   /* distinct textures decoded and uploaded */
    /* Pipeline state the pixel engine asked for and this backend cannot
     * reproduce exactly. Counted rather than warned about, because a title
     * issues these thousands of times a frame and a log line per draw would
     * cost more than the draw. A non-zero value here is the thing to chase
     * when a frame is nearly right. */
    u64 alpha_test_unmapped;  /* two-sided compare that is not one RSX func */
    u64 cull_unmapped;        /* culling asked for while the group is off  */
    /* Draws that kill Zcull for the rest of the frame: depth writes enabled
     * with a depth function on the opposite side of the configured Zcull
     * direction (which is LESS, so GREATER / GEQUAL / NOTEQUAL / ALWAYS).
     * Once a region is invalidated it stays dead until the next clear
     * revalidates it, so a large number here means the Zcull unit is switched
     * on and doing nothing for most of the frame -- which is a performance
     * fact with no visible symptom whatsoever, and therefore has to be
     * counted or it will never be noticed. */
    u64 zcull_invalidating;
    /* The two GX features this backend still does not reproduce, counted so
     * "a race is nearly right" can be turned into a number. Indirect
     * texturing is used by 35 of the 396 course and kart materials on the
     * disc; the fog range adjustment is used by none of them (Mario Kart Wii
     * calls GXSetFogRangeAdj(0, 0, NULL) unconditionally), so a non-zero
     * count there means the measurement was wrong rather than incomplete. */
    u64 indirect_unmapped;    /* draws that configure an indirect stage    */
    u64 fog_range_unmapped;   /* draws asking for the x-axis range adjust  */
    /* Draws wanting more texture coordinates than the vertex carries. The
     * matrices and the generated code go to eight; the vertex stops at four,
     * which is the most any material on the disc asks for. A texgen past that
     * whose source is the normal or the geometry still works -- only one that
     * wants a fifth *coordinate* does not. */
    u64 texcoords_unmapped;

    /* Growth counters. A frame rate that decays rather than merely being low
     * is something getting bigger or slower every frame, and the only way to
     * tell which is to watch these *per interval* rather than in total: a
     * healthy steady state holds every one of them flat. They are read by the
     * platform's periodic report. */
    u64 program_evictions;  /* shader entries thrown out to make room  */
    u64 texture_evictions;  /* texture entries freed to make room      */
    u64 texture_undecodable;/* binds of a format we cannot decode      */
    u64 texture_admit_fail; /* binds that found no slot (drawn plain)  */
    u64 texture_bytes_total;/* cumulative RSX bytes handed to textures */
    u32 texture_bytes_live; /* RSX bytes currently held by textures    */
    u32 texture_entries;    /* occupied texture-cache slots            */
    u32 program_entries;    /* occupied shader-cache slots             */
    u32 retire_pending;     /* freed allocations still in quarantine   */

    /* Render to texture. `efb_copies_texture` counts every non-XFB copy the
     * title asks for; `efb_copies_resolved` counts the ones this backend
     * actually turned into a surface. A gap between them is a copy whose
     * destination format or geometry is not modelled, and it is exactly the
     * set of pixels a later draw will sample as garbage. */
    u64 efb_copies_xfb;      /* frame-end copies: GX rendered this frame */
    u64 efb_copies_texture;
    u64 efb_copies_resolved;
    u64 efb_bind_resized;   /* bind whose declared size != the copy's       */
} GXRenderStats;

extern GXRenderStats g_gx_render;

/* Refresh texture/shader cache occupancy in g_gx_render. */
void gx_render_sample_caches(void);

/* List live texture cache entries through a line callback. */
void gx_render_dump_textures(void (*out)(const char *));

/* One line naming everything that could grow, for a periodic console report.
 * Deltas are what matter, so the caller prints it every N frames and compares.
 * Returns `buf`. */
char *gx_render_stats_line(char *buf, unsigned len);

/* Runtime pipeline-state groups, steppable from the pad so one console session
 * can name a regression instead of one launch per guess. 0 = the known-good
 * baseline (write nothing at all). The bits themselves live in gx_features.h,
 * which the shader generators include without dragging in the RSX. */

void gx_render_debug_textures(u32 *fb, int pitch_words, int y0, int thumb,
                              int count);

/* Allocate the vertex arena and shader cache. rsx_video_init must have run. */
int  gx_render_init(void);
void gx_render_shutdown(void);

/* Fill in the backend so gx_state_init drives this renderer. The state tracker
 * owns the parser and applies every register write itself; the backend only
 * renders. */
void gx_render_bind(GXBackend *backend);

/* Called when the guest triggers an EFB copy to the external framebuffer --
 * the moment a title declares its frame finished. The platform presents on
 * it. */
void gx_render_set_frame_handler(void (*fn)(void *ctx), void *ctx);

/* Frame boundaries, so the backend can reset its per-frame vertex arena. */
void gx_render_frame_begin(void);
void gx_render_frame_end(void);


/* Census of the viewport/scissor rectangles and EFB copies actually used, so
 * "the world renders into a corner" can be attributed to a specific rectangle
 * rather than inferred from a sampled log. */
#define GX_VP_CENSUS_MAX 24
#define GX_CP_CENSUS_MAX 24

typedef struct {
    u16 vx, vy, vw, vh;
    u16 sx, sy, sw, sh;
    u32 uses;
    /* The guest's own numbers, before any scaling, so a degenerate rectangle
     * can be traced to what the title actually wrote rather than inferred. */
    float raw_sc[3], raw_off[3];
    /* Draws that actually rendered with this rectangle, split by projection.
     * `uses` counts rectangle CHANGES, which says how often the title
     * switches; it does not say where the world went. */
    u32 draws, draws_persp;
} GxVpCensus;

typedef struct {
    u16 w, h;
    u8  to_xfb, matched;
    u32 uses;
} GxCpCensus;

extern GxVpCensus g_vp_census[GX_VP_CENSUS_MAX];
extern unsigned   g_vp_census_n;
extern GxCpCensus g_cp_census[GX_CP_CENSUS_MAX];
extern unsigned   g_cp_census_n;

#endif
