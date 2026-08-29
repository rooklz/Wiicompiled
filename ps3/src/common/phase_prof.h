/* phase_prof.h — where the console's wall clock actually goes.
 *
 * Every speed decision in this project so far has assumed the emulated CPU is
 * the bottleneck. That is a hypothesis, not a measurement: a frame on the
 * console is guest execution *plus* the GX FIFO, plus our renderer's own CPU
 * work, plus flips, plus whatever time the PPE spends asleep waiting for the
 * RSX. Optimising the recompiler when a third of the frame is a vsync wait is
 * wasted effort, and the only instrument that can tell the two apart is a
 * per-phase wall-clock breakdown taken on the machine itself.
 *
 * This is that instrument. It is deliberately tiny and header-only:
 *
 *   - Timestamps come from `mftb`, the PPE's 79.8 MHz time base. 12.5 ns of
 *     resolution, a handful of cycles to read, no syscall, no clock_gettime,
 *     nothing that could itself perturb a measurement of microseconds.
 *
 *   - Times are *exclusive*. Regions nest -- gx_state_run calls the draw
 *     callback, which generates shaders, decodes vertices and emits RSX
 *     commands; the EFB copy inside it calls the frame handler, which waits on
 *     the GPU and flips. A profiler that reported inclusive time would say
 *     "gx_state_run: 90%" and answer nothing. So a stack of open regions is
 *     kept, and every transition charges the elapsed ticks to whichever region
 *     is innermost at that moment. The sum over all phases is then exactly the
 *     interval's wall clock, which the report checks and prints.
 *
 *   - The bottom of the stack is PH_OTHER, so time in the main loop that no
 *     region claims is still counted rather than silently lost. If PH_OTHER is
 *     large, the instrumentation is missing something and says so.
 *
 * Overhead: two `mftb`s and a couple of adds per region. The hottest use is
 * four regions per draw, a few hundred draws per frame -- order 10 us against
 * a 50 ms frame, i.e. 0.02%. Measuring finer (per vertex, per FIFO command)
 * would start to perturb what it measures, which is why it is not done.
 *
 * Everything is compiled out to nothing on a host that has no time base
 * (prof_tb() returns 0), and every entry point is a no-op until prof_reset()
 * arms it, so the pre-boot self-test stages are untouched.
 */
#ifndef DOLPHIN_COMMON_PHASE_PROF_H
#define DOLPHIN_COMMON_PHASE_PROF_H

#include "types.h"

/* ------------------------------------------------------------------ */
/* The phases                                                          */
/*                                                                     */
/* One line per thing a frame can be spending time in. The order here is the   */
/* order they appear in the report, which is roughly the order they happen in. */
/* ------------------------------------------------------------------ */
enum {
    PH_OTHER = 0,   /* main-loop bookkeeping nothing else claimed          */
    PH_JIT,         /* jit_run: emulated PowerPC execution                 */
    PH_TIMING,      /* timing_slice/advance + device updates (dsp/ai/vi/   */
                    /* ipc) + ios_bt_update (bluetooth/Wiimote)            */
    PH_GXFIFO,      /* gx_state_run: parsing the GX FIFO, minus the        */
                    /* renderer work it calls into                         */
    PH_GXSTATE,     /* gx_fifo, split out: the BP/CP/XF register traffic    */
                    /* the parser hands to the state model. 8,451 BP +      */
                    /* 2,762 CP + 1,873 XF writes a frame in a race, and     */
                    /* gx_fifo is 24% of the frame with the walk itself O(1) */
                    /* per draw -- so this is where to look first.           */
    PH_DRAWDISP,    /* the renderer draw callback, minus every inner phase  */
                    /* it enters (shader/tex/cmd/vertex). What is left is    */
                    /* per-draw setup: arena checks, state compares, census. */
    PH_SPUJOB,      /* building the SPU vertex job: format decode plus the   */
                    /* index-window scan. Split out of draw_disp because it   */
                    /* is the part that could move ONTO an idle SPU.          */
    PH_VTX,         /* renderer: vertex decode + transform maths           */
    PH_SPUWAIT,     /* ASLEEP: PPU blocked in spu_vtx_join waiting for the  */
                    /* SPU to finish decoding this draw's vertices. Was     */
                    /* previously charged to gx_fifo, which made FIFO       */
                    /* parsing look far more expensive than it is.          */
    PH_TEX,         /* renderer: texture decode + upload to RSX memory     */
    PH_SHADER,      /* renderer: shader cache lookup + program generation  */
    PH_CMD,         /* renderer: RSX command emission (state, binds, draw) */
    PH_PRESENT,     /* flip submission + surface setup, minus the wait     */
    PH_WAITFLIP,    /* ASLEEP: waiting for the flip to retire == vsync     */
    PH_WAITGPU,     /* ASLEEP: rsx_wait_idle, waiting for the RSX to drain */
    PH_OVERLAY,     /* the debug overlay: text, thumbnails, pixel counting */
    PH_PAD,         /* reading the PS3 pad (lv2 syscalls)                  */
    PH_BOOTUI,      /* pre-first-frame heartbeat screen (should be 0 once  */
                    /* the title is drawing)                               */
    PH_REPORT,      /* writing this report; excluded from the verdict      */
    PH_COUNT
};

#define PROF_STACK_MAX 32

typedef struct {
    u64 acc[PH_COUNT];      /* exclusive ticks, current interval */
    u64 cnt[PH_COUNT];      /* region entries,  current interval */
    u64 tot[PH_COUNT];      /* exclusive ticks, whole session    */
    u64 tot_cnt[PH_COUNT];
    u64 win_t0;             /* time base at interval start       */
    u64 last;               /* time base at the last transition  */
    int stack[PROF_STACK_MAX];
    int depth;
    int enabled;
    int legend_done;
} PhaseProf;

#ifdef PHASE_PROF_IMPL
PhaseProf g_prof;
#else
extern PhaseProf g_prof;
#endif

/* The PPE time base. 79.8 MHz on retail hardware, which is the constant the
 * rest of main.c already uses; reading it is a move-from-SPR, not a syscall. */
static inline u64 prof_tb(void)
{
#if defined(__powerpc64__) || defined(__PPC64__)
    u64 v;
    __asm__ __volatile__ ("mftb %0" : "=r"(v));
    return v;
#else
    return 0;
#endif
}

/* Charge everything since the last transition to whichever region is innermost,
 * then open a new one. */
static inline void prof_enter(int id)
{
    u64 now;
    int cur;
    if (!g_prof.enabled) return;
    now = prof_tb();
    cur = g_prof.stack[g_prof.depth];
    g_prof.acc[cur] += now - g_prof.last;
    g_prof.last = now;
    if (g_prof.depth + 1 < PROF_STACK_MAX)
        g_prof.stack[++g_prof.depth] = id;
    else
        g_prof.depth++;             /* keep pushes and pops balanced */
    g_prof.cnt[id]++;
}

static inline void prof_exit(void)
{
    u64 now;
    int cur;
    if (!g_prof.enabled) return;
    now = prof_tb();
    cur = g_prof.stack[g_prof.depth < PROF_STACK_MAX
                       ? g_prof.depth : PROF_STACK_MAX - 1];
    g_prof.acc[cur] += now - g_prof.last;
    g_prof.last = now;
    if (g_prof.depth > 0) g_prof.depth--;
}

/* Arm the profiler and start the first interval. Nothing above does anything
 * until this has run. */
static inline void prof_reset(void)
{
    int i;
    for (i = 0; i < PH_COUNT; i++) {
        g_prof.acc[i] = g_prof.cnt[i] = 0;
        g_prof.tot[i] = g_prof.tot_cnt[i] = 0;
    }
    g_prof.depth   = 0;
    g_prof.stack[0] = PH_OTHER;
    g_prof.last    = prof_tb();
    g_prof.win_t0  = g_prof.last;
    g_prof.enabled = 1;
    g_prof.legend_done = 0;
}

/* Fold the interval into the session totals and start a fresh interval. */
static inline void prof_window_close(void)
{
    int i;
    for (i = 0; i < PH_COUNT; i++) {
        g_prof.tot[i]     += g_prof.acc[i];
        g_prof.tot_cnt[i] += g_prof.cnt[i];
        g_prof.acc[i] = g_prof.cnt[i] = 0;
    }
    g_prof.win_t0 = prof_tb();
}

#ifdef PHASE_PROF_IMPL

#include <stdio.h>

static const char *const k_prof_name[PH_COUNT] = {
    "other/loop",   "jit_run",      "timing+dev",   "gx_fifo",
    "gx_state",     "draw_disp",    "spu_job",      "vertex",       "WAIT spu",     "texture",      "shader",
    "rsx_cmd",      "present",      "WAIT vsync",   "WAIT gpu",
    "overlay",      "pad",          "bootui",       "prof report"
};

static const char *const k_prof_what[PH_COUNT] = {
    "main loop bookkeeping, spin checks, everything unclaimed",
    "emulated PowerPC: the JIT, AOT bodies and interpreter fallbacks",
    "timing_slice/advance, device callbacks (dsp/ai/vi/ipc), bluetooth",
    "gx_state_run: walking the GX FIFO, minus the renderer below",
    "BP/CP/XF register traffic the parser hands to the state model",
    "per-draw setup in the renderer callback, minus its inner phases",
    "building the SPU vertex job: format decode + index-window scan",
    "renderer: vertex decode (vtx_decode) + projection/texmtx maths",
    "ASLEEP in spu_vtx_join: PPU waiting on the SPU vertex decoder",
    "renderer: GX texture decode + restripe + upload (cache misses)",
    "renderer: shader-cache lookup and vp/fp program generation",
    "renderer: writing RSX methods (pipeline state, binds, draw)",
    "flip submission, surface/viewport setup, per-frame clears",
    "ASLEEP waiting for the flip to retire -- this IS the vsync stall",
    "ASLEEP in rsx_wait_idle waiting for the RSX to finish the frame",
    "debug overlay: CPU text, texture thumbnails, framebuffer scan",
    "reading the PS3 pad through lv2 (once per presented frame)",
    "pre-first-frame heartbeat screen; must be zero once drawing",
    "formatting and writing this breakdown"
};

/* Print one breakdown.
 *
 * `out` is the report sink (main.c's emit_line), so this lands in the file on
 * the HDD, which is the console's only output channel.
 *
 * Everything is absolute: microseconds of wall clock per presented frame, and
 * the percentage of the frame that is. The last column is the one that decides
 * what to work on next -- the whole-emulator speedup that would result if this
 * phase became free, which is 1/(1-p). A phase at 10% is worth at most 1.11x
 * no matter how brilliantly it is optimised.
 */
/* Set by the caller from g_jit_stats.dispatch_lookups deltas. */
static u64 g_prof_dispatches;

static void prof_dump(void (*out)(void *, const char *), void *ctx,
                      double tb_hz, const char *label, int lifetime,
                      u64 frames, u64 insts, u64 draws, u64 verts)
{
    char b[200];
    const u64 *a  = lifetime ? g_prof.tot     : g_prof.acc;
    const u64 *nc = lifetime ? g_prof.tot_cnt : g_prof.cnt;
    u64 sum = 0, wall;
    double us_per_frame, secs;
    int i;

    if (frames == 0) frames = 1;
    for (i = 0; i < PH_COUNT; i++) sum += a[i];

    /* THE number for a CPU-bound emulator: how many PPE cycles we spend per
     * guest instruction. Everything else in this dump is a share of a frame;
     * this is an absolute efficiency figure that can be compared against what
     * the hardware should give.
     *
     * Reference points: the JIT emits 5.36 host instructions per guest one,
     * and the PPE is a 3.2 GHz in-order dual-issue core, so a well-behaved
     * JIT should land near 5-10 cycles per guest instruction. Anything far
     * above that is stalls -- load-hit-store (the PPE has NO store-to-load
     * forwarding and its LHS compare is partial, so unrelated addresses
     * false-alias), D-ERAT misses (64 entries x 4KB, ONE outstanding miss
     * chip-wide), or microcoded instructions (every variable shift), not
     * instruction count. */
    if (insts) {
        double jit_secs = (double)a[PH_JIT] / tb_hz;
        double cyc = jit_secs * 3.2e9 / (double)insts;
        snprintf(b, sizeof b,
                 "[PROF]   COST: %.1f PPE cycles per guest instruction "
                 "(%.1f M guest inst/s in jit_run)",
                 cyc, (double)insts / jit_secs / 1e6);
        out(ctx, b);
        /* Distinguish the two candidate explanations for a high cycle count:
         *   many instructions per dispatch  -> the EMITTED CODE is slow
         *   few  instructions per dispatch  -> DISPATCH overhead dominates
         * and give the per-dispatch cycle cost directly so it can be compared
         * against the ~17 host instructions a small block actually contains. */
        if (g_prof_dispatches) {
            double disp = (double)g_prof_dispatches;
            snprintf(b, sizeof b,
                     "[PROF]   COST: %.1f guest inst/dispatch, "
                     "%.0f cycles/dispatch (%llu dispatches)",
                     (double)insts / disp,
                     jit_secs * 3.2e9 / disp,
                     (unsigned long long)g_prof_dispatches);
            out(ctx, b);
        }
    }
    /* The interval's true wall clock, measured independently of the phase
     * accounting. Printing both is the self-check: if they disagree, the
     * accounting has a hole and no conclusion drawn from it is safe. */
    wall = lifetime ? sum : (prof_tb() - g_prof.win_t0);
    if (sum == 0) return;
    secs = (double)wall / tb_hz;

    if (!g_prof.legend_done) {
        g_prof.legend_done = 1;
        out(ctx, "");
        out(ctx, "=== PHASE PROFILE: where the console's wall clock goes ===");
        out(ctx, "  us/frame = microseconds of wall clock in this phase per");
        out(ctx, "    PRESENTED title frame.  Phases are EXCLUSIVE: nested time");
        out(ctx, "    is charged to the innermost phase, so the column sums to");
        out(ctx, "    the frame's whole wall clock.  'x-if-free' is the speedup");
        out(ctx, "    of the WHOLE emulator if this phase cost nothing, 1/(1-p):");
        out(ctx, "    that is the ceiling on any work spent optimising it.");
        out(ctx, "  WAIT rows are the PPE doing nothing at all -- asleep on the");
        out(ctx, "    RSX or on scanout.  If they are large the emulator is");
        out(ctx, "    GPU/vsync bound and no amount of CPU work will help.");
        for (i = 0; i < PH_COUNT; i++) {
            snprintf(b, sizeof b, "    %-11s %s", k_prof_name[i],
                     k_prof_what[i]);
            out(ctx, b);
        }
        out(ctx, "");
    }

    snprintf(b, sizeof b,
             "[PROF] %s: %llu frames in %d.%03d s = %d.%01d fps",
             label, (unsigned long long)frames,
             (int)secs, (int)(secs * 1000.0) % 1000,
             (int)((double)frames / (secs > 1e-6 ? secs : 1.0)),
             (int)((double)frames * 10.0 / (secs > 1e-6 ? secs : 1.0)) % 10);
    out(ctx, b);
    snprintf(b, sizeof b,
             "[PROF]   %llu guest insts (%d M/s), %llu draws, %llu verts"
             "  -> %llu insts, %llu draws per frame",
             (unsigned long long)insts,
             (int)((double)insts / (secs > 1e-6 ? secs : 1.0) / 1e6),
             (unsigned long long)draws, (unsigned long long)verts,
             (unsigned long long)(insts / frames),
             (unsigned long long)(draws / frames));
    out(ctx, b);
    out(ctx, "[PROF]   phase        us/frame   share   x-if-free   calls/frame");

    for (i = 0; i < PH_COUNT; i++) {
        unsigned pct10 = (unsigned)((a[i] * 1000ull + sum / 2) / sum);
        unsigned cpf10 = (unsigned)((nc[i] * 10ull) / frames);
        unsigned gain100;
        if (a[i] == 0 && nc[i] == 0) continue;
        us_per_frame = (double)a[i] * 1e6 / tb_hz / (double)frames;
        /* 1/(1-p), in hundredths. Capped so a phase that is the entire frame
         * prints a number rather than an infinity. */
        gain100 = (pct10 >= 999u) ? 99999u
                                  : (unsigned)(100000ull / (1000ull - pct10));
        snprintf(b, sizeof b,
                 "[PROF]   %-11s %8u  %3u.%01u%%    %2u.%02ux   %6u.%01u",
                 k_prof_name[i], (unsigned)(us_per_frame + 0.5),
                 pct10 / 10u, pct10 % 10u,
                 gain100 / 100u, gain100 % 100u,
                 cpf10 / 10u, cpf10 % 10u);
        out(ctx, b);
    }

    /* Accounted vs measured: these must agree to within the cost of the
     * transitions themselves. A gap means a phase is missing. */
    snprintf(b, sizeof b,
             "[PROF]   accounted %llu ticks, interval wall %llu ticks"
             " (%d.%02d%% covered)",
             (unsigned long long)sum, (unsigned long long)wall,
             (int)((double)sum * 100.0 / (double)(wall ? wall : 1)),
             (int)((double)sum * 10000.0 / (double)(wall ? wall : 1)) % 100);
    out(ctx, b);

    /* The question this whole exercise exists to answer, answered in one line
     * rather than left to be inferred from a table. */
    {
        u64 waits = a[PH_WAITFLIP] + a[PH_WAITGPU];
        unsigned wpct10 = (unsigned)((waits * 1000ull + sum / 2) / sum);
        double wus = (double)waits * 1e6 / tb_hz / (double)frames;
        snprintf(b, sizeof b,
                 "[PROF]   GPU/VSYNC STALL: %u us/frame = %u.%01u%% of the frame"
                 "  (vsync %u us, gpu-drain %u us)",
                 (unsigned)(wus + 0.5), wpct10 / 10u, wpct10 % 10u,
                 (unsigned)((double)a[PH_WAITFLIP] * 1e6 / tb_hz
                            / (double)frames + 0.5),
                 (unsigned)((double)a[PH_WAITGPU] * 1e6 / tb_hz
                            / (double)frames + 0.5));
        out(ctx, b);
        snprintf(b, sizeof b,
                 "[PROF]   VERDICT: %s",
                 wpct10 >= 200u
                   ? "GPU/VSYNC BOUND -- the PPE is idle for a fifth of the"
                     " frame or more"
                   : (a[PH_JIT] * 2ull > sum
                        ? "CPU BOUND on guest execution (jit_run > half)"
                        : "CPU BOUND, but NOT mostly in jit_run -- see the"
                          " table"));
        out(ctx, b);
    }
    {
        /* Everything that is not guest execution, together: the number that
         * says how much of a "make the recompiler faster" campaign is even
         * addressable. */
        u64 nonjit = sum - a[PH_JIT];
        unsigned npct10 = (unsigned)((nonjit * 1000ull + sum / 2) / sum);
        snprintf(b, sizeof b,
                 "[PROF]   non-jit_run total: %u.%01u%% of the frame"
                 "  -> an infinitely fast CPU core caps at %u.%02ux",
                 npct10 / 10u, npct10 % 10u,
                 (unsigned)(1000ull * 100ull / (npct10 ? npct10 : 1)) / 100u,
                 (unsigned)(1000ull * 100ull / (npct10 ? npct10 : 1)) % 100u);
        out(ctx, b);
    }
    out(ctx, "");
}

#endif /* PHASE_PROF_IMPL */

#endif /* DOLPHIN_COMMON_PHASE_PROF_H */
