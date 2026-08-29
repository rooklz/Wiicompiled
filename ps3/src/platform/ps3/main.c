/* main.c — PS3 entry point: recompiler self-test and benchmark.
 *
 * Answers the two questions no amount of workstation testing can:
 *
 *   1. Does the recompiled code actually *run* correctly on the PPE?
 *   2. How fast is it, really?
 *
 * Structured defensively, because the first attempt returned to the XMB
 * without leaving a trace. On a console there is no stderr, no debugger and no
 * exit status -- an unexplained return to the dashboard is the *only* symptom
 * available. So the report file is opened before anything else, and every
 * stage writes a breadcrumb that is flushed immediately. If the program dies,
 * the last line in the file names the stage it died in.
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include "../../core/ppc/difftest.h"
#include "../../core/ppc/realtest.h"
#include "../../core/ppc/jit/jit.h"
#include "../../core/ppc/jit/ppc_emitter.h"
#include "../../core/ppc/interp/interp.h"
#include "../../core/core_timing.h"
#include "../../core/mem/memmap.h"
#include "../../core/mem/mem_platform.h"
#include "../../video/rsx/rsx_video.h"
#include "../../video/rsx/xfb_present.h"
#include "../../video/rsx/rsx_tritest.h"
#include "../../video/rsx/gx_render.h"
#include "../../video/rsx/efb_copy.h"
#include "../../core/hw/hardware.h"
#include "../../core/disc/dol.h"
#include "../../../build/guest/gxtri_blob.h"
#include "../../core/ios/ios_hle.h"
#include <io/pad.h>
#include "../../core/disc/disc_image.h"
#include "../../core/gx/gx_state.h"
#include "../../core/hw/audio_out.h"
#include "../../core/hw/dsp_ax.h"

/* Real Mario Kart Wii boot data, embedded via mkwii_blobs.S (.incbin). */
#include "../../core/ios/wii_nand_defaults.h"
extern const unsigned char mkwii_dol_blob[],     mkwii_dol_blob_end[];
extern void aot_register_all(void);
extern const unsigned char mkwii_fst_blob[],     mkwii_fst_blob_end[];
extern const unsigned char mkwii_sysconf_blob[], mkwii_sysconf_blob_end[];
extern const unsigned char mkwii_setting_blob[], mkwii_setting_blob_end[];
#include "../../../build/guest/gxanim_blob.h"
#include "../../common/log.h"

/* ===================== PHASE PROFILE INSTRUMENTATION ======================
 * Everything between a "PHASE PROFILE" marker and its matching end exists to
 * answer one question: where does the console's wall clock actually go?  It
 * adds two `mftb`s per instrumented region and changes no behaviour.  Delete
 * every marked hunk and the emulator is exactly as it was.
 * The measurement itself, the phase list and the report format live in
 * src/common/phase_prof.h; this file only says which code is which phase.
 * ------------------------------------------------------------------------ */
#define PHASE_PROF_IMPL
#include "../../common/phase_prof.h"
/* =================== END PHASE PROFILE INSTRUMENTATION ==================== */

/* The console audio sink (platform/ps3/audio_ps3.c). Opening the port is
 * deliberately a separate, skippable step: g_audio_enable gates it, and every
 * failure path inside leaves the emulator running silently rather than
 * stopping. */
int  audio_ps3_init(void);
void audio_ps3_update(void);
void audio_ps3_shutdown(void);
void audio_ps3_stats(unsigned *blocks, unsigned *underruns, unsigned *queued);
extern int g_audio_enable;

#include <sys/process.h>
#include <lv2/process.h>
#include <sysutil/sysutil.h>
#include <net/net.h>
#include <sysmodule/sysmodule.h>
#include <sys/thread.h>
#include <sys/dbg.h>
#include <sys/memory.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include "../../core/difftrace.h"

/* Mandatory. This emits the .sys_proc_param section that lv2.ld places into
 * the PT_PROC_PARAM program header; without it that header is all zeros and
 * the lv2 loader refuses the process before a single instruction runs --
 * which presents as a silent, instant return to the XMB. Every PSL1GHT sample
 * carries this line; its absence here is why the first launches did nothing. */
SYS_PROCESS_PARAM(1001, 0x100000)

/* Tried in order; the first that opens wins. /dev_hdd0/tmp is the natural home
 * but is not guaranteed writable on every firmware, whereas the application's
 * own directory certainly is. */
static const char *const k_report_paths[] = {
    "/dev_hdd0/tmp/dolphin-ps3-selftest.txt",
    "/dev_hdd0/game/DOLPHIN01/USRDIR/selftest.txt",
    "/dev_usb000/dolphin-ps3-selftest.txt",
};

/* Reporting goes through lv2's file syscalls rather than stdio.
 *
 * newlib's stdio on this platform is a layer whose behaviour in a freshly
 * launched homebrew process is an assumption, not something verified -- and
 * when the symptom under investigation is "produced no output at all", the
 * logging path is the last thing that should be taken on trust. sysLv2FsWrite
 * is the syscall stdio would eventually reach anyway, with nothing in between
 * and no buffering to lose. */
static int s_fd = -1;

/* ------------------------------------------------------------------ */
/* XMB integration                                                      */
/*                                                                      */
/* PSL1GHT links libsysutil, but a title that never SERVICES the system
 * callback queue looks hung to lv2: choosing "Quit Game" leaves the XMB
 * waiting for an acknowledgement that never comes, and after about ten
 * seconds the system gives up, beeps and resets the console. Registering a
 * handler and polling it makes the exit immediate and clean -- and the same
 * queue carries the in-game XMB open/close events, so we can idle politely
 * while the user is in the menu instead of fighting it for the GPU. */
static volatile int s_exit_requested;
static volatile int s_relaunch_requested;
static unsigned s_bench_next, s_bench_step, s_bench_left;
static unsigned s_analyze_shot;
static void devlink_emit_line(const char *l);
static volatile u16 s_inject_buttons;   /* devlink-driven Wii Remote buttons */
static volatile unsigned s_inject_frames;
static u16 s_inj_hold; static int s_inj_active;
static float s_ptr_hold_x=0.5f, s_ptr_hold_y=0.5f; static int s_ptr_hold;
static unsigned s_sessions;
static volatile int s_xmb_menu_open;
static unsigned s_xfb_frames;      /* video-interface frames presented */

/* Width in pixels to read the video-interface framebuffer at. 640 is the
 * standard XFB width; the value is overridable so the correct one can be found
 * by sweeping and comparing against the reference rather than asserted. */
/* ON by default now, with a file to turn it OFF.
 *
 * This was off-unless-asked because an earlier PERMISSIVE rule ("present
 * whenever GX did not produce a frame-end copy") painted a YUV reinterpretation
 * over correctly rendered 3D. That rule is gone; what remains is the strict one
 * -- eight consecutive frames with no GX drawing at all, and the framebuffer
 * repointed at least twice -- which a rendered scene cannot produce, because a
 * race issues thousands of draws every frame.
 *
 * Leaving it disabled meant the title's videos never appeared at all, which is
 * a visible fault in its own right, and one nobody would hit the flag file to
 * discover. The conservative trigger is the protection; the flag was belt and
 * braces that cost the feature entirely. `dolphin-noxfb.txt` restores the old
 * behaviour if this ever misfires. */
static int xfb_present_enabled(void)
{
    static int on = -1;
    if (on < 0) {
        FILE *f = fopen("/dev_hdd0/tmp/dolphin-noxfb.txt", "r");
        on = 1;
        if (f) { fclose(f); on = 0; }
        LOG_INFO(LOG_VIDEO, "XFB video presentation %s",
                 on ? "enabled (strict trigger)" : "DISABLED by flag");
    }
    return on;
}

/* Arm the PI early-exit escape hatch. Read once at boot, beside the other
 * flag files, so a single build can be measured both ways on hardware. */
/* Read the slice ceiling from dolphin-slicecap.txt (a decimal count of guest
 * instructions). Absent or 0 keeps the original unbounded-by-cap behaviour. */
static void slice_cap_flag_init(void)
{
    extern s32 g_timing_slice_cap;
    FILE *f = fopen("/dev_hdd0/tmp/dolphin-slicecap.txt", "r");
    if (f) {
        long v = 0;
        if (fscanf(f, "%ld", &v) == 1 && v > 0 && v < 1000000L)
            g_timing_slice_cap = (s32)v;
        fclose(f);
    }
    LOG_INFO(LOG_CORE, "scheduler slice cap: %ld guest insts%s",
             (long)g_timing_slice_cap,
             g_timing_slice_cap ? "" : " (none, event-bound only)");
}

/* The vertex index-window scan runs on the SPU consumers.
 *
 * Measured in-race at matched load (6,242 vs 6,231 draws/frame), with the
 * format-decode cache in both: job build fell from 13,173 us/frame (8.2%,
 * 2.11 us/draw) to 3,829 (2.1%, 0.61 us/draw) -- 71% off, about 9.3 ms a
 * frame. WAIT spu stayed at 0.0%, so the extra DMA round trip the consumer
 * pays to read the indices costs the PPE nothing; it is spent out of the 95%
 * of the time an SPU was idle anyway. Verified in-race: geometry unchanged.
 *
 * dolphin-nospuscan.txt puts the scan back on the PPU. */
static void spu_scan_flag_init(void)
{
    extern int g_spu_scan_on;
    FILE *f = fopen("/dev_hdd0/tmp/dolphin-nospuscan.txt", "r");
    g_spu_scan_on = 1;
    if (f) { fclose(f); g_spu_scan_on = 0; }
    LOG_INFO(LOG_CORE, "SPU-side index scan %s",
             g_spu_scan_on ? "enabled" : "DISABLED by flag (PPU scans)");
}

/* Snapshot plumbing; see the run loop for why this cannot be read directly. */
int      g_snap_req, g_snap_ready;
/* Armed by dolphin-noni.txt: stop mirroring guest FPSCR[NI] to the host. */
static int g_ni_sync_off;
PPCState g_snap_state;

static void pi_irq_exit_flag_init(void)
{
    extern int g_pi_no_irq_exit;
    FILE *f = fopen("/dev_hdd0/tmp/dolphin-noirqexit.txt", "r");
    if (f) { fclose(f); g_pi_no_irq_exit = 1; }
    LOG_INFO(LOG_CORE, "IPC/interrupt early slice exit %s",
             g_pi_no_irq_exit ? "DISABLED by flag" : "enabled");
}

static unsigned xfb_width(void)
{
    /* Ask the video interface, which now models PICTURE_CONFIGURATION. The
     * file override stays for sweeping, but it is no longer how the correct
     * value is found: the title programs it and we read it. Not cached,
     * because a title may reconfigure the VI between scenes. */
    static int forced = -1;
    unsigned w;
    if (forced < 0) {
        FILE *f = fopen("/dev_hdd0/tmp/dolphin-xfbw.txt", "r");
        forced = 0;
        if (f) { if (fscanf(f, "%d", &forced) != 1) forced = 0; fclose(f); }
        if (forced && (forced < 320 || forced > 1024)) forced = 0;
    }
    if (forced) return (unsigned)forced;
    w = vi_xfb_width();
    return w ? w : 640u;
}
/* Inputs to the video-presentation trigger. File scope so a benchmark capture
 * can report WHY it did or did not fire: a video that never reaches the screen
 * leaves the guest's own clear colour showing, and MKWii clears to magenta --
 * which is exactly the "purple video" symptom. Without these the trigger is
 * invisible and the fault is indistinguishable from a colour-conversion bug. */
static unsigned s_xfb_quiet_frames, s_xfb_moves;
static unsigned s_efb_width, s_efb_height;
/* Which entry of the pipeline-state step list is live. Starts at the entry
 * that equals gx_features.c's default, so R2 and L2 move away from the shipped
 * build in both directions rather than jumping to it. */
/* The pad bisect is a DEBUG tool and starts disabled: the shipped default now
 * renders correctly on its own, and a selector whose index did not correspond
 * to the booted mask made the first trigger press jump to an unrelated state
 * set -- which is what put bars over everything. Enable with devlink
 * "bisect 1" when a bisect is actually wanted. */
static int s_gfx_bisect_on;
static unsigned s_gfx_step = 9;

/* The renderer asks whether the XMB owns the screen, so its flip wait can
 * yield instead of hanging behind the console's own overlay. */
int rsx_xmb_menu_open(void) { return s_xmb_menu_open; }

static void ps3_sysutil_cb(u64 status, u64 param, void *userdata)
{
    (void)param; (void)userdata;
    switch (status) {
    case SYSUTIL_EXIT_GAME:   s_exit_requested = 1; break;
    case SYSUTIL_MENU_OPEN:   s_xmb_menu_open  = 1; break;
    case SYSUTIL_MENU_CLOSE:  s_xmb_menu_open  = 0; break;
    default: break;
    }
}

/* ------------------------------------------------------------------ */
/* Live developer link                                                  */
/*                                                                      */
/* The old loop was: build, upload, ask a human to launch from the XMB,
 * wait for the session to end, fetch a log file over HTTP, guess, repeat.
 * That is one experiment per launch, with post-mortem evidence only, and it
 * is why several bugs took a launch each to find.
 *
 * This is a plain TCP listener on port 4000. Anything the emulator logs is
 * mirrored to the connected developer LIVE, and simple text commands come
 * back the other way, so a single launch supports many experiments and the
 * feature bisects that previously needed the pad can be driven remotely.  */
#define DEVLINK_PORT 4000
static int s_dev_listen = -1, s_dev_client = -1;

/* Non-zero once the statically recompiled game owns guest execution: the main
 * loop then drives the devices only and does not execute guest code itself. */
int g_wc_running;

/* Mirror one log line to the attached developer. netSend, not write: an lv2
 * socket is not a file descriptor. */
static void devlink_write(const char *buf, unsigned len)
{
    if (s_dev_client >= 0 && netSend(s_dev_client, buf, len, 0) < 0) {
        netClose(s_dev_client);
        s_dev_client = -1;
        log_set_mirror(NULL);
    }
}
static void emit_line(void *ctx, const char *line);
static void emitf(const char *fmt, ...);
static int  s_overlay_on;

static void boot_note(const char *what);   /* defined with the breadcrumb below */
static void gx_worker_drain(void);          /* defined with the GX worker below */
static void gx_worker_resume(void);

/* ------------------------------------------------------------------ */
/* Rescue listener                                                      */
/*                                                                      */
/* devlink is polled from the main loop, which is exactly the thing that  */
/* stops working when the emulator wedges -- and a wedged emulator cannot */
/* be relaunched, so unattended development ends until somebody walks to  */
/* the console. This second listener runs on its own thread and does the  */
/* one thing that always has to work: replace this process with the image */
/* on disk. It never touches emulator state, so nothing the emulator can  */
/* do to itself can jam it.                                              */
/*                                                                      */
/* Deliberately tiny: accept, read a line, act. Two commands only.       */
/* ------------------------------------------------------------------ */
#define RESCUE_PORT 4001

/* Where the main loop last was, and how many times it has gone round.
 *
 * A wedge is invisible from outside: the process is alive, the XMB is happy,
 * and every port except devlink answers. Knowing WHICH of the half-dozen
 * places the loop can block in it stopped at turns a night of guessing into
 * one question. Written by the main loop with plain stores (a torn read here
 * costs nothing -- it is a debugging hint, not state), read by the rescue
 * thread, which keeps running when the main loop does not. */
volatile unsigned g_wd_mark;      /* WD_* below */
volatile unsigned g_wd_seq;       /* bumped every main-loop iteration */

#define WD_LOOPTOP   1
#define WD_JIT       2
#define WD_GXFIFO    3
#define WD_TIMING    4
#define WD_PRESENT   5
#define WD_PAD       6
#define WD_REPORT    7

static const char *wd_name(unsigned m)
{
    switch (m) {
    case WD_LOOPTOP: return "loop-top";
    case WD_JIT:     return "jit_run";
    case WD_GXFIFO:  return "gx_state_run";
    case WD_TIMING:  return "timing/devices";
    case WD_PRESENT: return "present/flip";
    case WD_PAD:     return "pad";
    case WD_REPORT:  return "report";
    default:         return "?";
    }
}

/* The emulator (main) thread, recorded so the rescue thread can ask lv2 for
 * its register file when it stops advancing. */
static sys_ppu_thread_t g_emu_tid;

static void rescue_spawn(void)
{
    static const char *k_targets[2] = {
        "/dev_hdd0/game/DOLPHIN01/USRDIR/RELOAD.SELF",
        "/dev_hdd0/game/DOLPHIN01/USRDIR/EBOOT.BIN"
    };
    const char *self = k_targets[1];
    const char *argv[2];
    struct stat st;
    unsigned i;
    for (i = 0; i < 2; i++)
        if (stat(k_targets[i], &st) == 0) { self = k_targets[i]; break; }
    boot_note("rescue: spawning from disk");
    argv[0] = self; argv[1] = NULL;
    sysProcessExitSpawn2(self, argv, NULL, NULL, 0, 1001,
                         SYS_PROCESS_SPAWN_STACK_SIZE_1M);
    boot_note("rescue: spawn RETURNED (failed)");
}

/* The read runs on a throwaway thread, never on the listener.
 *
 * Learned the hard way: sysDbgReadPPUThreadContext BLOCKS when the target
 * thread is running, and calling it inline killed the rescue listener --
 * losing remote control of a console whose emulator was otherwise healthy.
 * The worker owns its buffer, so if it never returns it leaks one allocation
 * and the listener carries on. */
typedef struct {
    volatile int                 done;
    volatile s32                 rc;
    sys_ppu_thread_t             target;
    sys_dbg_ppu_thread_context_t ctx;
} CtxReq;

static void ctx_worker(void *arg)
{
    CtxReq *q = (CtxReq *)arg;
    q->rc = sysDbgReadPPUThreadContext(q->target, &q->ctx);
    q->done = 1;
    sysThreadExit(0);
}

static int rescue_ctx(char *out, unsigned cap)
{
    sys_dbg_ppu_thread_context_t *c;
    sys_ppu_thread_t w;
    CtxReq *q;
    unsigned k = 0, spin;

    if (!g_emu_tid)
        return snprintf(out, cap, "ctx: emulator thread id not recorded\n");

    q = (CtxReq *)calloc(1, sizeof *q);
    if (!q)
        return snprintf(out, cap, "ctx: out of memory\n");
    q->target = g_emu_tid;

    if (sysThreadCreate(&w, ctx_worker, q, 1000, 0x8000, 0, "ctxrd") != 0) {
        free(q);
        return snprintf(out, cap, "ctx: could not spawn reader thread\n");
    }
    for (spin = 0; spin < 200 && !q->done; spin++)
        usleep(10000);                      /* up to 2 s */
    if (!q->done)
        return snprintf(out, cap,
                        "ctx: read still blocked after 2 s -- the target is "
                        "RUNNING, not stopped (leaked one request)\n");
    if (q->rc != 0) {
        k = (unsigned)snprintf(out, cap, "ctx: read failed rc=%d\n", (int)q->rc);
        free(q);
        return (int)k;
    }

    c = &q->ctx;
    k += snprintf(out + k, cap - k, "ctx: pc=%08x lr=%08x ctr=%08x cr=%08x\n",
                  c->pc, c->lr, c->ctr, c->cr);
    k += snprintf(out + k, cap - k,
                  "ctx: r14(mem)=%016llx r15(state)=%016llx dcount=%d\n",
                  (unsigned long long)c->gpr[14],
                  (unsigned long long)c->gpr[15], (int)(s32)(u32)c->gpr[16]);
    k += snprintf(out + k, cap - k, "ctx: r10=%08x r11=%08x r12=%08x\n",
                  (u32)c->gpr[10], (u32)c->gpr[11], (u32)c->gpr[12]);
    {   unsigned g;
        for (g = 18; g <= 30; g += 4)
            k += snprintf(out + k, cap - k,
                          "ctx: r%u=%08x r%u=%08x r%u=%08x r%u=%08x\n",
                          g, (u32)c->gpr[g], g+1, (u32)c->gpr[g+1],
                          g+2, (u32)c->gpr[g+2], g+3, (u32)c->gpr[g+3]);
    }
    {   extern PPCState *g_live_cpu;
        u32 gpc = 0, gend = 0, off = 0;
        if (g_live_cpu && jit_block_at_host((void *)(u64)c->pc, &gpc, &gend, &off))
            k += snprintf(out + k, cap - k,
                          "ctx: in guest block %08x..%08x at host word %u\n",
                          gpc, gend, off);
        else
            k += snprintf(out + k, cap - k, "ctx: host pc outside code arena\n");
    }
    free(q);
    return (int)k;
}

/* A blocking listener on its own thread, deliberately independent of the run
 * loop: its whole purpose is to answer when the emulator does not. Commands are
 * one line per connection.
 *
 * Nothing here may block indefinitely -- see rescue_ctx, which spawns a worker
 * precisely so a stuck lv2 call cannot take the listener down with it. */
/* Zero-lag-ish canary watcher. The real zero-lag check lives in wc_irq_poll
 * (ppc_runtime.h), but that is inlined into all 13,675 translated units and
 * those only recompile on a cold build (wcbuild.sh compares .o vs .cpp mtime
 * only -- header edits do not propagate). Until the next cold build, this
 * thread polls the canary word every 50 us: the stomp lands within 1-2 guest
 * calls of the snapshot instead of the dozens a device-slice check allowed.
 * wc_canary_trip() is idempotent (state-gated), so racing the inline check
 * after a cold build is harmless. */
static void canary_watcher(void *arg)
{
    extern volatile const unsigned int *g_wc_canary_ptr;
    extern volatile int g_wc_canary_state;
    extern void wc_canary_trip(void);
    (void)arg;
    for (;;) {
        volatile const unsigned int *p = g_wc_canary_ptr;
        if (p && *p != 0x524D4345u)
            wc_canary_trip();
        if (g_wc_canary_state >= 2)
            break;              /* snapshot taken; the device slice reports */
        usleep(50);
    }
    sysThreadExit(0);
}

static void rescue_thread(void *arg)
{
    struct sockaddr_in a;
    int one = 1, ls;

    (void)arg;
    sysModuleLoad(SYSMODULE_NET);
    netInitialize();

    ls = netSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ls < 0) { sysThreadExit(0); return; }
    netSetSockOpt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    memset(&a, 0, sizeof a);
    a.sin_family      = AF_INET;
    a.sin_port        = htons(RESCUE_PORT);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (netBind(ls, (struct sockaddr *)&a, sizeof a) < 0 ||
        netListen(ls, 2) < 0) {
        netClose(ls);
        sysThreadExit(0);
        return;
    }

    for (;;) {
        char buf[64];
        int c = netAccept(ls, NULL, NULL);
        int n;
        if (c < 0) { usleep(200000); continue; }

        n = netRecv(c, buf, (int)sizeof buf - 1, 0);
        if (n > 0) {
            buf[n] = 0;
            if (!strncmp(buf, "relaunch", 8)) {
                netSend(c, "spawning\n", 9, 0);
                netClose(c);
                rescue_spawn();
                continue;
            }
            if (!strncmp(buf, "ping", 4)) {
                netSend(c, "alive\n", 6, 0);
            } else if (!strncmp(buf, "put ", 4)) {
                /* Raw file upload over the rescue socket, the deploy lane
                 * of last resort when webMAN FTP/HTTP dies under load.
                 * "put <path> <size>" then <size> bytes; temp+rename. */
                char path[128], tmp[136];
                unsigned long want = 0, got = 0;
                if (sscanf(buf + 4, "%127s %lu", path, &want) == 2 &&
                    want > 0 && want < 256UL * 1024UL * 1024UL) {
                    char tmpn[136];
                    FILE *pf;
                    snprintf(tmpn, sizeof tmpn, "%s.part", path);
                    pf = fopen(tmpn, "wb");
                    if (pf) {
                        static char pbuf[32768];
                        netSend(c, "GO\n", 3, 0);
                        while (got < want) {
                            unsigned long chunk = want - got;
                            int r;
                            if (chunk > sizeof pbuf) chunk = sizeof pbuf;
                            r = netRecv(c, pbuf, (int)chunk, 0);
                            if (r < 0) { usleep(2000); continue; }
                            if (r == 0) break;
                            fwrite(pbuf, 1, (size_t)r, pf);
                            got += (unsigned long)r;
                        }
                        fclose(pf);
                        if (got == want) {
                            remove(path);
                            rename(tmpn, path);
                            netSend(c, "OK\n", 3, 0);
                        } else {
                            remove(tmpn);
                            netSend(c, "SHORT\n", 6, 0);
                        }
                    } else netSend(c, "EOPEN\n", 6, 0);
                } else netSend(c, "EARGS\n", 6, 0);
            } else if (!strncmp(buf, "mq", 2) && (buf[2] == 0 || buf[2] == '\n' || buf[2] == '\r')) {
                /* Complete queue-traffic picture: per-thread last receive,
                 * per-thread last sleep, and the full send ring. */
                extern u32 g_rcv_thr[], g_rcv_q[], g_rcv_lr[];
                extern volatile unsigned g_rcv_cnt[];
                extern u32 g_slp_thr[], g_slp_q[], g_slp_lr[];
                extern volatile unsigned g_slp_cnt[];
                extern u32 g_snd_q[], g_snd_msg[], g_snd_lr[], g_snd_cur[];
                extern volatile unsigned g_snd_n;
                char out[1800];
                unsigned t, j, tot = g_snd_n;
                int k = snprintf(out, sizeof out, "recv (per thread):\n");
                for (t = 0; t < 8u && g_rcv_thr[t]; t++)
                    k += snprintf(out + k, sizeof out - (size_t)k,
                                  "  thr=%08x q=%08x lr=%08x n=%u\n",
                                  (unsigned)g_rcv_thr[t], (unsigned)g_rcv_q[t],
                                  (unsigned)g_rcv_lr[t], (unsigned)g_rcv_cnt[t]);
                k += snprintf(out + k, sizeof out - (size_t)k,
                              "sleep (per thread):\n");
                for (t = 0; t < 8u && g_slp_thr[t]; t++)
                    k += snprintf(out + k, sizeof out - (size_t)k,
                                  "  thr=%08x q=%08x lr=%08x n=%u\n",
                                  (unsigned)g_slp_thr[t], (unsigned)g_slp_q[t],
                                  (unsigned)g_slp_lr[t], (unsigned)g_slp_cnt[t]);
                k += snprintf(out + k, sizeof out - (size_t)k,
                              "sends total=%u:\n", tot);
                for (j = (tot > 16u ? 16u : tot); j >= 1; j--) {
                    unsigned ix = (tot - j) & 15u;
                    k += snprintf(out + k, sizeof out - (size_t)k,
                                  "  q=%08x msg=%08x lr=%08x cur=%08x\n",
                                  (unsigned)g_snd_q[ix], (unsigned)g_snd_msg[ix],
                                  (unsigned)g_snd_lr[ix], (unsigned)g_snd_cur[ix]);
                }
                netSend(c, out, k, 0);
            } else if (!strncmp(buf, "fbt ", 4)) {
                /* fbt N: backtrace of fiber N from its SAVED OSContext --
                 * srr0 (resume pc) plus the saved r1 back-chain. The live
                 * variant below covers only the running fiber. */
                extern unsigned wcf_fiber_osthread(int);
                int fi = atoi(buf + 4);
                unsigned os = wcf_fiber_osthread(fi);
                char out[512];
                int k;
                if (!os) { netSend(c, "no fiber\n", 9, 0); }
                else {
                    unsigned srr0 = mem_read32(os + 0x198);
                    unsigned lrs  = mem_read32(os + 0x84);
                    unsigned r1   = mem_read32(os + 4);
                    unsigned i;
                    k = snprintf(out, sizeof out,
                                 "f%d os=%08x srr0=%08x lr=%08x r1=%08x\n",
                                 fi, os, srr0, lrs, r1);
                    for (i = 0; i < 10 && r1 >= 0x80000000u && r1 < 0x94000000u; i++) {
                        unsigned fp = mem_read32(r1);
                        unsigned lr2 = fp ? mem_read32(fp + 4) : 0;
                        k += snprintf(out + k, sizeof out - (size_t)k,
                                      "  %08x\n", lr2);
                        if (!fp || fp <= r1) break;
                        r1 = fp;
                    }
                    netSend(c, out, k, 0);
                }
            } else if (!strncmp(buf, "fbt", 3)) {
                /* Fiber-build backtrace: walk the LIVE CpuContext's guest
                 * stack chain ([r1] = caller frame, [fp+4] = saved LR). */
                extern void *wcf_ctx_raw(void);
                void *fc = wcf_ctx_raw();
                char out[512];
                int k = 0;
                if (!fc) { netSend(c, "no ctx\n", 7, 0); }
                else {
                    unsigned r1, i;
                    extern unsigned wcf_ctx_gpr(int);
                    extern unsigned wcf_ctx_lr(void);
                    r1 = wcf_ctx_gpr(1);
                    k = snprintf(out, sizeof out, "fbt r1=%08x lr=%08x\n",
                                 r1, wcf_ctx_lr());
                    for (i = 0; i < 12 && r1 >= 0x80000000u && r1 < 0x94000000u; i++) {
                        unsigned fp = mem_read32(r1);
                        unsigned lr = fp ? mem_read32(fp + 4) : 0;
                        k += snprintf(out + k, sizeof out - (size_t)k,
                                      "  %08x\n", lr);
                        if (!fp || fp <= r1) break;
                        r1 = fp;
                    }
                    netSend(c, out, k, 0);
                }
            } else if (!strncmp(buf, "fibers", 6)) {
                extern int wcf_report(char *out, int cap);
                char out[900];
                int k = wcf_report(out, (int)sizeof out);
                netSend(c, out, k, 0);
            } else if (!strncmp(buf, "sleeps", 6)) {
                extern u32 g_sleep_q[], g_sleep_lr[], g_sleep_cur[];
                extern volatile unsigned g_sleep_n;
                char out[1024];
                unsigned tot = g_sleep_n, j;
                int k = snprintf(out, sizeof out, "sleeps total=%u\n", tot);
                for (j = (tot > 4u ? 4u : tot); j >= 1; j--) {
                    unsigned ix = (tot - j) & 15u;
                    k += snprintf(out + k, sizeof out - (size_t)k,
                                  "  recent q=%08x lr=%08x cur=%08x\n",
                                  (unsigned)g_sleep_q[ix],
                                  (unsigned)g_sleep_lr[ix],
                                  (unsigned)g_sleep_cur[ix]);
                }
                {   extern u32 g_slp_thr[], g_slp_q[], g_slp_lr[];
                    extern volatile unsigned g_slp_cnt[];
                    unsigned t;
                    k += snprintf(out + k, sizeof out - (size_t)k,
                                  "per-thread last sleep:\n");
                    for (t = 0; t < 8u && g_slp_thr[t]; t++)
                        k += snprintf(out + k, sizeof out - (size_t)k,
                                      "  thr=%08x q=%08x lr=%08x n=%u\n",
                                      (unsigned)g_slp_thr[t],
                                      (unsigned)g_slp_q[t],
                                      (unsigned)g_slp_lr[t],
                                      (unsigned)g_slp_cnt[t]);
                }
                netSend(c, out, k, 0);
            } else if (!strncmp(buf, "bt", 2)) {
                extern int wc_bt_report(int slot, char *out, int cap);
                char out[2048];
                int sl = (buf[2] == ' ') ? (int)strtol(buf + 3, NULL, 0) : -1;
                int k = wc_bt_report(sl, out, (int)sizeof out);
                netSend(c, out, k, 0);
            } else if (!strncmp(buf, "regs ", 5)) {
                extern int wc_regs_report(int slot, char *out, int cap);
                char out[1024];
                int k = wc_regs_report((int)strtol(buf + 5, NULL, 0),
                                       out, (int)sizeof out);
                netSend(c, out, k, 0);
            } else if (!strncmp(buf, "dump ", 5)) {
                /* LIVE GUEST MEMORY, from the listener that actually works.
                 * devlink (port 4000) has the same command but its netAccept
                 * never fires on this stack, so every "what is that pointer?"
                 * question cost a full build+deploy+boot to answer with a
                 * printf. Reading it live turns that 18 s cycle into 2 s and
                 * removes the rebuild entirely.
                 *   dump <hexaddr> [words]      e.g. dump 80381c48 8        */
                char *endp = NULL;
                u32 addr = (u32)strtoul(buf + 5, &endp, 16);
                unsigned words = endp ? (unsigned)strtoul(endp, NULL, 0) : 0;
                unsigned w;
                char out[1024];
                int k = 0;
                if (words == 0 || words > 64) words = 16;
                for (w = 0; w < words && k < (int)sizeof out - 96; w += 4)
                    k += snprintf(out + k, sizeof out - (size_t)k,
                                  "%08x: %08x %08x %08x %08x\n",
                                  addr + w * 4,
                                  (unsigned)mem_read32(addr + w * 4),
                                  (unsigned)mem_read32(addr + w * 4 + 4),
                                  (unsigned)mem_read32(addr + w * 4 + 8),
                                  (unsigned)mem_read32(addr + w * 4 + 12));
                netSend(c, out, k, 0);
            } else if (!strncmp(buf, "deref ", 6)) {
                /* Chase a pointer chain: "deref <base> <off1> <off2> ..."
                 * prints each hop. The stuck vtable call needs exactly this
                 * (object -> +84 -> vtable -> +40) and hand-dumping each hop
                 * costs three round trips. */
                char *p2 = buf + 6, *e2 = NULL;
                u32 v = (u32)strtoul(p2, &e2, 16);
                char out[1024];
                int k = snprintf(out, sizeof out, "base %08x\n", v);
                p2 = e2;
                while (p2 && *p2 && k < (int)sizeof out - 64) {
                    long off = strtol(p2, &e2, 0);
                    if (e2 == p2) break;
                    p2 = e2;
                    v = (u32)mem_read32((u32)((s32)v + (s32)off));
                    k += snprintf(out + k, sizeof out - (size_t)k,
                                  "  +%ld -> %08x\n", off, v);
                }
                netSend(c, out, k, 0);
            } else if (!strncmp(buf, "trail", 5)) {
                /* The last guest calls, on demand.
                 *
                 * The watchdog prints this when the game STOPS, which does not
                 * cover the more common case: a game that is running hard and
                 * getting nowhere. A loop that keeps calling functions looks
                 * perfectly healthy to a progress counter, so the only way to
                 * see what it is looping on is to ask while it runs. */
                extern volatile unsigned g_wc_calls;
                extern uint32_t          g_wc_crumb[];
                char out[1024];
                unsigned calls = g_wc_calls;
                unsigned n = calls < 64u ? calls : 64u, k2;
                int used = snprintf(out, sizeof out, "calls=%u last=%u\n",
                                    calls, n);
                for (k2 = 0; k2 < n; k2++) {
                    unsigned idx = (calls - n + k2) & 63u;
                    used += snprintf(out + used, sizeof out - (size_t)used,
                                     "%08x%s", (unsigned)g_wc_crumb[idx],
                                     (k2 % 8) == 7 ? "\n" : " ");
                }
                if (n % 8) used += snprintf(out + used, sizeof out - (size_t)used, "\n");
                {   /* Interrupt state, because a port that never receives one
                     * looks exactly like a port whose handler does nothing. */
                    extern volatile int      g_wc_irq_pending;
                    extern volatile unsigned g_wc_irq_raised, g_wc_irq_delivered;
                    extern u32 pi_intsr_raw(void), pi_intmr_raw(void);
                    extern u32 wc_current_msr(void);
                    used += snprintf(out + used, sizeof out - (size_t)used,
                                     "irq pending=%d raised=%u delivered=%u "
                                     "intsr=%08x intmr=%08x curctx=%08x msr=%08x\n",
                                     g_wc_irq_pending, g_wc_irq_raised,
                                     g_wc_irq_delivered,
                                     (unsigned)pi_intsr_raw(),
                                     (unsigned)pi_intmr_raw(),
                                     (unsigned)mem_read32(0x800000D4u),
                                     (unsigned)wc_current_msr());
                {   extern volatile unsigned g_wc_dec_delivered;
                    extern u64 wc_dec_expiry_pub(void);
                    extern volatile int g_wc_irq_owner;
                    extern volatile unsigned g_wc_irq_excnum, g_wc_irq_cause;
                    u32 tbl = mem_read32(0x803824E8u);
                    used += snprintf(out + used, sizeof out - (size_t)used,
                                     "dec delivered=%u armed=%llu tbl=%08x h8=%08x "
                                     "irqowner=%d exc=%u cause=%08x\n",
                                     g_wc_dec_delivered,
                                     (unsigned long long)wc_dec_expiry_pub(),
                                     tbl, tbl ? mem_read32(tbl + 32u) : 0u,
                                     g_wc_irq_owner, g_wc_irq_excnum,
                                     g_wc_irq_cause);
                {   extern volatile uint16_t g_wc_where[16];
                    extern volatile unsigned g_wc_where_n;
                    unsigned wn = g_wc_where_n, k3;
                    used += snprintf(out + used, sizeof out - (size_t)used, "where:");
                    for (k3 = 0; k3 < 16u; k3++)
                        used += snprintf(out + used, sizeof out - (size_t)used,
                                         " %u", g_wc_where[(wn + k3) & 15u]);
                    used += snprintf(out + used, sizeof out - (size_t)used, "\n");
                }
                }
                }
                netSend(c, out, used, 0);
            } else if (!strncmp(buf, "sched", 5)) {
                extern int wc_sched_report(char *out, int cap);
                char out[900];
                int k2 = wc_sched_report(out, sizeof out);
                if (k2 > 0) netSend(c, out, k2, 0);
            } else if (!strncmp(buf, "ctx", 3)) {
                char out[900];
                int k = rescue_ctx(out, sizeof out);
                if (k > 0) netSend(c, out, k, 0);
            } else if (!strncmp(buf, "stat", 4)) {
                extern u64 g_mkw_frames_pub;
                extern int g_wc_running;
                char out[220];
                int k;
                if (g_wc_running) {
                    /* In port mode the run-loop counters describe the device
                     * loop, not the game. The game's own progress is the call
                     * heartbeat, which is the only thing that distinguishes a
                     * spinning guest from a stopped one. */
                    extern volatile unsigned g_wc_calls;
                    k = snprintf(out, sizeof out,
                                 "PORT mark=%u(%s) seq=%u frames=%llu calls=%u\n",
                                 g_wd_mark, wd_name(g_wd_mark), g_wd_seq,
                                 (unsigned long long)g_mkw_frames_pub,
                                 g_wc_calls);
                } else {
                    k = snprintf(out, sizeof out,
                                 "mark=%u(%s) seq=%u frames=%llu\n",
                                 g_wd_mark, wd_name(g_wd_mark), g_wd_seq,
                                 (unsigned long long)g_mkw_frames_pub);
                }
                if (k > 0) netSend(c, out, k, 0);
            }
        }
        netClose(c);
    }
}

static void rescue_init(void)
{
    sys_ppu_thread_t t;
    if (sysThreadCreate(&t, rescue_thread, NULL, 1200, 0x4000, 0, "rescue") == 0)
        boot_note("rescue listener up (port 4001)");
    else
        boot_note("rescue listener FAILED");
}

static void devlink_init(void)
{
    struct sockaddr_in a;
    int one = 1;
    {
        int rc;
        /* PSL1GHT needs the network module loaded before netInitialize; the
         * silent failure here is why the listener never came up. */
        /* rescue_init() has usually done both already; netInitialize is
         * idempotent enough to call twice, and a negative return here just
         * means it was already up. Do not bail on it. */
        sysModuleLoad(SYSMODULE_NET);
        rc = netInitialize();
        if (rc < 0) emitf("devlink: netInitialize returned %d (already up?)", rc);
    }
    s_dev_listen = netSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_dev_listen < 0) return;
    netSetSockOpt(s_dev_listen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port   = htons(DEVLINK_PORT);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    {   int nb = 1;
        netSetSockOpt(s_dev_listen, SOL_SOCKET, SO_NBIO, &nb, sizeof nb);
    }
    if (netBind(s_dev_listen, (struct sockaddr *)&a, sizeof a) < 0 ||
        netListen(s_dev_listen, 1) < 0) {
        netClose(s_dev_listen); s_dev_listen = -1; return;
    }
    emitf("devlink: listening on port %d", DEVLINK_PORT);
}

/* Called from the run loop; never blocks. */
/* Tentative definitions so the devlink handler, which appears above them, can
 * arm the benchmark. The real definitions follow with the bench code. */
static unsigned long long s_mkw_frames;
static u64 s_bench_at, s_bench_len;
static int s_bench_armed, s_bench_running, s_bench_done;

static void devlink_poll(void)
{
    char cmd[128];
    int n;
    if (s_dev_listen < 0) return;

    {   /* Always try to accept, even while a client is held, and let a NEW
         * connection displace the old one.
         *
         * The old code only accepted when no client was attached, and treated
         * every negative netRecv as "nothing pending" -- but a non-blocking
         * socket returns -1 for a genuine error (a reset peer) exactly as it
         * does for EWOULDBLOCK. A client that vanished without a clean FIN,
         * which is what a timed-out script leaves behind, therefore stayed
         * held forever: recv failed silently on every poll, the slot was never
         * released, and no new connection was ever accepted. From outside that
         * looks identical to a wedged emulator, and it cost several recovery
         * cycles before the difference was noticed (the game kept rendering
         * throughout).
         *
         * Accepting unconditionally makes reconnection always possible, which
         * is the property that matters for unattended work: whatever state the
         * previous connection died in, the next one gets in.
         *
         * Accept directly on the non-blocking listener. netSelect never
         * reported the socket readable on this stack, so a connection was
         * accepted into the backlog and then never taken -- the developer saw
         * a timeout while the log claimed the listener was up. */
        int c = netAccept(s_dev_listen, NULL, NULL);
        if (c >= 0) {
            int nb = 1;
            if (s_dev_client >= 0) {        /* displace the stale one */
                log_set_mirror(NULL);
                netClose(s_dev_client);
            }
            s_dev_client = c;
            netSetSockOpt(c, SOL_SOCKET, SO_NBIO, &nb, sizeof nb);
            log_set_mirror(devlink_write);
            emit_line(NULL, "devlink: developer attached");
        }
        if (s_dev_client < 0)
            return;
    }

    {   /* Commands, one per line, on a non-blocking socket. */
        n = netRecv(s_dev_client, cmd, sizeof cmd - 1, 0);
        if (n < 0) return;                  /* nothing pending */
        if (n == 0) {                       /* developer went away */
            log_set_mirror(NULL);
            netClose(s_dev_client); s_dev_client = -1;
            return;
        }
        cmd[n] = 0;
        while (n > 0 && (cmd[n-1] == '\n' || cmd[n-1] == '\r')) cmd[--n] = 0;

        if (!strncmp(cmd, "mask ", 5)) {
            g_gx_state_mask = (unsigned)strtoul(cmd + 5, NULL, 0);
            emitf("devlink: gx state mask -> 0x%x", g_gx_state_mask);
        } else if (!strcmp(cmd, "quit")) {
            emit_line(NULL, "devlink: quit requested");
            s_exit_requested = 1;
        } else if (!strcmp(cmd, "relaunch")) {
            /* Restart the emulator without anyone touching the console: the
             * whole point of unattended development. The new EBOOT is picked
             * up because the process is replaced from disk. */
            emit_line(NULL, "devlink: relaunching");
            s_relaunch_requested = 1;
            s_exit_requested = 1;
        } else if (!strncmp(cmd, "aot ", 4)) {
            /* Live A/B between the ahead-of-time recompiled bodies and the
             * JIT. The qemu harness compiles no AOT at all, so the console
             * runs guest code that emulation-side testing never exercises --
             * this is how that difference gets measured rather than argued
             * about. Both directions flush, so the switch is honest. */
            if (strtoul(cmd + 4, NULL, 0)) { jit_aot_enable_all();
                emit_line(NULL, "devlink: AOT enabled"); }
            else { jit_aot_disable();
                emit_line(NULL, "devlink: AOT disabled"); }
        } else if (!strncmp(cmd, "dump ", 5)) {
            /* Read guest memory live. The title takes one branch under qemu
             * and the other on hardware from an identical call chain, so the
             * deciding code has to be read rather than guessed at. */
            char *endp = NULL;
            u32 addr = (u32)strtoul(cmd + 5, &endp, 16);
            unsigned words = endp ? (unsigned)strtoul(endp, NULL, 0) : 0;
            unsigned w;
            if (words == 0 || words > 64) words = 16;
            for (w = 0; w < words; w += 4)
                emitf("  %08x: %08x %08x %08x %08x", addr + w * 4,
                      mem_read32(addr + w * 4), mem_read32(addr + w * 4 + 4),
                      mem_read32(addr + w * 4 + 8),
                      mem_read32(addr + w * 4 + 12));
        } else if (!strncmp(cmd, "find ", 5)) {
            /* Scan guest RAM for a byte pattern. The title refuses the link
             * after failing a 6-byte BD_ADDR compare (guest 0x801cf648), so
             * the question is whether our remote's address is anywhere in the
             * table it consults -- which a scan answers directly. */
            u8 pat[16]; unsigned np = 0; const char *q = cmd + 5;
            u32 a2; unsigned hits = 0;
            while (np < sizeof pat && q[0] && q[1]) {
                char t[3]; t[0]=q[0]; t[1]=q[1]; t[2]=0;
                pat[np++] = (u8)strtoul(t, NULL, 16);
                q += 2; while (*q == ' ') q++;
            }
            if (np) {
                for (a2 = 0x80000000u; a2 < 0x817ffff0u; a2++) {
                    unsigned k = 0;
                    while (k < np && mem_read8(a2 + k) == pat[k]) k++;
                    if (k == np) {
                        emitf("  found at %08x", a2);
                        if (++hits >= 24) break;
                    }
                }
            }
            emitf("  find: %u byte pattern, %u hit(s)", np, hits);
        } else if (!strncmp(cmd, "shot", 4) &&
                   (cmd[4] == ' ' || cmd[4] == '\0')) {
            /* Writing the framebuffer to the hard disk is done from the
             * emulation thread, and it is slow enough that doing it often
             * stalls the loop until devlink itself stops answering -- which,
             * with no way to start a title remotely, costs a manual relaunch.
             * That has happened twice. So: ONE buffer per request, a coarse
             * default step, and a hard rate limit no caller can override. */
            /* Rate-limited by PRESENTED FRAMES rather than wall clock: it
             * needs no timebase intrinsic and it throttles by exactly the
             * thing that matters, which is how often the frame loop is asked
             * to stop and write to disk. */
            static u64 last_shot_frame;
            extern u64 g_mkw_frames_pub;
            u64 now_tb = g_mkw_frames_pub ? g_mkw_frames_pub : g_rsx.frames;
            unsigned step = (unsigned)strtoul(cmd + 4, NULL, 0);
            /* The floor exists because a 1080p framebuffer is 8 MB and
             * writing that from the frame loop wedges the emulator. At 480p a
             * full-resolution capture is 1 MB, which the loop absorbs, so the
             * floor scales with the surface rather than being a flat rule that
             * makes native-resolution output impossible to inspect. */
            unsigned px = g_rsx.width * g_rsx.height;
            unsigned floor_step = (px > 1280u * 720u) ? 4u
                                : (px > 720u * 576u)  ? 2u : 1u;
            if (!step) step = floor_step;
            if (step < floor_step) step = floor_step;
            int explicit_step = (cmd[5] >= '0' && cmd[5] <= '9');
            if (!explicit_step && last_shot_frame &&
                now_tb - last_shot_frame < 8ull) {
                /* 8 flips, not 45: in-race the game can sit many wall-seconds
                 * between flips, and a 45-flip window locked screenshots out
                 * for minutes exactly when they matter most. 8 flips still
                 * prevents the disk-write-per-frame wedge. */
                emit_line(NULL, "devlink: screenshot rate-limited (8 frames)");
            } else {
                unsigned front =
                    (unsigned)((g_rsx.current + RSX_BUFFERS - 1) % RSX_BUFFERS);
                char sp[64];
                int rc;
                /* Unique name per capture. A fixed name plus FTP DELE raced
                 * often enough that stale files were analysed as fresh -- a
                 * class of false result that wasted whole investigations. */
                snprintf(sp, sizeof sp, "/dev_hdd0/tmp/shot-%llu.ppm",
                         (unsigned long long)g_rsx.frames);
                (void)front;
                rc = rsx_video_screenshot(sp, step, 255u); /* last_queued */
                last_shot_frame = now_tb;
                emitf("devlink: screenshot %s rc=%d", sp, rc);
            }
        } else if (!strncmp(cmd, "point ", 6)) {
            /* Position the IR cursor at (x,y) in [0,1] and hold it there, so
             * pointer-driven menus (track/character select) can be automated.
             * s_ptr_hold is re-asserted every frame in the pad block; without
             * it the pointer would only apply on frames a pad is read. */
            float x=0.5f, y=0.5f;
            sscanf(cmd + 6, "%f %f", &x, &y);
            s_ptr_hold_x = x; s_ptr_hold_y = y; s_ptr_hold = 1;
            ios_bt_set_pointer(x, y);
            emitf("devlink: pointer -> %d,%d %%", (int)(x*100), (int)(y*100));
        } else if (!strncmp(cmd, "click ", 6)) {
            /* Move to (x,y), settle, then A -- one command to hit a menu item. */
            float x=0.5f, y=0.5f;
            sscanf(cmd + 6, "%f %f", &x, &y);
            s_ptr_hold_x = x; s_ptr_hold_y = y; s_ptr_hold = 1;
            ios_bt_set_pointer(x, y);
            s_inject_buttons = 0x0008; s_inject_frames = 10;
            emitf("devlink: click %d,%d %%", (int)(x*100), (int)(y*100));
        } else if (!strncmp(cmd, "btn ", 4)) {
            /* Inject a Wii Remote button mask for a moment, so the emulator can
             * be driven from here instead of needing someone at the console.
             * Wii core buttons: A=0x0008, B=0x0004, 1=0x0002, 2=0x0001,
             * +=0x1000, HOME=0x0080, dpad up/down/left/right = 0x0800/0x0400/
             * 0x0100/0x0200. */
            unsigned m = (unsigned)strtoul(cmd + 4, NULL, 0);
            s_inject_buttons = (u16)m;
            s_inject_frames  = 12;      /* held long enough to register */
            emitf("devlink: injecting buttons %04x for %u frames",
                  m, s_inject_frames);
        } else if (!strncmp(cmd, "bench ", 6)) {
            unsigned a1 = 0, st = 60, n = 8;
            sscanf(cmd + 6, "%u %u %u", &a1, &st, &n);
            /* Hard floor on the spacing. Captures write a framebuffer to the
             * hard disk from inside the frame loop, and asking for them back
             * to back wedged the emulator badly enough that even devlink
             * stopped answering -- with no way to restart a title remotely,
             * that costs a manual relaunch. Thirty frames is half a second
             * between writes, which the loop absorbs comfortably. */
            if (st < 30) st = 30;
            if (!n || n > 32) n = 8;
            /* Never start behind the current frame, for the same reason. */
            if (a1 < (unsigned)g_rsx.frames) a1 = (unsigned)g_rsx.frames + st;
            s_bench_next = a1; s_bench_step = st; s_bench_left = n;
            remove("/dev_hdd0/tmp/bench.txt");
            emitf("devlink: bench from frame %u step %u count %u", a1, st, n);
        } else if (!strncmp(cmd, "bisect ", 7)) {
            s_gfx_bisect_on = (int)strtoul(cmd + 7, NULL, 0);
            emitf("devlink: pad gfx bisect %s",
                  s_gfx_bisect_on ? "ENABLED" : "disabled");
        } else if (!strncmp(cmd, "dwin ", 5)) {
            extern u32 g_draw_win_min, g_draw_win_max;
            unsigned lo = 0, hi = 0;
            sscanf(cmd + 5, "%u %u", &lo, &hi);
            g_draw_win_min = lo; g_draw_win_max = hi;
            emitf("devlink: draw window [%u,%u)", lo, hi);
        } else if (!strncmp(cmd, "shota", 5)) {
            extern int g_shot_alpha;
            g_shot_alpha = (cmd[5] == ' ' ) ? (int)strtoul(cmd+6,NULL,0) : 1;
            emitf("devlink: alpha screenshots %d", g_shot_alpha);
        } else if (!strncmp(cmd, "showred", 7)) {
            extern int g_tev_show_red; extern int g_shader_flush_req;
            g_tev_show_red = !g_tev_show_red;
            g_shader_flush_req = 1;
            emitf("devlink: showred=%d", g_tev_show_red);
        } else if (!strncmp(cmd, "showalpha", 9)) {
            extern int g_tev_show_alpha; extern int g_shader_flush_req;
            g_tev_show_alpha = (cmd[9]==' ') ? (int)strtoul(cmd+10,NULL,0) : 1;
            g_shader_flush_req = 1;     /* regenerate everything */
            emitf("devlink: tev alpha view %d", g_tev_show_alpha);
        } else if (!strncmp(cmd, "bp ", 3)) {
            g_bp_pc[0] = (u32)strtoul(cmd + 3, NULL, 16);
            g_bp_hits[0] = 0;
            emitf("devlink: bp0 at %08x", g_bp_pc[0]);
        } else if (!strncmp(cmd, "stw ", 4)) {
            extern u32 g_stw_lo, g_stw_hi; extern unsigned g_stw_n;
            unsigned lo=0,hi=0; sscanf(cmd+4,"%x %x",&lo,&hi);
            g_stw_lo=lo; g_stw_hi=hi; g_stw_n=0;
            emitf("devlink: store watch [%08x,%08x)", lo, hi);
        } else if (!strcmp(cmd, "vscan")) {
            /* Scan a wide window for the busiest non-gray page: locates the
             * THP output plane without having to catch a decode burst, since
             * a decoded frame persists in memory between bursts. */
            u32 a2, best = 0, besta = 0, second = 0, seconda = 0;
            for (a2 = 0x80800000u; a2 < 0x80f00000u; a2 += 0x1000u) {
                u32 nz = 0, k;
                for (k = 0; k < 0x1000u; k += 4) {
                    u32 w = mem_read32(a2 + k);
                    if (w && w != 0x80808080u && w != 0x10101010u) nz++;
                }
                if (nz > best) { second = best; seconda = besta;
                                 best = nz; besta = a2; }
                else if (nz > second) { second = nz; seconda = a2; }
            }
            emitf("devlink: vscan busiest %08x nz=%u/1024, 2nd %08x nz=%u",
                  besta, best, seconda, second);
        } else if (!strcmp(cmd, "stwinfo")) {
            extern u32 g_stw_min[8], g_stw_max[8]; extern u64 g_stw_cnt[8];
            unsigned i;
            for (i=0;i<8;i++) if (g_stw_cnt[i])
                emitf("  band%u %08x..%08x span=%uK writes=%llu", i,
                      g_stw_min[i], g_stw_max[i],
                      (g_stw_max[i]-g_stw_min[i])/1024u,
                      (unsigned long long)g_stw_cnt[i]);
        } else if (!strcmp(cmd, "spuhalt")) {
            extern void spu_vtx_shutdown(void);
            spu_vtx_shutdown();
            emitf("devlink: SPU path shut down");
        } else if (!strcmp(cmd, "spustat")) {
            extern void spu_vtx_stat(void (*)(const char *, ...));
            spu_vtx_stat(emitf);
        } else if (!strcmp(cmd, "jitstat")) {
            /* The flat in-race profile says this is a throughput problem, not
             * a hotspot problem -- so the numbers that matter are the ones
             * that apply to EVERY block: how much host code we emit per guest
             * instruction, and how often we bail to the interpreter. */
            emitf("devlink: jit blocks=%llu guest=%llu host=%llu (%.2fx) "
                  "fallbacks=%llu lookups=%llu",
                  (unsigned long long)g_jit_stats.blocks_compiled,
                  (unsigned long long)g_jit_stats.guest_insts_compiled,
                  (unsigned long long)g_jit_stats.host_insts_emitted,
                  jit_expansion_ratio(),
                  (unsigned long long)g_jit_stats.fallback_insts,
                  (unsigned long long)g_jit_stats.dispatch_lookups);
            {   unsigned o; char ln[160]; int n = 0;
                ln[0] = 0;
                for (o = 0; o < 64; o++)
                    if (g_jit_stats.fallback_by_opcd[o]) {
                        char t[32];
                        snprintf(t, sizeof t, "%u:%llu ", o,
                                 (unsigned long long)g_jit_stats.fallback_by_opcd[o]);
                        strncat(ln, t, sizeof ln - strlen(ln) - 1);
                        if (++n == 6) { emitf("  fb %s", ln); ln[0]=0; n=0; }
                    }
                if (n) emitf("  fb %s", ln);
            }
        } else if (!strcmp(cmd, "jreset")) {
            jit_profile_reset();
            emitf("devlink: jit profile reset");
        } else if (!strcmp(cmd, "gxstat")) {
            GXState *gg = gx_state();
            {   extern u64 g_addr_flips;
                emitf("devlink: gx cmds=%llu unknown=%llu last_unk=%02x draws=%llu "
                      "xfbcopies=%llu texcopies=%llu addrflips=%llu rsxframes=%u",
                      (unsigned long long)gg->parser.commands,
                      (unsigned long long)gg->parser.unknown_opcodes,
                      gg->parser.last_unknown,
                      (unsigned long long)g_gx_render.draws,
                      (unsigned long long)g_gx_render.efb_copies_xfb,
                      (unsigned long long)g_gx_render.efb_copies_texture,
                      (unsigned long long)g_addr_flips,
                      (unsigned)g_rsx.frames);
                {   extern void spu_vtx_stat(void (*)(const char *, ...));
                    spu_vtx_stat(emitf);
                }
            }
        } else if (!strcmp(cmd, "pc")) {
            extern PPCState *g_live_cpu;
            if (g_live_cpu)
                emitf("devlink: pc=%08x lr=%08x sp=%08x msr=%08x dc=%d "
                      "ctr=%08x", g_live_cpu->pc, g_live_cpu->lr,
                      g_live_cpu->gpr[1], g_live_cpu->msr,
                      (int)g_live_cpu->downcount, g_live_cpu->ctr);
                /* r2 and r13 are the small-data bases. A constant loaded off
                 * r2 that arrives wrong is either a bad base or a bad load,
                 * and only the base distinguishes them -- MKWii also loads
                 * StaticR.rel, which need not share the DOL's bases. */
                emitf("devlink: r2=%08x r13=%08x r3=%08x r4=%08x r5=%08x",
                      g_live_cpu->gpr[2], g_live_cpu->gpr[13],
                      g_live_cpu->gpr[3], g_live_cpu->gpr[4],
                      g_live_cpu->gpr[5]);
        } else if (!strncmp(cmd, "benchnow ", 9)) {
            /* Start the fixed-window measurement HERE, at the next completed
             * frame, for N frames.
             *
             * The frame-indexed script navigates blind: it presses at fixed
             * frames whatever is on screen, and one such path walks the title
             * into a state whose float range-reduction loop does not converge
             * (see docs/PLAN.md). tools/race.py navigates the same menus but
             * verifies every step by screenshot, so it reliably reaches a
             * race. This command lets that reliable navigation drive a precise
             * measurement: race.py gets in, then this starts the window. */
            unsigned long n = strtoul(cmd + 9, NULL, 0);
            if (n == 0 || n > 100000) n = 300;
            s_bench_at   = s_mkw_frames + 2;
            s_bench_len  = n;
            s_bench_armed = 1;
            s_bench_running = 0;
            s_bench_done = 0;
            emitf("devlink: bench window %lu frames from frame %llu", n,
                  (unsigned long long)s_bench_at);
        } else if (!strncmp(cmd, "jitcode ", 8)) {
            /* Dump the host code the recompiler emitted for the block that
             * STARTS at this guest pc.
             *
             * The register snapshot says f2 arrives at the loop as 0.0 when
             * the constant it is loaded from is 65536.0 and memory is intact.
             * Everything else has been checked from the outside; what has not
             * been read is the code actually generated for that load. */
            extern PPCState *g_live_cpu;
            char *endp = NULL;
            u32 want = (u32)strtoul(cmd + 8, &endp, 16);
            unsigned off = (endp && *endp) ? (unsigned)strtoul(endp, NULL, 10) : 0u;
            JitBlock *blk = g_live_cpu ? jit_get_block(g_live_cpu, want) : NULL;
            if (!blk || !blk->code) {
                emitf("devlink: no block starting at %08x", want);
            } else {
                unsigned n = blk->code_words, i, end;
                emitf("devlink: JITCODE %08x..%08x  %u guest -> %u host words "
                      "(from word %u)",
                      blk->guest_pc, blk->guest_end, blk->guest_insts, n, off);
                end = off + 150u;
                if (end > n) end = n;
                for (i = off; i < end; i++)
                    emitf("devlink: JW %u %08x", i, blk->code[i]);
            }
        } else if (!strcmp(cmd, "snap")) {
            /* Ask the run loop for an authoritative snapshot, and cut the
             * slice short so it happens at the next block boundary rather than
             * whenever the grant happens to run out. */
            extern PPCState *g_live_cpu;
            g_snap_ready = 0;
            g_snap_req   = 1;
            if (g_live_cpu) g_live_cpu->exit_requested = 1;
            emitf("devlink: snapshot requested");
        } else if (!strcmp(cmd, "snapshow")) {
            if (!g_snap_ready) {
                emit_line(NULL, "devlink: no snapshot yet (send `snap` first)");
            } else {
                unsigned k;
                emitf("devlink: SNAP pc=%08x lr=%08x r2=%08x r13=%08x",
                      g_snap_state.pc, g_snap_state.lr,
                      g_snap_state.gpr[2], g_snap_state.gpr[13]);
                for (k = 0; k < 8; k++)
                    emitf("devlink: SNAP f%u ps0=%016llx (%.9g)", k,
                          (unsigned long long)g_snap_state.ps[k].ps0.u,
                          g_snap_state.ps[k].ps0.f);
                emitf("devlink: SNAP fpscr=%08x hid2=%08x", g_snap_state.fpscr,
                      g_snap_state.hid2);
                /* GQR store/load types decide whether compile_psq specialises
                 * or declines; HID2[LSQE] decides whether it even tries. */
                emitf("devlink: SNAP gqr0=%08x gqr1=%08x gqr2=%08x gqr3=%08x",
                      g_snap_state.gqr[0], g_snap_state.gqr[1],
                      g_snap_state.gqr[2], g_snap_state.gqr[3]);
                emitf("devlink: SNAP gqr4=%08x gqr5=%08x gqr6=%08x gqr7=%08x",
                      g_snap_state.gqr[4], g_snap_state.gqr[5],
                      g_snap_state.gqr[6], g_snap_state.gqr[7]);
            }
        } else if (!strcmp(cmd, "fpr")) {
            /* The float registers, as raw bits. A range-reduction loop that
             * never terminates (fsubs/fcmpu/branch-back) is doing so because
             * one of its operands is wrong -- a zero or denormal divisor, or a
             * NaN dividend -- and only the bits say which. */
            extern PPCState *g_live_cpu;
            if (g_live_cpu) {
                unsigned k;
                for (k = 0; k < 8; k++) {
                    emitf("devlink: f%u ps0=%016llx (%.9g) ps1=%016llx", k,
                          (unsigned long long)g_live_cpu->ps[k].ps0.u,
                          g_live_cpu->ps[k].ps0.f,
                          (unsigned long long)g_live_cpu->ps[k].ps1.u);
                }
                emitf("devlink: fpscr=%08x AT pc=%08x lr=%08x sp=%08x",
                      g_live_cpu->fpscr, g_live_cpu->pc, g_live_cpu->lr,
                      g_live_cpu->gpr[1]);
            }
        } else if (!strncmp(cmd, "bpc ", 4)) {
            unsigned pc9=0, rg=0, vv=0;
            sscanf(cmd+4, "%x %u %x", &pc9, &rg, &vv);
            g_bp_pc[0]=pc9; g_bp_hits[0]=0;
            g_bp_cond_reg=rg & 31u; g_bp_cond_val=vv;
            emitf("devlink: bpc pc=%08x r%u==%08x", pc9, rg, vv);
        } else if (!strcmp(cmd, "bpinfo")) {
            emitf("devlink: bp0 pc=%08x hits=%llu r3=%08x r4=%08x r5=%08x "
                  "r6=%08x lr=%08x",
                  g_bp_pc[0], (unsigned long long)g_bp_hits[0],
                  g_bp_gpr[0][3], g_bp_gpr[0][4], g_bp_gpr[0][5],
                  g_bp_gpr[0][6], g_bp_lr[0]);
            emitf("devlink:  r25=%08x r27=%08x r28=%08x r30=%08x r31=%08x sp=%08x",
                  g_bp_gpr[0][25], g_bp_gpr[0][27], g_bp_gpr[0][28],
                  g_bp_gpr[0][30], g_bp_gpr[0][31], g_bp_gpr[0][1]);
            /* Dereference the first four args: the decoder's dst/src plane
             * pointers and what they point at. */
            {   unsigned k; u32 r[4];
                r[0]=g_bp_gpr[0][3]; r[1]=g_bp_gpr[0][4];
                r[2]=g_bp_gpr[0][5]; r[3]=g_bp_gpr[0][6];
                for (k=0;k<4;k++)
                    if (r[k] >= 0x80000000u && r[k] < 0x81800000u)
                        emitf("   [r%u=%08x] = %08x %08x %08x %08x", k+3, r[k],
                              mem_read32(r[k]), mem_read32(r[k]+4),
                              mem_read32(r[k]+8), mem_read32(r[k]+12));
                    else emitf("   r%u=%08x (not RAM)", k+3, r[k]);
            }
        } else if (!strcmp(cmd, "lcdma")) {
            extern u64 g_lcdma_loads, g_lcdma_stores, g_lcdma_last_mem;
            emitf("devlink: lcdma loads=%llu stores=%llu last_store=%08llx",
                  (unsigned long long)g_lcdma_loads,
                  (unsigned long long)g_lcdma_stores,
                  (unsigned long long)g_lcdma_last_mem);
        } else if (!strcmp(cmd, "ax")) {
            emitf("devlink: ax frames=%llu audible=%llu",
                  (unsigned long long)ax_stat_frames(),
                  (unsigned long long)ax_stat_audible_frames());
        } else if (!strcmp(cmd, "di")) {
            extern u64 g_di_last_offset, g_di_reads_pub;
            {   extern u64 g_di_last_dst;
                emitf("devlink: di reads=%llu last_off=%llx dst=%08llx",
                      (unsigned long long)g_di_reads_pub,
                      (unsigned long long)g_di_last_offset,
                      (unsigned long long)g_di_last_dst);
            }
        } else if (!strcmp(cmd, "jprof")) {
            jit_profile_report(devlink_emit_line, 24);
        } else if (!strcmp(cmd, "shflush")) {
            extern int g_shader_flush_req;
            g_shader_flush_req = 1;
            emit_line(NULL, "devlink: shader cache flush armed");
        } else if (!strncmp(cmd, "dbig ", 5)) {
            extern u32 g_draw_minverts;
            g_draw_minverts = (u32)strtoul(cmd + 5, NULL, 0);
            emitf("devlink: draw filter >=%u verts", g_draw_minverts);
        } else if (!strncmp(cmd, "bigdraw ", 8)) {
            /* Dump the first draw with >= N vertices in an upcoming frame --
             * finds the actual track geometry instead of a fixed index. */
            extern int g_draw_info_arm; extern u32 g_draw_info_minv;
            g_draw_info_minv = (u32)strtoul(cmd + 8, NULL, 0);
            g_draw_info_arm = 1;
            emitf("devlink: bigdraw >=%u verts armed", g_draw_info_minv);
        } else if (!strncmp(cmd, "analyze ", 8)) {
            /* Atomic per-draw analysis: for ONE upcoming frame, dump the
             * chosen draw's full state, render ONLY that draw, and screenshot
             * the result -- all inside the same frame, so the attract timer
             * cannot desynchronise the evidence the way it did across
             * separate commands. */
            extern u32 g_draw_info_idx; extern int g_draw_info_arm;
            extern u32 g_draw_win_min, g_draw_win_max;
            unsigned n2 = (unsigned)strtoul(cmd + 8, NULL, 0);
            { extern u32 g_draw_info_minv; g_draw_info_minv = 0; }
            g_draw_info_idx = n2; g_draw_info_arm = 1;
            g_draw_win_min = n2; g_draw_win_max = n2 + 1;
            s_analyze_shot = 2;         /* capture after two presented frames */
            emitf("devlink: analyzing draw %u", n2);
        } else if (!strncmp(cmd, "dinfo ", 6)) {
            extern u32 g_draw_info_idx; extern int g_draw_info_arm;
            g_draw_info_idx = (u32)strtoul(cmd + 6, NULL, 0);
            g_draw_info_arm = 1;
            emitf("devlink: dinfo armed for draw %u", g_draw_info_idx);
        } else if (!strcmp(cmd, "textures")) {
            gx_render_dump_textures(devlink_emit_line);
        } else if (!strcmp(cmd, "overlay")) {
            s_overlay_on = !s_overlay_on;
            emitf("devlink: overlay %s", s_overlay_on ? "on" : "off");
        } else if (!strncmp(cmd, "bt ", 3)) {
            extern unsigned g_bt_experiment;
            g_bt_experiment = (unsigned)strtoul(cmd + 3, NULL, 0);
            emitf("devlink: bt experiment mask -> %u", g_bt_experiment);
        } else if (!strcmp(cmd, "bt")) {
            emitf("devlink: bt channels=%u", ios_bt_channels());
        } else {
            emitf("devlink: commands are: mask <hex>, overlay, bt, "
                  "aot <0|1>, relaunch, quit");
        }
    }
}


/* ---------------------------------------------------------------- benchmark
 *
 * A reproducible capture: at chosen frame numbers, write the framebuffer AND a
 * full record of the renderer's state at that instant. The picture alone says
 * a frame is wrong; the record says WHY -- how many draws it took, how many
 * shader programs failed to fit, how many textures were undecodable, how many
 * draws were dropped for want of a position attribute, which state groups were
 * enabled. Comparing our frame N against Dolphin's frame N then localises a
 * difference to a subsystem instead of leaving "it looks off".
 *
 * Frame number is the anchor because it is the one clock both emulators agree
 * on from a cold boot with no input. */
static void bench_capture(void)
{
    static GXRenderStats prev;
    static int have_prev;
    char path[80];
    GXRenderStats *g = &g_gx_render;
    FILE *f;

    if (!s_bench_left || (unsigned)g_rsx.frames < s_bench_next) return;
    gx_render_sample_caches();      /* occupancy is sampled, not accumulated */

    /* Write the RECORD first and the image second. A 1.5 MB synchronous write
     * inside the frame loop is slow enough to stall it, and doing the image
     * first meant a stall lost the record too -- the first attempt produced a
     * single zero-byte PPM and no records at all. The small write always
     * lands; the picture is the part that can afford to fail. */
    f = fopen("/dev_hdd0/tmp/bench.txt", "a");
    if (f) {
#define D(field) (unsigned long long)(g->field - (have_prev ? prev.field : 0))
        fprintf(f,
            "frame=%u mask=%04x xfb=%08x "
            "draws=%llu verts=%llu progs_built=%llu prog_hits=%llu "
            "progs_failed=%llu tex_decoded=%llu tex_undecodable=%llu "
            "tex_admit_fail=%llu tex_entries=%u prog_entries=%u "
            "efb_tex=%llu efb_resolved=%llu no_pos=%llu overflow=%llu "
            "alpha_unmapped=%llu cull_unmapped=%llu ind_unmapped=%llu "
            "fog_unmapped=%llu texcoord_unmapped=%llu "
            "tex_bytes_live=%u xfb_presented=%u xfb_quiet=%u xfb_moves=%u "
            "bp_copies=%llu clearcol=%08x "
            "tb=%llu\n",
            (unsigned)g_rsx.frames, g_gx_state_mask, (unsigned)vi_current_xfb(),
            D(draws), D(vertices), D(programs_built), D(program_hits),
            D(programs_failed), D(textures_decoded), D(texture_undecodable),
            D(texture_admit_fail), g->texture_entries, g->program_entries,
            D(efb_copies_texture), D(efb_copies_resolved),
            D(skipped_no_pos), D(overflow),
            D(alpha_test_unmapped), D(cull_unmapped), D(indirect_unmapped),
            D(fog_range_unmapped), D(texcoords_unmapped),
            g->texture_bytes_live, s_xfb_frames, s_xfb_quiet_frames,
            s_xfb_moves,
            /* Copies the BP layer DECODED versus copies the renderer
             * EXECUTED. If the game is issuing copies and the executed
             * counters stay at zero, we are dropping them -- which leaves the
             * rendered scene stranded in the EFB and the display showing only
             * the clear colour, which is the flat pink on screen now. */
            (unsigned long long)gx_state()->bp.copies,
            (unsigned)gx_state()->bp.copy.clear_color,
            (unsigned long long)timing_timebase());
#undef D
        fclose(f);
    }
    snprintf(path, sizeof path, "/dev_hdd0/tmp/bench-%05u.ppm",
             (unsigned)g_rsx.frames);
    /* The flip chain is THREE buffers, so `current ^ 1` does not name the one
     * on screen and can index past the end -- which made every benchmark frame
     * come out a uniform clear colour and looked exactly like a rendering
     * fault. The buffer just presented is the one drawn before the current
     * one, which for a round-robin chain is current-1 modulo the count. */
    rsx_video_screenshot(path, 4u,
        (unsigned)((g_rsx.current + RSX_BUFFERS - 1) % RSX_BUFFERS));

    prev = *g; have_prev = 1;
    s_bench_left--;
    /* Always advance past the CURRENT frame, so a start point already behind
     * the frame counter cannot fire on every consecutive frame -- which is
     * what turned eight spaced captures into eight back-to-back 1.5 MB writes
     * and stalled the loop. */
    s_bench_next = (unsigned)g_rsx.frames + s_bench_step;
}

static void devlink_emit_line(const char *l) { emit_line(NULL, l); }

static void report_open(int *which)
{
    unsigned i;
    for (i = 0; i < sizeof k_report_paths / sizeof k_report_paths[0]; i++) {
        int fd = open(k_report_paths[i], O_CREAT | O_TRUNC | O_WRONLY,
                      S_IRWXU | S_IRWXG | S_IRWXO);
        if (fd >= 0) { s_fd = fd; *which = (int)i; log_set_fd(fd); return; }
    }
    *which = -1;
}

static void emit_line(void *ctx, const char *line)
{
    char nl = '\n';
    size_t n = 0;
    (void)ctx;
    while (line[n]) n++;

    /* fd 2 first, always -- even when no report file could be opened.
     *
     * PSL1GHT routes writes to fd 1 and 2 through sys_tty_write rather than the
     * filesystem, which makes it the one output channel that needs nothing to
     * have succeeded beforehand: no mount, no writable path, no open. RPCS3
     * transcribes it into its log, and a console debugger picks it up over the
     * wire, so a run that dies before it can create a file still says where. */
    write(2, line, n);
    write(2, &nl, 1);

    if (s_fd < 0) return;
    /* Unbuffered by construction: every line reaches the disk before the next
     * statement runs, so a crash cannot swallow the breadcrumb identifying
     * where it happened. */
    write(s_fd, line, n);
    write(s_fd, &nl, 1);
}

static void emitf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    emit_line(NULL, buf);
}

/* A breadcrumb. If the report ends with one of these, that stage is where
 * execution stopped. */
static void stage(const char *what)
{
    emitf(">> %s", what);
}

/* Whether this host will execute code the emulator generates at run time.
 *
 * A real PS3 always will -- lv2 pages are executable and there is no W^X. RPCS3
 * will not: it dispatches PPU instructions through a table populated when a
 * module is loaded, so memory filled with instructions at run time has no entry
 * and branching into it faults with "Segfault executing location 0". Verified
 * against both lv2 allocators (sysMemoryAllocate and the mmapper
 * reserve/allocate/map sequence), which rules out a permissions problem on our
 * side.
 *
 * This is a sentinel file rather than a probe on purpose: probing for "can I
 * execute generated code" means executing generated code, and on a host that
 * cannot, the probe *is* the crash. The harness that knows the answer states
 * it; the console, where the file never exists, executes everything.
 *
 * What is still verified without execution: process startup, the arena and
 * fastmem, syscall and file I/O paths, table init, the code cache, the timing
 * core, and that every block compiles with the expected shape. What moves to
 * qemu-ppc64 and the console is execution itself. */
static int execution_allowed(void)
{
    /* Not under /dev_hdd0/tmp: RPCS3 empties that directory at emulation start,
     * so a sentinel placed there is gone before the process can read it. It is
     * still the right home for the *report*, which is written afterwards. */
    static const char *const k_flags[] = {
        "/dev_hdd0/dolphin-ps3-nojit",
        "/dev_hdd0/game/DOLPHIN01/USRDIR/nojit",
    };
    unsigned i;
    for (i = 0; i < sizeof k_flags / sizeof k_flags[0]; i++) {
        int fd = open(k_flags[i], O_RDONLY);
        if (fd >= 0) { close(fd); return 0; }
    }
    return 1;
}


/* ------------------------------------------------------------------ */
/* Timing                                                              */
/* ------------------------------------------------------------------ */

#define PS3_TIMEBASE_HZ 79800000.0

static u64 read_timebase(void)
{
#if defined(__powerpc64__) || defined(__PPC64__)
    u64 t;
    __asm__ __volatile__ ("mftb %0" : "=r"(t));
    return t;
#else
    return 0;
#endif
}

/* ------------------------------------------------------------------ */
/* Benchmark                                                           */
/* ------------------------------------------------------------------ */

#define BENCH_CODE  0x80320000u
#define BENCH_DATA  0x80340000u
#define BENCH_ITERS 50000u

/* One rep is ~1,127 guest instructions (measured), so this is roughly 22
 * million -- enough that the 79.8 MHz timebase resolves the difference
 * comfortably, few enough that the interpreter side still finishes in a couple
 * of seconds and the whole console run stays well under a minute. */
#define REALBENCH_REPS 20000u

static void build_benchmark(void)
{
    PPCEmitter e;
    static u32 buf[64];
    unsigned i, n;

    emit_init(&e, buf, sizeof buf);

    /* Shaped like real game code: pointer-chasing loads, integer arithmetic, a
     * store, a compare and a backward branch -- the last of which is what block
     * linking rewrites, so this measures that too. */
    e_lwz(&e, 5, 0, 3);
    e_lwz(&e, 6, 4, 3);
    e_lwz(&e, 7, 8, 3);
    e_add(&e, 8, 5, 6);
    e_xor(&e, 9, 8, 7);
    e_rlwinm(&e, 10, 9, 3, 0, 28);
    e_add(&e, 11, 10, 5);
    e_stw(&e, 11, 12, 3);
    e_addi(&e, 3, 3, 16);
    e_cmpw(&e, 0, 3, 4);
    e_bc(&e, BO_FALSE, BI_EQ(0), -40);
    e_blr(&e);

    n = (unsigned)(emit_size(&e) / 4);
    for (i = 0; i < n; i++)
        mem_write32(BENCH_CODE + i * 4, buf[i]);
}

static void setup_bench_state(PPCState *s, u32 iters)
{
    memset(s, 0, sizeof *s);
    s->msr       = MSR_FP;
    s->const_one = 1.0;
    s->hid2      = HID2_PSE | HID2_LSQE;
    s->pc        = BENCH_CODE;
    s->gpr[3]    = BENCH_DATA;
    s->gpr[4]    = BENCH_DATA + iters * 16u;
    s->downcount = (s32)(iters * 16u + 1024u);
    s->exit_requested = 0;
}

/* The same workload as a bdnz counted loop -- the shape GCC gives every
 * `for (i=0; i<n; i++)`. The block's back-edge targets its own entry, so it is
 * exactly what loop register retention compiles: this is the benchmark that
 * measures that optimisation, where the cmpw/bc loop above cannot. Ten guest
 * instructions per iteration (the compare is gone; CTR carries the count). */
static void build_benchmark_ctr(void)
{
    PPCEmitter e;
    static u32 buf[64];
    unsigned i, n;

    emit_init(&e, buf, sizeof buf);
    e_lwz(&e, 5, 0, 3);
    e_lwz(&e, 6, 4, 3);
    e_lwz(&e, 7, 8, 3);
    e_add(&e, 8, 5, 6);
    e_xor(&e, 9, 8, 7);
    e_rlwinm(&e, 10, 9, 3, 0, 28);
    e_add(&e, 11, 10, 5);
    e_stw(&e, 11, 12, 3);
    e_addi(&e, 3, 3, 16);
    e_bc(&e, BO_DNZ, 0, -36);       /* bdnz back to the first lwz */
    e_blr(&e);

    n = (unsigned)(emit_size(&e) / 4);
    for (i = 0; i < n; i++)
        mem_write32(BENCH_CODE + i * 4, buf[i]);
}

static double run_bench_ctr(int use_jit, u32 iters, u64 *out_ticks)
{
    static PPCState s;
    u64 t0, t1;
    double secs;

    setup_bench_state(&s, iters);
    s.ctr = iters;                  /* bdnz counts this down */
    s.downcount = (s32)(iters * 10u + 1024u);

    t0 = read_timebase();
    if (use_jit) jit_run(&s); else interp_run(&s);
    t1 = read_timebase();

    *out_ticks = t1 - t0;
    secs = (double)(t1 - t0) / PS3_TIMEBASE_HZ;
    return secs > 0.0 ? ((double)iters * 10.0) / secs : 0.0;  /* 10 insts/iter */
}

static double run_bench(int use_jit, u32 iters, u64 *out_ticks)
{
    static PPCState s;
    u64 t0, t1;
    double secs;

    setup_bench_state(&s, iters);

    t0 = read_timebase();
    if (use_jit)
        jit_run(&s);
    else
        interp_run(&s);
    t1 = read_timebase();

    *out_ticks = t1 - t0;
    secs = (double)(t1 - t0) / PS3_TIMEBASE_HZ;

    /* 11 guest instructions per loop iteration. */
    return secs > 0.0 ? ((double)iters * 11.0) / secs : 0.0;
}

/* ------------------------------------------------------------------ */

/* Frame handler for the animated guest: present, then start the next frame.
 * Samples a pixel near the animated vertex on the first and last frames so
 * "it animated" is a measured statement, not an impression. */
static unsigned s_anim_frames;
static u32 s_anim_first, s_anim_last;

static void anim_frame_done(void *ctx)
{
    u32 px;
    int b = g_rsx.current;
    (void)ctx;

    rsx_wait_idle();
    px = rsx_sample(b, 0.30f, 0.68f);       /* near the animated vertex */
    if (s_anim_frames == 0) s_anim_first = px;
    s_anim_last = px;
    s_anim_frames++;

    rsx_frame_end();
    rsx_frame_begin();
    rsx_clear(0xFF101018u);
    gx_render_frame_begin();
}

/* ------------------------------------------------------------------ */
/* Boot the real Mario Kart Wii executable on the PPE and show it on TV */
/* ------------------------------------------------------------------ */

static void mkw_w32(u32 a, u32 v) { mem_write32(a, v); }

/* Frames the title finishes. An EFB copy to the external framebuffer is the
 * moment it says "this image is done" -- which is exactly when to flip it to
 * the TV and start the next one. Without this the title renders into a buffer
 * that is never presented and the screen stays as it was. */
static unsigned long long s_mkw_frames;
u64 g_mkw_frames_pub;
/* Speed accounting. The goal for this project is native speed, and the only
 * number that can settle it is measured on the console: guest instructions
 * retired per second of wall clock. Broadway issues 729 M/s, and this title
 * needs 8,500,207 guest instructions per frame at 60 fps -- 510 M/s -- so that
 * is what 1.00x means here. Both counters are written by the boot loop and
 * read by the overlay; nothing else touches them. */
static unsigned long long s_mkw_insts;
/* Instructions the guest actually retired: credited minus idle-skipped, summed
 * per slice where that difference is meaningful. */
static unsigned long long s_real_insts;
static unsigned long long s_idle_insts;   /* of those, never actually run */
static unsigned long long s_prof_ii0;
static u64 s_mkw_t0;
/* Rolling window for the on-screen rate, so it reports the present rather than
 * the average of everything since the machine was switched on. */
static u64 s_win_t0;
static unsigned long long s_win_f0, s_win_i0;
static double s_win_fps, s_win_ips;
#define NATIVE_INST_PER_SEC 510000000.0

/* ===================== PHASE PROFILE: interval reporting ==================
 * A breakdown every PROF_REPORT_EVERY presented frames.  Sixty is about three
 * seconds at the rate the title screen currently runs -- long enough that the
 * 79.8 MHz time base averages out, short enough that a session yields dozens
 * of independent samples rather than one grand average that hides drift.
 *
 * The three overlay sub-timers are outside the phase table on purpose.  The
 * overlay is one phase but three unrelated jobs -- CPU text into RSX memory,
 * thumbnail blits, and a framebuffer scan that runs one frame in 64 -- and
 * which of them dominates decides whether the overlay can simply be deleted.
 * They nest inside PH_OVERLAY, so they are a sub-split, not extra time.
 * ------------------------------------------------------------------------ */
#define PROF_REPORT_EVERY 60u
static u64 s_ov_text_tb, s_ov_thumb_tb, s_ov_scan_tb;
static u64 s_ov_text_n,  s_ov_thumb_n,  s_ov_scan_n;
static unsigned long long s_prof_f0, s_prof_i0, s_prof_d0, s_prof_v0;
static unsigned long long s_prof_c0, s_prof_b0;   /* jit compile ticks/blocks */
/* ================= END PHASE PROFILE: interval reporting ================== */

/* ===================== PHASE PROFILE: the breakdown =======================
 * Called from the frame handler once every PROF_REPORT_EVERY presented frames.
 * Writes to the report file, which is the console's only output channel.
 * ------------------------------------------------------------------------ */
static void prof_report_interval(void)
{
    {   /* The JIT's own health, per interval. A collapse in guest throughput
         * with an unchanged instruction count is either cache thrash or a
         * fallback storm, and these two counters separate them. */
        static u64 lb, lf;
        {   extern u32 g_warm_captured, g_warm_had_cold;
            emitf("[PROF]   jit warm self-loops: %u blocks (%u with interior "
                  "branch exits)", g_warm_captured, g_warm_had_cold);
        }
        emitf("[PROF]   jit: %llu blocks (+%llu), %llu flushes (+%llu)",
              (unsigned long long)g_jit_stats.blocks_compiled,
              (unsigned long long)(g_jit_stats.blocks_compiled - lb),
              (unsigned long long)g_jit_stats.cache_flushes,
              (unsigned long long)(g_jit_stats.cache_flushes - lf));
        lb = g_jit_stats.blocks_compiled; lf = g_jit_stats.cache_flushes;

        /* Code footprint. The recompiler hits 1.0 cycles per guest
         * instruction when its output is cache-resident and ~20 in the game,
         * so the bottleneck is not what we emit but how much of it the PPE
         * has to fetch. 32 KB L1-I and 512 KB L2 are shared by both threads;
         * once the hot code exceeds L2 every dispatch pays a memory round
         * trip. Bytes per guest instruction is the number to drive down. */
        if (g_jit_stats.blocks_compiled) {
            unsigned long long cb = (unsigned long long)g_jit_stats.code_bytes_used;
            unsigned long long nb = (unsigned long long)g_jit_stats.blocks_compiled;
            emitf("[PROF]   jit code: %llu KiB over %llu blocks "
                  "(%llu B/block, %llu x L2)",
                  cb >> 10, nb, cb / nb, cb / (512ull << 10));
        }
    }

    unsigned long long df = (unsigned long long)s_mkw_frames        - s_prof_f0;
    unsigned long long di = (unsigned long long)s_mkw_insts         - s_prof_i0;
    unsigned long long dii = (unsigned long long)s_idle_insts        - s_prof_ii0;
    unsigned long long dd = (unsigned long long)g_gx_render.draws   - s_prof_d0;
    unsigned long long dv = (unsigned long long)g_gx_render.vertices- s_prof_v0;

    {   static u64 s_prof_disp0;
        g_prof_dispatches = g_jit_stats.dispatch_lookups - s_prof_disp0;
        s_prof_disp0 = g_jit_stats.dispatch_lookups;
    }
    prof_enter(PH_REPORT);
    prof_dump(emit_line, NULL, PS3_TIMEBASE_HZ, "interval", 0, df, di, dd, dv);
    if (df) {
        char b[200];
        snprintf(b, sizeof b,
                 "[PROF]   overlay split: text %u us/f (%llu), thumbs %u us/f"
                 " (%llu), fb scan %u us/f (%llu runs)",
                 (unsigned)((double)s_ov_text_tb  * 1e6 / PS3_TIMEBASE_HZ / (double)df),
                 (unsigned long long)s_ov_text_n,
                 (unsigned)((double)s_ov_thumb_tb * 1e6 / PS3_TIMEBASE_HZ / (double)df),
                 (unsigned long long)s_ov_thumb_n,
                 (unsigned)((double)s_ov_scan_tb  * 1e6 / PS3_TIMEBASE_HZ / (double)df),
                 (unsigned long long)s_ov_scan_n);
        emit_line(NULL, b);
        snprintf(b, sizeof b,
                 "[PROF]   flips: %llu presented, %llu flip timeouts so far",
                 (unsigned long long)g_rsx.frames,
                 (unsigned long long)g_rsx.flip_timeouts);
        emit_line(NULL, b);
        /* Compilation hides inside jit_run and so inside PH_JIT, but it does
         * not arrive smoothly: a scene change after a load compiles thousands
         * of cold blocks in a handful of slices. Per-interval microseconds
         * plus the worst single compile is what says whether such a burst is
         * long enough for the player to see it as a hitch. */
        {   /* Real guest work vs. spins the JIT skipped. Everything derived
             * from the raw instruction counter -- inst/s, cycles/instruction --
             * is inflated by the second number, so print both rather than let
             * a flattering ratio stand unqualified. */
            snprintf(b, sizeof b,
                     "[PROF]   REAL: %llu insts/frame executed, %llu skipped "
                     "(%.1f%% of the credited count was never run)",
                     (unsigned long long)((di > dii ? di - dii : 0) / df),
                     (unsigned long long)(dii / df),
                     di ? 100.0 * (double)dii / (double)di : 0.0);
            emit_line(NULL, b);
        }
        {   /* All-direct draws could bypass vertex decode entirely by handing
             * the guest's own buffer to the RSX; indexed ones cannot. The
             * ratio is the ceiling on that idea. */
            extern unsigned long long g_gx_draws_direct, g_gx_draws_indexed;
            u64 tot = g_gx_draws_direct + g_gx_draws_indexed;
            snprintf(b, sizeof b,
                     "[PROF]   vtx draws: %llu direct, %llu indexed (%.1f%% direct)",
                     (unsigned long long)g_gx_draws_direct,
                     (unsigned long long)g_gx_draws_indexed,
                     tot ? 100.0 * (double)g_gx_draws_direct / (double)tot : 0.0);
            emit_line(NULL, b);
        }
        {   /* What the SPU itself says it did. `work` is the per-job critical
             * path the PPU ends up spinning on, and `dma` is how much of that
             * is the SPU blocked on main memory rather than computing --
             * which decides whether the fix is fewer round trips or a faster
             * decoder. Ticks are the 79.8 MHz timebase. */
            extern void spu_vtx_spustat(u32 *, u32 *, u32 *, u32 *);
            u32 sj = 0, sp = 0, sw = 0, sd = 0;
            spu_vtx_spustat(&sj, &sp, &sw, &sd);
            if (sj) {
                double tick_us = 1.0 / 79.8;   /* microseconds per tick */
                snprintf(b, sizeof b,
                         "[PROF]   spu self: work %.2f us/job "
                         "(dma %.2f = %.0f%%), idle %.2f us/job, over %u jobs",
                         (double)sw * tick_us / (double)sj,
                         (double)sd * tick_us / (double)sj,
                         sw ? 100.0 * (double)sd / (double)sw : 0.0,
                         (double)sp * tick_us / (double)sj, sj);
                emit_line(NULL, b);
            }
        }
        {               /* How EFB copies are serving texture binds. A bind that matches a
             * copy destination but finds no surface behind it falls through to
             * decoding guest memory -- and guest memory at a copy destination
             * was never written, because the copy happened on the GPU. That
             * decode is the full-screen RGB noise the attract demo shows. */
            snprintf(b, sizeof b,
                     "[PROF]   efb copies: tex=%llu xfb=%llu unmodelled=%llu "
                     "evict=%llu | binds ok=%llu STALE=%llu inval=%llu/%llu",
                     (unsigned long long)g_efb_copy.copies_texture,
                     (unsigned long long)g_efb_copy.copies_xfb,
                     (unsigned long long)g_efb_copy.copies_unmodelled,
                     (unsigned long long)g_efb_copy.evictions,
                     (unsigned long long)g_efb_copy.binds_resolved,
                     (unsigned long long)g_efb_copy.binds_stale,
                     (unsigned long long)g_efb_copy.cpu_invalidations,
                     (unsigned long long)g_efb_copy.guard_invalidations);
            emit_line(NULL, b);
        }
        {   /* SPU draw coverage. `vertex` is PPU-side decode, so it is
             * non-zero exactly to the extent draws FAIL to reach the SPU --
             * either too many vertices for one job, or a format the recipe
             * builder refuses. Both are fixable; neither is visible without
             * this line. */
            extern u64 g_jit_fallback_op[64], g_jit_fallback_total;
            extern PPCState *g_live_cpu;
            {   /* The recompiler's declines, ranked. An instruction the JIT
                 * will not compile is interpreted one at a time behind a full
                 * spill and reload, so a common one costs far more than its
                 * share of the static instruction count suggests. */
                unsigned o, best[4] = {0,0,0,0};
                for (o = 0; o < 64; o++) {
                    unsigned k;
                    for (k = 0; k < 4; k++)
                        if (g_jit_fallback_op[o] > g_jit_fallback_op[best[k]]) {
                            unsigned j;
                            for (j = 3; j > k; j--) best[j] = best[j-1];
                            best[k] = o; break;
                        }
                }
                {   /* Per INTERVAL, and ranked by what actually executed. */
                    static u32 lfb[64];
                    unsigned o2, b2[3] = {0,0,0};
                    u64 tot = 0;
                    u32 cur[64];
                    for (o2 = 0; o2 < 64; o2++) {
                        cur[o2] = g_live_cpu ? g_live_cpu->fallback_by_op[o2] : 0u;
                        cur[o2] -= lfb[o2];
                    }
                    for (o2 = 0; o2 < 64; o2++) {
                        unsigned k2;
                        tot += cur[o2];
                        for (k2 = 0; k2 < 3; k2++)
                            if (cur[o2] > cur[b2[k2]]) {
                                unsigned j2;
                                for (j2 = 2; j2 > k2; j2--) b2[j2] = b2[j2-1];
                                b2[k2] = o2; break;
                            }
                    }
                    for (o2 = 0; o2 < 64; o2++)
                        lfb[o2] = g_live_cpu ? g_live_cpu->fallback_by_op[o2] : 0u;
                    snprintf(b, sizeof b,
                             "[PROF]   jit fallbacks RUN: %llu/frame (%.3f%% of "
                             "insts); top op %u:%u %u:%u %u:%u",
                             (unsigned long long)(tot / df), 
                             di ? 100.0 * (double)tot / (double)di : 0.0,
                             b2[0], cur[b2[0]], b2[1], cur[b2[1]],
                             b2[2], cur[b2[2]]);
                    emit_line(NULL, b);
                }
                snprintf(b, sizeof b,
                         "[PROF]   jit declines: %llu total; top opcodes "
                         "%u:%llu %u:%llu %u:%llu %u:%llu",
                         (unsigned long long)g_jit_fallback_total,
                         best[0], (unsigned long long)g_jit_fallback_op[best[0]],
                         best[1], (unsigned long long)g_jit_fallback_op[best[1]],
                         best[2], (unsigned long long)g_jit_fallback_op[best[2]],
                         best[3], (unsigned long long)g_jit_fallback_op[best[3]]);
                emit_line(NULL, b);
            }
            extern u64 g_spu_jobs, g_spu_fallbacks, g_spu_too_big;
            static u64 lj, lf, lb;
            u64 dj = g_spu_jobs - lj, df2 = g_spu_fallbacks - lf;
            u64 db = g_spu_too_big - lb, tot;
            lj = g_spu_jobs; lf = g_spu_fallbacks; lb = g_spu_too_big;
            tot = dj + df2 + db;
            snprintf(b, sizeof b,
                     "[PROF]   spu coverage: %llu jobs, %llu unbuildable, "
                     "%llu too-big (%llu%% of draws reached the SPU)",
                     (unsigned long long)dj, (unsigned long long)df2,
                     (unsigned long long)db,
                     (unsigned long long)(tot ? dj * 100 / tot : 0));
            emit_line(NULL, b);
        }
        {   extern u64 g_ring_low_hits, g_ring_low_tests;
            extern u32 g_ring_span_min;
            /* If the headroom fence fires on nearly every draw then the ring
             * is too small and the SPU can never run ahead -- which is the
             * same stall the per-draw fence caused, arrived at differently. */
            snprintf(b, sizeof b,
                     "[PROF]   ring fence: %llu of %llu tests fired, "
                     "min headroom %u KiB",
                     (unsigned long long)g_ring_low_hits,
                     (unsigned long long)g_ring_low_tests,
                     (unsigned)(g_ring_span_min >> 10));
            emit_line(NULL, b);
        }
        {   extern u64 g_efb_blits, g_efb_blit_skips;
            snprintf(b, sizeof b,
                     "[PROF]   efb blits: %llu done, %llu skipped",
                     (unsigned long long)g_efb_blits,
                     (unsigned long long)g_efb_blit_skips);
            emit_line(NULL, b);
        }
        {   /* Which rectangles the world and the HUD actually got, and which
             * EFB copies the presenter recognised. In-race the world lands in
             * a corner of the EFB while the HUD draws full size; a sampled log
             * cannot say whether that is the viewport, the scissor, or the
             * copy, but a census of distinct rectangles can. */
            unsigned i;
            for (i = 0; i < g_vp_census_n && i < 8; i++) {
                snprintf(b, sizeof b,
                         "[PROF]   vp[%u] view %ux%u@%u,%u  scissor %ux%u@%u,%u"
                         "  uses=%u draws=%u persp=%u  raw sc=%d,%d off=%d,%d", i,
                         g_vp_census[i].vw, g_vp_census[i].vh,
                         g_vp_census[i].vx, g_vp_census[i].vy,
                         g_vp_census[i].sw, g_vp_census[i].sh,
                         g_vp_census[i].sx, g_vp_census[i].sy,
                         (unsigned)g_vp_census[i].uses,
                         (unsigned)g_vp_census[i].draws,
                         (unsigned)g_vp_census[i].draws_persp,
                         (int)g_vp_census[i].raw_sc[0],
                         (int)g_vp_census[i].raw_sc[1],
                         (int)g_vp_census[i].raw_off[0],
                         (int)g_vp_census[i].raw_off[1]);
                emit_line(NULL, b);
            }
            for (i = 0; i < g_cp_census_n && i < 8; i++) {
                snprintf(b, sizeof b,
                         "[PROF]   cp[%u] %ux%u to_xfb=%u presented=%u uses=%u",
                         i, g_cp_census[i].w, g_cp_census[i].h,
                         g_cp_census[i].to_xfb, g_cp_census[i].matched,
                         (unsigned)g_cp_census[i].uses);
                emit_line(NULL, b);
            }
        }
        {   /* What the FIFO phase is actually made of. Draw count alone does
             * not explain it: a stream of one-triangle draws each preceded by
             * a state change costs very differently from a few large ones. */
            static u64 c_bp, c_cp, c_xf, c_xw, c_dl;
            const GXParser *gp = &gx_state()->parser;
            snprintf(b, sizeof b,
                     "[PROF]   gx cmds/frame: bp %llu, cp %llu, xf %llu "
                     "(%llu words), dlist %llu",
                     (unsigned long long)((gp->n_bp - c_bp) / df),
                     (unsigned long long)((gp->n_cp - c_cp) / df),
                     (unsigned long long)((gp->n_xf - c_xf) / df),
                     (unsigned long long)((gp->n_xf_words - c_xw) / df),
                     (unsigned long long)((gp->n_dlist - c_dl) / df));
            emit_line(NULL, b);
            c_bp = gp->n_bp; c_cp = gp->n_cp; c_xf = gp->n_xf;
            c_xw = gp->n_xf_words; c_dl = gp->n_dlist;
        }
        {   /* The SPU join: how often the PPU actually had to wait, and how
             * hard. `spun` well below `calls` means the SPU is comfortably
             * ahead and the join is just a cheap check; `spun` close to
             * `calls` with a large spin count means the SPU is the critical
             * path and the PPU is burning cycles (and its SMT sibling's
             * issue slots) doing nothing. */
            extern unsigned long long g_spu_join_calls, g_spu_join_spun,
                                      g_spu_join_spins;
            snprintf(b, sizeof b,
                     "[PROF]   spu join: %llu calls, %llu spun (%.1f%%), "
                     "%llu spin iters (%.0f avg per spinning join)",
                     g_spu_join_calls, g_spu_join_spun,
                     g_spu_join_calls ? 100.0 * (double)g_spu_join_spun
                                        / (double)g_spu_join_calls : 0.0,
                     g_spu_join_spins,
                     g_spu_join_spun ? (double)g_spu_join_spins
                                       / (double)g_spu_join_spun : 0.0);
            emit_line(NULL, b);
        }
        snprintf(b, sizeof b,
                 "[PROF]   jit compile: %u us/f over %llu blocks this interval"
                 " (worst single compile %u us)",
                 (unsigned)((double)(g_jit_stats.compile_ticks - s_prof_c0)
                            * 1e6 / PS3_TIMEBASE_HZ / (double)df),
                 (unsigned long long)(g_jit_stats.blocks_compiled - s_prof_b0),
                 (unsigned)((double)g_jit_stats.compile_ticks_max
                            * 1e6 / PS3_TIMEBASE_HZ));
        emit_line(NULL, b);
    }
    s_ov_text_tb = s_ov_thumb_tb = s_ov_scan_tb = 0;
    s_ov_text_n  = s_ov_thumb_n  = s_ov_scan_n  = 0;
    s_prof_f0 = s_mkw_frames;         s_prof_i0 = s_mkw_insts;
    s_prof_ii0 = s_idle_insts;
    s_prof_d0 = g_gx_render.draws;    s_prof_v0 = g_gx_render.vertices;
    s_prof_c0 = g_jit_stats.compile_ticks;
    s_prof_b0 = g_jit_stats.blocks_compiled;
    prof_exit();
    prof_window_close();
}
/* =================== END PHASE PROFILE: the breakdown ===================== */

/* ------------------------------------------------------------------ */
/* Fixed-scene benchmark                                                 */
/*                                                                       */
/* Every on-console A/B so far has been navigated by screenshot, which    */
/* does not reproduce a scene: two runs of the same script landed on      */
/* 3,381 and 7,037 draws per frame, and cost per guest instruction varies */
/* from 1.0 to 35 on scene alone. That is enough to swamp any change      */
/* smaller than about 2x, which is every remaining candidate.             */
/*                                                                       */
/* So drive input from the GUEST FRAME COUNTER instead of from what is on */
/* screen. The same script then produces the same guest state at the same */
/* frame on every run, whatever the host does; only wall time varies,     */
/* which is precisely the quantity under test.                            */
/*                                                                       */
/* Script: /dev_hdd0/tmp/dolphin-bench.txt, one directive per line.       */
/*     <frame> <buttons-hex> [hold]   press buttons at that guest frame   */
/*     measure <frame> <count>        time `count` frames from `frame`    */
/* Lines starting with # are ignored.                                     */
/* ------------------------------------------------------------------ */
#define BENCH_MAX_STEPS 64

/* x,y are normalised pointer coordinates; negative means "leave the pointer
 * alone". Without these the script could only press buttons, so it could not
 * walk a menu -- which meant every benchmark window landed on a title screen
 * with 50 draws a frame and said nothing about the graphics path. */
typedef struct { u64 frame; u16 buttons; u16 hold; float x, y; } BenchStep;
static BenchStep s_bench[BENCH_MAX_STEPS];
static unsigned  s_bench_n;
static u64       s_bench_at, s_bench_len;
static int       s_bench_armed, s_bench_running, s_bench_done;
static u64       s_bench_t0, s_bench_i0, s_bench_idle0, s_bench_d0;

static void bench_load(void)
{
    FILE *f = fopen("/dev_hdd0/tmp/dolphin-bench.txt", "r");
    char line[128];
    if (!f) return;
    while (fgets(line, sizeof line, f)) {
        unsigned long long fr; unsigned b, h;
        if (line[0] == '#' || line[0] == '\n') continue;
        if (sscanf(line, "measure %llu %llu",
                   (unsigned long long *)&s_bench_at,
                   (unsigned long long *)&s_bench_len) == 2)
            continue;
        h = 12;
        {   double px = -1.0, py = -1.0;
            int got = sscanf(line, "%llu %x %u %lf %lf", &fr, &b, &h, &px, &py);
            if (got >= 2 && s_bench_n < BENCH_MAX_STEPS) {
                if (got < 3) h = 12;
                s_bench[s_bench_n].frame   = (u64)fr;
                s_bench[s_bench_n].buttons = (u16)b;
                s_bench[s_bench_n].hold    = (u16)h;
                s_bench[s_bench_n].x       = (float)px;
                s_bench[s_bench_n].y       = (float)py;
                s_bench_n++;
            }
        }
    }
    fclose(f);
    if (s_bench_n || s_bench_len) {
        s_bench_armed = 1;
        emitf("bench: %u input steps, measuring %llu frames from frame %llu",
              s_bench_n, (unsigned long long)s_bench_len,
              (unsigned long long)s_bench_at);
    }
}

/* Called once per completed guest frame. */
static void bench_tick(u64 frame)
{
    unsigned i;
    if (!s_bench_armed || s_bench_done) return;

    for (i = 0; i < s_bench_n; i++)
        if (s_bench[i].frame == frame) {
            if (s_bench[i].x >= 0.0f && s_bench[i].y >= 0.0f)
                ios_bt_set_pointer(s_bench[i].x, s_bench[i].y);
            s_inject_buttons = s_bench[i].buttons;
            s_inject_frames  = s_bench[i].hold;
        }

    if (!s_bench_running && s_bench_len && frame == s_bench_at) {
        s_bench_running = 1;
        s_bench_t0    = read_timebase();
        s_bench_i0    = s_mkw_insts;
        s_bench_idle0 = s_real_insts;
        s_bench_d0    = g_gx_render.draws;
        emitf("bench: START at frame %llu", (unsigned long long)frame);
    } else if (s_bench_running && frame == s_bench_at + s_bench_len) {
        double secs = (double)(read_timebase() - s_bench_t0) / PS3_TIMEBASE_HZ;
        u64 di   = s_mkw_insts - s_bench_i0;
        u64 real = s_real_insts - s_bench_idle0;   /* accumulated per slice */
        u64 dii  = di > real ? di - real : 0;      /* the skipped remainder */
        s_bench_running = 0;
        s_bench_done    = 1;
        emitf("bench: RESULT %llu frames in %.3f s = %.3f fps",
              (unsigned long long)s_bench_len, secs,
              secs > 0 ? (double)s_bench_len / secs : 0.0);
        /* Report the INPUTS, not just the difference.
         *
         * `real = credited - idle_skipped` read 0 for every in-race window
         * while working fine on the title screen, and a metric that silently
         * collapses to zero in exactly the scene being optimised is worse than
         * no metric. Per slice the credit is `grant - downcount` and an idle
         * skip zeroes the downcount before charging the remainder, so credited
         * should never be the smaller of the two -- printing both says which
         * assumption is wrong instead of leaving it to be re-derived. */
        emitf("bench: RESULT credited=%llu idle_skipped=%llu real=%llu",
              (unsigned long long)di, (unsigned long long)dii,
              (unsigned long long)real);
        emitf("bench: RESULT %llu real insts, %.3f cycles per guest instruction",
              (unsigned long long)real,
              real ? secs * 3.2e9 / (double)real : 0.0);
        /* Say what was actually measured. A window that lands on a menu is
         * perfectly reproducible and tells you nothing about the graphics
         * path -- which is exactly the trap this hit once already. */
        emitf("bench: RESULT %llu draws over the window = %llu draws/frame",
              (unsigned long long)(g_gx_render.draws - s_bench_d0),
              (unsigned long long)((g_gx_render.draws - s_bench_d0) /
                                   (s_bench_len ? s_bench_len : 1)));
        if (s_fd >= 0) fsync(s_fd);
    }
}

static void mkwii_frame_done(void *ctx)
{
    (void)ctx;
    /* From here on this thread presents, draws the overlay and may take a
     * screenshot -- all of which write the RSX command ring the worker owns.
     * Take ownership back first. */
    gx_worker_drain();
    s_mkw_frames++, g_mkw_frames_pub = s_mkw_frames;
    bench_tick(s_mkw_frames);

    /* Presenting every frame means waiting for a display flip every frame, and
     * a flip that does not retire costs its full timeout -- with a thousand
     * frames in a boot that is minutes of pure stalling, which is exactly what
     * a first console run looked like. The title's frame rate is not the
     * display's, so show one frame in every k and simply start the next one
     * otherwise: the picture still updates several times a second and the boot
     * runs at the speed of the recompiler rather than the flip timeout. */
    /* Native pacing: present every frame, vsync-locked. rsx_frame_end waits
     * for the flip, so the display's 60 Hz is the frame pacer -- exactly the
     * cadence a real Wii delivers -- and with idle skip the emulator has the
     * headroom to make every deadline. The safety valve: if flips start
     * timing out (the pre-fix wedge mode), retreat to presenting one frame
     * in thirty so the run degrades instead of freezing. */
    unsigned present_every = (g_rsx.flip_timeouts < 30) ? 1u : 30u;
    if (s_mkw_frames <= 4 || (s_mkw_frames % present_every) == 0) {
        /* Write the counters into the frame the title just finished, before it
         * is flipped. Presenting them as a separate CPU frame does not work:
         * the next title frame flips over them faster than they can be read. */
        if (s_xmb_menu_open) {
            /* The XMB owns the screen while its in-game menu is up. Continuing
             * to submit flips and clear buffers underneath it is what made the
             * PS button crash. */
            return;
        }
        {
            /* VIDEO PLAYBACK. A title playing THP writes decoded frames
             * straight into the external framebuffer and never touches GX, so
             * a renderer that only presents GX output shows none of it. If the
             * video interface points at a framebuffer and GX has drawn
             * nothing this frame, display the XFB instead -- converting from
             * YUV 4:2:2, which is what it actually holds. */
            /* The trigger must be CONSERVATIVE. A probe run showed the naive
             * condition ("GX drew nothing since the last frame") firing
             * constantly -- any momentarily static screen would qualify, and
             * presenting raw framebuffer memory over a good rendered frame
             * would flicker. Video playback is different in kind: GX goes
             * quiet for MANY consecutive frames while the video interface is
             * repointed at fresh decoded frames. So require both, and hold
             * the decision until it is unambiguous. */
            static u64 last_draws;
            static u32 last_xfb;
            u32 xfb = vi_current_xfb();
            u64 draws_now = g_gx_render.draws;

            /* Video is not "GX went quiet"; it is "the video interface was
             * pointed somewhere GX did not render". A THP stream is decoded by
             * the CPU straight into the XFB and produces NO frame-end EFB
             * copy, whereas every rendered frame ends with one. Requiring
             * eight consecutive silent frames instead missed the case where
             * the title keeps drawing an overlay during playback -- the video
             * then never reached the screen and the display kept whatever was
             * under it, which is the magenta the console shows. */
            static u64 last_xfb_copies;
            u64 xfb_copies_now = g_gx_render.efb_copies_xfb;
            int rendered_this_frame = (xfb_copies_now != last_xfb_copies);
            last_xfb_copies = xfb_copies_now;

            if (draws_now == last_draws) {
                if (s_xfb_quiet_frames < 1000) s_xfb_quiet_frames++;
            } else                             s_xfb_quiet_frames = 0;
            if (xfb && xfb != last_xfb) {
                last_xfb = xfb;
                if (s_xfb_moves < 1000) s_xfb_moves++;
            }

            /* Eight quiet frames is a third of a second of GX silence, which a
             * rendered scene never produces, and the framebuffer must have
             * been repointed at least twice so a single stale pointer cannot
             * trigger it. */
            /* REVERTED to the conservative rule, and gated off entirely by
             * default.
             *
             * The permissive rule ("present whenever GX did not produce a
             * frame-end copy") was wrong for this title. Mario Kart Wii's
             * attract sequence is not a THP video at all -- the per-frame
             * records show GX issuing ~50 draws every frame throughout it, so
             * it is 3D being rendered. Presenting the XFB over that replaced a
             * correct render with a YUV reinterpretation of framebuffer
             * memory, which looks like a flickering gradient, and the CPU-side
             * conversion of 640x480 to 1080p every frame cost most of the
             * frame rate.
             *
             * So: the strict rule only, and only when explicitly enabled, so a
             * wrong guess here can never degrade normal rendering again. */
            if (xfb && g_rsx.inited && xfb_present_enabled() &&
                s_xfb_moves >= 2 && s_xfb_quiet_frames >= 8) {
                u32 *fbx = g_rsx.buffer[g_rsx.current];
                /* The XFB the video interface scans is NOT the EFB copy
                 * rectangle. MKWii renders a 608x456 EFB, but a THP video is
                 * decoded straight into an XFB whose stride the VI sets, and
                 * reading it at the EFB width walks every row off by the
                 * difference -- which skews the picture into the flickering
                 * gradient the console shows. The VI stride register is not
                 * modelled yet, so the width is tunable by file and settled by
                 * measurement rather than by guessing the register layout. */
                if (xfb_present(xfb, xfb_width(),
                                s_efb_height ? s_efb_height : 480,
                                fbx, g_rsx.pitch / 4,
                                g_rsx.width, g_rsx.height) == 0)
                    s_xfb_frames++;
            }
            last_draws = draws_now;
        }
        if (g_rsx.inited) {
            char o1[96];
            u32 *fb;
            /* The GPU renders the frame asynchronously; writing text before it
             * has finished just gets painted over. Wait for it to go idle, then
             * the framebuffer is ours until the flip. */
            /* The overlay writes text into the framebuffer with the CPU, which
             * requires the GPU to be finished with it -- and that full drain
             * was measured at 22.0% of every frame, on top of the overlay's
             * own cost. It is a debug readout, not part of the game, so it now
             * runs once every 30 presented frames (about twice a second) and
             * the rest of the frames never stall at all. */
            /* Off by default: the overlay is a debug readout, and the game
             * is the product now. Amortising it to one frame in thirty made
             * it FLICKER once the emulator went triple buffered (it lands in
             * one buffer of three), so instead it is either fully on -- drawn
             * every frame, stable -- or fully off and costing nothing. L1
             * toggles it. */
            if (!s_overlay_on) goto overlay_done;
            prof_enter(PH_OVERLAY);     /* PHASE PROFILE: overlay begins */
            rsx_wait_idle();            /* its naps are carved out as PH_WAITGPU */
            fb = g_rsx.buffer[g_rsx.current];
            int p0 = (int)g_rsx.pitch;
            /* The one number that splits "GPU draws nothing" from "drawn but
             * not shown": pixels in this buffer differing from the clear
             * colour, counted before the overlay touches it. Every ~64th
             * present -- the count walks 8 MB of RSX memory. */
            {
                static unsigned gpx_div;
                static u32 gpx_last;
                /* Retired: walking 8 MB of RSX local memory from the PPU cost
                 * 1113 us per frame and existed to answer "is the GPU drawing
                 * anything at all", which the game answers by rendering. */
                (void)gpx_div; gpx_last = 0;
                snprintf(o1, sizeof o1,
                         "FRAMES %llu  DRAWS %llu  GPUPX %u  DISC %u",
                         (unsigned long long)s_mkw_frames,
                         (unsigned long long)g_gx_render.draws,
                         (unsigned)gpx_last, ios_progress_disc_reads());
            }
            /* Shadow first: the strap screen is white, and white-on-white
             * counters read as "blank screen, probably hung" from the sofa. */
            {   u64 t_ = prof_tb();             /* PHASE PROFILE */
            rsx_draw_text_scaled(fb, p0, 43, 43, 0xFF000000u, 3, o1);
            rsx_draw_text_scaled(fb, p0, 40, 40, 0xFFFFFFFFu, 3, o1);
                s_ov_text_tb += prof_tb() - t_; s_ov_text_n++; }
            /* The line the whole project is judged on.
             *
             * Measured over a *window*, not over the process lifetime. The
             * lifetime form -- frames divided by seconds since slice 0 --
             * includes the entire boot, during which no frame exists at all,
             * and every lighter screen that came before this one. Once the
             * title settles at a rate below that running average the displayed
             * number slides downward continuously, by ever-smaller steps,
             * forever: 15.5, 14.1, 13.9, ... That is the arithmetic of a
             * cumulative mean converging from above, and by eye it is exactly
             * what a memory leak looks like. A two-second window reports what
             * the emulator is doing *now*, so a steady state reads steady and
             * an actual regression is visible as one. */
            if (s_mkw_t0) {
                char o2[96];
                double now_s = (double)(read_timebase() - s_win_t0)
                             / PS3_TIMEBASE_HZ;
                double ips, fps;
                if (!s_win_t0) {
                    s_win_t0 = read_timebase();
                    s_win_f0 = s_mkw_frames;
                    s_win_i0 = s_mkw_insts;
                    now_s = 0.0;
                }
                if (now_s >= 2.0) {
                    s_win_fps = (double)(s_mkw_frames - s_win_f0) / now_s;
                    s_win_ips = (double)(s_mkw_insts  - s_win_i0) / now_s;
                    s_win_t0  = read_timebase();
                    s_win_f0  = s_mkw_frames;
                    s_win_i0  = s_mkw_insts;
                }
                /* Until the first window closes there is nothing to report
                 * but the lifetime figure; after that the window owns it. */
                if (s_win_fps > 0.0 || s_win_ips > 0.0) {
                    ips = s_win_ips;
                    fps = s_win_fps;
                } else {
                    double life = (double)(read_timebase() - s_mkw_t0)
                                / PS3_TIMEBASE_HZ;
                    if (life < 0.01) life = 0.01;
                    ips = (double)s_mkw_insts / life;
                    fps = (double)s_mkw_frames / life;
                }
                snprintf(o2, sizeof o2,
                         "%d M INST/S   %d.%02dx NATIVE   %d.%01d FPS",
                         (int)(ips / 1e6),
                         (int)(ips / NATIVE_INST_PER_SEC),
                         (int)(ips / NATIVE_INST_PER_SEC * 100) % 100,
                         (int)fps, ((int)(fps * 10)) % 10);
                {   u64 t_ = prof_tb();         /* PHASE PROFILE */
                rsx_draw_text_scaled(fb, p0, 43, 93, 0xFF000000u, 3, o2);
                rsx_draw_text_scaled(fb, p0, 40, 90, 0xFFFFE080u, 3, o2);
                    s_ov_text_tb += prof_tb() - t_; }
                /* Everything in the renderer that could grow without bound,
                 * on screen. A frame rate that falls while every one of these
                 * holds still is not a leak in the renderer -- which is the
                 * fact this line exists to establish, one way or the other. */
                {
                    char o3[128];
                    gx_render_stats_line(o3, sizeof o3);
                    {
                        size_t ln = strlen(o3);
                        /* Both the step number and the mask: the step is what
                         * the pad moves, the mask is what the code reads, and
                         * a report that names only one of them cannot be acted
                         * on by someone reading gx_features.h. */
                        snprintf(o3 + ln, sizeof o3 - ln, "  GFX%u=%04x",
                                 s_gfx_step, g_gx_state_mask);
                    }
                    {   u64 t_ = prof_tb();     /* PHASE PROFILE */
                    rsx_draw_text_scaled(fb, p0, 43, 143, 0xFF000000u, 2, o3);
                    rsx_draw_text_scaled(fb, p0, 40, 140, 0xFF90FF90u, 2, o3);
                        s_ov_text_tb += prof_tb() - t_; }
                }
                /* Decoded-texture thumbnails along the bottom: the one-glance
                 * verdict on whether texture decode works on this console. */
                {   u64 t_ = prof_tb();         /* PHASE PROFILE */
                /* Debug thumbnails retired: 12 x 96x96 = 110,592 iterations
                 * per presented frame, each doing TWO 32-bit integer divides
                 * (~20-40 non-pipelined cycles each on the PPE) and striding
                 * across RSX local memory with no locality. They existed to
                 * prove textures decoded at all, which the game itself now
                 * demonstrates every frame. */
                    s_ov_thumb_tb += prof_tb() - t_; s_ov_thumb_n++; }
            }
            prof_exit();                /* PHASE PROFILE: overlay ends */
        overlay_done: ;
        }
        /* PHASE PROFILE: flip submission + surface setup.  The stall inside
         * rsx_frame_end's poll loop is charged to PH_WAITFLIP by rsx_video.c,
         * so what is left here is the cost of *submitting* the frame. */
        g_wd_mark = WD_PRESENT;
        prof_enter(PH_PRESENT);
        bench_capture();                /* before the flip swaps buffers */
        {   extern u64 g_draw_frame_base;
            g_draw_frame_base = g_gx_render.draws;
        }
        if (s_analyze_shot && --s_analyze_shot == 0) {
            extern u32 g_draw_win_min, g_draw_win_max;
            {
                char sp2[64];
                snprintf(sp2, sizeof sp2, "/dev_hdd0/tmp/shot-%llu.ppm",
                         (unsigned long long)g_rsx.frames);
                rsx_video_screenshot(sp2, 1u, 255u);   /* last_queued */
                emitf("devlink: analyze shot %s", sp2);
            }
            g_draw_win_min = 0; g_draw_win_max = 0;   /* back to full render */
            emit_line(NULL, "devlink: analyze capture done");
        }
        rsx_frame_end();                /* flip: costs one display interval */
        rsx_frame_begin();
        rsx_clear(0xFF101018u);
        prof_exit();                    /* PHASE PROFILE */
    }
    /* Controller: the PS3 pad becomes the Wiimote, read once per presented
     * frame. Cross=A, Circle=B, Start=+, Select=-, dpad through, Triangle=HOME,
     * Square=1, R1=2 -- enough to drive every menu. */
    prof_enter(PH_PAD);                 /* PHASE PROFILE */
    {
        static int pad_inited;
        padInfo pi2;
        padData pd;
        if (!pad_inited) {
            ioPadInit(7); pad_inited = 1;
            LOG_INFO(LOG_CORE, "pad: ioPadInit done");
        }
        {
            static int pad_seen = -1;
            int now_seen = (ioPadGetInfo(&pi2) == 0 && pi2.status[0]) ? 1 : 0;
            if (now_seen != pad_seen) {
                pad_seen = now_seen;
                LOG_INFO(LOG_CORE, "pad: controller %s", now_seen ? "DETECTED" : "not present");
            }
        }
        /* Injected buttons fire even with NO physical PS3 pad attached --
         * unattended input must not depend on a controller being paired to
         * the PS3 itself. Previously the injection lived inside the
         * pad-present block, so with no pad it never reached WPAD and the game
         * could not be driven from the host at all. */
        s_inj_hold   = s_inject_frames ? s_inject_buttons : 0;
        s_inj_active = s_inject_frames > 0;
        if (s_inj_active) s_inject_frames--;
        if (!(ioPadGetInfo(&pi2) == 0 && pi2.status[0] &&
              ioPadGetData(0, &pd) == 0 && pd.len >= 8)) {
            ios_bt_set_buttons(s_inj_hold);
        } else {
            u16 hi = 0, lo = 0;
            if (pd.BTN_LEFT)     hi |= 0x01;
            if (pd.BTN_RIGHT)    hi |= 0x02;
            if (pd.BTN_DOWN)     hi |= 0x04;
            if (pd.BTN_UP)       hi |= 0x08;
            if (pd.BTN_START)    hi |= 0x10;   /* plus  */
            if (pd.BTN_R1)       lo |= 0x01;   /* two   */
            if (pd.BTN_SQUARE)   lo |= 0x02;   /* one   */
            if (pd.BTN_CIRCLE)   lo |= 0x04;   /* B     */
            if (pd.BTN_CROSS)    lo |= 0x08;   /* A     */
            if (pd.BTN_SELECT)   lo |= 0x10;   /* minus */
            if (pd.BTN_TRIANGLE) lo |= 0x80;   /* home  */
            {   /* A devlink-injected press overrides the pad for a few
                 * frames, so the game can be driven from the host. */
                u16 bits = (u16)((hi << 8) | lo);
                if (s_inj_active) bits |= s_inj_hold;
                ios_bt_set_buttons(bits);
            }
            /* L2/R2 step the pipeline-state groups live, so one session can
             * name which group broke the picture instead of one launch per
             * guess.
             *
             * A *list* rather than an increment. The mask used to be stepped
             * as `(mask + 1) & 15`, which was right while there were four
             * groups and became actively wrong once there were fourteen: the
             * first press dropped the viewport, the scissor and both
             * render-to-texture groups on the floor, so the very first step of
             * a bisect changed five things at once. Each entry below adds
             * exactly one group to the one above it, which is the property the
             * bisect depends on.
             *
             * Entry 5 is the build the console has already agreed with. The
             * entries after it are the 3D groups, and the last two are the two
             * possible answers to the winding question culling cannot settle
             * off hardware -- exactly one of them leaves the title screen
             * looking as it does today, and that is the measurement. */
            static const unsigned k_gfx_steps[] = {
                0u,
                GX_STATE_BLEND,
                GX_STATE_BLEND | GX_STATE_ALPHATEST,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
                    GX_STATE_VIEWPORT | GX_STATE_SCISSOR,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
                    GX_STATE_VIEWPORT | GX_STATE_SCISSOR |
                    GX_STATE_EFB_COPY | GX_STATE_EFB_CLEAR,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
                    GX_STATE_VIEWPORT | GX_STATE_SCISSOR |
                    GX_STATE_EFB_COPY | GX_STATE_EFB_CLEAR | GX_STATE_KONST,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
                    GX_STATE_VIEWPORT | GX_STATE_SCISSOR |
                    GX_STATE_EFB_COPY | GX_STATE_EFB_CLEAR | GX_STATE_KONST |
                    GX_STATE_TEXGEN,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
                    GX_STATE_VIEWPORT | GX_STATE_SCISSOR |
                    GX_STATE_EFB_COPY | GX_STATE_EFB_CLEAR | GX_STATE_KONST |
                    GX_STATE_TEXGEN | GX_STATE_LIGHTING,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
                    GX_STATE_VIEWPORT | GX_STATE_SCISSOR |
                    GX_STATE_EFB_COPY | GX_STATE_EFB_CLEAR | GX_STATE_KONST |
                    GX_STATE_TEXGEN | GX_STATE_LIGHTING | GX_STATE_FOG,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
                    GX_STATE_VIEWPORT | GX_STATE_SCISSOR |
                    GX_STATE_EFB_COPY | GX_STATE_EFB_CLEAR | GX_STATE_KONST |
                    GX_STATE_TEXGEN | GX_STATE_LIGHTING | GX_STATE_FOG |
                    GX_STATE_CULL,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
                    GX_STATE_VIEWPORT | GX_STATE_SCISSOR |
                    GX_STATE_EFB_COPY | GX_STATE_EFB_CLEAR | GX_STATE_KONST |
                    GX_STATE_TEXGEN | GX_STATE_LIGHTING | GX_STATE_FOG |
                    GX_STATE_CULL | GX_STATE_INDIRECT,
                GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
                    GX_STATE_VIEWPORT | GX_STATE_SCISSOR |
                    GX_STATE_EFB_COPY | GX_STATE_EFB_CLEAR | GX_STATE_KONST |
                    GX_STATE_TEXGEN | GX_STATE_LIGHTING | GX_STATE_FOG |
                    GX_STATE_CULL | GX_STATE_INDIRECT | GX_STATE_CULL_FLIP,
            };
            {
                static int l1_prev;
                int l1 = pd.BTN_L1 ? 1 : 0;
                if (l1 && !l1_prev) s_overlay_on = !s_overlay_on;
                l1_prev = l1;
                static int l2_prev, r2_prev;
                int l2 = pd.BTN_L2 ? 1 : 0, r2 = pd.BTN_R2 ? 1 : 0;
                const unsigned n_steps =
                    (unsigned)(sizeof k_gfx_steps / sizeof k_gfx_steps[0]);
                if (!s_gfx_bisect_on) { l2 = r2 = 0; }
                if (r2 && !r2_prev && s_gfx_step + 1 < n_steps)
                    g_gx_state_mask = k_gfx_steps[++s_gfx_step];
                if (l2 && !l2_prev && s_gfx_step > 0)
                    g_gx_state_mask = k_gfx_steps[--s_gfx_step];
                l2_prev = l2; r2_prev = r2;
            }
            /* Left stick = IR pointer, absolute: centred stick is centred
             * cursor. MKWii's menus are pointer-driven; without IR dots the
             * cursor does not exist and A lands nowhere. */
            if (!s_ptr_hold)
                ios_bt_set_pointer((float)pd.ANA_L_H / 255.0f,
                                   (float)pd.ANA_L_V / 255.0f);
        }
    }
    prof_exit();                        /* PHASE PROFILE */

    /* The vertex buffer is reset every frame regardless -- only the *display*
     * runs at a slower rate than the title. Starting a GPU frame (and clearing)
     * for every title frame would queue thirty times the commands between
     * flips, which is a good way to overrun the command ring. */
    gx_render_frame_begin();

    /* PHASE PROFILE: one breakdown every PROF_REPORT_EVERY presented frames. */
    if (g_prof.enabled && (s_mkw_frames % PROF_REPORT_EVERY) == 0)
        prof_report_interval();
    gx_worker_resume();   /* hand the FIFO back to the GP thread */
}

/* The Wii low-memory OS globals a title finds at boot (verified against
 * Dolphin's SetupWiiMemory / IOS36). Verbatim from tools/bootgame.c. */
static void mkw_setup_globals(u32 fst_addr, u32 fst_size)
{
    u32 a;
    for (a = 0x80000000u; a < 0x80004000u; a += 4) mkw_w32(a, 0);
    mkw_w32(0x80000000, 0x524D4345); mkw_w32(0x80000004, 0x30310000);
    mkw_w32(0x80000018, 0x5D1C9EA3);
    mkw_w32(0x80000020, 0x0D15EA5E); mkw_w32(0x80000024, 0x00000001);
    mkw_w32(0x80000028, 0x01800000); mkw_w32(0x8000002C, 0x00000023);
    mkw_w32(0x80000030, 0x00000000); mkw_w32(0x80000034, 0x817FEC60);
    mkw_w32(0x80000038, fst_addr);   mkw_w32(0x8000003C, fst_size);
    mkw_w32(0x800000CC, 0x00000000); mkw_w32(0x800000E4, 0x8008F7B8);
    mkw_w32(0x800000F0, 0x01800000); mkw_w32(0x800000F4, 0x8179B500);
    mkw_w32(0x800000F8, 0x0E7BE2C0); mkw_w32(0x800000FC, 0x2B73A840);
    {
        static const u32 vec[] = {0x100,0x200,0x300,0x400,0x500,0x600,0x700,
            0x800,0x900,0xC00,0xD00,0xF00,0x1300,0x1400,0x1700};
        unsigned k;
        for (k = 0; k < sizeof vec/sizeof vec[0]; k++)
            mkw_w32(0x80000000u + vec[k], 0x4C000064u);   /* rfi */
    }
    mkw_w32(0x800030D8, 0xFFFFFFFF); mkw_w32(0x800030E4, 0x00008201);
    mkw_w32(0x80003100, 0x01800000); mkw_w32(0x80003104, 0x01800000);
    mkw_w32(0x80003108, 0x81800000); mkw_w32(0x8000310C, 0x00000000);
    mkw_w32(0x80003110, 0x81800000); mkw_w32(0x80003114, 0xDEADBEEF);
    mkw_w32(0x80003118, 0x04000000); mkw_w32(0x8000311C, 0x04000000);
    mkw_w32(0x80003120, 0x93600000); mkw_w32(0x80003124, 0x90000800);
    mkw_w32(0x80003128, 0x935E0000); mkw_w32(0x8000312C, 0xDEADBEEF);
    mkw_w32(0x80003130, 0x935E0000); mkw_w32(0x80003134, 0x93600000);
    mkw_w32(0x80003138, 0x00000011); mkw_w32(0x8000313C, 0xDEADBEEF);
    mkw_w32(0x80003140, 0x00240E18); mkw_w32(0x80003144, 0x00030110);
    mkw_w32(0x80003148, 0x93600000); mkw_w32(0x8000314C, 0x93620000);
    mkw_w32(0x80003150, 0xDEADBEEF); mkw_w32(0x80003154, 0xDEADBEEF);
    mkw_w32(0x80003158, 0x0000FF01); mkw_w32(0x8000315C, 0x80AD0113);
    mkw_w32(0x80003180, 0x524D4345); mkw_w32(0x80003184, 0x80000000);
    mkw_w32(0x80003188, 0x00240E18); mkw_w32(0x80003198, 0x03E00000);
}

/* Run the real game on the PPE and draw a status screen on the TV. */
/* The decisive measurement for the static-recompilation pivot: run one real
 * MKWii paired-single routine (fn_math, 0x80199CD8, 21 paired-single ops)
 * three ways -- interpreter, runtime JIT, and the statically recompiled native
 * function -- on the actual PPE, timed with the hardware time base. This is the
 * number host and qemu cannot give: the in-order core's true recomp-vs-JIT
 * ratio. */
extern void fn_math(PPCState *s);
void rec_fallback(PPCState *s, u32 pc, u32 op) { (void)s; (void)pc; (void)op; }

static void recomp_benchmark_stage(void)
{
    static PPCState bc;
    DolHeader h;
    const u32 A = 0x80300000u, B = 0x80300100u;
    const u32 OI = 0x80301000u, OJ = 0x80301500u, OR = 0x80302000u;
    const u32 FUNC = 0x80199CD8u, SENT = 0x80303000u;
    unsigned i;

    stage("static recompiler benchmark (fn_math: 21 paired-single ops)");
    /* bc is a private CPU state: give it the constant/scale tables the JIT
     * reads for quantised loads (the interpreter and the recompiler compute
     * scales directly and do not need them, which is why interp ran and the
     * JIT faulted on the zero-filled tables). */
    memset(&bc, 0, sizeof bc);
    ppc_init_constants(&bc);
    if (dol_load(mkwii_dol_blob, (u32)(mkwii_dol_blob_end - mkwii_dol_blob), &h) != 0) {
        emit_line(NULL, "   dol load failed"); return;
    }
    for (i = 0; i < 48; i++) { union { float f; u32 u; } c; c.f = (float)(i%7)*0.5f - 1.25f; mem_write32(A + i*4, c.u); }
    for (i = 0; i < 48; i++) { union { float f; u32 u; } c; c.f = (float)((i*3)%11)*0.3f - 1.0f; mem_write32(B + i*4, c.u); }
    mem_write32(SENT, 0x48000000u);   /* b .  -- a valid, backed return target */
    emit_line(NULL, "   [bm] dol+inputs ok");

#define RC_SETUP(out) do { memset(bc.gpr,0,sizeof bc.gpr); memset(bc.ps,0,sizeof bc.ps); \
    bc.gpr[1]=0x816F0000u; /* valid guest stack: JIT fallbacks push here */ \
    bc.gpr[3]=A; bc.gpr[4]=B; bc.gpr[5]=(out); bc.gpr[6]=1; \
    bc.msr|=MSR_FP; bc.hid2|=HID2_PSE|HID2_LSQE; bc.gqr[0]=0; } while(0)

    /* --- SAFE FIRST: interpreter vs recompiler (both proven on this PPE) --- */
    { unsigned g=0; RC_SETUP(OI); bc.pc=FUNC; bc.lr=SENT;
      while (bc.pc!=SENT){ interp_step(&bc); if(++g>100000) break; } }
    { RC_SETUP(OR); fn_math(&bc); }
    { int d=0; for(i=0;i<64;i++) if(mem_read8(OI+i)!=mem_read8(OR+i))d++;
      emitf("   correctness: interp==recomp %s (%d diff)", d?"FAIL":"ok", d); }
    { const unsigned long N=400000UL; volatile u32 sink=0; unsigned long k; u64 t0,t1,t2; unsigned g;
      t0=read_timebase();
      for(k=0;k<N;k++){ mem_write8(A,(u8)k); g=0; RC_SETUP(OI); bc.pc=FUNC; bc.lr=SENT;
        while(bc.pc!=SENT){interp_step(&bc); if(++g>100000)break;} for(i=0;i<64;i++)sink+=mem_read8(OI+i); }
      t1=read_timebase();
      for(k=0;k<N;k++){ mem_write8(A,(u8)k); RC_SETUP(OR); fn_math(&bc); for(i=0;i<64;i++)sink+=mem_read8(OR+i); }
      t2=read_timebase();
      double ti=(double)(t1-t0)/PS3_TIMEBASE_HZ, tr=(double)(t2-t1)/PS3_TIMEBASE_HZ;
      emitf("   interp: %d.%03d s  (%lu k/s)", (int)ti,(int)(ti*1000)%1000,(unsigned long)(ti>0?N/ti/1000:0));
      emitf("   RECOMP: %d.%03d s  (%lu k/s)", (int)tr,(int)(tr*1000)%1000,(unsigned long)(tr>0?N/tr/1000:0));
      if (tr>0) emitf("   >>> RECOMP vs INTERP on real PPE: %d.%02dx  <<<", (int)(ti/tr),(int)(ti/tr*100)%100);
      (void)sink; }

    emit_line(NULL, "   (JIT-in-isolation measurement deferred: faults executing a single");
    emit_line(NULL, "    block outside the dispatch loop; recomp-vs-interp above is the clean result)");

}

PPCState *g_live_cpu;   /* devlink `pc` reads the running guest state */

/* ---- GX worker: the whole GX pipeline on the PPE's second hardware thread.
 * Armed by /dev_hdd0/tmp/dolphin-gxthread.txt so the change can be A/B'd
 * against the synchronous path with a file touch. ---- */
#include <sys/thread.h>
#include <sys/sem.h>
static sys_sem_t s_gx_sem;
static volatile int s_gx_kick;
static int s_gx_worker_on;

/* Posted-vs-completed handoff between the recompiler thread and the GX worker.
 *
 * The PPE's second hardware thread is the most valuable idle resource on this
 * machine for THIS workload: the frame is ~79% recompiler and the recompiler
 * is memory-stall-bound, and an SMT sibling runs in exactly those stall
 * cycles. But the worker owns the renderer -- gx_state_run walks the FIFO and
 * writes the RSX command buffer -- and the main thread presents. Without a
 * handoff both write the command ring at once. Armed as originally written it
 * reached frame 41 and stopped.
 *
 * So: the main thread may kick freely while it executes guest code, and must
 * drain before it touches the RSX itself. Counters rather than a flag, because
 * kicks can queue up behind a slow parse and "busy" would miss the queued
 * ones. */
static volatile u32 s_gx_posted, s_gx_done;

/* Consume the FIFO CONTINUOUSLY, the way the real graphics processor does.
 *
 * The first design kicked the worker once per CPU slice and waited on a
 * semaphore. Measured, that was 24% SLOWER than doing the work in-line (3.171
 * vs 4.160 fps) and the guest executed three times as many instructions for
 * the same 300 frames -- 9.1 billion against 3.06. The reason is coupling the
 * emulator did not invent: the title polls the FIFO read pointer to pace
 * itself, so any latency added to consumption is paid back as guest spin. A
 * kick-per-slice hands the GP work to another thread and then makes the guest
 * wait for it.
 *
 * Hardware does not work that way, so neither should this: the worker spins on
 * the FIFO and drains it as it arrives, keeping the read pointer close behind
 * the write pointer. Spinning an SMT sibling is affordable here precisely
 * because the recompiler thread is memory-stall-bound -- it is not using the
 * issue slots this loop consumes. `paused` lets the main thread take the
 * renderer back without stopping the thread. */
static volatile int s_gx_pause, s_gx_idle;

static void gx_worker(void *arg)
{
    (void)arg;
    for (;;) {
        if (__atomic_load_n(&s_gx_pause, __ATOMIC_SEQ_CST)) {
            __atomic_store_n(&s_gx_idle, 1, __ATOMIC_SEQ_CST);
            usleep(50);
            continue;
        }
        __atomic_store_n(&s_gx_idle, 0, __ATOMIC_SEQ_CST);
        if (gx_state_run(gx_state()) == 0)
            usleep(20);          /* nothing to do; do not burn the bus */
        __atomic_add_fetch(&s_gx_done, 1, __ATOMIC_SEQ_CST);
    }
}

/* Wait until the worker has consumed every kick issued so far. Bounded: a
 * worker that dies must not take the emulator with it, and a wedge here is
 * invisible from outside (it was, for one whole session). */
/* Park the worker so this thread can own the RSX (present, overlay, capture).
 * Bounded: a worker that dies must not take the emulator with it. */
static void gx_worker_drain(void)
{
    unsigned spins = 0;
    if (!s_gx_worker_on) return;
    __atomic_store_n(&s_gx_pause, 1, __ATOMIC_SEQ_CST);
    while (!__atomic_load_n(&s_gx_idle, __ATOMIC_SEQ_CST)) {
        if (++spins > 20000000u) {
            LOG_WARN(LOG_CORE, "gx worker: PARK TIMEOUT, in-line GX from here");
            s_gx_worker_on = 0;
            __atomic_store_n(&s_gx_pause, 0, __ATOMIC_SEQ_CST);
            return;
        }
    }
}

static void gx_worker_resume(void)
{
    if (s_gx_worker_on)
        __atomic_store_n(&s_gx_pause, 0, __ATOMIC_SEQ_CST);
}
static void gx_worker_start(void)
{
    FILE *f = fopen("/dev_hdd0/tmp/dolphin-gxthread.txt", "rb");
    sys_ppu_thread_t t;
    sys_sem_attr_t attr;
    if (!f) return;
    fclose(f);
    memset(&attr, 0, sizeof attr);
    attr.attr_protocol = SYS_SEM_ATTR_PROTOCOL;
    attr.attr_pshared  = SYS_SEM_ATTR_PSHARED;
    if (sysSemCreate(&s_gx_sem, &attr, 0, 4096) != 0) return;
    if (sysThreadCreate(&t, gx_worker, NULL, 1500, 0x8000, 0, "gxworker") == 0) {
        s_gx_worker_on = 1;
        LOG_INFO(LOG_CORE, "GX worker thread armed (PPE SMT)");
    }
}

static void mkwii_boot_stage(void)
{
    static PPCState cpu;
    g_live_cpu = &cpu;
    DolHeader h;
    /* Power-on scrub. The boot selftests (difftest at 0x80300000, realtest's
     * GUEST_SENTINEL at 0x80500000, benchmark code and data) leave droppings
     * all over guest RAM. Real hardware boots the title with zeroed memory;
     * MKWii reads some heap fields it never initialises, and those reads
     * landing on our leftovers produced an INTERMITTENT race-load crash:
     * a jump to 0x80500000 (the realtest sentinel address, which realtest had
     * zeroed) -> illegal-instruction storm at vector 0x700. Scrub both RAM
     * banks to the power-on state before the title boots. */
    {
        u8 *m1 = mem_ptr(0x80000000u);
        u8 *m2 = mem_ptr(0x90000000u);
        if (m1) memset(m1, 0, 24u * 1024u * 1024u);
        if (m2) memset(m2, 0, 64u * 1024u * 1024u);
        jit_flush_all();
    }
    gx_worker_start();
    {   extern void spu_vtx_init(void);
        spu_vtx_init();
    }
    u32 dol_size = (u32)(mkwii_dol_blob_end - mkwii_dol_blob);
    u32 fst_size = (u32)(mkwii_fst_blob_end - mkwii_fst_blob);
    /* The filesystem must live ABOVE the arena: an apploader loads it just
     * under ArenaHi and lowers ArenaHi beneath it, so the title's own allocator
     * can never overwrite the table it later needs to find files by name.
     * Leaving it at a fixed address inside the arena is exactly what stopped
     * the console run -- the FST was overwritten, every lookup failed, nothing
     * was read from the disc, and the resource the scene wanted stayed empty. */
    u32 fst_addr = (0x817FEC60u - (u32)(mkwii_fst_blob_end - mkwii_fst_blob)) & ~0x1Fu;
    unsigned slices;
    u64 total_insts = 0;
    u32 win_lo = 0xFFFFFFFFu, win_hi = 0, stuck = 0;
    int halted = 0;
    char panic[160]; panic[0] = 0;

    stage("MKWii: boot real main.dol on the PPE");

    mem_reset();
    hw_init(&cpu, 1);
    pi_irq_exit_flag_init();
    slice_cap_flag_init();
    {   extern int g_warm_no_cold;
        FILE *wf = fopen("/dev_hdd0/tmp/dolphin-nowarmcold.txt", "r");
        if (wf) { fclose(wf); g_warm_no_cold = 1; }
        LOG_INFO(LOG_CORE, "warm self-loop w/ interior exits %s",
                 g_warm_no_cold ? "REFUSED by flag" : "allowed (default)");
    }
    {   extern int g_no_psq_store;
        FILE *qf = fopen("/dev_hdd0/tmp/dolphin-nopsqst.txt", "r");
        if (qf) { fclose(qf); g_no_psq_store = 1; }
        LOG_INFO(LOG_CORE, "compiled psq_st %s",
                 g_no_psq_store ? "DISABLED by flag" : "enabled");
    }
    {   FILE *nf = fopen("/dev_hdd0/tmp/dolphin-noni.txt", "r");
        if (nf) { fclose(nf); g_ni_sync_off = 1; }
        LOG_INFO(LOG_CORE, "guest FPSCR[NI] -> host mirror %s",
                 g_ni_sync_off ? "DISABLED by flag" : "enabled");
    }
    spu_scan_flag_init();
    hw_reset();
    jit_flush_all();

    /* Audio out. The emulated DSP fills a ring whether or not anybody is
     * listening; this is what drains it to the speakers. */
    if (g_audio_enable && audio_ps3_init() == 0 && g_audio_enable)
        emit_line(NULL, "   audio: port open");
    else
        emit_line(NULL, "   audio: disabled");

    /* Bind the real renderer so the title's draws reach the RSX rather than
     * being counted and dropped. */
    {
        static GXBackend be;
        if (gx_render_init() != 0) {
            emit_line(NULL, "   gx_render init FAILED");
        } else {
            gx_render_bind(&be);
            gx_state_init(gx_state(), &be);
            gx_render_set_frame_handler(mkwii_frame_done, NULL);
            rsx_frame_begin();
            rsx_clear(0xFF203A6Au);
            gx_render_frame_begin();
        }
    }

    if (dol_load(mkwii_dol_blob, dol_size, &h) != 0) {
        emit_line(NULL, "   MKWii DOL did not load");
        return;
    }
    emitf("   DOL: entry %08x, %u bytes", (unsigned)h.entry_point, dol_size);
    dol_setup_boot_state(&cpu, &h, 1);

    mem_write_block(fst_addr, mkwii_fst_blob, fst_size);

    /* The complete disc, streamed from the HDD -- 2.5 GiB of content built by
     * tools/mkfullslice.py from the filesystem itself, so every file the game
     * can open is present. The embedded few-megabyte boot slice remains as
     * the fallback so the app still boots to the strap screen without the
     * upload; the boot ran off that slice's edge the first time the game
     * opened an archive the boot had not touched (ARCInitHandle fatal). */
    if (disc_slice_open("/dev_hdd0/game/DOLPHIN01/USRDIR/mkwii_full.slice")
            == 0)
        emit_line(NULL, "   full disc mounted from HDD (2.5 GiB, streamed)");
    else
        emit_line(NULL, "   FATAL: no disc slice on HDD "
                  "(/dev_hdd0/game/DOLPHIN01/USRDIR/mkwii_full.slice)");
    /* The 17.8 MiB embedded fallback slice is gone: it cost more as resident
     * RAM (starving the NAND file allocator that rksys.dat needs) than it was
     * worth as a convenience. */

    /* Heap headroom probe: the save flow needs a ~3 MiB allocation to live.
     * Say out loud what the largest available block is. */
    {
        u32 mb;
        for (mb = 64; mb >= 1; mb >>= 1) {
            void *pr = malloc((size_t)mb << 20);
            if (pr) { free(pr); break; }
        }
        emitf("   heap probe: largest block >= %u MiB", mb);
    }

    ios_di_set_partition(0xF800000ull);
    ios_es_set_title(0x0001000000000000ull |
                     ((u64)('R'<<24 | 'M'<<16 | 'C'<<8 | 'E')));
    ios_fs_provision_wc24();
#define NAND_DIR "/dev_hdd0/game/DOLPHIN01/USRDIR/nand"
    /* The authentic WiiConnect24 NAND defaults. Every one of these was
     * previously answered ENOENT, and MKWii's save/WC24 path reads them. */
#define WC24_EXTERN(np, ap, sym) \
    extern const unsigned char wc24_##sym##_blob[], wc24_##sym##_blob_end[];
    WII_NAND_DEFAULT_FILES(WC24_EXTERN)
#undef WC24_EXTERN
    /* Writable copies: the blobs live in .rodata and the title writes to
     * several of these (mailbox control blocks especially). */
#define WC24_REG(np, ap, sym)                                                 \
    { u32 n_##sym = (u32)(wc24_##sym##_blob_end - wc24_##sym##_blob);         \
      u8 *b_##sym = (u8 *)malloc(n_##sym);                                    \
      if (b_##sym) {                                                          \
          memcpy(b_##sym, wc24_##sym##_blob, n_##sym);                        \
          ios_fs_register_file((np), b_##sym, n_##sym);                       \
      } else emitf("   WC24: alloc failed for %s", (np)); }
    WII_NAND_DEFAULT_FILES(WC24_REG)
#undef WC24_REG
    {   /* The shipped blobs must be registered BEFORE the persisted state is
         * restored over them. Registered the other way round, the restore
         * found no SYSCONF entry, created a zero-length one, and "restored"
         * it by reading zero bytes -- so the console booted with a 16 KB
         * SYSCONF read returning 0 while qemu returned 16384.
         *
         * That single difference is what broke the controller: with no
         * SYSCONF the title never builds its registered-device table, so the
         * 6-byte BD_ADDR compare at guest 0x801cf648 fails and it answers our
         * HID channel with L2CAP result 3 and hangs up. It also shifted every
         * downstream timer, which is why the two machines' clocks parted at
         * slice 340 and never re-converged. */
        static u8 sysconf[0x4000], setting[0x100];
        memcpy(sysconf, mkwii_sysconf_blob, sizeof sysconf);
        memcpy(setting, mkwii_setting_blob, sizeof setting);
        ios_fs_register_file("/shared2/sys/SYSCONF", sysconf, 0x4000);
        ios_fs_register_file("/title/00000001/00000002/data/setting.txt",
                             setting, 0x100);
    }

    /* Anything a previous session wrote wins over the shipped defaults. */
    mkdir(NAND_DIR, 0777);
    ios_fs_persist_load(NAND_DIR);

    /* Statically recompiled hot functions, difftest-proven bit-exact and
     * full-boot equivalence-proven under qemu. Registered after the DOL is
     * in guest memory; the enable call is the integrity gate. */
    aot_register_all();
    {   /* WiiCompiled-translated functions (src/core/ppc/wc): opt-in per
         * boot so an A/B holds every other AOT body constant. */
        extern void wc_register_all(void);
        FILE *wf = fopen("/dev_hdd0/tmp/dolphin-wc.txt", "r");
        if (wf) { fclose(wf); wc_register_all();
                  LOG_INFO(LOG_JIT, "WC: translated functions registered"); }
    }
    {   /* Boot-time AOT switch. Toggling AOT over devlink mid-run cannot undo
         * a divergence that already happened -- the guest's memory is already
         * wrong by then -- so an honest A/B has to start with it off. */
        FILE *af = fopen("/dev_hdd0/tmp/dolphin-aot.txt", "r");
        int want = 1;
        if (af) { if (fscanf(af, "%d", &want) != 1) want = 1; fclose(af); }
        if (want) jit_aot_enable_all();
        else LOG_INFO(LOG_JIT, "AOT left DISABLED at boot by request");
    }
    mkw_setup_globals(fst_addr, fst_size);
    mem_write32(0x80000034u, fst_addr);   /* ArenaHi now sits below the FST */
    mem_write32(0x80003110u, fst_addr);

#ifdef WC_GAME_LINKED
    /* THE PORT. With /dev_hdd0/tmp/dolphin-wcboot.txt present, the game runs as
     * the statically recompiled native code linked into this image instead of
     * being emulated: no JIT, no dispatcher, no interpreter. Everything set up
     * above still applies -- guest RAM, the disc, IOS, the RSX backend and the
     * device model are the same, because a port needs all of that too. What
     * changes is who executes the guest's instructions.
     *
     * Behind a flag because the two cannot run at once and the emulator has to
     * stay available: it is the reference the port is validated against. */
    {   FILE *wf = fopen("/dev_hdd0/tmp/dolphin-wcboot.txt", "r");
        if (wf) {
            extern int wc_boot(void);
            fclose(wf);
            /* ONE SHOT. The flag is deleted before the port starts, so a boot
             * that wedges the console costs exactly one launch instead of
             * every launch after it.
             *
             * This is not hypothetical: the first threaded boot starved lv2 of
             * every other thread -- FTP, the rescue listener and devlink all
             * stopped answering -- and with the flag still on disk there was
             * no way to ask for a safe boot instead. The console had to be
             * power-cycled, and would have wedged again on the next launch.
             * Ask for port mode again by writing the flag again. */
            remove("/dev_hdd0/tmp/dolphin-wcboot.txt");
            LOG_INFO(LOG_CORE, "WC: native port mode -- the JIT will not run");
            /* Low-mem canary: arm NOW, before the first guest instruction --
             * the device-slice arming raced the ctor-pass stomp and could
             * lose (a boot with no CANARY line at all was the arm race lost,
             * not the stomp fixed). The disc id is already in place. The
             * 50 us watcher thread starts here too: its old spawn site was
             * the OTHER flag-read branch, which fast boot never executes. */
            /* The canary ARMS inside wc_boot (after wc_data_init: the
             * arena must be live -- arming here read through a NULL
             * g_wc_arena and killed the process before the first log line
             * could say why). The watcher only polls the armed pointer, so
             * spawning it before the arm is safe. */
            {   sys_ppu_thread_t ct;
                if (sysThreadCreate(&ct, canary_watcher, NULL, 900, 0x4000,
                                    0, "canary") == 0)
                    LOG_INFO(LOG_CORE, "WC: canary watcher up (50 us poll)");
            }
            /* Starts the game on its own thread and returns. Execution
             * continues into the ordinary loop below, which still drives
             * timing, the devices, the FIFO and the flip -- everything except
             * jit_run, which g_wc_running suppresses. That loop is what
             * delivers the retrace the game is about to wait for. */
            wc_boot();
        }
    }
#endif

    /* Put something on screen before the long part starts: the boot runs for
     * hundreds of millions of instructions before the title draws anything, and
     * an unexplained black screen is indistinguishable from a hang. */
    if (g_rsx.inited) {
        int b0 = 0, p0 = (int)g_rsx.pitch;
        rsx_fill_cpu(b0, 0xFF101830u);
        rsx_draw_text_scaled(g_rsx.buffer[b0], p0, 60, 60,  0xFFFFFFFFu, 6,
                             "MARIO KART WII");
        rsx_draw_text_scaled(g_rsx.buffer[b0], p0, 60, 150, 0xFFC0D0FFu, 3,
                             "BOOTING ON THE PPE - THIS TAKES A MINUTE");
        rsx_draw_text_scaled(g_rsx.buffer[b0], p0, 60, 200, 0xFF90FF90u, 3,
                             "SCREEN UPDATES ONCE IT STARTS DRAWING");
        rsx_present_cpu(b0);
    }

    /* Verify the entry code landed in guest memory and that the JIT compiles
     * its first block -- the console is the first time MKWii runs through the
     * recompiler (the host JIT is compile-only), so confirm the basics. */
    emitf("   entry@800060a4: %08x %08x %08x  pc=%08x",
          (unsigned)mem_read32(0x800060a4u), (unsigned)mem_read32(0x800060a8u),
          (unsigned)mem_read32(0x800060acu), (unsigned)cpu.pc);
    if (!g_wc_running) {
        /* Port mode skips jit_init entirely; this sanity compile then wedged
         * the main thread before the device loop ever started -- both
         * fast-boot console runs froze at exactly this point (seq=0, game
         * thread stranded 188 calls in). The port needs no first block. */
        JitBlock *blk = jit_get_block(&cpu, cpu.pc);
        emitf("   first block: %s, %u guest insts",
              blk ? "compiled" : "NULL(failed)", blk ? blk->guest_insts : 0);
    }

    {
    u32 first_pc = cpu.pc, after_first = 0;
    const char *why = "budget";
    u64 t_wall0 = read_timebase();
    /* PHASE PROFILE: arm the accounting.  Nothing above this point is timed,
     * so the self-test stages and the pre-loop setup stay untouched. */
    prof_reset();
    s_prof_f0 = s_mkw_frames;      s_prof_i0 = s_mkw_insts;
    s_prof_ii0 = s_idle_insts;
    s_prof_d0 = g_gx_render.draws; s_prof_v0 = g_gx_render.vertices;
    /* The bound is wall-clock, not slices: with the full disc the game runs
     * indefinitely (menus, attract mode), and a slice count translates to an
     * unpredictable session length once frames are vsync-paced. Ten minutes,
     * then a clean report. */
    /* 100M slices ~= 1251 s at the measured ~80k slices/s: every "20-minute
     * death" of the port era was THIS cap ending the session ("stop=budget"),
     * not a guest fault. The port runs indefinitely; the 4-hour wall backstop
     * below still guards a wedged run. */
    for (slices = 0; g_wc_running || slices < 100000000u; slices++) {
        s32 grant;
        /* The game is playable now, so the old ten-minute bring-up cap just
         * ended sessions mid-play. Four hours is a backstop against a wedged
         * run holding the console, not a play limit; PS button quits. */
        if ((slices & 2047u) == 0 &&
            (double)(read_timebase() - t_wall0) / PS3_TIMEBASE_HZ > 14400.0) {
            why = "session time (4 h)";
            break;
        }
        /* Service the XMB queue often enough that "Quit Game" is instant.
         * Every 2048 slices is well under a millisecond of latency and costs
         * nothing measurable. */
        if ((slices & 255u) == 0) {
            /* YIELD. The emulation loop is a 100%-CPU loop on one PPE thread,
             * and starving the system is not free: webMAN's FTP service died
             * outright during a session and did not come back, which cost the
             * only remote route for deploying a build. The XMB needs time for
             * the same reason. A yield costs nothing when no other thread is
             * ready to run, and it is the difference between a console that
             * stays usable during a session and one that does not. */
            sysThreadYield();
            devlink_poll();
            sysUtilCheckCallback();
            if (s_exit_requested) { why = "XMB quit"; break; }
            /* PAUSE FOR THE XMB. The emulation loop is a 100%-CPU loop that
             * never yields, and the in-game XMB needs both processor time and
             * the GPU to draw its own overlay. Merely not presenting was not
             * enough -- the console still locked -- because we kept the core
             * pinned. While the menu is up we stop emulating entirely and
             * sleep in short slices, servicing the callback queue so the menu
             * stays responsive and the resume event is seen promptly. The
             * picture holding still under the menu is the normal behaviour of
             * a paused title. */
            while (s_xmb_menu_open && !s_exit_requested) {
                usleep(4000);
                sysUtilCheckCallback();
            }
            if (s_exit_requested) { why = "XMB quit"; break; }
        }
        if (!halted && cpu.pc >= 0x8012e508u && cpu.pc <= 0x8012e514u) {
            unsigned k;
            halted = 1;
            for (k = 0; k < sizeof panic - 1; k++) {
                u8 c = mem_read8(0x81400000u + k);
                if (!c) break;
                panic[k] = (c >= 32 && c < 127) ? (char)c : '.';
            }
            panic[k] = 0;
            why = "halt"; break;
        }
        /* ---- PHASE PROFILE: the four things a slice is made of ---------
         * Times are exclusive, so gx_state_run below reports only the FIFO
         * walk: the renderer it calls, and the whole present path the EFB copy
         * triggers, are charged to their own phases. ------------------------ */
        prof_enter(PH_TIMING);
        grant = timing_slice();
        prof_exit();
        /* Mirror the guest's non-IEEE mode into the host FPSCR.
         *
         * Gekko FPSCR[NI] means denormalised operands and results are flushed
         * to zero; it is how a title gets predictable FP timing, and MKWii
         * sets it -- every snapshot taken at the stall reads NI=1. The
         * emulator ignored the bit entirely (FPSCR_NI was defined in gekko.h
         * and referenced nowhere), so denormals survive here that hardware
         * would have zeroed. One was sitting in a guest register at the stall:
         * f0 = 0x7d00 = 1.58e-319.
         *
         * The PPE has the same bit, so the hardware does the work: set it and
         * both the recompiled code and the interpreter's own C arithmetic
         * flush the same way Gekko does. Synced once per slice rather than on
         * every FPSCR write, because the host register can be clobbered by
         * anything else that runs on this thread.
         *
         * A differential fuzzer cannot find this class: both sides ignore the
         * bit identically and agree with each other while both differ from the
         * machine. dolphin-noni.txt disables it. */
        if (!g_ni_sync_off) {
            static u32 last_ni = 0xFFFFFFFFu;
            u32 ni = cpu.fpscr & FPSCR_NI;
            if (ni != last_ni) {
                last_ni = ni;
                if (ni) __asm__ __volatile__("mtfsb1 29" ::: "memory");
                else    __asm__ __volatile__("mtfsb0 29" ::: "memory");
            }
        }
        cpu.downcount = grant;
        cpu.exit_requested = 0;
        g_wd_mark = WD_JIT;
        prof_enter(PH_JIT);
#ifdef WC_GAME_LINKED
        /* In port mode the translated game is executing on its own thread;
         * this loop exists only to run the devices for it. */
        if (g_wc_running) {
            prof_exit();
            /* ADVANCE EMULATED TIME FROM THE WALL CLOCK.
             *
             * timing_advance derives elapsed guest cycles from how much of the
             * slice the CPU consumed: granted - downcount. Skipping jit_run
             * leaves downcount at the full grant, so it consumed nothing,
             * emulated time stood still, and not one scheduled device event
             * ever fired. The game got as far as __AIClockInit, waited on the
             * audio interface, and waited forever -- with the device model
             * frozen at t=0 behind it.
             *
             * The port runs the game at native speed, so the honest clock is
             * the real one: guest cycles advance at Broadway's 729 MHz per
             * second of wall time, which is what makes VI retrace land at
             * 60 Hz, audio DMA run at its true rate, and every alarm the game
             * sets expire when the game expects it to.
             *
             * The remainder is carried rather than dropped. A slice can only
             * ever be credited its own grant, so without a carry every slice
             * that ran long would quietly lose the excess and emulated time
             * would drift permanently behind the wall clock. */
            {   static u64 s_wc_tb_last, s_wc_cyc_owed;
                u64 tb = read_timebase(), pay;
                if (!s_wc_tb_last) s_wc_tb_last = tb;
                s_wc_cyc_owed += (tb - s_wc_tb_last) *
                                 (u64)WII_CPU_HZ / PS3_TIMEBASE_HZ;
                s_wc_tb_last = tb;
                pay = s_wc_cyc_owed;
                if (pay > (u64)(u32)grant) pay = (u64)(u32)grant;
                s_wc_cyc_owed -= pay;
                cpu.downcount = grant - (s32)pay;
            }
            goto wc_skip_exec;
        }
#endif
        {   /* Boot-time interpreter switch. The console and qemu charge different
             * cycle counts for identical guest work from slice 357 onward; running
             * both on the interpreter isolates whether that accounting difference
             * comes from the JIT executing natively rather than under qemu. */
            static int use_jit = -1;
            if (use_jit < 0) {
                FILE *jf = fopen("/dev_hdd0/tmp/dolphin-nojit.txt", "r");
                use_jit = 1;
                if (jf) { fclose(jf); use_jit = 0;
                    LOG_INFO(LOG_JIT, "JIT DISABLED at boot by request"); }
            }
            if (use_jit) jit_run(&cpu); else interp_run(&cpu);
            /* Spill-accurate register snapshot.
             *
             * PPCState.ps[] is NOT where a live guest FPR is: the recompiler
             * keeps them in host f14..f31 and spills only at block boundaries.
             * Reading that memory from the devlink thread -- which is what the
             * `fpr` command did -- reports whatever was last spilled, and two
             * diagnoses were drawn from exactly that stale data (§22).
             *
             * jit_run has just returned, which means the block exited and its
             * dirty registers were written back, so PPCState is authoritative
             * at this instant and nowhere else. */
            if (g_snap_req) {
                g_snap_state = cpu;
                g_snap_req   = 0;
                g_snap_ready = 1;
            }
        }
        prof_exit();
#ifdef WC_GAME_LINKED
    wc_skip_exec: ;
#endif
        prof_enter(PH_TIMING);
        /* The device model is shared with the game thread in port mode, which
         * runs its MMIO accesses on its own thread. Advance it under the same
         * lock those accesses take. */
        {   extern volatile unsigned g_devloop_hb;
            g_devloop_hb++;
        }
        dev_lock();
        dev_lock_tag("devloop-timing");
        timing_advance(&cpu);
        /* Release queued IPC replies every slice. The emulator never needed
         * this call here because pre-latency-floor replies were released
         * synchronously on the guest's own ack write; the 100 us floor made
         * the periodic retry the only release path, and in port mode this
         * loop is the only place it can live. Without it the very first
         * deferred reply (the STM open at boot) never reached the guest and
         * the port froze at ~1k calls with nothing pending. */
        ipc_update();
        {   /* CALL-FREE RETRACE-POLL ESCAPE (fiber build). A guest spin that
             * polls the retrace count in RAM with zero dispatches can never
             * be reached by the boundary pump; with the VI line pending the
             * machine freezes (measured at RKSystem's third heap create,
             * calls static, intsr=0x100). Nudge the ONE counter the VI
             * handler would increment; the spin exits, the next boundary
             * delivers the real handler. Rate-limited to the frame period. */
            extern volatile unsigned long long g_wc_calls_probe_last;
            extern unsigned wc_calls_probe(void);
            extern uint32_t pi_intsr_raw(void), pi_intmr_raw(void);
            static unsigned last_calls, still;
            static u64 last_nudge;
            unsigned now_calls = wc_calls_probe();
            if ((pi_intsr_raw() & pi_intmr_raw() & 0x100u) &&
                now_calls == last_calls) {
                if (++still > 200) {   /* ~ms of call-free with VI pending */
        {   /* Arm the back-edge delivery points whenever a delivery could
             * actually land; the poll itself stays a single flag test. */
            extern volatile unsigned g_wc_backedge_arm;
            extern u32 pi_intsr_raw(void), pi_intmr_raw(void);
            extern int wc_dec_due(void);
            if ((pi_intsr_raw() & pi_intmr_raw()) || wc_dec_due())
                g_wc_backedge_arm = 1;
        }
        {   /* MEMWATCH: strap structures under suspicion of a RAM stomp.
             * req=0x80426760: +232 TaskThread ptr, +80 result; queue count
             * at taskthread+80. Log every transition with the dispatch clock. */
            extern volatile unsigned g_wc_calls;
            static u32 mw_prev[4]; static int mw_init; static unsigned mw_n;
            static const u32 mw_addr[4] = { 0x80426848u, 0x804267B0u,
                                            0x80426760u, 0x804278C0u };
            u32 mw_now[4]; int mi;
            for (mi = 0; mi < 4; mi++) mw_now[mi] = mem_read32(mw_addr[mi]);
            if (!mw_init) { mw_init = 1;
                for (mi = 0; mi < 4; mi++) mw_prev[mi] = mw_now[mi]; }
            for (mi = 0; mi < 4; mi++)
                if (mw_now[mi] != mw_prev[mi]) {
                    if (mw_n < 40u) { mw_n++;
                        LOG_WARN(LOG_CORE, "MEMWATCH %08x %08x->%08x calls=%u",
                                 mw_addr[mi], mw_prev[mi], mw_now[mi],
                                 g_wc_calls); }
                    mw_prev[mi] = mw_now[mi];
                }
        }

                    u64 nb = timing_timebase();
                    if (nb - last_nudge > 800000u) {
                        mem_write32(0x80382864u, mem_read32(0x80382864u) + 1u);
                        last_nudge = nb;
                    }
                    still = 0;
                }
            } else {
                still = 0;
                last_calls = now_calls;
            }
        }
        /* LOW-MEMORY CANARY (zero-lag). The device-slice check lagged the
         * stomp by up to a slice; trips at calls=1163 and 1224 bracketed
         * different windows. The check now runs at every guest call boundary
         * (wc_irq_poll -> wc_canary_trip snapshots the crumb ring inside the
         * offending dispatch); this slice code only arms it and reports. */
        if (g_wc_running) {
            extern volatile int g_wc_canary_state;
            extern volatile unsigned g_wc_canary_trip_calls;
            extern volatile uint32_t g_wc_canary_val;
            extern uint32_t g_wc_canary_ring[64];
            extern volatile unsigned g_wc_calls;
            extern void wc_canary_service(void);
            wc_canary_service();
            if (g_wc_canary_state == 2) {
                unsigned c2 = g_wc_canary_trip_calls, k;
                char lb[160]; int used = 0;
                g_wc_canary_state = 3;
                LOG_WARN(LOG_CORE, "CANARY: 0x80000000 = %08x (was RMCE) at "
                         "calls=%u ZERO-LAG: newest crumb is the stomping call",
                         (unsigned)g_wc_canary_val, c2);
                for (k = 64; k >= 1; k--) {
                    used += snprintf(lb + used, sizeof lb - (size_t)used,
                                     " %08x",
                                     (unsigned)g_wc_canary_ring[(c2 - k) & 63u]);
                    if ((k % 8) == 1) {
                        LOG_WARN(LOG_CORE, "CANARY:%s", lb);
                        used = 0; lb[0] = 0;
                    }
                }
                /* The SHAPE of the stomp: a single pointer, a {next,fn,arg}
                 * list node at +0/+4/+8, or a run. Plus the two small-data
                 * globals the suspect window traffics in: the 800207D8 list
                 * head (r13-27500) and the version string ptr (r13-32616). */
                for (k = 0; k < 16; k++) {
                    used += snprintf(lb + used, sizeof lb - (size_t)used,
                                     " %08x", mem_read32(0x80000000u + k * 4u));
                    if ((k % 8) == 7) {
                        LOG_WARN(LOG_CORE, "CANARY mem:%s", lb);
                        used = 0; lb[0] = 0;
                    }
                }
                LOG_WARN(LOG_CORE, "CANARY globals: pushhead[80381D74]=%08x "
                         "verstr[80380BF8]=%08x now_calls=%u",
                         mem_read32(0x80381D74u), mem_read32(0x80380BF8u),
                         (unsigned)g_wc_calls);
            }
        }
        dev_unlock();
        prof_exit();
        /* The command processor has to consume what the title writes, or the
         * FIFO fills and the title waits for space that never frees.
         *
         * With the GX worker armed, the drain runs on the PPE's SECOND
         * HARDWARE THREAD, overlapping the whole GX pipeline (parse, vertex
         * decode, RSX command build) with the next JIT slice -- which is also
         * how the real machine behaves: the GP consumes the FIFO in parallel
         * with the CPU. Without the flag file this is the old synchronous
         * call. */
        g_wd_mark = WD_GXFIFO;
        if (s_gx_worker_on) {
            /* At most ONE kick outstanding.
             *
             * Posting per slice let the semaphore fill (it is created with a
             * maximum), after which sysSemPost quietly fails while `posted`
             * kept incrementing -- so the drain could never be satisfied and
             * the worker looked dead. The exchange makes the kick idempotent:
             * a parse already pending will pick up whatever else the guest has
             * written by the time it runs, which is the whole point of a FIFO.
             * Only count a kick we actually delivered. */
            /* Nothing to do: the worker is already draining the FIFO. */
            (void)s_gx_posted;
        } else {
            prof_enter(PH_GXFIFO);
            gx_state_run(gx_state());
            prof_exit();
        }
        g_wd_mark = WD_TIMING;
        prof_enter(PH_TIMING);
        /* Under the device lock: the BT model's ACL queue is also touched by
         * guest ioctl paths that hold dev_lock, and the unlocked pump raced
         * the reconnect-watchdog's queue flush against an in-flight consume --
         * check-then-decrement interleaved with count=0 and the counter
         * wrapped to ~4.29 billion, after which every Wiimote input TX
         * dropped forever and WPAD retried into a 378k-line error storm. */
        dev_lock();
        dev_lock_tag("devloop-bt");
        ios_bt_update();    /* wiimote link + input stream heartbeat */
        dev_unlock();
        difftrace_sample(&cpu);
        /* Hand the console's audio port whatever the DSP has produced. This
         * has to happen on the run loop rather than on a notify event, because
         * blocking for an event here would stall the emulated CPU; the port's
         * eight blocks are the elasticity that makes polling enough. */
        audio_ps3_update();
        prof_exit();        /* ---- end PHASE PROFILE ---- */

        /* Heartbeat. The boot runs for hundreds of millions of instructions
         * before the title draws anything, and on a console there is no other
         * way to tell "working" from "wedged" -- so say so on screen, with the
         * counters that show it is moving. Once the title presents frames of
         * its own this stops and gets out of the way. */
        /* Once the title presents frames, the status panel must never paint
         * again: it flips a CPU-filled buffer over the game and reads as "the
         * old piracy debug screen flickering in", which is exactly what it
         * is. It exists only to prove life before the first frame. */
        /* Port mode runs ~80k slices/s: every-4000 meant TWENTY full-screen
         * CPU fills per second -- 34.6% of the whole machine (PROF, cycle 1)
         * spent repainting a static status panel. Once per second proves
         * life just as well. */
        if (s_mkw_frames == 0 && g_rsx.inited &&
            (slices % (g_wc_running ? 80000u : 4000u)) == 0) {
            char h1[96], h2[96];
            prof_enter(PH_BOOTUI);      /* PHASE PROFILE */
            int b0 = 0, p0 = (int)g_rsx.pitch;
            u32 *hb = g_rsx.buffer[b0];

            rsx_fill_cpu(b0, 0xFF101830u);
            rsx_draw_text_scaled(hb, p0, 60,  50, 0xFFFFFFFFu, 6,
                                 "MARIO KART WII");
            rsx_draw_text_scaled(hb, p0, 60, 140, 0xFFC0D0FFu, 3,
                                 s_mkw_frames ? "RUNNING - TITLE IS DRAWING"
                                              : "BOOTING ON THE PPE");
            snprintf(h1, sizeof h1, "%llu M INSTRUCTIONS   PC %08X",
                     (unsigned long long)(total_insts / 1000000ull),
                     (unsigned)cpu.pc);
            rsx_draw_text_scaled(hb, p0, 60, 200, 0xFFFFFFFFu, 3, h1);
            snprintf(h2, sizeof h2, "DISC %u  FRAMES %llu  DRAWS %llu  VERTS %llu",
                     ios_progress_disc_reads(),
                     (unsigned long long)s_mkw_frames,
                     (unsigned long long)g_gx_render.draws,
                     (unsigned long long)g_gx_render.vertices);
            rsx_draw_text_scaled(hb, p0, 60, 250, 0xFF90FF90u, 3, h2);
            rsx_draw_text_scaled(hb, p0, 60, 320, 0xFFFFE080u, 2,
                                 "COUNTERS MOVING = RUNNING.  FROZEN = HUNG.");
            rsx_present_cpu(b0);
            prof_exit();                /* PHASE PROFILE */
        }
        g_wd_seq++;
        {   /* Per-slice accounting, because the window-level subtraction was
             * not sound.
             *
             * `real = credited_total - skipped_total` differenced two counters
             * maintained independently, and in-race the skipped total came out
             * LARGER than the credited one (5.59G against 3.93G), which cannot
             * be true slice by slice: an idle skip zeroes the downcount, so
             * that slice is credited its whole grant and the skip is a part of
             * it. Whatever drifts between them -- and both are also read by
             * other windows -- the fix is to do the subtraction where the
             * invariant actually holds, once per slice, and accumulate the
             * result. */
            u32 used    = (u32)(grant - (cpu.downcount < 0 ? 0 : cpu.downcount));
            u32 skipped = cpu.idle_skipped_last;
            if (skipped > used) skipped = used;   /* cannot skip more than ran */
            total_insts   += used;
            s_real_insts  += used - skipped;
        }
        /* Separate the spins the JIT skipped from work the guest really did.
         * An idle skip charges the rest of the slice, so `total_insts` counts
         * instructions that were never executed; without this split, "guest
         * instructions per second" measures how much MKWii waits as much as
         * how fast we run it. */
        s_idle_insts += cpu.idle_skipped_last;
        cpu.idle_skipped_last = 0;
        s_mkw_insts = total_insts;
        if (slices == 0) { after_first = cpu.pc; s_mkw_t0 = read_timebase(); }


        if (g_wc_running) {
            /* The port does not execute through cpu.pc at all: the translated
             * game runs on its own thread and this loop only drives the
             * devices for it. cpu.pc therefore sits at the entry point
             * forever, which is precisely the signature this detector treats
             * as a hung guest -- so it ended the session 16 ms after the game
             * started, four more sessions ran and leaked a 48 MiB vertex arena
             * each, and by the fifth there was no heap left. The game was
             * healthy the whole time; the only evidence it ever ran was two
             * SDK banners in the log.
             *
             * Liveness for the port is guest progress, which wc_watchdog
             * already watches and reports on. */
            stuck = 0;
        } else if (cpu.pc >= win_lo && cpu.pc <= win_hi && (win_hi - win_lo) < 0x80) {
            /* Debt payoff legitimately freezes the pc for many slices after
             * a whole-function AOT decompression; only count genuine spins. */
            if (jit_aot_debt_pending() > 0) stuck = 0;
            else if (++stuck > 400) { why = "spin"; break; }
        } else { stuck = 0; win_lo = win_hi = cpu.pc; }
        if (cpu.pc < win_lo) win_lo = cpu.pc;
        if (cpu.pc > win_hi) win_hi = cpu.pc;
        if (cpu.pc == VEC_PROGRAM) {
            /* Name the offending instruction instead of just the vector: a
             * Program exception here means the guest executed something we do
             * not implement, and the opcode fields say exactly what. */
            u32 fa = cpu.spr[SPR_SRR0], w0 = mem_read32(fa);
            emitf("   PROGRAM: srr0=%08x insn=%08x primary=%u xo10=%u xo5=%u",
                  (unsigned)fa, (unsigned)w0, (unsigned)((w0 >> 26) & 0x3F),
                  (unsigned)((w0 >> 1) & 0x3FF), (unsigned)((w0 >> 1) & 0x1F));
            emitf("   PROGRAM: -8:%08x -4:%08x [%08x] +4:%08x +8:%08x",
                  (unsigned)mem_read32(fa - 8), (unsigned)mem_read32(fa - 4),
                  (unsigned)w0, (unsigned)mem_read32(fa + 4),
                  (unsigned)mem_read32(fa + 8));
            emitf("   PROGRAM: lr=%08x r3=%08x r4=%08x msr=%08x",
                  (unsigned)cpu.lr, (unsigned)cpu.gpr[3],
                  (unsigned)cpu.gpr[4], (unsigned)cpu.msr);
            why = "PROGRAM"; break;
        }
        if (cpu.pc == VEC_DSI)     { why = "DSI"; break; }
        if (cpu.pc == VEC_ISI)     { why = "ISI"; break; }
    }

    /* PHASE PROFILE: fold the last partial interval into the session totals
     * and print the whole-run breakdown beside the speed numbers. */
    prof_window_close();
    prof_dump(emit_line, NULL, PS3_TIMEBASE_HZ, "LIFETIME (boot loop)", 1,
              (u64)s_mkw_frames, total_insts,
              g_gx_render.draws, g_gx_render.vertices);
    g_prof.enabled = 0;
    /* ---- end PHASE PROFILE ---- */

    {
        unsigned opens = ios_progress_opens();
        unsigned ap    = ios_progress_antipiracy();
        int passed = (ap >= 2);
        char l1[96], l2[96], l3[96];

        emitf("   stop=%s pc=%08x srr0=%08x dar=%08x", why,
              (unsigned)cpu.pc, (unsigned)cpu.spr[SPR_SRR0],
              (unsigned)cpu.spr[SPR_DAR]);
        emitf("   first_pc=%08x after_first_slice=%08x insts=%llu",
              (unsigned)first_pc, (unsigned)after_first,
              (unsigned long long)total_insts);
        emitf("   IOS opens %u, anti-piracy answered %u, disc reads %u",
              opens, ap, ios_progress_disc_reads());
        emitf("   FRAMES PRESENTED: %llu", (unsigned long long)s_mkw_frames);
        {   /* The audio frame clock, and whether the console kept up with it.
             * Underruns mean the run loop is not draining the port often
             * enough, which is a pacing problem rather than a mixer one --
             * worth being able to tell apart from the log alone. */
            unsigned ab = 0, au = 0, aq = 0;
            audio_ps3_stats(&ab, &au, &aq);
            emitf("   AUDIO: ucode %08x AID=%llu lists=%llu frames=%llu "
                  "voices=%llu(%llu active) audible=%llu peak=%u",
                  (unsigned)ax_ucode_crc(),
                  (unsigned long long)dsp_stat_aid_interrupts(),
                  (unsigned long long)dsp_stat_command_lists(),
                  (unsigned long long)ax_stat_frames(),
                  (unsigned long long)ax_stat_voices(),
                  (unsigned long long)ax_stat_active_voices(),
                  (unsigned long long)ax_stat_audible_frames(),
                  (unsigned)ax_stat_peak());
            emitf("   AUDIO OUT: %s blocks=%u queued=%u underruns=%u "
                  "pushed=%llu dropped=%llu",
                  g_audio_enable ? "on" : "off", ab, aq, au,
                  (unsigned long long)audio_out_pushed(),
                  (unsigned long long)audio_out_dropped());
        }
        audio_ps3_shutdown();
        /* Superblocks make blocks big; if the cache thrashes, this says so. */
        emitf("   JIT: %llu blocks, %llu cache flushes, expansion %.2fx",
              (unsigned long long)g_jit_stats.blocks_compiled,
              (unsigned long long)g_jit_stats.cache_flushes,
              jit_expansion_ratio());
        emitf("   JIT compile time: %.3f s total, worst single block %u us",
              (double)g_jit_stats.compile_ticks / PS3_TIMEBASE_HZ,
              (unsigned)((double)g_jit_stats.compile_ticks_max
                         * 1e6 / PS3_TIMEBASE_HZ));
        /* The speed verdict, in the log rather than only on the TV: this is the
         * measurement the whole exercise turns on, and reading it off a
         * photograph of a television is no way to settle it. Broadway retires
         * 729 M/s; this title needs 510 M/s of *emulated* work to hold 60 fps,
         * so that ratio is what "native" means. */
        if (s_mkw_t0) {
            double sec = (double)(read_timebase() - s_mkw_t0) / PS3_TIMEBASE_HZ;
            double ips = sec > 0.001 ? (double)total_insts / sec : 0.0;
            emitf("   SPEED: %llu insts in %d.%03d s = %d M inst/s",
                  (unsigned long long)total_insts,
                  (int)sec, (int)(sec * 1000) % 1000, (int)(ips / 1e6));
            emitf("   SPEED: %d.%02dx native (1.00x = %d M inst/s), %d.%01d fps",
                  (int)(ips / NATIVE_INST_PER_SEC),
                  (int)(ips / NATIVE_INST_PER_SEC * 100) % 100,
                  (int)(NATIVE_INST_PER_SEC / 1e6),
                  (int)(s_mkw_frames / (sec > 0.001 ? sec : 1.0)),
                  (int)(s_mkw_frames * 10 / (sec > 0.001 ? sec : 1.0)) % 10);
        }
        emitf("   GX: %llu draws, %llu vertices, %llu programs",
              (unsigned long long)g_gx_render.draws,
              (unsigned long long)g_gx_render.vertices,
              (unsigned long long)g_gx_render.programs_built);
        /* Draws that never reached the GPU. Silent otherwise, and the first
         * thing to check if the counts look right but the screen does not. */
        emitf("   GX dropped: %llu overflow, %llu no-position%s%s",
              (unsigned long long)g_gx_render.overflow,
              (unsigned long long)g_gx_render.skipped_no_pos,
              panic[0] ? ", halt: " : "", panic);

        /* Draw the status into BOTH framebuffers (so whichever is being scanned
         * out shows it) via the proven CPU path, and report the present result
         * so a blank TV can be told apart from a failed flip. */
        if (g_rsx.inited) {
            int bb, pr0, pr1, p = (int)g_rsx.pitch;
            for (bb = 0; bb < RSX_BUFFERS; bb++) {
                u32 *fb = g_rsx.buffer[bb];
                rsx_fill_cpu(bb, 0xFF104818u);      /* deep green background */
                rsx_draw_text_scaled(fb, p, 60,  50, 0xFFFFFFFFu, 6, "MARIO KART WII");
                rsx_draw_text_scaled(fb, p, 60, 140, 0xFFC0D0FFu, 4, "RMCE01 ON PS3 PPE");
                snprintf(l1, sizeof l1, "%llu M INSTS   STOP %s",
                         (unsigned long long)(total_insts / 1000000ull), why);
                rsx_draw_text_scaled(fb, p, 60, 195, 0xFFFFFFFFu, 3, l1);
                /* The numbers that say whether the title actually ran: files
                 * read off the disc, frames it finished, geometry it drew. On
                 * screen because reading them off the TV is more reliable than
                 * fetching a report from a console that may be switched off. */
                snprintf(l3, sizeof l3, "DISC READS %u   IOS OPENS %u",
                         ios_progress_disc_reads(), opens);
                rsx_draw_text_scaled(fb, p, 60, 240, 0xFFFFFFFFu, 3, l3);
                snprintf(l2, sizeof l2, "FRAMES %llu  DRAWS %llu  VERTS %llu",
                         (unsigned long long)s_mkw_frames,
                         (unsigned long long)g_gx_render.draws,
                         (unsigned long long)g_gx_render.vertices);
                rsx_draw_text_scaled(fb, p, 60, 285, 0xFF90FF90u, 3, l2);
                rsx_draw_text_scaled(fb, p, 60, 345,
                                     passed ? 0xFF40FF40u : 0xFFFF6060u, 5,
                                     passed ? "ANTI-PIRACY PASSED"
                                            : "ANTI-PIRACY NOT PASSED");
            }
            pr0 = rsx_present_cpu(0);
            pr1 = rsx_present_cpu(1);
            emitf("   display: w=%u h=%u pitch=%u present0=%d present1=%d",
                  (unsigned)g_rsx.width, (unsigned)g_rsx.height,
                  (unsigned)g_rsx.pitch, pr0, pr1);
        } else {
            emit_line(NULL, "   display: g_rsx not inited");
        }
    }
    }
}

/* Append one line to a tiny file the instant this process starts, before any
 * subsystem is touched. A self-relaunch that leaves no line here was refused
 * by the loader; a line with no session after it means the image started and
 * died during init. Without this the two are indistinguishable -- both just
 * leave the console on the XMB. */
/* Append one line to the breadcrumb file. Opened and closed per call on
 * purpose: the point is that the line survives whatever happens next. */
static void boot_note(const char *what)
{
    static const char *k_path = "/dev_hdd0/tmp/dolphin-boot.txt";
    char line[160];
    int fd, n;
    fd = open(k_path, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0) return;
    n = snprintf(line, sizeof line, "boot:   %s\n", what);
    if (n > 0) write(fd, line, (size_t)n);
    close(fd);
}

static void boot_breadcrumb(void)
{
    static const char *k_path = "/dev_hdd0/tmp/dolphin-boot.txt";
    struct stat st;
    char line[128];
    int fd, n;
    long long sz = -1;
    if (stat("/dev_hdd0/game/DOLPHIN01/USRDIR/EBOOT.BIN", &st) == 0)
        sz = (long long)st.st_size;
    fd = open(k_path, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0) return;
    n = snprintf(line, sizeof line, "boot: eboot=%lld bytes\n", sz);
    if (n > 0) write(fd, line, (size_t)n);
    close(fd);
}

int main(void)
{
    boot_breadcrumb();
    /* Before anything that can fail: if the rest of init dies, or the emulator
     * later wedges, this is the only way back in without touching the console. */
    {   int rc;
        sysModuleLoad(SYSMODULE_NET);
        rc = netInitialize();
        sysThreadGetId(&g_emu_tid);   /* for the rescue ctx reader */
        /* sysDbgInitializePPUExceptionHandler was tried here and is FATAL at
         * boot on this (non-debug) firmware: the process died between the
         * eboot-size boot note and rescue_init, no log line, and it cost a
         * launch at the console. The ctx reader stays opportunistic: its read
         * runs on a throwaway thread precisely because it may never return. */
        if (rc >= 0) rescue_init();
        else boot_note("rescue: netInitialize failed");
    }
    {   /* Boot-time state-mask override, before ANY subsystem starts. Tiling
         * and Zcull are chosen inside rsx_video_init and cannot be changed
         * afterwards, so a runtime "mask" command can never turn tiling off --
         * which makes it untestable without this. Placed at the top of main
         * because the first attempt went into the video SELF-TEST stage, which
         * this build does not reach, so the override silently never ran. */
        extern int g_tex_force_clamp;
        FILE *cf = fopen("/dev_hdd0/tmp/dolphin-clamp.txt", "r");
        FILE *mf;
        if (cf) { g_tex_force_clamp = 1; fclose(cf); }
        {
            extern int g_tex_force_trilinear;
            FILE *tf2 = fopen("/dev_hdd0/tmp/dolphin-trilinear.txt", "r");
            if (tf2) { g_tex_force_trilinear = 1; fclose(tf2); }
        }
        mf = fopen("/dev_hdd0/tmp/dolphin-mask.txt", "r");
        if (mf) {
            unsigned m = 0;
            if (fscanf(mf, "%x", &m) == 1 && m) g_gx_state_mask = m;
            fclose(mf);
        }
    }
    {   /* A/B handle for the PPE data-prefetch engine over FIFO windows.
         * Unconditional code cannot be measured, and this needs measuring. */
        extern int g_ppe_prefetch_off;
        FILE *nf = fopen("/dev_hdd0/tmp/dolphin-nodpfe.txt", "rb");
        if (nf) { fclose(nf); g_ppe_prefetch_off = 1; }
    }
    bench_load();
    /* Decide port mode HERE, before anything large is allocated.
     *
     * It used to be decided at the point the port starts, which is after the
     * RSX, the vertex arena and the JIT cache have all taken their memory --
     * so the port-mode reductions never applied. The result was a 48 MiB
     * vertex arena on top of a 68 MiB image, an exhausted heap
     * ("largest block >= 0 MiB"), and every later allocation failing: the NAND
     * files, the WC24 files, and finally sysThreadCreate for the game thread
     * itself. The port never ran and the log blamed a missing disc slice.
     *
     * The flag is only READ here; it is deleted (one-shot) and acted on later. */
    {   FILE *pf = fopen("/dev_hdd0/tmp/dolphin-wcboot.txt", "r");
        if (pf) { fclose(pf); g_wc_running = 1;
                  boot_note("port mode: sizing allocations for the native game");
                  { sys_ppu_thread_t ct;
                    if (sysThreadCreate(&ct, canary_watcher, NULL, 900, 0x4000,
                                        0, "canary") == 0)
                        boot_note("canary watcher up (50 us poll)"); } }
    }
    boot_note("flags read");
    /* Registered before anything else can block: the XMB must be able to
     * reach us from the first instant the title is on screen. */
    sysUtilRegisterCallback(SYSUTIL_EVENT_SLOT0, ps3_sysutil_cb, NULL);
    boot_note("sysutil callback registered");
    DiffResults res;
    u64 ticks_i = 0, ticks_j = 0;
    double mips_i = 0.0, mips_j = 0.0;
    int rc = 0;
    unsigned i;

    /* devlink FIRST, and a missing report is no longer fatal.
     *
     * Both of these cost a session once. `report_open` failing used to
     * `return 0`, so the process vanished with no report AND no devlink -- and
     * with no devlink there is no `relaunch`, which is the only remote way to
     * start the next build. One transient failure to create a file therefore
     * ended unattended development until somebody pressed a button on the
     * console. Starting devlink before anything that can fail keeps a remote
     * handle open no matter what follows, and `emit_line` already falls back to
     * fd 2 when there is no report file. */
    devlink_init();
    boot_note("devlink up");
    {
        int which = -1;
        report_open(&which);
        i = (which < 0) ? 0u : (unsigned)which;
        boot_note(which < 0 ? "report FAILED (continuing)" : "report open");
    }

    emit_line(NULL, "==== Dolphin-PS3 recompiler self-test ====");
    /* Stamp the build into the report. Without it there is no way to tell a
     * deploy that landed from one that silently booted the previous image,
     * and that failure mode is invisible: every other line looks normal and
     * the boot log reports the size of the file on disk rather than of the
     * image actually running. Compare this against the local build before
     * trusting any measurement. */
    emitf("build: %s %s", __DATE__, __TIME__);
    emitf("report: %s", k_report_paths[i]);

    stage("log_init");
    log_init();
    log_set_level(LOG_MEM,    LOG_LEVEL_ERROR);
    log_set_level(LOG_INTERP, LOG_LEVEL_WARN);
    log_set_level(LOG_VIDEO,  LOG_LEVEL_ERROR);
    /* CORE and JIT stay at INFO: the report is the console's only voice, and
     * the save-flow FS narration lives on LOG_CORE. Three launches went by
     * with the narration compiled in but runtime-filtered right here. */
    log_set_level(LOG_CORE,   LOG_LEVEL_INFO);
    log_set_level(LOG_JIT,    LOG_LEVEL_INFO);

    /* Before any device can be touched, and before the game thread exists. */
    dev_lock_init();
    stage("mem_init (reserving 1 GiB of address space)");
    if (mem_init(1) != 0) {
        emit_line(NULL, "FATAL: guest memory init failed");
        goto done;
    }
    emitf("   arena at %p, fastmem %s",
          (void *)mem_base(), g_mem.fastmem_ok ? "ON" : "OFF");

    /* The write-then-execute probes that used to sit here have done their
     * job and are gone. They established, on the console, that the PS3
     * refuses to execute code written at run time into heap, mapper, or .bss
     * memory (retail firmware enforces W^X in userland), while the loadable
     * text segment executes runtime-written code and is writable in practice.
     * The JIT code cache is therefore a static array in .text (see
     * code_buffer_alloc in jit.c), and the stages below now reach real
     * recompiled execution instead of dying on a no-execute page. */

    stage("interp_init_tables");
    interp_init_tables();

    /* Size the code cache to the game's working set, not to a round number.
     *
     * Measured in-race: an 8 MiB cache held ~12,800 blocks and overflowed
     * roughly once a second, and a flush is all-or-nothing -- every one of
     * those blocks had to be recompiled from cold. That showed up as 52 ms
     * per frame inside jit_run, 23% of the entire frame, spent in the
     * compiler rather than running guest code, and it never converged
     * because the working set was permanently a little larger than the
     * cache. Above the working set the flush rate falls to zero and that
     * cost disappears entirely; below it, no amount of compiler tuning
     * helps, because the work is being thrown away.
     *
     * Descending sizes rather than one fixed request: the cache is a single
     * contiguous allocation and the emulator has already taken its arena and
     * vertex buffers by this point, so a large request can legitimately fail
     * on a console with less free memory than this one. Falling back keeps
     * such a machine booting -- slower, but running. */
    {
        /* In port mode the JIT never compiles anything, and its cache is pure
         * loss: 24 MiB against a budget that the translated game has already
         * spent 45 MiB of. The first threaded port boot went over that budget
         * -- 74 MiB image + 92 MiB guest RAM + 48 MiB vertex arena + 24 MiB
         * here is 238 MiB on a console that gives a game about 213 -- and the
         * result was not a clean allocation failure but a wedged machine that
         * stopped answering FTP, devlink and the rescue port. So the flag is
         * read here, before anything large is taken, rather than at the point
         * the port starts. */
        static const unsigned k_cache_mib_jit[]  = { 24, 16, 8 };
        static const unsigned k_cache_mib_port[] = { 1 };
        const unsigned *k_cache_mib = k_cache_mib_jit;
        size_t k_cache_n = sizeof k_cache_mib_jit / sizeof k_cache_mib_jit[0];
        if (g_wc_running) { k_cache_mib = k_cache_mib_port; k_cache_n = 1;
            emit_line(NULL, "port mode: JIT cache reduced to 1 MiB"); }
        size_t ci;
        int ok = 0;
        if (g_wc_running) {
            /* A/B bisect handle: with the flag present the port boots exactly
             * as the proven pre-hybrid build (no jit_init, bridge disarmed,
             * misses skip) -- one relaunch separates a corpus regression from
             * a runtime one. */
            FILE *nh = fopen("/dev_hdd0/tmp/dolphin-nohybrid.txt", "rb");
            if (nh) { fclose(nh);
                stage("jit_init skipped (nohybrid flag)");
                ok = 1;
                goto jit_done;
            }
            /* Hybrid: StaticR.rel executes under the JIT (the full-native
             * link is 154 MB against an ~85 MB image budget), so port mode
             * now NEEDS the JIT. Small cache: the REL working set compiles
             * incrementally and the hot functions get promoted to natives. */
            static const unsigned k_cache_hybrid[] = { 8, 6, 4 };
            for (ci = 0; ci < 3; ci++) {
                char sb[64];
                snprintf(sb, sizeof sb, "jit_init hybrid (%u MiB cache)",
                         k_cache_hybrid[ci]);
                stage(sb);
                if (jit_init((size_t)k_cache_hybrid[ci] << 20) == 0) { ok = 1; break; }
            }
            if (ok) {
                extern void wc_bridge_init(void);
                wc_bridge_init();
            } else
                stage("jit_init FAILED -- REL calls will be skipped");
            ok = 1;
jit_done: ;
        } else for (ci = 0; ci < k_cache_n; ci++) {
            char sb[64];
            snprintf(sb, sizeof sb, "jit_init (%u MiB code cache)",
                     k_cache_mib[ci]);
            stage(sb);
            if (jit_init((size_t)k_cache_mib[ci] << 20) == 0) { ok = 1; break; }
        }
        if (!ok) {
            emit_line(NULL, "FATAL: JIT init failed");
            goto done;
        }
    }

    stage("timing_init");
    timing_init(1);

    if (g_wc_running) {
        /* The self-test stages between here and the game execute guest code
         * through the JIT, which port mode no longer initialises (its AOT
         * stub patching writes .text -- fatal under RPCS3's RX mapping, pure
         * waste on console). Straight to the game. */
        emit_line(NULL, "   port mode: skipping self-test stages");
        goto port_video;   /* video init must still run -- the first jump went
                            * straight to the game and gx_render came up dead
                            * (console: seq=0, device loop never started) */
    }

    /* Compile one block before trusting the whole suite: a failure here is a
     * compiler problem, whereas a failure in the next stage is an execution
     * problem, and the two want completely different investigations. */
    if (!g_wc_running) {
    stage("compile a single block");
    {
        static PPCState probe;
        JitBlock *b;
        memset(&probe, 0, sizeof probe);
        probe.msr = MSR_FP;
        probe.const_one = 1.0;
        probe.pc = BENCH_CODE;
        build_benchmark();
        b = jit_compile_block(&probe, BENCH_CODE);
        if (!b) {
            emit_line(NULL, "FATAL: could not compile a block");
            goto done;
        }
        emitf("   block: %u guest -> %u host instructions",
              b->guest_insts, b->code_words);
    }
    }

    if (!execution_allowed()) {
        /* Everything below this point runs recompiled code. Report the shape of
         * what was compiled -- which is the part still worth checking here --
         * and finish cleanly, so this run is a usable pass/fail signal rather
         * than a crash that means nothing. */
        emit_line(NULL, "");
        emit_line(NULL, "host cannot execute generated code: "
                        "execution stages skipped by request");
        emitf("   compiled %llu blocks, %llu guest -> %llu host (%.2fx)",
              (unsigned long long)g_jit_stats.blocks_compiled,
              (unsigned long long)g_jit_stats.guest_insts_compiled,
              (unsigned long long)g_jit_stats.host_insts_emitted,
              jit_expansion_ratio());

        stage("interpreter benchmark");
        mips_i = run_bench(0, BENCH_ITERS, &ticks_i);
        emitf("interpreter : %8.1f M guest inst/s  (%llu ticks)",
              mips_i / 1e6, (unsigned long long)ticks_i);
        goto done;
    }

    /* The CPU suites (EXECUTE, differential, realcode, realbench) leave
     * sentinel addresses, test code and thousands of intentional program
     * exceptions behind -- noise that repeatedly masqueraded as game crashes
     * (the "race-load PE storm" was partly realtest's own end-of-case
     * exceptions at GUEST_SENTINEL) and cost ~30s per boot. Bring-up
     * diagnostics; run only when the flag file asks. */
    {
        FILE *sf2 = fopen("/dev_hdd0/tmp/dolphin-selftest.txt", "rb");
        if (!sf2) {
            emit_line(NULL, "   CPU suites skipped (dolphin-selftest.txt absent)");
            goto done;
        }
        fclose(sf2);
    }

    /* Executing recompiled code for the first time is the single most likely
     * place to die, so it gets its own breadcrumb and a deliberately tiny
     * workload. */
    stage("EXECUTE recompiled code (first time, 4 iterations)");
    {
        static PPCState probe;
        setup_bench_state(&probe, 4);
        jit_run(&probe);
        emitf("   returned: pc=%08x r3=%08x downcount=%d",
              (unsigned)probe.pc, (unsigned)probe.gpr[3], (int)probe.downcount);
    }

    stage("differential suite");
    rc = difftest_run_all(emit_line, NULL, &res);
    emitf("   jit_executed=%d cases=%u failed=%u mismatches=%u",
          res.jit_executed, res.cases_run, res.cases_failed,
          res.state_mismatches);

    /* Reset the fallback histogram so what it captures next is the real-code
     * profile specifically -- the representative workload -- and not the
     * deliberately-fallback differential cases compiled earlier. */
    memset(g_jit_stats.fallback_by_opcd, 0, sizeof g_jit_stats.fallback_by_opcd);
    memset(g_jit_stats.fallback_x31_xo, 0, sizeof g_jit_stats.fallback_x31_xo);
    memset(g_jit_stats.fallback_x63_xo, 0, sizeof g_jit_stats.fallback_x63_xo);

    stage("real compiler output suite");
    {
        int checks = 0, failures = 0;
        if (realtest_run_all(emit_line, NULL, &checks, &failures) != 0)
            rc = -1;
        emitf("   realcode: %d checks, %d failures", checks, failures);
    }

    if (rc != 0) {
        emit_line(NULL, "");
        emit_line(NULL, "correctness FAILED - skipping benchmark");
        goto done;
    }

    stage("benchmark");
    build_benchmark();
    jit_flush_all();
    mips_j = run_bench(1, BENCH_ITERS, &ticks_j);
    mips_i = run_bench(0, BENCH_ITERS, &ticks_i);

    emit_line(NULL, "");
    emit_line(NULL, "==== results ====");
    emitf("interpreter : %8.1f M guest inst/s  (%llu ticks)",
          mips_i / 1e6, (unsigned long long)ticks_i);
    emitf("recompiler  : %8.1f M guest inst/s  (%llu ticks)",
          mips_j / 1e6, (unsigned long long)ticks_j);
    if (mips_i > 0.0)
        emitf("speedup     : %8.2fx over the interpreter", mips_j / mips_i);
    emitf("vs Broadway : %8.2fx real-time (against 800 M inst/s)",
          mips_j / 800e6);
    emitf("jit: %llu blocks, %llu guest -> %llu host (%.2fx), %llu fallbacks",
          (unsigned long long)g_jit_stats.blocks_compiled,
          (unsigned long long)g_jit_stats.guest_insts_compiled,
          (unsigned long long)g_jit_stats.host_insts_emitted,
          jit_expansion_ratio(),
          (unsigned long long)g_jit_stats.fallback_insts);
    emitf("links: %llu emitted, %llu resolved",
          (unsigned long long)g_jit_stats.links_emitted,
          (unsigned long long)g_jit_stats.links_resolved);

    /* The counted-loop benchmark: same work, a bdnz back-edge, so this is the
     * one that exercises loop register retention. */
    {
        u64 tci, tcj;
        double ci, cj;
        build_benchmark_ctr();
        jit_flush_all();
        cj = run_bench_ctr(1, BENCH_ITERS, &tcj);
        ci = run_bench_ctr(0, BENCH_ITERS, &tci);
        emit_line(NULL, "");
        emitf("counted loop (bdnz, retained):");
        emitf("  interpreter : %8.1f M guest inst/s", ci / 1e6);
        emitf("  recompiler  : %8.1f M guest inst/s  (%.2fx real-time)",
              cj / 1e6, cj / 800e6);
        if (ci > 0.0)
            emitf("  speedup     : %8.2fx over the interpreter", cj / ci);
    }

    /* The real-code fallback profile: the top primary opcodes that still fell
     * back, with a finer breakdown for opcode 31 (integer/system) and 63
     * (double float). This names the next instructions to implement. */
    {
        unsigned o, top;
        emit_line(NULL, "realcode fallbacks by opcode (primary):");
        for (top = 0; top < 6; top++) {
            unsigned best = 0; u64 bestn = 0;
            for (o = 0; o < 64; o++)
                if (g_jit_stats.fallback_by_opcd[o] > bestn) {
                    bestn = g_jit_stats.fallback_by_opcd[o]; best = o;
                }
            if (bestn == 0) break;
            emitf("   opcd %2u : %llu", best, (unsigned long long)bestn);
            g_jit_stats.fallback_by_opcd[best] = 0;   /* pop for next pass */
        }
        emit_line(NULL, "  ...opcode-31 extended (XO):");
        for (top = 0; top < 6; top++) {
            unsigned best = 0; u64 bestn = 0;
            for (o = 0; o < 1024; o++)
                if (g_jit_stats.fallback_x31_xo[o] > bestn) {
                    bestn = g_jit_stats.fallback_x31_xo[o]; best = o;
                }
            if (bestn == 0) break;
            emitf("   x31 XO %4u : %llu", best, (unsigned long long)bestn);
            g_jit_stats.fallback_x31_xo[best] = 0;
        }
        emit_line(NULL, "  ...opcode-63 extended (XO):");
        for (top = 0; top < 6; top++) {
            unsigned best = 0; u64 bestn = 0;
            for (o = 0; o < 1024; o++)
                if (g_jit_stats.fallback_x63_xo[o] > bestn) {
                    bestn = g_jit_stats.fallback_x63_xo[o]; best = o;
                }
            if (bestn == 0) break;
            emitf("   x63 XO %4u : %llu", best, (unsigned long long)bestn);
            g_jit_stats.fallback_x63_xo[best] = 0;
        }
    }

    /* The numbers above measure an instruction mix I chose. This one measures
     * one GCC chose -- a sort, a checksum, 64-bit arithmetic and a float
     * transform, compiled at -O2. It is the better predictor of how a title
     * will run, because it contains the carry chains and division sequences the
     * recompiler still falls back on, in the proportions a compiler actually
     * emits them. */
    stage("benchmark: real compiler output");
    {
        u64 insts_i = 0, insts_j = 0, t0, t1;
        double secs_i = 0.0, secs_j = 0.0;

        jit_flush_all();
        t0 = read_timebase();
        if (realtest_benchmark(1, REALBENCH_REPS, &insts_j) != 0)
            emit_line(NULL, "   (recompiler workload did not complete)");
        t1 = read_timebase();
        secs_j = (double)(t1 - t0) / PS3_TIMEBASE_HZ;

        t0 = read_timebase();
        if (realtest_benchmark(0, REALBENCH_REPS, &insts_i) != 0)
            emit_line(NULL, "   (interpreter workload did not complete)");
        t1 = read_timebase();
        secs_i = (double)(t1 - t0) / PS3_TIMEBASE_HZ;

        if (secs_i > 0.0 && secs_j > 0.0) {
            double mips_ri = (double)insts_i / secs_i;
            double mips_rj = (double)insts_j / secs_j;
            emitf("realcode interp : %8.1f M guest inst/s (%llu insts)",
                  mips_ri / 1e6, (unsigned long long)insts_i);
            emitf("realcode recomp : %8.1f M guest inst/s (%llu insts)",
                  mips_rj / 1e6, (unsigned long long)insts_j);
            if (mips_ri > 0.0)
                emitf("realcode speedup: %8.2fx over the interpreter",
                      mips_rj / mips_ri);
            emitf("realcode vs Broadway: %8.2fx real-time (against 800 M inst/s)",
                  mips_rj / 800e6);
        }
    }

done:
    /* Display output. Everything above this point is measured through a text
     * file; this is the first thing that reaches the television, and it is the
     * emulator's real video path rather than a demo -- the same rsx_video layer
     * the GX front end will render through.
     *
     * It runs for a fixed number of frames and then returns to the XMB on its
     * own. That is deliberate: quitting otherwise needs a controller, and a
     * homebrew app that requires one to exit is a bad way to find out your
     * controller is flat.
     *
     * The pattern is chosen to be unmistakable on a TV -- three primary colours
     * in sequence, then a grey ramp -- so "the screen changed colour" cannot be
     * confused with a stuck frame or a black screen. */
    port_video:
    stage("video output");
    if (rsx_video_init() != 0) {
        emit_line(NULL, "   video init FAILED (see log)");
    } else if (execution_allowed()) {
        /* FAST BOOT: straight into the game. The colour bars, geometry
         * pipeline stages and guest triangle exist for bring-up and cost the
         * user half a minute of test cards on every launch; the game itself
         * is now the better diagnostic. They still run on hosts where
         * generated code cannot execute, where they are all there is. */
        {
            /* The benchmark hijacked the boot path for a few sessions; the
             * game is the product. Bench only when the flag file exists. */
            FILE *bf = fopen("/dev_hdd0/tmp/dolphin-bench.flag", "rb");
            if (bf) {
                fclose(bf);
                emit_line(NULL, "   bench flag: recomp benchmark, then exit");
                recomp_benchmark_stage();
                rsx_video_shutdown();
                goto finished;
            }
        }
    port_boot:
        emit_line(NULL, "   fast boot: straight into the game");
    session_restart:
        mkwii_boot_stage();
        rsx_video_shutdown();
        goto finished;   /* clean exit to XMB -- no hold screen */
    } else {
        unsigned f;
        int alive, shown;

        emitf("   %ux%u, pitch %u",
              (unsigned)g_rsx.width, (unsigned)g_rsx.height,
              (unsigned)g_rsx.pitch);

        /* Every flip timed out on the previous run, which has two very
         * different explanations -- the RSX is not executing our command
         * buffer at all, or it is but the display engine never retires the
         * flip. These two probes separate them before any more guessing. */
        alive = rsx_probe_alive();
        emitf("   RSX executes commands: %s", alive ? "YES" : "NO (timeout)");

        /* The display path with the GPU taken out of the picture: fill a
         * framebuffer with the CPU and present it. */
        rsx_fill_cpu(0, 0xFFCC2020u);           /* red */
        shown = rsx_present_cpu(0);
        emitf("   CPU-filled buffer 0 presented: %s", shown ? "YES" : "NO");
        sleep(2);

        rsx_fill_cpu(1, 0xFF20CC20u);           /* green */
        shown = rsx_present_cpu(1);
        emitf("   CPU-filled buffer 1 presented: %s", shown ? "YES" : "NO");
        sleep(2);

        /* GPU clear, which the previous run proved works. */
        for (f = 0; f < 60; f++) {
            rsx_frame_begin();
            rsx_clear(0xFF2060CCu);                 /* blue */
            rsx_frame_end();
        }
        emitf("   GPU clears: %llu frames, %llu flip timeouts",
              (unsigned long long)g_rsx.frames,
              (unsigned long long)g_rsx.flip_timeouts);

        /* The claim the whole graphics port rests on: that the microcode our
         * own encoders emit is executed by a real RSX. Verified against
         * cgcomp offline, but "matches the reference assembler" and "runs on
         * the GPU" are different claims. A visible triangle settles the second. */
        /* Two shader modes, because a solid triangle is what both a broken
         * fragment program and a broken interpolant produce. Mode 0 writes a
         * known constant -- if that colour is not what appears, the fault is in
         * the fragment program or its upload. Mode 1 writes the interpolated
         * colour, sampled near each vertex so "is it interpolating at all" is
         * answered by three numbers rather than one. */
        stage("geometry: constant-colour fragment program");
        if (rsx_tritest_init(0) != 0) {
            emit_line(NULL, "   setup FAILED");
        } else {
            const u32 clear = 0xFF101018u;
            u32 centre = 0;
            int b = 0;
            for (f = 0; f < 60; f++) {
                b = g_rsx.current;
                rsx_frame_begin();
                rsx_clear(clear);
                rsx_tritest_draw();
                if (f == 30) { rsx_wait_idle(); centre = rsx_sample(b, 0.5f, 0.5f); }
                rsx_frame_end();
            }
            emitf("   centre %08x (expect ~ff8040 = 1.0,0.5,0.25)",
                  (unsigned)centre);
            emitf("   fragment output path: %s",
                  ((centre >> 16 & 0xFF) > 0xE0 &&
                   (centre >>  8 & 0xFF) > 0x60 && (centre >> 8 & 0xFF) < 0xA0)
                  ? "CORRECT" : "WRONG");
            rsx_tritest_shutdown();
        }

        /* Which byte order does the fragment-program fetch actually use? The
         * same constant-colour program is uploaded four ways; the variant that
         * renders ~ff8040 is the truth, everything else is theory. */
        stage("geometry: fragment byte-order sweep");
        if (rsx_tritest_init(0) != 0) {
            emit_line(NULL, "   setup FAILED");
        } else {
            static const char *k_vname[4] = {
                "as-stored (halfswapped)", "halfword-swapped back (raw)",
                "full byteswap", "byteswap within halves" };
            int v;
            for (v = 0; v < 4; v++) {
                const u32 clear = 0xFF101018u;
                u32 c = 0;
                int b = 0;
                if (rsx_tritest_fp_variant(v) != 0) break;
                for (f = 0; f < 30; f++) {
                    b = g_rsx.current;
                    rsx_frame_begin();
                    rsx_clear(clear);
                    rsx_tritest_draw_novariant();
                    if (f == 15) { rsx_wait_idle(); c = rsx_sample(b, 0.5f, 0.5f); }
                    rsx_frame_end();
                }
                emitf("   variant %d (%s): centre %08x %s", v, k_vname[v],
                      (unsigned)c,
                      ((c & 0xFFFFFF) >= 0xFE7F30 && (c & 0xFFFFFF) <= 0xFF8150)
                      ? "<-- CORRECT" : "");
            }
            rsx_tritest_shutdown();
        }

        /* The reference shader: cgcomp's own binary, loaded with PSL1GHT's own
         * accessors. This is the control that separates "our shader container
         * is wrong" from "the state around the draw is wrong". */
        stage("geometry: cgcomp reference shader");
        if (rsx_tritest_init(2) != 0) {
            emit_line(NULL, "   setup FAILED");
        } else {
            const u32 clear = 0xFF101018u;
            u32 rr = 0, gg = 0, bb = 0;
            int b = 0;
            for (f = 0; f < 90; f++) {
                b = g_rsx.current;
                rsx_frame_begin();
                rsx_clear(clear);
                rsx_tritest_draw();
                if (f == 45) {
                    rsx_wait_idle();
                    rr = rsx_sample(b, 0.28f, 0.70f);
                    gg = rsx_sample(b, 0.72f, 0.70f);
                    bb = rsx_sample(b, 0.50f, 0.28f);
                }
                rsx_frame_end();
            }
            emitf("   ref near-red %08x  near-green %08x  near-blue %08x",
                  (unsigned)rr, (unsigned)gg, (unsigned)bb);
            emitf("   reference interpolating: %s",
                  (rr != gg || gg != bb) ? "YES" : "NO -- flat");
            rsx_tritest_shutdown();
        }

        /* Position-as-colour: a known-good attribute driving the same
         * interpolant. Splits "attribute 3 never arrives" from "the
         * interpolant is not interpolating". */
        stage("geometry: position routed into COL0");
        if (rsx_tritest_init(3) != 0) {
            emit_line(NULL, "   setup FAILED");
        } else {
            const u32 clear = 0xFF101018u;
            u32 rr = 0, gg = 0, bb = 0;
            int b = 0;
            for (f = 0; f < 90; f++) {
                b = g_rsx.current;
                rsx_frame_begin();
                rsx_clear(clear);
                rsx_tritest_draw();
                if (f == 45) {
                    rsx_wait_idle();
                    rr = rsx_sample(b, 0.28f, 0.70f);
                    gg = rsx_sample(b, 0.72f, 0.70f);
                    bb = rsx_sample(b, 0.50f, 0.28f);
                }
                rsx_frame_end();
            }
            emitf("   pos-as-col %08x %08x %08x", (unsigned)rr, (unsigned)gg,
                  (unsigned)bb);
            {
                /* What the GPU actually fetches. The program is read from RSX
                 * memory by the hardware, so the bytes sitting there are the
                 * only ones that matter -- not the ones we meant to copy. */
                u32 w[8], n, im = 0, om = 0, vpw;
                n = rsx_tritest_fp_words(w, 8);
                vpw = rsx_tritest_vp_masks(&im, &om);
                emitf("   fp offset %08x, %u words in RSX memory:",
                      (unsigned)rsx_tritest_fp_offset(), (unsigned)n);
                if (n >= 4)
                    emitf("     %08x %08x %08x %08x",
                          (unsigned)w[0], (unsigned)w[1],
                          (unsigned)w[2], (unsigned)w[3]);
                emitf("   vp %u words, input_mask %08x output_mask %08x "
                      "(expect 00000009 / 00000005)",
                      (unsigned)vpw, (unsigned)im, (unsigned)om);
            }
            emitf("   varies with a known-good attribute: %s",
                  (rr != gg || gg != bb) ? "YES -- attr 3 data path is at fault"
                                         : "NO -- the interpolant itself");
            rsx_tritest_shutdown();
        }

        /* Colour carried through TEX0 rather than COL0. */
        stage("geometry: varying through TEX0");
        if (rsx_tritest_init(4) != 0) {
            emit_line(NULL, "   setup FAILED");
        } else {
            const u32 clear = 0xFF101018u;
            u32 rr = 0, gg = 0, bb = 0, im = 0, om = 0;
            int b = 0;
            for (f = 0; f < 90; f++) {
                b = g_rsx.current;
                rsx_frame_begin();
                rsx_clear(clear);
                rsx_tritest_draw();
                if (f == 45) {
                    rsx_wait_idle();
                    rr = rsx_sample(b, 0.28f, 0.70f);
                    gg = rsx_sample(b, 0.72f, 0.70f);
                    bb = rsx_sample(b, 0.50f, 0.28f);
                    rsx_tritest_vp_masks(&im, &om);
                }
                rsx_frame_end();
            }
            emitf("   tex0 %08x %08x %08x (masks %08x/%08x, expect om 00004000)",
                  (unsigned)rr, (unsigned)gg, (unsigned)bb,
                  (unsigned)im, (unsigned)om);
            emitf("   TEX0 interpolating: %s",
                  (rr != gg || gg != bb) ? "YES -- COL0 routing is the fault"
                                         : "NO -- all interpolants are flat");
            rsx_tritest_shutdown();
        }

        /* WPOS through the fragment program: the rasteriser's own product,
         * touching none of the suspect paths. White = FP executes correctly. */
        stage("geometry: fragment reads WPOS");
        if (rsx_tritest_init(5) != 0) {
            emit_line(NULL, "   setup FAILED");
        } else {
            const u32 clear = 0xFF101018u;
            u32 c1 = 0, c2 = 0;
            int b = 0;
            for (f = 0; f < 90; f++) {
                b = g_rsx.current;
                rsx_frame_begin();
                rsx_clear(clear);
                rsx_tritest_draw();
                if (f == 45) {
                    rsx_wait_idle();
                    c1 = rsx_sample(b, 0.50f, 0.50f);
                    c2 = rsx_sample(b, 0.40f, 0.60f);
                }
                rsx_frame_end();
            }
            emitf("   wpos samples %08x %08x", (unsigned)c1, (unsigned)c2);
            /* WPOS = (x_pix, y_pix, z, 1/w): x and y saturate, z is the
             * depth, 0.5 for this flat triangle. ffffff80 is the correct
             * answer, not white -- the first version of this check demanded
             * white and flagged a perfectly working program. */
            emitf("   fragment program executes: %s",
                  ((c1 & 0x00FFFF00) == 0x00FFFF00 &&
                   ((c1 >> 0) & 0xFF) > 0x70 && ((c1 >> 0) & 0xFF) < 0x90)
                  ? "YES (x,y saturated, z=0.5)" : "NOT CORRECTLY");
            rsx_tritest_shutdown();
        }

        stage("geometry: interpolated colour");
        if (rsx_tritest_init(1) != 0) {
            emit_line(NULL, "   setup FAILED");
        } else {
            const u32 clear = 0xFF101018u;
            u32 near_r = 0, near_g = 0, near_b = 0, centre = 0;
            u32 differing = 0;
            int b = 0;
            static u32 cmdbuf[512];
            u32 cmdn = 0;
            for (f = 0; f < 120; f++) {
                b = g_rsx.current;
                if (f == 60) rsx_cmd_mark();
                rsx_frame_begin();
                rsx_clear(clear);
                rsx_tritest_draw();
                if (f == 60) {
                    /* The full frame's command words, captured before the flip
                     * appends its own. Decoded offline against nv40.h. */
                    cmdn = rsx_cmd_since_mark(cmdbuf, 512);
                    rsx_wait_idle();
                    /* Screen coords: the triangle spans roughly x 0.2-0.8,
                     * y 0.2-0.75 after the y-flip. Sample inside each corner. */
                    near_r    = rsx_sample(b, 0.28f, 0.70f);   /* red vertex   */
                    near_g    = rsx_sample(b, 0.72f, 0.70f);   /* green vertex */
                    near_b    = rsx_sample(b, 0.50f, 0.28f);   /* blue vertex  */
                    centre    = rsx_sample(b, 0.50f, 0.50f);
                    differing = rsx_count_differing(b, clear, NULL);
                }
                rsx_frame_end();
            }
            emitf("   pixels drawn: %u", (unsigned)differing);
            emitf("   near-red   %08x", (unsigned)near_r);
            emitf("   near-green %08x", (unsigned)near_g);
            emitf("   near-blue  %08x", (unsigned)near_b);
            emitf("   centre     %08x", (unsigned)centre);
            emitf("   interpolating: %s",
                  (near_r != near_g || near_g != near_b) ? "YES" : "NO -- flat");
            {
                u32 k;
                emitf("   command stream, %u words:", (unsigned)cmdn);
                for (k = 0; k + 8 <= cmdn; k += 8)
                    emitf("   %08x %08x %08x %08x %08x %08x %08x %08x",
                          (unsigned)cmdbuf[k+0], (unsigned)cmdbuf[k+1],
                          (unsigned)cmdbuf[k+2], (unsigned)cmdbuf[k+3],
                          (unsigned)cmdbuf[k+4], (unsigned)cmdbuf[k+5],
                          (unsigned)cmdbuf[k+6], (unsigned)cmdbuf[k+7]);
                for (; k < cmdn; k++)
                    emitf("   %08x", (unsigned)cmdbuf[k]);
            }
            rsx_tritest_shutdown();
        }

        /* The milestone stage: a real GameCube program, recompiled and executed
         * on the PPE, drawing through the emulated GX pipeline onto the RSX.
         * Every store the guest makes to the write-gather pipe travels the
         * whole road: recompiler -> MMIO -> FIFO -> parser -> state tracker ->
         * generated shaders -> rasteriser -> television. */
        stage("guest Wii code renders a triangle");
        {
            static PPCState cpu;
            DolHeader h;
            GXBackend be;
            unsigned slices = 0;
            int finished = -1;

            mem_reset();
            emit_line(NULL, "   [tri] mem_reset ok");
            /* hw_init, not merely hw_reset: init is what registers the MMIO
             * handlers. Without it every store the guest makes to the gather
             * pipe is silently dropped, and the guest "finishes" having drawn
             * nothing -- which is exactly how the first run of this stage went. */
            hw_init(&cpu, 1);
            emit_line(NULL, "   [tri] hw_init ok");
            hw_reset();
            emit_line(NULL, "   [tri] hw_reset ok");
            jit_flush_all();
            emit_line(NULL, "   [tri] jit_flush ok");

            if (gx_render_init() != 0) {
                emit_line(NULL, "   gx_render init FAILED");
            } else if (dol_load(gxtri_blob, sizeof gxtri_blob, &h) != 0) {
                emit_line(NULL, "   guest DOL did not load");
            } else {
                gx_render_bind(&be);
                gx_state_init(gx_state(), &be);
                dol_setup_boot_state(&cpu, &h, 1);
                emit_line(NULL, "   [tri] gx bound, boot state set");

                rsx_frame_begin();
                rsx_clear(0xFF101018u);
                gx_render_frame_begin();

                emitf("   [tri] entry %08x, first slice...",
                      (unsigned)cpu.pc);
                for (slices = 0; slices < 2000; slices++) {
                    cpu.downcount = 20000;
                    cpu.exit_requested = 0;
                    jit_run(&cpu);
                    gx_state_run(gx_state());
                    /* Unbuffered breadcrumbs: if this stage wedges, the last
                     * line in the report says exactly which slice and where
                     * the guest pc was -- or, if nothing prints after "first
                     * slice...", that jit_run itself never came back. */
                    if (slices < 3 || (slices % 200) == 0)
                        emitf("   [tri] slice %u pc=%08x dc=%d",
                              slices, (unsigned)cpu.pc, (int)cpu.downcount);
                    if (cpu.pc == VEC_PROGRAM) {
                        finished = (cpu.spr[SPR_SRR0] == 0) ? 0 : -2;
                        break;
                    }
                }

                emitf("   guest %s (%u slices)",
                      finished == 0 ? "finished" :
                      finished == -2 ? "FAULTED" : "did not finish",
                      (unsigned)slices);
                emitf("   draws %llu, vertices %llu, programs built %llu, "
                      "overflow %llu",
                      (unsigned long long)g_gx_render.draws,
                      (unsigned long long)g_gx_render.vertices,
                      (unsigned long long)g_gx_render.programs_built,
                      (unsigned long long)g_gx_render.overflow);

                {
                    u32 diff, centre;
                    int b = g_rsx.current;
                    rsx_wait_idle();
                    diff = rsx_count_differing(b, 0xFF101018u, &centre);
                    emitf("   pixels drawn %u, centre %08x",
                          (unsigned)diff, (unsigned)centre);
                    emitf("   WII TRIANGLE ON SCREEN: %s",
                          diff > 1000 ? "YES" : "no");
                    rsx_frame_end();
                    /* Hold the drawn buffer on screen so it can be seen. */
                    rsx_present_cpu(b);
                    sleep(3);
                }
                gx_render_shutdown();
            }
        }

        /* The frame loop: sixty animated frames through the EFB-copy
         * lifecycle. This is a game's skeleton -- draw, declare the frame
         * done, repeat -- and the timing across it is the first honest
         * frames-per-second number for guest-driven rendering. */
        stage("guest frame loop (60 frames, animated)");
        {
            static PPCState cpu;
            DolHeader h;
            GXBackend be;
            unsigned slices = 0;
            int finished = -1;
            u64 t0, t1;

            mem_reset();
            hw_init(&cpu, 1);
            hw_reset();
            jit_flush_all();

            if (gx_render_init() != 0) {
                emit_line(NULL, "   gx_render init FAILED");
            } else if (dol_load(gxanim_blob, sizeof gxanim_blob, &h) != 0) {
                emit_line(NULL, "   guest DOL did not load");
            } else {
                gx_render_bind(&be);
                gx_render_set_frame_handler(anim_frame_done, NULL);
                gx_state_init(gx_state(), &be);
                dol_setup_boot_state(&cpu, &h, 1);
                s_anim_frames = 0;
                s_anim_first = s_anim_last = 0;

                rsx_frame_begin();
                rsx_clear(0xFF101018u);
                gx_render_frame_begin();

                t0 = read_timebase();
                for (slices = 0; slices < 20000; slices++) {
                    cpu.downcount = 20000;
                    cpu.exit_requested = 0;
                    jit_run(&cpu);
                    gx_state_run(gx_state());
                    if (cpu.pc == VEC_PROGRAM) {
                        finished = (cpu.spr[SPR_SRR0] == 0) ? 0 : -2;
                        break;
                    }
                }
                t1 = read_timebase();

                {
                    double secs = (double)(t1 - t0) / PS3_TIMEBASE_HZ;
                    emitf("   guest %s, %u frames in %.3f s = %.1f fps",
                          finished == 0 ? "finished" : "DID NOT FINISH",
                          (unsigned)s_anim_frames, secs,
                          secs > 0.0 ? (double)s_anim_frames / secs : 0.0);
                    emitf("   first frame sample %08x, last %08x -- %s",
                          (unsigned)s_anim_first, (unsigned)s_anim_last,
                          (s_anim_first != s_anim_last && s_anim_frames > 30)
                          ? "ANIMATING" : "not animating");
                }
                gx_render_shutdown();
            }
        }

        /* The headline: boot the real Mario Kart Wii executable on the PPE and
         * put its status -- including the anti-piracy result -- on the TV. Only
         * where generated code can execute (real PS3, not RPCS3's W^X). */
        if (execution_allowed()) {
            mkwii_boot_stage();
            sleep(12);          /* hold the status screen so it can be read */
        }
        rsx_video_shutdown();
    }

finished:
    /* SUPERVISOR: unless the user (or the XMB) asked us to quit, start another
     * session in-process instead of exiting. The console cannot be launched
     * remotely -- webMAN's play/mount endpoints return 200 without starting a
     * title, and this firmware's PS3MAPI reports every command unimplemented
     * (both verified) -- so the only route to unattended development is to
     * never need launching twice. One manual start now yields an unbounded
     * number of automated test cycles, and a build deployed mid-session is
     * picked up by the self-relaunch command over devlink. */
    if (!s_exit_requested && !s_relaunch_requested && s_sessions < 1000) {
        s_sessions++;
        emitf("==== session %u ended; restarting ====", s_sessions);
        devlink_poll();          /* keep the developer link alive across it */
        if (rsx_video_init() != 0) {
            emit_line(NULL, "   restart: video init failed, exiting");
        } else {
            goto session_restart;
        }
    }
    emit_line(NULL, "");
    if (s_exit_requested)
        emit_line(NULL, "==== end (XMB quit) ====");
    else
        emit_line(NULL, "==== end (clean exit) -- returning to XMB now ====");
    ios_fs_persist_save(NAND_DIR);   /* the save survives the session */
    if (s_relaunch_requested) {
        /* Replace this process with a fresh copy of the EBOOT on disk, so a
         * build can be deployed and restarted with nobody at the console --
         * one manual start then yields an unlimited number of test cycles.
         *
         * Two things matter here and both were wrong before. The announcement
         * must be written and FLUSHED while the report file is still open,
         * otherwise a failed spawn leaves no trace at all (that is exactly how
         * the first attempt vanished). And the spawn must happen BEFORE the
         * emulator is torn down: if it fails we still hold a working session
         * and can carry on, instead of dropping the console to the XMB and
         * costing a manual relaunch. A successful spawn never returns, and the
         * kernel reclaims everything this process held. */
        /* Try the non-NPDRM SELF first. The NPDRM EBOOT is accepted by the
         * exitspawn syscall -- it never returns -- but the loader then starts
         * nothing, proven by a boot breadcrumb that stays at one line across
         * the attempt. A plain SELF takes a different validation path. */
        static const char *k_targets[2] = {
            "/dev_hdd0/game/DOLPHIN01/USRDIR/RELOAD.SELF",
            "/dev_hdd0/game/DOLPHIN01/USRDIR/EBOOT.BIN"
        };
        const char *k_self = k_targets[0];
        const char *argv[2];
        struct stat tst;
        unsigned ti;
        for (ti = 0; ti < 2; ti++)
            if (stat(k_targets[ti], &tst) == 0) { k_self = k_targets[ti]; break; }
        argv[0] = k_self; argv[1] = NULL;
        {
            struct stat st;
            if (stat(k_self, &st) != 0)
                emitf("==== relaunch ABORTED: cannot stat %s ====", k_self);
            else
                emitf("==== relaunching from disk (%lld bytes) ====",
                      (long long)st.st_size);
        }
        if (s_fd >= 0) fsync(s_fd);
        sysUtilUnregisterCallback(SYSUTIL_EVENT_SLOT0);
        /* Returns void: on success it never comes back. */
        sysProcessExitSpawn2(k_self, argv, NULL, NULL, 0, 1001,
                             SYS_PROCESS_SPAWN_STACK_SIZE_1M);
        emit_line(NULL, "   relaunch FAILED (spawn returned) -- "
                        "continuing this session");
        if (s_fd >= 0) fsync(s_fd);
        sysUtilRegisterCallback(SYSUTIL_EVENT_SLOT0, ps3_sysutil_cb, NULL);
        s_relaunch_requested = 0;
        s_exit_requested = 0;
        if (rsx_video_init() == 0) {
            s_sessions++;
            goto session_restart;
        }
        emit_line(NULL, "   relaunch fallback: video init failed");
    }
    sysUtilUnregisterCallback(SYSUTIL_EVENT_SLOT0);
    if (s_fd >= 0) { close(s_fd); s_fd = -1; }
    jit_shutdown();
    mem_shutdown();
    sysProcessExit(0);   /* explicit: return to XMB, do not sit on a black screen */
    return 0;
}
