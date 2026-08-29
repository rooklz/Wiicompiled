/* wc_boot.cpp -- start the statically recompiled game.
 *
 * This is the port's entry: no JIT, no dispatcher, no block cache. The guest's
 * code is native PPE code in this image; what remains is to give it the machine
 * state a real console's loader would have left, and then call it.
 *
 * THE ONE HARD PROBLEM is yielding. Under emulation the JIT returns to the run
 * loop every slice, which is where interrupts get delivered and where the frame
 * loop lives. Native code returns when the guest function returns, and the
 * guest's entry point does not return -- it runs the game. Something has to
 * deliver VI retrace, IPC completion and the audio DMA callback while it runs.
 *
 * The guest's own OS is translated along with everything else, so it still has
 * its scheduler (SelectThread) and its interrupt handlers. What it no longer
 * has is anything to *raise* an interrupt. So the port runs the guest on its
 * own host thread and drives the devices from the main thread, exactly as the
 * hardware did: the device model already exists (src/core/hw), and it already
 * knows how to set the interrupt cause bits the guest polls and how to invoke
 * the guest's handler.
 *
 * Bring-up order below matters and mirrors the console's:
 *   1. guest RAM must exist before anything is written into it
 *   2. the DOL's data sections are placed (wc_data_init)
 *   3. r2/r13 get the small-data bases the boot code would have installed --
 *      before any translated code runs, because every global access uses them
 *   4. the stack pointer is placed in MEM1 where the SDK's own boot leaves it
 *   5. HID2 enables paired singles and quantised load/store, which the guest
 *      assumes from its first vector op
 */
extern "C" {
#include "../../../common/log.h"
#include "../gekko.h"
#include "../../mem/memmap.h"
}
#include "ppc_runtime.h"
#include "memory.h"
#include "wc_ps.h"
#include "gen/wc_calls.h"
#include <cstring>
#ifdef __PS3__
#include <ppu-lv2.h>
#include <sys/thread.h>
#endif

extern "C" void wc_data_init(void);
extern "C" void wc_memory_init(void);
extern "C" void wc_os_init(void);
extern "C" void wc_irq_start(void);

/* The guest's entry point, from the DOL header (NTSC-U RMCE01). */
extern "C" void func_800060A4(CpuContext *ctx);

/* RuntimeConfig.h values, read out of __init_registers' lis/ori pairs. */
static const uint32_t kSda1Base = 0x80388880u;   /* r13 */
static const uint32_t kSda2Base = 0x8038AC20u;   /* r2  */

/* Where the SDK's boot code leaves the stack: the top of MEM1, below the
 * arena high mark the loader publishes at 0x80000034. */
static const uint32_t kInitialStack = 0x817F8000u;

CpuContext g_wc_ctx;

/* Bumped by the call layer so the device loop can tell a game that is making
 * progress from one that is spinning. Without it a wedge is indistinguishable
 * from a crash from outside, which is what made the first threaded boot so
 * expensive to diagnose. */
/* g_wc_calls and g_wc_crumb live in wc_os.cpp, not here: the dispatch layer
 * that writes them is linked into the differential-test harness too, and that
 * harness does not link this file. */

static void wc_game_thread(void *arg)
{
    CpuContext *c = &g_wc_ctx;
    (void)arg;
    wc_current_ctx = c;
    func_800060A4(c);
    /* Returning at all means the game exited, which on a console only happens
     * on a reset request. */
    LOG_WARN(LOG_CORE, "WC: translated entry RETURNED (lr=%08x r3=%08x)",
             c->lr, c->gpr[3]);
#ifdef __PS3__
    sysThreadExit(0);
#endif
}

/* sys_memory_get_user_memory_size (syscall 352) fills two u64: the total
 * user memory the process was granted and how much of it is still available.
 * PSL1GHT has no wrapper for it, and without it "allocation failed" carries no
 * information about how much was actually left. */
static void wc_user_memory(u64 out[2])
{
#ifdef __PS3__
    /* sys_memory_info_t is two 32-bit counts, not two 64-bit ones. Declared
     * as u64 it reported a total of 919191720826208 KiB and 0 available --
     * two adjacent u32 reads fused into one garbage number, and an
     * "available" that was really the high half of nothing. */
    struct { u32 total, avail; } info = {0, 0};
    lv2syscall1(352, (u64)(uintptr_t)&info);
    out[0] = info.total;
    out[1] = info.avail;
#else
    out[0] = out[1] = 0;
#endif
}

extern "C" void wc_canary_service(void);
extern "C" { extern volatile int g_wc_canary_state; }

extern "C" int wc_boot(void)
{
    LOG_WARN(LOG_CORE, "WCBUILD %s %s", __DATE__, __TIME__);
    CpuContext *c = &g_wc_ctx;

    {   /* Baseline, before the port allocates anything of its own: how much
         * user memory the image left the process. Logged unconditionally
         * because every allocation failure downstream is read against it. */
        u64 mem[2] = {0, 0};
        wc_user_memory(mem);
        LOG_INFO(LOG_CORE, "WC: user memory total %llu KiB, available %llu KiB",
                 (unsigned long long)(mem[0] >> 10),
                 (unsigned long long)(mem[1] >> 10));
    }

    if (!g_mem.mem1) {
        LOG_ERROR(LOG_CORE, "WC: guest memory not initialised");
        return -1;
    }
    /* The port has NO slow path. Translated code indexes the arena as
     * base + Fold(ea) with no bounds test and no backing-pointer lookup -- that
     * is the whole point of it, and it is what makes a guest load one
     * instruction instead of a call. It also means that without the arena the
     * base is NULL and the first guest load is a null dereference somewhere
     * inside 13,675 generated functions, which is the least debuggable crash
     * this port could produce.
     *
     * The emulator tolerates a missing arena by falling back to the
     * interpreter. The port cannot, so it says so here rather than dying
     * later with no explanation. */
    if (!g_mem.fastmem_ok || !mem_base()) {
        LOG_ERROR(LOG_CORE, "WC: the port requires the fastmem arena "
                            "(fastmem_ok=%d, base=%p) -- refusing to run "
                            "translated code against a null arena",
                  g_mem.fastmem_ok, (void *)mem_base());
        return -1;
    }
    wc_memory_init();
    /* Bring the guest threading layer up BEFORE any guest code runs.
     *
     * It was never called. Every entry point in it begins "if (!g_lock_ready)
     * return", so with the mutex uncreated the whole layer was silently inert:
     * __OSThreadInit adopted nothing, and OSLoadContext -- the override whose
     * entire job is to make a thread switch actually happen -- returned
     * without switching. The guest scheduler ran perfectly, picked the next
     * thread, called the switch, and stayed exactly where it was. */
    wc_os_init();
    std::memset(c, 0, sizeof *c);
    wc_data_init();

    /* Arm the low-memory canary here: the arena is live and the game thread
     * has not started, so the disc id is provably intact at arm time and the
     * arm cannot race the ctor-pass stomp. The 50 us watcher (main.c) and
     * the per-call hook both key off the pointer this sets. */
    wc_canary_service();
    LOG_INFO(LOG_CORE, "WC: canary armed=%d", g_wc_canary_state == 1);

    c->gpr[1]  = kInitialStack;
    c->gpr[2]  = kSda2Base;
    c->gpr[13] = kSda1Base;
    c->pc      = 0x800060A4u;
    c->msr     = MSR_FP;
    c->hid2    = HID2_PSE | HID2_LSQE;
    wc_current_ctx = c;

    LOG_INFO(LOG_CORE, "WC: entering translated game at %08x "
             "(sp=%08x r2=%08x r13=%08x)",
             c->pc, c->gpr[1], c->gpr[2], c->gpr[13]);

    /* The game runs on its own thread, and the caller keeps the device loop.
     *
     * This is not a detail: the guest's very first frame ends in
     * VIWaitForRetrace, which blocks until a retrace interrupt arrives. The
     * interrupt comes from the VI model, the VI model is driven by the timing
     * loop, and the timing loop is the caller. Run the game *instead* of that
     * loop -- which is what the first boot did -- and the game reaches its
     * first wait and stops there forever, having done everything right.
     *
     * On the console the CPU and the video hardware ran concurrently. Two
     * threads is the same arrangement, and the same one the SPU and RSX
     * already use here. */
#ifdef __PS3__
    {   sys_ppu_thread_t t;
        /* Priority 1500, deliberately BELOW the device loop, the rescue
         * listener and the network threads (lv2 counts up from 0, so a larger
         * number yields to a smaller one).
         *
         * The game thread runs native code with no yield of its own: if it
         * spins -- waiting on a device flag that has not been set yet, which
         * is exactly what early boot does -- at equal priority it starves
         * everything, including the threads that would have set the flag and
         * the ones that let a developer see why. That happened on the first
         * threaded boot and cost the console. Running the game below its own
         * infrastructure means a spin costs frames, not control. */
        int trc = sysThreadCreate(&t, wc_game_thread, NULL, 1500, 0x40000, 0,
                                  (char *)"mkw-game");
        if (trc != 0) {
            /* Report the reason, not just the fact. This failed once and the
             * only evidence was "could not create", which is consistent with
             * out-of-memory, a bad priority and a stack-size limit alike --
             * three different fixes. lv2 answers all three: the return code
             * says which, and the user-memory figures say whether the image
             * left the process anything to allocate from. */
            u64 mem[2] = {0, 0};
            wc_user_memory(mem);
            LOG_ERROR(LOG_CORE,
                      "WC: could not create the game thread: rc=%d (0x%08x); "
                      "user memory total %llu KiB, available %llu KiB",
                      trc, (unsigned)trc,
                      (unsigned long long)(mem[0] >> 10),
                      (unsigned long long)(mem[1] >> 10));
            return -1;
        }
        /* Watches guest progress and parks this thread if it stops making any,
         * so a spin costs frames rather than the console. */
        {   extern void wc_watchdog_start(void *game_thread_id);
            wc_watchdog_start(&t);
        }
        /* Device interrupts, without which every IOS call blocks forever. */
        wc_irq_start();
    }
#else
    wc_game_thread(NULL);
#endif
    return 0;
}
