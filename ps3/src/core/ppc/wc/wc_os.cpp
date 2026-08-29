/* wc_os.cpp -- guest threading and interrupts for the native port.
 *
 * WHY THIS EXISTS AT ALL
 *
 * Everything else in the guest's OS can stay translated: it reads and writes
 * hardware registers, and the device model behind Memory:: answers exactly as
 * the hardware would. Context switching cannot. The guest switches threads by
 * saving the PowerPC register file into an OSContext and loading another one --
 * but after static recompilation the guest's "registers" are C locals in the
 * frames of native functions. There is nothing to save and nowhere to jump.
 *
 * So guest threads become host threads, and the guest's scheduler is replaced
 * rather than emulated.
 *
 * THE MODEL: one runner, handed off explicitly
 *
 * The Wii ran one thread at a time on one core, and the game is written to
 * that. Reproducing it exactly is both the simplest correct choice and the
 * fastest to reason about: exactly one guest thread is runnable at any instant,
 * and a switch is a hand-off. No guest data race can exist that did not exist
 * on the console, which matters when the goal is a port with no behavioural
 * differences rather than a plausible-looking one.
 *
 * Concurrency is not lost -- it moves. The RSX, the SPUs, audio output and the
 * device model all run alongside on their own threads, as the real hardware's
 * GPU, DSP and DI did.
 *
 * INTERRUPTS
 *
 * A game that never yields cannot be interrupted, and translated code never
 * yields on its own. It does not need to: everything the guest waits for it
 * waits for *by blocking* -- VIWaitForRetrace sleeps on a message queue, a DVD
 * read sleeps until completion, the audio callback runs off DMA. So a pending
 * interrupt is delivered at the next blocking point, which is precisely where
 * the console would have delivered it too, and the frame loop keeps its shape.
 */
extern "C" {
#include "../../../common/log.h"
#include "../gekko.h"
#include "../../mem/memmap.h"
}
#include "ppc_runtime.h"
#include "memory.h"
#include "gen/wc_calls.h"
#include <cstring>
#include <unistd.h>
#include <cstdio>
#include <csetjmp>

extern "C" void wc_where(unsigned id);

/* Defined with the delivery instrumentation further down; referenced by the
 * OSLoadContext hook above it. */
extern volatile int      g_wc_irq_owner;
extern volatile unsigned g_wc_irq_excnum, g_wc_irq_cause;
#ifdef __PS3__
#include <sys/thread.h>
#include <sys/mutex.h>
#include <sys/cond.h>
#include <sys/systime.h>
#endif

namespace {

/* A guest thread. `ctx` is its register file; `entry` the translated function
 * it was created to run. The OSThread the game allocated stays in guest memory
 * and stays authoritative for anything the game reads out of it -- this is
 * only what the host needs to run it. */
struct GuestThread {
    uint32_t   osthread;        /* guest address of its OSThread   */
    uint32_t   entry;           /* guest function it runs          */
    uint32_t   param;           /* r3 at entry                     */
    CpuContext ctx;
    /* The context this thread's translated frames are actually using. For a
     * thread created here that is the `ctx` above. For the initial thread it
     * is wc_boot's own context, which the game's whole call stack already
     * holds a pointer to and which cannot be rebound underneath it -- so the
     * slot points at that one instead of shadowing it with an empty copy. */
    CpuContext *ctxp;
    int        runnable;
    volatile int parked;   /* host thread inside sysCondWait: diagnostics + all-parked gate */
    int        finished;
    int        exiting;          /* in OSExitThread: hand off, then die */
    int        used;
#ifdef __PS3__
    sys_ppu_thread_t host;
    sys_cond_t       wake;
#endif
};

/* __OSCurrentContext, and the context the system software parks in when it idles. The
 * idle context's address is fixed in the image; SelectThread loads it as a
 * literal (0x80340000 + 13616) on the no-runnable-thread path. */
const uint32_t kCurrentContext = 0x800000D4u;
const uint32_t kIdleContext    = 0x80343530u;

/* The scheduler's run-queue hint: SelectThread's idle loop spins on this word
 * (read as [r13 - 25312] with r13 = 0x80388880). Nonzero means "something may
 * be runnable, run the scheduler". */
const uint32_t kRunQueueHint   = 0x80388880u - 25312u;   /* 0x803825A0 */

/* The OS exception-handler table pointer ([r13 - 25496]); entry 8 is the
 * decrementer handler (AlarmHandler once __OSInitAlarm has run). */
const uint32_t kExcTablePtr    = 0x80388880u - 25496u;   /* 0x803824E8 */
const uint32_t kExcDecrementer = 8u;

#define WC_MAX_THREADS 24
GuestThread g_threads[WC_MAX_THREADS];
int         g_current = -1;         /* index of the running guest thread */

/* DEDICATED PARKABLE DELIVERY THREAD.
 *
 * Poller-context delivery leaked poller state into the guest scheduler the
 * moment a handler slept: the sleep enqueued whatever "current" was, later
 * loads asked for OSThread 93640000 (the poller's stack base) and were
 * refused 4,008 times while the game waited forever. Deliveries now run on
 * a REGISTERED pseudo-guest thread with a synthetic OSThread in MEM2
 * headroom: a sleeping handler enqueues IT, legally; the wake schedules IT,
 * legally; every existing hand-off path applies. */
int          g_deliver_slot = -1;
volatile int g_deliver_kick;
volatile unsigned g_deliver_hb;      /* loop heartbeat: frozen = host-blocked */
volatile unsigned g_deliver_n;       /* deliveries performed by the thread    */
const uint32_t kDeliverThread = 0x93630000u;   /* synthetic OSThread block  */

/* True when every registered guest thread (delivery slot excluded) is parked:
 * the only state in which a delivery-thread DEC delivery cannot collide with
 * a boundary delivery and its possible mid-handler sleep. */
static int wc_all_guest_parked(void)
{
    int used_n = 0, parked_n = 0, i2;
    for (i2 = 0; i2 < WC_MAX_THREADS; i2++)
        if (g_threads[i2].used && !g_threads[i2].finished
            && i2 != g_deliver_slot) {
            used_n++;
            if (g_threads[i2].parked) parked_n++;
        }
    return used_n > 0 && parked_n == used_n;
}
const uint32_t kDeliverStack  = 0x9363F000u;   /* guest stack, grows down   */

#ifdef __PS3__
sys_mutex_t g_lock;                 /* guards everything above           */
int         g_lock_ready;
#endif

/* Interrupt state. `masked` mirrors the guest's own MSR[EE]/interrupt mask so
 * that a handler is never entered while the game has interrupts off, exactly
 * as the hardware would not. */
volatile int      g_int_masked;
volatile uint32_t g_int_pending;

} /* namespace */

/* Set while SOME thread is executing the guest's interrupt handler. Global,
 * not namespace-local: the call-boundary hook in every translated unit reads
 * it through ppc_runtime.h. */
volatile int g_wc_in_irq;

/* Set on whichever thread is currently inside the guest's interrupt handler,
 * so that thread's own nested guest calls do not wait for themselves. */
static __thread int t_in_irq;

/* This host thread's guest-thread slot. Identity must NOT be derived from
 * g_current: preemption moves g_current while the preempted thread is still
 * executing, and a thread that asks "who am I?" by reading g_current after
 * that gets someone else's answer -- the hand-off then waits on the wrong
 * predicate or returns as if already running. Set at adoption, at trampoline
 * start, and never changed for the life of the host thread. */
static __thread int t_my_slot = -1;

/* THE `rfi` AT THE END OF THE HANDLER.
 *
 * OSLoadContext ends in rfi, so it NEVER RETURNS: on the console every
 * instruction after the closing OSLoadContext in __OSDispatchInterrupt and in
 * ExternalInterruptHandler is unreachable. Calling the handler as an ordinary
 * function makes all of it reachable, and it runs on every single delivery --
 * which is why re-entering the handler was not idempotent, and why the guest's
 * IPC enqueue counter walked ahead of the requests it had actually built.
 *
 * setjmp/longjmp is the exact shape of what the hardware does: leave the
 * handler from wherever the context load happens, without unwinding through
 * the code that follows it. It is safe across translated code -- built with
 * -fno-exceptions, no destructors, plain locals in every frame. */
static __thread jmp_buf  t_irq_jmp;
static __thread int      t_irq_jmp_armed;
static __thread uint32_t t_irq_ctx;      /* the context the vector passed in */
static __thread uint32_t t_irq_switch_to; /* preemption target, carried out at exit */
static __thread int      t_irq_slept;     /* handler slept; its rfi is deferred */
static __thread int      t_irq_on_guest;  /* delivered on the game thread itself */


extern "C" {

void wc_os_init(void)
{
#ifdef __PS3__
    sys_mutex_attr_t ma;
    if (g_lock_ready) return;
    sysMutexAttrInitialize(ma);
    if (sysMutexCreate(&g_lock, &ma) == 0) g_lock_ready = 1;
#endif
    std::memset(g_threads, 0, sizeof g_threads);
    g_current = -1;
    g_int_masked = 0;
    g_int_pending = 0;
    LOG_INFO(LOG_CORE, "WC: guest threading up (single-runner hand-off model)");
}

/* Raise a device interrupt at the guest. Called from the host threads that
 * model the hardware (VI retrace, IPC completion, audio DMA). It never runs
 * guest code itself: it records the cause and lets the guest take it at its
 * next blocking point, which is where the console delivered it too. */
void wc_os_raise(uint32_t cause_bit)
{
    g_int_pending |= cause_bit;
}

int wc_os_interrupts_pending(void)
{
    return !g_int_masked && g_int_pending != 0;
}

/* The guest's OSDisableInterrupts/OSEnableInterrupts. Returns the previous
 * state, which is what the guest's callers store and restore. */
uint32_t wc_os_disable_interrupts(void)
{
    uint32_t prev = (uint32_t)(g_int_masked == 0);
    g_int_masked = 1;
    return prev;
}
uint32_t wc_os_enable_interrupts(void)
{
    uint32_t prev = (uint32_t)(g_int_masked == 0);
    g_int_masked = 0;
    return prev;
}
uint32_t wc_os_restore_interrupts(uint32_t level)
{
    uint32_t prev = (uint32_t)(g_int_masked == 0);
    g_int_masked = level ? 0 : 1;
    return prev;
}

} /* extern "C" */

/* ------------------------------------------------------------------ */
/* Threads                                                              */
/*                                                                      */
/* The guest switches threads by saving the PowerPC register file into  */
/* an OSContext and loading another. That is the one thing static       */
/* recompilation cannot reproduce: the guest's registers are C locals   */
/* in native frames, so there is nothing to save and nowhere to jump.   */
/* Its scheduler is therefore replaced, not emulated.                   */
/*                                                                      */
/* The model is the console's own: exactly one guest thread runs at a   */
/* time and a switch is a hand-off. That is not a simplification for    */
/* convenience -- it is what makes the port's concurrency identical to  */
/* the hardware's, so no guest data race can appear that could not have */
/* happened on a Wii. Real concurrency lives where it lived there: the  */
/* RSX, the SPUs, audio and the device model, all on their own threads. */
/*                                                                      */
/* The guest's OSThread structure stays in guest memory and stays       */
/* authoritative for everything the game reads out of it. What is kept  */
/* here is only what the host needs to run one.                         */
/* ------------------------------------------------------------------ */

namespace {

int find_thread(uint32_t osthread)
{
    int i;
    for (i = 0; i < WC_MAX_THREADS; i++)
        if (g_threads[i].used && g_threads[i].osthread == osthread) return i;
    return -1;
}

int alloc_thread(uint32_t osthread)
{
    int i;
    for (i = 0; i < WC_MAX_THREADS; i++)
        if (!g_threads[i].used) {
            std::memset(&g_threads[i], 0, sizeof g_threads[i]);
            g_threads[i].used = 1;
            g_threads[i].osthread = osthread;
            g_threads[i].ctxp = &g_threads[i].ctx;
#ifdef __PS3__
            /* The condvar has to exist before anything can be signalled on
             * it. The slot used to be memset to zero and handed back, and the
             * first hand-off then signalled condvar 0 -- which is not this
             * thread's, and may not be anyone's. */
            {   sys_cond_attr_t ca;
                sysCondAttrInitialize(ca);
                if (sysCondCreate(&g_threads[i].wake, g_lock, &ca) != 0) {
                    LOG_ERROR(LOG_CORE, "WC: no condvar for guest thread %d", i);
                    g_threads[i].used = 0;
                    return -1;
                }
            }
#endif
            return i;
        }
    return -1;
}

} /* namespace */

extern "C" {

void wc_os_thread_exited(void);   /* defined below; the trampoline needs it */

/* One host thread per guest thread. It parks immediately and runs nothing
 * until the guest's own scheduler hands it the processor, which is what keeps
 * exactly one guest thread running at any instant. */
static void wc_guest_thread_main(void *arg)
{
#ifdef __PS3__
    int me = (int)(intptr_t)arg;

    t_my_slot = me;
    sysMutexLock(g_lock, 0);
    g_threads[me].parked = 1;
    while (g_current != me && !g_threads[me].finished)
        sysCondWait(g_threads[me].wake, 0);
    g_threads[me].parked = 0;
    MemoryInline::Store<uint32_t>(0x800000E4u, g_threads[me].osthread);
    MemoryInline::Store<uint32_t>(kCurrentContext, g_threads[me].osthread);
    sysMutexUnlock(g_lock);

    if (!g_threads[me].finished) {
        /* Only one guest thread runs at a time, so the running one owns the
         * context pointer outright -- no TLS, and no cost on the paired-single
         * paths that read it. Every resume restores it; this is the first. */
        wc_current_ctx = g_threads[me].ctxp;
        g_threads[me].ctxp->gpr[3] = g_threads[me].param;
        InvokeIndirectCpu(g_threads[me].entry, g_threads[me].ctxp);
    }

    /* Falling off the end of a guest thread body is OSExitThread's job in the
     * game; if it returns anyway, do not leave the slot marked runnable or the
     * scheduler will hand the processor to a thread that no longer exists. */
    wc_os_thread_exited();
    sysMutexLock(g_lock, 0);
    {   int i;
        for (i = 0; i < WC_MAX_THREADS; i++)
            if (g_threads[i].used && !g_threads[i].finished && g_threads[i].runnable) {
                g_current = i;
                sysCondSignal(g_threads[i].wake);
                break;
            }
    }
    sysMutexUnlock(g_lock);
    sysThreadExit(0);
#else
    (void)arg;
#endif
}

/* Hand the processor to `next` and block until it is handed back.
 *
 * Called from the guest's scheduler wherever it would have loaded a context.
 * A thread that has never run is started here; one that has is simply woken.
 * The caller then waits, which is what makes this a hand-off rather than a
 * fork -- on return, this thread is the running one again. */
/* Ring of recent hand-offs, for the rescue "sched" command. */
volatile uint32_t g_wc_sw_log[16];
volatile unsigned g_wc_sw_n;

/* Preemption performed OFF the game thread: the handler (delivered by the
 * poller while the guest busy-waits) chose a thread; make it the running one
 * and signal its host thread. The preempted thread is NOT parked -- it keeps
 * spinning on its flag in parallel, which is the stall-breaker's already-
 * accepted concurrency: on hardware the handler's wakeups preempt the spin;
 * here they overlap it. Dropping the switch instead (the previous behaviour)
 * meant EGG's display thread -- woken by every retrace to do the actual frame
 * work -- never ran once, and the game sat at 60 Hz doing nothing forever. */
void wc_os_preempt_to(uint32_t next_osthread)
{
#ifdef __PS3__
    int to;
    if (!g_lock_ready) return;
    sysMutexLock(g_lock, 0);
    to = find_thread(next_osthread);
    if (to >= 0 && !g_threads[to].finished) {
        extern volatile uint32_t g_wc_sw_log[16];
        extern volatile unsigned g_wc_sw_n;
        g_wc_sw_log[g_wc_sw_n++ & 15u] = next_osthread | 1u;   /* tag: preempt */
        g_threads[to].runnable = 1;
        g_current = to;
        sysCondSignal(g_threads[to].wake);
    } else {
        /* Unregistered target: fall back to re-running the guest scheduler on
         * the game thread by restoring the hint it consumed. */
        MemoryInline::Store<uint32_t>(kRunQueueHint, 1u);
    }
    sysMutexUnlock(g_lock);
#else
    (void)next_osthread;
#endif
}

void wc_os_switch_to(uint32_t next_osthread)
{
    int me = t_my_slot >= 0 ? t_my_slot : g_current, to;
    g_wc_sw_log[g_wc_sw_n++ & 15u] = next_osthread;
#ifdef __PS3__
    if (!g_lock_ready) {
        /* Silently returning here means the guest asked for a thread switch
         * and did not get one, with no record that anything went wrong -- the
         * game simply stops. Say it once; the cause is always the same. */
        static int moaned;
        if (!moaned) { moaned = 1;
            LOG_ERROR(LOG_CORE, "WC: thread switch requested before "
                                "wc_os_init -- threading is inert"); }
        return;
    }
    sysMutexLock(g_lock, 0);
#endif
    if (g_wc_in_irq && t_in_irq) {
        /* The HANDLER's own reschedule: its context loads end the interrupt
         * via the rfi longjmp before reaching here, so this is only a
         * belt-and-braces guard against a handler path that schedules without
         * loading. Nothing to hand off. */
#ifdef __PS3__
        sysMutexUnlock(g_lock);
#endif
        return;
    }
    if (g_wc_in_irq) {
        /* A NON-handler thread switching away while a handler runs elsewhere.
         * The old behaviour returned as if the switch had happened, so a
         * thread that went to SLEEP kept running -- a silent corruption
         * window on every delivery. Wait the handler out (they are
         * microseconds), then switch for real. */
#ifdef __PS3__
        sysMutexUnlock(g_lock);
        wc_where(32);
        while (g_wc_in_irq) sysThreadYield();
        wc_where(33);
        sysMutexLock(g_lock, 0);
        me = t_my_slot >= 0 ? t_my_slot : g_current;
#endif
    }
    to = find_thread(next_osthread);
    if (to < 0) {
        /* Do NOT invent a slot here. An unregistered OSThread has no host
         * thread and no entry point, so signalling it wakes nobody and the
         * caller then waits to be handed a processor that nothing will ever
         * hand back -- a silent deadlock several seconds before the watchdog
         * notices, with nothing in the log to say which thread it was.
         *
         * Every real guest thread is registered: created ones by the
         * OSCreateThread override, the initial one by the __OSThreadInit
         * override. Anything else is OSLoadContext being used for something
         * that is not a thread switch, and the right answer is to stay on the
         * current thread and say so. */
        LOG_ERROR(LOG_CORE, "WC: switch to unregistered OSThread %08x ignored",
                  (unsigned)next_osthread);
#ifdef __PS3__
        sysMutexUnlock(g_lock);
#endif
        return;
    }
    if (to == me) {
#ifdef __PS3__
        sysMutexUnlock(g_lock);
#endif
        return;                      /* already running: nothing to do */
    }
    g_threads[to].runnable = 1;
    g_current = to;
#ifdef __PS3__
    sysCondSignal(g_threads[to].wake);
    if (me < 0) {
        /* A NON-guest caller (the poller delivering into a handler that then
         * switched) cannot park, and its signal is lost when the target's
         * host thread is not parked either -- the common case being the game
         * thread spinning in the guest IDLE loop whose run-queue hint the
         * handler's own SelectThread just consumed. Restore the hint so the
         * spin exits and re-runs the scheduler; harmless when the target was
         * genuinely parked, since SelectThread re-checks the queues anyway. */
        MemoryInline::Store<uint32_t>(kRunQueueHint, 1u);
    }
    if (me >= 0 && g_threads[me].exiting) {
        /* OSExitThread does not return, on the console or here. The hand-off
         * above is the last thing this guest thread does, so the host thread
         * ends inside it rather than waiting to be resumed -- waiting would
         * park it on a condvar nothing will ever signal, and returning would
         * run it alongside its own successor.
         *
         * The C++ stack is abandoned deliberately: the guest thread it
         * belonged to no longer exists. Nothing on it owns a resource (no
         * exceptions, no destructors -- translated frames are plain locals),
         * and the guest stack it used is the game's to reclaim. */
        g_threads[me].finished = 1;
        g_threads[me].used     = 0;
        sysCondDestroy(g_threads[me].wake);
        sysMutexUnlock(g_lock);
        sysThreadExit(0);
    }
    if (me >= 0) {
        /* Wait for someone to hand it back. The predicate is g_current, not a
         * flag of our own: a thread is running exactly when it is current. */
        wc_where(30);
        g_threads[me].parked = 1;
        {   static unsigned n_park;
            if (n_park < 8u) {
                n_park++;
                LOG_WARN(LOG_CORE, "WC: park slot=%d msr=%08x lr=%08x r1=%08x "
                         "r28=%08x in_irq=%d", me,
                         (unsigned)g_threads[me].ctxp->msr,
                         (unsigned)g_threads[me].ctxp->lr,
                         (unsigned)g_threads[me].ctxp->gpr[1],
                         (unsigned)g_threads[me].ctxp->gpr[28], g_wc_in_irq);
            }
        }
        while (g_current != me && !g_threads[me].finished)
            sysCondWait(g_threads[me].wake, 0);
        g_threads[me].parked = 0;
        /* CURRENT-THREAD INVARIANT. On hardware, whoever is executing IS
         * OS_CURRENT_THREAD: SelectThread writes it before every load. Our
         * hand-off resumes a thread without running the waker's SelectThread
         * tail on its behalf, and the guest's idle path zeroes the global --
         * so code resumed from a park could run with current == 0. Harmless
         * until something READS it: EGG::TaskThread::request checks "am I
         * the task thread?" against it, read 0, took the cross-thread path,
         * and the STRAP SCENE (running on the task thread) posted its disc
         * rip to ITS OWN queue and waited forever -- the send ring's
         * cur=00000000 entries were this exact hole. Restore the invariant
         * at every resume. */
        MemoryInline::Store<uint32_t>(0x800000E4u, g_threads[me].osthread);
        MemoryInline::Store<uint32_t>(kCurrentContext, g_threads[me].osthread);
        wc_where(31);
        {   static unsigned n_res;
            if (n_res < 8u) {
                n_res++;
                LOG_WARN(LOG_CORE, "WC: resume slot=%d msr=%08x lr=%08x r1=%08x "
                         "r28=%08x srr1=%08x", me,
                         (unsigned)g_threads[me].ctxp->msr,
                         (unsigned)g_threads[me].ctxp->lr,
                         (unsigned)g_threads[me].ctxp->gpr[1],
                         (unsigned)g_threads[me].ctxp->gpr[28],
                         (unsigned)MemoryInline::Load<uint32_t>(
                             g_threads[me].osthread + 0x19Cu));
            }
        }
        /* We are the running thread again: take back the context pointer the
         * thread we handed off to was using. */
        wc_current_ctx = g_threads[me].ctxp;
    }
    sysMutexUnlock(g_lock);
#endif
}

/* OSCreateThread. The guest has already built its OSThread and stack; this
 * records the pairing and creates the host thread, which parks immediately and
 * runs nothing until the guest's scheduler hands it the processor. Starting it
 * eagerly would run two guest threads at once, which the game is not written
 * for. */
void wc_os_thread_created(uint32_t osthread, uint32_t entry, uint32_t param,
                          uint32_t stack_top)
{
    int i;
#ifdef __PS3__
    if (!g_lock_ready) return;
    sysMutexLock(g_lock, 0);
#endif
    i = alloc_thread(osthread);
    if (i < 0) {
        LOG_ERROR(LOG_CORE, "WC: out of guest thread slots at create");
#ifdef __PS3__
        sysMutexUnlock(g_lock);
#endif
        return;
    }
    g_threads[i].entry = entry;
    g_threads[i].param = param;
    g_threads[i].ctx.gpr[1] = stack_top;
    g_threads[i].ctx.gpr[3] = param;
    g_threads[i].ctx.gpr[2] = 0x8038AC20u;   /* r2/r13 are process-wide */
    g_threads[i].ctx.gpr[13] = 0x80388880u;
    g_threads[i].ctx.msr  = MSR_FP | 0x8000u;
    /* EE ON at birth: OSCreateThread builds the new thread's srr1 with
     * external interrupts enabled -- a thread created deaf (observed: the DVD
     * service thread frozen call-free at msr=0x2000, its completion interrupt
     * pending forever undeliverable) never hears the event it exists to
     * wait for. */
    g_threads[i].ctx.hid2 = HID2_PSE | HID2_LSQE;
#ifdef __PS3__
    /* Priority matches the boot thread's: the hand-off, not lv2, decides who
     * runs, and giving these threads different priorities would let lv2
     * reorder guest execution behind the scheduler's back.
     *
     * The stack is the HOST stack for the translated C++ frames. It is not the
     * guest stack -- that one the game allocated itself and lives in the arena
     * at gpr[1] -- but translated code nests one host frame per guest call, so
     * it has to be deep enough for the game's deepest call chain. */
    if (sysThreadCreate(&g_threads[i].host, wc_guest_thread_main,
                        (void *)(intptr_t)i, 1500, 0x40000, 0,
                        (char *)"mkw-guest") != 0) {
        LOG_ERROR(LOG_CORE, "WC: could not create host thread for guest thread %d", i);
        g_threads[i].used = 0;
        sysMutexUnlock(g_lock);
        return;
    }
#endif
    LOG_INFO(LOG_CORE, "WC: guest thread %d created (os=%08x entry=%08x sp=%08x)",
             i, osthread, entry, stack_top);
#ifdef __PS3__
    sysMutexUnlock(g_lock);
#endif
}

/* The running thread has finished. Mark it and let the scheduler pick another;
 * the guest's own OSExitThread does the bookkeeping the game reads. */
void wc_os_thread_exited(void)
{
#ifdef __PS3__
    if (!g_lock_ready) return;
    sysMutexLock(g_lock, 0);
#endif
    if (g_current >= 0) g_threads[g_current].finished = 1;
#ifdef __PS3__
    sysMutexUnlock(g_lock);
#endif
}

} /* extern "C" */

/* ------------------------------------------------------------------ */
/* HLE overrides: the three points where the guest scheduler touches    */
/* something static recompilation cannot reproduce.                     */
/*                                                                      */
/* Everything else about the guest's threading stays translated and     */
/* stays authoritative -- the ready queues, the priorities, the mutex   */
/* and condvar implementations, the OSThread bookkeeping the game reads.*/
/* Only these three are replaced, because only these three depend on    */
/* the register file being a real register file.                        */
/* ------------------------------------------------------------------ */

/* Offsets into the game's OSThread. Its OSContext is at offset 0, so an
 * OSContext* handed to OSLoadContext IS the OSThread* for a thread context,
 * which is what lets the hand-off find its slot from the argument alone. */
#define OSCTX_GPR(n)   ((n) * 4u)     /* gpr[n]            */
#define OSCTX_SRR0     0x198u         /* resume PC         */
#define OSCTX_SRR1     0x19Cu         /* MSR the rfi restores */

extern "C" {

/* OSLoadContext -- the switch itself.
 *
 * The translator turned this one into a function that loads 32 registers and
 * then RETURNS, because the `rfi` that transfers control has no meaning once
 * the register file is a set of C locals. Left alone it produces the worst
 * possible outcome: the scheduler runs on with another thread's register
 * values and no switch has happened.
 *
 * Natively it is a hand-off. The host thread that owns this context is made
 * runnable and this one blocks, so control really does move -- and it moves to
 * a host thread whose C++ call stack is that guest thread's call stack, which
 * is the whole reason a thread gets a host thread of its own. When this thread
 * is handed the processor back it returns normally, resuming exactly where the
 * guest's own scheduler would have resumed it. */
void wc_hle_OSLoadContext(CpuContext *ctx)
{
#ifdef WC_FIBER_SCHED
    /* IN-DELIVERY RFI, exactly as the old model proved it. A handler
     * delivered by the pump ends in OSLoadContext of the interrupted
     * context; that is the rfi, and it must unwind to the delivery site --
     * NOT be treated as a fiber switch. Without this, the closing load of
     * an exception-scratch context hit the unknown-fiber fallback and
     * replaced the live register mirror with exception-frame garbage
     * (measured: frozen guest, r1=0x71, lr=0x6803a638, stable across
     * samples). */
    if (t_in_irq && t_irq_jmp_armed) {
        uint32_t osctx_rfi = ctx->gpr[3];
        if (osctx_rfi && osctx_rfi == t_irq_ctx) {
            uint32_t s1 = MemoryInline::Load<uint32_t>(osctx_rfi + OSCTX_SRR1);
            func_801A1EB8(ctx);
            if (s1) ctx->msr = s1;
            t_irq_switch_to = 0;
            t_irq_jmp_armed = 0;
            longjmp(t_irq_jmp, 1);
        }
        /* A handler that switches away mid-delivery has architecturally
         * ENDED its interrupt (the old model's hard-won rule). Leaving the
         * delivery state armed parked g_wc_in_irq=1 with the fiber, and
         * every other fiber's delivery failed the CAS forever: the world
         * slept under an asserted VI line with delivered frozen. Clear it
         * all before the switch. */
        {   static unsigned n;
            if (n < 8u) { n++;
                LOG_WARN(LOG_CORE, "WCF: mid-handler switch to %08x ends irq (irqctx=%08x)",
                         (unsigned)osctx_rfi, (unsigned)t_irq_ctx); } }
        t_irq_jmp_armed = 0;
        t_irq_switch_to = 0;
        g_wc_irq_owner  = -2;
        t_in_irq        = 0;
        g_wc_in_irq     = 0;
    }
    {   extern void wcf_hle_OSLoadContext(CpuContext *);
        wcf_hle_OSLoadContext(ctx);
        return;
    }
#endif
    uint32_t osctx = ctx->gpr[3];
    if (!osctx) return;

    /* Inside the interrupt handler every context load ends the interrupt --
     * the rfi. Loading the interrupted context resumes it in place; loading
     * any other is a preemption, recorded and carried out at the delivery
     * site. Either way the code after the load never runs, exactly as on the
     * console. The translated body is NOT run here: it would only fill the
     * handler's scratch context, which the longjmp is about to discard. */
    if (t_in_irq && t_irq_jmp_armed) {
        if (osctx == t_irq_ctx) {
            /* The true rfi. The handler ran on the interrupted thread's own
             * context and clobbered its registers; hardware's rfi restores
             * them from the OSContext -- run the translated load body to do
             * exactly that (it also brings back r3/r4), take MSR from srr1,
             * then unwind. Code after the load never runs. */
            uint32_t s1 = MemoryInline::Load<uint32_t>(osctx + OSCTX_SRR1);
            func_801A1EB8(ctx);
            if (s1) ctx->msr = s1;
            t_irq_switch_to = 0;
            t_irq_jmp_armed = 0;
            longjmp(t_irq_jmp, 1);
        }
        /* A DIFFERENT context mid-handler: not only the dispatcher's closing
         * preemption -- HANDLERS SLEEP (the BTE alarm tick's sync IPC parks in
         * OSSleepThread and resumes exactly there). The old longjmp discarded
         * the handler's live frames; the reply then woke a continuation that
         * no longer existed. On console a handler that switches away has
         * architecturally ended the interrupt. Mirror that: end the irq state
         * HERE, keep every frame, and take the ordinary hand-off below; the
         * chain finishes when this thread is resumed, and its closing load
         * then returns through the delivery site normally. */
        if (t_my_slot < 0 ||
            (g_deliver_slot >= 0 && t_my_slot == g_deliver_slot)) {
            /* NON-GUEST (poller/delivery) handler at a cross-load: this is its closing
             * reschedule -- there is no sleepable continuation to preserve,
             * and letting it RETURN was the final wake-eater: the dispatcher's
             * dead-code tail ran on the poller and its own reschedule
             * re-consumed the run-queue hint microseconds after we restored
             * it, before the spinning main thread could ever see it (widened
             * delivery log: hint=0 at landing on every wake). Schedule the
             * target, then END the delivery so the tail never executes. */
            {   static unsigned n_ts;
                if (n_ts < 12u) { n_ts++;
                    LOG_WARN(LOG_CORE, "WC: tail-skip slot=%d -> osctx=%08x",
                             t_my_slot, (unsigned)osctx); } }
            wc_os_switch_to(osctx);   /* me<0: signal + hint restore */
            t_irq_jmp_armed = 0;
            t_irq_switch_to = 0;
            longjmp(t_irq_jmp, 1);
        }
        {   static unsigned n_hs;
            if (n_hs < 12u) { n_hs++;
                LOG_WARN(LOG_CORE, "WC: handler-sleep slot=%d osctx=%08x",
                         t_my_slot, (unsigned)osctx); } }
        t_irq_slept = 1;    /* the closing rfi arrives later: defer it */
        t_irq_jmp_armed = 0;
        t_irq_switch_to = 0;
        g_wc_irq_owner  = -2;
        t_in_irq        = 0;
        g_wc_in_irq     = 0;
    }

    /* SELF-RESUME: the scheduler picked the thread that is already running
     * (slept, was woken before anything else ran). The caller's code after
     * this call reloads every register from ctx, expecting the load to have
     * restored them from the OSContext -- OSSaveContext saved r1 and stored 1
     * in the saved r3, so the restore is what makes the sleep "return 1" with
     * an intact stack. Skipping the translated body here handed the caller
     * STALE registers, including a stale stack pointer, and the thread resumed
     * into garbage: the boot froze with the scheduler state perfectly
     * consistent and calls at a standstill.
     *
     * The rfi's MSR restore comes from the context's srr1, as before. */
#ifdef __PS3__
    {   int me = t_my_slot >= 0 ? t_my_slot : g_current;
        uint32_t mine = (me >= 0) ? g_threads[me].osthread : 0u;
        uint32_t srr1;
        if (t_irq_slept && !t_in_irq && mine && osctx == mine) {
            /* DEFERRED RFI. This thread's handler slept mid-delivery; the
             * delivery frame (and its setjmp) still live below us on this
             * very stack. Without this, the closing load took the plain
             * self-resume path and RETURNED -- into the dispatcher's
             * dead-code tail that hardware never executes past rfi; the
             * thread wedged call-free at RKSystem::initialize+0x364 with
             * both EGGCREATE pairs closed and the third dispatch's crumb
             * already recorded. Restore from the context as the rfi would,
             * then unwind to the delivery site. */
            uint32_t s1d = MemoryInline::Load<uint32_t>(osctx + OSCTX_SRR1);
            t_irq_slept = 0;
            func_801A1EB8(ctx);
            if (s1d) ctx->msr = s1d;
            wc_where(35);
            longjmp(t_irq_jmp, 1);
        }
        if (osctx == kDeliverThread) {
            /* The delivery pseudo-thread got woken (a slept handler's wake
             * re-queued it) and the scheduler picked it. Its handler frames
             * were abandoned at the tail-skip -- there is nothing to resume.
             * Force a re-selection instead of switching to a ghost. */
            MemoryInline::Store<uint32_t>(kRunQueueHint, 1u);
            return;
        }
        if (osctx == kIdleContext && me >= 0 && me != g_deliver_slot) {
            /* OSLoadContext(idle context): the guest scheduler found nothing
             * runnable. In the single-runner model nobody runs the idle spin
             * for real -- the caller parks until a wakeup schedules someone
             * (the delivery thread's idle gate fires on curctx==idle and the
             * handler's OSWakeupThread switches to the woken thread, which
             * may be us). Ignoring this load (the old unregistered-OSThread
             * path) made OSSleepThread return without sleeping: half a
             * billion calls of a sleep loop that never slept. */
            {   static unsigned n_ip;
                if (n_ip < 12u) { n_ip++;
                    LOG_WARN(LOG_CORE, "WC: idle-park slot=%d gcur=%d", me,
                             g_current); } }
            sysMutexLock(g_lock, 0);
            g_wc_sw_log[g_wc_sw_n++ & 15u] = kIdleContext;
            if (g_current == me) g_current = -1;
            g_threads[me].parked = 1;
            while (g_current != me && !g_threads[me].finished)
                sysCondWait(g_threads[me].wake, 0);
            g_threads[me].parked = 0;
            wc_current_ctx = g_threads[me].ctxp;
            MemoryInline::Store<uint32_t>(0x800000E4u, mine);
            MemoryInline::Store<uint32_t>(kCurrentContext, mine);
            sysMutexUnlock(g_lock);
            srr1 = MemoryInline::Load<uint32_t>(mine + OSCTX_SRR1);
            if (srr1) ctx->msr = srr1;
            ctx->gpr[3] = mine;
            func_801A1EB8(ctx);
            return;
        }
        if (mine && osctx == mine) {
            /* genuine self-resume */
            srr1 = MemoryInline::Load<uint32_t>(osctx + OSCTX_SRR1);
            if (srr1) ctx->msr = srr1;
            func_801A1EB8(ctx);           /* restore registers from the context */
            return;
        }
        if (!mine) {
            /* NON-guest caller (the poller, mid-handler): it cannot be
             * resumed as a guest thread, but the load's TARGET must still be
             * scheduled -- the old code fell into self-restore here and
             * silently returned, so the woken thread never ran and the idle
             * spin waited forever on a hint nobody set. switch_to with a
             * negative caller signals the target and restores the run-queue
             * hint, then returns immediately. */
            wc_os_switch_to(osctx);
            srr1 = MemoryInline::Load<uint32_t>(osctx + OSCTX_SRR1);
            if (srr1) ctx->msr = srr1;
            func_801A1EB8(ctx);
            return;
        }

        /* CROSS-THREAD: hand the processor over and block. Nothing of the
         * TARGET's is restored into OUR ctx -- the target runs on its own
         * context; writing its registers into ours before blocking corrupted
         * the very state we resume with. */
        wc_os_switch_to(osctx);

        /* Handed back: we are the running thread again. Restore OUR OWN saved
         * context -- the caller's code after this load reloads from ctx and
         * must see the state our OSSaveContext saved, not the state we
         * happened to have when we gave the processor away. */
        mine = (me >= 0) ? g_threads[me].osthread
             : (g_current >= 0 ? g_threads[g_current].osthread : 0u);
        if (mine) {
            srr1 = MemoryInline::Load<uint32_t>(mine + OSCTX_SRR1);
            if (srr1) ctx->msr = srr1;
            ctx->gpr[3] = mine;
            func_801A1EB8(ctx);
        }
    }
#else
    func_801A1EB8(ctx);
#endif
}

/* OSCreateThread -- register the pairing, then let the system software do its own work.
 *
 * The translated body is called first and stays in charge of everything the
 * game can observe: the OSThread fields, the ready queue, the priority, the
 * stack guard words. Only after it has run is the thread registered here, and
 * the entry point and stack pointer are read back out of the OSContext the system software
 * itself just filled in rather than reinterpreted from the argument list --
 * so a wrong guess about the calling convention cannot silently produce a
 * thread that starts at the wrong address. */
void wc_hle_OSCreateThread(CpuContext *ctx)
{
    /* r3 AT ENTRY is the OSThread the caller allocated. The system software's return
     * value is a BOOL -- reading r3 after the call registered "thread 1"
     * twice and left every real thread unregistered, so the scheduler's
     * switches to them were refused as unknown. */
    uint32_t thread = ctx->gpr[3];
    func_801A9DE4(ctx);                       /* the game's own OSCreateThread */
    if (!thread || !ctx->gpr[3]) return;      /* creation failed */
#ifdef WC_FIBER_SCHED
    {   extern int wcf_create(uint32_t);
        wcf_create(thread);
        return;
    }
#endif
    wc_os_thread_created(thread,
                         MemoryInline::Load<uint32_t>(thread + OSCTX_SRR0),
                         MemoryInline::Load<uint32_t>(thread + OSCTX_GPR(3)),
                         MemoryInline::Load<uint32_t>(thread + OSCTX_GPR(1)));
}

/* OSExitThread -- mark the slot dead before the guest reschedules away.
 *
 * The translated body ends in a reschedule that never returns, so this host
 * thread will block inside the hand-off and stay blocked. Marking it finished
 * first means the hand-off's wait predicate can release it instead of leaving
 * a host thread parked on a condvar nobody will ever signal. */
void wc_hle_OSExitThread(CpuContext *ctx)
{
    /* Flagged, not finished: the translated body still has to run, because it
     * is what removes the thread from the ready queue, wakes anyone joining on
     * it and runs its detach bookkeeping -- all of it state the game reads.
     * It ends in a reschedule, and the flag tells that hand-off this thread is
     * leaving for good. Marking it finished here instead would let it fall
     * straight through the hand-off's wait and keep running beside the thread
     * it just handed the processor to. */
#ifdef __PS3__
    if (g_lock_ready) {
        sysMutexLock(g_lock, 0);
        if (g_current >= 0) g_threads[g_current].exiting = 1;
        sysMutexUnlock(g_lock);
    }
#endif
    func_801AA050(ctx);                       /* the game's own OSExitThread */
}

} /* extern "C" */

/* __OSThreadInit -- adopt the thread the system software is already running on.
 *
 * The initial thread is not created by OSCreateThread; the system software builds it in
 * place during OSInit and starts running on it. It therefore never passes
 * through the registration above, and the first time the game schedules away
 * from it and back the hand-off would find no slot for it.
 *
 * It needs no host thread -- it already has one, the thread wc_boot started,
 * which is the one executing this. So the slot is adopted rather than created:
 * marked current and given a condvar so the processor can be handed back. */
extern "C" { volatile unsigned g_wc_dispatch_total; }

/* DELIVERY-THREAD SELECTTHREAD ESCAPE. SelectThread's no-runnable path does
 * not load a context at all: after the switch callback and
 * OSSetCurrentContext(idle) it SPINS INLINE on the run-queue hint -- that is
 * the hardware idle loop, designed to run EE-on taking interrupts. A
 * delivery-thread handler that reaches it (dispatcher exit with nothing
 * runnable) spins forever with in_irq held and every boundary blocked: the
 * whole world freezes call-free (observed: delivery slot at SelectThread's
 * callback site, disp frozen, zero loads, zero instrumentation firings).
 * The handler's architectural work -- device ack, wakes -- is complete by
 * the time it reschedules; rescheduling from the delivery thread is
 * meaningless. So: dispatching SelectThread inside a delivery-thread
 * delivery sets the hint (real threads rescan at their own boundaries) and
 * unwinds to the delivery site. Lives here, not in the 13,675 stale
 * translated objects: every dispatch flows through the freshly built
 * wc_calls.o templates. */
extern "C" { volatile unsigned g_wc_vi_handler_n, g_wc_vi_wait_n; }
extern "C" { volatile unsigned g_wc_dvd_open_n, g_wc_dvd_ra_n,
                               g_wc_dvd_rp_n, g_wc_dvd_thread_n; }
extern "C" { volatile unsigned g_wc_gx_peinit_n, g_wc_gx_sync_n, g_wc_gx_cb_n; }
extern "C" { volatile unsigned g_wc_gki_send_n, g_wc_gki_read_n; }

extern "C" void wc_dispatch_guard(uint32_t a)
{
#ifdef __PS3__
    /* The VI wake chain question, answered by counting: does the guest's
     * retrace handler ever dispatch, and how often does something enter
     * VIWaitForRetrace? RKSystem::initialize waits on a retrace before its
     * third heap create; every recent boot idles exactly there. */
    if (a == 0x801B8844u) g_wc_vi_handler_n++;
    else if (a == 0x801B994Cu) g_wc_vi_wait_n++;
    else if (a == 0x8016ED74u) g_wc_gx_peinit_n++;  /* __GXPEInit        */
    else if (a == 0x8016E95Cu) g_wc_gx_sync_n++;    /* GXSetDrawSync     */
    else if (a == 0x802389BCu) g_wc_gx_cb_n++;      /* callbackDrawSync  */
    else if (a == 0x8015E21Cu) g_wc_dvd_open_n++;   /* DVDOpen           */
    else if (a == 0x8015E6ACu) g_wc_dvd_ra_n++;     /* DVDReadAsyncPrio  */
    else if (a == 0x8015E794u) g_wc_dvd_rp_n++;     /* DVDReadPrio       */
    else if (a == 0x80008CD8u) g_wc_dvd_thread_n++; /* DvdThread_main    */
    else if (a == 0x8012EF50u) g_wc_gki_send_n++;   /* GKI_send_msg      */
    else if (a == 0x8012F10Cu) g_wc_gki_read_n++;   /* GKI_read_mbox     */
    if (a == 0x801A9B68u && t_in_irq && t_irq_jmp_armed &&
        g_deliver_slot >= 0 && t_my_slot == g_deliver_slot &&
        wc_all_guest_parked()) {
        /* ONLY when nothing is runnable: that is the sole case where
         * SelectThread's inline idle spin wedges. Firing on every
         * SelectThread dispatch abandoned the dispatcher's TAIL -- the
         * interrupt-ack bookkeeping -- and a DI transaction died mid-ack:
         * deterministic __DVDShowFatalMessage/OSFatal at calls~33k, with
         * the guard's sticky hint=1 as the fingerprint. With guest threads
         * runnable, SelectThread completes and its load takes the normal
         * tail-skip path. */
        static unsigned n_gd;
        if (n_gd < 8u) { n_gd++;
            LOG_WARN(LOG_CORE, "WC: guard escape (all parked)"); }
        MemoryInline::Store<uint32_t>(kRunQueueHint, 1u);
        t_irq_jmp_armed = 0;
        t_irq_switch_to = 0;
        wc_where(36);
        longjmp(t_irq_jmp, 1);
    }
#else
    (void)a;
#endif
}

extern "C" void wc_hle_OSThreadInit(CpuContext *ctx)
{
    uint32_t cur;
    func_801A957C(ctx);                       /* the game's own __OSThreadInit */
    /* The adopted boot thread's OSContext has srr1 == 0 (nothing ever wrote
     * it), so every OSLoadContext resume handed it MSR = 0 -- FP and EE both
     * off -- until the guest's own OSRestoreInterrupts. Threads made by
     * OSCreateThread get srr1 from OSInitContext; give the adopted one the
     * same treatment: EE|FP, the state the boot thread actually runs in. */
    {   uint32_t cur = MemoryInline::Load<uint32_t>(0x800000E4u);
        if (cur && !MemoryInline::Load<uint32_t>(cur + OSCTX_SRR1))
            MemoryInline::Store<uint32_t>(cur + OSCTX_SRR1, 0x0000A000u);
    }

    cur = MemoryInline::Load<uint32_t>(0x800000E4u);   /* __OSCurrentThread */
    if (!cur) {
        LOG_ERROR(LOG_CORE, "WC: __OSThreadInit left no current thread");
        return;
    }
#ifdef WC_FIBER_SCHED
    {   extern int wcf_register_root(uint32_t, CpuContext *);
        wcf_register_root(cur, ctx);
        return;
    }
#endif
#ifdef __PS3__
    if (!g_lock_ready) return;
    sysMutexLock(g_lock, 0);
#endif
    if (find_thread(cur) < 0) {
        int i = alloc_thread(cur);
        if (i < 0) {
            LOG_ERROR(LOG_CORE, "WC: no slot for the initial guest thread");
        } else {
            g_threads[i].runnable = 1;
            g_threads[i].ctxp = ctx;          /* the context we are running on */
            g_current = i;
            t_my_slot = i;
            LOG_INFO(LOG_CORE, "WC: adopted initial guest thread %d (os=%08x)",
                     i, (unsigned)cur);
        }
    }
#ifdef __PS3__
    sysMutexUnlock(g_lock);
#endif
}

/* ------------------------------------------------------------------ */
/* INTERRUPT DELIVERY                                                   */
/*                                                                      */
/* The guest's OS blocks by sleeping a thread and, when nothing else is */
/* runnable, spinning in its idle loop with interrupts enabled until a  */
/* device interrupt makes something runnable again. That is not a       */
/* corner case -- it is how every IOS call returns. The first one the   */
/* game makes is __OSInitSTM opening /dev/stm, and the port sat in that */
/* spin forever because nothing ever delivered the IPC completion.      */
/*                                                                      */
/* WHERE IT IS SAFE TO DELIVER                                          */
/*                                                                      */
/* Running guest code on a second host thread races with the game       */
/* thread in general. In the idle loop it does not: that loop reads one */
/* word and writes MSR, touching nothing the handler touches. It is     */
/* also exactly where the hardware would have taken the interrupt. So   */
/* delivery is gated on the guest actually running its idle context,    */
/* which the system software records in __OSCurrentContext when it gives up.        */
/*                                                                      */
/* The handler brackets itself in OSDisableScheduler/OSEnableScheduler, */
/* so the OSWakeupThread inside it sets the run-queue hint but does not */
/* switch threads. The idle loop sees the hint, leaves the spin, and    */
/* the guest's own scheduler performs the switch on the game thread --  */
/* where the woken thread's C++ stack actually is.                      */
/* ------------------------------------------------------------------ */

extern "C" int pi_interrupt_pending(void);
extern "C" uint32_t pi_intsr_raw(void);
extern "C" uint32_t pi_intmr_raw(void);
extern "C" void dev_lock(void);
extern "C" void dev_unlock(void);
extern "C" unsigned ipc_guest_activity(void);
extern "C" uint32_t pi_raise_seq(void);

namespace {


/* OS_EXCEPTION_EXTERNAL_INTERRUPT, as the system software numbers its exceptions. */
const uint32_t kExcExternal = 4u;

} /* namespace */

/* Raised by the poller when the device model has an interrupt for the guest,
 * cleared once a handler has actually run. Read at every guest call boundary,
 * so it lives next to the call counter and shares its cache line. */
/* The call breadcrumb the generated dispatch layer writes: the call counter
 * doubles as the ring index and as the watchdog's progress signal. Defined
 * here because every build that links the dispatch layer links this file. */
volatile unsigned g_wc_calls;
uint32_t          g_wc_crumb[WC_CRUMB_N];

volatile int g_wc_irq_pending;

/* Delivery accounting, read by the rescue "trail" command. Without it a port
 * that is not receiving interrupts is indistinguishable from one whose
 * handler runs and does nothing. */
volatile unsigned g_wc_irq_raised, g_wc_irq_delivered;
volatile unsigned g_wc_dec_delivered;
/* Who is inside the handler right now, for the sched dump: the delivering
 * slot (-1 = poller), the exception number, and the masked cause at entry.
 * Set at delivery entry, cleared at exit; if in_irq reads 1 while these are
 * stale, the flag leaked rather than the handler wedged. */
volatile int      g_wc_irq_owner = -2;
/* Host-side breadcrumbs: a tiny ring of numbered waypoints through the PORT's
 * own code, because a wedge after the guest's last call is invisible to guest
 * crumbs. Read via the sched dump. */
volatile uint16_t g_wc_where[16];
volatile unsigned g_wc_where_n;
extern "C" void wc_where(unsigned id) { g_wc_where[g_wc_where_n++ & 15u] = (uint16_t)id; }
volatile uint32_t g_wc_irq_excnum, g_wc_irq_cause;

/* Run the guest's interrupt handler on the CURRENT thread.
 *
 * Called from a guest call boundary (wc_irq_poll) or, when the guest has gone
 * idle and stopped making calls, from the poller. One at a time: the
 * compare-and-swap is what stops the poller and the game thread from entering
 * the handler together, and it doubles as the re-entry guard, since the
 * handler itself makes guest calls that pass through the same hook. */

void wc_irq_wait(void)
{
#ifdef WC_FIBER_SCHED
    /* One host thread runs every fiber: waiting here waits on ourselves
     * (the delivery owner is parked up this same stack). Refuse. */
    {   extern volatile int g_host_site;
        static unsigned n;
        g_host_site = 2;
        if (n < 6u) { n++;
            LOG_WARN(LOG_CORE, "WC: irq_wait on fiber build (owner=%d) -- returning",
                     (int)g_wc_irq_owner); }
        g_host_site = 0;
        return;
    }
#endif
#ifdef __PS3__
    if (t_in_irq) return;             /* we ARE the handler */
    if (!g_wc_in_irq) return;
    wc_where(20);
    while (g_wc_in_irq) sysThreadYield();
    wc_where(21);
#endif
}

void wc_irq_deliver_from(CpuContext *ctx, int on_guest_thread);
void wc_irq_deliver_exc(CpuContext *ctx, int on_guest_thread,
                        uint32_t excnum, uint32_t entry);
extern "C" int wc_dec_due_and_deliver(CpuContext *rc, int on_guest_thread);
extern "C" int wc_dec_due(void);
extern "C" int wc_dec_overdue(void);

void wc_irq_deliver_exc(CpuContext *ctx, int on_guest_thread,
                        uint32_t excnum, uint32_t entry)
{
#ifdef __PS3__
    uint32_t saved_msr;
    uint32_t saved_gpr[13];          /* r0, r3..r12: the volatile GPR set */
    uint32_t saved_lr, saved_ctr, saved_cr, saved_xer;
    PPC_FPR  saved_fpr[14];          /* f0..f13 + fpscr: handlers run FP-on */
    uint32_t saved_fpscr;
    int k_sv;
    if (!__sync_bool_compare_and_swap(&g_wc_in_irq, 0, 1)) {
        wc_irq_wait();
        return;
    }
    t_in_irq = 1;
    t_irq_on_guest = on_guest_thread;
    t_irq_switch_to = 0;
    wc_where(1);
    g_wc_irq_owner  = on_guest_thread ? t_my_slot : -1;
    g_wc_irq_excnum = excnum;
    g_wc_irq_cause  = pi_intsr_raw() & pi_intmr_raw();

    if (excnum == kExcExternal && !pi_interrupt_pending()) {
        g_wc_irq_pending = 0;
        g_wc_irq_owner = -2;
        t_in_irq = 0;
        g_wc_in_irq = 0;
        return;
    }
    if (!ctx) {                       /* nothing to run a handler on */
        g_wc_irq_owner = -2;
        t_in_irq = 0;
        g_wc_in_irq = 0;
        return;
    }

    /* ONE REGISTER FILE, AS ON HARDWARE.
     *
     * The handler used to run on a scratch CpuContext seeded from the live
     * one. That broke the moment a handler SLEPT: OSSaveContext saved the
     * scratch identity, the resume restored a different thread's state into
     * the scratch, and the corruption eventually surfaced as the main thread
     * spinning with MSR == 0 -- every delivery gate correctly closed against
     * a machine state hardware could never reach that way.
     *
     * The handler now runs on the interrupted thread's OWN context. Its
     * prologue saves to the OSContext, a mid-handler sleep saves/restores the
     * REAL identity, and the closing rfi restores the registers from the
     * OSContext via the translated load body before the longjmp -- so the
     * interrupted code continues with exactly the registers it had. */
    t_irq_ctx = MemoryInline::Load<uint32_t>(kCurrentContext);
    if (!t_irq_ctx) t_irq_ctx = kIdleContext;
    /* v3 aimed t_irq_ctx at the idle context to stop the prologue-save
     * contamination -- wrong layer: the prologue is GUEST code that reads
     * kCurrentContext itself, so the redirect only broke the closing-rfi
     * match and every delivery was tail-skip-abandoned mid-handler (STM's
     * state machine panicked on its very first events). The contamination
     * is closed below with a context snapshot instead. */
    MemoryInline::Store<uint32_t>(t_irq_ctx + OSCTX_SRR1, ctx->msr);
    /* FULL VOLATILE SAVE. The hardware vector stub preserves r3-r5 in the
     * OSContext and the dispatch path preserves the remaining volatiles; the
     * translated handler's own save/restore covers them only on the flavors
     * that run the complete rfi body restore. The flavor that does not lost
     * exactly one register in the wild: sinit_ef_effectsystem's list-push
     * arrived with node (r5) == 0 while r3/r4 -- the two this code already
     * side-saved -- were intact, and the null-node push wrote head over
     * 0x80000000 (the boot-killer stomp, calls~1026-1266). Handlers also run
     * with MSR_FP on and no lazy-FP trap exists here, so the volatile FPRs
     * get the same treatment. */
    saved_msr = ctx->msr;
    saved_gpr[0] = ctx->gpr[0];
    for (k_sv = 0; k_sv < 10; k_sv++) saved_gpr[1 + k_sv] = ctx->gpr[3 + k_sv];
    saved_lr  = ctx->lr;  saved_ctr = ctx->ctr;
    saved_cr  = ctx->cr;  saved_xer = ctx->xer;
    for (k_sv = 0; k_sv < 14; k_sv++) saved_fpr[k_sv] = ctx->fpr[k_sv];
    saved_fpscr = ctx->fpscr;
    ctx->gpr[3] = excnum;
    ctx->gpr[4] = t_irq_ctx;
    ctx->msr    = MSR_FP;

    g_wc_irq_delivered++;
    /* OUT-OF-BAND CONTAMINATION GUARD. The handler's prologue saves the
     * RUNNING register file into *(kCurrentContext). For call-boundary
     * deliveries that file IS the interrupted thread's -- hardware-exact.
     * For delivery-thread deliveries it is the delivery context: a handler
     * that completes restores it via its closing rfi (net zero), but an
     * ABANDONED handler (tail-skip on a mid-handler switch) leaves the
     * delivery stack pointer et al. saved in a REAL context, and a later
     * cross-resume feeds it into a real thread -- the 93640000 family.
     * Snapshot the context before, restore it at landing, unconditionally:
     * after a completed rfi the area is architecturally dead anyway. */
    {   enum { kSnapWords = 0x2C8 / 4 };
        static __thread uint32_t s_osnap[kSnapWords];
        int do_snap = (g_deliver_slot >= 0 && t_my_slot == g_deliver_slot);
        int i_sn;
        if (do_snap)
            for (i_sn = 0; i_sn < kSnapWords; i_sn++)
                s_osnap[i_sn] = MemoryInline::Load<uint32_t>(
                    t_irq_ctx + (uint32_t)i_sn * 4u);
    /* Record what the handler actually does, for the first few deliveries.
         * The main crumb ring is useless here: the handler runs a few dozen
         * calls against billions from the game, so sampling the ring will
         * essentially never land inside one. */
        unsigned before = g_wc_calls, after, n, k;
        jmp_buf prev_jmp;
        int had_prev = t_irq_slept;
        if (had_prev)
            std::memcpy(&prev_jmp, &t_irq_jmp, sizeof prev_jmp);
        t_irq_jmp_armed = 1;
        if (setjmp(t_irq_jmp) == 0) {
            /* (ring rewound below: handler calls are bookkeeping, and at 60 Hz
             * they bury the trail of what the THREADS were doing -- which is
             * the only question a stall ever asks.) */
            /* External enters the vector's own handler; anything else enters
             * the handler the guest installed in its exception table, which
             * is how the 0x900 stub dispatches the decrementer. */
            if (excnum == kExcExternal) { wc_where(2); func_801A6C40(ctx); }
            else                        { wc_where(3); InvokeIndirectCpu(entry, ctx); }
            wc_where(4);              /* handler returned without rfi */
        }
        /* Reached either by the longjmp that stands in for the handler's
         * closing rfi (the normal path), or by the handler returning, which
         * the console's never does. */
        t_irq_jmp_armed = 0;
        wc_where(5);                  /* landed (rfi or return) */
        /* The hardware vector saves the ORIGINAL r3/r4 into the exception
         * context before loading the exception arguments; this delivery
         * overwrote them first, so the handler's prologue saved excnum/ctx
         * into the OSContext and the rfi restore handed the interrupted code
         * r3 == exception number. Measured as a 238-call freeze: the first
         * dec tick at a call boundary poisoned the caller's argument and the
         * boot never even unmasked IPC. Restore the pre-delivery values the
         * stub would have preserved. */
        ctx->gpr[0] = saved_gpr[0];
        for (k_sv = 0; k_sv < 10; k_sv++) ctx->gpr[3 + k_sv] = saved_gpr[1 + k_sv];
        ctx->lr  = saved_lr;  ctx->ctr = saved_ctr;
        ctx->cr  = saved_cr;  ctx->xer = saved_xer;
        for (k_sv = 0; k_sv < 14; k_sv++) ctx->fpr[k_sv] = saved_fpr[k_sv];
        ctx->fpscr = saved_fpscr;
        ctx->msr    = saved_msr;
        after = g_wc_calls;
        /* Rewind the crumb ring past the handler's own calls. The counter is
         * the watchdog's progress signal too -- rewinding makes handler-only
         * periods read as "no progress", which is exactly right: a game whose
         * only activity is its retrace handler IS stalled. */
        if (after - before < WC_CRUMB_N) g_wc_calls = before;
        if (do_snap)
            for (i_sn = 0; i_sn < kSnapWords; i_sn++)
                MemoryInline::Store<uint32_t>(
                    t_irq_ctx + (uint32_t)i_sn * 4u, s_osnap[i_sn]);
        if (had_prev)
            std::memcpy(&t_irq_jmp, &prev_jmp, sizeof prev_jmp);
        if (g_wc_irq_delivered <= 10u) {
            n = after - before;
            LOG_WARN(LOG_CORE, "WC: irq #%u exc=%u dispatched %u call(s), "
                               "intsr now %08x switch_to=%08x hint=%08x",
                     g_wc_irq_delivered, excnum, n,
                     (unsigned)pi_intsr_raw(), (unsigned)t_irq_switch_to,
                     (unsigned)MemoryInline::Load<uint32_t>(kRunQueueHint));
            if (n > WC_CRUMB_N) n = WC_CRUMB_N;
            {   char line[160]; int used = 0;
                for (k = 0; k < n; k++) {
                    used += snprintf(line + used, sizeof line - (size_t)used,
                                     " %08x",
                                     (unsigned)g_wc_crumb[(after - n + k) & (WC_CRUMB_N - 1u)]);
                    if ((k % 8) == 7 || k == n - 1) {
                        LOG_WARN(LOG_CORE, "WC:  irq%s", line);
                        used = 0; line[0] = 0;
                    }
                }
            }
        }
    }

    /* Re-arm from the line, as the hardware would: it stays asserted until the
     * guest's handler clears it, and while it is asserted the console keeps
     * taking the interrupt.
     *
     * Earlier revisions delivered less often -- on the rising edge of the PI
     * cause, then once per logical device event -- because the guest's IPC
     * queue counters ran ahead of the requests it had actually enqueued. Each
     * of those fixed one symptom and broke another. The cause was not the
     * delivery policy: the port was entering the handler one level too deep,
     * at __OSDispatchInterrupt rather than at ExternalInterruptHandler, so the
     * exception-context bookkeeping the system software does on the way in never ran. */
    g_wc_irq_pending = pi_interrupt_pending() ? 1 : 0;
    g_wc_irq_owner = -2;
    t_in_irq = 0;
    g_wc_in_irq = 0;

    /* Carry out the preemption the handler asked for.
     *
     * Only when delivery ran on the game thread: the hand-off blocks the
     * calling thread until the processor comes back, which is exactly right
     * for the preempted guest thread and exactly wrong for the poller. In the
     * poller's case the guest is in its idle loop, the wakeup has already set
     * the run-queue hint, and the idle loop's own SelectThread will perform
     * this same switch on the game thread the moment it sees the hint. */
    if (t_irq_switch_to && on_guest_thread && t_my_slot != g_deliver_slot) {
        uint32_t to = t_irq_switch_to;
        t_irq_switch_to = 0;
        wc_os_switch_to(to);
    } else if (t_irq_switch_to) {
        /* Poller-era path, now also the delivery thread's: signal the target
         * without parking -- nothing in the guest run queues would ever
         * schedule the delivery thread back for a plain preemption. */
        uint32_t pto = t_irq_switch_to;
        t_irq_switch_to = 0;
        wc_os_preempt_to(pto);
    } else if (0) {
        /* Poller-delivered, and the handler picked a thread to run. The switch
         * cannot be made from here -- the poller is not a guest thread -- but
         * it cannot be dropped either: the handler's own SelectThread has
         * ALREADY CONSUMED the run-queue hint on its way to choosing this
         * thread, so the idle spin the game thread sits in is now waiting on a
         * hint that has been cleared. That was the deadlock: DVD reply
         * delivered, thread woken, hint consumed, switch dropped, spin
         * eternal.
         *
         * Restore the hint. The idle loop sees it, leaves the spin, and the
         * game thread's own SelectThread re-picks this same thread with the
         * guest's full scheduling logic -- the switch happens where it must,
         * on the game thread. */
        t_irq_switch_to = 0;
        MemoryInline::Store<uint32_t>(kRunQueueHint, 1u);
    }
#else
    (void)ctx; (void)on_guest_thread;
#endif
}

void wc_irq_deliver_from(CpuContext *ctx, int on_guest_thread)
{
    wc_irq_deliver_exc(ctx, on_guest_thread, kExcExternal, 0x801A6C40u);
}

void wc_irq_deliver(CpuContext *ctx)
{
    /* The call-boundary hook's chooser: hardware priority is external first,
     * then the decrementer -- but NOT either/or. On hardware the external
     * handler's rfi restores EE and a due decrementer traps immediately; a
     * strict else-branch here starved the decrementer whenever a device line
     * was busy. Measured: at the render phase the VI/BT traffic kept the PI
     * line high enough that dec delivered froze at 148621 with a dec due
     * (armed=0) -- alarms dead, KPAD/WPAD timing dead, VIWaitForRetrace
     * waiting on a chain that needs alarms. Deliver both. */
    /* ONE exception per call boundary (a dec delivered in the same boundary
     * as an ext collided with the ext handler's switch carry-out: the alarm
     * handler's re-arm never ran, mtdec stopped at #2, GKI/BT timers died and
     * BTA_Init crawled). Fairness instead comes from aging: a dec overdue by
     * more than ~1 ms delivers FIRST, so a busy device line can no longer
     * starve the alarm clock (dec delivered froze at 148621 with a due dec
     * while VI/BT traffic held the PI line high). */
    if (wc_dec_overdue() && wc_dec_due_and_deliver(ctx, 1)) return;
    if (pi_interrupt_pending()) { wc_irq_deliver_from(ctx, 1); return; }
    if (wc_dec_due_and_deliver(ctx, 1)) return;
    g_wc_irq_pending = 0;
}

#ifdef __PS3__
/* Poller-side deliveries run on THIS context, never on a guest thread's.
 *
 * The one-register-file model is right when the interrupted thread is parked
 * at a call boundary -- it is the hardware picture. It is WRONG for the
 * poller: the "interrupted" idle thread is not parked at all, it is actively
 * spinning through the very CpuContext the handler would be clobbering from
 * another host thread (the spin reads and writes ctx->msr). Measured as the
 * wake advancing exactly one call per IOS round trip while the resumed state
 * kept dissolving underneath. The poller's handlers get a private context on
 * a private guest stack; a poller-side handler that sleeps remains the
 * recorded residual gap (none observed -- idle-time handlers wake and exit
 * via the tail-skip). */
static CpuContext s_poller_ctx;

static CpuContext *wc_poller_ctx(void)
{
    if (!s_poller_ctx.gpr[1]) {
        std::memset(&s_poller_ctx, 0, sizeof s_poller_ctx);
        /* NOT 0x817F0000: that sat in the ~3 KB gap between the game's MEM1
         * arena top (0x817ef3e0) and the FST -- deep handler chains grew the
         * poller's guest stack straight down into the top of the game heap,
         * and the g3d ResFile panic at 68 frames has "corrupted allocation
         * near arena top" written all over it. 0x93640000 sits in MEM2 above
         * both the game's MEM2 arena end (0x935E0000) and the IPC buffer
         * range (..0x93600000), with 256 KB of clear headroom. */
        s_poller_ctx.gpr[1]  = 0x93640000u;
        s_poller_ctx.gpr[2]  = 0x8038AC20u;
        s_poller_ctx.gpr[13] = 0x80388880u;
        s_poller_ctx.msr     = MSR_FP;
        s_poller_ctx.hid2    = HID2_PSE | HID2_LSQE;
    }
    return &s_poller_ctx;
}

static void wc_irq_thread(void *arg)
{
#ifdef WC_FIBER_SCHED
    return;               /* fiber build: delivery happens on the one guest thread */
#endif
    (void)arg;
    for (;;) {
        {   /* Watch MSR[EE] on the running guest thread.
             *
             * Delivery is gated on it, and it went to zero at some point the
             * log does not show, after which no interrupt could ever be
             * delivered again. Log every change with the guest call that was
             * in flight, so the transition can be attributed to a function
             * rather than guessed at. */
            static uint32_t last_msr = 0xFFFFFFFFu;
            static unsigned logged;
            uint32_t m = (g_current >= 0 && g_threads[g_current].ctxp)
                         ? g_threads[g_current].ctxp->msr : 0xFFFFFFFFu;
            if (m != last_msr && logged < 24u) {
                unsigned c2 = g_wc_calls;
                LOG_WARN(LOG_CORE, "WC: msr %08x -> %08x (EE %s) at call %u, "
                                   "last %08x %08x",
                         (unsigned)last_msr, (unsigned)m,
                         (m & 0x8000u) ? "on" : "OFF", c2,
                         (unsigned)g_wc_crumb[(c2 - 2u) & (WC_CRUMB_N - 1u)],
                         (unsigned)g_wc_crumb[(c2 - 1u) & (WC_CRUMB_N - 1u)]);
                logged++;
            }
            last_msr = m;
        }
        {   /* WHEN TO RAISE
             *
             * Level alone is wrong: the guest's handler does not always clear
             * the line, so re-raising from the level runs the handler again at
             * the next guest call, and again, and the guest's IPC layer
             * advances its enqueue counter every time. Measured on console,
             * one interrupt walked that counter 0x0f -> 0x13 and the guest then
             * submitted three ring slots it had never filled.
             *
             * A pure edge on the cause bits is wrong the other way: every IPC
             * event sets the SAME bit, so after the first one the cause never
             * "changes" and nothing is ever delivered again.
             *
             * So: raise when the cause changes, OR when the line is still high
             * and the guest has touched the IPC registers since the last
             * delivery. The second clause is the useful one -- it means the
             * guest has acted on the previous interrupt, so a line still high
             * afterwards genuinely has more work behind it. */
            {   int due = pi_interrupt_pending();
                if (!due) due = wc_dec_due();
                if (due) {
                    if (!g_wc_irq_pending) g_wc_irq_raised++;
                    g_wc_irq_pending = 1;
                }
            }
        }
        if (g_wc_irq_pending) {
            /* The guest can also be sitting in its idle loop, which makes no
             * calls at all -- nothing there will ever reach the call-boundary
             * hook. That spin touches one word and the MSR and nothing the
             * handler touches, and it is exactly where the hardware would have
             * taken the interrupt, so deliver it here instead. */
            int all_parked = 0;
            {   /* When every registered guest thread is parked, nobody makes
                 * calls and nobody runs the idle loop either -- curctx stays
                 * on the last runner and the idle gate below never opens.
                 * On hardware the idle context would be running with EE on
                 * and take the interrupt; this is that case. */
                int used_n = 0, parked_n = 0, i2;
                for (i2 = 0; i2 < WC_MAX_THREADS; i2++)
                    if (g_threads[i2].used && !g_threads[i2].finished) {
                        used_n++;
                        if (g_threads[i2].parked) parked_n++;
                    }
                all_parked = (used_n > 0 && parked_n == used_n);
            }
            if (MemoryInline::Load<uint32_t>(kCurrentContext) == kIdleContext
                || all_parked) {
                g_deliver_kick = 1;
            } else {
                /* STALL-BREAKER: the guest is running, not idle, and has a
                 * deliverable interrupt it cannot receive. A guest loop that
                 * makes no calls -- spinning on a memory flag that only this
                 * interrupt's callback will set -- never reaches the
                 * call-boundary hook, and the idle gate above never opens. On
                 * hardware the interrupt simply lands mid-loop; that is what
                 * preemption is, and the DVD driver's sync wait relies on it.
                 *
                 * Deliver from here once the guest has demonstrably stopped
                 * calling (~50 ms with EE on). The handler then runs guest
                 * code in parallel with the spinning loop -- which is the
                 * console's interleaving made truly concurrent. The spin reads
                 * one word; the handler's completion callback writes it
                 * (word-sized, aligned, atomic on PPC), and the call-boundary
                 * hook still serialises any call the guest makes while the
                 * handler runs. */
                static unsigned stale_calls, stale_ticks;
                CpuContext *rc = g_current >= 0 ? g_threads[g_current].ctxp
                                                : nullptr;
                if (g_wc_calls != stale_calls) {
                    stale_calls = g_wc_calls;
                    stale_ticks = 0;
                } else if (rc && ((rc->msr & 0x8000u) ||
                           /* ctx->msr can be a stale MSR_FP left by a
                            * handler-sleep delivery (the landing restore was
                            * skipped by design when the handler crossed).
                            * The OS's own view -- SRR1 in the current
                            * context -- is the honest EE bit; a PI cause
                            * pending and unmasked with EE on THERE must
                            * deliver, or WPAD's flush wait stalls forever
                            * with every counter healthy. */
                           (MemoryInline::Load<uint32_t>(
                                MemoryInline::Load<uint32_t>(kCurrentContext)
                                + 0x19Cu) & 0x8000u)) &&
                           ++stale_ticks >= 8u) {
                    /* PRIORITY-INVERSION BREAKER. The running thread spins
                     * call-free on state that only ANOTHER runnable thread
                     * will produce -- EGG's display thread is woken by every
                     * retrace, is lower priority than the spinning main, and
                     * on the console would run the moment main blocks. Main
                     * never blocks here (its wait is the spin itself), so the
                     * guest's own priority rules starve the very thread it is
                     * waiting for.
                     *
                     * After repeated stall-breaker rounds with ZERO guest
                     * calls, hand the processor to each other runnable thread
                     * in turn. Main keeps spinning in parallel -- accepted
                     * concurrency -- and exits its spin the moment the thread
                     * it depends on produces the state it wants. */
                    static unsigned quiet_rounds, rr;
                    if (++quiet_rounds >= 4u) {
                        int i, n;
                        quiet_rounds = 0;
                        for (n = 1; n <= WC_MAX_THREADS; n++) {
                            i = (int)((rr + (unsigned)n) % WC_MAX_THREADS);
                            if (g_threads[i].used && g_threads[i].runnable &&
                                !g_threads[i].finished && i != g_current) {
                                rr = (unsigned)i;
                                wc_os_preempt_to(g_threads[i].osthread);
                                break;
                            }
                        }
                    }     /* ~0.5 ms of 60 us polls
                    * The original 50 ms was caution, and it showed up as a
                    * FRAME PACER: MKWii's main loop busy-waits call-free for
                    * the retrace count to change, so VI interrupts could only
                    * land through this gate -- at 20 Hz instead of 60. The
                    * game ran at a quarter speed with every counter healthy.
                    * On hardware EE-on delivery is immediate; half a
                    * millisecond is the polling loop's own resolution. */
                    stale_ticks = 0;
                    g_deliver_kick = 2;   /* breaker-forced: authorizes dec */
                }
            }
        }
        usleep(60);
    }
}
#endif

#ifdef __PS3__
static void wc_deliver_thread_main(void *arg)
{
#ifdef WC_FIBER_SCHED
    return;               /* fiber build: delivery happens on the one guest thread */
#endif
    (void)arg;
    t_my_slot = g_deliver_slot;
    for (;;) {
        g_deliver_hb++;
        if (!g_deliver_kick) { usleep(60); continue; }
        {   int kick_forced = (g_deliver_kick == 2);
            g_deliver_kick = 0;
        drain:
        g_deliver_n++;
        {   static unsigned n_dlv;
            if (n_dlv < 16u) { n_dlv++;
                LOG_WARN(LOG_CORE, "WC: DLV[%u] pend=%d dec_due=%d curctx=%08x",
                         n_dlv, pi_interrupt_pending(), wc_dec_due(),
                         (unsigned)MemoryInline::Load<uint32_t>(kCurrentContext)); } }
        {   CpuContext *dc = g_threads[g_deliver_slot].ctxp;
            /* NO masquerade: current stays the real current thread, exactly
             * as on hardware, so a sleeping handler enqueues a real thread
             * (v2's masquerade displaced the interrupted thread into no
             * queue at all and SelectThread span forever on an empty world).
             * The prologue-save contamination is handled at t_irq_ctx
             * selection instead: it lands in the idle context. */
            /* DEC stays on the boundary path whenever any guest thread can
             * take it: the AlarmHandler's BTE tick SLEEPS mid-callback, and
             * an abandoned (tail-skipped) handler splits the alarm queue
             * surgery in half -- one abandonment made the queue CYCLIC and
             * the next boundary delivery walked it forever (disp frozen at
             * OS_Alarm_DecrementerExceptionCallback+0x68). A boundary
             * delivery sleeps correctly: the sleeping thread owns its own
             * frames. Only a fully parked world, where no boundary can ever
             * fire, justifies a delivery-thread dec. */
            if (pi_interrupt_pending()) wc_irq_deliver_from(dc, 1);
            else if (kick_forced || wc_all_guest_parked() ||
                     MemoryInline::Load<uint32_t>(kCurrentContext) == kIdleContext)
                /* kick_forced: the stall-breaker saw the guest call-free for
                 * ~0.5 ms with a due interrupt -- hardware would already have
                 * delivered. The caught wedge: SelectThread's hint-spin with
                 * curctx = the spinning thread's OWN context (pre-idle), dec
                 * due forever, chooser refusing 779 times (DLV log,
                 * pend=0 dec_due=1 curctx=80343118). */
                /* The idle hint-spin leaves threads spinning EE-off with
                 * park=0, so "all parked" never holds there -- and the guest
                 * armed a 0.649 ms periodic (mtdec log) that was delivering
                 * once per ~27 s: the idle state could get no dec at all.
                 * Hardware takes the decrementer exactly in the idle loop;
                 * so do we. */
                wc_dec_due_and_deliver(dc, 1);
            /* DRAIN: the guest runs a ~65 us periodic alarm (mtdec 3948
             * ticks); waiting for the next kick cycle capped delivery at
             * ~1.2 kHz and stretched every GKI/BT tick-counted timeout ~12x
             * (HCI init at one command a minute). While something is due,
             * deliver it now -- the hardware never waited either. */
            if ((pi_interrupt_pending() &&
                 MemoryInline::Load<uint32_t>(kCurrentContext) == kIdleContext)
                || (wc_dec_due() && (kick_forced || wc_all_guest_parked() ||
                    MemoryInline::Load<uint32_t>(kCurrentContext) == kIdleContext)))
                goto drain;
        }
        }
    }
}
#endif

extern "C" void wc_irq_start(void)
{
#ifdef __PS3__
    sys_ppu_thread_t t;
    /* The delivery pseudo-thread, before the poller that kicks it. */
    {   uint32_t a;
        int i;
        for (a = 0; a < 0x400u; a += 4)
            MemoryInline::Store<uint32_t>(kDeliverThread + a, 0u);
        MemoryInline::Store<uint32_t>(kDeliverThread + OSCTX_SRR1, 0x0000A000u);
        MemoryInline::Store<uint32_t>(kDeliverThread + OSCTX_GPR(1), kDeliverStack);
        sysMutexLock(g_lock, 0);
        i = alloc_thread(kDeliverThread);
        if (i >= 0) {
            g_threads[i].ctx.gpr[1]  = kDeliverStack;
            g_threads[i].ctx.gpr[2]  = 0x8038AC20u;
            g_threads[i].ctx.gpr[13] = 0x80388880u;
            g_threads[i].ctx.msr     = MSR_FP | 0x8000u;
            g_threads[i].ctx.hid2    = HID2_PSE | HID2_LSQE;
            g_deliver_slot = i;
            if (sysThreadCreate(&g_threads[i].host, wc_deliver_thread_main,
                                nullptr, 1400, 0x10000, 0,
                                (char *)"mkw-dlv") != 0) {
                LOG_ERROR(LOG_CORE, "WC: no delivery thread");
                g_threads[i].used = 0;
                g_deliver_slot = -1;
            } else
                LOG_INFO(LOG_CORE, "WC: delivery thread up (slot %d)", i);
        }
        sysMutexUnlock(g_lock);
    }
    /* Above the game thread (lv2 counts up), so a device completion is
     * delivered promptly rather than behind a guest thread that is spinning
     * and will not yield until this runs. */
    if (sysThreadCreate(&t, wc_irq_thread, NULL, 1400, 0x10000, 0,
                        (char *)"mkw-irq") != 0)
        LOG_ERROR(LOG_CORE, "WC: could not create the interrupt thread");
    else
        LOG_INFO(LOG_CORE, "WC: interrupt delivery up");
#endif
}

/* The running guest thread's MSR, for the interrupt diagnostics: whether the
 * guest has interrupts enabled is the difference between a port that is
 * refusing to deliver and one that has nothing to deliver. */
extern "C" uint32_t wc_current_msr(void)
{
    if (g_current >= 0 && g_threads[g_current].ctxp)
        return g_threads[g_current].ctxp->msr;
    return 0xFFFFFFFFu;
}

/* Scheduler state for the rescue "sched" command: what the guest scheduler
 * believes, next to what the host threading layer believes. The stalls so far
 * have all been disagreements between the two, and every one was diagnosed a
 * build late because this view did not exist. */
extern "C" { volatile unsigned g_devloop_hb; }

/* Live register file of one guest thread. The stuck-thread questions all end
 * "...and what is r31 there?", which previously meant a printf and a rebuild.
 * Reading the slot's CpuContext answers it in the 2 s a rescue round trip
 * costs. */
extern "C" int wc_regs_report(int slot, char *out, int cap)
{
    int used = 0, i;
    CpuContext *c;
    if (slot < 0 || slot >= WC_MAX_THREADS || !g_threads[slot].used)
        return snprintf(out, (size_t)cap, "slot %d not in use\n", slot);
    c = g_threads[slot].ctxp;
    if (!c) return snprintf(out, (size_t)cap, "slot %d has no context\n", slot);
    used += snprintf(out + used, (size_t)(cap - used),
                     "slot%d os=%08x pc=%08x lr=%08x ctr=%08x msr=%08x cr=%08x\n",
                     slot, (unsigned)g_threads[slot].osthread,
                     (unsigned)c->pc, (unsigned)c->lr, (unsigned)c->ctr,
                     (unsigned)c->msr, (unsigned)c->cr);
    for (i = 0; i < 32; i += 4)
        used += snprintf(out + used, (size_t)(cap - used),
                         "r%-2d %08x  r%-2d %08x  r%-2d %08x  r%-2d %08x\n",
                         i, (unsigned)c->gpr[i], i + 1, (unsigned)c->gpr[i + 1],
                         i + 2, (unsigned)c->gpr[i + 2],
                         i + 3, (unsigned)c->gpr[i + 3]);
    return used;
}

/* Guest backtrace for one slot, walked from its saved r1 down the PPC back
 * chain ([r1] = caller frame, [frame+4] = that frame's LR). Every stuck-thread
 * question is really "what is each thread's call stack", and doing it by hand
 * costs two rescue round trips PER FRAME. slot -1 walks every live slot. */
extern "C" int wc_bt_report(int slot, char *out, int cap)
{
    int used = 0, i, lo = slot, hi = slot;
    if (slot < 0) { lo = 0; hi = WC_MAX_THREADS - 1; }
    for (i = lo; i <= hi && i < WC_MAX_THREADS && used < cap - 160; i++) {
        CpuContext *c;
        uint32_t fp;
        int d;
        if (!g_threads[i].used || g_threads[i].finished) continue;
        c = g_threads[i].ctxp;
        if (!c) continue;
        used += snprintf(out + used, (size_t)(cap - used),
                         "slot%d os=%08x lr=%08x r1=%08x park=%d\n", i,
                         (unsigned)g_threads[i].osthread, (unsigned)c->lr,
                         (unsigned)c->gpr[1], g_threads[i].parked);
        fp = c->gpr[1];
        for (d = 0; d < 10 && used < cap - 40; d++) {
            uint32_t next, lr;
            /* Only walk plausible guest stack pointers; a wild frame pointer
             * would otherwise read MMIO or run off the arena. */
            if ((fp & 3u) || (fp >> 28) == 0u) break;
            next = MemoryInline::Load<uint32_t>(fp);
            lr   = MemoryInline::Load<uint32_t>(fp + 4u);
            if (lr) used += snprintf(out + used, (size_t)(cap - used),
                                     "   %08x\n", (unsigned)lr);
            if (next <= fp) break;      /* back chain must grow upward */
            fp = next;
        }
    }
    return used;
}

extern "C" int wc_sched_report(char *out, int cap)
{
    int used = 0, i;
    used += snprintf(out + used, (size_t)(cap - used),
                     "hint=%08x curctx=%08x gcur=%d in_irq=%d pend=%d disp=%u\n",
                     (unsigned)MemoryInline::Load<uint32_t>(kRunQueueHint),
                     (unsigned)MemoryInline::Load<uint32_t>(kCurrentContext),
                     g_current, g_wc_in_irq, g_wc_irq_pending,
                     (unsigned)g_wc_dispatch_total);
    {   extern volatile const char *g_devlock_owner;
        extern volatile int g_devlock_depth;
        used += snprintf(out + used, (size_t)(cap - used),
                         "devloop_hb=%u devlock=%s depth=%d "
                         "vi_h=%u vi_w=%u\n",
                         (unsigned)g_devloop_hb,
                         g_devlock_owner ? (const char *)g_devlock_owner : "-",
                         g_devlock_depth,
                         (unsigned)g_wc_vi_handler_n,
                         (unsigned)g_wc_vi_wait_n);
    }
    {   extern void ios_bt_state(unsigned out[5]);
        unsigned b[5];
        ios_bt_state(b);
        used += snprintf(out + used, (size_t)(cap - used),
                         "bt: conn=%u intr=%u aclrx=%u credited=%u uncred=%u\n",
                         b[0], b[1], b[2], b[3], b[4]);
    {   extern void pe_debug_state(unsigned out[4]);
        unsigned pe[4];
        pe_debug_state(pe);
        used += snprintf(out + used, (size_t)(cap - used),
                         "gx: peinit=%u sync=%u cb=%u pe_ctrl=%04x tok=%04x "
                         "pend=%u/%u\n",
                         (unsigned)g_wc_gx_peinit_n, (unsigned)g_wc_gx_sync_n,
                         (unsigned)g_wc_gx_cb_n, pe[0], pe[1], pe[2], pe[3]);
    }
        used += snprintf(out + used, (size_t)(cap - used),
                         "dvd: open=%u ra=%u rp=%u thr=%u dlv_hb=%u dlv_n=%u\n",
                         (unsigned)g_wc_dvd_open_n, (unsigned)g_wc_dvd_ra_n,
                         (unsigned)g_wc_dvd_rp_n, (unsigned)g_wc_dvd_thread_n,
                         (unsigned)g_deliver_hb, (unsigned)g_deliver_n);
    }
    for (i = 0; i < WC_MAX_THREADS; i++)
        if (g_threads[i].used)
            used += snprintf(out + used, (size_t)(cap - used),
                             "slot%d os=%08x run=%d fin=%d park=%d "
                             "msr=%08x lr=%08x\n", i,
                             (unsigned)g_threads[i].osthread,
                             g_threads[i].runnable, g_threads[i].finished,
                             g_threads[i].parked,
                             g_threads[i].ctxp ? (unsigned)g_threads[i].ctxp->msr : 0u,
                             g_threads[i].ctxp ? (unsigned)g_threads[i].ctxp->lr : 0u);
    {   extern volatile uint32_t g_wc_sw_log[16];
        extern volatile unsigned g_wc_sw_n;
        unsigned n = g_wc_sw_n < 16u ? g_wc_sw_n : 16u, k;
        used += snprintf(out + used, (size_t)(cap - used), "switches(%u):",
                         g_wc_sw_n);
        for (k = 0; k < n; k++)
            used += snprintf(out + used, (size_t)(cap - used), " %08x",
                             (unsigned)g_wc_sw_log[(g_wc_sw_n - n + k) & 15u]);
        used += snprintf(out + used, (size_t)(cap - used), "\n");
    }
    return used;
}

/* ------------------------------------------------------------------ */
/* Decrementer: the guest's one programmable timer.                     */
/* ------------------------------------------------------------------ */

/* The PPU timebase, read directly: the emulator's read_timebase is a static
 * inline in its own translation unit and cannot be linked to. SPR 268 is the
 * 64-bit timebase on a 64-bit PPU; the PS3's ticks at 79.8 MHz. */
static inline u64 wc_tb(void)
{
    u64 t;
    __asm__ __volatile__("mfspr %0, 268" : "=r"(t));
    return t;
}
#define PS3_TIMEBASE_HZ 79800000ull
#define WC_DEC_HZ 60750000ull        /* Broadway timebase: 243 MHz bus / 4 */

static volatile u64 g_wc_dec_expiry;     /* PS3 timebase ticks; 0 = unarmed */

extern "C" u64 wc_dec_expiry_pub(void) { return g_wc_dec_expiry; }

extern "C" void wc_dec_write(uint32_t value)
{
#ifdef __PS3__
    /* 0x7FFFFFFF is the system software parking the decrementer. ZERO IS NOT: the
     * hardware DEC underflows one tick after mtdec(0) and interrupts --
     * it is InsertAlarm's "this alarm is already due, fire NOW" path.
     * Treating zero as a park silently dropped exactly that case, and the
     * Bluetooth stack's GKI tick -- the first alarm the boot arms -- was
     * due immediately: one dropped write, and BTA_DmIsDeviceUp span seven
     * million times a second forever. */
    if (value >= 0x7FFFFFFFu) { g_wc_dec_expiry = 0; return; }
    g_wc_dec_expiry = wc_tb()
                    + (u64)value * PS3_TIMEBASE_HZ / WC_DEC_HZ;
    if (!g_wc_dec_expiry) g_wc_dec_expiry = 1;   /* 0 means unarmed */
#else
    (void)value;
#endif
}

/* Deliver the DEC exception: enter the guest's own installed handler exactly
 * as the 0x900 vector stub would -- handler = table[8], r3 = exception number,
 * r4 = the interrupted context -- under the same serialisation, seeding and
 * rfi machinery as the external interrupt. */
/* Remaining decrementer ticks, for mfdec: the game's GetTimer reads it back
 * to decide whether an alarm is still pending. */
extern "C" uint32_t wc_dec_remaining(void)
{
#ifdef __PS3__
    u64 exp = g_wc_dec_expiry, now;
    if (!exp) return 0x7FFFFFFFu;
    now = wc_tb();
    if (now >= exp) return 0;
    return (uint32_t)((exp - now) * WC_DEC_HZ / PS3_TIMEBASE_HZ);
#else
    return 0x7FFFFFFFu;
#endif
}

/* Aged due: past expiry by more than ~1 ms. Used to let the decrementer
 * preempt a busy external line without ever double-delivering in one
 * call boundary. */
extern "C" int wc_dec_overdue(void)
{
#ifdef __PS3__
    u64 exp = g_wc_dec_expiry;
    return exp != 0 && wc_tb() >= exp + PS3_TIMEBASE_HZ / 1000ull;
#else
    return 0;
#endif
}

extern "C" int wc_dec_due(void)
{
#ifdef __PS3__
    u64 exp = g_wc_dec_expiry;
    return exp != 0 && wc_tb() >= exp;
#else
    return 0;
#endif
}

extern "C" int wc_dec_due_and_deliver(CpuContext *rc, int on_guest_thread)
{
#ifdef __PS3__
    u64 exp = g_wc_dec_expiry;
    uint32_t tbl, h;
    if (!exp || wc_tb() < exp) return 0;
    tbl = MemoryInline::Load<uint32_t>(kExcTablePtr);
    if (!tbl) { g_wc_dec_expiry = 0; return 0; }
    h = MemoryInline::Load<uint32_t>(tbl + kExcDecrementer * 4u);
    if (!h) { g_wc_dec_expiry = 0; return 0; }
    /* One-shot: the guest's AlarmHandler re-arms via PPCMtdec. Clear before
     * delivery so a handler that never rearms does not fire forever. */
    g_wc_dec_expiry = 0;
    {   extern volatile unsigned g_wc_dec_delivered;
        g_wc_dec_delivered++;
        if (g_wc_dec_delivered <= 3u)
            LOG_INFO(LOG_CORE, "WC: dec delivery #%u -> handler %08x",
                     g_wc_dec_delivered, h);
    }
    wc_irq_deliver_exc(rc, on_guest_thread, kExcDecrementer, h);
    return 1;
#else
    (void)rc; (void)on_guest_thread; return 0;
#endif
}

/* ---- zero-lag low-memory canary ----------------------------------------
 * The device-slice canary in main.c lagged the stomp by up to a slice: two
 * trips (calls=1163 and calls=1224) bracketed different innocent-looking
 * windows, so slice-time attribution can never name the stomper. This
 * version is checked at every guest call boundary (wc_irq_poll): the trip
 * snapshot is taken inside the very dispatch that first observed the
 * corruption, so the newest crumb in g_wc_canary_ring IS the call during
 * which the word changed. main.c arms it (wc_canary_service) and reports
 * the snapshot at its leisure. */
extern "C" {
volatile const uint32_t *g_wc_canary_ptr = nullptr;   /* non-null while armed */
volatile int       g_wc_canary_state = 0;   /* 0 idle 1 armed 2 tripped 3 reported */
volatile unsigned  g_wc_canary_trip_calls = 0;
volatile uint32_t  g_wc_canary_val = 0;
uint32_t           g_wc_canary_ring[64];

void wc_canary_trip(void)
{
    if (g_wc_canary_state != 1) return;
    g_wc_canary_state = 2;
    g_wc_canary_val = *g_wc_canary_ptr;
    g_wc_canary_trip_calls = g_wc_calls;
    for (unsigned i = 0; i < 64; i++) g_wc_canary_ring[i] = g_wc_crumb[i];
    g_wc_canary_ptr = nullptr;   /* one shot */
}

void wc_canary_service(void)
{
    if (g_wc_canary_state != 0) return;
    if (MemoryInline::FlatRead32(0x80000000u) == 0x524D4345u) {
        g_wc_canary_ptr = reinterpret_cast<volatile const uint32_t *>(
            MemoryInline::ResolveRangeHost(0x80000000u, 0, 4u, true, false));
        if (g_wc_canary_ptr) g_wc_canary_state = 1;
    }
}
} /* extern "C" */

/* Fiber-build interrupt pump: deliver whatever is due as a nested call on
 * the CURRENT fiber. Handlers that reschedule fiber-switch away and finish
 * when the interrupted thread is next picked. */
#ifdef __PS3__
static sys_ppu_thread_t s_guest_tid;
extern "C" void wc_note_guest_thread(void)
{
    sysThreadGetId(&s_guest_tid);
}

/* Delivery point for arming edges that produce no dispatch: the guest's
 * post-submit IPC control writes are inlined MMIO stores, and the wait that
 * follows polls RAM call-free. The MMIO WRITE handler runs host-side on the
 * guest's own thread -- the one place both the line state and a legal
 * delivery context are in hand. */
extern "C" {
volatile unsigned g_mp_calls, g_mp_line0, g_mp_thread, g_mp_ee0, g_mp_hit;
}
extern "C" void wc_pump_from_mmio(void)
{
    extern CpuContext *wcf_ctx(void);
    sys_ppu_thread_t me;
    g_mp_calls++;
    if (!(pi_intsr_raw() & pi_intmr_raw())) { g_mp_line0++; return; }
    sysThreadGetId(&me);
    if (me != s_guest_tid) { g_mp_thread++; return; }
    {   CpuContext *c = wcf_ctx();
        /* Only when the guest could take the interrupt RIGHT HERE: the
         * arming writes inside OSDisableInterrupts brackets must not run the
         * handler mid-critical-section (measured: 130k early-returns and a
         * corrupted open sequence when they did). */
        if (c && (c->msr & 0x8000u)) { g_mp_hit++; wc_irq_pump(c); }
        else g_mp_ee0++;
    }
}
#endif

/* Catch the current-thread global being stomped with a small integer, at
 * the first dispatch after it happens; the crumb ring then holds the
 * culprit's neighbourhood. */
extern "C" void wc_dump_crumbs_now(void)
{
    /* The ring is g_wc_crumb[64], indexed by g_wc_calls (ppc_runtime.h). */
    unsigned n = (unsigned)g_wc_calls, i;
    for (i = 0; i < 4; i++) {
        unsigned b = (n - 32u + i * 8u);
        LOG_WARN(LOG_CORE, "CRUMB: %08x %08x %08x %08x %08x %08x %08x %08x",
                 g_wc_crumb[b & 63u], g_wc_crumb[(b+1)&63u],
                 g_wc_crumb[(b+2)&63u], g_wc_crumb[(b+3)&63u],
                 g_wc_crumb[(b+4)&63u], g_wc_crumb[(b+5)&63u],
                 g_wc_crumb[(b+6)&63u], g_wc_crumb[(b+7)&63u]);
    }
}

extern "C" { volatile int g_host_site; }   /* 1=idle 2=irq_wait 3=ipc2sync 4=ipc2cb */

extern "C" { volatile unsigned g_bdrain_hits; }
extern "C" unsigned wc_calls_probe(void) { return (unsigned)g_wc_calls; }
extern "C" int t_in_irq_probe(void) { return t_in_irq; }
extern "C" int g_wc_in_irq_probe_val(void) { return (int)g_wc_in_irq; }

extern "C" void wc_irq_state_force_clear(void)
{
    if (g_wc_in_irq || t_in_irq) {
        static unsigned n;
        if (n < 8u) { n++;
            LOG_WARN(LOG_CORE, "WC: idle clears leaked irq state (t=%d g=%d owner=%d)",
                     (int)t_in_irq, (int)g_wc_in_irq, (int)g_wc_irq_owner); }
        t_irq_jmp_armed = 0;
        t_irq_switch_to = 0;
        g_wc_irq_owner  = -2;
        t_in_irq        = 0;
        g_wc_in_irq     = 0;
    }
}

extern "C" void wc_e4_stomp_check(void);

/* Boundary sanity v2: E4 plausibility AND the live mirror's r1/lr. The
 * mirror poisonings (r1=0x31000000, lr=string bytes) slipped every
 * restore-side detector -- catch them within ONE dispatch of the write,
 * with the crumb ring naming the neighbourhood. */
extern "C" void wc_e4_stomp_check2(CpuContext *ctx)
{
    static int fired2;
    if (!fired2) {
        uint32_t r1 = (uint32_t)ctx->gpr[1], lr = (uint32_t)ctx->lr;
        int bad = 0;
        if (r1 < 0x80003000u || r1 >= 0x94000000u) bad = 1;
        if (lr && (lr < 0x80004000u || lr >= 0x80290000u)) bad = 1;
        if (bad) {
            fired2 = 1;
            LOG_WARN(LOG_CORE, "MIRRORPOISON r1=%08x lr=%08x -- crumbs follow",
                     r1, lr);
            wc_dump_crumbs_now();
        }
    }
    wc_e4_stomp_check();
}

extern "C" void wc_e4_stomp_check(void)
{
    static int fired;
    if (fired) return;
    /* Pre-threading, E4 holds loader leftovers; the trap burned its one
     * shot on that (8008f7b8, a text address, D4=0). Arm only once the
     * fiber system has adopted the boot thread. */
    {   extern uint32_t wcf_current_osthread(void);
        if (!wcf_current_osthread()) return;
    }
    {   uint32_t v = MemoryInline::Load<uint32_t>(0x800000E4u);
        int bad = 0;
        if (v) {
            if (v < 0x80003000u || v >= 0x93F00000u || (v & 3u)) bad = 1;
            else {
                uint32_t st = MemoryInline::Load<uint32_t>(v + 0x2C8u) >> 16;
                if (st != 0u && st != 1u && st != 2u && st != 4u && st != 8u)
                    bad = 1;
            }
        }
        if (bad) {
            fired = 1;
            LOG_WARN(LOG_CORE, "E4STOMP: OS_CURRENT_THREAD=%08x (D4=%08x) -- crumbs follow",
                     v, MemoryInline::Load<uint32_t>(0x800000D4u));
            wc_dump_crumbs_now();
        }
    }
}

extern "C" void wc_irq_pump(CpuContext *ctx)
{
    /* Reentrancy, PER FIBER, with the flag stored ON the fiber. The
     * skip-if-pumping test must see the flag of the fiber that is executing
     * NOW; a fiber that switched away mid-delivery keeps its own flag set
     * until it resumes and finishes -- and no other fiber is affected. The
     * previous owner-variable version leaked exactly like the global it
     * replaced: the restoring line was parked with the switching fiber. */
    extern int  wcf_pump_enter(void);
    extern void wcf_pump_exit(void);
    /* A nested pump while a delivery is in flight ON THIS HOST THREAD is a
     * nested interrupt while one is in service: hardware defers it to the
     * rfi. Waiting here (wc_irq_wait) waits on OURSELVES -- the owner is
     * parked up this very stack. Skip; the line stays high and the next
     * legal pump takes it. */
    if (t_in_irq) return;
    if (!wcf_pump_enter()) return;
    {   unsigned before = (unsigned)g_wc_irq_delivered;
        int line_hi = (pi_intsr_raw() & pi_intmr_raw()) != 0;
        if (line_hi)           wc_irq_deliver_from(ctx, 1);
        else if (wc_dec_due()) wc_dec_due_and_deliver(ctx, 1);
        if (line_hi && (unsigned)g_wc_irq_delivered == before &&
            (pi_intsr_raw() & pi_intmr_raw())) {
            static unsigned n;
            if (n < 6u) { n++;
                LOG_WARN(LOG_CORE, "PUMPSTALL[%u] in_irq=%d t_in=%d armed=%d owner=%d line=%04x",
                         n, (int)g_wc_in_irq, (int)t_in_irq,
                         (int)t_irq_jmp_armed, (int)g_wc_irq_owner,
                         pi_intsr_raw() & pi_intmr_raw()); }
        }
    }
    wcf_pump_exit();
    /* POST-DELIVERY PREEMPTION -- hardware's exception epilogue runs
     * __OSReschedule, so a handler that readied a higher-priority thread
     * preempts the interrupted code at the rfi. Without it, a polling wait
     * monopolized the single guest thread forever: the BTA tick fired
     * 25,441 times, woke the BTU task each time, and the task never got
     * the processor (sw frozen at 4). Full volatile save: the interrupted
     * caller's staged argument registers must survive. */
    if (!t_in_irq &&
        MemoryInline::Load<uint32_t>(0x803825A0u) != 0) {
        extern void wc_hle_SelectThread(CpuContext *);
        uint32_t sg[13]; PPC_FPR sf[14];
        uint32_t slr = (uint32_t)ctx->lr, sctr = (uint32_t)ctx->ctr;
        uint32_t scr = ctx->cr, sxer = ctx->xer;
        int k;
        sg[0] = ctx->gpr[0];
        for (k = 0; k < 10; k++) sg[k + 1] = ctx->gpr[3 + k];
        for (k = 0; k < 14; k++) sf[k] = ctx->fpr[k];
        ctx->gpr[3] = 0;
        wc_hle_SelectThread(ctx);
        ctx->gpr[0] = sg[0];
        for (k = 0; k < 10; k++) ctx->gpr[3 + k] = sg[k + 1];
        for (k = 0; k < 14; k++) ctx->fpr[k] = sf[k];
        ctx->lr = slr; ctx->ctr = sctr;
        ctx->cr = scr; ctx->xer = sxer;
    }
}
