/* core_timing.c — guest clock and event scheduler.
 *
 * The event set is small -- a handful of devices, each with one or two pending
 * events -- so an ordered singly-linked list beats a heap here: insertion walks
 * two or three nodes, and the "what runs next" query, which happens far more
 * often, is a pointer read.
 */
#include "difftrace.h"
#include <stdio.h>
#include <stdlib.h>
#include "core_timing.h"
#include "ppc/interp/interp.h"
#include "../common/log.h"

#include <string.h>

u32 g_cpu_hz = WII_CPU_HZ;
u32 g_bus_hz = WII_BUS_HZ;

/* A slice long enough that scheduling overhead is negligible, short enough that
 * the host stays responsive when the guest has nothing scheduled. At 729 MHz
 * this is a little over a millisecond. */
#define MAX_SLICE_CYCLES 1000000

typedef struct Event {
    u64            when;        /* absolute cycle count */
    u64            userdata;
    int            type;
    struct Event  *next;
} Event;

typedef struct {
    const char    *name;
    TimingCallback cb;
} EventType;

static EventType s_types[TIMING_MAX_EVENT_TYPES];
static int       s_type_count;

/* One slot per type: a device only ever has one pending event of a kind, and
 * rescheduling replaces it. This removes all allocation from the scheduler. */
static Event     s_slots[TIMING_MAX_EVENT_TYPES];
static Event    *s_head;

static u64       s_now;             /* cycles retired before the current slice */
static s32       s_slice_granted;   /* cycles handed to the CPU for this slice */

/* ------------------------------------------------------------------ */

void timing_init(int wii_mode)
{
    g_cpu_hz = wii_mode ? WII_CPU_HZ : GC_CPU_HZ;
    g_bus_hz = wii_mode ? WII_BUS_HZ : GC_BUS_HZ;

    memset(s_types, 0, sizeof s_types);
    memset(s_slots, 0, sizeof s_slots);
    s_type_count = 0;
    s_head = NULL;
    s_now = 0;
    s_slice_granted = 0;

    LOG_INFO(LOG_CORE, "timing: CPU %u MHz, bus %u MHz, time base %u MHz",
             g_cpu_hz / 1000000u, g_bus_hz / 1000000u,
             g_bus_hz / TB_DIVISOR / 1000000u);
}

void timing_shutdown(void)
{
    s_head = NULL;
    s_type_count = 0;
}

static void unlink_event(int type);

int timing_register_event(const char *name, TimingCallback cb)
{
    int i;

    /* Registering the same name twice must return the SAME event type, not a
     * second one. There is one queue slot per type, so a duplicate type meant
     * a duplicate pending event: hw_init() runs more than once on the console
     * (the RSX self-test and the session supervisor both call it) and every
     * pass re-registered "VI line", leaving TWO VI events queued at now=0
     * where a single-init build has one.
     *
     * That is not cosmetic. timing_slice() caps each slice at the next
     * scheduled event, so an extra event changes slice lengths, which changes
     * how many cycles are charged for identical guest work, which moves the
     * decrementer. Measured against qemu, the two clocks first part at slice
     * 357 and never recover; by the time the title is setting up Bluetooth it
     * has taken a different number of interrupts and is in a different state.
     *
     * Any pending event for the type is dropped, so re-initialisation starts
     * from a clean queue exactly as a first initialisation would. */
    for (i = 0; i < s_type_count; i++) {
        if (s_types[i].name && name && !strcmp(s_types[i].name, name)) {
            s_types[i].cb = cb;
            unlink_event(i);
            return i;
        }
    }

    if (s_type_count >= TIMING_MAX_EVENT_TYPES) {
        LOG_ERROR(LOG_CORE, "too many event types registering '%s'", name);
        return -1;
    }
    s_types[s_type_count].name = name;
    s_types[s_type_count].cb   = cb;
    s_slots[s_type_count].type = s_type_count;
    return s_type_count++;
}

u64 timing_now(void)
{
    /* Updated at slice boundaries, and crucially *before* callbacks run, so a
     * device reading the clock from inside its own callback sees the time its
     * event was due rather than the previous slice's. */
    return s_now;
}

/* ------------------------------------------------------------------ */
/* Queue                                                               */
/* ------------------------------------------------------------------ */


static void unlink_event(int type)
{
    Event **pp = &s_head;
    while (*pp) {
        if ((*pp)->type == type) { *pp = (*pp)->next; return; }
        pp = &(*pp)->next;
    }
}

void timing_remove(int event_type)
{
    if (event_type < 0 || event_type >= s_type_count)
        return;
    unlink_event(event_type);
}

void timing_schedule(int event_type, s64 cycles_from_now, u64 userdata)
{
    {   /* Trace the event schedule. timing_slice() caps every slice at the
         * next scheduled event, so two builds with different event schedules
         * take different slice lengths and charge different cycles for the
         * same guest work -- which is exactly the divergence being chased.
         * Diffing these two streams says whether the schedules differ. */
        static int on = -1;
        static unsigned n;
        if (on < 0) {
            FILE *f;
            on = getenv("EVTRACE") != NULL;
            if (!on && (f = fopen("/dev_hdd0/tmp/dolphin-evtrace.txt", "r")))
                { fclose(f); on = 1; }
        }
        if (on && n < 4000) { n++;
            LOG_INFO(LOG_CORE, "EV %s +%lld now=%llu",
                     s_types[event_type].name ? s_types[event_type].name : "?",
                     (long long)cycles_from_now, (unsigned long long)s_now); }
    }

    Event *e, **pp;

    if (event_type < 0 || event_type >= s_type_count)
        return;
    if (cycles_from_now < 0)
        cycles_from_now = 0;

    unlink_event(event_type);           /* rescheduling replaces */

    e = &s_slots[event_type];
    e->when     = s_now + (u64)cycles_from_now;
    e->userdata = userdata;
    e->type     = event_type;

    for (pp = &s_head; *pp && (*pp)->when <= e->when; pp = &(*pp)->next)
        ;
    e->next = *pp;
    *pp = e;
}

/* The CPU whose downcount measures progress inside the current slice. Bound
 * once at init; only read here. */
static PPCState *s_bound_cpu;

void timing_bind_cpu(PPCState *cpu) { s_bound_cpu = cpu; }
PPCState *timing_bound_cpu(void) { return s_bound_cpu; }

/* Cycles now -- *including* the portion of the current slice already executed.
 *
 * s_now alone advances only at slice boundaries, which quantises every derived
 * clock to the slice length. That is invisible to code that just paces itself,
 * but fatal to code that *measures*: MKWii's AI calibration times a single
 * sample-counter tick against the time base, and with both clocks frozen
 * mid-slice each measurement collapsed to "exactly one slice", the same at
 * 32 kHz and 48 kHz, and its sanity check span forever. The downcount is the
 * midslice progress meter -- the interpreter keeps it current per instruction,
 * and the JIT stores it on every exit and every MMIO escape, so any code that
 * can observe a clock also observes a fresh downcount. */
static u64 now_cycles(void)
{
    u64 n = s_now;
    if (s_bound_cpu && s_slice_granted > 0) {
        s64 used = (s64)s_slice_granted - (s64)s_bound_cpu->downcount;
        if (used > 0) {
            if (used > (s64)s_slice_granted)
                used = s_slice_granted;
            n += (u64)used;
        }
    }
    return n;
}

/* Optional ceiling on a slice, in guest instructions.
 *
 * A guest thread that spins waiting on another thread cannot make progress
 * until the scheduler runs, and the scheduler only runs when the slice ends --
 * so every instruction of the remaining grant is burned in the spin. MKWii
 * does exactly this: the block profiler attributes 52% of ALL executed guest
 * instructions to one RFL wait loop, and the title executes ~23.7M
 * instructions per frame against a real Wii's ~12.2M.
 *
 * Shortening the slice bounds that waste directly, at the cost of running the
 * scheduler more often (measured at ~0.8 us per slice, 1.3% of the frame).
 * Zero means "no extra ceiling" -- the original behaviour. */
s32 g_timing_slice_cap;

s32 timing_slice(void)
{
    s64 budget = MAX_SLICE_CYCLES;

    if (g_timing_slice_cap > 0 && budget > (s64)g_timing_slice_cap)
        budget = g_timing_slice_cap;

    if (s_head) {
        s64 until = (s64)(s_head->when - s_now);
        if (until < budget)
            budget = until;
    }
    if (budget < 0)
        budget = 0;

    s_slice_granted = (s32)budget;
    return s_slice_granted;
}

void timing_advance(PPCState *s)
{
    /* The CPU stops with `downcount` at or below zero; whatever is left (or
     * overshot) tells us exactly how much of the grant was consumed. Deriving
     * it this way rather than assuming the full slice ran is what keeps the
     * clock correct when a block exits early -- for an exception, a device
     * access, or a request to stop.
     *
     * exit_slack is the budget a forced exit destroyed by driving downcount
     * negative; without subtracting it, every such exit would charge the whole
     * remaining slice and emulated time would run fast (gekko.h, exit_slack).
     * The clamp is belt and braces: consumed can never legitimately exceed what
     * was granted, and a bug that made it so would corrupt the clock silently. */
    s64 consumed = (s64)s_slice_granted - (s64)s->downcount - (s64)s->exit_slack;
    if (consumed < 0)
        consumed = 0;
    if (consumed > (s64)s_slice_granted)
        consumed = s_slice_granted;
    difftrace_note_slice(s_slice_granted, s->downcount, s->exit_slack,
                         consumed);
    s_now += (u64)consumed;
    s_slice_granted = 0;
    s->downcount = 0;
    s->exit_slack = 0;

    while (s_head && s_head->when <= s_now) {
        Event *e = s_head;
        s64 late = (s64)(s_now - e->when);
        s_head = e->next;
        e->next = NULL;
        if (s_types[e->type].cb)
            s_types[e->type].cb(e->userdata, late);
    }

    /* The decrementer is sampled rather than ticked, so it is checked once per
     * slice instead of once per instruction. */
    if ((s->msr & MSR_EE) && timing_read_decrementer(s) == 0 &&
        !(s->exceptions & EXC_DECREMENTER))
        ppc_raise(s, EXC_DECREMENTER);

    s->tb = timing_timebase();
}

u64 timing_timebase(void)
{
    /* Time base ticks at bus/4, and the clock is counted in CPU cycles, so
     * convert with the fixed CPU:bus ratio rather than a floating factor. */
    return now_cycles() / ((u64)(g_cpu_hz / g_bus_hz) * TB_DIVISOR);
}

/* ------------------------------------------------------------------ */
/* Decrementer                                                         */
/*                                                                     */
/* Modelled by remembering the value and the moment it was written, then        */
/* computing the current reading on demand. Ticking it per instruction would    */
/* cost more than the whole rest of the scheduler and gain nothing: the guest   */
/* can only observe it through `mfspr`, which is rare.                          */
/* ------------------------------------------------------------------ */

void timing_write_decrementer(PPCState *s, u32 value)
{
    s->dec = value;
    s->dec_write_tb = timing_timebase();
}

u32 timing_read_decrementer(const PPCState *s)
{
    u64 elapsed = timing_timebase() - s->dec_write_tb;
    if (elapsed >= (u64)s->dec)
        return 0;
    return s->dec - (u32)elapsed;
}
