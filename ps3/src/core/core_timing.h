/* core_timing.h — the emulated machine's clock and event scheduler.
 *
 * Everything outside the CPU happens on a schedule: the decrementer fires, the
 * video interface reaches a scanline, the DSP finishes a block, the disc drive
 * completes a transfer. Games are written around that cadence and notice when
 * it is wrong -- a vertical blank that arrives early makes animation stutter, a
 * decrementer that drifts breaks frame pacing.
 *
 * The design fits the recompiler rather than fighting it. The JIT already
 * decrements a pinned register per block and exits when it goes negative
 * (docs/ARCHITECTURE.md §4.2); that counter *is* this scheduler's slice budget.
 * The CPU is handed exactly as many cycles as remain until the next event, runs
 * them at full speed with no per-instruction checks, and control returns
 * naturally when the budget is spent. There is no polling anywhere.
 *
 * Time is counted in *guest CPU cycles* throughout. Converting at the edges
 * rather than in the middle keeps the arithmetic exact: everything the guest
 * can observe is derived from one integer.
 */
#ifndef DOLPHIN_CORE_TIMING_H
#define DOLPHIN_CORE_TIMING_H

#include "ppc/gekko.h"

/* ------------------------------------------------------------------ */
/* Clocks                                                              */
/*                                                                     */
/* The GameCube and Wii differ by exactly 1.5x throughout: CPU, bus and GPU all */
/* scale together, which is why a single `wii_mode` flag is enough.             */
/* ------------------------------------------------------------------ */

#define GC_CPU_HZ     486000000u    /* Gekko    */
#define GC_BUS_HZ     162000000u
#define WII_CPU_HZ    729000000u    /* Broadway */
#define WII_BUS_HZ    243000000u

/* The time base counts at one quarter of the bus clock. Titles read it for
 * timing and to seed random number generators, so its rate has to be right. */
#define TB_DIVISOR    4u

extern u32 g_cpu_hz;
extern u32 g_bus_hz;

DOL_INLINE s64 timing_us_to_cycles(s64 us)
{
    return (s64)((u64)us * (u64)g_cpu_hz / 1000000ull);
}

DOL_INLINE s64 timing_ms_to_cycles(s64 ms)
{
    return (s64)((u64)ms * (u64)g_cpu_hz / 1000ull);
}

/* ------------------------------------------------------------------ */
/* Events                                                              */
/* ------------------------------------------------------------------ */

/* `cycles_late` is how far past the scheduled time the callback actually ran.
 * Periodic events must fold it back into their next deadline, otherwise the
 * error accumulates and the emulated machine slowly runs slow -- the classic
 * way an emulator ends up at 59.2 Hz instead of 59.94. */
typedef void (*TimingCallback)(u64 userdata, s64 cycles_late);

#define TIMING_MAX_EVENT_TYPES 24

void timing_init(int wii_mode);
void timing_shutdown(void);

/* Register a kind of event once at startup; returns its handle. Named so that
 * the scheduler can be dumped meaningfully when something misbehaves. */
int  timing_register_event(const char *name, TimingCallback cb);

/* Schedule (or reschedule) an event. Scheduling a type that is already pending
 * replaces it, which is what every periodic device wants. */
void timing_schedule(int event_type, s64 cycles_from_now, u64 userdata);
void timing_remove(int event_type);

/* Total guest cycles executed since reset. */
u64  timing_now(void);

/* Fold the cycles a CPU slice actually consumed into the global clock, then run
 * every event that has come due. Called by the run loop when the downcount is
 * exhausted. */
void timing_advance(PPCState *s);

/* Cycles the CPU may run before the next event is due; this becomes the
 * downcount. Clamped to a maximum slice so an idle machine still returns to the
 * host regularly for input and frame presentation. */
s32  timing_slice(void);

/* The guest's time base, derived from the cycle count so the two can never
 * drift apart. */
u64  timing_timebase(void);
void timing_bind_cpu(PPCState *cpu);
PPCState *timing_bound_cpu(void);

/* Decrementer support: the guest's DEC counts down at the time-base rate and
 * raises an exception when it passes zero. */
void timing_write_decrementer(PPCState *s, u32 value);
u32  timing_read_decrementer(const PPCState *s);

#endif /* DOLPHIN_CORE_TIMING_H */
