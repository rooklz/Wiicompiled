/* wc_watchdog.cpp -- keep the console reachable when the game misbehaves.
 *
 * The port runs the game as native code on its own thread. If that thread
 * spins -- an early-boot poll on a device flag that never gets set, a wait on
 * an interrupt that never arrives -- it does so at full speed and forever,
 * because unlike the JIT it has no slice boundary to return through.
 *
 * That already cost one console: FTP, devlink, the rescue listener and even
 * webMAN stopped answering, TCP connections were accepted by the kernel and
 * then serviced by nobody, and the machine had to be power-cycled. Nothing
 * about that was diagnosable from outside, which is the part worth fixing.
 *
 * So: a thread that watches guest progress and, when it stops, takes the game
 * thread's priority away rather than the machine's. Priority, not a kill --
 * the guest's state stays intact and readable, `stat` keeps answering, and a
 * developer can still ask what it was doing. A wedged game becomes a slow
 * game, and control is never lost.
 */
extern "C" {
#include "../../../common/log.h"
#include "../gekko.h"
}
#include "ppc_runtime.h"
#ifdef __PS3__
#include <sys/thread.h>
#include <sys/systime.h>
#include <unistd.h>
#include <cstdio>
#endif

extern volatile unsigned g_wc_calls;
extern volatile unsigned g_wc_dispatch_total;
extern uint32_t          g_wc_crumb[];
extern "C" u64 g_mkw_frames_pub;

namespace {
#ifdef __PS3__
sys_ppu_thread_t g_wd_thread;
#endif
volatile int g_wd_tripped;

/* The game thread's priority, and where it is pushed when it stops making
 * progress. 2500 is well below every service thread but still runnable, so a
 * guest that is merely slow recovers on its own. */
const int kGamePrio    = 1500;
const int kParkedPrio  = 2500;

void watchdog_thread(void *arg)
{
    unsigned last_calls = 0;
    u64      last_frames = 0;
    int      idle = 0;
#ifdef __PS3__
    sys_ppu_thread_t game = *(sys_ppu_thread_t *)arg;
#else
    (void)arg;
#endif
    for (;;) {
#ifdef __PS3__
        usleep(1000000);
#endif
        {   /* g_wc_calls is rewound past handler-only activity by design; a
             * legitimately alarm-driven idle phase therefore reads as frozen
             * and the demotion never lifts. The dispatch total is the honest
             * "is guest code executing" signal. */
            unsigned c = g_wc_dispatch_total;
            u64      f = g_mkw_frames_pub;
            if (c != last_calls || f != last_frames) {
                last_calls = c; last_frames = f; idle = 0;
                if (g_wd_tripped) {
                    /* It came back. Give it its priority back rather than
                     * leaving the game permanently slow. */
                    g_wd_tripped = 0;
#ifdef __PS3__
                    sysThreadSetPriority(game, kGamePrio);
#endif
                    LOG_INFO(LOG_CORE, "WC: guest progressing again, priority restored");
                }
                continue;
            }
        }
        if (++idle == 10 && !g_wd_tripped) {
            g_wd_tripped = 1;
#ifdef __PS3__
            sysThreadSetPriority(game, kParkedPrio);
#endif
            LOG_WARN(LOG_CORE, "WC: no guest progress for 10 s (calls=%u frames=%llu) "
                     "-- game thread parked at low priority so the console stays "
                     "reachable", g_wc_calls,
                     (unsigned long long)g_mkw_frames_pub);
            /* The last guest calls before it stopped, oldest first. Native code
             * has no pc to report and no interpreter to ask, so without this a
             * hang is one log line and 13,675 candidate functions. Addresses
             * are NTSC guest addresses: look them up in MAP_ntsc_full.txt. */
            {   unsigned n = g_wc_calls < WC_CRUMB_N ? g_wc_calls : WC_CRUMB_N;
                unsigned k;
                char line[128];
                int  used = 0;
                LOG_WARN(LOG_CORE, "WC: last %u guest calls before the stall:", n);
                for (k = 0; k < n; k++) {
                    unsigned idx = (g_wc_calls - n + k) & (WC_CRUMB_N - 1u);
                    used += snprintf(line + used, sizeof line - (size_t)used,
                                     " %08x", (unsigned)g_wc_crumb[idx]);
                    if ((k % 8) == 7 || k == n - 1) {
                        LOG_WARN(LOG_CORE, "WC:  %s", line);
                        used = 0; line[0] = 0;
                    }
                }
            }
        }
    }
}
} /* namespace */

extern "C" void wc_watchdog_start(void *game_thread_id)
{
#ifdef __PS3__
    static sys_ppu_thread_t game;
    game = *(sys_ppu_thread_t *)game_thread_id;
    if (sysThreadCreate(&g_wd_thread, watchdog_thread, &game, 900, 0x4000, 0,
                        (char *)"mkw-wd") != 0)
        LOG_WARN(LOG_CORE, "WC: watchdog thread not created");
    else
        LOG_INFO(LOG_CORE, "WC: progress watchdog up");
#else
    (void)game_thread_id;
#endif
}
