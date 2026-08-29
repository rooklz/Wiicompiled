/* difftrace.c -- lockstep execution fingerprints for console-vs-qemu diffing.
 *
 * The title issues byte-identical HCI commands and receives byte-identical
 * events and ACL frames on both machines, yet arrives at the same guest
 * moment with materially different memory -- so the divergence is in guest
 * EXECUTION, not in anything the Bluetooth layer hands it. Arguing about which
 * instruction is at fault is hopeless; measuring it is not.
 *
 * Both builds sample the CPU at the same cadence and print a fingerprint.
 * Diffing the two streams gives the first sample where they part, which bounds
 * the offending instruction to one sampling interval; re-running with a finer
 * interval around that point narrows it to the instruction itself.
 *
 * The sample must depend only on guest state, never on host time, or it would
 * differ for reasons that have nothing to do with the bug.
 */
#include "difftrace.h"
#include <stdio.h>
#include <stdlib.h>
#include "../common/log.h"
#include "core_timing.h"

static int      s_on = -1;
static u32      s_interval = 1000;
static u64      s_from;      /* start sampling at this slice */
static u64      s_to;        /* stop after this slice (0 = no limit) */
static u64      s_seq;
int             g_dt_pctrace;      /* 1 while inside the traced slice */

static void difftrace_config(void)
{
    const char *e;
    FILE *f;
    if (s_on >= 0) return;
    s_on = 0;
    /* Accepts "interval", "interval:from" or "interval:from:to" so a coarse
     * pass can bound the divergence and a second pass can resample that window
     * one slice at a time without drowning in output. */
    {
        unsigned long a = 0, b = 0, c2 = 0;
        int got = 0;
        if ((e = getenv("DIFFTRACE")) && *e) {
            got = sscanf(e, "%lu:%lu:%lu", &a, &b, &c2);
        } else if ((f = fopen("/dev_hdd0/tmp/wiicompiled-difftrace.txt", "r")) != NULL) {
            got = fscanf(f, "%lu:%lu:%lu", &a, &b, &c2);
            fclose(f);
        }
        if (got >= 1 && a) {
            s_on = 1;
            s_interval = (u32)a;
            if (got >= 2) s_from = (u64)b;
            if (got >= 3) s_to   = (u64)c2;
        }
    }
    if (s_interval == 0) s_interval = 1000;
    if (s_on)
        LOG_INFO(LOG_CORE, "DIFFTRACE: every %u slices, from %llu to %llu",
                 s_interval, (unsigned long long)s_from,
                 (unsigned long long)s_to);
}

/* Log the slice accounting itself: granted budget, the downcount left, the
 * refund parked in exit_slack, and what the scheduler therefore charged. The
 * two machines run identical instructions and identical event schedules yet
 * charge different cycles, so one of these four terms has to differ -- this
 * says which, instead of leaving it to inference. */
void difftrace_note_slice(s32 granted, s32 downcount, s32 slack, s64 consumed)
{
    difftrace_config();
    if (!s_on) return;
    /* Arm the per-instruction trace for the NEXT slice while it is inside the
     * window. The two machines take different path lengths through the same
     * code from identical registers, so the only way to name the instruction
     * where they part is to print the path. */
    g_dt_pctrace = (s_seq + 1 >= s_from && (!s_to || s_seq + 1 <= s_to));
    if (s_seq < s_from) return;
    if (s_to && s_seq > s_to) return;
    LOG_INFO(LOG_CORE, "DS %010llu granted=%d dc=%d slack=%d consumed=%lld",
             (unsigned long long)s_seq, (int)granted, (int)downcount,
             (int)slack, (long long)consumed);
}

void difftrace_sample(const PPCState *s)
{
    u64 h = 1469598103934665603ull;      /* FNV-1a */
    unsigned i;
    u32 words[8];
    u64 seq;

    difftrace_config();
    if (!s_on) return;
    {
        seq = s_seq++;
        if (seq < s_from) return;
        if (s_to && seq > s_to) return;
        if (((seq - s_from) % s_interval) != 0) return;
    }

    words[0] = s->pc;      words[1] = s->lr;      words[2] = s->ctr;
    words[3] = s->cr;      words[4] = s->msr;     words[5] = s->fpscr;
    words[6] = s->xer_ca | (s->xer_so << 1) | (s->xer_ov << 2);
    words[7] = s->hid2;
    for (i = 0; i < 8; i++) {
        h ^= words[i]; h *= 1099511628211ull;
    }
    for (i = 0; i < 32; i++) {
        h ^= s->gpr[i]; h *= 1099511628211ull;
    }
    /* The floating-point file matters as much as the integer one. A loop whose
     * exit condition is an FP compare runs a different number of iterations if
     * the two machines disagree about a result -- different cycle counts for
     * the same visible integer state, which is precisely the symptom. Native
     * PPE arithmetic and qemu's emulation of it are the obvious place for such
     * a disagreement, and hashing only the GPRs would hide it completely. */
    for (i = 0; i < 32; i++) {
        h ^= s->ps[i].ps0.u; h *= 1099511628211ull;
        h ^= s->ps[i].ps1.u; h *= 1099511628211ull;
    }
    /* The register hashes matched exactly up to the slice where the console
     * takes a decrementer exception that qemu does not, so the GUEST state is
     * identical and the EMULATOR's cycle accounting is not. Printing the
     * timebase alongside shows which slice the two clocks part on -- that is
     * the thing that actually diverges, and everything else follows from it. */
    LOG_INFO(LOG_CORE, "DT %010llu pc=%08x lr=%08x tb=%012llx dec=%08x h=%016llx",
             (unsigned long long)seq, (unsigned)s->pc, (unsigned)s->lr,
             (unsigned long long)timing_timebase(),
             (unsigned)timing_read_decrementer(s),
             (unsigned long long)h);
}
