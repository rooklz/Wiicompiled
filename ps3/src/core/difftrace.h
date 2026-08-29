/* difftrace.h -- see difftrace.c. Sampling is off unless DIFFTRACE is set in
 * the environment (qemu) or /dev_hdd0/tmp/wiicompiled-difftrace.txt exists on the
 * console; the value is the sampling interval in slices. */
#ifndef DOLPHIN_CORE_DIFFTRACE_H
#define DOLPHIN_CORE_DIFFTRACE_H

#include "ppc/gekko.h"

void difftrace_sample(const PPCState *s);
void difftrace_note_slice(s32 granted, s32 downcount, s32 slack, s64 consumed);

#endif
