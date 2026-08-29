/* dev_lock.c -- one lock over the emulated device model.
 *
 * WHY
 *
 * In the emulator the guest and the devices share a thread: the JIT executes,
 * hits an MMIO access, runs the device handler inline, returns. Nothing can
 * interleave, and none of the device state needs a lock.
 *
 * The port breaks that. Translated guest code runs on the game thread and its
 * MMIO accesses run the device handlers THERE, while the main loop advances
 * emulated time and services the same devices on its own thread -- and the
 * interrupt thread touches them too. PI, IPC, VI, DSP, AI and DI all became
 * shared mutable state with no synchronisation at all.
 *
 * It showed up in IPC, which has the most state: the guest submitted a request
 * while the main loop was releasing a reply, and a block IOS had already
 * stamped with its reply marker got dispatched a second time. The model
 * answered the stale block "bad command 8" -> EINVAL, the guest took that -4
 * as the result of its own /dev/di open, and MKWii put up the Wii system
 * error screen.
 *
 * COARSE ON PURPOSE
 *
 * One lock for the whole device model, not one per device. The devices call
 * each other -- IPC raises a PI interrupt, DI completes through IPC -- so
 * per-device locks would have to be ordered, and the ordering would be a
 * second thing to get wrong. Guest MMIO is not a hot path: it is register
 * traffic, thousands of accesses a frame against millions of instructions.
 *
 * It is recursive because a device handler reached through MMIO can perform
 * further guest memory accesses that land back in MMIO.
 */
#include "hardware.h"
#include "../../common/log.h"

#ifdef __PS3__
#include <sys/mutex.h>
#include <sys/thread.h>

static sys_mutex_t s_lock;
static int         s_ready;

void dev_lock_init(void)
{
    sys_mutex_attr_t ma;
    if (s_ready) return;
    sysMutexAttrInitialize(ma);
    /* Recursive: an MMIO handler may itself touch guest memory that is MMIO. */
    ma.attr_recursive = SYS_MUTEX_ATTR_RECURSIVE;
    if (sysMutexCreate(&s_lock, &ma) == 0) s_ready = 1;
    else LOG_ERROR(LOG_MEM, "dev_lock: could not create the device mutex");
}

/* Wedge diagnostics: who holds the device lock, and how deep. The owner tag
 * is written only while the lock is held, so a stale read is at worst a
 * moment old -- good enough to name a deadlock's parties from the rescue
 * listener. */
volatile const char *g_devlock_owner;
volatile int         g_devlock_depth;

void dev_lock(void)
{
    if (!s_ready) return;
    sysMutexLock(s_lock, 0);
    g_devlock_depth++;
}
void dev_lock_tag(const char *who)
{
    g_devlock_owner = who;
}
void dev_unlock(void)
{
    if (!s_ready) return;
    if (--g_devlock_depth == 0) g_devlock_owner = 0;
    sysMutexUnlock(s_lock);
}

#else
void dev_lock_init(void) { }
void dev_lock(void)      { }
void dev_lock_tag(const char *who) { (void)who; }
void dev_unlock(void)    { }
#endif
