/* libc_lock_fix.c — repairs a startup-order bug in the PSL1GHT/newlib runtime.
 *
 * This file is the reason the emulator now runs on a PS3 at all. Every launch
 * before it -- fake-signed, NPDRM-signed, packaged, unpackaged -- returned to
 * the XMB instantly with no output and no report file, because the process
 * died inside libc initialisation, before `main` and before the diagnostic
 * file could even be opened.
 *
 * The bug lives in newlib's lv2 port. Two constructors share priority 105:
 *
 *   newlib/libc/sys/lv2/lock_internal.c   init_metalock       (105)
 *   psl1ght/ppu/librt/globfile.c          __glob_file_init    (105)
 *
 * `init_metalock` creates the lwmutex that guards lazy allocation of every
 * other libc lock. `__glob_file_init` calls strdup("/"), which mallocs, which
 * takes newlib's malloc lock, which -- being statically zero-initialised --
 * goes down the lazy-allocation path and locks the metalock.
 *
 * Equal priorities are resolved by link order, and this toolchain's order puts
 * `__glob_file_init` first. `.ctors` is walked backwards by
 * __do_global_ctors_aux, so the observed execution order is:
 *
 *   sbrk_init(103) -> __syscalls_init -> __glob_file_init(105) -> init_metalock(105)
 *                                        ^^^^^^^^^^^^^^^^^^^^
 *                                        mallocs here, metalock still zeroed
 *
 * sys_lwmutex_lock on a zeroed lwmutex fails -- its sleep-queue id is 0, which
 * is not a live kernel object -- and __libc_auto_lock_allocate's response to
 * any failure is abort(). abort() is a silent death on a console.
 *
 * The fix is to take the metalock out of the picture. `__libc_auto_lock_allocate`
 * is a global symbol, so defining it here overrides the library's copy: the
 * linker resolves the reference against this object and never extracts
 * lock_internal.o, which also means its broken constructor is never registered.
 * Mutual exclusion comes from a compare-and-swap on a plain word, which needs
 * no constructor and is therefore correct from the first instruction of the
 * process.
 *
 * Verified against RPCS3: before, `sys_tty_write("Abort called.")` twelve
 * microseconds after sbrk_init; after, execution reaches main.
 */
#include <stdint.h>
#include <sys/lock.h>

/* Mirrors newlib's private lock_internal.h. Kept byte-identical so a lock this
 * file creates is indistinguishable from one the library would have created. */
#define SYS_LWMUTEX_ATTR_PROTOCOL   0x0002
#define SYS_LWMUTEX_ATTR_RECURSIVE  0x0010
#define SYS_LWMUTEX_UNINITIALIZED(p) (!((p)->attribute))

typedef struct _sys_lwmutex sys_lwmutex_t;

typedef struct _sys_lwmutex_attr {
    uint32_t attr_protocol;
    uint32_t attr_recursive;
    char     name[8];
} sys_lwmutex_attr_t;

extern int32_t sys_lwmutex_create(sys_lwmutex_t *lwmutex,
                                  const sys_lwmutex_attr_t *attr);

static const sys_lwmutex_attr_t k_libc_lock_attributes = {
    SYS_LWMUTEX_ATTR_PROTOCOL, SYS_LWMUTEX_ATTR_RECURSIVE, ""
};

/* A word, not a lock. Anything richer would need initialising, and the whole
 * point is to work before initialisers have run. */
static volatile uint32_t s_guard;

void __libc_auto_lock_allocate(sys_lwmutex_t *lwmutex);

void __libc_auto_lock_allocate(sys_lwmutex_t *lwmutex)
{
    while (!__sync_bool_compare_and_swap(&s_guard, 0u, 1u)) {
        /* `or 27,27,27` drops this hardware thread to low priority. The PPE is
         * 2-way SMT with shared issue, so a spinning thread that does not yield
         * priority steals issue slots from the thread holding the guard --
         * which on a 2-issue in-order core is the difference between a spin of
         * nanoseconds and one of microseconds. */
        __asm__ __volatile__ ("or 27,27,27" ::: "memory");
    }
    __asm__ __volatile__ ("or 2,2,2" ::: "memory");   /* back to normal priority */
    __sync_synchronize();

    if (SYS_LWMUTEX_UNINITIALIZED(lwmutex))
        (void)sys_lwmutex_create(lwmutex, &k_libc_lock_attributes);

    /* The created lock must be visible to whoever observes s_guard == 0 next. */
    __sync_synchronize();
    s_guard = 0u;
}
