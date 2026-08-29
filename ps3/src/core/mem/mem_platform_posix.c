/* mem_platform_posix.c — arena backend for the workstation build.
 *
 * This is not a toy. The verification harness (tools/) runs the real emulator
 * core on a workstation to differentially test the JIT against the interpreter,
 * so this backend must produce an address space with the same shape as the
 * console's — same fold, same regions, same holes that must fault.
 */
#include "mem_platform.h"
#include "../../common/log.h"

#if !defined(__PS3__) && !defined(__lv2ppu__)

#include <sys/mman.h>
#include <stdlib.h>
#include <string.h>

#ifndef MAP_NORESERVE
#  define MAP_NORESERVE 0        /* macOS does not define it; it is advisory */
#endif
#ifndef MAP_ANONYMOUS
#  define MAP_ANONYMOUS MAP_ANON
#endif

typedef struct {
    u64 size;
} PosixArena;

int memplat_init(MemArena *a, u64 arena_size)
{
    PosixArena *p;
    void *base;

    /* PROT_NONE so that every address we have not explicitly committed faults.
     * That is the whole mechanism by which MMIO and wild pointers are caught. */
    base = mmap(NULL, (size_t)arena_size, PROT_NONE,
                MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if (base == MAP_FAILED) {
        LOG_ERROR(LOG_MEM, "mmap of %llu MiB arena failed",
                  (unsigned long long)(arena_size >> 20));
        return -1;
    }

    p = (PosixArena *)calloc(1, sizeof *p);
    if (!p) {
        munmap(base, (size_t)arena_size);
        return -1;
    }
    p->size = arena_size;

    a->base        = (u8 *)base;
    a->platform    = p;
    a->fastmem_ok  = 1;
    return 0;
}

int memplat_map_region(MemArena *a, const char *name, u32 size,
                       u8 **out_ptr, u32 fold_offset)
{
    void *want = a->base + fold_offset;
    void *got  = mmap(want, size, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    if (got == MAP_FAILED || got != want) {
        LOG_ERROR(LOG_MEM, "could not commit %s (%u MiB) at arena+%08x",
                  name, size >> 20, fold_offset);
        return -1;
    }
    *out_ptr = (u8 *)got;
    LOG_DEBUG(LOG_MEM, "%-12s %7u KiB at arena+%08x", name, size >> 10, fold_offset);
    return 0;
}

void memplat_shutdown(MemArena *a)
{
    PosixArena *p = (PosixArena *)a->platform;
    if (p) {
        if (a->base)
            munmap(a->base, (size_t)p->size);
        free(p);
    }
    a->platform = NULL;
    a->base = NULL;
    a->fastmem_ok = 0;
}

#endif /* !__PS3__ */
