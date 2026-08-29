/* mem_platform.h — the small OS-specific surface of the memory system.
 *
 * Two backends implement this: lv2 (`mem_platform_ps3.c`) for the console, and
 * POSIX (`mem_platform_posix.c`) for the workstation build that runs the
 * verification harness. Keeping the surface this narrow is what lets the
 * differential tests exercise the *same* memory code that runs on hardware.
 *
 * Note there is no aliasing primitive here. Earlier designs needed one to map
 * MEM1 into its cached and uncached mirrors; address folding
 * (memmap.h, ARCHITECTURE.md §3.2) produces those mirrors arithmetically, so
 * each region is mapped exactly once and the backend stays simple.
 */
#ifndef DOLPHIN_CORE_MEM_PLATFORM_H
#define DOLPHIN_CORE_MEM_PLATFORM_H

#include "memmap.h"

/* Reserve `arena_size` bytes of contiguous *address space* with no backing
 * pages and no access rights. Sets a->base and a->fastmem_ok on success.
 * Returns 0 on success, non-zero if the reservation could not be made. */
int memplat_init(MemArena *a, u64 arena_size);

/* Commit `size` bytes of read/write memory at `fold_offset` within the arena
 * and store the resulting host pointer in *out_ptr. */
int memplat_map_region(MemArena *a, const char *name, u32 size,
                       u8 **out_ptr, u32 fold_offset);

/* Release every mapping and the reservation itself. Safe to call on a
 * partially-initialized arena, which is what the failure path in mem_init
 * relies on. */
void memplat_shutdown(MemArena *a);

#if defined(__PS3__) || defined(__lv2ppu__)
/* Memory the JIT writes instructions into and then jumps to.
 *
 * lv2 has no execute permission bit -- user pages are executable -- so on the
 * console this is simply "memory". It is separated out anyway because the
 * *mechanism* matters to anything watching from outside: sysMemoryAllocate and
 * the mmapper reserve/allocate/map sequence produce identically usable pages on
 * hardware but are distinguishable to an emulator running us, and the code
 * cache is the one allocation whose contents get executed.
 *
 * Returns 0 on success. *out_handle receives an opaque token for the matching
 * free; it is meaningless to the caller. */
int  memplat_alloc_executable(u64 size, u8 **out_ptr, void **out_handle);
void memplat_free_executable(void *handle);
#endif

#endif /* DOLPHIN_CORE_MEM_PLATFORM_H */
