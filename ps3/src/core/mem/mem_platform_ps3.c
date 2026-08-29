/* mem_platform_ps3.c — arena backend on lv2.
 *
 * The constraint that shapes this file: PS3 PPU user virtual addresses are
 * 32-bit (`sys_mem_addr_t` is a u32), so the entire process address space is
 * 4 GiB and is already shared with the executable, stack, heap and RSX
 * mappings. We reserve 1 GiB of it and rely on address folding to make the
 * guest's mirrors land inside that window (memmap.h, ARCHITECTURE.md §3.2).
 *
 * lv2 separates *reserving address space* from *allocating memory*:
 *   sysMMapperAllocateAddress  — reserve a virtual range
 *   sysMMapperAllocateMemory   — create a physical memory object (mem_id)
 *   sysMMapperMapMemory        — bind an object into the reserved range
 * Anything inside the reservation we never bind stays unmapped and faults,
 * which is exactly how MMIO and wild guest pointers get caught.
 */
#include "mem_platform.h"
#include "../../common/log.h"

#if defined(__PS3__) || defined(__lv2ppu__)

#include <sys/memory.h>
#include <stdlib.h>
#include <string.h>

#define PS3_MAX_MAPPED 8

typedef struct {
    sys_mem_addr_t arena;
    u64            size;
    struct {
        sys_mem_id_t   id;
        sys_mem_addr_t addr;
        u32            size;
    } mapped[PS3_MAX_MAPPED];
    unsigned count;
} Ps3Arena;

/* Every region is backed by 1 MiB pages, and not only because large pages are
 * desirable — though they are: the PPE's TLB is small and a miss costs ~120 ns
 * on top of the access, so covering 88 MiB of guest RAM with 1 MiB pages rather
 * than 64 KiB ones cuts translations by 16x and keeps the guest's working set
 * resident.
 *
 * The binding constraint is stricter than preference. A reservation made with
 * SYS_MEMORY_PAGE_SIZE_1M is a 1 MiB-page region, and lv2 refuses
 * (CELL_EINVAL) to map a 64 KiB-page memory object into it. The locked cache is
 * 16 KiB, so the "obvious" choice of a 64 KiB page for it fails at map time —
 * after the two large regions have already succeeded, which makes it look like
 * a locked-cache problem rather than a page-size one. Rounding it to a full
 * 1 MiB costs under 1 MiB of a 213 MiB budget and removes the whole class. */
#define PS3_PAGE_GRANULE 0x100000u

static u32 round_to_page(u32 size)
{
    return (size + PS3_PAGE_GRANULE - 1u) & ~(PS3_PAGE_GRANULE - 1u);
}

int memplat_init(MemArena *a, u64 arena_size)
{
    Ps3Arena *p;
    sys_mem_addr_t addr = 0;
    s32 rc;

    p = (Ps3Arena *)calloc(1, sizeof *p);
    if (!p)
        return -1;

    /* Reserve address space only — no physical memory is consumed here.
     *
     * lv2 hands out address space in 256 MiB units and will only align a
     * reservation to a multiple of that unit: asking for 1 MiB alignment, which
     * looks entirely reasonable and matches the page size we then map with,
     * is rejected outright with CELL_EALIGN. That distinction cost a boot, so
     * the accepted alignments are tried explicitly and in order of preference
     * rather than assumed.
     *
     * 0 means "wherever it fits", which is the most likely to succeed on a
     * console whose address space is already carved up by the loader, the heap
     * and RSX -- so it is the last resort, not the first choice. */
    {
        static const size_t k_alignments[] = {
            0x40000000u,   /* 1 GiB: arena base is then fold-aligned  */
            0x10000000u,   /* 256 MiB: lv2's allocation granule       */
            0u             /* anywhere lv2 can find room              */
        };
        unsigned i;
        rc = -1;
        for (i = 0; i < sizeof k_alignments / sizeof k_alignments[0]; i++) {
            rc = sysMMapperAllocateAddress((size_t)arena_size,
                                           SYS_MEMORY_PAGE_SIZE_1M,
                                           k_alignments[i], &addr);
            if (rc == 0) {
                LOG_DEBUG(LOG_MEM, "reservation accepted at alignment %u MiB",
                          (unsigned)(k_alignments[i] >> 20));
                break;
            }
            LOG_DEBUG(LOG_MEM, "alignment %u MiB rejected: %08x",
                      (unsigned)(k_alignments[i] >> 20), (unsigned)rc);
        }
    }
    if (rc != 0) {
        LOG_ERROR(LOG_MEM, "sysMMapperAllocateAddress(%llu MiB) failed: %08x",
                  (unsigned long long)(arena_size >> 20), (unsigned)rc);
        LOG_ERROR(LOG_MEM, "falling back to software address translation");
        free(p);
        return -1;
    }

    p->arena = addr;
    p->size  = arena_size;

    a->base       = (u8 *)(size_t)addr;
    a->platform   = p;
    a->fastmem_ok = 1;

    LOG_INFO(LOG_MEM, "reserved %llu MiB of address space at %08x",
             (unsigned long long)(arena_size >> 20), (unsigned)addr);
    return 0;
}

int memplat_map_region(MemArena *a, const char *name, u32 size,
                       u8 **out_ptr, u32 fold_offset)
{
    Ps3Arena *p = (Ps3Arena *)a->platform;
    sys_mem_id_t id = 0;
    sys_mem_addr_t at;
    u32 alloc_size;
    s32 rc;

    if (!p || p->count >= PS3_MAX_MAPPED)
        return -1;

    alloc_size = round_to_page(size);

    rc = sysMMapperAllocateMemory((size_t)alloc_size, SYS_MEMORY_PAGE_SIZE_1M,
                                  &id);
    if (rc != 0) {
        LOG_ERROR(LOG_MEM, "sysMMapperAllocateMemory(%s, %u KiB) failed: %08x",
                  name, alloc_size >> 10, (unsigned)rc);
        return -1;
    }

    at = p->arena + fold_offset;
    rc = sysMMapperMapMemory(at, id, SYS_MEMORY_PROT_READ_WRITE);
    if (rc != 0) {
        LOG_ERROR(LOG_MEM, "sysMMapperMapMemory(%s @ %08x) failed: %08x",
                  name, (unsigned)at, (unsigned)rc);
        sysMMapperFreeMemory(id);
        return -1;
    }

    p->mapped[p->count].id   = id;
    p->mapped[p->count].addr = at;
    p->mapped[p->count].size = alloc_size;
    p->count++;

    *out_ptr = (u8 *)(size_t)at;
    LOG_DEBUG(LOG_MEM, "%-12s %7u KiB at %08x (%u KiB backed)", name, size >> 10,
              (unsigned)at, alloc_size >> 10);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Executable memory                                                    */
/* ------------------------------------------------------------------ */

typedef struct {
    sys_mem_addr_t addr;
    sys_mem_id_t   id;
} Ps3ExecBlock;

int memplat_alloc_executable(u64 size, u8 **out_ptr, void **out_handle)
{
    Ps3ExecBlock *b;
    sys_mem_addr_t addr = 0;
    sys_mem_id_t   id   = 0;
    u32 bytes = round_to_page((u32)size);
    s32 rc;

    b = (Ps3ExecBlock *)calloc(1, sizeof *b);
    if (!b)
        return -1;

    /* Reserved through the mmapper rather than taken from the process heap.
     * The heap is a single large region shared with every other allocation, so
     * a code cache carved out of it is indistinguishable from data to anything
     * inspecting the process -- and it inherits whatever page size sbrk chose.
     * A dedicated reservation is 1 MiB-paged, contiguous, and its lifetime is
     * ours. lv2 hands out address space in 256 MiB units regardless of how much
     * is asked for, so the reservation is that size and only `bytes` is backed. */
    rc = sysMMapperAllocateAddress(0x10000000u, SYS_MEMORY_PAGE_SIZE_1M,
                                   0x10000000u, &addr);
    if (rc != 0) {
        LOG_ERROR(LOG_JIT, "code cache: address reservation failed: %08x",
                  (unsigned)rc);
        free(b);
        return -1;
    }

    rc = sysMMapperAllocateMemory((size_t)bytes, SYS_MEMORY_PAGE_SIZE_1M, &id);
    if (rc != 0) {
        LOG_ERROR(LOG_JIT, "code cache: %u KiB allocation failed: %08x",
                  bytes >> 10, (unsigned)rc);
        sysMMapperFreeAddress(addr);
        free(b);
        return -1;
    }

    rc = sysMMapperMapMemory(addr, id, SYS_MEMORY_PROT_READ_WRITE);
    if (rc != 0) {
        LOG_ERROR(LOG_JIT, "code cache: map at %08x failed: %08x",
                  (unsigned)addr, (unsigned)rc);
        sysMMapperFreeMemory(id);
        sysMMapperFreeAddress(addr);
        free(b);
        return -1;
    }

    b->addr = addr;
    b->id   = id;
    *out_ptr    = (u8 *)(size_t)addr;
    *out_handle = b;

    LOG_INFO(LOG_JIT, "code cache: %u KiB at %08x", bytes >> 10, (unsigned)addr);
    return 0;
}

void memplat_free_executable(void *handle)
{
    Ps3ExecBlock *b = (Ps3ExecBlock *)handle;
    sys_mem_id_t id;

    if (!b)
        return;
    id = b->id;
    sysMMapperUnmapMemory(b->addr, &id);
    sysMMapperFreeMemory(b->id);
    sysMMapperFreeAddress(b->addr);
    free(b);
}

void memplat_shutdown(MemArena *a)
{
    Ps3Arena *p = (Ps3Arena *)a->platform;
    unsigned i;

    if (!p)
        return;

    for (i = 0; i < p->count; i++) {
        sys_mem_id_t id = p->mapped[i].id;
        sysMMapperUnmapMemory(p->mapped[i].addr, &id);
        sysMMapperFreeMemory(p->mapped[i].id);
    }
    if (p->arena)
        sysMMapperFreeAddress(p->arena);

    free(p);
    a->platform = NULL;
    a->base = NULL;
    a->fastmem_ok = 0;
}

#endif /* __PS3__ */
