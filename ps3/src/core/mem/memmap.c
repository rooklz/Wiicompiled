/* memmap.c — guest address space, MMIO dispatch and slow-path accessors.
 *
 * The fast path for guest memory is not here: the JIT emits a single indexed
 * load/store against the arena base and never calls into this file. What lives
 * here is everything that has to be *correct* rather than fast -- the
 * interpreter's accessors, device DMA and the debugger.
 *
 * Note that every accessor resolves through the region table and a backing
 * pointer rather than through base+EA. That is what allows the emulator to run
 * at all when the 1 GiB arena cannot be reserved: the regions become ordinary
 * allocations, fastmem and the JIT switch off, and everything else is
 * unaffected.
 */
#include "memmap.h"
#include "mem_platform.h"
#include "../../common/log.h"

#include <stdlib.h>
#include <string.h>
#include "../hw/hardware.h"   /* dev_lock: MMIO is cross-thread in the port */

MemArena g_mem;

/* ------------------------------------------------------------------ */
/* Region table                                                         */
/*                                                                      */
/* Every effective address the guest may legitimately name, and which backing   */
/* store it resolves to. The cached/uncached mirrors are separate entries but   */
/* share a backing pointer, so a write through one is visible through the other */
/* with no coherency work — which is precisely the hardware's behaviour.        */
/* ------------------------------------------------------------------ */

/* Regions are described by their *folded* position, which is why there are
 * three of them and not seven: the cached and uncached mirrors of MEM1 and
 * MEM2 fold onto their base region rather than needing separate entries or
 * separate mappings. Aliasing is performed by the fold, not by the mapper. */
typedef struct {
    u32   fold_base;
    u32   size;
    u8  **backing;      /* indirection: filled in after allocation */
    int   wii_only;
    const char *name;
} MemRegionDesc;

static const MemRegionDesc k_regions[] = {
    { FOLD_MEM1,   MEM1_SIZE,         &g_mem.mem1,         0, "MEM1"         },
    { FOLD_MEM2,   MEM2_SIZE,         &g_mem.mem2,         1, "MEM2"         },
    { FOLD_LOCKED, LOCKED_CACHE_SIZE, &g_mem.locked_cache, 0, "locked cache" },
};

#define REGION_COUNT DOL_ARRAY_COUNT(k_regions)

/* Resolve an EA to its region, or NULL. Linear scan over three entries beats a
 * tree or hash here, and this is the slow path regardless. */
static const MemRegionDesc *region_for(u32 ea)
{
    u32 f = mem_fold(ea);
    unsigned i;
    for (i = 0; i < REGION_COUNT; i++) {
        const MemRegionDesc *r = &k_regions[i];
        if (r->wii_only && !g_mem.wii_mode)
            continue;
        if (f - r->fold_base < r->size)     /* unsigned: also rejects f < base */
            return r;
    }
    return NULL;
}

int mem_is_ram(u32 ea)
{
    return region_for(ea) != NULL;
}

void *mem_ptr(u32 ea)
{
    const MemRegionDesc *r = region_for(ea);
    if (!r || !*r->backing)
        return NULL;
    return *r->backing + (mem_fold(ea) - r->fold_base);
}

u32 mem_valid_span(u32 ea)
{
    const MemRegionDesc *r = region_for(ea);
    if (!r)
        return 0;
    return r->size - (mem_fold(ea) - r->fold_base);
}

/* ------------------------------------------------------------------ */
/* MMIO                                                                 */
/* ------------------------------------------------------------------ */

static MmioRange s_mmio[MMIO_MAX_RANGES];
static unsigned  s_mmio_count;

int mmio_register(u32 base, u32 size, MmioReadFn r, MmioWriteFn w,
                  void *ctx, const char *name)
{
    if (s_mmio_count >= MMIO_MAX_RANGES) {
        LOG_ERROR(LOG_MEM, "MMIO table full; cannot register %s", name);
        return -1;
    }
    s_mmio[s_mmio_count].base  = base;
    s_mmio[s_mmio_count].size  = size;
    s_mmio[s_mmio_count].read  = r;
    s_mmio[s_mmio_count].write = w;
    s_mmio[s_mmio_count].ctx   = ctx;
    s_mmio[s_mmio_count].name  = name;
    s_mmio_count++;
    LOG_DEBUG(LOG_MEM, "MMIO %-10s %08x..%08x", name, base, base + size - 1);
    return 0;
}

void mmio_reset(void)
{
    s_mmio_count = 0;
    memset(s_mmio, 0, sizeof s_mmio);
}

/* MMIO is addressed through both 0xCC00_0000 and the 0x0C00_0000 physical
 * alias; normalize so devices only ever see one form. */
static u32 mmio_normalize(u32 addr)
{
    u32 f = mem_fold(addr);
    if ((f & 0xFFFF0000u) == 0x0D800000u)
        f = HOLLYWOOD_PHYS | (f & 0xFFFFu);     /* mirror -> the real block */
    if ((f - HOLLYWOOD_PHYS) < HOLLYWOOD_SIZE) {
        u32 off = f & (HOLLYWOOD_SIZE - 1);
        /* The Wii exposes the GameCube device block inside its own window too:
         * DI at +0x6000, SI at +0x6400, EXI at +0x6800, AI at +0x6C00. A title
         * reaches them through 0x0D006xxx as readily as through 0x0C006xxx, so
         * fold that part onto the devices already registered for the GameCube
         * window rather than leaving it unclaimed. */
        if (off >= 0x6000u && off < 0x7000u)
            return MMIO_CACHED | off;
        return HOLLYWOOD_CACHED | off;
    }
    return MMIO_CACHED | (addr & 0x0000FFFFu);
}

static const MmioRange *mmio_find(u32 addr)
{
    unsigned i;
    for (i = 0; i < s_mmio_count; i++)
        if (addr - s_mmio[i].base < s_mmio[i].size)
            return &s_mmio[i];
    return NULL;
}

u32 mmio_read(u32 addr, unsigned size)
{
    u32 a = mmio_normalize(addr);
    const MmioRange *r = mmio_find(a);
    if (!r || !r->read) {
        /* Unclaimed MMIO reads are common early in a title's life while it
         * probes for hardware; log once per address rather than per access so
         * the console stays readable. */
        LOG_WARN_ONCE(LOG_MEM, "unhandled MMIO read%u @ %08x", size * 8, a);
        return 0;
    }
    return r->read(a, size, r->ctx);
}

void mmio_write(u32 addr, u32 value, unsigned size)
{
    u32 a = mmio_normalize(addr);
    const MmioRange *r = mmio_find(a);
    if (!r || !r->write) {
        LOG_WARN_ONCE(LOG_MEM, "unhandled MMIO write%u @ %08x = %08x",
                      size * 8, a, value);
        return;
    }
    r->write(a, value, size, r->ctx);
}

/* ------------------------------------------------------------------ */
/* Slow-path accessors                                                  */
/*                                                                      */
/* RAM is checked before MMIO: RAM accesses vastly outnumber MMIO even on this  */
/* path, and the region scan is cheaper than the MMIO scan.                     */
/* ------------------------------------------------------------------ */

#define DEFINE_READ(bits, type, loader)                                     \
    type mem_read##bits(u32 ea)                                             \
    {                                                                       \
        void *p = mem_ptr(ea);                                              \
        if (LIKELY(p != NULL))                                              \
            return (type)loader(p);                                         \
        if (mem_is_mmio(ea)) {                                              \
            type v_;                                                        \
            dev_lock();                                                     \
            v_ = (type)mmio_read(ea, bits / 8);                             \
            dev_unlock();                                                   \
            return v_;                                                      \
        }                                                                   \
        LOG_WARN_ONCE(LOG_MEM, "read%u from unmapped %08x", bits, ea);       \
        return 0;                                                           \
    }

DEFINE_READ(8,  u8,  dol_be8)
DEFINE_READ(16, u16, dol_be16)
DEFINE_READ(32, u32, dol_be32)
DEFINE_READ(64, u64, dol_be64)
#undef DEFINE_READ

#define DEFINE_WRITE(bits, type, storer)                                    \
    void mem_write##bits(u32 ea, type v)                                    \
    {                                                                       \
        void *p = mem_ptr(ea);                                              \
        if (LIKELY(p != NULL)) { storer(p, v); return; }                    \
        if (mem_is_mmio(ea)) {                                              \
            dev_lock();                                                     \
            mmio_write(ea, (u32)v, bits / 8);                               \
            dev_unlock();                                                   \
            return;                                                         \
        }                                                                   \
        LOG_WARN_ONCE(LOG_MEM, "write%u to unmapped %08x", bits, ea);        \
    }

DEFINE_WRITE(8,  u8,  dol_put_be8)
DEFINE_WRITE(16, u16, dol_put_be16)
DEFINE_WRITE(32, u32, dol_put_be32)
DEFINE_WRITE(64, u64, dol_put_be64)
#undef DEFINE_WRITE

u32 mem_read32_for_fetch(u32 ea)
{
    void *p = mem_ptr(ea);
    if (LIKELY(p != NULL))
        return dol_be32(p);

    /* Returning 0 is not arbitrary: primary opcode 0 is an illegal
     * instruction, so the interpreter raises a program exception exactly as
     * hardware would when execution runs off into nothing. */
    LOG_WARN_ONCE(LOG_MEM, "instruction fetch from unmapped %08x", ea);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Block transfer                                                       */
/*                                                                      */
/* Device DMA can name a length that runs past the end of a region. Rather than */
/* faulting or wrapping, we clamp to the region and report it: a title doing    */
/* this is either buggy or relying on hardware behaviour we need to learn       */
/* about, and silently truncating would hide both.                              */
/* ------------------------------------------------------------------ */

void mem_read_block(u32 ea, void *dst, u32 len)
{
    u8 *out = (u8 *)dst;
    while (len) {
        u32 span = mem_valid_span(ea);
        void *p  = mem_ptr(ea);
        u32 n;
        if (!p || !span) {
            LOG_WARN_ONCE(LOG_MEM, "DMA read from unmapped %08x (%u bytes)", ea, len);
            memset(out, 0, len);
            return;
        }
        n = (len < span) ? len : span;
        memcpy(out, p, n);
        out += n; ea += n; len -= n;
    }
}

void mem_write_block(u32 ea, const void *src, u32 len)
{
    const u8 *in = (const u8 *)src;
    while (len) {
        u32 span = mem_valid_span(ea);
        void *p  = mem_ptr(ea);
        u32 n;
        if (!p || !span) {
            LOG_WARN_ONCE(LOG_MEM, "DMA write to unmapped %08x (%u bytes)", ea, len);
            return;
        }
        n = (len < span) ? len : span;
        memcpy(p, in, n);
        in += n; ea += n; len -= n;
    }
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                            */
/* ------------------------------------------------------------------ */

/* The emulator is still perfectly capable of running without an arena: every
 * slow-path accessor resolves through the region table and a backing pointer,
 * not through base+EA. So a machine that cannot give us the window gets plain
 * allocations rather than a refusal to start.
 *
 * What is lost is fastmem, and with it the JIT: compiled code addresses memory
 * as an offset from a pinned arena base, which no longer exists. jit_run
 * detects this and stays on the interpreter. Slower, correct, and it actually
 * boots, which beats an emulator that will not start on a machine with a
 * fragmented address space. */
static int mem_init_without_arena(int wii_mode)
{
    g_mem.base = NULL;
    g_mem.fastmem_ok = 0;

    g_mem.mem1 = (u8 *)calloc(1, MEM1_SIZE);
    g_mem.mem2 = wii_mode ? (u8 *)calloc(1, MEM2_SIZE) : NULL;
    g_mem.locked_cache = (u8 *)calloc(1, LOCKED_CACHE_SIZE);

    if (!g_mem.mem1 || (wii_mode && !g_mem.mem2) || !g_mem.locked_cache) {
        LOG_ERROR(LOG_MEM, "could not allocate guest RAM either");
        mem_shutdown();
        return -1;
    }

    mmio_reset();
    LOG_INFO(LOG_MEM, "guest RAM allocated without an arena: MEM1 %u MiB%s",
             MEM1_SIZE >> 20, wii_mode ? " + MEM2 64 MiB" : "");
    return 0;
}

int mem_init(int wii_mode)
{
    memset(&g_mem, 0, sizeof g_mem);
    g_mem.wii_mode = wii_mode;

    if (memplat_init(&g_mem, ARENA_SIZE) != 0) {
        LOG_WARN(LOG_MEM, "could not reserve the %llu MiB arena; "
                          "continuing without fastmem (interpreter only)",
                 (unsigned long long)(ARENA_SIZE >> 20));
        return mem_init_without_arena(wii_mode);
    }

    /* One mapping per region: the cached/uncached mirrors are produced by the
     * address fold, so they cost no additional address space. */
    if (memplat_map_region(&g_mem, "MEM1", MEM1_SIZE, &g_mem.mem1, FOLD_MEM1) != 0)
        goto fail;

    if (wii_mode &&
        memplat_map_region(&g_mem, "MEM2", MEM2_SIZE, &g_mem.mem2, FOLD_MEM2) != 0)
        goto fail;

    if (memplat_map_region(&g_mem, "LC", LOCKED_CACHE_SIZE, &g_mem.locked_cache,
                           FOLD_LOCKED) != 0)
        goto fail;

    mmio_reset();
    mem_reset();

    LOG_INFO(LOG_MEM, "arena at %p, MEM1 %u MiB%s, fastmem %s",
             (void *)g_mem.base, MEM1_SIZE >> 20,
             wii_mode ? " + MEM2 64 MiB" : "",
             g_mem.fastmem_ok ? "enabled" : "DISABLED (slow accessors)");
    return 0;

fail:
    /* Reserving the window succeeded but populating it did not. Give the
     * address space back and take the no-arena route, which is a working
     * emulator rather than no emulator.
     *
     * Clearing the region pointers is not tidiness. They are views into the
     * arena, never allocations, and memplat_shutdown has just cleared
     * fastmem_ok -- which is precisely the flag mem_shutdown consults to decide
     * whether to free() them. Left set, they become free() calls on addresses
     * malloc never issued, and the resulting fault lands nowhere near the
     * mapping failure that caused it. */
    memplat_shutdown(&g_mem);
    g_mem.mem1 = NULL;
    g_mem.mem2 = NULL;
    g_mem.locked_cache = NULL;

    LOG_WARN(LOG_MEM, "arena reserved but could not be populated; "
                      "continuing without fastmem (interpreter only)");
    return mem_init_without_arena(wii_mode);
}

void mem_reset(void)
{
    /* Hardware powers up with RAM in an indeterminate state, but titles
     * routinely depend on the IPL having zeroed it. Zeroing is also what makes
     * runs reproducible, which the differential harness requires. */
    if (g_mem.mem1) memset(g_mem.mem1, 0, MEM1_SIZE);
    if (g_mem.mem2) memset(g_mem.mem2, 0, MEM2_SIZE);
    if (g_mem.locked_cache) memset(g_mem.locked_cache, 0, LOCKED_CACHE_SIZE);
}

void mem_shutdown(void)
{
    if (!g_mem.fastmem_ok) {
        /* Fallback path: the regions are ordinary allocations rather than
         * views into a reserved window. */
        free(g_mem.mem1);
        free(g_mem.mem2);
        free(g_mem.locked_cache);
    }
    memplat_shutdown(&g_mem);
    memset(&g_mem, 0, sizeof g_mem);
}
