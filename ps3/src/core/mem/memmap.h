/* memmap.h — guest address space and the fastmem arena.
 *
 * The central trick (ARCHITECTURE.md §3.2): rather than translating every guest
 * address in software, we reserve a 4 GiB virtual window and map the console's
 * ~88 MiB of real memory into it at every address the guest can legitimately
 * name. A guest load then compiles to exactly one host instruction:
 *
 *     lwzx  rDst, r14, rAddr        ; r14 = arena base, rAddr = zero-extended EA
 *
 * Everything the guest must *not* touch directly — MMIO registers, unmapped
 * holes — is simply left unmapped, so it takes a page fault. The fault handler
 * decodes the instruction and dispatches to the device model. MMIO is rare and
 * intrinsically slow, so paying a fault there costs nothing measurable, while
 * the common case pays nothing at all.
 *
 * Two properties are worth stating as invariants rather than conventions:
 *
 *   - Guest EAs are held *zero-extended* in 64-bit host registers. Therefore
 *     (base + ea) is confined to [base, base + 4 GiB) for every possible guest
 *     value. A wild guest pointer cannot reach host memory. This is a safety
 *     property of the arithmetic, not of any bounds check we could forget.
 *   - MEM1 and MEM2 are *aliased* into their cached and uncached mirrors, not
 *     copied. A write through 0x8000_0000 is instantly visible at 0xC000_0000
 *     because they are the same physical pages, which is exactly the hardware's
 *     behaviour and costs us no coherency work.
 */
#ifndef DOLPHIN_CORE_MEM_MEMMAP_H
#define DOLPHIN_CORE_MEM_MEMMAP_H

#include "../../common/types.h"

/* ------------------------------------------------------------------ */
/* Guest memory geometry                                                */
/* ------------------------------------------------------------------ */

#define MEM1_SIZE       0x01800000u   /* 24 MiB — GameCube + Wii  */
#define MEM2_SIZE       0x04000000u   /* 64 MiB — Wii only        */
#define LOCKED_CACHE_SIZE 0x00004000u /* 16 KiB Gekko locked L1D  */

/* Physical (real-mode) bases. */
#define MEM1_PHYS_BASE  0x00000000u
#define MEM2_PHYS_BASE  0x10000000u
#define MMIO_PHYS_BASE  0x0C000000u

/* Effective-address mirrors established by the standard BAT configuration
 * every GameCube/Wii title sets up in its bootstrap. */
#define MEM1_CACHED     0x80000000u
#define MEM1_UNCACHED   0xC0000000u
#define MEM2_CACHED     0x90000000u
#define MEM2_UNCACHED   0xD0000000u
#define MMIO_CACHED     0xCC000000u
#define LOCKED_CACHE_EA 0xE0000000u

#define MMIO_SIZE       0x00010000u   /* 0xCC000000 .. 0xCC00FFFF */

/* The Wii adds a second register window: Hollywood, at physical 0x0D000000
 * (cached 0xCD000000), holding the IPC block that talks to IOS and the rest of
 * the Wii-only hardware. It is a separate window from the GameCube registers,
 * so it gets its own fold constant and normalisation. */
#define HOLLYWOOD_PHYS   0x0D000000u
#define HOLLYWOOD_CACHED 0xCD000000u
#define HOLLYWOOD_SIZE   0x00040000u   /* 0x0D000000 .. 0x0D03FFFF */

/* ------------------------------------------------------------------ */
/* Address folding                                                      */
/*                                                                      */
/* PS3 PPU user virtual addresses are 32-bit (`sys_mem_addr_t` is u32), so the  */
/* whole process address space is 4 GiB and a 4 GiB arena is impossible.        */
/* Folding the guest EA with 0x3FFFFFFF collapses it onto a 1 GiB window, which */
/* is *correct* rather than merely convenient: every pair of addresses that     */
/* collides is already an alias of the same physical memory on real hardware    */
/* (the cached/uncached mirrors), and MMIO folds onto its own physical base.    */
/* See docs/ARCHITECTURE.md §3.2.                                               */
/* ------------------------------------------------------------------ */

#define ARENA_MASK      0x3FFFFFFFu
#define ARENA_SIZE      0x40000000ull   /* 1 GiB of address space, ~88 MiB backed */

DOL_INLINE u32 mem_fold(u32 ea) { return ea & ARENA_MASK; }

/* Folded positions of each region — where they actually live in the arena. */
#define FOLD_MEM1       0x00000000u
#define FOLD_MMIO       0x0C000000u     /* == MMIO_PHYS_BASE, by construction */
#define FOLD_MEM2       0x10000000u
#define FOLD_LOCKED     0x20000000u

/* Highest byte the arena must cover. Everything above this folds nowhere useful
 * and must fault, so the reservation still spans the full mask range. */
#define ARENA_USED_TOP  (FOLD_LOCKED + LOCKED_CACHE_SIZE)

/* ------------------------------------------------------------------ */
/* Arena                                                                */
/* ------------------------------------------------------------------ */

typedef struct {
    u8 *base;           /* virtual base of the 4 GiB window                  */
    u8 *mem1;           /* direct pointer to MEM1 backing store              */
    u8 *mem2;           /* direct pointer to MEM2 backing store (NULL on GC) */
    u8 *locked_cache;   /* Gekko locked-cache backing store                  */
    int wii_mode;       /* MEM2 present                                      */
    int fastmem_ok;     /* arena reservation succeeded; JIT may use fastmem  */
    void *platform;     /* backend bookkeeping (fds, lv2 handles)            */
} MemArena;

extern MemArena g_mem;

/* Reserve the window and alias the backing stores into every mirror.
 * Returns 0 on success. On failure `fastmem_ok` is left 0 and the emulator
 * still runs correctly through the slow accessors below — this is the
 * documented fallback, not an error path we hope never happens. */
int  mem_init(int wii_mode);
void mem_shutdown(void);

/* Reset RAM contents without tearing down the mapping (title reboot). */
void mem_reset(void);

DOL_INLINE u8 *mem_base(void) { return g_mem.base; }

/* ------------------------------------------------------------------ */
/* MMIO device registration                                             */
/*                                                                      */
/* Devices claim subranges of the 0xCC00_0000 block. `size` is in bytes and the */
/* access width is passed explicitly because several GX/PE registers behave     */
/* differently for 8/16/32-bit access.                                          */
/* ------------------------------------------------------------------ */

typedef u32  (*MmioReadFn)(u32 addr, unsigned size, void *ctx);
typedef void (*MmioWriteFn)(u32 addr, u32 value, unsigned size, void *ctx);

typedef struct {
    u32         base;       /* EA of first register, within the MMIO block */
    u32         size;
    MmioReadFn  read;
    MmioWriteFn write;
    void       *ctx;
    const char *name;
} MmioRange;

#define MMIO_MAX_RANGES 32

int  mmio_register(u32 base, u32 size, MmioReadFn r, MmioWriteFn w,
                   void *ctx, const char *name);
void mmio_reset(void);

u32  mmio_read(u32 addr, unsigned size);
void mmio_write(u32 addr, u32 value, unsigned size);

/* ------------------------------------------------------------------ */
/* Address classification                                               */
/* ------------------------------------------------------------------ */

DOL_INLINE int mem_is_mmio(u32 ea)
{
    /* Folding maps both the 0xCC00_0000 mirror the OS uses and the
     * 0x0C00_0000 physical alias onto the same place, so one test covers
     * both spellings. */
    u32 f = mem_fold(ea);
    return (f - FOLD_MMIO) < MMIO_SIZE ||
           (f - HOLLYWOOD_PHYS) < HOLLYWOOD_SIZE ||
           /* 0x0D80_0000 is a mirror of the Wii MMIO block (Dolphin MMIO.h:59). */
           (f & 0xFFFF0000u) == 0x0D800000u;
}

/* True when the EA falls in a range the arena has real pages for, so the
 * JIT may emit a bare indexed access with no guard. */
int mem_is_ram(u32 ea);

/* ------------------------------------------------------------------ */
/* Slow-path accessors                                                  */
/*                                                                      */
/* Used by the interpreter, the debugger, device DMA and the fastmem fault      */
/* handler. The JIT does *not* call these on the hot path — it emits the single */
/* indexed instruction instead — so clarity is preferred to cleverness here.    */
/* ------------------------------------------------------------------ */

u8   mem_read8  (u32 ea);
u16  mem_read16 (u32 ea);
u32  mem_read32 (u32 ea);
u64  mem_read64 (u32 ea);

/* Instruction fetch. Kept distinct from a data read so that a jump into
 * unmapped memory is reported (and can raise ISI) rather than being
 * indistinguishable from a stray data access -- the two have different causes
 * and wildly different debugging stories. */
u32  mem_read32_for_fetch(u32 ea);

void mem_write8 (u32 ea, u8  v);
void mem_write16(u32 ea, u16 v);
void mem_write32(u32 ea, u32 v);
void mem_write64(u32 ea, u64 v);

/* Bulk transfer for DMA engines (DI, AI, DSP, GX FIFO). Handles crossing
 * between mirrors and refuses to run off the end of a region. */
void mem_read_block (u32 ea, void *dst, u32 len);
void mem_write_block(u32 ea, const void *src, u32 len);

/* Translate a guest EA to a host pointer, or NULL if the address has no
 * backing store (MMIO or a hole). Callers that hand the result to hardware
 * must respect the region length, which `mem_valid_span` reports. */
void *mem_ptr(u32 ea);
u32   mem_valid_span(u32 ea);

#endif /* DOLPHIN_CORE_MEM_MEMMAP_H */
