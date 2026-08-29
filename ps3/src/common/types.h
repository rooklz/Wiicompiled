/* types.h — fundamental types and host/guest endian accessors.
 *
 * The defining property of this port: guest (Broadway) and host (Cell PPE) are
 * both 32/64-bit *big-endian PowerPC*. Every accessor below therefore compiles
 * to a single load or store with no byte-reversal. On an x86 host the same
 * accessors would each cost an additional bswap; loads and stores are roughly
 * 30% of the dynamic guest instruction stream, so this is not a micro-detail.
 *
 * The swap paths are still written out, guarded by DOLPHIN_HOST_LITTLE_ENDIAN,
 * so host-side tools (the verification harness on a workstation) share exactly
 * this code and cannot drift from the target's semantics.
 */
#ifndef DOLPHIN_COMMON_TYPES_H
#define DOLPHIN_COMMON_TYPES_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Fixed-width types                                                    */
/*                                                                      */
/* Derived from <stdint.h> rather than spelled out as `unsigned long long` and  */
/* friends, because PSL1GHT's ppu-types.h declares the same short names from    */
/* the same source. On LP64 PowerPC, uint64_t is `unsigned long`, so hand-      */
/* written `unsigned long long` typedefs are a *different type* of the same     */
/* width and every translation unit that sees both headers fails to compile.    */
/* Matching stdint exactly makes the two declarations identical, which C        */
/* permits, and keeps us interoperable with the platform toolchain.                   */
/* ------------------------------------------------------------------ */

typedef uint8_t   u8;
typedef int8_t    s8;
typedef uint16_t  u16;
typedef int16_t   s16;
typedef uint32_t  u32;
typedef int32_t   s32;
typedef uint64_t  u64;
typedef int64_t   s64;

typedef float               f32;
typedef double              f64;

/* Guest effective address. Always held zero-extended in a 64-bit host
 * register so that (arena_base + ea) provably cannot escape the 4 GiB
 * fastmem window regardless of guest behaviour. See docs/ARCHITECTURE.md §3.2. */
typedef u32                 guest_addr_t;

/* ------------------------------------------------------------------ */
/* Compiler shims — kept GCC-4.x-clean; the ps3toolchain compiler is old.  */
/* ------------------------------------------------------------------ */

#define DOL_INLINE          static inline __attribute__((always_inline))
#define DOL_NOINLINE        __attribute__((noinline))
#define DOL_PURE            __attribute__((pure))
#define DOL_CONST           __attribute__((const))
#define DOL_UNUSED          __attribute__((unused))
#define DOL_PACKED          __attribute__((packed))
#define DOL_ALIGN(n)        __attribute__((aligned(n)))
#define DOL_HOT             __attribute__((hot))
#define DOL_COLD            __attribute__((cold))
#define DOL_NORETURN        __attribute__((noreturn))
#define DOL_PRINTF(f, a)    __attribute__((format(printf, f, a)))

#define LIKELY(x)           __builtin_expect(!!(x), 1)
#define UNLIKELY(x)         __builtin_expect(!!(x), 0)

/* Cell PPE cache line. Both L1 and L2 use 128-byte lines; the SPU DMA engine
 * also prefers 128-byte alignment for peak transfer rate, so this constant
 * governs both PPCState layout and every PPE<->SPU ring buffer. */
#define DOL_CACHELINE       128
#define DOL_CACHE_ALIGN     DOL_ALIGN(DOL_CACHELINE)

/* C11 _Static_assert is not reliably present in the toolchain's GCC. */
#define DOL_STATIC_ASSERT(cond, tag) \
    typedef char dol_static_assert_##tag[(cond) ? 1 : -1] DOL_UNUSED

#define DOL_ARRAY_COUNT(a)  (sizeof(a) / sizeof((a)[0]))

DOL_STATIC_ASSERT(sizeof(u8) == 1,  u8_size);
DOL_STATIC_ASSERT(sizeof(u16) == 2, u16_size);
DOL_STATIC_ASSERT(sizeof(u32) == 4, u32_size);
DOL_STATIC_ASSERT(sizeof(u64) == 8, u64_size);
DOL_STATIC_ASSERT(sizeof(f32) == 4, f32_size);
DOL_STATIC_ASSERT(sizeof(f64) == 8, f64_size);

/* ------------------------------------------------------------------ */
/* Host endianness                                                      */
/* ------------------------------------------------------------------ */

#if defined(__BIG_ENDIAN__) || \
    (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#  define DOLPHIN_HOST_BIG_ENDIAN     1
#  define DOLPHIN_HOST_LITTLE_ENDIAN  0
#else
#  define DOLPHIN_HOST_BIG_ENDIAN     0
#  define DOLPHIN_HOST_LITTLE_ENDIAN  1
#endif

#if DOLPHIN_HOST_BIG_ENDIAN
/* The target. Guest byte order == host byte order: every accessor is identity. */
#  define DOL_SWAP16(x)  (x)
#  define DOL_SWAP32(x)  (x)
#  define DOL_SWAP64(x)  (x)
#else
/* Workstation tooling only. GCC lowers these to a single rotate/bswap. */
#  define DOL_SWAP16(x)  __builtin_bswap16(x)
#  define DOL_SWAP32(x)  __builtin_bswap32(x)
#  define DOL_SWAP64(x)  __builtin_bswap64(x)
#endif

/* ------------------------------------------------------------------ */
/* Unaligned-safe raw memory access                                     */
/*                                                                      */
/* PowerPC hardware handles unaligned word/halfword loads natively, but we go   */
/* through memcpy so the same source is valid C on any host and so GCC can      */
/* still fold each one into a single instruction on PPC. Verified: at -O2 the   */
/* PPU compiler emits exactly `lwz`/`stw` for read32_raw/write32_raw.           */
/* ------------------------------------------------------------------ */

DOL_INLINE u16 dol_load16_raw(const void *p)
{
    u16 v; memcpy(&v, p, sizeof v); return v;
}
DOL_INLINE u32 dol_load32_raw(const void *p)
{
    u32 v; memcpy(&v, p, sizeof v); return v;
}
DOL_INLINE u64 dol_load64_raw(const void *p)
{
    u64 v; memcpy(&v, p, sizeof v); return v;
}
DOL_INLINE void dol_store16_raw(void *p, u16 v) { memcpy(p, &v, sizeof v); }
DOL_INLINE void dol_store32_raw(void *p, u32 v) { memcpy(p, &v, sizeof v); }
DOL_INLINE void dol_store64_raw(void *p, u64 v) { memcpy(p, &v, sizeof v); }

/* ------------------------------------------------------------------ */
/* Big-endian (guest-order) accessors                                   */
/*                                                                      */
/* These are the accessors the emulator core uses everywhere. On PS3 each is    */
/* one instruction. Naming them explicitly (rather than dereferencing pointers) */
/* keeps guest-order access auditable and lets the workstation build stay       */
/* bit-identical to the console build.                                          */
/* ------------------------------------------------------------------ */

DOL_INLINE u8  dol_be8 (const void *p) { return *(const u8 *)p; }
DOL_INLINE u16 dol_be16(const void *p) { return DOL_SWAP16(dol_load16_raw(p)); }
DOL_INLINE u32 dol_be32(const void *p) { return DOL_SWAP32(dol_load32_raw(p)); }
DOL_INLINE u64 dol_be64(const void *p) { return DOL_SWAP64(dol_load64_raw(p)); }

DOL_INLINE void dol_put_be8 (void *p, u8  v) { *(u8 *)p = v; }
DOL_INLINE void dol_put_be16(void *p, u16 v) { dol_store16_raw(p, DOL_SWAP16(v)); }
DOL_INLINE void dol_put_be32(void *p, u32 v) { dol_store32_raw(p, DOL_SWAP32(v)); }
DOL_INLINE void dol_put_be64(void *p, u64 v) { dol_store64_raw(p, DOL_SWAP64(v)); }

/* Reinterpretation between float and its bit pattern. A union through memcpy is
 * the only strict-aliasing-safe spelling; it compiles to zero instructions when
 * the value is already in the right register class, and to a single store/load
 * pair otherwise (PPC has no direct GPR<->FPR move before Power7). */
DOL_INLINE u32 dol_f32_bits(f32 v) { u32 b; memcpy(&b, &v, 4); return b; }
DOL_INLINE f32 dol_bits_f32(u32 b) { f32 v; memcpy(&v, &b, 4); return v; }
DOL_INLINE u64 dol_f64_bits(f64 v) { u64 b; memcpy(&b, &v, 8); return b; }
DOL_INLINE f64 dol_bits_f64(u64 b) { f64 v; memcpy(&v, &b, 8); return v; }

/* ------------------------------------------------------------------ */
/* Bit manipulation                                                     */
/* ------------------------------------------------------------------ */

/* PowerPC bit numbering is MSB-first: bit 0 is the most significant. Guest
 * instruction fields are specified that way in the Gekko manual, so the decoder
 * uses these helpers rather than open-coding shifts and inverting by hand. */
#define PPC_BIT32(n)            (0x80000000u >> (n))
#define PPC_BITS32(hi, lo)      ((0xFFFFFFFFu >> (hi)) & (0xFFFFFFFFu << (31 - (lo))))

/* Extract MSB-first bit range [hi, lo] (inclusive) from a 32-bit instruction. */
DOL_INLINE u32 dol_bits(u32 v, unsigned hi, unsigned lo)
{
    return (v >> (31 - lo)) & ((1u << (lo - hi + 1)) - 1);
}

DOL_INLINE u32 dol_rotl32(u32 v, unsigned n)
{
    return n ? ((v << n) | (v >> (32 - n))) : v;
}

/* Sign-extend the low `bits` of v. */
DOL_INLINE s32 dol_sext32(u32 v, unsigned bits)
{
    const unsigned sh = 32 - bits;
    return (s32)(v << sh) >> sh;
}

DOL_INLINE unsigned dol_clz32(u32 v) { return v ? (unsigned)__builtin_clz(v) : 32u; }
DOL_INLINE unsigned dol_popcount32(u32 v) { return (unsigned)__builtin_popcount(v); }

DOL_INLINE u32 dol_align_up32(u32 v, u32 a)   { return (v + (a - 1)) & ~(a - 1); }
DOL_INLINE u32 dol_align_down32(u32 v, u32 a) { return v & ~(a - 1); }
DOL_INLINE u64 dol_align_up64(u64 v, u64 a)   { return (v + (a - 1)) & ~(a - 1); }

DOL_INLINE u32 dol_min32(u32 a, u32 b) { return a < b ? a : b; }
DOL_INLINE u32 dol_max32(u32 a, u32 b) { return a > b ? a : b; }

#endif /* DOLPHIN_COMMON_TYPES_H */
