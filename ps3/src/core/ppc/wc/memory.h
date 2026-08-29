/* memory.h -- guest memory for statically recompiled MKWii code, PS3 edition.
 *
 * WiiCompiled reserves a flat 4 GiB window and lets page faults intercept MMIO;
 * lv2 offers neither, so this serves guest memory out of the emulator's fastmem
 * arena the way the JIT does: one base pointer, a fold mask that collapses the
 * cached/uncached mirrors and MEM2 onto the arena, and an MMIO test. The host
 * is big-endian, so a proven-RAM access is a mask and one load -- no byte swap
 * anywhere, which is the single biggest reason this guest suits this host.
 *
 * Every signature here matches WiiCompiled's runtime/include/memory_access.h
 * exactly, so the 11,367 translated functions compile UNMODIFIED. The arena
 * base is a global rather than a parameter for the same reason.
 */
#pragma once
#include <cstdint>
#include <cstring>
extern "C" {
#include "../../mem/memmap.h"
#include "../interp/interp_fputil.h"
}

/* The guest arena base, pinned in r14 for the whole port.
 *
 * Every guest memory access needs it, and as an ordinary global gcc has to
 * reload it after every store -- it cannot prove a store did not write the
 * pointer itself. In PSMTXConcat that showed up as 169 `ld` in a function whose
 * real work is a 4x4 multiply.
 *
 * r14 is callee-saved in the PowerPC ELF ABI, so anything the port calls --
 * libc, the system software, the emulator's own C -- is already obliged to preserve it.
 * Pinning it costs one register and removes a load from every access. This is
 * the same trick the JIT uses (H_MEMBASE), for the same reason.
 *
 * WC_PIN_ARENA is set by the build for the translated game's translation units.
 * Everything else sees a normal global, so code that merely wants the pointer
 * (the adapter, the data loader) is unaffected. */
#ifdef WC_PIN_ARENA
register uint8_t *g_wc_arena asm("r14");
#else
extern uint8_t *g_wc_arena;
#endif

namespace MemoryInline {

/* 64-bit fold so gcc emits one rldicl rather than rlwinm + clrldi. */
inline uint64_t Fold(uint32_t ea) { return static_cast<uint64_t>(ea) & UINT64_C(0x3FFFFFFF); }
/* Top byte 0xCC/0xCD (cached) or 0x0C/0x0D (physical): one rotate-and-mask. */
inline bool IsMmio(uint32_t ea) { return ((ea >> 24) & 0x3Eu) == 0x0Cu; }

template <typename T> inline T SlowLd(uint32_t a);
template <> inline uint8_t  SlowLd<uint8_t >(uint32_t a) { return mem_read8(a);  }
template <> inline uint16_t SlowLd<uint16_t>(uint32_t a) { return mem_read16(a); }
template <> inline uint32_t SlowLd<uint32_t>(uint32_t a) { return mem_read32(a); }
template <> inline uint64_t SlowLd<uint64_t>(uint32_t a) { return mem_read64(a); }
template <typename T> inline void SlowSt(uint32_t a, T v);
template <> inline void SlowSt<uint8_t >(uint32_t a, uint8_t  v) { mem_write8(a, v);  }
template <> inline void SlowSt<uint16_t>(uint32_t a, uint16_t v) { mem_write16(a, v); }
template <> inline void SlowSt<uint32_t>(uint32_t a, uint32_t v) { mem_write32(a, v); }
template <> inline void SlowSt<uint64_t>(uint32_t a, uint64_t v) { mem_write64(a, v); }

/* WC_SAFE_MEMORY routes every access through the emulator's checked accessors.
 *
 * The fast path indexes the arena with a folded address and no bounds test,
 * which is right on the console: the arena is a 1 GiB reservation and the fold
 * cannot leave it. It is wrong under the differential harness, where arguments
 * are manufactured and a guest pointer is as likely wild as valid -- there the
 * translated code walks off the committed pages and takes a host SIGSEGV while
 * the interpreter, which checks, returns zero and carries on. The harness would
 * be measuring the fastmem shortcut rather than the translation.
 *
 * So the harness builds checked, and compares semantics. The console build
 * keeps the fast path. */
#ifdef WC_SAFE_MEMORY
template <typename T> inline T Load(uint32_t a) { return SlowLd<T>(a); }
template <typename T> inline void Store(uint32_t a, T v) { SlowSt<T>(a, v); }
template <typename T> inline void StoreRam(uint32_t a, T v) { SlowSt<T>(a, v); }
#else
template <typename T> inline T Load(uint32_t a) {
    if (__builtin_expect(!IsMmio(a), 1)) { T v; std::memcpy(&v, g_wc_arena + Fold(a), sizeof v); return v; }
    return SlowLd<T>(a);
}
template <typename T> inline void Store(uint32_t a, T v) {
    if (__builtin_expect(!IsMmio(a), 1)) { std::memcpy(g_wc_arena + Fold(a), &v, sizeof v); return; }
    SlowSt<T>(a, v);
}
/* The translator proved these targets are RAM (stack, known objects): no test. */
template <typename T> inline void StoreRam(uint32_t a, T v) { std::memcpy(g_wc_arena + Fold(a), &v, sizeof v); }
#endif

inline float  F32(uint32_t b) { float f;  std::memcpy(&f, &b, 4); return f; }
inline double F64(uint64_t b) { double d; std::memcpy(&d, &b, 8); return d; }
inline uint64_t B64(double d) { uint64_t b; std::memcpy(&b, &d, 8); return b; }
/* stfs stores a double through the Gekko single conversion (interp_fputil.h). */
inline uint32_t SingleBits(double v) { uint64_t b; std::memcpy(&b, &v, 8); return ppc_convert_to_single(b); }

inline uint8_t  FlatRead8 (uint32_t a) { return Load<uint8_t >(a); }
inline uint16_t FlatRead16(uint32_t a) { return Load<uint16_t>(a); }
inline uint32_t FlatRead32(uint32_t a) { return Load<uint32_t>(a); }
inline uint64_t FlatRead64(uint32_t a) { return Load<uint64_t>(a); }
inline float    FlatReadFloat32(uint32_t a) { return F32(Load<uint32_t>(a)); }
inline double   FlatReadFloat64(uint32_t a) { return F64(Load<uint64_t>(a)); }
inline void FlatWrite8 (uint32_t a, uint8_t  v) { Store<uint8_t >(a, v); }
inline void FlatWrite16(uint32_t a, uint16_t v) { Store<uint16_t>(a, v); }
inline void FlatWrite32(uint32_t a, uint32_t v) { Store<uint32_t>(a, v); }
inline void FlatWrite64(uint32_t a, uint64_t v) { Store<uint64_t>(a, v); }
inline void FlatWriteFloat32(uint32_t a, double v) { Store<uint32_t>(a, SingleBits(v)); }
inline void FlatWriteFloat64(uint32_t a, double v) { Store<uint64_t>(a, B64(v)); }
inline void FlatWriteRam8 (uint32_t a, uint8_t  v) { StoreRam<uint8_t >(a, v); }
inline void FlatWriteRam16(uint32_t a, uint16_t v) { StoreRam<uint16_t>(a, v); }
inline void FlatWriteRam32(uint32_t a, uint32_t v) { StoreRam<uint32_t>(a, v); }
inline void FlatWriteRamFloat32(uint32_t a, double v) { StoreRam<uint32_t>(a, SingleBits(v)); }
inline void FlatWriteRamFloat64(uint32_t a, double v) { StoreRam<uint64_t>(a, B64(v)); }

/* ---- resolved tier ------------------------------------------------------
 * The translator proves a base register addresses one contiguous range and
 * resolves it once; every access in the range is then host + offset. Here
 * "resolved" means the range lies wholly inside one backed arena region and
 * clear of MMIO, so the per-access test disappears. */
struct ResolvedLoadPair { uint32_t first = 0, second = 0; bool valid = false; };

inline bool RangeInRam(uint64_t f0, uint64_t f1) {
    return (f0 - FOLD_MEM1 < MEM1_SIZE && f1 - FOLD_MEM1 < MEM1_SIZE) ||
           (g_mem.mem2 && f0 - FOLD_MEM2 < MEM2_SIZE && f1 - FOLD_MEM2 < MEM2_SIZE) ||
           (f0 - FOLD_LOCKED < LOCKED_CACHE_SIZE && f1 - FOLD_LOCKED < LOCKED_CACHE_SIZE);
}
inline uint8_t *ResolveRangeHost(uint32_t base, int32_t minOffset, uint32_t length,
                                 bool needsRead, bool needsWrite) {
    (void)needsRead; (void)needsWrite;
#ifdef WC_SAFE_MEMORY
    /* Refusing to resolve sends every access down the checked path, which is
     * what the harness wants; the resolved tier is an optimisation, not a
     * behaviour. */
    (void)base; (void)minOffset; (void)length;
    return nullptr;
#else
    const uint32_t start = base + static_cast<uint32_t>(minOffset);
    if (length == 0 || start > UINT32_MAX - (length - 1)) return nullptr;
    if (!RangeInRam(Fold(start), Fold(start + (length - 1)))) return nullptr;
    return g_wc_arena + Fold(start);
#endif
}
template <typename T> inline bool RdRes(uint8_t *h, uint32_t o, T &out) {
    if (!h) return false; std::memcpy(&out, h + o, sizeof out); return true;
}
template <typename T> inline bool WrRes(uint8_t *h, uint32_t o, T v) {
    if (!h) return false; std::memcpy(h + o, &v, sizeof v); return true;
}
inline uint8_t  ReadResolved8 (uint8_t *r, uint32_t o, uint32_t a) { uint8_t  v; return RdRes(r,o,v) ? v : SlowLd<uint8_t >(a); }
inline uint16_t ReadResolved16(uint8_t *r, uint32_t o, uint32_t a) { uint16_t v; return RdRes(r,o,v) ? v : SlowLd<uint16_t>(a); }
inline uint32_t ReadResolved32(uint8_t *r, uint32_t o, uint32_t a) { uint32_t v; return RdRes(r,o,v) ? v : SlowLd<uint32_t>(a); }
inline uint64_t ReadResolved64(uint8_t *r, uint32_t o, uint32_t a) { uint64_t v; return RdRes(r,o,v) ? v : SlowLd<uint64_t>(a); }
inline float    ReadResolvedFloat32(uint8_t *r, uint32_t o, uint32_t a) { return F32(ReadResolved32(r,o,a)); }
inline double   ReadResolvedFloat64(uint8_t *r, uint32_t o, uint32_t a) { return F64(ReadResolved64(r,o,a)); }
inline void WriteResolved8 (uint8_t *r, uint32_t o, uint32_t a, uint8_t  v) { if (!WrRes(r,o,v)) SlowSt<uint8_t >(a,v); }
inline void WriteResolved16(uint8_t *r, uint32_t o, uint32_t a, uint16_t v) { if (!WrRes(r,o,v)) SlowSt<uint16_t>(a,v); }
inline void WriteResolved32(uint8_t *r, uint32_t o, uint32_t a, uint32_t v) { if (!WrRes(r,o,v)) SlowSt<uint32_t>(a,v); }
inline void WriteResolved64(uint8_t *r, uint32_t o, uint32_t a, uint64_t v) { if (!WrRes(r,o,v)) SlowSt<uint64_t>(a,v); }
inline void WriteResolvedFloat32(uint8_t *r, uint32_t o, uint32_t a, double v) { WriteResolved32(r,o,a,SingleBits(v)); }
inline void WriteResolvedFloat64(uint8_t *r, uint32_t o, uint32_t a, double v) { WriteResolved64(r,o,a,B64(v)); }
inline ResolvedLoadPair ReadResolvedPair16(uint8_t *h, uint32_t o) {
    uint32_t p; if (!RdRes(h,o,p)) return {}; return { p >> 16, p & 0xFFFFu, true };
}
inline ResolvedLoadPair ReadResolvedPair32(uint8_t *h, uint32_t o) {
    uint64_t p; if (!RdRes(h,o,p)) return {}; return { static_cast<uint32_t>(p >> 32), static_cast<uint32_t>(p), true };
}
inline bool WriteResolvedPair16(uint8_t *h, uint32_t o, uint32_t packed) { return WrRes(h,o,packed); }
inline bool WriteResolvedPair32(uint8_t *h, uint32_t o, uint64_t packed) { return WrRes(h,o,packed); }

} /* namespace MemoryInline */

namespace Memory {
inline uint8_t  Read8 (uint32_t a) { return MemoryInline::FlatRead8(a);  }
inline uint16_t Read16(uint32_t a) { return MemoryInline::FlatRead16(a); }
inline uint32_t Read32(uint32_t a) { return MemoryInline::FlatRead32(a); }
inline uint64_t Read64(uint32_t a) { return MemoryInline::FlatRead64(a); }
inline void Write8 (uint32_t a, uint8_t  v) { MemoryInline::FlatWrite8(a, v);  }
inline void Write16(uint32_t a, uint16_t v) { MemoryInline::FlatWrite16(a, v); }
inline void Write32(uint32_t a, uint32_t v) { MemoryInline::FlatWrite32(a, v); }
inline void Write64(uint32_t a, uint64_t v) { MemoryInline::FlatWrite64(a, v); }
inline void WriteFloat32(uint32_t a, double v) { MemoryInline::FlatWriteFloat32(a, v); }
inline void WriteFloat64(uint32_t a, double v) { MemoryInline::FlatWriteFloat64(a, v); }
inline uint8_t *GetPointer(uint32_t a) { return g_wc_arena + MemoryInline::Fold(a); }
}
