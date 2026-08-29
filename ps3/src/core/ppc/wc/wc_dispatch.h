/* wc_dispatch.h -- call-target traits for the statically recompiled game.
 *
 * WiiCompiled's emitter asks, at each call site, whether the target is known
 * statically (`KnownTranslatedCpuCall<T>::kAvailable`) so it can emit a direct
 * call, and whether a mod could republish it
 * (`kMustRemainDynamicallyDispatchable`). That machinery exists to let Retro
 * Rewind replace functions at run time.
 *
 * This port has no mods: every target is the base translation, resolved once by
 * tools/wc_gen_calls.py into an explicit `InvokeDirectCpu<T>` specialization
 * that calls `func_T` (or its native HLE override) directly. So the traits
 * report "not statically known, dispatch dynamically" and the emitted code
 * takes the InvokeDirectCpu path -- which is already a direct call. One generic
 * definition instead of 11,367 specializations, and no dispatch cost.
 */
#pragma once
#include "ppc_runtime.h"

inline constexpr uint32_t kPpcAllNonvolatileFprMask = 0xffffc000u;

template <uint32_t Target>
struct KnownTranslatedCpuCall {
    static constexpr bool     kAvailable = false;
    static constexpr uint32_t kNonvolatileFprWriteMask = kPpcAllNonvolatileFprMask;
    static constexpr bool     kMustRemainDynamicallyDispatchable = true;
    static constexpr void (*Entry)(CpuContext *) = nullptr;
};

/* Native (HLE) overrides are bound by address in gen/wc_calls.cpp rather than
 * through this trait, so the generated code never needs to know about them. */
template <uint32_t Target>
struct KnownNativeCpuCall {
    static constexpr bool     kAvailable = false;
    static constexpr uint32_t kNonvolatileFprWriteMask = kPpcAllNonvolatileFprMask;
    static constexpr void (*Entry)(CpuContext *) = nullptr;
};
template <uint32_t Target>
struct KnownTypedNativeCpuCall {
    static constexpr bool kAvailable = false;
};

/* No mod can republish a target here, so the base translation is always the
 * live one. */
template <uint32_t Target>
inline bool IsBaseTranslatedCpuTargetActive() { return true; }

/* A guest `bctr` without link: a tail call. Same resolution as a call. */
void InvokeIndirectCpu(uint32_t target, CpuContext *ctx);
inline void InvokeIndirectJump(uint32_t target, CpuContext *ctx) { InvokeIndirectCpu(target, ctx); }

/* GX command FIFO. Translated GX code builds display lists by writing through
 * these hooks; the port feeds them straight into the existing FIFO parser
 * (src/core/hw/gx_fifo.c), which is why the 139 GX entry points do not each
 * need a native reimplementation. */
extern "C" {
void GX_HLE_FIFO_Write8(uint8_t v);
void GX_HLE_FIFO_Write16(uint16_t v);
void GX_HLE_FIFO_Write32(uint32_t v);
void GX_HLE_FIFO_WriteFloat(float v);
void GX_HLE_FIFO_WriteBurst(const uint8_t *data, uint32_t sizeBytes);
}
