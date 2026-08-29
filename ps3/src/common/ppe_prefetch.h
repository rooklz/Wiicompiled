/* ppe_prefetch.h — drive the PPE's data-prefetch engine over a known span.
 *
 * From the Cell Broadband Engine Programming Handbook, §6.1.6:
 *
 *   "The PPE's data-prefetch engine (DPFE) implements the dcbt and dcbst
 *    instructions. The prefetch engine services all active streams
 *    simultaneously in a round-robin fashion."
 *
 * and it has eight entries, so up to eight line requests can be outstanding.
 * The VMX data-stream instructions are NOT an option here -- the same handbook
 * lists `dst dstt dstst dss dssall` as "not implemented ... treated as no-ops"
 * on this core, so `dcbt` is the whole mechanism.
 *
 * WHY THIS LIVES IN C AND NOT IN THE RECOMPILER. Emitting `dcbt` around guest
 * loads was tried twice and corrupted guest state both times, because every
 * scratch register at that point turned out to be live. Here the span is one
 * this emulator owns and is about to walk from end to end -- a FIFO window, a
 * texture -- so there is no guest state to get wrong and no register to
 * clobber. `dcbt` is architecturally a hint: it cannot fault and cannot change
 * results, only timing.
 *
 * Two limits from the same section are designed around below: prefetch is the
 * LOWEST priority queue in the load subunit by default (raising it needs
 * CIU_ModeSetup, which is privileged and unreachable from a game process), and
 * a stream is terminated at every 64 KB boundary regardless of page size.
 */
#ifndef DOLPHIN_COMMON_PPE_PREFETCH_H
#define DOLPHIN_COMMON_PPE_PREFETCH_H

#include <stddef.h>

#define PPE_LINE 128u          /* PPE cache line */
#define PPE_DPFE_ENTRIES 8u    /* outstanding line requests the engine holds */

/* Touch up to PPE_DPFE_ENTRIES lines covering [p, p+bytes), oldest first.
 * Asking for more than the engine can hold just evicts our own requests, so
 * the loop is capped rather than covering the whole span. */
extern int g_ppe_prefetch_off;   /* armed by /dev_hdd0/tmp/dolphin-nodpfe.txt */

static inline void ppe_prefetch_span(const void *p, size_t bytes)
{
    if (g_ppe_prefetch_off) return;
#if defined(__PPC__) || defined(__powerpc__) || defined(__PS3__) || defined(__lv2ppu__)
    const char *a = (const char *)p;
    size_t n = (bytes + PPE_LINE - 1u) / PPE_LINE;
    size_t i;
    if (n > PPE_DPFE_ENTRIES) n = PPE_DPFE_ENTRIES;
    for (i = 0; i < n; i++)
        __asm__ __volatile__("dcbt 0,%0" :: "r"(a + i * PPE_LINE) : "memory");
#else
    (void)p; (void)bytes;
#endif
}

#endif /* DOLPHIN_COMMON_PPE_PREFETCH_H */
