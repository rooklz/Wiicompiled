/* efb_copy.h — EFB copies that land in a texture rather than on the screen.
 *
 * A GX title ends its frame with an EFB copy whose `to_xfb` bit is set: the
 * embedded framebuffer goes to the external one and the video interface scans
 * it out. Every *other* EFB copy is a render-to-texture: the title has just
 * drawn something into a corner of the EFB and wants those pixels back as a
 * texture it can sample. Mario Kart Wii's licence-selection screen is built
 * that way -- each Mii head is rendered, copied to a texture, and then drawn
 * as a textured quad -- so a backend that ignores non-XFB copies samples
 * whatever happened to be at the destination address in guest memory, which is
 * the stale, never-written garbage the screen shows today.
 *
 * This file is the part of the answer that has nothing to do with any GPU: the
 * *registry* of copy destinations. It answers two questions, and it answers
 * them the same way on a workstation, under qemu and on the console:
 *
 *   1. Which bytes of guest memory would this copy have written?  That is
 *      pure address arithmetic over the destination format's tile geometry,
 *      and it is the thing that has to be right for question 2 to mean
 *      anything.
 *   2. Is the texture this draw is about to sample one of those destinations?
 *      If it is, the backend must bind the copied surface; decoding guest
 *      memory would decode bytes the copy never wrote.
 *
 * Keeping it separate from the RSX code is what makes the feature checkable
 * off hardware: tests/test_efbcopy.c exercises the arithmetic directly, and
 * the qemu audit harness (tests/efbaudit.c) drives the *same* registry from a
 * real Mario Kart Wii boot, so "which draws resolve to a copy target" is a
 * question that gets a real answer without a television.
 */
#ifndef DOLPHIN_VIDEO_RSX_EFB_COPY_H
#define DOLPHIN_VIDEO_RSX_EFB_COPY_H

#include "../../common/types.h"
#include "../../core/gx/bp.h"
#include "texture_decode.h"

/* Sixteen is generous for what titles actually do -- Mario Kart Wii's busiest
 * screen uses a handful -- and the table is scanned linearly on every texture
 * bind, so a large one would cost more than it stores. */
#define EFB_COPY_TARGETS 48   /* 16 thrashed in-race: 12 kart shadows + scene + effects per frame evicted the scene texture before it was sampled */

/* Guard words: how the registry notices that the CPU has overwritten a copy
 * destination.
 *
 * The texture cache deliberately does not watch guest memory, and neither can
 * this: there is no write barrier on the guest address space and adding one to
 * every store would cost more than the whole renderer. But a copy target is a
 * *specific*, known, small range, and the game only ever invalidates it by
 * writing over it -- so sampling a handful of words at copy time and checking
 * them again at bind time catches the overwrite for eight loads.
 *
 * What it catches: a title that DMAs new texture data over a buffer it once
 * used as a copy destination, or clears it, or reuses the allocation for
 * something else entirely. What it cannot catch: a write that happens to
 * leave all eight sampled words unchanged. Eight words spread across the
 * range make that vanishingly unlikely for real data and impossible to rule
 * out in principle, which is the honest description of the guarantee. */
#define EFB_COPY_GUARDS 8

typedef struct {
    u32 addr;           /* guest physical address the copy was aimed at     */
    u32 bytes;          /* extent it would have written, from the geometry  */
    u32 stride;         /* destination stride in bytes (BP_EFB_STRIDE << 5) */
    u16 width, height;  /* destination size in texels                       */
    u16 src_x, src_y;   /* EFB rectangle the pixels came from               */
    u8  fmt;            /* GX *texture* format the copy writes (see below)  */
    u8  has_alpha;      /* destination format carries alpha                 */
    u8  intensity;      /* copy asked for an intensity (luminance) format   */
    u8  valid;

    u64 copies;         /* times this destination has been written          */
    u64 binds;          /* times a draw sampled it                          */
    u32 serial;         /* bumped on every write: "did this change?"        */

    u32 guard[EFB_COPY_GUARDS];   /* guest words sampled when the copy ran  */
    u32 guard_at[EFB_COPY_GUARDS];/* the addresses they were sampled from   */
} EfbCopyTarget;

typedef struct {
    u64 copies_xfb;         /* copies that present a frame                  */
    u64 copies_texture;     /* copies that produce a texture                */
    u64 copies_unmodelled;  /* destination format with no tile geometry     */
    u64 evictions;          /* live targets thrown out to make room         */
    u64 binds_resolved;     /* texture binds served from a copy target      */
    u64 binds_stale;        /* binds that matched an address but no surface */
    u64 cpu_invalidations;  /* explicit invalidations from a guest write    */
    u64 guard_invalidations;/* targets dropped because the CPU overwrote them */
} EfbCopyStats;

extern EfbCopyStats g_efb_copy;

/* The GX texture format an EFB copy writes.
 *
 * The copy's format field is a *different enumeration* from the texture
 * format field -- it names channel selections (R8, G8, GB8 ...) rather than
 * texture encodings, and the `intensity` bit turns the first four into the
 * luminance formats. The two only agree on RGB565/RGB5A3/RGBA8, which is
 * exactly the set a render-to-texture effect uses, so getting the rest merely
 * approximately right is acceptable while getting those three wrong would not
 * be. Returns 0xFF for a copy format with no texture equivalent. */
u8 efb_copy_texture_format(unsigned copy_fmt, unsigned intensity);

/* Bytes in guest memory the copy writes: whole tiles, `stride` per row of
 * tiles, exactly as the hardware lays a texture out. Returns 0 if the
 * destination format has no modelled tile geometry, which the caller must
 * treat as "cannot track this copy" rather than as an empty one. */
u32 efb_copy_dest_bytes(const BPCopy *c);

/* Record a copy. Replaces the entry with the same destination address (a
 * second copy to the same place *is* a new image -- the licence screen
 * re-renders a Mii head into the same buffer whenever it changes), otherwise
 * takes a free slot, otherwise evicts the least recently written one.
 * Returns the entry, or NULL if the copy cannot be tracked at all -- an
 * unmodelled destination format, or a zero destination address. */
EfbCopyTarget *efb_copy_note(const BPCopy *c);

/* Find the target a texture bind should be served from. `addr` is the
 * texture's base address; width/height/fmt are what the title declared. An
 * exact address match wins; failing that a texture that starts *inside* a
 * copy target is still a hit, because a title may sample a sub-rectangle.
 * Returns NULL when the bind should go through the ordinary decode path. */
EfbCopyTarget *efb_copy_find(u32 addr, unsigned width, unsigned height,
                             unsigned fmt);

/* Drop every target overlapping [addr, addr+bytes). The cache does not watch
 * guest memory (see the note in efb_copy.c), so this is only called from
 * places that know a write happened. */
void efb_copy_invalidate_range(u32 addr, u32 bytes);

/* Drop a single entry (the backend calls this when it frees the surface). */
void efb_copy_drop(EfbCopyTarget *e);

/* Iteration, and the slot number of an entry.
 *
 * The GPU-side surface is deliberately NOT stored in the entry: it is held by
 * the backend in a parallel array indexed by this number, one allocation per
 * slot, grown when a slot's copy grows and never freed while the emulator
 * runs. That is not laziness either -- freeing RSX memory that a command
 * already in the ring may still be sampling is a class of bug with no
 * symptom short of a corrupted frame, and there are sixteen slots. */
EfbCopyTarget *efb_copy_entry(unsigned i);
unsigned efb_copy_index(const EfbCopyTarget *e);

void efb_copy_reset(void);

/* One line for the periodic console report. Returns `buf`. */
char *efb_copy_stats_line(char *buf, unsigned len);

#endif /* DOLPHIN_VIDEO_RSX_EFB_COPY_H */
