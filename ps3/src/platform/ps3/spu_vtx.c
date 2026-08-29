/* spu_vtx.c — PPU side of the SPU vertex-decode pipeline.
 *
 * Owns the SPU thread running vtx_spu.elf, the job ring in main memory, and
 * the vertex arena the SPU writes into. The arena is main memory mapped for
 * the RSX (gcmMapMainMemory -> GCM_LOCATION_CELL), because an SPU cannot DMA
 * into RSX local memory: the GPU fetches SPU-produced vertices from XDR.
 *
 * SAFETY: every wait on the SPU is bounded. An unbounded spin here once
 * turned a stalled SPU into a console that no longer serviced devlink, FTP
 * or webMAN and needed a physical power-cycle. On timeout the fast path is
 * disabled for the session and everything falls back to PPU decoding.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <malloc.h>
#include <sys/spu.h>
#include <rsx/rsx.h>
#include "../../common/log.h"
#include "../../core/mem/memmap.h"
#include "../../core/gx/gx.h"
#include "../../video/rsx/spu_vtx_shared.h"
#include "../../common/phase_prof.h"

#define SPU_ARENA_BYTES (16u * 1024u * 1024u)
#define RING_BYTES      (SPU_OFF_SLOTS + SPU_VTX_RING * sizeof(SpuVtxJobSlot))
#define SPIN_LIMIT      20000000u

static u8   *s_ring;              /* raw, 128-aligned; see shared header */
/* Last observed SPU consumer position. Advisory only: re-read on a near-miss.
 * DONE is written by the SPU, so reading it is a coherent load of a line this
 * core does not own -- an L2 miss on the critical path of every draw. */
static u32   s_done_cache;
static u32   s_nspu = 1;          /* consumers actually started */

/* How many SPUs to run as vertex consumers. The console has six; this used
 * one. wiicompiled-spucount.txt overrides, which is how a bad count is backed out
 * without a rebuild. */
static u32 spu_count_wanted(void)
{
    FILE *f = fopen("/dev_hdd0/tmp/wiicompiled-spucount.txt", "r");
    long v = 5;                   /* leave one SPU for the system */
    if (f) { if (fscanf(f, "%ld", &v) != 1) v = 5; fclose(f); }
    if (v < 1) v = 1;
    if (v > SPU_VTX_MAX_SPU) v = SPU_VTX_MAX_SPU;
    return (u32)v;
}

static u8   *s_arena;             /* main memory, RSX-mapped */
static u32   s_arena_off;         /* RSX IO offset of the arena */
static u32   s_arena_used;
static int   s_on;
static u32   s_grp;
static sysSpuImage s_img;

u64 g_spu_jobs, g_spu_fallbacks, g_spu_join_timeouts, g_spu_too_big;

static volatile u32 *ring_word(u32 off)
{ return (volatile u32 *)(s_ring + off); }

/* The ring is drained when EVERY consumer has passed `h`. Each consumer's DONE
 * counts in the global index space (it starts at its id and steps by the
 * consumer count), so "all jobs below h are finished" is exactly "every DONE
 * is at or past h". */
static u32 spu_done_min(void)
{
    u32 i, m = *ring_word(SPU_OFF_DONE);
    for (i = 1; i < s_nspu; i++) {
        u32 d = *(volatile u32 *)(s_ring + SPU_OFF_DONE + i * SPU_LINE);
        if ((s32)(d - m) < 0) m = d;
    }
    return m;
}

int  spu_vtx_active(void)      { return s_on; }
void spu_vtx_arena_reset(void) { s_arena_used = 0; }

void spu_vtx_init(void)
{
    FILE *f = fopen("/dev_hdd0/tmp/wiicompiled-spu.txt", "rb");
    if (!f) { LOG_INFO(LOG_CORE, "spu_vtx: flag absent, SPU path off"); return; }
    fclose(f);
    LOG_INFO(LOG_CORE, "spu_vtx: initialising");

    s_ring = (u8 *)memalign(128, RING_BYTES);
    if (!s_ring) return;
    memset(s_ring, 0, RING_BYTES);

    s_arena = (u8 *)memalign(1024 * 1024, SPU_ARENA_BYTES);
    if (!s_arena) return;
    if (gcmMapMainMemory(s_arena, SPU_ARENA_BYTES, &s_arena_off) != 0) {
        LOG_WARN(LOG_CORE, "spu_vtx: gcmMapMainMemory failed");
        return;
    }

    sysSpuInitialize(6, 0);
    {
        static u8 elf[96 * 1024];
        FILE *ef = fopen("/dev_hdd0/game/WCPS3001/USRDIR/vtx_spu.elf", "rb");
        size_t n;
        u32 thr;
        sysSpuThreadArgument arg;
        sysSpuThreadGroupAttribute gattr;
        sysSpuThreadAttribute tattr;
        static const char gname[] = "gxvtx";
        static const char tname[] = "vtxdec";
        int rc;

        if (!ef) { LOG_WARN(LOG_CORE, "spu_vtx: elf open failed"); return; }
        n = fread(elf, 1, sizeof elf, ef);
        fclose(ef);

        /* sysSpuImageImport returned nonsense segment counts on this
         * firmware; build the image from the ELF by hand instead. */
        {
            static sysSpuSegment segs[16];
            u32 entry = 0, nseg = 0;
            if (sysSpuElfGetInformation(elf, &entry, &nseg) != 0 ||
                nseg == 0 || nseg > 16) {
                LOG_WARN(LOG_CORE, "spu_vtx: elf info failed (n=%u)", (unsigned)n);
                return;
            }
            if (sysSpuElfGetSegments(elf, segs, nseg) != 0) {
                LOG_WARN(LOG_CORE, "spu_vtx: elf segments failed");
                return;
            }
            s_img.type = SPU_IMAGE_TYPE_USER;
            s_img.entryPoint = entry;
            s_img.segments = (u32)(uintptr_t)segs;
            s_img.segmentCount = nseg;
            LOG_INFO(LOG_CORE, "spu_vtx: image entry=%08x segs=%u", entry, nseg);
        }

        memset(&gattr, 0, sizeof gattr);
        gattr.nsize = sizeof gname; gattr.name = gname;
        s_nspu = spu_count_wanted();
        /* Tell the image how many consumers there are BEFORE the group runs:
         * each SPU reads this once at startup to size its stride. */
        *ring_word(SPU_OFF_NSPU) = s_nspu;
        __asm__ volatile("sync" ::: "memory");
        if ((rc = sysSpuThreadGroupCreate(&s_grp, s_nspu, 100, &gattr)) != 0) {
            LOG_WARN(LOG_CORE, "spu_vtx: group create rc=%08x", rc);
            return;
        }
        memset(&tattr, 0, sizeof tattr);
        tattr.nsize = sizeof tname; tattr.name = tname;
        tattr.option = SPU_THREAD_ATTR_NONE;
        {   u32 i;
            for (i = 0; i < s_nspu; i++) {
                memset(&arg, 0, sizeof arg);
                arg.arg0 = (u64)(uintptr_t)s_ring;
                arg.arg1 = (u64)i;          /* consumer index */
                if ((rc = sysSpuThreadInitialize(&thr, s_grp, i, &s_img,
                                                 &tattr, &arg)) != 0) {
                    LOG_WARN(LOG_CORE, "spu_vtx: thread %u init rc=%08x", i, rc);
                    return;
                }
            }
        }
        if ((rc = sysSpuThreadGroupStart(s_grp)) != 0) {
            LOG_WARN(LOG_CORE, "spu_vtx: group start rc=%08x", rc);
            return;
        }
    }

    /* Wait (bounded) for the SPU to prove it reached main() and can DMA. */
    {
        u32 spins = 0, i;
        for (i = 0; i < s_nspu; i++) {
            spins = 0;
            while (*(volatile u32 *)(s_ring + SPU_OFF_HB + i * SPU_LINE)
                   != 0xC0DE0001u) {
                if (++spins > SPIN_LIMIT) {
                    LOG_WARN(LOG_CORE,
                             "spu_vtx: consumer %u NO HEARTBEAT -- path off", i);
                    return;
                }
            }
        }
    }
    s_on = 1;
    LOG_INFO(LOG_CORE, "spu_vtx: ONLINE with %u consumer(s) "
             "(heartbeat ok, arena io=%08x)", s_nspu, s_arena_off);
}

/* How much PPU time the join actually costs, and how much of it is spinning.
 * This used to be charged to gx_fifo, because the call sites sit outside the
 * renderer's own profiled regions -- which made FIFO parsing look like a third
 * of the frame when much of that was the PPU waiting on the SPU. */
unsigned long long g_spu_join_calls;
unsigned long long g_spu_join_spun;    /* joins that spun at least once */
unsigned long long g_spu_join_spins;   /* total spin iterations         */

void spu_vtx_join(void)
{
    u32 spins = 0;
    if (!s_on) return;
    g_spu_join_calls++;
    prof_enter(PH_SPUWAIT);
    /* Spin politely.
     *
     * `or 1,1,1` is `cctpl` in the Cell handbook's nop table (§10.6.2): change
     * the current thread's priority to low. On a core whose two hardware
     * threads share dispatch, a busy-wait at normal priority takes issue slots
     * from whatever is doing real work -- today only this thread runs, so the
     * effect is small, but the moment anything else is scheduled on the
     * sibling this is the difference between a wait and a tax. Restored to
     * medium on the way out. */
    __asm__ __volatile__("or 1,1,1" ::: "memory");
    while ((s32)(spu_done_min() - *ring_word(SPU_OFF_HEAD)) < 0) {
        if (++spins > SPIN_LIMIT) {
            g_spu_join_spins += spins;
            if (spins) g_spu_join_spun++;
            prof_exit();
            g_spu_join_timeouts++;
            s_on = 0;
            LOG_WARN(LOG_CORE, "spu_vtx: JOIN TIMEOUT (head=%u done=%u) -- "
                     "SPU path disabled, PPU decode from here",
                     *ring_word(SPU_OFF_HEAD), *ring_word(SPU_OFF_DONE));
            *ring_word(SPU_OFF_HEAD) = spu_done_min();
            s_done_cache = spu_done_min();
            __asm__ __volatile__("or 2,2,2" ::: "memory");   /* cctpm */
            return;
        }
    }
    __asm__ __volatile__("or 2,2,2" ::: "memory");           /* cctpm */
    g_spu_join_spins += spins;
    if (spins) g_spu_join_spun++;
    prof_exit();
}

u8 *spu_vtx_reserve(u32 count, u32 *io_off_out)
{
    u32 bytes = (count * SPU_RV_BYTES + 127u) & ~127u;
    u32 at;
    if (!s_on || s_arena_used + bytes > SPU_ARENA_BYTES) return NULL;
    at = s_arena_used;
    s_arena_used += bytes;
    *io_off_out = s_arena_off + at;
    return s_arena + at;
}

int spu_vtx_submit(const SpuVtxJob *job)
{
    u32 h, spins = 0;
    if (!s_on) return -1;
    h = *ring_word(SPU_OFF_HEAD);
    /* Only re-read DONE when the CACHED value says the ring might be full.
     * DONE is written by the SPU, so every read of it is a coherent load of a
     * line this core does not own -- an L2 miss on the critical path of every
     * single draw. In a race that is 15,817 reads a frame to answer a question
     * whose answer is almost always "plenty of room". */
    if (h - s_done_cache >= SPU_VTX_RING)
        s_done_cache = spu_done_min();
    while (h - s_done_cache >= SPU_VTX_RING) {
        s_done_cache = spu_done_min();
        if (++spins > SPIN_LIMIT) {
            g_spu_join_timeouts++;
            s_on = 0;
            LOG_WARN(LOG_CORE, "spu_vtx: RING STALL -- SPU path disabled");
            *ring_word(SPU_OFF_HEAD) = spu_done_min();
            s_done_cache = spu_done_min();
            return -1;
        }
    }
    memcpy(s_ring + SPU_OFF_SLOTS +
           (h & (SPU_VTX_RING - 1u)) * sizeof(SpuVtxJobSlot),
           job, sizeof *job);
    /* `lwsync`, not `sync`.
     *
     * What this barrier has to guarantee is store-store: the job slot must be
     * visible before the HEAD that publishes it. lwsync orders exactly that
     * among cacheable, coherent stores, which is what the ring is -- while
     * `sync` is the heavyweight barrier that additionally drains the store
     * queue and waits for every outstanding operation. Cell's PPE charges
     * hundreds of cycles for that, and it was paid TWICE on every draw.
     *
     * The second `sync`, after the HEAD store, guarded nothing at all: no
     * store after it needs ordering against it, and the SPU's view of HEAD is
     * already ordered by the barrier above. It is gone. */
    __asm__ volatile("lwsync" ::: "memory");
    *ring_word(SPU_OFF_HEAD) = h + 1;
    g_spu_jobs++;
    return 0;
}

/* The SPU's own accounting, published into the heartbeat line every 256 jobs.
 * Word 0 stays the heartbeat so the online check is unaffected; the rest are
 * decrementer ticks (79.8 MHz timebase, ~12.53 ns each). */
void spu_vtx_spustat(u32 *jobs, u32 *poll, u32 *work, u32 *dma)
{
    if (!s_ring) { *jobs = *poll = *work = *dma = 0; return; }
    *jobs = *ring_word(SPU_OFF_HB + 4);
    *poll = *ring_word(SPU_OFF_HB + 8);
    *work = *ring_word(SPU_OFF_HB + 12);
    *dma  = *ring_word(SPU_OFF_HB + 16);
}

void spu_vtx_stat(void (*emit)(const char *fmt, ...))
{
    if (!s_ring) { emit("spu: no ring"); return; }
    emit("spu: on=%d head=%u done=%u hb=%08x jobs=%llu fb=%llu timeouts=%llu",
         s_on, *ring_word(SPU_OFF_HEAD), *ring_word(SPU_OFF_DONE),
         *ring_word(SPU_OFF_HB), (unsigned long long)g_spu_jobs,
         (unsigned long long)g_spu_fallbacks,
         (unsigned long long)g_spu_join_timeouts);
}

void spu_vtx_shutdown(void)
{
    if (!s_ring) return;
    *ring_word(SPU_OFF_QUIT) = 1;
    __asm__ volatile("sync" ::: "memory");
    s_on = 0;
}
