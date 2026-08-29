/* vtx_spu.c — SPU vertex decoder.
 *
 * Decodes GX vertex streams into RenderVertex records, replacing the PPU's
 * per-vertex C loop (measured at 26-35% of an in-race frame). One SPU thread
 * polls the job ring in main memory, DMAs in the stream (and, for indexed
 * attributes, a bounded window of the array), decodes into LS, and DMAs
 * RenderVertex records back out to the RSX-mapped arena.
 *
 * Every PPU<->SPU field lives on its own 128-byte line (see
 * spu_vtx_shared.h): whole-line DMAs are MFC-legal at any size and the two
 * processors never contend for a line.
 *
 * LS budget (256KB total): 64K stream + 64K window + 44K output + ~10K code
 * and statics, leaving ~70K of stack. */
#include <spu_intrinsics.h>
#include <spu_mfcio.h>
#include <sys/spu_thread.h>
#include <stdint.h>
#include "src/video/rsx/spu_vtx_shared.h"

#define TAG_IN   0
#define TAG_OUT  1
#define TAG_DONE 2

static uint8_t  line_in[SPU_LINE]   __attribute__((aligned(128)));
static uint8_t  line_out[SPU_LINE]  __attribute__((aligned(128)));
static SpuVtxJobSlot jobslot        __attribute__((aligned(128)));
static uint8_t  stream_buf[64*1024] __attribute__((aligned(128)));
static uint8_t  window_buf[SPU_VTX_MAX_WINDOW] __attribute__((aligned(128)));
static float    out_buf[SPU_VTX_MAX_VERTS * SPU_RV_FLOATS] __attribute__((aligned(128)));

static uint64_t g_ring;

/* ---------------- self-measurement ----------------------------------
 *
 * The PPU could see only that it waited; it could not see what for. The SPU
 * decrementer counts down at the 79.8 MHz timebase (12.5 ns a tick), which is
 * ample for per-job accounting and costs a single channel read. Deltas are
 * taken modulo 2^32 because it counts DOWN and wraps. */
static uint32_t st_jobs;    /* jobs in THIS window (reset each publish) */
static uint32_t st_poll;    /* ticks spent looking for work             */
static uint32_t st_work;    /* ticks inside the job path                */
static uint32_t st_dma;     /* ticks blocked in dma_wait, ALL contexts  */
static uint32_t st_workdma; /* ticks blocked in dma_wait inside a job   */
static uint32_t st_total;   /* jobs since boot, for a sanity check      */
static uint8_t  stat_out[SPU_LINE] __attribute__((aligned(128)));

static inline uint32_t dec_now(void) { return spu_read_decrementer(); }
static inline uint32_t dec_since(uint32_t t0) { return (t0 - dec_now()) & 0xFFFFFFFFu; }

static void dma_get(void *ls, uint64_t ea, uint32_t size, int tag)
{
    uint8_t *p = (uint8_t *)ls;
    while (size) {
        uint32_t n = size > 16384u ? 16384u : size;
        mfc_get(p, ea, n, tag, 0, 0);
        p += n; ea += n; size -= n;
    }
}
static void dma_put(void *ls, uint64_t ea, uint32_t size, int tag)
{
    uint8_t *p = (uint8_t *)ls;
    while (size) {
        uint32_t n = size > 16384u ? 16384u : size;
        mfc_put(p, ea, n, tag, 0, 0);
        p += n; ea += n; size -= n;
    }
}
static void dma_wait(int tag)
{
    uint32_t t0 = dec_now();
    mfc_write_tag_mask(1u << tag);
    mfc_read_tag_status_all();
    st_dma += dec_since(t0);
}

/* Fetch [ea, ea+size) into a 128-byte-aligned LS buffer, returning the offset
 * within that buffer where the requested bytes begin.
 *
 * The MFC accepts sizes of 1/2/4/8 or a MULTIPLE OF 16 only, and wants the EA
 * and LS address to share alignment (SPU MFC manual 3.1: "transfer sizes of
 * 1, 2, 4, 8, and multiples of 16-bytes ... peak performance ... both the EA
 * and LA are 128-byte aligned"). Guest vertex streams sit at arbitrary
 * addresses with arbitrary lengths, so issuing them raw is an ILLEGAL
 * transfer that silently wedges the MFC queue -- which is precisely how the
 * first working SPU build still failed to retire a single job. Rounding the
 * EA down and the length up to whole 128-byte lines makes every transfer
 * both legal and the hardware's fast case. */
static uint32_t dma_get_window(void *ls_base, uint64_t ea, uint32_t size,
                               int tag)
{
    uint32_t skew  = (uint32_t)(ea & 127u);
    uint32_t total = (skew + size + 127u) & ~127u;
    dma_get(ls_base, ea - skew, total, tag);
    return skew;
}
/* Publish one u32 by writing its whole (SPU-owned) line. */
static void publish(uint32_t off, uint32_t val)
{
    *(uint32_t *)line_out = val;
    dma_put(line_out, g_ring + off, SPU_LINE, TAG_OUT);
    dma_wait(TAG_OUT);
}
static uint32_t fetch(uint32_t off)
{
    dma_get(line_in, g_ring + off, SPU_LINE, TAG_IN);
    dma_wait(TAG_IN);
    return *(uint32_t *)line_in;
}

/* The PPU-written half of the ring header in ONE transfer.
 *
 * HEAD and QUIT sit on separate 128-byte lines (deliberately -- they must not
 * share a line with anything the SPU writes), so reading them with two
 * `fetch` calls cost two full round trips to main memory on EVERY pass of the
 * idle loop, just to ask whether there was work. One 384-byte get covers
 * HEAD, DONE and QUIT together for the price of one. */
static uint8_t hdr_in[384] __attribute__((aligned(128)));
static void fetch_hdr(void)
{
    dma_get(hdr_in, g_ring, 384u, TAG_IN);
    dma_wait(TAG_IN);
}

/* This consumer's identity. Jobs are partitioned statically: this SPU takes
 * every ring index congruent to g_id modulo g_nspu, so several SPUs share the
 * ring without an atomic or a lock between them. */
static uint32_t g_id, g_nspu = 1;
#define HDR_U32(off) (*(uint32_t *)(hdr_in + (off)))

/* DONE without waiting for it to land.
 *
 * run_job has already waited for the vertex data put, so the data is in
 * memory before this is even issued and the PPU cannot observe DONE early.
 * Waiting here added a full round trip to the critical path of every job --
 * and that path is exactly what the PPU is spinning on. */
static uint8_t done_out[SPU_LINE] __attribute__((aligned(128)));
static void publish_done(uint32_t val)
{
    mfc_write_tag_mask(1u << TAG_DONE);
    mfc_read_tag_status_all();          /* the PREVIOUS one; long since done */
    *(uint32_t *)done_out = val;
    mfc_put(done_out, g_ring + SPU_OFF_DONE + g_id * SPU_LINE,
            SPU_LINE, TAG_DONE, 0, 0);
}

static void publish_stats(void)
{
    uint32_t *w = (uint32_t *)stat_out;
    w[0] = 0xC0DE0001u;                 /* heartbeat, unchanged */
    w[1] = st_jobs; w[2] = st_poll; w[3] = st_work; w[4] = st_workdma;
    w[5] = st_total;
    dma_put(stat_out, g_ring + SPU_OFF_HB, SPU_LINE, TAG_OUT);
    dma_wait(TAG_OUT);
    /* Per-window, not cumulative: a running total divided by a running job
     * count is dominated by however long the SPU sat idle in a menu, which
     * says nothing about what a job costs. */
    st_jobs = st_poll = st_work = st_workdma = 0;
}

/* Wide reads without byte-by-byte assembly.
 *
 * The SPU has no byte load: `p[0]` is a quadword load, a rotate and a mask, so
 * building a 32-bit field out of four of them runs to twenty-odd instructions
 * -- for a value whose bytes were already adjacent and already in the right
 * order, guest and SPE both being big-endian. Measured cost of the old form:
 * ~2.0 us of pure compute per six-vertex job, which the PPU then sat and span
 * on. The packed struct states the unaligned access and lets the compiler
 * pick its own sequence. */
struct un16 { uint16_t v; } __attribute__((packed));
struct un32 { uint32_t v; } __attribute__((packed));
static inline uint16_t rd16(const uint8_t *p)
{ return ((const struct un16 *)p)->v; }
static inline uint32_t rd32(const uint8_t *p)
{ return ((const struct un32 *)p)->v; }
static inline float rdf32(const uint8_t *p)
{ union { uint32_t u; float f; } v; v.u = rd32(p); return v.f; }

/* Every scale is an exact power of two, so the table is bit-identical to the
 * division it replaces -- and the division ran once per COMPONENT per vertex
 * on a processor with no divide instruction. */
static float frac_scale(unsigned frac)
{
    static const float k[32] = {
        1.0f/(1u<<0),  1.0f/(1u<<1),  1.0f/(1u<<2),  1.0f/(1u<<3),
        1.0f/(1u<<4),  1.0f/(1u<<5),  1.0f/(1u<<6),  1.0f/(1u<<7),
        1.0f/(1u<<8),  1.0f/(1u<<9),  1.0f/(1u<<10), 1.0f/(1u<<11),
        1.0f/(1u<<12), 1.0f/(1u<<13), 1.0f/(1u<<14), 1.0f/(1u<<15),
        1.0f/(1u<<16), 1.0f/(1u<<17), 1.0f/(1u<<18), 1.0f/(1u<<19),
        1.0f/(1u<<20), 1.0f/(1u<<21), 1.0f/(1u<<22), 1.0f/(1u<<23),
        1.0f/(1u<<24), 1.0f/(1u<<25), 1.0f/(1u<<26), 1.0f/(1u<<27),
        1.0f/(1u<<28), 1.0f/(1u<<29), 1.0f/(1u<<30), 1.0f/(1u<<31)
    };
    return k[frac & 31u];
}

static float dequant(const uint8_t *p, unsigned fmt, unsigned frac, unsigned i)
{
    float scale = frac_scale(frac);
    switch (fmt) {
    case SVF_U8:  return (float)p[i] * scale;
    case SVF_S8:  return (float)(int8_t)p[i] * scale;
    case SVF_U16: return (float)rd16(p + i*2) * scale;
    case SVF_S16: return (float)(int16_t)rd16(p + i*2) * scale;
    default:      return rdf32(p + i*4);
    }
}
static unsigned fmt_bytes(unsigned fmt)
{ return fmt <= SVF_S8 ? 1u : fmt <= SVF_S16 ? 2u : 4u; }

static void col_decode(const uint8_t *p, unsigned fmt, float *rgba)
{
    switch (fmt) {
    case SVC_RGB565: { uint16_t v = rd16(p);
        rgba[0]=(float)((v>>11)&31)/31.0f; rgba[1]=(float)((v>>5)&63)/63.0f;
        rgba[2]=(float)(v&31)/31.0f; rgba[3]=1.0f; break; }
    case SVC_RGB8: case SVC_RGBX8:
        rgba[0]=p[0]/255.0f; rgba[1]=p[1]/255.0f; rgba[2]=p[2]/255.0f;
        rgba[3]=1.0f; break;
    case SVC_RGBA4: { uint16_t v = rd16(p);
        rgba[0]=(float)((v>>12)&15)/15.0f; rgba[1]=(float)((v>>8)&15)/15.0f;
        rgba[2]=(float)((v>>4)&15)/15.0f;  rgba[3]=(float)(v&15)/15.0f; break; }
    case SVC_RGBA6: { uint32_t v=((uint32_t)p[0]<<16)|((uint32_t)p[1]<<8)|p[2];
        rgba[0]=(float)((v>>18)&63)/63.0f; rgba[1]=(float)((v>>12)&63)/63.0f;
        rgba[2]=(float)((v>>6)&63)/63.0f;  rgba[3]=(float)(v&63)/63.0f; break; }
    default:
        rgba[0]=p[0]/255.0f; rgba[1]=p[1]/255.0f;
        rgba[2]=p[2]/255.0f; rgba[3]=p[3]/255.0f; break;
    }
}
static unsigned col_bytes(unsigned fmt)
{
    switch (fmt) {
    case SVC_RGB565: case SVC_RGBA4: return 2u;
    case SVC_RGB8:   case SVC_RGBA6: return 3u;
    default:                         return 4u;
    }
}

static uint32_t window_off[SPU_VTX_MAX_ATTRS];

/* Byte offset of attribute `a`'s field within a vertex. The recipe lists the
 * attributes in stream order, so this is the running size of everything before
 * it -- the same walk the decode loop does, hoisted so the scan can address
 * each index directly. */
static uint32_t attr_pos(const SpuVtxJob *j, unsigned want)
{
    uint32_t off = 0;
    unsigned a;
    for (a = 0; a < j->nattrs && a < want; a++) {
        const SpuVtxAttr *at = &j->attr[a];
        if (at->kind == SVA_PNMTXIDX || at->kind == SVA_TEXMTXIDX) { off += 1; continue; }
        if (at->mode != SVM_DIRECT) { off += (at->mode == SVM_IDX8) ? 1u : 2u; continue; }
        /* Exactly the advances the decode loop below makes: colours are the
         * only kind measured with col_bytes; position, normal and every
         * texture coordinate use count x fmt_bytes. Getting this wrong reads
         * the indices from the wrong byte and silently corrupts geometry. */
        switch (at->kind) {
        case SVA_COL0: case SVA_COL1:
            off += col_bytes(at->fmt); break;
        default:
            off += (uint32_t)at->count * fmt_bytes(at->fmt); break;
        }
    }
    return off;
}

static uint16_t local_base[SPU_VTX_MAX_ATTRS];
static uint8_t  gather[SPU_VTX_MAX_ATTRS];
/* Staging for the one-element fallback above; 128-aligned for the MFC. */
static uint8_t  elem_buf[256] __attribute__((aligned(128)));

static void run_job(const SpuVtxJob *j)
{
    uint32_t win_used = 0, stream_skew;
    unsigned a, v;

    stream_skew = dma_get_window(stream_buf, j->stream_ea,
                                 (uint32_t)j->vert_count * j->vert_size,
                                 TAG_IN);

    if (j->flags & SPU_JOB_SCAN) {
        /* The PPU handed over the index scan. The indices live in the vertex
         * stream, so that has to land first -- one extra round trip here, and
         * none at all on the PPE, which is the side that is short of time. */
        unsigned v2;
        dma_wait(TAG_IN);
        for (a = 0; a < j->nattrs; a++) {
            const SpuVtxAttr *at = &j->attr[a];
            uint32_t mn = 0xFFFFFFFFu, mx = 0, span, need, off;
            const uint8_t *sp2;
            window_off[a] = 0; local_base[a] = 0; gather[a] = 0;
            if (at->mode == SVM_DIRECT || at->kind == SVA_END) continue;

            sp2 = stream_buf + stream_skew + attr_pos(j, a);
            for (v2 = 0; v2 < j->vert_count; v2++, sp2 += j->vert_size) {
                uint32_t idx = (at->mode == SVM_IDX8) ? *sp2 : rd16(sp2);
                if (idx < mn) mn = idx;
                if (idx > mx) mx = idx;
            }
            local_base[a] = (uint16_t)mn;
            span = (mx - mn + 1u) * at->stride;
            off  = mn * at->stride;
            /* Only fetch what the PPU vouched for, and only if it fits. */
            if (off + span > j->window_len[a]) { gather[a] = 1; continue; }
            need = ((uint32_t)((at->array_ea + off) & 127u) + span + 127u)
                   & ~127u;
            if (win_used + need > SPU_VTX_MAX_WINDOW) { gather[a] = 1; continue; }
            window_off[a] = win_used +
                dma_get_window(window_buf + win_used, at->array_ea + off,
                               span, TAG_IN);
            win_used += need;
        }
        dma_wait(TAG_IN);
    } else {
        for (a = 0; a < j->nattrs; a++) {
            const SpuVtxAttr *at = &j->attr[a];
            uint32_t need;
            window_off[a] = 0; local_base[a] = at->base_idx; gather[a] = 0;
            if (at->mode == SVM_DIRECT || at->kind == SVA_END) continue;
            need = ((uint32_t)(at->array_ea & 127u) + j->window_len[a] + 127u)
                   & ~127u;
            if (win_used + need > SPU_VTX_MAX_WINDOW) continue;
            window_off[a] = win_used +
                dma_get_window(window_buf + win_used, at->array_ea,
                               j->window_len[a], TAG_IN);
            win_used += need;
        }
        dma_wait(TAG_IN);
    }

    {
        const uint8_t *vp = stream_buf + stream_skew;
        float *ov = out_buf;
        for (v = 0; v < j->vert_count; v++, vp += j->vert_size,
             ov += SPU_RV_FLOATS) {
            const uint8_t *sp = vp;
            unsigned t;
            ov[0]=ov[1]=ov[2]=0.0f;
            ov[3]=0.0f; ov[4]=0.0f; ov[5]=1.0f;
            ov[6]=ov[7]=ov[8]=ov[9]=1.0f;
            for (t=0;t<4;t++){ ov[10+t*3]=0.0f; ov[11+t*3]=0.0f; ov[12+t*3]=1.0f; }

            for (a = 0; a < j->nattrs; a++) {
                const SpuVtxAttr *at = &j->attr[a];
                const uint8_t *src;
                if (at->kind == SVA_END) break;
                if (at->kind == SVA_PNMTXIDX || at->kind == SVA_TEXMTXIDX) {
                    sp += 1; continue;
                }
                if (at->mode == SVM_DIRECT) {
                    src = sp;
                } else {
                    uint32_t idx = (at->mode == SVM_IDX8) ? *sp : rd16(sp);
                    sp += (at->mode == SVM_IDX8) ? 1u : 2u;
                    idx -= local_base[a];
                    if (gather[a]) {
                        /* Window did not fit: fetch this one element. Slow,
                         * and correct, and it happens on the consumer that
                         * has time to spare rather than on the PPE. */
                        dma_get(elem_buf, at->array_ea +
                                ((uint32_t)local_base[a] + idx) * at->stride,
                                (at->stride + 15u) & ~15u, TAG_IN);
                        dma_wait(TAG_IN);
                        src = elem_buf;
                    } else {
                        src = window_buf + window_off[a] + idx * at->stride;
                    }
                }
                switch (at->kind) {
                case SVA_POS: {
                    unsigned i, n = at->count;
                    for (i = 0; i < n && i < 3u; i++)
                        ov[i] = dequant(src, at->fmt, at->frac, i);
                    if (at->mode == SVM_DIRECT) sp += n * fmt_bytes(at->fmt);
                    break; }
                case SVA_NRM: {
                    unsigned i, n = at->count;
                    for (i = 0; i < 3u && i < n; i++)
                        ov[3+i] = dequant(src, at->fmt, at->frac, i);
                    if (at->mode == SVM_DIRECT) sp += n * fmt_bytes(at->fmt);
                    break; }
                case SVA_COL0:
                    col_decode(src, at->fmt, &ov[6]);
                    if (at->mode == SVM_DIRECT) sp += col_bytes(at->fmt);
                    break;
                case SVA_COL1:
                    if (at->mode == SVM_DIRECT) sp += col_bytes(at->fmt);
                    break;
                default: {
                    unsigned i, n = at->count;
                    unsigned tn = at->kind - SVA_TEX0;
                    if (tn < 4u)
                        for (i = 0; i < n && i < 2u; i++)
                            ov[10 + tn*3 + i] = dequant(src, at->fmt, at->frac, i);
                    if (at->mode == SVM_DIRECT) sp += n * fmt_bytes(at->fmt);
                    break; }
                }
            }
        }
    }
    /* dest_ea is 128-aligned (spu_vtx_reserve rounds every allocation up to a
     * whole line), so rounding the length up to a line is both legal and
     * in-bounds. */
    dma_put(out_buf, j->dest_ea,
            ((uint32_t)j->vert_count * SPU_RV_BYTES + 127u) & ~127u, TAG_OUT);
    dma_wait(TAG_OUT);
}

int main(uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4)
{
    uint32_t next;
    (void)arg3; (void)arg4;
    g_ring = arg1;                     /* sysSpuThreadArgument.arg0 */
    g_id   = (uint32_t)arg2;           /* consumer index, 0..nspu-1        */

    spu_write_decrementer(0xFFFFFFFFu); /* free-running, for the accounting */

    /* The consumer count is written by the PPU before the group starts, so a
     * single image serves any number of SPUs and the count stays a runtime
     * choice. Clamp: a zero here would make the stride zero and spin forever
     * on one job. */
    g_nspu = fetch(SPU_OFF_NSPU);
    if (g_nspu == 0 || g_nspu > SPU_VTX_MAX_SPU) g_nspu = 1;
    if (g_id >= g_nspu) g_id = 0;
    next = g_id;                       /* first job that belongs to me */

    /* Publish my starting position BEFORE the heartbeat.
     *
     * DONE means "the lowest job index I have not finished", so at startup it
     * is my id. Without this it stayed at the memset zero, and a consumer that
     * had not yet been handed any work pinned the PPU's minimum at 0 -- so
     * spu_vtx_join waited for a job that did not exist and burned the whole
     * spin limit (measured: one join, 20,000,001 iterations). Publishing here,
     * before the heartbeat the PPU waits on, means the PPU never observes an
     * unpublished DONE. */
    publish(SPU_OFF_DONE + g_id * SPU_LINE, next);
    publish(SPU_OFF_HB   + g_id * SPU_LINE, 0xC0DE0001u);

    for (;;) {
        uint32_t t_poll = dec_now();
        fetch_hdr();
        if (HDR_U32(SPU_OFF_QUIT)) break;
        /* Unsigned compare, so a HEAD that has not yet reached my next slot
         * means idle. HEAD only grows. */
        if ((int32_t)(HDR_U32(SPU_OFF_HEAD) - next) <= 0) {
            st_poll += dec_since(t_poll); continue;
        }
        st_poll += dec_since(t_poll);

        {
            uint32_t t_work = dec_now();
            uint32_t d0 = st_dma;
            dma_get(&jobslot, g_ring + SPU_OFF_SLOTS +
                    (uint64_t)(next & (SPU_VTX_RING - 1u)) * sizeof(SpuVtxJobSlot),
                    sizeof(SpuVtxJobSlot), TAG_IN);
            dma_wait(TAG_IN);
            run_job(&jobslot.job);
            next += g_nspu;            /* skip the slots my peers own */
            publish_done(next);
            st_work += dec_since(t_work);
            st_workdma += st_dma - d0;
        }
        /* Every 256 jobs, not every job: the stats put is itself a round trip
         * and must not become part of what it is measuring. */
        st_jobs++; st_total++;
        if (st_jobs >= 256u) publish_stats();
    }
    publish(SPU_OFF_HB + g_id * SPU_LINE, 0xC0DEDEADu);
    spu_thread_exit(0);
    return 0;
}
