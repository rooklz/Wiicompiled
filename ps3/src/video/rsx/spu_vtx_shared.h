/* spu_vtx_shared.h — job protocol between the PPU GX backend and the SPU
 * vertex decoder. Compiled by BOTH toolchains (ppu-gcc and spu-gcc).
 *
 * LAYOUT IS LOAD-BEARING. The MFC's rules (SPE Users Manual §DMA):
 *   - transfers of 1/2/4/8 bytes require the LS address and the EA to have
 *     the SAME low four bits;
 *   - transfers of 16 bytes and up require both to be 16-byte aligned
 *     (128-byte aligned is a full cache line and is what the hardware wants).
 * The first version of this file put head/tail_done/quit adjacent in one
 * word-sized header and wrote them with 4-byte DMAs from offset-0 LS
 * buffers, which violates the first rule and stalls the MFC.
 *
 * So every field the two processors exchange gets its OWN 128-byte line:
 * the DMAs are whole aligned lines, and PPU and SPU never write the same
 * line (false sharing on a shared line costs an atomic-unit round trip per
 * access). The job slots start at line 4 and are 256 bytes each, so every
 * slot is 128-byte aligned too. */
#ifndef SPU_VTX_SHARED_H
#define SPU_VTX_SHARED_H

#include <stdint.h>

#define SPU_VTX_MAX_ATTRS   10
#define SPU_VTX_RING        64        /* jobs in flight; power of two */
#define SPU_VTX_MAX_VERTS   512       /* per job; larger draws stay on PPU */
#define SPU_VTX_MAX_WINDOW  (64*1024) /* per-attr indexed-array DMA window */

/* Consumers. The PS3 gives a game six SPUs and this used exactly one; the
 * other five had never executed an instruction. Jobs are partitioned
 * STATICALLY -- SPU `id` takes every job whose ring index is congruent to id
 * modulo the consumer count -- so no atomics, no lock, and no contended line
 * are needed to share the ring. Each consumer owns its own DONE and HB word,
 * on its own 128-byte line, because two SPUs writing one line would ping-pong
 * it between them on every job.
 *
 * The count is written into the header by the PPU before the group starts, so
 * it is a runtime choice (wiicompiled-spucount.txt) rather than a rebuild, and the
 * SPU image adapts to whatever it is told. */
#define SPU_VTX_MAX_SPU     6

#define SPU_LINE            128u
#define SPU_OFF_HEAD        0u        /* PPU writes, SPU reads  */
#define SPU_OFF_QUIT        128u      /* PPU writes, SPU reads  */
#define SPU_OFF_NSPU        256u      /* PPU writes once, SPU reads at start */
#define SPU_OFF_DONE        384u      /* SPU writes, PPU reads; +id*SPU_LINE */
#define SPU_OFF_HB          (SPU_OFF_DONE + SPU_VTX_MAX_SPU * SPU_LINE)
#define SPU_OFF_SLOTS       (SPU_OFF_HB   + SPU_VTX_MAX_SPU * SPU_LINE)

/* kind */
enum {
    SVA_END = 0,
    SVA_PNMTXIDX,
    SVA_TEXMTXIDX,
    SVA_POS,
    SVA_NRM,
    SVA_COL0, SVA_COL1,
    SVA_TEX0
};
/* mode */
enum { SVM_DIRECT = 0, SVM_IDX8 = 1, SVM_IDX16 = 2 };
/* component formats */
enum { SVF_U8 = 0, SVF_S8, SVF_U16, SVF_S16, SVF_F32 };
/* colour formats */
enum { SVC_RGB565 = 0, SVC_RGB8, SVC_RGBX8, SVC_RGBA4, SVC_RGBA6, SVC_RGBA8 };

typedef struct {
    uint8_t  kind;
    uint8_t  mode;
    uint8_t  fmt;
    uint8_t  count;
    uint8_t  frac;
    uint8_t  texn;
    uint16_t stride;
    uint16_t base_idx;
    uint16_t pad2;
    uint32_t pad3;
    uint64_t array_ea;               /* HOST effective address, 64-bit:
                                      * an MFC EA is 64-bit, and a 32-bit
                                      * field silently truncates host
                                      * pointers on a 64-bit test build. */
} SpuVtxAttr;                        /* 24 bytes */

/* job.flags */
/* The PPU did NOT pre-scan the index streams: array_ea points at element 0 of
 * the whole array and window_len is the contiguous span the PPU could vouch
 * for. The consumer works out its own min/max from the indices, which it
 * already has locally once the vertex stream lands, and fetches just that.
 *
 * The scan is O(vertices x indexed attributes) and it was running on the PPE,
 * which is the bottleneck, purely to serve an SPU that is idle 95% of the
 * time. Doing it on the consumer costs a second DMA round trip there (the
 * stream has to arrive before the indices can be read) and nothing on the PPE.
 */
#define SPU_JOB_SCAN 0x0001u

typedef struct {
    uint64_t stream_ea;
    uint64_t dest_ea;
    uint16_t vert_count;
    uint16_t vert_size;
    uint16_t nattrs;
    uint16_t flags;
    SpuVtxAttr attr[SPU_VTX_MAX_ATTRS];
    uint32_t window_len[SPU_VTX_MAX_ATTRS];
} SpuVtxJob;                         /* 24 + 240 + 40 = 304 */

typedef struct {
    SpuVtxJob job;
    uint8_t   pad[384 - 304];
} SpuVtxJobSlot;                     /* 384 = 3 whole 128-byte lines */

#define SPU_RV_FLOATS 22
#define SPU_RV_BYTES  88

#endif
