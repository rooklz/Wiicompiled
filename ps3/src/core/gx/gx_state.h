/* gx_state.h — the graphics state the command stream builds up.
 *
 * This is the join between the front end and any renderer. The command
 * processor parses; this owns what the parsing means. Nothing here draws
 * anything — it maintains the state a draw would be issued against, and hands
 * completed draws to whatever backend is attached.
 *
 * The split matters because the backend is the one part that cannot be tested
 * on a workstation. Everything up to and including this file is ordinary code
 * over ordinary memory, so the whole path from "guest stores to 0xCC008000" to
 * "the pixel engine is configured this way" is verifiable without a GPU, on any
 * host. What remains for the console is translation, not interpretation.
 *
 * XF ("transform") state is kept as its raw memory plus a handful of decoded
 * fields, for the same reason BP is: a register we have not studied yet must
 * still be recorded, because a dropped one is invisible.
 */
#ifndef DOLPHIN_CORE_GX_GX_STATE_H
#define DOLPHIN_CORE_GX_GX_STATE_H

#include "gx.h"
#include "bp.h"

/* ------------------------------------------------------------------ */
/* XF memory map                                                        */
/*                                                                      */
/* Addresses are the hardware's. The four memories below sit in one address     */
/* space with the registers, which is why a single write command can target any */
/* of them and why they are stored as one array.                                */
/* ------------------------------------------------------------------ */

#define XF_MEM_SIZE         0x1058u   /* through the last register we model */

#define XF_POSMATRIX_BASE   0x0000u   /* 64 matrices, 4 rows of 3 (+pad)    */
#define XF_POSMATRIX_SIZE   0x0100u
#define XF_NORMMATRIX_BASE  0x0400u
#define XF_NORMMATRIX_SIZE  0x0060u
#define XF_POSTMATRIX_BASE  0x0500u
#define XF_POSTMATRIX_SIZE  0x0100u
#define XF_LIGHT_BASE       0x0600u
#define XF_LIGHT_SIZE       0x0080u

#define XF_REG_BASE         0x1000u
#define XF_ERROR            0x1000u
#define XF_DIAGNOSTICS      0x1001u
#define XF_STATE0           0x1002u
#define XF_STATE1           0x1003u
#define XF_CLOCK            0x1004u
#define XF_CLIPDISABLE      0x1005u
#define XF_INVTXSPEC        0x1008u
#define XF_NUMCOLORS        0x1009u
#define XF_AMBIENT0         0x100Au
#define XF_AMBIENT1         0x100Bu
#define XF_MATERIAL0        0x100Cu
#define XF_MATERIAL1        0x100Du
#define XF_COLOR0CNTRL      0x100Eu
#define XF_COLOR1CNTRL      0x100Fu
#define XF_ALPHA0CNTRL      0x1010u
#define XF_ALPHA1CNTRL      0x1011u
#define XF_DUALTEXTRAN      0x1012u
#define XF_MATRIXINDEX_A    0x1018u
#define XF_MATRIXINDEX_B    0x1019u
#define XF_VIEWPORT         0x101Au   /* 6 words: 3 scales, 3 offsets       */
#define XF_PROJECTION       0x1020u   /* 6 coefficients + a type word       */
#define XF_NUMTEXGENS       0x103Fu
#define XF_TEXMTXINFO       0x1040u   /* + 0..7                              */
#define XF_POSTMTXINFO      0x1050u   /* + 0..7                              */

typedef struct {
    u32 mem[XF_MEM_SIZE];

    /* Decoded because a renderer reads them for every draw. The viewport in
     * particular is scale/offset rather than a rectangle, and converting it at
     * every use is both wasteful and a place to get the sign wrong. */
    float viewport_scale[3];
    float viewport_offset[3];
    float projection[6];
    unsigned projection_orthographic;

    unsigned num_texgens;       /* 0..8 */
    unsigned num_colorchans;    /* 0..2 */
} XFState;

/* ------------------------------------------------------------------ */
/* Backend                                                              */
/*                                                                      */
/* What a renderer implements. Deliberately narrow: everything else it needs is */
/* readable from the GXState it is handed.                                      */
/* ------------------------------------------------------------------ */

struct GXState;

typedef struct {
    void *ctx;

    /* One primitive, with its vertices still in guest memory. The backend
     * reads them through the vertex descriptor in `state`. */
    void (*draw)(void *ctx, const struct GXState *state, GXPrimitive prim,
                 unsigned vat, u16 vertex_count, u32 data_addr, u32 vertex_size);

    /* The embedded framebuffer is being copied out -- to the external
     * framebuffer the video interface scans out, or to a texture. This is the
     * point at which anything becomes visible. */
    void (*efb_copy)(void *ctx, const struct GXState *state, const BPCopy *copy);
} GXBackend;

typedef struct GXState {
    GXParser  parser;
    BPState   bp;
    XFState   xf;
    GXBackend backend;

    /* Draws issued while no backend was attached. Not an error -- the front end
     * is useful on its own, and the count is how a test says "the stream got
     * this far" without pretending to render. */
    u64 draws_dropped;
} GXState;

void gx_state_init(GXState *g, const GXBackend *backend);
void gx_state_reset(GXState *g);

/* Drain the command FIFO into state and backend calls. Called from the
 * scheduler; returns bytes consumed. */
u32  gx_state_run(GXState *g);

/* Parse a standalone display list, for a backend that wants to defer them. */
u32  gx_state_run_list(GXState *g, u32 addr, u32 size);

/* The machine's single instance, so the MMIO layer and the scheduler can reach
 * it without threading a pointer through every device. */
GXState *gx_state(void);

#endif /* DOLPHIN_CORE_GX_GX_STATE_H */
