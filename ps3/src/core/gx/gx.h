/* gx.h — the GameCube/Wii graphics command stream: registers and vertex format.
 *
 * The command processor consumes a byte stream, not a register interface, and
 * that stream is self-describing only if you already know the vertex format.
 * A draw command says "112 vertices of primitive type 3, attribute table 0"
 * and then hands over raw bytes; how many bytes depends on state loaded
 * earlier — which attributes are present (the vertex descriptor) and how each
 * is encoded (the vertex attribute table).
 *
 * That is why vertex-size computation lives here rather than inside the parser:
 * get it wrong by one byte and the parser does not produce a wrong triangle, it
 * loses stream synchronisation permanently and every subsequent command is
 * garbage. It is the single most consequential arithmetic in the graphics
 * front end, and it is pure — state in, size out — so it can be tested
 * exhaustively without a GPU.
 */
#ifndef DOLPHIN_CORE_GX_GX_H
#define DOLPHIN_CORE_GX_GX_H

#include "../ppc/gekko.h"

/* ------------------------------------------------------------------ */
/* FIFO opcodes                                                         */
/* ------------------------------------------------------------------ */

#define GX_NOP              0x00
#define GX_LOAD_CP_REG      0x08
#define GX_LOAD_XF_REG      0x10
#define GX_LOAD_INDX_A      0x20    /* position matrices  */
#define GX_LOAD_INDX_B      0x28    /* normal matrices    */
#define GX_LOAD_INDX_C      0x30    /* texture matrices   */
#define GX_LOAD_INDX_D      0x38    /* light objects      */
#define GX_CMD_CALL_DL      0x40
#define GX_CMD_INVL_VC      0x44    /* invalidate vertex cache */
#define GX_LOAD_BP_REG      0x61

/* Primitives occupy 0x80..0xBF: the top five bits select the primitive and the
 * low three select which of the eight vertex attribute tables describes the
 * vertices that follow. */
#define GX_PRIMITIVE_MASK   0xF8
#define GX_PRIMITIVE_BASE   0x80
#define GX_VAT_MASK         0x07

typedef enum {
    GX_QUADS          = 0x80,
    GX_QUADS_2        = 0x88,   /* undocumented; same geometry as QUADS */
    GX_TRIANGLES      = 0x90,
    GX_TRIANGLE_STRIP = 0x98,
    GX_TRIANGLE_FAN   = 0xA0,
    GX_LINES          = 0xA8,
    GX_LINE_STRIP     = 0xB0,
    GX_POINTS         = 0xB8
} GXPrimitive;

/* ------------------------------------------------------------------ */
/* Vertex descriptor and attribute table                                */
/* ------------------------------------------------------------------ */

/* How an attribute reaches the GPU. Indexed attributes cost one or two bytes in
 * the vertex and are dereferenced through an array base register, which is how
 * a title shares one vertex pool across many draws. */
typedef enum {
    GX_ATTR_NONE    = 0,
    GX_ATTR_DIRECT  = 1,
    GX_ATTR_INDEX8  = 2,
    GX_ATTR_INDEX16 = 3
} GXAttrType;

/* Component encodings. The sizes are what the vertex-size arithmetic needs. */
typedef enum {
    GX_COMP_U8  = 0,
    GX_COMP_S8  = 1,
    GX_COMP_U16 = 2,
    GX_COMP_S16 = 3,
    GX_COMP_F32 = 4
} GXCompFormat;

typedef enum {
    GX_CLR_RGB565 = 0,   /* 2 bytes */
    GX_CLR_RGB8   = 1,   /* 3 */
    GX_CLR_RGBX8  = 2,   /* 4 */
    GX_CLR_RGBA4  = 3,   /* 2 */
    GX_CLR_RGBA6  = 4,   /* 3 */
    GX_CLR_RGBA8  = 5    /* 4 */
} GXColorFormat;

/* CP register numbers. */
#define CP_REG_MATINDEX_A   0x30
#define CP_REG_MATINDEX_B   0x40
#define CP_REG_VCD_LO       0x50
#define CP_REG_VCD_HI       0x60
#define CP_REG_VAT_A        0x70    /* + n, n = 0..7 */
#define CP_REG_VAT_B        0x80
#define CP_REG_VAT_C        0x90
#define CP_REG_ARRAY_BASE   0xA0    /* + n, n = 0..15 */
#define CP_REG_ARRAY_STRIDE 0xB0

#define GX_NUM_VAT          8
#define GX_NUM_TEXCOORD     8
#define GX_NUM_ARRAYS       16

typedef struct {
    u32 vcd_lo;                     /* CP 0x50 */
    u32 vcd_hi;                     /* CP 0x60 */
    u32 vat_a[GX_NUM_VAT];          /* CP 0x70 + n */
    u32 vat_b[GX_NUM_VAT];          /* CP 0x80 + n */
    u32 vat_c[GX_NUM_VAT];          /* CP 0x90 + n */
    u32 matindex_a, matindex_b;
    u32 array_base[GX_NUM_ARRAYS];
    u32 array_stride[GX_NUM_ARRAYS];
} GXCPRegs;

/* Bytes one vertex occupies in the command stream, for the given descriptor and
 * attribute table. Returns 0 only if `vat` is out of range, which the parser
 * treats as a stream it cannot follow. */
u32 gx_vertex_size(const GXCPRegs *cp, unsigned vat);

/* Broken out so the size arithmetic can be checked piece by piece rather than
 * only in aggregate -- an error in one attribute is otherwise indistinguishable
 * from an error in another. */
u32 gx_component_size(GXCompFormat fmt);
u32 gx_color_size(GXColorFormat fmt);

/* ------------------------------------------------------------------ */
/* Where parsed commands go                                             */
/*                                                                      */
/* The parser knows how to walk the stream; it deliberately knows nothing about */
/* rendering. Everything it recognises is handed to a sink, so the same parser   */
/* drives the RSX backend, a FIFO analyser, or a test that just counts.          */
/* ------------------------------------------------------------------ */

typedef struct {
    void *ctx;
    void (*load_cp)(void *ctx, u8 reg, u32 value);
    void (*load_xf)(void *ctx, u16 addr, const u32 *values, unsigned count);
    void (*load_bp)(void *ctx, u8 reg, u32 value);
    void (*load_index)(void *ctx, u8 which, u16 index, u16 xf_addr, u8 count);
    void (*draw)(void *ctx, GXPrimitive prim, unsigned vat,
                 u16 vertex_count, u32 data_addr, u32 vertex_size);
    void (*call_list)(void *ctx, u32 addr, u32 size);
} GXSink;

/* ------------------------------------------------------------------ */
/* Parser                                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    GXCPRegs cp;
    GXSink   sink;

    /* Statistics, which double as the parser's own health report: a stream it
     * is following correctly produces no unknown opcodes, ever. */
    u64 commands;
    u64 vertices;
    u64 draws;
    /* Per-class command counts. The FIFO phase is one of the largest single
     * costs in a frame and "parsing" is not a useful answer as to why; a
     * stream dominated by BP writes and one dominated by matrix uploads want
     * completely different fixes. */
    u64 n_bp;
    u64 n_cp;
    u64 n_xf;
    u64 n_xf_words;
    u64 n_dlist;
    u64 unknown_opcodes;
    u8  last_unknown;
} GXParser;

void gx_parser_init(GXParser *p, const GXSink *sink);
void gx_parser_reset(GXParser *p);

/* Parse commands out of the command FIFO until it runs dry or the next command
 * is incomplete. Returns the number of bytes consumed.
 *
 * Stopping on an incomplete command rather than blocking is what lets the
 * graphics front end be driven from the scheduler: a title writes a draw's
 * header and its vertices in separate bursts, and the GPU is expected to wait. */
u32 gx_parser_run(GXParser *p);

/* Parse a standalone buffer -- a display list called from the stream, or a
 * captured FIFO in a test. Returns bytes consumed. */
u32 gx_parser_run_memory(GXParser *p, u32 addr, u32 size);

#endif /* DOLPHIN_CORE_GX_GX_H */
