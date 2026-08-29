/* vertex_loader.h — GX vertex attributes to float vectors.
 *
 * `gx_vertex_size` (gx.h) answers how many bytes a vertex occupies; this
 * answers what those bytes *mean*. They are separate problems: the size
 * computation only needs the attribute types and component widths, while
 * decoding needs the fractional-bit counts, the fixed normal scaling, six
 * different packed colour layouts, and — for indexed attributes — a
 * dereference through an array base and stride.
 *
 * The failure this file is built against is scale, not corruption. A fixed-
 * point position is a plain integer divided by 2^frac, and the exponent lives
 * in a five-bit field nobody looks at. Get it wrong and geometry is not
 * garbage: it is a recognisable model rendered 256 times too large or too
 * small, or a texture that tiles when it should not. Nothing about the picture
 * says "fractional bits", so the shift is tested directly, per format, against
 * hand-computed values.
 */
#ifndef DOLPHIN_VIDEO_RSX_VERTEX_LOADER_H
#define DOLPHIN_VIDEO_RSX_VERTEX_LOADER_H

#include "../../core/gx/gx.h"

#define VTX_MAX_TEXCOORD  GX_NUM_TEXCOORD

typedef struct {
    float position[3];
    unsigned position_count;    /* 2 or 3, or 0 when absent */

    float normal[9];            /* normal, or normal+binormal+tangent */
    unsigned normal_count;      /* 0, 3 or 9 */

    float color[2][4];          /* RGBA, 0..1 */
    unsigned color_present[2];

    float texcoord[VTX_MAX_TEXCOORD][2];
    unsigned texcoord_count[VTX_MAX_TEXCOORD];   /* 0, 1 or 2 */

    unsigned pos_matrix_index;
    unsigned tex_matrix_index[VTX_MAX_TEXCOORD];
    unsigned has_pos_matrix_index;
} VtxAttributes;

/* Decode one vertex.
 *
 * `addr` is the guest address of the vertex in the command stream; indexed
 * attributes are followed into the arrays the CP registers point at, which is
 * why this reads guest memory rather than taking a buffer. Returns the number
 * of bytes consumed from the stream, which must equal gx_vertex_size -- and the
 * tests assert exactly that, because the two computations are independent and a
 * disagreement between them is a desynchronised parser. */
u32 vtx_decode(const GXCPRegs *cp, unsigned vat, u32 addr, VtxAttributes *out);

/* Component scaling, exposed so the fractional-bit handling can be tested on
 * its own rather than only through a whole vertex. */
float vtx_dequantize(u32 raw, GXCompFormat fmt, unsigned frac_bits);

#endif /* DOLPHIN_VIDEO_RSX_VERTEX_LOADER_H */
