/* rsx_tritest.c — draw one triangle with shaders built by our own encoders.
 *
 * The narrowest possible test of the claim the whole graphics port rests on:
 * that fp_emitter.h and vp_emitter.h emit microcode a real RSX will execute.
 * They are verified against `cgcomp` word for word, which proves the encoding
 * matches cgcomp -- but "matches the reference assembler" and "runs on
 * the GPU" are different claims, and only hardware can settle the second.
 *
 * Deliberately minimal: a fixed triangle in clip space, no matrices, no
 * textures, no TEV. If this shows a coloured triangle then the encoders, the
 * program headers, the upload path, attribute binding and the draw command are
 * all correct, and anything that fails afterwards is in the GX translation
 * rather than underneath it.
 */
#include "rsx_tritest.h"
#include "rsx_video.h"
#include "rsx_shader.h"
#include "vp_emitter.h"
#include "fp_emitter.h"
#include "cgcomp_tri.h"
#include "../../common/log.h"

#include <string.h>
#include <rsx/rsx.h>

extern gcmContextData *rsx_context(void);

/* Position and colour, interleaved, in the layout the attribute binding below
 * describes. Clip space directly: the vertex program passes position through,
 * so no projection matrix is involved and nothing can go wrong in one. */
typedef struct { f32 x, y, z; f32 r, g, b; } TriVertex;

static const TriVertex k_tri[3] = {
    { -0.6f, -0.5f, 0.0f,   1.0f, 0.2f, 0.2f },   /* red   */
    {  0.6f, -0.5f, 0.0f,   0.2f, 1.0f, 0.2f },   /* green */
    {  0.0f,  0.6f, 0.0f,   0.3f, 0.4f, 1.0f },   /* blue  */
};

/* RSX attribute indices. 0 is position by convention; any other index works for
 * colour as long as the vertex program reads the same one. */
#define ATTR_POS 0
#define ATTR_COL 3

static u32 s_vp_code[64];
static u32 s_fp_code[64];
static RsxVertProgram s_vp;
static RsxFragProgram s_fp;
static const rsxVertexProgram *s_ref_vp;
static const rsxFragmentProgram *s_ref_fp;
static void          *s_ref_fp_ucode;   /* copy in RSX memory */
static u32            s_ref_fp_offset;
static int            s_use_ref;
static TriVertex     *s_vb;
static u32            s_vb_offset;
static int            s_ready;

int rsx_tritest_init(int mode)
{
    VPEmitter ve;
    FPEmitter fe;

    if (s_ready)
        return 0;

    /* Vertex program: out.position = in[0]; out.col0 = in[3].
     *
     * Two MOVs. The output mask must name exactly the outputs written, or the
     * RSX routes an interpolant the fragment program then reads as garbage. */
    vp_init(&ve, s_vp_code, sizeof s_vp_code / 4);
    vp_emit_vec(&ve, VP_VEC_MOV, VP_OUT_POS,  1, VP_MASK_ALL, 0,
                vp_input(ATTR_POS), vp_none(), vp_none());
    /* Mode 3 routes the *position* attribute into COL0. Position is
     * demonstrably fetched correctly -- the triangle has exactly the right
     * shape and area -- so if the colour still comes out flat with a known-good
     * attribute feeding it, the fault is in the interpolant, and if it varies,
     * the fault is in attribute 3's data path. Same instruction either way, so
     * nothing else changes between the two. */
    /* Mode 4 carries the varying through TEX0 instead of COL0. The texture
     * coordinate interpolants are the most heavily exercised path on this
     * hardware, so if TEX0 varies where COL0 does not, the fault is specific to
     * the colour interpolant -- and carrying colour through a texcoord is a
     * legitimate way to render regardless. */
    if (mode == 4) {
        vp_emit_vec(&ve, VP_VEC_MOV, VP_OUT_TEX(0), 1, VP_MASK_ALL, 0,
                    vp_input(ATTR_COL), vp_none(), vp_none());
    } else {
        vp_emit_vec(&ve, VP_VEC_MOV, VP_OUT_COL0, 1, VP_MASK_ALL, 0,
                    vp_input(mode == 3 ? ATTR_POS : ATTR_COL),
                    vp_none(), vp_none());
    }
    vp_finish(&ve);

    if (ve.overflow) {
        LOG_ERROR(LOG_VIDEO, "tritest: vertex program overflowed");
        return -1;
    }

    /* Fragment program.
     *
     * Mode 0 writes a known immediate, which isolates the fragment output path
     * from interpolation completely: if the screen is not that exact colour,
     * the problem is the program or its upload, not the interpolant. Mode 1
     * writes the interpolated COL0, which is the real case. Splitting them is
     * the only way to tell "the fragment program is wrong" from "the value
     * arriving at the fragment program is wrong" -- both render a perfectly
     * solid triangle. */
    fp_init(&fe, s_fp_code, sizeof s_fp_code / 4);
    if (mode == 0) {
        static const float k_const[4] = { 1.0f, 0.5f, 0.25f, 1.0f };
        fp_emit(&fe, FP_OP_MOV, 0, FP_MASK_ALL, 0, 0,
                fp_imm(k_const[0], k_const[1], k_const[2], k_const[3]),
                fp_none(), fp_none());
    } else if (mode == 4) {
        fp_emit(&fe, FP_OP_MOV, 0, FP_MASK_ALL, 0, 0,
                fp_input(FP_IN_TEX(0)), fp_none(), fp_none());
    } else if (mode == 5) {
        /* WPOS: generated by the rasteriser itself, independent of the vertex
         * program, the attributes and the interpolant routing. Values are in
         * pixels, so every channel saturates and a correctly executing FP
         * renders the triangle *white*. Still green means the FP itself is
         * misdecoding; white means the FP is fine and only COL0's path is
         * dead. */
        fp_emit(&fe, FP_OP_MOV, 0, FP_MASK_ALL, 0, 0,
                fp_input(FP_IN_WPOS), fp_none(), fp_none());
    } else {
        fp_emit(&fe, FP_OP_MOV, 0, FP_MASK_ALL, 0, 0,
                fp_input(FP_IN_COL0), fp_none(), fp_none());
    }
    fp_finish(&fe);

    if (fe.overflow) {
        LOG_ERROR(LOG_VIDEO, "tritest: fragment program overflowed");
        return -1;
    }

    /* Mode 2 discards everything above and drives the draw from cgcomp's own
     * compiled binaries through PSL1GHT's own accessors. If the reference
     * renders and ours does not, the fault is in our program container; if
     * both come out the same, the fault is in the state around the draw. No
     * amount of reading either side settles that -- only running both. */
    s_use_ref = (mode == 2);
    if (s_use_ref) {
        void *uc; u32 ucsize;

        s_ref_vp = (const rsxVertexProgram *)k_cgcomp_vpo;
        s_ref_fp = (const rsxFragmentProgram *)k_cgcomp_fpo;

        rsxFragmentProgramGetUCode(s_ref_fp, &uc, &ucsize);
        s_ref_fp_ucode = rsxMemalign(64, ucsize);
        if (!s_ref_fp_ucode ||
            rsxAddressToOffset(s_ref_fp_ucode, &s_ref_fp_offset) != 0) {
            LOG_ERROR(LOG_VIDEO, "tritest: reference fp allocation failed");
            return -1;
        }
        memcpy(s_ref_fp_ucode, uc, ucsize);
        LOG_INFO(LOG_VIDEO, "tritest: reference fp %u bytes", (unsigned)ucsize);
    }

    rsx_vp_create(&s_vp, s_vp_code, ve.used, ve.used / 4,
                  ve.input_mask, ve.output_mask);
    if (rsx_fp_create(&s_fp, s_fp_code, fe.used, 2,
                      (mode == 4) ? 1u : 0u) != 0)
        return -1;

    /* The vertex buffer must be RSX-visible: the GPU pulls attributes itself. */
    s_vb = (TriVertex *)rsxMemalign(128, sizeof k_tri);
    if (!s_vb || rsxAddressToOffset(s_vb, &s_vb_offset) != 0) {
        LOG_ERROR(LOG_VIDEO, "tritest: vertex buffer allocation failed");
        return -1;
    }
    memcpy(s_vb, k_tri, sizeof k_tri);

    LOG_INFO(LOG_VIDEO, "tritest: vp %u words, fp %u words",
             (unsigned)ve.used, (unsigned)fe.used);
    s_ready = 1;
    return 0;
}

/* Draw without rebinding the fragment program, so a byte-order variant loaded
 * by rsx_tritest_fp_variant stays in effect. The vertex program is still
 * (re)bound -- it is uploaded through the FIFO and unaffected by the sweep. */
void rsx_tritest_draw_novariant(void)
{
    gcmContextData *ctx = rsx_context();
    void *uc; u32 ucsize;
    rsxVertexProgram vph;

    if (!s_ready || !ctx)
        return;

    memset(&vph, 0, sizeof vph);
    vph.num_insn    = (u16)s_vp.instructions;
    vph.num_regs    = 2;
    vph.input_mask  = s_vp.input_mask;
    vph.output_mask = s_vp.output_mask;
    (void)uc; (void)ucsize;
    rsxLoadVertexProgram(ctx, &vph, s_vp.ucode);

    rsxSetDepthTestEnable(ctx, GCM_FALSE);
    rsxSetCullFaceEnable(ctx, GCM_FALSE);
    rsxSetBlendEnable(ctx, GCM_FALSE);
    rsxSetTwoSideLightEnable(ctx, GCM_FALSE);

    rsxBindVertexArrayAttrib(ctx, ATTR_POS, 0, s_vb_offset,
                             sizeof(TriVertex), 3,
                             GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_RSX);
    rsxBindVertexArrayAttrib(ctx, ATTR_COL, 0, s_vb_offset + 12,
                             sizeof(TriVertex), 3,
                             GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_RSX);
    rsxDrawVertexArray(ctx, GCM_TYPE_TRIANGLES, 0, 3);
}

void rsx_tritest_draw(void)
{
    gcmContextData *ctx = rsx_context();
    if (!s_ready || !ctx)
        return;

    if (s_use_ref) {
        void *uc; u32 ucsize;
        rsxVertexProgramGetUCode(s_ref_vp, &uc, &ucsize);
        rsxLoadVertexProgram(ctx, s_ref_vp, uc);
        rsxLoadFragmentProgramLocation(ctx, s_ref_fp, s_ref_fp_offset,
                                       GCM_LOCATION_RSX);
    } else {
        rsx_bind_programs(&s_vp, &s_fp);
    }

    /* Depth testing off: the triangle is the only geometry and the depth buffer
     * is not cleared to anything meaningful in this test. */
    rsxSetDepthTestEnable(ctx, GCM_FALSE);
    rsxSetCullFaceEnable(ctx, GCM_FALSE);
    rsxSetBlendEnable(ctx, GCM_FALSE);
    /* With both front and back diffuse enabled in the result mask (which is
     * what cgcomp emits), two-sided lighting lets the hardware select the back
     * colour for a back-facing primitive -- and the vertex program never writes
     * it, so the fragment stage reads a constant. Disabling it forces the front
     * colour, which is the one being written. */
    rsxSetTwoSideLightEnable(ctx, GCM_FALSE);

    /* Position: 3 floats at offset 0 of a 24-byte vertex.
     * Colour:   3 floats at offset 12. */
    rsxBindVertexArrayAttrib(ctx, ATTR_POS, 0, s_vb_offset,
                             sizeof(TriVertex), 3,
                             GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_RSX);
    rsxBindVertexArrayAttrib(ctx, ATTR_COL, 0, s_vb_offset + 12,
                             sizeof(TriVertex), 3,
                             GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_RSX);

    rsxDrawVertexArray(ctx, GCM_TYPE_TRIANGLES, 0, 3);
}

u32 rsx_tritest_fp_words(u32 *out, u32 max_words)
{ return rsx_fp_readback(&s_fp, out, max_words); }

u32 rsx_tritest_fp_offset(void) { return s_fp.offset; }

u32 rsx_tritest_vp_masks(u32 *im, u32 *om)
{ if (im) *im = s_vp.input_mask; if (om) *om = s_vp.output_mask; return s_vp.words; }

/* Byte-order sweep. The constant-colour program's expected output is
 * unmistakable (1.0, 0.5, 0.25 -> ff8040), so uploading the same logical
 * program in each plausible storage order and reading one pixel back per
 * variant identifies the fetch path's real byte order empirically -- the one
 * question no document, reference binary, or code reading has settled. */
static u32 *s_var_buf;
static u32  s_var_offset;

int rsx_tritest_fp_variant(int variant)
{
    gcmContextData *ctx = rsx_context();
    rsxFragmentProgram fph;
    u32 i, w, n = s_fp.words;
    const u32 *src = (const u32 *)s_fp.ucode;

    if (!s_ready || !ctx)
        return -1;
    if (!s_var_buf) {
        s_var_buf = (u32 *)rsxMemalign(64, 64 * 4);
        if (!s_var_buf || rsxAddressToOffset(s_var_buf, &s_var_offset) != 0)
            return -1;
    }

    for (i = 0; i < n && i < 64; i++) {
        w = src[i];                          /* variant 0: as stored now */
        if (variant == 1)                    /* halfword swap            */
            w = (w << 16) | (w >> 16);
        else if (variant == 2)               /* full byte swap           */
            w = ((w & 0x000000FFu) << 24) | ((w & 0x0000FF00u) << 8) |
                ((w & 0x00FF0000u) >> 8)  | ((w & 0xFF000000u) >> 24);
        else if (variant == 3)               /* swap bytes within halves */
            w = ((w & 0x00FF00FFu) << 8) | ((w & 0xFF00FF00u) >> 8);
        s_var_buf[i] = w;
    }

    memset(&fph, 0, sizeof fph);
    fph.num_insn   = (u16)(n / 4);
    fph.num_regs   = 2;
    fph.fp_control = 0x40;      /* colour result is R0, FP32 -- see rsx_shader.c */
    rsxLoadFragmentProgramLocation(ctx, &fph, s_var_offset, GCM_LOCATION_RSX);
    return 0;
}

void rsx_tritest_shutdown(void)
{
    if (s_var_buf) { rsxFree(s_var_buf); s_var_buf = NULL; }
    if (s_ref_fp_ucode) { rsxFree(s_ref_fp_ucode); s_ref_fp_ucode = NULL; }
    s_use_ref = 0;
    if (s_vb) { rsxFree(s_vb); s_vb = NULL; }
    rsx_fp_destroy(&s_fp);
    s_ready = 0;
}
