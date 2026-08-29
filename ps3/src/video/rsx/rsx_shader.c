/* rsx_shader.c — see rsx_shader.h.
 *
 * The header structures (rsxVertexProgram / rsxFragmentProgram) are normally
 * produced by cgcomp and read straight out of a .vpo/.fpo file. We fill them in
 * ourselves. Only the fields the upload path actually reads are set:
 *
 *   - ucode_off: the *byte offset from the header to the microcode*. Our
 *     microcode is a separate array, so for vertex programs the pointer is
 *     passed alongside and this field is unused; for fragment programs the
 *     GPU is given an RSX offset instead.
 *   - num_insn / num_regs: how much to upload and how many temporaries the
 *     program needs. Understating num_regs on the fragment side is the classic
 *     way to get a program that renders garbage on some pixels only.
 *   - input_mask / output_mask: which vertex attributes are read and which
 *     interpolants are written. The RSX uses these to route the two stages
 *     together; a mismatch silently drops an interpolant.
 */
#include "rsx_shader.h"
#include "rsx_video.h"
#include "../../common/log.h"

#include <string.h>
#include <rsx/rsx.h>

extern gcmContextData *rsx_context(void);

int rsx_fp_create(RsxFragProgram *fp, const u32 *ucode, u32 words,
                  u32 num_regs, u32 texcoord_mask)
{
    memset(fp, 0, sizeof *fp);

    /* 64-byte alignment: the fragment program fetch reads whole cache lines,
     * and an unaligned program is a documented way to hang the pipeline. */
    fp->ucode = rsxMemalign(64, words * 4);
    if (!fp->ucode) {
        LOG_ERROR(LOG_VIDEO, "fp: %u words would not allocate", (unsigned)words);
        return -1;
    }
    memcpy(fp->ucode, ucode, words * 4);

    if (rsxAddressToOffset(fp->ucode, &fp->offset) != 0) {
        /* Freed, not abandoned. This path leaked the allocation, and the
         * caller's cache slot is left unused on failure -- so the next draw in
         * the same state came straight back here and leaked another copy, at
         * draw rate, out of video memory that nothing else ever returns. */
        LOG_ERROR(LOG_VIDEO, "fp: microcode has no RSX offset");
        rsxFree(fp->ucode);
        memset(fp, 0, sizeof *fp);
        return -1;
    }

    fp->words    = words;
    fp->num_regs = num_regs < 1 ? 1 : num_regs;
    fp->texcoords = texcoord_mask;
    fp->valid    = 1;
    return 0;
}

void rsx_fp_destroy(RsxFragProgram *fp)
{
    if (fp->ucode) rsxFree(fp->ucode);
    memset(fp, 0, sizeof *fp);
}

void rsx_vp_create(RsxVertProgram *vp, const u32 *ucode, u32 words,
                   u32 instructions, u32 input_mask, u32 output_mask)
{
    vp->ucode        = ucode;
    vp->words        = words;
    vp->instructions = instructions;
    vp->input_mask   = input_mask;
    vp->output_mask  = output_mask;
    vp->valid        = 1;
}

/* Last pair actually loaded, and the frame it was loaded in.
 *
 * Binding was unconditional: every draw re-uploaded the whole vertex program
 * microcode into the command buffer, even though consecutive draws routinely
 * share a program -- a title sorts by material, so the same pair is bound over
 * and over. Loading a program is not a pointer swap; the vertex microcode is
 * copied word by word into the ring.
 *
 * The epoch is what makes skipping safe: the flip reinitialises the RSX
 * context, so a binding cannot be assumed to survive into the next frame. It
 * is bumped once per frame by rsx_shader_new_frame(). */
static const RsxVertProgram *s_bound_vp;
static const RsxFragProgram *s_bound_fp;
static u32 s_bound_epoch, s_shader_epoch = 1;

void rsx_shader_new_frame(void) { s_shader_epoch++; }

void rsx_bind_programs(const RsxVertProgram *vp, const RsxFragProgram *fp)
{
    gcmContextData *ctx = rsx_context();
    rsxVertexProgram   vph;
    rsxFragmentProgram fph;

    if (!ctx || !vp->valid || !fp->valid)
        return;

    if (vp == s_bound_vp && fp == s_bound_fp && s_bound_epoch == s_shader_epoch)
        return;                     /* already the live pair this frame */
    s_bound_vp = vp; s_bound_fp = fp; s_bound_epoch = s_shader_epoch;

    memset(&vph, 0, sizeof vph);
    vph.num_insn    = (u16)vp->instructions;
    /* The vertex program's register count decides exactly one thing in the
     * loader: whether TRANSFORM_TIMEOUT is programmed for a program using more
     * than 32 temporaries. NV4x has 32, the generated programs use nine at
     * their longest (a lit channel), and understating the count therefore
     * selects the same configuration a truthful one would. Checked rather than
     * assumed, because "the register count is a lie" is an alarming thing to
     * read next to a generator that now uses nine of them. */
    vph.num_regs    = 2;
    vph.input_mask  = vp->input_mask;
    vph.output_mask = vp->output_mask;
    vph.const_start = 0;
    vph.insn_start  = 0;
    rsxLoadVertexProgram(ctx, &vph, vp->ucode);

    memset(&fph, 0, sizeof fph);
    fph.num_insn = (u16)(fp->words / 4);
    fph.num_regs = (u16)fp->num_regs;
    /* Left at zero on purpose. The loader builds FP_CONTROL itself:
     *     fp_control | (num_regs << TEMP_COUNT_SHIFT) | (1 << 10)
     * so setting the temporary count here too ORs it in twice and corrupts the
     * field. num_regs alone is what the count should come from. */
    /* 0x40: the colour result is R0, in FP32.
     *
     * cgcomp sets this only when the parsed program writes an OUTPUT-typed
     * register (emit_dst, compilerfp.cpp); Cg-generated shaders always do, so
     * every working homebrew carries it. Assembly that writes plain R0 -- ours
     * and cgcomp's own `-a` output alike -- parses it as a temp, and the flag
     * is silently absent. Without it the ROP reads the output as FP16 halves
     * of R0: every constant renders as a saturate/zero pattern of its float's
     * halfwords, interpolants collapse to (0,1,0,1), and WPOS leaks its
     * half-pixel fraction into one channel. Matched, bit for bit, on all of
     * them. */
    fph.fp_control = 0x40;
    /* Which texture-coordinate interpolants this program reads. The loader
     * emits a TEX_COORD_CONTROL per set bit; with none set the hardware leaves
     * the coordinate in its default 2D configuration and only the first two
     * components carry a varying value -- which showed up as a gradient in red
     * with green and blue frozen. */
    fph.texcoords  = (u16)fp->texcoords;
    fph.texcoord2D = 0;
    fph.texcoord3D = 0;
    rsxLoadFragmentProgramLocation(ctx, &fph, fp->offset, GCM_LOCATION_RSX);
}

u32 rsx_fp_readback(const RsxFragProgram *fp, u32 *out, u32 max_words)
{
    u32 n, i;
    const volatile u32 *p = (const volatile u32 *)fp->ucode;
    if (!fp->valid || !fp->ucode) return 0;
    n = fp->words < max_words ? fp->words : max_words;
    for (i = 0; i < n; i++) out[i] = p[i];
    return n;
}

void rsx_vp_constants(u32 base, u32 rows, const f32 *values)
{
    gcmContextData *ctx = rsx_context();
    if (!ctx) return;
    /* The block form writes raw constant rows by index, which is what a
     * generated program needs: xf_program.h fixes the slot numbers (projection
     * at 0, texture matrices after it) precisely so the upload does not depend
     * on which program is loaded. */
    rsxLoadVertexProgramParameterBlock(ctx, base, rows, values);
}
