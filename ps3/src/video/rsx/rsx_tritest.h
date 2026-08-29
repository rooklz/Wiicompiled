/* rsx_tritest.h — a single triangle drawn with our own generated shaders.
 *
 * Proves on hardware what cgcomp comparison cannot: that the microcode our
 * encoders emit is actually executed by the RSX. See rsx_tritest.c. */
#ifndef DOLPHIN_VIDEO_RSX_TRITEST_H
#define DOLPHIN_VIDEO_RSX_TRITEST_H

#include "../../common/types.h"

/* mode 0: fragment program writes a known immediate constant, isolating the
 *         fragment output path from interpolation entirely.
 * mode 1: fragment program writes the interpolated COL0. */
int  rsx_tritest_init(int mode);
void rsx_tritest_draw(void);
void rsx_tritest_draw_novariant(void);
void rsx_tritest_shutdown(void);

/* Diagnostics: what is actually in RSX memory for the current programs. */
u32  rsx_tritest_fp_words(u32 *out, u32 max_words);
u32  rsx_tritest_fp_offset(void);
u32  rsx_tritest_vp_masks(u32 *input_mask, u32 *output_mask);

/* Re-upload the current fragment program with a byte-order transform applied:
 * 0 = as stored, 1 = halfword swap, 2 = full byte swap, 3 = byte swap within
 * halves. For the storage-order sweep. */
int  rsx_tritest_fp_variant(int variant);

#endif
