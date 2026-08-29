/* xfb_present.h -- display the video interface's framebuffer (YUV 4:2:2).
 *
 * Video playback writes decoded frames straight into the external
 * framebuffer without using GX, so a renderer that only presents GX output
 * shows nothing of them. Returns 0 when a frame was converted. */
#ifndef DOLPHIN_XFB_PRESENT_H
#define DOLPHIN_XFB_PRESENT_H

#include "../../common/types.h"

int xfb_present(u32 xfb_addr, unsigned width, unsigned height,
                u32 *dst, unsigned dst_pitch_words,
                unsigned dst_width, unsigned dst_height);

#endif
