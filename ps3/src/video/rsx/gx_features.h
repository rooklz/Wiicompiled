/* gx_features.h — the runtime pipeline-feature mask.
 *
 * One bit per group of GX state this backend translates. Bit 0 clear across
 * the board reproduces the build that was known good on the console before any
 * of it existed, and each bit adds exactly one group -- which is how the depth
 * bug was found in a single hardware session rather than one launch per guess.
 *
 * The mask lives in its own translation unit rather than in gx_render.c
 * because the *shader generators* read it too, and they are built and tested
 * without any RSX: tests/test_tev.c and tests/test_xf.c link tev_program.c and
 * xf_program.c on their own. A feature that changes generated code has to be
 * switchable in exactly the same way as one that changes a register write,
 * otherwise a hardware bisect stops halfway down the pipeline.
 */
#ifndef DOLPHIN_VIDEO_RSX_GX_FEATURES_H
#define DOLPHIN_VIDEO_RSX_GX_FEATURES_H

/* The RSX memory-configuration group, above the pipeline groups because it is
 * not a pipeline group at all: it changes how the framebuffer and the depth
 * buffer are laid out in RSX local memory, and it is therefore decided ONCE,
 * inside rsx_video_init, before a single command has been submitted.
 *
 * With the bit set, rsx_video_init picks a legal tiled pitch, allocates the
 * colour buffers and the depth buffer 64 KiB-aligned in 64 KiB multiples,
 * binds one tiled region per surface (compressed for depth), and binds a Zcull
 * region over the depth buffer; rsx_clear and gx_render's per-title-frame
 * depth clear then keep that Zcull region valid. With it clear, every one of
 * those steps is skipped and the allocation is byte-for-byte what it was
 * before any of this existed.
 *
 * Two consequences of "decided once" worth stating rather than discovering.
 * The pad-driven bisect (k_gfx_steps in platform/ps3/main.c) cannot turn
 * tiling back ON at run time -- by the time a pad is read the buffers have
 * been allocated -- so bisecting the *tiling* half means changing the default
 * below and rebuilding. Clearing the bit at run time DOES stop the per-frame
 * Zcull programming, which is the half that can plausibly cull something it
 * should not, so the pad still bisects the risky half on its own. */
#define GX_STATE_RSX_TILE   0x4000u

/* Groups that were already on the console and bisected there. */
#define GX_STATE_BLEND      0x0001u
#define GX_STATE_ALPHATEST  0x0002u
#define GX_STATE_DEPTH      0x0004u
#define GX_STATE_MASKS      0x0008u
#define GX_STATE_VIEWPORT   0x0010u   /* honour the guest viewport rectangle  */
#define GX_STATE_SCISSOR    0x0020u   /* honour the guest scissor rectangle   */
#define GX_STATE_EFB_COPY   0x0040u   /* resolve non-XFB copies into textures */
#define GX_STATE_EFB_CLEAR  0x0080u   /* clear the EFB rect a copy clears     */

/* The 3D groups, in the order a Mario Kart race needs them. What each one is
 * worth, and what it costs on the title screen, is a measurement rather than a
 * judgement: the per-draw histogram over a complete boot is in tests/gxaudit.c,
 * and the per-material histogram over six courses and two karts read straight
 * off the disc is in the same place.
 *
 * All but CULL are inert on the title screen by that measurement -- it
 * configures no lit channel, no fog and no konst selection on any of its
 * 174,282 draws, and its only texgen is a plain regular one -- which is why
 * they are on by default. CULL is not inert and is not on by default; see
 * gx_features.c. */
#define GX_STATE_CULL       0x0100u   /* honour the guest cull mode           */
#define GX_STATE_CULL_FLIP  0x0200u   /* ...with the opposite front winding   */
#define GX_STATE_LIGHTING   0x0400u   /* per-channel lighting in the VP       */
#define GX_STATE_TEXGEN     0x0800u   /* texgen source/type/post-transform    */
/* The whole KSEL register: which constant a stage's KONST operand resolves to,
 * and the four swap tables, which share the same eight registers and are
 * therefore one group whether or not they are one feature. */
#define GX_STATE_KONST      0x1000u
#define GX_STATE_FOG        0x2000u   /* fog coordinate and blend             */
/* Indirect texturing: a second texture lookup whose result is transformed by a
 * small matrix and added to the coordinate the stage was about to sample with.
 * Measured over a complete race, 1,813,066 of 9,482,453 draws (19.1%) configure
 * one -- so this is not a corner of GX, it is a fifth of the frame -- and with
 * the bit clear the generator emits exactly the coordinate it emitted before,
 * which is the picture the console has already agreed with. */
#define GX_STATE_INDIRECT   0x8000u

extern unsigned g_gx_state_mask;

#endif /* DOLPHIN_VIDEO_RSX_GX_FEATURES_H */
