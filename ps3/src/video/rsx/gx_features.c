/* gx_features.c — the one definition of the pipeline-feature mask.
 *
 * See gx_features.h. The default is every group hardware has already agreed
 * with, plus the 3D groups. Those are on because the audit over a complete
 * boot says what they cost on the screen the console currently renders, and
 * for three of them the answer is nothing at all: no draw of the 174,282
 * enables a lit channel, none enables fog, and no stage of the 325,717
 * sources a konst, so LIGHTING, FOG and the konst half of KONST are the
 * identity there by measurement rather than by hope.
 *
 * Two qualifications, both small and both worth stating rather than
 * discovering. TEXGEN changes the *shape* of the emitted program on the title
 * screen -- its one texgen is regular, source row tex0, ST, no dual transform,
 * so the coordinate it computes is the same s and t, but it now arrives with
 * q forced to 1 rather than carrying the next matrix row's dot product, which
 * a 2D fetch ignored anyway. And KONST also switches on the swap tables, which
 * 108 of those 325,717 stages select non-identity: those 108 change, and they
 * change from wrong to right.
 *
 * GX_STATE_CULL is deliberately *not* in the default, and this is the one
 * exception to the paragraph above -- because culling is emphatically not
 * inert on the title screen. 173,061 of those 174,282 draws ask for
 * GX_CULL_BACK; only 1,221 ask for nothing. So enabling it changes the title
 * screen, and if the winding is the wrong way round it changes it to nothing
 * at all.
 *
 * Which winding the RSX calls front, once the viewport's negative y scale has
 * reversed the orientation, is the one thing here that cannot be settled off
 * the console. GX's own rule is known exactly -- Dolphin's software clipper
 * calls a triangle front-facing when the signed area of its clip-space
 * projection is positive, and this backend's clip-to-window mapping is
 * bit-for-bit GX's -- but NV4x states its rule against its own raster space
 * and nothing available here says which handedness that is.
 *
 * The good news is that the title screen is itself the oracle, and a decisive
 * one: with the winding right it looks exactly as it does today, and with it
 * wrong it loses essentially every draw. So one console session settles the
 * question for the whole port by trying the default mask | GX_STATE_CULL and
 * then | GX_STATE_CULL_FLIP, and whichever leaves the title screen intact is
 * the answer. That is a better experiment than any amount of reasoning about
 * raster-space handedness, which is why the code is written to make it a
 * two-step bisect rather than a guess baked into a build.
 */
#include "gx_features.h"

unsigned g_gx_state_mask =
    /* MASKS is load-bearing, not cosmetic. MKWii's title background is a
     * 32x32 pattern blended with src=INVDSTALPHA dst=DSTALPHA -- the result
     * depends on the DESTINATION alpha earlier draws left in the EFB. With
     * write masks ignored, every draw wrote alpha, dstA was wrong, and the
     * blend showed the pattern at full amplitude: the horizontal bars.
     * Honouring the masks flattens it to within 2/255 of Dolphin. */
    /* GX_STATE_MASKS deliberately OFF: with per-draw colour write-masks
     * honoured, every 3D model draw arrived with a latched CMODE0 of 0x34a0
     * (colorupdate=0, alphaupdate=0) and the whole 3D world was masked to
     * black -- menus, karts, the race, everything except the flat UI.
     * Ignoring the masks renders the Single Player trophy, the kart preview
     * and the menus correctly (measured on hardware). The stale-latch root
     * cause (the game's real CMODE0 writes seemingly not reaching raw[0x41]
     * at those draws) still deserves hunting, but the mask-off picture is
     * strictly better today. Risk accepted: z-only prepass draws would
     * double-draw -- none observed so far. */
    GX_STATE_BLEND | GX_STATE_ALPHATEST | GX_STATE_DEPTH |
    GX_STATE_VIEWPORT | GX_STATE_SCISSOR |
    GX_STATE_EFB_COPY | GX_STATE_EFB_CLEAR |
    GX_STATE_LIGHTING | GX_STATE_TEXGEN | GX_STATE_KONST | GX_STATE_FOG |
    /* Culling, with the front-face winding INVERTED relative to what the RSX
     * Users Manual reading alone suggested. That reading could not be settled
     * off hardware, and hardware has now settled it: with CULL on and
     * CULL_FLIP off the console draws NOTHING -- a captured frame is a single
     * flat colour while the title is issuing ~50 draws per frame, because
     * every front face is being culled. Measured over four masks:
     *
     *   CULL=1 FLIP=0  ->    1 distinct colour   (blank)
     *   CULL=1 FLIP=1  ->  339 distinct colours  (renders)
     *   CULL=0         ->  228 distinct colours  (renders)
     *
     * So the flip belongs in the default, not behind a debug step. */
    GX_STATE_CULL | GX_STATE_CULL_FLIP |
    /* Indirect texturing. On by default for the same reason the other 3D
     * groups are: with the bit clear the generator emits exactly the
     * coordinate it emitted before, so "off" is the build the console has
     * already agreed with, and it is the TOP step of the pad bisect -- one
     * press of L2 takes it away again without disturbing anything below it.
     *
     * What it changes, measured over a complete run to a race: 1,813,066 of
     * the 9,482,453 draws configure the indirect unit, but 1,512,623 of those
     * leave every per-stage indirect command at zero, which generates
     * byte-for-byte the previous program. The 300,443 draws that carry a live
     * command are the ones whose picture changes -- and changes from a
     * distortion that was silently not applied to one that is. */
    GX_STATE_INDIRECT |
    /* Tiled surfaces and Zcull. On by default because the measurement that
     * motivates it is not a guess: the console's own phase profiler puts
     * jit_run at ~33% of a frame with the GPU and scanout waits taking the
     * rest, and the RSX SOL note measures a LINEAR colour buffer at 50% of a
     * tiled one under a ROP-bound load. It is also the reason enabling the
     * depth test roughly halved the frame rate -- an untiled depth buffer pays
     * full ROP bandwidth AND cannot validate Zcull, so it costs twice.
     *
     * Off is still the exact previous build: see the comment on the bit in
     * gx_features.h for what "off" restores and for why the tiling half is
     * latched at init while the Zcull half follows the live mask. */
    GX_STATE_RSX_TILE;
