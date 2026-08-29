/* dsp_ax.h — high-level emulation of the Wii AX audio microcode.
 *
 * The DSP is a real second processor and the title uploads real DSP machine
 * code to it. Rather than emulate that processor, we recognise *which* ucode
 * was uploaded (by hashing the image the boot ROM was told to load, exactly as
 * Dolphin does in ROMUCode::BootUCode) and then reimplement its externally
 * visible behaviour in C. That behaviour is entirely defined by memory: AX
 * walks a linked list of voice parameter blocks, mixes each voice into an
 * internal accumulator, writes the finished stereo frame back to main memory
 * for the AI to DMA out, and writes each PB back with its sample position
 * advanced.
 *
 * The PB write-back is not a detail. nw4r::snd reads `running` to notice that a
 * one-shot sound has finished and reads the current address to know where a
 * stream has got to; a title whose PBs never change never retires a voice.
 */
#ifndef DOLPHIN_CORE_HW_DSP_AX_H
#define DOLPHIN_CORE_HW_DSP_AX_H

#include "../../common/types.h"

/* Samples in one AX Wii frame: 3 ms at the DSP's fixed 32 kHz process rate. */
#define AX_FRAME_SAMPLES 96
/* Wiimote speaker samples per frame (96 downsampled by 16/3). */
#define AX_WM_SAMPLES    18

void ax_reset(void);

/* Called when the boot ROM is told to start a ucode: `crc` is the Ector hash of
 * the uploaded image (Dolphin Common/Hash.cpp HashEctor). Recognised AXWii
 * hashes enable the mixer; anything else leaves it inactive and the DSP behaves
 * exactly as the mailbox-only stub did. */
void ax_boot_ucode(u32 crc);
u32  ax_ucode_crc(void);
int  ax_active(void);

/* Execute one AX command list. Returns the number of OUTPUT commands run, i.e.
 * how many finished frames were written to main memory. */
int  ax_run_cmdlist(u32 addr, u16 size_words);

/* Diagnostics, for the boot log and the unit tests. */
u64  ax_stat_frames(void);
u64  ax_stat_voices(void);
/* Of those, the ones that were actually running and so were mixed. */
u64  ax_stat_active_voices(void);
u64  ax_stat_samples_read(void);
/* Frames whose finished stereo output was not all zeroes, and the largest
 * absolute sample seen. Between them these answer "is the mixer producing
 * audio, or silence that happens to arrive on time" without a speaker. */
u64  ax_stat_audible_frames(void);
u32  ax_stat_peak(void);

/* ------------------------------------------------------------------ */
/* Exposed for the unit tests: one voice, one frame, no guest memory.   */
/* ------------------------------------------------------------------ */

/* The PB in host byte order, laid out exactly as the largest Wii variant
 * (Dolphin AXStructs.h struct AXPBWii). Every member is 16 bits, so the
 * structure is its own offset table and the per-ucode layout differences are
 * expressible as a list of word ranges. */
typedef struct { u16 volume, volume_delta; } AXVol;

typedef struct {
    u16 next_pb_hi, next_pb_lo;
    u16 this_pb_hi, this_pb_lo;
    u16 src_type, coef_select;
    u16 mixer_control_hi, mixer_control_lo;
    u16 running, is_stream;

    /* PBMixerWii */
    AXVol main_left, main_right;
    AXVol auxA_left, auxA_right;
    AXVol auxB_left, auxB_right;
    AXVol auxC_left, auxC_right;
    AXVol main_surround, auxA_surround, auxB_surround, auxC_surround;

    /* PBInitialTimeDelay */
    u16 itd_on, itd_addr_hi, itd_addr_lo;
    u16 itd_off_l, itd_off_r, itd_tgt_l, itd_tgt_r;

    /* PBDpopWii: main/auxA/auxB/auxC for L, then R, then surround. */
    s16 dpop[12];

    /* PBUpdatesWii — absent from the guest layout of newer ucodes. */
    u16 upd_num[3], upd_hi, upd_lo;

    /* PBVolumeEnvelope */
    s16 cur_volume, cur_volume_delta;

    /* PBAudioAddr */
    u16 looping, sample_format;
    u16 loop_addr_hi, loop_addr_lo;
    u16 end_addr_hi, end_addr_lo;
    u16 cur_addr_hi, cur_addr_lo;

    /* PBADPCMInfo */
    s16 coefs[16];
    u16 gain, pred_scale;
    s16 yn1, yn2;

    /* PBSampleRateConverter */
    u16 ratio_hi, ratio_lo, cur_addr_frac;
    s16 last_samples[4];

    /* PBADPCMLoopInfo */
    u16 loop_pred_scale, loop_yn1, loop_yn2;

    /* PBLowPassFilter */
    u16 lpf_on; s16 lpf_yn1; u16 lpf_a0; s16 lpf_b0;

    /* union { PBHighPassFilter; PBBiquadFilter } — ten words either way. */
    u16 bq_on;
    s16 bq_xn1, bq_xn2, bq_yn1, bq_yn2, bq_b0, bq_b1, bq_b2, bq_a1, bq_a2;

    /* Wiimote speaker path */
    u16 remote, remote_mixer_control;
    AXVol rm_main0, rm_aux0, rm_main1, rm_aux1;
    AXVol rm_main2, rm_aux2, rm_main3, rm_aux3;
    s16 rdpop[8];
    u16 rsrc_frac; s16 rsrc_last[4];
    u16 riir[10];

    u16 pad[2];
} AXPBWii;

#define AXPB_WORDS      155u        /* sizeof(AXPBWii) / 2 */
#define AXPB_UPD_WORD    53u        /* index of upd_num[0]  */
#define AXPB_FILT2_WORD 102u        /* index of bq_on       */

/* AX Wii sample formats (Dolphin AXStructs.h + DSPAccelerator.h). */
#define AX_FMT_ADPCM  0x0000
#define AX_FMT_PCM8   0x0019
#define AX_FMT_PCM16  0x000A

#define AX_SRC_POLYPHASE 0
#define AX_SRC_LINEAR    1
#define AX_SRC_NEAREST   2

/* The twelve regular mixing buffers of one frame, in AXBuffers order. */
typedef struct {
    s32 *buf[12];       /* main L/R/S, auxA L/R/S, auxB L/R/S, auxC L/R/S */
    s32 *wm[8];         /* wm0, aux0, wm1, aux1, wm2, aux2, wm3, aux3     */
} AXBufferSet;

/* Mix one voice for `count` samples. Reads samples through the accelerator
 * (i.e. from guest RAM), so the caller must have the guest memory set up; the
 * unit tests do exactly that. `pb` is updated in place the way the real ucode
 * updates it. */
void ax_process_voice(AXPBWii *pb, const AXBufferSet *bufs, u32 count,
                      int new_filter);

/* The mixer-control conversion, exposed because it is worth testing on its
 * own (Dolphin AXWii.cpp AXWiiUCode::ConvertMixerControl). */
u32 ax_convert_mixer_control(u32 mixer_control);

/* Resampler. The input is pulled through a callback because the real one comes
 * from the accelerator, which decodes ADPCM and handles looping as a side
 * effect of being read. Returns the new fractional position. */
typedef s16 (*AXSampleFn)(void *ctx, u32 index);
u32 ax_resample(AXSampleFn fn, void *ctx, s16 *out, u32 count,
                s16 *last_samples, u32 curr_pos, u32 ratio, int srctype);

/* Convenience wrapper for the tests: resample from a plain array. */
u32 ax_resample_array(const s16 *in, s16 *out, u32 count, s16 *last_samples,
                      u32 curr_pos, u32 ratio, int srctype);

#endif /* DOLPHIN_CORE_HW_DSP_AX_H */
