/* dsp_ax.c — the Wii AX microcode, high-level.
 *
 * Reference: Dolphin's DSP HLE, which is the closest thing to a specification
 * that exists for this hardware.
 *
 *   Source/Core/Core/HW/DSPHLE/UCodes/AXWii.cpp   command list, PB list, output
 *   Source/Core/Core/HW/DSPHLE/UCodes/AXVoice.h   per-voice mixing, resampling
 *   Source/Core/Core/HW/DSPHLE/UCodes/AXStructs.h the parameter block layout
 *   Source/Core/Core/DSP/DSPAccelerator.cpp       sample fetch and ADPCM decode
 *
 * MKWii's ucode is identified rather than assumed: the boot ROM is handed the
 * image address (0x8027B4E0) and length (0x2000), and the Ector hash of those
 * bytes is 0x347112BA -- which is the "new" AXWii variant in Dolphin's
 * UCodeFactory (UCodes.cpp), meaning: no `updates` field in the guest PB, a
 * biquad filter present, and an explicit volume argument on OUTPUT. Everything
 * layout-dependent below follows from that one hash, so it is computed at run
 * time instead of being baked in.
 *
 * What this does *not* do (all of it inaudible or nearly so, and all of it
 * matching Dolphin's own omissions): initial time delay, Dolby Pro Logic II
 * downmixing, and polyphase resampling -- the last needs the DSP's coefficient
 * ROM, which we do not have, and Dolphin falls back to linear interpolation in
 * exactly the same case.
 */
#include "dsp_ax.h"
#include "hardware.h"
#include "../mem/memmap.h"
#include "../../common/log.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

DOL_STATIC_ASSERT(sizeof(AXPBWii) == AXPB_WORDS * 2, axpb_size);
DOL_STATIC_ASSERT(offsetof(AXPBWii, upd_num) == AXPB_UPD_WORD * 2, axpb_upd);
DOL_STATIC_ASSERT(offsetof(AXPBWii, bq_on) == AXPB_FILT2_WORD * 2, axpb_filt2);

/* ------------------------------------------------------------------ */
/* Mixer control bits (Dolphin AX.h enum AXMixControl)                  */
/* ------------------------------------------------------------------ */

#define MIX_MAIN_L      0x000001u
#define MIX_MAIN_L_RAMP 0x000002u
#define MIX_MAIN_R      0x000004u
#define MIX_MAIN_R_RAMP 0x000008u
#define MIX_MAIN_S      0x000010u
#define MIX_MAIN_S_RAMP 0x000020u
#define MIX_AUXA_L      0x000040u
#define MIX_AUXA_L_RAMP 0x000080u
#define MIX_AUXA_R      0x000100u
#define MIX_AUXA_R_RAMP 0x000200u
#define MIX_AUXA_S      0x000400u
#define MIX_AUXA_S_RAMP 0x000800u
#define MIX_AUXB_L      0x001000u
#define MIX_AUXB_L_RAMP 0x002000u
#define MIX_AUXB_R      0x004000u
#define MIX_AUXB_R_RAMP 0x008000u
#define MIX_AUXB_S      0x010000u
#define MIX_AUXB_S_RAMP 0x020000u
#define MIX_AUXC_L      0x040000u
#define MIX_AUXC_L_RAMP 0x080000u
#define MIX_AUXC_R      0x100000u
#define MIX_AUXC_R_RAMP 0x200000u
#define MIX_AUXC_S      0x400000u
#define MIX_AUXC_S_RAMP 0x800000u

/* Buffer indices into AXBufferSet::buf, in the order AXBuffers declares. */
enum {
    B_MAIN_L = 0, B_MAIN_R, B_MAIN_S,
    B_AUXA_L,     B_AUXA_R, B_AUXA_S,
    B_AUXB_L,     B_AUXB_R, B_AUXB_S,
    B_AUXC_L,     B_AUXC_R, B_AUXC_S
};

/* Dpop slots, in PBDpopWii order. */
enum {
    D_MAIN_L = 0, D_AUXA_L, D_AUXB_L, D_AUXC_L,
    D_MAIN_R,     D_AUXA_R, D_AUXB_R, D_AUXC_R,
    D_MAIN_S,     D_AUXA_S, D_AUXB_S, D_AUXC_S
};

/* ------------------------------------------------------------------ */
/* State                                                                */
/* ------------------------------------------------------------------ */

static u32 s_crc;
static int s_active;
static int s_old_axwii;         /* PB still carries the `updates` field   */
static int s_new_filter;        /* biquad + wiimote filtering supported   */
static int s_no_output_volume;  /* 0xd9c4bf34: OUTPUT has no volume word  */

static s32 s_main_l[AX_FRAME_SAMPLES], s_main_r[AX_FRAME_SAMPLES], s_main_s[AX_FRAME_SAMPLES];
static s32 s_auxA_l[AX_FRAME_SAMPLES], s_auxA_r[AX_FRAME_SAMPLES], s_auxA_s[AX_FRAME_SAMPLES];
static s32 s_auxB_l[AX_FRAME_SAMPLES], s_auxB_r[AX_FRAME_SAMPLES], s_auxB_s[AX_FRAME_SAMPLES];
static s32 s_auxC_l[AX_FRAME_SAMPLES], s_auxC_r[AX_FRAME_SAMPLES], s_auxC_s[AX_FRAME_SAMPLES];
static s32 s_wm[8][AX_WM_SAMPLES];

static u16 s_cmdlist[512];

static u16 s_last_main_volume = 0x8000;
static u16 s_last_aux_volume[3] = { 0x8000, 0x8000, 0x8000 };

static u64 s_frames, s_voices, s_voices_active, s_samples_read, s_audible;
static u32 s_peak;

static int ax_trace(void)
{
    static int t = -1;
    if (t < 0) t = getenv("AX_TRACE") != NULL;
    return t;
}

u32 ax_ucode_crc(void)      { return s_crc; }
int ax_active(void)         { return s_active; }
u64 ax_stat_frames(void)    { return s_frames; }
u64 ax_stat_voices(void)    { return s_voices; }
u64 ax_stat_active_voices(void) { return s_voices_active; }
u64 ax_stat_samples_read(void) { return s_samples_read; }
u64 ax_stat_audible_frames(void) { return s_audible; }
u32 ax_stat_peak(void)      { return s_peak; }

void ax_reset(void)
{
    s_crc = 0;
    s_active = 0;
    s_old_axwii = s_new_filter = s_no_output_volume = 0;
    memset(s_main_l, 0, sizeof s_main_l); memset(s_main_r, 0, sizeof s_main_r);
    memset(s_main_s, 0, sizeof s_main_s);
    memset(s_auxA_l, 0, sizeof s_auxA_l); memset(s_auxA_r, 0, sizeof s_auxA_r);
    memset(s_auxA_s, 0, sizeof s_auxA_s);
    memset(s_auxB_l, 0, sizeof s_auxB_l); memset(s_auxB_r, 0, sizeof s_auxB_r);
    memset(s_auxB_s, 0, sizeof s_auxB_s);
    memset(s_auxC_l, 0, sizeof s_auxC_l); memset(s_auxC_r, 0, sizeof s_auxC_r);
    memset(s_auxC_s, 0, sizeof s_auxC_s);
    memset(s_wm, 0, sizeof s_wm);
    s_last_main_volume = 0x8000;
    s_last_aux_volume[0] = s_last_aux_volume[1] = s_last_aux_volume[2] = 0x8000;
    s_frames = s_voices = s_voices_active = s_samples_read = s_audible = 0;
    s_peak = 0;
}

void ax_boot_ucode(u32 crc)
{
    s_crc = crc;
    s_old_axwii = (crc == 0xfa450138u || crc == 0x7699af32u);
    s_new_filter = (crc == 0x347112bau || crc == 0x4cc52064u);
    s_no_output_volume = (crc == 0xd9c4bf34u);
    switch (crc) {
    case 0x2ea36ce6u: case 0x5ef56da3u: case 0x347112bau: case 0xfa450138u:
    case 0xadbc06bdu: case 0x4cc52064u: case 0xd9c4bf34u: case 0x7699af32u:
        s_active = 1;
        break;
    default:
        /* Unknown ucode. Dolphin forces AXWii on Wii and warns; we decline
         * instead, because a wrong PB layout would corrupt the title's own
         * memory, which is worse than silence. */
        s_active = 0;
        break;
    }
    s_last_main_volume = 0x8000;
    s_last_aux_volume[0] = s_last_aux_volume[1] = s_last_aux_volume[2] = 0x8000;
    LOG_INFO(LOG_CORE, "AX ucode %08x: %s%s", (unsigned)crc,
             s_active ? "AXWii HLE active" : "unrecognised, HLE inactive",
             s_old_axwii ? " (old layout)" : "");
}

/* ------------------------------------------------------------------ */
/* Guest memory helpers                                                 */
/*                                                                      */
/* PB and command-list addresses are ordinary cached EAs. Accelerator     */
/* addresses are *physical*: bit 0x10000000 selects MEM2 (Dolphin           */
/* DSP.cpp::ReadARAM, whose Wii branch reads EXRAM for those and main RAM   */
/* otherwise -- the Wii has no real ARAM and AX reads samples straight out  */
/* of memory).                                                             */
/* ------------------------------------------------------------------ */

static u8 *ax_host_ptr(u32 ea, u32 len)
{
    if (mem_valid_span(ea) < len)
        return NULL;
    return (u8 *)mem_ptr(ea);
}

static u8 ax_ram8(u32 phys)
{
    if (phys & 0x10000000u) {
        u32 o = phys & (MEM2_SIZE - 1u);
        return g_mem.mem2 ? g_mem.mem2[o] : 0u;
    } else {
        u32 o = phys & 0x01FFFFFFu;
        return (g_mem.mem1 && o < MEM1_SIZE) ? g_mem.mem1[o] : 0u;
    }
}

/* ------------------------------------------------------------------ */
/* Accelerator (Dolphin Core/DSP/DSPAccelerator.cpp)                    */
/* ------------------------------------------------------------------ */

typedef struct {
    u32 start, end, cur;
    u16 format;
    s16 gain, yn1, yn2;
    u16 pred_scale;
    int reads_stopped;
    AXPBWii *pb;
} AXAccel;

#define ACC_START_END_MASK 0x3FFFFFFFu
#define ACC_CUR_MASK       0xBFFFFFFFu

static u16 acc_current_sample(AXAccel *a)
{
    unsigned size = a->format & 3u;      /* FormatSize */
    u16 val = 0;
    switch (size) {
    case 0:                              /* 4-bit */
        val = ax_ram8(a->cur >> 1);
        if (a->cur & 1) val &= 0xFu; else val >>= 4;
        break;
    case 1:                              /* 8-bit */
        val = ax_ram8(a->cur);
        break;
    case 2:                              /* 16-bit */
        val = (u16)((ax_ram8(a->cur * 2u) << 8) | ax_ram8(a->cur * 2u + 1u));
        break;
    default:
        break;                           /* garbage, but still steps the address */
    }
    return val;
}

static s16 acc_clamp16(s64 v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (s16)v;
}

/* The end-of-sample exception. Dolphin models it as a virtual call on the
 * accelerator; AX's override either restarts at the loop point or stops the
 * voice (AXVoice.h HLEAccelerator::OnSampleReadEndException). */
static void acc_on_end(AXAccel *a)
{
    AXPBWii *pb = a->pb;
    if (pb->looping) {
        a->pred_scale = (u16)(pb->loop_pred_scale & 0x7Fu);
        if (pb->is_stream != 1) {
            a->yn1 = (s16)pb->loop_yn1;
            a->yn2 = (s16)pb->loop_yn2;
            a->reads_stopped = 0;
        } else {
            /* Rewriting YN2 is what resumes reads on hardware; the values
             * themselves are unchanged for a stream. */
            a->reads_stopped = 0;
        }
    } else {
        pb->running = 0;
    }
}

static u16 acc_read_sample(AXAccel *a, const s16 *coefs)
{
    unsigned decode = (a->format >> 2) & 3u;
    unsigned gain_scale = (a->format >> 4) & 3u;
    s16 raw;
    u16 val = 0;
    u8 step = 0;
    int coef_idx;
    s32 coef1, coef2;

    if (a->reads_stopped)
        return 0;

    raw = (s16)acc_current_sample(a);
    coef_idx = (a->pred_scale >> 4) & 7;
    coef1 = coefs[coef_idx * 2 + 0];
    coef2 = coefs[coef_idx * 2 + 1];

    if (decode == 0) {                   /* ADPCM */
        s32 rs = raw & 0xF;
        s32 scale = 1 << (a->pred_scale & 0xF);
        s32 v32;
        if (rs >= 8) rs -= 16;
        v32 = (scale * rs) + ((0x400 + coef1 * a->yn1 + coef2 * a->yn2) >> 11);
        val = (u16)acc_clamp16(v32);
        step = 2;
        a->yn2 = a->yn1;
        a->yn1 = (s16)val;
        a->cur += 1;

        if ((a->end & 0xF) == 0x0 && a->cur == a->end) {
            a->cur = a->start + 1;
        } else if ((a->end & 0xF) == 0x1 && a->cur == a->end - 1) {
            a->cur = a->start;
        } else if ((a->cur & 15) == 0) {
            a->pred_scale = ax_ram8((a->cur & ~15u) >> 1);
            a->cur += 2;
            step = (u8)(step + 2);
        }
    } else {                             /* PCM, possibly through ACIN */
        u8 shift = 0;
        s32 v32;
        switch (gain_scale) {
        case 0: shift = 11; break;
        case 1: shift = 0;  break;
        case 2: shift = 16; break;
        default: break;
        }
        v32 = (((s32)a->gain * raw) >> shift) +
              (((coef1 * a->yn1) >> shift) + ((coef2 * a->yn2) >> shift));
        val = (u16)(s16)v32;
        a->yn2 = a->yn1;
        a->yn1 = (s16)val;
        step = 2;
        if (decode != 1)
            a->cur += 1;
    }

    if (a->cur == a->end + step - 1u) {
        a->cur = a->start;
        a->reads_stopped = 1;
        acc_on_end(a);
    }
    a->cur &= ACC_CUR_MASK;
    return val;
}

static void acc_setup(AXAccel *a, AXPBWii *pb)
{
    a->pb = pb;
    a->start = (((u32)pb->loop_addr_hi << 16) | pb->loop_addr_lo) & ACC_START_END_MASK;
    a->end   = (((u32)pb->end_addr_hi  << 16) | pb->end_addr_lo)  & ACC_START_END_MASK;
    a->cur   = (((u32)pb->cur_addr_hi  << 16) | pb->cur_addr_lo)  & ACC_CUR_MASK;
    a->format = pb->sample_format;
    a->gain = (s16)pb->gain;
    a->yn1 = pb->yn1;
    a->yn2 = pb->yn2;
    a->pred_scale = (u16)(pb->pred_scale & 0x7Fu);
    a->reads_stopped = 0;
}

/* ------------------------------------------------------------------ */
/* Resampling (Dolphin AXVoice.h ResampleAudio)                         */
/* ------------------------------------------------------------------ */

u32 ax_resample(AXSampleFn fn, void *ctx, s16 *out, u32 count,
                s16 *last_samples, u32 curr_pos, u32 ratio, int srctype)
{
    u32 read_count = 0;
    u32 i;

    if (srctype == AX_SRC_LINEAR || srctype == AX_SRC_POLYPHASE) {
        /* Polyphase needs the DSP coefficient ROM, which is not available
         * here; Dolphin takes this same branch whenever it cannot load
         * dsp_coef.bin. Linear interpolation is what the hardware does for
         * SRCTYPE_LINEAR and a close approximation otherwise. */
        s16 temp[4];
        u32 idx = 0;
        temp[idx++ & 3] = last_samples[0];
        temp[idx++ & 3] = last_samples[1];
        temp[idx++ & 3] = last_samples[2];
        temp[idx++ & 3] = last_samples[3];

        for (i = 0; i < count; ++i) {
            u16 frac, inv;
            s16 sample;
            curr_pos += ratio;
            while (curr_pos >= 0x10000u) {
                temp[idx++ & 3] = fn(ctx, read_count++);
                curr_pos -= 0x10000u;
            }
            frac = (u16)(curr_pos & 0xFFFFu);
            inv = (u16)(-(int)frac);
            if (frac) {
                s32 s0 = temp[idx++ & 3];
                s32 s1 = temp[idx++ & 3];
                sample = (s16)(((s0 * inv) + (s1 * frac)) >> 16);
                idx += 2;
            } else {
                sample = temp[idx++ & 3];
                idx += 3;
            }
            out[i] = sample;
        }
        last_samples[3] = temp[--idx & 3];
        last_samples[2] = temp[--idx & 3];
        last_samples[1] = temp[--idx & 3];
        last_samples[0] = temp[--idx & 3];
    } else {                              /* SRCTYPE_NEAREST */
        for (i = 0; i < count; ++i)
            out[i] = fn(ctx, i);
        if (count >= 4)
            memcpy(last_samples, out + count - 4, 4 * sizeof(s16));
    }
    return curr_pos;
}

typedef struct { const s16 *in; u32 n; } AXArraySrc;
static s16 ax_array_fn(void *ctx, u32 i)
{
    AXArraySrc *s = (AXArraySrc *)ctx;
    return (i < s->n) ? s->in[i] : 0;
}

u32 ax_resample_array(const s16 *in, s16 *out, u32 count, s16 *last_samples,
                      u32 curr_pos, u32 ratio, int srctype)
{
    AXArraySrc src;
    src.in = in;
    src.n = 0xFFFFFFFFu;
    return ax_resample(ax_array_fn, &src, out, count, last_samples,
                       curr_pos, ratio, srctype);
}

/* ------------------------------------------------------------------ */
/* Voice processing (Dolphin AXVoice.h ProcessVoice)                    */
/* ------------------------------------------------------------------ */

u32 ax_convert_mixer_control(u32 mc)
{
    u32 r = 0;
    if (mc & 0x00000001u) r |= MIX_MAIN_L;
    if (mc & 0x00000002u) r |= MIX_MAIN_R;
    if (mc & 0x00000004u) r |= MIX_MAIN_L | MIX_MAIN_R | MIX_MAIN_L_RAMP | MIX_MAIN_R_RAMP;
    if (mc & 0x00000008u) r |= MIX_MAIN_S;
    if (mc & 0x00000010u) r |= MIX_MAIN_S | MIX_MAIN_S_RAMP;
    if (mc & 0x00010000u) r |= MIX_AUXA_L;
    if (mc & 0x00020000u) r |= MIX_AUXA_R;
    if (mc & 0x00040000u) r |= MIX_AUXA_L | MIX_AUXA_R | MIX_AUXA_L_RAMP | MIX_AUXA_R_RAMP;
    if (mc & 0x00080000u) r |= MIX_AUXA_S;
    if (mc & 0x00100000u) r |= MIX_AUXA_S | MIX_AUXA_S_RAMP;
    if (mc & 0x00200000u) r |= MIX_AUXB_L;
    if (mc & 0x00400000u) r |= MIX_AUXB_R;
    if (mc & 0x00800000u) r |= MIX_AUXB_L | MIX_AUXB_R | MIX_AUXB_L_RAMP | MIX_AUXB_R_RAMP;
    if (mc & 0x01000000u) r |= MIX_AUXB_S;
    if (mc & 0x02000000u) r |= MIX_AUXB_S | MIX_AUXB_S_RAMP;
    if (mc & 0x04000000u) r |= MIX_AUXC_L;
    if (mc & 0x08000000u) r |= MIX_AUXC_R;
    if (mc & 0x10000000u) r |= MIX_AUXC_L | MIX_AUXC_R | MIX_AUXC_L_RAMP | MIX_AUXC_R_RAMP;
    if (mc & 0x20000000u) r |= MIX_AUXC_S;
    if (mc & 0x40000000u) r |= MIX_AUXC_S | MIX_AUXC_S_RAMP;
    return r;
}

static void ax_mix_add(s32 *out, const s16 *in, u32 count, AXVol *vd,
                       s16 *dpop, int ramp)
{
    u16 volume = vd->volume;
    u16 delta = ramp ? vd->volume_delta : 0;
    u32 i;
    for (i = 0; i < count; ++i) {
        s64 s = in[i];
        s16 s16v;
        s *= volume;
        s >>= 15;
        s16v = acc_clamp16(s);
        out[i] += s16v;
        volume = (u16)(volume + delta);
        *dpop = s16v;
    }
    vd->volume = volume;
}

static void ax_lowpass(s16 *samples, u32 count, u16 *on, s16 *yn1, u16 *a0, s16 *b0)
{
    u32 i;
    (void)on;
    for (i = 0; i < count; ++i) {
        s32 v = ((s32)*a0 * (s32)samples[i] + (s32)*b0 * (s32)*yn1) >> 15;
        *yn1 = samples[i] = acc_clamp16(v);
    }
}

static void ax_biquad(s16 *samples, u32 count, s16 *xn1, s16 *xn2, s16 *yn1,
                      s16 *yn2, s16 b0, s16 b1, s16 b2, s16 a1, s16 a2)
{
    u32 i;
    for (i = 0; i < count; ++i) {
        s16 xn0 = samples[i];
        s16 yn0;
        s64 tmp = 0;
        tmp += (s64)b0 * (s32)xn0;
        tmp += (s64)b1 * (s32)*xn1;
        tmp += (s64)b2 * (s32)*xn2;
        tmp += (s64)a1 * (s32)*yn1;
        tmp += (s64)a2 * (s32)*yn2;
        tmp <<= 2;
        if (tmp & 0x10000) tmp += 0x8000; else tmp += 0x7FFF;
        tmp >>= 16;
        yn0 = acc_clamp16(tmp);
        *xn2 = *xn1; *yn2 = *yn1; *xn1 = xn0; *yn1 = yn0;
        samples[i] = yn0;
    }
}

typedef struct { AXAccel *acc; const s16 *coefs; } AXVoiceSrc;
static s16 ax_voice_fn(void *ctx, u32 i)
{
    AXVoiceSrc *s = (AXVoiceSrc *)ctx;
    (void)i;
    s_samples_read++;
    return (s16)acc_read_sample(s->acc, s->coefs);
}

typedef struct { const s16 *in; } AXWmSrc;
static s16 ax_wm_fn(void *ctx, u32 i)
{
    return ((AXWmSrc *)ctx)->in[i];
}

void ax_process_voice(AXPBWii *pb, const AXBufferSet *bufs, u32 count,
                      int new_filter)
{
    s16 samples[AX_FRAME_SAMPLES];
    AXAccel acc;
    AXVoiceSrc src;
    u32 mctrl, curr_pos;
    u32 i;

    if (pb->running != 1)
        return;
    if (count > AX_FRAME_SAMPLES)
        count = AX_FRAME_SAMPLES;

    /* Fetch and rate-convert. */
    acc_setup(&acc, pb);
    src.acc = &acc;
    src.coefs = pb->coefs;
    curr_pos = ax_resample(ax_voice_fn, &src, samples, count, pb->last_samples,
                           pb->cur_addr_frac,
                           ((u32)pb->ratio_hi << 16) | pb->ratio_lo,
                           (int)pb->src_type);
    pb->cur_addr_frac = (u16)(curr_pos & 0xFFFFu);
    pb->cur_addr_hi = (u16)(acc.cur >> 16);
    pb->cur_addr_lo = (u16)acc.cur;
    pb->yn1 = acc.yn1;
    pb->yn2 = acc.yn2;
    pb->pred_scale = acc.pred_scale;

    /* Volume envelope: a per-sample ramp applied before any mixing. */
    for (i = 0; i < count; ++i) {
        /* Unsigned on Wii (Dolphin AXVoice.h ProcessVoice, AX_WII branch). */
        s32 volume = (u16)pb->cur_volume;
        s32 s = ((s32)samples[i] * volume) >> 15;
        samples[i] = acc_clamp16(s);
        pb->cur_volume = (s16)(pb->cur_volume + pb->cur_volume_delta);
    }

    if (pb->lpf_on)
        ax_lowpass(samples, count, &pb->lpf_on, &pb->lpf_yn1, &pb->lpf_a0, &pb->lpf_b0);
    if (new_filter && pb->bq_on)
        ax_biquad(samples, count, &pb->bq_xn1, &pb->bq_xn2, &pb->bq_yn1, &pb->bq_yn2,
                  pb->bq_b0, pb->bq_b1, pb->bq_b2, pb->bq_a1, pb->bq_a2);

    mctrl = ax_convert_mixer_control(((u32)pb->mixer_control_hi << 16) |
                                     pb->mixer_control_lo);

#define MIXTO(bit, bufi, vol, dp)                                              \
    if (mctrl & MIX_##bit)                                                     \
        ax_mix_add(bufs->buf[bufi], samples, count, &pb->vol, &pb->dpop[dp],   \
                   (mctrl & MIX_##bit##_RAMP) != 0)

    MIXTO(MAIN_L, B_MAIN_L, main_left,      D_MAIN_L);
    MIXTO(MAIN_R, B_MAIN_R, main_right,     D_MAIN_R);
    MIXTO(MAIN_S, B_MAIN_S, main_surround,  D_MAIN_S);
    MIXTO(AUXA_L, B_AUXA_L, auxA_left,      D_AUXA_L);
    MIXTO(AUXA_R, B_AUXA_R, auxA_right,     D_AUXA_R);
    MIXTO(AUXA_S, B_AUXA_S, auxA_surround,  D_AUXA_S);
    MIXTO(AUXB_L, B_AUXB_L, auxB_left,      D_AUXB_L);
    MIXTO(AUXB_R, B_AUXB_R, auxB_right,     D_AUXB_R);
    MIXTO(AUXB_S, B_AUXB_S, auxB_surround,  D_AUXB_S);
    MIXTO(AUXC_L, B_AUXC_L, auxC_left,      D_AUXC_L);
    MIXTO(AUXC_R, B_AUXC_R, auxC_right,     D_AUXC_R);
    MIXTO(AUXC_S, B_AUXC_S, auxC_surround,  D_AUXC_S);
#undef MIXTO

    /* Wiimote speaker path. */
    if (pb->remote && bufs->wm[0]) {
        s16 wm_samples[AX_WM_SAMPLES];
        AXWmSrc wsrc;
        u32 wm_count = (count == AX_FRAME_SAMPLES) ? 18u : 6u;
        u32 wpos;
        AXVol *wvol[8];
        s32 *wbuf[8];
        u32 c;

        if (new_filter && pb->riir[0] != 0) {
            if (pb->riir[0] == 2)
                ax_biquad(samples, count, (s16 *)&pb->riir[1], (s16 *)&pb->riir[2],
                          (s16 *)&pb->riir[3], (s16 *)&pb->riir[4],
                          (s16)pb->riir[5], (s16)pb->riir[6], (s16)pb->riir[7],
                          (s16)pb->riir[8], (s16)pb->riir[9]);
            else
                ax_lowpass(samples, count, &pb->riir[0], (s16 *)&pb->riir[1],
                           &pb->riir[2], (s16 *)&pb->riir[3]);
        }

        wsrc.in = samples;
        /* 0x55555 == 96/18 as closely as the fixed-point ratio allows. */
        wpos = ax_resample(ax_wm_fn, &wsrc, wm_samples, wm_count, pb->rsrc_last,
                           pb->rsrc_frac, 0x55555u, AX_SRC_POLYPHASE);
        pb->rsrc_frac = (u16)(wpos & 0xFFFFu);

        wvol[0] = &pb->rm_main0; wvol[1] = &pb->rm_aux0;
        wvol[2] = &pb->rm_main1; wvol[3] = &pb->rm_aux1;
        wvol[4] = &pb->rm_main2; wvol[5] = &pb->rm_aux2;
        wvol[6] = &pb->rm_main3; wvol[7] = &pb->rm_aux3;
        for (c = 0; c < 8; ++c) wbuf[c] = bufs->wm[c];
        for (c = 0; c < 8; ++c) {
            u32 f = (pb->remote_mixer_control >> (2 * c)) & 3u;
            if (f)
                ax_mix_add(wbuf[c], wm_samples, wm_count, wvol[c],
                           &pb->rdpop[c], (f & 2) != 0);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Parameter block transfer                                             */
/*                                                                      */
/* The Wii PB layout changed twice; Dolphin reads into the largest version   */
/* and skips the words the running ucode does not have (AXWii.cpp ReadPB /   */
/* WritePB). Expressed here as a list of (host word, count) runs consumed    */
/* in order from the guest image, which is the same thing with the offsets   */
/* written down once.                                                       */
/* ------------------------------------------------------------------ */

typedef struct { u16 host_word, count; } AXPBRun;

static u32 ax_pb_runs(AXPBRun *runs)
{
    if (s_old_axwii) {
        /* Everything up to the end of the (smaller) high-pass filter, then the
         * tail after the biquad's extra words. */
        runs[0].host_word = 0;   runs[0].count = (u16)(AXPB_FILT2_WORD + 4u);
        runs[1].host_word = (u16)(AXPB_FILT2_WORD + 10u);
        runs[1].count = (u16)(AXPB_WORDS - (AXPB_FILT2_WORD + 10u));
        return 2;
    }
    if (s_new_filter) {
        /* Only the updates block is missing. */
        runs[0].host_word = 0;   runs[0].count = AXPB_UPD_WORD;
        runs[1].host_word = (u16)(AXPB_UPD_WORD + 5u);
        runs[1].count = (u16)(AXPB_WORDS - (AXPB_UPD_WORD + 5u));
        return 2;
    }
    /* 0xd9c4bf34 / 0xadbc06bd: no updates and no biquad. */
    runs[0].host_word = 0;   runs[0].count = AXPB_UPD_WORD;
    runs[1].host_word = (u16)(AXPB_UPD_WORD + 5u);
    runs[1].count = (u16)(AXPB_FILT2_WORD + 4u - (AXPB_UPD_WORD + 5u));
    runs[2].host_word = (u16)(AXPB_FILT2_WORD + 10u);
    runs[2].count = (u16)(AXPB_WORDS - (AXPB_FILT2_WORD + 10u));
    return 3;
}

static u32 ax_pb_guest_words(void)
{
    AXPBRun runs[4];
    u32 n = ax_pb_runs(runs), i, t = 0;
    for (i = 0; i < n; i++) t += runs[i].count;
    return t;
}

static int ax_read_pb(u32 addr, AXPBWii *pb)
{
    AXPBRun runs[4];
    u32 n = ax_pb_runs(runs), i, j, off = 0;
    u16 *dst = (u16 *)pb;
    const u8 *src = ax_host_ptr(addr, ax_pb_guest_words() * 2u);
    if (!src)
        return 0;
    memset(pb, 0, sizeof *pb);
    for (i = 0; i < n; i++)
        for (j = 0; j < runs[i].count; j++, off += 2)
            dst[runs[i].host_word + j] = dol_be16(src + off);
    return 1;
}

static void ax_write_pb(u32 addr, const AXPBWii *pb)
{
    AXPBRun runs[4];
    u32 n = ax_pb_runs(runs), i, j, off = 0;
    const u16 *s = (const u16 *)pb;
    u8 *dst = ax_host_ptr(addr, ax_pb_guest_words() * 2u);
    if (!dst)
        return;
    for (i = 0; i < n; i++)
        for (j = 0; j < runs[i].count; j++, off += 2)
            dol_put_be16(dst + off, s[runs[i].host_word + j]);
}

/* ------------------------------------------------------------------ */
/* Command list                                                         */
/* ------------------------------------------------------------------ */

static void ax_fill_buffers(AXBufferSet *b)
{
    b->buf[B_MAIN_L] = s_main_l; b->buf[B_MAIN_R] = s_main_r; b->buf[B_MAIN_S] = s_main_s;
    b->buf[B_AUXA_L] = s_auxA_l; b->buf[B_AUXA_R] = s_auxA_r; b->buf[B_AUXA_S] = s_auxA_s;
    b->buf[B_AUXB_L] = s_auxB_l; b->buf[B_AUXB_R] = s_auxB_r; b->buf[B_AUXB_S] = s_auxB_s;
    b->buf[B_AUXC_L] = s_auxC_l; b->buf[B_AUXC_R] = s_auxC_r; b->buf[B_AUXC_S] = s_auxC_s;
    { int i; for (i = 0; i < 8; i++) b->wm[i] = s_wm[i]; }
}

/* SETUP: each of the twenty buffers is given a starting value and a per-sample
 * delta, or zeroed when the value is zero (Dolphin AX.h InitMixingBuffers). */
static void ax_setup_processing(u32 init_addr)
{
    static const u8 spms[20] = { 32,32,32, 32,32,32, 32,32,32, 32,32,32,
                                  6,6,6,6,6,6,6,6 };
    AXBufferSet b;
    s32 *ptrs[20];
    const u8 *p = ax_host_ptr(init_addr, 20u * 3u * 2u);
    unsigned i;

    ax_fill_buffers(&b);
    for (i = 0; i < 12; i++) ptrs[i] = b.buf[i];
    for (i = 0; i < 8; i++)  ptrs[12 + i] = b.wm[i];

    for (i = 0; i < 20; i++) {
        u32 n = (u32)spms[i] * 3u;
        s32 value = 0; s16 delta = 0;
        if (p) {
            value = (s32)(((u32)dol_be16(p + i * 6 + 0) << 16) |
                          dol_be16(p + i * 6 + 2));
            delta = (s16)dol_be16(p + i * 6 + 4);
        }
        if (value == 0) {
            memset(ptrs[i], 0, n * sizeof(s32));
        } else {
            u32 j;
            for (j = 0; j < n; j++) ptrs[i][j] = value + (s32)j * delta;
        }
    }
}

static void ax_add_to_lr(u32 addr, int neg)
{
    const u8 *p = ax_host_ptr(addr, AX_FRAME_SAMPLES * 4u);
    u32 i;
    if (!p) return;
    for (i = 0; i < AX_FRAME_SAMPLES; i++) {
        s32 v = (s32)dol_be32(p + i * 4);
        if (neg) v = -v;
        s_main_l[i] += v;
        s_main_r[i] += v;
    }
}

static void ax_add_sub_to_lr(u32 addr)
{
    const u8 *p = ax_host_ptr(addr, 2u * AX_FRAME_SAMPLES * 4u);
    u32 i;
    if (!p) return;
    for (i = 0; i < AX_FRAME_SAMPLES; i++)
        s_main_l[i] += (s32)dol_be32(p + i * 4);
    for (i = 0; i < AX_FRAME_SAMPLES; i++)
        s_main_r[i] -= (s32)dol_be32(p + (AX_FRAME_SAMPLES + i) * 4);
}

static void ax_process_pb_list(u32 pb_addr)
{
    AXBufferSet b;
    AXPBWii pb;
    unsigned guard = 0;

    ax_fill_buffers(&b);
    while (pb_addr && guard++ < 512u) {
        /* A title keeps its whole voice pool linked whether or not the voices
         * are playing -- MKWii walks 96 parameter blocks every 3 ms and at
         * most a handful are running. Reading the first ten words is enough to
         * find `running` and the next pointer, and skipping the other 280
         * bytes in each direction is bit-identical: a stopped voice is
         * untouched by the mixer, so writing it back would write exactly what
         * was read. On the console that is the difference between the mixer
         * costing a fraction of a percent and costing several. */
        const u8 *hdr = ax_host_ptr(pb_addr, 20u);
        if (!hdr)
            break;
        s_voices++;
        if (dol_be16(hdr + 16) != 1u) {          /* AXPBWii::running */
            pb_addr = ((u32)dol_be16(hdr) << 16) | dol_be16(hdr + 2);
            continue;
        }
        if (!ax_read_pb(pb_addr, &pb))
            break;
        s_voices_active++;
        ax_process_voice(&pb, &b, AX_FRAME_SAMPLES, s_new_filter);
        ax_write_pb(pb_addr, &pb);
        pb_addr = ((u32)pb.next_pb_hi << 16) | pb.next_pb_lo;
    }
}

/* Volume ramps interpolate between the previous frame's volume and this one's,
 * so a volume change does not step (Dolphin AXWii.cpp GenerateVolumeRamp). */
static void ax_volume_ramp(u16 *out, u16 v1, u16 v2, u32 n)
{
    float cur = (float)v1;
    u32 i;
    for (i = 0; i < n; i++) {
        cur += (float)((int)v2 - (int)v1) / (float)n;
        out[i] = (u16)cur;
    }
}

static void ax_mix_aux(int aux_id, u32 write_addr, u32 read_addr, u16 volume)
{
    u16 ramp[AX_FRAME_SAMPLES];
    s32 *main_bufs[3] = { s_main_l, s_main_r, s_main_s };
    const s32 *src[3];
    u32 i, j;

    ax_volume_ramp(ramp, s_last_aux_volume[aux_id], volume, AX_FRAME_SAMPLES);
    s_last_aux_volume[aux_id] = volume;

    switch (aux_id) {
    case 0: src[0] = s_auxA_l; src[1] = s_auxA_r; src[2] = s_auxA_s; break;
    case 1: src[0] = s_auxB_l; src[1] = s_auxB_r; src[2] = s_auxB_s; break;
    default: src[0] = s_auxC_l; src[1] = s_auxC_r; src[2] = s_auxC_s; break;
    }

    if (write_addr) {
        u8 *w = ax_host_ptr(write_addr, 3u * AX_FRAME_SAMPLES * 4u);
        if (w)
            for (i = 0; i < 3; i++)
                for (j = 0; j < AX_FRAME_SAMPLES; j++)
                    dol_put_be32(w + (i * AX_FRAME_SAMPLES + j) * 4, (u32)src[i][j]);
    }

    {
        const u8 *r = ax_host_ptr(read_addr, 3u * AX_FRAME_SAMPLES * 4u);
        if (!r) return;
        for (i = 0; i < 3; i++)
            for (j = 0; j < AX_FRAME_SAMPLES; j++) {
                s64 s = (s32)dol_be32(r + (i * AX_FRAME_SAMPLES + j) * 4);
                s *= ramp[j];
                main_bufs[i][j] += (s32)(s >> 15);
            }
    }
}

static void ax_upload_aux_mix_lrsc(int aux_id, const u32 *addr, u16 volume)
{
    s32 *aux_l = aux_id ? s_auxB_l : s_auxA_l;
    s32 *aux_r = aux_id ? s_auxB_r : s_auxA_r;
    s32 *aux_s = aux_id ? s_auxB_s : s_auxA_s;
    s32 *auxc   = aux_id ? s_auxC_s : s_auxC_r;
    s32 *dest[4] = { s_main_l, s_main_r, s_main_s, s_auxC_l };
    u16 ramp[AX_FRAME_SAMPLES];
    u32 i, k;
    u8 *w;

    w = ax_host_ptr(addr[0], 3u * AX_FRAME_SAMPLES * 4u);
    if (w) {
        for (i = 0; i < AX_FRAME_SAMPLES; i++) {
            dol_put_be32(w + i * 4, (u32)aux_l[i]);
            dol_put_be32(w + (AX_FRAME_SAMPLES + i) * 4, (u32)aux_r[i]);
            dol_put_be32(w + (2 * AX_FRAME_SAMPLES + i) * 4, (u32)aux_s[i]);
        }
    }
    w = ax_host_ptr(addr[1], AX_FRAME_SAMPLES * 4u);
    if (w)
        for (i = 0; i < AX_FRAME_SAMPLES; i++)
            dol_put_be32(w + i * 4, (u32)auxc[i]);

    ax_volume_ramp(ramp, s_last_aux_volume[aux_id], volume, AX_FRAME_SAMPLES);
    s_last_aux_volume[aux_id] = volume;

    for (k = 0; k < 4; k++) {
        const u8 *r = ax_host_ptr(addr[2 + k], AX_FRAME_SAMPLES * 4u);
        if (!r) continue;
        for (i = 0; i < AX_FRAME_SAMPLES; i++) {
            s64 s = (s32)dol_be32(r + i * 4);
            s *= ramp[i];
            dest[k][i] += (s32)(s >> 15);
        }
    }
}

/* The finished frame. Left and right are clamped to 16 bits, interleaved
 * *right first* (Dolphin AXWii.cpp OutputSamples), and written where the AI
 * audio DMA will pick them up. */
static void ax_output_samples(u32 lr_addr, u32 surround_addr, u16 volume,
                              int upload_auxc)
{
    u16 ramp[AX_FRAME_SAMPLES];
    u8 *p;
    u32 i;

    ax_volume_ramp(ramp, s_last_main_volume, volume, AX_FRAME_SAMPLES);
    s_last_main_volume = volume;

    p = ax_host_ptr(surround_addr, (upload_auxc ? 2u : 1u) * AX_FRAME_SAMPLES * 4u);
    if (p) {
        for (i = 0; i < AX_FRAME_SAMPLES; i++)
            dol_put_be32(p + i * 4, (u32)s_main_s[i]);
        if (upload_auxc)
            for (i = 0; i < AX_FRAME_SAMPLES; i++)
                dol_put_be32(p + (AX_FRAME_SAMPLES + i) * 4, (u32)s_auxC_l[i]);
    }

    for (i = 0; i < AX_FRAME_SAMPLES; i++) {
        s64 l = s_main_l[i], r = s_main_r[i];
        l = (l * ramp[i]) >> 15;
        r = (r * ramp[i]) >> 15;
        s_main_l[i] = acc_clamp16(l);
        s_main_r[i] = acc_clamp16(r);
    }

    p = ax_host_ptr(lr_addr, AX_FRAME_SAMPLES * 4u);
    if (p) {
        for (i = 0; i < AX_FRAME_SAMPLES; i++) {
            dol_put_be16(p + i * 4 + 0, (u16)(s16)s_main_r[i]);
            dol_put_be16(p + i * 4 + 2, (u16)(s16)s_main_l[i]);
        }
    }
    {   /* Cheap evidence that the mixer produced signal rather than a
         * correctly-timed silence: the loudest sample in the frame. */
        u32 peak = 0;
        for (i = 0; i < AX_FRAME_SAMPLES; i++) {
            s32 a = s_main_l[i] < 0 ? -s_main_l[i] : s_main_l[i];
            s32 b = s_main_r[i] < 0 ? -s_main_r[i] : s_main_r[i];
            if ((u32)a > peak) peak = (u32)a;
            if ((u32)b > peak) peak = (u32)b;
        }
        if (peak) s_audible++;
        if (peak > s_peak) s_peak = peak;
    }
    s_frames++;
}

static void ax_output_wm(const u32 *addr)
{
    static const int order[4] = { 0, 2, 4, 6 };   /* wm0, wm1, wm2, wm3 */
    u32 i, j;
    for (i = 0; i < 4; i++) {
        u8 *p = ax_host_ptr(addr[i], AX_WM_SAMPLES * 2u);
        if (!p) continue;
        for (j = 0; j < AX_WM_SAMPLES; j++)
            dol_put_be16(p + j * 2, (u16)acc_clamp16(s_wm[order[i]][j]));
    }
}

static u32 ax_copy_cmdlist(u32 addr, u16 size)
{
    const u8 *p;
    u32 i;
    if (size >= (u32)DOL_ARRAY_COUNT(s_cmdlist))
        return 0;
    p = ax_host_ptr(addr, (u32)size * 2u);
    if (!p)
        return 0;
    for (i = 0; i < size; i++)
        s_cmdlist[i] = dol_be16(p + i * 2);
    return size;
}

/* Command opcodes. The "old" AXWii kept a separate PB_ADDR command, which
 * shifts everything after it by one (Dolphin AXWii.h CmdType / CmdTypeOld). */
enum {
    CMD_SETUP = 0x00, CMD_ADD_TO_LR, CMD_SUB_TO_LR, CMD_ADD_SUB_TO_LR,
    CMD_PROCESS, CMD_MIX_AUXA, CMD_MIX_AUXB, CMD_MIX_AUXC,
    CMD_UPL_AUXA_MIX_LRSC, CMD_UPL_AUXB_MIX_LRSC, CMD_COMPRESSOR,
    CMD_OUTPUT, CMD_OUTPUT_DPL2, CMD_WM_OUTPUT, CMD_END
};

int ax_run_cmdlist(u32 addr, u16 size_words)
{
    u32 n, idx = 0;
    int outputs = 0;
    int end = 0;
    u32 pb_addr = 0;
    unsigned guard = 0;

    /* -1 means "the list was not executed at all", which the caller needs to
     * distinguish from "executed and produced no output frame": the first must
     * still be answered on the mailbox exactly as the pre-HLE stub answered it,
     * the second must not invent a sync the real ucode would not send. */
    if (!s_active)
        return -1;
    n = ax_copy_cmdlist(addr, size_words);
    if (!n)
        return -1;

#define NEXT() (idx < n ? s_cmdlist[idx++] : (u16)(end = 1, CMD_END))

    while (!end && guard++ < 2048u) {
        u16 cmd = NEXT();
        u16 hi, lo, hi2, lo2, volume;
        /* The old layout inserts PB_ADDR at 0x04 and pushes the rest up by
         * one; normalising here keeps a single switch below. */
        if (s_old_axwii) {
            if (cmd == 0x04) {           /* CMD_PB_ADDR_OLD */
                hi = NEXT(); lo = NEXT();
                pb_addr = ((u32)hi << 16) | lo;
                continue;
            }
            if (cmd > 0x04) cmd = (u16)(cmd - 1);
        }
        switch (cmd) {
        case CMD_SETUP:
            hi = NEXT(); lo = NEXT();
            ax_setup_processing(((u32)hi << 16) | lo);
            break;
        case CMD_ADD_TO_LR:
        case CMD_SUB_TO_LR:
            hi = NEXT(); lo = NEXT();
            ax_add_to_lr(((u32)hi << 16) | lo, cmd == CMD_SUB_TO_LR);
            break;
        case CMD_ADD_SUB_TO_LR:
            hi = NEXT(); lo = NEXT();
            ax_add_sub_to_lr(((u32)hi << 16) | lo);
            break;
        case CMD_PROCESS:
            if (s_old_axwii) {
                ax_process_pb_list(pb_addr);
            } else {
                hi = NEXT(); lo = NEXT();
                ax_process_pb_list(((u32)hi << 16) | lo);
            }
            break;
        case CMD_MIX_AUXA:
        case CMD_MIX_AUXB:
        case CMD_MIX_AUXC:
            volume = NEXT();
            hi = NEXT(); lo = NEXT(); hi2 = NEXT(); lo2 = NEXT();
            ax_mix_aux((int)(cmd - CMD_MIX_AUXA), ((u32)hi << 16) | lo,
                       ((u32)hi2 << 16) | lo2, volume);
            break;
        case CMD_UPL_AUXA_MIX_LRSC:
        case CMD_UPL_AUXB_MIX_LRSC: {
            u32 a[6];
            unsigned k;
            volume = NEXT();
            for (k = 0; k < 6; k++) { hi = NEXT(); lo = NEXT(); a[k] = ((u32)hi << 16) | lo; }
            ax_upload_aux_mix_lrsc(cmd == CMD_UPL_AUXB_MIX_LRSC, a, volume);
            break;
        }
        case CMD_COMPRESSOR:
            /* Threshold, frame count and table address. Dolphin implements the
             * table lookup; skipping it costs a little dynamic range on very
             * loud frames and nothing else, so the arguments are consumed and
             * the samples left alone. */
            (void)NEXT(); (void)NEXT(); (void)NEXT(); (void)NEXT();
            break;
        case CMD_OUTPUT:
        case CMD_OUTPUT_DPL2:
            volume = (s_old_axwii || s_no_output_volume) ? 0x8000u : NEXT();
            hi = NEXT(); lo = NEXT(); hi2 = NEXT(); lo2 = NEXT();
            ax_output_samples(((u32)hi2 << 16) | lo2, ((u32)hi << 16) | lo,
                              volume, cmd == CMD_OUTPUT_DPL2);
            outputs++;
            break;
        case CMD_WM_OUTPUT: {
            u32 a[4];
            unsigned k;
            for (k = 0; k < 4; k++) { hi = NEXT(); lo = NEXT(); a[k] = ((u32)hi << 16) | lo; }
            ax_output_wm(a);
            break;
        }
        case CMD_END:
            end = 1;
            break;
        default:
            /* An opcode we do not know cannot have its arguments skipped
             * safely, so stop rather than misparse the rest of the list. */
            LOG_WARN(LOG_CORE, "AX: unknown command %u at word %u", cmd, idx - 1);
            end = 1;
            break;
        }
    }
#undef NEXT

    if (ax_trace())
        fprintf(stderr, "[ax] list %08x words=%u outputs=%d voices=%llu\n",
                (unsigned)addr, (unsigned)n, outputs,
                (unsigned long long)s_voices);
    return outputs;
}
