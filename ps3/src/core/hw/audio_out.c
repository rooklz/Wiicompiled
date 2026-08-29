/* audio_out.c — see audio_out.h. */
#include "audio_out.h"

#include <string.h>

/* Interleaved left, right in host order. */
static s16 s_ring[AUDIO_RING_FRAMES * 2u];
static volatile u32 s_head;     /* next frame index the producer will write */
static volatile u32 s_tail;     /* next frame index the consumer will read  */
static int s_enabled;
static u32 s_rate = 32000u;
static u64 s_pushed, s_dropped, s_underruns;

/* Resampler state, carried across calls so consecutive blocks join without a
 * seam: the phase between two input frames, 16.16 fixed point, and the pair of
 * frames being interpolated between. */
static u32 s_phase;
static s16 s_cur[2], s_nxt[2];
static int s_primed;

/* Indices are free-running and only ever compared by difference, so the ring
 * never needs a "full vs empty" flag and wrapping is not a special case. */
#define RING_MASK (AUDIO_RING_FRAMES - 1u)

void audio_out_reset(void)
{
    s_head = s_tail = 0;
    s_pushed = s_dropped = s_underruns = 0;
    s_rate = 32000u;
    s_phase = 0;
    s_primed = 0;
    s_cur[0] = s_cur[1] = s_nxt[0] = s_nxt[1] = 0;
    memset(s_ring, 0, sizeof s_ring);
}

void audio_out_set_enabled(int on) { s_enabled = on ? 1 : 0; }
int  audio_out_enabled(void)       { return s_enabled; }
void audio_out_set_rate(u32 hz)    { if (hz) s_rate = hz; }
u32  audio_out_rate(void)          { return s_rate; }
u64  audio_out_pushed(void)        { return s_pushed; }
u64  audio_out_dropped(void)       { return s_dropped; }
u64  audio_out_underruns(void)     { return s_underruns; }

unsigned audio_out_available(void)
{
    return (unsigned)(s_head - s_tail);
}

void audio_out_push_be_rl(const void *src, unsigned frames)
{
    const u8 *p = (const u8 *)src;
    u32 head, used;
    unsigned i;

    if (!s_enabled || !src)
        return;

    head = s_head;
    used = head - s_tail;
    if (used + frames > AUDIO_RING_FRAMES) {
        unsigned room = (used >= AUDIO_RING_FRAMES) ? 0u
                                                    : AUDIO_RING_FRAMES - used;
        s_dropped += frames - room;
        frames = room;
        if (!frames)
            return;
    }

    for (i = 0; i < frames; i++) {
        /* The DMA buffer is big-endian and right channel first (Dolphin
         * Mixer::PushSamples: "Big-endian RL-ordered stereo samples"). */
        s16 r = (s16)dol_be16(p + i * 4 + 0);
        s16 l = (s16)dol_be16(p + i * 4 + 2);
        u32 slot = (head + i) & RING_MASK;
        s_ring[slot * 2 + 0] = l;
        s_ring[slot * 2 + 1] = r;
    }

    /* Publish the samples before the index that makes them visible. */
    __sync_synchronize();
    s_head = head + frames;
    s_pushed += frames;
}

unsigned audio_out_pop(s16 *dst, unsigned frames)
{
    u32 tail = s_tail;
    u32 avail = s_head - tail;
    unsigned i;

    __sync_synchronize();
    if (frames > avail)
        frames = (unsigned)avail;
    for (i = 0; i < frames; i++) {
        u32 slot = (tail + i) & RING_MASK;
        dst[i * 2 + 0] = s_ring[slot * 2 + 0];
        dst[i * 2 + 1] = s_ring[slot * 2 + 1];
    }
    s_tail = tail + frames;
    return frames;
}

static void advance_input(void)
{
    s16 f[2];
    s_cur[0] = s_nxt[0];
    s_cur[1] = s_nxt[1];
    if (audio_out_pop(f, 1) == 1) {
        s_nxt[0] = f[0];
        s_nxt[1] = f[1];
    } else {
        s_underruns++;              /* hold the last frame */
    }
}

void audio_out_render_f32(float *dst, unsigned frames, u32 out_rate)
{
    u32 in_rate = s_rate ? s_rate : 32000u;
    u32 step;
    unsigned i;

    if (!out_rate)
        out_rate = 48000u;
    /* 16.16 input frames consumed per output frame. */
    step = (u32)(((u64)in_rate << 16) / out_rate);

    if (!s_primed) {
        s16 f[2];
        if (audio_out_pop(f, 1) == 1) { s_cur[0] = f[0]; s_cur[1] = f[1]; }
        if (audio_out_pop(f, 1) == 1) { s_nxt[0] = f[0]; s_nxt[1] = f[1]; }
        s_primed = 1;
        s_phase = 0;
    }

    for (i = 0; i < frames; i++) {
        u32 frac = s_phase & 0xFFFFu;
        s32 l = (s32)s_cur[0] +
                (s32)((((s64)s_nxt[0] - (s64)s_cur[0]) * (s64)frac) >> 16);
        s32 r = (s32)s_cur[1] +
                (s32)((((s64)s_nxt[1] - (s64)s_cur[1]) * (s64)frac) >> 16);
        dst[i * 2 + 0] = (float)l * (1.0f / 32768.0f);
        dst[i * 2 + 1] = (float)r * (1.0f / 32768.0f);

        s_phase += step;
        while (s_phase >= 0x10000u) {
            advance_input();
            s_phase -= 0x10000u;
        }
    }
}
