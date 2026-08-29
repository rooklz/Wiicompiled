/* audio_out.h — the emulator's audio sink: one ring buffer, one producer.
 *
 * The producer is the audio DMA block clock in dsp.c: every 32 bytes the AI
 * drains out of main memory are 8 stereo frames, and they are pushed here in
 * exactly the order the hardware would have handed them to the DAC. The
 * consumer is whatever the platform provides -- libaudio on the console, a file
 * or nothing at all off it.
 *
 * Keeping the ring in portable core code rather than in the PS3 backend is what
 * makes the audio path testable without a console: a host test can drive the
 * DMA and read the frames back out, and the console build adds only the part
 * that genuinely cannot be tested here.
 *
 * Concurrency: single producer, single consumer, no locks. The console's audio
 * callback runs on its own thread, and a mutex on a 3 ms deadline is exactly
 * the sort of thing that produces dropouts under load.
 */
#ifndef DOLPHIN_CORE_HW_AUDIO_OUT_H
#define DOLPHIN_CORE_HW_AUDIO_OUT_H

#include "../../common/types.h"

/* Frames of stereo audio the ring holds. 8192 at 32 kHz is 256 ms, which is
 * far more than any sane latency target but costs 32 KiB and means a long
 * host-side stall (a shader compile, a disc seek) does not lose samples. */
#define AUDIO_RING_FRAMES 8192u

void audio_out_reset(void);

/* Enable/disable the sink. Disabled is the default so that nothing about the
 * emulator's behaviour changes until the console side is deliberately turned
 * on; pushes are dropped cheaply while disabled. */
void audio_out_set_enabled(int on);
int  audio_out_enabled(void);

/* The rate the producer is currently feeding at, in Hz -- the AI's DAC rate.
 * The consumer needs it to resample to whatever the host wants. */
void audio_out_set_rate(u32 hz);
u32  audio_out_rate(void);

/* Push `frames` stereo frames from a guest-order (big-endian, right channel
 * first) buffer -- i.e. straight out of the audio DMA. Frames that do not fit
 * are dropped, and counted. */
void audio_out_push_be_rl(const void *src, unsigned frames);

/* Pop up to `frames` stereo frames as host-order interleaved left, right.
 * Returns how many were actually available. */
unsigned audio_out_pop(s16 *dst, unsigned frames);

unsigned audio_out_available(void);
u64      audio_out_pushed(void);
u64      audio_out_dropped(void);

/* Render `frames` interleaved 32-bit float stereo frames at `out_rate`,
 * resampling from whatever the producer is feeding at. The console's audio
 * port is a fixed 48 kHz and the Wii's DAC is 32 kHz, so something has to do
 * this conversion; doing it here rather than in the platform backend is what
 * lets it be tested without a console. Samples are in [-1, 1].
 *
 * Always writes `frames` frames. When the ring is starved the last frame is
 * held rather than a click being emitted, and the shortfall is counted. */
void audio_out_render_f32(float *dst, unsigned frames, u32 out_rate);
u64  audio_out_underruns(void);

#endif /* DOLPHIN_CORE_HW_AUDIO_OUT_H */
