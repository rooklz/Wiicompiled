/* audio_ps3.c — the console end of the audio path: libaudio.
 *
 * The emulator produces stereo 16-bit samples at the Wii's DAC rate (32 kHz
 * once a title's AIInit has run) into a ring buffer (core/hw/audio_out.c). The
 * PS3's audio port wants 32-bit float at 48 kHz in fixed 256-frame blocks. This
 * file is the adapter and nothing more -- even the rate conversion lives in the
 * core, where a test without a console can reach it, so what remains here is
 * exactly the part that only hardware can confirm.
 *
 * The port is driven by polling rather than by blocking on its notify event
 * queue. Blocking would have to happen on the emulator's own thread -- the only
 * thread there is -- and a 20 ms wait inside the run loop is a 20 ms stall of
 * the emulated CPU. Polling costs one load per slice, and the port's own eight
 * blocks (about 43 ms at 48 kHz) absorb the jitter the run loop's uneven
 * cadence introduces.
 *
 * NOT VERIFIED ON HARDWARE. The libaudio call sequence is written against the
 * PSL1GHT headers and can only be confirmed on a console, so it is fail-safe by
 * construction: an error at any step disables the sink and leaves the emulator
 * running exactly as it did before, and the whole thing is skipped when
 * g_audio_enable is zero.
 */
#include "../../core/hw/audio_out.h"
#include "../../common/log.h"

#ifndef EMU_AUDIO_DEFAULT
#define EMU_AUDIO_DEFAULT 1
#endif

/* The toggle. Clear it before audio_ps3_init() -- or build with
 * -DEMU_AUDIO_DEFAULT=0 -- to keep the console completely silent and the audio
 * port unopened. */
int g_audio_enable = EMU_AUDIO_DEFAULT;

int  audio_ps3_init(void);
void audio_ps3_update(void);
void audio_ps3_shutdown(void);
void audio_ps3_stats(unsigned *blocks, unsigned *underruns, unsigned *queued);

#ifdef __PS3__

#include <audio/audio.h>
#include <sys/event_queue.h>
#include <string.h>

static int s_ready;
static u32 s_port;
static audioPortConfig s_config;
static sys_event_queue_t s_queue;
static sys_ipc_key_t s_key;
static int s_have_queue;
static u64 s_write_block;
static unsigned s_blocks_written;

int audio_ps3_init(void)
{
    audioPortParam params;

    if (!g_audio_enable || s_ready)
        return 0;

    if (audioInit() != 0) {
        LOG_WARN(LOG_CORE, "audio: audioInit failed, running silent");
        g_audio_enable = 0;
        return -1;
    }

    memset(&params, 0, sizeof params);
    params.numChannels = AUDIO_PORT_2CH;
    params.numBlocks   = AUDIO_BLOCK_8;
    params.attrib      = 0;
    params.level       = 1.0f;

    if (audioPortOpen(&params, &s_port) != 0) {
        LOG_WARN(LOG_CORE, "audio: audioPortOpen failed, running silent");
        audioQuit();
        g_audio_enable = 0;
        return -1;
    }
    if (audioGetPortConfig(s_port, &s_config) != 0) {
        LOG_WARN(LOG_CORE, "audio: audioGetPortConfig failed, running silent");
        audioPortClose(s_port);
        audioQuit();
        g_audio_enable = 0;
        return -1;
    }

    /* The notify queue is part of the documented open sequence even though we
     * never block on it; draining it up front keeps it from filling. */
    if (audioCreateNotifyEventQueue(&s_queue, &s_key) == 0) {
        s_have_queue = 1;
        audioSetNotifyEventQueue(s_key);
        sysEventQueueDrain(s_queue);
    }

    if (audioPortStart(s_port) != 0) {
        LOG_WARN(LOG_CORE, "audio: audioPortStart failed, running silent");
        if (s_have_queue) {
            audioRemoveNotifyEventQueue(s_key);
            sysEventQueueDestroy(s_queue, 0);
            s_have_queue = 0;
        }
        audioPortClose(s_port);
        audioQuit();
        g_audio_enable = 0;
        return -1;
    }

    s_write_block = 0;
    s_ready = 1;
    audio_out_set_enabled(1);
    LOG_INFO(LOG_CORE, "audio: port %u open, %llu blocks of %d frames",
             (unsigned)s_port, (unsigned long long)s_config.numBlocks,
             AUDIO_BLOCK_SAMPLES);
    return 0;
}

void audio_ps3_update(void)
{
    u64 read_block, nblocks;

    if (!s_ready || !g_audio_enable)
        return;

    nblocks = s_config.numBlocks;
    if (nblocks < 2)
        return;

    /* readIndex is the address of a u64 the audio server keeps updated with
     * the block it is currently playing. */
    read_block = *(volatile u64 *)(u64)s_config.readIndex;
    if (read_block >= nblocks)
        read_block = 0;

    /* Keep the port one block short of full: writing into the block being
     * played is what produces the classic tearing buzz. */
    for (;;) {
        u64 ahead = (s_write_block + nblocks - read_block) % nblocks;
        float *dst;
        if (ahead >= nblocks - 1)
            break;
        dst = (float *)((u64)s_config.audioDataStart +
                        s_write_block * s_config.channelCount *
                            AUDIO_BLOCK_SAMPLES * sizeof(float));
        /* The port is a fixed 48 kHz; the emulator feeds 32 kHz once the title
         * has programmed AIDFR. The conversion lives in the core so it can be
         * tested without a console. */
        audio_out_render_f32(dst, AUDIO_BLOCK_SAMPLES, 48000u);
        s_write_block = (s_write_block + 1) % nblocks;
        s_blocks_written++;
    }
}

void audio_ps3_shutdown(void)
{
    if (!s_ready)
        return;
    s_ready = 0;
    audio_out_set_enabled(0);
    audioPortStop(s_port);
    if (s_have_queue) {
        audioRemoveNotifyEventQueue(s_key);
        sysEventQueueDestroy(s_queue, 0);
        s_have_queue = 0;
    }
    audioPortClose(s_port);
    audioQuit();
}

void audio_ps3_stats(unsigned *blocks, unsigned *underruns, unsigned *queued)
{
    if (blocks)    *blocks = s_blocks_written;
    if (underruns) *underruns = (unsigned)audio_out_underruns();
    if (queued)    *queued = audio_out_available();
}

#else  /* not the console: the same interface, doing nothing */

int  audio_ps3_init(void)     { return 0; }
void audio_ps3_update(void)   { }
void audio_ps3_shutdown(void) { }
void audio_ps3_stats(unsigned *blocks, unsigned *underruns, unsigned *queued)
{
    if (blocks)    *blocks = 0;
    if (underruns) *underruns = (unsigned)audio_out_underruns();
    if (queued)    *queued = audio_out_available();
}

#endif
