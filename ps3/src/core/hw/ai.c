/* ai.c — the audio interface.
 *
 * Two things live here, and a title's audio system needs both:
 *
 *   * The sample counter and its interrupt (registers at 0xCC006C00). The
 *     counter runs at the *streaming* rate (AISFR) while playback is enabled,
 *     and the interrupt fires when it crosses the programmed target -- which is
 *     how the OS paces streaming audio.
 *   * The audio DMA, whose registers sit in the DSP's block at 0xCC005030 and
 *     whose completion raises AIDINT in the DSP control register. That
 *     completion is the heartbeat an audio frame loop waits on: it is the
 *     signal that says "the previous buffer has been consumed, here is the next
 *     boundary". A title that never sees it simply stops.
 *
 * The two clocks are *different registers with opposite senses*, which is the
 * single most load-bearing fact in this file (Dolphin AudioInterface.cpp:64-70
 * and AudioInterface.h:72-79):
 *
 *      AISFR (bit 1, 0x02):  0 = 32 kHz, 1 = 48 kHz   -- sample counter
 *      AIDFR (bit 6, 0x40):  0 = 48 kHz, 1 = 32 kHz   -- DMA/DAC drain rate
 *
 * Getting AIDFR wrong is not a sound bug, it is a *timing* bug: the audio DMA
 * block clock next door is derived from it, the AX frame period is derived from
 * that, and a title's music, streams and any logic paced off the audio frame
 * run fast or slow by exactly the ratio of the error.
 */
#include "hardware.h"
#include "../mem/memmap.h"
#include "../core_timing.h"
#include "../../common/log.h"

#include <stdio.h>
#include <stdlib.h>

/* Registers, offsets from 0xCC006C00. */
#define AI_CONTROL   0x00
#define AI_VOLUME    0x04
#define AI_SAMPLE_CNT 0x08
#define AI_INT_TIMING 0x0C

/* AI_CONTROL bits (Dolphin AudioInterface.h, union AICR). */
#define AICR_PSTAT     0x0001   /* sample counter / playback enable        */
#define AICR_AISFR     0x0002   /* 0 = 32 kHz, 1 = 48 kHz  (stream clock)  */
#define AICR_AIINTMSK  0x0004   /* sample-counter interrupt enable         */
#define AICR_AIINT     0x0008   /* interrupt status, write 1 to clear      */
#define AICR_AIINTVLD  0x0010
#define AICR_SCRESET   0x0020   /* clear the sample counter                */
#define AICR_AIDFR     0x0040   /* 0 = 48 kHz, 1 = 32 kHz  (DAC/DMA clock) */

/* Bits the guest may store. AIINT is write-1-to-clear and SCRESET is a
 * command, so neither is written through. */
#define AICR_STORED (AICR_PSTAT | AICR_AISFR | AICR_AIINTMSK | \
                     AICR_AIINTVLD | AICR_AIDFR)

static u32 s_control, s_volume, s_int_timing;

/* The counter is *accumulated across re-bases* rather than derived from a
 * single origin, because its rate can change underneath it: MKWii's own SRC
 * calibration (NTSC 0x8012429c) flips AISFR between two counter edges and times
 * them. Deriving from one origin would make the whole counter jump by the rate
 * ratio at that moment; accumulating means only the slope changes, which is
 * what the hardware does. */
static u32 s_count_base;        /* samples counted before the last re-base */
static u64 s_base_tb;           /* time base at the last re-base           */
static u32 s_irq_seen;          /* counter value at the last interrupt check */
static int s_ev_tick = -1;      /* scheduler handle for the counter poll   */

static int ai_trace(void)
{
    static int t = -1;
    if (t < 0) t = getenv("AI_TRACE") != NULL;
    return t;
}

/* The streaming clock: what the sample counter counts at. */
static u32 ai_stream_rate(void)
{
    /* The sense of this bit is load-bearing: MKWii's AI calibration flips it
     * and *measures* that the counter rate changes the right way, retrying
     * forever when it does not. Inverted, that loop is an infinite white
     * screen at the end of the strap phase. */
    return (s_control & AICR_AISFR) ? 48000u : 32000u;
}

/* The DAC rate, for the audio DMA next door: the DSP block owns the DMA
 * registers but the AI owns the clock that drains them.
 *
 * AIDFR is a *different bit from AISFR and its sense is inverted*
 * (Dolphin AudioInterface.cpp:67-69, AID_32KHz = 1 / AID_48KHz = 0). This used
 * to read the streaming bit, which has neither the right position nor the right
 * polarity. On MKWii specifically the two mistakes cancel in the steady state --
 * AIInit leaves AICR at 0x40, so AISFR reads clear (32 kHz by the old rule) and
 * AIDFR reads set (32 kHz by the right one) -- which is why the audio frame
 * clock was already 3 ms before the fix and why the bug survived this long. It
 * is a real bug all the same, and it is live for any title that streams at
 * 48 kHz while its DAC runs at 32 kHz, i.e. AICR = 0x42: the old rule reports
 * 48 kHz there and the audio DMA drains 1.5x too fast. That combination is
 * Dolphin's own power-on state (AudioInterface.cpp:196-199), so it is not an
 * exotic one.
 *
 * MKWii independently proves the 32 kHz answer is the right one, from its own
 * register writes: it programs a 12-block (384-byte) DMA buffer, which is 96
 * stereo samples, which is exactly one 3 ms AX frame at 32 kHz. */
u32 ai_dac_rate(void)
{
    return (s_control & AICR_AIDFR) ? 32000u : 48000u;
}

static u32 ai_tb_hz(void)
{
    /* bus/4; 60.75 MHz on Wii, 40.5 MHz on GameCube. Derived rather than
     * hardcoded so GameCube mode is not silently 1.5x wrong. */
    return g_bus_hz / TB_DIVISOR;
}

/* Samples elapsed since the last re-base. Only advances while PSTAT is set:
 * the counter counts samples the DAC has actually consumed, and a stopped DAC
 * consumes none (Dolphin AudioInterface.cpp:IncreaseSampleCount, which returns
 * immediately when !IsPlaying). */
static u32 ai_sample_count(void)
{
    u64 tb, elapsed;
    if (!(s_control & AICR_PSTAT))
        return s_count_base;
    tb = timing_timebase();
    elapsed = (tb > s_base_tb) ? tb - s_base_tb : 0;
    return s_count_base + (u32)((elapsed * ai_stream_rate()) / ai_tb_hz());
}

/* Fold everything counted so far into the base and restart the clock. Called
 * before any change that alters the counter's slope or its enable. */
static void ai_rebase(void)
{
    s_count_base = ai_sample_count();
    s_base_tb = timing_timebase();
}

static void ai_update_irq(void)
{
    int line = (s_control & AICR_AIINT) && (s_control & AICR_AIINTMSK);
    pi_set_interrupt(PI_INT_AI, line ? 1 : 0);
}

/* Called from the scheduler: raise the interrupt when the counter *crosses*
 * the programmed target.
 *
 * The crossing test is Dolphin's (AudioInterface.cpp:IncreaseSampleCount) and
 * is not the same as `count >= target`. `>=` re-arms the instant the guest
 * acknowledges AIINT without reprogramming the target, which is a spurious
 * interrupt storm; the unsigned difference form fires exactly once per pass
 * and stays correct across the counter's 32-bit wrap. */
void ai_update(void)
{
    u32 cur, old;
    if (!(s_control & AICR_PSTAT))
        return;
    cur = ai_sample_count();
    old = s_irq_seen + 1u;
    if ((u32)(s_int_timing - old) <= (u32)(cur - old)) {
        s_control |= AICR_AIINT;
        ai_update_irq();
    }
    s_irq_seen = cur;
}

static u32 ai_read(u32 addr, unsigned size, void *ctx)
{
    (void)size; (void)ctx;
    switch (addr - 0xCC006C00u) {
    case AI_CONTROL:    return s_control;
    case AI_VOLUME:     return s_volume;
    case AI_SAMPLE_CNT: return ai_sample_count();
    case AI_INT_TIMING: return s_int_timing;
    default:            return 0;
    }
}

static void ai_write(u32 addr, u32 value, unsigned size, void *ctx)
{
    (void)size; (void)ctx;
    switch (addr - 0xCC006C00u) {
    case AI_CONTROL: {
        /* The interrupt bit is write-one-to-clear; the reset bit is a command
         * rather than state. Everything else is stored. */
        u32 status = s_control & AICR_AIINT & ~(value & AICR_AIINT);
        ai_rebase();                    /* settle the old slope first */
        s_control = (value & AICR_STORED) | status;
        if (value & AICR_SCRESET) {
            s_count_base = 0;
            s_base_tb = timing_timebase();
        }
        s_irq_seen = ai_sample_count();
        if (ai_trace())
            fprintf(stderr, "[ai] cr<-%08x stream=%u dac=%u pstat=%d\n",
                    (unsigned)value, ai_stream_rate(), ai_dac_rate(),
                    (s_control & AICR_PSTAT) ? 1 : 0);
        ai_update_irq();
        break;
    }
    case AI_VOLUME:
        s_volume = value;
        break;
    case AI_SAMPLE_CNT:
        /* Writable on hardware; Dolphin registers a ComplexWrite that stores
         * the value and restarts the sample clock from it. */
        ai_rebase();
        s_count_base = value;
        s_base_tb = timing_timebase();
        s_irq_seen = value;
        break;
    case AI_INT_TIMING:
        s_int_timing = value;
        if (ai_trace())
            fprintf(stderr, "[ai] timing<-%08x count=%u\n",
                    (unsigned)value, (unsigned)ai_sample_count());
        break;
    default:
        break;
    }
}

/* The sample counter is derived on read, so nothing needs ticking to keep it
 * right -- but its *interrupt* has to be noticed by somebody, and the only
 * thing that can notice is the scheduler. Poll it once a millisecond of guest
 * time: fine enough that AIRegisterSampleCallback's period (typically several
 * milliseconds) lands within a millisecond of where it should, cheap enough
 * to be invisible. Without this the interrupt simply never fires, which is
 * silent: nothing crashes, the title's audio clock just never starts.
 *
 * Dolphin instead schedules the event *at* the crossing (AudioInterface.cpp
 * GetAIPeriod), which is exact rather than within a millisecond. The
 * difference is only visible to a title that both enables streaming playback
 * and programs a target; MKWii's AIInit leaves PSTAT clear when it finishes,
 * so on this title the interrupt never arms at all and the cheaper poll costs
 * nothing. Worth revisiting for a title that streams from the disc. */
static void ai_on_tick(u64 userdata, s64 cycles_late)
{
    (void)userdata;
    ai_update();
    timing_schedule(s_ev_tick, (s64)(g_cpu_hz / 1000u) - cycles_late, 0);
}

void ai_reset(void)
{
    /* Reset leaves AIDFR clear, i.e. a 48 kHz DAC. That is deliberately *not*
     * Dolphin's power-on value (its Init() presets AISFR=48 kHz and
     * AIDFR=32 kHz, AudioInterface.cpp:196-199): MKWii's AIInit reads AIDFR
     * back and only runs its SRC calibration when it finds the bit clear
     * (NTSC 0x80124108), so presetting it would make the title skip a path it
     * exercises on this machine today. Both spellings converge -- AIInit
     * leaves AIDFR set either way -- and nothing drains the DMA before then,
     * so the observable difference is confined to that calibration. */
    s_control = s_volume = s_int_timing = 0;
    s_count_base = 0;
    s_irq_seen = 0;
    s_base_tb = timing_timebase();
    if (s_ev_tick >= 0)
        timing_schedule(s_ev_tick, (s64)(g_cpu_hz / 1000u), 0);
}

void ai_init(void)
{
    /* Always re-register: timing_init clears the event table, so a handle
     * cached from a previous machine would refer to nothing. */
    s_ev_tick = timing_register_event("AI sample counter", ai_on_tick);
    ai_reset();
    mmio_register(0xCC006C00u, 0x20, ai_read, ai_write, NULL, "AI");
}
