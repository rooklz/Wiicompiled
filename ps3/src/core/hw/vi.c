/* vi.c — video interface: scanout timing and the frame heartbeat.
 *
 * The VI is not a renderer. It scans an external framebuffer out to the
 * display, and in doing so provides the clock essentially every title is built
 * around: four programmable display interrupts that fire at chosen scanlines,
 * one of which is conventionally placed at the start of vertical blank.
 *
 * Getting the *rate* right matters more than getting the registers complete.
 * A title paces animation, audio streaming and input polling off these
 * interrupts, so a field time that is off by a percent shows up as everything
 * running fractionally slow -- the classic emulator that reports 60 fps while
 * feeling wrong. The field time here is therefore derived from the real line
 * count and line rate rather than from a rounded 16.67 ms.
 */
unsigned g_vi_irqs;
#include "hardware.h"
#include "../core_timing.h"
#include "../mem/memmap.h"
#include "../../common/log.h"

#include <string.h>

/* Register offsets from HW_VI_BASE. Only the ones with observable behaviour
 * are modelled; the rest read back what was written, which is what titles
 * expect for configuration they set and never inspect. */
#define VI_VTR          0x00    /* vertical timing            */
#define VI_DCR          0x02    /* display configuration      */
#define VI_HTR0         0x04
#define VI_TFBL         0x1C    /* top field base, left       */
#define VI_TFBR         0x20
#define VI_BFBL         0x24    /* bottom field base, left    */
#define VI_BFBR         0x28
#define VI_DPV          0x2C    /* current vertical position  */
#define VI_DPH          0x2E    /* current horizontal position*/
#define VI_DI0          0x30    /* display interrupt 0        */
#define VI_DI1          0x34
#define VI_DI2          0x38
#define VI_DI3          0x3C
#define VI_PICCONF      0x48    /* picture configuration: WPL + STD */

#define VI_NUM_DI       4

/* Display-interrupt register layout:
 *   bits 0..9   horizontal position
 *   bits 16..26 vertical position (line)
 *   bit  28     interrupt enable
 *   bit  31     interrupt status (write 0 to acknowledge) */
#define VI_DI_ENABLE    0x10000000u
#define VI_DI_STATUS    0x80000000u

/* Real timings. NTSC is 525 lines at 59.94 Hz interlaced, so a field is 262.5
 * lines; PAL is 625 at 50 Hz. Line *rate* is the quantity that stays fixed, so
 * everything is derived from it. */
#define NTSC_LINES_PER_FIELD  263
#define NTSC_FIELDS_PER_SEC   5994      /* hundredths, i.e. 59.94 */
#define PAL_LINES_PER_FIELD   313
#define PAL_FIELDS_PER_SEC    5000

typedef struct {
    u16 regs[0x100 / 2];        /* raw halfword register file  */
    u32 di[VI_NUM_DI];

    VIStandard standard;
    u32 lines_per_field;
    s64 cycles_per_line;
    s64 cycles_per_field;

    u64 field_count;
    u32 current_line;           /* 1-based, as the hardware counts */
    int odd_field;
} VIState;

static VIState s_vi;
static int     s_ev_line = -1;
static u64     s_field_start;   /* absolute cycle at which this field began */

/* ------------------------------------------------------------------ */

static void vi_raise_if_due(void)
{
    int i;
    int any = 0;

    for (i = 0; i < VI_NUM_DI; i++) {
        u32 reg  = s_vi.di[i];
        u32 line = (reg >> 16) & 0x7FFu;

        if (!(reg & VI_DI_ENABLE) || line != s_vi.current_line)
            continue;

        s_vi.di[i] |= VI_DI_STATUS;
        any = 1;
    }
    if (any)
        pi_set_interrupt(PI_INT_VI, 1);
        g_vi_irqs++;
}

/* One event per scanline. That is more granular than most titles need, but it
 * is what makes the programmable interrupt positions actually work -- a title
 * that asks for an interrupt at line 200 to start a mid-frame effect gets it at
 * line 200, not at the next field boundary. At ~4600 cycles per line the
 * scheduling overhead is negligible. */
static void vi_on_line(u64 userdata, s64 cycles_late)
{
    u64 target;
    s64 delta;

    (void)userdata;
    (void)cycles_late;

    s_vi.current_line++;
    if (s_vi.current_line > s_vi.lines_per_field) {
        s_vi.current_line = 1;
        s_vi.odd_field = !s_vi.odd_field;
        s_vi.field_count++;
        s_field_start += (u64)s_vi.cycles_per_field;
    }

    vi_raise_if_due();

    /* Each line's deadline is computed from the *field* start rather than by
     * adding a per-line increment. A truncated per-line value would be short by
     * a fraction of a cycle every line, which compounds into a field rate that
     * is measurably fast -- exactly the drift that makes an emulator report the
     * right frame rate while slowly desynchronising from real time. Deriving
     * from the field start keeps every field exactly cycles_per_field long. */
    target = s_field_start +
             ((u64)s_vi.current_line * (u64)s_vi.cycles_per_field) /
             s_vi.lines_per_field;

    delta = (s64)(target - timing_now());
    if (delta < 1)
        delta = 1;
    timing_schedule(s_ev_line, delta, 0);
}

/* ------------------------------------------------------------------ */
/* MMIO — the VI register file is halfword-oriented                     */
/* ------------------------------------------------------------------ */

static u32 vi_read(u32 addr, unsigned size, void *ctx)
{
    u32 off = addr - HW_VI_BASE;
    (void)ctx;

    if (off >= VI_DI0 && off < VI_DI0 + VI_NUM_DI * 4) {
        u32 i = (off - VI_DI0) / 4;
        u32 v = s_vi.di[i];
        if (size == 2)
            return (off & 2) ? (v & 0xFFFFu) : (v >> 16);
        return v;
    }

    switch (off) {
    case VI_DPV:
        /* Current scanline. Titles busy-wait on this to synchronize with
         * scanout, so it has to advance even when nothing else does. */
        return s_vi.current_line;
    case VI_DPH:
        return 0;
    default:
        break;
    }

    if (size == 4)
        return ((u32)s_vi.regs[off / 2] << 16) | s_vi.regs[off / 2 + 1];
    return s_vi.regs[off / 2];
}

/* Every distinct VI register the title writes, as a bitmap over the 16-bit
 * register file. The XFB stride is programmed through one of these and is
 * currently NOT modelled -- `xfb_width()` hardcodes 640 while this title's EFB
 * copies are 608 wide, and reading a framebuffer at the wrong stride skews
 * every row, which is what a "flickering gradient" looks like. Recording which
 * registers are actually touched turns that from a guess into a short list. */
unsigned long long g_vi_written_lo, g_vi_written_hi;

/* Width in pixels of the framebuffer the video interface scans out.
 *
 * PICTURE_CONFIGURATION (0x48) holds WPL, "words per line", in units of 16
 * pixels, so the XFB width is WPL * 16: a 640-pixel buffer programs 40 and this
 * title's 608-pixel one programs 38. That register was previously not modelled
 * at all and the presenter hardcoded 640 -- reading a 608-wide framebuffer at
 * 640 walks every row 32 pixels further than it should, which shears the
 * picture progressively down the screen and reads as a flickering diagonal
 * gradient rather than as a clean offset.
 *
 * Returns 0 when the title has not programmed it yet, so the caller can keep
 * its own default rather than scan from a zero-width buffer. */
unsigned vi_xfb_width(void)
{
    unsigned wpl = s_vi.regs[VI_PICCONF / 2] & 0x7Fu;
    unsigned w = wpl * 16u;
    return (w >= 320u && w <= 1024u) ? w : 0u;
}

static void vi_write(u32 addr, u32 value, unsigned size, void *ctx)
{
    u32 off = addr - HW_VI_BASE;
    (void)ctx;
    {   unsigned idx = off / 2;
        if (idx < 64)       g_vi_written_lo |= 1ull << idx;
        else if (idx < 128) g_vi_written_hi |= 1ull << (idx - 64);
    }

    if (off >= VI_DI0 && off < VI_DI0 + VI_NUM_DI * 4) {
        u32 i = (off - VI_DI0) / 4;
        u32 old = s_vi.di[i];
        u32 nv;

        if (size == 2) {
            nv = (off & 2) ? ((old & 0xFFFF0000u) | (value & 0xFFFFu))
                           : ((old & 0x0000FFFFu) | (value << 16));
        } else {
            nv = value;
        }

        /* The status bit is acknowledge-by-writing-zero; it is never set by the
         * guest. Letting a write set it would make a title that reconfigures an
         * interrupt appear to have taken one. */
        if (!(nv & VI_DI_STATUS))
            nv &= ~VI_DI_STATUS;
        else
            nv = (nv & ~VI_DI_STATUS) | (old & VI_DI_STATUS);

        s_vi.di[i] = nv;

        /* Deassert the shared line once every source has been acknowledged. */
        {
            int j, still = 0;
            for (j = 0; j < VI_NUM_DI; j++)
                if (s_vi.di[j] & VI_DI_STATUS)
                    still = 1;
            if (!still)
                pi_set_interrupt(PI_INT_VI, 0);
        }
        return;
    }

    if (size == 4) {
        s_vi.regs[off / 2]     = (u16)(value >> 16);
        s_vi.regs[off / 2 + 1] = (u16)value;
    } else {
        s_vi.regs[off / 2] = (u16)value;
    }
}

/* ------------------------------------------------------------------ */

u64 vi_field_count(void) { return s_vi.field_count; }

u32 vi_current_xfb(void)
{
    /* Framebuffer addresses are stored shifted right by 5 with a flag in the
     * top bit; titles set both fields even for progressive output. */
    u32 hi = s_vi.regs[VI_TFBL / 2];
    u32 lo = s_vi.regs[VI_TFBL / 2 + 1];
    u32 v  = ((u32)hi << 16) | lo;
    u32 addr = (v & 0x00FFFFFFu);
    if (v & 0x10000000u)
        addr <<= 5;
    return addr ? (MEM1_CACHED | addr) : 0;
}

void vi_reset(void)
{
    u64 fields_hundredths;

    memset(s_vi.regs, 0, sizeof s_vi.regs);
    memset(s_vi.di, 0, sizeof s_vi.di);
    s_vi.field_count = 0;
    s_vi.current_line = 1;
    s_vi.odd_field = 0;

    if (s_vi.standard == VI_PAL) {
        s_vi.lines_per_field = PAL_LINES_PER_FIELD;
        fields_hundredths = PAL_FIELDS_PER_SEC;
    } else {
        s_vi.lines_per_field = NTSC_LINES_PER_FIELD;
        fields_hundredths = NTSC_FIELDS_PER_SEC;
    }

    /* Derived from the true field rate rather than a rounded frame time: the
     * difference between 59.94 and 60 is 0.1%, which is inaudible for one
     * frame and obvious over a minute of streamed audio. */
    s_vi.cycles_per_field =
        (s64)((u64)g_cpu_hz * 100ull / fields_hundredths);
    s_vi.cycles_per_line  = s_vi.cycles_per_field / s_vi.lines_per_field;

    LOG_INFO(LOG_VIDEO, "VI: %s, %u lines/field, %lld cycles/field (%.2f Hz)",
             s_vi.standard == VI_PAL ? "PAL" : "NTSC",
             s_vi.lines_per_field, (long long)s_vi.cycles_per_field,
             (double)g_cpu_hz / (double)s_vi.cycles_per_field);

    s_field_start = timing_now();
    if (s_ev_line >= 0)
        timing_schedule(s_ev_line, s_vi.cycles_per_line, 0);
}

void vi_init(VIStandard standard)
{
    s_vi.standard = standard;
    /* Always re-register: timing_init clears the event table, so a handle
     * cached from a previous machine would refer to nothing and the interrupt
     * would silently never fire. */
    s_ev_line = timing_register_event("VI line", vi_on_line);
    vi_reset();
    mmio_register(HW_VI_BASE, 0x100, vi_read, vi_write, NULL, "VI");
}
