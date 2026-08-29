unsigned g_gp_writes;
/* gx_fifo.c — write-gather pipe and command-processor FIFO.
 *
 * See gx_fifo.h for what this is and why the ring bookkeeping is the part that
 * matters. This file implements the plumbing; parsing what comes out the other
 * end is the graphics backend's job.
 */
#include "gx_fifo.h"
#include "hardware.h"
#include "../mem/memmap.h"
#include "../../common/log.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* CP register offsets from HW_CP_BASE.                                 */
/*                                                                      */
/* Every one of these is 16 bits, and the 32-bit quantities are split across a  */
/* LO/HI pair at consecutive addresses. That is not a modelling choice -- the   */
/* hardware registers really are 16-bit, and titles write the halves            */
/* separately, so a model that stored 32-bit values and ignored the split would */
/* see a pointer update as two nonsensical intermediate values.                 */
/* ------------------------------------------------------------------ */

#define CP_STATUS           0x00
#define CP_CONTROL          0x02
#define CP_CLEAR            0x04
#define CP_PERF_SELECT      0x06
#define CP_TOKEN            0x0E
#define CP_BBOX_LEFT        0x10
#define CP_BBOX_RIGHT       0x12
#define CP_BBOX_TOP         0x14
#define CP_BBOX_BOTTOM      0x16
#define CP_FIFO_BASE_LO     0x20
#define CP_FIFO_BASE_HI     0x22
#define CP_FIFO_END_LO      0x24
#define CP_FIFO_END_HI      0x26
#define CP_FIFO_HIWATER_LO  0x28
#define CP_FIFO_HIWATER_HI  0x2A
#define CP_FIFO_LOWATER_LO  0x2C
#define CP_FIFO_LOWATER_HI  0x2E
#define CP_FIFO_RWDIST_LO   0x30
#define CP_FIFO_RWDIST_HI   0x32
#define CP_FIFO_WPTR_LO     0x34
#define CP_FIFO_WPTR_HI     0x36
#define CP_FIFO_RPTR_LO     0x38
#define CP_FIFO_RPTR_HI     0x3A
#define CP_FIFO_BP_LO       0x3C
#define CP_FIFO_BP_HI       0x3E

/* STATUS */
#define CP_ST_OVERFLOW      0x0001u   /* distance passed the high watermark */
#define CP_ST_UNDERFLOW     0x0002u   /* distance fell below the low one    */
#define CP_ST_READ_IDLE     0x0004u
#define CP_ST_COMMAND_IDLE  0x0008u
#define CP_ST_BREAKPOINT    0x0010u

/* CONTROL */
#define CP_CT_READ_ENABLE   0x0001u
#define CP_CT_BP_ENABLE     0x0002u
#define CP_CT_OVERFLOW_INT  0x0004u
#define CP_CT_UNDERFLOW_INT 0x0008u
#define CP_CT_GPLINK        0x0010u   /* CP write pointer follows PI's */
#define CP_CT_BP_INT        0x0020u

/* CLEAR (write-one-to-clear against STATUS) */
#define CP_CL_OVERFLOW      0x0001u
#define CP_CL_UNDERFLOW     0x0002u

#define GATHER_PIPE_SIZE    32u

static CPFifo s_cp;

/* ------------------------------------------------------------------ */
/* Interrupt decision                                                   */
/*                                                                      */
/* One function, for the same reason PI aggregates interrupts in one place: the */
/* CP can assert for three independent reasons and the guest masks them         */
/* separately, so deciding "is the CP line asserted" anywhere else guarantees   */
/* the line eventually gets stuck.                                              */
/* ------------------------------------------------------------------ */

static void cp_update_interrupt(void)
{
    int assert_line =
        ((s_cp.status & CP_ST_OVERFLOW)   && (s_cp.control & CP_CT_OVERFLOW_INT))  ||
        ((s_cp.status & CP_ST_UNDERFLOW)  && (s_cp.control & CP_CT_UNDERFLOW_INT)) ||
        ((s_cp.status & CP_ST_BREAKPOINT) && (s_cp.control & CP_CT_BP_INT));

    pi_set_interrupt(PI_INT_CP, assert_line);
}

/* Watermarks are hysteresis, not thresholds: overflow latches when the FIFO
 * fills past the high mark and is cleared by the guest, not by draining. A
 * title's pacing loop depends on that -- it enables the interrupt, sleeps, and
 * expects to be woken once rather than continuously. */
static void cp_check_watermarks(void)
{
    if (s_cp.rw_distance > s_cp.hi_watermark)
        s_cp.status |= CP_ST_OVERFLOW;
    if (s_cp.rw_distance < s_cp.lo_watermark)
        s_cp.status |= CP_ST_UNDERFLOW;
    cp_update_interrupt();
}

static void cp_check_breakpoint(void)
{
    if (!(s_cp.control & CP_CT_BP_ENABLE))
        return;
    /* The breakpoint fires when the *read* pointer reaches it: it exists so a
     * title can be told "the GPU has consumed everything up to here", which is
     * how double-buffered display lists are recycled safely. */
    if (s_cp.read_pointer == s_cp.breakpoint) {
        s_cp.status |= CP_ST_BREAKPOINT;
        cp_update_interrupt();
    }
}

/* ------------------------------------------------------------------ */
/* Ring arithmetic                                                      */
/*                                                                      */
/* `end` names the last valid 32-byte cell, not one past it, which is how the   */
/* hardware register is defined. Every wrap test below is written against that  */
/* meaning; treating `end` as exclusive costs exactly one cell of the ring and  */
/* produces a corruption that only appears once a display list happens to land  */
/* on the boundary.                                                             */
/* ------------------------------------------------------------------ */

static u32 ring_size(void)
{
    return s_cp.end + GATHER_PIPE_SIZE - s_cp.base;
}

static u32 ring_advance(u32 ptr, u32 bytes)
{
    u32 size = ring_size();
    if (size == 0)
        return ptr;
    /* Modulo rather than a single compare-and-subtract: a consumer may advance
     * by more than one cell at a time, and a wrap test that assumes 32-byte
     * steps silently produces a pointer outside the ring when it does not. */
    return s_cp.base + ((ptr - s_cp.base + bytes) % size);
}

static int ring_configured(void)
{
    return s_cp.end > s_cp.base;
}

/* ------------------------------------------------------------------ */
/* Write-gather pipe                                                    */
/* ------------------------------------------------------------------ */

static void gather_burst(PPCState *s)
{
    u32 wptr;

    /* Where the burst lands is the processor interface's business: the CPU-side
     * write pointer lives in PI, while the GPU-side read pointer lives in CP.
     * They address the same ring from opposite ends. */
    u32 base, end;
    pi_fifo_window(&base, &end, &wptr);

    if (end <= base) {
        /* A title that has not configured the FIFO yet is still allowed to
         * write to the gather pipe; the bytes simply go nowhere. Dropping them
         * silently would hide a real misconfiguration, so say it once. */
        LOG_WARN_ONCE(LOG_VIDEO,
                      "gather pipe burst with no FIFO window configured");
        s->gather_pipe_count = 0;
        return;
    }

    mem_write_block(wptr, s->gather_pipe, GATHER_PIPE_SIZE);

    wptr += GATHER_PIPE_SIZE;
    if (wptr > end)
        wptr = base;
    pi_fifo_set_write_pointer(wptr);

    s->gather_pipe_count = 0;

    /* With the GP link enabled the command processor's write pointer tracks the
     * CPU's automatically -- which is how titles run in practice, because the
     * alternative is updating a GPU register on every burst. */
    if (s_cp.control & CP_CT_GPLINK) {
        s_cp.write_pointer = wptr;
        __atomic_fetch_add(&s_cp.rw_distance, GATHER_PIPE_SIZE, __ATOMIC_SEQ_CST);
        cp_check_watermarks();
    }
}

void gxfifo_gather_write(PPCState *s, u32 value_hi, u32 value_lo, unsigned size)
{
    g_gp_writes++;
    u8 bytes[8];
    unsigned i;

    /* Big-endian, because that is what the command processor reads and what the
     * guest wrote. The host is also big-endian, but writing the bytes out
     * explicitly keeps this correct if the core is ever built elsewhere -- and
     * the whole verification harness depends on being able to do exactly
     * that. */
    switch (size) {
    case 1:
        bytes[0] = (u8)value_lo;
        break;
    case 2:
        bytes[0] = (u8)(value_lo >> 8); bytes[1] = (u8)value_lo;
        break;
    case 4:
        bytes[0] = (u8)(value_lo >> 24); bytes[1] = (u8)(value_lo >> 16);
        bytes[2] = (u8)(value_lo >> 8);  bytes[3] = (u8)value_lo;
        break;
    case 8:
        bytes[0] = (u8)(value_hi >> 24); bytes[1] = (u8)(value_hi >> 16);
        bytes[2] = (u8)(value_hi >> 8);  bytes[3] = (u8)value_hi;
        bytes[4] = (u8)(value_lo >> 24); bytes[5] = (u8)(value_lo >> 16);
        bytes[6] = (u8)(value_lo >> 8);  bytes[7] = (u8)value_lo;
        break;
    default:
        LOG_WARN_ONCE(LOG_VIDEO, "gather pipe write of %u bytes", size);
        return;
    }

    /* A byte stream, not a sequence of whole stores.
     *
     * The pipe bursts every 32 bytes accumulated, wherever the store boundaries
     * happen to fall -- a four-byte store arriving with 30 bytes pending puts
     * two bytes in, bursts, and starts the next burst with the other two. An
     * earlier version required each store to fit entirely and discarded the
     * pipe when one did not, which silently ate the vertex count and seven of
     * nine vertex words in the first real guest program that hit the case.
     * Unit tests missed it because a test naturally writes patterns that fill
     * the pipe exactly; only guest code compiled by a compiler produces the
     * awkward alignment. */
    for (i = 0; i < size; i++) {
        s->gather_pipe[s->gather_pipe_count++] = bytes[i];
        if (s->gather_pipe_count >= GATHER_PIPE_SIZE)
            gather_burst(s);
    }
}

void gxfifo_gather_flush(PPCState *s)
{
    if (s->gather_pipe_count == 0)
        return;
    /* Pad with zeros: 0x00 is the command processor's NOP, so a partial burst
     * is harmless rather than a truncated command. */
    memset(s->gather_pipe + s->gather_pipe_count, 0,
           GATHER_PIPE_SIZE - s->gather_pipe_count);
    s->gather_pipe_count = GATHER_PIPE_SIZE;
    gather_burst(s);
}

/* ------------------------------------------------------------------ */
/* Read side                                                            */
/* ------------------------------------------------------------------ */

u32 gxfifo_readable(void)
{
    if (!(s_cp.control & CP_CT_READ_ENABLE) || !ring_configured())
        return 0;
    return s_cp.rw_distance;
}

void gxfifo_consume(u32 bytes)
{
    u32 old_read;
    if (!ring_configured() || bytes == 0)
        return;
    if (bytes > s_cp.rw_distance)
        bytes = s_cp.rw_distance;

    old_read = s_cp.read_pointer;
    s_cp.read_pointer = ring_advance(s_cp.read_pointer, bytes);
    __atomic_fetch_sub(&s_cp.rw_distance, bytes, __ATOMIC_SEQ_CST);

    /* Breakpoint CROSSING, not just landing: commands are consumed in
     * multi-byte chunks, so the read pointer routinely steps OVER the
     * breakpoint without ever equalling it. The exact-equality check below
     * then never fired, the game never got its breakpoint interrupt, and its
     * FIFO feeder thread waited forever -- in-race (dual-buffered fifo with a
     * breakpoint at the safe point) the whole 3D scene simply stopped being
     * submitted while the HUD (fed before the wait) still drew. */
    if ((s_cp.control & CP_CT_BP_ENABLE) && bytes) {
        u32 rel = ring_advance(s_cp.breakpoint, 0);   /* normalised */
        u32 span_start = old_read, span_bytes = bytes;
        /* Distance from span start to the breakpoint, along the ring. */
        u32 d = (rel >= span_start)
              ? rel - span_start
              : (ring_size() - (span_start - rel));
        if (d < span_bytes) {
            s_cp.status |= CP_ST_BREAKPOINT;
            cp_update_interrupt();
        }
    }

    if (s_cp.rw_distance == 0)
        s_cp.status |= CP_ST_READ_IDLE | CP_ST_COMMAND_IDLE;
    else
        s_cp.status &= (u16)~(CP_ST_READ_IDLE | CP_ST_COMMAND_IDLE);

    cp_check_watermarks();
    cp_check_breakpoint();
}

u32 gxfifo_peek_address(u32 offset)
{
    return ring_advance(s_cp.read_pointer, offset);
}

u8 gxfifo_peek8(u32 offset)
{
    return mem_read8(gxfifo_peek_address(offset));
}

/* Bytes readable contiguously in guest memory from `offset` before the ring
 * wraps back to its base. A consumer that wants a direct window instead of a
 * translated read per byte must clamp to this: past the wrap, consecutive
 * stream positions are not consecutive addresses. */
u32 gxfifo_peek_contig(u32 offset)
{
    u32 a;
    if (!ring_configured())
        return 0;
    a = gxfifo_peek_address(offset);
    return (s_cp.end + GATHER_PIPE_SIZE) - a;
}

/* Assembled byte by byte rather than with a wide load: a multi-byte field can
 * straddle the wrap, where the bytes are not contiguous in guest memory. A
 * mem_read16 at the last byte of the ring would read one byte of command and
 * one byte of whatever follows the FIFO. */
u16 gxfifo_peek16(u32 offset)
{
    return (u16)(((u16)gxfifo_peek8(offset) << 8) | gxfifo_peek8(offset + 1));
}

u32 gxfifo_peek32(u32 offset)
{
    return ((u32)gxfifo_peek8(offset)     << 24) |
           ((u32)gxfifo_peek8(offset + 1) << 16) |
           ((u32)gxfifo_peek8(offset + 2) <<  8) |
            (u32)gxfifo_peek8(offset + 3);
}

const CPFifo *gxfifo_state(void) { return &s_cp; }

/* ------------------------------------------------------------------ */
/* MMIO                                                                 */
/* ------------------------------------------------------------------ */

/* 32-bit halves are assembled from the LO/HI pair. Reads return the half the
 * guest asked for; a 32-bit read of a LO offset returns both, which is what
 * libogc's register macros do. */
static u32 pair_read(u32 v, u32 off_is_hi, unsigned size)
{
    u32 half = off_is_hi ? (v >> 16) : (v & 0xFFFFu);
    return (size == 4) ? v : half;
}

static void pair_write(u32 *v, u32 off_is_hi, u32 value, unsigned size)
{
    if (size == 4) { *v = value; return; }
    if (off_is_hi) *v = (*v & 0x0000FFFFu) | ((value & 0xFFFFu) << 16);
    else           *v = (*v & 0xFFFF0000u) | (value & 0xFFFFu);
}

static u32 cp_read(u32 addr, unsigned size, void *ctx)
{
    u32 off = addr - HW_CP_BASE;
    (void)ctx;

    switch (off) {
    case CP_STATUS:      return s_cp.status;
    case CP_CONTROL:     return s_cp.control;
    case CP_CLEAR:       return s_cp.clear;
    case CP_TOKEN:       return s_cp.token;
    case CP_BBOX_LEFT:   return s_cp.bbox_left;
    case CP_BBOX_RIGHT:  return s_cp.bbox_right;
    case CP_BBOX_TOP:    return s_cp.bbox_top;
    case CP_BBOX_BOTTOM: return s_cp.bbox_bottom;

    case CP_FIFO_BASE_LO:    case CP_FIFO_BASE_HI:
        return pair_read(s_cp.base, off == CP_FIFO_BASE_HI, size);
    case CP_FIFO_END_LO:     case CP_FIFO_END_HI:
        return pair_read(s_cp.end, off == CP_FIFO_END_HI, size);
    case CP_FIFO_HIWATER_LO: case CP_FIFO_HIWATER_HI:
        return pair_read(s_cp.hi_watermark, off == CP_FIFO_HIWATER_HI, size);
    case CP_FIFO_LOWATER_LO: case CP_FIFO_LOWATER_HI:
        return pair_read(s_cp.lo_watermark, off == CP_FIFO_LOWATER_HI, size);
    case CP_FIFO_RWDIST_LO:  case CP_FIFO_RWDIST_HI:
        return pair_read(s_cp.rw_distance, off == CP_FIFO_RWDIST_HI, size);
    case CP_FIFO_WPTR_LO:    case CP_FIFO_WPTR_HI:
        return pair_read(s_cp.write_pointer, off == CP_FIFO_WPTR_HI, size);
    case CP_FIFO_RPTR_LO:    case CP_FIFO_RPTR_HI:
        return pair_read(s_cp.read_pointer, off == CP_FIFO_RPTR_HI, size);
    case CP_FIFO_BP_LO:      case CP_FIFO_BP_HI:
        return pair_read(s_cp.breakpoint, off == CP_FIFO_BP_HI, size);

    case CP_PERF_SELECT: return 0;
    default:
        LOG_WARN_ONCE(LOG_VIDEO, "CP: read from unmapped +%03x", off);
        return 0;
    }
}

static void cp_write(u32 addr, u32 value, unsigned size, void *ctx)
{
    u32 off = addr - HW_CP_BASE;
    (void)ctx;

    switch (off) {
    case CP_CONTROL:
        s_cp.control = (u16)value;
        /* Enabling the breakpoint can expose one the read pointer is already
         * sitting on, the same way unmasking an interrupt can. */
        cp_check_breakpoint();
        cp_update_interrupt();
        return;

    case CP_CLEAR:
        /* Write-one-to-clear against STATUS. This is the only way the latched
         * watermark bits go away -- draining the FIFO does not clear them. */
        if (value & CP_CL_OVERFLOW)  s_cp.status &= (u16)~CP_ST_OVERFLOW;
        if (value & CP_CL_UNDERFLOW) s_cp.status &= (u16)~CP_ST_UNDERFLOW;
        s_cp.clear = (u16)value;
        cp_update_interrupt();
        return;

    case CP_STATUS:
        /* The breakpoint bit is acknowledged through STATUS rather than CLEAR. */
        s_cp.status &= (u16)~(value & CP_ST_BREAKPOINT);
        cp_update_interrupt();
        return;

    case CP_TOKEN:       s_cp.token       = (u16)value; return;
    case CP_BBOX_LEFT:   s_cp.bbox_left   = (u16)value; return;
    case CP_BBOX_RIGHT:  s_cp.bbox_right  = (u16)value; return;
    case CP_BBOX_TOP:    s_cp.bbox_top    = (u16)value; return;
    case CP_BBOX_BOTTOM: s_cp.bbox_bottom = (u16)value; return;

    case CP_FIFO_BASE_LO: case CP_FIFO_BASE_HI:
        pair_write(&s_cp.base, off == CP_FIFO_BASE_HI, value, size);
        s_cp.base &= ~31u;
        return;
    case CP_FIFO_END_LO: case CP_FIFO_END_HI:
        pair_write(&s_cp.end, off == CP_FIFO_END_HI, value, size);
        s_cp.end &= ~31u;
        return;
    case CP_FIFO_HIWATER_LO: case CP_FIFO_HIWATER_HI:
        pair_write(&s_cp.hi_watermark, off == CP_FIFO_HIWATER_HI, value, size);
        return;
    case CP_FIFO_LOWATER_LO: case CP_FIFO_LOWATER_HI:
        pair_write(&s_cp.lo_watermark, off == CP_FIFO_LOWATER_HI, value, size);
        return;
    case CP_FIFO_RWDIST_LO: case CP_FIFO_RWDIST_HI:
        pair_write(&s_cp.rw_distance, off == CP_FIFO_RWDIST_HI, value, size);
        cp_check_watermarks();
        return;
    case CP_FIFO_WPTR_LO: case CP_FIFO_WPTR_HI:
        pair_write(&s_cp.write_pointer, off == CP_FIFO_WPTR_HI, value, size);
        return;
    case CP_FIFO_RPTR_LO: case CP_FIFO_RPTR_HI:
        pair_write(&s_cp.read_pointer, off == CP_FIFO_RPTR_HI, value, size);
        cp_check_breakpoint();
        return;
    case CP_FIFO_BP_LO: case CP_FIFO_BP_HI:
        pair_write(&s_cp.breakpoint, off == CP_FIFO_BP_HI, value, size);
        cp_check_breakpoint();
        return;

    case CP_PERF_SELECT: return;
    default:
        LOG_WARN_ONCE(LOG_VIDEO, "CP: write to unmapped +%03x = %08x", off, value);
        return;
    }
}

/* The gather pipe occupies a whole 4 KiB page and every address in it behaves
 * identically -- titles deliberately write to varying offsets so consecutive
 * stores do not contend for one store-buffer entry. */
static u32 gp_read(u32 addr, unsigned size, void *ctx)
{
    (void)addr; (void)size; (void)ctx;
    /* The pipe is write-only; reads return zero on hardware. */
    return 0;
}

/* ------------------------------------------------------------------ */

/* The pipe is part of the CPU (it is Gekko's store buffer), so its staging
 * bytes live in PPCState rather than here. */
static PPCState *s_cpu;

static void gp_write(u32 addr, u32 value, unsigned size, void *ctx)
{
    (void)addr; (void)ctx;
    if (!s_cpu) {
        LOG_WARN_ONCE(LOG_VIDEO, "gather pipe write before the CPU was attached");
        return;
    }
    gxfifo_gather_write(s_cpu, 0, value, size);
}

void gxfifo_init(void)
{
    gxfifo_reset();
    mmio_register(HW_CP_BASE, 0x100, cp_read, cp_write, NULL, "CP");
    mmio_register(HW_GPFIFO_BASE, 0x1000, gp_read, gp_write, NULL, "GP");
}

void gxfifo_reset(void)
{
    memset(&s_cp, 0, sizeof s_cp);
    /* Idle out of reset: nothing written, nothing to read. */
    s_cp.status = CP_ST_READ_IDLE | CP_ST_COMMAND_IDLE;
}

void gxfifo_attach_cpu(PPCState *s)
{
    s_cpu = s;
}
