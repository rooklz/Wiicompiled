/* gx_fifo.h — the write-gather pipe and the command processor's FIFO.
 *
 * This is the road every triangle travels. The CPU does not talk to the GPU
 * through registers; it *streams* to it, and the stream has three stages:
 *
 *   1. The write-gather pipe. Gekko has a 32-byte store buffer behind address
 *      0xCC008000. Stores of any width accumulate there; when 32 bytes are
 *      present the buffer bursts to memory in one transaction. This exists
 *      because the alternative -- a store per command byte -- would spend the
 *      entire memory bus on 1-byte writes.
 *
 *   2. The FIFO in main memory. The burst lands at the processor interface's
 *      write pointer, which advances and wraps inside a window the guest
 *      configures. It is an ordinary ring buffer in MEM1.
 *
 *   3. The command processor reads from the other end of that ring and
 *      executes what it finds.
 *
 * Getting the ring's bookkeeping right matters more than it looks. A title
 * builds its display list *while* the GPU is consuming it, and paces itself
 * against the read/write distance and two watermarks. If the distance is wrong,
 * a title either stalls forever waiting for space that already exists, or
 * overwrites commands the GPU has not read yet -- and the second failure shows
 * up as corrupted geometry many frames later.
 */
#ifndef DOLPHIN_CORE_HW_GX_FIFO_H
#define DOLPHIN_CORE_HW_GX_FIFO_H

#include "../ppc/gekko.h"

/* Attach before init: the gather pipe stages into PPCState, so the MMIO handler
 * needs the CPU before it can accept a write. */
void gxfifo_attach_cpu(PPCState *s);
void gxfifo_init(void);
void gxfifo_reset(void);

/* ------------------------------------------------------------------ */
/* Write-gather pipe                                                    */
/* ------------------------------------------------------------------ */

/* A guest store to the 0xCC008000 window. `size` is 1, 2, 4 or 8 bytes; the
 * value is big-endian in the low bits, as the guest wrote it. Bursts to memory
 * automatically whenever 32 bytes have accumulated. */
void gxfifo_gather_write(PPCState *s, u32 value_hi, u32 value_lo, unsigned size);

/* Flush a partial pipe. The guest does this explicitly (GX's "flush" writes 32
 * bytes of padding), but a state save or a reset needs it too. */
void gxfifo_gather_flush(PPCState *s);

/* ------------------------------------------------------------------ */
/* Command processor FIFO state                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    u32 base;           /* first byte of the ring          */
    u32 end;            /* last 32-byte cell of the ring   */
    u32 hi_watermark;   /* distance at which "too full"    */
    u32 lo_watermark;   /* distance at which "drained"     */
    u32 rw_distance;    /* bytes written but not yet read  */
    u32 write_pointer;
    u32 read_pointer;
    u32 breakpoint;

    u16 status;
    u16 control;
    u16 clear;
    u16 token;

    u16 bbox_left, bbox_right, bbox_top, bbox_bottom;
} CPFifo;

/* Read-only view, for tests and for the graphics backend's consumer. */
const CPFifo *gxfifo_state(void);

/* How many bytes the command processor may consume right now. Zero when the
 * FIFO is empty or the guest has not enabled reads. */
u32  gxfifo_readable(void);

/* Consume `bytes` from the read side, advancing and wrapping the read pointer.
 * The graphics backend calls this as it parses commands. */
void gxfifo_consume(u32 bytes);

/* Read a byte `offset` bytes ahead of the read pointer, wrapping.
 *
 * The parser has to look before it commits: a draw command's length depends on
 * a vertex count that is itself in the stream, and a command split across two
 * bursts must be left alone until the rest arrives. Peeking keeps the ring
 * arithmetic in one file instead of duplicating the wrap in the parser -- which
 * is exactly the duplication that produces two subtly different wraps. */
u8  gxfifo_peek8(u32 offset);
/* Contiguous bytes in guest memory from `offset` before the ring wraps. */
u32 gxfifo_peek_contig(u32 offset);
u16 gxfifo_peek16(u32 offset);
u32 gxfifo_peek32(u32 offset);

/* Guest address of the byte `offset` ahead of the read pointer. The parser
 * hands this to the sink for vertex data, so a backend can read the vertices
 * straight out of guest memory rather than through a copy. */
u32 gxfifo_peek_address(u32 offset);

#endif /* DOLPHIN_CORE_HW_GX_FIFO_H */
