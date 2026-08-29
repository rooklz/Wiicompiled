/* gx_parse.c — the command processor's opcode loop.
 *
 * Reads the byte stream the FIFO delivers and turns it into calls on a sink.
 * It deliberately knows nothing about rendering: the same loop drives the RSX
 * backend, a FIFO analyser, and a test that only counts.
 *
 * Two properties matter more than anything else here.
 *
 * *It never guesses a length.* Every command's size is either fixed or
 * computable from bytes already in the stream, and the loop refuses to start a
 * command it cannot finish. A parser that consumed a partial draw would
 * resynchronise on vertex data and interpret it as opcodes -- and vertex data
 * contains every byte value, so it would keep "succeeding" indefinitely while
 * producing nonsense.
 *
 * *It stops cleanly when the stream runs out.* A title writes a draw's header
 * and its vertices in separate 32-byte bursts, so an incomplete command is the
 * normal case, not an error. The GPU waits; so does this.
 */
#include "gx.h"
#include "../hw/gx_fifo.h"
#include "../mem/memmap.h"
#include "../../common/log.h"
#ifdef __PS3__
#include "../../common/phase_prof.h"
/* Only the console build links the phase profiler (g_prof lives wherever
 * PHASE_PROF_IMPL is set). The host and qemu harnesses compile this file too,
 * so the timing calls have to compile away for them. */
#define GXSTATE_ENTER()  prof_enter(PH_GXSTATE)
#define GXSTATE_EXIT()   prof_exit()
#define DRAWDISP_ENTER() prof_enter(PH_DRAWDISP)
#define DRAWDISP_EXIT()  prof_exit()
#else
#define GXSTATE_ENTER()  ((void)0)
#define GXSTATE_EXIT()   ((void)0)
#define DRAWDISP_ENTER() ((void)0)
#define DRAWDISP_EXIT()  ((void)0)
#endif

#include <string.h>
#include "../../common/ppe_prefetch.h"

/* Defined here rather than in the PS3 platform layer: the FIFO parser is the
 * only user, and the host and big-endian test harnesses link this file but not
 * main.c. */
int g_ppe_prefetch_off;

/* Nonzero while the parser is inside a called display list, and the draw
 * attribution that depends on it. Research on Dolphin's implementation flags
 * display lists as "usually immutable", which would make a decoded list
 * replayable instead of re-walked every frame; the value of that is the share
 * of draws issued from inside one. */
unsigned g_gx_dl_depth;
unsigned long long g_gx_draws_in_dl, g_gx_draws_top;
unsigned long long g_gx_dl_stable, g_gx_dl_mutated;
unsigned long long g_gx_bp_in_dl, g_gx_bp_top, g_gx_cmode0_in_dl, g_gx_cmode0_top;

/* ------------------------------------------------------------------ */
/* Stream access                                                        */
/*                                                                      */
/* The parser runs over two different sources -- the FIFO ring, and a flat span */
/* of guest memory for a called display list -- so reads go through a small     */
/* indirection rather than being written twice. Writing them twice is how the   */
/* ring version and the memory version end up disagreeing about a wrap.         */
/* ------------------------------------------------------------------ */

typedef struct {
    int from_fifo;
    u32 base;       /* memory source only */
    u32 avail;      /* bytes reachable from the current position   */
    u32 pos;        /* bytes consumed so far                       */

    /* Direct host window over stream positions [win_lo, win_lo + win_len).
     *
     * Without it every byte of the command stream cost a branch plus a full
     * guest translation -- and a 32-bit field cost four of them, since both
     * the ring and the memory source assembled wide reads a byte at a time.
     * In-race that made FIFO parsing 21.6% of the frame for roughly 1.5 MB of
     * data, about 32 ns per byte, two orders of magnitude off what the memory
     * system can do. Positions are absolute, so the window survives `pos`
     * advancing and is rebuilt only when a read leaves it. `win_len == 0`
     * means no window is held and every accessor falls back to the slow path,
     * which stays the sole authority on correctness. */
    const u8 *win;
    u32       win_lo;
    u32       win_len;
} Stream;

static u32 st_available(const Stream *s) { return s->avail - s->pos; }

static u8 st_slow8(const Stream *s, u32 p)
{
    return s->from_fifo ? gxfifo_peek8(p) : mem_read8(s->base + p);
}

/* Point the window at stream position `p`, or leave it empty if that position
 * has no contiguous host backing. Three separate limits apply and the window
 * is the smallest: what the memory region guarantees, what the stream still
 * has, and -- for the ring -- how far it is to the wrap, past which stream
 * positions are no longer contiguous in guest memory. */
static void st_window(Stream *s, u32 p)
{
    u32 span, room;
    u32 ga;
    void *hp;

    s->win = NULL;
    s->win_lo = 0;
    s->win_len = 0;

    if (p >= s->avail)
        return;

    ga = s->from_fifo ? gxfifo_peek_address(p) : s->base + p;
    hp = mem_ptr(ga);
    if (!hp)
        return;

    span = mem_valid_span(ga);
    room = s->avail - p;
    if (span > room)
        span = room;
    if (s->from_fifo) {
        u32 contig = gxfifo_peek_contig(p);
        if (span > contig)
            span = contig;
    }
    if (span == 0)
        return;

    s->win     = (const u8 *)hp;
    s->win_lo  = p;
    s->win_len = span;

    /* The parser is about to walk this window from end to end, and the walk is
     * strictly forward -- exactly the access pattern the PPE's prefetch engine
     * exists for. Nothing here can be wrong: the span was just validated, and
     * `dcbt` is a hint. */
    ppe_prefetch_span(s->win, span);
}

/* Bytes of window available at stream position `p`, rebuilding it once if `p`
 * has moved outside. The subtraction is unsigned on purpose: a position before
 * the window wraps to a huge value and fails the bound, same as one past it. */
static u32 st_win_at(Stream *s, u32 p)
{
    u32 d = p - s->win_lo;
    if (d < s->win_len)
        return s->win_len - d;
    st_window(s, p);
    d = p - s->win_lo;
    return (d < s->win_len) ? s->win_len - d : 0;
}

/* Both guest and console are big-endian, so on the PS3 a wide field is one
 * load. The byte-assembling form is kept for little-endian workstation builds,
 * where it is still far cheaper than four translated reads. */
static u32 rd_be32(const u8 *q)
{
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    u32 v;
    memcpy(&v, q, 4);
    return v;
#else
    return ((u32)q[0] << 24) | ((u32)q[1] << 16) | ((u32)q[2] << 8) | q[3];
#endif
}

static u8 st_u8(Stream *s, u32 off)
{
    u32 p = s->pos + off;
    if (st_win_at(s, p) >= 1)
        return s->win[p - s->win_lo];
    return st_slow8(s, p);
}

static u16 st_u16(Stream *s, u32 off)
{
    u32 p = s->pos + off;
    if (st_win_at(s, p) >= 2) {
        const u8 *q = s->win + (p - s->win_lo);
        return (u16)(((u16)q[0] << 8) | q[1]);
    }
    return (u16)(((u16)st_slow8(s, p) << 8) | st_slow8(s, p + 1));
}

static u32 st_u32(Stream *s, u32 off)
{
    u32 p = s->pos + off;
    if (st_win_at(s, p) >= 4)
        return rd_be32(s->win + (p - s->win_lo));
    return ((u32)st_slow8(s, p)     << 24) | ((u32)st_slow8(s, p + 1) << 16) |
           ((u32)st_slow8(s, p + 2) <<  8) |  (u32)st_slow8(s, p + 3);
}

/* Guest address of a position in the stream, so the sink can read vertex data
 * in place. Only meaningful for the memory source and for FIFO data that does
 * not straddle the wrap; callers that need the general case copy instead. */
static u32 st_address(const Stream *s, u32 off)
{
    return s->from_fifo ? gxfifo_peek_address(s->pos + off)
                        : s->base + s->pos + off;
}

/* ------------------------------------------------------------------ */
/* CP register writes                                                   */
/*                                                                      */
/* The parser keeps its own copy of the vertex descriptor and attribute tables  */
/* because it needs them to compute lengths. Everything else is forwarded and   */
/* forgotten.                                                                   */
/* ------------------------------------------------------------------ */

static void apply_cp_reg(GXParser *p, u8 reg, u32 value)
{
    unsigned n = reg & 0x0Fu;

    switch (reg & 0xF0u) {
    case CP_REG_MATINDEX_A:  p->cp.matindex_a = value; break;
    case CP_REG_MATINDEX_B:  p->cp.matindex_b = value; break;
    case CP_REG_VCD_LO:      p->cp.vcd_lo = value;     break;
    case CP_REG_VCD_HI:      p->cp.vcd_hi = value;     break;
    case CP_REG_VAT_A:       if (n < GX_NUM_VAT) p->cp.vat_a[n] = value; break;
    case CP_REG_VAT_B:       if (n < GX_NUM_VAT) p->cp.vat_b[n] = value; break;
    case CP_REG_VAT_C:       if (n < GX_NUM_VAT) p->cp.vat_c[n] = value; break;
    case CP_REG_ARRAY_BASE:  p->cp.array_base[n]   = value; break;
    case CP_REG_ARRAY_STRIDE:p->cp.array_stride[n] = value; break;
    default:
        LOG_WARN_ONCE(LOG_VIDEO, "CP register %02x is not modelled", reg);
        break;
    }

    if (p->sink.load_cp) {
        GXSTATE_ENTER();
        p->sink.load_cp(p->sink.ctx, reg, value);
        GXSTATE_EXIT();
    }
}

/* ------------------------------------------------------------------ */
/* One command                                                          */
/*                                                                      */
/* Returns the number of bytes the command occupies, or 0 when the stream does  */
/* not yet hold all of it. Zero is "come back later", never "skip this".        */
/* ------------------------------------------------------------------ */

static u32 parse_one(GXParser *p, Stream *st)
{
    u32 have = st_available(st);
    u8  op;

    if (have < 1)
        return 0;

    op = st_u8(st, 0);

    /* Primitives first: they are the overwhelming majority of the stream, and
     * the range test is cheaper than walking the fixed-opcode cases. */
    if ((op & GX_PRIMITIVE_MASK) >= GX_PRIMITIVE_BASE &&
        (op & GX_PRIMITIVE_MASK) <= GX_POINTS) {
        unsigned vat = op & GX_VAT_MASK;
        u32 vsize, total;
        u16 count;

        if (have < 3)
            return 0;                       /* opcode + 16-bit vertex count */

        count = st_u16(st, 1);
        vsize = gx_vertex_size(&p->cp, vat);

        if (vsize == 0 && count != 0) {
            /* A draw whose vertices have no size cannot be walked past, so the
             * stream is unrecoverable from here. Say so once and consume the
             * header only -- continuing is wrong, but so is spinning. */
            LOG_WARN_ONCE(LOG_VIDEO,
                          "draw with a zero-size vertex format (VAT %u, "
                          "vcd %08x/%08x)", vat, p->cp.vcd_lo, p->cp.vcd_hi);
            return 3;
        }

        total = 3u + (u32)count * vsize;
        if (have < total)
            return 0;                       /* vertices still arriving */

        if (p->sink.draw) {
            DRAWDISP_ENTER();
            p->sink.draw(p->sink.ctx, (GXPrimitive)(op & GX_PRIMITIVE_MASK),
                         vat, count, st_address(st, 3), vsize);
            DRAWDISP_EXIT();
        }

        p->vertices += count;
        p->draws++;
        if (g_gx_dl_depth) g_gx_draws_in_dl++; else g_gx_draws_top++;
        return total;
    }

    switch (op) {
    case GX_NOP:
        return 1;

    case GX_CMD_INVL_VC:
        /* Nothing is cached yet, so there is nothing to invalidate. */
        return 1;

    case GX_LOAD_CP_REG:
        if (have < 6) return 0;             /* opcode + reg + 32-bit value */
        p->n_cp++;
        apply_cp_reg(p, st_u8(st, 1), st_u32(st, 2));
        return 6;

    case GX_LOAD_XF_REG: {
        /* The header packs a count and a destination: bits 31..16 hold
         * (count - 1), bits 15..0 the XF address. Sixteen registers is the
         * maximum a single command can carry. */
        u32 header, count, addr, total;
        u32 values[16];
        unsigned i;

        if (have < 5) return 0;
        header = st_u32(st, 1);
        count  = ((header >> 16) & 0xFu) + 1u;
        addr   = header & 0xFFFFu;
        total  = 5u + count * 4u;
        if (have < total) return 0;

        for (i = 0; i < count; i++)
            values[i] = st_u32(st, 5 + i * 4);

        p->n_xf++;
        p->n_xf_words += count;
        if (p->sink.load_xf) {
            GXSTATE_ENTER();
            p->sink.load_xf(p->sink.ctx, (u16)addr, values, count);
            GXSTATE_EXIT();
        }
        return total;
    }

    case GX_LOAD_INDX_A:
    case GX_LOAD_INDX_B:
    case GX_LOAD_INDX_C:
    case GX_LOAD_INDX_D: {
        /* An index into one of the arrays, plus where the fetched data lands in
         * XF memory. This is how a title uploads a matrix without putting it in
         * the command stream. */
        u32 word, index, xf_addr, count;
        if (have < 5) return 0;
        word    = st_u32(st, 1);
        index   = (word >> 16) & 0xFFFFu;
        count   = ((word >> 12) & 0xFu) + 1u;
        xf_addr = word & 0xFFFu;
        if (p->sink.load_index)
            p->sink.load_index(p->sink.ctx, op, (u16)index, (u16)xf_addr,
                               (u8)count);
        return 5;
    }

    case GX_LOAD_BP_REG: {
        /* One 32-bit word carrying its own register number in the top byte --
         * the pixel engine's entire configuration arrives this way. */
        u32 word;
        if (have < 5) return 0;
        word = st_u32(st, 1);
        p->n_bp++;
        /* Where do BP register writes come from -- the top-level FIFO, or
         * inside a called display list? 85% of draws are issued from inside a
         * list, and the 3D world is exactly the population whose CMODE0 is
         * stale. If state writes inside lists are being lost, that is the
         * root cause the write-mask workaround has been papering over. */
        if (g_gx_dl_depth) g_gx_bp_in_dl++; else g_gx_bp_top++;
        if ((u8)(word >> 24) == 0x41) {
            if (g_gx_dl_depth) g_gx_cmode0_in_dl++; else g_gx_cmode0_top++;
        }
        if (p->sink.load_bp) {
            GXSTATE_ENTER();
            p->sink.load_bp(p->sink.ctx, (u8)(word >> 24), word & 0x00FFFFFFu);
            GXSTATE_EXIT();
        }
        return 5;
    }

    case GX_CMD_CALL_DL: {
        /* A display list is a second command stream in memory. It is *called*,
         * not inlined, and may not call further -- the hardware has one level,
         * so a nested call is a stream error rather than recursion. */
        u32 addr, size;
        if (have < 9) return 0;
        addr = st_u32(st, 1);
        size = st_u32(st, 5);
        if (p->sink.call_list)
            p->sink.call_list(p->sink.ctx, addr, size);
        else
            p->n_dlist++;
            /* Attribute draws to display lists vs the top-level FIFO.
             *
             * Research on Dolphin's implementation flags display lists as
             * "usually immutable", which would make their parse cacheable --
             * a decoded list could be replayed instead of re-walked every
             * frame. The value of that is the share of draws that come from
             * inside one, which nobody had measured. */
            /* Is a display list at a given address actually immutable?
             *
             * Caching a decoded list is only sound if re-calling the same
             * address yields the same bytes. Hash a bounded prefix (enough to
             * catch edits, cheap enough to run on every call) and count how
             * often an address is seen again with DIFFERENT content. If that
             * count is ~0, the decoded output can be cached and 85% of draws
             * stop being re-walked every frame. */
            {   static struct { u32 addr, size, hash; } seen[512];
                static unsigned nseen;
                u32 h = 2166136261u, i2, lim = size < 4096u ? size : 4096u;
                unsigned k2, found = 0;
                for (i2 = 0; i2 < lim; i2 += 4)
                    h = (h ^ mem_read32(addr + i2)) * 16777619u;
                for (k2 = 0; k2 < nseen; k2++)
                    if (seen[k2].addr == addr && seen[k2].size == size) {
                        found = 1;
                        if (seen[k2].hash != h) { g_gx_dl_mutated++; seen[k2].hash = h; }
                        else                      g_gx_dl_stable++;
                        break;
                    }
                if (!found && nseen < 512) {
                    seen[nseen].addr = addr; seen[nseen].size = size;
                    seen[nseen].hash = h; nseen++;
                }
            }
            g_gx_dl_depth++;
            gx_parser_run_memory(p, addr, size);
            g_gx_dl_depth--;
        return 9;
    }

    case 0x48:
        /* GX_CMD_INVL_VTX: invalidate the vertex cache. One byte, no payload.
         * The RSX backend has no equivalent cache to flush, so consuming it is
         * the whole implementation -- but consuming it EXPLICITLY, because the
         * unknown-opcode fallback also happens to skip one byte and that
         * coincidence should not be load-bearing. */
        return 1;

    default:
        /* Every valid opcode is accounted for above, so this is a stream that
         * has lost synchronisation. Skipping one byte is the only move that can
         * possibly recover, and the counter is what makes the situation
         * visible rather than merely quiet. */
        p->unknown_opcodes++;
        p->last_unknown = op;
        LOG_WARN_ONCE(LOG_VIDEO, "unknown GX opcode %02x", op);
        return 1;
    }
}

/* ------------------------------------------------------------------ */

void gx_parser_init(GXParser *p, const GXSink *sink)
{
    memset(p, 0, sizeof *p);
    if (sink)
        p->sink = *sink;
}

void gx_parser_reset(GXParser *p)
{
    GXSink saved = p->sink;
    memset(p, 0, sizeof *p);
    p->sink = saved;
}

u32 gx_parser_run(GXParser *p)
{
    Stream st;
    u32 consumed = 0;

    st.from_fifo = 1;
    st.base = 0;
    st.pos = 0;
    st.avail = gxfifo_readable();
    st.win = NULL; st.win_lo = 0; st.win_len = 0;

    for (;;) {
        u32 n = parse_one(p, &st);
        if (n == 0)
            break;
        st.pos += n;
        p->commands++;
        consumed = st.pos;
    }

    /* Consume in one call at the end rather than per command: the read pointer
     * is what the guest polls to pace itself, and moving it in steps would let
     * a title observe the GPU part-way through a command it considers atomic. */
    if (consumed)
        gxfifo_consume(consumed);
    return consumed;
}

u32 gx_parser_run_memory(GXParser *p, u32 addr, u32 size)
{
    Stream st;

    st.from_fifo = 0;
    st.base = addr;
    st.pos = 0;
    st.avail = size;
    st.win = NULL; st.win_lo = 0; st.win_len = 0;

    for (;;) {
        u32 n = parse_one(p, &st);
        if (n == 0)
            break;
        st.pos += n;
        p->commands++;
    }
    return st.pos;
}
