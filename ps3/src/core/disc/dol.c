/* dol.c — GameCube/Wii executable loader.
 *
 * A DOL has no relocation and no dynamic linking: it is a list of sections,
 * each with a file offset, a load address and a size, that are copied verbatim
 * into guest memory before control jumps to the entry point. That simplicity is
 * exactly why it is the right first loader -- being able to load one is the step
 * from "a CPU that executes instructions" to "a machine that boots a program".
 *
 * The one thing this file is strict about is *bounds*. A DOL is untrusted input;
 * a section whose load address or size falls outside real guest memory would,
 * copied blindly, scribble over the emulator's own state. Every section is
 * checked against the guest memory map before a byte is written.
 */
#include "dol.h"
#include "../mem/memmap.h"
#include "../../common/log.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */

static u32 rd32(const u8 *p) { return dol_be32(p); }

int dol_parse_header(const void *raw, size_t size, DolHeader *out)
{
    const u8 *p = (const u8 *)raw;
    unsigned i;

    if (size < DOL_HEADER_SIZE) {
        LOG_ERROR(LOG_DISC, "DOL too small (%zu bytes)", size);
        return -1;
    }

    memset(out, 0, sizeof *out);

    for (i = 0; i < DOL_NUM_TEXT; i++) out->text_offset[i]  = rd32(p + 0x00 + i * 4);
    for (i = 0; i < DOL_NUM_DATA; i++) out->data_offset[i]  = rd32(p + 0x1C + i * 4);
    for (i = 0; i < DOL_NUM_TEXT; i++) out->text_address[i] = rd32(p + 0x48 + i * 4);
    for (i = 0; i < DOL_NUM_DATA; i++) out->data_address[i] = rd32(p + 0x64 + i * 4);
    for (i = 0; i < DOL_NUM_TEXT; i++) out->text_size[i]    = rd32(p + 0x90 + i * 4);
    for (i = 0; i < DOL_NUM_DATA; i++) out->data_size[i]    = rd32(p + 0xAC + i * 4);

    out->bss_address = rd32(p + 0xD8);
    out->bss_size    = rd32(p + 0xDC);
    out->entry_point = rd32(p + 0xE0);

    return 0;
}

/* A section is acceptable if [address, address+size) lies wholly within one
 * guest RAM region and [offset, offset+size) within the file. */
static int section_ok(const char *what, unsigned idx, u32 addr, u32 off,
                      u32 sz, size_t filesize)
{
    if (sz == 0)
        return 1;                       /* empty section: nothing to place */

    if ((u64)off + sz > filesize) {
        LOG_ERROR(LOG_DISC, "%s%u runs past end of file (off %08x + %u > %zu)",
                  what, idx, off, sz, filesize);
        return 0;
    }
    if (!mem_is_ram(addr) || mem_valid_span(addr) < sz) {
        LOG_ERROR(LOG_DISC, "%s%u load address %08x (+%u) is not in guest RAM",
                  what, idx, addr, sz);
        return 0;
    }
    return 1;
}

int dol_load(const void *raw, size_t size, DolHeader *out_header)
{
    DolHeader h;
    const u8 *p = (const u8 *)raw;
    unsigned i;

    if (dol_parse_header(raw, size, &h) != 0)
        return -1;

    /* Validate every section before writing any, so a bad DOL is rejected
     * whole rather than half-loaded. */
    for (i = 0; i < DOL_NUM_TEXT; i++)
        if (!section_ok("text", i, h.text_address[i], h.text_offset[i],
                        h.text_size[i], size))
            return -1;
    for (i = 0; i < DOL_NUM_DATA; i++)
        if (!section_ok("data", i, h.data_address[i], h.data_offset[i],
                        h.data_size[i], size))
            return -1;

    if (h.bss_size && (!mem_is_ram(h.bss_address) ||
                       mem_valid_span(h.bss_address) < h.bss_size)) {
        /* BSS overlapping the end of RAM is common and harmless as long as the
         * part in RAM is what gets cleared; clamp rather than reject. */
        LOG_WARN(LOG_DISC, "BSS %08x+%u extends past RAM; clamping",
                 h.bss_address, h.bss_size);
    }

    /* Zero the BSS FIRST, then load the sections on top. A DOL's BSS range can
     * overlap its initialised data sections (Mario Kart Wii's does: its BSS
     * spans several .data sections holding pre-initialised function pointers).
     * Clearing BSS after loading would wipe those pointers -- so data must win.
     * This matches how the real apploader leaves memory before the game runs. */
    if (h.bss_size) {
        u32 span = mem_valid_span(h.bss_address);
        u32 n = (span < h.bss_size) ? span : h.bss_size;
        u32 done = 0;
        u8 zeros[256];
        memset(zeros, 0, sizeof zeros);
        while (done < n) {
            u32 chunk = (n - done < sizeof zeros) ? (n - done) : sizeof zeros;
            mem_write_block(h.bss_address + done, zeros, chunk);
            done += chunk;
        }
        LOG_DEBUG(LOG_DISC, "bss  -> %08x (%u bytes cleared)", h.bss_address, n);
    }

    /* Copy the sections. */
    for (i = 0; i < DOL_NUM_TEXT; i++) {
        if (!h.text_size[i]) continue;
        mem_write_block(h.text_address[i], p + h.text_offset[i], h.text_size[i]);
        LOG_DEBUG(LOG_DISC, "text%u -> %08x (%u bytes)",
                  i, h.text_address[i], h.text_size[i]);
    }
    for (i = 0; i < DOL_NUM_DATA; i++) {
        if (!h.data_size[i]) continue;
        mem_write_block(h.data_address[i], p + h.data_offset[i], h.data_size[i]);
        LOG_DEBUG(LOG_DISC, "data%u -> %08x (%u bytes)",
                  i, h.data_address[i], h.data_size[i]);
    }

    LOG_INFO(LOG_DISC, "DOL loaded: entry %08x", h.entry_point);
    if (out_header)
        *out_header = h;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Boot state                                                          */
/*                                                                     */
/* The IPL leaves a specific machine state before jumping into a title: paired  */
/* singles enabled, the BAT-mapped address space configured, and a handful of   */
/* low-memory globals a title's runtime reads. A homebrew DOL is far less        */
/* demanding than a retail title -- most read only the console-type and memory-  */
/* size globals -- so this sets the essential ones and can grow as titles ask   */
/* for more.                                                                     */
/* ------------------------------------------------------------------ */

void dol_setup_boot_state(PPCState *s, const DolHeader *h, int wii_mode)
{
    memset(s, 0, sizeof *s);

    /* Gekko/Broadway come out of the IPL with paired singles and the quantized
     * load/store unit already enabled; a DOL assumes this. */
    s->hid2      = HID2_PSE | HID2_LSQE | HID2_WPE;
    s->msr       = MSR_FP | MSR_ME | MSR_RI;
    s->const_one = 1.0;
    s->fprf_src  = s->fprf_ack = FPRF_SRC_NONE;   /* see ppc_init_constants */

    s->pc  = h->entry_point;
    s->npc = h->entry_point + 4;

    /* A stack near the top of MEM1, 8-aligned as the ABI requires. The title's
     * own runtime relocates this almost immediately, but it must be valid for
     * the first few instructions. */
    s->gpr[1] = (MEM1_CACHED + MEM1_SIZE - 0x10000) & ~0xFu;

    /* Low-memory OS globals the boot code reads. Addresses are the documented
     * GameCube/Wii OSGlobals; values describe the machine we present. */
    mem_write32(0x80000028, wii_mode ? MEM2_SIZE : 0);        /* MEM2 size (Wii) */
    mem_write32(0x8000002C, wii_mode ? 0x00000011 : 0x00000001); /* console type */
    mem_write32(0x80000030, 0);                               /* ARENA lo        */
    mem_write32(0x80000034, MEM1_CACHED + MEM1_SIZE - 0x10000); /* ARENA hi       */
    mem_write32(0x800000F0, MEM1_SIZE);                       /* simulated mem sz */
    mem_write32(0x800000F8, 0x0E7BE2C0);                      /* bus clock speed  */
    mem_write32(0x800000FC, 0x2B73A840);                      /* CPU clock speed  */

    /* The disc-game magic word tells the runtime a disc is present. Homebrew
     * usually ignores it; retail bootstraps check it. */
    mem_write32(0x80000020, 0x0D15EA5E);
    mem_write32(0x80000024, 0x00000001);

    s->downcount = 0;
    s->exit_requested = 0;
}

/* ------------------------------------------------------------------ */

void dol_describe(const DolHeader *h, void (*out)(void *, const char *),
                  void *ctx)
{
    char line[128];
    unsigned i;
    u32 total = 0;

    snprintf(line, sizeof line, "DOL entry %08x, bss %08x+%u",
             h->entry_point, h->bss_address, h->bss_size);
    out(ctx, line);

    for (i = 0; i < DOL_NUM_TEXT; i++) {
        if (!h->text_size[i]) continue;
        snprintf(line, sizeof line, "  text%u %08x  %6u bytes",
                 i, h->text_address[i], h->text_size[i]);
        out(ctx, line);
        total += h->text_size[i];
    }
    for (i = 0; i < DOL_NUM_DATA; i++) {
        if (!h->data_size[i]) continue;
        snprintf(line, sizeof line, "  data%u %08x  %6u bytes",
                 i, h->data_address[i], h->data_size[i]);
        out(ctx, line);
        total += h->data_size[i];
    }
    snprintf(line, sizeof line, "  %u bytes across sections", total);
    out(ctx, line);
}
