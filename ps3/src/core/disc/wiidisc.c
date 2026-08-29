/* wiidisc.c — see wiidisc.h. */
#include "wiidisc.h"

#include <string.h>

/* Everything on a disc is big-endian. */
static u32 be32(const u8 *p)
{
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

#define GC_MAGIC  0xC2339F3Du
#define WII_MAGIC 0x5D1C9EA3u

int wiidisc_parse_header(const void *raw, u32 size, DiscHeader *out)
{
    const u8 *p = (const u8 *)raw;
    unsigned i;

    if (size < 0x80)
        return -1;

    memset(out, 0, sizeof *out);

    for (i = 0; i < 6; i++)
        out->game_id[i] = (char)p[i];
    out->game_id[6] = '\0';
    out->disc_number = p[6];
    out->version     = p[7];

    /* The title is at 0x20, up to 64 bytes, NUL- or space-padded. */
    for (i = 0; i < 64; i++)
        out->title[i] = (char)p[0x20 + i];
    out->title[64] = '\0';
    /* Trim trailing spaces and NULs so the title compares cleanly. */
    for (i = 64; i > 0 && (out->title[i - 1] == ' ' || out->title[i - 1] == '\0');)
        out->title[--i] = '\0';

    /* The two magic words sit at different offsets, which is itself how the two
     * console generations are told apart. */
    if (be32(p + 0x18) == WII_MAGIC)
        out->kind = DISC_WII;
    else if (be32(p + 0x1C) == GC_MAGIC)
        out->kind = DISC_GAMECUBE;
    else
        out->kind = DISC_UNKNOWN;

    return out->kind == DISC_UNKNOWN ? -1 : 0;
}

int wiidisc_read_header(DiscRead read, void *ctx, DiscHeader *out)
{
    u8 buf[0x80];
    if (read(ctx, 0, buf, sizeof buf) != 0)
        return -1;
    return wiidisc_parse_header(buf, sizeof buf, out);
}

int wiidisc_read_partitions(DiscRead read, void *ctx, WiiPartitionTable *out)
{
    u8 table[0x20];
    unsigned g;

    memset(out, 0, sizeof *out);
    out->data_index = -1;

    /* The partition-group table is at 0x40000: four groups, each a (count,
     * offset) pair. Offsets on a Wii disc are stored in units of 4 bytes, so
     * they are shifted left by 2 to become byte offsets. */
    if (read(ctx, 0x40000, table, sizeof table) != 0)
        return -1;

    for (g = 0; g < 4; g++) {
        u32 n   = be32(table + g * 8);
        u64 off = (u64)be32(table + g * 8 + 4) << 2;
        u32 i;

        if (n == 0 || n > 64 || off == 0)
            continue;

        for (i = 0; i < n && out->count < WIIDISC_MAX_PARTITIONS; i++) {
            u8 ent[8];
            if (read(ctx, off + (u64)i * 8, ent, sizeof ent) != 0)
                return -1;

            {
                WiiPartition *wp = &out->part[out->count];
                wp->offset = (u64)be32(ent) << 2;
                wp->type   = be32(ent + 4);
                if (wp->type == 0 && out->data_index < 0)
                    out->data_index = (int)out->count;
                out->count++;
            }
        }
    }

    return out->count == 0 ? -1 : 0;
}

int wiidisc_parse_boot(const void *boot, u32 size, int wii, DiscFst *out)
{
    const u8 *p = (const u8 *)boot;
    u32 shift = wii ? 2 : 0;

    if (size < 0x440)
        return -1;

    memset(out, 0, sizeof *out);
    out->wii = wii;
    /* boot.bin: DOL offset at 0x420, FST offset at 0x424, both in the
     * partition's address space and, on Wii, in 4-byte units. */
    out->dol_offset = be32(p + 0x420) << shift;
    out->fst_offset = be32(p + 0x424) << shift;
    return 0;
}

int wiidisc_set_fst(DiscFst *fst, const u8 *raw, u32 size, int wii)
{
    u32 count;

    if (size < 12)
        return -1;

    /* The root entry's size field is the total number of entries. */
    count = be32(raw + 8);
    if (count == 0 || (u64)count * 12u > size)
        return -1;

    fst->fst      = raw;
    fst->fst_size = size;
    fst->count    = count;
    fst->wii      = wii;
    fst->strings  = (const char *)(raw + count * 12u);
    return 0;
}

int wiidisc_fst_entry(const DiscFst *fst, u32 i, FstEntry *out)
{
    const u8 *e;
    u32 name_off, shift;

    if (i >= fst->count)
        return -1;

    e        = fst->fst + i * 12u;
    out->is_dir = (e[0] != 0);
    name_off = ((u32)e[1] << 16) | ((u32)e[2] << 8) | e[3];
    shift    = fst->wii ? 2 : 0;

    if (out->is_dir) {
        /* Directory: the "offset" is the parent index, the "size" is the index
         * one past the last child. */
        out->offset = be32(e + 4);
        out->size   = be32(e + 8);
    } else {
        out->offset = be32(e + 4) << shift;   /* byte offset in the partition */
        out->size   = be32(e + 8);            /* byte length                  */
    }

    /* Copy the name out of the string table, bounded. */
    {
        const char *s = fst->strings + name_off;
        const char *end = (const char *)fst->fst + fst->fst_size;
        unsigned k = 0;
        while (s + k < end && s[k] != '\0' && k < sizeof out->name - 1) {
            out->name[k] = s[k];
            k++;
        }
        out->name[k] = '\0';
    }
    return 0;
}

/* Walk the FST resolving a slash-separated path. The FST is a pre-order flat
 * array: a directory entry's `size` is the index just past its subtree, which
 * is exactly what bounds the search for a child without recursion. */
int wiidisc_find(const DiscFst *fst, const char *path, FstEntry *out)
{
    u32 scope_end = fst->count;    /* current directory's subtree end */
    u32 i = 1;                     /* entry 0 is the root; skip it     */
    const char *seg = path;

    if (*seg == '/')
        seg++;

    while (*seg) {
        const char *slash = seg;
        char want[256];
        unsigned wl = 0;
        int found = -1;

        while (*slash && *slash != '/')
            slash++;
        while (seg < slash && wl < sizeof want - 1)
            want[wl++] = *seg++;
        want[wl] = '\0';
        if (*seg == '/')
            seg++;

        /* Scan the current directory's immediate children for `want`. A child
         * directory's subtree is skipped over so only direct children match. */
        while (i < scope_end) {
            FstEntry ent;
            if (wiidisc_fst_entry(fst, i, &ent) != 0)
                return -1;

            if (strcmp(ent.name, want) == 0) {
                found = (int)i;
                if (*seg == '\0') {          /* last path segment: done */
                    *out = ent;
                    return (int)i;
                }
                if (!ent.is_dir)
                    return -1;               /* path continues into a file */
                i = i + 1;                    /* descend                 */
                scope_end = ent.size;
                break;
            }

            i = ent.is_dir ? ent.size : i + 1;   /* skip subtree or advance */
        }

        if (found < 0)
            return -1;
    }

    return -1;
}
