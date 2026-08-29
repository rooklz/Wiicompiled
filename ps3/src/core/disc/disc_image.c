/* disc_image.c — byte access to a mounted Wii disc.
 *
 * The layer under IOS's /dev/di: it answers "give me `len` bytes at this
 * partition-relative offset". How those bytes are stored is deliberately
 * hidden here -- for host bring-up they come from a flat file of the decrypted
 * DATA partition (extracted from the RVZ image once). The console cannot carry
 * a 4 GiB partition, so it mounts a *slice* instead: the handful of ranges a
 * boot actually reads, carved out by tools/mkdiscslice.py and carried in
 * memory. /dev/di and the rest of IOS do not care which; they call
 * disc_image_read().
 */
#include "disc_image.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__lv2ppu__)
#include <sys/file.h>
#endif

static FILE *s_fp;
static u64   s_size;

/* A mounted slice: "DSLC", u32 count, count x { u64 offset, u32 len, u32 pos },
 * then the data. All big-endian, which is native here. */
static const u8 *s_slice;
static u32       s_slice_count;

/* A slice served from a file instead of memory: same DSLC layout, but the
 * payload stays on disk and every read seeks. Needed because the *complete*
 * disc -- 2.5 GiB once the FST names every file -- cannot live in an EBOOT or
 * in RAM. The seek must be 64-bit: newlib fseek takes a 32-bit long, which a
 * 2.5 GiB payload walks straight past, so the console uses lv2 directly. */
static u32 rd32(const u8 *p);
static u64 rd64(const u8 *p);

typedef struct { u64 off; u32 len; u32 pos; } SliceRange;
static SliceRange *s_franges;
static u32         s_frange_count;
#if defined(__lv2ppu__)
static s32         s_ffd = -1;
#else
static FILE       *s_ffp;
#endif

/* JIT_AOT_TRACE (measurement builds only, tests/test_mkwii_jit.c): host
 * disc reads are one of the things a loading stall can be made of, so the
 * freeze instrumentation times them. Compiled out entirely otherwise. */
#ifdef JIT_AOT_TRACE
extern void disc_trace_read(u64 t0_ns, u32 n);
extern u64  trace_now_ns(void);
static int slice_file_pread_raw(void *dst, u64 pos, u32 n);
static int slice_file_pread(void *dst, u64 pos, u32 n)
{
    u64 t0 = trace_now_ns();
    int rc = slice_file_pread_raw(dst, pos, n);
    disc_trace_read(t0, n);
    return rc;
}
#define slice_file_pread slice_file_pread_raw
#endif

static int slice_file_pread(void *dst, u64 pos, u32 n)
{
#if defined(__lv2ppu__)
    u64 done = 0;
    if (sysLv2FsLSeek64(s_ffd, (s64)pos, SEEK_SET, &done) != 0)
        return -1;
    if (sysLv2FsRead(s_ffd, dst, n, &done) != 0 || done != n)
        return -1;
    return 0;
#else
    if (fseeko(s_ffp, (off_t)pos, SEEK_SET) != 0)
        return -1;
    return fread(dst, 1, n, s_ffp) == n ? 0 : -1;
#endif
}

int disc_slice_open(const char *path)
{
    u8 hdr[8];
    u8 *tab;
    u32 i;

#if defined(__lv2ppu__)
    if (sysLv2FsOpen(path, SYS_O_RDONLY, &s_ffd, 0, NULL, 0) != 0)
        { s_ffd = -1; return -1; }
#else
    s_ffp = fopen(path, "rb");
    if (!s_ffp)
        return -1;
#endif
    if (slice_file_pread(hdr, 0, 8) != 0 || memcmp(hdr, "DSLC", 4) != 0)
        return -1;
    s_frange_count = rd32(hdr + 4);
    if (s_frange_count == 0 || s_frange_count > 65536)
        return -1;
    tab = (u8 *)malloc((size_t)s_frange_count * 16);
    s_franges = (SliceRange *)malloc(s_frange_count * sizeof *s_franges);
    if (!tab || !s_franges || slice_file_pread(tab, 8, s_frange_count * 16) != 0)
        { free(tab); return -1; }
    for (i = 0; i < s_frange_count; i++) {
        s_franges[i].off = rd64(tab + i * 16);
        s_franges[i].len = rd32(tab + i * 16 + 8);
        s_franges[i].pos = rd32(tab + i * 16 + 12);
    }
    free(tab);
    return 0;
}

static u32 rd32(const u8 *p)
{
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

static u64 rd64(const u8 *p)
{
    return ((u64)rd32(p) << 32) | rd32(p + 4);
}

int disc_slice_mount(const void *data, u32 len)
{
    const u8 *p = (const u8 *)data;
    if (!p || len < 8 || p[0] != 'D' || p[1] != 'S' || p[2] != 'L' || p[3] != 'C')
        return -1;
    s_slice = p;
    s_slice_count = rd32(p + 4);
    return 0;
}

int disc_image_open(const char *path)
{
    if (s_fp)
        fclose(s_fp);
    s_fp = fopen(path, "rb");
    if (!s_fp)
        return -1;
    fseek(s_fp, 0, SEEK_END);
    s_size = (u64)ftell(s_fp);
    fseek(s_fp, 0, SEEK_SET);
    return 0;
}

void disc_image_close(void)
{
    if (s_fp) { fclose(s_fp); s_fp = NULL; }
    s_size = 0;
}

int disc_image_mounted(void)
{
    return s_fp != NULL || s_slice != NULL || s_franges != NULL;
}

u64 disc_image_size(void)
{
    return s_size;
}

int disc_image_read(u64 partition_offset, void *dst, u32 len)
{
    size_t got;

    if (s_franges) {
        /* File-backed slice. A request may straddle a range edge; serve the
         * covered part, zero the rest, exactly as the memory slice does. */
        u32 i;
        memset(dst, 0, len);
        for (i = 0; i < s_frange_count; i++) {
            SliceRange *r = &s_franges[i];
            if (partition_offset >= r->off &&
                partition_offset < r->off + r->len) {
                u64 within = partition_offset - r->off;
                u32 take = (u32)(r->len - within);
                if (take > len) take = len;
                if (slice_file_pread(dst,
                        8ull + (u64)s_frange_count * 16 + r->pos + within,
                        take) != 0)
                    return -1;
                return 0;
            }
        }
        return 0;
    }

    if (s_slice) {
        /* Serve from whichever span covers the request; anything outside the
         * slice reads as zeros, exactly as unused disc space does. */
        const u8 *ent = s_slice + 8;
        u32 i;
        memset(dst, 0, len);
        for (i = 0; i < s_slice_count; i++, ent += 16) {
            u64 off = rd64(ent);
            u32 elen = rd32(ent + 8), epos = rd32(ent + 12);
            if (partition_offset >= off && partition_offset < off + elen) {
                u64 within = partition_offset - off;
                u32 take = (u32)(elen - within);
                if (take > len) take = len;
                memcpy(dst, s_slice + 8 + s_slice_count * 16 + epos + within, take);
                return 0;
            }
        }
        return 0;
    }

    if (!s_fp)
        return -1;
    /* A read past the end returns what exists and zero-fills the rest, the way
     * a real drive returns zeros for unused space rather than failing. */
    if (partition_offset >= s_size) {
        memset(dst, 0, len);
        return 0;
    }
    if (fseek(s_fp, (long)partition_offset, SEEK_SET) != 0)
        return -1;
    got = fread(dst, 1, len, s_fp);
    if (got < len)
        memset((char *)dst + got, 0, len - got);
    return 0;
}
