/* ios_hle.c — answers IOS IPC requests for the early boot of a retail title.
 *
 * Spec and citations: docs/IOS_HLE_SPEC.md. Shapes and numbers mirror Dolphin's
 * Source/Core/Core/IOS (IOS.cpp, Device.cpp, DI/DI.cpp, ES/ES.cpp,
 * FS/FileSystemProxy.cpp, STM/STM.cpp) so a game that boots under Dolphin sees
 * the same answers here.
 *
 * Ownership split: hw/ipc.c owns the mailbox (X1/Y2/Y1/X2, IRQ pacing) and the
 * reply write-back; this file owns "what does the request mean and what comes
 * back". A handler either produces a result now (IOS_DISPATCH_REPLY) or parks
 * the request (IOS_DISPATCH_PARKED) to be completed later via ipc_queue_reply().
 */
#include "ios_hle.h"
#include "../../common/log.h"

#include <string.h>
#include <stdio.h>
#include "../core_timing.h"
#include <stdlib.h>

/* ------------------------------------------------------------------ fd table */

typedef enum {
    DEV_NONE = 0,
    DEV_STM_IMM,        /* /dev/stm/immediate                       */
    DEV_STM_EH,         /* /dev/stm/eventhook                       */
    DEV_FS,             /* /dev/fs                                  */
    DEV_FS_FILE,        /* a NAND file opened by path               */
    DEV_DI,             /* /dev/di                                  */
    DEV_ES,             /* /dev/es                                  */
    DEV_BT,             /* /dev/usb/oh1/57e/305 — parked            */
    DEV_KD_TIME,        /* /dev/net/kd/time -- the WiiConnect24 clock */
    DEV_STUB,           /* opened successfully, everything returns 0 */
} dev_kind;

#define IOS_MAX_FDS 24              /* IPC_MAX_FDS = 0x18 (Dolphin IOS.h) */

typedef struct {
    dev_kind kind;
    int      file;                  /* DEV_FS_FILE: index into s_files    */
    u32      pos;                   /* DEV_FS_FILE: seek position         */
} fd_entry;

static fd_entry s_fds[IOS_MAX_FDS];

/* ------------------------------------------------------------ virtual NAND  */

#define MAX_NAND_FILES 32

typedef struct {
    char name[64];
    u8  *data;
    u32  len;
    u32  cap;                           /* 0 for registered read-only blobs */
    int  grow;                          /* created dynamically: may realloc */
} nand_file;

static nand_file s_files[MAX_NAND_FILES];
static int       s_num_files;

static int nand_lookup(const char *path);
static int nand_create(const char *path);
static void nand_add_dir(const char *path);
static u32 nand_ensure(int fi, u32 need);

void ios_fs_provision_wc24(void)
{
    /* The directory skeleton every real NAND carries. */
    static const char *k_dirs[] = {
        "/tmp", "/import", "/meta", "/ticket", "/sys", "/shared1", "/shared2",
        "/shared2/sys", "/shared2/title", "/shared2/menu",
        "/shared2/menu/FaceLib", "/shared2/wc24", "/shared2/wc24/mbox",
        "/title", "/title/00000001", "/title/00000001/00000002",
        "/title/00000001/00000002/data", "/title/00010000",
        "/title/00010000/524d4345", "/title/00010000/524d4345/data",
        "/title/00010001", "/title/00010003", "/title/00010004",
        "/title/00010005", "/title/00010006", "/title/00010007",
    };
    unsigned d3;
    for (d3 = 0; d3 < sizeof k_dirs / sizeof k_dirs[0]; d3++)
        nand_add_dir(k_dirs[d3]);

    /* The WiiConnect24 files themselves are NO LONGER FABRICATED HERE: every
     * one of them is a real, structured NAND file (friend list, download
     * list, mailbox control blocks) that a title reads and validates, and a
     * hand-rolled approximation is the same "plausible lie" that has cost
     * this project three separate bug hunts. The authentic defaults now ship
     * as assets and are registered by the platform (see ios_fs_provision_wii
     * callers); this function only guarantees the directories exist. */
}

/* ------------------------------------------------------------------ */
/* NAND persistence                                                     */
/*                                                                      */
/* The virtual NAND lived only in RAM, so every launch recreated the save
 * from scratch: the title found no rksys.dat, built a fresh one, and any
 * progress from the previous session was gone. Saves are the whole point of
 * a save system, so the dynamically created files (the ones a title made:
 * rksys.dat, banner.bin, the WC24 volumes) are written out at shutdown and
 * read back at boot. Registered blobs -- SYSCONF, the WC24 defaults, the Mii
 * database -- are also restored if a previous session modified them, so the
 * console keeps its settings.
 *
 * The layout is deliberately dumb: one file per NAND path with '/' mapped to
 * '_', so it can be inspected, deleted or backed up with a file manager. */
static void nand_mangle(const char *path, char *out, unsigned n)
{
    unsigned i = 0;
    while (*path && i + 1 < n) {
        char c = *path++;
        out[i++] = (c == '/') ? '_' : c;
    }
    out[i] = 0;
}

/* An index file carries the real NAND paths, because mangling '/' to '_' is
 * not reversible (play_rec.dat already contains an underscore). */
void ios_fs_persist_save(const char *dir)
{
    char idxpath[256];
    FILE *idx;
    int i;
    unsigned saved = 0;

    snprintf(idxpath, sizeof idxpath, "%s/index.txt", dir);
    idx = fopen(idxpath, "wb");
    if (!idx) { LOG_ERROR(LOG_CORE, "NAND: cannot write %s", idxpath); return; }

    for (i = 0; i < s_num_files; i++) {
        char mangled[80], full[256];
        FILE *f;
        if (!s_files[i].data || s_files[i].len == 0) continue;
        nand_mangle(s_files[i].name, mangled, sizeof mangled);
        snprintf(full, sizeof full, "%s/%s", dir, mangled);
        f = fopen(full, "wb");
        if (!f) continue;
        if (fwrite(s_files[i].data, 1, s_files[i].len, f) == s_files[i].len) {
            fprintf(idx, "%s\t%s\n", s_files[i].name, mangled);
            saved++;
        }
        fclose(f);
    }
    fclose(idx);
    LOG_INFO(LOG_CORE, "NAND: persisted %u file(s) to %s", saved, dir);
}

void ios_fs_persist_load(const char *dir)
{
    char idxpath[256], line[192];
    FILE *idx;
    unsigned loaded = 0;

    snprintf(idxpath, sizeof idxpath, "%s/index.txt", dir);
    idx = fopen(idxpath, "rb");
    if (!idx) { LOG_INFO(LOG_CORE, "NAND: no saved state in %s", dir); return; }

    while (fgets(line, sizeof line, idx)) {
        char *tab = strchr(line, '\t');
        char full[256];
        FILE *f;
        long n;
        int fi;
        if (!tab) continue;
        *tab++ = 0;
        { char *nl = strchr(tab, '\n'); if (nl) *nl = 0; }

        snprintf(full, sizeof full, "%s/%s", dir, tab);
        f = fopen(full, "rb");
        if (!f) continue;
        fseek(f, 0, SEEK_END); n = ftell(f); fseek(f, 0, SEEK_SET);

        fi = nand_lookup(line);
        if (fi < 0) fi = nand_create(line);      /* a file the title made */
        if (fi >= 0 && n > 0) {
            /* Read the file's REAL size. nand_ensure() returns the capacity it
             * allocated, and it rounds up to 1 MiB granularity -- so using its
             * return value as the byte count asked fread() for more bytes than
             * the file holds, the "== want" check failed, and the length was
             * never set. Every grow-file whose size is not a whole number of
             * mebibytes therefore restored as LENGTH ZERO, silently: 13 of 17
             * files loaded on the console, and the missing one that mattered
             * was rksys.dat, so Mario Kart found an empty save, declared it
             * corrupt, and offered to recreate it. */
            u32 want = (u32)n;
            if (s_files[fi].grow) {
                u32 cap = nand_ensure(fi, want);
                if (cap < want) want = cap;      /* capacity-limited, not size */
            } else if (want > s_files[fi].len) {
                want = s_files[fi].len;
            }
            if (want && fread(s_files[fi].data, 1, want, f) == want) {
                if (s_files[fi].grow) s_files[fi].len = want;
                loaded++;
            } else if (want) {
                LOG_ERROR(LOG_CORE, "NAND: short restore of %s (%u bytes)",
                          s_files[fi].name, want);
            }
        }
        fclose(f);
    }
    fclose(idx);
    LOG_INFO(LOG_CORE, "NAND: restored %u file(s) from %s", loaded, dir);
}

u32 ios_fs_file_len(const char *path)
{
    int i = nand_lookup(path);
    return (i >= 0) ? s_files[i].len : 0u;
}

unsigned ios_fs_file_count(const char *path)
{
    unsigned n = 0;
    int i;
    for (i = 0; i < s_num_files; i++)
        if (strcmp(s_files[i].name, path) == 0) n++;
    return n;
}

void ios_fs_register_file(const char *path, u8 *data, u32 len)
{
    nand_file *f;
    int existing = nand_lookup(path);
    /* Replace rather than append. Two entries with the same name meant reads
     * resolved to whichever came first, which is how a zero-length SYSCONF
     * could shadow a correctly registered 16 KB one. */
    if (existing >= 0) {
        f = &s_files[existing];
        f->data = data;
        f->len  = len;
        return;
    }
    if (s_num_files >= MAX_NAND_FILES) return;
    f = &s_files[s_num_files++];
    snprintf(f->name, sizeof f->name, "%s", path);
    f->data = data;
    f->len  = len;
}

/* A title creates and rewrites NAND files as it runs -- its save, the Mii
 * database, WiiConnect24 bookkeeping. On a real console those live in flash; a
 * fresh console simply has empty ones. Rather than refuse the open (which stops
 * the title dead where a real one would carry on), give it a zero-length file
 * backed by memory, grown on demand. Contents do not persist across runs yet;
 * what matters here is that the file exists and can be written. */
/* Created files grow on demand up to this bound. The old fixed 1 MiB cap was
 * exactly one game short: MKWii's rksys.dat is 2.5 MiB, its create-save write
 * came back truncated, and the game showed its "could not write to Wii system
 * memory" screen -- the first legible thing this emulator ever rendered. */
#define NAND_DYNAMIC_MAX   (8u << 20)
#define NAND_DYNAMIC_FIRST (256u << 10)

static int nand_create(const char *path)
{
    nand_file *f;
    u8 *buf;

    if (s_num_files >= MAX_NAND_FILES)
        return -1;
    buf = (u8 *)calloc(1, NAND_DYNAMIC_FIRST);
    if (!buf)
        return -1;

    f = &s_files[s_num_files];
    snprintf(f->name, sizeof f->name, "%s", path);
    f->data = buf;
    f->len  = 0;                        /* empty until written */
    f->cap  = NAND_DYNAMIC_FIRST;
    f->grow = 1;
    return s_num_files++;
}

/* Ensure a growable file can hold `need` bytes; returns the usable limit. */
static u32 nand_ensure(int fi, u32 need)
{
    nand_file *f = &s_files[fi];
    if (need <= f->cap) return f->cap;
    if (!f->grow || need > NAND_DYNAMIC_MAX) {
        LOG_ERROR(LOG_CORE, "NAND: %s cannot grow to %u (cap %u, grow %d)",
                  f->name, need, f->cap, f->grow);
        return f->cap;
    }
    {
        /* Straight to the required size, 1 MiB granularity. The doubling chain
         * this replaces made four transient old+new peaks on the PS3's tight
         * heap; a silent failure here truncates a write, and a truncated
         * rksys.dat is exactly the "could not write to Wii system memory"
         * error screen. If it fails now, it fails LOUDLY. */
        u32 newcap = (need + 0xFFFFFu) & ~0xFFFFFu;
        u8 *nd;
        if (newcap > NAND_DYNAMIC_MAX) newcap = NAND_DYNAMIC_MAX;
        nd = realloc(f->data, newcap);
        if (!nd) {
            LOG_ERROR(LOG_CORE, "NAND: realloc(%u) FAILED for %s (cap stays %u)",
                      newcap, f->name, f->cap);
            return f->cap;
        }
        memset(nd + f->cap, 0, newcap - f->cap);
        f->data = nd; f->cap = newcap;
    }
    return f->cap;
}

static int nand_lookup(const char *path)
{
    for (int i = 0; i < s_num_files; i++)
        if (strcmp(s_files[i].name, path) == 0)
            return i;
    return -1;
}

/* -------------------------------------------------------------- device state */

static u64 s_di_partition;          /* raw byte offset of open partition   */
static int s_di_partition_open;
static u32 s_di_last_length;        /* for DVDLowGetLength (with IOS bug)  */
static unsigned s_di_reads;

/* Optional: record every decrypted-partition read, so the exact slice of the
 * disc a boot touches can be extracted and shipped instead of the whole 3.96
 * GiB image. */
static FILE *s_read_log;
void ios_di_log_reads(FILE *f) { s_read_log = f; }
static void (*s_read_hook)(unsigned long long, unsigned);
u64 g_di_last_offset; u64 g_di_reads_pub; u64 g_di_last_dst;

void ios_di_set_read_hook(void (*fn)(unsigned long long, unsigned))
{ s_read_hook = fn; }
static u32 s_di_error;              /* drive error: (state<<24)|code       */
static unsigned s_ios_opens;        /* how many IOS_Open calls succeeded    */
static unsigned s_di_probe_hits;    /* anti-piracy drive-error queries answered */

static u64 s_es_title_id;
static int s_es_title_active;

static u32 s_stm_eventhook_req;     /* parked eventhook request, 0 = none  */

#define IOS_MAX_OUTSTANDING 16
static struct { u32 req; u32 cmd; u32 fd; u32 arg0; u32 kind; u32 arg1; }
    s_out[IOS_MAX_OUTSTANDING];
static unsigned s_out_n;


void ios_di_set_partition(u64 part_offset)
{
    s_di_partition = part_offset;
    s_di_partition_open = 1;
}

void ios_es_set_title(u64 title_id)
{
    s_es_title_id = title_id;
    s_es_title_active = 1;
}

/* ------------------------------------------------------------------ helpers  */

static void read_guest_string(u32 addr, char *out, unsigned max)
{
    unsigned i;
    for (i = 0; i + 1 < max; i++) {
        u8 c = mem_read8(addr + i);
        if (c == 0) break;
        out[i] = (char)c;
    }
    out[i] = '\0';
}

/* ioctlv vector i (in vectors first, then io vectors). */
static void read_iovec(u32 vec_base, u32 i, u32 *paddr, u32 *len)
{
    *paddr = mem_read32(vec_base + i * IOS_IOVEC_SIZE);
    *len   = mem_read32(vec_base + i * IOS_IOVEC_SIZE + 4);
}

static void zero_guest(u32 addr, u32 len)
{
    for (u32 i = 0; i + 4 <= len; i += 4) mem_write32(addr + i, 0);
    for (u32 i = len & ~3u; i < len; i++) mem_write8(addr + i, 0);
}

static s32 alloc_fd(dev_kind kind)
{
    for (int i = 0; i < IOS_MAX_FDS; i++) {
        if (s_fds[i].kind == DEV_NONE) {
            s_fds[i].kind = kind;
            s_fds[i].file = -1;
            s_fds[i].pos  = 0;
            return i;
        }
    }
    return IPC_EMAX;                /* -5: too many open fds */
}

/* The directory namespace. A path "exists" as a directory if the system
 * provisions it, a title created it, or any held file lives beneath it.
 * Before this list existed, ReadDir succeeded for ANY path -- including
 * ".../banner.bin" -- and the SDK's NANDGetType concluded every nonexistent
 * file was an existing directory. MKWii's save flow then chose "open the
 * banner" over "create the banner", forever. */
#define MAX_NAND_DIRS 64
static char s_dirs[MAX_NAND_DIRS][64];
static int  s_num_dirs;

static void nand_add_dir(const char *path)
{
    int i;
    for (i = 0; i < s_num_dirs; i++)
        if (!strcmp(s_dirs[i], path)) return;
    if (s_num_dirs < MAX_NAND_DIRS)
        snprintf(s_dirs[s_num_dirs++], 64, "%s", path);
}

static int nand_dir_exists(const char *path)
{
    int i;
    size_t plen = strlen(path);
    if (!strcmp(path, "/")) return 1;
    for (i = 0; i < s_num_dirs; i++)
        if (!strcmp(s_dirs[i], path)) return 1;
    for (i = 0; i < s_num_files; i++)
        if (!strncmp(s_files[i].name, path, plen) &&
            s_files[i].name[plen] == '/') return 1;
    return 0;
}

/* ISFS_GETUSAGE (ioctlv 0x0C): in = path, out = u32 used blocks (0x4000 each)
 * and u32 used inodes under it. The SDK's NANDCheck subtracts this from the
 * filesystem totals to decide whether a save will fit; answering "success"
 * with the caller's buffers untouched fed it stack garbage, and its verdict
 * was the game's "could not write to Wii system memory" screen. */
static s32 fs_getusage(u32 nin, u32 nio, u32 vec)
{
    char path[65];
    u32 pp, pl, bp, bl, ip, il;
    u32 blocks = 0, inodes = 1;         /* the directory itself */
    int i;
    size_t plen;

    if (nin < 1 || nio < 2) return FS_EINVAL;
    read_iovec(vec, 0, &pp, &pl);
    read_guest_string(pp, path, sizeof path);
    plen = strlen(path);

    for (i = 0; i < s_num_files; i++) {
        if (strncmp(s_files[i].name, path, plen) == 0 &&
            s_files[i].name[plen] == '/') {
            blocks += (s_files[i].len + 0x3FFFu) >> 14;
            inodes++;
        }
    }
    read_iovec(vec, nin, &bp, &bl);
    read_iovec(vec, nin + 1, &ip, &il);
    if (bl < 4 || il < 4) return FS_EINVAL;
    mem_write32(bp, blocks);
    mem_write32(ip, inodes);
    LOG_INFO(LOG_CORE, "FS: GetUsage(\"%s\") -> %u blocks, %u inodes",
             path, blocks, inodes);
    return IPC_SUCCESS;
}

/* ISFS_READDIR (ioctlv 4). Two shapes, both from Dolphin FSDevice::ReadDirectory:
 * with one vector pair the caller only wants a count (max in, count out); with
 * two it also wants the names, 13 bytes each. Our virtual NAND holds only the
 * files a title has opened or created, so a listing is honest but short --
 * which is what a fresh console's directories look like anyway. Refusing the
 * call instead (as we did) leaves a title that lists a directory stuck. */
static s32 fs_readdir(u32 nin, u32 nio, u32 vec)
{
    char dir[65];
    u32 pp, pl, max_count = 0, list_addr = 0, count_addr = 0, cl;
    {
        u32 p0, l0;
        read_iovec(vec, 0, &p0, &l0);
        read_guest_string(p0, dir, sizeof dir);
        if (!nand_dir_exists(dir)) {
            LOG_INFO(LOG_CORE, "FS: ReadDir(\"%s\") -> ENOENT", dir);
            return FS_ENOENT;
        }
    }
    unsigned matched = 0;
    int i;

    if (nin == 0 || nin != nio || nin > 2) return FS_EINVAL;
    read_iovec(vec, 0, &pp, &pl);
    if (pl != 64) return FS_EINVAL;
    read_guest_string(pp, dir, sizeof dir);

    if (nin == 2) {
        u32 mp, ml;
        read_iovec(vec, 1, &mp, &ml);            /* in: max entries  */
        read_iovec(vec, 2, &list_addr, &cl);     /* io: the names    */
        read_iovec(vec, 3, &count_addr, &ml);    /* io: the count    */
        max_count = mem_read32(mp);
    } else {
        read_iovec(vec, 1, &count_addr, &cl);
        max_count = mem_read32(count_addr);
    }

    /* Names directly inside `dir`. */
    for (i = 0; i < s_num_files && matched < max_count; i++) {
        const char *name = s_files[i].name;
        size_t dl = strlen(dir);
        const char *tail;
        if (strncmp(name, dir, dl) != 0) continue;
        tail = name + dl;
        if (*tail == '/') tail++;
        else if (*tail != '\0') continue;
        if (*tail == '\0' || strchr(tail, '/')) continue;   /* not direct */
        if (list_addr) {
            unsigned k;
            for (k = 0; k < 12; k++)
                mem_write8(list_addr + matched * 13 + k,
                           k < strlen(tail) ? (u8)tail[k] : 0);
            mem_write8(list_addr + matched * 13 + 12, 0);
        }
        matched++;
    }

    if (count_addr) mem_write32(count_addr, matched);
    return IPC_SUCCESS;
}

/* ====================================================== /dev/net/kd/time === */
/* The WiiConnect24 clock. A title reads it during start-up, and the values only
 * have to be self-consistent: the adjusted UTC it hands out and the difference
 * from the RTC counter it was given. Numbers and layout follow Dolphin's
 * NetKDTime (IOCtl): a common result word at buffer_out+0, the 64-bit time at
 * buffer_out+4. */

#define IOCTL_NW24_GET_UNIVERSAL_TIME 0x14
#define IOCTL_NW24_SET_UNIVERSAL_TIME 0x15
#define IOCTL_NW24_UNIMPLEMENTED      0x16
#define IOCTL_NW24_SET_RTC_COUNTER    0x17
#define IOCTL_NW24_GET_TIME_DIFF      0x18

/* Wii epoch: seconds from 1970 to 2000, in the units the SDK uses. */
static u64 s_kd_utc = 0x0000000100000000ull;
static u32 s_kd_rtc;

static s32 kd_time_ioctl(u32 num, u32 in, u32 in_len, u32 out, u32 out_len)
{
    s32 result = 0;
    (void)in_len;

    switch (num) {
    case IOCTL_NW24_GET_UNIVERSAL_TIME:
        if (out_len >= 12) {
            mem_write32(out + 4, (u32)(s_kd_utc >> 32));
            mem_write32(out + 8, (u32)s_kd_utc);
        }
        break;

    case IOCTL_NW24_SET_UNIVERSAL_TIME:
        s_kd_utc = ((u64)mem_read32(in) << 32) | mem_read32(in + 4);
        break;

    case IOCTL_NW24_SET_RTC_COUNTER:
        s_kd_rtc = mem_read32(in);
        break;

    case IOCTL_NW24_GET_TIME_DIFF: {
        u64 diff = s_kd_utc - (u64)s_kd_rtc;
        if (out_len >= 12) {
            mem_write32(out + 4, (u32)(diff >> 32));
            mem_write32(out + 8, (u32)diff);
        }
        break;
    }

    case IOCTL_NW24_UNIMPLEMENTED:
        result = -9;
        break;

    default:
        LOG_WARN(LOG_CORE, "kd/time: unknown ioctl %#x", num);
        break;
    }

    if (out_len >= 4)
        mem_write32(out, 0);            /* the common result word */
    return result;
}

/* ============================================================= /dev/di ===== */
/* IOCTL: buffer_in is always 0x20 bytes, word 0 = cmd << 24, payload words    */
/* follow; result codes are the DIResult family (success == 1).                */
/* Numbers/semantics: Dolphin Core/IOS/DI/DI.{h,cpp}, StartIOCtl().            */

enum {
    DVDLowInquiry              = 0x12,
    DVDLowReadDiskID           = 0x70,
    DVDLowRead                 = 0x71,
    DVDLowWaitForCoverClose    = 0x79,
    DVDLowGetCoverRegister     = 0x7a,
    DVDLowNotifyReset          = 0x7e,
    DVDLowMaskCoverInterrupt   = 0x85,
    DVDLowClearCoverInterrupt  = 0x86,
    DVDLowGetCoverStatus       = 0x88,
    DVDLowUnmaskCoverInterrupt = 0x89,
    DVDLowReset                = 0x8a,
    DVDLowOpenPartition        = 0x8b,  /* ioctlv only */
    DVDLowClosePartition       = 0x8c,
    DVDLowUnencryptedRead      = 0x8d,
    DVDLowGetLength            = 0x83,
    DVDLowSeek                 = 0xab,
    DVDLowReportKey            = 0xa4,
    DVDLowRequestError         = 0xe0,
    DVDLowStopMotor            = 0xe3,
    DVDLowAudioBufferConfig    = 0xe4,
};

static s32 di_ioctl(u32 num, u32 in, u32 in_len, u32 out, u32 out_len)
{
    if (in_len != 0x20) {
        LOG_WARN(LOG_CORE, "DI: ioctl %#x with in_len %#x != 0x20", num, in_len);
        return DI_SECURITY;         /* Dolphin DI.cpp:147 */
    }

    switch (num) {
    case DVDLowInquiry:
        /* Drive info a real drive returns (Dolphin DVDInterface.cpp:810). */
        if (out_len < 0x20) return DI_TIMEDOUT;
        zero_guest(out, 0x20);
        mem_write32(out + 0, 0x00000002);   /* revision / device code */
        mem_write32(out + 4, 0x20060526);   /* release date           */
        mem_write32(out + 8, 0x41000000);   /* version                */
        return DI_SUCCESS;

    case DVDLowReadDiskID: {
        /* First 0x20 bytes of the raw disc (the header / disk ID). */
        if (out_len < 0x20) return DI_TIMEDOUT;
        void *dst = mem_ptr(out);
        if (!dst || disc_read_raw(0, 0x20, dst) != 0) return DI_DRIVE_ERROR;
        return DI_SUCCESS;
    }

    case DVDLowRead: {
        s_di_reads++;
        g_di_last_offset = (u64)mem_read32(in + 8) << 2;
        g_di_reads_pub = s_di_reads;
        { extern u64 g_di_last_dst; g_di_last_dst = out; }
        if (s_read_log) {
            fprintf(s_read_log, "%llu %u\n",
                    (unsigned long long)((u64)mem_read32(in + 8) << 2),
                    (unsigned)mem_read32(in + 4));
            fflush(s_read_log);
        }
        if (s_read_hook)
            s_read_hook((unsigned long long)((u64)mem_read32(in + 8) << 2),
                        (unsigned)mem_read32(in + 4));
        /* Decrypted partition read: w1 = length in bytes, w2 = offset >> 2
         * into the decrypted data area of the open partition. */
        u32 length   = mem_read32(in + 4);
        u32 position = mem_read32(in + 8);
        if (!s_di_partition_open) {
            LOG_WARN(LOG_CORE, "DI: DVDLowRead with no partition open");
            return DI_SECURITY;
        }
        if (out_len < length) {
            LOG_WARN(LOG_CORE, "DVDLowRead REJECTED: out_len %#x < length %#x",
                     (unsigned)out_len, (unsigned)length);
            return DI_SECURITY;
        }
        s_di_last_length = position;        /* faithful IOS bug (DI.cpp:201) */
        void *dst = mem_ptr(out);
        int rc = dst ? disc_read((u64)position << 2, length, dst) : -1;
        if (rc != 0)
            return DI_DRIVE_ERROR;
        return DI_SUCCESS;
    }

    case DVDLowUnencryptedRead: {
        /* Raw read, only inside the whitelisted ranges (32-bit offsets). */
        u32 length   = mem_read32(in + 4);
        u32 position = mem_read32(in + 8);
        u32 end      = position + (length >> 2);
        /* The disc "system area" [0, 0x14000) is genuinely readable. */
        if (position <= 0x00014000u && end <= 0x00014000u) {
            if (out_len < length) return DI_TIMEDOUT;
            s_di_last_length = length;
            void *dst = mem_ptr(out);
            if (!dst || disc_read_raw((u64)position << 2, length, dst) != 0)
                return DI_DRIVE_ERROR;
            return DI_SUCCESS;
        }
        /* The error-001 anti-piracy ranges: offsets just past the end of a
         * genuine disc but inside a bootleg DVD-R. Mario Kart Wii reads them
         * and demands the drive REJECT the read with BlockOOB (0x52100). A real
         * disc fails here; a bootleg succeeds -- and success is what the game
         * treats as "unauthorized device". So we fail exactly as a real drive
         * does: set the drive error and return DriveError (Dolphin:
         * DVDInterface.cpp:764-773 sets BlockOOB + DEINT; DI.cpp:620 maps DEINT
         * -> DIResult::DriveError). The code is then read via DVDLowRequestError. */
        if ((position >= 0x460A0000u && end <= 0x460A0008u) ||
            (position >= 0x7ED40000u && end <= 0x7ED40008u)) {
            s_di_error = 0x00052100u;       /* Ready | BlockOOB */
            return DI_DRIVE_ERROR;
        }
        LOG_WARN(LOG_CORE, "DI: unencrypted read outside legal region "
                 "(pos %#x len %#x)", position, length);
        return DI_SECURITY;
    }

    case DVDLowWaitForCoverClose:
        return DI_COVER_CLOSED;             /* 4 — this is its success value */

    case DVDLowGetCoverRegister:            /* DICVR: 0 = closed, quiet */
        if (out_len < 4) return DI_SECURITY;
        mem_write32(out, 0);
        return DI_SUCCESS;

    case DVDLowGetCoverStatus:              /* 2 = disc inside, 1 = none */
        if (out_len < 4) return DI_SECURITY;
        mem_write32(out, 2);
        return DI_SUCCESS;

    case DVDLowGetLength:
        if (out_len < 4) return DI_SECURITY;
        mem_write32(out, s_di_last_length);
        return DI_SUCCESS;

    case DVDLowReportKey:
        /* A DVD-Video CSS command. A genuine Wii drive rejects it, and the
         * anti-piracy check's third stage confirms the disc by reading back
         * that rejection (InvalidCommand, 0x52000). */
        s_di_error = 0x00052000u;
        return DI_DRIVE_ERROR;

    case DVDLowRequestError:
        if (s_di_error == 0x00052100u || s_di_error == 0x00052000u)
            s_di_probe_hits++;
        /* Report the packed drive error (state<<24 | code), then clear it --
         * exactly what Dolphin's RequestError does (DVDInterface.cpp:982-994).
         * After an error-001 OOB read this is 0x00052100, which is what the
         * anti-piracy check compares against to confirm a genuine disc. */
        if (out_len < 4) return DI_SECURITY;
        mem_write32(out, s_di_error);
        s_di_error = 0;
        return DI_SUCCESS;

    case DVDLowReset:
        /* Real IOS closes the partition here; we keep it because our loader
         * pre-opened it in place of the apploader (spec §5.1, boot note). */
        return DI_SUCCESS;

    case DVDLowClosePartition:
        s_di_partition_open = 0;
    s_di_error = 0;
    ios_bt_reset();
    s_ios_opens = 0;
    s_di_probe_hits = 0;
    s_di_reads = 0;
    s_out_n = 0;
        return DI_SUCCESS;

    case DVDLowNotifyReset:
    case DVDLowMaskCoverInterrupt:
    case DVDLowClearCoverInterrupt:
    case DVDLowUnmaskCoverInterrupt:
    case DVDLowSeek:
    case DVDLowStopMotor:
    case DVDLowAudioBufferConfig:
        return DI_SUCCESS;

    default:
        LOG_WARN(LOG_CORE, "DI: unknown ioctl %#x", num);
        return DI_SECURITY;                 /* Dolphin's default (DI.cpp:538) */
    }
}

/* DVDLowOpenPartition — the one DI ioctlv (3 in / 2 out; Dolphin DI.cpp:711).
 * in[0]=0x20 cmd block (w1 = partition offset >> 2), in[1]=ticket or 0,
 * in[2]=certs or 0; io[0]=raw TMD (0x49E4), io[1]=u32 ES error. */
static s32 di_ioctlv(u32 num, u32 nin, u32 nio, u32 vec)
{
    if (num != DVDLowOpenPartition) {
        LOG_WARN(LOG_CORE, "DI: unknown ioctlv %#x", num);
        return DI_BADARG;
    }
    if (nin != 3 || nio != 2) return DI_BADARG;

    u32 cmd_p, cmd_l, tmd_p, tmd_l, err_p, err_l;
    read_iovec(vec, 0, &cmd_p, &cmd_l);
    read_iovec(vec, 3, &tmd_p, &tmd_l);     /* io[0] */
    read_iovec(vec, 4, &err_p, &err_l);     /* io[1] */
    if (cmd_l != 0x20) return DI_BADARG;

    u64 part = (u64)mem_read32(cmd_p + 4) << 2;
    ios_di_set_partition(part);

    /* Partition header (raw): +0x2A4 u32 tmd_size, +0x2A8 u32 tmd_offset>>2. */
    u8 hdr[8];
    if (disc_read_raw(part + 0x2A4, 8, hdr) == 0) {
        u32 tmd_size = (u32)hdr[0] << 24 | hdr[1] << 16 | hdr[2] << 8 | hdr[3];
        u64 tmd_off  = (u64)((u32)hdr[4] << 24 | hdr[5] << 16 |
                             (u32)hdr[6] << 8 | hdr[7]) << 2;
        if (tmd_size > 0x49E4) tmd_size = 0x49E4;
        if (tmd_size > tmd_l)  tmd_size = tmd_l;
        void *dst = mem_ptr(tmd_p);
        if (dst && tmd_size)
            disc_read_raw(part + tmd_off, tmd_size, dst);
        /* Establishing the title context is ES_DIVerify's job on real IOS;
         * the title id lives at TMD + 0x18C. */
        if (tmd_size >= 0x194) {
            u64 tid = ((u64)mem_read32(tmd_p + 0x18C) << 32)
                    |        mem_read32(tmd_p + 0x190);
            ios_es_set_title(tid);
        }
    }
    if (err_l >= 4) mem_write32(err_p, 0);  /* ES error = success */

    LOG_INFO(LOG_CORE, "DI: OpenPartition @ %#llx", (unsigned long long)part);
    return DI_SUCCESS;
}

/* ============================================================= /dev/es ===== */
/* Everything is ioctlv (Dolphin Core/IOS/ES). -1017 = ES_EINVAL default.      */

enum {
    IOCTL_ES_GETDEVICEID    = 0x07,
    IOCTL_ES_GETCONSUMPTION = 0x16,
    IOCTL_ES_GETVIEWCNT     = 0x12,
    IOCTL_ES_GETVIEWS       = 0x13,
    IOCTL_ES_GETTMDVIEWCNT  = 0x14,
    IOCTL_ES_GETTMDVIEWS    = 0x15,
    IOCTL_ES_DIGETTICKETVIEW = 0x1B,
    IOCTL_ES_DIVERIFY       = 0x1C,
    IOCTL_ES_GETTITLEDIR    = 0x1D,
    IOCTL_ES_GETTITLEID     = 0x20,
};

static int ios_trace(void)
{
    static int t = -1;
    if (t < 0) t = getenv("DSP_TRACE") != NULL;
    return t;
}

static s32 es_ioctlv(u32 num, u32 nin, u32 nio, u32 vec)
{
    u32 p, l;

    if (ios_trace())
        fprintf(stderr, "[es] ioctlv %#x (%u in/%u io)\n", num, nin, nio);

    switch (num) {
    case IOCTL_ES_GETDEVICEID:              /* 0 in / 1 out (u32 NG id) */
        if (nin != 0 || nio != 1) return ES_EINVAL;
        read_iovec(vec, 0, &p, &l);
        if (l < 4) return ES_EINVAL;
        mem_write32(p, 0x0403AC68);         /* any plausible device id */
        return IPC_SUCCESS;

    case IOCTL_ES_GETTITLEID:               /* 0 in / 1 out (u64) */
        if (nin != 0 || nio != 1) return ES_EINVAL;
        if (!s_es_title_active)   return ES_EINVAL;
        read_iovec(vec, 0, &p, &l);
        if (l < 8) return ES_EINVAL;
        mem_write32(p,     (u32)(s_es_title_id >> 32));
        mem_write32(p + 4, (u32)(s_es_title_id));
        return IPC_SUCCESS;

    case IOCTL_ES_GETTITLEDIR: {            /* 1 in (u64 tid) / 1 out (path) */
        if (nin != 1 || nio != 1) return ES_EINVAL;
        u32 ip, il;
        read_iovec(vec, 0, &ip, &il);
        read_iovec(vec, 1, &p, &l);
        if (il < 8) return ES_EINVAL;
        u32 hi = mem_read32(ip), lo = mem_read32(ip + 4);
        char path[64];
        int n = snprintf(path, sizeof path, "/title/%08x/%08x/data", hi, lo);
        for (int i = 0; i <= n && (u32)i < l; i++)
            mem_write8(p + i, (u8)path[i]);
        return IPC_SUCCESS;
    }

    case IOCTL_ES_GETCONSUMPTION:           /* 1 in / 2 out; io[1] = u32 0 */
        if (nin != 1 || nio != 2) return ES_EINVAL;
        read_iovec(vec, 2, &p, &l);
        if (l >= 4) mem_write32(p, 0);
        return IPC_SUCCESS;

    case IOCTL_ES_GETVIEWCNT: {
        /* 1 in (u64 title id) / 1 io (u32 count). MKWii asks after creating
         * its save; refusing it crashed the boot -- the game used the
         * never-written count and called through a null pointer (executed the
         * disc header at physical 0 until the zero word at +8). One view for
         * any title is what a retail console reports for an owned title. */
        u32 op, ol;
        if (nin != 1 || nio != 1) return ES_EINVAL;
        read_iovec(vec, nin, &op, &ol);
        if (ol < 4) return ES_EINVAL;
        mem_write32(op, 1);
        return IPC_SUCCESS;
    }

    case IOCTL_ES_GETVIEWS: {
        /* 2 in (u64 title id, u32 count) / 1 io (count x 0xD8 views). Same
         * fabricated view as DIGETTICKETVIEW, with the *requested* title id
         * echoed back rather than the running title's. */
        u32 ip, il, op, ol;
        u64 tid;
        if (nin != 2 || nio != 1) return ES_EINVAL;
        read_iovec(vec, 0, &ip, &il);
        if (il < 8) return ES_EINVAL;
        tid = ((u64)mem_read32(ip) << 32) | mem_read32(ip + 4);
        read_iovec(vec, nin, &op, &ol);
        if (ol < 0xD8) return ES_EINVAL;
        zero_guest(op, 0xD8);
        mem_write8(op, 1);                   /* view version */
        mem_write32(op + 0x10, (u32)(tid >> 32));
        mem_write32(op + 0x14, (u32)tid);
        return IPC_SUCCESS;
    }

    case IOCTL_ES_GETTMDVIEWCNT: {
        /* 1 in (u64 title id) / 1 io (u32 *size in bytes* of the TMD view --
         * a size, not a count, despite the name; Dolphin ES_GetTMDViewSize).
         * The game allocates this then asks for the view itself, and -- as
         * with the ticket views -- uses the result without checking, so a
         * refusal is a jump through null. */
        u32 op, ol;
        if (nin != 1 || nio != 1) return ES_EINVAL;
        read_iovec(vec, nin, &op, &ol);
        if (ol < 4) return ES_EINVAL;
        mem_write32(op, 0x64 + 0x10);       /* base + one content entry */
        return IPC_SUCCESS;
    }

    case IOCTL_ES_GETTMDVIEWS: {
        /* 2 in (u64 title id, u32 size) / 1 io (the view). Zeroed but
         * self-consistent: the title id at its offset (0x0C), one content,
         * so the declared size and the view agree. */
        u32 ip, il, op, ol;
        u64 tid;
        if (nin != 2 || nio != 1) return ES_EINVAL;
        read_iovec(vec, 0, &ip, &il);
        if (il < 8) return ES_EINVAL;
        tid = ((u64)mem_read32(ip) << 32) | mem_read32(ip + 4);
        read_iovec(vec, nin, &op, &ol);
        if (ol < 0x74) return ES_EINVAL;
        zero_guest(op, 0x74);
        mem_write32(op + 0x0C, (u32)(tid >> 32));
        mem_write32(op + 0x10, (u32)tid);
        mem_write16(op + 0x60, 1);          /* num_contents */
        return IPC_SUCCESS;
    }

    case IOCTL_ES_DIGETTICKETVIEW: {
        /* No ticket in -> return the running title's ticket view (0xD8 bytes).
         * The game reads the title id from it (view offset 0x10). Layout:
         * Dolphin ES/Formats (GetRawTicketView): version byte, then the ticket
         * from its ticket_id field, so title_id lands at 0x10. */
        u32 op, ol;
        if (nio != 1) return ES_EINVAL;
        if (!s_es_title_active) return ES_EINVAL;
        read_iovec(vec, nin, &op, &ol);      /* first io-vector */
        if (ol < 0xD8) return ES_EINVAL;
        zero_guest(op, 0xD8);
        mem_write8(op, 1);                   /* view version */
        mem_write32(op + 0x10, (u32)(s_es_title_id >> 32));
        mem_write32(op + 0x14, (u32)s_es_title_id);
        return IPC_SUCCESS;
    }

    case IOCTL_ES_DIVERIFY:
        /* Not callable from the PPC on real IOS; Dolphin blocks it too
         * (ES.cpp:805). The context comes from boot seeding / OpenPartition. */
        return ES_EINVAL;

    default:
        LOG_WARN(LOG_CORE, "ES: unimplemented ioctlv %#x (%u in/%u io)",
                 num, nin, nio);
        return ES_EINVAL;
    }
}

/* ============================================================= /dev/fs ===== */
/* /dev/fs fd: management ioctls; other fds: NAND files (READ/SEEK/CLOSE and   */
/* ioctl 11 GetFileStats). Numbers: Dolphin FS/FileSystemProxy.h.              */

enum {
    ISFS_IOCTL_GETSTATS     = 2,
    ISFS_IOCTL_CREATEDIR    = 3,
    ISFS_IOCTL_SETATTR      = 5,
    ISFS_IOCTL_GETATTR      = 6,
    ISFS_IOCTL_DELETE       = 7,
    ISFS_IOCTL_CREATEFILE   = 9,
    ISFS_IOCTL_RENAME       = 8,
    ISFS_IOCTL_GETFILESTATS = 11,
    ISFS_IOCTL_SHUTDOWN     = 13,
};

static s32 fs_ioctl(fd_entry *fd, u32 num, u32 in, u32 in_len,
                    u32 out, u32 out_len)
{
    (void)in; (void)in_len;

    if (fd->kind == DEV_FS_FILE) {
        if (num == ISFS_IOCTL_GETFILESTATS) {   /* out: {u32 size, u32 pos} */
            if (out_len < 8) return FS_EINVAL;
            mem_write32(out,     s_files[fd->file].len);
            mem_write32(out + 4, fd->pos);
            return IPC_SUCCESS;
        }
        return FS_EINVAL;
    }

    switch (num) {
    case ISFS_IOCTL_GETSTATS:               /* 7 u32: plausible NAND stats */
        if (out_len < 0x1C) return FS_EINVAL;
        mem_write32(out + 0x00, 0x4000);    /* cluster size    */
        mem_write32(out + 0x04, 0x5DEC);    /* free clusters   */
        mem_write32(out + 0x08, 0x1DD4);    /* used clusters   */
        mem_write32(out + 0x0C, 0x10);      /* bad clusters    */
        mem_write32(out + 0x10, 0x02F0);    /* reserved        */
        mem_write32(out + 0x14, 0x146B);    /* free inodes     */
        mem_write32(out + 0x18, 0x0394);    /* used inodes     */
        return IPC_SUCCESS;

    case ISFS_IOCTL_CREATEDIR: {
        char path[65];
        read_guest_string(in + 6, path, sizeof path);
        nand_add_dir(path);
        LOG_INFO(LOG_CORE, "FS: CreateDir(\"%s\") -> 0", path);
        return IPC_SUCCESS;
    }
    case ISFS_IOCTL_RENAME: {
        /* in: two 64-byte paths, old then new. The save flow stages files in
         * /tmp and renames them into the title directory -- the last call in
         * the banner's journey. Silently replaces an existing target, as
         * real IOS does. */
        char from[65], to[65];
        int fi2, ti2;
        read_guest_string(in, from, sizeof from);
        read_guest_string(in + 64, to, sizeof to);
        fi2 = nand_lookup(from);
        if (fi2 < 0) {
            LOG_INFO(LOG_CORE, "FS: Rename(\"%s\") -> ENOENT", from);
            return FS_ENOENT;
        }
        ti2 = nand_lookup(to);
        if (ti2 >= 0 && ti2 != fi2) {
            if (s_files[ti2].grow) free(s_files[ti2].data);
            memmove(&s_files[ti2], &s_files[ti2 + 1],
                    (size_t)(s_num_files - ti2 - 1) * sizeof s_files[0]);
            s_num_files--;
            if (fi2 > ti2) fi2--;
        }
        snprintf(s_files[fi2].name, sizeof s_files[fi2].name, "%s", to);
        LOG_INFO(LOG_CORE, "FS: Rename(\"%s\" -> \"%s\") -> 0", from, to);
        return IPC_SUCCESS;
    }

    case ISFS_IOCTL_SHUTDOWN:
        return IPC_SUCCESS;

    case ISFS_IOCTL_SETATTR: {
        /* The save library sets ownership and permissions on every file it
         * has just created, as its LAST step. Rejecting it (this fell into
         * the unimplemented default and returned EINVAL) fails the save at
         * the finish line, which the title reports as broken system memory.
         * In: u32 uid, u16 gid, char path[64] at +6, then owner/group/other
         * permission bytes and the attribute byte -- the same layout GetAttr
         * reports back. We hold no per-file ownership, so the honest answer
         * is: the path must exist, and then the operation succeeds. */
        char path[65];
        if (in_len < 0x4A) return FS_EINVAL;
        read_guest_string(in + 6, path, sizeof path);
        if (nand_lookup(path) < 0 && !nand_dir_exists(path)) {
            LOG_INFO(LOG_CORE, "FS: SetAttr(\"%s\") -> ENOENT", path);
            return FS_ENOENT;
        }
        LOG_INFO(LOG_CORE, "FS: SetAttr(\"%s\") -> 0", path);
        return IPC_SUCCESS;
    }

    case ISFS_IOCTL_GETATTR: {
        /* Attributes of an existing path must SUCCEED: the save library
         * verifies what it just created, and an ENOENT here reads as "NAND
         * broken" -- the exact on-screen message. Out: uid, gid, attr, and
         * three permission bytes; full access for everything we hold. */
        char path[65];
        read_guest_string(in, path, sizeof path);
        if (nand_lookup(path) < 0 && !nand_dir_exists(path)) {
            LOG_INFO(LOG_CORE, "FS: GetAttr(\"%s\") -> ENOENT", path);
            return FS_ENOENT;
        }
        if (out_len < 0x10) return FS_EINVAL;
        mem_write32(out, 0);                 /* uid          */
        mem_write16(out + 4, 0);             /* gid          */
        mem_write8(out + 6, 0);              /* attributes   */
        mem_write8(out + 7, 3);              /* owner: rw    */
        mem_write8(out + 8, 3);              /* group: rw    */
        mem_write8(out + 9, 3);              /* other: rw    */
        LOG_INFO(LOG_CORE, "FS: GetAttr(\"%s\") -> 0", path);
        return IPC_SUCCESS;
    }

    case ISFS_IOCTL_DELETE: {
        char path[65];
        int fi;
        read_guest_string(in, path, sizeof path);
        fi = nand_lookup(path);
        if (fi < 0) {
            LOG_INFO(LOG_CORE, "FS: Delete(\"%s\") -> ENOENT", path);
            return FS_ENOENT;
        }
        /* Drop the file: growable data is freed, registered blobs are only
         * unlinked (their storage belongs to the loader). Compact the table
         * so lookup stays a plain scan. */
        if (s_files[fi].grow) free(s_files[fi].data);
        memmove(&s_files[fi], &s_files[fi + 1],
                (size_t)(s_num_files - fi - 1) * sizeof s_files[0]);
        s_num_files--;
        LOG_INFO(LOG_CORE, "FS: Delete(\"%s\") -> 0", path);
        return IPC_SUCCESS;
    }

    case ISFS_IOCTL_CREATEFILE: {
        char path[65];
        /* ISFSParams: u32 uid, u16 gid, char path[64], ... — path at +6 for
         * SETATTR/CREATE*, at +0 for DELETE/GETATTR (just a 64-byte path). */
        u32 at = (num == ISFS_IOCTL_CREATEFILE) ? in + 6 : in;
        read_guest_string(at, path, sizeof path);
        if (num == ISFS_IOCTL_CREATEFILE) {
            /* Actually create: this is the one place files come into being.
             * The subsequent open must find it, or the title's
             * ENOENT -> CreateFile -> open recovery loops forever. */
            if (nand_lookup(path) < 0 && nand_create(path) < 0)
                return FS_EINVAL;
            LOG_INFO(LOG_CORE, "FS: CreateFile(\"%s\") -> 0", path);
            return IPC_SUCCESS;
        }
        LOG_INFO(LOG_CORE, "FS: ioctl %u on \"%s\" -> ENOENT stub", num, path);
        return FS_ENOENT;
    }

    default:
        LOG_WARN(LOG_CORE, "FS: unimplemented ioctl %u", num);
        return FS_EINVAL;
    }
}

static s32 fs_read(fd_entry *fd, u32 buf, u32 len)
{
    if (ios_trace() && fd->file >= 0)
        fprintf(stderr, "[fs] read %s pos=%u len=%u flen=%u\n",
                s_files[fd->file].name, fd->pos, len, s_files[fd->file].len);

    nand_file *f = &s_files[fd->file];
    u32 n = len;
    {   /* A 16 KB read returns 16384 bytes under qemu and 0 on the console,
         * and 16384 is exactly SYSCONF -- so the title never gets its
         * registered-device table and refuses the remote. Print what the file
         * layer actually holds so "the file is empty" and "the position is
         * wrong" can be told apart. */
        static unsigned rl;
        if (rl < 40) { rl++;
            LOG_INFO(LOG_CORE, "FS: read %s pos=%u len=%u flen=%u data=%s",
                     f->name, fd->pos, len, f->len, f->data ? "yes" : "NULL"); }
    }
    if (fd->pos >= f->len) n = 0;
    else if (fd->pos + n > f->len) n = f->len - fd->pos;   /* short read ok */
    if (n) mem_write_block(buf, f->data + fd->pos, n);
    fd->pos += n;
    return (s32)n;
}

static s32 fs_write(fd_entry *fd, u32 buf, u32 len)
{
    nand_file *f = &s_files[fd->file];
    u32 limit = nand_ensure(fd->file, fd->pos + len);
    if (ios_trace())
        fprintf(stderr, "[fs] write %s pos=%u len=%u limit=%u\n",
                f->name, fd->pos, len, limit);
    u32 n = len;

    if (fd->pos >= limit) n = 0;
    else if (fd->pos + n > limit) n = limit - fd->pos;
    if (n) {
        mem_read_block(buf, f->data + fd->pos, n);
        fd->pos += n;
        if (fd->pos > f->len) f->len = fd->pos;   /* the file grew */
    }
    if (n != len)
        LOG_ERROR(LOG_CORE, "FS: SHORT WRITE %s pos=%u want=%u got=%u limit=%u",
                  f->name, fd->pos, len, n, limit);
    else if (strncmp(f->name, "/title/", 7) == 0) {
        static unsigned wr_tick;
        if ((wr_tick++ % 50) == 0)
            LOG_INFO(LOG_CORE, "FS: write #%u %s pos=%u len=%u",
                     wr_tick, f->name, fd->pos - n, n);
    }
    return (s32)n;
}

static s32 fs_seek(fd_entry *fd, s32 off, u32 whence)
{
    nand_file *f = &s_files[fd->file];
    s64 base = (whence == 0) ? 0 : (whence == 1) ? (s64)fd->pos : (s64)f->len;
    s64 npos = base + off;
    if (npos < 0 || npos > (s64)f->len) return FS_EINVAL;
    fd->pos = (u32)npos;
    return (s32)fd->pos;
}

/* ============================================================ /dev/stm ===== */

enum {
    IOCTL_STM_EVENTHOOK  = 0x1000,
    IOCTL_STM_RELEASE_EH = 0x3002,
};

/* ============================================================== dispatch === */

static s32 do_open(u32 req)
{
    char path[64];
    u32  mode = mem_read32(req + IOS_REQ_ARG1);
    read_guest_string(mem_read32(req + IOS_REQ_ARG0), path, sizeof path);

    dev_kind kind;
    int file = -1;

    if      (!strcmp(path, "/dev/stm/immediate")) kind = DEV_STM_IMM;
    else if (!strcmp(path, "/dev/stm/eventhook")) kind = DEV_STM_EH;
    else if (!strcmp(path, "/dev/fs"))            kind = DEV_FS;
    else if (!strcmp(path, "/dev/di"))            kind = DEV_DI;
    else if (!strcmp(path, "/dev/es"))            kind = DEV_ES;
    else if (!strcmp(path, "/dev/usb/oh1/57e/305")) kind = DEV_BT;
    else if (!strcmp(path, "/dev/usb/oh1"))       kind = DEV_STUB;
    /* The network stack. A title opens these during init and waits on the
     * replies; refusing them leaves its state machine parked. Serving them as
     * stubs (open succeeds, requests report success with zeroed output) lets
     * initialisation complete on a console with no network, which is what we
     * present. */
    else if (!strcmp(path, "/dev/net/kd/time"))   kind = DEV_KD_TIME;
    else if (!strncmp(path, "/dev/net/", 9))      kind = DEV_STUB;
    else if (!strncmp(path, "/dev/", 5)) {
        LOG_WARN(LOG_CORE, "IOS_Open(\"%s\"): unknown device -> ENOENT", path);
        return IPC_ENOENT;
    } else {
        /* Any other absolute path is a NAND file served by FS
         * (Dolphin IOS.cpp OpenDevice, line 685). */
        file = nand_lookup(path);
        if (file < 0 && ios_trace() && strstr(path, "banner.bin")) {
            /* The save flow's last visible act before the error screen: name
             * the whole call chain. The open executes inside the guest's IPC
             * write, so the bound CPU's stack is the caller's. */
            PPCState *cs = timing_bound_cpu();
            if (cs) {
                u32 fp = cs->gpr[1];
                unsigned d2;
                fprintf(stderr, "[who] banner open: pc=%08x lr=%08x\n",
                        (unsigned)cs->pc, (unsigned)cs->lr);
                for (d2 = 0; d2 < 10 && (fp >> 28); d2++) {
                    u32 next = mem_read32(fp), slr = mem_read32(fp + 4);
                    fprintf(stderr, "[who]   sp=%08x lr=%08x\n",
                            (unsigned)fp, (unsigned)slr);
                    if (next <= fp) break;
                    fp = next;
                }
                {
                    /* One-shot dump of the REL save-manager decision code:
                     * it is only in memory at run time. */
                    static int dumped;
                    if (!dumped) {
                        u32 v2;
                        dumped = 1;
                        for (v2 = 0x80528180u; v2 < 0x80528330u; v2 += 4)
                            fprintf(stderr, "[rel] %08x: %08x\n",
                                    (unsigned)v2, (unsigned)mem_read32(v2));
                    }
                }
            }
        }
        if (file < 0) {
            /* Real IOS semantics: opening a file that does not exist FAILS
             * with -106. Auto-creating a zeroed file here handed titles
             * plausible-looking garbage they then trusted -- MKWii read a
             * 1 MiB-of-zeros Mii database as valid and jumped through a null
             * face-loader field ("face_1"). A title that gets ENOENT runs its
             * own recovery: ISFS_CreateFile (ioctl 9, where creation belongs)
             * followed by a write of properly formatted content. */
            LOG_INFO(LOG_CORE, "IOS_Open(\"%s\"): no such NAND file -> -106",
                     path);
            return FS_ENOENT;
        }
        kind = DEV_FS_FILE;
    }

    s32 fd = alloc_fd(kind);
    if (fd >= 0) { s_fds[fd].file = file; s_ios_opens++; }
    LOG_INFO(LOG_CORE, "IOS_Open(\"%s\", %u) -> %d", path, mode, (int)fd);
    return fd;
}

/* Every request we refuse is a place a real console would have said yes. With a
 * known-good emulator to compare against, these are the candidates for whatever
 * is stopping the boot. */
static void note_call(u32 cmd, u32 fdn, int kind, u32 num)
{
    static unsigned n;
    if (n < 40) {
        LOG_DEBUG(LOG_CORE, "IOS call: cmd %u fd %u kind %d num %#x", cmd, fdn, kind, num);
        n++;
    }
}

static void note_refusal(u32 cmd, u32 fdn, int kind, u32 num, s32 result)
{
    static unsigned n;
    if (result >= 0 || n >= 20) return;
    LOG_DEBUG(LOG_CORE, "IOS refused: cmd %u fd %u kind %d num %#x -> %d",
             cmd, fdn, kind, num, (int)result);
    n++;
}

ios_dispatch_status ios_dispatch(u32 req, s32 *result)
{
    u32 cmd = mem_read32(req + IOS_REQ_CMD);
    u32 fdn = mem_read32(req + IOS_REQ_FD);
    fd_entry *fd = (fdn < IOS_MAX_FDS && s_fds[fdn].kind != DEV_NONE)
                 ? &s_fds[fdn] : NULL;

    {   /* Trace the whole conversation for the first few dozen commands.
         * Chasing one failing device at a time keeps arriving at "the command
         * never reached the model" -- which is itself the answer, but only if
         * you can see what DID reach it. */
        static unsigned traced;
        if (traced < 64u) {
            traced++;
            LOG_INFO(LOG_CORE, "IPC[%u]: cmd=%u fd=%u (%s) arg=%08x",
                     traced, cmd, fdn, fd ? "open" : "NO SUCH FD",
                     mem_read32(req + IOS_REQ_ARG0));
        }
    }
    if (cmd == IOS_CMD_OPEN) {
        {   /* An empty path here means the guest's path BUFFER read as empty,
             * which is memory visibility, not a guest decision. Capture the
             * pointer and the raw bytes behind it before answering. */
            u32 pp = mem_read32(req + IOS_REQ_ARG0);
            if (pp && mem_read8(pp) == 0) {
                LOG_WARN(LOG_CORE, "IOS: OPEN with empty path: ptr=%08x "
                         "bytes %02x %02x %02x %02x %02x %02x %02x %02x",
                         pp,
                         mem_read8(pp+0), mem_read8(pp+1), mem_read8(pp+2),
                         mem_read8(pp+3), mem_read8(pp+4), mem_read8(pp+5),
                         mem_read8(pp+6), mem_read8(pp+7));
            }
        }
        *result = do_open(req);
        return IOS_DISPATCH_REPLY;
    }
    if (!fd) {                              /* invalid fd for any other cmd */
        LOG_WARN(LOG_CORE, "IOS: cmd %u on unopened fd %u -> EINVAL", cmd, fdn);
        *result = IPC_EINVAL;
        return IOS_DISPATCH_REPLY;
    }

    switch (cmd) {
    case IOS_CMD_CLOSE:
        if (fd->kind == DEV_STM_EH) s_stm_eventhook_req = 0;
        if (fd->kind == DEV_FS_FILE &&
            strncmp(s_files[fd->file].name, "/title/", 7) == 0)
            LOG_INFO(LOG_CORE, "FS: Close(%s) size=%u",
                     s_files[fd->file].name, s_files[fd->file].len);
        fd->kind = DEV_NONE;
        *result = IPC_SUCCESS;
        return IOS_DISPATCH_REPLY;

    case IOS_CMD_READ: {
        u32 buf = mem_read32(req + IOS_REQ_ARG0);
        u32 len = mem_read32(req + IOS_REQ_ARG1);
        *result = (fd->kind == DEV_FS_FILE) ? fs_read(fd, buf, len)
                                            : IPC_EINVAL;
        return IOS_DISPATCH_REPLY;
    }
    case IOS_CMD_WRITE: {
        u32 buf = mem_read32(req + IOS_REQ_ARG0);
        u32 len = mem_read32(req + IOS_REQ_ARG1);
        *result = (fd->kind == DEV_FS_FILE) ? fs_write(fd, buf, len)
                                            : IPC_EINVAL;
        return IOS_DISPATCH_REPLY;
    }
    case IOS_CMD_SEEK: {
        s32 off    = (s32)mem_read32(req + IOS_REQ_ARG0);
        u32 whence =      mem_read32(req + IOS_REQ_ARG1);
        *result = (fd->kind == DEV_FS_FILE) ? fs_seek(fd, off, whence)
                                            : IPC_EINVAL;
        return IOS_DISPATCH_REPLY;
    }

    case IOS_CMD_IOCTL: {
        int refuse_kind = fd->kind;
        note_call(cmd, fdn, fd->kind, mem_read32(req + IOS_REQ_ARG0));
        u32 num     = mem_read32(req + IOS_REQ_ARG0);
        u32 in      = mem_read32(req + IOS_REQ_ARG1);
        u32 in_len  = mem_read32(req + IOS_REQ_ARG2);
        u32 out     = mem_read32(req + IOS_REQ_ARG3);
        u32 out_len = mem_read32(req + IOS_REQ_ARG4);

        switch (fd->kind) {
        case DEV_DI:
            *result = di_ioctl(num, in, in_len, out, out_len);
            if (num == 0x71 && out_len >= 16 && *result == 1) {
                /* Content spot-check: the game died on an NW4R ResFile
                 * signature panic, and the shortest path to "is the disc
                 * data intact?" is the first bytes of every partition read
                 * as the guest will see them. */
                static unsigned d71;
                if (d71 < 12u) { d71++;
                    LOG_INFO(LOG_CORE, "DI71[%u] out=%08x: "
                             "%02x %02x %02x %02x %02x %02x %02x %02x",
                             d71, out,
                             mem_read8(out+0), mem_read8(out+1),
                             mem_read8(out+2), mem_read8(out+3),
                             mem_read8(out+4), mem_read8(out+5),
                             mem_read8(out+6), mem_read8(out+7));
                }
            }
            {   /* The DVD driver turns a bad result into __DVDShowFatalMessage
                 * and the Wii error screen, with nothing in between to say
                 * which command it was. Name every one of the first few. */
                static unsigned di_logged;
                if (di_logged < 24u) {
                    di_logged++;
                    LOG_INFO(LOG_CORE, "DI: ioctl %#x in=%08x/%x out=%08x/%x"
                             " -> %d", num, in, in_len, out, out_len,
                             (int)*result);
                }
            }
            return IOS_DISPATCH_REPLY;
        case DEV_FS:
        case DEV_FS_FILE:
            *result = fs_ioctl(fd, num, in, in_len, out, out_len);
            return IOS_DISPATCH_REPLY;
        case DEV_STM_IMM:
            if (num == IOCTL_STM_RELEASE_EH) {
                if (!s_stm_eventhook_req) { *result = IPC_ENOENT; }
                else {
                    /* Complete the parked hook with event 0 (STM.cpp:33). */
                    u32 eh = s_stm_eventhook_req;
                    mem_write32(mem_read32(eh + IOS_REQ_ARG3), 0);
                    ios_write_reply(eh, IPC_SUCCESS);
                    ipc_queue_reply(eh);
                    s_stm_eventhook_req = 0;
                    *result = IPC_SUCCESS;
                }
            } else {
                LOG_INFO(LOG_CORE, "STM: ioctl %#x -> 0", num);
                *result = IPC_SUCCESS;      /* hotreset/vidimming/ledmode/... */
            }
            return IOS_DISPATCH_REPLY;
        case DEV_STM_EH:
            if (num == IOCTL_STM_EVENTHOOK) {
                /* Held until a reset/power event; never answer now
                 * (Dolphin STM.cpp:86-97). */
                s_stm_eventhook_req = req;
                return IOS_DISPATCH_PARKED;
            }
            *result = IPC_UNKNOWN;
            return IOS_DISPATCH_REPLY;
        case DEV_BT:
            return IOS_DISPATCH_PARKED;     /* HCI: async forever, WPAD stalls */
        case DEV_KD_TIME:
            *result = kd_time_ioctl(num, in, in_len, out, out_len);
            return IOS_DISPATCH_REPLY;
        case DEV_STUB:
            /* A stub that answers success while writing NOTHING into the out
             * buffers hands the caller uninitialised data it will treat as
             * real. Name every stubbed ioctl so a caller acting on garbage is
             * attributable. */
            LOG_WARN(LOG_CORE, "IOS: STUB ioctl %#x on fd %u (in %x/%x out %x/%x)",
                     num, fdn, in, in_len, out, out_len);
            *result = IPC_SUCCESS;
            return IOS_DISPATCH_REPLY;
        default:
            *result = IPC_EINVAL;
            return IOS_DISPATCH_REPLY;
        }
    }

    case IOS_CMD_IOCTLV: {
        u32 num = mem_read32(req + IOS_REQ_ARG0);
        note_call(cmd, fdn, fd->kind, num);
        u32 nin = mem_read32(req + IOS_REQ_ARG1);
        u32 nio = mem_read32(req + IOS_REQ_ARG2);
        u32 vec = mem_read32(req + IOS_REQ_ARG3);

        switch (fd->kind) {
        case DEV_DI: *result = di_ioctlv(num, nin, nio, vec); break;
        case DEV_ES: *result = es_ioctlv(num, nin, nio, vec); break;
        case DEV_FS:
        case DEV_FS_FILE:
            *result = (num == 4)    ? fs_readdir(nin, nio, vec)
                    : (num == 0x0C) ? fs_getusage(nin, nio, vec)
                    : IPC_SUCCESS;
            break;
        case DEV_BT: {
            int r = ios_bt_ioctlv(req, num, nin, nio, vec);
            if (r < 0)
                return IOS_DISPATCH_PARKED;   /* waiting for an event */
            *result = r;
            break;
        }
        case DEV_KD_TIME:
        case DEV_STUB: *result = IPC_SUCCESS; break;
        default:
            LOG_WARN(LOG_CORE, "IOS: ioctlv %#x on fd %u (kind %d)?",
                     num, fdn, fd->kind);
            *result = IPC_EINVAL;
            break;
        }
        note_refusal(cmd, fdn, fd->kind, num, *result);
        return IOS_DISPATCH_REPLY;
    }

    default:
        /* Dump the block, not just the opcode. A command field the model does
         * not know is almost never a command the guest meant to send: it is a
         * request read from the wrong address, or read before the guest
         * finished writing it, and the only way to tell those apart is to see
         * what the other words say. */
        LOG_WARN(LOG_CORE, "IOS: bad command %u at req %08x: "
                 "%08x %08x %08x %08x %08x %08x %08x %08x",
                 cmd, req,
                 mem_read32(req +  0), mem_read32(req +  4),
                 mem_read32(req +  8), mem_read32(req + 12),
                 mem_read32(req + 16), mem_read32(req + 20),
                 mem_read32(req + 24), mem_read32(req + 28));
        *result = IPC_EINVAL;
        return IOS_DISPATCH_REPLY;
    }
}

/* Completion trio, exactly as IOS writes it (Dolphin IOS.cpp:798-809). */
void ios_write_reply(u32 req, s32 result)
{
    u32 cmd = mem_read32(req + IOS_REQ_CMD);
    mem_write32(req + IOS_REQ_RESULT, (u32)result);
    mem_write32(req + IOS_REQ_FD, cmd);         /* fd := original command */
    mem_write32(req + IOS_REQ_CMD, IOS_CMD_REPLY);
}

unsigned ios_progress_opens(void) { return s_ios_opens; }
unsigned ios_progress_antipiracy(void) { return s_di_probe_hits; }
unsigned ios_progress_disc_reads(void) { return s_di_reads; }

/* For the __ios_Ipc2 HLE: DI replies must NOT be delivered at-issue. The
 * anti-piracy probe's intentional DriveError, arriving before the DVD state
 * machine marked the command outstanding, was processed as a spontaneous
 * drive failure -- the disc-error screen latched on a genuine-disc verdict.
 * DI completions take the device-loop release path (real latency, delivered
 * into the DVD layer's proper sleep), exactly as hardware times them. */
int ios_fd_is_di(u32 fdn)
{
    if (fdn >= IOS_MAX_FDS) return 0;
    return s_fds[fdn].kind == DEV_DI;
}

/* Outstanding-request book-keeping: which IOS calls have been dispatched and
 * never answered. A boot that waits forever is always waiting on one of these,
 * so recording the command and device makes the stall self-describing. */
void ios_note_outstanding(u32 req, int replied)
{
    unsigned i;
    if (replied) {
        for (i = 0; i < s_out_n; i++)
            if (s_out[i].req == req) {
                s_out[i] = s_out[--s_out_n];
                return;
            }
        return;
    }
    if (s_out_n >= IOS_MAX_OUTSTANDING) return;
    s_out[s_out_n].req  = req;
    s_out[s_out_n].cmd  = mem_read32(req + IOS_REQ_CMD);
    s_out[s_out_n].fd   = mem_read32(req + IOS_REQ_FD);
    s_out[s_out_n].arg0 = mem_read32(req + IOS_REQ_ARG0);
    s_out[s_out_n].arg1 = mem_read32(req + IOS_REQ_ARG0 + 4);
    {   u32 f = mem_read32(req + IOS_REQ_FD);
        s_out[s_out_n].kind = f < IOS_MAX_FDS ? (u32)s_fds[f].kind : 99u; }
    s_out_n++;
}

void ios_report_outstanding(void)
{
    unsigned i;
    printf("outstanding IOS requests: %u\n", s_out_n);
    for (i = 0; i < s_out_n; i++) {
        unsigned k = s_out[i].fd < IOS_MAX_FDS ? s_fds[s_out[i].fd].kind : 99;
        printf("   req %08x cmd %u fd %u (kind now %u, was %u) arg0 %#x "
               "arg1 %#x\n", s_out[i].req, s_out[i].cmd, s_out[i].fd, k,
               s_out[i].kind, s_out[i].arg0, s_out[i].arg1);
    }
}

void ios_hle_reset(void)
{
    memset(s_fds, 0, sizeof s_fds);
    s_stm_eventhook_req = 0;
    s_di_partition_open = 0;
    s_di_last_length = 0;
    /* Note: registered NAND files and boot seeding survive a reset on
     * purpose — an IOS reload does not wipe the NAND or the disc. */
}

void ios_hle_init(void)
{
    s_num_files = 0;
    s_es_title_active = 0;
    ios_hle_reset();
}
