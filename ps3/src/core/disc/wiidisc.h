/* wiidisc.h — reading a GameCube/Wii disc image.
 *
 * The layer between a disc image on storage and the emulator's loader. It is
 * deliberately format-agnostic about *transport*: every access goes through a
 * read callback, so the same parser serves a raw .iso, a .wbfs, a decompressed
 * .rvz, or sectors streamed off the PS3's own storage. What it knows is the
 * *disc layout*: the header that identifies the game, the partition table, and
 * the filesystem inside a partition.
 *
 * Scope, stated honestly: this reads structure and locates files. It does not
 * decrypt Wii partition data (that needs the console's common key and per-title
 * key, and AES the caller supplies) and it does not boot anything -- the
 * apploader, IOS and the disc drive model are separate, much larger pieces.
 * What it delivers is the step every one of those depends on: turning an image
 * into "this is RMCE01, Mario Kart Wii, its DATA partition is here, and its
 * main executable is that file".
 */
#ifndef DOLPHIN_CORE_DISC_WIIDISC_H
#define DOLPHIN_CORE_DISC_WIIDISC_H

#include "../ppc/gekko.h"

/* How the parser reads bytes from the image. Returns 0 on success. `len` bytes
 * at disc offset `off` are copied to `buf`. The offset is into the *logical*
 * disc (a Wii disc is ~4.7 GB / a dual-layer ~8.5 GB), so it is 64-bit. */
typedef int (*DiscRead)(void *ctx, u64 off, void *buf, u32 len);

typedef enum {
    DISC_UNKNOWN = 0,
    DISC_GAMECUBE,      /* magic 0xC2339F3D at 0x1C */
    DISC_WII            /* magic 0x5D1C9EA3 at 0x18 */
} DiscKind;

typedef struct {
    DiscKind kind;
    char     game_id[7];    /* 6 chars + NUL: e.g. "RMCE01"          */
    u8       disc_number;
    u8       version;
    char     title[65];     /* 64 chars + NUL                        */
} DiscHeader;

/* A Wii disc has up to four partition groups, each listing partitions. The one
 * that matters for booting is the DATA partition (type 0). */
typedef struct {
    u64      offset;        /* start of the partition on the disc     */
    u32      type;          /* 0 = DATA, 1 = UPDATE, 2 = CHANNEL...   */
} WiiPartition;

#define WIIDISC_MAX_PARTITIONS 16

typedef struct {
    unsigned     count;
    WiiPartition part[WIIDISC_MAX_PARTITIONS];
    int          data_index;    /* index of the DATA partition, or -1 */
} WiiPartitionTable;

/* Parse the 0x80-byte (or larger) disc header. Returns 0 on success. `raw` must
 * hold at least 0x80 bytes. */
int wiidisc_parse_header(const void *raw, u32 size, DiscHeader *out);

/* Read and identify a disc through the callback. Returns 0 on success. */
int wiidisc_read_header(DiscRead read, void *ctx, DiscHeader *out);

/* Read the Wii partition table (at disc offset 0x40000). Returns 0 on success;
 * fails cleanly on a GameCube disc, which has no partitions. */
int wiidisc_read_partitions(DiscRead read, void *ctx, WiiPartitionTable *out);

/* ------------------------------------------------------------------ */
/* Filesystem (FST)                                                     */
/*                                                                      */
/* Inside a partition, boot.bin gives the offsets of the main DOL and the FST;   */
/* the FST is a flat array of 12-byte entries (files and directories) followed   */
/* by a string table. Offsets in a Wii FST are in 4-byte units; in a GameCube    */
/* FST they are byte offsets -- a distinction that silently corrupts every file  */
/* location if missed.                                                           */
/* ------------------------------------------------------------------ */

typedef struct {
    int  is_dir;
    char name[256];
    u32  offset;        /* file: data offset within the partition (bytes) */
    u32  size;          /* file: byte length; dir: index of first sibling */
} FstEntry;

typedef struct {
    const u8 *fst;      /* the raw FST bytes (caller-owned)          */
    u32       fst_size;
    u32       count;    /* number of entries                        */
    const char *strings;/* the string table (points inside fst)     */
    int       wii;      /* 1 = Wii (4-byte offset units), 0 = GC    */
    u32       dol_offset;
    u32       fst_offset;
} DiscFst;

/* Parse boot.bin (0x440 bytes) for the DOL and FST offsets. */
int wiidisc_parse_boot(const void *boot, u32 size, int wii, DiscFst *out);

/* Attach a raw FST buffer already read from the disc. */
int wiidisc_set_fst(DiscFst *fst, const u8 *raw, u32 size, int wii);

/* Decode entry `i` (0-based). Returns 0 on success. */
int wiidisc_fst_entry(const DiscFst *fst, u32 i, FstEntry *out);

/* Find a file by exact path ("/sound/foo.brsar"), case-sensitive. Returns the
 * entry index, or -1 if not found. Directories are walked, so this resolves a
 * full path rather than only a top-level name. */
int wiidisc_find(const DiscFst *fst, const char *path, FstEntry *out);

#endif
