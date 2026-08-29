/* disc_image.h — byte access to a mounted Wii disc (see disc_image.c). */
#ifndef DOLPHIN_CORE_DISC_DISC_IMAGE_H
#define DOLPHIN_CORE_DISC_DISC_IMAGE_H

#include "../ppc/gekko.h"

/* Mount a decrypted DATA-partition image from a flat file. Returns 0 on
 * success. Offsets passed to disc_image_read are relative to the start of the
 * partition's decrypted data (offset 0 = boot.bin). */
int  disc_image_open(const char *path);
void disc_image_close(void);
int  disc_image_mounted(void);
u64  disc_image_size(void);

/* Read `len` bytes at `partition_offset` into `dst`. Reads past the end are
 * zero-filled. Returns 0 on success, -1 if nothing is mounted. */
int  disc_image_read(u64 partition_offset, void *dst, u32 len);

/* Mount a disc *slice* held in memory -- the ranges a boot reads, produced by
 * tools/mkdiscslice.py. Used where a full partition cannot be carried. */
int  disc_slice_mount(const void *data, u32 len);
int  disc_slice_open(const char *path);

#endif
