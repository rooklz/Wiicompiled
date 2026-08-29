/* ios_disc_glue.c — connect IOS's /dev/di to the mounted disc image.
 *
 * ios_hle.c calls disc_read()/disc_read_raw() and does not care where the bytes
 * come from; this binds them to disc_image, which serves the decrypted DATA
 * partition. Raw reads (disk ID, a couple of unencrypted ranges) are answered
 * from the disc header the mount knows, since the flat image starts at the
 * partition's decrypted data (offset 0 = the copy of the disc header).
 */
#include "ios_hle.h"
#include "../disc/disc_image.h"

#include <string.h>

/* Decrypted DATA-partition space: offset 0 is boot.bin. */
int disc_read(u64 offset, u32 len, void *dst)
{
    return disc_image_read(offset, dst, len);
}

/* Raw disc space. The only raw reads a boot makes are the 0x20-byte disk ID at
 * offset 0 (identical to the partition's first bytes) and a couple of fixed
 * ranges the drive returns as zeros. Serve the ID from the image head and zero
 * the rest, which is what those ranges hold on a retail disc anyway. */
int disc_read_raw(u64 offset, u32 len, void *dst)
{
    if (offset < 0x20)
        return disc_image_read(offset, dst, len);
    memset(dst, 0, len);
    return 0;
}
