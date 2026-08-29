/* dol.h — the GameCube/Wii executable format.
 *
 * A DOL is what a disc's `main.dol` is and what every piece of homebrew ships
 * as: a fixed 256-byte header describing up to 7 text and 11 data sections,
 * each with a file offset, a load address and a size, plus a BSS range and an
 * entry point. There is no relocation and no dynamic linking -- sections are
 * copied to fixed addresses and control jumps to the entry point.
 *
 * That simplicity is why this is the right first loader: it is the format the
 * console's own IPL consumes, so being able to load one is exactly the step
 * between "a CPU that executes instructions" and "a machine that boots a
 * program".
 */
#ifndef DOLPHIN_CORE_DISC_DOL_H
#define DOLPHIN_CORE_DISC_DOL_H

#include "../ppc/gekko.h"

#define DOL_NUM_TEXT 7
#define DOL_NUM_DATA 11
#define DOL_HEADER_SIZE 0x100

typedef struct {
    u32 text_offset[DOL_NUM_TEXT];
    u32 data_offset[DOL_NUM_DATA];
    u32 text_address[DOL_NUM_TEXT];
    u32 data_address[DOL_NUM_DATA];
    u32 text_size[DOL_NUM_TEXT];
    u32 data_size[DOL_NUM_DATA];
    u32 bss_address;
    u32 bss_size;
    u32 entry_point;
} DolHeader;

/* Parse a header from raw bytes (big-endian, as stored). Returns 0 on success.
 * Rejects anything whose sections do not land in real guest memory rather than
 * trusting the file, because a malformed DOL would otherwise scribble outside
 * the guest's address space. */
int dol_parse_header(const void *raw, size_t size, DolHeader *out);

/* Copy every section into guest memory and zero the BSS. Returns 0 on success.
 * `size` is the whole file; sections are bounds-checked against it. */
int dol_load(const void *raw, size_t size, DolHeader *out_header);

/* Set up CPU state the way the console's IPL leaves it for a title, then point
 * execution at the DOL's entry point. */
void dol_setup_boot_state(PPCState *s, const DolHeader *h, int wii_mode);

/* Human-readable summary, for logs and for the loader test. */
void dol_describe(const DolHeader *h, void (*out)(void *, const char *),
                  void *ctx);

#endif /* DOLPHIN_CORE_DISC_DOL_H */
