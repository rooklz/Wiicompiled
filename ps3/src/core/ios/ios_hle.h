/* ios_hle.h — high-level emulation of the IOS IPC interface.
 *
 * The mailbox itself (registers at 0xCD000000, the X1/Y1/Y2/X2 dance) lives in
 * hw/ipc.c; this layer answers the requests the mailbox delivers. The request
 * format, ioctl numbers and expected results are specified in
 * docs/IOS_HLE_SPEC.md, verified against Dolphin's Source/Core/Core/IOS and
 * WiiBrew (Hardware/IPC, IOS).
 */
#ifndef IOS_HLE_H
#define IOS_HLE_H

#include <stdio.h>
#include "../mem/memmap.h"   /* u8/u32/s32/u64, mem_* accessors */

/* ---- The 0x40-byte request block (all fields big-endian u32) -------------- */
/* Only the first 0x20 bytes belong to IOS; 0x20..0x3F are PPC-side state.     */

/* These offsets are Nintendo's `IOSResourceRequest` exactly -- confirmed
 * against rvl/include/iostypes.h in the IOS source (see docs/LEAKS.md), not
 * merely inferred: cmd, status, handle, then a union of the per-command
 * argument blocks. Ioctl is (cmd, inPtr, inLen, outPtr, outLen) and ioctlv is
 * (cmd, readCount, writeCount, vector), which is why ARG0..ARG4 are read
 * differently by the two paths below. */
#define IOS_REQ_CMD     0x00   /* 1..7; IOS writes 8 (reply) here on completion */
#define IOS_REQ_RESULT  0x04   /* s32 result, written by IOS                    */
#define IOS_REQ_FD      0x08   /* fd; IOS overwrites with original cmd on reply */
#define IOS_REQ_ARG0    0x0C
#define IOS_REQ_ARG1    0x10
#define IOS_REQ_ARG2    0x14
#define IOS_REQ_ARG3    0x18
#define IOS_REQ_ARG4    0x1C

enum ios_cmd {
    IOS_CMD_OPEN   = 1,
    IOS_CMD_CLOSE  = 2,
    IOS_CMD_READ   = 3,
    IOS_CMD_WRITE  = 4,
    IOS_CMD_SEEK   = 5,
    IOS_CMD_IOCTL  = 6,
    IOS_CMD_IOCTLV = 7,
    IOS_CMD_REPLY  = 8,
};

/* ioctlv vector entry in guest memory: { u32 paddr; u32 len; } */
#define IOS_IOVEC_SIZE  8

/* ---- Return codes (Dolphin Core/IOS/Device.h) ----------------------------- */

#define IPC_SUCCESS       0
#define IPC_EACCES       (-1)
#define IPC_EEXIST       (-2)
#define IPC_EINVAL       (-4)
#define IPC_EMAX         (-5)
#define IPC_ENOENT       (-6)
#define IPC_UNKNOWN      (-9)
#define FS_EINVAL      (-101)
#define FS_EACCESS     (-102)
#define FS_ENOENT      (-106)
#define ES_EINVAL     (-1017)

/* /dev/di results are NOT the 0-success family (Dolphin Core/IOS/DI/DI.h). */
#define DI_SUCCESS        1
#define DI_DRIVE_ERROR    2
#define DI_COVER_CLOSED   4
#define DI_TIMEDOUT      16
#define DI_SECURITY      32
#define DI_VERIFY        64
#define DI_BADARG       128

/* ---- Dispatch ------------------------------------------------------------- */

typedef enum {
    IOS_DISPATCH_REPLY,     /* *result is valid; complete the request now      */
    IOS_DISPATCH_PARKED,    /* async: no reply now (STM eventhook, bluetooth); */
                            /* the request block address has been retained and */
                            /* will be completed via ipc_queue_reply() later   */
} ios_dispatch_status;

/* Execute the request whose block sits at physical address `req`.
 * Does NOT touch the block's cmd/result/fd fields and does NOT touch the
 * mailbox; the ipc layer owns the reply write-back and Y1 sequencing. */
ios_dispatch_status ios_dispatch(u32 req, s32 *result);

/* Write the completion trio into a request block, exactly as IOS does
 * (Dolphin EnqueueIPCReply): +4 = result, +8 = original cmd, +0 = 8. */
void ios_write_reply(u32 req, s32 result);

void ios_hle_init(void);
void ios_hle_reset(void);

/* ---- Hooks the rest of the emulator provides ------------------------------ */

/* Provided by hw/ipc.c: queue a completed request block for delivery to the
 * PPC (sets ARMMSG/Y1 when the mailbox is free). Used for parked requests. */
void ipc_queue_reply(u32 req);

/* Provided by the disc layer (core/disc). Both return 0 on success.
 * disc_read:      byte offset into the DECRYPTED DATA PARTITION contents
 *                 (DVDLowRead offsets are word offsets in that space).
 * disc_read_raw:  byte offset into the raw disc image
 *                 (disc header for ReadDiskID, DVDLowUnencryptedRead ranges). */
int disc_read(u64 offset, u32 len, void *dst);
int disc_read_raw(u64 offset, u32 len, void *dst);

/* ---- Boot-time seeding (see spec section 3) ------------------------------- */

/* Mark the DATA partition open, as the System Menu/apploader leave it.
 * `part_offset` = absolute byte offset of the partition on the raw disc. */
void ios_di_set_partition(u64 part_offset);

/* Establish the ES title context (RMCE01: 0x00010000524D4345ULL). */
void ios_es_set_title(u64 title_id);

/* Register an in-memory NAND file served through the FS device
 * (e.g. "/shared2/sys/SYSCONF", "/title/00000001/00000002/data/setting.txt").
 * `data` must stay valid; writes are stored back into it (up to len). */
void ios_fs_provision_wc24(void);
void ios_fs_register_file(const char *path, u8 *data, u32 len);

/* Persist the virtual NAND across sessions (one file per path under dir). */
void ios_fs_persist_save(const char *dir);
void ios_fs_persist_load(const char *dir);

/* Introspection for tests: the recorded length of a registered NAND file, and
 * how many entries carry that path. A second entry with the same name is a
 * bug -- reads resolve to whichever comes first, which is how a zero-length
 * SYSCONF once shadowed a correctly registered 16 KB one. */
u32  ios_fs_file_len(const char *path);
unsigned ios_fs_file_count(const char *path);

/* Boot-progress signals for a status display: how many IOS_Open calls have
 * succeeded, and how many disc-auth drive-error queries have been answered
 * (2 = the full Error #001 sequence passed). */
/* The Bluetooth adapter (ios_bt.c). ios_bt_ioctlv returns >=0 to reply with
 * that result now, or -1 when the request has been parked for later. */
int  ios_bt_ioctlv(u32 req, u32 num, u32 nin, u32 nio, u32 vec);
void ios_bt_reset(void);
/* Call periodically so a parked event request is served promptly. */
void ios_bt_update(void);
void ios_bt_set_buttons(u16 core_buttons);
void ios_bt_set_pointer(float x, float y);
unsigned ios_bt_commands(void);
unsigned ios_bt_events_sent(void);
unsigned ios_bt_parked(void);
unsigned ios_bt_channels(void);
unsigned ios_bt_acl_sent(void);
unsigned ios_bt_acl_recv(void);
void ios_report_outstanding(void);
unsigned ios_bt_queued(void);

unsigned ios_progress_opens(void);
unsigned ios_progress_discauth(void);
unsigned ios_progress_disc_reads(void);
/* Record every partition read to `f` as "offset length" lines. */
void ios_di_log_reads(FILE *f);
/* Same stream, delivered as a call rather than a line: the harness names each
 * read through the FST, which is how it knows which scene the game is in. */
void ios_di_set_read_hook(void (*fn)(unsigned long long offset, unsigned len));
/* HID input reports actually handed to L2CAP, and whether the link is up.
 * A probe that presses a button has to know the button was transmitted while
 * the game had a link, or it measures nothing. */
unsigned ios_bt_input_reports(void);
int ios_bt_link_up(void);
/* Gate the emulated remote's connection offer, so a harness can decide WHEN
 * the remote appears rather than only whether. */
void ios_bt_allow_connect(int on);
/* Called at the instant the guest issues an HCI Disconnect. The IPC doorbell
 * is serviced synchronously, so the CPU is still standing in the guest's own
 * IPC-send routine and its stack chain names the code that decided to drop
 * the remote -- which is otherwise invisible. */
void ios_bt_set_disconnect_hook(void (*fn)(void));

#endif /* IOS_HLE_H */
