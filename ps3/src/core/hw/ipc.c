/* ipc.c — the Wii IPC block, a minimal IOS, and the reply interrupt.
 *
 * A Wii title asks IOS -- the OS on the Starlet ARM core -- for everything real
 * over a mailbox: write a request's address to IPC_PPCMSG, set X1 in
 * IPC_PPCCTRL, and wait. IOS acknowledges (Y2), runs the command, and posts a
 * reply (Y1); each of those raises the Hollywood IPC interrupt, which vectors
 * the PPC to its handler, which reads IPC_ARMMSG and wakes whatever was
 * blocked. Without that interrupt the game polls a completion flag its handler
 * never gets to set, and spins forever -- which is exactly the wall a stub that
 * only flips the control bits hits.
 *
 * Register layout, control-bit positions, cause value, and the "interrupt on
 * Y1/Y2 written while IY1/IY2 set" rule are taken from Dolphin's
 * Source/Core/Core/HW/WII_IPC.cpp / .h. The device set is still a stub that
 * names the request and returns success; making those answers real (backed by
 * the disc reader) is the next layer.
 */
#include "hardware.h"
#include "../mem/memmap.h"
#include "../ppc/gekko.h"
#include "../ios/ios_hle.h"
#include "../../common/log.h"

#include <string.h>
#include <sys/time.h>

/* Registers, offsets from HOLLYWOOD_CACHED (0xCD000000). */
#define IPC_PPCMSG     0x000
#define IPC_PPCCTRL    0x004
#define IPC_ARMMSG     0x008
#define IPC_PPCIRQFLAG 0x030
#define IPC_PPCIRQMASK 0x034

/* PPCCTRL bit positions, from WII_IPC.h CtrlRegister::ppc(). */
#define C_X1  0x01
#define C_Y2  0x02
#define C_Y1  0x04
#define C_X2  0x08
#define C_IY1 0x10
#define C_IY2 0x20

#define INT_CAUSE_IPC_BROADWAY 0x40000000u

static u32 s_ppcmsg, s_armmsg;
static unsigned s_dispatched, s_replied;
unsigned ipc_stat_dispatched(void) { return s_dispatched; }
unsigned ipc_stat_replied(void) { return s_replied; }
static u32 s_irq_flags, s_irq_masks;
static int s_X1, s_Y1, s_Y2, s_X2, s_IY1, s_IY2;

/* Replies waiting to be handed to the PPC.
 *
 * The hardware shows exactly ONE reply at a time (ARMMSG + Y1), so replies must
 * queue: a device completing a previously parked request (a Bluetooth HCI
 * event, an STM eventhook) can land while the reply to the command currently
 * being dispatched is still outstanding. Overwriting ARMMSG in that window
 * loses one of them -- and losing a reply hangs whatever was waiting on it,
 * which is exactly how the controller stack stalled mid-firmware-upload. */
#define IPC_REPLY_QUEUE 16

typedef struct {
    u32 req;
    s32 result;
    int write_result;    /* 1: we still owe the block its result trio */
} IpcReply;

static IpcReply s_replies[IPC_REPLY_QUEUE];
static unsigned s_reply_head, s_reply_count;
volatile unsigned g_ipc_backlog;        /* mirrors s_reply_count for cheap
                                         * cross-file gating of the drain */
volatile unsigned g_ipc_pushed;

#include <stdio.h>
#include <stdlib.h>
static int ipc_trace(void)
{
    static int t = -1;
    if (t < 0) t = getenv("DSP_TRACE") != NULL;
    return t;
}

static void ipc_push_reply(u32 req, s32 result, int write_result)
{
    IpcReply *r;
    if (ipc_trace())
        fprintf(stderr, "[ipc] push req=%08x res=%d\n", (unsigned)req,
                (int)result);
    if (s_reply_count >= IPC_REPLY_QUEUE)
        return;                     /* cannot happen with IOS's fd limit */
    r = &s_replies[(s_reply_head + s_reply_count) % IPC_REPLY_QUEUE];
    r->req = req;
    r->result = result;
    r->write_result = write_result;
    s_reply_count++;
    g_ipc_backlog = s_reply_count;
    g_ipc_pushed++;
}

/* Raise or lower the Broadway IPC interrupt from the current bits, then push
 * the aggregate line to the processor interface. */
static void ipc_update_irq(void)
{
    /* One request produces TWO interrupt-worthy events: the acknowledge (Y2)
     * and, later, the reply (Y1). Both assert the same PI cause line, so if the
     * line is already high from the first the second raises no edge at all.
     *
     * That matters to the port, which delivers the guest's handler itself
     * rather than having a CPU take an interrupt: counting PI assertions
     * coalesced the ack and the reply into one delivery, and the reply was
     * never processed. Count the LOGICAL events here, where they are
     * distinguishable, and let the port deliver one handler run per event. */
    static int last_state = -1;
    int state = (s_Y1 ? 1 : 0) | (s_Y2 ? 2 : 0);

    if ((s_Y1 && s_IY1) || (s_Y2 && s_IY2))
        s_irq_flags |= INT_CAUSE_IPC_BROADWAY;
    if (state != last_state && (s_irq_flags & s_irq_masks))
        pi_note_event();
    last_state = state;
    {   /* HOST MODE: the __ios_Ipc2 layer is answered host-side and replies
         * are consumed by the scheduler drain -- the guest IPC interrupt
         * handler must never run (it drains a staging ring the HLE never
         * writes; measured: 54 junk submits, "NO SUCH FD", double-handled
         * requests). Keep every register readable; just never assert the
         * PI cause. */
        extern int ipc_host_mode_flag;
        pi_set_interrupt(PI_INT_IPC,
                         (!ipc_host_mode_flag && (s_irq_flags & s_irq_masks)) ? 1 : 0);
    }
}

int ipc_host_mode_flag;
void ipc_host_mode(int on) { ipc_host_mode_flag = on; }

/* Hand the next queued reply to the PPC, if it is not already looking at one. */
static void ipc_deliver_next(void)
{
    IpcReply *r;
    if (s_Y1 || s_reply_count == 0)
        return;
    /* IOS REPLY LATENCY. Real IOS takes tens to hundreds of microseconds per
     * round trip. Releasing the next queued reply the instant the guest acks
     * the last one created a self-feeding cycle at interrupt level: the IPC
     * handler drains a reply, the BT stack immediately submits the next HCI
     * command, the model answers instantly, Y1 re-asserts while the handler
     * is still in its service loop -- and the handler never exits. Measured
     * on console as a total freeze with the where-ring pumping 10/11 forever.
     * A 100 us floor between replies is faster than real IOS ever was and
     * lets the handler's drain loop actually finish. */
    {   /* HOST time, deliberately. The floor models real IOS round-trip
         * latency, a wall-clock property. The guest-derived timebase FREEZES
         * whenever the guest stops dispatching -- and the sync-IPC wait is
         * exactly such a stop (a call-free poll), so a guest-time floor makes
         * the release wait for the guest while the guest waits for the
         * release: the fiber build's boot froze at DVDLowInit's first DI
         * ioctl with the reply queued and the line never rising. */
        static u64 s_last_reply_us;
        struct timeval tv;
        u64 now_us;
        gettimeofday(&tv, NULL);
        now_us = (u64)tv.tv_sec * 1000000ull + (u64)tv.tv_usec;
        if (now_us - s_last_reply_us < 100u)
            return;
        s_last_reply_us = now_us;
    }
    r = &s_replies[s_reply_head];
    if (r->write_result)
        ios_write_reply(r->req, r->result);
    s_armmsg = r->req;
    if (ipc_trace())
        fprintf(stderr, "[ipc] deliver req=%08x\n", (unsigned)r->req);
    s_reply_head = (s_reply_head + 1) % IPC_REPLY_QUEUE;
    s_reply_count--;
    g_ipc_backlog = s_reply_count;
    s_Y1 = 1;
    {   static unsigned n;
        if (n < 14u) {
            n++;
            LOG_WARN(LOG_CORE, "IPCREL[%u] req=%08x IY1=%d masks=%08x",
                     n, (unsigned)s_armmsg, s_IY1, (unsigned)s_irq_masks);
        }
    }
}

static u32 ipc_ppcctrl(void)
{
    return (s_IY2 << 5) | (s_IY1 << 4) | (s_X2 << 3) |
           (s_Y1  << 2) | (s_Y2  << 1) | s_X1;
}

static u32 ipc_read(u32 addr, unsigned size, void *ctx)
{
    (void)size; (void)ctx;
    /* The sync-IPC ack wait polls PPCCTRL via inlined MMIO reads -- the ONE
     * hook that fires inside that otherwise call-free spin. Deliver the
     * pending interrupt here (guest thread + EE + non-reentrant guards live
     * inside), so the handler the poll depends on actually runs. The guest
     * then reads post-interrupt state, exactly as hardware interleaves it. */
    {   extern void wc_pump_from_mmio(void);
        wc_pump_from_mmio();
    }
    switch (addr - HOLLYWOOD_CACHED) {
    case IPC_PPCMSG:     return s_ppcmsg;
    case IPC_PPCCTRL:    return ipc_ppcctrl();
    case IPC_ARMMSG:     return s_armmsg;
    case IPC_PPCIRQFLAG: return s_irq_flags;
    case IPC_PPCIRQMASK: return s_irq_masks;
    default:             return 0;
    }
}

/* Counts guest writes to the IPC registers. The port uses it to tell "the
 * interrupt line is still high because there is more to do" from "the line is
 * still high because the guest has not acted yet" -- delivering again in the
 * second case just runs the handler on nothing, repeatedly. */
static volatile unsigned s_guest_reg_writes;
unsigned ipc_guest_activity(void) { return s_guest_reg_writes; }

extern void wc_where(unsigned id);

static void ipc_write(u32 addr, u32 value, unsigned size, void *ctx)
{
    (void)ctx;
    wc_where(10);
    s_guest_reg_writes++;
    {   /* Every register write, with its access SIZE.
         *
         * The port's request stream showed four consecutive dispatches reading
         * the same stale block, which means some writes to PPCMSG are not
         * reaching this function while the neighbouring ones are. Whether the
         * write arrives at all, and at what width, is the thing to see. */
        static unsigned n;
        if (n < 48u) {
            n++;
            LOG_INFO(LOG_CORE, "IPCREG[%u]: +%02x = %08x (size %u)",
                     n, (unsigned)(addr - HOLLYWOOD_CACHED), value, size);
        }
    }
    switch (addr - HOLLYWOOD_CACHED) {
    case IPC_PPCMSG:
        s_ppcmsg = value;
        break;

    case IPC_PPCCTRL:
        /* Y1/Y2 are cleared by writing 1 (the PPC acknowledging the reply/ack).
         * IY1/IY2 are plain enable bits. X1 launches a command. */
        if (value & C_Y1) s_Y1 = 0;
        if (value & C_Y2) s_Y2 = 0;
        s_IY1 = (value & C_IY1) ? 1 : 0;
        s_IY2 = (value & C_IY2) ? 1 : 0;

        if (value & C_X1) {
            /* Run the command now (IOS is instantaneous to the PPC here) and
             * acknowledge (Y2). The reply (Y1) follows once the PPC has seen the
             * ack -- two interrupts, as on hardware. A parked request (async STM
             * eventhook, bluetooth) gets the ack but no reply until its device
             * calls ipc_queue_reply(). */
            s32 result = 0;
            ios_dispatch_status st;
            {   /* The block exactly as the guest handed it over. Comparing the
                 * same request between emulator and port is what separates a
                 * guest that built it wrong from a model that read it wrong. */
                static unsigned nsub;
                if (nsub < 24u) {
                    nsub++;
                    /* The guest's own IPC ring lives at 0x80341240: a
                     * request count and a 4-bit slot index. If the guest
                     * submits a slot it never filled, these are where it went
                     * wrong. */
                    LOG_INFO(LOG_CORE, "SUBMIT[%u] ring %08x/%08x/%08x",
                             nsub, mem_read32(0x80341240u),
                             mem_read32(0x80341244u), mem_read32(0x80341248u));
                    LOG_INFO(LOG_CORE, "SUBMIT[%u] %08x: %08x %08x %08x %08x "
                             "%08x %08x %08x %08x", nsub, s_ppcmsg,
                             mem_read32(s_ppcmsg +  0), mem_read32(s_ppcmsg +  4),
                             mem_read32(s_ppcmsg +  8), mem_read32(s_ppcmsg + 12),
                             mem_read32(s_ppcmsg + 16), mem_read32(s_ppcmsg + 20),
                             mem_read32(s_ppcmsg + 24), mem_read32(s_ppcmsg + 28));
                }
            }
            st = ios_dispatch(s_ppcmsg, &result);
            s_dispatched++;
            {   /* Remember requests we do not answer, so a stalled boot can be
                 * attributed to the exact IOS call still outstanding. */
                extern void ios_note_outstanding(u32 req, int replied);
                ios_note_outstanding(s_ppcmsg, st == IOS_DISPATCH_REPLY);
            }
            if (st == IOS_DISPATCH_REPLY) {
                ipc_push_reply(s_ppcmsg, result, 1);
                s_replied++;
            }
            wc_where(11);
            s_Y2 = 1;                 /* ack */
            /* Do NOT hand the reply back in the same write that submitted the
             * command. Real IOS takes microseconds to answer, and the caller
             * arms its wait *after* the submit returns; replying instantly can
             * land the completion before anything is waiting for it, and the
             * wakeup is lost. Replies are released from ipc_update(). */
        } else {
            extern int ipc_host_mode_flag;
            if (!ipc_host_mode_flag)
                ipc_deliver_next();
        }
        ipc_update_irq();
        break;

    case IPC_PPCIRQFLAG:
        s_irq_flags &= ~value;        /* write 1 to clear */
        /* Re-derive the line level: if a reply/ack (Y1/Y2) is still pending, the
         * flag re-asserts immediately -- the interrupt is a level, not an edge.
         * Without this, acknowledging the ack interrupt also swallows the reply
         * interrupt raised in the same handler, and the IPC never completes. */
        ipc_update_irq();
        break;

    case IPC_PPCIRQMASK:
        s_irq_masks = value;
        ipc_update_irq();
        break;

    default:
        break;
    }
    {   /* Fiber build: the write that arms the interrupt may be the last
         * instruction before a call-free RAM poll; deliver here if the line
         * is (now) high. Guarded inside: guest thread only, non-reentrant. */
        extern void wc_pump_from_mmio(void);
        wc_pump_from_mmio();
    }
}

/* A parked (async) request has completed: its device has already written the
 * result into the block via ios_write_reply. Deliver the reply the same way a
 * synchronous one is delivered -- point ARMMSG at it and raise Y1. */
void ipc_queue_reply(u32 req)
{
    /* The device has already written the block's result via ios_write_reply. */
    { extern void ios_note_outstanding(u32 req, int replied);
      ios_note_outstanding(req, 1); }
    ipc_push_reply(req, 0, 0);
    {   extern int ipc_host_mode_flag;
        if (!ipc_host_mode_flag)
            ipc_deliver_next();
    }
    ipc_update_irq();
}

/* Release queued replies. Called from the emulation loop so a reply reaches the
 * guest a moment after the command that produced it, as on hardware. */
void ipc_update(void)
{
    /* Host mode: replies are consumed by the scheduler drain, not the
     * ARMMSG/Y1 interrupt protocol -- releasing them here moved each reply
     * into register state no one reads (measured: drain popped nothing, BT
     * retried HCI_Reset forever at 3M calls/s). */
    extern int ipc_host_mode_flag;
    if (ipc_host_mode_flag) return;
    ipc_deliver_next();
    ipc_update_irq();
}

/* Host-side IOS entry for the __ios_Ipc2 HLE: dispatch the request the same
 * way an X1 submit would, but synchronously, with the block's result written
 * before return. Takes the guest VIRTUAL block address; the mailbox carries
 * physical, so convert as the inlined submit did (virt - 0x80000000). */
s32 ios_dispatch_hle_direct(u32 req_virt, s32 *result)
{
    u32 phys = req_virt - 0x80000000u;
    ios_dispatch_status st;
    dev_lock();
    st = ios_dispatch(phys, result);
    s_dispatched++;
    {   extern void ios_note_outstanding(u32 req, int replied);
        ios_note_outstanding(phys, st == IOS_DISPATCH_REPLY);
    }
    if (st == IOS_DISPATCH_REPLY) {
        ios_write_reply(phys, *result);
        s_replied++;
    }
    dev_unlock();
    return st == IOS_DISPATCH_REPLY ? 1 : 0;
}

/* For a synchronous wait on a parked request: consume its reply directly if
 * a device has queued it, bypassing the interrupt path entirely. */
int ios_take_reply_for(u32 req_virt)
{
    u32 phys = req_virt - 0x80000000u;
    unsigned i;
    int found = 0;
    dev_lock();
    for (i = 0; i < s_reply_count; i++) {
        unsigned ix = (s_reply_head + i) % IPC_REPLY_QUEUE;
        if (s_replies[ix].req == phys) {
            if (s_replies[ix].write_result)
                ios_write_reply(phys, s_replies[ix].result);
            /* remove by shifting the tail of the window */
            for (; i + 1 < s_reply_count; i++) {
                unsigned a = (s_reply_head + i) % IPC_REPLY_QUEUE;
                unsigned b = (s_reply_head + i + 1) % IPC_REPLY_QUEUE;
                s_replies[a] = s_replies[b];
            }
            s_reply_count--;
            g_ipc_backlog = s_reply_count;
            found = 1;
            break;
        }
    }
    dev_unlock();
    return found;
}

/* Queue a completed request's reply for guest-callback delivery, from the
 * HLE (already device-written). FRONT of the queue: this is the submitting
 * transfer's own ACK, and hardware completes it BEFORE any event the
 * command generated -- the device emits events during dispatch, so plain
 * FIFO order held [event, ack] and the BT stack processed the event while
 * its pipe was still busy: the next command was never sent (0x1009/0x1003
 * timeouts). */
void ipc_queue_reply_virt(u32 req_virt)
{
    dev_lock();
    if (s_reply_count < IPC_REPLY_QUEUE) {
        s_reply_head = (s_reply_head + IPC_REPLY_QUEUE - 1) % IPC_REPLY_QUEUE;
        s_replies[s_reply_head].req = req_virt - 0x80000000u;
        s_replies[s_reply_head].result = 0;
        s_replies[s_reply_head].write_result = 0;
        s_reply_count++;
        g_ipc_backlog = s_reply_count;
        g_ipc_pushed++;
    }
    dev_unlock();
}

/* Racy-read gate for the boundary drain: the truth, not a mirror (the
 * mirror desynced -- 3.1M drain attempts against an empty queue while the
 * gate read 1). A stale read here only delays or wastes one drain call. */
unsigned ipc_backlog_probe(void) { return s_reply_count; }

/* Pop any queued reply for host-side callback completion (fiber build).
 * Returns the request's VIRTUAL address, or 0 when the queue is empty. */
u32 ipc_pop_reply_virt(void)
{
    u32 req = 0;
    dev_lock();
    if (s_reply_count) {
        IpcReply *r = &s_replies[s_reply_head];
        if (r->write_result)
            ios_write_reply(r->req, r->result);
        req = r->req + 0x80000000u;
        s_reply_head = (s_reply_head + 1) % IPC_REPLY_QUEUE;
        s_reply_count--;
    }
    dev_unlock();
    return req;
}

void ipc_reset(void)
{
    s_ppcmsg = s_armmsg = 0;
    s_irq_flags = s_irq_masks = 0;
    s_X1 = s_Y1 = s_Y2 = s_X2 = s_IY1 = s_IY2 = 0;
    s_reply_head = s_reply_count = 0;
    s_dispatched = s_replied = 0;
    ios_hle_reset();
}

void ipc_init(void)
{
    ipc_reset();
    ios_hle_init();
    /* Cover the mailbox registers and the Hollywood IRQ flag/mask pair. */
    mmio_register(HOLLYWOOD_CACHED + IPC_PPCMSG, 0x40, ipc_read, ipc_write,
                  NULL, "IPC");
}
