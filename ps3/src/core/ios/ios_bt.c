/* ios_bt.c — the Wii's Bluetooth adapter (/dev/usb/oh1/57e/305), HCI only.
 *
 * A title's controller stack (BTA_Init, inside IOS's widcomm port) will not
 * finish initialising until the adapter answers its HCI commands. Until it
 * does, the game sits in its boot state machine forever -- which is exactly
 * where Mario Kart Wii stopped: issuing HCI_Reset over and over into a device
 * that never replied.
 *
 * Scope, deliberately: this emulates the *adapter*, not any Wiimote. That is
 * enough for stack setup to complete and the game to proceed -- Dolphin boots
 * every Wii game to its title screen with no Wiimote connected, because a
 * remote's connection is device-initiated and only happens after the stack
 * enables page scan (BTEmu.cpp RemoteConnect / IsPageScanEnabled). Adding a
 * controller later means L2CAP + HID on top of this, not changes to it.
 *
 * Wire format notes that matter on this host: HCI is little-endian on the wire
 * while the guest is big-endian PowerPC, so every multi-byte field is emitted
 * byte-by-byte rather than stored as a native struct.
 *
 * Values and behaviour mirror Dolphin's Source/Core/Core/IOS/USB/Bluetooth/
 * BTEmu.cpp (ExecuteHCICommandMessage, SendEventCommandComplete, AddEventToQueue)
 * and hci.h.
 */
#include <stdlib.h>
#include "ios_hle.h"
#include "../../common/log.h"
#include "../core_timing.h"

#include <stdio.h>
static int bt_trace(void)
{
    static int t = -1;
    if (t < 0) t = getenv("DSP_TRACE") != NULL;
    return t;
}

#include <string.h>

/* USB request types (Dolphin IOS/USB/USBV0.h). */
#define USBV0_CTRLMSG 0
#define USBV0_BLKMSG  1
#define USBV0_INTRMSG 2

/* Endpoints (BTBase.h). */
#define EP_HCI_EVENT   0x81
#define EP_ACL_IN      0x82

/* Event codes (hci.h). */
#define EVT_COMMAND_COMPLETE  0x0E
#define EVT_RETURN_LINK_KEYS  0x15
#define EVT_COMMAND_STATUS    0x0F

/* Commands the stack issues while coming up (hci.h). */
#define CMD_RESET                   0x0C03
#define CMD_SET_EVENT_FILTER        0x0C05
#define CMD_WRITE_PIN_TYPE          0x0C0A
#define CMD_READ_STORED_LINK_KEY    0x0C0D
#define CMD_WRITE_STORED_LINK_KEY   0x0C11
#define CMD_DELETE_STORED_LINK_KEY  0x0C12
#define CMD_WRITE_LOCAL_NAME        0x0C13
#define CMD_WRITE_PAGE_TIMEOUT      0x0C18
#define CMD_WRITE_SCAN_ENABLE       0x0C1A
#define CMD_WRITE_UNIT_CLASS        0x0C24
#define CMD_HOST_BUFFER_SIZE        0x0C33
#define CMD_WRITE_LINK_SUP_TIMEOUT  0x0C37
#define CMD_WRITE_LINK_POLICY       0x080D
#define CMD_VENDOR_FC4C             0xFC4C
#define CMD_VENDOR_FC4F             0xFC4F
#define CMD_WRITE_INQUIRY_SCAN_TYPE 0x0C43
#define CMD_WRITE_INQUIRY_MODE      0x0C45
#define CMD_WRITE_PAGE_SCAN_TYPE    0x0C47
#define CMD_READ_LOCAL_VER          0x1001
#define CMD_READ_LOCAL_FEATURES     0x1003
#define CMD_READ_BUFFER_SIZE        0x1005
#define CMD_READ_BDADDR             0x1009

/* Connection phase: issued once a remote asks to connect. */
#define CMD_DISCONNECT              0x0406
#define CMD_CREATE_CON              0x0405
#define CMD_ACCEPT_CON              0x0409
#define CMD_REJECT_CON              0x040A
#define CMD_LINK_KEY_REP            0x040B
#define CMD_LINK_KEY_NEG_REP        0x040C
#define CMD_CHANGE_CON_PACKET_TYPE  0x040F
#define CMD_AUTH_REQ                0x0411
#define CMD_REMOTE_NAME_REQ         0x0419
#define CMD_READ_REMOTE_FEATURES    0x041B
#define CMD_READ_CLOCK_OFFSET       0x041F
#define CMD_SNIFF_MODE              0x0803
#define CMD_WRITE_LINK_POLICY       0x080D

#define EVT_CON_COMPL               0x03
#define EVT_CON_REQ                 0x04
#define EVT_DISCON_COMPL            0x05
#define EVT_LINK_KEY_REQUEST         0x17
#define EVT_AUTH_COMPL              0x06
#define EVT_ENCRYPT_CHANGE          0x08
#define EVT_REMOTE_NAME_COMPL       0x07
#define EVT_READ_REMOTE_FEAT_COMPL  0x0B
#define EVT_ROLE_CHANGE             0x12
#define EVT_MODE_CHANGE             0x14
#define EVT_LINK_KEY_NOTIFICATION   0x18
#define EVT_READ_CLOCK_OFF_COMPL    0x1C
#define EVT_CON_PKT_TYPE_CHANGED    0x1D

#define HCI_LINK_ACL                0x01
#define PAGE_SCAN_ENABLE            0x02

/* The remote we present: Dolphin's first emulated Wii Remote, whose address and
 * device class the stack matches against SYSCONF's BT.DINF. */
static const u8 k_wm_bdaddr[6] = { 0x11, 0x02, 0x19, 0x79, 0x00, 0x00 };
static const u8 k_wm_class[3]  = { 0x00, 0x04, 0x48 };
static const char k_wm_name[]  = "Nintendo RVL-CNT-01";
#define WM_CON_HANDLE 0x0100        /* 0x100 + bdaddr[5] (Dolphin) */

/* Controller identity and limits, as Dolphin's emulated adapter reports them.
 * num_acl must stay <= 10 or the widcomm stack underflows (BTEmu.cpp:1806). */
#define EVT_NUM_COMPL_PKTS 0x13
#define ACL_PKT_SIZE 339
#define ACL_PKT_NUM  10
#define SCO_PKT_SIZE 64
#define SCO_PKT_NUM  0

static const u8 k_bdaddr[6]   = { 0x11, 0x02, 0x19, 0x79, 0x00, 0xff };
/* The ADAPTER's own LMP features (Read_Local_Supported_Features), which is a
 * full-featured Broadcom part. */
static const u8 k_features[8] = { 0xFF, 0xFF, 0x8D, 0xFE, 0x9B, 0xF9, 0x00, 0x80 };
/* The REMOTE's features, which are a very different and much smaller set --
 * no multi-slot packets, no hold, no park, no SCO. We were reporting the
 * adapter's set for the remote as well, i.e. telling the stack the remote
 * supports link modes it does not have. Dolphin's WiimoteDevice.cpp:66. */
static const u8 k_wm_features[8] = { 0xBC, 0x02, 0x04, 0x38, 0x08, 0x00, 0x00, 0x00 };
#define WM_LMP_VERSION     0x02
#define WM_LMP_SUBVERSION  0x0229
#define WM_MANUFACTURER    0x000F

/* One ioctlv vector: { u32 physical address; u32 length }. */
static void read_iovec(u32 vec_base, u32 i, u32 *paddr, u32 *len)
{
    *paddr = mem_read32(vec_base + i * IOS_IOVEC_SIZE);
    *len   = mem_read32(vec_base + i * IOS_IOVEC_SIZE + 4);
}

/* ------------------------------------------------------------------ */
/* Event queue and the parked interrupt-IN request                      */
/* ------------------------------------------------------------------ */

#define BT_MAX_EVENTS 16
#define BT_EVENT_MAX  264
/* Remotes the adapter holds keys for: 4 Wii Remotes + Balance Board, the
 * same five slots the real adapter reports (Dolphin BTEmu MAX_BBMOTES). */
/* How many stored keys the adapter reports. Read from a file on the console so
 * it can be changed with an FTP upload and a restart, no rebuild.
 *
 * This is not cosmetic. The title walks its registered-device table comparing
 * 6-byte BD_ADDRs (guest 0x801cf648: memcmp of 6 bytes, then "beq error" when
 * nothing matched) and takes an error path straight to HCI Disconnect when our
 * remote is not in it -- which is exactly the +16 ms disconnect hardware
 * shows where qemu sends Authentication_Requested at +15 ms. The title also
 * DELETES every key it reads back from us, so how many we report changes what
 * survives in that table. */
#define BT_NUM_STORED_KEYS_MAX 5
static unsigned bt_num_stored_keys(void)
{
    static int n = -1;
    if (n < 0) {
        FILE *f = fopen("/dev_hdd0/tmp/wiicompiled-keys.txt", "r");
        const char *e;
        n = 5;
        if (f) { if (fscanf(f, "%d", &n) != 1) n = 5; fclose(f); }
        else if ((e = getenv("BT_NUM_KEYS")) && *e) n = (int)strtol(e, NULL, 0);
        if (n < 0) n = 0;
        if (n > BT_NUM_STORED_KEYS_MAX) n = BT_NUM_STORED_KEYS_MAX;
        LOG_INFO(LOG_CORE, "BT: reporting %d stored link key(s)", n);
    }
    return (unsigned)n;
}

typedef struct { u8 data[BT_EVENT_MAX]; u32 len; } BtEvent;

static BtEvent  s_events[BT_MAX_EVENTS];
static unsigned s_ev_head, s_ev_count;

/* The guest's outstanding "give me an HCI event" reads, held until there is an
 * event to hand back. The stack keeps SEVERAL of these in flight at once, so a
 * single slot is not enough: overwriting it strands every earlier request, and
 * whatever thread was waiting on one never wakes. */
#define BT_MAX_REQS 8

typedef struct { u32 req, buf, len; } BtReq;

static BtReq    s_hci_reqs[BT_MAX_REQS];
static unsigned s_hci_rhead, s_hci_rcount;
static BtReq    s_acl_reqs[BT_MAX_REQS];
static unsigned s_acl_rhead, s_acl_rcount;

static int bt_push_req(BtReq *q, unsigned *head, unsigned *count,
                       u32 req, u32 buf, u32 len)
{
    if (*count >= BT_MAX_REQS)
        return 0;
    q[(*head + *count) % BT_MAX_REQS].req = req;
    q[(*head + *count) % BT_MAX_REQS].buf = buf;
    q[(*head + *count) % BT_MAX_REQS].len = len;
    (*count)++;
    return 1;
}

static unsigned s_scan_enable;          /* set by Write_Scan_Enable  */
static unsigned s_scan_ticks;           /* ticks since scanning began */
static unsigned s_link_ticks;           /* ticks since the link came up  */

/* All connection pacing is GUEST TIME, not update-call counts: the update
 * cadence differs wildly between the qemu harness and the console (the old
 * 30000-call offer delay was ~1s under qemu and MINUTES of wall clock on the
 * PS3 -- the wiimote "never" connected simply because nobody waited). */
#define BT_TB_SEC 60750000ull

/* Milliseconds of GUEST time since the baseband link came up. Without this the
 * log cannot distinguish "the title disconnected because of something we sent"
 * from "the title gave up after a fixed timeout" -- and those call for
 * opposite fixes. Guest time, not wall time, because that is the clock the
 * title's own timers run on. */
static u64 s_link_up_tb;
static unsigned bt_link_ms(void)
{
    u64 now;
    if (!s_link_up_tb) return 0;
    now = timing_timebase();
    if (now < s_link_up_tb) return 0;
    return (unsigned)(((now - s_link_up_tb) * 1000ull) / BT_TB_SEC);
}

/* How long to wait after the title enables page scan before offering the
 * remote, in guest seconds. Measured at the identical guest instant (first
 * link-up), the title has our BD_ADDR in 12+ places under qemu but only 3 on
 * the console -- it has not finished populating its registered-device table
 * when our connection arrives, and it then fails the 6-byte address compare
 * at 0x801cf648 and disconnects. Configurable by file so it can be swept with
 * an FTP upload and a restart, no rebuild. */
static u64 bt_offer_delay_tb(void)
{
    static s64 secs = -1;
    if (secs < 0) {
        FILE *f = fopen("/dev_hdd0/tmp/wiicompiled-btdelay.txt", "r");
        const char *e;
        int v = 1;
        if (f) { if (fscanf(f, "%d", &v) != 1) v = 1; fclose(f); }
        else if ((e = getenv("BT_OFFER_DELAY")) && *e) v = (int)strtol(e, NULL, 0);
        if (v < 0) v = 0;
        if (v > 60) v = 60;
        secs = v;
        LOG_INFO(LOG_CORE, "BT: offer delay = %d guest second(s)", v);
    }
    return (u64)secs * BT_TB_SEC;
}

static int s_wm_requested;              /* connection request sent   */
static int s_allow_connect = 1;         /* harness gate on offering  */
static void (*s_disc_hook)(void);        /* see ios_bt_set_disconnect_hook */
static int s_wm_connected;              /* link established          */
static unsigned s_commands, s_events_sent, s_acl_sent, s_acl_recv;

unsigned ios_bt_commands(void)   { return s_commands; }
unsigned ios_bt_events_sent(void){ return s_events_sent; }
unsigned ios_bt_acl_sent(void){ return s_acl_sent; }
unsigned ios_bt_acl_recv(void){ return s_acl_recv; }

#define BT_MAX_ACL 32   /* was 8: a connected Wiimote streams ~200 Hz input
                        * reports and the guest drains in bursts; 8 slots
                        * dropped reports at boot ("TX DROPPED, queue 8 full"),
                        * which WPAD reads as a flaky remote. */
#define BT_ACL_MAX   80

typedef struct { u8 data[BT_ACL_MAX]; u32 len; u64 due; } BtAcl;

static BtAcl    s_acl[BT_MAX_ACL];
static unsigned s_acl_head, s_acl_count;

/* Our channel identifiers, and the ones the stack hands back. */
#define CID_SIGNAL       0x0001
#define CID_LOCAL_CNTL   0x0040
#define CID_LOCAL_INTR   0x0041
#define PSM_HID_CNTL     0x0011
#define PSM_HID_INTR     0x0013

#define L2CAP_COMMAND_REJECT 0x01
#define L2CAP_CONNECT_REQ  0x02
#define L2CAP_CONNECT_RSP  0x03
#define L2CAP_CONFIG_REQ   0x04
#define L2CAP_CONFIG_RSP   0x05
#define L2CAP_DISCONNECT_REQ 0x06
#define L2CAP_DISCONNECT_RSP 0x07

static u16 s_remote_cid_cntl, s_remote_cid_intr;   /* the stack's CIDs */
static u8  s_l2_ident = 1;
static int s_cntl_open, s_intr_open;
static int s_status_pending;
/* Runtime experiment knobs, settable over devlink so a single console
 * session can test several hypotheses instead of one per launch.
 *   bit0  do NOT initiate the HID channels; wait for the title to do it
 *   bit1  do NOT raise Link_Key_Request unprompted
 *   bit2  delay the first channel offer by a further guest second     */
unsigned g_bt_experiment;

/* Let the harness drive the same experiment mask the console exposes over
 * devlink, so a hardware observation can be reproduced under qemu as a
 * controlled A/B instead of being argued about. */
static void bt_experiment_from_env(void)
{
    static int done;
    const char *e;
    if (done) return;
    done = 1;
    e = getenv("BT_EXPERIMENT");
    if (e && *e) {
        g_bt_experiment = (unsigned)strtoul(e, NULL, 0);
        LOG_INFO(LOG_CORE, "BT: experiment mask = %u (from env)",
                 g_bt_experiment);
    }
}

/* The link key for our emulated remote. Seeded with the value we report if the
 * title never writes one, and overwritten by Write_Stored_Link_Key so we hand
 * back exactly the key the title expects rather than one we invented. */
/* Set when the title has refused an HID channel with L2CAP result 3
 * ("security block"). Hardware does this when it considers the link
 * unauthenticated; qemu never does. Once seen, we stop waiting for the title
 * to drive authentication and declare the link secure ourselves on the next
 * link, which is honest here: we emulate both the adapter and a remote the
 * console's own SYSCONF registers, so no real pairing secret is missing. */
static int s_security_refused;

static u8 s_stored_key[16] = {
    0xa0,0xa0,0xa0,0xa0,0xa0,0xa0,0xa0,0xa0,
    0xa0,0xa0,0xa0,0xa0,0xa0,0xa0,0xa0,0xa0
};
static int s_link_setup_done;
static int s_auth_pending;   /* raise Link_Key_Request on the next pump */
static int s_authenticated;  /* the link key has been supplied      */
static u64 s_link_tb;      /* L2CAP connect retry clock; 0 = connect now */
static int s_l2_started;

static void bt_try_deliver_acl(void);

static void bt_push_event(const u8 *ev, u32 len);

/* Drop queued-but-undelivered data frames. Called at every link boundary:
 * frames composed for one connection epoch must never be delivered into the
 * next. The queues were previously cleared only by a full HCI reset, so on
 * real hardware -- where the game is GPU-busy and slow to post its bulk-IN
 * reads during a scene change -- our L2CAP retries piled up, the queue
 * filled, NEW handshake frames were then silently dropped (bt_send_l2cap
 * returns without a trace on a full queue), and every later connection began
 * by feeding the game a backlog of stale signals from dead links. qemu never
 * showed it because the game there reads promptly and the queue never backs
 * up. 372 baseband links, zero channel setups. */
static void bt_flush_data_queues(const char *why)
{
    if (s_acl_count || s_ev_count)
        LOG_INFO(LOG_CORE, "BT: flushing queues (%s): acl=%u ev=%u",
                 why, s_acl_count, s_ev_count);
    s_acl_head = s_acl_count = 0;
    /* events: keep any COMMAND status/complete already queued -- those answer
     * specific host commands -- but data-bearing remnants go with the link.
     * In practice the event queue at a link boundary holds only our own
     * connection events, so clearing it entirely is correct and simpler. */
    s_ev_head = s_ev_count = 0;
}

/* Every path that ends a baseband link comes through here.
 *
 * L2CAP channels belong to the LINK, not to the device: when the link goes
 * away so do the channels, and a fresh link starts with none. Leaving the CIDs
 * and the "configured" flags standing across a teardown is not a cosmetic
 * bookkeeping slip -- it is silent, total input loss. The remote then believes
 * its HID channels are still up, never re-runs the L2CAP handshake on the new
 * link, and goes on transmitting button reports to a CID the host no longer
 * has. The host's USB layer still receives every frame (they are visible in
 * its ACL buffers), its L2CAP layer discards every one of them, and the game
 * sits there ignoring a controller that looks perfectly healthy from this
 * side: channels 3/3, thousands of frames delivered, an empty queue, and not
 * one button ever reaching WPAD.
 *
 * That is exactly what Mario Kart Wii did. It brought the first link up,
 * completed both channels, read the EEPROM, set report mode 0x30 -- and then
 * tore the link down and paged again. Everything after that point was
 * transmitted into a void. */
/* ACL packets received from the host since we last returned its credits.
 * See the Number_Of_Completed_Packets block in ios_bt_update. */
static u16 s_acl_out_uncredited;
static unsigned s_input_reports;   /* HID input reports handed to L2CAP */
static unsigned s_acl_credited;    /* ACL credits handed back to the host  */
static int      s_read_1770_seen;  /* one 0x1770 probe seen on this link  */
/* The IPC request currently being serviced. An IOS request block carries the
 * guest's async callback pointer, which names the code that issued the
 * command -- the only cheap way to find out WHO disconnects the remote. */
static u32 s_cur_req;

static void bt_link_down(const char *why)
{
    s_link_setup_done = 0;
    s_link_up_tb = 0;
    s_authenticated = 0;
    s_auth_pending = 0;
    s_acl_out_uncredited = 0;
    s_read_1770_seen = 0;
    LOG_INFO(LOG_CORE, "BT: link down (%s) -- dropping L2CAP channel state "
             "(cntl=%d intr=%d)", why, s_cntl_open, s_intr_open);
    s_wm_connected = 0;
    s_wm_requested = 0;
    s_remote_cid_cntl = s_remote_cid_intr = 0;
    s_cntl_open = s_intr_open = 0;
    s_l2_started = 0;
    s_link_ticks = 0;
    s_link_tb = 0;
    s_status_pending = 0;
    bt_flush_data_queues(why);
}

/* Deliver one queued event into a parked request, if both exist. */
static void bt_try_deliver(void)
{
    BtEvent *e;
    u32 n, i;

    BtReq *r;

    if (s_hci_rcount == 0 || s_ev_count == 0)
        return;

    r = &s_hci_reqs[s_hci_rhead];
    e = &s_events[s_ev_head];
    {   /* Report the stack's event-read buffer size once. If it is smaller
         * than our largest events -- Remote_Name_Complete is 257 bytes and a
         * full Return_Link_Keys is 113 -- every one of them is delivered
         * short, and a name the stack cannot read is a remote it will not
         * authenticate. Unconditional so one run answers the question. */
        static int reported;
        if (!reported) { reported = 1;
            LOG_INFO(LOG_CORE, "BT: HCI event read buffer = %u bytes", r->len); }
    }
    n = e->len < r->len ? e->len : r->len;
    {   /* Dump every event of the first link verbatim. The title reaches the
         * SAME branch point ~15 ms after link-up and goes one way under qemu
         * (Authentication_Requested) and the other on hardware (Disconnect),
         * so either some event carries different bytes on the two machines or
         * the divergence is in guest execution itself. Printing the bytes is
         * the only way to tell those apart rather than guessing. */
        static unsigned dumped;
        if (dumped < 24) {
            char hx[3 * 64 + 8];
            unsigned k, w = 0, lim = n < 64 ? n : 64;
            dumped++;
            for (k = 0; k < lim && w + 4 < sizeof hx; k++)
                w += (unsigned)snprintf(hx + w, sizeof hx - w, "%02x ",
                                        e->data[k]);
            if (w) hx[w - 1] = 0; else hx[0] = 0;
            LOG_INFO(LOG_CORE, "BT: EV[%02u] len=%u/%u: %s",
                     dumped, e->len, n, hx);
        }
    }
    if (n < e->len) {
        /* The stack asked for fewer bytes than the event carries, so it will
         * parse a short record and quietly get it wrong. Remote_Name_Complete
         * is 257 bytes and Return_Link_Keys with a full store is 113, so this
         * is not hypothetical -- and a name the stack cannot read is a remote
         * it will not authenticate. */
        static unsigned trunc_rl;
        if (trunc_rl < 16) { trunc_rl++;
            LOG_WARN(LOG_CORE, "BT: event 0x%02x TRUNCATED %u -> %u bytes "
                     "(guest buffer too small)", e->data[0], e->len, r->len); }
    }
    for (i = 0; i < n; i++)
        mem_write8(r->buf + i, e->data[i]);

    s_ev_head = (s_ev_head + 1) % BT_MAX_EVENTS;
    s_ev_count--;

    /* The ioctlv's result is the number of bytes delivered. */
    ios_write_reply(r->req, (s32)n);
    ipc_queue_reply(r->req);
    s_hci_rhead = (s_hci_rhead + 1) % BT_MAX_REQS;
    s_hci_rcount--;
    s_events_sent++;
}

static unsigned s_acl_delivered, s_acl_queued_total;

static void bt_try_deliver_acl(void)
{
    if (bt_trace() && s_acl_count)
        fprintf(stderr, "[bt] deliver_acl: queued %u, parked reads %u\n",
                s_acl_count, s_acl_rcount);
    BtAcl *a;
    u32 n, i;

    /* Events outrank ACL data, and a frame only moves when the stack has a
     * bulk-IN request waiting (Dolphin BTEmu.cpp SendACLPacket). */
    BtReq *r;

    /* Events outrank ACL data only when an event can ACTUALLY be delivered
     * right now -- i.e. one is queued AND a request is parked to receive it.
     * Blocking on a merely-queued event starves ACL forever whenever an event
     * has nobody waiting for it: the button reports then never reach WPAD, it
     * decides the remote is gone, and the stack drops and re-pairs it in a
     * loop -- exactly the reconnect churn in the console log while reports
     * were being generated at channels=3. */
    if (s_acl_rcount == 0 || s_acl_count == 0 ||
        (s_ev_count != 0 && s_hci_rcount != 0))
        return;

    r = &s_acl_reqs[s_acl_rhead];
    a = &s_acl[s_acl_head];
    /* A real remote answers over the air, milliseconds later. Here the reply
     * is composed inside the guest's own send call, so it can arrive far
     * sooner than any hardware would manage -- and a stack that updates its
     * bookkeeping AFTER the send can then be overtaken by its own answer.
     * BT_REPLY_DELAY_US holds each frame back by that much guest time. */
    if (a->due && timing_timebase() < a->due) return;
    n = a->len < r->len ? a->len : r->len;
    {   /* The one payload not yet compared across the two machines. The title
         * reaches the same branch at the same guest time having received
         * byte-identical HCI events, so if it still decides differently the
         * difference must be in what we hand it here. */
        static unsigned dumped_acl;
        if (dumped_acl < 12) {
            char hx[3 * 32 + 8];
            unsigned k, w = 0, lim = n < 32 ? n : 32;
            dumped_acl++;
            for (k = 0; k < lim && w + 4 < sizeof hx; k++)
                w += (unsigned)snprintf(hx + w, sizeof hx - w, "%02x ",
                                        a->data[k]);
            if (w) hx[w - 1] = 0; else hx[0] = 0;
            LOG_INFO(LOG_CORE, "BT: ACL-OUT[%02u] len=%u/%u: %s",
                     dumped_acl, a->len, n, hx);
        }
    }
    for (i = 0; i < n; i++)
        mem_write8(r->buf + i, a->data[i]);

    s_acl_head = (s_acl_head + 1) % BT_MAX_ACL;
    if (s_acl_count) s_acl_count--;   /* belt and braces vs teardown races */

    ios_write_reply(r->req, (s32)n);
    ipc_queue_reply(r->req);
    s_acl_delivered++;
    s_acl_rhead = (s_acl_rhead + 1) % BT_MAX_REQS;
    s_acl_rcount--;
    s_acl_sent++;
}

static void bt_push_event(const u8 *ev, u32 len)
{
    BtEvent *slot;
    if (s_ev_count >= BT_MAX_EVENTS) {
        LOG_WARN(LOG_CORE, "BT: event queue full, dropping");
        return;
    }
    slot = &s_events[(s_ev_head + s_ev_count) % BT_MAX_EVENTS];
    if (len > BT_EVENT_MAX) len = BT_EVENT_MAX;
    memcpy(slot->data, ev, len);
    slot->len = len;
    s_ev_count++;
    bt_try_deliver();
}

/* ------------------------------------------------------------------ */
/* ACL data and L2CAP                                                   */
/*                                                                      */
/* Once the baseband link is up the remote is the side that opens its    */
/* channels: HID control (PSM 0x11) and HID interrupt (PSM 0x13). Those  */
/* live on L2CAP, carried as ACL data, so the adapter needs a second     */
/* queue -- events always outrank it, as in Dolphin's Update().          */
/* ------------------------------------------------------------------ */


/* Queue one L2CAP frame on `cid`, wrapped in its ACL header. */
static void bt_send_l2cap(u16 cid, const u8 *payload, u32 len)
{
    if (bt_trace())
        fprintf(stderr, "[bt] l2cap tx cid=%04x len=%u b0=%02x b4=%02x\n",
                cid, (unsigned)len, len ? payload[0] : 0,
                len > 4 ? payload[4] : 0);
    BtAcl *slot;
    u32 off = 0, i;
    u16 handle = WM_CON_HANDLE | 0x2000;      /* packet-boundary = start */

    if (s_acl_count >= BT_MAX_ACL || len + 8 > BT_ACL_MAX) {
        static unsigned dropped;
        if (dropped < 8) { dropped++;
            LOG_INFO(LOG_CORE, "BT: TX DROPPED (queue %u full) cid=%04x b0=%02x",
                     s_acl_count, cid, len ? payload[0] : 0); }
        return;
    }
    slot = &s_acl[(s_acl_head + s_acl_count) % BT_MAX_ACL];

    slot->data[off++] = (u8)(handle & 0xFF);  /* ACL header, little-endian */
    slot->data[off++] = (u8)(handle >> 8);
    slot->data[off++] = (u8)((len + 4) & 0xFF);
    slot->data[off++] = (u8)((len + 4) >> 8);
    slot->data[off++] = (u8)(len & 0xFF);     /* L2CAP header */
    slot->data[off++] = (u8)(len >> 8);
    slot->data[off++] = (u8)(cid & 0xFF);
    slot->data[off++] = (u8)(cid >> 8);
    for (i = 0; i < len; i++) slot->data[off++] = payload[i];

    slot->len = off;
    {   static s64 delay_tb = -1;
        if (delay_tb < 0) {
            const char *e = getenv("BT_REPLY_DELAY_US");
            delay_tb = (e && *e)
                ? (s64)((u64)strtoul(e, NULL, 0) * (BT_TB_SEC / 1000000u)) : 0;
        }
        slot->due = delay_tb ? timing_timebase() + (u64)delay_tb : 0;
    }
    s_acl_count++;
    bt_try_deliver_acl();
}

/* One L2CAP signalling command on the signalling channel. */
static void bt_signal_log(u8 code, u32 len)
{
    static unsigned n;
    if (n < 40) { n++; LOG_INFO(LOG_CORE, "BT: [+%4ums] l2cap TX code=%02x len=%u", bt_link_ms(), code, len); }
}

static void bt_send_signal(u8 code, u8 ident, const u8 *data, u32 len)
{
    bt_signal_log(code, len);
    u8 buf[32];
    u32 i;
    if (len + 4 > sizeof buf) return;
    buf[0] = code;
    buf[1] = ident;
    buf[2] = (u8)(len & 0xFF);
    buf[3] = (u8)(len >> 8);
    for (i = 0; i < len; i++) buf[4 + i] = data[i];
    bt_send_l2cap(CID_SIGNAL, buf, len + 4);
}

static void bt_l2cap_connect(u16 psm, u16 local_cid)
{
    u8 d[4];
    d[0] = (u8)(psm & 0xFF);       d[1] = (u8)(psm >> 8);
    d[2] = (u8)(local_cid & 0xFF); d[3] = (u8)(local_cid >> 8);
    bt_send_signal(L2CAP_CONNECT_REQ, s_l2_ident++, d, 4);
}

/* ------------------------------------------------------------------ */
/* HCI responder                                                        */
/* ------------------------------------------------------------------ */

/* Command Complete: [0x0E][3+N][num_pkts=1][opcode lo][opcode hi][params]. */
static u32 build_cc(u8 *out, u16 opcode, const u8 *params, u8 n)
{
    out[0] = EVT_COMMAND_COMPLETE;
    out[1] = (u8)(3 + n);
    out[2] = 0x01;
    out[3] = (u8)(opcode & 0xFF);
    out[4] = (u8)(opcode >> 8);
    if (n) memcpy(out + 5, params, n);
    return 5u + n;
}

/* Command Status: [0x0F][4][status][num_pkts=1][opcode lo][opcode hi]. Used for
 * the commands whose real result arrives later as its own event. */
static u32 build_cs(u8 *out, u16 opcode, u8 status)
{
    out[0] = EVT_COMMAND_STATUS;
    out[1] = 0x04;
    out[2] = status;
    out[3] = 0x01;
    out[4] = (u8)(opcode & 0xFF);
    out[5] = (u8)(opcode >> 8);
    return 6;
}

static void bt_execute_command(u16 opcode, const u8 *payload, u8 payload_len)
{
    {   /* Every command the stack issues, in order. The bring-up sequence is
         * the thing being debugged and it is invisible without this. */
        static unsigned n;
        if (n < 160) { n++;
            PPCState *cs = timing_bound_cpu();
            LOG_INFO(LOG_CORE, "BT: [+%4ums] cmd %04x plen=%u lr=%08x",
                     bt_link_ms(), opcode, (unsigned)payload_len,
                     cs ? (unsigned)cs->lr : 0u);
            /* For the two commands that ARE the divergence -- the title sends
             * Authentication_Requested under qemu and Disconnect on hardware,
             * at the same guest time, having received byte-identical events
             * and ACL frames -- walk the guest's own call chain. Comparing the
             * two chains says which guest function took the other branch,
             * which is the only thing left that can name the cause. */
            if (cs && (opcode == 0x0406 || opcode == 0x0411)) {
                u32 fp = cs->gpr[1];
                /* The deciding compare is memcmp(r28, r31+0xd00, 6) at guest
                 * 0x801cf648; r26..r31 are callee-saved, so if the inner
                 * frames left them alone these still name both operands. */
                LOG_INFO(LOG_CORE, "BT:   r26=%08x r27=%08x r28=%08x "
                         "r29=%08x r30=%08x r31=%08x",
                         (unsigned)cs->gpr[26], (unsigned)cs->gpr[27],
                         (unsigned)cs->gpr[28], (unsigned)cs->gpr[29],
                         (unsigned)cs->gpr[30], (unsigned)cs->gpr[31]);
                LOG_INFO(LOG_CORE, "BT:   [r28]=%02x %02x %02x %02x %02x %02x"
                         "  [r31+0xd00]=%02x %02x %02x %02x %02x %02x",
                         mem_read8(cs->gpr[28] + 0), mem_read8(cs->gpr[28] + 1),
                         mem_read8(cs->gpr[28] + 2), mem_read8(cs->gpr[28] + 3),
                         mem_read8(cs->gpr[28] + 4), mem_read8(cs->gpr[28] + 5),
                         mem_read8(cs->gpr[31] + 0xd00),
                         mem_read8(cs->gpr[31] + 0xd01),
                         mem_read8(cs->gpr[31] + 0xd02),
                         mem_read8(cs->gpr[31] + 0xd03),
                         mem_read8(cs->gpr[31] + 0xd04),
                         mem_read8(cs->gpr[31] + 0xd05));
                unsigned d;
                for (d = 0; d < 8 && (fp >> 28); d++) {
                    u32 next = mem_read32(fp), slr = mem_read32(fp + 4);
                    LOG_INFO(LOG_CORE, "BT:   frame%u sp=%08x lr=%08x",
                             d, (unsigned)fp, (unsigned)slr);
                    /* The frame that returns to 0x80194520 belongs to the
                     * function holding the failed BD_ADDR compare. Its saved
                     * r26..r31 live in this frame, and r31 is the base the
                     * compare reads from (r31+0xd00) -- the operand that
                     * cannot be recovered from the live registers because the
                     * inner IPC frames overwrite them. */
                    if (slr == 0x80194520u) {
                        unsigned w;
                        for (w = 0; w < 16; w += 4)
                            LOG_INFO(LOG_CORE,
                                     "BT:     f%u+%02x: %08x %08x %08x %08x",
                                     d, w * 4,
                                     mem_read32(fp + w * 4),
                                     mem_read32(fp + w * 4 + 4),
                                     mem_read32(fp + w * 4 + 8),
                                     mem_read32(fp + w * 4 + 12));
                    }
                    if (next <= fp) break;
                    fp = next;
                }
            } } }
    if (bt_trace())
        fprintf(stderr, "[bt] cmd %04x len=%u\n", opcode, payload_len);
    u8 out[BT_EVENT_MAX], p[16];

    s_commands++;

    switch (opcode) {
    case CMD_READ_BUFFER_SIZE: {
        /* How many ACL packets the host may keep in flight. Overridable so
         * "the host stopped transmitting" can be tested against it directly. */
        const char *e = getenv("BT_ACL_PKTS");
        u16 npkts = (e && *e) ? (u16)strtoul(e, NULL, 0) : (u16)ACL_PKT_NUM;
        p[0] = 0x00;
        p[1] = (u8)(ACL_PKT_SIZE & 0xFF); p[2] = (u8)(ACL_PKT_SIZE >> 8);
        p[3] = (u8)SCO_PKT_SIZE;
        p[4] = (u8)(npkts & 0xFF);        p[5] = (u8)(npkts >> 8);
        p[6] = (u8)(SCO_PKT_NUM & 0xFF);  p[7] = (u8)(SCO_PKT_NUM >> 8);
        bt_push_event(out, build_cc(out, opcode, p, 8));
        return;
    }

    case CMD_READ_LOCAL_VER:
        p[0] = 0x00;
        p[1] = 0x03;                        /* HCI 1.1        */
        p[2] = 0xA7; p[3] = 0x40;           /* revision       */
        p[4] = 0x03;                        /* LMP 1.1        */
        p[5] = 0x0F; p[6] = 0x00;           /* manufacturer   */
        p[7] = 0x0E; p[8] = 0x43;           /* LMP subversion */
        bt_push_event(out, build_cc(out, opcode, p, 9));
        return;

    case CMD_READ_LOCAL_FEATURES:
        p[0] = 0x00;
        memcpy(p + 1, k_features, 8);
        bt_push_event(out, build_cc(out, opcode, p, 9));
        return;

    case CMD_READ_BDADDR:
        p[0] = 0x00;
        memcpy(p + 1, k_bdaddr, 6);
        bt_push_event(out, build_cc(out, opcode, p, 7));
        return;

    case CMD_WRITE_LINK_SUP_TIMEOUT:
        /* The title's LAST per-link setup step. Offering HID before this is
         * what the console answers with L2CAP result 3: it has not finished
         * bringing the link up, so its HID service is not yet listening. */
        {
        s_link_setup_done = 1;
        u16 handle = 0;
        if (payload_len >= 4)
            LOG_INFO(LOG_CORE, "BT: link supervision timeout = %u slots "
                     "(%u ms)", (unsigned)(payload[2] | (payload[3] << 8)),
                     (unsigned)((payload[2] | (payload[3] << 8)) * 625u / 1000u));
        if (payload_len >= 2)
            handle = (u16)(payload[0] | (payload[1] << 8));
        p[0] = 0x00;
        p[1] = (u8)(handle & 0xFF); p[2] = (u8)(handle >> 8);
        bt_push_event(out, build_cc(out, opcode, p, 3));
        return;
    }

    case CMD_WRITE_LINK_POLICY:
        /* Command STATUS, not Command Complete. The spec says otherwise, but
         * the Wii's own stack is what has to be satisfied and Dolphin -- which
         * runs this title -- answers this one with a Command Status
         * (BTEmu.cpp:1453). We were answering with an EMPTY Command Complete:
         * the wrong event, carrying no status byte at all, so the stack read
         * its result from whatever followed in the buffer. */
        bt_push_event(out, build_cs(out, opcode, 0x00));
        return;

    case CMD_VENDOR_FC4C:
    case CMD_VENDOR_FC4F:
        /* The Broadcom firmware-patch upload (WUDiAppendRuntimePatch): write
         * RAM, then launch RAM. Each completes with a one-byte STATUS, which
         * the stack reads; an empty Command Complete left it reading a
         * result byte we never wrote. */
        p[0] = 0x00;
        bt_push_event(out, build_cc(out, opcode, p, 1));
        return;

    case CMD_DELETE_STORED_LINK_KEY:
        p[0] = 0x00; p[1] = 0x00; p[2] = 0x00;   /* deleted none */
        bt_push_event(out, build_cc(out, opcode, p, 3));
        return;

    case CMD_WRITE_STORED_LINK_KEY: {
        /* num_keys(1) then num x { BD_ADDR[6], link_key[16] }. This is how the
         * title hands the adapter the key it holds for a remote listed in
         * SYSCONF's BT.DINF, so the key it will later expect from us is
         * exactly this one -- not a constant of our own choosing. */
        unsigned n = (payload_len >= 1) ? payload[0] : 0, i, j;
        for (i = 0; i < n && 1 + i * 22 + 22 <= payload_len; i++) {
            const u8 *rec = payload + 1 + i * 22;
            for (j = 0; j < 6 && rec[j] == k_wm_bdaddr[j]; j++) { }
            if (j == 6) {
                for (j = 0; j < 16; j++) s_stored_key[j] = rec[6 + j];
                LOG_INFO(LOG_CORE, "BT: title stored our link key "
                         "(%02x%02x..%02x)", s_stored_key[0], s_stored_key[1],
                         s_stored_key[15]);
            }
        }
        p[0] = 0x00; p[1] = (u8)n;
        bt_push_event(out, build_cc(out, opcode, p, 2));
        return;
    }

    case CMD_READ_STORED_LINK_KEY: {
        /* The adapter reports its stored keys first, then completes the command
         * (Dolphin BTEmu.cpp:1508-1540). One key per remote registered in
         * SYSCONF's BT.DINF: bdaddr then a 16-byte key of 0xa0+i, matching
         * Dolphin's WiimoteDevice. Reporting none makes the stack's security
         * path retry instead of settling. */
        u8 rk[BT_EVENT_MAX];
        unsigned i, off = 0;
        /* Report the WHOLE key store in ONE event, exactly as the real
         * adapter does (Dolphin BTEmu.cpp:861-892 SendEventLinkKeyNotification,
         * reached from CommandReadStoredLinkKey): five records for
         * 11:02:19:79:00:00 .. :04, each with a 16-byte key of 0xa0+index,
         * then Command_Complete with num_keys_read = 5, max_num_keys = 255.
         *
         * Reporting only ONE key is what hardware rejected: the title issued
         * Read_Stored_Link_Key and IMMEDIATELY followed it with
         * Delete_Stored_Link_Key (0x0C12), i.e. it threw the store away as
         * inconsistent. With no key it cannot authenticate, so it answered
         * our very first HID CONNECT_REQ with L2CAP result 3 and hung up in
         * the same breath -- never sending Authentication_Requested at all,
         * which is why no amount of fixing the auth handler helped.
         *
         * Slot 0 uses s_stored_key so a key the title writes back via
         * Write_Stored_Link_Key still wins for the remote we present. */
        unsigned slot;
        rk[off++] = EVT_RETURN_LINK_KEYS;
        rk[off++] = 0;                           /* length, filled below */
        unsigned nkeys = bt_num_stored_keys();
        rk[off++] = (u8)nkeys;
        for (slot = 0; slot < nkeys; slot++) {
            for (i = 0; i < 5; i++) rk[off++] = k_wm_bdaddr[i];
            rk[off++] = (u8)slot;                /* bdaddr[5] = remote index */
            for (i = 0; i < 16; i++)
                rk[off++] = slot ? (u8)(0xa0 + slot) : s_stored_key[i];
        }
        rk[1] = (u8)(off - 2);
        bt_push_event(rk, off);

        p[0] = 0x00;
        p[1] = 0xFF; p[2] = 0x00;                /* max_num_keys  = 255 */
        p[3] = (u8)nkeys; p[4] = 0x00;           /* num_keys_read */
        bt_push_event(out, build_cc(out, opcode, p, 5));
        return;
    }

    case CMD_WRITE_SCAN_ENABLE:
        if (payload_len >= 1) s_scan_enable = payload[0];
        if (bt_trace())
            fprintf(stderr, "[bt] scan_enable=%02x\n", s_scan_enable);
        p[0] = 0x00;
        bt_push_event(out, build_cc(out, opcode, p, 1));
        return;

    case CMD_RESET:
    case CMD_SET_EVENT_FILTER:
    case CMD_WRITE_PIN_TYPE:
    case CMD_WRITE_LOCAL_NAME:
    case CMD_WRITE_PAGE_TIMEOUT:
    case CMD_WRITE_UNIT_CLASS:
    case CMD_HOST_BUFFER_SIZE:
    case CMD_WRITE_INQUIRY_SCAN_TYPE:
    case CMD_WRITE_INQUIRY_MODE:
    case CMD_WRITE_PAGE_SCAN_TYPE:
        p[0] = 0x00;
        bt_push_event(out, build_cc(out, opcode, p, 1));
        return;

    case CMD_CREATE_CON: {
        /* The console connecting TO the remote, rather than accepting one.
         * WPAD takes this path for a remote it has seen before -- which is
         * every boot once the save persists and a licence exists, and it is
         * why the console looped forever offering a connection the title was
         * never going to accept: it was busy trying to place its own call and
         * we never answered. Reply exactly as for an accepted connection:
         * Command Status, then Connection Complete carrying the handle. */
        u8 ev[BT_EVENT_MAX];
        unsigned i, off = 0;
        bt_push_event(out, build_cs(out, opcode, 0x00));

        ev[off++] = EVT_CON_COMPL;
        ev[off++] = 11;
        ev[off++] = 0x00;                              /* status  */
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        for (i = 0; i < 6; i++) ev[off++] = k_wm_bdaddr[i];
        ev[off++] = HCI_LINK_ACL;
        ev[off++] = 0x00;                              /* no encryption */
        bt_push_event(ev, off);
        s_wm_connected = 1;
        /* Ask the host for the stored link key IMMEDIATELY, without waiting
         * for an Authentication_Requested. On hardware the title never sends
         * one (measured: zero HCI 0x0411 across 18 links) and then refuses our
         * HID channel with L2CAP result 3, "security block" -- its stack will
         * not put HID on an unauthenticated link. A real controller resolves
         * this during connection setup by raising Link_Key_Request itself; the
         * host answers from SYSCONF's BT.DINF entry and the link is
         * authenticated before any channel is offered. */
        s_auth_pending = 1;
        s_authenticated = 0;
        s_link_up_tb = timing_timebase();
        s_wm_requested = 1;    /* stop offering: the link is up */
        s_link_tb = 0;         /* bring the HID channels up straight away */
        LOG_INFO(LOG_CORE, "BT: console CREATED connection to the wiimote");
        return;
    }

    case CMD_ACCEPT_CON: {
        bt_flush_data_queues("fresh link");
        /* The stack accepted our remote. Status first, then the link is up:
         * Connection Complete carries the handle everything later refers to. */
        u8 ev[BT_EVENT_MAX];
        unsigned i, off = 0;
        bt_push_event(out, build_cs(out, opcode, 0x00));

        /* The accept asks for a role; when the host wants master (role 0) the
         * controller performs the switch and reports it before the link comes
         * up (Dolphin CommandAcceptCon). Without this the stack sees the
         * connection complete in a role it did not agree to and goes quiet. */
        if (payload_len >= 7 && payload[6] == 0x00) {
            ev[off++] = EVT_ROLE_CHANGE;
            ev[off++] = 8;
            ev[off++] = 0x00;                          /* status */
            for (i = 0; i < 6; i++) ev[off++] = k_wm_bdaddr[i];
            ev[off++] = 0x00;                          /* new role: master */
            bt_push_event(ev, off);
            off = 0;
        }

        ev[off++] = EVT_CON_COMPL;
        ev[off++] = 11;
        ev[off++] = 0x00;                              /* status  */
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        for (i = 0; i < 6; i++) ev[off++] = k_wm_bdaddr[i];
        ev[off++] = HCI_LINK_ACL;
        ev[off++] = 0x00;                              /* no encryption */
        bt_push_event(ev, off);
        s_wm_connected = 1;
        /* Ask the host for the stored link key IMMEDIATELY, without waiting
         * for an Authentication_Requested. On hardware the title never sends
         * one (measured: zero HCI 0x0411 across 18 links) and then refuses our
         * HID channel with L2CAP result 3, "security block" -- its stack will
         * not put HID on an unauthenticated link. A real controller resolves
         * this during connection setup by raising Link_Key_Request itself; the
         * host answers from SYSCONF's BT.DINF entry and the link is
         * authenticated before any channel is offered. */
        s_auth_pending = 1;
        s_authenticated = 0;
        s_link_up_tb = timing_timebase();
        /* Connect the HID channels IMMEDIATELY on a fresh link. The retry
         * clock is shared, so after the first connection it still held the
         * previous link's timestamp and made every RE-connection wait half a
         * guest second -- and the title re-initialises its stack faster than
         * that, which is why the console logged 140 baseband links and zero
         * channel completions in a row. */
        s_link_tb = 0;
        /* Do NOT clear the channel state here. This handler runs whenever the
         * stack accepts a connection, which can happen while an L2CAP
         * handshake for the SAME link is already in flight -- wiping the CIDs
         * then discards a CONNECT_RSP that has already arrived and the
         * channels can never reach the configured state. The console showed
         * exactly that: 220 baseband links, zero channel completions. The
         * teardown path already clears these, which is the correct place. */
        LOG_INFO(LOG_CORE, "BT: wiimote ACL link up");
        {   /* Scan guest RAM for our BD_ADDR at the EXACT guest moment the
             * link comes up, on both machines. Scanning at end-of-run instead
             * compares two different points in the title's life and answers
             * nothing -- which is what the first attempt did. */
            static int scanned;
            const char *want = getenv("BT_SCAN_LINKUP");
            if (!want || !*want) {
                /* The console build has no environment, so the same switch is
                 * a file that can be dropped in over FTP. */
                FILE *tf = fopen("/dev_hdd0/tmp/wiicompiled-scan.txt", "r");
                if (tf) { fclose(tf); want = "1"; }
            }
            if (!scanned && want && *want) {
                static const u8 pat[6] = {0x11,0x02,0x19,0x79,0x00,0x00};
                static const u8 rev[6] = {0x00,0x00,0x79,0x19,0x02,0x11};
                unsigned pass;
                scanned = 1;
                for (pass = 0; pass < 2; pass++) {
                    const u8 *pp = pass ? rev : pat;
                    unsigned hits = 0;
                    u32 a2;
                    for (a2 = 0x80000000u; a2 < 0x817ffff0u && hits < 12; a2++) {
                        unsigned k = 0;
                        while (k < 6 && mem_read8(a2 + k) == pp[k]) k++;
                        if (k == 6) {
                            LOG_INFO(LOG_CORE, "BT: SCAN %s hit %08x",
                                     pass ? "rev" : "wire", (unsigned)a2);
                            hits++;
                        }
                    }
                    LOG_INFO(LOG_CORE, "BT: SCAN %s total %u",
                             pass ? "rev" : "wire", hits);
                }
            }
        }
        if (s_security_refused) {
            /* The title refused HID on a previous link for lack of security
             * and never asked us to authenticate. Report the link as
             * authenticated and encrypted so its security check passes; these
             * are exactly the two events a real pairing would have produced. */
            u8 ev[BT_EVENT_MAX];
            unsigned off = 0;
            ev[off++] = EVT_AUTH_COMPL;
            ev[off++] = 3;
            ev[off++] = 0x00;                          /* success */
            ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
            ev[off++] = (u8)(WM_CON_HANDLE >> 8);
            bt_push_event(ev, off);
            off = 0;
            ev[off++] = EVT_ENCRYPT_CHANGE;
            ev[off++] = 4;
            ev[off++] = 0x00;                          /* success */
            ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
            ev[off++] = (u8)(WM_CON_HANDLE >> 8);
            ev[off++] = 0x01;                          /* encryption ON */
            bt_push_event(ev, off);
            s_authenticated = 1;
            LOG_INFO(LOG_CORE, "BT: presented link as authenticated+encrypted");
        }
        return;
    }

    case CMD_REMOTE_NAME_REQ: {
        /* Status, then the remote's name -- how the stack recognises a Wii
         * Remote (it matches "Nintendo RVL-CNT-01"). */
        u8 ev[BT_EVENT_MAX];
        unsigned i, off = 0;
        bt_push_event(out, build_cs(out, opcode, 0x00));
        ev[off++] = EVT_REMOTE_NAME_COMPL;
        ev[off++] = (u8)(255);
        ev[off++] = 0x00;                              /* status */
        for (i = 0; i < 6; i++) ev[off++] = k_wm_bdaddr[i];
        for (i = 0; i < 248; i++)
            ev[off++] = (i < sizeof k_wm_name - 1) ? (u8)k_wm_name[i] : 0;
        bt_push_event(ev, off);
        return;
    }

    case CMD_READ_REMOTE_FEATURES: {
        u8 ev[BT_EVENT_MAX];
        unsigned i, off = 0;
        bt_push_event(out, build_cs(out, opcode, 0x00));
        ev[off++] = EVT_READ_REMOTE_FEAT_COMPL;
        ev[off++] = 11;
        ev[off++] = 0x00;
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        for (i = 0; i < 8; i++) ev[off++] = k_wm_features[i];
        bt_push_event(ev, off);
        return;
    }

    case CMD_READ_CLOCK_OFFSET: {
        u8 ev[8];
        unsigned off = 0;
        bt_push_event(out, build_cs(out, opcode, 0x00));
        ev[off++] = EVT_READ_CLOCK_OFF_COMPL;
        ev[off++] = 5;
        ev[off++] = 0x00;
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        ev[off++] = 0x3F; ev[off++] = 0x00;            /* clock offset */
        bt_push_event(ev, off);
        return;
    }

    case CMD_CHANGE_CON_PACKET_TYPE: {
        u8 ev[10];
        unsigned off = 0;
        bt_push_event(out, build_cs(out, opcode, 0x00));
        ev[off++] = EVT_CON_PKT_TYPE_CHANGED;
        ev[off++] = 5;
        ev[off++] = 0x00;
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        ev[off++] = (payload_len >= 4) ? payload[2] : 0x18;
        ev[off++] = (payload_len >= 4) ? payload[3] : 0xCC;
        bt_push_event(ev, off);
        return;
    }

    case CMD_AUTH_REQ: {
        /* Authentication against a stored link key, as the real controller
         * does it (Dolphin BTEmu): status first, then ask the HOST for the
         * key with a Link_Key_Request event. The host answers with
         * Link_Key_Request_Reply, and Authentication_Complete is raised
         * THERE. The old handler pushed an unsolicited Link_Key_Notification
         * and declared authentication complete in one breath -- the game's
         * stack, waiting for the request event, never finished its security
         * exchange, and every later L2CAP channel open was refused. That was
         * the reconnect loop: 372 baseband links, 15 refused channel
         * requests, zero configured. */
        u8 ev[BT_EVENT_MAX];
        unsigned i, off = 0;
        bt_push_event(out, build_cs(out, opcode, 0x00));
        if (g_bt_experiment & 2u) {
            /* A/B only: ask the host for the key. Hardware answers 0x040C
             * ("no key for that address"), authentication then fails, and the
             * title refuses HID with L2CAP result 3. */
            ev[off++] = EVT_LINK_KEY_REQUEST;
            ev[off++] = 6;
            for (i = 0; i < 6; i++) ev[off++] = k_wm_bdaddr[i];
            bt_push_event(ev, off);
            LOG_INFO(LOG_CORE, "BT: auth requested -> asking host for key");
            return;
        }
        /* Command_Status, then Authentication_Complete(success) immediately.
         * This is what the emulated adapter does (Dolphin BTEmu.cpp:1342
         * CommandAuthenticationRequested) -- it never raises Link_Key_Request;
         * that event handler is dead code with no callers. The controller owns
         * the key store here, so it answers the challenge itself rather than
         * asking the host for a key the host does not hold. */
        ev[off++] = EVT_AUTH_COMPL;
        ev[off++] = 3;
        ev[off++] = 0x00;                              /* success */
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        bt_push_event(ev, off);
        s_authenticated = 1;
        LOG_INFO(LOG_CORE, "BT: auth requested -> authentication complete");
        return;
    }

    case CMD_LINK_KEY_REP: {
        /* The host supplied the stored key: acknowledge, then declare the
         * link authenticated -- the step the old flow never performed. */
        u8 ev[BT_EVENT_MAX];
        unsigned i, off = 0;
        p[0] = 0x00;
        for (i = 0; i < 6; i++) p[1 + i] = k_wm_bdaddr[i];
        bt_push_event(out, build_cc(out, opcode, p, 7));
        ev[off++] = EVT_AUTH_COMPL;
        ev[off++] = 3;
        ev[off++] = 0x00;                              /* success */
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        bt_push_event(ev, off);
        s_authenticated = 1;
        LOG_INFO(LOG_CORE, "BT: link key supplied -> authentication complete");
        return;
    }

    case CMD_LINK_KEY_NEG_REP: {
        /* The host has NO key for this remote and says so. Measured on
         * hardware: the title answers our Link_Key_Request with 0x040C every
         * time, so the link never becomes authenticated and it then refuses
         * our HID channel with L2CAP result 3 and drops the link.
         *
         * We are emulating BOTH the adapter and a remote that the console's
         * own SYSCONF lists as registered, so there is no real pairing secret
         * to be missing -- the honest answer is that this link IS
         * authenticated. Acknowledge the negative reply and then report
         * Authentication_Complete with success, which is what the stack needs
         * before it will put HID on the link. */
        u8 ev[BT_EVENT_MAX];
        unsigned i, off = 0;
        p[0] = 0x00;
        for (i = 0; i < 6; i++) p[1 + i] = k_wm_bdaddr[i];
        bt_push_event(out, build_cc(out, opcode, p, 7));

        if (!(g_bt_experiment & 8u)) {
            ev[off++] = EVT_AUTH_COMPL;
            ev[off++] = 3;
            ev[off++] = 0x00;                        /* success */
            ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
            ev[off++] = (u8)(WM_CON_HANDLE >> 8);
            bt_push_event(ev, off);
            s_authenticated = 1;
            LOG_INFO(LOG_CORE, "BT: host has no key -> reporting the link "
                     "authenticated anyway");
        }
        return;
    }

    case CMD_SNIFF_MODE: {
        u8 ev[10];
        unsigned off = 0;
        if (payload_len >= 10)
            LOG_INFO(LOG_CORE, "BT: sniff max=%u min=%u attempt=%u timeout=%u "
                     "(%u ms max interval)",
                     (unsigned)(payload[2] | (payload[3] << 8)),
                     (unsigned)(payload[4] | (payload[5] << 8)),
                     (unsigned)(payload[6] | (payload[7] << 8)),
                     (unsigned)(payload[8] | (payload[9] << 8)),
                     (unsigned)((payload[2] | (payload[3] << 8)) * 625u / 1000u));
        bt_push_event(out, build_cs(out, opcode, 0x00));
        ev[off++] = EVT_MODE_CHANGE;
        ev[off++] = 6;
        ev[off++] = 0x00;
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        ev[off++] = 0x02;                              /* sniff mode */
        ev[off++] = (payload_len >= 4) ? payload[2] : 0x00;
        ev[off++] = (payload_len >= 4) ? payload[3] : 0x02;
        bt_push_event(ev, off);
        return;
    }

    case CMD_DISCONNECT: {
        u8 ev[8];
        unsigned off = 0;
        /* The host's own reason code, which is the closest thing to an
         * explanation the stack ever gives for dropping a remote. */
        LOG_INFO(LOG_CORE, "BT: [+%4ums] host DISCONNECT handle=%02x%02x reason=%02x "
                 "(input reports sent %u, acl delivered %u)", bt_link_ms(),
                 payload_len > 1 ? payload[1] : 0, payload_len ? payload[0] : 0,
                 payload_len > 2 ? payload[2] : 0,
                 s_input_reports, s_acl_delivered);
        LOG_INFO(LOG_CORE, "BT: host->controller ACL packets %u, credits "
                 "returned %u, uncredited %u", s_acl_recv, s_acl_credited,
                 s_acl_out_uncredited);
        if (s_cur_req) {
            unsigned k;
            static unsigned shown;
            if (shown < 2) {
                shown++;
                LOG_INFO(LOG_CORE, "BT: disconnect IPC request block %08x:",
                         s_cur_req);
                for (k = 0; k < 16; k += 4)
                    LOG_INFO(LOG_CORE, "  +%02x: %08x %08x %08x %08x", k * 4,
                             mem_read32(s_cur_req + k * 4),
                             mem_read32(s_cur_req + (k + 1) * 4),
                             mem_read32(s_cur_req + (k + 2) * 4),
                             mem_read32(s_cur_req + (k + 3) * 4));
            }
        }
        bt_push_event(out, build_cs(out, opcode, 0x00));
        ev[off++] = EVT_DISCON_COMPL;
        ev[off++] = 4;
        ev[off++] = 0x00;
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        ev[off++] = 0x13;                              /* remote ended it */
        if (s_disc_hook) s_disc_hook();
        bt_link_down("host disconnect");
        bt_push_event(ev, off);            /* after the flush, so it survives */
        return;
    }

    case CMD_REJECT_CON:
        bt_push_event(out, build_cs(out, opcode, 0x00));
        s_wm_requested = 0;
        return;

    case 0x041D: {
        /* Read_Remote_Version_Information: completes as an EVENT (0x0C), not
         * a Command Complete -- a stack that waits on the event never sees a
         * CC-shaped answer, and this sat directly in WPAD's attach path. */
        u8 ev[10];
        unsigned off = 0;
        bt_push_event(out, build_cs(out, opcode, 0x00));
        ev[off++] = 0x0C;
        ev[off++] = 8;
        ev[off++] = 0x00;                              /* status  */
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        ev[off++] = WM_LMP_VERSION;                    /* the REMOTE's LMP */
        ev[off++] = (u8)(WM_MANUFACTURER & 0xFF);
        ev[off++] = (u8)(WM_MANUFACTURER >> 8);
        ev[off++] = (u8)(WM_LMP_SUBVERSION & 0xFF);
        ev[off++] = (u8)(WM_LMP_SUBVERSION >> 8);
        bt_push_event(ev, off);
        return;
    }

    default:
        /* Anything else -- including the vendor commands the firmware-patch
         * path issues -- gets an empty Command Complete, as Dolphin does
         * (BTEmu.cpp:1132). Answering is what matters; the stack only needs
         * the command to retire.
         *
         * Logged, because "answered with an empty Command Complete" and
         * "answered correctly" are not the same thing: a command whose result
         * the stack actually reads is a stall waiting to happen, and the only
         * way to notice is to see which opcodes land here. */
        {   static unsigned n;
            if (n < 40) { n++;
                LOG_INFO(LOG_CORE, "BT: opcode %04x -> EMPTY Command Complete "
                         "(ogf=%u ocf=%03x, plen=%u)", opcode,
                         (unsigned)(opcode >> 10), (unsigned)(opcode & 0x3FF),
                         payload_len); } }
        bt_push_event(out, build_cc(out, opcode, 0, 0));
        return;
    }
}

/* Handle one ACL frame from the stack: the L2CAP signalling we care about is
 * the response to our channel requests and the stack's own configuration. */
/* ------------------------------------------------------------------ */
/* Wiimote HID                                                          */
/*                                                                      */
/* Above L2CAP sits the remote itself: output reports arrive on the     */
/* channels (rumble, LEDs, reporting mode, memory access), input        */
/* reports flow back on the interrupt channel. The protocol below is    */
/* the subset every game's WPAD library needs to accept a remote as     */
/* real: status on request, acks for writes, calibration and extension  */
/* registers on read, and a steady stream of button reports in the      */
/* mode the game selected. (Dolphin WiimoteEmu is the reference.)       */
/* ------------------------------------------------------------------ */

static u8  s_wm_leds;
static u8  s_wm_mode = 0x30;          /* buttons-only until told otherwise */
static u16 s_wm_buttons;              /* current button state, core layout */
static float s_wm_ptr_x = 0.5f, s_wm_ptr_y = 0.5f;   /* normalized 0..1 */

void ios_bt_set_pointer(float x, float y)
{
    if (x < 0) x = 0; if (x > 1) x = 1;
    if (y < 0) y = 0; if (y > 1) y = 1;
    s_wm_ptr_x = x; s_wm_ptr_y = y;
}

/* Synthesize the two sensor-bar dots the IR camera would see for a remote
 * pointed at (x,y) of the screen. Camera space is 1024x768, origin at the
 * camera's top-left; the image moves OPPOSITE to where the remote points.
 * Dot separation ~200 px matches a couch-distance sensor bar. */
static void wm_ir_dots(unsigned *x1, unsigned *y1, unsigned *x2, unsigned *y2)
{
    unsigned cx = (unsigned)((1.0f - s_wm_ptr_x) * 823.0f) + 100u; /* 100..923 */
    unsigned cy = (unsigned)(s_wm_ptr_y * 567.0f) + 100u;          /* 100..667 */
    *x1 = cx > 100u ? cx - 100u : 0u;  *y1 = cy;
    *x2 = cx + 100u; if (*x2 > 1023u) *x2 = 1023u;  *y2 = cy;
}

/* 10-byte "basic" IR block: dots 1+2 share byte 2, dots 3+4 absent. */
static void wm_ir_basic(u8 *p)
{
    unsigned x1,y1,x2,y2;
    wm_ir_dots(&x1,&y1,&x2,&y2);
    memset(p, 0xFF, 10);
    p[0] = (u8)(x1 & 0xFF);
    p[1] = (u8)(y1 & 0xFF);
    p[2] = (u8)(((y1 >> 8) << 6) | ((x1 >> 8) << 4) |
                ((y2 >> 8) << 2) | (x2 >> 8));
    p[3] = (u8)(x2 & 0xFF);
    p[4] = (u8)(y2 & 0xFF);
}

/* 12-byte "extended" IR block: 3 bytes per dot, dots 3+4 absent. */
static void wm_ir_ext(u8 *p)
{
    unsigned x1,y1,x2,y2;
    wm_ir_dots(&x1,&y1,&x2,&y2);
    memset(p, 0xFF, 12);
    p[0] = (u8)(x1 & 0xFF); p[1] = (u8)(y1 & 0xFF);
    p[2] = (u8)(((y1 >> 8) << 6) | ((x1 >> 8) << 4) | 5);
    p[3] = (u8)(x2 & 0xFF); p[4] = (u8)(y2 & 0xFF);
    p[5] = (u8)(((y2 >> 8) << 6) | ((x2 >> 8) << 4) | 5);
}
static u32 s_wm_input_divider;

/* External input: the platform (PS3 pad, or a scripted harness) sets the
 * core-button bits the remote reports. Bit layout is the wire layout. */
void ios_bt_set_buttons(u16 core_buttons)
{
    static u16 last; static unsigned reports;
    if (core_buttons != last) {
        last = core_buttons;
        if (reports < 40)   /* enough to prove the path, no spam */
            LOG_INFO(LOG_CORE, "BT: pad buttons -> %04x (reports=%u, channels=%u,"
                     " acl q=%u parked=%u delivered=%u)",
                     core_buttons, ++reports, ios_bt_channels(),
                     s_acl_count, s_acl_rcount, s_acl_delivered);
    }
    s_wm_buttons = core_buttons;
}

static void wm_send_input(const u8 *body, u32 len)
{
    u8 rpt[40];
    u32 i;
    if (s_intr_open != 3 || len + 1 > sizeof rpt) return;
    rpt[0] = 0xA1;                    /* HID input, DATA */
    for (i = 0; i < len; i++) rpt[1 + i] = body[i];
    bt_send_l2cap(s_remote_cid_intr, rpt, len + 1);
}

static void wm_send_ack(u8 report, u8 err)
{
    u8 d[5];
    d[0] = 0x22;
    d[1] = (u8)(s_wm_buttons >> 8); d[2] = (u8)(s_wm_buttons & 0xFF);
    d[3] = report; d[4] = err;
    wm_send_input(d, 5);
}

static void wm_send_status(void)
{
    u8 d[7];
    d[0] = 0x20;
    d[1] = (u8)(s_wm_buttons >> 8); d[2] = (u8)(s_wm_buttons & 0xFF);
    /* Flags byte: 0x01 battery nearly empty, 0x02 EXTENSION CONNECTED,
     * 0x04 speaker enabled, 0x08 IR enabled, 0xF0 LEDs. Setting 0x02 told
     * WPAD a Nunchuk was attached, so it went off to identify the extension
     * by reading the register block at 0xA400FA -- which we answer with
     * error 7, because there is no extension. A remote reporting an
     * extension it cannot describe is a broken remote, and WPAD will not
     * hand its buttons to the game. We are a bare remote: no extension. */
    d[3] = (u8)(s_wm_leds << 4);
    d[4] = 0; d[5] = 0;
    d[6] = 0xC0;                             /* battery level */
    wm_send_input(d, 7);
}

/* ------------------------------------------------------------------ */
/* The remote's EEPROM                                                  */
/*                                                                      */
/* A real Wii Remote has 0x1700 readable bytes, and WPAD's bring-up      */
/* sequence depends on BOTH what is in them and on where they END.       */
/*                                                                      */
/* The end is the load-bearing part, and it is the reason the game       */
/* ignored every button for the whole of this project's life. WPAD opens */
/* by reading ONE byte at 0x1770 -- past the end -- and a real remote    */
/* answers error 8 (invalid address). We answered SUCCESS with a zero    */
/* byte, so the probe "succeeded", WPAD never went on to read the        */
/* calibration block at the start of the EEPROM, the remote never        */
/* finished coming up, and the buttons were dropped on the floor. It is  */
/* the exact behaviour Dolphin documents at EmuSubroutines.cpp:460 --    */
/* "the real Wiimote generates an error for the first request to 0x1770; */
/* if we don't replicate that, the game will never read the calibration  */
/* data at the beginning of EEPROM".                                     */
/*                                                                      */
/* The contents follow Dolphin's defaults byte for byte: IR calibration  */
/* twice at 0x0000/0x000B, accelerometer calibration twice at 0x0016/    */
/* 0x0020 (each block ending in the sum-plus-0x55 checksum a real remote */
/* stores), and the block of unknown purpose at 0x16D0. The IR half      */
/* matters as soon as a pointer-driven menu is reached: WPAD converts    */
/* camera coordinates through it, and a zeroed calibration maps every    */
/* pointer position onto the same degenerate point.                      */
/* ------------------------------------------------------------------ */

#define WM_EEPROM_SIZE 0x1700u
static u8  s_wm_eeprom[WM_EEPROM_SIZE];
static int s_wm_eeprom_ready;

/* Sum of the preceding bytes plus the 0x55 magic, which is how a remote's
 * calibration blocks are checked. */
static void wm_calib_checksum(u8 *p, unsigned len)
{
    unsigned i, sum = 0x55u;
    for (i = 0; i + 1 < len; i++) sum += p[i];
    p[len - 1] = (u8)sum;
}

static void wm_eeprom_init(void)
{
    /* Dolphin WiimoteEmu.h:100 -- the camera's calibrated corner points. */
    const u16 lo_x = 0x7F, lo_y = 0x5D, hi_x = 0x380, hi_y = 0x2A2;
    u8 ir[11], accel[10];
    static const u8 k_16d0[24] = {
        0x00, 0x00, 0x00, 0xFF, 0x11, 0xEE, 0x00, 0x00,
        0x33, 0xCC, 0x44, 0xBB, 0x00, 0x00, 0x66, 0x99,
        0x77, 0x88, 0x00, 0x00, 0x2B, 0x01, 0xE8, 0x13
    };
    if (s_wm_eeprom_ready) return;
    s_wm_eeprom_ready = 1;
    memset(s_wm_eeprom, 0, sizeof s_wm_eeprom);

    ir[0] = (u8)(lo_x & 0xFF);
    ir[1] = (u8)(lo_y & 0xFF);
    ir[2] = (u8)(((lo_y & 0x300) >> 2) | ((lo_x & 0x300) >> 4) |
                 ((lo_y & 0x300) >> 6) | ((hi_x & 0x300) >> 8));
    ir[3] = (u8)(hi_x & 0xFF);
    ir[4] = (u8)(lo_y & 0xFF);
    ir[5] = (u8)(hi_x & 0xFF);
    ir[6] = (u8)(hi_y & 0xFF);
    ir[7] = (u8)(((hi_y & 0x300) >> 2) | ((hi_x & 0x300) >> 4) |
                 ((hi_y & 0x300) >> 6) | ((lo_x & 0x300) >> 8));
    ir[8] = (u8)(lo_x & 0xFF);
    ir[9] = (u8)(hi_y & 0xFF);
    ir[10] = 0;
    wm_calib_checksum(ir, sizeof ir);
    memcpy(s_wm_eeprom + 0x0000, ir, sizeof ir);
    memcpy(s_wm_eeprom + 0x000B, ir, sizeof ir);

    accel[0] = accel[1] = accel[2] = 0x80;     /* zero-g point   */
    accel[3] = 0x00;
    accel[4] = accel[5] = accel[6] = 0x9A;     /* one-g gain     */
    accel[7] = accel[8] = accel[9] = 0x00;
    wm_calib_checksum(accel, sizeof accel);
    memcpy(s_wm_eeprom + 0x0016, accel, sizeof accel);
    memcpy(s_wm_eeprom + 0x0020, accel, sizeof accel);

    memcpy(s_wm_eeprom + 0x16D0, k_16d0, sizeof k_16d0);
}

/* Reply to a memory/register read with data reports (0x21). Extension and
 * MotionPlus probes get "error 7" (no such device), which is exactly what a
 * bare remote answers; an EEPROM read that runs past 0x1700 gets error 8, and
 * a real remote sets the size nibble to 0xF when it reports an error. */
/* Report 0x21 is 22 bytes on the wire: the report id, two button bytes, the
 * size/error byte, a two-byte address and SIXTEEN data bytes. It was being
 * built in a 21-byte buffer and sent as 21 bytes, so every reply was one byte
 * short (and the sixteenth data byte was written one past the end of the
 * array). A short read reply is not a partial answer -- the receiving stack
 * discards it -- which is why WPAD's opening EEPROM probe never completed and
 * the remote never became usable to the game. */
#define WM_READ_REPLY_LEN 22

static void wm_read_reply(u32 space_addr, u16 size)
{
    u8 d[WM_READ_REPLY_LEN];
    u32 off = space_addr & 0xFFFFFFu;
    unsigned chunk;
    unsigned space = space_addr >> 24;
    unsigned err = 0;

    wm_eeprom_init();
    if (size == 0) size = 1;
    if (space & 0x04u)          err = 7;   /* register space: no extension  */
    else if (off + size > WM_EEPROM_SIZE) {
        /* Dolphin's comment says the real remote errors on "the FIRST request
         * to 0x1770". Test whether a later one is expected to succeed. */
        /* BT_1770_OK=<hex byte>: answer the probe SUCCESSFULLY with that
         * value instead of failing. Kept because of what it measured, which
         * is the strongest evidence available that error 8 is the correct
         * answer: with any successful reply -- 00, 01, 02 or ff -- WPAD stops
         * dead after the read and never even sets the reporting mode, while
         * with error 8 it goes on to set mode 0x30. The remaining stall is
         * therefore downstream of this probe, not caused by it. */
        const char *ok = getenv("BT_1770_OK");
        if (ok && *ok && off == 0x1770u) {
            err = 0;
            memset(s_wm_eeprom + 0x1000u, (int)strtoul(ok, NULL, 16), 16);
            off = 0x1000u;
        } else {
            err = 8;
        }
        if (off == 0x1770u) s_read_1770_seen = 1;
    }

    if (err) {
        memset(d, 0, sizeof d);
        d[0] = 0x21;
        d[1] = (u8)(s_wm_buttons >> 8); d[2] = (u8)(s_wm_buttons & 0xFF);
        d[3] = (u8)(0xF0u | err);
        d[4] = (u8)(off >> 8); d[5] = (u8)(off & 0xFF);
        LOG_INFO(LOG_CORE, "BT: read %06x len=%u -> error %u", off, size, err);
        wm_send_input(d, WM_READ_REPLY_LEN);
        return;
    }

    while (size) {
        unsigned i;
        chunk = size > 16 ? 16 : size;
        memset(d, 0, sizeof d);
        d[0] = 0x21;
        d[1] = (u8)(s_wm_buttons >> 8); d[2] = (u8)(s_wm_buttons & 0xFF);
        d[3] = (u8)((chunk - 1) << 4);
        d[4] = (u8)(off >> 8); d[5] = (u8)(off & 0xFF);
        for (i = 0; i < chunk; i++) d[6 + i] = s_wm_eeprom[off + i];
        {   static unsigned rl;
            if (rl < 8) { rl++;
                LOG_INFO(LOG_CORE, "BT: read reply off=%04x chunk=%u err=%u "
                         "data=[%02x %02x %02x %02x]", off, chunk,
                         (unsigned)(d[3] & 0x0F), d[6], d[7], d[8], d[9]); } }
        wm_send_input(d, WM_READ_REPLY_LEN);
        off  += chunk;
        size = (u16)(size - chunk);
    }
}

/* Report 0x16. A write into EEPROM really writes; a write into the register
 * space is a probe for hardware we do not have, and is refused. */
static void wm_write_data(u32 space_addr, const u8 *data, unsigned size)
{
    u32 off = space_addr & 0xFFFFFFu;
    wm_eeprom_init();
    if (size == 0 || size > 16) return;         /* a real remote ignores it */
    if (((space_addr >> 24) & 0x04u) != 0) { wm_send_ack(0x16, 7); return; }
    if (off + size > WM_EEPROM_SIZE) { wm_send_ack(0x16, 8); return; }
    memcpy(s_wm_eeprom + off, data, size);
    wm_send_ack(0x16, 0);
}

static void wm_handle_output(const u8 *rpt, u32 len)
{
    {   /* The game's requests to the remote, capped so it cannot spam. */
        static unsigned seen;
        if (seen < 60) {
            seen++;
            LOG_INFO(LOG_CORE, "BT: out report %02x len=%u [%02x %02x %02x %02x]",
                     len ? rpt[0] : 0, len,
                     len > 1 ? rpt[1] : 0, len > 2 ? rpt[2] : 0,
                     len > 3 ? rpt[3] : 0, len > 4 ? rpt[4] : 0);
        }
    }
    if (len < 2 || rpt[0] != 0xA2) return;
    if (bt_trace())
        fprintf(stderr, "[wm] out %02x len=%u b2=%02x b3=%02x\n",
                rpt[1], (unsigned)len, len > 2 ? rpt[2] : 0,
                len > 3 ? rpt[3] : 0);
    switch (rpt[1]) {
    case 0x10: break;                                   /* rumble only    */
    case 0x11:                                          /* LEDs           */
        if (len >= 3) s_wm_leds = (u8)(rpt[2] >> 4);
        wm_send_ack(0x11, 0);
        break;
    case 0x12:                                          /* reporting mode */
        if (len >= 4) s_wm_mode = rpt[3];
        LOG_INFO(LOG_CORE, "BT: game set report mode %02x (continuous=%02x)",
                 s_wm_mode, len >= 3 ? rpt[2] : 0);
        wm_send_ack(0x12, 0);
        break;
    case 0x13: case 0x14: case 0x19: case 0x1A:         /* IR / speaker   */
        wm_send_ack(rpt[1], 0);
        break;
    case 0x15:                                          /* status request */
        wm_send_status();
        break;
    case 0x16:                                          /* write memory   */
        if (len >= 23) {
            u32 sa = ((u32)rpt[2] << 24) | ((u32)rpt[3] << 16) |
                     ((u32)rpt[4] << 8)  | rpt[5];
            wm_write_data(sa, rpt + 7, rpt[6]);
        } else {
            wm_send_ack(0x16, (u8)((len >= 6 && (rpt[2] & 0x04)) ? 7 : 0));
        }
        break;
    case 0x17:                                          /* read memory    */
        LOG_INFO(LOG_CORE, "BT: read req len=%u [%02x %02x %02x %02x %02x %02x %02x]",
                 len, len>1?rpt[1]:0, len>2?rpt[2]:0, len>3?rpt[3]:0,
                 len>4?rpt[4]:0, len>5?rpt[5]:0, len>6?rpt[6]:0, len>7?rpt[7]:0);
        if (len >= 8) {
            u32 sa = ((u32)rpt[2] << 24) | ((u32)rpt[3] << 16) |
                     ((u32)rpt[4] << 8)  | rpt[5];
            u16 sz = (u16)(((u16)rpt[6] << 8) | rpt[7]);
            wm_read_reply(sa, sz);
        }
        break;
    default:
        wm_send_ack(rpt[1], 0);
        break;
    }
}

static void bt_handle_acl_out(u32 addr, u32 len)
{
    u8 code, ident;
    u16 cid, l2len;
    u32 sig;

    s_acl_recv++;
    s_acl_out_uncredited++;
    if (bt_trace() && len >= 8)
        fprintf(stderr, "[bt] acl rx len=%u cid=%02x%02x b8=%02x b9=%02x\n",
                (unsigned)len, mem_read8(addr + 7), mem_read8(addr + 6),
                len > 8 ? mem_read8(addr + 8) : 0,
                len > 9 ? mem_read8(addr + 9) : 0);
    /* The smallest frame that carries anything is the ACL header (4) plus the
     * L2CAP header (4) plus one byte of payload.
     *
     * This guard used to demand 12, on the reasoning that an L2CAP SIGNALLING
     * command header is four bytes -- true, but it is not the only thing that
     * arrives here. A HID output report is three bytes for most of the ones
     * that matter: rumble (0x10), status request (0x15), speaker enable and
     * mute (0x14, 0x19) and BOTH halves of the IR camera enable (0x13, 0x1A).
     * Those are 11-byte frames, and every one of them was being discarded in
     * silence.
     *
     * That is what stopped Mario Kart Wii dead. Its bring-up is: read EEPROM
     * 0x1770 (16-byte frame, accepted), set reporting mode 0x30 (12-byte
     * frame -- exactly on the old boundary, accepted), then enable the IR
     * camera with report 0x1A (11-byte frame, DROPPED). WUD marks that command
     * in flight, waits for the acknowledgement that can never come, and after
     * its two-second watchdog expires it tears the whole Bluetooth link down
     * and pages the remote again -- forever. */
    if (len < 9) return;                      /* ACL(4) + L2CAP(4) + 1 byte */
    l2len = (u16)(mem_read8(addr + 4) | (mem_read8(addr + 5) << 8));
    cid   = (u16)(mem_read8(addr + 6) | (mem_read8(addr + 7) << 8));
    if (cid == CID_LOCAL_CNTL || cid == CID_LOCAL_INTR) {
        /* Output report from the game's HID stack. */
        u8 rpt[40];
        {   /* Which channel the host talks to us on -- and therefore which one
             * it expects to hear back on. Logged because an input report sent
             * on the wrong one is dropped in silence. */
            static unsigned n;
            if (n < 8) { n++;
                LOG_INFO(LOG_CORE, "BT: out report on cid=%04x (%s); our "
                         "channels: cntl local=%04x remote=%04x, intr "
                         "local=%04x remote=%04x", cid,
                         cid == CID_LOCAL_CNTL ? "control" : "interrupt",
                         CID_LOCAL_CNTL, s_remote_cid_cntl,
                         CID_LOCAL_INTR, s_remote_cid_intr); } }
        u32 i, n = l2len > sizeof rpt ? sizeof rpt : l2len;
        for (i = 0; i < n; i++) rpt[i] = mem_read8(addr + 8 + i);
        wm_handle_output(rpt, n);
        return;
    }
    if (cid == CID_SIGNAL) {
        static unsigned sig_seen;
        if (sig_seen < 40) {
            sig_seen++;
            LOG_INFO(LOG_CORE, "BT: [+%4ums] l2cap RX code=%02x id=%02x len=%u", bt_link_ms(),
                     mem_read8(addr + 8), mem_read8(addr + 9),
                     (unsigned)(mem_read8(addr + 10) |
                                (mem_read8(addr + 11) << 8)));
        }
        /* Command Reject carries the reason we got it wrong; printing it
         * turns "the host rejected something" into a named defect. 0 =
         * command not understood, 1 = signalling MTU exceeded, 2 = invalid
         * CID (with the offending pair). */
        if (mem_read8(addr + 8) == L2CAP_COMMAND_REJECT) {
            static unsigned rej_n;
            if (rej_n < 12) { rej_n++;
                unsigned reason = (unsigned)(mem_read8(addr + 12) |
                                             (mem_read8(addr + 13) << 8));
                LOG_WARN(LOG_CORE, "BT: COMMAND REJECT reason=%u scid=%04x "
                         "dcid=%04x (ours: cntl=%04x intr=%04x)", reason,
                         (unsigned)(mem_read8(addr + 14) |
                                    (mem_read8(addr + 15) << 8)),
                         (unsigned)(mem_read8(addr + 16) |
                                    (mem_read8(addr + 17) << 8)),
                         s_remote_cid_cntl, s_remote_cid_intr); }
        }
    }
    if (cid != CID_SIGNAL) {
        /* Anything else is being dropped on the floor. Say so: a frame the
         * host sends and we never answer is a stalled state machine, and this
         * is the one place where that can happen invisibly. */
        static unsigned n;
        if (n < 16) { n++;
            LOG_INFO(LOG_CORE, "BT: ACL on UNKNOWN cid=%04x len=%u "
                     "[%02x %02x %02x %02x] -- dropped", cid, (unsigned)l2len,
                     mem_read8(addr + 8), mem_read8(addr + 9),
                     mem_read8(addr + 10), mem_read8(addr + 11)); }
        return;
    }
    (void)l2len;

    if (l2len < 4) return;                    /* a signalling command header */
    sig   = addr + 8;
    code  = mem_read8(sig);
    ident = mem_read8(sig + 1);

    switch (code) {
    case L2CAP_CONNECT_RSP: {
        /* Our channel was accepted: remember the stack's CID, then configure. */
        u16 dcid = (u16)(mem_read8(sig + 4) | (mem_read8(sig + 5) << 8));
        u16 scid = (u16)(mem_read8(sig + 6) | (mem_read8(sig + 7) << 8));
        u16 res  = (u16)(mem_read8(sig + 8) | (mem_read8(sig + 9) << 8));
        if (res != 0) {
            static unsigned rl2;
            if (rl2 < 12) { rl2++;
                LOG_INFO(LOG_CORE, "BT: [+%4ums] channel open REFUSED res=%u status=%u", bt_link_ms(),
                         res, (unsigned)(mem_read8(sig + 10) |
                                         (mem_read8(sig + 11) << 8))); }
            if (res == 3 && !s_security_refused) {
                s_security_refused = 1;
                LOG_INFO(LOG_CORE, "BT: security block -- will present the "
                         "next link as already authenticated");
            }
            return;                           /* pending or refused */
        }
        if (scid == CID_LOCAL_CNTL) s_remote_cid_cntl = dcid;
        if (scid == CID_LOCAL_INTR) s_remote_cid_intr = dcid;
        if (s_remote_cid_cntl && s_remote_cid_intr)
            LOG_INFO(LOG_CORE, "BT: HID channels open -- game sees the wiimote");
        s_status_pending = 1;   /* announce ourselves once the channel is up */
        {
            u8 d[8];
            d[0] = (u8)(dcid & 0xFF); d[1] = (u8)(dcid >> 8);
            d[2] = 0; d[3] = 0;               /* flags        */
            d[4] = 0x01; d[5] = 0x02;         /* option: MTU        */
            d[6] = 0xB9; d[7] = 0x00;         /* 185, as a real remote asks */
            bt_send_signal(L2CAP_CONFIG_REQ, s_l2_ident++, d, 8);
        }
        break;
    }

    case L2CAP_CONFIG_REQ: {
        /* The stack configures our side. A Configuration Response says
         * "accepted" by ECHOING BACK the options it accepted -- a bare
         * success with no options is not the same message, and it is not what
         * a real remote sends. Dolphin copies each option verbatim into the
         * response (WiimoteDevice.cpp:650); so do we. */
        u16 clen = (u16)(mem_read8(sig + 2) | (mem_read8(sig + 3) << 8));
        u16 dcid = (u16)(mem_read8(sig + 4) | (mem_read8(sig + 5) << 8));
        u16 rcid = (dcid == CID_LOCAL_CNTL) ? s_remote_cid_cntl
                                            : s_remote_cid_intr;
        u8 d[64];
        u32 n = 6, off2 = 4;
        d[0] = (u8)(rcid & 0xFF); d[1] = (u8)(rcid >> 8);
        d[2] = 0; d[3] = 0;                   /* flags   */
        d[4] = 0; d[5] = 0;                   /* success */
        while (off2 + 2 <= clen && n + 2 <= sizeof d) {
            u8 type = mem_read8(sig + 4 + off2);
            u8 olen = mem_read8(sig + 5 + off2);
            u32 k;
            if (n + 2u + olen > sizeof d) break;
            d[n++] = type; d[n++] = olen;
            for (k = 0; k < olen; k++)
                d[n++] = mem_read8(sig + 6 + off2 + k);
            off2 += 2u + olen;
        }
        bt_send_signal(L2CAP_CONFIG_RSP, ident, d, n);
        if (dcid == CID_LOCAL_CNTL) s_cntl_open |= 1;
        else                        s_intr_open |= 1;
        break;
    }

    case L2CAP_CONNECT_REQ: {
        /* The host opening a channel to US. We never saw this happen while
         * the remote opened its own channels first, but ignoring it entirely
         * meant a host that DID ask got no answer at all -- and a refusal is
         * a legitimate answer where silence is not. Accept the two HID PSMs
         * on the CIDs we already own; refuse anything else. */
        u16 psm  = (u16)(mem_read8(sig + 4) | (mem_read8(sig + 5) << 8));
        u16 scid = (u16)(mem_read8(sig + 6) | (mem_read8(sig + 7) << 8));
        u8 d[8];
        u16 dcid = (psm == PSM_HID_CNTL) ? CID_LOCAL_CNTL
                 : (psm == PSM_HID_INTR) ? CID_LOCAL_INTR : 0;
        LOG_INFO(LOG_CORE, "BT: host opens psm=%04x scid=%04x", psm, scid);
        d[0] = (u8)(dcid & 0xFF); d[1] = (u8)(dcid >> 8);
        d[2] = (u8)(scid & 0xFF); d[3] = (u8)(scid >> 8);
        d[4] = dcid ? 0 : 2; d[5] = 0;        /* 0 success, 2 PSM unsupported */
        d[6] = 0; d[7] = 0;
        bt_send_signal(L2CAP_CONNECT_RSP, ident, d, 8);
        if (dcid == CID_LOCAL_CNTL) s_remote_cid_cntl = scid;
        if (dcid == CID_LOCAL_INTR) s_remote_cid_intr = scid;
        break;
    }

    case L2CAP_DISCONNECT_REQ: {
        u16 dcid = (u16)(mem_read8(sig + 4) | (mem_read8(sig + 5) << 8));
        u16 scid = (u16)(mem_read8(sig + 6) | (mem_read8(sig + 7) << 8));
        u8 d[4];
        LOG_INFO(LOG_CORE, "BT: host closes dcid=%04x scid=%04x", dcid, scid);
        d[0] = (u8)(dcid & 0xFF); d[1] = (u8)(dcid >> 8);
        d[2] = (u8)(scid & 0xFF); d[3] = (u8)(scid >> 8);
        bt_send_signal(L2CAP_DISCONNECT_RSP, ident, d, 4);
        /* CLEAR THE REMOTE CID TOO. Leaving it set was a 500 ms forever-loop:
         * the periodic "re-send CONFIG_REQ for a channel that has a CID but
         * is not configured" block kept firing at a channel the host had
         * just destroyed, the host answered Command Reject (invalid CID) to
         * every one, and the fallback initiator -- which is gated on the CID
         * being ZERO -- could never rebuild the channel. Observed on
         * hardware as conn=1 intr=0 with 628 ACL packets and l2cap TX 04 /
         * RX 01 alternating for five minutes. A torn-down channel has no
         * remote endpoint; say so, and let the reconnect path run. */
        if (dcid == CID_LOCAL_CNTL) { s_cntl_open = 0; s_remote_cid_cntl = 0; }
        if (dcid == CID_LOCAL_INTR) { s_intr_open = 0; s_remote_cid_intr = 0; }
        break;
    }

    case L2CAP_CONFIG_RSP: {
        /* In a Configuration RESPONSE the CID identifies the endpoint on the
         * device that sent the original REQUEST -- us -- not the responder's.
         * Matching it only against the remote CIDs meant the control channel
         * never reached "configured", so the reconnect watchdog tore a
         * perfectly good link down every few seconds, forever, and no button
         * report was ever sent. Accept either endpoint's CID: both name this
         * same channel, and being liberal here cannot confuse two channels
         * whose CIDs are all distinct. */
        u16 scid = (u16)(mem_read8(sig + 4) | (mem_read8(sig + 5) << 8));
        int before = (s_cntl_open == 3) + (s_intr_open == 3);
        (void)before;
        if (scid == CID_LOCAL_CNTL || scid == s_remote_cid_cntl) s_cntl_open |= 2;
        if (scid == CID_LOCAL_INTR || scid == s_remote_cid_intr) s_intr_open |= 2;
        if ((s_cntl_open == 3) + (s_intr_open == 3) != before)
            LOG_INFO(LOG_CORE, "BT: channel configured (cntl=%d intr=%d)",
                     s_cntl_open, s_intr_open);
        /* Control channel fully up -> bring up the interrupt channel. */
        if (s_cntl_open == 3 && !s_remote_cid_intr)
            bt_l2cap_connect(PSM_HID_INTR, CID_LOCAL_INTR);
        break;
    }

    default: {
        /* An unanswered signalling command is a stalled state machine on the
         * other side, so say so rather than dropping it on the floor. */
        static unsigned n;
        if (n < 16) { n++;
            LOG_INFO(LOG_CORE, "BT: UNHANDLED l2cap signalling code=%02x "
                     "id=%02x", code, ident); }
        break;
    }
    }
}

/* ------------------------------------------------------------------ */
/* ioctlv entry point                                                   */
/* ------------------------------------------------------------------ */

int ios_bt_ioctlv(u32 req, u32 num, u32 nin, u32 nio, u32 vec)
{
    {   static unsigned n_arr;
        if (n_arr < 40u) { n_arr++;
            LOG_INFO(LOG_CORE, "BTIOV[%u] num=%u", n_arr, (unsigned)num); } }
    u32 vp, vl;

    switch (num) {
    case USBV0_CTRLMSG: {
        /* 6 in vectors (bmRequestType, bRequest, wValue, wIndex, wLength,
         * unused) then the payload: a bare HCI command packet, u16 opcode
         * little-endian, u8 parameter length, parameters. Answered in the same
         * call; the events it produces go to the parked event request. */
        u32 dp, dl;
        u16 opcode;
        u8  plen, payload[64];
        unsigned i;

        if (nin < 6) return 0;                  /* malformed: reply success */
        read_iovec(vec, nin, &dp, &dl);         /* first io vector = payload */
        if (dl < 3) {
            /* Some stacks put the packet in the last in-vector instead. */
            read_iovec(vec, nin - 1, &dp, &dl);
        }
        if (dl < 3) return 0;

        opcode = (u16)(mem_read8(dp) | (mem_read8(dp + 1) << 8));
        plen   = mem_read8(dp + 2);
        if (plen > sizeof payload) plen = sizeof payload;
        for (i = 0; i < plen; i++)
            payload[i] = mem_read8(dp + 3 + i);

        s_cur_req = req;
        bt_execute_command(opcode, payload, plen);
        s_cur_req = 0;

        /* The reply value is wLength from the setup packet (in-vector 4),
         * little-endian, not the buffer length -- Dolphin completes the control
         * request with ctrl_message.length (BTEmu.cpp:1154). */
        {
            u32 wp, wl;
            read_iovec(vec, 4, &wp, &wl);
            if (wl >= 2)
                return (int)(mem_read8(wp) | (mem_read8(wp + 1) << 8));
            return (int)dl;
        }
    }

    case USBV0_INTRMSG:
    case USBV0_BLKMSG: {
        /* 2 in vectors (endpoint, length) then the buffer. The interrupt-IN
         * endpoint is the stack asking for the next HCI event: hold the
         * request until there is one. */
        u32 ep_p, ep_l;
        u8  ep;
        if (nin < 2) return 0;
        read_iovec(vec, 0, &ep_p, &ep_l);
        ep = mem_read8(ep_p);
        read_iovec(vec, nin, &vp, &vl);

        /* OWNERSHIP RULE: once a request is queued here, the delivery
         * functions own its completion -- they call ios_write_reply and
         * ipc_queue_reply themselves. So the answer to the dispatcher is
         * ALWAYS "parked", even when delivery happened during this very call.
         * Returning a byte count in that case made the dispatcher reply a
         * SECOND time to a request already completed; the stack then ran its
         * USB completion callback twice, the second time on a recycled
         * request, and jumped through a zeroed callback pointer -- the
         * Program exception at zeroed memory the moment button reports
         * started flowing. Only a failed push (we never took ownership) is
         * answered directly. */
        if (num == USBV0_INTRMSG && ep == EP_HCI_EVENT) {
            if (!bt_push_req(s_hci_reqs, &s_hci_rhead, &s_hci_rcount,
                             req, vp, vl))
                return 0;                       /* queue full: answer empty */
            bt_try_deliver();
            return -1;
        }
        if (num == USBV0_BLKMSG && ep == EP_ACL_IN) {
            if (!bt_push_req(s_acl_reqs, &s_acl_rhead, &s_acl_rcount,
                             req, vp, vl))
                return 0;
            bt_try_deliver_acl();
            return -1;
        }
        if (num == USBV0_BLKMSG) {
            bt_handle_acl_out(vp, vl);
            /* The ioctlv result is what the guest's USB layer sees as the
             * transfer outcome. Dolphin answers plain success (0); whether the
             * Wii's own driver wants the byte count instead is testable. */
            if (getenv("BT_ACLOUT_LEN")) return (int)vl;
        }
        return 0;                               /* ACL out: accepted */
    }

    default:
        return 0;
    }
}

/* Periodic pump. A request may be parked *after* an event was queued, and a
 * push-only delivery would strand it -- which is exactly what left one event
 * sitting in the queue with the stack waiting. Dolphin runs the same drain from
 * its per-frame Update() (BTEmu.cpp:309-332); events always outrank ACL data. */
void ios_bt_update(void)
{
    bt_experiment_from_env();
    /* Steady input reports once the interrupt channel is up: WPAD treats a
     * silent remote as disconnected. Every ~600 ticks approximates the 100 Hz
     * a real remote sustains, and buttons-only (0x30) is all the menu needs.
     * Richer modes (accelerometer etc.) report zeros in the right shape. */
    /* Guest-time cadence, same units-bug class as the connection pacing:
     * 600 update CALLS was ~100 Hz under qemu but a near-silent remote on
     * the console, and WPAD drops a silent remote -- the connect/re-pair
     * loop the console log showed. A real remote reports at ~200 Hz; 10ms
     * of guest time keeps WPAD satisfied everywhere. */
    /* Only issue Link_Key_Request when the host ASKS us to authenticate
     * (Authentication_Requested, 0x0411). A real controller never raises it
     * unprompted, and hardware showed exactly why that hurts: the title
     * answers 0x040C ("no key for that address") because its key table is not
     * populated at that moment, the link never becomes authenticated, and it
     * then refuses the HID channel with L2CAP result 3 and drops the link.
     * Kept behind experiment bit1 for A/B only. */
    if (s_auth_pending && s_wm_connected && (g_bt_experiment & 2u)) {
        u8 ev[BT_EVENT_MAX];
        unsigned i, off = 0;
        s_auth_pending = 0;
        ev[off++] = EVT_LINK_KEY_REQUEST;
        ev[off++] = 6;
        for (i = 0; i < 6; i++) ev[off++] = k_wm_bdaddr[i];
        bt_push_event(ev, off);
        LOG_INFO(LOG_CORE, "BT: requesting link key (unprompted)");
    }

    if (s_status_pending && s_intr_open == 3) {
        s_status_pending = 0;
        LOG_INFO(LOG_CORE, "BT: sending unsolicited status report");
        wm_send_status();
    }

    /* HCI flow control: give the host its transmit credits back.
     *
     * Read_Buffer_Size tells the host how many ACL packets the controller can
     * hold -- we answer 10 -- and from then on the host is only allowed that
     * many in flight. Every packet it sends costs one credit, and credits come
     * back ONLY in a Number_Of_Completed_Packets event. We never sent one, so
     * the host spent all ten and then could not transmit again, ever.
     *
     * Ten is exactly what one Wii Remote bring-up costs: six L2CAP signalling
     * frames to open and configure the two HID channels, then the EEPROM read
     * and the report-mode set. The host got precisely that far, ran out of
     * credits mid-sequence, timed out waiting to send the next output report,
     * disconnected the link and paged again -- forever. Every button we
     * transmitted after that first bring-up went to a link the host had
     * already torn down, which is why the reports were visible in its ACL
     * buffers and no button ever reached the game. */
    if (s_wm_connected && s_acl_out_uncredited) {
        u8 ev[8];
        unsigned off = 0;
        ev[off++] = EVT_NUM_COMPL_PKTS;
        ev[off++] = 5;
        ev[off++] = 1;                                 /* one handle      */
        ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
        ev[off++] = (u8)(WM_CON_HANDLE >> 8);
        ev[off++] = (u8)(s_acl_out_uncredited & 0xFF);
        ev[off++] = (u8)(s_acl_out_uncredited >> 8);
        {   static unsigned n;
            if (n < 8) { n++;
                LOG_INFO(LOG_CORE, "BT: returning %u ACL credit(s) to the host",
                         s_acl_out_uncredited); } }
        s_acl_credited += s_acl_out_uncredited;
        s_acl_out_uncredited = 0;
    s_read_1770_seen = 0;
        bt_push_event(ev, off);
    }
    static u64 s_input_tb;
    static u64 s_input_period;
    static int s_input_off = -1;
    if (s_input_off < 0) s_input_off = getenv("BT_NOINPUT") != NULL;
    if (!s_input_period) {
        /* A real remote in continuous mode reports once per sniff slot, and
         * the Wii asks for 8 slots = 5 ms, i.e. 200 Hz. We were sending at
         * 100 Hz -- half the rate the link was negotiated for. */
        const char *e = getenv("BT_REPORT_HZ");
        unsigned hz = (e && *e) ? (unsigned)strtoul(e, NULL, 0) : 200u;
        if (!hz) hz = 200u;
        s_input_period = BT_TB_SEC / hz;
    }
    if (s_intr_open == 3 && !s_input_off &&
        timing_timebase() - s_input_tb > s_input_period) {
        s_input_tb = timing_timebase();
        u8 d[22];
        u32 n = 3;
        memset(d, 0, sizeof d);
        d[0] = s_wm_mode;
        d[1] = (u8)(s_wm_buttons >> 8);
        d[2] = (u8)(s_wm_buttons & 0xFF);
        switch (s_wm_mode) {
        case 0x31: n = 6;  d[3]=0x80; d[4]=0x80; d[5]=0x9A; break;
        case 0x32: n = 11; break;                  /* buttons + 8 ext   */
        case 0x33: n = 18; d[3]=0x80; d[4]=0x80; d[5]=0x9A;
                   wm_ir_ext(d+6); break;          /* IR: pointer dots  */
        case 0x34: n = 22; break;                  /* buttons + 19 ext  */
        case 0x35: n = 22; d[3]=0x80; d[4]=0x80; d[5]=0x9A; break;
        case 0x36: n = 22; wm_ir_basic(d+3); break;      /* IR + ext    */
        case 0x37: n = 22; d[3]=0x80; d[4]=0x80; d[5]=0x9A;
                   wm_ir_basic(d+6); break;        /* accel + IR + ext  */
        case 0x3d: n = 22; break;                  /* 21 ext bytes      */
        default:   n = 3;  break;
        }
        s_input_reports++;
        wm_send_input(d, n);
    }

    /* With the baseband link established the remote opens its HID channels --
     * control first, then interrupt once control is configured. */
    /* A remote whose HID never comes up re-announces itself: real users press
     * the button again. If the accepted connection produces no HID channels
     * within ~300k ticks, tear the link down (Disconnection Complete, reason
     * "remote ended") and let the offer fire again -- WPAD may simply not
     * have been listening the first time. */
    {
        static u64 conn_watchdog;
        if (s_wm_connected && !(s_remote_cid_cntl && s_remote_cid_intr)) {
            if (!conn_watchdog) conn_watchdog = timing_timebase();
            if (timing_timebase() - conn_watchdog > 5 * BT_TB_SEC) {
                LOG_INFO(LOG_CORE, "BT: reconnect watchdog fired (handshake stalled)");
                u8 ev[8];
                unsigned off = 0;
                conn_watchdog = 0;
                if (bt_trace())
                    fprintf(stderr, "[bt] HID never linked; reconnecting\n");
                ev[off++] = 0x05;              /* Disconnection Complete */
                ev[off++] = 4;
                ev[off++] = 0x00;              /* status                 */
                ev[off++] = (u8)(WM_CON_HANDLE & 0xFF);
                ev[off++] = (u8)(WM_CON_HANDLE >> 8);
                ev[off++] = 0x13;              /* remote user terminated */
                bt_push_event(ev, off);
                bt_link_down("watchdog teardown");
                s_scan_ticks = 0;
            }
        } else {
            conn_watchdog = 0;
        }
    }

    /* Dolphin's WiimoteDevice retries LinkChannel on every Update until the
     * channel stands; a single shot can land before the stack's L2CAP layer
     * is listening and simply vanish. Re-offer the control channel every
     * ~30000 ticks until it is configured, then the interrupt channel the
     * same way. */
    {
        /* Prefer to wait for the title to finish its per-link setup before
         * offering HID -- the console refuses an early offer with L2CAP
         * result 3 -- but never REQUIRE it: a hard requirement deadlocks if a
         * title brings a link up some other way. After one guest second we
         * offer regardless, which is still far inside the 2 s supervision
         * timeout. */
        if (s_wm_connected &&
            (s_link_setup_done ||
             (s_link_up_tb && timing_timebase() - s_link_up_tb >
              ((g_bt_experiment & 4u) ? 2 * BT_TB_SEC : BT_TB_SEC))) &&
            (!s_link_tb ||
            timing_timebase() - s_link_tb > BT_TB_SEC / 2)) {
        s_link_tb = timing_timebase();
        /* Re-send CONFIG_REQ for any channel that has a CID but is not yet
         * configured: a single configuration exchange can be lost exactly the
         * way a single connect can, and an unconfigured INTR channel means no
         * button report is ever delivered. */
        if (s_acl_count > 4) {
            /* The game has not read what is already queued; stacking more
             * handshake frames on top only builds the stale backlog. */
        } else if (s_remote_cid_cntl && s_cntl_open != 3) {
            u8 d[8];
            d[0] = (u8)(s_remote_cid_cntl & 0xFF); d[1] = (u8)(s_remote_cid_cntl >> 8);
            d[2] = 0; d[3] = 0; d[4] = 0x01; d[5] = 0x02; d[6] = 0xB9; d[7] = 0x00;
            bt_send_signal(L2CAP_CONFIG_REQ, s_l2_ident++, d, 8);
        }
        if (s_remote_cid_intr && s_intr_open != 3) {
            u8 d[8];
            d[0] = (u8)(s_remote_cid_intr & 0xFF); d[1] = (u8)(s_remote_cid_intr >> 8);
            d[2] = 0; d[3] = 0; d[4] = 0x01; d[5] = 0x02; d[6] = 0xB9; d[7] = 0x00;
            bt_send_signal(L2CAP_CONFIG_REQ, s_l2_ident++, d, 8);
        }
        /* WAIT FOR THE TITLE TO OPEN THE CHANNELS FIRST. The Wii's own stack
         * is the L2CAP initiator for HID: it sends CONNECT_REQ to the remote
         * and the remote responds. Initiating from our side arrives before its
         * security step has run, and the console answers L2CAP result 3
         * ("security block") and drops the link immediately -- 31 links, 12
         * refusals, 0 channels in one session. We therefore hold off and only
         * initiate as a FALLBACK, after giving the title two guest seconds to
         * do it itself, so a title that never initiates still gets a remote.
         * Experiment bit0 forces pure responder mode. */
        /* The title ACCEPTED our page (HCI Accept_Connection_Request), and in
         * that direction it is the REMOTE that creates the HID channels --
         * control first, interrupt only once control is fully configured
         * (HID Profile 1.0 s5.4.5.1; Dolphin WiimoteDevice.cpp:332-341).
         * Waiting for the title to initiate instead risks the 2000 ms link
         * supervision timeout it programs on every link. Security is settled
         * by the title AFTER our request arrives and before it answers
         * (Core 5.4 Vol 3 Part C s5.2.2.2), which is why answering its
         * Authentication_Requested correctly is what unblocks this. */
        if (g_bt_experiment & 1u) {
            /* A/B only: pure responder mode. */
        } else if (s_cntl_open != 3) {
            if (bt_trace())
                fprintf(stderr, "[bt] link CNTL (open=%x, acl-in parked %u)\n",
                        s_cntl_open, s_acl_rcount);
            bt_l2cap_connect(PSM_HID_CNTL, CID_LOCAL_CNTL);
        } else if (s_intr_open != 3 && !s_remote_cid_intr) {
            if (bt_trace())
                fprintf(stderr, "[bt] link INTR (open=%x)\n", s_intr_open);
            bt_l2cap_connect(PSM_HID_INTR, CID_LOCAL_INTR);
        }
        }
    }

    /* Once the stack is scanning, a remote can ask to connect. Dolphin does the
     * same from WiimoteDevice::Update, gated on page scan being enabled -- and
     * because it ships with a remote configured, games see a controller and
     * carry on. With nothing ever connecting, a title waits, times out, and
     * re-runs stack setup, which is exactly what this boot did. */
    /* Give the stack a moment after it starts scanning before offering: on
     * hardware a remote is found some frames later, and firing in the same
     * instant it enables page scan lands while it is still settling. */
    {
        static u64 scan_tb;
        if (!(s_scan_enable & PAGE_SCAN_ENABLE)) scan_tb = 0;
        else if (!scan_tb) {
            scan_tb = timing_timebase();
            LOG_INFO(LOG_CORE, "BT: page scan enabled by the game");
        }
        if ((s_scan_enable & PAGE_SCAN_ENABLE) && !s_wm_requested &&
            s_allow_connect && !getenv("BT_NOCONN") && scan_tb &&
            timing_timebase() - scan_tb > bt_offer_delay_tb()) {
            scan_tb = 0;
            LOG_INFO(LOG_CORE, "BT: offering wiimote connection");
            goto do_offer;
        }
    }
    if (0) {
do_offer:;
        if (bt_trace()) fprintf(stderr, "[bt] offering connection\n");
        u8 ev[16];
        unsigned i, off = 0;
        s_wm_requested = 1;
        ev[off++] = EVT_CON_REQ;
        ev[off++] = 10;
        for (i = 0; i < 6; i++) ev[off++] = k_wm_bdaddr[i];
        for (i = 0; i < 3; i++) ev[off++] = k_wm_class[i];
        ev[off++] = HCI_LINK_ACL;
        bt_push_event(ev, off);
    }

    bt_try_deliver();
    bt_try_deliver_acl();

    /* Requests with nothing to deliver stay parked, as on hardware -- the
     * stack's reader threads are meant to block. (Timing them out was tried as
     * a diagnostic: both threads re-arm, and the boot still waits, which is how
     * we know the main thread is waiting for an event we do not generate --
     * a remote connecting -- rather than for a reply we owe it.) */
}

unsigned ios_bt_channels(void)
{
    return (unsigned)((s_cntl_open == 3 ? 1u : 0u) | (s_intr_open == 3 ? 2u : 0u));
}

unsigned ios_bt_parked(void)
{
    return (s_hci_rcount ? 1u : 0u) | (s_acl_rcount ? 2u : 0u);
}

unsigned ios_bt_queued(void) { return s_ev_count; }
unsigned ios_bt_input_reports(void) { return s_input_reports; }
int ios_bt_link_up(void) { return s_wm_connected && s_intr_open == 3; }

/* Rescue diagnostics: the connection state machine's five load-bearing
 * numbers, readable while the guest churns (WPADiManageHandler polling
 * forever = ask these first). */
void ios_bt_state(unsigned out[5])
{
    out[0] = (unsigned)s_wm_connected;
    out[1] = (unsigned)s_intr_open;
    out[2] = s_acl_recv;
    out[3] = s_acl_credited;
    out[4] = (unsigned)s_acl_out_uncredited;
}
void ios_bt_allow_connect(int on) { s_allow_connect = on; }
void ios_bt_set_disconnect_hook(void (*fn)(void)) { s_disc_hook = fn; }

void ios_bt_reset(void)
{
    s_ev_head = s_ev_count = 0;
    s_hci_rhead = s_hci_rcount = 0;
    s_acl_rhead = s_acl_rcount = 0;
    s_scan_enable = 0;
    s_wm_requested = s_wm_connected = 0;
    s_acl_head = s_acl_count = 0;
    s_remote_cid_cntl = s_remote_cid_intr = 0;
    s_l2_ident = 1; s_cntl_open = s_intr_open = 0; s_l2_started = 0;
    s_scan_ticks = 0;
    s_link_ticks = 0;
    s_commands = s_events_sent = s_acl_sent = s_acl_recv = 0;
}
