/* dsp.c — the DSP/ARAM audio block: mailbox, control register, audio DMA.
 *
 * The audio DSP is a second processor with its own boot ROM. A title's early
 * code resets it, hands the ROM a microcode image to load, and then talks to
 * the running microcode over a pair of mailboxes. We do not emulate the DSP
 * itself; we recognise the microcode it was asked to run and reimplement that
 * microcode's observable behaviour (dsp_ax.c) -- which is what Dolphin's DSP
 * HLE does, and the reason this file's job is the *protocol* rather than the
 * signal processing.
 *
 * Three separate mechanisms live here and a title's audio needs all three:
 *
 *   * The mailbox, including its queue-and-halt semantics. Modelled on
 *     Dolphin's CMailHandler (Source/Core/Core/HW/DSPHLE/MailHandler.cpp),
 *     because the "is there mail waiting" bit is derived state, not a flag:
 *     it is the top bit of the last mail, cleared by reading the low half.
 *   * The control register's ucode-switching edges. Reset selects the boot ROM
 *     (which announces itself with 0x8071FEED); clearing DSPInit selects the
 *     init microcode, which answers 0x80544348 (Dolphin DSPHLE.cpp:214-227 and
 *     UCodes/INIT.cpp:21). __OSInitAudioSystem waits for exactly that.
 *   * The audio DMA, which drains 32 bytes at a time at the AI's DAC rate and
 *     raises AIDINT when the buffer wraps. That interrupt is the audio frame
 *     clock: MKWii programs a 12-block buffer, which is 96 stereo samples,
 *     which is one 3 ms AX frame at 32 kHz, and it builds exactly one DSP
 *     command list per interrupt.
 */
#include <stdio.h>
#include <stdlib.h>
#include "hardware.h"
#include "dsp_ax.h"
#include "audio_out.h"
#include "../mem/memmap.h"
#include "../core_timing.h"
#include "../../common/log.h"

/* Register offsets from 0xCC005000. */
#define DSP_MBOX_IN_H   0x00
#define DSP_MBOX_IN_L   0x02
#define DSP_MBOX_OUT_H  0x04
#define DSP_MBOX_OUT_L  0x06
#define DSP_CSR         0x0A    /* control/status */
#define AR_DMA_MMADDR   0x20
#define AR_DMA_ARADDR   0x24
#define AR_DMA_CNT      0x28
#define AI_DMA_ADDR_HI  0x30    /* audio DMA: main-memory source          */
#define AI_DMA_ADDR_LO  0x32
#define AI_DMA_CONTROL  0x36    /* bit 15 enables; low bits = block count */
#define AI_DMA_LEFT     0x3A

/* CSR bits. */
#define CSR_RES        0x0001   /* reset; self-clearing                   */
#define CSR_ASSERTINT  0x0002
#define CSR_HALT       0x0004   /* DSP halted; cleared to run it          */
#define CSR_AIDINT     0x0008   /* AI-DMA interrupt status  (write 1 = ack) */
#define CSR_AIDINTMSK  0x0010
#define CSR_ARINT      0x0020   /* ARAM-DMA interrupt status (w1c)        */
#define CSR_ARINTMSK   0x0040
#define CSR_DSPINT     0x0080   /* DSP mailbox interrupt status (w1c)     */
#define CSR_DSPINTMSK  0x0100
#define CSR_DMASTATE   0x0200   /* read-only: ARAM DMA in progress        */
#define CSR_INITCODE   0x0400
#define CSR_INIT       0x0800
#define CSR_STATUS     (CSR_AIDINT | CSR_ARINT | CSR_DSPINT)

/* Task mails the uploaded ucode sends back (Dolphin UCodes.h). */
#define DSP_INIT    0xDCD10000u
#define DSP_RESUME  0xDCD10001u
#define DSP_YIELD   0xDCD10002u
#define DSP_SYNC    0xDCD10004u

/* The mail the init microcode answers with (Dolphin UCodes/INIT.cpp:21). */
#define INIT_UCODE_MAIL 0x80544348u
/* The boot ROM's greeting (Dolphin UCodes/ROM.cpp:32). */
#define ROM_UCODE_MAIL  0x8071FEEDu

static u16 s_csr;
static u32 s_mbox_in;

/* ------------------------------------------------------------------ */
/* Mail queue (Dolphin DSPHLE/MailHandler.cpp)                          */
/*                                                                      */
/* The real DSP has one pair of registers and no queue; HLE needs a small one   */
/* because a ucode replies to a command list with two mails back to back and    */
/* the CPU reads them one at a time. `last_mail` is what the registers actually  */
/* read back, and reading the low half clears its top bit -- that bit is the     */
/* "mail waiting" flag every boot poll spins on.                                */
/* ------------------------------------------------------------------ */

#define MAILQ_MAX 8
static u32 s_mailq[MAILQ_MAX];
static u8  s_mailq_irq[MAILQ_MAX];
static int s_mailq_head, s_mailq_count;
static u32 s_last_mail;
static int s_mail_halted;

static u32 s_boot_param;        /* pending 0x80F3xxxx command             */
static int s_running_ucode;     /* an audio ucode has been booted          */
static int s_cmdlist_pending;   /* 0xBABE seen, address expected next      */
static u16 s_cmdlist_size;

/* Microcode upload parameters collected by the boot ROM. */
static u32 s_uc_ram_addr, s_uc_length, s_uc_dmem_len, s_uc_imem_addr;

/* Audio DMA. */
static u16 s_ai_dma_ctrl, s_ai_dma_left;
static u32 s_ai_dma_addr;       /* programmed source (latched on reload)   */
static u32 s_ai_dma_cur;        /* address the next block comes from       */
static int s_ev_aidma = -1;     /* scheduler handle for the block clock    */
static int s_ev_aidstart = -1;  /* the "DMA has started" interrupt         */

/* The init microcode's DSPInitCode bit reads back set for a short while
 * (Dolphin DSPHLE.cpp:225 -- "number obtained from real hardware on a Wii"). */
static u64 s_initcode_clear_tb;

static u64 s_stat_aidint, s_stat_cmdlists;

static int dsp_trace(void)
{
    static int t = -1;
    if (t < 0) t = getenv("DSP_TRACE") != NULL;
    return t;
}

static int dsp_ax_disabled(void)
{
    static int d = -1;
    if (d < 0) d = getenv("DSP_NO_AX") != NULL;
    return d;
}

/* The DSP interrupt reaches the CPU through the processor interface, and only
 * when its enable bit is set -- the same status-and-enable pairing the hardware
 * uses (Dolphin DSP.cpp UpdateInterrupts). Without this the mailbox can be
 * perfectly correct and the guest still never notices: a title that waits on
 * the DSP interrupt after booting its audio ucode simply stops. */
static void dsp_update_irq(void)
{
    int line = ((s_csr >> 1) & s_csr & CSR_STATUS) != 0;
    pi_set_interrupt(PI_INT_DSP, line);
}

static void dsp_raise(u16 bit)
{
    s_csr |= bit;
    dsp_update_irq();
}

/* Post mail to the CPU, optionally raising the interrupt with it. */
static void dsp_post(u32 mail, int irq)
{
    int slot;
    if (dsp_trace())
        fprintf(stderr, "[dsp] post %08x irq=%d\n", (unsigned)mail, irq);

    if (s_mailq_count >= MAILQ_MAX) {
        LOG_WARN(LOG_CORE, "DSP mail queue overflow, dropping %08x",
                 (unsigned)mail);
        return;
    }
    if (irq) {
        if (s_mailq_count == 0)
            dsp_raise(CSR_DSPINT);
        else
            s_mailq_irq[s_mailq_head] = 1;   /* fires when the front is read */
    }
    slot = (s_mailq_head + s_mailq_count) % MAILQ_MAX;
    s_mailq[slot] = mail;
    s_mailq_irq[slot] = 0;
    s_mailq_count++;
}

static void dsp_mail_clear(void)
{
    s_mailq_head = s_mailq_count = 0;
}

/* Switching microcode drops whatever the previous one had queued and lets the
 * new one announce itself (Dolphin DSPHLE::SetUCode). */
static void dsp_set_ucode_rom(void)
{
    s_running_ucode = 0;
    s_boot_param = 0;
    s_cmdlist_pending = 0;
    dsp_mail_clear();
    dsp_post(ROM_UCODE_MAIL, 0);
}

static void dsp_set_ucode_init(void)
{
    s_running_ucode = 0;
    s_boot_param = 0;
    s_cmdlist_pending = 0;
    dsp_mail_clear();
    dsp_post(INIT_UCODE_MAIL, 0);
}

/* Ector hash, the identity function Dolphin uses to recognise a microcode
 * image (Common/Hash.cpp HashEctor, called from ROMUCode::BootUCode). */
static u32 dsp_hash_ector(const u8 *p, u32 len)
{
    u32 crc = 0, i;
    for (i = 0; i < len; i++) {
        crc ^= p[i];
        crc = (crc << 3) | (crc >> 29);
    }
    return crc;
}

static void dsp_boot_ucode(void)
{
    u32 crc = 0;
    if (s_uc_length && mem_valid_span(s_uc_ram_addr) >= s_uc_length) {
        const u8 *p = (const u8 *)mem_ptr(s_uc_ram_addr);
        if (p)
            crc = dsp_hash_ector(p, s_uc_length);
    }
    LOG_INFO(LOG_CORE, "DSP ucode boot: src=%08x len=%04x imem=%04x dmem=%04x crc=%08x",
             (unsigned)s_uc_ram_addr, (unsigned)s_uc_length,
             (unsigned)s_uc_imem_addr, (unsigned)s_uc_dmem_len, (unsigned)crc);
    if (dsp_trace())
        fprintf(stderr, "[dsp] ucode src=%08x len=%04x crc=%08x\n",
                (unsigned)s_uc_ram_addr, (unsigned)s_uc_length, (unsigned)crc);
    ax_boot_ucode(crc);
    s_running_ucode = 1;
    dsp_post(DSP_INIT, 1);
}

static u32 dsp_read(u32 addr, unsigned size, void *ctx)
{
    (void)size; (void)ctx;
    switch (addr - 0xCC005000u) {
    case DSP_CSR:
        /* DSPInitCode is set when the init ucode is entered and clears itself
         * a short time later; a title that polls it must eventually see it go
         * away (Dolphin DSPHLE::DSP_ReadControlRegister). */
        if ((s_csr & CSR_INITCODE) && timing_timebase() >= s_initcode_clear_tb)
            s_csr &= (u16)~CSR_INITCODE;
        return s_csr;

    case AI_DMA_CONTROL: return s_ai_dma_ctrl;
    case AI_DMA_LEFT:
        /* Zero-based: Dolphin returns remaining-1 and notes that DreamMix
         * World Fighters hangs if the register never reaches zero. */
        return s_ai_dma_left ? (u32)(s_ai_dma_left - 1u) : 0u;
    case AI_DMA_ADDR_HI: return s_ai_dma_addr >> 16;
    case AI_DMA_ADDR_LO: return s_ai_dma_addr & 0xFFFFu;

    case DSP_MBOX_OUT_H:
        if (!s_mail_halted && s_mailq_count)
            s_last_mail = s_mailq[s_mailq_head];
        return (s_last_mail >> 16) & 0xFFFFu;

    case DSP_MBOX_OUT_L: {
        int irq = 0;
        if (!s_mail_halted && s_mailq_count) {
            s_last_mail = s_mailq[s_mailq_head];
            irq = s_mailq_irq[s_mailq_head];
            s_mailq_head = (s_mailq_head + 1) % MAILQ_MAX;
            s_mailq_count--;
            if (dsp_trace())
                fprintf(stderr, "[dsp] out  %08x read\n", (unsigned)s_last_mail);
        }
        /* Reading the low half clears the "mail waiting" bit; the rest of the
         * value keeps reading back until new mail arrives. */
        s_last_mail &= 0x7FFFFFFFu;
        if (irq)
            dsp_raise(CSR_DSPINT);
        return s_last_mail & 0xFFFFu;
    }
    default:
        return 0;
    }
}

/* One audio-DMA block is 32 bytes: 8 stereo 16-bit frames, drained at the AI's
 * DAC rate. Dolphin derives exactly this period from the AID rate divisor
 * (SystemTimers.cpp GetAudioDMACallbackPeriod); at the 32 kHz the SDK
 * programs it is 4 kHz, so a 12-block buffer is 3 ms -- exactly the AX frame
 * period, which is the check that this rate is right rather than plausible. */
static s64 dsp_block_cycles(void)
{
    u32 rate = ai_dac_rate();
    if (!rate) rate = 32000u;
    return (s64)((u64)g_cpu_hz * 8ull / (u64)rate);
}

static void dsp_on_aidma(u64 userdata, s64 cycles_late)
{
    (void)userdata;
    if (!(s_ai_dma_ctrl & 0x8000))
        return;                     /* disabled between blocks; stay stopped */

    /* Hand the block to the sink before advancing, in the same order the DAC
     * would have consumed it. */
    if (audio_out_enabled()) {
        const void *p = (mem_valid_span(s_ai_dma_cur) >= 32u)
                            ? mem_ptr(s_ai_dma_cur) : NULL;
        audio_out_set_rate(ai_dac_rate());
        if (p)
            audio_out_push_be_rl(p, 8);
    }

    if (s_ai_dma_left) {
        s_ai_dma_left--;
        s_ai_dma_cur += 32u;
    }
    if (s_ai_dma_left == 0) {
        /* Buffer consumed: relatch the (possibly newly written) source and
         * block count and raise the interrupt, the way the hardware
         * auto-repeats until the DMA is disabled. */
        s_ai_dma_cur = s_ai_dma_addr;
        s_ai_dma_left = (u16)(s_ai_dma_ctrl & 0x7FFF);
        s_stat_aidint++;
        dsp_raise(CSR_AIDINT);
    }
    timing_schedule(s_ev_aidma, dsp_block_cycles() - cycles_late, 0);
}

/* Dolphin schedules the "transfer started" AID interrupt 200 cycles after the
 * enable edge and documents why it must not be sooner (DSP.cpp:341-347: Sky
 * Crawlers crashes below 87 cycles). */
static void dsp_on_aidstart(u64 userdata, s64 cycles_late)
{
    (void)userdata; (void)cycles_late;
    s_stat_aidint++;
    dsp_raise(CSR_AIDINT);
}

static void dsp_handle_mail(u32 mail)
{
    if (!s_running_ucode) {
        /* Boot ROM: command words 0x80F3xxxx each take one parameter, and
         * 0x80F3D001 (the start vector) is what actually launches the
         * uploaded ucode (Dolphin UCodes/ROM.cpp HandleMail). */
        if (s_boot_param == 0) {
            if ((mail & 0xFFFF0000u) == 0x80F30000u)
                s_boot_param = mail;
            else
                dsp_post(0xFEEE0000u | (mail & 0xFFFFu), 0);
            return;
        }
        switch (s_boot_param) {
        case 0x80F3A001u: s_uc_ram_addr  = mail;            break;
        case 0x80F3A002u: s_uc_length    = mail & 0xFFFFu;  break;
        case 0x80F3B002u: s_uc_dmem_len  = mail & 0xFFFFu;  break;
        case 0x80F3C002u: s_uc_imem_addr = mail & 0xFFFFu;  break;
        case 0x80F3D001u:
            s_boot_param = 0;
            dsp_boot_ucode();
            return;
        default: break;
        }
        s_boot_param = 0;
        return;
    }

    /* A running audio ucode: a command list is announced with 0xBABExxxx
     * carrying its length in words, followed by its address; the ucode
     * answers with a sync (from its OUTPUT command) and then a yield. The
     * 0xCDD1xxxx mails are task control. */
    if (s_cmdlist_pending) {
        int outputs;
        s_cmdlist_pending = 0;
        s_stat_cmdlists++;
        outputs = dsp_ax_disabled() ? -1 : ax_run_cmdlist(mail, s_cmdlist_size);
        /* AXWii's OUTPUT command is what pushes the sync mail
         * (AXWii.cpp OutputSamples); the yield always follows it
         * (AX.cpp SignalWorkEnd). A list we could not execute is answered the
         * way the mailbox-only stub answered it, so turning the HLE off with
         * DSP_NO_AX leaves the protocol exactly as it was. */
        if (outputs != 0)
            dsp_post(DSP_SYNC, 1);
        dsp_post(DSP_YIELD, 1);
        return;
    }
    if ((mail & 0xFFFF0000u) == 0xBABE0000u) {
        s_cmdlist_pending = 1;
        s_cmdlist_size = (u16)(mail & 0xFFFFu);
        return;
    }
    switch (mail) {
    case 0xCDD10000u:               /* MAIL_RESUME    */
        dsp_post(DSP_RESUME, 1);
        break;
    case 0xCDD10002u:               /* MAIL_RESET     */
        dsp_set_ucode_rom();
        break;
    case 0xCDD10001u:               /* MAIL_NEW_UCODE */
    case 0xCDD10003u:               /* MAIL_CONTINUE  */
    default:
        break;                      /* no acknowledgement is expected */
    }
}

static void dsp_write(u32 addr, u32 value, unsigned size, void *ctx)
{
    (void)size; (void)ctx;
    switch (addr - 0xCC005000u) {
    case DSP_CSR: {
        u16 v = (u16)value;
        int was_halted = (s_csr & CSR_HALT) != 0;
        int was_init   = (s_csr & CSR_INIT) != 0;
        int now_halted, now_init;

        if (dsp_trace())
            fprintf(stderr, "[dsp] csr<-%04x (was %04x)\n",
                    (unsigned)v, (unsigned)s_csr);
        LOG_DEBUG(LOG_CORE, "DSP CSR <- %04x", (unsigned)v);

        {
            /* Real CSR semantics (Dolphin DSP.cpp DSP_CONTROL write): the
             * three interrupt-status bits are write-1-to-CLEAR -- writing 0
             * leaves them alone -- masks and mode bits are stored, DMASTATE is
             * read-only, and RES is a command that self-clears. The naive
             * `s_csr = value` this replaces meant every interrupt ack *stored*
             * the ack bit as status and wiped the masks: MKWii acks its stale
             * DSP interrupt while enabling masks in one write (0x0990), and
             * that corruption is exactly where its audio init froze. */
            u16 status = CSR_AIDINT | CSR_ARINT | CSR_DSPINT;
            u16 kept   = (u16)(s_csr & status & ~v);   /* w1c */
            u16 stored = (u16)(v & ~(status | CSR_DMASTATE | CSR_RES | CSR_INITCODE));
            s_csr = (u16)(stored | kept | (s_csr & (CSR_DMASTATE | CSR_INITCODE)));
        }
        now_halted = (s_csr & CSR_HALT) != 0;
        now_init   = (s_csr & CSR_INIT) != 0;

        /* The halt bit gates *visibility* of mail, not its production. Dolphin
         * only calls SetHalted on a change, which is why the power-on state
         * (halted, but never written as a transition) still lets the boot read
         * the ROM's greeting. */
        if (was_halted != now_halted)
            s_mail_halted = now_halted;

        if (v & CSR_RES) {
            s_ai_dma_ctrl = 0;      /* Dolphin DSP.cpp:266-270 */
            s_ai_dma_left = 0;
            if (s_ev_aidma >= 0) timing_remove(s_ev_aidma);
            dsp_set_ucode_rom();
        } else if (was_init && !now_init) {
            /* DSPInit 1 -> 0 uploads and enters the init microcode, which
             * replies 0x80544348 (Dolphin DSPHLE.cpp:214-227). This is the
             * mail __OSInitAudioSystem drains before it uploads DSPInitCode;
             * answering with the boot ROM's greeting instead happened to work
             * only because the title does not inspect the value. */
            dsp_set_ucode_init();
            s_csr |= CSR_INITCODE;
            s_initcode_clear_tb = timing_timebase() + 130ull;
        }
        dsp_update_irq();
        break;
    }
    case DSP_MBOX_IN_H:
        s_mbox_in = (s_mbox_in & 0x0000FFFFu) | (value << 16);
        break;
    case DSP_MBOX_IN_L:
        s_mbox_in = (s_mbox_in & 0xFFFF0000u) | (value & 0xFFFFu);
        if (dsp_trace())
            fprintf(stderr, "[dsp] in   %08x\n", (unsigned)s_mbox_in);
        LOG_DEBUG(LOG_CORE, "DSP mail-in  %08x", (unsigned)s_mbox_in);
        dsp_handle_mail(s_mbox_in);   /* the low write delivers the mail */
        break;

    case AI_DMA_CONTROL: {
        /* Starting the audio DMA arms the block-complete interrupt: the AI
         * drains the buffer at the DAC rate and raises AIDINT when the
         * programmed block count runs out, then relatches and keeps going.
         * That repeat is the audio frame clock -- AX builds one DSP command
         * list per AID interrupt, and nw4r::snd's sound thread runs off AX's
         * frame callback.
         *
         * The source address and block count are latched only on the
         * disabled->enabled edge (Dolphin DSP.cpp:328-338): while the DMA is
         * running, new values are picked up at the next wrap instead, which is
         * exactly how a title double-buffers without a gap. */
        int already = (s_ai_dma_ctrl & 0x8000) != 0;
        s_ai_dma_ctrl = (u16)value;
        if (dsp_trace())
            fprintf(stderr, "[dsp] aidma ctrl<-%04x addr=%08x\n",
                    (unsigned)value, (unsigned)s_ai_dma_addr);
        if (!already && (s_ai_dma_ctrl & 0x8000)) {
            s_ai_dma_cur = s_ai_dma_addr;
            s_ai_dma_left = (u16)(s_ai_dma_ctrl & 0x7FFF);
            if (s_ev_aidstart >= 0)
                timing_schedule(s_ev_aidstart, 200, 0);
            if (s_ev_aidma >= 0 && s_ai_dma_left)
                timing_schedule(s_ev_aidma, dsp_block_cycles(), 0);
        } else if (!(s_ai_dma_ctrl & 0x8000)) {
            if (s_ev_aidma >= 0) timing_remove(s_ev_aidma);
        }
        break;
    }

    case AI_DMA_ADDR_HI:
        s_ai_dma_addr = (s_ai_dma_addr & 0x0000FFFFu) | (value << 16);
        break;
    case AI_DMA_ADDR_LO:
        s_ai_dma_addr = (s_ai_dma_addr & 0xFFFF0000u) | (value & 0xFFFFu);
        break;

    case AR_DMA_CNT:
        /* Programming the transfer count kicks the ARAM DMA. It completes at
         * once and raises the ARAM-DMA-complete bit the boot polls for. */
        dsp_raise(CSR_ARINT);
        break;
    default:
        break;
    }
}

u64 dsp_stat_aid_interrupts(void) { return s_stat_aidint; }
u64 dsp_stat_command_lists(void)  { return s_stat_cmdlists; }

void dsp_reset(void)
{
    s_csr = CSR_HALT | CSR_INIT;        /* power-on state (Dolphin DSPHLE)  */
    s_boot_param = 0;
    s_running_ucode = 0;
    s_cmdlist_pending = 0;
    s_cmdlist_size = 0;
    s_uc_ram_addr = s_uc_length = s_uc_dmem_len = s_uc_imem_addr = 0;
    s_ai_dma_ctrl = s_ai_dma_left = 0;
    s_ai_dma_addr = s_ai_dma_cur = 0;
    s_initcode_clear_tb = 0;
    s_stat_aidint = s_stat_cmdlists = 0;
    if (s_ev_aidma >= 0)
        timing_remove(s_ev_aidma);
    if (s_ev_aidstart >= 0)
        timing_remove(s_ev_aidstart);
    s_mbox_in = 0;
    s_last_mail = 0;
    /* Not halted for mail purposes even though the halt bit is set: Dolphin's
     * mail handler only learns about halting from a *change* written to the
     * control register, and the boot depends on being able to read the ROM's
     * greeting before it ever clears the bit. */
    s_mail_halted = 0;
    dsp_mail_clear();
    dsp_post(ROM_UCODE_MAIL, 0);
    ax_reset();
    audio_out_reset();
}

void dsp_init(void)
{
    /* Always re-register: timing_init clears the event table, so a handle
     * cached from a previous machine would refer to nothing and the audio
     * frame clock would silently never tick. */
    s_ev_aidma = timing_register_event("DSP audio DMA", dsp_on_aidma);
    s_ev_aidstart = timing_register_event("DSP audio DMA start", dsp_on_aidstart);
    dsp_reset();
    mmio_register(0xCC005000u, 0x200, dsp_read, dsp_write, NULL, "DSP");
}
