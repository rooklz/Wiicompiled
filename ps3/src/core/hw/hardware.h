/* hardware.h — the emulated machine's device set.
 *
 * Devices are memory-mapped in the 0xCC00_0000 block and reached through the
 * MMIO dispatcher (memmap.h). Each one registers its own register range at
 * init, so adding a device touches exactly one file.
 *
 * The two devices here are the ones a game's main loop is actually built
 * around: the processor interface, which aggregates every interrupt source and
 * decides when the CPU sees one, and the video interface, whose vertical blank
 * is the heartbeat that paces essentially every title.
 */
#ifndef DOLPHIN_CORE_HW_HARDWARE_H
#define DOLPHIN_CORE_HW_HARDWARE_H

#include "../ppc/gekko.h"

/* ------------------------------------------------------------------ */
/* MMIO block layout (offsets from 0xCC000000)                          */
/* ------------------------------------------------------------------ */

#define HW_CP_BASE      0xCC000000u   /* command processor        */
#define HW_PE_BASE      0xCC001000u   /* pixel engine             */
#define HW_VI_BASE      0xCC002000u   /* video interface          */
#define HW_PI_BASE      0xCC003000u   /* processor interface      */
#define HW_MI_BASE      0xCC004000u   /* memory interface         */
#define HW_DSP_BASE     0xCC005000u   /* DSP + audio DMA          */
#define HW_DI_BASE      0xCC006000u   /* disc interface           */
#define HW_SI_BASE      0xCC006400u   /* serial (controllers)     */
#define HW_EXI_BASE     0xCC006800u   /* expansion interface      */
#define HW_AI_BASE      0xCC006C00u   /* audio interface          */
#define HW_GPFIFO_BASE  0xCC008000u   /* write-gather pipe        */

void hw_init(PPCState *state, int wii_mode);
void hw_reset(void);
void hw_shutdown(void);

/* The Wii IPC block (ipc.c): the mailbox to IOS. Registered by hw_init in Wii
 * mode. */
void ipc_init(void);
void ipc_reset(void);
void ipc_update(void);
unsigned ipc_stat_dispatched(void);
unsigned ipc_stat_replied(void);

/* The audio DSP + ARAM block (dsp.c). */
void dsp_init(void);
void dsp_reset(void);
/* Audio frame clock accounting, for the boot log: how many times the audio DMA
 * has wrapped (and so raised AIDINT), and how many DSP command lists the title
 * has submitted. In a healthy boot these track each other one to one. */
u64  dsp_stat_aid_interrupts(void);
u64  dsp_stat_command_lists(void);

/* The audio interface (ai.c): sample counter, its interrupt, and the audio DMA
 * completion that paces a title's audio frame loop. */
/* The pixel engine (pe.c): the GPU's "I have finished" signalling, which a
 * title's frame loop waits on. */
void pe_init(void);
void pe_reset(void);
void pe_signal_finish(void);
void pe_signal_token(u16 token, int with_interrupt);

void ai_init(void);
void ai_reset(void);
void ai_update(void);
/* The AI's DAC rate in Hz -- the clock the audio DMA next door drains at. */
u32 ai_dac_rate(void);

/* ------------------------------------------------------------------ */
/* Processor interface: interrupt aggregation                           */
/*                                                                      */
/* Every device signals here rather than touching the CPU directly, which keeps */
/* the "is an interrupt visible right now" decision in one place -- it depends  */
/* on the cause register, the mask register and MSR[EE] together, and splitting  */
/* that across devices is how emulators end up with interrupts that fire twice  */
/* or never.                                                                    */
/* ------------------------------------------------------------------ */

typedef enum {
    PI_INT_ERROR      = 0,
    PI_INT_RESET      = 1,   /* reset switch          */
    PI_INT_DI         = 2,   /* disc interface        */
    PI_INT_SI         = 3,   /* serial interface      */
    PI_INT_EXI        = 4,   /* expansion interface   */
    PI_INT_AI         = 5,   /* audio interface       */
    PI_INT_DSP        = 6,
    PI_INT_MEM        = 7,   /* memory interface      */
    PI_INT_VI         = 8,   /* video interface       */
    PI_INT_PE_TOKEN   = 9,   /* pixel engine token    */
    PI_INT_PE_FINISH  = 10,  /* pixel engine finish   */
    PI_INT_CP         = 11,  /* command processor     */
    PI_INT_DEBUG      = 12,
    PI_INT_HSP        = 13,
    PI_INT_IPC        = 14   /* Wii: Hollywood/Starlet */
} PIInterrupt;

void pi_init(void);
void pi_reset(void);
void pi_set_interrupt(PIInterrupt which, int asserted);

/* True when an unmasked interrupt is pending. The CPU checks this at a
 * scheduling boundary rather than per instruction. */
int  pi_interrupt_pending(void);

/* The device-model lock. Held across every guest MMIO access and across the
 * main loop's device servicing, because in the port those run on different
 * threads. See dev_lock.c. */
unsigned ipc_guest_activity(void);
u32      pi_raise_seq(void);
void     pi_note_event(void);

void dev_lock_init(void);
void dev_lock(void);
void dev_lock_tag(const char *who);   /* name the current holder (diagnostics) */
void dev_unlock(void);

/* The CPU-side end of the graphics FIFO ring. The GPU-side end lives in CP;
 * see gx_fifo.h. */
void pi_fifo_window(u32 *base, u32 *end, u32 *wptr);
void pi_fifo_set_write_pointer(u32 wptr);

/* ------------------------------------------------------------------ */
/* Video interface: the frame heartbeat                                 */
/* ------------------------------------------------------------------ */

typedef enum {
    VI_NTSC = 0,      /* 59.94 Hz, 525 lines */
    VI_PAL  = 1,      /* 50.00 Hz, 625 lines */
    VI_MPAL = 2
} VIStandard;

extern unsigned g_vi_irqs;
extern unsigned g_gp_writes;
void vi_init(VIStandard standard);
void vi_reset(void);

/* Fields completed since reset -- a cheap way for the frontend to pace
 * presentation without inspecting the scheduler. */
u64  vi_field_count(void);

/* Address of the external framebuffer the video interface is currently
 * scanning out, or 0 if the guest has not configured one. */
u32  vi_current_xfb(void);

#endif /* DOLPHIN_CORE_HW_HARDWARE_H */
