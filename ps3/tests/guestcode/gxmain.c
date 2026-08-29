/* gxmain.c — a GameCube program that talks to the GPU.
 *
 * Compiled to real 32-bit PowerPC and executed by the emulator, this does what
 * a title's rendering code does and nothing else: configure the command FIFO,
 * then stream commands to the GPU through the write-gather pipe at
 * 0xCC008000.
 *
 * It exists because every graphics test so far calls the emulator's own
 * functions to poke the front end. That verifies each stage but not the
 * junction between the CPU and the GPU -- the recompiler has to turn these
 * stores into MMIO writes that reach the gather pipe, the pipe has to burst
 * into a ring the CPU also configured, and the parser has to read it back.
 * Nothing but real guest code exercises that whole path.
 *
 * No libc, no relocation, no initialised globals beyond the constants below.
 * Control returns by branching to address 0, which is illegal and traps.
 */
typedef unsigned int   u32;
typedef unsigned short u16;
typedef unsigned char  u8;

/* Processor interface: the CPU side of the FIFO ring. */
#define PI_FIFO_BASE (*(volatile u32 *)0xCC00300Cu)
#define PI_FIFO_END  (*(volatile u32 *)0xCC003010u)
#define PI_FIFO_WPTR (*(volatile u32 *)0xCC003014u)

/* Command processor: the GPU side. 16-bit registers in LO/HI pairs. */
#define CP_CONTROL   (*(volatile u16 *)0xCC000002u)
#define CP_BASE_LO   (*(volatile u16 *)0xCC000020u)
#define CP_BASE_HI   (*(volatile u16 *)0xCC000022u)
#define CP_END_LO    (*(volatile u16 *)0xCC000024u)
#define CP_END_HI    (*(volatile u16 *)0xCC000026u)
#define CP_HIWATER_LO (*(volatile u16 *)0xCC000028u)
#define CP_HIWATER_HI (*(volatile u16 *)0xCC00002Au)
#define CP_RWDIST_LO (*(volatile u16 *)0xCC000030u)
#define CP_RWDIST_HI (*(volatile u16 *)0xCC000032u)
#define CP_RPTR_LO   (*(volatile u16 *)0xCC000038u)
#define CP_RPTR_HI   (*(volatile u16 *)0xCC00003Au)

/* The write-gather pipe. Every store here goes into Gekko's 32-byte buffer and
 * bursts to the FIFO when it fills; the width of the store is what decides how
 * many bytes are appended, so the three sizes are all used deliberately. */
#define GP8   (*(volatile u8  *)0xCC008000u)
#define GP16  (*(volatile u16 *)0xCC008000u)
#define GP32  (*(volatile u32 *)0xCC008000u)

#define FIFO_BASE 0x80280000u
#define FIFO_END  (FIFO_BASE + 127u * 32u)      /* 128 cells: 4 KiB */

#define GX_NOP           0x00
#define GX_LOAD_CP_REG   0x08
#define GX_LOAD_XF_REG   0x10
#define GX_LOAD_BP_REG   0x61
#define GX_TRIANGLES     0x90

static void fifo_init(void)
{
    PI_FIFO_BASE = FIFO_BASE;
    PI_FIFO_END  = FIFO_END;
    PI_FIFO_WPTR = FIFO_BASE;

    CP_BASE_LO = (u16)(FIFO_BASE & 0xFFFFu);
    CP_BASE_HI = (u16)(FIFO_BASE >> 16);
    CP_END_LO  = (u16)(FIFO_END & 0xFFFFu);
    CP_END_HI  = (u16)(FIFO_END >> 16);
    CP_RPTR_LO = (u16)(FIFO_BASE & 0xFFFFu);
    CP_RPTR_HI = (u16)(FIFO_BASE >> 16);
    CP_RWDIST_LO = 0;
    CP_RWDIST_HI = 0;
    /* Watermark out of the way: this program is not testing flow control. */
    CP_HIWATER_LO = 0xFFFFu;
    CP_HIWATER_HI = 0xFFFFu;

    /* Read enable + GP link, so the CP's write pointer follows the CPU's. */
    CP_CONTROL = 0x0011u;
}

/* GX pads to a burst boundary before the GPU is expected to see anything. */
static void gp_flush(void)
{
    int i;
    for (i = 0; i < 32; i++)
        GP8 = GX_NOP;
}

void _start(void)
{
    int i;

    fifo_init();

    /* Pixel engine: eight TEV stages, cull back faces. */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x00u << 24) | (7u << 10) | (1u << 14);

    /* Depth test enabled, function 3, writes on. */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x40u << 24) | (1u | (3u << 1) | (1u << 4));

    /* Transform unit: three texture coordinate generators. */
    GP8  = GX_LOAD_XF_REG;
    GP32 = (0u << 16) | 0x103Fu;
    GP32 = 3u;

    /* Vertex format: position only, three floats, direct. */
    GP8  = GX_LOAD_CP_REG;
    GP8  = 0x50u;                       /* VCD lo */
    GP32 = (1u << 9);                   /* position = direct */
    GP8  = GX_LOAD_CP_REG;
    GP8  = 0x70u;                       /* VAT A  */
    GP32 = (1u << 0) | (4u << 1);       /* xyz, f32 */

    /* One triangle: three vertices of twelve bytes each. The values are
     * recognisable so the harness can prove the vertex data arrived intact and
     * at the address the parser reported. */
    GP8  = GX_TRIANGLES;
    GP16 = 3;
    for (i = 0; i < 9; i++)
        GP32 = 0xAA000000u | (u32)i;

    /* An EFB copy to the external framebuffer -- the command that makes a frame
     * visible, and the last thing a title's frame does. */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x49u << 24) | 0u;                          /* source top-left  */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x4Au << 24) | (639u | (479u << 10));       /* 640 x 480        */
    /* The destination is a *physical* address: the register's value field is
     * 24 bits holding 32-byte units, so a cached address like 0x80600000 does
     * not fit and its high bits would corrupt the register number written
     * alongside it. */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x4Bu << 24) | (0x00600000u >> 5);          /* destination      */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x52u << 24) | (1u << 14);                  /* trigger, to XFB  */

    gp_flush();

    /* Back to the loader. */
    ((void(*)(void))0)();
}
