/* gxanim.c — a GameCube program with a frame loop.
 *
 * gxtri.c draws one frame. This is the next structural step toward a game:
 * sixty frames, each drawn, finished with an EFB copy, and visibly different
 * from the last -- the skeleton every real title's main loop has. The colour
 * animation is integer arithmetic on the packed vertex colour, because the
 * guest deliberately contains no floating-point code.
 */
typedef unsigned int   u32;
typedef unsigned short u16;
typedef unsigned char  u8;

#define PI_FIFO_BASE (*(volatile u32 *)0xCC00300Cu)
#define PI_FIFO_END  (*(volatile u32 *)0xCC003010u)
#define PI_FIFO_WPTR (*(volatile u32 *)0xCC003014u)
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

#define GP8   (*(volatile u8  *)0xCC008000u)
#define GP16  (*(volatile u16 *)0xCC008000u)
#define GP32  (*(volatile u32 *)0xCC008000u)

#define FIFO_BASE 0x80280000u
#define FIFO_END  (FIFO_BASE + 127u * 32u)

#define GX_NOP           0x00
#define GX_LOAD_CP_REG   0x08
#define GX_LOAD_XF_REG   0x10
#define GX_LOAD_BP_REG   0x61
#define GX_TRIANGLES     0x90

#define FRAMES 60

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
    CP_HIWATER_LO = 0xFFFFu;
    CP_HIWATER_HI = 0xFFFFu;
    CP_CONTROL = 0x0011u;
}

static void gp_flush(void)
{
    int i;
    for (i = 0; i < 32; i++)
        GP8 = GX_NOP;
}

void _start(void)
{
    u32 f;

    fifo_init();

    /* Static state, once: TEV passthrough, projection, colour channel,
     * vertex format. Identical to gxtri.c. */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x00u << 24) | (0u << 10);
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0xC0u << 24) | (15u << 12) | (15u << 8) | (15u << 4) | 10u
                         | (1u << 19);
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0xC1u << 24) | (7u << 13) | (7u << 10) | (7u << 7) | (5u << 4)
                         | (1u << 19);
    GP8  = GX_LOAD_XF_REG;
    GP32 = (6u << 16) | 0x1020u;
    GP32 = 0x3F800000u; GP32 = 0; GP32 = 0x3F800000u; GP32 = 0;
    GP32 = 0x3F800000u; GP32 = 0; GP32 = 1u;
    GP8  = GX_LOAD_XF_REG;
    GP32 = (0u << 16) | 0x1009u;
    GP32 = 1u;
    GP8  = GX_LOAD_CP_REG;
    GP8  = 0x50u;
    GP32 = (1u << 9) | (1u << 13);
    GP8  = GX_LOAD_CP_REG;
    GP8  = 0x70u;
    GP32 = (1u << 0) | (4u << 1) | (5u << 14);

    for (f = 0; f < FRAMES; f++) {
        /* The first vertex's colour sweeps red -> yellow as the green byte
         * climbs; the other two stay fixed. Visible motion, integer-only. */
        u32 c0 = 0xFF0000FFu | (((f * 4u) & 0xFFu) << 16);

        GP8  = GX_TRIANGLES;
        GP16 = 3;
        GP32 = 0xBF19999Au; GP32 = 0xBF000000u; GP32 = 0x00000000u;
        GP32 = c0;
        GP32 = 0x3F19999Au; GP32 = 0xBF000000u; GP32 = 0x00000000u;
        GP32 = 0x33FF33FFu;
        GP32 = 0x00000000u; GP32 = 0x3F19999Au; GP32 = 0x00000000u;
        GP32 = 0x4D66FFFFu;

        /* Frame done. */
        GP8  = GX_LOAD_BP_REG;
        GP32 = (0x49u << 24) | 0u;
        GP8  = GX_LOAD_BP_REG;
        GP32 = (0x4Au << 24) | (639u | (479u << 10));
        GP8  = GX_LOAD_BP_REG;
        GP32 = (0x4Bu << 24) | (0x00600000u >> 5);
        GP8  = GX_LOAD_BP_REG;
        GP32 = (0x52u << 24) | (1u << 14);
        gp_flush();
    }

    ((void(*)(void))0)();
}
