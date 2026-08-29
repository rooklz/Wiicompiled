/* gxtri.c — a GameCube program that draws a triangle you can see.
 *
 * gxmain.c proves the CPU-to-GPU junction with deliberately unrenderable
 * values. This is its sibling with the opposite goal: real coordinates, a real
 * projection, one visible triangle -- the smallest complete frame a Wii title
 * could draw. When this renders on the television, the entire pipeline has
 * worked: recompiled guest stores -> write-gather pipe -> FIFO -> command
 * parser -> state tracker -> generated RSX shaders -> rasteriser.
 *
 * Same conventions as gxmain.c: no libc, floats written as their bit patterns
 * (no FP code in the guest, so no FP state to set up), return by branching to
 * zero.
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
    fifo_init();

    /* One TEV stage. */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x00u << 24) | (0u << 10);

    /* TEV stage 0: pass the rasterised vertex colour straight through --
     * colour combiner d = RASC with a/b/c zero, alpha combiner d = RASA.
     * The classic GX_PASSCLR configuration every 2D game uses. */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0xC0u << 24) | (15u << 12) | (15u << 8) | (15u << 4) | 10u
                         | (1u << 19);
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0xC1u << 24) | (7u << 13) | (7u << 10) | (7u << 7) | (5u << 4)
                         | (1u << 19);

    /* An identity orthographic projection: clip space in, clip space out.
     * Coefficients (1,0,1,0,1,0) and type 1 (ortho), loaded as seven words at
     * XF 0x1020 -- exactly what GXSetProjection writes. */
    GP8  = GX_LOAD_XF_REG;
    GP32 = (6u << 16) | 0x1020u;        /* count-1 = 6: seven words */
    GP32 = 0x3F800000u;                 /* 1.0f */
    GP32 = 0x00000000u;
    GP32 = 0x3F800000u;
    GP32 = 0x00000000u;
    GP32 = 0x3F800000u;
    GP32 = 0x00000000u;
    GP32 = 1u;                          /* orthographic */

    /* One colour channel, so the vertex program routes COL0 through. */
    GP8  = GX_LOAD_XF_REG;
    GP32 = (0u << 16) | 0x1009u;
    GP32 = 1u;

    /* Vertex format: position (three f32) and colour 0 (RGBA8), both direct. */
    GP8  = GX_LOAD_CP_REG;
    GP8  = 0x50u;
    GP32 = (1u << 9) | (1u << 13);
    GP8  = GX_LOAD_CP_REG;
    GP8  = 0x70u;
    GP32 = (1u << 0) | (4u << 1) | (5u << 14);

    /* The triangle, in clip coordinates the identity projection passes
     * through: the same shape rsx_tritest proved the RSX renders. */
    GP8  = GX_TRIANGLES;
    GP16 = 3;
    GP32 = 0xBF19999Au; GP32 = 0xBF000000u; GP32 = 0x00000000u; /* -0.6 -0.5 */
    GP32 = 0xFF3333FFu;                                          /* red      */
    GP32 = 0x3F19999Au; GP32 = 0xBF000000u; GP32 = 0x00000000u; /*  0.6 -0.5 */
    GP32 = 0x33FF33FFu;                                          /* green    */
    GP32 = 0x00000000u; GP32 = 0x3F19999Au; GP32 = 0x00000000u; /*  0.0  0.6 */
    GP32 = 0x4D66FFFFu;                                          /* blue     */

    /* Frame done: the EFB copy that makes it visible. */
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x49u << 24) | 0u;
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x4Au << 24) | (639u | (479u << 10));
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x4Bu << 24) | (0x00600000u >> 5);
    GP8  = GX_LOAD_BP_REG;
    GP32 = (0x52u << 24) | (1u << 14);

    gp_flush();

    ((void(*)(void))0)();
}
