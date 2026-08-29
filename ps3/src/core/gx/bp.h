/* bp.h — the pixel engine's register file ("BP memory").
 *
 * Everything about how a pixel is produced arrives as BP register writes: how
 * many TEV stages combine textures and colours, what the depth and blend
 * functions are, which textures are bound and where they live, and — the one
 * that makes anything appear on screen — when to copy the embedded framebuffer
 * out to main memory.
 *
 * The design decision here is that *all 256 registers are recorded raw* and
 * only the ones a renderer acts on are decoded. That is not laziness. The
 * alternative — decode everything up front — means a register we have not
 * studied yet is dropped on the floor, and a dropped BP write is invisible:
 * the frame renders, slightly wrong, with nothing to point at. Keeping the raw
 * word means the state is always complete and decoding can catch up later,
 * which is the right way round for a subsystem this large.
 *
 * Register numbers are the hardware's, and are used rather than an internal
 * enum so this file can be read next to a FIFO capture.
 */
#ifndef DOLPHIN_CORE_GX_BP_H
#define DOLPHIN_CORE_GX_BP_H

#include "../ppc/gekko.h"

#define BP_NUM_REGS         256
#define BP_MAX_TEV_STAGES   16
#define BP_NUM_TEXTURES     8

/* ------------------------------------------------------------------ */
/* Register numbers                                                     */
/* ------------------------------------------------------------------ */

#define BP_GENMODE          0x00
#define BP_SCISSOR_TL       0x20
#define BP_SCISSOR_BR       0x21
#define BP_LINE_PT_WIDTH    0x22
#define BP_TEV_REF          0x28    /* + 0..7: two stages each          */
#define BP_ZMODE            0x40
#define BP_BLENDMODE        0x41
#define BP_CONSTANTALPHA    0x42
#define BP_ZCOMPARE         0x43   /* PE_CONTROL: pixel format, z format */
#define BP_SET_DRAWDONE     0x45
#define BP_PE_TOKEN         0x47
#define BP_PE_TOKEN_INT     0x48
#define BP_EFB_TL           0x49
#define BP_EFB_WH           0x4A
#define BP_EFB_DEST_ADDR    0x4B
#define BP_EFB_STRIDE       0x4D
#define BP_COPY_YSCALE      0x4E
#define BP_CLEAR_AR         0x4F
#define BP_CLEAR_GB         0x50
#define BP_CLEAR_Z          0x51
#define BP_TRIGGER_EFB_COPY 0x52
#define BP_SCISSOR_OFFSET   0x59
#define BP_TEX_INVALIDATE   0x68
#define BP_FIELDMODE        0x69
#define BP_TX_SETMODE0      0x80    /* + 0..3 : textures 0-3            */
#define BP_TX_SETMODE1      0x84
#define BP_TX_SETIMAGE0     0x88
#define BP_TX_SETIMAGE1     0x8C
#define BP_TX_SETIMAGE2     0x90
#define BP_TX_SETIMAGE3     0x94
#define BP_TX_SETTLUT       0x98
#define BP_TX_SETMODE0_4    0xA0    /* + 0..3 : textures 4-7            */
#define BP_TX_SETMODE1_4    0xA4
#define BP_TX_SETIMAGE0_4   0xA8
#define BP_TX_SETIMAGE1_4   0xAC
#define BP_TX_SETIMAGE2_4   0xB0
#define BP_TX_SETIMAGE3_4   0xB4
#define BP_TX_SETTLUT_4     0xB8
#define BP_IND_MTX          0x06    /* + 0..8: three 2x3 matrices        */
#define BP_RAS1_SS0         0x25    /* indirect coordinate scale, 2 stages */
#define BP_RAS1_SS1         0x26
#define BP_RAS1_IREF        0x27    /* which map/coord each ind stage reads */
#define BP_TEV_COLOR_ENV    0xC0    /* colour/alpha interleaved, 16 stages */
#define BP_TEV_ALPHA_ENV    0xC1
#define BP_TEV_REGISTER_L   0xE0    /* + 0..7: four registers, lo/hi    */
#define BP_IND_CMD          0x10    /* + 0..15: one per TEV stage        */
#define BP_FOGRANGE         0xE8    /* + 0..5: base and five K pairs     */
#define BP_FOGPARAM0        0xEE    /* A: 11-bit mantissa + 8-bit exp    */
#define BP_FOGBMAGNITUDE    0xEF
#define BP_FOGBEXPONENT     0xF0
#define BP_FOGPARAM3        0xF1    /* C, projection and fog type        */
#define BP_FOGCOLOR         0xF2
#define BP_ALPHACOMPARE     0xF3
#define BP_TEV_KSEL         0xF6    /* + 0..7                            */
#define BP_BP_MASK          0xFE

/* ------------------------------------------------------------------ */
/* Decoded state                                                        */
/* ------------------------------------------------------------------ */

typedef enum {
    BP_CULL_NONE = 0, BP_CULL_BACK = 1, BP_CULL_FRONT = 2, BP_CULL_ALL = 3
} BPCullMode;

typedef struct {
    unsigned num_texgens;       /* 0..8  */
    unsigned num_colorchans;    /* 0..2  */
    unsigned num_tev_stages;    /* 1..16 */
    unsigned num_indstages;     /* 0..4  */
    BPCullMode cull;
    unsigned zfreeze;
} BPGenMode;

typedef struct {
    unsigned enable;
    unsigned func;              /* 0..7, GX compare function             */
    unsigned update_enable;     /* writes to the depth buffer            */
} BPZMode;

typedef struct {
    unsigned blend_enable;
    unsigned logic_enable;
    unsigned dither;
    unsigned color_update;
    unsigned alpha_update;
    unsigned dst_factor;        /* 0..7 */
    unsigned src_factor;        /* 0..7 */
    unsigned subtract;
    unsigned logic_op;          /* 0..15 */
} BPBlendMode;

typedef struct {
    unsigned ref0, ref1;
    unsigned comp0, comp1;      /* 0..7 */
    unsigned logic;             /* 0 = and, 1 = or, 2 = xor, 3 = xnor    */
} BPAlphaTest;

/* One TEV stage: a colour combiner and an alpha combiner, plus the "order"
 * fields that say which texture and which rasterised colour it reads. The
 * order fields are packed two stages to a register, which is why they are
 * decoded into a per-stage form here. */
typedef struct {
    u32 color_env;
    u32 alpha_env;
    unsigned tex_map;
    unsigned tex_coord;
    unsigned tex_enable;
    unsigned ras_channel;
    /* Konst selection, decoded from the KSEL registers rather than read there
     * by every consumer. KSEL packs *two* stages into one register -- the even
     * stage in bits 4..13, the odd one in 14..23 -- and mixing the halves up
     * gives a stage the neighbouring stage's constant, which is a wrong colour
     * on exactly half the geometry and nothing pointing at why. */
    unsigned konst_color;       /* 0..31, see GX's KonstSel encoding     */
    unsigned konst_alpha;       /* 0..31                                 */
} BPTevStage;

/* One TEV stage's indirect command (BP 0x10 + n).
 *
 * The indirect unit is a second, smaller texture pipeline that runs *before*
 * the ordinary one: it samples a texture, treats the result as a vector of
 * three signed numbers, transforms it by a small matrix and adds the result to
 * the coordinate the stage was about to sample with. That is how a race gets
 * heat haze, water ripples, the road's puddle distortion and the tyre marks
 * -- 19% of a race's draws configure one.
 *
 * The fields that look alike but are not: `matrix_index` says WHICH of the
 * three matrices to use (0 means no offset at all), and `matrix_id` says HOW
 * -- 0 the ordinary matrix multiply, 1 and 2 the two "dynamic" forms that
 * scale the stage's own coordinate by one component of the indirect sample.
 * They are adjacent two-bit fields and swapping them gives a stage that
 * offsets by a plausible-looking wrong amount. */
typedef struct {
    unsigned raw;               /* the register, for hashing and auditing */
    unsigned ind_stage;         /* 0..3: which indirect lookup feeds this  */
    unsigned format;            /* 0..3: ITF_8, ITF_5, ITF_4, ITF_3        */
    unsigned bias;              /* bit per axis: 1 = S, 2 = T, 4 = U       */
    unsigned bump_alpha;        /* 0 off, 1 S, 2 T, 3 U                    */
    unsigned matrix_index;      /* 0 off, 1..3 select a matrix             */
    unsigned matrix_id;         /* 0 matrix, 1 dynamic S, 2 dynamic T      */
    unsigned wrap_s, wrap_t;    /* 0 off, 1..5 = 256..16 texels, 6/7 = 0   */
    unsigned lod_unmodified;    /* LOD from the unmodified coordinate      */
    unsigned add_prev;          /* add the previous stage's coordinate     */
} BPTevIndirect;

/* One indirect matrix: two rows of three 11-bit fixed-point values with ten
 * fractional bits, plus a shared exponent. The exponent is split two bits to a
 * register across the three registers the matrix occupies, which is the field
 * most easily lost -- and losing it scales every offset by a power of two. */
typedef struct {
    int      m[2][3];           /* raw 11-bit signed; divide by 1024       */
    unsigned scale;             /* 0..31 exponent, applied as 2^(s - 17)   */
} BPIndMatrix;

typedef struct {
    u32 mode0, mode1;
    u32 image0, image1, image2, image3;
    u32 tlut;                   /* TX_SETTLUT: TMEM offset and format      */
    /* Decoded from image0/image3 because a renderer needs them constantly. */
    unsigned width, height;     /* 1-based                                */
    unsigned format;
    u32 address;                /* physical, already shifted from the register */
} BPTexture;

/* An EFB copy: the moment pixels leave the embedded framebuffer for main
 * memory. Either to the external framebuffer the video interface scans out
 * (`to_xfb`), or to a texture the title will sample later. */
typedef struct {
    unsigned src_x, src_y, width, height;
    u32      dest_addr;
    unsigned dest_stride;
    unsigned to_xfb;
    unsigned clear;             /* clear the EFB as part of the copy      */
    /* The two scaling controls, which are separate bits and mean different
     * things. `half_scale` is the mipmap path: 2x2 EFB pixels are averaged
     * into one destination texel, so the destination is half the size of the
     * source rectangle. `scale_invert` selects whether the Y-scale register
     * is used as a numerator or a denominator, and only applies to copies
     * that scale vertically (i.e. XFB copies). */
    unsigned half_scale;
    unsigned scale_invert;
    unsigned format;            /* copy format, unrotated: see bp.c          */
    unsigned format_raw;        /* the register field as written             */
    unsigned intensity;         /* luminance arithmetic on the way out      */
    unsigned gamma;             /* 0 = 1.0, 1 = 1.7, 2 = 2.2, 3 = invalid   */
    u32      clear_color;       /* ARGB, assembled from CLEAR_AR/CLEAR_GB */
    u32      clear_z;
} BPCopy;

typedef struct {
    /* Complete, always. Decoding is a view onto this, never a replacement. */
    u32 raw[BP_NUM_REGS];

    BPGenMode    genmode;
    BPZMode      zmode;
    BPBlendMode  blend;
    unsigned     pe_control;    /* raw BP 0x43: bits 0..2 pixel format
                                 * (0 RGB8_Z24, 1 RGBA6_Z24, 2 RGB565_Z16),
                                 * bits 3..5 z format */
    BPAlphaTest  alpha_test;
    BPTevStage   tev[BP_MAX_TEV_STAGES];
    BPTexture    tex[BP_NUM_TEXTURES];

    /* Indirect texturing. Decoded rather than left raw because the fragment
     * program generator acts on every field of it. */
    BPTevIndirect tevind[BP_MAX_TEV_STAGES];
    BPIndMatrix   ind_mtx[3];
    struct {
        unsigned map, coord;    /* RAS1_IREF: what the lookup samples      */
        unsigned scale_s;       /* RAS1_SS: coordinate is divided by 2^s   */
        unsigned scale_t;
    } ind_stage[4];

    /* TEV register colours, decoded from 0xE0-0xE7. Each register is a lo/hi
     * pair -- red+alpha then blue+green, 11-bit fields -- and bit 23 of the
     * write selects the konstant bank instead of the ordinary one. The
     * material colours of everything on screen live here; a fragment program
     * that reads them uninitialised multiplies the whole frame by zero. */
    float        tev_reg[4][4];     /* [reg][rgba], 0..1                  */
    float        tev_konst[4][4];

    /* The four TEV swap tables, each naming which input channel each output
     * channel comes from. Packed two per KSEL register (the even register
     * carries red and green, the odd one blue and alpha), which is why they
     * are decoded here rather than at every use. The reset value is the
     * identity, because a swap table nobody has written must not permute. */
    unsigned     tev_swap[4][4];    /* [table][rgba] -> source channel    */

    /* Fog, decoded from 0xE8..0xF2. `fsel` 0 means no fog at all, which is
     * the state the whole title screen runs in. */
    struct {
        unsigned fsel;              /* 0 off, 2 linear, 4/5 exp, 6/7 rev  */
        unsigned projection;        /* 0 perspective, 1 orthographic      */
        unsigned range_enable;      /* the x-axis range adjustment        */
        int      range_center;      /* viewport centre, +342 bias removed */
        unsigned b_magnitude;
        unsigned b_shift;
        float    a, c;              /* the two 11-bit floats             */
        float    color[3];          /* RGB, 0..1                          */
        float    range_k[10];       /* the adjustment table               */
    } fog;

    /* Screen space: the hardware's +342 bias is already removed, so these are
     * signed -- a title can legitimately scissor to a rectangle whose left edge
     * is negative. */
    int scissor_left, scissor_top, scissor_right, scissor_bottom;
    int      scissor_offset_x, scissor_offset_y;

    /* The last copy requested, and a counter so a frontend can tell that one
     * happened without polling every field. */
    BPCopy   copy;
    u64      copies;
    u64      draw_done;         /* BP_SET_DRAWDONE writes                 */
    u64      tokens;            /* BP_PE_TOKEN writes                     */
    u16      last_token;
    /* Texture palette memory. A title loads a palette with the TLUT load
     * command (BP 0x64/0x65), which copies entries out of main memory into
     * this area; a paletted texture then indexes it. Without it the C4/C8/
     * C14X2 formats cannot be decoded at all -- they were previously refused,
     * which is why palette-using content (the attract movie among it) drew as
     * a flat error colour. 0x2000 16-bit entries covers the hardware's TLUT
     * region. */
    u16 tlut_mem[0x2000];
    u32 tlut_src;               /* latched by BP 0x64, consumed by 0x65    */
    int tlut_loaded;            /* a palette has been copied in            */
    u16 (*read16)(u32 addr);    /* supplied by the embedder; may be NULL   */

} BPState;

/* ------------------------------------------------------------------ */

void bp_reset(BPState *bp);

/* Apply one BP register write. `value` is the low 24 bits of the command word;
 * the register number is the top byte, already separated by the parser. */
void bp_write(BPState *bp, u8 reg, u32 value);

/* Convenience for a renderer: how many TEV stages are actually active. The
 * register stores count - 1, and reading it raw is a classic off-by-one. */
unsigned bp_tev_stage_count(const BPState *bp);

/* Install the pixel-engine signalling hooks (hw/pe.c does this at init). */
void bp_set_pe_hooks(void (*finish)(void),
                     void (*token)(u16 token, int with_interrupt));

#endif /* DOLPHIN_CORE_GX_BP_H */
