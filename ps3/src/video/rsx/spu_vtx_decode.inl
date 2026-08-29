/* spu_vtx_decode.inl — the vertex decode itself, with no DMA in it.
 *
 * Included BOTH by the SPU program (spu/vtx_spu.c, which wraps it in MFC
 * transfers) and by the host test (tests/test_spuvtx.c, which feeds it plain
 * pointers and diffs every float against vtx_decode). The decode that ships
 * is therefore literally the decode that is tested -- the alternative, a
 * reimplementation in the test, can agree with itself while both disagree
 * with the PPU path.
 *
 * All reads are explicit big-endian byte assembly: the guest is big-endian
 * and so are both processors, but spelling it out keeps the file honest and
 * portable to a host test compiled anywhere. */


/* Wide reads without byte-by-byte assembly on a big-endian machine.
 *
 * The SPU has no byte load: `p[0]` is a quadword load, a rotate and a mask, so
 * assembling a 32-bit field from four of them costs upwards of twenty
 * instructions -- to produce a value whose four bytes were already adjacent
 * and already in the right order, because guest, PPE and SPE are all
 * big-endian. The packed struct says "unaligned load of this width" and lets
 * the compiler emit its own sequence, which is a few instructions.
 *
 * The byte-assembling form is kept for little-endian builds: the host
 * differential test compiles this same file on a workstation, where the bytes
 * genuinely do need reordering. Both forms return the identical value, so the
 * two paths cannot disagree. */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
struct sv_un16 { uint16_t v; } __attribute__((packed));
struct sv_un32 { uint32_t v; } __attribute__((packed));
#define SV_BE_LOADS 1
#endif

static inline uint16_t sv_rd16(const uint8_t *p)
{
#ifdef SV_BE_LOADS
    return ((const struct sv_un16 *)p)->v;
#else
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
#endif
}
static inline uint32_t sv_rd32(const uint8_t *p)
{
#ifdef SV_BE_LOADS
    return ((const struct sv_un32 *)p)->v;
#else
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
#endif
}
static inline float sv_rdf32(const uint8_t *p)
{ union { uint32_t u; float f; } v; v.u = sv_rd32(p); return v.f; }

static float sv_frac_scale(unsigned frac)
{
    static const float k[32] = {
        1.0f/(1u<<0),  1.0f/(1u<<1),  1.0f/(1u<<2),  1.0f/(1u<<3),
        1.0f/(1u<<4),  1.0f/(1u<<5),  1.0f/(1u<<6),  1.0f/(1u<<7),
        1.0f/(1u<<8),  1.0f/(1u<<9),  1.0f/(1u<<10), 1.0f/(1u<<11),
        1.0f/(1u<<12), 1.0f/(1u<<13), 1.0f/(1u<<14), 1.0f/(1u<<15),
        1.0f/(1u<<16), 1.0f/(1u<<17), 1.0f/(1u<<18), 1.0f/(1u<<19),
        1.0f/(1u<<20), 1.0f/(1u<<21), 1.0f/(1u<<22), 1.0f/(1u<<23),
        1.0f/(1u<<24), 1.0f/(1u<<25), 1.0f/(1u<<26), 1.0f/(1u<<27),
        1.0f/(1u<<28), 1.0f/(1u<<29), 1.0f/(1u<<30), 1.0f/(1u<<31)
    };
    return k[frac & 31u];
}

static float sv_dequant(const uint8_t *p, unsigned fmt, unsigned frac,
                        unsigned i)
{
    /* Table, not a divide. `frac` is a shift count, so every scale is an exact
     * power of two and the table is bit-identical to the division it replaces
     * -- but this ran once per COMPONENT per vertex, and the SPU has no divide
     * instruction: it is a reciprocal estimate plus Newton-Raphson every
     * time. */
    float scale = sv_frac_scale(frac);
    switch (fmt) {
    case SVF_U8:  return (float)p[i] * scale;
    case SVF_S8:  return (float)(signed char)p[i] * scale;
    case SVF_U16: return (float)sv_rd16(p + i*2) * scale;
    case SVF_S16: return (float)(short)sv_rd16(p + i*2) * scale;
    default:      return sv_rdf32(p + i*4);
    }
}
static unsigned sv_fmt_bytes(unsigned fmt)
{ return fmt <= SVF_S8 ? 1u : fmt <= SVF_S16 ? 2u : 4u; }

static void sv_col_decode(const uint8_t *p, unsigned fmt, float *rgba)
{
    switch (fmt) {
    case SVC_RGB565: { uint16_t v = sv_rd16(p);
        rgba[0]=(float)((v>>11)&31)/31.0f; rgba[1]=(float)((v>>5)&63)/63.0f;
        rgba[2]=(float)(v&31)/31.0f; rgba[3]=1.0f; break; }
    case SVC_RGB8: case SVC_RGBX8:
        rgba[0]=p[0]/255.0f; rgba[1]=p[1]/255.0f; rgba[2]=p[2]/255.0f;
        rgba[3]=1.0f; break;
    case SVC_RGBA4: { uint16_t v = sv_rd16(p);
        rgba[0]=(float)((v>>12)&15)/15.0f; rgba[1]=(float)((v>>8)&15)/15.0f;
        rgba[2]=(float)((v>>4)&15)/15.0f;  rgba[3]=(float)(v&15)/15.0f; break; }
    case SVC_RGBA6: { uint32_t v=((uint32_t)p[0]<<16)|((uint32_t)p[1]<<8)|p[2];
        rgba[0]=(float)((v>>18)&63)/63.0f; rgba[1]=(float)((v>>12)&63)/63.0f;
        rgba[2]=(float)((v>>6)&63)/63.0f;  rgba[3]=(float)(v&63)/63.0f; break; }
    default:
        rgba[0]=p[0]/255.0f; rgba[1]=p[1]/255.0f;
        rgba[2]=p[2]/255.0f; rgba[3]=p[3]/255.0f; break;
    }
}
static unsigned sv_col_bytes(unsigned fmt)
{
    switch (fmt) {
    case SVC_RGB565: case SVC_RGBA4: return 2u;
    case SVC_RGB8:   case SVC_RGBA6: return 3u;
    default:                         return 4u;
    }
}

/* Decode j->vert_count vertices from `stream` into RenderVertex floats.
 * windows[a] is the base of attribute a's indexed array (already rebased so
 * that element base_idx sits at offset 0), or NULL for direct attributes. */
static void sv_decode_verts(const SpuVtxJob *j, const uint8_t *stream,
                            const uint8_t *const *windows, float *out)
{
    unsigned v, a, t;
    for (v = 0; v < j->vert_count; v++,
         stream += j->vert_size, out += SPU_RV_FLOATS) {
        const uint8_t *sp = stream;

        /* Defaults must match the PPU path in gx_render.c exactly: absent
         * colour is opaque white, absent normal points at the eye, absent
         * texcoord is (0,0) with q=1. */
        out[0]=out[1]=out[2]=0.0f;
        out[3]=0.0f; out[4]=0.0f; out[5]=1.0f;
        out[6]=out[7]=out[8]=out[9]=1.0f;
        for (t = 0; t < 4; t++) {
            out[10+t*3]=0.0f; out[11+t*3]=0.0f; out[12+t*3]=1.0f;
        }

        for (a = 0; a < j->nattrs; a++) {
            const SpuVtxAttr *at = &j->attr[a];
            const uint8_t *src;
            if (at->kind == SVA_END) break;
            if (at->kind == SVA_PNMTXIDX || at->kind == SVA_TEXMTXIDX) {
                sp += 1; continue;
            }
            if (at->mode == SVM_DIRECT) {
                src = sp;
            } else {
                uint32_t idx = (at->mode == SVM_IDX8) ? (uint32_t)*sp
                                                      : sv_rd16(sp);
                sp += (at->mode == SVM_IDX8) ? 1u : 2u;
                idx -= at->base_idx;
                src = windows[a] + (size_t)idx * at->stride;
            }
            switch (at->kind) {
            case SVA_POS: {
                unsigned i, n = at->count;
                for (i = 0; i < n && i < 3u; i++)
                    out[i] = sv_dequant(src, at->fmt, at->frac, i);
                if (at->mode == SVM_DIRECT) sp += n * sv_fmt_bytes(at->fmt);
                break; }
            case SVA_NRM: {
                unsigned i, n = at->count;
                for (i = 0; i < 3u && i < n; i++)
                    out[3+i] = sv_dequant(src, at->fmt, at->frac, i);
                if (at->mode == SVM_DIRECT) sp += n * sv_fmt_bytes(at->fmt);
                break; }
            case SVA_COL0:
                sv_col_decode(src, at->fmt, &out[6]);
                if (at->mode == SVM_DIRECT) sp += sv_col_bytes(at->fmt);
                break;
            case SVA_COL1:
                if (at->mode == SVM_DIRECT) sp += sv_col_bytes(at->fmt);
                break;
            default: {
                unsigned i, n = at->count;
                unsigned tn = (unsigned)(at->kind - SVA_TEX0);
                if (tn < 4u)
                    for (i = 0; i < n && i < 2u; i++)
                        out[10 + tn*3 + i] =
                            sv_dequant(src, at->fmt, at->frac, i);
                if (at->mode == SVM_DIRECT) sp += n * sv_fmt_bytes(at->fmt);
                break; }
            }
        }
    }
}
