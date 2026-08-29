/* test_realcode.c — run real compiler output through the emulator.
 *
 * The differential suite exercises instructions I chose. This exercises
 * instructions *GCC* chose, at -O2, from ordinary C: CTR loops, update-form
 * addressing, rotate-and-mask bitfield work, 64-bit carry chains on a 32-bit
 * machine, division sequences, and enough live values to force the register
 * cache to evict. None of it was picked to be easy.
 *
 * The oracle is the same source compiled natively and linked into this binary,
 * so "correct" means "produces what the C actually means", not "matches my
 * expectation of what the C means".
 *
 * The guest is called through the 32-bit PowerPC SysV convention: integer
 * arguments in r3.., floating-point in f1.., results in r3 / f1, return address
 * in LR. Execution stops when control reaches a sentinel address holding an
 * illegal instruction, which is how a guest "returns" to us.
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "realtest.h"
#include "../../../build/guest/guest_blob.h"
#include "jit/jit.h"
#include "interp/interp.h"
#include "../mem/memmap.h"
#include "../../common/log.h"

static void rt_printf(const char *fmt, ...) DOL_PRINTF(1, 2);

/* The natively-compiled oracle. */
typedef unsigned char u8_t;
int    gf_checksum(const unsigned char *p, int n);
int    gf_collatz_steps(int n);
unsigned gf_mix64(unsigned a, unsigned b);
unsigned gf_bitfields(unsigned v);
int    gf_sort_sum(int *a, int n);
unsigned gf_widths(unsigned char *p, int n);
float  gf_dot(const float *a, const float *b, int n);
float  gf_transform(float *v, int n, float sx, float sy);
float  gf_clampsum(const float *a, int n, float lo, float hi);
double gf_dsum(const double *a, int n);

#define GUEST_STACK_TOP 0x817F0000u
#define GUEST_SENTINEL  0x80500000u     /* holds an illegal instruction */
#define GUEST_DATA      0x80600000u
#define BUDGET          40000000


/* Output goes through a caller-supplied sink: the console has nowhere
 * convenient to print, and the same suite must run in both places. */
static RealTestOutFn s_out;
static void         *s_ctx;

static void rt_printf(const char *fmt, ...)
{
    static char line[256];
    static unsigned used;
    char buf[256];
    va_list ap;
    char *nl;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    /* The cases print in fragments ending with an explicit newline, so
     * accumulate until one arrives and emit whole lines. */
    for (nl = buf; *nl; nl++) {
        if (*nl == '\n') {
            line[used] = 0;
            s_out(s_ctx, line);
            used = 0;
        } else if (used + 1 < sizeof line) {
            line[used++] = *nl;
        }
    }
}

static int g_fail;
static int g_checks;

#define CHECK(cond, ...)                                                     \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(cond)) { rt_printf("  FAIL: "); rt_printf(__VA_ARGS__);              \
                       rt_printf("\n"); g_fail++; }                             \
    } while (0)

/* ------------------------------------------------------------------ */

static void guest_load(void)
{
    unsigned i;
    for (i = 0; i < GUEST_CODE_SIZE; i++)
        mem_write8(GUEST_CODE_BASE + i, guest_code[i]);
    /* Opcode 0 is illegal, so reaching the sentinel raises a program
     * exception, which stops both engines cleanly. */
    mem_write32(GUEST_SENTINEL, 0);
}

typedef struct {
    u32 gpr[8];         /* r3..r10 */
    f64 fpr[8];         /* f1..f8  */
} GuestArgs;

static void call_setup(PPCState *s, u32 entry, const GuestArgs *a)
{
    unsigned i;
    memset(s, 0, sizeof *s);
    s->msr       = MSR_FP;
    s->hid2      = HID2_PSE | HID2_LSQE;
    s->const_one = 1.0;
    s->fprf_src  = s->fprf_ack = FPRF_SRC_NONE;   /* see ppc_init_constants */
    s->pc        = entry;
    s->lr        = GUEST_SENTINEL;
    s->gpr[1]    = GUEST_STACK_TOP;     /* stack pointer */
    for (i = 0; i < 8; i++) {
        s->gpr[3 + i] = a->gpr[i];
        s->ps[1 + i].ps0.f = a->fpr[i];
    }
    s->downcount = BUDGET;
    s->exit_requested = 0;
}

/* Returns 0 if the guest ran to completion. */
static int call_guest(PPCState *s, u32 entry, const GuestArgs *a, int use_jit)
{
    call_setup(s, entry, a);
    if (use_jit)
        jit_run(s);
    else
        interp_run(s);

    /* The sentinel holds an illegal instruction, so a clean return is a program
     * exception whose SRR0 names the sentinel -- not a program counter sitting
     * near it. That distinction became real when exception delivery was
     * implemented: pc is now the handler vector, and SRR0 is the only thing
     * that still says where control came from. Testing pc against the sentinel
     * would silently accept a guest that faulted anywhere else. */
    if (!(s->pc == VEC_PROGRAM && s->spr[SPR_SRR0] == GUEST_SENTINEL)) {
        rt_printf("  (guest did not return: pc=%08x srr0=%08x downcount=%d)\n",
               (unsigned)s->pc, (unsigned)s->spr[SPR_SRR0], (int)s->downcount);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Cases                                                                */
/* ------------------------------------------------------------------ */

static void write_bytes(u32 addr, const unsigned char *p, unsigned n)
{
    unsigned i;
    for (i = 0; i < n; i++)
        mem_write8(addr + i, p[i]);
}

static void write_floats(u32 addr, const float *p, unsigned n)
{
    unsigned i;
    for (i = 0; i < n; i++) {
        u32 bits;
        memcpy(&bits, &p[i], 4);
        mem_write32(addr + i * 4, bits);
    }
}

static void test_integer(int use_jit)
{
    PPCState s;
    GuestArgs a;
    unsigned char buf[64];
    unsigned i;

    for (i = 0; i < sizeof buf; i++)
        buf[i] = (unsigned char)(i * 37 + 11);
    write_bytes(GUEST_DATA, buf, sizeof buf);

    memset(&a, 0, sizeof a);
    a.gpr[0] = GUEST_DATA;
    a.gpr[1] = (u32)sizeof buf;
    if (call_guest(&s, GA_GF_CHECKSUM, &a, use_jit) == 0)
        CHECK((int)s.gpr[3] == gf_checksum(buf, (int)sizeof buf),
              "gf_checksum: guest=%d native=%d",
              (int)s.gpr[3], gf_checksum(buf, (int)sizeof buf));

    for (i = 1; i <= 27; i += 13) {
        memset(&a, 0, sizeof a);
        a.gpr[0] = i;
        if (call_guest(&s, GA_GF_COLLATZ_STEPS, &a, use_jit) == 0)
            CHECK((int)s.gpr[3] == gf_collatz_steps((int)i),
                  "gf_collatz_steps(%u): guest=%d native=%d",
                  i, (int)s.gpr[3], gf_collatz_steps((int)i));
    }

    memset(&a, 0, sizeof a);
    a.gpr[0] = 0x12345678u; a.gpr[1] = 0x9ABCDEF0u;
    if (call_guest(&s, GA_GF_MIX64, &a, use_jit) == 0)
        CHECK(s.gpr[3] == gf_mix64(0x12345678u, 0x9ABCDEF0u),
              "gf_mix64: guest=%08x native=%08x",
              (unsigned)s.gpr[3], gf_mix64(0x12345678u, 0x9ABCDEF0u));

    memset(&a, 0, sizeof a);
    a.gpr[0] = 0xDEADBEEFu;
    if (call_guest(&s, GA_GF_BITFIELDS, &a, use_jit) == 0)
        CHECK(s.gpr[3] == gf_bitfields(0xDEADBEEFu),
              "gf_bitfields: guest=%08x native=%08x",
              (unsigned)s.gpr[3], gf_bitfields(0xDEADBEEFu));
}

static void test_memory(int use_jit)
{
    PPCState s;
    GuestArgs a;
    int arr[24], ref[24];
    unsigned char bytes[48], refbytes[48];
    unsigned i;

    for (i = 0; i < 24; i++) {
        arr[i] = (int)((i * 7919) % 101) - 50;
        ref[i] = arr[i];
    }
    for (i = 0; i < 24; i++)
        mem_write32(GUEST_DATA + i * 4, (u32)arr[i]);

    memset(&a, 0, sizeof a);
    a.gpr[0] = GUEST_DATA;
    a.gpr[1] = 24;
    if (call_guest(&s, GA_GF_SORT_SUM, &a, use_jit) == 0) {
        int want = gf_sort_sum(ref, 24);
        CHECK((int)s.gpr[3] == want, "gf_sort_sum: guest=%d native=%d",
              (int)s.gpr[3], want);
        /* The sort is in-place, so guest memory must match too -- this catches
         * stores that landed at the wrong address. */
        for (i = 0; i < 24; i++)
            CHECK((int)mem_read32(GUEST_DATA + i * 4) == ref[i],
                  "gf_sort_sum: element %u guest=%d native=%d", i,
                  (int)mem_read32(GUEST_DATA + i * 4), ref[i]);
    }

    for (i = 0; i < sizeof bytes; i++) {
        bytes[i] = (unsigned char)(i * 13 + 7);
        refbytes[i] = bytes[i];
    }
    write_bytes(GUEST_DATA, bytes, sizeof bytes);
    memset(&a, 0, sizeof a);
    a.gpr[0] = GUEST_DATA;
    a.gpr[1] = (u32)sizeof bytes;
    if (call_guest(&s, GA_GF_WIDTHS, &a, use_jit) == 0) {
        unsigned want = gf_widths(refbytes, (int)sizeof refbytes);
        CHECK(s.gpr[3] == want, "gf_widths: guest=%08x native=%08x",
              (unsigned)s.gpr[3], want);
        for (i = 0; i < sizeof bytes; i++)
            CHECK(mem_read8(GUEST_DATA + i) == refbytes[i],
                  "gf_widths: byte %u differs", i);
    }
}

static void test_float(int use_jit)
{
    PPCState s;
    GuestArgs a;
    float va[16], vb[16], vt[16], vref[16];
    double da[8];
    unsigned i;

    for (i = 0; i < 16; i++) {
        va[i] = (float)(i + 1) * 0.5f;
        vb[i] = (float)(16 - i) * 0.25f;
        vt[i] = vref[i] = (float)(i * 3 + 1) * 0.125f;
    }
    write_floats(GUEST_DATA, va, 16);
    write_floats(GUEST_DATA + 64, vb, 16);

    memset(&a, 0, sizeof a);
    a.gpr[0] = GUEST_DATA; a.gpr[1] = GUEST_DATA + 64; a.gpr[2] = 16;
    if (call_guest(&s, GA_GF_DOT, &a, use_jit) == 0) {
        float want = gf_dot(va, vb, 16);
        float got  = (float)s.ps[1].ps0.f;
        CHECK(got == want, "gf_dot: guest=%.9g native=%.9g", got, want);
    }

    write_floats(GUEST_DATA + 128, vt, 16);
    memset(&a, 0, sizeof a);
    a.gpr[0] = GUEST_DATA + 128; a.gpr[1] = 16;
    a.fpr[0] = 1.25; a.fpr[1] = -0.75;
    if (call_guest(&s, GA_GF_TRANSFORM, &a, use_jit) == 0) {
        float want = gf_transform(vref, 16, 1.25f, -0.75f);
        float got  = (float)s.ps[1].ps0.f;
        CHECK(got == want, "gf_transform: guest=%.9g native=%.9g", got, want);
        for (i = 0; i < 16; i++) {
            u32 bits = mem_read32(GUEST_DATA + 128 + i * 4);
            float gv; memcpy(&gv, &bits, 4);
            CHECK(gv == vref[i], "gf_transform: element %u guest=%.9g native=%.9g",
                  i, gv, vref[i]);
        }
    }

    memset(&a, 0, sizeof a);
    a.gpr[0] = GUEST_DATA; a.gpr[1] = 16;
    a.fpr[0] = 1.0; a.fpr[1] = 5.0;
    if (call_guest(&s, GA_GF_CLAMPSUM, &a, use_jit) == 0) {
        float want = gf_clampsum(va, 16, 1.0f, 5.0f);
        float got  = (float)s.ps[1].ps0.f;
        CHECK(got == want, "gf_clampsum: guest=%.9g native=%.9g", got, want);
    }

    for (i = 0; i < 8; i++) {
        da[i] = (double)(i + 1) * 1.375;
        mem_write64(GUEST_DATA + 256 + i * 8, *(const u64 *)&da[i]);
    }
    memset(&a, 0, sizeof a);
    a.gpr[0] = GUEST_DATA + 256; a.gpr[1] = 8;
    if (call_guest(&s, GA_GF_DSUM, &a, use_jit) == 0) {
        double want = gf_dsum(da, 8);
        double got  = s.ps[1].ps0.f;
        CHECK(got == want, "gf_dsum: guest=%.17g native=%.17g", got, want);
    }
}

/* ------------------------------------------------------------------ */

int realtest_run_all(RealTestOutFn out, void *ctx, int *checks, int *failures)
{
    int engine;

    s_out = out;
    s_ctx = ctx;
    g_fail = 0;
    g_checks = 0;

    guest_load();

    rt_printf("real compiler output (%u bytes of PowerPC from GCC -O2)\n",
              GUEST_CODE_SIZE);

    for (engine = 0; engine < 2; engine++) {
        int before = g_fail;
#if defined(__powerpc64__) || defined(__PPC64__)
        rt_printf("  %s:\n", engine ? "recompiler (executing)" : "interpreter");
#else
        rt_printf("  %s:\n", engine ? "recompiler (compiles; host cannot execute)"
                                     : "interpreter");
#endif
        jit_flush_all();
        test_integer(engine);
        test_memory(engine);
        test_float(engine);
        rt_printf("    %s\n", (g_fail == before) ? "ok" : "FAILED");
    }

    if (checks)   *checks = g_checks;
    if (failures) *failures = g_fail;
    return g_fail ? -1 : 0;
}

/* ------------------------------------------------------------------ */
/* Benchmark                                                            */
/* ------------------------------------------------------------------ */

/* Guest instructions a single call consumed.
 *
 * Derived the same way the scheduler derives elapsed time, and for the same
 * reason: the guest stops by running into the sentinel, which raises, which
 * forces an exit -- and a forced exit drives downcount negative after handing
 * the unspent budget to exit_slack. Subtracting only downcount would count the
 * whole remaining budget as executed (gekko.h, exit_slack). */
static u64 consumed_by(const PPCState *s)
{
    s64 n = (s64)BUDGET - (s64)s->downcount - (s64)s->exit_slack;
    return n > 0 ? (u64)n : 0;
}

int realtest_benchmark(int use_jit, unsigned reps, u64 *guest_insts)
{
    PPCState s;
    GuestArgs a;
    unsigned char bytes[64];
    u64 total = 0;
    unsigned r, i;

    guest_load();

    for (i = 0; i < sizeof bytes; i++)
        bytes[i] = (unsigned char)(i * 37 + 11);

    for (r = 0; r < reps; r++) {
        /* Re-seeded every rep so the sort always has work to do; a sort given
         * already-sorted input measures the best case and nothing else. */
        static const s32 seed[16] = { 9, 3, 14, 1, 15, 92, 6, 5,
                                      35, 8, 97, 93, 2, 38, 46, 26 };
        for (i = 0; i < 16; i++)
            mem_write32(GUEST_DATA + i * 4, (u32)seed[i]);

        memset(&a, 0, sizeof a);
        a.gpr[0] = GUEST_DATA;
        a.gpr[1] = 16;
        if (call_guest(&s, GA_GF_SORT_SUM, &a, use_jit) != 0) return -1;
        total += consumed_by(&s);

        memset(&a, 0, sizeof a);
        a.gpr[0] = GUEST_DATA;
        a.gpr[1] = (u32)sizeof bytes;
        if (call_guest(&s, GA_GF_CHECKSUM, &a, use_jit) != 0) return -1;
        total += consumed_by(&s);

        memset(&a, 0, sizeof a);
        a.gpr[0] = (u32)(r + 1);
        if (call_guest(&s, GA_GF_MIX64, &a, use_jit) != 0) return -1;
        total += consumed_by(&s);

        memset(&a, 0, sizeof a);
        a.gpr[0] = GUEST_DATA;
        a.gpr[1] = 16;
        if (call_guest(&s, GA_GF_TRANSFORM, &a, use_jit) != 0) return -1;
        total += consumed_by(&s);
    }

    if (guest_insts)
        *guest_insts = total;
    return 0;
}
