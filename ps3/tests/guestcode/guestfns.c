/* guestfns.c — ordinary C, compiled twice.
 *
 * Once for 32-bit PowerPC (the guest) and once natively (the oracle). The
 * emulator runs the PowerPC build; the results are compared against the native
 * build of the *same source*.
 *
 * This is a materially stronger test than hand-written instruction cases,
 * because those only cover what the author thought to cover. A compiler at -O2
 * emits what it likes: CTR loops, update-form addressing, rotate-and-mask
 * bitfield work, carry chains, conditional moves via `fsel`, multiply/divide
 * sequences, and register pressure high enough to force the JIT's cache to
 * evict. None of that was chosen by me.
 *
 * Constraints: no libc, no globals, no floating-point library calls -- these
 * run against a bare guest with a stack and nothing else.
 */

typedef unsigned char      u8;
typedef unsigned short     u16;
typedef unsigned int       u32;
typedef signed int         s32;
typedef unsigned long long u64;

/* ---- integer: shifts, rotates, carry, byte access ---- */

int gf_checksum(const u8 *p, int n)
{
    int s = 0, i;
    for (i = 0; i < n; i++)
        s = (s << 3) ^ (s >> 5) ^ p[i];
    return s;
}

/* Exercises multiply, divide and the modulo sequence a compiler synthesizes
 * from them -- divide is one of the slowest paths on the guest and one of the
 * easiest to get wrong at the edges. */
int gf_collatz_steps(int n)
{
    int steps = 0;
    while (n != 1 && steps < 1000) {
        n = (n % 2 == 0) ? n / 2 : 3 * n + 1;
        steps++;
    }
    return steps;
}

/* 64-bit arithmetic on a 32-bit machine: the compiler emits addc/adde carry
 * chains, which the JIT currently declines and hands to the interpreter. This
 * checks that hand-off in real code rather than in isolation. */
u32 gf_mix64(u32 a, u32 b)
{
    u64 x = ((u64)a << 32) | b;
    x ^= x >> 33;
    x *= 0xFF51AFD7ED558CCDull;
    x ^= x >> 29;
    return (u32)(x ^ (x >> 32));
}

/* Bitfield work: rlwinm/rlwimi territory. */
u32 gf_bitfields(u32 v)
{
    u32 r = 0;
    r |= (v & 0x000000FFu) << 24;
    r |= (v & 0x0000FF00u) << 8;
    r |= (v & 0x00FF0000u) >> 8;
    r |= (v & 0xFF000000u) >> 24;
    r ^= (v >> 3) | (v << 29);
    return r;
}

/* ---- memory: loads, stores, branches, register pressure ---- */

int gf_sort_sum(s32 *a, int n)
{
    int i, j, sum = 0;
    for (i = 1; i < n; i++) {          /* insertion sort */
        s32 key = a[i];
        j = i - 1;
        while (j >= 0 && a[j] > key) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
    for (i = 0; i < n; i++)
        sum += a[i] * (i + 1);
    return sum;
}

/* Mixed access widths, which land on lbz/lhz/lwz and their store counterparts. */
u32 gf_widths(u8 *p, int n)
{
    u32 acc = 0;
    int i;
    for (i = 0; i + 4 <= n; i += 4) {
        u16 h = (u16)((p[i] << 8) | p[i + 1]);
        acc += h;
        acc ^= (u32)p[i + 2] << 16;
        p[i + 3] = (u8)(acc & 0xFF);
    }
    return acc;
}

/* ---- floating point ---- */

/* Dot product and normalization: fmuls/fadds/fdivs, and the single<->double
 * conversions on every load and store, which is the path that costs an x86
 * backend a software routine per access and costs this one nothing. */
float gf_dot(const float *a, const float *b, int n)
{
    float s = 0.0f;
    int i;
    for (i = 0; i < n; i++)
        s += a[i] * b[i];
    return s;
}

float gf_transform(float *v, int n, float sx, float sy)
{
    float acc = 0.0f;
    int i;
    for (i = 0; i + 2 <= n; i += 2) {
        float x = v[i] * sx + sy;
        float y = v[i + 1] * sy - sx;
        v[i] = x;
        v[i + 1] = y;
        acc += x * y;
    }
    return acc;
}

/* Comparisons and selects: the compiler may use fsel here, which has the
 * unusual >= 0 semantic that makes -0.0 select the true branch. */
float gf_clampsum(const float *a, int n, float lo, float hi)
{
    float s = 0.0f;
    int i;
    for (i = 0; i < n; i++) {
        float x = a[i];
        if (x < lo) x = lo;
        if (x > hi) x = hi;
        s += x;
    }
    return s;
}

/* Double precision, to cover lfd/stfd and the non-single arithmetic forms. */
double gf_dsum(const double *a, int n)
{
    double s = 0.0;
    int i;
    for (i = 0; i < n; i++)
        s = s * 1.5 + a[i];
    return s;
}
