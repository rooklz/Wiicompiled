/* log.c — logging backend. See log.h for why the level test is compile-time. */
#include "log.h"

#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

u8 g_log_level[LOG_CATEGORY_COUNT];

static const char *const k_cat_name[LOG_CATEGORY_COUNT] = {
    "CORE", "MEM", "JIT", "INTERP", "MMU", "VIDEO", "RSX", "SHADER",
    "AUDIO", "DSP", "SPU", "IOS", "DISC", "INPUT", "PERF"
};

static const char *const k_level_name[] = {
    "-", "ERR", "WRN", "INF", "DBG", "TRC"
};

void log_init(void)
{
    unsigned i;
    for (i = 0; i < LOG_CATEGORY_COUNT; i++)
        g_log_level[i] = LOG_LEVEL_INFO;
}

void log_set_level(LogCategory cat, unsigned level)
{
    if (cat < LOG_CATEGORY_COUNT)
        g_log_level[cat] = (u8)level;
}

/* Trim the path so the console shows "memmap.c:214" and not eighty columns of
 * build directory. */
static const char *short_file(const char *path)
{
    const char *p = path, *last = path;
    for (; *p; p++)
        if (*p == '/' || *p == '\\')
            last = p + 1;
    return last;
}

/* Where log lines go. Defaults to fd 2; the PS3 port points it at the report
 * file, because PSL1GHT's fd 2 is a tty channel nobody can read after the
 * fact. One write() per line so a crash cannot swallow the breadcrumb. */
static int s_log_fd = 2;
void log_set_fd(int fd) { if (fd >= 0) s_log_fd = fd; }

/* A second destination, used by the PS3 port to mirror the log down a TCP
 * connection so a developer sees it LIVE instead of fetching a file after the
 * run. -1 when nobody is listening. */
/* A socket on the PS3 is NOT a file descriptor -- lv2 network sockets live in
 * their own namespace and write() cannot reach them, which is why the first
 * attempt attached cleanly and then delivered nothing. The platform supplies
 * a writer instead. */
static void (*s_log_mirror_fn)(const char *, unsigned);
void log_set_mirror(void (*fn)(const char *, unsigned)) { s_log_mirror_fn = fn; }

static void log_vwrite(LogCategory cat, unsigned level, const char *file,
                       int line, const char *fmt, va_list ap)
{
    char buf[512];
    int n = snprintf(buf, sizeof buf, "[%-3s %-6s] %s:%d: ",
            k_level_name[level < DOL_ARRAY_COUNT(k_level_name) ? level : 0],
            (cat < LOG_CATEGORY_COUNT) ? k_cat_name[cat] : "?",
            short_file(file), line);
    if (n < 0) return;
    n += vsnprintf(buf + n, sizeof buf - (size_t)n, fmt, ap);
    if (n > (int)sizeof buf - 2) n = (int)sizeof buf - 2;
    buf[n++] = '\n';
    write(s_log_fd, buf, (size_t)n);
    if (s_log_mirror_fn) s_log_mirror_fn(buf, (unsigned)n);
}

void log_write(LogCategory cat, unsigned level, const char *file, int line,
               const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_vwrite(cat, level, file, line, fmt, ap);
    va_end(ap);
}

void log_write_once(int *seen, LogCategory cat, unsigned level,
                    const char *file, int line, const char *fmt, ...)
{
    va_list ap;
    if (*seen)
        return;
    *seen = 1;
    va_start(ap, fmt);
    log_vwrite(cat, level, file, line, fmt, ap);
    va_end(ap);
    { static const char k_sup[] = "      (further reports from this site suppressed)\n";
      write(s_log_fd, k_sup, sizeof k_sup - 1); }
}

void dol_panic(const char *file, int line, const char *fmt, ...)
{
    va_list ap;
    fprintf(stderr, "\n*** PANIC at %s:%d: ", short_file(file), line);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
    fflush(stderr);
    abort();
}
