/* log.h — logging with compile-time elimination.
 *
 * On a 3.2 GHz in-order core with a 24-cycle branch mispredict penalty, a
 * logging call left in a hot path is not free even when it does nothing: it
 * costs an instruction fetch, a load, a test and a branch the predictor must
 * learn. So the level check is *compile-time* — below LOG_LEVEL_COMPILED the
 * call vanishes entirely, arguments unevaluated, leaving no code at all.
 *
 * The release build sets LOG_LEVEL_COMPILED to LOG_LEVEL_WARN, which removes
 * every LOG_DEBUG/LOG_TRACE site from the emulator core.
 */
#ifndef DOLPHIN_COMMON_LOG_H
#define DOLPHIN_COMMON_LOG_H

#include "types.h"

/* ------------------------------------------------------------------ */
/* Levels                                                               */
/* ------------------------------------------------------------------ */

#define LOG_LEVEL_NONE   0
#define LOG_LEVEL_ERROR  1
#define LOG_LEVEL_WARN   2
#define LOG_LEVEL_INFO   3
#define LOG_LEVEL_DEBUG  4
#define LOG_LEVEL_TRACE  5

#ifndef LOG_LEVEL_COMPILED
#  ifdef NDEBUG
#    define LOG_LEVEL_COMPILED LOG_LEVEL_WARN
#  else
#    define LOG_LEVEL_COMPILED LOG_LEVEL_DEBUG
#  endif
#endif

/* ------------------------------------------------------------------ */
/* Categories                                                           */
/* ------------------------------------------------------------------ */

typedef enum {
    LOG_CORE = 0,
    LOG_MEM,
    LOG_JIT,
    LOG_INTERP,
    LOG_MMU,
    LOG_VIDEO,
    LOG_RSX,
    LOG_SHADER,
    LOG_AUDIO,
    LOG_DSP,
    LOG_SPU,
    LOG_IOS,
    LOG_DISC,
    LOG_INPUT,
    LOG_PERF,
    LOG_CATEGORY_COUNT
} LogCategory;

/* Per-category runtime level, for turning one subsystem up without
 * recompiling. Never rises above LOG_LEVEL_COMPILED, which is the hard ceiling. */
extern u8 g_log_level[LOG_CATEGORY_COUNT];

void log_init(void);
void log_set_level(LogCategory cat, unsigned level);
void log_set_fd(int fd);
void log_set_mirror(void (*fn)(const char *line, unsigned len));
void log_write(LogCategory cat, unsigned level, const char *file, int line,
               const char *fmt, ...) DOL_PRINTF(5, 6);

/* Emits at most once per call site. Used for "unhandled hardware register"
 * style messages, where a title may hit the same path a million times and the
 * hundredth report tells us nothing the first did not. */
void log_write_once(int *seen, LogCategory cat, unsigned level,
                    const char *file, int line, const char *fmt, ...)
    DOL_PRINTF(6, 7);

/* ------------------------------------------------------------------ */
/* Macros                                                              */
/* ------------------------------------------------------------------ */

#define LOG_AT(cat, level, ...)                                              \
    do {                                                                     \
        if ((level) <= LOG_LEVEL_COMPILED &&                                 \
            UNLIKELY((level) <= g_log_level[(cat)]))                         \
            log_write((cat), (level), __FILE__, __LINE__, __VA_ARGS__);      \
    } while (0)

#define LOG_ERROR(cat, ...) LOG_AT(cat, LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_WARN(cat, ...)  LOG_AT(cat, LOG_LEVEL_WARN,  __VA_ARGS__)
#define LOG_INFO(cat, ...)  LOG_AT(cat, LOG_LEVEL_INFO,  __VA_ARGS__)
#define LOG_DEBUG(cat, ...) LOG_AT(cat, LOG_LEVEL_DEBUG, __VA_ARGS__)
#define LOG_TRACE(cat, ...) LOG_AT(cat, LOG_LEVEL_TRACE, __VA_ARGS__)

#define LOG_AT_ONCE(cat, level, ...)                                         \
    do {                                                                     \
        if ((level) <= LOG_LEVEL_COMPILED) {                                 \
            static int dol_log_seen_ = 0;                                    \
            if (UNLIKELY(!dol_log_seen_ && (level) <= g_log_level[(cat)]))   \
                log_write_once(&dol_log_seen_, (cat), (level),               \
                               __FILE__, __LINE__, __VA_ARGS__);             \
        }                                                                    \
    } while (0)

#define LOG_ERROR_ONCE(cat, ...) LOG_AT_ONCE(cat, LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_WARN_ONCE(cat, ...)  LOG_AT_ONCE(cat, LOG_LEVEL_WARN,  __VA_ARGS__)
#define LOG_INFO_ONCE(cat, ...)  LOG_AT_ONCE(cat, LOG_LEVEL_INFO,  __VA_ARGS__)

/* Assertions. Kept in release builds for invariants whose violation would
 * corrupt guest state silently; the emulator is far easier to debug when it
 * stops at the broken invariant than three subsystems later. */
DOL_NORETURN void dol_panic(const char *file, int line, const char *fmt, ...)
    DOL_PRINTF(3, 4);

#define PANIC(...) dol_panic(__FILE__, __LINE__, __VA_ARGS__)

#define ASSERT(cond)                                                         \
    do {                                                                     \
        if (UNLIKELY(!(cond)))                                               \
            dol_panic(__FILE__, __LINE__, "assertion failed: %s", #cond);    \
    } while (0)

#ifdef NDEBUG
#  define DEBUG_ASSERT(cond) ((void)0)
#else
#  define DEBUG_ASSERT(cond) ASSERT(cond)
#endif

#endif /* DOLPHIN_COMMON_LOG_H */
