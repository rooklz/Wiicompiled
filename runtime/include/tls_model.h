#pragma once

// Thread-local storage model for the variables the indirect-dispatch path touches on every
// guest call (the CpuContext pointer, the diagnostic execution address, the dispatch memos).
//
// The default model for a `thread_local` with external/inline linkage is general-dynamic, which
// on Mach-O resolves through a call to `_tlv_get_addr` on every access - a real function call,
// measured at 1.3% of race CPU by itself, on top of the calls it forces the compiler to make
// around it. All of these live in the main executable, which is loaded before any thread starts,
// so initial-exec is valid and lets the access compile to a direct thread-pointer offset load.
// Only correct for the executable and libraries loaded at startup; never for dlopen()'d code.
#if defined(_WIN32)
#define MKW_TLS_FAST
#elif defined(__clang__) || defined(__GNUC__)
#define MKW_TLS_FAST __attribute__((tls_model("initial-exec")))
#else
#define MKW_TLS_FAST
#endif
