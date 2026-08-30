# Performance and portability: measurements, results, and plan

Everything here is measured on this tree (RMCE01, macOS/arm64, M5) with the harness in
`runlogs/` described at the end. Numbers are from a **real Grand Prix race** (player-controlled
Mario, 12 karts, full HUD), not the attract loop, unless stated.

## 1. The target question, answered with arithmetic

The stated goal was "run on a crappy Windows 95 machine". That is not reachable, and the reason
is the game, not the port:

| Resource | Mario Kart Wii needs | Typical Win95 box (P100-200) |
|---|---|---|
| Guest RAM | 24 MiB MEM1 + 64 MiB MEM2 = ~88 MiB, before host overhead | 16-32 MB total |
| CPU | 729 MHz PowerPC 750CL with paired-singles SIMD | ~100-200 MHz x86, no SIMD |
| GPU | Hollywood/GX, hardware T&L, textured 3D at 480p | none, or S3 Virge class |

A static recompilation **preserves the original workload** - it re-expresses the same computation
for a new CPU. It cannot make the game need less than the console needed. This is the key
difference from `isle-portable`: Lego Island *targeted* a P133 in 1997, so weak hardware was
always in scope for it.

What is genuinely reachable, in order of difficulty:

* **Tier A (works today):** 64-bit desktop with Metal/Vulkan/D3D12. 60 fps, large headroom.
* **Tier B (very achievable):** GL 3.3 / GLES 3.0 devices - Raspberry Pi 4/5, Intel Macs, Steam
  Deck, phones, older gaming PCs. Blocked only by the Dawn dependency (see §5).
* **Tier C (stretch):** 32-bit hosts. Blocked by the flat address model (see §5).
* **Out of reach:** Win95-era hardware, for the reasons above.

## 2. Baseline

| Metric | Value |
|---|---|
| Binary | 75,975,800 bytes (76.0 MB) |
| `__text` | 51.5 MB, 73,043 symbols |
| Generated C++ | 475 MB across 204 shards |
| Frame rate | locked 60 fps (VI-paced) |
| **CPU in a race** | **0.360 cores (36.0% of one core) -> ~2.8x headroom** |
| CPU in attract demo | 0.298 cores (29.8%) |

The game is **pacing-bound, not CPU-bound**, on this machine. Optimizing for an M5 is pointless;
every win here is a win for weak hardware, and that is the only reason to pursue it.

### Where the binary actually is

| Component | Size | Share |
|---|---:|---:|
| Translated guest code (`func_*`, 30,474 symbols) | 40.2 MB | 68.1% |
| Runtime + HLE | 11.6 MB | 19.6% |
| Tint (WGSL compiler, inside Dawn) | 2.4 MB | 4.0% |
| SDL3 | 1.4 MB | 2.3% |
| Dawn (WebGPU) | 1.2 MB | 2.0% |
| SQLite, png/freetype/zstd, Crypto++, ImGui, abseil, aurora | 2.5 MB | 4.0% |

**68% of the binary is the game itself.** The entire graphics stack is ~4 MB, so "trim the
dependencies" is not where size lives - the translated code is.

## 3. Where the CPU goes in a race

Self-time from `sample`, blocking excluded, 5,383 working samples:

| Cost | Share |
|---|---:|
| **Indirect-call dispatch** (`TryDispatchRawCpuTarget`, `InvokeIndirectCpu`, `InvokeIndirectJump`, registry misses) | **35.4%** |
| Translated guest code (`func_*`) - the actual game | 20.0% |
| aurora / GX command submission | 8.1% |
| `_tlv_get_addr` (thread-local access thunk) | 1.3% |
| everything else | rest |

**A third of CPU time is recompiler dispatch overhead - more than the game's own logic.** This is
the single largest lever, and it is exactly the cost that matters on a weak CPU.

Per indirect call the bridge was paying: a TLS read for the CpuContext, two more TLS accesses for
a diagnostic execution-address scope, and construction/destruction of a GPR guard (18 words) and
an FPR guard (up to 18 doubles).

## 4. What was tried

### Landed: `initial-exec` TLS model (`runtime/include/tls_model.h`)

The hot thread-locals (`s_cpuContext`, `g_currentTranslatedExecutionAddress`, the two dispatch
memos) defaulted to the general-dynamic TLS model, which on Mach-O resolves through a real
function call to `_tlv_get_addr` on every access. All of them live in the main executable, which
is loaded before any thread exists, so initial-exec is valid and compiles to a direct
thread-pointer offset load.

| | dispatch | guest code | aurora |
|---|---:|---:|---:|
| baseline | 35.4% | 20.0% | 8.1% |
| + initial-exec TLS | **31.2%** | 21.3% | 10.7% |

Dispatch share fell 4.2 points (~12% relative) and the share spent in real game code rose.

### Rejected: "proven-empty guard" fast path

**94.3% of dispatch records (27,235 of 28,869) already carry no register-preservation work**
(`fprMask=0`, `preserveGprs=false`) - the translator's effect analysis is good, so both guards
are already inert for almost every call. A fast path that branched around them measured 34.5%
dispatch (vs 31.2%), i.e. no improvement once the extra loads and branches are paid for. Reverted
rather than keep unproven complexity.

### Measured knob, not enabled: `MKW_TRANSLATED_OPT_LEVEL`

The 28k translated functions are ~51 MB of `__text`, so their optimization level is the dominant
size knob (`runtime/cmake/PublicProducts.cmake`, default `-O2`).

| Level | Binary | vs baseline | CPU in race |
|---|---:|---:|---:|
| `-O2` (default) | 76.0 MB | - | 0.360 cores |
| `-Oz` | **64.8 MB** | **-14.7%** | **0.438 cores (+22%)** |

Size-first costs real CPU. On weak hardware CPU is the binding constraint, so `-Oz` is the wrong
default; it is there for storage-constrained targets. `-Os` is untested and likely a better
middle point.

## 5. The two hard portability blockers

Both blockers below are now **addressed in the build/source**; neither has been *run* on a target
that exercises it, which is the honest status.

1. **Dawn/WebGPU only targeting Metal, Vulkan and D3D12** - fixed as a configuration problem, not
   a missing renderer: see §5f. Needs a from-source Dawn on a GLES-capable target to validate.
2. **The 4 GiB fixed-base guest reservation excluding 32-bit hosts** - implemented as a masked
   512 MiB window: see §5g. Needs a 32-bit toolchain to validate.

## 5b. Correction: the earlier A/B numbers were contaminated

While chasing benchmark variance I found **five leaked `WiiCompiled` processes running at once**
(45%, 31%, 11%, 3%, 3% of a core - about 93% of a core of stolen CPU). The benchmark scripts were
killing the process they launched but leaking instances across runs, and the contamination grew
over time. **Every CPU-time A/B in section 4 was measured against that moving background and must
be re-verified.** The harness now kills by exact process name (`pkill -9 -x`) before and after
every run. Binary sizes are unaffected - those are exact.

This is also why measured "spread" wandered between 2.4% and 57% across otherwise identical runs.

## 5c. The right instrument: uncapped mode (`[video] frame_limit = false`)

CPU-percentage-while-paced was the wrong metric: at a locked 60 fps the process sleeps most of
every frame, so the signal is swamped by idle. `frame_limit = false` removes the cap and the
runtime reports guest throughput directly.

Removing the pace sleep alone was not enough - the guest self-paces on `VIWaitForRetrace` and the
retrace timeline still advanced on the wall clock, so it busy-waited on the same 60 Hz grid
(measured: 60.6 fps "unpaced"). The VI period itself has to collapse; benchmark mode shortens it
to 1 ms, and the guest then runs as fast as the host allows.

Sampling on a wall-clock timer was still wrong: unpaced, each build reaches a different point in
the attract loop after N seconds, and the **same binary measured 87 and 120 fps** on two runs.
The fix is to index by frame number - the attract loop is a deterministic replay, so frame N is
always the same guest content. The runtime now reports "seconds to reach frame N".

**Measured, time to frame 8000, alternating runs:**

| Build | seconds | fps | binary |
|---|---:|---:|---:|
| baseline | 90.41 / 91.21 | 88.48 / 87.71 | 76.0 MB |
| ThinLTO | 90.56 / 90.38 | 88.34 / 88.51 | 78.1 MB |

Run-to-run precision is **~0.9%**, enough to resolve a 2% effect. **Steady state is ~88 fps =
1.47x realtime on an M5 performance core.** ThinLTO is neutral on speed and costs 2.1 MB, so it
stays off (`MKW_LTO=off` by default); note the *contaminated* earlier measurement had suggested
it was 5.5% faster, which is a good illustration of why §5b matters.

That single number is the budget for every portability question: the recompilation currently
needs a CPU no more than about **2x slower than an M5 performance core** to hold 60 fps.

## 5d. What that means for old hardware

| Host | Single-thread vs an M5 P-core (approx) | Expected |
|---|---|---|
| M5 | 1x | 88 fps (1.47x realtime) |
| Modern midrange x86 | ~1.5x slower | ~60 fps, marginal |
| Core 2 Duo (2006, best XP era) | ~5-8x slower | ~11-18 fps |
| Pentium 4 / Athlon XP (2003) | ~10-15x slower | under 9 fps |

Reaching 60 fps on the *best* XP-era CPU needs roughly a **4-5x** speedup. Indirect dispatch is
~34% of CPU, so removing it **entirely** yields about 1.5x. 3-4x is therefore not reachable by
optimizing the current design; it would need a fundamentally cheaper execution strategy, and the
GPU side (Wii GX/TEV on a DX9-class part) is a second, independent problem. A late-XP machine at
20-30 fps is a credible target; "native speed on any computer from that era" is not.

## 5e. The benchmark that finally worked, and PGO

Three workloads were tried before one held still:

| Workload | Problem | Spread |
|---|---|---|
| Attract loop, paced, CPU% | dominated by idle (sleeps most of each frame) | 2-57% |
| Attract loop, unpaced, frame-indexed | content mix diverges: state transitions are wall-clock driven, so a faster build reaches different scenes at the same frame number - the same binary measured 87 and 340 fps | up to 50% |
| **Scripted race, paced, CPU%** | **homogeneous work (12 karts driving) for the whole window** | **1-4%** |

The attract loop is only deterministic while it is *paced*; removing the cap makes the frame rate
feed back into the content mix. A scripted race is the right benchmark - `[input] pad_script`
drives the menus and the measurement window sits entirely inside the race.

### PGO: the first verified win

| Build | cores (median) | runs | spread | binary |
|---|---:|---|---:|---:|
| baseline | 0.4562 | 0.454, 0.455, 0.458, 0.474 | 4.3% | 75,977,576 |
| **PGO** | **0.4292** | 0.429, 0.429, 0.434 | **1.0%** | **74,871,256** |

The two distributions do not overlap. **PGO costs ~6% less CPU and produces a 1.5% smaller
binary** - `MKW_PGO=generate` / `MKW_PGO=use` in `runtime/cmake/PublicProducts.cmake`. This is
consistent with the earlier finding that the dispatch cost is branch behaviour and layout rather
than the C++ bookkeeping around it: the two source-level attempts at that bookkeeping measured
nothing, and the profile-driven one measured 6%.

Collecting the profile needs continuous mode (`LLVM_PROFILE_FILE=...%c.profraw`) because the
benchmark kills the process, and a plain instrumented run flushes nothing on SIGKILL.

### Status of the other knobs

| Change | Verdict |
|---|---|
| PGO | **enable** - 6% CPU, 1.5% smaller, verified |
| ThinLTO | off - neutral speed, +2.1 MB |
| `-Oz` translated code | knob only - 14.7% smaller; CPU cost needs re-measuring on the race benchmark (the +22% figure was taken during the contaminated period, §5b) |
| initial-exec TLS | kept - sound in principle, magnitude never verified on a trustworthy instrument |
| empty-guard dispatch fast path | rejected - 94.3% of records are already inert |

## 5f. Portability: the renderer blocker is now unlocked

The Dawn dependency was the thing excluding every GL-era device, and the cause turned out to be
build configuration rather than missing code. Two separate places in `aurora-main/cmake/
AuroraDawnProvider.cmake` wrote the backend choice as `set(... CACHE INTERNAL ...)`, which in
CMake **always overwrites** - so a consumer could never select its own backends, no matter what
it set first:

* `_aurora_dawn_set_platform_backends()` stamped all seven backend variables per platform;
* the vendor (from-source) branch separately forced both GL backends off.

Both now fall back to their existing defaults only when the parent has not already chosen
(`_aurora_dawn_default_backend`), so behaviour is unchanged unless something asks otherwise.
`runtime/CMakeLists.txt` exposes the choice:

| Option | Effect |
|---|---|
| `MKW_GPU_GLES` | Dawn's OpenGL ES backend (GLES 3.1) + Tint's GLSL writer |
| `MKW_GPU_DESKTOP_GL` | Dawn's desktop OpenGL backend (GL 4.4) |
| `MKW_GPU_VULKAN_ON_APPLE` | Vulkan on Apple via MoltenVK - how the non-Metal path gets exercised on a Mac |

Verified: with no options set the Dawn cache is byte-for-byte what it was (Metal ON, everything
else OFF) and the product builds unchanged; with `-DMKW_GPU_GLES=ON` the setting reaches
`DAWN_ENABLE_OPENGLES=ON` and `TINT_BUILD_GLSL_WRITER=ON`.

aurora already carries `BACKEND_OPENGL` / `BACKEND_OPENGLES` in its enum and compiles its GL
surface glue from these same variables (`aurora_core.cmake`), so no new renderer is needed -
Dawn implements WebGPU over GLES 3.1 already.

**Not yet done:** an actual GLES build. It needs `AURORA_DAWN_PROVIDER=vendor` (the prebuilt Dawn
package ships Metal/D3D12/Null only - confirmed by `nm` on `libwebgpu_dawn.a`), and macOS has no
native GLES, so the build and the run both belong on a Linux/Pi target. This change is what makes
that attempt possible; it is not itself a working GLES build.

## 5g. Portability: the 32-bit blocker is now implemented

The other hard blocker was the address-space model: the guest range was a 4 GiB reservation at a
fixed 16 TiB base, which no 32-bit process can satisfy. That is now a compile-time policy in
`guest_flat_memory.h`:

| Host | Reservation | Guest -> host | Base |
|---|---|---|---|
| 64-bit | 4 GiB at a fixed base | `base + addr` | compile-time constant |
| **32-bit** | **512 MiB window** | **`base + (addr & 0x1FFFFFFF)`** | **chosen by the OS, read from a global** |

512 MiB with a `0x1FFFFFFF` mask is the smallest window that keeps every range the game uses
distinct, verified against a live run: MEM1 (`0x80000000`) at offset 0 for 26 MiB, MMIO
(`0xCC000000`) at 192 MiB, MEM2 (`0x90000000`) at 256 MiB for its 128 MiB. The physical mirror at
`0x00000000` aliases MEM1, which is what the hardware does. Only touched pages commit, so
resident memory still tracks the ~88 MiB actually used rather than the reservation.

Every guest access funnels through five sites (`memory_access.h`, `ppc_isa_quantized.h`), all now
routed via `MKW_GUEST_OFFSET`. **The 64-bit binary is byte-for-byte identical afterwards** - the
mask is `0xFFFFFFFF` against a `uint32_t` operand, so it folds away completely - and the game
boots and runs unchanged. The 32-bit path costs one global load per access, which is inherent to
not having a fixed base.

**Not yet done:** an actual 32-bit build. That needs a 32-bit toolchain and target; what exists
here is the model, the reservation path and the proof that enabling it costs 64-bit nothing.

## 6. Prioritized plan

1. **Re-verify section 4 with the uncapped instrument** (built, §5c) and clean process hygiene
   (§5b). Uncapped fps is CPU-bound and directly meaningful, unlike CPU-% under pacing. Every
   optimization claim above needs re-measuring against it before being trusted.
2. **Attack indirect dispatch (31%).** With a trustworthy benchmark: per-call-site inline caching
   (guest address -> entry) to skip the registry walk, and making the diagnostic
   execution-address scope compile-time optional (it costs 2-3 TLS accesses on every dispatch for
   a crash-reporting feature).
3. **GL/GLES backend** - unlocks Tier B, the whole "runs on anything" story.
4. **Size:** ThinLTO (never enabled; helps size and speed together), then hot/cold splitting
   driven by profile data - `-Oz` for cold functions, `-O2/-O3` for hot ones, which should beat
   both uniform settings.
5. **32-bit addressing mode** - Tier C.
6. **PGO** - never enabled; for a 28k-function recompilation with heavy indirect dispatch this is
   typically a large win in both layout and branch prediction.

## 7. Harness (in `runlogs/`)

* `drive.sh <exe> <secs> <tag> [interval]` - run a product, capture its window periodically.
* `cpubench.sh <exe> <warmup> <window> <tag>` - steady-state CPU seconds per wall second.
* `prof.sh <exe> <warmup> <secs> <tag>` - `sample` the process; parse self-time per symbol.
* `pad-race.txt` + `[input] pad_script` - scripted controller timeline that boots to a real race.

Getting into a race unattended needed two runtime fixes, both landed:
`[input] keyboard_port` (drives a GameCube port from the keyboard through aurora's own binding
API) and `[input] pad_script`. The controller must be installed **before the first `PADRead`** -
the game latches which ports exist from that first read, and installing it lazily left port 1 as
`PAD_ERR_NO_CONTROLLER` forever, which is why menu input appeared to do nothing.
