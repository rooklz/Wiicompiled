# WiiCompiled on macOS (Apple silicon)

A native arm64 build of the static recompilation: no emulator, no interpreter, no JIT. The
translated game runs as ordinary machine code; the Wii's hardware surface is served by the
runtime's HLE, and the GX command stream reaches the GPU through aurora's WebGPU layer on Dawn's
**Metal** backend - the one GPU API the platform exposes natively, with no Vulkan/MoltenVK layer in
between.

## Why this shape (and not something lower)

* CPU: the game is already native code after translation. What remains between the game and
  the silicon is the runtime (HLE + scheduler + memory). Below that there is nothing to go lower to.
* GPU: Metal is the lowest supported layer on macOS. aurora already generates its pipelines for
  WebGPU and Dawn maps them 1:1 onto Metal objects; Dawn runs with validation and robustness
  checks disabled (aurora's device toggles), so per-call overhead is a thin translation of
  descriptors, not a second graphics API. Driving the AGX GPU below Metal (private command
  buffers) is a research exercise with no correctness guarantees and no benefit for a 2008 workload;
  it was deliberately not pursued.
* Everything in the executable is statically linked except the OS frameworks and the SDK's own
  zlib/bzip2: SDL3, Dawn, libpng, FreeType, abseil, fmt, xxHash, zstd, SQLite, Dear ImGui.

## What was ported

| Area | Windows original | macOS |
| --- | --- | --- |
| Guest threads | Win32 fibers | libco coroutines (from the Linux port) on 1 MiB private mappings with a guard page each |
| Guest RAM | placeholder + `MapViewOfFile3` dual views | Mach named memory entry mapped twice (`mach_make_memory_entry_64` + `mach_vm_map` with `VM_FLAGS_OVERWRITE`) |
| Fault interception | vectored exception handler | `SIGSEGV`/`SIGBUS` on an alternate stack; write/read direction from `ESR_EL1` (`__es.__esr`) |
| Page granularity | 4 KiB assumed | read from the host (16 KiB on Apple silicon) for every `mprotect` range and page table |
| FP environment | MXCSR FTZ/DAZ | FPCR FZ (from the aarch64 port) |
| Paired singles | SSE | NEON `float32x2_t` (from the aarch64 port) |
| Two-value state-free results | `ext_vector_type(2)` (xmm0) | 16-byte aggregate returned in `x0`/`x1` (no NEON round-trip per call) |
| AX/DSP mix kernels | AVX2 | NEON, bit-exact against the scalar references (differential test) |
| VI frame pacing | high-resolution waitable timer + spin | `mach_wait_until` to an absolute deadline + 80 µs residual spin |
| Executable/data paths | `%LOCALAPPDATA%` | `~/Library/Application Support/WiiCompiled`, `_NSGetExecutablePath` |
| Media ducking | WinRT session monitor | disabled (no public equivalent) |
| WUP-028 adapter | WinUSB driver | SDL controller path (libusb loaded at run time when present) |
| Generated data blobs | COFF `.rdata` | Mach-O `__TEXT,__const`, underscore-prefixed symbols |
| Crash reports | `dbghelp` stack walk | `backtrace_symbols_fd` |

The main guest thread runs at `QOS_CLASS_USER_INTERACTIVE`.

## The NTSC-U question

Upstream only accepts the PAL executable because every native override and every SDK global
the HLE touches is spelled as a PAL address. This port keeps those PAL spellings as **identities**
and resolves them through a per-region table (`runtime/include/region/guest_region.h`,
`rmcp01.h`, `rmce01.h`), which the translator reads too. The RMCE01 table was built from
evidence, not from a delta:

1. `tools/region/port_map.py` ports the 29,792-entry PAL function map through the mkw-sp
   project's PAL->NTSC-U chunk table and validates every entry against the NTSC-U binaries
   themselves: `bl` targets and REL relocation targets (proven), vtable/function-pointer
   references, or instruction-shape plausibility. Entries the binaries do not support are not
   emitted. Report: `projects/mkwii-ntsc/MAP_REPORT.md`.
2. Native-override addresses that are not function entries (jump-table labels, mid-function
   hooks) are accepted only as a fixed offset inside a validated containing function.
3. Data globals are resolved by disassembling the NTSC-U SDK function that references each one
   (`tools/region/disasm.py`, llvm-mc based); the evidence per entry is kept in
   `projects/mkwii-ntsc/data_addresses.txt`.
4. Region facts (game code `RMCE`, `VI_NTSC`, SC area/game `USA`/`US`, product code `LU`)
   replace the PAL constants the runtime used to seed low memory and the SC/NAND/ES HLE.
5. The MEM1 arena-lo word the runtime seeds is the executable's initial stack top
   (`__init_registers`: PAL `0x80399180`, NTSC-U `0x80394E00`). The game's boot heap grows from
   it, so it decides where the game's own REL loader places `StaticR.rel` (`0x805102E0` on PAL,
   `0x8050BF60` on NTSC-U); a PAL value there shifted the whole REL by `0x4380`.
6. The map also seeds every function entry the NTSC-U binaries prove on their own (`bl`
   targets, relocation call targets, `.ctors`/`.dtors` entries, vtable pointers) so functions
   that live in the regions the community table cannot map - the UI-control code was reordered
   between regions - are still translated; the REL prolog calls 192 static constructors and each
   must be a translated entry.

Things that turned out to be region-independent and were verified rather than assumed: the
r13/r2-relative offsets of nearly every SDK global (`.sbss` is byte-identical below the
`EGG::Screen` statics and 8 bytes shorter above), the `.sdata2` arrays, and the Retro-WFC
bootstrap hook (`DWCi_Auth_SendRequest`, a fixed offset inside its function in both regions).

`tools/region/gen_region_headers.py` regenerates both headers and refuses to write an incomplete
NTSC-U table.

## Building

Prerequisites: Xcode command line tools, `brew install cmake ninja`, the .NET 8 SDK
(`dotnet-install.sh --channel 8.0`), `cargo install nodtool` (disc extraction), and your own
clean disc image (RVZ/ISO/WBFS accepted).

```
./build-macos.sh --disc "/path/to/Mario Kart Wii.rvz"                        # base game
./build-macos.sh --disc "/path/to/Mario Kart Wii.rvz" --profile wiimmfi      # Wiimmfi
./build-macos.sh --disc "/path/to/Mario Kart Wii.rvz" --profile retro-rewind # Retro Rewind
```

Products land in `dist/<region>[-wiimmfi|-retro-rewind]/`. The first run creates
`~/Library/Application Support/WiiCompiled/Config.toml`; set `[paths] dvd_root` to the extracted
DATA directory the build printed. `tools/macos/mkwii-launch.sh base|wiimmfi|retro-rewind` starts
a product with the matching overlay configuration.

## Online play

* **Wiimmfi**: not runnable by a static recompilation, and the reason is structural, not a
  missing feature. Wiimm's patch (`wstrt patch --wiimmfi`) adds one 0x468-byte section at
  `0x802C0000` and makes it the entry point. Disassembled (`tools/region/disasm.py` on the patched
  DOL), that stub (a) fingerprints the loader environment, (b) carves a block from the top of the
  MEM1 arena, (c) XOR-decodes an obfuscated code blob (`0x802C0360..0x802C0460`, rotating key)
  into it, (d) walks an embedded patch list that writes `b` branches into the game's code and
  32-bit values into memory, (e) zeroes itself, and (f) jumps to `__start`. The translator only
  ever sees two functions (`func_802C0000`, `func_802C003C`); the code that actually talks to the
  server exists solely at run time in memory the runtime's executable-write guard protects, so a
  faithful port would have to reverse-engineer and statically re-apply a deliberately protected
  anti-tamper mechanism - which is also what Wiimmfi's own rules forbid. Upstream WiiCompiled
  does not support Wiimmfi for the same reason; it supports Retro-WFC, whose payload is an open,
  statically translatable format. The `wiimmfi` profile, the patched inputs and the UI text
  overlay are kept in the tree as the record of that analysis (`projects/mkwii-ntsc-wiimmfi`,
  `overlays/wiimmfi`), not as a supported product. Wiimmfi play stays a Dolphin/console matter
  unless Wiimm publishes a recompilation-compatible patch.
* **Retro Rewind / Retro-WFC**: upstream's static profile (Kamek/Pulsar code translated together
  with the game), instantiated for NTSC-U with the pack's own `E` chunk, the RMCE Retro-WFC
  payload and the ported bootstrap hook. See the status section.
* **CTGP-R**: the current distribution (v1.03) is a closed, encrypted blob loaded by an online
  channel with its own anti-tamper; there is nothing to translate and it is not portable this way.
  The 2011-2014 open-source CT-CODE engine could be built as a profile like Retro Rewind.

## Status

_(filled in at the end of the port session; see the commit log for the latest state)_
