# WiiCompiled on PS3

The translated game running natively on a real PS3. Same idea as the PC port,
different target: the translator emits 32-bit big-endian PowerPC, the Cell PPU
is 64-bit big-endian PowerPC, so the code comes across almost 1:1 and there is
no byte swapping anywhere at runtime.

No emulator in the loop here either. The game's threads run as fibers on one
PPU hardware thread, the Wii OS layer (threads, interrupts, IPC) is
reimplemented host-side, video goes out through RSX, and the SPUs pick up
offload work like the vertex loader.

> [!IMPORTANT]
> Same rules as the main project. No Nintendo code, no assets and no game data
> live in this folder or its history. You supply your own legally dumped copy
> (this target is NTSC-U for now), and the translation runs on your machine.

## Where it's at

It reaches the menus now. On real hardware it boots through OS init, disc init
and the Bluetooth handshake, past the strap screen, and into the game's own
front end — license creation, character select — drawing at roughly 30–60 fps.
Getting there meant chasing down a run of boot-time hangs; the ones that mattered:

- interrupt delivery now restores the full register set on the way out, not
  just the caller-saved half. A handler that parked mid-flight was leaving one
  garbage pointer behind, and the strap loader was reading its disc destination
  through it — 290 KB of the boot archive landing on top of the globals.
- DVD read completions go through the interrupt path instead of running their
  callback inline. Inline, the game's file reader (which chains one read from
  the previous one's callback) recursed until its four-slot context pool ran
  dry and the game killed itself on purpose.
- interrupt delivery holds off across the two-instruction window where the OS
  swaps its current-thread pointers, which real hardware runs with interrupts
  off anyway.

What's under it:

- fiber scheduler for the game's threads, running the game's own SelectThread
  logic rather than emulating a scheduler
- IOS reimplemented host-side: `/dev/di`, `/dev/fs`, ES, Bluetooth
- RSX framebuffer output driven by the game's own XFB flip chain
- disc reads served from a WBFS-backed image over the DVD interface the game
  expects
- runs on stock CFW through webMAN — no devkit

## The whole game, not just the boot code

The DOL (system and boot code) has been native from day one. The rest of the
game lives in `StaticR.rel`, and that's now translated too — about 47,000
functions, with the module's own relocation tables scanned to find every entry
point (`tools/wc_find_rel_targets.py`, checked against the addresses the game's
loader prints on the console).

The catch: translated whole and linked native, the image is ~154 MB, and the
PS3 only hands a game about 85 MB of code space. So it can't all be native. The
plan is a hybrid — DOL native, `StaticR.rel` run under the recompiler's JIT from
the copy the game itself loads into RAM, with a bridge so REL code still reaches
the host-side OS/IOS layer (`src/core/ppc/wc/wc_bridge.cpp`). Linked that way it
fits at ~70 MB. It doesn't boot cleanly yet — it stalls in the C runtime setup
before the game's own code runs — so a working whole-game boot is the current
frontier. `tools/rel_build.sh` builds the full corpus; `tools/wc_subset.py`
picks the slice that gets linked native.

## Building

The console build runs inside the ps3dev toolchain container:

    docker build -f Dockerfile.toolchain -t ps3dev .
    make -f ps3.mk self

Anything derived from your dump — the translated functions under
`src/core/ppc/wc/gen/`, the recompiled helpers under `tools/rec/`, and the boot
data in `src/platform/ps3/mkwii_blobs.S` — is generated locally the first time
you build and is gitignored, exactly like the main project's `generated/`. A
fresh clone will not build until you point the translator at your own disc.

`tools/cycle.sh` chains build, FTP deploy, relaunch and log fetch. The scripts
in `tools/` take the console IP at the top (placeholder `192.168.1.123`).

## Layout

    src/         host runtime — core (OS/IPC/IOS/GX), video/rsx, common, platform/ps3
    spu/         SPU offload programs
    tools/       build, deploy and codegen scripts
    external/mkwii-ntsc/   translator project config for the NTSC-U build
