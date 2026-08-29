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

Honestly: it boots. On real hardware it gets through OS init, disc init and the
Bluetooth pairing chain, reads the boot archive off the disc byte-perfect, and
starts drawing. Not playable yet — the strap-to-menu handoff is the current
frontier.

- fiber scheduler for the game's threads, running the game's own SelectThread
  logic rather than emulating a scheduler
- IOS reimplemented host-side: `/dev/di`, `/dev/fs`, ES, Bluetooth
- RSX framebuffer output driven by the game's own XFB flip chain
- disc reads served from a WBFS-backed image over the DVD interface the game
  expects
- runs on stock CFW through webMAN — no devkit

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
