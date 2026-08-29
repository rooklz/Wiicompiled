# ps3.mk — the console build. Runs *inside* the ps3dev container
# (`make ps3` from the top-level Makefile), where the cross-toolchain lives.

PS3DEV   ?= /usr/local/ps3dev
PPU_BIN  := $(PS3DEV)/ppu/bin
SPU_BIN  := $(PS3DEV)/spu/bin
TOOL_BIN := $(PS3DEV)/bin

PPU_CC   := $(PPU_BIN)/powerpc64-ps3-elf-gcc
PPU_LD   := $(PPU_BIN)/powerpc64-ps3-elf-gcc
PPU_CXX  := $(PPU_BIN)/powerpc64-ps3-elf-g++
PPU_OBJDUMP := $(PPU_BIN)/powerpc64-ps3-elf-objdump
SPU_CC   := $(SPU_BIN)/spu-gcc

BUILD    := build/ppu
SPUBUILD := build/spu

# -mcpu=cell is not cosmetic: it makes GCC schedule for the PPE's *in-order*
# dual-issue pipeline. On an out-of-order host the scheduler's choices barely
# matter; here they are worth real percentage points, because nothing in the
# hardware will rescue a badly ordered dependent pair (docs/HARDWARE.md §1).
PPU_ARCH  := -mcpu=cell -maltivec -mabi=altivec -mhard-float
PPU_WARN  := -Wall -Wextra -Wno-unused-parameter -Wshadow -Wpointer-arith
PPU_OPT   := -O3 -fno-strict-aliasing -fomit-frame-pointer
PPU_DEF   := -D__PS3__ -DNDEBUG -DLOG_LEVEL_COMPILED=3
PPU_INC   := -I/work -I$(PS3DEV)/ppu/include
PPU_CFLAGS := $(PPU_ARCH) $(PPU_OPT) $(PPU_WARN) $(PPU_DEF) $(PPU_INC) -std=gnu11
# Translated guest functions (src/core/ppc/wc): C++17 for gcc 7, no EH/RTTI.
PPU_CXXFLAGS := $(PPU_ARCH) $(PPU_OPT) $(PPU_DEF) $(PPU_INC) -Isrc/core/ppc/wc \
                -std=gnu++17 -fno-exceptions -fno-rtti -Wall -Wno-unused

# -fno-strict-aliasing is deliberate. A recompiler necessarily reinterprets the
# same storage as instructions, integers and floats; strict aliasing would let
# the compiler assume those never overlap, which is exactly wrong here.

SPU_CFLAGS := -O3 -std=gnu11 -Wall -Wextra -I/work -I$(PS3DEV)/spu/include

# -lm last: the interpreter uses sqrt/fma/ldexp/nearbyint for the FP oracle,
# and newlib's libm also supplies internals (z_infinity, numtest) that libc
# references, so it has to resolve after everything else.
LIBS := -L$(PS3DEV)/ppu/lib -lgcm_sys -lrsx -lnet -lnetctl -lsysutil -lio -laudio -lsysmodule -lstdc++ -llv2dbg -llv2 -lm

GUESTBLOB := build/guest/guest_blob.h

# Real 32-bit PowerPC compiled from ordinary C, used as emulator input. Linked
# at a fixed guest address so the harness knows where each function lives.
$(GUESTBLOB): tests/guestcode/guestfns.c tools/mkguestblob.py
	@mkdir -p build/guest
	@printf "  GUEST    %s\n" "$<"
	@$(PPU_CC) -m32 -O2 -ffreestanding -fno-builtin -c $< -o build/guest/guestfns.o
	@$(PPU_BIN)/powerpc64-ps3-elf-ld -m elf32ppc -Ttext=0x80400000 -e 0x80400000 \
	    -o build/guest/guestfns.elf build/guest/guestfns.o
	@$(PPU_BIN)/powerpc64-ps3-elf-objcopy -O binary build/guest/guestfns.elf build/guest/guestfns.bin
	@$(PPU_BIN)/powerpc64-ps3-elf-nm -n build/guest/guestfns.elf > build/guest/guestfns.nm
	@python3 tools/mkguestblob.py build/guest/guestfns.bin build/guest/guestfns.nm \
	    80400000 $@

build/guest/dol_blob.h: tests/guestcode/dolmain.c tools/mkdolblob.py
	@mkdir -p build/guest
	@printf "  DOL      %s\n" "$<"
	@$(PPU_CC) -m32 -O2 -ffreestanding -fno-builtin -nostdlib -c tests/guestcode/dolmain.c -o build/guest/dolmain.o
	@$(PPU_BIN)/powerpc64-ps3-elf-ld -m elf32ppc -Ttext=0x80003100 -e _start -o build/guest/dolmain.elf build/guest/dolmain.o
	@$(PPU_BIN)/powerpc64-ps3-elf-objcopy -O binary build/guest/dolmain.elf build/guest/dolmain.bin
	@python3 tools/mkdolblob.py build/guest/dolmain.elf build/guest/dolmain.bin \
	    0x$$($(PPU_BIN)/powerpc64-ps3-elf-nm build/guest/dolmain.elf | grep " T _start" | cut -d" " -f1) \
	    0x80300000 0xC4BB86B8 $@

# A second guest program, one that drives the GPU. Same shape as the first: real
# 32-bit PowerPC, linked at a fixed address, packed into a DOL. It is what turns
# "each graphics stage works" into "the CPU can actually reach the GPU".
# The animated guest: sixty frames through the EFB-copy frame lifecycle -- the
# skeleton of a game's main loop, and the first frames-per-second measurement.
build/guest/gxanim_blob.h: tests/guestcode/gxanim.c tools/mkdolblob.py
	@mkdir -p build/guest
	@printf "  DOL      %s\n" "$<"
	@$(PPU_CC) -m32 -O2 -ffreestanding -fno-builtin -nostdlib -c tests/guestcode/gxanim.c -o build/guest/gxanim.o
	@$(PPU_BIN)/powerpc64-ps3-elf-ld -m elf32ppc -Ttext=0x80003100 -e _start -o build/guest/gxanim.elf build/guest/gxanim.o
	@$(PPU_BIN)/powerpc64-ps3-elf-objcopy -O binary build/guest/gxanim.elf build/guest/gxanim.bin
	@python3 tools/mkdolblob.py build/guest/gxanim.elf build/guest/gxanim.bin \
	    0x$$($(PPU_BIN)/powerpc64-ps3-elf-nm build/guest/gxanim.elf | grep " T _start" | cut -d" " -f1) \
	    0x80300000 0x00000000 $@ gxanim

# The visible-triangle guest: the smallest complete frame a Wii title could
# draw, run on the console to light the whole pipeline end to end.
build/guest/gxtri_blob.h: tests/guestcode/gxtri.c tools/mkdolblob.py
	@mkdir -p build/guest
	@printf "  DOL      %s\n" "$<"
	@$(PPU_CC) -m32 -O2 -ffreestanding -fno-builtin -nostdlib -c tests/guestcode/gxtri.c -o build/guest/gxtri.o
	@$(PPU_BIN)/powerpc64-ps3-elf-ld -m elf32ppc -Ttext=0x80003100 -e _start -o build/guest/gxtri.elf build/guest/gxtri.o
	@$(PPU_BIN)/powerpc64-ps3-elf-objcopy -O binary build/guest/gxtri.elf build/guest/gxtri.bin
	@python3 tools/mkdolblob.py build/guest/gxtri.elf build/guest/gxtri.bin \
	    0x$$($(PPU_BIN)/powerpc64-ps3-elf-nm build/guest/gxtri.elf | grep " T _start" | cut -d" " -f1) \
	    0x80300000 0x00000000 $@ gxtri

build/guest/gx_blob.h: tests/guestcode/gxmain.c tools/mkdolblob.py
	@mkdir -p build/guest
	@printf "  DOL      %s\n" "$<"
	@$(PPU_CC) -m32 -O2 -ffreestanding -fno-builtin -nostdlib -c tests/guestcode/gxmain.c -o build/guest/gxmain.o
	@$(PPU_BIN)/powerpc64-ps3-elf-ld -m elf32ppc -Ttext=0x80003100 -e _start -o build/guest/gxmain.elf build/guest/gxmain.o
	@$(PPU_BIN)/powerpc64-ps3-elf-objcopy -O binary build/guest/gxmain.elf build/guest/gxmain.bin
	@python3 tools/mkdolblob.py build/guest/gxmain.elf build/guest/gxmain.bin \
	    0x$$($(PPU_BIN)/powerpc64-ps3-elf-nm build/guest/gxmain.elf | grep " T _start" | cut -d" " -f1) \
	    0x80300000 0x00000000 $@ gx

# libc_lock_fix.c must be in the link: it overrides a broken newlib symbol, and
# the override only happens because our objects are scanned before -lc.
PPU_SRCS := \
    src/platform/ps3/libc_lock_fix.c \
    src/common/log.c \
    src/core/mem/memmap.c \
    src/core/mem/mem_platform_ps3.c \
    src/core/core_timing.c \
    src/core/difftrace.c \
    src/core/hw/hardware.c src/core/hw/ipc.c src/core/hw/dsp.c src/core/hw/ai.c src/core/hw/pe.c \
    src/core/hw/dsp_ax.c src/core/hw/audio_out.c src/platform/ps3/audio_ps3.c \
             src/core/ios/ios_hle.c src/core/ios/ios_bt.c src/core/ios/ios_disc_glue.c src/core/disc/disc_image.c \
    src/core/hw/pi.c \
    src/core/hw/dev_lock.c \
    src/core/hw/vi.c \
    src/core/hw/gx_fifo.c \
    src/core/gx/gx_vertex.c \
    src/core/gx/gx_parse.c \
    src/core/gx/bp.c \
    src/core/gx/gx_state.c \
    src/video/rsx/vertex_loader.c \
    src/video/rsx/texture_decode.c \
    src/video/rsx/xfb_present.c \
    src/video/rsx/efb_copy.c \
    src/video/rsx/xf_program.c \
    src/video/rsx/gx_features.c \
    src/video/rsx/rsx_video.c \
    src/video/rsx/rsx_shader.c \
    src/video/rsx/rsx_tritest.c \
    src/video/rsx/gx_render.c \
    src/video/rsx/tev_program.c \
    src/core/disc/dol.c \
    src/core/ppc/interp/interp_core.c \
    src/core/ppc/interp/interp_integer.c \
    src/core/ppc/interp/interp_loadstore.c \
    src/core/ppc/interp/interp_branch.c \
    src/core/ppc/interp/interp_system.c \
    src/core/ppc/interp/interp_float.c \
    src/core/ppc/interp/interp_paired.c \
    src/core/ppc/jit/jit.c \
    src/core/ppc/jit/jit_compile.c \
    src/core/ppc/difftest.c \
    src/core/ppc/realtest.c \
    tests/guestcode/guestfns.c \
    tools/rec/fn_math.c \
    tools/rec/aot_fns.c \
    tools/rec/aot_manifest.c \
    src/platform/ps3/main.c \
    src/platform/ps3/spu_vtx.c

PPU_ASM  := src/core/ppc/jit/jit_entry.S src/platform/ps3/mkwii_blobs.S
PPU_CXXSRCS := src/core/ppc/wc/wc_adapter.cpp \
               src/core/ppc/wc/gen/func_801B5AD4.cpp

# The statically recompiled game.
#
# 11,367 translated guest functions plus the call layer and the embedded DOL
# data sections. Built into an archive rather than listed here so the link only
# pulls what is reachable, and so a rebuild of the emulator does not recompile
# 100 MB of generated C++. tools/wc_build.sh populates build/wcobj.
#
# -Os is not usable: gcc 7.2 ICEs in rs6000_savres_routine_name on functions
# with large register save sets. -O2 costs about 2.6 KiB of .text per function,
# which is ~29 MB for the whole game -- affordable on a 256 MB console.
WC_LIB     := build/libwcgame.a
WC_EXTRA   := src/core/ppc/wc/wc_boot.cpp \
              src/core/ppc/wc/wc_helpers.cpp \
              src/core/ppc/wc/wc_os.cpp \
            src/core/ppc/wc/wc_fiber.cpp src/core/ppc/wc/wc_sched.cpp \
              src/core/ppc/wc/wc_watchdog.cpp \
              src/core/ppc/wc/wc_hle_os.cpp \
              src/core/ppc/wc/gen/wc_calls.cpp \
              src/core/ppc/wc/gen/wc_data_init.cpp
WC_ASM     := src/core/ppc/wc/gen/wc_data_blobs.S src/core/ppc/wc/fiber_ps3.S

ifneq ($(FIBER),)
PPU_CFLAGS   += -DWC_FIBER_SCHED
PPU_CXXFLAGS += -DWC_FIBER_SCHED
endif

PPU_OBJS := $(patsubst %.c,$(BUILD)/%.o,$(PPU_SRCS)) \
            $(patsubst %.S,$(BUILD)/%.o,$(PPU_ASM)) \
            $(patsubst %.cpp,$(BUILD)/%.o,$(PPU_CXXSRCS))

# Present only once the game has been translated and compiled; the emulator
# links and runs without it.
WC_OBJS := $(patsubst %.cpp,$(BUILD)/%.o,$(WC_EXTRA)) \
           $(patsubst %.S,$(BUILD)/%.o,$(WC_ASM))
WC_PRESENT := $(wildcard $(WC_LIB))
ifneq ($(WC_PRESENT),)
PPU_OBJS += $(WC_OBJS)
PPU_LIBS_EXTRA := $(WC_LIB)
# PPU_CFLAGS/PPU_CXXFLAGS are immediate (:=) and were expanded above, so adding
# to PPU_DEF here would never reach the compiler -- the define has to go onto
# the flag variables themselves. That mistake linked an EBOOT with the port
# compiled out and no error anywhere.
PPU_CFLAGS   += -DWC_GAME_LINKED
PPU_CXXFLAGS += -DWC_GAME_LINKED
endif

.PHONY: all objs clean disasm self pkg

all: self

TARGET  := wiicompiled-ps3
ELF     := build/$(TARGET).elf
SELF    := build/$(TARGET).self

# PSL1GHT link: sprxlinker fixes up the PRX import stubs the lv2 libraries use,
# and fself produces an unsigned SELF, which is what custom firmware runs.
$(ELF): $(PPU_OBJS)
	@printf "  LD(ppu)  %s\n" "$@"
	@$(PPU_LD) $(PPU_ARCH) -Wl,--gc-sections -o $@ $(PPU_OBJS) $(PPU_LIBS_EXTRA) $(LIBS)
	@$(TOOL_BIN)/sprxlinker $@

$(SELF): $(ELF)
	@printf "  SELF     %s\n" "$@"
	@$(TOOL_BIN)/fself $(ELF) $(SELF)
	@ls -l $(SELF)

self: $(SELF)

# ---------------------------------------------------------------- package
#
# An installable package, which is how custom firmware gets homebrew onto the
# XMB.
#
# The EBOOT is produced with `fself -n` (fake NPDRM), *not* make_self_npdrm.
# make_self_npdrm produces a genuinely signed SELF, which requires Sony's keys;
# without them the console rejects the executable at launch and drops straight
# back to the dashboard with no diagnostic whatsoever. `fself -n` is the
# fake-signed equivalent that custom firmware accepts, and is what homebrew
# actually ships. package_finalize then adjusts the package for CFW install.
APPID     := WCPS3001
TITLE     := WiiCompiled PS3
CONTENTID := UP0001-$(APPID)_00-0000000000000000
PKGDIR    := build/pkg
PKGFILE   := build/$(TARGET).pkg

# SPU programs need the lv2 SPU-thread CRT (-lsputhread) and the SPU MACHDEP
# flags from PSL1GHT's spu_rules. Without libsputhread the ELF entry point is
# newlib's _start, which expects an environment lv2 does not provide: the
# thread is created and "started" successfully and then never reaches main().
# That cost a console lockup to find, so the flags are spelled out here.
SPU_CC     := /usr/local/ps3dev/spu/bin/spu-gcc
SPU_MACHDEP:= -mdual-nops -fmodulo-sched -ffunction-sections -fdata-sections
SPU_CFLAGS := -O2 -Wall $(SPU_MACHDEP) -I. -I/usr/local/ps3dev/spu/include
SPU_LDdFLAGS := -Wl,--gc-sections -L/usr/local/ps3dev/spu/lib
SPU_LIBS   := -lsputhread

$(BUILD)/vtx_spu.elf: spu/vtx_spu.c src/video/rsx/spu_vtx_shared.h
	$(SPU_CC) $(SPU_CFLAGS) -o $@ spu/vtx_spu.c $(SPU_LDdFLAGS) $(SPU_LIBS)

pkg: $(BUILD)/vtx_spu.elf $(PKGFILE)

$(PKGFILE): $(ELF)
	@printf "  PKG      %s\n" "$@"
	@mkdir -p $(PKGDIR)/USRDIR
	@cp $(PS3DEV)/bin/ICON0.PNG $(PKGDIR)/ICON0.PNG
	@$(TOOL_BIN)/fself -n $(ELF) $(PKGDIR)/USRDIR/EBOOT.BIN > /dev/null
# A plain (non-NPDRM) SELF of the same build, as an alternative spawn target.
# Measured: sysProcessExitSpawn2 to the NPDRM EBOOT is accepted by the syscall
# but the loader never runs a single instruction of the new image -- the boot
# breadcrumb stays at one line. Non-NPDRM SELFs go through different loader
# validation, so the relaunch tries this first and falls back to the EBOOT.
	@$(TOOL_BIN)/fself $(ELF) $(PKGDIR)/USRDIR/RELOAD.SELF > /dev/null
# The SPU image is loaded from USRDIR at run time, so it has to be staged with
# the rest of the package. It was previously left out, which meant an edited
# SPU program compiled cleanly and then never ran -- the console kept loading
# whatever vtx_spu.elf happened to be there. A stale SPU image does not fail
# loudly; it looks exactly like a hung SPU.
	@cp $(BUILD)/vtx_spu.elf $(PKGDIR)/USRDIR/vtx_spu.elf
	@$(TOOL_BIN)/sfo --title "$(TITLE)" --appid "$(APPID)" -f $(PS3DEV)/bin/sfo.xml $(PKGDIR)/PARAM.SFO
	@$(TOOL_BIN)/pkg --contentid $(CONTENTID) $(PKGDIR)/ $@ > /dev/null
	@$(TOOL_BIN)/package_finalize $@ > /dev/null 2>&1 || true
	@ls -l $@

objs: $(PPU_OBJS)
	@echo "PPU objects built: $(words $(PPU_OBJS))"

$(BUILD)/src/core/ppc/realtest.o: $(GUESTBLOB)

$(BUILD)/src/platform/ps3/mkwii_blobs.o: assets/mkwii/mkwii_main.dol \
    assets/mkwii/mkwii_fst.bin assets/mkwii/sysconf.bin assets/mkwii/setting.bin \
    assets/mkwii/mkwii_disc.slice assets/wii/shared2/menu/FaceLib/RFL_DB.dat

$(BUILD)/src/platform/ps3/main.o: build/guest/gxtri_blob.h build/guest/gxanim_blob.h

# -MMD -MP emits a .d file listing every header the object depends on, and the
# -include below feeds them back to make.
#
# Without this, editing a header rebuilds nothing. That is not a tidiness issue:
# a fix made in vp_emitter.h was deployed to the console three times without
# ever being compiled in, and the identical wrong output each time was read as
# "the fix didn't work" rather than "the fix wasn't in the binary". Stale-object
# builds cost more debugging time here than any actual bug.
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@printf "  CC(ppu)  %s\n" "$<"
	@$(PPU_CC) $(PPU_CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@printf "  CXX(ppu) %s\n" "$<"
	@$(PPU_CXX) $(PPU_CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.S
	@mkdir -p $(dir $@)
	@printf "  AS(ppu)  %s\n" "$<"
	@$(PPU_CC) $(PPU_CFLAGS) -c $< -o $@

$(SPUBUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@printf "  CC(spu)  %s\n" "$<"
	@$(SPU_CC) $(SPU_CFLAGS) -c $< -o $@

# Inspecting real codegen is part of the workflow, not a debugging afterthought:
# the JIT's cost model is only as good as our knowledge of what the compiler
# actually emits for the interpreter and helper paths.
disasm: objs
	@for o in $(PPU_OBJS); do echo "=== $$o ==="; $(PPU_OBJDUMP) -d "$$o" | head -60; done

clean:
	rm -rf $(BUILD) $(SPUBUILD)

-include $(PPU_OBJS:.o=.d)
