#!/bin/sh
# Regenerate the whole generated call layer for the static recompilation.
#
# Run this after changing the target scanner, the HLE bindings, or the
# translated function set. It is NOT part of ps3.mk on purpose -- it rewrites
# checked-in generated sources -- but that means it is easy to run wrong, so
# the arguments live here rather than in anyone's shell history.
#
# The override file is hle_active.txt: the ADDRESSES ACTUALLY BOUND. Do not
# pass hle_overrides.txt -- that is the full 272-row catalogue of candidates
# with every row commented out, and passing it silently produces a call layer
# with zero native overrides that still builds and still boots and then runs
# the guest's own hardware-touching code.
set -e
cd "$(dirname "$0")/.."
python3 tools/wc_find_targets.py assets/mkwii/mkwii_main.dol \
    external/mkwii-ntsc/MAP_ntsc_dol.txt external/mkwii-ntsc/MAP_ntsc_full.txt
python3 tools/wc_gen_calls.py external/mkwii-ntsc/full/functions \
    src/core/ppc/wc/gen external/mkwii-ntsc/hle_active.txt
grep -c 'wc_hle_' src/core/ppc/wc/gen/wc_calls.cpp | sed 's/^/hle references: /'
