#!/bin/sh
# rel_build.sh -- post-translation pipeline: compile the (now DOL+REL) function
# corpus, regenerate the call layer, and link the FIBER self.
# Run after translate-recursive completes. Steps are incremental where safe.
set -e
cd "$(dirname "$0")/.."

echo "== drop objects with no source (pruned functions) =="
python3 - <<'PY'
import os, glob, re
have = {re.sub(r'\.o$', '', os.path.basename(f)) for f in glob.glob('build/wcobj/*.o')}
want = {re.sub(r'\.cpp$', '', os.path.basename(f))
        for f in glob.glob('external/mkwii-ntsc/full/functions/*.cpp')}
for s in have - want:
    os.remove('build/wcobj/%s.o' % s)
print("  stale removed: %d, to compile: %d" % (len(have - want), len(want - have)))
PY

echo "== compile (container-local: the bind mount costs 3.5 s/file, local ~0.3 s) =="
# One bulk copy in, compile at -P8 against container-local disk, one tar back.
docker run --rm -m 8g -v "$(pwd)":/work -w /work ps3dev:latest sh -c '
  set -e
  mkdir -p /tmp/fn /tmp/obj
  echo "  staging sources+headers into the container..."
  (cd /work/external/mkwii-ntsc/full && tar cf - functions) | (cd /tmp/fn && tar xf -)
  # Full src tree, same shape as the mount: headers climb out of ppc/
  # (gekko.h -> ../../common/types.h), so a subtree copy cannot compile.
  mkdir -p /tmp/w && (cd /work && tar cf - src) | (cd /tmp/w && tar xf -)
  # Seed already-compiled objects (checkpoint or prior run): restarts resume.
  if [ -f /work/build/wcobj_ckpt.tar ]; then (cd /tmp/obj && tar xf /work/build/wcobj_ckpt.tar) || true; fi
  if [ -d /work/build/wcobj ]; then (cd /work/build/wcobj && tar cf - .) | (cd /tmp/obj && tar xf -) || true; fi
  echo "  seeded $(ls /tmp/obj | wc -l) existing objects"
  G=/usr/local/ps3dev/ppu/bin/powerpc64-ps3-elf-g++
  # Precompiled header: the header stack costs seconds of frontend per TU;
  # the .gch cuts a 3.2 s median compile to ~0.3 s (measured). Flags must
  # match the per-file set exactly, -include picks it up via -I.
  printf "#include <cstdint>\n#include \"ppc_runtime.h\"\n#include \"abi_bridge.h\"\n#include \"memory.h\"\n#include \"recomp_mod_loader.h\"\n" > /tmp/w/src/core/ppc/wc/wc_pch.h
  echo "  building precompiled header..."
  $G -std=gnu++17 -fno-exceptions -fno-rtti -O2 -mcpu=cell -ffunction-sections -fdata-sections -fno-strict-aliasing -DWC_PIN_ARENA -ffixed-r14 -I/tmp/w -I/tmp/w/src/core/ppc/wc -x c++-header /tmp/w/src/core/ppc/wc/wc_pch.h -o /tmp/w/src/core/ppc/wc/wc_pch.h.gch
  N=$(ls /tmp/fn/functions | wc -l)
  echo "  compiling $N functions at -P8 (container-local)..."
  # Largest first: 44 files exceed 200 KB (max 722 KB) and cost minutes each;
  # queued last they pin a single worker while seven idle.
  find /tmp/fn/functions -name "*.cpp" | xargs -r stat -c "%s %n" | sort -rn | cut -d" " -f2- > /tmp/queue.txt
  tr "\n" "\0" < /tmp/queue.txt | xargs -0 -P 10 -n1 sh -c '"'"'
    f="$1"; o=/tmp/obj/$(basename "$f" .cpp).o
    [ -f "$o" ] && exit 0
    /usr/local/ps3dev/ppu/bin/powerpc64-ps3-elf-g++ -std=gnu++17 -fno-exceptions -fno-rtti       -O2 -mcpu=cell -ffunction-sections -fdata-sections -fno-strict-aliasing       -DWC_PIN_ARENA -ffixed-r14 -I/tmp/w -I/tmp/w/src/core/ppc/wc -include wc_pch.h -Winvalid-pch -c "$f" -o "$o" || echo "FAILED: $f"
  '"'"' sh | grep FAILED | head -20 || true
  echo "  objects built: $(ls /tmp/obj | wc -l)"
  NOBJ=$(ls /tmp/obj | wc -l)
  if [ "$NOBJ" -lt "$N" ]; then
    echo "  container-side incomplete ($NOBJ of $N) -- leaving mount objects untouched"
  else
    echo "  streaming objects back to the mount..."
    rm -rf /work/build/wcobj && mkdir -p /work/build/wcobj
    (cd /tmp/obj && tar cf - .) | (cd /work/build/wcobj && tar xf -)
  fi
'
NCPP=$(find external/mkwii-ntsc/full/functions -name '*.cpp' | wc -l | tr -d ' ')
NOBJ=$(find build/wcobj -name '*.o' | wc -l | tr -d ' ')
echo "  objects: $NOBJ / $NCPP"
if [ "$NOBJ" != "$NCPP" ]; then
    echo "COMPILE INCOMPLETE ($NOBJ of $NCPP) -- aborting before archive/link"
    exit 1
fi

echo "== regenerate call layer (hle_active.txt -- NOT the overrides catalogue) =="
python3 tools/wc_gen_calls.py external/mkwii-ntsc/full/functions src/core/ppc/wc/gen \
    external/mkwii-ntsc/hle_active.txt
grep -c 'wc_hle_' src/core/ppc/wc/gen/wc_calls.cpp | sed 's/^/  hle references: /'

echo "== archive =="
# Archive (ARG_MAX-safe: batched ar appends from a list). WC_SUBSET optional:
# unset links the whole corpus (gc-sections drops what nothing references);
# set it to a wc_subset.py output to link a partial set with matching gen.
if [ -n "${WC_SUBSET:-}" ]; then
    python3 - "$WC_SUBSET" > build/link_objs.txt <<'PYL'
import sys
for ln in open(sys.argv[1]):
    a = ln.strip()
    if a:
        print("build/wcobj/func_%s.o" % a.upper())
PYL
else
    find build/wcobj -name '*.o' | sort > build/link_objs.txt
fi
MISSING=$(while read o; do [ -f "$o" ] || echo "$o"; done < build/link_objs.txt | head -5)
[ -n "$MISSING" ] && { echo "OBJECTS MISSING (compile still running?):"; echo "$MISSING"; exit 1; }
docker run --rm -m 8g -v "$(pwd)":/work -w /work ps3dev:latest sh -c \
  'cd /work && rm -f build/libwcgame.a && tr "\n" "\0" < build/link_objs.txt | xargs -0 -n 500 /usr/local/ps3dev/ppu/bin/powerpc64-ps3-elf-ar r build/libwcgame.a && /usr/local/ps3dev/ppu/bin/powerpc64-ps3-elf-ranlib build/libwcgame.a'
echo "  archived: $(wc -l < build/link_objs.txt | tr -d ' ') objects"
ls -la build/libwcgame.a | awk '{print "  libwcgame.a:",$5,"bytes"}'

echo "== clean runtime rebuild + link (gen headers changed) =="
rm -rf build/ppu
docker run --rm -m 3g -v "$(pwd)":/work -w /work ps3dev:latest \
  bash -c "make -f ps3.mk FIBER=1 self" > /tmp/rel_link.log 2>&1 || { echo "LINK FAILED:"; grep -iE 'error|undefined' /tmp/rel_link.log | head -12; exit 1; }
ls -la build/dolphin-ps3.self | awk '{print "  SELF:",$5,"bytes"}'
docker run --rm -v "$(pwd)":/work -w /work ps3dev:latest \
  /usr/local/ps3dev/ppu/bin/powerpc64-ps3-elf-size -A build/dolphin-ps3.elf | grep -E '^\.text|Total'
echo "== done: deploy with the usual flag+relaunch lane =="
