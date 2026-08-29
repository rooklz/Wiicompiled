#!/usr/bin/env python3
"""Function starts for StaticR.rel (NTSC-U), from the artifact itself.

No PAL porting: the REL's own relocation tables name every data-held code
pointer (vtables, callback tables), branch scanning names every direct
target, and the header names prolog/epilog/unresolved. Runtime addresses
use the observed console link base (module header at BASE; each section at
BASE-relative file offset, exec flag in offset bit0 -- validated live against
the loader's printed section spans)."""
import struct, sys

REL   = sys.argv[1] if len(sys.argv) > 1 else "assets/mkwii/mkwii_StaticR.rel"
BASE  = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x8050bf60
d     = open(REL, "rb").read()
u32   = lambda o: struct.unpack_from(">I", d, o)[0]
u16   = lambda o: struct.unpack_from(">H", d, o)[0]
u8    = lambda o: d[o]

num_sections     = u32(0x0C)
section_info_off = u32(0x10)
prolog_sec, epilog_sec, unresolved_sec = u8(0x20), u8(0x21), u8(0x22)
prolog_off, epilog_off, unresolved_off = u32(0x24), u32(0x28), u32(0x2C)
imp_off, imp_size = u32(0x28+0x0C+0x04-0x04), 0  # placeholder, fixed below
# v3 header: 0x00 id, 04 next, 08 prev, 0C numSections, 10 sectionInfoOffset,
# 14 nameOffset, 18 nameSize, 1C version, 20 bssSize, 24 relOffset, 28 impOffset,
# 2C impSize, 30 prologSection, 31 epilogSection, 32 unresolvedSection, 33 pad,
# 34 prolog, 38 epilog, 3C unresolved, 40 align, 44 bssAlign, 48 fixSize
version    = u32(0x1C)
bss_size   = u32(0x20)
rel_offset = u32(0x24)
imp_offset = u32(0x28)
imp_size   = u32(0x2C)
prolog_sec, epilog_sec, unresolved_sec = u8(0x30), u8(0x31), u8(0x32)
prolog_off, epilog_off, unresolved_off = u32(0x34), u32(0x38), u32(0x3C)

secs = []           # (runtime_addr, size, exec, file_off)
for i in range(num_sections):
    off  = u32(section_info_off + i*8)
    size = u32(section_info_off + i*8 + 4)
    ex   = off & 1
    fo   = off & ~3
    addr = BASE + fo if fo else 0
    secs.append((addr, size, ex, fo))

def sec_addr(si, off):
    a = secs[si][0]
    return (a + off) if a else 0

starts = set()
for nm, si, off in (("prolog",prolog_sec,prolog_off),("epilog",epilog_sec,epilog_off),
                    ("unresolved",unresolved_sec,unresolved_off)):
    a = sec_addr(si, off)
    if a: starts.add(a)

execs = [(a, a+s) for (a,s,ex,fo) in secs if ex and a]
def in_exec(a):
    return any(lo <= a < hi for lo,hi in execs)

# 1) direct branch targets within exec sections
for (a0, sz, ex, fo) in secs:
    if not ex or not a0: continue
    for o in range(0, sz & ~3, 4):
        w = u32(fo + o)
        op = w >> 26
        if op == 18:                     # b/bl
            imm = w & 0x03FFFFFC
            if imm & 0x02000000: imm -= 0x04000000
            if not (w & 2):              # AA=0
                t = a0 + o + imm
                if in_exec(t): starts.add(t)

# 2) relocation-table code pointers (vtables/callbacks): walk imp table
n_addr32 = 0
for ie in range(0, imp_size, 8):
    mod_id  = u32(imp_offset + ie)
    tab_off = u32(imp_offset + ie + 4)
    o = tab_off
    cur = 0
    csec = 0
    while True:
        skip = u16(o); rtype = u8(o+2); rsec = u8(o+3); radd = u32(o+4)
        o += 8
        if rtype == 203: break            # R_RVL_STOP
        if rtype == 202:                  # R_RVL_SECT: switch section
            csec = rsec; cur = 0; continue
        cur += skip
        if rtype in (1,):                 # R_PPC_ADDR32
            # value written = address of (rsec, radd) in THIS module (id match)
            t = sec_addr(rsec, radd)
            if t and in_exec(t) and (t & 3) == 0:
                starts.add(t); n_addr32 += 1

starts = sorted(a for a in starts if in_exec(a))
out = sys.argv[3] if len(sys.argv) > 3 else "external/mkwii-ntsc/MAP_ntsc_rel_scan.txt"
with open(out, "w") as f:
    for a in starts: f.write(f"{a:08x} rel_{a:08x}\n")

print(f"sections={num_sections} version={version} bss={bss_size}")
for i,(a,s,ex,fo) in enumerate(secs):
    if a: print(f"  sec{i}: {a:08x}-{a+s:08x} exec={ex} (file+{fo:x})")
print(f"prolog={sec_addr(prolog_sec,prolog_off):08x} epilog={sec_addr(epilog_sec,epilog_off):08x}")
print(f"function starts: {len(starts)} (addr32-derived {n_addr32}) -> {out}")
