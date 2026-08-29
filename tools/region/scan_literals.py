#!/usr/bin/env python3
"""Inventory every 0x8xxxxxxx guest-address literal in the runtime sources and classify it by
the PAL (RMCP01) section it falls in. Output is a TSV consumed by the region refactor."""
import os, re, sys, collections
ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "runtime")
PAL_DOL = [("dol.init",0x80004000,0x80006460),("dol.extab",0x80006460,0x80006A20),("dol.extabindex",0x80006A20,0x800072C0),
    ("dol.text",0x800072C0,0x80244DE0),("dol.ctors",0x80244DE0,0x80244EA0),("dol.dtors",0x80244EA0,0x80244EC0),
    ("dol.rodata",0x80244EC0,0x80258580),("dol.data",0x80258580,0x802A4040),("dol.bss",0x802A4080,0x80384C00),
    ("dol.sdata",0x80384C00,0x80385FC0),("dol.sbss",0x80385FC0,0x80386FA0),("dol.sdata2",0x80386FA0,0x80389140),("dol.sbss2",0x80389140,0x8038917C)]
PAL_REL = [("rel.header",0x805102E0,0x805103B4),("rel.text",0x805103B4,0x8088F400),("rel.ctors",0x8088F400,0x8088F704),("rel.dtors",0x8088F704,0x8088F710),
    ("rel.rodata",0x8088F720,0x808B2BD0),("rel.data",0x808B2BD0,0x808DD3D4),("rel.bss",0x809BD6E0,0x809C4F90)]
def classify(v):
    if v < 0x80004000: return "lowmem"
    for n,a,b in PAL_DOL+PAL_REL:
        if a <= v < b: return n
    if 0x80389180 <= v < 0x805102E0: return "arena-mem1"   # heap/arena between DOL and REL
    if 0x809C4F90 <= v < 0x81800000: return "mem1-high"
    if 0x81800000 <= v < 0x82000000: return "mem1-tail"
    if 0x90000000 <= v < 0x94000000: return "mem2"
    if 0xC0000000 <= v < 0xD0000000: return "cached-mirror"
    if 0xCC000000 <= v < 0xCE000000: return "mmio"
    return "other"
pat = re.compile(r"0x(8[0-9A-Fa-f]{7})\b")
rows=[]
for dp,_,fns in os.walk(ROOT):
    if "third_party" in dp: continue
    for fn in fns:
        if not fn.endswith((".cpp",".h",".hpp",".inc")): continue
        p=os.path.join(dp,fn); rel=os.path.relpath(p, os.path.join(ROOT,".."))
        for i,line in enumerate(open(p,encoding="utf-8",errors="replace"),1):
            for m in pat.finditer(line):
                v=int(m.group(1),16)
                rows.append((rel,i,f"0x{v:08X}",classify(v),line.strip()[:140]))
hist=collections.Counter(r[3] for r in rows)
out=sys.argv[1] if len(sys.argv)>1 else "/dev/stdout"
with open(out,"w") as f:
    for r in rows: f.write("\t".join(map(str,r))+"\n")
print("total literals:",len(rows)); print(sorted(hist.items(), key=lambda x:-x[1]))
print("files:",len(set(r[0] for r in rows)))
