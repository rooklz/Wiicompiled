#!/usr/bin/env python3
"""wc_find_targets.py -- find indirect branch targets the call graph misses.

Recursive discovery follows direct calls and the symbol map. Two kinds of code
are invisible to both: a function only ever reached through a pointer (a vtable
slot, a callback registered at run time) and a jump-table label inside another
function. C cannot jump into the middle of a function, so both have to become
entry points the translator emits separately.

They are found the same way: every 4-byte-aligned word in the image that points
at aligned, plausible code inside the text range is a candidate. That over-
collects -- an integer can look like an address -- so each candidate is
verified by decoding at it: the word must be a legal instruction, and it must
not be in the middle of an obvious literal pool. A false positive costs one
translated function that nothing calls; a false negative costs a crash at run
time, so the bias is deliberate.

    wc_find_targets.py <main.dol> <existing_map> <out_map>
"""
import struct, sys, os

dol, existing, out = sys.argv[1], sys.argv[2], sys.argv[3]
d = open(dol, "rb").read()
toff = struct.unpack(">7I", d[0:28]);  doff = struct.unpack(">11I", d[28:72])
tadr = struct.unpack(">7I", d[72:100]); dadr = struct.unpack(">11I", d[100:144])
tsz  = struct.unpack(">7I", d[144:172]); dsz = struct.unpack(">11I", d[172:216])
text = [(tadr[i], tsz[i], toff[i]) for i in range(7) if tsz[i]]
data = [(dadr[i], dsz[i], doff[i]) for i in range(11) if dsz[i]]
tlo = min(a for a, _, _ in text)
thi = max(a + s for a, s, _ in text)

def rd32(addr):
    for b, s, o in text + data:
        if b <= addr < b + s:
            return struct.unpack(">I", d[o + addr - b:o + addr - b + 4])[0]
    return None

def in_text(addr):
    """Inside an actual text section -- NOT merely between the lowest and
    highest, which spans the data section sitting in the gap between text0 and
    text1 and let data be decoded as code."""
    for b, s, _ in text:
        if b <= addr < b + s:
            return True
    return False

def is_entry(addr):
    """A plausible BRANCH TARGET, not merely a plausible instruction.

    A word in a data section that happens to decode as an instruction proves
    nothing -- in 2.7 MB of data, thousands do. A real function pointer points
    at something the code can actually be entered at: a known function start,
    or a boundary where the previous word ends a function (blr/b/bctr) or is
    padding. Requiring that removes most of the coincidences while keeping
    every genuine vtable slot, callback and jump-table label."""
    if addr in have:
        return True                      # a named function start
    prev = rd32(addr - 4)
    if prev is None:
        return False
    if prev in (0, 0x60000000):          # padding or nop before the entry
        return True
    op = (prev >> 26) & 0x3F
    if op == 18:                         # b / bl
        return True
    if op == 19:                         # blr / bctr
        xo = (prev >> 1) & 0x3FF
        return xo in (16, 528)
    return False

def is_code(addr):
    """A plausible instruction at addr, and not obviously data."""
    if addr % 4 or not in_text(addr):
        return False
    w = rd32(addr)
    if w is None or w == 0 or w == 0xFFFFFFFF:
        return False
    op = (w >> 26) & 0x3F
    # Primary opcodes Broadway actually issues. 1,2,5,6 and 30 are not among
    # them, and a word decoding to one of those is data misread as code.
    return op in (3,4,7,8,10,11,12,13,14,15,16,17,18,19,20,21,23,24,25,26,27,28,
                  29,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,
                  50,51,52,53,54,55,56,57,58,59,60,61,62,63)

have = set()
names = {}
for ln in open(existing):
    p = ln.split()
    if len(p) >= 2:
        have.add(int(p[0], 16)); names[int(p[0], 16)] = p[1]

found = set()
# 1. Addresses stored as data: vtable slots, jump tables, callback tables.
for b, s, o in text + data:            # jump tables live in text too
    for off in range(0, s - 3, 4):
        w = struct.unpack(">I", d[o + off:o + off + 4])[0]
        if w in found:
            continue
        if is_code(w) and is_entry(w):
            found.add(w)

# 2. Addresses BUILT IN CODE by an lis/addi or lis/ori pair.
#
# This is how PowerPC materialises a pointer, and it is invisible to a scan of
# stored words: `OSSetSwitchThreadCallback` takes the address of
# `DefaultSwitchThreadCallback` with lis+addi and the value never appears in
# data at all. Missing it cost a boot -- the game called through the callback
# and the port had no entry for it.
#
# lis rD,hi  is addis rD,0,hi (primary 15, rA=0); the low half follows in the
# same register, either addi (primary 14, sign-extended) or ori (primary 24).
# The pair need not be adjacent, so a small window is scanned and the register
# tracked.
for b, s, o in text:
    hi = {}                                    # rD -> high half seen
    for off in range(0, s - 3, 4):
        w = struct.unpack(">I", d[o + off:o + off + 4])[0]
        op = (w >> 26) & 0x3F
        rd = (w >> 21) & 0x1F
        ra = (w >> 16) & 0x1F
        if op == 15 and ra == 0:                       # lis rD, imm
            hi[rd] = (w & 0xFFFF) << 16
            continue
        if rd in hi and ra == rd:
            if op == 14:                               # addi rD,rD,simm
                lo = w & 0xFFFF
                addr = (hi[rd] + (lo - 0x10000 if lo >= 0x8000 else lo)) & 0xFFFFFFFF
            elif op == 24:                             # ori rD,rD,uimm
                addr = hi[rd] | (w & 0xFFFF)
            else:
                hi.pop(rd, None)
                continue
            hi.pop(rd, None)
            if addr not in found and is_code(addr) and is_entry(addr):
                found.add(addr)
            continue
        if rd in hi and op in (14, 15, 24, 25):
            hi.pop(rd, None)                           # register reused

# Persistent extra targets: addresses the code scanner cannot see -- function
# pointers materialized from DATA tables (the BT stack's bta_* callback
# tables were the first proven case: bta_dm_compress_cback reached
# InvokeIndirectCpu with no entry, the call was swallowed, and WPAD sync
# idled forever). Kept in a checked-in file so a rescan cannot lose them.
extra_path = os.path.join(os.path.dirname(out), "targets_extra.txt")
if os.path.exists(extra_path):
    n_extra = 0
    for ln in open(extra_path):
        ln = ln.strip()
        if not ln or ln.startswith("#"):
            continue
        a = int(ln.split()[0], 16)
        if a not in found:
            found.add(a)
            n_extra += 1
    print("  targets_extra: +%d" % n_extra)

merged = sorted(have | found)
with open(out, "w") as fh:
    for a in merged:
        fh.write("%08x %s\n" % (a, names.get(a, "ptr_%08x" % a)))

# The indirect targets, on their own. Keeping them in a separate file rather
# than inferring them from a `ptr_` prefix is the point: a target that also has
# a name -- DefaultSwitchThreadCallback, every virtual method -- is still a
# target, and inferring from the name left it out of the dispatch table. The
# game then called through a pointer the port had no entry for.
tgt = out.replace(".txt", "_targets.txt")
with open(tgt, "w") as fh:
    for a in sorted(found):
        fh.write("%08x\n" % a)
print("map entries %d, indirect targets %d (%d of them named), total %d"
      % (len(have), len(found), len(found & have), len(merged)))
print("  %s\n  %s" % (out, tgt))
