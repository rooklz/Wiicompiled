#!/usr/bin/env python3
"""wc_postprocess.py -- give a WiiCompiled-emitted function a guest instruction count.

The PS3 dispatcher charges every AOT function the number of guest instructions
it retired (the scheduler's clock), which the emitted C++ does not track. Each
basic block is labelled `loc_<guest address>`, and basic blocks partition the
function's instruction range, so a block's length is the distance to the next
label in address order -- the last block runs to the function end. A counter
local is bumped per block and folded into ctx->insn_count at every return.
tools/rec/aot_difftest.c compares that count against the interpreter's step
count on every trial, so a wrong length here cannot go unnoticed.

    wc_postprocess.py <in.cpp> <out.cpp> <function_end_hex>
"""
import re, sys

src, dst = sys.argv[1], sys.argv[2]
end_arg = int(sys.argv[3], 16) if len(sys.argv) > 3 else None
lines = open(src).read().split("\n")

# Function starts from the NTSC map: the end of the function owning a label
# is the next start after it. Inlined callee blocks carry the callee's own
# addresses, so each label is bounded by its own function, never a neighbour.
import bisect, os
starts = []
mp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "external", "mkwii-ntsc", "MAP_ntsc_dol.txt")
if os.path.exists(mp):
    starts = sorted(int(l.split()[0], 16) for l in open(mp) if l.strip())
def owner_end(a):
    i = bisect.bisect_right(starts, a)
    return starts[i] if i < len(starts) else (end_arg or a + 4)

labels = sorted({int(m.group(1), 16) for ln in lines
                 for m in [re.match(r'^loc_([0-9A-Fa-f]{8}):$', ln)] if m})
length = {}
for i, a in enumerate(labels):
    nxt = labels[i + 1] if i + 1 < len(labels) else 0xFFFFFFFF
    bound = end_arg if (end_arg and i + 1 == len(labels)) else owner_end(a)
    length[a] = (min(nxt, bound) - a) // 4
if any(v <= 0 for v in length.values()):
    sys.exit("non-positive block length: check function_end")

out, n_blocks, n_returns, declared = [], 0, 0, False
i = 0
while i < len(lines):
    ln = lines[i]
    out.append(ln)
    if not declared and re.match(r'^extern "C" void func_[0-9A-F]+\(CpuContext\*', ln):
        # opening brace is the next line
        out.append(lines[i + 1]); i += 1
        out.append("    uint32_t insn = 0;   /* wc_postprocess: guest instructions retired */")
        declared = True
    m = re.match(r'^loc_([0-9A-Fa-f]{8}):$', ln)
    if m and i + 1 < len(lines) and lines[i + 1].strip() == "{":
        out.append(lines[i + 1]); i += 1
        out.append("    insn += %du;" % length[int(m.group(1), 16)])
        n_blocks += 1
    if re.match(r'^\s*return;$', ln):
        out.pop()
        out.append(re.sub(r'return;', 'ctx->insn_count += insn; return;', ln))
        n_returns += 1
    i += 1
open(dst, "w").write("\n".join(out))
print("%s: %d blocks counted (%d guest instructions spanned), %d returns" %
      (dst, n_blocks, sum(length.values()), n_returns))
