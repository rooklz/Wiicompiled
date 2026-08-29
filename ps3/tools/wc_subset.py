#!/usr/bin/env python3
"""Static direct-call closure over the translated corpus.

The generated call layer defines InvokeDirectCpu<T> for every table address in
one object, so a partial link must generate for exactly the linked subset.
This walker computes that subset: seeds (all DOL functions + the REL addresses
the game has been observed to need) closed over InvokeDirectCpu<0x...> edges
in the generated sources. Indirect calls resolve through the runtime table and
miss into WcUnresolvedCall, whose log lines are next iteration's seeds.

  wc_subset.py <functions_dir> <seeds.txt> <out_subset.txt> [--dol-all]
"""
import os, re, sys

fdir, seedf, outf = sys.argv[1], sys.argv[2], sys.argv[3]
dol_all = "--dol-all" in sys.argv

have = {}
for f in os.listdir(fdir):
    m = re.fullmatch(r'func_([0-9A-Fa-f]{8})\.cpp', f)
    if m: have[int(m.group(1), 16)] = os.path.join(fdir, f)

seeds = set()
for ln in open(seedf):
    ln = ln.split('#')[0].strip().split()
    if ln:
        a = int(ln[0], 16)
        if a in have: seeds.add(a)
if dol_all:
    seeds |= {a for a in have if a < 0x80500000}

edge_re = re.compile(r'InvokeDirectCpu<0x([0-9A-Fa-f]{8})u>')
sub, work = set(), sorted(seeds)
while work:
    a = work.pop()
    if a in sub: continue
    sub.add(a)
    try: src = open(have[a]).read()
    except KeyError: continue
    for m in edge_re.finditer(src):
        t = int(m.group(1), 16)
        if t in have and t not in sub: work.append(t)

with open(outf, 'w') as f:
    for a in sorted(sub): f.write(f"{a:08x}\n")
rel = sum(1 for a in sub if a >= 0x80500000)
print(f"subset: {len(sub)} functions ({rel} REL) from {len(seeds)} seeds -> {outf}")
