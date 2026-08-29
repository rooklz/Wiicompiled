#!/usr/bin/env python3
"""Disassemble a range of a DOL (or REL text) at a guest address using llvm-mc (PowerPC).
Usage: disasm.py <main.dol> <hexaddr> [words] [--rel StaticR.rel --rel-base 0x8050BF60]
Prints one instruction per line with its guest address; r13/r2-relative and lis/addi(ori) global
references are annotated so data symbols can be located in another region's binary."""
import struct, subprocess, sys, shutil, os
def parse_dol(path):
    d=open(path,'rb').read()
    offs=struct.unpack('>18I',d[0:72]); addrs=struct.unpack('>18I',d[72:144]); sizes=struct.unpack('>18I',d[144:216])
    secs=[(addrs[i],offs[i],sizes[i]) for i in range(18) if sizes[i]]
    return d,secs
def parse_rel(path, base):
    d=open(path,'rb').read()
    num=struct.unpack('>I',d[0x0C:0x10])[0]; stab=struct.unpack('>I',d[0x10:0x14])[0]
    secs=[]
    for i in range(num):
        off,size=struct.unpack('>II',d[stab+8*i:stab+8*i+8])
        exec_=off&1; off&=~1
        if off and size: secs.append((base+off,off,size))
    return d,secs
def read_words(img,secs,addr,n):
    for va,fo,sz in secs:
        if va<=addr<va+sz:
            avail=min(n*4, va+sz-addr)
            return img[fo+(addr-va):fo+(addr-va)+avail]
    raise SystemExit(f"address {addr:#x} not in any section")
def main():
    a=sys.argv[1:]
    dol=a[0]; addr=int(a[1],16); n=int(a[2]) if len(a)>2 and not a[2].startswith('--') else 48
    rel=None; relbase=0x8050BF60
    if '--rel' in a: rel=a[a.index('--rel')+1]
    if '--rel-base' in a: relbase=int(a[a.index('--rel-base')+1],16)
    img,secs=parse_dol(dol)
    if addr>=0x80500000 and rel:
        img,secs=parse_rel(rel,relbase)
    b=read_words(img,secs,addr,n)
    mc=shutil.which('llvm-mc') or '/opt/homebrew/opt/llvm/bin/llvm-mc'
    hexs=' '.join(f'0x{x:02x}' for x in b)
    out=subprocess.run([mc,'--disassemble','-triple=powerpc-unknown-linux-gnu','-mcpu=750'],input=hexs,capture_output=True,text=True)
    lines=[l.strip() for l in out.stdout.splitlines() if l.strip() and not l.strip().startswith('.')]
    pc=addr; hi={}
    for i in range(0,len(b),4):
        w=struct.unpack('>I',b[i:i+4])[0]; ins=lines[i//4] if i//4<len(lines) else '??'
        note=''
        op=w>>26; rt=(w>>21)&31; ra=(w>>16)&31; simm=w&0xFFFF; simm=simm-0x10000 if simm&0x8000 else simm
        if op in (32,33,34,35,36,37,38,39,40,41,44,45,48,49,50,51,52,53,54,55,56,57,60,61) and ra in (2,13):
            base=0x80388880 if ra==13 else 0x8038AC20
            note=f'   ; r{ra}{simm:+#x} -> NTSC {base+simm:#x}'
        if op==15 and ra==0: hi[rt]=(w&0xFFFF)<<16; note=f'   ; lis r{rt}'
        elif op==14 and ra in hi: note=f'   ; addi -> {hi[ra]+simm:#x}'
        elif op==24 and rt in hi: note=f'   ; ori -> {hi[rt]|(w&0xFFFF):#x}'
        elif op in (32,33,34,35,36,37,38,39,40,41,44,45,48,49,50,51,52,53,54,55,56,57,60,61) and ra in hi:
            note=f'   ; [lis-based] -> {hi[ra]+simm:#x}'
        print(f'{pc:08x}  {w:08x}  {ins}{note}')
        pc+=4
main()
