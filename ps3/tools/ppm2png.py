#!/usr/bin/env python3
"""ppm2png.py -- convert a binary PPM (P6) to PNG with only the standard library,
so console screenshots (devlink `shot` -> /dev_hdd0/tmp/wiicompiled-shot.ppm) can be
viewed without an image library.   ppm2png.py in.ppm out.png [scale_div]"""
import struct, sys, zlib

def read_ppm(path):
    data = open(path, "rb").read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos+1].isspace(): pos += 1
        if data[pos:pos+1] == b"#":
            while data[pos:pos+1] not in (b"\n", b""): pos += 1
            continue
        start = pos
        while not data[pos:pos+1].isspace(): pos += 1
        tokens.append(data[start:pos])
    pos += 1
    assert tokens[0] == b"P6", tokens[0]
    w, h, mx = int(tokens[1]), int(tokens[2]), int(tokens[3])
    return w, h, data[pos:pos + w * h * 3]

def write_png(path, w, h, rgb, div=1):
    if div > 1:
        ow, oh = w // div, h // div
        rows = bytearray()
        for y in range(oh):
            rows.append(0)
            src = rgb[(y * div) * w * 3:(y * div) * w * 3 + w * 3]
            rows.extend(src[x * div * 3:x * div * 3 + 3] for x in range(ow)) if False else None
            row = bytearray()
            for x in range(ow):
                i = x * div * 3
                row += src[i:i + 3]
            rows += row
        w, h = ow, oh
    else:
        rows = bytearray()
        for y in range(h):
            rows.append(0); rows += rgb[y * w * 3:(y + 1) * w * 3]
    def chunk(t, b): return struct.pack(">I", len(b)) + t + b + struct.pack(">I", zlib.crc32(t + b) & 0xffffffff)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(bytes(rows), 6)) + chunk(b"IEND", b"")
    open(path, "wb").write(png)

w, h, rgb = read_ppm(sys.argv[1])
write_png(sys.argv[2], w, h, rgb, int(sys.argv[3]) if len(sys.argv) > 3 else 1)
print("wrote", sys.argv[2], w, h)
