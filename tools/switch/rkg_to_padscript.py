#!/usr/bin/env python3
"""Convert a Mario Kart Wii ghost (RKG) into a pad_script timeline.

Staff-ghost RKGs on the disc (files/Race/TimeAttack/ghost1/*.rkg) record exact per-frame
inputs: three RLE streams (face buttons, stick, tricks) behind an optional Yaz1 layer. This
emits one "F<frame> ..." pad_script step per input change, so the runtime replays the ghost's
lap deterministically (pad_script is frame-keyed).

RKG layout facts follow the community documentation; the decode approach matches
mkw-roblox's tools/rkg.py. EXPERIMENTAL until validated on hardware: the button mapping
below (A accel, B brake/drift, L item -> GC A/B+R/L) reproduces a lap only if it matches
what the game latched when the ghost was recorded - verify by replaying a staff ghost and
comparing the finish time to the ghost's own header time.

The ghost's input stream begins ~3 s before GO with no stored launch frame: the timeline
must be aligned to the race start (--start-frame; ~188 worked for the Luigi staff ghost in
mkw-roblox's replay tool, but calibrate per boot flow).

Usage: rkg_to_padscript.py <ghost.rkg> <out.txt> [--start-frame N] [--b-to-r]
"""
import struct, sys

def yaz_decompress(data):
    assert data[:4] in (b"Yaz0", b"Yaz1"), data[:4]
    size = struct.unpack_from(">I", data, 4)[0]
    out = bytearray(); pos = 16; code = 0; bits = 0
    while len(out) < size:
        if bits == 0:
            code = data[pos]; pos += 1; bits = 8
        if code & 0x80:
            out.append(data[pos]); pos += 1
        else:
            b1, b2 = data[pos], data[pos + 1]; pos += 2
            dist = ((b1 & 0xF) << 8) | b2
            copy = b1 >> 4
            n = (data[pos] + 0x12, 3)[0] if copy == 0 else copy + 2
            if copy == 0:
                pos += 1
            start = len(out) - dist - 1
            for i in range(n):
                out.append(out[start + i])
        code <<= 1; bits -= 1
    return bytes(out)

def read_frames(path):
    d = open(path, "rb").read()
    assert d[:4] == b"RKGD", "not an RKG"
    compressed = (d[0xC] >> 3) & 1
    body = d[0x88:]
    payload = yaz_decompress(body[4:]) if compressed else body
    n_button, n_analog, n_trick = struct.unpack_from(">HHH", payload, 0)
    pos = 8
    frames = []
    for _ in range(n_button):
        state, dur = payload[pos], payload[pos + 1]; pos += 2
        frames.extend([[state, 7, 7]] * dur)
    idx = 0
    for _ in range(n_analog):
        state, dur = payload[pos], payload[pos + 1]; pos += 2
        x, y = state >> 4, state & 0xF
        for _ in range(dur):
            if idx < len(frames):
                frames[idx] = [frames[idx][0], x, y]
            idx += 1
    return frames

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    start = 0
    for a in sys.argv[1:]:
        if a.startswith("--start-frame"):
            start = int(a.split("=", 1)[1] if "=" in a else sys.argv[sys.argv.index(a) + 1])
    b_to_r = "--b-to-r" in sys.argv
    frames = read_frames(args[0])
    # GC stick full deflection ~ +/-72 in PADStatus units; RKG stick is 15 steps around 7,
    # with nibble 0 = full RIGHT (community docs + mkw-roblox replay validation), so X flips
    # into PADStatus's positive-right convention. Y polarity is unvalidated until a replayed
    # staff ghost reproduces its header time on hardware.
    scale_x = lambda v: round((7 - v) / 7.0 * 72)
    scale_y = lambda v: round((v - 7) / 7.0 * 72)
    lines, prev = [], None
    for i, (state, x, y) in enumerate(frames):
        toks = []
        if state & 1: toks.append("A")
        if state & 2: toks.append("R" if b_to_r else "B")
        if state & 4: toks.append("L")
        toks.append(f"SX={scale_x(x)}")
        toks.append(f"SY={scale_y(y)}")
        cur = " ".join(toks)
        if cur != prev:
            lines.append(f"F{start + i} {cur}")
            prev = cur
    with open(args[1], "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"{args[0]} -> {args[1]}: {len(frames)} frames, {len(lines)} steps, start F{start}")

if __name__ == "__main__":
    main()
