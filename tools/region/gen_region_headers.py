#!/usr/bin/env python3
"""Generate runtime/include/region/<region>.h from the guest-address identities the runtime uses.

The runtime spells every guest address by its PAL (RMCP01) identity (see region/guest_region.h).
This tool scans the runtime for those identities and writes, per region, the `#define MKW_G_<pal>
<region>` table the compiler and the translator both resolve them through, plus the region facts
(game code, TV format, SC area/game indices, product code).

  rmcp01: identity table.
  rmce01: code addresses through the community PAL->NTSC-U chunk table (mkw-sp port.py, MIT),
          cross-checked against the validated NTSC-U function map when present; data addresses
          from projects/mkwii-ntsc/data_addresses.txt (each line `<pal> <ntsc> <evidence>`), which
          records the disassembly evidence for every value. Any identity that neither source
          resolves aborts the generation - a region table is complete or it is not written.
"""
import os, re, sys, json, collections

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
RUNTIME = os.path.join(ROOT, "runtime")
OUT_DIR = os.path.join(RUNTIME, "include", "region")

PAL_TEXT = [(0x80004000, 0x80006460), (0x800072C0, 0x80244DE0), (0x805103B4, 0x8088F400)]

REGIONS = {
    "rmcp01": {
        "game_id": "RMCP01", "game_code": 0x524D4350, "region_letter": "P",
        "vi_tv_format": 1, "sc_area": 2, "sc_game_region": 2, "sc_product_code": "LEH",
        # The executable's initial stack top (__init_registers: lis/ori r1), which is where the
        # SDK's OSInit starts the MEM1 arena; the boot heap and therefore StaticR.rel's load
        # address derive from it.
        "mem1_arena_lo": 0x80399180,
        "display": "Mario Kart Wii PAL (RMCP01)",
    },
    "rmce01": {
        "game_id": "RMCE01", "game_code": 0x524D4345, "region_letter": "E",
        "vi_tv_format": 0, "sc_area": 1, "sc_game_region": 1, "sc_product_code": "LU",
        # Verified: RMCE01 __init_registers @80006284 lis r1,0x8039 / @80006288 ori r1,r1,0x4e00.
        "mem1_arena_lo": 0x80394E00,
        "display": "Mario Kart Wii NTSC-U (RMCE01)",
    },
}

TOKEN_PATTERNS = [
    re.compile(r"(?:PPC_NATIVE_OVERRIDE(?:_VOID)?|GX_FATAL_STUB)\s*\(\s*([0-9A-Fa-f]{8})\b"),
    re.compile(r"MKW_GADDR\s*\(\s*([0-9A-Fa-f]{8})\s*\)"),
    re.compile(r"MKW_GUEST_FUNC\s*\(\s*([0-9A-Fa-f]{8})\s*\)"),
]


def scan_tokens():
    """Every PAL identity token as it is spelled in the sources (case preserved)."""
    spellings = collections.defaultdict(set)
    for dp, _, fns in os.walk(RUNTIME):
        if "third_party" in dp or os.path.join("include", "region") in dp:
            continue
        for fn in fns:
            if not fn.endswith((".cpp", ".h", ".hpp", ".inc")):
                continue
            text = open(os.path.join(dp, fn), encoding="utf-8", errors="replace").read()
            for pat in TOKEN_PATTERNS:
                for m in pat.finditer(text):
                    spellings[int(m.group(1), 16)].add(m.group(1))
    return spellings


def is_text(addr):
    return any(a <= addr < b for a, b in PAL_TEXT)


def load_chunks():
    path = os.path.join(ROOT, "projects", "mkwii-ntsc", "region_port.json")
    if os.path.exists(path):
        def num(v):
            return int(v, 16) if isinstance(v, str) else int(v)
        return [tuple(num(v) for v in c) for c in json.load(open(path))["chunks"]]
    sys.exit(f"missing {path} (produced by tools/region/port_map.py)")


def port_code(chunks, addr):
    for s, e, d in chunks:
        if s <= addr < e:
            return addr - s + d
    return None


def load_data_table(region):
    path = os.path.join(ROOT, "projects", "mkwii-ntsc", "data_addresses.txt")
    table = {}
    if not os.path.exists(path):
        return table, path
    for line in open(path):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2 or parts[1] == "?":
            continue
        table[int(parts[0], 16)] = int(parts[1], 16)
    return table, path


def load_map(path):
    entries = []
    for line in open(path):
        line = line.strip()
        if line and not line.startswith("#"):
            addr, _, name = line.partition(" ")
            entries.append((int(addr, 16), name.strip()))
    entries.sort()
    return entries


def load_validated_map():
    path = os.path.join(ROOT, "projects", "mkwii-ntsc", "MAP.txt")
    if not os.path.exists(path):
        return None
    return load_map(path)


LABEL_NAME = re.compile(r"(_switch$|_caseD_|^0x)")


def containing_function(pal_entries, addr):
    """Nearest PAL map entry at or below addr that names a real function (not a jump-table
    label or an unnamed entry), and addr's offset inside it."""
    import bisect
    addrs = [a for a, _ in pal_entries]
    i = bisect.bisect_right(addrs, addr) - 1
    while i >= 0 and LABEL_NAME.search(pal_entries[i][1]):
        i -= 1
    if i < 0:
        return None, None, None
    start, name = pal_entries[i]
    return start, name, addr - start


def write_header(region, facts, mapping, spellings, provenance):
    path = os.path.join(OUT_DIR, f"{region}.h")
    lines = [
        "// AUTO-GENERATED by tools/region/gen_region_headers.py - DO NOT EDIT.",
        f"// Guest address table for {facts['display']}: PAL identity -> {region} address.",
        f"// {provenance}",
        "#pragma once",
        "",
        f"#define MKW_REGION_GAME_ID \"{facts['game_id']}\"",
        f"#define MKW_REGION_LETTER '{facts['region_letter']}'",
        f"#define MKW_REGION_GAME_CODE 0x{facts['game_code']:08X}u  // \"{facts['game_id'][:4]}\"",
        f"#define MKW_REGION_VI_TV_FORMAT {facts['vi_tv_format']}u  // 0 = VI_NTSC, 1 = VI_PAL",
        f"#define MKW_REGION_SC_AREA {facts['sc_area']}u  // SC AREA index: JPN=0 USA=1 EUR=2",
        f"#define MKW_REGION_SC_GAME_REGION {facts['sc_game_region']}u  // SC GAME index: JP=0 US=1 EU=2",
        f"#define MKW_REGION_SC_PRODUCT_CODE \"{facts['sc_product_code']}\"",
        f"#define MKW_REGION_MEM1_ARENA_LO 0x{facts['mem1_arena_lo']:08X}u  // initial stack top / arena lo",
        "",
    ]
    for pal in sorted(mapping):
        target = mapping[pal]
        for spelling in sorted(spellings[pal]):
            # An identity table keeps each token's own spelling so the PAL build's generated
            # symbol names (func_<token>, gx_stub_<token>) stay byte-identical to upstream.
            token = spelling if target == pal else f"{target:08X}"
            lines.append(f"#define MKW_G_{spelling} {token}")
    lines.append("")
    os.makedirs(OUT_DIR, exist_ok=True)
    open(path, "w").write("\n".join(lines))
    return path


def main():
    spellings = scan_tokens()
    identities = sorted(spellings)
    print(f"{len(identities)} guest-address identities in the runtime "
          f"({sum(1 for a in identities if is_text(a))} code, {sum(1 for a in identities if not is_text(a))} data)")

    # PAL: identity.
    path = write_header("rmcp01", REGIONS["rmcp01"], {a: a for a in identities}, spellings,
                        "Identity table: the runtime is written against this executable.")
    print("wrote", path)

    # NTSC-U.
    chunks = load_chunks()
    data_table, data_path = load_data_table("rmce01")
    validated_entries = load_validated_map()
    validated = {a for a, _ in validated_entries} if validated_entries is not None else None
    validated_by_name = {}
    if validated_entries is not None:
        for a, n in validated_entries:
            validated_by_name.setdefault(n, a)
    pal_entries = load_map(os.path.join(ROOT, "projects", "mkwii", "MAP.txt"))
    provisional = "--provisional" in sys.argv
    mapping, problems, provisional_data = {}, [], []
    for pal in identities:
        if is_text(pal):
            ported = port_code(chunks, pal)
            if ported is None:
                problems.append(f"  {pal:08X}: code address outside every port chunk")
                continue
            if validated is not None and ported not in validated:
                # Not a function entry: a jump-table label or a mid-function hook. It is right
                # exactly when it sits at the same offset inside a validated containing function,
                # i.e. that function's NTSC-U entry plus the PAL offset lands on the ported value
                # (which also proves the function and the hook share one port chunk).
                start, name, offset = containing_function(pal_entries, pal)
                ntsc_start = validated_by_name.get(name) if name else None
                if ntsc_start is None or ntsc_start + offset != ported:
                    problems.append(f"  {pal:08X} -> {ported:08X}: not a validated NTSC-U function entry "
                                    f"(containing PAL function {name} @ {start:08X if start else 0} "
                                    f"not confirmed at the same offset)")
                    continue
                print(f"  hook {pal:08X} -> {ported:08X}: {name}+{offset:#x} (function entry validated)")
            mapping[pal] = ported
        else:
            if pal in data_table:
                mapping[pal] = data_table[pal]
            elif provisional:
                # Toolchain-validation builds only: an unverified data address is left at its PAL
                # value, which is wrong at run time. The header says so in its first line.
                mapping[pal] = pal
                provisional_data.append(pal)
            else:
                problems.append(f"  {pal:08X}: data address missing from {os.path.relpath(data_path, ROOT)}")
    if problems:
        print("rmce01 table is incomplete; not written:\n" + "\n".join(problems))
        sys.exit(1)
    provenance = ("Code: mkw-sp PAL->NTSC-U chunk table (validated against projects/mkwii-ntsc/MAP.txt); "
                  "data: projects/mkwii-ntsc/data_addresses.txt (disassembly evidence per entry).")
    if provisional_data:
        provenance = ("PROVISIONAL - NOT FOR PLAY: " + str(len(provisional_data)) +
                      " data addresses left at their PAL values pending verification. " + provenance)
        print(f"WARNING: provisional rmce01 table, {len(provisional_data)} data identities unverified:",
              " ".join(f"{a:08X}" for a in provisional_data))
    path = write_header("rmce01", REGIONS["rmce01"], mapping, spellings, provenance)
    print("wrote", path)


if __name__ == "__main__":
    main()
