#!/usr/bin/env bash
# Launch a built product with the matching overlay configuration.
#
#   mkwii-launch.sh [base|wiimmfi|retro-rewind] [-- <runtime args>]
#
# The products are separate native executables (that is how WiiCompiled keeps each profile
# statically bound, with no runtime patching); what differs at launch time is only which binary
# runs and which disc-shaped overlay directories the runtime mounts over the extracted disc.
# This script rewrites the `overlay_roots` line of Config.toml accordingly and starts the
# product. Everything else in Config.toml (dvd_root, video, audio, controllers) is shared.
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd "$here/../.." && pwd)
profile=${1:-base}; shift || true
[[ "${1:-}" == "--" ]] && shift
config="$HOME/Library/Application Support/WiiCompiled/Config.toml"
region=${MKW_REGION:-rmce01}

case "$profile" in
    base)         exe=$workspace/dist/$region/WiiCompiled;                 overlays="" ;;
    wiimmfi)      exe=$workspace/dist/$region-wiimmfi/WiiCompiled-Wiimmfi; overlays="\"$workspace/overlays/wiimmfi\"" ;;
    retro-rewind) exe=$workspace/dist/$region-retro-rewind/RetroRewind;    overlays=""; retro_root=$workspace/PulsarPacks/completed/RetroRewind/RetroRewind6 ;;
    *) echo "usage: $0 [base|wiimmfi|retro-rewind] [-- runtime args]" >&2; exit 2 ;;
esac
[[ -x "$exe" ]] || { echo "$profile is not built: $exe (run ./build-macos.sh --profile $profile)" >&2; exit 1; }
[[ -f "$config" ]] || { echo "no Config.toml at $config; run the base product once or create it (see docs/MACOS.md)" >&2; exit 1; }

# Replace (or append) the overlay_roots / retro_rewind_root lines inside [paths]. The Retro Rewind
# product reads its pack through retro_rewind_root (the runtime mounts the pack's Riivolution XML
# from there); the other products use plain disc-shaped overlays.
python3 - "$config" "$overlays" "${retro_root:-}" <<'PY'
import re, sys
path, overlays, retro_root = sys.argv[1], sys.argv[2], sys.argv[3]
text = open(path, encoding="utf-8").read()
def set_line(text, key, line):
    if re.search(rf"^\s*#?\s*{key}\s*=.*$", text, re.M):
        return re.sub(rf"^\s*#?\s*{key}\s*=.*\n?", line, text, count=1, flags=re.M)
    if line:
        return text.replace("[paths]\n", "[paths]\n" + line, 1) if "[paths]" in text else text + "\n[paths]\n" + line
    return text
text = set_line(text, "overlay_roots", f"overlay_roots = [{overlays}]\n" if overlays else "")
text = set_line(text, "retro_rewind_root", f'retro_rewind_root = "{retro_root}"\n' if retro_root else "")
open(path, "w", encoding="utf-8").write(text)
PY
echo "launching $profile: $exe"
exec "$exe" "$@"
