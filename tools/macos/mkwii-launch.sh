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
    retro-rewind) exe=$workspace/dist/$region-retro-rewind/RetroRewind;    overlays="\"$workspace/PulsarPacks/completed/RetroRewind/RetroRewind6\"" ;;
    *) echo "usage: $0 [base|wiimmfi|retro-rewind] [-- runtime args]" >&2; exit 2 ;;
esac
[[ -x "$exe" ]] || { echo "$profile is not built: $exe (run ./build-macos.sh --profile $profile)" >&2; exit 1; }
[[ -f "$config" ]] || { echo "no Config.toml at $config; run the base product once or create it (see docs/MACOS.md)" >&2; exit 1; }

# Replace (or append) the overlay_roots line inside [paths].
python3 - "$config" "$overlays" <<'PY'
import re, sys
path, overlays = sys.argv[1], sys.argv[2]
text = open(path, encoding="utf-8").read()
line = f"overlay_roots = [{overlays}]\n" if overlays else ""
if re.search(r"^\s*#?\s*overlay_roots\s*=.*$", text, re.M):
    text = re.sub(r"^\s*#?\s*overlay_roots\s*=.*\n?", line, text, count=1, flags=re.M)
elif line:
    if "[paths]" in text:
        text = text.replace("[paths]\n", "[paths]\n" + line, 1)
    else:
        text += "\n[paths]\n" + line
open(path, "w", encoding="utf-8").write(text)
PY
echo "launching $profile: $exe"
exec "$exe" "$@"
