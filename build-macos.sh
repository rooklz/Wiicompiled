#!/usr/bin/env bash
# Local macOS (Apple silicon) build of WiiCompiled: translate the user's own executables, then
# compile the runtime and the translated code into a native arm64 binary with the Metal backend.
#
#   ./build-macos.sh --disc <image.rvz|.iso|.wbfs|dir>   extract (nodtool) or use an extracted
#                                                        DATA directory containing sys/ and files/
#   ./build-macos.sh --region rmce01|rmcp01              executable region (default: detected)
#   ./build-macos.sh --clean                             discard translation and build caches
#   ./build-macos.sh --parallel N                        pin all parallelism to N
#   ./build-macos.sh --cpu <-mcpu flag>                  e.g. -mcpu=apple-m4 (default -mcpu=native)
#   ./build-macos.sh --profile base|wiimmfi              wiimmfi: translate the executables patched by
#                                                        Wiimm's patcher (WIIMMFI_PATCHER_DIR, run under
#                                                        Rosetta) and build the Wiimmfi UI overlay
#
# Every step mirrors Launcher/LocalBuild.ps1 (Windows) and Launcher/local-build.sh (Linux).
set -euo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
disc=""; region=""; clean=0; parallel=0; cpu_flag="-mcpu=native"; extract_dir="$workspace/disc"; profile="base"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --disc) disc=$2; shift 2 ;;
        --profile) profile=$2; shift 2 ;;
        --region) region=$2; shift 2 ;;
        --clean) clean=1; shift ;;
        --parallel) parallel=$2; shift 2 ;;
        --cpu) cpu_flag=$2; shift 2 ;;
        --extract-dir) extract_dir=$2; shift 2 ;;
        -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

fail() { echo "MKWCBUILD: ERROR: $*" >&2; exit 1; }
step() { echo "MKWCBUILD:STEP:$1 $2"; }
need() { command -v "$1" >/dev/null 2>&1 || fail "$1 is required ($2)"; }

# --- toolchain ---------------------------------------------------------------------------------
[[ "$(uname -s)" == "Darwin" && "$(uname -m)" == "arm64" ]] || fail "this script targets macOS on Apple silicon"
need cmake "brew install cmake"; need ninja "brew install ninja"
dotnet_bin=${DOTNET_BIN:-}
for candidate in "$dotnet_bin" "$HOME/.dotnet/dotnet" "$(command -v dotnet || true)"; do
    [[ -n "$candidate" && -x "$candidate" ]] && { dotnet_bin=$candidate; break; }
done
[[ -x "${dotnet_bin:-}" ]] || fail ".NET 8 SDK (dotnet) not found; install with https://dot.net/v1/dotnet-install.sh --channel 8.0"
export DOTNET_ROOT=$(dirname "$dotnet_bin") DOTNET_CLI_TELEMETRY_OPTOUT=1 DOTNET_NOLOGO=1
nodtool_bin=${NODTOOL_BIN:-$(command -v nodtool || echo "$HOME/.cargo/bin/nodtool")}
cc_bin=${CC:-$(xcrun -f clang)}; cxx_bin=${CXX:-$(xcrun -f clang++)}
[[ -x "$cc_bin" && -x "$cxx_bin" ]] || fail "Xcode command line tools (clang) are required"

cpu_count=$(sysctl -n hw.ncpu); mem_gib=$(( $(sysctl -n hw.memsize) / 1024 / 1024 / 1024 ))
if (( parallel > 0 )); then translator_threads=$parallel; translated_jobs=$parallel; global_jobs=$parallel
else
    translator_threads=$(( cpu_count < 16 ? cpu_count : 16 ))
    translated_jobs=$(( mem_gib / 2 )); (( translated_jobs > cpu_count )) && translated_jobs=$cpu_count; (( translated_jobs < 1 )) && translated_jobs=1
    global_jobs=$cpu_count
fi

# --- game inputs -------------------------------------------------------------------------------
assets=$workspace/Assets
if [[ -n "$disc" ]]; then
    if [[ -d "$disc" ]]; then
        data_root=$disc
    else
        [[ -x "$nodtool_bin" ]] || fail "nodtool is required to extract a disc image (cargo install nodtool)"
        data_root=$extract_dir/$("$nodtool_bin" info "$disc" | awk '/Game ID:/ {print $3; exit}')
        if [[ ! -f "$data_root/sys/main.dol" ]]; then
            step extract-disc "Extracting the data partition of $disc"
            mkdir -p "$extract_dir"; "$nodtool_bin" extract -q -p data "$disc" "$data_root"
        fi
    fi
    [[ -f "$data_root/sys/main.dol" && -f "$data_root/files/rel/StaticR.rel" ]] || fail "$data_root is not an extracted DATA directory"
    mkdir -p "$assets"
    cp -f "$data_root/sys/main.dol" "$assets/main.dol"
    cp -f "$data_root/files/rel/StaticR.rel" "$assets/StaticR.rel"
    echo "$data_root" > "$assets/dvd_root.txt"
fi
[[ -f "$assets/main.dol" && -f "$assets/StaticR.rel" ]] || fail "Assets/main.dol and Assets/StaticR.rel are missing; pass --disc"

game_id=$(dd if="$assets/main.dol" bs=1 count=0 2>/dev/null; python3 - "$assets/main.dol" <<'PY'
import sys, hashlib
h = hashlib.sha256(open(sys.argv[1], 'rb').read()).hexdigest()
print({'80d18895b39c63bd80f457398bfcbb91b7d16ac116a41a88967e954080155b05': 'rmcp01',
       'd2beec1b1645fcd134efe9e7e63774b546667764ed8d431029daccd725995694': 'rmce01'}.get(h, 'unknown'))
PY
)
[[ -n "$region" ]] || region=$game_id
[[ "$region" == "rmcp01" || "$region" == "rmce01" ]] || fail "unrecognised executable (sha256 not a clean RMCP01/RMCE01 main.dol); pass --region explicitly"
case "$region" in
    rmcp01) project=$workspace/projects/mkwii/recomp.yml; manifest_region=P ;;
    rmce01) project=$workspace/projects/mkwii-ntsc/recomp.yml; manifest_region=E ;;
esac
product_name=WiiCompiled
generated_name=generated
if [[ "$profile" == "wiimmfi" ]]; then
    [[ "$region" == "rmce01" ]] || fail "the wiimmfi profile is only wired for RMCE01 here"
    patcher=${WIIMMFI_PATCHER_DIR:-$HOME/Downloads/wiimmfi-patcher-v7.5}
    wszst=$patcher/bin/mac/wszst
    if [[ ! -f "$assets/wiimmfi/main.dol" || ! -f "$assets/wiimmfi/StaticR.rel" ]]; then
        [[ -x "$wszst" ]] || fail "Wiimmfi patcher not found at $patcher (set WIIMMFI_PATCHER_DIR); it provides bin/mac/wszst"
        step wiimmfi-patch "Applying Wiimm's Wiimmfi patch to copies of the executables"
        mkdir -p "$assets/wiimmfi"
        cp -f "$assets/main.dol" "$assets/wiimmfi/main.dol"; cp -f "$assets/StaticR.rel" "$assets/wiimmfi/StaticR.rel"
        arch -x86_64 "$wszst" wstrt patch "$assets/wiimmfi/main.dol" "$assets/wiimmfi/StaticR.rel" --clean-dol --wiimmfi --all-ranks
    fi
    overlay=$workspace/overlays/wiimmfi/Scene/UI
    if [[ -f "$assets/dvd_root.txt" && ! -d "$overlay" ]]; then
        [[ -x "$wszst" ]] || fail "Wiimmfi patcher not found at $patcher (needed for the UI text overlay)"
        step wiimmfi-overlay "Building the Wiimmfi UI text overlay"
        mkdir -p "$overlay"
        for lang in U Q M E F G I S J K; do
            for f in "$(cat "$assets/dvd_root.txt")"/files/Scene/UI/*_"$lang".szs; do
                [[ -f "$f" && -f "$patcher/bmg/wiimmfi-$lang.txt" ]] || continue
                cp -f "$f" "$overlay/$(basename "$f")"
                arch -x86_64 "$wszst" -q patch "$overlay/$(basename "$f")" --ignore --patch-bmg "repl=$patcher/bmg/wiimmfi-$lang.txt"
            done
        done
    fi
    project=$workspace/projects/mkwii-ntsc-wiimmfi/recomp.yml
    product_name=WiiCompiled-Wiimmfi
    generated_name=generated-wiimmfi
fi
echo "MKWCBUILD: region=$region profile=$profile project=$(basename "$(dirname "$project")") cpu=$cpu_flag jobs=$global_jobs translated=$translated_jobs"

# --- translator --------------------------------------------------------------------------------
translator_dll=$workspace/translator/src/Translator.Cli/bin/Release/net8.0/Translator.Cli.dll
step build-translator "Building the translator"
"$dotnet_bin" build "$workspace/translator/src/Translator.Cli/Translator.Cli.csproj" -c Release --nologo -v q
translator() { "$dotnet_bin" "$translator_dll" "$@"; }

generated=$workspace/$generated_name; functions=$generated/functions
base_metadata=$generated/base_translation_output.json
# (No `$([[ test ]] && echo x)` substitutions here: under set -e an assignment whose command
# substitution returns non-zero - which that idiom does whenever the test is false - aborts.)
if [[ "$profile" == "wiimmfi" ]]; then
    base_manifest_dir=$workspace/build/wiimmfi; build=$workspace/native-build-$region-wiimmfi; dist=$workspace/dist/$region-wiimmfi
else
    base_manifest_dir=$workspace/build/base; build=$workspace/native-build-$region; dist=$workspace/dist/$region
fi
base_manifest=$base_manifest_dir/mkwii_base_manifest.json
shards=$generated/build_shards
provenance=$generated/translation-provenance.json
if (( clean )); then rm -rf "$generated" "$base_manifest_dir" "$build"; fi

if [[ "$region" == "rmce01" ]]; then
    step region-table "Regenerating the guest address tables"
    # MKW_REGION_TABLE_PROVISIONAL=1 accepts unverified data addresses (toolchain validation
    # builds only; the resulting binary is not for play and its header says so).
    python3 "$workspace/tools/region/gen_region_headers.py" ${MKW_REGION_TABLE_PROVISIONAL:+--provisional}
fi

entry_point=$(awk '/^translation:/{t=1} t && /^[[:space:]]*-[[:space:]]*0[xX][0-9a-fA-F]+/{gsub(/^[[:space:]]*-[[:space:]]*/,""); print; exit}' "$project")
dol_in=$assets/main.dol; rel_in=$assets/StaticR.rel
[[ "$profile" == "wiimmfi" ]] && { dol_in=$assets/wiimmfi/main.dol; rel_in=$assets/wiimmfi/StaticR.rel; }
# Everything the base translation depends on: the inputs, the project, the region table, the
# translator build itself, and the runtime sources (their native registrations decide which
# functions are excluded from translation and which call sites bind natively).
runtime_sources_hash=$(find "$workspace/runtime/src" "$workspace/runtime/include" -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.inc' \) -not -path '*/third_party/*' | LC_ALL=C sort | xargs shasum -a 256 | shasum -a 256 | cut -d' ' -f1)
fingerprint=$(cat "$dol_in" "$rel_in" "$project" "$workspace/runtime/include/region/$region.h" "$translator_dll" <(echo "$runtime_sources_hash") | shasum -a 256 | cut -d' ' -f1)
reuse=0
if [[ -f "$provenance" ]] && grep -q "\"$fingerprint\"" "$provenance" && [[ -f "$base_metadata" && -f "$base_manifest" ]]; then reuse=1; fi
if (( reuse )); then
    step reuse-base-translation "Reusing the completed base translation"
else
    rm -f "$provenance"; mkdir -p "$generated" "$base_manifest_dir"
    step translate-base "Translating the user-owned base game ($region)"
    translator translate-recursive "$entry_point" --project "$project" \
        --outdir "$functions" --output-metadata "$base_metadata" \
        --production-source-bundle "$generated/base_translation_sources.bin" \
        --no-function-files --prune-stale --threads "$translator_threads"
    step emit-base-manifest "Creating the local base translation manifest"
    translator emit-base-manifest --project "$project" --out "$base_manifest_dir" \
        --functions-dir "$functions" --translation-output-metadata "$base_metadata" --region "$manifest_region"
    printf '{"SchemaVersion":1,"TranslationFingerprint":"%s"}\n' "$fingerprint" > "$provenance"
fi
step generate-data-init "Generating local game data initialization"
translator generate-data-init --project "$project"
step emit-build-shards "Preparing local native build shards"
translator emit-build-shards --project "$project" --base-metadata "$base_metadata" \
    --base-functions-dir "$functions" --native-source-dir "$workspace/runtime/src" --out "$shards"

# --- native build ------------------------------------------------------------------------------
step configure-native "Configuring the native build ($build)"
cmake -S "$workspace/runtime" -B "$build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER="$cc_bin" -DCMAKE_CXX_COMPILER="$cxx_bin" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=14.0 \
    -DAURORA_DAWN_PROVIDER=package -DAURORA_SDL3_PROVIDER=vendor -DAURORA_SDL3_LINKAGE=static \
    -DMKW_GUEST_REGION="$region" -DMKW_AARCH64_CPU_FLAG="$cpu_flag" -DMKW_GENERATED_DIR="$generated" \
    -DMKW_TRANSLATED_COMPILE_JOBS="$translated_jobs"
step compile "Compiling WiiCompiled"
cmake --build "$build" --target WiiCompiled --parallel "$global_jobs"

# --- publish -----------------------------------------------------------------------------------
mkdir -p "$dist"
cp -f "$build/WiiCompiled" "$dist/$product_name"
for name in dsp_coef.bin initial_pipeline_cache.db; do [[ -f "$build/$name" ]] && cp -f "$build/$name" "$dist/"; done
[[ -d "$build/wii_bootstrap" ]] && rm -rf "$dist/wii_bootstrap" && cp -R "$build/wii_bootstrap" "$dist/wii_bootstrap"
cat > "$dist/local-build.json" <<JSON
{
  "SchemaVersion": 1,
  "Profile": "$profile",
  "Region": "$region",
  "DolSha256Source": "$dol_in",
  "BuiltUtc": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "DolSha256": "$(shasum -a 256 "$dol_in" | cut -d' ' -f1)",
  "RelSha256": "$(shasum -a 256 "$rel_in" | cut -d' ' -f1)",
  "Compiler": "$("$cxx_bin" --version | head -1)",
  "CpuFlag": "$cpu_flag"
}
JSON
echo "MKWCBUILD:OUTPUT=$dist"
[[ -f "$assets/dvd_root.txt" ]] && echo "MKWCBUILD: set [paths] dvd_root = \"$(cat "$assets/dvd_root.txt")\" in Config.toml (see README)"
[[ "$profile" == "wiimmfi" ]] && echo "MKWCBUILD: and overlay_roots = [\"$workspace/overlays/wiimmfi\"] for the Wiimmfi UI text"
true
