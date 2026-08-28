# Wiimmfi UI overlay (NTSC-U)

`Scene/UI/*_{U,Q,M}.szs` are the disc's own UI archives with Wiimm's message replacements
(`wiimmfi-patcher-v7.5/bmg/wiimmfi-{U,Q,M}.txt`) applied by `wszst patch --patch-bmg`, exactly as
the official patcher does for a disc image. They only change text ("Nintendo WFC" -> "Wiimmfi"
and the related notices). The runtime mounts this directory as a disc-shaped overlay
(`[paths] overlay_roots` in Config.toml) for the Wiimmfi product; the clean disc stays untouched.

Regenerate with `./build-macos.sh --profile wiimmfi` (needs the patcher's macOS `wszst`, run under
Rosetta; set WIIMMFI_PATCHER_DIR to its directory).
