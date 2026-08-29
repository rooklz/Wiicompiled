#!/bin/sh
# deploy_ps3.sh — build, push to the console, and collect the results.
#
# The console round-trip is now one command and one button press. Getting here
# took working out which of webMAN's channels actually move bytes, because
# several look like they should and do not:
#
#   /install.ps3 <net path>   accepted, returns 200, never opens the file
#   /copy.ps3, /cpy.ps3 +     the same -- webMAN does not expose copy for
#   /paste.ps3, &to=          network paths at all, only for hdd and usb
#   ftp on port 21            closed
#
# What does work is webMAN's own FTP service, which is enabled by default but
# listens on the port in its settings rather than 21 -- 38008 on this console,
# the same number as the ps3netsrv default, which is why it reads like a
# misconfiguration and is not one.
#
# With a write channel there is no reason to install a package at all: the app
# is already registered with the XMB, so overwriting its EBOOT updates it in
# place. That removes the entire Package Manager dance from the loop. The
# package is still built and uploaded, because a first-time install on another
# console needs one.
#
#   ./tools/deploy_ps3.sh [ps3_ip] [timeout_seconds]

set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PS3=${1:-192.168.1.123}
TIMEOUT=${2:-1800}
FTP_PORT=${PS3_FTP_PORT:-38008}
APPID=WCPS3001
REMOTE=/dev_hdd0/tmp/wiicompiled-ps3-selftest.txt
OUT=$ROOT/build/ps3-selftest-results.txt

cd "$ROOT"

printf '== build ==\n'
docker run --rm -v "$ROOT":/work -w /work ps3dev:latest make -f ps3.mk pkg >/dev/null
EBOOT=build/pkg/USRDIR/EBOOT.BIN
RELOAD=${PS3_RELOAD:-build/pkg/USRDIR/RELOAD.SELF}
SPUELF=${PS3_SPUELF:-build/ppu/vtx_spu.elf}
printf '   EBOOT %s KB\n' "$(( $(wc -c < $EBOOT) / 1024 ))"

printf '== console ==\n'
if ! curl -s -m 6 "http://$PS3/index.ps3" -o /dev/null; then
    printf '   %s is not answering on HTTP; is it on?\n' "$PS3"
    exit 1
fi
if ! nc -z -G 3 "$PS3" "$FTP_PORT" 2>/dev/null; then
    printf '   no FTP on %s:%s -- check webMAN setup, "Disable FTP service"\n' \
        "$PS3" "$FTP_PORT"
    exit 1
fi

# In place, so the XMB entry keeps working and nothing has to be installed.
curl -s -m 1800 -T "$EBOOT" \
    "ftp://$PS3:$FTP_PORT/dev_hdd0/game/$APPID/USRDIR/EBOOT.BIN" >/dev/null
printf '   EBOOT written to /dev_hdd0/game/%s/USRDIR/\n' "$APPID"

# RELOAD.SELF too, and this is not optional.
#
# The in-emulator `relaunch` command -- the thing that makes unattended
# iteration possible at all -- prefers RELOAD.SELF and only falls back to
# EBOOT.BIN when it is absent. Uploading just the EBOOT therefore looks like a
# successful deploy and then boots the PREVIOUS build forever: the boot log
# even reports the new EBOOT's size, because the running process stats that
# path regardless of which image it was spawned from. That cost a long
# debugging detour once; keeping the two in lockstep is what prevents it.
if [ -f "$RELOAD" ]; then
    curl -s -m 1800 -T "$RELOAD" \
        "ftp://$PS3:$FTP_PORT/dev_hdd0/game/$APPID/USRDIR/RELOAD.SELF" >/dev/null
    printf '   RELOAD.SELF written (relaunch target)\n'
else
    printf '   WARNING: %s missing -- relaunch would boot the OLD build\n' "$RELOAD"
fi

# The SPU image, for the same reason as RELOAD.SELF: it is loaded from USRDIR
# at run time, and a stale one presents as a hung SPU rather than as an error.
if [ -f "$SPUELF" ]; then
    curl -s -m 600 -T "$SPUELF" \
        "ftp://$PS3:$FTP_PORT/dev_hdd0/game/$APPID/USRDIR/vtx_spu.elf" >/dev/null
    printf '   vtx_spu.elf written (%s bytes)\n' "$(wc -c < "$SPUELF" | tr -d ' ')"
else
    printf '   WARNING: %s missing -- SPU would run the OLD image\n' "$SPUELF"
fi

# The package is only needed by a console that does not have the app yet, and
# it is by far the largest upload; skip it unless asked.
if [ -n "${PS3_DEPLOY_PKG:-}" ]; then
    curl -s -m 1800 -T build/wiicompiled-ps3.pkg \
        "ftp://$PS3:$FTP_PORT/dev_hdd0/packages/wiicompiled-ps3.pkg" >/dev/null
    printf '   package staged in /dev_hdd0/packages/\n'
fi

# A stale report would be indistinguishable from a fresh one that never ran.
curl -s -m 10 "http://$PS3/delete.ps3$REMOTE" -o /dev/null || true

cat <<EOF

  On the console: launch "WiiCompiled PS3" from the Game menu.
  Nothing to install -- the app was updated in place.

  Waiting for results (timeout ${TIMEOUT}s).

EOF

elapsed=0
while [ "$elapsed" -lt "$TIMEOUT" ]; do
    code=$(curl -s -m 8 -o "$OUT.tmp" -w '%{http_code}' "http://$PS3$REMOTE" 2>/dev/null || echo 000)
    size=$(wc -c < "$OUT.tmp" 2>/dev/null | tr -d ' ')

    # webMAN answers 200 with an HTML error page for a missing file, so the
    # content has to be checked rather than the status code.
    if [ "$code" = "200" ] && [ "${size:-0}" -gt 32 ] && \
       ! grep -qi '<html\|wMAN' "$OUT.tmp" 2>/dev/null; then
        mv "$OUT.tmp" "$OUT"
        printf '\n================ PS3 SELF-TEST RESULTS ================\n'
        cat "$OUT"
        printf '======================================================\n(saved to %s)\n' "$OUT"
        exit 0
    fi
    sleep 10
    elapsed=$((elapsed + 10))
    [ $((elapsed % 120)) -eq 0 ] && printf '   ...waiting (%ss)\n' "$elapsed"
done

rm -f "$OUT.tmp"
printf 'timed out after %ss - not launched yet.\n' "$TIMEOUT"
exit 1
