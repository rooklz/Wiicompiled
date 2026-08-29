#!/bin/sh
# Upload the two files the console actually runs, re-arm port mode, relaunch.
#
# RELOAD.SELF is NOT optional: the rescue listener's relaunch spawns THAT file,
# not EBOOT.BIN. Uploading only the EBOOT relaunches the previous build and
# every conclusion drawn from the run afterwards is about the old image.
set -e
# Self-serializing: concurrent deploys corrupt each other's FTP windows, and
# caller-side pgrep gates deadlocked on their own command lines containing
# this script's name. The lock lives HERE now; callers just call.
while ! mkdir /tmp/wiicompiled-ps3-deploy.lock 2>/dev/null; do sleep 5; done
trap 'rmdir /tmp/wiicompiled-ps3-deploy.lock' EXIT
PS3=${PS3_IP:-192.168.1.123}
cd "$(dirname "$0")/.."
# Upload from a SNAPSHOT: a build finishing mid-upload rewrites these files
# under curl and the console ends up holding a mixed image -- nearly happened
# once (pre-build overlapped a deploy's FTP window). The copy is atomic enough:
# cp completes before curl starts.
mkdir -p build/deploy-stage
cp build/pkg/USRDIR/EBOOT.BIN build/pkg/USRDIR/RELOAD.SELF build/deploy-stage/
# FAST MODE IS THE DEFAULT: the rescue relaunch boots RELOAD.SELF, so EBOOT
# only matters for XMB launches. Uploading both cost ~2x the deploy time of
# every iteration all session. FULL=1 uploads both.
if [ -n "${FULL:-}" ]; then
    curl -s -m 1800 -T build/deploy-stage/EBOOT.BIN \
        "ftp://$PS3:21/dev_hdd0/game/WCPS3001/USRDIR/EBOOT.BIN"
    echo "  EBOOT.BIN uploaded (FULL=1)"
fi
curl -s -m 1800 -T build/deploy-stage/RELOAD.SELF \
    "ftp://$PS3:21/dev_hdd0/game/WCPS3001/USRDIR/RELOAD.SELF"
echo "  RELOAD.SELF uploaded"
if [ -z "${NO_PORT:-}" ]; then
    printf 'port\n' > /tmp/wcboot.txt
    curl -s -m 60 -T /tmp/wcboot.txt "ftp://$PS3:21/dev_hdd0/tmp/wiicompiled-wcboot.txt"
    echo "  port mode armed (one-shot)"
fi
curl -s -m 20 "http://$PS3/delete.ps3/dev_hdd0/tmp/wiicompiled-ps3-selftest.txt" -o /dev/null || true
printf 'relaunch\n' | nc -w 8 "$PS3" 4001 | head -1
