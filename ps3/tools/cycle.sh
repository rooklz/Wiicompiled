#!/bin/sh
# Fiber-model builds: FIBER=1 cycle.sh ... ; the two models share build/ppu,
# so a flag flip must purge the flag-sensitive objects (make cannot see -D
# changes). A marker file remembers the last state.
MAKEVARS=""
FLIPMARK=build/.last_fiber_state
want="${FIBER:-0}"
have="$(cat $FLIPMARK 2>/dev/null || echo unset)"
if [ "$want" != "$have" ]; then
  rm -f build/ppu/src/core/ppc/wc/wc_os.o build/ppu/src/core/ppc/wc/wc_sched.o \
        build/ppu/src/core/ppc/wc/wc_fiber.o build/ppu/src/core/ppc/wc/fiber_ps3.o \
        build/ppu/src/core/ppc/wc/gen/wc_calls.o
  echo "$want" > $FLIPMARK
fi
[ "$want" = "1" ] && MAKEVARS="FIBER=1"
# ppc_runtime.h is inlined into wc_calls.o; make cannot see header edits
# through the generated .cpp's mtime. Purge it when the header is newer.
if [ src/core/ppc/wc/ppc_runtime.h -nt build/ppu/src/core/ppc/wc/gen/wc_calls.o ] 2>/dev/null; then
  rm -f build/ppu/src/core/ppc/wc/gen/wc_calls.o
fi
# One fast iteration: build, upload, relaunch, and wait ADAPTIVELY.
#
# Replaces the hand-rolled "build_pkg.sh && redeploy.sh && sleep 115 && poll
# every 20 s" chain. Measured costs of that chain, per iteration:
#
#   build (make pkg)   32 s   <- 17 s of it writes a 59 MB EBOOT and a 118 MB
#                                PKG that the rescue relaunch never reads
#   upload RELOAD.SELF  8 s   (59 MB at ~7.4 MB/s -- fine, leave it)
#   sleep 115         115 s   <- fixed, regardless of how fast the boot is
#   poll every 20 s   0-480 s <- coarse: a result that lands at t+3 s is not
#                                seen until t+20 s
#
# So ~85% of a cycle was MY OWN WAITING, not the machinery. This script builds
# only what is deployed and polls on a 3 s grain from the moment the console
# comes back, exiting the instant the pattern shows.
#
#   cycle.sh                 -- build, deploy, boot, report when settled
#   cycle.sh 'report mode'   -- ... and stop as soon as that appears
#   cycle.sh 'PROF interval' 600
set -e
cd "$(dirname "$0")/.."
PS3=${PS3_IP:-192.168.1.123}
PAT=${1:-}
MAX=${2:-300}
LOG="ftp://$PS3/dev_hdd0/tmp/wiicompiled-ps3-selftest.txt"

T0=$(date +%s)
if [ -z "${NO_BUILD:-}" ]; then
    exec 9> /tmp/wiicompiled-ps3-build.lock
    if command -v flock >/dev/null 2>&1; then flock 9; fi
    if docker run --rm -m 3g -v "$(pwd)":/work -w /work ps3dev:latest \
           make -f ps3.mk $MAKEVARS self > /tmp/wiicompiled-ps3-build.log 2>&1; then
        :
    else
        echo "BUILD FAILED:"; grep -E 'error|Error' /tmp/wiicompiled-ps3-build.log | head -10
        exit 1
    fi
    mkdir -p build/deploy-stage
    cp build/wiicompiled-ps3.self build/deploy-stage/RELOAD.SELF
fi
echo "  build ok ($(($(date +%s)-T0))s)"

T1=$(date +%s)
while ! mkdir /tmp/wiicompiled-ps3-deploy.lock 2>/dev/null; do sleep 3; done
trap 'rmdir /tmp/wiicompiled-ps3-deploy.lock 2>/dev/null || true' EXIT
curl -s -m 600 -T build/deploy-stage/RELOAD.SELF \
     "ftp://$PS3:21/dev_hdd0/game/WCPS3001/USRDIR/RELOAD.SELF"
printf 'port\n' > /tmp/wcboot.txt
curl -s -m 60 -T /tmp/wcboot.txt "ftp://$PS3:21/dev_hdd0/tmp/wiicompiled-wcboot.txt"
curl -s -m 20 "http://$PS3/delete.ps3/dev_hdd0/tmp/wiicompiled-ps3-selftest.txt" -o /dev/null || true
printf 'relaunch\n' | nc -w 8 "$PS3" 4001 | head -1
echo "  deployed + relaunched ($(($(date +%s)-T1))s)"

# ADAPTIVE WAIT. Two phases, both on a 3 s grain:
#   1. the log reappears  -> the new image is running (was: blind sleep 115)
#   2. the pattern shows, or the log stops growing for 30 s (settled/stalled)
T2=$(date +%s)
BOOTED=0
while [ $(($(date +%s)-T2)) -lt "$MAX" ]; do
    SZ=$(curl -s -m 8 "$LOG" 2>/dev/null | wc -c | tr -d ' ')
    if [ "${SZ:-0}" -gt 200 ]; then BOOTED=1; break; fi
    sleep 3
done
[ "$BOOTED" = 1 ] && echo "  console back ($(($(date +%s)-T2))s)" \
                  || { echo "  NO BOOT within ${MAX}s"; exit 1; }

LAST=0; STILL=0
while [ $(($(date +%s)-T2)) -lt "$MAX" ]; do
    L=$(curl -s -m 8 "$LOG" 2>/dev/null)
    SZ=$(printf '%s' "$L" | wc -c | tr -d ' ')
    if [ -n "$PAT" ] && printf '%s' "$L" | grep -aq "$PAT"; then
        echo "  MATCHED '$PAT' at t+$(($(date +%s)-T2))s"
        printf '%s' "$L" | grep -a "$PAT" | tail -3
        break
    fi
    if [ "$SZ" = "$LAST" ]; then
        STILL=$((STILL+3))
        [ "$STILL" -ge 30 ] && { echo "  settled (log static 30 s) at t+$(($(date +%s)-T2))s"; break; }
    else
        STILL=0; LAST=$SZ
    fi
    sleep 3
done
echo "== total $(($(date +%s)-T0))s =="
