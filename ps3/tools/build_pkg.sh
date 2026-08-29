#!/bin/sh
# Serialised build with an HONEST exit status.
#
# Two `docker make` runs in the same tree corrupt each other's objects, so a
# lock serialises them. Just as important: `docker make | tail` reports TAIL's
# exit status, not make's -- a failed build looked successful, the && chain
# behind it deployed the PREVIOUS EBOOT, relaunched the console, and the run's
# results were read as if the new code were running. Capture to a file, then
# propagate make's own status.
set -e
cd "$(dirname "$0")/.."
exec 9> /tmp/wiicompiled-ps3-build.lock
if command -v flock >/dev/null 2>&1; then flock 9
else while ! mkdir /tmp/wiicompiled-ps3-build.lockdir 2>/dev/null; do sleep 5; done
     trap 'rmdir /tmp/wiicompiled-ps3-build.lockdir' EXIT; fi
LOG=/tmp/wiicompiled-ps3-build.log
if docker run --rm -m 3g -v "$(pwd)":/work -w /work ps3dev:latest \
       make -f ps3.mk pkg > "$LOG" 2>&1; then
    tail -3 "$LOG"
else
    rc=$?
    echo "BUILD FAILED (rc=$rc):"
    grep -E 'error|Error' "$LOG" | head -10
    exit "$rc"
fi
