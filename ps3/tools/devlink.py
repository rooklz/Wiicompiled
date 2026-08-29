#!/usr/bin/env python3
"""devlink.py -- talk to the emulator running on the console.

  devlink.py log [seconds]     stream the live log
  devlink.py cmd <text>        send one command (mask <hex>, overlay, bt,
                               relaunch, quit) and print the reply
  devlink.py cycle             deploy-and-restart: relaunch, wait, reattach

The point of this is unattended development: with the emulator running, a
build can be deployed and restarted and its log read back without anyone
touching the console.
"""
import socket, sys, time

HOST, PORT = "192.168.1.123", 4000

def connect(timeout=6):
    """Attach to the emulator. A refused connection means it is not running:
    the console cannot be launched remotely (webMAN's play/launch endpoints and
    PS3MAPI were both tested and cannot start a title), so a cold start needs
    one action at the console. Everything after that is automated."""
    try:
        s = socket.create_connection((HOST, PORT), timeout=timeout)
    except (ConnectionRefusedError, OSError) as e:
        print("devlink: cannot attach (%s)." % e)
        print("The emulator is not running. Start it once on the console;")
        print("after that this tool can deploy, restart and read it remotely.")
        raise SystemExit(2)
    s.settimeout(2)
    return s

def stream(sec):
    s = connect(); t0 = time.time()
    try:
        while time.time() - t0 < sec:
            try:
                d = s.recv(4096)
                if not d: break
                sys.stdout.write(d.decode(errors="replace")); sys.stdout.flush()
            except socket.timeout:
                pass
    finally:
        s.close()

def cmd(text, listen=3):
    s = connect(); s.sendall((text + "\n").encode())
    t0 = time.time(); out = b""
    try:
        while time.time() - t0 < listen:
            try:
                d = s.recv(4096)
                if not d: break
                out += d
            except socket.timeout:
                break
    finally:
        s.close()
    sys.stdout.write(out.decode(errors="replace"))

if __name__ == "__main__":
    if len(sys.argv) < 2: print(__doc__); sys.exit(1)
    a = sys.argv[1]
    if   a == "log":   stream(float(sys.argv[2]) if len(sys.argv) > 2 else 20)
    elif a == "cmd":   cmd(" ".join(sys.argv[2:]))
    elif a == "cycle":
        cmd("relaunch", listen=1); time.sleep(25); stream(20)
    else: print(__doc__)
