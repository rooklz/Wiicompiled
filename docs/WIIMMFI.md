# Wiimmfi: how the Mario Kart Wii patch works, and the stage-1 integration

Research notes from decompiling Wiimm's Wiimmfi patch for Mario Kart Wii (Wiimmfi Patcher
v7.5, `wstrt patch --wiimmfi`, patch version 5) against the NTSC-U executables, and what it
takes to run the result as a static recompilation. Working files (decrypted blobs, disassembly,
the brainslug module analysis) live outside the tree in `../wiimmfi-research/`.

## 1. What the patcher changes

### 1.1 Static edits to `main.dol` / `StaticR.rel` (89 + 74 byte runs)

* Every `nintendowifi.net` host becomes `wiimmfi.de`; every `https://` becomes `http://`
  (the Wii's TLS is TLS 1.0, so Wiimmfi terminates plain HTTP for everything but one
  endpoint, see below). GameSpy hosts keep their names under the new domain:
  `gpcm.gs.wiimmfi.de`, `%s.sake.gs.wiimmfi.de`, `mariokartwii.race.gs.wiimmfi.de`.
* The NAS (Nintendo Authentication Server) URL table at `0x80276134` (three `/ac` URLs the DWC
  library indexes by "server type": test / release / dev) is repurposed:

  | index | original                                   | patched                              |
  |-------|--------------------------------------------|--------------------------------------|
  | 0     | `https://naswii.test.nintendowifi.net/ac`  | `http://ca.nas.wiimmfi.de/ca`        |
  | 1     | `https://naswii.nintendowifi.net/ac`       | `http://naswii.wiimmfi.de/ac`        |
  | 2     | `https://naswii.dev.nintendowifi.net/ac`   | `https://main.nas.wiimmfi.de/pe`     |

  The `/pr` (profanity) table gets the same treatment. `/ca` serves a CA certificate, `/ac`
  is the real login, `/pe` (`p` + region letter: pe/pj/pk/pp) serves a **code payload**.
* `DWCi_Auth_SendRequest+0x180` (`0x800ED7C8`): the `bl NHTTPAddHeaderField` that adds a
  second `Host:` header is NOPed. DWC adds `Host` by hand and NHTTP adds its own; nginx
  rejects duplicate `Host` headers, so Wiimmfi drops the DWC one. (Their brainslug module
  does the same at runtime; that is the "dup host header" logic in `wiimmfi.mod`.)
* `--all-ranks`: the SAKE rank filter strings in the REL (`%s >= %d and %s <= %d` ->
  `(%s >= %d or %s <= %d)`), i.e. the ghost/rankings search ignores the VR window.
* The BMG text overlay (`overlays/wiimmfi`) is a separate, optional `wstrt --patch-bmg`.

### 1.2 The injected section at `0x802C0000` (0x468 bytes) - a self-decrypting loader

The DOL entry point is moved to this section. It is a small stub (0x338 bytes of code and
tables) followed by 0x130 bytes of ciphertext. In order:

1. **Loader detection.** Compares `0x80001F70` / `0x8000191C` with the Gecko code-handler
   signature (`lis r3,0xD0; ori r3,r3,0xC0DE`) and `0x80001D6C` with two handler-variant
   words, records which kind was found (`r25`, `r26`), and if a handler is present adjusts a
   few of its words (byte offset list at `0x802C00D0`) and clears the hook flag at
   `0x80002774`. Whether `LR` was zero at entry (disc boot) or not (launched from a loader)
   is also recorded.
2. **Reserves a block at the top of the MEM1 arena.** `arenaHi = (arenaHi - 0x100 - 16) &
   ~31`, written back to `0x80003110` and the legacy `0x80000034`. A 16-byte info header goes
   at the block: `{0x0C076514, 0xDC5A99A5 | 0x4DCD01F1 (LR zero / non-zero), flags, 0}` with
   `flags = LR!=0 | handlerKind<<1 | loaderKind<<4 | 1<<6 | 5<<8` (5 = patch version).
3. **Decrypts** everything from `0x802C0338` to the end of the section with a chained stream:
   `w' = rotl(w,13) ^ key; key += w' >> 5`, initial key `0xDC020463` (stored in the stub).
4. **Copies the 0x100-byte code** (`0x802C0368..0x802C0468` decrypted) behind the info header.
5. **Applies the patch table** (`0x802C0338`, decrypted). The record types are Gecko code
   types: `04 addr value` = write32, `C2 addr codeOff retOff` = hook (writes `b block+codeOff`
   at `addr` and `b addr+4` into the block at `retOff`), `00` = end.
6. **Wipes itself** (zeroes the table, the ciphertext and its own code up to the wipe loop)
   and jumps to the original `__start`.

Decrypted, the table is exactly three records:

```
write32  0x800ECA0C  0x3BC00000     DWCi_Auth_InitInterface+0x20: mr r30,r3 -> li r30,0
hook     0x800EE300  -> block+0x00  DWCi_Auth_HandleResponse+0x174 (returns to +0x178)
hook     0x801D4E5C  -> block+0x50  NHTTPi_SocSSLConnect+0xB8      (returns to +0xBC)
```

### 1.3 What the relocated block does (the two hooks)

`DWCi_Auth_InitInterface(serverType)` normally stores its argument as the NAS server index;
the write32 forces index 0, i.e. the first request the game makes goes to `/ca`.

**Hook A** (`DWCi_Auth_HandleResponse`, right after `NHTTPGetBodyAll(req, &body)` returns the
body length in `r3`):

```
if (length > 0) {
    if (serverIndex == 2) {                 // 2nd response: the /pe body
        entry = (body + 3) & ~3;
        entry += *(u8*)entry;               // first byte = offset of the entry point
        dcbf entry; (*entry)();             // run the payload in the NHTTP callback context
    } else {                                // 1st response: the /ca body
        savedBody = body;  serverIndex = 2; // remember it, next request goes to /pe (HTTPS)
    }
}
r3 = -1;                                     // the SDK treats the response as failed and retries
```

**Hook B** (`NHTTPi_SocSSLConnect`, in place of `lwz r4, 0xC0(r28)` = the request's root-CA
pointer):

```
if (savedBody > 1) {
    NETSHA1Init / NETSHA1Update(savedBody, 0x554) / NETSHA1GetDigest
    req->rootCaLength = 0x554;                       // 1364-byte DER certificate
    r4 = (digest == 0fff1f07 00e638c9 49fbeffa 79022d3a 84ab134f) ? savedBody : 0;
} else r4 = 0;
-> SSLSetRootCA(ssl, r4, req->rootCaLength)
```

So the whole client-side patch is a **bootstrap**: fetch Wiimmfi's CA over plain HTTP, pin it
by SHA-1, fetch a per-region code payload over TLS from `main.nas.wiimmfi.de/pe`, execute it.
Everything Wiimmfi actually does in the game - the login extension, the anti-cheat, whatever
else - is in that server-supplied payload, and it can change without a new patcher. (Retro-WFC
/ WiiLink WFC use the same shape: a stage-1 in the mod and a downloaded payload, but theirs is
an open format the recompiler already translates statically.)

### 1.4 Server facts measured on 2026-08-29

* `main.nas.wiimmfi.de:443` speaks **TLS 1.0 and 1.1 only** (`ECDHE-RSA-AES256-SHA`,
  0xC014); a TLS 1.2 client hello gets a protocol-version alert. macOS's SecureTransport still
  negotiates it (verified with a small probe); Python/OpenSSL defaults do not.
* `/ca` and `/pe` answer `401 Unauthorized` to anything that is not the real client's request
  (a DWC-encoded `action=login` form with the right headers was not enough); `/ac` answers
  Wiimmfi's own `935` status with a UTF-16 error page for malformed logins. The payload can
  therefore only be captured by running the actual login from the game.
* The DWC NAS form encodes values in base64 with `*`, `?`, `>` for `+`, `/`, `=`.

### 1.5 The brainslug module (`wiimmfi.mod`, generic Wiimmfi patcher 0.7.5)

A relocatable ELF with full symbols (`Wiimmfi_start`, `WiimmfiOSLink`,
`game_id_to_dup_host_header_offset`, `patchCode1/2`, `patchCodeFix51420`, `newURL*`). It is
the generic runtime patcher used by loaders: it scans the loaded game for the
`nintendowifi.net` strings and the `https://naswii...` URLs and rewrites them in place, hooks
`OSLink` so RELs get the same treatment when they are linked, fixes the duplicate `Host`
header per game ID, and has per-game code patches (the `0x801D24F4/0x801D2544/0x801D25F8`
constants in `patchCode2` are the PAL NHTTP/SHA-1 addresses of the same login bootstrap as
above; `newURL3E/J/K/P` are the `/pe`-style payload URLs). Details:
`../wiimmfi-research/bslug-analysis.md`.

## 2. Integration: what a static recompilation can and cannot do

| piece | status |
|-------|--------|
| String/NOP edits | in the patched executables; translated like any other bytes |
| Injector stub | never needs to run: its only lasting effects are the block, the three patches and the lowered arena, all reproduced by `tools/wiimmfi/gen_stage1.py` as a Kamek chunk (`Assets/wiimmfi/stage1.kamek`) placed at `0x817F0400` by the `wiimmfi` profile |
| Hooks into the middle of `DWCi_Auth_HandleResponse` / `NHTTPi_SocSSLConnect` | the translator's Kamek path handles branch hooks with continuations (same machinery Retro Rewind uses) |
| `/ca` over HTTP, `/ac` over HTTP | the existing socket HLE |
| `/pe` over TLS 1.0/1.1 with a game-installed root CA | **new**: a SecureTransport backend for `/dev/net/ssl` on macOS (`runtime/src/hle/net/network_ssl.cpp`), honouring `SSL_SETROOTCA` |
| Executing the payload | **not possible statically**: it is code the server sends at login. Hook A's `(*entry)()` lands on an address the translator has no function for, and anything the payload patches into game code is invisible to already-translated functions |

The honest end state of the stage-1 build is therefore: the game logs in, downloads the CA
and the payload, and stops at the point where it would execute it. What that gives us is the
payload itself (logged by the runtime), which is the input for the two possible next steps:

1. **Snapshot and translate the payload** as a second Kamek-style overlay (its hooks into game
   code become translation-time patches, exactly like stage 1), pinned by hash and refreshed
   when Wiimmfi ships a new one. Faithful, and re-translation is mechanical, but it needs
   re-doing whenever the server payload changes, and the payload must be well-behaved (only
   branch/word patches, no code it generates at runtime).
2. **A PowerPC interpreter in the runtime** for downloaded code. General, but the payload's own
   hooks into game functions would still not take effect in translated code, so it only works
   if the payload never patches code - which an anti-cheat payload almost certainly does.

Option 1 is the workable one. Whether Wiimmfi would accept a client that is not a Wii or
Dolphin is a separate, non-technical question.

## 3. Runtime additions made for this

* `network_ssl.cpp`: SecureTransport TLS client (TLS 1.0-1.2, blocking callbacks over the
  game's socket, chain verified against the DER roots the game installs with `SETROOTCA`,
  unverified when it installs none - the Windows backend's policy).
* `keyboard_gamepad.cpp`: `[input] keyboard_gamepad = true` attaches the keyboard as a
  virtual SDL gamepad (WASD stick, arrows D-pad, IJKL C-stick, X/Z/C/V = A/B/X/Y, Q/E = L/R,
  Shift = Z, Enter = Start); `keyboard_script = "file"` replays a `<seconds> [key ...]`
  timeline for unattended runs.
* Product `Wiimmfi` (`runtime/src/product/wiimmfi_product.cpp`): the mod-overlay executable
  built from the `wiimmfi` profile; `build-macos.sh --profile wiimmfi`.

## 4. Building your own online play (notes)

What the game needs from a server is the GameSpy-era stack DWC talks to: NAS (`/ac` login,
`/pr` profanity), GPCM/GPSP (presence, friends, search), the QR2 master server for room
listing, NATNEG for UDP hole punching, SAKE (ghost/ranking storage) and the `race.gs` web
services. All of it is plain HTTP / simple TCP-UDP protocols and all of it exists as open
source: WiiLink's `wfc-server` (Go; Retro-WFC is a deployment of it) and the older
`dwc_network_server_emulator` (Python). The client side of a native recompilation is where
the room to improve is:

* the transport is already native (sockets, and now TLS) - the server can be anything that
  speaks the protocol, on any port, with real certificates;
* NAT traversal can be replaced by a relay or a modern hole-punching service without touching
  the game's own RACE packet protocol (which stays bit-exact, as it must to race real Wiis);
* console identity (`userid`/`passwd`/`macadr`/`csnum`) is synthesised by the runtime, so a
  private server can use proper accounts instead of console certificates;
* anti-cheat belongs on the server (WWFC validates what clients report); the client is already
  a known, unmodified translation of the game, which is more than a console can promise.

Wiimmfi's design - a signed, server-updated payload that the client executes blindly - is the
one thing not worth copying: it is exactly what makes it impossible to run on anything but a
real PowerPC.
