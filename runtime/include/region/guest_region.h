#pragma once
// Guest-address regioning.
//
// The runtime was written against the PAL executable (RMCP01), and every guest address it names -
// the entry points its native overrides replace, the SDK globals its HLE reads and writes, the
// callback and range constants - is spelled as that PAL address. Those PAL addresses are kept
// exactly as written and treated as the *identity* of the thing they name; a per-region table
// then maps each identity to the address it has in the executable actually being built.
//
// Mechanically: a bare 8-digit hex token such as 801A9E84 goes through MKW_REGION_TOKEN, which
// pastes it onto MKW_G_ and expands to the region's token (801A9DE4 for NTSC-U, the same token for
// PAL). Two extra macro levels let the expanded token be pasted again into func_/0x forms. The
// translator applies the same table (see tools/region/gen_region_headers.py, which writes both
// this header's region files and keeps them in the form the translator's source scan reads).
//
// Any PAL address the region header does not define fails to compile ("use of undeclared
// identifier 'MKW_G_xxxxxxxx'"): a region table is complete or the build does not exist.

#if defined(MKW_GUEST_REGION_HEADER)
#include MKW_GUEST_REGION_HEADER
#else
#include "region/rmcp01.h"
#endif

#define MKW_REGION_TOKEN(pal_hex) MKW_G_##pal_hex

// 0x<region address>u from a PAL identity token, usable wherever an integer literal is.
#define MKW_GADDR(pal_hex) MKW_GADDR_I(MKW_REGION_TOKEN(pal_hex))
#define MKW_GADDR_I(tok) MKW_GADDR_II(tok)
#define MKW_GADDR_II(tok) 0x##tok##u

#ifndef MKW_REGION_GAME_ID
#error "The guest region header must define MKW_REGION_GAME_ID and the other MKW_REGION_* facts"
#endif
