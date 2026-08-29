/* wii_nand_defaults.h -- the standard files a Wii NAND carries under
 * /shared2, which a title reads and VALIDATES (nwc24msg.cfg carries an
 * additive checksum over its first 0x3FC bytes; the mailbox control blocks
 * carry 'WcTf'/'WcFl'/'WcDl' headers). Approximating them is the "plausible
 * lie" failure mode this project has been bitten by, so the rule is: answer
 * with the real bytes, from your own system's files, or answer ENOENT --
 * never invent structure. The Mii database (RFL_DB.dat) is validated the same
 * way, by its own CRC. You supply all of these from your own dump; none ship
 * here.
 *
 * X(nand_path, asset_relative_path, symbol_stem)
 *
 * symbol_stem feeds src/platform/ps3/main.c's registration macros, which spell
 * the blob symbols wc24_<stem>_blob / wc24_<stem>_blob_end.  The wc24_ prefix
 * is that table's ABI now, not a claim about what the file is.
 */
#define WII_NAND_DEFAULT_FILES(X)                                             \
    X("/shared2/wc24/misc.bin",           "shared2/wc24/misc.bin",       misc)\
    X("/shared2/wc24/nwc24dl.bin",        "shared2/wc24/nwc24dl.bin",    dl)  \
    X("/shared2/wc24/nwc24fl.bin",        "shared2/wc24/nwc24fl.bin",    fl)  \
    X("/shared2/wc24/nwc24fls.bin",       "shared2/wc24/nwc24fls.bin",   fls) \
    X("/shared2/wc24/nwc24msg.cbk",       "shared2/wc24/nwc24msg.cbk",   cbk) \
    X("/shared2/wc24/nwc24msg.cfg",       "shared2/wc24/nwc24msg.cfg",   cfg) \
    X("/shared2/wc24/mbox/wc24recv.ctl",  "shared2/wc24/mbox/wc24recv.ctl", rctl) \
    X("/shared2/wc24/mbox/wc24recv.mbx",  "shared2/wc24/mbox/wc24recv.mbx", rmbx) \
    X("/shared2/wc24/mbox/wc24send.ctl",  "shared2/wc24/mbox/wc24send.ctl", sctl) \
    X("/shared2/wc24/mbox/wc24send.mbx",  "shared2/wc24/mbox/wc24send.mbx", smbx) \
    X("/shared2/menu/FaceLib/RFL_DB.dat", "shared2/menu/FaceLib/RFL_DB.dat", rfldb)
