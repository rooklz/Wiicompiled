/* wii_nand_defaults.h -- the files every real Wii NAND carries.
 *
 * These are not fabrications: they are the authentic default files Dolphin
 * copies into a fresh Wii root (Data/Sys/Wii/shared2/wc24/), byte for byte.
 * A title reads and VALIDATES them -- nwc24msg.cfg carries an additive
 * checksum over its first 0x3FC bytes, the mailbox control blocks carry
 * their own headers ('WcTf'/'WcFl'/'WcDl') -- so approximating them is the
 * "plausible lie" failure mode this project has been bitten by three times.
 * Answer with the real bytes or answer ENOENT; never invent structure.
 *
 * The Mii database is not a Dolphin copy -- Dolphin ships no RFL_DB.dat -- but
 * it is not a fabrication either: tools/mk_rfl_db.py generates it from the
 * format read out of the retail DOL's own RFL library (RFLiInitDB defines the
 * layout, RFLiCalculateCRC + the load-time CRC pass define validity), and the
 * generator re-runs those acceptance tests on its own output.  See that file.
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
