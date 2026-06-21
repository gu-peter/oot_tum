#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Feed a structured block payload (0000..1111..2222..3333) repeatedly so an
armed ILA on m_txPayload can reveal whether the s8 CHDR transport preserves
byte order. block size = 2048 bytes."""
import os, sys, time, numpy as np
os.environ.setdefault("UHD_MODULE_PATH",
    "/home/peter/git/oot_tum/rfnoc-oot_tum/build/lib/librfnoc-oot_tum.so")
import transceiver as tx

BLK = 2048
def main():
    s = tx.build_graph(); g, txs, rxs, ofdm, radio = s
    n = tx.NUM_TX_SAMPS
    payload = ((np.arange(n) // BLK) % 4).astype(np.uint8)
    print("payload first/blockedges:", payload[0], payload[BLK-1], payload[BLK], payload[2*BLK])
    for i in range(int(sys.argv[1]) if len(sys.argv)>1 else 4):
        print(f"\n==== pattern feed {i+1} ====", flush=True)
        tx.feed_enable_capture(txs, rxs, ofdm, radio, payload.copy())
        time.sleep(0.4)

if __name__ == "__main__":
    main()
