#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Feed the golden payload through the live graph a few times so an armed ILA
catches a frame. No capture analysis -- just exercise run_frame/payload reads."""
import os
import sys
import time

os.environ.setdefault(
    "UHD_MODULE_PATH",
    "/home/peter/git/oot_tum/rfnoc-oot_tum/build/lib/librfnoc-oot_tum.so",
)

import transceiver as tx  # noqa: E402

N_REPEAT = int(sys.argv[1]) if len(sys.argv) > 1 else 4


def main():
    graph, tx_streamer, rx_streamer, ofdm, radio = tx.build_graph()
    golden = tx.load_input_payload(tx.INPUT_HEX)
    for i in range(N_REPEAT):
        print(f"\n==== golden feed {i + 1}/{N_REPEAT} ====", flush=True)
        tx.feed_enable_capture(tx_streamer, rx_streamer, ofdm, radio, golden.copy())
        time.sleep(0.5)


if __name__ == "__main__":
    main()
