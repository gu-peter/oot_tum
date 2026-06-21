#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Feed golden twice, capture RX each time, and check whether the modulated
OUTPUT is deterministic run-to-run. Input dibits at the modulator were shown
(via ILA) to be lossless & deterministic, so this distinguishes:
  output deterministic   -> block fine; verification mapping was wrong.
  output non-deterministic-> downstream ce_250 metastability confirmed.
"""
import os, numpy as np
os.environ.setdefault("UHD_MODULE_PATH",
    "/home/peter/git/oot_tum/rfnoc-oot_tum/build/lib/librfnoc-oot_tum.so")
import transceiver as tx

def grab(streamers):
    g, txs, rxs, ofdm, radio = streamers
    golden = tx.load_input_payload(tx.INPUT_HEX)
    cap = tx.feed_enable_capture(txs, rxs, ofdm, radio, golden.copy())
    v = tx.reconstruct_valid(cap)
    return v

def align_to(ref, v):
    """Return v aligned so its sync matches ref (golden expected), via the
    payload-independent magnitude envelope."""
    off, _, lock = tx.align_by_magnitude(v, ref)
    return off, lock

def main():
    s = tx.build_graph()
    # capture twice
    vA = None; vB = None
    for attempt in range(8):
        v = grab(s)
        if vA is None: vA = v
        elif vB is None: vB = v; break
    expected = tx.load_expected_output(tx.EXPECTED_HEX)
    oA, lA = align_to(expected, vA)
    oB, lB = align_to(expected, vB)
    print(f"A: len={len(vA)} align_off={oA} lock={lA:.3f}")
    print(f"B: len={len(vB)} align_off={oB} lock={lB:.3f}")
    a = vA[oA:]; b = vB[oB:]
    n = min(len(a), len(b))
    ca = tx.to_complex(a[:n]).astype(np.complex64)
    cb = tx.to_complex(b[:n]).astype(np.complex64)
    # exact-equality of raw sc16 words over aligned region
    eq = np.mean(a[:n] == b[:n]) * 100
    # complex closeness (normalized)
    denom = np.maximum(np.abs(ca), 1)
    rel = np.abs(ca - cb) / np.maximum(np.abs(ca)+np.abs(cb), 1e-9)
    print(f"\naligned over {n} samples")
    print(f"raw sc16 word exact-equal A vs B: {eq:.2f}%")
    print(f"median complex rel-diff: {np.median(rel):.4f}  (0=identical)")
    # per data-symbol bit determinism using the validated demod
    np.save(os.path.join(tx._HERE,'det_vA.npy'), vA)
    np.save(os.path.join(tx._HERE,'det_vB.npy'), vB)
    print("saved det_vA.npy / det_vB.npy")

if __name__ == "__main__":
    main()
