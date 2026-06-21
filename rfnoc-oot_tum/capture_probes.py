#!/usr/bin/env python3
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""Payload-delivery probe for the ofdm_tx_sl block on the CURRENT bitfile.

Feeds three payloads through the same digital-loopback path transceiver.py uses
-- a constant (all 3), a ramp (n%4), and the golden MATLAB payload -- captures
each, then demodulates the data symbols and checks the recovered QPSK bits
against what was fed. No golden expected file is needed for the check; the
demodulator (validated to decode the golden pair at 100%) recovers the bits the
hardware actually modulated.

Purpose (triage, point 2): the wrapper datapath + backpressure logic is now
sim-proven correct, so this isolates whether the hardware payload corruption is
data-dependent.

  * const stays 100% value-3  AND  golden is garbage  => data-dependent payload
    delivery bug (chase the ce_250 MMCM/CDC path or the real CHDR transport).
  * const is ALSO garbage                              => the whole payload path
    is broken on this bitfile.

Run on the device host (same env as transceiver.py):
    export UHD_MODULE_PATH=.../librfnoc-oot-blocks.so
    python3 capture_probes.py
"""
import os

import numpy as np

import transceiver as tx  # reuse the proven graph/feed/capture path

# Validated demod convention (see diagnose / TB test 6): drop CP, 256-pt FFT,
# data bins in fft-shifted order, b0 = Re<0, b1 = Im<0, byte = {b1,b0}.
FFT = tx.FFT_LEN          # 256
CP = tx.CP_LEN            # 64
SPS = tx.SAMP_PER_SYM     # 320
N_SYMS = tx.N_SYMS        # 240
DATA_BINS = np.r_[142:256, 0:114]
N_DATA = 228


def is_data(s1):
    """True for a payload-bearing data symbol (1-based index)."""
    return not (s1 == 1 or ((s1 - 2) % 4) == 0)


def demod_payload(valid, offset):
    """Recover the modulated payload bytes from an aligned valid stream.

    Returns the per-data-symbol recovered byte arrays (in golden symbol order
    starting at the capture's first symbol). Assumes the capture begins on a
    symbol boundary (offset % SPS == 0), as the single-frame feed produces.
    """
    # iq-swap variant: the radio loopback delivers sc16 words in the opposite
    # order to the testbench convention (established in diagnose_symbols).
    g = 1j * np.conj(tx.to_complex(valid)) / 32768.0
    s0 = (offset // SPS)              # golden symbol index of the first capture symbol
    n_cap = len(g) // SPS
    out = []                         # list of (golden_sym_1based, recovered_bytes)
    for s in range(n_cap):
        s1 = (s0 + s) % N_SYMS + 1
        if not is_data(s1):
            continue
        win = g[s * SPS + CP: s * SPS + CP + FFT]
        if len(win) < FFT:
            break
        X = np.fft.fft(win)[DATA_BINS]
        rec = (np.real(X) < 0).astype(np.uint8) | ((np.imag(X) < 0).astype(np.uint8) << 1)
        out.append((s1, rec))
    return s0, out


def expected_for_symbol(name, payload, golden_data_idx):
    """Expected 228 payload bytes for the j-th golden data symbol (0-based)."""
    seg = payload[golden_data_idx * N_DATA:(golden_data_idx + 1) * N_DATA]
    return seg


def analyze(name, payload, captured):
    valid = tx.reconstruct_valid(captured)
    if len(valid) < tx.MIN_VALID:
        print(f"[{name}] only {len(valid)} valid samples; cannot analyze")
        return
    # Align using the magnitude envelope (sync/ref are payload-independent, so
    # this locks for any payload). Golden frame is the reference grid.
    expected = tx.load_expected_output(tx.EXPECTED_HEX)
    offset, _, lock = tx.align_by_magnitude(valid, expected)
    s0, recs = demod_payload(valid, offset)
    if offset % SPS != 0:
        print(f"[{name}] WARNING: capture offset {offset} not symbol-aligned; "
              f"payload mapping may be shifted")

    # Count how many golden data symbols we have, and the per-symbol bit match.
    # golden_data_idx advances once per data symbol in golden order.
    matches = []
    gdi_base = sum(1 for s1 in range(1, s0 + 1) if is_data(s1))  # data syms before s0
    seen_data = 0
    for (s1, rec) in recs:
        gdi = gdi_base + seen_data
        exp = expected_for_symbol(name, payload, gdi)
        if len(exp) < N_DATA:
            break
        matches.append(np.mean(rec == (exp & 3)) * 100.0)
        seen_data += 1

    if not matches:
        print(f"[{name}] no full data symbols captured")
        return
    m = np.array(matches)
    # For const, also report fraction == 3 (offset/position independent).
    if name == "const":
        frac3 = np.array([np.mean(rec == 3) * 100.0 for _, rec in recs])
        print(f"[const ] offset={offset} lock={lock:.1f}  data syms={len(frac3)}  "
              f"value==3: mean {frac3.mean():.1f}%  min {frac3.min():.1f}%  "
              f"(100% everywhere => payload path delivers a constant correctly)")
    print(f"[{name:6s}] bit-match to fed payload: mean {m.mean():.1f}%  "
          f"min {m.min():.1f}%  over {len(m)} data syms  "
          f"(100% = correct, ~25% = random/garbage)")


def main():
    graph, tx_streamer, rx_streamer, ofdm, radio = tx.build_graph()

    golden = tx.load_input_payload(tx.INPUT_HEX)
    const = np.full(tx.NUM_TX_SAMPS, 3, dtype=np.uint8)
    ramp = (np.arange(tx.NUM_TX_SAMPS) % 4).astype(np.uint8)

    for name, payload in (("const", const), ("ramp", ramp), ("golden", golden)):
        print(f"\n==== feeding {name} payload ====")
        captured = tx.feed_enable_capture(
            tx_streamer, rx_streamer, ofdm, radio, payload.copy())
        np.save(os.path.join(tx._HERE, f"{name}_capture_new.npy"), captured)
        analyze(name, payload.astype(np.int64), captured)


if __name__ == "__main__":
    main()
