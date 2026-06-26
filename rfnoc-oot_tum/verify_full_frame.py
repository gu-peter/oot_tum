#!/usr/bin/env python3
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""End-to-end on-hardware verification of the ofdm_tx_sl block output.

The radio digital-loopback path (transceiver.py) can only capture ~150 of the
240 OFDM symbols before the host RX overflows, so it can never reach the tail of
the frame. This script instead reads the block's modulated output (s_txData)
directly at the block boundary via the on-chip ILA, in a sweep of windows
positioned across the whole frame, and compares every symbol to the MATLAB
golden reference -- giving deterministic coverage of all 240 symbols.

For each window it:
  1. arms the ILA (Vivado batch + tools/ila_cap_symk.tcl) to trigger near a
     chosen frame symbol and store 8192 consecutive output beats,
  2. feeds the golden payload over UHD (reusing transceiver.py) to produce a
     frame so the trigger fires,
  3. aligns the captured window to the golden frame by a direct offset search
     (FFT correlation is unreliable here -- OFDM frames are self-similar),
  4. checks every I/Q sample against golden within TOL_LSB.

The union of the windows must cover all 240 symbols with zero out-of-tolerance
samples for a PASS.

Run (system Python + UHD, same env as transceiver.py):
    export UHD_MODULE_PATH=.../build/lib/librfnoc-oot_tum.so
    PYTHONPATH=/usr/lib/python3/dist-packages:.../build/python \\
    LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu \\
    /usr/bin/python3 verify_full_frame.py
"""
import os
import subprocess
import sys
import time

import numpy as np

import transceiver as tx

VIVADO = os.environ.get("VIVADO_BIN", "/tools/Xilinx/Vivado/2021.1/bin/vivado")
HERE = os.path.dirname(os.path.abspath(__file__))
TCL = os.path.join(HERE, "tools", "ila_cap_symk.tcl")
LTX = os.path.join(
    HERE, "build", "build-x440_X4_200_rfnoc_image_core_custom_ofdm", "x4xx.ltx")
CSV_DIR = os.environ.get("ILA_CSV_DIR", "/tmp")

# sym_mirror trigger values. 0 uses the run_frame trigger (frame start). The
# block's output lags sym_mirror by ~3.5 symbols (FFT/CP/windowing pipeline),
# and each window spans ~25 symbols, so two triggers cover all 28 symbols. With
# only 28 symbols/frame (8960 samples) the whole frame now also fits in a single
# transport capture, so this ILA sweep is mostly a redundant cross-check.
SYM_TRIGGERS = [0, 3]

SPS = tx.SAMP_PER_SYM      # 320
N_RX = tx.NUM_RX_SAMPS     # 8960
N_SYM = N_RX // SPS        # 28
TOL = tx.TOL_LSB


def split_iq(words):
    w = np.asarray(words, dtype=np.uint32)
    i = ((w >> 16) & 0xFFFF).astype(np.int16).astype(np.int32)
    q = (w & 0xFFFF).astype(np.int16).astype(np.int32)
    return i, q


def load_window(csv_path):
    """Return the valid s_txData output words (uint32) from an ILA CSV."""
    import csv as _csv
    with open(csv_path) as f:
        rows = list(_csv.reader(f))
    hdr = rows[0]

    def col(name):
        return next(i for i, h in enumerate(hdr) if name in h)
    ci_d, ci_v, ci_r = (col("s_txData_axis_tdata"),
                        col("s_txData_axis_tvalid"),
                        col("s_txData_axis_tready"))
    data = rows[2:]
    if not data:
        return np.array([], dtype=np.uint32)
    sd = np.array([int(r[ci_d], 16) for r in data], dtype=np.uint32)
    sv = np.array([int(r[ci_v], 16) for r in data], dtype=np.uint8)
    sr = np.array([int(r[ci_r], 16) for r in data], dtype=np.uint8)
    return sd[(sv & sr) == 1]


def best_offset(io, qo, ie, qe, probe=512):
    """Find the golden-frame offset that best matches the window's first
    `probe` samples (direct L1 search; robust to OFDM self-similarity)."""
    n = min(probe, len(io))
    err = np.array([
        np.sum(np.abs(io[:n] - ie[o:o + n])) + np.sum(np.abs(qo[:n] - qe[o:o + n]))
        for o in range(0, N_RX - n)
    ])
    return int(np.argmin(err))


def capture_window(symk, feed_fn):
    """Arm the ILA for trigger `symk`, drive frames, return the CSV path."""
    csv_path = os.path.join(CSV_DIR, f"ila_sym_{symk}.csv")
    log_path = os.path.join(CSV_DIR, f"ila_sym_{symk}.log")
    for p in (csv_path, log_path):
        try:
            os.remove(p)
        except FileNotFoundError:
            pass
    env = dict(os.environ, SYMK=str(symk), ILA_CSV=csv_path, ILA_LTX=LTX)
    with open(log_path, "w") as log:
        proc = subprocess.Popen(
            [VIVADO, "-mode", "batch", "-nojournal", "-nolog", "-source", TCL],
            stdout=log, stderr=subprocess.STDOUT, env=env)
        # Wait until the ILA is armed before producing frames.
        for _ in range(120):
            if proc.poll() is not None:
                break
            if "ILA_ARMED" in open(log_path).read():
                break
            time.sleep(1)
        else:
            pass
        # Drive a few frames so the trigger fires while armed.
        for _ in range(3):
            if proc.poll() is not None:
                break
            feed_fn()
        proc.wait(timeout=180)
    if "WROTE_CSV" not in open(log_path).read():
        raise RuntimeError(f"ILA capture for sym {symk} failed; see {log_path}")
    return csv_path


def main():
    expected = tx.load_expected_output(tx.EXPECTED_HEX)
    ie, qe = split_iq(expected)

    graph, txs, rxs, ofdm, radio = tx.build_graph()
    golden = tx.load_input_payload(tx.INPUT_HEX)

    def feed_fn():
        tx.feed_enable_capture(txs, rxs, ofdm, radio, golden.copy())

    covered = np.zeros(N_SYM, dtype=bool)
    worst = 0
    total_fail = 0
    total_chk = 0
    print(f"Sweeping {len(SYM_TRIGGERS)} ILA windows over the {N_SYM}-symbol frame...\n")
    print(f"{'trig':>5} {'off':>6} {'sym':>7} {'samples':>8} {'maxdev':>7} {'nfail':>7}  symbols")
    for symk in SYM_TRIGGERS:
        csv_path = capture_window(symk, feed_fn)
        out = load_window(csv_path)
        if len(out) < 2000:
            print(f"{symk:>5} {'--':>6} {'--':>7} {len(out):>8}  (too few samples, skipped)")
            continue
        io, qo = split_iq(out)
        off = best_offset(io, qo, ie, qe)
        n = min(len(out), N_RX - off)
        di = np.abs(io[:n] - ie[off:off + n])
        dq = np.abs(qo[:n] - qe[off:off + n])
        nfail = int(np.sum((di > TOL) | (dq > TOL)))
        maxdev = int(max(di.max(), dq.max()))
        total_fail += nfail
        total_chk += n
        worst = max(worst, maxdev)
        s0, s1 = off // SPS, min((off + n + SPS - 1) // SPS, N_SYM)
        covered[s0:s1] = True
        print(f"{symk:>5} {off:>6} {off / SPS:>7.1f} {n:>8} {maxdev:>7} {nfail:>7}  {s0}..{s1 - 1}")

    miss = np.where(~covered)[0]
    print(f"\nChecked {total_chk} samples across windows; "
          f"{total_fail} out of tolerance (>{TOL} LSB); worst deviation {worst} LSB.")
    print(f"Symbol coverage: {covered.sum()}/{N_SYM}")
    if total_fail == 0 and len(miss) == 0:
        print(f"PASS: all {N_SYM} symbols verified bit-exact against the MATLAB golden frame.")
        sys.exit(0)
    if len(miss):
        print(f"INCOMPLETE: symbols not covered: {miss.tolist()}")
    if total_fail:
        print(f"FAIL: {total_fail} samples exceeded {TOL} LSB.")
    sys.exit(1)


if __name__ == "__main__":
    main()
