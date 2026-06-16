#!/usr/bin/env python3
#
# Copyright 2026 Ettus Research, a National Instruments Brand
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""Probe ep0 + ofdm_tx — captures OFDM sc16 for MATLAB validation.

Topology (bitfile: usrp_x440_fpga_X4_200_ofdm_tx_only_ctrl):

    Host (s16) → ep0 → ofdm_tx0 → Radio#0 TX ──┐
                                                  │ digital loopback
    Host (sc16) ← ep0 ← Radio#0 RX ←────────────┘

OFDM parameters: FFT=256, CP=64, 240 symbols/frame → 76,800 sc16/frame.
Radio is 250 Msps (250 MHz clock).

Feeds ofdm_tx with N_CHUNKS=20 back-to-back bursts of TX_SAMPS=81,624 s16.
send() blocks on CHDR flow control when the EP TX FIFO is full, so the loop
throttles to ofdm_tx's consumption rate (~10 ms total). After the loop the
FIFO is full and all symbol types are populated; loopback is then enabled and
stream_now captures RX_SAMPS=32,768 sc16 of valid OFDM output.

Run with:
    export UHD_MODULE_PATH=/home/peter/git/rfnoc-oot-blocks/build/lib/librfnoc-oot-blocks.so  \\
        python3 probe_ofdm_tx_ep0.py
"""

import time
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import uhd

ADDR = "addr=10.157.161.243, master_clock_rate=250e6"
RATE = 250e6

FFT_LEN      = 256
CP_LEN       = 64
N_SYMS       = 240
SAMP_PER_SYM = FFT_LEN + CP_LEN        # 320

TX_SAMPS = 40_812  # s16 samples per feed chunk (2 bits packed per sample: bit[0]=I, bit[1]=Q)
N_CHUNKS = 20      # chunks to feed; send() blocks on flow control, keeping ofdm_tx fed
RX_SAMPS = 32_768  # sc16 to capture; EP buffer fits ~51840

SPP_TX   = 1024
SPP_RX   = 8192

SAVE_PATH = "/tmp/ofdm_tx_capture.mat"


def main():
    graph = uhd.rfnoc.RfnocGraph(ADDR)

    sa_tx = uhd.usrp.StreamArgs("s16", "s16")
    sa_tx.args = f"spp={SPP_TX}"
    tx_streamer = graph.create_tx_streamer(1, sa_tx)

    sa_rx = uhd.usrp.StreamArgs("sc16", "sc16")
    sa_rx.args = f"spp={SPP_RX}"
    rx_streamer = graph.create_rx_streamer(1, sa_rx)

    ofdm_tx = graph.get_block("0/Ofdm_tx#0")
    radio   = uhd.rfnoc.RadioControl(graph.get_block("0/Radio#0"))

    graph.connect(tx_streamer, 0, ofdm_tx.get_unique_id(), 0)
    uhd.rfnoc.connect_through_blocks(
        graph, ofdm_tx.get_unique_id(), 0, radio.get_unique_id(), 0)
    graph.connect(radio.get_unique_id(), 0, rx_streamer, 0)

    graph.commit()
    # Configure radio AFTER commit so rate propagates through the full graph
    # (DUC interpolation etc.).  Calling set_rate before commit can leave
    # intermediate blocks at the wrong ratio.
    radio.set_rate(RATE)
    radio.set_properties(f"spp={SPP_RX}", 0)
    radio.poke32(0x1000, 0)  # loopback OFF while feeding

    # Queue N_CHUNKS bursts. send() blocks on CHDR flow control once the EP TX
    # FIFO (524 kB = 262144 s16 samples) fills, so t_loop_ms >> 1 ms confirms
    # the FPGA is actively consuming data (ofdm_tx warmup counter advancing).
    rng  = np.random.default_rng(42)
    # Two QPSK bits are packed into each s16 sample: bit[0] = I, bit[1] = Q.
    # The RTL extracts txPayload[0] and txPayload[1] directly, so values 0-3 work.
    bit_i   = rng.integers(0, 2, TX_SAMPS, dtype=np.int16)  # QPSK I bits
    bit_q   = rng.integers(0, 2, TX_SAMPS, dtype=np.int16)  # QPSK Q bits
    data_in = (bit_i | (bit_q << 1)).reshape(1, TX_SAMPS)
    tx_md = uhd.types.TXMetadata()
    tx_md.has_time_spec = False
    t0 = time.time()
    for i in range(N_CHUNKS):
        tx_md.start_of_burst = (i == 0)
        tx_md.end_of_burst   = False
        num_tx = tx_streamer.send(data_in, tx_md, timeout=10.0)
        if i == 0:
            print(f"  chunk 0: send() returned {num_tx}/{TX_SAMPS} samples")
    t_loop_ms = (time.time() - t0) * 1e3
    # If t_loop_ms < 2 ms all data fit in the host DMA buffer without FPGA
    # flow-control — the FPGA might not yet be consuming (graph issue).
    print(f"Fed {N_CHUNKS}×{TX_SAMPS} s16 in {t_loop_ms:.1f} ms "
          f"({'flow-control active — FPGA consuming' if t_loop_ms > 2 else 'WARNING: no flow-control, check graph'})")

    # Enable loopback now so ofdm_tx output loops to RX from this point on.
    # Read back to confirm the register write reached the FPGA.
    radio.poke32(0x1000, 1)
    lb_rb = radio.peek32(0x1000)
    print(f"Loopback register readback: {lb_rb} (expected 1)")

    # Use a timed RX command 50 ms in the future so the capture window is
    # well inside the ofdm_tx steady-state output (warmup < 1 ms at 250 MHz).
    rx_time = radio.get_time_now().get_real_secs() + 0.050
    stream_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
    stream_cmd.stream_now = False
    stream_cmd.time_spec  = uhd.types.TimeSpec(rx_time)
    stream_cmd.num_samps  = RX_SAMPS
    rx_streamer.issue_stream_cmd(stream_cmd)

    rx_md   = uhd.types.RXMetadata()
    raw_out = np.zeros((1, RX_SAMPS), dtype=np.uint32)
    chunk   = np.zeros((1, SPP_RX),   dtype=np.uint32)
    got = 0
    while got < RX_SAMPS:
        n = rx_streamer.recv(chunk, rx_md, timeout=10.0)
        if n == 0:
            print(f"recv=0  error={rx_md.error_code}")
            break
        copy = min(n, RX_SAMPS - got)
        raw_out[0, got:got + copy] = chunk[0, :copy]
        got += copy

    radio.poke32(0x1000, 0)  # disable loopback
    # End the open TX burst cleanly.
    tx_md.start_of_burst = False
    tx_md.end_of_burst   = True
    tx_streamer.send(np.zeros((1, 1), dtype=np.int16), tx_md, timeout=5.0)
    print(f"Received {got}/{RX_SAMPS} sc16")
    print(f"Radio tick rate: {radio.get_tick_rate()/1e6:.3f} MHz")

    nz = int(np.count_nonzero(raw_out[0, :got]))
    print(f"Non-zero uint32 words: {nz}/{got}")
    if nz > 0:
        max_amp = int(np.max(np.abs(raw_out[0, :got].view(np.int16))))
        print(f"Max |int16| amplitude: {max_amp}")
    else:
        print("All samples exactly 0x00000000 — radio TX is idle/underflowing")

    i_s16 = raw_out[0, :got].view(np.int16)[0::2].copy()
    q_s16 = raw_out[0, :got].view(np.int16)[1::2].copy()

    # Burst analysis — ofdm_tx has a known ~25% duty cycle bug (symbolFormation.sv
    # 4× overhead): expect ~329 valid radio samples followed by ~983 zeros at 25% duty.
    nonzero_mask = (raw_out[0, :got] != 0)
    edges = np.where(np.diff(nonzero_mask.astype(np.int8)) != 0)[0] + 1
    if len(edges) >= 2:
        burst_starts = edges[::2] if nonzero_mask[0] else edges[1::2]
        burst_ends   = edges[1::2] if nonzero_mask[0] else edges[2::2]
        burst_lens   = burst_ends - burst_starts
        if len(burst_lens) > 1:
            periods = np.diff(burst_starts)
            print(f"Burst length  : {np.median(burst_lens):.0f} radio samples "
                  f"(expected ~{SAMP_PER_SYM} = FFT+CP per symbol)")
            print(f"Burst period  : {np.median(periods):.0f} radio samples "
                  f"(4× duty cycle → ~{4*SAMP_PER_SYM} expected)")
            print(f"Duty cycle    : {np.mean(burst_lens)/np.mean(periods)*100:.1f}% "
                  f"(expected ~25%)")
            # Extract just the valid bursts for saving
            burst_samples = np.concatenate([raw_out[0, s:e] for s, e in
                                            zip(burst_starts[:20], burst_ends[:20])])
        else:
            burst_samples = raw_out[0, :got]
    else:
        burst_samples = raw_out[0, :got]
    burst_iq = burst_samples.view(np.int16)
    burst_i  = burst_iq[0::2].astype(np.float32)
    burst_q  = burst_iq[1::2].astype(np.float32)

    scipy.io.savemat(SAVE_PATH, {
        "waveform":       i_s16.astype(np.float32) + 1j * q_s16.astype(np.float32),
        "waveform_burst": burst_i + 1j * burst_q,
        "i":              i_s16,
        "q":              q_s16,
        "rate":           float(RATE),
        "fft_len":        int(FFT_LEN),
        "cp_len":         int(CP_LEN),
        "n_syms":         int(N_SYMS),
        "samp_per_frame": int(N_SYMS * SAMP_PER_SYM),
        "samp_per_sym":   int(SAMP_PER_SYM),
    })
    print(f"Saved → {SAVE_PATH}  (waveform_burst = valid-only samples)")

    mag = np.sqrt(i_s16.astype(np.float32)**2 + q_s16.astype(np.float32)**2)

    fig, axes = plt.subplots(5, 1, figsize=(14, 14))
    ax1, ax2, ax3, ax4, ax5 = axes
    ax1.plot(data_in[0])
    ax1.set_ylabel("s16 (0-3, packed 2 bits/sample)")
    ax1.set_title(f"ofdm_tx input — {TX_SAMPS} samples × 2 bits = {2*TX_SAMPS} payload bits")
    ax1.grid(True)
    ax2.plot(i_s16)
    ax2.set_ylabel("I (sc16)")
    ax2.set_title(f"ofdm_tx output — {got} sc16 @ {RATE/1e6:.0f} Msps  "
                  f"({nz} non-zero / {got} = {nz/got*100:.1f}%)")
    ax2.grid(True)
    ax3.plot(q_s16)
    ax3.set_ylabel("Q (sc16)")
    ax3.grid(True)
    ax4.plot(mag)
    ax4.set_ylabel("|I+jQ|")
    ax4.set_xlabel("Sample index")
    ax4.grid(True)
    ax5.plot(burst_i[:min(3*SAMP_PER_SYM, len(burst_i))],
             burst_q[:min(3*SAMP_PER_SYM, len(burst_q))], ',', alpha=0.3)
    ax5.set_aspect('equal')
    ax5.set_xlabel("I")
    ax5.set_ylabel("Q")
    ax5.set_title("Constellation (first 3 symbols from valid burst)")
    ax5.grid(True)
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
