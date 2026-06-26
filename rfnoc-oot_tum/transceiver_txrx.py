#!/usr/bin/env python3
#
# Copyright 2026 Ettus Research, a National Instruments Brand
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""End-to-end hardware verification for the COMBINED ofdm_tx_sl + ofdm_rx_sl image.

This is the on-hardware test for the x440_X4_200_rfnoc_image_core_custom_ofdm_txrx
image core, which places BOTH OFDM blocks on RF A:0:

    Host payload (dibits) -> Ofdm_tx_sl#0 -> Radio#0 TX --+
                                                          | digital loopback (poke 0x1000)
    Host payload (dibits) <- Ofdm_rx_sl#0 <- Radio#0 RX <-+

So a single host streamer pair drives the WHOLE modem in the FPGA: the transmit
block modulates the payload into an OFDM waveform, the radio's digital loopback
carries that waveform straight into the receive block, and the receive block
synchronizes on it, demodulates, and hands the recovered payload bits back to the
host. A pass means both the TX and RX blocks work together end to end --
synchronization (preamble/timing/frequency lock) AND bit reconstruction.

This combines transceiver.py (drove ofdm_tx, captured the radio waveform) and
transceiver_rx.py (fed a golden waveform to the radio, checked ofdm_rx's bits):
here neither golden waveform nor a separate stimulus is needed -- ofdm_tx *is* the
waveform source for ofdm_rx.

Golden reference:
    ofdm_tx_input.hex  -- the QPSK dibits (0..3) we feed into ofdm_tx AND expect
                          ofdm_rx to recover. One frame = NUM_TX_SAMPS dibits.

Why we feed CONTINUOUSLY: the receiver must see a lead-in plus a few waveform
repeats to synchronize (it spends the first repeat(s) locking, then demodulates
cleanly). ofdm_tx's input FIFO only holds ~one frame, so we keep feeding whole
frames back-to-back in one open burst (a daemon thread) -- the block then emits a
continuous periodic waveform and the receiver locks and free-runs on it. The
~7% per-symbol duty-cycle bubbles ofdm_tx leaves between samples are filled by the
radio TX with underflow zeros; whether the receiver tolerates that regular gap
pattern is exactly what this end-to-end test exercises (peakInfo below reports the
sync result).

Run with (UHD must be able to find the OOT block controllers):
    export UHD_MODULE_PATH=.../librfnoc-oot-tum.so
    python3 transceiver_txrx.py
"""

import os
import sys
import threading
import time

import numpy as np
import uhd

from rfnoc_oot_tum import OfdmTxSlBlockControl, OfdmRxSlBlockControl


# --- Configuration ---
DEVICE_ARGS = ("addr=10.157.161.243, master_clock_rate=250e6, "
               "num_recv_frames=4096, recv_frame_size=1472, recv_buff_size=50000000, "
               "num_send_frames=4096, send_frame_size=8000, send_buff_size=50000000")
RATE = 250e6  # radio sample rate; == master_clock_rate so the DUC/DDC ratio is 1

_HERE = os.path.dirname(os.path.abspath(__file__))
PAYLOAD_HEX = os.path.join(_HERE, "ofdm_tx_input.hex")   # QPSK dibits 0..3 (sent + expected)

NUM_TX_SAMPS = 4560   # QPSK dibits in one frame (20 data syms * 228)
NUM_RX_SAMPS = 8960   # sc16 samples in one OFDM frame waveform (28 syms * 320)
DIBITS_PER_WORD = 16  # dibits packed per 32-bit payload item
WORDS_PER_FRAME = (NUM_TX_SAMPS + DIBITS_PER_WORD - 1) // DIBITS_PER_WORD  # 285

# OFDM frame geometry (for prints only).
FFT_LEN = 256
CP_LEN = 64
N_SYMS = 28
SAMP_PER_SYM = FFT_LEN + CP_LEN  # 320

# How many frames' worth of recovered payload to drain. The receiver needs the
# first repeat(s) to lock, so capture several frames and pick the best-aligned one.
N_CAP_FRAMES = 8
SPP_TX = 1024   # payload words per packet into ofdm_tx
SPP_RX = 1024   # payload words per packet out of ofdm_rx

SHOW_PLOTS = os.environ.get("SHOW_PLOTS", "1") != "0"


# ---------------------------------------------------------------------------
# Payload pack/unpack -- identical to transceiver.py / transceiver_rx.py so the
# TX feed and RX recovery use the same 16-dibit/word + sc16 half-swap convention.
# ---------------------------------------------------------------------------
def load_dibits(path):
    """Load the golden payload dibits (one hex byte 0..3 per line)."""
    return (np.loadtxt(path, dtype=np.uint8,
                       converters={0: lambda s: int(s, 16)}).reshape(-1) & 3)


def half_swap(words):
    """Swap the two 16-bit halves of each 32-bit word (sc16 CPU <-> item32 wire)."""
    w = np.asarray(words, dtype=np.uint32)
    return (((w & 0x0000FFFF) << 16) | ((w >> 16) & 0x0000FFFF)).astype(np.uint32)


def pack_dibits_to_words(payload):
    """Pack per-dibit payload (0..3) into the 32-bit sc16 items ofdm_tx consumes.

    16 dibits/word (dibit p in bits [2*p +: 2]), then pre-swap the 16-bit halves
    to cancel UHD's sc16 (I<<16)|Q packing. Mirrors transceiver.py exactly.
    """
    d = (np.asarray(payload).reshape(-1) & 3).astype(np.uint32)
    if len(d) % NUM_TX_SAMPS != 0:
        raise ValueError(f"payload length {len(d)} not a multiple of one frame "
                         f"({NUM_TX_SAMPS})")
    words = []
    for f in range(len(d) // NUM_TX_SAMPS):
        frame = d[f * NUM_TX_SAMPS:(f + 1) * NUM_TX_SAMPS]
        pad = (-len(frame)) % DIBITS_PER_WORD
        frame = np.concatenate([frame, np.zeros(pad, dtype=np.uint32)])
        cols = frame.reshape(-1, DIBITS_PER_WORD)
        w = np.zeros(cols.shape[0], dtype=np.uint32)
        for p in range(DIBITS_PER_WORD):
            w |= (cols[:, p] & 3) << (2 * p)
        words.append(w)
    out = np.concatenate(words).astype(np.uint32)
    return ((out & 0x0000FFFF) << 16) | ((out >> 16) & 0x0000FFFF)


def unpack_words_to_dibits(words):
    """Inverse of pack: un-swap sc16 halves, then extract dibit p from bits [2*p +: 2]."""
    w = half_swap(words)
    d = np.empty((len(w), DIBITS_PER_WORD), dtype=np.uint8)
    for p in range(DIBITS_PER_WORD):
        d[:, p] = (w >> (2 * p)) & 3
    return d.reshape(-1)


def dibits_to_bits(d):
    """Dibit array (0..3) -> message bits (bit0 = dibit&1, bit1 = dibit>>1)."""
    d = np.asarray(d, dtype=np.uint8)
    out = np.empty(2 * len(d), dtype=np.uint8)
    out[0::2] = d & 1
    out[1::2] = (d >> 1) & 1
    return out


def transform_dibits(d, xor_c, swap):
    """Apply one of the 8 QPSK-ambiguity transforms (rotation / I-Q swap)."""
    d = np.asarray(d, dtype=np.uint8)
    b0 = d & 1
    b1 = (d >> 1) & 1
    if swap:
        b0, b1 = b1, b0
    b0 = b0 ^ (xor_c & 1)
    b1 = b1 ^ ((xor_c >> 1) & 1)
    return (b0 | (b1 << 1)).astype(np.uint8)


def align_and_score(recovered, golden):
    """Slide the golden frame across the recovered dibit stream over all 8 QPSK
    ambiguity transforms; return (offset, xor_c, swap, n_bit_err, n_bits) of best."""
    L = len(golden)
    gbits = dibits_to_bits(golden)
    n_bits = len(gbits)
    if len(recovered) < L:
        return None
    best = None
    for off in range(0, len(recovered) - L + 1):
        seg = recovered[off:off + L]
        for xor_c in range(4):
            for swap in (False, True):
                t = transform_dibits(seg, xor_c, swap)
                err = int(np.count_nonzero(dibits_to_bits(t) != gbits))
                if best is None or err < best[3]:
                    best = (off, xor_c, swap, err, n_bits)
                    if err == 0:
                        return best
    return best


# ---------------------------------------------------------------------------
# Graph
# ---------------------------------------------------------------------------
def build_graph():
    """tx_streamer -> ofdm_tx -> radio0 TX -(loopback)-> radio0 RX -> ofdm_rx -> rx_streamer."""
    graph = uhd.rfnoc.RfnocGraph(DEVICE_ARGS)

    sa_tx = uhd.usrp.StreamArgs("sc16", "sc16")
    sa_tx.args = f"spp={SPP_TX}"
    tx_streamer = graph.create_tx_streamer(1, sa_tx)

    sa_rx = uhd.usrp.StreamArgs("sc16", "sc16")
    sa_rx.args = f"spp={SPP_RX}"
    rx_streamer = graph.create_rx_streamer(1, sa_rx)

    ofdm_tx = OfdmTxSlBlockControl(graph.get_block("0/Ofdm_tx_sl#0"))
    ofdm_rx = OfdmRxSlBlockControl(graph.get_block("0/Ofdm_rx_sl#0"))
    radio = uhd.rfnoc.RadioControl(graph.get_block("0/Radio#0"))

    # host -> ofdm_tx (input)
    graph.connect(tx_streamer, 0, ofdm_tx.get_unique_id(), 0)
    # ofdm_tx -> radio0 TX (static image hop). connect_through_blocks registers
    # the path so properties/actions propagate along it.
    uhd.rfnoc.connect_through_blocks(
        graph, ofdm_tx.get_unique_id(), 0, radio.get_unique_id(), 0)
    # radio0 RX -> ofdm_rx (static image hop). REQUIRED: without this the rx
    # streamer's stream command cannot reach the radio ("no neighbour found"), so
    # the radio RX never starts and the receiver sees nothing (peakInfo count=0).
    uhd.rfnoc.connect_through_blocks(
        graph, radio.get_unique_id(), 0, ofdm_rx.get_unique_id(), 0)
    # ofdm_rx -> host (output)
    graph.connect(ofdm_rx.get_unique_id(), 0, rx_streamer, 0)

    graph.commit()

    radio.set_rate(RATE)
    radio.set_properties(f"spp={SPP_RX}", 0)
    radio.poke32(0x1000, 0)  # loopback OFF for now

    return graph, tx_streamer, rx_streamer, ofdm_tx, ofdm_rx, radio


def _start_tx_feeder(tx_streamer, data_in, stop_evt):
    """Continuously stream the golden frame so ofdm_tx emits a periodic waveform.

    ofdm_tx's input FIFO holds ~one frame, so we send whole frames back-to-back in
    one open burst (SOB on the first, EOB only on stop); tx_streamer.send
    backpressures, self-pacing to the block's consumption rate. Daemon thread until
    stop_evt is set. Mirrors transceiver.py's _start_tx_feeder.
    """
    def loop():
        md = uhd.types.TXMetadata()
        md.has_time_spec = False
        first = True
        while not stop_evt.is_set():
            md.start_of_burst = first
            md.end_of_burst = False
            try:
                tx_streamer.send(data_in, md, timeout=1.0)
            except Exception:
                break
            first = False
        md.start_of_burst = False
        md.end_of_burst = True
        try:
            tx_streamer.send(data_in[:, :0], md, timeout=1.0)
        except Exception:
            pass

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t


def feed_and_capture(tx_streamer, rx_streamer, ofdm_tx, ofdm_rx, radio, payload):
    """Run the full on-chip modem and capture the recovered payload words.

    Sequence: arm the receiver, enable the radio loopback, buffer the first frame
    into ofdm_tx while it is disabled, start a continuous payload capture, then
    enable ofdm_tx and keep feeding frames so the receiver sees a continuous
    waveform to lock on. Returns the captured payload words (uint32, sc16 CPU layout).
    """
    data_in = pack_dibits_to_words(payload).reshape(1, -1).astype(np.uint32)
    cap_words = N_CAP_FRAMES * WORDS_PER_FRAME

    # Start TX disabled so the input FIFO fills (no pops) before we enable.
    ofdm_tx.set_enable(False)

    # Arm the receiver: synchronizer + frequency correction on.
    ofdm_rx.set_freq_correction_en(True)
    ofdm_rx.set_start(True)
    print(f"rx start={int(ofdm_rx.get_start())} "
          f"freqCorrectionEn={int(ofdm_rx.get_freq_correction_en())}")

    radio.poke32(0x1000, 1)  # digital loopback ON
    print(f"Loopback register readback: {radio.peek32(0x1000)} (expected 1)")

    # Continuous payload feeder (frame 1 buffers while disabled, rest stream as the
    # FIFO drains once enabled).
    stop_evt = threading.Event()
    feeder = _start_tx_feeder(tx_streamer, data_in, stop_evt)

    # Wait for the input FIFO to report full, like the TB's enabler.
    deadline = time.time() + 5.0
    while time.time() < deadline and ofdm_tx.get_tx_payload_ready():
        time.sleep(0.001)
    print("ofdm_tx input FIFO full (txPayloadReady low)"
          if not ofdm_tx.get_tx_payload_ready()
          else "FIFO did not report full within 5 s; proceeding anyway")

    # Start a continuous recovered-payload capture (the payload rate is low: one
    # word per ~16 sample cycles, data symbols only, so a continuous stream is far
    # from the radio-RX-FIFO ceiling). Continuous (not num_done) so the radio
    # free-runs the whole waveform -- a count would stop the radio in *sample*
    # units before a full preamble is seen. Mirrors transceiver_rx.py.
    rx_md = uhd.types.RXMetadata()
    raw_out = np.zeros((1, cap_words), dtype=np.uint32)
    stream_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    stream_cmd.stream_now = True
    rx_streamer.issue_stream_cmd(stream_cmd)

    # Enable the transmit block: it now emits its buffered frame and, as the feeder
    # keeps topping up the FIFO, a continuous periodic waveform -> the receiver
    # locks and demodulates.
    ofdm_tx.set_enable(True)
    print(f"ofdm_tx enable readback: {int(ofdm_tx.get_enable())} (expected 1)")

    got = 0
    while got < cap_words:
        n = rx_streamer.recv(raw_out[:, got:], rx_md, timeout=3.0)
        err = str(rx_md.error_code).lower()
        if "overflow" in err:
            print(f"Overflow after {got} payload words; keeping the clean prefix")
            break
        if "timeout" in err:
            print(f"recv timeout after {got} payload words (receiver idle?)")
            break
        if n == 0:
            break
        got += n

    stop_mode = getattr(uhd.types.StreamMode, "stop_cont",
                        getattr(uhd.types.StreamMode, "stop_continuous", None))
    if stop_mode is not None:
        rx_streamer.issue_stream_cmd(uhd.types.StreamCMD(stop_mode))

    # Tear down.
    ofdm_tx.set_enable(False)
    stop_evt.set()
    feeder.join(timeout=2.0)
    radio.poke32(0x1000, 0)
    ofdm_rx.set_start(False)

    # Receiver synchronizer diagnostics.
    print(f"peakInfo: count={ofdm_rx.get_peak_count()} "
          f"correlation={ofdm_rx.get_peak_correlation()} "
          f"threshold={ofdm_rx.get_peak_threshold()} "
          f"tOffset={ofdm_rx.get_peak_toffset()} "
          f"fOffset={ofdm_rx.get_peak_foffset()}")
    print(f"Captured {got}/{cap_words} payload words (~{got / WORDS_PER_FRAME:.1f} frames)")

    return raw_out[0, :got]


def plot_result(recovered, golden, best):
    """Overlay recovered vs golden payload bits and the bit-error map."""
    import matplotlib.pyplot as plt

    off, xor_c, swap, err, n_bits = best
    seg = transform_dibits(recovered[off:off + len(golden)], xor_c, swap)
    gbits = dibits_to_bits(golden)
    rbits = dibits_to_bits(seg)
    n_show = min(400, len(gbits))

    fig, axes = plt.subplots(2, 1, figsize=(14, 7))
    axes[0].step(np.arange(n_show), gbits[:n_show], where="mid", label="golden bits")
    axes[0].step(np.arange(n_show), rbits[:n_show] + 1.5, where="mid",
                 label="recovered bits (offset +1.5)")
    axes[0].set_title(f"ofdm_txrx end-to-end payload (offset={off}, xor={xor_c}, "
                      f"swap={swap}, BER={err / n_bits:.2e})")
    axes[0].legend(loc="upper right"); axes[0].grid(True)

    diff = (gbits != rbits).astype(int)
    axes[1].plot(diff, lw=0.5)
    axes[1].set_title(f"bit errors ({err}/{n_bits})")
    axes[1].set_xlabel("bit index"); axes[1].grid(True)

    fig.tight_layout()
    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass


def main():
    golden = load_dibits(PAYLOAD_HEX)[:NUM_TX_SAMPS]
    print(f"Loaded golden payload = {len(golden)} dibits (1 frame)")

    graph, tx_streamer, rx_streamer, ofdm_tx, ofdm_rx, radio = build_graph()

    captured = feed_and_capture(tx_streamer, rx_streamer, ofdm_tx, ofdm_rx, radio,
                                golden)
    np.save(os.path.join(_HERE, "captured_txrx_payload_raw.npy"), captured)

    if len(captured) < WORDS_PER_FRAME:
        print(f"FAIL: only {len(captured)} payload words captured "
              f"(< {WORDS_PER_FRAME} for one frame). Is the receiver synchronizing? "
              "Check peakInfo count above (0 => no preamble detected -- the ofdm_tx "
              "duty-cycle gaps in the looped-back waveform may be breaking sync).")
        sys.exit(1)

    recovered = unpack_words_to_dibits(captured)
    print(f"Recovered {len(recovered)} payload dibits "
          f"(~{len(recovered) / NUM_TX_SAMPS:.1f} frames)")

    best = align_and_score(recovered, golden)
    if best is None:
        print("FAIL: not enough recovered dibits to align one golden frame")
        sys.exit(1)

    off, xor_c, swap, err, n_bits = best
    ber = err / n_bits
    print(f"Best alignment: offset={off} dibits, ambiguity xor={xor_c} swap={swap}")
    print(f"BER = {err}/{n_bits} = {ber:.3e} over one frame ({NUM_TX_SAMPS} dibits)")

    passed = (err == 0)
    if passed:
        print("PASS: end-to-end OFDM TX->RX recovered the payload bits exactly "
              "(synchronization AND bit reconstruction both work on hardware)")
    elif ber < 1e-2:
        print(f"NEAR PASS: {err} bit errors (BER {ber:.2e}); likely residual "
              "frequency offset or a few edge symbols -- inspect peakInfo and the plot")
    else:
        print("FAIL: recovered payload does not match the golden payload. "
              "Check synchronization (peakInfo count/correlation vs threshold) -- a "
              "count of 0 means the receiver never locked, most likely because the "
              "ofdm_tx->radio duty-cycle bubbles punch gaps into the looped-back "
              "waveform that desync the comb-pilot frame grid.")

    if SHOW_PLOTS:
        try:
            plot_result(recovered, golden, best)
        except Exception as exc:
            print(f"(plot skipped: {exc})")

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
