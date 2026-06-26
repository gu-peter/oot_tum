#!/usr/bin/env python3
#
# Copyright 2026 Ettus Research, a National Instruments Brand
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""Hardware verification for the ofdm_rx_sl RFNoC block.

This is the receive-side counterpart of transceiver.py. Where the TX script
fed a golden payload into ofdm_tx_sl and checked the modulated waveform, this
script feeds a golden OFDM *waveform* into the radio, lets the radio's digital
loopback carry it into ofdm_rx_sl, and checks that the recovered payload bits
equal the original golden payload.

Golden references (the TX vectors, used in reverse):

  * RX input waveform  = ofdm_tx_expected.hex  (the modulator's golden output,
                         sc16 item32 words {I[31:16], Q[15:0]})
  * RX expected payload = ofdm_tx_input.hex    (the original QPSK dibits, 0..3)

Datapath (radio digital loopback):

    Host (sc16 waveform) -> Radio#0 TX --+
                                         | digital loopback (poke 0x1000)
    Host (payload bits) <- Ofdm_rx_sl#0 <- Radio#0 RX <-+

The OFDM receiver must first SYNCHRONIZE on the waveform (find the preamble /
timing, estimate the frequency offset) before it can demodulate the data
symbols, so it needs to see a full frame with some lead-in. We therefore
transmit the golden waveform tiled several times back-to-back: the receiver
locks during the first repeat(s) and emits clean payload for the later, fully
synchronized frame(s). (This is the "capture roughly twice the waveform length
to synchronize" the block needs.)

The recovered payload comes out packed 16 QPSK dibits per 32-bit item (dibit p
in bits [2*p +: 2]) -- exactly the packing ofdm_tx_sl consumes -- so one frame
is 4560 dibits = 285 words. We unpack, slide the golden 4560-dibit frame across
the recovered stream to find the frame alignment, resolve the QPSK phase
ambiguity (a 90/180/270 deg rotation or I/Q conjugation permutes the dibit
labels), and report the bit error rate.

Run with (UHD must be able to find the OOT block controller):
    export UHD_MODULE_PATH=.../librfnoc-oot-tum.so
    python3 transceiver_rx.py
"""

import os
import sys
import time

import numpy as np
import uhd

from rfnoc_oot_tum import OfdmRxSlBlockControl


# --- Configuration ---
DEVICE_ARGS = ("addr=10.157.161.243, master_clock_rate=250e6, "
               "num_recv_frames=4096, recv_frame_size=1472, recv_buff_size=50000000, "
               "num_send_frames=4096, send_frame_size=8000, send_buff_size=50000000")
RATE = 250e6  # radio sample rate; == master_clock_rate so the DUC/DDC ratio is 1

_HERE = os.path.dirname(os.path.abspath(__file__))
# The RX golden references are the TX vectors used in reverse (see module docstring).
WAVEFORM_HEX = os.path.join(_HERE, "ofdm_tx_expected.hex")  # sc16 item32 words, RX input
PAYLOAD_HEX = os.path.join(_HERE, "ofdm_tx_input.hex")      # QPSK dibits 0..3, expected output

NUM_TX_SAMPS = 4560   # QPSK dibits in one frame (20 data syms * 228)
NUM_RX_SAMPS = 8960   # sc16 samples in one OFDM frame waveform (28 syms * 320)
DIBITS_PER_WORD = 16  # dibits packed per 32-bit payload item
WORDS_PER_FRAME = NUM_TX_SAMPS // DIBITS_PER_WORD  # 285 packed payload words/frame

# How many times to transmit the golden waveform back-to-back. The receiver
# spends the first repeat(s) synchronizing; later repeats demodulate cleanly.
N_WAVE_REPEAT = 6

# Zero lead-in prepended before the first waveform repeat. The block's front-end
# warm-up gate holds the core's 'start' low for WARMUP_SAMPS (1024) valid samples
# and the 256-tap sync correlator / energy moving-average need to fill before the
# first preamble arrives -- the HDL is designed around "a 2048 pre-roll" (see the
# wrapper's WARMUP_SAMPS comment and the testbench's GUARD_SAMPS=2048). Without
# this lead-in the first preamble lands during warm-up with the windows full of
# signal, so the receiver locks late/misaligned, the free-running comb-pilot grid
# reads the wrong symbols as channel references, and the equalizer collapses to
# ~0 (recovered payload comes out almost all-zero dibits). Match the TB's 2048.
GUARD_SAMPS = 2048

# --garbage mode: embed the waveform in random "garbage" noise to test detection
# (radio TB test 5). Stream = GARBAGE_PRE noise | waveform x N_EMBED | GARBAGE_POST
# noise. The noise has no cyclic-prefix repetition, so the synchronizer's CP
# correlation stays below threshold on it (no false lock) and fires only on the
# embedded preamble(s). GARBAGE_BITS=9 -> signed I/Q in +-256, ~30 dB below the
# signal RMS (~8000) -- a realistic capture noise floor that does not perturb the
# fixed-gain (0.875) front end. Mirrors the testbench's garbage stimulus.
GARBAGE_PRE = 4096
GARBAGE_POST = 8960
N_EMBED = 4
GARBAGE_BITS = 9

# OFDM frame geometry (for prints only).
FFT_LEN = 256
CP_LEN = 64
N_SYMS = 28
SAMP_PER_SYM = FFT_LEN + CP_LEN  # 320

SPP_TX = 8192   # waveform samples per packet to the radio TX
SPP_RX = 1024   # payload words per packet from the ofdm_rx block

SHOW_PLOTS = os.environ.get("SHOW_PLOTS", "1") != "0"


def load_words(path):
    """Load an "%08X"-per-line uint32 file (sc16 item32 words)."""
    return np.loadtxt(path, dtype=np.uint32,
                      converters={0: lambda s: int(s, 16)}).reshape(-1)


def load_dibits(path):
    """Load the golden payload dibits (one hex byte 0..3 per line)."""
    return (np.loadtxt(path, dtype=np.uint8,
                       converters={0: lambda s: int(s, 16)}).reshape(-1) & 3)


def half_swap(words):
    """Swap the two 16-bit halves of each 32-bit word.

    This is an involution that converts between UHD's sc16 CPU layout
    ({real=I low, imag=Q high}) and the wire/item32 layout ({I high, Q low}).
    Used on TX to make the wire carry the intended item32 word, and on RX to
    undo the same swap on captured words. Identical to transceiver.py's
    sc16_cpu_to_item32 / the pack_dibits_to_words pre-swap.
    """
    w = np.asarray(words, dtype=np.uint32)
    return (((w & 0x0000FFFF) << 16) | ((w >> 16) & 0x0000FFFF)).astype(np.uint32)


def split_iq(words):
    """sc16-as-uint32 (item32 {I high, Q low}) -> signed int32 (I, Q)."""
    words = np.asarray(words, dtype=np.uint32)
    i = ((words >> 16) & 0xFFFF).astype(np.uint16).view(np.int16).astype(np.int32)
    q = (words & 0xFFFF).astype(np.uint16).view(np.int16).astype(np.int32)
    return i, q


def unpack_words_to_dibits(words):
    """Unpack captured payload words (sc16 CPU layout) into a flat dibit array.

    Un-swaps the sc16 CPU halves to recover the item32 word the block emitted,
    then extracts dibit p from bits [2*p +: 2] (p = 0 first), the inverse of the
    FPGA axis_dibit_pack / the host pack_dibits_to_words on the TX side.
    """
    w = half_swap(words)
    d = np.empty((len(w), DIBITS_PER_WORD), dtype=np.uint8)
    for p in range(DIBITS_PER_WORD):
        d[:, p] = (w >> (2 * p)) & 3
    return d.reshape(-1)


def dibits_to_bits(d):
    """Dibit array (0..3) -> message bit array (bit0 = dibit&1, bit1 = dibit>>1).

    Matches transceiver.py's bit->dibit mapping (dibit = b0 | b1<<1), so the bit
    order here is the original message bit order.
    """
    d = np.asarray(d, dtype=np.uint8)
    out = np.empty(2 * len(d), dtype=np.uint8)
    out[0::2] = d & 1
    out[1::2] = (d >> 1) & 1
    return out


def transform_dibits(d, xor_c, swap):
    """Apply a QPSK-ambiguity transform to a dibit array.

    A residual constellation rotation (k*90 deg) or I/Q conjugation in the
    loopback permutes the two demodulated bits: optionally swapping b0/b1 (I/Q
    swap / conjugation) and/or XORing each with a constant (rotation). Trying all
    8 (xor_c in 0..3, swap in {0,1}) combinations resolves the ambiguity that
    channel equalization may leave when there is no absolute phase reference.
    """
    d = np.asarray(d, dtype=np.uint8)
    b0 = d & 1
    b1 = (d >> 1) & 1
    if swap:
        b0, b1 = b1, b0
    b0 = b0 ^ (xor_c & 1)
    b1 = b1 ^ ((xor_c >> 1) & 1)
    return (b0 | (b1 << 1)).astype(np.uint8)


def build_graph():
    """Host waveform -> radio0 TX -(loopback)-> radio0 RX -> ofdm_rx -> host."""
    graph = uhd.rfnoc.RfnocGraph(DEVICE_ARGS)

    # Waveform to the radio TX: sc16 (one uint32 word per sample).
    sa_tx = uhd.usrp.StreamArgs("sc16", "sc16")
    sa_tx.args = f"spp={SPP_TX}"
    tx_streamer = graph.create_tx_streamer(1, sa_tx)

    # Recovered payload words from the ofdm_rx block: sc16 (uint32 per word).
    sa_rx = uhd.usrp.StreamArgs("sc16", "sc16")
    sa_rx.args = f"spp={SPP_RX}"
    rx_streamer = graph.create_rx_streamer(1, sa_rx)

    ofdm = OfdmRxSlBlockControl(graph.get_block("0/Ofdm_rx_sl#0"))
    radio = uhd.rfnoc.RadioControl(graph.get_block("0/Radio#0"))

    # host -> radio0 TX
    graph.connect(tx_streamer, 0, radio.get_unique_id(), 0)
    # radio0 RX -> ofdm_rx
    uhd.rfnoc.connect_through_blocks(
        graph, radio.get_unique_id(), 0, ofdm.get_unique_id(), 0)
    # ofdm_rx -> host
    graph.connect(ofdm.get_unique_id(), 0, rx_streamer, 0)

    graph.commit()

    radio.set_rate(RATE)
    radio.set_properties(f"spp={SPP_RX}", 0)
    radio.poke32(0x1000, 0)  # loopback OFF for now

    return graph, tx_streamer, rx_streamer, ofdm, radio


def make_garbage(n, rng):
    """n random sc16 words (CPU layout): independent signed I/Q in +-2**(BITS-1).

    Built as item32 {I high, Q low} then half-swapped to the sc16 CPU layout, so
    the wire carries the intended item32 word -- exactly like the waveform path.
    """
    lo, hi = -(1 << (GARBAGE_BITS - 1)), (1 << (GARBAGE_BITS - 1))
    i16 = rng.integers(lo, hi, size=n).astype(np.int16)
    q16 = rng.integers(lo, hi, size=n).astype(np.int16)
    item32 = ((i16.astype(np.uint16).astype(np.uint32) << 16)
              | q16.astype(np.uint16).astype(np.uint32))
    return half_swap(item32)


def build_tiled_stream(waveform_words):
    """Normal stimulus: GUARD_SAMPS zeros + the golden waveform tiled N_WAVE_REPEAT."""
    wave_cpu = half_swap(waveform_words)
    guard = np.zeros(GUARD_SAMPS, dtype=np.uint32)
    stream = np.concatenate([guard, np.tile(wave_cpu, N_WAVE_REPEAT)])
    label = f"{GUARD_SAMPS} guard + {N_WAVE_REPEAT} x {len(waveform_words)}"
    cap_words = (N_WAVE_REPEAT + 1) * WORDS_PER_FRAME
    return stream.astype(np.uint32), cap_words, label


def build_garbage_stream(waveform_words):
    """Detection stimulus: garbage noise | waveform x N_EMBED | garbage noise.

    The receiver must reject the structureless noise (no CP correlation) and lock
    only on the embedded waveform, then recover its payload bit-exact.
    """
    rng = np.random.default_rng(0xC0FFEE)
    wave_cpu = half_swap(waveform_words)
    stream = np.concatenate([
        make_garbage(GARBAGE_PRE, rng),
        np.tile(wave_cpu, N_EMBED),
        make_garbage(GARBAGE_POST, rng),
    ])
    label = (f"{GARBAGE_PRE} garbage + {N_EMBED} x {len(waveform_words)} wave "
             f"+ {GARBAGE_POST} garbage")
    cap_words = (N_EMBED + 3) * WORDS_PER_FRAME
    return stream.astype(np.uint32), cap_words, label


def feed_and_capture(tx_streamer, rx_streamer, ofdm, radio, tx_cpu, cap_words, label):
    """Transmit a prebuilt sample stream and capture the recovered payload words.

    Arms the receiver (start + freqCorrectionEn on), turns the radio digital
    loopback on, starts a continuous payload capture, then sends `tx_cpu` (sc16
    CPU-layout words) as one timed burst. Returns the captured payload words
    (uint32, sc16 CPU layout).
    """
    # Arm the receiver.
    ofdm.set_freq_correction_en(True)
    ofdm.set_start(True)
    print(f"start={int(ofdm.get_start())} freqCorrectionEn={int(ofdm.get_freq_correction_en())}")

    radio.poke32(0x1000, 1)  # digital loopback ON
    print(f"Loopback register readback: {radio.peek32(0x1000)} (expected 1)")

    tiled = tx_cpu.reshape(1, -1).astype(np.uint32)

    # Capture a few frames' worth of payload (one frame = WORDS_PER_FRAME words);
    # the payload rate is low (one word per 16 sample cycles, data symbols only),
    # so this is far from the radio-RX-FIFO ceiling.
    rx_md = uhd.types.RXMetadata()
    raw_out = np.zeros((1, cap_words), dtype=np.uint32)

    # Stream CONTINUOUSLY rather than with a bounded num_done count. The rx
    # streamer reads the ofdm_rx *payload* output (one word per ~16 sample
    # cycles, data symbols only), but a num_done stream command propagates
    # upstream to the radio, which applies num_samps in *radio sample* units --
    # so a count sized in payload words would stop the radio after a few
    # thousand samples, far short of even one 8960-sample OFDM frame, and the
    # receiver would never see a full preamble to synchronize on. Continuous
    # streaming carries no count, so the radio free-runs and feeds the whole
    # tiled waveform through; the host drains the low-rate payload bounded by
    # cap_words below and stops the stream afterwards.
    stream_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    stream_cmd.stream_now = True
    rx_streamer.issue_stream_cmd(stream_cmd)

    # Transmit the waveform as one TIMED burst (SOB+EOB). Schedule it a little in
    # the future so UHD buffers the whole waveform on the device before the radio
    # starts draining it -- the radio then streams it out contiguously at 250
    # Msps. A non-timed (stream-now) burst makes the radio start consuming
    # immediately, but the host cannot sustain 250 Msps (1 GB/s) over the
    # transport, so the radio TX underflows and punches gaps into the looped-back
    # waveform. With the receiver locking once and free-running its comb-pilot
    # frame grid, a single mid-stream gap desynchronizes every later frame and the
    # recovered payload comes out almost all-zero -- the failure the gap-free sim
    # BFM never sees.
    tx_md = uhd.types.TXMetadata()
    tx_md.start_of_burst = True
    tx_md.end_of_burst = True
    tx_md.has_time_spec = True
    tx_md.time_spec = radio.get_time_now() + uhd.types.TimeSpec(0.5)
    n_fed = tx_streamer.send(tiled, tx_md, timeout=10.0)
    print(f"Fed {n_fed}/{tiled.shape[-1]} samples ({label})")

    # Drain the recovered payload until the bounded count arrives or it goes idle.
    got = 0
    while got < cap_words:
        n = rx_streamer.recv(raw_out[:, got:], rx_md, timeout=3.0)
        err = str(rx_md.error_code).lower()
        if "overflow" in err:
            print(f"Overflow after {got} payload words; keeping the clean prefix")
            break
        if "timeout" in err:
            # Expected once the receiver stops emitting (waveform finished).
            break
        if n == 0:
            break
        got += n

    stop_mode = getattr(uhd.types.StreamMode, "stop_cont",
                        getattr(uhd.types.StreamMode, "stop_continuous", None))
    if stop_mode is not None:
        rx_streamer.issue_stream_cmd(uhd.types.StreamCMD(stop_mode))

    radio.poke32(0x1000, 0)

    # Debug: peakInfo from the receiver's synchronizer.
    print(f"peakInfo: count={ofdm.get_peak_count()} "
          f"correlation={ofdm.get_peak_correlation()} "
          f"threshold={ofdm.get_peak_threshold()} "
          f"tOffset={ofdm.get_peak_toffset()} "
          f"fOffset={ofdm.get_peak_foffset()}")
    print(f"Captured {got}/{cap_words} payload words "
          f"(~{got / WORDS_PER_FRAME:.1f} frames)")

    return raw_out[0, :got]


def align_and_score(recovered, golden):
    """Find the best frame alignment + QPSK-ambiguity transform of `recovered`.

    Slides the golden frame (length NUM_TX_SAMPS dibits) across the recovered
    dibit stream, trying all 8 ambiguity transforms at each offset, and returns
    the (offset, xor_c, swap, n_bit_err, n_bits) with the fewest bit errors.
    """
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


def plot_result(recovered, golden, best):
    """Overlay the recovered constellation/bits against the golden payload."""
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
    axes[0].set_title(f"ofdm_rx payload bits (offset={off}, xor={xor_c}, swap={swap}, "
                      f"BER={err / n_bits:.2e})")
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


def sweep_garbage_bits(waveform, golden, levels=(9, 11, 12, 13, 14, 15)):
    """Sweep the garbage noise amplitude to find where detection breaks down.

    Each level runs in its OWN fresh device session (a subprocess re-invoking this
    script with --garbage --garbage-bits=N). This is deliberate: the receiver's
    frame grid (Frame_Counter) and synchronizer lock only re-zero on a data-path
    reset, so reusing one session would leave every level after the first
    demodulating with the first level's stale lock phase (identical, bogus ~0.5
    BER) -- an artifact, not a real noise limit. Uniform +-2**(BITS-1) noise has
    RMS 2**(BITS-1)/sqrt(3); signal RMS ~8000, so BITS=14 (~4730 RMS) is ~0 dB and
    BITS=15 over-ranges it.
    """
    import re
    import subprocess
    sig_rms = 8000.0
    print(f"\n{'bits':>4} {'noiseRMS':>9} {'SNR_dB':>7} {'count':>6} "
          f"{'BER':>12}  result")
    for bits in levels:
        noise_rms = (1 << (bits - 1)) / np.sqrt(3.0)
        snr_db = 20.0 * np.log10(sig_rms / noise_rms)
        out = subprocess.run(
            [sys.executable, os.path.abspath(__file__),
             "--garbage", f"--garbage-bits={bits}"],
            capture_output=True, text=True, env={**os.environ, "SHOW_PLOTS": "0"})
        txt = out.stdout + out.stderr
        m_cnt = re.search(r"peakInfo: count=(\d+)", txt)
        m_ber = re.search(r"BER = (\d+)/(\d+)", txt)
        n_det = int(m_cnt.group(1)) if m_cnt else -1
        if m_ber:
            err, n_bits = int(m_ber.group(1)), int(m_ber.group(2))
            ber = err / n_bits
            res = "PASS" if err == 0 else ("near" if ber < 1e-2 else "FAIL")
        else:
            ber, res = float("nan"), "no-run"
        print(f"{bits:>4} {noise_rms:>9.0f} {snr_db:>7.1f} {n_det:>6} "
              f"{ber:>12.3e}  {res}    <-- summary")
    return


def main():
    waveform = load_words(WAVEFORM_HEX)
    golden = load_dibits(PAYLOAD_HEX)
    print(f"Loaded waveform = {len(waveform)} sc16 words "
          f"({len(waveform) / NUM_RX_SAMPS:.0f} frame(s)); "
          f"golden payload = {len(golden)} dibits "
          f"({len(golden) / NUM_TX_SAMPS:.0f} frame(s))")
    if len(waveform) != NUM_RX_SAMPS:
        print(f"NOTE: waveform length {len(waveform)} != one frame ({NUM_RX_SAMPS}); "
              "tiling/comparison still uses one golden frame of payload")
    golden = golden[:NUM_TX_SAMPS]  # compare against one frame of payload

    if "--sweep" in sys.argv:
        sweep_garbage_bits(waveform, golden)
        sys.exit(0)

    for a in sys.argv:
        if a.startswith("--garbage-bits="):
            globals()["GARBAGE_BITS"] = int(a.split("=", 1)[1])

    garbage_mode = "--garbage" in sys.argv
    if garbage_mode:
        tx_cpu, cap_words, label = build_garbage_stream(waveform)
        print(f"GARBAGE-DETECTION mode: stream = {label} "
              f"(waveform sits {GARBAGE_PRE} samples = {GARBAGE_PRE / NUM_RX_SAMPS:.2f} "
              "frames into the burst)")
    else:
        tx_cpu, cap_words, label = build_tiled_stream(waveform)

    graph, tx_streamer, rx_streamer, ofdm, radio = build_graph()

    captured = feed_and_capture(
        tx_streamer, rx_streamer, ofdm, radio, tx_cpu, cap_words, label)
    np.save(os.path.join(_HERE, "captured_payload_raw.npy"), captured)

    if len(captured) < WORDS_PER_FRAME:
        print(f"FAIL: only {len(captured)} payload words captured "
              f"(< {WORDS_PER_FRAME} for one frame). Is the receiver synchronizing? "
              "Check peakInfo count above (0 => no preamble detected).")
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
        print("PASS: recovered payload bits match the golden TX payload exactly")
    elif ber < 1e-2:
        print(f"NEAR PASS: {err} bit errors (BER {ber:.2e}); likely residual "
              "frequency offset / a few edge symbols -- inspect peakInfo and the plot")
    else:
        print("FAIL: recovered payload does not match the golden payload. "
              "Check synchronization (peakInfo count/correlation vs threshold), "
              "the loopback, and that freqCorrectionEn is on.")

    if garbage_mode:
        n_det = ofdm.get_peak_count()
        if 0 < n_det <= N_EMBED and passed:
            verdict = ("DETECTED the embedded waveform amid garbage and recovered "
                       "the payload bit-exact; no false locks on the noise")
        elif n_det == 0:
            verdict = "did NOT detect any preamble (the noise floor may be masking it)"
        elif n_det > N_EMBED:
            verdict = (f"detected {n_det} > {N_EMBED} preambles -- some are false "
                       "locks on the garbage noise")
        else:
            verdict = "detected the waveform but the payload did not match"
        print(f"DETECTION SUMMARY: peakInfo count={n_det} "
              f"(expected ~{N_EMBED}, one per embedded frame) -- {verdict}")

    if SHOW_PLOTS:
        try:
            plot_result(recovered, golden, best)
        except Exception as exc:  # plotting is diagnostic only
            print(f"(plot skipped: {exc})")

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
