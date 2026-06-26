#!/usr/bin/env python3
"""Plot the garbage|waveform|garbage sample stream with the regions marked.

This is the exact stream `transceiver_rx.py --garbage` transmits through the
radio's digital loopback into ofdm_rx. The loopback is bit-exact (DUC/DDC ratio
1 at master_clock_rate == RATE, which is why recovery is BER=0), so these samples
are precisely what the OFDM receiver gets fed. The plot marks which samples are
random "garbage" noise and which are the embedded OFDM waveform, so it's clear
where the receiver must detect the waveform and recover the payload from.

    /usr/bin/python3 plot_loopback.py [--garbage-bits=N]
"""
import os
import sys
import numpy as np

import transceiver_rx as trx   # reuse the stream builder + constants

def out_png():
    return os.path.join(trx._HERE, f"loopback_stream_bits{trx.GARBAGE_BITS}.png")


def main():
    for a in sys.argv:
        if a.startswith("--garbage-bits="):
            trx.GARBAGE_BITS = int(a.split("=", 1)[1])

    waveform = trx.load_words(trx.WAVEFORM_HEX)
    tx_cpu, _, label = trx.build_garbage_stream(waveform)
    print(f"stream: {label}  ({len(tx_cpu)} samples, GARBAGE_BITS={trx.GARBAGE_BITS})")

    # |sample| over the whole stream (magnitude is invariant to the sc16 half-swap).
    i, q = trx.split_iq(tx_cpu)
    mag = np.sqrt(i.astype(np.float64) ** 2 + q.astype(np.float64) ** 2)

    # Known exact region boundaries from the construction:
    #   [0, PRE)  garbage | PRE + k*NUM_RX_SAMPS  waveform copies | tail garbage
    PRE = trx.GARBAGE_PRE
    W = trx.NUM_RX_SAMPS
    n = trx.N_EMBED
    wave_start, wave_end = PRE, PRE + n * W
    total = len(tx_cpu)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(15, 5))
    ax.plot(mag, lw=0.4, color="0.30")

    # Shade the two garbage regions and the waveform region.
    ax.axvspan(0, PRE, color="tab:orange", alpha=0.20, label="garbage noise")
    ax.axvspan(wave_end, total, color="tab:orange", alpha=0.20)
    ax.axvspan(wave_start, wave_end, color="tab:green", alpha=0.22,
               label=f"OFDM waveform (x{n})")
    # Dashed lines at each individual OFDM-frame boundary inside the waveform.
    for k in range(n + 1):
        ax.axvline(PRE + k * W, color="tab:green", lw=0.8, ls="--", alpha=0.7)

    # Annotate.
    ax.annotate("noise\nlead-in", ((PRE) / 2, mag.max() * 0.92),
                ha="center", va="top", color="tab:orange", fontsize=9)
    ax.annotate(f"{n} OFDM frames\n(receiver detects & demodulates these)",
                ((wave_start + wave_end) / 2, mag.max() * 0.98),
                ha="center", va="top", color="tab:green", fontsize=9)
    ax.annotate("noise\ntail", ((wave_end + total) / 2, mag.max() * 0.92),
                ha="center", va="top", color="tab:orange", fontsize=9)

    sig_rms = np.sqrt(np.mean(mag[wave_start:wave_end] ** 2))
    noi_rms = np.sqrt(np.mean(mag[:PRE] ** 2))
    ax.set_title(f"Loopback sample stream  ({label})\n"
                 f"waveform |.|RMS={sig_rms:.0f}   garbage |.|RMS={noi_rms:.0f}   "
                 f"(ratio {20*np.log10(sig_rms/noi_rms):+.1f} dB)")
    ax.set_xlabel("sample index  (250 Msps; 1 OFDM frame = 8960 samples)")
    ax.set_ylabel("|sample|  (sc16 counts)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.margins(x=0.01)
    fig.tight_layout()
    fig.savefig(out_png(), dpi=110)
    print(f"saved {out_png()}")


if __name__ == "__main__":
    main()
