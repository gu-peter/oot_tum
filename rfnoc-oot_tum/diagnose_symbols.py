#!/usr/bin/env python3
"""Per-symbol breakdown of a captured ofdm_tx_sl frame vs the golden reference.

Self-contained (no uhd import): reuses the capture saved by transceiver.py
(captured_raw.npy) and re-implements the same reconstruct/align/compare math,
then reports, for every 320-sample OFDM symbol, whether it matches golden --
labeled sync / reference / data.

The hardware capture matches up to sample 640 (sync symbol 1 + reference symbol
2, both payload-independent) and diverges at the first data symbol. Reference
symbols recur every 4th symbol (2, 6, 10, ...) and are also payload-independent,
so the per-symbol map disambiguates:
  * REF after 640 PASS, DATA FAIL  => payload content corrupted (host packing).
  * REF after 640 also FAIL         => stream de-sync (drop / zero-run strip).

Run:  python3 diagnose_symbols.py
"""
import os
import numpy as np

try:
    from scipy.signal import correlate as _scipy_correlate
except ImportError:
    _scipy_correlate = None

_HERE = os.path.dirname(os.path.abspath(__file__))
EXPECTED_HEX = os.path.join(_HERE, "ofdm_tx_expected.hex")
CAPTURED_NPY = os.path.join(_HERE, "captured_raw.npy")

NUM_RX = 76800
SPS = 320            # FFT_LEN(256) + CP_LEN(64)
N_SYMS = 240
TOL = 33
CORR_LEN = 2048
MIN_GAP = 16


def load_expected_output(path):
    vals = np.loadtxt(path, dtype=np.uint32, converters={0: lambda s: int(s, 16)})
    return vals.reshape(-1)


def split_iq(words):
    words = np.asarray(words, dtype=np.uint32)
    i = ((words >> 16) & 0xFFFF).astype(np.uint16).view(np.int16).astype(np.int32)
    q = (words & 0xFFFF).astype(np.uint16).view(np.int16).astype(np.int32)
    return i, q


def to_complex(words):
    i, q = split_iq(words)
    return i.astype(np.float64) + 1j * q.astype(np.float64)


def reconstruct_valid(captured):
    isz = captured == 0
    starts = np.flatnonzero(np.r_[True, np.diff(isz.astype(np.int8)) != 0])
    runlens = np.diff(np.r_[starts, len(captured)])
    keep = np.ones(len(captured), dtype=bool)
    n_gap = 0
    for s, ln in zip(starts, runlens):
        if isz[s] and ln >= MIN_GAP:
            keep[s:s + ln] = False
            n_gap += ln
    valid = captured[keep]
    print(f"reconstruct: removed {n_gap} samples in >={MIN_GAP}-long zero runs; "
          f"valid stream {len(valid)}/{len(captured)}")
    return valid


def align_by_magnitude(valid, expected):
    tiled_mag = np.abs(to_complex(np.tile(expected, 2)))
    tiled_mag -= tiled_mag.mean()
    tmpl_mag = np.abs(to_complex(valid[:CORR_LEN]))
    tmpl_mag -= tmpl_mag.mean()
    if _scipy_correlate is not None:
        corr = _scipy_correlate(tiled_mag, tmpl_mag, mode="valid", method="fft")
    else:
        corr = np.correlate(tiled_mag, tmpl_mag, mode="valid")
    region = np.abs(corr[:NUM_RX])
    offset = int(np.argmax(region))
    lock_ratio = float(region[offset] / (np.median(region) + 1e-9))
    overlap_len = min(len(valid), NUM_RX)
    return offset, overlap_len, lock_ratio


def best_variant(valid, offset, overlap_len, expected):
    got = to_complex(valid[:overlap_len])
    exp = to_complex(np.tile(expected, 2)[offset:offset + overlap_len])
    got_swap = 1j * np.conj(got)
    amask = np.abs(exp) > 2000
    best = None
    for name, gz in (("direct", got), ("iq-swap", got_swap)):
        evm = (float(np.linalg.norm((gz - exp)[amask]) / (np.linalg.norm(exp[amask]) + 1e-9))
               if np.any(amask) else float("inf"))
        dev_i = np.abs(np.real(gz) - np.real(exp))
        dev_q = np.abs(np.imag(gz) - np.imag(exp))
        print(f"  [{name:8s}] EVM={evm*100:6.2f}%  max_dev={max(dev_i.max(), dev_q.max()):.0f} LSB  "
              f"n_fail={int(np.count_nonzero((dev_i > TOL) | (dev_q > TOL)))}")
        info = dict(name=name, evm=evm, dev_i=dev_i, dev_q=dev_q)
        if best is None or evm < best["evm"]:
            best = info
    return best


def symbol_type(sidx0):
    s1 = sidx0 + 1
    if s1 == 1:
        return "sync"
    if (s1 - 2) % 4 == 0:
        return "ref"
    return "data"


def main():
    if not os.path.exists(CAPTURED_NPY):
        raise SystemExit(f"{CAPTURED_NPY} not found -- run transceiver.py first.")
    captured = np.load(CAPTURED_NPY)
    expected = load_expected_output(EXPECTED_HEX)
    print(f"captured {len(captured)} samples ({int((captured==0).sum())} zeros), "
          f"expected {len(expected)}")

    valid = reconstruct_valid(captured)
    offset, overlap_len, lock_ratio = align_by_magnitude(valid, expected)
    print(f"aligned at golden offset {offset}  (lock ratio {lock_ratio:.1f}; >~3 = good)")
    print(f"verifying {overlap_len} samples = {overlap_len / SPS:.1f} OFDM symbols")
    best = best_variant(valid, offset, overlap_len, expected)
    print(f"using '{best['name']}' variant\n")

    dev_i, dev_q = best["dev_i"], best["dev_q"]
    dev = np.maximum(dev_i, dev_q)
    fail = (dev_i > TOL) | (dev_q > TOL)

    k = np.arange(overlap_len)
    gidx = (offset + k) % NUM_RX
    sidx = gidx // SPS

    if fail.any():
        first = int(np.flatnonzero(fail)[0])
        g0 = (offset + first) % NUM_RX
        print(f"first out-of-tolerance sample: aligned #{first}  golden #{g0}  symbol {g0//SPS + 1}\n")

    print(f"{'sym':>4} {'type':>5} {'n':>5} {'maxdev':>7} {'nfail':>6}  status")
    print("-" * 44)
    type_stat = {"sync": [0, 0], "ref": [0, 0], "data": [0, 0]}
    for s in range(N_SYMS):
        m = sidx == s
        n = int(m.sum())
        if n == 0:
            continue
        md = float(dev[m].max())
        nf = int(fail[m].sum())
        t = symbol_type(s)
        ok = nf == 0
        type_stat[t][0 if ok else 1] += 1
        if t in ("sync", "ref") or s < 4 or not ok:
            print(f"{s+1:>4} {t:>5} {n:>5} {md:>7.0f} {nf:>6}  {'OK' if ok else 'FAIL'}")

    print("-" * 44)
    for t in ("sync", "ref", "data"):
        p, f = type_stat[t]
        print(f"{t:>5}: {p} symbols OK, {f} FAIL")

    print()
    ref_fail = type_stat["ref"][1]
    data_fail = type_stat["data"][1]
    if ref_fail == 0 and data_fail > 0:
        print(">>> REF all pass, DATA fail => PAYLOAD CONTENT corrupted (host s8 packing /")
        print(">>> bit-order / sample shift). OFDM core + clocking + loopback are proven OK.")
    elif ref_fail > 0:
        print(">>> REF also fail after 640 => STREAM DE-SYNC (dropped sample or a real")
        print(">>> zero-run stripped by reconstruct_valid). Check zero-run lengths.")
    else:
        print(">>> No data-symbol failures in this window.")


if __name__ == "__main__":
    main()
