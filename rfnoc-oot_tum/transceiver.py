#!/usr/bin/env python3
#
# Copyright 2025 Ettus Research, a National Instruments Brand
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""Demonstrates how to send and receive IQ samples using the RFNoC Radio block.

All ports of either one or two daughter boards can be used in this example. The
transmitted and received data is plotted as time-domain IQ waveforms.
"""

import time
import matplotlib.pyplot as plt
import numpy as np
import uhd
from rfnoc_oot_tum import OfdmTxSlBlockControl


# --- Configuration ---
DEVICE_ARGS = "addr=10.157.161.243, master_clock_rate=250e6"
NUM_SAMPLES = 40812  # 81624 bits / 2 bits per QPSK sample
BITS = np.random.randint(0, 2, 2 * NUM_SAMPLES)  # 2 bits per QPSK symbol
AMPLITUDE = 0.5
CHANNELS = [0, 1]
LOOPBACK = True
DELAY = 56     # FPGA clock cycles to delay RX vs. TX (None = auto)
TX_GAIN = 15   # dB
RX_GAIN = 50   # dB
FREQ = 1.5e9   # Hz

graph_connections = {
    0: {"tx": [("0/Radio#0", 1)], "rx": [("0/Radio#0", 1)]},
    1: {"tx": [("0/Ofdm_tx_sl#0", 0, "0/Radio#0", 0)], "rx": [("0/Radio#0", 0)]},
}


def complex128_to_sc16(data, scale=32767):
    """Convert complex128 NumPy to sc16 stores as uint32."""
    real = np.int16(np.real(data) * scale)
    imag = np.int16(np.imag(data) * scale)
    return np.uint32((np.uint32(imag) << 16) | (np.uint32(real) & 0xFFFF))


def sc16_to_complex128(data):
    """Convert sc16 stored as uint32 to complex128 NumPy."""
    real = np.array(np.int16(data & 0xFFFF), dtype=np.float64) / 32767
    imag = np.array(np.int16((data >> 16) & 0xFFFF), dtype=np.float64) / 32767
    return np.array(real + 1j * imag, dtype=np.complex128)


def create_block_controller(block):
    """Create block controller for a given RfNoc block."""
    name = block.get_unique_id()
    if name[2:].startswith("Radio"):
        block = uhd.rfnoc.RadioControl(block)
    elif name[2:].startswith("Ofdm_tx_sl"):
        block = OfdmTxSlBlockControl(block)
    else:
        raise NotImplementedError(f"Cannot create controller for block {name}")
    return block


def connect_graph(graph, graph_connections, channels=[], tx_streamer=None, rx_streamer=None):
    """Connect the RfNoc graph and create the block controllers automatically."""
    controllers = dict()
    for chan_idx, chan in enumerate(channels):
        for direction in ["tx", "rx"]:
            print(f"Connections for channel {chan}/{direction.upper()}:")
            connections = graph_connections[chan][direction]
            for idx, conn in enumerate(connections):
                # Normalize tuple lengths:
                #   2-tuple: (block, port)                            — direct streamer connection
                #   4-tuple: (src, src_p, dst, dst_p)                — no converter
                #   6-tuple: (src, src_p, cvtr, cvtr_p, dst, dst_p) — with optional converter
                if len(conn) == 2:
                    src_name, src_idx = conn
                    dst_name, dst_idx = conn
                    cvtr_name, _ = None, None
                elif len(conn) == 4:
                    src_name, src_idx, dst_name, dst_idx = conn
                    cvtr_name, _ = None, None
                elif len(conn) == 6:
                    src_name, src_idx, cvtr_name, _, dst_name, dst_idx = conn
                else:
                    raise ValueError(f"Connection tuple must have 2, 4, or 6 elements, got {len(conn)}: {conn}")
                first = idx == 0
                last = idx == (len(connections) - 1)
                if src_name not in controllers:
                    controllers[src_name] = create_block_controller(graph.get_block(src_name))
                if dst_name not in controllers:
                    controllers[dst_name] = create_block_controller(graph.get_block(dst_name))
                if direction == "tx" and first and tx_streamer is not None:
                    print(f"    TX streamer/port{chan_idx} -> {src_name}/port{src_idx}")
                    graph.connect(
                        tx_streamer,
                        chan_idx,
                        controllers[src_name].get_unique_id(),
                        src_idx,
                    )
                # skip src->dst link when the 2-tuple shorthand points the same block/port
                if src_name != dst_name or src_idx != dst_idx:
                    print(f"    {src_name}/port{src_idx} -> {dst_name}/port{dst_idx}")
                    uhd.rfnoc.connect_through_blocks(
                        graph,
                        controllers[src_name].get_unique_id(),
                        src_idx,
                        controllers[dst_name].get_unique_id(),
                        dst_idx,
                    )
                if cvtr_name is not None:
                    try:
                        controllers[cvtr_name] = create_block_controller(graph.get_block(cvtr_name))
                    except RuntimeError:
                        pass
                if direction == "rx" and last and rx_streamer is not None:
                    # connect RX streamer
                    print(f"    {dst_name}/port{dst_idx} -> RX streamer/port{chan_idx}")
                    graph.connect(
                        controllers[dst_name].get_unique_id(),
                        dst_idx,
                        rx_streamer,
                        chan_idx,
                    )
    return (graph, controllers.values())


def configure_radio_block(
    radio, spp, freq=None, tx_gain=None, rx_gain=None, digital_loopback=False
):
    """Configure the Radio block."""
    for blk_chan_idx in range(radio.get_num_input_ports()):
        # if the loopback argument was provided, enable the digital loopback
        # inside in the radio block. No external connection is required on the
        # USRP.
        radio.poke32(0x1000 + 128 * blk_chan_idx, int(digital_loopback))

        # Set the radio's RX packet size
        radio.set_properties("spp=" + str(spp), blk_chan_idx)

        if not digital_loopback:
            # Set TX/RX frequency and gain
            if freq is not None:
                radio.set_tx_frequency(freq, blk_chan_idx)
                radio.set_rx_frequency(freq, blk_chan_idx)
            if tx_gain is not None:
                radio.set_tx_gain(tx_gain, blk_chan_idx)
            if rx_gain is not None:
                radio.set_rx_gain(rx_gain, blk_chan_idx)


def get_tx_rx_delay(radio, delay, loopback):
    """Determine the loopback delay."""
    tick_rate = radio.get_tick_rate()
    if delay is not None:
        loopback_delay = delay
    elif loopback:
        # With internal digital loopback, there's a fixed delay between TX and RX
        if tick_rate in [122.88e6, 125.0e6]:
            # Add 2 cycles for 100 MHz FPGA
            loopback_delay = 2
        elif tick_rate in [245.76e6, 250.0e6]:
            # Add 12 cycles for 200 MHz FPGA
            loopback_delay = 12
        elif tick_rate in [491.52e6, 500.0e6]:
            # Add 24 cycles for 400 MHz FPGA
            loopback_delay = 24
        else:
            raise NotImplementedError(f"Unsupported tick rate: {tick_rate / 1e6:0.2f} MS/s")
    else:
        # With RF loopback there's a fixed delay between TX and RX
        if tick_rate in [245.76e6, 250.0e6]:
            # Add 188 cycles for 200 MHz FPGA
            loopback_delay = 188
        elif tick_rate in [491.52e6, 500.0e6]:
            # Add 356 cycles for 400 MHz FPGA
            loopback_delay = 356
        else:
            raise NotImplementedError(f"Unsupported tick rate: {tick_rate / 1e6:0.2f} MS/s")
    return loopback_delay / tick_rate


def transmit_and_receive(
    tx_streamer, rx_streamer, tx_time, data_in, rx_time, data_out_size, num_samps
):
    """Transmit and receive."""
    stream_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
    stream_cmd.stream_now = False
    stream_cmd.time_spec = uhd.types.TimeSpec(rx_time)
    stream_cmd.num_samps = num_samps
    rx_streamer.issue_stream_cmd(stream_cmd)

    tx_md = uhd.types.TXMetadata()
    tx_md.time_spec = uhd.types.TimeSpec(tx_time)
    tx_md.has_time_spec = True
    tx_md.start_of_burst = True
    tx_md.end_of_burst = True
    num_tx = tx_streamer.send(data_in, tx_md, timeout=5.0)
    print(f"Sent {num_tx} samples")

    rx_md = uhd.types.RXMetadata()
    data_out = np.zeros(data_out_size, dtype=np.uint8)
    num_rx = rx_streamer.recv(data_out, rx_md, 5.0)
    print(f"Received {num_rx} samples")

    if num_rx < num_tx:
        raise RuntimeError("ERROR: number of received samples is too low")

    return data_out


def main():
    """Main function of the example."""
    num_chan = len(CHANNELS)

    spp = min(NUM_SAMPLES, 1996)
    print(f"Using samples per packet of {spp}")

    graph = uhd.rfnoc.RfnocGraph(DEVICE_ARGS)

    tx_sa = uhd.usrp.StreamArgs("u8", "u8")
    tx_sa.args = f"spp={spp}"
    tx_streamer = graph.create_tx_streamer(num_chan, tx_sa)
    rx_sa = uhd.usrp.StreamArgs("u8", "u8")
    rx_sa.args = f"spp={spp}"
    rx_streamer = graph.create_rx_streamer(num_chan, rx_sa)

    graph, blocks = connect_graph(graph, graph_connections, CHANNELS, tx_streamer, rx_streamer)
    radio_blocks = [x for x in blocks if isinstance(x, uhd.libpyuhd.rfnoc.radio_control)]

    graph.commit()

    for radio in radio_blocks:
        configure_radio_block(
            radio,
            spp,
            freq=FREQ,
            rx_gain=RX_GAIN,
            tx_gain=TX_GAIN,
            digital_loopback=LOOPBACK,
        )

    # Pack 2 bits per sample as ufix1: bit0 → txPayload_0 (tdata[0]), bit1 → txPayload_1 (tdata[1])
    bits = BITS.reshape(NUM_SAMPLES, 2)
    data_in_row = np.uint8((bits[:, 1] << 1) | bits[:, 0])
    data_in = np.tile(data_in_row, (num_chan, 1))
    bits_in = np.stack([data_in & 0x01, (data_in >> 1) & 0x01])  # for plotting only

    radio = radio_blocks[0]
    tx_time = radio.get_time_now().get_real_secs() + 1.0
    data_out = transmit_and_receive(
        tx_streamer=tx_streamer,
        rx_streamer=rx_streamer,
        tx_time=tx_time,
        rx_time=tx_time + get_tx_rx_delay(radio, DELAY, LOOPBACK),
        data_in=data_in,
        data_out_size=(num_chan, NUM_SAMPLES),
        num_samps=NUM_SAMPLES,
    )

    bits_out = np.stack([data_out & 0x01, (data_out >> 1) & 0x01])

    _, axes = plt.subplots(2, num_chan, figsize=(14, 6))
    if num_chan == 1:
        axes = axes.reshape(-1, 1)
    for chan_idx, chan in enumerate(CHANNELS):
        axes[0][chan_idx].set_title(f"Channel {chan} TX")
        axes[0][chan_idx].plot(bits_in[0][chan_idx], label="bit0")
        axes[0][chan_idx].plot(bits_in[1][chan_idx], label="bit1")
        axes[0][chan_idx].legend()
        axes[0][chan_idx].grid(True)
        axes[1][chan_idx].set_title(f"Channel {chan} RX")
        axes[1][chan_idx].plot(bits_out[0][chan_idx], label="bit0")
        axes[1][chan_idx].plot(bits_out[1][chan_idx], label="bit1")
        axes[1][chan_idx].legend()
        axes[1][chan_idx].grid(True)
    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
