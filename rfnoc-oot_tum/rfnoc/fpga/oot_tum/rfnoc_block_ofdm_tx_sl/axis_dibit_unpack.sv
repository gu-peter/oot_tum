//
// Copyright 2026 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: axis_dibit_unpack
//
// Description:
//
//   Unpacks a 32-bit payload item stream into the 1-dibit-per-cycle stream the
//   OFDM_Transmitter core consumes. Each 32-bit word carries 16 QPSK dibits,
//   dibit p in bits [2*p +: 2] (p = 0 first / LSBs first), so payload bandwidth
//   on the CHDR link is fully used (16/16 dibits per word) instead of the 2-of-8
//   bits the old per-byte s8 interface wasted.
//
//   Using a real 32-bit item also removes the byte-scramble that plagued the s8
//   interface: UHD's s8 CPU<->wire converter views the byte buffer as item32 and
//   byteswaps 32 bits at a time, reordering the sub-bytes (measured as a within-
//   8-byte XOR-5 permutation). A 32-bit item round-trips intact (same path the
//   sc16 txData output already uses), so word[2*p +: 2] here is exactly the
//   dibit the host placed at position p -- no compensating permutation required.
//
//   Frame alignment: the host pads each frame up to a whole number of 32-bit
//   words, so the last word of a frame contains DIBITS_PER_FRAME % 16 real
//   dibits followed by pad. This module drops the pad (advances through it
//   without presenting it to the core) so that EXACTLY DIBITS_PER_FRAME dibits
//   reach the core per frame -- no per-frame drift in the core's payload FIFO.
//
// Parameters:
//
//   DIBITS_PER_FRAME : QPSK dibits the core consumes per OFDM frame
//                      (20 data symbols x 228 subcarriers = 4560).
//

`default_nettype none

module axis_dibit_unpack #(
  parameter int DIBITS_PER_FRAME = 4560
) (
  input  wire        clk,
  input  wire        rst,

  // Packed 32-bit payload words (16 dibits each, dibit p at [2*p +: 2]).
  input  wire [31:0] s_tdata,
  input  wire        s_tvalid,
  output wire        s_tready,

  // Dibit stream to the OFDM_Transmitter core.
  output wire [1:0]  m_dibit,
  output wire        m_dvalid,
  input  wire        m_dready
);

  localparam int FC_W = $clog2(DIBITS_PER_FRAME);

  reg  [31:0]     sr       = '0;   // shift register: current word, LSB dibit first
  reg  [4:0]      cnt      = '0;   // dibits remaining in sr (0..16)
  reg  [FC_W-1:0] fcnt     = '0;   // real dibits emitted in the current frame
  reg             draining = 1'b0; // dropping the pad dibits at end of a frame

  wire word_empty = (cnt == 5'd0);

  // Load a new word whenever the shift register is empty.
  assign s_tready = word_empty;
  // Present a real dibit unless we are draining frame pad.
  assign m_dibit  = sr[1:0];
  assign m_dvalid = (cnt != 5'd0) && !draining;

  wire load = s_tvalid && s_tready;        // accept a new 32-bit word
  wire emit = m_dvalid && m_dready;        // core consumed a real dibit
  wire skip = (cnt != 5'd0) && draining;   // silently discard a pad dibit

  always @(posedge clk) begin
    if (rst) begin
      sr       <= '0;
      cnt      <= '0;
      fcnt     <= '0;
      draining <= 1'b0;
    end else if (load) begin
      sr       <= s_tdata;
      cnt      <= 5'd16;
      draining <= 1'b0;                     // fresh word -> resume emitting
    end else if (emit) begin
      sr  <= sr >> 2;
      cnt <= cnt - 1'b1;
      if (fcnt == DIBITS_PER_FRAME[FC_W-1:0] - 1'b1) begin
        fcnt     <= '0;                     // frame complete -> drop rest of word
        draining <= 1'b1;
      end else begin
        fcnt <= fcnt + 1'b1;
      end
    end else if (skip) begin
      sr  <= sr >> 2;
      cnt <= cnt - 1'b1;                    // drain pad; fcnt/draining held
    end
  end

endmodule

`default_nettype wire
