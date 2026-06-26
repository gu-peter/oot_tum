//
// Copyright 2026 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: axis_dibit_pack
//
// Description:
//
//   Packs the OFDM_Receiver core's 1-dibit-per-cycle payload output into the
//   32-bit item stream the block emits over CHDR. This is the exact inverse of
//   axis_dibit_unpack on the transmit side: 16 QPSK dibits are packed per 32-bit
//   word, dibit p in bits [2*p +: 2] (p = 0 first / LSBs first), so the payload
//   round-trips byte-for-byte against the transmit golden vectors and the full
//   CHDR payload bandwidth is used (16/16 dibits per word).
//
//   One OFDM frame is 20 data symbols x 228 subcarriers = 4560 dibits = exactly
//   285 packed words (4560 % 16 == 0), so a frame always ends on a word
//   boundary and no partial-word flush is required.
//
//   The receiver core is free-running (no output-ready handshake) and must NOT
//   be back-pressured -- stalling its pipeline would break synchronization,
//   which needs a continuous input sample stream. The payload output rate is far
//   below the CHDR link rate (one word per 16 sample cycles, and only during the
//   data symbols of a frame), so the downstream output rxsl_FIFO drains it easily.
//   m_overflow flags the (not expected in loopback) case where a freshly
//   assembled word could not be accepted before the next one completed; it is
//   exposed for debug only and does not gate the core.
//
// Parameters:
//
//   DIBITS_PER_WORD : QPSK dibits packed per 32-bit item (16).
//

`default_nettype none

module axis_dibit_pack #(
  parameter int DIBITS_PER_WORD = 16
) (
  input  wire        clk,
  input  wire        rst,

  // Dibit stream from the OFDM_Receiver core. d_valid must already be gated by
  // the core's clk_enable so a dibit is counted exactly once.
  input  wire [1:0]  s_dibit,
  input  wire        s_dvalid,

  // Packed 32-bit payload words (16 dibits each, dibit p at [2*p +: 2]).
  output wire [31:0] m_tdata,
  output wire        m_tvalid,
  input  wire        m_tready,

  // Sticky status: a completed word was overwritten before being accepted.
  output reg         m_overflow
);

  localparam int CNT_W = $clog2(DIBITS_PER_WORD);

  reg [31:0]       acc;       // dibits accumulated so far (LSB dibit first)
  reg [CNT_W:0]    cnt;       // number of dibits in acc (0..DIBITS_PER_WORD)
  reg [31:0]       word_out;  // assembled word awaiting acceptance
  reg              word_vld;  // word_out valid

  wire accept   = m_tvalid && m_tready;            // downstream took the word
  wire complete = s_dvalid && (cnt == DIBITS_PER_WORD-1); // 16th dibit this cycle

  assign m_tdata  = word_out;
  assign m_tvalid = word_vld;

  always @(posedge clk) begin
    if (rst) begin
      acc        <= '0;
      cnt        <= '0;
      word_out   <= '0;
      word_vld   <= 1'b0;
      m_overflow <= 1'b0;
    end else begin
      // Output word handshake: clear valid once accepted.
      if (accept) begin
        word_vld <= 1'b0;
      end

      // Accumulate incoming dibits, place dibit p at bit [2*p +: 2].
      if (s_dvalid) begin
        if (cnt == DIBITS_PER_WORD-1) begin
          // Word complete: acc already holds dibits 0..14 in bits [29:0]; this
          // last dibit (p = 15) occupies the top two bits [31:30].
          word_out <= {s_dibit, acc[2*(DIBITS_PER_WORD-1)-1:0]};
          word_vld <= 1'b1;
          cnt      <= '0;
          acc      <= '0;
          // Overflow if a previous word is still waiting and not accepted now.
          if (word_vld && !accept) begin
            m_overflow <= 1'b1;
          end
        end else begin
          acc[2*cnt +: 2] <= s_dibit;
          cnt             <= cnt + 1'b1;
        end
      end
    end
  end

endmodule

`default_nettype wire
