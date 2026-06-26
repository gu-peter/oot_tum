//
// Copyright 2026 <author>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Module: rfnoc_block_ofdm_rx_sl_tb
//
// Description: Testbench for the ofdm_rx_sl RFNoC block.
//
//   This is the receive-side counterpart of rfnoc_block_ofdm_tx_sl_tb. The TX
//   TB fed a golden payload into ofdm_tx_sl and checked the modulated waveform;
//   here we feed the golden *waveform* into ofdm_rx_sl and check that the
//   recovered payload bits equal the original golden payload.
//
//   Golden references (the TX vectors, used in reverse):
//     - ofdm_tx_expected.hex : the modulator's golden sc16 output, item32 words
//                              {I[31:16], Q[15:0]} -> the RX block's input.
//     - ofdm_tx_input.hex    : the original QPSK dibits (0..3) -> the expected
//                              RX block output.
//
//   The receiver must SYNCHRONIZE on the waveform before it can demodulate, so
//   we feed a short zero lead-in followed by the golden waveform tiled several
//   times (the "capture roughly twice the waveform length to synchronize" the
//   block needs). The recovered payload comes out packed 16 dibits per 32-bit
//   item (dibit p in bits [2*p +: 2]) -- exactly the packing ofdm_tx_sl consumes
//   -- so we unpack, find the frame alignment, resolve the QPSK phase ambiguity
//   (a 90/180/270 deg rotation or I/Q conjugation permutes the dibit labels),
//   and assert zero bit errors over one frame.
//

`default_nettype none


module rfnoc_block_ofdm_rx_sl_tb;

  `include "test_exec.svh"

  import PkgTestExec::*;
  import rfnoc_chdr_utils_pkg::*;
  import PkgChdrData::*;
  import PkgRfnocBlockCtrlBfm::*;
  import PkgRfnocItemUtils::*;

  //---------------------------------------------------------------------------
  // Testbench Configuration
  //---------------------------------------------------------------------------

  localparam [31:0] NOC_ID          = 32'h23AB3958;
  localparam [ 9:0] THIS_PORTID     = 10'h123;
  localparam int    CHDR_W          = 64;    // CHDR size in bits
  localparam int    MTU             = 10;    // Log2 of max transmission unit in CHDR words
  localparam int    NUM_PORTS_I     = 1;
  localparam int    NUM_PORTS_O     = 1;
  localparam int    ITEM_W          = 32;    // Sample size in bits
  localparam int    SPP             = 64;    // Samples per packet
  localparam int    PKT_SIZE_BYTES  = SPP * (ITEM_W/8);
  localparam int    STALL_PROB      = 25;    // Default BFM stall probability
  // Clock periods match the X440 X4_200 hardware image (see the TX TB notes):
  //   rfnoc_chdr_clk : 200 MHz, rfnoc_ctrl_clk : 40 MHz, ce_clk : 266.667 MHz.
  // The block derives its 250 MHz ce_250 data-path clock from ce_clk with an
  // internal MMCM, so ce_clk only feeds the MMCM here.
  localparam real   CHDR_CLK_PER    = 5.0;    // 200 MHz
  localparam real   CTRL_CLK_PER    = 25.0;   // 40 MHz
  localparam real   CE_CLK_PER      = 3.75;   // 266.667 MHz

  // Client register map (matches rfnoc_block_ofdm_rx_sl.sv).
  localparam int    REG_START_ADDR            = 'h00;  // RW: receiver 'start'
  localparam int    REG_FREQ_CORR_EN_ADDR     = 'h04;  // RW: 'freqCorrectionEn'
  localparam int    REG_PEAK_CORRELATION_ADDR = 'h08;  // RO: peakInfo correlation (ufix32_En24)
  localparam int    REG_PEAK_THRESHOLD_ADDR   = 'h0C;  // RO: peakInfo threshold   (ufix32_En24)
  localparam int    REG_PEAK_TOFFSET_ADDR     = 'h10;  // RO: peakInfo tOffset     (ufix14)
  localparam int    REG_PEAK_FOFFSET_ADDR     = 'h14;  // RO: peakInfo fOffset     (int32)
  localparam int    REG_PEAK_COUNT_ADDR       = 'h18;  // RO: peakInfoValid strobe count

  // Frame geometry (mirrors the TX block / golden vectors).
  localparam int    NUM_TX_SAMPS    = 4560;   // QPSK dibits (20*228) per frame
  localparam int    NUM_RX_SAMPS    = 8960;   // sc16 waveform samples per frame (28*320)
  localparam int    DIBITS_PER_WORD = 16;     // dibits packed per 32-bit item
  localparam int    WORDS_PER_FRAME = NUM_TX_SAMPS / DIBITS_PER_WORD;  // 285

  // Golden-reference files (live next to this testbench; xsim's run dir is deep
  // inside the Vivado sim project, so use an absolute path).
  localparam string VEC_DIR =
    "/home/peter/git/oot_tum/rfnoc-oot_tum/rfnoc/fpga/oot_tum/rfnoc_block_ofdm_rx_sl/";
  localparam string WAVE_FILE    = {VEC_DIR, "ofdm_rx_input.hex"};    // RX input waveform
  localparam string PAYLOAD_FILE = {VEC_DIR, "ofdm_rx_expected.hex"}; // expected dibits

  // Stimulus shape. A short zero lead-in lets the receiver pipeline / scaling
  // settle (the radio sees noise/zeros before the waveform on hardware), then
  // the golden waveform is tiled N_WAVE_REPEAT times so the synchronizer has
  // several frames to lock on and emit clean payload for the later ones.
  localparam int    GUARD_SAMPS    = 2048;
  localparam int    N_WAVE_REPEAT  = 6;
  // How many recovered frames of payload to capture and search for a match.
  localparam int    N_CAP_FRAMES   = 2;
  localparam int    CAP_WORDS      = N_CAP_FRAMES * WORDS_PER_FRAME;

  // Alignment search bounds (keep the SV-modeled search cheap): a coarse prefix
  // screen over offset x ambiguity-transform, then one full-frame compare.
  localparam int    SEARCH_PREFIX  = 128;     // dibits used to screen alignment
  localparam int    MAX_BIT_ERR    = 0;       // require an exact payload match

  // Test 5: the captured-window-with-garbage scenario. We feed exactly twice the
  // waveform length (2 x NUM_RX_SAMPS samples), with one copy of the golden
  // waveform embedded at GARBAGE_PRE and random "garbage" (a moderate-amplitude
  // LFSR noise, ~signal RMS, no cyclic-prefix structure) filling the rest. The
  // receiver must reject the garbage and lock on the embedded preamble. Front-
  // end scaling is a fixed 0.875 gain (Input_Scaling), so the garbage amplitude
  // does not perturb the signal path; the synchronizer's CP correlation simply
  // stays below threshold on the structureless noise.
  localparam int    GARBAGE_PRE    = 2048;                          // noise before the waveform
  localparam int    N_EMBED        = 1;                             // waveform copies embedded (user spec: 1)
  localparam int    TOTAL_SAMPS_5  = 2 * NUM_RX_SAMPS;              // 2x waveform length (user spec)
  localparam int    GARBAGE_POST   = TOTAL_SAMPS_5 - GARBAGE_PRE - N_EMBED*NUM_RX_SAMPS;
  // A recovered BER below this counts as "payload recovered" (no-recovery would
  // be ~50%); the embedded golden frame should in fact come back bit-exact.
  localparam real   MAX_BER_5      = 0.01;

  // Test 6: pure white noise, no waveform embedded anywhere (2x the waveform
  // length of garbage). There is no preamble in the buffer, so a correct detector
  // must NOT fire. This isolates the false-detection behavior from test 5: any
  // peakInfo strobe here is a pure false alarm on noise (no signal to confuse it
  // with), which directly characterizes the energy-adaptive threshold's behavior.
  localparam int    TOTAL_SAMPS_6  = 2 * NUM_RX_SAMPS;

  // Golden memories.
  logic [31:0] wave_mem    [NUM_RX_SAMPS];
  logic [ 7:0] payload_mem [NUM_TX_SAMPS];

  //---------------------------------------------------------------------------
  // Clocks and Resets
  //---------------------------------------------------------------------------

  bit rfnoc_chdr_clk;
  bit rfnoc_ctrl_clk;
  bit ce_clk;

  sim_clock_gen #(CHDR_CLK_PER) rfnoc_chdr_clk_gen (.clk(rfnoc_chdr_clk), .rst());
  sim_clock_gen #(CTRL_CLK_PER) rfnoc_ctrl_clk_gen (.clk(rfnoc_ctrl_clk), .rst());
  sim_clock_gen #(CE_CLK_PER) ce_clk_gen (.clk(ce_clk), .rst());

  //---------------------------------------------------------------------------
  // Bus Functional Models
  //---------------------------------------------------------------------------

  RfnocBackendIf        backend (rfnoc_chdr_clk, rfnoc_ctrl_clk);
  AxiStreamIf #(32)     m_ctrl (rfnoc_ctrl_clk, 1'b0);
  AxiStreamIf #(32)     s_ctrl (rfnoc_ctrl_clk, 1'b0);
  AxiStreamIf #(CHDR_W) m_chdr [NUM_PORTS_I] (rfnoc_chdr_clk, 1'b0);
  AxiStreamIf #(CHDR_W) s_chdr [NUM_PORTS_O] (rfnoc_chdr_clk, 1'b0);

  RfnocBlockCtrlBfm #(CHDR_W, ITEM_W) blk_ctrl = new(backend, m_ctrl, s_ctrl);

  typedef ChdrData #(CHDR_W, ITEM_W)::chdr_word_t chdr_word_t;
  typedef ChdrData #(CHDR_W, ITEM_W)::item_t      item_t;

  for (genvar i = 0; i < NUM_PORTS_I; i++) begin : gen_bfm_input_connections
    initial begin
      blk_ctrl.connect_master_data_port(i, m_chdr[i], PKT_SIZE_BYTES);
      blk_ctrl.set_master_stall_prob(i, STALL_PROB);
    end
  end
  for (genvar i = 0; i < NUM_PORTS_O; i++) begin : gen_bfm_output_connections
    initial begin
      blk_ctrl.connect_slave_data_port(i, s_chdr[i]);
      blk_ctrl.set_slave_stall_prob(i, STALL_PROB);
    end
  end

  //---------------------------------------------------------------------------
  // Device Under Test (DUT)
  //---------------------------------------------------------------------------

  logic [CHDR_W*NUM_PORTS_I-1:0] s_rfnoc_chdr_tdata;
  logic [       NUM_PORTS_I-1:0] s_rfnoc_chdr_tlast;
  logic [       NUM_PORTS_I-1:0] s_rfnoc_chdr_tvalid;
  logic [       NUM_PORTS_I-1:0] s_rfnoc_chdr_tready;

  logic [CHDR_W*NUM_PORTS_O-1:0] m_rfnoc_chdr_tdata;
  logic [       NUM_PORTS_O-1:0] m_rfnoc_chdr_tlast;
  logic [       NUM_PORTS_O-1:0] m_rfnoc_chdr_tvalid;
  logic [       NUM_PORTS_O-1:0] m_rfnoc_chdr_tready;

  for (genvar i = 0; i < NUM_PORTS_I; i++) begin : gen_dut_input_connections
    assign s_rfnoc_chdr_tdata[CHDR_W*i+:CHDR_W] = m_chdr[i].tdata;
    assign s_rfnoc_chdr_tlast[i]                = m_chdr[i].tlast;
    assign s_rfnoc_chdr_tvalid[i]               = m_chdr[i].tvalid;
    assign m_chdr[i].tready                     = s_rfnoc_chdr_tready[i];
  end
  for (genvar i = 0; i < NUM_PORTS_O; i++) begin : gen_dut_output_connections
    assign s_chdr[i].tdata        = m_rfnoc_chdr_tdata[CHDR_W*i+:CHDR_W];
    assign s_chdr[i].tlast        = m_rfnoc_chdr_tlast[i];
    assign s_chdr[i].tvalid       = m_rfnoc_chdr_tvalid[i];
    assign m_rfnoc_chdr_tready[i] = s_chdr[i].tready;
  end

  rfnoc_block_ofdm_rx_sl #(
    .THIS_PORTID         (THIS_PORTID),
    .CHDR_W              (CHDR_W),
    .MTU                 (MTU)
  ) dut (
    .rfnoc_chdr_clk      (rfnoc_chdr_clk),
    .rfnoc_ctrl_clk      (rfnoc_ctrl_clk),
    .ce_clk              (ce_clk),
    // 250 MHz data-path clock generated inside the block (MMCM output). Left
    // open here; the block drives it.
    .ce_250_clk          (),
    .rfnoc_core_config   (backend.cfg),
    .rfnoc_core_status   (backend.sts),
    .s_rfnoc_chdr_tdata  (s_rfnoc_chdr_tdata),
    .s_rfnoc_chdr_tlast  (s_rfnoc_chdr_tlast),
    .s_rfnoc_chdr_tvalid (s_rfnoc_chdr_tvalid),
    .s_rfnoc_chdr_tready (s_rfnoc_chdr_tready),
    .m_rfnoc_chdr_tdata  (m_rfnoc_chdr_tdata),
    .m_rfnoc_chdr_tlast  (m_rfnoc_chdr_tlast),
    .m_rfnoc_chdr_tvalid (m_rfnoc_chdr_tvalid),
    .m_rfnoc_chdr_tready (m_rfnoc_chdr_tready),
    .s_rfnoc_ctrl_tdata  (m_ctrl.tdata),
    .s_rfnoc_ctrl_tlast  (m_ctrl.tlast),
    .s_rfnoc_ctrl_tvalid (m_ctrl.tvalid),
    .s_rfnoc_ctrl_tready (m_ctrl.tready),
    .m_rfnoc_ctrl_tdata  (s_ctrl.tdata),
    .m_rfnoc_ctrl_tlast  (s_ctrl.tlast),
    .m_rfnoc_ctrl_tvalid (s_ctrl.tvalid),
    .m_rfnoc_ctrl_tready (s_ctrl.tready)
  );

  //---------------------------------------------------------------------------
  // Clock Sanity Check
  //---------------------------------------------------------------------------

  initial begin : clk_check
    int n_chdr, n_ctrl, n_ce;
    n_chdr = 0; n_ctrl = 0; n_ce = 0;
    fork
      forever @(posedge rfnoc_chdr_clk) n_chdr++;
      forever @(posedge rfnoc_ctrl_clk) n_ctrl++;
      forever @(posedge ce_clk)         n_ce++;
    join_none
    #1000;
    $display("CLOCKCHK @1us: chdr=%0d (exp ~200) ctrl=%0d (exp ~40) ce=%0d (exp ~266)",
             n_chdr, n_ctrl, n_ce);
    `ASSERT_ERROR(n_chdr > 0 && n_ctrl > 0 && n_ce > 0,
                  "One or more framework clocks is not toggling");
  end

  //---------------------------------------------------------------------------
  // Helper Tasks / Functions
  //---------------------------------------------------------------------------

  // Feed the captured-waveform stimulus into the 'in' port as 32-bit sc16 items
  // ({I[31:16], Q[15:0]}). A short zero lead-in is followed by the golden
  // waveform tiled N_WAVE_REPEAT times. send_packets_items feeds them directly
  // as items (no CPU<->wire sc16 swap, unlike the host path), so the core sees
  // each word verbatim: rxData_re = word[31:16], rxData_im = word[15:0].
  task automatic feed_waveform();
    item_t items[$];
    items = {};
    for (int g = 0; g < GUARD_SAMPS; g++) items.push_back(32'h0);
    for (int r = 0; r < N_WAVE_REPEAT; r++)
      for (int n = 0; n < NUM_RX_SAMPS; n++) items.push_back(wave_mem[n]);
    $display("feed_waveform: sending %0d items (%0d guard + %0d x %0d waveform)",
             items.size(), GUARD_SAMPS, N_WAVE_REPEAT, NUM_RX_SAMPS);
    blk_ctrl.send_packets_items(0, items);
  endtask : feed_waveform

  // Capture nwords recovered payload items from the 'out' port and unpack them
  // into a flat dibit queue (dibit p from bits [2*p +: 2], p = 0 first).
  task automatic capture_payload(input int nwords, output logic [1:0] rec[$]);
    item_t words[$];
    int    got;
    rec = {};
    got = 0;
    while (got < nwords) begin
      blk_ctrl.recv_items(0, words);
      foreach (words[i]) begin
        if (got >= nwords) break;
        for (int p = 0; p < DIBITS_PER_WORD; p++)
          rec.push_back(words[i][2*p +: 2]);
        got++;
      end
    end
    $display("capture_payload: collected %0d words (%0d dibits)", got, rec.size());
  endtask : capture_payload

  // One random "garbage" sc16 word from a 32-bit Galois LFSR: independent
  // GARBAGE_BITS-wide signed I and Q. With GARBAGE_BITS = 9 the range is +-256,
  // ~30 dB below the signal RMS (~8015) -- a realistic capture noise floor.
  // Random noise has no cyclic-prefix repetition, so the receiver's CP
  // correlation stays below threshold on it (no false lock) AND the low level
  // does not corrupt the frequency/timing estimate the way a 0 dB-SNR noise
  // would.
  localparam int GARBAGE_BITS = 9;   // signed noise width -> +-2**(BITS-1)
  function automatic logic [31:0] garbage_word(ref logic [31:0] lfsr);
    logic signed [15:0] gi, gq;
    lfsr = lfsr[0] ? ((lfsr >> 1) ^ 32'hA300_0000) : (lfsr >> 1);
    gi   = $signed(lfsr[GARBAGE_BITS-1:0]);
    lfsr = lfsr[0] ? ((lfsr >> 1) ^ 32'hA300_0000) : (lfsr >> 1);
    gq   = $signed(lfsr[GARBAGE_BITS-1:0]);
    return {gi, gq};
  endfunction : garbage_word

  // Feed the "captured window" stimulus for test 5: GARBAGE_PRE noise samples,
  // then one copy of the golden waveform, then GARBAGE_POST noise samples
  // (total 2 x NUM_RX_SAMPS). The waveform is thus embedded somewhere in twice
  // its length of garbage.
  task automatic feed_garbage_waveform();
    item_t       items[$];
    logic [31:0] lfsr;
    items = {};
    lfsr  = 32'hCAFE_BABE;
    for (int g = 0; g < GARBAGE_PRE;  g++) items.push_back(garbage_word(lfsr));
    for (int r = 0; r < N_EMBED; r++)
      for (int n = 0; n < NUM_RX_SAMPS; n++) items.push_back(wave_mem[n]);
    for (int g = 0; g < GARBAGE_POST; g++) items.push_back(garbage_word(lfsr));
    $display("feed_garbage_waveform: %0d items = %0d garbage + %0d waveform + %0d garbage (waveform at sample %0d of %0d)",
             items.size(), GARBAGE_PRE, N_EMBED*NUM_RX_SAMPS, GARBAGE_POST,
             GARBAGE_PRE, TOTAL_SAMPS_5);
    blk_ctrl.send_packets_items(0, items);
  endtask : feed_garbage_waveform

  // Feed pure white-noise garbage for test 6: TOTAL_SAMPS_6 (= 2 x NUM_RX_SAMPS)
  // LFSR-noise samples with NO waveform embedded anywhere. A correct detector
  // sees no preamble and must never strobe peakInfo. Uses a different LFSR seed
  // than test 5 so the noise is independent.
  task automatic feed_pure_noise();
    item_t       items[$];
    logic [31:0] lfsr;
    items = {};
    lfsr  = 32'h1234_5678;
    for (int g = 0; g < TOTAL_SAMPS_6; g++) items.push_back(garbage_word(lfsr));
    $display("feed_pure_noise: sending %0d items of pure white-noise garbage (no waveform embedded)",
             items.size());
    blk_ctrl.send_packets_items(0, items);
  endtask : feed_pure_noise

  // Read and print the latched peakInfo debug registers. correlation/threshold
  // are raw ufix32_En24 (divide by 2**24 for the normalized value); tOffset is
  // ufix14; fOffset is signed int32.
  task automatic print_peakinfo(input string ctx);
    logic [31:0] cnt, corr, thr, toff, foff;
    blk_ctrl.reg_read(REG_PEAK_COUNT_ADDR,       cnt);
    blk_ctrl.reg_read(REG_PEAK_CORRELATION_ADDR, corr);
    blk_ctrl.reg_read(REG_PEAK_THRESHOLD_ADDR,   thr);
    blk_ctrl.reg_read(REG_PEAK_TOFFSET_ADDR,     toff);
    blk_ctrl.reg_read(REG_PEAK_FOFFSET_ADDR,     foff);
    $display("peakInfo [%s]:", ctx);
    $display("  count=%0d  correlation=%0d (En24)  threshold=%0d (En24)  corr>=thr? %0d",
             cnt, corr, thr, (corr >= thr));
    $display("  tOffset=%0d  fOffset=%0d", toff[13:0], $signed(foff));
  endtask : print_peakinfo

  // Discard any output packets left in the slave BFM queue (e.g. extra golden
  // frames emitted by a previous test) so the next capture starts clean. Settle
  // first so in-flight packets land in the queue, then drain non-blocking.
  task automatic drain_output(input int settle_cyc = 4000);
    ChdrPacket #(CHDR_W) pkt;
    int n;
    n = 0;
    repeat (settle_cyc) @(posedge rfnoc_chdr_clk);
    while (blk_ctrl.try_get_chdr(0, pkt)) n++;
    $display("drain_output: discarded %0d leftover output packets", n);
  endtask : drain_output

  // Bit errors between a recovered dibit (after an ambiguity transform) and a
  // golden dibit. The transform optionally swaps the two bits (I/Q swap /
  // conjugation) and XORs each with a constant (a k*90 deg rotation).
  function automatic int dibit_bit_errs(input logic [1:0] r,
                                        input logic [1:0] g,
                                        input int xor_c,
                                        input bit swap);
    logic b0, b1, t;
    b0 = r[0];
    b1 = r[1];
    if (swap) begin t = b0; b0 = b1; b1 = t; end
    b0 = b0 ^ xor_c[0];
    b1 = b1 ^ xor_c[1];
    dibit_bit_errs = (b0 !== g[0]) + (b1 !== g[1]);
  endfunction : dibit_bit_errs

  // Slide the golden frame across the recovered dibit stream, trying all 8
  // ambiguity transforms, to find the best (offset, xor_c, swap). A cheap prefix
  // screen (SEARCH_PREFIX dibits) selects the alignment, then one full-frame
  // compare reports the exact bit-error count.
  task automatic align_compare(input  logic [1:0] rec[$],
                               output int best_off,
                               output int best_xor,
                               output bit best_swap,
                               output int best_err,
                               output int n_bits);
    int rec_len, max_off, prefix_err, best_prefix;
    rec_len = rec.size();
    max_off = rec_len - NUM_TX_SAMPS;
    best_prefix = 1 << 30;
    best_off = 0; best_xor = 0; best_swap = 1'b0;

    if (max_off < 0) begin
      best_err = -1; n_bits = 2*NUM_TX_SAMPS;
      return;
    end

    for (int off = 0; off <= max_off; off++) begin
      for (int xc = 0; xc < 4; xc++) begin
        for (int sw = 0; sw < 2; sw++) begin
          prefix_err = 0;
          for (int i = 0; i < SEARCH_PREFIX; i++)
            prefix_err += dibit_bit_errs(rec[off+i], payload_mem[i][1:0],
                                         xc, sw[0]);
          if (prefix_err < best_prefix) begin
            best_prefix = prefix_err;
            best_off    = off;
            best_xor    = xc;
            best_swap   = sw[0];
          end
        end
      end
    end

    // Full-frame compare at the chosen alignment.
    best_err = 0;
    for (int i = 0; i < NUM_TX_SAMPS; i++)
      best_err += dibit_bit_errs(rec[best_off+i], payload_mem[i][1:0],
                                 best_xor, best_swap);
    n_bits = 2 * NUM_TX_SAMPS;
  endtask : align_compare

  //---------------------------------------------------------------------------
  // Main Test Process
  //---------------------------------------------------------------------------

  initial begin : tb_main

    test.start_tb("rfnoc_block_ofdm_rx_sl_tb", 300ms);

    // Load the golden references (waveform in, payload out).
    $readmemh(WAVE_FILE, wave_mem);
    $readmemh(PAYLOAD_FILE, payload_mem);

    // Start the BFMs running
    blk_ctrl.run();

    //--------------------------------
    // Wait for the block's 250 MHz data-path clock to come up
    //--------------------------------
    // The block derives ce_250 from ce_clk with an internal MMCM (same as the TX
    // block). The data-path flush in flush_and_reset spans the ce_250 domain, so
    // wait for MMCM lock before exercising any flush/reset.
    $display("Waiting for ce_250 MMCM to lock...");
    wait (dut.ce_250_locked === 1'b1);
    $display("ce_250 MMCM locked at t=%0t", $time);

    // Re-init the chdr<->ce_250 CDC FIFOs now that both clocks are running.
    blk_ctrl.reset_chdr();

    //--------------------------------
    // Reset
    //--------------------------------

    test.start_test("Flush block then reset it", 10us);
    blk_ctrl.flush_and_reset();
    test.end_test();

    //--------------------------------
    // Verify Block Info
    //--------------------------------

    test.start_test("Verify Block Info", 2us);
    `ASSERT_ERROR(blk_ctrl.get_noc_id() == NOC_ID, "Incorrect NOC_ID Value");
    `ASSERT_ERROR(blk_ctrl.get_num_data_i() == NUM_PORTS_I, "Incorrect NUM_DATA_I Value");
    `ASSERT_ERROR(blk_ctrl.get_num_data_o() == NUM_PORTS_O, "Incorrect NUM_DATA_O Value");
    `ASSERT_ERROR(blk_ctrl.get_mtu() == MTU, "Incorrect MTU Value");
    test.end_test();

    //--------------------------------
    // Control register read/write
    //--------------------------------

    begin : test_ctrl_regs
      logic [31:0] rb;
      test.start_test("Control registers: start / freqCorrectionEn read-write", 20us);

      // Both default to 1 in the block; flip and read back to prove the path.
      blk_ctrl.reg_write(REG_START_ADDR, 0);
      blk_ctrl.reg_read(REG_START_ADDR, rb);
      `ASSERT_ERROR(rb[0] == 1'b0, "start did not read back as 0");
      blk_ctrl.reg_write(REG_START_ADDR, 1);
      blk_ctrl.reg_read(REG_START_ADDR, rb);
      `ASSERT_ERROR(rb[0] == 1'b1, "start did not read back as 1");

      blk_ctrl.reg_write(REG_FREQ_CORR_EN_ADDR, 0);
      blk_ctrl.reg_read(REG_FREQ_CORR_EN_ADDR, rb);
      `ASSERT_ERROR(rb[0] == 1'b0, "freqCorrectionEn did not read back as 0");
      blk_ctrl.reg_write(REG_FREQ_CORR_EN_ADDR, 1);
      blk_ctrl.reg_read(REG_FREQ_CORR_EN_ADDR, rb);
      `ASSERT_ERROR(rb[0] == 1'b1, "freqCorrectionEn did not read back as 1");

      test.end_test();
    end : test_ctrl_regs

    //--------------------------------
    // Feed waveform, recover payload, compare against golden
    //--------------------------------

    begin : test_recover_payload
      logic [1:0]  rec[$];
      logic [31:0] peak_cnt;
      int          best_off, best_xor, best_err, n_bits;
      bit          best_swap;
      real         ber;

      test.start_test("Feed golden waveform, recover payload, match golden bits", 250ms);

      // Arm the receiver: start + frequency correction on (both default 1, set
      // explicitly so the test is self-contained).
      blk_ctrl.reg_write(REG_FREQ_CORR_EN_ADDR, 1);
      blk_ctrl.reg_write(REG_START_ADDR, 1);

      // Feed the waveform and capture the recovered payload concurrently. The
      // receiver synchronizes during the early waveform repeats and emits clean
      // payload for the later, fully synchronized frames; capture_payload blocks
      // until CAP_WORDS arrive.
      fork
        begin : producer
          feed_waveform();
        end
        begin : consumer
          capture_payload(CAP_WORDS, rec);
        end
      join

      // The synchronizer should have fired at least once (a detected preamble).
      blk_ctrl.reg_read(REG_PEAK_COUNT_ADDR, peak_cnt);
      print_peakinfo("test4 clean tiled");
      `ASSERT_ERROR(peak_cnt > 0,
                    "Receiver never detected a preamble (peakInfo count == 0)");

      // Align to one golden frame and resolve the QPSK ambiguity.
      align_compare(rec, best_off, best_xor, best_swap, best_err, n_bits);
      `ASSERT_ERROR(best_err >= 0,
                    "Not enough recovered dibits to align one golden frame");
      ber = real'(best_err) / real'(n_bits);
      $display("align_compare: offset=%0d xor=%0d swap=%0b  bit errors=%0d/%0d (BER=%.3e)",
               best_off, best_xor, best_swap, best_err, n_bits, ber);

      `ASSERT_ERROR(best_err <= MAX_BIT_ERR,
                    "Recovered payload bits do not match the golden payload");

      test.end_test();
    end : test_recover_payload

    //--------------------------------
    // Recover payload from a waveform embedded in garbage
    //--------------------------------

    begin : test_recover_in_garbage
      logic [1:0]  rec[$];
      logic [31:0] peak_cnt;
      int          best_off, best_xor, best_err, n_bits;
      bit          best_swap;
      real         ber;

      test.start_test("Waveform embedded in 2x-length garbage: recover golden bits", 250ms);

      // Clean slate: reset the block (clears the framer + receiver state and the
      // ctrlport regs back to their defaults start=1/freqCorrectionEn=1) and
      // drain any leftover golden frames test 4 left in the slave BFM queue, so
      // this capture can only contain payload from the garbage-embedded feed.
      blk_ctrl.flush_and_reset();
      drain_output();
      blk_ctrl.reg_write(REG_FREQ_CORR_EN_ADDR, 1);
      blk_ctrl.reg_write(REG_START_ADDR, 1);

      // Feed [garbage][one waveform][garbage] (2x the waveform length) and
      // capture one frame of recovered payload. The receiver must reject the
      // garbage, lock on the single embedded waveform, and emit exactly one
      // 285-word payload frame.
      fork
        begin : producer5
          feed_garbage_waveform();
        end
        begin : consumer5
          capture_payload(WORDS_PER_FRAME, rec);
        end
      join

      blk_ctrl.reg_read(REG_PEAK_COUNT_ADDR, peak_cnt);
      print_peakinfo("test5 garbage-embedded");
      `ASSERT_ERROR(peak_cnt > 0,
                    "Receiver never detected the embedded preamble (peakInfo count == 0)");

      align_compare(rec, best_off, best_xor, best_swap, best_err, n_bits);
      `ASSERT_ERROR(best_err >= 0,
                    "Not enough recovered dibits to align one golden frame");
      ber = real'(best_err) / real'(n_bits);
      $display("align_compare: offset=%0d xor=%0d swap=%0b  bit errors=%0d/%0d (BER=%.3e)",
               best_off, best_xor, best_swap, best_err, n_bits, ber);

      // Characterization result. With a quiet (zero) pre-roll and contiguous
      // clean frames (test 4) the receiver locks and recovers the payload
      // bit-exact. Here the signal is preceded by *garbage*: the synchronizer's
      // detection threshold is energy-adaptive, so on the low-energy noise it
      // collapses (threshold ~277 En24 here vs ~28e6 with signal present) and the
      // noise correlation trips a FALSE lock at the very start of the buffer
      // (tOffset=1, bogus fOffset) before the real preamble arrives. The receiver
      // never re-locks, so it demodulates from the wrong position (BER ~50%).
      // This is a real receiver limitation: it needs a quiet pre-roll (or an
      // absolute detection-threshold floor) to find a signal embedded in noise.
      // The hardware path (transceiver_rx.py) sidesteps it by transmitting the
      // waveform into an otherwise idle (zero) radio TX stream.
      if (ber < MAX_BER_5) begin
        $display("RESULT: payload RECOVERED from the garbage-embedded waveform (BER=%.3e)",
                 ber);
      end else begin
        $display("RESULT: payload NOT recovered (BER=%.3e). Root cause: energy-adaptive detector false-locked on pre-signal garbage (tOffset=1, threshold collapsed) instead of the real preamble; a quiet (zero) pre-roll as in test 4 recovers it bit-exact.",
                 ber);
      end
      // With the front-end warm-up gate on 'start' (see rfnoc_block_ofdm_rx_sl.sv)
      // the receiver no longer false-locks on the pre-signal garbage: it stays
      // blind until its sync windows fill, then locks on the real embedded
      // preamble and recovers the payload bit-exact. Enforce that.
      `ASSERT_ERROR(ber < MAX_BER_5,
                    "Payload not recovered from waveform embedded in garbage (receiver false-locked on pre-signal noise; warm-up gate should prevent this)");

      test.end_test();
    end : test_recover_in_garbage

    //--------------------------------
    // Pure white noise: does the detector fire?
    //--------------------------------

    begin : test_pure_noise
      logic [31:0] peak_cnt;

      test.start_test("Pure white noise (no waveform): detector should NOT fire", 250ms);

      // Clean slate so any detection here is attributable only to this feed.
      blk_ctrl.flush_and_reset();
      drain_output();
      blk_ctrl.reg_write(REG_FREQ_CORR_EN_ADDR, 1);
      blk_ctrl.reg_write(REG_START_ADDR, 1);

      // Feed pure noise only -- no preamble anywhere in the buffer. Do NOT block
      // on capture_payload: with no real signal the receiver may emit no payload
      // (or only spurious frames from a false lock), so just push the whole
      // buffer and then inspect peakInfo.
      feed_pure_noise();

      // Let the pipeline drain and any (false) detection latch into the regs;
      // also count how many payload frames, if any, were spuriously emitted.
      drain_output();

      blk_ctrl.reg_read(REG_PEAK_COUNT_ADDR, peak_cnt);
      print_peakinfo("test6 pure noise");
      if (peak_cnt == 0) begin
        $display("RESULT: detector correctly did NOT fire on pure white noise (peakInfo count == 0)");
      end else begin
        $display("RESULT: detector FIRED on pure white noise (count=%0d) -- a FALSE alarm (no signal present). Same root cause as test 5: the energy-adaptive threshold collapses to its hardThreshold floor on low-energy noise and the unsettled front-end startup transient trips a false lock (typically tOffset=1).",
                 peak_cnt);
      end
      // A correct detector never fires without a signal. With the warm-up gate
      // on 'start' the startup transient is masked, so the detector must stay
      // silent on pure noise. Enforce it.
      `ASSERT_ERROR(peak_cnt == 0,
                    "Detector fired on pure white noise (false alarm; warm-up gate should prevent this)");

      test.end_test();
    end : test_pure_noise

    //--------------------------------
    // Finish Up
    //--------------------------------

    test.end_tb();
  end : tb_main

endmodule : rfnoc_block_ofdm_rx_sl_tb


`default_nettype wire
