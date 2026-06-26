//
// Copyright 2026 <author>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Module: rfnoc_block_ofdm_tx_sl
//
// Description:
//
//   <Add block description here>
//
// Parameters:
//
//   THIS_PORTID : Control crossbar port to which this block is connected
//   CHDR_W      : AXIS-CHDR data bus width
//   MTU         : Maximum transmission unit (i.e., maximum packet size in
//                 CHDR words is 2**MTU).
//

`default_nettype none

module rfnoc_block_ofdm_tx_sl #(
  parameter [9:0] THIS_PORTID     = 10'd0,
  parameter       CHDR_W          = 64,
  parameter [5:0] MTU             = 10
)(
  // RFNoC Framework Clocks and Resets
  input  wire                   rfnoc_chdr_clk,
  input  wire                   rfnoc_ctrl_clk,
  input  wire                   ce_clk,
  // 250 MHz data-path clock generated inside this block (MMCM, from ce_clk).
  // Declared 'direction: out' in the block YAML, so the image core treats this
  // as a block output and the block drives it.
  output wire                   ce_250_clk,
  // AXIS-CHDR Input Ports (from framework)
  input  wire [(1)*CHDR_W-1:0] s_rfnoc_chdr_tdata,
  input  wire [(1)-1:0]        s_rfnoc_chdr_tlast,
  input  wire [(1)-1:0]        s_rfnoc_chdr_tvalid,
  output wire [(1)-1:0]        s_rfnoc_chdr_tready,
  // AXIS-CHDR Output Ports (to framework)
  output wire [(1)*CHDR_W-1:0] m_rfnoc_chdr_tdata,
  output wire [(1)-1:0]        m_rfnoc_chdr_tlast,
  output wire [(1)-1:0]        m_rfnoc_chdr_tvalid,
  input  wire [(1)-1:0]        m_rfnoc_chdr_tready,
  // AXIS-Ctrl Input Port (from framework)
  input  wire [31:0]            s_rfnoc_ctrl_tdata,
  input  wire                   s_rfnoc_ctrl_tlast,
  input  wire                   s_rfnoc_ctrl_tvalid,
  output wire                   s_rfnoc_ctrl_tready,
  // AXIS-Ctrl Output Port (to framework)
  output wire [31:0]            m_rfnoc_ctrl_tdata,
  output wire                   m_rfnoc_ctrl_tlast,
  output wire                   m_rfnoc_ctrl_tvalid,
  input  wire                   m_rfnoc_ctrl_tready,
  // RFNoC Backend Interface
  input  wire [511:0]           rfnoc_core_config,
  output wire [511:0]           rfnoc_core_status
);

  //---------------------------------------------------------------------------
  // Signal Declarations
  //---------------------------------------------------------------------------

  // Clocks and Resets
  wire               ctrlport_clk;
  wire               ctrlport_rst;
  wire               axis_data_clk;
  wire               axis_data_rst;
  wire               ce_250_rst;    // ce_250_clk-domain reset from the NoC shell
  wire               ce_250_locked; // MMCM lock status for the 250 MHz clock
  // CtrlPort Master
  wire               m_ctrlport_req_wr;
  wire               m_ctrlport_req_rd;
  wire [19:0]        m_ctrlport_req_addr;
  wire [31:0]        m_ctrlport_req_data;
  reg                m_ctrlport_resp_ack;
  reg  [31:0]        m_ctrlport_resp_data;
  // Data Stream to User Logic: txPayload (now a 32-bit packed item: 16 dibits
  // per word, expanded to the core's 1-dibit-per-cycle stream by
  // axis_dibit_unpack below). See that module + ofdm_tx_sl.yml for the rationale
  // (full payload-bandwidth use and avoidance of the s8 item32 byte scramble).
  wire [32*1-1:0]    m_txPayload_axis_tdata;
  wire [1-1:0]       m_txPayload_axis_tkeep;
  wire               m_txPayload_axis_tlast;
  (* mark_debug = "true" *) wire               m_txPayload_axis_tvalid;
  (* mark_debug = "true" *) wire               m_txPayload_axis_tready;
  wire [63:0]        m_txPayload_axis_ttimestamp;
  wire               m_txPayload_axis_thas_time;
  wire [15:0]        m_txPayload_axis_tlength;
  wire               m_txPayload_axis_teov;
  wire               m_txPayload_axis_teob;
  // Data Stream from User Logic: txData
  (* mark_debug = "true" *) wire [32*1-1:0]    s_txData_axis_tdata;
  wire [1-1:0]       s_txData_axis_tkeep;
  wire               s_txData_axis_tlast;
  (* mark_debug = "true" *) wire               s_txData_axis_tvalid;
  (* mark_debug = "true" *) wire               s_txData_axis_tready;
  wire [63:0]        s_txData_axis_ttimestamp;
  wire               s_txData_axis_thas_time;
  wire [15:0]        s_txData_axis_tlength;
  wire               s_txData_axis_teov;
  wire               s_txData_axis_teob;

  //---------------------------------------------------------------------------
  // 250 MHz Data-Path Clock Generation
  //---------------------------------------------------------------------------
  //
  // The X440 BSP supplies 'ce_clk' at 266.667 MHz, but the downstream radio
  // runs at 250 MHz. Generate a 250 MHz clock from ce_clk with a Xilinx MMCM so
  // the whole data path (the NoC-shell gearboxes and the OFDM core) runs at the
  // radio's rate. This is the HDL that backs the 'ce_250' (direction: out)
  // clock declared in the block YAML.
  //
  //   VCO = 266.667 MHz * 3.75            = 1000.0 MHz   (in 800-1600 range)
  //   ce_250 = VCO / 4.0                  =  250.0 MHz
  //
  // The feedback is buffered through a BUFG (CLKFBOUT -> BUFG -> CLKFBIN), and
  // the MMCM is reset from the ce_clk-domain reset. Downstream logic is held in
  // reset until the MMCM locks (see ofdm_data_rst below), so the OFDM core
  // never sees the 250 MHz clock before it is stable.
  //---------------------------------------------------------------------------

  wire ce_250_clk_unbuf;
  wire ce_250_fb_unbuf;
  wire ce_250_fb;

  MMCME4_ADV #(
    .BANDWIDTH          ("OPTIMIZED"),
    .CLKOUT4_CASCADE    ("FALSE"),
    .COMPENSATION       ("AUTO"),
    .STARTUP_WAIT       ("FALSE"),
    .DIVCLK_DIVIDE      (1),
    .CLKFBOUT_MULT_F    (3.750),
    .CLKFBOUT_PHASE     (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F   (4.000),
    .CLKOUT0_PHASE      (0.000),
    .CLKOUT0_DUTY_CYCLE (0.500),
    .CLKOUT0_USE_FINE_PS ("FALSE"),
    .CLKIN1_PERIOD      (3.750)
  ) ce_250_mmcm_i (
    // Output clocks
    .CLKFBOUT     (ce_250_fb_unbuf),
    .CLKFBOUTB    (),
    .CLKOUT0      (ce_250_clk_unbuf),
    .CLKOUT0B     (),
    .CLKOUT1      (), .CLKOUT1B (),
    .CLKOUT2      (), .CLKOUT2B (),
    .CLKOUT3      (), .CLKOUT3B (),
    .CLKOUT4      (),
    .CLKOUT5      (),
    .CLKOUT6      (),
    // Input clock control
    .CLKFBIN      (ce_250_fb),
    .CLKIN1       (ce_clk),
    .CLKIN2       (1'b0),
    .CLKINSEL     (1'b1),
    // Dynamic reconfiguration port (unused)
    .DADDR        (7'h0),
    .DCLK         (1'b0),
    .DEN          (1'b0),
    .DI           (16'h0),
    .DO           (),
    .DRDY         (),
    .DWE          (1'b0),
    .CDDCDONE     (),
    .CDDCREQ      (1'b0),
    // Dynamic phase shift (unused)
    .PSCLK        (1'b0),
    .PSEN         (1'b0),
    .PSINCDEC     (1'b0),
    .PSDONE       (),
    // Status and control
    .LOCKED       (ce_250_locked),
    .CLKINSTOPPED (),
    .CLKFBSTOPPED (),
    .PWRDWN       (1'b0),
    // Free-running derived clock: do not reset the MMCM on block resets, so
    // ce_250_clk keeps toggling and the NoC shell's ce_250_rst synchronizer
    // continues to work. Downstream logic is instead held off via the
    // ce_250_locked gating below until the MMCM has locked.
    .RST          (1'b0)
  );

  BUFG ce_250_fb_bufg_i (
    .I (ce_250_fb_unbuf),
    .O (ce_250_fb)
  );

  BUFG ce_250_bufg_i (
    .I (ce_250_clk_unbuf),
    .O (ce_250_clk)
  );

  //---------------------------------------------------------------------------
  // NoC Shell
  //---------------------------------------------------------------------------

  noc_shell_ofdm_tx_sl #(
    .CHDR_W              (CHDR_W),
    .THIS_PORTID         (THIS_PORTID),
    .MTU                 (MTU)
  ) noc_shell_ofdm_tx_sl_i (
    //---------------------
    // Framework Interface
    //---------------------

    // Clock Inputs
    .rfnoc_chdr_clk      (rfnoc_chdr_clk),
    .rfnoc_ctrl_clk      (rfnoc_ctrl_clk),
    .ce_clk              (ce_clk),
    .ce_250_clk          (ce_250_clk),
    // Reset Outputs
    .rfnoc_chdr_rst      (),
    .rfnoc_ctrl_rst      (),
    .ce_rst              (),
    .ce_250_rst          (ce_250_rst),
    // CHDR Input Ports  (from framework)
    .s_rfnoc_chdr_tdata  (s_rfnoc_chdr_tdata),
    .s_rfnoc_chdr_tlast  (s_rfnoc_chdr_tlast),
    .s_rfnoc_chdr_tvalid (s_rfnoc_chdr_tvalid),
    .s_rfnoc_chdr_tready (s_rfnoc_chdr_tready),
    // CHDR Output Ports (to framework)
    .m_rfnoc_chdr_tdata  (m_rfnoc_chdr_tdata),
    .m_rfnoc_chdr_tlast  (m_rfnoc_chdr_tlast),
    .m_rfnoc_chdr_tvalid (m_rfnoc_chdr_tvalid),
    .m_rfnoc_chdr_tready (m_rfnoc_chdr_tready),
    // AXIS-Ctrl Input Port (from framework)
    .s_rfnoc_ctrl_tdata  (s_rfnoc_ctrl_tdata),
    .s_rfnoc_ctrl_tlast  (s_rfnoc_ctrl_tlast),
    .s_rfnoc_ctrl_tvalid (s_rfnoc_ctrl_tvalid),
    .s_rfnoc_ctrl_tready (s_rfnoc_ctrl_tready),
    // AXIS-Ctrl Output Port (to framework)
    .m_rfnoc_ctrl_tdata  (m_rfnoc_ctrl_tdata),
    .m_rfnoc_ctrl_tlast  (m_rfnoc_ctrl_tlast),
    .m_rfnoc_ctrl_tvalid (m_rfnoc_ctrl_tvalid),
    .m_rfnoc_ctrl_tready (m_rfnoc_ctrl_tready),

    //---------------------
    // Client Interface
    //---------------------

    // CtrlPort Clock and Reset
    .ctrlport_clk              (ctrlport_clk),
    .ctrlport_rst              (ctrlport_rst),
    // CtrlPort Master
    .m_ctrlport_req_wr         (m_ctrlport_req_wr),
    .m_ctrlport_req_rd         (m_ctrlport_req_rd),
    .m_ctrlport_req_addr       (m_ctrlport_req_addr),
    .m_ctrlport_req_data       (m_ctrlport_req_data),
    .m_ctrlport_resp_ack       (m_ctrlport_resp_ack),
    .m_ctrlport_resp_data      (m_ctrlport_resp_data),

    // AXI-Stream Clock and Reset
    .axis_data_clk (axis_data_clk),
    .axis_data_rst (axis_data_rst),
    // Data Stream to User Logic: txPayload
    .m_txPayload_axis_tdata      (m_txPayload_axis_tdata),
    .m_txPayload_axis_tkeep      (m_txPayload_axis_tkeep),
    .m_txPayload_axis_tlast      (m_txPayload_axis_tlast),
    .m_txPayload_axis_tvalid     (m_txPayload_axis_tvalid),
    .m_txPayload_axis_tready     (m_txPayload_axis_tready),
    .m_txPayload_axis_ttimestamp (m_txPayload_axis_ttimestamp),
    .m_txPayload_axis_thas_time  (m_txPayload_axis_thas_time),
    .m_txPayload_axis_tlength    (m_txPayload_axis_tlength),
    .m_txPayload_axis_teov       (m_txPayload_axis_teov),
    .m_txPayload_axis_teob       (m_txPayload_axis_teob),
    // Data Stream from User Logic: txData
    .s_txData_axis_tdata      (s_txData_axis_tdata),
    .s_txData_axis_tkeep      (s_txData_axis_tkeep),
    .s_txData_axis_tlast      (s_txData_axis_tlast),
    .s_txData_axis_tvalid     (s_txData_axis_tvalid),
    .s_txData_axis_tready     (s_txData_axis_tready),
    .s_txData_axis_ttimestamp (s_txData_axis_ttimestamp),
    .s_txData_axis_thas_time  (s_txData_axis_thas_time),
    .s_txData_axis_tlength    (s_txData_axis_tlength),
    .s_txData_axis_teov       (s_txData_axis_teov),
    .s_txData_axis_teob       (s_txData_axis_teob),

    //---------------------------
    // RFNoC Backend Interface
    //---------------------------
    .rfnoc_core_config   (rfnoc_core_config),
    .rfnoc_core_status   (rfnoc_core_status)
  );

  //---------------------------------------------------------------------------
  // User Logic
  //---------------------------------------------------------------------------

  // < Replace this section with your logic >

  // Data-path reset for the 250 MHz (ce_250) domain. Combine the NoC-shell
  // ce_250 reset with the MMCM lock status so the OFDM core and its
  // axis-domain synchronizer stay in reset until the 250 MHz clock is stable.
  wire axis_data_rst_locked = axis_data_rst | ~ce_250_locked;

  //---------------------------------------------------------------------------
  // Registers
  //---------------------------------------------------------------------------
  //
  // REG_TX_PAYLOAD_READY is a read-only status register. It tells the host
  // whether the txPayload input is ready to accept more data, i.e., whether
  // new input samples may be flushed into the block.
  //
  // m_txPayload_axis_tready lives in the axis_data_clk (ce) domain, while
  // ctrlport registers live in the ctrlport_clk (rfnoc_chdr) domain, so the
  // status bit is passed through a synchronizer.
  //
  //---------------------------------------------------------------------------

  localparam REG_TX_PAYLOAD_READY_ADDR = 0;
  localparam REG_ENABLE_ADDR           = 1;

  wire txPayloadReady;
  wire core_pl_ready;          // OFDM core txPayloadReady (low = >=1 frame buffered)
  reg  reg_enable = 1'b0;
  wire enable_axis_clk;

  synchronizer #(
    .WIDTH            (1),
    .STAGES           (2),
    .INITIAL_VAL      (1'b0),
    .FALSE_PATH_TO_IN (1)
  ) txPayloadReady_sync_i (
    .clk (ctrlport_clk),
    .rst (ctrlport_rst),
    .in  (core_pl_ready),
    .out (txPayloadReady)
  );

  synchronizer #(
    .WIDTH            (1),
    .STAGES           (2),
    .INITIAL_VAL      (1'b0),
    .FALSE_PATH_TO_IN (1)
  ) enable_sync_i (
    .clk (axis_data_clk),
    .rst (axis_data_rst_locked),
    .in  (reg_enable),
    .out (enable_axis_clk)
  );

  always @(posedge ctrlport_clk) begin
    if (ctrlport_rst) begin
      m_ctrlport_resp_ack <= 1'b0;
      reg_enable          <= 1'b0;
    end else begin
      // Default assignment
      m_ctrlport_resp_ack <= 1'b0;

      // Handle read requests
      if (m_ctrlport_req_rd) begin
        case (m_ctrlport_req_addr)
          REG_TX_PAYLOAD_READY_ADDR: begin
            m_ctrlport_resp_ack  <= 1'b1;
            m_ctrlport_resp_data <= {31'b0, txPayloadReady};
          end
          REG_ENABLE_ADDR: begin
            m_ctrlport_resp_ack  <= 1'b1;
            m_ctrlport_resp_data <= {31'b0, reg_enable};
          end
        endcase
      end

      // Handle write requests
      if (m_ctrlport_req_wr) begin
        case (m_ctrlport_req_addr)
          REG_ENABLE_ADDR: begin
            m_ctrlport_resp_ack <= 1'b1;
            reg_enable          <= m_ctrlport_req_data[0];
          end
        endcase
      end
    end
  end

  //---------------------------------------------------------------------------
  // OFDM Transmitter
  //---------------------------------------------------------------------------
  //
  // txPayload carries 2 bits per clock cycle (one QPSK symbol's worth of
  // payload bits). txData carries the modulated I/Q samples, with I in the
  // upper 16 bits and Q in the lower 16 bits (sc16 convention).
  //
  //---------------------------------------------------------------------------

  //---------------------------------------------------------------------------
  // Frame-gated enable (one full frame at a time)
  //---------------------------------------------------------------------------
  //
  // The OFDM_Transmitter core is free-running: once 'enable' is high its
  // Frame_Counter advances every (clk_enable & enable) cycle, and the data
  // resource elements pop the internal payload FIFO. If that FIFO underruns
  // mid-frame (the host cannot sustain a gap-free payload stream), the core
  // modulates stale samples -- the data symbols come out corrupted while the
  // payload-independent sync/reference symbols stay correct (observed on
  // hardware via the loopback ramp/constant tests).
  //
  // Fix: only let the core run while a *full frame* of payload is buffered.
  // FWFT_FIFO drives txPayloadReady low once occupancy reaches one frame
  // (fwft_logic_num_o >= 4560), so ~core_pl_ready means "a complete
  // frame is queued". The core's 'enable' input *holds* the Frame_Counter when
  // low (it does not gate the payload push), so we can pause/resume the framer
  // at frame boundaries without losing buffered payload. We mirror the core's
  // Frame_Counter here -- identical advance condition (clk_enable & enable) and
  // reset -- so we know exactly when its framer sits at a frame boundary, and
  // gate 'enable' to start a frame only when one is fully buffered. Because a
  // frame consumes exactly one frame of payload (20 data symbols x 228 = 4560)
  // and we start with that buffered, the FIFO can never underrun mid-frame.
  //
  // Frame geometry mirrors Frame_Counter: subcarrier counter wraps at 320
  // (RE/symbol = FFT 256 + CP 64), symbol counter wraps at 28 (symbols/frame).
  localparam int RE_PER_SYM    = 320;
  localparam int SYM_PER_FRAME = 28;

  // Payload stream into the core, expanded from the 32-bit packed item by
  // axis_dibit_unpack (instantiated just before the core, below).
  localparam int DIBITS_PER_FRAME = 20*228;            // 4560 data dibits/frame
  (* mark_debug = "true" *) wire [1:0] pl_dibit;        // unpacked dibit to core
  (* mark_debug = "true" *) wire       pl_dvalid;       // unpacked dibit valid

  wire frame_buffered = ~core_pl_ready;                // occupancy >= one frame
  (* mark_debug = "true" *) reg  run_frame = 1'b0;     // gated core enable
  // Same advance condition as the core's Frame_Counter (enb & enable).
  wire core_advance = s_txData_axis_tready & run_frame;

  // ILA: mirror counters give the RE/symbol position so a capture can be lined
  // up with which subcarrier/symbol each consumed payload byte belongs to.
  (* mark_debug = "true" *) reg [8:0] re_mirror  = '0;   // mirrors subcarrier_counter (0..319)
  (* mark_debug = "true" *) reg [7:0] sym_mirror = '0;   // mirrors ofdm_symbol_counter (0..27)
  wire last_re_of_frame = (re_mirror == RE_PER_SYM-1) &&
                          (sym_mirror == SYM_PER_FRAME-1);

  always @(posedge axis_data_clk) begin
    if (axis_data_rst_locked) begin
      re_mirror  <= '0;
      sym_mirror <= '0;
    end else if (core_advance) begin
      if (re_mirror == RE_PER_SYM-1) begin
        re_mirror  <= '0;
        sym_mirror <= (sym_mirror == SYM_PER_FRAME-1) ? '0 : sym_mirror + 1'b1;
      end else begin
        re_mirror <= re_mirror + 1'b1;
      end
    end
  end

  always @(posedge axis_data_clk) begin
    if (axis_data_rst_locked) begin
      run_frame <= 1'b0;
    end else if (!enable_axis_clk) begin
      run_frame <= 1'b0;                       // host disabled: pause
    end else if (!run_frame) begin
      run_frame <= frame_buffered;             // start a frame once one is queued
    end else if (core_advance && last_re_of_frame) begin
      run_frame <= frame_buffered;             // continue only if the next is queued
    end
  end

  // The OFDM_Transmitter core has no output-ready (txData backpressure) port: it
  // free-runs and asserts txValid whenever it has a sample. Its only stall input
  // is clk_enable. The core now runs in the ce_250 domain (250 MHz, generated by
  // the MMCM above) so its nominal sample rate matches the downstream radio's
  // 250 MHz. Even so, the core's output is bursty (gaps for CP insertion / frame
  // structure), so AXI-Stream backpressure is still required: gating clk_enable
  // with s_txData_axis_tready freezes the whole core (output and payload
  // consumption) whenever the output FIFO is full. No combinational loop:
  // s_txData_axis_tready comes from the registered FIFO fullness, not from
  // txValid. 'enable' is the frame-gated run_frame (see above).
  // Expand each 32-bit packed payload word (16 dibits) into the core's
  // 1-dibit-per-cycle stream, dropping the per-frame pad so exactly
  // DIBITS_PER_FRAME dibits reach the core per frame. The unpacker's input-ready
  // backpressures the NoC shell; the core's txPayloadReady (core_pl_ready)
  // backpressures the unpacker.
  axis_dibit_unpack #(
    .DIBITS_PER_FRAME (DIBITS_PER_FRAME)
  ) dibit_unpack_i (
    .clk      (axis_data_clk),
    .rst      (axis_data_rst_locked),
    .s_tdata  (m_txPayload_axis_tdata),
    .s_tvalid (m_txPayload_axis_tvalid),
    .s_tready (m_txPayload_axis_tready),
    .m_dibit  (pl_dibit),
    .m_dvalid (pl_dvalid),
    .m_dready (core_pl_ready)
  );

  OFDM_Transmitter ofdm_transmitter_i (
    .clk            (axis_data_clk),
    .reset          (axis_data_rst_locked),
    .clk_enable     (s_txData_axis_tready),
    .enable         (run_frame),
    .txPayload_0    (pl_dibit[0]),
    .txPayload_1    (pl_dibit[1]),
    // Gate the payload-push with the AXI-Stream handshake. The OFDM core's
    // internal FWFT FIFO pushes on txPayloadValid alone and only honors
    // backpressure via txPayloadReady on its output. Without this gating the
    // FIFO keeps accepting the upstream-held word every cycle while ready is
    // low, pinning its occupancy at the full threshold so txPayloadReady never
    // re-asserts. ANDing valid with ready makes a sample push only when the
    // FIFO has room (proper AXI-Stream flow control).
    .txPayloadValid (pl_dvalid & core_pl_ready),
    .ce_out         (),
    .txData_re      (s_txData_axis_tdata[31:16]),
    .txData_im      (s_txData_axis_tdata[15:0]),
    .txValid        (s_txData_axis_tvalid),
    .txPayloadReady (core_pl_ready)
  );

  // Packetize the OFDM output stream. The output framer (axis_data_to_chdr,
  // SIDEBAND_AT_END) frames a CHDR packet for every input packet, delimited by
  // s_axis_tlast. Since the OFDM transmitter does not emit a packet boundary,
  // count valid output samples and assert tlast every OUT_SPP samples so the
  // framer produces fixed-size CHDR packets.
  //
  // OUT_SPP must stay within the MTU (2**MTU CHDR words; with CHDR_W=64 and a
  // 32-bit item that is 2**MTU * 2 items, i.e. 2048 for MTU=10). Larger packets
  // mean less per-packet header/handshake overhead, so the downstream radio --
  // which consumes a sample every cycle at its fixed rate -- is far less likely
  // to see inter-packet delivery gaps and underflow.
  //
  // OUT_SPP must also DIVIDE the per-frame output length (28 syms x 320 = 8960
  // samples) so a frame ends exactly on a packet boundary. Since this framer only
  // closes a packet on the OUT_SPP-th sample (no frame-boundary tlast), a frame
  // whose length is not a multiple of OUT_SPP leaves a partial final packet
  // stranded in the framer (never sent), starving the consumer. 8960 = 5 * 1792,
  // and 1792 <= 2048 (MTU), so 1792 gives 5 whole packets/frame with the lowest
  // per-packet overhead available. (The old 1024 only worked because the former
  // 240-symbol frame of 76800 = 75 * 1024 happened to divide evenly.)
  localparam int OUT_SPP = 1792;

  reg [$clog2(OUT_SPP)-1:0] out_samp_cnt = '0;
  always @(posedge axis_data_clk) begin
    if (axis_data_rst_locked) begin
      out_samp_cnt <= '0;
    end else if (s_txData_axis_tvalid && s_txData_axis_tready) begin
      out_samp_cnt <= (out_samp_cnt == OUT_SPP-1) ? '0 : out_samp_cnt + 1'b1;
    end
  end

  assign s_txData_axis_tkeep      = 1'b1;
  assign s_txData_axis_tlast      = (out_samp_cnt == OUT_SPP-1);
  assign s_txData_axis_ttimestamp = 64'b0;
  assign s_txData_axis_thas_time  = 1'b0;
  assign s_txData_axis_tlength    = 16'b0;
  assign s_txData_axis_teov       = 1'b0;
  assign s_txData_axis_teob       = 1'b0;

  //---------------------------------------------------------------------------
  // ILA debug core (ce_250 / axis_data_clk domain)
  //---------------------------------------------------------------------------
  // Localizes the data-dependent payload corruption. A constant/periodic payload
  // comes through correct on hardware, but an arbitrary (golden) payload yields
  // random, non-deterministic data subcarriers -- so the payload bits the core
  // consumes diverge from what was fed. Capture those bits in-situ:
  //   probe0 = unpacked dibit into the core, probe1/2 = dibit valid/ready (a
  //   dibit is consumed when both are high). probe3..5 = core output +
  //   backpressure (clk_enable = s_txData_axis_tready). probe6 = run_frame
  //   (gated enable). probe7/8 = RE/symbol position, so each consumed dibit maps
  //   to its subcarrier/symbol. Trigger on run_frame rising (frame start); the
  //   consumed-dibit sequence (probe0 when probe1 && probe2) should equal the
  //   fed payload dibits.
  ila_0 u_ila_txpayload (
    .clk    (axis_data_clk),
    .probe0 (pl_dibit),
    .probe1 (pl_dvalid),
    .probe2 (core_pl_ready),
    .probe3 (s_txData_axis_tdata),
    .probe4 (s_txData_axis_tvalid),
    .probe5 (s_txData_axis_tready),
    .probe6 (run_frame),
    .probe7 (re_mirror),
    .probe8 (sym_mirror)
  );

endmodule // rfnoc_block_ofdm_tx_sl

`default_nettype wire
