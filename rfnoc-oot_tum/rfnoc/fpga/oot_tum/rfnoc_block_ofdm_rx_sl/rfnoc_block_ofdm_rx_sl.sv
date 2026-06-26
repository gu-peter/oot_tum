//
// Copyright 2026 <author>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Module: rfnoc_block_ofdm_rx_sl
//
// Description:
//
//   OFDM receiver RFNoC block. This is the exact counterpart of ofdm_tx_sl: it
//   consumes a stream of sc16 I/Q samples (the captured OFDM waveform, item32
//   layout {I[31:16], Q[15:0]}) on its 'in' port and produces the recovered
//   QPSK payload dibits on its 'out' port, packed 16 dibits per 32-bit item
//   (dibit p at bits [2*p +: 2]) -- the same packed-item convention ofdm_tx_sl
//   consumes, so a TX-input golden vector round-trips back out here.
//
//   The OFDM_Receiver core performs synchronization (timing + optional frequency
//   correction), OFDM demodulation (CP removal + FFT), channel estimation /
//   equalization, and QPSK demodulation. It is driven by two control inputs,
//   'start' and 'freqCorrectionEn', exposed as ctrlport registers. peakInfo
//   (correlation / threshold / timing offset / frequency offset) is latched into
//   debug read registers when the core asserts peakInfoValid.
//
// Parameters:
//
//   THIS_PORTID : Control crossbar port to which this block is connected
//   CHDR_W      : AXIS-CHDR data bus width
//   MTU         : Maximum transmission unit (i.e., maximum packet size in
//                 CHDR words is 2**MTU).
//

`default_nettype none

module rfnoc_block_ofdm_rx_sl #(
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
  // Data Stream to User Logic: in (captured OFDM waveform, sc16 item32)
  wire [32*1-1:0]    m_in_axis_tdata;
  wire [1-1:0]       m_in_axis_tkeep;
  wire               m_in_axis_tlast;
  wire               m_in_axis_tvalid;
  wire               m_in_axis_tready;
  wire [63:0]        m_in_axis_ttimestamp;
  wire               m_in_axis_thas_time;
  wire [15:0]        m_in_axis_tlength;
  wire               m_in_axis_teov;
  wire               m_in_axis_teob;
  // Data Stream from User Logic: out (packed payload dibits)
  wire [32*1-1:0]    s_out_axis_tdata;
  wire [0:0]         s_out_axis_tkeep;
  wire               s_out_axis_tlast;
  wire               s_out_axis_tvalid;
  wire               s_out_axis_tready;
  wire [63:0]        s_out_axis_ttimestamp;
  wire               s_out_axis_thas_time;
  wire [15:0]        s_out_axis_tlength;
  wire               s_out_axis_teov;
  wire               s_out_axis_teob;

  //---------------------------------------------------------------------------
  // 250 MHz Data-Path Clock Generation
  //---------------------------------------------------------------------------
  //
  // Identical to ofdm_tx_sl: the X440 BSP supplies 'ce_clk' at 266.667 MHz, but
  // the radio runs at 250 MHz. Generate a 250 MHz clock from ce_clk with a
  // Xilinx MMCM so the whole data path (the NoC-shell gearboxes and the OFDM
  // receiver core) runs at the radio's rate -- the captured samples arrive at
  // 250 MHz. This is the HDL that backs the 'ce_250' (direction: out) clock
  // declared in the block YAML.
  //
  //   VCO = 266.667 MHz * 3.75            = 1000.0 MHz   (in 800-1600 range)
  //   ce_250 = VCO / 4.0                  =  250.0 MHz
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

  noc_shell_ofdm_rx_sl #(
    .CHDR_W              (CHDR_W),
    .THIS_PORTID         (THIS_PORTID),
    .MTU                 (MTU)
  ) noc_shell_ofdm_rx_sl_i (
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
    // Data Stream to User Logic: in
    .m_in_axis_tdata      (m_in_axis_tdata),
    .m_in_axis_tkeep      (m_in_axis_tkeep),
    .m_in_axis_tlast      (m_in_axis_tlast),
    .m_in_axis_tvalid     (m_in_axis_tvalid),
    .m_in_axis_tready     (m_in_axis_tready),
    .m_in_axis_ttimestamp (m_in_axis_ttimestamp),
    .m_in_axis_thas_time  (m_in_axis_thas_time),
    .m_in_axis_tlength    (m_in_axis_tlength),
    .m_in_axis_teov       (m_in_axis_teov),
    .m_in_axis_teob       (m_in_axis_teob),
    // Data Stream from User Logic: out
    .s_out_axis_tdata      (s_out_axis_tdata),
    .s_out_axis_tkeep      (s_out_axis_tkeep),
    .s_out_axis_tlast      (s_out_axis_tlast),
    .s_out_axis_tvalid     (s_out_axis_tvalid),
    .s_out_axis_tready     (s_out_axis_tready),
    .s_out_axis_ttimestamp (s_out_axis_ttimestamp),
    .s_out_axis_thas_time  (s_out_axis_thas_time),
    .s_out_axis_tlength    (s_out_axis_tlength),
    .s_out_axis_teov       (s_out_axis_teov),
    .s_out_axis_teob       (s_out_axis_teob),

    //---------------------------
    // RFNoC Backend Interface
    //---------------------------
    .rfnoc_core_config   (rfnoc_core_config),
    .rfnoc_core_status   (rfnoc_core_status)
  );

  //---------------------------------------------------------------------------
  // User Logic
  //---------------------------------------------------------------------------

  // Data-path reset for the 250 MHz (ce_250) domain. Combine the NoC-shell
  // ce_250 reset with the MMCM lock status so the OFDM core and its axis-domain
  // logic stay in reset until the 250 MHz clock is stable. Mirrors ofdm_tx_sl.
  wire axis_data_rst_locked = axis_data_rst | ~ce_250_locked;

  //---------------------------------------------------------------------------
  // Control / status registers (ctrlport_clk = rfnoc_chdr domain)
  //---------------------------------------------------------------------------
  //
  //   REG_START            (w/r) : OFDM_Receiver 'start' input (level). Raise to
  //                                arm synchronization. Default 1 (auto-run).
  //   REG_FREQ_CORR_EN     (w/r) : OFDM_Receiver 'freqCorrectionEn' input.
  //                                Default 1 (frequency correction on).
  //   REG_PEAK_CORRELATION (r)   : latched peakInfo_correlation (ufix32_En24)
  //   REG_PEAK_THRESHOLD   (r)   : latched peakInfo_threshold   (ufix32_En24)
  //   REG_PEAK_TOFFSET     (r)   : latched peakInfo_tOffset      (ufix14)
  //   REG_PEAK_FOFFSET     (r)   : latched peakInfo_fOffset      (int32)
  //   REG_PEAK_COUNT       (r)   : number of peakInfoValid strobes seen (debug)
  //
  //---------------------------------------------------------------------------

  localparam REG_START_ADDR            = 20'h00;
  localparam REG_FREQ_CORR_EN_ADDR     = 20'h04;
  localparam REG_PEAK_CORRELATION_ADDR = 20'h08;
  localparam REG_PEAK_THRESHOLD_ADDR   = 20'h0C;
  localparam REG_PEAK_TOFFSET_ADDR     = 20'h10;
  localparam REG_PEAK_FOFFSET_ADDR     = 20'h14;
  localparam REG_PEAK_COUNT_ADDR       = 20'h18;

  reg        reg_start         = 1'b1;  // default: receiver armed
  reg        reg_freq_corr_en  = 1'b1;  // default: frequency correction on

  wire        start_axis;
  wire        freq_corr_en_axis;

  synchronizer #(.WIDTH(1), .STAGES(2), .INITIAL_VAL(1'b1), .FALSE_PATH_TO_IN(1)
  ) start_sync_i (
    .clk (axis_data_clk), .rst (axis_data_rst_locked),
    .in  (reg_start), .out (start_axis)
  );

  synchronizer #(.WIDTH(1), .STAGES(2), .INITIAL_VAL(1'b1), .FALSE_PATH_TO_IN(1)
  ) freq_corr_sync_i (
    .clk (axis_data_clk), .rst (axis_data_rst_locked),
    .in  (reg_freq_corr_en), .out (freq_corr_en_axis)
  );

  // peakInfo debug values, captured in the axis domain and brought across to the
  // ctrlport domain with a toggle handshake (data is stable when the toggle edge
  // arrives, so the multi-bit buses can be sampled directly).
  wire [31:0] peakInfo_correlation;
  wire [31:0] peakInfo_threshold;
  wire [13:0] peakInfo_tOffset;
  wire [31:0] peakInfo_fOffset;
  wire        peakInfoValid;

  reg  [31:0] peak_corr_axis, peak_thr_axis, peak_foff_axis;
  reg  [13:0] peak_toff_axis;
  reg  [31:0] peak_count_axis;
  reg         peak_toggle_axis = 1'b0;

  always @(posedge axis_data_clk) begin
    if (axis_data_rst_locked) begin
      peak_corr_axis   <= '0;
      peak_thr_axis    <= '0;
      peak_foff_axis   <= '0;
      peak_toff_axis   <= '0;
      peak_count_axis  <= '0;
      peak_toggle_axis <= 1'b0;
    end else if (peakInfoValid) begin
      peak_corr_axis   <= peakInfo_correlation;
      peak_thr_axis    <= peakInfo_threshold;
      peak_foff_axis   <= peakInfo_fOffset;
      peak_toff_axis   <= peakInfo_tOffset;
      peak_count_axis  <= peak_count_axis + 1'b1;
      peak_toggle_axis <= ~peak_toggle_axis;
    end
  end

  wire peak_toggle_cp;
  synchronizer #(.WIDTH(1), .STAGES(2), .INITIAL_VAL(1'b0), .FALSE_PATH_TO_IN(1)
  ) peak_toggle_sync_i (
    .clk (ctrlport_clk), .rst (ctrlport_rst),
    .in  (peak_toggle_axis), .out (peak_toggle_cp)
  );

  reg         peak_toggle_cp_d = 1'b0;
  reg  [31:0] peak_corr_cp, peak_thr_cp, peak_foff_cp, peak_count_cp;
  reg  [13:0] peak_toff_cp;

  always @(posedge ctrlport_clk) begin
    if (ctrlport_rst) begin
      peak_toggle_cp_d <= 1'b0;
      peak_corr_cp     <= '0;
      peak_thr_cp      <= '0;
      peak_foff_cp     <= '0;
      peak_toff_cp     <= '0;
      peak_count_cp    <= '0;
    end else begin
      peak_toggle_cp_d <= peak_toggle_cp;
      if (peak_toggle_cp ^ peak_toggle_cp_d) begin
        peak_corr_cp  <= peak_corr_axis;
        peak_thr_cp   <= peak_thr_axis;
        peak_foff_cp  <= peak_foff_axis;
        peak_toff_cp  <= peak_toff_axis;
        peak_count_cp <= peak_count_axis;
      end
    end
  end

  always @(posedge ctrlport_clk) begin
    if (ctrlport_rst) begin
      m_ctrlport_resp_ack <= 1'b0;
      reg_start           <= 1'b1;
      reg_freq_corr_en    <= 1'b1;
    end else begin
      m_ctrlport_resp_ack <= 1'b0;

      if (m_ctrlport_req_rd) begin
        m_ctrlport_resp_ack <= 1'b1;
        case (m_ctrlport_req_addr)
          REG_START_ADDR:            m_ctrlport_resp_data <= {31'b0, reg_start};
          REG_FREQ_CORR_EN_ADDR:     m_ctrlport_resp_data <= {31'b0, reg_freq_corr_en};
          REG_PEAK_CORRELATION_ADDR: m_ctrlport_resp_data <= peak_corr_cp;
          REG_PEAK_THRESHOLD_ADDR:   m_ctrlport_resp_data <= peak_thr_cp;
          REG_PEAK_TOFFSET_ADDR:     m_ctrlport_resp_data <= {18'b0, peak_toff_cp};
          REG_PEAK_FOFFSET_ADDR:     m_ctrlport_resp_data <= peak_foff_cp;
          REG_PEAK_COUNT_ADDR:       m_ctrlport_resp_data <= peak_count_cp;
          default:                   m_ctrlport_resp_data <= 32'b0;
        endcase
      end

      if (m_ctrlport_req_wr) begin
        m_ctrlport_resp_ack <= 1'b1;
        case (m_ctrlport_req_addr)
          REG_START_ADDR:        reg_start        <= m_ctrlport_req_data[0];
          REG_FREQ_CORR_EN_ADDR: reg_freq_corr_en <= m_ctrlport_req_data[0];
        endcase
      end
    end
  end

  //---------------------------------------------------------------------------
  // OFDM Receiver
  //---------------------------------------------------------------------------
  //
  // The captured waveform arrives on the 'in' port as sc16 item32 words
  // ({I[31:16], Q[15:0]}, Q1.15). Drive the core one input sample per enabled
  // cycle: clk_enable gates on input validity (the radio delivers a gap-free
  // 250 MHz stream during a burst; underflow gaps simply freeze the pipeline),
  // and rxValid is then always asserted on an advanced cycle. tready is held
  // high -- the free-running core must not be back-pressured or synchronization
  // breaks, so we never stall the input. Mirrors ofdm_tx_sl's clk_enable gating.
  //---------------------------------------------------------------------------

  wire core_enb = m_in_axis_tvalid;     // advance only when an input sample is present

  assign m_in_axis_tready = 1'b1;       // always accept (core can't be stalled)

  //---------------------------------------------------------------------------
  // Front-end warm-up gate on 'start'
  //---------------------------------------------------------------------------
  // The detection test is |corr|^2 > max(corrThreshold*energy, hardThreshold).
  // Right after reset the 256-tap sync correlator and the Nfft-sample energy
  // moving-average are still full of zeros, so 'energy' ~ 0 and the threshold
  // drops to its hardThreshold floor; meanwhile the partially-filled correlator
  // emits a fixed-point startup transient that clears that floor and latches a
  // FALSE lock on the very first sample (observed as tOffset=1 on both a
  // noise-embedded waveform and on pure noise). Once the windows are full the
  // matched filter rejects noise on its own.
  //
  // Fix: hold the core's 'start' low until WARMUP_SAMPS *valid* samples (not
  // clock cycles -- count core_enb so pipeline freezes don't warm up early)
  // have flowed, i.e. until the front-end windows are full and the threshold is
  // energy-driven. WARMUP_SAMPS must be >= front-end fill depth (Input_Scaling
  // latency + the 256-tap windows) and < the quiet lead-in before a real
  // preamble; 1024 sits comfortably inside that range for the capture geometry
  // here (>=~300 fill, <~2388 to the first preamble after a 2048 pre-roll).
  localparam int WARMUP_SAMPS = 1024;
  reg [$clog2(WARMUP_SAMPS+1)-1:0] warmup_cnt = '0;
  reg                              warmed_up   = 1'b0;
  always @(posedge axis_data_clk) begin
    if (axis_data_rst_locked) begin
      warmup_cnt <= '0;
      warmed_up  <= 1'b0;
    end else if (core_enb && !warmed_up) begin
      if (warmup_cnt == WARMUP_SAMPS-1) warmed_up <= 1'b1;
      else                              warmup_cnt <= warmup_cnt + 1'b1;
    end
  end
  wire start_gated = start_axis & warmed_up;   // host 'start', gated by warm-up

  wire        rxPayload_0;
  wire        rxPayload_1;
  wire        rxPayloadValid;

  OFDM_Receiver ofdm_receiver_i (
    .clk                  (axis_data_clk),
    .reset                (axis_data_rst_locked),
    .clk_enable           (core_enb),
    .rxData_re            (m_in_axis_tdata[31:16]),
    .rxData_im            (m_in_axis_tdata[15:0]),
    .rxValid              (1'b1),
    .start                (start_gated),
    .freqCorrectionEn     (freq_corr_en_axis),
    .ce_out               (),
    .rxPayload_0          (rxPayload_0),
    .rxPayload_1          (rxPayload_1),
    .rxPayloadValid       (rxPayloadValid),
    .peakInfo_correlation (peakInfo_correlation),
    .peakInfo_threshold   (peakInfo_threshold),
    .peakInfo_tOffset     (peakInfo_tOffset),
    .peakInfo_fOffset     (peakInfo_fOffset),
    .peakInfoValid        (peakInfoValid)
  );

  //---------------------------------------------------------------------------
  // Payload packing
  //---------------------------------------------------------------------------
  //
  // Pack the core's 1-dibit-per-cycle payload (dibit = {rxPayload_1,
  // rxPayload_0}) into 32-bit items (16 dibits/word), the exact inverse of
  // ofdm_tx_sl's axis_dibit_unpack. The dibit is valid on (rxPayloadValid &
  // core_enb) -- gate with core_enb so a dibit held across a pipeline freeze is
  // not counted twice.
  //---------------------------------------------------------------------------

  wire [1:0] rx_dibit  = {rxPayload_1, rxPayload_0};
  wire       rx_dvalid = rxPayloadValid & core_enb;
  wire       pack_overflow;

  axis_dibit_pack #(
    .DIBITS_PER_WORD (16)
  ) dibit_pack_i (
    .clk        (axis_data_clk),
    .rst        (axis_data_rst_locked),
    .s_dibit    (rx_dibit),
    .s_dvalid   (rx_dvalid),
    .m_tdata    (s_out_axis_tdata),
    .m_tvalid   (s_out_axis_tvalid),
    .m_tready   (s_out_axis_tready),
    .m_overflow (pack_overflow)
  );

  //---------------------------------------------------------------------------
  // Output packetization
  //---------------------------------------------------------------------------
  //
  // axis_data_to_chdr (SIDEBAND_AT_END) frames a CHDR packet on each s_axis
  // tlast. The OFDM payload has a natural frame boundary every WORDS_PER_FRAME
  // packed words (one OFDM frame = 4560 dibits = exactly 285 words), so assert
  // tlast there: each CHDR packet carries exactly one frame of payload
  // (285 words = 1140 bytes, well under the MTU).
  //---------------------------------------------------------------------------

  localparam int WORDS_PER_FRAME = 285;   // 4560 dibits / 16 dibits-per-word

  reg [$clog2(WORDS_PER_FRAME)-1:0] out_word_cnt = '0;
  always @(posedge axis_data_clk) begin
    if (axis_data_rst_locked) begin
      out_word_cnt <= '0;
    end else if (s_out_axis_tvalid && s_out_axis_tready) begin
      out_word_cnt <= (out_word_cnt == WORDS_PER_FRAME-1) ? '0 : out_word_cnt + 1'b1;
    end
  end

  assign s_out_axis_tkeep      = 1'b1;
  assign s_out_axis_tlast      = (out_word_cnt == WORDS_PER_FRAME-1);
  assign s_out_axis_ttimestamp = 64'b0;
  assign s_out_axis_thas_time  = 1'b0;
  assign s_out_axis_tlength    = 16'b0;
  assign s_out_axis_teov       = 1'b0;
  assign s_out_axis_teob       = 1'b0;

endmodule // rfnoc_block_ofdm_rx_sl

`default_nettype wire
