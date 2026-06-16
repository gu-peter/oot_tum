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
  // CtrlPort Master
  wire               m_ctrlport_req_wr;
  wire               m_ctrlport_req_rd;
  wire [19:0]        m_ctrlport_req_addr;
  wire [31:0]        m_ctrlport_req_data;
  reg                m_ctrlport_resp_ack;
  reg  [31:0]        m_ctrlport_resp_data;
  // Data Stream to User Logic: txPayload
  wire [2*1-1:0]     m_txPayload_axis_tdata;
  wire [1-1:0]       m_txPayload_axis_tkeep;
  wire               m_txPayload_axis_tlast;
  wire               m_txPayload_axis_tvalid;
  wire               m_txPayload_axis_tready;
  wire [63:0]        m_txPayload_axis_ttimestamp;
  wire               m_txPayload_axis_thas_time;
  wire [15:0]        m_txPayload_axis_tlength;
  wire               m_txPayload_axis_teov;
  wire               m_txPayload_axis_teob;
  // Data Stream from User Logic: txData
  wire [32*1-1:0]    s_txData_axis_tdata;
  wire [1-1:0]       s_txData_axis_tkeep;
  wire               s_txData_axis_tlast;
  wire               s_txData_axis_tvalid;
  wire               s_txData_axis_tready;
  wire [63:0]        s_txData_axis_ttimestamp;
  wire               s_txData_axis_thas_time;
  wire [15:0]        s_txData_axis_tlength;
  wire               s_txData_axis_teov;
  wire               s_txData_axis_teob;

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
    // Reset Outputs
    .rfnoc_chdr_rst      (),
    .rfnoc_ctrl_rst      (),
    .ce_rst              (),
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
    .in  (m_txPayload_axis_tready),
    .out (txPayloadReady)
  );

  synchronizer #(
    .WIDTH            (1),
    .STAGES           (2),
    .INITIAL_VAL      (1'b0),
    .FALSE_PATH_TO_IN (1)
  ) enable_sync_i (
    .clk (axis_data_clk),
    .rst (axis_data_rst),
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

  OFDM_Transmitter ofdm_transmitter_i (
    .clk            (axis_data_clk),
    .reset          (axis_data_rst),
    .clk_enable     (1'b1),
    .enable         (enable_axis_clk),
    .txPayload_0    (m_txPayload_axis_tdata[0]),
    .txPayload_1    (m_txPayload_axis_tdata[1]),
    .txPayloadValid (m_txPayload_axis_tvalid),
    .ce_out         (),
    .txData_re      (s_txData_axis_tdata[31:16]),
    .txData_im      (s_txData_axis_tdata[15:0]),
    .txValid        (s_txData_axis_tvalid),
    .txPayloadReady (m_txPayload_axis_tready)
  );

  assign s_txData_axis_tkeep      = 1'b1;
  assign s_txData_axis_tlast      = 1'b0;
  assign s_txData_axis_ttimestamp = 64'b0;
  assign s_txData_axis_thas_time  = 1'b0;
  assign s_txData_axis_tlength    = 16'b0;
  assign s_txData_axis_teov       = 1'b0;
  assign s_txData_axis_teob       = 1'b0;

endmodule // rfnoc_block_ofdm_tx_sl

`default_nettype wire
