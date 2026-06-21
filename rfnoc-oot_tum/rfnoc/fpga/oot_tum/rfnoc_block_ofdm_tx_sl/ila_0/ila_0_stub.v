// Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2021.1_AR76780 (lin64) Build 3247384 Thu Jun 10 19:36:07 MDT 2021
// Date        : Sun Jun 21 02:39:45 2026
// Host        : newcom-upsilon running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/peter/git/oot_tum/rfnoc-oot_tum/rfnoc/fpga/oot_tum/rfnoc_block_ofdm_tx_sl/ila_0/ila_0_stub.v
// Design      : ila_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xczu28dr-ffvg1517-2-e
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "ila,Vivado 2021.1_AR76780" *)
module ila_0(clk, probe0, probe1, probe2, probe3, probe4, probe5, 
  probe6, probe7, probe8)
/* synthesis syn_black_box black_box_pad_pin="clk,probe0[1:0],probe1[0:0],probe2[0:0],probe3[31:0],probe4[0:0],probe5[0:0],probe6[0:0],probe7[8:0],probe8[7:0]" */;
  input clk;
  input [1:0]probe0;
  input [0:0]probe1;
  input [0:0]probe2;
  input [31:0]probe3;
  input [0:0]probe4;
  input [0:0]probe5;
  input [0:0]probe6;
  input [8:0]probe7;
  input [7:0]probe8;
endmodule
