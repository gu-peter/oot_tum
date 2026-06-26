#
# Copyright 2026 Ettus Research, a National Instruments Brand
#
# SPDX-License-Identifier: LGPL-3.0-or-later
#
# Timing constraints for the ofdm_rx_sl block.
#
# Peak-diagnostic CDC (axis_data_clk / ce_250  ->  ctrlport_clk / clk200):
#
# In rfnoc_block_ofdm_rx_sl.sv the peak_*_axis data buses are captured in the
# ce_250 domain together with a 1-bit handshake flag (peak_toggle_axis). Only
# the flag is synchronized into the ctrlport (clk200) domain via a 2-FF
# synchronizer; the clk200 side then samples the (already stable) peak_*_axis
# buses into peak_*_cp on the synchronized toggle edge. The data buses are
# therefore a classic MCP/handshake crossing and must NOT be timed as
# full-rate single-cycle paths (default timing of these paths reports a
# spurious ~-50 ps setup violation between the asynchronous 250/200 MHz
# clocks). The flag itself is already false-pathed inside the synchronizer
# (FALSE_PATH_TO_IN=1), so only the data buses need bounding here.
#
# -datapath_only removes the clock-relationship pessimism and just bounds the
# net+logic delay so all bits land within one source period (4 ns @ ce_250),
# well inside the multi-cycle window the data is held stable.
set_max_delay -datapath_only \
  -from [get_cells -hierarchical -filter {NAME =~ "*peak_*_axis_reg[*]"}] \
  -to   [get_cells -hierarchical -filter {NAME =~ "*peak_*_cp_reg[*]"}] \
  4.000
