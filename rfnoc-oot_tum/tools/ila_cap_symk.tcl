# Capture the ofdm_tx_sl block output (s_txData) into a CSV via the on-chip ILA.
#
# Driven by verify_full_frame.py. Reads three environment variables:
#   SYMK     : frame symbol to trigger on. 0 -> trigger on run_frame rising
#              (frame start; sym_mirror==0 is also the idle value, so it cannot
#              be used as a trigger). >0 -> trigger on sym_mirror == SYMK.
#   ILA_CSV  : output CSV path.
#   ILA_LTX  : probes (.ltx) file for the loaded bitfile.
#
# Stores only valid output beats (s_txData tvalid && tready), so the 8192-deep
# window holds ~25 consecutive OFDM output symbols starting near symbol SYMK.
set K   $::env(SYMK)
set CSV $::env(ILA_CSV)
set ltx $::env(ILA_LTX)

set_param labtools.enable_cs_server 0
open_hw_manager
connect_hw_server -url localhost:3121
current_hw_target [lindex [get_hw_targets] 0]
open_hw_target
set dev [lindex [get_hw_devices xczu28dr_0] 0]
current_hw_device $dev
set_property PROBES.FILE      $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
refresh_hw_device -update_hw_probes true $dev

set ila [lindex [get_hw_ilas -of_objects $dev] 0]
set_property CONTROL.TRIGGER_POSITION 0      $ila
set_property CONTROL.TRIGGER_MODE BASIC_ONLY $ila
set_property CONTROL.CAPTURE_MODE BASIC      $ila
set_property CONTROL.TRIGGER_CONDITION AND   $ila
set_property CONTROL.CAPTURE_CONDITION AND   $ila

set p_dv [get_hw_probes *s_txData_axis_tvalid -of_objects $ila]
set p_dr [get_hw_probes *s_txData_axis_tready -of_objects $ila]
set_property CAPTURE_COMPARE_VALUE eq1'b1 $p_dv
set_property CAPTURE_COMPARE_VALUE eq1'b1 $p_dr

if {$K == 0} {
  set p_run [get_hw_probes *run_frame -of_objects $ila]
  set_property TRIGGER_COMPARE_VALUE eq1'b1 $p_run
} else {
  set p_sym [get_hw_probes *sym_mirror -of_objects $ila]
  set_property TRIGGER_COMPARE_VALUE "eq8'h[format %02x $K]" $p_sym
}

run_hw_ila $ila
puts "ILA_ARMED"
flush stdout
wait_on_hw_ila -timeout 120 $ila
set d [upload_hw_ila_data $ila]
write_hw_ila_data -csv_file -force $CSV $d
puts "WROTE_CSV $CSV"
close_hw_target
exit 0
