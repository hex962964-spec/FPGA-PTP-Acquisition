# Local constraints for the generated, independently clocked StreamFifoCC
# channels. Gray pointers pass through two destination registers. The payload
# slot is written before the synchronized write pointer can authorize its read,
# and cannot be overwritten before the read pointer returns. The external AXI
# FIFO test and source review are recorded in work/timing_closure_1/axi.
#
# Only Gray source -> first synchronizer D and storage write clock -> registered
# read D receive bounded datapath constraints. Destination read address -> RAM
# -> read register, and synchronizer stage0 -> stage1 remain normally timed.
# No whole-clock groups, no global cuts, no exceptions to synchronizer outputs.
proc af_need {items label} {
 if {[llength $items]==0} {error "AXI FIFO CDC: missing $label"}
 return $items
}
proc af_cells {pattern} {
 return [get_cells -quiet -hier -filter [format {NAME =~ "%s"} $pattern]]
}
proc af_d {cells label} {
 return [af_need [get_pins -quiet -of_objects $cells -filter {REF_PIN_NAME == D}] "$label D pins"]
}
proc af_period {pins label} {
 set clocks [af_need [get_clocks -quiet -of_objects $pins] "$label clocks"]
 set answer 1.0e9
 foreach c $clocks {
  set t [get_property PERIOD $c]
  if {$t<=0} {error "AXI FIFO CDC: invalid clock $c"}
  if {$t<$answer} {set answer $t}
 }
 return $answer
}
proc af_gray {src dst label} {
 set src [af_need $src "$label source Gray"]
 set dst [af_need $dst "$label first stage"]
 if {[llength $src]!=[llength $dst]} {error "AXI FIFO CDC: width mismatch $label"}
 set clocks [af_need [get_pins -quiet -of_objects $src -filter {IS_CLOCK == 1 && IS_LEAF == 1}] "$label source clock pins"]
 set period [af_period $clocks $label]
 set endpoints [af_d $dst $label]
 set_max_delay -datapath_only $period -from $src -to $endpoints
 set_bus_skew $period -from $src -to $endpoints
 puts "AXI_GRAY_CDC=$label WIDTH=[llength $src] MAX_DELAY=$period"
 return [dict create label $label source_bits [llength $src] destination_bits [llength $dst] max_delay $period]
}
set af_fifos [af_need [get_cells -quiet -hier -filter {REF_NAME =~ StreamFifoCC* || ORIG_REF_NAME =~ StreamFifoCC*}] "StreamFifoCC instances"]
if {[llength $af_fifos]!=10} {error "AXI FIFO CDC: expected ten AXI channel FIFOs; inspect changed synthesis hierarchy"}
set af_summary {}
foreach af_fifo $af_fifos {
 set af_name [get_property NAME $af_fifo]
 set af_wgray [af_gray [af_cells "$af_name/pushCC_pushPtrGray_reg*"] [af_cells "$af_name/pushToPopGray_buffercc/buffers_0_reg*"] "$af_name/write-pointer"]
 set af_rgray [af_gray [af_cells "$af_name/popCC_ptrToPush_reg*"] [af_cells "$af_name/popToPushGray_buffercc/buffers_0_reg*"] "$af_name/read-pointer"]
 set af_ram [get_cells -quiet -hier -filter [format {NAME =~ "%s/ram_reg*" && IS_PRIMITIVE == 1 && (REF_NAME =~ RAM* || ORIG_REF_NAME =~ RAM*)} $af_name]]
 # RAM32M's macro WCLK also has IS_CLOCK=1, but it is NOT a timing
 # startpoint. Only physical internal RAMD/RAMS CLK pins have IS_LEAF=1.
 set af_write_clock [get_pins -quiet -of_objects $af_ram -filter {IS_CLOCK == 1 && IS_LEAF == 1}]
 # Small/optimized RAMs may be implemented as flip-flop storage. Keep that
 # equally local, and never replace it with a whole-clock exception.
 set af_memory_ff [get_cells -quiet -hier -filter [format {NAME =~ "%s/ram_reg*" && IS_PRIMITIVE == 1 && REF_NAME =~ FD*} $af_name]]
 if {[llength $af_memory_ff]} {
  set af_write_clock [concat $af_write_clock [get_pins -quiet -of_objects $af_memory_ff -filter {IS_CLOCK == 1 && IS_LEAF == 1}]]
 }
 set af_write_clock [lsort -unique $af_write_clock]
 set af_read_regs [af_cells "$af_name/_zz_ram_port1_reg*"]
 if {![llength $af_write_clock] && ![llength $af_read_regs]} {
  # In SOC8 u_Axi_CDC/bFifo has no remaining payload storage or read FF.
  # There is no payload path to exempt. Still constrain both Gray pointers.
  puts "AXI_RAM_CDC=$af_name NO_STORAGE_OR_READ_REGISTER: no data exception added"
  lappend af_summary [dict create fifo $af_name write_gray $af_wgray read_gray $af_rgray data_status no_storage_no_exception]
  continue
 }
 set af_write_clock [af_need $af_write_clock "$af_name physical storage write clock"]
 set af_read_regs [af_need $af_read_regs "$af_name registered read payload"]
 set af_read_d [af_d $af_read_regs "$af_name read data"]
 set af_write_period [af_period $af_write_clock "$af_name RAM writer"]
 set af_read_period [af_period [get_pins -quiet -of_objects $af_read_regs -filter {IS_CLOCK == 1 && IS_LEAF == 1}] "$af_name RAM reader"]
 set af_bound [expr {min($af_write_period,$af_read_period)}]
 set_max_delay -datapath_only $af_bound -from $af_write_clock -to $af_read_d
 puts "AXI_RAM_CDC=$af_name RAM_CLOCKS=[llength $af_write_clock] READ_BITS=[llength $af_read_d] MAX_DELAY=$af_bound"
 lappend af_summary [dict create fifo $af_name write_gray $af_wgray read_gray $af_rgray data_status bounded_storage_path write_clocks [llength $af_write_clock] read_bits [llength $af_read_d] max_delay $af_bound]
}
puts "AXI_FIFO_CDC_INSTANCES=[llength $af_fifos]"
